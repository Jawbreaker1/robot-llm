"""Bounded, chassis-stationary recovery of BLAST range evidence."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import math
import time

from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_observation_monitor import (
    CONTROLLER_ID,
    ROBOT_ID,
    SETTLED_OBSERVATION_COMMAND,
    BlastControllerError,
    blast_range_state,
)
from .blast_scan_observation import (
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
)


MAX_SETTLED_ATTEMPTS = 2
DEFAULT_RECONNECT_POLL_LIMIT = 80
_RECONNECT_ERRORS = frozenset((
    "blast_observation_unavailable",
    "blast_observation_stale",
    "controller_command_timeout",
    "controller_unavailable",
    "stale_controller_command",
))


class BlastStationaryEvidenceStatus(str, Enum):
    """The only four results of stationary evidence recovery."""

    MEASURED_SAFE = "MEASURED_SAFE"
    EXACT_NVD = "EXACT_NVD"
    EXHAUSTED = "EXHAUSTED"
    CONTROLLED = "CONTROLLED"


@dataclass(frozen=True)
class BlastStationaryEvidenceOutcome:
    """Typed recovery result; ``EXACT_NVD`` never implies clearance."""

    status: BlastStationaryEvidenceStatus
    settled_attempts: int
    reconnect_generations: int
    observation: Mapping[str, object] | None = None
    receipt: Mapping[str, object] | None = None
    control: object | None = None
    reason: str | None = None

    def __post_init__(self) -> None:
        if (
            not isinstance(self.status, BlastStationaryEvidenceStatus)
            or type(self.settled_attempts) is not int
            or not 0 <= self.settled_attempts <= MAX_SETTLED_ATTEMPTS
            or type(self.reconnect_generations) is not int
            or not 0 <= self.reconnect_generations <= 1
            or self.observation is not None
            and not isinstance(self.observation, Mapping)
            or self.receipt is not None
            and not isinstance(self.receipt, Mapping)
            or self.reason is not None
            and (
                not isinstance(self.reason, str)
                or not self.reason
                or self.reason != self.reason.strip()
            )
        ):
            raise ValueError("BLAST stationary evidence outcome is invalid")
        if self.status in (
            BlastStationaryEvidenceStatus.MEASURED_SAFE,
            BlastStationaryEvidenceStatus.EXACT_NVD,
        ) and (self.observation is None or self.receipt is None):
            raise ValueError("BLAST stationary evidence is missing")
        if (
            self.status == BlastStationaryEvidenceStatus.CONTROLLED
            and self.control is None
        ):
            raise ValueError("BLAST stationary control outcome is missing")
        if self.observation is not None:
            object.__setattr__(
                self, "observation", deepcopy(dict(self.observation)),
            )
        if self.receipt is not None:
            object.__setattr__(self, "receipt", deepcopy(dict(self.receipt)))


def _finite_number(value) -> float | None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        return None
    return float(value)


def _navigation_body_valid(observation: Mapping[str, object]) -> bool:
    motors = observation.get("motor_angles_deg")
    body = motors.get("body") if isinstance(motors, Mapping) else None
    return (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.range_sensor_extrinsics
        .matches_navigation_body_angle(body)
    )


def _drive_state(observation, expected_drive_angles) -> str:
    """Return MATCHED, MISSING, or DISCONTINUOUS for drive encoders."""

    motors = observation.get("motor_angles_deg")
    if not isinstance(motors, Mapping):
        return "MISSING"
    actual = tuple(_finite_number(motors.get(role)) for role in (
        "left_drive", "right_drive",
    ))
    if any(value is None for value in actual):
        return "MISSING"
    expected = tuple(float(expected_drive_angles[role]) for role in (
        "left_drive", "right_drive",
    ))
    return (
        "MATCHED"
        if all(abs(value - reference) <= 1.0 for value, reference in zip(
            actual, expected,
        ))
        else "DISCONTINUOUS"
    )


def _drive_angles(observation):
    motors = observation.get("motor_angles_deg")
    if not isinstance(motors, Mapping):
        return None
    values = tuple(_finite_number(motors.get(role)) for role in (
        "left_drive", "right_drive",
    ))
    return None if any(value is None for value in values) else values


def _snapshot_identity_valid(snapshot) -> bool:
    return (
        isinstance(snapshot, Mapping)
        and snapshot.get("robot_id") == ROBOT_ID
        and snapshot.get("controller_id") == CONTROLLER_ID
    )


def _commandable_snapshot(snapshot, expected_drive_angles) -> tuple[bool, str]:
    if not _snapshot_identity_valid(snapshot):
        return False, "snapshot_identity_invalid"
    if snapshot.get("state") != "online":
        return False, "controller_offline"
    observation = snapshot.get("observation")
    if not isinstance(observation, Mapping):
        return False, "observation_missing"
    if observation.get("motion_active") is not False:
        return False, "motion_not_idle"
    drive_state = _drive_state(observation, expected_drive_angles)
    if drive_state != "MATCHED":
        return False, (
            "drive_encoder_discontinuity"
            if drive_state == "DISCONTINUOUS"
            else "drive_encoders_missing"
        )
    return True, "stationary_anchor_matched"


def _receipt_classification(
    result,
    snapshot,
    *,
    issued_after_ms,
    issued_generation,
    current_generation,
    expected_drive_angles,
    minimum_safe_distance_mm,
    max_observation_age_ms,
    monotonic_ms,
    body_valid,
):
    if issued_generation != current_generation:
        return None, "settled_snapshot_not_causal"

    result_identity = (
        isinstance(result, Mapping)
        and result.get("robot_id") == ROBOT_ID
        and result.get("controller_id") == CONTROLLER_ID
        and result.get("command") == SETTLED_OBSERVATION_COMMAND
        and result.get("accepted") is True
        and result.get("completed") is True
    )
    result_observation = (
        result.get("observation") if result_identity else None
    )
    snapshot_identity = (
        _snapshot_identity_valid(snapshot)
        and snapshot.get("state") == "online"
    )
    snapshot_observation = (
        snapshot.get("observation") if snapshot_identity else None
    )
    observed_ms = (
        snapshot.get("last_observed_at_monotonic_ms")
        if snapshot_identity else None
    )
    now_ms = monotonic_ms()
    snapshot_causal = (
        isinstance(snapshot_observation, Mapping)
        and type(observed_ms) is int
        and observed_ms > issued_after_ms
        and 0 <= now_ms - observed_ms <= max_observation_age_ms
    )
    result_stationary = (
        isinstance(result_observation, Mapping)
        and result_observation.get("motion_active") is False
        and _drive_state(
            result_observation, expected_drive_angles,
        ) == "MATCHED"
        and body_valid(result_observation)
    )
    snapshot_stationary = (
        snapshot_causal
        and snapshot_observation.get("motion_active") is False
        and _drive_state(
            snapshot_observation, expected_drive_angles,
        ) == "MATCHED"
        and body_valid(snapshot_observation)
    )
    result_distance = (
        result_observation.get("distance_mm")
        if result_stationary else None
    )
    snapshot_distance = (
        snapshot_observation.get("distance_mm")
        if snapshot_stationary else None
    )
    result_state = blast_range_state(result_distance)
    snapshot_state = blast_range_state(snapshot_distance)
    if (
        result_state == RANGE_STATE_MEASURED
        and float(result_distance) <= minimum_safe_distance_mm
        or snapshot_state == RANGE_STATE_MEASURED
        and float(snapshot_distance) <= minimum_safe_distance_mm
    ):
        return "HARD", "measured_clearance_unsafe"
    if (
        isinstance(result_observation, Mapping) and not result_stationary
        or snapshot_causal and not snapshot_stationary
    ):
        return "HARD", "stationary_integrity_unverified"
    if not result_identity or not isinstance(result_observation, Mapping):
        return None, "settled_receipt_incomplete"
    if not snapshot_causal:
        return None, "settled_snapshot_not_causal"
    if _drive_angles(result_observation) != _drive_angles(
        snapshot_observation,
    ):
        return "HARD", "stationary_integrity_unverified"
    if result.get("observation_settled") is not True:
        return None, "settled_receipt_incomplete"
    if result_state == snapshot_state == RANGE_STATE_NO_VALID_DISTANCE:
        return BlastStationaryEvidenceStatus.EXACT_NVD, None
    if result_state == snapshot_state == RANGE_STATE_MEASURED:
        if (
            float(result_distance) > minimum_safe_distance_mm
            and float(snapshot_distance) > minimum_safe_distance_mm
        ):
            return BlastStationaryEvidenceStatus.MEASURED_SAFE, None
        return "HARD", "measured_clearance_unsafe"
    return None, "settled_range_unmatched_or_invalid"


def _default_pause() -> None:
    time.sleep(0.05)


def collect_blast_stationary_evidence(
    *,
    controller,
    expected_drive_angles: Mapping[str, object],
    minimum_safe_distance_mm: float,
    control_outcome: Callable[[], object | None],
    session_generation: Callable[[], int],
    monotonic_ms: Callable[[], int],
    body_valid: Callable[[Mapping[str, object]], bool] = (
        _navigation_body_valid
    ),
    max_observation_age_ms: int = 3_000,
    reconnect_poll_limit: int = DEFAULT_RECONNECT_POLL_LIMIT,
    pause: Callable[[], None] = _default_pause,
) -> BlastStationaryEvidenceOutcome:
    """Collect fresh same-session evidence without authorizing any motion.

    At most two ``observe_settled`` commands and one session-generation change
    are accepted. A motorless command timeout is reconnectable, but its old
    receipt or snapshot is never reused: a new generation and a new complete
    settled receipt are required. An exact Pybricks no-valid-distance receipt
    is returned as ``EXACT_NVD``; the caller must still possess an independent
    frozen-geometry permit before using it for any action.
    """

    expected = (
        expected_drive_angles
        if isinstance(expected_drive_angles, Mapping) else {}
    )
    expected_values = tuple(_finite_number(expected.get(role)) for role in (
        "left_drive", "right_drive",
    ))
    minimum = _finite_number(minimum_safe_distance_mm)
    if not (
        all(value is not None for value in expected_values)
        and minimum is not None
        and minimum >= 0
        and callable(getattr(controller, "snapshot", None))
        and callable(getattr(controller, "command", None))
        and callable(control_outcome)
        and callable(session_generation)
        and callable(monotonic_ms)
        and callable(body_valid)
        and type(max_observation_age_ms) is int
        and 0 <= max_observation_age_ms <= 60_000
        and type(reconnect_poll_limit) is int
        and 1 <= reconnect_poll_limit <= 1_000
        and callable(pause)
    ):
        raise ValueError("BLAST stationary evidence configuration is invalid")

    attempts = 0
    reconnects = 0
    polls = 0
    active_generation = session_generation()
    if type(active_generation) is not int or active_generation < 0:
        raise ValueError("BLAST session generation is invalid")
    require_new_generation = None

    def outcome(status, *, observation=None, receipt=None, control=None,
                reason=None):
        return BlastStationaryEvidenceOutcome(
            status=status,
            settled_attempts=attempts,
            reconnect_generations=reconnects,
            observation=observation,
            receipt=receipt,
            control=control,
            reason=reason,
        )

    def controlled():
        value = control_outcome()
        return (
            None if value is None else outcome(
                BlastStationaryEvidenceStatus.CONTROLLED,
                control=value,
            )
        )

    def note_generation(value):
        nonlocal active_generation, reconnects
        if type(value) is not int or value < 0:
            return "session_generation_invalid"
        if value != active_generation:
            if reconnects >= 1:
                return "reconnect_generation_exhausted"
            reconnects += 1
            active_generation = value
        return None

    while attempts < MAX_SETTLED_ATTEMPTS:
        stopped = controlled()
        if stopped is not None:
            return stopped
        before_generation = session_generation()
        generation_error = note_generation(before_generation)
        snapshot = controller.snapshot()
        after_generation = session_generation()
        generation_error = generation_error or note_generation(
            after_generation,
        )
        if generation_error is not None:
            return outcome(
                BlastStationaryEvidenceStatus.EXHAUSTED,
                reason=generation_error,
            )
        stable_generation = before_generation == after_generation
        commandable, snapshot_reason = _commandable_snapshot(
            snapshot, expected,
        )
        if (
            not stable_generation
            or require_new_generation == active_generation
            or not commandable
        ):
            if snapshot_reason in (
                "drive_encoder_discontinuity", "motion_not_idle",
            ):
                return outcome(
                    BlastStationaryEvidenceStatus.EXHAUSTED,
                    reason=snapshot_reason,
                )
            polls += 1
            if polls >= reconnect_poll_limit:
                return outcome(
                    BlastStationaryEvidenceStatus.EXHAUSTED,
                    reason=snapshot_reason,
                )
            pause()
            continue
        require_new_generation = None
        observed_ms = snapshot.get("last_observed_at_monotonic_ms")
        issued_after_ms = observed_ms if type(observed_ms) is int else -1
        issued_generation = active_generation
        attempts += 1
        try:
            result = controller.command(
                SETTLED_OBSERVATION_COMMAND,
                cancel_requested=lambda: control_outcome() is not None,
            )
        except BlastControllerError as error:
            stopped = controlled()
            if stopped is not None:
                return stopped
            generation_error = note_generation(session_generation())
            if generation_error is not None:
                return outcome(
                    BlastStationaryEvidenceStatus.EXHAUSTED,
                    reason=generation_error,
                )
            if error.code not in _RECONNECT_ERRORS:
                return outcome(
                    BlastStationaryEvidenceStatus.EXHAUSTED,
                    reason=error.code,
                )
            require_new_generation = issued_generation
            if attempts >= MAX_SETTLED_ATTEMPTS:
                return outcome(
                    BlastStationaryEvidenceStatus.EXHAUSTED,
                    reason=error.code,
                )
            continue
        stopped = controlled()
        if stopped is not None:
            return stopped
        result_generation = session_generation()
        generation_error = note_generation(result_generation)
        if generation_error is not None:
            return outcome(
                BlastStationaryEvidenceStatus.EXHAUSTED,
                reason=generation_error,
            )
        snapshot = controller.snapshot()
        snapshot_generation = session_generation()
        generation_error = note_generation(snapshot_generation)
        if generation_error is not None:
            return outcome(
                BlastStationaryEvidenceStatus.EXHAUSTED,
                reason=generation_error,
            )
        classification, reason = _receipt_classification(
            result,
            snapshot,
            issued_after_ms=issued_after_ms,
            issued_generation=issued_generation,
            current_generation=snapshot_generation,
            expected_drive_angles=expected,
            minimum_safe_distance_mm=minimum,
            max_observation_age_ms=max_observation_age_ms,
            monotonic_ms=monotonic_ms,
            body_valid=body_valid,
        )
        if classification in (
            BlastStationaryEvidenceStatus.MEASURED_SAFE,
            BlastStationaryEvidenceStatus.EXACT_NVD,
        ):
            stopped = controlled()
            if stopped is not None:
                return stopped
            return outcome(
                classification,
                observation=snapshot["observation"],
                receipt=result,
            )
        if classification == "HARD":
            return outcome(
                BlastStationaryEvidenceStatus.EXHAUSTED,
                reason=reason,
            )
        if attempts >= MAX_SETTLED_ATTEMPTS:
            return outcome(
                BlastStationaryEvidenceStatus.EXHAUSTED,
                reason=reason,
            )

    return outcome(
        BlastStationaryEvidenceStatus.EXHAUSTED,
        reason="settled_attempts_exhausted",
    )


__all__ = (
    "BlastStationaryEvidenceOutcome",
    "BlastStationaryEvidenceStatus",
    "collect_blast_stationary_evidence",
)
