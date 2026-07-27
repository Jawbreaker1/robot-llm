"""Strict contracts for simulator-first autonomous navigation.

The language model may propose a semantic next segment.  Host code stamps
authority, source, receive time and TTL; none of those fields are controlled
by the model.  Wheel commands are produced only by the motion supervisor.
"""

from dataclasses import dataclass
from collections import deque
import json
import threading
from typing import Mapping, Optional, Union


NAVIGATION_PROPOSAL_SCHEMA = "robot-navigation-proposal/v1"
MAX_NAVIGATION_PROPOSAL_BYTES = 16 * 1024


class NavigationContractError(ValueError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def identifier(name: str, value: str, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise NavigationContractError(
            "invalid_identifier",
            "{} is invalid".format(name),
        )
    return value


def integer(name: str, value: int, minimum: int, maximum: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise NavigationContractError(
            "invalid_integer",
            "{} is invalid".format(name),
        )
    return value


def boolean(name: str, value: bool) -> bool:
    if type(value) is not bool:
        raise NavigationContractError(
            "invalid_boolean",
            "{} is invalid".format(name),
        )
    return value


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


@dataclass(frozen=True)
class WaypointGoal:
    goal_id: str
    goal_epoch: int
    plan_revision: int
    target_x_mm: int
    target_y_mm: int
    tolerance_mm: int = 35

    def __post_init__(self) -> None:
        identifier("goal_id", self.goal_id)
        integer("goal_epoch", self.goal_epoch, 1, 2**63 - 1)
        integer("plan_revision", self.plan_revision, 1, 2**63 - 1)
        integer("target_x_mm", self.target_x_mm, -1_000_000, 1_000_000)
        integer("target_y_mm", self.target_y_mm, -1_000_000, 1_000_000)
        integer("tolerance_mm", self.tolerance_mm, 1, 10_000)


@dataclass(frozen=True)
class AdvanceSegment:
    distance_mm: int
    speed_mm_s: int

    def __post_init__(self) -> None:
        # Reverse is intentionally absent in the first slice: EV3RSTORM has
        # no positive rear-clearance evidence.
        integer("distance_mm", self.distance_mm, 1, 10_000)
        integer("speed_mm_s", self.speed_mm_s, 1, 2_000)

    def semantic_key(self):
        return ("ADVANCE", self.distance_mm, self.speed_mm_s)


@dataclass(frozen=True)
class TurnSegment:
    angle_mdeg: int
    angular_speed_mdeg_s: int

    def __post_init__(self) -> None:
        integer("angle_mdeg", self.angle_mdeg, -360_000, 360_000)
        if self.angle_mdeg == 0:
            raise NavigationContractError(
                "zero_turn",
                "TURN angle must be non-zero",
            )
        integer(
            "angular_speed_mdeg_s",
            self.angular_speed_mdeg_s,
            1,
            720_000,
        )

    def semantic_key(self):
        return (
            "TURN",
            self.angle_mdeg,
            self.angular_speed_mdeg_s,
        )


NavigationSegment = Union[AdvanceSegment, TurnSegment]


@dataclass(frozen=True)
class PlannerProposal:
    proposal_id: str
    goal_id: str
    goal_epoch: int
    plan_revision: int
    based_on_state_version: int
    based_on_world_model_version: int
    decision: str
    confidence_milli: int
    segment: Optional[NavigationSegment] = None
    reason_code: Optional[str] = None

    def __post_init__(self) -> None:
        identifier("proposal_id", self.proposal_id)
        identifier("goal_id", self.goal_id)
        integer("goal_epoch", self.goal_epoch, 1, 2**63 - 1)
        integer("plan_revision", self.plan_revision, 1, 2**63 - 1)
        integer(
            "based_on_state_version",
            self.based_on_state_version,
            1,
            2**63 - 1,
        )
        integer(
            "based_on_world_model_version",
            self.based_on_world_model_version,
            1,
            2**63 - 1,
        )
        integer("confidence_milli", self.confidence_milli, 0, 1_000)
        if self.decision == "NEXT_SEGMENT":
            if not isinstance(self.segment, (AdvanceSegment, TurnSegment)):
                raise NavigationContractError(
                    "missing_segment",
                    "NEXT_SEGMENT requires a navigation segment",
                )
            if self.reason_code is not None:
                raise NavigationContractError(
                    "unexpected_reason",
                    "NEXT_SEGMENT cannot contain reason_code",
                )
        elif self.decision in ("HOLD", "ABORT"):
            if self.segment is not None:
                raise NavigationContractError(
                    "unexpected_segment",
                    "{} cannot contain a segment".format(self.decision),
                )
            identifier("reason_code", self.reason_code or "", 64)
        else:
            raise NavigationContractError(
                "invalid_decision",
                "Decision must be NEXT_SEGMENT, HOLD or ABORT",
            )

    def semantic_key(self):
        if self.decision == "NEXT_SEGMENT":
            return (self.decision,) + self.segment.semantic_key()
        return (self.decision, self.reason_code)


def decode_navigation_proposal(raw: bytes) -> PlannerProposal:
    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > MAX_NAVIGATION_PROPOSAL_BYTES
    ):
        raise NavigationContractError(
            "invalid_proposal_body",
            "Navigation proposal body is invalid",
        )
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, TypeError, ValueError):
        raise NavigationContractError(
            "invalid_proposal_json",
            "Planner returned invalid JSON",
        ) from None
    if not isinstance(value, dict):
        raise NavigationContractError(
            "invalid_proposal_shape",
            "Navigation proposal must be an object",
        )

    common = {
        "schema",
        "proposal_id",
        "goal_id",
        "goal_epoch",
        "plan_revision",
        "based_on_state_version",
        "based_on_world_model_version",
        "decision",
        "confidence_milli",
    }
    if value.get("schema") != NAVIGATION_PROPOSAL_SCHEMA:
        raise NavigationContractError(
            "invalid_proposal_schema",
            "Navigation proposal schema is not supported",
        )

    decision = value.get("decision")
    segment = None
    reason_code = None
    if decision == "NEXT_SEGMENT":
        if set(value) != common | {"segment"}:
            raise NavigationContractError(
                "invalid_proposal_fields",
                "NEXT_SEGMENT fields are invalid",
            )
        action = value["segment"]
        if not isinstance(action, dict):
            raise NavigationContractError(
                "invalid_segment",
                "Navigation segment must be an object",
            )
        if action.get("type") == "ADVANCE":
            if set(action) != {"type", "distance_mm", "speed_mm_s"}:
                raise NavigationContractError(
                    "invalid_segment_fields",
                    "ADVANCE fields are invalid",
                )
            segment = AdvanceSegment(
                distance_mm=integer(
                    "distance_mm",
                    action["distance_mm"],
                    1,
                    10_000,
                ),
                speed_mm_s=integer(
                    "speed_mm_s",
                    action["speed_mm_s"],
                    1,
                    2_000,
                ),
            )
        elif action.get("type") == "TURN":
            if set(action) != {
                "type",
                "angle_mdeg",
                "angular_speed_mdeg_s",
            }:
                raise NavigationContractError(
                    "invalid_segment_fields",
                    "TURN fields are invalid",
                )
            segment = TurnSegment(
                angle_mdeg=integer(
                    "angle_mdeg",
                    action["angle_mdeg"],
                    -360_000,
                    360_000,
                ),
                angular_speed_mdeg_s=integer(
                    "angular_speed_mdeg_s",
                    action["angular_speed_mdeg_s"],
                    1,
                    720_000,
                ),
            )
        else:
            raise NavigationContractError(
                "invalid_segment_type",
                "Segment type must be ADVANCE or TURN",
            )
    elif decision in ("HOLD", "ABORT"):
        if set(value) != common | {"reason_code"}:
            raise NavigationContractError(
                "invalid_proposal_fields",
                "{} fields are invalid".format(decision),
            )
        reason_code = identifier("reason_code", value["reason_code"], 64)
    else:
        raise NavigationContractError(
            "invalid_decision",
            "Decision must be NEXT_SEGMENT, HOLD or ABORT",
        )

    return PlannerProposal(
        proposal_id=identifier("proposal_id", value["proposal_id"]),
        goal_id=identifier("goal_id", value["goal_id"]),
        goal_epoch=integer(
            "goal_epoch",
            value["goal_epoch"],
            1,
            2**63 - 1,
        ),
        plan_revision=integer(
            "plan_revision",
            value["plan_revision"],
            1,
            2**63 - 1,
        ),
        based_on_state_version=integer(
            "based_on_state_version",
            value["based_on_state_version"],
            1,
            2**63 - 1,
        ),
        based_on_world_model_version=integer(
            "based_on_world_model_version",
            value["based_on_world_model_version"],
            1,
            2**63 - 1,
        ),
        decision=decision,
        confidence_milli=integer(
            "confidence_milli",
            value["confidence_milli"],
            0,
            1_000,
        ),
        segment=segment,
        reason_code=reason_code,
    )


@dataclass(frozen=True)
class StampedProposal:
    proposal: PlannerProposal
    source_id: str
    source_sequence: int
    received_at_ms: int
    valid_until_ms: int
    authority_rank: int
    priority: int

    def __post_init__(self) -> None:
        if not isinstance(self.proposal, PlannerProposal):
            raise NavigationContractError(
                "invalid_proposal",
                "Stamped proposal requires PlannerProposal",
            )
        identifier("source_id", self.source_id)
        integer("source_sequence", self.source_sequence, 1, 2**63 - 1)
        integer("received_at_ms", self.received_at_ms, 0, 2**63 - 1)
        integer(
            "valid_until_ms",
            self.valid_until_ms,
            self.received_at_ms + 1,
            2**63 - 1,
        )
        integer("authority_rank", self.authority_rank, 0, 10_000)
        integer("priority", self.priority, 0, 10_000)


@dataclass(frozen=True)
class DriveCalibrationProfile:
    calibration_id: str
    status: str
    surface: str
    left_motor_sign: int
    right_motor_sign: int
    encoder_mdeg_per_mm: Optional[int]
    encoder_mdeg_per_body_degree: Optional[int]
    max_wheel_speed_dps: int
    max_pulse_ms: int
    physical_stop_latency_verified: bool = False

    def __post_init__(self) -> None:
        identifier("calibration_id", self.calibration_id)
        if self.status not in (
            "simulation_only",
            "provisional",
            "physical_verified",
        ):
            raise NavigationContractError(
                "invalid_calibration_status",
                "Calibration status is invalid",
            )
        identifier("surface", self.surface, 128)
        if self.left_motor_sign not in (-1, 1):
            raise NavigationContractError(
                "invalid_motor_sign",
                "left_motor_sign must be -1 or 1",
            )
        if self.right_motor_sign not in (-1, 1):
            raise NavigationContractError(
                "invalid_motor_sign",
                "right_motor_sign must be -1 or 1",
            )
        if self.encoder_mdeg_per_mm is not None:
            integer(
                "encoder_mdeg_per_mm",
                self.encoder_mdeg_per_mm,
                1,
                1_000_000,
            )
        if self.encoder_mdeg_per_body_degree is not None:
            integer(
                "encoder_mdeg_per_body_degree",
                self.encoder_mdeg_per_body_degree,
                1,
                10_000_000,
            )
        integer(
            "max_wheel_speed_dps",
            self.max_wheel_speed_dps,
            1,
            100_000,
        )
        integer("max_pulse_ms", self.max_pulse_ms, 1, 5_000)
        boolean(
            "physical_stop_latency_verified",
            self.physical_stop_latency_verified,
        )

    @property
    def ready_for_physical_motion(self) -> bool:
        return (
            self.status == "physical_verified"
            and self.encoder_mdeg_per_mm is not None
            and self.encoder_mdeg_per_body_degree is not None
            and self.physical_stop_latency_verified
        )

    def require_complete_geometry(self) -> None:
        if (
            self.encoder_mdeg_per_mm is None
            or self.encoder_mdeg_per_body_degree is None
        ):
            raise NavigationContractError(
                "incomplete_calibration",
                "Linear and turn calibration are both required",
            )


@dataclass(frozen=True)
class DrivePulse:
    decision_id: str
    arbiter_id: str
    robot_id: str
    controller_instance_id: str
    goal_id: str
    goal_epoch: int
    plan_revision: int
    based_on_state_version: int
    based_on_world_model_version: int
    kind: str
    left_speed_dps: int
    right_speed_dps: int
    duration_ms: int
    reason_code: str
    proposal_id: Optional[str] = None

    def __post_init__(self) -> None:
        identifier("decision_id", self.decision_id)
        identifier("arbiter_id", self.arbiter_id)
        identifier("robot_id", self.robot_id)
        identifier("controller_instance_id", self.controller_instance_id)
        identifier("goal_id", self.goal_id)
        integer("goal_epoch", self.goal_epoch, 1, 2**63 - 1)
        integer("plan_revision", self.plan_revision, 1, 2**63 - 1)
        integer(
            "based_on_state_version",
            self.based_on_state_version,
            1,
            2**63 - 1,
        )
        integer(
            "based_on_world_model_version",
            self.based_on_world_model_version,
            1,
            2**63 - 1,
        )
        identifier("reason_code", self.reason_code, 96)
        if self.proposal_id is not None:
            identifier("proposal_id", self.proposal_id)
        if self.kind == "DRIVE":
            integer(
                "left_speed_dps",
                self.left_speed_dps,
                -100_000,
                100_000,
            )
            integer(
                "right_speed_dps",
                self.right_speed_dps,
                -100_000,
                100_000,
            )
            if self.left_speed_dps == self.right_speed_dps == 0:
                raise NavigationContractError(
                    "zero_drive",
                    "DRIVE must move at least one wheel",
                )
            integer("duration_ms", self.duration_ms, 1, 5_000)
        elif self.kind == "STOP":
            if (
                self.left_speed_dps != 0
                or self.right_speed_dps != 0
                or self.duration_ms != 0
            ):
                raise NavigationContractError(
                    "invalid_stop",
                    "STOP must have zero wheel speeds and duration",
                )
        else:
            raise NavigationContractError(
                "invalid_pulse_kind",
                "Drive pulse kind must be DRIVE or STOP",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "decision_id": self.decision_id,
            "arbiter_id": self.arbiter_id,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "goal_id": self.goal_id,
            "goal_epoch": self.goal_epoch,
            "plan_revision": self.plan_revision,
            "based_on_state_version": self.based_on_state_version,
            "based_on_world_model_version": (
                self.based_on_world_model_version
            ),
            "kind": self.kind,
            "left_speed_dps": self.left_speed_dps,
            "right_speed_dps": self.right_speed_dps,
            "duration_ms": self.duration_ms,
            "reason_code": self.reason_code,
            "proposal_id": self.proposal_id,
        }


class MotionAuthority:
    """Private one-shot registry between a supervisor and one motion bus.

    The capability object never travels inside ``DrivePulse`` and therefore
    cannot leak through traces or the plant's applied-pulse history.
    """

    def __init__(
        self,
        max_pending: int = 64,
        replay_window: int = 4_096,
    ):
        integer("max_pending", max_pending, 1, 10_000)
        integer(
            "replay_window",
            replay_window,
            max_pending,
            100_000,
        )
        self._max_pending = max_pending
        self._replay_window = replay_window
        self._pending = {}
        self._consumed = set()
        self._consumed_order = deque()
        self._lock = threading.Lock()

    def _remember_consumed_locked(self, decision_id: str) -> None:
        if len(self._consumed_order) >= self._replay_window:
            expired_id = self._consumed_order.popleft()
            self._consumed.discard(expired_id)
        self._consumed.add(decision_id)
        self._consumed_order.append(decision_id)

    def authorize(self, pulse: DrivePulse) -> None:
        if not isinstance(pulse, DrivePulse):
            raise NavigationContractError(
                "invalid_drive_pulse",
                "Motion authority only accepts DrivePulse",
            )
        with self._lock:
            if (
                pulse.decision_id in self._pending
                or pulse.decision_id in self._consumed
            ):
                raise NavigationContractError(
                    "duplicate_motion_authorization",
                    "Motion decision ID has already been authorized",
                )
            if pulse.kind == "STOP":
                pending_ids = tuple(self._pending)
                self._pending.clear()
                for decision_id in pending_ids:
                    self._remember_consumed_locked(decision_id)
            if len(self._pending) >= self._max_pending:
                raise NavigationContractError(
                    "motion_authority_full",
                    "Pending motion authorization capacity exhausted",
                )
            self._pending[pulse.decision_id] = pulse

    def consume(self, pulse: DrivePulse) -> None:
        if not isinstance(pulse, DrivePulse):
            raise NavigationContractError(
                "invalid_drive_pulse",
                "Motion authority only accepts DrivePulse",
            )
        with self._lock:
            expected = self._pending.get(pulse.decision_id)
            if expected is None or expected != pulse:
                raise NavigationContractError(
                    "unauthorized_motion_owner",
                    "Drive pulse has no matching one-shot authorization",
                )
            del self._pending[pulse.decision_id]
            self._remember_consumed_locked(pulse.decision_id)

    def cancel(self, pulse: DrivePulse) -> None:
        """Atomically revoke an authorized pulse that will not dispatch."""

        if not isinstance(pulse, DrivePulse):
            raise NavigationContractError(
                "invalid_drive_pulse",
                "Motion authority only accepts DrivePulse",
            )
        with self._lock:
            expected = self._pending.get(pulse.decision_id)
            if expected is None or expected != pulse:
                raise NavigationContractError(
                    "unknown_motion_authorization",
                    "Drive pulse has no matching pending authorization",
                )
            del self._pending[pulse.decision_id]
            self._remember_consumed_locked(pulse.decision_id)
