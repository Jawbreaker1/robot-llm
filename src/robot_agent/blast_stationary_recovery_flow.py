"""Episode-level adaptation of bounded stationary BLAST evidence."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

from .blast_episode_deadline import SETTLED_OBSERVATION_HEADROOM_MS
from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_observation_monitor import (
    SETTLED_OBSERVATION_COMMAND,
    blast_range_state,
)
from .blast_scan_observation import (
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
)
from .blast_stationary_evidence import (
    BlastStationaryEvidenceOutcome,
    BlastStationaryEvidenceStatus,
    collect_blast_stationary_evidence,
)
from .physical_navigation_contract import (
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)


_INITIAL_SOFT_OBSERVATION_ERRORS = frozenset((
    "blast_observation_unavailable",
    "blast_observation_stale",
))


@dataclass(frozen=True)
class BlastEpisodeStationaryRecovery:
    """One bounded evidence attempt translated to episode coordinates."""

    evidence: BlastStationaryEvidenceOutcome
    observation: Mapping[str, object] | None = None
    prior_receipt: Mapping[str, object] | None = None

    @property
    def status(self) -> BlastStationaryEvidenceStatus:
        return self.evidence.status

    @property
    def control(self):
        return self.evidence.control


def _finite_drive_anchor(snapshot):
    observation = (
        snapshot.get("observation") if isinstance(snapshot, Mapping) else None
    )
    angles = (
        observation.get("motor_angles_deg")
        if isinstance(observation, Mapping) else None
    )
    if not isinstance(angles, Mapping):
        return None
    anchor = {}
    for role in ("left_drive", "right_drive"):
        value = angles.get(role)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            return None
        anchor[role] = float(value)
    return anchor


def _stationary_anchor_recoverable(
    snapshot, expected_angles, minimum_safe_distance_mm,
):
    """Reject hard integrity/clearance faults before any new command."""

    observation = (
        snapshot.get("observation") if isinstance(snapshot, Mapping) else None
    )
    if not isinstance(observation, Mapping):
        return False
    angles = observation.get("motor_angles_deg")
    body = angles.get("body") if isinstance(angles, Mapping) else None
    if not (
        observation.get("motion_active") is False
        and isinstance(angles, Mapping)
        and all(
            isinstance(angles.get(role), (int, float))
            and not isinstance(angles.get(role), bool)
            and math.isfinite(float(angles[role]))
            and abs(float(angles[role]) - expected_angles[role]) <= 1.0
            for role in ("left_drive", "right_drive")
        )
        and BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .range_sensor_extrinsics.matches_navigation_body_angle(body)
    ):
        return False
    distance = observation.get("distance_mm")
    state = blast_range_state(distance)
    return (
        state != RANGE_STATE_MEASURED
        or float(distance) > minimum_safe_distance_mm
    )


def _matching_recovered_observation(
    adapter, evidence, expected_angles, minimum_safe_distance_mm,
):
    """Read the latest snapshot without silently changing evidence class."""

    try:
        observation = adapter._observation()
    except Exception:
        return None
    sensors = observation["sensors"]
    angles = sensors.get("motor_angles_deg")
    if not (
        isinstance(angles, Mapping)
        and all(
            isinstance(angles.get(role), (int, float))
            and not isinstance(angles.get(role), bool)
            and math.isfinite(float(angles[role]))
            and abs(float(angles[role]) - expected_angles[role]) <= 1.0
            for role in ("left_drive", "right_drive")
        )
    ):
        return None
    latest_state = blast_range_state(sensors.get("distance_mm"))
    if evidence.status == BlastStationaryEvidenceStatus.MEASURED_SAFE:
        if latest_state != RANGE_STATE_MEASURED:
            return None
        if (
            float(sensors["distance_mm"])
            <= minimum_safe_distance_mm
        ):
            return None
    elif evidence.status == BlastStationaryEvidenceStatus.EXACT_NVD:
        if latest_state != RANGE_STATE_NO_VALID_DISTANCE:
            return None
    else:
        return None
    return observation


def collect_episode_stationary_evidence(
    adapter,
    *,
    context,
    deadline_ms,
    motion_executor=None,
    episode_start_heading=None,
    headroom_ms=SETTLED_OBSERVATION_HEADROOM_MS,
    minimum_safe_distance_mm=None,
) -> BlastEpisodeStationaryRecovery:
    """Collect at most two motorless readings and preserve control priority."""

    generation = getattr(adapter.controller, "runtime_generation", None)
    snapshot = adapter.controller.snapshot()
    expected_angles = (
        motion_executor.expected_start_angles
        if motion_executor is not None
        else _finite_drive_anchor(snapshot)
    )
    if not callable(generation) or expected_angles is None:
        evidence = BlastStationaryEvidenceOutcome(
            status=BlastStationaryEvidenceStatus.EXHAUSTED,
            settled_attempts=0,
            reconnect_generations=0,
            reason=(
                "session_generation_unavailable"
                if not callable(generation)
                else "drive_encoders_missing"
            ),
        )
        return BlastEpisodeStationaryRecovery(evidence)

    minimum = (
        adapter.minimum_forward_clearance_mm
        if minimum_safe_distance_mm is None
        else minimum_safe_distance_mm
    )
    if not _stationary_anchor_recoverable(
        snapshot, expected_angles, minimum,
    ):
        return BlastEpisodeStationaryRecovery(
            BlastStationaryEvidenceOutcome(
                status=BlastStationaryEvidenceStatus.EXHAUSTED,
                settled_attempts=0,
                reconnect_generations=0,
                reason="stationary_anchor_not_recoverable",
            )
        )

    evidence = collect_blast_stationary_evidence(
        controller=adapter.controller,
        expected_drive_angles=expected_angles,
        minimum_safe_distance_mm=minimum,
        control_outcome=lambda: adapter._control_outcome(
            context, deadline_ms, headroom_ms,
        ),
        session_generation=generation,
        monotonic_ms=adapter.monotonic_ms,
        max_observation_age_ms=adapter.max_observation_age_ms,
    )
    if evidence.status == BlastStationaryEvidenceStatus.CONTROLLED:
        return BlastEpisodeStationaryRecovery(evidence)
    observation = _matching_recovered_observation(
        adapter, evidence, expected_angles, minimum,
    )
    if observation is None:
        if evidence.status in (
            BlastStationaryEvidenceStatus.MEASURED_SAFE,
            BlastStationaryEvidenceStatus.EXACT_NVD,
        ):
            evidence = BlastStationaryEvidenceOutcome(
                status=BlastStationaryEvidenceStatus.EXHAUSTED,
                settled_attempts=evidence.settled_attempts,
                reconnect_generations=evidence.reconnect_generations,
                reason="recovered_snapshot_changed",
            )
        return BlastEpisodeStationaryRecovery(evidence)
    if episode_start_heading is not None:
        observation = adapter._with_navigation_reference(
            observation, episode_start_heading,
        )
    if motion_executor is not None:
        observation["odometry"] = motion_executor.pose.to_dict()
        prior_receipt = {
            "action": SETTLED_OBSERVATION_COMMAND,
            "result_observation": dict(evidence.observation),
            "observation_settled": True,
            "pose": motion_executor.pose.to_dict(),
            "stationary_evidence_status": evidence.status.value,
        }
    else:
        prior_receipt = None
    return BlastEpisodeStationaryRecovery(
        evidence=evidence,
        observation=observation,
        prior_receipt=prior_receipt,
    )


def read_episode_observation(
    adapter, *, context, deadline_ms, initial=False,
    minimum_safe_distance_mm=None,
):
    """Read normally, recovering only an initial stale/offline snapshot."""

    try:
        return adapter._observation(), None
    except Exception as error:
        if not initial or getattr(error, "code", None) not in (
            _INITIAL_SOFT_OBSERVATION_ERRORS
        ):
            raise
        recovered = collect_episode_stationary_evidence(
            adapter, context=context, deadline_ms=deadline_ms,
            minimum_safe_distance_mm=minimum_safe_distance_mm,
        )
        if recovered.control is not None:
            return None, recovered.control
        if recovered.observation is None:
            raise error
        return recovered.observation, None


def prepare_blast_iteration_actions(
    adapter, *, observation, history, selected_detour_side,
    navigation_state, latest_scan_view, recovery, motion_executor,
):
    """Apply scan-side and agentic-recovery filters in one reusable step."""

    available_actions = adapter._available_actions(observation, history)
    scan_is_current = adapter._scan_is_current(history)
    scan_allows_turn = (
        adapter._current_scan_allows_quarter_turn(history)
        and latest_scan_view is not None
    )
    if scan_is_current and not scan_allows_turn:
        available_actions = tuple(
            action for action in available_actions
            if action not in (TURN_LEFT_90, TURN_RIGHT_90)
        )
    observation, available_actions = recovery.enrich_planner_iteration(
        adapter, observation, available_actions,
        navigation_state, latest_scan_view,
    )
    return observation, available_actions, scan_allows_turn


def begin_blast_iteration(
    adapter, *, context, deadline_ms, index, history,
    selected_detour_side, navigation_state, latest_scan_view, recovery,
    motion_executor=None, episode_start_heading=None,
    motion_executor_factory=None, minimum_rotation_clearance_mm=None,
):
    """Read, initialize and enrich one BLAST decision iteration."""

    observation, outcome = read_episode_observation(
        adapter, context=context, deadline_ms=deadline_ms,
        initial=index == 0,
        minimum_safe_distance_mm=minimum_rotation_clearance_mm,
    )
    if outcome is not None:
        return None, None, None, None, outcome
    if index == 0:
        episode_start_heading = adapter._heading(observation["sensors"])
        motion_executor = motion_executor_factory(
            controller=adapter.controller,
            initial_observation=observation["sensors"],
        )
    observation = adapter._with_navigation_reference(
        observation, episode_start_heading,
    )
    observation["odometry"] = motion_executor.pose.to_dict()
    observation, available_actions, scan_allows_turn = (
        prepare_blast_iteration_actions(
            adapter, observation=observation, history=history,
            selected_detour_side=selected_detour_side,
            navigation_state=navigation_state,
            latest_scan_view=latest_scan_view, recovery=recovery,
            motion_executor=motion_executor,
        )
    )
    return (
        observation, available_actions, scan_allows_turn,
        (motion_executor, episode_start_heading), None,
    )


def recover_planner_soft_no_action(
    adapter, *, observation, context, deadline_ms, motion_executor,
    episode_start_heading,
):
    """Retry missing/invalid/NVD planner evidence, never measured blocks."""

    if blast_range_state(
        observation["sensors"].get("distance_mm")
    ) == RANGE_STATE_MEASURED:
        return BlastEpisodeStationaryRecovery(
            BlastStationaryEvidenceOutcome(
                status=BlastStationaryEvidenceStatus.EXHAUSTED,
                settled_attempts=0,
                reconnect_generations=0,
                reason="measured_evidence_not_soft",
            )
        )
    return collect_episode_stationary_evidence(
        adapter,
        context=context,
        deadline_ms=deadline_ms,
        motion_executor=motion_executor,
        episode_start_heading=episode_start_heading,
    )


def recover_planner_iteration_actions(
    adapter, *, observation, available_actions, completion_allowed,
    context, deadline_ms, motion_executor, episode_start_heading,
    history, selected_detour_side, navigation_state, latest_scan_view,
    recovery,
):
    """Refresh one planner-owned no-action iteration when evidence is soft."""

    prior_motion = history[-1].get("motion") if history else None
    if (
        available_actions
        or completion_allowed
        or isinstance(prior_motion, Mapping)
        and prior_motion.get("command_completed") is not True
    ):
        return observation, available_actions, None, None
    recovered = recover_planner_soft_no_action(
        adapter, observation=observation, context=context,
        deadline_ms=deadline_ms, motion_executor=motion_executor,
        episode_start_heading=episode_start_heading,
    )
    if recovered.control is not None:
        return observation, available_actions, None, recovered.control
    if recovered.observation is None:
        return observation, available_actions, None, None
    observation, available_actions, scan_allows_turn = (
        prepare_blast_iteration_actions(
            adapter, observation=recovered.observation, history=history,
            selected_detour_side=selected_detour_side,
            navigation_state=navigation_state,
            latest_scan_view=latest_scan_view, recovery=recovery,
            motion_executor=motion_executor,
        )
    )
    return observation, available_actions, scan_allows_turn, None


def recover_scan_start_observation(
    adapter, *, context, deadline_ms, motion_executor,
    episode_start_heading, allow_no_return, minimum_safe_distance_mm,
):
    """Recover one refused scan start without granting unbounded NVD use."""

    recovered = collect_episode_stationary_evidence(
        adapter,
        context=context,
        deadline_ms=deadline_ms,
        motion_executor=motion_executor,
        episode_start_heading=episode_start_heading,
        minimum_safe_distance_mm=minimum_safe_distance_mm,
    )
    if recovered.control is not None or recovered.observation is None:
        return None, recovered.control
    if (
        recovered.status == BlastStationaryEvidenceStatus.MEASURED_SAFE
        and adapter._current_observation_allows_action(
            SCAN_FRONT_ARC, recovered.observation,
        )
    ):
        return recovered.observation, None
    if (
        recovered.status == BlastStationaryEvidenceStatus.EXACT_NVD
        and allow_no_return
    ):
        return recovered.observation, None
    return None, None


__all__ = (
    "BlastEpisodeStationaryRecovery",
    "begin_blast_iteration",
    "collect_episode_stationary_evidence",
    "prepare_blast_iteration_actions",
    "read_episode_observation",
    "recover_planner_soft_no_action",
    "recover_planner_iteration_actions",
    "recover_scan_start_observation",
)
