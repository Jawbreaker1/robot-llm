"""Fresh, motorless admission for one spoken BLAST action."""

from __future__ import annotations

from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_observation_monitor import (
    BlastControllerError,
    RANGE_STATE_NO_VALID_DISTANCE,
    blast_range_state,
)
from .blast_stationary_evidence import BlastStationaryEvidenceStatus
from .blast_stationary_recovery_flow import (
    collect_episode_stationary_evidence,
)
from .physical_navigation_contract import (
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)


def _interrupted() -> BlastControllerError:
    return BlastControllerError(
        "controller_command_interrupted",
        "BLAST observation was cancelled",
        motion_started=False,
    )


def _read_observation(
    adapter, episode_start_heading, motion_executor, cancel_requested,
):
    if cancel_requested():
        raise _interrupted()
    observation = adapter._with_navigation_reference(
        adapter._observation(), episode_start_heading,
    )
    observation["odometry"] = motion_executor.pose.to_dict()
    return observation


def _side_selection_observation(
    adapter, *, selects_detour_side, episode_start_heading,
    motion_executor, context, deadline_ms, episode_error_type,
):
    recovered = collect_episode_stationary_evidence(
        adapter, context=context, deadline_ms=deadline_ms,
        motion_executor=motion_executor,
        episode_start_heading=episode_start_heading,
        minimum_safe_distance_mm=(
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
            .minimum_rotation_clearance_mm()
        ),
    )
    if recovered.control is not None:
        raise _interrupted()
    if (
        recovered.status != BlastStationaryEvidenceStatus.MEASURED_SAFE
        or recovered.observation is None
    ):
        raise episode_error_type(
            (
                "blast_side_search_blocked" if selects_detour_side
                else "blast_action_start_unverified"
            ),
            "BLAST action has no fresh settled range",
        )
    return recovered.observation


def fresh_blast_action_observation(
    adapter, *, action, selects_detour_side, episode_start_heading,
    motion_executor, cancel_requested, episode_error_type,
    encoder_anchor_correlated, navigation_body_matched,
    allow_turn_no_valid_with_bounded_evidence=False,
    context=None, deadline_ms=None,
):
    observation = _read_observation(
        adapter, episode_start_heading, motion_executor,
        cancel_requested,
    )
    should_remeasure = (
        selects_detour_side
        and action in (TURN_LEFT_90, TURN_RIGHT_90)
        and blast_range_state(observation["sensors"].get("distance_mm"))
        == RANGE_STATE_NO_VALID_DISTANCE
        and encoder_anchor_correlated(observation, motion_executor)
        and navigation_body_matched(observation["sensors"])
    )
    if should_remeasure:
        observation = _side_selection_observation(
            adapter, selects_detour_side=selects_detour_side,
            episode_start_heading=episode_start_heading,
            motion_executor=motion_executor, context=context,
            deadline_ms=deadline_ms, episode_error_type=episode_error_type,
        )
    if not encoder_anchor_correlated(observation, motion_executor):
        raise episode_error_type(
            "blast_action_start_unverified",
            "BLAST drive encoders lost their trusted pose anchor",
        )
    exact_nvd_scan = (
        action == SCAN_FRONT_ARC
        and blast_range_state(observation["sensors"].get("distance_mm"))
        == RANGE_STATE_NO_VALID_DISTANCE
        and navigation_body_matched(observation["sensors"])
    )
    exact_nvd_bounded_turn = (
        allow_turn_no_valid_with_bounded_evidence is True
        and action in (TURN_LEFT_90, TURN_RIGHT_90)
        and blast_range_state(observation["sensors"].get("distance_mm"))
        == RANGE_STATE_NO_VALID_DISTANCE
        and navigation_body_matched(observation["sensors"])
    )
    if (
        action != SCAN_FRONT_ARC
        and not exact_nvd_scan
        and not exact_nvd_bounded_turn
        and not adapter._current_observation_allows_action(action, observation)
    ):
        code = (
            "blast_side_search_blocked" if selects_detour_side
            else "blast_action_start_unverified"
        )
        raise episode_error_type(
            code, "BLAST action lost current motion safety evidence",
        )
    if selects_detour_side and not (
        encoder_anchor_correlated(observation, motion_executor)
        and navigation_body_matched(observation["sensors"])
        and adapter._current_range_allows_rotation(observation)
    ):
        raise episode_error_type(
            "blast_side_search_blocked",
            "BLAST side selection lost current motion safety evidence",
        )
    return observation


def admit_blast_spoken_action(
    adapter, speech, step, observation, motion_executor,
    episode_start_heading, context, deadline_ms, progress_revision,
):
    admission = speech.offer(
        step["utterance"], progress_revision=progress_revision,
    )
    if admission is None:
        return observation, None
    control_requested = lambda: adapter._control_outcome(
        context, deadline_ms,
    ) is not None
    speech.await_admission(
        admission, cancel_requested=control_requested,
    )
    outcome = adapter._control_outcome(context, deadline_ms)
    if outcome is not None:
        return observation, outcome
    return adapter._fresh_planner_observation_or_stop(
        step["action"], step["selects_detour_side"],
        episode_start_heading, motion_executor, context, deadline_ms,
    )


def blast_action_phase_flags(
    side_search_progress, detour_guidance, detour_scan_role, action,
):
    return (
        side_search_progress is not None
        and side_search_progress["phase"] == "REORIENT"
        and action in (TURN_LEFT_90, TURN_RIGHT_90),
        side_search_progress is not None
        and side_search_progress["phase"] == "RESCAN"
        and action == SCAN_FRONT_ARC,
        detour_guidance is not None
        and detour_scan_role == "PASS" and action == SCAN_FRONT_ARC,
        detour_guidance is not None
        and detour_scan_role == "FINAL" and action == SCAN_FRONT_ARC,
    )


__all__ = (
    "admit_blast_spoken_action",
    "blast_action_phase_flags",
    "fresh_blast_action_observation",
)
