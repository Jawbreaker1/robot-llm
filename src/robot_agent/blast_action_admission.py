"""Fresh, motorless admission for one spoken BLAST action."""

from __future__ import annotations

from .blast_observation_monitor import (
    BlastControllerError,
    RANGE_STATE_NO_VALID_DISTANCE,
    blast_range_state,
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


def fresh_blast_action_observation(
    adapter, *, action, episode_start_heading,
    motion_executor, cancel_requested, episode_error_type,
    encoder_anchor_correlated, navigation_body_matched,
    allow_turn_no_valid_with_bounded_evidence=False,
):
    observation = _read_observation(
        adapter, episode_start_heading, motion_executor,
        cancel_requested,
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
        raise episode_error_type(
            "blast_action_start_unverified",
            "BLAST action lost current motion safety evidence",
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
        step["action"], episode_start_heading, motion_executor,
        context, deadline_ms,
        allow_turn_no_valid_with_bounded_evidence=(
            step["bounded_turn_no_valid_eligible"]
        ),
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
