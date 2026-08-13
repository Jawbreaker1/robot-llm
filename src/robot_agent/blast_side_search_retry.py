"""Motorless same-pose retry for one host-owned BLAST side pulse."""

from __future__ import annotations

from .blast_episode_deadline import (
    SETTLED_OBSERVATION_HEADROOM_MS,
    blast_action_deadline_headroom_ms,
)
from .blast_observation_monitor import (
    RANGE_STATE_NO_VALID_DISTANCE,
    SETTLED_OBSERVATION_COMMAND,
    blast_range_state,
)
from .blast_stationary_evidence import BlastStationaryEvidenceStatus
from .blast_stationary_recovery_flow import (
    collect_episode_stationary_evidence,
)
from .physical_navigation_contract import ADVANCE


def retry_settled_side_search_advance(
    adapter,
    *,
    observation,
    motion_executor,
    episode_start_heading,
    context,
    deadline_ms,
    episode_error_type,
    encoder_anchor_correlated,
    body_matched,
):
    """Return a fresh exact-pose receipt without authorizing motion itself."""

    if (
        blast_range_state(observation["sensors"].get("distance_mm"))
        != RANGE_STATE_NO_VALID_DISTANCE
    ):
        return observation, None, None
    recovered = collect_episode_stationary_evidence(
        adapter, context=context, deadline_ms=deadline_ms,
        motion_executor=motion_executor,
        episode_start_heading=episode_start_heading,
        headroom_ms=SETTLED_OBSERVATION_HEADROOM_MS,
        minimum_safe_distance_mm=adapter.minimum_forward_clearance_mm,
    )
    if recovered.control is not None:
        return observation, None, recovered.control
    if (
        recovered.status not in (
            BlastStationaryEvidenceStatus.MEASURED_SAFE,
            BlastStationaryEvidenceStatus.EXACT_NVD,
        )
        or recovered.observation is None
        or not encoder_anchor_correlated(
            recovered.observation, motion_executor,
        )
        or not body_matched(recovered.observation["sensors"])
    ):
        raise episode_error_type(
            "blast_side_search_blocked",
            "BLAST side travel has no fresh same-pose settled range",
        )
    observation = recovered.observation
    receipt = {
        "action": SETTLED_OBSERVATION_COMMAND,
        "result_observation": dict(recovered.evidence.observation),
        "observation_settled": True,
        "pose": motion_executor.pose.to_dict(),
    }
    return observation, receipt, adapter._control_outcome(
        context, deadline_ms, blast_action_deadline_headroom_ms(ADVANCE),
    )


def admit_side_search_step(
    adapter,
    *,
    side_search_progress,
    navigation_state,
    side_search_waypoint,
    observation,
    available_actions,
    evidence_correlated,
    motion_executor,
    episode_start_heading,
    context,
    deadline_ms,
    history,
    episode_error_type,
    encoder_anchor_correlated,
    body_matched,
    action_admission,
    advance_sweep_clear,
    scan_sweep_clear,
    turn_sweep_clear,
):
    """Apply the bounded same-pose no-return policy to one side step."""

    prior = history[-1] if history else None
    advance_clear = False
    if (
        side_search_progress["phase"] == "OUTBOUND"
        and side_search_progress["required_action"] == ADVANCE
        and evidence_correlated
        and blast_range_state(observation["sensors"].get("distance_mm"))
        == RANGE_STATE_NO_VALID_DISTANCE
    ):
        observation, prior, outcome = retry_settled_side_search_advance(
            adapter,
            observation=observation,
            motion_executor=motion_executor,
            episode_start_heading=episode_start_heading,
            context=context,
            deadline_ms=deadline_ms,
            episode_error_type=episode_error_type,
            encoder_anchor_correlated=encoder_anchor_correlated,
            body_matched=body_matched,
        )
        if outcome is not None:
            return observation, (), outcome
        observation["navigation_intent"] = {
            "selected_detour_side_relative_to_scan": (
                navigation_state.selected_side
            ),
            "side_search_waypoint": dict(side_search_waypoint),
        }
        evidence_correlated = (
            encoder_anchor_correlated(observation, motion_executor)
            and body_matched(observation["sensors"])
        )
        available_actions = adapter._available_actions(observation, history)
        advance_clear = advance_sweep_clear(
            navigation_state.origin_scan_view, motion_executor.pose,
        )
    phase = side_search_progress["phase"]
    scan_clear = phase == "RESCAN" and scan_sweep_clear(
        navigation_state.origin_scan_view, motion_executor.pose,
    )
    turn_clear = phase in ("ORIENT_INWARD", "REORIENT") and turn_sweep_clear(
        navigation_state.origin_scan_view,
        motion_executor.pose,
        side_search_progress["required_action"],
    )
    if (
        phase == "ORIENT_INWARD"
        and side_search_progress["required_action"] in available_actions
        and evidence_correlated
        and turn_clear
    ):
        actions, blocked = (
            (side_search_progress["required_action"],), None
        )
    else:
        actions, blocked = action_admission(
            side_search_progress,
            side_search_waypoint,
            observation["sensors"],
            available_actions,
            evidence_correlated,
            adapter._current_range_allows_rotation(observation),
            current_pose=motion_executor.pose,
            prior_receipt=prior,
            no_return_scan_geometry_checked=scan_clear,
            no_return_advance_geometry_checked=advance_clear,
            no_return_turn_geometry_checked=turn_clear,
        )
    if blocked in (
        "target_reacquisition_blocked", "recovery_rebase_blocked",
    ):
        return observation, (), adapter._outcome(
            blocked, False,
            (
                "BLAST cannot safely return to the recovery origin"
                if blocked == "recovery_rebase_blocked"
                else "BLAST cannot safely reach the next target view"
            ),
        )
    if blocked is not None:
        raise episode_error_type(
            "blast_side_search_blocked",
            "BLAST has no verified side-search progress action",
        )
    return observation, actions, None


__all__ = (
    "admit_side_search_step",
    "retry_settled_side_search_advance",
)
