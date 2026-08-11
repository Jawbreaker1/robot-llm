"""Focused host-owned BLAST local-detour runtime decisions."""

from __future__ import annotations

from .blast_detour_route import (
    blast_detour_action_sweep_is_clear,
    blast_detour_guidance,
    blast_detour_needs_pass_buffer,
    blast_detour_scan_allows_progress,
    blast_detour_scan_sweep_is_clear,
)
from .blast_navigation_policy import settled_no_return_at_pose
from .blast_observation_monitor import (
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    blast_range_state,
)
from .local_detour_route import (
    MERGE_GOAL_AXIS,
    PASS_BEYOND_TARGET,
    ROUTE_COMPLETE,
)
from .physical_navigation_contract import (
    ADVANCE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from .physical_odometry import normalize_heading_mdeg


class BlastDetourRuntimeBlocked(RuntimeError):
    """The bound local route has no admissible next action."""

    code = "blast_detour_blocked"


class BlastNoReturnScanPermitUnavailable(RuntimeError):
    """A host-owned no-return scan lacked its bounded permit."""

    code = "blast_action_start_unverified"


def blast_local_detour_step(
    *,
    route,
    pose,
    distance_mm,
    available_actions,
    pass_scan_complete,
    mission,
    prior_receipt,
    rotation_allowed,
    evidence_correlated,
):
    """Return the next host-owned local-detour action without side effects."""

    no_return_at_pose = settled_no_return_at_pose(
        distance_mm, pose, prior_receipt,
    )
    guidance_actions = list(available_actions)
    if no_return_at_pose:
        guidance_actions.extend((ADVANCE, TURN_LEFT_90, TURN_RIGHT_90))
    guidance = blast_detour_guidance(
        route, pose, tuple(dict.fromkeys(guidance_actions)),
    )
    route = guidance.route
    scan_role = None
    needs_pass_buffer = (
        route.status != ROUTE_COMPLETE
        and not pass_scan_complete
        and blast_detour_needs_pass_buffer(route, pose)
    )
    if route.status == ROUTE_COMPLETE:
        if (
            not mission.heading_aligned(pose)
            or abs(mission.lateral_offset_mm(pose))
            > route.position_tolerance_mm
        ):
            required_action = None
        elif mission.longitudinal_progress_mm(pose) < (
            mission.minimum_forward_progress_mm
        ):
            required_action = ADVANCE
        else:
            required_action = SCAN_FRONT_ARC
            scan_role = "FINAL"
    elif needs_pass_buffer:
        required_action = ADVANCE
    elif (
        route.active_waypoint.kind == MERGE_GOAL_AXIS
        and not pass_scan_complete
    ):
        required_action = SCAN_FRONT_ARC
        scan_role = "PASS"
    else:
        choices = guidance.allowed_motion_actions
        required_action = (
            next(iter(choices))
            if choices is not None and len(choices) == 1
            else None
        )
    pass_advance = (
        required_action == ADVANCE
        and route.status != ROUTE_COMPLETE
        and (
            needs_pass_buffer
            or route.active_waypoint.kind == PASS_BEYOND_TARGET
        )
    )
    side_sign = 1 if route.detour_side == "LEFT_OF_GOAL" else -1
    if pass_advance and (
        abs(normalize_heading_mdeg(
            pose.heading_mdeg - route.goal_heading_mdeg
        )) > route.heading_tolerance_mdeg
        or side_sign * (
            mission.lateral_offset_mm(pose)
            - route.route_lateral_offset_mm
        ) < -route.position_tolerance_mm
    ):
        required_action = None
    if (
        required_action in (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90)
        and not blast_detour_action_sweep_is_clear(
            route, pose, required_action,
        )
    ):
        required_action = None
    no_return_allowed = (
        no_return_at_pose
        and required_action is not None
        and (
            required_action != SCAN_FRONT_ARC
            or blast_detour_scan_sweep_is_clear(route, pose)
        )
    )
    if not evidence_correlated:
        required_action = None
        no_return_allowed = False
    if required_action == ADVANCE and (
        (
            blast_range_state(distance_mm) != RANGE_STATE_MEASURED
            or ADVANCE not in available_actions
        )
        and not no_return_allowed
    ):
        required_action = None
    if required_action in (
        TURN_LEFT_90, TURN_RIGHT_90, SCAN_FRONT_ARC,
    ) and not (rotation_allowed or no_return_allowed):
        required_action = None
    if required_action not in available_actions and not no_return_allowed:
        raise BlastDetourRuntimeBlocked(
            "BLAST has no verified local-detour progress action"
        )
    return route, guidance, required_action, scan_role


def blast_detour_scan_no_return_allows_progress(
    *,
    scan_view,
    role,
    selected_side,
    result_observation,
    observation_settled,
    minimum_forward_clearance_mm,
    route,
    navigation_body_matched,
    heading_correlated,
):
    """Whether exact settled no-return evidence admits route progress."""

    return (
        role == "PASS"
        and observation_settled is True
        and blast_range_state(result_observation.get("distance_mm"))
        == RANGE_STATE_NO_VALID_DISTANCE
        and blast_detour_scan_allows_progress(
            scan_view,
            role=role,
            selected_side=selected_side,
            minimum_clearance_mm=minimum_forward_clearance_mm,
            route=route,
        )
        and navigation_body_matched
        and heading_correlated
    )


def blast_detour_scan_verified(
    *,
    scan_view,
    role,
    selected_side,
    result_observation,
    minimum_forward_clearance_mm,
    route,
    navigation_body_matched,
    heading_correlated,
):
    """Whether measured scan evidence verifies detour progress."""

    distance = result_observation.get("distance_mm")
    return (
        blast_detour_scan_allows_progress(
            scan_view,
            role=role,
            selected_side=selected_side,
            minimum_clearance_mm=minimum_forward_clearance_mm,
            route=route,
        )
        and blast_range_state(distance) == RANGE_STATE_MEASURED
        and float(distance) > minimum_forward_clearance_mm
        and navigation_body_matched
        and heading_correlated
    )


def issue_blast_no_return_scan_permit(
    *,
    controller,
    action,
    host_side_scan,
    host_detour_scan,
    distance_mm,
    side_search_geometry_checked,
    route,
    pose,
    prior_receipt,
):
    """Issue the existing bounded permit for one host-owned NVD scan."""

    action_permit = None
    if (
        action == SCAN_FRONT_ARC
        and (host_side_scan or host_detour_scan)
        and blast_range_state(distance_mm) == RANGE_STATE_NO_VALID_DISTANCE
    ):
        geometry_checked = (
            side_search_geometry_checked
            if host_side_scan
            else blast_detour_scan_sweep_is_clear(route, pose)
        )
        issue = getattr(controller, "issue_no_return_scan_permit", None)
        if geometry_checked and callable(issue):
            action_permit = issue(
                pose=pose.to_dict(),
                prior_receipt=prior_receipt,
                geometry_checked=True,
            )
        if action_permit is None:
            raise BlastNoReturnScanPermitUnavailable(
                "BLAST no-return action permit was unavailable"
            )
    return action_permit


__all__ = (
    "BlastDetourRuntimeBlocked",
    "BlastNoReturnScanPermitUnavailable",
    "blast_detour_scan_no_return_allows_progress",
    "blast_detour_scan_verified",
    "blast_local_detour_step",
    "issue_blast_no_return_scan_permit",
)
