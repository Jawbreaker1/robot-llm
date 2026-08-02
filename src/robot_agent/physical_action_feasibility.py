"""Deterministic action feasibility published before model planning."""

from typing import Mapping

from .active_ir_scan_contract import ActiveIrScanCalibration
from .maneuver_commitment import (
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
)
from .physical_navigation_contract import (
    ACTIONS,
    MOTION_ACTIONS,
    OBSERVE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    motion_budget_allows,
)
from .physical_odometry import OdometryCalibration, PhysicalPose
from .provisional_hazard_map import ProvisionalHazardMap


DETOUR_TURN_ACTIONS = frozenset((TURN_LEFT_90, TURN_RIGHT_90))
DETOUR_SUCCESS_FACT_KEYS = frozenset((
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
))
def _active_goal_conflict_ids(navigation: Mapping[str, object]):
    return frozenset(
        conflict["hypothesis_id"]
        for conflict in navigation["goal_geometry"]["conflicts"]
        if conflict["active_for_collision"] is True
    )


def detour_scan_required_target_ids(
    navigation: Mapping[str, object],
    *,
    active_maneuver,
    scan_available: bool,
    scan_eligible_target_ids,
    scan_blocked_by_rotation: bool = False,
    clearance_reverse_available: bool = False,
):
    """Identify unresolved forward hazards before a first detour turn."""

    if not (
        scan_available
        or (scan_blocked_by_rotation and clearance_reverse_available)
    ):
        return ()
    active_target_id = (
        active_maneuver.get("target_hypothesis_id")
        if isinstance(active_maneuver, Mapping)
        else None
    )
    eligible = frozenset(scan_eligible_target_ids)
    conflict_ids = _active_goal_conflict_ids(navigation)
    return tuple(sorted(
        hypothesis["hypothesis_id"]
        for hypothesis in navigation["navigation_hazard_hypotheses"]
        if hypothesis["active_for_collision"] is True
        and hypothesis["hypothesis_id"] in conflict_ids
        and hypothesis["hypothesis_id"] in eligible
        and hypothesis["hypothesis_id"] != active_target_id
        and hypothesis["route_commitment_ready"] is not True
    ))


def detour_scan_gate(
    navigation: Mapping[str, object],
    *,
    active_maneuver,
    scan_eligible_target_ids,
    scan_budget_available: bool,
    reverse_budget_available: bool,
):
    """Publish scan availability and gate turns while retreat can make room."""

    feasibility = navigation["action_feasibility"]
    rotation = feasibility["active_scan"]
    scan_ready_except_rotation = (
        bool(scan_eligible_target_ids) and scan_budget_available
    )
    scan_available = scan_ready_except_rotation and rotation["allowed"]
    reverse_available = (
        feasibility["motion_actions"]["REVERSE"]["allowed"]
        and reverse_budget_available
    )
    required = detour_scan_required_target_ids(
        navigation,
        active_maneuver=active_maneuver,
        scan_available=scan_available,
        scan_eligible_target_ids=scan_eligible_target_ids,
        scan_blocked_by_rotation=(
            scan_ready_except_rotation
            and rotation["reason"]
            == "provisional_hazard_rotation_sweep_collision"
        ),
        clearance_reverse_available=reverse_available,
    )
    return scan_available, required


def detour_turn_commitment_error(
    action: str,
    commitment: Mapping[str, object],
    navigation: Mapping[str, object],
):
    """Keep strategic route authorization separate from physical dispatch."""

    transition = commitment.get("transition")
    if transition == "START" and action != OBSERVE:
        return (
            "detour_start_requires_observe",
            "Authorize a detour with singleton OBSERVE before route execution",
        )
    conflict_ids = _active_goal_conflict_ids(navigation)
    required_ids = frozenset(
        hypothesis["hypothesis_id"]
        for hypothesis in navigation["navigation_hazard_hypotheses"]
        if hypothesis["active_for_collision"] is True
        and hypothesis["hypothesis_id"] in conflict_ids
        and hypothesis["route_commitment_ready"] is True
    )
    if not required_ids:
        return None
    if action in DETOUR_TURN_ACTIONS and transition == "NONE":
        return (
            "detour_commitment_required",
            "Authorize the scanned detour route before physical turning",
        )
    if (
        transition == "START"
        and commitment.get("target_hypothesis_id") not in required_ids
    ):
        return (
            "detour_commitment_target_mismatch",
            "The detour commitment must target the scanned goal conflict",
        )
    if transition == "START" and (
        frozenset(commitment.get("success_fact_keys", ()))
        != DETOUR_SUCCESS_FACT_KEYS
        or commitment.get("current_focus_fact_key")
        != FACT_GOAL_CORRIDOR_CLEAR
    ):
        return (
            "detour_commitment_facts_required",
            "A detour must track corridor, heading, and target-passed facts",
        )
    return None


def detour_scan_target_error(
    action: str,
    target_hypothesis_id,
    navigation: Mapping[str, object],
):
    """Keep a required pre-detour scan correlated with its conflict."""

    required = frozenset(
        navigation.get("detour_scan_required_target_hypothesis_ids", ())
    )
    if (
        action == SCAN_FRONT_ARC
        and required
        and target_hypothesis_id not in required
    ):
        return (
            "detour_scan_target_mismatch",
            "The required pre-detour scan must target the goal conflict",
        )
    return None


def available_navigation_actions(
    *,
    action_feasibility: Mapping[str, object],
    action_specs: Mapping[str, Mapping[str, object]],
    observation: Mapping[str, object],
    repeated_uninformative_observe: bool,
    scan_available: bool,
    detour_scan_required_target_ids,
):
    """Apply deterministic budget, scan, and pre-detour availability gates."""

    required = bool(detour_scan_required_target_ids)
    return tuple(
        action
        for action in sorted(ACTIONS)
        if (action != OBSERVE or not repeated_uninformative_observe)
        and (
            action not in MOTION_ACTIONS
            or (
                action_feasibility["motion_actions"][action]["allowed"]
                and motion_budget_allows(action, observation, action_specs)
            )
        )
        and (action != SCAN_FRONT_ARC or scan_available)
        and (action not in DETOUR_TURN_ACTIONS or not required)
    )


def prepare_navigation_availability(
    navigation: Mapping[str, object],
    *,
    active_maneuver,
    scan_eligible_target_ids,
    scan_blocked_target_ids,
    scan_budget_available: bool,
    reverse_budget_available: bool,
    action_specs,
    observation,
    repeated_uninformative_observe: bool,
):
    """Publish scan gates and return the deterministic base action set."""

    # This value is part of the JSON-shaped planner contract.  Keep it a
    # list even when the internal callers use tuples or sets; the planner
    # deliberately rejects non-JSON sequence types before inference.
    route_ready_ids = frozenset(
        hypothesis["hypothesis_id"]
        for hypothesis in navigation[
            "navigation_hazard_hypotheses"
        ]
        if hypothesis["route_commitment_ready"] is True
    )
    eligible = sorted(
        hypothesis_id
        for hypothesis_id in scan_eligible_target_ids
        if hypothesis_id not in route_ready_ids
    )
    navigation["scan_eligible_target_hypothesis_ids"] = eligible
    navigation["scan_progress_blocked_target_hypothesis_ids"] = sorted(
        scan_blocked_target_ids
    )
    scan_available, required = detour_scan_gate(
        navigation,
        active_maneuver=active_maneuver,
        scan_eligible_target_ids=eligible,
        scan_budget_available=scan_budget_available,
        reverse_budget_available=reverse_budget_available,
    )
    navigation["detour_scan_required_target_hypothesis_ids"] = list(
        required
    )
    return available_navigation_actions(
        action_feasibility=navigation["action_feasibility"],
        action_specs=action_specs,
        observation=observation,
        repeated_uninformative_observe=repeated_uninformative_observe,
        scan_available=scan_available,
        detour_scan_required_target_ids=required,
    )


def detour_decision_error(
    action,
    perception_target_hypothesis_id,
    commitment,
    navigation,
):
    """Return the first deterministic detour-contract violation."""

    error = detour_turn_commitment_error(action, commitment, navigation)
    if error is not None:
        return error
    error = detour_scan_target_error(
        action,
        perception_target_hypothesis_id,
        navigation,
    )
    if error is not None:
        return error
    return None


def navigation_action_feasibility(
    *,
    hazard_map: ProvisionalHazardMap,
    pose: PhysicalPose,
    action_specs: Mapping[str, Mapping[str, object]],
    odometry_calibration: OdometryCalibration,
    active_scan_calibration: ActiveIrScanCalibration,
) -> Mapping[str, object]:
    """Describe geometry feasibility without ranking or selecting an action."""

    if not isinstance(hazard_map, ProvisionalHazardMap):
        raise ValueError("action feasibility hazard map is invalid")
    if not isinstance(pose, PhysicalPose):
        raise ValueError("action feasibility pose is invalid")
    if not isinstance(action_specs, Mapping):
        raise ValueError("action feasibility specs are invalid")
    if not isinstance(odometry_calibration, OdometryCalibration):
        raise ValueError("action feasibility odometry is invalid")
    if not isinstance(active_scan_calibration, ActiveIrScanCalibration):
        raise ValueError("action feasibility scan calibration is invalid")

    motion = {}
    for action in sorted(MOTION_ACTIONS):
        result = hazard_map.validate_swept_path(
            pose,
            action,
            action_specs,
            odometry_calibration,
        )
        motion[action] = {
            "allowed": result["allowed"],
            "reason": result["reason"],
            "hazard_ids": list(result["hazard_ids"]),
            "monotonic_escape_hazard_ids": list(
                result.get("monotonic_escape_hazard_ids", ())
            ),
            "maximum_endpoint": result.get("maximum_endpoint"),
            "host_selected_alternative_action": False,
        }

    scan = hazard_map.validate_in_place_rotation(
        pose,
        active_scan_calibration.coarse_offsets_mdeg,
        alignment_tolerance_mdeg=(
            active_scan_calibration.alignment_tolerance_mdeg
        ),
    )
    return {
        "collision_geometry": hazard_map.calibration.collision_geometry(),
        "motion_actions": motion,
        "active_scan": {
            "allowed": scan["allowed"],
            "reason": scan["reason"],
            "hazard_ids": list(scan["hazard_ids"]),
            "minimum_relative_heading_mdeg": scan[
                "minimum_relative_heading_mdeg"
            ],
            "maximum_relative_heading_mdeg": scan[
                "maximum_relative_heading_mdeg"
            ],
            "host_selected_alternative_action": False,
        },
        "host_ranked_or_selected_action": False,
    }


__all__ = (
    "DETOUR_TURN_ACTIONS",
    "available_navigation_actions",
    "detour_decision_error",
    "detour_scan_gate",
    "detour_scan_required_target_ids",
    "detour_scan_target_error",
    "detour_turn_commitment_error",
    "navigation_action_feasibility",
    "prepare_navigation_availability",
)
