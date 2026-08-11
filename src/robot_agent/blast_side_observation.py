"""Pure diagnostic assembly for bounded BLAST side observations."""

from __future__ import annotations

import copy
import math
from typing import Mapping

from .blast_side_search_geometry import POSITION_TOLERANCE_MM
from .blast_side_search_geometry import (
    TARGET_REACQUISITION_SEARCH_BASIS,
    target_reacquisition_resolved,
    target_reacquisition_waypoint,
)
from .blast_observation_monitor import RANGE_STATE_MEASURED, blast_range_state
from .blast_navigation_policy import settled_no_return_at_pose
from .physical_navigation_contract import (
    ADVANCE, SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90,
)
from .physical_odometry import PhysicalPose


def side_search_action_admission(
    progress, waypoint, sensors, available_actions, evidence_correlated,
    rotation_allowed, *, current_pose=None, prior_receipt=None,
    no_return_scan_geometry_checked=False,
):
    """Return one admitted host action and an optional typed block reason."""

    action = progress.get("required_action")
    no_return_scan = (
        progress.get("phase") == "RESCAN"
        and action == SCAN_FRONT_ARC
        and no_return_scan_geometry_checked is True
        and settled_no_return_at_pose(
            sensors.get("distance_mm"), current_pose, prior_receipt,
        )
    )
    if not evidence_correlated:
        action = None
        no_return_scan = False
    if action == ADVANCE and (
        blast_range_state(sensors.get("distance_mm"))
        != RANGE_STATE_MEASURED or ADVANCE not in available_actions
    ):
        action = None
    if action in (TURN_LEFT_90, TURN_RIGHT_90, SCAN_FRONT_ARC) and not (
        rotation_allowed or no_return_scan
    ):
        action = None
    if action is not None and (
        action in available_actions or no_return_scan
    ):
        return (action,), None
    reason = (
        "target_reacquisition_blocked"
        if waypoint.get("search_basis")
        == TARGET_REACQUISITION_SEARCH_BASIS
        else "blast_side_search_blocked"
    )
    return (), reason


def side_search_planned_leg(selected_side, waypoint, bind_pose):
    """Build the conservative map leg for the current search waypoint."""

    if (
        selected_side not in ("LEFT", "RIGHT")
        or not isinstance(waypoint, Mapping)
        or not isinstance(bind_pose, PhysicalPose)
    ):
        return None
    return {
        "kind": "SIDE_SEARCH",
        "scope": "SEARCH_POSITION_ONLY",
        "clearance_proven": False,
        "passage_proven": False,
        "route_eligible": False,
        "selected_side": selected_side,
        "bind_pose": {
            "x_mm": bind_pose.x_mm,
            "y_mm": bind_pose.y_mm,
            "heading_mdeg": bind_pose.heading_mdeg,
        },
        "waypoint": {
            "x_mm": waypoint["target_x_mm"],
            "y_mm": waypoint["target_y_mm"],
            "heading_mdeg": waypoint["target_heading_mdeg"],
        },
    }


def build_blast_multi_view_observation(
    *,
    origin_view,
    side_view,
    selected_side,
    waypoint,
    pose,
    diagnostic_scan,
    host_actions,
):
    """Return the detached, explicitly non-proving multi-view payload."""

    if (
        selected_side not in ("LEFT", "RIGHT")
        or not isinstance(origin_view, Mapping)
        or not isinstance(side_view, Mapping)
        or not isinstance(waypoint, Mapping)
        or not isinstance(pose, PhysicalPose)
        or not isinstance(diagnostic_scan, Mapping)
        or not isinstance(host_actions, (list, tuple))
        or side_view.get("scan_pose") != pose.to_dict()
    ):
        raise ValueError("BLAST side observation is invalid")
    origin_pose = origin_view.get("scan_pose")
    side_pose = side_view.get("scan_pose")
    if not isinstance(origin_pose, Mapping) or not isinstance(
        side_pose, Mapping
    ):
        raise ValueError("BLAST side observation pose is invalid")
    try:
        separation = int(round(math.hypot(
            side_pose["x_mm"] - origin_pose["x_mm"],
            side_pose["y_mm"] - origin_pose["y_mm"],
        )))
        stride = int(round(math.hypot(
            waypoint["target_x_mm"] - origin_pose["x_mm"],
            waypoint["target_y_mm"] - origin_pose["y_mm"],
        )))
    except (KeyError, TypeError, ValueError):
        raise ValueError("BLAST side observation pose is invalid") from None
    if separation < stride - POSITION_TOLERANCE_MM:
        raise ValueError("BLAST side viewpoints are not distinct")
    final_scan = copy.deepcopy(diagnostic_scan)
    final_scan["multi_view_observations"] = {
        "schema": "blast-multi-view-scan-observations/v1",
        "frame": "EPISODE_LOCAL_ODOMETRY",
        "quality": "PROVISIONAL_YAW_ONLY",
        "selected_side": selected_side,
        "strategy_source": "PLANNER_ACTION",
        "execution_source": "HOST_SIDE_SEARCH_ACTION",
        "host_action_count": len(host_actions),
        "host_action_trace": list(host_actions),
        "viewpoint_separation_mm": separation,
        "object_association_proven": False,
        "clearance_proven": False,
        "passage_proven": False,
        "route_eligible": False,
        "views": [copy.deepcopy(origin_view), copy.deepcopy(side_view)],
    }
    return final_scan


def finish_target_reacquisition(
    final_scan, origin_view, side_view, selected_side, waypoint,
):
    """Annotate and classify the one allowed reacquisition observation."""

    if waypoint.get("search_basis") != TARGET_REACQUISITION_SEARCH_BASIS:
        return None
    resolved = target_reacquisition_resolved(
        origin_view, side_view, selected_side,
    )
    final_scan["multi_view_observations"]["target_reacquisition"] = {
        "attempted": True,
        "resolved": resolved,
        "search_basis": TARGET_REACQUISITION_SEARCH_BASIS,
    }
    return (
        "target_reacquisition_observation_collected"
        if resolved else "target_reacquisition_unresolved"
    )


def plan_target_reacquisition(
    final_scan,
    origin_view,
    side_view,
    selected_side,
    pose,
    remaining_slots,
):
    """Return `(waypoint, budget_insufficient)` for exact no-return loss."""

    try:
        waypoint = target_reacquisition_waypoint(
            origin_view, side_view, selected_side, pose,
        )
    except ValueError:
        return None, False
    if waypoint["required_action_slots"] > remaining_slots:
        return None, True
    final_scan["multi_view_observations"]["target_reacquisition"] = {
        "attempted": False,
        "resolved": False,
        "search_basis": TARGET_REACQUISITION_SEARCH_BASIS,
    }
    return waypoint, False


__all__ = (
    "build_blast_multi_view_observation",
    "finish_target_reacquisition",
    "plan_target_reacquisition",
    "side_search_action_admission",
    "side_search_planned_leg",
)
