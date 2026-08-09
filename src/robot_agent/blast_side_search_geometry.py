"""Pure, provisional BLAST side-observation geometry."""

from __future__ import annotations

import math
from typing import Mapping

from .blast_navigation_action_profile import BLAST_NAVIGATION_ACTION_SPECS
from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .physical_navigation_contract import (
    ADVANCE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from .physical_odometry import (
    PhysicalPose,
    nominal_effect,
    normalize_heading_mdeg,
)


POSITION_TOLERANCE_MM = 35
HEADING_TOLERANCE_MDEG = 20_000
_PROJECTION_SCHEMA = "blast-planar-scan-projection/v1"


def _nominal_pose(pose: PhysicalPose, action: str) -> PhysicalPose:
    return nominal_effect(
        pose,
        action,
        BLAST_NAVIGATION_ACTION_SPECS,
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
    )[0]


def _moves_toward_origin_front(
    heading_mdeg: int,
    origin_heading_mdeg: int,
) -> bool:
    difference = math.radians(
        normalize_heading_mdeg(heading_mdeg - origin_heading_mdeg) / 1_000.0
    )
    return math.cos(difference) > 1e-9


def side_search_distance(pose: PhysicalPose, waypoint) -> int:
    return int(round(math.hypot(
        waypoint["target_x_mm"] - pose.x_mm,
        waypoint["target_y_mm"] - pose.y_mm,
    )))


def _outward_mm(pose, side, scan_view, footprint, stride_mm):
    if scan_view is None:
        return stride_mm, False
    projection = scan_view.get("planar_projection") if isinstance(
        scan_view, Mapping
    ) else None
    if not (
        isinstance(scan_view, Mapping)
        and scan_view.get("scan_pose") == pose.to_dict()
        and isinstance(projection, Mapping)
        and projection.get("schema") == _PROJECTION_SCHEMA
        and projection.get("frame") == "EPISODE_LOCAL_ODOMETRY"
        and projection.get("quality") == "PROVISIONAL_YAW_ONLY"
        and isinstance(projection.get("points"), list)
    ):
        raise ValueError("BLAST side-search scan frame is invalid")
    heading = math.radians(pose.heading_mdeg / 1_000.0)
    forward = (math.cos(heading), math.sin(heading))
    left = (-forward[1], forward[0])
    local = []
    for point in projection["points"]:
        if not isinstance(point, Mapping):
            raise ValueError("BLAST side-search projection is invalid")
        x = point.get("nominal_echo_x_mm")
        y = point.get("nominal_echo_y_mm")
        if type(x) is not int or type(y) is not int:
            raise ValueError("BLAST side-search projection is invalid")
        delta = (x - pose.x_mm, y - pose.y_mm)
        local.append((
            point.get("side"),
            delta[0] * forward[0] + delta[1] * forward[1],
            delta[0] * left[0] + delta[1] * left[1],
        ))
    centers = [point for point in local if point[0] == "center"]
    if len(centers) != 1:
        raise ValueError("BLAST side-search center projection is invalid")
    sign = 1 if side == "LEFT" else -1
    prefix = "left_" if side == "LEFT" else "right_"
    band = footprint.clearance_margin_mm + POSITION_TOLERANCE_MM
    outward = [
        sign * lateral
        for ray_side, longitudinal, lateral in local
        if isinstance(ray_side, str)
        and ray_side.startswith(prefix)
        and abs(longitudinal - centers[0][1]) <= band
    ]
    if not outward:
        return stride_mm, False
    facing_extent = (
        footprint.right_extent_mm
        if side == "LEFT"
        else footprint.left_extent_mm
    )
    derived = math.ceil(max(outward) + facing_extent + band)
    return min(2 * stride_mm, max(stride_mm, derived)), derived > 2 * stride_mm


def side_search_waypoint(
    pose: PhysicalPose,
    side: str,
    *,
    scan_view=None,
    outbound_pose: PhysicalPose | None = None,
):
    footprint, _sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    )
    stride_mm = (
        footprint.left_extent_mm
        + footprint.right_extent_mm
        + 2 * footprint.clearance_margin_mm
    )
    if side not in ("LEFT", "RIGHT"):
        raise ValueError("BLAST side-search side is invalid")
    outward_mm, target_capped = _outward_mm(
        pose, side, scan_view, footprint, stride_mm
    )
    sign = 1 if side == "LEFT" else -1
    nominal_heading = normalize_heading_mdeg(pose.heading_mdeg + sign * 90_000)
    target_heading = nominal_heading
    travel_origin = pose
    travel_mm = outward_mm
    if outbound_pose is not None:
        if not isinstance(outbound_pose, PhysicalPose):
            raise ValueError("BLAST side-search outbound pose is invalid")
        error = normalize_heading_mdeg(
            outbound_pose.heading_mdeg - nominal_heading
        )
        if abs(error) > HEADING_TOLERANCE_MDEG:
            raise ValueError("BLAST side-search outbound pose is invalid")
        travel_origin = outbound_pose
        target_heading = outbound_pose.heading_mdeg
        origin_heading = math.radians(pose.heading_mdeg / 1_000.0)
        left = (-math.sin(origin_heading), math.cos(origin_heading))
        delta = (
            outbound_pose.x_mm - pose.x_mm,
            outbound_pose.y_mm - pose.y_mm,
        )
        current_lateral = delta[0] * left[0] + delta[1] * left[1]
        travel_heading = math.radians(target_heading / 1_000.0)
        travel_forward = (
            math.cos(travel_heading),
            math.sin(travel_heading),
        )
        if _moves_toward_origin_front(
            target_heading,
            pose.heading_mdeg,
        ):
            raise ValueError("BLAST side-search outbound pose is invalid")
        lateral_per_mm = (
            travel_forward[0] * left[0]
            + travel_forward[1] * left[1]
        )
        travel_mm = (sign * outward_mm - current_lateral) / lateral_per_mm
        if not math.isfinite(travel_mm) or travel_mm <= 0:
            raise ValueError("BLAST side-search outbound pose is invalid")
    travel_heading = math.radians(target_heading / 1_000.0)
    return {
        "kind": "SIDE_SEARCH",
        "selected_side": side,
        "scope": "SEARCH_POSITION_ONLY",
        "clearance_proven": False,
        "frame": "EPISODE_LOCAL_ODOMETRY",
        "origin_pose": pose.to_dict(),
        "target_x_mm": int(round(
            travel_origin.x_mm + travel_mm * math.cos(travel_heading)
        )),
        "target_y_mm": int(round(
            travel_origin.y_mm + travel_mm * math.sin(travel_heading)
        )),
        "target_heading_mdeg": target_heading,
        "position_tolerance_mm": POSITION_TOLERANCE_MM,
        "target_lateral_offset_mm": sign * outward_mm,
        "search_basis": (
            "PROVISIONAL_SAME_DEPTH_ECHO_REACH"
            if outward_mm > stride_mm
            else "FOOTPRINT_MINIMUM"
        ),
        "search_target_capped": target_capped,
    }


def _advance_slots(distance_mm: float) -> int:
    step = _nominal_pose(PhysicalPose(), ADVANCE)
    step_mm = int(round(math.hypot(step.x_mm, step.y_mm)))
    return max(0, math.ceil(
        (distance_mm - POSITION_TOLERANCE_MM) / step_mm
    ))


def side_search_required_slots(waypoint) -> int:
    outward_mm = abs(waypoint["target_lateral_offset_mm"])
    worst_path_mm = outward_mm / math.cos(math.radians(
        HEADING_TOLERANCE_MDEG / 1_000.0
    ))
    return 1 + _advance_slots(worst_path_mm) + 1 + 1


def side_search_followup_slots(pose, waypoint) -> int:
    return _advance_slots(side_search_distance(pose, waypoint)) + 1 + 1


def side_search_progress(
    pose: PhysicalPose,
    waypoint: Mapping[str, object],
    *,
    reorientation_attempted: bool = False,
):
    """Derive one bounded action for the selected side observation pose."""

    distance = side_search_distance(pose, waypoint)
    heading_error = normalize_heading_mdeg(
        waypoint["target_heading_mdeg"] - pose.heading_mdeg
    )
    required_action = None
    phase = "BLOCKED"
    origin_heading = waypoint["origin_pose"]["heading_mdeg"]
    origin_error = normalize_heading_mdeg(origin_heading - pose.heading_mdeg)
    if reorientation_attempted:
        if distance <= POSITION_TOLERANCE_MM and abs(
            origin_error
        ) <= HEADING_TOLERANCE_MDEG:
            phase, heading_error, required_action = (
                "RESCAN", origin_error, SCAN_FRONT_ARC
            )
    elif distance > POSITION_TOLERANCE_MM:
        phase = "OUTBOUND"
        if (
            abs(heading_error) <= HEADING_TOLERANCE_MDEG
            and not _moves_toward_origin_front(
                pose.heading_mdeg,
                origin_heading,
            )
            and side_search_distance(
                _nominal_pose(pose, ADVANCE), waypoint
            ) < distance
        ):
            required_action = ADVANCE
    else:
        phase = "REORIENT"
        heading_error = origin_error
        action = (
            TURN_RIGHT_90
            if waypoint["selected_side"] == "LEFT"
            else TURN_LEFT_90
        )
        projected_error = normalize_heading_mdeg(
            origin_heading - _nominal_pose(pose, action).heading_mdeg
        )
        if (
            abs(projected_error) < abs(heading_error)
            and abs(projected_error) <= HEADING_TOLERANCE_MDEG
        ):
            required_action = action
    return {
        "phase": phase,
        "distance_remaining_mm": distance,
        "heading_error_mdeg": heading_error,
        "required_action": required_action,
    }


__all__ = (
    "HEADING_TOLERANCE_MDEG",
    "POSITION_TOLERANCE_MM",
    "side_search_distance",
    "side_search_followup_slots",
    "side_search_progress",
    "side_search_required_slots",
    "side_search_waypoint",
)
