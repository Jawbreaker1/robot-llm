"""Neutral safety gates for one bounded BLAST scan."""

from __future__ import annotations

import math
from typing import Mapping

from .blast_navigation_action_profile import blast_scan_turn_maximum_pose
from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_observation_monitor import (
    RANGE_STATE_NO_VALID_DISTANCE,
    blast_range_state,
)
from .physical_footprint import footprint_sweep_intersects
from .physical_navigation_contract import (
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from .physical_odometry import PhysicalPose


_PROJECTION_SCHEMA = "blast-planar-scan-projection/v1"
_TARGET_DEPTH_BAND_MM = 45
_TARGET_INTERIOR_MM = 45


class BlastScanPermitUnavailable(RuntimeError):
    """A bounded scan lacked its anchor-bound action permit."""

    code = "blast_action_start_unverified"


def _pose(value, name):
    try:
        return PhysicalPose.from_mapping(dict(value))
    except (TypeError, ValueError):
        raise ValueError(f"BLAST {name} pose is invalid") from None


def _axes(origin: PhysicalPose, x_mm: int, y_mm: int):
    heading = math.radians(origin.heading_mdeg / 1_000.0)
    dx = x_mm - origin.x_mm
    dy = y_mm - origin.y_mm
    return (
        dx * math.cos(heading) + dy * math.sin(heading),
        -dx * math.sin(heading) + dy * math.cos(heading),
    )


def _projection_points(view, origin):
    projection = view.get("planar_projection") if isinstance(
        view, Mapping
    ) else None
    if not (
        isinstance(projection, Mapping)
        and projection.get("schema") == _PROJECTION_SCHEMA
        and projection.get("frame") == "EPISODE_LOCAL_ODOMETRY"
        and projection.get("quality") == "PROVISIONAL_YAW_ONLY"
        and isinstance(projection.get("points"), list)
    ):
        raise ValueError("BLAST target projection is invalid")
    points = []
    for point in projection["points"]:
        x_mm = point.get("nominal_echo_x_mm") if isinstance(
            point, Mapping
        ) else None
        y_mm = point.get("nominal_echo_y_mm") if isinstance(
            point, Mapping
        ) else None
        side = point.get("side") if isinstance(point, Mapping) else None
        if type(x_mm) is not int or type(y_mm) is not int or not isinstance(
            side, str
        ):
            raise ValueError("BLAST target projection point is invalid")
        longitudinal, lateral = _axes(origin, x_mm, y_mm)
        points.append((side, longitudinal, lateral, point))
    return points


def _frozen_target_support(origin_view):
    if not isinstance(origin_view, Mapping):
        raise ValueError("BLAST frozen target view is invalid")
    origin = _pose(origin_view.get("scan_pose"), "frozen target")
    points = _projection_points(origin_view, origin)
    centers = [point for point in points if point[0] == "center"]
    if len(centers) != 1:
        raise ValueError("BLAST frozen target center is unavailable")
    center = centers[0]
    support = [
        point for point in points
        if abs(point[1] - center[1]) <= _TARGET_DEPTH_BAND_MM
    ]
    lateral_min = min((point[2] for point in support), default=math.inf)
    lateral_max = max((point[2] for point in support), default=-math.inf)
    if not (
        math.isfinite(lateral_min)
        and math.isfinite(lateral_max)
        and lateral_min <= center[2] <= lateral_max
        and lateral_max - lateral_min >= 2 * _TARGET_INTERIOR_MM
    ):
        raise ValueError("BLAST frozen target support is insufficient")
    radius = max(1, math.ceil(max(
        math.hypot(point[1] - center[1], point[2] - center[2])
        for point in support
    )))
    raw_center = center[3]
    return {
        "center_x_mm": raw_center["nominal_echo_x_mm"],
        "center_y_mm": raw_center["nominal_echo_y_mm"],
        "radius_mm": radius,
    }


def blast_scan_sweep_is_clear(scan_view, pose) -> bool:
    """Whether the remembered obstacle is outside either scan-turn sweep."""

    if not isinstance(pose, PhysicalPose):
        return False
    try:
        support = _frozen_target_support(scan_view)
    except ValueError:
        return False
    footprint, _sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    )
    for action in (TURN_LEFT_90, TURN_RIGHT_90):
        maximum = blast_scan_turn_maximum_pose(pose, action)
        _start, intersects = footprint_sweep_intersects(
            obstacle_x_mm=support["center_x_mm"],
            obstacle_y_mm=support["center_y_mm"],
            obstacle_radius_mm=support["radius_mm"],
            start=pose,
            end=maximum,
            footprint=footprint,
        )
        if intersects:
            return False
    return True


def issue_blast_scan_permit(
    *, controller, action, distance_mm, geometry_checked, pose,
    prior_receipt, expected_drive_angles,
):
    """Issue one anchor-bound permit; NVD retains its geometry gate."""

    action_permit = None
    if action == SCAN_FRONT_ARC:
        allow_no_return = (
            blast_range_state(distance_mm) == RANGE_STATE_NO_VALID_DISTANCE
            and geometry_checked
        )
        issue = getattr(controller, "issue_no_return_scan_permit", None)
        if callable(issue):
            action_permit = issue(
                pose=pose.to_dict(),
                prior_receipt=prior_receipt,
                geometry_checked=geometry_checked,
                expected_drive_angles=expected_drive_angles,
                allow_no_return=allow_no_return,
            )
        if action_permit is None and (callable(issue) or allow_no_return):
            raise BlastScanPermitUnavailable(
                "BLAST scan action permit was unavailable"
            )
    return action_permit


__all__ = (
    "BlastScanPermitUnavailable",
    "blast_scan_sweep_is_clear",
    "issue_blast_scan_permit",
)
