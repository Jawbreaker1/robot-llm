"""Context-only local-detour candidates from one restored BLAST scan."""

import math
from typing import Mapping

from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
    BlastNavigationCalibration,
)
from .blast_observation_monitor import SCAN_RESULT_SCHEMA
from .local_detour_route import build_local_detour_route
from .physical_odometry import PhysicalPose, normalize_heading_mdeg


CANDIDATE_SCHEMA = "blast-provisional-local-detour-candidates/v1"
_NO_VALID_RETURN_MM = 2_000.0
_POSITION_TOLERANCE_MM = 35
_RAY_ORDER = (
    "center", "left_near", "left_far", "right_near", "right_far",
)
_SIDE_RAYS = (
    ("LEFT_OF_GOAL", ("left_near", "left_far")),
    ("RIGHT_OF_GOAL", ("right_near", "right_far")),
)


def _finite(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _rays(scan):
    if (
        not isinstance(scan, Mapping)
        or scan.get("schema") != SCAN_RESULT_SCHEMA
        or scan.get("state") != "complete"
        or scan.get("result") != "restored"
        or scan.get("restoration_verified") is not True
        or scan.get("all_observations_settled") is not True
        or not isinstance(scan.get("rays"), list)
        or len(scan["rays"]) != 5
    ):
        return None
    result = {}
    for side, ray in zip(_RAY_ORDER, scan["rays"]):
        if (
            not isinstance(ray, Mapping)
            or ray.get("side") != side
            or ray.get("observation_settled") is not True
            or not _finite(ray.get("distance_mm"))
            or not 0 < float(ray["distance_mm"]) <= _NO_VALID_RETURN_MM
            or not _finite(ray.get("relative_heading_deg"))
            or not -180 <= float(ray["relative_heading_deg"]) <= 180
        ):
            return None
        result[side] = ray
    headings = tuple(
        float(result[side]["relative_heading_deg"])
        for side in _RAY_ORDER
    )
    if not (
        headings[2] < headings[1] < headings[0]
        < headings[3] < headings[4]
        and headings[1] < 0 < headings[3]
        and abs(headings[0]) <= 1
    ):
        return None
    return result


def _project(ray, pose, sensor):
    # BLAST reports negative scan deltas to the left; our map uses +Y left.
    bearing = normalize_heading_mdeg(
        pose.heading_mdeg
        - int(round(ray["relative_heading_deg"] * 1_000))
    )
    angle = math.radians(bearing / 1_000.0)
    origin = (
        pose.x_mm
        + sensor.forward_offset_mm * math.cos(angle)
        - sensor.left_offset_mm * math.sin(angle),
        pose.y_mm
        + sensor.forward_offset_mm * math.sin(angle)
        + sensor.left_offset_mm * math.cos(angle),
    )
    beam = math.radians(normalize_heading_mdeg(
        bearing + sensor.yaw_mdeg
    ) / 1_000.0)
    distance = float(ray["distance_mm"])
    return {
        "side": ray["side"],
        "distance": distance,
        "origin": origin,
        "endpoint": (
            origin[0] + distance * math.cos(beam),
            origin[1] + distance * math.sin(beam),
        ),
    }


def _point(values):
    return tuple(int(round(value)) for value in values)


def _front_intersection(ray, target_front):
    span = ray["endpoint"][0] - ray["origin"][0]
    if span <= 0:
        return None
    ratio = (target_front - ray["origin"][0]) / span
    if not 0 <= ratio <= 1:
        return None
    return _point((
        target_front,
        ray["origin"][1]
        + ratio * (ray["endpoint"][1] - ray["origin"][1]),
    ))


def build_blast_scan_route_candidates(
    scan,
    *,
    scan_pose: PhysicalPose,
    frame_id: str,
    map_generation_id: str,
    map_version: int,
    calibration: BlastNavigationCalibration = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
    ),
):
    """Build provisional route facts in the fixed episode frame (0, 0, 0)."""

    if not isinstance(scan_pose, PhysicalPose):
        raise ValueError("BLAST scan pose is invalid")
    rays = _rays(scan)
    if rays is None:
        return None
    footprint, sensor = calibration.require_complete()
    projected = {
        side: _project(ray, scan_pose, sensor)
        for side, ray in rays.items()
    }
    center = projected["center"]
    if (
        center["distance"] >= _NO_VALID_RETURN_MM
        or center["endpoint"][0] <= scan_pose.x_mm
    ):
        return None
    target_front = center["endpoint"][0]
    target_radius = max(1, footprint.clearance_margin_mm)
    pass_clearance = (
        target_radius
        + int(math.ceil(footprint.maximum_corner_radius_mm))
        + footprint.clearance_margin_mm
        + _POSITION_TOLERANCE_MM
    )
    preliminary_pass = target_front + pass_clearance
    target_x, target_y = _point(center["endpoint"])
    supports = {(target_x, target_y)}
    openings = {}
    for detour_side, ray_names in _SIDE_RAYS:
        opening = None
        for ray_name in ray_names:
            ray = projected[ray_name]
            if ray["distance"] >= _NO_VALID_RETURN_MM:
                continue
            if (
                ray["origin"][0] <= target_front
                and ray["endpoint"][0] >= preliminary_pass
            ):
                if opening is None:
                    opening = ray
                continue
            supports.add(_point(ray["endpoint"]))
            if opening is not None:
                opening = None
                break
        if opening is not None:
            bracket = _front_intersection(opening, target_front)
            bracket_on_side = (
                bracket is not None
                and (
                    bracket[1] > target_y
                    if detour_side == "LEFT_OF_GOAL"
                    else bracket[1] < target_y
                )
            )
            if bracket_on_side:
                supports.add(bracket)
                openings[detour_side] = opening
    supports = tuple(sorted(supports))
    corridor_min = -footprint.right_extent_mm - footprint.clearance_margin_mm
    corridor_max = footprint.left_extent_mm + footprint.clearance_margin_mm
    if not any(
        corridor_min <= y_mm <= corridor_max
        for _x_mm, y_mm in supports
    ):
        return None

    target_id = "blast-scan-target-{}".format(map_version)
    candidates = []
    for detour_side, _ray_names in _SIDE_RAYS:
        opening = openings.get(detour_side)
        if opening is None:
            continue
        route = build_local_detour_route(
            current_pose=scan_pose,
            goal_heading_mdeg=0,
            detour_side=detour_side,
            target_hypothesis_id=target_id,
            target_centroid_x_mm=target_x,
            target_centroid_y_mm=target_y,
            target_radius_mm=target_radius,
            target_support_points=supports,
            footprint=footprint,
            frame_id=frame_id,
            map_generation_id=map_generation_id,
            map_version=map_version,
            goal_origin_x_mm=0,
            goal_origin_y_mm=0,
            position_tolerance_mm=_POSITION_TOLERANCE_MM,
        )
        if opening["endpoint"][0] < route.pass_longitudinal_offset_mm:
            continue
        candidates.append({
            "detour_side": detour_side,
            "opening_ray": opening["side"],
            "measured_distance_mm": opening["distance"],
            "projected_endpoint_mm": list(_point(opening["endpoint"])),
            "route": route.to_dict(),
        })
    return {
        "schema": CANDIDATE_SCHEMA,
        "status": "provisional_context_only",
        "source_scan_pose": scan_pose.to_dict(),
        "target": {
            "hypothesis_id": target_id,
            "centroid_mm": [target_x, target_y],
            "support_points_mm": [list(point) for point in supports],
            "support_point_semantics": (
                "measured_endpoints_and_inferred_front_brackets"
            ),
            "full_depth_clearance_proven": False,
        },
        "candidates": candidates,
        "route_execution_authorized": False,
    }
