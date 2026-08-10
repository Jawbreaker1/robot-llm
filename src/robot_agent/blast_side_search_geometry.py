"""Pure, provisional BLAST side-observation geometry."""

from __future__ import annotations

import math
from typing import Mapping

from .blast_navigation_action_profile import BLAST_NAVIGATION_ACTION_SPECS
from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_observation_monitor import (
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    validate_blast_scan_ray_contract,
)
from .physical_footprint import footprint_sweep_intersects
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
# Admit the observed 85.96-degree turn while retaining useful margin inside
# the calibrated rotation envelope at the current 450 mm search cap.
_MAX_TOWARD_FRONT_HEADING_MDEG = 4_100
TARGET_REACQUISITION_SEARCH_BASIS = "FROZEN_SUPPORT_REACQUISITION"
_TARGET_DEPTH_BAND_MM = 45
_TARGET_INTERIOR_MM = 45
_TARGET_MAX_BEARING_DEG = 30.0
_TARGET_MAX_INWARD_PULSES = 6
_TARGET_OBSERVATION_HEADING_TOLERANCE_MDEG = 5_000


def _nominal_pose(pose: PhysicalPose, action: str) -> PhysicalPose:
    return nominal_effect(
        pose,
        action,
        BLAST_NAVIGATION_ACTION_SPECS,
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
    )[0]


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
        "origin": origin,
        "center_x_mm": raw_center["nominal_echo_x_mm"],
        "center_y_mm": raw_center["nominal_echo_y_mm"],
        "center_longitudinal_mm": center[1],
        "center_lateral_mm": center[2],
        "lateral_min_mm": lateral_min,
        "lateral_max_mm": lateral_max,
        "radius_mm": radius,
    }


def _sensor_target_geometry(pose, scan_heading_mdeg, support):
    sensor = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.range_sensor_extrinsics
    if not sensor.complete:
        raise ValueError("BLAST range-sensor geometry is unavailable")
    forward = sensor.forward_offset_mm
    left = sensor.left_offset_mm
    yaw = sensor.yaw_mdeg
    assert forward is not None and left is not None and yaw is not None
    heading = math.radians(scan_heading_mdeg / 1_000.0)
    sensor_x = pose.x_mm + forward * math.cos(heading) - left * math.sin(
        heading
    )
    sensor_y = pose.y_mm + forward * math.sin(heading) + left * math.cos(
        heading
    )
    dx = support["center_x_mm"] - sensor_x
    dy = support["center_y_mm"] - sensor_y
    beam_heading = math.radians((scan_heading_mdeg + yaw) / 1_000.0)
    forward_delta = dx * math.cos(beam_heading) + dy * math.sin(beam_heading)
    left_delta = -dx * math.sin(beam_heading) + dy * math.cos(beam_heading)
    _longitudinal, sensor_lateral = _axes(
        support["origin"], round(sensor_x), round(sensor_y)
    )
    return sensor_lateral, forward_delta, math.degrees(math.atan2(
        left_delta, forward_delta
    ))


def _target_side_has_only_no_valid_rays(view, selected_side):
    if not isinstance(view, Mapping):
        return False
    try:
        scan = validate_blast_scan_ray_contract(view.get("scan"))
    except ValueError:
        return False
    prefix = "right_" if selected_side == "LEFT" else "left_"
    rays = [ray for ray in scan["rays"] if ray["side"].startswith(prefix)]
    center = [ray for ray in scan["rays"] if ray["side"] == "center"]
    return (
        len(center) == 1
        and center[0]["range_state"] == RANGE_STATE_MEASURED
        and len(rays) == 2
        and all(
            ray["range_state"] == RANGE_STATE_NO_VALID_DISTANCE
            and float(ray["distance_mm"]) == 2_000.0
            for ray in rays
        )
    )


def _reacquisition_action_is_clear(pose, waypoint, action):
    try:
        target_x = waypoint["frozen_target_centroid_x_mm"]
        target_y = waypoint["frozen_target_centroid_y_mm"]
        radius = waypoint["frozen_target_radius_mm"]
    except (KeyError, TypeError):
        return False
    if any(type(value) is not int for value in (target_x, target_y, radius)):
        return False
    if action not in (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90):
        return False
    _nominal, maximum = nominal_effect(
        pose,
        action,
        BLAST_NAVIGATION_ACTION_SPECS,
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
    )
    footprint, _sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    )
    _start_intersects, sweep_intersects = footprint_sweep_intersects(
        obstacle_x_mm=target_x,
        obstacle_y_mm=target_y,
        obstacle_radius_mm=radius,
        start=pose,
        end=maximum,
        footprint=footprint,
    )
    return not sweep_intersects


def _exceeds_toward_front_limit(
    heading_mdeg: int,
    origin_heading_mdeg: int,
) -> bool:
    difference = abs(normalize_heading_mdeg(
        heading_mdeg - origin_heading_mdeg
    ))
    return difference < (
        90_000 - _MAX_TOWARD_FRONT_HEADING_MDEG
    )


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
        if _exceeds_toward_front_limit(
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


def target_reacquisition_waypoint(
    origin_view,
    failed_side_view,
    selected_side,
    current_pose,
):
    """Return one bounded inward viewpoint for a frozen target hint."""

    if selected_side not in ("LEFT", "RIGHT") or not isinstance(
        current_pose, PhysicalPose
    ):
        raise ValueError("BLAST target reacquisition request is invalid")
    support = _frozen_target_support(origin_view)
    if not isinstance(failed_side_view, Mapping) or (
        failed_side_view.get("scan_pose") != current_pose.to_dict()
    ) or not (
        abs(normalize_heading_mdeg(
            current_pose.heading_mdeg - support["origin"].heading_mdeg
        )) <= _TARGET_OBSERVATION_HEADING_TOLERANCE_MDEG
        and _target_side_has_only_no_valid_rays(
            failed_side_view, selected_side
        )
    ):
        raise ValueError("BLAST target reacquisition evidence is unavailable")
    current_sensor_lateral, target_forward, target_bearing = (
        _sensor_target_geometry(
            current_pose, current_pose.heading_mdeg, support
        )
    )
    sign = 1 if selected_side == "LEFT" else -1
    if target_forward <= 0 or sign * target_bearing >= 0:
        raise ValueError("BLAST frozen target is not on the inward side")
    interior_min = support["lateral_min_mm"] + _TARGET_INTERIOR_MM
    interior_max = support["lateral_max_mm"] - _TARGET_INTERIOR_MM
    if interior_min <= current_sensor_lateral <= interior_max:
        raise ValueError("BLAST target view is already inside its support")
    inward_action = TURN_RIGHT_90 if selected_side == "LEFT" else TURN_LEFT_90
    restore_action = TURN_LEFT_90 if selected_side == "LEFT" else TURN_RIGHT_90
    candidate = _nominal_pose(current_pose, inward_action)
    selected = None
    for pulse_count in range(1, _TARGET_MAX_INWARD_PULSES + 1):
        candidate = _nominal_pose(candidate, ADVANCE)
        sensor_lateral, forward, bearing = _sensor_target_geometry(
            candidate, current_pose.heading_mdeg, support
        )
        candidate_lateral = _axes(
            support["origin"], candidate.x_mm, candidate.y_mm
        )[1]
        current_lateral = _axes(
            support["origin"], current_pose.x_mm, current_pose.y_mm
        )[1]
        if (
            sign * (candidate_lateral - current_lateral) < 0
            and interior_min <= sensor_lateral <= interior_max
            and forward > 0
            and sign * bearing < 0
            and abs(bearing) <= _TARGET_MAX_BEARING_DEG
        ):
            selected = (candidate, pulse_count, candidate_lateral, bearing)
            break
    if selected is None:
        raise ValueError("BLAST target has no bounded reacquisition viewpoint")
    candidate, pulse_count, candidate_lateral, bearing = selected
    travel_side = "RIGHT" if selected_side == "LEFT" else "LEFT"
    return {
        "kind": "SIDE_SEARCH",
        "selected_side": selected_side,
        "travel_side": travel_side,
        "scope": "SEARCH_POSITION_ONLY",
        "clearance_proven": False,
        "passage_proven": False,
        "route_eligible": False,
        "frame": "EPISODE_LOCAL_ODOMETRY",
        "origin_pose": current_pose.to_dict(),
        "target_x_mm": candidate.x_mm,
        "target_y_mm": candidate.y_mm,
        "target_heading_mdeg": candidate.heading_mdeg,
        "position_tolerance_mm": POSITION_TOLERANCE_MM,
        "target_lateral_offset_mm": int(round(candidate_lateral)),
        "search_basis": TARGET_REACQUISITION_SEARCH_BASIS,
        "search_target_capped": False,
        "reacquisition_advance_count": pulse_count,
        "required_action_slots": pulse_count + 3,
        "predicted_target_bearing_mdeg": round(bearing * 1_000),
        "frozen_target_centroid_x_mm": support["center_x_mm"],
        "frozen_target_centroid_y_mm": support["center_y_mm"],
        "frozen_target_radius_mm": support["radius_mm"],
        "inward_turn_action": inward_action,
        "restore_turn_action": restore_action,
    }


def target_reacquisition_resolved(
    origin_view,
    reacquisition_view,
    selected_side,
) -> bool:
    """Whether a closer view measured the frozen front support again."""

    if selected_side not in ("LEFT", "RIGHT"):
        return False
    try:
        support = _frozen_target_support(origin_view)
        scan_pose = _pose(
            reacquisition_view.get("scan_pose"), "target reacquisition"
        )
        scan = validate_blast_scan_ray_contract(reacquisition_view.get("scan"))
        points = _projection_points(
            reacquisition_view, support["origin"]
        )
    except (AttributeError, ValueError):
        return False
    if abs(normalize_heading_mdeg(
        scan_pose.heading_mdeg - support["origin"].heading_mdeg
    )) > _TARGET_OBSERVATION_HEADING_TOLERANCE_MDEG:
        return False
    _sensor_lateral, forward, bearing = _sensor_target_geometry(
        scan_pose, scan_pose.heading_mdeg, support
    )
    sign = 1 if selected_side == "LEFT" else -1
    if forward <= 0 or sign * bearing >= 0 or abs(
        bearing
    ) > _TARGET_MAX_BEARING_DEG:
        return False
    prefix = "right_" if selected_side == "LEFT" else "left_"
    measured_sides = {
        ray["side"] for ray in scan["rays"]
        if ray["side"].startswith(prefix)
        and ray["range_state"] == RANGE_STATE_MEASURED
    }
    return any(
        side in measured_sides
        and abs(longitudinal - support["center_longitudinal_mm"])
        <= _TARGET_DEPTH_BAND_MM
        and support["lateral_min_mm"] <= lateral
        <= support["lateral_max_mm"]
        for side, longitudinal, lateral, _point in points
    )


def _advance_slots(distance_mm: float) -> int:
    step = _nominal_pose(PhysicalPose(), ADVANCE)
    step_mm = int(round(math.hypot(step.x_mm, step.y_mm)))
    return max(0, math.ceil(
        (distance_mm - POSITION_TOLERANCE_MM) / step_mm
    ))


def side_search_required_slots(waypoint) -> int:
    if waypoint.get("search_basis") == TARGET_REACQUISITION_SEARCH_BASIS:
        required = waypoint.get("required_action_slots")
        if type(required) is not int or not 4 <= required <= 9:
            raise ValueError("BLAST target reacquisition budget is invalid")
        return required
    outward_mm = abs(waypoint["target_lateral_offset_mm"])
    worst_path_mm = outward_mm / math.cos(math.radians(
        HEADING_TOLERANCE_MDEG / 1_000.0
    ))
    return 1 + _advance_slots(worst_path_mm) + 1 + 1


def side_search_followup_slots(pose, waypoint) -> int:
    if waypoint.get("search_basis") == TARGET_REACQUISITION_SEARCH_BASIS:
        return side_search_required_slots(waypoint)
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
    reacquiring = (
        waypoint.get("search_basis")
        == TARGET_REACQUISITION_SEARCH_BASIS
    )
    if reorientation_attempted:
        rescan_tolerance = (
            _TARGET_OBSERVATION_HEADING_TOLERANCE_MDEG
            if reacquiring else HEADING_TOLERANCE_MDEG
        )
        if distance <= POSITION_TOLERANCE_MM and abs(
            origin_error
        ) <= rescan_tolerance:
            phase, heading_error, required_action = (
                "RESCAN", origin_error, SCAN_FRONT_ARC
            )
    elif distance > POSITION_TOLERANCE_MM:
        phase = "OUTBOUND"
        if (
            abs(heading_error) <= HEADING_TOLERANCE_MDEG
            and not _exceeds_toward_front_limit(
                pose.heading_mdeg,
                origin_heading,
            )
            and side_search_distance(
                _nominal_pose(pose, ADVANCE), waypoint
            ) < distance
            and (
                not reacquiring
                or _reacquisition_action_is_clear(pose, waypoint, ADVANCE)
            )
        ):
            required_action = ADVANCE
        elif (
            reacquiring
            and abs(origin_error) <= HEADING_TOLERANCE_MDEG
        ):
            action = waypoint.get("inward_turn_action")
            if action in (TURN_LEFT_90, TURN_RIGHT_90):
                projected_error = normalize_heading_mdeg(
                    waypoint["target_heading_mdeg"]
                    - _nominal_pose(pose, action).heading_mdeg
                )
                if abs(projected_error) <= HEADING_TOLERANCE_MDEG:
                    phase = "ORIENT_INWARD"
                    if _reacquisition_action_is_clear(
                        pose, waypoint, action
                    ):
                        required_action = action
    else:
        phase = "REORIENT"
        heading_error = origin_error
        action = waypoint.get("restore_turn_action") if reacquiring else (
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
            and (
                not reacquiring
                or _reacquisition_action_is_clear(pose, waypoint, action)
            )
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
    "TARGET_REACQUISITION_SEARCH_BASIS",
    "side_search_distance",
    "side_search_followup_slots",
    "side_search_progress",
    "side_search_required_slots",
    "side_search_waypoint",
    "target_reacquisition_resolved",
    "target_reacquisition_waypoint",
)
