"""Bind BLAST's two-view box evidence to the shared local detour route."""

from __future__ import annotations

import math
from typing import Mapping

from .blast_navigation_action_profile import (
    BLAST_NAVIGATION_ACTION_SPECS,
    blast_scan_turn_maximum_pose,
)
from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_observation_monitor import (
    RANGE_STATE_INVALID,
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    SCAN_RAY_EVIDENCE_SWEEP_ONLY,
    validate_blast_scan_ray_contract,
)
from .blast_side_search_geometry import (
    target_side_has_only_settled_no_return,
)
from .local_detour_collision_snapshot import LocalDetourCollisionSnapshot
from .local_detour_controller import (
    build_local_detour_route_from_collision_snapshot,
    derive_local_detour_guidance,
)
from .local_detour_route import MERGE_GOAL_AXIS, ROUTE_COMPLETE
from .physical_footprint import footprint_sweep_intersects
from .physical_navigation_contract import ADVANCE, TURN_LEFT_90, TURN_RIGHT_90
from .physical_navigation_mission import DirectionalMission
from .physical_odometry import PhysicalPose, nominal_effect, normalize_heading_mdeg


_DEPTH_BAND_MM = 45
_MAX_PROVISIONAL_RADIUS_MM = 500
_MAX_ROUTE_STEPS = 64
# One coarse-turn residual can otherwise consume the route's longitudinal
# tolerance while BLAST merges across a wide lateral offset.
PASS_BUFFER_MM = 90
_ORIGIN_LATERAL_TOLERANCE_MM = 35


def _axes(mission: DirectionalMission, x_mm: int, y_mm: int):
    angle = math.radians(mission.reference_heading_mdeg / 1_000.0)
    dx = x_mm - mission.origin_x_mm
    dy = y_mm - mission.origin_y_mm
    return (
        dx * math.cos(angle) + dy * math.sin(angle),
        -dx * math.sin(angle) + dy * math.cos(angle),
    )


def _route_lateral(route, x_mm: int, y_mm: int) -> float:
    angle = math.radians(route.goal_heading_mdeg / 1_000.0)
    dx = x_mm - route.goal_origin_x_mm
    dy = y_mm - route.goal_origin_y_mm
    return -dx * math.sin(angle) + dy * math.cos(angle)


def _route_longitudinal(route, x_mm: int, y_mm: int) -> float:
    angle = math.radians(route.goal_heading_mdeg / 1_000.0)
    dx = x_mm - route.goal_origin_x_mm
    dy = y_mm - route.goal_origin_y_mm
    return dx * math.cos(angle) + dy * math.sin(angle)


def _finite_number(value) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _evidence_rays(scan):
    angular = scan.get("angular_rays") if isinstance(scan, Mapping) else None
    return angular if isinstance(angular, list) else scan.get("rays", [])


def _projection(view):
    if not isinstance(view, Mapping):
        raise ValueError("BLAST detour scan view is invalid")
    projection = view.get("planar_projection")
    if not (
        isinstance(projection, Mapping)
        and projection.get("schema") == "blast-planar-scan-projection/v1"
        and projection.get("frame") == "EPISODE_LOCAL_ODOMETRY"
        and projection.get("quality") == "PROVISIONAL_YAW_ONLY"
        and isinstance(projection.get("points"), list)
    ):
        raise ValueError("BLAST detour projection is invalid")
    return projection


def _points(view):
    values = []
    for point in _projection(view)["points"]:
        if not isinstance(point, Mapping):
            raise ValueError("BLAST detour projection point is invalid")
        x_mm = point.get("nominal_echo_x_mm")
        y_mm = point.get("nominal_echo_y_mm")
        if type(x_mm) is not int or type(y_mm) is not int:
            raise ValueError("BLAST detour projection point is invalid")
        values.append((point.get("side"), x_mm, y_mm, point))
    return values


def _center(values):
    centers = [value for value in values if value[0] == "center"]
    if len(centers) != 1:
        raise ValueError("BLAST detour center echo is unavailable")
    return centers[0]


def _side_radius(mission, origin_view, selected_side):
    values = _points(origin_view)
    center = _center(values)
    center_longitudinal, center_lateral = _axes(
        mission, center[1], center[2]
    )
    sign = 1 if selected_side == "LEFT" else -1
    prefix = "left_" if selected_side == "LEFT" else "right_"
    reaches = []
    for side, x_mm, y_mm, _point in values:
        longitudinal, lateral = _axes(mission, x_mm, y_mm)
        reach = sign * (lateral - center_lateral)
        if (
            isinstance(side, str)
            and side.startswith(prefix)
            and abs(longitudinal - center_longitudinal) <= _DEPTH_BAND_MM
            and reach > 0
        ):
            reaches.append(reach)
    if not reaches:
        raise ValueError("BLAST selected-side target extent is unavailable")
    radius = math.ceil(max(reaches))
    if not 1 <= radius <= _MAX_PROVISIONAL_RADIUS_MM:
        raise ValueError("BLAST provisional target envelope is invalid")
    return center, radius


def _side_view_covers_pass(
    mission, side_view, route, current_pose, side_waypoint,
):
    scan_pose = side_view.get("scan_pose") if isinstance(
        side_view, Mapping
    ) else None
    if scan_pose != current_pose.to_dict():
        return False
    if abs(normalize_heading_mdeg(
        current_pose.heading_mdeg - mission.reference_heading_mdeg
    )) > route.heading_tolerance_mdeg:
        return False
    selected_left = route.detour_side == "LEFT_OF_GOAL"
    if blast_side_view_associates_frozen_target(
        side_view,
        route,
        "LEFT" if selected_left else "RIGHT",
    ):
        return True
    if _target_side_no_return_search(
        side_view, "LEFT" if selected_left else "RIGHT"
    ):
        target_x = side_waypoint.get("target_x_mm")
        target_y = side_waypoint.get("target_y_mm")
        tolerance = side_waypoint.get("position_tolerance_mm")
        return (
            type(target_x) is int
            and type(target_y) is int
            and type(tolerance) is int
            and math.hypot(
                current_pose.x_mm - target_x,
                current_pose.y_mm - target_y,
            ) <= tolerance
        )
    values = _points(side_view)
    center = _center(values)
    echo_longitudinal, _echo_lateral = _axes(
        mission, center[1], center[2]
    )
    footprint = route.inflated_pass_clearance_mm
    if echo_longitudinal < route.pass_longitudinal_offset_mm + footprint:
        return False

    if target_side_has_only_settled_no_return(
        side_view, "LEFT" if selected_left else "RIGHT"
    ):
        return True
    if _target_side_mixed_far_view(
        side_view,
        route,
        "LEFT" if selected_left else "RIGHT",
    ):
        return True

    # The straight-ahead ray can pass beside a wider obstacle while BLAST's
    # inner body edge still overlaps it.  Require one measured ray aimed back
    # toward the goal axis to traverse the inferred front plane and continue
    # beyond the pass plane.  This is intentionally a provisional corridor
    # sample, not an object-boundary or free-space claim.
    merge_prefix = "right_" if selected_left else "left_"
    current_lateral = mission.lateral_offset_mm(current_pose)
    robot_footprint, _sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    )
    inner_body_lateral = current_lateral + (
        -robot_footprint.right_extent_mm
        if selected_left
        else robot_footprint.left_extent_mm
    )
    corridor_clear_through = (
        route.pass_longitudinal_offset_mm
        + PASS_BUFFER_MM
        + max(
            robot_footprint.left_extent_mm,
            robot_footprint.right_extent_mm,
        )
        + robot_footprint.clearance_margin_mm
    )
    target_longitudinal, _target_lateral = _axes(
        mission, route.target_centroid_x_mm, route.target_centroid_y_mm,
    )
    for side, echo_x, echo_y, point in values:
        if not (isinstance(side, str) and side.startswith(merge_prefix)):
            continue
        origin_x = point.get("sensor_origin_x_mm")
        origin_y = point.get("sensor_origin_y_mm")
        if type(origin_x) is not int or type(origin_y) is not int:
            continue
        origin_longitudinal, origin_lateral = _axes(
            mission, origin_x, origin_y,
        )
        ray_longitudinal, ray_lateral = _axes(
            mission, echo_x, echo_y,
        )
        span = ray_longitudinal - origin_longitudinal
        if (
            span <= 0
            or ray_longitudinal < corridor_clear_through
            or not origin_longitudinal < target_longitudinal < ray_longitudinal
        ):
            continue
        crossing_lateral = origin_lateral + (
            (target_longitudinal - origin_longitudinal)
            / span
            * (ray_lateral - origin_lateral)
        )
        if (
            selected_left and crossing_lateral <= inner_body_lateral
        ) or (
            not selected_left and crossing_lateral >= inner_body_lateral
        ):
            return True
    return False


def _target_side_mixed_far_view(view, route, selected_side):
    """Accept one far background echo plus one settled angled no-return."""

    try:
        scan = validate_blast_scan_ray_contract(view.get("scan"))
        values = _points(view)
    except (AttributeError, ValueError):
        return False
    prefix = "right_" if selected_side == "LEFT" else "left_"
    rays = {
        ray["side"]: ray for ray in _evidence_rays(scan)
        if ray["side"].startswith(prefix)
    }
    dense = "angular_rays" in scan
    near_side = f"{prefix}2" if dense else f"{prefix}near"
    far_side = f"{prefix}4" if dense else f"{prefix}far"
    near = rays.get(near_side)
    far = rays.get(far_side)
    def merge_ray_clear(ray):
        if not isinstance(ray, Mapping) or ray.get(
            "observation_settled"
        ) is not True:
            return False
        state = ray.get("range_state")
        distance = ray.get("distance_mm")
        if not _finite_number(distance):
            return False
        if state == RANGE_STATE_MEASURED:
            return float(distance) > route.inflated_pass_clearance_mm
        return (
            state == RANGE_STATE_NO_VALID_DISTANCE
            and float(distance) == 2_000.0
        )
    if not (
        isinstance(near, Mapping)
        and isinstance(far, Mapping)
        and len(rays) == (4 if dense else 2)
        and all(merge_ray_clear(ray) for ray in rays.values())
        and near.get("observation_settled") is True
        and near.get("range_state") == RANGE_STATE_MEASURED
        and float(near["distance_mm"]) > route.inflated_pass_clearance_mm
        and far.get("observation_settled") is True
        and far.get("range_state") == RANGE_STATE_NO_VALID_DISTANCE
        and float(far["distance_mm"]) == 2_000.0
    ):
        return False
    points = [value for value in values if value[0] == near_side]
    if len(points) != 1:
        return False
    _side, echo_x, echo_y, point = points[0]
    point_range = point.get("measured_range_mm")
    if not _finite_number(point_range) or float(point_range) != float(
        near["distance_mm"]
    ):
        return False
    echo_longitudinal = _route_longitudinal(route, echo_x, echo_y)
    target_distance = math.hypot(
        echo_x - route.target_centroid_x_mm,
        echo_y - route.target_centroid_y_mm,
    )
    return (
        echo_longitudinal >= route.pass_longitudinal_offset_mm
        and target_distance
        > route.target_radius_mm + route.position_tolerance_mm
    )


def blast_side_view_associates_frozen_target(
    view, route, selected_side,
):
    """Recognize one dense restored side view tied to the frozen envelope.

    No-return remains non-free-space evidence.  This narrow case only binds
    the new view to the remembered object: center and every outward ray must
    be settled no-return while all four inward rays are settled measurements,
    with at least two distinct echoes intersecting the frozen envelope.
    """

    try:
        scan = validate_blast_scan_ray_contract(view.get("scan"))
        values = _points(view)
    except (AttributeError, ValueError):
        return False
    angular = scan.get("angular_rays")
    if not (
        isinstance(angular, list)
        and len(angular) == 9
        and scan.get("state") == "complete"
        and scan.get("result") == "restored"
        and scan.get("restoration_verified") is True
        and scan.get("all_observations_settled") is True
        and all(ray.get("observation_settled") is True for ray in angular)
    ):
        return False
    outward_prefix = "left_" if selected_side == "LEFT" else "right_"
    inward_prefix = "right_" if selected_side == "LEFT" else "left_"
    center = [ray for ray in angular if ray["side"] == "center"]
    outward = [
        ray for ray in angular if ray["side"].startswith(outward_prefix)
    ]
    inward = [
        ray for ray in angular if ray["side"].startswith(inward_prefix)
    ]

    def exact_no_return(ray):
        return (
            ray.get("range_state") == RANGE_STATE_NO_VALID_DISTANCE
            and _finite_number(ray.get("distance_mm"))
            and float(ray["distance_mm"]) == 2_000.0
        )

    rotation_clearance = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .minimum_rotation_clearance_mm()
    )
    if not (
        len(center) == 1
        and len(outward) == 4
        and len(inward) == 4
        and exact_no_return(center[0])
        and all(exact_no_return(ray) for ray in outward)
        and all(
            ray.get("range_state") == RANGE_STATE_MEASURED
            and _finite_number(ray.get("distance_mm"))
            and float(ray["distance_mm"]) > rotation_clearance
            for ray in inward
        )
    ):
        return False
    inward_by_side = {ray["side"]: ray for ray in inward}
    point_by_side = {}
    for side, echo_x, echo_y, point in values:
        if not isinstance(side, str) or side in point_by_side:
            return False
        point_by_side[side] = (echo_x, echo_y, point)
    if set(point_by_side) != set(inward_by_side):
        return False
    associated = set()
    for side, ray in inward_by_side.items():
        echo_x, echo_y, point = point_by_side[side]
        point_range = point.get("measured_range_mm")
        if not _finite_number(point_range) or float(point_range) != float(
            ray["distance_mm"]
        ):
            return False
        if math.hypot(
            echo_x - route.target_centroid_x_mm,
            echo_y - route.target_centroid_y_mm,
        ) <= route.target_radius_mm + route.position_tolerance_mm:
            associated.add((echo_x, echo_y))
    return len(associated) >= 2


def _target_side_no_return_search(view, selected_side):
    """Recognize a restored search view with no stable target return."""

    try:
        scan = validate_blast_scan_ray_contract(view.get("scan"))
        projected_sides = {value[0] for value in _points(view)}
    except (AttributeError, ValueError):
        return False
    if not (
        scan.get("state") == "complete"
        and scan.get("result") == "restored"
        and scan.get("restoration_verified") is True
    ):
        return False
    rotation_clearance = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .minimum_rotation_clearance_mm()
    )
    evidence_rays = _evidence_rays(scan)
    if any(
        ray.get("range_state") == RANGE_STATE_INVALID
        or (
            ray.get("range_state") == RANGE_STATE_MEASURED
            and (
                not _finite_number(ray.get("distance_mm"))
                or float(ray["distance_mm"]) <= rotation_clearance
            )
        )
        for ray in evidence_rays
    ):
        return False
    prefix = "right_" if selected_side == "LEFT" else "left_"
    center = [ray for ray in evidence_rays if ray["side"] == "center"]
    target = [
        ray for ray in evidence_rays if ray["side"].startswith(prefix)
    ]

    def settled_no_return(ray):
        return (
            ray.get("observation_settled") is True
            and ray.get("range_state") == RANGE_STATE_NO_VALID_DISTANCE
            and float(ray["distance_mm"]) == 2_000.0
        )

    def unresolved_without_close(ray):
        if settled_no_return(ray):
            return True
        distance = ray.get("distance_mm")
        return (
            ray.get("observation_settled") is False
            and ray.get("evidence_use") == SCAN_RAY_EVIDENCE_SWEEP_ONLY
            and (
                ray.get("range_state") == RANGE_STATE_NO_VALID_DISTANCE
                or (
                    ray.get("range_state") == RANGE_STATE_MEASURED
                    and _finite_number(distance)
                    and float(distance) > rotation_clearance
                )
            )
        )

    return (
        len(center) == 1
        and settled_no_return(center[0])
        and len(target) == (4 if "angular_rays" in scan else 2)
        and projected_sides <= {ray["side"] for ray in evidence_rays}
        and "center" not in projected_sides
        and not any(side.startswith(prefix) for side in projected_sides)
        and any(settled_no_return(ray) for ray in target)
        and all(unresolved_without_close(ray) for ray in target)
    )


def _no_return_pass_pose_matches(view, route):
    scan_pose = view.get("scan_pose") if isinstance(view, Mapping) else None
    try:
        pose = PhysicalPose.from_mapping(scan_pose)
    except (TypeError, ValueError):
        return False
    active = route.active_waypoint
    if active is None or active.kind != MERGE_GOAL_AXIS:
        return False
    longitudinal = _route_longitudinal(route, pose.x_mm, pose.y_mm)
    lateral = _route_lateral(route, pose.x_mm, pose.y_mm)
    side_sign = 1 if route.detour_side == "LEFT_OF_GOAL" else -1
    return (
        longitudinal >= (
            route.pass_longitudinal_offset_mm
            + PASS_BUFFER_MM
            - route.position_tolerance_mm
        )
        and side_sign * (lateral - route.route_lateral_offset_mm)
        >= -route.position_tolerance_mm
        and abs(normalize_heading_mdeg(
            pose.heading_mdeg - route.goal_heading_mdeg
        )) <= route.heading_tolerance_mdeg
    )


def bind_blast_detour_route(
    *,
    origin_view: Mapping[str, object],
    side_view: Mapping[str, object],
    selected_side: str,
    side_waypoint: Mapping[str, object],
    mission: DirectionalMission,
    current_pose: PhysicalPose,
):
    """Return a shared route admitted by two settled BLAST viewpoints."""

    if (
        selected_side not in ("LEFT", "RIGHT")
        or not isinstance(mission, DirectionalMission)
        or not isinstance(current_pose, PhysicalPose)
        or not isinstance(side_waypoint, Mapping)
        or side_waypoint.get("search_basis")
        != "PROVISIONAL_SAME_DEPTH_ECHO_REACH"
        or side_waypoint.get("search_target_capped") is not False
    ):
        raise ValueError("BLAST detour admission evidence is invalid")
    origin_pose = origin_view.get("scan_pose") if isinstance(
        origin_view, Mapping
    ) else None
    try:
        detour_origin_pose = PhysicalPose.from_mapping(origin_pose)
    except (TypeError, ValueError):
        raise ValueError("BLAST detour origin frame is invalid") from None
    origin_progress = mission.longitudinal_progress_mm(detour_origin_pose)
    if (
        side_waypoint.get("selected_side") != selected_side
        or side_waypoint.get("origin_pose") != origin_pose
        or not mission.heading_aligned(detour_origin_pose)
        or abs(mission.lateral_offset_mm(detour_origin_pose))
        > _ORIGIN_LATERAL_TOLERANCE_MM
        or not 0 <= origin_progress < mission.minimum_forward_progress_mm
    ):
        raise ValueError("BLAST detour origin frame is invalid")
    center, radius = _side_radius(mission, origin_view, selected_side)
    footprint, _sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    )
    centroid = (center[1], center[2])
    snapshot = LocalDetourCollisionSnapshot(
        frame_id="EPISODE_LOCAL_ODOMETRY",
        map_generation_id=mission.episode_id,
        map_version=2,
        target_hypothesis_id="blast-provisional-two-view-target",
        target_centroid_x_mm=centroid[0],
        target_centroid_y_mm=centroid[1],
        target_envelope_radius_mm=radius,
        target_support_points=(centroid,),
        robot_footprint=footprint,
        lateral_clearance_margin_mm=0,
    )
    route = build_local_detour_route_from_collision_snapshot(
        snapshot,
        current_pose=detour_origin_pose,
        mission=mission,
        detour_side=(
            "LEFT_OF_GOAL" if selected_side == "LEFT" else "RIGHT_OF_GOAL"
        ),
    )
    if not _side_view_covers_pass(
        mission, side_view, route, current_pose, side_waypoint,
    ):
        raise ValueError("BLAST side view does not cover the pass waypoint")
    lateral = mission.lateral_offset_mm(current_pose)
    if abs(lateral - route.route_lateral_offset_mm) > route.position_tolerance_mm:
        raise ValueError("BLAST side pose does not match the detour route")
    return route.advance_reached(current_pose)


def blast_detour_guidance(route, pose, available_actions):
    feasibility = {
        action: {"allowed": action in available_actions}
        for action in (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90)
    }
    return derive_local_detour_guidance(
        route,
        current_pose=pose,
        motion_feasibility=feasibility,
        action_specs=BLAST_NAVIGATION_ACTION_SPECS,
        odometry_calibration=BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
    )


def blast_detour_action_sweep_is_clear(route, pose, action):
    if action not in (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90):
        return False
    _nominal, maximum = nominal_effect(
        pose,
        action,
        BLAST_NAVIGATION_ACTION_SPECS,
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
    )
    footprint, _sensor = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    _start_intersects, sweep_intersects = footprint_sweep_intersects(
        obstacle_x_mm=route.target_centroid_x_mm,
        obstacle_y_mm=route.target_centroid_y_mm,
        obstacle_radius_mm=route.target_radius_mm,
        start=pose,
        end=maximum,
        footprint=footprint,
    )
    return not sweep_intersects


def blast_detour_scan_sweep_is_clear(route, pose):
    """Conservatively cover both two-pulse scan excursions."""

    footprint, _sensor = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    for action in (TURN_LEFT_90, TURN_RIGHT_90):
        maximum = blast_scan_turn_maximum_pose(pose, action)
        _start_intersects, sweep_intersects = footprint_sweep_intersects(
            obstacle_x_mm=route.target_centroid_x_mm,
            obstacle_y_mm=route.target_centroid_y_mm,
            obstacle_radius_mm=route.target_radius_mm,
            start=pose,
            end=maximum,
            footprint=footprint,
        )
        if sweep_intersects:
            return False
    return True


def blast_detour_needs_pass_buffer(route, pose):
    fresh = route.advance_reached(pose)
    if (
        fresh.status == ROUTE_COMPLETE
        or fresh.active_waypoint.kind != MERGE_GOAL_AXIS
    ):
        return False
    angle = math.radians(fresh.goal_heading_mdeg / 1_000.0)
    longitudinal = (
        (pose.x_mm - fresh.goal_origin_x_mm) * math.cos(angle)
        + (pose.y_mm - fresh.goal_origin_y_mm) * math.sin(angle)
    )
    return longitudinal < fresh.pass_longitudinal_offset_mm + PASS_BUFFER_MM


def blast_detour_scan_allows_progress(
    view, *, role, selected_side, minimum_clearance_mm, route=None,
):
    scan = view.get("scan") if isinstance(view, Mapping) else None
    rays = _evidence_rays(scan) if isinstance(scan, Mapping) else None
    if (
        role not in ("PASS", "FINAL")
        or selected_side not in ("LEFT", "RIGHT")
        or type(minimum_clearance_mm) is not int
        or minimum_clearance_mm < 1
        or not isinstance(rays, list)
    ):
        return False
    if (
        role == "PASS"
        and route is not None
        and _target_side_no_return_search(view, selected_side)
        and _no_return_pass_pose_matches(view, route)
    ):
        return True
    try:
        _center(_points(view))
    except ValueError:
        return False

    def measured(ray):
        distance = ray.get("distance_mm") if isinstance(ray, Mapping) else None
        return (
            isinstance(distance, (int, float))
            and not isinstance(distance, bool)
            and math.isfinite(float(distance))
            and ray.get("range_state") == RANGE_STATE_MEASURED
        )

    centers = [
        ray for ray in rays
        if isinstance(ray, Mapping) and ray.get("side") == "center"
    ]
    if (
        len(centers) != 1
        or not measured(centers[0])
        or float(centers[0]["distance_mm"]) <= minimum_clearance_mm
    ):
        return False
    if role == "FINAL":
        return True
    if route is None:
        return False
    if target_side_has_only_settled_no_return(view, selected_side):
        return True
    if _target_side_mixed_far_view(
        view,
        route,
        selected_side,
    ):
        return True
    merge_prefix = "right_" if selected_side == "LEFT" else "left_"
    merge_rays = [
        ray for ray in rays
        if isinstance(ray, Mapping)
        and str(ray.get("side", "")).startswith(merge_prefix)
        and measured(ray)
    ]
    if not merge_rays or any(
        float(ray["distance_mm"]) <= minimum_clearance_mm
        for ray in merge_rays
    ):
        return False
    measured_sides = {ray["side"] for ray in merge_rays}
    merge_points = [
        point for point in _projection(view)["points"]
        if isinstance(point, Mapping)
        and str(point.get("side", "")).startswith(merge_prefix)
        and point.get("side") in measured_sides
        and _finite_number(point.get("measured_range_mm"))
        and float(point["measured_range_mm"]) > minimum_clearance_mm
    ]
    footprint, _sensor = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    far_extent = (
        footprint.right_extent_mm
        if selected_side == "LEFT"
        else footprint.left_extent_mm
    ) + footprint.clearance_margin_mm
    for point in merge_points:
        lateral = _route_lateral(
            route,
            point["nominal_echo_x_mm"],
            point["nominal_echo_y_mm"],
        )
        if (
            selected_side == "LEFT" and lateral <= -far_extent
        ) or (
            selected_side == "RIGHT" and lateral >= far_extent
        ):
            return True
    return False


def blast_detour_required_slots(route, pose, mission=None):
    """Count nominal motion slots plus one pass-plane verification scan."""

    slots = 0
    pass_scan_complete = False
    current_route = route
    current_pose = pose
    for _unused in range(_MAX_ROUTE_STEPS):
        guidance = blast_detour_guidance(
            current_route,
            current_pose,
            (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90),
        )
        current_route = guidance.route
        if current_route.status == ROUTE_COMPLETE:
            if (
                isinstance(mission, DirectionalMission)
                and mission.longitudinal_progress_mm(current_pose)
                < mission.minimum_forward_progress_mm
            ):
                current_pose = nominal_effect(
                    current_pose,
                    ADVANCE,
                    BLAST_NAVIGATION_ACTION_SPECS,
                    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
                )[0]
                slots += 1
                continue
            return slots + 1
        if (
            not pass_scan_complete
            and blast_detour_needs_pass_buffer(current_route, current_pose)
        ):
            action = ADVANCE
        elif (
            current_route.active_waypoint.kind == MERGE_GOAL_AXIS
            and not pass_scan_complete
        ):
            slots += 1
            pass_scan_complete = True
            continue
        else:
            actions = guidance.allowed_motion_actions
            if actions is None or len(actions) != 1:
                raise ValueError("BLAST detour has no unique nominal progress")
            action = next(iter(actions))
        current_pose = nominal_effect(
            current_pose,
            action,
            BLAST_NAVIGATION_ACTION_SPECS,
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
        )[0]
        slots += 1
    raise ValueError("BLAST detour exceeds its bounded action budget")


__all__ = (
    "PASS_BUFFER_MM",
    "bind_blast_detour_route",
    "blast_detour_action_sweep_is_clear",
    "blast_detour_guidance",
    "blast_detour_needs_pass_buffer",
    "blast_detour_required_slots",
    "blast_detour_scan_allows_progress",
    "blast_detour_scan_sweep_is_clear",
    "blast_side_view_associates_frozen_target",
)
