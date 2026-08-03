"""Deterministic execution guidance for a model-chosen local detour.

The language model owns the strategic commitment, including which side of a
remembered obstacle to use.  This module only turns that commitment into a
persistent geometric route and keeps short model-authored motion tails
correlated with the route's current waypoint.
"""

from dataclasses import dataclass
import math
from typing import Mapping, Optional, Tuple

from .local_detour_route import (
    LATERAL_CLEARANCE,
    ROUTE_ACTIVE,
    ROUTE_COMPLETE,
    ROUTE_INVALID,
    LocalDetourRoute,
    build_local_detour_route,
)
from .maneuver_commitment import ActiveManeuver, DETOUR_SIDES
from .physical_navigation_contract import (
    ADVANCE,
    MOTION_ACTIONS,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from .physical_navigation_mission import DirectionalMission
from .physical_odometry import (
    OdometryCalibration,
    PhysicalPose,
    nominal_effect,
    normalize_heading_mdeg,
)
from .provisional_hazard_map import ProvisionalHazardMap


SYNC_INACTIVE = "INACTIVE"
SYNC_BUILT = "BUILT"
SYNC_UNCHANGED = "UNCHANGED"
SYNC_ADVANCED = "ADVANCED"
SYNC_COMPLETED = "COMPLETED"
SYNC_REBUILT = "REBUILT"
SYNC_INVALIDATED = "INVALIDATED"
SYNC_CLEARED = "CLEARED"
SYNC_TARGET_MISSING = "TARGET_MISSING"
SYNC_EVENTS = frozenset((
    SYNC_INACTIVE,
    SYNC_BUILT,
    SYNC_UNCHANGED,
    SYNC_ADVANCED,
    SYNC_COMPLETED,
    SYNC_REBUILT,
    SYNC_INVALIDATED,
    SYNC_CLEARED,
    SYNC_TARGET_MISSING,
))

GUIDANCE_INACTIVE = "INACTIVE"
GUIDANCE_ROUTE_COMPLETE = "ROUTE_COMPLETE"
GUIDANCE_ROUTE_INVALID = "ROUTE_INVALID"
GUIDANCE_TURN_TO_WAYPOINT = "TURN_TO_WAYPOINT"
GUIDANCE_ADVANCE_TO_WAYPOINT = "ADVANCE_TO_WAYPOINT"
GUIDANCE_ADVANCE_FOR_TURN_ROOM = "ADVANCE_FOR_TURN_ROOM"
GUIDANCE_REVERSE_FOR_TURN_ROOM = "REVERSE_FOR_TURN_ROOM"
GUIDANCE_NO_PROGRESS_ACTION = "NO_PROGRESS_ACTION"


class LocalDetourControllerError(ValueError):
    pass


@dataclass(frozen=True)
class LocalDetourRouteSync:
    """Result of reconciling one persisted route with current state."""

    route: Optional[LocalDetourRoute]
    event: str
    previous_route_id: Optional[str] = None
    replaced_route_status: Optional[str] = None
    reason: Optional[str] = None

    def __post_init__(self) -> None:
        if self.event not in SYNC_EVENTS:
            raise LocalDetourControllerError("route sync event is invalid")

    def to_dict(self) -> Mapping[str, object]:
        return {
            "event": self.event,
            "previous_route_id": self.previous_route_id,
            "replaced_route_status": self.replaced_route_status,
            "reason": self.reason,
            "route": None if self.route is None else self.route.to_dict(),
        }


@dataclass(frozen=True)
class LocalDetourGuidance:
    """Motion gate derived from one freshly synchronized route."""

    route: Optional[LocalDetourRoute]
    allowed_motion_actions: Optional[frozenset]
    reason: str
    active_waypoint_index: Optional[int] = None
    active_waypoint_kind: Optional[str] = None
    distance_to_waypoint_mm: Optional[int] = None
    heading_error_mdeg: Optional[int] = None

    @property
    def gate_active(self) -> bool:
        return self.allowed_motion_actions is not None

    def to_dict(self) -> Mapping[str, object]:
        return {
            "gate_active": self.gate_active,
            "reason": self.reason,
            "allowed_motion_actions": (
                None
                if self.allowed_motion_actions is None
                else sorted(self.allowed_motion_actions)
            ),
            "active_waypoint_index": self.active_waypoint_index,
            "active_waypoint_kind": self.active_waypoint_kind,
            "distance_to_waypoint_mm": self.distance_to_waypoint_mm,
            "heading_error_mdeg": self.heading_error_mdeg,
            "route": None if self.route is None else self.route.to_dict(),
        }


def _maneuver_route_choice(active_maneuver) -> Optional[Tuple[str, str]]:
    if active_maneuver is None:
        return None
    if isinstance(active_maneuver, ActiveManeuver):
        target_id = active_maneuver.target_hypothesis_id
        detour_side = active_maneuver.detour_side
    elif isinstance(active_maneuver, Mapping):
        target_id = active_maneuver.get("target_hypothesis_id")
        detour_side = active_maneuver.get("detour_side")
    else:
        raise LocalDetourControllerError("active maneuver is invalid")
    if (
        not isinstance(target_id, str)
        or not target_id
        or detour_side not in DETOUR_SIDES
    ):
        raise LocalDetourControllerError(
            "active maneuver route choice is invalid"
        )
    return target_id, detour_side


def _route_target_geometry(
    hazard_map: ProvisionalHazardMap,
    target_id: str,
):
    target = hazard_map.get(target_id)
    group = hazard_map.active_collision_group(target_id)
    if target is None or not group:
        return None
    support_points = tuple(dict.fromkeys(
        point
        for hazard in group
        for point in hazard_map.active_collision_support_points(
            hazard.hypothesis_id
        )
    ))
    if not support_points:
        return None
    return target, max(hazard.radius_mm for hazard in group), support_points


def _build_route(
    *,
    current_pose: PhysicalPose,
    mission: DirectionalMission,
    hazard_map: ProvisionalHazardMap,
    target_id: str,
    detour_side: str,
    position_tolerance_mm: int,
    heading_tolerance_mdeg: int,
) -> Optional[LocalDetourRoute]:
    geometry = _route_target_geometry(hazard_map, target_id)
    if geometry is None:
        return None
    hazard, target_radius_mm, target_support_points = geometry
    footprint = hazard_map.calibration.robot_footprint
    if footprint is None:
        raise LocalDetourControllerError(
            "local detour routes require a calibrated robot footprint"
        )
    return build_local_detour_route(
        current_pose=current_pose,
        goal_heading_mdeg=mission.reference_heading_mdeg,
        detour_side=detour_side,
        target_hypothesis_id=target_id,
        target_centroid_x_mm=hazard.centroid_x_mm,
        target_centroid_y_mm=hazard.centroid_y_mm,
        target_radius_mm=target_radius_mm,
        target_support_points=target_support_points,
        footprint=footprint,
        frame_id=hazard_map.frame_id,
        map_generation_id=hazard_map.map_generation_id,
        map_version=hazard_map.revision,
        goal_origin_x_mm=mission.origin_x_mm,
        goal_origin_y_mm=mission.origin_y_mm,
        position_tolerance_mm=position_tolerance_mm,
        heading_tolerance_mdeg=heading_tolerance_mdeg,
    ).advance_reached(current_pose)


def synchronize_local_detour_route(
    route: Optional[LocalDetourRoute],
    *,
    active_maneuver,
    current_pose: PhysicalPose,
    mission: DirectionalMission,
    hazard_map: ProvisionalHazardMap,
    position_tolerance_mm: int = 35,
    heading_tolerance_mdeg: int = 20_000,
) -> LocalDetourRouteSync:
    """Build, reconcile, advance, or retire a persistent local route.

    A changed target, target geometry, goal frame, or model-authored side
    causes a rebuild from the verified current pose.  Structural frame and
    map-generation changes invalidate the route instead: the directional
    mission's coordinates cannot safely be assumed to belong to a new frame.
    """

    if route is not None and not isinstance(route, LocalDetourRoute):
        raise LocalDetourControllerError("local detour route is invalid")
    if not isinstance(current_pose, PhysicalPose):
        raise LocalDetourControllerError("current pose is invalid")
    if not isinstance(mission, DirectionalMission):
        raise LocalDetourControllerError("directional mission is invalid")
    if not isinstance(hazard_map, ProvisionalHazardMap):
        raise LocalDetourControllerError("hazard map is invalid")

    choice = _maneuver_route_choice(active_maneuver)
    if choice is None:
        if route is None:
            return LocalDetourRouteSync(None, SYNC_INACTIVE)
        return LocalDetourRouteSync(
            None,
            SYNC_CLEARED,
            previous_route_id=route.route_id,
            replaced_route_status=route.status,
            reason="NO_ACTIVE_MANEUVER",
        )
    target_id, detour_side = choice
    geometry = _route_target_geometry(hazard_map, target_id)
    target_active = geometry is not None
    if target_active:
        hazard, target_radius_mm, target_support_points = geometry
    else:
        hazard = None
        target_radius_mm = None
        target_support_points = ()

    if route is None:
        built = _build_route(
            current_pose=current_pose,
            mission=mission,
            hazard_map=hazard_map,
            target_id=target_id,
            detour_side=detour_side,
            position_tolerance_mm=position_tolerance_mm,
            heading_tolerance_mdeg=heading_tolerance_mdeg,
        )
        return LocalDetourRouteSync(
            built,
            SYNC_BUILT if built is not None else SYNC_TARGET_MISSING,
            reason=None if built is not None else "TARGET_MISSING",
        )

    if route.status == ROUTE_INVALID:
        if not target_active or route.invalidation_reason in (
            "FRAME_MISMATCH",
            "MAP_GENERATION_MISMATCH",
            "MAP_VERSION_REGRESSION",
        ):
            return LocalDetourRouteSync(
                route,
                SYNC_INVALIDATED,
                previous_route_id=route.route_id,
                replaced_route_status=route.status,
                reason=route.invalidation_reason,
            )
        rebuilt = _build_route(
            current_pose=current_pose,
            mission=mission,
            hazard_map=hazard_map,
            target_id=target_id,
            detour_side=detour_side,
            position_tolerance_mm=position_tolerance_mm,
            heading_tolerance_mdeg=heading_tolerance_mdeg,
        )
        return LocalDetourRouteSync(
            rebuilt,
            SYNC_REBUILT if rebuilt is not None else SYNC_TARGET_MISSING,
            previous_route_id=route.route_id,
            replaced_route_status=route.status,
            reason="TARGET_ACTIVE_AGAIN",
        )

    reconciled = route.reconcile(
        frame_id=hazard_map.frame_id,
        map_generation_id=hazard_map.map_generation_id,
        map_version=hazard_map.revision,
        target_hypothesis_id=target_id if target_active else None,
        target_centroid_x_mm=(
            None if not target_active else hazard.centroid_x_mm
        ),
        target_centroid_y_mm=(
            None if not target_active else hazard.centroid_y_mm
        ),
        target_radius_mm=target_radius_mm,
        target_support_points=(
            None if not target_active else target_support_points
        ),
    )
    structural_invalidation = (
        reconciled.status == ROUTE_INVALID
        and reconciled.invalidation_reason
        in (
            "FRAME_MISMATCH",
            "MAP_GENERATION_MISMATCH",
            "MAP_VERSION_REGRESSION",
            "TARGET_MISSING",
        )
    )
    if structural_invalidation:
        return LocalDetourRouteSync(
            reconciled,
            SYNC_INVALIDATED,
            previous_route_id=route.route_id,
            replaced_route_status=route.status,
            reason=reconciled.invalidation_reason,
        )

    goal_frame_changed = (
        route.goal_origin_x_mm != mission.origin_x_mm
        or route.goal_origin_y_mm != mission.origin_y_mm
        or route.goal_heading_mdeg != mission.reference_heading_mdeg
    )
    model_route_changed = (
        route.target_hypothesis_id != target_id
        or route.detour_side != detour_side
    )
    geometry_changed = reconciled.status == ROUTE_INVALID
    if goal_frame_changed or model_route_changed or geometry_changed:
        rebuilt = _build_route(
            current_pose=current_pose,
            mission=mission,
            hazard_map=hazard_map,
            target_id=target_id,
            detour_side=detour_side,
            position_tolerance_mm=position_tolerance_mm,
            heading_tolerance_mdeg=heading_tolerance_mdeg,
        )
        reason = (
            "GOAL_FRAME_CHANGED"
            if goal_frame_changed
            else "MODEL_ROUTE_CHANGED"
            if model_route_changed
            else reconciled.invalidation_reason
        )
        return LocalDetourRouteSync(
            rebuilt,
            SYNC_REBUILT if rebuilt is not None else SYNC_TARGET_MISSING,
            previous_route_id=route.route_id,
            replaced_route_status=route.status,
            reason=reason,
        )

    advanced = route.advance_reached(current_pose)
    if advanced is route:
        return LocalDetourRouteSync(route, SYNC_UNCHANGED)
    return LocalDetourRouteSync(
        advanced,
        SYNC_COMPLETED if advanced.status == ROUTE_COMPLETE else SYNC_ADVANCED,
        previous_route_id=route.route_id,
        replaced_route_status=route.status,
        reason="WAYPOINT_REACHED",
    )


def _motion_allowed(
    motion_feasibility: Mapping[str, Mapping[str, object]],
    action: str,
) -> bool:
    value = motion_feasibility.get(action)
    return isinstance(value, Mapping) and value.get("allowed") is True


def _distance_mm(pose: PhysicalPose, x_mm: int, y_mm: int) -> int:
    return int(round(math.hypot(x_mm - pose.x_mm, y_mm - pose.y_mm)))


def _signed_lateral_position_mm(
    route: LocalDetourRoute,
    pose: PhysicalPose,
) -> int:
    angle = math.radians(route.goal_heading_mdeg / 1_000.0)
    relative_x = pose.x_mm - route.goal_origin_x_mm
    relative_y = pose.y_mm - route.goal_origin_y_mm
    lateral = int(round(
        -relative_x * math.sin(angle) + relative_y * math.cos(angle)
    ))
    side_sign = 1 if route.detour_side == "LEFT_OF_GOAL" else -1
    return side_sign * lateral


def _nominal_pose(
    pose: PhysicalPose,
    action: str,
    action_specs: Mapping[str, Mapping[str, object]],
    calibration: OdometryCalibration,
) -> PhysicalPose:
    nominal, _maximum = nominal_effect(
        pose,
        action,
        action_specs,
        calibration,
    )
    return nominal


def _minimum_target_distance_mm(
    route: LocalDetourRoute,
    pose: PhysicalPose,
) -> int:
    return min(
        _distance_mm(pose, x_mm, y_mm)
        for x_mm, y_mm in route.target_support_points
    )


def derive_local_detour_guidance(
    route: Optional[LocalDetourRoute],
    *,
    current_pose: PhysicalPose,
    motion_feasibility: Mapping[str, Mapping[str, object]],
    action_specs: Mapping[str, Mapping[str, object]],
    odometry_calibration: OdometryCalibration = OdometryCalibration(),
) -> LocalDetourGuidance:
    """Derive motion choices that make geometric waypoint progress.

    Non-motion choices are deliberately outside this function.  When a route
    is active it first admits a feasible 90-degree turn that best reduces the
    waypoint heading error, then admits ``ADVANCE`` only when its nominal
    endpoint gets closer to the active waypoint.  If the target is too close
    for either turn, one feasible ``REVERSE`` pulse may instead increase
    geometric clearance until the authorized turn becomes possible.  This
    never chooses the detour side or a motor command.
    """

    if route is not None and not isinstance(route, LocalDetourRoute):
        raise LocalDetourControllerError("local detour route is invalid")
    if not isinstance(current_pose, PhysicalPose):
        raise LocalDetourControllerError("current pose is invalid")
    if not isinstance(motion_feasibility, Mapping):
        raise LocalDetourControllerError("motion feasibility is invalid")
    if not isinstance(action_specs, Mapping):
        raise LocalDetourControllerError("action specs are invalid")
    if route is None:
        return LocalDetourGuidance(
            route=None,
            allowed_motion_actions=None,
            reason=GUIDANCE_INACTIVE,
        )

    fresh_route = route.advance_reached(current_pose)
    if fresh_route.status == ROUTE_COMPLETE:
        return LocalDetourGuidance(
            route=fresh_route,
            allowed_motion_actions=None,
            reason=GUIDANCE_ROUTE_COMPLETE,
        )
    if fresh_route.status == ROUTE_INVALID:
        return LocalDetourGuidance(
            route=fresh_route,
            allowed_motion_actions=frozenset(),
            reason=GUIDANCE_ROUTE_INVALID,
        )

    waypoint = fresh_route.active_waypoint
    heading_error = normalize_heading_mdeg(
        waypoint.heading_mdeg - current_pose.heading_mdeg
    )
    distance = _distance_mm(current_pose, waypoint.x_mm, waypoint.y_mm)
    allowed = frozenset()
    reason = GUIDANCE_NO_PROGRESS_ACTION

    if abs(heading_error) > fresh_route.heading_tolerance_mdeg:
        reducing_turns = {}
        for action in (TURN_LEFT_90, TURN_RIGHT_90):
            if not _motion_allowed(motion_feasibility, action):
                continue
            projected = _nominal_pose(
                current_pose,
                action,
                action_specs,
                odometry_calibration,
            )
            projected_error = abs(normalize_heading_mdeg(
                waypoint.heading_mdeg - projected.heading_mdeg
            ))
            if projected_error < abs(heading_error):
                reducing_turns[action] = projected_error
        if reducing_turns:
            best_error = min(reducing_turns.values())
            allowed = frozenset(
                action
                for action, error in reducing_turns.items()
                if error == best_error
            )
            reason = GUIDANCE_TURN_TO_WAYPOINT
        elif _motion_allowed(motion_feasibility, ADVANCE):
            projected = _nominal_pose(
                current_pose,
                ADVANCE,
                action_specs,
                odometry_calibration,
            )
            projected_distance = _distance_mm(
                projected,
                waypoint.x_mm,
                waypoint.y_mm,
            )
            if projected_distance < distance:
                allowed = frozenset((ADVANCE,))
                reason = GUIDANCE_ADVANCE_FOR_TURN_ROOM
        if (
            not allowed
            and _motion_allowed(motion_feasibility, REVERSE)
        ):
            projected = _nominal_pose(
                current_pose,
                REVERSE,
                action_specs,
                odometry_calibration,
            )
            if _minimum_target_distance_mm(
                fresh_route,
                projected,
            ) > _minimum_target_distance_mm(
                fresh_route,
                current_pose,
            ):
                allowed = frozenset((REVERSE,))
                reason = GUIDANCE_REVERSE_FOR_TURN_ROOM
    elif _motion_allowed(motion_feasibility, ADVANCE):
        projected = _nominal_pose(
            current_pose,
            ADVANCE,
            action_specs,
            odometry_calibration,
        )
        projected_distance = _distance_mm(
            projected,
            waypoint.x_mm,
            waypoint.y_mm,
        )
        side_sign = (
            1 if fresh_route.detour_side == "LEFT_OF_GOAL" else -1
        )
        current_lateral = _signed_lateral_position_mm(
            fresh_route,
            current_pose,
        )
        projected_lateral = _signed_lateral_position_mm(
            fresh_route,
            projected,
        )
        # A coarse pulse can cross the clearance line while its Euclidean
        # distance grows after a long backoff.  Crossing outward is still
        # deterministic progress toward the lateral staging waypoint.
        lateral_staging_progress = (
            waypoint.kind == LATERAL_CLEARANCE
            and current_lateral
            < side_sign * fresh_route.route_lateral_offset_mm
            and projected_lateral > current_lateral
        )
        if projected_distance < distance or lateral_staging_progress:
            allowed = frozenset((ADVANCE,))
            reason = GUIDANCE_ADVANCE_TO_WAYPOINT

    return LocalDetourGuidance(
        route=fresh_route,
        allowed_motion_actions=allowed,
        reason=reason,
        active_waypoint_index=fresh_route.active_index,
        active_waypoint_kind=waypoint.kind,
        distance_to_waypoint_mm=distance,
        heading_error_mdeg=heading_error,
    )


def filter_local_detour_actions(
    available_actions,
    guidance: LocalDetourGuidance,
):
    """Filter only motion actions, retaining the model's non-motion tools."""

    if not isinstance(guidance, LocalDetourGuidance):
        raise LocalDetourControllerError("local detour guidance is invalid")
    actions = tuple(available_actions)
    if guidance.allowed_motion_actions is None:
        return actions
    return tuple(
        action
        for action in actions
        if (
            action not in MOTION_ACTIONS
            or action in guidance.allowed_motion_actions
        )
    )


def local_detour_tail_action_allowed(
    action: str,
    *,
    route: Optional[LocalDetourRoute],
    current_pose: PhysicalPose,
    motion_feasibility: Mapping[str, Mapping[str, object]],
    action_specs: Mapping[str, Mapping[str, object]],
    odometry_calibration: OdometryCalibration = OdometryCalibration(),
) -> bool:
    """Revalidate a queued motion against the freshly reached waypoint."""

    if action not in MOTION_ACTIONS:
        raise LocalDetourControllerError(
            "a local detour plan tail must contain a motion action"
        )
    guidance = derive_local_detour_guidance(
        route,
        current_pose=current_pose,
        motion_feasibility=motion_feasibility,
        action_specs=action_specs,
        odometry_calibration=odometry_calibration,
    )
    if guidance.route is None:
        return True
    return (
        guidance.route.status == ROUTE_ACTIVE
        and guidance.allowed_motion_actions is not None
        and action in guidance.allowed_motion_actions
    )


__all__ = (
    "GUIDANCE_ADVANCE_FOR_TURN_ROOM",
    "GUIDANCE_ADVANCE_TO_WAYPOINT",
    "GUIDANCE_INACTIVE",
    "GUIDANCE_NO_PROGRESS_ACTION",
    "GUIDANCE_ROUTE_COMPLETE",
    "GUIDANCE_ROUTE_INVALID",
    "GUIDANCE_TURN_TO_WAYPOINT",
    "LocalDetourControllerError",
    "LocalDetourGuidance",
    "LocalDetourRouteSync",
    "SYNC_ADVANCED",
    "SYNC_BUILT",
    "SYNC_CLEARED",
    "SYNC_COMPLETED",
    "SYNC_INACTIVE",
    "SYNC_INVALIDATED",
    "SYNC_REBUILT",
    "SYNC_TARGET_MISSING",
    "SYNC_UNCHANGED",
    "derive_local_detour_guidance",
    "filter_local_detour_actions",
    "local_detour_tail_action_allowed",
    "synchronize_local_detour_route",
)
