"""Persistent rectilinear local routes for committed EV3 detours."""

from dataclasses import dataclass, replace
import hashlib
import json
import math
from types import SimpleNamespace
from typing import Mapping, Optional, Tuple

from .maneuver_commitment import (
    DETOUR_SIDES,
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
)
from .physical_footprint import RobotFootprint
from .physical_odometry import PhysicalPose, normalize_heading_mdeg


ROUTE_SCHEMA = "robot-local-detour-route/v1"
ROUTE_ACTIVE = "ACTIVE"
ROUTE_COMPLETE = "COMPLETE"
ROUTE_INVALID = "INVALID"
ROUTE_STATUSES = frozenset((ROUTE_ACTIVE, ROUTE_COMPLETE, ROUTE_INVALID))

LATERAL_CLEARANCE = "LATERAL_CLEARANCE"
REACQUIRE_GOAL_HEADING = "REACQUIRE_GOAL_HEADING"
PASS_BEYOND_TARGET = "PASS_BEYOND_TARGET"
MERGE_GOAL_AXIS = "MERGE_GOAL_AXIS"
RESUME_GOAL_HEADING = "RESUME_GOAL_HEADING"
WAYPOINT_KINDS = frozenset((
    LATERAL_CLEARANCE,
    REACQUIRE_GOAL_HEADING,
    PASS_BEYOND_TARGET,
    MERGE_GOAL_AXIS,
    RESUME_GOAL_HEADING,
))

INVALIDATION_REASONS = frozenset((
    "FRAME_MISMATCH",
    "MAP_GENERATION_MISMATCH",
    "MAP_VERSION_REGRESSION",
    "TARGET_MISSING",
    "TARGET_ID_MISMATCH",
    "TARGET_GEOMETRY_MISMATCH",
))


class LocalDetourRouteError(ValueError):
    pass


def _identifier(value, name):
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 128
        or any(ord(character) < 32 for character in value)
    ):
        raise LocalDetourRouteError("{} is invalid".format(name))
    return value


def _integer(value, name, *, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise LocalDetourRouteError("{} is invalid".format(name))
    if minimum is not None and value < minimum:
        raise LocalDetourRouteError("{} is invalid".format(name))
    if maximum is not None and value > maximum:
        raise LocalDetourRouteError("{} is invalid".format(name))
    return value


def _normalized_support_points(
    values,
    *,
    centroid_x_mm: int,
    centroid_y_mm: int,
) -> Tuple[Tuple[int, int], ...]:
    if values is None:
        values = ()
    if not isinstance(values, (tuple, list)):
        raise LocalDetourRouteError("target support points are invalid")
    points = [(centroid_x_mm, centroid_y_mm)]
    for value in values:
        if not isinstance(value, (tuple, list)) or len(value) != 2:
            raise LocalDetourRouteError("target support point is invalid")
        points.append((
            _integer(value[0], "target support x"),
            _integer(value[1], "target support y"),
        ))
    normalized = tuple(sorted(set(points)))
    if len(normalized) > 64:
        raise LocalDetourRouteError("too many target support points")
    return normalized


def _target_signature(
    x_mm: int,
    y_mm: int,
    radius_mm: int,
    support_points: Tuple[Tuple[int, int], ...],
) -> str:
    payload = json.dumps(
        [x_mm, y_mm, radius_mm, support_points],
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _world_point(
    longitudinal_mm: float,
    lateral_mm: float,
    *,
    origin_x_mm: int,
    origin_y_mm: int,
    goal_heading_mdeg: int,
) -> Tuple[int, int]:
    angle = math.radians(goal_heading_mdeg / 1_000.0)
    x_mm = (
        origin_x_mm
        + longitudinal_mm * math.cos(angle)
        - lateral_mm * math.sin(angle)
    )
    y_mm = (
        origin_y_mm
        + longitudinal_mm * math.sin(angle)
        + lateral_mm * math.cos(angle)
    )
    return int(round(x_mm)), int(round(y_mm))


def _goal_axes(
    x_mm: int,
    y_mm: int,
    *,
    origin_x_mm: int,
    origin_y_mm: int,
    goal_heading_mdeg: int,
) -> Tuple[int, int]:
    angle = math.radians(goal_heading_mdeg / 1_000.0)
    relative_x = x_mm - origin_x_mm
    relative_y = y_mm - origin_y_mm
    longitudinal = (
        relative_x * math.cos(angle) + relative_y * math.sin(angle)
    )
    lateral = (
        -relative_x * math.sin(angle) + relative_y * math.cos(angle)
    )
    return int(round(longitudinal)), int(round(lateral))


@dataclass(frozen=True)
class LocalDetourWaypoint:
    ordinal: int
    kind: str
    x_mm: int
    y_mm: int
    heading_mdeg: int
    fact_key: Optional[str]

    def __post_init__(self) -> None:
        _integer(self.ordinal, "waypoint ordinal", minimum=0, maximum=8)
        if self.kind not in WAYPOINT_KINDS:
            raise LocalDetourRouteError("waypoint kind is invalid")
        _integer(self.x_mm, "waypoint x")
        _integer(self.y_mm, "waypoint y")
        _integer(
            self.heading_mdeg,
            "waypoint heading",
            minimum=-180_000,
            maximum=179_999,
        )
        if self.fact_key not in (
            None,
            FACT_GOAL_CORRIDOR_CLEAR,
            FACT_GOAL_HEADING_ALIGNED,
            FACT_TARGET_BEHIND,
        ):
            raise LocalDetourRouteError("waypoint fact key is invalid")

    def to_dict(self) -> Mapping[str, object]:
        return {
            "ordinal": self.ordinal,
            "kind": self.kind,
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
            "heading_mdeg": self.heading_mdeg,
            "fact_key": self.fact_key,
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]):
        fields = {
            "ordinal",
            "kind",
            "x_mm",
            "y_mm",
            "heading_mdeg",
            "fact_key",
        }
        if not isinstance(value, dict) or set(value) != fields:
            raise LocalDetourRouteError("waypoint fields are invalid")
        return cls(**value)


def _route_identity_basis(route) -> Mapping[str, object]:
    return {
        "schema": ROUTE_SCHEMA,
        "frame_id": route.frame_id,
        "map_generation_id": route.map_generation_id,
        "based_on_map_version": route.based_on_map_version,
        "target_hypothesis_id": route.target_hypothesis_id,
        "target_geometry_signature": route.target_geometry_signature,
        "target_support_points": route.target_support_points,
        "detour_side": route.detour_side,
        "created_pose": route.created_pose.to_dict(),
        "goal_origin_x_mm": route.goal_origin_x_mm,
        "goal_origin_y_mm": route.goal_origin_y_mm,
        "goal_heading_mdeg": route.goal_heading_mdeg,
        "route_lateral_offset_mm": route.route_lateral_offset_mm,
        "pass_longitudinal_offset_mm": (
            route.pass_longitudinal_offset_mm
        ),
        "inflated_lateral_clearance_mm": (
            route.inflated_lateral_clearance_mm
        ),
        "inflated_pass_clearance_mm": route.inflated_pass_clearance_mm,
        "position_tolerance_mm": route.position_tolerance_mm,
        "heading_tolerance_mdeg": route.heading_tolerance_mdeg,
        "waypoints": [item.to_dict() for item in route.waypoints],
    }


def _route_id(route) -> str:
    encoded = json.dumps(
        _route_identity_basis(route),
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return "local-detour-{}".format(hashlib.sha256(encoded).hexdigest()[:24])


@dataclass(frozen=True)
class LocalDetourRoute:
    route_id: str
    version: int
    status: str
    frame_id: str
    map_generation_id: str
    based_on_map_version: int
    target_hypothesis_id: str
    target_centroid_x_mm: int
    target_centroid_y_mm: int
    target_radius_mm: int
    target_geometry_signature: str
    target_support_points: Tuple[Tuple[int, int], ...]
    detour_side: str
    created_pose: PhysicalPose
    goal_origin_x_mm: int
    goal_origin_y_mm: int
    goal_heading_mdeg: int
    route_lateral_offset_mm: int
    pass_longitudinal_offset_mm: int
    inflated_lateral_clearance_mm: int
    inflated_pass_clearance_mm: int
    position_tolerance_mm: int
    heading_tolerance_mdeg: int
    waypoints: Tuple[LocalDetourWaypoint, ...]
    active_index: int = 0
    invalidation_reason: Optional[str] = None

    def __post_init__(self) -> None:
        _identifier(self.route_id, "route id")
        _identifier(self.frame_id, "frame id")
        _identifier(self.map_generation_id, "map generation id")
        _identifier(self.target_hypothesis_id, "target hypothesis id")
        _integer(self.version, "route version", minimum=1)
        _integer(self.based_on_map_version, "map version", minimum=0)
        for name in (
            "target_centroid_x_mm",
            "target_centroid_y_mm",
            "goal_origin_x_mm",
            "goal_origin_y_mm",
            "route_lateral_offset_mm",
            "pass_longitudinal_offset_mm",
        ):
            _integer(getattr(self, name), name)
        _integer(self.target_radius_mm, "target radius", minimum=1)
        normalized_supports = _normalized_support_points(
            self.target_support_points,
            centroid_x_mm=self.target_centroid_x_mm,
            centroid_y_mm=self.target_centroid_y_mm,
        )
        if self.target_support_points != normalized_supports:
            raise LocalDetourRouteError("target support points are invalid")
        _integer(
            self.goal_heading_mdeg,
            "goal heading",
            minimum=-180_000,
            maximum=179_999,
        )
        _integer(
            self.inflated_lateral_clearance_mm,
            "inflated lateral clearance",
            minimum=1,
        )
        _integer(
            self.inflated_pass_clearance_mm,
            "inflated pass clearance",
            minimum=1,
        )
        _integer(
            self.position_tolerance_mm,
            "position tolerance",
            minimum=1,
            maximum=500,
        )
        _integer(
            self.heading_tolerance_mdeg,
            "heading tolerance",
            minimum=1,
            maximum=45_000,
        )
        if self.status not in ROUTE_STATUSES:
            raise LocalDetourRouteError("route status is invalid")
        if self.detour_side not in DETOUR_SIDES:
            raise LocalDetourRouteError("detour side is invalid")
        if not isinstance(self.created_pose, PhysicalPose):
            raise LocalDetourRouteError("created pose is invalid")
        if (
            not isinstance(self.waypoints, tuple)
            or not 1 <= len(self.waypoints) <= 5
            or any(
                not isinstance(item, LocalDetourWaypoint)
                for item in self.waypoints
            )
            or tuple(item.ordinal for item in self.waypoints)
            != tuple(range(len(self.waypoints)))
        ):
            raise LocalDetourRouteError("route waypoints are invalid")
        _integer(
            self.active_index,
            "active waypoint index",
            minimum=0,
            maximum=len(self.waypoints),
        )
        if (
            self.status == ROUTE_ACTIVE
            and self.active_index >= len(self.waypoints)
        ) or (
            self.status == ROUTE_COMPLETE
            and self.active_index != len(self.waypoints)
        ) or (
            self.status == ROUTE_INVALID
            and self.invalidation_reason not in INVALIDATION_REASONS
        ) or (
            self.status != ROUTE_INVALID
            and self.invalidation_reason is not None
        ):
            raise LocalDetourRouteError("route lifecycle is invalid")
        expected_signature = _target_signature(
            self.target_centroid_x_mm,
            self.target_centroid_y_mm,
            self.target_radius_mm,
            self.target_support_points,
        )
        if self.target_geometry_signature != expected_signature:
            raise LocalDetourRouteError("target geometry signature is invalid")
        if self.route_id != _route_id(self):
            raise LocalDetourRouteError("route identity is invalid")

    @property
    def active_waypoint(self) -> Optional[LocalDetourWaypoint]:
        if self.status != ROUTE_ACTIVE:
            return None
        return self.waypoints[self.active_index]

    def _pose_axes(self, pose: PhysicalPose) -> Tuple[int, int]:
        return _goal_axes(
            pose.x_mm,
            pose.y_mm,
            origin_x_mm=self.goal_origin_x_mm,
            origin_y_mm=self.goal_origin_y_mm,
            goal_heading_mdeg=self.goal_heading_mdeg,
        )

    def _heading_reached(self, pose: PhysicalPose, target: int) -> bool:
        return abs(normalize_heading_mdeg(
            target - pose.heading_mdeg
        )) <= self.heading_tolerance_mdeg

    def _waypoint_reached(
        self,
        waypoint: LocalDetourWaypoint,
        pose: PhysicalPose,
    ) -> bool:
        longitudinal, lateral = self._pose_axes(pose)
        side_sign = 1 if self.detour_side == "LEFT_OF_GOAL" else -1
        side_clear = (
            side_sign * (lateral - self.route_lateral_offset_mm)
            >= -self.position_tolerance_mm
        )
        # The route already carries the position tolerance as clearance.
        # Preserve that reserve until the outward staging leg is complete.
        staging_side_clear = (
            side_sign * (lateral - self.route_lateral_offset_mm) >= 0
        )
        passed = (
            longitudinal
            >= self.pass_longitudinal_offset_mm
            - self.position_tolerance_mm
        )
        merged = (
            -side_sign * lateral >= -self.position_tolerance_mm
        )
        heading_reached = self._heading_reached(
            pose, waypoint.heading_mdeg
        )
        if waypoint.kind == LATERAL_CLEARANCE:
            return staging_side_clear and heading_reached
        if waypoint.kind == REACQUIRE_GOAL_HEADING:
            return side_clear and heading_reached
        if waypoint.kind == PASS_BEYOND_TARGET:
            return side_clear and passed and heading_reached
        if waypoint.kind in (MERGE_GOAL_AXIS, RESUME_GOAL_HEADING):
            return passed and merged and heading_reached
        return False

    def advance_reached(self, pose: PhysicalPose):
        """Advance over every ordered waypoint proven reached by one pose."""

        if not isinstance(pose, PhysicalPose):
            raise LocalDetourRouteError("route advancement pose is invalid")
        if self.status != ROUTE_ACTIVE:
            return self
        active_index = self.active_index
        while (
            active_index < len(self.waypoints)
            and self._waypoint_reached(self.waypoints[active_index], pose)
        ):
            active_index += 1
        if active_index == self.active_index:
            return self
        return replace(
            self,
            version=self.version + active_index - self.active_index,
            active_index=active_index,
            status=(
                ROUTE_COMPLETE
                if active_index == len(self.waypoints)
                else ROUTE_ACTIVE
            ),
        )

    def invalidate(self, reason: str):
        if reason not in INVALIDATION_REASONS:
            raise LocalDetourRouteError("invalidation reason is invalid")
        if self.status == ROUTE_INVALID:
            return self
        return replace(
            self,
            version=self.version + 1,
            status=ROUTE_INVALID,
            invalidation_reason=reason,
        )

    def reconcile(
        self,
        *,
        frame_id: str,
        map_generation_id: str,
        map_version: int,
        target_hypothesis_id: Optional[str],
        target_centroid_x_mm: Optional[int],
        target_centroid_y_mm: Optional[int],
        target_radius_mm: Optional[int],
        target_support_points=None,
    ):
        """Invalidate stale routes without treating normal map growth as stale."""

        _identifier(frame_id, "frame id")
        _identifier(map_generation_id, "map generation id")
        _integer(map_version, "map version", minimum=0)
        if frame_id != self.frame_id:
            return self.invalidate("FRAME_MISMATCH")
        if map_generation_id != self.map_generation_id:
            return self.invalidate("MAP_GENERATION_MISMATCH")
        if map_version < self.based_on_map_version:
            return self.invalidate("MAP_VERSION_REGRESSION")
        if target_hypothesis_id is None:
            return self.invalidate("TARGET_MISSING")
        if target_hypothesis_id != self.target_hypothesis_id:
            return self.invalidate("TARGET_ID_MISMATCH")
        try:
            centroid_x_mm = _integer(
                target_centroid_x_mm, "target centroid x"
            )
            centroid_y_mm = _integer(
                target_centroid_y_mm, "target centroid y"
            )
            signature = _target_signature(
                centroid_x_mm,
                centroid_y_mm,
                _integer(target_radius_mm, "target radius", minimum=1),
                _normalized_support_points(
                    target_support_points,
                    centroid_x_mm=centroid_x_mm,
                    centroid_y_mm=centroid_y_mm,
                ),
            )
        except LocalDetourRouteError:
            return self.invalidate("TARGET_GEOMETRY_MISMATCH")
        if signature != self.target_geometry_signature:
            return self.invalidate("TARGET_GEOMETRY_MISMATCH")
        return self

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": ROUTE_SCHEMA,
            "route_id": self.route_id,
            "version": self.version,
            "status": self.status,
            "frame_id": self.frame_id,
            "map_generation_id": self.map_generation_id,
            "based_on_map_version": self.based_on_map_version,
            "target_hypothesis_id": self.target_hypothesis_id,
            "target_centroid_x_mm": self.target_centroid_x_mm,
            "target_centroid_y_mm": self.target_centroid_y_mm,
            "target_radius_mm": self.target_radius_mm,
            "target_geometry_signature": self.target_geometry_signature,
            "target_support_points": [
                [x_mm, y_mm]
                for x_mm, y_mm in self.target_support_points
            ],
            "detour_side": self.detour_side,
            "created_pose": self.created_pose.to_dict(),
            "goal_origin_x_mm": self.goal_origin_x_mm,
            "goal_origin_y_mm": self.goal_origin_y_mm,
            "goal_heading_mdeg": self.goal_heading_mdeg,
            "route_lateral_offset_mm": self.route_lateral_offset_mm,
            "pass_longitudinal_offset_mm": (
                self.pass_longitudinal_offset_mm
            ),
            "inflated_lateral_clearance_mm": (
                self.inflated_lateral_clearance_mm
            ),
            "inflated_pass_clearance_mm": self.inflated_pass_clearance_mm,
            "position_tolerance_mm": self.position_tolerance_mm,
            "heading_tolerance_mdeg": self.heading_tolerance_mdeg,
            "waypoints": [item.to_dict() for item in self.waypoints],
            "active_index": self.active_index,
            "invalidation_reason": self.invalidation_reason,
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]):
        fields = {
            "schema",
            "route_id",
            "version",
            "status",
            "frame_id",
            "map_generation_id",
            "based_on_map_version",
            "target_hypothesis_id",
            "target_centroid_x_mm",
            "target_centroid_y_mm",
            "target_radius_mm",
            "target_geometry_signature",
            "target_support_points",
            "detour_side",
            "created_pose",
            "goal_origin_x_mm",
            "goal_origin_y_mm",
            "goal_heading_mdeg",
            "route_lateral_offset_mm",
            "pass_longitudinal_offset_mm",
            "inflated_lateral_clearance_mm",
            "inflated_pass_clearance_mm",
            "position_tolerance_mm",
            "heading_tolerance_mdeg",
            "waypoints",
            "active_index",
            "invalidation_reason",
        }
        if (
            not isinstance(value, dict)
            or set(value) != fields
            or value["schema"] != ROUTE_SCHEMA
            or not isinstance(value["waypoints"], list)
        ):
            raise LocalDetourRouteError("route fields are invalid")
        arguments = dict(value)
        del arguments["schema"]
        arguments["created_pose"] = PhysicalPose.from_mapping(
            arguments["created_pose"]
        )
        arguments["target_support_points"] = tuple(
            tuple(item) for item in arguments["target_support_points"]
        )
        arguments["waypoints"] = tuple(
            LocalDetourWaypoint.from_mapping(item)
            for item in arguments["waypoints"]
        )
        return cls(**arguments)


def build_local_detour_route(
    *,
    current_pose: PhysicalPose,
    goal_heading_mdeg: int,
    detour_side: str,
    target_hypothesis_id: str,
    target_centroid_x_mm: int,
    target_centroid_y_mm: int,
    target_radius_mm: int,
    footprint: RobotFootprint,
    frame_id: str,
    map_generation_id: str,
    map_version: int,
    goal_origin_x_mm: Optional[int] = None,
    goal_origin_y_mm: Optional[int] = None,
    position_tolerance_mm: int = 35,
    heading_tolerance_mdeg: int = 20_000,
    lateral_clearance_margin_mm: int = 0,
    target_support_points=(),
) -> LocalDetourRoute:
    """Build a route for an LLM-chosen side without selecting that side."""

    if not isinstance(current_pose, PhysicalPose):
        raise LocalDetourRouteError("current pose is invalid")
    if not isinstance(footprint, RobotFootprint):
        raise LocalDetourRouteError("robot footprint is invalid")
    if detour_side not in DETOUR_SIDES:
        raise LocalDetourRouteError("detour side must be chosen explicitly")
    _identifier(target_hypothesis_id, "target hypothesis id")
    _identifier(frame_id, "frame id")
    _identifier(map_generation_id, "map generation id")
    _integer(map_version, "map version", minimum=0)
    _integer(target_centroid_x_mm, "target centroid x")
    _integer(target_centroid_y_mm, "target centroid y")
    _integer(target_radius_mm, "target radius", minimum=1)
    target_support_points = _normalized_support_points(
        target_support_points,
        centroid_x_mm=target_centroid_x_mm,
        centroid_y_mm=target_centroid_y_mm,
    )
    goal_heading_mdeg = normalize_heading_mdeg(goal_heading_mdeg)
    if goal_origin_x_mm is None:
        goal_origin_x_mm = current_pose.x_mm
    if goal_origin_y_mm is None:
        goal_origin_y_mm = current_pose.y_mm
    _integer(goal_origin_x_mm, "goal origin x")
    _integer(goal_origin_y_mm, "goal origin y")
    _integer(
        position_tolerance_mm,
        "position tolerance",
        minimum=1,
        maximum=500,
    )
    _integer(
        heading_tolerance_mdeg,
        "heading tolerance",
        minimum=1,
        maximum=45_000,
    )
    _integer(
        lateral_clearance_margin_mm,
        "lateral clearance margin",
        minimum=0,
        maximum=500,
    )

    side_sign = 1 if detour_side == "LEFT_OF_GOAL" else -1
    body_side_extent = (
        footprint.right_extent_mm
        if side_sign > 0
        else footprint.left_extent_mm
    )
    lateral_clearance = (
        target_radius_mm
        + body_side_extent
        + footprint.clearance_margin_mm
        + position_tolerance_mm
        + lateral_clearance_margin_mm
    )
    # Match the map's TARGET_ENVELOPE_BEHIND_GOAL_ORIGIN proof.  A route
    # waypoint must carry the complete, potentially asymmetric body beyond
    # the remembered object before it can ask the model to merge back onto
    # the original goal axis.
    pass_clearance = (
        target_radius_mm
        + int(math.ceil(footprint.maximum_corner_radius_mm))
        + footprint.clearance_margin_mm
        + position_tolerance_mm
    )
    current_longitudinal, current_lateral = _goal_axes(
        current_pose.x_mm,
        current_pose.y_mm,
        origin_x_mm=goal_origin_x_mm,
        origin_y_mm=goal_origin_y_mm,
        goal_heading_mdeg=goal_heading_mdeg,
    )
    support_axes = tuple(
        _goal_axes(
            support_x_mm,
            support_y_mm,
            origin_x_mm=goal_origin_x_mm,
            origin_y_mm=goal_origin_y_mm,
            goal_heading_mdeg=goal_heading_mdeg,
        )
        for support_x_mm, support_y_mm in target_support_points
    )
    requested_lateral = (
        max(lateral for _longitudinal, lateral in support_axes)
        + lateral_clearance
        if side_sign > 0
        else min(lateral for _longitudinal, lateral in support_axes)
        - lateral_clearance
    )
    route_lateral = (
        max(current_lateral, requested_lateral)
        if side_sign > 0
        else min(current_lateral, requested_lateral)
    )
    pass_longitudinal = max(
        current_longitudinal,
        max(longitudinal for longitudinal, _lateral in support_axes)
        + pass_clearance,
    )
    side_heading = normalize_heading_mdeg(
        goal_heading_mdeg + side_sign * 90_000
    )
    merge_heading = normalize_heading_mdeg(
        goal_heading_mdeg - side_sign * 90_000
    )
    waypoint_values = []

    def append_waypoint(kind, longitudinal, lateral, heading, fact_key):
        x_mm, y_mm = _world_point(
            longitudinal,
            lateral,
            origin_x_mm=goal_origin_x_mm,
            origin_y_mm=goal_origin_y_mm,
            goal_heading_mdeg=goal_heading_mdeg,
        )
        waypoint_values.append(LocalDetourWaypoint(
            ordinal=len(waypoint_values),
            kind=kind,
            x_mm=x_mm,
            y_mm=y_mm,
            heading_mdeg=heading,
            fact_key=fact_key,
        ))

    lateral_leg_required = (
        side_sign * (route_lateral - current_lateral)
        > 0
    )
    if lateral_leg_required:
        append_waypoint(
            LATERAL_CLEARANCE,
            current_longitudinal,
            route_lateral,
            side_heading,
            FACT_GOAL_CORRIDOR_CLEAR,
        )
        append_waypoint(
            REACQUIRE_GOAL_HEADING,
            current_longitudinal,
            route_lateral,
            goal_heading_mdeg,
            FACT_GOAL_HEADING_ALIGNED,
        )
    append_waypoint(
        PASS_BEYOND_TARGET,
        pass_longitudinal,
        route_lateral,
        goal_heading_mdeg,
        FACT_TARGET_BEHIND,
    )
    if abs(route_lateral) > position_tolerance_mm:
        append_waypoint(
            MERGE_GOAL_AXIS,
            pass_longitudinal,
            0,
            merge_heading,
            None,
        )
        append_waypoint(
            RESUME_GOAL_HEADING,
            pass_longitudinal,
            0,
            goal_heading_mdeg,
            FACT_GOAL_HEADING_ALIGNED,
        )

    route_values = {
        "version": 1,
        "status": ROUTE_ACTIVE,
        "frame_id": frame_id,
        "map_generation_id": map_generation_id,
        "based_on_map_version": map_version,
        "target_hypothesis_id": target_hypothesis_id,
        "target_centroid_x_mm": target_centroid_x_mm,
        "target_centroid_y_mm": target_centroid_y_mm,
        "target_radius_mm": target_radius_mm,
        "target_support_points": target_support_points,
        "target_geometry_signature": _target_signature(
            target_centroid_x_mm,
            target_centroid_y_mm,
            target_radius_mm,
            target_support_points,
        ),
        "detour_side": detour_side,
        "created_pose": current_pose,
        "goal_origin_x_mm": goal_origin_x_mm,
        "goal_origin_y_mm": goal_origin_y_mm,
        "goal_heading_mdeg": goal_heading_mdeg,
        "route_lateral_offset_mm": route_lateral,
        "pass_longitudinal_offset_mm": pass_longitudinal,
        "inflated_lateral_clearance_mm": lateral_clearance,
        "inflated_pass_clearance_mm": pass_clearance,
        "position_tolerance_mm": position_tolerance_mm,
        "heading_tolerance_mdeg": heading_tolerance_mdeg,
        "waypoints": tuple(waypoint_values),
    }
    route_values["route_id"] = _route_id(SimpleNamespace(**route_values))
    return LocalDetourRoute(**route_values)


__all__ = (
    "LATERAL_CLEARANCE",
    "LocalDetourRoute",
    "LocalDetourRouteError",
    "LocalDetourWaypoint",
    "MERGE_GOAL_AXIS",
    "PASS_BEYOND_TARGET",
    "REACQUIRE_GOAL_HEADING",
    "RESUME_GOAL_HEADING",
    "ROUTE_ACTIVE",
    "ROUTE_COMPLETE",
    "ROUTE_INVALID",
    "ROUTE_SCHEMA",
    "build_local_detour_route",
)
