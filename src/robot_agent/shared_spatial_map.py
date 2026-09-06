"""Read-only projection of robot-local poses into one calibrated world.

This shared-map slice carries robot poses, physical footprints, and detached
navigation diagnostics.  It does not fuse sensor evidence, infer object
identity, or own navigation authority.  A calibration is fenced to the exact
local map generation so a reset robot cannot silently reuse an old fixed-start
pose or navigation trace.
"""

from copy import deepcopy
from typing import Mapping

from .blast_spatial_map import (
    MAX_PROVISIONAL_OBSTACLE_HYPOTHESES,
    NAVIGATION_TRACE_SCHEMA,
    _provisional_obstacle_hypothesis,
    _trace_inputs,
)
from .shared_frame_transform import (
    CalibratedFrameTransform,
    FrameTransformError,
)
from .spatial_map_contract import (
    DASHBOARD_SPATIAL_MAP_SCHEMA,
    LOCAL_ODOMETRY,
    MAX_POSE_HISTORY,
    SpatialCollisionGeometry,
)


SHARED_SPATIAL_MAP_SCHEMA = "robot-spatial-map/v2"
SHARED_FIXED_START = "SHARED_FIXED_START"
LATEST_AVAILABLE_NOT_ATOMIC = "LATEST_AVAILABLE_NOT_ATOMIC"
MAX_SHARED_ROBOTS = 16
MAX_SHARED_POSE_HISTORY = MAX_POSE_HISTORY
_MAX_INT = 2**63 - 1
_NAVIGATION_TRACE_FIELDS = (
    "schema",
    "read_only",
    "frame_id",
    "provenance",
    "final_goal",
    "imu_heading",
    "planned_leg",
    "advisory_waypoint",
    "planar_scan_views",
)
_NAVIGATION_TRACE_ROUTE_FIELD = "local_detour_route"
_NAVIGATION_TRACE_GRID_FIELD = "coarse_grid"


class SharedSpatialMapError(ValueError):
    """Invalid compositor configuration."""


class _SourceUnavailable(ValueError):
    def __init__(self, reason_code: str):
        self.reason_code = reason_code
        super().__init__(reason_code)


def _identifier(name: str, value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 128
        or any(ord(character) < 32 for character in value)
    ):
        raise SharedSpatialMapError("{} is invalid".format(name))
    return value


def _optional_identifier(value: object, maximum: int = 128):
    if value is None:
        return None
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise _SourceUnavailable("source_pose_invalid")
    return value


def _optional_integer(value: object):
    if value is None:
        return None
    if type(value) is not int or not 0 <= value <= _MAX_INT:
        raise _SourceUnavailable("source_pose_invalid")
    return value


def _source_identity(transform: CalibratedFrameTransform):
    return {
        "source_robot_id": transform.source_robot_id,
        "source_controller_id": transform.source_controller_id,
        "source_frame_id": transform.source_frame_id,
        "source_generation_id": transform.source_generation_id,
    }


def _transform_pose(
    value: object,
    transform: CalibratedFrameTransform,
):
    if not isinstance(value, Mapping):
        raise _SourceUnavailable("source_pose_invalid")
    if value.get("frame_id") != transform.source_frame_id:
        raise _SourceUnavailable("source_pose_frame_mismatch")
    try:
        x_mm, y_mm = transform.to_world_point(
            value.get("x_mm"),
            value.get("y_mm"),
            **_source_identity(transform),
        )
        heading_mdeg = transform.to_world_heading(
            value.get("heading_mdeg"),
            **_source_identity(transform),
        )
    except (FrameTransformError, TypeError):
        raise _SourceUnavailable("source_pose_invalid") from None

    result = {
        "x_mm": x_mm,
        "y_mm": y_mm,
        "heading_mdeg": heading_mdeg,
        "frame_id": transform.world_frame_id,
        "local_frame_id": transform.source_frame_id,
        "state_version": _optional_integer(value.get("state_version")),
        "source_id": _optional_identifier(value.get("source_id")),
        "provenance": _optional_identifier(
            value.get("provenance"),
            maximum=512,
        ),
        "observed_at_unix_ms": _optional_integer(
            value.get("observed_at_unix_ms")
        ),
        "age_ms": _optional_integer(value.get("age_ms")),
    }
    if any(
        result[field] is None
        for field in (
            "state_version", "source_id", "provenance",
            "observed_at_unix_ms", "age_ms",
        )
    ):
        raise _SourceUnavailable("source_pose_invalid")
    return result


def _collision_geometry(value: object):
    if value is None:
        return None
    try:
        return SpatialCollisionGeometry.from_mapping(value).to_dict()
    except (TypeError, ValueError):
        raise _SourceUnavailable(
            "source_collision_geometry_invalid"
        ) from None


def _source_scalar(value: object, *, identifier: bool = False):
    if value is None:
        return None
    if identifier:
        return _optional_identifier(value)
    return _optional_integer(value)


def _trace_identifier(value: object, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise _SourceUnavailable("source_navigation_trace_invalid")
    return value


def _transform_trace_point(
    x_mm: object,
    y_mm: object,
    transform: CalibratedFrameTransform,
):
    try:
        return transform.to_world_point(
            x_mm,
            y_mm,
            **_source_identity(transform),
        )
    except (FrameTransformError, TypeError):
        raise _SourceUnavailable(
            "source_navigation_trace_invalid"
        ) from None


def _transform_trace_heading(
    heading_mdeg: object,
    transform: CalibratedFrameTransform,
):
    try:
        return transform.to_world_heading(
            heading_mdeg,
            **_source_identity(transform),
        )
    except (FrameTransformError, TypeError):
        raise _SourceUnavailable(
            "source_navigation_trace_invalid"
        ) from None


def _transform_trace_pose(
    value: Mapping,
    transform: CalibratedFrameTransform,
):
    x_mm, y_mm = _transform_trace_point(
        value["x_mm"], value["y_mm"], transform
    )
    return {
        "x_mm": x_mm,
        "y_mm": y_mm,
        "heading_mdeg": _transform_trace_heading(
            value["heading_mdeg"], transform
        ),
    }


def _transform_provisional_obstacles(
    values: object,
    transform: CalibratedFrameTransform,
    *,
    captured_at_unix_ms: int,
):
    if not isinstance(values, (list, tuple)):
        raise _SourceUnavailable("source_object_hypotheses_invalid")
    selected = [
        value for value in values
        if isinstance(value, Mapping)
        and (
            value.get("classification")
            == "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER"
            or value.get("geometry_kind")
            == "PROVISIONAL_ULTRASONIC_ECHO_CLUSTER"
        )
    ]
    if len(selected) > MAX_PROVISIONAL_OBSTACLE_HYPOTHESES or any(
        not _provisional_obstacle_hypothesis(value, captured_at_unix_ms)
        for value in selected
    ):
        raise _SourceUnavailable("source_object_hypotheses_invalid")
    transformed = []
    for source in selected:
        value = deepcopy(source)
        value["x_mm"], value["y_mm"] = _transform_trace_point(
            source["x_mm"], source["y_mm"], transform
        )
        for point, source_point in zip(
            value["support_points"], source["support_points"]
        ):
            point["x_mm"], point["y_mm"] = _transform_trace_point(
                source_point["x_mm"], source_point["y_mm"], transform
            )
        transformed.append(value)
    return transformed


def _transform_navigation_trace(
    value: object,
    transform: CalibratedFrameTransform,
    *,
    captured_at_unix_ms: int,
):
    if value is None:
        return None
    if (
        not isinstance(value, Mapping)
        or not set(_NAVIGATION_TRACE_FIELDS).issubset(value)
        or set(value) - set(_NAVIGATION_TRACE_FIELDS) - {
            _NAVIGATION_TRACE_ROUTE_FIELD,
            _NAVIGATION_TRACE_GRID_FIELD,
        }
        or value.get("schema") != NAVIGATION_TRACE_SCHEMA
        or value.get("read_only") is not True
    ):
        raise _SourceUnavailable("source_navigation_trace_invalid")
    if value.get("frame_id") != transform.source_frame_id:
        raise _SourceUnavailable(
            "source_navigation_trace_frame_mismatch"
        )
    provenance = _trace_identifier(
        value.get("provenance"), maximum=512
    )
    if not _trace_inputs(
        value.get("final_goal"),
        value.get("planned_leg"),
        value.get("imu_heading"),
        value.get("planar_scan_views"),
        captured_at_unix_ms,
        value.get(_NAVIGATION_TRACE_ROUTE_FIELD),
        value.get("advisory_waypoint"),
        value.get(_NAVIGATION_TRACE_GRID_FIELD),
    ):
        raise _SourceUnavailable("source_navigation_trace_invalid")

    final_goal = dict(value["final_goal"])
    origin_x, origin_y = _transform_trace_point(
        final_goal["origin_x_mm"],
        final_goal["origin_y_mm"],
        transform,
    )
    target_x, target_y = _transform_trace_point(
        final_goal["target_x_mm"],
        final_goal["target_y_mm"],
        transform,
    )
    final_goal.update({
        "origin_x_mm": origin_x,
        "origin_y_mm": origin_y,
        "target_x_mm": target_x,
        "target_y_mm": target_y,
        "desired_heading_mdeg": _transform_trace_heading(
            final_goal["desired_heading_mdeg"], transform
        ),
    })

    imu_heading = value["imu_heading"]
    if imu_heading is not None:
        imu_heading = dict(imu_heading)
        imu_heading["heading_mdeg"] = _transform_trace_heading(
            imu_heading["heading_mdeg"], transform
        )

    planned_leg = value["planned_leg"]
    if planned_leg is not None:
        planned_leg = dict(planned_leg)
        planned_leg["bind_pose"] = _transform_trace_pose(
            planned_leg["bind_pose"], transform
        )
        planned_leg["waypoint"] = _transform_trace_pose(
            planned_leg["waypoint"], transform
        )

    advisory_waypoint = value["advisory_waypoint"]
    if advisory_waypoint is not None:
        advisory_waypoint = dict(advisory_waypoint)
        x_mm, y_mm = _transform_trace_point(
            advisory_waypoint["x_mm"],
            advisory_waypoint["y_mm"],
            transform,
        )
        advisory_waypoint.update({"x_mm": x_mm, "y_mm": y_mm})

    local_detour_route = value.get(_NAVIGATION_TRACE_ROUTE_FIELD)
    if local_detour_route is not None:
        local_detour_route = deepcopy(local_detour_route)
        for waypoint in local_detour_route["waypoints"]:
            x_mm, y_mm = _transform_trace_point(
                waypoint["x_mm"], waypoint["y_mm"], transform
            )
            waypoint.update({
                "x_mm": x_mm,
                "y_mm": y_mm,
                "heading_mdeg": _transform_trace_heading(
                    waypoint["heading_mdeg"], transform
                ),
            })

    planar_scan_views = []
    for source_view in value["planar_scan_views"]:
        view = dict(source_view)
        view["scan_pose"] = _transform_trace_pose(
            source_view["scan_pose"], transform
        )
        source_projection = source_view["projection"]
        projection = dict(source_projection)
        projection["frame"] = SHARED_FIXED_START
        projection["local_frame"] = source_projection["frame"]
        points = []
        for source_point in source_projection["points"]:
            point = dict(source_point)
            origin_x, origin_y = _transform_trace_point(
                source_point["sensor_origin_x_mm"],
                source_point["sensor_origin_y_mm"],
                transform,
            )
            echo_x, echo_y = _transform_trace_point(
                source_point["nominal_echo_x_mm"],
                source_point["nominal_echo_y_mm"],
                transform,
            )
            point.update({
                "sensor_origin_x_mm": origin_x,
                "sensor_origin_y_mm": origin_y,
                "beam_heading_mdeg": _transform_trace_heading(
                    source_point["beam_heading_mdeg"], transform
                ),
                "nominal_echo_x_mm": echo_x,
                "nominal_echo_y_mm": echo_y,
            })
            points.append(point)
        projection["points"] = points
        view["projection"] = projection
        planar_scan_views.append(view)

    result = {
        "schema": value["schema"],
        "read_only": value["read_only"],
        "frame_id": transform.world_frame_id,
        "world_generation_id": transform.world_generation_id,
        "local_frame_id": transform.source_frame_id,
        "local_generation_id": transform.source_generation_id,
        "source_robot_id": transform.source_robot_id,
        "source_controller_instance_id": transform.source_controller_id,
        "provenance": provenance,
        "transform_provenance": list(transform.provenance),
        "final_goal": final_goal,
        "imu_heading": imu_heading,
        "planned_leg": planned_leg,
        "advisory_waypoint": advisory_waypoint,
        "planar_scan_views": planar_scan_views,
    }
    if _NAVIGATION_TRACE_ROUTE_FIELD in value:
        result[_NAVIGATION_TRACE_ROUTE_FIELD] = local_detour_route
    return result


class SharedSpatialMapCompositor:
    """Compose detached local pose snapshots without touching their owners."""

    def __init__(
        self,
        *,
        world_frame_id: str,
        world_generation_id: str,
        bindings: tuple,
    ):
        self.world_frame_id = _identifier(
            "world_frame_id", world_frame_id
        )
        self.world_generation_id = _identifier(
            "world_generation_id", world_generation_id
        )
        if (
            not isinstance(bindings, tuple)
            or len(bindings) > MAX_SHARED_ROBOTS
        ):
            raise SharedSpatialMapError("shared map bindings are invalid")

        checked = []
        provider_ids = set()
        source_ids = set()
        robot_controller_ids = set()
        for binding in bindings:
            if not isinstance(binding, tuple) or len(binding) != 2:
                raise SharedSpatialMapError(
                    "shared map binding is invalid"
                )
            provider, transform = binding
            if not callable(getattr(provider, "snapshot", None)):
                raise SharedSpatialMapError(
                    "shared map provider snapshot is invalid"
                )
            if not isinstance(transform, CalibratedFrameTransform):
                raise SharedSpatialMapError(
                    "shared map frame transform is invalid"
                )
            if (
                transform.world_frame_id != self.world_frame_id
                or transform.world_generation_id
                != self.world_generation_id
            ):
                raise SharedSpatialMapError(
                    "shared map world transform identity changed"
                )
            source_id = (
                transform.source_robot_id,
                transform.source_controller_id,
                transform.source_frame_id,
                transform.source_generation_id,
            )
            robot_controller_id = source_id[:2]
            if (
                id(provider) in provider_ids
                or source_id in source_ids
                or robot_controller_id in robot_controller_ids
            ):
                raise SharedSpatialMapError(
                    "duplicate shared map binding"
                )
            provider_ids.add(id(provider))
            source_ids.add(source_id)
            robot_controller_ids.add(robot_controller_id)
            checked.append((provider, transform))

        self._bindings = tuple(sorted(
            checked,
            key=lambda item: (
                item[1].source_robot_id,
                item[1].source_controller_id,
                item[1].source_frame_id,
                item[1].source_generation_id,
            ),
        ))

    @staticmethod
    def _unavailable_robot(
        transform: CalibratedFrameTransform,
        reason_code: str,
    ):
        return {
            "read_only": True,
            "status": "unavailable",
            "reason_code": reason_code,
            "robot_id": transform.source_robot_id,
            "controller_instance_id": transform.source_controller_id,
            "local_frame_id": transform.source_frame_id,
            "local_generation_id": transform.source_generation_id,
            "robot_pose": None,
            "pose_history": [],
            "pose_history_evicted": 0,
            "collision_geometry": None,
            "navigation_trace": None,
            "object_hypotheses": [],
            "frame_transform": transform.to_dict(),
            "source_map_id": None,
            "source_map_version": None,
            "source_status": None,
            "captured_at_unix_ms": None,
            "source_age_ms": None,
        }

    @staticmethod
    def _available_robot(
        snapshot: object,
        transform: CalibratedFrameTransform,
    ):
        if not isinstance(snapshot, Mapping):
            raise _SourceUnavailable("source_snapshot_invalid")
        if (
            snapshot.get("schema") != DASHBOARD_SPATIAL_MAP_SCHEMA
            or snapshot.get("read_only") is not True
        ):
            raise _SourceUnavailable("source_contract_mismatch")
        if (
            snapshot.get("robot_id") != transform.source_robot_id
            or snapshot.get("controller_instance_id")
            != transform.source_controller_id
            or snapshot.get("frame_id") != transform.source_frame_id
            or snapshot.get("local_generation_id")
            != transform.source_generation_id
        ):
            raise _SourceUnavailable("source_identity_mismatch")
        if snapshot.get("frame_kind") != LOCAL_ODOMETRY:
            raise _SourceUnavailable("source_frame_kind_mismatch")
        if snapshot.get("status") not in (
            "available", "pose_only", "qualitative_only",
        ):
            raise _SourceUnavailable("source_map_unavailable")

        source_map_id = _source_scalar(
            snapshot.get("map_id"), identifier=True
        )
        source_map_version = _source_scalar(snapshot.get("map_version"))
        captured_at_unix_ms = _source_scalar(
            snapshot.get("captured_at_unix_ms")
        )
        source_age_ms = _source_scalar(snapshot.get("age_ms"))
        if (
            source_map_id is None
            or source_map_version is None
            or captured_at_unix_ms is None
            or source_age_ms is None
        ):
            raise _SourceUnavailable("source_contract_mismatch")

        current_pose = snapshot.get("robot_pose")
        if current_pose is None:
            raise _SourceUnavailable("source_pose_unavailable")
        robot_pose = _transform_pose(current_pose, transform)

        history = snapshot.get("pose_history")
        if not isinstance(history, (list, tuple)):
            raise _SourceUnavailable("source_pose_history_invalid")
        retained = history[-MAX_SHARED_POSE_HISTORY:]
        transformed_history = [
            _transform_pose(pose, transform) for pose in retained
        ]
        if not transformed_history or any(
            robot_pose[field] != transformed_history[-1][field]
            for field in ("x_mm", "y_mm", "heading_mdeg")
        ):
            raise _SourceUnavailable("source_pose_history_invalid")
        if robot_pose["observed_at_unix_ms"] > captured_at_unix_ms:
            raise _SourceUnavailable("source_pose_invalid")
        source_evicted = snapshot.get("pose_history_evicted", 0)
        if (
            type(source_evicted) is not int
            or not 0 <= source_evicted <= _MAX_INT
        ):
            raise _SourceUnavailable("source_pose_history_invalid")
        locally_evicted = max(0, len(history) - len(retained))
        if source_evicted > _MAX_INT - locally_evicted:
            raise _SourceUnavailable("source_pose_history_invalid")

        navigation_trace = _transform_navigation_trace(
            snapshot.get("navigation_trace"),
            transform,
            captured_at_unix_ms=captured_at_unix_ms,
        )
        object_hypotheses = _transform_provisional_obstacles(
            snapshot.get("object_hypotheses", ()),
            transform,
            captured_at_unix_ms=captured_at_unix_ms,
        )

        return {
            "read_only": True,
            "status": "available",
            "reason_code": "pose_transformed",
            "robot_id": transform.source_robot_id,
            "controller_instance_id": transform.source_controller_id,
            "local_frame_id": transform.source_frame_id,
            "local_generation_id": transform.source_generation_id,
            "robot_pose": robot_pose,
            "pose_history": transformed_history,
            "pose_history_evicted": source_evicted + locally_evicted,
            "collision_geometry": _collision_geometry(
                snapshot.get("collision_geometry")
            ),
            "navigation_trace": navigation_trace,
            "object_hypotheses": object_hypotheses,
            "frame_transform": transform.to_dict(),
            "source_map_id": source_map_id,
            "source_map_version": source_map_version,
            "source_status": snapshot["status"],
            "captured_at_unix_ms": captured_at_unix_ms,
            "source_age_ms": source_age_ms,
        }

    def snapshot(self):
        """Return one fresh, deeply detached view of all bound robots."""

        robots = []
        for provider, transform in self._bindings:
            try:
                local_snapshot = provider.snapshot()
            except Exception:
                robots.append(self._unavailable_robot(
                    transform,
                    "source_snapshot_failed",
                ))
                continue
            try:
                robots.append(self._available_robot(
                    local_snapshot,
                    transform,
                ))
            except _SourceUnavailable as error:
                robots.append(self._unavailable_robot(
                    transform,
                    error.reason_code,
                ))
            except Exception:
                robots.append(self._unavailable_robot(
                    transform,
                    "source_snapshot_invalid",
                ))

        available_count = sum(
            robot["status"] == "available" for robot in robots
        )
        if available_count == len(robots) and robots:
            status = "available"
            reason_code = "all_sources_available"
        elif available_count:
            status = "degraded"
            reason_code = "some_sources_unavailable"
        else:
            status = "unavailable"
            reason_code = "no_sources_available"

        captured_values = [
            robot["captured_at_unix_ms"]
            for robot in robots
            if robot["captured_at_unix_ms"] is not None
        ]
        value = {
            "schema": SHARED_SPATIAL_MAP_SCHEMA,
            "read_only": True,
            "status": status,
            "reason_code": reason_code,
            "map_id": "{}.shared-fixed-start.{}".format(
                self.world_frame_id,
                self.world_generation_id,
            ),
            "frame_id": self.world_frame_id,
            "frame_kind": SHARED_FIXED_START,
            "world_generation_id": self.world_generation_id,
            "source_id": "shared-spatial-map-compositor",
            "provenance": "CALIBRATED_FIXED_START_SE2_PROJECTION",
            "snapshot_semantics": LATEST_AVAILABLE_NOT_ATOMIC,
            "robots": robots,
            "bounds": None,
            "cells": [],
            "sensor_rays": [],
            "qualitative_observations": [],
            "scan_evidence_history": [],
            "object_hypotheses": [],
            "navigation_authority": None,
            "captured_at_unix_ms": (
                max(captured_values) if captured_values else None
            ),
        }
        return deepcopy(value)


__all__ = (
    "MAX_SHARED_POSE_HISTORY",
    "MAX_SHARED_ROBOTS",
    "LATEST_AVAILABLE_NOT_ATOMIC",
    "SHARED_FIXED_START",
    "SHARED_SPATIAL_MAP_SCHEMA",
    "SharedSpatialMapCompositor",
    "SharedSpatialMapError",
)
