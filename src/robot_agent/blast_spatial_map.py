"""Read-only BLAST diagnostics in the existing spatial-map contract.

The bridge publishes episode-local encoder odometry, explicitly provisional
navigation intent, and bounded ultrasonic echo clusters.  The clusters never
claim occupancy, clearance, persistent object identity, or motion authority.
"""

from copy import deepcopy
import hashlib
import json
import math
import threading
import time
from typing import Callable, Mapping, Optional

from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_observation_monitor import CONTROLLER_ID, ROBOT_ID
from .blast_scan_observation import (
    SCAN_ANGULAR_RAY_SIDES,
    SCAN_MAX_ABSOLUTE_BEARING_DEG,
    SCAN_RAY_SIDES,
)
from .local_detour_route import (
    ROUTE_ACTIVE,
    ROUTE_COMPLETE,
    ROUTE_INVALID,
    ROUTE_SCHEMA,
    WAYPOINT_KINDS,
)
from .physical_odometry import PhysicalPose
from .spatial_map_contract import (
    ASYMMETRIC_RECTANGLE,
    DASHBOARD_SPATIAL_MAP_SCHEMA,
    DIFFERENTIAL_DRIVE_ORIGIN,
    LOCAL_ODOMETRY,
    MAP_EMPTY,
    MAX_POSE_HISTORY,
)


NAVIGATION_TRACE_SCHEMA = "robot-navigation-trace/v1"
MAX_PLANAR_SCAN_VIEWS = 16
MAX_PROVISIONAL_OBSTACLE_HYPOTHESES = 64
_ODOMETRY_PROVENANCE = "PROVISIONAL_ENCODER_ODOMETRY"
_TRACE_PROVENANCE = (
    "PROVISIONAL_ENCODER_ODOMETRY + PROVISIONAL_YAW_ONLY"
)
_SCAN_SIDES = frozenset((*SCAN_RAY_SIDES, *SCAN_ANGULAR_RAY_SIDES))
_SCAN_MAX_ABSOLUTE_BEARING_MDEG = round(
    SCAN_MAX_ABSOLUTE_BEARING_DEG * 1_000
)
_MAX_TRACE_BYTES = 64 * 1024
_OBSTACLE_CLASSIFICATION = "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER"
_OBSTACLE_GEOMETRY = "PROVISIONAL_ULTRASONIC_ECHO_CLUSTER"
_OBSTACLE_SOURCE = "blast-settled-measured-planar-projection"
_OBSTACLE_PROVENANCE = (
    "SETTLED_MEASURED_ULTRASONIC + PROVISIONAL_YAW_ONLY"
)
_WAYPOINT_STATUSES = frozenset(("COMPLETED", "ACTIVE", "UPCOMING"))


def _monotonic_ms() -> int:
    return time.monotonic_ns() // 1_000_000


def _unix_ms() -> int:
    return time.time_ns() // 1_000_000


def _identifier(name: str, value: object, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise ValueError(f"{name} is invalid")
    return value


def _clock_value(name: str, clock: Callable[[], int]) -> int:
    value = clock()
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} is invalid")
    return value


def _empty_snapshot(robot_id: str, controller_instance_id: str):
    return {
        "schema": DASHBOARD_SPATIAL_MAP_SCHEMA,
        "read_only": True,
        "status": "unavailable",
        "reason_code": "no_observations",
        "map_id": f"{robot_id}-local-map",
        "robot_id": robot_id,
        "controller_instance_id": controller_instance_id,
        "frame_id": None,
        "local_generation_id": None,
        "frame_kind": LOCAL_ODOMETRY,
        "map_quality": MAP_EMPTY,
        "map_version": 0,
        "based_on_state_version": 0,
        "based_on_world_model_version": 0,
        "resolution_mm": None,
        "capacity": MAX_POSE_HISTORY,
        "cells_evicted": 0,
        "source_id": "blast-navigation-motion-executor",
        "provenance": _ODOMETRY_PROVENANCE,
        "bounds": None,
        "robot_pose": None,
        "pose_history": [],
        "pose_history_evicted": 0,
        "collision_geometry": None,
        "sensor_rays": [],
        "cells": [],
        "qualitative_observations": [],
        "qualitative_observations_evicted": 0,
        "object_hypotheses": [],
        "scan_evidence_history": [],
        "scan_evidence_history_evicted": 0,
        "navigation_trace": None,
        "captured_at_unix_ms": None,
        "observed_at_unix_ms": None,
        "observed_age_ms": None,
        "age_ms": None,
    }


def _exact(value, fields) -> bool:
    return isinstance(value, Mapping) and set(value) == set(fields)


def _integer(value, minimum, maximum) -> bool:
    return type(value) is int and minimum <= value <= maximum


def _number(value, minimum, maximum) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
        and minimum <= float(value) <= maximum
    )


def _pose(value) -> bool:
    return _exact(value, ("x_mm", "y_mm", "heading_mdeg")) and (
        _integer(value["x_mm"], -1_000_000, 1_000_000)
        and _integer(value["y_mm"], -1_000_000, 1_000_000)
        and _integer(value["heading_mdeg"], -180_000, 179_999)
    )


def _final_goal(value) -> bool:
    fields = (
        "kind", "origin_x_mm", "origin_y_mm", "target_x_mm",
        "target_y_mm", "desired_heading_mdeg",
        "minimum_forward_progress_mm", "heading_tolerance_mdeg",
        "current_forward_progress_mm", "remaining_forward_progress_mm",
        "navigation_enforced",
    )
    if not _exact(value, fields) or not (
        value["kind"] == "DIRECTIONAL_HEADING"
        and type(value["navigation_enforced"]) is bool
        and _integer(value["origin_x_mm"], -1_000_000, 1_000_000)
        and _integer(value["origin_y_mm"], -1_000_000, 1_000_000)
        and _integer(value["target_x_mm"], -1_000_000, 1_000_000)
        and _integer(value["target_y_mm"], -1_000_000, 1_000_000)
        and _integer(value["desired_heading_mdeg"], -180_000, 179_999)
        and _integer(value["minimum_forward_progress_mm"], 1, 2_000)
        and _integer(value["heading_tolerance_mdeg"], 1_000, 45_000)
        and _integer(value["current_forward_progress_mm"], -1_000_000,
                     1_000_000)
        and _integer(value["remaining_forward_progress_mm"], 0, 1_000_000)
    ):
        return False
    radians = math.radians(value["desired_heading_mdeg"] / 1_000)
    distance = value["minimum_forward_progress_mm"]
    expected_x = value["origin_x_mm"] + distance * math.cos(radians)
    expected_y = value["origin_y_mm"] + distance * math.sin(radians)
    expected_remaining = max(
        0, distance - value["current_forward_progress_mm"]
    )
    return (
        abs(value["target_x_mm"] - expected_x) <= 1.5
        and abs(value["target_y_mm"] - expected_y) <= 1.5
        and value["remaining_forward_progress_mm"] == expected_remaining
    )


def _planned_leg(value) -> bool:
    if value is None:
        return True
    if not (
        _exact(value, (
            "kind", "scope", "clearance_proven", "passage_proven",
            "route_eligible", "selected_side", "bind_pose", "waypoint",
        ))
        and value["clearance_proven"] is False
        and value["passage_proven"] is False
        and value["selected_side"] in ("LEFT", "RIGHT")
        and _pose(value["bind_pose"])
        and _pose(value["waypoint"])
    ):
        return False
    if value["scope"] == "SEARCH_POSITION_ONLY":
        return (
            value["kind"] == "SIDE_SEARCH"
            and value["route_eligible"] is False
        )
    if value["scope"] == "LOCAL_DETOUR_ROUTE":
        return (
            value["kind"] in WAYPOINT_KINDS
            and value["route_eligible"] is True
        )
    return False


def _route_waypoint(value) -> bool:
    return (
        _exact(value, (
            "ordinal", "kind", "x_mm", "y_mm", "heading_mdeg",
            "fact_key", "status",
        ))
        and _integer(value["ordinal"], 0, 8)
        and value["kind"] in WAYPOINT_KINDS
        and _integer(value["x_mm"], -1_000_000, 1_000_000)
        and _integer(value["y_mm"], -1_000_000, 1_000_000)
        and _integer(value["heading_mdeg"], -180_000, 179_999)
        and (
            value["fact_key"] is None
            or isinstance(value["fact_key"], str)
            and 0 < len(value["fact_key"]) <= 128
        )
        and value["status"] in _WAYPOINT_STATUSES
    )


def _local_detour_route(value) -> bool:
    if value is None:
        return True
    if not (
        _exact(value, (
            "schema", "read_only", "provisional", "route_id", "version",
            "status", "detour_side", "active_index", "waypoints",
        ))
        and value["schema"] == ROUTE_SCHEMA
        and value["read_only"] is True
        and value["provisional"] is True
        and isinstance(value["route_id"], str)
        and 0 < len(value["route_id"]) <= 128
        and _integer(value["version"], 1, 1_000_000)
        and value["status"] in (ROUTE_ACTIVE, ROUTE_COMPLETE, ROUTE_INVALID)
        and value["detour_side"] in ("LEFT_OF_GOAL", "RIGHT_OF_GOAL")
        and isinstance(value["waypoints"], (tuple, list))
        and 1 <= len(value["waypoints"]) <= 5
        and all(_route_waypoint(item) for item in value["waypoints"])
        and [item["ordinal"] for item in value["waypoints"]]
        == list(range(len(value["waypoints"])))
        and _integer(value["active_index"], 0, len(value["waypoints"]))
    ):
        return False
    active_index = value["active_index"]
    expected = []
    for index in range(len(value["waypoints"])):
        if index < active_index:
            expected.append("COMPLETED")
        elif index == active_index and value["status"] == ROUTE_ACTIVE:
            expected.append("ACTIVE")
        else:
            expected.append("UPCOMING")
    if value["status"] == ROUTE_COMPLETE:
        if active_index != len(value["waypoints"]):
            return False
        expected = ["COMPLETED"] * len(value["waypoints"])
    elif value["status"] == ROUTE_ACTIVE and active_index >= len(
        value["waypoints"]
    ):
        return False
    return [item["status"] for item in value["waypoints"]] == expected


def _imu_heading(value, now_unix_ms) -> bool:
    return value is None or (
        _exact(value, ("heading_mdeg", "reference", "observed_at_unix_ms"))
        and _integer(value["heading_mdeg"], -180_000, 179_999)
        and value["reference"] == "EPISODE_START"
        and _integer(value["observed_at_unix_ms"], 0, now_unix_ms)
    )


def _scan_point(value) -> bool:
    fields = (
        "side", "measured_range_mm", "relative_bearing_mdeg",
        "sensor_origin_x_mm", "sensor_origin_y_mm", "beam_heading_mdeg",
        "nominal_echo_x_mm", "nominal_echo_y_mm",
    )
    if not _exact(value, fields) or not (
        value["side"] in _SCAN_SIDES
        and _number(value["measured_range_mm"], 0, 1_999.999999)
        and _integer(
            value["relative_bearing_mdeg"],
            -_SCAN_MAX_ABSOLUTE_BEARING_MDEG,
            _SCAN_MAX_ABSOLUTE_BEARING_MDEG,
        )
        and _integer(value["sensor_origin_x_mm"], -1_000_000, 1_000_000)
        and _integer(value["sensor_origin_y_mm"], -1_000_000, 1_000_000)
        and _integer(value["beam_heading_mdeg"], -180_000, 179_999)
        and _integer(value["nominal_echo_x_mm"], -1_000_000, 1_000_000)
        and _integer(value["nominal_echo_y_mm"], -1_000_000, 1_000_000)
    ):
        return False
    relative = value["relative_bearing_mdeg"]
    if (
        (value["side"] == "center" and relative != 0)
        or (value["side"].startswith("left_") and relative <= 0)
        or (value["side"].startswith("right_") and relative >= 0)
    ):
        return False
    radians = math.radians(value["beam_heading_mdeg"] / 1_000)
    expected_x = value["sensor_origin_x_mm"] + (
        float(value["measured_range_mm"]) * math.cos(radians)
    )
    expected_y = value["sensor_origin_y_mm"] + (
        float(value["measured_range_mm"]) * math.sin(radians)
    )
    return (
        abs(value["nominal_echo_x_mm"] - expected_x) <= 1.5
        and abs(value["nominal_echo_y_mm"] - expected_y) <= 1.5
    )


def _scan_view(value, now_unix_ms) -> bool:
    if not _exact(value, (
        "scan_id", "observed_at_unix_ms", "scan_pose", "projection",
    )) or not (
        isinstance(value["scan_id"], str)
        and 0 < len(value["scan_id"]) <= 128
        and _integer(value["observed_at_unix_ms"], 0, now_unix_ms)
        and _pose(value["scan_pose"])
    ):
        return False
    projection = value["projection"]
    if not _exact(projection, (
        "schema", "frame", "quality", "vertical_pitch_compensated",
        "ultrasonic_beam_width_modeled", "scan_turn_translation_compensated",
        "points",
    )) or not (
        projection["schema"] == "blast-planar-scan-projection/v1"
        and projection["frame"] == "EPISODE_LOCAL_ODOMETRY"
        and projection["quality"] == "PROVISIONAL_YAW_ONLY"
        and projection["vertical_pitch_compensated"] is False
        and projection["ultrasonic_beam_width_modeled"] is False
        and projection["scan_turn_translation_compensated"] is False
        and isinstance(projection["points"], list)
        and len(projection["points"]) <= len(SCAN_ANGULAR_RAY_SIDES)
        and all(_scan_point(point) for point in projection["points"])
    ):
        return False
    sides = [point["side"] for point in projection["points"]]
    return len(sides) == len(set(sides))


def _trace_inputs(final_goal, planned_leg, imu_heading,
                  planar_scan_views, now_unix_ms,
                  local_detour_route=None) -> bool:
    try:
        encoded = json.dumps(
            {
                "final_goal": final_goal,
                "planned_leg": planned_leg,
                "imu_heading": imu_heading,
                "planar_scan_views": planar_scan_views,
                "local_detour_route": local_detour_route,
            },
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except (TypeError, ValueError):
        return False
    if len(encoded) > _MAX_TRACE_BYTES or not (
        _final_goal(final_goal)
        and _planned_leg(planned_leg)
        and _imu_heading(imu_heading, now_unix_ms)
        and _local_detour_route(local_detour_route)
        and isinstance(planar_scan_views, (tuple, list))
        and len(planar_scan_views) <= MAX_PLANAR_SCAN_VIEWS
        and all(_scan_view(view, now_unix_ms) for view in planar_scan_views)
    ):
        return False
    if planned_leg is not None and (
        (
            planned_leg["scope"] == "SEARCH_POSITION_ONLY"
            and final_goal["navigation_enforced"] is not False
        )
        or (
            planned_leg["scope"] == "LOCAL_DETOUR_ROUTE"
            and final_goal["navigation_enforced"] is not True
        )
    ):
        return False
    ids = [view["scan_id"] for view in planar_scan_views]
    return len(ids) == len(set(ids))


def _cluster_scan_points(points):
    """Group angular neighbours without claiming persistent object identity."""

    ordered = sorted(
        points,
        key=lambda item: item["relative_bearing_mdeg"],
        reverse=True,
    )
    clusters = []
    for point in ordered:
        if not clusters:
            clusters.append([point])
            continue
        previous = clusters[-1][-1]
        bearing_gap = abs(
            point["relative_bearing_mdeg"]
            - previous["relative_bearing_mdeg"]
        )
        endpoint_gap = math.hypot(
            point["nominal_echo_x_mm"] - previous["nominal_echo_x_mm"],
            point["nominal_echo_y_mm"] - previous["nominal_echo_y_mm"],
        )
        if bearing_gap <= 30_000 and endpoint_gap <= 225:
            clusters[-1].append(point)
        else:
            clusters.append([point])
    return clusters


def _provisional_obstacle_hypotheses(planar_scan_views):
    """Infer bounded echo clusters only from validated projection points."""

    values = []
    for view in planar_scan_views:
        for ordinal, cluster in enumerate(_cluster_scan_points(
            view["projection"]["points"]
        )):
            centroid_x = int(round(sum(
                point["nominal_echo_x_mm"] for point in cluster
            ) / len(cluster)))
            centroid_y = int(round(sum(
                point["nominal_echo_y_mm"] for point in cluster
            ) / len(cluster)))
            support_radius = max(35, int(math.ceil(max(
                math.hypot(
                    point["nominal_echo_x_mm"] - centroid_x,
                    point["nominal_echo_y_mm"] - centroid_y,
                )
                for point in cluster
            ))) + 35)
            mean_bearing = int(round(sum(
                point["relative_bearing_mdeg"] for point in cluster
            ) / len(cluster)))
            relation = (
                "LEFT_OF_SCAN" if mean_bearing > 7_500
                else "RIGHT_OF_SCAN" if mean_bearing < -7_500
                else "FRONT_OF_SCAN"
            )
            identity = json.dumps(
                [
                    view["scan_id"],
                    ordinal,
                    [
                        [point["side"], point["nominal_echo_x_mm"],
                         point["nominal_echo_y_mm"]]
                        for point in cluster
                    ],
                ],
                separators=(",", ":"),
            ).encode("utf-8")
            values.append({
                "hypothesis_id": "blast-ultrasonic-{}".format(
                    hashlib.sha256(identity).hexdigest()[:20]
                ),
                "classification": _OBSTACLE_CLASSIFICATION,
                "label": _OBSTACLE_CLASSIFICATION,
                "x_mm": centroid_x,
                "y_mm": centroid_y,
                "geometry_kind": _OBSTACLE_GEOMETRY,
                "support_radius_mm": support_radius,
                "support_points": [
                    {
                        "side": point["side"],
                        "x_mm": point["nominal_echo_x_mm"],
                        "y_mm": point["nominal_echo_y_mm"],
                        "measured_range_mm": point["measured_range_mm"],
                        "relative_bearing_mdeg": (
                            point["relative_bearing_mdeg"]
                        ),
                    }
                    for point in cluster
                ],
                "source_scan_ids": [view["scan_id"]],
                "bearing": relation.removesuffix("_OF_SCAN"),
                "relation": relation,
                "evidence_count": len(cluster),
                "confidence_milli": min(350, 200 + 25 * (len(cluster) - 1)),
                "source_id": _OBSTACLE_SOURCE,
                "provenance": _OBSTACLE_PROVENANCE,
                "quality": "PROVISIONAL_YAW_ONLY",
                "settled_measured_only": True,
                "provisional": True,
                "read_only": True,
                "observed_at_unix_ms": view["observed_at_unix_ms"],
                "age_ms": 0,
            })
    return values[-MAX_PROVISIONAL_OBSTACLE_HYPOTHESES:]


def _provisional_obstacle_hypothesis(value, now_unix_ms) -> bool:
    fields = (
        "hypothesis_id", "classification", "label", "x_mm", "y_mm",
        "geometry_kind", "support_radius_mm", "support_points",
        "source_scan_ids", "bearing", "relation", "evidence_count",
        "confidence_milli", "source_id", "provenance", "quality",
        "settled_measured_only", "provisional", "read_only",
        "observed_at_unix_ms", "age_ms",
    )
    if not _exact(value, fields) or not (
        isinstance(value["hypothesis_id"], str)
        and 0 < len(value["hypothesis_id"]) <= 128
        and value["classification"] == _OBSTACLE_CLASSIFICATION
        and value["label"] == _OBSTACLE_CLASSIFICATION
        and _integer(value["x_mm"], -1_000_000, 1_000_000)
        and _integer(value["y_mm"], -1_000_000, 1_000_000)
        and value["geometry_kind"] == _OBSTACLE_GEOMETRY
        and _integer(value["support_radius_mm"], 1, 10_000)
        and isinstance(value["support_points"], list)
        and 1 <= len(value["support_points"]) <= len(SCAN_ANGULAR_RAY_SIDES)
        and isinstance(value["source_scan_ids"], list)
        and 1 <= len(value["source_scan_ids"]) <= MAX_PLANAR_SCAN_VIEWS
        and all(
            isinstance(scan_id, str) and 0 < len(scan_id) <= 128
            for scan_id in value["source_scan_ids"]
        )
        and len(value["source_scan_ids"]) == len(set(value["source_scan_ids"]))
        and value["bearing"] in ("LEFT", "FRONT", "RIGHT")
        and value["relation"] in (
            "LEFT_OF_SCAN", "FRONT_OF_SCAN", "RIGHT_OF_SCAN",
        )
        and value["evidence_count"] == len(value["support_points"])
        and _integer(value["confidence_milli"], 1, 400)
        and value["source_id"] == _OBSTACLE_SOURCE
        and value["provenance"] == _OBSTACLE_PROVENANCE
        and value["quality"] == "PROVISIONAL_YAW_ONLY"
        and value["settled_measured_only"] is True
        and value["provisional"] is True
        and value["read_only"] is True
        and _integer(value["observed_at_unix_ms"], 0, now_unix_ms)
        and _integer(value["age_ms"], 0, 2**63 - 1)
    ):
        return False
    points = []
    for point in value["support_points"]:
        if not (
            _exact(point, (
                "side", "x_mm", "y_mm", "measured_range_mm",
                "relative_bearing_mdeg",
            ))
            and point["side"] in _SCAN_SIDES
            and _integer(point["x_mm"], -1_000_000, 1_000_000)
            and _integer(point["y_mm"], -1_000_000, 1_000_000)
            and _number(point["measured_range_mm"], 0, 1_999.999999)
            and _integer(
                point["relative_bearing_mdeg"],
                -_SCAN_MAX_ABSOLUTE_BEARING_MDEG,
                _SCAN_MAX_ABSOLUTE_BEARING_MDEG,
            )
            and (
                point["side"] != "center"
                or point["relative_bearing_mdeg"] == 0
            )
            and (
                not point["side"].startswith("left_")
                or point["relative_bearing_mdeg"] > 0
            )
            and (
                not point["side"].startswith("right_")
                or point["relative_bearing_mdeg"] < 0
            )
        ):
            return False
        points.append(point)
    centroid_x = sum(point["x_mm"] for point in points) / len(points)
    centroid_y = sum(point["y_mm"] for point in points) / len(points)
    radius = max(math.hypot(
        point["x_mm"] - value["x_mm"],
        point["y_mm"] - value["y_mm"],
    ) for point in points)
    mean_bearing = sum(
        point["relative_bearing_mdeg"] for point in points
    ) / len(points)
    relation = (
        "LEFT_OF_SCAN" if mean_bearing > 7_500
        else "RIGHT_OF_SCAN" if mean_bearing < -7_500
        else "FRONT_OF_SCAN"
    )
    return (
        abs(value["x_mm"] - centroid_x) <= 1.5
        and abs(value["y_mm"] - centroid_y) <= 1.5
        and radius <= value["support_radius_mm"] <= radius + 100
        and value["relation"] == relation
        and value["bearing"] == relation.removesuffix("_OF_SCAN")
    )


class BlastSpatialMapBridge:
    """Cache one detached, pose-only map for the active BLAST episode."""

    def __init__(
        self,
        *,
        robot_id: str = ROBOT_ID,
        controller_instance_id: str = CONTROLLER_ID,
        monotonic_clock_ms: Callable[[], int] = _monotonic_ms,
        unix_clock_ms: Callable[[], int] = _unix_ms,
    ):
        self.robot_id = _identifier("robot_id", robot_id)
        self.controller_instance_id = _identifier(
            "controller_instance_id", controller_instance_id
        )
        if not callable(monotonic_clock_ms) or not callable(unix_clock_ms):
            raise ValueError("BLAST spatial map clocks are invalid")
        footprint = (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.robot_footprint
        )
        if footprint is None:
            raise ValueError("BLAST spatial footprint is unavailable")
        self._collision_geometry = {
            "geometry": ASYMMETRIC_RECTANGLE,
            "reference_point": DIFFERENTIAL_DRIVE_ORIGIN,
            "front_extent_mm": footprint.front_extent_mm,
            "rear_extent_mm": footprint.rear_extent_mm,
            "left_extent_mm": footprint.left_extent_mm,
            "right_extent_mm": footprint.right_extent_mm,
            "clearance_margin_mm": footprint.clearance_margin_mm,
            "calibration_status": footprint.calibration_status,
            "calibration_evidence": footprint.calibration_evidence,
        }
        self._monotonic_clock_ms = monotonic_clock_ms
        self._unix_clock_ms = unix_clock_ms
        self._lock = threading.RLock()
        self._accepting = True
        self._generation = 0
        self._episode_id: Optional[str] = None
        self._frame_id: Optional[str] = None
        self._state_version = 0
        self._world_model_version = 0
        self._map_version = 0
        self._pose_history = []
        self._pose_history_evicted = 0
        self._object_hypotheses = []
        self._last_monotonic_ms: Optional[int] = None
        self._last_unix_ms: Optional[int] = None
        self._trace = None
        self._snapshot = _empty_snapshot(
            self.robot_id, self.controller_instance_id
        )

    @staticmethod
    def _validate_observation(observation: Mapping[str, object]) -> None:
        if not isinstance(observation, Mapping):
            raise ValueError("BLAST map observation is invalid")
        motors = observation.get("motor_angles_deg")
        if (
            not isinstance(motors, Mapping)
            or type(motors.get("left_drive")) is not int
            or type(motors.get("right_drive")) is not int
            or observation.get("motion_active") is not False
        ):
            raise ValueError("BLAST map observation is not idle and anchored")

    @staticmethod
    def _pose_dict(pose: PhysicalPose, frame_id: str, version: int,
                   observed_at_unix_ms: int):
        if not isinstance(pose, PhysicalPose):
            raise ValueError("BLAST map pose is invalid")
        return {
            "x_mm": pose.x_mm,
            "y_mm": pose.y_mm,
            "heading_mdeg": pose.heading_mdeg,
            "frame_id": frame_id,
            "state_version": version,
            "source_id": "blast-navigation-motion-executor",
            "provenance": _ODOMETRY_PROVENANCE,
            "observed_at_unix_ms": observed_at_unix_ms,
            "age_ms": 0,
        }

    def _retain_pose(self, pose) -> None:
        if self._pose_history and all(
            self._pose_history[-1][field] == pose[field]
            for field in ("x_mm", "y_mm", "heading_mdeg")
        ):
            self._pose_history[-1] = pose
            return
        if len(self._pose_history) == MAX_POSE_HISTORY:
            del self._pose_history[0]
            self._pose_history_evicted += 1
        self._pose_history.append(pose)

    def _refresh_snapshot(self, pose) -> None:
        assert self._frame_id is not None
        assert self._episode_id is not None
        assert self._last_unix_ms is not None
        has_obstacle_hypotheses = bool(self._object_hypotheses)
        self._snapshot = {
            "schema": DASHBOARD_SPATIAL_MAP_SCHEMA,
            "read_only": True,
            "status": (
                "qualitative_only" if has_obstacle_hypotheses else "pose_only"
            ),
            "reason_code": (
                "provisional_ultrasonic_obstacle_hypotheses"
                if has_obstacle_hypotheses else "pose_only"
            ),
            "map_id": f"{self.robot_id}-local-map",
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "frame_id": self._frame_id,
            "local_generation_id": self._episode_id,
            "frame_kind": LOCAL_ODOMETRY,
            "map_quality": MAP_EMPTY,
            "map_version": self._map_version,
            "based_on_state_version": self._state_version,
            "based_on_world_model_version": self._world_model_version,
            "resolution_mm": None,
            "capacity": MAX_POSE_HISTORY,
            "cells_evicted": 0,
            "source_id": "blast-navigation-motion-executor",
            "provenance": _ODOMETRY_PROVENANCE,
            "bounds": None,
            "robot_pose": pose,
            "pose_history": list(self._pose_history),
            "pose_history_evicted": self._pose_history_evicted,
            "collision_geometry": dict(self._collision_geometry),
            "sensor_rays": [],
            "cells": [],
            "qualitative_observations": [],
            "qualitative_observations_evicted": 0,
            "object_hypotheses": deepcopy(self._object_hypotheses),
            "scan_evidence_history": [],
            "scan_evidence_history_evicted": 0,
            "navigation_trace": deepcopy(self._trace),
            "localization": {
                "valid": True,
                "ground_truth_available": False,
                "imu_fused": False,
                "provenance": _ODOMETRY_PROVENANCE,
            },
            "captured_at_unix_ms": self._last_unix_ms,
            "observed_at_unix_ms": self._last_unix_ms,
            "observed_age_ms": 0,
            "age_ms": 0,
        }

    def begin_episode(
        self, *, episode_id: str, pose: PhysicalPose,
        observation: Mapping[str, object],
    ) -> bool:
        episode_id = _identifier("episode_id", episode_id)
        self._validate_observation(observation)
        now_monotonic = _clock_value(
            "BLAST spatial monotonic clock", self._monotonic_clock_ms
        )
        now_unix = _clock_value(
            "BLAST spatial Unix clock", self._unix_clock_ms
        )
        with self._lock:
            if not self._accepting:
                return False
            self._generation += 1
            self._episode_id = episode_id
            self._frame_id = (
                f"{self.robot_id}.episode-local-{self._generation}"
            )
            self._state_version = 1
            self._world_model_version += 1
            self._map_version = 1
            self._pose_history = []
            self._pose_history_evicted = 0
            self._object_hypotheses = []
            self._trace = None
            self._last_monotonic_ms = now_monotonic
            self._last_unix_ms = now_unix
            item = self._pose_dict(
                pose, self._frame_id, self._state_version, now_unix
            )
            self._retain_pose(item)
            self._refresh_snapshot(item)
            return True

    def offer_pose(
        self, *, episode_id: str, pose: PhysicalPose,
        observation: Mapping[str, object],
    ) -> bool:
        self._validate_observation(observation)
        now_monotonic = _clock_value(
            "BLAST spatial monotonic clock", self._monotonic_clock_ms
        )
        now_unix = _clock_value(
            "BLAST spatial Unix clock", self._unix_clock_ms
        )
        with self._lock:
            if not self._accepting or episode_id != self._episode_id:
                return False
            if (
                self._last_monotonic_ms is not None
                and now_monotonic < self._last_monotonic_ms
            ):
                return False
            self._state_version += 1
            self._map_version += 1
            self._last_monotonic_ms = now_monotonic
            self._last_unix_ms = now_unix
            assert self._frame_id is not None
            item = self._pose_dict(
                pose, self._frame_id, self._state_version, now_unix
            )
            self._retain_pose(item)
            self._refresh_snapshot(item)
            return True

    def offer_trace(
        self, *, episode_id: str, final_goal: Mapping[str, object],
        planned_leg: Optional[Mapping[str, object]] = None,
        imu_heading: Optional[Mapping[str, object]] = None,
        planar_scan_views=(),
        local_detour_route: Optional[Mapping[str, object]] = None,
    ) -> bool:
        now_monotonic = _clock_value(
            "BLAST spatial monotonic clock", self._monotonic_clock_ms
        )
        now_unix = _clock_value(
            "BLAST spatial Unix clock", self._unix_clock_ms
        )
        if not _trace_inputs(
            final_goal, planned_leg, imu_heading, planar_scan_views, now_unix,
            local_detour_route,
        ):
            raise ValueError("BLAST navigation trace is invalid")
        with self._lock:
            if not self._accepting or episode_id != self._episode_id:
                return False
            if (
                self._last_monotonic_ms is not None
                and now_monotonic < self._last_monotonic_ms
            ):
                return False
            assert self._frame_id is not None
            self._trace = {
                "schema": NAVIGATION_TRACE_SCHEMA,
                "read_only": True,
                "frame_id": self._frame_id,
                "provenance": _TRACE_PROVENANCE,
                "final_goal": deepcopy(dict(final_goal)),
                "imu_heading": (
                    None if imu_heading is None else deepcopy(dict(imu_heading))
                ),
                "planned_leg": (
                    None if planned_leg is None else deepcopy(dict(planned_leg))
                ),
                "local_detour_route": (
                    None
                    if local_detour_route is None
                    else deepcopy(dict(local_detour_route))
                ),
                "planar_scan_views": deepcopy(list(planar_scan_views)),
            }
            self._object_hypotheses = _provisional_obstacle_hypotheses(
                planar_scan_views
            )
            self._map_version += 1
            self._last_monotonic_ms = now_monotonic
            self._last_unix_ms = now_unix
            pose = self._snapshot.get("robot_pose")
            if isinstance(pose, Mapping):
                self._refresh_snapshot(dict(pose))
            return True

    def snapshot(self):
        with self._lock:
            value = deepcopy(self._snapshot)
            if self._last_monotonic_ms is not None:
                now_unix = _clock_value(
                    "BLAST spatial Unix clock", self._unix_clock_ms
                )
                age = max(0, _clock_value(
                    "BLAST spatial monotonic clock", self._monotonic_clock_ms
                ) - self._last_monotonic_ms)
                value["observed_age_ms"] = age
                value["age_ms"] = age
                for pose in value.get("pose_history", ()):
                    pose["age_ms"] = max(
                        0, now_unix - pose["observed_at_unix_ms"]
                    )
                if isinstance(value.get("robot_pose"), Mapping):
                    value["robot_pose"]["age_ms"] = max(
                        0,
                        now_unix
                        - value["robot_pose"]["observed_at_unix_ms"],
                    )
            return value

    def close(self, drain: bool = True, timeout_s: float = 5.0) -> bool:
        if type(drain) is not bool or not isinstance(timeout_s, (int, float)):
            raise ValueError("BLAST spatial map close options are invalid")
        if not math.isfinite(float(timeout_s)) or not 0 <= timeout_s <= 60:
            raise ValueError("BLAST spatial map close timeout is invalid")
        with self._lock:
            self._accepting = False
        return True


__all__ = (
    "NAVIGATION_TRACE_SCHEMA",
    "MAX_PROVISIONAL_OBSTACLE_HYPOTHESES",
    "BlastSpatialMapBridge",
)
