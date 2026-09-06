"""Fail-open projection of one BLAST episode into the diagnostic map."""

import copy
import math
import time
from typing import Mapping

from .blast_observation_monitor import ROBOT_ID
from .blast_mission_completion import (
    BLAST_GOAL_HEADING_TOLERANCE_MDEG,
    BLAST_GOAL_RADIUS_MM,
)
from .blast_spatial_map import (
    MAX_PLANAR_SCAN_VIEWS,
    MODEL_WAYPOINT,
    provisional_obstacle_hypotheses,
)
from .coarse_navigation_grid import (
    GRID_CELL_SIZE_MM,
    ROUTE_CLEARANCE_MM,
    build_coarse_navigation_grid,
    known_clear_axis_reach_mm,
    model_route_blockage,
    route_blockage_from_echoes,
)
from .local_detour_route import (
    ROUTE_ACTIVE,
    ROUTE_SCHEMA,
)
from .physical_navigation_mission import DirectionalMission
from .physical_navigation_contract import (
    ADVANCE, REVERSE, SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90,
)
from .physical_odometry import PhysicalPose, normalize_heading_mdeg


_MOTION_ACTIONS = frozenset((
    ADVANCE, REVERSE, TURN_LEFT_90, TURN_RIGHT_90,
))
_VISITED_CELL_LIMIT = 128


def _map_pose(pose):
    return {
        "x_mm": pose.x_mm,
        "y_mm": pose.y_mm,
        "heading_mdeg": pose.heading_mdeg,
    }


class _BlastEpisodeMapTrace:
    """Fail-open projection of one episode into the diagnostic map sink."""

    def __init__(
        self,
        *,
        bridge,
        episode_id,
        pose,
        observation,
        observed_at_unix_ms,
        episode_start_heading,
        minimum_forward_progress_mm,
    ):
        self.bridge = bridge
        self.episode_id = episode_id
        self.episode_start_heading = episode_start_heading
        self.mission = DirectionalMission.begin(
            episode_id=episode_id,
            minimum_forward_progress_mm=minimum_forward_progress_mm,
            pose=pose,
            heading_tolerance_mdeg=BLAST_GOAL_HEADING_TOLERANCE_MDEG,
        )
        self.advisory_waypoint = None
        self.advisory_waypoint_plan = ()
        self._waypoint_plan_version = 0
        self.planar_scan_views = []
        self._obstacle_points = []
        self.visited_cells = []
        self._scan_sequence = 0
        self._last_pose = pose
        self._last_observation = copy.deepcopy(observation)
        self._last_observed_at_unix_ms = observed_at_unix_ms
        self._record_visited_cell(pose)
        self._offer(
            "begin_episode",
            episode_id=episode_id,
            pose=pose,
            observation=observation,
        )
        self._offer_trace(pose, observation, observed_at_unix_ms)

    def _offer(self, method, **values):
        try:
            return getattr(self.bridge, method)(**values)
        except Exception:
            return False

    def _final_goal(self, pose):
        distance = self.mission.minimum_forward_progress_mm
        current = self.mission.longitudinal_progress_mm(pose)
        target_x, target_y = self.mission.target_point()
        return {
            "kind": "DIRECTIONAL_HEADING",
            "navigation_enforced": False,
            "origin_x_mm": self.mission.origin_x_mm,
            "origin_y_mm": self.mission.origin_y_mm,
            "target_x_mm": target_x,
            "target_y_mm": target_y,
            "goal_radius_mm": BLAST_GOAL_RADIUS_MM,
            "distance_to_goal_mm": self.mission.distance_to_target_mm(pose),
            "desired_heading_mdeg": self.mission.reference_heading_mdeg,
            "minimum_forward_progress_mm": distance,
            "heading_tolerance_mdeg": self.mission.heading_tolerance_mdeg,
            "current_forward_progress_mm": current,
            "current_lateral_offset_mm": self.mission.lateral_offset_mm(pose),
            "remaining_forward_progress_mm": max(0, distance - current),
        }

    def _imu_heading(self, observation, observed_at_unix_ms):
        imu = observation.get("imu") if isinstance(observation, Mapping) else None
        heading = imu.get("heading_deg") if isinstance(imu, Mapping) else None
        if (
            isinstance(heading, bool)
            or not isinstance(heading, (int, float))
            or not math.isfinite(float(heading))
            or isinstance(self.episode_start_heading, bool)
            or not isinstance(self.episode_start_heading, (int, float))
            or not math.isfinite(float(self.episode_start_heading))
            or type(observed_at_unix_ms) is not int
        ):
            return None
        return {
            # Display the same motion-local heading used for driving. Raw gyro
            # drift during a planner pause must not rotate a second map arrow.
            "heading_mdeg": self._last_pose.heading_mdeg,
            "reference": "EPISODE_START",
            "observed_at_unix_ms": observed_at_unix_ms,
        }

    def _offer_trace(self, pose, observation, observed_at_unix_ms):
        self._record_visited_cell(pose)
        self._last_pose = pose
        self._last_observation = copy.deepcopy(observation)
        self._last_observed_at_unix_ms = observed_at_unix_ms
        return self._offer(
            "offer_trace",
            episode_id=self.episode_id,
            final_goal=self._final_goal(pose),
            planned_leg=None,
            advisory_waypoint=self.advisory_waypoint,
            imu_heading=self._imu_heading(
                observation, observed_at_unix_ms
            ),
            planar_scan_views=tuple(self.planar_scan_views),
            local_detour_route=self._advisory_route(pose),
            coarse_grid=self._coarse_grid(pose),
        )

    def _advisory_route(self, pose):
        """Project Gemma's ordered hypothesis into the existing map shape."""

        if not self.advisory_waypoint_plan:
            return None
        start = (pose.x_mm, pose.y_mm)
        previous = start
        waypoints = []
        for index, waypoint in enumerate(self.advisory_waypoint_plan):
            delta_x = waypoint["x_mm"] - previous[0]
            delta_y = waypoint["y_mm"] - previous[1]
            heading = normalize_heading_mdeg(round(
                math.degrees(math.atan2(delta_y, delta_x)) * 1_000
            ))
            waypoints.append({
                "ordinal": index,
                "kind": MODEL_WAYPOINT,
                "purpose": waypoint["purpose"],
                "x_mm": waypoint["x_mm"],
                "y_mm": waypoint["y_mm"],
                "heading_mdeg": heading,
                "fact_key": None,
                "status": "ACTIVE" if index == 0 else "UPCOMING",
            })
            previous = (waypoint["x_mm"], waypoint["y_mm"])
        lateral_delta = next((
            self._episode_axes(item["x_mm"], item["y_mm"])[1]
            - self.mission.lateral_offset_mm(pose)
            for item in self.advisory_waypoint_plan
            if self._episode_axes(item["x_mm"], item["y_mm"])[1]
            != self.mission.lateral_offset_mm(pose)
        ), 0)
        return {
            "schema": ROUTE_SCHEMA,
            "read_only": True,
            "provisional": True,
            "route_id": "gemma-waypoints-{}".format(self.episode_id)[:128],
            "version": max(1, self._waypoint_plan_version),
            "status": ROUTE_ACTIVE,
            "detour_side": (
                "LEFT_OF_GOAL" if lateral_delta > 0 else "RIGHT_OF_GOAL"
            ),
            "active_index": 0,
            "waypoints": waypoints,
        }

    def _episode_axes(self, x_mm, y_mm):
        heading = math.radians(
            self.mission.reference_heading_mdeg / 1_000.0
        )
        relative_x = x_mm - self.mission.origin_x_mm
        relative_y = y_mm - self.mission.origin_y_mm
        return (
            relative_x * math.cos(heading)
            + relative_y * math.sin(heading),
            -relative_x * math.sin(heading)
            + relative_y * math.cos(heading),
        )

    def _record_visited_cell(self, pose):
        if not isinstance(pose, PhysicalPose):
            return
        x_mm, y_mm = self._episode_axes(pose.x_mm, pose.y_mm)
        cell = {
            "x_mm": (
                math.floor(x_mm / GRID_CELL_SIZE_MM + 0.5)
                * GRID_CELL_SIZE_MM
            ),
            "y_mm": (
                math.floor(y_mm / GRID_CELL_SIZE_MM + 0.5)
                * GRID_CELL_SIZE_MM
            ),
        }
        if not self.visited_cells or self.visited_cells[-1] != cell:
            self.visited_cells.append(cell)
            del self.visited_cells[:-_VISITED_CELL_LIMIT]

    def _remember_obstacles(self, points):
        """Missing echoes do not erase objects; measured free rays can.

        Nearby repeat hits replace previous evidence instead of inflating the
        same box on every scan. This is map memory, not route selection.
        """
        spacing = GRID_CELL_SIZE_MM / 2
        retained = []
        for old in self._obstacle_points:
            ox, oy = old["nominal_echo_x_mm"], old["nominal_echo_y_mm"]
            replaced = False
            for point in points:
                ex, ey = point["nominal_echo_x_mm"], point["nominal_echo_y_mm"]
                if math.hypot(ex - ox, ey - oy) <= spacing:
                    replaced = True
                    break
                if not all(k in point for k in (
                    "sensor_origin_x_mm", "sensor_origin_y_mm",
                )):
                    continue
                sx, sy = point["sensor_origin_x_mm"], point["sensor_origin_y_mm"]
                dx, dy = ex - sx, ey - sy
                length = math.hypot(dx, dy)
                if length == 0:
                    continue
                along = ((ox - sx) * dx + (oy - sy) * dy) / length
                across = abs((ox - sx) * dy - (oy - sy) * dx) / length
                if 0 < along < length - spacing and across < spacing:
                    replaced = True
                    break
            if not replaced:
                retained.append(old)
        self._obstacle_points = retained + copy.deepcopy(points)

    def _coarse_navigation_observations(self):
        """Use retained obstacles and measured rays, never no-return as free."""
        possible_obstacles = [self._episode_axes(
            point["nominal_echo_x_mm"], point["nominal_echo_y_mm"],
        ) for point in self._obstacle_points]
        clear_segments = []
        for view in self.planar_scan_views:
            for point in view["projection"]["points"]:
                if all(key in point for key in (
                    "sensor_origin_x_mm", "sensor_origin_y_mm",
                )):
                    clear_segments.append((
                        self._episode_axes(
                            point["sensor_origin_x_mm"],
                            point["sensor_origin_y_mm"],
                        ),
                        self._episode_axes(
                            point["nominal_echo_x_mm"],
                            point["nominal_echo_y_mm"],
                        ),
                    ))
        return tuple(possible_obstacles), tuple(clear_segments)

    def _coarse_grid(self, pose):
        possible_obstacles, clear_segments = (
            self._coarse_navigation_observations()
        )
        waypoint = None
        if self.advisory_waypoint is not None:
            waypoint = self._episode_axes(
                self.advisory_waypoint["x_mm"],
                self.advisory_waypoint["y_mm"],
            )
        robot_position = (
            self.mission.longitudinal_progress_mm(pose),
            self.mission.lateral_offset_mm(pose),
        )
        return build_coarse_navigation_grid(
            robots=({
                "symbol": "B",
                "robot_id": ROBOT_ID,
                "forward_mm": robot_position[0],
                "left_mm": robot_position[1],
                "heading_mdeg": normalize_heading_mdeg(
                    pose.heading_mdeg
                    - self.mission.reference_heading_mdeg
                ),
            },),
            goal=(self.mission.minimum_forward_progress_mm, 0),
            waypoint=waypoint,
            possible_obstacles=possible_obstacles,
            clear_segments=clear_segments,
            window_center=robot_position,
        )

    def _current_echo_clusters(self):
        """Summarize retained echo evidence without choosing a route or side."""

        if not self.planar_scan_views:
            return []
        try:
            hypotheses = provisional_obstacle_hypotheses(
                ({"scan_id": "episode-map-memory",
                  "observed_at_unix_ms": self._last_observed_at_unix_ms,
                  "projection": {
                    "points": self._obstacle_points,
                }},)
            )
            values = []
            for ordinal, hypothesis in enumerate(hypotheses, 1):
                points = [
                    self._episode_axes(point["x_mm"], point["y_mm"])
                    for point in hypothesis["support_points"]
                ]
                x_values = [point[0] for point in points]
                y_values = [point[1] for point in points]
                echo_bounds = {
                    "x_min_mm": round(min(x_values)),
                    "x_max_mm": round(max(x_values)),
                    "y_min_mm": round(min(y_values)),
                    "y_max_mm": round(max(y_values)),
                }
                values.append({
                    "cluster": ordinal,
                    "provisional": True,
                    "evidence_count": hypothesis["evidence_count"],
                    "echo_bounds_mm": echo_bounds,
                    "robot_center_keep_out_bounds_mm": {
                        "x_min_mm": (
                            echo_bounds["x_min_mm"] - ROUTE_CLEARANCE_MM
                        ),
                        "x_max_mm": (
                            echo_bounds["x_max_mm"] + ROUTE_CLEARANCE_MM
                        ),
                        "y_min_mm": (
                            echo_bounds["y_min_mm"] - ROUTE_CLEARANCE_MM
                        ),
                        "y_max_mm": (
                            echo_bounds["y_max_mm"] + ROUTE_CLEARANCE_MM
                        ),
                    },
                })
            return values
        except (KeyError, TypeError, ValueError):
            return []

    @staticmethod
    def _direct_detour_axis_candidates(robot_position, target_position, clusters):
        """Expose both nearest lateral grid lines; never select one."""

        start_x, start_y = robot_position
        target_x, target_y = target_position
        x_min, x_max = sorted((start_x, target_x))
        y_min, y_max = sorted((start_y, target_y))
        blockers = []
        for cluster in clusters:
            bounds = cluster["robot_center_keep_out_bounds_mm"]
            if (
                bounds["x_max_mm"] >= x_min
                and bounds["x_min_mm"] <= x_max
                and bounds["y_max_mm"] >= y_min
                and bounds["y_min_mm"] <= y_max
            ):
                blockers.append(bounds)
        if not blockers:
            return None

        left_boundary = max(item["y_max_mm"] for item in blockers)
        right_boundary = min(item["y_min_mm"] for item in blockers)
        return {
            "basis": "CURRENT_SCAN_ECHO_BOUNDS",
            "side_selected": False,
            "left_y_mm": (
                math.ceil(left_boundary / GRID_CELL_SIZE_MM)
                * GRID_CELL_SIZE_MM
            ),
            "right_y_mm": (
                math.floor(right_boundary / GRID_CELL_SIZE_MM)
                * GRID_CELL_SIZE_MM
            ),
        }

    def planner_local_map_evidence(self, pose):
        """Return a compact echo-point map with no inferred free space."""

        if not isinstance(pose, PhysicalPose):
            return None
        try:
            goal = self._final_goal(pose)
            possible_obstacles, _clear_segments = (
                self._coarse_navigation_observations()
            )
            robot_position = self._episode_axes(pose.x_mm, pose.y_mm)
            target_position = self._episode_axes(
                goal["target_x_mm"], goal["target_y_mm"],
            )
            goal_delta_x = target_position[0] - robot_position[0]
            goal_delta_y = target_position[1] - robot_position[1]
            signed_forward_error = (
                goal["minimum_forward_progress_mm"]
                - goal["current_forward_progress_mm"]
            )
            direct_goal_blockage = route_blockage_from_echoes(
                start=robot_position,
                waypoints=(self._episode_axes(
                    goal["target_x_mm"], goal["target_y_mm"],
                ),),
                possible_obstacles=possible_obstacles,
            )
            coarse_grid = self._coarse_grid(pose)
            planner_grid = {
                key: copy.deepcopy(coarse_grid[key])
                for key in ("cell_size_mm", "window")
            }
            cell_size_mm = planner_grid["cell_size_mm"]
            window = planner_grid["window"]
            planner_grid["rows"] = [
                {
                    "x_mm": window["x_max_mm"] - index * cell_size_mm,
                    "cells": row,
                }
                for index, row in enumerate(coarse_grid["rows"])
            ]
            planner_grid["column_y_mm"] = [
                window["y_max_mm"] - index * cell_size_mm
                for index in range(len(coarse_grid["rows"][0]))
            ]
            evidence = {
                "schema": "blast-local-map-evidence/v1",
                "frame": "EPISODE_LOCAL_ODOMETRY",
                "coordinate_convention": {
                    "x_positive": "EPISODE_START_FORWARD",
                    "y_positive": "EPISODE_START_LEFT",
                    "heading_positive": "LEFT_CCW",
                },
                "unobserved_space": "UNKNOWN_NOT_FREE",
                "route_clearance_mm": ROUTE_CLEARANCE_MM,
                "coarse_grid": planner_grid,
                "known_clear_axis_reach_mm": (
                    known_clear_axis_reach_mm(coarse_grid)
                ),
                "visited_cells": copy.deepcopy(self.visited_cells),
                "robot_pose": _map_pose(pose),
                "directional_goal": {
                    key: goal[key]
                    for key in (
                        "target_x_mm", "target_y_mm",
                        "desired_heading_mdeg",
                        "goal_radius_mm", "distance_to_goal_mm",
                        "remaining_forward_progress_mm",
                    )
                } | {
                    "signed_forward_error_mm": signed_forward_error,
                    "longitudinal_relation": (
                        "BEFORE_GOAL_LINE"
                        if signed_forward_error > 0
                        else "BEYOND_GOAL_LINE"
                        if signed_forward_error < 0
                        else "ON_GOAL_LINE"
                    ),
                    "goal_vector": {
                        "delta_x_mm": round(goal_delta_x),
                        "delta_y_mm": round(goal_delta_y),
                        "distance_mm": round(math.hypot(
                            goal_delta_x, goal_delta_y,
                        )),
                    },
                    "corridor_entered": (
                        self.mission.distance_to_target_mm(pose)
                        <= BLAST_GOAL_RADIUS_MM
                    ),
                    "heading_aligned": self.mission.heading_aligned(pose),
                    "heading_error_mdeg": normalize_heading_mdeg(
                        self.mission.reference_heading_mdeg
                        - pose.heading_mdeg
                    ),
                },
            }
            current_echo_clusters = self._current_echo_clusters()
            if current_echo_clusters:
                evidence["current_echo_clusters"] = current_echo_clusters
                detour_candidates = self._direct_detour_axis_candidates(
                    robot_position, target_position, current_echo_clusters,
                )
                if detour_candidates is not None:
                    evidence["direct_detour_axis_candidates"] = (
                        detour_candidates
                    )
            if direct_goal_blockage is not None:
                evidence["direct_goal_blockage"] = direct_goal_blockage
            return evidence
        except (KeyError, TypeError, ValueError):
            return None

    def advisory_route_blockage(self, pose):
        """Check Gemma's route against known echo body clearance."""

        if not self.advisory_waypoint_plan:
            return None
        return model_route_blockage(
            start=self._episode_axes(pose.x_mm, pose.y_mm),
            waypoints=(self._episode_axes(
                self.advisory_waypoint_plan[0]["x_mm"],
                self.advisory_waypoint_plan[0]["y_mm"],
            ),),
            possible_obstacles=(
                self._coarse_navigation_observations()[0]
            ),
        )

    def set_advisory_waypoint_plan(
        self, waypoints, *, pose, observation, observed_at_unix_ms,
        source="GEMMA_MODEL",
    ):
        """Publish Gemma's ordered hypothesis without selecting its points."""

        if source != "GEMMA_MODEL":
            return False

        try:
            plan = tuple({
                "x_mm": waypoint["x_mm"],
                "y_mm": waypoint["y_mm"],
                "purpose": waypoint["purpose"],
            } for waypoint in waypoints)
        except (KeyError, TypeError):
            return False
        if len(plan) > 4:
            return False
        self.advisory_waypoint_plan = plan
        self._waypoint_plan_version += 1
        self.advisory_waypoint = None if not plan else {
            **plan[0],
            "source": source,
            "read_only": True,
        }
        return self._offer_trace(pose, observation, observed_at_unix_ms)

    def set_advisory_waypoint(
        self, waypoint, *, pose, observation, observed_at_unix_ms,
    ):
        return self.set_advisory_waypoint_plan(
            () if waypoint is None else (waypoint,),
            pose=pose,
            observation=observation,
            observed_at_unix_ms=observed_at_unix_ms,
        )

    def invalidate_localization(self):
        """Make an ambiguous post-motion pose explicitly unavailable."""

        return self._offer(
            "invalidate_localization", episode_id=self.episode_id,
        )

    def finalize(
        self, *, pose=None, observation=None, observed_at_unix_ms=None,
    ):
        """Publish the final diagnostic pose, scan history and waypoint."""

        pose = self._last_pose if pose is None else pose
        observation = (
            self._last_observation if observation is None else observation
        )
        if not isinstance(observation, Mapping):
            return False
        if observed_at_unix_ms is None:
            observed_at_unix_ms = self._last_observed_at_unix_ms
        return self._offer_trace(pose, observation, observed_at_unix_ms)

    def record(
        self,
        *,
        pose,
        observation,
        pose_observed,
        scan_view,
    ):
        if not isinstance(observation, Mapping):
            return
        observed_at_unix_ms = time.time_ns() // 1_000_000
        if pose_observed:
            self._offer(
                "offer_pose",
                episode_id=self.episode_id,
                pose=pose,
                observation=observation,
            )
        if isinstance(scan_view, Mapping):
            self._scan_sequence += 1
            self._remember_obstacles(scan_view["planar_projection"]["points"])
            self.planar_scan_views.append({
                "scan_id": "{}-scan-{}".format(
                    self.episode_id,
                    self._scan_sequence,
                ),
                "observed_at_unix_ms": observed_at_unix_ms,
                "scan_pose": {
                    key: scan_view["scan_pose"][key]
                    for key in ("x_mm", "y_mm", "heading_mdeg")
                },
                "projection": copy.deepcopy(
                    scan_view["planar_projection"]
                ),
            })
            del self.planar_scan_views[:-MAX_PLANAR_SCAN_VIEWS]
        self._offer_trace(pose, observation, observed_at_unix_ms)

    def record_action(
        self, action, pose, observation, scan_view, pose_observed=None,
    ):
        self.record(
            pose=pose,
            observation=observation,
            pose_observed=(
                action in _MOTION_ACTIONS
                if pose_observed is None else pose_observed
            ),
            scan_view=(scan_view if action == SCAN_FRONT_ARC else None),
        )


__all__ = ("_BlastEpisodeMapTrace",)
