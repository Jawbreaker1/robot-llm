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
from .blast_spatial_map import MAX_PLANAR_SCAN_VIEWS
from .coarse_navigation_grid import (
    GRID_CELL_SIZE_MM,
    build_coarse_navigation_grid,
    known_clear_axis_reach_mm,
    model_route_blockage,
    route_blockage_from_echoes,
)
from .local_detour_route import (
    LATERAL_CLEARANCE,
    MERGE_GOAL_AXIS,
    PASS_BEYOND_TARGET,
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
        relative = (
            float(heading) - float(self.episode_start_heading) + 180.0
        ) % 360.0 - 180.0
        return {
            "heading_mdeg": normalize_heading_mdeg(
                -round(relative * 1_000)
            ),
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
        count = len(self.advisory_waypoint_plan)
        for index, waypoint in enumerate(self.advisory_waypoint_plan):
            delta_x = waypoint["x_mm"] - previous[0]
            delta_y = waypoint["y_mm"] - previous[1]
            heading = normalize_heading_mdeg(round(
                math.degrees(math.atan2(delta_y, delta_x)) * 1_000
            ))
            kind = (
                LATERAL_CLEARANCE if index == 0
                else MERGE_GOAL_AXIS if index == count - 1
                else PASS_BEYOND_TARGET
            )
            waypoints.append({
                "ordinal": index,
                "kind": kind,
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

    def _coarse_navigation_observations(self):
        possible_obstacles = []
        clear_segments = []
        for view in self.planar_scan_views:
            for point in view["projection"]["points"]:
                possible_obstacles.append(self._episode_axes(
                    point["nominal_echo_x_mm"],
                    point["nominal_echo_y_mm"],
                ))
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
