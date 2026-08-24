"""Fail-open projection of one BLAST episode into the diagnostic map."""

import copy
import math
import time
from typing import Mapping

from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_mission_completion import (
    BLAST_GOAL_HEADING_TOLERANCE_MDEG,
    BLAST_GOAL_RADIUS_MM,
)
from .blast_spatial_map import MAX_PLANAR_SCAN_VIEWS
from .physical_navigation_mission import DirectionalMission
from .physical_navigation_contract import (
    ADVANCE, REVERSE, SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90,
)
from .physical_odometry import PhysicalPose, normalize_heading_mdeg


_MOTION_ACTIONS = frozenset((
    ADVANCE, REVERSE, TURN_LEFT_90, TURN_RIGHT_90,
))
_PLANNER_SCAN_VIEW_LIMIT = 4


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
        self.planar_scan_views = []
        self._scan_sequence = 0
        self._last_pose = pose
        self._last_observation = copy.deepcopy(observation)
        self._last_observed_at_unix_ms = observed_at_unix_ms
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
            local_detour_route=None,
        )

    def planner_local_map_evidence(self, pose):
        """Return a compact echo-point map with no inferred free space."""

        footprint = (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.robot_footprint
        )
        if not isinstance(pose, PhysicalPose) or footprint is None:
            return None
        try:
            goal = self._final_goal(pose)
            retained = self.planar_scan_views[-_PLANNER_SCAN_VIEW_LIMIT:]
            return {
                "schema": "blast-local-map-evidence/v1",
                "frame": "EPISODE_LOCAL_ODOMETRY",
                "quality": "PROVISIONAL_YAW_ONLY",
                "coordinate_convention": {
                    "x_positive": "EPISODE_START_FORWARD",
                    "y_positive": "EPISODE_START_LEFT",
                    "heading_positive": "LEFT_CCW",
                },
                "echo_points_mean": (
                    "POSSIBLE_OBSTACLE_RETURN_NOT_OBJECT_BOUNDARY"
                ),
                "unobserved_space": "UNKNOWN_NOT_FREE",
                "occupancy_model": "NONE",
                "robot_pose": _map_pose(pose),
                "directional_goal": {
                    key: goal[key]
                    for key in (
                        "target_x_mm", "target_y_mm",
                        "desired_heading_mdeg",
                        "goal_radius_mm", "distance_to_goal_mm",
                        "minimum_forward_progress_mm",
                        "current_forward_progress_mm",
                        "current_lateral_offset_mm",
                        "remaining_forward_progress_mm",
                    )
                },
                "robot_footprint_mm": {
                    "front": footprint.front_extent_mm,
                    "rear": footprint.rear_extent_mm,
                    "left": footprint.left_extent_mm,
                    "right": footprint.right_extent_mm,
                    "clearance_margin": footprint.clearance_margin_mm,
                },
                "scan_views": [
                    {
                        "scan_id": view["scan_id"],
                        "scan_pose": {
                            key: view["scan_pose"][key]
                            for key in ("x_mm", "y_mm", "heading_mdeg")
                        },
                        "echo_points": [
                            {
                                "x_mm": point["nominal_echo_x_mm"],
                                "y_mm": point["nominal_echo_y_mm"],
                            }
                            for point in view["projection"]["points"]
                        ],
                    }
                    for view in retained
                ],
                "truncated": (
                    len(self.planar_scan_views) > _PLANNER_SCAN_VIEW_LIMIT
                ),
            }
        except (KeyError, TypeError, ValueError):
            return None

    def set_advisory_waypoint(
        self, waypoint, *, pose, observation, observed_at_unix_ms,
    ):
        """Publish Gemma's memory without granting it motion authority."""

        try:
            candidate = None if waypoint is None else {
                "x_mm": waypoint["x_mm"],
                "y_mm": waypoint["y_mm"],
                "purpose": waypoint["purpose"],
                "source": "GEMMA_MODEL",
                "read_only": True,
            }
        except (KeyError, TypeError):
            return False
        self.advisory_waypoint = candidate
        return self._offer_trace(pose, observation, observed_at_unix_ms)

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
