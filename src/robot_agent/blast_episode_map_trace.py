"""Fail-open projection of one BLAST episode into the diagnostic map."""

import copy
import math
import time
from typing import Mapping

from .blast_side_observation import side_search_planned_leg
from .local_detour_route import ROUTE_COMPLETE
from .physical_navigation_mission import DirectionalMission
from .physical_navigation_contract import (
    ADVANCE, REVERSE, SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90,
)
from .physical_odometry import normalize_heading_mdeg


_MOTION_ACTIONS = frozenset((
    ADVANCE, REVERSE, TURN_LEFT_90, TURN_RIGHT_90,
))


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
        )
        self.navigation_enforced = False
        self.planned_leg = None
        self.planar_scan_views = []
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
        heading = math.radians(
            self.mission.reference_heading_mdeg / 1_000.0
        )
        distance = self.mission.minimum_forward_progress_mm
        current = self.mission.longitudinal_progress_mm(pose)
        return {
            "kind": "DIRECTIONAL_HEADING",
            "navigation_enforced": self.navigation_enforced,
            "origin_x_mm": self.mission.origin_x_mm,
            "origin_y_mm": self.mission.origin_y_mm,
            "target_x_mm": int(round(
                self.mission.origin_x_mm + distance * math.cos(heading)
            )),
            "target_y_mm": int(round(
                self.mission.origin_y_mm + distance * math.sin(heading)
            )),
            "desired_heading_mdeg": self.mission.reference_heading_mdeg,
            "minimum_forward_progress_mm": distance,
            "heading_tolerance_mdeg": self.mission.heading_tolerance_mdeg,
            "current_forward_progress_mm": current,
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
        self._offer(
            "offer_trace",
            episode_id=self.episode_id,
            final_goal=self._final_goal(pose),
            planned_leg=self.planned_leg,
            imu_heading=self._imu_heading(
                observation, observed_at_unix_ms
            ),
            planar_scan_views=tuple(self.planar_scan_views),
        )

    def record(
        self,
        *,
        pose,
        observation,
        pose_observed,
        selected_side,
        waypoint,
        bind_pose,
        scan_view,
        route=None,
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
        next_leg = side_search_planned_leg(
            selected_side, waypoint, bind_pose)
        if next_leg is not None and (
            self.planned_leg is None
            or self.planned_leg.get("waypoint") != next_leg["waypoint"]
        ):
            self.planned_leg = next_leg
        if route is not None:
            self.navigation_enforced = True
            if route.status == ROUTE_COMPLETE:
                self.planned_leg = None
            else:
                active = route.active_waypoint
                self.planned_leg = {
                    "kind": active.kind,
                    "scope": "LOCAL_DETOUR_ROUTE",
                    "clearance_proven": False,
                    "passage_proven": False,
                    "route_eligible": True,
                    "selected_side": (
                        "LEFT"
                        if route.detour_side == "LEFT_OF_GOAL"
                        else "RIGHT"
                    ),
                    "bind_pose": _map_pose(pose),
                    "waypoint": {
                        "x_mm": active.x_mm,
                        "y_mm": active.y_mm,
                        "heading_mdeg": active.heading_mdeg,
                    },
                }
        if isinstance(scan_view, Mapping):
            self.planar_scan_views.append({
                "scan_id": "{}-scan-{}".format(
                    self.episode_id,
                    len(self.planar_scan_views) + 1,
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
        self._offer_trace(pose, observation, observed_at_unix_ms)

    def record_action(
        self, action, pose, observation, selected_side, waypoint,
        scan_view, route=None, pose_observed=None,
    ):
        self.record(
            pose=pose,
            observation=observation,
            pose_observed=(
                action in _MOTION_ACTIONS
                if pose_observed is None else pose_observed
            ),
            selected_side=selected_side,
            waypoint=waypoint,
            bind_pose=pose,
            scan_view=(scan_view if action == SCAN_FRONT_ARC else None),
            route=route,
        )


__all__ = ("_BlastEpisodeMapTrace",)
