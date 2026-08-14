"""Frozen directional mission arithmetic for one physical episode."""

from dataclasses import dataclass
import math
from typing import Mapping

from .physical_navigation_contract import MOTION_ACTIONS
from .physical_odometry import (
    OdometryCalibration,
    PhysicalPose,
    nominal_effect,
    normalize_heading_mdeg,
)


MISSION_SCHEMA = "robot-physical-directional-mission/v1"


@dataclass
class DirectionalMission:
    """Measure forward progress against an origin frozen at episode start."""

    episode_id: str
    minimum_forward_progress_mm: int
    origin_x_mm: int
    origin_y_mm: int
    reference_heading_mdeg: int
    peak_longitudinal_progress_mm: int = 0
    heading_tolerance_mdeg: int = 5_000

    @classmethod
    def begin(
        cls,
        *,
        episode_id: str,
        minimum_forward_progress_mm: int,
        pose: PhysicalPose,
        heading_tolerance_mdeg: int = 5_000,
    ):
        if (
            not isinstance(episode_id, str)
            or not episode_id
            or isinstance(minimum_forward_progress_mm, bool)
            or not isinstance(minimum_forward_progress_mm, int)
            or not 1 <= minimum_forward_progress_mm <= 2_000
            or isinstance(heading_tolerance_mdeg, bool)
            or not isinstance(heading_tolerance_mdeg, int)
            or not 1_000 <= heading_tolerance_mdeg <= 45_000
        ):
            raise ValueError("directional mission configuration is invalid")
        return cls(
            episode_id=episode_id,
            minimum_forward_progress_mm=minimum_forward_progress_mm,
            origin_x_mm=pose.x_mm,
            origin_y_mm=pose.y_mm,
            reference_heading_mdeg=pose.heading_mdeg,
            heading_tolerance_mdeg=heading_tolerance_mdeg,
        )

    def longitudinal_progress_mm(self, pose: PhysicalPose) -> int:
        heading = math.radians(self.reference_heading_mdeg / 1000.0)
        return int(
            round(
                (pose.x_mm - self.origin_x_mm) * math.cos(heading)
                + (pose.y_mm - self.origin_y_mm) * math.sin(heading)
            )
        )

    def lateral_offset_mm(self, pose: PhysicalPose) -> int:
        heading = math.radians(self.reference_heading_mdeg / 1000.0)
        return int(
            round(
                -(pose.x_mm - self.origin_x_mm) * math.sin(heading)
                + (pose.y_mm - self.origin_y_mm) * math.cos(heading)
            )
        )

    def target_point(self) -> tuple[int, int]:
        heading = math.radians(self.reference_heading_mdeg / 1000.0)
        return (
            int(round(
                self.origin_x_mm
                + self.minimum_forward_progress_mm * math.cos(heading)
            )),
            int(round(
                self.origin_y_mm
                + self.minimum_forward_progress_mm * math.sin(heading)
            )),
        )

    def distance_to_target_mm(self, pose: PhysicalPose) -> int:
        target_x, target_y = self.target_point()
        return int(round(math.hypot(
            target_x - pose.x_mm,
            target_y - pose.y_mm,
        )))

    def heading_aligned(self, pose: PhysicalPose) -> bool:
        return (
            abs(
                normalize_heading_mdeg(
                    pose.heading_mdeg - self.reference_heading_mdeg
                )
            )
            <= self.heading_tolerance_mdeg
        )

    def snapshot(
        self,
        *,
        pose: PhysicalPose,
        action_specs: Mapping[str, Mapping[str, object]],
        goal_corridor_clear: bool,
        all_known_hazards_passed: bool,
        localization_valid: bool,
        touch_pressed: bool,
        calibration: OdometryCalibration = OdometryCalibration(),
    ) -> Mapping[str, object]:
        current = self.longitudinal_progress_mm(pose)
        self.peak_longitudinal_progress_mm = max(
            self.peak_longitudinal_progress_mm,
            current,
        )
        candidate_deltas = {}
        projected_remaining = {}
        projected_alignment = {}
        for action in sorted(MOTION_ACTIONS):
            if action not in action_specs:
                continue
            nominal, _maximum = nominal_effect(
                pose,
                action,
                action_specs,
                calibration,
            )
            projected_progress = self.longitudinal_progress_mm(nominal)
            candidate_deltas[action] = projected_progress - current
            projected_remaining[action] = max(
                0,
                self.minimum_forward_progress_mm - projected_progress,
            )
            projected_alignment[action] = self.heading_aligned(nominal)
        heading_aligned = self.heading_aligned(pose)
        completed = (
            localization_valid is True
            and touch_pressed is False
            and goal_corridor_clear is True
            and all_known_hazards_passed is True
            and heading_aligned
            and current >= self.minimum_forward_progress_mm
        )
        return {
            "schema": MISSION_SCHEMA,
            "episode_id": self.episode_id,
            "origin_frozen": True,
            "origin_x_mm": self.origin_x_mm,
            "origin_y_mm": self.origin_y_mm,
            "reference_heading_mdeg": self.reference_heading_mdeg,
            "minimum_forward_progress_mm": (
                self.minimum_forward_progress_mm
            ),
            "current_longitudinal_progress_mm": current,
            "peak_longitudinal_progress_mm": (
                self.peak_longitudinal_progress_mm
            ),
            "regression_from_peak_mm": (
                self.peak_longitudinal_progress_mm - current
            ),
            "remaining_longitudinal_progress_mm": max(
                0,
                self.minimum_forward_progress_mm - current,
            ),
            "lateral_offset_mm": self.lateral_offset_mm(pose),
            "goal_heading_aligned": heading_aligned,
            "goal_corridor_clear": goal_corridor_clear,
            "all_known_hazards_passed": all_known_hazards_passed,
            "localization_valid": localization_valid,
            "touch_clear": not touch_pressed,
            "candidate_action_longitudinal_deltas_mm": candidate_deltas,
            "projected_remaining_after_action_mm": projected_remaining,
            "projected_goal_heading_aligned_after_action": (
                projected_alignment
            ),
            "completed": completed,
        }
