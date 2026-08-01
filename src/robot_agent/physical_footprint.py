"""Profile-specific robot body geometry and conservative sweep checks."""

from dataclasses import dataclass
import math
from typing import Mapping, Tuple

from .physical_odometry import PhysicalPose, normalize_heading_mdeg


@dataclass(frozen=True)
class RobotFootprint:
    """Body extents around the differential-drive origin.

    Forward is positive local X and left is positive local Y.  The separate
    sides let an assembled robot describe an arm or sensor that protrudes on
    only one side.
    """

    front_extent_mm: int
    rear_extent_mm: int
    left_extent_mm: int
    right_extent_mm: int
    clearance_margin_mm: int = 0
    calibration_status: str = "unspecified"
    calibration_evidence: str = "not provided"

    def __post_init__(self) -> None:
        extents = (
            self.front_extent_mm,
            self.rear_extent_mm,
            self.left_extent_mm,
            self.right_extent_mm,
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 1 <= value <= 1_000
            for value in extents
        ) or (
            isinstance(self.clearance_margin_mm, bool)
            or not isinstance(self.clearance_margin_mm, int)
            or not 0 <= self.clearance_margin_mm <= 500
        ) or any(
            not isinstance(value, str)
            or not value
            or value != value.strip()
            or len(value) > 512
            or any(ord(character) < 32 for character in value)
            for value in (
                self.calibration_status,
                self.calibration_evidence,
            )
        ):
            raise ValueError("robot footprint is invalid")

    @property
    def maximum_corner_radius_mm(self) -> float:
        return max(
            math.hypot(longitudinal, lateral)
            for longitudinal in (
                self.front_extent_mm,
                self.rear_extent_mm,
            )
            for lateral in (
                self.left_extent_mm,
                self.right_extent_mm,
            )
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "geometry": "ASYMMETRIC_RECTANGLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "front_extent_mm": self.front_extent_mm,
            "rear_extent_mm": self.rear_extent_mm,
            "left_extent_mm": self.left_extent_mm,
            "right_extent_mm": self.right_extent_mm,
            "clearance_margin_mm": self.clearance_margin_mm,
            "calibration_status": self.calibration_status,
            "calibration_evidence": self.calibration_evidence,
        }


def _point_distance(
    px: float,
    py: float,
    pose: PhysicalPose,
    footprint: RobotFootprint,
) -> float:
    heading = math.radians(pose.heading_mdeg / 1000.0)
    relative_x = px - pose.x_mm
    relative_y = py - pose.y_mm
    local_forward = (
        relative_x * math.cos(heading)
        + relative_y * math.sin(heading)
    )
    local_left = (
        -relative_x * math.sin(heading)
        + relative_y * math.cos(heading)
    )
    outside_forward = max(
        -footprint.rear_extent_mm - local_forward,
        0.0,
        local_forward - footprint.front_extent_mm,
    )
    outside_lateral = max(
        -footprint.right_extent_mm - local_left,
        0.0,
        local_left - footprint.left_extent_mm,
    )
    return math.hypot(outside_forward, outside_lateral)


def _interpolated_sweep(
    start: PhysicalPose,
    end: PhysicalPose,
    footprint: RobotFootprint,
) -> Tuple[Tuple[PhysicalPose, ...], int]:
    """Sample a pose sweep with a conservative between-sample inflation."""

    translation_mm = math.hypot(
        end.x_mm - start.x_mm,
        end.y_mm - start.y_mm,
    )
    heading_delta_mdeg = normalize_heading_mdeg(
        end.heading_mdeg - start.heading_mdeg
    )
    steps = max(
        1,
        int(math.ceil(translation_mm / 10.0)),
        int(math.ceil(abs(heading_delta_mdeg) / 5_000.0)),
    )
    heading_step_radians = math.radians(
        abs(heading_delta_mdeg) / 1000.0 / steps
    )
    per_step_motion_bound = (
        translation_mm / steps
        + 2.0
        * footprint.maximum_corner_radius_mm
        * math.sin(heading_step_radians / 2.0)
    )
    # Integer pose interpolation can add less than one millimetre.
    # Every point between two samples is at most half an interval from its
    # nearest endpoint.  Inflating by the full interval was safe but caused
    # needless EV3 vetoes around the already conservative provisional IR
    # envelope.
    coverage_margin_mm = (
        int(math.ceil(per_step_motion_bound / 2.0)) + 1
    )
    samples = []
    for index in range(steps + 1):
        ratio = index / float(steps)
        samples.append(
            PhysicalPose(
                x_mm=int(round(
                    start.x_mm + (end.x_mm - start.x_mm) * ratio
                )),
                y_mm=int(round(
                    start.y_mm + (end.y_mm - start.y_mm) * ratio
                )),
                heading_mdeg=normalize_heading_mdeg(
                    start.heading_mdeg
                    + int(round(heading_delta_mdeg * ratio))
                ),
                verified_motion_count=start.verified_motion_count,
                total_forward_mm=start.total_forward_mm,
                total_turn_mdeg=start.total_turn_mdeg,
            )
        )
    return tuple(samples), coverage_margin_mm


def footprint_sweep_intersects(
    *,
    obstacle_x_mm: int,
    obstacle_y_mm: int,
    obstacle_radius_mm: int,
    start: PhysicalPose,
    end: PhysicalPose,
    footprint: RobotFootprint,
) -> Tuple[bool, bool]:
    """Return ``(start_intersects, complete_sweep_intersects)``."""

    base_clearance_mm = (
        obstacle_radius_mm + footprint.clearance_margin_mm
    )
    start_intersects = (
        _point_distance(
            obstacle_x_mm,
            obstacle_y_mm,
            start,
            footprint,
        )
        <= base_clearance_mm
    )
    samples, coverage_margin_mm = _interpolated_sweep(
        start,
        end,
        footprint,
    )
    swept_intersects = any(
        _point_distance(
            obstacle_x_mm,
            obstacle_y_mm,
            sample,
            footprint,
        )
        <= base_clearance_mm + coverage_margin_mm
        for sample in samples
    )
    return start_intersects, swept_intersects


__all__ = ("RobotFootprint", "footprint_sweep_intersects")
