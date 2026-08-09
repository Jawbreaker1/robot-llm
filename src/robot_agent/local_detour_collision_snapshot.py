"""Source-neutral collision geometry consumed by local detour routes."""

from dataclasses import dataclass
from typing import Optional, Tuple

from .physical_footprint import RobotFootprint


def _identifier(value: object) -> bool:
    return (
        isinstance(value, str)
        and value == value.strip()
        and 0 < len(value) <= 128
        and all(ord(character) >= 32 for character in value)
    )
@dataclass(frozen=True)
class LocalDetourCollisionSnapshot:
    """One target's immutable collision envelope at one map revision."""

    frame_id: str
    map_generation_id: str
    map_version: int
    target_hypothesis_id: str
    target_centroid_x_mm: int
    target_centroid_y_mm: int
    target_envelope_radius_mm: int
    target_support_points: Tuple[Tuple[int, int], ...]
    robot_footprint: Optional[RobotFootprint]
    lateral_clearance_margin_mm: int

    def __post_init__(self) -> None:
        supports = self.target_support_points
        centroid = (self.target_centroid_x_mm, self.target_centroid_y_mm)
        if (
            not _identifier(self.frame_id)
            or not _identifier(self.map_generation_id)
            or not _identifier(self.target_hypothesis_id)
            or type(self.map_version) is not int
            or self.map_version < 0
            or type(self.target_centroid_x_mm) is not int
            or type(self.target_centroid_y_mm) is not int
            or type(self.target_envelope_radius_mm) is not int
            or self.target_envelope_radius_mm <= 0
            or not isinstance(supports, tuple)
            or not 1 <= len(supports) <= 4_160  # supports plus centroids
            or any(
                not isinstance(point, tuple)
                or len(point) != 2
                or any(type(value) is not int for value in point)
                for point in supports
            )
            or tuple(sorted(set(supports))) != supports
            or centroid not in supports
            or (
                self.robot_footprint is not None
                and not isinstance(self.robot_footprint, RobotFootprint)
            )
            or type(self.lateral_clearance_margin_mm) is not int
            or not 0 <= self.lateral_clearance_margin_mm <= 500
        ):
            raise ValueError("local detour collision snapshot is invalid")


__all__ = ("LocalDetourCollisionSnapshot",)
