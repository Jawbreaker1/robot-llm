"""Calibrated rigid transforms from one robot-local frame to a world frame.

The transform is deliberately geometry-only.  It does not infer a calibration
or decide whether two observations describe the same physical object.  Every
forward operation must present the exact local-frame identity so stale local
odometry generations cannot silently enter a shared world map.
"""

from dataclasses import dataclass
import math
from typing import Mapping, Tuple

from .physical_odometry import PhysicalPose, normalize_heading_mdeg


MAX_FRAME_COORDINATE_MM = 1_000_000
MAX_POSITION_UNCERTAINTY_MM = 1_000_000
MAX_YAW_UNCERTAINTY_MDEG = 180_000


class FrameTransformError(ValueError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _identifier(value: object, name: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 128
        or any(ord(character) < 32 for character in value)
    ):
        raise FrameTransformError(
            "invalid_frame_transform_identity",
            "{} is invalid".format(name),
        )
    return value


def _integer(
    value: object,
    name: str,
    *,
    minimum: int,
    maximum: int,
) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise FrameTransformError(
            "invalid_frame_transform_value",
            "{} is invalid".format(name),
        )
    return value


def _coordinate(value: object, name: str) -> int:
    return _integer(
        value,
        name,
        minimum=-MAX_FRAME_COORDINATE_MM,
        maximum=MAX_FRAME_COORDINATE_MM,
    )


def _heading(value: object, name: str) -> int:
    return _integer(value, name, minimum=-180_000, maximum=179_999)


def _rotate(x_mm: int, y_mm: int, yaw_mdeg: int) -> Tuple[int, int]:
    angle = math.radians(yaw_mdeg / 1_000.0)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        int(round(x_mm * cosine - y_mm * sine)),
        int(round(x_mm * sine + y_mm * cosine)),
    )


@dataclass(frozen=True)
class CalibratedFrameTransform:
    """An immutable local-to-world SE(2) calibration and its identity fence."""

    source_robot_id: str
    source_controller_id: str
    source_frame_id: str
    source_generation_id: str
    world_frame_id: str
    world_generation_id: str
    tx_mm: int
    ty_mm: int
    yaw_mdeg: int
    position_uncertainty_mm: int
    yaw_uncertainty_mdeg: int
    provenance: Tuple[str, ...]

    def __post_init__(self) -> None:
        for name in (
            "source_robot_id",
            "source_controller_id",
            "source_frame_id",
            "source_generation_id",
            "world_frame_id",
            "world_generation_id",
        ):
            _identifier(getattr(self, name), name)
        _coordinate(self.tx_mm, "tx_mm")
        _coordinate(self.ty_mm, "ty_mm")
        _heading(self.yaw_mdeg, "yaw_mdeg")
        _integer(
            self.position_uncertainty_mm,
            "position_uncertainty_mm",
            minimum=0,
            maximum=MAX_POSITION_UNCERTAINTY_MM,
        )
        _integer(
            self.yaw_uncertainty_mdeg,
            "yaw_uncertainty_mdeg",
            minimum=0,
            maximum=MAX_YAW_UNCERTAINTY_MDEG,
        )
        if (
            not isinstance(self.provenance, tuple)
            or not self.provenance
            or len(self.provenance) > 32
        ):
            raise FrameTransformError(
                "invalid_frame_transform_provenance",
                "Frame transform provenance is invalid",
            )
        for value in self.provenance:
            _identifier(value, "provenance")
        if len(set(self.provenance)) != len(self.provenance):
            raise FrameTransformError(
                "invalid_frame_transform_provenance",
                "Frame transform provenance is invalid",
            )

    def _require_source(
        self,
        *,
        source_robot_id: str,
        source_controller_id: str,
        source_frame_id: str,
        source_generation_id: str,
    ) -> None:
        presented = (
            source_robot_id,
            source_controller_id,
            source_frame_id,
            source_generation_id,
        )
        expected = (
            self.source_robot_id,
            self.source_controller_id,
            self.source_frame_id,
            self.source_generation_id,
        )
        if presented != expected:
            raise FrameTransformError(
                "source_frame_mismatch",
                "Source identity does not match frame calibration",
            )

    def _require_world(
        self,
        *,
        world_frame_id: str,
        world_generation_id: str,
    ) -> None:
        if (
            world_frame_id != self.world_frame_id
            or world_generation_id != self.world_generation_id
        ):
            raise FrameTransformError(
                "world_frame_mismatch",
                "World identity does not match frame calibration",
            )

    def to_world_point(
        self,
        x_mm: int,
        y_mm: int,
        *,
        source_robot_id: str,
        source_controller_id: str,
        source_frame_id: str,
        source_generation_id: str,
    ) -> Tuple[int, int]:
        self._require_source(
            source_robot_id=source_robot_id,
            source_controller_id=source_controller_id,
            source_frame_id=source_frame_id,
            source_generation_id=source_generation_id,
        )
        x_mm = _coordinate(x_mm, "x_mm")
        y_mm = _coordinate(y_mm, "y_mm")
        rotated_x, rotated_y = _rotate(x_mm, y_mm, self.yaw_mdeg)
        return (
            _coordinate(self.tx_mm + rotated_x, "world_x_mm"),
            _coordinate(self.ty_mm + rotated_y, "world_y_mm"),
        )

    def to_world_heading(
        self,
        heading_mdeg: int,
        *,
        source_robot_id: str,
        source_controller_id: str,
        source_frame_id: str,
        source_generation_id: str,
    ) -> int:
        self._require_source(
            source_robot_id=source_robot_id,
            source_controller_id=source_controller_id,
            source_frame_id=source_frame_id,
            source_generation_id=source_generation_id,
        )
        return normalize_heading_mdeg(
            _heading(heading_mdeg, "heading_mdeg") + self.yaw_mdeg
        )

    def to_world_pose(
        self,
        pose: PhysicalPose,
        *,
        source_robot_id: str,
        source_controller_id: str,
        source_frame_id: str,
        source_generation_id: str,
    ) -> PhysicalPose:
        if not isinstance(pose, PhysicalPose):
            raise FrameTransformError(
                "invalid_frame_transform_pose",
                "Physical pose is invalid",
            )
        identity = {
            "source_robot_id": source_robot_id,
            "source_controller_id": source_controller_id,
            "source_frame_id": source_frame_id,
            "source_generation_id": source_generation_id,
        }
        x_mm, y_mm = self.to_world_point(pose.x_mm, pose.y_mm, **identity)
        return PhysicalPose(
            x_mm=x_mm,
            y_mm=y_mm,
            heading_mdeg=self.to_world_heading(
                pose.heading_mdeg,
                **identity,
            ),
            verified_motion_count=pose.verified_motion_count,
            total_forward_mm=pose.total_forward_mm,
            total_turn_mdeg=pose.total_turn_mdeg,
        )

    def to_source_point(
        self,
        x_mm: int,
        y_mm: int,
        *,
        world_frame_id: str,
        world_generation_id: str,
    ) -> Tuple[int, int]:
        self._require_world(
            world_frame_id=world_frame_id,
            world_generation_id=world_generation_id,
        )
        translated_x = _coordinate(x_mm, "world_x_mm") - self.tx_mm
        translated_y = _coordinate(y_mm, "world_y_mm") - self.ty_mm
        source_x, source_y = _rotate(
            translated_x,
            translated_y,
            -self.yaw_mdeg,
        )
        return (
            _coordinate(source_x, "source_x_mm"),
            _coordinate(source_y, "source_y_mm"),
        )

    def to_source_heading(
        self,
        heading_mdeg: int,
        *,
        world_frame_id: str,
        world_generation_id: str,
    ) -> int:
        self._require_world(
            world_frame_id=world_frame_id,
            world_generation_id=world_generation_id,
        )
        return normalize_heading_mdeg(
            _heading(heading_mdeg, "heading_mdeg") - self.yaw_mdeg
        )

    def then(
        self,
        outer: "CalibratedFrameTransform",
    ) -> "CalibratedFrameTransform":
        """Compose ``self`` (source to intermediate) with ``outer``."""

        if not isinstance(outer, CalibratedFrameTransform):
            raise FrameTransformError(
                "invalid_frame_transform_composition",
                "Outer frame transform is invalid",
            )
        if (
            outer.source_robot_id != self.source_robot_id
            or outer.source_controller_id != self.source_controller_id
            or outer.source_frame_id != self.world_frame_id
            or outer.source_generation_id != self.world_generation_id
        ):
            raise FrameTransformError(
                "frame_transform_composition_mismatch",
                "Frame transforms do not share an intermediate identity",
            )
        rotated_x, rotated_y = _rotate(
            self.tx_mm,
            self.ty_mm,
            outer.yaw_mdeg,
        )
        provenance = tuple(dict.fromkeys(
            self.provenance + outer.provenance
        ))
        return CalibratedFrameTransform(
            source_robot_id=self.source_robot_id,
            source_controller_id=self.source_controller_id,
            source_frame_id=self.source_frame_id,
            source_generation_id=self.source_generation_id,
            world_frame_id=outer.world_frame_id,
            world_generation_id=outer.world_generation_id,
            tx_mm=_coordinate(
                outer.tx_mm + rotated_x,
                "composed_tx_mm",
            ),
            ty_mm=_coordinate(
                outer.ty_mm + rotated_y,
                "composed_ty_mm",
            ),
            yaw_mdeg=normalize_heading_mdeg(
                self.yaw_mdeg + outer.yaw_mdeg
            ),
            position_uncertainty_mm=_integer(
                self.position_uncertainty_mm
                + outer.position_uncertainty_mm,
                "composed_position_uncertainty_mm",
                minimum=0,
                maximum=MAX_POSITION_UNCERTAINTY_MM,
            ),
            yaw_uncertainty_mdeg=_integer(
                self.yaw_uncertainty_mdeg
                + outer.yaw_uncertainty_mdeg,
                "composed_yaw_uncertainty_mdeg",
                minimum=0,
                maximum=MAX_YAW_UNCERTAINTY_MDEG,
            ),
            provenance=provenance,
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "source_robot_id": self.source_robot_id,
            "source_controller_id": self.source_controller_id,
            "source_frame_id": self.source_frame_id,
            "source_generation_id": self.source_generation_id,
            "world_frame_id": self.world_frame_id,
            "world_generation_id": self.world_generation_id,
            "tx_mm": self.tx_mm,
            "ty_mm": self.ty_mm,
            "yaw_mdeg": self.yaw_mdeg,
            "position_uncertainty_mm": self.position_uncertainty_mm,
            "yaw_uncertainty_mdeg": self.yaw_uncertainty_mdeg,
            "provenance": list(self.provenance),
        }


__all__ = (
    "CalibratedFrameTransform",
    "FrameTransformError",
    "MAX_FRAME_COORDINATE_MM",
    "MAX_POSITION_UNCERTAINTY_MM",
    "MAX_YAW_UNCERTAINTY_MDEG",
)
