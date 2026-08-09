"""Typed BLAST navigation facts, deliberately not wired into runtime yet."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from .physical_footprint import RobotFootprint
from .physical_odometry import OdometryCalibration


BLAST_NAVIGATION_EVIDENCE_ID = (
    "EXP-BLAST-NAV-CALIBRATION-20260808-001"
)


def _text(value: object) -> bool:
    return (
        isinstance(value, str)
        and value == value.strip()
        and 0 < len(value) <= 512
        and all(ord(character) >= 32 for character in value)
    )


@dataclass(frozen=True)
class BlastRangeSensorExtrinsics:
    """Planar ultrasonic pose relative to the differential-drive origin."""

    forward_offset_mm: Optional[int]
    left_offset_mm: Optional[int]
    yaw_mdeg: Optional[int]
    calibration_status: str
    calibration_evidence: str
    navigation_body_motor_angle_deg: Optional[int] = None
    navigation_body_motor_tolerance_deg: int = 0

    def __post_init__(self) -> None:
        values = (
            self.forward_offset_mm,
            self.left_offset_mm,
            self.yaw_mdeg,
        )
        measured = tuple(value is not None for value in values)
        if any(measured) and not all(measured):
            raise ValueError("BLAST range-sensor extrinsics are partial")
        if all(measured):
            forward, left, yaw = values
            if (
                any(type(value) is not int for value in values)
                or not -1_000 <= forward <= 1_000
                or not -1_000 <= left <= 1_000
                or not -180_000 <= yaw <= 179_999
            ):
                raise ValueError("BLAST range-sensor extrinsics are invalid")
        angle = self.navigation_body_motor_angle_deg
        if angle is not None and (
            type(angle) is not int
            or not -1_000_000 <= angle <= 1_000_000
        ):
            raise ValueError("BLAST navigation body reference is invalid")
        tolerance = self.navigation_body_motor_tolerance_deg
        if type(tolerance) is not int or not 0 <= tolerance <= 10:
            raise ValueError("BLAST navigation body tolerance is invalid")
        if not _text(self.calibration_status) or not _text(
            self.calibration_evidence
        ):
            raise ValueError("BLAST range-sensor provenance is invalid")

    @property
    def complete(self) -> bool:
        return self.forward_offset_mm is not None

    @property
    def navigation_body_reference_complete(self) -> bool:
        return self.navigation_body_motor_angle_deg is not None

    def matches_navigation_body_angle(self, value: object) -> bool:
        if (
            not self.navigation_body_reference_complete
            or type(value) is not int
        ):
            return False
        assert self.navigation_body_motor_angle_deg is not None
        return (
            abs(value - self.navigation_body_motor_angle_deg)
            <= self.navigation_body_motor_tolerance_deg
        )


@dataclass(frozen=True)
class BlastNavigationCalibration:
    """Known motion scale plus geometry required before route activation."""

    odometry: OdometryCalibration
    odometry_status: str
    odometry_evidence: str
    robot_footprint: Optional[RobotFootprint]
    footprint_status: str
    footprint_evidence: str
    range_sensor_extrinsics: BlastRangeSensorExtrinsics
    evidence_id: str

    def __post_init__(self) -> None:
        if (
            not isinstance(self.odometry, OdometryCalibration)
            or (
                self.robot_footprint is not None
                and not isinstance(self.robot_footprint, RobotFootprint)
            )
            or not isinstance(
                self.range_sensor_extrinsics,
                BlastRangeSensorExtrinsics,
            )
            or any(
                not _text(value)
                for value in (
                    self.odometry_status,
                    self.odometry_evidence,
                    self.footprint_status,
                    self.footprint_evidence,
                    self.evidence_id,
                )
            )
        ):
            raise ValueError("BLAST navigation calibration is invalid")

    @property
    def complete(self) -> bool:
        return (
            self.robot_footprint is not None
            and self.range_sensor_extrinsics.complete
        )

    def require_complete(
        self,
    ) -> Tuple[RobotFootprint, BlastRangeSensorExtrinsics]:
        if not self.complete:
            raise ValueError("BLAST navigation geometry is incomplete")
        assert self.robot_footprint is not None
        return self.robot_footprint, self.range_sensor_extrinsics

    def minimum_rotation_clearance_mm(self) -> int:
        """Return the provisional front-range margin for body rotation."""

        footprint, sensor = self.require_complete()
        assert sensor.forward_offset_mm is not None
        return max(0, math.ceil(
            footprint.maximum_corner_radius_mm
            + footprint.clearance_margin_mm
            - sensor.forward_offset_mm
        ))


BLAST_PROVISIONAL_NAVIGATION_CALIBRATION = BlastNavigationCalibration(
    odometry=OdometryCalibration(
        linear_mm_per_encoder_degree=0.5,
        turn_mdeg_per_opposed_encoder_degree=490,
    ),
    odometry_status="provisional-live-encoder-and-reference-derived",
    odometry_evidence=(
        "Two 90-degree drive pulses changed range 358->313->268 mm; "
        "one four-pulse left turn used 194 actual opposed encoder degrees "
        "for approximately 95 body degrees and the mirrored right turn "
        "returned to the physical start pose"
    ),
    robot_footprint=RobotFootprint(
        front_extent_mm=110,
        rear_extent_mm=60,
        left_extent_mm=105,
        right_extent_mm=100,
        clearance_margin_mm=10,
        calibration_status="measured-approximate-current-build",
        calibration_evidence=(
            "Operator folding-rule measurement from the differential-drive "
            "origin; 10 mm margin is separate from the measured extents"
        ),
    ),
    footprint_status="measured-approximate-current-build",
    footprint_evidence=(
        "Operator measured front 110, rear 60, left 105 and right 100 mm "
        "from the differential-drive origin with a folding rule"
    ),
    range_sensor_extrinsics=BlastRangeSensorExtrinsics(
        forward_offset_mm=110,
        left_offset_mm=80,
        yaw_mdeg=0,
        calibration_status="measured-approximate-navigation-pose",
        calibration_evidence=(
            "Operator folding-rule measurement to the ultrasonic face "
            "centre with the left arm in navigation pose; the body encoder "
            "remained exactly 158 degrees across a hub power cycle while "
            "the operator confirmed the sensor still faced nearly forward. "
            "A later restored side scan was rejected by exact per-ray "
            "comparison although its final settled body reading was 158; "
            "one degree is the smallest non-zero provisional allowance"
        ),
        navigation_body_motor_angle_deg=158,
        navigation_body_motor_tolerance_deg=1,
    ),
    evidence_id=BLAST_NAVIGATION_EVIDENCE_ID,
)


__all__ = (
    "BLAST_NAVIGATION_EVIDENCE_ID",
    "BLAST_PROVISIONAL_NAVIGATION_CALIBRATION",
    "BlastNavigationCalibration",
    "BlastRangeSensorExtrinsics",
)
