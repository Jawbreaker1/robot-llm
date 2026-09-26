"""Conservative local odometry derived from clean encoder observations.

Command conformance and encoder evidence are deliberately separate: a wheel
may undertravel while its before/after encoder readings still describe the
robot's best-known physical motion.
"""

from dataclasses import dataclass
import math
from typing import Mapping, Tuple

from .physical_navigation_contract import (
    ADVANCE,
    MOTION_ACTIONS,
    REVERSE,
    SCAN_TURN_OPERATION,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    PhysicalNavigationContractError,
    expected_scan_turn_spec,
)


def normalize_heading_mdeg(value: int) -> int:
    return int((int(round(value)) + 180_000) % 360_000 - 180_000)


@dataclass(frozen=True)
class OdometryCalibration:
    linear_mm_per_encoder_degree: float = 0.35
    turn_mdeg_per_opposed_encoder_degree: int = 132

    def __post_init__(self) -> None:
        if (
            isinstance(self.linear_mm_per_encoder_degree, bool)
            or not isinstance(
                self.linear_mm_per_encoder_degree,
                (int, float),
            )
            or not math.isfinite(self.linear_mm_per_encoder_degree)
            or self.linear_mm_per_encoder_degree <= 0
        ):
            raise PhysicalNavigationContractError(
                "invalid_linear_calibration",
                "Linear odometry calibration is invalid",
            )
        if (
            isinstance(self.turn_mdeg_per_opposed_encoder_degree, bool)
            or not isinstance(
                self.turn_mdeg_per_opposed_encoder_degree,
                int,
            )
            or self.turn_mdeg_per_opposed_encoder_degree <= 0
        ):
            raise PhysicalNavigationContractError(
                "invalid_turn_calibration",
                "Turn odometry calibration is invalid",
            )


@dataclass(frozen=True)
class DriveMotorRoles:
    """Logical side-to-configured-role mapping, injected from robot config."""

    left: str = "left_drive"
    right: str = "right_drive"

    def __post_init__(self) -> None:
        if (
            not isinstance(self.left, str)
            or not self.left
            or not isinstance(self.right, str)
            or not self.right
            or self.left == self.right
            or any(
                len(value) > 64
                or any(character in value for character in "\x00\r\n")
                for value in (self.left, self.right)
            )
        ):
            raise PhysicalNavigationContractError(
                "invalid_drive_motor_roles",
                "Drive motor role mapping is invalid",
            )

    def to_dict(self) -> Mapping[str, str]:
        return {"left": self.left, "right": self.right}


@dataclass(frozen=True)
class PhysicalPose:
    x_mm: int = 0
    y_mm: int = 0
    heading_mdeg: int = 0
    verified_motion_count: int = 0
    total_forward_mm: int = 0
    total_turn_mdeg: int = 0

    def __post_init__(self) -> None:
        for name in (
            "x_mm",
            "y_mm",
            "heading_mdeg",
            "verified_motion_count",
            "total_forward_mm",
            "total_turn_mdeg",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise PhysicalNavigationContractError(
                    "invalid_pose",
                    "Physical pose is invalid",
                )
        if not -180_000 <= self.heading_mdeg <= 179_999:
            raise PhysicalNavigationContractError(
                "invalid_heading",
                "Physical pose heading is invalid",
            )
        if (
            self.verified_motion_count < 0
            or self.total_forward_mm < 0
            or self.total_turn_mdeg < 0
        ):
            raise PhysicalNavigationContractError(
                "invalid_pose_totals",
                "Physical pose totals are invalid",
            )

    def to_dict(self) -> Mapping[str, int]:
        return {
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
            "heading_mdeg": self.heading_mdeg,
            "verified_motion_count": self.verified_motion_count,
            "total_forward_mm": self.total_forward_mm,
            "total_turn_mdeg": self.total_turn_mdeg,
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]):
        if not isinstance(value, dict) or set(value) != {
            "x_mm",
            "y_mm",
            "heading_mdeg",
            "verified_motion_count",
            "total_forward_mm",
            "total_turn_mdeg",
        }:
            raise PhysicalNavigationContractError(
                "invalid_pose_fields",
                "Physical pose fields are invalid",
            )
        return cls(**value)


@dataclass(frozen=True)
class EncoderMotionSegment:
    left_encoder_delta_degrees: int
    right_encoder_delta_degrees: int
    command_verified: bool
    status: str
    kind: str = "commanded"


@dataclass(frozen=True)
class VerifiedMotion:
    action: str
    left_encoder_delta_degrees: int
    right_encoder_delta_degrees: int
    verified_slice_count: int
    requested_slice_count: int
    status: str
    segments: Tuple[EncoderMotionSegment, ...] = ()
    requested_turn_mdeg: int = 0

    @property
    def complete(self) -> bool:
        return (
            self.status == "completed"
            and self.verified_slice_count == self.requested_slice_count
        )

    @property
    def observed_slice_count(self) -> int:
        return len(self.segments)

    def to_dict(self) -> Mapping[str, object]:
        return {
            "action": self.action,
            "left_encoder_delta_degrees": (
                self.left_encoder_delta_degrees
            ),
            "right_encoder_delta_degrees": (
                self.right_encoder_delta_degrees
            ),
            "verified_slice_count": self.verified_slice_count,
            "observed_slice_count": self.observed_slice_count,
            "requested_slice_count": self.requested_slice_count,
            "command_completed": self.complete,
        }


def _motor_bounds(
    receipt: Mapping[str, object],
    role: str,
    side: str,
) -> Tuple[int, int, int]:
    motors = receipt.get("motors")
    if not isinstance(motors, list):
        raise PhysicalNavigationContractError(
            "invalid_slice_motors",
            "Motion slice had no motor receipts",
        )
    matches = [
        item
        for item in motors
        if isinstance(item, dict) and item.get("role") == role
    ]
    if len(matches) != 1:
        raise PhysicalNavigationContractError(
            "invalid_slice_motor_role",
            "Motion slice did not contain one receipt per drive motor",
        )
    motor = matches[0]
    required = {
        "side",
        "role",
        "position_before",
        "position_after",
        "position_delta",
        "state",
    }
    if set(motor) != required:
        raise PhysicalNavigationContractError(
            "invalid_slice_motor_fields",
            "Motion slice motor fields are invalid",
        )
    if motor["side"] != side:
        raise PhysicalNavigationContractError(
            "drive_side_role_mismatch",
            "Configured drive role was reported for the wrong side",
        )
    before = motor["position_before"]
    after = motor["position_after"]
    delta = motor["position_delta"]
    if any(
        isinstance(value, bool) or not isinstance(value, int)
        for value in (before, after, delta)
    ) or after - before != delta:
        raise PhysicalNavigationContractError(
            "invalid_encoder_receipt",
            "Encoder receipt is inconsistent",
        )
    return before, after, delta


def _motor_delta(
    receipt: Mapping[str, object],
    role: str,
    side: str,
) -> int:
    return _motor_bounds(receipt, role, side)[2]


def verified_motion_from_result(
    action: str,
    result: Mapping[str, object],
    drive_roles: DriveMotorRoles = DriveMotorRoles(),
) -> VerifiedMotion:
    """Return the contiguous observable prefix; reject ambiguous receipts."""

    is_scan_turn = action == SCAN_TURN_OPERATION
    if action not in MOTION_ACTIONS and not is_scan_turn:
        raise PhysicalNavigationContractError(
            "invalid_motion_action",
            "Odometry can only consume a motion action",
        )
    identity = "relative_delta_mdeg" if is_scan_turn else "action"
    if (
        not isinstance(result, dict)
        or set(result) != {identity, "outcome", "observation", "stop"}
        or (not is_scan_turn and result[identity] != action)
    ):
        raise PhysicalNavigationContractError(
            "motion_result_correlation_mismatch",
            "Motion result does not match the requested action",
        )
    stop = result["stop"]
    if (
        not isinstance(stop, dict)
        or stop.get("stop_confirmed") is not True
        or stop.get("errors") not in ([], None)
        or stop.get("fault_tokens") not in ([], {}, None)
        or stop.get("cleanup_errors", []) != []
    ):
        raise PhysicalNavigationContractError(
            "motion_stop_not_verified",
            "Motion result did not end in a clean verified stop",
        )
    requested_turn = result[identity] if is_scan_turn else 0
    if is_scan_turn:
        expected_scan_turn_spec(requested_turn)
    outcome = result["outcome"]
    outcome_identity = "requested_relative_delta_mdeg" if is_scan_turn else "action"
    if (
        not isinstance(outcome, dict)
        or outcome.get(outcome_identity) != result[identity]
        or (is_scan_turn and outcome.get("kind") != SCAN_TURN_OPERATION)
    ):
        raise PhysicalNavigationContractError(
            "invalid_motion_outcome",
            "Motion outcome is invalid",
        )
    status = outcome.get("status")
    if status not in (
        "completed",
        "interrupted",
        "denied",
        "verification_failed",
    ):
        raise PhysicalNavigationContractError(
            "invalid_motion_status",
            "Motion outcome status is invalid",
        )
    requested = outcome.get("requested_slice_count")
    completed = outcome.get("completed_slice_count")
    if (
        isinstance(requested, bool)
        or not isinstance(requested, int)
        or requested <= 0
        or isinstance(completed, bool)
        or not isinstance(completed, int)
        or not 0 <= completed <= requested
    ):
        raise PhysicalNavigationContractError(
            "invalid_slice_accounting",
            "Motion slice accounting is invalid",
        )
    slices = outcome.get("slices")
    if not isinstance(slices, list) or len(slices) > requested:
        raise PhysicalNavigationContractError(
            "invalid_motion_slices",
            "Motion slices are invalid",
        )

    segments = []
    verified_count = 0
    previous_segment_after = None
    for expected_index, receipt in enumerate(slices, 1):
        if not isinstance(receipt, dict):
            raise PhysicalNavigationContractError(
                "invalid_motion_slice",
                "Motion slice is invalid",
            )
        if (
            receipt.get("slice_index") != expected_index
            or receipt.get("slice_count") != requested
        ):
            raise PhysicalNavigationContractError(
                "noncontiguous_motion_slices",
                "Motion receipts must form a contiguous prefix",
            )
        verification = receipt.get("encoder_verification")
        receipt_stop = receipt.get("stop")
        if (
            not isinstance(verification, dict)
            or type(verification.get("passed")) is not bool
            or not isinstance(receipt_stop, dict)
            or receipt_stop.get("stop_confirmed") is not True
            or receipt_stop.get("errors") not in ([], None)
            or receipt_stop.get("fault_tokens") not in ([], {}, None)
        ):
            break
        motors = receipt.get("motors")
        if motors == []:
            if receipt.get("status") != "denied":
                raise PhysicalNavigationContractError(
                    "missing_encoder_evidence",
                    "A started motion receipt lacks encoder evidence",
                )
            break
        command_verified = verification.get("passed") is True
        outer_left_delta = _motor_delta(
            receipt,
            drive_roles.left,
            "left",
        )
        outer_right_delta = _motor_delta(
            receipt,
            drive_roles.right,
            "right",
        )
        segment_start = len(segments)
        temporal = receipt.get("segments")
        if temporal is None:
            temporal = [receipt]
        elif not isinstance(temporal, list) or not temporal:
            raise PhysicalNavigationContractError(
                "invalid_motion_segments",
                "Started motion lacks temporal encoder segments",
            )
        for segment in temporal:
            if not isinstance(segment, dict):
                raise PhysicalNavigationContractError(
                    "invalid_motion_segment",
                    "Temporal encoder segment is invalid",
                )
            segment_stop = segment.get("stop")
            segment_verification = segment.get("encoder_verification")
            if (
                not isinstance(segment_stop, dict)
                or segment_stop.get("stop_confirmed") is not True
                or segment_stop.get("errors") not in ([], None)
                or segment_stop.get("fault_tokens") not in ([], {}, None)
                or not isinstance(segment_verification, dict)
                or type(segment_verification.get("passed")) is not bool
            ):
                raise PhysicalNavigationContractError(
                    "unverified_motion_segment",
                    "Temporal encoder segment lacks clean evidence",
                )
            left_before, left_after, left_delta = _motor_bounds(
                segment,
                drive_roles.left,
                "left",
            )
            right_before, right_after, right_delta = _motor_bounds(
                segment,
                drive_roles.right,
                "right",
            )
            segment_before = {"left": left_before, "right": right_before}
            if previous_segment_after is not None and segment_before != previous_segment_after:
                raise PhysicalNavigationContractError(
                    "encoder_position_discontinuity",
                    "Temporal encoder segments are not continuous",
                )
            previous_segment_after = {"left": left_after, "right": right_after}
            segments.append(
                EncoderMotionSegment(
                    left_encoder_delta_degrees=left_delta,
                    right_encoder_delta_degrees=right_delta,
                    command_verified=(
                        segment_verification.get("passed") is True
                    ),
                    status=segment.get("status"),
                    kind=segment.get("kind", "commanded"),
                )
            )
        added = segments[segment_start:]
        if (
            sum(
                segment.left_encoder_delta_degrees
                for segment in added
            )
            != outer_left_delta
            or sum(
                segment.right_encoder_delta_degrees
                for segment in added
            )
            != outer_right_delta
        ):
            raise PhysicalNavigationContractError(
                "motion_segment_aggregate_mismatch",
                "Temporal encoder segments do not match the slice total",
            )
        if command_verified:
            verified_count += 1
        if receipt.get("status") != "completed":
            break

    if completed > verified_count:
        raise PhysicalNavigationContractError(
            "unverified_completed_slice",
            "Worker counted an unverified motion slice as completed",
        )
    verification = outcome.get("encoder_verification")
    if (
        not isinstance(verification, dict)
        or verification.get("requested_slice_count") != requested
        or verification.get("verified_slice_count") != verified_count
    ):
        raise PhysicalNavigationContractError(
            "invalid_top_level_verification",
            "Top-level encoder verification is inconsistent",
        )
    if status == "completed" and (
        verification.get("passed") is not True
        or verified_count != requested
        or completed != requested
    ):
        raise PhysicalNavigationContractError(
            "incomplete_completed_motion",
            "Completed motion lacks complete encoder verification",
        )
    return VerifiedMotion(
        action=action,
        left_encoder_delta_degrees=sum(
            segment.left_encoder_delta_degrees
            for segment in segments
        ),
        right_encoder_delta_degrees=sum(
            segment.right_encoder_delta_degrees
            for segment in segments
        ),
        verified_slice_count=verified_count,
        requested_slice_count=requested,
        status=status,
        segments=tuple(segments),
        requested_turn_mdeg=requested_turn,
    )


def _validate_direction(action: str, left: int, right: int) -> None:
    valid = {
        ADVANCE: left >= 0 and right >= 0,
        REVERSE: left <= 0 and right <= 0,
        TURN_LEFT_90: left <= 0 and right >= 0,
        TURN_RIGHT_90: left >= 0 and right <= 0,
    }[action]
    if not valid:
        raise PhysicalNavigationContractError(
            "encoder_direction_mismatch",
            "Verified encoder direction contradicts the semantic action",
        )


def apply_verified_motion(
    pose: PhysicalPose,
    motion: VerifiedMotion,
    calibration: OdometryCalibration = OdometryCalibration(),
    *,
    max_uncommanded_drift_degrees: int = 0,
) -> PhysicalPose:
    """Apply clean, direction-consistent encoder evidence to local odometry."""

    if (
        type(max_uncommanded_drift_degrees) is not int
        or max_uncommanded_drift_degrees < 0
    ):
        raise PhysicalNavigationContractError(
            "invalid_uncommanded_drift_limit",
            "Uncommanded encoder drift limit is invalid",
        )

    segments = motion.segments
    if not segments and motion.verified_slice_count:
        segments = (
            EncoderMotionSegment(
                left_encoder_delta_degrees=(
                    motion.left_encoder_delta_degrees
                ),
                right_encoder_delta_degrees=(
                    motion.right_encoder_delta_degrees
                ),
                command_verified=motion.complete,
                status=motion.status,
                kind="commanded",
            ),
        )
    if not segments:
        return pose
    if not any(
        segment.left_encoder_delta_degrees
        or segment.right_encoder_delta_degrees
        for segment in segments
    ):
        return pose
    x_mm = pose.x_mm
    y_mm = pose.y_mm
    heading_mdeg = pose.heading_mdeg
    travelled_mm = 0
    turned_mdeg = 0
    for segment in segments:
        left = segment.left_encoder_delta_degrees
        right = segment.right_encoder_delta_degrees
        uncommanded_settling = segment.kind in {
            "inter_action_settling",
            "inter_slice_settling",
        }
        if uncommanded_settling:
            bounded_uncommanded_drift = (
                max_uncommanded_drift_degrees > 0
                and segment.command_verified is False
                and abs(left) <= max_uncommanded_drift_degrees
                and abs(right) <= max_uncommanded_drift_degrees
            )
            if not bounded_uncommanded_drift:
                raise PhysicalNavigationContractError(
                    "encoder_direction_mismatch",
                    "Uncommanded encoder drift exceeds its allowance",
                )
        else:
            direction_action = motion.action
            if motion.action == SCAN_TURN_OPERATION:
                direction_action = (
                    TURN_LEFT_90 if motion.requested_turn_mdeg > 0 else TURN_RIGHT_90
                )
            _validate_direction(direction_action, left, right)
        x_mm, y_mm, heading_mdeg, segment_travelled, segment_turned = (
            _integrate_encoder_delta(
                x_mm,
                y_mm,
                heading_mdeg,
                left,
                right,
                calibration,
            )
        )
        travelled_mm += segment_travelled
        turned_mdeg += segment_turned
    return PhysicalPose(
        x_mm=x_mm,
        y_mm=y_mm,
        heading_mdeg=heading_mdeg,
        verified_motion_count=pose.verified_motion_count + 1,
        total_forward_mm=pose.total_forward_mm + travelled_mm,
        total_turn_mdeg=pose.total_turn_mdeg + turned_mdeg,
    )


def _integrate_encoder_delta(
    x_mm: int,
    y_mm: int,
    heading_mdeg: int,
    left_encoder_delta_degrees: int,
    right_encoder_delta_degrees: int,
    calibration: OdometryCalibration,
) -> Tuple[int, int, int, int, int]:
    """Integrate one pair of absolute encoder deltas into the local pose."""

    center_mm = (
        left_encoder_delta_degrees + right_encoder_delta_degrees
    ) / 2.0 * calibration.linear_mm_per_encoder_degree
    delta_mdeg = int(
        round(
            (right_encoder_delta_degrees - left_encoder_delta_degrees)
            / 2.0
            * calibration.turn_mdeg_per_opposed_encoder_degree
        )
    )
    heading_radians = math.radians(heading_mdeg / 1000.0)
    delta_radians = math.radians(delta_mdeg / 1000.0)
    if delta_mdeg == 0:
        delta_x = math.cos(heading_radians) * center_mm
        delta_y = math.sin(heading_radians) * center_mm
    else:
        radius_mm = center_mm / delta_radians
        delta_x = radius_mm * (
            math.sin(heading_radians + delta_radians)
            - math.sin(heading_radians)
        )
        delta_y = -radius_mm * (
            math.cos(heading_radians + delta_radians)
            - math.cos(heading_radians)
        )
    return (
        x_mm + int(round(delta_x)),
        y_mm + int(round(delta_y)),
        normalize_heading_mdeg(heading_mdeg + delta_mdeg),
        abs(int(round(center_mm))),
        abs(delta_mdeg),
    )


def apply_encoder_observation_delta(
    pose: PhysicalPose,
    left_encoder_delta_degrees: int,
    right_encoder_delta_degrees: int,
    calibration: OdometryCalibration = OdometryCalibration(),
) -> PhysicalPose:
    """Apply movement found by comparing two absolute encoder observations.

    This is intentionally not counted as a commanded motion. It closes the
    gap between worker receipts while keeping the same physical integration as
    normal verified motion.
    """

    if any(
        isinstance(value, bool) or not isinstance(value, int)
        for value in (left_encoder_delta_degrees, right_encoder_delta_degrees)
    ):
        raise PhysicalNavigationContractError(
            "invalid_encoder_observation_delta",
            "Observed encoder deltas are invalid",
        )
    x_mm, y_mm, heading_mdeg, travelled_mm, turned_mdeg = (
        _integrate_encoder_delta(
            pose.x_mm,
            pose.y_mm,
            pose.heading_mdeg,
            left_encoder_delta_degrees,
            right_encoder_delta_degrees,
            calibration,
        )
    )
    return PhysicalPose(
        x_mm=x_mm,
        y_mm=y_mm,
        heading_mdeg=heading_mdeg,
        verified_motion_count=pose.verified_motion_count,
        total_forward_mm=pose.total_forward_mm + travelled_mm,
        total_turn_mdeg=pose.total_turn_mdeg + turned_mdeg,
    )


def nominal_effect(
    pose: PhysicalPose,
    action: str,
    action_specs: Mapping[str, Mapping[str, object]],
    calibration: OdometryCalibration = OdometryCalibration(),
    accepted_encoder_ratio: float = 1.75,
) -> Tuple[PhysicalPose, PhysicalPose]:
    """Return nominal and conservative maximum-travel action endpoints."""

    if action not in MOTION_ACTIONS or action not in action_specs:
        raise PhysicalNavigationContractError(
            "unknown_action_spec",
            "Action has no physical motion specification",
        )
    spec = action_specs[action]
    left = int(
        round(spec["left_speed_dps"] * spec["total_duration_ms"] / 1000.0)
    )
    right = int(
        round(spec["right_speed_dps"] * spec["total_duration_ms"] / 1000.0)
    )
    target = spec.get("target_mean_abs_encoder_degrees")
    if target is not None:
        left = (1 if left >= 0 else -1) * int(target)
        right = (1 if right >= 0 else -1) * int(target)
    nominal_motion = VerifiedMotion(
        action,
        left,
        right,
        1,
        1,
        "completed",
    )
    maximum_motion = VerifiedMotion(
        action,
        int(round(left * accepted_encoder_ratio)),
        int(round(right * accepted_encoder_ratio)),
        1,
        1,
        "completed",
    )
    return (
        apply_verified_motion(pose, nominal_motion, calibration),
        apply_verified_motion(pose, maximum_motion, calibration),
    )
