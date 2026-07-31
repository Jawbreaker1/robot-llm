"""Conservative local odometry derived only from verified encoder receipts."""

from dataclasses import dataclass
import math
from typing import Mapping, Tuple

from .physical_navigation_contract import (
    ADVANCE,
    MOTION_ACTIONS,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    PhysicalNavigationContractError,
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
class VerifiedMotion:
    action: str
    left_encoder_delta_degrees: int
    right_encoder_delta_degrees: int
    verified_slice_count: int
    requested_slice_count: int
    status: str

    @property
    def complete(self) -> bool:
        return (
            self.status == "completed"
            and self.verified_slice_count == self.requested_slice_count
        )


def _motor_delta(
    receipt: Mapping[str, object],
    role: str,
    side: str,
) -> int:
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
    return delta


def verified_motion_from_result(
    action: str,
    result: Mapping[str, object],
    drive_roles: DriveMotorRoles = DriveMotorRoles(),
) -> VerifiedMotion:
    """Return the contiguous verified prefix; reject ambiguous receipts."""

    if action not in MOTION_ACTIONS:
        raise PhysicalNavigationContractError(
            "invalid_motion_action",
            "Odometry can only consume a motion action",
        )
    if (
        not isinstance(result, dict)
        or set(result) != {"action", "outcome", "observation", "stop"}
        or result["action"] != action
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
    outcome = result["outcome"]
    if not isinstance(outcome, dict) or outcome.get("action") != action:
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

    left_total = 0
    right_total = 0
    verified_count = 0
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
            or verification.get("passed") is not True
            or not isinstance(receipt_stop, dict)
            or receipt_stop.get("stop_confirmed") is not True
            or receipt_stop.get("errors") not in ([], None)
            or receipt_stop.get("fault_tokens") not in ([], {}, None)
        ):
            break
        left_total += _motor_delta(
            receipt,
            drive_roles.left,
            "left",
        )
        right_total += _motor_delta(
            receipt,
            drive_roles.right,
            "right",
        )
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
        left_encoder_delta_degrees=left_total,
        right_encoder_delta_degrees=right_total,
        verified_slice_count=verified_count,
        requested_slice_count=requested,
        status=status,
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
) -> PhysicalPose:
    """Apply only the verified prefix of a result to local odometry."""

    if motion.verified_slice_count == 0:
        return pose
    left = motion.left_encoder_delta_degrees
    right = motion.right_encoder_delta_degrees
    _validate_direction(motion.action, left, right)
    if motion.action in (ADVANCE, REVERSE):
        signed_encoder_degrees = (left + right) / 2.0
        distance_mm = int(
            round(
                signed_encoder_degrees
                * calibration.linear_mm_per_encoder_degree
            )
        )
        heading_radians = math.radians(pose.heading_mdeg / 1000.0)
        return PhysicalPose(
            x_mm=pose.x_mm
            + int(round(math.cos(heading_radians) * distance_mm)),
            y_mm=pose.y_mm
            + int(round(math.sin(heading_radians) * distance_mm)),
            heading_mdeg=pose.heading_mdeg,
            verified_motion_count=pose.verified_motion_count + 1,
            total_forward_mm=pose.total_forward_mm + abs(distance_mm),
            total_turn_mdeg=pose.total_turn_mdeg,
        )
    opposed_encoder_degrees = (right - left) / 2.0
    delta_mdeg = int(
        round(
            opposed_encoder_degrees
            * calibration.turn_mdeg_per_opposed_encoder_degree
        )
    )
    return PhysicalPose(
        x_mm=pose.x_mm,
        y_mm=pose.y_mm,
        heading_mdeg=normalize_heading_mdeg(
            pose.heading_mdeg + delta_mdeg
        ),
        verified_motion_count=pose.verified_motion_count + 1,
        total_forward_mm=pose.total_forward_mm,
        total_turn_mdeg=pose.total_turn_mdeg + abs(delta_mdeg),
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
