"""Physical EV3 rig for deterministic active infrared scanning.

The language model selects only ``SCAN_FRONT_ARC`` and a published obstacle
hypothesis.  This adapter owns the lower-level, fixed scan-turn lattice and
derives heading changes from verified encoder receipts rather than trusting
timed motor commands or worker-provided pose estimates.
"""

import time
from typing import Callable, Mapping

from .active_ir_scan import ActiveIrScanExecutor
from .active_ir_scan_contract import (
    ActiveIrScanCalibration,
    ActiveIrScanContractError,
    ActiveIrScanRequest,
)
from .ev3_navigation_transport import EV3NavigationTransportError
from .physical_navigation_contract import (
    SCAN_SAMPLE_OPERATION,
    SCAN_TURN_ALLOWED_DELTAS_MDEG,
    SCAN_TURN_OPERATION,
    SCAN_TURN_REFERENCE_BODY_MDEG,
    SCAN_TURN_REFERENCE_ENCODER_DEGREES,
    expected_scan_turn_profile,
    expected_scan_sample_profile,
    validate_observation,
)
from .physical_odometry import normalize_heading_mdeg


class EV3ActiveIrScanRig:
    """Translate fixed scan bearings into strictly verified EV3 operations."""

    def __init__(
        self,
        *,
        transport,
        clock_ms: Callable[[], int],
        cancel_requested: Callable[[], bool] = lambda: False,
        request_timeout_seconds: float = 8.0,
    ):
        if any(
            not callable(getattr(transport, name, None))
            for name in ("request", "abort")
        ):
            raise ValueError("EV3 scan transport is invalid")
        if not callable(clock_ms) or not callable(cancel_requested):
            raise ValueError("EV3 scan rig callback is invalid")
        if (
            isinstance(request_timeout_seconds, bool)
            or not isinstance(request_timeout_seconds, (int, float))
            or not 0.1 <= float(request_timeout_seconds) <= 30.0
        ):
            raise ValueError("EV3 scan request timeout is invalid")
        self.transport = transport
        self.clock_ms = clock_ms
        self.cancel_requested = cancel_requested
        self.request_timeout_seconds = float(request_timeout_seconds)
        self._heading_mdeg = None
        self._last_state_version = None
        self._last_observed_at_ms = None
        self._deadline_ms = None
        self._left_role = None
        self._right_role = None

    def bind_cancel_requested(self, cancel_requested) -> None:
        if not callable(cancel_requested):
            raise ActiveIrScanContractError(
                "invalid_scan_cancellation_probe",
                "Physical scan cancellation probe is invalid",
            )
        if self._deadline_ms is not None:
            raise ActiveIrScanContractError(
                "scan_rig_already_armed",
                "Physical scan cancellation cannot change while armed",
            )
        self.cancel_requested = cancel_requested

    def release_scan(self) -> None:
        self._heading_mdeg = None
        self._last_state_version = None
        self._last_observed_at_ms = None
        self._deadline_ms = None
        self._left_role = None
        self._right_role = None

    def _now_ms(self) -> int:
        value = self.clock_ms()
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ActiveIrScanContractError(
                "invalid_scan_clock",
                "Physical scan clock returned an invalid value",
            )
        return value

    def _cancelled(self) -> bool:
        try:
            return self.cancel_requested() is True
        except BaseException:
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_cancellation_probe_failed",
                "Physical scan cancellation state could not be read",
            ) from None

    def _require_not_cancelled(self) -> None:
        if self._cancelled():
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_cancelled",
                "Physical scan was cancelled and its SSH channel was closed",
            )

    def _description(self) -> Mapping[str, object]:
        description = getattr(self.transport, "worker_description", None)
        if (
            not isinstance(description, dict)
            or description.get("scan_turn")
            != expected_scan_turn_profile()
            or description.get("scan_sample")
            != expected_scan_sample_profile()
        ):
            raise ActiveIrScanContractError(
                "scan_worker_not_described",
                "Physical scan requires the validated worker description",
            )
        geometry = description.get("drive_geometry")
        if (
            not isinstance(geometry, dict)
            or set(geometry)
            != {
                "left_motor_role",
                "right_motor_role",
                "forward_speed_sign",
            }
            or not isinstance(geometry["left_motor_role"], str)
            or not geometry["left_motor_role"]
            or not isinstance(geometry["right_motor_role"], str)
            or not geometry["right_motor_role"]
            or geometry["left_motor_role"]
            == geometry["right_motor_role"]
        ):
            raise ActiveIrScanContractError(
                "invalid_scan_drive_geometry",
                "Physical scan drive geometry is invalid",
            )
        return description

    def begin_scan(self, request: ActiveIrScanRequest) -> None:
        if not isinstance(request, ActiveIrScanRequest):
            raise ActiveIrScanContractError(
                "invalid_scan_request",
                "Physical scan request has the wrong type",
            )
        self._require_not_cancelled()
        description = self._description()
        current_state_version = getattr(
            self.transport,
            "last_state_version",
            None,
        )
        if current_state_version != request.start_state_version:
            raise ActiveIrScanContractError(
                "scan_start_state_mismatch",
                "Worker state changed before physical scanning began",
            )
        now = self._now_ms()
        if now < request.created_at_ms or now >= request.deadline_ms:
            raise ActiveIrScanContractError(
                "scan_start_deadline",
                "Physical scan could not begin inside its validity window",
            )
        geometry = description["drive_geometry"]
        self._left_role = geometry["left_motor_role"]
        self._right_role = geometry["right_motor_role"]
        self._heading_mdeg = request.start_pose.heading_mdeg
        self._last_state_version = request.start_state_version
        self._last_observed_at_ms = request.created_at_ms
        self._deadline_ms = request.deadline_ms

    def _require_active(
        self,
        deadline_ms: int,
    ) -> None:
        if (
            self._heading_mdeg is None
            or self._last_state_version is None
            or self._deadline_ms != deadline_ms
        ):
            raise ActiveIrScanContractError(
                "scan_rig_not_armed",
                "Physical scan rig is not armed for this request",
            )
        self._require_not_cancelled()
        if self._now_ms() >= deadline_ms:
            raise ActiveIrScanContractError(
                "scan_deadline_exceeded",
                "Physical scan operation missed its deadline",
            )

    def _remaining_timeout(self, deadline_ms: int) -> float:
        remaining = (deadline_ms - self._now_ms()) / 1000.0
        if remaining < 0.1:
            raise ActiveIrScanContractError(
                "scan_deadline_exceeded",
                "Physical scan has insufficient request headroom",
            )
        return min(self.request_timeout_seconds, remaining)

    def _request(
        self,
        operation: str,
        arguments: Mapping[str, object],
        deadline_ms: int,
        *,
        cancellation_enabled: bool,
    ) -> Mapping[str, object]:
        try:
            return self.transport.request(
                operation,
                arguments,
                self._remaining_timeout(deadline_ms),
                cancel_requested=(
                    self.cancel_requested
                    if cancellation_enabled
                    else None
                ),
            )
        except EV3NavigationTransportError as error:
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_transport_failed",
                "Physical scan transport failed: {}".format(error),
            ) from None

    @staticmethod
    def _encoder_totals(
        outcome: Mapping[str, object],
        *,
        left_role: str,
        right_role: str,
    ):
        totals = {"left": 0, "right": 0}
        roles = {"left": left_role, "right": right_role}
        slices = outcome.get("slices")
        if not isinstance(slices, list) or not slices:
            raise ActiveIrScanContractError(
                "scan_encoder_receipt_missing",
                "Physical scan turn has no slice receipts",
            )
        for receipt in slices:
            if (
                not isinstance(receipt, dict)
                or receipt.get("status") != "completed"
                or receipt.get("stop", {}).get("stop_confirmed") is not True
                or receipt.get("encoder_verification", {}).get("passed")
                is not True
            ):
                raise ActiveIrScanContractError(
                    "scan_slice_unverified",
                    "Physical scan turn contains an unverified slice",
                )
            motors = receipt.get("motors")
            if not isinstance(motors, list) or len(motors) != 2:
                raise ActiveIrScanContractError(
                    "scan_encoder_receipt_invalid",
                    "Physical scan turn motor receipts are invalid",
                )
            seen = set()
            for motor in motors:
                side = motor.get("side")
                delta = motor.get("position_delta")
                if (
                    side not in totals
                    or side in seen
                    or motor.get("role") != roles[side]
                    or isinstance(delta, bool)
                    or not isinstance(delta, int)
                ):
                    raise ActiveIrScanContractError(
                        "scan_encoder_receipt_invalid",
                        "Physical scan turn motor receipt is uncorrelated",
                    )
                seen.add(side)
                totals[side] += delta
            if seen != {"left", "right"}:
                raise ActiveIrScanContractError(
                    "scan_encoder_receipt_invalid",
                    "Physical scan turn is missing one drive side",
                )
        return totals

    def turn_relative_mdeg(
        self,
        relative_delta_mdeg: int,
        calibration: ActiveIrScanCalibration,
        deadline_ms: int,
    ) -> Mapping[str, object]:
        self._require_active(deadline_ms)
        if (
            not isinstance(calibration, ActiveIrScanCalibration)
            or isinstance(relative_delta_mdeg, bool)
            or relative_delta_mdeg
            not in SCAN_TURN_ALLOWED_DELTAS_MDEG
        ):
            raise ActiveIrScanContractError(
                "invalid_scan_turn",
                "Physical scan turn is outside the fixed lattice",
            )
        response = self._request(
            SCAN_TURN_OPERATION,
            {"relative_delta_mdeg": relative_delta_mdeg},
            deadline_ms,
            cancellation_enabled=True,
        )
        state_version = response.get("state_version")
        if (
            isinstance(state_version, bool)
            or not isinstance(state_version, int)
            or state_version <= self._last_state_version
        ):
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_state_version_mismatch",
                "Physical scan turn did not advance worker state",
            )
        result = response.get("result")
        if not isinstance(result, dict):
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_turn_receipt_invalid",
                "Physical scan turn result is missing",
            )
        outcome = result.get("outcome")
        observation = validate_observation(result.get("observation"))
        if (
            not isinstance(outcome, dict)
            or outcome.get("status") != "completed"
            or outcome.get("encoder_verification", {}).get("passed")
            is not True
            or outcome.get("stop_confirmed") is not True
            or observation["state_version"] != state_version
            or observation["last_outcome"] != outcome
            or observation["touch"]["pressed"]
            or observation["budgets"]["motion_fault_latched"]
        ):
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_turn_receipt_invalid",
                "Physical scan turn was not safely completed",
            )
        totals = self._encoder_totals(
            outcome,
            left_role=self._left_role,
            right_role=self._right_role,
        )
        direction = 1 if relative_delta_mdeg > 0 else -1
        if (
            totals["left"] * direction >= 0
            or totals["right"] * direction <= 0
        ):
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_encoder_direction_mismatch",
                "Physical scan encoders disagree with requested direction",
            )
        profile = expected_scan_turn_profile()
        if (
            abs(abs(totals["left"]) - abs(totals["right"]))
            > profile["max_side_divergence_degrees"]
        ):
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_encoder_divergence",
                "Physical scan drive sides diverged beyond calibration",
            )
        mean_abs_encoder_degrees = (
            abs(totals["left"]) + abs(totals["right"])
        ) / 2.0
        actual_delta_mdeg = direction * int(
            round(
                mean_abs_encoder_degrees
                * SCAN_TURN_REFERENCE_BODY_MDEG
                / SCAN_TURN_REFERENCE_ENCODER_DEGREES
            )
        )
        if (
            abs(actual_delta_mdeg - relative_delta_mdeg)
            > calibration.alignment_tolerance_mdeg
        ):
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_encoder_pose_mismatch",
                "Encoder-derived scan heading missed its target",
            )
        completed_at_ms = self._now_ms()
        if completed_at_ms > deadline_ms:
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_deadline_exceeded",
                "Physical scan turn completed after its deadline",
            )
        self._heading_mdeg = normalize_heading_mdeg(
            self._heading_mdeg + actual_delta_mdeg
        )
        self._last_state_version = state_version
        return {
            "requested_delta_mdeg": relative_delta_mdeg,
            "actual_delta_mdeg": actual_delta_mdeg,
            "completed_at_ms": completed_at_ms,
            "stop_confirmed": True,
        }

    def read_snapshot(self) -> Mapping[str, object]:
        if self._deadline_ms is None:
            raise ActiveIrScanContractError(
                "scan_rig_not_armed",
                "Physical scan rig is not armed",
            )
        self._require_active(self._deadline_ms)
        response = self._request(
            SCAN_SAMPLE_OPERATION,
            {},
            self._deadline_ms,
            cancellation_enabled=True,
        )
        result = response.get("result", {})
        observation = validate_observation(result.get("observation"))
        state_version = response.get("state_version")
        observed_at_ms = self._now_ms()
        if (
            observation["state_version"] != state_version
            or state_version <= self._last_state_version
            or observed_at_ms <= self._last_observed_at_ms
            or observed_at_ms > self._deadline_ms
        ):
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_snapshot_state_mismatch",
                "Physical scan snapshot is stale or uncorrelated",
            )
        self._last_state_version = state_version
        self._last_observed_at_ms = observed_at_ms
        infrared = observation["infrared"]
        return {
            "state_version": state_version,
            "observed_at_ms": observed_at_ms,
            "pose_heading_mdeg": self._heading_mdeg,
            "touch_pressed": observation["touch"]["pressed"],
            "motion_fault_latched": observation["budgets"][
                "motion_fault_latched"
            ],
            "infrared": {
                "raw": infrared["raw"],
                "filtered": infrared["filtered"],
                "blocked": infrared["blocked"],
            },
        }

    def stop(self) -> Mapping[str, object]:
        if self._deadline_ms is None:
            return {"stop_confirmed": True}
        response = self._request(
            "stop",
            {},
            self._deadline_ms,
            cancellation_enabled=False,
        )
        result = response.get("result")
        if (
            not isinstance(result, dict)
            or set(result) != {"outcome", "observation", "stop"}
            or result["stop"].get("stop_confirmed") is not True
            or result["outcome"].get("stop_confirmed") is not True
        ):
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_stop_unverified",
                "Physical scan stop could not be verified",
            )
        observation = validate_observation(result["observation"])
        if (
            observation["state_version"] != response["state_version"]
            or response["state_version"] <= self._last_state_version
        ):
            self.transport.abort()
            raise ActiveIrScanContractError(
                "scan_stop_state_mismatch",
                "Physical scan stop receipt is stale",
            )
        self._last_state_version = response["state_version"]
        return {"stop_confirmed": True}


def build_ev3_active_ir_scan_executor(
    transport,
    *,
    clock_ms: Callable[[], int] = lambda: int(time.time() * 1000),
    cancel_requested: Callable[[], bool] = lambda: False,
    request_timeout_seconds: float = 8.0,
) -> ActiveIrScanExecutor:
    """Factory seam for ``PhysicalNavigationRuntimeAdapter``."""
    rig = EV3ActiveIrScanRig(
        transport=transport,
        clock_ms=clock_ms,
        cancel_requested=cancel_requested,
        request_timeout_seconds=request_timeout_seconds,
    )
    return ActiveIrScanExecutor(rig=rig, clock_ms=clock_ms)
