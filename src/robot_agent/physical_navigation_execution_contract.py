"""Robot-specific execution boundary for shared physical navigation."""

from copy import deepcopy
from typing import Mapping, Optional, Protocol, Tuple

from .physical_navigation_contract import (
    EXPECTED_ACTION_SPECS,
    EXPECTED_WORKER_OPERATIONS,
    EXPECTED_WORKER_SAFETY,
    SCAN_SAMPLE_OPERATION,
    SCAN_TURN_OPERATION,
    expected_scan_sample_profile,
    expected_scan_turn_profile,
    validate_observation,
)
from .physical_navigation_runtime_errors import (
    PhysicalNavigationRuntimeError,
)
from .physical_odometry import DriveMotorRoles


class PhysicalNavigationExecutionContract(Protocol):
    """Validates one controller's protocol at the shared runtime boundary."""

    def parse_description(
        self,
        response: Mapping[str, object],
    ) -> Tuple[
        Mapping[str, object],
        Mapping[str, Mapping[str, object]],
        DriveMotorRoles,
        int,
    ]: ...

    def parse_observation(
        self,
        operation: str,
        response: Mapping[str, object],
        expected_action: Optional[str] = None,
    ) -> Mapping[str, object]: ...

    def shutdown_verified(self, response: Mapping[str, object]) -> bool: ...


class EV3NavigationExecutionContract:
    """Preserves the existing EV3 worker contract without runtime policy."""

    @staticmethod
    def parse_description(
        response: Mapping[str, object],
    ) -> Tuple[
        Mapping[str, object],
        Mapping[str, Mapping[str, object]],
        DriveMotorRoles,
        int,
    ]:
        result = response.get("result")
        if not isinstance(result, dict):
            raise PhysicalNavigationRuntimeError(
                "invalid_worker_description",
                "Worker description is missing",
            )
        required = {
            "worker_id",
            "demo_only",
            "policy_owner",
            "controller_id",
            "request_schema",
            "response_schema",
            "operations",
            "pulse",
            "scan_turn",
            "scan_sample",
            "safety",
            "process",
            "observation",
            "drive_geometry",
        }
        if set(result) != required or result["policy_owner"] != "host":
            raise PhysicalNavigationRuntimeError(
                "invalid_worker_description",
                "Worker identity/policy boundary is invalid",
            )
        pulse = result["pulse"]
        safety = result["safety"]
        process = result["process"]
        if (
            not isinstance(process, dict)
            or set(process) != {"absolute_max_ms", "max_requests"}
            or isinstance(process["absolute_max_ms"], bool)
            or not isinstance(process["absolute_max_ms"], int)
            or not 5_000 <= process["absolute_max_ms"] <= 180_000
            or isinstance(process["max_requests"], bool)
            or not isinstance(process["max_requests"], int)
            or process["max_requests"] <= 0
        ):
            raise PhysicalNavigationRuntimeError(
                "invalid_worker_process_contract",
                "Worker process lifetime contract is invalid",
            )
        geometry = result["drive_geometry"]
        if (
            not isinstance(geometry, dict)
            or set(geometry)
            != {
                "left_motor_role",
                "right_motor_role",
                "forward_speed_sign",
            }
            or not isinstance(geometry["forward_speed_sign"], dict)
        ):
            raise PhysicalNavigationRuntimeError(
                "invalid_worker_drive_geometry",
                "Worker drive geometry is invalid",
            )
        drive_roles = DriveMotorRoles(
            left=geometry["left_motor_role"],
            right=geometry["right_motor_role"],
        )
        if (
            set(geometry["forward_speed_sign"])
            != {drive_roles.left, drive_roles.right}
            or geometry["forward_speed_sign"][drive_roles.left] != 1
            or geometry["forward_speed_sign"][drive_roles.right] != 1
        ):
            raise PhysicalNavigationRuntimeError(
                "unsupported_worker_drive_sign",
                "Semantic action profile requires positive-forward drive roles",
            )
        if (
            not isinstance(pulse, dict)
            or pulse.get("actions") != EXPECTED_ACTION_SPECS
            or not isinstance(result["operations"], list)
            or set(result["operations"]) != EXPECTED_WORKER_OPERATIONS
            or result["scan_turn"] != expected_scan_turn_profile()
            or result["scan_sample"] != expected_scan_sample_profile()
            or safety != EXPECTED_WORKER_SAFETY
        ):
            raise PhysicalNavigationRuntimeError(
                "unsafe_worker_contract",
                "Worker safety or semantic action contract is invalid",
            )
        observation = validate_observation(result["observation"])
        return (
            observation,
            deepcopy(pulse["actions"]),
            drive_roles,
            process["absolute_max_ms"],
        )

    @staticmethod
    def parse_observation(
        operation: str,
        response: Mapping[str, object],
        expected_action: Optional[str] = None,
    ) -> Mapping[str, object]:
        result = response.get("result")
        if not isinstance(result, dict):
            raise PhysicalNavigationRuntimeError(
                "invalid_worker_result",
                "Worker result is missing",
            )
        if operation == "observe":
            if set(result) != {"observation"}:
                raise PhysicalNavigationRuntimeError(
                    "invalid_observe_result",
                    "Observe result fields are invalid",
                )
            observation = validate_observation(result["observation"])
        elif operation == "pulse":
            if (
                set(result) != {"action", "outcome", "observation", "stop"}
                or result["action"] != expected_action
                or not isinstance(result["outcome"], dict)
                or result["outcome"].get("action") != expected_action
                or result["outcome"].get("stop_confirmed") is not True
            ):
                raise PhysicalNavigationRuntimeError(
                    "invalid_pulse_result",
                    "Pulse result is not correlated and stopped",
                )
            observation = validate_observation(result["observation"])
            if observation["last_outcome"] != result["outcome"]:
                raise PhysicalNavigationRuntimeError(
                    "pulse_outcome_mismatch",
                    "Pulse observation lacks its correlated outcome",
                )
        elif operation in (SCAN_TURN_OPERATION, SCAN_SAMPLE_OPERATION):
            # EV3NavigationSSHTransport has already validated the complete
            # operation-specific receipt. Accept only its correlated snapshot.
            observation = validate_observation(result.get("observation"))
        else:
            raise PhysicalNavigationRuntimeError(
                "invalid_observation_operation",
                "Operation has no observation contract",
            )
        if response.get("state_version") != observation["state_version"]:
            raise PhysicalNavigationRuntimeError(
                "worker_state_version_mismatch",
                "Response and observation state versions differ",
            )
        return observation

    @staticmethod
    def shutdown_verified(response: Mapping[str, object]) -> bool:
        result = response.get("result", {})
        outcome = result.get("outcome", {})
        return (
            outcome.get("status") == "completed"
            and outcome.get("stop_confirmed") is True
            and outcome.get("motor_owner_closed") is True
        )
