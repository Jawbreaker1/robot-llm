"""Small typed RobotAPI for host-side experiments.

The API accepts only typed, snapshot-bound requests.  Natural-language
classification belongs to a planner adapter, never to this module.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
import json
import math
from pathlib import Path
import secrets
import threading
from typing import Callable, Mapping, Optional, Tuple

from .contract import MotionCommand, RobotState
from .safety import SafetyLimits, SafetyPolicy, SafetyViolation
from .simulated_robot import SimulatedRobot


MAX_ACTION_TTL_MS = 1_000
MAX_OBSERVATION_AGE_MS = 500


class RobotAPIError(RuntimeError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


class RobotAPIContractError(RobotAPIError):
    pass


class RobotActionRejected(RobotAPIError):
    pass


def _identifier(name: str, value: str, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise RobotAPIContractError(
            "invalid_identifier",
            "{} is invalid".format(name),
        )
    return value


def _integer(name: str, value: int, minimum: int, maximum: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise RobotAPIContractError(
            "invalid_integer",
            "{} is invalid".format(name),
        )
    return value


@dataclass(frozen=True)
class CapabilityGate:
    supported: bool
    enabled: bool
    available: bool
    reason_code: Optional[str] = None

    def __post_init__(self) -> None:
        if not all(
            isinstance(value, bool)
            for value in (self.supported, self.enabled, self.available)
        ):
            raise RobotAPIContractError(
                "invalid_capability",
                "Capability flags must be boolean",
            )
        if (self.enabled or self.available) and not self.supported:
            raise RobotAPIContractError(
                "invalid_capability",
                "Unsupported capability cannot be enabled or available",
            )
        if self.reason_code is not None:
            _identifier("reason_code", self.reason_code, 64)

    @property
    def executable(self) -> bool:
        return self.supported and self.enabled and self.available

    def to_dict(self) -> Mapping[str, object]:
        return {
            "supported": self.supported,
            "enabled": self.enabled,
            "available": self.available,
            "reason_code": self.reason_code,
        }


@dataclass(frozen=True)
class MotorCapability:
    motor_role: str
    gate: CapabilityGate
    max_abs_speed_dps: int
    max_duration_ms: int

    def __post_init__(self) -> None:
        _identifier("motor_role", self.motor_role)
        if not isinstance(self.gate, CapabilityGate):
            raise RobotAPIContractError(
                "invalid_capability",
                "Motor capability gate is invalid",
            )
        _integer("max_abs_speed_dps", self.max_abs_speed_dps, 1, 100_000)
        _integer("max_duration_ms", self.max_duration_ms, 1, 60_000)

    def to_dict(self) -> Mapping[str, object]:
        return {
            "tool_id": "move_motor",
            "motor_role": self.motor_role,
            "gate": self.gate.to_dict(),
            "max_abs_speed_dps": self.max_abs_speed_dps,
            "max_duration_ms": self.max_duration_ms,
        }


@dataclass(frozen=True)
class ControllerCapabilities:
    robot_id: str
    controller_id: str
    controller_instance_id: str
    host_clock_id: str
    observe: CapabilityGate
    emergency_stop: CapabilityGate
    motors: Tuple[MotorCapability, ...]
    motion_retry_semantics: str
    motion_execution_model: str

    def __post_init__(self) -> None:
        _identifier("robot_id", self.robot_id)
        _identifier("controller_id", self.controller_id)
        _identifier("controller_instance_id", self.controller_instance_id)
        _identifier("host_clock_id", self.host_clock_id)
        if not isinstance(self.observe, CapabilityGate) or not isinstance(
            self.emergency_stop,
            CapabilityGate,
        ):
            raise RobotAPIContractError(
                "invalid_capability",
                "Controller capability gates are invalid",
            )
        if not isinstance(self.motors, tuple):
            raise RobotAPIContractError(
                "invalid_capability",
                "Motor capabilities must be a tuple",
            )
        if any(
            not isinstance(motor, MotorCapability)
            for motor in self.motors
        ):
            raise RobotAPIContractError(
                "invalid_capability",
                "Motor capability entry is invalid",
            )
        roles = tuple(motor.motor_role for motor in self.motors)
        if len(set(roles)) != len(roles):
            raise RobotAPIContractError(
                "duplicate_motor_role",
                "Motor capabilities contain duplicate roles",
            )
        if self.motion_retry_semantics != "at_most_once":
            raise RobotAPIContractError(
                "invalid_capability",
                "Motion retry semantics are unsupported",
            )
        if self.motion_execution_model != "accelerated_synchronous":
            raise RobotAPIContractError(
                "invalid_capability",
                "Motion execution model is unsupported",
            )

    def motor(self, role: str) -> MotorCapability:
        for capability in self.motors:
            if capability.motor_role == role:
                return capability
        raise RobotActionRejected(
            "motor_not_exposed",
            "Motor role is not exposed by this RobotAPI",
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "robot_id": self.robot_id,
            "controller_id": self.controller_id,
            "controller_instance_id": self.controller_instance_id,
            "host_clock_id": self.host_clock_id,
            "observe": self.observe.to_dict(),
            "emergency_stop": self.emergency_stop.to_dict(),
            "motors": [motor.to_dict() for motor in self.motors],
            "motion_retry_semantics": self.motion_retry_semantics,
            "motion_execution_model": self.motion_execution_model,
        }


@dataclass(frozen=True)
class ObservationEnvelope:
    robot_id: str
    controller_id: str
    controller_instance_id: str
    host_clock_id: str
    state_version: int
    received_at_host_ms: int
    state: RobotState

    def __post_init__(self) -> None:
        _identifier("robot_id", self.robot_id)
        _identifier("controller_id", self.controller_id)
        _identifier("controller_instance_id", self.controller_instance_id)
        _identifier("host_clock_id", self.host_clock_id)
        _integer("state_version", self.state_version, 1, 2**63 - 1)
        _integer(
            "received_at_host_ms",
            self.received_at_host_ms,
            0,
            2**63 - 1,
        )
        if not isinstance(self.state, RobotState):
            raise RobotAPIContractError(
                "invalid_observation",
                "Observation state must be RobotState",
            )
        _integer(
            "state.observed_at_ms",
            self.state.observed_at_ms,
            0,
            2**63 - 1,
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "robot_id": self.robot_id,
            "controller_id": self.controller_id,
            "controller_instance_id": self.controller_instance_id,
            "host_clock_id": self.host_clock_id,
            "state_version": self.state_version,
            "received_at_host_ms": self.received_at_host_ms,
            "state": self.state.to_dict(),
        }


@dataclass(frozen=True)
class ActionContext:
    robot_id: str
    controller_id: str
    controller_instance_id: str
    action_id: str
    segment_id: str
    source_id: str
    host_clock_id: str
    based_on_state_version: int
    based_on_received_at_host_ms: int
    issued_at_host_ms: int
    valid_until_host_ms: int

    def __post_init__(self) -> None:
        for name in (
            "robot_id",
            "controller_id",
            "controller_instance_id",
            "action_id",
            "source_id",
            "host_clock_id",
        ):
            _identifier(name, getattr(self, name))
        _identifier("segment_id", self.segment_id, 96)
        _integer(
            "based_on_state_version",
            self.based_on_state_version,
            1,
            2**63 - 1,
        )
        _integer(
            "based_on_received_at_host_ms",
            self.based_on_received_at_host_ms,
            0,
            2**63 - 1,
        )
        _integer(
            "issued_at_host_ms",
            self.issued_at_host_ms,
            0,
            2**63 - 1,
        )
        _integer(
            "valid_until_host_ms",
            self.valid_until_host_ms,
            1,
            2**63 - 1,
        )
        if self.valid_until_host_ms <= self.issued_at_host_ms:
            raise RobotAPIContractError(
                "invalid_deadline",
                "Action deadline must follow issue time",
            )
        if self.issued_at_host_ms < self.based_on_received_at_host_ms:
            raise RobotAPIContractError(
                "invalid_causality",
                "Action cannot predate its observation",
            )


@dataclass(frozen=True)
class MotionRequest:
    context: ActionContext
    command: MotionCommand

    def __post_init__(self) -> None:
        if not isinstance(self.context, ActionContext):
            raise RobotAPIContractError(
                "invalid_context",
                "Motion request requires ActionContext",
            )
        if not isinstance(self.command, MotionCommand):
            raise RobotAPIContractError(
                "invalid_command",
                "Motion request requires MotionCommand",
            )


@dataclass(frozen=True)
class StopRequest:
    robot_id: str
    controller_id: str
    controller_instance_id: str
    action_id: str
    segment_id: str
    source_id: str

    def __post_init__(self) -> None:
        for name in (
            "robot_id",
            "controller_id",
            "controller_instance_id",
            "action_id",
            "source_id",
        ):
            _identifier(name, getattr(self, name))
        _identifier("segment_id", self.segment_id, 96)


@dataclass(frozen=True)
class ActionReceipt:
    action_id: str
    segment_id: str
    controller_id: str
    controller_instance_id: str
    status: str
    code: str
    based_on_state_version: int
    resulting_state_version: int
    position_before: Optional[int] = None
    position_after: Optional[int] = None

    def __post_init__(self) -> None:
        for name in (
            "action_id",
            "controller_id",
            "controller_instance_id",
            "code",
        ):
            _identifier(name, getattr(self, name))
        _identifier("segment_id", self.segment_id, 96)
        if self.status not in ("completed", "stopped"):
            raise RobotAPIContractError(
                "invalid_status",
                "Action receipt status is invalid",
            )
        _integer(
            "based_on_state_version",
            self.based_on_state_version,
            1,
            2**63 - 1,
        )
        _integer(
            "resulting_state_version",
            self.resulting_state_version,
            1,
            2**63 - 1,
        )
        for name in ("position_before", "position_after"):
            value = getattr(self, name)
            if value is not None:
                _integer(name, value, -(2**63), 2**63 - 1)


class RobotAPI(ABC):
    @abstractmethod
    def capabilities(self) -> ControllerCapabilities:
        raise NotImplementedError

    @abstractmethod
    def observe(self) -> ObservationEnvelope:
        raise NotImplementedError

    @abstractmethod
    def execute_motion(self, request: MotionRequest) -> ActionReceipt:
        raise NotImplementedError

    @abstractmethod
    def stop_all(self, request: StopRequest) -> ActionReceipt:
        raise NotImplementedError


class SimulatedRobotAPI(RobotAPI):
    """Accelerated synchronous, at-most-once simulator adapter.

    The adapter updates state and returns only after its simulated action.
    It does not model wall-clock motion, RUNNING state, heartbeat, urgent
    interruption, or recovery of a receipt after a lost response.
    """

    def __init__(
        self,
        robot: SimulatedRobot,
        capabilities: ControllerCapabilities,
        clock_ms: Callable[[], int],
        maximum_action_ttl_ms: int = MAX_ACTION_TTL_MS,
        maximum_observation_age_ms: int = MAX_OBSERVATION_AGE_MS,
    ):
        if not isinstance(robot, SimulatedRobot) or not callable(clock_ms):
            raise RobotAPIContractError(
                "invalid_dependency",
                "Simulator dependencies are invalid",
            )
        if not isinstance(capabilities, ControllerCapabilities):
            raise RobotAPIContractError(
                "invalid_capability",
                "Controller capabilities are invalid",
            )
        _integer(
            "maximum_action_ttl_ms",
            maximum_action_ttl_ms,
            1,
            60_000,
        )
        _integer(
            "maximum_observation_age_ms",
            maximum_observation_age_ms,
            1,
            60_000,
        )
        self._robot = robot
        self._capabilities = capabilities
        self._clock_ms = clock_ms
        self._maximum_action_ttl_ms = maximum_action_ttl_ms
        self._maximum_observation_age_ms = maximum_observation_age_ms
        self._state_version = 1
        self._last_observation_at_ms = None
        self._seen_segments = set()
        self._state_lock = threading.Lock()

    @classmethod
    def from_config(
        cls,
        path: Path,
        clock_ms: Callable[[], int],
        host_clock_id: str = "host-monotonic-ms",
        controller_instance_id: Optional[str] = None,
    ) -> "SimulatedRobotAPI":
        path = Path(path)
        with path.open("r", encoding="utf-8") as handle:
            config = json.load(handle)
        limits = SafetyLimits.from_file(path)
        robot = SimulatedRobot(SafetyPolicy(limits), clock_ms)
        if controller_instance_id is None:
            controller_instance_id = secrets.token_hex(16)
        geometry = config["drive_geometry"]
        drive_roles = {
            geometry["left_motor_role"],
            geometry["right_motor_role"],
        }
        enabled = CapabilityGate(True, True, True)
        try:
            exposed_roles = config["agent_api"]["move_motor_roles"]
        except (KeyError, TypeError):
            raise RobotAPIContractError(
                "invalid_capability_config",
                "agent_api.move_motor_roles is required",
            ) from None
        if (
            not isinstance(exposed_roles, list)
            or not exposed_roles
            or any(
                not isinstance(role, str)
                or role not in limits.motor_roles
                or role in drive_roles
                for role in exposed_roles
            )
            or len(set(exposed_roles)) != len(exposed_roles)
        ):
            raise RobotAPIContractError(
                "invalid_capability_config",
                "Explicit auxiliary motor allowlist is invalid",
            )
        motors = tuple(
            MotorCapability(
                motor_role=role,
                gate=enabled,
                max_abs_speed_dps=limits.limit_for_role(
                    role
                ).max_abs_speed_dps,
                max_duration_ms=limits.limit_for_role(role).max_duration_ms,
            )
            for role in sorted(exposed_roles)
        )
        capabilities = ControllerCapabilities(
            robot_id=config["robot_id"],
            controller_id=config["controller_id"],
            controller_instance_id=controller_instance_id,
            host_clock_id=host_clock_id,
            observe=enabled,
            emergency_stop=enabled,
            motors=motors,
            motion_retry_semantics="at_most_once",
            motion_execution_model="accelerated_synchronous",
        )
        return cls(robot, capabilities, clock_ms)

    def _now_ms(self) -> int:
        return _integer("clock_ms", self._clock_ms(), 0, 2**63 - 1)

    def capabilities(self) -> ControllerCapabilities:
        return self._capabilities

    def observe(self) -> ObservationEnvelope:
        if not self._capabilities.observe.executable:
            raise RobotActionRejected(
                "capability_unavailable",
                "Observation capability is unavailable",
            )
        with self._state_lock:
            observed_at_ms = self._now_ms()
            self._last_observation_at_ms = observed_at_ms
            return ObservationEnvelope(
                robot_id=self._capabilities.robot_id,
                controller_id=self._capabilities.controller_id,
                controller_instance_id=(
                    self._capabilities.controller_instance_id
                ),
                host_clock_id=self._capabilities.host_clock_id,
                state_version=self._state_version,
                received_at_host_ms=observed_at_ms,
                state=self._robot.read_state(),
            )

    def _validate_context(self, context: ActionContext) -> int:
        if not isinstance(context, ActionContext):
            raise RobotAPIContractError(
                "invalid_context",
                "Motion request has invalid context",
            )
        expected = self._capabilities
        if (
            context.robot_id != expected.robot_id
            or context.controller_id != expected.controller_id
            or context.controller_instance_id
            != expected.controller_instance_id
        ):
            raise RobotActionRejected(
                "identity_mismatch",
                "Action targeted another controller instance",
            )
        if context.host_clock_id != expected.host_clock_id:
            raise RobotActionRejected(
                "clock_mismatch",
                "Action used another clock domain",
            )
        if context.based_on_state_version != self._state_version:
            raise RobotActionRejected(
                "stale_state",
                "Action referenced a stale state version",
            )
        now_ms = self._now_ms()
        if (
            self._last_observation_at_ms is None
            or context.based_on_received_at_host_ms
            != self._last_observation_at_ms
            or now_ms < context.based_on_received_at_host_ms
            or now_ms - context.based_on_received_at_host_ms
            > self._maximum_observation_age_ms
        ):
            raise RobotActionRejected(
                "stale_observation",
                "Action referenced a stale observation",
            )
        if context.issued_at_host_ms > now_ms:
            raise RobotActionRejected(
                "future_action",
                "Action issue time is in the future",
            )
        if now_ms >= context.valid_until_host_ms:
            raise RobotActionRejected(
                "stale_action",
                "Action deadline expired",
            )
        if (
            context.valid_until_host_ms - context.issued_at_host_ms
            > self._maximum_action_ttl_ms
        ):
            raise RobotActionRejected(
                "ttl_limit",
                "Action TTL exceeds host policy",
            )
        if context.segment_id in self._seen_segments:
            raise RobotActionRejected(
                "duplicate_segment",
                "Action segment has already been consumed",
            )
        return now_ms

    def execute_motion(self, request: MotionRequest) -> ActionReceipt:
        if not isinstance(request, MotionRequest):
            raise RobotAPIContractError(
                "invalid_request",
                "RobotAPI accepts only MotionRequest",
            )
        with self._state_lock:
            self._validate_context(request.context)
            command = request.command
            if (
                command.command_id != request.context.segment_id
                or command.issued_at_ms
                != request.context.issued_at_host_ms
            ):
                raise RobotActionRejected(
                    "command_context_mismatch",
                    "MotionCommand did not match its ActionContext",
                )
            capability = self._capabilities.motor(command.motor_role)
            if not capability.gate.executable:
                raise RobotActionRejected(
                    "capability_unavailable",
                    "Motor capability is not executable",
                )
            if (
                isinstance(command.speed_dps, bool)
                or not isinstance(command.speed_dps, int)
                or command.speed_dps == 0
                or abs(command.speed_dps)
                > capability.max_abs_speed_dps
            ):
                raise RobotActionRejected(
                    "speed_limit",
                    "Motor speed is invalid",
                )
            if (
                isinstance(command.duration_ms, bool)
                or not isinstance(command.duration_ms, int)
                or not 1
                <= command.duration_ms
                <= capability.max_duration_ms
            ):
                raise RobotActionRejected(
                    "duration_limit",
                    "Motor duration is invalid",
                )
            dispatch_now_ms = self._validate_context(request.context)
            try:
                self._robot.validate_motion(command, dispatch_now_ms)
            except SafetyViolation as error:
                raise RobotActionRejected(
                    error.code,
                    str(error),
                ) from None
            self._seen_segments.add(request.context.segment_id)
            try:
                result = self._robot.execute_motion(
                    command,
                    valid_until_ms=request.context.valid_until_host_ms,
                )
            except SafetyViolation as error:
                raise RobotActionRejected(
                    error.code,
                    str(error),
                ) from None
            self._state_version += 1
            return ActionReceipt(
                action_id=request.context.action_id,
                segment_id=request.context.segment_id,
                controller_id=request.context.controller_id,
                controller_instance_id=(
                    request.context.controller_instance_id
                ),
                status="completed",
                code="simulated_motion_verified",
                based_on_state_version=(
                    request.context.based_on_state_version
                ),
                resulting_state_version=self._state_version,
                position_before=result.position_before,
                position_after=result.position_after,
            )

    def stop_all(self, request: StopRequest) -> ActionReceipt:
        if not isinstance(request, StopRequest):
            raise RobotAPIContractError(
                "invalid_request",
                "stop_all requires StopRequest",
            )
        with self._state_lock:
            if (
                request.robot_id != self._capabilities.robot_id
                or request.controller_id
                != self._capabilities.controller_id
                or request.controller_instance_id
                != self._capabilities.controller_instance_id
            ):
                raise RobotActionRejected(
                    "identity_mismatch",
                    "Stop targeted another controller",
                )
            before_version = self._state_version
            self._robot.stop_all()
            if any(
                motor.running
                for motor in self._robot.read_state().motors.values()
            ):
                raise RobotAPIError(
                    "stop_not_verified",
                    "Simulator did not verify all motors stopped",
                )
            self._state_version += 1
            return ActionReceipt(
                action_id=request.action_id,
                segment_id=request.segment_id,
                controller_id=request.controller_id,
                controller_instance_id=(
                    self._capabilities.controller_instance_id
                ),
                status="stopped",
                code="simulated_stop_verified",
                based_on_state_version=before_version,
                resulting_state_version=self._state_version,
            )

    def set_sensor(self, name: str, value: object) -> None:
        _identifier("sensor_name", name)
        if (
            not isinstance(value, (bool, int, float, str, type(None)))
            or isinstance(value, float)
            and not math.isfinite(value)
        ):
            raise RobotAPIContractError(
                "invalid_sensor_value",
                "Sensor value must be a finite JSON scalar",
            )
        with self._state_lock:
            self._robot.set_sensor(name, value)
            self._state_version += 1
