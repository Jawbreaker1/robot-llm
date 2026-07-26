from dataclasses import asdict, dataclass, field
import math
from types import MappingProxyType
from typing import Dict, Mapping, Optional, Tuple


@dataclass(frozen=True)
class MotionCommand:
    command_id: str
    motor_role: str
    speed_dps: int
    duration_ms: int
    issued_at_ms: int

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class MotorState:
    role: str
    port: str
    position_degrees: int
    running: bool = False

    def __post_init__(self) -> None:
        if (
            not isinstance(self.role, str)
            or not self.role
            or not isinstance(self.port, str)
            or not self.port
            or isinstance(self.position_degrees, bool)
            or not isinstance(self.position_degrees, int)
            or type(self.running) is not bool
        ):
            raise ValueError("MotorState fields are invalid")

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class RobotState:
    observed_at_ms: int
    motors: Mapping[str, MotorState]
    sensors: Mapping[str, object] = field(default_factory=dict)
    active_faults: Tuple[str, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if (
            isinstance(self.observed_at_ms, bool)
            or not isinstance(self.observed_at_ms, int)
            or self.observed_at_ms < 0
        ):
            raise ValueError("RobotState observed_at_ms is invalid")
        motors = dict(self.motors)
        sensors = dict(self.sensors)
        faults = tuple(self.active_faults)
        if (
            any(
                not isinstance(role, str)
                or not role
                or not isinstance(motor, MotorState)
                or motor.role != role
                for role, motor in motors.items()
            )
            or any(
                not isinstance(name, str)
                or not name
                or not isinstance(
                    value,
                    (bool, int, float, str, type(None)),
                )
                or isinstance(value, float)
                and not math.isfinite(value)
                for name, value in sensors.items()
            )
            or any(
                not isinstance(fault, str) or not fault
                for fault in faults
            )
        ):
            raise ValueError("RobotState contents are invalid")
        object.__setattr__(
            self,
            "motors",
            MappingProxyType(motors),
        )
        object.__setattr__(
            self,
            "sensors",
            MappingProxyType(sensors),
        )
        object.__setattr__(
            self,
            "active_faults",
            faults,
        )

    def to_dict(self) -> Dict[str, object]:
        return {
            "observed_at_ms": self.observed_at_ms,
            "motors": {
                role: state.to_dict() for role, state in self.motors.items()
            },
            "sensors": dict(self.sensors),
            "active_faults": list(self.active_faults),
        }


@dataclass(frozen=True)
class CommandResult:
    command_id: str
    status: str
    started_at_ms: int
    completed_at_ms: int
    position_before: Optional[int] = None
    position_after: Optional[int] = None
    detail: str = ""

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)
