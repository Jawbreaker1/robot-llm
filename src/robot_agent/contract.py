from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional


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

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class RobotState:
    observed_at_ms: int
    motors: Dict[str, MotorState]
    sensors: Dict[str, object] = field(default_factory=dict)
    active_faults: List[str] = field(default_factory=list)

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
