from typing import Callable, Dict, Set

from .contract import CommandResult, MotionCommand, MotorState, RobotState
from .safety import SafetyPolicy, SafetyViolation


class SimulatedRobot:
    """Deterministic backend used before physical agent experiments."""

    def __init__(self, safety: SafetyPolicy, clock_ms: Callable[[], int]):
        self._safety = safety
        self._clock_ms = clock_ms
        self._positions: Dict[str, int] = {
            role: 0 for role in safety.limits.motor_roles
        }
        self._seen_command_ids: Set[str] = set()
        self._stopped = True
        self._sensors: Dict[str, object] = {
            "touch": False,
            "color_reflected_percent": 0,
            "infrared_proximity_percent": 100,
        }

    def execute_motion(self, command: MotionCommand) -> CommandResult:
        now_ms = self._clock_ms()
        self._safety.validate_motion(command, now_ms)
        if command.command_id in self._seen_command_ids:
            raise SafetyViolation(
                "duplicate_command",
                "Command {!r} has already been processed".format(command.command_id),
            )

        self._seen_command_ids.add(command.command_id)
        before = self._positions[command.motor_role]
        delta = round(command.speed_dps * command.duration_ms / 1000)
        self._stopped = False
        self._positions[command.motor_role] = before + delta
        self._stopped = True

        return CommandResult(
            command_id=command.command_id,
            status="completed",
            started_at_ms=now_ms,
            completed_at_ms=now_ms + command.duration_ms,
            position_before=before,
            position_after=self._positions[command.motor_role],
            detail="simulated run-timed motion",
        )

    def stop_all(self) -> None:
        self._stopped = True

    def set_sensor(self, name: str, value: object) -> None:
        if name not in self._sensors:
            raise KeyError(name)
        self._sensors[name] = value

    def read_state(self) -> RobotState:
        return RobotState(
            observed_at_ms=self._clock_ms(),
            motors={
                role: MotorState(
                    role=role,
                    port=self._safety.limits.motor_roles[role],
                    position_degrees=position,
                    running=not self._stopped,
                )
                for role, position in self._positions.items()
            },
            sensors=dict(self._sensors),
        )
