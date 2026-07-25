import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

from .contract import MotionCommand


class SafetyViolation(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class MotionLimit:
    max_abs_speed_dps: int
    max_duration_ms: int


@dataclass(frozen=True)
class SafetyLimits:
    motor_roles: Dict[str, str]
    drive: MotionLimit
    arm: MotionLimit
    heartbeat_timeout_ms: int
    max_command_age_ms: int = 1_000

    @classmethod
    def from_file(cls, path: Path) -> "SafetyLimits":
        with path.open("r", encoding="utf-8") as handle:
            config = json.load(handle)

        roles = {
            role: details["port"] for role, details in config["motors"].items()
        }
        limit_config = config["limits"]
        return cls(
            motor_roles=roles,
            drive=MotionLimit(**limit_config["drive"]),
            arm=MotionLimit(**limit_config["arm"]),
            heartbeat_timeout_ms=limit_config["heartbeat"]["timeout_ms"],
        )

    def limit_for_role(self, role: str) -> MotionLimit:
        if role not in self.motor_roles:
            raise SafetyViolation(
                "unknown_motor", "Motor role {!r} is not configured".format(role)
            )
        return self.drive if role.startswith("drive_") else self.arm


class SafetyPolicy:
    def __init__(self, limits: SafetyLimits):
        self.limits = limits

    def validate_motion(self, command: MotionCommand, now_ms: int) -> None:
        if not command.command_id or len(command.command_id) > 128:
            raise SafetyViolation(
                "invalid_command_id", "command_id must contain 1-128 characters"
            )

        age_ms = now_ms - command.issued_at_ms
        if age_ms < 0:
            raise SafetyViolation(
                "future_command", "Command timestamp is in the future"
            )
        if age_ms > self.limits.max_command_age_ms:
            raise SafetyViolation(
                "stale_command",
                "Command is {} ms old; maximum is {} ms".format(
                    age_ms, self.limits.max_command_age_ms
                ),
            )

        limit = self.limits.limit_for_role(command.motor_role)
        if isinstance(command.speed_dps, bool) or not isinstance(
            command.speed_dps, int
        ):
            raise SafetyViolation("invalid_speed", "speed_dps must be an integer")
        if command.speed_dps == 0:
            raise SafetyViolation("zero_speed", "Use stop instead of zero speed")
        if abs(command.speed_dps) > limit.max_abs_speed_dps:
            raise SafetyViolation(
                "speed_limit",
                "Requested speed {} exceeds limit {}".format(
                    command.speed_dps, limit.max_abs_speed_dps
                ),
            )

        if isinstance(command.duration_ms, bool) or not isinstance(
            command.duration_ms, int
        ):
            raise SafetyViolation(
                "invalid_duration", "duration_ms must be an integer"
            )
        if command.duration_ms <= 0:
            raise SafetyViolation(
                "invalid_duration", "duration_ms must be greater than zero"
            )
        if command.duration_ms > limit.max_duration_ms:
            raise SafetyViolation(
                "duration_limit",
                "Requested duration {} exceeds limit {}".format(
                    command.duration_ms, limit.max_duration_ms
                ),
            )
