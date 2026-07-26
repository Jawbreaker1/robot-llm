import json
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

from .contract import MotionCommand


class SafetyViolation(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class MotionLimit:
    max_abs_speed_dps: int
    max_duration_ms: int

    def __post_init__(self) -> None:
        for name, value in (
            ("max_abs_speed_dps", self.max_abs_speed_dps),
            ("max_duration_ms", self.max_duration_ms),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError("{} must be a positive integer".format(name))


@dataclass(frozen=True)
class SafetyLimits:
    motor_roles: Mapping[str, str]
    role_limits: Mapping[str, MotionLimit]
    drive: MotionLimit
    arm: MotionLimit
    heartbeat_timeout_ms: int
    max_command_age_ms: int = 1_000

    def __post_init__(self) -> None:
        motor_roles = dict(self.motor_roles)
        role_limits = dict(self.role_limits)
        if (
            not motor_roles
            or set(motor_roles) != set(role_limits)
            or any(
                not isinstance(role, str)
                or not role
                or not isinstance(port, str)
                or not port
                for role, port in motor_roles.items()
            )
            or len(set(motor_roles.values())) != len(motor_roles)
            or any(
                not isinstance(limit, MotionLimit)
                for limit in role_limits.values()
            )
        ):
            raise ValueError("Motor role limits are invalid")
        for name, value in (
            ("heartbeat_timeout_ms", self.heartbeat_timeout_ms),
            ("max_command_age_ms", self.max_command_age_ms),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError("{} must be a positive integer".format(name))
        object.__setattr__(
            self,
            "motor_roles",
            MappingProxyType(motor_roles),
        )
        object.__setattr__(
            self,
            "role_limits",
            MappingProxyType(role_limits),
        )

    @classmethod
    def from_file(cls, path: Path) -> "SafetyLimits":
        with path.open("r", encoding="utf-8") as handle:
            config = json.load(
                handle,
                object_pairs_hook=_strict_object,
                parse_constant=_reject_constant,
            )

        try:
            motor_config = config["motors"]
            roles = {
                role: details["port"]
                for role, details in motor_config.items()
            }
            limit_config = config["limits"]
            drive_limit = MotionLimit(**limit_config["drive"])
            arm_limit = MotionLimit(**limit_config["arm"])
            profiles = {
                "drive": drive_limit,
                "arm": arm_limit,
            }
            role_limits = {
                role: profiles[details["limit_profile"]]
                for role, details in motor_config.items()
            }
            drive_geometry = config["drive_geometry"]
            drive_roles = {
                drive_geometry["left_motor_role"],
                drive_geometry["right_motor_role"],
            }
            if (
                len(drive_roles) != 2
                or not drive_roles.issubset(roles)
                or any(
                    role_limits[role] is not drive_limit
                    for role in drive_roles
                )
            ):
                raise ValueError(
                    "Drive geometry does not identify two drive motors"
                )
            return cls(
                motor_roles=roles,
                role_limits=role_limits,
                drive=drive_limit,
                arm=arm_limit,
                heartbeat_timeout_ms=(
                    limit_config["heartbeat"]["timeout_ms"]
                ),
            )
        except (KeyError, TypeError, AttributeError):
            raise ValueError("Safety configuration is invalid") from None

    def limit_for_role(self, role: str) -> MotionLimit:
        if role not in self.role_limits:
            raise SafetyViolation(
                "unknown_motor", "Motor role {!r} is not configured".format(role)
            )
        return self.role_limits[role]


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate configuration key")
        result[key] = value
    return result


def _reject_constant(_value):
    raise ValueError("Non-finite configuration value")


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
