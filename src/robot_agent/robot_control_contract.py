"""Typed contracts for the dashboard's physical robot control plane.

The dashboard can observe and request robot episodes through these immutable
values.  Concrete navigation and EV3 transport live behind an injected runtime
adapter; this module contains no motor, SSH, or hardware imports.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
from typing import Mapping, Optional, Tuple

from .dashboard_contract import (
    DashboardContractError,
    RESPONSE_LOCALES,
    freeze_json,
    thaw_json,
)
from .lm_studio import DEFAULT_MODEL


CONTROL_SCHEMA = "robot-control/v1"
SETTINGS_SCHEMA = "robot-control-settings/v1"
START_REQUEST_SCHEMA = "robot-episode-start/v1"
EVENT_PAGE_SCHEMA = "robot-control-event-page/v1"
SNAPSHOT_PAGE_SCHEMA = "robot-control-snapshot-page/v1"

DISABLED = "DISABLED"
IDLE = "IDLE"
STARTING = "STARTING"
RUNNING = "RUNNING"
STOPPING = "STOPPING"
FAULTED = "FAULTED"
CONTROL_STATES = (
    DISABLED,
    IDLE,
    STARTING,
    RUNNING,
    STOPPING,
    FAULTED,
)
ACTIVE_STATES = (STARTING, RUNNING, STOPPING)

SPEECH_STATES = (
    "disabled",
    "idle",
    "generating",
    "queued",
    "playing",
    "completed",
    "dropped",
    "cancelled",
    "failed",
)
RUNTIME_UPDATE_FIELDS = frozenset(
    (
        "current_action",
        "obstacle",
        "plan",
        "scan",
        "model_latency_ms",
        "speech_status",
        "message",
    )
)

MAX_GOAL_CHARACTERS = 2_000
MAX_MODEL_CHARACTERS = 256
MAX_RUNTIME_UPDATE_BYTES = 32 * 1024
MAX_FAULT_DIAGNOSTIC_CHARACTERS = 240
MAX_PLAN_STEPS = 16
MAX_PLAN_STEP_CHARACTERS = 256
MAX_INT = 2**63 - 1


def _identifier(name: str, value: str, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 33 or ord(character) > 126 for character in value)
    ):
        raise DashboardContractError(
            "invalid_robot_identifier",
            "{} is invalid".format(name),
        )
    return value


def _text(name: str, value: str, maximum: int) -> str:
    if (
        not isinstance(value, str)
        or not value.strip()
        or value != value.strip()
        or len(value) > maximum
        or any(
            ord(character) < 32 and character not in "\n\r\t"
            for character in value
        )
    ):
        raise DashboardContractError(
            "invalid_robot_text",
            "{} is invalid".format(name),
        )
    return value


def _integer(
    name: str,
    value: int,
    minimum: int = 0,
    maximum: int = MAX_INT,
) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise DashboardContractError(
            "invalid_robot_integer",
            "{} is invalid".format(name),
        )
    return value


def _optional_json(name: str, value):
    if value is None:
        return None
    frozen = freeze_json(value)
    try:
        encoded = json.dumps(
            thaw_json(frozen),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except (TypeError, ValueError):
        raise DashboardContractError(
            "invalid_robot_runtime_update",
            "{} is invalid".format(name),
        ) from None
    if len(encoded) > MAX_RUNTIME_UPDATE_BYTES:
        raise DashboardContractError(
            "robot_runtime_update_too_large",
            "{} is too large".format(name),
        )
    return frozen


@dataclass(frozen=True)
class RobotControlSettings:
    """Settings captured atomically when a physical episode starts."""

    revision: int = 1
    model: str = DEFAULT_MODEL
    max_episode_ms: int = 15 * 60 * 1_000
    speech_enabled: bool = True

    def __post_init__(self) -> None:
        _integer("revision", self.revision, 1)
        _text("model", self.model, MAX_MODEL_CHARACTERS)
        if (
            self.model != self.model.strip()
            or any(
                ord(character) < 33 or ord(character) == 127
                for character in self.model
            )
        ):
            raise DashboardContractError(
                "invalid_robot_model",
                "Robot model is invalid",
            )
        _integer(
            "max_episode_ms",
            self.max_episode_ms,
            1_000,
            60 * 60 * 1_000,
        )
        if not isinstance(self.speech_enabled, bool):
            raise DashboardContractError(
                "invalid_robot_speech_setting",
                "Robot speech setting is invalid",
            )

    def with_updates(
        self,
        changes: Mapping[str, object],
    ) -> "RobotControlSettings":
        if not isinstance(changes, Mapping) or not changes:
            raise DashboardContractError(
                "invalid_robot_settings",
                "Robot settings changes must be a non-empty object",
            )
        allowed = {"model", "max_episode_ms", "speech_enabled"}
        if not set(changes) <= allowed:
            raise DashboardContractError(
                "invalid_robot_settings_fields",
                "Robot settings fields are invalid",
            )
        return replace(
            self,
            revision=self.revision + 1,
            **dict(changes),
        )

    def to_dict(self):
        return {
            "schema": SETTINGS_SCHEMA,
            "revision": self.revision,
            "model": self.model,
            "max_episode_ms": self.max_episode_ms,
            "speech_enabled": self.speech_enabled,
        }


@dataclass(frozen=True)
class RobotEpisodeStart:
    """The complete and deliberately small request accepted from the GUI."""

    goal: str
    locale: str
    client_request_id: str
    expected_revision: int

    def __post_init__(self) -> None:
        _text("goal", self.goal, MAX_GOAL_CHARACTERS)
        if self.locale not in RESPONSE_LOCALES:
            raise DashboardContractError(
                "invalid_robot_locale",
                "Robot response locale is unsupported",
            )
        _identifier("client_request_id", self.client_request_id)
        _integer("expected_revision", self.expected_revision, 1)

    def to_dict(self):
        return {
            "schema": START_REQUEST_SCHEMA,
            "goal": self.goal,
            "locale": self.locale,
            "client_request_id": self.client_request_id,
            "expected_revision": self.expected_revision,
        }


@dataclass(frozen=True)
class RobotRuntimeUpdate:
    """One validated, display-only update published by a runtime adapter."""

    current_action: Optional[str] = None
    obstacle: object = None
    plan: Tuple[str, ...] = ()
    scan: object = None
    model_latency_ms: Optional[int] = None
    speech_status: str = "idle"
    message: Optional[str] = None

    def __post_init__(self) -> None:
        if self.current_action is not None:
            _text("current_action", self.current_action, 256)
        object.__setattr__(
            self,
            "obstacle",
            _optional_json("obstacle", self.obstacle),
        )
        if (
            not isinstance(self.plan, tuple)
            or len(self.plan) > MAX_PLAN_STEPS
        ):
            raise DashboardContractError(
                "invalid_robot_plan",
                "Robot plan is invalid",
            )
        for step in self.plan:
            _text("plan step", step, MAX_PLAN_STEP_CHARACTERS)
        object.__setattr__(
            self,
            "scan",
            _optional_json("scan", self.scan),
        )
        if self.model_latency_ms is not None:
            _integer(
                "model_latency_ms",
                self.model_latency_ms,
                0,
                10 * 60 * 1_000,
            )
        if self.speech_status not in SPEECH_STATES:
            raise DashboardContractError(
                "invalid_robot_speech_status",
                "Robot speech status is invalid",
            )
        if self.message is not None:
            _text("message", self.message, 1_000)

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, object],
        previous: Optional["RobotRuntimeUpdate"] = None,
    ) -> "RobotRuntimeUpdate":
        if not isinstance(value, Mapping) or not value:
            raise DashboardContractError(
                "invalid_robot_runtime_update",
                "Robot runtime update must be a non-empty object",
            )
        if not set(value) <= RUNTIME_UPDATE_FIELDS:
            raise DashboardContractError(
                "invalid_robot_runtime_update_fields",
                "Robot runtime update fields are invalid",
            )
        prior = previous or cls()
        plan = value.get("plan", prior.plan)
        if isinstance(plan, list):
            plan = tuple(plan)
        return cls(
            current_action=value.get(
                "current_action",
                prior.current_action,
            ),
            obstacle=value.get("obstacle", prior.obstacle),
            plan=plan,
            scan=value.get("scan", prior.scan),
            model_latency_ms=value.get(
                "model_latency_ms",
                prior.model_latency_ms,
            ),
            speech_status=value.get(
                "speech_status",
                prior.speech_status,
            ),
            message=value.get("message", prior.message),
        )

    def to_dict(self):
        return {
            "current_action": self.current_action,
            "obstacle": thaw_json(self.obstacle),
            "plan": list(self.plan),
            "scan": thaw_json(self.scan),
            "model_latency_ms": self.model_latency_ms,
            "speech_status": self.speech_status,
            "message": self.message,
        }


@dataclass(frozen=True)
class RobotControlSnapshot:
    """One immutable public view of the robot control state machine."""

    sequence: int
    state: str
    enabled: bool
    accepting: bool
    settings: RobotControlSettings
    episode_id: Optional[str]
    goal: Optional[str]
    locale: Optional[str]
    started_at_unix_ms: Optional[int]
    updated_at_unix_ms: int
    terminal_reason: Optional[str]
    last_error_code: Optional[str]
    runtime: RobotRuntimeUpdate
    primary_error_code: Optional[str] = None
    primary_error_message: Optional[str] = None

    def __post_init__(self) -> None:
        _integer("sequence", self.sequence, 1)
        if self.state not in CONTROL_STATES:
            raise DashboardContractError(
                "invalid_robot_control_state",
                "Robot control state is invalid",
            )
        if not isinstance(self.enabled, bool) or not isinstance(
            self.accepting,
            bool,
        ):
            raise DashboardContractError(
                "invalid_robot_control_flags",
                "Robot control flags are invalid",
            )
        if not isinstance(self.settings, RobotControlSettings):
            raise DashboardContractError(
                "invalid_robot_settings",
                "Robot settings are invalid",
            )
        if self.episode_id is not None:
            _identifier("episode_id", self.episode_id)
        if self.goal is not None:
            _text("goal", self.goal, MAX_GOAL_CHARACTERS)
        if self.locale is not None and self.locale not in RESPONSE_LOCALES:
            raise DashboardContractError(
                "invalid_robot_locale",
                "Robot locale is invalid",
            )
        for name, value in (
            ("started_at_unix_ms", self.started_at_unix_ms),
            ("updated_at_unix_ms", self.updated_at_unix_ms),
        ):
            if value is not None:
                _integer(name, value)
        if self.terminal_reason is not None:
            _text("terminal_reason", self.terminal_reason, 128)
        if self.last_error_code is not None:
            _identifier("last_error_code", self.last_error_code)
        if (self.primary_error_code is None) != (
            self.primary_error_message is None
        ):
            raise DashboardContractError(
                "invalid_robot_fault_diagnostic",
                "Robot primary fault diagnostic is incomplete",
            )
        if self.primary_error_code is not None:
            _identifier("primary_error_code", self.primary_error_code)
            _text(
                "primary_error_message",
                self.primary_error_message,
                MAX_FAULT_DIAGNOSTIC_CHARACTERS,
            )
            if self.primary_error_message != " ".join(
                self.primary_error_message.split()
            ):
                raise DashboardContractError(
                    "invalid_robot_fault_diagnostic",
                    "Robot primary fault diagnostic must be single-line",
                )
        if not isinstance(self.runtime, RobotRuntimeUpdate):
            raise DashboardContractError(
                "invalid_robot_runtime_update",
                "Robot runtime view is invalid",
            )

    def to_dict(self):
        return {
            "schema": CONTROL_SCHEMA,
            "sequence": self.sequence,
            "state": self.state,
            "enabled": self.enabled,
            "accepting": self.accepting,
            "settings": self.settings.to_dict(),
            "episode": {
                "episode_id": self.episode_id,
                "goal": self.goal,
                "locale": self.locale,
                "started_at_unix_ms": self.started_at_unix_ms,
                "terminal_reason": self.terminal_reason,
            },
            "updated_at_unix_ms": self.updated_at_unix_ms,
            "last_error_code": self.last_error_code,
            "primary_error_code": self.primary_error_code,
            "primary_error_message": self.primary_error_message,
            "runtime": self.runtime.to_dict(),
        }


def finite_unix_ms(value) -> int:
    """Validate an injected clock result for public snapshots."""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DashboardContractError(
            "invalid_robot_clock",
            "Robot control clock returned an invalid value",
        )
    if not math.isfinite(float(value)):
        raise DashboardContractError(
            "invalid_robot_clock",
            "Robot control clock returned an invalid value",
        )
    result = int(value)
    _integer("clock", result)
    return result
