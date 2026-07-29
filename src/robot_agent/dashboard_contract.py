"""Typed, motion-free contracts for the local Mac dashboard.

The dashboard is intentionally a descriptive and conversational plane.  The
contracts in this module contain no executable callbacks, transport handles,
credentials, or imports from the robot execution stack.
"""

from dataclasses import dataclass, replace
import json
import math
from types import MappingProxyType
from typing import Mapping, Optional, Tuple


SETTINGS_SCHEMA = "dashboard-settings/v1"
EVENT_SCHEMA = "technical-event/v1"
EVENT_PAGE_SCHEMA = "technical-event-page/v1"
ROBOT_SCHEMA = "dashboard-robot/v1"
NODE_SCHEMA = "dashboard-node/v1"
REGISTRY_SCHEMA = "dashboard-registry/v1"
EXPERIMENT_SCHEMA = "dashboard-experiment/v1"
MESSAGE_SCHEMA = "chat-message/v1"
CONVERSATION_SCHEMA = "conversation/v1"
TURN_SCHEMA = "chat-turn/v1"

CHAT_MODES = ("conversation", "research_required")
RESPONSE_LOCALES = ("sv", "en")
RESPONSE_LOCALE_NAMES = {
    "sv": "Swedish",
    "en": "English",
}
REGISTRY_DISPLAY_NAME_KEYS = (
    "registry.names.composite_lab_robot",
    "registry.names.front_camera",
    "registry.names.microphone_array",
    "registry.names.vision_node",
    "registry.names.audio_node",
    "registry.names.mac_host",
)
EXPERIMENT_TITLE_KEYS = (
    "experiments.curated.dynamic_ir.title",
    "experiments.curated.weather_tool.title",
    "experiments.curated.idle_autonomy.title",
    "experiments.curated.ev3_preflight.title",
)
EXPERIMENT_SUMMARY_KEYS = (
    "experiments.curated.dynamic_ir.summary",
    "experiments.curated.weather_tool.summary",
    "experiments.curated.idle_autonomy.summary",
    "experiments.curated.ev3_preflight.summary",
)
EXPERIMENT_STATUSES = ("verified", "waiting")
LOG_LEVELS = ("debug", "info", "warning", "error")
ROBOT_LIFECYCLES = (
    "declared",
    "observed_online",
    "observed_offline",
    "stale",
)
NODE_LIFECYCLES = ROBOT_LIFECYCLES
NODE_KINDS = (
    "controller",
    "camera",
    "microphone",
    "compute",
    "model_server",
    "provider",
    "other",
)
TURN_STATUSES = (
    "queued",
    "running",
    "answered",
    "clarification_required",
    "failed",
)
TERMINAL_TURN_STATUSES = (
    "answered",
    "clarification_required",
    "failed",
)

_MAX_INT = 2**63 - 1
_MAX_JSON_DEPTH = 16
_MAX_JSON_ITEMS = 4_096


class DashboardContractError(ValueError):
    """A safely reportable dashboard contract violation."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


def strict_json_loads(raw):
    """Decode strict UTF-8 JSON.

    The HTTP caller owns the byte-size limit.  This decoder rejects duplicate
    keys, non-finite numbers, malformed UTF-8 and excessive recursion.
    """

    if not isinstance(raw, (bytes, str)) or not raw:
        raise DashboardContractError(
            "invalid_json_body",
            "JSON body must be non-empty UTF-8 bytes or text",
        )
    try:
        text = raw.decode("utf-8") if isinstance(raw, bytes) else raw
        return json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (
        RecursionError,
        UnicodeDecodeError,
        TypeError,
        ValueError,
    ):
        raise DashboardContractError(
            "invalid_json",
            "Request body is not strict JSON",
        ) from None


def strict_json_object(raw) -> Mapping[str, object]:
    value = strict_json_loads(raw)
    if not isinstance(value, dict):
        raise DashboardContractError(
            "invalid_json_shape",
            "Request body must be a JSON object",
        )
    return value


def _identifier(name: str, value: str, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise DashboardContractError(
            "invalid_identifier",
            "{} is invalid".format(name),
        )
    return value


def _optional_identifier(
    name: str,
    value: Optional[str],
    maximum: int = 128,
) -> Optional[str]:
    if value is not None:
        _identifier(name, value, maximum)
    return value


def _text(
    name: str,
    value: str,
    maximum: int,
    *,
    allow_newlines: bool = True,
) -> str:
    if (
        not isinstance(value, str)
        or not value.strip()
        or value != value.strip()
        or len(value) > maximum
        or any(
            ord(character) < 32
            and (
                character not in "\n\r\t"
                or not allow_newlines
            )
            for character in value
        )
    ):
        raise DashboardContractError(
            "invalid_text",
            "{} is invalid".format(name),
        )
    return value


def _optional_text(
    name: str,
    value: Optional[str],
    maximum: int,
) -> Optional[str]:
    if value is not None:
        _text(name, value, maximum)
    return value


def _integer(
    name: str,
    value: int,
    minimum: int,
    maximum: int,
) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise DashboardContractError(
            "invalid_integer",
            "{} is invalid".format(name),
        )
    return value


def freeze_json(value, _depth=0, _counter=None):
    """Create an immutable deep copy of a finite JSON-compatible value."""

    if _counter is None:
        _counter = [0]
    _counter[0] += 1
    if _depth > _MAX_JSON_DEPTH or _counter[0] > _MAX_JSON_ITEMS:
        raise DashboardContractError(
            "json_complexity_limit",
            "JSON value exceeded the dashboard complexity limit",
        )
    if isinstance(value, Mapping):
        result = {}
        for key, item in value.items():
            if (
                not isinstance(key, str)
                or not key
                or len(key) > 256
                or any(ord(character) < 32 for character in key)
            ):
                raise DashboardContractError(
                    "invalid_json_value",
                    "JSON object key is invalid",
                )
            result[key] = freeze_json(
                item,
                _depth + 1,
                _counter,
            )
        return MappingProxyType(result)
    if isinstance(value, (list, tuple)):
        return tuple(
            freeze_json(item, _depth + 1, _counter)
            for item in value
        )
    if value is None or type(value) in (bool, int, str):
        return value
    if isinstance(value, float) and math.isfinite(value):
        return value
    raise DashboardContractError(
        "invalid_json_value",
        "Value is not finite JSON data",
    )


def thaw_json(value):
    if isinstance(value, Mapping):
        return {
            key: thaw_json(item)
            for key, item in value.items()
        }
    if isinstance(value, tuple):
        return [thaw_json(item) for item in value]
    return value


@dataclass(frozen=True)
class DashboardSettings:
    """Session-only settings captured by revision for every queued turn."""

    revision: int = 1
    chat_mode: str = "conversation"
    log_level: str = "debug"
    max_elapsed_ms: int = 30_000
    max_planner_latency_ms: int = 10_000
    max_planner_turns: int = 6
    max_tool_calls: int = 1
    max_replans: int = 4
    tool_request_ttl_ms: int = 8_000
    evidence_ttl_ms: int = 10 * 60 * 1_000
    max_weather_observation_skew_ms: int = 2 * 60 * 60 * 1_000

    def __post_init__(self) -> None:
        _integer("revision", self.revision, 1, _MAX_INT)
        if self.chat_mode not in CHAT_MODES:
            raise DashboardContractError(
                "invalid_chat_mode",
                "Chat mode is unsupported",
            )
        if self.log_level not in LOG_LEVELS:
            raise DashboardContractError(
                "invalid_log_level",
                "Log level is unsupported",
            )
        _integer(
            "max_elapsed_ms",
            self.max_elapsed_ms,
            1,
            300_000,
        )
        _integer(
            "max_planner_latency_ms",
            self.max_planner_latency_ms,
            1,
            self.max_elapsed_ms,
        )
        _integer(
            "max_planner_turns",
            self.max_planner_turns,
            1,
            100,
        )
        _integer("max_tool_calls", self.max_tool_calls, 0, 100)
        _integer("max_replans", self.max_replans, 0, 100)
        _integer(
            "tool_request_ttl_ms",
            self.tool_request_ttl_ms,
            1,
            self.max_elapsed_ms,
        )
        _integer(
            "evidence_ttl_ms",
            self.evidence_ttl_ms,
            1,
            24 * 60 * 60 * 1_000,
        )
        _integer(
            "max_weather_observation_skew_ms",
            self.max_weather_observation_skew_ms,
            1,
            24 * 60 * 60 * 1_000,
        )

    @classmethod
    def defaults(cls) -> "DashboardSettings":
        return cls()

    @property
    def require_evidence(self) -> bool:
        return self.chat_mode == "research_required"

    def to_research_limits_kwargs(self) -> Mapping[str, int]:
        return {
            "max_elapsed_ms": self.max_elapsed_ms,
            "max_planner_latency_ms": self.max_planner_latency_ms,
            "max_planner_turns": self.max_planner_turns,
            "max_tool_calls": self.max_tool_calls,
            "max_replans": self.max_replans,
            "tool_request_ttl_ms": self.tool_request_ttl_ms,
            "evidence_ttl_ms": self.evidence_ttl_ms,
            "max_weather_observation_skew_ms": (
                self.max_weather_observation_skew_ms
            ),
        }

    def with_updates(
        self,
        changes: Mapping[str, object],
        next_revision: Optional[int] = None,
    ) -> "DashboardSettings":
        if not isinstance(changes, Mapping) or not changes:
            raise DashboardContractError(
                "invalid_settings_update",
                "Settings changes must be a non-empty object",
            )
        allowed = {"chat_mode", "log_level", "research"}
        if not set(changes).issubset(allowed):
            raise DashboardContractError(
                "unknown_setting",
                "Settings update contains an unknown field",
            )
        values = {}
        if "chat_mode" in changes:
            values["chat_mode"] = changes["chat_mode"]
        if "log_level" in changes:
            values["log_level"] = changes["log_level"]
        if "research" in changes:
            research = changes["research"]
            allowed_research = set(self.to_research_limits_kwargs())
            if (
                not isinstance(research, Mapping)
                or not research
                or not set(research).issubset(allowed_research)
            ):
                raise DashboardContractError(
                    "unknown_setting",
                    "Research settings contain an unknown field",
                )
            values.update(research)
        revision = (
            self.revision + 1
            if next_revision is None
            else next_revision
        )
        _integer("next_revision", revision, self.revision + 1, _MAX_INT)
        return replace(self, revision=revision, **values)

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": SETTINGS_SCHEMA,
            "revision": self.revision,
            "chat_mode": self.chat_mode,
            "require_evidence": self.require_evidence,
            "log_level": self.log_level,
            "persistence": "memory_only",
            "resets_on_restart": True,
            "research": dict(self.to_research_limits_kwargs()),
        }


@dataclass(frozen=True)
class TechnicalEvent:
    server_instance_id: str
    sequence: int
    event_id: str
    recorded_at_unix_ms: int
    recorded_at_monotonic_ms: int
    level: str
    category: str
    event_type: str
    source_id: str
    message: str
    data: Mapping[str, object]
    robot_id: Optional[str] = None
    node_id: Optional[str] = None
    request_id: Optional[str] = None
    conversation_id: Optional[str] = None
    turn_id: Optional[str] = None
    tool_call_id: Optional[str] = None

    def __post_init__(self) -> None:
        _identifier("server_instance_id", self.server_instance_id)
        _integer("sequence", self.sequence, 1, _MAX_INT)
        _identifier("event_id", self.event_id)
        _integer(
            "recorded_at_unix_ms",
            self.recorded_at_unix_ms,
            0,
            _MAX_INT,
        )
        _integer(
            "recorded_at_monotonic_ms",
            self.recorded_at_monotonic_ms,
            0,
            _MAX_INT,
        )
        if self.level not in LOG_LEVELS:
            raise DashboardContractError(
                "invalid_log_level",
                "Event log level is unsupported",
            )
        _identifier("category", self.category, 64)
        _identifier("event_type", self.event_type)
        _identifier("source_id", self.source_id)
        _text("event message", self.message, 1_000, allow_newlines=False)
        for name in (
            "robot_id",
            "node_id",
            "request_id",
            "conversation_id",
            "turn_id",
            "tool_call_id",
        ):
            _optional_identifier(name, getattr(self, name))
        if not isinstance(self.data, Mapping):
            raise DashboardContractError(
                "invalid_event_data",
                "Event data must be an object",
            )
        object.__setattr__(self, "data", freeze_json(dict(self.data)))

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": EVENT_SCHEMA,
            "server_instance_id": self.server_instance_id,
            "sequence": self.sequence,
            "event_id": self.event_id,
            "recorded_at_unix_ms": self.recorded_at_unix_ms,
            "recorded_at_monotonic_ms": self.recorded_at_monotonic_ms,
            "level": self.level,
            "category": self.category,
            "event_type": self.event_type,
            "source": {
                "source_id": self.source_id,
                "robot_id": self.robot_id,
                "node_id": self.node_id,
            },
            "correlation": {
                "request_id": self.request_id,
                "conversation_id": self.conversation_id,
                "turn_id": self.turn_id,
                "tool_call_id": self.tool_call_id,
            },
            "message": self.message,
            "data": thaw_json(self.data),
        }


@dataclass(frozen=True)
class EventPage:
    server_instance_id: str
    after_sequence: int
    oldest_sequence: Optional[int]
    newest_sequence: int
    next_after_sequence: int
    gap: bool
    dropped_total: int
    events: Tuple[TechnicalEvent, ...]

    def __post_init__(self) -> None:
        _identifier("server_instance_id", self.server_instance_id)
        _integer("after_sequence", self.after_sequence, 0, _MAX_INT)
        if self.oldest_sequence is not None:
            _integer(
                "oldest_sequence",
                self.oldest_sequence,
                1,
                _MAX_INT,
            )
        _integer("newest_sequence", self.newest_sequence, 0, _MAX_INT)
        _integer(
            "next_after_sequence",
            self.next_after_sequence,
            0,
            _MAX_INT,
        )
        if type(self.gap) is not bool:
            raise DashboardContractError(
                "invalid_gap",
                "Event page gap marker is invalid",
            )
        _integer("dropped_total", self.dropped_total, 0, _MAX_INT)
        if (
            not isinstance(self.events, tuple)
            or any(
                not isinstance(event, TechnicalEvent)
                for event in self.events
            )
        ):
            raise DashboardContractError(
                "invalid_events",
                "Event page contents are invalid",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": EVENT_PAGE_SCHEMA,
            "server_instance_id": self.server_instance_id,
            "after_sequence": self.after_sequence,
            "oldest_sequence": self.oldest_sequence,
            "newest_sequence": self.newest_sequence,
            "next_after_sequence": self.next_after_sequence,
            "gap": self.gap,
            "dropped_total": self.dropped_total,
            "events": [event.to_dict() for event in self.events],
        }


@dataclass(frozen=True)
class ExperimentDescriptor:
    experiment_id: str
    title_key: str
    summary_key: str
    status: str
    component_ids: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        _identifier("experiment_id", self.experiment_id)
        if self.title_key not in EXPERIMENT_TITLE_KEYS:
            raise DashboardContractError(
                "invalid_experiment_title_key",
                "Experiment title catalog key is unsupported",
            )
        if self.summary_key not in EXPERIMENT_SUMMARY_KEYS:
            raise DashboardContractError(
                "invalid_experiment_summary_key",
                "Experiment summary catalog key is unsupported",
            )
        if self.status not in EXPERIMENT_STATUSES:
            raise DashboardContractError(
                "invalid_experiment_status",
                "Experiment status is unsupported",
            )
        if (
            not isinstance(self.component_ids, tuple)
            or len(set(self.component_ids)) != len(self.component_ids)
        ):
            raise DashboardContractError(
                "invalid_experiment_component_ids",
                "Experiment component IDs are invalid",
            )
        for component_id in self.component_ids:
            _identifier("component_id", component_id)

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": EXPERIMENT_SCHEMA,
            "experiment_id": self.experiment_id,
            "title_key": self.title_key,
            "summary_key": self.summary_key,
            "status": self.status,
            "component_ids": list(self.component_ids),
        }


@dataclass(frozen=True)
class RobotDescriptor:
    robot_id: str
    display_name: str
    robot_kind: str
    display_name_key: Optional[str] = None
    lifecycle: str = "declared"
    node_ids: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        _identifier("robot_id", self.robot_id)
        _text("display_name", self.display_name, 120)
        _identifier("robot_kind", self.robot_kind, 64)
        if (
            self.display_name_key is not None
            and self.display_name_key not in REGISTRY_DISPLAY_NAME_KEYS
        ):
            raise DashboardContractError(
                "invalid_registry_display_name_key",
                "Robot display-name catalog key is unsupported",
            )
        if self.lifecycle not in ROBOT_LIFECYCLES:
            raise DashboardContractError(
                "invalid_robot_lifecycle",
                "Robot lifecycle is unsupported",
            )
        if (
            not isinstance(self.node_ids, tuple)
            or len(set(self.node_ids)) != len(self.node_ids)
        ):
            raise DashboardContractError(
                "invalid_node_ids",
                "Robot node IDs are invalid",
            )
        for node_id in self.node_ids:
            _identifier("node_id", node_id)

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": ROBOT_SCHEMA,
            "robot_id": self.robot_id,
            "display_name": self.display_name,
            "display_name_key": self.display_name_key,
            "robot_kind": self.robot_kind,
            "lifecycle": self.lifecycle,
            "node_ids": list(self.node_ids),
            "control_exposed": False,
        }


@dataclass(frozen=True)
class NodeDescriptor:
    node_id: str
    display_name: str
    node_kind: str
    display_name_key: Optional[str] = None
    lifecycle: str = "declared"
    robot_id: Optional[str] = None
    controller_id: Optional[str] = None
    source_id: Optional[str] = None
    capabilities: Tuple[str, ...] = ()
    last_observed_at_unix_ms: Optional[int] = None
    last_observed_at_monotonic_ms: Optional[int] = None
    status_reason_code: Optional[str] = None
    control_exposed: bool = False

    def __post_init__(self) -> None:
        _identifier("node_id", self.node_id)
        _text("display_name", self.display_name, 120)
        if (
            self.display_name_key is not None
            and self.display_name_key not in REGISTRY_DISPLAY_NAME_KEYS
        ):
            raise DashboardContractError(
                "invalid_registry_display_name_key",
                "Node display-name catalog key is unsupported",
            )
        if self.node_kind not in NODE_KINDS:
            raise DashboardContractError(
                "invalid_node_kind",
                "Node kind is unsupported",
            )
        if self.lifecycle not in NODE_LIFECYCLES:
            raise DashboardContractError(
                "invalid_node_lifecycle",
                "Node lifecycle is unsupported",
            )
        _optional_identifier("robot_id", self.robot_id)
        _optional_identifier("controller_id", self.controller_id)
        _optional_identifier("source_id", self.source_id)
        _optional_identifier(
            "status_reason_code",
            self.status_reason_code,
            64,
        )
        if (
            not isinstance(self.capabilities, tuple)
            or len(set(self.capabilities)) != len(self.capabilities)
        ):
            raise DashboardContractError(
                "invalid_capabilities",
                "Node capabilities are invalid",
            )
        for capability in self.capabilities:
            _identifier("capability", capability)
        times = (
            self.last_observed_at_unix_ms,
            self.last_observed_at_monotonic_ms,
        )
        if (times[0] is None) != (times[1] is None):
            raise DashboardContractError(
                "invalid_observation_time",
                "Node observation times must be supplied together",
            )
        if times[0] is not None:
            _integer("last_observed_at_unix_ms", times[0], 0, _MAX_INT)
            _integer(
                "last_observed_at_monotonic_ms",
                times[1],
                0,
                _MAX_INT,
            )
        if self.lifecycle != "declared" and times[0] is None:
            raise DashboardContractError(
                "missing_observation_time",
                "Observed node lifecycle requires observation times",
            )
        if type(self.control_exposed) is not bool or self.control_exposed:
            raise DashboardContractError(
                "physical_control_forbidden",
                "Dashboard registry cannot expose physical control",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": NODE_SCHEMA,
            "node_id": self.node_id,
            "display_name": self.display_name,
            "display_name_key": self.display_name_key,
            "node_kind": self.node_kind,
            "lifecycle": self.lifecycle,
            "robot_id": self.robot_id,
            "controller_id": self.controller_id,
            "source_id": self.source_id,
            "capabilities": list(self.capabilities),
            "last_observed_at_unix_ms": (
                self.last_observed_at_unix_ms
            ),
            "last_observed_at_monotonic_ms": (
                self.last_observed_at_monotonic_ms
            ),
            "status_reason_code": self.status_reason_code,
            "control_exposed": False,
        }


@dataclass(frozen=True)
class RegistrySnapshot:
    server_instance_id: str
    version: int
    generated_at_unix_ms: int
    generated_at_monotonic_ms: int
    robots: Tuple[RobotDescriptor, ...]
    nodes: Tuple[NodeDescriptor, ...]
    physical_control_enabled: bool = False

    def __post_init__(self) -> None:
        _identifier("server_instance_id", self.server_instance_id)
        _integer("version", self.version, 1, _MAX_INT)
        _integer(
            "generated_at_unix_ms",
            self.generated_at_unix_ms,
            0,
            _MAX_INT,
        )
        _integer(
            "generated_at_monotonic_ms",
            self.generated_at_monotonic_ms,
            0,
            _MAX_INT,
        )
        if (
            not isinstance(self.robots, tuple)
            or any(
                not isinstance(robot, RobotDescriptor)
                for robot in self.robots
            )
            or not isinstance(self.nodes, tuple)
            or any(
                not isinstance(node, NodeDescriptor)
                for node in self.nodes
            )
        ):
            raise DashboardContractError(
                "invalid_registry",
                "Registry contents are invalid",
            )
        if type(self.physical_control_enabled) is not bool or (
            self.physical_control_enabled
        ):
            raise DashboardContractError(
                "physical_control_forbidden",
                "Dashboard registry cannot enable physical control",
            )
        robot_ids = [robot.robot_id for robot in self.robots]
        node_ids = [node.node_id for node in self.nodes]
        if (
            len(set(robot_ids)) != len(robot_ids)
            or len(set(node_ids)) != len(node_ids)
        ):
            raise DashboardContractError(
                "duplicate_registry_identity",
                "Registry identities must be unique",
            )
        by_robot = {robot.robot_id: robot for robot in self.robots}
        by_node = {node.node_id: node for node in self.nodes}
        controllers = [
            node.controller_id
            for node in self.nodes
            if node.controller_id is not None
        ]
        sources = [
            node.source_id
            for node in self.nodes
            if node.source_id is not None
        ]
        if (
            len(set(controllers)) != len(controllers)
            or len(set(sources)) != len(sources)
        ):
            raise DashboardContractError(
                "duplicate_registry_identity",
                "Controller and source identities must be unique",
            )
        for node in self.nodes:
            if node.robot_id is not None and node.robot_id not in by_robot:
                raise DashboardContractError(
                    "unknown_robot",
                    "Node references an unknown robot",
                )
        for robot in self.robots:
            for node_id in robot.node_ids:
                node = by_node.get(node_id)
                if node is None or node.robot_id != robot.robot_id:
                    raise DashboardContractError(
                        "registry_association_mismatch",
                        "Robot and node association is inconsistent",
                    )
            associated = {
                node.node_id
                for node in self.nodes
                if node.robot_id == robot.robot_id
            }
            if associated != set(robot.node_ids):
                raise DashboardContractError(
                    "registry_association_mismatch",
                    "Robot and node association list is incomplete",
                )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": REGISTRY_SCHEMA,
            "server_instance_id": self.server_instance_id,
            "version": self.version,
            "generated_at_unix_ms": self.generated_at_unix_ms,
            "generated_at_monotonic_ms": self.generated_at_monotonic_ms,
            "physical_control_enabled": False,
            "robots": [robot.to_dict() for robot in self.robots],
            "nodes": [node.to_dict() for node in self.nodes],
        }


@dataclass(frozen=True)
class ChatMessage:
    message_id: str
    turn_id: str
    role: str
    content: str
    created_at_unix_ms: int
    citation_ids: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        _identifier("message_id", self.message_id)
        _identifier("turn_id", self.turn_id)
        if self.role not in ("user", "assistant"):
            raise DashboardContractError(
                "invalid_message_role",
                "Chat message role is unsupported",
            )
        _text("message content", self.content, 4_000)
        _integer(
            "created_at_unix_ms",
            self.created_at_unix_ms,
            0,
            _MAX_INT,
        )
        if (
            not isinstance(self.citation_ids, tuple)
            or len(set(self.citation_ids)) != len(self.citation_ids)
            or len(self.citation_ids) > 32
        ):
            raise DashboardContractError(
                "invalid_citation_ids",
                "Message citation IDs are invalid",
            )
        for citation_id in self.citation_ids:
            _identifier("citation_id", citation_id)
        if self.role == "user" and self.citation_ids:
            raise DashboardContractError(
                "invalid_citation_ids",
                "User messages cannot carry research citations",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": MESSAGE_SCHEMA,
            "message_id": self.message_id,
            "turn_id": self.turn_id,
            "role": self.role,
            "content": self.content,
            "created_at_unix_ms": self.created_at_unix_ms,
            "citation_ids": list(self.citation_ids),
        }


@dataclass(frozen=True)
class Conversation:
    conversation_id: str
    version: int
    created_at_unix_ms: int
    updated_at_unix_ms: int
    messages: Tuple[ChatMessage, ...]
    active_turn_id: Optional[str] = None
    title: Optional[str] = None
    context_mode: str = "typed_history"

    def __post_init__(self) -> None:
        _identifier("conversation_id", self.conversation_id)
        _integer("version", self.version, 1, _MAX_INT)
        _integer(
            "created_at_unix_ms",
            self.created_at_unix_ms,
            0,
            _MAX_INT,
        )
        _integer(
            "updated_at_unix_ms",
            self.updated_at_unix_ms,
            self.created_at_unix_ms,
            _MAX_INT,
        )
        _optional_identifier("active_turn_id", self.active_turn_id)
        _optional_text("conversation title", self.title, 120)
        if self.context_mode != "typed_history":
            raise DashboardContractError(
                "invalid_context_mode",
                "Conversation context mode is unsupported",
            )
        if (
            not isinstance(self.messages, tuple)
            or any(
                not isinstance(message, ChatMessage)
                for message in self.messages
            )
        ):
            raise DashboardContractError(
                "invalid_messages",
                "Conversation messages are invalid",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": CONVERSATION_SCHEMA,
            "conversation_id": self.conversation_id,
            "version": self.version,
            "title": self.title,
            "context_mode": self.context_mode,
            "created_at_unix_ms": self.created_at_unix_ms,
            "updated_at_unix_ms": self.updated_at_unix_ms,
            "active_turn_id": self.active_turn_id,
            "messages": [message.to_dict() for message in self.messages],
        }


@dataclass(frozen=True)
class ChatTurn:
    turn_id: str
    conversation_id: str
    client_request_id: str
    mode: str
    response_locale: str
    settings_revision: int
    status: str
    content: str
    created_at_unix_ms: int
    started_at_unix_ms: Optional[int] = None
    completed_at_unix_ms: Optional[int] = None
    answer_text: Optional[str] = None
    clarification_question: Optional[str] = None
    citation_ids: Tuple[str, ...] = ()
    error_code: Optional[str] = None

    def __post_init__(self) -> None:
        _identifier("turn_id", self.turn_id)
        _identifier("conversation_id", self.conversation_id)
        _identifier("client_request_id", self.client_request_id)
        if self.mode not in CHAT_MODES:
            raise DashboardContractError(
                "invalid_chat_mode",
                "Chat turn mode is unsupported",
            )
        if self.response_locale not in RESPONSE_LOCALES:
            raise DashboardContractError(
                "invalid_response_locale",
                "Chat response locale is unsupported",
            )
        _integer(
            "settings_revision",
            self.settings_revision,
            1,
            _MAX_INT,
        )
        if self.status not in TURN_STATUSES:
            raise DashboardContractError(
                "invalid_turn_status",
                "Chat turn status is unsupported",
            )
        _text("turn content", self.content, 4_000)
        _integer(
            "created_at_unix_ms",
            self.created_at_unix_ms,
            0,
            _MAX_INT,
        )
        if self.started_at_unix_ms is not None:
            _integer(
                "started_at_unix_ms",
                self.started_at_unix_ms,
                self.created_at_unix_ms,
                _MAX_INT,
            )
        if self.completed_at_unix_ms is not None:
            minimum = (
                self.started_at_unix_ms
                if self.started_at_unix_ms is not None
                else self.created_at_unix_ms
            )
            _integer(
                "completed_at_unix_ms",
                self.completed_at_unix_ms,
                minimum,
                _MAX_INT,
            )
        _optional_text("answer_text", self.answer_text, 4_000)
        _optional_text(
            "clarification_question",
            self.clarification_question,
            1_000,
        )
        _optional_identifier("error_code", self.error_code, 64)
        if (
            not isinstance(self.citation_ids, tuple)
            or len(set(self.citation_ids)) != len(self.citation_ids)
            or len(self.citation_ids) > 32
        ):
            raise DashboardContractError(
                "invalid_citation_ids",
                "Turn citation IDs are invalid",
            )
        for citation_id in self.citation_ids:
            _identifier("citation_id", citation_id)

        if self.status == "queued":
            valid_shape = (
                self.started_at_unix_ms is None
                and self.completed_at_unix_ms is None
                and self.answer_text is None
                and self.clarification_question is None
                and not self.citation_ids
                and self.error_code is None
            )
        elif self.status == "running":
            valid_shape = (
                self.started_at_unix_ms is not None
                and self.completed_at_unix_ms is None
                and self.answer_text is None
                and self.clarification_question is None
                and not self.citation_ids
                and self.error_code is None
            )
        elif self.status == "answered":
            valid_shape = (
                self.started_at_unix_ms is not None
                and self.completed_at_unix_ms is not None
                and self.answer_text is not None
                and self.clarification_question is None
                and self.error_code is None
            )
        elif self.status == "clarification_required":
            valid_shape = (
                self.started_at_unix_ms is not None
                and self.completed_at_unix_ms is not None
                and self.answer_text is None
                and self.clarification_question is not None
                and not self.citation_ids
                and self.error_code is None
            )
        else:
            valid_shape = (
                self.started_at_unix_ms is not None
                and self.completed_at_unix_ms is not None
                and self.answer_text is None
                and self.clarification_question is None
                and not self.citation_ids
                and self.error_code is not None
            )
        if not valid_shape:
            raise DashboardContractError(
                "invalid_turn_shape",
                "Chat turn fields do not match its status",
            )

    @property
    def terminal(self) -> bool:
        return self.status in TERMINAL_TURN_STATUSES

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": TURN_SCHEMA,
            "turn_id": self.turn_id,
            "conversation_id": self.conversation_id,
            "client_request_id": self.client_request_id,
            "mode": self.mode,
            "response_locale": self.response_locale,
            "require_evidence": self.mode == "research_required",
            "settings_revision": self.settings_revision,
            "status": self.status,
            "content": self.content,
            "created_at_unix_ms": self.created_at_unix_ms,
            "started_at_unix_ms": self.started_at_unix_ms,
            "completed_at_unix_ms": self.completed_at_unix_ms,
            "answer_text": self.answer_text,
            "clarification_question": self.clarification_question,
            "citation_ids": list(self.citation_ids),
            "error_code": self.error_code,
        }
