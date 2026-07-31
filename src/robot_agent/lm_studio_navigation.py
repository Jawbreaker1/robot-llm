"""Structured LM Studio planner for physical goal-directed navigation."""

from dataclasses import dataclass
import ipaddress
import json
import time
from typing import Callable, Mapping, Optional, Sequence
from urllib.parse import urlsplit

from .lm_studio import _stdlib_post
from .maneuver_commitment import (
    DETOUR_SIDES,
    FACT_KEYS,
    FIELDS as MANEUVER_FIELDS,
    TRANSITIONS,
)
from .physical_navigation_contract import (
    DECISION_SCHEMA,
    MOTION_ACTIONS,
    NavigationDecision,
    PhysicalNavigationContractError,
    REASON_CODES,
    SCAN_FRONT_ARC,
    json_bytes,
    strict_json_loads,
)


MAX_MODEL_RESPONSE_BYTES = 64 * 1024
MAX_PLANNER_CONTEXT_BYTES = 64 * 1024
OPENAI_V1_CHAT_COMPLETIONS_PATH = "/v1/chat/completions"
LM_STUDIO_V0_CHAT_COMPLETIONS_PATH = "/api/v0/chat/completions"
CHAT_COMPLETIONS_PATHS = frozenset(
    (
        OPENAI_V1_CHAT_COMPLETIONS_PATH,
        LM_STUDIO_V0_CHAT_COMPLETIONS_PATH,
    )
)
UTTERANCE_LANGUAGE_BY_LOCALE = {
    "sv": "Swedish or null",
    "en": "English or null",
}


class LMStudioNavigationError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        code: str = "lm_studio_navigation_failed",
        latency_ms: int = 0,
    ):
        self.code = code
        self.latency_ms = latency_ms
        super().__init__(message)


class LMStudioNavigationDecisionError(LMStudioNavigationError):
    """A completed model call whose proposed decision violated the contract."""

    def __init__(self, code: str, message: str, *, latency_ms: int):
        self.feedback_message = message
        super().__init__(
            "LM Studio returned an invalid navigation decision: {}".format(
                message
            ),
            code=code,
            latency_ms=latency_ms,
        )


_RFC1918_NETWORKS = tuple(
    ipaddress.ip_network(value)
    for value in (
        "10.0.0.0/8",
        "172.16.0.0/12",
        "192.168.0.0/16",
    )
)
_PRIVATE_IPV6_NETWORK = ipaddress.ip_network("fc00::/7")


def _private_lan_address(address) -> bool:
    if address.version == 4:
        return any(address in network for network in _RFC1918_NETWORKS)
    return address in _PRIVATE_IPV6_NETWORK


def _base_url(value: str, *, allow_private_lan: bool = False) -> str:
    try:
        parsed = urlsplit(value)
        hostname = parsed.hostname
        parsed.port
    except (AttributeError, ValueError):
        raise LMStudioNavigationError("LM Studio base URL is invalid") from None
    if (
        parsed.scheme not in ("http", "https")
        or not parsed.netloc
        or parsed.username is not None
        or parsed.password is not None
        or parsed.path not in ("", "/")
        or parsed.query
        or parsed.fragment
        or hostname is None
    ):
        raise LMStudioNavigationError("LM Studio base URL is invalid")
    if not isinstance(allow_private_lan, bool):
        raise LMStudioNavigationError("LM Studio base URL is invalid")
    if hostname.lower() != "localhost":
        try:
            address = ipaddress.ip_address(hostname)
            if not address.is_loopback and not (
                allow_private_lan and _private_lan_address(address)
            ):
                raise ValueError
        except ValueError:
            raise LMStudioNavigationError(
                "LM Studio must use an allowed numeric address"
            ) from None
    return value.rstrip("/")


def _model_id(value: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 200
        or any(ord(character) < 32 for character in value)
    ):
        raise LMStudioNavigationError("LM Studio model id is invalid")
    return value


@dataclass(frozen=True)
class NavigationPlannerResult:
    decision: NavigationDecision
    latency_ms: int
    served_model: Optional[str]
    usage: Optional[Mapping[str, object]]
    stats: Optional[Mapping[str, object]] = None


def _empty_maneuver_schema() -> Mapping[str, object]:
    none_properties = {
        "id": {"type": "null"},
        "revision": {"type": "integer", "const": 0},
        "transition": {"type": "string", "const": "NONE"},
        "objective": {"type": "null"},
        "target_hypothesis_id": {"type": "null"},
        "detour_side": {"type": "null"},
        "success_fact_keys": {
            "type": "array",
            "items": {"type": "string"},
            "minItems": 0,
            "maxItems": 0,
        },
        "current_focus_fact_key": {"type": "null"},
        "revision_reason": {"type": "null"},
    }
    return {
        "type": "object",
        "properties": none_properties,
        "required": sorted(MANEUVER_FIELDS),
        "additionalProperties": False,
    }


def _active_maneuver_schema() -> Mapping[str, object]:
    active_properties = {
        "id": {"type": "string", "minLength": 1, "maxLength": 64},
        "revision": {"type": "integer", "minimum": 1},
        "transition": {
            "type": "string",
            "enum": sorted(TRANSITIONS - {"NONE"}),
        },
        "objective": {
            "type": "string",
            "minLength": 1,
            "maxLength": 160,
        },
        "target_hypothesis_id": {
            "type": "string",
            "minLength": 1,
            "maxLength": 128,
        },
        "detour_side": {
            "type": "string",
            "enum": sorted(DETOUR_SIDES),
        },
        "success_fact_keys": {
            "type": "array",
            "items": {"type": "string", "enum": sorted(FACT_KEYS)},
            "minItems": 1,
            "maxItems": len(FACT_KEYS),
            "uniqueItems": True,
        },
        "current_focus_fact_key": {
            "type": ["string", "null"],
            "enum": [None] + sorted(FACT_KEYS),
        },
        "revision_reason": {
            "type": ["string", "null"],
            "maxLength": 160,
        },
    }
    return {
        "type": "object",
        "properties": active_properties,
        "required": sorted(MANEUVER_FIELDS),
        "additionalProperties": False,
    }


def _maneuver_schema() -> Mapping[str, object]:
    return {
        "anyOf": [
            _empty_maneuver_schema(),
            _active_maneuver_schema(),
        ]
    }


def _response_schema(
    *,
    episode_id: str,
    turn: int,
    state_version: int,
    available_actions: Sequence[str],
    target_ids: Sequence[str],
    empty_maneuver_required: bool,
) -> Mapping[str, object]:
    actions = tuple(dict.fromkeys(available_actions))
    motion = [item for item in actions if item in MOTION_ACTIONS]
    singleton = [item for item in actions if item not in MOTION_ACTIONS]
    plan_variants = []
    if motion:
        plan_variants.append(
            {
                "type": "array",
                "items": {"type": "string", "enum": motion},
                "minItems": 2,
                "maxItems": 3,
            }
        )
    if singleton:
        plan_variants.append(
            {
                "type": "array",
                "items": {"type": "string", "enum": singleton},
                "minItems": 1,
                "maxItems": 1,
            }
        )
    target_values = sorted(set(target_ids))
    properties = {
        "schema": {"type": "string", "const": DECISION_SCHEMA},
        "episode_id": {"type": "string", "const": episode_id},
        "turn": {"type": "integer", "const": turn},
        "based_on_state_version": {
            "type": "integer",
            "const": state_version,
        },
        "action": {"type": "string", "enum": list(actions)},
        "plan": {"anyOf": plan_variants},
        "reason_code": {
            "type": "string",
            "enum": sorted(REASON_CODES),
        },
        "assessment": {
            "type": "string",
            "minLength": 1,
            "maxLength": 160,
        },
        "utterance": {
            "anyOf": [
                {"type": "null"},
                {
                    "type": "string",
                    "minLength": 1,
                    "maxLength": 160,
                },
            ]
        },
        "perception_target_hypothesis_id": {"type": "null"},
        "maneuver_commitment": (
            _empty_maneuver_schema()
            if empty_maneuver_required
            else _maneuver_schema()
        ),
    }
    variants = []
    if SCAN_FRONT_ARC in actions:
        scan_properties = dict(properties)
        scan_properties["action"] = {
            "type": "string",
            "const": SCAN_FRONT_ARC,
        }
        scan_properties["perception_target_hypothesis_id"] = {
            "type": "string",
            "enum": target_values,
        }
        variants.append(
            {
                "type": "object",
                "properties": scan_properties,
                "required": sorted(scan_properties),
                "additionalProperties": False,
            }
        )
    non_scan_actions = [
        action for action in actions if action != SCAN_FRONT_ARC
    ]
    if non_scan_actions:
        non_scan_properties = dict(properties)
        non_scan_properties["action"] = {
            "type": "string",
            "enum": non_scan_actions,
        }
        variants.append(
            {
                "type": "object",
                "properties": non_scan_properties,
                "required": sorted(non_scan_properties),
                "additionalProperties": False,
            }
        )
    return {"oneOf": variants}


SYSTEM_PROMPT = """You choose goal-directed semantic actions for a harmless LEGO robot.
The host executes your exact action, vetoes it neutrally, or stops. It never
ranks, substitutes, or chooses a turn side for you. A motion decision must
author an exact two- or three-motion plan whose first entry equals action.
After every step the host takes a fresh observation and cancels the remaining
tail if safety, map generation, hazard identities, commitment, focus truth, or
localization changed. OBSERVE, SCAN_FRONT_ARC, and FINISH are singleton plans.
The host classifies whether a fresh OBSERVE changed any decision-relevant
physical fact. If latest_tool_result reports information_gain NONE, OBSERVE is
temporarily removed from available_actions; choose another listed action or
finish only when the mission facts permit it.

Infrared is qualitative reflection evidence: do not invent distance, object
identity, or a measured surface. A remembered provisional hazard remains after
the robot turns and obtains a clear reading. Before choosing the first detour
side around a target, request SCAN_FRONT_ARC for that published target while
keeping maneuver_commitment at the exact NONE sentinel. The deterministic scan
samples both sides but does not choose a route. START a route only after the
context reports completed bilateral evidence for that same target.

perception_target_hypothesis_id names what SCAN_FRONT_ARC will scan. It must
name one published target for SCAN_FRONT_ARC and must be null for every other
action.

Treat the frozen directional mission and signed progress arithmetic as facts.
Temporary regression is justified only by concrete obstacle clearance. FINISH
only when mission.completed is true. Maneuver commitments are model-owned but
must obey their lifecycle. If utterance is not null, write it only in the
episode locale specified by output_languages. Return one JSON object matching
the schema only."""


class LMStudioNavigationPlanner:
    """No-tools structured-output client; returned decisions remain untrusted."""

    def __init__(
        self,
        *,
        base_url: str,
        model: str,
        transport: Callable = _stdlib_post,
        clock: Callable[[], float] = time.monotonic,
        timeout_seconds: float = 10.0,
        allow_private_lan: bool = False,
        inference_path: str = OPENAI_V1_CHAT_COMPLETIONS_PATH,
    ):
        self.base_url = _base_url(
            base_url,
            allow_private_lan=allow_private_lan,
        )
        self.model = _model_id(model)
        if not callable(transport) or not callable(clock):
            raise LMStudioNavigationError(
                "LM Studio planner dependencies are invalid"
            )
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not 0.5 <= timeout_seconds <= 60
        ):
            raise LMStudioNavigationError(
                "LM Studio planner timeout is invalid"
            )
        if (
            not isinstance(inference_path, str)
            or inference_path not in CHAT_COMPLETIONS_PATHS
        ):
            raise LMStudioNavigationError(
                "LM Studio inference path is invalid"
            )
        self.transport = transport
        self.clock = clock
        self.timeout_seconds = float(timeout_seconds)
        self.inference_path = inference_path

    def decide(
        self,
        *,
        episode_id: str,
        turn: int,
        locale: str,
        observation: Mapping[str, object],
        mission: Mapping[str, object],
        navigation: Mapping[str, object],
        maneuver_state: Mapping[str, object],
        available_actions: Sequence[str],
        last_tool_result: Optional[Mapping[str, object]],
        validation_feedback: Optional[Mapping[str, object]] = None,
    ) -> NavigationPlannerResult:
        if locale not in UTTERANCE_LANGUAGE_BY_LOCALE:
            raise LMStudioNavigationError(
                "Navigation utterance locale is unsupported"
            )
        target_ids = sorted(
            item["hypothesis_id"]
            for item in navigation["navigation_hazard_hypotheses"]
        )
        bilateral_target_ids = {
            item["hypothesis_id"]
            for item in navigation["navigation_hazard_hypotheses"]
            if item.get("scan_completed_at_ms") is not None
            and item.get("scan_left_boundary_mdeg") is not None
            and item.get("scan_right_boundary_mdeg") is not None
            and item["scan_left_boundary_mdeg"] > 0
            and item["scan_right_boundary_mdeg"] < 0
        }
        empty_maneuver_required = (
            maneuver_state.get("active") is None
            and not bilateral_target_ids
        )
        actions = [
            item
            for item in available_actions
            if item != SCAN_FRONT_ARC or target_ids
        ]
        if not actions:
            raise LMStudioNavigationError(
                "No actions are available to the planner"
            )
        schema = _response_schema(
            episode_id=episode_id,
            turn=turn,
            state_version=observation["state_version"],
            available_actions=actions,
            target_ids=target_ids,
            empty_maneuver_required=empty_maneuver_required,
        )
        context = {
            "episode_id": episode_id,
            "turn": turn,
            "episode_locale": locale,
            "observation": observation,
            "directional_mission": mission,
            "navigation": navigation,
            "maneuver_commitment": maneuver_state,
            "available_actions": actions,
            "latest_tool_result": last_tool_result,
            "validation_feedback": validation_feedback,
            "host_ranked_or_selected_action": False,
            "output_languages": {
                "assessment": "English",
                "utterance": UTTERANCE_LANGUAGE_BY_LOCALE[locale],
            },
        }
        context_bytes = json_bytes(context)
        if len(context_bytes) > MAX_PLANNER_CONTEXT_BYTES:
            raise LMStudioNavigationError(
                "Navigation planner context exceeded its byte limit"
            )
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {
                    "role": "user",
                    "content": context_bytes.decode("utf-8"),
                },
            ],
            "response_format": {
                "type": "json_schema",
                "json_schema": {
                    "name": "physical_navigation_decision",
                    "strict": True,
                    "schema": schema,
                },
            },
            "temperature": 0,
            "reasoning_effort": "none",
            "max_tokens": 520,
            "stream": False,
            "store": False,
        }
        started = self.clock()
        try:
            raw = self.transport(
                self.base_url + self.inference_path,
                json_bytes(payload),
                {
                    "Accept": "application/json",
                    "Content-Type": "application/json; charset=utf-8",
                },
                self.timeout_seconds,
                MAX_MODEL_RESPONSE_BYTES,
            )
        except Exception as error:
            elapsed_ms = int(round((self.clock() - started) * 1000))
            raise LMStudioNavigationError(
                "LM Studio navigation request failed: {}".format(error),
                code="planner_transport_failed",
                latency_ms=max(0, elapsed_ms),
            ) from error
        elapsed_ms = int(round((self.clock() - started) * 1000))
        try:
            wire = strict_json_loads(raw, MAX_MODEL_RESPONSE_BYTES)
            if not isinstance(wire, dict):
                raise KeyError
            choices = wire["choices"]
            if not isinstance(choices, list) or len(choices) != 1:
                raise KeyError
            content = choices[0]["message"]["content"]
            if not isinstance(content, str):
                raise KeyError
        except (
            KeyError,
            TypeError,
            PhysicalNavigationContractError,
        ) as error:
            raise LMStudioNavigationError(
                "LM Studio returned an invalid navigation response",
                code="planner_response_invalid",
                latency_ms=max(0, elapsed_ms),
            ) from error
        try:
            decoded = strict_json_loads(
                content.encode("utf-8"),
                MAX_MODEL_RESPONSE_BYTES,
            )
            decision = NavigationDecision.from_mapping(
                decoded,
                episode_id=episode_id,
                turn=turn,
                state_version=observation["state_version"],
                available_actions=actions,
                published_target_ids=target_ids,
            )
        except PhysicalNavigationContractError as error:
            raise LMStudioNavigationDecisionError(
                error.code,
                str(error),
                latency_ms=elapsed_ms,
            ) from error
        served_model = wire.get("model")
        if served_model is not None and not isinstance(served_model, str):
            served_model = None
        usage = wire.get("usage")
        if usage is not None and not isinstance(usage, dict):
            usage = None
        stats = wire.get("stats")
        if stats is not None and not isinstance(stats, dict):
            stats = None
        return NavigationPlannerResult(
            decision=decision,
            latency_ms=elapsed_ms,
            served_model=served_model,
            usage=usage,
            stats=stats,
        )
