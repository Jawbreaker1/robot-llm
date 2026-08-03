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
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_KEYS,
    FIELDS as MANEUVER_FIELDS,
    TRANSITIONS,
)
from .physical_navigation_contract import (
    DECISION_SCHEMA,
    MOTION_ACTIONS,
    NavigationDecision,
    OBSERVE,
    PhysicalNavigationContractError,
    REASON_CODES,
    SCAN_FRONT_ARC,
    json_bytes,
    strict_json_loads,
)
from .physical_navigation_planner_context import (
    PhysicalPlannerContextError,
    project_navigation_context,
)


MAX_MODEL_RESPONSE_BYTES = 64 * 1024
TARGET_PLANNER_CONTEXT_BYTES = 56 * 1024
MAX_PLANNER_CONTEXT_BYTES = 64 * 1024
PLANNER_CONTEXT_WINDOW_TOKENS = 32_768
PLANNER_MAX_OUTPUT_TOKENS = 520
PLANNER_CONTEXT_HEADROOM_TOKENS = 1_024
PLANNER_TOKEN_ESTIMATE_FIXED_OVERHEAD = 512
PLANNER_REQUEST_WRAPPER_RESERVE_BYTES = 2_048
# Gemma JSON is denser in practice.  Eleven bytes per four tokens is used only
# as a deliberately conservative deterministic admission bound; the runtime
# does not depend on any model tokenizer package.
PLANNER_ESTIMATE_BYTES_NUMERATOR = 11
PLANNER_ESTIMATE_TOKENS_DENOMINATOR = 4
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
UTTERANCE_PERSONA_BY_LOCALE = {
    "sv": (
        "Du är en uttrycksfull men harmlös gammal grinig LEGO-robot. "
        "Kommentera kort och spontant; muttra, gnäll och klaga när det "
        "passar. Vid hinder, motorstrul, upprepade försök eller "
        "misslyckanden är naturliga svenska svordomar, från milda till "
        "kraftiga, välkomna. Svär inte i varje replik, hota eller angrip "
        "aldrig personer och återanvänd inte en nyligen uttalad "
        "formulering. Personligheten får bara påverka utterance, aldrig "
        "det strukturerade beslutet."
    ),
    "en": (
        "You are an expressive but harmless grumpy old LEGO robot. Keep "
        "comments brief and spontaneous; grumble, moan, and complain when "
        "it fits. Obstacles, motor trouble, repeated attempts, or failure "
        "may warrant natural English profanity, from mild to strong. Do "
        "not swear in every utterance, threaten or abuse people, or reuse "
        "a recently spoken formulation. Personality may affect utterance "
        "only, never the structured decision."
    ),
}
MAX_RECENT_COMMITTED_UTTERANCES = 6
MAX_UTTERANCE_CHARACTERS = 160


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
    context_byte_count: int = 0
    prompt_tokens: Optional[int] = None
    completion_tokens: Optional[int] = None
    total_tokens: Optional[int] = None
    estimated_prompt_tokens: int = 0
    prompt_token_budget: int = 0
    accounted_prompt_bytes: int = 0
    context_target_byte_count: int = 0
    context_hard_byte_count: int = 0


def _usage_token_count(usage, field: str) -> Optional[int]:
    if not isinstance(usage, Mapping):
        return None
    value = usage.get(field)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _conservative_prompt_token_estimate(accounted_bytes: int) -> int:
    if (
        isinstance(accounted_bytes, bool)
        or not isinstance(accounted_bytes, int)
        or accounted_bytes < 0
    ):
        raise LMStudioNavigationError("Planner prompt size is invalid")
    numerator = (
        accounted_bytes * PLANNER_ESTIMATE_TOKENS_DENOMINATOR
    )
    estimated = (
        numerator + PLANNER_ESTIMATE_BYTES_NUMERATOR - 1
    ) // PLANNER_ESTIMATE_BYTES_NUMERATOR
    return estimated + PLANNER_TOKEN_ESTIMATE_FIXED_OVERHEAD


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


def _start_maneuver_schema(
    target_hypothesis_ids: Sequence[str],
) -> Mapping[str, object]:
    """Constrain only the authorization carrier; the model owns its route."""

    schema = _active_maneuver_schema()
    properties = dict(schema["properties"])
    properties.update(
        {
            "revision": {"type": "integer", "const": 1},
            "transition": {"type": "string", "const": "START"},
            "target_hypothesis_id": {
                "type": "string",
                "enum": sorted(set(target_hypothesis_ids)),
            },
            "success_fact_keys": {
                "type": "array",
                "const": sorted(FACT_KEYS),
            },
            "current_focus_fact_key": {
                "type": "string",
                "const": FACT_GOAL_CORRIDOR_CLEAR,
            },
            "revision_reason": {"type": "null"},
        }
    )
    return {**schema, "properties": properties}


def _response_schema(
    *,
    episode_id: str,
    turn: int,
    state_version: int,
    available_actions: Sequence[str],
    target_ids: Sequence[str],
    empty_maneuver_required: bool,
    start_maneuver_target_ids: Sequence[str] = (),
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
    if start_maneuver_target_ids:
        if OBSERVE not in actions:
            raise LMStudioNavigationError(
                "Route authorization requires OBSERVE"
            )
        start_properties = dict(properties)
        start_properties.update(
            {
                "action": {"type": "string", "const": OBSERVE},
                "plan": {
                    "type": "array",
                    "items": {"type": "string", "const": OBSERVE},
                    "minItems": 1,
                    "maxItems": 1,
                },
                "maneuver_commitment": _start_maneuver_schema(
                    start_maneuver_target_ids
                ),
            }
        )
        return {
            "oneOf": [
                {
                    "type": "object",
                    "properties": start_properties,
                    "required": sorted(start_properties),
                    "additionalProperties": False,
                }
            ]
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
temporarily removed from available_actions unless it must carry a ready route
authorization; choose another listed action or finish only when the mission
facts permit it.

navigation.action_feasibility publishes deterministic swept-body facts for
every motion and the active scan. An action that does not fit the current
robot footprint is removed from available_actions before you plan; choose
among the remaining semantic actions rather than retrying an unavailable one.

navigation.experience_ledger is bounded factual episode history, not an action
ranking. Use attempt_identity and attempt_relation to distinguish an exact
typed retry on an unchanged physical evidence basis from the same action after
verified pose, sensor, hazard, or scan-evidence change. Scans of different
target_hypothesis_id values are different attempts. Do not repeat an unchanged
attempt unless another published fact provides a concrete new reason.
current_basis_action_rollups retain typed counts and latest outcomes after
detailed entries age out; explicit bucket/attempt omission counters disclose
any bounded distribution detail. navigation.planner_context_projection reports
exact detail retention and omission counts. Compact current-pose route scan
records preserve IDs, pose, boundaries, relation, and aggregate ray facts;
compact scan_evidence_summary values are facts, not inferred object identity or
host action recommendations.

Infrared is qualitative reflection evidence: do not invent distance, object
identity, or a measured surface. A remembered provisional hazard remains after
the robot turns and obtains a clear reading. Scan bearings are body-relative:
positive is the robot's physical left and negative is its physical right.
Before choosing the first detour side around a target, request SCAN_FRONT_ARC
for that published target while keeping maneuver_commitment at the exact NONE
sentinel. The deterministic scan samples both sides but does not choose a
route. START a route only when route_commitment_ready is true for that target
at the current verified pose. route_commitment_evidence_strength distinguishes
bilateral boundaries from explicitly best-effort unilateral, blocked-arc, or
all-clear-arc evidence; use weaker evidence cautiously, but keep making
physical progress instead of abandoning the maneuver state. ALL_CLEAR_ARC
means only that the sampled front arc was clear at the current verified pose;
it does not erase the remembered obstacle or prove the whole route. Historical
rays from other poses still inform collision geometry but do not authorize a
new route.

Authorize a ready target and detour_side with START on singleton OBSERVE. Do
not START with a turn or other motion. The host then builds the geometric route
for that model-chosen side and follows each uniquely determined waypoint pulse
without another model call. Changing target or side with REVISE, or ABANDONing
the route, likewise uses singleton OBSERVE so the strategic change takes effect
before any later physical motion.

The host persists an active maneuver commitment. When its strategy and focus
do not change, return the exact NONE sentinel instead of repeating the active
commitment as CONTINUE. NONE also preserves the active commitment while
scanning its current target. CONTINUE remains available for an intentional
focus change that preserves the same commitment revision, target, and side.
When every maneuver success fact is true but the directional mission continues,
use COMPLETE with singleton OBSERVE to retire only the maneuver. COMPLETE may
accompany FINISH when the complete mission is ready to finish. Never combine
COMPLETE with physical motion.

When navigation.local_detour_route is ACTIVE, it is the persistent geometric
route for your chosen target and detour_side. Follow its active waypoint in
order; navigation.local_detour_guidance explains the current heading error,
distance, and why motion choices were filtered. The route first establishes
lateral clearance, passes the complete remembered object envelope, merges
back onto the frozen goal axis, and resumes the original goal heading. It is
rebuilt from verified pose when map geometry changes. Do not restart the
maneuver merely because a waypoint advanced. The route and its active waypoint
are the authoritative local execution state; maneuver focus remains strategic
context rather than a second waypoint implementation.

perception_target_hypothesis_id names what SCAN_FRONT_ARC will scan. It must
name one published target for SCAN_FRONT_ARC and must be null for every other
action.

Treat the frozen directional mission and signed progress arithmetic as facts.
Temporary regression is justified only by concrete obstacle clearance. FINISH
only when mission.completed is true. Maneuver commitments are model-owned but
must obey their lifecycle.

Utterance is optional, motion-free commentary and never control data. Follow
the host-authored utterance_guidance for its persona and language, but never
let style change action, plan, assessment, reason_code, perception target, or
maneuver commitment. recent_committed_utterances contains speech that was
actually accepted for this episode: do not repeat or closely paraphrase it.
Prefer null when there is nothing fresh to say. If utterance is not null,
write it only in the episode locale specified by output_languages. Return one
JSON object matching the schema only."""


def _recent_committed_utterances(
    values: Sequence[str],
) -> list[str]:
    if (
        isinstance(values, (str, bytes))
        or not isinstance(values, Sequence)
        or len(values) > MAX_RECENT_COMMITTED_UTTERANCES
    ):
        raise LMStudioNavigationError(
            "Recent committed navigation utterances are invalid"
        )
    checked = []
    for value in values:
        if (
            not isinstance(value, str)
            or not value
            or value != value.strip()
            or len(value) > MAX_UTTERANCE_CHARACTERS
            or any(
                ord(character) < 32 and character not in "\n\r\t"
                for character in value
            )
        ):
            raise LMStudioNavigationError(
                "Recent committed navigation utterances are invalid"
            )
        checked.append(value)
    return checked


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
        recent_committed_utterances: Sequence[str] = (),
    ) -> NavigationPlannerResult:
        if locale not in UTTERANCE_LANGUAGE_BY_LOCALE:
            raise LMStudioNavigationError(
                "Navigation utterance locale is unsupported"
            )
        target_ids = sorted(
            item["hypothesis_id"]
            for item in navigation["navigation_hazard_hypotheses"]
        )
        scan_target_ids = navigation.get(
            "scan_eligible_target_hypothesis_ids",
            target_ids,
        )
        if (
            not isinstance(scan_target_ids, list)
            or len(set(scan_target_ids)) != len(scan_target_ids)
            or any(item not in target_ids for item in scan_target_ids)
        ):
            raise LMStudioNavigationError(
                "Scan target eligibility is invalid"
            )
        scan_target_ids = sorted(scan_target_ids)
        route_ready_target_ids = {
            item["hypothesis_id"]
            for item in navigation["navigation_hazard_hypotheses"]
            if item.get(
                "route_commitment_ready",
                item.get(
                    "bilateral_scan_complete",
                    item.get("scan_completed_at_ms") is not None
                    and item.get("scan_left_boundary_mdeg") is not None
                    and item.get("scan_right_boundary_mdeg") is not None
                    and item["scan_left_boundary_mdeg"] > 0
                    and item["scan_right_boundary_mdeg"] < 0,
                ),
            )
            is True
        }
        empty_maneuver_required = (
            maneuver_state.get("active") is None
            and not route_ready_target_ids
        )
        start_maneuver_target_ids = navigation.get(
            "route_authorization_required_target_hypothesis_ids",
            [],
        )
        if (
            not isinstance(start_maneuver_target_ids, list)
            or len(set(start_maneuver_target_ids))
            != len(start_maneuver_target_ids)
            or any(
                item not in route_ready_target_ids
                for item in start_maneuver_target_ids
            )
            or (
                start_maneuver_target_ids
                and maneuver_state.get("active") is not None
            )
        ):
            raise LMStudioNavigationError(
                "Route authorization targets are invalid"
            )
        start_maneuver_target_ids = sorted(start_maneuver_target_ids)
        actions = [
            item
            for item in available_actions
            if item != SCAN_FRONT_ARC or scan_target_ids
        ]
        if not actions:
            raise LMStudioNavigationError(
                "No actions are available to the planner"
            )
        recent_utterances = _recent_committed_utterances(
            recent_committed_utterances
        )
        schema = _response_schema(
            episode_id=episode_id,
            turn=turn,
            state_version=observation["state_version"],
            available_actions=actions,
            target_ids=scan_target_ids,
            empty_maneuver_required=empty_maneuver_required,
            start_maneuver_target_ids=start_maneuver_target_ids,
        )
        prompt_token_budget = (
            PLANNER_CONTEXT_WINDOW_TOKENS
            - PLANNER_MAX_OUTPUT_TOKENS
            - PLANNER_CONTEXT_HEADROOM_TOKENS
        )
        fixed_accounted_bytes = (
            len(SYSTEM_PROMPT.encode("utf-8"))
            + len(json_bytes(schema))
            + PLANNER_REQUEST_WRAPPER_RESERVE_BYTES
        )
        maximum_accounted_bytes = (
            (
                prompt_token_budget
                - PLANNER_TOKEN_ESTIMATE_FIXED_OVERHEAD
            )
            * PLANNER_ESTIMATE_BYTES_NUMERATOR
        ) // PLANNER_ESTIMATE_TOKENS_DENOMINATOR
        token_safe_context_bytes = (
            maximum_accounted_bytes - fixed_accounted_bytes
        )
        hard_context_bytes = min(
            MAX_PLANNER_CONTEXT_BYTES,
            token_safe_context_bytes,
        )
        target_context_bytes = min(
            TARGET_PLANNER_CONTEXT_BYTES,
            hard_context_bytes,
        )
        if target_context_bytes <= 0:
            raise LMStudioNavigationError(
                "Planner schema leaves no safe model context budget"
            )
        context = {
            "episode_id": episode_id,
            "turn": turn,
            "episode_locale": locale,
            "observation": observation,
            "directional_mission": mission,
            "navigation": {},
            "maneuver_commitment": maneuver_state,
            "available_actions": actions,
            "latest_tool_result": last_tool_result,
            "validation_feedback": validation_feedback,
            "host_ranked_or_selected_action": False,
            "output_languages": {
                "assessment": "English",
                "utterance": UTTERANCE_LANGUAGE_BY_LOCALE[locale],
            },
            "utterance_guidance": {
                "persona": UTTERANCE_PERSONA_BY_LOCALE[locale],
                "recent_committed_utterances": recent_utterances,
            },
        }
        empty_navigation_context_bytes = len(json_bytes(context))
        target_navigation_bytes = (
            target_context_bytes
            - empty_navigation_context_bytes
            + len(json_bytes({}))
        )
        hard_navigation_bytes = (
            hard_context_bytes
            - empty_navigation_context_bytes
            + len(json_bytes({}))
        )
        if target_navigation_bytes <= 0:
            raise LMStudioNavigationError(
                "Planner turn facts leave no safe navigation budget"
            )
        try:
            context["navigation"] = project_navigation_context(
                navigation,
                maneuver_state=maneuver_state,
                last_tool_result=last_tool_result,
                target_budget_bytes=target_navigation_bytes,
                hard_budget_bytes=hard_navigation_bytes,
            )
        except PhysicalPlannerContextError as error:
            raise LMStudioNavigationError(
                "Navigation planner context projection failed: {}".format(
                    error
                )
            ) from error
        context_bytes = json_bytes(context)
        accounted_prompt_bytes = (
            len(context_bytes) + fixed_accounted_bytes
        )
        estimated_prompt_tokens = _conservative_prompt_token_estimate(
            accounted_prompt_bytes
        )
        if (
            len(context_bytes) > hard_context_bytes
            or estimated_prompt_tokens > prompt_token_budget
        ):
            raise LMStudioNavigationError(
                "Navigation planner context exceeded its 32k-safe limit"
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
            "max_tokens": PLANNER_MAX_OUTPUT_TOKENS,
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
            context_byte_count=len(context_bytes),
            prompt_tokens=_usage_token_count(usage, "prompt_tokens"),
            completion_tokens=_usage_token_count(
                usage,
                "completion_tokens",
            ),
            total_tokens=_usage_token_count(usage, "total_tokens"),
            estimated_prompt_tokens=estimated_prompt_tokens,
            prompt_token_budget=prompt_token_budget,
            accounted_prompt_bytes=accounted_prompt_bytes,
            context_target_byte_count=target_context_bytes,
            context_hard_byte_count=hard_context_bytes,
        )
