"""Bounded, read-only research loop with strictly typed model decisions.

This module intentionally has no dependency on the robot execution stack.
Planners can request an exact, registered information tool or return text,
but neither planner output nor external evidence can carry physical authority.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import secrets
from types import MappingProxyType
from typing import Callable, Mapping, Optional, Tuple
from urllib.parse import parse_qsl, urlsplit

from .research import (
    ResearchError,
    WeatherResearchRequest,
    WeatherResearchResult,
    WeatherTool,
)


DECISION_SCHEMA = "research-decision/v1"
MAX_DECISION_BYTES = 16 * 1024
MAX_EVIDENCE_BYTES = 64 * 1024

ANSWERED = "ANSWERED"
CLARIFICATION_REQUIRED = "CLARIFICATION_REQUIRED"
PLANNER_ABORTED = "PLANNER_ABORTED"
PLANNER_FAILED = "PLANNER_FAILED"
TOOL_FAILED = "TOOL_FAILED"
BUDGET_EXHAUSTED = "BUDGET_EXHAUSTED"
INVALID_RESEARCH_GOAL = "INVALID_RESEARCH_GOAL"


class ResearchLoopError(ValueError):
    """A typed, safely reportable research-loop failure."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _identifier(name: str, value: str, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise ResearchLoopError(
            "invalid_identifier",
            "{} is invalid".format(name),
        )
    return value


def _text(name: str, value: str, maximum: int) -> str:
    if (
        not isinstance(value, str)
        or not value.strip()
        or len(value) > maximum
        or any(
            ord(character) < 32 and character not in "\n\r\t"
            for character in value
        )
    ):
        raise ResearchLoopError(
            "invalid_text",
            "{} is invalid".format(name),
        )
    return value


def _integer(name: str, value: int, minimum: int, maximum: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise ResearchLoopError(
            "invalid_integer",
            "{} is invalid".format(name),
        )
    return value


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


def _freeze_json(value, _depth=0, _counter=None):
    if _counter is None:
        _counter = [0]
    _counter[0] += 1
    if _depth > 16 or _counter[0] > 4_096:
        raise ResearchLoopError(
            "json_complexity_limit",
            "JSON value exceeded the complexity limit",
        )
    if isinstance(value, dict):
        frozen = {}
        for key, item in value.items():
            if not isinstance(key, str) or len(key) > 1_000:
                raise ResearchLoopError(
                    "invalid_json_value",
                    "JSON object key is invalid",
                )
            frozen[key] = _freeze_json(
                item,
                _depth + 1,
                _counter,
            )
        return MappingProxyType(frozen)
    if isinstance(value, list):
        return tuple(
            _freeze_json(item, _depth + 1, _counter)
            for item in value
        )
    if isinstance(value, str) and len(value) > MAX_EVIDENCE_BYTES:
        raise ResearchLoopError(
            "json_string_limit",
            "JSON string exceeded the size limit",
        )
    if value is None or type(value) in (bool, int, str):
        return value
    if isinstance(value, float) and math.isfinite(value):
        return value
    raise ResearchLoopError(
        "invalid_json_value",
        "JSON value is invalid",
    )


def _thaw_json(value):
    if isinstance(value, Mapping):
        return {
            key: _thaw_json(item)
            for key, item in value.items()
        }
    if isinstance(value, tuple):
        return [_thaw_json(item) for item in value]
    return value


@dataclass(frozen=True)
class ResearchGoal:
    turn_id: str
    user_query: str
    require_evidence: bool = False

    def __post_init__(self) -> None:
        _identifier("turn_id", self.turn_id)
        _text("user_query", self.user_query, 4_000)
        if type(self.require_evidence) is not bool:
            raise ResearchLoopError(
                "invalid_require_evidence",
                "require_evidence must be a boolean",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "turn_id": self.turn_id,
            "user_query": self.user_query,
            "require_evidence": self.require_evidence,
        }


@dataclass(frozen=True)
class ToolCallIntent:
    name: str
    arguments: Mapping[str, object]

    def __post_init__(self) -> None:
        _identifier("tool name", self.name)
        if not isinstance(self.arguments, Mapping):
            raise ResearchLoopError(
                "invalid_tool_arguments",
                "Tool arguments must be an object",
            )
        object.__setattr__(
            self,
            "arguments",
            _freeze_json(dict(self.arguments)),
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "name": self.name,
            "arguments": _thaw_json(self.arguments),
        }


@dataclass(frozen=True)
class AnswerIntent:
    text: str
    evidence_ids: Tuple[str, ...]

    def __post_init__(self) -> None:
        _text("answer text", self.text, 4_000)
        if (
            not isinstance(self.evidence_ids, tuple)
            or len(self.evidence_ids) > 32
        ):
            raise ResearchLoopError(
                "invalid_evidence_ids",
                "Answer evidence IDs are invalid",
            )
        seen = set()
        for evidence_id in self.evidence_ids:
            _identifier("evidence_id", evidence_id)
            if evidence_id in seen:
                raise ResearchLoopError(
                    "duplicate_evidence_id",
                    "Answer repeated an evidence ID",
                )
            seen.add(evidence_id)

    def to_dict(self) -> Mapping[str, object]:
        return {
            "text": self.text,
            "evidence_ids": list(self.evidence_ids),
        }


@dataclass(frozen=True)
class ResearchDecision:
    proposal_id: str
    turn_id: str
    based_on_context_version: int
    decision: str
    tool: Optional[ToolCallIntent] = None
    answer: Optional[AnswerIntent] = None
    question: Optional[str] = None
    abort_code: Optional[str] = None


def decode_research_decision(raw: bytes) -> ResearchDecision:
    """Decode one exact decision without interpreting natural language."""

    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > MAX_DECISION_BYTES
    ):
        raise ResearchLoopError(
            "invalid_decision_body",
            "Planner decision body is invalid",
        )
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except RecursionError:
        raise ResearchLoopError(
            "json_complexity_limit",
            "Planner decision exceeded the JSON complexity limit",
        ) from None
    except (UnicodeDecodeError, TypeError, ValueError):
        raise ResearchLoopError(
            "invalid_decision_json",
            "Planner returned invalid JSON",
        ) from None
    if not isinstance(value, dict):
        raise ResearchLoopError(
            "invalid_decision_shape",
            "Planner decision must be an object",
        )

    common = {
        "schema",
        "proposal_id",
        "turn_id",
        "based_on_context_version",
        "decision",
    }
    decision = value.get("decision")
    tool = None
    answer = None
    question = None
    abort_code = None

    if decision == "CALL_TOOL":
        if set(value) != common | {"tool"}:
            raise ResearchLoopError(
                "invalid_decision_fields",
                "CALL_TOOL decision fields are invalid",
            )
        tool_value = value["tool"]
        if (
            not isinstance(tool_value, dict)
            or set(tool_value) != {"name", "arguments"}
            or not isinstance(tool_value.get("arguments"), dict)
        ):
            raise ResearchLoopError(
                "invalid_tool",
                "Tool proposal is invalid",
            )
        tool = ToolCallIntent(
            name=_identifier("tool name", tool_value["name"]),
            arguments=tool_value["arguments"],
        )
    elif decision == "ANSWER":
        if set(value) != common | {"answer"}:
            raise ResearchLoopError(
                "invalid_decision_fields",
                "ANSWER decision fields are invalid",
            )
        answer_value = value["answer"]
        if (
            not isinstance(answer_value, dict)
            or set(answer_value) != {"text", "evidence_ids"}
            or not isinstance(answer_value.get("evidence_ids"), list)
        ):
            raise ResearchLoopError(
                "invalid_answer",
                "Answer proposal is invalid",
            )
        answer = AnswerIntent(
            text=_text("answer text", answer_value["text"], 4_000),
            evidence_ids=tuple(answer_value["evidence_ids"]),
        )
    elif decision == "CLARIFY":
        if set(value) != common | {"question"}:
            raise ResearchLoopError(
                "invalid_decision_fields",
                "CLARIFY decision fields are invalid",
            )
        question = _text("question", value["question"], 1_000)
    elif decision == "ABORT":
        if set(value) != common | {"abort_code"}:
            raise ResearchLoopError(
                "invalid_decision_fields",
                "ABORT decision fields are invalid",
            )
        abort_code = _identifier("abort_code", value["abort_code"], 64)
    else:
        raise ResearchLoopError(
            "invalid_decision",
            "Decision must be CALL_TOOL, ANSWER, CLARIFY or ABORT",
        )

    if value.get("schema") != DECISION_SCHEMA:
        raise ResearchLoopError(
            "invalid_decision_schema",
            "Planner decision schema is not supported",
        )
    return ResearchDecision(
        proposal_id=_identifier("proposal_id", value["proposal_id"]),
        turn_id=_identifier("turn_id", value["turn_id"]),
        based_on_context_version=_integer(
            "based_on_context_version",
            value["based_on_context_version"],
            1,
            2**63 - 1,
        ),
        decision=decision,
        tool=tool,
        answer=answer,
        question=question,
        abort_code=abort_code,
    )


@dataclass(frozen=True)
class ResearchEvidenceEnvelope:
    evidence_id: str
    tool_call_id: str
    tool_name: str
    produced_from_context_version: int
    received_at_monotonic_ms: int
    valid_until_monotonic_ms: int
    payload: Mapping[str, object]
    trust: str = "untrusted_external_data"

    def __post_init__(self) -> None:
        _identifier("evidence_id", self.evidence_id)
        _identifier("tool_call_id", self.tool_call_id)
        _identifier("tool_name", self.tool_name)
        _integer(
            "produced_from_context_version",
            self.produced_from_context_version,
            1,
            2**63 - 1,
        )
        received = _integer(
            "received_at_monotonic_ms",
            self.received_at_monotonic_ms,
            0,
            2**63 - 1,
        )
        valid_until = _integer(
            "valid_until_monotonic_ms",
            self.valid_until_monotonic_ms,
            0,
            2**63 - 1,
        )
        if valid_until <= received:
            raise ResearchLoopError(
                "invalid_evidence_lifetime",
                "Evidence lifetime is invalid",
            )
        if self.trust != "untrusted_external_data":
            raise ResearchLoopError(
                "invalid_evidence_trust",
                "Evidence trust marker is invalid",
            )
        if not isinstance(self.payload, Mapping):
            raise ResearchLoopError(
                "invalid_evidence_payload",
                "Evidence payload must be an object",
            )
        object.__setattr__(
            self,
            "payload",
            _freeze_json(dict(self.payload)),
        )

    def fresh(self, now_ms: int) -> bool:
        return (
            type(now_ms) is int
            and now_ms >= self.received_at_monotonic_ms
            and now_ms < self.valid_until_monotonic_ms
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "evidence_id": self.evidence_id,
            "tool_call_id": self.tool_call_id,
            "tool_name": self.tool_name,
            "produced_from_context_version": (
                self.produced_from_context_version
            ),
            "received_at_monotonic_ms": self.received_at_monotonic_ms,
            "valid_until_monotonic_ms": self.valid_until_monotonic_ms,
            "trust": self.trust,
            "payload": _thaw_json(self.payload),
        }


@dataclass(frozen=True)
class ResearchFeedback:
    code: str
    context_version: int
    tool_call_id: Optional[str] = None

    def __post_init__(self) -> None:
        _identifier("feedback code", self.code, 64)
        _integer("context_version", self.context_version, 1, 2**63 - 1)
        if self.tool_call_id is not None:
            _identifier("tool_call_id", self.tool_call_id)

    def to_dict(self) -> Mapping[str, object]:
        return {
            "code": self.code,
            "context_version": self.context_version,
            "tool_call_id": self.tool_call_id,
        }


@dataclass(frozen=True)
class ResearchPlanningContext:
    turn_id: str
    user_query: str
    require_evidence: bool
    available_tools: Tuple[str, ...]
    used_proposal_ids: Tuple[str, ...]
    context_version: int
    evidence: Tuple[ResearchEvidenceEnvelope, ...]
    previous_feedback: Optional[ResearchFeedback]
    planner_turn: int
    remaining_tool_calls: int
    remaining_replans: int
    remaining_elapsed_ms: int
    planner_timeout_ms: int

    def __post_init__(self) -> None:
        _identifier("turn_id", self.turn_id)
        _text("user_query", self.user_query, 4_000)
        if type(self.require_evidence) is not bool:
            raise ResearchLoopError(
                "invalid_require_evidence",
                "require_evidence must be a boolean",
            )
        if (
            not isinstance(self.available_tools, tuple)
            or not self.available_tools
            or len(set(self.available_tools))
            != len(self.available_tools)
        ):
            raise ResearchLoopError(
                "invalid_available_tools",
                "Available tools are invalid",
            )
        for name in self.available_tools:
            _identifier("available tool", name)
        if (
            not isinstance(self.used_proposal_ids, tuple)
            or len(set(self.used_proposal_ids))
            != len(self.used_proposal_ids)
        ):
            raise ResearchLoopError(
                "invalid_used_proposal_ids",
                "Used proposal IDs are invalid",
            )
        for proposal_id in self.used_proposal_ids:
            _identifier("used proposal_id", proposal_id)
        _integer("context_version", self.context_version, 1, 2**63 - 1)
        if (
            not isinstance(self.evidence, tuple)
            or any(
                not isinstance(item, ResearchEvidenceEnvelope)
                for item in self.evidence
            )
        ):
            raise ResearchLoopError(
                "invalid_context_evidence",
                "Planning context evidence is invalid",
            )
        if (
            self.previous_feedback is not None
            and not isinstance(self.previous_feedback, ResearchFeedback)
        ):
            raise ResearchLoopError(
                "invalid_context_feedback",
                "Planning context feedback is invalid",
            )
        _integer("planner_turn", self.planner_turn, 1, 10_000)
        _integer(
            "remaining_tool_calls",
            self.remaining_tool_calls,
            0,
            10_000,
        )
        _integer(
            "remaining_replans",
            self.remaining_replans,
            0,
            10_000,
        )
        _integer(
            "remaining_elapsed_ms",
            self.remaining_elapsed_ms,
            0,
            2**63 - 1,
        )
        timeout_ms = _integer(
            "planner_timeout_ms",
            self.planner_timeout_ms,
            1,
            300_000,
        )
        if timeout_ms > self.remaining_elapsed_ms:
            raise ResearchLoopError(
                "invalid_planner_timeout",
                "Planner timeout exceeds remaining episode time",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "turn_id": self.turn_id,
            "user_query": self.user_query,
            "require_evidence": self.require_evidence,
            "available_tools": list(self.available_tools),
            "used_proposal_ids": list(self.used_proposal_ids),
            "context_version": self.context_version,
            "evidence": [item.to_dict() for item in self.evidence],
            "previous_feedback": (
                None
                if self.previous_feedback is None
                else self.previous_feedback.to_dict()
            ),
            "planner_turn": self.planner_turn,
            "remaining_tool_calls": self.remaining_tool_calls,
            "remaining_replans": self.remaining_replans,
            "remaining_elapsed_ms": self.remaining_elapsed_ms,
            "planner_timeout_ms": self.planner_timeout_ms,
        }


@dataclass(frozen=True)
class ResearchLimits:
    max_elapsed_ms: int = 10_000
    max_planner_latency_ms: int = 3_000
    max_planner_turns: int = 6
    max_tool_calls: int = 2
    max_replans: int = 4
    tool_request_ttl_ms: int = 4_000
    evidence_ttl_ms: int = 10 * 60 * 1_000
    max_weather_observation_skew_ms: int = 2 * 60 * 60 * 1_000

    def __post_init__(self) -> None:
        _integer("max_elapsed_ms", self.max_elapsed_ms, 1, 300_000)
        _integer(
            "max_planner_latency_ms",
            self.max_planner_latency_ms,
            1,
            self.max_elapsed_ms,
        )
        _integer("max_planner_turns", self.max_planner_turns, 1, 100)
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


@dataclass(frozen=True)
class ResearchEpisodeResult:
    turn_id: str
    completed: bool
    termination: str
    answer_text: Optional[str]
    clarification_question: Optional[str]
    citation_ids: Tuple[str, ...]
    planner_turns: int
    tool_calls: int
    replans: int
    final_context_version: int
    evidence: Tuple[ResearchEvidenceEnvelope, ...]
    trace: Tuple[str, ...]

    def to_dict(self) -> Mapping[str, object]:
        return {
            "turn_id": self.turn_id,
            "completed": self.completed,
            "termination": self.termination,
            "answer_text": self.answer_text,
            "clarification_question": self.clarification_question,
            "citation_ids": list(self.citation_ids),
            "planner_turns": self.planner_turns,
            "tool_calls": self.tool_calls,
            "replans": self.replans,
            "final_context_version": self.final_context_version,
            "evidence": [item.to_dict() for item in self.evidence],
            "trace": list(self.trace),
        }


ResearchPlanner = Callable[[ResearchPlanningContext], bytes]
Clock = Callable[[], int]
IDFactory = Callable[[], str]


class ResearchToolRegistry:
    """Exact-name registry of read-only research tools."""

    WEATHER_CURRENT = "weather.current"

    def __init__(self, weather_tool: WeatherTool):
        if (
            weather_tool is None
            or not callable(getattr(weather_tool, "current", None))
        ):
            raise ResearchLoopError(
                "invalid_weather_tool",
                "Weather tool is invalid",
            )
        self._weather_tool = weather_tool
        self._names = (self.WEATHER_CURRENT,)

    @property
    def names(self) -> Tuple[str, ...]:
        return self._names

    def validate(
        self,
        name: str,
        arguments: Mapping[str, object],
    ) -> None:
        if name != self.WEATHER_CURRENT:
            raise ResearchLoopError(
                "unknown_tool",
                "Research tool is not registered",
            )
        if (
            not isinstance(arguments, Mapping)
            or set(arguments) != {"location_query"}
        ):
            raise ResearchLoopError(
                "invalid_tool_arguments",
                "Weather arguments are invalid",
            )
        location_query = arguments["location_query"]
        if (
            not isinstance(location_query, str)
            or location_query != location_query.strip()
            or not 2 <= len(location_query) <= 200
            or not location_query.isprintable()
        ):
            raise ResearchLoopError(
                "invalid_tool_arguments",
                "Weather location query is invalid",
            )

    def execute(
        self,
        name: str,
        arguments: Mapping[str, object],
        request_id: str,
        issued_at_ms: int,
        valid_until_ms: int,
    ) -> WeatherResearchResult:
        self.validate(name, arguments)
        location_query = arguments["location_query"]
        request = WeatherResearchRequest(
            request_id=request_id,
            location_query=location_query,
            issued_at_monotonic_ms=issued_at_ms,
            valid_until_monotonic_ms=valid_until_ms,
        )
        result = self._weather_tool.current(request)
        if not isinstance(result, WeatherResearchResult):
            raise ResearchLoopError(
                "invalid_tool_result",
                "Weather tool returned an invalid result",
            )
        return result


class ResearchLoop:
    """Bounded plan-research-answer loop with no physical capabilities."""

    def __init__(
        self,
        planner: ResearchPlanner,
        tools: ResearchToolRegistry,
        clock_ms: Clock,
        limits: ResearchLimits = ResearchLimits(),
        id_factory: IDFactory = lambda: secrets.token_hex(8),
    ):
        if not callable(planner) or not callable(clock_ms):
            raise ResearchLoopError(
                "invalid_dependency",
                "Planner or clock is invalid",
            )
        if not isinstance(tools, ResearchToolRegistry):
            raise ResearchLoopError(
                "invalid_tool_registry",
                "Tool registry is invalid",
            )
        if not isinstance(limits, ResearchLimits) or not callable(id_factory):
            raise ResearchLoopError(
                "invalid_dependency",
                "Limits or ID factory is invalid",
            )
        self._planner = planner
        self._tools = tools
        self._clock_ms = clock_ms
        self._limits = limits
        self._id_factory = id_factory

    def _now_ms(self) -> int:
        return _integer("clock_ms", self._clock_ms(), 0, 2**63 - 1)

    def _finish(
        self,
        goal: ResearchGoal,
        termination: str,
        context_version: int,
        evidence,
        counters,
        trace,
        answer_text=None,
        clarification_question=None,
        citation_ids=(),
    ) -> ResearchEpisodeResult:
        completed = termination == ANSWERED
        trace.append(
            "ANSWERED"
            if completed
            else "CLARIFICATION_REQUIRED"
            if termination == CLARIFICATION_REQUIRED
            else "ABORTED"
            if termination == PLANNER_ABORTED
            else "FAILED"
        )
        return ResearchEpisodeResult(
            turn_id=goal.turn_id,
            completed=completed,
            termination=termination,
            answer_text=answer_text,
            clarification_question=clarification_question,
            citation_ids=tuple(citation_ids),
            planner_turns=counters["planner_turns"],
            tool_calls=counters["tool_calls"],
            replans=counters["replans"],
            final_context_version=context_version,
            evidence=tuple(evidence),
            trace=tuple(trace),
        )

    def _deadline(
        self,
        started_at_ms: int,
    ) -> Tuple[int, bool]:
        now_ms = self._now_ms()
        return (
            now_ms,
            now_ms < started_at_ms
            or now_ms - started_at_ms >= self._limits.max_elapsed_ms,
        )

    def _replan(
        self,
        code: str,
        context_version: int,
        counters,
        tool_call_id=None,
    ):
        if counters["replans"] >= self._limits.max_replans:
            return None
        counters["replans"] += 1
        next_version = context_version + 1
        return (
            next_version,
            ResearchFeedback(
                code=code,
                context_version=next_version,
                tool_call_id=tool_call_id,
            ),
        )

    def run(self, goal: ResearchGoal) -> ResearchEpisodeResult:
        if not isinstance(goal, ResearchGoal):
            raise ResearchLoopError(
                "invalid_research_goal",
                "ResearchLoop requires ResearchGoal",
            )
        started_at_ms = self._now_ms()
        context_version = 1
        evidence = []
        feedback = None
        seen_proposals = set()
        proposal_history = []
        seen_tool_call_ids = set()
        seen_evidence_ids = set()
        counters = {
            "planner_turns": 0,
            "tool_calls": 0,
            "replans": 0,
        }
        trace = ["CREATED"]

        while True:
            now_ms, exhausted = self._deadline(started_at_ms)
            if (
                exhausted
                or counters["planner_turns"]
                >= self._limits.max_planner_turns
            ):
                return self._finish(
                    goal,
                    BUDGET_EXHAUSTED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                )

            fresh_evidence = tuple(
                item for item in evidence if item.fresh(now_ms)
            )
            remaining_elapsed_ms = max(
                0,
                self._limits.max_elapsed_ms - (now_ms - started_at_ms),
            )
            context = ResearchPlanningContext(
                turn_id=goal.turn_id,
                user_query=goal.user_query,
                require_evidence=goal.require_evidence,
                available_tools=self._tools.names,
                used_proposal_ids=tuple(proposal_history),
                context_version=context_version,
                evidence=fresh_evidence,
                previous_feedback=feedback,
                planner_turn=counters["planner_turns"] + 1,
                remaining_tool_calls=(
                    self._limits.max_tool_calls - counters["tool_calls"]
                ),
                remaining_replans=(
                    self._limits.max_replans - counters["replans"]
                ),
                remaining_elapsed_ms=remaining_elapsed_ms,
                planner_timeout_ms=min(
                    self._limits.max_planner_latency_ms,
                    remaining_elapsed_ms,
                ),
            )
            trace.append("PLANNING")
            planner_started = self._now_ms()
            try:
                raw = self._planner(context)
            except Exception:
                counters["planner_turns"] += 1
                return self._finish(
                    goal,
                    PLANNER_FAILED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                )
            planner_completed = self._now_ms()
            counters["planner_turns"] += 1
            if (
                planner_completed < planner_started
                or planner_completed - planner_started
                >= self._limits.max_planner_latency_ms
            ):
                return self._finish(
                    goal,
                    PLANNER_FAILED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                )
            _, exhausted = self._deadline(started_at_ms)
            if exhausted:
                return self._finish(
                    goal,
                    BUDGET_EXHAUSTED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                )

            proposal = None
            try:
                proposal = decode_research_decision(raw)
                if proposal.proposal_id in seen_proposals:
                    raise ResearchLoopError(
                        "duplicate_proposal",
                        "Proposal ID was replayed",
                    )
                seen_proposals.add(proposal.proposal_id)
                proposal_history.append(proposal.proposal_id)
                if proposal.turn_id != goal.turn_id:
                    raise ResearchLoopError(
                        "wrong_turn",
                        "Proposal referenced another turn",
                    )
                if (
                    proposal.based_on_context_version
                    != context_version
                ):
                    raise ResearchLoopError(
                        "stale_context",
                        "Proposal referenced another context version",
                    )
            except ResearchLoopError as error:
                replanned = self._replan(
                    error.code,
                    context_version,
                    counters,
                )
                if replanned is None:
                    return self._finish(
                        goal,
                        BUDGET_EXHAUSTED,
                        context_version,
                        evidence,
                        counters,
                        trace,
                    )
                context_version, feedback = replanned
                trace.append("REPLANNING")
                continue

            if proposal.decision == "ABORT":
                return self._finish(
                    goal,
                    PLANNER_ABORTED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                )
            if proposal.decision == "CLARIFY":
                return self._finish(
                    goal,
                    CLARIFICATION_REQUIRED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                    clarification_question=proposal.question,
                )
            if proposal.decision == "ANSWER":
                now_ms, exhausted = self._deadline(started_at_ms)
                if exhausted:
                    return self._finish(
                        goal,
                        BUDGET_EXHAUSTED,
                        context_version,
                        evidence,
                        counters,
                        trace,
                    )
                by_id = {
                    item.evidence_id: item
                    for item in evidence
                }
                citation_ids = proposal.answer.evidence_ids
                citations_valid = all(
                    evidence_id in by_id
                    and by_id[evidence_id].fresh(now_ms)
                    for evidence_id in citation_ids
                )
                if goal.require_evidence and not citation_ids:
                    citations_valid = False
                if not citations_valid:
                    replanned = self._replan(
                        "invalid_or_stale_citation",
                        context_version,
                        counters,
                    )
                    if replanned is None:
                        return self._finish(
                            goal,
                            BUDGET_EXHAUSTED,
                            context_version,
                            evidence,
                            counters,
                            trace,
                        )
                    context_version, feedback = replanned
                    trace.append("REPLANNING")
                    continue
                return self._finish(
                    goal,
                    ANSWERED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                    answer_text=proposal.answer.text,
                    citation_ids=citation_ids,
                )

            if counters["tool_calls"] >= self._limits.max_tool_calls:
                return self._finish(
                    goal,
                    BUDGET_EXHAUSTED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                )
            if counters["replans"] >= self._limits.max_replans:
                return self._finish(
                    goal,
                    BUDGET_EXHAUSTED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                )
            try:
                self._tools.validate(
                    proposal.tool.name,
                    proposal.tool.arguments,
                )
            except ResearchLoopError as error:
                replanned = self._replan(
                    error.code,
                    context_version,
                    counters,
                )
                if replanned is None:
                    return self._finish(
                        goal,
                        BUDGET_EXHAUSTED,
                        context_version,
                        evidence,
                        counters,
                        trace,
                    )
                context_version, feedback = replanned
                trace.append("REPLANNING")
                continue
            tool_call_id = "tool-{}".format(self._id_factory())
            _identifier("tool_call_id", tool_call_id)
            if tool_call_id in seen_tool_call_ids:
                return self._finish(
                    goal,
                    TOOL_FAILED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                )
            seen_tool_call_ids.add(tool_call_id)
            issued_at_ms, exhausted = self._deadline(started_at_ms)
            if exhausted:
                return self._finish(
                    goal,
                    BUDGET_EXHAUSTED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                )
            episode_deadline_ms = (
                started_at_ms + self._limits.max_elapsed_ms
            )
            valid_until_ms = min(
                issued_at_ms + self._limits.tool_request_ttl_ms,
                episode_deadline_ms,
            )
            counters["tool_calls"] += 1
            trace.append("CALLING_TOOL")
            try:
                result = self._tools.execute(
                    proposal.tool.name,
                    proposal.tool.arguments,
                    tool_call_id,
                    issued_at_ms,
                    valid_until_ms,
                )
                completed_at_ms = self._now_ms()
                if (
                    completed_at_ms < issued_at_ms
                    or completed_at_ms >= valid_until_ms
                ):
                    raise ResearchLoopError(
                        "tool_timeout",
                        "Research tool exceeded its deadline",
                    )
                envelope = self._weather_evidence(
                    result,
                    proposal.tool.name,
                    tool_call_id,
                    context_version,
                    proposal.tool.arguments["location_query"],
                    issued_at_ms,
                    valid_until_ms,
                    completed_at_ms,
                )
            except ResearchError:
                replanned = self._replan(
                    "tool_failed",
                    context_version,
                    counters,
                    tool_call_id,
                )
                if replanned is None:
                    return self._finish(
                        goal,
                        TOOL_FAILED,
                        context_version,
                        evidence,
                        counters,
                        trace,
                    )
                context_version, feedback = replanned
                trace.append("REPLANNING")
                continue
            except ResearchLoopError as error:
                if error.code != "tool_timeout":
                    return self._finish(
                        goal,
                        TOOL_FAILED,
                        context_version,
                        evidence,
                        counters,
                        trace,
                    )
                replanned = self._replan(
                    error.code,
                    context_version,
                    counters,
                    tool_call_id,
                )
                if replanned is None:
                    return self._finish(
                        goal,
                        TOOL_FAILED,
                        context_version,
                        evidence,
                        counters,
                        trace,
                    )
                context_version, feedback = replanned
                trace.append("REPLANNING")
                continue
            if envelope.evidence_id in seen_evidence_ids:
                return self._finish(
                    goal,
                    TOOL_FAILED,
                    context_version,
                    evidence,
                    counters,
                    trace,
                )
            seen_evidence_ids.add(envelope.evidence_id)
            evidence.append(envelope)
            context_version += 1
            counters["replans"] += 1
            feedback = ResearchFeedback(
                code="tool_completed",
                context_version=context_version,
                tool_call_id=tool_call_id,
            )
            trace.extend(("EVIDENCE_RECEIVED", "REPLANNING"))

    def _weather_evidence(
        self,
        result: WeatherResearchResult,
        tool_name: str,
        tool_call_id: str,
        context_version: int,
        expected_location_query: str,
        issued_at_ms: int,
        request_valid_until_ms: int,
        received_at_ms: int,
    ) -> ResearchEvidenceEnvelope:
        """Convert a validated weather result into passive model evidence."""

        request = result.request
        if (
            not isinstance(request, WeatherResearchRequest)
            or result.request_id != tool_call_id
            or request.request_id != tool_call_id
            or request.location_query != expected_location_query
            or request.issued_at_monotonic_ms != issued_at_ms
            or request.valid_until_monotonic_ms
            != request_valid_until_ms
            or isinstance(result.completed_at_monotonic_ms, bool)
            or not isinstance(result.completed_at_monotonic_ms, int)
            or result.completed_at_monotonic_ms < issued_at_ms
            or result.completed_at_monotonic_ms > received_at_ms
        ):
            raise ResearchLoopError(
                "invalid_tool_result",
                "Weather result did not match its request",
            )
        try:
            location_query = dict(
                parse_qsl(
                    urlsplit(
                        result.location_evidence.provenance.source_url
                    ).query,
                    keep_blank_values=True,
                    strict_parsing=True,
                )
            )["name"]
            weather_query = dict(
                parse_qsl(
                    urlsplit(
                        result.weather_evidence.provenance.source_url
                    ).query,
                    keep_blank_values=True,
                    strict_parsing=True,
                )
            )
        except (KeyError, ValueError):
            raise ResearchLoopError(
                "invalid_tool_result",
                "Weather provenance did not match the request",
            ) from None
        latitude_token = (
            "0"
            if result.location.latitude == 0.0
            else "{:.6f}".format(result.location.latitude)
            .rstrip("0")
            .rstrip(".")
        )
        longitude_token = (
            "0"
            if result.location.longitude == 0.0
            else "{:.6f}".format(result.location.longitude)
            .rstrip("0")
            .rstrip(".")
        )
        if (
            location_query != expected_location_query
            or weather_query.get("latitude") != latitude_token
            or weather_query.get("longitude") != longitude_token
        ):
            raise ResearchLoopError(
                "invalid_tool_result",
                "Weather provenance did not match the request",
            )
        try:
            observed_at_ms = int(
                datetime.fromisoformat(
                    result.weather.observed_at
                )
                .replace(tzinfo=timezone.utc)
                .timestamp()
                * 1_000
            )
        except (OverflowError, ValueError):
            raise ResearchLoopError(
                "invalid_tool_result",
                "Weather observation time is invalid",
            ) from None
        retrieved_at_ms = (
            result.weather_evidence.provenance.retrieved_at_unix_ms
        )
        observation_skew_ms = abs(
            retrieved_at_ms - observed_at_ms
        )
        if (
            observation_skew_ms
            >= self._limits.max_weather_observation_skew_ms
        ):
            raise ResearchLoopError(
                "stale_tool_result",
                "Weather observation was stale",
            )
        remaining_observation_freshness_ms = (
            self._limits.max_weather_observation_skew_ms
            - max(0, retrieved_at_ms - observed_at_ms)
        )
        ttl_values = []
        for item in result.evidence:
            provenance = item.provenance
            ttl = provenance.ttl_ms
            if (
                isinstance(ttl, bool)
                or not isinstance(ttl, int)
                or ttl <= 0
            ):
                raise ResearchLoopError(
                    "invalid_tool_result",
                    "Weather evidence TTL is invalid",
                )
            ttl_values.append(ttl)
        if not ttl_values:
            raise ResearchLoopError(
                "invalid_tool_result",
                "Weather result contained no evidence",
            )
        raw_payload = result.to_dict()
        if not isinstance(raw_payload, Mapping):
            raise ResearchLoopError(
                "invalid_tool_result",
                "Weather result payload is invalid",
            )
        try:
            encoded_payload = json.dumps(
                raw_payload,
                ensure_ascii=False,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        except (TypeError, ValueError):
            raise ResearchLoopError(
                "invalid_tool_result",
                "Weather result payload is not JSON-safe",
            ) from None
        if len(encoded_payload) > MAX_EVIDENCE_BYTES:
            raise ResearchLoopError(
                "evidence_too_large",
                "Weather evidence exceeded the payload limit",
            )
        evidence_id = "evidence-{}".format(self._id_factory())
        _identifier("evidence_id", evidence_id)
        return ResearchEvidenceEnvelope(
            evidence_id=evidence_id,
            tool_call_id=tool_call_id,
            tool_name=tool_name,
            produced_from_context_version=context_version,
            received_at_monotonic_ms=received_at_ms,
            valid_until_monotonic_ms=(
                received_at_ms
                + min(
                    min(ttl_values),
                    self._limits.evidence_ttl_ms,
                    remaining_observation_freshness_ms,
                )
            ),
            payload=raw_payload,
        )
