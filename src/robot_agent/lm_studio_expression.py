"""Strict LM Studio adapter for asynchronous, motion-free expression proposals.

The adapter gives a local language model one immutable
:class:`~robot_agent.interaction_contract.InteractionSnapshot` and asks for a
typed expression proposal.  It has no robot, motor, audio, SSH, or tool
imports.  Its result is still untrusted JSON bytes; the interaction layer must
decode it with ``decode_expression_proposal`` and re-check its snapshot
bindings before accepting it.
"""

from __future__ import annotations

import json
import socket
import time
from typing import Callable, Mapping

from .interaction_contract import (
    EXPRESSION_PROPOSAL_SCHEMA,
    InteractionSnapshot,
    expression_proposal_id_for_snapshot,
)
from .lm_studio import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    LMStudioConfigurationError,
    LMStudioError,
    LMStudioProtocolError,
    LMStudioResponseTooLargeError,
    LMStudioTimeoutError,
    LMStudioTransportError,
    _safe_base_url,
    _safe_model,
    _stdlib_post,
)


CHAT_COMPLETIONS_PATH = "/v1/chat/completions"
EXPRESSION_REQUEST_TIMEOUT_SECONDS = 10.0
MAX_EXPRESSION_RESPONSE_BYTES = 64 * 1024
MAX_EXPRESSION_PROPOSAL_BYTES = 16 * 1024
MAX_EXPRESSION_OUTPUT_TOKENS = 512
MAX_MODEL_UTTERANCE_CHARS = 120
_TRIMMED_NO_CONTROL_PATTERN = (
    r"^[^\u0000-\u0020\u0085\u00A0\u1680\u2000-\u200A"
    r"\u2028\u2029\u202F\u205F\u3000]"
    r"([^\u0000-\u001F]*"
    r"[^\u0000-\u0020\u0085\u00A0\u1680\u2000-\u200A"
    r"\u2028\u2029\u202F\u205F\u3000])?$"
)

Transport = Callable[[str, bytes, Mapping[str, str], float, int], bytes]
Clock = Callable[[], float]


_SYSTEM_PROMPT = (
    "You are the harmless, grumpy LEGO robot persona for Robot LLM Lab. "
    "Return exactly one expression proposal matching the supplied strict "
    "schema, including the host-assigned proposal_id constant. Speech does "
    "not require a gesture. Propose the semantic "
    "PROPELLER_WAVE gesture only when the robot is genuinely strongly upset, "
    "and otherwise return a speech-only EXPRESS intent, "
    "but never motor roles, ports, speed, duration, source, TTL, priority, "
    "authority, or executable physical commands. The host alone decides "
    "whether and how a semantic gesture is executed. The response_locale "
    "field is authoritative: write the utterance naturally in exactly that "
    "locale and do not infer its language from any other input. If "
    "interaction_snapshot.evidence.object_id is null, never claim to know "
    "the obstruction's identity. Keep the utterance concise and no longer "
    "than {} characters. Be amusingly grumpy without threats, abuse, or "
    "unsafe language. Return JSON only, with no markdown."
).format(MAX_MODEL_UTTERANCE_CHARS)


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


def _identifier_schema(
    maximum: int = 128,
    patterned: bool = True,
) -> Mapping[str, object]:
    value = {
        "type": "string",
        "minLength": 1,
        "maxLength": maximum,
    }
    if patterned:
        value["pattern"] = _TRIMMED_NO_CONTROL_PATTERN
    return value


def _const_string(value: str) -> Mapping[str, object]:
    return {
        "type": "string",
        "const": value,
    }


def _const_integer(value: int) -> Mapping[str, object]:
    return {
        "type": "integer",
        "const": value,
    }


def _object_schema(
    properties: Mapping[str, object],
    required,
) -> Mapping[str, object]:
    return {
        "type": "object",
        "properties": dict(properties),
        "required": list(required),
        "additionalProperties": False,
    }


def _safe_response_locale(value: str) -> str:
    # Keep locale handling generic and aligned with ExpressionIntent: this is
    # an identifier contract, not a language allowlist or detection rule.
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 64
        or any(ord(character) < 32 for character in value)
    ):
        raise LMStudioConfigurationError(
            "LM Studio expression response locale is invalid"
        )
    return value


def _common_properties(
    snapshot: InteractionSnapshot,
    decision: str,
) -> Mapping[str, object]:
    evidence_id = (
        None if snapshot.evidence is None else snapshot.evidence.evidence_id
    )
    evidence_binding = {
        "type": "null",
        "const": None,
    }
    if evidence_id is not None:
        evidence_binding = _const_string(evidence_id)
    return {
        "schema": _const_string(EXPRESSION_PROPOSAL_SCHEMA),
        "robot_id": _const_string(snapshot.robot_id),
        "controller_instance_id": _const_string(
            snapshot.controller_instance_id
        ),
        "goal_id": _const_string(snapshot.goal_id),
        "goal_epoch": _const_integer(snapshot.goal_epoch),
        "plan_revision": _const_integer(snapshot.plan_revision),
        "based_on_interaction_state_version": _const_integer(
            snapshot.interaction_state_version
        ),
        "based_on_world_model_version": _const_integer(
            snapshot.world_model_version
        ),
        "obstruction_epoch": _const_integer(snapshot.obstruction_epoch),
        "based_on_evidence_id": evidence_binding,
        "decision": _const_string(decision),
        "confidence_milli": {
            "type": "integer",
            "minimum": 0,
            "maximum": 1_000,
        },
        "proposal_id": _const_string(
            expression_proposal_id_for_snapshot(snapshot)
        ),
    }


_COMMON_REQUIRED = (
    "schema",
    "proposal_id",
    "robot_id",
    "controller_instance_id",
    "goal_id",
    "goal_epoch",
    "plan_revision",
    "based_on_interaction_state_version",
    "based_on_world_model_version",
    "obstruction_epoch",
    "based_on_evidence_id",
    "decision",
    "confidence_milli",
)


def _proposal_schema(
    snapshot: InteractionSnapshot,
    response_locale: str,
) -> Mapping[str, object]:
    variants = []
    if snapshot.evidence is not None:
        for gesture_kind in (None, "PROPELLER_WAVE"):
            express_properties = dict(
                _common_properties(snapshot, "EXPRESS")
            )
            gesture_schema = {
                "type": "null",
                "const": None,
            }
            repetitions_schema = _const_integer(0)
            if gesture_kind is not None:
                gesture_schema = _const_string(gesture_kind)
                repetitions_schema = {
                    "type": "integer",
                    "minimum": 1,
                    "maximum": 2,
                }
            proposal_id_schema = express_properties.pop("proposal_id")
            express_properties["intent"] = _object_schema(
                {
                    "utterance_locale": _const_string(response_locale),
                    "gesture_kind": gesture_schema,
                    "affect_label": _identifier_schema(
                        64,
                        patterned=False,
                    ),
                    "intensity": {
                        "type": "integer",
                        "minimum": 0,
                        "maximum": 1_000,
                    },
                    "repetitions": repetitions_schema,
                    # Keep the patterned field last for LM Studio grammar
                    # compatibility.
                    "utterance": {
                        "type": "string",
                        "minLength": 1,
                        "maxLength": MAX_MODEL_UTTERANCE_CHARS,
                        "pattern": _TRIMMED_NO_CONTROL_PATTERN,
                        "description": (
                            "Write one concise utterance in exactly "
                            "response_locale {!r}; that host-authored locale "
                            "is authoritative. Use at most {} characters."
                        ).format(
                            response_locale,
                            MAX_MODEL_UTTERANCE_CHARS,
                        ),
                    },
                },
                (
                    "utterance_locale",
                    "gesture_kind",
                    "affect_label",
                    "intensity",
                    "repetitions",
                    "utterance",
                ),
            )
            express_properties["proposal_id"] = proposal_id_schema
            variants.append(
                _object_schema(
                    express_properties,
                    _COMMON_REQUIRED + ("intent",),
                )
            )

    for decision in ("HOLD", "ABORT"):
        properties = dict(_common_properties(snapshot, decision))
        proposal_id_schema = properties.pop("proposal_id")
        properties["reason_code"] = _identifier_schema(
            64,
            patterned=False,
        )
        properties["proposal_id"] = proposal_id_schema
        variants.append(
            _object_schema(
                properties,
                _COMMON_REQUIRED + ("reason_code",),
            )
        )
    return {"oneOf": variants}


def _decode_completion(body: bytes, expected_model: str) -> bytes:
    if not isinstance(body, bytes):
        raise LMStudioProtocolError(
            "LM Studio expression response was not bytes"
        )
    if len(body) > MAX_EXPRESSION_RESPONSE_BYTES:
        raise LMStudioResponseTooLargeError(
            "LM Studio expression response was too large"
        )
    try:
        value = json.loads(
            body.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, TypeError, ValueError):
        raise LMStudioProtocolError(
            "LM Studio returned invalid expression JSON"
        ) from None
    if not isinstance(value, dict):
        raise LMStudioProtocolError(
            "LM Studio expression response was not an object"
        )
    if value.get("object") != "chat.completion":
        raise LMStudioProtocolError(
            "LM Studio expression response object was invalid"
        )
    if value.get("model") != expected_model:
        raise LMStudioProtocolError(
            "LM Studio expression response model did not match"
        )
    choices = value.get("choices")
    if not isinstance(choices, list) or len(choices) != 1:
        raise LMStudioProtocolError(
            "LM Studio expression response must contain one choice"
        )
    choice = choices[0]
    if (
        not isinstance(choice, dict)
        or type(choice.get("index")) is not int
        or choice.get("index") != 0
        or choice.get("finish_reason") != "stop"
    ):
        raise LMStudioProtocolError(
            "LM Studio expression choice was invalid"
        )
    message = choice.get("message")
    if (
        not isinstance(message, dict)
        or message.get("role") != "assistant"
        or message.get("tool_calls") not in (None, [])
    ):
        raise LMStudioProtocolError(
            "LM Studio expression message was invalid"
        )
    content = message.get("content")
    if not isinstance(content, str) or not content.strip():
        raise LMStudioProtocolError(
            "LM Studio expression message was empty"
        )
    proposal = content.encode("utf-8")
    if len(proposal) > MAX_EXPRESSION_PROPOSAL_BYTES:
        raise LMStudioResponseTooLargeError(
            "LM Studio expression proposal was too large"
        )
    return proposal


class LMStudioExpressionPlanner:
    """Callable structured-output adapter for expression-only proposals."""

    def __init__(
        self,
        response_locale: str,
        base_url: str = DEFAULT_BASE_URL,
        model: str = DEFAULT_MODEL,
        transport: Transport = _stdlib_post,
        clock: Clock = time.monotonic,
        timeout_seconds: float = EXPRESSION_REQUEST_TIMEOUT_SECONDS,
    ):
        if (
            not callable(transport)
            or not callable(clock)
            or isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not 0.1 <= timeout_seconds <= 10.0
        ):
            raise LMStudioConfigurationError(
                "LM Studio expression planner dependency is invalid"
            )
        self._response_locale = _safe_response_locale(response_locale)
        self._base_url = _safe_base_url(base_url)
        self._model = _safe_model(model)
        self._transport = transport
        self._clock = clock
        self._timeout_seconds = float(timeout_seconds)

    @property
    def model(self) -> str:
        return self._model

    @property
    def response_locale(self) -> str:
        return self._response_locale

    def __call__(self, snapshot: InteractionSnapshot) -> bytes:
        if not isinstance(snapshot, InteractionSnapshot):
            raise LMStudioProtocolError(
                "Interaction snapshot is invalid"
            )
        if snapshot.response_locale != self._response_locale:
            raise LMStudioProtocolError(
                "Interaction snapshot response locale did not match"
            )
        payload = {
            "interaction_snapshot": snapshot.to_dict(),
            "proposal_id": expression_proposal_id_for_snapshot(snapshot),
            "response_locale": self._response_locale,
        }
        request_value = {
            "model": self._model,
            "messages": [
                {
                    "role": "system",
                    "content": _SYSTEM_PROMPT,
                },
                {
                    "role": "user",
                    "content": json.dumps(
                        payload,
                        ensure_ascii=False,
                        allow_nan=False,
                        separators=(",", ":"),
                    ),
                },
            ],
            "response_format": {
                "type": "json_schema",
                "json_schema": {
                    "name": "expression_proposal",
                    "strict": True,
                    "schema": _proposal_schema(
                        snapshot,
                        self._response_locale,
                    ),
                },
            },
            "temperature": 0,
            "reasoning_effort": "none",
            "max_tokens": MAX_EXPRESSION_OUTPUT_TOKENS,
            "stream": False,
            "store": False,
        }
        try:
            body = json.dumps(
                request_value,
                ensure_ascii=False,
                allow_nan=False,
                separators=(",", ":"),
            ).encode("utf-8")
        except (TypeError, ValueError):
            raise LMStudioProtocolError(
                "Interaction snapshot was not strict JSON"
            ) from None
        headers = {
            "Accept": "application/json",
            "Content-Type": "application/json; charset=utf-8",
        }

        started = self._clock()
        try:
            response_body = self._transport(
                self._base_url + CHAT_COMPLETIONS_PATH,
                body,
                headers,
                self._timeout_seconds,
                MAX_EXPRESSION_RESPONSE_BYTES,
            )
        except LMStudioError:
            raise
        except (socket.timeout, TimeoutError):
            raise LMStudioTimeoutError(
                "LM Studio expression request timed out"
            ) from None
        except OSError:
            raise LMStudioTransportError(
                "LM Studio expression request failed"
            ) from None
        completed = self._clock()
        if max(0.0, completed - started) >= self._timeout_seconds:
            raise LMStudioTimeoutError(
                "LM Studio expression request timed out"
            )
        return _decode_completion(response_body, self._model)


# A descriptive alias for callers that view this class as an adapter rather
# than as one asynchronous proposal producer.
LMStudioExpressionAdapter = LMStudioExpressionPlanner
