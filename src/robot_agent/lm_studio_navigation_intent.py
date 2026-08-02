"""Compact LM Studio client for identity-free navigation-intent planning."""

from dataclasses import dataclass
import json
import math
import time
from typing import Callable, Mapping, Optional

from .lm_studio import LMStudioHTTPError, _stdlib_post
from .lm_studio_endpoint import (
    CHAT_COMPLETIONS_PATHS,
    LMStudioEndpointError,
    OPENAI_V1_CHAT_COMPLETIONS_PATH,
    validate_lm_studio_base_url,
    validate_lm_studio_model_id,
)
from .navigation_intent_context import (
    MAX_NAVIGATION_INTENT_ACCOUNTED_BYTES,
    MAX_NAVIGATION_INTENT_CONTEXT_BYTES,
    NAVIGATION_INTENT_WRAPPER_RESERVE_BYTES,
    SYSTEM_PROMPT,
    NavigationIntentPrompt,
)
from .navigation_intent_proposal import (
    MAX_NAVIGATION_INTENT_TTL_MS,
    NavigationIntentEnvelope,
    NavigationIntentOffer,
    NavigationIntentProposalError,
    bind_navigation_intent_proposal,
    build_navigation_intent_proposal_schema,
    decode_navigation_intent_proposal,
)
from .physical_navigation_contract import (
    PhysicalNavigationContractError,
    strict_json_loads,
)


MAX_RESPONSE_BYTES = 16 * 1024
MAX_OUTPUT_TOKENS = 80
DEFAULT_PROPOSAL_TTL_MS = 5_000
DEFAULT_TIMEOUT_SECONDS = 45.0

Transport = Callable[[str, bytes, Mapping[str, str], float, int], bytes]
MonotonicClock = Callable[[], float]
UnixClock = Callable[[], int]

_CONTEXT_FIELDS = frozenset((
    "objective",
    "locale",
    "mission",
    "pose",
    "known_hazard_count",
    "active_intent",
    "intent_progress",
    "offered_target_evidence",
    "latest_outcome",
))
_FORBIDDEN_MESSAGE_KEYS = frozenset((
    "ticket",
    "ticket_id",
    "proposal_id",
    "planning_ticket",
    "basis",
    "controller_key",
    "controller_id",
    "controller_instance_id",
    "robot_id",
    "goal_id",
    "goal_epoch",
    "intent_id",
    "state",
    "state_version",
    "controller_state_version",
    "world_generation_id",
    "world_model_version",
    "navigation_basis_id",
    "frame_id",
    "calibration_fingerprint",
    "accepted_at_ms",
))


class LMStudioNavigationIntentError(RuntimeError):
    def __init__(
        self,
        code: str,
        message: str,
        *,
        latency_ms: int = 0,
        http_status: Optional[int] = None,
        proposal_error_code: Optional[str] = None,
    ):
        self.code = code
        self.latency_ms = latency_ms
        self.http_status = http_status
        self.proposal_error_code = proposal_error_code
        super().__init__(message)


@dataclass(frozen=True)
class LMStudioNavigationIntentResult:
    envelope: NavigationIntentEnvelope
    latency_ms: int
    served_model: str
    prompt_tokens: Optional[int]
    completion_tokens: Optional[int]
    total_tokens: Optional[int]
    server_tokens_per_second: Optional[float]
    server_time_to_first_token_seconds: Optional[float]
    context_bytes: int
    accounted_bytes: int


def _json_bytes(value: object) -> bytes:
    try:
        return json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    except (TypeError, ValueError):
        raise LMStudioNavigationIntentError(
            "intent_prompt_invalid",
            "Navigation intent prompt is not finite JSON",
        ) from None


def _contains_forbidden_key(value: object) -> bool:
    if isinstance(value, Mapping):
        return any(
            key in _FORBIDDEN_MESSAGE_KEYS
            or _contains_forbidden_key(item)
            for key, item in value.items()
        )
    if isinstance(value, (list, tuple)):
        return any(_contains_forbidden_key(item) for item in value)
    return False


def _validate_prompt(
    prompt: NavigationIntentPrompt,
    offer: NavigationIntentOffer,
) -> bytes:
    if not isinstance(prompt, NavigationIntentPrompt) or not isinstance(
        offer, NavigationIntentOffer
    ):
        raise LMStudioNavigationIntentError(
            "intent_prompt_invalid",
            "Prompt and offer must use the navigation-intent contracts",
        )
    if prompt.system_prompt != SYSTEM_PROMPT:
        raise LMStudioNavigationIntentError(
            "intent_prompt_invalid",
            "Prompt system instructions differ from the fixed contract",
        )
    if set(prompt.context) != _CONTEXT_FIELDS:
        raise LMStudioNavigationIntentError(
            "intent_prompt_invalid",
            "Prompt context fields differ from the identity-free projection",
        )
    if _contains_forbidden_key(prompt.context):
        raise LMStudioNavigationIntentError(
            "intent_prompt_identity_leak",
            "Host identity or state leaked into the model message",
        )
    evidence = prompt.context.get("offered_target_evidence")
    target_ids = tuple(sorted(set(
        offer.scan_target_ids + offer.detour_target_ids
    )))
    evidence_ids = (
        tuple(item.get("target_id") for item in evidence)
        if isinstance(evidence, list)
        and all(isinstance(item, Mapping) for item in evidence)
        else None
    )
    if (
        evidence_ids is None
        or any(not isinstance(item, str) for item in evidence_ids)
        or tuple(sorted(evidence_ids)) != target_ids
    ):
        raise LMStudioNavigationIntentError(
            "intent_prompt_offer_mismatch",
            "Prompt target evidence does not match the current host offer",
        )
    expected_schema = build_navigation_intent_proposal_schema(offer)
    if prompt.response_schema != expected_schema:
        raise LMStudioNavigationIntentError(
            "intent_prompt_offer_mismatch",
            "Prompt response schema does not match the current host offer",
        )
    context_bytes = _json_bytes(prompt.context)
    schema_bytes = _json_bytes(expected_schema)
    accounted = (
        len(SYSTEM_PROMPT.encode("utf-8"))
        + len(context_bytes)
        + len(schema_bytes)
        + NAVIGATION_INTENT_WRAPPER_RESERVE_BYTES
    )
    if (
        prompt.context_bytes != len(context_bytes)
        or prompt.accounted_bytes != accounted
        or len(context_bytes) > MAX_NAVIGATION_INTENT_CONTEXT_BYTES
        or accounted > MAX_NAVIGATION_INTENT_ACCOUNTED_BYTES
    ):
        raise LMStudioNavigationIntentError(
            "intent_prompt_size_mismatch",
            "Prompt size accounting is invalid or exceeds its budget",
        )
    return context_bytes


def _elapsed_ms(started: object, finished: object) -> int:
    if (
        isinstance(started, bool)
        or isinstance(finished, bool)
        or not isinstance(started, (int, float))
        or not isinstance(finished, (int, float))
        or not math.isfinite(float(started))
        or not math.isfinite(float(finished))
        or finished < started
    ):
        raise LMStudioNavigationIntentError(
            "intent_clock_invalid",
            "Injected monotonic clock is invalid",
        )
    return int(round((finished - started) * 1_000))


def _token_count(usage: object, name: str) -> Optional[int]:
    if not isinstance(usage, Mapping):
        return None
    value = usage.get(name)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _stat(stats: object, name: str) -> Optional[float]:
    if not isinstance(stats, Mapping):
        return None
    value = stats.get(name)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or value < 0
    ):
        return None
    return float(value)


class LMStudioNavigationIntentClient:
    """Tool-free client that returns a host-bound semantic intent envelope."""

    def __init__(
        self,
        *,
        base_url: str,
        model: str,
        transport: Transport = _stdlib_post,
        monotonic: MonotonicClock = time.monotonic,
        unix_ms: UnixClock = lambda: time.time_ns() // 1_000_000,
        timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
        inference_path: str = OPENAI_V1_CHAT_COMPLETIONS_PATH,
        allow_private_lan: bool = False,
        proposal_ttl_ms: int = DEFAULT_PROPOSAL_TTL_MS,
    ):
        try:
            self.base_url = validate_lm_studio_base_url(
                base_url,
                allow_private_lan=allow_private_lan,
            )
            self.model = validate_lm_studio_model_id(model)
        except LMStudioEndpointError as error:
            raise LMStudioNavigationIntentError(
                "intent_configuration_invalid",
                str(error),
            ) from error
        if (
            not callable(transport)
            or not callable(monotonic)
            or not callable(unix_ms)
            or not isinstance(inference_path, str)
            or inference_path not in CHAT_COMPLETIONS_PATHS
            or isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not 0.5 <= timeout_seconds <= 60.0
            or isinstance(proposal_ttl_ms, bool)
            or not isinstance(proposal_ttl_ms, int)
            or not 1 <= proposal_ttl_ms <= MAX_NAVIGATION_INTENT_TTL_MS
        ):
            raise LMStudioNavigationIntentError(
                "intent_configuration_invalid",
                "Navigation intent client configuration is invalid",
            )
        self.transport = transport
        self.monotonic = monotonic
        self.unix_ms = unix_ms
        self.timeout_seconds = float(timeout_seconds)
        self.inference_path = inference_path
        self.proposal_ttl_ms = proposal_ttl_ms

    def decide(
        self,
        prompt: NavigationIntentPrompt,
        *,
        offer: NavigationIntentOffer,
        proposal_id: str,
    ) -> LMStudioNavigationIntentResult:
        context_bytes = _validate_prompt(prompt, offer)
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": context_bytes.decode("utf-8")},
            ],
            "response_format": {
                "type": "json_schema",
                "json_schema": {
                    "name": "navigation_intent_proposal",
                    "strict": True,
                    "schema": prompt.response_schema,
                },
            },
            "temperature": 0,
            "reasoning_effort": "none",
            "max_tokens": MAX_OUTPUT_TOKENS,
            "stream": False,
            "store": False,
        }
        started = self.monotonic()
        try:
            raw = self.transport(
                self.base_url + self.inference_path,
                _json_bytes(payload),
                {
                    "Accept": "application/json",
                    "Content-Type": "application/json; charset=utf-8",
                },
                self.timeout_seconds,
                MAX_RESPONSE_BYTES,
            )
        except LMStudioHTTPError as error:
            latency_ms = _elapsed_ms(started, self.monotonic())
            raise LMStudioNavigationIntentError(
                "intent_http_error",
                "LM Studio returned an HTTP error",
                latency_ms=latency_ms,
                http_status=error.status_code,
            ) from error
        except Exception as error:
            latency_ms = _elapsed_ms(started, self.monotonic())
            raise LMStudioNavigationIntentError(
                "intent_transport_failed",
                "LM Studio navigation-intent request failed",
                latency_ms=latency_ms,
            ) from error
        latency_ms = _elapsed_ms(started, self.monotonic())
        try:
            wire = strict_json_loads(raw, MAX_RESPONSE_BYTES)
            if not isinstance(wire, dict):
                raise KeyError
            if wire.get("model") != self.model:
                raise LMStudioNavigationIntentError(
                    "intent_served_model_mismatch",
                    "LM Studio served a different or unidentified model",
                    latency_ms=latency_ms,
                )
            choices = wire["choices"]
            if not isinstance(choices, list) or len(choices) != 1:
                raise KeyError
            choice = choices[0]
            message = choice["message"]
            if not isinstance(choice, dict) or not isinstance(message, dict):
                raise KeyError
            if message.get("role", "assistant") != "assistant":
                raise KeyError
            content = message["content"]
            if not isinstance(content, str):
                raise KeyError
        except LMStudioNavigationIntentError:
            raise
        except (KeyError, TypeError, PhysicalNavigationContractError) as error:
            raise LMStudioNavigationIntentError(
                "intent_response_invalid",
                "LM Studio returned an invalid navigation-intent response",
                latency_ms=latency_ms,
            ) from error

        try:
            proposal = decode_navigation_intent_proposal(
                content.encode("utf-8"),
                offer,
            )
        except (NavigationIntentProposalError, UnicodeEncodeError) as error:
            raise LMStudioNavigationIntentError(
                "intent_proposal_invalid",
                "LM Studio returned an invalid navigation intent",
                latency_ms=latency_ms,
                proposal_error_code=getattr(error, "code", None),
            ) from error

        usage = wire.get("usage")
        stats = wire.get("stats")
        completion_tokens = _token_count(usage, "completion_tokens")
        reasoning_tokens = _token_count(stats, "reasoning_output_tokens")
        if reasoning_tokens not in (None, 0):
            raise LMStudioNavigationIntentError(
                "intent_reasoning_policy_violated",
                "LM Studio produced reasoning tokens despite reasoning none",
                latency_ms=latency_ms,
            )
        if completion_tokens is not None and completion_tokens > MAX_OUTPUT_TOKENS:
            raise LMStudioNavigationIntentError(
                "intent_output_budget_violated",
                "LM Studio exceeded the navigation-intent output budget",
                latency_ms=latency_ms,
            )
        try:
            received_at_ms = self.unix_ms()
            envelope = bind_navigation_intent_proposal(
                proposal,
                offer=offer,
                proposal_id=proposal_id,
                received_at_ms=received_at_ms,
                valid_until_ms=received_at_ms + self.proposal_ttl_ms,
            )
        except Exception as error:
            code = getattr(error, "code", None)
            raise LMStudioNavigationIntentError(
                "intent_binding_failed",
                "Host identity, time, or TTL binding failed",
                latency_ms=latency_ms,
                proposal_error_code=code,
            ) from error
        return LMStudioNavigationIntentResult(
            envelope=envelope,
            latency_ms=latency_ms,
            served_model=self.model,
            prompt_tokens=_token_count(usage, "prompt_tokens"),
            completion_tokens=completion_tokens,
            total_tokens=_token_count(usage, "total_tokens"),
            server_tokens_per_second=_stat(stats, "tokens_per_second"),
            server_time_to_first_token_seconds=_stat(
                stats, "time_to_first_token"
            ),
            context_bytes=prompt.context_bytes,
            accounted_bytes=prompt.accounted_bytes,
        )


__all__ = (
    "DEFAULT_PROPOSAL_TTL_MS",
    "LMStudioNavigationIntentClient",
    "LMStudioNavigationIntentError",
    "LMStudioNavigationIntentResult",
    "MAX_OUTPUT_TOKENS",
    "MAX_RESPONSE_BYTES",
)
