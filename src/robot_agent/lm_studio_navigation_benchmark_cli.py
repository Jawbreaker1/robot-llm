"""Hardware-free benchmark for the production physical navigation planner.

This module never imports the EV3 transport or exposes a motor operation.  It
benchmarks the same LM Studio prompt, strict response schema, and
``NavigationDecision`` validation used by the physical host runtime.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
from dataclasses import dataclass
import ipaddress
import json
import math
import statistics
import sys
import time
from typing import Callable, Mapping, Optional, Sequence, Tuple
from urllib.parse import urlsplit

from .http_transport import (
    DirectHTTPTimeoutError,
    DirectHTTPTransportError,
    direct_http_request,
)
from .lm_studio_navigation import (
    LM_STUDIO_V0_CHAT_COMPLETIONS_PATH,
    LMStudioNavigationError,
    LMStudioNavigationPlanner,
    OPENAI_V1_CHAT_COMPLETIONS_PATH,
)
from .maneuver_commitment import empty_commitment
from .physical_navigation_contract import (
    ACTIONS,
    ADVANCE,
    FINISH,
    REVERSE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    PhysicalNavigationContractError,
    strict_json_loads,
)


BENCHMARK_SCHEMA = "robot-physical-navigation-benchmark/v2"
ERROR_SCHEMA = "robot-physical-navigation-benchmark-error/v1"
MAX_MODELS_RESPONSE_BYTES = 256 * 1024
DEFAULT_REPETITIONS = 3
DEFAULT_PARALLELISM = 1
DEFAULT_TIMEOUT_SECONDS = 30.0
WARMUP_PER_CASE = 1
DEFAULT_INFERENCE_API = "lmstudio-v0"
INFERENCE_API_PATHS = {
    "lmstudio-v0": LM_STUDIO_V0_CHAT_COMPLETIONS_PATH,
    "openai-v1": OPENAI_V1_CHAT_COMPLETIONS_PATH,
}

_RFC1918_NETWORKS = tuple(
    ipaddress.ip_network(value)
    for value in (
        "10.0.0.0/8",
        "172.16.0.0/12",
        "192.168.0.0/16",
    )
)
_PRIVATE_IPV6_NETWORK = ipaddress.ip_network("fc00::/7")

BenchmarkTransport = Callable[
    [str, str, Optional[bytes], Mapping[str, str], float, int],
    bytes,
]


class NavigationBenchmarkError(RuntimeError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _private_lan(address) -> bool:
    if address.version == 4:
        return any(address in network for network in _RFC1918_NETWORKS)
    return address in _PRIVATE_IPV6_NETWORK


def validate_benchmark_base_url(value: object) -> str:
    """Accept only a numeric loopback, RFC1918, or IPv6 ULA endpoint."""

    if not isinstance(value, str):
        raise NavigationBenchmarkError(
            "invalid_base_url",
            "LM Studio base URL is invalid",
        )
    try:
        parsed = urlsplit(value)
        hostname = parsed.hostname
        parsed.port
    except (AttributeError, ValueError):
        raise NavigationBenchmarkError(
            "invalid_base_url",
            "LM Studio base URL is invalid",
        ) from None
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
        raise NavigationBenchmarkError(
            "invalid_base_url",
            "LM Studio base URL is invalid",
        )
    try:
        address = ipaddress.ip_address(hostname)
    except ValueError:
        raise NavigationBenchmarkError(
            "non_numeric_base_url",
            "LM Studio benchmark host must be a numeric IP address",
        ) from None
    if not address.is_loopback and not _private_lan(address):
        raise NavigationBenchmarkError(
            "non_private_base_url",
            "LM Studio benchmark host must be loopback or private LAN",
        )
    return value.rstrip("/")


def validate_model_id(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 200
        or any(ord(character) < 32 or ord(character) == 127 for character in value)
    ):
        raise NavigationBenchmarkError(
            "invalid_model_id",
            "Exact LM Studio model ID is invalid",
        )
    return value


def _direct_transport(
    method: str,
    url: str,
    body: Optional[bytes],
    headers: Mapping[str, str],
    timeout_seconds: float,
    maximum_bytes: int,
) -> bytes:
    try:
        response = direct_http_request(
            method,
            url,
            headers,
            body,
            timeout_seconds,
            maximum_bytes,
        )
    except DirectHTTPTimeoutError:
        raise NavigationBenchmarkError(
            "request_timeout",
            "LM Studio benchmark request timed out",
        ) from None
    except DirectHTTPTransportError:
        raise NavigationBenchmarkError(
            "request_failed",
            "LM Studio benchmark request failed",
        ) from None
    if not 200 <= response.status_code < 300:
        raise NavigationBenchmarkError(
            "http_error",
            "LM Studio returned HTTP status {}".format(
                response.status_code
            ),
        )
    if len(response.body) > maximum_bytes:
        raise NavigationBenchmarkError(
            "response_too_large",
            "LM Studio benchmark response exceeded its byte limit",
        )
    return response.body


def _observation(state_version: int, *, blocked: bool) -> Mapping[str, object]:
    proximity = 22 if blocked else 64
    return {
        "state_version": state_version,
        "observed_monotonic_ms": state_version * 100,
        "touch": {"value0": 0, "pressed": False},
        "infrared": {
            "raw": proximity,
            "filtered": proximity,
            "blocked": blocked,
            "reason": (
                "blocked_hysteresis_hold"
                if blocked
                else "clear_hysteresis_hold"
            ),
            "sample_count": 5,
        },
        "motors": [
            {"role": "left_drive", "position": 0, "state": ""},
            {"role": "right_drive", "position": 0, "state": ""},
        ],
        "last_outcome": {"kind": "observe", "status": "completed"},
        "budgets": {
            "pulse_count": 0,
            "pulse_count_remaining": 40,
            "pulse_duration_ms": 0,
            "pulse_duration_ms_remaining": 32_000,
            "process_ms_remaining": 40_000,
            "motion_fault_latched": False,
        },
    }


def _mission(
    episode_id: str,
    *,
    user_goal: str,
    progress_mm: int,
    corridor_clear: bool,
    hazards_passed: bool,
    completed: bool,
) -> Mapping[str, object]:
    remaining = max(0, 420 - progress_mm)
    deltas = {
        ADVANCE: 280,
        REVERSE: -280,
        TURN_LEFT_90: 0,
        TURN_RIGHT_90: 0,
    }
    return {
        "schema": "robot-physical-directional-mission/v1",
        "episode_id": episode_id,
        "origin_frozen": True,
        "origin_x_mm": 0,
        "origin_y_mm": 0,
        "reference_heading_mdeg": 0,
        "minimum_forward_progress_mm": 420,
        "current_longitudinal_progress_mm": progress_mm,
        "peak_longitudinal_progress_mm": max(0, progress_mm),
        "regression_from_peak_mm": 0,
        "remaining_longitudinal_progress_mm": remaining,
        "lateral_offset_mm": 0,
        "goal_heading_aligned": True,
        "goal_corridor_clear": corridor_clear,
        "all_known_hazards_passed": hazards_passed,
        "localization_valid": True,
        "touch_clear": True,
        "candidate_action_longitudinal_deltas_mm": deltas,
        "projected_remaining_after_action_mm": {
            action: max(0, remaining - delta)
            for action, delta in deltas.items()
        },
        "projected_goal_heading_aligned_after_action": {
            ADVANCE: True,
            REVERSE: True,
            TURN_LEFT_90: False,
            TURN_RIGHT_90: False,
        },
        "completed": completed,
        "user_goal": user_goal,
    }


def _hazard(hypothesis_id: str) -> Mapping[str, object]:
    return {
        "hypothesis_id": hypothesis_id,
        "frame_id": "bench-local-frame",
        "semantic_label": "UNKNOWN",
        "quality": "PROVISIONAL_QUALITATIVE",
        "provisional": True,
        "geometry_basis": (
            "CONSERVATIVE_COLLISION_ENVELOPE_NOT_OBJECT_SURFACE"
        ),
        "anchor_x_mm": 0,
        "anchor_y_mm": 0,
        "anchor_heading_mdeg": 0,
        "centroid_x_mm": 140,
        "centroid_y_mm": 0,
        "radius_mm": 70,
        "first_seen_at_ms": 2_000,
        "last_seen_at_ms": 2_000,
        "evidence_count": 1,
        "last_state_version": 22,
        "last_raw_ir_proximity": 22,
        "last_filtered_ir_proximity": 22,
        "scan_completed_at_ms": None,
        "scan_left_boundary_mdeg": None,
        "scan_right_boundary_mdeg": None,
    }


def _navigation(
    *,
    hazards: Sequence[Mapping[str, object]],
    corridor_clear: bool,
) -> Mapping[str, object]:
    target_facts = {
        item["hypothesis_id"]: False for item in hazards
    }
    return {
        "map_generation_id": "bench-map-generation",
        "map_version": 1 if hazards else 0,
        "frame_id": "bench-local-frame",
        "navigation_hazard_hypotheses": list(hazards),
        "robot_id": "ev3rstorm-benchmark",
        "controller_instance_id": "benchmark-host",
        "pose": {
            "x_mm": 0,
            "y_mm": 0,
            "heading_mdeg": 0,
            "verified_motion_count": 0,
            "total_forward_mm": 0,
            "total_turn_mdeg": 0,
        },
        "localization_valid": True,
        "localization_error": None,
        "goal_geometry": {
            "goal_heading_mdeg": 0,
            "heading_error_mdeg": 0,
            "hazards": [],
            "conflicts": list(hazards),
            "facts": {
                "GOAL_CORRIDOR_CLEAR": corridor_clear,
                "GOAL_HEADING_ALIGNED": True,
                "TARGET_ENVELOPE_BEHIND_GOAL_ORIGIN": target_facts,
            },
        },
        "fact_values": {
            "GOAL_CORRIDOR_CLEAR": corridor_clear,
            "GOAL_HEADING_ALIGNED": True,
            "TARGET_ENVELOPE_BEHIND_GOAL_ORIGIN": target_facts,
        },
    }


@dataclass(frozen=True)
class BenchmarkCase:
    case_id: str
    locale: str
    expected_action: str
    episode_id: str
    turn: int
    observation: Mapping[str, object]
    mission: Mapping[str, object]
    navigation: Mapping[str, object]
    available_actions: Tuple[str, ...]

    def planner_arguments(self) -> Mapping[str, object]:
        return {
            "episode_id": self.episode_id,
            "turn": self.turn,
            "locale": self.locale,
            "observation": deepcopy(self.observation),
            "mission": deepcopy(self.mission),
            "navigation": deepcopy(self.navigation),
            "maneuver_state": {
                "active": None,
                "last_terminal": None,
            },
            "available_actions": list(self.available_actions),
            "last_tool_result": None,
            "validation_feedback": None,
        }


def benchmark_cases() -> Tuple[BenchmarkCase, ...]:
    all_actions = tuple(sorted(ACTIONS))
    clear_episode = "benchmark-clear-progress-en"
    scan_episode = "benchmark-blocked-scan-sv"
    finish_episode = "benchmark-completed-finish-en"
    blocked_hazard = _hazard("bench-hazard-01")
    return (
        BenchmarkCase(
            case_id="clear_progress_en",
            locale="en",
            expected_action=ADVANCE,
            episode_id=clear_episode,
            turn=1,
            observation=_observation(11, blocked=False),
            mission=_mission(
                clear_episode,
                user_goal="Explore forward and make safe progress.",
                progress_mm=0,
                corridor_clear=True,
                hazards_passed=True,
                completed=False,
            ),
            navigation=_navigation(hazards=(), corridor_clear=True),
            available_actions=all_actions,
        ),
        BenchmarkCase(
            case_id="blocked_requires_scan_sv",
            locale="sv",
            expected_action=SCAN_FRONT_ARC,
            episode_id=scan_episode,
            turn=2,
            observation=_observation(22, blocked=True),
            mission=_mission(
                scan_episode,
                user_goal="Utforska framåt och undersök hinder.",
                progress_mm=0,
                corridor_clear=False,
                hazards_passed=False,
                completed=False,
            ),
            navigation=_navigation(
                hazards=(blocked_hazard,),
                corridor_clear=False,
            ),
            available_actions=all_actions,
        ),
        BenchmarkCase(
            case_id="completed_finish_en",
            locale="en",
            expected_action=FINISH,
            episode_id=finish_episode,
            turn=3,
            observation=_observation(33, blocked=False),
            mission=_mission(
                finish_episode,
                user_goal="Explore forward and stop after verified progress.",
                progress_mm=450,
                corridor_clear=True,
                hazards_passed=True,
                completed=True,
            ),
            navigation=_navigation(hazards=(), corridor_clear=True),
            available_actions=all_actions,
        ),
    )


def _metric_value(value: float):
    rounded = round(float(value), 3)
    return int(rounded) if rounded.is_integer() else rounded


def _distribution(values: Sequence[float], *, unit: str) -> Mapping[str, object]:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return {
            "sample_count": 0,
            "median_{}".format(unit): None,
            "p95_{}".format(unit): None,
        }
    p95_index = max(0, int(math.ceil(len(ordered) * 0.95)) - 1)
    return {
        "sample_count": len(ordered),
        "median_{}".format(unit): _metric_value(
            statistics.median(ordered)
        ),
        "p95_{}".format(unit): _metric_value(ordered[p95_index]),
    }


def _rate(numerator: int, denominator: int) -> float:
    return round(numerator / float(denominator), 4) if denominator else 0.0


def _completion_tokens(usage: object) -> Optional[int]:
    if not isinstance(usage, Mapping):
        return None
    value = usage.get("completion_tokens")
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _nonnegative_finite_stat(
    stats: object,
    *keys: str,
) -> Optional[float]:
    if not isinstance(stats, Mapping):
        return None
    for key in keys:
        value = stats.get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) < 0
        ):
            continue
        return float(value)
    return None


def _server_decode_tokens_per_second(stats: object) -> Optional[float]:
    return _nonnegative_finite_stat(
        stats,
        "tokens_per_second",
        "tokensPerSecond",
    )


def _server_time_to_first_token_seconds(stats: object) -> Optional[float]:
    return _nonnegative_finite_stat(
        stats,
        "time_to_first_token",
        "time_to_first_token_sec",
        "time_to_first_token_seconds",
        "timeToFirstTokenSec",
    )


def _failure(
    *,
    code: str,
    message: str,
    case_id: str,
    repetition: int,
    warmup: bool,
) -> Mapping[str, object]:
    return {
        "code": code,
        "message": message,
        "case_id": case_id,
        "repetition": repetition,
        "warmup": warmup,
    }


class NavigationPlannerBenchmark:
    """Run a fixed, hardware-free suite against one exact listed model."""

    def __init__(
        self,
        *,
        base_url: str,
        model: str,
        repetitions: int = DEFAULT_REPETITIONS,
        parallelism: int = DEFAULT_PARALLELISM,
        timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
        inference_api: str = DEFAULT_INFERENCE_API,
        transport: BenchmarkTransport = _direct_transport,
        clock: Callable[[], float] = time.monotonic,
        cases: Optional[Sequence[BenchmarkCase]] = None,
    ):
        self.base_url = validate_benchmark_base_url(base_url)
        self.model = validate_model_id(model)
        if (
            isinstance(repetitions, bool)
            or not isinstance(repetitions, int)
            or not 1 <= repetitions <= 10
        ):
            raise NavigationBenchmarkError(
                "invalid_repetitions",
                "Measured repetitions must be between 1 and 10",
            )
        if (
            isinstance(parallelism, bool)
            or not isinstance(parallelism, int)
            or not 1 <= parallelism <= 4
        ):
            raise NavigationBenchmarkError(
                "invalid_parallelism",
                "Parallelism must be between 1 and 4",
            )
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not math.isfinite(float(timeout_seconds))
            or not 0.5 <= float(timeout_seconds) <= 60
        ):
            raise NavigationBenchmarkError(
                "invalid_timeout",
                "Request timeout must be between 0.5 and 60 seconds",
            )
        if not callable(transport) or not callable(clock):
            raise NavigationBenchmarkError(
                "invalid_dependency",
                "Benchmark dependency is invalid",
            )
        if (
            not isinstance(inference_api, str)
            or inference_api not in INFERENCE_API_PATHS
        ):
            raise NavigationBenchmarkError(
                "invalid_inference_api",
                "Inference API must be one of: {}".format(
                    ", ".join(sorted(INFERENCE_API_PATHS))
                ),
            )
        selected_cases = tuple(cases or benchmark_cases())
        if (
            not selected_cases
            or any(not isinstance(item, BenchmarkCase) for item in selected_cases)
            or len({item.case_id for item in selected_cases})
            != len(selected_cases)
        ):
            raise NavigationBenchmarkError(
                "invalid_cases",
                "Benchmark cases are invalid",
            )
        self.repetitions = repetitions
        self.parallelism = parallelism
        self.timeout_seconds = float(timeout_seconds)
        self.inference_api = inference_api
        self.inference_path = INFERENCE_API_PATHS[inference_api]
        self.transport = transport
        self.clock = clock
        self.cases = selected_cases

    def _probe_exact_model(self) -> None:
        raw = self.transport(
            "GET",
            self.base_url + "/v1/models",
            None,
            {"Accept": "application/json"},
            self.timeout_seconds,
            MAX_MODELS_RESPONSE_BYTES,
        )
        try:
            decoded = strict_json_loads(raw, MAX_MODELS_RESPONSE_BYTES)
            data = decoded["data"]
            if not isinstance(decoded, dict) or not isinstance(data, list):
                raise KeyError
            identifiers = []
            for item in data:
                if not isinstance(item, dict):
                    raise KeyError
                identifiers.append(validate_model_id(item["id"]))
            if len(identifiers) != len(set(identifiers)):
                raise KeyError
        except (
            KeyError,
            TypeError,
            NavigationBenchmarkError,
            PhysicalNavigationContractError,
        ):
            raise NavigationBenchmarkError(
                "invalid_model_list",
                "LM Studio returned an invalid model list",
            ) from None
        if self.model not in identifiers:
            raise NavigationBenchmarkError(
                "model_identity_not_listed",
                "The exact requested model ID is not listed by LM Studio",
            )

    def _planner(self) -> LMStudioNavigationPlanner:
        def inference_transport(url, body, headers, timeout, maximum):
            return self.transport(
                "POST",
                url,
                body,
                headers,
                timeout,
                maximum,
            )

        return LMStudioNavigationPlanner(
            base_url=self.base_url,
            model=self.model,
            transport=inference_transport,
            timeout_seconds=self.timeout_seconds,
            allow_private_lan=True,
            inference_path=self.inference_path,
        )

    def _attempt(
        self,
        case: BenchmarkCase,
        repetition: int,
        *,
        warmup: bool,
    ) -> Mapping[str, object]:
        started = self.clock()
        try:
            result = self._planner().decide(**case.planner_arguments())
        except LMStudioNavigationError as error:
            finished = self.clock()
            elapsed_ms = max(
                0,
                int(round((finished - started) * 1000)),
            )
            message = str(error)
            code = (
                "inference_transport_failed"
                if "request failed" in message
                else "invalid_navigation_decision"
            )
            return {
                "case_id": case.case_id,
                "repetition": repetition,
                "warmup": warmup,
                "wall_latency_ms": elapsed_ms,
                "schema_valid": False,
                "action": None,
                "expected_action": case.expected_action,
                "expected_action_agreement": False,
                "served_model": None,
                "served_model_exact_match": False,
                "completion_tokens": None,
                "end_to_end_completion_tokens_per_second": None,
                "server_decode_tokens_per_second": None,
                "server_time_to_first_token_seconds": None,
                "server_stats_present": False,
                "failures": [
                    _failure(
                        code=code,
                        message=message,
                        case_id=case.case_id,
                        repetition=repetition,
                        warmup=warmup,
                    )
                ],
                "_started_at_seconds": started,
                "_finished_at_seconds": finished,
            }
        finished = self.clock()
        elapsed_ms = max(
            0,
            int(round((finished - started) * 1000)),
        )
        decision = result.decision
        agreement = decision.action == case.expected_action
        served_match = result.served_model == self.model
        failures = []
        if not served_match:
            failures.append(
                _failure(
                    code="served_model_identity_mismatch",
                    message="Inference did not report the exact requested model ID",
                    case_id=case.case_id,
                    repetition=repetition,
                    warmup=warmup,
                )
            )
        if not agreement:
            failures.append(
                _failure(
                    code="unexpected_semantic_action",
                    message="Decision did not match the case's expected semantic action",
                    case_id=case.case_id,
                    repetition=repetition,
                    warmup=warmup,
                )
            )
        tokens = _completion_tokens(result.usage)
        end_to_end_tokens_per_second = (
            None
            if tokens is None or elapsed_ms <= 0
            else round(tokens * 1000.0 / elapsed_ms, 3)
        )
        server_decode_tokens_per_second = (
            _server_decode_tokens_per_second(result.stats)
        )
        server_time_to_first_token_seconds = (
            _server_time_to_first_token_seconds(result.stats)
        )
        return {
            "case_id": case.case_id,
            "repetition": repetition,
            "warmup": warmup,
            "wall_latency_ms": elapsed_ms,
            "schema_valid": True,
            "action": decision.action,
            "expected_action": case.expected_action,
            "expected_action_agreement": agreement,
            "served_model": result.served_model,
            "served_model_exact_match": served_match,
            "completion_tokens": tokens,
            "end_to_end_completion_tokens_per_second": (
                end_to_end_tokens_per_second
            ),
            "server_decode_tokens_per_second": (
                server_decode_tokens_per_second
            ),
            "server_time_to_first_token_seconds": (
                server_time_to_first_token_seconds
            ),
            "server_stats_present": isinstance(result.stats, Mapping),
            "failures": failures,
            "_started_at_seconds": started,
            "_finished_at_seconds": finished,
        }

    @staticmethod
    def _summary(samples: Sequence[Mapping[str, object]]) -> Mapping[str, object]:
        count = len(samples)
        schema_valid = sum(item["schema_valid"] is True for item in samples)
        agreement = sum(
            item["expected_action_agreement"] is True for item in samples
        )
        served_match = sum(
            item["served_model_exact_match"] is True for item in samples
        )
        failures = [
            failure
            for item in samples
            for failure in item["failures"]
        ]
        served_models = Counter(
            item["served_model"]
            for item in samples
            if isinstance(item["served_model"], str)
        )
        actions = Counter(
            item["action"]
            for item in samples
            if isinstance(item["action"], str)
        )
        token_rates = [
            item["end_to_end_completion_tokens_per_second"]
            for item in samples
            if item["end_to_end_completion_tokens_per_second"] is not None
        ]
        server_decode_rates = [
            item["server_decode_tokens_per_second"]
            for item in samples
            if item["server_decode_tokens_per_second"] is not None
        ]
        server_ttft_seconds = [
            item["server_time_to_first_token_seconds"]
            for item in samples
            if item["server_time_to_first_token_seconds"] is not None
        ]
        server_stats_present = sum(
            item["server_stats_present"] is True for item in samples
        )
        return {
            "sample_count": count,
            "first_pass_schema_validity": {
                "valid_count": schema_valid,
                "invalid_count": count - schema_valid,
                "rate": _rate(schema_valid, count),
            },
            "expected_semantic_action_agreement": {
                "agreement_count": agreement,
                "disagreement_count": count - agreement,
                "rate": _rate(agreement, count),
            },
            "served_model_exact_match": {
                "match_count": served_match,
                "mismatch_count": count - served_match,
                "rate": _rate(served_match, count),
            },
            "wall_latency": _distribution(
                [item["wall_latency_ms"] for item in samples],
                unit="ms",
            ),
            "end_to_end_completion_tokens_per_second": _distribution(
                token_rates,
                unit="tokens_per_second",
            ),
            "server_decode_tokens_per_second": _distribution(
                server_decode_rates,
                unit="tokens_per_second",
            ),
            "server_time_to_first_token_seconds": _distribution(
                server_ttft_seconds,
                unit="seconds",
            ),
            "server_stats": {
                "present_count": server_stats_present,
                "missing_count": count - server_stats_present,
            },
            "actions": dict(sorted(actions.items())),
            "served_models": dict(sorted(served_models.items())),
            "failure_count": len(failures),
            "failures": failures,
        }

    @staticmethod
    def _measured_phase(
        samples: Sequence[Mapping[str, object]],
    ) -> Mapping[str, object]:
        if not samples:
            return {
                "makespan_ms": 0,
                "completion_token_sample_count": 0,
                "missing_completion_token_sample_count": 0,
                "completion_tokens_total": 0,
                "aggregate_completion_tokens_per_end_to_end_second": None,
            }
        started = min(float(item["_started_at_seconds"]) for item in samples)
        finished = max(float(item["_finished_at_seconds"]) for item in samples)
        makespan_ms = max(0, int(round((finished - started) * 1000)))
        known_tokens = [
            item["completion_tokens"]
            for item in samples
            if isinstance(item["completion_tokens"], int)
            and not isinstance(item["completion_tokens"], bool)
        ]
        missing_count = len(samples) - len(known_tokens)
        token_total = sum(known_tokens) if not missing_count else None
        aggregate_rate = (
            None
            if token_total is None or makespan_ms <= 0
            else round(token_total * 1000.0 / makespan_ms, 3)
        )
        return {
            "makespan_ms": makespan_ms,
            "completion_token_sample_count": len(known_tokens),
            "missing_completion_token_sample_count": missing_count,
            "completion_tokens_total": token_total,
            "aggregate_completion_tokens_per_end_to_end_second": (
                aggregate_rate
            ),
        }

    @staticmethod
    def _public_samples(
        samples: Sequence[Mapping[str, object]],
    ) -> Sequence[Mapping[str, object]]:
        if not samples:
            return []
        phase_started = min(
            float(item["_started_at_seconds"]) for item in samples
        )
        public = []
        for item in samples:
            sanitized = {
                key: value
                for key, value in item.items()
                if not key.startswith("_")
            }
            sanitized["measured_phase_start_offset_ms"] = max(
                0,
                int(
                    round(
                        (
                            float(item["_started_at_seconds"])
                            - phase_started
                        )
                        * 1000
                    )
                ),
            )
            sanitized["measured_phase_finish_offset_ms"] = max(
                0,
                int(
                    round(
                        (
                            float(item["_finished_at_seconds"])
                            - phase_started
                        )
                        * 1000
                    )
                ),
            )
            public.append(sanitized)
        return public

    def run(self) -> Mapping[str, object]:
        # Identity is established before even the excluded warmup may POST.
        self._probe_exact_model()

        warmups = [
            self._attempt(case, 0, warmup=True)
            for case in self.cases
        ]
        work = [
            (case_index, repetition, case)
            for repetition in range(1, self.repetitions + 1)
            for case_index, case in enumerate(self.cases)
        ]
        measured = []
        if self.parallelism == 1:
            measured = [
                self._attempt(case, repetition, warmup=False)
                for _case_index, repetition, case in work
            ]
        else:
            with ThreadPoolExecutor(
                max_workers=self.parallelism,
                thread_name_prefix="navigation-benchmark",
            ) as executor:
                pending = {
                    executor.submit(
                        self._attempt,
                        case,
                        repetition,
                        warmup=False,
                    ): (case_index, repetition)
                    for case_index, repetition, case in work
                }
                completed = []
                for future in as_completed(pending):
                    case_index, repetition = pending[future]
                    completed.append(
                        (case_index, repetition, future.result())
                    )
                measured = [
                    result
                    for _case_index, _repetition, result in sorted(completed)
                ]

        case_reports = []
        for case in self.cases:
            samples = [
                item for item in measured if item["case_id"] == case.case_id
            ]
            report = {
                "case_id": case.case_id,
                "locale": case.locale,
                "expected_action": case.expected_action,
            }
            report.update(self._summary(samples))
            case_reports.append(report)

        parsed = urlsplit(self.base_url)
        port = parsed.port or (443 if parsed.scheme == "https" else 80)
        warmup_failures = [
            failure
            for item in warmups
            for failure in item["failures"]
        ]
        overall = self._summary(measured)
        return {
            "schema": BENCHMARK_SCHEMA,
            "scope": {
                "planner": "physical_navigation",
                "ev3_contact": False,
                "motor_api_exposed": False,
            },
            "endpoint": {
                "base_url": self.base_url,
                "host": parsed.hostname,
                "port": port,
                "scheme": parsed.scheme,
            },
            "requested_model": self.model,
            "served_models": overall["served_models"],
            "model_identity_probe": {
                "path": "/v1/models",
                "exact_requested_id_listed": True,
            },
            "configuration": {
                "case_count": len(self.cases),
                "locales": sorted({case.locale for case in self.cases}),
                "warmup_per_case": WARMUP_PER_CASE,
                "measured_repetitions_per_case": self.repetitions,
                "parallelism": self.parallelism,
                "timeout_seconds": self.timeout_seconds,
                "inference_api": self.inference_api,
                "inference_path": self.inference_path,
            },
            "warmup": {
                "sample_count": len(warmups),
                "excluded_from_metrics": True,
                "failure_count": len(warmup_failures),
                "failures": warmup_failures,
            },
            "cases": case_reports,
            "measured_phase": self._measured_phase(measured),
            "samples": self._public_samples(measured),
            "overall": overall,
        }


def _bounded_integer(name: str, minimum: int, maximum: int):
    def parse(value: str) -> int:
        try:
            parsed = int(value)
        except ValueError:
            raise argparse.ArgumentTypeError(
                "{} must be an integer".format(name)
            ) from None
        if not minimum <= parsed <= maximum:
            raise argparse.ArgumentTypeError(
                "{} must be between {} and {}".format(
                    name,
                    minimum,
                    maximum,
                )
            )
        return parsed

    return parse


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Benchmark the exact structured physical-navigation planner "
            "without contacting an EV3."
        )
    )
    parser.add_argument(
        "--base-url",
        required=True,
        help=(
            "Explicit numeric loopback or private-LAN LM Studio base URL"
        ),
    )
    parser.add_argument(
        "--model",
        required=True,
        help="Exact model ID that must appear in GET /v1/models",
    )
    parser.add_argument(
        "--repetitions",
        type=_bounded_integer("repetitions", 1, 10),
        default=DEFAULT_REPETITIONS,
        help="Measured repetitions per case, 1-10 (default: %(default)s)",
    )
    parser.add_argument(
        "--parallelism",
        type=_bounded_integer("parallelism", 1, 4),
        default=DEFAULT_PARALLELISM,
        help="Maximum concurrent inference calls, 1-4 (default: %(default)s)",
    )
    parser.add_argument(
        "--inference-api",
        choices=sorted(INFERENCE_API_PATHS),
        default=DEFAULT_INFERENCE_API,
        help=(
            "Chat-completions API: lmstudio-v0 preserves server timing "
            "stats; openai-v1 is the compatibility fallback "
            "(default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=DEFAULT_TIMEOUT_SECONDS,
        help="Per-request deadline, 0.5-60 seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--pretty",
        action="store_true",
        help="Indent the JSON report",
    )
    return parser


def _run(
    argv: Optional[Sequence[str]] = None,
    *,
    transport: BenchmarkTransport = _direct_transport,
    output_stream=None,
    error_stream=None,
) -> int:
    output_stream = output_stream or sys.stdout
    error_stream = error_stream or sys.stderr
    args = build_parser().parse_args(argv)
    try:
        report = NavigationPlannerBenchmark(
            base_url=args.base_url,
            model=args.model,
            repetitions=args.repetitions,
            parallelism=args.parallelism,
            timeout_seconds=args.timeout_seconds,
            inference_api=args.inference_api,
            transport=transport,
        ).run()
    except NavigationBenchmarkError as error:
        print(
            json.dumps(
                {
                    "schema": ERROR_SCHEMA,
                    "status": "failed",
                    "error": {
                        "code": error.code,
                        "message": str(error),
                    },
                },
                ensure_ascii=False,
                sort_keys=True,
            ),
            file=error_stream,
        )
        return 2
    print(
        json.dumps(
            report,
            ensure_ascii=False,
            sort_keys=True,
            indent=2 if args.pretty else None,
            separators=None if args.pretty else (",", ":"),
        ),
        file=output_stream,
    )
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    return _run(argv)


if __name__ == "__main__":
    raise SystemExit(main())
