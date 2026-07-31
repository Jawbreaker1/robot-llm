import json
import threading
import unittest

from robot_agent.lm_studio_navigation import (
    LMStudioNavigationError,
    LMStudioNavigationPlanner,
    SYSTEM_PROMPT,
)
from robot_agent.lm_studio_navigation_benchmark_cli import (
    BENCHMARK_SCHEMA,
    NavigationBenchmarkError,
    NavigationPlannerBenchmark,
    validate_benchmark_base_url,
)
from robot_agent.maneuver_commitment import empty_commitment
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    DECISION_SCHEMA,
    FINISH,
    SCAN_FRONT_ARC,
)


MODEL_ID = "mlx-community/gemma-robot-planner-4bit"


def _decision_for_payload(body):
    request = json.loads(body.decode("utf-8"))
    context = json.loads(request["messages"][1]["content"])
    episode_id = context["episode_id"]
    if episode_id == "benchmark-clear-progress-en":
        action = ADVANCE
        plan = [ADVANCE, ADVANCE]
        reason = "PROGRESS_GOAL"
        target = None
    elif episode_id == "benchmark-blocked-scan-sv":
        action = SCAN_FRONT_ARC
        plan = [SCAN_FRONT_ARC]
        reason = "PROBE_CLEARANCE"
        target = "bench-hazard-01"
    elif episode_id == "benchmark-completed-finish-en":
        action = FINISH
        plan = [FINISH]
        reason = "COMPLETE_GOAL"
        target = None
    else:
        raise AssertionError("unexpected benchmark episode")
    return request, {
        "schema": DECISION_SCHEMA,
        "episode_id": episode_id,
        "turn": context["turn"],
        "based_on_state_version": context["observation"]["state_version"],
        "action": action,
        "plan": plan,
        "reason_code": reason,
        "assessment": "The action follows the published mission facts.",
        "utterance": None,
        "perception_target_hypothesis_id": target,
        "maneuver_commitment": empty_commitment(),
    }


class FakeLMStudioTransport:
    def __init__(
        self,
        *,
        listed_model=MODEL_ID,
        malformed_post_indexes=(),
        served_model=MODEL_ID,
        completion_tokens=20,
        include_stats=True,
        server_tokens_per_second=97.5,
        server_time_to_first_token=0.125,
    ):
        self.listed_model = listed_model
        self.malformed_post_indexes = set(malformed_post_indexes)
        self.served_model = served_model
        self.completion_tokens = completion_tokens
        self.include_stats = include_stats
        self.server_tokens_per_second = server_tokens_per_second
        self.server_time_to_first_token = server_time_to_first_token
        self.calls = []
        self.payloads = []
        self._post_count = 0
        self._lock = threading.Lock()

    def __call__(self, method, url, body, headers, timeout, maximum):
        with self._lock:
            self.calls.append((method, url, body, headers, timeout, maximum))
            if method == "GET":
                return json.dumps(
                    {
                        "object": "list",
                        "data": [
                            {
                                "id": self.listed_model,
                                "object": "model",
                            }
                        ],
                    }
                ).encode("utf-8")
            self._post_count += 1
            post_index = self._post_count
        request, decision = _decision_for_payload(body)
        with self._lock:
            self.payloads.append(request)
        content = (
            "not-json"
            if post_index in self.malformed_post_indexes
            else json.dumps(decision)
        )
        response = {
            "model": self.served_model,
            "choices": [{"message": {"content": content}}],
            "usage": {"completion_tokens": self.completion_tokens},
        }
        if self.include_stats:
            response["stats"] = {
                "tokens_per_second": self.server_tokens_per_second,
                "time_to_first_token": self.server_time_to_first_token,
                "stop_reason": "eosFound",
            }
        return json.dumps(response).encode("utf-8")


class DurationClock:
    """Return one deterministic start/end pair for each benchmark attempt."""

    def __init__(self, durations_ms):
        values = []
        current = 0.0
        for duration_ms in durations_ms:
            values.append(current)
            current += duration_ms / 1000.0
            values.append(current)
        self._values = iter(values)

    def __call__(self):
        return next(self._values)


class NavigationBenchmarkEndpointTests(unittest.TestCase):
    def test_accepts_only_numeric_loopback_rfc1918_and_ipv6_ula(self):
        accepted = (
            "http://127.0.0.1:1234",
            "http://10.4.5.6:1234/",
            "https://172.16.0.1:443",
            "http://172.31.255.254:1234",
            "http://192.168.50.9:1234",
            "http://[::1]:1234",
            "http://[fd12:3456::9]:1234",
        )
        for value in accepted:
            with self.subTest(value=value):
                self.assertEqual(
                    validate_benchmark_base_url(value),
                    value.rstrip("/"),
                )

        rejected = (
            "http://localhost:1234",
            "http://example.com:1234",
            "http://8.8.8.8:1234",
            "http://169.254.1.2:1234",
            "http://100.64.1.2:1234",
            "http://0.0.0.0:1234",
            "http://user:pass@192.168.1.2:1234",
            "http://192.168.1.2:1234/v1",
            "http://192.168.1.2:1234?model=x",
        )
        for value in rejected:
            with self.subTest(value=value):
                with self.assertRaises(NavigationBenchmarkError):
                    validate_benchmark_base_url(value)

    def test_private_lan_is_explicit_and_planner_default_stays_loopback_only(self):
        with self.assertRaises(LMStudioNavigationError):
            LMStudioNavigationPlanner(
                base_url="http://192.168.1.50:1234",
                model=MODEL_ID,
            )
        planner = LMStudioNavigationPlanner(
            base_url="http://192.168.1.50:1234",
            model=MODEL_ID,
            allow_private_lan=True,
        )
        self.assertEqual(planner.base_url, "http://192.168.1.50:1234")

    def test_wrong_identity_prevents_every_inference_post(self):
        transport = FakeLMStudioTransport(listed_model="different-model")
        benchmark = NavigationPlannerBenchmark(
            base_url="http://192.168.1.50:1234",
            model=MODEL_ID,
            repetitions=1,
            transport=transport,
        )
        with self.assertRaises(NavigationBenchmarkError) as caught:
            benchmark.run()
        self.assertEqual(caught.exception.code, "model_identity_not_listed")
        self.assertEqual(
            [(method, url) for method, url, *_rest in transport.calls],
            [("GET", "http://192.168.1.50:1234/v1/models")],
        )


class NavigationBenchmarkReportTests(unittest.TestCase):
    def test_uses_production_prompt_schema_and_validation(self):
        transport = FakeLMStudioTransport()
        benchmark = NavigationPlannerBenchmark(
            base_url="http://192.168.1.50:1234",
            model=MODEL_ID,
            repetitions=1,
            transport=transport,
        )
        report = benchmark.run()

        self.assertEqual(report["schema"], BENCHMARK_SCHEMA)
        self.assertFalse(report["scope"]["ev3_contact"])
        self.assertFalse(report["scope"]["motor_api_exposed"])
        self.assertEqual(transport.calls[0][0], "GET")
        self.assertTrue(all(call[0] == "POST" for call in transport.calls[1:]))
        self.assertTrue(
            all(
                call[1].endswith("/api/v0/chat/completions")
                for call in transport.calls[1:]
            )
        )
        self.assertEqual(len(transport.payloads), 6)
        first = transport.payloads[0]
        self.assertEqual(first["model"], MODEL_ID)
        self.assertEqual(first["messages"][0]["content"], SYSTEM_PROMPT)
        self.assertTrue(first["response_format"]["json_schema"]["strict"])
        self.assertEqual(
            first["response_format"]["json_schema"]["name"],
            "physical_navigation_decision",
        )
        self.assertNotIn("tools", first)
        self.assertEqual(
            report["overall"]["first_pass_schema_validity"]["valid_count"],
            3,
        )
        self.assertEqual(
            report["overall"]["expected_semantic_action_agreement"]["rate"],
            1.0,
        )

    def test_malformed_output_is_counted_without_retry(self):
        # POSTs 1-3 are excluded warmups; POST 4 is the first measured case.
        transport = FakeLMStudioTransport(malformed_post_indexes=(4,))
        benchmark = NavigationPlannerBenchmark(
            base_url="http://10.0.0.8:1234",
            model=MODEL_ID,
            repetitions=2,
            transport=transport,
        )
        report = benchmark.run()

        self.assertEqual(len(transport.payloads), 9)
        validity = report["overall"]["first_pass_schema_validity"]
        self.assertEqual(validity["valid_count"], 5)
        self.assertEqual(validity["invalid_count"], 1)
        self.assertEqual(report["overall"]["failure_count"], 1)
        self.assertEqual(
            report["overall"]["failures"][0]["code"],
            "invalid_navigation_decision",
        )
        clear = next(
            item
            for item in report["cases"]
            if item["case_id"] == "clear_progress_en"
        )
        self.assertEqual(
            clear["first_pass_schema_validity"],
            {"valid_count": 1, "invalid_count": 1, "rate": 0.5},
        )

    def test_warmup_is_excluded_and_metrics_are_deterministic(self):
        transport = FakeLMStudioTransport(completion_tokens=20)
        clock = DurationClock(
            [999, 999, 999, 100, 200, 300, 400, 500, 600]
        )
        report = NavigationPlannerBenchmark(
            base_url="http://127.0.0.1:1234",
            model=MODEL_ID,
            repetitions=2,
            parallelism=1,
            transport=transport,
            clock=clock,
        ).run()

        self.assertEqual(
            report["warmup"],
            {
                "sample_count": 3,
                "excluded_from_metrics": True,
                "failure_count": 0,
                "failures": [],
            },
        )
        self.assertEqual(
            report["overall"]["wall_latency"],
            {"sample_count": 6, "median_ms": 350, "p95_ms": 600},
        )
        self.assertEqual(
            report["overall"][
                "end_to_end_completion_tokens_per_second"
            ],
            {
                "sample_count": 6,
                "median_tokens_per_second": 58.334,
                "p95_tokens_per_second": 200,
            },
        )
        self.assertEqual(
            report["overall"]["server_decode_tokens_per_second"],
            {
                "sample_count": 6,
                "median_tokens_per_second": 97.5,
                "p95_tokens_per_second": 97.5,
            },
        )
        self.assertEqual(
            report["overall"]["server_time_to_first_token_seconds"],
            {
                "sample_count": 6,
                "median_seconds": 0.125,
                "p95_seconds": 0.125,
            },
        )
        self.assertEqual(
            report["measured_phase"],
            {
                "makespan_ms": 2100,
                "completion_token_sample_count": 6,
                "missing_completion_token_sample_count": 0,
                "completion_tokens_total": 120,
                "aggregate_completion_tokens_per_end_to_end_second": 57.143,
            },
        )
        self.assertEqual(len(report["samples"]), 6)
        self.assertEqual(
            report["samples"][0]["measured_phase_start_offset_ms"],
            0,
        )
        self.assertEqual(
            report["samples"][-1]["measured_phase_finish_offset_ms"],
            2100,
        )
        clear = report["cases"][0]
        self.assertEqual(
            clear["wall_latency"],
            {"sample_count": 2, "median_ms": 250, "p95_ms": 400},
        )
        self.assertEqual(report["served_models"], {MODEL_ID: 6})

    def test_parallelism_is_bounded_and_report_order_is_stable(self):
        transport = FakeLMStudioTransport()
        report = NavigationPlannerBenchmark(
            base_url="http://10.0.0.8:1234",
            model=MODEL_ID,
            repetitions=2,
            parallelism=4,
            transport=transport,
        ).run()
        self.assertEqual(
            [item["case_id"] for item in report["cases"]],
            [
                "clear_progress_en",
                "blocked_requires_scan_sv",
                "completed_finish_en",
            ],
        )
        self.assertEqual(report["overall"]["sample_count"], 6)
        with self.assertRaises(NavigationBenchmarkError):
            NavigationPlannerBenchmark(
                base_url="http://10.0.0.8:1234",
                model=MODEL_ID,
                parallelism=5,
                transport=transport,
            )

    def test_openai_v1_fallback_reports_missing_server_stats(self):
        transport = FakeLMStudioTransport(include_stats=False)
        report = NavigationPlannerBenchmark(
            base_url="http://127.0.0.1:1234",
            model=MODEL_ID,
            repetitions=1,
            inference_api="openai-v1",
            transport=transport,
        ).run()

        self.assertTrue(
            all(
                call[1].endswith("/v1/chat/completions")
                for call in transport.calls
                if call[0] == "POST"
            )
        )
        self.assertEqual(
            report["configuration"]["inference_path"],
            "/v1/chat/completions",
        )
        self.assertEqual(
            report["overall"]["server_stats"],
            {"present_count": 0, "missing_count": 3},
        )
        self.assertEqual(
            report["overall"]["server_decode_tokens_per_second"],
            {
                "sample_count": 0,
                "median_tokens_per_second": None,
                "p95_tokens_per_second": None,
            },
        )

        with self.assertRaises(NavigationBenchmarkError) as caught:
            NavigationPlannerBenchmark(
                base_url="http://127.0.0.1:1234",
                model=MODEL_ID,
                inference_api="unknown",
                transport=transport,
            )
        self.assertEqual(caught.exception.code, "invalid_inference_api")


if __name__ == "__main__":
    unittest.main()
