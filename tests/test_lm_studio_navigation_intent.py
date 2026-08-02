import copy
import json
import unittest

from robot_agent.lm_studio import LMStudioHTTPError
from robot_agent.lm_studio_endpoint import (
    LM_STUDIO_V0_CHAT_COMPLETIONS_PATH,
)
from robot_agent.lm_studio_navigation_intent import (
    LMStudioNavigationIntentClient,
    LMStudioNavigationIntentError,
    MAX_OUTPUT_TOKENS,
    MAX_RESPONSE_BYTES,
)
from robot_agent.navigation_intent_context import (
    NAVIGATION_INTENT_WRAPPER_RESERVE_BYTES,
    SYSTEM_PROMPT,
    NavigationIntentPrompt,
)
from robot_agent.navigation_intent_proposal import (
    FOLLOW_DIRECTION,
    MAX_NAVIGATION_INTENT_TTL_MS,
    NavigationIntentOffer,
    NavigationIntentProposalError,
    SCAN_TARGET,
    build_navigation_intent_proposal_schema,
)
from robot_agent.physical_agent_state import ControllerKey, NavigationBasis


MODEL = "google/gemma-4-26b-a4b-qat"


def json_bytes(value):
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def basis():
    return NavigationBasis(
        controller_key=ControllerKey(
            robot_id="robot-a",
            controller_id="ev3-a",
            controller_instance_id="ev3-boot-a",
        ),
        goal_epoch=3,
        controller_state_version=11,
        world_generation_id="world-a",
        world_model_version=7,
        navigation_basis_id="nav-basis-a",
        frame_id="frame-a",
        calibration_fingerprint="calibration-a",
    )


def offer(**changes):
    values = {
        "ticket_id": "host-ticket-secret",
        "basis": basis(),
        "offered_intents": (FOLLOW_DIRECTION,),
    }
    values.update(changes)
    return NavigationIntentOffer(**values)


def context():
    return {
        "objective": "Continue through the room.",
        "locale": "en",
        "mission": {
            "current_longitudinal_progress_mm": 20,
            "remaining_longitudinal_progress_mm": 400,
            "regression_from_peak_mm": 0,
            "lateral_offset_mm": 0,
            "goal_heading_aligned": True,
            "goal_corridor_clear": True,
            "all_known_hazards_passed": True,
            "localization_valid": True,
            "touch_clear": True,
            "completed": False,
        },
        "pose": {"x_mm": 20, "y_mm": 0, "heading_mdeg": 0},
        "known_hazard_count": 0,
        "active_intent": None,
        "intent_progress": None,
        "offered_target_evidence": [],
        "latest_outcome": None,
    }


def prompt(*, prompt_context=None, response_schema=None, size_delta=0):
    current_offer = offer()
    value = copy.deepcopy(prompt_context or context())
    schema = copy.deepcopy(
        response_schema
        if response_schema is not None
        else build_navigation_intent_proposal_schema(current_offer)
    )
    context_size = len(json_bytes(value))
    accounted = (
        len(SYSTEM_PROMPT.encode("utf-8"))
        + context_size
        + len(json_bytes(schema))
        + NAVIGATION_INTENT_WRAPPER_RESERVE_BYTES
    )
    return NavigationIntentPrompt(
        system_prompt=SYSTEM_PROMPT,
        context=value,
        response_schema=schema,
        context_bytes=context_size + size_delta,
        accounted_bytes=accounted + size_delta,
    )


def response(
    *,
    model=MODEL,
    content=None,
    completion_tokens=7,
    reasoning_tokens=0,
):
    if content is None:
        content = json.dumps({"intent": FOLLOW_DIRECTION})
    return json.dumps({
        "model": model,
        "choices": [{
            "message": {"role": "assistant", "content": content},
        }],
        "usage": {
            "prompt_tokens": 321,
            "completion_tokens": completion_tokens,
            "total_tokens": 321 + completion_tokens,
        },
        "stats": {
            "reasoning_output_tokens": reasoning_tokens,
            "tokens_per_second": 98.5,
            "time_to_first_token": 0.12,
        },
    }).encode("utf-8")


class CaptureTransport:
    def __init__(self, result):
        self.result = result
        self.calls = []

    def __call__(self, url, body, headers, timeout, maximum):
        self.calls.append((url, body, headers, timeout, maximum))
        if isinstance(self.result, BaseException):
            raise self.result
        return self.result


def client(
    transport,
    *,
    times=(4.0, 4.025),
    unix_ms=lambda: 10_000,
    **changes
):
    values = {
        "base_url": "http://127.0.0.1:1234",
        "model": MODEL,
        "transport": transport,
        "monotonic": lambda: next(iter_times),
        "unix_ms": unix_ms,
        "proposal_ttl_ms": 5_000,
    }
    iter_times = iter(times)
    values.update(changes)
    return LMStudioNavigationIntentClient(**values)


class LMStudioNavigationIntentPayloadTests(unittest.TestCase):
    def test_sends_only_fixed_prompt_and_identity_free_context_then_binds_host(self):
        transport = CaptureTransport(response())
        current_offer = offer()
        current_prompt = prompt()
        value = client(transport).decide(
            current_prompt,
            offer=current_offer,
            proposal_id="proposal-host-a",
        )

        self.assertEqual(len(transport.calls), 1)
        url, body, headers, timeout, maximum = transport.calls[0]
        self.assertEqual(url, "http://127.0.0.1:1234/v1/chat/completions")
        self.assertEqual(timeout, 45.0)
        self.assertEqual(maximum, MAX_RESPONSE_BYTES)
        self.assertEqual(headers["Accept"], "application/json")
        payload = json.loads(body.decode("utf-8"))
        self.assertEqual(
            set(payload),
            {
                "model",
                "messages",
                "response_format",
                "temperature",
                "reasoning_effort",
                "max_tokens",
                "stream",
                "store",
            },
        )
        self.assertEqual(payload["model"], MODEL)
        self.assertEqual(payload["temperature"], 0)
        self.assertEqual(payload["reasoning_effort"], "none")
        self.assertEqual(payload["max_tokens"], MAX_OUTPUT_TOKENS)
        self.assertIs(payload["stream"], False)
        self.assertIs(payload["store"], False)
        self.assertEqual(
            payload["messages"],
            [
                {"role": "system", "content": SYSTEM_PROMPT},
                {
                    "role": "user",
                    "content": json_bytes(current_prompt.context).decode("utf-8"),
                },
            ],
        )
        messages = json.dumps(payload["messages"], sort_keys=True)
        for secret in (
            current_offer.ticket_id,
            current_offer.basis.controller_key.controller_id,
            current_offer.basis.navigation_basis_id,
        ):
            self.assertNotIn(secret, messages)
        self.assertNotIn("tools", payload)
        response_format = payload["response_format"]
        self.assertEqual(response_format["type"], "json_schema")
        self.assertIs(response_format["json_schema"]["strict"], True)
        self.assertEqual(
            response_format["json_schema"]["schema"],
            current_prompt.response_schema,
        )

        self.assertEqual(value.latency_ms, 25)
        self.assertEqual(value.served_model, MODEL)
        self.assertEqual(value.prompt_tokens, 321)
        self.assertEqual(value.completion_tokens, 7)
        self.assertEqual(value.total_tokens, 328)
        self.assertEqual(value.server_tokens_per_second, 98.5)
        self.assertEqual(value.server_time_to_first_token_seconds, 0.12)
        self.assertEqual(value.context_bytes, current_prompt.context_bytes)
        self.assertEqual(value.envelope.proposal.intent, FOLLOW_DIRECTION)
        self.assertEqual(value.envelope.proposal_id, "proposal-host-a")
        self.assertEqual(value.envelope.ticket_id, current_offer.ticket_id)
        self.assertEqual(value.envelope.basis, current_offer.basis)
        self.assertEqual(value.envelope.received_at_ms, 10_000)
        self.assertEqual(value.envelope.valid_until_ms, 15_000)

    def test_uses_the_other_allowed_lm_studio_chat_path(self):
        transport = CaptureTransport(response())
        value = client(
            transport,
            base_url="http://192.168.1.20:1234/",
            allow_private_lan=True,
            inference_path=LM_STUDIO_V0_CHAT_COMPLETIONS_PATH,
        )

        value.decide(prompt(), offer=offer(), proposal_id="proposal-a")

        self.assertEqual(
            transport.calls[0][0],
            "http://192.168.1.20:1234/api/v0/chat/completions",
        )


class LMStudioNavigationIntentValidationTests(unittest.TestCase):
    def test_rejects_unsafe_urls_paths_and_ttl(self):
        transport = CaptureTransport(response())
        cases = (
            {"base_url": "http://example.com:1234"},
            {"base_url": "http://192.168.1.20:1234"},
            {"inference_path": "/v1/not-chat"},
            {"inference_path": []},
            {"proposal_ttl_ms": 0},
            {"proposal_ttl_ms": MAX_NAVIGATION_INTENT_TTL_MS + 1},
        )
        for changes in cases:
            with self.subTest(changes=changes):
                with self.assertRaises(LMStudioNavigationIntentError) as caught:
                    client(transport, **changes)
                self.assertEqual(
                    caught.exception.code,
                    "intent_configuration_invalid",
                )

    def test_rejects_identity_leaks_offer_mismatch_and_false_size_accounting(self):
        leaked = context()
        leaked["mission"]["ticket_id"] = "must-not-leak"
        scan_offer = offer(
            offered_intents=(FOLLOW_DIRECTION, SCAN_TARGET),
            scan_target_ids=("hazard-a",),
        )
        cases = (
            (
                prompt(prompt_context=leaked),
                offer(),
                "intent_prompt_identity_leak",
            ),
            (
                prompt(response_schema={"oneOf": []}),
                offer(),
                "intent_prompt_offer_mismatch",
            ),
            (
                prompt(
                    response_schema=build_navigation_intent_proposal_schema(
                        scan_offer
                    )
                ),
                scan_offer,
                "intent_prompt_offer_mismatch",
            ),
            (prompt(size_delta=1), offer(), "intent_prompt_size_mismatch"),
        )
        for current_prompt, current_offer, code in cases:
            with self.subTest(code=code):
                transport = CaptureTransport(response())
                with self.assertRaises(LMStudioNavigationIntentError) as caught:
                    client(transport).decide(
                        current_prompt,
                        offer=current_offer,
                        proposal_id="proposal-a",
                    )
                self.assertEqual(caught.exception.code, code)
                self.assertEqual(transport.calls, [])

    def test_rejects_invalid_outer_and_inner_json(self):
        cases = (
            (b"{", "intent_response_invalid", None),
            (
                response(content="{"),
                "intent_proposal_invalid",
                "invalid_proposal_json",
            ),
            (
                response(content=json.dumps({
                    "intent": "ABORT",
                    "reason": "NOT_OFFERED",
                })),
                "intent_proposal_invalid",
                "unoffered_intent",
            ),
        )
        for raw, code, proposal_code in cases:
            with self.subTest(code=code, proposal_code=proposal_code):
                with self.assertRaises(LMStudioNavigationIntentError) as caught:
                    client(CaptureTransport(raw)).decide(
                        prompt(), offer=offer(), proposal_id="proposal-a"
                    )
                self.assertEqual(caught.exception.code, code)
                self.assertEqual(
                    caught.exception.proposal_error_code,
                    proposal_code,
                )

    def test_enforces_exact_served_model_reasoning_and_output_budget(self):
        cases = (
            (
                response(model="different-model"),
                "intent_served_model_mismatch",
            ),
            (
                response(reasoning_tokens=1),
                "intent_reasoning_policy_violated",
            ),
            (
                response(completion_tokens=MAX_OUTPUT_TOKENS + 1),
                "intent_output_budget_violated",
            ),
        )
        for raw, code in cases:
            with self.subTest(code=code):
                with self.assertRaises(LMStudioNavigationIntentError) as caught:
                    client(CaptureTransport(raw)).decide(
                        prompt(), offer=offer(), proposal_id="proposal-a"
                    )
                self.assertEqual(caught.exception.code, code)

    def test_reports_http_and_transport_failures_with_latency(self):
        cases = (
            (
                LMStudioHTTPError(503),
                "intent_http_error",
                503,
            ),
            (
                RuntimeError("offline"),
                "intent_transport_failed",
                None,
            ),
        )
        for failure, code, status in cases:
            with self.subTest(code=code):
                with self.assertRaises(LMStudioNavigationIntentError) as caught:
                    client(CaptureTransport(failure), times=(1.0, 1.017)).decide(
                        prompt(), offer=offer(), proposal_id="proposal-a"
                    )
                self.assertEqual(caught.exception.code, code)
                self.assertEqual(caught.exception.latency_ms, 17)
                self.assertEqual(caught.exception.http_status, status)

    def test_binds_bounded_ttl_and_expiry_stays_in_envelope_contract(self):
        current_offer = offer()
        value = client(
            CaptureTransport(response()),
            proposal_ttl_ms=MAX_NAVIGATION_INTENT_TTL_MS,
        ).decide(prompt(), offer=current_offer, proposal_id="proposal-a")

        value.envelope.assert_current(
            proposal_id="proposal-a",
            ticket_id=current_offer.ticket_id,
            basis=current_offer.basis,
            now_ms=10_000 + MAX_NAVIGATION_INTENT_TTL_MS - 1,
        )
        with self.assertRaises(NavigationIntentProposalError) as caught:
            value.envelope.assert_current(
                proposal_id="proposal-a",
                ticket_id=current_offer.ticket_id,
                basis=current_offer.basis,
                now_ms=10_000 + MAX_NAVIGATION_INTENT_TTL_MS,
            )
        self.assertEqual(caught.exception.code, "expired_proposal")

        with self.assertRaises(LMStudioNavigationIntentError) as failed_binding:
            client(
                CaptureTransport(response()),
                unix_ms=lambda: True,
            ).decide(prompt(), offer=current_offer, proposal_id="proposal-a")
        self.assertEqual(failed_binding.exception.code, "intent_binding_failed")


if __name__ == "__main__":
    unittest.main()
