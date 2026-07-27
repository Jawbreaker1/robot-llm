import json
import re
import socket
import unittest

from robot_agent.interaction_contract import (
    EXPRESSION_PROPOSAL_SCHEMA,
    InteractionSnapshot,
    ObjectEvidence,
    decode_expression_proposal,
)
from robot_agent.lm_studio import (
    LMStudioConfigurationError,
    LMStudioProtocolError,
    LMStudioResponseTooLargeError,
    LMStudioTimeoutError,
    LMStudioTransportError,
)
from robot_agent.lm_studio_expression import (
    CHAT_COMPLETIONS_PATH,
    MAX_EXPRESSION_PROPOSAL_BYTES,
    MAX_EXPRESSION_RESPONSE_BYTES,
    LMStudioExpressionPlanner,
)


MODEL = "local/expression-model"


def snapshot(object_id=None, response_locale="en-GB"):
    return InteractionSnapshot(
        robot_id="ev3rstorm-1",
        controller_instance_id="controller-7",
        goal_id="goal-waypoint-2",
        goal_epoch=3,
        plan_revision=4,
        interaction_state_version=11,
        world_model_version=8,
        captured_at_ms=10_000,
        obstruction_epoch=2,
        drive_phase="BLOCKED",
        response_locale=response_locale,
        evidence=ObjectEvidence(
            evidence_id="evidence-9",
            relation="BLOCKING_PATH",
            object_id=object_id,
            source="range-fusion",
            observed_at_ms=9_990,
            confidence_milli=940,
        ),
    )


def proposal_value(
    decision="EXPRESS",
    locale="en-GB",
    gesture_kind="PROPELLER_WAVE",
):
    value = {
        "schema": EXPRESSION_PROPOSAL_SCHEMA,
        "proposal_id": "expression-12",
        "robot_id": "ev3rstorm-1",
        "controller_instance_id": "controller-7",
        "goal_id": "goal-waypoint-2",
        "goal_epoch": 3,
        "plan_revision": 4,
        "based_on_interaction_state_version": 11,
        "based_on_world_model_version": 8,
        "obstruction_epoch": 2,
        "based_on_evidence_id": "evidence-9",
        "decision": decision,
        "confidence_milli": 880,
    }
    if decision == "EXPRESS":
        value["intent"] = {
            "utterance": "Oh splendid, another unidentified thing in my way.",
            "utterance_locale": locale,
            "gesture_kind": gesture_kind,
            "affect_label": "indignant",
            "intensity": 810,
            "repetitions": 0 if gesture_kind is None else 2,
        }
    else:
        value["reason_code"] = "expression_not_needed"
    return value


def completion(
    proposal=None,
    model=MODEL,
    finish_reason="stop",
    choices=None,
):
    if proposal is None:
        proposal = proposal_value()
    if choices is None:
        choices = [
            {
                "index": 0,
                "finish_reason": finish_reason,
                "message": {
                    "role": "assistant",
                    "content": json.dumps(proposal),
                },
            }
        ]
    return json.dumps(
        {
            "id": "chatcmpl-expression",
            "object": "chat.completion",
            "model": model,
            "choices": choices,
        }
    ).encode("utf-8")


class CapturingTransport:
    def __init__(self, response=None, error=None):
        self.response = completion() if response is None else response
        self.error = error
        self.calls = []

    def __call__(
        self,
        url,
        body,
        headers,
        timeout_seconds,
        max_response_bytes,
    ):
        self.calls.append(
            (
                url,
                body,
                headers,
                timeout_seconds,
                max_response_bytes,
            )
        )
        if self.error is not None:
            raise self.error
        return self.response


class Clock:
    def __init__(self, *values):
        self.values = iter(values or (0.0, 0.01))

    def __call__(self):
        return next(self.values)


def schema_branches(schema, decision):
    return [
        branch
        for branch in schema["oneOf"]
        if branch["properties"]["decision"]["const"] == decision
    ]


def schema_branch(schema, decision):
    matches = schema_branches(schema, decision)
    if len(matches) != 1:
        raise AssertionError(
            "expected one {} branch, found {}".format(
                decision,
                len(matches),
            )
        )
    return matches[0]


def collect_property_names(value):
    names = set()
    if isinstance(value, dict):
        properties = value.get("properties")
        if isinstance(properties, dict):
            names.update(properties)
        for nested in value.values():
            names.update(collect_property_names(nested))
    elif isinstance(value, list):
        for nested in value:
            names.update(collect_property_names(nested))
    return names


class LMStudioExpressionRequestTests(unittest.TestCase):
    def test_builds_exact_tool_free_strict_schema_bound_to_snapshot(self):
        transport = CapturingTransport()
        planner = LMStudioExpressionPlanner(
            response_locale="en-GB",
            base_url="http://127.0.0.1:1234/",
            model=MODEL,
            transport=transport,
            clock=Clock(),
            timeout_seconds=2.5,
        )

        result = planner(snapshot())

        self.assertEqual(len(transport.calls), 1)
        url, body, headers, timeout, maximum = transport.calls[0]
        self.assertEqual(
            url,
            "http://127.0.0.1:1234" + CHAT_COMPLETIONS_PATH,
        )
        self.assertEqual(headers["Accept"], "application/json")
        self.assertEqual(timeout, 2.5)
        self.assertEqual(maximum, MAX_EXPRESSION_RESPONSE_BYTES)

        request = json.loads(body)
        self.assertEqual(request["model"], MODEL)
        self.assertEqual(request["temperature"], 0)
        self.assertEqual(request["reasoning_effort"], "none")
        self.assertFalse(request["stream"])
        self.assertFalse(request["store"])
        self.assertNotIn("tools", request)
        self.assertNotIn("tool_choice", request)
        self.assertEqual(
            request["response_format"]["type"],
            "json_schema",
        )
        json_schema = request["response_format"]["json_schema"]
        self.assertEqual(json_schema["name"], "expression_proposal")
        self.assertIs(json_schema["strict"], True)

        system_prompt = request["messages"][0]["content"]
        prompt_lower = system_prompt.lower()
        self.assertIn("harmless", prompt_lower)
        self.assertIn("grumpy", prompt_lower)
        self.assertIn("object_id is null", prompt_lower)
        self.assertIn("response_locale", system_prompt)
        self.assertIn("authoritative", prompt_lower)
        self.assertIn("speech-only", prompt_lower)
        self.assertIn("genuinely strongly upset", prompt_lower)
        self.assertIn("no markdown", prompt_lower)
        user_payload = json.loads(request["messages"][1]["content"])
        self.assertEqual(user_payload["response_locale"], "en-GB")
        self.assertEqual(
            user_payload["interaction_snapshot"],
            snapshot().to_dict(),
        )

        schema = json_schema["schema"]
        self.assertEqual(
            [
                branch["properties"]["decision"]["const"]
                for branch in schema["oneOf"]
            ],
            ["EXPRESS", "EXPRESS", "HOLD", "ABORT"],
        )
        express_variants = schema_branches(schema, "EXPRESS")
        self.assertEqual(len(express_variants), 2)
        expected_constants = {
            "schema": EXPRESSION_PROPOSAL_SCHEMA,
            "robot_id": "ev3rstorm-1",
            "controller_instance_id": "controller-7",
            "goal_id": "goal-waypoint-2",
            "goal_epoch": 3,
            "plan_revision": 4,
            "based_on_interaction_state_version": 11,
            "based_on_world_model_version": 8,
            "obstruction_epoch": 2,
            "based_on_evidence_id": "evidence-9",
            "decision": "EXPRESS",
        }
        by_gesture = {}
        for express in express_variants:
            properties = express["properties"]
            for field, expected in expected_constants.items():
                with self.subTest(field=field):
                    self.assertEqual(
                        properties[field]["const"],
                        expected,
                    )
            self.assertFalse(express["additionalProperties"])
            self.assertEqual(
                set(properties["intent"]["properties"]),
                {
                    "utterance",
                    "utterance_locale",
                    "gesture_kind",
                    "affect_label",
                    "intensity",
                    "repetitions",
                },
            )
            self.assertFalse(
                properties["intent"]["additionalProperties"]
            )
            self.assertEqual(
                properties["intent"]["properties"][
                    "utterance_locale"
                ]["const"],
                "en-GB",
            )
            intent_properties = properties["intent"]["properties"]
            by_gesture[
                intent_properties["gesture_kind"]["const"]
            ] = intent_properties

        self.assertEqual(set(by_gesture), {None, "PROPELLER_WAVE"})
        self.assertEqual(by_gesture[None]["repetitions"]["const"], 0)
        self.assertEqual(
            by_gesture["PROPELLER_WAVE"]["repetitions"]["minimum"],
            1,
        )
        self.assertEqual(
            by_gesture["PROPELLER_WAVE"]["repetitions"]["maximum"],
            2,
        )
        pattern = by_gesture[None]["utterance"]["pattern"]
        self.assertIsNotNone(re.search(pattern, "Valid utterance"))
        for invalid in (" leading", "trailing ", "line\nbreak"):
            with self.subTest(invalid=invalid):
                self.assertIsNone(re.search(pattern, invalid))
        self.assertEqual(
            express_variants[0]["properties"]["proposal_id"]["pattern"],
            pattern,
        )
        forbidden = {
            "motor_role",
            "motor_port",
            "port",
            "speed",
            "speed_dps",
            "duration",
            "duration_ms",
            "source",
            "ttl",
            "ttl_ms",
            "priority",
            "authority",
        }
        self.assertFalse(forbidden & collect_property_names(schema))

        decoded = decode_expression_proposal(result)
        decoded.assert_matches_snapshot(snapshot())
        self.assertEqual(decoded.intent.utterance_locale, "en-GB")

    def test_accepts_structured_speech_only_expression(self):
        response = completion(
            proposal_value(gesture_kind=None)
        )
        planner = LMStudioExpressionPlanner(
            response_locale="en-GB",
            model=MODEL,
            transport=CapturingTransport(response=response),
            clock=Clock(),
        )

        decoded = decode_expression_proposal(planner(snapshot()))

        decoded.assert_matches_snapshot(snapshot())
        self.assertIsNone(decoded.intent.gesture_kind)
        self.assertEqual(decoded.intent.repetitions, 0)

    def test_no_evidence_cannot_produce_an_invalid_express_branch(self):
        no_evidence = InteractionSnapshot(
            robot_id="ev3rstorm-1",
            controller_instance_id="controller-7",
            goal_id="goal-waypoint-2",
            goal_epoch=3,
            plan_revision=4,
            interaction_state_version=11,
            world_model_version=8,
            captured_at_ms=10_000,
            obstruction_epoch=0,
            drive_phase="MOVING",
            response_locale="x-demo",
            evidence=None,
        )
        response_value = proposal_value("HOLD")
        response_value["obstruction_epoch"] = 0
        response_value["based_on_evidence_id"] = None
        transport = CapturingTransport(
            response=completion(response_value)
        )
        planner = LMStudioExpressionPlanner(
            response_locale="x-demo",
            model=MODEL,
            transport=transport,
            clock=Clock(),
        )

        result = planner(no_evidence)

        request = json.loads(transport.calls[0][1])
        schema = request["response_format"]["json_schema"]["schema"]
        self.assertEqual(
            [
                item["properties"]["decision"]["const"]
                for item in schema["oneOf"]
            ],
            ["HOLD", "ABORT"],
        )
        self.assertIsNone(
            schema_branch(schema, "HOLD")["properties"][
                "based_on_evidence_id"
            ]["const"]
        )
        decode_expression_proposal(result).assert_matches_snapshot(
            no_evidence
        )


class LMStudioExpressionConfigurationTests(unittest.TestCase):
    def test_accepts_generic_locale_identifiers(self):
        for locale in ("sv", "en-US", "pl-Latn-PL", "x-demo"):
            with self.subTest(locale=locale):
                planner = LMStudioExpressionPlanner(
                    response_locale=locale,
                    transport=CapturingTransport(),
                )
                self.assertEqual(planner.response_locale, locale)

    def test_rejects_invalid_locale_dependency_and_timeout(self):
        invalid_locales = (None, "", " en", "en\nUS", "x" * 65)
        for locale in invalid_locales:
            with self.subTest(locale=locale):
                with self.assertRaises(LMStudioConfigurationError):
                    LMStudioExpressionPlanner(response_locale=locale)
        invalid_options = (
            {"transport": None},
            {"clock": None},
            {"timeout_seconds": True},
            {"timeout_seconds": 0.09},
            {"timeout_seconds": 10.01},
        )
        for options in invalid_options:
            with self.subTest(options=options):
                with self.assertRaises(LMStudioConfigurationError):
                    LMStudioExpressionPlanner(
                        response_locale="en",
                        **options
                    )

    def test_allows_only_loopback_base_urls(self):
        for base_url in (
            "http://example.com:1234",
            "http://192.168.1.20:1234",
            "http://user@localhost:1234",
            "http://localhost:1234/path",
        ):
            with self.subTest(base_url=base_url):
                with self.assertRaises(LMStudioConfigurationError):
                    LMStudioExpressionPlanner(
                        response_locale="en",
                        base_url=base_url,
                    )
        for base_url in (
            "http://localhost:1234",
            "https://127.0.0.1:1234",
            "http://[::1]:1234",
        ):
            with self.subTest(base_url=base_url):
                planner = LMStudioExpressionPlanner(
                    response_locale="en",
                    base_url=base_url,
                )
                self.assertEqual(planner.response_locale, "en")

    def test_requires_an_interaction_snapshot(self):
        planner = LMStudioExpressionPlanner(
            response_locale="en",
            transport=CapturingTransport(),
        )
        with self.assertRaises(LMStudioProtocolError):
            planner({"robot_id": "spoofed"})

    def test_requires_configured_locale_to_match_host_snapshot(self):
        transport = CapturingTransport()
        planner = LMStudioExpressionPlanner(
            response_locale="sv",
            model=MODEL,
            transport=transport,
            clock=Clock(),
        )

        with self.assertRaises(LMStudioProtocolError):
            planner(snapshot(response_locale="en"))

        self.assertEqual(transport.calls, [])


class LMStudioExpressionFailureTests(unittest.TestCase):
    def test_maps_timeout_and_transport_failures(self):
        cases = (
            (socket.timeout("late"), LMStudioTimeoutError),
            (TimeoutError("late"), LMStudioTimeoutError),
            (OSError("offline"), LMStudioTransportError),
        )
        for error, expected in cases:
            with self.subTest(error=error):
                planner = LMStudioExpressionPlanner(
                    response_locale="en",
                    model=MODEL,
                    transport=CapturingTransport(error=error),
                    clock=Clock(),
                )
                with self.assertRaises(expected):
                    planner(snapshot(response_locale="en"))

    def test_enforces_elapsed_deadline_at_ten_seconds_or_less(self):
        transport = CapturingTransport()
        planner = LMStudioExpressionPlanner(
            response_locale="en",
            model=MODEL,
            transport=transport,
            clock=Clock(2.0, 3.0),
            timeout_seconds=1.0,
        )

        with self.assertRaises(LMStudioTimeoutError):
            planner(snapshot(response_locale="en"))
        self.assertEqual(transport.calls[0][3], 1.0)

    def test_rejects_model_mismatch_and_malformed_completions(self):
        malformed = (
            b"not-json",
            json.dumps([]).encode("utf-8"),
            completion(model="other/model"),
            completion(choices=[]),
            completion(finish_reason="length"),
            completion(
                choices=[
                    {
                        "index": 1,
                        "finish_reason": "stop",
                        "message": {
                            "role": "assistant",
                            "content": "{}",
                        },
                    }
                ]
            ),
            completion(
                choices=[
                    {
                        "index": 0,
                        "finish_reason": "stop",
                        "message": {
                            "role": "tool",
                            "content": "{}",
                        },
                    }
                ]
            ),
            completion(
                choices=[
                    {
                        "index": 0,
                        "finish_reason": "stop",
                        "message": {
                            "role": "assistant",
                            "content": "",
                        },
                    }
                ]
            ),
            completion(
                choices=[
                    {
                        "index": 0,
                        "finish_reason": "stop",
                        "message": {
                            "role": "assistant",
                            "content": "{}",
                            "tool_calls": [{"type": "function"}],
                        },
                    }
                ]
            ),
            completion(
                choices=[
                    {
                        "index": False,
                        "finish_reason": "stop",
                        "message": {
                            "role": "assistant",
                            "content": "{}",
                        },
                    }
                ]
            ),
            completion(
                choices=[
                    {
                        "index": 0.0,
                        "finish_reason": "stop",
                        "message": {
                            "role": "assistant",
                            "content": "{}",
                        },
                    }
                ]
            ),
        )
        for response in malformed:
            with self.subTest(response=response[:80]):
                planner = LMStudioExpressionPlanner(
                    response_locale="en-GB",
                    model=MODEL,
                    transport=CapturingTransport(response=response),
                    clock=Clock(),
                )
                with self.assertRaises(LMStudioProtocolError):
                    planner(snapshot())

    def test_rejects_duplicate_completion_keys_and_bounded_bodies(self):
        duplicate = completion().replace(
            b'"model": "local/expression-model",',
            b'"model": "local/expression-model",'
            b'"model": "other/model",',
        )
        planner = LMStudioExpressionPlanner(
            response_locale="en",
            model=MODEL,
            transport=CapturingTransport(response=duplicate),
            clock=Clock(),
        )
        with self.assertRaises(LMStudioProtocolError):
            planner(snapshot(response_locale="en"))

        oversized_body = b"{" + b" " * MAX_EXPRESSION_RESPONSE_BYTES + b"}"
        planner = LMStudioExpressionPlanner(
            response_locale="en",
            model=MODEL,
            transport=CapturingTransport(response=oversized_body),
            clock=Clock(),
        )
        with self.assertRaises(LMStudioResponseTooLargeError):
            planner(snapshot(response_locale="en"))

        large_content = "x" * (MAX_EXPRESSION_PROPOSAL_BYTES + 1)
        oversized_proposal = completion(
            choices=[
                {
                    "index": 0,
                    "finish_reason": "stop",
                    "message": {
                        "role": "assistant",
                        "content": large_content,
                    },
                }
            ]
        )
        planner = LMStudioExpressionPlanner(
            response_locale="en",
            model=MODEL,
            transport=CapturingTransport(response=oversized_proposal),
            clock=Clock(),
        )
        with self.assertRaises(LMStudioResponseTooLargeError):
            planner(snapshot(response_locale="en"))


if __name__ == "__main__":
    unittest.main()
