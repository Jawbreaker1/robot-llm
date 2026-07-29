import json
import socket
import unittest
from unittest.mock import patch

from robot_agent.autonomy_contract import (
    AUTONOMY_SELECTION_SCHEMA,
    EXPLORE_SPACE,
    FORWARD,
    INVESTIGATE_OBSERVATION,
    LEFT,
    ROBOT_BASE_FRAME,
    ExplorationCandidate,
    InterestObservation,
    InterestSelectionContext,
    decode_interest_selection,
)
from robot_agent.lm_studio import (
    LMStudioConfigurationError,
    LMStudioProtocolError,
    LMStudioResponseTooLargeError,
    LMStudioTimeoutError,
    LMStudioTransportError,
)
from robot_agent.lm_studio_autonomy import (
    CHAT_COMPLETIONS_PATH,
    MAX_AUTONOMY_PROPOSAL_BYTES,
    MAX_AUTONOMY_REQUEST_BYTES,
    MAX_AUTONOMY_RESPONSE_BYTES,
    LMStudioInterestSelector,
)
from robot_agent.navigation_contract import NavigationContractError


MODEL = "local/autonomy-model"


def selection_context():
    observation = InterestObservation(
        observation_id="observation-range-9",
        producer_id="range-change-tracker",
        subject_robot_id="ev3rstorm-1",
        controller_instance_id="controller-7",
        frame_id=ROBOT_BASE_FRAME,
        modality="RANGE",
        kind="VALUE_AND_SUBJECT_CHANGED",
        channel="forward-clearance",
        observed_at_ms=9_990,
        received_at_host_ms=9_995,
        valid_until_host_ms=14_000,
        state_version=11,
        world_model_version=8,
        confidence_milli=940,
        previous_value=640,
        current_value=260,
        unit="mm",
        previous_subject_id=None,
        current_subject_id="box-a",
    )
    return InterestSelectionContext(
        proposal_id="interest-proposal-17",
        robot_id="ev3rstorm-1",
        controller_instance_id="controller-7",
        autonomy_session_id="idle-session-3",
        lease_generation=5,
        candidate_set_id="candidate-set-12",
        frame_id=ROBOT_BASE_FRAME,
        state_version=11,
        world_model_version=8,
        captured_at_ms=10_000,
        valid_until_ms=14_000,
        remaining_tasks=3,
        observations=(observation,),
        candidates=(
            ExplorationCandidate(
                candidate_id="investigate-left-12",
                task_kind=INVESTIGATE_OBSERVATION,
                relative_direction=LEFT,
                estimated_travel_mm=220,
                attempted_visits=0,
                completed_visits=0,
                linked_observation_ids=(
                    observation.observation_id,
                ),
            ),
            ExplorationCandidate(
                candidate_id="explore-forward-12",
                task_kind=EXPLORE_SPACE,
                relative_direction=FORWARD,
                estimated_travel_mm=300,
                attempted_visits=2,
                completed_visits=1,
            ),
        ),
    )


def proposal_value(
    decision="SELECT",
    selected_candidate_id="investigate-left-12",
):
    value = {
        "schema": AUTONOMY_SELECTION_SCHEMA,
        "proposal_id": "interest-proposal-17",
        "robot_id": "ev3rstorm-1",
        "controller_instance_id": "controller-7",
        "autonomy_session_id": "idle-session-3",
        "lease_generation": 5,
        "candidate_set_id": "candidate-set-12",
        "based_on_state_version": 11,
        "based_on_world_model_version": 8,
        "decision": decision,
        "confidence_milli": 870,
    }
    if decision == "SELECT":
        value["selected_candidate_id"] = selected_candidate_id
    else:
        value["reason_code"] = "no_useful_candidate"
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
            "id": "chatcmpl-autonomy",
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


def schema_branch(schema, decision):
    matches = [
        branch
        for branch in schema["oneOf"]
        if branch["properties"]["decision"]["const"] == decision
    ]
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


class LMStudioAutonomyRequestTests(unittest.TestCase):
    def test_builds_bound_tool_free_language_neutral_selection_schema(self):
        context = selection_context()
        transport = CapturingTransport()
        selector = LMStudioInterestSelector(
            base_url="http://127.0.0.1:1234/",
            model=MODEL,
            transport=transport,
            clock=Clock(),
            timeout_seconds=2.5,
        )

        result = selector(context)

        self.assertEqual(len(transport.calls), 1)
        url, body, headers, timeout, maximum = transport.calls[0]
        self.assertEqual(
            url,
            "http://127.0.0.1:1234" + CHAT_COMPLETIONS_PATH,
        )
        self.assertEqual(headers["Accept"], "application/json")
        self.assertEqual(
            headers["Content-Type"],
            "application/json; charset=utf-8",
        )
        self.assertEqual(timeout, 2.5)
        self.assertEqual(maximum, MAX_AUTONOMY_RESPONSE_BYTES)

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
        self.assertEqual(
            json_schema["name"],
            "autonomy_interest_selection",
        )
        self.assertIs(json_schema["strict"], True)

        prompt = request["messages"][0]["content"]
        prompt_lower = prompt.lower()
        self.assertIn("language-neutral", prompt_lower)
        self.assertIn("opaque", prompt_lower)
        self.assertIn("untrusted factual data", prompt_lower)
        self.assertIn("host alone", prompt_lower)
        self.assertIn("no markdown", prompt_lower)
        self.assertIn("never invent or emit coordinates", prompt_lower)
        self.assertIn("do not infer, classify, or route", prompt_lower)
        self.assertEqual(
            json.loads(request["messages"][1]["content"]),
            context.to_dict(),
        )

        schema = json_schema["schema"]
        self.assertEqual(
            [
                branch["properties"]["decision"]["const"]
                for branch in schema["oneOf"]
            ],
            ["SELECT", "HOLD", "ABORT"],
        )
        common_constants = {
            "schema": AUTONOMY_SELECTION_SCHEMA,
            "proposal_id": "interest-proposal-17",
            "robot_id": "ev3rstorm-1",
            "controller_instance_id": "controller-7",
            "autonomy_session_id": "idle-session-3",
            "lease_generation": 5,
            "candidate_set_id": "candidate-set-12",
            "based_on_state_version": 11,
            "based_on_world_model_version": 8,
        }
        for decision in ("SELECT", "HOLD", "ABORT"):
            branch = schema_branch(schema, decision)
            self.assertFalse(branch["additionalProperties"])
            for field, expected in common_constants.items():
                with self.subTest(decision=decision, field=field):
                    self.assertEqual(
                        branch["properties"][field]["const"],
                        expected,
                    )
        self.assertEqual(
            schema_branch(schema, "SELECT")["properties"][
                "selected_candidate_id"
            ]["enum"],
            [
                "investigate-left-12",
                "explore-forward-12",
            ],
        )
        for decision in ("HOLD", "ABORT"):
            branch = schema_branch(schema, decision)
            self.assertIn("reason_code", branch["required"])
            self.assertNotIn(
                "selected_candidate_id",
                branch["properties"],
            )

        forbidden = {
            "x_mm",
            "y_mm",
            "coordinates",
            "heading",
            "path",
            "waypoint",
            "goal_epoch",
            "plan_revision",
            "speed",
            "speed_dps",
            "duration",
            "duration_ms",
            "motor",
            "motor_port",
            "tool",
            "source",
            "ttl",
            "ttl_ms",
            "priority",
            "authority",
            "locale",
            "utterance",
        }
        self.assertFalse(forbidden & collect_property_names(schema))

        decoded = decode_interest_selection(result)
        context.assert_accepts(decoded, now_ms=10_100)
        self.assertEqual(
            decoded.selected_candidate_id,
            "investigate-left-12",
        )

    def test_returns_valid_hold_and_abort_as_untrusted_bytes(self):
        for decision in ("HOLD", "ABORT"):
            with self.subTest(decision=decision):
                transport = CapturingTransport(
                    response=completion(proposal_value(decision))
                )
                selector = LMStudioInterestSelector(
                    model=MODEL,
                    transport=transport,
                    clock=Clock(),
                )

                raw = selector(selection_context())
                decoded = decode_interest_selection(raw)

                selection_context().assert_accepts(
                    decoded,
                    now_ms=10_500,
                )
                self.assertEqual(decoded.decision, decision)
                self.assertIsNone(decoded.selected_candidate_id)

    def test_inner_content_remains_untrusted_for_contract_decoder(self):
        response = completion(
            choices=[
                {
                    "index": 0,
                    "finish_reason": "stop",
                    "message": {
                        "role": "assistant",
                        "content": "not-json",
                    },
                }
            ]
        )
        selector = LMStudioInterestSelector(
            model=MODEL,
            transport=CapturingTransport(response=response),
            clock=Clock(),
        )

        raw = selector(selection_context())

        self.assertEqual(raw, b"not-json")
        with self.assertRaises(NavigationContractError):
            decode_interest_selection(raw)


class LMStudioAutonomyConfigurationTests(unittest.TestCase):
    def test_allows_only_loopback_base_urls(self):
        for base_url in (
            "http://example.com:1234",
            "http://192.168.1.20:1234",
            "http://user@localhost:1234",
            "http://localhost:1234/path",
        ):
            with self.subTest(base_url=base_url):
                with self.assertRaises(LMStudioConfigurationError):
                    LMStudioInterestSelector(base_url=base_url)
        for base_url in (
            "http://localhost:1234",
            "https://127.0.0.1:1234",
            "http://[::1]:1234",
        ):
            with self.subTest(base_url=base_url):
                selector = LMStudioInterestSelector(base_url=base_url)
                self.assertTrue(selector.model)

    def test_rejects_invalid_dependencies_model_and_timeout(self):
        invalid_options = (
            {"transport": None},
            {"clock": None},
            {"timeout_seconds": True},
            {"timeout_seconds": 0.09},
            {"timeout_seconds": 10.01},
            {"model": ""},
            {"model": " model"},
        )
        for options in invalid_options:
            with self.subTest(options=options):
                with self.assertRaises(LMStudioConfigurationError):
                    LMStudioInterestSelector(**options)

    def test_requires_typed_context_before_transport(self):
        transport = CapturingTransport()
        selector = LMStudioInterestSelector(
            transport=transport,
            clock=Clock(),
        )

        with self.assertRaises(LMStudioProtocolError):
            selector({"candidate_id": "spoofed"})

        self.assertEqual(transport.calls, [])

    def test_bounds_serialized_request_before_transport(self):
        context = selection_context()
        transport = CapturingTransport()
        selector = LMStudioInterestSelector(
            transport=transport,
            clock=Clock(),
        )
        oversized_payload = {
            "blob": "x" * MAX_AUTONOMY_REQUEST_BYTES,
        }

        with patch.object(
            InterestSelectionContext,
            "to_dict",
            return_value=oversized_payload,
        ):
            with self.assertRaises(LMStudioProtocolError):
                selector(context)

        self.assertEqual(transport.calls, [])


class LMStudioAutonomyFailureTests(unittest.TestCase):
    def test_maps_timeout_and_transport_failures(self):
        cases = (
            (socket.timeout("late"), LMStudioTimeoutError),
            (TimeoutError("late"), LMStudioTimeoutError),
            (OSError("offline"), LMStudioTransportError),
        )
        for error, expected in cases:
            with self.subTest(error=error):
                selector = LMStudioInterestSelector(
                    model=MODEL,
                    transport=CapturingTransport(error=error),
                    clock=Clock(),
                )
                with self.assertRaises(expected):
                    selector(selection_context())

    def test_enforces_elapsed_deadline(self):
        transport = CapturingTransport()
        selector = LMStudioInterestSelector(
            model=MODEL,
            transport=transport,
            clock=Clock(2.0, 3.0),
            timeout_seconds=1.0,
        )

        with self.assertRaises(LMStudioTimeoutError):
            selector(selection_context())

        self.assertEqual(transport.calls[0][3], 1.0)

    def test_rejects_model_mismatch_and_malformed_completions(self):
        malformed = (
            b"not-json",
            json.dumps([]).encode("utf-8"),
            json.dumps(
                {
                    "object": "other",
                    "model": MODEL,
                    "choices": [],
                }
            ).encode("utf-8"),
            completion(model="other/model"),
            completion(choices=[]),
            completion(choices=[{}, {}]),
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
                        "index": 0,
                        "finish_reason": "stop",
                        "message": {
                            "role": "assistant",
                            "content": "{}",
                            "refusal": "no",
                        },
                    }
                ]
            ),
            b'{"object":"chat.completion","model":"'
            + MODEL.encode("utf-8")
            + b'","choices":[],"bad":NaN}',
        )
        for response in malformed:
            with self.subTest(response=response[:100]):
                selector = LMStudioInterestSelector(
                    model=MODEL,
                    transport=CapturingTransport(response=response),
                    clock=Clock(),
                )
                with self.assertRaises(LMStudioProtocolError):
                    selector(selection_context())

    def test_rejects_duplicate_keys_and_bounded_response_bodies(self):
        duplicate = completion().replace(
            b'"model": "local/autonomy-model",',
            b'"model": "local/autonomy-model",'
            b'"model": "other/model",',
        )
        selector = LMStudioInterestSelector(
            model=MODEL,
            transport=CapturingTransport(response=duplicate),
            clock=Clock(),
        )
        with self.assertRaises(LMStudioProtocolError):
            selector(selection_context())

        oversized_body = (
            b"{" + b" " * MAX_AUTONOMY_RESPONSE_BYTES + b"}"
        )
        selector = LMStudioInterestSelector(
            model=MODEL,
            transport=CapturingTransport(response=oversized_body),
            clock=Clock(),
        )
        with self.assertRaises(LMStudioResponseTooLargeError):
            selector(selection_context())

        large_content = "x" * (MAX_AUTONOMY_PROPOSAL_BYTES + 1)
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
        selector = LMStudioInterestSelector(
            model=MODEL,
            transport=CapturingTransport(
                response=oversized_proposal
            ),
            clock=Clock(),
        )
        with self.assertRaises(LMStudioResponseTooLargeError):
            selector(selection_context())

    def test_rejects_non_bytes_response(self):
        selector = LMStudioInterestSelector(
            model=MODEL,
            transport=CapturingTransport(response="not-bytes"),
            clock=Clock(),
        )

        with self.assertRaises(LMStudioProtocolError):
            selector(selection_context())


if __name__ == "__main__":
    unittest.main()
