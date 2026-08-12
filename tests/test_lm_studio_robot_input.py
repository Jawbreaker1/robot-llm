import json
import socket
import unittest

from robot_agent.lm_studio import (
    LMStudioConfigurationError,
    LMStudioInputError,
)
from robot_agent.blast_personality import (
    BLAST_PERSONA_BY_LOCALE,
    MAX_PERSONA_CHARS,
)
from robot_agent.lm_studio_robot_input import (
    CHAT_COMPLETIONS_PATH,
    CLARIFY,
    CONVERSE,
    LMStudioRobotInputModel,
    MAX_FACTS_BYTES,
    MAX_INPUT_CHARS,
    MAX_REPLY_CHARS,
    MAX_RESPONSE_BYTES,
    MIN_PHYSICAL_CONFIDENCE_MILLI,
    PHYSICAL_TASK,
    READ_ONLY_TASK,
    STOP_TASK,
    RobotInput,
    RobotInputDecision,
)


MODEL = "local/gemma-robot-input"


def robot_input(locale="sv", text="Ser du något?"):
    return RobotInput("turn-17", text, locale)


def completion(
    intent=READ_ONLY_TASK,
    reply="IR-kartan visar ett hinder framför mig.",
    *,
    model=MODEL,
    finish="stop",
    message_changes=None,
    output_changes=None,
):
    output = {
        "intent": intent,
        "confidence_milli": 930,
        "reply_text": reply,
    }
    output.update(output_changes or {})
    message = {"role": "assistant", "content": json.dumps(output)}
    message.update(message_changes or {})
    return json.dumps({
        "object": "chat.completion",
        "model": model,
        "choices": [{"index": 0, "finish_reason": finish, "message": message}],
    }).encode()


class Transport:
    def __init__(self, response=None, error=None):
        self.response = completion() if response is None else response
        self.error = error
        self.calls = []

    def __call__(self, url, body, headers, timeout, maximum):
        self.calls.append((url, body, headers, timeout, maximum))
        if self.error:
            raise self.error
        return self.response


class RobotInputContractTests(unittest.TestCase):
    def test_accepts_bounded_inputs_and_decisions(self):
        for locale in ("sv", "en"):
            value = RobotInput("id", "x" * MAX_INPUT_CHARS, locale)
            self.assertEqual(value.locale, locale)
        self.assertEqual(
            RobotInputDecision(PHYSICAL_TASK, 900, None),
            RobotInputDecision(PHYSICAL_TASK, 900, None, False),
        )
        self.assertIsNone(RobotInputDecision(STOP_TASK, 900, None).reply_text)
        self.assertEqual(RobotInputDecision(CONVERSE, 800, "Jaha.").reply_text, "Jaha.")

    def test_rejects_invalid_input_and_decision_invariants(self):
        invalid_inputs = (
            ("", "hej", "sv"),
            ("id", " leading", "sv"),
            ("id", "x" * (MAX_INPUT_CHARS + 1), "sv"),
            ("id", "hej", "sv-SE"),
        )
        for values in invalid_inputs:
            with self.subTest(values=values), self.assertRaises(LMStudioInputError):
                RobotInput(*values)
        invalid_decisions = (
            ("MOVE", 900, None, False),
            (PHYSICAL_TASK, 900, "Kör", False),
            (PHYSICAL_TASK, 900, None, True),
            (PHYSICAL_TASK, MIN_PHYSICAL_CONFIDENCE_MILLI - 1, None, False),
            (CONVERSE, True, "Hej", False),
            (CONVERSE, 1001, "Hej", False),
            (CONVERSE, 900, "x" * (MAX_REPLY_CHARS + 1), False),
        )
        for values in invalid_decisions:
            with self.subTest(values=values), self.assertRaises(LMStudioInputError):
                RobotInputDecision(*values)


class LMStudioRobotInputTests(unittest.TestCase):
    def model(self, response=None, error=None, **options):
        transport = Transport(response, error)
        return LMStudioRobotInputModel(
            model=MODEL,
            transport=transport,
            timeout_seconds=2.5,
            **options,
        ), transport

    def test_one_tool_free_call_contains_input_facts_and_strict_schema(self):
        model, transport = self.model()
        facts = {
            "spatial_map": {"available": True, "observed_age_ms": 20},
            "camera_vision": {"available": False},
        }

        result = model.interpret(robot_input(), facts)

        self.assertEqual(result.intent, READ_ONLY_TASK)
        self.assertFalse(result.fallback)
        self.assertEqual(len(transport.calls), 1)
        url, body, headers, timeout, maximum = transport.calls[0]
        self.assertEqual(url, "http://127.0.0.1:1234" + CHAT_COMPLETIONS_PATH)
        self.assertEqual(headers["Content-Type"], "application/json; charset=utf-8")
        self.assertEqual((timeout, maximum), (2.5, MAX_RESPONSE_BYTES))
        payload = json.loads(body)
        self.assertEqual(payload["model"], MODEL)
        self.assertEqual(payload["temperature"], 0)
        self.assertEqual(payload["reasoning_effort"], "none")
        self.assertFalse(payload["stream"])
        self.assertFalse(payload["store"])
        self.assertNotIn("tools", payload)
        self.assertNotIn("tool_choice", payload)
        supplied = json.loads(payload["messages"][1]["content"])
        self.assertEqual(supplied, {
            "request_id": "turn-17",
            "text": "Ser du något?",
            "locale": "sv",
            "facts": facts,
        })
        prompt = payload["messages"][0]["content"]
        for phrase in (
            "never keywords, regex",
            "why the robot is in its current state",
            "gain new evidence by changing pose or orientation",
            "unambiguous antecedent",
            "IR/range and map data are not camera vision",
            "tired, grumpy but harmless",
            "For PHYSICAL_TASK and STOP_TASK set reply_text to null",
        ):
            self.assertIn(phrase, prompt)
        schema = payload["response_format"]["json_schema"]
        self.assertTrue(schema["strict"])
        self.assertFalse(schema["schema"]["additionalProperties"])
        properties = schema["schema"]["properties"]
        self.assertEqual(properties["intent"]["enum"], [
            CONVERSE, READ_ONLY_TASK, PHYSICAL_TASK, STOP_TASK, CLARIFY
        ])
        self.assertEqual(properties["reply_text"]["oneOf"][1], {"type": "null"})

    def test_all_intents_use_one_call_and_physical_has_no_reply(self):
        cases = (
            (CONVERSE, "Jaså, du vill prata.", False),
            (READ_ONLY_TASK, "IR visar ett hinder.", False),
            (CLARIFY, "Vad menar du?", False),
            (PHYSICAL_TASK, None, False),
            (STOP_TASK, None, False),
        )
        for intent, reply, fallback in cases:
            with self.subTest(intent=intent):
                model, transport = self.model(completion(intent, reply))
                result = model.interpret(robot_input(), {})
                self.assertEqual(
                    result, RobotInputDecision(intent, 930, reply, fallback)
                )
                self.assertEqual(len(transport.calls), 1)

    def test_blast_persona_changes_only_the_locale_specific_system_prompt(self):
        default_model, default_transport = self.model()
        blast_model, blast_transport = self.model(
            reply_persona_by_locale=BLAST_PERSONA_BY_LOCALE
        )

        default_model.interpret(robot_input(), {"distance_mm": 480})
        blast_model.interpret(robot_input(), {"distance_mm": 480})

        default_payload = json.loads(default_transport.calls[0][1])
        blast_payload = json.loads(blast_transport.calls[0][1])
        default_prompt = default_payload["messages"][0]["content"]
        blast_prompt = blast_payload["messages"][0]["content"]
        self.assertTrue(blast_prompt.startswith(default_prompt))
        self.assertIn(BLAST_PERSONA_BY_LOCALE["sv"], blast_prompt)
        self.assertNotIn(BLAST_PERSONA_BY_LOCALE["en"], blast_prompt)
        for guardrail in (
            "only to the wording and tone of reply_text",
            "never influence intent",
            "sensor facts",
            "safety",
            "whether reply_text must be null",
        ):
            self.assertIn(guardrail, blast_prompt)
        blast_payload["messages"][0]["content"] = default_prompt
        self.assertEqual(blast_payload, default_payload)

        english_model, english_transport = self.model(
            reply_persona_by_locale=BLAST_PERSONA_BY_LOCALE
        )
        english_model.interpret(robot_input("en", "Hello."), {})
        english_prompt = json.loads(english_transport.calls[0][1])[
            "messages"
        ][0]["content"]
        self.assertIn(BLAST_PERSONA_BY_LOCALE["en"], english_prompt)
        self.assertNotIn(BLAST_PERSONA_BY_LOCALE["sv"], english_prompt)

    def test_bad_nonphysical_reply_uses_same_intent_localized_fallback(self):
        for locale, marker in (("sv", "låtsas"), ("en", "pretend")):
            with self.subTest(locale=locale):
                response = completion(READ_ONLY_TASK, "x" * (MAX_REPLY_CHARS + 1))
                model, _ = self.model(response)
                result = model.interpret(robot_input(locale), {})
                self.assertEqual((result.intent, result.confidence_milli), (READ_ONLY_TASK, 930))
                self.assertTrue(result.fallback)
                self.assertIn(marker, result.reply_text)

    def test_physical_text_and_invalid_protocol_fail_closed(self):
        invalid = (
            completion(PHYSICAL_TASK, "Jag kör nu."),
            b"not-json",
            completion(model="wrong/model"),
            completion(finish="length"),
            completion(message_changes={"role": "tool"}),
            completion(message_changes={"tool_calls": [{"type": "function"}]}),
            completion(message_changes={"refusal": "no"}),
            completion(output_changes={"intent": "MOVE"}),
            completion(output_changes={"confidence_milli": True}),
            completion(
                PHYSICAL_TASK,
                None,
                output_changes={
                    "confidence_milli": MIN_PHYSICAL_CONFIDENCE_MILLI - 1
                },
            ),
            completion(output_changes={"extra": 1}),
            b"{" + b" " * MAX_RESPONSE_BYTES + b"}",
        )
        for response in invalid:
            with self.subTest(response=response[:60]):
                model, _ = self.model(response)
                result = model.interpret(robot_input(), {})
                self.assertEqual((result.intent, result.confidence_milli), (CLARIFY, 0))
                self.assertTrue(result.fallback)
                self.assertIn("förtydliga", result.reply_text)

    def test_transport_failures_fail_closed(self):
        for error in (socket.timeout(), TimeoutError(), OSError()):
            with self.subTest(error=error):
                model, _ = self.model(error=error)
                self.assertEqual(model.interpret(robot_input(), {}).intent, CLARIFY)

    def test_invalid_facts_and_calls_are_rejected_without_transport(self):
        model, transport = self.model()
        invalid = (None, {"bad": object()}, {"bad": float("nan")})
        for facts in invalid:
            with self.subTest(facts=facts), self.assertRaises(LMStudioInputError):
                model.interpret(robot_input(), facts)
        with self.assertRaises(LMStudioInputError):
            model.interpret(object(), {})
        with self.assertRaises(LMStudioInputError):
            model.interpret(robot_input(), {"value": "x" * MAX_FACTS_BYTES})
        self.assertEqual(transport.calls, [])

    def test_configuration_is_loopback_and_bounded(self):
        invalid = (
            {"base_url": "https://example.com"},
            {"model": ""},
            {"transport": None},
            {"timeout_seconds": True},
            {"timeout_seconds": 60.1},
            {"reply_persona_by_locale": {}},
            {"reply_persona_by_locale": {"sv": "Bara svenska."}},
            {"reply_persona_by_locale": {
                "sv": "Svenska.", "en": "English.", "de": "Deutsch."
            }},
            {"reply_persona_by_locale": ("sv", "en")},
            {"reply_persona_by_locale": {"sv": " Svenska.", "en": "English."}},
            {"reply_persona_by_locale": {"sv": "Svenska.\n", "en": "English."}},
            {"reply_persona_by_locale": {
                "sv": "x" * (MAX_PERSONA_CHARS + 1), "en": "English."
            }},
        )
        for options in invalid:
            with self.subTest(options=options), self.assertRaises(LMStudioConfigurationError):
                LMStudioRobotInputModel(**options)


if __name__ == "__main__":
    unittest.main()
