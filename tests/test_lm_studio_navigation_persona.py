import json
import unittest

from robot_agent.lm_studio_navigation import (
    MAX_RECENT_COMMITTED_UTTERANCES,
    LMStudioNavigationError,
    LMStudioNavigationPlanner,
    UTTERANCE_LANGUAGE_BY_LOCALE,
    UTTERANCE_PERSONA_BY_LOCALE,
)
from robot_agent.maneuver_commitment import empty_commitment
from robot_agent.physical_navigation_contract import (
    DECISION_SCHEMA,
    OBSERVE,
)


def decision(*, episode_id="episode-persona", turn=1, state_version=7):
    return {
        "schema": DECISION_SCHEMA,
        "episode_id": episode_id,
        "turn": turn,
        "based_on_state_version": state_version,
        "action": OBSERVE,
        "plan": [OBSERVE],
        "reason_code": "VERIFY_RESULT",
        "assessment": "A fresh observation is appropriate.",
        "utterance": None,
        "perception_target_hypothesis_id": None,
        "maneuver_commitment": empty_commitment(),
    }


class CapturingTransport:
    def __init__(self):
        self.calls = []

    def __call__(self, url, body, headers, timeout, maximum):
        self.calls.append({
            "url": url,
            "payload": json.loads(body.decode("utf-8")),
            "headers": headers,
            "timeout": timeout,
            "maximum": maximum,
        })
        request_context = json.loads(
            self.calls[-1]["payload"]["messages"][1]["content"]
        )
        value = decision(
            episode_id=request_context["episode_id"],
            turn=request_context["turn"],
            state_version=request_context["observation"]["state_version"],
        )
        return json.dumps({
            "model": self.calls[-1]["payload"]["model"],
            "choices": [{"message": {"content": json.dumps(value)}}],
        }).encode("utf-8")


def invoke(
    transport,
    *,
    locale="sv",
    recent=(),
    observation=None,
    last_tool_result=None,
):
    planner = LMStudioNavigationPlanner(
        base_url="http://127.0.0.1:1234",
        model="local/test-model",
        transport=transport,
        clock=lambda: 1.0,
    )
    return planner.decide(
        episode_id="episode-persona",
        turn=1,
        locale=locale,
        observation=(
            {"state_version": 7, "infrared": {"blocked": True}}
            if observation is None
            else observation
        ),
        mission={"completed": False},
        navigation={"navigation_hazard_hypotheses": []},
        maneuver_state={"active": None},
        available_actions=[OBSERVE],
        last_tool_result=last_tool_result,
        recent_committed_utterances=recent,
    )


class LMStudioNavigationPersonaTests(unittest.TestCase):
    def test_host_supplies_locale_specific_grumpy_persona(self):
        captured = {}
        for locale in ("sv", "en"):
            with self.subTest(locale=locale):
                transport = CapturingTransport()
                invoke(transport, locale=locale)
                payload = transport.calls[0]["payload"]
                context = json.loads(payload["messages"][1]["content"])
                guidance = context["utterance_guidance"]
                captured[locale] = guidance["persona"]

                self.assertEqual(
                    context["output_languages"]["utterance"],
                    UTTERANCE_LANGUAGE_BY_LOCALE[locale],
                )
                self.assertEqual(
                    guidance,
                    {
                        "persona": UTTERANCE_PERSONA_BY_LOCALE[locale],
                        "recent_committed_utterances": [],
                    },
                )
                self.assertIn("LEGO", guidance["persona"])
                self.assertIn(
                    "strukturerade beslutet"
                    if locale == "sv"
                    else "structured decision",
                    guidance["persona"],
                )

                system_prompt = payload["messages"][0]["content"].lower()
                self.assertIn("utterance is optional", system_prompt)
                self.assertIn("never control data", system_prompt)
                self.assertIn("do not repeat or closely paraphrase", system_prompt)
                self.assertIn("prefer null", system_prompt)
                self.assertNotIn("tools", payload)
                self.assertEqual(payload["temperature"], 0)

        self.assertIn("svordomar", captured["sv"])
        self.assertIn("milda till kraftiga", captured["sv"])
        self.assertIn("profanity", captured["en"])
        self.assertIn("mild to strong", captured["en"])
        self.assertIn("Svär inte i varje replik", captured["sv"])
        self.assertIn("Do not swear in every utterance", captured["en"])

    def test_recent_committed_speech_is_explicit_model_context(self):
        transport = CapturingTransport()
        spoken = [
            "Jaha, ännu en låda i vägen.",
            "Förbannade motor, kom igen nu.",
        ]

        invoke(transport, locale="sv", recent=spoken)

        context = json.loads(
            transport.calls[0]["payload"]["messages"][1]["content"]
        )
        self.assertEqual(
            context["utterance_guidance"][
                "recent_committed_utterances"
            ],
            spoken,
        )

    def test_persona_does_not_depend_on_phrase_or_event_heuristics(self):
        scenarios = (
            (
                {"state_version": 7, "infrared": {"blocked": True}},
                {"operation": "scan", "status": "cancelled"},
            ),
            (
                {"state_version": 7, "infrared": {"blocked": False}},
                {
                    "operation": "pulse",
                    "status": "degraded",
                    "reason": "motor_progress_mismatch",
                },
            ),
        )
        personas = []
        for observation, latest_result in scenarios:
            transport = CapturingTransport()
            invoke(
                transport,
                locale="en",
                observation=observation,
                last_tool_result=latest_result,
            )
            context = json.loads(
                transport.calls[0]["payload"]["messages"][1]["content"]
            )
            personas.append(context["utterance_guidance"]["persona"])
            self.assertEqual(context["observation"], observation)
            self.assertEqual(context["latest_tool_result"], latest_result)

        self.assertEqual(personas, [UTTERANCE_PERSONA_BY_LOCALE["en"]] * 2)

    def test_recent_committed_speech_boundary_is_fail_closed(self):
        invalid_values = (
            "not-a-sequence-of-utterances",
            ["spoken"] * (MAX_RECENT_COMMITTED_UTTERANCES + 1),
            [""],
            [" leading whitespace"],
            ["x" * 161],
            ["hidden\x00control"],
        )
        for recent in invalid_values:
            with self.subTest(recent=recent):
                transport = CapturingTransport()
                with self.assertRaises(LMStudioNavigationError):
                    invoke(transport, recent=recent)
                self.assertEqual(transport.calls, [])


if __name__ == "__main__":
    unittest.main()
