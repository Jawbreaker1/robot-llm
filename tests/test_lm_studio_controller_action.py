import json
import unittest

from robot_agent.lm_studio import (
    LMStudioInputError,
    LMStudioProtocolError,
)
from robot_agent.lm_studio_controller_action import (
    COMPLETE,
    ControllerActionContext,
    LMStudioControllerActionPlanner,
)
from robot_agent.physical_navigation_contract import SCAN_FRONT_ARC


MODEL = "local/controller-model"


def context(**changes):
    values = {
        "goal": "Kör mot hindret och stanna ungefär 25 cm ifrån.",
        "locale": "sv",
        "robot_id": "blast-01",
        "controller_id": "blast-01.hub",
        "available_actions": (
            "DRIVE_FORWARD",
            "DRIVE_REVERSE",
            "TURN_LEFT",
            "TURN_RIGHT",
        ),
        "observation": {
            "distance_mm": 480,
            "motion_active": False,
            "imu": {"ready": True, "heading_deg": 0},
        },
        "history": (),
    }
    values.update(changes)
    return ControllerActionContext(**values)


def completion(output, **changes):
    value = {
        "object": "chat.completion",
        "model": MODEL,
        "choices": [
            {
                "index": 0,
                "finish_reason": "stop",
                "message": {
                    "role": "assistant",
                    "content": json.dumps(output),
                },
            }
        ],
    }
    value.update(changes)
    return json.dumps(value).encode("utf-8")


class Transport:
    def __init__(self, response):
        self.response = response
        self.calls = []

    def __call__(self, *args):
        self.calls.append(args)
        if isinstance(self.response, BaseException):
            raise self.response
        return self.response


class ControllerActionPlannerTests(unittest.TestCase):
    def planner(self, response, clock_values=(1.0, 1.125)):
        transport = Transport(response)
        values = iter(clock_values)
        planner = LMStudioControllerActionPlanner(
            model=MODEL,
            transport=transport,
            clock=lambda: next(values),
        )
        return planner, transport

    def test_returns_one_observation_bound_action(self):
        planner, transport = self.planner(completion({
            "action": "DRIVE_FORWARD",
            "confidence_milli": 940,
            "assessment": "Det är fortfarande gott om plats framåt.",
            "plan": ["DRIVE_FORWARD", "COMPLETE"],
            "utterance": "Jaja, jag kör väl en bit till då.",
        }))

        result = planner.decide(context())

        self.assertEqual(result.latency_ms, 125)
        self.assertEqual(result.decision.action, "DRIVE_FORWARD")
        self.assertEqual(
            result.decision.plan,
            ("DRIVE_FORWARD", "COMPLETE"),
        )
        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        self.assertEqual(supplied["observation"]["distance_mm"], 480)
        self.assertEqual(supplied["goal"], context().goal)
        action_schema = request["response_format"]["json_schema"][
            "schema"
        ]["properties"]["action"]
        self.assertEqual(
            action_schema["enum"],
            [
                "DRIVE_FORWARD",
                "DRIVE_REVERSE",
                "TURN_LEFT",
                "TURN_RIGHT",
                "COMPLETE",
                "ABORT",
            ],
        )
        self.assertEqual(request["reasoning_effort"], "none")

    def test_terminal_decision_normalizes_one_redundant_terminal_step(self):
        planner, _ = self.planner(completion({
            "action": COMPLETE,
            "confidence_milli": 900,
            "assessment": "Målet är uppnått.",
            "plan": [],
            "utterance": None,
        }))
        self.assertEqual(
            planner.decide(context()).decision.action,
            COMPLETE,
        )

        invalid, _ = self.planner(completion({
            "action": COMPLETE,
            "confidence_milli": 900,
            "assessment": "Målet är uppnått.",
            "plan": [COMPLETE],
            "utterance": None,
        }))
        self.assertEqual(
            invalid.decide(context()).decision.plan,
            (),
        )

        invalid, _ = self.planner(completion({
            "action": COMPLETE,
            "confidence_milli": 900,
            "assessment": "Målet är uppnått.",
            "plan": ["TURN_LEFT"],
            "utterance": None,
        }))
        with self.assertRaises(LMStudioProtocolError):
            invalid.decide(context())

    def test_scan_action_is_described_as_a_returning_two_sided_sweep(self):
        planner, transport = self.planner(completion({
            "action": SCAN_FRONT_ARC,
            "confidence_milli": 900,
            "assessment": "Jag behöver se båda sidorna.",
            "plan": [SCAN_FRONT_ARC],
            "utterance": None,
        }))

        result = planner.decide(context(available_actions=(
            "TURN_LEFT",
            "TURN_RIGHT",
            SCAN_FRONT_ARC,
        )))

        self.assertEqual(result.decision.action, SCAN_FRONT_ARC)
        request = json.loads(transport.calls[0][1])
        system_prompt = request["messages"][0]["content"]
        self.assertIn(SCAN_FRONT_ARC, system_prompt)
        self.assertIn("both sides", system_prompt)
        self.assertIn("returns near its starting heading", system_prompt)

    def test_nonterminal_plan_must_start_with_selected_action(self):
        planner, _ = self.planner(completion({
            "action": "TURN_LEFT",
            "confidence_milli": 800,
            "assessment": "Jag behöver vrida mig.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": None,
        }))
        with self.assertRaises(LMStudioProtocolError):
            planner.decide(context())

    def test_model_cannot_invent_an_action(self):
        planner, _ = self.planner(completion({
            "action": "JUMP",
            "confidence_milli": 999,
            "assessment": "Hoppa.",
            "plan": ["JUMP"],
            "utterance": None,
        }))
        with self.assertRaises(LMStudioProtocolError):
            planner.decide(context())

    def test_context_rejects_invalid_json_and_action_sets(self):
        invalid = (
            {"observation": {"value": float("nan")}},
            {"observation": {"value": object()}},
            {"available_actions": ("DRIVE_FORWARD", "DRIVE_FORWARD")},
            {"available_actions": ("COMPLETE",)},
            {"available_actions": ([],)},
            {"history": ({"value": object()},)},
        )
        for changes in invalid:
            with self.subTest(changes=changes), self.assertRaises(
                LMStudioInputError
            ):
                context(**changes)

    def test_rejects_invalid_completion_envelopes(self):
        valid = {
            "action": "DRIVE_FORWARD",
            "confidence_milli": 900,
            "assessment": "Fortsätt.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": None,
        }
        invalid = (
            b"{}",
            completion(valid, model="other/model"),
            completion(valid, choices=[]),
            completion(valid, choices=[{
                "index": False,
                "finish_reason": "stop",
                "message": {
                    "role": "assistant",
                    "content": json.dumps(valid),
                },
            }]),
            completion({**valid, "confidence_milli": True}),
            completion({**valid, "extra": "no"}),
            completion({**valid, "utterance": ""}),
        )
        for response in invalid:
            with self.subTest(response=response[:80]):
                planner, _ = self.planner(response)
                with self.assertRaises(LMStudioProtocolError):
                    planner.decide(context())


if __name__ == "__main__":
    unittest.main()
