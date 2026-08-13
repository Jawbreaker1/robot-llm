import threading
import unittest

from robot_agent.lm_studio_robot_input import (
    CLARIFY,
    CONVERSE,
    PHYSICAL_TASK,
    READ_ONLY_TASK,
    STOP_TASK,
    UNSUPPORTED_PHYSICAL_TASK,
    RobotInputDecision,
)
from robot_agent.robot_control_service import RobotControlServiceError
from robot_agent.robot_input_service import RobotInputService


class FakeControl:
    def __init__(self, state="IDLE"):
        self.state = state
        self.started = []
        self.stopped = 0

    def status(self):
        return {
            "state": self.state,
            "episode": {
                "episode_id": (
                    "episode-1" if self.state == "RUNNING" else None
                ),
                "goal": (
                    "Explore the room" if self.state == "RUNNING" else None
                ),
                "locale": "en" if self.state == "RUNNING" else None,
            },
            "runtime": {
                "current_action": "ADVANCE" if self.state == "RUNNING" else None,
                "message": "Passing the obstacle",
            },
        }

    def settings(self):
        return {
            "revision": 3,
            "model": "gemma-test",
            "speech_enabled": True,
        }

    def start(self, text, locale, request_id, expected_revision):
        if self.state != "IDLE":
            raise RobotControlServiceError(
                409,
                "robot_episode_active",
                "A robot episode is already active",
            )
        self.started.append((text, locale, request_id, expected_revision))
        return {
            "accepted_episode_id": "episode-1",
            "idempotent": False,
            "control": {"state": "STARTING"},
        }

    def stop(self):
        self.stopped += 1
        return {"state": "STOPPING", "sequence": 7}


class Model:
    def __init__(self, decision, seen):
        self.decision = decision
        self.seen = seen

    def interpret(self, input_value, facts):
        self.seen.append((input_value, facts))
        return self.decision


class RobotInputServiceTests(unittest.TestCase):
    def service(self, control, decision, *, spoken=None, seen=None):
        spoken = [] if spoken is None else spoken
        seen = [] if seen is None else seen
        service = RobotInputService(
            control_service=control,
            model_factory=lambda model: Model(decision, seen),
            spatial_map_provider=type(
                "Map",
                (),
                {
                    "snapshot": lambda _self: {
                        "observed_age_ms": 250,
                        "qualitative_observations": [
                            {"relation": "front_blocked"}
                        ],
                    }
                },
            )(),
            speech_sink=lambda request_id, text, locale: (
                spoken.append((request_id, text, locale)) or True
            ),
            clock_ms=lambda: 5_000,
        )
        return service, spoken, seen

    def test_read_only_question_works_while_navigation_runs(self):
        control = FakeControl("RUNNING")
        decision = RobotInputDecision(
            READ_ONLY_TASK,
            930,
            "Jag kör runt hindret, lugna ner dig.",
        )
        service, spoken, seen = self.service(control, decision)

        turn = service.dispatch("Hur går det?", "sv", "request-1", 3)

        self.assertEqual(turn["intent"], READ_ONLY_TASK)
        self.assertEqual(turn["answer_text"], decision.reply_text)
        self.assertTrue(turn["speech_queued"])
        self.assertEqual(control.started, [])
        self.assertEqual(seen[0][1]["control"]["state"], "RUNNING")
        self.assertEqual(
            seen[0][1]["control"]["episode"]["goal"],
            "Explore the room",
        )
        self.assertFalse(seen[0][1]["camera_vision"]["available"])
        self.assertEqual(spoken[0][1], decision.reply_text)

    def test_only_physical_intent_delegates_original_text(self):
        control = FakeControl()
        decision = RobotInputDecision(PHYSICAL_TASK, 900, None)
        service, spoken, _seen = self.service(control, decision)

        turn = service.dispatch(
            "Kör framåt och navigera runt hindret.",
            "sv",
            "request-2",
            3,
        )

        self.assertIsNone(turn["answer_text"])
        self.assertEqual(turn["episode"]["accepted_episode_id"], "episode-1")
        self.assertEqual(
            control.started,
            [(
                "Kör framåt och navigera runt hindret.",
                "sv",
                "request-2",
                3,
            )],
        )
        self.assertEqual(spoken, [])

    def test_unsupported_physical_task_never_starts_navigation(self):
        control = FakeControl()
        decision = RobotInputDecision(
            UNSUPPORTED_PHYSICAL_TASK,
            930,
            "Jag kan navigera, men jag kan inte vinka med en arm.",
        )
        service, spoken, _seen = self.service(control, decision)

        turn = service.dispatch(
            "Vinka med höger arm.",
            "sv",
            "request-unsupported",
            3,
        )

        self.assertEqual(turn["intent"], UNSUPPORTED_PHYSICAL_TASK)
        self.assertIsNone(turn["episode"])
        self.assertEqual(control.started, [])
        self.assertEqual(spoken[0][1], decision.reply_text)

    def test_nonphysical_intents_never_start_robot(self):
        for intent in (CONVERSE, READ_ONLY_TASK, CLARIFY):
            with self.subTest(intent=intent):
                control = FakeControl()
                service, _spoken, _seen = self.service(
                    control,
                    RobotInputDecision(intent, 700, "Kort svar."),
                )
                service.dispatch("Test", "sv", "request-" + intent, 3)
                self.assertEqual(control.started, [])

    def test_model_failure_fails_closed_without_motion(self):
        class BrokenModel:
            def interpret(self, _input, _facts):
                raise RuntimeError("broken")

        control = FakeControl()
        service = RobotInputService(
            control_service=control,
            model_factory=lambda _model: BrokenModel(),
            clock_ms=lambda: 1,
        )

        turn = service.dispatch("Gör något", "sv", "request-3", 3)

        self.assertEqual(turn["intent"], CLARIFY)
        self.assertIsNone(turn["episode"])
        self.assertEqual(control.started, [])

    def test_physical_input_during_navigation_returns_visible_guidance(self):
        control = FakeControl("RUNNING")
        service, spoken, _seen = self.service(
            control,
            RobotInputDecision(PHYSICAL_TASK, 900, None),
        )

        turn = service.dispatch("Sväng höger", "sv", "request-4", 3)

        self.assertEqual(turn["intent"], CLARIFY)
        self.assertIn("Be mig stoppa först", turn["answer_text"])
        self.assertTrue(turn["speech_queued"])
        self.assertEqual(control.started, [])
        self.assertEqual(spoken[0][1], turn["answer_text"])

    def test_stop_intent_uses_stop_control_path_during_navigation(self):
        control = FakeControl("RUNNING")
        service, spoken, _seen = self.service(
            control,
            RobotInputDecision(STOP_TASK, 950, None),
        )

        turn = service.dispatch("Stanna.", "sv", "request-stop", 3)

        self.assertEqual(turn["intent"], STOP_TASK)
        self.assertEqual(
            turn["control"],
            {"state": "STOPPING", "sequence": 7},
        )
        self.assertIsNone(turn["episode"])
        self.assertEqual(control.stopped, 1)
        self.assertEqual(control.started, [])
        self.assertEqual(spoken, [])

    def test_idempotent_read_only_turn_does_not_speak_twice(self):
        control = FakeControl()
        service, spoken, seen = self.service(
            control,
            RobotInputDecision(CONVERSE, 800, "Jaja, jag hör dig."),
        )

        first = service.dispatch("Hej", "sv", "request-5", 3)
        second = service.dispatch("Hej", "sv", "request-5", 3)

        self.assertEqual(first, second)
        self.assertEqual(len(spoken), 1)
        self.assertEqual(len(seen), 1)
        with self.assertRaises(RobotControlServiceError):
            service.dispatch("Annat", "sv", "request-5", 3)

    def test_duplicate_inflight_request_is_rejected(self):
        entered = threading.Event()
        release = threading.Event()

        class BlockingModel:
            def interpret(self, _input, _facts):
                entered.set()
                release.wait(1)
                return RobotInputDecision(CONVERSE, 700, "Svar")

        service = RobotInputService(
            control_service=FakeControl(),
            model_factory=lambda _model: BlockingModel(),
            clock_ms=lambda: 1,
        )
        thread = threading.Thread(
            target=lambda: service.dispatch("Hej", "sv", "same", 3)
        )
        thread.start()
        self.assertTrue(entered.wait(1))
        try:
            with self.assertRaises(RobotControlServiceError) as caught:
                service.dispatch("Hej", "sv", "same", 3)
            self.assertEqual(caught.exception.code, "robot_input_inflight")
        finally:
            release.set()
            thread.join(1)


if __name__ == "__main__":
    unittest.main()
