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
    def test_disconnected_navigation_preserves_named_conversation_and_speech(self):
        from unittest.mock import Mock
        from robot_agent.robot_control_contract import RobotControlTarget
        from robot_agent.robot_control_service import RobotControlService
        control = RobotControlService(
            target=RobotControlTarget("ev3rstorm-01", "EV3RSTORM"),
            disabled_reason="EV3 navigation is temporarily unavailable.",
        )
        self.addCleanup(control.shutdown)
        seen = []
        sink = Mock(return_value=True)
        service = RobotInputService(
            control_service=control,
            model_factory=lambda _: Model(RobotInputDecision(CONVERSE, 950, "Still here. Still grumpy."), seen),
            speech_sink=sink,
        )
        turn = service.dispatch("Hello EV3", "en", "disabled-chat-1", control.settings()["revision"])
        self.assertEqual(turn["answer_text"], "Still here. Still grumpy.")
        self.assertTrue(turn["speech_queued"])
        sink.assert_called_once_with("disabled-chat-1", "Still here. Still grumpy.", "en")
        self.assertIsNone(turn["episode"])
        self.assertFalse(seen[0][1]["control"]["enabled"])
        self.assertEqual(seen[0][1]["control"]["target"]["robot_id"], "ev3rstorm-01")
        self.assertIn("temporarily unavailable", seen[0][1]["control"]["runtime"]["message"])
        with self.assertRaisesRegex(RobotControlServiceError, "temporarily unavailable"):
            control.start("Drive forward", "en", "disabled-drive-1", control.settings()["revision"])

    def test_hardware_description_reaches_model_without_granting_arm_actions(self):
        control = FakeControl()
        description = {"robot_id": "ev3rstorm-01", "capabilities": ["motor.arm"]}
        seen = []
        decision = RobotInputDecision(UNSUPPORTED_PHYSICAL_TASK, 950,
                                     "I have an arm, but cannot wave it from this interface yet.")
        service = RobotInputService(control_service=control, robot_description=description,
                                    model_factory=lambda _: Model(decision, seen))
        description["capabilities"].clear()
        turn = service.dispatch("Wave your arm", "en", "arm-1", 3)
        self.assertEqual(seen[0][1]["robot_description"]["capabilities"], ["motor.arm"])
        self.assertEqual(turn["intent"], UNSUPPORTED_PHYSICAL_TASK)
        self.assertEqual(control.started, [])
        seen[0][1]["robot_description"]["capabilities"].clear()
        self.assertEqual(service._facts()["robot_description"]["capabilities"], ["motor.arm"])

    def test_expression_is_queued_once_with_reply_and_never_starts_navigation(self):
        from unittest.mock import Mock
        expression = {"face": "happy", "gesture": "claw_snap"}
        control = FakeControl()
        sink = Mock(return_value=True)
        service = RobotInputService(
            control_service=control, speech_sink=sink,
            model_factory=lambda _: Model(RobotInputDecision(
                CONVERSE, 950, "Watch this!", expression=expression,
            ), []),
        )
        first = service.dispatch("Snap your claw", "en", "social-1", 3)
        second = service.dispatch("Snap your claw", "en", "social-1", 3)
        self.assertEqual(first, second)
        self.assertEqual(first["expression"], expression)
        sink.assert_called_once_with("social-1", "Watch this!", "en", expression=expression)
        self.assertEqual(control.started, [])

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
        self.assertTrue(turn["fallback"])
        self.assertIn("tekniskt fel", turn["answer_text"])
        self.assertNotIn("förtydliga", turn["answer_text"])
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
