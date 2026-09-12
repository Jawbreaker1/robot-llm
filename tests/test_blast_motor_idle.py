"""Exercise the actual hub idle-release code without physical motors."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class BlastMotorIdleTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / "hub_programs/blast_01/runtime.py"
        tree = ast.parse(path.read_text())
        selected = [node for node in tree.body if (
            isinstance(node, ast.FunctionDef) and node.name == "poll_background_tasks"
            or isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name)
                and target.id in ("MOTOR_SETTLE_MS", "motor_release_at_ms")
                for target in node.targets
            )
        )]
        # Run the real dispatch hook too: stop and already-completed poses must
        # schedule release even when no poll ever observes a moving motor.
        dispatch = next(node for node in tree.body if isinstance(node, ast.While))
        self.schedule_node = next(node for node in dispatch.body[1].body if (
            isinstance(node, ast.If) and any(
                isinstance(child, ast.Name) and child.id == "motor_release_at_ms"
                for child in ast.walk(node)
            )
        ))
        self.motors = {role: Mock() for role in ("left", "right", "body", "claw")}
        for motor in self.motors.values():
            motor.done.return_value = True
        self.clock = Mock()
        self.clock.time.return_value = 1000
        self.speaker = Mock()
        self.ns = {"motors": self.motors, "clock": self.clock,
                   "sampled_audio_supported": True,
                   "hub": SimpleNamespace(speaker=self.speaker)}
        exec(compile(ast.Module(selected, []), str(path), "exec"), self.ns)

    def schedule(self, operation):
        self.ns["operation"] = operation
        exec(compile(ast.Module([self.schedule_node], []), "dispatch", "exec"), self.ns)

    def test_motor_commands_release_once_after_settling_including_stop_and_hold(self):
        for operation in ("stop", "drive_pulse", "turn_pulse", "turn_trim_pulse",
                          "scan_turn_pulse", "scan_trim_pulse", "body_pulse",
                          "claw_pulse", "set_pose"):
            with self.subTest(operation=operation):
                self.setUp()
                self.schedule(operation)
                self.ns["poll_background_tasks"]()
                for motor in self.motors.values():
                    motor.stop.assert_not_called()
                self.clock.time.return_value += self.ns["MOTOR_SETTLE_MS"]
                self.ns["poll_background_tasks"]()
                self.ns["poll_background_tasks"]()
                for motor in self.motors.values():
                    motor.stop.assert_called_once_with()

    def test_active_motor_defers_release_of_every_motor_and_audio_still_runs(self):
        self.ns["motor_release_at_ms"] = 1000
        self.motors["body"].done.return_value = False
        self.clock.time.return_value = 2000
        self.ns["poll_background_tasks"]()
        self.motors["body"].done.return_value = True
        self.ns["poll_background_tasks"]()
        for motor in self.motors.values():
            motor.stop.assert_not_called()
        self.clock.time.return_value += self.ns["MOTOR_SETTLE_MS"]
        self.ns["poll_background_tasks"]()
        for motor in self.motors.values():
            motor.stop.assert_called_once_with()
        self.assertEqual(self.speaker.done.call_count, 3)

    def test_observation_and_speech_do_not_restart_release_timer(self):
        for operation in ("ping", "observe", "show_face", "play_pcm"):
            self.schedule(operation)
            self.assertIsNone(self.ns["motor_release_at_ms"])
        self.ns["poll_background_tasks"]()
        for motor in self.motors.values():
            motor.stop.assert_not_called()


if __name__ == "__main__":
    unittest.main()
