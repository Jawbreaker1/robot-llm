"""Test the real hub accessory functions without starting physical hardware."""
import ast
import runpy
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock, call, patch


class BlastExpressionTests(unittest.TestCase):
    def test_only_sensor_arm_gets_native_return_position_settings(self):
        path = Path(__file__).resolve().parents[1] / "hub_programs/blast_01/runtime.py"
        tree = ast.parse(path.read_text())
        settings = [node for node in tree.body if isinstance(node, ast.Expr)
                    and isinstance(node.value, ast.Call)
                    and isinstance(node.value.func, ast.Attribute)
                    and node.value.func.attr in ("pid", "target_tolerances")]
        exec(compile(ast.Module(settings, []), str(path), "exec"), {"motors": self.motors})
        self.motors["body"].control.pid.assert_called_once_with(integral_deadzone=1)
        self.motors["body"].control.target_tolerances.assert_called_once_with(position=1)
        for role in ("left_drive", "right_drive", "claw"):
            self.assertEqual(self.motors[role].control.mock_calls, [])

    def setUp(self):
        path = Path(__file__).resolve().parents[1] / "hub_programs/blast_01/runtime.py"
        tree = ast.parse(path.read_text())
        selected = [node for node in tree.body if (
            isinstance(node, ast.FunctionDef) and node.name in ("set_pose", "show_face")
            or isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id in ("FACE_PATTERNS", "IDLE_EYE_PATTERNS")
                for target in node.targets
            )
        )]
        self.motors = {name: Mock() for name in ("body", "claw", "left_drive", "right_drive")}
        for motor in self.motors.values():
            motor.done.return_value = True
            motor.angle.return_value = 158
        self.display = Mock()
        self.light = Mock()
        self.namespace = {
            "motors": self.motors,
            "hub": SimpleNamespace(display=self.display, light=self.light),
            "Stop": SimpleNamespace(BRAKE="brake"),
            "Side": SimpleNamespace(RIGHT="right"),
            "Color": SimpleNamespace(GREEN="green"),
        }
        exec(compile(ast.Module(selected, []), str(path), "exec"), self.namespace)

    def test_pose_returns_to_existing_encoder_reference_without_reset(self):
        self.motors["body"].angle.return_value = 188
        result = self.namespace["set_pose"]("body", 158)
        self.assertEqual(result["before_angle_deg"], 188)
        self.assertEqual(result["target_angle_deg"], 158)
        self.motors["body"].run_target.assert_called_once_with(
            500, 158, then="brake", wait=False,
        )
        for motor in self.motors.values():
            motor.reset_angle.assert_not_called()
        for role in ("left_drive", "right_drive", "claw"):
            self.motors[role].run_target.assert_not_called()

    def test_claw_pose_does_not_move_body_or_wheels(self):
        self.namespace["set_pose"]("claw", 203)
        self.motors["claw"].run_target.assert_called_once()
        self.motors["body"].run_target.assert_not_called()
        self.motors["left_drive"].run_target.assert_not_called()
        self.motors["right_drive"].run_target.assert_not_called()

    def test_rejects_drive_motors_invalid_targets_and_busy_motion(self):
        for role, target in (("left_drive", 0), ("body", True), ("body", 1.5),
                             ("body", 1100), ("claw", 400)):
            with self.subTest(role=role, target=target), self.assertRaises(ValueError):
                self.namespace["set_pose"](role, target)
        self.motors["right_drive"].done.return_value = False
        with self.assertRaisesRegex(ValueError, "idle"):
            self.namespace["set_pose"]("body", 158)
        for motor in self.motors.values():
            motor.run_target.assert_not_called()

    def test_faces_are_explicit_five_by_five_images_without_motion(self):
        for expression in self.namespace["FACE_PATTERNS"]:
            self.namespace["show_face"](expression)
            self.display.orientation.assert_called_with(up="right")
            image = self.display.icon.call_args.args[0]
            self.assertEqual(len(image), 5)
            self.assertTrue(all(len(row) == 5 for row in image))
            self.assertTrue(all(pixel in (0, 100) for row in image for pixel in row))
        with self.assertRaises(ValueError):
            self.namespace["show_face"]("unknown")
        self.assertEqual(self.namespace["FACE_PATTERNS"]["angry"],
                         ("00000", "01110", "10001", "10001", "00000"))
        self.assertEqual(self.namespace["FACE_PATTERNS"]["neutral"][1:4],
                         ("11011",) * 3)
        for motor in self.motors.values():
            motor.run_target.assert_not_called()

    def test_geared_arm_accepts_full_excursion_but_claw_does_not(self):
        self.namespace["set_pose"]("body", 958)
        self.motors["body"].run_target.assert_called_once_with(
            500, 958, then="brake", wait=False,
        )
        with self.assertRaises(ValueError):
            self.namespace["set_pose"]("claw", 958)

    def test_idle_is_local_animation_replaced_by_next_expression(self):
        self.namespace["show_face"]("idle")
        frames = self.display.animate.call_args.args[0]
        self.assertEqual(self.display.animate.call_args.kwargs, {"interval": 150})
        self.assertEqual(len(frames), 77)
        self.assertEqual(len({str(frame) for frame in frames}), 4)
        for frame in frames:
            self.assertEqual(len(frame), 5)
            self.assertTrue(all(len(row) == 5 for row in frame))
        self.display.icon.assert_not_called()
        self.display.reset_mock()
        self.namespace["show_face"]("angry")
        self.display.off.assert_called_once()
        self.display.icon.assert_called_once()
        self.display.animate.assert_not_called()
        self.namespace["show_face"]("neutral")
        self.light.on.assert_not_called()
        self.light.off.assert_not_called()
        for motor in self.motors.values():
            motor.run_target.assert_not_called()

    def test_startup_claims_matrix_for_idle_eyes_before_ready(self):
        path = Path(__file__).resolve().parents[1] / "hub_programs/blast_01/runtime.py"
        body = ast.parse(path.read_text()).body
        startup = [node for node in body if isinstance(node, ast.Expr)
                   and isinstance(node.value, ast.Call)]
        ready = next(i for i, node in enumerate(startup)
                     if isinstance(node.value.func, ast.Name)
                     and node.value.func.id == "emit")
        exec(compile(ast.Module(startup[ready - 2:ready], []), str(path), "exec"),
             self.namespace)
        self.display.animate.assert_called_once()
        self.display.orientation.assert_called_once_with(up="right")
        self.light.on.assert_called_once_with("green")
        for motor in self.motors.values():
            motor.run_target.assert_not_called()


class BlastExpressionSpeechProbeTests(unittest.IsolatedAsyncioTestCase):
    async def test_arm_gestures_share_executor_and_restore_navigation_reference(self):
        from robot_agent.blast_observation_monitor import BlastObservationMonitor
        for gesture, poses in (
            ("arm_wave", [("body", -442), ("body", 158)]),
            ("claw_flourish", [("body", -442), ("claw", 200), ("claw", 325),
                               ("claw", 200), ("claw", 325), ("claw", 200), ("body", 158)]),
        ):
            monitor = BlastObservationMonitor()
            observed = {"motion_active": False, "motor_angles_deg": {"claw": 189, "body": 155}}
            runtime = Mock(observe=AsyncMock(return_value=observed),
                           set_pose=AsyncMock(return_value={"accepted": True}))
            monitor._observe_until_idle = AsyncMock(return_value=observed)
            result = await monitor._perform_command(runtime, 1, gesture)
            self.assertTrue(result["completed"])
            self.assertEqual(runtime.mock_calls, [call.observe(), *[call.set_pose(*pose) for pose in poses]])

    async def test_monitor_claw_gesture_uses_fixed_endpoints_twice_and_respects_stop(self):
        from robot_agent.blast_observation_monitor import BlastObservationMonitor, BlastControllerError
        monitor = BlastObservationMonitor()
        runtime = Mock()
        observation = {"motion_active": False, "motor_angles_deg": {
            "claw": 325, "body": 158, "left_drive": -16, "right_drive": 0,
        }}
        runtime.observe = AsyncMock(return_value=observation)
        runtime.set_pose = AsyncMock(return_value={"accepted": True})
        monitor._observe_until_idle = AsyncMock(return_value=observation)
        result = await monitor._perform_command(runtime, 1, "claw_snap")
        self.assertTrue(result["completed"])
        self.assertEqual(runtime.mock_calls, [call.observe(), call.set_pose("claw", 200),
                         call.set_pose("claw", 325), call.set_pose("claw", 200),
                         call.set_pose("claw", 325), call.set_pose("claw", 200)])
        runtime.reset_mock()
        monitor._observe_until_idle.side_effect = BlastControllerError("controller_command_interrupted", "Stop")
        with self.assertRaises(BlastControllerError):
            await monitor._perform_command(runtime, 1, "claw_snap")
        runtime.set_pose.assert_called_once_with("claw", 200)

    async def test_audio_is_preloaded_before_face_and_playback_start(self):
        script = Path(__file__).resolve().parents[1] / "scripts/probe_blast_expressions.py"
        present = runpy.run_path(str(script))["present_face"]
        runtime = Mock()
        runtime.begin_pcm = AsyncMock(return_value={
            "transfer_id": 7, "batch_bytes": 2, "fletcher16": 42,
        })
        runtime.write_pcm_batch = AsyncMock()
        runtime.show_face = AsyncMock(return_value={"accepted": True})
        runtime.start_pcm = AsyncMock(return_value={"duration_ms": 2000})
        audio = SimpleNamespace(payload=b"abcd")
        with patch("builtins.print"):
            self.assertEqual(await present(runtime, "idle", audio), 2)
        self.assertEqual(runtime.mock_calls, [
            call.begin_pcm(b"abcd"), call.write_pcm_batch(0, b"ab"),
            call.write_pcm_batch(2, b"cd"), call.show_face("idle"),
            call.start_pcm(7, 4, 42),
        ])
        runtime.reset_mock()
        with patch("builtins.print"):
            self.assertEqual(await present(runtime, "neutral"), 0)
        self.assertEqual(runtime.mock_calls, [call.show_face("neutral")])


if __name__ == "__main__":
    unittest.main()
