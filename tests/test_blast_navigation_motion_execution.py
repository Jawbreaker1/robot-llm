import unittest

from robot_agent.blast_navigation_motion_execution import (
    MAX_RESTORED_SCAN_COMMON_MODE_RESIDUE_DEGREES,
    MAX_RESTORED_SCAN_OPPOSED_RESIDUE_DEGREES,
    BlastNavigationMotionExecutor,
)
from robot_agent.blast_observation_monitor import (
    BlastControllerError,
    COMMAND_RESULT_SCHEMA,
    CONTROLLER_ID,
    ROBOT_ID,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    PhysicalNavigationContractError,
)


class FakeController:
    def __init__(self):
        self.commands = []
        self.angles = {"left_drive": 100, "right_drive": 200}
        self.next_gap = None
        self._left_turn_pulse = 0

    def observation(self):
        return {
            "motion_active": False,
            "motor_angles_deg": {
                **self.angles,
                "claw": 0,
                "body": 0,
            },
        }

    def snapshot(self):
        return {"observation": self.observation()}

    def command(self, command, *, cancel_requested=None):
        self.commands.append(command)
        if self.next_gap is not None:
            for role, delta in zip(
                ("left_drive", "right_drive"),
                self.next_gap,
            ):
                self.angles[role] += delta
            self.next_gap = None
        before = dict(self.angles)
        if command == "drive_forward":
            direction, speed, angle_field, angle = (
                "forward", 120, "angle_deg", 90
            )
            deltas = (90, 90)
        elif command == "drive_reverse":
            direction, speed, angle_field, angle = (
                "reverse", 120, "angle_deg", 90
            )
            deltas = (-90, -90)
        elif command == "turn_left":
            direction, speed, angle_field, angle = (
                "left", 180, "wheel_angle_deg", 45
            )
            # Four live-like pulses total -188/+198 = 193 opposed degrees.
            right = (49, 49, 50, 50)[self._left_turn_pulse % 4]
            self._left_turn_pulse += 1
            deltas = (-47, right)
        elif command == "turn_right":
            direction, speed, angle_field, angle = (
                "right", 180, "wheel_angle_deg", 45
            )
            deltas = (49, -47)
        else:
            raise AssertionError("unexpected command")
        for role, delta in zip(("left_drive", "right_drive"), deltas):
            self.angles[role] += delta
        return {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": command,
            "accepted": True,
            "completed": True,
            "receipt": {
                "accepted": True,
                "direction": direction,
                "speed_dps": speed,
                angle_field: angle,
                "before_angles_deg": before,
            },
            "observation": self.observation(),
            "observation_settled": True,
        }


def scan_result(controller, *, restored):
    return {
        "schema": COMMAND_RESULT_SCHEMA,
        "robot_id": ROBOT_ID,
        "controller_id": CONTROLLER_ID,
        "command": "scan_front_arc",
        "accepted": True,
        "completed": True,
        "receipt": {"turn_count": 8},
        "observation": controller.observation(),
        "observation_settled": True,
        "scan": {"restoration_verified": restored},
    }


class BlastNavigationMotionExecutorTests(unittest.TestCase):
    def executor(self):
        controller = FakeController()
        return controller, BlastNavigationMotionExecutor(
            controller=controller,
            initial_observation=controller.observation(),
        )

    def test_advance_dispatches_once_and_applies_shared_odometry(self):
        controller, executor = self.executor()

        result = executor.execute(ADVANCE)

        self.assertEqual(controller.commands, ["drive_forward"])
        self.assertTrue(result.motion.complete)
        self.assertEqual(result.pose.x_mm, 45)
        self.assertEqual(result.pose.y_mm, 0)
        self.assertEqual(
            executor.expected_start_angles,
            {"left_drive": 190, "right_drive": 290},
        )

    def test_quarter_turn_dispatches_four_pulses_and_uses_calibration(self):
        controller, executor = self.executor()

        result = executor.execute(TURN_LEFT_90)

        self.assertEqual(controller.commands, ["turn_left"] * 4)
        self.assertTrue(result.motion.complete)
        self.assertEqual(result.motion.verified_slice_count, 4)
        self.assertEqual(result.pose.heading_mdeg, 94_570)

    def test_inter_slice_settling_is_retained_but_not_command_verified(self):
        controller, executor = self.executor()
        original_command = controller.command
        calls = 0

        def command(name, *, cancel_requested=None):
            nonlocal calls
            if calls == 1:
                controller.next_gap = (-1, 0)
            calls += 1
            return original_command(
                name,
                cancel_requested=cancel_requested,
            )

        controller.command = command

        result = executor.execute(TURN_LEFT_90)

        self.assertTrue(result.motion.complete)
        self.assertEqual(result.motion.verified_slice_count, 4)
        self.assertEqual(result.motion.observed_slice_count, 5)
        self.assertEqual(result.pose.heading_mdeg, 94_815)

    def test_opposite_single_degree_backlash_remains_localizable(self):
        controller, executor = self.executor()
        original_command = controller.command
        calls = 0

        def command(name, *, cancel_requested=None):
            nonlocal calls
            if calls == 2:
                controller.next_gap = (-1, 0)
            calls += 1
            return original_command(
                name,
                cancel_requested=cancel_requested,
            )

        controller.command = command

        result = executor.execute(TURN_RIGHT_90)

        self.assertTrue(result.motion.complete)
        self.assertEqual(result.motion.observed_slice_count, 5)
        self.assertEqual(result.pose.heading_mdeg, -93_835)
        self.assertTrue(executor.localization_valid)

    def test_single_degree_receipt_gap_across_actions_is_localizable(self):
        controller, executor = self.executor()
        executor.execute(TURN_LEFT_90)
        controller.next_gap = (0, 1)

        result = executor.execute(ADVANCE)

        self.assertTrue(result.motion.complete)
        self.assertEqual(result.motion.observed_slice_count, 2)
        self.assertEqual(
            result.motion.segments[0].kind,
            "inter_action_settling",
        )
        self.assertEqual(
            (
                result.motion.left_encoder_delta_degrees,
                result.motion.right_encoder_delta_degrees,
            ),
            (90, 91),
        )
        self.assertEqual(executor.expected_start_angles, controller.angles)
        self.assertTrue(executor.localization_valid)

    def test_single_degree_pre_command_settling_is_localizable(self):
        controller, executor = self.executor()
        executor.execute(TURN_LEFT_90)
        controller.angles["right_drive"] += 1

        result = executor.execute(ADVANCE)

        self.assertTrue(result.motion.complete)
        self.assertEqual(result.motion.observed_slice_count, 2)
        self.assertEqual(
            result.motion.segments[0].kind,
            "inter_action_settling",
        )
        self.assertEqual(
            (
                result.motion.left_encoder_delta_degrees,
                result.motion.right_encoder_delta_degrees,
            ),
            (90, 91),
        )
        self.assertEqual(executor.expected_start_angles, controller.angles)
        self.assertTrue(executor.localization_valid)

    def test_accumulated_pre_command_and_receipt_gap_latches(self):
        controller, executor = self.executor()
        executor.execute(TURN_LEFT_90)
        controller.angles["right_drive"] += 1
        controller.next_gap = (0, 1)
        command_count = len(controller.commands)
        trusted_pose = executor.pose
        trusted_anchor = executor.expected_start_angles

        with self.assertRaises(PhysicalNavigationContractError) as raised:
            executor.execute(ADVANCE)

        self.assertEqual(
            raised.exception.code,
            "blast_motion_slice_discontinuous",
        )
        self.assertEqual(len(controller.commands), command_count + 1)
        self.assertEqual(executor.pose, trusted_pose)
        self.assertEqual(executor.expected_start_angles, trusted_anchor)
        self.assertFalse(executor.localization_valid)

    def test_two_degree_receipt_gap_across_actions_latches_localization(self):
        controller, executor = self.executor()
        executor.execute(TURN_LEFT_90)
        controller.next_gap = (0, 2)

        with self.assertRaises(PhysicalNavigationContractError) as raised:
            executor.execute(ADVANCE)

        self.assertEqual(
            raised.exception.code,
            "blast_motion_slice_discontinuous",
        )
        self.assertFalse(executor.localization_valid)

    def test_two_degree_internal_gap_still_latches_localization(self):
        controller, executor = self.executor()
        original_command = controller.command
        calls = 0

        def command(name, *, cancel_requested=None):
            nonlocal calls
            if calls == 2:
                controller.next_gap = (-2, 0)
            calls += 1
            return original_command(
                name,
                cancel_requested=cancel_requested,
            )

        controller.command = command

        with self.assertRaises(PhysicalNavigationContractError) as raised:
            executor.execute(TURN_RIGHT_90)

        self.assertEqual(
            raised.exception.code,
            "blast_motion_slice_discontinuous",
        )
        self.assertFalse(executor.localization_valid)

    def test_cancelled_turn_retains_only_observed_prefix(self):
        controller, executor = self.executor()

        result = executor.execute(
            TURN_LEFT_90,
            cancel_requested=lambda: len(controller.commands) == 2,
        )

        self.assertEqual(controller.commands, ["turn_left"] * 2)
        self.assertFalse(result.motion.complete)
        self.assertEqual(result.motion.status, "interrupted")
        self.assertEqual(result.motion.verified_slice_count, 2)
        self.assertEqual(result.motion.requested_slice_count, 4)
        self.assertEqual(result.pose.heading_mdeg, 47_040)

    def test_prestart_interruption_of_next_slice_retains_verified_prefix(self):
        controller, executor = self.executor()
        original_command = controller.command
        invocations = 0

        def interrupt_second(name, *, cancel_requested=None):
            nonlocal invocations
            invocations += 1
            if invocations == 2:
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "cancelled before motor start",
                    motion_started=False,
                )
            return original_command(
                name, cancel_requested=cancel_requested,
            )

        controller.command = interrupt_second

        result = executor.execute(TURN_LEFT_90)

        self.assertEqual(invocations, 2)
        self.assertEqual(controller.commands, ["turn_left"])
        self.assertFalse(result.motion.complete)
        self.assertEqual(result.motion.verified_slice_count, 1)
        self.assertEqual(result.pose.heading_mdeg, 23_520)
        self.assertTrue(executor.localization_valid)

    def test_turn_continuation_gate_stops_after_close_slice(self):
        controller, executor = self.executor()
        original_command = controller.command

        def command(name, *, cancel_requested=None):
            result = original_command(
                name,
                cancel_requested=cancel_requested,
            )
            result["observation"]["distance_mm"] = 40
            return result

        controller.command = command

        result = executor.execute(
            TURN_LEFT_90,
            continue_requested=lambda command_result: (
                command_result["observation"]["distance_mm"] > 53
            ),
        )

        self.assertEqual(controller.commands, ["turn_left"])
        self.assertFalse(result.motion.complete)
        self.assertEqual(result.motion.verified_slice_count, 1)
        self.assertEqual(result.pose.heading_mdeg, 23_520)

    def test_invalid_first_turn_receipt_stops_before_second_pulse(self):
        for corruption in ("command", "encoder_direction"):
            with self.subTest(corruption=corruption):
                controller, executor = self.executor()
                original_command = controller.command

                def command(name, *, cancel_requested=None):
                    result = original_command(
                        name,
                        cancel_requested=cancel_requested,
                    )
                    if corruption == "command":
                        result["command"] = "turn_right"
                    else:
                        before = result["receipt"]["before_angles_deg"]
                        result["observation"]["motor_angles_deg"].update({
                            "left_drive": before["left_drive"] + 47,
                            "right_drive": before["right_drive"] - 49,
                        })
                    return result

                controller.command = command

                with self.assertRaises(PhysicalNavigationContractError):
                    executor.execute(
                        TURN_LEFT_90,
                        continue_requested=lambda _result: True,
                    )

                self.assertEqual(controller.commands, ["turn_left"])
                self.assertFalse(executor.localization_valid)

    def test_cross_action_encoder_gap_fails_closed_without_advancing_pose(self):
        controller, executor = self.executor()
        first = executor.execute(ADVANCE)
        controller.angles["left_drive"] += 2
        controller.angles["right_drive"] += 2
        command_count = len(controller.commands)

        with self.assertRaises(PhysicalNavigationContractError) as raised:
            executor.execute(ADVANCE)

        self.assertEqual(
            raised.exception.code,
            "blast_motion_slice_discontinuous",
        )
        self.assertEqual(
            str(raised.exception),
            (
                "BLAST encoders changed outside a verified motion action: "
                "expected=(190, 290) observed=(192, 292) delta=(2, 2)"
            ),
        )
        self.assertEqual(executor.pose, first.pose)
        self.assertFalse(executor.localization_valid)
        self.assertEqual(len(controller.commands), command_count)
        self.assertEqual(
            executor.expected_start_angles,
            {"left_drive": 190, "right_drive": 290},
        )
        with self.assertRaises(PhysicalNavigationContractError) as latched:
            executor.execute(ADVANCE)
        self.assertEqual(
            latched.exception.code,
            "blast_navigation_localization_invalid",
        )
        self.assertEqual(len(controller.commands), command_count)

    def test_cancel_before_first_pulse_uses_controller_interruption(self):
        controller, executor = self.executor()

        with self.assertRaises(BlastControllerError) as raised:
            executor.execute(TURN_LEFT_90, cancel_requested=lambda: True)

        self.assertEqual(
            getattr(raised.exception, "code", None),
            "controller_command_interrupted",
        )
        self.assertTrue(executor.localization_valid)
        self.assertEqual(controller.commands, [])

    def test_invalid_native_result_latches_localization(self):
        controller, executor = self.executor()
        controller.command = lambda *_args, **_kwargs: None

        with self.assertRaises(PhysicalNavigationContractError) as raised:
            executor.execute(ADVANCE)

        self.assertEqual(
            raised.exception.code,
            "blast_command_result_invalid",
        )
        self.assertFalse(executor.localization_valid)

    def test_restored_scan_reanchors_encoders_without_claiming_pose(self):
        controller, executor = self.executor()
        controller.angles["left_drive"] += 8
        controller.angles["right_drive"] += 8
        restored = scan_result(controller, restored=True)

        reanchored = executor.reanchor_after_restored_scan(restored)
        pose_after_scan = executor.pose
        result = executor.execute(TURN_LEFT_90)

        self.assertTrue(reanchored)
        self.assertEqual(pose_after_scan.x_mm, 0)
        self.assertEqual(pose_after_scan.y_mm, 0)
        self.assertEqual(result.pose.heading_mdeg, 94_570)

    def test_live_scan_common_mode_residue_reanchors_without_false_pose(self):
        controller, executor = self.executor()
        controller.angles["left_drive"] += 16
        controller.angles["right_drive"] += 10

        reanchored = executor.reanchor_after_restored_scan(
            scan_result(controller, restored=True)
        )

        self.assertTrue(reanchored)
        self.assertEqual(executor.pose.x_mm, 0)
        self.assertEqual(executor.pose.y_mm, 0)
        self.assertTrue(executor.localization_valid)

    def test_unrestored_scan_residue_blocks_motion_before_command(self):
        controller, executor = self.executor()
        controller.angles["left_drive"] += 8
        controller.angles["right_drive"] += 8

        with self.assertRaises(PhysicalNavigationContractError) as raised:
            executor.reanchor_after_restored_scan(
                scan_result(controller, restored=False)
            )

        self.assertEqual(
            raised.exception.code,
            "blast_scan_restoration_unverified",
        )
        self.assertFalse(executor.localization_valid)
        self.assertEqual(controller.commands, [])

    def test_excessive_common_mode_scan_residue_is_not_discarded(self):
        controller, executor = self.executor()
        excessive = MAX_RESTORED_SCAN_COMMON_MODE_RESIDUE_DEGREES + 1
        controller.angles["left_drive"] += excessive
        controller.angles["right_drive"] += excessive

        with self.assertRaises(PhysicalNavigationContractError) as raised:
            executor.reanchor_after_restored_scan(
                scan_result(controller, restored=True)
            )

        self.assertEqual(
            raised.exception.code,
            "blast_scan_encoder_residue_excessive",
        )
        self.assertFalse(executor.localization_valid)

    def test_excessive_opposed_scan_residue_is_not_discarded(self):
        controller, executor = self.executor()
        excessive = MAX_RESTORED_SCAN_OPPOSED_RESIDUE_DEGREES + 1
        controller.angles["left_drive"] += excessive
        controller.angles["right_drive"] -= excessive

        with self.assertRaises(PhysicalNavigationContractError) as raised:
            executor.reanchor_after_restored_scan(
                scan_result(controller, restored=True)
            )

        self.assertEqual(
            raised.exception.code,
            "blast_scan_encoder_residue_excessive",
        )
        self.assertFalse(executor.localization_valid)

    def test_controller_failure_latches_localization(self):
        controller, executor = self.executor()
        original = controller.command

        def fail_on_third(command, *, cancel_requested=None):
            if len(controller.commands) == 2:
                raise RuntimeError("lost receipt")
            return original(command, cancel_requested=cancel_requested)

        controller.command = fail_on_third
        with self.assertRaisesRegex(RuntimeError, "lost receipt"):
            executor.execute(TURN_LEFT_90)

        self.assertEqual(controller.commands, ["turn_left"] * 2)
        self.assertFalse(executor.localization_valid)
        with self.assertRaises(PhysicalNavigationContractError) as raised:
            executor.execute(ADVANCE)
        self.assertEqual(
            raised.exception.code,
            "blast_navigation_localization_invalid",
        )

    def test_scripted_l_shape_carries_pose_between_semantic_actions(self):
        controller, executor = self.executor()

        for action in (
            ADVANCE,
            ADVANCE,
            TURN_LEFT_90,
            ADVANCE,
            ADVANCE,
        ):
            executor.execute(action)

        self.assertEqual(
            controller.commands,
            ["drive_forward"] * 2
            + ["turn_left"] * 4
            + ["drive_forward"] * 2,
        )
        self.assertLessEqual(abs(executor.pose.x_mm - 90), 10)
        self.assertLessEqual(abs(executor.pose.y_mm - 90), 10)
        self.assertEqual(executor.pose.verified_motion_count, 5)


if __name__ == "__main__":
    unittest.main()
