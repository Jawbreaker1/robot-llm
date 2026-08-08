import copy
import unittest

from robot_agent.blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from robot_agent.blast_navigation_motion_result import (
    build_blast_navigation_motion_result,
)
from robot_agent.blast_observation_monitor import (
    COMMAND_RESULT_SCHEMA,
    CONTROLLER_ID,
    ROBOT_ID,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    PhysicalNavigationContractError,
)
from robot_agent.physical_odometry import (
    DriveMotorRoles,
    PhysicalPose,
    apply_verified_motion,
    verified_motion_from_result,
)


_PROFILES = {
    "drive_forward": ("forward", 120, "angle_deg", 90, (1, 1)),
    "drive_reverse": ("reverse", 120, "angle_deg", 90, (-1, -1)),
    "turn_left": ("left", 180, "wheel_angle_deg", 45, (-1, 1)),
    "turn_right": ("right", 180, "wheel_angle_deg", 45, (1, -1)),
}


def command_result(command, before, deltas, *, settled=True):
    direction, speed, angle_field, angle, _signs = _PROFILES[command]
    after = tuple(start + delta for start, delta in zip(before, deltas))
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
            "before_angles_deg": {
                "left_drive": before[0],
                "right_drive": before[1],
            },
        },
        "observation": {
            "motion_active": False,
            "motor_angles_deg": {
                "left_drive": after[0],
                "right_drive": after[1],
                "claw": 0,
                "body": 0,
            },
        },
        "observation_settled": settled,
    }


def turn_results(command, count=4):
    _direction, _speed, _angle_field, _angle, signs = _PROFILES[command]
    results = []
    before = (100, 200)
    for _index in range(count):
        deltas = tuple(sign * 45 for sign in signs)
        result = command_result(command, before, deltas)
        results.append(result)
        angles = result["observation"]["motor_angles_deg"]
        before = (angles["left_drive"], angles["right_drive"])
    return results


def start_angles(results):
    return dict(results[0]["receipt"]["before_angles_deg"])


def canonical_observation(results):
    angles = results[-1]["observation"]["motor_angles_deg"]
    return {
        "motors": [
            {"role": "left_drive", "position": angles["left_drive"], "state": ""},
            {"role": "right_drive", "position": angles["right_drive"], "state": ""},
        ],
        "last_outcome": {},
    }


def build(action, results):
    return build_blast_navigation_motion_result(
        action,
        results,
        expected_start_angles=start_angles(results),
        canonical_observation=canonical_observation(results),
    )


class BlastNavigationMotionResultTests(unittest.TestCase):
    def test_drive_results_feed_existing_shared_odometry(self):
        calibration = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
        cases = (
            (ADVANCE, "drive_forward", (90, 90), 45),
            (REVERSE, "drive_reverse", (-90, -90), -45),
        )
        for action, command, deltas, expected_x in cases:
            with self.subTest(action=action):
                results = [command_result(command, (10, 20), deltas)]
                raw = build(action, results)
                motion = verified_motion_from_result(
                    action,
                    raw,
                    DriveMotorRoles(),
                )
                pose = apply_verified_motion(
                    PhysicalPose(),
                    motion,
                    calibration,
                )
                self.assertTrue(motion.complete)
                self.assertEqual(
                    (
                        motion.left_encoder_delta_degrees,
                        motion.right_encoder_delta_degrees,
                    ),
                    deltas,
                )
                self.assertEqual(pose.x_mm, expected_x)

    def test_four_exact_command_receipts_apply_actual_encoder_scale(self):
        calibration = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
        cases = (
            (TURN_LEFT_90, "turn_left", (-180, 180), 88_200),
            (TURN_RIGHT_90, "turn_right", (180, -180), -88_200),
        )
        for action, command, totals, expected_heading in cases:
            with self.subTest(action=action):
                results = turn_results(command)
                motion = verified_motion_from_result(
                    action,
                    build(action, results),
                )
                pose = apply_verified_motion(
                    PhysicalPose(),
                    motion,
                    calibration,
                )
                self.assertTrue(motion.complete)
                self.assertEqual(motion.observed_slice_count, 4)
                self.assertEqual(
                    (
                        motion.left_encoder_delta_degrees,
                        motion.right_encoder_delta_degrees,
                    ),
                    totals,
                )
                self.assertEqual(pose.heading_mdeg, expected_heading)

    def test_bounded_inter_slice_settling_is_preserved_in_odometry(self):
        results = turn_results("turn_left")
        third_after = results[2]["observation"]["motor_angles_deg"]
        fourth_before = (
            third_after["left_drive"] - 1,
            third_after["right_drive"],
        )
        results[3] = command_result(
            "turn_left",
            fourth_before,
            (-45, 45),
        )

        raw = build(TURN_LEFT_90, results)
        fourth = raw["outcome"]["slices"][3]
        motion = verified_motion_from_result(TURN_LEFT_90, raw)
        pose = apply_verified_motion(
            PhysicalPose(),
            motion,
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
        )

        self.assertEqual(
            [segment["kind"] for segment in fourth["segments"]],
            ["inter_slice_settling", "commanded"],
        )
        self.assertEqual(
            [
                motor["position_delta"]
                for motor in fourth["segments"][0]["motors"]
            ],
            [-1, 0],
        )
        self.assertFalse(
            fourth["segments"][0]["encoder_verification"]["passed"]
        )
        self.assertTrue(motion.complete)
        self.assertEqual(motion.observed_slice_count, 5)
        self.assertEqual(motion.verified_slice_count, 4)
        self.assertEqual(
            (
                motion.left_encoder_delta_degrees,
                motion.right_encoder_delta_degrees,
            ),
            (-181, 180),
        )
        self.assertEqual(pose.heading_mdeg, 88_445)

    def test_live_turn_receipts_keep_the_observed_settling_degree(self):
        results = [
            command_result("turn_left", (-158, 84), (-45, 51)),
            command_result("turn_left", (-203, 135), (-46, 50)),
            command_result("turn_left", (-249, 185), (-49, 48)),
            command_result("turn_left", (-299, 233), (-47, 50)),
        ]

        raw = build(TURN_LEFT_90, results)
        motion = verified_motion_from_result(TURN_LEFT_90, raw)
        pose = apply_verified_motion(
            PhysicalPose(),
            motion,
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
        )

        self.assertTrue(motion.complete)
        self.assertEqual(motion.observed_slice_count, 5)
        self.assertEqual(motion.verified_slice_count, 4)
        self.assertEqual(
            (
                motion.left_encoder_delta_degrees,
                motion.right_encoder_delta_degrees,
            ),
            (-188, 199),
        )
        settling = raw["outcome"]["slices"][3]["segments"][0]
        self.assertEqual(
            [
                (
                    motor["position_before"],
                    motor["position_after"],
                )
                for motor in settling["motors"]
            ],
            [(-298, -299), (233, 233)],
        )
        self.assertEqual(pose.heading_mdeg, 94_815)

    def test_settling_and_failed_command_keep_distinct_evidence(self):
        results = turn_results("turn_left")
        third_after = results[2]["observation"]["motor_angles_deg"]
        results[3] = command_result(
            "turn_left",
            (
                third_after["left_drive"] - 1,
                third_after["right_drive"],
            ),
            (0, 45),
        )

        raw = build(TURN_LEFT_90, results)
        settling, commanded = raw["outcome"]["slices"][3]["segments"]

        self.assertEqual(
            settling["encoder_verification"],
            {
                "passed": False,
                "error": "uncommanded encoder settling",
                "checks": [
                    {"side": "left", "passed": True},
                    {"side": "right", "passed": True},
                ],
            },
        )
        self.assertEqual(
            commanded["encoder_verification"]["error"],
            "encoder direction missing",
        )
        self.assertEqual(raw["outcome"]["status"], "verification_failed")

    def test_actual_unequal_travel_is_not_replaced_by_nominal(self):
        results = [command_result("drive_forward", (0, 0), (71, 83))]
        motion = verified_motion_from_result(ADVANCE, build(ADVANCE, results))

        self.assertTrue(motion.complete)
        self.assertEqual(motion.left_encoder_delta_degrees, 71)
        self.assertEqual(motion.right_encoder_delta_degrees, 83)

    def test_failed_slice_keeps_all_observed_encoder_evidence(self):
        results = turn_results("turn_left")
        second_before = results[1]["receipt"]["before_angles_deg"]
        results[1] = command_result(
            "turn_left",
            (second_before["left_drive"], second_before["right_drive"]),
            (0, 45),
        )
        after = results[1]["observation"]["motor_angles_deg"]
        for index in range(2, 4):
            results[index] = command_result(
                "turn_left",
                (after["left_drive"], after["right_drive"]),
                (-45, 45),
            )
            after = results[index]["observation"]["motor_angles_deg"]

        motion = verified_motion_from_result(
            TURN_LEFT_90,
            build(TURN_LEFT_90, results),
        )

        self.assertEqual(motion.status, "verification_failed")
        self.assertEqual(motion.verified_slice_count, 3)
        self.assertEqual(motion.observed_slice_count, 4)
        self.assertEqual(motion.left_encoder_delta_degrees, -135)
        self.assertEqual(motion.right_encoder_delta_degrees, 180)

    def test_verified_turn_prefix_is_interrupted_not_completed(self):
        results = turn_results("turn_left", count=2)
        motion = verified_motion_from_result(
            TURN_LEFT_90,
            build(TURN_LEFT_90, results),
        )

        self.assertEqual(motion.status, "interrupted")
        self.assertEqual(motion.verified_slice_count, 2)
        self.assertEqual(motion.requested_slice_count, 4)
        self.assertEqual(motion.observed_slice_count, 2)

    def test_unsettled_sensor_quality_keeps_idle_encoder_evidence(self):
        results = [
            command_result(
                "drive_forward",
                (0, 0),
                (90, 90),
                settled=False,
            )
        ]
        motion = verified_motion_from_result(ADVANCE, build(ADVANCE, results))

        self.assertTrue(motion.complete)

    def test_start_anchor_and_internal_turn_continuity_are_required(self):
        drive = [command_result("drive_forward", (0, 0), (90, 90))]
        with self.assertRaises(PhysicalNavigationContractError) as caught:
            build_blast_navigation_motion_result(
                ADVANCE,
                drive,
                expected_start_angles={"left_drive": -1, "right_drive": 0},
                canonical_observation=canonical_observation(drive),
            )
        self.assertEqual(caught.exception.code, "blast_motion_slice_discontinuous")

        turns = turn_results("turn_left")
        turns[1]["receipt"]["before_angles_deg"]["left_drive"] -= 2
        with self.assertRaises(PhysicalNavigationContractError) as caught:
            build(TURN_LEFT_90, turns)
        self.assertEqual(caught.exception.code, "blast_motion_slice_discontinuous")

        turns = turn_results("turn_left")
        turns[1]["receipt"]["before_angles_deg"]["left_drive"] += 1
        with self.assertRaises(PhysicalNavigationContractError) as caught:
            build(TURN_LEFT_90, turns)
        self.assertEqual(caught.exception.code, "blast_motion_slice_discontinuous")

    def test_wrong_direction_is_retained_and_odometry_fails_closed(self):
        cases = (
            (ADVANCE, "drive_forward", (-90, 90)),
            (REVERSE, "drive_reverse", (90, -90)),
            (TURN_LEFT_90, "turn_left", (45, 45)),
            (TURN_RIGHT_90, "turn_right", (-45, -45)),
        )
        for action, command, deltas in cases:
            with self.subTest(action=action):
                results = [command_result(command, (0, 0), deltas)]
                raw = build(action, results)
                motion = verified_motion_from_result(action, raw)
                self.assertEqual(motion.status, "verification_failed")
                with self.assertRaises(PhysicalNavigationContractError) as caught:
                    apply_verified_motion(
                        PhysicalPose(),
                        motion,
                        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
                    )
                self.assertEqual(caught.exception.code, "encoder_direction_mismatch")

    def test_untrusted_contract_profile_and_encoder_data_fail_closed(self):
        base = command_result("drive_forward", (0, 0), (90, 90))
        changes = (
            ("blast_motion_result_contract_mismatch", "robot_id", "other"),
            ("blast_motion_result_contract_mismatch", "accepted", 1),
            ("blast_motion_receipt_profile_mismatch", "receipt.speed_dps", 121),
            ("blast_motion_encoder_evidence_invalid", "observation.motion_active", True),
            ("blast_motion_encoder_evidence_invalid", "observation.motor_angles_deg.left_drive", False),
            ("blast_motion_encoder_evidence_invalid", "observation.motor_angles_deg.left_drive", 1_000_000),
        )
        for expected_code, path, value in changes:
            with self.subTest(path=path, value=value):
                changed = copy.deepcopy(base)
                target = changed
                parts = path.split(".")
                for part in parts[:-1]:
                    target = target[part]
                target[parts[-1]] = value
                with self.assertRaises(PhysicalNavigationContractError) as caught:
                    build(ADVANCE, [changed])
                self.assertEqual(caught.exception.code, expected_code)

    def test_final_canonical_observation_must_match_receipt(self):
        results = [command_result("drive_forward", (0, 0), (90, 90))]
        observation = canonical_observation(results)
        observation["motors"][0]["position"] = 89
        with self.assertRaises(PhysicalNavigationContractError) as caught:
            build_blast_navigation_motion_result(
                ADVANCE,
                results,
                expected_start_angles=start_angles(results),
                canonical_observation=observation,
            )
        self.assertEqual(caught.exception.code, "blast_canonical_observation_invalid")

    def test_invalid_action_and_result_count_are_rejected(self):
        with self.assertRaises(PhysicalNavigationContractError) as caught:
            build_blast_navigation_motion_result(
                "DANCE",
                [],
                expected_start_angles={},
                canonical_observation={},
            )
        self.assertEqual(caught.exception.code, "invalid_blast_motion_action")

        with self.assertRaises(PhysicalNavigationContractError) as caught:
            build_blast_navigation_motion_result(
                TURN_LEFT_90,
                [],
                expected_start_angles={},
                canonical_observation={},
            )
        self.assertEqual(caught.exception.code, "blast_motion_result_count_mismatch")


if __name__ == "__main__":
    unittest.main()
