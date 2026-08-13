import unittest

from robot_agent.blast_navigation_action_profile import (
    BLAST_NAVIGATION_COMMANDS,
    DRIVE_ENCODER_DEGREES,
    SCAN_TURN_ENCODER_DEGREES_PER_PULSE,
    SCAN_TURN_PULSES_PER_SIDE,
    TURN_ENCODER_DEGREES_PER_PULSE,
    TURN_ENCODER_DEGREES_PER_QUARTER_TURN,
    TURN_EXPECTED_ACTUAL_OPPOSED_ENCODER_DEGREES_PER_QUARTER_TURN,
    TURN_PULSES_PER_QUARTER_TURN,
    blast_navigation_action_specs,
    blast_scan_turn_maximum_pose,
)
from robot_agent.blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    MOTION_ACTIONS,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from robot_agent.physical_odometry import PhysicalPose, nominal_effect


class BlastNavigationActionProfileTests(unittest.TestCase):
    def test_every_semantic_motion_has_one_fixed_command_sequence(self):
        self.assertEqual(set(BLAST_NAVIGATION_COMMANDS), MOTION_ACTIONS)
        self.assertEqual(BLAST_NAVIGATION_COMMANDS[ADVANCE], ("drive_forward",))
        self.assertEqual(BLAST_NAVIGATION_COMMANDS[REVERSE], ("drive_reverse",))
        self.assertEqual(
            BLAST_NAVIGATION_COMMANDS[TURN_LEFT_90],
            ("turn_left",) * TURN_PULSES_PER_QUARTER_TURN,
        )
        self.assertEqual(
            BLAST_NAVIGATION_COMMANDS[TURN_RIGHT_90],
            ("turn_right",) * TURN_PULSES_PER_QUARTER_TURN,
        )

    def test_specs_match_the_bounded_hub_pulses(self):
        specs = blast_navigation_action_specs()

        self.assertEqual(set(specs), MOTION_ACTIONS)
        self.assertEqual(
            specs[ADVANCE]["target_mean_abs_encoder_degrees"],
            DRIVE_ENCODER_DEGREES,
        )
        self.assertEqual(
            specs[TURN_LEFT_90]["target_mean_abs_encoder_degrees"],
            TURN_EXPECTED_ACTUAL_OPPOSED_ENCODER_DEGREES_PER_QUARTER_TURN,
        )
        self.assertEqual(
            TURN_ENCODER_DEGREES_PER_QUARTER_TURN,
            TURN_ENCODER_DEGREES_PER_PULSE
            * TURN_PULSES_PER_QUARTER_TURN,
        )
        self.assertEqual(
            TURN_EXPECTED_ACTUAL_OPPOSED_ENCODER_DEGREES_PER_QUARTER_TURN,
            193,
        )
        self.assertEqual(
            specs[TURN_LEFT_90]["estimated_body_turn_degrees"],
            94.57,
        )
        self.assertEqual(
            specs[TURN_RIGHT_90]["estimated_body_turn_degrees"],
            -94.57,
        )
        for action, commands in BLAST_NAVIGATION_COMMANDS.items():
            with self.subTest(action=action):
                spec = specs[action]
                self.assertEqual(spec["slice_count"], len(commands))
                self.assertEqual(
                    spec["total_duration_ms"],
                    sum(spec["slice_durations_ms"]),
                )

    def test_calibrated_nominal_effects_have_physical_meaning(self):
        specs = blast_navigation_action_specs()
        calibration = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
        origin = PhysicalPose()

        advance, _ = nominal_effect(origin, ADVANCE, specs, calibration)
        reverse, _ = nominal_effect(origin, REVERSE, specs, calibration)
        left, _ = nominal_effect(origin, TURN_LEFT_90, specs, calibration)
        right, _ = nominal_effect(origin, TURN_RIGHT_90, specs, calibration)

        self.assertEqual((advance.x_mm, advance.y_mm), (45, 0))
        self.assertEqual((reverse.x_mm, reverse.y_mm), (-45, 0))
        self.assertEqual(left.heading_mdeg, 94_570)
        self.assertEqual(right.heading_mdeg, -94_570)

    def test_scan_sweep_uses_four_calibrated_pulses_per_side(self):
        origin = PhysicalPose()

        left = blast_scan_turn_maximum_pose(origin, TURN_LEFT_90)
        right = blast_scan_turn_maximum_pose(origin, TURN_RIGHT_90)
        _nominal, full_turn_maximum = nominal_effect(
            origin,
            TURN_LEFT_90,
            blast_navigation_action_specs(),
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
        )

        self.assertEqual(SCAN_TURN_PULSES_PER_SIDE, 4)
        self.assertEqual(SCAN_TURN_ENCODER_DEGREES_PER_PULSE, 45)
        self.assertEqual(
            TURN_EXPECTED_ACTUAL_OPPOSED_ENCODER_DEGREES_PER_QUARTER_TURN
            * BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
            .turn_mdeg_per_opposed_encoder_degree,
            94_570,
        )
        self.assertEqual(left.heading_mdeg, full_turn_maximum.heading_mdeg)
        self.assertEqual(right.heading_mdeg, -full_turn_maximum.heading_mdeg)
        with self.assertRaises(ValueError):
            blast_scan_turn_maximum_pose(origin, ADVANCE)

    def test_callers_cannot_mutate_the_checked_in_profile(self):
        changed = blast_navigation_action_specs()
        changed[ADVANCE]["slice_durations_ms"].append(1)
        changed[ADVANCE]["calibration_evidence"][
            "observed_forward_progress_mm"
        ] = 999

        fresh = blast_navigation_action_specs()

        self.assertEqual(fresh[ADVANCE]["slice_durations_ms"], [750])
        self.assertEqual(
            changed[REVERSE]["calibration_evidence"][
                "observed_forward_progress_mm"
            ],
            45,
        )
        self.assertEqual(
            fresh[ADVANCE]["calibration_evidence"][
                "observed_forward_progress_mm"
            ],
            45,
        )


if __name__ == "__main__":
    unittest.main()
