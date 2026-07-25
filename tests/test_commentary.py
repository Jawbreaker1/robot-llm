import json
import unittest
from pathlib import Path

from robot_agent import (
    DEFAULT_OBSTACLE_GATE_POLICY,
    DEFAULT_PROXIMITY_THRESHOLDS,
    ObstacleEvidenceGate,
    ObstacleGatePolicy,
    ProximityThresholds,
    StableZoneTracker,
    classify_infrared,
    fallback_comment,
    validate_generated_comment,
)


PROJECT_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "ev3rstorm.json"


class InfraredCommentaryTests(unittest.TestCase):
    def test_median_sample_is_classified_as_relative_zone(self):
        observation = classify_infrared([64, 66, 67, 68, 99], 12_345)

        self.assertEqual(observation.filtered_percent, 67)
        self.assertEqual(observation.zone, "far_or_no_clear_return")
        self.assertEqual(observation.observed_at_ms, 12_345)
        self.assertEqual(
            fallback_comment(observation),
            "Jag får ingen tydlig närträff framför mig.",
        )

    def test_all_provisional_zone_boundaries(self):
        expected = {
            0: "strong_return",
            16: "strong_return",
            17: "near_return",
            35: "near_return",
            36: "mid_return",
            47: "mid_return",
            48: "far_or_no_clear_return",
            100: "far_or_no_clear_return",
        }
        for value, zone in expected.items():
            with self.subTest(value=value):
                self.assertEqual(
                    classify_infrared([value], 0).zone,
                    zone,
                )

    def test_invalid_ir_samples_are_rejected(self):
        invalid_sets = [[], [True], [1.5], [-1], [101]]
        for samples in invalid_sets:
            with self.subTest(samples=samples):
                with self.assertRaises(ValueError):
                    classify_infrared(samples, 0)

    def test_defaults_match_checked_in_physical_calibration(self):
        with CONFIG_PATH.open("r", encoding="utf-8") as handle:
            config = json.load(handle)

        self.assertEqual(
            ProximityThresholds.from_config(config),
            DEFAULT_PROXIMITY_THRESHOLDS,
        )
        self.assertEqual(
            ObstacleGatePolicy.from_config(config),
            DEFAULT_OBSTACLE_GATE_POLICY,
        )

    def test_generated_comment_is_bounded_and_one_line(self):
        self.assertEqual(
            validate_generated_comment("  Något   är nära.  "),
            "Något är nära.",
        )
        for text in [
            "",
            "rad ett\nrad två",
            "x" * 81,
            "ett två tre fyra fem sex sju åtta nio tio elva tolv "
            "tretton fjorton femton",
        ]:
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    validate_generated_comment(text)

    def test_tracker_emits_only_stable_transitions(self):
        tracker = StableZoneTracker(required_consecutive=3)

        self.assertIsNone(tracker.observe("mid_return"))
        self.assertIsNone(tracker.observe("near_return"))
        self.assertIsNone(tracker.observe("near_return"))
        self.assertEqual(tracker.observe("near_return"), "near_return")
        self.assertIsNone(tracker.observe("near_return"))
        self.assertIsNone(tracker.observe("mid_return"))
        self.assertIsNone(tracker.observe("near_return"))
        self.assertIsNone(tracker.observe("mid_return"))
        self.assertIsNone(tracker.observe("mid_return"))
        self.assertEqual(tracker.observe("mid_return"), "mid_return")

    def test_tracker_rejects_unknown_zone(self):
        with self.assertRaises(ValueError):
            StableZoneTracker().observe("dog")

    def test_obstacle_gate_starts_unknown_then_accepts_clear_evidence(self):
        gate = ObstacleEvidenceGate()

        for value in [52, 52, 52, 52]:
            self.assertIsNone(gate.observe(value))
            self.assertFalse(gate.motion_allowed)
            self.assertTrue(gate.stop_required)

        self.assertFalse(gate.observe(52))
        self.assertTrue(gate.motion_allowed)
        self.assertFalse(gate.stop_required)

    def test_obstacle_gate_replays_narrow_box_sweep(self):
        gate = ObstacleEvidenceGate()
        for value in [52, 52, 52, 52, 52]:
            gate.observe(value)
        self.assertTrue(gate.motion_allowed)

        states = [
            gate.observe(value)
            for value in [47, 42, 38, 34, 31, 29]
        ]
        self.assertIn(True, states)
        self.assertTrue(gate.state)
        self.assertTrue(gate.stop_required)
        self.assertLessEqual(gate.filtered_percent, 35)

    def test_obstacle_gate_requires_four_28_samples_from_startup(self):
        gate = ObstacleEvidenceGate()
        self.assertIsNone(gate.observe(28))
        self.assertIsNone(gate.observe(28))
        self.assertIsNone(gate.observe(28))
        self.assertTrue(gate.observe(28))

    def test_obstacle_gate_enters_immediately_on_strong_return(self):
        gate = ObstacleEvidenceGate()
        self.assertTrue(gate.observe(13))
        self.assertTrue(gate.state)
        self.assertTrue(gate.stop_required)

    def test_obstacle_gate_clears_slowly_and_deadband_keeps_state(self):
        gate = ObstacleEvidenceGate()
        gate.observe(13)
        self.assertTrue(gate.state)

        for value in [36, 37, 39]:
            self.assertTrue(gate.observe(value))
        for value in [43, 47, 51]:
            gate.observe(value)
        self.assertTrue(gate.state)
        for value in [52, 52]:
            gate.observe(value)
        self.assertFalse(gate.state)
        self.assertTrue(gate.motion_allowed)

    def test_obstacle_gate_rejects_bad_values_and_policy_bools(self):
        gate = ObstacleEvidenceGate()
        for value in [True, -1, 101]:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    gate.observe(value)

        invalid_policies = [
            dict(
                immediate_enter_max=True,
                enter_max=35,
                exit_min=40,
                median_window=3,
                enter_consecutive=2,
                exit_consecutive=3,
            ),
            dict(
                immediate_enter_max=16,
                enter_max=35,
                exit_min=40,
                median_window=True,
                enter_consecutive=2,
                exit_consecutive=3,
            ),
        ]
        for values in invalid_policies:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    ObstacleGatePolicy(**values)


if __name__ == "__main__":
    unittest.main()
