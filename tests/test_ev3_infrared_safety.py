import json
import unittest

from ev3.infrared_safety import (
    REASON_IMMEDIATE_ENTRY,
    REASON_INVALID_SAMPLE,
    REASON_STABLE_ENTRY,
    REASON_STABLE_EXIT,
    REASON_UNVERIFIED_STARTUP,
    InfraredGatePolicy,
    InfraredObstacleGate,
)
from ev3.robot_hal import SafetyError


def default_policy():
    return InfraredGatePolicy(
        immediate_enter_max=16,
        enter_max=35,
        exit_min=40,
        median_window=3,
        enter_consecutive=2,
        exit_consecutive=3,
    )


def released_gate():
    gate = InfraredObstacleGate(default_policy())
    for value in [52, 52, 52, 52, 52]:
        result = gate.observe(value)
    if result["blocked"]:
        raise AssertionError("test gate did not release")
    return gate


class InfraredGatePolicyTests(unittest.TestCase):
    def test_policy_round_trips_through_plain_dict(self):
        policy = default_policy()

        self.assertEqual(
            policy.to_dict(),
            {
                "immediate_enter_max": 16,
                "enter_max": 35,
                "exit_min": 40,
                "median_window": 3,
                "enter_consecutive": 2,
                "exit_consecutive": 3,
            },
        )
        self.assertEqual(
            InfraredGatePolicy.from_config(
                {
                    "calibration": {
                        "infrared_proximity": {
                            "obstacle_gate": policy.to_dict(),
                        },
                    },
                }
            ).to_dict(),
            policy.to_dict(),
        )

    def test_rejects_bools_invalid_thresholds_and_unbounded_counts(self):
        valid = default_policy().to_dict()
        mutations = [
            ("immediate_enter_max", True),
            ("enter_max", 101),
            ("exit_min", 35),
            ("median_window", 0),
            ("median_window", 4),
            ("median_window", 33),
            ("enter_consecutive", 0),
            ("enter_consecutive", 21),
            ("exit_consecutive", 21),
        ]
        for name, value in mutations:
            with self.subTest(name=name, value=value):
                values = dict(valid)
                values[name] = value
                with self.assertRaises(SafetyError):
                    InfraredGatePolicy(**values)

    def test_missing_config_is_rejected(self):
        for config in [None, {}, {"calibration": None}]:
            with self.subTest(config=config):
                with self.assertRaises(SafetyError):
                    InfraredGatePolicy.from_config(config)


class InfraredObstacleGateTests(unittest.TestCase):
    def test_starts_blocked_and_exposes_only_bounded_state(self):
        gate = InfraredObstacleGate(default_policy())

        self.assertEqual(
            gate.snapshot(),
            {
                "raw": None,
                "filtered": None,
                "blocked": True,
                "reason": REASON_UNVERIFIED_STARTUP,
                "sample_count": 0,
            },
        )
        self.assertEqual(
            set(gate.snapshot()),
            {"raw", "filtered", "blocked", "reason", "sample_count"},
        )
        json.dumps(gate.snapshot(), sort_keys=True)

    def test_stable_clear_evidence_releases_after_full_window(self):
        gate = InfraredObstacleGate(default_policy())

        states = [gate.observe(52) for _ in range(5)]

        self.assertTrue(all(state["blocked"] for state in states[:4]))
        self.assertFalse(states[4]["blocked"])
        self.assertEqual(states[4]["filtered"], 52)
        self.assertEqual(states[4]["reason"], REASON_STABLE_EXIT)
        self.assertEqual(states[4]["sample_count"], 5)

    def test_strong_raw_return_blocks_immediately(self):
        gate = released_gate()

        result = gate.observe(16)

        self.assertTrue(result["blocked"])
        self.assertEqual(result["raw"], 16)
        self.assertEqual(result["reason"], REASON_IMMEDIATE_ENTRY)

    def test_median_entry_requires_consecutive_stable_decisions(self):
        gate = released_gate()

        first = gate.observe(35)
        second = gate.observe(35)
        third = gate.observe(35)

        self.assertFalse(first["blocked"])
        self.assertFalse(second["blocked"])
        self.assertTrue(third["blocked"])
        self.assertEqual(third["filtered"], 35)
        self.assertEqual(third["reason"], REASON_STABLE_ENTRY)

    def test_hysteresis_holds_block_until_stable_release(self):
        gate = InfraredObstacleGate(default_policy())
        gate.observe(13)

        for value in [36, 37, 39, 40, 40, 40]:
            result = gate.observe(value)
            self.assertTrue(result["blocked"])

        result = gate.observe(40)
        self.assertFalse(result["blocked"])
        self.assertEqual(result["filtered"], 40)
        self.assertEqual(result["reason"], REASON_STABLE_EXIT)

    def test_malformed_sample_latches_closed_and_discards_old_window(self):
        for value in [True, -1, 101, 1.5, "25", None]:
            with self.subTest(value=value):
                gate = released_gate()
                accepted_count = gate.snapshot()["sample_count"]

                with self.assertRaises(SafetyError):
                    gate.observe(value)

                self.assertEqual(
                    gate.snapshot(),
                    {
                        "raw": None,
                        "filtered": None,
                        "blocked": True,
                        "reason": REASON_INVALID_SAMPLE,
                        "sample_count": accepted_count,
                    },
                )
                for _ in range(4):
                    self.assertTrue(gate.observe(52)["blocked"])
                self.assertFalse(gate.observe(52)["blocked"])

    def test_rejects_non_policy_dependency(self):
        for policy in [None, {}, True]:
            with self.subTest(policy=policy):
                with self.assertRaises(SafetyError):
                    InfraredObstacleGate(policy)


if __name__ == "__main__":
    unittest.main()
