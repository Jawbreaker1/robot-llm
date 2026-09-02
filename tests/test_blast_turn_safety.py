import copy
import unittest

from robot_agent.blast_turn_safety import (
    blast_turn_slice_allows_continuation,
)


def turn_result(distance_mm=500):
    return {
        "completed": True,
        "observation_settled": True,
        "observation": {
            "motion_active": False,
            "distance_mm": distance_mm,
            "motor_angles_deg": {"body": 158},
            "imu": {"ready": False, "stationary": False},
            "rotation_sweep_window_verified": True,
        },
    }


class BlastTurnSafetyTests(unittest.TestCase):
    def test_missing_or_drifting_imu_does_not_veto_safe_turn_slice(self):
        for imu in (
            {"ready": False, "stationary": False},
            {"ready": True, "stationary": False, "heading_deg": 38.96},
        ):
            with self.subTest(imu=imu):
                result = turn_result()
                result["observation"]["imu"] = imu
                self.assertTrue(blast_turn_slice_allows_continuation(result))

    def test_non_imu_turn_slice_gates_remain_fail_closed(self):
        safe = turn_result()
        mutations = (
            lambda value: value.update(completed=False),
            lambda value: value["observation"].update(motion_active=True),
            lambda value: value["observation"]["motor_angles_deg"].update(
                body=120,
            ),
            lambda value: value["observation"].update(distance_mm=40),
            lambda value: value["observation"].update(distance_mm=None),
        )
        for mutate in mutations:
            candidate = copy.deepcopy(safe)
            mutate(candidate)
            self.assertFalse(blast_turn_slice_allows_continuation(candidate))

    def test_settling_window_does_not_truncate_a_clear_bounded_turn(self):
        result = turn_result()
        result["observation_settled"] = False
        result["observation"]["rotation_sweep_window_verified"] = False

        self.assertTrue(blast_turn_slice_allows_continuation(result))

    def test_no_valid_range_still_needs_explicit_bounded_evidence(self):
        result = turn_result(2_000)

        self.assertFalse(blast_turn_slice_allows_continuation(result))
        self.assertTrue(blast_turn_slice_allows_continuation(
            result,
            allow_no_valid_distance_with_bounded_evidence=True,
        ))

    def test_clear_unsettled_window_preserves_hard_range_gate(self):
        result = turn_result()
        result["observation_settled"] = False
        self.assertTrue(blast_turn_slice_allows_continuation(result))

        result["observation"]["distance_mm"] = 40
        self.assertFalse(blast_turn_slice_allows_continuation(result))

        result["observation"]["distance_mm"] = 2_000
        self.assertFalse(blast_turn_slice_allows_continuation(result))
        self.assertTrue(blast_turn_slice_allows_continuation(
            result,
            allow_no_valid_distance_with_bounded_evidence=True,
        ))


if __name__ == "__main__":
    unittest.main()
