import unittest

from robot_agent.blast_navigation_policy import (
    settled_no_return_at_pose,
)
from robot_agent.physical_odometry import PhysicalPose


def settled_receipt(pose, *, distance_mm=2_000):
    return {
        "observation_settled": True,
        "pose": pose.to_dict(),
        "result_observation": {
            "distance_mm": distance_mm,
            "motion_active": False,
        },
    }


def settled_motion_receipt(pose, *, command_completed=True):
    return {
        **settled_receipt(pose),
        "motion": {"command_completed": command_completed},
    }


class BlastNavigationPolicyTests(unittest.TestCase):
    def test_exact_no_return_requires_settled_same_pose_receipt(self):
        pose = PhysicalPose(
            x_mm=45,
            y_mm=360,
            heading_mdeg=0,
            verified_motion_count=9,
            total_forward_mm=360,
            total_turn_mdeg=190_000,
        )
        receipt = settled_receipt(pose)

        self.assertTrue(settled_no_return_at_pose(2_000, pose, receipt))
        self.assertTrue(settled_no_return_at_pose(
            2_000, pose, settled_motion_receipt(pose),
        ))

    def test_close_measured_or_invalid_current_range_is_not_no_return(self):
        pose = PhysicalPose(x_mm=45, y_mm=360)
        receipt = settled_receipt(pose)

        for distance_mm in (40, 120, 500, None, -1, 2_001):
            with self.subTest(distance_mm=distance_mm):
                self.assertFalse(settled_no_return_at_pose(
                    distance_mm, pose, receipt,
                ))

    def test_no_return_rejects_mismatch_or_weak_receipt(self):
        pose = PhysicalPose(x_mm=45, y_mm=360)
        valid = settled_receipt(pose)
        cases = {
            "missing_receipt": None,
            "unsettled": {
                **valid,
                "observation_settled": False,
            },
            "different_pose": settled_receipt(
                PhysicalPose(x_mm=46, y_mm=360)
            ),
            "receipt_not_no_return": settled_receipt(
                pose, distance_mm=500,
            ),
            "motion_active": {
                **valid,
                "result_observation": {
                    **valid["result_observation"],
                    "motion_active": True,
                },
            },
            "motion_incomplete": settled_motion_receipt(
                pose, command_completed=False,
            ),
        }

        for name, receipt in cases.items():
            with self.subTest(case=name):
                self.assertFalse(settled_no_return_at_pose(
                    2_000, pose, receipt,
                ))


if __name__ == "__main__":
    unittest.main()
