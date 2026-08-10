import unittest

from robot_agent.blast_side_observation import (
    build_blast_multi_view_observation,
)
from robot_agent.physical_odometry import PhysicalPose


class BlastSideObservationTests(unittest.TestCase):
    def test_payload_is_detached_and_never_claims_navigation_proof(self):
        origin = {"scan_pose": PhysicalPose().to_dict(), "scan": {}}
        pose = PhysicalPose(y_mm=100)
        side = {"scan_pose": pose.to_dict(), "scan": {}}
        waypoint = {"target_x_mm": 0, "target_y_mm": 100}

        result = build_blast_multi_view_observation(
            origin_view=origin,
            side_view=side,
            selected_side="LEFT",
            waypoint=waypoint,
            pose=pose,
            diagnostic_scan={"state": "complete"},
            host_actions=["ADVANCE", "SCAN_FRONT_ARC"],
        )

        evidence = result["multi_view_observations"]
        self.assertEqual(evidence["viewpoint_separation_mm"], 100)
        for fact in (
            "object_association_proven",
            "clearance_proven",
            "passage_proven",
            "route_eligible",
        ):
            self.assertFalse(evidence[fact])
        origin["scan_pose"]["x_mm"] = 999
        self.assertEqual(evidence["views"][0]["scan_pose"]["x_mm"], 0)

    def test_refuses_a_view_that_did_not_reach_its_waypoint(self):
        origin = {"scan_pose": PhysicalPose().to_dict()}
        pose = PhysicalPose(y_mm=40)

        with self.assertRaises(ValueError):
            build_blast_multi_view_observation(
                origin_view=origin,
                side_view={"scan_pose": pose.to_dict()},
                selected_side="LEFT",
                waypoint={"target_x_mm": 0, "target_y_mm": 100},
                pose=pose,
                diagnostic_scan={"state": "complete"},
                host_actions=(),
            )


if __name__ == "__main__":
    unittest.main()
