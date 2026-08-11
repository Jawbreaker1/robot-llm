from dataclasses import replace
import unittest

from robot_agent.blast_detour_runtime import (
    BlastDetourRuntimeBlocked,
    blast_local_detour_step,
)
from robot_agent.blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from robot_agent.local_detour_route import build_local_detour_route
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from robot_agent.physical_navigation_mission import DirectionalMission
from robot_agent.physical_odometry import PhysicalPose


class BlastDetourRuntimeTests(unittest.TestCase):
    def test_live_pass_drift_uses_route_tolerance(self):
        mission = DirectionalMission.begin(
            episode_id="episode-live-pass",
            minimum_forward_progress_mm=420,
            pose=PhysicalPose(),
        )
        footprint = (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.robot_footprint
        )
        route = build_local_detour_route(
            current_pose=PhysicalPose(x_mm=92, y_mm=0),
            goal_heading_mdeg=0,
            detour_side="LEFT_OF_GOAL",
            target_hypothesis_id="blast-provisional-two-view-target",
            target_centroid_x_mm=379,
            target_centroid_y_mm=80,
            target_radius_mm=122,
            target_support_points=((379, 80),),
            footprint=footprint,
            frame_id="EPISODE_LOCAL_ODOMETRY",
            map_generation_id=mission.episode_id,
            map_version=2,
            goal_origin_x_mm=0,
            goal_origin_y_mm=0,
        )
        route = replace(route, active_index=2)
        pose = PhysicalPose(x_mm=414, y_mm=341, heading_mdeg=-5_635)

        self.assertFalse(mission.heading_aligned(pose))
        self.assertLessEqual(
            abs(pose.heading_mdeg), route.heading_tolerance_mdeg,
        )
        updated, guidance, action, scan_role = blast_local_detour_step(
            route=route,
            pose=pose,
            distance_mm=1_070,
            available_actions=(ADVANCE, TURN_LEFT_90, TURN_RIGHT_90),
            pass_scan_complete=False,
            mission=mission,
            prior_receipt=None,
            rotation_allowed=True,
            evidence_correlated=True,
        )

        self.assertIs(updated, route)
        self.assertEqual(guidance.allowed_motion_actions, frozenset((ADVANCE,)))
        self.assertEqual(action, ADVANCE)
        self.assertIsNone(scan_role)

        for heading_mdeg in (-20_000, 20_000):
            with self.subTest(heading_mdeg=heading_mdeg):
                result = blast_local_detour_step(
                    route=route,
                    pose=replace(pose, heading_mdeg=heading_mdeg),
                    distance_mm=1_070,
                    available_actions=(ADVANCE, TURN_LEFT_90, TURN_RIGHT_90),
                    pass_scan_complete=False,
                    mission=mission,
                    prior_receipt=None,
                    rotation_allowed=True,
                    evidence_correlated=True,
                )
                self.assertEqual(result[2], ADVANCE)
        for heading_mdeg in (-20_001, 20_001):
            with self.subTest(heading_mdeg=heading_mdeg):
                with self.assertRaises(BlastDetourRuntimeBlocked):
                    blast_local_detour_step(
                        route=route,
                        pose=replace(pose, heading_mdeg=heading_mdeg),
                        distance_mm=1_070,
                        available_actions=(
                            ADVANCE, TURN_LEFT_90, TURN_RIGHT_90,
                        ),
                        pass_scan_complete=False,
                        mission=mission,
                        prior_receipt=None,
                        rotation_allowed=True,
                        evidence_correlated=True,
                    )

        merge_route = replace(route, active_index=3)
        for heading_mdeg in (-20_001, 20_001):
            with self.subTest(
                phase="pass_buffer", heading_mdeg=heading_mdeg,
            ):
                with self.assertRaises(BlastDetourRuntimeBlocked):
                    blast_local_detour_step(
                        route=merge_route,
                        pose=PhysicalPose(
                            x_mm=699,
                            y_mm=347,
                            heading_mdeg=heading_mdeg,
                        ),
                        distance_mm=1_070,
                        available_actions=(
                            ADVANCE, TURN_LEFT_90, TURN_RIGHT_90,
                        ),
                        pass_scan_complete=False,
                        mission=mission,
                        prior_receipt=None,
                        rotation_allowed=True,
                        evidence_correlated=True,
                    )


if __name__ == "__main__":
    unittest.main()
