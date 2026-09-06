"""Physical September 6 observations, not preselected navigation routes."""

import json
from pathlib import Path
import unittest
from unittest.mock import patch

from robot_agent.blast_episode_adapter import BlastEpisodeRuntimeAdapter
from robot_agent.blast_episode_map_trace import _BlastEpisodeMapTrace
from robot_agent.blast_navigation_motion_execution import BlastNavigationMotionExecutor
from robot_agent.blast_scan_planar_projection import project_blast_scan_planar_surfaces
from robot_agent.physical_navigation_contract import ADVANCE, TURN_LEFT_90
from robot_agent.physical_odometry import PhysicalPose
from test_blast_navigation_motion_execution import FakeController


FIXTURE = Path(__file__).parent / "fixtures/blast_navigation_regression_20260906.json"


class RecordedNavigationTests(unittest.TestCase):
    def setUp(self):
        self.recorded = json.loads(FIXTURE.read_text())

    def test_idle_gyro_drift_never_becomes_commanded_rotation(self):
        class DriftingGyro(FakeController):
            bias = 0

            def observation(self):
                value = super().observation()
                turn = ((self.angles["right_drive"] - 200)
                        - (self.angles["left_drive"] - 100)) / 2 * .49
                value["imu"] = {"heading_deg": self.bias - turn}
                return value

        controller = DriftingGyro()
        executor = BlastNavigationMotionExecutor(
            controller=controller, initial_observation=controller.observation(),
        )
        for pause in self.recorded["pauses"]:
            controller.bias += pause["heading_after"] - pause["heading_before"]
            executor.execute(ADVANCE)
            self.assertEqual(executor.pose.heading_mdeg, 0)
            self.assertEqual(executor.pose.y_mm, 0)
        # An arbitrary old offset must not suppress an actual, current turn.
        controller.bias += 90
        executor.execute(TURN_LEFT_90)
        self.assertAlmostEqual(executor.pose.heading_mdeg / 1000, 94.57, places=2)

    def test_recorded_front_scans_do_not_forget_the_startup_box(self):
        trace = _BlastEpisodeMapTrace(
            bridge=None, episode_id="recorded", pose=PhysicalPose(),
            observation={}, observed_at_unix_ms=1, episode_start_heading=0,
            minimum_forward_progress_mm=800,
        )
        for index, sample in enumerate(self.recorded["scans"]):
            pose = PhysicalPose(**sample["pose_before"])
            result = sample["result"]
            projection = project_blast_scan_planar_surfaces(
                scan=result["scan"], scan_pose=pose,
            )
            trace.record(
                pose=pose, observation=result["observation"], pose_observed=True,
                scan_view={"scan_pose": pose.to_dict(), "planar_projection": projection},
            )
            with self.subTest(scan=index + 1):
                points, _ = trace._coarse_navigation_observations()
                self.assertTrue(any(300 < x < 400 and -150 < y < 250 for x, y in points))
                # From the original start, the box still blocks the direct goal.
                evidence = trace.planner_local_map_evidence(PhysicalPose())
                self.assertIsNotNone(evidence.get("direct_goal_blockage"))
        count = len(trace._obstacle_points)
        for _ in range(20):
            trace._remember_obstacles(projection["points"])
        self.assertEqual(len(trace._obstacle_points), count)

    def test_recorded_no_return_allows_the_retained_route_not_forced_turns(self):
        sample = self.recorded["scans"][2]
        scan = sample["result"]["scan"]
        pose = {"x_mm": -55, "y_mm": -308, "heading_mdeg": 66}
        adapter = BlastEpisodeRuntimeAdapter.__new__(BlastEpisodeRuntimeAdapter)
        adapter.minimum_forward_clearance_mm = 120
        history = ({"action": "SCAN_FRONT_ARC", "pose": pose,
                    "result_observation": sample["result"]["observation"]},)
        view = {"scan": scan, "scan_pose": sample["pose_before"]}
        observation = {"sensors": sample["result"]["observation"]}
        available = adapter._available_actions(observation, history, view)
        self.assertIn(ADVANCE, available)
        self.assertEqual(adapter._waypoint_follow_motion_action(
            PhysicalPose(**pose), {"x_mm": 600, "y_mm": -300}, available,
        ), ADVANCE)

    def test_only_a_measured_ray_through_old_evidence_removes_it(self):
        trace = _BlastEpisodeMapTrace(
            bridge=None, episode_id="memory", pose=PhysicalPose(),
            observation={}, observed_at_unix_ms=1, episode_start_heading=0,
            minimum_forward_progress_mm=800,
        )
        point = {"nominal_echo_x_mm": 400, "nominal_echo_y_mm": 0,
                 "sensor_origin_x_mm": 0, "sensor_origin_y_mm": 0}
        trace._remember_obstacles([point])
        trace._remember_obstacles([])  # No return / no projected point.
        self.assertEqual(trace._obstacle_points, [point])
        side = {**point, "nominal_echo_y_mm": 450}
        trace._remember_obstacles([side])
        self.assertIn(point, trace._obstacle_points)
        farther = {**point, "nominal_echo_x_mm": 1000}
        trace._remember_obstacles([farther])
        self.assertNotIn(point, trace._obstacle_points)
        self.assertIn(side, trace._obstacle_points)

    def test_turn_feedback_uses_the_same_command_local_heading(self):
        callback = BlastEpisodeRuntimeAdapter._waypoint_alignment_continuation(
            desired_heading=90, direction=1, start_heading_mdeg=0,
            start_observation={"sensors": {"imu": {"heading_deg": 70}}},
            allow_no_valid_distance=True,
        )
        with patch("robot_agent.blast_episode_adapter.blast_turn_slice_allows_continuation",
                   return_value=True):
            self.assertTrue(callback({"observation": {"imu": {"heading_deg": 25}}}))
            self.assertFalse(callback({"observation": {"imu": {"heading_deg": -20}}}))

    def test_zero_encoder_progress_cannot_turn_gyro_noise_into_progress(self):
        class BlockedWithDrift(FakeController):
            yaw = 0

            def observation(self):
                return {**super().observation(), "imu": {"heading_deg": self.yaw}}

            def command(self, command, **kwargs):
                before = dict(self.angles)
                result = super().command(command, **kwargs)
                self.angles = before
                self.yaw += .2
                result["observation"] = self.observation()
                return result

        controller = BlockedWithDrift()
        executor = BlastNavigationMotionExecutor(
            controller=controller, initial_observation=controller.observation(),
        )
        result = executor.execute(ADVANCE)
        self.assertEqual(result.motion.verified_slice_count, 0)
        self.assertEqual(executor.pose, PhysicalPose())


if __name__ == "__main__":
    unittest.main()
