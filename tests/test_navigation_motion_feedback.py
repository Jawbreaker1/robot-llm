import unittest
from robot_agent.physical_odometry import PhysicalPose
from robot_agent.waypoint_geometry import measured_motion_feedback, latest_motion_feedback


class MotionFeedbackTests(unittest.TestCase):
    def test_shortened_alignment_is_complete_when_requested_heading_is_reached(self):
        feedback = measured_motion_feedback("TURN_LEFT_90", PhysicalPose(),
            PhysicalPose(heading_mdeg=66_200), requested_turn_deg=68.1, completed=False)
        self.assertTrue(feedback["completed"])
        self.assertTrue(feedback["made_progress"])

    def test_nominal_pulse_completion_is_not_alignment_completion(self):
        feedback = measured_motion_feedback("TURN_LEFT_90", PhysicalPose(),
            PhysicalPose(heading_mdeg=88_200), requested_turn_deg=120, completed=True)
        self.assertFalse(feedback["completed"])

    def test_mechanical_failure_is_not_erased_by_heading_tolerance(self):
        feedback = measured_motion_feedback("TURN_LEFT_90", PhysicalPose(),
            PhysicalPose(heading_mdeg=80_000), requested_turn_deg=90, completed=False,
            reason="encoder_target_not_reached")
        self.assertFalse(feedback["completed"])

    def test_completed_scan_at_same_pose_is_not_a_stall(self):
        pose = PhysicalPose()
        feedback = measured_motion_feedback("SCAN_FRONT_ARC", pose, pose,
            completed=True, scan_coverage_deg=120)
        self.assertEqual(feedback["kind"], "scan")
        self.assertEqual(feedback["measured_turn_deg"], 0)
        self.assertTrue(feedback["made_progress"])
        self.assertEqual(latest_motion_feedback([{"last_motion": feedback}])["consecutive_no_progress_attempts"], 0)

    def test_scan_replaces_old_drive_result_even_when_no_turn_was_possible(self):
        before, after = PhysicalPose(), PhysicalPose(45, 0, 0)
        drive = measured_motion_feedback("ADVANCE", before, after, completed=True)
        scan = measured_motion_feedback("SCAN_FRONT_ARC", after, after,
            completed=False, reason="coverage_incomplete", scan_coverage_deg=0)
        feedback = latest_motion_feedback([{"last_motion": drive}, {"last_motion": scan}])
        self.assertEqual(feedback["action"], "SCAN_FRONT_ARC")
        self.assertFalse(feedback["completed"])
        self.assertFalse(feedback["made_progress"])
        self.assertEqual(feedback["measured_translation_mm"], 0)

    def test_zero_motion_is_not_translation_or_a_completed_turn(self):
        pose = PhysicalPose(592, 315, -69768)
        feedback = measured_motion_feedback("TURN_RIGHT_90", pose, pose,
            requested_turn_deg=-90, completed=False)
        self.assertEqual(feedback["kind"], "turn")
        self.assertEqual(feedback["measured_translation_mm"], 0)
        self.assertFalse(feedback["made_progress"])
        history = [{"last_motion": feedback, "pose": {**pose.to_dict(), "verified_motion_count": n}}
                   for n in range(3)]
        self.assertEqual(latest_motion_feedback(history)["consecutive_no_progress_attempts"], 3)

    def test_progress_resets_count_and_wraparound_is_small_turn(self):
        before, after = PhysicalPose(0, 0, 179000), PhysicalPose(0, 0, -179000)
        feedback = measured_motion_feedback("TURN_LEFT_90", before, after,
            requested_turn_deg=2, completed=True)
        self.assertEqual(feedback["measured_turn_deg"], 2)
        self.assertTrue(feedback["made_progress"])
        self.assertEqual(latest_motion_feedback([{"last_motion": feedback}])["consecutive_no_progress_attempts"], 0)
