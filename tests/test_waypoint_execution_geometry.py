"""Expose executed body geometry, not the ideal polyline through waypoints."""
import unittest
import math

from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
from robot_agent.blast_navigation_calibration import BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
from robot_agent.physical_odometry import PhysicalPose
from robot_agent.physical_navigation_contract import TURN_LEFT_90
from robot_agent.waypoint_geometry import waypoint_motion_hint


class WaypointExecutionGeometryTests(unittest.TestCase):
    def test_reported_turn_circle_contains_the_same_body_as_collision_physics(self):
        for footprint in (EV3RSTORMProfile().hazard_calibration.robot_footprint,
                          BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.robot_footprint):
            with self.subTest(footprint=footprint):
                radius = footprint.planner_view()["turn_sweep_radius_mm"]
                for heading in range(-180_000, 180_000, 1_000):
                    corners = footprint.world_corners(PhysicalPose(heading_mdeg=heading))
                    distances = [math.hypot(x, y) for x, y in corners]
                    self.assertLessEqual(max(distances), radius)
                    self.assertGreater(max(distances), radius - 1)

    def test_recorded_ev3_corner_exposes_lost_side_clearance(self):
        footprint = EV3RSTORMProfile().hazard_calibration.robot_footprint
        target = {"x_mm": 800, "y_mm": 0}
        ideal = waypoint_motion_hint(PhysicalPose(x_mm=800, y_mm=-300, heading_mdeg=90_000),
                                     target, footprint=footprint)["straight_motion_projection"]
        actual = waypoint_motion_hint(PhysicalPose(x_mm=774, y_mm=-302, heading_mdeg=91_689),
                                      target, footprint=footprint)["straight_motion_projection"]
        self.assertEqual(ideal["swept_body_bounds_mm"]["min_x_mm"], 690)
        self.assertEqual(actual["swept_body_bounds_mm"]["min_x_mm"], 651)
        self.assertEqual(actual["closest_center_to_waypoint_mm"], {"x_mm": 765, "y_mm": -1})
        self.assertEqual(actual["waypoint_miss_mm"], 35)
        self.assertEqual(actual["travel_mm"], 301)

    def test_projection_uses_each_robots_asymmetric_body_and_margin_once(self):
        for footprint in (EV3RSTORMProfile().hazard_calibration.robot_footprint,
                          BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.robot_footprint):
            with self.subTest(footprint=footprint):
                result = waypoint_motion_hint(PhysicalPose(), {"x_mm": 500, "y_mm": 0},
                                               footprint=footprint)["straight_motion_projection"]
                margin = footprint.clearance_margin_mm
                self.assertEqual(result["swept_body_bounds_mm"], {
                    "min_x_mm": -footprint.rear_extent_mm - margin,
                    "max_x_mm": 500 + footprint.front_extent_mm + margin,
                    "min_y_mm": -footprint.right_extent_mm - margin,
                    "max_y_mm": footprint.left_extent_mm + margin,
                })
                self.assertEqual(result["waypoint_miss_mm"], 0)

    def test_turn_hint_does_not_claim_translation_before_alignment(self):
        footprint = EV3RSTORMProfile().hazard_calibration.robot_footprint
        result = waypoint_motion_hint(PhysicalPose(), {"x_mm": 0, "y_mm": 500},
                                      footprint=footprint)
        self.assertEqual(result, {"action": TURN_LEFT_90, "target_heading_deg": 90,
                                  "remaining_turn_deg": 90})
        self.assertIsNone(waypoint_motion_hint(PhysicalPose(), None, footprint=footprint))
