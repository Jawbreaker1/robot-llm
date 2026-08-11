import unittest

from robot_agent.blast_navigation_state import (
    LocalDetourNavigationState,
    PlannerNavigationState,
    SideSearchNavigationState,
)
from robot_agent.local_detour_route import build_local_detour_route
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_navigation_contract import ADVANCE, TURN_RIGHT_90
from robot_agent.physical_odometry import PhysicalPose


def route():
    return build_local_detour_route(
        current_pose=PhysicalPose(),
        goal_heading_mdeg=0,
        detour_side="LEFT_OF_GOAL",
        target_hypothesis_id="hazard-a",
        target_centroid_x_mm=200,
        target_centroid_y_mm=0,
        target_radius_mm=50,
        footprint=RobotFootprint(
            front_extent_mm=70,
            rear_extent_mm=60,
            left_extent_mm=80,
            right_extent_mm=100,
            clearance_margin_mm=20,
            calibration_status="test",
            calibration_evidence="test fixture",
        ),
        frame_id="frame-a",
        map_generation_id="map-a",
        map_version=7,
        goal_origin_x_mm=0,
        goal_origin_y_mm=0,
    )


class BlastNavigationStateTests(unittest.TestCase):
    def setUp(self):
        self.waypoint = {
            "target_x_mm": 0,
            "target_y_mm": 360,
            "target_heading_mdeg": 0,
        }
        self.origin_view = {
            "scan_pose": PhysicalPose().to_dict(),
            "scan": {"schema": "test-scan"},
        }

    def test_planner_begins_side_search(self):
        state = PlannerNavigationState().begin_side_search(
            selected_side="LEFT",
            waypoint=self.waypoint,
            origin_scan_view=self.origin_view,
        )

        self.assertIsInstance(state, SideSearchNavigationState)
        self.assertEqual(state.waypoint["target_y_mm"], 360)
        self.assertEqual(state.origin_scan_view["scan"]["schema"], "test-scan")
        self.assertEqual(state.host_actions, ())

    def test_side_search_transitions_are_immutable(self):
        initial = PlannerNavigationState().begin_side_search(
            selected_side="LEFT",
            waypoint=self.waypoint,
            origin_scan_view=self.origin_view,
        )
        moved = initial.record_host_action(
            ADVANCE, outbound_distance_mm=180,
        )
        restored = moved.record_host_action(
            TURN_RIGHT_90,
        ).mark_reorientation_attempted()

        self.assertEqual(initial.host_actions, ())
        self.assertIsNone(initial.previous_outbound_distance_mm)
        self.assertEqual(restored.host_actions, (ADVANCE, TURN_RIGHT_90))
        self.assertEqual(restored.previous_outbound_distance_mm, 180)
        self.assertTrue(restored.reorientation_attempted)

    def test_reacquisition_resets_only_waypoint_progress(self):
        state = PlannerNavigationState().begin_side_search(
            selected_side="RIGHT",
            waypoint=self.waypoint,
            origin_scan_view=self.origin_view,
        ).record_host_action(ADVANCE, outbound_distance_mm=90)
        state = state.mark_reorientation_attempted()

        continued = state.continue_to_waypoint({
            **self.waypoint,
            "target_y_mm": -180,
        })

        self.assertEqual(continued.selected_side, "RIGHT")
        self.assertEqual(continued.waypoint["target_y_mm"], -180)
        self.assertFalse(continued.reorientation_attempted)
        self.assertIsNone(continued.previous_outbound_distance_mm)
        self.assertEqual(continued.host_actions, (ADVANCE,))
        self.assertEqual(continued.origin_scan_view, state.origin_scan_view)

    def test_route_binding_and_pass_verification_are_explicit(self):
        side = PlannerNavigationState().begin_side_search(
            selected_side="LEFT",
            waypoint=self.waypoint,
            origin_scan_view=self.origin_view,
        )
        bound = side.bind_local_detour(route())
        verified = bound.mark_pass_scan_complete()

        self.assertIsInstance(bound, LocalDetourNavigationState)
        self.assertFalse(bound.pass_scan_complete)
        self.assertTrue(verified.pass_scan_complete)
        self.assertEqual(verified.selected_side, "LEFT")

if __name__ == "__main__":
    unittest.main()
