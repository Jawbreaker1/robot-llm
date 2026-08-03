import copy
import unittest

from robot_agent.local_detour_route import (
    LATERAL_CLEARANCE,
    MERGE_GOAL_AXIS,
    PASS_BEYOND_TARGET,
    REACQUIRE_GOAL_HEADING,
    RESUME_GOAL_HEADING,
    ROUTE_COMPLETE,
    ROUTE_INVALID,
    LocalDetourRoute,
    LocalDetourRouteError,
    build_local_detour_route,
)
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_odometry import PhysicalPose


def footprint():
    return RobotFootprint(
        front_extent_mm=70,
        rear_extent_mm=60,
        left_extent_mm=80,
        right_extent_mm=100,
        clearance_margin_mm=20,
        calibration_status="test",
        calibration_evidence="test fixture",
    )


def route(*, side="LEFT_OF_GOAL", pose=PhysicalPose(), **overrides):
    values = {
        "current_pose": pose,
        "goal_heading_mdeg": 0,
        "detour_side": side,
        "target_hypothesis_id": "hazard-a",
        "target_centroid_x_mm": 200,
        "target_centroid_y_mm": 0,
        "target_radius_mm": 50,
        "footprint": footprint(),
        "frame_id": "frame-a",
        "map_generation_id": "map-a",
        "map_version": 7,
        "goal_origin_x_mm": 0,
        "goal_origin_y_mm": 0,
    }
    values.update(overrides)
    return build_local_detour_route(**values)


class LocalDetourRouteGeometryTests(unittest.TestCase):
    def test_left_route_uses_right_body_extent_and_rectilinear_waypoints(self):
        value = route()

        self.assertEqual(value.inflated_lateral_clearance_mm, 205)
        self.assertEqual(value.inflated_pass_clearance_mm, 228)
        self.assertEqual(value.route_lateral_offset_mm, 205)
        self.assertEqual(value.pass_longitudinal_offset_mm, 428)
        self.assertEqual(
            [item.kind for item in value.waypoints],
            [
                LATERAL_CLEARANCE,
                REACQUIRE_GOAL_HEADING,
                PASS_BEYOND_TARGET,
                MERGE_GOAL_AXIS,
                RESUME_GOAL_HEADING,
            ],
        )
        self.assertEqual(
            [
                (item.x_mm, item.y_mm, item.heading_mdeg)
                for item in value.waypoints
            ],
            [
                (0, 205, 90_000),
                (0, 205, 0),
                (428, 205, 0),
                (428, 0, -90_000),
                (428, 0, 0),
            ],
        )

    def test_right_route_uses_left_body_extent_without_host_side_choice(self):
        value = route(side="RIGHT_OF_GOAL")

        self.assertEqual(value.inflated_lateral_clearance_mm, 185)
        self.assertEqual(value.route_lateral_offset_mm, -185)
        self.assertEqual(value.waypoints[0].heading_mdeg, -90_000)
        self.assertEqual(value.waypoints[-2].heading_mdeg, 90_000)
        with self.assertRaises(LocalDetourRouteError):
            route(side="AUTO")

    def test_waypoint_tolerance_cannot_consume_collision_clearance(self):
        value = route()

        self.assertEqual(
            value.route_lateral_offset_mm - value.position_tolerance_mm,
            50 + 100 + 20,
        )
        self.assertEqual(
            value.pass_longitudinal_offset_mm - value.position_tolerance_mm,
            200 + 50 + 123 + 20,
        )

    def test_existing_lateral_clearance_never_routes_back_toward_hazard(self):
        value = route(pose=PhysicalPose(y_mm=230))

        self.assertEqual(value.route_lateral_offset_mm, 230)
        self.assertNotEqual(value.waypoints[0].kind, LATERAL_CLEARANCE)
        self.assertEqual(value.waypoints[0].kind, PASS_BEYOND_TARGET)

    def test_lateral_deficit_inside_tolerance_still_stages_outward(self):
        value = route(pose=PhysicalPose(y_mm=190))

        self.assertEqual(value.route_lateral_offset_mm, 205)
        self.assertEqual(value.waypoints[0].kind, LATERAL_CLEARANCE)

        mirrored = route(
            side="RIGHT_OF_GOAL",
            pose=PhysicalPose(y_mm=-170),
        )
        self.assertEqual(mirrored.route_lateral_offset_mm, -185)
        self.assertEqual(mirrored.waypoints[0].kind, LATERAL_CLEARANCE)

    def test_goal_frame_rotation_produces_same_route_shape(self):
        value = route(
            goal_heading_mdeg=90_000,
            target_centroid_x_mm=0,
            target_centroid_y_mm=200,
        )

        self.assertEqual(
            (value.waypoints[0].x_mm, value.waypoints[0].y_mm),
            (-205, 0),
        )
        self.assertEqual(value.waypoints[0].heading_mdeg, -180_000)
        self.assertEqual(
            (value.waypoints[2].x_mm, value.waypoints[2].y_mm),
            (-205, 428),
        )

    def test_scan_support_envelope_expands_route_beyond_centroid(self):
        value = route(target_support_points=((240, -40), (260, 90)))

        self.assertEqual(value.route_lateral_offset_mm, 295)
        self.assertEqual(value.pass_longitudinal_offset_mm, 488)
        self.assertEqual(
            value.target_support_points,
            ((200, 0), (240, -40), (260, 90)),
        )


class LocalDetourRouteLifecycleTests(unittest.TestCase):
    def test_route_identity_is_stable_and_json_shape_round_trips(self):
        first = route()
        second = route()

        self.assertEqual(first.route_id, second.route_id)
        self.assertEqual(first.version, 1)
        serialized = first.to_dict()
        self.assertEqual(LocalDetourRoute.from_mapping(serialized), first)
        malformed = copy.deepcopy(serialized)
        malformed["unexpected"] = True
        with self.assertRaises(LocalDetourRouteError):
            LocalDetourRoute.from_mapping(malformed)

    def test_reached_poses_advance_ordered_waypoints_and_version(self):
        value = route()

        value = value.advance_reached(
            PhysicalPose(x_mm=0, y_mm=180, heading_mdeg=90_000)
        )
        self.assertEqual(value.active_index, 0)
        self.assertEqual(value.version, 1)
        value = value.advance_reached(
            PhysicalPose(x_mm=0, y_mm=205, heading_mdeg=90_000)
        )
        self.assertEqual(value.active_index, 1)
        self.assertEqual(value.version, 2)
        value = value.advance_reached(
            PhysicalPose(x_mm=0, y_mm=204, heading_mdeg=0)
        )
        self.assertEqual(value.active_index, 2)
        value = value.advance_reached(
            PhysicalPose(x_mm=403, y_mm=180, heading_mdeg=0)
        )
        self.assertEqual(value.active_index, 3)
        value = value.advance_reached(
            PhysicalPose(x_mm=403, y_mm=-10, heading_mdeg=-90_000)
        )
        self.assertEqual(value.active_index, 4)
        value = value.advance_reached(
            PhysicalPose(x_mm=403, y_mm=-10, heading_mdeg=0)
        )
        self.assertEqual(value.status, ROUTE_COMPLETE)
        self.assertEqual(value.active_index, len(value.waypoints))
        self.assertIsNone(value.active_waypoint)

    def test_matching_newer_map_keeps_route_but_mismatches_invalidate(self):
        value = route()
        matching = value.reconcile(
            frame_id="frame-a",
            map_generation_id="map-a",
            map_version=9,
            target_hypothesis_id="hazard-a",
            target_centroid_x_mm=200,
            target_centroid_y_mm=0,
            target_radius_mm=50,
            target_support_points=((200, 0),),
        )
        self.assertIs(matching, value)
        cases = (
            ({"frame_id": "frame-b"}, "FRAME_MISMATCH"),
            (
                {"map_generation_id": "map-b"},
                "MAP_GENERATION_MISMATCH",
            ),
            ({"map_version": 6}, "MAP_VERSION_REGRESSION"),
            ({"target_hypothesis_id": None}, "TARGET_MISSING"),
            (
                {"target_hypothesis_id": "hazard-b"},
                "TARGET_ID_MISMATCH",
            ),
            (
                {"target_centroid_x_mm": 201},
                "TARGET_GEOMETRY_MISMATCH",
            ),
            (
                {"target_support_points": ((200, 0), (200, 20))},
                "TARGET_GEOMETRY_MISMATCH",
            ),
        )
        common = {
            "frame_id": "frame-a",
            "map_generation_id": "map-a",
            "map_version": 9,
            "target_hypothesis_id": "hazard-a",
            "target_centroid_x_mm": 200,
            "target_centroid_y_mm": 0,
            "target_radius_mm": 50,
            "target_support_points": ((200, 0),),
        }
        for changes, expected_reason in cases:
            with self.subTest(reason=expected_reason):
                invalid = value.reconcile(**{**common, **changes})
                self.assertEqual(invalid.status, ROUTE_INVALID)
                self.assertEqual(
                    invalid.invalidation_reason, expected_reason
                )
                self.assertEqual(invalid.route_id, value.route_id)
                self.assertEqual(invalid.version, value.version + 1)


if __name__ == "__main__":
    unittest.main()
