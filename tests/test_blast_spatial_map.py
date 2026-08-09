from copy import deepcopy
from unittest import TestCase

from robot_agent.blast_spatial_map import BlastSpatialMapBridge
from robot_agent.physical_odometry import PhysicalPose


class Clock:
    def __init__(self, value):
        self.value = value

    def __call__(self):
        self.value += 1
        return self.value


def observation(left=0, right=0):
    return {
        "motion_active": False,
        "motor_angles_deg": {
            "left_drive": left,
            "right_drive": right,
            "body": 158,
        },
        "imu": {"heading_deg": 0.0},
    }


def final_goal(current=0, navigation_enforced=False):
    return {
        "kind": "DIRECTIONAL_HEADING",
        "navigation_enforced": navigation_enforced,
        "origin_x_mm": 0,
        "origin_y_mm": 0,
        "target_x_mm": 420,
        "target_y_mm": 0,
        "desired_heading_mdeg": 0,
        "minimum_forward_progress_mm": 420,
        "heading_tolerance_mdeg": 5_000,
        "current_forward_progress_mm": current,
        "remaining_forward_progress_mm": max(0, 420 - current),
    }


def planned_leg(kind="SIDE_SEARCH", scope="SEARCH_POSITION_ONLY",
                route_eligible=False):
    return {
        "kind": kind,
        "scope": scope,
        "clearance_proven": False,
        "passage_proven": False,
        "route_eligible": route_eligible,
        "selected_side": "LEFT",
        "bind_pose": {"x_mm": 45, "y_mm": 0, "heading_mdeg": 0},
        "waypoint": {"x_mm": 90, "y_mm": 210, "heading_mdeg": 90_000},
    }


class BlastSpatialMapBridgeTests(TestCase):
    def bridge(self):
        return BlastSpatialMapBridge(
            monotonic_clock_ms=Clock(100),
            unix_clock_ms=Clock(1_000),
        )

    def test_episode_publishes_pose_only_existing_map_contract(self):
        bridge = self.bridge()

        self.assertTrue(bridge.begin_episode(
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
        ))
        self.assertTrue(bridge.offer_trace(
            episode_id="episode-a",
            final_goal=final_goal(),
            imu_heading={
                "heading_mdeg": 0,
                "reference": "EPISODE_START",
                "observed_at_unix_ms": 1_001,
            },
        ))

        value = bridge.snapshot()
        self.assertEqual(value["schema"], "robot-spatial-map/v1")
        self.assertEqual(value["status"], "pose_only")
        self.assertEqual(value["frame_kind"], "LOCAL_ODOMETRY")
        self.assertIsNone(value["resolution_mm"])
        self.assertEqual(value["robot_pose"]["x_mm"], 0)
        self.assertEqual(
            value["robot_pose"]["provenance"],
            "PROVISIONAL_ENCODER_ODOMETRY",
        )
        self.assertEqual(value["navigation_trace"]["final_goal"], final_goal())
        self.assertEqual(value["cells"], [])
        self.assertEqual(value["sensor_rays"], [])
        self.assertEqual(value["object_hypotheses"], [])
        self.assertFalse(value["localization"]["ground_truth_available"])
        self.assertEqual(value["captured_at_unix_ms"], 1_002)
        self.assertEqual(value["age_ms"], 1)
        self.assertEqual(value["robot_pose"]["age_ms"], 2)
        self.assertEqual(
            value["collision_geometry"]["geometry"],
            "ASYMMETRIC_RECTANGLE",
        )

    def test_pose_history_is_detached_bounded_and_resets_per_episode(self):
        bridge = self.bridge()
        bridge.begin_episode(
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
        )
        bridge.offer_pose(
            episode_id="episode-a",
            pose=PhysicalPose(x_mm=45, verified_motion_count=1,
                              total_forward_mm=45),
            observation=observation(90, 90),
        )
        first = bridge.snapshot()
        first["pose_history"][0]["x_mm"] = 999
        self.assertEqual(bridge.snapshot()["pose_history"][0]["x_mm"], 0)

        old_frame = bridge.snapshot()["frame_id"]
        bridge.begin_episode(
            episode_id="episode-b",
            pose=PhysicalPose(),
            observation=observation(),
        )
        second = bridge.snapshot()
        self.assertNotEqual(second["frame_id"], old_frame)
        self.assertEqual(len(second["pose_history"]), 1)
        self.assertFalse(bridge.offer_pose(
            episode_id="episode-a",
            pose=PhysicalPose(x_mm=90),
            observation=observation(),
        ))

    def test_trace_is_detached_and_carries_only_provisional_scan_geometry(self):
        bridge = self.bridge()
        bridge.begin_episode(
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
        )
        views = [{
            "scan_id": "scan-1",
            "observed_at_unix_ms": 1_001,
            "scan_pose": {"x_mm": 0, "y_mm": 0, "heading_mdeg": 0},
            "projection": {
                "schema": "blast-planar-scan-projection/v1",
                "frame": "EPISODE_LOCAL_ODOMETRY",
                "quality": "PROVISIONAL_YAW_ONLY",
                "vertical_pitch_compensated": False,
                "ultrasonic_beam_width_modeled": False,
                "scan_turn_translation_compensated": False,
                "points": [],
            },
        }]
        bridge.offer_trace(
            episode_id="episode-a",
            final_goal=final_goal(),
            planar_scan_views=views,
        )
        views[0]["scan_id"] = "changed"

        trace = bridge.snapshot()["navigation_trace"]
        self.assertEqual(
            trace["planar_scan_views"][0]["scan_id"], "scan-1"
        )
        self.assertEqual(trace["provenance"], (
            "PROVISIONAL_ENCODER_ODOMETRY + PROVISIONAL_YAW_ONLY"
        ))

    def test_enforced_local_detour_trace_accepts_shared_waypoint_kinds(self):
        waypoint_kinds = (
            "LATERAL_CLEARANCE",
            "REACQUIRE_GOAL_HEADING",
            "PASS_BEYOND_TARGET",
            "MERGE_GOAL_AXIS",
            "RESUME_GOAL_HEADING",
        )
        for kind in waypoint_kinds:
            with self.subTest(kind=kind):
                bridge = self.bridge()
                bridge.begin_episode(
                    episode_id="episode-a",
                    pose=PhysicalPose(),
                    observation=observation(),
                )
                leg = planned_leg(
                    kind=kind,
                    scope="LOCAL_DETOUR_ROUTE",
                    route_eligible=True,
                )

                self.assertTrue(bridge.offer_trace(
                    episode_id="episode-a",
                    final_goal=final_goal(navigation_enforced=True),
                    planned_leg=leg,
                ))
                trace = bridge.snapshot()["navigation_trace"]
                self.assertTrue(trace["final_goal"]["navigation_enforced"])
                self.assertEqual(trace["planned_leg"], leg)

    def test_planned_leg_claims_and_fields_remain_exact_and_conservative(self):
        bridge = self.bridge()
        bridge.begin_episode(
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
        )
        invalid_legs = []
        extra = planned_leg()
        extra["future_claim"] = False
        invalid_legs.append(extra)
        for field in ("clearance_proven", "passage_proven"):
            claimed = planned_leg(
                kind="PASS_BEYOND_TARGET",
                scope="LOCAL_DETOUR_ROUTE",
                route_eligible=True,
            )
            claimed[field] = True
            invalid_legs.append(claimed)
        invalid_legs.append(planned_leg(
            kind="SIDE_SEARCH",
            scope="SEARCH_POSITION_ONLY",
            route_eligible=True,
        ))
        invalid_legs.append(planned_leg(
            kind="SIDE_SEARCH",
            scope="LOCAL_DETOUR_ROUTE",
            route_eligible=True,
        ))

        for leg in invalid_legs:
            with self.subTest(leg=leg):
                with self.assertRaisesRegex(ValueError, "trace is invalid"):
                    bridge.offer_trace(
                        episode_id="episode-a",
                        final_goal=final_goal(navigation_enforced=True),
                        planned_leg=leg,
                    )
        mismatches = (
            (
                final_goal(navigation_enforced=False),
                planned_leg(
                    kind="PASS_BEYOND_TARGET",
                    scope="LOCAL_DETOUR_ROUTE",
                    route_eligible=True,
                ),
            ),
            (
                final_goal(navigation_enforced=True),
                planned_leg(),
            ),
        )
        for goal, leg in mismatches:
            with self.subTest(goal=goal, leg=leg):
                with self.assertRaisesRegex(ValueError, "trace is invalid"):
                    bridge.offer_trace(
                        episode_id="episode-a",
                        final_goal=goal,
                        planned_leg=leg,
                    )

    def test_invalid_or_closed_publication_fails_closed(self):
        bridge = self.bridge()
        with self.assertRaisesRegex(ValueError, "not idle and anchored"):
            bridge.begin_episode(
                episode_id="episode-a",
                pose=PhysicalPose(),
                observation={"motion_active": True},
            )
        bridge.begin_episode(
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
        )
        self.assertTrue(bridge.close(drain=True))
        self.assertFalse(bridge.offer_pose(
            episode_id="episode-a",
            pose=PhysicalPose(x_mm=45),
            observation=observation(),
        ))

    def test_invalid_nested_trace_is_rejected_without_replacing_valid_trace(self):
        bridge = self.bridge()
        bridge.begin_episode(
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
        )
        views = [{
            "scan_id": "scan-1",
            "observed_at_unix_ms": 1_001,
            "scan_pose": {"x_mm": 0, "y_mm": 0, "heading_mdeg": 0},
            "projection": {
                "schema": "blast-planar-scan-projection/v1",
                "frame": "EPISODE_LOCAL_ODOMETRY",
                "quality": "PROVISIONAL_YAW_ONLY",
                "vertical_pitch_compensated": False,
                "ultrasonic_beam_width_modeled": False,
                "scan_turn_translation_compensated": False,
                "points": [{
                    "side": "center",
                    "measured_range_mm": 300.0,
                    "relative_bearing_mdeg": 0,
                    "sensor_origin_x_mm": 110,
                    "sensor_origin_y_mm": 80,
                    "beam_heading_mdeg": 0,
                    "nominal_echo_x_mm": 410,
                    "nominal_echo_y_mm": 80,
                }],
            },
        }]
        bridge.offer_trace(
            episode_id="episode-a",
            final_goal=final_goal(),
            planar_scan_views=views,
        )
        valid = bridge.snapshot()["navigation_trace"]
        invalid = deepcopy(views)
        invalid[0]["projection"]["points"][0][
            "measured_range_mm"
        ] = float("nan")

        with self.assertRaisesRegex(ValueError, "trace is invalid"):
            bridge.offer_trace(
                episode_id="episode-a",
                final_goal=final_goal(),
                planar_scan_views=invalid,
            )

        self.assertEqual(bridge.snapshot()["navigation_trace"], valid)


if __name__ == "__main__":
    import unittest
    unittest.main()
