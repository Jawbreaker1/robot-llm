from copy import deepcopy
import math
import time
from unittest import TestCase

from robot_agent.blast_episode_map_trace import _BlastEpisodeMapTrace
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
        self.assertEqual(value["local_generation_id"], "episode-a")
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

    def test_failed_scan_marks_stale_pose_as_localization_lost(self):
        bridge = self.bridge()
        bridge.begin_episode(
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
        )

        self.assertTrue(bridge.invalidate_localization(
            episode_id="episode-a",
        ))

        value = bridge.snapshot()
        self.assertEqual(value["status"], "unavailable")
        self.assertEqual(value["reason_code"], "localization_lost")
        self.assertIsNone(value["robot_pose"])
        self.assertFalse(value["localization"]["valid"])
        self.assertFalse(bridge.invalidate_localization(
            episode_id="wrong-episode",
        ))

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
        self.assertEqual(first["local_generation_id"], "episode-a")
        bridge.begin_episode(
            episode_id="episode-b",
            pose=PhysicalPose(),
            observation=observation(),
        )
        second = bridge.snapshot()
        self.assertNotEqual(second["frame_id"], old_frame)
        self.assertEqual(second["local_generation_id"], "episode-b")
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
        # NVD/unsettled rays are absent from the validated projection and
        # therefore can never become obstacle hypotheses.
        self.assertEqual(bridge.snapshot()["object_hypotheses"], [])

    def test_trace_accepts_nine_dense_provisional_scan_points(self):
        bridge = self.bridge()
        bridge.begin_episode(
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
        )
        sides = (
            "center", "left_1", "left_2", "left_3", "left_4",
            "right_1", "right_2", "right_3", "right_4",
        )
        bearings = (0, 25_000, 50_000, 75_000, 95_000,
                    -25_000, -50_000, -75_000, -95_000)
        points = []
        for side, bearing in zip(sides, bearings):
            angle = math.radians(bearing / 1_000)
            points.append({
                "side": side,
                "measured_range_mm": 100.0,
                "relative_bearing_mdeg": bearing,
                "sensor_origin_x_mm": 0,
                "sensor_origin_y_mm": 0,
                "beam_heading_mdeg": bearing,
                "nominal_echo_x_mm": round(100 * math.cos(angle)),
                "nominal_echo_y_mm": round(100 * math.sin(angle)),
            })
        views = [{
            "scan_id": "dense-scan",
            "observed_at_unix_ms": 1_001,
            "scan_pose": {"x_mm": 0, "y_mm": 0, "heading_mdeg": 0},
            "projection": {
                "schema": "blast-planar-scan-projection/v1",
                "frame": "EPISODE_LOCAL_ODOMETRY",
                "quality": "PROVISIONAL_YAW_ONLY",
                "vertical_pitch_compensated": False,
                "ultrasonic_beam_width_modeled": False,
                "scan_turn_translation_compensated": False,
                "points": points,
            },
        }]

        self.assertTrue(bridge.offer_trace(
            episode_id="episode-a",
            final_goal=final_goal(),
            planar_scan_views=views,
        ))
        self.assertEqual(
            len(bridge.snapshot()["navigation_trace"]
                ["planar_scan_views"][0]["projection"]["points"]),
            9,
        )
        snapshot = bridge.snapshot()
        self.assertEqual(snapshot["status"], "qualitative_only")
        self.assertEqual(snapshot["sensor_rays"], [])
        self.assertEqual(len(snapshot["object_hypotheses"]), 1)
        hypothesis = snapshot["object_hypotheses"][0]
        self.assertEqual(
            hypothesis["classification"],
            "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER",
        )
        self.assertEqual(
            hypothesis["geometry_kind"],
            "PROVISIONAL_ULTRASONIC_ECHO_CLUSTER",
        )
        self.assertEqual(hypothesis["evidence_count"], 9)
        self.assertEqual(hypothesis["source_scan_ids"], ["dense-scan"])
        self.assertTrue(hypothesis["settled_measured_only"])
        self.assertTrue(hypothesis["provisional"])
        self.assertTrue(hypothesis["read_only"])
        self.assertEqual(len(hypothesis["support_points"]), 9)

    def test_full_detour_route_waypoints_are_strict_and_status_bearing(self):
        bridge = self.bridge()
        bridge.begin_episode(
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
        )
        route = {
            "schema": "robot-local-detour-route/v1",
            "read_only": True,
            "provisional": True,
            "route_id": "route-a",
            "version": 2,
            "status": "ACTIVE",
            "detour_side": "RIGHT_OF_GOAL",
            "active_index": 1,
            "waypoints": [
                {
                    "ordinal": 0,
                    "kind": "LATERAL_CLEARANCE",
                    "x_mm": 0,
                    "y_mm": -225,
                    "heading_mdeg": -90_000,
                    "fact_key": None,
                    "status": "COMPLETED",
                },
                {
                    "ordinal": 1,
                    "kind": "REACQUIRE_GOAL_HEADING",
                    "x_mm": 0,
                    "y_mm": -225,
                    "heading_mdeg": 0,
                    "fact_key": "GOAL_HEADING_ALIGNED",
                    "status": "ACTIVE",
                },
                {
                    "ordinal": 2,
                    "kind": "PASS_BEYOND_TARGET",
                    "x_mm": 500,
                    "y_mm": -225,
                    "heading_mdeg": 0,
                    "fact_key": "TARGET_BEHIND",
                    "status": "UPCOMING",
                },
            ],
        }
        self.assertTrue(bridge.offer_trace(
            episode_id="episode-a",
            final_goal=final_goal(navigation_enforced=True),
            planned_leg=planned_leg(
                kind="REACQUIRE_GOAL_HEADING",
                scope="LOCAL_DETOUR_ROUTE",
                route_eligible=True,
            ),
            local_detour_route=route,
        ))
        stored = bridge.snapshot()["navigation_trace"]["local_detour_route"]
        self.assertEqual(
            [item["status"] for item in stored["waypoints"]],
            ["COMPLETED", "ACTIVE", "UPCOMING"],
        )

        invalid = deepcopy(route)
        invalid["waypoints"][2]["status"] = "COMPLETED"
        with self.assertRaisesRegex(ValueError, "trace is invalid"):
            bridge.offer_trace(
                episode_id="episode-a",
                final_goal=final_goal(navigation_enforced=True),
                planned_leg=planned_leg(
                    kind="REACQUIRE_GOAL_HEADING",
                    scope="LOCAL_DETOUR_ROUTE",
                    route_eligible=True,
                ),
                local_detour_route=invalid,
            )

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

    def test_terminal_trace_invalidates_active_route_without_erasing_it(self):
        bridge = BlastSpatialMapBridge()
        observed_at_unix_ms = time.time_ns() // 1_000_000
        trace = _BlastEpisodeMapTrace(
            bridge=bridge,
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
            observed_at_unix_ms=observed_at_unix_ms,
            episode_start_heading=0.0,
            minimum_forward_progress_mm=420,
        )
        trace.navigation_enforced = True
        trace.planned_leg = planned_leg(
            kind="REACQUIRE_GOAL_HEADING",
            scope="LOCAL_DETOUR_ROUTE",
            route_eligible=True,
        )
        trace.local_detour_route = {
            "schema": "robot-local-detour-route/v1",
            "read_only": True,
            "provisional": True,
            "route_id": "route-a",
            "version": 2,
            "status": "ACTIVE",
            "detour_side": "RIGHT_OF_GOAL",
            "active_index": 1,
            "waypoints": [
                {"ordinal": 0, "kind": "LATERAL_CLEARANCE",
                 "x_mm": 0, "y_mm": -225, "heading_mdeg": -90_000,
                 "fact_key": None, "status": "COMPLETED"},
                {"ordinal": 1, "kind": "REACQUIRE_GOAL_HEADING",
                 "x_mm": 0, "y_mm": -225, "heading_mdeg": 0,
                 "fact_key": "GOAL_HEADING_ALIGNED", "status": "ACTIVE"},
                {"ordinal": 2, "kind": "PASS_BEYOND_TARGET",
                 "x_mm": 500, "y_mm": -225, "heading_mdeg": 0,
                 "fact_key": "TARGET_BEHIND", "status": "UPCOMING"},
            ],
        }
        scan_view = {
            "scan_pose": {"x_mm": 0, "y_mm": 0, "heading_mdeg": 0},
            "planar_projection": {
                "schema": "blast-planar-scan-projection/v1",
                "frame": "EPISODE_LOCAL_ODOMETRY",
                "quality": "PROVISIONAL_YAW_ONLY",
                "vertical_pitch_compensated": False,
                "ultrasonic_beam_width_modeled": False,
                "scan_turn_translation_compensated": False,
                "points": [],
            },
        }
        for _unused in range(17):
            trace.record(
                pose=PhysicalPose(),
                observation=observation(),
                pose_observed=False,
                selected_side=None,
                waypoint=None,
                bind_pose=PhysicalPose(),
                scan_view=scan_view,
            )

        active = bridge.snapshot()["navigation_trace"]
        self.assertEqual(len(active["planar_scan_views"]), 16)
        self.assertEqual(
            [view["scan_id"] for view in active["planar_scan_views"]],
            [f"episode-a-scan-{index}" for index in range(2, 18)],
        )
        self.assertEqual(
            len({view["scan_id"] for view in active["planar_scan_views"]}),
            16,
        )
        self.assertEqual(active["local_detour_route"]["status"], "ACTIVE")

        self.assertTrue(trace.finalize())

        value = bridge.snapshot()["navigation_trace"]
        self.assertEqual(value["local_detour_route"]["status"], "INVALID")
        self.assertEqual(
            [item["status"] for item in value["local_detour_route"]["waypoints"]],
            ["COMPLETED", "UPCOMING", "UPCOMING"],
        )
        self.assertIsNone(value["planned_leg"])
        self.assertFalse(value["final_goal"]["navigation_enforced"])

        self.assertTrue(trace.clear_route(
            pose=PhysicalPose(), observation=observation(),
        ))
        self.assertIsNone(
            bridge.snapshot()["navigation_trace"]["local_detour_route"]
        )

    def test_planner_map_evidence_is_compact_echo_only_and_detached(self):
        trace = _BlastEpisodeMapTrace(
            bridge=self.bridge(),
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
            observed_at_unix_ms=1_000,
            episode_start_heading=0.0,
            minimum_forward_progress_mm=420,
        )
        for index in range(5):
            trace.planar_scan_views.append({
                "scan_id": "scan-{}".format(index),
                "scan_pose": {
                    "x_mm": index * 10,
                    "y_mm": -index * 5,
                    "heading_mdeg": index * 1_000,
                },
                "projection": {
                    "points": [{
                        "nominal_echo_x_mm": 200 + index,
                        "nominal_echo_y_mm": -100 - index,
                    }],
                },
            })

        evidence = trace.planner_local_map_evidence(
            PhysicalPose(x_mm=45, y_mm=-10, heading_mdeg=-90_000),
        )

        self.assertEqual(evidence["schema"], "blast-local-map-evidence/v1")
        self.assertEqual(evidence["occupancy_model"], "NONE")
        self.assertEqual(evidence["unobserved_space"], "UNKNOWN_NOT_FREE")
        self.assertEqual(evidence["robot_pose"], {
            "x_mm": 45, "y_mm": -10, "heading_mdeg": -90_000,
        })
        self.assertEqual(evidence["directional_goal"], {
            "target_x_mm": 420,
            "target_y_mm": 0,
            "desired_heading_mdeg": 0,
            "remaining_forward_progress_mm": 375,
        })
        self.assertEqual(
            [view["scan_id"] for view in evidence["scan_views"]],
            ["scan-1", "scan-2", "scan-3", "scan-4"],
        )
        self.assertTrue(evidence["truncated"])
        self.assertEqual(
            evidence["scan_views"][-1]["echo_points"],
            [{"x_mm": 204, "y_mm": -104}],
        )
        self.assertEqual(evidence["robot_footprint_mm"], {
            "front": 110,
            "rear": 60,
            "left": 105,
            "right": 100,
            "clearance_margin": 10,
        })

        evidence["scan_views"][0]["echo_points"][0]["x_mm"] = 999
        fresh = trace.planner_local_map_evidence(PhysicalPose())
        self.assertEqual(
            fresh["scan_views"][0]["echo_points"][0]["x_mm"], 201,
        )

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
