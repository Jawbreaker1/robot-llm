from copy import deepcopy
import math
from unittest import TestCase

from robot_agent.blast_episode_map_trace import _BlastEpisodeMapTrace
from robot_agent.blast_mission_completion import BLAST_GOAL_RADIUS_MM
from robot_agent.blast_spatial_map import BlastSpatialMapBridge
from robot_agent.coarse_navigation_grid import (
    build_coarse_navigation_grid,
    known_clear_axis_reach_mm,
    model_route_blockage,
    route_blockage_from_echoes,
)
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


def final_goal(current=0, navigation_enforced=False, lateral=0):
    return {
        "kind": "DIRECTIONAL_HEADING",
        "navigation_enforced": navigation_enforced,
        "origin_x_mm": 0,
        "origin_y_mm": 0,
        "target_x_mm": 420,
        "target_y_mm": 0,
        "goal_radius_mm": 120,
        "distance_to_goal_mm": round(math.hypot(420 - current, lateral)),
        "desired_heading_mdeg": 0,
        "minimum_forward_progress_mm": 420,
        "heading_tolerance_mdeg": 5_000,
        "current_forward_progress_mm": current,
        "current_lateral_offset_mm": lateral,
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

    def test_coarse_grid_distinguishes_blast_and_ev3(self):
        grid = build_coarse_navigation_grid(
            robots=(
                {
                    "symbol": "B", "robot_id": "blast-01",
                    "forward_mm": 0, "left_mm": 0,
                    "heading_mdeg": 0,
                },
                {
                    "symbol": "E", "robot_id": "ev3rstorm-01",
                    "forward_mm": 150, "left_mm": 150,
                    "heading_mdeg": 90_000,
                },
            ),
            goal=(420, 0),
        )

        self.assertEqual(grid["rows"][6], "....E......")
        self.assertEqual(grid["rows"][7], ".....B.....")
        self.assertEqual(
            [(robot["symbol"], robot["heading"])
             for robot in grid["robots"]],
            [("B", "UP"), ("E", "LEFT")],
        )
        self.assertEqual(grid["window"], {
            "x_min_mm": -450, "x_max_mm": 1050,
            "y_min_mm": -750, "y_max_mm": 750,
        })

    def test_coarse_grid_rolls_with_robot_in_stable_episode_coordinates(self):
        grid = build_coarse_navigation_grid(
            robots=({
                "symbol": "B", "robot_id": "blast-01",
                "forward_mm": 1_500, "left_mm": 450,
                "heading_mdeg": 0,
            },),
            goal=(1_800, 0),
            possible_obstacles=((1_800, 450),),
            window_center=(1_500, 450),
        )

        self.assertEqual(grid["window"], {
            "x_min_mm": 1_050, "x_max_mm": 2_550,
            "y_min_mm": -300, "y_max_mm": 1_200,
        })
        self.assertEqual(grid["robots"][0]["row"], 7)
        self.assertEqual(grid["robots"][0]["column"], 5)
        self.assertEqual(grid["rows"][7][5], "B")
        self.assertEqual(grid["rows"][5][5], "?")
        self.assertEqual(grid["rows"][5][8], "G")
        self.assertEqual(len(grid["robot_center_keep_out_cells"]), 9)

    def test_coarse_grid_shows_clear_ray_keep_out_and_target_overlap(self):
        robot = ({
            "symbol": "B", "robot_id": "blast-01",
            "forward_mm": 0, "left_mm": 0, "heading_mdeg": 0,
        },)
        values = {
            "robots": robot,
            "goal": (800, 0),
            "possible_obstacles": ((450, 0),),
            "clear_segments": (((0, 0), (450, 0)),),
        }

        goal_waypoint = build_coarse_navigation_grid(
            **values, waypoint=(800, 0),
        )
        blocked_waypoint = build_coarse_navigation_grid(
            **values, waypoint=(450, 0),
        )

        self.assertEqual(goal_waypoint["rows"][2], ".....X.....")
        self.assertEqual(goal_waypoint["rows"][4], "....#?#....")
        self.assertEqual(goal_waypoint["rows"][6], ".....o.....")
        self.assertEqual(blocked_waypoint["rows"][2], ".....G.....")
        self.assertEqual(blocked_waypoint["rows"][4], "....#x#....")

    def test_coarse_grid_summarizes_known_clear_episode_axes(self):
        grid = build_coarse_navigation_grid(
            robots=({
                "symbol": "B", "robot_id": "blast-01",
                "forward_mm": 0, "left_mm": 0, "heading_mdeg": 0,
            },),
            goal=(900, 0),
            clear_segments=(
                ((0, 0), (600, 0)),
                ((0, 0), (0, 600)),
                ((0, 0), (0, -300)),
                ((0, 0), (-300, 0)),
            ),
        )

        self.assertEqual(known_clear_axis_reach_mm(grid), {
            "episode_forward_mm": 600,
            "episode_left_mm": 600,
            "episode_right_mm": 150,
            "episode_back_mm": 150,
        })

    def test_goal_and_waypoint_overlay_preserve_known_clear_reach(self):
        grid = build_coarse_navigation_grid(
            robots=({
                "symbol": "B", "robot_id": "blast-01",
                "forward_mm": 0, "left_mm": 0, "heading_mdeg": 0,
            },),
            goal=(150, 0),
            waypoint=(150, 0),
            clear_segments=(((0, 0), (600, 0)),),
        )

        self.assertIn("X", grid["rows"][6])
        self.assertEqual(
            known_clear_axis_reach_mm(grid)["episode_forward_mm"],
            600,
        )

    def test_blocked_goal_marker_preserves_keep_out_meaning(self):
        goal_only = build_coarse_navigation_grid(
            robots=({
                "symbol": "B", "robot_id": "blast-01",
                "forward_mm": 0, "left_mm": 0, "heading_mdeg": 0,
            },),
            goal=(600, 0),
            possible_obstacles=((450, 0),),
        )
        goal_waypoint = build_coarse_navigation_grid(
            robots=({
                "symbol": "B", "robot_id": "blast-01",
                "forward_mm": 0, "left_mm": 0, "heading_mdeg": 0,
            },),
            goal=(600, 0),
            waypoint=(600, 0),
            possible_obstacles=((450, 0),),
        )

        self.assertIn("g", goal_only["rows"][3])
        self.assertIn("x", goal_waypoint["rows"][3])

    def test_coarse_grid_rejects_known_blocked_route_but_not_clear_detour(self):
        grid = build_coarse_navigation_grid(
            robots=({
                "symbol": "B", "robot_id": "blast-01",
                "forward_mm": 0, "left_mm": 0, "heading_mdeg": 0,
            },),
            goal=(800, 0),
            waypoint=(450, 300),
            possible_obstacles=((450, 0),),
            clear_segments=(((0, 0), (450, 0)),),
        )

        blocked = route_blockage_from_echoes(
            start=(0, 0),
            waypoints=((450, 300), (800, 300), (800, 0)),
            possible_obstacles=((330, 80),),
        )
        clear = route_blockage_from_echoes(
            start=(0, 0),
            waypoints=((0, 450), (750, 450), (800, 0)),
            possible_obstacles=((330, 80),),
        )

        self.assertEqual(
            blocked["reason"], "KNOWN_ECHO_CLEARANCE_INTERSECTION",
        )
        self.assertEqual(blocked["leg_index"], 1)
        self.assertEqual(
            blocked["blocking_echo_point"],
            {"x_mm": 330, "y_mm": 80},
        )
        self.assertIsNone(clear)

    def test_route_veto_accepts_pure_side_clearance_beside_front_echo(self):
        self.assertIsNone(route_blockage_from_echoes(
            start=(0, 0),
            waypoints=((0, 450),),
            possible_obstacles=((300, 0),),
        ))

    def test_model_route_rejects_diagonal_before_motion(self):
        blocked = model_route_blockage(
            start=(0, 0),
            waypoints=((300, -250),),
            possible_obstacles=(),
        )

        self.assertEqual(blocked, {
            "reason": "NON_ORTHOGONAL_ROUTE_LEG",
            "leg_index": 1,
            "basis": "COARSE_EPISODE_AXES",
            "axis_tolerance_mm": 75,
            "delta_x_mm": 300,
            "delta_y_mm": -250,
        })

    def test_model_route_allows_half_coarse_cell_of_odometry_drift(self):
        self.assertIsNone(model_route_blockage(
            start=(0, 0),
            waypoints=((70, -450),),
            possible_obstacles=(),
        ))

    def test_model_route_rejects_one_cell_diagonal_as_not_orthogonal(self):
        blocked = model_route_blockage(
            start=(0, 0),
            waypoints=((150, 150),),
            possible_obstacles=(),
        )

        self.assertEqual(blocked["reason"], "NON_ORTHOGONAL_ROUTE_LEG")

    def test_model_route_uses_the_shared_echo_clearance(self):
        route = ((800, 250),)
        obstacle = ((320, 80),)

        direct = route_blockage_from_echoes(
            start=(0, 250),
            waypoints=route,
            possible_obstacles=obstacle,
        )
        model = model_route_blockage(
            start=(0, 250),
            waypoints=route,
            possible_obstacles=obstacle,
        )

        self.assertEqual(model, direct)
        self.assertEqual(model["clearance_mm"], 200)

    def test_route_veto_rejects_sparse_ray_gap_that_hits_robot_body(self):
        blocked = route_blockage_from_echoes(
            start=(0, 0),
            waypoints=((350, -250),),
            possible_obstacles=((335, -50),),
        )

        self.assertEqual(
            blocked["reason"], "KNOWN_ECHO_CLEARANCE_INTERSECTION",
        )

    def test_route_blockage_selects_nearest_crossing_not_scan_order(self):
        route = ((900, 0),)
        obstacles = ((750, 0), (300, 0))

        forward_order = route_blockage_from_echoes(
            start=(0, 0),
            waypoints=route,
            possible_obstacles=obstacles,
        )
        reverse_order = route_blockage_from_echoes(
            start=(0, 0),
            waypoints=route,
            possible_obstacles=tuple(reversed(obstacles)),
        )

        self.assertEqual(forward_order, reverse_order)
        self.assertEqual(
            forward_order["blocking_echo_point"],
            {"x_mm": 300, "y_mm": 0},
        )

    def test_route_can_move_away_from_adjacent_echo(self):
        self.assertIsNone(route_blockage_from_echoes(
            start=(150, 0),
            waypoints=((0, 0),),
            possible_obstacles=((300, 0),),
        ))

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

    def test_trace_retains_latest_sixteen_scans_without_host_route(self):
        bridge = BlastSpatialMapBridge()
        trace = _BlastEpisodeMapTrace(
            bridge=bridge,
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
            observed_at_unix_ms=1_000,
            episode_start_heading=0.0,
            minimum_forward_progress_mm=420,
        )
        self.assertEqual(
            bridge.snapshot()["navigation_trace"]["coarse_grid"]["rows"][7],
            ".....B.....",
        )
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
                scan_view=scan_view,
            )

        value = bridge.snapshot()["navigation_trace"]
        self.assertEqual(len(value["planar_scan_views"]), 16)
        self.assertEqual(
            [view["scan_id"] for view in value["planar_scan_views"]],
            [f"episode-a-scan-{index}" for index in range(2, 18)],
        )
        self.assertEqual(
            len({view["scan_id"] for view in value["planar_scan_views"]}),
            16,
        )
        self.assertIsNone(value["local_detour_route"])
        self.assertIsNone(value["planned_leg"])
        self.assertFalse(value["final_goal"]["navigation_enforced"])

        self.assertTrue(trace.finalize())
        final = bridge.snapshot()["navigation_trace"]
        self.assertIsNone(final["local_detour_route"])
        self.assertIsNone(final["planned_leg"])
        self.assertFalse(final["final_goal"]["navigation_enforced"])

    def test_planner_map_evidence_is_compact_coarse_map_and_detached(self):
        bridge = self.bridge()
        trace = _BlastEpisodeMapTrace(
            bridge=bridge,
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
                "observed_at_unix_ms": 1_001 + index,
                "scan_pose": {
                    "x_mm": index * 10,
                    "y_mm": -index * 5,
                    "heading_mdeg": index * 1_000,
                },
                "projection": {
                    "points": [{
                        "side": "center",
                        "measured_range_mm": 225.0,
                        "relative_bearing_mdeg": 0,
                        "sensor_origin_x_mm": 0,
                        "sensor_origin_y_mm": 0,
                        "beam_heading_mdeg": 0,
                        "nominal_echo_x_mm": 200 + index,
                        "nominal_echo_y_mm": -100 - index,
                    }],
                },
            })

        planner_pose = PhysicalPose(
            x_mm=45, y_mm=-10, heading_mdeg=-90_000,
        )
        evidence = trace.planner_local_map_evidence(planner_pose)
        self.assertEqual(evidence["schema"], "blast-local-map-evidence/v1")
        self.assertEqual(evidence["unobserved_space"], "UNKNOWN_NOT_FREE")
        self.assertEqual(evidence["known_clear_axis_reach_mm"], {
            "episode_forward_mm": 0,
            "episode_left_mm": 0,
            "episode_right_mm": 0,
            "episode_back_mm": 0,
        })
        self.assertEqual(
            evidence["direct_goal_blockage"]["reason"],
            "KNOWN_ECHO_CLEARANCE_INTERSECTION",
        )
        self.assertNotIn("keep_out_regions", evidence)
        self.assertNotIn(
            "blocking_region", evidence["direct_goal_blockage"],
        )
        self.assertEqual(evidence["robot_pose"], {
            "x_mm": 45, "y_mm": -10, "heading_mdeg": -90_000,
        })
        self.assertNotIn("obstacle_regions", evidence)
        self.assertEqual(evidence["directional_goal"], {
            "target_x_mm": 420,
            "target_y_mm": 0,
            "desired_heading_mdeg": 0,
            "heading_error_mdeg": 90_000,
            "goal_radius_mm": BLAST_GOAL_RADIUS_MM,
            "distance_to_goal_mm": 375,
            "remaining_forward_progress_mm": 375,
            "signed_forward_error_mm": 375,
            "longitudinal_relation": "BEFORE_GOAL_LINE",
            "goal_vector": {
                "delta_x_mm": 375,
                "delta_y_mm": 10,
                "distance_mm": 375,
            },
            "corridor_entered": False,
            "heading_aligned": False,
        })
        self.assertEqual(evidence["coarse_grid"], {
            "cell_size_mm": 150,
            "window": {
                "x_min_mm": -450,
                "x_max_mm": 1050,
                "y_min_mm": -750,
                "y_max_mm": 750,
            },
            "rows": [
                {"x_mm": 1050, "cells": "..........."},
                {"x_mm": 900, "cells": "..........."},
                {"x_mm": 750, "cells": "..........."},
                {"x_mm": 600, "cells": "..........."},
                {"x_mm": 450, "cells": ".....G....."},
                {"x_mm": 300, "cells": ".....###..."},
                {"x_mm": 150, "cells": ".....#?#..."},
                {"x_mm": 0, "cells": ".....B##..."},
                {"x_mm": -150, "cells": "..........."},
                {"x_mm": -300, "cells": "..........."},
                {"x_mm": -450, "cells": "..........."},
            ],
            "column_y_mm": [
                750, 600, 450, 300, 150, 0,
                -150, -300, -450, -600, -750,
            ],
        })
        self.assertNotIn("scan_views", evidence)
        self.assertNotIn("robot_center_keep_out_cells", evidence["coarse_grid"])
        self.assertNotIn("robot_footprint_mm", evidence)
        self.assertEqual(evidence["visited_cells"], [
            {"x_mm": 0, "y_mm": 0},
        ])
        self.assertNotIn(
            "robot_relation", evidence["direct_goal_blockage"],
        )

        trace.record(
            pose=PhysicalPose(x_mm=330, y_mm=310),
            observation=observation(), pose_observed=True, scan_view=None,
        )
        moved = trace.planner_local_map_evidence(
            PhysicalPose(x_mm=330, y_mm=310),
        )
        self.assertEqual(moved["visited_cells"], [
            {"x_mm": 0, "y_mm": 0},
            {"x_mm": 300, "y_mm": 300},
        ])
        self.assertEqual(moved["coarse_grid"]["window"], {
            "x_min_mm": -150, "x_max_mm": 1_350,
            "y_min_mm": -450, "y_max_mm": 1_050,
        })
        self.assertEqual(
            [row["x_mm"] for row in moved["coarse_grid"]["rows"]],
            [1350, 1200, 1050, 900, 750, 600, 450, 300, 150, 0, -150],
        )
        self.assertEqual(
            moved["coarse_grid"]["column_y_mm"],
            [1050, 900, 750, 600, 450, 300, 150, 0, -150, -300, -450],
        )

        evidence["coarse_grid"]["rows"][0]["cells"] = "changed"
        fresh = trace.planner_local_map_evidence(PhysicalPose())
        self.assertNotEqual(
            fresh["coarse_grid"]["rows"][0]["cells"], "changed",
        )

    def test_visited_trail_retains_a_room_scale_ordered_history(self):
        trace = _BlastEpisodeMapTrace(
            bridge=self.bridge(),
            episode_id="episode-a",
            pose=PhysicalPose(),
            observation=observation(),
            observed_at_unix_ms=1_000,
            episode_start_heading=0.0,
            minimum_forward_progress_mm=420,
        )

        for index in range(140):
            trace._record_visited_cell(PhysicalPose(x_mm=index * 150))

        trail = trace.planner_local_map_evidence(
            PhysicalPose(x_mm=139 * 150),
        )["visited_cells"]
        self.assertEqual(len(trail), 128)
        self.assertEqual(trail[0], {"x_mm": 1_800, "y_mm": 0})
        self.assertEqual(trail[-1], {"x_mm": 20_850, "y_mm": 0})

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
