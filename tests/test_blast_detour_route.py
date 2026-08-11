from dataclasses import replace
from types import SimpleNamespace
import unittest

from robot_agent.blast_detour_route import (
    bind_blast_detour_route,
    blast_detour_action_sweep_is_clear,
    blast_detour_guidance,
    blast_detour_needs_pass_buffer,
    blast_detour_required_slots,
    blast_detour_scan_allows_progress,
    blast_detour_scan_sweep_is_clear,
)
from robot_agent.local_detour_route import (
    LATERAL_CLEARANCE,
    MERGE_GOAL_AXIS,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    TURN_LEFT_90,
)
from robot_agent.physical_navigation_mission import DirectionalMission
from robot_agent.physical_odometry import PhysicalPose


def view(pose, points):
    return {
        "scan_pose": pose.to_dict(),
        "planar_projection": {
            "schema": "blast-planar-scan-projection/v1",
            "frame": "EPISODE_LOCAL_ODOMETRY",
            "quality": "PROVISIONAL_YAW_ONLY",
            "points": [
                {
                    "side": side,
                    "sensor_origin_x_mm": pose.x_mm,
                    "sensor_origin_y_mm": pose.y_mm,
                    "nominal_echo_x_mm": x_mm,
                    "nominal_echo_y_mm": y_mm,
                }
                for side, x_mm, y_mm in points
            ],
        },
    }


def mission():
    return DirectionalMission.begin(
        episode_id="episode-route",
        minimum_forward_progress_mm=420,
        pose=PhysicalPose(),
    )


def waypoint(side, origin_pose=PhysicalPose()):
    return {
        "search_basis": "PROVISIONAL_SAME_DEPTH_ECHO_REACH",
        "search_target_capped": False,
        "selected_side": side,
        "origin_pose": origin_pose.to_dict(),
    }


def with_settled_scan(value, distances):
    headings = {
        "center": 0.0,
        "left_near": -22.0,
        "left_far": -45.0,
        "right_near": 24.0,
        "right_far": 47.0,
    }
    measured = {
        side: distance for side, distance in distances if distance != 2_000
    }
    for point in value["planar_projection"]["points"]:
        point["measured_range_mm"] = measured[point["side"]]
    value["scan"] = {
        "schema": "blast-scan-front-arc/v3",
        "state": "complete",
        "result": "restored",
        "start_heading_deg": 0.0,
        "final_heading_deg": 0.0,
        "restoration_error_deg": 0.0,
        "restoration_verified": True,
        "all_observations_settled": True,
        "rays": [
            {
                "side": side,
                "distance_mm": distance,
                "range_state": (
                    "NO_VALID_DISTANCE" if distance == 2_000
                    else "MEASURED"
                ),
                "body_motor_angle_deg": 158,
                "heading_deg": headings[side],
                "relative_heading_deg": headings[side],
                "observation_settled": True,
            }
            for side, distance in distances
        ],
    }
    return value


def live_mixed_views(*, near_echo_x=709, far_settled=True):
    detour_origin = PhysicalPose(x_mm=137, y_mm=-2, heading_mdeg=-1_715)
    current = PhysicalPose(x_mm=121, y_mm=276, heading_mdeg=1_225)
    origin = with_settled_scan(view(detour_origin, (
        ("center", 329, 72),
        ("left_near", 329, 156),
        ("right_near", 333, -5),
    )), (
        ("center", 80),
        ("left_near", 125),
        ("left_far", 2_000),
        ("right_near", 69),
        ("right_far", 2_000),
    ))
    side = with_settled_scan(view(current, (
        ("center", 1_497, 385),
        ("left_near", 1_377, 916),
        ("left_far", 718, 1_026),
        ("right_near", near_echo_x, 108),
    )), (
        ("center", 1_268),
        ("left_near", 1_298),
        ("left_far", 845),
        ("right_near", 496 if near_echo_x == 709 else 390),
        ("right_far", 2_000),
    ))
    if not far_settled:
        side["scan"]["all_observations_settled"] = False
        side["scan"]["rays"][4]["observation_settled"] = False
    return detour_origin, current, origin, side


class BlastDetourRouteTests(unittest.TestCase):
    def test_live_moved_origin_and_mixed_far_view_bind_provisional_route(self):
        detour_origin, current, origin, side = live_mixed_views()

        route = bind_blast_detour_route(
            origin_view=origin,
            side_view=side,
            selected_side="LEFT",
            side_waypoint=waypoint("LEFT", detour_origin),
            mission=mission(),
            current_pose=current,
        )

        self.assertEqual(route.created_pose, detour_origin)
        self.assertEqual((route.goal_origin_x_mm, route.goal_origin_y_mm), (0, 0))
        self.assertEqual(route.route_lateral_offset_mm, 301)
        self.assertEqual(route.pass_longitudinal_offset_mm, 611)
        self.assertTrue(blast_detour_scan_allows_progress(
            side,
            role="PASS",
            selected_side="LEFT",
            minimum_clearance_mm=120,
            route=route,
        ))

    def test_mixed_far_view_requires_settled_no_return_and_echo_past_pass(self):
        cases = (live_mixed_views(far_settled=False), live_mixed_views(
            near_echo_x=600,
        ))
        malformed = live_mixed_views()
        malformed[3]["planar_projection"]["points"][3][
            "measured_range_mm"
        ] = None
        cases += (malformed,)
        for detour_origin, current, origin, failed_side in cases:
            with self.subTest(failed_side=failed_side):
                with self.assertRaises(ValueError):
                    bind_blast_detour_route(
                        origin_view=origin,
                        side_view=failed_side,
                        selected_side="LEFT",
                        side_waypoint=waypoint("LEFT", detour_origin),
                        mission=mission(),
                        current_pose=current,
                    )

    def test_scan_sweep_uses_live_two_pulse_excursion_geometry(self):
        route = SimpleNamespace(
            target_centroid_x_mm=396,
            target_centroid_y_mm=80,
            target_radius_mm=166,
        )

        self.assertTrue(blast_detour_scan_sweep_is_clear(
            route,
            PhysicalPose(x_mm=-21, y_mm=369, heading_mdeg=-980),
        ))
        self.assertFalse(blast_detour_scan_sweep_is_clear(
            route,
            PhysicalPose(x_mm=60, y_mm=80),
        ))

    def test_pass_and_final_scans_require_measured_clearance(self):
        pose = PhysicalPose(x_mm=850, y_mm=391)

        def scan_view(center, merge, merge_state="MEASURED"):
            value = view(pose, (
                ("center", 1_000, 391),
                ("right_near", 900, -150),
            ))
            for point in value["planar_projection"]["points"]:
                point["measured_range_mm"] = (
                    center if point["side"] == "center" else merge
                )
            value["scan"] = {
                "rays": [
                    {
                        "side": "center",
                        "distance_mm": center,
                        "range_state": "MEASURED",
                    },
                    {
                        "side": "right_near",
                        "distance_mm": merge,
                        "range_state": merge_state,
                    },
                ],
            }
            return value

        route = SimpleNamespace(
            goal_origin_x_mm=0,
            goal_origin_y_mm=0,
            goal_heading_mdeg=0,
        )

        self.assertTrue(blast_detour_scan_allows_progress(
            scan_view(121, 121),
            role="PASS",
            selected_side="LEFT",
            minimum_clearance_mm=120,
            route=route,
        ))
        self.assertFalse(blast_detour_scan_allows_progress(
            scan_view(121, 54),
            role="PASS",
            selected_side="LEFT",
            minimum_clearance_mm=120,
            route=route,
        ))
        self.assertFalse(blast_detour_scan_allows_progress(
            scan_view(121, 2_000, "NO_VALID_DISTANCE"),
            role="PASS",
            selected_side="LEFT",
            minimum_clearance_mm=120,
            route=route,
        ))
        self.assertFalse(blast_detour_scan_allows_progress(
            scan_view(54, 500),
            role="FINAL",
            selected_side="LEFT",
            minimum_clearance_mm=120,
        ))

    def test_live_two_view_geometry_binds_shared_left_route(self):
        current = PhysicalPose(x_mm=-21, y_mm=369, heading_mdeg=-980)

        route = bind_blast_detour_route(
            origin_view=view(PhysicalPose(), (
                ("center", 396, 80),
                ("left_near", 376, 246),
                ("right_near", 392, -70),
            )),
            side_view=view(current, (
                ("center", 1_569, 449),
                ("right_near", 1_200, -112),
            )),
            selected_side="LEFT",
            side_waypoint=waypoint("LEFT"),
            mission=mission(),
            current_pose=current,
        )

        self.assertEqual(route.active_waypoint.kind, LATERAL_CLEARANCE)
        self.assertEqual(route.target_centroid_x_mm, 396)
        self.assertEqual(route.target_centroid_y_mm, 80)
        self.assertEqual(route.target_radius_mm, 166)
        self.assertEqual(route.route_lateral_offset_mm, 391)
        self.assertEqual(route.pass_longitudinal_offset_mm, 760)
        self.assertGreater(blast_detour_required_slots(route, current), 20)
        guidance = blast_detour_guidance(
            route,
            current,
            (ADVANCE, TURN_LEFT_90),
        )
        self.assertEqual(
            guidance.allowed_motion_actions,
            frozenset((TURN_LEFT_90,)),
        )

    def test_far_center_alone_cannot_hide_a_wider_side_obstacle(self):
        current = PhysicalPose(x_mm=-21, y_mm=369, heading_mdeg=-980)

        with self.assertRaises(ValueError):
            bind_blast_detour_route(
                origin_view=view(PhysicalPose(), (
                    ("center", 396, 80),
                    ("left_near", 376, 246),
                    ("right_near", 392, -70),
                )),
                side_view=view(current, (
                    ("center", 1_569, 449),
                    # This merge-facing ray hits the same wider front face;
                    # the center ray seeing far ahead is not enough.
                    ("right_near", 396, 263),
                )),
                selected_side="LEFT",
                side_waypoint=waypoint("LEFT"),
                mission=mission(),
                current_pose=current,
            )

        with self.assertRaises(ValueError):
            bind_blast_detour_route(
                origin_view=view(PhysicalPose(), (
                    ("center", 396, 80),
                    ("left_near", 376, 246),
                    ("right_near", 392, -70),
                )),
                side_view=view(current, (
                    ("center", 1_569, 449),
                    # It crosses the inner corridor, but the echo at the
                    # pass plane leaves no room for BLAST's merge sweep.
                    ("right_near", 760, 180),
                )),
                selected_side="LEFT",
                side_waypoint=waypoint("LEFT"),
                mission=mission(),
                current_pose=current,
            )

    def test_right_side_uses_only_right_same_depth_reach(self):
        current = PhysicalPose(x_mm=-10, y_mm=-210, heading_mdeg=1_000)

        route = bind_blast_detour_route(
            origin_view=view(PhysicalPose(), (
                ("center", 396, 80),
                ("left_near", 376, 246),
                ("right_near", 392, -70),
            )),
            side_view=view(current, (
                ("center", 1_500, 0),
                ("left_near", 1_200, 390),
            )),
            selected_side="RIGHT",
            side_waypoint=waypoint("RIGHT"),
            mission=mission(),
            current_pose=current,
        )

        self.assertEqual(route.target_radius_mm, 150)
        self.assertEqual(route.route_lateral_offset_mm, -220)
        self.assertEqual(route.active_waypoint.kind, LATERAL_CLEARANCE)

    def test_short_or_untrusted_view_refuses_route(self):
        origin = view(PhysicalPose(), (
            ("center", 396, 80),
            ("left_near", 376, 246),
        ))
        current = PhysicalPose(x_mm=0, y_mm=391, heading_mdeg=0)
        cases = (
            (view(current, (("center", 600, 471),)), waypoint("LEFT")),
            (view(current, (("center", 1_500, 471),)), {
                **waypoint("LEFT"),
                "search_target_capped": True,
            }),
        )
        for side_view, side_waypoint in cases:
            with self.subTest(side_waypoint=side_waypoint):
                with self.assertRaises(ValueError):
                    bind_blast_detour_route(
                        origin_view=origin,
                        side_view=side_view,
                        selected_side="LEFT",
                        side_waypoint=side_waypoint,
                        mission=mission(),
                        current_pose=current,
                    )

    def test_guidance_advances_to_merge_after_pass(self):
        current = PhysicalPose(x_mm=-21, y_mm=369, heading_mdeg=-980)
        route = bind_blast_detour_route(
            origin_view=view(PhysicalPose(), (
                ("center", 396, 80),
                ("left_near", 376, 246),
            )),
            side_view=view(current, (
                ("center", 1_569, 449),
                ("right_near", 1_200, -112),
            )),
            selected_side="LEFT",
            side_waypoint=waypoint("LEFT"),
            mission=mission(),
            current_pose=current,
        )

        pass_pose = PhysicalPose(
            x_mm=route.pass_longitudinal_offset_mm,
            y_mm=route.route_lateral_offset_mm,
            heading_mdeg=0,
        )
        merge_route = replace(route, version=route.version + 3, active_index=3)
        self.assertTrue(
            blast_detour_needs_pass_buffer(merge_route, pass_pose)
        )
        buffered = PhysicalPose(
            x_mm=route.pass_longitudinal_offset_mm + 90,
            y_mm=route.route_lateral_offset_mm,
            heading_mdeg=0,
        )
        guidance = blast_detour_guidance(
            merge_route,
            buffered,
            (ADVANCE,),
        )

        self.assertEqual(guidance.active_waypoint_kind, MERGE_GOAL_AXIS)
        self.assertFalse(
            blast_detour_needs_pass_buffer(merge_route, buffered)
        )

    def test_maximum_pulse_sweep_blocks_large_envelope_merge(self):
        current = PhysicalPose(x_mm=0, y_mm=-414, heading_mdeg=0)
        route = bind_blast_detour_route(
            origin_view=view(PhysicalPose(), (
                ("center", 400, 80),
                ("right_near", 400, -264),
            )),
            side_view=view(current, (
                ("center", 1_900, -334),
                ("left_near", 1_200, 258),
            )),
            selected_side="RIGHT",
            side_waypoint=waypoint("RIGHT"),
            mission=mission(),
            current_pose=current,
        )
        merge_route = replace(route, version=route.version + 3, active_index=3)

        self.assertFalse(blast_detour_action_sweep_is_clear(
            merge_route,
            PhysicalPose(x_mm=912, y_mm=-78, heading_mdeg=110_000),
            ADVANCE,
        ))


if __name__ == "__main__":
    unittest.main()
