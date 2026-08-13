from copy import deepcopy
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
    blast_side_view_associates_frozen_target,
)
from robot_agent.blast_observation_monitor import blast_range_state
from robot_agent.blast_scan_observation import (
    SCAN_ANGULAR_RAY_SIDES,
    encoder_relative_bearing_deg,
)
from robot_agent.blast_scan_planar_projection import (
    project_blast_scan_planar_surfaces,
)
from robot_agent.blast_side_search_geometry import side_search_waypoint
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


def encoder_scan_bearing_evidence(requested_bearing_deg):
    """Build one exact integer-encoder bearing for a synthetic scan ray."""

    opposed = round(abs(requested_bearing_deg) / 0.490)
    if requested_bearing_deg < 0:
        delta = {"left_drive": -opposed, "right_drive": opposed}
    elif requested_bearing_deg > 0:
        delta = {"left_drive": opposed, "right_drive": -opposed}
    else:
        delta = {"left_drive": 0, "right_drive": 0}
    bearing = encoder_relative_bearing_deg(
        {"motor_angles_deg": delta},
        {"left_drive": 0, "right_drive": 0},
    )
    return delta, bearing


def dense_scan_view(pose, ranges, relative, restoration_error=0.0):
    start = pose.heading_mdeg / 1_000.0
    encoder_evidence = tuple(
        encoder_scan_bearing_evidence(heading) for heading in relative
    )
    angular = [
        {
            "side": side,
            "distance_mm": distance,
            "range_state": blast_range_state(distance),
            "body_motor_angle_deg": 158,
            "heading_deg": evidence[1],
            "relative_heading_deg": evidence[1],
            "imu_heading_deg": start + heading,
            "drive_encoder_delta_deg": evidence[0],
            "observation_settled": True,
        }
        for side, distance, heading, evidence in zip(
            SCAN_ANGULAR_RAY_SIDES, ranges, relative, encoder_evidence,
        )
    ]
    scan = {
        "schema": "blast-scan-front-arc/v3",
        "state": "complete",
        "result": "restored",
        "bearing_source": "DRIVE_ENCODER_ODOMETRY",
        "bearing_frame": "ROBOT_RELATIVE_AT_SCAN_START",
        "start_heading_deg": 0.0,
        "final_heading_deg": 0.0,
        "restoration_error_deg": 0.0,
        "restoration_verified": True,
        "encoder_start_angles_deg": {
            "left_drive": 0, "right_drive": 0,
        },
        "encoder_final_angles_deg": {
            "left_drive": 0, "right_drive": 0,
        },
        "encoder_restoration": {
            "common_mode_residue_mm": 0.0,
            "opposed_residue_deg": 0.0,
            "motion_stopped": True,
            "observation_settled": True,
            "body_pose_verified": True,
        },
        "imu_heading_diagnostics": {
            "authority": "DIAGNOSTIC_ONLY",
            "start_heading_deg": start,
            "final_heading_deg": start + restoration_error,
            "restoration_error_deg": restoration_error,
        },
        "all_observations_settled": True,
        "rays": [
            {**deepcopy(angular[index]), "side": side}
            for index, side in (
                (0, "center"),
                (2, "left_near"),
                (4, "left_far"),
                (6, "right_near"),
                (8, "right_far"),
            )
        ],
        "angular_rays": angular,
    }
    return {
        "scan_pose": pose.to_dict(),
        "scan": scan,
        "planar_projection": project_blast_scan_planar_surfaces(
            scan=scan, scan_pose=pose,
        ),
    }


def live_dense_right_views(side_pose):
    origin = dense_scan_view(
        PhysicalPose(),
        (265, 285, 310, 351, 379, 250, 250, 251, 1_228),
        (0.0, -9.98, -21.16, -32.58, -43.90,
         11.48, 22.21, 33.75, 44.56),
        0.434,
    )
    side = dense_scan_view(
        side_pose,
        (2_000, 487, 496, 485, 504,
         2_000, 2_000, 2_000, 2_000),
        (0.0, -12.24, -21.56, -33.28, -43.91,
         9.25, 20.36, 31.31, 41.82),
        -2.158,
    )
    return origin, side


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
    encoder_evidence = {
        side: encoder_scan_bearing_evidence(heading)
        for side, heading in headings.items()
    }
    value["scan"] = {
        "schema": "blast-scan-front-arc/v3",
        "state": "complete",
        "result": "restored",
        "bearing_source": "DRIVE_ENCODER_ODOMETRY",
        "bearing_frame": "ROBOT_RELATIVE_AT_SCAN_START",
        "start_heading_deg": 0.0,
        "final_heading_deg": 0.0,
        "restoration_error_deg": 0.0,
        "restoration_verified": True,
        "encoder_start_angles_deg": {
            "left_drive": 0, "right_drive": 0,
        },
        "encoder_final_angles_deg": {
            "left_drive": 0, "right_drive": 0,
        },
        "encoder_restoration": {
            "common_mode_residue_mm": 0.0,
            "opposed_residue_deg": 0.0,
            "motion_stopped": True,
            "observation_settled": True,
            "body_pose_verified": True,
        },
        "imu_heading_diagnostics": {
            "authority": "DIAGNOSTIC_ONLY",
            "start_heading_deg": 0.0,
            "final_heading_deg": 0.0,
            "restoration_error_deg": 0.0,
        },
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
                "heading_deg": encoder_evidence[side][1],
                "relative_heading_deg": encoder_evidence[side][1],
                "imu_heading_deg": headings[side],
                "drive_encoder_delta_deg": encoder_evidence[side][0],
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


def live_no_return_views(*, center_settled=True, sweep_distance=691):
    detour_origin = PhysicalPose(
        x_mm=91, y_mm=0, heading_mdeg=-245,
        verified_motion_count=2, total_forward_mm=91, total_turn_mdeg=735,
    )
    current = PhysicalPose(
        x_mm=74, y_mm=324, heading_mdeg=980,
        verified_motion_count=11,
        total_forward_mm=417,
        total_turn_mdeg=189_140,
    )
    origin = with_settled_scan(view(detour_origin, (
        ("center", 337, 79),
        ("left_near", 336, 190),
        ("right_near", 333, -19),
    )), (
        ("center", 136),
        ("left_near", 189),
        ("left_far", 2_000),
        ("right_near", 119),
        ("right_far", 2_000),
    ))
    side = with_settled_scan(view(current, (
        ("left_near", 1_410, 1_018),
    )), (
        ("center", 2_000),
        ("left_near", 1_393),
        ("left_far", 2_000),
        ("right_near", sweep_distance),
        ("right_far", 2_000),
    ))
    side["scan"]["all_observations_settled"] = False
    side["scan"]["rays"][0].update({
        "observation_settled": center_settled,
        "evidence_use": (
            "SETTLED_RANGE" if center_settled
            else "SWEEP_CONTINUATION_ONLY"
        ),
    })
    side["scan"]["rays"][3].update({
        "observation_settled": False,
        "evidence_use": "SWEEP_CONTINUATION_ONLY",
    })
    return detour_origin, current, origin, side


def no_return_waypoint(detour_origin):
    return {
        **waypoint("LEFT", detour_origin),
        "target_x_mm": 73,
        "target_y_mm": 337,
        "position_tolerance_mm": 35,
    }


class BlastDetourRouteTests(unittest.TestCase):
    def test_live_dense_right_scan_derives_284_mm_route_and_binds_at_it(self):
        side_pose = PhysicalPose(y_mm=-284, heading_mdeg=-735)
        origin, side = live_dense_right_views(side_pose)
        search = side_search_waypoint(
            PhysicalPose(), "RIGHT", scan_view=origin,
        )

        route = bind_blast_detour_route(
            origin_view=origin,
            side_view=side,
            selected_side="RIGHT",
            side_waypoint=search,
            mission=mission(),
            current_pose=side_pose,
        )

        self.assertEqual(search["target_lateral_offset_mm"], -284)
        self.assertEqual(
            search["search_basis"],
            "PROVISIONAL_SAME_DEPTH_ECHO_REACH",
        )
        self.assertEqual(route.target_radius_mm, 214)
        self.assertEqual(route.route_lateral_offset_mm, -284)
        self.assertTrue(blast_side_view_associates_frozen_target(
            side, route, "RIGHT",
        ))

    def test_live_dense_right_scan_rejects_caught_minus_228_pose(self):
        caught = PhysicalPose(x_mm=-18, y_mm=-228, heading_mdeg=-735)
        origin, side = live_dense_right_views(caught)
        search = side_search_waypoint(
            PhysicalPose(), "RIGHT", scan_view=origin,
        )

        with self.assertRaisesRegex(
            ValueError, "side pose does not match",
        ):
            bind_blast_detour_route(
                origin_view=origin,
                side_view=side,
                selected_side="RIGHT",
                side_waypoint=search,
                mission=mission(),
                current_pose=caught,
            )

    def test_dense_final_view_does_not_generalize_no_return_to_free(self):
        side_pose = PhysicalPose(y_mm=-284, heading_mdeg=-735)
        origin, side = live_dense_right_views(side_pose)
        search = side_search_waypoint(
            PhysicalPose(), "RIGHT", scan_view=origin,
        )
        close_outward = deepcopy(side)
        close_outward["scan"]["angular_rays"][5].update({
            "distance_mm": 50,
            "range_state": "MEASURED",
        })
        close_outward["planar_projection"] = (
            project_blast_scan_planar_surfaces(
                scan=close_outward["scan"], scan_pose=side_pose,
            )
        )
        unassociated = deepcopy(side)
        for point in unassociated["planar_projection"]["points"][:3]:
            point["nominal_echo_x_mm"] += 1_000
            point["nominal_echo_y_mm"] += 1_000

        for failed in (close_outward, unassociated):
            with self.subTest(failed=failed):
                provisional_route = bind_blast_detour_route(
                    origin_view=origin,
                    side_view=side,
                    selected_side="RIGHT",
                    side_waypoint=search,
                    mission=mission(),
                    current_pose=side_pose,
                )
                self.assertFalse(blast_side_view_associates_frozen_target(
                    failed, provisional_route, "RIGHT",
                ))
                with self.assertRaises(ValueError):
                    bind_blast_detour_route(
                        origin_view=origin,
                        side_view=failed,
                        selected_side="RIGHT",
                        side_waypoint=search,
                        mission=mission(),
                        current_pose=side_pose,
                    )

    def test_live_settled_no_return_side_view_binds_frozen_route(self):
        detour_origin, current, origin, side = live_no_return_views()

        route = bind_blast_detour_route(
            origin_view=origin,
            side_view=side,
            selected_side="LEFT",
            side_waypoint=no_return_waypoint(detour_origin),
            mission=mission(),
            current_pose=current,
        )

        self.assertEqual(route.created_pose, detour_origin)
        self.assertEqual(route.route_lateral_offset_mm, 335)
        self.assertEqual(route.pass_longitudinal_offset_mm, 646)
        merge_route = replace(route, version=route.version + 3, active_index=3)
        pass_pose = PhysicalPose(
            x_mm=route.pass_longitudinal_offset_mm + 90,
            y_mm=route.route_lateral_offset_mm,
        )
        pass_view = {**side, "scan_pose": pass_pose.to_dict()}
        self.assertTrue(blast_detour_scan_allows_progress(
            pass_view,
            role="PASS",
            selected_side="LEFT",
            minimum_clearance_mm=120,
            route=merge_route,
        ))
        self.assertFalse(blast_detour_scan_allows_progress(
            pass_view,
            role="FINAL",
            selected_side="LEFT",
            minimum_clearance_mm=120,
            route=merge_route,
        ))

    def test_no_return_side_view_rejects_unsettled_center_or_close_window(self):
        cases = (
            live_no_return_views(center_settled=False),
            live_no_return_views(sweep_distance=53),
        )
        detour_origin, _current, origin, side = live_no_return_views()
        spoofed_pose = PhysicalPose(x_mm=700, y_mm=337)
        cases += ((detour_origin, spoofed_pose, origin, {
            **side, "scan_pose": spoofed_pose.to_dict(),
        }),)
        detour_origin, current, origin, malformed = live_no_return_views()
        malformed["planar_projection"]["points"][0]["side"] = None
        cases += ((detour_origin, current, origin, malformed),)
        for distance, state in ((53, "MEASURED"), (None, "INVALID")):
            detour_origin, current, origin, blocked = live_no_return_views()
            blocked["scan"]["rays"][1].update({
                "distance_mm": distance,
                "range_state": state,
            })
            cases += ((detour_origin, current, origin, blocked),)
        for detour_origin, current, origin, failed_side in cases:
            with self.subTest(failed_side=failed_side):
                with self.assertRaises(ValueError):
                    bind_blast_detour_route(
                        origin_view=origin,
                        side_view=failed_side,
                        selected_side="LEFT",
                        side_waypoint=no_return_waypoint(detour_origin),
                        mission=mission(),
                        current_pose=current,
                    )

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

    def test_dense_mixed_far_view_rejects_close_intermediate_ray(self):
        side_pose = PhysicalPose(y_mm=-284, heading_mdeg=-735)
        origin, side = live_dense_right_views(side_pose)
        search = side_search_waypoint(
            PhysicalPose(), "RIGHT", scan_view=origin,
        )
        route = bind_blast_detour_route(
            origin_view=origin,
            side_view=side,
            selected_side="RIGHT",
            side_waypoint=search,
            mission=mission(),
            current_pose=side_pose,
        )
        pass_view = dense_scan_view(
            PhysicalPose(x_mm=877, y_mm=-284),
            (1_000, 50, 1_000, 1_000, 2_000,
             1_000, 1_000, 1_000, 1_000),
            (0.0, -10.0, -21.0, -33.0, -44.0,
             10.0, 21.0, 33.0, 44.0),
        )

        self.assertFalse(blast_detour_scan_allows_progress(
            pass_view,
            role="PASS",
            selected_side="RIGHT",
            minimum_clearance_mm=120,
            route=route,
        ))

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
