import unittest

from robot_agent.active_ir_scan_contract import ActiveIrScanCalibration
from robot_agent.blast_navigation_action_profile import (
    BLAST_NAVIGATION_ACTION_SPECS,
)
from robot_agent.blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
from robot_agent.local_detour_controller import (
    GUIDANCE_ADVANCE_FOR_TURN_ROOM,
    GUIDANCE_ADVANCE_TO_WAYPOINT,
    GUIDANCE_REVERSE_FOR_TURN_ROOM,
    GUIDANCE_ROUTE_COMPLETE,
    GUIDANCE_ROUTE_INVALID,
    GUIDANCE_TURN_TO_WAYPOINT,
    SYNC_ADVANCED,
    SYNC_BUILT,
    SYNC_INVALIDATED,
    SYNC_REBUILT,
    SYNC_TARGET_MISSING,
    LocalDetourControllerError,
    derive_local_detour_guidance,
    filter_local_detour_actions,
    local_detour_tail_action_allowed,
    synchronize_local_detour_route,
)
from robot_agent.local_detour_route import (
    MERGE_GOAL_AXIS,
    PASS_BEYOND_TARGET,
    ROUTE_COMPLETE,
    ROUTE_INVALID,
)
from robot_agent.maneuver_commitment import ActiveManeuver
from robot_agent.physical_action_feasibility import (
    navigation_action_feasibility,
)
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    EXPECTED_ACTION_SPECS,
    OBSERVE,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from robot_agent.physical_navigation_mission import DirectionalMission
from robot_agent.physical_odometry import PhysicalPose
from robot_agent.physical_scan_evidence import AngularCollisionSupport
from robot_agent.provisional_hazard_map import (
    HazardMapCalibration,
    ProvisionalHazard,
    ProvisionalHazardMap,
)


TARGET_ID = "box-a"
ALL_ACTIONS = (
    ADVANCE,
    OBSERVE,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)


def footprint():
    return RobotFootprint(
        front_extent_mm=70,
        rear_extent_mm=60,
        left_extent_mm=80,
        right_extent_mm=100,
        clearance_margin_mm=20,
        calibration_status="test",
        calibration_evidence="controller fixture",
    )


def hazard(
    *,
    hypothesis_id=TARGET_ID,
    centroid_x_mm=200,
    centroid_y_mm=0,
    radius_mm=50,
    collision_supports=(),
):
    return ProvisionalHazard(
        hypothesis_id=hypothesis_id,
        frame_id="frame-a",
        anchor_x_mm=0,
        anchor_y_mm=0,
        anchor_heading_mdeg=0,
        centroid_x_mm=centroid_x_mm,
        centroid_y_mm=centroid_y_mm,
        radius_mm=radius_mm,
        first_seen_at_ms=1,
        last_seen_at_ms=1,
        evidence_count=1,
        last_state_version=1,
        last_raw_ir_proximity=30,
        last_filtered_ir_proximity=30,
        collision_supports=collision_supports,
    )


def hazard_map(
    *,
    values=None,
    revision=7,
    frame_id="frame-a",
    generation_id="generation-a",
    calibrated=True,
):
    if values is None:
        values = (hazard(),)
    adjusted = tuple(
        value
        if value.frame_id == frame_id
        else ProvisionalHazard(
            **{
                **value.__dict__,
                "frame_id": frame_id,
            }
        )
        for value in values
    )
    return ProvisionalHazardMap(
        frame_id=frame_id,
        map_generation_id=generation_id,
        hazards=adjusted,
        revision=revision,
        calibration=HazardMapCalibration(
            robot_footprint=footprint() if calibrated else None
        ),
    )


def mission(*, pose=PhysicalPose()):
    return DirectionalMission.begin(
        episode_id="episode-a",
        minimum_forward_progress_mm=600,
        pose=pose,
        heading_tolerance_mdeg=20_000,
    )


def maneuver(*, side="LEFT_OF_GOAL", target_id=TARGET_ID, revision=1):
    return ActiveManeuver(
        commitment_id="maneuver-a",
        revision=revision,
        objective="Pass the remembered box",
        target_hypothesis_id=target_id,
        detour_side=side,
        success_fact_keys=(
            "GOAL_CORRIDOR_CLEAR",
            "GOAL_HEADING_ALIGNED",
            "TARGET_ENVELOPE_BEHIND_GOAL_ORIGIN",
        ),
        current_focus_fact_key="GOAL_CORRIDOR_CLEAR",
        started_turn=2,
        last_confirmed_turn=2,
    )


def feasibility(**changes):
    values = {
        action: {"allowed": True}
        for action in (ADVANCE, REVERSE, TURN_LEFT_90, TURN_RIGHT_90)
    }
    for action, allowed in changes.items():
        values[action]["allowed"] = allowed
    return values


def built_route(*, side="LEFT_OF_GOAL", pose=PhysicalPose()):
    result = synchronize_local_detour_route(
        None,
        active_maneuver=maneuver(side=side),
        current_pose=pose,
        mission=mission(),
        hazard_map=hazard_map(),
    )
    if result.route is None:
        raise AssertionError("route fixture did not build")
    return result.route


class ActiveCollisionGroupTests(unittest.TestCase):
    def test_live_white_box_hypotheses_form_one_derived_group(self):
        value = hazard_map(values=(
            hazard(
                hypothesis_id="h2",
                centroid_x_mm=256,
                centroid_y_mm=-4,
                radius_mm=70,
            ),
            hazard(
                hypothesis_id="h1",
                centroid_x_mm=377,
                centroid_y_mm=-12,
                radius_mm=70,
            ),
            hazard(
                hypothesis_id="separate",
                centroid_x_mm=800,
                centroid_y_mm=0,
                radius_mm=70,
            ),
        ))

        group = value.active_collision_group("h2")

        self.assertEqual(
            tuple(item.hypothesis_id for item in group),
            ("h1", "h2"),
        )
        self.assertEqual(value.hazard_ids, ("h1", "h2", "separate"))

    def test_collision_group_includes_a_transitive_envelope_chain(self):
        value = hazard_map(values=(
            hazard(hypothesis_id="a", centroid_x_mm=0, radius_mm=70),
            hazard(hypothesis_id="b", centroid_x_mm=130, radius_mm=70),
            hazard(hypothesis_id="c", centroid_x_mm=260, radius_mm=70),
        ))

        self.assertEqual(
            tuple(
                item.hypothesis_id
                for item in value.active_collision_group("a")
            ),
            ("a", "b", "c"),
        )


class LocalDetourRouteSynchronizationTests(unittest.TestCase):
    def test_model_side_builds_and_persists_the_route(self):
        first = synchronize_local_detour_route(
            None,
            active_maneuver=maneuver(side="RIGHT_OF_GOAL"),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(),
        )

        self.assertEqual(first.event, SYNC_BUILT)
        self.assertEqual(first.route.detour_side, "RIGHT_OF_GOAL")
        same = synchronize_local_detour_route(
            first.route,
            active_maneuver=maneuver(side="RIGHT_OF_GOAL"),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(revision=8),
        )
        self.assertIs(same.route, first.route)

    def test_model_revision_to_other_side_rebuilds_from_current_pose(self):
        first = built_route(side="LEFT_OF_GOAL")
        pose = PhysicalPose(x_mm=0, y_mm=70, heading_mdeg=90_000)

        result = synchronize_local_detour_route(
            first,
            active_maneuver=maneuver(
                side="RIGHT_OF_GOAL",
                revision=2,
            ),
            current_pose=pose,
            mission=mission(),
            hazard_map=hazard_map(revision=8),
        )

        self.assertEqual(result.event, SYNC_REBUILT)
        self.assertEqual(result.reason, "MODEL_ROUTE_CHANGED")
        self.assertEqual(result.route.detour_side, "RIGHT_OF_GOAL")
        self.assertEqual(result.route.created_pose, pose)
        self.assertNotEqual(result.route.route_id, first.route_id)

    def test_changed_target_geometry_replans_from_verified_pose(self):
        first = built_route()
        pose = PhysicalPose(x_mm=30, y_mm=40, heading_mdeg=30_000)

        result = synchronize_local_detour_route(
            first,
            active_maneuver=maneuver(),
            current_pose=pose,
            mission=mission(),
            hazard_map=hazard_map(
                values=(hazard(centroid_x_mm=230),),
                revision=8,
            ),
        )

        self.assertEqual(result.event, SYNC_REBUILT)
        self.assertEqual(result.reason, "TARGET_GEOMETRY_MISMATCH")
        self.assertEqual(result.route.target_centroid_x_mm, 230)
        self.assertEqual(result.route.created_pose, pose)

    def test_new_scan_support_rebuilds_the_complete_object_envelope(self):
        first = built_route()
        expanded = hazard(collision_supports=(
            AngularCollisionSupport(
                source_scan_id="scan-a",
                completed_at_ms=2,
                pose_x_mm=0,
                pose_y_mm=0,
                pose_heading_mdeg=0,
                actual_relative_bearing_mdeg=90_000,
                based_on_map_version=7,
            ),
        ))

        result = synchronize_local_detour_route(
            first,
            active_maneuver=maneuver(),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(values=(expanded,), revision=8),
        )

        self.assertEqual(result.event, SYNC_REBUILT)
        self.assertEqual(result.reason, "TARGET_GEOMETRY_MISMATCH")
        self.assertIn((0, 140), result.route.target_support_points)
        self.assertGreater(
            result.route.route_lateral_offset_mm,
            first.route_lateral_offset_mm,
        )

    def test_route_clears_the_complete_connected_hazard_group(self):
        live_hazards = (
            hazard(
                hypothesis_id="h2",
                centroid_x_mm=256,
                centroid_y_mm=-4,
                radius_mm=70,
            ),
            hazard(
                hypothesis_id="h1",
                centroid_x_mm=377,
                centroid_y_mm=-12,
                radius_mm=70,
            ),
        )

        result = synchronize_local_detour_route(
            None,
            active_maneuver=maneuver(
                side="LEFT_OF_GOAL",
                target_id="h2",
            ),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(values=live_hazards),
        )

        self.assertEqual(result.event, SYNC_BUILT)
        self.assertEqual(result.route.target_hypothesis_id, "h2")
        self.assertEqual(result.route.detour_side, "LEFT_OF_GOAL")
        self.assertEqual(
            set(result.route.target_support_points),
            {(256, -4), (377, -12)},
        )
        self.assertGreater(result.route.pass_longitudinal_offset_mm, 377)

    def test_connected_new_hazard_rebuilds_but_disjoint_hazard_does_not(self):
        target = hazard(
            hypothesis_id="h2",
            centroid_x_mm=256,
            centroid_y_mm=-4,
            radius_mm=70,
        )
        first = synchronize_local_detour_route(
            None,
            active_maneuver=maneuver(target_id="h2"),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(values=(target,)),
        ).route
        disjoint = synchronize_local_detour_route(
            first,
            active_maneuver=maneuver(target_id="h2"),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(
                values=(
                    target,
                    hazard(
                        hypothesis_id="far",
                        centroid_x_mm=800,
                        centroid_y_mm=0,
                        radius_mm=70,
                    ),
                ),
                revision=8,
            ),
        )
        connected = synchronize_local_detour_route(
            disjoint.route,
            active_maneuver=maneuver(target_id="h2"),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(
                values=(
                    target,
                    hazard(
                        hypothesis_id="h1",
                        centroid_x_mm=377,
                        centroid_y_mm=-12,
                        radius_mm=70,
                    ),
                ),
                revision=9,
            ),
        )

        self.assertIs(disjoint.route, first)
        self.assertEqual(connected.event, SYNC_REBUILT)
        self.assertEqual(connected.reason, "TARGET_GEOMETRY_MISMATCH")
        self.assertGreater(
            connected.route.pass_longitudinal_offset_mm,
            first.pass_longitudinal_offset_mm,
        )

    def test_route_does_not_merge_until_the_complete_group_is_passed(self):
        starting_pose = PhysicalPose(y_mm=300)
        target = hazard(
            hypothesis_id="h2",
            centroid_x_mm=256,
            centroid_y_mm=-4,
            radius_mm=70,
        )
        group_route = synchronize_local_detour_route(
            None,
            active_maneuver=maneuver(target_id="h2"),
            current_pose=starting_pose,
            mission=mission(),
            hazard_map=hazard_map(values=(
                target,
                hazard(
                    hypothesis_id="h1",
                    centroid_x_mm=377,
                    centroid_y_mm=-12,
                    radius_mm=70,
                ),
            )),
        ).route
        target_only_route = synchronize_local_detour_route(
            None,
            active_maneuver=maneuver(target_id="h2"),
            current_pose=starting_pose,
            mission=mission(),
            hazard_map=hazard_map(values=(target,)),
        ).route

        at_old_pass_point = PhysicalPose(
            x_mm=target_only_route.pass_longitudinal_offset_mm,
            y_mm=group_route.route_lateral_offset_mm,
        )
        at_group_pass_point = PhysicalPose(
            x_mm=group_route.pass_longitudinal_offset_mm,
            y_mm=group_route.route_lateral_offset_mm,
        )

        self.assertEqual(
            group_route.advance_reached(
                at_old_pass_point
            ).active_waypoint.kind,
            PASS_BEYOND_TARGET,
        )
        self.assertEqual(
            group_route.advance_reached(
                at_group_pass_point
            ).active_waypoint.kind,
            MERGE_GOAL_AXIS,
        )

    def test_new_map_generation_invalidates_instead_of_mixing_frames(self):
        first = built_route()

        result = synchronize_local_detour_route(
            first,
            active_maneuver=maneuver(),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(generation_id="generation-b"),
        )

        self.assertEqual(result.event, SYNC_INVALIDATED)
        self.assertEqual(result.route.status, ROUTE_INVALID)
        self.assertEqual(result.reason, "MAP_GENERATION_MISMATCH")

    def test_missing_active_target_never_invents_a_route(self):
        result = synchronize_local_detour_route(
            None,
            active_maneuver=maneuver(),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(values=()),
        )

        self.assertEqual(result.event, SYNC_TARGET_MISSING)
        self.assertIsNone(result.route)

    def test_contested_target_can_rebuild_when_collision_support_returns(self):
        first = built_route()
        contested = ProvisionalHazard(
            **{
                **hazard().__dict__,
                "collision_contested_at_ms": 2,
            }
        )
        invalid = synchronize_local_detour_route(
            first,
            active_maneuver=maneuver(),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(values=(contested,), revision=8),
        )
        self.assertEqual(invalid.event, SYNC_INVALIDATED)
        self.assertEqual(invalid.reason, "TARGET_MISSING")

        recovered = synchronize_local_detour_route(
            invalid.route,
            active_maneuver=maneuver(),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=hazard_map(revision=9),
        )
        self.assertEqual(recovered.event, SYNC_REBUILT)
        self.assertEqual(recovered.reason, "TARGET_ACTIVE_AGAIN")
        self.assertNotEqual(recovered.route.status, ROUTE_INVALID)

    def test_route_requires_explicit_footprint_calibration(self):
        with self.assertRaises(LocalDetourControllerError):
            synchronize_local_detour_route(
                None,
                active_maneuver=maneuver(),
                current_pose=PhysicalPose(),
                mission=mission(),
                hazard_map=hazard_map(calibrated=False),
            )


class LocalDetourGuidanceTests(unittest.TestCase):
    def test_shared_route_guidance_accepts_blast_motion_profile(self):
        cases = (
            (
                "LEFT_OF_GOAL",
                (
                    ("LATERAL_CLEARANCE", TURN_LEFT_90, 205, 90_000),
                    ("REACQUIRE_GOAL_HEADING", TURN_RIGHT_90, 0, -90_000),
                    ("PASS_BEYOND_TARGET", ADVANCE, 428, 0),
                    ("MERGE_GOAL_AXIS", TURN_RIGHT_90, 205, -90_000),
                    ("RESUME_GOAL_HEADING", TURN_LEFT_90, 0, 90_000),
                ),
            ),
            (
                "RIGHT_OF_GOAL",
                (
                    ("LATERAL_CLEARANCE", TURN_RIGHT_90, 185, -90_000),
                    ("REACQUIRE_GOAL_HEADING", TURN_LEFT_90, 0, 90_000),
                    ("PASS_BEYOND_TARGET", ADVANCE, 428, 0),
                    ("MERGE_GOAL_AXIS", TURN_LEFT_90, 185, 90_000),
                    ("RESUME_GOAL_HEADING", TURN_RIGHT_90, 0, -90_000),
                ),
            ),
        )
        calibration = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
        for side, expected_steps in cases:
            with self.subTest(side=side):
                route = built_route(side=side)
                pose = PhysicalPose()
                for index, expected in enumerate(expected_steps):
                    kind, action, distance, heading_error = expected
                    guidance = derive_local_detour_guidance(
                        route,
                        current_pose=pose,
                        motion_feasibility=feasibility(),
                        action_specs=BLAST_NAVIGATION_ACTION_SPECS,
                        odometry_calibration=calibration,
                    )

                    self.assertIs(guidance.route, route)
                    self.assertEqual(guidance.active_waypoint_kind, kind)
                    self.assertEqual(
                        guidance.allowed_motion_actions,
                        frozenset((action,)),
                    )
                    self.assertEqual(
                        guidance.reason,
                        GUIDANCE_ADVANCE_TO_WAYPOINT
                        if action == ADVANCE
                        else GUIDANCE_TURN_TO_WAYPOINT,
                    )
                    self.assertEqual(
                        guidance.distance_to_waypoint_mm,
                        distance,
                    )
                    self.assertEqual(
                        guidance.heading_error_mdeg,
                        heading_error,
                    )

                    waypoint = route.active_waypoint
                    previous = route
                    pose = PhysicalPose(
                        x_mm=waypoint.x_mm,
                        y_mm=waypoint.y_mm,
                        heading_mdeg=waypoint.heading_mdeg,
                    )
                    route = route.advance_reached(pose)
                    self.assertEqual(route.version, previous.version + 1)
                    self.assertEqual(route.active_index, index + 1)

                self.assertEqual(route.status, ROUTE_COMPLETE)

    def test_gate_activity_is_derived_but_remains_serialized(self):
        inactive = derive_local_detour_guidance(
            None,
            current_pose=PhysicalPose(),
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        )
        active = derive_local_detour_guidance(
            built_route(),
            current_pose=PhysicalPose(),
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        )

        self.assertNotIn("gate_active", vars(inactive))
        self.assertFalse(inactive.gate_active)
        self.assertFalse(inactive.to_dict()["gate_active"])
        self.assertNotIn("gate_active", vars(active))
        self.assertTrue(active.gate_active)
        self.assertTrue(active.to_dict()["gate_active"])

    def test_ev3_route_reverses_until_close_target_allows_turn(self):
        profile = EV3RSTORMProfile()
        live_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="generation-a",
            hazards=(hazard(centroid_x_mm=140, radius_mm=70),),
            revision=1,
            calibration=profile.hazard_calibration,
        )
        route = synchronize_local_detour_route(
            None,
            active_maneuver=maneuver(),
            current_pose=PhysicalPose(),
            mission=mission(),
            hazard_map=live_map,
        ).route
        self.assertEqual(route.inflated_lateral_clearance_mm, 275)
        motion = navigation_action_feasibility(
            hazard_map=live_map,
            pose=PhysicalPose(),
            action_specs=EXPECTED_ACTION_SPECS,
            odometry_calibration=profile.odometry_calibration,
            active_scan_calibration=ActiveIrScanCalibration(),
        )["motion_actions"]

        self.assertFalse(motion[ADVANCE]["allowed"])
        self.assertTrue(motion[REVERSE]["allowed"])
        self.assertFalse(motion[TURN_LEFT_90]["allowed"])
        self.assertFalse(motion[TURN_RIGHT_90]["allowed"])
        guidance = derive_local_detour_guidance(
            route,
            current_pose=PhysicalPose(),
            motion_feasibility=motion,
            action_specs=EXPECTED_ACTION_SPECS,
            odometry_calibration=profile.odometry_calibration,
        )
        self.assertEqual(
            guidance.reason,
            GUIDANCE_REVERSE_FOR_TURN_ROOM,
        )
        self.assertEqual(
            guidance.allowed_motion_actions,
            frozenset((REVERSE,)),
        )

        staged_pose = PhysicalPose(x_mm=-200)
        staged_motion = navigation_action_feasibility(
            hazard_map=live_map,
            pose=staged_pose,
            action_specs=EXPECTED_ACTION_SPECS,
            odometry_calibration=profile.odometry_calibration,
            active_scan_calibration=ActiveIrScanCalibration(),
        )["motion_actions"]
        staged = derive_local_detour_guidance(
            route,
            current_pose=staged_pose,
            motion_feasibility=staged_motion,
            action_specs=EXPECTED_ACTION_SPECS,
            odometry_calibration=profile.odometry_calibration,
        )
        self.assertEqual(staged.reason, GUIDANCE_TURN_TO_WAYPOINT)
        self.assertEqual(
            staged.allowed_motion_actions,
            frozenset((TURN_LEFT_90,)),
        )

    def test_turns_to_first_waypoint_before_admitting_advance(self):
        route = built_route()

        guidance = derive_local_detour_guidance(
            route,
            current_pose=PhysicalPose(),
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        )

        self.assertEqual(guidance.reason, GUIDANCE_TURN_TO_WAYPOINT)
        self.assertEqual(
            guidance.allowed_motion_actions,
            frozenset((TURN_LEFT_90,)),
        )
        filtered = filter_local_detour_actions(ALL_ACTIONS, guidance)
        self.assertIn(OBSERVE, filtered)
        self.assertIn(TURN_LEFT_90, filtered)
        self.assertNotIn(ADVANCE, filtered)
        self.assertNotIn(REVERSE, filtered)
        self.assertNotIn(TURN_RIGHT_90, filtered)

    def test_aligned_heading_advances_only_toward_active_waypoint(self):
        route = built_route()
        pose = PhysicalPose(heading_mdeg=90_000)

        guidance = derive_local_detour_guidance(
            route,
            current_pose=pose,
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        )

        self.assertEqual(guidance.reason, GUIDANCE_ADVANCE_TO_WAYPOINT)
        self.assertEqual(
            guidance.allowed_motion_actions,
            frozenset((ADVANCE,)),
        )

    def test_lateral_staging_can_cross_clearance_after_long_backoff(self):
        cases = (
            ("LEFT_OF_GOAL", 190, 90_000),
            ("RIGHT_OF_GOAL", -170, -90_000),
        )
        for side, y_mm, heading_mdeg in cases:
            with self.subTest(side=side):
                route = built_route(side=side)
                pose = PhysicalPose(
                    x_mm=-210,
                    y_mm=y_mm,
                    heading_mdeg=heading_mdeg,
                )

                guidance = derive_local_detour_guidance(
                    route,
                    current_pose=pose,
                    motion_feasibility=feasibility(),
                    action_specs=EXPECTED_ACTION_SPECS,
                )

                self.assertEqual(
                    guidance.reason,
                    GUIDANCE_ADVANCE_TO_WAYPOINT,
                )
                self.assertEqual(
                    guidance.allowed_motion_actions,
                    frozenset((ADVANCE,)),
                )

    def test_heading_drift_can_advance_only_when_it_closes_distance(self):
        route = built_route()
        pose = PhysicalPose(heading_mdeg=60_000)

        guidance = derive_local_detour_guidance(
            route,
            current_pose=pose,
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        )

        self.assertEqual(
            guidance.reason,
            GUIDANCE_ADVANCE_FOR_TURN_ROOM,
        )
        self.assertEqual(
            guidance.allowed_motion_actions,
            frozenset((ADVANCE,)),
        )

    def test_reached_waypoint_cancels_stale_advance_tail(self):
        route = built_route()
        reached = PhysicalPose(
            x_mm=0,
            y_mm=route.route_lateral_offset_mm,
            heading_mdeg=90_000,
        )
        sync = synchronize_local_detour_route(
            route,
            active_maneuver=maneuver(),
            current_pose=reached,
            mission=mission(),
            hazard_map=hazard_map(revision=8),
        )

        self.assertEqual(sync.event, SYNC_ADVANCED)
        self.assertFalse(local_detour_tail_action_allowed(
            ADVANCE,
            route=sync.route,
            current_pose=reached,
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        ))
        self.assertTrue(local_detour_tail_action_allowed(
            TURN_RIGHT_90,
            route=sync.route,
            current_pose=reached,
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        ))

    def test_reached_heading_cancels_stale_turn_tail(self):
        route = built_route()
        pose = PhysicalPose(heading_mdeg=90_000)

        self.assertFalse(local_detour_tail_action_allowed(
            TURN_LEFT_90,
            route=route,
            current_pose=pose,
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        ))
        self.assertTrue(local_detour_tail_action_allowed(
            ADVANCE,
            route=route,
            current_pose=pose,
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        ))

    def test_completed_route_releases_the_local_gate(self):
        route = built_route()
        pose = PhysicalPose()
        for waypoint in route.waypoints:
            pose = PhysicalPose(
                x_mm=waypoint.x_mm,
                y_mm=waypoint.y_mm,
                heading_mdeg=waypoint.heading_mdeg,
            )
            route = route.advance_reached(pose)
        guidance = derive_local_detour_guidance(
            route,
            current_pose=pose,
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        )

        self.assertEqual(guidance.reason, GUIDANCE_ROUTE_COMPLETE)
        self.assertEqual(guidance.route.status, ROUTE_COMPLETE)
        self.assertIsNone(guidance.allowed_motion_actions)
        self.assertEqual(
            filter_local_detour_actions(ALL_ACTIONS, guidance),
            ALL_ACTIONS,
        )
        self.assertFalse(local_detour_tail_action_allowed(
            ADVANCE,
            route=route,
            current_pose=pose,
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        ))

    def test_invalid_route_blocks_motion_but_keeps_nonmotion_tools(self):
        route = built_route().invalidate("TARGET_MISSING")
        guidance = derive_local_detour_guidance(
            route,
            current_pose=PhysicalPose(),
            motion_feasibility=feasibility(),
            action_specs=EXPECTED_ACTION_SPECS,
        )

        self.assertEqual(guidance.reason, GUIDANCE_ROUTE_INVALID)
        self.assertEqual(
            filter_local_detour_actions(ALL_ACTIONS, guidance),
            (OBSERVE,),
        )


if __name__ == "__main__":
    unittest.main()
