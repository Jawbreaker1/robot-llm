import copy
from dataclasses import replace
import unittest

from robot_agent.legacy_control_projection import (
    LegacyControlProjectionError,
    project_active_maneuver_intent,
    project_local_detour_execution_plan,
    project_navigation_plan_tail_execution_plan,
)
from robot_agent.local_detour_route import build_local_detour_route
from robot_agent.maneuver_commitment import (
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
)
from robot_agent.navigation_plan_tail import NavigationPlanTail
from robot_agent.physical_agent_state import (
    ControllerKey,
    DetourSide,
    DetourTargetIntent,
    GoalAssignment,
    NavigationBasis,
    PrimitiveStep,
    WaypointStep,
)
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from robot_agent.physical_odometry import PhysicalPose


def basis(**changes):
    values = {
        "controller_key": ControllerKey(
            robot_id="robot-a",
            controller_id="ev3-a",
            controller_instance_id="ev3-boot-a",
        ),
        "goal_epoch": 3,
        "controller_state_version": 11,
        "world_generation_id": "map-a",
        "world_model_version": 9,
        "navigation_basis_id": "basis-a",
        "frame_id": "frame-a",
        "calibration_fingerprint": "ev3-calibration-a",
    }
    values.update(changes)
    return NavigationBasis(**values)


def goal(**changes):
    values = {
        "goal_id": "goal-a",
        "goal_epoch": 3,
        "objective": "Move forward and navigate around obstacles.",
        "source": "legacy-shadow-test",
        "locale": "en",
        "activated_at_ms": 1_000,
    }
    values.update(changes)
    return GoalAssignment(**values)


def active_commitment(**changes):
    active = {
        "id": "legacy-commitment-a",
        "revision": 2,
        "objective": "Pass the remembered obstacle on the left.",
        "target_hypothesis_id": "hazard-a",
        "detour_side": "LEFT_OF_GOAL",
        "success_fact_keys": [
            FACT_GOAL_CORRIDOR_CLEAR,
            FACT_TARGET_BEHIND,
            FACT_GOAL_HEADING_ALIGNED,
        ],
        "current_focus_fact_key": FACT_TARGET_BEHIND,
        "started_turn": 4,
        "last_confirmed_turn": 7,
    }
    active.update(changes)
    return {"active": active, "last_terminal": None}


def intent(*, nav_basis=None, state=None, intent_id="intent-a"):
    nav_basis = nav_basis or basis()
    return project_active_maneuver_intent(
        state or active_commitment(),
        intent_id=intent_id,
        goal=goal(),
        basis=nav_basis,
        accepted_at_ms=1_100,
    )


def footprint():
    return RobotFootprint(
        front_extent_mm=70,
        rear_extent_mm=60,
        left_extent_mm=80,
        right_extent_mm=100,
        clearance_margin_mm=20,
        calibration_status="test",
        calibration_evidence="legacy projection fixture",
    )


def route(**changes):
    values = {
        "current_pose": PhysicalPose(),
        "goal_heading_mdeg": 0,
        "detour_side": "LEFT_OF_GOAL",
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
    values.update(changes)
    return build_local_detour_route(**values)


def tail(**changes):
    values = {
        "source_turn": 8,
        "source_plan": (REVERSE, TURN_LEFT_90, ADVANCE),
        "remaining_actions": (TURN_LEFT_90, ADVANCE),
        "created_monotonic": 10.0,
        "expires_monotonic": 20.0,
        "map_generation_id": "map-a",
        "last_map_version": 8,
        "last_observation_state_version": 10,
        "hazard_ids": ("hazard-a",),
        "safety_signature": (False, False, False),
        "active_commitment": active_commitment()["active"],
        "focus_truth": (FACT_TARGET_BEHIND, False),
        "scan_staging_target_ids": (),
    }
    values.update(changes)
    return NavigationPlanTail(**values)


class LegacyIntentProjectionTests(unittest.TestCase):
    def test_active_commitment_projects_with_only_injected_host_binding(self):
        legacy = active_commitment()
        original = copy.deepcopy(legacy)
        nav_basis = basis()

        value = project_active_maneuver_intent(
            legacy,
            intent_id="injected-intent-id",
            goal=goal(),
            basis=nav_basis,
            accepted_at_ms=9_999,
        )

        self.assertEqual(value.intent_id, "injected-intent-id")
        self.assertEqual(value.revision, 2)
        self.assertEqual(value.goal_id, "goal-a")
        self.assertEqual(value.goal_epoch, 3)
        self.assertEqual(value.accepted_basis, nav_basis)
        self.assertEqual(value.accepted_at_ms, 9_999)
        self.assertEqual(
            value.payload,
            DetourTargetIntent("hazard-a", DetourSide.LEFT_OF_GOAL),
        )
        self.assertEqual(legacy, original)

    def test_inactive_malformed_and_cross_epoch_state_has_typed_errors(self):
        cases = (
            (
                {"active": None, "last_terminal": None},
                goal(),
                basis(),
                "inactive_legacy_maneuver",
            ),
            (
                {"active": {"id": "incomplete"}, "last_terminal": None},
                goal(),
                basis(),
                "invalid_legacy_active_maneuver",
            ),
            (
                active_commitment(last_confirmed_turn=3),
                goal(),
                basis(),
                "invalid_legacy_active_maneuver",
            ),
            (
                active_commitment(),
                goal(goal_epoch=4),
                basis(),
                "goal_basis_mismatch",
            ),
        )
        for state, assigned_goal, nav_basis, error_code in cases:
            with self.subTest(error_code=error_code):
                with self.assertRaises(LegacyControlProjectionError) as caught:
                    project_active_maneuver_intent(
                        state,
                        intent_id="intent-a",
                        goal=assigned_goal,
                        basis=nav_basis,
                        accepted_at_ms=1_100,
                    )
                self.assertEqual(caught.exception.code, error_code)


class LegacyRouteProjectionTests(unittest.TestCase):
    def test_route_projects_waypoints_binding_and_existing_cursor(self):
        legacy = route().advance_reached(
            PhysicalPose(x_mm=0, y_mm=205, heading_mdeg=90_000)
        )
        ids = tuple(
            "route-step-{}".format(index)
            for index in range(len(legacy.waypoints))
        )
        nav_basis = basis()
        active_intent = intent(nav_basis=nav_basis)

        value = project_local_detour_execution_plan(
            legacy,
            plan_id="route-shadow-plan",
            plan_revision=4,
            step_ids=ids,
            goal=goal(),
            intent=active_intent,
            basis=nav_basis,
            created_at_ms=2_000,
        )

        self.assertEqual(value.plan_id, "route-shadow-plan")
        self.assertEqual(value.revision, 4)
        self.assertEqual(value.cursor, legacy.active_index)
        self.assertEqual(value.created_at_ms, 2_000)
        self.assertTrue(all(isinstance(item, WaypointStep) for item in value.steps))
        self.assertEqual(tuple(item.step_id for item in value.steps), ids)
        self.assertEqual(
            [
                (item.x_mm, item.y_mm, item.heading_mdeg)
                for item in value.steps
            ],
            [
                (item.x_mm, item.y_mm, item.heading_mdeg)
                for item in legacy.waypoints
            ],
        )
        self.assertEqual(
            value.binding.target_geometry_signatures,
            (("hazard-a", legacy.target_geometry_signature),),
        )
        self.assertEqual(
            value.binding.based_on_navigation_basis_id,
            nav_basis.navigation_basis_id,
        )

    def test_route_projection_rejects_invalid_or_mismatched_authority(self):
        nav_basis = basis()
        active_intent = intent(nav_basis=nav_basis)
        right_intent = replace(
            active_intent,
            payload=DetourTargetIntent(
                "hazard-a", DetourSide.RIGHT_OF_GOAL
            ),
        )
        values = (
            (
                route().invalidate("TARGET_MISSING"),
                active_intent,
                nav_basis,
                "invalidated_legacy_route",
            ),
            (
                route(),
                right_intent,
                nav_basis,
                "legacy_route_intent_mismatch",
            ),
            (
                route(frame_id="frame-b"),
                active_intent,
                nav_basis,
                "legacy_route_basis_mismatch",
            ),
        )
        for legacy, selected_intent, selected_basis, error_code in values:
            with self.subTest(error_code=error_code):
                with self.assertRaises(LegacyControlProjectionError) as caught:
                    project_local_detour_execution_plan(
                        legacy,
                        plan_id="route-plan",
                        plan_revision=1,
                        step_ids=tuple(
                            "step-{}".format(index)
                            for index in range(len(legacy.waypoints))
                        ),
                        goal=goal(),
                        intent=selected_intent,
                        basis=selected_basis,
                        created_at_ms=2_000,
                    )
                self.assertEqual(caught.exception.code, error_code)

        with self.assertRaises(LegacyControlProjectionError) as caught:
            project_local_detour_execution_plan(
                route(),
                plan_id="route-plan",
                plan_revision=1,
                step_ids=("one-step-only",),
                goal=goal(),
                intent=active_intent,
                basis=nav_basis,
                created_at_ms=2_000,
            )
        self.assertEqual(caught.exception.code, "invalid_step_ids")


class LegacyPlanTailProjectionTests(unittest.TestCase):
    def test_tail_projects_full_source_and_cursor_as_transitional_primitives(self):
        legacy = tail()
        nav_basis = basis()
        active_intent = intent(nav_basis=nav_basis)

        value = project_navigation_plan_tail_execution_plan(
            legacy,
            plan_id="tail-shadow-plan",
            plan_revision=5,
            step_ids=("primitive-0", "primitive-1", "primitive-2"),
            goal=goal(),
            intent=active_intent,
            basis=nav_basis,
            created_at_ms=2_100,
            now_monotonic=11.0,
        )

        self.assertEqual(value.cursor, 1)
        self.assertEqual(value.created_at_ms, 2_100)
        self.assertEqual(
            tuple(item.action for item in value.steps),
            legacy.source_plan,
        )
        self.assertTrue(all(isinstance(item, PrimitiveStep) for item in value.steps))
        self.assertEqual(value.active_step.action, TURN_LEFT_90)
        self.assertEqual(value.binding.target_geometry_signatures, ())

    def test_tail_projection_rejects_expired_cancelled_and_inconsistent_state(self):
        nav_basis = basis()
        active_intent = intent(nav_basis=nav_basis)
        cases = (
            (
                tail(),
                active_intent,
                nav_basis,
                20.0,
                "expired_legacy_plan_tail",
            ),
            (
                tail(cancelled_reason="legacy_veto"),
                active_intent,
                nav_basis,
                11.0,
                "cancelled_legacy_plan_tail",
            ),
            (
                tail(remaining_actions=(TURN_RIGHT_90,)),
                active_intent,
                nav_basis,
                11.0,
                "invalid_legacy_plan_tail",
            ),
            (
                tail(),
                replace(
                    active_intent,
                    payload=DetourTargetIntent(
                        "hazard-a", DetourSide.RIGHT_OF_GOAL
                    ),
                ),
                nav_basis,
                11.0,
                "legacy_plan_tail_intent_mismatch",
            ),
            (
                tail(),
                active_intent,
                replace(nav_basis, world_model_version=7),
                11.0,
                "legacy_plan_tail_basis_mismatch",
            ),
        )
        for legacy, selected_intent, selected_basis, now, error_code in cases:
            with self.subTest(error_code=error_code):
                with self.assertRaises(LegacyControlProjectionError) as caught:
                    project_navigation_plan_tail_execution_plan(
                        legacy,
                        plan_id="tail-plan",
                        plan_revision=1,
                        step_ids=("step-0", "step-1", "step-2"),
                        goal=goal(),
                        intent=selected_intent,
                        basis=selected_basis,
                        created_at_ms=2_100,
                        now_monotonic=now,
                    )
                self.assertEqual(caught.exception.code, error_code)


if __name__ == "__main__":
    unittest.main()
