import json
import unittest

from robot_agent.legacy_decision_shadow_projection import (
    EXACT_CONTRACT,
    LOSSY_COMPATIBILITY,
    NOT_EVALUATED,
    NOT_EXECUTION_EQUIVALENT,
    LegacyDecisionShadowProjectionError,
    project_validated_legacy_decision,
)
from robot_agent.maneuver_commitment import (
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
)
from robot_agent.physical_agent_state import (
    ControllerKey,
    DetourSide,
    DetourTargetIntent,
    FollowDirectionIntent,
    GoalAssignment,
    NavigationBasis,
    PrimitiveStep,
    ScanTargetIntent,
    SensorStep,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    FINISH,
    OBSERVE,
    REVERSE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    NavigationDecision,
)


def basis():
    return NavigationBasis(
        controller_key=ControllerKey(
            robot_id="robot-a",
            controller_id="ev3-a",
            controller_instance_id="ev3-boot-a",
        ),
        goal_epoch=3,
        controller_state_version=11,
        world_generation_id="map-a",
        world_model_version=9,
        navigation_basis_id="basis-a",
        frame_id="frame-a",
        calibration_fingerprint="ev3-calibration-a",
    )


def goal():
    return GoalAssignment(
        goal_id="goal-a",
        goal_epoch=3,
        objective="Move forward and navigate around obstacles.",
        source="legacy-shadow-test",
        locale="en",
        activated_at_ms=1_000,
    )


def inactive_state(last_terminal=None):
    return {"active": None, "last_terminal": last_terminal}


def active_state():
    return {
        "active": {
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
        },
        "last_terminal": None,
    }


def decision(
    action,
    plan,
    *,
    target_id=None,
    turn=8,
):
    return NavigationDecision(
        episode_id="episode-a",
        turn=turn,
        based_on_state_version=11,
        action=action,
        plan=tuple(plan),
        reason_code="VERIFY_RESULT",
        assessment="Host-validated legacy decision.",
        utterance=None,
        perception_target_hypothesis_id=target_id,
        maneuver_commitment={},
    )


def project(value, state, *, step_ids):
    return project_validated_legacy_decision(
        value,
        state,
        goal=goal(),
        basis=basis(),
        accepted_at_ms=12_345,
        intent_id="shadow-intent-a",
        plan_id="shadow-plan-a",
        plan_revision=4,
        step_ids=step_ids,
        scan_profile_id="front-arc-v1",
    )


class LegacyDecisionShadowProjectionTests(unittest.TestCase):
    def test_active_maneuver_projects_detour_even_for_observe(self):
        value = project(
            decision(OBSERVE, (OBSERVE,)),
            active_state(),
            step_ids=("observe-step-a",),
        )

        self.assertFalse(value.terminal)
        self.assertEqual(value.intent.intent_id, "shadow-intent-a")
        self.assertEqual(value.intent.revision, 2)
        self.assertEqual(value.intent.accepted_at_ms, 12_345)
        self.assertEqual(
            value.intent.payload,
            DetourTargetIntent("hazard-a", DetourSide.LEFT_OF_GOAL),
        )
        self.assertEqual(value.execution_plan.plan_id, "shadow-plan-a")
        self.assertEqual(value.execution_plan.revision, 4)
        self.assertEqual(value.execution_plan.created_at_ms, 12_345)
        self.assertEqual(
            value.execution_plan.steps,
            (SensorStep("observe-step-a", OBSERVE),),
        )
        self.assertEqual(value.classification.mapping_class, EXACT_CONTRACT)
        self.assertEqual(
            value.classification.execution_equivalence,
            NOT_EXECUTION_EQUIVALENT,
        )
        self.assertEqual(value.classification.offer_parity, NOT_EVALUATED)
        self.assertEqual(value.classification.receipt_parity, NOT_EVALUATED)

    def test_active_maneuver_projects_detour_even_for_scan(self):
        value = project(
            decision(
                SCAN_FRONT_ARC,
                (SCAN_FRONT_ARC,),
                target_id="hazard-a",
            ),
            active_state(),
            step_ids=("scan-step-a",),
        )

        self.assertIsInstance(value.intent.payload, DetourTargetIntent)
        self.assertEqual(
            value.execution_plan.steps,
            (
                SensorStep(
                    "scan-step-a",
                    SCAN_FRONT_ARC,
                    "hazard-a",
                    "front-arc-v1",
                ),
            ),
        )
        self.assertEqual(
            value.classification.intent_mapping_class,
            EXACT_CONTRACT,
        )
        self.assertEqual(
            value.classification.plan_mapping_class,
            EXACT_CONTRACT,
        )

    def test_inactive_scan_projects_scan_target_and_sensor_step(self):
        value = project(
            decision(
                SCAN_FRONT_ARC,
                (SCAN_FRONT_ARC,),
                target_id="hazard-b",
            ),
            inactive_state(),
            step_ids=("scan-step-b",),
        )

        self.assertEqual(
            value.intent.payload,
            ScanTargetIntent("hazard-b", "front-arc-v1"),
        )
        self.assertEqual(value.intent.revision, 1)
        self.assertIsInstance(value.execution_plan.steps[0], SensorStep)
        self.assertEqual(value.classification.mapping_class, EXACT_CONTRACT)

    def test_other_inactive_actions_project_follow_direction(self):
        observe = project(
            decision(OBSERVE, (OBSERVE,)),
            inactive_state(),
            step_ids=("observe-step-a",),
        )
        motion = project(
            decision(ADVANCE, (ADVANCE, TURN_LEFT_90, REVERSE)),
            inactive_state(),
            step_ids=("motion-a", "motion-b", "motion-c"),
        )

        self.assertIsInstance(observe.intent.payload, FollowDirectionIntent)
        self.assertIsInstance(motion.intent.payload, FollowDirectionIntent)
        self.assertEqual(
            observe.classification.intent_mapping_class,
            LOSSY_COMPATIBILITY,
        )
        self.assertEqual(
            tuple(step.action for step in motion.execution_plan.steps),
            (ADVANCE, TURN_LEFT_90, REVERSE),
        )
        self.assertTrue(
            all(
                isinstance(step, PrimitiveStep)
                for step in motion.execution_plan.steps
            )
        )
        self.assertEqual(
            motion.classification.plan_mapping_class,
            LOSSY_COMPATIBILITY,
        )
        self.assertEqual(
            motion.classification.mapping_class,
            LOSSY_COMPATIBILITY,
        )

    def test_mixed_sensor_and_motion_steps_are_projectable(self):
        # The v1 decoder currently rejects mixed plans.  Keeping the pure
        # projector generic avoids another migration if that contract widens.
        value = project(
            decision(
                ADVANCE,
                (ADVANCE, SCAN_FRONT_ARC, OBSERVE),
                target_id="hazard-a",
            ),
            active_state(),
            step_ids=("mixed-a", "mixed-b", "mixed-c"),
        )

        self.assertEqual(
            tuple(type(step) for step in value.execution_plan.steps),
            (PrimitiveStep, SensorStep, SensorStep),
        )
        self.assertEqual(
            value.execution_plan.steps[1],
            SensorStep(
                "mixed-b",
                SCAN_FRONT_ARC,
                "hazard-a",
                "front-arc-v1",
            ),
        )
        self.assertEqual(
            value.classification.plan_mapping_class,
            LOSSY_COMPATIBILITY,
        )

    def test_finish_is_terminal_and_creates_no_intent_or_plan(self):
        value = project(
            decision(FINISH, (FINISH,)),
            active_state(),
            step_ids=(),
        )

        self.assertTrue(value.terminal)
        self.assertIsNone(value.intent)
        self.assertIsNone(value.execution_plan)
        self.assertEqual(value.classification.mapping_class, EXACT_CONTRACT)

    def test_projection_is_stable_and_directly_json_serializable(self):
        source = decision(ADVANCE, (ADVANCE, REVERSE))
        first = project(
            source,
            inactive_state(),
            step_ids=("stable-a", "stable-b"),
        )
        second = project(
            source,
            inactive_state(),
            step_ids=("stable-a", "stable-b"),
        )

        self.assertEqual(first, second)
        payload = first.to_dict()
        self.assertEqual(json.loads(json.dumps(payload)), payload)
        self.assertEqual(
            [
                step["step_id"]
                for step in payload["execution_plan"]["steps"]
            ],
            ["stable-a", "stable-b"],
        )
        self.assertNotIn("receipt", payload)
        self.assertNotIn("reducer_state", payload)
        self.assertEqual(
            payload["classification"]["receipt_parity"],
            NOT_EVALUATED,
        )

    def test_invalid_or_unrepresentable_inputs_fail_with_typed_codes(self):
        cases = (
            (
                lambda: project(
                    decision(SCAN_FRONT_ARC, (SCAN_FRONT_ARC,)),
                    inactive_state(),
                    step_ids=("scan-a",),
                ),
                "missing_scan_target",
            ),
            (
                lambda: project(
                    decision(ADVANCE, (ADVANCE, REVERSE)),
                    inactive_state(),
                    step_ids=("only-one",),
                ),
                "invalid_step_ids",
            ),
            (
                lambda: project(
                    decision(FINISH, (FINISH,)),
                    inactive_state(),
                    step_ids=("unused",),
                ),
                "unexpected_terminal_step_ids",
            ),
            (
                lambda: project(
                    decision(ADVANCE, (ADVANCE, FINISH)),
                    inactive_state(),
                    step_ids=("one", "two"),
                ),
                "unsupported_terminal_tail",
            ),
            (
                lambda: project(
                    decision(
                        SCAN_FRONT_ARC,
                        (SCAN_FRONT_ARC,),
                        target_id="different-hazard",
                    ),
                    active_state(),
                    step_ids=("scan",),
                ),
                "scan_intent_target_mismatch",
            ),
            (
                lambda: project(
                    decision(OBSERVE, (OBSERVE,)),
                    {"active": None},
                    step_ids=("observe",),
                ),
                "invalid_post_maneuver_state",
            ),
        )
        for operation, code in cases:
            with self.subTest(code=code):
                with self.assertRaises(
                    LegacyDecisionShadowProjectionError
                ) as caught:
                    operation()
                self.assertEqual(caught.exception.code, code)


if __name__ == "__main__":
    unittest.main()
