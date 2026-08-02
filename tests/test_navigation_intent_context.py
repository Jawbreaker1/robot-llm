import json
import unittest

from robot_agent.navigation_intent_context import (
    MAX_NAVIGATION_INTENT_ACCOUNTED_BYTES,
    MAX_NAVIGATION_INTENT_CONTEXT_BYTES,
    NavigationIntentContextError,
    SYSTEM_PROMPT,
    build_navigation_intent_prompt,
)
from robot_agent.navigation_intent_proposal import (
    ABORT,
    DETOUR_TARGET,
    FOLLOW_DIRECTION,
    HOLD,
    LEFT,
    NavigationIntentOffer,
    RIGHT,
    SCAN_TARGET,
)
from robot_agent.physical_agent_state import (
    ControllerKey,
    GoalAssignment,
    NavigationBasis,
)


def basis():
    return NavigationBasis(
        controller_key=ControllerKey("robot-a", "ev3-a", "boot-a"),
        goal_epoch=3,
        controller_state_version=11,
        world_generation_id="map-a",
        world_model_version=9,
        navigation_basis_id="basis-a",
        frame_id="frame-a",
        calibration_fingerprint="calibration-a",
    )


def goal():
    return GoalAssignment(
        goal_id="goal-a",
        goal_epoch=3,
        objective="Move forward and navigate around obstacles.",
        source="user",
        locale="en",
        activated_at_ms=1_000,
    )


def offer(**changes):
    values = {
        "ticket_id": "ticket-a",
        "basis": basis(),
        "offered_intents": (
            FOLLOW_DIRECTION,
            SCAN_TARGET,
            DETOUR_TARGET,
            HOLD,
            ABORT,
        ),
        "scan_target_ids": ("hazard-a",),
        "detour_target_ids": ("hazard-a",),
        "detour_sides": (LEFT, RIGHT),
        "hold_reasons": ("WAIT_FOR_EVIDENCE",),
        "abort_reasons": ("LOCALIZATION_LOST",),
    }
    values.update(changes)
    return NavigationIntentOffer(**values)


def mission(**changes):
    values = {
        "current_longitudinal_progress_mm": 120,
        "remaining_longitudinal_progress_mm": 300,
        "regression_from_peak_mm": 0,
        "lateral_offset_mm": 4,
        "goal_heading_aligned": True,
        "goal_corridor_clear": False,
        "all_known_hazards_passed": False,
        "localization_valid": True,
        "touch_clear": True,
        "completed": False,
    }
    values.update(changes)
    return values


def navigation(**changes):
    values = {
        "pose": {"x_mm": 120, "y_mm": 4, "heading_mdeg": 0},
        "goal_geometry": {
            "conflicts": [{
                "hypothesis_id": "hazard-a",
                "active_for_collision": True,
            }],
        },
        "navigation_hazard_hypotheses": [{
            "hypothesis_id": "hazard-a",
            "centroid_x_mm": 230,
            "centroid_y_mm": 0,
            "radius_mm": 65,
            "active_for_collision": True,
            "collision_support_count": 3,
            "route_commitment_ready": True,
            "route_evidence": {
                "reason": "COMPLEMENTARY_BOUNDARIES_AT_CURRENT_POSE",
            },
            "scan_evidence_history": [{
                "completed_at_ms": 5_000,
                "status": "COMPLETED",
                "observation_pattern": "MIXED",
                "arc_coverage": "BILATERAL_ARC",
                "boundary_coverage": "BILATERAL_BOUNDARIES",
                "hypothesis_relation": "SUPPORTS_BLOCKED_HYPOTHESIS",
                "left_boundary_mdeg": 25_000,
                "right_boundary_mdeg": -20_000,
                "rays": [{"raw": "large payload that must not leak"}],
            }],
        }],
    }
    values.update(changes)
    return values


class NavigationIntentContextTests(unittest.TestCase):
    def test_prompt_keeps_causal_target_facts_without_host_identity_or_raw_rays(self):
        prompt = build_navigation_intent_prompt(
            goal=goal(),
            mission=mission(),
            navigation=navigation(),
            offer=offer(),
            latest_outcome={
                "operation": "scan",
                "status": "completed",
                "target_hypothesis_id": "hazard-a",
                "ignored_blob": "x" * 100_000,
            },
        )

        encoded = json.dumps(prompt.context, sort_keys=True)
        self.assertNotIn("ticket-a", encoded)
        self.assertNotIn("goal-a", encoded)
        self.assertNotIn("boot-a", encoded)
        self.assertNotIn("large payload", encoded)
        self.assertNotIn("ignored_blob", encoded)
        target = prompt.context["offered_target_evidence"][0]
        self.assertEqual(target["target_id"], "hazard-a")
        self.assertTrue(target["goal_conflict"])
        self.assertTrue(target["route_ready"])
        self.assertEqual(
            target["latest_scan"]["boundary_coverage"],
            "BILATERAL_BOUNDARIES",
        )

    def test_prompt_is_small_and_deterministic(self):
        first = build_navigation_intent_prompt(
            goal=goal(), mission=mission(), navigation=navigation(), offer=offer()
        )
        second = build_navigation_intent_prompt(
            goal=goal(), mission=mission(), navigation=navigation(), offer=offer()
        )

        self.assertEqual(first, second)
        schema_bytes = len(json.dumps(
            first.response_schema,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8"))
        self.assertEqual(len(SYSTEM_PROMPT.encode("utf-8")), 758)
        self.assertEqual(schema_bytes, 980)
        self.assertEqual(first.context_bytes, 1_036)
        self.assertEqual(first.accounted_bytes, 4_822)
        self.assertLessEqual(len(SYSTEM_PROMPT.encode("utf-8")), 1024)
        self.assertLess(first.context_bytes, MAX_NAVIGATION_INTENT_CONTEXT_BYTES)
        self.assertLess(first.accounted_bytes, MAX_NAVIGATION_INTENT_ACCOUNTED_BYTES)
        self.assertLessEqual(first.context_bytes, 1536)
        self.assertLessEqual(first.accounted_bytes, 6 * 1024)

    def test_rich_unoffered_memory_stays_on_host_without_growing_prompt(self):
        value = navigation()
        for index in range(200):
            value["navigation_hazard_hypotheses"].append({
                "hypothesis_id": "remembered-{}".format(index),
                "active_for_collision": False,
                "route_commitment_ready": False,
                "scan_evidence_history": [{"blob": "x" * 10_000}],
            })

        prompt = build_navigation_intent_prompt(
            goal=goal(), mission=mission(), navigation=value, offer=offer()
        )

        self.assertEqual(prompt.context["known_hazard_count"], 201)
        self.assertEqual(len(prompt.context["offered_target_evidence"]), 1)
        self.assertLess(prompt.context_bytes, 4_000)

    def test_missing_offered_target_and_invalid_mission_fail_closed(self):
        no_targets = navigation(navigation_hazard_hypotheses=[])
        with self.assertRaises(NavigationIntentContextError) as caught:
            build_navigation_intent_prompt(
                goal=goal(), mission=mission(), navigation=no_targets, offer=offer()
            )
        self.assertEqual(caught.exception.code, "offered_target_missing")

        with self.assertRaises(NavigationIntentContextError):
            build_navigation_intent_prompt(
                goal=goal(),
                mission=mission(localization_valid="yes"),
                navigation=navigation(),
                offer=offer(),
            )


if __name__ == "__main__":
    unittest.main()
