import unittest

from robot_agent.hybrid_navigation_planner import (
    HybridNavigationPlanner,
    LEGACY_FULL_PATH,
    ZERO_CALL_FOLLOW_PATH,
    ZERO_CALL_SCAN_PATH,
)
from robot_agent.lm_studio_navigation import NavigationPlannerResult
from robot_agent.maneuver_commitment import empty_commitment
from robot_agent.physical_navigation_contract import (
    ACTIONS,
    ADVANCE,
    DECISION_SCHEMA,
    FINISH,
    SCAN_FRONT_ARC,
    NavigationDecision,
)


def observation(*, blocked=False):
    return {
        "state_version": 7,
        "touch": {"pressed": False},
        "infrared": {"blocked": blocked},
        "budgets": {"motion_fault_latched": False},
    }


def mission(**changes):
    value = {
        "user_goal": "Explore forward",
        "completed": False,
        "localization_valid": True,
        "touch_clear": True,
        "goal_corridor_clear": True,
        "goal_heading_aligned": True,
        "all_known_hazards_passed": True,
        "candidate_action_longitudinal_deltas_mm": {ADVANCE: 80},
        "current_longitudinal_progress_mm": 0,
        "remaining_longitudinal_progress_mm": 420,
        "regression_from_peak_mm": 0,
        "lateral_offset_mm": 0,
    }
    value.update(changes)
    return value


def navigation(*, hazards=(), conflicts=(), eligible=()):
    return {
        "robot_id": "ev3rstorm-01",
        "controller_instance_id": "ev3-main",
        "map_version": 2,
        "pose": {"x_mm": 0, "y_mm": 0, "heading_mdeg": 0},
        "goal_geometry": {"conflicts": list(conflicts)},
        "navigation_hazard_hypotheses": list(hazards),
        "scan_eligible_target_hypothesis_ids": list(eligible),
    }


def legacy_result():
    decision = NavigationDecision.from_mapping(
        {
            "schema": DECISION_SCHEMA,
            "episode_id": "episode-a",
            "turn": 1,
            "based_on_state_version": 7,
            "action": FINISH,
            "plan": [FINISH],
            "reason_code": "COMPLETE_GOAL",
            "assessment": "Legacy result",
            "utterance": None,
            "perception_target_hypothesis_id": None,
            "maneuver_commitment": empty_commitment(),
        },
        episode_id="episode-a",
        turn=1,
        state_version=7,
        available_actions=ACTIONS,
    )
    return NavigationPlannerResult(
        decision=decision,
        latency_ms=123,
        served_model="model-a",
        usage=None,
        stats={"tokens_per_second": 80.0},
    )


class RecordingPlanner:
    def __init__(self):
        self.calls = []
        self.result = legacy_result()

    def decide(self, **kwargs):
        self.calls.append(kwargs)
        return self.result


class ForbiddenCompactClient:
    def __init__(self):
        self.calls = []

    def decide(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        raise AssertionError("one-choice compact path called the model")


def arguments(**changes):
    value = {
        "episode_id": "episode-a",
        "turn": 1,
        "locale": "sv",
        "observation": observation(),
        "mission": mission(),
        "navigation": navigation(),
        "maneuver_state": {"active": None, "last_terminal": None},
        "available_actions": tuple(ACTIONS),
        "last_tool_result": None,
        "validation_feedback": None,
        "recent_committed_utterances": (),
    }
    value.update(changes)
    return value


class HybridNavigationPlannerTests(unittest.TestCase):
    def planner(self):
        self.legacy = RecordingPlanner()
        self.compact = ForbiddenCompactClient()
        self.telemetry = []
        return HybridNavigationPlanner(
            legacy_planner=self.legacy,
            compact_client=self.compact,
            monotonic=lambda: 3.0,
            unix_ms=lambda: 2_000,
            telemetry=self.telemetry.append,
        )

    def test_clear_aligned_corridor_compiles_follow_without_model_call(self):
        result = self.planner().decide(**arguments())

        self.assertEqual(result.decision.action, ADVANCE)
        self.assertEqual(result.decision.plan, (ADVANCE, ADVANCE))
        self.assertIsNone(result.decision.utterance)
        self.assertEqual(result.decision_path, ZERO_CALL_FOLLOW_PATH)
        self.assertEqual(result.model_call_count, 0)
        self.assertEqual(self.legacy.calls, [])
        self.assertEqual(self.compact.calls, [])
        self.assertEqual(self.telemetry[0].intent, "FOLLOW_DIRECTION")

    def test_short_remaining_goal_and_degraded_result_use_legacy(self):
        for changes in (
            {"mission": mission(remaining_longitudinal_progress_mm=100)},
            {
                "last_tool_result": {
                    "operation": "pulse",
                    "requested_action": ADVANCE,
                    "status": "degraded",
                    "reason": "encoder_undertravel_observed",
                }
            },
            {
                "last_tool_result": {
                    "operation": "pulse",
                    "status": "vetoed",
                }
            },
        ):
            with self.subTest(changes=changes):
                planner = self.planner()
                result = planner.decide(**arguments(**changes))

                self.assertEqual(result.decision_path, LEGACY_FULL_PATH)
                self.assertEqual(len(self.legacy.calls), 1)
                self.assertEqual(self.compact.calls, [])

    def test_one_blocking_unscanned_target_compiles_scan_without_model_call(self):
        hazard = {
            "hypothesis_id": "hazard-a",
            "route_commitment_ready": False,
        }
        conflict = {"hypothesis_id": "hazard-a"}
        result = self.planner().decide(**arguments(
            observation=observation(blocked=True),
            mission=mission(
                goal_corridor_clear=False,
                all_known_hazards_passed=False,
            ),
            navigation=navigation(
                hazards=(hazard,),
                conflicts=(conflict,),
                eligible=("hazard-a",),
            ),
        ))

        self.assertEqual(result.decision.action, SCAN_FRONT_ARC)
        self.assertEqual(result.decision.plan, (SCAN_FRONT_ARC,))
        self.assertEqual(
            result.decision.perception_target_hypothesis_id,
            "hazard-a",
        )
        self.assertEqual(result.decision_path, ZERO_CALL_SCAN_PATH)
        self.assertEqual(result.model_call_count, 0)
        self.assertEqual(self.legacy.calls, [])
        self.assertEqual(self.compact.calls, [])

    def test_verified_infrared_stop_can_enter_single_target_scan(self):
        hazard = {
            "hypothesis_id": "hazard-a",
            "route_commitment_ready": False,
        }
        result = self.planner().decide(**arguments(
            observation=observation(blocked=True),
            mission=mission(
                goal_corridor_clear=False,
                all_known_hazards_passed=False,
            ),
            navigation=navigation(
                hazards=(hazard,),
                conflicts=({"hypothesis_id": "hazard-a"},),
                eligible=("hazard-a",),
            ),
            last_tool_result={
                "operation": "pulse",
                "requested_action": ADVANCE,
                "status": "interrupted",
                "reason": "infrared_blocked",
            },
        ))

        self.assertEqual(result.decision_path, ZERO_CALL_SCAN_PATH)
        self.assertEqual(self.legacy.calls, [])
        self.assertEqual(self.compact.calls, [])

    def test_active_maneuver_selects_legacy_before_compact_path(self):
        planner = self.planner()
        result = planner.decide(**arguments(
            maneuver_state={"active": {"id": "route-a"}},
        ))

        self.assertEqual(result.decision, self.legacy.result.decision)
        self.assertEqual(result.decision_path, LEGACY_FULL_PATH)
        self.assertEqual(result.model_call_count, 1)
        self.assertEqual(len(self.legacy.calls), 1)
        self.assertEqual(self.compact.calls, [])

    def test_ambiguous_scan_targets_select_legacy_before_compact_path(self):
        hazards = (
            {"hypothesis_id": "hazard-a", "route_commitment_ready": False},
            {"hypothesis_id": "hazard-b", "route_commitment_ready": False},
        )
        conflicts = (
            {"hypothesis_id": "hazard-a"},
            {"hypothesis_id": "hazard-b"},
        )
        planner = self.planner()
        result = planner.decide(**arguments(
            observation=observation(blocked=True),
            mission=mission(
                goal_corridor_clear=False,
                all_known_hazards_passed=False,
            ),
            navigation=navigation(
                hazards=hazards,
                conflicts=conflicts,
                eligible=("hazard-a", "hazard-b"),
            ),
        ))

        self.assertEqual(result.decision_path, LEGACY_FULL_PATH)
        self.assertEqual(len(self.legacy.calls), 1)
        self.assertEqual(self.compact.calls, [])


if __name__ == "__main__":
    unittest.main()
