import json
import unittest

from robot_agent.lm_studio_navigation import (
    TARGET_PLANNER_CONTEXT_BYTES,
    LMStudioNavigationPlanner,
)
from robot_agent.maneuver_commitment import empty_commitment
from robot_agent.physical_navigation_contract import (
    ACTIONS,
    DECISION_SCHEMA,
    OBSERVE,
    json_bytes,
)
from robot_agent.physical_navigation_experience import (
    MAX_EXPERIENCE_CONTEXT_BYTES,
)
from robot_agent.physical_navigation_planner_context import (
    COMPACT_ROUTE_SCAN_DETAIL,
    project_navigation_context,
)


def scan(index):
    return {
        "scan_id": "scan-{:03d}".format(index),
        "completed_at_ms": 10_000 + index,
        "status": "COMPLETED",
        "reason": "bilateral_boundaries_observed",
        "observation_pattern": "MIXED",
        "arc_coverage": "BILATERAL_ARC",
        "boundary_coverage": "BILATERAL_BOUNDARIES",
        "hypothesis_relation": "SUPPORTS_BLOCKED_HYPOTHESIS",
        "left_boundary_mdeg": 20_000,
        "right_boundary_mdeg": -20_000,
        "scan_pose": {
            "x_mm": index,
            "y_mm": 0,
            "heading_mdeg": 0,
        },
        "based_on_map_version": index,
        "rays": [
            {
                "requested_relative_bearing_mdeg": (ray - 8) * 1_000,
                "actual_relative_bearing_mdeg": (ray - 8) * 1_000,
                "blocked": ray % 2 == 0,
                "raw": 20 if ray % 2 == 0 else 60,
                "filtered": 20 if ray % 2 == 0 else 60,
            }
            for ray in range(16)
        ],
    }


def hazard(index, *, applicable=False):
    scan_id = "scan-{:03d}".format(index)
    return {
        "hypothesis_id": "hazard-{:02d}".format(index),
        "centroid_x_mm": 140 + index,
        "centroid_y_mm": 0,
        "radius_mm": 70,
        "active_for_collision": True,
        "collision_support_count": 2,
        "route_commitment_ready": applicable,
        "route_evidence": {
            "ready": applicable,
            "reason": (
                "COMPLEMENTARY_BOUNDARIES_AT_CURRENT_POSE"
                if applicable
                else "NO_SCAN_EVIDENCE_AT_CURRENT_VERIFIED_POSE"
            ),
            "applicable_scan_ids": [scan_id] if applicable else [],
            "positive_boundary_scan_ids": [scan_id] if applicable else [],
            "negative_boundary_scan_ids": [scan_id] if applicable else [],
        },
        "scan_evidence_history": [scan(index)],
    }


def navigation(count=6):
    return {
        "map_generation_id": "map-a",
        "map_version": 4,
        "pose": {"x_mm": 0, "y_mm": 0, "heading_mdeg": 0},
        "navigation_hazard_hypotheses": [
            hazard(index, applicable=index == 4) for index in range(count)
        ],
        "goal_geometry": {
            "conflicts": [{"hypothesis_id": "hazard-01"}],
            "facts": {"GOAL_CORRIDOR_CLEAR": False},
        },
        "action_feasibility": {
            "motion_actions": {
                "ADVANCE": {
                    "allowed": False,
                    "hazard_ids": ["hazard-02"],
                    "monotonic_escape_hazard_ids": [],
                },
            },
            "active_scan": {
                "allowed": True,
                "hazard_ids": [],
            },
        },
        "scan_front_arc_feasibility": {
            "allowed": True,
            "hazard_ids": [],
        },
    }


class PhysicalPlannerProjectionTests(unittest.TestCase):
    def test_typed_focus_preserves_detail_and_compacts_other_evidence(self):
        value = navigation()
        projected = project_navigation_context(
            value,
            maneuver_state={
                "active": {"target_hypothesis_id": "hazard-00"},
            },
            last_tool_result={
                "target_hypothesis_id": "hazard-03",
            },
            target_budget_bytes=128 * 1024,
            hard_budget_bytes=160 * 1024,
        )
        by_id = {
            item["hypothesis_id"]: item
            for item in projected["navigation_hazard_hypotheses"]
        }

        self.assertEqual(
            projected["planner_context_projection"][
                "focused_hypothesis_ids"
            ],
            ["hazard-00", "hazard-03", "hazard-04"],
        )
        self.assertEqual(
            projected["planner_context_projection"][
                "referenced_hypothesis_ids"
            ],
            ["hazard-00", "hazard-01", "hazard-02", "hazard-03", "hazard-04"],
        )
        for hypothesis_id in (
            "hazard-00",
            "hazard-03",
        ):
            self.assertEqual(len(by_id[hypothesis_id]["scan_evidence_history"]), 1)
            self.assertEqual(
                by_id[hypothesis_id]["scan_evidence_summary"][
                    "attempt_counts"
                ][
                    "omitted_detail"
                ],
                0,
            )
        self.assertEqual(
            by_id["hazard-04"]["scan_evidence_history"][0][
                "detail_projection"
            ],
            COMPACT_ROUTE_SCAN_DETAIL,
        )
        self.assertEqual(
            by_id["hazard-04"]["scan_evidence_summary"]["scan_ids"],
            ["scan-004"],
        )
        self.assertEqual(by_id["hazard-05"]["scan_evidence_history"], [])
        self.assertEqual(
            by_id["hazard-05"]["scan_evidence_summary"][
                "retained_attempt_count"
            ],
            1,
        )
        self.assertNotIn("scan_front_arc_feasibility", projected)
        self.assertIn("active_scan", projected["action_feasibility"])

    def test_worst_case_projection_is_bounded_and_preserves_all_hazard_ids(self):
        value = navigation(64)
        value["goal_geometry"]["conflicts"] = [
            {"hypothesis_id": "hazard-{:02d}".format(index)}
            for index in range(64)
        ]
        first = project_navigation_context(
            value,
            maneuver_state={"active": None},
            last_tool_result=None,
            target_budget_bytes=128 * 1024,
            hard_budget_bytes=160 * 1024,
        )
        second = project_navigation_context(
            value,
            maneuver_state={"active": None},
            last_tool_result=None,
            target_budget_bytes=128 * 1024,
            hard_budget_bytes=160 * 1024,
        )

        self.assertLessEqual(len(json_bytes(first)), 128 * 1024)
        self.assertEqual(json_bytes(first), json_bytes(second))
        self.assertEqual(
            [
                item["hypothesis_id"]
                for item in first["navigation_hazard_hypotheses"]
            ],
            ["hazard-{:02d}".format(index) for index in range(64)],
        )
        metadata = first["planner_context_projection"]
        self.assertEqual(metadata["scan_attempt_count"], 64)
        self.assertGreater(metadata["scan_detail_omitted_count"], 0)
        self.assertTrue(metadata["all_hazard_ids_preserved"])
        protected = first["navigation_hazard_hypotheses"][4]
        self.assertEqual(
            protected["scan_evidence_summary"]["scan_ids"],
            ["scan-004"],
        )

    def test_protected_route_scans_compact_before_the_hard_budget(self):
        value = navigation(64)
        for index, item in enumerate(
            value["navigation_hazard_hypotheses"]
        ):
            applicable = index < 32
            scan_id = item["scan_evidence_history"][0]["scan_id"]
            item["route_commitment_ready"] = applicable
            item["route_evidence"] = {
                "ready": applicable,
                "reason": (
                    "COMPLEMENTARY_BOUNDARIES_AT_CURRENT_POSE"
                    if applicable
                    else "NO_SCAN_EVIDENCE_AT_CURRENT_VERIFIED_POSE"
                ),
                "applicable_scan_ids": [scan_id] if applicable else [],
                "positive_boundary_scan_ids": (
                    [scan_id] if applicable else []
                ),
                "negative_boundary_scan_ids": (
                    [scan_id] if applicable else []
                ),
            }
        value["experience_ledger"] = {
            "schema": "robot-physical-navigation-experience/v1",
            "entries": [
                {
                    "outcome": {"status": "verification_failed"},
                    "padding": "x" * 900,
                }
                for _index in range(60)
            ],
        }

        projected = project_navigation_context(
            value,
            maneuver_state={"active": None},
            last_tool_result=None,
            target_budget_bytes=128 * 1024,
            hard_budget_bytes=160 * 1024,
        )

        self.assertLessEqual(len(json_bytes(projected)), 160 * 1024)
        self.assertEqual(
            [
                item["hypothesis_id"]
                for item in projected["navigation_hazard_hypotheses"]
            ],
            ["hazard-{:02d}".format(index) for index in range(64)],
        )
        self.assertEqual(
            projected["action_feasibility"],
            value["action_feasibility"],
        )
        metadata = projected["planner_context_projection"]
        self.assertEqual(metadata["scan_compact_detail_retained_count"], 32)
        self.assertEqual(metadata["scan_full_detail_retained_count"], 0)
        self.assertEqual(
            projected["navigation_hazard_hypotheses"][0][
                "scan_evidence_summary"
            ]["scan_ids"],
            ["scan-000"],
        )

    def test_max_ids_and_outcome_rollups_keep_mandatory_experience(self):
        value = navigation(64)
        for index, item in enumerate(
            value["navigation_hazard_hypotheses"]
        ):
            old_scan_id = item["scan_evidence_history"][0]["scan_id"]
            prefix = "scan-{:03d}-".format(index)
            scan_id = prefix + "x" * (128 - len(prefix))
            item["scan_evidence_history"][0]["scan_id"] = scan_id
            route = item["route_evidence"]
            for field in (
                "applicable_scan_ids",
                "positive_boundary_scan_ids",
                "negative_boundary_scan_ids",
            ):
                route[field] = [
                    scan_id if candidate == old_scan_id else candidate
                    for candidate in route[field]
                ]
        value["goal_geometry"]["conflicts"] = [
            {"hypothesis_id": "hazard-{:02d}".format(index)}
            for index in range(64)
        ]
        outcomes = [
            {
                "outcome": {
                    "operation": "observe",
                    "status": "observed",
                    "reason_code": "OUTCOME_{:03d}".format(index),
                },
                "count": 1,
            }
            for index in range(500)
        ]
        rollups = []
        for index, action in enumerate(sorted(ACTIONS)):
            bucket_count = 500 if index == 0 else 1
            rollups.append({
                "action": action,
                "attempt_count": bucket_count,
                "first_sequence": 1,
                "latest_sequence": 500,
                "outcome_bucket_count": bucket_count,
                "outcome_bucket_retained_count": bucket_count,
                "outcome_bucket_omitted_count": 0,
                "outcome_attempt_retained_count": bucket_count,
                "outcome_attempt_omitted_count": 0,
                "outcome_distribution": outcomes if index == 0 else [],
                "latest_outcome": {
                    "operation": "observe",
                    "status": "observed",
                    "reason_code": "LATEST_" + "y" * 150,
                },
            })
        value["experience_ledger"] = {
            "schema": "robot-physical-navigation-experience/v1",
            "episode_id": "max-stress",
            "scope": "EPISODE",
            "persisted": False,
            "host_ranked_or_selected_action": False,
            "capacity": 64,
            "retained_count": 64,
            "total_recorded_count": 14_400,
            "seen_action_basis_capacity": 43_200,
            "seen_action_basis_retained_count": 43_200,
            "seen_attempt_basis_capacity": 43_200,
            "seen_attempt_basis_retained_count": 43_200,
            "current_basis_id": "basis-" + "a" * 20,
            "current_basis_action_rollups": rollups,
            "entries": [],
        }
        self.assertLessEqual(
            len(json_bytes(value["experience_ledger"])),
            MAX_EXPERIENCE_CONTEXT_BYTES,
        )

        projected = project_navigation_context(
            value,
            maneuver_state={"active": None},
            last_tool_result=None,
            target_budget_bytes=56 * 1024,
            hard_budget_bytes=64 * 1024,
        )

        self.assertLessEqual(len(json_bytes(projected)), 64 * 1024)
        experience = projected["experience_ledger"]
        self.assertEqual(experience["total_recorded_count"], 14_400)
        self.assertEqual(experience["entries"], [])
        self.assertTrue(
            experience["planner_projection"][
                "target_budget_exceeded_due_to_mandatory_facts"
            ]
        )
        self.assertTrue(
            projected["planner_context_projection"][
                "target_budget_exceeded_due_to_mandatory_facts"
            ]
        )


class PlannerProjectionIntegrationTests(unittest.TestCase):
    def test_planner_reports_exact_context_bytes_and_typed_usage(self):
        captured = {}
        stress_navigation = navigation(64)
        stress_navigation["goal_geometry"]["conflicts"] = [
            {"hypothesis_id": "hazard-{:02d}".format(index)}
            for index in range(64)
        ]
        stress_navigation["scan_eligible_target_hypothesis_ids"] = [
            "hazard-{:02d}".format(index) for index in range(64)
        ]
        stress_navigation["experience_ledger"] = {
            "schema": "robot-physical-navigation-experience/v1",
            "total_recorded_count": 14_400,
            "retained_count": 64,
            "current_basis_action_rollups": [],
            "entries": [
                {
                    "sequence": index,
                    "outcome": {"status": "verification_failed"},
                    "basis_before": {"typed_facts": "x" * 850},
                }
                for index in range(64)
            ],
        }

        def transport(_url, body, _headers, _timeout, _maximum):
            payload = json.loads(body.decode("utf-8"))
            captured["context"] = payload["messages"][1]["content"]
            decision = {
                "schema": DECISION_SCHEMA,
                "episode_id": "episode-projection",
                "turn": 1,
                "based_on_state_version": 7,
                "action": OBSERVE,
                "plan": [OBSERVE],
                "reason_code": "VERIFY_RESULT",
                "assessment": "Use the projected physical facts.",
                "utterance": None,
                "perception_target_hypothesis_id": None,
                "maneuver_commitment": empty_commitment(),
            }
            return json.dumps({
                "model": payload["model"],
                "choices": [{"message": {"content": json.dumps(decision)}}],
                "usage": {
                    "prompt_tokens": 123,
                    "completion_tokens": 19,
                    "total_tokens": 142,
                },
            }).encode("utf-8")

        planner = LMStudioNavigationPlanner(
            base_url="http://127.0.0.1:1234",
            model="test-model",
            transport=transport,
            clock=lambda: 1.0,
        )
        result = planner.decide(
            episode_id="episode-projection",
            turn=1,
            locale="en",
            observation={"state_version": 7},
            mission={"completed": False},
            navigation=stress_navigation,
            maneuver_state={"active": None, "last_terminal": None},
            available_actions=[OBSERVE],
            last_tool_result=None,
        )

        self.assertEqual(
            result.context_byte_count,
            len(captured["context"].encode("utf-8")),
        )
        self.assertLessEqual(
            result.context_byte_count,
            TARGET_PLANNER_CONTEXT_BYTES,
        )
        self.assertEqual(result.prompt_tokens, 123)
        self.assertEqual(result.completion_tokens, 19)
        self.assertEqual(result.total_tokens, 142)
        self.assertLessEqual(
            result.estimated_prompt_tokens,
            result.prompt_token_budget,
        )
        self.assertLessEqual(
            result.context_byte_count,
            result.context_hard_byte_count,
        )
        projected = json.loads(captured["context"])["navigation"]
        experience = projected["experience_ledger"]
        self.assertEqual(experience["total_recorded_count"], 14_400)
        self.assertGreater(
            experience["planner_projection"][
                "omitted_detailed_entry_count"
            ],
            0,
        )


if __name__ == "__main__":
    unittest.main()
