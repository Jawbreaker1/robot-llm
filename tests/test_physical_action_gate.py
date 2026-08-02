import unittest
from unittest import mock
from types import SimpleNamespace

from robot_agent.physical_action_gate import (
    PhysicalActionGateDecision,
    PhysicalActionGateError,
    PhysicalNavigationActionGate,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    EXPECTED_ACTION_SPECS,
    FINISH,
)


def observation(
    *,
    pulse_count_remaining=40,
    pulse_duration_ms_remaining=32_000,
    process_ms_remaining=40_000,
    motion_fault_latched=False,
):
    return {
        "state_version": 1,
        "observed_monotonic_ms": 10,
        "touch": {"value0": 0, "pressed": False},
        "infrared": {
            "raw": 60,
            "filtered": 60,
            "blocked": False,
            "reason": "clear_hysteresis_hold",
            "sample_count": 5,
        },
        "motors": [
            {"role": "left_drive", "position": 0, "state": ""},
            {"role": "right_drive", "position": 0, "state": ""},
        ],
        "last_outcome": {"kind": "observe", "status": "completed"},
        "budgets": {
            "pulse_count": 0,
            "pulse_count_remaining": pulse_count_remaining,
            "pulse_duration_ms": 0,
            "pulse_duration_ms_remaining": pulse_duration_ms_remaining,
            "process_ms_remaining": process_ms_remaining,
            "motion_fault_latched": motion_fault_latched,
        },
    }


class PhysicalActionGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = PhysicalNavigationActionGate()
        self.hazard_map = mock.Mock()
        self.hazard_map.validate_swept_path.return_value = {
            "allowed": True,
            "reason": "clear",
            "hazard_ids": [],
        }

    def evaluate(self, **overrides):
        values = {
            "action": ADVANCE,
            "observation": observation(),
            "action_specs": EXPECTED_ACTION_SPECS,
            "hazard_map": self.hazard_map,
            "pose": object(),
            "odometry_calibration": object(),
            "remaining_seconds": 10.0,
        }
        values.update(overrides)
        action = values.pop("action")
        return self.gate.evaluate_motion(action, **values)

    def test_prepare_returns_detached_navigation_and_exact_actions(self):
        navigation = {"action_feasibility": {}, "kept": {"value": 1}}

        def prepare(target, **_fields):
            target["scan_eligible_target_hypothesis_ids"] = ["box-a"]
            target["kept"]["value"] = 2
            return ("OBSERVE",)

        with mock.patch(
            "robot_agent.physical_action_gate."
            "physical_action_feasibility.prepare_navigation_availability",
            side_effect=prepare,
        ):
            result = self.gate.prepare(
                navigation,
                active_maneuver=None,
                scan_eligible_target_ids=("box-a",),
                scan_blocked_target_ids=(),
                scan_budget_available=True,
                reverse_budget_available=True,
                action_specs={},
                observation={},
                repeated_uninformative_observe=False,
            )

        self.assertEqual(result.actions, ("OBSERVE",))
        self.assertEqual(
            result.navigation["scan_eligible_target_hypothesis_ids"],
            ["box-a"],
        )
        self.assertEqual(result.navigation["kept"]["value"], 2)
        self.assertEqual(navigation, {
            "action_feasibility": {},
            "kept": {"value": 1},
        })

    def test_describe_feasibility_is_owned_and_detached_by_gate(self):
        source = {"motion_actions": {ADVANCE: {"allowed": True}}}
        with mock.patch(
            "robot_agent.physical_action_gate.physical_action_feasibility."
            "navigation_action_feasibility",
            return_value=source,
        ) as describe:
            result = self.gate.describe_navigation_feasibility(
                hazard_map=object(),
                pose=object(),
                action_specs={},
                odometry_calibration=object(),
                active_scan_calibration=object(),
            )

        result["motion_actions"][ADVANCE]["allowed"] = False
        self.assertTrue(source["motion_actions"][ADVANCE]["allowed"])
        describe.assert_called_once()

    def test_planner_proposal_gate_preserves_mission_and_detour_vetoes(self):
        mission = {
            "completed": False,
            "candidate_action_longitudinal_deltas_mm": {ADVANCE: 10},
            "projected_goal_heading_aligned_after_action": {ADVANCE: True},
        }
        navigation = {
            "navigation_hazard_hypotheses": [],
            "goal_geometry": {"conflicts": []},
        }
        advance = SimpleNamespace(
            action=ADVANCE,
            plan=(ADVANCE,),
            reason_code="PROGRESS_GOAL",
            perception_target_hypothesis_id=None,
            maneuver_commitment={},
        )

        allowed = self.gate.evaluate_planner_decision(
            advance,
            mission=mission,
            navigation=navigation,
        )
        self.assertTrue(allowed.allowed)

        finish = SimpleNamespace(
            action=FINISH,
            plan=(FINISH,),
            reason_code="COMPLETE_GOAL",
            perception_target_hypothesis_id=None,
            maneuver_commitment={},
        )
        denied = self.gate.evaluate_planner_decision(
            finish,
            mission=mission,
            navigation=navigation,
        )
        self.assertEqual(denied.reason_code, "premature_mission_finish")

        with mock.patch(
            "robot_agent.physical_action_gate.physical_action_feasibility."
            "detour_decision_error",
            return_value=("detour_contract_failed", "detail"),
        ):
            denied = self.gate.evaluate_planner_decision(
                advance,
                mission=mission,
                navigation=navigation,
            )
        self.assertEqual(denied.reason_code, "detour_contract_failed")

    def test_allows_motion_only_after_budget_geometry_and_deadline(self):
        result = self.evaluate()

        self.assertTrue(result.allowed)
        self.assertIsNone(result.veto_mapping())
        self.hazard_map.validate_swept_path.assert_called_once()

    def test_worker_budget_denial_does_not_consult_geometry(self):
        result = self.evaluate(
            observation=observation(pulse_count_remaining=0),
        )

        self.assertFalse(result.allowed)
        self.assertEqual(
            result.veto_mapping(),
            {
                "code": "worker_budget_insufficient",
                "action": ADVANCE,
                "host_selected_alternative_action": False,
            },
        )
        self.hazard_map.validate_swept_path.assert_not_called()

    def test_geometry_denial_preserves_typed_swept_path(self):
        swept = {
            "allowed": False,
            "reason": "provisional_hazard_sweep_collision",
            "hazard_ids": ["box-a"],
        }
        self.hazard_map.validate_swept_path.return_value = swept

        result = self.evaluate()

        self.assertEqual(
            result.veto_mapping(),
            {
                "code": "provisional_hazard_sweep_collision",
                "action": ADVANCE,
                "swept_path": swept,
                "host_selected_alternative_action": False,
            },
        )

    def test_deadline_denial_preserves_required_headroom(self):
        result = self.evaluate(remaining_seconds=0.5)

        veto = result.veto_mapping()
        self.assertEqual(veto["code"], "host_deadline_headroom_insufficient")
        self.assertGreater(veto["required_seconds"], 0.5)

    def test_decision_rejects_invalid_allow_and_veto_combinations(self):
        with self.assertRaises(PhysicalActionGateError):
            PhysicalActionGateDecision(
                action=ADVANCE,
                allowed=True,
                reason_code="unexpected",
            )
        with self.assertRaises(PhysicalActionGateError):
            PhysicalActionGateDecision(
                action=ADVANCE,
                allowed=False,
                reason_code=None,
            )


if __name__ == "__main__":
    unittest.main()
