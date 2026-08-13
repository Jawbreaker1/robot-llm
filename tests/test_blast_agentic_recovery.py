import unittest

from robot_agent.blast_agentic_recovery import (
    BlastAgenticRecovery,
    TARGET_REACQUISITION_UNRESOLVED,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)


class BlastAgenticRecoveryTests(unittest.TestCase):
    def test_recovery_requests_fresh_scan_then_only_untried_side(self):
        state = BlastAgenticRecovery().record_side("RIGHT").recover(
            reason=TARGET_REACQUISITION_UNRESOLVED,
            evidence={
                "multi_view_observations": {
                    "selected_side": "RIGHT",
                    "route_eligible": False,
                    "views": [{"large": "payload"}],
                },
            },
        )

        self.assertIsNotNone(state)
        self.assertEqual(state.planner_actions(
            (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90, SCAN_FRONT_ARC),
            scan_is_current=False,
        ), (SCAN_FRONT_ARC,))
        self.assertEqual(state.planner_actions(
            (TURN_LEFT_90, TURN_RIGHT_90),
            scan_is_current=True,
        ), (TURN_LEFT_90,))
        context = state.context()
        self.assertEqual(context["attempted_sides"], ["RIGHT"])
        self.assertEqual(context["evidence"], {
            "selected_side": "RIGHT",
            "route_eligible": False,
        })

    def test_recovery_budget_is_explicit_and_finite(self):
        state = BlastAgenticRecovery(max_replans=2)
        state = state.recover(
            reason=TARGET_REACQUISITION_UNRESOLVED,
            evidence=None,
        )
        state = state.recover(
            reason=TARGET_REACQUISITION_UNRESOLVED,
            evidence=None,
        )

        self.assertIsNone(state.recover(
            reason=TARGET_REACQUISITION_UNRESOLVED,
            evidence=None,
        ))


if __name__ == "__main__":
    unittest.main()
