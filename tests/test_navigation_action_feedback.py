"""Action completion must not be inferred from stale wheel movement or range."""
import copy
import unittest

from robot_agent.route_execution_memory import latest_action_feedback, route_execution_memory
from robot_agent.blast_episode_planning import _planner_history


class ActionFeedbackTests(unittest.TestCase):
    def event(self, completed=True):
        return {"action": "OBSERVE", "action_completed": completed,
                "observation_acquired": completed, "result": "observation_refreshed",
                "last_motion": None, "pose": {"x_mm": 0, "y_mm": 0, "heading_deg": 0},
                "waypoint_plan": [{"x_mm": 800, "y_mm": 0}]}

    def test_stationary_success_is_not_fabricated_motion(self):
        history = [{"action": "ADVANCE", "last_motion": {"completed": True, "made_progress": True}},
                   self.event()]
        receipt = latest_action_feedback(history)
        self.assertEqual(receipt["action"], "OBSERVE")
        self.assertTrue(receipt["completed"])
        record = route_execution_memory(history)["records"][0]
        self.assertTrue(record["action_completed"])
        self.assertIsNone(record["completed"])
        self.assertIsNone(record["made_progress"])
        self.assertEqual(_planner_history(history)[-1]["action_completed"], True)

    def test_unknown_result_does_not_inherit_previous_success(self):
        history = [self.event(), {"action": "OBSERVE", "event": "transport_retry"}]
        self.assertIsNone(latest_action_feedback(history)["completed"])
        self.assertIsNone(latest_action_feedback(history)["observation_acquired"])
        self.assertIsNone(latest_action_feedback([]))

    def test_failed_and_completed_measurements_do_not_collapse(self):
        history = [self.event(False), self.event(True), self.event(True)]
        records = route_execution_memory(history)["records"]
        self.assertEqual([r["count"] for r in records], [1, 2])
        self.assertEqual([r["action_completed"] for r in records], [False, True])

    def test_encoder_exposes_receipt_and_memory_without_changing_legacy_inputs(self):
        from experiments.navigation_attention_policy import observations, GLOBAL_SIZE, MEMORY_FIELDS
        from experiments.navigation_set_policy import observations as legacy
        history = [self.event()]
        packet = {"observation": {"last_action": latest_action_feedback(history)},
                  "local_map_evidence": {"route_execution_memory": route_execution_memory(history)}}
        g, _, m, _ = observations(packet)
        self.assertEqual(len(g), GLOBAL_SIZE)
        self.assertEqual(len(m[0]), len(MEMORY_FIELDS))
        self.assertEqual(g[267:271], [1, 0, 1, 0])
        self.assertEqual(m[0][70:], [1, 0, 1, 0])
        self.assertEqual(m[0][12:16], [0, 0, 0, 0])
        unknown = copy.deepcopy(packet)
        unknown["observation"]["last_action"] = {"action": "OBSERVE"}
        unknown["local_map_evidence"]["route_execution_memory"]["records"][0].pop("action_completed")
        self.assertNotEqual(observations(unknown)[0], g)
        self.assertEqual(legacy(unknown), legacy(packet))


if __name__ == "__main__":
    unittest.main()
