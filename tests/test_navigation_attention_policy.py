import copy
import tempfile
import unittest
from pathlib import Path

try:
    import numpy as np
except ImportError:
    np = None


@unittest.skipIf(np is None, "Optional NumPy experiment dependency")
class AttentionPolicyTests(unittest.TestCase):
    def setUp(self):
        from experiments.navigation_attention_policy import AttentionPolicy, empty_weights
        self.policy_type = AttentionPolicy
        self.w = empty_weights()
        self.packet = {"available_actions": ["ADVANCE", "OBSERVE"],
            "active_waypoint": {"x_mm": 500, "y_mm": 0},
            "local_map_evidence": {"range_observations": [
                {"distance_mm": 200, "range_state": "MEASURED"},
                {"distance_mm": 800, "range_state": "MEASURED"}]}}

    def test_dimensions_and_no_hidden_training(self):
        self.assertEqual(sum(v.size for v in self.w.values()), 553352)
        policy = self.policy_type(self.w)
        self.assertEqual(policy.scores(self.packet), [0] * 8)
        for value in self.w.values():
            self.assertFalse(np.any(value))

    def test_unpooled_tokens_and_attention_affect_scores(self):
        self.w["ray_w"][3, 0] = 1
        self.w["b0_v_w"][0, 0] = 1
        self.w["b0_o_w"][0, 1] = 1
        self.w["out_w"][1, 1] = 1
        policy = self.policy_type(self.w)
        before = policy.scores(self.packet)
        changed = copy.deepcopy(self.packet)
        changed["local_map_evidence"]["range_observations"][1]["distance_mm"] = 1200
        self.assertGreater(policy.scores(changed)[1], before[1])
        changed = copy.deepcopy(self.packet)
        changed["local_map_evidence"]["range_observations"].reverse()
        np.testing.assert_allclose(policy.scores(changed), before, atol=1e-6)

    def test_memory_tokens_survive_more_than_four_records(self):
        self.w["memory_w"][22, 0] = 1
        self.w["b0_v_w"][0, 0] = 1
        self.w["b0_o_w"][0, 1] = 1
        self.w["out_w"][1, 7] = 1
        self.packet["local_map_evidence"]["route_execution_memory"] = {"event_count": 10, "records": [
            {"waypoint_plan": [{"x_mm": 500, "y_mm": 0}], "count": i+1, "last_seen": i}
            for i in range(10)]}
        policy = self.policy_type(self.w)
        before = policy.scores(self.packet)[7]
        self.packet["local_map_evidence"]["route_execution_memory"]["records"][0]["count"] = 100
        self.assertGreater(policy.scores(self.packet)[7], before)

    def test_mask_matches_existing_interface(self):
        self.w["out_b"][3] = 100
        self.w["out_b"][6] = 1
        self.assertEqual(self.policy_type(self.w).predict(self.packet)[0], "OBSERVE")
        self.w["out_b"][7] = 2
        self.assertEqual(self.policy_type(self.w).predict(self.packet)[0], "REPLAN")

    def test_map_geometry_is_observed_rotated_and_reaches_scores(self):
        from experiments.navigation_attention_policy import observations, MAP_FIELDS
        evidence = self.packet["local_map_evidence"]
        evidence.update(robot_pose={"x_mm":100,"y_mm":200,"heading_deg":90},
            coarse_grid={"cell_size_mm":100,"column_y_mm":[400],"rows":[{"x_mm":100,"cells":"#"}]},
            current_echo_clusters=[{"echo_bounds_mm":{"x_min_mm":100,"x_max_mm":100,"y_min_mm":400,"y_max_mm":400},
                                   "evidence_count":2,"provisional":True}],
            visited_cells=[{"x_mm":100,"y_mm":200}])
        tokens = observations(self.packet)[3]
        self.assertEqual(len(tokens),3)
        self.assertTrue(all(len(t)==len(MAP_FIELDS) for t in tokens))
        self.assertEqual(tokens[0][MAP_FIELDS.index("body_expanded")],1)
        self.assertEqual(tokens[1][MAP_FIELDS.index("body_expanded")],0)
        self.assertAlmostEqual(tokens[1][MAP_FIELDS.index("corner_0_forward_m")],.2)
        self.assertAlmostEqual(tokens[1][MAP_FIELDS.index("corner_0_left_m")],0)
        self.w["map_w"][MAP_FIELDS.index("corner_0_forward_m"),0]=1
        self.w["b0_v_w"][0,0]=1; self.w["b0_o_w"][0,1]=1; self.w["out_w"][1,1]=1
        policy=self.policy_type(self.w)
        before=policy.scores(self.packet)
        evidence["current_echo_clusters"][0]["echo_bounds_mm"]["y_max_mm"]=900
        evidence["current_echo_clusters"][0]["echo_bounds_mm"]["y_min_mm"]=900
        self.assertNotEqual(policy.scores(self.packet),before)

    def test_ir_unknown_bounds_remain_unknown_and_no_truth_is_read(self):
        from experiments.navigation_attention_policy import observations, MAP_FIELDS
        packet={"local_map_evidence":{"ir_survey":{"samples":[]}},
                "obstacle_estimates":[{"bounds_mm":{"min_x_mm":None},"confidence_milli":800}]}
        values=observations(packet)
        self.assertEqual(values[3][0][MAP_FIELDS.index("metric_bounds_present")],0)
        self.assertEqual(values[3][0][13:21],[0]*8)
        self.assertEqual(values[3][0][5:12],[0]*7)
        packet["simulator_truth"]={"obstacles":[[1,2,3,4]]}
        self.assertEqual(observations(packet),values)

    def test_execution_geometry_changes_numeric_input(self):
        from experiments.navigation_attention_policy import observations
        before=observations(self.packet)[0]
        self.packet["execution"]={"advance_step_mm":45,"reverse_step_mm":45,
            "follow_waypoint_max_forward_pulses":1,"scan_front_arc":{
                "relative_bearings_deg":[-60,0,60],"motion":"in_place_body_rotation","returns_to_start_heading":True}}
        after=observations(self.packet)[0]
        self.assertNotEqual(before,after)
        np.testing.assert_allclose(after[-11:-3],[.045,.045,1,-1/3,1/3,.03,1,1])

    def test_load_numeric_arrays_no_executable_payload(self):
        from experiments.local_navigation import LocalPolicy
        from experiments.navigation_attention_policy import save_weights
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.npz"
            save_weights(path, self.w, robot="blast")
            self.assertIsInstance(LocalPolicy.load(path), self.policy_type)
            np.testing.assert_array_equal(LocalPolicy.load(path).scores(self.packet), [0]*8)
        self.w["out_b"][0] = float("nan")
        with self.assertRaises(ValueError):
            self.policy_type(self.w)

    def test_constructor_guard(self):
        from experiments.navigation_attention_author import audit_constructor
        for code in ("import os\ndef build(robot): pass", "def build(robot): open('x')",
                     "def build(robot): return np.load('x')"):
            with self.assertRaises(ValueError):
                audit_constructor(code)
        audit_constructor("import numpy as np\ndef build(robot): return {}")


if __name__ == "__main__":
    unittest.main()
