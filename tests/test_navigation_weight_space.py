"""Check authoring plumbing on fixture weights, never tune the real candidate."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiments import local_navigation as network
from experiments.navigation_weight_space import (
    HIDDEN, METHOD, REPAIR_METHOD, author_request, compact_packet, expand_weights, feature_layout,
)


def specification():
    return {"design_summary":"test fixture", "limitations":"not a navigator",
            "units":[{"index":3,"bias":0.2,"purpose":"not executable",
                      "inputs":[{"feature":1,"weight":2.5}],
                      "outputs":[{"action":7,"weight":4.0}]}],
            "output_bias":[0.0]*8}


class WeightSpaceTests(unittest.TestCase):
    def test_layout_is_contiguous_and_matches_encoder(self):
        rows = feature_layout()
        cursor = 0
        for row in rows:
            self.assertEqual(row["start"], cursor)
            cursor += row["count"]
        self.assertEqual(cursor, len(network.features({})))
        by_name = {r["name"]:r for r in rows}
        self.assertEqual(by_name["last_motion.present"]["start"],73)
        self.assertEqual(by_name["latest_route_rejection.present"]["start"],167)
        self.assertEqual(by_name["history[3].pose"]["start"],1146)
        self.assertEqual(by_name["rays"]["start"],1444)
        packet = {"active_waypoint":{"x_mm":700,"y_mm":-200},
                  "local_map_evidence":{"latest_route_rejection":{
                      "rejection":{"reason":"test"},"repeat_count":2}},
                  "history":[{"action":"OBSERVE","result":"test"}]}
        values = network.features(packet)
        self.assertEqual(values[1:3],[0.7,-0.2])
        self.assertEqual(values[167],1)
        self.assertEqual(values[168:200],network._category("test"))
        self.assertEqual(values[1158:1190],network._category("test"))

    def test_expansion_only_copies_numbers_without_training_or_inference(self):
        spec = specification()
        before = deepcopy(spec)
        with patch.object(network,"train",side_effect=AssertionError("training")), \
             patch.object(network.LocalPolicy,"predict",side_effect=AssertionError("inference")):
            result = expand_weights(spec)
        self.assertEqual(spec,before)
        self.assertEqual(result["w1"][1][3],2.5)
        self.assertEqual(result["w2"][3][7],4.0)
        self.assertEqual(result["b1"][3],0.2)
        self.assertEqual(sum(v != 0 for row in result["w1"] for v in row),1)
        self.assertEqual(len(result["b1"]),HIDDEN)
        self.assertEqual(result["parameters"],79272)
        self.assertEqual(result["method"],METHOD)
        for key in ("training_rows","gradient_steps","author_forward_passes","author_environment_rollouts"):
            self.assertEqual(result[key],0)
        # A compiler test fixture, not an authored candidate or feedback to it.
        action, _ = network.LocalPolicy(result).predict({
            "active_waypoint":{"x_mm":1000,"y_mm":0},
            "available_actions":["FOLLOW_WAYPOINT"]})
        self.assertEqual(action,"REPLAN")

    def test_invalid_numeric_assignments_are_not_repaired(self):
        for weight in (True,"3",float("nan"),float("inf")):
            spec = specification()
            spec["units"][0]["inputs"][0]["weight"] = weight
            with self.subTest(weight=weight), self.assertRaises(ValueError):
                expand_weights(spec)
        for index in (-1,2468,True,1.0):
            spec = specification()
            spec["units"][0]["inputs"][0]["feature"] = index
            with self.subTest(index=index), self.assertRaises(ValueError):
                expand_weights(spec)
        for mutation in ("unit","input","output","bias"):
            spec = specification()
            if mutation == "unit":
                spec["units"] *= 2
            elif mutation == "bias":
                spec["output_bias"].pop()
            else:
                spec["units"][0][mutation+"s"] *= 2
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                expand_weights(spec)

    def test_author_receives_interface_not_training_or_results(self):
        request = author_request()
        self.assertEqual(request["reasoning_effort"],"high")
        contract = json.loads(request["messages"][1]["content"])
        self.assertEqual(contract["architecture"]["features"],2468)
        self.assertIn("def features(",contract["source"])
        self.assertNotIn("def train(",contract["source"])
        self.assertNotIn("local-artifacts",json.dumps(request))
        self.assertNotIn("test_cases",contract)
        self.assertEqual(contract["robots"]["EV3"]["advance_step_mm"],52)
        self.assertEqual(contract["robots"]["Blast"]["advance_step_mm"],45)
        self.assertNotEqual(contract["robots"]["EV3"]["body"],contract["robots"]["Blast"]["body"])

    def test_repair_projection_preserves_every_encoded_feature(self):
        from test_local_navigation_experiment import packet
        for is_ir in (False,True):
            p = packet()
            p["world"] = {"secret_obstacle":42}
            p["execution"] = {"next_waypoint_motion":{"action":"ADVANCE",
                "remaining_turn_deg":3,"unused":"discard",
                "straight_motion_projection":{"travel_mm":44,"waypoint_miss_mm":11}}}
            p["observation"]["last_motion"] = {"reason":"simulation_body_blocked",
                "action":"ADVANCE","completed":False,"made_progress":False}
            p["history"] = [{"action":"REVERSE","result":"semantic_action_completed",
                "last_motion":{"action":"REVERSE","completed":True,"measured_translation_mm":52},
                "scan":{"state":"partial","restoration_verified":False,"unused":"discard"}}]
            if is_ir:
                p["local_map_evidence"]["ir_survey"] = {"complete":False,"samples":[
                    {"observed_from":{"x_mm":10,"y_mm":20},"heading_deg":35,"filtered_ir_proximity":23}]}
                p["observation"]["infrared"] = {"filtered":23,"unused":"discard"}
            projected = compact_packet(p)
            self.assertEqual(network.features(projected),network.features(p))
            self.assertNotIn("world",projected)
            self.assertNotIn("unused",json.dumps(projected))

    def test_repair_request_is_not_misrepresented_as_zero_shot(self):
        evidence = {"previous_specification":specification(),"cases":[{"snapshots":[]}]}
        request = author_request(evidence)
        contract = json.loads(request["messages"][1]["content"])
        self.assertEqual(contract["repair_evidence"],evidence)
        self.assertIn("not zero-shot",contract["author_protocol"])
        self.assertIn("No training",contract["author_protocol"])
        self.assertEqual(contract["architecture"]["hidden"],32)
        self.assertIn("positioned sensor-ray geometry",contract["task"])

    def test_report_identifies_direct_weights_and_reused_reference(self):
        from scripts.local_navigation_experiment import report
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output, reference = root/"new", root/"reference"
            output.mkdir()
            for folder in (output/"ev3-box-local",reference/"ev3-box-astra"):
                folder.mkdir(parents=True)
                (folder/"summary.json").write_text(json.dumps({"clean":True}))
            weights = root/"weights.json"
            weights.write_text(json.dumps({"method":METHOD}))
            with patch("builtins.print"):
                report(SimpleNamespace(output=output,reference_root=reference,weights=weights))
            result = json.loads((output/"comparison.json").read_text())
            weights.write_text(json.dumps({"method":REPAIR_METHOD}))
            with patch("builtins.print"):
                report(SimpleNamespace(output=output,reference_root=reference,weights=weights))
            repair_result = json.loads((output/"comparison.json").read_text())
        self.assertEqual(result["local_policy_method"],METHOD)
        self.assertTrue(result["astra_reference_reused"])
        self.assertEqual(result["clean_astra"],1)
        self.assertEqual(result["clean_local"],1)
        self.assertTrue(result["results"][1]["local"]["missing"])
        self.assertFalse(any("Behavioural cloning" in v for v in result["limitations"]))
        self.assertTrue(any("development rerun" in v for v in repair_result["limitations"]))
        self.assertFalse(any("no training or evaluation feedback" in v for v in repair_result["limitations"]))


if __name__ == "__main__":
    unittest.main()
