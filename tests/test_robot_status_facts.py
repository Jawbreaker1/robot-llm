import unittest
import json
from pathlib import Path

from robot_agent.lm_studio_robot_input import MAX_FACTS_BYTES, MAX_INPUT_CHARS
from robot_agent.robot_status_facts import project_robot_status_facts
from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge


class RobotStatusFactsTests(unittest.TestCase):
    def test_recorded_scan_and_dense_obstacle_evidence_fit_conversation_budget(self):
        scan = json.loads((Path(__file__).parent / "fixtures" /
            "blast_open_floor_scan_20260912.json").read_text())["scans"][0]
        scan["planar_projection"] = {"diagnostics": "x" * 20_000}
        objects = [{
            "hypothesis_id": "obstacle-" + str(i), "x_mm": 300, "y_mm": i * 150,
            "provisional": True, "quality": "PROVISIONAL_YAW_ONLY",
            "evidence_count": 100, "observed_at_unix_ms": 1_000,
            "support_points": [{"x_mm": 300, "y_mm": 100}] * 100,
        } for i in range(8)]
        control = {"state": "IDLE", "episode": {
            "goal": "g" * MAX_INPUT_CHARS, "terminal_reason": "completed",
        }, "runtime": {"scan": scan, "message": "Goal reached"}}
        facts = project_robot_status_facts(control, {
            "status": "available", "object_hypotheses": objects,
            "robot_pose": {"x_mm": 841, "y_mm": 27, "heading_mdeg": -17_881},
            "observed_age_ms": 500, "scan_evidence_history": [scan] * 4,
        }, captured_at_unix_ms=1_500)

        self.assertLess(len(json.dumps(facts).encode()), MAX_FACTS_BYTES)
        summary = facts["control"]["runtime"]["scan"]
        self.assertEqual(summary["ray_count"], 16)
        self.assertEqual(len(summary["rays"]), 16)
        self.assertFalse(summary["all_observations_settled"])
        self.assertFalse(summary["rays"][0]["observation_settled"])
        self.assertEqual(summary["rays"][0]["range_state"], "NO_VALID_DISTANCE")
        self.assertIsNone(summary["rays"][0]["distance_mm"])
        self.assertEqual(facts["spatial_map"]["robot_pose"]["heading_deg"], -17.9)
        self.assertNotIn("heading_mdeg", facts["spatial_map"]["robot_pose"])
        self.assertNotIn("planar_projection", summary)
        self.assertNotIn("drive_encoder_delta_deg", summary["rays"][0])
        self.assertNotIn("support_points", facts["spatial_map"]["object_hypotheses"][0])
        self.assertTrue(facts["spatial_map"]["object_hypotheses"][0]["provisional"])
        self.assertEqual(facts["spatial_map"]["observed_age_ms"], 500)
        self.assertIn("planar_projection", scan)  # never mutate the live map

    def test_ev3_summary_preserves_nonmetric_ir_and_scan_boundaries(self):
        facts = project_robot_status_facts({"state": "RUNNING"}, {
            "object_hypotheses": [{
                "hypothesis_id": "ir-1", "x_mm": None, "y_mm": None,
                "geometry_kind": "QUALITATIVE_FORWARD_ENVELOPE",
                "relation": "NEAR_OBSTACLE", "provisional": True,
                "anchor_pose": {"x_mm": 10, "y_mm": 20},
            }],
            "scan_evidence_history": [{
                "status": "complete", "geometry_kind": "ANGULAR_NONMETRIC_IR_SCAN",
                "left_boundary_mdeg": 45_000, "right_boundary_mdeg": -45_000,
                "completed_at_unix_ms": 100, "rays": [{"raw_ir_proximity": 20}],
            }],
        }, captured_at_unix_ms=500)
        obstacle = facts["spatial_map"]["object_hypotheses"][0]
        self.assertIsNone(obstacle["x_mm"])
        self.assertEqual(obstacle["relation"], "NEAR_OBSTACLE")
        scan = facts["spatial_map"]["scan_evidence_history"][0]
        self.assertEqual(scan["geometry_kind"], "ANGULAR_NONMETRIC_IR_SCAN")
        self.assertEqual(scan["left_boundary_mdeg"], 45_000)
        self.assertNotIn("distance_mm", scan)

    def test_projection_keeps_status_and_latest_bounded_map_evidence(self):
        control = {
            "state": "RUNNING",
            "episode": {
                "episode_id": "episode-1",
                "goal": "Explore",
                "locale": "en",
            },
            "runtime": {
                "current_action": "ADVANCE",
                "message": "Taking the right route",
                "total_tokens": 99,
            },
            "private": "not exposed",
        }
        spatial = {
            "observed_age_ms": 750,
            "robot_pose": {"x_mm": 120},
            "qualitative_observations": [
                {"sequence": sequence} for sequence in range(12)
            ],
            "object_hypotheses": [{"id": "box"}],
        }

        facts = project_robot_status_facts(
            control,
            spatial,
            captured_at_unix_ms=5_000,
        )

        self.assertEqual(facts["control"]["state"], "RUNNING")
        self.assertEqual(
            facts["control"]["episode"]["goal"],
            "Explore",
        )
        self.assertEqual(
            facts["control"]["runtime"]["current_action"],
            "ADVANCE",
        )
        self.assertNotIn("private", facts["control"])
        self.assertNotIn("total_tokens", facts["control"]["runtime"])
        self.assertEqual(
            [
                item["sequence"]
                for item in facts["spatial_map"][
                    "qualitative_observations"
                ]
            ],
            list(range(4, 12)),
        )
        self.assertFalse(facts["camera_vision"]["available"])

        control["runtime"]["message"] = "mutated"
        spatial["object_hypotheses"][0]["id"] = "mutated"
        self.assertEqual(
            facts["control"]["runtime"]["message"],
            "Taking the right route",
        )
        self.assertEqual(
            facts["spatial_map"]["object_hypotheses"][0]["id"],
            "box",
        )

    def test_unavailable_map_and_control_are_explicit(self):
        facts = project_robot_status_facts(
            None,
            None,
            captured_at_unix_ms=1,
        )

        self.assertFalse(facts["control"]["available"])
        self.assertFalse(facts["spatial_map"]["available"])
        self.assertEqual(
            facts["spatial_map"]["reason"],
            "not_available",
        )

    def test_initial_physical_bridge_remains_explicitly_unavailable(self):
        bridge = PhysicalSpatialMapBridge(
            robot_id="robot-1",
            controller_instance_id="controller-1",
            clock_ms=lambda: 1_000,
        )

        facts = project_robot_status_facts(
            {"state": "IDLE"},
            bridge.snapshot(),
            captured_at_unix_ms=1_000,
        )

        self.assertFalse(facts["spatial_map"]["available"])
        self.assertEqual(facts["spatial_map"]["status"], "unavailable")
        self.assertEqual(
            facts["spatial_map"]["reason_code"],
            "no_physical_observations",
        )


if __name__ == "__main__":
    unittest.main()
