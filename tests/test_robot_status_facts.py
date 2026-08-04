import unittest

from robot_agent.robot_status_facts import project_robot_status_facts
from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge


class RobotStatusFactsTests(unittest.TestCase):
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
