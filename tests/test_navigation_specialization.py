"""Offline specialization may change weights, not runtime or test conditions."""
import json
import unittest
from unittest.mock import patch
from experiments.navigation_specialization import schedule, specialization_request
from experiments.navigation_set_policy import SetPolicy
from test_navigation_set_policy import specification


class NavigationSpecializationTests(unittest.TestCase):
    def test_author_sees_only_target_profile_and_unchanged_architecture(self):
        for robot, name in (("ev3","EV3"),("blast","Blast")):
            with self.subTest(robot=robot), patch.object(SetPolicy,"scores",side_effect=AssertionError("inference")):
                request = specialization_request(robot,specification())
            contract = json.loads(request["messages"][1]["content"])
            self.assertEqual(list(contract["robots"]),[name])
            self.assertEqual(contract["architecture"]["head_units"],32)
            self.assertEqual(contract["architecture"]["set_units"],16)
            self.assertEqual(request["reasoning_effort"],"high")
            self.assertEqual(contract["repair_evidence"]["previous_specification"],specification())
            self.assertNotIn("cases",contract["repair_evidence"])
            self.assertTrue(any("REQUIRED motion" in s for s in contract["execution_contract"]))
            self.assertIn('goal/waypoint geometry',contract['task'])
            self.assertIn('memory leg/pose XY',contract['audit_findings']['zero_connections'])
            self.assertIn('score AND action sensitivity',contract['geometry_objective'])
        with self.assertRaises(ValueError):
            specialization_request("unknown",specification())

    def test_same_scenes_repeats_and_budget_independent_of_variant(self):
        jobs = schedule()
        self.assertEqual(len(jobs),16)
        self.assertEqual(len({j["name"] for j in jobs}),16)
        for robot in ("ev3","blast"):
            for variant in ("shared","specialized"):
                selected = [(j["scene"],j["repeat"]) for j in jobs
                            if j["robot"] == robot and j["variant"] == variant]
                self.assertEqual(selected,[("box",1),("bend",1),("many-boxes",1),("many-boxes",2)])
        self.assertEqual(jobs[0]["variant"],"shared")
        self.assertEqual(jobs[2]["variant"],"specialized")


if __name__ == "__main__":
    unittest.main()
