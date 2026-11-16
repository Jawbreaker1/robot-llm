import unittest

from robot_agent.navigation_simulation_scenarios import (
    blast_gemma_validation_scenarios,
    ev3_validation_scenarios,
    navigation_validation_scenarios,
)


class NavigationSimulationScenarioTests(unittest.TestCase):
    def test_ev3_suite_reuses_worlds_without_routes_and_keeps_goals_in_mission_contract(self):
        from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
        from robot_agent.navigation_state import PoseEstimate
        scenarios = {scene.scenario_id: scene for scene in ev3_validation_scenarios()}
        self.assertEqual(set(scenarios), {"open", "front", "front-right", "front-left", "corridor",
            "bend", "staggered", "dead-end", "many-boxes", "maze"})
        self.assertEqual(len(scenarios["many-boxes"].obstacles), 8)
        self.assertEqual(len(scenarios["maze"].obstacles), 5)
        shared = {scene.scenario_id: scene for scene in navigation_validation_scenarios()}
        self.assertEqual(scenarios["staggered"].obstacles, shared["staggered-multi-obstacle-room"].obstacles)
        self.assertEqual(scenarios["dead-end"].obstacles, shared["dead-end-and-backtrack-room"].obstacles)
        for scene in scenarios.values():
            with self.subTest(scene=scene.scenario_id):
                robot, goal = scene.robots[0], scene.goals[0]
                self.assertEqual(len(scene.robots), 1)
                self.assertEqual(robot.robot_id, "ev3")
                self.assertEqual(robot.footprint, EV3RSTORMProfile().hazard_calibration.robot_footprint)
                self.assertEqual(robot.pose.heading_mdeg, 0)
                self.assertEqual(goal.y_mm, robot.pose.y_mm)
                self.assertTrue(1 <= goal.x_mm - robot.pose.x_mm <= 2_000)
                self.assertFalse(hasattr(scene, "route"))
                self.assertFalse(hasattr(scene, "waypoints"))
                self.assertFalse(scene.build()._pose_collides("ev3", PoseEstimate(goal.x_mm, goal.y_mm, 0)))

    def test_every_scenario_contains_both_robots_and_no_prescribed_route(self):
        scenarios = navigation_validation_scenarios()

        self.assertEqual(len(scenarios), 4)
        for scenario in scenarios:
            self.assertEqual(
                {robot.robot_id for robot in scenario.robots},
                {"blast", "ev3"},
            )
            self.assertEqual(
                {goal.robot_id for goal in scenario.goals},
                {"blast", "ev3"},
            )
            self.assertFalse(hasattr(scenario, "route"))
            self.assertFalse(hasattr(scenario, "waypoints"))
            self.assertEqual(
                set(scenario.build().goals),
                {"blast", "ev3"},
            )

    def test_suite_progresses_from_clear_room_to_backtracking(self):
        scenarios = navigation_validation_scenarios()

        self.assertEqual(len(scenarios[0].obstacles), 0)
        self.assertEqual(len(scenarios[1].obstacles), 1)
        self.assertGreaterEqual(len(scenarios[2].obstacles), 4)
        self.assertEqual(
            scenarios[3].scenario_id,
            "dead-end-and-backtrack-room",
        )

    def test_blast_gemma_suite_matches_the_current_mission_without_routes(self):
        scenarios = blast_gemma_validation_scenarios()

        self.assertEqual(len(scenarios), 7)
        self.assertEqual(
            {scenario.scenario_id for scenario in scenarios},
            {
                "blast-box-front",
                "blast-measured-box-and-chair",
                "blast-box-at-side",
                "blast-boxes-both-sides",
                "blast-straight-corridor",
                "blast-bent-corridor",
                "blast-open-floor",
            },
        )
        for scenario in scenarios:
            self.assertEqual(
                {robot.robot_id for robot in scenario.robots},
                {"blast"},
            )
            self.assertEqual(
                [(goal.x_mm, goal.y_mm) for goal in scenario.goals],
                [(800, 0)],
            )
            self.assertFalse(hasattr(scenario, "route"))
            self.assertFalse(hasattr(scenario, "waypoints"))
            self.assertEqual(set(scenario.build().goals), {"blast"})


if __name__ == "__main__":
    unittest.main()
