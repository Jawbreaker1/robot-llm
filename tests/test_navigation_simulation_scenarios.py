import unittest

from robot_agent.navigation_simulation_scenarios import (
    blast_gemma_validation_scenarios,
    navigation_validation_scenarios,
)


class NavigationSimulationScenarioTests(unittest.TestCase):
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

        self.assertEqual(len(scenarios), 5)
        self.assertEqual(
            {scenario.scenario_id for scenario in scenarios},
            {
                "blast-box-front",
                "blast-box-at-side",
                "blast-boxes-both-sides",
                "blast-straight-corridor",
                "blast-bent-corridor",
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
