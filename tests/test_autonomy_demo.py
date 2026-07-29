from contextlib import redirect_stdout
import io
import unittest
from unittest.mock import patch

from robot_agent.autonomy_contract import INVESTIGATE_OBSERVATION
from robot_agent.autonomy_demo import (
    main,
    run_demo,
    run_range_change_demo,
)
from robot_agent.autonomy_runtime import IDLE_TASK_COMPLETED


class AutonomyDemoTests(unittest.TestCase):
    def test_offline_demo_completes_multiple_self_selected_tasks(self):
        result, plant = run_demo(tasks=3)

        self.assertEqual(result.tasks_completed, 3)
        self.assertEqual(len(result.tasks), 3)
        self.assertTrue(result.terminal_stop_verified)
        self.assertEqual(plant.collision_count, 0)
        self.assertTrue(
            all(
                task.termination == IDLE_TASK_COMPLETED
                for task in result.tasks
            )
        )
        self.assertEqual(plant.applied_pulses[-1].kind, "STOP")

    def test_clear_world_long_run_reaches_boundary_candidates_safely(self):
        result, plant = run_demo(
            tasks=12,
            with_obstacle=False,
        )

        self.assertEqual(result.tasks_completed, 12)
        self.assertEqual(len(result.tasks), 12)
        self.assertTrue(all(task.completed for task in result.tasks))
        self.assertEqual(plant.collision_count, 0)
        self.assertTrue(result.terminal_stop_verified)

    def test_range_change_becomes_typed_investigation_opportunity(self):
        tasks, plant = run_range_change_demo()

        self.assertEqual(len(tasks), 2)
        self.assertTrue(all(task.completed for task in tasks))
        transition = tasks[1].observation
        self.assertEqual(transition.kind, "METRIC_TRANSITION")
        self.assertEqual(transition.previous_subject_id, "demo-box")
        self.assertEqual(transition.current_subject_id, "demo-box")
        self.assertNotEqual(
            transition.previous_value,
            transition.current_value,
        )
        selected = next(
            candidate
            for candidate in tasks[1].candidates
            if candidate.candidate_id
            == tasks[1].selected_candidate_id
        )
        self.assertEqual(
            selected.task_kind,
            INVESTIGATE_OBSERVATION,
        )
        self.assertEqual(plant.collision_count, 0)
        self.assertTrue(tasks[-1].terminal_stop_verified)

    def test_cli_requires_every_requested_task_to_complete(self):
        incomplete, plant = run_demo(tasks=1)

        with patch(
            "robot_agent.autonomy_demo.run_demo",
            return_value=(incomplete, plant),
        ):
            with redirect_stdout(io.StringIO()):
                exit_code = main(["--tasks", "2", "--compact"])

        self.assertEqual(exit_code, 1)


if __name__ == "__main__":
    unittest.main()
