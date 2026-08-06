import threading
import unittest
from types import SimpleNamespace

from robot_agent.blast_episode_adapter import (
    BlastEpisodeError,
    BlastEpisodeRuntimeAdapter,
)
from robot_agent.blast_observation_monitor import BlastControllerError
from robot_agent.lm_studio_controller_action import (
    COMPLETE,
    ControllerActionDecision,
    ControllerActionPlannerResult,
)


def decision(action, *, plan=(), assessment="ok"):
    return ControllerActionPlannerResult(
        decision=ControllerActionDecision(
            action=action,
            confidence_milli=900,
            assessment=assessment,
            plan=tuple(plan),
            utterance=None,
        ),
        latency_ms=12,
    )


class FakeController:
    def __init__(self, distance_mm=500):
        self.commands = []
        self.snapshot_value = {
            "robot_id": "blast-01",
            "controller_id": "blast-01.hub",
            "state": "online",
            "last_observed_at_unix_ms": 1_000,
            "last_observed_at_monotonic_ms": 1_000,
            "observation": {
                "distance_mm": distance_mm,
                "motion_active": False,
                "imu": {"ready": True, "heading_deg": 0},
            },
        }

    def snapshot(self):
        return self.snapshot_value

    def command(self, command, *, cancel_requested=None):
        if cancel_requested is not None and cancel_requested():
            raise BlastControllerError(
                "controller_command_interrupted",
                "cancelled",
            )
        self.commands.append(command)
        observation = dict(self.snapshot_value["observation"])
        if command == "drive_forward":
            observation["distance_mm"] -= 45
        self.snapshot_value = {
            **self.snapshot_value,
            "observation": observation,
        }
        return {
            "command": command,
            "completed": True,
            "observation": observation,
            "observation_settled": True,
        }


class Planner:
    def __init__(self, decisions):
        self.decisions = list(decisions)
        self.contexts = []

    def decide(self, context):
        self.contexts.append(context)
        return self.decisions.pop(0)


def episode_context():
    updates = []
    value = SimpleNamespace(
        episode_id="episode-1",
        request=SimpleNamespace(goal="Approach the obstacle", locale="en"),
        settings=SimpleNamespace(model="local/model"),
        stop_requested=threading.Event(),
        emergency_stop_requested=threading.Event(),
        publish=updates.append,
    )
    return value, updates


class BlastEpisodeRuntimeAdapterTests(unittest.TestCase):
    def adapter(self, controller, planner, **changes):
        return BlastEpisodeRuntimeAdapter(
            controller=controller,
            planner_factory=lambda _model: planner,
            monotonic_ms=lambda: 1_000,
            **changes,
        )

    def test_replans_after_each_bounded_action_and_completes(self):
        controller = FakeController(500)
        planner = Planner([
            decision(
                "ADVANCE",
                plan=("ADVANCE", COMPLETE),
                assessment="Move one bounded pulse.",
            ),
            decision(
                COMPLETE,
                assessment="The requested state is reached.",
            ),
        ])
        context, updates = episode_context()

        result = self.adapter(controller, planner).run(context)

        self.assertTrue(result.completed)
        self.assertEqual(result.terminal_reason, "completed")
        self.assertEqual(controller.commands, ["drive_forward"])
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(
            planner.contexts[1].history[0]["action"],
            "ADVANCE",
        )
        self.assertTrue(
            planner.contexts[1].history[0]["observation_settled"]
        )
        self.assertEqual(
            planner.contexts[1].observation["sensors"]["distance_mm"],
            455,
        )
        self.assertEqual(updates[0]["plan"], ["ADVANCE", COMPLETE])

    def test_close_obstacle_removes_only_forward_action(self):
        controller = FakeController(100)
        planner = Planner([decision(COMPLETE, assessment="Already close")])
        context, _updates = episode_context()

        self.adapter(controller, planner).run(context)

        self.assertNotIn(
            "ADVANCE",
            planner.contexts[0].available_actions,
        )
        self.assertIn("REVERSE", planner.contexts[0].available_actions)

    def test_stop_wins_after_planning_but_before_motor_start(self):
        controller = FakeController()
        context, _updates = episode_context()

        class CancellingPlanner:
            def decide(self, _context):
                context.stop_requested.set()
                return decision("ADVANCE", plan=("ADVANCE",))

        result = self.adapter(
            controller,
            CancellingPlanner(),
        ).run(context)

        self.assertEqual(result.terminal_reason, "stopped")
        self.assertFalse(result.completed)
        self.assertEqual(controller.commands, [])

    def test_request_stop_uses_same_controller_owner(self):
        controller = FakeController()
        entered = threading.Event()
        release = threading.Event()
        context, _updates = episode_context()

        class BlockingPlanner:
            def decide(self, _context):
                entered.set()
                release.wait(2)
                return decision(COMPLETE, assessment="done")

        adapter = self.adapter(controller, BlockingPlanner())
        result = []
        worker = threading.Thread(target=lambda: result.append(
            adapter.run(context)
        ))
        worker.start()
        self.assertTrue(entered.wait(1))

        context.stop_requested.set()
        adapter.request_stop()
        release.set()
        worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(controller.commands, ["stop"])
        self.assertEqual(result[0].terminal_reason, "stopped")

    def test_rejects_stale_or_nonidle_observation(self):
        for changes, code in (
            ({"last_observed_at_monotonic_ms": 1}, "blast_observation_stale"),
            (
                {"observation": {"distance_mm": 500, "motion_active": True}},
                "blast_motion_not_idle",
            ),
        ):
            controller = FakeController()
            controller.snapshot_value = {
                **controller.snapshot_value,
                **changes,
            }
            adapter = self.adapter(
                controller,
                Planner([decision(COMPLETE)]),
                max_observation_age_ms=500,
            )
            with self.subTest(code=code), self.assertRaises(
                BlastEpisodeError
            ) as raised:
                adapter.run(episode_context()[0])
            self.assertEqual(raised.exception.code, code)


if __name__ == "__main__":
    unittest.main()
