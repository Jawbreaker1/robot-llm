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
from robot_agent.physical_navigation_contract import SCAN_FRONT_ARC


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


class FakeScanController(FakeController):
    def command(self, command, *, cancel_requested=None):
        result = super().command(
            command,
            cancel_requested=cancel_requested,
        )
        if command == "scan_front_arc":
            result["scan"] = {
                "schema": "blast-scan-front-arc/v1",
                "restoration_verified": True,
                "rays": [
                    {"side": "center", "distance_mm": 500.0},
                    {"side": "left_near", "distance_mm": 900.0},
                    {"side": "right_near", "distance_mm": 1_200.0},
                ],
            }
        return result


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
        self.assertEqual(
            planner.contexts[0].observation["navigation_reference"],
            {
                "episode_start_heading_deg": 0.0,
                "current_heading_deg": 0.0,
                "heading_error_deg": 0.0,
            },
        )

    def test_scan_is_one_agent_action_with_stable_heading_reference(self):
        class ScanController(FakeController):
            def __init__(self):
                super().__init__(300)
                self.snapshot_value["observation"]["imu"][
                    "heading_deg"
                ] = 179

            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                observation = {
                    **result["observation"],
                    "imu": {
                        **result["observation"]["imu"],
                        "heading_deg": -179,
                    },
                }
                result["observation"] = observation
                result["scan"] = {
                    "schema": "blast-scan-front-arc/v1",
                    "restoration_verified": True,
                    "rays": [
                        {"side": "center", "distance_mm": 300.0},
                        {"side": "left", "distance_mm": 900.0},
                        {"side": "right", "distance_mm": 1_200.0},
                    ],
                }
                self.snapshot_value = {
                    **self.snapshot_value,
                    "observation": observation,
                }
                return result

        controller = ScanController()
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision(COMPLETE, assessment="The route is understood."),
        ])
        context, updates = episode_context()

        result = self.adapter(controller, planner).run(context)

        self.assertTrue(result.completed)
        self.assertEqual(controller.commands, ["scan_front_arc"])
        scan = planner.contexts[1].history[0]["scan"]
        self.assertTrue(scan["restoration_verified"])
        self.assertEqual(
            [update["scan"] for update in updates if "scan" in update],
            [scan],
        )
        self.assertIn(
            SCAN_FRONT_ARC,
            planner.contexts[0].available_actions,
        )
        self.assertNotIn(
            SCAN_FRONT_ARC,
            planner.contexts[1].available_actions,
        )
        self.assertEqual(
            planner.contexts[1].observation["navigation_reference"],
            {
                "episode_start_heading_deg": 179.0,
                "current_heading_deg": -179.0,
                "heading_error_deg": 2.0,
            },
        )

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
        self.assertIn(
            SCAN_FRONT_ARC,
            planner.contexts[0].available_actions,
        )

    def test_scan_is_hidden_without_heading_or_distance(self):
        for missing in ("heading", "distance"):
            controller = FakeController()
            if missing == "heading":
                controller.snapshot_value["observation"]["imu"].pop(
                    "heading_deg"
                )
            else:
                controller.snapshot_value["observation"].pop("distance_mm")
            planner = Planner([decision(COMPLETE)])

            self.adapter(controller, planner).run(episode_context()[0])

            with self.subTest(missing=missing):
                self.assertNotIn(
                    SCAN_FRONT_ARC,
                    planner.contexts[0].available_actions,
                )

    def test_motion_makes_scan_available_again(self):
        adapter = self.adapter(FakeController(), Planner([]))
        observation = {
            "sensors": {
                "distance_mm": 300,
                "imu": {"heading_deg": 0},
            }
        }

        available = adapter._available_actions(
            observation,
            (
                {"action": SCAN_FRONT_ARC},
                {"action": "TURN_LEFT"},
            ),
        )

        self.assertIn(SCAN_FRONT_ARC, available)

    def test_scan_guided_motion_requires_a_fresh_scan_before_completion(self):
        controller = FakeScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision("TURN_LEFT", plan=("TURN_LEFT",)),
            decision(COMPLETE, assessment="Unreachable invalid completion."),
        ])
        context, _updates = episode_context()

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(context)

        self.assertEqual(raised.exception.code, "blast_planner_action_invalid")
        self.assertTrue(planner.contexts[0].completion_allowed)
        self.assertTrue(planner.contexts[1].completion_allowed)
        self.assertFalse(planner.contexts[2].completion_allowed)
        self.assertIn(
            SCAN_FRONT_ARC,
            planner.contexts[2].available_actions,
        )

    def test_fresh_scan_allows_completion_after_scan_guided_motion(self):
        controller = FakeScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision("TURN_LEFT", plan=("TURN_LEFT",)),
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision(COMPLETE, assessment="Fresh final scan confirms it."),
        ])
        context, _updates = episode_context()

        result = self.adapter(controller, planner).run(context)

        self.assertTrue(result.completed)
        self.assertEqual(
            controller.commands,
            ["scan_front_arc", "turn_left", "scan_front_arc"],
        )
        self.assertTrue(planner.contexts[3].completion_allowed)

    def test_scan_requires_a_structured_controller_result(self):
        controller = FakeController()
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
        ])

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(raised.exception.code, "blast_scan_result_invalid")
        self.assertEqual(controller.commands, ["scan_front_arc"])

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
