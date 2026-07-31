import itertools
import threading
import time
import unittest

from robot_agent.robot_control_contract import (
    DISABLED,
    FAULTED,
    IDLE,
    RUNNING,
    STARTING,
    STOPPING,
    RobotControlSettings,
)
from robot_agent.robot_control_service import (
    RobotControlService,
    RobotControlServiceError,
)


class FakeRuntimeAdapter:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.contexts = []
        self.stop_calls = 0
        self.emergency_calls = 0
        self.exited = threading.Event()
        self.raise_error = None
        self.invalid_update = None

    def run(self, context):
        try:
            self.contexts.append(context)
            self.entered.set()
            context.publish(
                {
                    "current_action": "advance",
                    "plan": ["advance", "scan"],
                    "obstacle": {"target_id": "obstacle-1"},
                    "scan": {"state": "pending"},
                    "model_latency_ms": 42,
                    "speech_status": "generating",
                }
            )
            if self.invalid_update is not None:
                context.publish(self.invalid_update)
            if self.raise_error is not None:
                raise self.raise_error
            while (
                not self.release.wait(0.005)
                and not context.stop_requested.is_set()
            ):
                pass
            return {
                "current_action": "stop",
                "speech_status": "completed",
            }
        finally:
            self.exited.set()

    def request_stop(self):
        self.stop_calls += 1

    def emergency_stop(self):
        self.emergency_calls += 1
        self.release.set()


class BlockingEmergencyAdapter(FakeRuntimeAdapter):
    def __init__(self):
        super().__init__()
        self.emergency_entered = threading.Event()
        self.finish_emergency = threading.Event()

    def emergency_stop(self):
        self.emergency_calls += 1
        self.emergency_entered.set()
        self.finish_emergency.wait(1.0)
        self.release.set()


class FailingEmergencyAdapter(FakeRuntimeAdapter):
    """Keep the runner alive after an emergency transport failure."""

    def run(self, context):
        try:
            self.contexts.append(context)
            self.entered.set()
            self.release.wait(1.0)
            return {"current_action": "stop"}
        finally:
            self.exited.set()

    def emergency_stop(self):
        self.emergency_calls += 1
        raise RuntimeError("emergency transport unavailable")


class DeterministicValues:
    def __init__(self):
        self.ids = itertools.count(1)
        self.clocks = itertools.count(1_000)

    def identifier(self):
        return "id{}".format(next(self.ids))

    def clock(self):
        return next(self.clocks)


def wait_for_state(service, expected, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = service.status()
        if value["state"] == expected:
            return value
        time.sleep(0.005)
    raise AssertionError(
        "state did not become {}: {}".format(expected, service.status())
    )


class RobotControlServiceTests(unittest.TestCase):
    def make_service(self, adapter=None, **kwargs):
        values = DeterministicValues()
        service = RobotControlService(
            adapter,
            clock_ms=values.clock,
            id_factory=values.identifier,
            **kwargs
        )
        self.addCleanup(service.shutdown, 0.2)
        return service

    def start(self, service, request_id="request-1", revision=1):
        return service.start(
            "Utforska rummet och navigera runt hinder.",
            "sv",
            request_id,
            revision,
        )

    def test_without_adapter_is_explicitly_disabled(self):
        service = self.make_service()
        self.assertEqual(service.status()["state"], DISABLED)
        self.assertFalse(service.status()["enabled"])
        with self.assertRaises(RobotControlServiceError) as raised:
            self.start(service)
        self.assertEqual(raised.exception.status, 503)
        self.assertEqual(
            raised.exception.code,
            "robot_control_disabled",
        )
        self.assertEqual(service.stop()["state"], DISABLED)

    def test_start_runs_one_episode_and_publishes_typed_status(self):
        adapter = FakeRuntimeAdapter()
        service = self.make_service(adapter)

        accepted = self.start(service)
        self.assertEqual(accepted["control"]["state"], STARTING)
        self.assertFalse(accepted["idempotent"])
        self.assertTrue(adapter.entered.wait(1.0))
        running = wait_for_state(service, RUNNING)

        self.assertEqual(
            running["runtime"]["current_action"],
            "advance",
        )
        self.assertEqual(
            running["runtime"]["obstacle"]["target_id"],
            "obstacle-1",
        )
        self.assertEqual(
            running["runtime"]["plan"],
            ["advance", "scan"],
        )
        self.assertEqual(running["runtime"]["model_latency_ms"], 42)
        self.assertEqual(
            adapter.contexts[0].settings.revision,
            1,
        )

        adapter.release.set()
        finished = wait_for_state(service, IDLE)
        self.assertEqual(
            finished["episode"]["terminal_reason"],
            "completed",
        )
        self.assertEqual(
            finished["runtime"]["speech_status"],
            "completed",
        )

    def test_same_request_is_idempotent_and_other_episode_is_rejected(self):
        adapter = FakeRuntimeAdapter()
        service = self.make_service(adapter)
        first = self.start(service)
        self.assertTrue(adapter.entered.wait(1.0))

        retry = self.start(service)
        self.assertTrue(retry["idempotent"])
        self.assertEqual(
            retry["accepted_episode_id"],
            first["accepted_episode_id"],
        )
        with self.assertRaises(RobotControlServiceError) as raised:
            self.start(service, request_id="request-2")
        self.assertEqual(raised.exception.code, "robot_episode_active")

        with self.assertRaises(RobotControlServiceError) as conflict:
            service.start(
                "Ett annat mål.",
                "sv",
                "request-1",
                1,
            )
        self.assertEqual(
            conflict.exception.code,
            "robot_idempotency_conflict",
        )
        adapter.release.set()

    def test_settings_are_revisioned_and_idle_only(self):
        adapter = FakeRuntimeAdapter()
        service = self.make_service(adapter)

        updated = service.update_settings(
            1,
            {
                "model": "mlx-community/gemma-4-26b-a4b-it",
                "speech_enabled": False,
            },
        )
        self.assertEqual(updated["revision"], 2)
        self.assertEqual(
            updated["model"],
            "mlx-community/gemma-4-26b-a4b-it",
        )
        self.assertEqual(
            service.status()["runtime"]["speech_status"],
            "disabled",
        )
        with self.assertRaises(RobotControlServiceError) as revision:
            service.update_settings(1, {"speech_enabled": True})
        self.assertEqual(
            revision.exception.code,
            "robot_settings_revision_conflict",
        )

        self.start(service, revision=2)
        self.assertTrue(adapter.entered.wait(1.0))
        with self.assertRaises(RobotControlServiceError) as active:
            service.update_settings(2, {"speech_enabled": True})
        self.assertEqual(active.exception.code, "robot_not_idle")
        adapter.release.set()

    def test_stop_is_idempotent_and_signals_background_runner(self):
        adapter = FakeRuntimeAdapter()
        service = self.make_service(adapter)
        self.start(service)
        self.assertTrue(adapter.entered.wait(1.0))
        wait_for_state(service, RUNNING)

        stopping = service.stop()
        self.assertEqual(stopping["state"], STOPPING)
        again = service.stop()
        self.assertIn(again["state"], (STOPPING, IDLE))
        self.assertEqual(adapter.stop_calls, 1)
        finished = wait_for_state(service, IDLE)
        self.assertEqual(
            finished["episode"]["terminal_reason"],
            "stopped",
        )
        self.assertEqual(service.stop()["state"], IDLE)

    def test_emergency_stop_calls_adapter_outside_runner(self):
        adapter = FakeRuntimeAdapter()
        service = self.make_service(adapter)
        self.start(service)
        self.assertTrue(adapter.entered.wait(1.0))

        value = service.emergency_stop()
        self.assertEqual(adapter.emergency_calls, 1)
        self.assertIn(value["state"], (STOPPING, IDLE))
        finished = wait_for_state(service, IDLE)
        self.assertEqual(
            finished["episode"]["terminal_reason"],
            "emergency_stopped",
        )

    def test_emergency_stop_blocks_new_episode_until_signal_returns(self):
        adapter = BlockingEmergencyAdapter()
        service = self.make_service(adapter)
        self.start(service)
        self.assertTrue(adapter.entered.wait(1.0))

        result = []
        command = threading.Thread(
            target=lambda: result.append(service.emergency_stop()),
        )
        command.start()
        self.assertTrue(adapter.emergency_entered.wait(1.0))
        self.assertEqual(service.status()["state"], STOPPING)
        with self.assertRaises(RobotControlServiceError) as raised:
            self.start(service, request_id="request-2")
        self.assertEqual(raised.exception.code, "robot_episode_active")

        adapter.finish_emergency.set()
        command.join(1.0)
        self.assertFalse(command.is_alive())
        self.assertIn(result[0]["state"], (STOPPING, IDLE))
        finished = wait_for_state(service, IDLE)
        self.assertEqual(
            finished["episode"]["terminal_reason"],
            "emergency_stopped",
        )

    def test_invalid_runtime_update_faults_episode(self):
        adapter = FakeRuntimeAdapter()
        adapter.invalid_update = {"host_command": "ssh robot"}
        service = self.make_service(adapter)
        self.start(service)

        faulted = wait_for_state(service, FAULTED)
        self.assertEqual(
            faulted["last_error_code"],
            "invalid_robot_runtime_update_fields",
        )
        self.assertTrue(adapter.exited.wait(1.0))
        reset = service.stop()
        self.assertEqual(reset["state"], IDLE)

    def test_failed_emergency_stop_stays_faulted_after_runner_finishes(self):
        adapter = FailingEmergencyAdapter()
        service = self.make_service(adapter)
        self.start(service)
        self.assertTrue(adapter.entered.wait(1.0))

        with self.assertRaises(RobotControlServiceError) as raised:
            service.emergency_stop()
        self.assertEqual(
            raised.exception.code,
            "robot_emergency_stop_failed",
        )
        faulted = service.status()
        self.assertEqual(faulted["state"], FAULTED)
        self.assertEqual(
            faulted["last_error_code"],
            "robot_emergency_stop_failed",
        )

        with self.assertRaises(RobotControlServiceError) as blocked:
            self.start(service, request_id="request-2")
        self.assertEqual(blocked.exception.code, "robot_episode_active")

        # A normal stop may still help the runtime unwind, but cannot erase
        # an unacknowledged emergency-stop fault while its runner is active.
        still_faulted = service.stop()
        self.assertEqual(still_faulted["state"], FAULTED)
        self.assertEqual(adapter.stop_calls, 1)
        adapter.release.set()
        self.assertTrue(adapter.exited.wait(1.0))
        self.assertEqual(service.status()["state"], FAULTED)
        self.assertEqual(
            service.status()["last_error_code"],
            "robot_emergency_stop_failed",
        )

        acknowledged = service.stop()
        self.assertEqual(acknowledged["state"], IDLE)
        self.assertEqual(
            acknowledged["episode"]["terminal_reason"],
            "fault_acknowledged",
        )
        restarted = self.start(service, request_id="request-2")
        self.assertEqual(restarted["control"]["state"], STARTING)

    def test_events_and_snapshots_are_bounded_and_report_gaps(self):
        adapter = FakeRuntimeAdapter()
        service = self.make_service(
            adapter,
            event_capacity=3,
            snapshot_capacity=2,
        )
        service.update_settings(1, {"speech_enabled": False})
        service.update_settings(2, {"speech_enabled": True})
        service.update_settings(3, {"speech_enabled": False})

        events = service.events(0, 100)
        snapshots = service.snapshots(0, 100)
        self.assertEqual(len(events["events"]), 3)
        self.assertTrue(events["gap"])
        self.assertGreater(events["dropped_total"], 0)
        self.assertEqual(len(snapshots["snapshots"]), 2)
        self.assertTrue(snapshots["gap"])
        self.assertGreater(snapshots["dropped_total"], 0)

    def test_start_rejects_wrong_settings_revision(self):
        adapter = FakeRuntimeAdapter()
        service = self.make_service(
            adapter,
            settings=RobotControlSettings(revision=4),
        )
        with self.assertRaises(RobotControlServiceError) as raised:
            self.start(service, revision=3)
        self.assertEqual(
            raised.exception.code,
            "robot_settings_revision_conflict",
        )

    def test_model_identifier_rejects_embedded_whitespace(self):
        service = self.make_service(FakeRuntimeAdapter())
        with self.assertRaises(RobotControlServiceError) as raised:
            service.update_settings(
                1,
                {"model": "gemma\nother-model"},
            )
        self.assertEqual(
            raised.exception.code,
            "invalid_robot_model",
        )


if __name__ == "__main__":
    unittest.main()
