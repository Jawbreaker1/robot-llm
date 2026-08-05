import threading
import time
import unittest
from unittest import mock

from robot_agent.blast_observation_monitor import BlastObservationMonitor
from robot_agent.blast_observation_monitor import BlastControllerError


class FakeRuntime:
    instances = []

    def __init__(self, *, hub_name):
        self.hub_name = hub_name
        self.connected = False
        self.disconnected = False
        self.closed = False
        self.observe_calls = 0
        self.calls = []
        self.motion_observations = 0
        self.__class__.instances.append(self)

    async def connect(self):
        self.connected = True
        return {
            "type": "ready",
            "protocol_version": 1,
            "motion_enabled": True,
            "robot_id": "blast-01",
            "controller_id": "blast-01.hub",
        }

    async def observe(self):
        self.calls.append(("observe",))
        self.observe_calls += 1
        moving = self.motion_observations > 0
        if moving:
            self.motion_observations -= 1
        return {
            "observed_at_ms": self.observe_calls,
            "battery": {"voltage_mv": 7_800, "current_ma": 120},
            "imu": {"ready": True, "heading_deg": 12},
            "motor_angles_deg": {"left_drive": 10},
            "motion_active": moving,
            "color": "Color.WHITE",
            "distance_mm": 321,
        }

    async def disconnect(self):
        self.disconnected = True

    async def close(self):
        self.closed = True
        self.disconnected = True

    async def stop(self):
        self.calls.append(("stop",))
        self.motion_observations = 0
        return {"stopped": True}

    async def drive_pulse(self, direction):
        self.calls.append(("drive_pulse", direction))
        self.motion_observations = 1
        return {"accepted": True, "direction": direction}

    async def turn_pulse(self, direction):
        self.calls.append(("turn_pulse", direction))
        self.motion_observations = 1
        return {"accepted": True, "direction": direction}

    async def claw_pulse(self, direction):
        self.calls.append(("claw_pulse", direction))
        self.motion_observations = 1
        return {"accepted": True, "direction": direction}

    async def body_pulse(self, direction):
        self.calls.append(("body_pulse", direction))
        self.motion_observations = 1
        return {"accepted": True, "direction": direction}


class FailingRuntime(FakeRuntime):
    async def connect(self):
        raise RuntimeError("device unavailable")


class RecoveringFactory:
    def __init__(self):
        self.calls = 0

    def __call__(self, *, hub_name):
        self.calls += 1
        runtime_type = FailingRuntime if self.calls == 1 else FakeRuntime
        return runtime_type(hub_name=hub_name)


class BlastObservationMonitorTests(unittest.TestCase):
    def setUp(self):
        FakeRuntime.instances = []

    @staticmethod
    def wait_for(monitor, state, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = monitor.snapshot()
            if snapshot["state"] == state:
                return snapshot
            time.sleep(0.005)
        raise AssertionError("BLAST monitor did not reach {}".format(state))

    def test_reuses_one_runtime_and_publishes_detached_observations(self):
        monitor = BlastObservationMonitor(
            hub_name="BLAST-TEST",
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        snapshot = self.wait_for(monitor, "online")
        while snapshot["observation"] is None:
            time.sleep(0.005)
            snapshot = monitor.snapshot()

        self.assertEqual(len(FakeRuntime.instances), 1)
        runtime = FakeRuntime.instances[0]
        self.assertEqual(runtime.hub_name, "BLAST-TEST")
        self.assertTrue(runtime.connected)
        self.assertEqual(snapshot["controller_id"], "blast-01.hub")
        self.assertEqual(snapshot["observation"]["distance_mm"], 321)
        self.assertIsNotNone(snapshot["last_observed_at_unix_ms"])
        snapshot["observation"]["distance_mm"] = 0
        self.assertEqual(
            monitor.snapshot()["observation"]["distance_mm"],
            321,
        )

        monitor.close()

        self.assertTrue(runtime.disconnected)
        self.assertTrue(runtime.closed)
        self.assertEqual(monitor.snapshot()["state"], "stopped")

    def test_close_cancels_a_blocked_observation_and_disconnects(self):
        observing = threading.Event()

        class BlockedRuntime(FakeRuntime):
            async def observe(self):
                observing.set()
                await __import__("asyncio").Event().wait()

        monitor = BlastObservationMonitor(runtime_factory=BlockedRuntime)
        monitor.start()
        self.assertTrue(observing.wait(timeout=1.0))

        started = time.monotonic()
        monitor.close()

        self.assertLess(time.monotonic() - started, 1.0)
        self.assertTrue(FakeRuntime.instances[0].disconnected)
        self.assertEqual(monitor.snapshot()["state"], "stopped")

    def test_serializes_fixed_command_on_the_observer_runtime(self):
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        result = monitor.command("drive_forward")

        self.assertTrue(result["completed"])
        self.assertEqual(result["command"], "drive_forward")
        self.assertFalse(result["observation"]["motion_active"])
        self.assertEqual(len(FakeRuntime.instances), 1)
        self.assertIn(
            ("drive_pulse", "forward"),
            FakeRuntime.instances[0].calls,
        )
        monitor.close()

    def test_command_waits_for_an_inflight_observation(self):
        observing = threading.Event()
        release = threading.Event()

        class SlowObservationRuntime(FakeRuntime):
            async def observe(self):
                if self.observe_calls == 0:
                    observing.set()
                    while not release.is_set():
                        await __import__("asyncio").sleep(0.005)
                return await super().observe()

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=SlowObservationRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        self.assertTrue(observing.wait(timeout=1.0))
        outcomes = []
        command_thread = threading.Thread(
            target=lambda: outcomes.append(
                monitor.command("turn_right")
            )
        )
        command_thread.start()
        time.sleep(0.03)
        self.assertNotIn(
            ("turn_pulse", "right"),
            FakeRuntime.instances[0].calls,
        )

        release.set()
        command_thread.join(timeout=2.0)

        self.assertFalse(command_thread.is_alive())
        self.assertTrue(outcomes[0]["completed"])
        monitor.close()

    def test_timed_out_queued_command_is_never_executed(self):
        observing = threading.Event()
        release = threading.Event()

        class BlockedObservationRuntime(FakeRuntime):
            async def observe(self):
                if self.observe_calls == 0:
                    observing.set()
                    while not release.is_set():
                        await __import__("asyncio").sleep(0.005)
                return await super().observe()

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=BlockedObservationRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        self.assertTrue(observing.wait(timeout=1.0))

        with mock.patch(
            "robot_agent.blast_observation_monitor."
            "COMMAND_TIMEOUT_SECONDS",
            0.05,
        ):
            with self.assertRaises(BlastControllerError) as timeout:
                monitor.command("drive_forward")
        self.assertEqual(
            timeout.exception.code,
            "controller_command_timeout",
        )

        release.set()
        time.sleep(0.1)
        self.assertNotIn(
            ("drive_pulse", "forward"),
            FakeRuntime.instances[0].calls,
        )
        monitor.close()

    def test_nearly_expired_command_is_rejected_before_motor_call(self):
        observing = threading.Event()
        release = threading.Event()

        class SlowObservationRuntime(FakeRuntime):
            async def observe(self):
                if self.observe_calls == 0:
                    observing.set()
                    while not release.is_set():
                        await __import__("asyncio").sleep(0.005)
                return await super().observe()

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=SlowObservationRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        self.assertTrue(observing.wait(timeout=1.0))
        failures = []

        def command():
            try:
                monitor.command("drive_reverse")
            except BlastControllerError as error:
                failures.append(error.code)

        with mock.patch(
            "robot_agent.blast_observation_monitor."
            "COMMAND_TIMEOUT_SECONDS",
            0.3,
        ):
            command_thread = threading.Thread(target=command)
            command_thread.start()
            time.sleep(0.1)
            release.set()
            command_thread.join(timeout=1.0)

        self.assertFalse(command_thread.is_alive())
        self.assertEqual(failures, ["controller_command_timeout"])
        self.assertNotIn(
            ("drive_pulse", "reverse"),
            FakeRuntime.instances[0].calls,
        )
        monitor.close()

    def test_stop_requires_a_fresh_inactive_observation(self):
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        runtime = FakeRuntime.instances[0]
        runtime.motion_observations = 10

        result = monitor.command("stop")

        self.assertTrue(result["completed"])
        self.assertEqual(runtime.calls[-2:], [("stop",), ("observe",)])
        self.assertFalse(result["observation"]["motion_active"])
        monitor.close()

    def test_rejects_unknown_offline_and_parallel_motion_commands(self):
        monitor = BlastObservationMonitor(runtime_factory=FakeRuntime)
        with self.assertRaises(ValueError):
            monitor.command("run_anything")
        with self.assertRaises(BlastControllerError) as offline:
            monitor.command("drive_forward")
        self.assertEqual(offline.exception.code, "controller_unavailable")

        started = threading.Event()
        release = threading.Event()

        class BlockingDriveRuntime(FakeRuntime):
            async def drive_pulse(self, direction):
                started.set()
                while not release.is_set():
                    await __import__("asyncio").sleep(0.005)
                return await super().drive_pulse(direction)

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=BlockingDriveRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        command_thread = threading.Thread(
            target=lambda: monitor.command("drive_forward")
        )
        command_thread.start()
        self.assertTrue(started.wait(timeout=1.0))
        with self.assertRaises(BlastControllerError) as busy:
            monitor.command("turn_left")
        self.assertEqual(busy.exception.code, "controller_busy")
        release.set()
        command_thread.join(timeout=2.0)
        self.assertFalse(command_thread.is_alive())
        monitor.close()

    def test_command_failure_reconnects_before_accepting_more_work(self):
        class BrokenRuntime(FakeRuntime):
            async def claw_pulse(self, direction):
                raise RuntimeError("protocol failed")

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            reconnect_interval_seconds=0.05,
            runtime_factory=BrokenRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        with self.assertRaises(BlastControllerError) as failed:
            monitor.command("claw_open")

        self.assertEqual(failed.exception.code, "controller_command_failed")
        deadline = time.monotonic() + 1.0
        while len(FakeRuntime.instances) < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertGreaterEqual(len(FakeRuntime.instances), 2)
        self.assertTrue(FakeRuntime.instances[0].closed)
        monitor.close()

    def test_close_terminates_an_inflight_command_and_closes_runtime(self):
        started = threading.Event()

        class NeverEndingRuntime(FakeRuntime):
            async def body_pulse(self, direction):
                started.set()
                await __import__("asyncio").Event().wait()

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=NeverEndingRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        failures = []

        def command():
            try:
                monitor.command("body_left")
            except BlastControllerError as error:
                failures.append(error.code)

        command_thread = threading.Thread(target=command)
        command_thread.start()
        self.assertTrue(started.wait(timeout=1.0))

        monitor.close()
        command_thread.join(timeout=1.0)

        self.assertFalse(command_thread.is_alive())
        self.assertEqual(failures, ["controller_unavailable"])
        self.assertTrue(FakeRuntime.instances[0].closed)

    def test_connection_failure_becomes_offline_snapshot(self):
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FailingRuntime,
        )
        monitor.start()
        snapshot = self.wait_for(monitor, "offline")

        self.assertEqual(snapshot["reason_code"], "connection_failed")
        self.assertIsNone(snapshot["observation"])
        monitor.close()

    def test_start_is_single_owner(self):
        release = threading.Event()

        class BlockingRuntime(FakeRuntime):
            async def observe(self):
                while not release.is_set():
                    await __import__("asyncio").sleep(0.005)
                return await super().observe()

        monitor = BlastObservationMonitor(
            runtime_factory=BlockingRuntime,
        )
        monitor.start()
        with self.assertRaisesRegex(RuntimeError, "already started"):
            monitor.start()
        release.set()
        monitor.close()

    def test_reconnects_after_hub_becomes_available(self):
        factory = RecoveringFactory()
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            reconnect_interval_seconds=0.05,
            runtime_factory=factory,
        )
        monitor.start()

        snapshot = self.wait_for(monitor, "online")
        while snapshot["observation"] is None:
            time.sleep(0.005)
            snapshot = monitor.snapshot()

        self.assertEqual(factory.calls, 2)
        self.assertEqual(snapshot["observation"]["distance_mm"], 321)
        monitor.close()


if __name__ == "__main__":
    unittest.main()
