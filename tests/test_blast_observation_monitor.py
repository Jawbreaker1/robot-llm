import threading
import time
import unittest

from robot_agent.blast_observation_monitor import BlastObservationMonitor


class FakeRuntime:
    instances = []

    def __init__(self, *, hub_name):
        self.hub_name = hub_name
        self.connected = False
        self.disconnected = False
        self.closed = False
        self.observe_calls = 0
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
        self.observe_calls += 1
        return {
            "observed_at_ms": self.observe_calls,
            "battery": {"voltage_mv": 7_800, "current_ma": 120},
            "imu": {"ready": True, "heading_deg": 12},
            "motor_angles_deg": {"left_drive": 10},
            "motion_active": False,
            "color": "Color.WHITE",
            "distance_mm": 321,
        }

    async def disconnect(self):
        self.disconnected = True

    async def close(self):
        self.closed = True
        raise AssertionError("read-only observer must not close the hub program")


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
        self.assertFalse(runtime.closed)
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
