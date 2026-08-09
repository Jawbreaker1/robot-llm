import threading
import time
import unittest
from unittest import mock

from robot_agent.blast_observation_monitor import (
    RANGE_STATE_INVALID,
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    SCAN_COMMAND,
    SCAN_RESULT_SCHEMA,
    BlastControllerError,
    BlastObservationMonitor,
    blast_range_state,
)


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
            "imu": {
                "ready": True,
                "heading_deg": 12,
                "raw_tilt_deg": [0.0, 0.0],
            },
            "motor_angles_deg": {"left_drive": 10, "body": 158},
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

    def test_scan_front_arc_is_one_atomic_gyro_measured_command(self):
        class ScanningRuntime(FakeRuntime):
            def __init__(self, *, hub_name):
                super().__init__(hub_name=hub_name)
                self.heading = 179.0
                self.distance = 300
                self.turn_index = 0

            async def observe(self):
                observation = await super().observe()
                observation["imu"]["heading_deg"] = self.heading
                observation["distance_mm"] = self.distance
                return observation

            async def turn_pulse(self, direction):
                receipt = await super().turn_pulse(direction)
                expected = (
                    "left",
                    "left",
                    "right",
                    "right",
                    "right",
                    "right",
                    "left",
                    "left",
                )
                self.assert_direction(direction, expected[self.turn_index])
                delta = (
                    -22.0,
                    -23.0,
                    23.0,
                    22.0,
                    24.0,
                    23.0,
                    -22.0,
                    -23.0,
                )[self.turn_index]
                self.heading = (
                    self.heading
                    + delta
                    + 180.0
                ) % 360.0 - 180.0
                self.distance = (
                    720,
                    2_000,
                    720,
                    300,
                    1_100,
                    2_000,
                    1_100,
                    310,
                )[self.turn_index]
                self.turn_index += 1
                return receipt

            @staticmethod
            def assert_direction(actual, expected):
                if actual != expected:
                    raise AssertionError("unexpected scan direction")

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=ScanningRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        result = monitor.command(SCAN_COMMAND)

        runtime = FakeRuntime.instances[0]
        self.assertEqual(
            [call for call in runtime.calls if call[0] == "turn_pulse"],
            [
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
            ],
        )
        self.assertEqual(result["command"], SCAN_COMMAND)
        self.assertEqual(result["receipt"], {"turn_count": 8})
        self.assertEqual(result["observation"]["imu"]["heading_deg"], -179.0)
        scan = result["scan"]
        self.assertEqual(scan["schema"], SCAN_RESULT_SCHEMA)
        self.assertEqual(
            [ray["side"] for ray in scan["rays"]],
            [
                "center",
                "left_near",
                "left_far",
                "right_near",
                "right_far",
            ],
        )
        self.assertEqual(
            [ray["distance_mm"] for ray in scan["rays"]],
            [300.0, 720.0, 2_000.0, 1_100.0, 2_000.0],
        )
        self.assertEqual(
            [ray["range_state"] for ray in scan["rays"]],
            [
                RANGE_STATE_MEASURED,
                RANGE_STATE_MEASURED,
                RANGE_STATE_NO_VALID_DISTANCE,
                RANGE_STATE_MEASURED,
                RANGE_STATE_NO_VALID_DISTANCE,
            ],
        )
        self.assertEqual(
            [ray["body_motor_angle_deg"] for ray in scan["rays"]],
            [158] * 5,
        )
        self.assertEqual(
            [ray["relative_heading_deg"] for ray in scan["rays"]],
            [0.0, -22.0, -45.0, 24.0, 47.0],
        )
        self.assertEqual(scan["restoration_error_deg"], 2.0)
        self.assertTrue(scan["restoration_verified"])
        self.assertTrue(scan["all_observations_settled"])
        monitor.close()

    def test_scan_range_state_distinguishes_no_return_from_invalid(self):
        self.assertEqual(blast_range_state(1_999), RANGE_STATE_MEASURED)
        self.assertEqual(
            blast_range_state(2_000),
            RANGE_STATE_NO_VALID_DISTANCE,
        )
        for value in (None, True, -1, 2_001, float("nan"), float("inf")):
            with self.subTest(value=value):
                self.assertEqual(blast_range_state(value), RANGE_STATE_INVALID)

    def test_scan_checks_settled_center_before_first_turn(self):
        for distance, body in (
            (40, 158),
            (53, 158),
            (2_000, 158),
            (300, 157),
        ):
            with self.subTest(distance=distance, body=body):
                class UnsafeScanRuntime(FakeRuntime):
                    async def observe(self):
                        observation = await super().observe()
                        observation["distance_mm"] = distance
                        observation["motor_angles_deg"]["body"] = body
                        return observation

                monitor = BlastObservationMonitor(
                    poll_interval_seconds=0.05,
                    runtime_factory=UnsafeScanRuntime,
                )
                monitor.start()
                self.wait_for(monitor, "online")

                with self.assertRaises(BlastControllerError) as raised:
                    monitor.command(SCAN_COMMAND)

                self.assertEqual(
                    raised.exception.code,
                    "scan_start_clearance_unverified",
                )
                runtime = FakeRuntime.instances[-1]
                runtime_count = len(FakeRuntime.instances)
                self.assertEqual(
                    [
                        call for call in runtime.calls
                        if call[0] == "turn_pulse"
                    ],
                    [],
                )
                self.assertEqual(monitor.snapshot()["state"], "online")
                self.assertTrue(monitor.command("drive_forward")["completed"])
                self.assertEqual(len(FakeRuntime.instances), runtime_count)
                self.assertIs(FakeRuntime.instances[-1], runtime)
                monitor.close()

    def test_navigation_command_returns_latest_settled_observation(self):
        class RockingRuntime(FakeRuntime):
            async def drive_pulse(self, direction):
                receipt = await super().drive_pulse(direction)
                self.samples = [
                    (True, 250, 2.0),
                    (False, 244, 1.4),
                    (False, 248, 0.9),
                    (False, 251, 0.3),
                    (False, 250, 0.2),
                    (False, 252, 0.1),
                    (False, 251, 0.1),
                    (False, 251, 0.1),
                ]
                return receipt

            async def observe(self):
                observation = await super().observe()
                if getattr(self, "samples", None):
                    moving, distance, tilt = self.samples.pop(0)
                    observation["motion_active"] = moving
                    observation["distance_mm"] = distance
                    observation["imu"]["raw_tilt_deg"] = [tilt, 0.0]
                return observation

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=RockingRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        result = monitor.command("drive_forward")

        self.assertEqual(result["observation"]["distance_mm"], 251)
        self.assertEqual(
            result["observation"]["imu"]["raw_tilt_deg"],
            [0.1, 0.0],
        )
        self.assertFalse(result["observation"]["motion_active"])
        self.assertTrue(result["observation_settled"])
        self.assertGreaterEqual(FakeRuntime.instances[0].observe_calls, 8)
        monitor.close()

    def test_unsettled_navigation_returns_explicit_quality_flag(self):
        class RockingRuntime(FakeRuntime):
            async def drive_pulse(self, direction):
                receipt = await super().drive_pulse(direction)
                self.motion_observations = 0
                self.after_drive = True
                return receipt

            async def observe(self):
                observation = await super().observe()
                if getattr(self, "after_drive", False):
                    observation["motion_active"] = False
                    observation["distance_mm"] = 200 + self.observe_calls * 20
                return observation

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=RockingRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        with mock.patch(
            "robot_agent.blast_observation_monitor."
            "POST_MOTION_SETTLE_TIMEOUT_SECONDS",
            0.06,
        ):
            result = monitor.command("drive_forward")

        self.assertTrue(result["completed"])
        self.assertFalse(result["observation_settled"])
        self.assertFalse(result["observation"]["motion_active"])
        monitor.close()

    def test_stop_wins_at_the_final_settled_sample(self):
        stable_return_reached = threading.Event()
        allow_final_check = threading.Event()

        class FinalCheckMonitor(BlastObservationMonitor):
            @staticmethod
            def _settling_window_is_stable(samples):
                stable = BlastObservationMonitor._settling_window_is_stable(
                    samples
                )
                if stable:
                    stable_return_reached.set()
                    allow_final_check.wait(timeout=1.0)
                return stable

        monitor = FinalCheckMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        drive_failures = []
        stop_results = []

        def drive():
            try:
                monitor.command("drive_forward")
            except BlastControllerError as error:
                drive_failures.append(error.code)

        drive_thread = threading.Thread(target=drive)
        drive_thread.start()
        self.assertTrue(stable_return_reached.wait(timeout=1.0))
        stop_thread = threading.Thread(
            target=lambda: stop_results.append(monitor.command("stop"))
        )
        stop_thread.start()
        deadline = time.monotonic() + 1.0
        while (
            monitor._preempt_stop_request is None
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)
        allow_final_check.set()
        drive_thread.join(timeout=1.0)
        stop_thread.join(timeout=1.0)

        self.assertFalse(drive_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertEqual(
            drive_failures,
            ["controller_command_interrupted"],
        )
        self.assertTrue(stop_results[0]["completed"])
        monitor.close()

    def test_stop_preempts_navigation_during_post_motion_settling(self):
        settling_started = threading.Event()

        class UnsettledRuntime(FakeRuntime):
            async def drive_pulse(self, direction):
                receipt = await super().drive_pulse(direction)
                self.motion_observations = 0
                self.after_drive = True
                return receipt

            async def observe(self):
                observation = await super().observe()
                if getattr(self, "after_drive", False):
                    observation["motion_active"] = False
                    observation["distance_mm"] = 300 + self.observe_calls * 10
                    settling_started.set()
                return observation

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=UnsettledRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        failures = []

        def drive():
            try:
                monitor.command("drive_forward")
            except BlastControllerError as error:
                failures.append(error.code)

        drive_thread = threading.Thread(target=drive)
        drive_thread.start()
        self.assertTrue(settling_started.wait(timeout=1.0))

        stop_result = monitor.command("stop")
        drive_thread.join(timeout=1.0)

        self.assertFalse(drive_thread.is_alive())
        self.assertEqual(failures, ["controller_command_interrupted"])
        self.assertTrue(stop_result["completed"])
        self.assertIn(("stop",), FakeRuntime.instances[0].calls)
        monitor.close()

    def test_cancelled_agent_command_never_reaches_motor_queue(self):
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        with self.assertRaises(BlastControllerError) as raised:
            monitor.command(
                "drive_forward",
                cancel_requested=lambda: True,
            )

        self.assertEqual(
            raised.exception.code,
            "controller_command_interrupted",
        )
        self.assertNotIn(
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

    def test_stop_preempts_active_command_on_the_owner_runtime(self):
        pulse_started = threading.Event()
        moving_observed = threading.Event()

        class LongMotionRuntime(FakeRuntime):
            async def drive_pulse(self, direction):
                receipt = await super().drive_pulse(direction)
                self.motion_observations = 100
                pulse_started.set()
                return receipt

            async def observe(self):
                observation = await super().observe()
                if observation["motion_active"]:
                    moving_observed.set()
                return observation

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=LongMotionRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        command_failures = []

        def drive():
            try:
                monitor.command("drive_forward")
            except BlastControllerError as error:
                command_failures.append(error.code)

        command_thread = threading.Thread(target=drive)
        command_thread.start()
        self.assertTrue(pulse_started.wait(timeout=1.0))
        self.assertTrue(moving_observed.wait(timeout=1.0))

        stop_result = monitor.command("stop")
        command_thread.join(timeout=1.0)

        self.assertFalse(command_thread.is_alive())
        self.assertEqual(
            command_failures,
            ["controller_command_interrupted"],
        )
        self.assertTrue(stop_result["completed"])
        self.assertFalse(stop_result["observation"]["motion_active"])
        self.assertEqual(len(FakeRuntime.instances), 1)
        runtime = FakeRuntime.instances[0]
        self.assertIn(("stop",), runtime.calls)
        self.assertLess(runtime.calls.index(("stop",)), 100)
        self.assertEqual(monitor.snapshot()["state"], "online")
        monitor.close()

    def test_stop_interrupts_scan_before_any_later_turn(self):
        first_turn_started = threading.Event()

        class InterruptedScanRuntime(FakeRuntime):
            async def turn_pulse(self, direction):
                receipt = await super().turn_pulse(direction)
                self.motion_observations = 100
                first_turn_started.set()
                return receipt

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=InterruptedScanRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        scan_failures = []

        def scan():
            try:
                monitor.command(SCAN_COMMAND)
            except BlastControllerError as error:
                scan_failures.append(error.code)

        scan_thread = threading.Thread(target=scan)
        scan_thread.start()
        self.assertTrue(first_turn_started.wait(timeout=1.0))

        stop_result = monitor.command("stop")
        scan_thread.join(timeout=1.0)

        self.assertFalse(scan_thread.is_alive())
        self.assertEqual(
            scan_failures,
            ["controller_command_interrupted"],
        )
        self.assertTrue(stop_result["completed"])
        runtime = FakeRuntime.instances[0]
        self.assertEqual(
            [call for call in runtime.calls if call[0] == "turn_pulse"],
            [("turn_pulse", "left")],
        )
        self.assertIn(("stop",), runtime.calls)
        monitor.close()

    def test_stop_cancels_a_queued_command_before_motor_start(self):
        claim_gap_open = threading.Event()
        release_claim = threading.Event()

        class ClaimGapMonitor(BlastObservationMonitor):
            async def _service_preempt_stop(self, runtime, generation):
                result = await super()._service_preempt_stop(
                    runtime,
                    generation,
                )
                if not claim_gap_open.is_set():
                    claim_gap_open.set()
                    while not release_claim.is_set():
                        await __import__("asyncio").sleep(0.005)
                return result

        monitor = ClaimGapMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        self.assertTrue(claim_gap_open.wait(timeout=1.0))
        failures = []
        stop_results = []

        def command(name):
            try:
                result = monitor.command(name)
                if name == "stop":
                    stop_results.append(result)
            except BlastControllerError as error:
                failures.append((name, error.code))

        drive_thread = threading.Thread(
            target=command,
            args=("drive_forward",),
        )
        drive_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_command is None and time.monotonic() < deadline:
            time.sleep(0.005)
        stop_thread = threading.Thread(target=command, args=("stop",))
        stop_thread.start()
        release_claim.set()
        drive_thread.join(timeout=1.0)
        stop_thread.join(timeout=1.0)

        self.assertFalse(drive_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertEqual(
            failures,
            [("drive_forward", "controller_command_interrupted")],
        )
        self.assertTrue(stop_results[0]["completed"])
        runtime = FakeRuntime.instances[0]
        self.assertNotIn(("drive_pulse", "forward"), runtime.calls)
        self.assertIn(("stop",), runtime.calls)
        monitor.close()

    def test_duplicate_preemptive_stop_is_rejected_busy(self):
        moving_observed = threading.Event()
        stop_started = threading.Event()
        release_stop = threading.Event()

        class BlockingStopRuntime(FakeRuntime):
            async def drive_pulse(self, direction):
                receipt = await super().drive_pulse(direction)
                self.motion_observations = 100
                return receipt

            async def observe(self):
                observation = await super().observe()
                if observation["motion_active"]:
                    moving_observed.set()
                return observation

            async def stop(self):
                stop_started.set()
                while not release_stop.is_set():
                    await __import__("asyncio").sleep(0.005)
                return await super().stop()

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=BlockingStopRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        drive_failures = []
        stop_results = []

        def drive():
            try:
                monitor.command("drive_forward")
            except BlastControllerError as error:
                drive_failures.append(error.code)

        def stop():
            stop_results.append(monitor.command("stop"))

        drive_thread = threading.Thread(target=drive)
        drive_thread.start()
        self.assertTrue(moving_observed.wait(timeout=1.0))
        stop_thread = threading.Thread(target=stop)
        stop_thread.start()
        self.assertTrue(stop_started.wait(timeout=1.0))

        with self.assertRaises(BlastControllerError) as busy:
            monitor.command("stop")
        self.assertEqual(busy.exception.code, "controller_busy")

        release_stop.set()
        stop_thread.join(timeout=1.0)
        drive_thread.join(timeout=1.0)
        self.assertFalse(stop_thread.is_alive())
        self.assertFalse(drive_thread.is_alive())
        self.assertTrue(stop_results[0]["completed"])
        self.assertEqual(
            drive_failures,
            ["controller_command_interrupted"],
        )
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

    def test_connection_lifecycle_reuses_and_restarts_the_single_owner(self):
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )

        accepted = monitor.connect()
        self.assertIn(accepted["state"], {"connecting", "online"})
        self.wait_for(monitor, "online")
        self.assertEqual(monitor.connect()["state"], "online")
        self.assertEqual(len(FakeRuntime.instances), 1)

        disconnected = monitor.disconnect()
        self.assertEqual(disconnected["state"], "stopped")
        self.assertTrue(FakeRuntime.instances[0].closed)

        retried = monitor.retry()
        self.assertIn(retried["state"], {"connecting", "online"})
        self.wait_for(monitor, "online")
        self.assertEqual(len(FakeRuntime.instances), 2)
        monitor.close()

    def test_disconnect_before_connect_is_truthfully_stopped(self):
        monitor = BlastObservationMonitor(runtime_factory=FakeRuntime)

        snapshot = monitor.disconnect()

        self.assertEqual(snapshot["state"], "stopped")
        self.assertEqual(snapshot["reason_code"], "observer_stopped")
        self.assertEqual(FakeRuntime.instances, [])

    def test_disconnect_closes_command_admission_before_runtime_cleanup(self):
        cleanup_started = threading.Event()
        release_cleanup = threading.Event()

        class SlowCloseRuntime(FakeRuntime):
            async def close(self):
                cleanup_started.set()
                await __import__("asyncio").to_thread(
                    release_cleanup.wait,
                    2.0,
                )
                await super().close()

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=SlowCloseRuntime,
        )
        monitor.connect()
        self.wait_for(monitor, "online")
        disconnect_thread = threading.Thread(target=monitor.disconnect)
        disconnect_thread.start()
        self.assertTrue(cleanup_started.wait(timeout=1.0))

        with self.assertRaises(BlastControllerError) as rejected:
            monitor.command("claw_open")

        self.assertEqual(rejected.exception.code, "controller_unavailable")
        release_cleanup.set()
        disconnect_thread.join(timeout=1.0)
        self.assertFalse(disconnect_thread.is_alive())
        self.assertEqual(monitor.snapshot()["state"], "stopped")

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
