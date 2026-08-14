from concurrent.futures import Future
import threading
import time
import unittest
from unittest import mock

from robot_agent.blast_ble_runtime import (
    _adpcm_sample_count,
    _fletcher16,
    blast_adpcm_duration_ms,
)

from robot_agent.blast_observation_monitor import (
    RANGE_STATE_INVALID,
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    SCAN_COMMAND,
    SCAN_COMMAND_TIMEOUT_SECONDS,
    SCAN_INTERNAL_COMMAND_TIMEOUT_SECONDS,
    SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
    SCAN_PULSE_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
    SCAN_RAY_EVIDENCE_SETTLED,
    SCAN_RAY_EVIDENCE_SWEEP_ONLY,
    SCAN_RESULT_SCHEMA,
    SETTLED_OBSERVATION_COMMAND,
    BlastControllerError,
    BlastObservationMonitor,
    _BlastNoReturnScanPermit,
    blast_range_state,
    validate_blast_scan_ray_contract,
)
from robot_agent.blast_navigation_action_profile import (
    SCAN_TURN_ENCODER_DEGREES_PER_PULSE,
    TURN_SPEED_DPS,
)
from robot_agent.blast_scan_safety import issue_blast_scan_permit
from robot_agent.blast_pcm_upload import BlastPCMDeadline
from robot_agent.physical_navigation_contract import SCAN_FRONT_ARC
from robot_agent.physical_odometry import PhysicalPose


def adpcm_block(sample_count):
    payload = bytearray(7 + sample_count // 2)
    payload[3] = sample_count & 0xFF
    payload[4] = sample_count >> 8 & 0xFF
    payload[5] = sample_count >> 16 & 0xFF
    payload[6] = sample_count >> 24
    return bytes(payload)


def adpcm_block_for_size(byte_count):
    return adpcm_block((byte_count - 7) * 2)


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
        self.pcm_sample_count = None
        self.left_drive_angle = 10
        self.right_drive_angle = 10
        self.__class__.instances.append(self)

    @property
    def sampled_audio_aligned(self):
        return True

    async def connect(self):
        self.connected = True
        return {
            "type": "ready",
            "protocol_version": 1,
            "motion_enabled": True,
            "robot_id": "blast-01",
            "controller_id": "blast-01.hub",
            "capabilities": {
                "sampled_audio_v5": {
                    "sample_rate_hz": 16000,
                    "encoding": "ima_adpcm4_mono_stream_v1",
                    "max_bytes": 64007,
                    "transport": "app_data_v1",
                    "checksum": "fletcher16",
                }
            },
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
            "motor_angles_deg": {
                "left_drive": self.left_drive_angle,
                "right_drive": self.right_drive_angle,
                "body": 158,
            },
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

    async def scan_turn_pulse(self, direction):
        before = {
            "left_drive": self.left_drive_angle,
            "right_drive": self.right_drive_angle,
        }
        await self.turn_pulse(direction)
        sign = 1 if direction == "left" else -1
        self.left_drive_angle -= (
            sign * SCAN_TURN_ENCODER_DEGREES_PER_PULSE
        )
        self.right_drive_angle += (
            sign * SCAN_TURN_ENCODER_DEGREES_PER_PULSE
        )
        return {
            "accepted": True,
            "direction": direction,
            "speed_dps": TURN_SPEED_DPS,
            "wheel_angle_deg": SCAN_TURN_ENCODER_DEGREES_PER_PULSE,
            "before_angles_deg": before,
        }

    async def claw_pulse(self, direction):
        self.calls.append(("claw_pulse", direction))
        self.motion_observations = 1
        return {"accepted": True, "direction": direction}

    async def body_pulse(self, direction):
        self.calls.append(("body_pulse", direction))
        self.motion_observations = 1
        return {"accepted": True, "direction": direction}

    async def begin_pcm(self, payload, *, cancel_requested=None):
        byte_count = len(payload)
        self.pcm_sample_count = _adpcm_sample_count(payload)
        self.calls.append(("begin_pcm", byte_count))
        return {
            "transfer_id": 1,
            "byte_count": byte_count,
            "sample_count": self.pcm_sample_count,
            "fletcher16": _fletcher16(payload),
            "batch_bytes": 8,
        }

    async def write_pcm_batch(
        self, offset, payload, *, cancel_requested=None,
    ):
        self.calls.append(("write_pcm_batch", offset, payload))
        return {"received_bytes": offset + len(payload)}

    async def start_pcm(
        self, transfer_id, byte_count, fletcher16, *, cancel_requested=None,
    ):
        self.calls.append(("start_pcm", byte_count))
        return {
            "transfer_id": transfer_id,
            "byte_count": byte_count,
            "sample_count": self.pcm_sample_count,
            "sample_rate_hz": 16000,
            "encoding": "ima_adpcm4_mono_stream_v1",
            "fletcher16": fletcher16,
            "duration_ms": blast_adpcm_duration_ms(
                self.pcm_sample_count
            ),
        }


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

    def measured_scan_permit(self, monitor):
        snapshot = monitor.snapshot()
        while snapshot["observation"] is None:
            time.sleep(0.005)
            snapshot = monitor.snapshot()
        motors = snapshot["observation"]["motor_angles_deg"]
        permit = issue_blast_scan_permit(
            controller=monitor,
            action=SCAN_FRONT_ARC,
            distance_mm=snapshot["observation"]["distance_mm"],
            geometry_checked=False,
            pose=PhysicalPose(),
            prior_receipt=None,
            expected_drive_angles={
                role: motors[role]
                for role in ("left_drive", "right_drive")
            },
        )
        self.assertIsNotNone(permit)
        return permit

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

    def test_runtime_generation_is_read_only_and_tracks_ble_sessions(self):
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )

        self.assertEqual(monitor.runtime_generation(), 0)
        monitor.connect()
        self.wait_for(monitor, "online")
        self.assertEqual(monitor.runtime_generation(), 1)
        monitor.disconnect()
        self.assertEqual(monitor.runtime_generation(), 1)
        monitor.retry()
        self.wait_for(monitor, "online")
        self.assertEqual(monitor.runtime_generation(), 2)
        monitor.close()

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

    def test_play_pcm_uses_owned_runtime_and_advertised_capability(self):
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        snapshot = self.wait_for(monitor, "online")
        payload = adpcm_block(1)

        result = monitor.play_pcm(payload)

        self.assertIn(
            "sampled_audio_v5",
            snapshot["ready"]["capabilities"],
        )
        self.assertEqual(result["schema"], "controller-sampled-audio-result/v1")
        self.assertTrue(result["started"])
        self.assertFalse(result["completed"])
        self.assertEqual(result["byte_count"], len(payload))
        self.assertEqual(result["duration_ms"], 16)
        self.assertEqual(len(FakeRuntime.instances), 1)
        self.assertIn(("begin_pcm", len(payload)), FakeRuntime.instances[0].calls)
        self.assertIn(("start_pcm", len(payload)), FakeRuntime.instances[0].calls)
        monitor.close()

    def test_pcm_timeout_before_claim_releases_slot_and_discards_request(self):
        claim_open = threading.Event()
        release_claim = threading.Event()

        class DelayedSpeechClaimMonitor(BlastObservationMonitor):
            def _next_speech(self):
                if not release_claim.is_set():
                    claim_open.set()
                    release_claim.wait(timeout=2.0)
                return super()._next_speech()

        monitor = DelayedSpeechClaimMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        self.assertTrue(claim_open.wait(timeout=1.0))
        payload = adpcm_block(1)

        with mock.patch(
            "robot_agent.blast_observation_monitor."
            "SAMPLED_AUDIO_TIMEOUT_SECONDS",
            0.05,
        ):
            with self.assertRaises(BlastControllerError) as timed_out:
                monitor.play_pcm(payload)

        self.assertEqual(timed_out.exception.code, "controller_command_timeout")
        self.assertIsNone(monitor._pending_speech)
        self.assertTrue(monitor._speech_queue.empty())

        release_claim.set()
        second = monitor.play_pcm(payload)

        self.assertTrue(second["started"])
        self.assertEqual(
            FakeRuntime.instances[0].calls.count(("start_pcm", len(payload))),
            1,
        )
        monitor.close()

    def test_pcm_timeout_during_active_batch_allows_next_request(self):
        first_batch_started = threading.Event()
        release_first_batch = threading.Event()

        class BlockingFirstBatchRuntime(FakeRuntime):
            blocked_once = False

            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                self.calls.append(("write_pcm_batch", offset, payload))
                if not self.blocked_once:
                    self.blocked_once = True
                    first_batch_started.set()
                    await __import__("asyncio").to_thread(
                        release_first_batch.wait,
                        2.0,
                    )
                return {"received_bytes": offset + len(payload)}

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=BlockingFirstBatchRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        first_payload = adpcm_block_for_size(18)
        second_payload = adpcm_block_for_size(10)
        first_failures = []
        second_outcomes = []

        with mock.patch(
            "robot_agent.blast_observation_monitor."
            "SAMPLED_AUDIO_TIMEOUT_SECONDS",
            0.05,
        ):
            first_thread = threading.Thread(
                target=lambda: self._capture_failure(
                    first_failures,
                    lambda: monitor.play_pcm(first_payload),
                )
            )
            first_thread.start()
            self.assertTrue(first_batch_started.wait(timeout=1.0))
            first_thread.join(timeout=1.0)

        self.assertFalse(first_thread.is_alive())
        self.assertEqual(len(first_failures), 1)
        self.assertEqual(
            first_failures[0].code,
            "controller_command_timeout",
        )
        self.assertIsNone(monitor._pending_speech)

        second_thread = threading.Thread(
            target=lambda: second_outcomes.append(
                monitor.play_pcm(second_payload)
            )
        )
        second_thread.start()
        deadline = time.monotonic() + 1.0
        while (
            monitor._pending_speech is None
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)
        self.assertIsNotNone(monitor._pending_speech)
        release_first_batch.set()
        second_thread.join(timeout=3.0)

        self.assertFalse(second_thread.is_alive())
        self.assertEqual(len(second_outcomes), 1)
        self.assertTrue(second_outcomes[0]["started"])
        calls = FakeRuntime.instances[0].calls
        self.assertNotIn(("start_pcm", len(first_payload)), calls)
        self.assertIn(("start_pcm", len(second_payload)), calls)
        monitor.close()

    def test_navigation_command_wins_while_speech_waits(self):
        claim_open = threading.Event()
        release_claim = threading.Event()

        class ClaimMonitor(BlastObservationMonitor):
            def _next_command(self):
                if not claim_open.is_set():
                    claim_open.set()
                    release_claim.wait(timeout=2.0)
                return super()._next_command()

        monitor = ClaimMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        self.assertTrue(claim_open.wait(timeout=1.0))
        failures = []

        def speak():
            try:
                monitor.play_pcm(adpcm_block(1))
            except Exception as error:
                failures.append(error)

        def navigate():
            try:
                monitor.command("drive_forward")
            except Exception as error:
                failures.append(error)

        speech_thread = threading.Thread(target=speak)
        command_thread = threading.Thread(target=navigate)
        speech_thread.start()
        command_thread.start()
        deadline = time.monotonic() + 1.0
        while (
            (monitor._pending_speech is None or monitor._pending_command is None)
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)
        release_claim.set()
        speech_thread.join(timeout=3.0)
        command_thread.join(timeout=3.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(command_thread.is_alive())
        self.assertEqual(failures, [])
        calls = FakeRuntime.instances[0].calls
        self.assertLess(
            calls.index(("drive_pulse", "forward")),
            calls.index(("begin_pcm", 7)),
        )
        monitor.close()

    def test_navigation_runs_between_pcm_batches_then_upload_resumes(self):
        first_batch_started = threading.Event()
        release_first_batch = threading.Event()

        class FragmentRuntime(FakeRuntime):
            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                self.calls.append(("write_pcm_batch", offset, payload))
                if offset == 0:
                    first_batch_started.set()
                    await __import__("asyncio").to_thread(
                        release_first_batch.wait,
                        2.0,
                    )
                return {"received_bytes": offset + len(payload)}

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FragmentRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        outcomes = []
        speech_payload = adpcm_block_for_size(18)
        speech_thread = threading.Thread(
            target=lambda: outcomes.append(
                monitor.play_pcm(speech_payload)
            )
        )
        speech_thread.start()
        self.assertTrue(first_batch_started.wait(1.0))
        command_thread = threading.Thread(
            target=lambda: outcomes.append(
                monitor.command("drive_forward")
            )
        )
        command_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_command is None and time.monotonic() < deadline:
            time.sleep(0.005)
        release_first_batch.set()
        speech_thread.join(3.0)
        command_thread.join(3.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(command_thread.is_alive())
        self.assertEqual(len(outcomes), 2)
        calls = FakeRuntime.instances[0].calls
        first = calls.index(("write_pcm_batch", 0, speech_payload[:8]))
        navigation = calls.index(("drive_pulse", "forward"))
        second = calls.index(("write_pcm_batch", 8, speech_payload[8:16]))
        self.assertLess(first, navigation)
        self.assertLess(navigation, second)
        self.assertNotIn(("observe",), calls[first + 1:navigation])
        self.assertIn(("start_pcm", 18), calls)
        monitor.close()

    def test_upload_over_fifteen_simulated_seconds_does_not_timeout_navigation(
        self,
    ):
        first_batch_started = threading.Event()
        release_first_batch = threading.Event()

        class LongBoundedPreloadRuntime(FakeRuntime):
            def __init__(self, *, hub_name):
                super().__init__(hub_name=hub_name)
                self.simulated_upload_seconds = 0.0

            async def begin_pcm(self, payload, *, cancel_requested=None):
                self.simulated_upload_seconds += 4.0
                return await super().begin_pcm(
                    payload,
                    cancel_requested=cancel_requested,
                )

            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                self.calls.append(("write_pcm_batch", offset, payload))
                self.simulated_upload_seconds += 4.0
                if offset == 0:
                    first_batch_started.set()
                    await __import__("asyncio").to_thread(
                        release_first_batch.wait,
                        2.0,
                    )
                else:
                    # Scale a four-second slow-link phase to keep this
                    # scheduler regression fast. Consecutive draining would
                    # still exceed the correspondingly scaled command TTL.
                    await __import__("asyncio").sleep(0.08)
                return {"received_bytes": offset + len(payload)}

            async def start_pcm(
                self,
                transfer_id,
                byte_count,
                fletcher16,
                *,
                cancel_requested=None,
            ):
                await __import__("asyncio").sleep(0.08)
                return await super().start_pcm(
                    transfer_id,
                    byte_count,
                    fletcher16,
                    cancel_requested=cancel_requested,
                )

        class ImmediatelySettledMonitor(BlastObservationMonitor):
            @staticmethod
            def _settling_window_is_stable(samples):
                return bool(samples)

        monitor = ImmediatelySettledMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=LongBoundedPreloadRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        payload = adpcm_block_for_size(42)
        speech_results = []
        command_results = []
        failures = []
        speech_thread = threading.Thread(
            target=lambda: speech_results.append(monitor.play_pcm(payload))
        )
        speech_thread.start()
        self.assertTrue(first_batch_started.wait(timeout=1.0))

        def navigate():
            try:
                command_results.append(monitor.command("drive_forward"))
            except Exception as error:
                failures.append(error)

        with (
            mock.patch(
                "robot_agent.blast_observation_monitor."
                "COMMAND_TIMEOUT_SECONDS",
                0.3,
            ),
            mock.patch(
                "robot_agent.blast_observation_monitor."
                "INTERNAL_COMMAND_TIMEOUT_SECONDS",
                0.25,
            ),
            mock.patch(
                "robot_agent.blast_observation_monitor."
                "MIN_COMMAND_BUDGET_SECONDS",
                0.01,
            ),
            mock.patch(
                "robot_agent.blast_observation_monitor."
                "COMMAND_RESPONSE_MARGIN_SECONDS",
                0.01,
            ),
        ):
            command_thread = threading.Thread(target=navigate)
            command_thread.start()
            deadline = time.monotonic() + 1.0
            while (
                monitor._pending_command is None
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
            self.assertIsNotNone(monitor._pending_command)
            release_first_batch.set()
            speech_thread.join(timeout=3.0)
            command_thread.join(timeout=3.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(command_thread.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(len(speech_results), 1)
        self.assertEqual(len(command_results), 1)
        runtime = FakeRuntime.instances[0]
        self.assertGreater(runtime.simulated_upload_seconds, 15.0)
        calls = runtime.calls
        first = calls.index(("write_pcm_batch", 0, payload[:8]))
        navigation = calls.index(("drive_pulse", "forward"))
        second = calls.index(("write_pcm_batch", 8, payload[8:16]))
        started = calls.index(("start_pcm", len(payload)))
        self.assertLess(first, navigation)
        self.assertLess(navigation, second)
        self.assertLess(second, started)
        uploaded_before_start = b"".join(
            call[2]
            for call in calls[:started]
            if call[0] == "write_pcm_batch"
        )
        self.assertEqual(uploaded_before_start, payload)
        self.assertTrue(all(
            len(call[2]) <= 8
            for call in calls
            if call[0] == "write_pcm_batch"
        ))
        monitor.close()

    def test_over_sixty_second_navigation_stream_cannot_starve_v5_speech(self):
        claim_open = threading.Event()
        release_claim = threading.Event()
        command_count = 62

        class SimulatedClock:
            def __init__(self):
                self.value = 0.0
                self.lock = threading.Lock()

            def __call__(self):
                with self.lock:
                    return self.value

            def advance(self, seconds):
                with self.lock:
                    self.value += seconds

        simulated_clock = SimulatedClock()

        class V5Runtime(FakeRuntime):
            simulated_navigation_seconds = 0
            speech_started_at_simulated_seconds = None

            async def drive_pulse(self, direction):
                self.calls.append(("drive_pulse", direction))
                self.simulated_navigation_seconds += 5
                simulated_clock.advance(5)
                return {"accepted": True, "direction": direction}

            async def begin_pcm(self, payload, *, cancel_requested=None):
                begun = await super().begin_pcm(
                    payload,
                    cancel_requested=cancel_requested,
                )
                # Eight 509-byte AppData frames, the real v5 batch ceiling.
                begun["batch_bytes"] = 4_072
                return begun

            async def start_pcm(
                self,
                transfer_id,
                byte_count,
                fletcher16,
                *,
                cancel_requested=None,
            ):
                self.speech_started_at_simulated_seconds = simulated_clock()
                return await super().start_pcm(
                    transfer_id,
                    byte_count,
                    fletcher16,
                    cancel_requested=cancel_requested,
                )

        class ContinuousNavigationMonitor(BlastObservationMonitor):
            completed_commands = 0

            @staticmethod
            def _settling_window_is_stable(samples):
                return bool(samples)

            @staticmethod
            def _new_speech_deadline():
                return BlastPCMDeadline(
                    inactivity_seconds=60.0,
                    maximum_seconds=15.0 * 60.0,
                    clock=simulated_clock,
                )

            def _next_command(self):
                if not claim_open.is_set():
                    claim_open.set()
                    release_claim.wait(timeout=2.0)
                return super()._next_command()

            async def _execute_command(self, runtime, generation, request):
                completed = await super()._execute_command(
                    runtime,
                    generation,
                    request,
                )
                if request[1] != "drive_forward":
                    return completed
                self.completed_commands += 1
                if self.completed_commands >= command_count:
                    return completed
                # Reproduce a continuously fed navigator deterministically:
                # the next command is already waiting before arbitration.
                deadline = time.monotonic() + 1.0
                while (
                    self._pending_command is None
                    and time.monotonic() < deadline
                ):
                    await __import__("asyncio").sleep(0.001)
                return completed

        monitor = ContinuousNavigationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=V5Runtime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        self.assertTrue(claim_open.wait(timeout=1.0))
        # The exact complete five-second v5 reference utterance size. It must
        # be fully preloaded before start_pcm; no sentence fragments play.
        payload = adpcm_block_for_size(41_060)
        speech_outcomes = []
        navigation_outcomes = []
        failures = []
        speech_thread = threading.Thread(
            target=lambda: speech_outcomes.append(monitor.play_pcm(payload))
        )

        def navigate_continuously():
            try:
                for _ in range(command_count):
                    navigation_outcomes.append(
                        monitor.command("drive_forward")
                    )
            except Exception as error:
                failures.append(error)

        navigation_thread = threading.Thread(target=navigate_continuously)
        speech_thread.start()
        navigation_thread.start()
        deadline = time.monotonic() + 1.0
        while (
            (monitor._pending_speech is None or monitor._pending_command is None)
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)
        release_claim.set()
        speech_thread.join(timeout=5.0)
        navigation_thread.join(timeout=5.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(navigation_thread.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(len(speech_outcomes), 1)
        self.assertTrue(speech_outcomes[0]["started"])
        self.assertEqual(len(navigation_outcomes), command_count)
        runtime = FakeRuntime.instances[0]
        self.assertGreater(runtime.simulated_navigation_seconds, 60)
        self.assertGreater(
            runtime.speech_started_at_simulated_seconds,
            60,
        )
        calls = runtime.calls
        drive_indexes = [
            index for index, call in enumerate(calls)
            if call == ("drive_pulse", "forward")
        ]
        audio_indexes = [
            index for index, call in enumerate(calls)
            if call[0] in {"begin_pcm", "write_pcm_batch", "start_pcm"}
        ]
        self.assertEqual(len(drive_indexes), command_count)
        self.assertEqual(
            len([
                call for call in calls if call[0] == "write_pcm_batch"
            ]),
            11,
        )
        self.assertEqual(len(audio_indexes), 13)
        self.assertLess(
            calls.index(("start_pcm", len(payload))),
            drive_indexes[13],
        )
        for position, audio_index in enumerate(audio_indexes):
            self.assertLess(drive_indexes[position], audio_index)
            self.assertLess(audio_index, drive_indexes[position + 1])
        monitor.close()

    def test_pcm_progress_never_extends_absolute_lifetime_ceiling(self):
        class Clock:
            value = 0.0

            def __call__(self):
                return self.value

        clock = Clock()
        deadline = BlastPCMDeadline(
            inactivity_seconds=10.0,
            maximum_seconds=25.0,
            clock=clock,
        )

        clock.value = 9.0
        self.assertTrue(deadline.record_progress())
        self.assertEqual(deadline.remaining(), 10.0)
        clock.value = 18.0
        self.assertTrue(deadline.record_progress())
        self.assertEqual(deadline.remaining(), 7.0)
        clock.value = 25.0
        self.assertTrue(deadline.expired())
        self.assertTrue(deadline.claim_timeout())
        self.assertFalse(deadline.record_progress())

    def test_completed_future_wins_timeout_cleanup_race(self):
        class CompletionWinsDeadline:
            def remaining(self):
                return 0.0

            def start_in_flight(self):
                return False

            def claim_timeout(self):
                with monitor._lock:
                    result = monitor._pending_speech
                result.set_result({"started": True})
                return True

        class CompletionWinsMonitor(BlastObservationMonitor):
            @staticmethod
            def _new_speech_deadline():
                return CompletionWinsDeadline()

        monitor = CompletionWinsMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        result = monitor.play_pcm(adpcm_block(1))

        self.assertEqual(result, {"started": True})
        monitor.close()

    def test_speech_completion_and_timeout_cancel_are_atomic(self):
        done_entered = threading.Event()
        release_done = threading.Event()

        class PausingFuture(Future):
            def done(self):
                done_entered.set()
                release_done.wait(timeout=2.0)
                return super().done()

        monitor = BlastObservationMonitor(runtime_factory=FakeRuntime)
        result = PausingFuture()
        monitor._pending_speech = result
        finish_failures = []
        cancel_results = []
        finish_thread = threading.Thread(
            target=lambda: self._capture_failure(
                finish_failures,
                lambda: monitor._finish_speech(
                    result,
                    value={"started": True},
                ),
            )
        )
        finish_thread.start()
        self.assertTrue(done_entered.wait(timeout=1.0))

        def cancel_under_monitor_lock():
            with monitor._lock:
                cancel_results.append(result.cancel())

        cancel_thread = threading.Thread(target=cancel_under_monitor_lock)
        cancel_thread.start()
        self.assertTrue(cancel_thread.is_alive())
        release_done.set()
        finish_thread.join(timeout=2.0)
        cancel_thread.join(timeout=2.0)

        self.assertEqual(finish_failures, [])
        self.assertEqual(cancel_results, [False])
        self.assertEqual(result.result(), {"started": True})
        self.assertIsNone(monitor._pending_speech)

    def test_queued_stop_stays_ahead_of_fair_speech_turn(self):
        claim_open = threading.Event()
        release_claim = threading.Event()
        first_command_completed = threading.Event()
        release_first_command = threading.Event()
        cancel = threading.Event()

        class StopBoundaryMonitor(BlastObservationMonitor):
            def _next_command(self):
                if not claim_open.is_set():
                    claim_open.set()
                    release_claim.wait(timeout=2.0)
                return super()._next_command()

            async def _execute_command(self, runtime, generation, request):
                completed = await super()._execute_command(
                    runtime,
                    generation,
                    request,
                )
                if request[1] == "drive_forward":
                    first_command_completed.set()
                    await __import__("asyncio").to_thread(
                        release_first_command.wait,
                        2.0,
                    )
                return completed

        monitor = StopBoundaryMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        self.assertTrue(claim_open.wait(timeout=1.0))
        speech_failures = []
        navigation_results = []
        stop_results = []
        speech_thread = threading.Thread(
            target=lambda: self._capture_failure(
                speech_failures,
                lambda: monitor.play_pcm(
                    adpcm_block_for_size(18),
                    cancel_requested=cancel.is_set,
                ),
            )
        )
        navigation_thread = threading.Thread(
            target=lambda: navigation_results.append(
                monitor.command("drive_forward")
            )
        )
        speech_thread.start()
        navigation_thread.start()
        deadline = time.monotonic() + 1.0
        while (
            (monitor._pending_speech is None or monitor._pending_command is None)
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)
        release_claim.set()
        self.assertTrue(first_command_completed.wait(timeout=2.0))
        cancel.set()
        stop_thread = threading.Thread(
            target=lambda: stop_results.append(monitor.command("stop"))
        )
        stop_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_command != "stop" and time.monotonic() < deadline:
            time.sleep(0.005)
        release_first_command.set()
        speech_thread.join(timeout=3.0)
        navigation_thread.join(timeout=3.0)
        stop_thread.join(timeout=3.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(navigation_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertEqual(len(navigation_results), 1)
        self.assertEqual(len(stop_results), 1)
        self.assertTrue(stop_results[0]["completed"])
        self.assertEqual(len(speech_failures), 1)
        self.assertEqual(
            speech_failures[0].code,
            "controller_command_interrupted",
        )
        calls = FakeRuntime.instances[0].calls
        self.assertLess(
            calls.index(("drive_pulse", "forward")),
            calls.index(("stop",)),
        )
        self.assertFalse(any(call[0] == "begin_pcm" for call in calls))
        monitor.close()

    def test_stop_queued_before_session_loop_discards_older_speech(self):
        online_published = threading.Event()
        release_session_loop = threading.Event()

        class PrequeuedStopMonitor(BlastObservationMonitor):
            def _set_state(self, state, reason_code, **changes):
                super()._set_state(state, reason_code, **changes)
                if state == "online" and not online_published.is_set():
                    online_published.set()
                    release_session_loop.wait(timeout=2.0)

        monitor = PrequeuedStopMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.assertTrue(online_published.wait(timeout=1.0))
        speech_failures = []
        stop_results = []
        speech_thread = threading.Thread(
            target=lambda: self._capture_failure(
                speech_failures,
                lambda: monitor.play_pcm(adpcm_block_for_size(18)),
            )
        )
        speech_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_speech is None and time.monotonic() < deadline:
            time.sleep(0.005)
        stop_thread = threading.Thread(
            target=lambda: stop_results.append(monitor.command("stop"))
        )
        stop_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_command != "stop" and time.monotonic() < deadline:
            time.sleep(0.005)
        release_session_loop.set()
        speech_thread.join(timeout=3.0)
        stop_thread.join(timeout=3.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertEqual(len(speech_failures), 1)
        self.assertEqual(
            speech_failures[0].code,
            "controller_command_interrupted",
        )
        self.assertEqual(len(stop_results), 1)
        self.assertTrue(stop_results[0]["completed"])
        calls = FakeRuntime.instances[0].calls
        self.assertIn(("stop",), calls)
        self.assertFalse(any(call[0] == "begin_pcm" for call in calls))
        self.assertFalse(any(call[0] == "start_pcm" for call in calls))
        monitor.close()

    def test_stop_serviced_inside_navigation_cancels_queued_speech(self):
        drive_started = threading.Event()
        release_drive = threading.Event()

        class PreemptedRuntime(FakeRuntime):
            async def drive_pulse(self, direction):
                self.calls.append(("drive_pulse", direction))
                drive_started.set()
                await __import__("asyncio").to_thread(
                    release_drive.wait,
                    2.0,
                )
                self.motion_observations = 1
                return {"accepted": True, "direction": direction}

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=PreemptedRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        navigation_failures = []
        speech_failures = []
        stop_results = []
        navigation_thread = threading.Thread(
            target=lambda: self._capture_failure(
                navigation_failures,
                lambda: monitor.command("drive_forward"),
            )
        )
        navigation_thread.start()
        self.assertTrue(drive_started.wait(timeout=1.0))
        speech_thread = threading.Thread(
            target=lambda: self._capture_failure(
                speech_failures,
                lambda: monitor.play_pcm(adpcm_block_for_size(18)),
            )
        )
        speech_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_speech is None and time.monotonic() < deadline:
            time.sleep(0.005)
        stop_thread = threading.Thread(
            target=lambda: stop_results.append(monitor.command("stop"))
        )
        stop_thread.start()
        release_drive.set()
        navigation_thread.join(timeout=3.0)
        speech_thread.join(timeout=3.0)
        stop_thread.join(timeout=3.0)

        self.assertEqual(len(navigation_failures), 1)
        self.assertEqual(
            navigation_failures[0].code,
            "controller_command_interrupted",
        )
        self.assertEqual(len(speech_failures), 1)
        self.assertEqual(
            speech_failures[0].code,
            "controller_command_interrupted",
        )
        self.assertEqual(len(stop_results), 1)
        calls = FakeRuntime.instances[0].calls
        self.assertIn(("stop",), calls)
        self.assertFalse(any(call[0] == "begin_pcm" for call in calls))
        monitor.close()

    def test_stop_after_speech_step_claim_waits_only_for_one_bounded_batch(self):
        batch_started = threading.Event()
        release_batch = threading.Event()

        class ClaimedBatchRuntime(FakeRuntime):
            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                self.calls.append(("write_pcm_batch", offset, payload))
                batch_started.set()
                await __import__("asyncio").to_thread(
                    release_batch.wait,
                    2.0,
                )
                return {"received_bytes": offset + len(payload)}

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=ClaimedBatchRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        speech_failures = []
        stop_results = []
        payload = adpcm_block_for_size(18)
        speech_thread = threading.Thread(
            target=lambda: self._capture_failure(
                speech_failures,
                lambda: monitor.play_pcm(payload),
            )
        )
        speech_thread.start()
        self.assertTrue(batch_started.wait(timeout=1.0))
        stop_thread = threading.Thread(
            target=lambda: stop_results.append(monitor.command("stop"))
        )
        stop_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._preempt_stop_request is None and time.monotonic() < deadline:
            time.sleep(0.005)
        release_batch.set()
        speech_thread.join(timeout=3.0)
        stop_thread.join(timeout=3.0)

        self.assertEqual(len(stop_results), 1)
        self.assertEqual(len(speech_failures), 1)
        calls = FakeRuntime.instances[0].calls
        batch_index = calls.index(("write_pcm_batch", 0, payload[:8]))
        stop_index = calls.index(("stop",))
        self.assertLess(batch_index, stop_index)
        self.assertFalse(any(
            call[0] == "write_pcm_batch" and call[1] > 0
            for call in calls
        ))
        self.assertNotIn(("start_pcm", len(payload)), calls)
        monitor.close()

    def test_timeout_cannot_win_after_final_start_is_in_flight(self):
        start_entered = threading.Event()
        release_start = threading.Event()

        class SlowStartRuntime(FakeRuntime):
            async def start_pcm(
                self,
                transfer_id,
                byte_count,
                fletcher16,
                *,
                cancel_requested=None,
            ):
                start_entered.set()
                await __import__("asyncio").to_thread(
                    release_start.wait,
                    2.0,
                )
                return await super().start_pcm(
                    transfer_id,
                    byte_count,
                    fletcher16,
                    cancel_requested=cancel_requested,
                )

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=SlowStartRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        outcomes = []
        failures = []
        with mock.patch(
            "robot_agent.blast_observation_monitor."
            "SAMPLED_AUDIO_TIMEOUT_SECONDS",
            0.05,
        ):
            speech_thread = threading.Thread(
                target=lambda: self._capture_outcome(
                    outcomes,
                    failures,
                    lambda: monitor.play_pcm(adpcm_block(1)),
                )
            )
            speech_thread.start()
            self.assertTrue(start_entered.wait(timeout=1.0))
            time.sleep(0.1)
            self.assertTrue(speech_thread.is_alive())
            release_start.set()
            speech_thread.join(timeout=3.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(len(outcomes), 1)
        self.assertTrue(outcomes[0]["started"])
        self.assertIn(
            ("start_pcm", 7),
            FakeRuntime.instances[0].calls,
        )
        monitor.close()

    def test_multi_batch_burst_keeps_observation_snapshot_fresh(self):
        class SlowFragmentRuntime(FakeRuntime):
            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                await __import__("asyncio").sleep(0.02)
                return await super().write_pcm_batch(
                    offset,
                    payload,
                    cancel_requested=cancel_requested,
                )

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=SlowFragmentRuntime,
        )
        monitor.start()
        first_snapshot = self.wait_for(monitor, "online")
        while first_snapshot["observation"] is None:
            time.sleep(0.005)
            first_snapshot = monitor.snapshot()
        observed_before = first_snapshot["last_observed_at_monotonic_ms"]

        result = monitor.play_pcm(adpcm_block_for_size(82))

        calls = FakeRuntime.instances[0].calls
        batch_indexes = [
            index for index, call in enumerate(calls)
            if call[0] == "write_pcm_batch"
        ]
        self.assertGreater(len(batch_indexes), 1)
        start_index = calls.index(("start_pcm", 82))
        upload_observations = calls[
            batch_indexes[0] + 1:start_index
        ].count(("observe",))
        self.assertGreaterEqual(upload_observations, 1)
        self.assertLess(upload_observations, len(batch_indexes))
        self.assertTrue(result["started"])
        self.assertGreater(
            monitor.snapshot()["last_observed_at_monotonic_ms"],
            observed_before,
        )
        monitor.close()

    def test_post_scan_pcm_burst_keeps_snapshot_inside_episode_ttl(self):
        upload_reached_four_seconds = threading.Event()
        release_upload = threading.Event()

        class SimulatedMonotonicClock:
            def __init__(self):
                self._lock = threading.Lock()
                self._milliseconds = 1_000

            def __call__(self):
                with self._lock:
                    return self._milliseconds * 1_000_000

            def advance(self, milliseconds):
                with self._lock:
                    self._milliseconds += milliseconds

            def milliseconds(self):
                with self._lock:
                    return self._milliseconds

        clock = SimulatedMonotonicClock()

        class LongPostScanSpeechRuntime(FakeRuntime):
            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                clock.advance(1_100)
                if offset == 24:
                    upload_reached_four_seconds.set()
                    await __import__("asyncio").to_thread(
                        release_upload.wait,
                        2.0,
                    )
                return await super().write_pcm_batch(
                    offset,
                    payload,
                    cancel_requested=cancel_requested,
                )

        with mock.patch(
            "robot_agent.blast_observation_monitor.time.monotonic_ns",
            clock,
        ):
            monitor = BlastObservationMonitor(
                poll_interval_seconds=1.0,
                runtime_factory=LongPostScanSpeechRuntime,
            )
            monitor.start()
            self.wait_for(monitor, "online")
            scan = monitor.command(
                SCAN_COMMAND,
                action_permit=self.measured_scan_permit(monitor),
            )
            self.assertTrue(scan["completed"])
            failures = []
            speech_thread = threading.Thread(
                target=lambda: self._capture_failure(
                    failures,
                    lambda: monitor.play_pcm(adpcm_block_for_size(42)),
                )
            )
            speech_thread.start()
            self.assertTrue(upload_reached_four_seconds.wait(timeout=1.0))

            snapshot = monitor.snapshot()
            age_ms = (
                clock.milliseconds()
                - snapshot["last_observed_at_monotonic_ms"]
            )
            self.assertLessEqual(age_ms, 3_000)
            self.assertNotIn(
                ("start_pcm", 42),
                FakeRuntime.instances[0].calls,
            )

            release_upload.set()
            speech_thread.join(timeout=3.0)
            self.assertFalse(speech_thread.is_alive())
            self.assertEqual(failures, [])
            monitor.close()

    def test_stop_between_pcm_batches_drops_speech_upload(self):
        first_batch_started = threading.Event()
        release_first_batch = threading.Event()

        class FragmentRuntime(FakeRuntime):
            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                self.calls.append(("write_pcm_batch", offset, payload))
                if offset == 0:
                    first_batch_started.set()
                    await __import__("asyncio").to_thread(
                        release_first_batch.wait,
                        2.0,
                    )
                return {"received_bytes": offset + len(payload)}

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FragmentRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        speech_failure = []
        speech_thread = threading.Thread(
            target=lambda: self._capture_failure(
                speech_failure,
                lambda: monitor.play_pcm(adpcm_block_for_size(18)),
            )
        )
        speech_thread.start()
        self.assertTrue(first_batch_started.wait(1.0))
        stop_result = []
        stop_thread = threading.Thread(
            target=lambda: stop_result.append(monitor.command("stop"))
        )
        stop_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_command is None and time.monotonic() < deadline:
            time.sleep(0.005)
        release_first_batch.set()
        speech_thread.join(3.0)
        stop_thread.join(3.0)

        self.assertEqual(len(speech_failure), 1)
        self.assertEqual(
            speech_failure[0].code,
            "controller_command_interrupted",
        )
        self.assertEqual(len(stop_result), 1)
        calls = FakeRuntime.instances[0].calls
        self.assertIn(("stop",), calls)
        first_batch = calls.index(
            ("write_pcm_batch", 0, adpcm_block_for_size(18)[:8])
        )
        stop = calls.index(("stop",))
        self.assertNotIn(("observe",), calls[first_batch + 1:stop])
        self.assertNotIn(("start_pcm", 18), calls)
        self.assertFalse(any(
            call[0] == "write_pcm_batch" and call[1] > 0
            for call in calls
        ))
        monitor.close()

    def test_episode_cancel_between_pcm_batches_never_starts_audio(self):
        first_batch_started = threading.Event()
        release_first_batch = threading.Event()
        cancel = threading.Event()

        class FragmentRuntime(FakeRuntime):
            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                self.calls.append(("write_pcm_batch", offset, payload))
                if offset == 0:
                    first_batch_started.set()
                    await __import__("asyncio").to_thread(
                        release_first_batch.wait,
                        2.0,
                    )
                return {"received_bytes": offset + len(payload)}

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FragmentRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        speech_failure = []
        speech_thread = threading.Thread(
            target=lambda: self._capture_failure(
                speech_failure,
                lambda: monitor.play_pcm(
                    adpcm_block_for_size(18),
                    cancel_requested=cancel.is_set,
                ),
            )
        )
        speech_thread.start()
        self.assertTrue(first_batch_started.wait(1.0))

        cancel.set()
        release_first_batch.set()
        speech_thread.join(3.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertEqual(len(speech_failure), 1)
        self.assertEqual(
            speech_failure[0].code,
            "controller_command_interrupted",
        )
        calls = FakeRuntime.instances[0].calls
        self.assertNotIn(("start_pcm", 18), calls)
        self.assertFalse(any(
            call[0] == "write_pcm_batch" and call[1] > 0
            for call in calls
        ))
        monitor.close()

    def test_started_pcm_does_not_hold_ble_until_declared_duration(self):
        class ActiveAudioRuntime(FakeRuntime):
            audio_active = False
            drive_during_audio = False

            async def start_pcm(
                self,
                transfer_id,
                byte_count,
                fletcher16,
                *,
                cancel_requested=None,
            ):
                self.calls.append(("start_pcm", byte_count))
                self.audio_active = True
                return {
                    "transfer_id": transfer_id,
                    "byte_count": byte_count,
                    "sample_count": self.pcm_sample_count,
                    "sample_rate_hz": 16000,
                    "encoding": "ima_adpcm4_mono_stream_v1",
                    "fletcher16": fletcher16,
                    "duration_ms": 2_000,
                }

            async def drive_pulse(self, direction):
                self.drive_during_audio = self.audio_active
                return await super().drive_pulse(direction)

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=ActiveAudioRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        started_at = time.monotonic()
        speech = monitor.play_pcm(adpcm_block_for_size(16))
        navigation = monitor.command("drive_forward")

        runtime = FakeRuntime.instances[0]
        self.assertTrue(speech["started"])
        self.assertFalse(speech["completed"])
        self.assertTrue(navigation["completed"])
        self.assertTrue(runtime.audio_active)
        self.assertTrue(runtime.drive_during_audio)
        self.assertLess(time.monotonic() - started_at, 2.0)
        monitor.close()

    def test_sampled_audio_upload_does_not_require_idle_motors(self):
        class MovingRuntime(FakeRuntime):
            force_motion = False

            async def observe(self):
                observation = await super().observe()
                if self.force_motion:
                    observation["motion_active"] = True
                return observation

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=MovingRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        runtime = FakeRuntime.instances[0]
        runtime.force_motion = True

        result = monitor.play_pcm(adpcm_block(1))

        self.assertTrue(result["started"])
        self.assertIn(("begin_pcm", 7), runtime.calls)
        self.assertIn(("start_pcm", 7), runtime.calls)
        self.assertEqual(len(FakeRuntime.instances), 1)
        monitor.close()

    def test_sampled_audio_transport_failure_reconnects(self):
        class BrokenSpeechRuntime(FakeRuntime):
            sampled_audio_aligned = False

            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                raise RuntimeError("write failed")

        class Factory:
            def __init__(self):
                self.calls = 0

            def __call__(self, *, hub_name):
                self.calls += 1
                runtime_type = BrokenSpeechRuntime if self.calls == 1 else FakeRuntime
                return runtime_type(hub_name=hub_name)

        factory = Factory()
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            reconnect_interval_seconds=0.05,
            runtime_factory=factory,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        with self.assertRaises(BlastControllerError) as rejected:
            monitor.play_pcm(adpcm_block(1))

        self.assertEqual(rejected.exception.code, "controller_command_failed")
        deadline = time.monotonic() + 2.0
        while factory.calls < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertGreaterEqual(factory.calls, 2)
        monitor.close()

    def test_aligned_speech_phase_failure_keeps_live_session(self):
        class PhaseErrorRuntime(FakeRuntime):
            async def start_pcm(
                self,
                transfer_id,
                byte_count,
                fletcher16,
                *,
                cancel_requested=None,
            ):
                self.calls.append(("start_pcm_error", byte_count))
                raise RuntimeError("invalid started reply")

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=PhaseErrorRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        runtime = FakeRuntime.instances[0]
        runtime.calls.clear()

        with self.assertRaises(BlastControllerError) as rejected:
            monitor.play_pcm(adpcm_block(1))
        navigation = monitor.command("drive_forward")

        self.assertEqual(rejected.exception.code, "controller_command_failed")
        self.assertEqual(len(FakeRuntime.instances), 1)
        phase_error = runtime.calls.index(("start_pcm_error", 7))
        navigation_start = runtime.calls.index(("drive_pulse", "forward"))
        self.assertIn(("observe",), runtime.calls[phase_error:navigation_start])
        self.assertTrue(navigation["completed"])
        monitor.close()

    def test_hanging_pcm_batch_is_capped_before_fresh_queued_navigation(self):
        batch_started = threading.Event()

        class HangingBatchRuntime(FakeRuntime):
            batch_cancelled = False
            drive_saw_fresh_probe = False

            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                self.calls.append(("write_pcm_batch", offset, payload))
                batch_started.set()
                try:
                    await __import__("asyncio").sleep(10.0)
                finally:
                    self.batch_cancelled = True

            async def observe(self):
                observation = await super().observe()
                if self.batch_cancelled:
                    self.calls.append(("fresh_failure_probe",))
                return observation

            async def drive_pulse(self, direction):
                self.drive_saw_fresh_probe = (
                    self.batch_cancelled
                    and ("fresh_failure_probe",) in self.calls
                )
                return await super().drive_pulse(direction)

        class ImmediatelySettledMonitor(BlastObservationMonitor):
            @staticmethod
            def _settling_window_is_stable(samples):
                return bool(samples)

        monitor = ImmediatelySettledMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=HangingBatchRuntime,
        )
        monitor.start()
        snapshot = self.wait_for(monitor, "online")
        while snapshot["observation"] is None:
            time.sleep(0.005)
            snapshot = monitor.snapshot()
        speech_failure = []
        navigation_result = []
        payload = adpcm_block_for_size(18)

        with self.assertLogs(
            "robot_agent.blast_observation_monitor",
            level="WARNING",
        ) as diagnostics, mock.patch(
            "robot_agent.blast_pcm_upload.PCM_BATCH_TIMEOUT_SECONDS",
            0.05,
        ):
            speech_thread = threading.Thread(
                target=lambda: self._capture_failure(
                    speech_failure,
                    lambda: monitor.play_pcm(payload),
                )
            )
            speech_thread.start()
            self.assertTrue(batch_started.wait(1.0))
            observed_before = monitor.snapshot()[
                "last_observed_at_monotonic_ms"
            ]
            navigation_thread = threading.Thread(
                target=lambda: navigation_result.append(
                    monitor.command("drive_forward")
                )
            )
            started_at = time.monotonic()
            navigation_thread.start()
            speech_thread.join(1.0)
            navigation_thread.join(2.0)
            elapsed = time.monotonic() - started_at

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(navigation_thread.is_alive())
        self.assertLess(elapsed, 0.75)
        self.assertEqual(len(speech_failure), 1)
        self.assertEqual(
            speech_failure[0].code,
            "controller_command_failed",
        )
        diagnostic = "\n".join(diagnostics.output)
        self.assertIn("phase=batch", diagnostic)
        self.assertIn("offset=0", diagnostic)
        self.assertIn("byte_count=18", diagnostic)
        self.assertIn("error_type=TimeoutError", diagnostic)
        self.assertEqual(len(navigation_result), 1)
        self.assertTrue(navigation_result[0]["completed"])
        runtime = FakeRuntime.instances[0]
        self.assertTrue(runtime.drive_saw_fresh_probe)
        self.assertGreater(
            monitor.snapshot()["last_observed_at_monotonic_ms"],
            observed_before,
        )
        batch_index = next(
            index for index, call in enumerate(runtime.calls)
            if call[0] == "write_pcm_batch"
        )
        probe_index = runtime.calls.index(("fresh_failure_probe",))
        drive_index = runtime.calls.index(("drive_pulse", "forward"))
        self.assertLess(batch_index, probe_index)
        self.assertLess(probe_index, drive_index)
        self.assertNotIn(("start_pcm", len(payload)), runtime.calls)
        monitor.close()

    def test_aligned_start_timeout_refreshes_before_queued_navigation(self):
        start_entered = threading.Event()
        drive_started = threading.Event()

        class AlignedStartTimeoutRuntime(FakeRuntime):
            start_request_sent = False
            drive_saw_fresh_probe = False

            async def start_pcm(
                self,
                transfer_id,
                byte_count,
                fletcher16,
                *,
                cancel_requested=None,
            ):
                self.calls.append(("start_pcm_awaiting_reply", byte_count))
                self.start_request_sent = True
                start_entered.set()
                await __import__("asyncio").sleep(10.0)

            async def observe(self):
                observation = await super().observe()
                if self.start_request_sent:
                    self.calls.append(("fresh_start_failure_probe",))
                return observation

            async def drive_pulse(self, direction):
                drive_started.set()
                self.drive_saw_fresh_probe = (
                    self.start_request_sent
                    and ("fresh_start_failure_probe",) in self.calls
                )
                return await super().drive_pulse(direction)

        class ImmediatelySettledMonitor(BlastObservationMonitor):
            @staticmethod
            def _settling_window_is_stable(samples):
                return bool(samples)

        monitor = ImmediatelySettledMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=AlignedStartTimeoutRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        speech_failure = []
        navigation_result = []

        with mock.patch(
            "robot_agent.blast_pcm_upload.PCM_START_REPLY_TIMEOUT_SECONDS",
            0.2,
        ):
            speech_thread = threading.Thread(
                target=lambda: self._capture_failure(
                    speech_failure,
                    lambda: monitor.play_pcm(adpcm_block(1)),
                )
            )
            speech_thread.start()
            self.assertTrue(start_entered.wait(1.0))
            navigation_thread = threading.Thread(
                target=lambda: navigation_result.append(
                    monitor.command("drive_forward")
                )
            )
            navigation_thread.start()
            deadline = time.monotonic() + 1.0
            while (
                monitor._pending_command is None
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
            self.assertIsNotNone(monitor._pending_command)
            self.assertFalse(drive_started.wait(0.05))
            speech_thread.join(1.0)
            navigation_thread.join(2.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(navigation_thread.is_alive())
        self.assertEqual(len(speech_failure), 1)
        self.assertEqual(
            speech_failure[0].code,
            "sampled_audio_start_timeout",
        )
        self.assertEqual(len(navigation_result), 1)
        self.assertTrue(navigation_result[0]["completed"])
        self.assertEqual(len(FakeRuntime.instances), 1)
        runtime = FakeRuntime.instances[0]
        self.assertTrue(runtime.drive_saw_fresh_probe)
        start_index = runtime.calls.index(("start_pcm_awaiting_reply", 7))
        probe_index = runtime.calls.index(("fresh_start_failure_probe",))
        drive_index = runtime.calls.index(("drive_pulse", "forward"))
        self.assertLess(start_index, probe_index)
        self.assertLess(probe_index, drive_index)
        monitor.close()

    def test_ambiguous_start_timeout_reconnects_before_queued_navigation(self):
        start_entered = threading.Event()

        class AmbiguousStartRuntime(FakeRuntime):
            start_request_sent = False

            async def start_pcm(
                self,
                transfer_id,
                byte_count,
                fletcher16,
                *,
                cancel_requested=None,
            ):
                self.calls.append(("start_pcm_awaiting_reply", byte_count))
                self.start_request_sent = True
                start_entered.set()
                await __import__("asyncio").sleep(10.0)

            async def observe(self):
                if self.start_request_sent:
                    self.calls.append(("observe_out_of_alignment",))
                    raise RuntimeError("late started reply left in line stream")
                return await super().observe()

        class Factory:
            def __init__(self):
                self.calls = 0

            def __call__(self, *, hub_name):
                self.calls += 1
                runtime_type = (
                    AmbiguousStartRuntime
                    if self.calls == 1
                    else FakeRuntime
                )
                return runtime_type(hub_name=hub_name)

        class ImmediatelySettledMonitor(BlastObservationMonitor):
            @staticmethod
            def _settling_window_is_stable(samples):
                return bool(samples)

        factory = Factory()
        monitor = ImmediatelySettledMonitor(
            poll_interval_seconds=0.05,
            reconnect_interval_seconds=0.05,
            runtime_factory=factory,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        speech_failure = []
        navigation_result = []

        with mock.patch(
            "robot_agent.blast_pcm_upload.PCM_START_REPLY_TIMEOUT_SECONDS",
            0.05,
        ):
            speech_thread = threading.Thread(
                target=lambda: self._capture_failure(
                    speech_failure,
                    lambda: monitor.play_pcm(adpcm_block(1)),
                )
            )
            speech_thread.start()
            self.assertTrue(start_entered.wait(1.0))
            navigation_thread = threading.Thread(
                target=lambda: navigation_result.append(
                    monitor.command("drive_forward")
                )
            )
            navigation_thread.start()
            speech_thread.join(1.0)
            navigation_thread.join(2.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(navigation_thread.is_alive())
        self.assertEqual(len(speech_failure), 1)
        self.assertEqual(
            speech_failure[0].code,
            "sampled_audio_start_timeout",
        )
        self.assertEqual(len(navigation_result), 1)
        self.assertTrue(navigation_result[0]["completed"])
        self.assertGreaterEqual(factory.calls, 2)
        stale_runtime = FakeRuntime.instances[0]
        fresh_runtime = FakeRuntime.instances[-1]
        self.assertIn(("observe_out_of_alignment",), stale_runtime.calls)
        self.assertNotIn(
            ("drive_pulse", "forward"),
            stale_runtime.calls,
        )
        self.assertIn(
            ("drive_pulse", "forward"),
            fresh_runtime.calls,
        )
        monitor.close()

    def test_aligned_speech_disconnect_defers_queued_navigation(self):
        start_reached = threading.Event()
        release_failure = threading.Event()

        class DisconnectedPhaseRuntime(FakeRuntime):
            connection_lost = False

            async def start_pcm(
                self,
                transfer_id,
                byte_count,
                fletcher16,
                *,
                cancel_requested=None,
            ):
                self.calls.append(("start_pcm_error", byte_count))
                self.connection_lost = True
                start_reached.set()
                await __import__("asyncio").to_thread(
                    release_failure.wait,
                    2.0,
                )
                raise RuntimeError("connection lost before started reply")

            async def observe(self):
                if self.connection_lost:
                    self.calls.append(("observe_failed",))
                    raise RuntimeError("connection lost")
                return await super().observe()

        class Factory:
            def __init__(self):
                self.calls = 0

            def __call__(self, *, hub_name):
                self.calls += 1
                runtime_type = (
                    DisconnectedPhaseRuntime
                    if self.calls == 1
                    else FakeRuntime
                )
                return runtime_type(hub_name=hub_name)

        factory = Factory()
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            reconnect_interval_seconds=0.05,
            runtime_factory=factory,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        speech_failure = []
        navigation_result = []
        speech_thread = threading.Thread(
            target=lambda: self._capture_failure(
                speech_failure,
                lambda: monitor.play_pcm(adpcm_block(1)),
            )
        )
        speech_thread.start()
        self.assertTrue(start_reached.wait(1.0))
        navigation_thread = threading.Thread(
            target=lambda: navigation_result.append(
                monitor.command("drive_forward")
            )
        )
        navigation_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_command is None and time.monotonic() < deadline:
            time.sleep(0.005)
        release_failure.set()
        speech_thread.join(3.0)
        navigation_thread.join(3.0)

        self.assertEqual(len(speech_failure), 1)
        self.assertEqual(
            speech_failure[0].code,
            "controller_command_failed",
        )
        self.assertEqual(len(navigation_result), 1)
        self.assertTrue(navigation_result[0]["completed"])
        self.assertGreaterEqual(factory.calls, 2)
        self.assertIn(("observe_failed",), FakeRuntime.instances[0].calls)
        self.assertNotIn(
            ("drive_pulse", "forward"),
            FakeRuntime.instances[0].calls,
        )
        self.assertIn(
            ("drive_pulse", "forward"),
            FakeRuntime.instances[-1].calls,
        )
        monitor.close()

    def test_speech_failure_reconnects_before_queued_navigation_runs(self):
        speech_started = threading.Event()
        release_failure = threading.Event()

        class BrokenSpeechRuntime(FakeRuntime):
            sampled_audio_aligned = False

            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                self.calls.append(("write_pcm_batch", offset, payload))
                speech_started.set()
                await __import__("asyncio").to_thread(
                    release_failure.wait,
                    2.0,
                )
                raise RuntimeError("write failed")

        class Factory:
            def __init__(self):
                self.calls = 0

            def __call__(self, *, hub_name):
                self.calls += 1
                runtime_type = (
                    BrokenSpeechRuntime if self.calls == 1 else FakeRuntime
                )
                return runtime_type(hub_name=hub_name)

        factory = Factory()
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            reconnect_interval_seconds=0.05,
            runtime_factory=factory,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        speech_failure = []
        navigation_result = []

        def speak():
            try:
                monitor.play_pcm(adpcm_block(1))
            except Exception as error:
                speech_failure.append(error)

        def navigate():
            try:
                navigation_result.append(monitor.command("drive_forward"))
            except Exception as error:
                navigation_result.append(error)

        speech_thread = threading.Thread(target=speak)
        speech_thread.start()
        self.assertTrue(speech_started.wait(1.0))
        navigation_thread = threading.Thread(target=navigate)
        navigation_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_command is None and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(monitor._pending_command, "drive_forward")
        release_failure.set()
        speech_thread.join(3.0)
        navigation_thread.join(3.0)

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(navigation_thread.is_alive())
        self.assertEqual(len(speech_failure), 1)
        self.assertEqual(
            speech_failure[0].code,
            "controller_command_failed",
        )
        self.assertEqual(len(navigation_result), 1)
        self.assertIsInstance(navigation_result[0], dict)
        self.assertTrue(navigation_result[0]["completed"])
        self.assertGreaterEqual(factory.calls, 2)
        self.assertIn(
            ("drive_pulse", "forward"),
            FakeRuntime.instances[-1].calls,
        )
        monitor.close()

    def test_queued_navigation_bounds_aligned_speech_failure_probe(self):
        batch_started = threading.Event()
        release_failure = threading.Event()

        class BlockedProbeRuntime(FakeRuntime):
            batch_failed = False

            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                self.calls.append(("write_pcm_batch", offset, payload))
                batch_started.set()
                await __import__("asyncio").to_thread(
                    release_failure.wait,
                    2.0,
                )
                self.batch_failed = True
                raise RuntimeError("AppData write failed")

            async def observe(self):
                if not self.batch_failed:
                    return await super().observe()
                self.calls.append(("blocked_failure_probe",))
                await __import__("asyncio").sleep(2.0)
                raise RuntimeError("connection lost")

        class Factory:
            def __init__(self):
                self.calls = 0

            def __call__(self, *, hub_name):
                self.calls += 1
                runtime_type = (
                    BlockedProbeRuntime if self.calls == 1 else FakeRuntime
                )
                return runtime_type(hub_name=hub_name)

        factory = Factory()
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            reconnect_interval_seconds=0.05,
            runtime_factory=factory,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        speech_failure = []
        navigation_result = []
        speech_thread = threading.Thread(
            target=lambda: self._capture_failure(
                speech_failure,
                lambda: monitor.play_pcm(adpcm_block(1)),
            )
        )
        speech_thread.start()
        self.assertTrue(batch_started.wait(1.0))
        navigation_thread = threading.Thread(
            target=lambda: navigation_result.append(
                monitor.command("drive_forward")
            )
        )
        navigation_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_command is None and time.monotonic() < deadline:
            time.sleep(0.005)

        released_at = time.monotonic()
        release_failure.set()
        speech_thread.join(3.0)
        navigation_thread.join(3.0)
        elapsed = time.monotonic() - released_at

        self.assertFalse(speech_thread.is_alive())
        self.assertFalse(navigation_thread.is_alive())
        self.assertLess(elapsed, 1.25)
        self.assertEqual(len(speech_failure), 1)
        self.assertEqual(len(navigation_result), 1)
        self.assertTrue(navigation_result[0]["completed"])
        self.assertGreaterEqual(factory.calls, 2)
        self.assertIn(
            ("blocked_failure_probe",),
            FakeRuntime.instances[0].calls,
        )
        self.assertIn(
            ("drive_pulse", "forward"),
            FakeRuntime.instances[-1].calls,
        )
        monitor.close()

    def test_speech_failure_does_not_drop_a_queued_stop(self):
        speech_started = threading.Event()
        release_failure = threading.Event()

        class BrokenSpeechRuntime(FakeRuntime):
            sampled_audio_aligned = False

            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                speech_started.set()
                await __import__("asyncio").to_thread(
                    release_failure.wait,
                    2.0,
                )
                raise RuntimeError("write failed")

        class Factory:
            def __init__(self):
                self.calls = 0

            def __call__(self, *, hub_name):
                self.calls += 1
                runtime_type = (
                    BrokenSpeechRuntime if self.calls == 1 else FakeRuntime
                )
                return runtime_type(hub_name=hub_name)

        factory = Factory()
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            reconnect_interval_seconds=0.05,
            runtime_factory=factory,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        speech_thread = threading.Thread(
            target=lambda: self._ignore_failure(
                lambda: monitor.play_pcm(adpcm_block(1))
            )
        )
        stop_result = []
        speech_thread.start()
        self.assertTrue(speech_started.wait(1.0))
        stop_thread = threading.Thread(
            target=lambda: stop_result.append(monitor.command("stop"))
        )
        stop_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_command is None and time.monotonic() < deadline:
            time.sleep(0.005)
        release_failure.set()
        speech_thread.join(3.0)
        stop_thread.join(3.0)

        self.assertFalse(stop_thread.is_alive())
        self.assertEqual(len(stop_result), 1)
        self.assertTrue(stop_result[0]["completed"])
        self.assertIn(("stop",), FakeRuntime.instances[-1].calls)
        monitor.close()

    def test_speech_failure_does_not_replay_permit_bound_scan(self):
        batch_started = threading.Event()
        release_failure = threading.Event()

        class BrokenSpeechRuntime(FakeRuntime):
            sampled_audio_aligned = False

            async def write_pcm_batch(
                self, offset, payload, *, cancel_requested=None,
            ):
                batch_started.set()
                await __import__("asyncio").to_thread(
                    release_failure.wait,
                    2.0,
                )
                raise RuntimeError("raw write failed")

        class Factory:
            def __init__(self):
                self.calls = 0

            def __call__(self, *, hub_name):
                self.calls += 1
                runtime_type = (
                    BrokenSpeechRuntime if self.calls == 1 else FakeRuntime
                )
                return runtime_type(hub_name=hub_name)

        factory = Factory()
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            reconnect_interval_seconds=0.05,
            runtime_factory=factory,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        speech_thread = threading.Thread(
            target=lambda: self._ignore_failure(
                lambda: monitor.play_pcm(adpcm_block_for_size(16))
            )
        )
        speech_thread.start()
        self.assertTrue(batch_started.wait(1.0))
        with monitor._lock:
            permit = _BlastNoReturnScanPermit(
                runtime_generation=monitor._runtime_generation,
                expires_at_monotonic_ns=time.monotonic_ns() + 5_000_000_000,
                drive_angles_deg=(10.0, 10.0),
            )
            monitor._issued_scan_permit = permit
        scan_result = []

        def scan():
            try:
                scan_result.append(
                    monitor.command(SCAN_COMMAND, action_permit=permit)
                )
            except Exception as error:
                scan_result.append(error)

        scan_thread = threading.Thread(target=scan)
        scan_thread.start()
        deadline = time.monotonic() + 1.0
        while monitor._pending_command is None and time.monotonic() < deadline:
            time.sleep(0.005)
        release_failure.set()
        speech_thread.join(3.0)
        scan_thread.join(3.0)

        self.assertEqual(len(scan_result), 1)
        self.assertIsInstance(scan_result[0], BlastControllerError)
        self.assertEqual(scan_result[0].code, "controller_unavailable")
        deadline = time.monotonic() + 2.0
        while factory.calls < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertGreaterEqual(factory.calls, 2)
        self.assertFalse(any(
            call[0] == "turn_pulse"
            for call in FakeRuntime.instances[-1].calls
        ))
        monitor.close()

    @staticmethod
    def _ignore_failure(callback):
        try:
            callback()
        except Exception:
            pass

    @staticmethod
    def _capture_failure(target, callback):
        try:
            callback()
        except Exception as error:
            target.append(error)

    @staticmethod
    def _capture_outcome(outcomes, failures, callback):
        try:
            outcomes.append(callback())
        except Exception as error:
            failures.append(error)

    def test_settled_observation_command_is_motorless(self):
        class RecordingMonitor(BlastObservationMonitor):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.settle_timeouts = []

            async def _observe_until_settled(
                self,
                runtime,
                *,
                generation,
                initial_observation,
                timeout_seconds=None,
            ):
                self.settle_timeouts.append(timeout_seconds)
                return await super()._observe_until_settled(
                    runtime,
                    generation=generation,
                    initial_observation=initial_observation,
                    timeout_seconds=timeout_seconds,
                )

        monitor = RecordingMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        runtime = FakeRuntime.instances[0]
        runtime.calls.clear()

        result = monitor.command(SETTLED_OBSERVATION_COMMAND)

        self.assertEqual(result["command"], SETTLED_OBSERVATION_COMMAND)
        self.assertTrue(result["completed"])
        self.assertEqual(result["receipt"], {"motion_started": False})
        self.assertTrue(result["observation_settled"])
        self.assertEqual(monitor.settle_timeouts, [None])
        self.assertGreaterEqual(
            len([call for call in runtime.calls if call == ("observe",)]),
            5,
        )
        self.assertFalse(any(
            call[0] in {
                "drive_pulse",
                "turn_pulse",
                "claw_pulse",
                "body_pulse",
                "stop",
            }
            for call in runtime.calls
        ))
        monitor.close()

    def test_stop_preempts_settled_observation_without_motor_start(self):
        settling_started = threading.Event()

        class NeverSettledRuntime(FakeRuntime):
            async def observe(self):
                observation = await super().observe()
                observation["distance_mm"] = 300 + self.observe_calls * 10
                return observation

        class SignallingMonitor(BlastObservationMonitor):
            async def _observe_until_settled(
                self,
                runtime,
                *,
                generation,
                initial_observation,
                timeout_seconds=None,
            ):
                settling_started.set()
                return await super()._observe_until_settled(
                    runtime,
                    generation=generation,
                    initial_observation=initial_observation,
                    timeout_seconds=timeout_seconds,
                )

        monitor = SignallingMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=NeverSettledRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        failures = []

        def observe_settled():
            try:
                monitor.command(SETTLED_OBSERVATION_COMMAND)
            except BlastControllerError as error:
                failures.append(error.code)

        command_thread = threading.Thread(target=observe_settled)
        command_thread.start()
        self.assertTrue(settling_started.wait(timeout=1.0))

        stop_result = monitor.command("stop")
        command_thread.join(timeout=1.0)

        self.assertFalse(command_thread.is_alive())
        self.assertEqual(failures, ["controller_command_interrupted"])
        self.assertTrue(stop_result["completed"])
        runtime = FakeRuntime.instances[0]
        self.assertIn(("stop",), runtime.calls)
        self.assertFalse(any(
            call[0] in {
                "drive_pulse",
                "turn_pulse",
                "claw_pulse",
                "body_pulse",
            }
            for call in runtime.calls
        ))
        monitor.close()

    def test_scan_front_arc_is_one_atomic_encoder_measured_command(self):
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
                    "left",
                    "left",
                    "right",
                    "right",
                    "right",
                    "right",
                    "right",
                    "right",
                    "right",
                    "right",
                    "left",
                    "left",
                    "left",
                    "left",
                )
                self.assert_direction(direction, expected[self.turn_index])
                delta = (
                    -11.0, -11.0, -11.0, -11.0,
                    11.0, 11.0, 11.0, 11.0,
                    11.0, 11.0, 11.0, 11.0,
                    -11.0, -11.0, -11.0, -11.0,
                )[self.turn_index]
                self.heading = (
                    self.heading
                    + delta
                    + 180.0
                ) % 360.0 - 180.0
                self.distance = (
                    500, 720, 800, 2_000,
                    790, 720, 500, 300,
                    600, 1_100, 1_200, 2_000,
                    1_200, 1_100, 600, 310,
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

        result = monitor.command(
            SCAN_COMMAND,
            action_permit=self.measured_scan_permit(monitor),
        )

        runtime = FakeRuntime.instances[0]
        self.assertEqual(
            [call for call in runtime.calls if call[0] == "turn_pulse"],
            [
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
            ],
        )
        self.assertEqual(result["command"], SCAN_COMMAND)
        self.assertEqual(result["receipt"], {"turn_count": 16})
        self.assertEqual(result["observation"]["imu"]["heading_deg"], 179.0)
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
            [0.0, -44.1, -88.2, 44.1, 88.2],
        )
        self.assertEqual(scan["restoration_error_deg"], 0.0)
        self.assertTrue(scan["restoration_verified"])
        self.assertTrue(scan["all_observations_settled"])
        self.assertEqual(
            [ray["relative_heading_deg"] for ray in scan["angular_rays"]],
            [0.0, -22.05, -44.1, -66.15, -88.2,
             22.05, 44.1, 66.15, 88.2],
        )
        validate_blast_scan_ray_contract(scan)
        monitor.close()

    def test_encoder_restoration_ignores_large_imu_drift(self):
        class DriftingImuRuntime(FakeRuntime):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.turn_count = 0

            async def observe(self):
                observation = await super().observe()
                observation["imu"]["heading_deg"] = (
                    0.0 if self.turn_count == 0 else 38.0
                )
                return observation

            async def scan_turn_pulse(self, direction):
                result = await super().scan_turn_pulse(direction)
                self.turn_count += 1
                return result

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=DriftingImuRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        scan = monitor.command(
            SCAN_COMMAND,
            action_permit=self.measured_scan_permit(monitor),
        )["scan"]

        self.assertTrue(scan["restoration_verified"])
        self.assertEqual(scan["restoration_error_deg"], 0.0)
        self.assertEqual(
            scan["imu_heading_diagnostics"],
            {
                "authority": "DIAGNOSTIC_ONLY",
                "start_heading_deg": 0.0,
                "final_heading_deg": 38.0,
                "restoration_error_deg": 38.0,
            },
        )
        self.assertEqual(
            [ray["relative_heading_deg"] for ray in scan["angular_rays"]],
            [0.0, -22.05, -44.1, -66.15, -88.2,
             22.05, 44.1, 66.15, 88.2],
        )
        monitor.close()

    def test_scan_records_calibrated_ninety_five_degree_edges_bilaterally(self):
        class CalibratedSweepRuntime(FakeRuntime):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.direction_counts = {"left": 0, "right": 0}

            async def scan_turn_pulse(self, direction):
                before = {
                    "left_drive": self.left_drive_angle,
                    "right_drive": self.right_drive_angle,
                }
                await self.turn_pulse(direction)
                index = self.direction_counts[direction] % 4
                self.direction_counts[direction] += 1
                actual_encoder_degrees = (48, 48, 48, 49)[index]
                sign = 1 if direction == "left" else -1
                self.left_drive_angle -= sign * actual_encoder_degrees
                self.right_drive_angle += sign * actual_encoder_degrees
                return {
                    "accepted": True,
                    "direction": direction,
                    "speed_dps": TURN_SPEED_DPS,
                    "wheel_angle_deg": (
                        SCAN_TURN_ENCODER_DEGREES_PER_PULSE
                    ),
                    "before_angles_deg": before,
                }

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=CalibratedSweepRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        scan = monitor.command(
            SCAN_COMMAND,
            action_permit=self.measured_scan_permit(monitor),
        )["scan"]

        self.assertEqual(
            [ray["relative_heading_deg"] for ray in scan["angular_rays"]],
            [0.0, -23.52, -47.04, -70.56, -94.57,
             23.52, 47.04, 70.56, 94.57],
        )
        self.assertTrue(scan["restoration_verified"])
        self.assertEqual(scan["restoration_error_deg"], 0.0)
        monitor.close()

    def test_encoder_opposed_residue_over_ten_degrees_is_unrestored(self):
        class OpposedResidueRuntime(FakeRuntime):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.left_scan_pulses = 0
                self.right_scan_pulses = 0

            async def scan_turn_pulse(self, direction):
                before = {
                    "left_drive": self.left_drive_angle,
                    "right_drive": self.right_drive_angle,
                }
                await self.turn_pulse(direction)
                if direction == "left":
                    quotient, remainder = divmod(
                        8 * SCAN_TURN_ENCODER_DEGREES_PER_PULSE - 22,
                        8,
                    )
                    right_delta = quotient + (
                        self.left_scan_pulses < remainder
                    )
                    self.left_scan_pulses += 1
                    self.left_drive_angle -= (
                        SCAN_TURN_ENCODER_DEGREES_PER_PULSE
                    )
                    self.right_drive_angle += right_delta
                else:
                    quotient, remainder = divmod(
                        8 * SCAN_TURN_ENCODER_DEGREES_PER_PULSE + 22,
                        8,
                    )
                    left_delta = quotient + (
                        self.right_scan_pulses < remainder
                    )
                    self.right_scan_pulses += 1
                    self.left_drive_angle += left_delta
                    self.right_drive_angle -= (
                        SCAN_TURN_ENCODER_DEGREES_PER_PULSE
                    )
                return {
                    "accepted": True,
                    "direction": direction,
                    "speed_dps": TURN_SPEED_DPS,
                    "wheel_angle_deg": (
                        SCAN_TURN_ENCODER_DEGREES_PER_PULSE
                    ),
                    "before_angles_deg": before,
                }

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=OpposedResidueRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        scan = monitor.command(
            SCAN_COMMAND,
            action_permit=self.measured_scan_permit(monitor),
        )["scan"]

        self.assertFalse(scan["restoration_verified"])
        self.assertEqual(scan["result"], "restoration_unverified")
        self.assertGreater(abs(scan["restoration_error_deg"]), 10.0)
        monitor.close()

    def test_scan_restoration_uses_lego_scale_translation_band(self):
        for delta, expected_restored, expected_common_mm in (
            ((24, 20), True, 11.0),
            ((74, 66), False, 35.0),
        ):
            with self.subTest(delta=delta):
                class CommonResidueRuntime(FakeRuntime):
                    def __init__(self, **kwargs):
                        super().__init__(**kwargs)
                        self.left_scan_pulses = 0
                        self.right_scan_pulses = 0

                    async def scan_turn_pulse(self, direction):
                        before = {
                            "left_drive": self.left_drive_angle,
                            "right_drive": self.right_drive_angle,
                        }
                        await self.turn_pulse(direction)
                        if direction == "left":
                            quotient, remainder = divmod(
                                8 * SCAN_TURN_ENCODER_DEGREES_PER_PULSE
                                + delta[1],
                                8,
                            )
                            right_delta = quotient + (
                                self.left_scan_pulses < remainder
                            )
                            self.left_scan_pulses += 1
                            self.left_drive_angle -= (
                                SCAN_TURN_ENCODER_DEGREES_PER_PULSE
                            )
                            self.right_drive_angle += right_delta
                        else:
                            quotient, remainder = divmod(
                                8 * SCAN_TURN_ENCODER_DEGREES_PER_PULSE
                                + delta[0],
                                8,
                            )
                            left_delta = quotient + (
                                self.right_scan_pulses < remainder
                            )
                            self.right_scan_pulses += 1
                            self.left_drive_angle += left_delta
                            self.right_drive_angle -= (
                                SCAN_TURN_ENCODER_DEGREES_PER_PULSE
                            )
                        return {
                            "accepted": True,
                            "direction": direction,
                            "speed_dps": TURN_SPEED_DPS,
                            "wheel_angle_deg": (
                                SCAN_TURN_ENCODER_DEGREES_PER_PULSE
                            ),
                            "before_angles_deg": before,
                        }

                monitor = BlastObservationMonitor(
                    poll_interval_seconds=0.05,
                    runtime_factory=CommonResidueRuntime,
                )
                monitor.start()
                self.wait_for(monitor, "online")

                scan = monitor.command(
                    SCAN_COMMAND,
                    action_permit=self.measured_scan_permit(monitor),
                )["scan"]

                self.assertIs(
                    scan["restoration_verified"], expected_restored,
                )
                self.assertEqual(
                    scan["encoder_restoration"][
                        "common_mode_residue_mm"
                    ],
                    expected_common_mm,
                )
                monitor.close()

    def test_repeated_near_ray_only_replaces_weak_equivalent_evidence(self):
        monitor = BlastObservationMonitor(runtime_factory=FakeRuntime)

        def observation(distance, heading, observed_at, body=158):
            opposed = round(abs(heading) / 0.490)
            drive = (
                {"left_drive": -opposed, "right_drive": opposed}
                if heading < 0
                else {"left_drive": opposed, "right_drive": -opposed}
            )
            return {
                "distance_mm": distance,
                "imu": {"heading_deg": heading},
                "motor_angles_deg": {"body": body, **drive},
                "observed_at_ms": observed_at,
            }

        cases = (
            (
                "settled echo replaces no-return",
                (observation(2_000, -22.0, 1), True,
                 SCAN_RAY_EVIDENCE_SETTLED),
                (observation(180, -21.0, 2, 159), True,
                 SCAN_RAY_EVIDENCE_SETTLED),
                (180.0, -21.07, 159, 2, True,
                 SCAN_RAY_EVIDENCE_SETTLED),
            ),
            (
                "settled primary echo is not cherry-picked",
                (observation(400, -22.0, 1), True,
                 SCAN_RAY_EVIDENCE_SETTLED),
                (observation(180, -21.0, 2), True,
                 SCAN_RAY_EVIDENCE_SETTLED),
                (400.0, -22.05, 158, 1, True,
                 SCAN_RAY_EVIDENCE_SETTLED),
            ),
            (
                "unsettled return echo is ignored",
                (observation(2_000, -22.0, 1), True,
                 SCAN_RAY_EVIDENCE_SETTLED),
                (observation(180, -21.0, 2), False,
                 SCAN_RAY_EVIDENCE_SWEEP_ONLY),
                (2_000.0, -22.05, 158, 1, True,
                 SCAN_RAY_EVIDENCE_SETTLED),
            ),
            (
                "different return heading is ignored",
                (observation(2_000, -22.0, 1), True,
                 SCAN_RAY_EVIDENCE_SETTLED),
                (observation(180, -16.0, 2), True,
                 SCAN_RAY_EVIDENCE_SETTLED),
                (2_000.0, -22.05, 158, 1, True,
                 SCAN_RAY_EVIDENCE_SETTLED),
            ),
            (
                "settled no-return replaces unresolved primary",
                (observation(700, -22.0, 1), False,
                 SCAN_RAY_EVIDENCE_SWEEP_ONLY),
                (observation(2_000, -21.0, 2), True,
                 SCAN_RAY_EVIDENCE_SETTLED),
                (2_000.0, -21.07, 158, 2, True,
                 SCAN_RAY_EVIDENCE_SETTLED),
            ),
        )
        for name, primary, repeated, expected in cases:
            with self.subTest(name=name):
                ray = monitor._aggregate_repeated_scan_ray(
                    "left_near",
                    {"left_drive": 0, "right_drive": 0},
                    primary,
                    repeated,
                )
                self.assertEqual(
                    (
                        ray["distance_mm"],
                        ray["heading_deg"],
                        ray["body_motor_angle_deg"],
                        ray["observed_at_ms"],
                        ray["observation_settled"],
                        ray["evidence_use"],
                    ),
                    expected,
                )

    def test_close_or_invalid_return_ray_cannot_be_masked(self):
        for distance, error_code in (
            (40, "scan_sweep_clearance_lost"),
            (-1, "scan_sweep_observation_unverified"),
        ):
            with self.subTest(distance=distance):
                FakeRuntime.instances = []

                class UnsafeReturnRuntime(FakeRuntime):
                    def __init__(self, **kwargs):
                        super().__init__(**kwargs)
                        self.turn_count = 0

                    async def turn_pulse(self, direction):
                        receipt = await super().turn_pulse(direction)
                        self.turn_count += 1
                        return receipt

                    async def observe(self):
                        result = await super().observe()
                        if self.turn_count == 1:
                            result["distance_mm"] = 2_000
                        elif self.turn_count == 3:
                            result["distance_mm"] = distance
                        return result

                monitor = BlastObservationMonitor(
                    poll_interval_seconds=0.05,
                    runtime_factory=UnsafeReturnRuntime,
                )
                monitor.start()
                self.wait_for(monitor, "online")

                with self.assertRaises(BlastControllerError) as raised:
                    monitor.command(
                        SCAN_COMMAND,
                        action_permit=self.measured_scan_permit(monitor),
                    )

                self.assertEqual(raised.exception.code, error_code)
                self.assertIs(raised.exception.motion_started, True)
                runtime = FakeRuntime.instances[0]
                self.assertEqual(
                    len([
                        call for call in runtime.calls
                        if call[0] == "turn_pulse"
                    ]),
                    3,
                )
                monitor.close()

    def test_scan_and_turns_use_a_longer_settle_window_than_driving(self):
        class RecordingMonitor(BlastObservationMonitor):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.settle_timeouts = []

            async def _observe_until_settled(
                self,
                runtime,
                *,
                generation,
                initial_observation,
                timeout_seconds=None,
            ):
                self.settle_timeouts.append(timeout_seconds)
                return await super()._observe_until_settled(
                    runtime,
                    generation=generation,
                    initial_observation=initial_observation,
                    timeout_seconds=timeout_seconds,
                )

        monitor = RecordingMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        monitor.command(
            SCAN_COMMAND,
            action_permit=self.measured_scan_permit(monitor),
        )
        monitor.command("turn_left")
        monitor.command("drive_forward")

        self.assertEqual(monitor.settle_timeouts[:17], [
            SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
            *([SCAN_PULSE_POST_MOTION_SETTLE_TIMEOUT_SECONDS] * 16),
        ])
        self.assertEqual(
            monitor.settle_timeouts[-2:],
            [SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS, None],
        )
        # Hardware needs transport and scheduling headroom beyond the centre,
        # sixteen pulse windows and two possible outer-ray retries.
        self.assertGreaterEqual(
            SCAN_INTERNAL_COMMAND_TIMEOUT_SECONDS,
            SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS
            + 18 * SCAN_PULSE_POST_MOTION_SETTLE_TIMEOUT_SECONDS
            + 15,
        )
        self.assertGreaterEqual(
            SCAN_COMMAND_TIMEOUT_SECONDS,
            SCAN_INTERNAL_COMMAND_TIMEOUT_SECONDS + 3,
        )
        monitor.close()

    def test_scan_recovers_unresolved_near_ray_on_return_pass(self):
        class RetryMonitor(BlastObservationMonitor):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.turn_counts_at_settle = []

            async def _observe_until_settled(
                self,
                runtime,
                *,
                generation,
                initial_observation,
                timeout_seconds=None,
            ):
                self.turn_counts_at_settle.append(len([
                    call for call in runtime.calls
                    if call[0] == "turn_pulse"
                ]))
                if len(self.turn_counts_at_settle) == 3:
                    self._settling_samples = (
                        (1_391.0, 0.0, 0.0),
                        (1_503.0, 0.1, -0.1),
                        (1_420.0, 0.0, 0.0),
                        (2_000.0, -0.1, 0.1),
                        (1_489.0, 0.0, 0.0),
                    )
                    return {
                        **initial_observation,
                        "distance_mm": 1_489,
                    }, False
                return (
                    initial_observation,
                    True,
                )

        monitor = RetryMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        result = monitor.command(
            SCAN_COMMAND,
            action_permit=self.measured_scan_permit(monitor),
        )

        runtime = FakeRuntime.instances[0]
        self.assertTrue(result["completed"])
        self.assertEqual(
            len([call for call in runtime.calls if call[0] == "turn_pulse"]),
            16,
        )
        self.assertEqual(
            monitor.turn_counts_at_settle,
            list(range(17)),
        )
        self.assertTrue(result["scan"]["all_observations_settled"])
        left_near = result["scan"]["rays"][1]
        self.assertEqual(left_near["distance_mm"], 321.0)
        self.assertTrue(left_near["observation_settled"])
        self.assertEqual(
            left_near["evidence_use"], "SETTLED_RANGE",
        )
        monitor.close()

    def test_scan_reobserves_unsettled_far_ray_without_another_turn(self):
        class FarRetryMonitor(BlastObservationMonitor):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.turn_counts_at_settle = []

            async def _observe_until_settled(
                self,
                runtime,
                *,
                generation,
                initial_observation,
                timeout_seconds=None,
            ):
                turn_count = len([
                    call for call in runtime.calls
                    if call[0] == "turn_pulse"
                ])
                self.turn_counts_at_settle.append(turn_count)
                if len(self.turn_counts_at_settle) == 5:
                    self._settling_samples = (
                        (156.0, 0.0, 0.0),
                        (170.0, 0.1, -0.1),
                        (160.0, 0.0, 0.0),
                        (180.0, -0.1, 0.1),
                        (156.0, 0.0, 0.0),
                    )
                    return {
                        **initial_observation,
                        "distance_mm": 156,
                    }, False
                if len(self.turn_counts_at_settle) == 6:
                    return {
                        **initial_observation,
                        "distance_mm": 156,
                    }, True
                return initial_observation, True

        monitor = FarRetryMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        result = monitor.command(
            SCAN_COMMAND,
            action_permit=self.measured_scan_permit(monitor),
        )

        runtime = FakeRuntime.instances[0]
        self.assertEqual(
            [call for call in runtime.calls if call[0] == "turn_pulse"],
            [
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "right"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
                ("turn_pulse", "left"),
            ],
        )
        self.assertEqual(
            monitor.turn_counts_at_settle,
            [0, 1, 2, 3, 4, 4, 5, 6, 7, 8,
             9, 10, 11, 12, 13, 14, 15, 16],
        )
        left_far = result["scan"]["rays"][2]
        self.assertEqual(left_far["distance_mm"], 156.0)
        self.assertTrue(left_far["observation_settled"])
        self.assertEqual(
            left_far["evidence_use"], SCAN_RAY_EVIDENCE_SETTLED,
        )
        self.assertTrue(result["scan"]["all_observations_settled"])
        monitor.close()

    def test_unstored_sweep_only_pulses_do_not_change_ray_aggregate(self):
        for unsettled_call in (9, 17):
            with self.subTest(unsettled_call=unsettled_call):
                FakeRuntime.instances = []

                class UnstoredSweepOnlyMonitor(BlastObservationMonitor):
                    def __init__(self, **kwargs):
                        super().__init__(**kwargs)
                        self.settle_calls = 0

                    async def _observe_until_settled(
                        self,
                        runtime,
                        *,
                        generation,
                        initial_observation,
                        timeout_seconds=None,
                    ):
                        self.settle_calls += 1
                        if self.settle_calls == unsettled_call:
                            self._settling_samples = (
                                (1_400.0, 0.0, 0.0),
                            ) * 5
                            return initial_observation, False
                        return initial_observation, True

                monitor = UnstoredSweepOnlyMonitor(
                    poll_interval_seconds=0.05,
                    runtime_factory=FakeRuntime,
                )
                monitor.start()
                self.wait_for(monitor, "online")

                result = monitor.command(
                    SCAN_COMMAND,
                    action_permit=self.measured_scan_permit(monitor),
                )

                self.assertTrue(
                    result["scan"]["all_observations_settled"]
                )
                self.assertEqual(
                    result["observation_settled"],
                    unsettled_call != 17,
                )
                monitor.close()

    def test_safe_unsettled_left_ray_keeps_right_half_visible(self):
        class SoftLeftEvidenceMonitor(BlastObservationMonitor):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.settle_calls = 0

            async def _observe_until_settled(
                self,
                runtime,
                *,
                generation,
                initial_observation,
                timeout_seconds=None,
            ):
                self.settle_calls += 1
                # The first left ray and its matching return sample are
                # evidence-poor, but the stopped sweep window is still safe.
                if self.settle_calls in (2, 8):
                    self._settling_samples = (
                        (1_400.0, 0.0, 0.0),
                    ) * 5
                    return initial_observation, False
                return initial_observation, True

        monitor = SoftLeftEvidenceMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        result = monitor.command(
            SCAN_COMMAND,
            action_permit=self.measured_scan_permit(monitor),
        )

        rays = {
            ray["side"]: ray for ray in result["scan"]["angular_rays"]
        }
        self.assertFalse(rays["left_1"]["observation_settled"])
        self.assertEqual(
            rays["left_1"]["evidence_use"],
            SCAN_RAY_EVIDENCE_SWEEP_ONLY,
        )
        self.assertTrue(all(
            rays["right_{}".format(index)]["observation_settled"]
            for index in range(1, 5)
        ))
        runtime = FakeRuntime.instances[0]
        self.assertEqual(
            len([
                call for call in runtime.calls
                if call[0] == "turn_pulse"
            ]),
            16,
        )
        monitor.close()

    def test_sweep_only_window_rejects_each_pose_and_range_fault(self):
        monitor = BlastObservationMonitor(runtime_factory=FakeRuntime)
        observation = {
            "motion_active": False,
            "imu": {"heading_deg": 0.0},
            "motor_angles_deg": {"body": 158},
        }
        safe = ((1_400.0, 0.0, 0.0),) * 5
        monitor._settling_samples = safe
        self.assertTrue(
            monitor._scan_sweep_window_allows_continuation(observation)
        )

        cases = (
            ("short", safe[:4], observation),
            ("close", ((53.0, 0.0, 0.0),) * 5, observation),
            ("invalid", ((-1.0, 0.0, 0.0),) * 5, observation),
            ("tilt", safe[:4] + ((1_400.0, 1.1, 0.0),), observation),
            ("moving", safe, {**observation, "motion_active": True}),
            ("body", safe, {
                **observation,
                "motor_angles_deg": {"body": 160},
            }),
        )
        for name, samples, candidate in cases:
            with self.subTest(name=name):
                monitor._settling_samples = samples
                self.assertFalse(
                    monitor._scan_sweep_window_allows_continuation(candidate)
                )
        monitor._settling_samples = safe
        self.assertTrue(monitor._scan_sweep_window_allows_continuation({
            **observation, "imu": {},
        }))

    def test_scan_stops_when_unsettled_turn_has_no_safe_window(self):
        class NeverSettledTurnMonitor(BlastObservationMonitor):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.settle_calls = 0

            async def _observe_until_settled(
                self,
                runtime,
                *,
                generation,
                initial_observation,
                timeout_seconds=None,
            ):
                self.settle_calls += 1
                return initial_observation, self.settle_calls == 1

        monitor = NeverSettledTurnMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")

        with self.assertRaises(BlastControllerError) as raised:
            monitor.command(
                SCAN_COMMAND,
                action_permit=self.measured_scan_permit(monitor),
            )

        self.assertEqual(
            raised.exception.code,
            "scan_sweep_observation_unverified",
        )
        runtime = FakeRuntime.instances[0]
        self.assertEqual(
            [call for call in runtime.calls if call[0] == "turn_pulse"],
            [("turn_pulse", "left")],
        )
        self.assertEqual(monitor.settle_calls, 2)
        monitor.close()

    def test_scan_does_not_continue_unsettled_close_or_invalid_evidence(self):
        for distance in (40, -1):
            with self.subTest(distance=distance):
                FakeRuntime.instances = []

                class UnsafeTurnMonitor(BlastObservationMonitor):
                    def __init__(self, **kwargs):
                        super().__init__(**kwargs)
                        self.settle_calls = 0

                    async def _observe_until_settled(
                        self,
                        runtime,
                        *,
                        generation,
                        initial_observation,
                        timeout_seconds=None,
                    ):
                        self.settle_calls += 1
                        if self.settle_calls == 1:
                            return initial_observation, True
                        observation = {
                            **initial_observation,
                            "distance_mm": distance,
                        }
                        return observation, False

                monitor = UnsafeTurnMonitor(
                    poll_interval_seconds=0.05,
                    runtime_factory=FakeRuntime,
                )
                monitor.start()
                self.wait_for(monitor, "online")

                with self.assertRaises(BlastControllerError):
                    monitor.command(
                        SCAN_COMMAND,
                        action_permit=self.measured_scan_permit(monitor),
                    )

                runtime = FakeRuntime.instances[0]
                self.assertEqual(
                    [
                        call for call in runtime.calls
                        if call[0] == "turn_pulse"
                    ],
                    [("turn_pulse", "left")],
                )
                self.assertEqual(monitor.settle_calls, 2)
                monitor.close()

    def test_stop_cancellation_wins_during_scan_settle_window(self):
        settle_started = threading.Event()

        class UnstableAfterTurnRuntime(FakeRuntime):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.after_turn = False

            async def turn_pulse(self, direction):
                receipt = await super().turn_pulse(direction)
                self.after_turn = True
                return receipt

            async def observe(self):
                observation = await super().observe()
                if self.after_turn:
                    observation["distance_mm"] = (
                        300 + self.observe_calls * 20
                    )
                return observation

        class RetrySignallingMonitor(BlastObservationMonitor):
            def __init__(self, **kwargs):
                super().__init__(**kwargs)
                self.settle_calls = 0

            async def _observe_until_settled(self, *args, **kwargs):
                self.settle_calls += 1
                if self.settle_calls == 2:
                    settle_started.set()
                return await super()._observe_until_settled(
                    *args,
                    **kwargs,
                )

        monitor = RetrySignallingMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=UnstableAfterTurnRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        scan_failures = []
        scan_permit = self.measured_scan_permit(monitor)

        def scan():
            try:
                monitor.command(SCAN_COMMAND, action_permit=scan_permit)
            except BlastControllerError as error:
                scan_failures.append(error.code)

        with mock.patch(
            "robot_agent.blast_observation_monitor."
            "SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS",
            0.3,
        ):
            scan_thread = threading.Thread(target=scan)
            scan_thread.start()
            self.assertTrue(settle_started.wait(timeout=1.0))
            stop_result = monitor.command("stop")
            scan_thread.join(timeout=1.0)

        self.assertFalse(scan_thread.is_alive())
        self.assertEqual(scan_failures, ["controller_command_interrupted"])
        self.assertTrue(stop_result["completed"])
        runtime = FakeRuntime.instances[0]
        self.assertEqual(
            [call for call in runtime.calls if call[0] == "turn_pulse"],
            [("turn_pulse", "left")],
        )
        self.assertIn(("stop",), runtime.calls)
        monitor.close()

    def test_scan_stops_after_first_close_settled_turn_pulse(self):
        for distance, body, heading, error_code in (
            (40, 158, 12, "scan_sweep_clearance_lost"),
            (-1, 158, 12, "scan_sweep_observation_unverified"),
            (2_001, 158, 12, "scan_sweep_observation_unverified"),
            (321, 156, 12, "scan_sweep_observation_unverified"),
        ):
            with self.subTest(distance=distance, body=body, heading=heading):
                FakeRuntime.instances = []

                class UnsafeAfterFirstTurnRuntime(FakeRuntime):
                    def __init__(self, **kwargs):
                        super().__init__(**kwargs)
                        self.unsafe = False

                    async def observe(self):
                        observation = await super().observe()
                        if self.unsafe:
                            observation["distance_mm"] = distance
                            observation["motor_angles_deg"]["body"] = body
                            observation["imu"]["heading_deg"] = heading
                        return observation

                    async def turn_pulse(self, direction):
                        receipt = await super().turn_pulse(direction)
                        self.unsafe = True
                        return receipt

                monitor = BlastObservationMonitor(
                    poll_interval_seconds=0.05,
                    runtime_factory=UnsafeAfterFirstTurnRuntime,
                )
                monitor.start()
                self.wait_for(monitor, "online")

                with self.assertRaises(BlastControllerError) as raised:
                    monitor.command(
                        SCAN_COMMAND,
                        action_permit=self.measured_scan_permit(monitor),
                    )

                self.assertEqual(raised.exception.code, error_code)
                runtime = FakeRuntime.instances[0]
                self.assertEqual(
                    [
                        call for call in runtime.calls
                        if call[0] == "turn_pulse"
                    ],
                    [("turn_pulse", "left")],
                )
                self.assertEqual(monitor.snapshot()["state"], "online")
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
            (300, 156),
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
                    monitor.command(
                        SCAN_COMMAND,
                        action_permit=self.measured_scan_permit(monitor),
                    )

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

    def test_measured_scan_permit_closes_encoder_drift_before_first_pulse(self):
        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=FakeRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        result = monitor.command(
            SCAN_COMMAND,
            action_permit=self.measured_scan_permit(monitor),
        )
        self.assertTrue(result["completed"])
        monitor.close()

        for delta in (-2, 2):
            with self.subTest(delta=delta):
                monitor = BlastObservationMonitor(
                    poll_interval_seconds=0.05,
                    runtime_factory=FakeRuntime,
                )
                monitor.start()
                self.wait_for(monitor, "online")
                permit = self.measured_scan_permit(monitor)
                runtime = FakeRuntime.instances[-1]
                runtime.calls.clear()
                runtime.left_drive_angle += delta

                with self.assertRaises(BlastControllerError) as raised:
                    monitor.command(SCAN_COMMAND, action_permit=permit)

                self.assertEqual(
                    raised.exception.code,
                    "scan_start_clearance_unverified",
                )
                self.assertEqual(
                    [
                        call for call in runtime.calls
                        if call[0] == "turn_pulse"
                    ],
                    [],
                )
                monitor.close()

    def test_measured_scan_permit_allows_only_one_degree_anchor_settling(self):
        for delta, accepted in ((-2, False), (-1, True), (1, True), (2, False)):
            with self.subTest(delta=delta):
                monitor = BlastObservationMonitor(
                    poll_interval_seconds=0.05,
                    runtime_factory=FakeRuntime,
                )
                monitor.start()
                snapshot = self.wait_for(monitor, "online")
                while snapshot["observation"] is None:
                    time.sleep(0.005)
                    snapshot = monitor.snapshot()
                actual = snapshot["observation"]["motor_angles_deg"]
                expected = {
                    "left_drive": actual["left_drive"] + delta,
                    "right_drive": actual["right_drive"],
                }

                permit = monitor.issue_no_return_scan_permit(
                    expected_drive_angles=expected,
                    allow_no_return=False,
                )
                runtime = FakeRuntime.instances[-1]
                runtime.calls.clear()

                if accepted:
                    self.assertIsNotNone(permit)
                    self.assertEqual(
                        permit.drive_angles_deg,
                        (
                            float(actual["left_drive"]),
                            float(actual["right_drive"]),
                        ),
                    )
                    result = monitor.command(
                        SCAN_COMMAND,
                        action_permit=permit,
                    )
                    self.assertTrue(result["completed"])
                else:
                    self.assertIsNone(permit)
                    with self.assertRaises(BlastControllerError) as raised:
                        monitor.command(SCAN_COMMAND)
                    self.assertEqual(
                        raised.exception.code,
                        "scan_start_clearance_unverified",
                    )
                self.assertEqual(
                    len([
                        call for call in runtime.calls
                        if call[0] == "turn_pulse"
                    ]),
                    16 if accepted else 0,
                )
                monitor.close()

    def test_no_return_scan_permit_is_geometry_bound_and_single_use(self):
        class NoReturnRuntime(FakeRuntime):
            async def observe(self):
                observation = await super().observe()
                observation["distance_mm"] = 2_000
                observation["imu"] = {
                    "ready": False,
                    "stationary": False,
                    "raw_tilt_deg": [0.0, 0.0],
                }
                return observation

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=NoReturnRuntime,
        )
        monitor.start()
        snapshot = self.wait_for(monitor, "online")
        while snapshot["observation"] is None:
            time.sleep(0.005)
            snapshot = monitor.snapshot()
        runtime = FakeRuntime.instances[-1]
        runtime.calls.clear()

        with self.assertRaises(BlastControllerError) as unpermitted:
            monitor.command(SCAN_COMMAND)
        self.assertEqual(
            unpermitted.exception.code,
            "scan_start_clearance_unverified",
        )
        self.assertEqual(
            [call for call in runtime.calls if call[0] == "turn_pulse"],
            [],
        )

        pose = {"x_mm": 24, "y_mm": 373, "heading_mdeg": 980}
        prior = {
            "observation_settled": True,
            "pose": pose,
            "motion": {"command_completed": True},
            "result_observation": snapshot["observation"],
        }
        self.assertIsNone(monitor.issue_no_return_scan_permit(
            pose=pose,
            prior_receipt=prior,
            geometry_checked=False,
        ))
        missing_encoder = {
            **prior,
            "result_observation": {
                **prior["result_observation"],
                "motor_angles_deg": {
                    "left_drive": prior["result_observation"][
                        "motor_angles_deg"
                    ]["left_drive"],
                },
            },
        }
        self.assertIsNone(monitor.issue_no_return_scan_permit(
            pose=pose,
            prior_receipt=missing_encoder,
            geometry_checked=True,
        ))
        permit = monitor.issue_no_return_scan_permit(
            pose=pose,
            prior_receipt=prior,
            geometry_checked=True,
        )
        self.assertIsNotNone(permit)
        self.assertNotIn("heading_deg", snapshot["observation"]["imu"])

        result = monitor.command(SCAN_COMMAND, action_permit=permit)

        self.assertTrue(result["completed"])
        self.assertEqual(
            len([call for call in runtime.calls if call[0] == "turn_pulse"]),
            16,
        )
        with self.assertRaises(BlastControllerError) as reused:
            monitor.command(SCAN_COMMAND, action_permit=permit)
        self.assertEqual(
            reused.exception.code,
            "scan_start_clearance_unverified",
        )
        refreshed = monitor.snapshot()["observation"]
        stale_after_reconnect = monitor.issue_no_return_scan_permit(
            pose=pose,
            prior_receipt={
                "observation_settled": True,
                "pose": pose,
                "result_observation": refreshed,
            },
            geometry_checked=True,
        )
        self.assertIsNotNone(stale_after_reconnect)
        monitor.close()
        monitor.start()
        self.wait_for(monitor, "online")
        with self.assertRaises(BlastControllerError) as stale:
            monitor.command(
                SCAN_COMMAND,
                action_permit=stale_after_reconnect,
            )
        self.assertEqual(
            stale.exception.code,
            "scan_start_clearance_unverified",
        )
        monitor.close()

    def test_perception_only_permit_starts_no_return_scan_without_map(self):
        class NoReturnRuntime(FakeRuntime):
            async def observe(self):
                observation = await super().observe()
                observation["distance_mm"] = 2_000
                return observation

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=NoReturnRuntime,
        )
        monitor.start()
        snapshot = self.wait_for(monitor, "online")
        while snapshot["observation"] is None:
            time.sleep(0.005)
            snapshot = monitor.snapshot()
        runtime = FakeRuntime.instances[-1]
        runtime.calls.clear()

        permit = monitor.issue_no_return_scan_permit(
            geometry_checked=False,
            expected_drive_angles=snapshot["observation"][
                "motor_angles_deg"
            ],
            allow_no_return=True,
            perception_only=True,
        )
        self.assertIsNotNone(permit)
        result = monitor.command(SCAN_COMMAND, action_permit=permit)

        self.assertTrue(result["completed"])
        self.assertEqual(
            len([call for call in runtime.calls if call[0] == "turn_pulse"]),
            16,
        )
        monitor.close()

    def test_scan_settling_recovers_after_one_no_valid_center_sample(self):
        class TransientCenterRuntime(FakeRuntime):
            center_samples = None

            async def observe(self):
                observation = await super().observe()
                if self.center_samples:
                    observation["distance_mm"] = self.center_samples.pop(0)
                self.calls.append(("distance", observation["distance_mm"]))
                return observation

        monitor = BlastObservationMonitor(
            poll_interval_seconds=0.05,
            runtime_factory=TransientCenterRuntime,
        )
        monitor.start()
        self.wait_for(monitor, "online")
        runtime = FakeRuntime.instances[0]
        runtime.calls.clear()
        runtime.center_samples = [2_000] + [500] * 5

        result = monitor.command(
            SCAN_COMMAND,
            action_permit=self.measured_scan_permit(monitor),
        )

        first_turn = next(
            index
            for index, call in enumerate(runtime.calls)
            if call[0] == "turn_pulse"
        )
        pre_turn_distances = [
            call[1]
            for call in runtime.calls[:first_turn]
            if call[0] == "distance"
        ]
        self.assertIn(2_000, pre_turn_distances)
        self.assertEqual(pre_turn_distances[-5:], [500] * 5)
        self.assertEqual(result["scan"]["rays"][0]["distance_mm"], 500)
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
        scan_permit = self.measured_scan_permit(monitor)

        def scan():
            try:
                monitor.command(SCAN_COMMAND, action_permit=scan_permit)
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
