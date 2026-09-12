"""Persistent BLAST telemetry and bounded manual dashboard controls."""

from __future__ import annotations

from .blast_controller_contract import (
    SNAPSHOT_SCHEMA,
    ROBOT_ID,
    CONTROLLER_ID,
    DEFAULT_POLL_INTERVAL_SECONDS,
    DEFAULT_RECONNECT_INTERVAL_SECONDS,
    DISCONNECT_TIMEOUT_SECONDS,
    COMMAND_TIMEOUT_SECONDS,
    INTERNAL_COMMAND_TIMEOUT_SECONDS,
    SCAN_COMMAND_TIMEOUT_SECONDS,
    SCAN_INTERNAL_COMMAND_TIMEOUT_SECONDS,
    MIN_COMMAND_BUDGET_SECONDS,
    COMMAND_RESPONSE_MARGIN_SECONDS,
    MOTION_TIMEOUT_SECONDS,
    MOTION_POLL_INTERVAL_SECONDS,
    POST_MOTION_SETTLE_TIMEOUT_SECONDS,
    SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
    SCAN_PULSE_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
    POST_MOTION_IDLE_SAMPLE_COUNT,
    COMMAND_RESULT_SCHEMA,
    SAMPLED_AUDIO_RESULT_SCHEMA,
    SAMPLED_AUDIO_TIMEOUT_SECONDS,
    SAMPLED_AUDIO_MAX_TOTAL_SECONDS,
    SCAN_COMMAND,
    SURROUNDINGS_SCAN_COMMAND,
    SETTLED_OBSERVATION_COMMAND,
    CLAW_CLOSED_MOTOR_ANGLE_DEG,
    DOUBLE_CLAW_POSES,
    GESTURE_POSES,
    COMMANDS,
    NAVIGATION_MOTION_COMMANDS,
    _BlastNoReturnScanPermit,
    BlastControllerError,
)
from .blast_monitor_scan import _perform_scan_front_arc, _perform_scan_surroundings

import asyncio
from concurrent.futures import Future, TimeoutError as FutureTimeoutError
from copy import deepcopy
import logging
import math
from queue import Empty, Full, Queue
import threading
import time
from typing import Callable, Mapping, Optional

from .blast_ble_runtime import (
    DEFAULT_HUB_NAME,
    SAMPLED_AUDIO_CAPABILITY,
    SAMPLED_AUDIO_CHECKSUM,
    SAMPLED_AUDIO_ENCODING,
    SAMPLED_AUDIO_MAX_BYTES,
    SAMPLED_AUDIO_SAMPLE_RATE_HZ,
    SAMPLED_AUDIO_TRANSPORT,
    BlastBLERuntime,
    BlastCommandRejected,
    _adpcm_sample_count,
)
from .blast_pcm_upload import BlastPCMDeadline, BlastPCMStartTimeout, BlastPCMUpload
from .blast_navigation_calibration import (
    BLAST_ENCODER_SETTLING_DEGREES,
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_scan_observation import (
    RANGE_STATE_INVALID,
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    SCAN_RAY_SIDES,
    SCAN_RESTORATION_TOLERANCE_DEG,
    SCAN_RESULT_SCHEMA,
    blast_range_state,
    body_motor_angle as _body_motor_angle,
    validate_blast_scan_ray_contract,
)


logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())

# Measured closed reference for this BLAST linkage, not the last arbitrary pose.
# These are OFFSETS: _perform_command adds the 200-degree claw reference.
# The actual commanded claw targets are 200, 325, 200, 325, 200 degrees,
# all within the physically verified closed/open range (not absolute 0/125).
# Body offsets remain relative to the existing navigation sensor reference.
class BlastObservationMonitor:
    """Own one BLE session for telemetry and serialized fixed commands."""

    def __init__(
        self,
        *,
        hub_name: str = DEFAULT_HUB_NAME,
        poll_interval_seconds: float = DEFAULT_POLL_INTERVAL_SECONDS,
        reconnect_interval_seconds: float = DEFAULT_RECONNECT_INTERVAL_SECONDS,
        runtime_factory: Callable[..., object] = BlastBLERuntime,
    ) -> None:
        intervals = (poll_interval_seconds, reconnect_interval_seconds)
        valid_intervals = all(
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(float(value))
            and 0.05 <= float(value) <= 60.0
            for value in intervals
        )
        if not (
            isinstance(hub_name, str)
            and hub_name.strip()
            and valid_intervals
            and callable(runtime_factory)
        ):
            raise ValueError("BLAST observation configuration is invalid")
        self._hub_name = hub_name.strip()
        self._poll_interval_seconds = float(poll_interval_seconds)
        self._reconnect_interval_seconds = float(reconnect_interval_seconds)
        self._runtime_factory = runtime_factory
        self._lifecycle_lock = threading.RLock()
        self._lock = threading.RLock()
        self._stop_requested = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._loop = None
        self._task = None
        self._runtime_generation = 0
        self._settling_samples = ()
        self._pending_command = None
        self._pending_speech = None
        self._preempt_stop_request = None
        self._speech_step_active = False
        self._stop_epoch = 0
        self._issued_scan_permit = None
        self._command_queue = Queue(maxsize=1)
        self._speech_queue = Queue(maxsize=1)
        self._command_available = threading.Event()
        self._snapshot = {
            "schema": SNAPSHOT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "display_name": "BLAST",
            "hub_name": self._hub_name,
            "state": "configured",
            "reason_code": "observer_not_started",
            "last_observed_at_unix_ms": None,
            "last_observed_at_monotonic_ms": None,
            "ready": None,
            "observation": None,
        }

    def start(self) -> None:
        with self._lifecycle_lock:
            with self._lock:
                if self._thread is not None:
                    raise RuntimeError(
                        "BLAST observation monitor already started"
                    )
                self._stop_requested.clear()
                self._command_available.clear()
                self._set_state("connecting", "connecting")
                self._thread = threading.Thread(
                    target=self._run,
                    name="robot-llm-blast-observer",
                    daemon=True,
                )
                try:
                    self._thread.start()
                except Exception:
                    self._thread = None
                    self._set_state("offline", "observer_start_failed")
                    raise

    def connect(self):
        with self._lifecycle_lock:
            with self._lock:
                running = self._thread is not None
            if not running:
                self.start()
            return self.snapshot()

    def disconnect(self):
        with self._lifecycle_lock:
            self.close()
            return self.snapshot()

    def retry(self):
        with self._lifecycle_lock:
            self.close()
            self.start()
            return self.snapshot()

    def snapshot(self):
        with self._lock:
            return deepcopy(self._snapshot)

    def runtime_generation(self) -> int:
        """Return the current BLE session generation without exposing state."""

        with self._lock:
            return self._runtime_generation

    def issue_no_return_scan_permit(
        self, *, pose=None, prior_receipt=None, geometry_checked=False,
        expected_drive_angles=None, allow_no_return=True, perception_only=False,
    ):
        """Issue one short-lived token from an exact trusted scan anchor."""

        with self._lock:
            snapshot = deepcopy(self._snapshot)
            runtime_generation = self._runtime_generation
        observation = snapshot.get("observation")
        previous = prior_receipt.get("result_observation") if isinstance(
            prior_receipt, Mapping
        ) else None
        motors = observation.get("motor_angles_deg") if isinstance(
            observation, Mapping
        ) else None
        previous_motors = previous.get("motor_angles_deg") if isinstance(
            previous, Mapping
        ) else None
        now_ns = time.monotonic_ns()
        observed_ms = snapshot.get("last_observed_at_monotonic_ms")
        roles = ("left_drive", "right_drive")
        expected = (
            expected_drive_angles
            if isinstance(expected_drive_angles, Mapping) else motors
        )
        anchor_matched = (
            type(allow_no_return) is bool
            and isinstance(motors, Mapping)
            and isinstance(expected, Mapping)
            and all(
                isinstance(motors.get(role), (int, float))
                and not isinstance(motors.get(role), bool)
                and math.isfinite(float(motors[role]))
                and isinstance(expected.get(role), (int, float))
                and not isinstance(expected.get(role), bool)
                and math.isfinite(float(expected[role]))
                and abs(float(motors[role]) - float(expected[role]))
                <= BLAST_ENCODER_SETTLING_DEGREES
                for role in roles
            )
        )
        valid_pose = isinstance(pose, Mapping) and all(
            type(pose.get(key)) is int
            for key in ("x_mm", "y_mm", "heading_mdeg")
        )
        if not (
            snapshot.get("state") == "online"
            and type(perception_only) is bool
            and anchor_matched
            and isinstance(observed_ms, int)
            and 0 <= now_ns // 1_000_000 - observed_ms <= 3_000
            and isinstance(observation, Mapping)
            and observation.get("motion_active") is False
            and (
                not allow_no_return
                or perception_only is True
                or geometry_checked is True
                and valid_pose
                and isinstance(prior_receipt, Mapping)
                and prior_receipt.get("observation_settled") is True
                and prior_receipt.get("pose") == dict(pose)
                and (
                    not isinstance(prior_receipt.get("motion"), Mapping)
                    or prior_receipt["motion"].get(
                        "command_completed"
                    ) is True
                )
                and isinstance(previous, Mapping)
                and previous.get("motion_active") is False
                and blast_range_state(previous.get("distance_mm"))
                == RANGE_STATE_NO_VALID_DISTANCE
                and blast_range_state(observation.get("distance_mm"))
                == RANGE_STATE_NO_VALID_DISTANCE
                and isinstance(previous_motors, Mapping)
                and all(
                    isinstance(previous_motors.get(role), (int, float))
                    and not isinstance(previous_motors.get(role), bool)
                    and math.isfinite(float(previous_motors[role]))
                    and float(motors[role]) == float(previous_motors[role])
                    for role in roles
                )
            )
        ):
            return None
        permit = _BlastNoReturnScanPermit(
            runtime_generation=runtime_generation,
            expires_at_monotonic_ns=now_ns + 5_000_000_000,
            drive_angles_deg=tuple(float(motors[role]) for role in roles),
            allow_no_return=allow_no_return,
        )
        with self._lock:
            if (
                self._runtime_generation != runtime_generation
                or self._snapshot.get("state") != "online"
            ):
                return None
            self._issued_scan_permit = permit
        return permit

    def command(
        self, command: str, *, cancel_requested=None, action_permit=None,
    ):
        if command not in COMMANDS or (
            cancel_requested is not None
            and not callable(cancel_requested)
        ) or (
            action_permit is not None
            and not (
                command in (SCAN_COMMAND, SURROUNDINGS_SCAN_COMMAND)
                and isinstance(action_permit, _BlastNoReturnScanPermit)
            )
        ):
            raise ValueError("unsupported BLAST command")
        timeout_seconds = (
            SCAN_COMMAND_TIMEOUT_SECONDS
            if command in (SCAN_COMMAND, SURROUNDINGS_SCAN_COMMAND)
            else COMMAND_TIMEOUT_SECONDS
        )
        with self._lock:
            issued_permit = (
                self._issued_scan_permit
                if isinstance(action_permit, _BlastNoReturnScanPermit)
                else None
            )
            if action_permit is not None and (
                issued_permit is not action_permit
                or action_permit.runtime_generation
                != self._runtime_generation
                or action_permit.expires_at_monotonic_ns < time.monotonic_ns()
            ):
                raise BlastControllerError(
                    "scan_start_clearance_unverified",
                    "BLAST scan permit is stale or was not issued here",
                    motion_started=False,
                )
            if self._snapshot["state"] != "online":
                raise BlastControllerError(
                    "controller_unavailable",
                    "BLAST is not connected",
                )
            if cancel_requested is not None and cancel_requested():
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST command was cancelled before motor start",
                    motion_started=False,
                )
            preempts_active_command = (
                command == "stop"
                and (
                    self._pending_command is not None
                    and self._pending_command != "stop"
                    or self._speech_step_active
                )
            )
            if preempts_active_command:
                if self._preempt_stop_request is not None:
                    raise BlastControllerError(
                        "controller_busy",
                        "BLAST already has a pending stop command",
                    )
                result = Future()
                self._preempt_stop_request = (
                    self._runtime_generation,
                    result,
                    time.monotonic() + timeout_seconds,
                )
                self._stop_epoch += 1
                self._command_available.set()
            elif (
                self._pending_command is not None
                or self._preempt_stop_request is not None
            ):
                raise BlastControllerError(
                    "controller_busy",
                    "BLAST already has a pending command",
                )
            else:
                self._pending_command = command
                if command == "stop":
                    self._stop_epoch += 1
                result = Future()
                request = (
                    self._runtime_generation,
                    command,
                    result,
                    time.monotonic() + timeout_seconds,
                    action_permit,
                )
                if action_permit is not None:
                    self._issued_scan_permit = None
                try:
                    self._command_queue.put_nowait(request)
                except Full:
                    self._pending_command = None
                    raise BlastControllerError(
                        "controller_busy",
                        "BLAST command queue is full",
                    ) from None
                self._command_available.set()
        try:
            return result.result(timeout=timeout_seconds)
        except FutureTimeoutError:
            with self._lock:
                result.cancel()
            raise BlastControllerError(
                "controller_command_timeout",
                "BLAST command did not finish in time",
            ) from None

    def scan_surroundings(
        self, *, cancel_requested=None, action_permit=None,
    ):
        """Run the startup-only full surroundings scan."""

        return self.command(
            SURROUNDINGS_SCAN_COMMAND,
            cancel_requested=cancel_requested,
            action_permit=action_permit,
        )

    def play_pcm(self, payload: bytes, *, cancel_requested=None):
        """Preload one bounded utterance on the owned BLE session."""
        _adpcm_sample_count(payload)
        if cancel_requested is not None and not callable(cancel_requested):
            raise ValueError("cancel_requested must be callable")
        with self._lock:
            if self._snapshot["state"] != "online":
                raise BlastControllerError(
                    "controller_unavailable",
                    "BLAST is not connected",
                )
            capability = (
                self._snapshot.get("ready", {})
                .get("capabilities", {})
                .get(SAMPLED_AUDIO_CAPABILITY)
            )
            if capability != {
                "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
                "encoding": SAMPLED_AUDIO_ENCODING,
                "max_bytes": SAMPLED_AUDIO_MAX_BYTES,
                "transport": SAMPLED_AUDIO_TRANSPORT,
                "checksum": SAMPLED_AUDIO_CHECKSUM,
            }:
                raise BlastControllerError(
                    "sampled_audio_unavailable",
                    "BLAST firmware does not support sampled audio",
                    motion_started=False,
                )
            if cancel_requested is not None and cancel_requested():
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST speech was cancelled before playback",
                    motion_started=False,
                )
            if self._pending_speech is not None:
                raise BlastControllerError(
                    "controller_busy",
                    "BLAST already has pending speech",
                    motion_started=False,
                )
            result = Future()
            self._pending_speech = result
            deadline = self._new_speech_deadline()
            request = (
                self._runtime_generation,
                payload,
                result,
                deadline,
                cancel_requested,
            )
            try:
                self._speech_queue.put_nowait(request)
            except Full:
                self._pending_speech = None
                raise BlastControllerError(
                    "controller_busy",
                    "BLAST speech queue is full",
                    motion_started=False,
                ) from None
            self._command_available.set()
        while True:
            remaining = deadline.remaining()
            if remaining > 0:
                try:
                    return result.result(timeout=remaining)
                except FutureTimeoutError:
                    pass
            elif deadline.start_in_flight():
                # The bounded final start exchange is already committed. Its
                # verified receipt must win over the caller's deadline edge;
                # polling avoids both a tight loop and an unbounded wait.
                try:
                    return result.result(timeout=0.05)
                except FutureTimeoutError:
                    continue
            if not deadline.claim_timeout():
                continue
            with self._lock:
                cancelled = result.cancel()
            if not cancelled:
                return result.result()
            self._clear_timed_out_speech(result)
            raise BlastControllerError(
                "controller_command_timeout",
                "BLAST speech did not finish in time",
                motion_started=False,
            ) from None

    @staticmethod
    def _new_speech_deadline():
        return BlastPCMDeadline(
            inactivity_seconds=SAMPLED_AUDIO_TIMEOUT_SECONDS,
            maximum_seconds=SAMPLED_AUDIO_MAX_TOTAL_SECONDS,
            clock=time.monotonic,
        )

    def close(self) -> None:
        with self._lifecycle_lock:
            with self._lock:
                thread = self._thread
                self._issued_scan_permit = None
            if thread is None:
                self._set_state("stopped", "observer_stopped")
                return
            self._set_state("offline", "disconnecting")
            self._stop_requested.set()
            self._command_available.set()
            with self._lock:
                loop = self._loop
                task = self._task
            if loop is not None and task is not None:
                try:
                    loop.call_soon_threadsafe(task.cancel)
                except RuntimeError:
                    pass
            thread.join(timeout=12.0)
            if thread.is_alive():
                raise RuntimeError("BLAST observation monitor did not stop")
            self._reject_commands(
                BlastControllerError(
                    "controller_unavailable",
                    "BLAST connection ended",
                )
            )
            with self._lock:
                self._thread = None

    def _set_state(self, state: str, reason_code: Optional[str], **changes) -> None:
        with self._lock:
            snapshot = deepcopy(self._snapshot)
            snapshot.update(changes)
            snapshot["state"] = state
            snapshot["reason_code"] = reason_code
            self._snapshot = snapshot

    def _run(self) -> None:
        async def run_observer() -> None:
            with self._lock:
                self._loop = asyncio.get_running_loop()
                self._task = asyncio.current_task()
            try:
                await self._observe()
            except asyncio.CancelledError:
                pass
            finally:
                with self._lock:
                    self._loop = None
                    self._task = None

        asyncio.run(run_observer())

    async def _observe(self) -> None:
        try:
            while not self._stop_requested.is_set():
                try:
                    await self._observe_session()
                except asyncio.CancelledError:
                    raise
                except Exception:
                    logger.exception("BLAST observation session failed")
                    previous = self.snapshot()
                    reason = (
                        "observation_failed"
                        if previous["last_observed_at_unix_ms"] is not None
                        else "connection_failed"
                    )
                    self._set_state("offline", reason)
                if self._stop_requested.is_set():
                    break
                if await self._wait(self._reconnect_interval_seconds):
                    break
                self._set_state("connecting", "reconnecting")
        finally:
            if self._stop_requested.is_set():
                self._set_state("stopped", "observer_stopped")

    async def _observe_session(self) -> None:
        runtime = self._runtime_factory(hub_name=self._hub_name)
        speech_upload = None
        try:
            ready = await runtime.connect()
            if (
                ready.get("robot_id") != ROBOT_ID
                or ready.get("controller_id") != CONTROLLER_ID
            ):
                raise RuntimeError("BLAST controller identity mismatch")
            with self._lock:
                self._runtime_generation += 1
                generation = self._runtime_generation
                self._issued_scan_permit = None
            self._set_state("online", None, ready=ready)
            while not self._stop_requested.is_set():
                command_completed = False
                with self._lock:
                    stop_epoch = self._stop_epoch
                preempted = await self._service_preempt_stop(
                    runtime,
                    generation,
                )
                if preempted and speech_upload is not None:
                    self._interrupt_speech_upload(
                        speech_upload,
                        "BLAST speech upload was interrupted by stop",
                    )
                    speech_upload = None
                if preempted:
                    self._interrupt_pending_speech(
                        "BLAST speech was interrupted by stop"
                    )
                request = self._next_command()
                if preempted and request is not None:
                    self._finish_command(
                        request[2],
                        error=BlastControllerError(
                            "controller_command_interrupted",
                            "BLAST command was interrupted by stop",
                            motion_started=False,
                        ),
                    )
                elif request is None:
                    pass
                else:
                    if request[1] == "stop":
                        if speech_upload is not None:
                            self._interrupt_speech_upload(
                                speech_upload,
                                "BLAST speech upload was interrupted by stop",
                            )
                            speech_upload = None
                        self._interrupt_pending_speech(
                            "BLAST speech was interrupted by stop"
                        )
                    command_completed = await self._execute_command(
                        runtime,
                        generation,
                        request,
                    )
                    with self._lock:
                        stop_during_command = self._stop_epoch != stop_epoch
                    if stop_during_command:
                        preempted = True
                        if speech_upload is not None:
                            self._interrupt_speech_upload(
                                speech_upload,
                                "BLAST speech upload was interrupted by stop",
                            )
                            speech_upload = None
                        self._interrupt_pending_speech(
                            "BLAST speech was interrupted by stop"
                        )
                speech_turn = (
                    not preempted
                    and (
                        request is None
                        or request[1] != "stop"
                        and command_completed
                    )
                )
                if speech_turn:
                    if speech_upload is None:
                        speech_request = (
                            self._next_speech_after_command()
                            if request is not None
                            else self._next_speech()
                        )
                        if speech_request is not None:
                            speech_upload = BlastPCMUpload.from_request(
                                speech_request
                            )
                    if speech_upload is not None:
                        stopped_before_speech = (
                            await self._service_preempt_stop(
                                runtime,
                                generation,
                            )
                        )
                        if stopped_before_speech:
                            self._interrupt_speech_upload(
                                speech_upload,
                                "BLAST speech upload was interrupted by stop",
                            )
                            speech_upload = None
                            continue
                        if not self._claim_speech_step():
                            continue
                        try:
                            speech_upload, session_usable = (
                                await self._execute_speech_step(
                                    runtime,
                                    generation,
                                    speech_upload,
                                )
                            )
                        finally:
                            self._release_speech_step()
                        if not session_usable:
                            break
                        # One bounded audio phase gets a turn after a fixed
                        # command. The outer loop then admits pending
                        # navigation before the next upload phase.
                        if speech_upload is not None:
                            fixed_work_pending = self._fixed_work_pending()
                            with self._lock:
                                last_observed_ms = self._snapshot.get(
                                    "last_observed_at_monotonic_ms"
                                )
                            cancel_pending = (
                                speech_upload.cancel_requested is not None
                                and speech_upload.cancel_requested()
                            )
                            observation_due = (
                                not isinstance(last_observed_ms, int)
                                or time.monotonic_ns() // 1_000_000
                                - last_observed_ms
                                >= round(
                                    self._poll_interval_seconds * 1_000
                                )
                            )
                            if (
                                not fixed_work_pending
                                and not cancel_pending
                                and observation_due
                            ):
                                self._publish_observation(
                                    await runtime.observe()
                                )
                        continue
                    if request is None:
                        self._publish_observation(await runtime.observe())
                if await self._wait_for_work(
                    self._poll_interval_seconds
                ):
                    break
        finally:
            if speech_upload is not None:
                self._interrupt_speech_upload(
                    speech_upload,
                    "BLAST connection ended during speech upload",
                    code="controller_unavailable",
                )
            self._reject_commands(
                BlastControllerError(
                    "controller_unavailable",
                    "BLAST connection ended",
                )
            )
            try:
                await asyncio.wait_for(
                    runtime.close(),
                    timeout=DISCONNECT_TIMEOUT_SECONDS,
                )
            except Exception:
                pass

    def _next_command(self):
        try:
            return self._command_queue.get_nowait()
        except Empty:
            return None

    def _next_speech(self):
        # Fixed commands win the initial arbitration. Fair upload turns after
        # completed commands are claimed by _next_speech_after_command().
        with self._lock:
            if (
                self._pending_command is not None
                or self._preempt_stop_request is not None
            ):
                return None
            try:
                return self._speech_queue.get_nowait()
            except Empty:
                return None

    def _next_speech_after_command(self):
        """Claim one fair upload turn behind stop but ahead of more motion."""

        with self._lock:
            if (
                self._preempt_stop_request is not None
                or self._pending_command == "stop"
            ):
                return None
            try:
                return self._speech_queue.get_nowait()
            except Empty:
                return None

    def _claim_speech_step(self):
        with self._lock:
            if (
                self._preempt_stop_request is not None
                or self._pending_command == "stop"
            ):
                return False
            self._speech_step_active = True
            return True

    def _release_speech_step(self):
        with self._lock:
            self._speech_step_active = False

    def _interrupt_pending_speech(self, message):
        with self._lock:
            try:
                request = self._speech_queue.get_nowait()
            except Empty:
                return
        self._finish_speech(
            request[2],
            error=BlastControllerError(
                "controller_command_interrupted",
                message,
                motion_started=False,
            ),
        )

    def _fixed_work_pending(self):
        with self._lock:
            return (
                self._pending_command is not None
                or self._preempt_stop_request is not None
            )

    def _interrupt_speech_upload(
        self,
        upload,
        message,
        *,
        code="controller_command_interrupted",
    ):
        self._finish_speech(
            upload.result,
            error=BlastControllerError(
                code,
                message,
                motion_started=False,
            ),
        )

    async def _execute_speech_step(
        self, runtime, generation, upload,
    ):
        if upload.result.cancelled():
            self._finish_speech(upload.result)
            return None, True
        if upload.deadline.expired():
            upload.deadline.claim_timeout()
            self._finish_speech(
                upload.result,
                error=BlastControllerError(
                    "controller_command_timeout",
                    "BLAST speech did not make progress in time",
                    motion_started=False,
                ),
            )
            return None, True
        if upload.cancel_requested is not None and upload.cancel_requested():
            self._finish_speech(
                upload.result,
                error=BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST speech was cancelled before playback",
                    motion_started=False,
                ),
            )
            return None, True
        try:
            if upload.requested_generation != generation:
                raise BlastControllerError(
                    "stale_controller_command",
                    "BLAST reconnected before speech playback",
                    motion_started=False,
                )
            receipt = await upload.advance(runtime)
            if receipt is None:
                return upload, True
            self._finish_speech(
                upload.result,
                value={
                    "schema": SAMPLED_AUDIO_RESULT_SCHEMA,
                    "robot_id": ROBOT_ID,
                    "controller_id": CONTROLLER_ID,
                    "accepted": True,
                    "started": True,
                    "completed": False,
                    "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
                    "encoding": SAMPLED_AUDIO_ENCODING,
                    "byte_count": len(upload.payload),
                    "sample_count": receipt["sample_count"],
                    "duration_ms": receipt["duration_ms"],
                    "receipt": receipt,
                },
            )
            upload.deadline.finish_start()
            return None, True
        except asyncio.CancelledError:
            upload.deadline.finish_start()
            self._finish_speech(
                upload.result,
                error=BlastControllerError(
                    "controller_unavailable",
                    "BLAST controller stopped",
                    motion_started=False,
                ),
            )
            raise
        except Exception as error:
            upload.deadline.finish_start()
            cancelled = (
                upload.cancel_requested is not None
                and upload.cancel_requested()
            )
            start_timed_out = isinstance(error, BlastPCMStartTimeout)
            failure = BlastControllerError(
                (
                    "controller_command_interrupted"
                    if cancelled
                    else (
                        error.code
                        if start_timed_out
                        else "controller_command_failed"
                    )
                ),
                (
                    "BLAST speech was cancelled during playback"
                    if cancelled
                    else (
                        "BLAST sampled-audio start receipt timed out"
                        if start_timed_out
                        else "BLAST sampled-audio transport failed"
                    )
                ),
                motion_started=False,
            )
            logger.warning(
                "BLAST sampled-audio phase failed phase=%s offset=%d "
                "byte_count=%d code=%s error_type=%s",
                upload.current_phase,
                upload.offset,
                len(upload.payload),
                failure.code,
                type(error).__name__,
            )
            aligned = getattr(runtime, "sampled_audio_aligned", False) is True
            if aligned:
                try:
                    probe_limit = (
                        0.25 if self._fixed_work_pending() else 2.0
                    )
                    observation = await asyncio.wait_for(
                        runtime.observe(),
                        timeout=min(
                            probe_limit,
                            max(
                                0.1,
                                upload.deadline.remaining(),
                            ),
                        ),
                    )
                    self._publish_observation(observation)
                except Exception:
                    pass
                else:
                    self._finish_speech(upload.result, error=failure)
                    return None, True
            # A transport failure that also lost line-protocol alignment
            # requires a fresh owner session before admitted motor commands.
            self._set_state("offline", "sampled_audio_transport_failed")
            self._defer_commands_for_speech_reconnect()
            self._finish_speech(upload.result, error=failure)
            return None, False

    async def _execute_command(self, runtime, generation, request) -> bool:
        requested_generation, command, result, expires_at, permit = request
        if result.cancelled() or time.monotonic() >= expires_at:
            self._finish_command(result)
            return False
        remaining = expires_at - time.monotonic()
        if remaining < MIN_COMMAND_BUDGET_SECONDS:
            self._finish_command(
                result,
                error=BlastControllerError(
                    "controller_command_timeout",
                    "BLAST command expired before execution",
                ),
            )
            return False
        try:
            if requested_generation not in (None, generation):
                raise BlastControllerError(
                    "stale_controller_command",
                    "BLAST reconnected before the command ran",
                )
            with self._lock:
                stop_won_before_start = (
                    command != "stop"
                    and self._preempt_stop_request is not None
                )
            if stop_won_before_start:
                await self._service_preempt_stop(runtime, generation)
                self._finish_command(
                    result,
                    error=BlastControllerError(
                        "controller_command_interrupted",
                        "BLAST command was interrupted by stop",
                        motion_started=False,
                    ),
                )
                return False
            internal_timeout = (
                SCAN_INTERNAL_COMMAND_TIMEOUT_SECONDS
                if command in (SCAN_COMMAND, SURROUNDINGS_SCAN_COMMAND)
                else INTERNAL_COMMAND_TIMEOUT_SECONDS
            )
            value = await asyncio.wait_for(
                self._perform_command(
                    runtime, generation, command, action_permit=permit,
                ),
                timeout=min(
                    internal_timeout,
                    remaining - COMMAND_RESPONSE_MARGIN_SECONDS,
                ),
            )
            self._finish_command(result, value=value)
            return True
        except asyncio.CancelledError:
            self._finish_command(
                result,
                error=BlastControllerError(
                    "controller_unavailable",
                    "BLAST controller stopped",
                ),
            )
            raise
        except asyncio.TimeoutError:
            failure = BlastControllerError(
                "controller_command_timeout",
                "BLAST command did not finish in time",
            )
            self._set_state("offline", failure.code)
            self._finish_command(result, error=failure)
            raise failure
        except BlastCommandRejected as error:
            # A declined pose is not a lost connection. Restarting here also
            # kills unrelated audio that is already playing on the hub.
            self._finish_command(result, error=BlastControllerError(
                "controller_command_rejected", str(error),
            ))
            return False
        except Exception as error:
            failure = (
                error
                if isinstance(error, BlastControllerError)
                else BlastControllerError(
                    "controller_command_failed",
                    "BLAST command failed",
                )
            )
            if failure is not error:
                failure.__cause__ = error
            session_usable = failure.code in (
                "controller_busy",
                "controller_command_interrupted",
                "scan_start_clearance_unverified",
                "scan_sweep_clearance_lost",
                "scan_sweep_observation_unverified",
            )
            if not session_usable:
                self._set_state("offline", failure.code)
            self._finish_command(result, error=failure)
            if session_usable:
                return False
            raise failure

    def _scan_sweep_window_allows_continuation(self, observation) -> bool:
        samples = self._settling_samples
        if (
            observation.get("motion_active") is not False
            or not self._post_motion_window_is_idle(samples)
            or not (
                BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
                .range_sensor_extrinsics
                .matches_navigation_body_angle(_body_motor_angle(observation))
            )
        ):
            return False
        minimum = (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
            .minimum_rotation_clearance_mm()
        )
        return all(
            blast_range_state(sample.get("distance_mm")) == RANGE_STATE_NO_VALID_DISTANCE
            or (
                blast_range_state(sample.get("distance_mm")) == RANGE_STATE_MEASURED
                and sample["distance_mm"] > minimum
            )
            for sample in samples
        )

    async def _perform_command(
        self, runtime, generation, command: str, *, action_permit=None,
    ):
        if command == SCAN_COMMAND:
            return await _perform_scan_front_arc(self, runtime, generation, action_permit=action_permit,
            )
        if command == SURROUNDINGS_SCAN_COMMAND:
            return await _perform_scan_surroundings(self, runtime, generation, action_permit=action_permit,
            )
        if command == SETTLED_OBSERVATION_COMMAND:
            receipt = {"motion_started": False}
        elif command in GESTURE_POSES:
            before = await runtime.observe()
            if before.get("motion_active") is not False:
                raise BlastControllerError("controller_busy", "BLAST is moving")
            reference = {
                "claw": CLAW_CLOSED_MOTOR_ANGLE_DEG,
                "body": BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.range_sensor_extrinsics.navigation_body_motor_angle_deg,
            }
            for role, offset in GESTURE_POSES[command]:
                receipt = await runtime.set_pose(role, reference[role] + offset)
                await self._observe_until_idle(runtime, generation=generation, stop_only=False)
        else:
            operation, direction = COMMANDS[command]
            method = getattr(runtime, operation)
            receipt = (
                await method()
                if direction is None
                else await method(direction)
            )
        observation = await self._observe_until_idle(
            runtime,
            generation=generation,
            stop_only=command == "stop",
        )
        observation_settled = None
        turn_command = command.startswith("turn_")
        if command in NAVIGATION_MOTION_COMMANDS or command == SETTLED_OBSERVATION_COMMAND:
            observation, observation_settled = await self._observe_until_settled(
                runtime, generation=generation,
                initial_observation=observation,
                timeout_seconds=(
                    SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS
                    if turn_command else None
                ),
            )
        if turn_command:
            observation = dict(observation)
            observation["rotation_sweep_window_verified"] = (
                self._scan_sweep_window_allows_continuation(observation)
            )
        result = {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": command,
            "accepted": True,
            "completed": True,
            "receipt": receipt,
            "observation": observation,
        }
        if observation_settled is not None:
            result["observation_settled"] = observation_settled
        return result

    async def _observe_until_idle(
        self,
        runtime,
        *,
        generation,
        stop_only: bool,
    ):
        deadline = asyncio.get_running_loop().time() + (
            0.0 if stop_only else MOTION_TIMEOUT_SECONDS
        )
        while True:
            if (
                not stop_only
                and await self._service_preempt_stop(runtime, generation)
            ):
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST command was interrupted by stop",
                )
            observation = await runtime.observe()
            self._publish_observation(observation)
            if observation.get("motion_active") is False:
                return observation
            if stop_only or asyncio.get_running_loop().time() >= deadline:
                if not stop_only:
                    try:
                        await runtime.stop()
                    except Exception:
                        pass
                raise BlastControllerError(
                    "controller_motion_not_stopped",
                    "BLAST motion did not stop as expected",
                )
            await asyncio.sleep(MOTION_POLL_INTERVAL_SECONDS)

    @staticmethod
    def _post_motion_window_is_idle(samples) -> bool:
        return (
            len(samples) >= POST_MOTION_IDLE_SAMPLE_COUNT
            and all(sample.get("motion_active") is False for sample in samples)
        )

    async def _observe_until_settled(
        self,
        runtime,
        *,
        generation,
        initial_observation,
        timeout_seconds=None,
    ):
        # Keep the receipt's observation_settled field for compatibility. It
        # confirms idle sampling, not constant range/tilt or obstacle clearance.
        # Range validity and scan geometry are checked by their own consumers.
        settle_timeout = (
            POST_MOTION_SETTLE_TIMEOUT_SECONDS
            if timeout_seconds is None
            else timeout_seconds
        )
        deadline = (
            asyncio.get_running_loop().time()
            + settle_timeout
        )
        latest = initial_observation
        samples = []
        self._settling_samples = ()
        while True:
            if await self._service_preempt_stop(runtime, generation):
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST command was interrupted by stop",
                )
            if latest.get("motion_active") is False:
                samples.append(latest)
                del samples[:-POST_MOTION_IDLE_SAMPLE_COUNT]
            else:
                samples.clear()
            settled = self._post_motion_window_is_idle(samples)
            timed_out = asyncio.get_running_loop().time() >= deadline
            if settled or timed_out:
                self._settling_samples = tuple(samples)
                if not settled:
                    logger.info(
                        "BLAST post-motion idle unconfirmed: samples=%s/%s "
                        "motion_active=%s budget_seconds=%s",
                        len(samples), POST_MOTION_IDLE_SAMPLE_COUNT,
                        latest.get("motion_active"), settle_timeout,
                    )
                if await self._service_preempt_stop(runtime, generation):
                    raise BlastControllerError(
                        "controller_command_interrupted",
                        "BLAST command was interrupted by stop",
                    )
                # A missing confirmation retains an explicit quality flag;
                # it does not itself fail the navigation episode.
                return latest, settled
            await asyncio.sleep(MOTION_POLL_INTERVAL_SECONDS)
            latest = await runtime.observe()
            self._publish_observation(latest)

    async def _service_preempt_stop(self, runtime, generation) -> bool:
        with self._lock:
            request = self._preempt_stop_request
        if request is None:
            return False
        requested_generation, result, expires_at = request
        if result.cancelled() or time.monotonic() >= expires_at:
            self._finish_preempt_stop(
                result,
                error=(
                    None
                    if result.cancelled()
                    else BlastControllerError(
                        "controller_command_timeout",
                        "BLAST stop command expired before execution",
                    )
                ),
            )
            return False
        try:
            if requested_generation not in (None, generation):
                raise BlastControllerError(
                    "stale_controller_command",
                    "BLAST reconnected before stop ran",
                )
            self._finish_preempt_stop(
                result,
                value=await self._perform_command(
                    runtime,
                    generation,
                    "stop",
                ),
            )
            return True
        except asyncio.CancelledError:
            self._finish_preempt_stop(
                result,
                error=BlastControllerError(
                    "controller_unavailable",
                    "BLAST controller stopped",
                ),
            )
            raise
        except Exception as error:
            failure = (
                error
                if isinstance(error, BlastControllerError)
                else BlastControllerError(
                    "controller_command_failed",
                    "BLAST stop command failed",
                )
            )
            self._finish_preempt_stop(result, error=failure)
            raise failure

    def _publish_observation(self, observation) -> None:
        self._set_state(
            "online",
            None,
            observation=observation,
            last_observed_at_unix_ms=time.time_ns() // 1_000_000,
            last_observed_at_monotonic_ms=(
                time.monotonic_ns() // 1_000_000
            ),
        )

    def _finish_command(self, result, *, value=None, error=None):
        with self._lock:
            self._pending_command = None
            if not result.done():
                if error is None:
                    result.set_result(value)
                else:
                    result.set_exception(error)

    def _finish_speech(self, result, *, value=None, error=None):
        with self._lock:
            if self._pending_speech is result:
                self._pending_speech = None
            if not result.done():
                if error is None:
                    result.set_result(value)
                else:
                    result.set_exception(error)

    def _clear_timed_out_speech(self, result) -> None:
        """Release one timed-out speech slot and discard it if unclaimed."""

        with self._lock:
            if self._pending_speech is result:
                self._pending_speech = None
            try:
                request = self._speech_queue.get_nowait()
            except Empty:
                return
            if request[2] is not result:
                self._speech_queue.put_nowait(request)

    def _finish_preempt_stop(self, result, *, value=None, error=None):
        with self._lock:
            request = self._preempt_stop_request
            if request is not None and request[1] is result:
                self._preempt_stop_request = None
            if not result.done():
                if error is None:
                    result.set_result(value)
                else:
                    result.set_exception(error)

    def _defer_commands_for_speech_reconnect(self) -> None:
        """Retain unstarted fixed commands across a PCM-desynced session."""

        with self._lock:
            stop_request = self._preempt_stop_request
            if stop_request is not None:
                self._preempt_stop_request = (None,) + stop_request[1:]
            try:
                request = self._command_queue.get_nowait()
            except Empty:
                return
            generation, command, result, expires_at, permit = request
            if permit is None:
                request = (None, command, result, expires_at, permit)
            self._command_queue.put_nowait(request)
            self._command_available.set()

    def _reject_commands(self, error) -> None:
        with self._lock:
            stop_request = self._preempt_stop_request
        if stop_request is not None:
            if (
                stop_request[0] is None
                and not self._stop_requested.is_set()
                and not stop_request[1].cancelled()
                and time.monotonic() < stop_request[2]
            ):
                stop_request = None
            else:
                self._finish_preempt_stop(stop_request[1], error=error)
        deferred = None
        while True:
            request = self._next_command()
            if request is None:
                break
            if (
                request[0] is None
                and not self._stop_requested.is_set()
                and not request[2].cancelled()
                and time.monotonic() < request[3]
            ):
                deferred = request
                continue
            self._finish_command(
                request[2],
                error=error,
            )
        if deferred is not None:
            self._command_queue.put_nowait(deferred)
            self._command_available.set()
        while True:
            try:
                request = self._speech_queue.get_nowait()
            except Empty:
                return
            self._finish_speech(request[2], error=error)

    async def _wait_for_work(self, seconds: float) -> bool:
        await asyncio.to_thread(self._command_available.wait, seconds)
        self._command_available.clear()
        return self._stop_requested.is_set()

    async def _wait(self, seconds: float) -> bool:
        return await asyncio.to_thread(
            self._stop_requested.wait,
            seconds,
        )
