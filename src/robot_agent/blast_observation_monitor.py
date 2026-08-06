"""Persistent BLAST telemetry and bounded manual dashboard controls."""

from __future__ import annotations

import asyncio
from concurrent.futures import Future, TimeoutError as FutureTimeoutError
from copy import deepcopy
import math
from queue import Empty, Full, Queue
import threading
import time
from typing import Callable, Optional

from .blast_ble_runtime import BlastBLERuntime, DEFAULT_HUB_NAME


SNAPSHOT_SCHEMA = "controller-runtime-observation/v1"
ROBOT_ID = "blast-01"
CONTROLLER_ID = "blast-01.hub"
DEFAULT_POLL_INTERVAL_SECONDS = 1.0
DEFAULT_RECONNECT_INTERVAL_SECONDS = 3.0
DISCONNECT_TIMEOUT_SECONDS = 3.0
COMMAND_TIMEOUT_SECONDS = 15.0
INTERNAL_COMMAND_TIMEOUT_SECONDS = 12.0
MIN_COMMAND_BUDGET_SECONDS = 2.0
COMMAND_RESPONSE_MARGIN_SECONDS = 0.25
MOTION_TIMEOUT_SECONDS = 4.0
MOTION_POLL_INTERVAL_SECONDS = 0.05
POST_MOTION_SETTLE_TIMEOUT_SECONDS = 1.5
POST_MOTION_SETTLE_SAMPLE_COUNT = 5
POST_MOTION_DISTANCE_RANGE_MM = 5.0
POST_MOTION_TILT_RANGE_DEG = 1.0
COMMAND_RESULT_SCHEMA = "controller-command-result/v1"
COMMANDS = {
    "drive_forward": ("drive_pulse", "forward"),
    "drive_reverse": ("drive_pulse", "reverse"),
    "turn_left": ("turn_pulse", "left"),
    "turn_right": ("turn_pulse", "right"),
    "claw_open": ("claw_pulse", "open"),
    "claw_close": ("claw_pulse", "close"),
    "body_left": ("body_pulse", "left"),
    "body_right": ("body_pulse", "right"),
    "stop": ("stop", None),
}
NAVIGATION_MOTION_COMMANDS = {
    "drive_forward",
    "drive_reverse",
    "turn_left",
    "turn_right",
}


class BlastControllerError(RuntimeError):
    """A bounded BLAST command could not be accepted or verified."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)


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
        self._lock = threading.RLock()
        self._stop_requested = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._loop = None
        self._task = None
        self._runtime_generation = 0
        self._pending_command = None
        self._preempt_stop_request = None
        self._command_queue = Queue(maxsize=1)
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
        with self._lock:
            if self._thread is not None:
                raise RuntimeError("BLAST observation monitor already started")
            self._stop_requested.clear()
            self._command_available.clear()
            self._set_state("connecting", "connecting")
            self._thread = threading.Thread(
                target=self._run,
                name="robot-llm-blast-observer",
                daemon=True,
            )
            self._thread.start()

    def snapshot(self):
        with self._lock:
            return deepcopy(self._snapshot)

    def command(self, command: str, *, cancel_requested=None):
        if command not in COMMANDS or (
            cancel_requested is not None
            and not callable(cancel_requested)
        ):
            raise ValueError("unsupported BLAST command")
        with self._lock:
            if self._snapshot["state"] != "online":
                raise BlastControllerError(
                    "controller_unavailable",
                    "BLAST is not connected",
                )
            if cancel_requested is not None and cancel_requested():
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST command was cancelled before motor start",
                )
            preempts_active_command = (
                command == "stop"
                and self._pending_command is not None
                and self._pending_command != "stop"
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
                    time.monotonic() + COMMAND_TIMEOUT_SECONDS,
                )
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
                result = Future()
                request = (
                    self._runtime_generation,
                    command,
                    result,
                    time.monotonic() + COMMAND_TIMEOUT_SECONDS,
                )
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
            return result.result(timeout=COMMAND_TIMEOUT_SECONDS)
        except FutureTimeoutError:
            result.cancel()
            raise BlastControllerError(
                "controller_command_timeout",
                "BLAST command did not finish in time",
            ) from None

    def close(self) -> None:
        with self._lock:
            thread = self._thread
        if thread is None:
            return
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
            self._set_state("online", None, ready=ready)
            while not self._stop_requested.is_set():
                preempted = await self._service_preempt_stop(
                    runtime,
                    generation,
                )
                request = self._next_command()
                if preempted and request is not None:
                    self._finish_command(
                        request[2],
                        error=BlastControllerError(
                            "controller_command_interrupted",
                            "BLAST command was interrupted by stop",
                        ),
                    )
                elif request is None:
                    if not preempted:
                        self._publish_observation(await runtime.observe())
                else:
                    await self._execute_command(
                        runtime,
                        generation,
                        request,
                    )
                if await self._wait_for_work(
                    self._poll_interval_seconds
                ):
                    break
        finally:
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

    async def _execute_command(self, runtime, generation, request) -> None:
        requested_generation, command, result, expires_at = request
        if result.cancelled() or time.monotonic() >= expires_at:
            self._finish_command(result)
            return
        remaining = expires_at - time.monotonic()
        if remaining < MIN_COMMAND_BUDGET_SECONDS:
            self._finish_command(
                result,
                error=BlastControllerError(
                    "controller_command_timeout",
                    "BLAST command expired before execution",
                ),
            )
            return
        try:
            if requested_generation != generation:
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
                    ),
                )
                return
            value = await asyncio.wait_for(
                self._perform_command(runtime, generation, command),
                timeout=min(
                    INTERNAL_COMMAND_TIMEOUT_SECONDS,
                    remaining - COMMAND_RESPONSE_MARGIN_SECONDS,
                ),
            )
            self._finish_command(result, value=value)
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
            self._finish_command(result, error=failure)
            raise failure
        except Exception as error:
            failure = (
                error
                if isinstance(error, BlastControllerError)
                else BlastControllerError(
                    "controller_command_failed",
                    "BLAST command failed",
                )
            )
            self._finish_command(result, error=failure)
            if failure.code == "controller_command_interrupted":
                return
            raise failure

    async def _perform_command(self, runtime, generation, command: str):
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
        if command in NAVIGATION_MOTION_COMMANDS:
            observation, observation_settled = (
                await self._observe_until_settled(
                    runtime,
                    generation=generation,
                    initial_observation=observation,
                )
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
    def _settling_sample(observation):
        distance = observation.get("distance_mm")
        imu = observation.get("imu")
        tilt = None
        if isinstance(imu, dict):
            for key in ("tilt_deg", "raw_tilt_deg"):
                candidate = imu.get(key)
                if isinstance(candidate, (list, tuple)) and len(candidate) == 2:
                    tilt = candidate
                    break
        values = (
            distance,
            *(tilt if isinstance(tilt, (list, tuple)) else ()),
        )
        if (
            len(values) != 3
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in values
            )
        ):
            return None
        return tuple(float(value) for value in values)

    @staticmethod
    def _settling_window_is_stable(samples) -> bool:
        if len(samples) < POST_MOTION_SETTLE_SAMPLE_COUNT:
            return False
        ranges = tuple(
            max(sample[index] for sample in samples)
            - min(sample[index] for sample in samples)
            for index in range(3)
        )
        return (
            ranges[0] <= POST_MOTION_DISTANCE_RANGE_MM
            and ranges[1] <= POST_MOTION_TILT_RANGE_DEG
            and ranges[2] <= POST_MOTION_TILT_RANGE_DEG
        )

    async def _observe_until_settled(
        self,
        runtime,
        *,
        generation,
        initial_observation,
    ):
        deadline = (
            asyncio.get_running_loop().time()
            + POST_MOTION_SETTLE_TIMEOUT_SECONDS
        )
        latest = initial_observation
        samples = []
        while True:
            if await self._service_preempt_stop(runtime, generation):
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST command was interrupted by stop",
                )
            sample = self._settling_sample(latest)
            if latest.get("motion_active") is False and sample is not None:
                samples.append(sample)
                del samples[:-POST_MOTION_SETTLE_SAMPLE_COUNT]
            else:
                samples.clear()
            settled = self._settling_window_is_stable(samples)
            timed_out = asyncio.get_running_loop().time() >= deadline
            if settled or timed_out:
                if await self._service_preempt_stop(runtime, generation):
                    raise BlastControllerError(
                        "controller_command_interrupted",
                        "BLAST command was interrupted by stop",
                    )
                # Stabilisation improves evidence quality, but must not turn a
                # usable bounded movement into a failed robot episode.
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
            if requested_generation != generation:
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

    def _reject_commands(self, error) -> None:
        with self._lock:
            stop_request = self._preempt_stop_request
        if stop_request is not None:
            self._finish_preempt_stop(stop_request[1], error=error)
        while True:
            request = self._next_command()
            if request is None:
                return
            self._finish_command(
                request[2],
                error=error,
            )

    async def _wait_for_work(self, seconds: float) -> bool:
        await asyncio.to_thread(self._command_available.wait, seconds)
        self._command_available.clear()
        return self._stop_requested.is_set()

    async def _wait(self, seconds: float) -> bool:
        return await asyncio.to_thread(
            self._stop_requested.wait,
            seconds,
        )
