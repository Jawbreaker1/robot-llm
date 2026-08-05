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
        self._command_pending = False
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

    def command(self, command: str):
        if command not in COMMANDS:
            raise ValueError("unsupported BLAST command")
        with self._lock:
            if self._snapshot["state"] != "online":
                raise BlastControllerError(
                    "controller_unavailable",
                    "BLAST is not connected",
                )
            if self._command_pending:
                raise BlastControllerError(
                    "controller_busy",
                    "BLAST already has a pending command",
                )
            self._command_pending = True
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
                self._command_pending = False
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
                request = self._next_command()
                if request is None:
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
            value = await asyncio.wait_for(
                self._perform_command(runtime, command),
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
            raise failure

    async def _perform_command(self, runtime, command: str):
        operation, direction = COMMANDS[command]
        method = getattr(runtime, operation)
        receipt = (
            await method()
            if direction is None
            else await method(direction)
        )
        observation = await self._observe_until_idle(
            runtime,
            stop_only=command == "stop",
        )
        return {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": command,
            "accepted": True,
            "completed": True,
            "receipt": receipt,
            "observation": observation,
        }

    async def _observe_until_idle(self, runtime, *, stop_only: bool):
        deadline = asyncio.get_running_loop().time() + (
            0.0 if stop_only else MOTION_TIMEOUT_SECONDS
        )
        while True:
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
            self._command_pending = False
        if not result.done():
            if error is None:
                result.set_result(value)
            else:
                result.set_exception(error)

    def _reject_commands(self, error) -> None:
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
