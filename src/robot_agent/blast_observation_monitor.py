"""Background, read-only BLAST telemetry for the local dashboard."""

from __future__ import annotations

import asyncio
from copy import deepcopy
import math
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


class BlastObservationMonitor:
    """Own one BLE session and publish detached telemetry snapshots."""

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

    def close(self) -> None:
        with self._lock:
            thread = self._thread
        if thread is None:
            return
        self._stop_requested.set()
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
            self._set_state("online", None, ready=ready)
            while not self._stop_requested.is_set():
                observation = await runtime.observe()
                self._set_state(
                    "online",
                    None,
                    observation=observation,
                    last_observed_at_unix_ms=(
                        time.time_ns() // 1_000_000
                    ),
                    last_observed_at_monotonic_ms=(
                        time.monotonic_ns() // 1_000_000
                    ),
                )
                if await self._wait(self._poll_interval_seconds):
                    break
        finally:
            try:
                await asyncio.wait_for(
                    runtime.disconnect(),
                    timeout=DISCONNECT_TIMEOUT_SECONDS,
                )
            except Exception:
                pass

    async def _wait(self, seconds: float) -> bool:
        return await asyncio.to_thread(
            self._stop_requested.wait,
            seconds,
        )
