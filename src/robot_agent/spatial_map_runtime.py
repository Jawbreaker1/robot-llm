"""Non-blocking observation relay and asynchronous spatial-map worker.

Navigation publishes immutable snapshots through :meth:`offer_nowait`.
All ray projection, occupancy fusion, object clustering, and dashboard
serialization happen away from the motion tick.
"""

from collections import deque
import copy
from dataclasses import dataclass
import threading
import time
from typing import Callable, Deque, Mapping, Optional, Set, Tuple

from .navigation_contract import NavigationContractError, integer
from .navigation_state import NavigationSnapshot
from .spatial_dashboard import spatial_dashboard_view
from .spatial_mapping import BoundedOccupancyGrid


def _monotonic_ms() -> int:
    return int(time.monotonic() * 1_000)


def _unix_ms() -> int:
    return int(time.time() * 1_000)


@dataclass(frozen=True)
class SpatialMapRuntimeState:
    """Bounded relay health without exposing its write capability."""

    publication_sequence: int
    settled_sequence: int
    applied_updates: int
    ignored_updates: int
    dropped_total: int
    failure_total: int
    rejected_total: int
    queue_depth: int
    accepting: bool
    worker_alive: bool
    last_applied_received_at_ms: Optional[int]

    @property
    def incomplete(self) -> bool:
        return (
            self.dropped_total > 0
            or self.failure_total > 0
            or self.rejected_total > 0
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "publication_sequence": self.publication_sequence,
            "settled_sequence": self.settled_sequence,
            "applied_updates": self.applied_updates,
            "ignored_updates": self.ignored_updates,
            "dropped_total": self.dropped_total,
            "failure_total": self.failure_total,
            "rejected_total": self.rejected_total,
            "queue_depth": self.queue_depth,
            "accepting": self.accepting,
            "worker_alive": self.worker_alive,
            "incomplete": self.incomplete,
            "last_applied_received_at_ms": (
                self.last_applied_received_at_ms
            ),
        }


class SpatialMapRuntime:
    """Own one map worker and a drop-oldest bounded snapshot relay."""

    def __init__(
        self,
        grid: BoundedOccupancyGrid,
        queue_capacity: int = 64,
        ray_ttl_ms: int = 5_000,
        monotonic_clock_ms: Callable[[], int] = _monotonic_ms,
        unix_clock_ms: Callable[[], int] = _unix_ms,
    ):
        if not isinstance(grid, BoundedOccupancyGrid):
            raise NavigationContractError(
                "invalid_spatial_grid",
                "Spatial map runtime requires BoundedOccupancyGrid",
            )
        integer("queue_capacity", queue_capacity, 1, 10_000)
        integer("ray_ttl_ms", ray_ttl_ms, 1, 60_000)
        if not callable(monotonic_clock_ms) or not callable(
            unix_clock_ms
        ):
            raise NavigationContractError(
                "invalid_spatial_runtime_clock",
                "Spatial map runtime clocks must be callable",
            )
        self.grid = grid
        self.queue_capacity = queue_capacity
        self.ray_ttl_ms = ray_ttl_ms
        self._monotonic_clock_ms = monotonic_clock_ms
        self._unix_clock_ms = unix_clock_ms
        self._queue: Deque[
            Tuple[int, NavigationSnapshot, int]
        ] = deque()
        # Accepted publications remain pending while queued or in-flight.
        # The set is bounded by queue_capacity plus the single worker item.
        self._pending_sequences: Set[int] = set()
        self._condition = threading.Condition()
        self._publication_sequence = 0
        self._settled_sequence = 0
        self._applied_updates = 0
        self._ignored_updates = 0
        self._dropped_total = 0
        self._failure_total = 0
        self._rejected_total = 0
        self._last_applied_received_at_ms: Optional[int] = None
        self._accepting = True
        self._stop_requested = False
        self._latest_dashboard_view = spatial_dashboard_view(
            self.grid.snapshot(),
            now_unix_ms=self._now_unix_ms(),
            observed_age_ms=None,
            ray_ttl_ms=self.ray_ttl_ms,
        )
        self._worker = threading.Thread(
            target=self._work,
            name="robot-spatial-map",
            daemon=True,
        )
        self._worker.start()

    def _advance_settled_locked(self) -> None:
        """Advance only across a contiguous prefix no longer pending."""

        while (
            self._settled_sequence < self._publication_sequence
            and self._settled_sequence + 1
            not in self._pending_sequences
        ):
            self._settled_sequence += 1

    def _now_monotonic_ms(self) -> int:
        return integer(
            "spatial_runtime_monotonic_ms",
            self._monotonic_clock_ms(),
            0,
            2**63 - 1,
        )

    def _now_unix_ms(self) -> int:
        return integer(
            "spatial_runtime_unix_ms",
            self._unix_clock_ms(),
            0,
            2**63 - 1,
        )

    def offer_nowait(self, snapshot: NavigationSnapshot) -> bool:
        """Offer one snapshot in O(1); never run mapping on this thread."""

        if not isinstance(snapshot, NavigationSnapshot):
            with self._condition:
                self._rejected_total += 1
            return False
        try:
            received_at_ms = self._now_monotonic_ms()
        except Exception:
            with self._condition:
                self._rejected_total += 1
            return False
        with self._condition:
            if not self._accepting:
                self._rejected_total += 1
                return False
            self._publication_sequence += 1
            sequence = self._publication_sequence
            self._pending_sequences.add(sequence)
            if len(self._queue) >= self.queue_capacity:
                dropped_sequence, _snapshot, _received_at_ms = (
                    self._queue.popleft()
                )
                self._pending_sequences.remove(dropped_sequence)
                self._dropped_total += 1
                self._advance_settled_locked()
            self._queue.append(
                (sequence, snapshot, received_at_ms)
            )
            self._condition.notify()
        return True

    def _work(self) -> None:
        while True:
            with self._condition:
                while not self._queue and not self._stop_requested:
                    self._condition.wait()
                if not self._queue and self._stop_requested:
                    return
                sequence, snapshot, received_at_ms = (
                    self._queue.popleft()
                )
            applied = False
            ignored = False
            failed = False
            projected_view = None
            try:
                update = self.grid.ingest(snapshot)
                if update.applied:
                    applied = True
                    now_monotonic_ms = self._now_monotonic_ms()
                    projected_view = spatial_dashboard_view(
                        self.grid.snapshot(),
                        now_unix_ms=self._now_unix_ms(),
                        observed_age_ms=max(
                            0,
                            now_monotonic_ms - received_at_ms,
                        ),
                        ray_ttl_ms=self.ray_ttl_ms,
                    )
                else:
                    ignored = True
            except Exception:
                failed = True
            with self._condition:
                if applied:
                    self._applied_updates += 1
                    self._last_applied_received_at_ms = (
                        received_at_ms
                    )
                if projected_view is not None:
                    self._latest_dashboard_view = projected_view
                if ignored:
                    self._ignored_updates += 1
                if failed:
                    self._failure_total += 1
                self._pending_sequences.remove(sequence)
                self._advance_settled_locked()
                self._condition.notify_all()

    def _state_locked(self) -> SpatialMapRuntimeState:
        return SpatialMapRuntimeState(
            publication_sequence=self._publication_sequence,
            settled_sequence=self._settled_sequence,
            applied_updates=self._applied_updates,
            ignored_updates=self._ignored_updates,
            dropped_total=self._dropped_total,
            failure_total=self._failure_total,
            rejected_total=self._rejected_total,
            queue_depth=len(self._queue),
            accepting=self._accepting,
            worker_alive=self._worker.is_alive(),
            last_applied_received_at_ms=(
                self._last_applied_received_at_ms
            ),
        )

    def state(self) -> SpatialMapRuntimeState:
        with self._condition:
            return self._state_locked()

    def flush(self, timeout_s: float = 5.0) -> bool:
        """Wait for all observations offered before this call to settle."""

        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not 0 < float(timeout_s) <= 60
        ):
            raise ValueError("Spatial map flush timeout is invalid")
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            target = self._publication_sequence
            while self._settled_sequence < target:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
        return True

    def raw_snapshot(self):
        """Return the immutable core snapshot for non-UI consumers."""

        return self.grid.snapshot()

    def snapshot(self) -> Mapping[str, object]:
        """Return a detached-friendly, read-only dashboard projection."""

        with self._condition:
            runtime = self._state_locked()
            cached_view = self._latest_dashboard_view
        view = copy.deepcopy(cached_view)
        if runtime.last_applied_received_at_ms is not None:
            current_age_ms = max(
                0,
                self._now_monotonic_ms()
                - runtime.last_applied_received_at_ms,
            )
            view["observed_age_ms"] = current_age_ms
            view["age_ms"] = current_age_ms
        if runtime.failure_total:
            view["status"] = "degraded"
            view["reason_code"] = "mapping_failure"
        elif runtime.dropped_total:
            view["status"] = "degraded"
            view["reason_code"] = "observation_gap"
        elif runtime.rejected_total:
            view["status"] = "degraded"
            view["reason_code"] = "observation_rejected"
        view["runtime"] = runtime.to_dict()
        return view

    def close(
        self,
        drain: bool = True,
        timeout_s: float = 5.0,
    ) -> bool:
        """Stop accepting work and join the worker within a finite timeout."""

        if type(drain) is not bool:
            raise ValueError("Spatial map drain flag is invalid")
        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not 0 < float(timeout_s) <= 60
        ):
            raise ValueError("Spatial map close timeout is invalid")
        with self._condition:
            self._accepting = False
            if not drain and self._queue:
                self._dropped_total += len(self._queue)
                for sequence, _snapshot, _received_at_ms in self._queue:
                    self._pending_sequences.remove(sequence)
                self._queue.clear()
                self._advance_settled_locked()
            self._stop_requested = True
            self._condition.notify_all()
        self._worker.join(float(timeout_s))
        return not self._worker.is_alive()

    def __enter__(self):
        return self

    def __exit__(self, _error_type, _error, _traceback):
        self.close()
