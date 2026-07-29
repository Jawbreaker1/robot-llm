"""Cancelable single-flight execution for slow autonomy selectors.

The language-model call is deliberately isolated from goal and motor
authority.  One wrapper owns at most one daemon worker.  If a lease is
cancelled or its host deadline expires, the caller stops waiting immediately;
the worker's eventual result is discarded and no replacement worker may be
started until that worker has actually exited.
"""

from dataclasses import dataclass
import threading
import time
from typing import Callable, Optional

from .navigation_contract import (
    NavigationContractError,
    integer,
)


SELECTOR_COMPLETED = "COMPLETED"
SELECTOR_CANCELLED = "CANCELLED"
SELECTOR_DEADLINE_EXPIRED = "DEADLINE_EXPIRED"
SELECTOR_FAILED = "FAILED"
SELECTOR_BUSY = "BUSY"

SELECTOR_EXCEPTION = "SELECTOR_EXCEPTION"
SELECTOR_INVALID_PAYLOAD = "SELECTOR_INVALID_PAYLOAD"
SELECTOR_WORKER_FAILED = "SELECTOR_WORKER_FAILED"

_OUTCOME_STATUSES = (
    SELECTOR_COMPLETED,
    SELECTOR_CANCELLED,
    SELECTOR_DEADLINE_EXPIRED,
    SELECTOR_FAILED,
    SELECTOR_BUSY,
)
_FAILURE_KINDS = (
    SELECTOR_EXCEPTION,
    SELECTOR_INVALID_PAYLOAD,
    SELECTOR_WORKER_FAILED,
)


def _monotonic_ms() -> int:
    return int(time.monotonic() * 1_000)


@dataclass(frozen=True)
class SelectorCallOutcome:
    """Typed, exception-free result of one bounded selector wait."""

    status: str
    payload: Optional[bytes] = None
    failure_kind: Optional[str] = None

    def __post_init__(self) -> None:
        if self.status not in _OUTCOME_STATUSES:
            raise NavigationContractError(
                "invalid_selector_outcome",
                "Selector outcome status is invalid",
            )
        if self.status == SELECTOR_COMPLETED:
            if not isinstance(self.payload, bytes):
                raise NavigationContractError(
                    "invalid_selector_outcome",
                    "Completed selector outcome requires bytes",
                )
            if self.failure_kind is not None:
                raise NavigationContractError(
                    "invalid_selector_outcome",
                    "Completed selector outcome cannot carry failure",
                )
            return
        if self.payload is not None:
            raise NavigationContractError(
                "invalid_selector_outcome",
                "Non-completed selector outcome cannot carry payload",
            )
        if self.status == SELECTOR_FAILED:
            if self.failure_kind not in _FAILURE_KINDS:
                raise NavigationContractError(
                    "invalid_selector_outcome",
                    "Failed selector outcome requires failure kind",
                )
        elif self.failure_kind is not None:
            raise NavigationContractError(
                "invalid_selector_outcome",
                "Selector outcome cannot carry failure kind",
            )

    @property
    def completed(self) -> bool:
        return self.status == SELECTOR_COMPLETED


class _SelectorCallState:
    def __init__(self) -> None:
        self.done = threading.Event()
        self.lock = threading.Lock()
        self.thread: Optional[threading.Thread] = None
        self.payload: Optional[bytes] = None
        self.failure_kind: Optional[str] = None
        self.discarded = False

    def finish(
        self,
        payload: Optional[bytes],
        failure_kind: Optional[str],
    ) -> None:
        with self.lock:
            if not self.discarded:
                self.payload = payload
                self.failure_kind = failure_kind
        # This is the worker's final operation.  A waiter may safely join the
        # thread after observing ``done`` before allowing another call.
        self.done.set()

    def discard(self) -> None:
        with self.lock:
            self.discarded = True
            self.payload = None
            self.failure_kind = None

    def outcome(self) -> SelectorCallOutcome:
        with self.lock:
            if self.discarded:
                return SelectorCallOutcome(
                    status=SELECTOR_FAILED,
                    failure_kind=SELECTOR_WORKER_FAILED,
                )
            if self.failure_kind is not None:
                return SelectorCallOutcome(
                    status=SELECTOR_FAILED,
                    failure_kind=self.failure_kind,
                )
            if not isinstance(self.payload, bytes):
                return SelectorCallOutcome(
                    status=SELECTOR_FAILED,
                    failure_kind=SELECTOR_WORKER_FAILED,
                )
            return SelectorCallOutcome(
                status=SELECTOR_COMPLETED,
                payload=self.payload,
            )


class SingleFlightSelector:
    """Run one selector at a time without letting late calls accumulate."""

    def __init__(
        self,
        selector,
        clock_ms: Callable[[], int] = _monotonic_ms,
        poll_interval_ms: int = 10,
    ):
        if not callable(selector) or not callable(clock_ms):
            raise NavigationContractError(
                "invalid_selector_dependency",
                "Selector and host clock must be callable",
            )
        integer("poll_interval_ms", poll_interval_ms, 1, 1_000)
        self._selector = selector
        self._clock_ms = clock_ms
        self._poll_interval_ms = poll_interval_ms
        self._lock = threading.Lock()
        self._active: Optional[_SelectorCallState] = None

    def _now_ms(self) -> int:
        return integer(
            "selector_clock_ms",
            self._clock_ms(),
            0,
            2**63 - 1,
        )

    @staticmethod
    def _cancelled(cancel_event) -> bool:
        try:
            return bool(cancel_event.is_set())
        except Exception:
            raise NavigationContractError(
                "invalid_selector_cancel_event",
                "Selector cancellation event is invalid",
            ) from None

    def _worker(
        self,
        state: _SelectorCallState,
        context,
    ) -> None:
        payload = None
        failure_kind = None
        try:
            value = self._selector(context)
            if isinstance(value, bytes):
                payload = value
            else:
                failure_kind = SELECTOR_INVALID_PAYLOAD
        except BaseException:
            # The worker is an isolation boundary.  Never propagate selector
            # exceptions into threading's global exception hook, and never
            # retain the exception object after the call.
            failure_kind = SELECTOR_EXCEPTION
        state.finish(payload, failure_kind)

    def _start(self, context) -> Optional[_SelectorCallState]:
        with self._lock:
            active = self._active
            if active is not None:
                thread = active.thread
                if thread is not None and thread.is_alive():
                    return None
                if thread is not None:
                    thread.join()
                self._active = None

            state = _SelectorCallState()
            thread = threading.Thread(
                target=self._worker,
                args=(state, context),
                name="robot-autonomy-selector",
                daemon=True,
            )
            state.thread = thread
            self._active = state
            try:
                thread.start()
            except Exception:
                self._active = None
                return _SelectorCallState()
            return state

    def _clear(self, state: _SelectorCallState) -> None:
        thread = state.thread
        if thread is not None:
            thread.join()
        with self._lock:
            if self._active is state:
                self._active = None

    @property
    def busy(self) -> bool:
        """Whether this wrapper still owns a live selector worker."""

        with self._lock:
            active = self._active
            if active is None:
                return False
            thread = active.thread
            if thread is not None and thread.is_alive():
                return True
            if thread is not None:
                thread.join()
            self._active = None
            return False

    def call(
        self,
        context,
        cancel_event,
        deadline_ms: int,
    ) -> SelectorCallOutcome:
        """Wait for one result, cancellation, or the absolute host deadline.

        Cancellation has priority over a simultaneously completed result.  A
        deadline or cancellation marks the in-flight result discarded.  If
        that selector ignores cancellation and keeps running, later calls
        return ``BUSY`` rather than creating more daemon threads.
        """

        integer("selector_deadline_ms", deadline_ms, 0, 2**63 - 1)
        if not callable(getattr(cancel_event, "is_set", None)):
            raise NavigationContractError(
                "invalid_selector_cancel_event",
                "Selector cancellation event must expose is_set()",
            )
        if self._cancelled(cancel_event):
            return SelectorCallOutcome(status=SELECTOR_CANCELLED)
        started_at_ms = self._now_ms()
        if started_at_ms >= deadline_ms:
            return SelectorCallOutcome(
                status=SELECTOR_DEADLINE_EXPIRED
            )

        state = self._start(context)
        if state is None:
            return SelectorCallOutcome(status=SELECTOR_BUSY)
        if state.thread is None:
            return SelectorCallOutcome(
                status=SELECTOR_FAILED,
                failure_kind=SELECTOR_WORKER_FAILED,
            )

        # A second monotonic deadline prevents a broken or frozen injected
        # host clock from turning a bounded wait into an infinite one.
        wall_deadline = (
            time.monotonic()
            + (deadline_ms - started_at_ms) / 1_000.0
        )
        while True:
            if self._cancelled(cancel_event):
                state.discard()
                return SelectorCallOutcome(status=SELECTOR_CANCELLED)
            if (
                self._now_ms() >= deadline_ms
                or time.monotonic() >= wall_deadline
            ):
                state.discard()
                return SelectorCallOutcome(
                    status=SELECTOR_DEADLINE_EXPIRED
                )
            if state.done.is_set():
                self._clear(state)
                return state.outcome()

            remaining_wall_ms = max(
                1,
                int((wall_deadline - time.monotonic()) * 1_000),
            )
            state.done.wait(
                min(
                    self._poll_interval_ms,
                    remaining_wall_ms,
                )
                / 1_000.0
            )


__all__ = (
    "SELECTOR_BUSY",
    "SELECTOR_CANCELLED",
    "SELECTOR_COMPLETED",
    "SELECTOR_DEADLINE_EXPIRED",
    "SELECTOR_EXCEPTION",
    "SELECTOR_FAILED",
    "SELECTOR_INVALID_PAYLOAD",
    "SELECTOR_WORKER_FAILED",
    "SelectorCallOutcome",
    "SingleFlightSelector",
)
