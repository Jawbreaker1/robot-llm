import threading
import time
import unittest

from robot_agent.autonomy_selector import (
    SELECTOR_BUSY,
    SELECTOR_CANCELLED,
    SELECTOR_COMPLETED,
    SELECTOR_DEADLINE_EXPIRED,
    SELECTOR_EXCEPTION,
    SELECTOR_FAILED,
    SELECTOR_INVALID_PAYLOAD,
    SelectorCallOutcome,
    SingleFlightSelector,
)
from robot_agent.navigation_contract import NavigationContractError


def host_now_ms():
    return int(time.monotonic() * 1_000)


def wait_until_reaped(gate, timeout_seconds=1.0):
    deadline = time.monotonic() + timeout_seconds
    while gate.busy and time.monotonic() < deadline:
        time.sleep(0.005)
    return not gate.busy


class BlockingSelector:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.lock = threading.Lock()
        self.calls = 0
        self.active = 0
        self.max_active = 0
        self.daemon_flags = []

    def __call__(self, _context):
        with self.lock:
            self.calls += 1
            call = self.calls
            self.active += 1
            self.max_active = max(self.max_active, self.active)
            self.daemon_flags.append(threading.current_thread().daemon)
        try:
            if call == 1:
                self.entered.set()
                self.release.wait()
                return b"late-first-result"
            return b"fresh-result"
        finally:
            with self.lock:
                self.active -= 1


class SingleFlightSelectorTests(unittest.TestCase):
    def test_completed_call_runs_on_one_daemon_worker(self):
        daemon_flags = []

        def selector(context):
            daemon_flags.append(threading.current_thread().daemon)
            return ("result-" + context).encode("ascii")

        gate = SingleFlightSelector(selector)

        outcome = gate.call(
            "one",
            threading.Event(),
            host_now_ms() + 1_000,
        )

        self.assertEqual(outcome.status, SELECTOR_COMPLETED)
        self.assertEqual(outcome.payload, b"result-one")
        self.assertTrue(outcome.completed)
        self.assertEqual(daemon_flags, [True])
        self.assertFalse(gate.busy)

    def test_cancel_stops_waiting_drops_late_result_and_blocks_fanout(self):
        selector = BlockingSelector()
        gate = SingleFlightSelector(selector, poll_interval_ms=2)
        cancel_event = threading.Event()

        def cancel_after_entry():
            self.assertTrue(selector.entered.wait(1))
            cancel_event.set()

        canceller = threading.Thread(target=cancel_after_entry)
        canceller.start()
        outcome = gate.call(
            "first",
            cancel_event,
            host_now_ms() + 1_000,
        )
        canceller.join()

        self.assertEqual(outcome.status, SELECTOR_CANCELLED)
        self.assertTrue(gate.busy)
        busy = gate.call(
            "must-not-start",
            threading.Event(),
            host_now_ms() + 1_000,
        )
        self.assertEqual(busy.status, SELECTOR_BUSY)
        self.assertEqual(selector.calls, 1)
        self.assertEqual(selector.max_active, 1)

        selector.release.set()
        self.assertTrue(wait_until_reaped(gate))
        fresh = gate.call(
            "second",
            threading.Event(),
            host_now_ms() + 1_000,
        )

        self.assertEqual(fresh.status, SELECTOR_COMPLETED)
        self.assertEqual(fresh.payload, b"fresh-result")
        self.assertNotEqual(fresh.payload, b"late-first-result")
        self.assertEqual(selector.calls, 2)
        self.assertEqual(selector.max_active, 1)
        self.assertEqual(selector.daemon_flags, [True, True])

    def test_deadline_uses_wall_fallback_when_host_clock_freezes(self):
        selector = BlockingSelector()
        gate = SingleFlightSelector(
            selector,
            clock_ms=lambda: 10_000,
            poll_interval_ms=2,
        )
        started = time.monotonic()

        outcome = gate.call(
            "frozen-clock",
            threading.Event(),
            10_030,
        )
        elapsed = time.monotonic() - started

        self.assertEqual(
            outcome.status,
            SELECTOR_DEADLINE_EXPIRED,
        )
        self.assertLess(elapsed, 0.5)
        self.assertTrue(gate.busy)
        selector.release.set()
        self.assertTrue(wait_until_reaped(gate))

    def test_pre_cancel_and_pre_expired_deadline_start_no_worker(self):
        calls = []

        def selector(_context):
            calls.append("called")
            return b"result"

        gate = SingleFlightSelector(selector, clock_ms=lambda: 100)
        cancelled = threading.Event()
        cancelled.set()

        self.assertEqual(
            gate.call("one", cancelled, 200).status,
            SELECTOR_CANCELLED,
        )
        self.assertEqual(
            gate.call("two", threading.Event(), 100).status,
            SELECTOR_DEADLINE_EXPIRED,
        )
        self.assertEqual(calls, [])
        self.assertFalse(gate.busy)

    def test_selector_failures_are_typed_and_not_rethrown(self):
        def raises(_context):
            raise RuntimeError("untrusted selector detail")

        cases = (
            (
                raises,
                SELECTOR_EXCEPTION,
            ),
            (
                lambda _context: "not-bytes",
                SELECTOR_INVALID_PAYLOAD,
            ),
        )
        for selector, expected_kind in cases:
            with self.subTest(expected_kind=expected_kind):
                outcome = SingleFlightSelector(selector).call(
                    "context",
                    threading.Event(),
                    host_now_ms() + 1_000,
                )

                self.assertEqual(outcome.status, SELECTOR_FAILED)
                self.assertEqual(
                    outcome.failure_kind,
                    expected_kind,
                )
                self.assertIsNone(outcome.payload)

    def test_outcome_contract_rejects_impossible_combinations(self):
        invalid = (
            {"status": "UNKNOWN"},
            {"status": SELECTOR_COMPLETED},
            {
                "status": SELECTOR_CANCELLED,
                "payload": b"late",
            },
            {"status": SELECTOR_FAILED},
        )
        for value in invalid:
            with self.subTest(value=value):
                with self.assertRaises(NavigationContractError):
                    SelectorCallOutcome(**value)


if __name__ == "__main__":
    unittest.main()
