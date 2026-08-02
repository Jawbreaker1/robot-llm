import json
import unittest

from robot_agent.ev3_navigation_transport import (
    EV3NavigationCommittedNotDispatchedError,
    EV3NavigationPreWriteError,
    EV3NavigationSSHTransport,
    EV3NavigationTransportError,
)


class RecordingStdin:
    def __init__(self, events, *, fail_write=False, fail_flush=False):
        self.events = events
        self.fail_write = fail_write
        self.fail_flush = fail_flush
        self.frames = []
        self.closed = False

    def write(self, frame):
        self.events.append("write")
        self.frames.append(frame)
        if self.fail_write:
            raise OSError("simulated write failure")
        return len(frame)

    def flush(self):
        self.events.append("flush")
        if self.fail_flush:
            raise OSError("simulated flush failure")

    def close(self):
        self.closed = True


class FakeProcess:
    def __init__(self, stdin):
        self.stdin = stdin
        self.terminated = False

    def terminate(self):
        self.terminated = True


def prepared_transport(*, fail_write=False, fail_flush=False):
    events = []
    stdin = RecordingStdin(
        events,
        fail_write=fail_write,
        fail_flush=fail_flush,
    )
    process = FakeProcess(stdin)
    transport = EV3NavigationSSHTransport(
        target="robot@ev3.local",
        controller_id="ev3-main",
        remote_worker_path="/home/robot/robot-llm/ev3/navigation_worker.py",
    )
    transport._process = process
    return transport, process, stdin, events


class EV3NavigationPreWriteTests(unittest.TestCase):
    def test_hook_runs_once_immediately_before_first_write(self):
        transport, _process, stdin, events = prepared_transport()
        transport._responses.put(("eof", None))

        def before_write(request_id):
            events.append("hook:{}".format(request_id))

        with self.assertRaises(EV3NavigationTransportError):
            transport.request(
                "observe",
                {},
                0.1,
                before_write=before_write,
            )

        self.assertEqual(
            events[:3],
            ["hook:host-0001", "write", "flush"],
        )
        self.assertEqual(len(stdin.frames), 1)

    def test_hook_failure_writes_nothing_and_transport_is_reusable(self):
        transport, _process, stdin, events = prepared_transport()

        def fail_before_write(request_id):
            events.append("hook:{}".format(request_id))
            raise RuntimeError("journal unavailable")

        with self.assertRaises(EV3NavigationPreWriteError) as caught:
            transport.request(
                "observe",
                {},
                0.1,
                before_write=fail_before_write,
            )

        self.assertEqual(caught.exception.request_id, "host-0001")
        self.assertEqual(events, ["hook:host-0001"])
        self.assertEqual(stdin.frames, [])
        self.assertFalse(transport.aborted)

        transport._responses.put(("eof", None))
        with self.assertRaises(EV3NavigationTransportError):
            transport.request("observe", {}, 0.1)
        request = json.loads(stdin.frames[0].decode("utf-8"))
        self.assertEqual(request["request_id"], "host-0002")

    def test_probe_failure_after_hook_is_explicit_and_writes_nothing(self):
        transport, _process, stdin, events = prepared_transport()
        probe_calls = [0]

        def cancellation_probe():
            probe_calls[0] += 1
            if probe_calls[0] == 1:
                return False
            raise RuntimeError("probe unavailable")

        with self.assertRaises(
            EV3NavigationCommittedNotDispatchedError
        ) as caught:
            transport.request(
                "pulse",
                {"action": "ADVANCE"},
                0.1,
                cancel_requested=cancellation_probe,
                before_write=lambda request_id: events.append(
                    "hook:{}".format(request_id)
                ),
            )

        self.assertEqual(caught.exception.request_id, "host-0001")
        self.assertEqual(
            caught.exception.reason,
            EV3NavigationCommittedNotDispatchedError.
            CANCELLATION_PROBE_FAILED,
        )
        self.assertFalse(caught.exception.write_attempted)
        self.assertEqual(stdin.frames, [])
        self.assertFalse(transport.aborted)

    def test_cancel_before_hook_writes_nothing(self):
        transport, _process, stdin, events = prepared_transport()

        with self.assertRaises(EV3NavigationPreWriteError) as caught:
            transport.request(
                "pulse",
                {"action": "ADVANCE"},
                0.1,
                cancel_requested=lambda: True,
                before_write=lambda _request_id: events.append("hook"),
            )

        self.assertEqual(caught.exception.request_id, "host-0001")
        self.assertEqual(events, [])
        self.assertEqual(stdin.frames, [])
        self.assertFalse(transport.aborted)

    def test_cancel_during_hook_is_committed_but_not_dispatched(self):
        transport, _process, stdin, events = prepared_transport()
        cancelled = [False]

        def commit_before_write(request_id):
            events.append("hook:{}".format(request_id))
            cancelled[0] = True

        with self.assertRaises(
            EV3NavigationCommittedNotDispatchedError
        ) as caught:
            transport.request(
                "pulse",
                {"action": "ADVANCE"},
                0.1,
                cancel_requested=lambda: cancelled[0],
                before_write=commit_before_write,
            )

        self.assertEqual(caught.exception.request_id, "host-0001")
        self.assertEqual(
            caught.exception.reason,
            EV3NavigationCommittedNotDispatchedError.CANCELLED,
        )
        self.assertTrue(caught.exception.record_committed)
        self.assertFalse(caught.exception.write_attempted)
        self.assertEqual(caught.exception.bytes_sent, 0)
        self.assertTrue(caught.exception.physical_outcome_known)
        self.assertTrue(caught.exception.transport_reusable)
        self.assertEqual(events, ["hook:host-0001"])
        self.assertEqual(stdin.frames, [])
        self.assertFalse(transport.aborted)

    def test_write_failure_after_hook_poisons_transport(self):
        transport, process, stdin, events = prepared_transport(
            fail_write=True
        )

        with self.assertRaises(EV3NavigationTransportError):
            transport.request(
                "observe",
                {},
                0.1,
                before_write=lambda request_id: events.append(
                    "hook:{}".format(request_id)
                ),
            )

        self.assertEqual(events, ["hook:host-0001", "write"])
        self.assertEqual(len(stdin.frames), 1)
        self.assertTrue(transport.aborted)
        self.assertTrue(process.terminated)

    def test_flush_failure_after_hook_poisons_transport(self):
        transport, process, stdin, events = prepared_transport(
            fail_flush=True
        )

        with self.assertRaises(EV3NavigationTransportError):
            transport.request(
                "observe",
                {},
                0.1,
                before_write=lambda request_id: events.append(
                    "hook:{}".format(request_id)
                ),
            )

        self.assertEqual(
            events,
            ["hook:host-0001", "write", "flush"],
        )
        self.assertEqual(len(stdin.frames), 1)
        self.assertTrue(transport.aborted)
        self.assertTrue(process.terminated)

    def test_invalid_hook_is_rejected_before_request_allocation(self):
        transport, _process, stdin, _events = prepared_transport()

        with self.assertRaisesRegex(
            EV3NavigationTransportError,
            "before-write hook is invalid",
        ):
            transport.request("observe", {}, 0.1, before_write=object())

        self.assertEqual(transport._sequence, 0)
        self.assertEqual(stdin.frames, [])


if __name__ == "__main__":
    unittest.main()
