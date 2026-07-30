import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from robot_agent.supervisor_transport import (
    REMOTE_DAEMON,
    RESPONSE_SCHEMA,
    SupervisorRemoteError,
    SupervisorSSHChannelPoisonedError,
    SupervisorSSHConfigurationError,
    SupervisorSSHProtocolError,
    SupervisorSSHSession,
    SupervisorSSHTransportError,
    _decode_response,
    run_motion_free_supervisor_preflight,
)


CONTROLLER_ID = "ev3rstorm-01.ev3-main"
PROJECT_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "ev3rstorm.json"
FAKE_DAEMON = (
    PROJECT_ROOT
    / "tests"
    / "fixtures"
    / "run_fake_supervisor_daemon.py"
)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(value), encoding="ascii")


def create_fake_sysfs(root):
    root = Path(root)
    motors = (
        ("motor0", "outA", "lego-ev3-m-motor", 100),
        ("motor1", "outC", "lego-ev3-l-motor", 0),
        ("motor2", "outB", "lego-ev3-l-motor", 0),
    )
    for name, port, driver, position in motors:
        path = root / "tacho-motor" / name
        values = {
            "address": "ev3-ports:{}".format(port),
            "driver_name": driver,
            "position": position,
            "state": "",
            "max_speed": 1050,
            "speed_sp": 0,
            "time_sp": 0,
            "stop_action": "coast",
            "command": "",
        }
        for filename, value in values.items():
            write(path / filename, value)

    sensors = (
        ("sensor0", "in1", "lego-ev3-touch", "TOUCH", 0, ""),
        ("sensor1", "in4", "lego-ev3-ir", "IR-PROX", 50, "pct"),
        (
            "sensor2",
            "in3",
            "lego-ev3-color",
            "COL-REFLECT",
            4,
            "pct",
        ),
    )
    for name, port, driver, mode, value, units in sensors:
        path = root / "lego-sensor" / name
        values = {
            "address": "ev3-ports:{}".format(port),
            "driver_name": driver,
            "mode": mode,
            "value0": value,
            "units": units,
        }
        for filename, item in values.items():
            write(path / filename, item)


def response_bytes(
    request_id="request-1",
    controller_id=CONTROLLER_ID,
    ok=True,
    payload=None,
    **extra,
):
    value = {
        "schema": RESPONSE_SCHEMA,
        "request_id": request_id,
        "controller_id": controller_id,
        "ok": ok,
    }
    if ok:
        value["result"] = {} if payload is None else payload
    else:
        value["error"] = (
            {"code": "rejected", "message": "safe rejection"}
            if payload is None
            else payload
        )
    value.update(extra)
    return json.dumps(value, separators=(",", ":")).encode("utf-8") + b"\n"


class FakeWorkflowSession(SupervisorSSHSession):
    def __init__(self, fail_operation=None):
        self.controller_id = CONTROLLER_ID
        self.fail_operation = fail_operation
        self.calls = []
        self.state = "DISARMED"
        self.session_id = None
        self.closed_cleanly = False

    def request(self, operation, arguments=None, ttl_ms=500):
        arguments = {} if arguments is None else dict(arguments)
        self.calls.append((operation, arguments, ttl_ms))
        if operation == self.fail_operation:
            raise SupervisorRemoteError(
                "simulated_failure",
                "simulated remote rejection",
            )
        if operation == "describe":
            return {
                "robot_id": "ev3rstorm-01",
                "controller_id": CONTROLLER_ID,
                "controller_instance_id": "a" * 32,
                "motion_enabled": False,
                "remaining_motion_budget": 0,
                "capabilities": {
                    "status": {"enabled": True},
                    "emergency_stop": {"enabled": True},
                    "differential_drive_timed": {
                        "enabled": False,
                        "max_abs_speed_dps": 100,
                        "max_duration_ms": 300,
                    },
                },
            }
        if operation == "status":
            return {
                "state": self.state,
                "fault": None,
                "motion_allowed": self.state == "ARMED_IDLE",
                "session_active": self.session_id is not None,
                "touch": 0,
                "touch_released_samples": 3,
            }
        if operation == "claim":
            self.session_id = "server-session-secret"
            return {
                "status": "claimed",
                "state": self.state,
                "session_id": self.session_id,
            }
        if operation == "heartbeat":
            return {"status": "accepted"}
        if operation == "arm":
            self.state = "ARMED_IDLE"
            return {
                "state": self.state,
                "motion_allowed": True,
            }
        if operation in ("release", "stop"):
            self.state = "DISARMED"
            self.session_id = None
            return {
                "state": self.state,
                "motion_allowed": False,
            }
        if operation == "shutdown":
            self.state = "DISARMED"
            self.session_id = None
            return {
                "state": self.state,
                "motion_allowed": False,
            }
        raise AssertionError(operation)

    def wait_closed(self, timeout_seconds=3.0):
        self.closed_cleanly = True
        return 0


class EOFBytes:
    def readline(self, _maximum=-1):
        return b""

    def read(self, _maximum=-1):
        return b""

    def close(self):
        return None


class FailingOutput:
    def __init__(self):
        self.closed = False

    def readline(self, _maximum=-1):
        raise OSError("simulated reader failure")

    def close(self):
        self.closed = True


class ClosedInput:
    def write(self, _value):
        raise AssertionError("No write expected")

    def flush(self):
        return None

    def close(self):
        return None


class AlreadyExitedProcess:
    def __init__(self):
        self.stdin = ClosedInput()
        self.stdout = EOFBytes()
        self.stderr = EOFBytes()

    def poll(self):
        return 0

    def wait(self, timeout=None):
        return 0

    def terminate(self):
        raise AssertionError("terminate was not expected")

    def kill(self):
        raise AssertionError("kill was not expected")


class RecordingInput:
    def __init__(self, partial=False):
        self.partial = partial
        self.writes = []
        self.closed = False
        self.close_count = 0
        self._condition = threading.Condition()

    def write(self, value):
        with self._condition:
            if self.closed:
                raise ValueError("closed")
            self.writes.append(value)
            self._condition.notify_all()
            return len(value) - 1 if self.partial else len(value)

    def flush(self):
        if self.closed:
            raise ValueError("closed")

    def close(self):
        with self._condition:
            if not self.closed:
                self.closed = True
                self.close_count += 1
                self._condition.notify_all()

    def wait_for_writes(self, count, timeout=1.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while len(self.writes) < count:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
            return True


class BlockingRecordingInput(RecordingInput):
    def __init__(self):
        super().__init__()
        self.write_entered = threading.Event()
        self.release_write = threading.Event()
        self.poison_probe = lambda: False
        self.poisoned_before_write_return = None

    def write(self, value):
        written = super().write(value)
        self.write_entered.set()
        if not self.release_write.wait(timeout=2):
            raise OSError("test did not release blocked write")
        self.poisoned_before_write_return = self.poison_probe()
        return written


class PumpStateReleaseGate:
    def __init__(self, lock):
        self._lock = lock
        self.armed = False
        self.pump_released = threading.Event()
        self.resume_pump = threading.Event()

    def __enter__(self):
        self._lock.acquire()
        return self

    def __exit__(self, _error_type, _error, _traceback):
        self._lock.release()
        if (
            self.armed
            and threading.current_thread().name
            == "ev3-supervisor-stdout"
        ):
            self.pump_released.set()
            if not self.resume_pump.wait(timeout=2):
                raise RuntimeError("test did not release stdout pump")


class ControlledOutput:
    def __init__(self):
        self.items = queue.Queue()
        self.closed = False

    def send(self, value):
        self.items.put(value)

    def readline(self, _maximum=-1):
        value = self.items.get(timeout=2)
        return b"" if value is None else value

    def close(self):
        if not self.closed:
            self.closed = True
            self.items.put(None)


class ControlledProcess:
    def __init__(
        self,
        partial_write=False,
        input_stream=None,
        output_stream=None,
    ):
        self.stdin = (
            RecordingInput(partial=partial_write)
            if input_stream is None
            else input_stream
        )
        self.stdout = (
            ControlledOutput()
            if output_stream is None
            else output_stream
        )
        self.stderr = EOFBytes()

    def poll(self):
        return 0 if self.stdin.closed else None

    def wait(self, timeout=None):
        if self.stdin.closed:
            return 0
        raise subprocess.TimeoutExpired("fake", timeout)

    def terminate(self):
        self.stdin.close()

    def kill(self):
        self.stdin.close()


class ControlledProcessFactory:
    def __init__(
        self,
        partial_write=False,
        input_stream=None,
        output_stream=None,
    ):
        self.process = ControlledProcess(
            partial_write=partial_write,
            input_stream=input_stream,
            output_stream=output_stream,
        )

    def __call__(self, _argv, **_kwargs):
        return self.process


class RecordingProcessFactory:
    def __init__(self):
        self.argv = None
        self.kwargs = None

    def __call__(self, argv, **kwargs):
        self.argv = list(argv)
        self.kwargs = dict(kwargs)
        return AlreadyExitedProcess()


class SupervisorResponseTests(unittest.TestCase):
    def test_valid_success_and_error_shapes(self):
        success = _decode_response(
            response_bytes(payload={"state": "DISARMED"}),
            "request-1",
            CONTROLLER_ID,
        )
        self.assertTrue(success["ok"])

        error = _decode_response(
            response_bytes(ok=False),
            "request-1",
            CONTROLLER_ID,
        )
        self.assertFalse(error["ok"])

    def test_response_identity_and_exact_fields_are_required(self):
        invalid_frames = (
            response_bytes(request_id="wrong"),
            response_bytes(controller_id="wrong"),
            response_bytes(extra_field=True),
            b"[]\n",
            b"{}\n{}\n",
            b"\xff\n",
        )
        for raw in invalid_frames:
            with self.assertRaises(SupervisorSSHProtocolError):
                _decode_response(
                    raw,
                    "request-1",
                    CONTROLLER_ID,
                )

    def test_duplicate_keys_and_nonfinite_values_are_rejected(self):
        duplicate = (
            b'{"schema":"ev3-supervisor-response/v1",'
            b'"request_id":"request-1","request_id":"other",'
            b'"controller_id":"ev3rstorm-01.ev3-main",'
            b'"ok":true,"result":{}}\n'
        )
        nonfinite = (
            b'{"schema":"ev3-supervisor-response/v1",'
            b'"request_id":"request-1",'
            b'"controller_id":"ev3rstorm-01.ev3-main",'
            b'"ok":true,"result":{"value":NaN}}\n'
        )
        for raw in (duplicate, nonfinite):
            with self.assertRaises(SupervisorSSHProtocolError):
                _decode_response(
                    raw,
                    "request-1",
                    CONTROLLER_ID,
                )


class MotionFreeWorkflowTests(unittest.TestCase):
    def test_preflight_never_sends_a_motion_operation(self):
        session = FakeWorkflowSession()
        result = run_motion_free_supervisor_preflight(
            session,
            sleep_fn=lambda _seconds: None,
        )

        operations = [operation for operation, _, _ in session.calls]
        self.assertEqual(
            operations,
            [
                "describe",
                "status",
                "claim",
                "heartbeat",
                "status",
                "arm",
                "status",
                "release",
                "status",
                "shutdown",
            ],
        )
        self.assertNotIn("drive_timed", operations)
        self.assertEqual(result["motion_requests_sent"], 0)
        self.assertFalse(result["description"]["motion_enabled"])
        self.assertTrue(session.closed_cleanly)

    def test_failure_attempts_stop_and_shutdown(self):
        session = FakeWorkflowSession(fail_operation="arm")
        with self.assertRaises(SupervisorRemoteError):
            run_motion_free_supervisor_preflight(
                session,
                sleep_fn=lambda _seconds: None,
            )

        operations = [operation for operation, _, _ in session.calls]
        self.assertIn("stop", operations)
        self.assertEqual(operations[-1], "shutdown")
        self.assertNotIn("drive_timed", operations)

    def test_capability_mismatch_fails_before_claim(self):
        session = FakeWorkflowSession()
        real_request = session.request

        def mismatched(operation, arguments=None, ttl_ms=500):
            result = real_request(operation, arguments, ttl_ms)
            if operation == "describe":
                result["motion_enabled"] = True
            return result

        session.request = mismatched
        with self.assertRaises(SupervisorSSHProtocolError):
            run_motion_free_supervisor_preflight(
                session,
                sleep_fn=lambda _seconds: None,
            )

        operations = [operation for operation, _, _ in session.calls]
        self.assertEqual(operations, ["describe", "stop", "shutdown"])


class SupervisorSSHConstructionTests(unittest.TestCase):
    def test_remote_command_is_fixed_and_cannot_enable_motion(self):
        factory = RecordingProcessFactory()
        session = SupervisorSSHSession(
            "robot@ev3dev.local",
            CONTROLLER_ID,
            process_factory=factory,
        )
        try:
            self.assertEqual(factory.argv[0], "ssh")
            self.assertIn("BatchMode=yes", factory.argv)
            self.assertIn("StrictHostKeyChecking=yes", factory.argv)
            self.assertIn(REMOTE_DAEMON, factory.argv)
            self.assertNotIn("--allow-one-drive-test", factory.argv)
            self.assertIs(factory.kwargs["stdin"], subprocess.PIPE)
            self.assertIs(factory.kwargs["stdout"], subprocess.PIPE)
            self.assertIs(factory.kwargs["stderr"], subprocess.PIPE)
        finally:
            session.close()

    def test_target_validation_rejects_shell_and_option_syntax(self):
        factory = RecordingProcessFactory()
        for target in (
            "-oProxyCommand=bad",
            "robot@host;touch /tmp/bad",
            " robot@host",
        ):
            with self.assertRaises(
                SupervisorSSHConfigurationError
            ):
                SupervisorSSHSession(
                    target,
                    CONTROLLER_ID,
                    process_factory=factory,
                )


class SupervisorChannelPoisonTests(unittest.TestCase):
    def session(self, factory):
        session = SupervisorSSHSession(
            "robot@fake.local",
            CONTROLLER_ID,
            process_factory=factory,
            response_timeout_seconds=0.1,
            startup_response_timeout_seconds=0.1,
        )
        session._request_prefix = "fixed"
        return session

    def test_first_read_only_request_allows_slow_ev3_cold_start(self):
        factory = ControlledProcessFactory()
        session = SupervisorSSHSession(
            "robot@fake.local",
            CONTROLLER_ID,
            process_factory=factory,
            response_timeout_seconds=0.1,
            startup_response_timeout_seconds=0.5,
        )
        session._request_prefix = "cold"

        def send_cold_start_response():
            self.assertTrue(
                factory.process.stdin.wait_for_writes(1)
            )
            time.sleep(0.2)
            factory.process.stdout.send(
                response_bytes(
                    request_id="cold-1",
                    payload={"motion_enabled": False},
                )
            )

        responder = threading.Thread(
            target=send_cold_start_response
        )
        responder.start()
        try:
            self.assertEqual(
                session.request("describe"),
                {"motion_enabled": False},
            )
            responder.join(timeout=1)
            self.assertFalse(responder.is_alive())

            started_at = time.monotonic()
            with self.assertRaises(
                SupervisorSSHChannelPoisonedError
            ):
                session.request("status")
            self.assertLess(time.monotonic() - started_at, 0.3)
        finally:
            session.close()

    def test_timeout_poisons_and_late_response_cannot_shift_next_request(self):
        factory = ControlledProcessFactory()
        session = self.session(factory)
        try:
            with self.assertRaises(
                SupervisorSSHChannelPoisonedError
            ) as raised:
                session.request("status")
            self.assertTrue(raised.exception.outcome_unknown)
            self.assertEqual(raised.exception.request_id, "fixed-1")
            self.assertTrue(session.poisoned)
            self.assertEqual(len(factory.process.stdin.writes), 1)
            self.assertEqual(factory.process.stdin.close_count, 1)

            factory.process.stdout.send(
                response_bytes(request_id="fixed-1")
            )
            with self.assertRaises(
                SupervisorSSHChannelPoisonedError
            ):
                session.request("status")
            self.assertEqual(len(factory.process.stdin.writes), 1)
        finally:
            session.close()
        self.assertEqual(factory.process.stdin.close_count, 1)

    def test_request_id_mismatch_poisons_and_close_writes_no_cleanup(self):
        factory = ControlledProcessFactory()
        session = self.session(factory)
        factory.process.stdout.send(
            response_bytes(request_id="wrong")
        )
        with self.assertRaises(
            SupervisorSSHChannelPoisonedError
        ) as raised:
            session.request("status")
        self.assertTrue(raised.exception.outcome_unknown)
        self.assertEqual(len(factory.process.stdin.writes), 1)

        session.close()
        self.assertEqual(len(factory.process.stdin.writes), 1)

    def test_partial_write_poisons_with_unknown_outcome(self):
        factory = ControlledProcessFactory(partial_write=True)
        session = self.session(factory)
        try:
            with self.assertRaises(
                SupervisorSSHChannelPoisonedError
            ) as raised:
                session.request("status")
            self.assertTrue(raised.exception.outcome_unknown)
            self.assertTrue(session.poisoned)
        finally:
            session.close()

    def test_valid_remote_rejection_does_not_poison(self):
        factory = ControlledProcessFactory()
        session = self.session(factory)
        try:
            factory.process.stdout.send(
                response_bytes(
                    request_id="fixed-1",
                    ok=False,
                )
            )
            with self.assertRaises(SupervisorRemoteError):
                session.request("status")
            self.assertFalse(session.poisoned)

            factory.process.stdout.send(
                response_bytes(
                    request_id="fixed-2",
                    payload={"state": "DISARMED"},
                )
            )
            self.assertEqual(
                session.request("status")["state"],
                "DISARMED",
            )
            self.assertFalse(session.poisoned)
            self.assertEqual(len(factory.process.stdin.writes), 2)
        finally:
            session._shutdown_sent = True
            session.close()

    def test_local_oversize_rejection_does_not_write_or_poison(self):
        factory = ControlledProcessFactory()
        session = self.session(factory)
        try:
            with self.assertRaises(
                SupervisorSSHConfigurationError
            ):
                session.request(
                    "claim",
                    {"owner_id": "x" * 5000},
                )
            self.assertFalse(session.poisoned)
            self.assertEqual(factory.process.stdin.writes, [])
        finally:
            session._shutdown_sent = True
            session.close()

    def test_close_serializes_cleanup_and_rejects_parallel_motion(self):
        factory = ControlledProcessFactory()
        session = self.session(factory)
        close_thread = threading.Thread(target=session.close)
        close_thread.start()

        self.assertTrue(
            factory.process.stdin.wait_for_writes(1),
            "close did not send stop",
        )
        stop = json.loads(factory.process.stdin.writes[0])
        self.assertEqual(stop["op"], "stop")

        request_errors = []

        def request_motion():
            try:
                session.request(
                    "drive_timed",
                    {"left_speed_dps": 50, "right_speed_dps": 50},
                )
            except BaseException as error:
                request_errors.append(error)

        request_thread = threading.Thread(target=request_motion)
        request_thread.start()
        request_thread.join(timeout=0.5)
        self.assertFalse(request_thread.is_alive())
        self.assertEqual(len(request_errors), 1)
        self.assertIsInstance(
            request_errors[0],
            SupervisorSSHTransportError,
        )
        self.assertEqual(len(factory.process.stdin.writes), 1)

        factory.process.stdout.send(
            response_bytes(request_id=stop["request_id"])
        )
        self.assertTrue(
            factory.process.stdin.wait_for_writes(2),
            "close did not send shutdown",
        )
        shutdown = json.loads(factory.process.stdin.writes[1])
        self.assertEqual(shutdown["op"], "shutdown")
        factory.process.stdout.send(
            response_bytes(request_id=shutdown["request_id"])
        )
        close_thread.join(timeout=1)
        self.assertFalse(close_thread.is_alive())
        self.assertEqual(
            [
                json.loads(frame)["op"]
                for frame in factory.process.stdin.writes
            ],
            ["stop", "shutdown"],
        )

    def test_wait_closed_cannot_poison_an_inflight_writer(self):
        input_stream = BlockingRecordingInput()
        factory = ControlledProcessFactory(input_stream=input_stream)
        session = self.session(factory)
        input_stream.poison_probe = lambda: session.poisoned
        request_results = []
        request_errors = []

        def make_request():
            try:
                request_results.append(session.request("status"))
            except BaseException as error:
                request_errors.append(error)

        request_thread = threading.Thread(target=make_request)
        request_thread.start()
        self.assertTrue(input_stream.write_entered.wait(timeout=1))
        request = json.loads(input_stream.writes[0])
        factory.process.stdout.send(
            response_bytes(
                request_id=request["request_id"],
                payload={"state": "DISARMED"},
            )
        )

        wait_errors = []

        def wait_for_close():
            try:
                session.wait_closed(timeout_seconds=0.1)
            except BaseException as error:
                wait_errors.append(error)

        wait_thread = threading.Thread(target=wait_for_close)
        wait_thread.start()
        time.sleep(0.02)
        self.assertFalse(session.poisoned)
        self.assertTrue(wait_thread.is_alive())

        input_stream.release_write.set()
        request_thread.join(timeout=1)
        wait_thread.join(timeout=1)
        self.assertFalse(request_thread.is_alive())
        self.assertFalse(wait_thread.is_alive())
        self.assertEqual(request_results, [{"state": "DISARMED"}])
        self.assertEqual(request_errors, [])
        self.assertIs(
            input_stream.poisoned_before_write_return,
            False,
        )
        self.assertEqual(len(wait_errors), 1)
        self.assertIsInstance(
            wait_errors[0],
            SupervisorSSHChannelPoisonedError,
        )
        self.assertTrue(session.poisoned)
        session.close()

    def test_known_async_reader_failure_rejects_before_any_write(self):
        factory = ControlledProcessFactory(
            output_stream=FailingOutput()
        )
        session = self.session(factory)
        self.assertTrue(session._failed.wait(timeout=1))

        with self.assertRaises(SupervisorSSHChannelPoisonedError):
            session.request(
                "drive_timed",
                {"left_speed_dps": 50, "right_speed_dps": 50},
            )

        self.assertTrue(session.poisoned)
        self.assertEqual(factory.process.stdin.writes, [])
        session.close()

    def test_unsolicited_invalid_frame_rejects_before_any_write(self):
        factory = ControlledProcessFactory()
        session = self.session(factory)
        factory.process.stdout.send(b"{broken\n")
        self.assertTrue(session._failed.wait(timeout=1))

        with self.assertRaises(SupervisorSSHChannelPoisonedError):
            session.request(
                "drive_timed",
                {"left_speed_dps": 50, "right_speed_dps": 50},
            )

        self.assertEqual(factory.process.stdin.writes, [])
        session.close()

    def test_unsolicited_line_commits_failure_before_lock_release(self):
        factory = ControlledProcessFactory()
        session = self.session(factory)
        state_gate = PumpStateReleaseGate(session._state_lock)
        session._state_lock = state_gate
        state_gate.armed = True
        factory.process.stdout.send(
            response_bytes(request_id="unsolicited")
        )
        self.assertTrue(state_gate.pump_released.wait(timeout=1))

        try:
            with self.assertRaises(
                SupervisorSSHChannelPoisonedError
            ):
                session.request(
                    "drive_timed",
                    {"left_speed_dps": 50, "right_speed_dps": 50},
                )
            self.assertEqual(factory.process.stdin.writes, [])
        finally:
            state_gate.resume_pump.set()
            session.close()

    def test_duplicate_response_is_detected_before_a_future_write(self):
        input_stream = BlockingRecordingInput()
        factory = ControlledProcessFactory(input_stream=input_stream)
        session = self.session(factory)
        errors = []

        def make_request():
            try:
                session.request("status")
            except BaseException as error:
                errors.append(error)

        request_thread = threading.Thread(target=make_request)
        request_thread.start()
        self.assertTrue(input_stream.write_entered.wait(timeout=1))
        request = json.loads(input_stream.writes[0])
        response = response_bytes(request_id=request["request_id"])
        factory.process.stdout.send(response)
        factory.process.stdout.send(response)
        self.assertTrue(session._failed.wait(timeout=1))

        input_stream.release_write.set()
        request_thread.join(timeout=1)
        self.assertFalse(request_thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(
            errors[0],
            SupervisorSSHChannelPoisonedError,
        )

        with self.assertRaises(SupervisorSSHChannelPoisonedError):
            session.request(
                "drive_timed",
                {"left_speed_dps": 50, "right_speed_dps": 50},
            )
        self.assertEqual(len(input_stream.writes), 1)
        session.close()

    def test_successful_shutdown_closes_request_lifecycle(self):
        factory = ControlledProcessFactory()
        session = self.session(factory)
        factory.process.stdout.send(
            response_bytes(
                request_id="fixed-1",
                payload={"state": "DISARMED"},
            )
        )

        self.assertEqual(
            session.request("shutdown"),
            {"state": "DISARMED"},
        )
        with self.assertRaises(SupervisorSSHTransportError):
            session.request(
                "drive_timed",
                {"left_speed_dps": 50, "right_speed_dps": 50},
            )

        self.assertEqual(
            [
                json.loads(frame)["op"]
                for frame in factory.process.stdin.writes
            ],
            ["shutdown"],
        )
        session.close()


class ForegroundSubprocessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.sysfs_root = root / "class"
        create_fake_sysfs(self.sysfs_root)
        self.lock_path = root / "motors.lock"
        self.audit_path = root / "audit.jsonl"
        self.write_log = root / "writes.jsonl"
        self.processes = []

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=2)
        self.temp.cleanup()

    def process_factory(self, _remote_argv, **kwargs):
        environment = dict(os.environ)
        existing = environment.get("PYTHONPATH")
        environment["PYTHONPATH"] = (
            str(PROJECT_ROOT)
            if not existing
            else str(PROJECT_ROOT) + os.pathsep + existing
        )
        process = subprocess.Popen(
            [
                sys.executable,
                str(FAKE_DAEMON),
                "--config",
                str(CONFIG_PATH),
                "--sysfs-root",
                str(self.sysfs_root),
                "--lock-path",
                str(self.lock_path),
                "--audit-log",
                str(self.audit_path),
                "--write-log",
                str(self.write_log),
            ],
            env=environment,
            **kwargs,
        )
        self.processes.append(process)
        return process

    def session(self):
        return SupervisorSSHSession(
            "robot@fake.local",
            CONTROLLER_ID,
            process_factory=self.process_factory,
            response_timeout_seconds=2,
            remote_session_ms=10000,
        )

    def write_events(self):
        if not self.write_log.exists():
            return []
        return [
            json.loads(line)
            for line in self.write_log.read_text(
                encoding="utf-8"
            ).splitlines()
        ]

    def audit_events(self):
        return [
            json.loads(line)
            for line in self.audit_path.read_text(
                encoding="utf-8"
            ).splitlines()
        ]

    def assert_motion_free_closed(self):
        self.assertFalse(
            any(
                event["value"] == "run-timed"
                for event in self.write_events()
            )
        )
        events = self.audit_events()
        self.assertGreaterEqual(len(events), 2)
        self.assertEqual(events[0]["event"], "startup_complete")
        self.assertEqual(events[-1]["event"], "supervisor_closed")

    def test_real_pipes_complete_motion_free_handshake(self):
        session = self.session()
        try:
            result = run_motion_free_supervisor_preflight(
                session,
                sleep_fn=lambda _seconds: time.sleep(0.02),
            )
        finally:
            session.close()

        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["motion_requests_sent"], 0)
        self.assert_motion_free_closed()

    def test_real_stdin_eof_closes_supervisor_without_motion(self):
        session = self.session()
        try:
            description = session.request("describe")
            self.assertFalse(description["motion_enabled"])
            session._process.stdin.close()
            self.assertEqual(session.wait_closed(timeout_seconds=3), 0)
        finally:
            session._shutdown_sent = True
            session.close()

        self.assert_motion_free_closed()

    def test_real_malformed_frame_closes_without_motion(self):
        session = self.session()
        try:
            session._process.stdin.write(b"{broken json\n")
            session._process.stdin.flush()
            self.assertEqual(session.wait_closed(timeout_seconds=3), 0)
        finally:
            session._shutdown_sent = True
            session.close()

        self.assert_motion_free_closed()

    @unittest.skipUnless(
        hasattr(signal, "SIGTERM"),
        "SIGTERM is not available",
    )
    def test_real_sigterm_closes_supervisor_without_motion(self):
        session = self.session()
        try:
            session.request("status")
            session._process.send_signal(signal.SIGTERM)
            self.assertEqual(session.wait_closed(timeout_seconds=3), 0)
        finally:
            session._shutdown_sent = True
            session.close()

        self.assert_motion_free_closed()


if __name__ == "__main__":
    unittest.main()
