import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from robot_agent.supervisor_transport import (
    REMOTE_DAEMON,
    RESPONSE_SCHEMA,
    SupervisorRemoteError,
    SupervisorSSHConfigurationError,
    SupervisorSSHProtocolError,
    SupervisorSSHSession,
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
