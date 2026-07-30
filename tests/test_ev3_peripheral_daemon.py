import json
from pathlib import Path
import queue
import tempfile
import threading
import time
import unittest

from ev3.peripheral_daemon import (
    MAX_REQUESTS,
    MAX_SESSION_MS,
    PeripheralSession,
    SessionError,
    run_daemon,
)
from ev3.peripheral_protocol import (
    PROTOCOL_VERSION,
    PeripheralProtocol,
)
from ev3.robot_hal import RobotHAL


PROJECT_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "ev3rstorm.json"
CONTROLLER_ID = "ev3rstorm-01.ev3-main"


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(value), encoding="ascii")


def add_sensor(root, name, port, driver, mode, value, units=""):
    path = Path(root) / "lego-sensor" / name
    values = {
        "address": "ev3-ports:{}".format(port),
        "driver_name": driver,
        "mode": mode,
        "value0": value,
        "units": units,
    }
    for filename, item in values.items():
        write(path / filename, item)
    return path


def request_wire(
    operation="describe",
    arguments=None,
    request_id="request-1",
    controller_id=CONTROLLER_ID,
):
    return (
        json.dumps(
            {
                "protocol_version": PROTOCOL_VERSION,
                "controller_id": controller_id,
                "request_id": request_id,
                "op": operation,
                "queue_ttl_ms": 1000,
                "args": {} if arguments is None else arguments,
            },
            separators=(",", ":"),
        ).encode("utf-8")
        + b"\n"
    )


class InteractiveInput:
    def __init__(self):
        self.items = queue.Queue()

    def readline(self, _maximum):
        return self.items.get(timeout=2)

    def send(self, value):
        self.items.put(value)

    def close(self):
        self.items.put(b"")


class InteractiveOutput:
    def __init__(self, fail=False):
        self.items = queue.Queue()
        self.fail = fail

    def write(self, value):
        if self.fail:
            raise IOError("simulated output failure")
        self.items.put(value)

    def flush(self):
        return None

    def receive(self):
        return json.loads(
            self.items.get(timeout=2).decode("utf-8")
        )


class PeripheralDaemonTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.sysfs_root = Path(self.temp.name) / "class"
        add_sensor(
            self.sysfs_root,
            "sensor0",
            "in1",
            "lego-ev3-touch",
            "TOUCH",
            0,
        )
        self.infrared_path = add_sensor(
            self.sysfs_root,
            "sensor1",
            "in4",
            "lego-ev3-ir",
            "IR-PROX",
            52,
            "pct",
        )
        add_sensor(
            self.sysfs_root,
            "sensor2",
            "in3",
            "lego-ev3-color",
            "COL-REFLECT",
            8,
            "pct",
        )
        self.motor_traps = {}
        motor_path = self.sysfs_root / "tacho-motor" / "motor0"
        for filename, value in (
            ("command", "do-not-touch"),
            ("speed_sp", "unchanged-speed"),
            ("time_sp", "unchanged-time"),
            ("stop_action", "unchanged-stop-action"),
        ):
            path = motor_path / filename
            write(path, value)
            self.motor_traps[path] = value
        self.robot = RobotHAL(
            str(CONFIG_PATH),
            sysfs_root=str(self.sysfs_root),
        )

    def tearDown(self):
        self.temp.cleanup()

    def start(
        self,
        max_session_ms=5000,
        max_requests=16,
        output=None,
    ):
        input_stream = InteractiveInput()
        output_stream = (
            InteractiveOutput() if output is None else output
        )
        outcome = {}
        errors = []

        def target():
            try:
                outcome.update(
                    run_daemon(
                        self.robot,
                        input_stream,
                        output_stream,
                        max_session_ms=max_session_ms,
                        max_requests=max_requests,
                    )
                )
            except BaseException as error:
                errors.append(error)

        thread = threading.Thread(target=target)
        thread.start()
        return input_stream, output_stream, thread, outcome, errors

    def assert_finished(self, thread, errors):
        thread.join(timeout=2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertFalse(
            any(
                item.name == "ev3-peripheral-request-reader"
                for item in threading.enumerate()
            ),
            "Peripheral input reader leaked past session shutdown",
        )

    def assert_motor_traps_untouched(self):
        for path, expected in self.motor_traps.items():
            self.assertEqual(
                path.read_text(encoding="ascii"),
                expected,
            )

    def test_one_process_serves_repeated_fresh_reads_then_shutdown(self):
        (
            input_stream,
            output,
            thread,
            outcome,
            errors,
        ) = self.start()

        input_stream.send(request_wire(request_id="describe-1"))
        description = output.receive()
        self.assertTrue(description["ok"])
        self.assertFalse(description["result"]["motion_enabled"])
        self.assertFalse(description["result"]["speech_enabled"])

        input_stream.send(
            request_wire(
                "read_sensor",
                {"role": "infrared"},
                request_id="read-1",
            )
        )
        first = output.receive()
        self.assertEqual(first["result"]["value0"], 52)

        write(self.infrared_path / "value0", 19)
        input_stream.send(
            request_wire(
                "read_sensor",
                {"role": "infrared"},
                request_id="read-2",
            )
        )
        second = output.receive()
        self.assertEqual(second["result"]["value0"], 19)

        input_stream.send(
            request_wire(
                "shutdown",
                request_id="shutdown-1",
            )
        )
        shutdown = output.receive()
        self.assertTrue(shutdown["ok"])
        self.assertEqual(
            shutdown["result"],
            {"status": "closed"},
        )
        self.assert_finished(thread, errors)

        self.assertEqual(outcome["status"], "closed")
        self.assertEqual(outcome["close_reason"], "shutdown")
        self.assertIsNone(outcome["transport_failure"])
        self.assertEqual(outcome["requests_received"], 4)
        self.assert_motor_traps_untouched()

    def test_request_budget_rejects_next_frame_and_closes(self):
        (
            input_stream,
            output,
            thread,
            outcome,
            errors,
        ) = self.start(max_requests=2)

        input_stream.send(request_wire(request_id="describe-1"))
        self.assertTrue(output.receive()["ok"])
        input_stream.send(
            request_wire(
                "read_sensor",
                {"role": "touch"},
                request_id="read-1",
            )
        )
        self.assertTrue(output.receive()["ok"])
        input_stream.send(
            request_wire(
                "shutdown",
                request_id="shutdown-1",
            )
        )
        rejected = output.receive()
        self.assertFalse(rejected["ok"])
        self.assertEqual(
            rejected["error"]["code"],
            "request_budget_exhausted",
        )
        self.assert_finished(thread, errors)

        self.assertEqual(
            outcome["transport_failure"],
            "request_budget_exhausted",
        )
        self.assertEqual(outcome["requests_received"], 2)
        self.assert_motor_traps_untouched()

    def test_duplicate_request_id_is_fatal_before_redispatch(self):
        (
            input_stream,
            output,
            thread,
            outcome,
            errors,
        ) = self.start()
        frame = request_wire(request_id="same-request")

        input_stream.send(frame)
        self.assertTrue(output.receive()["ok"])
        input_stream.send(frame)
        rejected = output.receive()
        self.assertFalse(rejected["ok"])
        self.assertEqual(
            rejected["error"]["code"],
            "duplicate_request_id",
        )
        self.assert_finished(thread, errors)

        self.assertEqual(
            outcome["transport_failure"],
            "duplicate_request_id",
        )
        self.assertEqual(outcome["requests_received"], 2)
        self.assert_motor_traps_untouched()

    def test_fatal_malformed_or_truncated_frame_closes_session(self):
        malformed_frames = (
            (
                (
                    b'{"protocol_version":1,"controller_id":"'
                    + CONTROLLER_ID.encode("ascii")
                    + b'","request_id":"a","request_id":"b",'
                    b'"op":"describe","queue_ttl_ms":10,"args":{}}\n'
                ),
                "duplicate_json_key",
                1,
            ),
            (b"{}", "truncated_frame", 0),
        )
        for raw, expected_code, expected_count in malformed_frames:
            with self.subTest(code=expected_code):
                (
                    input_stream,
                    output,
                    thread,
                    outcome,
                    errors,
                ) = self.start()
                input_stream.send(raw)
                rejected = output.receive()
                self.assertFalse(rejected["ok"])
                self.assertEqual(
                    rejected["error"]["code"],
                    expected_code,
                )
                self.assert_finished(thread, errors)
                self.assertEqual(
                    outcome["transport_failure"],
                    expected_code,
                )
                self.assertEqual(
                    outcome["requests_received"],
                    expected_count,
                )
                self.assert_motor_traps_untouched()

    def test_wrong_controller_shutdown_does_not_close_session(self):
        (
            input_stream,
            output,
            thread,
            outcome,
            errors,
        ) = self.start()

        input_stream.send(
            request_wire(
                "shutdown",
                request_id="wrong-shutdown",
                controller_id="other.controller",
            )
        )
        rejected = output.receive()
        self.assertFalse(rejected["ok"])
        self.assertEqual(
            rejected["error"]["code"],
            "wrong_controller",
        )

        input_stream.send(request_wire(request_id="describe-after"))
        self.assertTrue(output.receive()["ok"])
        input_stream.send(
            request_wire(
                "shutdown",
                request_id="valid-shutdown",
            )
        )
        self.assertTrue(output.receive()["ok"])
        self.assert_finished(thread, errors)
        self.assertEqual(outcome["close_reason"], "shutdown")
        self.assertEqual(outcome["requests_received"], 3)
        self.assert_motor_traps_untouched()

    def test_rejected_request_id_is_reserved_against_reuse(self):
        (
            input_stream,
            output,
            thread,
            outcome,
            errors,
        ) = self.start()

        input_stream.send(
            request_wire(
                "not_allowed",
                request_id="reserved-id",
            )
        )
        first = output.receive()
        self.assertFalse(first["ok"])
        self.assertEqual(
            first["error"]["code"],
            "unknown_operation",
        )

        input_stream.send(
            request_wire(request_id="reserved-id")
        )
        duplicate = output.receive()
        self.assertFalse(duplicate["ok"])
        self.assertEqual(
            duplicate["error"]["code"],
            "duplicate_request_id",
        )
        self.assert_finished(thread, errors)
        self.assertEqual(
            outcome["transport_failure"],
            "duplicate_request_id",
        )
        self.assertEqual(outcome["requests_received"], 2)
        self.assert_motor_traps_untouched()

    def test_semantic_rejections_consume_request_budget(self):
        (
            input_stream,
            output,
            thread,
            outcome,
            errors,
        ) = self.start(max_requests=2)

        for index in range(2):
            input_stream.send(
                request_wire(
                    "not_allowed",
                    request_id="rejected-{}".format(index),
                )
            )
            self.assertEqual(
                output.receive()["error"]["code"],
                "unknown_operation",
            )

        input_stream.send(
            request_wire(
                "shutdown",
                request_id="over-budget",
            )
        )
        exhausted = output.receive()
        self.assertFalse(exhausted["ok"])
        self.assertEqual(
            exhausted["error"]["code"],
            "request_budget_exhausted",
        )
        self.assertIsNone(exhausted["request_id"])
        self.assert_finished(thread, errors)
        self.assertEqual(
            outcome["transport_failure"],
            "request_budget_exhausted",
        )
        self.assertEqual(outcome["requests_received"], 2)
        self.assert_motor_traps_untouched()

    def test_eof_and_output_failure_close_without_motor_access(self):
        (
            input_stream,
            _output,
            thread,
            outcome,
            errors,
        ) = self.start()
        input_stream.close()
        self.assert_finished(thread, errors)
        self.assertEqual(outcome["transport_failure"], "input_eof")
        self.assertEqual(outcome["requests_received"], 0)

        failing_output = InteractiveOutput(fail=True)
        (
            input_stream,
            _output,
            thread,
            outcome,
            errors,
        ) = self.start(output=failing_output)
        input_stream.send(request_wire())
        self.assert_finished(thread, errors)
        self.assertEqual(
            outcome["transport_failure"],
            "output_write_failed",
        )
        self.assertEqual(outcome["requests_received"], 1)
        self.assert_motor_traps_untouched()

    def test_session_deadline_expires_without_input(self):
        (
            _input_stream,
            _output,
            thread,
            outcome,
            errors,
        ) = self.start(max_session_ms=40)
        started = time.monotonic()
        self.assert_finished(thread, errors)

        self.assertLess(time.monotonic() - started, 1.0)
        self.assertEqual(outcome["close_reason"], "session_deadline")
        self.assertIsNone(outcome["transport_failure"])
        self.assertEqual(outcome["requests_received"], 0)
        self.assert_motor_traps_untouched()

    def test_constructor_rejects_unbounded_limits(self):
        protocol = PeripheralProtocol(
            self.robot,
            instance_id="test-instance",
        )
        for kwargs in (
            {"max_session_ms": MAX_SESSION_MS + 1},
            {"max_session_ms": True},
            {"max_requests": MAX_REQUESTS + 1},
            {"max_requests": True},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(SessionError):
                    PeripheralSession(
                        protocol,
                        InteractiveInput(),
                        InteractiveOutput(),
                        **kwargs
                    )


if __name__ == "__main__":
    unittest.main()
