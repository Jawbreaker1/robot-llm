import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from ev3.peripheral_protocol import (
    MAX_FRAME_BYTES,
    MAX_RESPONSE_BYTES,
    OPERATIONS,
    PROTOCOL_VERSION,
    RESPONSE_SCHEMA,
    PeripheralProtocol,
    ProtocolError,
    decode_request,
    encode_response,
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


def request_value(operation="describe", arguments=None, **overrides):
    value = {
        "protocol_version": PROTOCOL_VERSION,
        "controller_id": CONTROLLER_ID,
        "request_id": "request-1",
        "op": operation,
        "queue_ttl_ms": 500,
        "args": {} if arguments is None else arguments,
    }
    value.update(overrides)
    return value


def encoded(value):
    return (
        json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        + b"\n"
    )


class FakeClock:
    def __init__(self, now_ms=10_000):
        self.now_ms = now_ms

    def monotonic(self):
        return self.now_ms / 1000.0


class PeripheralProtocolTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.sysfs_root = Path(self.temp.name) / "class"
        self.paths = {
            "touch": add_sensor(
                self.sysfs_root,
                "sensor0",
                "in1",
                "lego-ev3-touch",
                "TOUCH",
                0,
            ),
            "infrared": add_sensor(
                self.sysfs_root,
                "sensor1",
                "in4",
                "lego-ev3-ir",
                "IR-PROX",
                51,
                "pct",
            ),
            "color": add_sensor(
                self.sysfs_root,
                "sensor2",
                "in3",
                "lego-ev3-color",
                "COL-REFLECT",
                7,
                "pct",
            ),
        }
        self.clock = FakeClock()
        self.robot = RobotHAL(
            str(CONFIG_PATH),
            sysfs_root=str(self.sysfs_root),
            monotonic_fn=self.clock.monotonic,
        )
        self.protocol = PeripheralProtocol(
            self.robot,
            instance_id="peripheral-instance-1",
        )

    def tearDown(self):
        self.temp.cleanup()

    def decode(self, operation="describe", arguments=None, **overrides):
        return decode_request(
            encoded(
                request_value(
                    operation=operation,
                    arguments=arguments,
                    **overrides
                )
            ),
            received_at_ms=self.clock.now_ms,
        )

    def assert_decode_error(self, code, raw):
        with self.assertRaises(ProtocolError) as context:
            decode_request(raw, received_at_ms=self.clock.now_ms)
        self.assertEqual(context.exception.code, code)
        return context.exception

    def test_operation_surface_is_exactly_motor_and_speech_free(self):
        self.assertEqual(
            OPERATIONS,
            frozenset(("describe", "read_sensor", "shutdown")),
        )
        for forbidden in (
            "drive",
            "drive_timed",
            "motor_test",
            "speak",
            "tts",
            "shell",
            "exec",
            "http",
            "listen",
        ):
            with self.subTest(operation=forbidden):
                error = self.assert_decode_error(
                    "unknown_operation",
                    encoded(request_value(operation=forbidden)),
                )
                self.assertFalse(error.fatal)

    def test_request_schema_is_exact_and_uses_local_receive_time(self):
        request = self.decode(
            "read_sensor",
            {"role": "infrared"},
        )

        self.assertEqual(request.operation, "read_sensor")
        self.assertEqual(request.arguments, {"role": "infrared"})
        self.assertEqual(request.received_at_ms, 10_000)
        self.assertEqual(request.deadline_ms, 10_500)
        self.assertFalse(hasattr(request, "issued_at_ms"))

        extra = request_value(host_timestamp_ms=1)
        self.assert_decode_error(
            "invalid_envelope_fields",
            encoded(extra),
        )
        self.assert_decode_error(
            "invalid_arguments_fields",
            encoded(
                request_value(
                    "read_sensor",
                    {"role": "infrared", "command": "anything"},
                )
            ),
        )

    def test_invalid_frames_and_duplicate_keys_are_fatal(self):
        duplicate = (
            b'{"protocol_version":1,"controller_id":"'
            + CONTROLLER_ID.encode("ascii")
            + b'","request_id":"a","request_id":"b",'
            b'"op":"describe","queue_ttl_ms":10,"args":{}}\n'
        )
        cases = (
            ("duplicate_json_key", duplicate),
            (
                "invalid_json",
                (
                    b'{"protocol_version":1,"controller_id":"x",'
                    b'"request_id":"a","op":"describe",'
                    b'"queue_ttl_ms":NaN,"args":{}}\n'
                ),
            ),
            ("invalid_utf8", b"\xff\n"),
            ("frame_too_large", b"x" * (MAX_FRAME_BYTES + 1)),
            ("invalid_frame", b"{}\n{}\n"),
        )
        for code, raw in cases:
            with self.subTest(code=code):
                error = self.assert_decode_error(code, raw)
                self.assertTrue(error.fatal)

    def test_ttl_role_and_protocol_values_are_bounded(self):
        self.assert_decode_error(
            "invalid_queue_ttl_ms",
            encoded(request_value(queue_ttl_ms=True)),
        )
        self.assert_decode_error(
            "invalid_role",
            encoded(
                request_value(
                    "read_sensor",
                    {"role": " infrared "},
                )
            ),
        )
        self.assert_decode_error(
            "unsupported_protocol_version",
            encoded(request_value(protocol_version=2)),
        )
        self.assert_decode_error(
            "unsupported_protocol_version",
            encoded(request_value(protocol_version=True)),
        )
        self.assert_decode_error(
            "unsupported_protocol_version",
            encoded(request_value(protocol_version=1.0)),
        )

    def test_description_explicitly_disables_motion_and_speech(self):
        response = self.protocol.execute(self.decode())

        self.assertTrue(response["ok"])
        result = response["result"]
        self.assertEqual(result["robot_id"], "ev3rstorm-01")
        self.assertEqual(result["controller_id"], CONTROLLER_ID)
        self.assertEqual(
            result["peripheral_instance_id"],
            "peripheral-instance-1",
        )
        self.assertIs(result["motion_enabled"], False)
        self.assertIs(result["speech_enabled"], False)
        self.assertEqual(
            result["capabilities"]["configured_sensor_read"]["roles"],
            ["color", "infrared", "touch"],
        )
        self.assertNotIn("motors", json.dumps(result))

    def test_repeated_read_uses_cached_verified_binding_and_fresh_value(self):
        first = self.protocol.execute(
            self.decode(
                "read_sensor",
                {"role": "infrared"},
            )
        )
        write(self.paths["infrared"] / "value0", 23)
        self.clock.now_ms += 25

        with patch.object(
            self.robot,
            "_sensor_path_for_role",
            side_effect=AssertionError("binding must remain cached"),
        ):
            second = self.protocol.execute(
                self.decode(
                    "read_sensor",
                    {"role": "infrared"},
                    request_id="request-2",
                )
            )

        self.assertEqual(first["result"]["value0"], 51)
        self.assertEqual(second["result"]["value0"], 23)
        self.assertEqual(
            second["result"]["observed_monotonic_ms"],
            10_025,
        )
        self.assertEqual(second["result"]["mode"], "IR-PROX")

    def test_changed_identity_or_mode_is_rejected_without_data(self):
        for filename, value in (
            ("driver_name", "unexpected-driver"),
            ("mode", "IR-SEEK"),
            ("address", "ev3-ports:in2"),
        ):
            with self.subTest(filename=filename):
                path = self.paths["infrared"] / filename
                original = path.read_text(encoding="ascii")
                write(path, value)
                response = self.protocol.execute(
                    self.decode(
                        "read_sensor",
                        {"role": "infrared"},
                    )
                )
                self.assertFalse(response["ok"])
                self.assertEqual(
                    response["error"]["code"],
                    "sensor_read_failed",
                )
                self.assertNotIn("result", response)
                write(path, original)

    def test_unknown_role_wrong_controller_and_stale_request_are_rejected(self):
        unknown = self.protocol.execute(
            self.decode(
                "read_sensor",
                {"role": "camera"},
            )
        )
        self.assertFalse(unknown["ok"])
        self.assertEqual(
            unknown["error"]["code"],
            "unknown_sensor_role",
        )

        wrong = self.protocol.execute(
            self.decode(controller_id="other.controller")
        )
        self.assertFalse(wrong["ok"])
        self.assertEqual(wrong["error"]["code"], "wrong_controller")

        request = self.decode()
        stale = self.protocol.execute(
            request,
            dispatch_at_ms=request.deadline_ms,
        )
        self.assertFalse(stale["ok"])
        self.assertEqual(stale["error"]["code"], "stale_request")

        shutdown = self.decode(
            "shutdown",
            request_id="shutdown-1",
        )
        accepted = self.protocol.execute(
            shutdown,
            dispatch_at_ms=shutdown.deadline_ms + 10_000,
        )
        self.assertTrue(accepted["ok"])
        self.assertEqual(accepted["result"], {"status": "closed"})

    def test_response_encoding_is_one_bounded_strict_json_line(self):
        response = self.protocol.execute(self.decode())
        wire = encode_response(response)

        self.assertLessEqual(len(wire), MAX_RESPONSE_BYTES)
        self.assertTrue(wire.endswith(b"\n"))
        self.assertEqual(wire.count(b"\n"), 1)
        self.assertEqual(json.loads(wire), response)
        self.assertEqual(response["schema"], RESPONSE_SCHEMA)

        with self.assertRaises(ProtocolError) as context:
            encode_response({"value": "x" * MAX_RESPONSE_BYTES})
        self.assertEqual(
            context.exception.code,
            "response_too_large",
        )


if __name__ == "__main__":
    unittest.main()
