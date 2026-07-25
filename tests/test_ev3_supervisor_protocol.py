import json
import unittest

from ev3.supervisor_protocol import (
    MAX_FRAME_BYTES,
    PROTOCOL_VERSION,
    ProtocolError,
    RESPONSE_SCHEMA,
    decode_request,
    encode_response,
    error_response,
)


CONTROLLER_ID = "ev3rstorm-01.ev3-main"


def request_value(operation="status", arguments=None, **overrides):
    value = {
        "protocol_version": PROTOCOL_VERSION,
        "controller_id": CONTROLLER_ID,
        "request_id": "request-1",
        "op": operation,
        "queue_ttl_ms": 250,
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


class SupervisorProtocolCodecTests(unittest.TestCase):
    def assert_protocol_error(self, code, raw, received_at_ms=10_000):
        with self.assertRaises(ProtocolError) as context:
            decode_request(raw, received_at_ms)
        self.assertEqual(context.exception.code, code)
        return context.exception

    def test_valid_request_uses_only_local_receive_time_for_deadline(self):
        request = decode_request(
            encoded(request_value()),
            received_at_ms=12_345,
        )

        self.assertEqual(request.controller_id, CONTROLLER_ID)
        self.assertEqual(request.operation, "status")
        self.assertEqual(request.received_at_ms, 12_345)
        self.assertEqual(request.deadline_ms, 12_595)
        self.assertFalse(hasattr(request, "issued_at_ms"))

    def test_envelope_is_exact_and_host_timestamp_is_not_accepted(self):
        extra = request_value(issued_at_ms=1)
        self.assert_protocol_error(
            "invalid_envelope_fields",
            encoded(extra),
        )

        missing = request_value()
        del missing["queue_ttl_ms"]
        self.assert_protocol_error(
            "invalid_envelope_fields",
            encoded(missing),
        )

    def test_unknown_operation_and_wrong_protocol_are_rejected(self):
        self.assert_protocol_error(
            "unknown_operation",
            encoded(request_value(operation="natural_language")),
        )
        self.assert_protocol_error(
            "unsupported_protocol_version",
            encoded(request_value(protocol_version=2)),
        )

    def test_operation_arguments_are_exact(self):
        self.assert_protocol_error(
            "invalid_arguments_fields",
            encoded(
                request_value(
                    operation="status",
                    arguments={"text": "kör framåt"},
                )
            ),
        )
        self.assert_protocol_error(
            "invalid_arguments_fields",
            encoded(
                request_value(
                    operation="heartbeat",
                    arguments={"session_id": "session-1"},
                )
            ),
        )

    def test_duplicate_keys_and_nonfinite_numbers_are_fatal(self):
        duplicate = (
            b'{"protocol_version":1,"controller_id":"'
            + CONTROLLER_ID.encode("ascii")
            + b'","request_id":"a","request_id":"b",'
            b'"op":"status","queue_ttl_ms":10,"args":{}}\n'
        )
        error = self.assert_protocol_error(
            "duplicate_json_key",
            duplicate,
        )
        self.assertTrue(error.fatal)

        error = self.assert_protocol_error(
            "invalid_json",
            (
                b'{"protocol_version":1,"controller_id":"x",'
                b'"request_id":"a","op":"status","queue_ttl_ms":NaN,'
                b'"args":{}}\n'
            ),
        )
        self.assertTrue(error.fatal)

    def test_invalid_utf8_oversize_and_embedded_lines_are_fatal(self):
        cases = (
            ("invalid_utf8", b"\xff\n"),
            ("frame_too_large", b"x" * (MAX_FRAME_BYTES + 1)),
            ("invalid_frame", b"{}\n{}\n"),
        )
        for code, raw in cases:
            error = self.assert_protocol_error(code, raw)
            self.assertTrue(error.fatal)

    def test_ttl_and_integer_fields_reject_bool(self):
        self.assert_protocol_error(
            "invalid_queue_ttl_ms",
            encoded(request_value(queue_ttl_ms=True)),
        )
        self.assert_protocol_error(
            "invalid_sequence_id",
            encoded(
                request_value(
                    operation="heartbeat",
                    arguments={
                        "session_id": "session-1",
                        "sequence_id": True,
                    },
                )
            ),
        )
        self.assert_protocol_error(
            "invalid_left_speed_dps",
            encoded(
                request_value(
                    operation="drive_timed",
                    arguments={
                        "session_id": "session-1",
                        "sequence_id": 3,
                        "command_id": "drive-1",
                        "reference_heartbeat_sequence": 1,
                        "left_speed_dps": True,
                        "right_speed_dps": 100,
                        "duration_ms": 300,
                    },
                )
            ),
        )

    def test_invalid_identifiers_are_bounded_and_request_id_is_preserved(self):
        error = self.assert_protocol_error(
            "invalid_controller_id",
            encoded(request_value(controller_id=" wrong ")),
        )
        self.assertEqual(error.request_id, "request-1")

        self.assert_protocol_error(
            "invalid_request_id",
            encoded(request_value(request_id="")),
        )

    def test_response_encoding_is_one_strict_json_line(self):
        response = {
            "schema": RESPONSE_SCHEMA,
            "request_id": "request-1",
            "controller_id": CONTROLLER_ID,
            "ok": True,
            "result": {"state": "DISARMED"},
        }
        wire = encode_response(response)

        self.assertTrue(wire.endswith(b"\n"))
        self.assertEqual(wire.count(b"\n"), 1)
        self.assertEqual(json.loads(wire), response)

    def test_error_response_never_echoes_the_request_body(self):
        error = ProtocolError(
            "bad_request",
            "safe explanation",
            request_id="request-1",
        )
        response = error_response(CONTROLLER_ID, error)

        self.assertFalse(response["ok"])
        self.assertEqual(response["error"]["code"], "bad_request")
        self.assertNotIn("args", json.dumps(response))


if __name__ == "__main__":
    unittest.main()
