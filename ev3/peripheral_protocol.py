#!/usr/bin/env python3
"""Strict, motor-free request protocol for persistent EV3 sensor reads.

The protocol is deliberately small and transport independent.  It can
describe the peripheral process, read one configured sensor, or shut the
session down.  Motor control, speech, shell execution, and networking are not
representable operations.
"""

from __future__ import print_function

import binascii
import json
import os

if __package__:
    from .robot_hal import RobotHAL, SafetyError, read_text
else:
    from robot_hal import RobotHAL, SafetyError, read_text


PROTOCOL_VERSION = 1
RESPONSE_SCHEMA = "ev3-peripheral-response/v1"
MAX_FRAME_BYTES = 2048
MAX_RESPONSE_BYTES = 4096
MAX_TTL_MS = 5000

OP_DESCRIBE = "describe"
OP_READ_SENSOR = "read_sensor"
OP_SHUTDOWN = "shutdown"

OPERATIONS = frozenset(
    (
        OP_DESCRIBE,
        OP_READ_SENSOR,
        OP_SHUTDOWN,
    )
)

_COMMON_FIELDS = frozenset(
    (
        "protocol_version",
        "controller_id",
        "request_id",
        "op",
        "queue_ttl_ms",
        "args",
    )
)
_ARGUMENT_FIELDS = {
    OP_DESCRIBE: frozenset(),
    OP_READ_SENSOR: frozenset(("role",)),
    OP_SHUTDOWN: frozenset(),
}


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _safe_detail(value):
    try:
        detail = " ".join(str(value).split())
    except Exception:
        detail = "Peripheral operation failed"
    return detail[:240]


class ProtocolError(ValueError):
    """A bounded error that is safe to encode in a protocol response."""

    def __init__(self, code, message, request_id=None, fatal=False):
        self.code = code
        self.request_id = request_id
        self.fatal = fatal
        ValueError.__init__(self, message)


class PeripheralRequest(object):
    """One validated request stamped in the EV3 monotonic clock domain."""

    def __init__(
        self,
        controller_id,
        request_id,
        operation,
        queue_ttl_ms,
        arguments,
        received_at_ms,
    ):
        self.controller_id = controller_id
        self.request_id = request_id
        self.operation = operation
        self.queue_ttl_ms = queue_ttl_ms
        self.arguments = dict(arguments)
        self.received_at_ms = received_at_ms
        self.deadline_ms = received_at_ms + queue_ttl_ms


def _identifier(name, value, maximum, request_id=None):
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or "\x00" in value
        or "\n" in value
        or "\r" in value
    ):
        raise ProtocolError(
            "invalid_{}".format(name),
            "{} is invalid".format(name),
            request_id=request_id,
        )
    return value


def _positive_int(name, value, maximum, request_id=None):
    if not _is_int(value) or value <= 0 or value > maximum:
        raise ProtocolError(
            "invalid_{}".format(name),
            "{} is invalid".format(name),
            request_id=request_id,
        )
    return value


def _exact_fields(value, expected, context, request_id=None):
    if not isinstance(value, dict):
        raise ProtocolError(
            "invalid_{}".format(context),
            "{} must be an object".format(context),
            request_id=request_id,
        )
    if frozenset(value) != expected:
        raise ProtocolError(
            "invalid_{}_fields".format(context),
            "{} fields do not match the operation schema".format(context),
            request_id=request_id,
        )


def _reject_constant(_value):
    raise ValueError("Non-finite JSON number")


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError(
                "duplicate_json_key",
                "JSON objects may not contain duplicate keys",
                fatal=True,
            )
        result[key] = value
    return result


def _decode_text(raw):
    if isinstance(raw, bytes):
        if len(raw) > MAX_FRAME_BYTES:
            raise ProtocolError(
                "frame_too_large",
                "Request frame exceeds the byte limit",
                fatal=True,
            )
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            raise ProtocolError(
                "invalid_utf8",
                "Request frame is not valid UTF-8",
                fatal=True,
            )
    elif isinstance(raw, str):
        try:
            size = len(raw.encode("utf-8"))
        except UnicodeEncodeError:
            raise ProtocolError(
                "invalid_utf8",
                "Request frame is not valid UTF-8",
                fatal=True,
            )
        if size > MAX_FRAME_BYTES:
            raise ProtocolError(
                "frame_too_large",
                "Request frame exceeds the byte limit",
                fatal=True,
            )
        text = raw
    else:
        raise ProtocolError(
            "invalid_frame_type",
            "Request frame must be bytes or text",
            fatal=True,
        )

    if text.endswith("\n"):
        text = text[:-1]
        if text.endswith("\r"):
            text = text[:-1]
    if not text or "\n" in text or "\r" in text or "\x00" in text:
        raise ProtocolError(
            "invalid_frame",
            "Request frame must contain one JSON object",
            fatal=True,
        )
    return text


def decode_request(raw, received_at_ms):
    """Decode one strict JSONL frame using only the local receive clock."""
    if not _is_int(received_at_ms) or received_at_ms < 0:
        raise ProtocolError(
            "invalid_receive_time",
            "Local receive time is invalid",
            fatal=True,
        )
    text = _decode_text(raw)
    try:
        value = json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except ProtocolError:
        raise
    except (TypeError, ValueError):
        raise ProtocolError(
            "invalid_json",
            "Request frame is not valid JSON",
            fatal=True,
        )

    _exact_fields(value, _COMMON_FIELDS, "envelope")
    request_id = _identifier(
        "request_id",
        value["request_id"],
        128,
    )
    if (
        not _is_int(value["protocol_version"])
        or value["protocol_version"] != PROTOCOL_VERSION
    ):
        raise ProtocolError(
            "unsupported_protocol_version",
            "Protocol version is not supported",
            request_id=request_id,
        )
    controller_id = _identifier(
        "controller_id",
        value["controller_id"],
        128,
        request_id=request_id,
    )
    operation = value["op"]
    if not isinstance(operation, str) or operation not in OPERATIONS:
        raise ProtocolError(
            "unknown_operation",
            "Operation is not allowed",
            request_id=request_id,
        )
    queue_ttl_ms = _positive_int(
        "queue_ttl_ms",
        value["queue_ttl_ms"],
        MAX_TTL_MS,
        request_id=request_id,
    )
    arguments = value["args"]
    _exact_fields(
        arguments,
        _ARGUMENT_FIELDS[operation],
        "arguments",
        request_id=request_id,
    )
    if operation == OP_READ_SENSOR:
        _identifier(
            "role",
            arguments["role"],
            64,
            request_id=request_id,
        )

    return PeripheralRequest(
        controller_id=controller_id,
        request_id=request_id,
        operation=operation,
        queue_ttl_ms=queue_ttl_ms,
        arguments=arguments,
        received_at_ms=received_at_ms,
    )


def success_response(request, controller_id, result):
    return {
        "schema": RESPONSE_SCHEMA,
        "request_id": request.request_id,
        "controller_id": controller_id,
        "ok": True,
        "result": result,
    }


def error_response(controller_id, error, request_id=None):
    resolved_request_id = request_id
    if resolved_request_id is None:
        resolved_request_id = getattr(error, "request_id", None)
    return {
        "schema": RESPONSE_SCHEMA,
        "request_id": resolved_request_id,
        "controller_id": controller_id,
        "ok": False,
        "error": {
            "code": getattr(error, "code", "protocol_error"),
            "message": _safe_detail(error),
        },
    }


def encode_response(response):
    try:
        wire = (
            json.dumps(
                response,
                allow_nan=False,
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            ).encode("utf-8")
            + b"\n"
        )
    except (TypeError, ValueError):
        raise ProtocolError(
            "invalid_response",
            "Response is not strict JSON",
            fatal=True,
        )
    if len(wire) > MAX_RESPONSE_BYTES:
        raise ProtocolError(
            "response_too_large",
            "Response exceeds the byte limit",
            fatal=True,
        )
    return wire


class PeripheralProtocol(object):
    """Exact dispatcher for configured sensor reads only."""

    def __init__(self, robot, instance_id=None):
        if not isinstance(robot, RobotHAL):
            raise ProtocolError(
                "invalid_hal",
                "Peripheral protocol requires RobotHAL",
                fatal=True,
            )
        self.robot = robot
        try:
            self.robot_id = _identifier(
                "robot_id",
                robot.config["robot_id"],
                128,
            )
            self.controller_id = _identifier(
                "controller_id",
                robot.config["controller_id"],
                128,
            )
            configured_sensors = robot.config["sensors"]
        except (KeyError, TypeError):
            raise ProtocolError(
                "invalid_peripheral_config",
                "Robot configuration is incomplete",
                fatal=True,
            )
        if not isinstance(configured_sensors, dict) or not configured_sensors:
            raise ProtocolError(
                "invalid_peripheral_config",
                "Robot configuration has no sensors",
                fatal=True,
            )
        if instance_id is None:
            instance_id = binascii.hexlify(os.urandom(16)).decode("ascii")
        self.instance_id = _identifier(
            "instance_id",
            instance_id,
            128,
        )
        self._bindings = {}
        try:
            for role in sorted(configured_sensors):
                _identifier("sensor_role", role, 64)
                configured = configured_sensors[role]
                port = _identifier(
                    "sensor_port",
                    configured["port"],
                    64,
                )
                driver = _identifier(
                    "sensor_driver",
                    configured["driver"],
                    128,
                )
                expected_mode = configured.get("mode")
                if expected_mode is not None:
                    expected_mode = _identifier(
                        "sensor_mode",
                        expected_mode,
                        64,
                    )
                path = robot._sensor_path_for_role(role)
                binding = {
                    "role": role,
                    "path": path,
                    "port": port,
                    "driver": driver,
                    "mode": expected_mode,
                }
                self._validate_binding(binding)
                self._bindings[role] = binding
        except (KeyError, TypeError, SafetyError, IOError, OSError, RuntimeError):
            raise ProtocolError(
                "sensor_binding_failed",
                "Configured sensors could not be bound safely",
                fatal=True,
            )

    @staticmethod
    def _address_matches(address, port):
        return address == port or address.endswith(":" + port)

    def _validate_binding(self, binding):
        path = binding["path"]
        address = read_text(os.path.join(path, "address"))
        driver = read_text(os.path.join(path, "driver_name"))
        mode = read_text(os.path.join(path, "mode"))
        if not self._address_matches(address, binding["port"]):
            raise SafetyError("Sensor address changed")
        if driver != binding["driver"]:
            raise SafetyError("Sensor driver changed")
        if binding["mode"] is not None and mode != binding["mode"]:
            raise SafetyError("Sensor mode changed")
        return mode

    def _description(self):
        return {
            "protocol_version": PROTOCOL_VERSION,
            "robot_id": self.robot_id,
            "controller_id": self.controller_id,
            "peripheral_instance_id": self.instance_id,
            "motion_enabled": False,
            "speech_enabled": False,
            "capabilities": {
                "configured_sensor_read": {
                    "enabled": True,
                    "roles": sorted(self._bindings),
                },
            },
        }

    def _read_sensor(self, role):
        binding = self._bindings.get(role)
        if binding is None:
            raise ProtocolError(
                "unknown_sensor_role",
                "Sensor role is not configured",
            )
        try:
            mode = self._validate_binding(binding)
            value = int(
                read_text(os.path.join(binding["path"], "value0"))
            )
            units = read_text(
                os.path.join(binding["path"], "units")
            )
        except (SafetyError, IOError, OSError, RuntimeError, ValueError):
            raise ProtocolError(
                "sensor_read_failed",
                "Sensor identity or value could not be verified",
            )
        return {
            "observed_monotonic_ms": int(
                self.robot.monotonic_fn() * 1000
            ),
            "role": role,
            "port": binding["port"],
            "driver": binding["driver"],
            "mode": mode,
            "value0": value,
            "units": units,
        }

    def execute(self, request, dispatch_at_ms=None):
        if not isinstance(request, PeripheralRequest):
            raise ProtocolError(
                "invalid_request",
                "Dispatcher requires a validated request",
                fatal=True,
            )
        if request.controller_id != self.controller_id:
            return error_response(
                self.controller_id,
                ProtocolError(
                    "wrong_controller",
                    "Request targets another controller",
                    request_id=request.request_id,
                ),
            )
        if dispatch_at_ms is None:
            dispatch_at_ms = int(self.robot.monotonic_fn() * 1000)
        if not _is_int(dispatch_at_ms) or dispatch_at_ms < 0:
            raise ProtocolError(
                "invalid_dispatch_time",
                "Local dispatch time is invalid",
                request_id=request.request_id,
                fatal=True,
            )
        if (
            request.operation != OP_SHUTDOWN
            and dispatch_at_ms >= request.deadline_ms
        ):
            return error_response(
                self.controller_id,
                ProtocolError(
                    "stale_request",
                    "Request expired before dispatch",
                    request_id=request.request_id,
                ),
            )
        try:
            if request.operation == OP_DESCRIBE:
                result = self._description()
            elif request.operation == OP_READ_SENSOR:
                result = self._read_sensor(
                    request.arguments["role"]
                )
            elif request.operation == OP_SHUTDOWN:
                result = {"status": "closed"}
            else:
                raise ProtocolError(
                    "unknown_operation",
                    "Operation is not allowed",
                    request_id=request.request_id,
                )
        except ProtocolError as error:
            error.request_id = request.request_id
            return error_response(
                self.controller_id,
                error,
                request_id=request.request_id,
            )
        return success_response(
            request,
            self.controller_id,
            result,
        )
