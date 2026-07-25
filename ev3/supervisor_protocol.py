#!/usr/bin/env python3
"""Strict, language-blind request protocol for the EV3 supervisor.

The protocol is deliberately transport independent.  SSH authenticates the
future foreground process; this module only validates small typed envelopes
and dispatches them to an already-created ``EV3Supervisor`` on its owner
thread.
"""

from __future__ import print_function

import json
import binascii
import os

try:
    from .supervisor import EV3Supervisor, SupervisorError
except (ImportError, ValueError, SystemError):
    from supervisor import EV3Supervisor, SupervisorError


PROTOCOL_VERSION = 1
RESPONSE_SCHEMA = "ev3-supervisor-response/v1"
MAX_FRAME_BYTES = 4096
MAX_TTL_MS = 1000

OP_STATUS = "status"
OP_DESCRIBE = "describe"
OP_CLAIM = "claim"
OP_HEARTBEAT = "heartbeat"
OP_ARM = "arm"
OP_DRIVE_TIMED = "drive_timed"
OP_RELEASE = "release"
OP_STOP = "stop"
OP_SHUTDOWN = "shutdown"

OPERATIONS = frozenset(
    (
        OP_STATUS,
        OP_DESCRIBE,
        OP_CLAIM,
        OP_HEARTBEAT,
        OP_ARM,
        OP_DRIVE_TIMED,
        OP_RELEASE,
        OP_STOP,
        OP_SHUTDOWN,
    )
)
STOP_OPERATIONS = frozenset((OP_STOP, OP_SHUTDOWN))

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
    OP_STATUS: frozenset(),
    OP_DESCRIBE: frozenset(),
    OP_CLAIM: frozenset(("owner_id",)),
    OP_HEARTBEAT: frozenset(("session_id", "sequence_id")),
    OP_ARM: frozenset(("session_id", "sequence_id")),
    OP_DRIVE_TIMED: frozenset(
        (
            "session_id",
            "sequence_id",
            "command_id",
            "reference_heartbeat_sequence",
            "left_speed_dps",
            "right_speed_dps",
            "duration_ms",
        )
    ),
    OP_RELEASE: frozenset(("session_id", "sequence_id")),
    OP_STOP: frozenset(),
    OP_SHUTDOWN: frozenset(),
}


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _safe_detail(value):
    try:
        detail = " ".join(str(value).split())
    except Exception:
        detail = "Protocol operation failed"
    return detail[:240]


class ProtocolError(ValueError):
    """A bounded, safely reportable request rejection."""

    def __init__(self, code, message, request_id=None, fatal=False):
        self.code = code
        self.request_id = request_id
        self.fatal = fatal
        ValueError.__init__(self, message)


class SupervisorRequest(object):
    """Validated request stamped only with the EV3 receive clock."""

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
    actual = frozenset(value)
    if actual != expected:
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
    """Decode one strict JSONL frame without consulting a host timestamp."""
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
    if value["protocol_version"] != PROTOCOL_VERSION:
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

    for name in ("owner_id", "session_id", "command_id"):
        if name in arguments:
            _identifier(
                name,
                arguments[name],
                128 if name != "owner_id" else 64,
                request_id=request_id,
            )
    for name in (
        "sequence_id",
        "reference_heartbeat_sequence",
    ):
        if name in arguments:
            _positive_int(
                name,
                arguments[name],
                2147483647,
                request_id=request_id,
            )
    for name in (
        "left_speed_dps",
        "right_speed_dps",
        "duration_ms",
    ):
        if name in arguments and not _is_int(arguments[name]):
            raise ProtocolError(
                "invalid_{}".format(name),
                "{} must be an integer".format(name),
                request_id=request_id,
            )

    return SupervisorRequest(
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
    return (
        json.dumps(
            response,
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
        + b"\n"
    )


class SupervisorProtocol(object):
    """Exact allowlist dispatcher for one physical controller."""

    def __init__(
        self,
        supervisor,
        controller_id,
        allow_motion=False,
        motion_budget=0,
        experiment_max_abs_speed_dps=100,
        experiment_max_duration_ms=300,
    ):
        if not isinstance(supervisor, EV3Supervisor):
            raise ProtocolError(
                "invalid_supervisor",
                "Protocol requires EV3Supervisor",
                fatal=True,
            )
        self.supervisor = supervisor
        self.controller_id = _identifier(
            "controller_id",
            controller_id,
            128,
        )
        if not isinstance(allow_motion, bool):
            raise ProtocolError(
                "invalid_motion_mode",
                "allow_motion must be boolean",
                fatal=True,
            )
        if not _is_int(motion_budget) or motion_budget < 0:
            raise ProtocolError(
                "invalid_motion_budget",
                "motion_budget is invalid",
                fatal=True,
            )
        self.allow_motion = allow_motion
        if (
            (allow_motion and motion_budget != 1)
            or (not allow_motion and motion_budget != 0)
        ):
            raise ProtocolError(
                "invalid_motion_budget",
                "Motion mode requires an exact one-shot budget",
                fatal=True,
            )
        self.remaining_motion_budget = motion_budget
        self.experiment_max_abs_speed_dps = _positive_int(
            "experiment_max_abs_speed_dps",
            experiment_max_abs_speed_dps,
            10000,
        )
        self.experiment_max_duration_ms = _positive_int(
            "experiment_max_duration_ms",
            experiment_max_duration_ms,
            60000,
        )
        try:
            self.robot_id = _identifier(
                "robot_id",
                supervisor.robot.config["robot_id"],
                128,
            )
        except (KeyError, TypeError):
            raise ProtocolError(
                "invalid_robot_id",
                "Supervisor configuration has no robot_id",
                fatal=True,
            )
        self.controller_instance_id = binascii.hexlify(
            os.urandom(16)
        ).decode("ascii")

    def _description(self):
        drive_speed_limit = min(
            self.supervisor.limits["drive_max_speed_dps"],
            self.experiment_max_abs_speed_dps,
        )
        drive_duration_limit = min(
            self.supervisor.limits["drive_max_duration_ms"],
            self.experiment_max_duration_ms,
        )
        return {
            "protocol_version": PROTOCOL_VERSION,
            "robot_id": self.robot_id,
            "controller_id": self.controller_id,
            "controller_instance_id": self.controller_instance_id,
            "motion_enabled": self.allow_motion,
            "remaining_motion_budget": self.remaining_motion_budget,
            "capabilities": {
                "status": {
                    "enabled": True,
                },
                "emergency_stop": {
                    "enabled": True,
                },
                "differential_drive_timed": {
                    "enabled": self.allow_motion,
                    "max_abs_speed_dps": drive_speed_limit,
                    "max_duration_ms": drive_duration_limit,
                },
            },
        }

    def _dispatch(self, request, cancellation_requested=None):
        arguments = request.arguments
        operation = request.operation
        if operation == OP_DESCRIBE:
            return self._description()
        if operation == OP_STATUS:
            return self.supervisor.status()
        if operation == OP_CLAIM:
            return self.supervisor.claim(arguments["owner_id"])
        if operation == OP_HEARTBEAT:
            return self.supervisor.heartbeat(
                arguments["session_id"],
                arguments["sequence_id"],
            )
        if operation == OP_ARM:
            return self.supervisor.arm(
                arguments["session_id"],
                arguments["sequence_id"],
            )
        if operation == OP_DRIVE_TIMED:
            if not self.allow_motion:
                raise ProtocolError(
                    "motion_disabled",
                    "This supervisor process was started motion-free",
                    request_id=request.request_id,
                )
            if self.remaining_motion_budget <= 0:
                raise ProtocolError(
                    "motion_budget_exhausted",
                    "The one-shot motion budget is exhausted",
                    request_id=request.request_id,
                )
            if (
                abs(arguments["left_speed_dps"])
                > self.experiment_max_abs_speed_dps
                or abs(arguments["right_speed_dps"])
                > self.experiment_max_abs_speed_dps
            ):
                raise ProtocolError(
                    "experiment_speed_limit",
                    "Drive speed exceeds the process experiment limit",
                    request_id=request.request_id,
                )
            if (
                arguments["duration_ms"]
                > self.experiment_max_duration_ms
            ):
                raise ProtocolError(
                    "experiment_duration_limit",
                    "Drive duration exceeds the process experiment limit",
                    request_id=request.request_id,
                )
            self.remaining_motion_budget -= 1
            def guard():
                if self.supervisor._now_ms() >= request.deadline_ms:
                    raise SupervisorError(
                        "stale_request",
                        "Request expired before motor start",
                    )
                if (
                    cancellation_requested is not None
                    and cancellation_requested()
                ):
                    raise SupervisorError(
                        "external_stop_requested",
                        "Stop or shutdown was requested before motor start",
                    )

            return self.supervisor.start_drive(
                arguments["session_id"],
                arguments["sequence_id"],
                arguments["command_id"],
                arguments["reference_heartbeat_sequence"],
                arguments["left_speed_dps"],
                arguments["right_speed_dps"],
                arguments["duration_ms"],
                external_start_guard=guard,
            )
        if operation == OP_RELEASE:
            return self.supervisor.release(
                arguments["session_id"],
                arguments["sequence_id"],
            )
        if operation == OP_STOP:
            return self.supervisor.stop()
        if operation == OP_SHUTDOWN:
            return self.supervisor.stop()
        raise ProtocolError(
            "unknown_operation",
            "Operation is not allowed",
            request_id=request.request_id,
        )

    def execute(
        self,
        request,
        dispatch_at_ms=None,
        cancellation_requested=None,
    ):
        if not isinstance(request, SupervisorRequest):
            raise ProtocolError(
                "invalid_request",
                "Dispatcher requires a validated request",
                fatal=True,
            )
        if (
            cancellation_requested is not None
            and not callable(cancellation_requested)
        ):
            raise ProtocolError(
                "invalid_cancellation_guard",
                "Cancellation guard must be callable",
                request_id=request.request_id,
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
            dispatch_at_ms = self.supervisor._now_ms()
        if not _is_int(dispatch_at_ms) or dispatch_at_ms < 0:
            raise ProtocolError(
                "invalid_dispatch_time",
                "Local dispatch time is invalid",
                request_id=request.request_id,
                fatal=True,
            )
        if (
            request.operation not in STOP_OPERATIONS
            and dispatch_at_ms >= request.deadline_ms
        ):
            return error_response(
                self.controller_id,
                ProtocolError(
                    "stale_request",
                    "Request expired in the local input queue",
                    request_id=request.request_id,
                ),
            )
        try:
            result = self._dispatch(
                request,
                cancellation_requested=cancellation_requested,
            )
        except (ProtocolError, SupervisorError) as error:
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
