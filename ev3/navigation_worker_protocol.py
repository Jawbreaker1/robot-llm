#!/usr/bin/env python3
"""Strict JSONL protocol used by the bounded EV3 navigation worker."""

from __future__ import print_function

import copy
import json
import sys

if __package__:
    from .navigation_profile import (
        ACTION_SPECS,
        ALLOWED_OPERATIONS,
        MAX_FRAME_BYTES,
        REQUEST_SCHEMA,
        RESPONSE_SCHEMA,
        SCAN_TURN_ALLOWED_DELTAS_MDEG,
    )
else:
    from navigation_profile import (
        ACTION_SPECS,
        ALLOWED_OPERATIONS,
        MAX_FRAME_BYTES,
        REQUEST_SCHEMA,
        RESPONSE_SCHEMA,
        SCAN_TURN_ALLOWED_DELTAS_MDEG,
    )


REQUEST_FIELDS = frozenset(
    ("schema", "controller_id", "request_id", "op", "args")
)


class WorkerError(RuntimeError):
    """Bounded protocol or worker failure suitable for a JSON error frame."""

    def __init__(self, code, message, request_id=None, fatal=False):
        self.code = code
        self.request_id = request_id
        self.fatal = fatal
        RuntimeError.__init__(self, message)


def validate_identifier(name, value, maximum):
    if not isinstance(value, str) or not value or len(value) > maximum:
        raise WorkerError(
            "invalid_{0}".format(name),
            "{0} is not a bounded non-empty string".format(name),
        )
    allowed = (
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "0123456789._:-"
    )
    if any(character not in allowed for character in value):
        raise WorkerError(
            "invalid_{0}".format(name),
            "{0} contains an unsupported character".format(name),
        )
    return value


def _strict_json_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError("duplicate JSON object key")
        value[key] = item
    return value


def _reject_json_constant(_value):
    raise ValueError("non-finite JSON number")


def decode_request(raw, controller_id):
    """Decode one newline-terminated request without accepting extensions."""
    request_id = None
    if not isinstance(raw, bytes):
        raise WorkerError(
            "invalid_frame",
            "Request frame must be bytes",
            fatal=True,
        )
    if len(raw) > MAX_FRAME_BYTES:
        raise WorkerError(
            "frame_too_large",
            "Request frame exceeds the fixed limit",
            fatal=True,
        )
    if not raw.endswith(b"\n"):
        raise WorkerError(
            "unterminated_frame",
            "Request frame must end with one newline",
            fatal=True,
        )
    try:
        decoded = raw.decode("utf-8")
        value = json.loads(
            decoded,
            object_pairs_hook=_strict_json_object,
            parse_constant=_reject_json_constant,
        )
    except (UnicodeDecodeError, ValueError):
        raise WorkerError(
            "invalid_json",
            "Request is not valid UTF-8 JSON",
            fatal=True,
        )
    if not isinstance(value, dict):
        raise WorkerError(
            "invalid_request",
            "Request must be a JSON object",
            fatal=True,
        )
    if set(value) != REQUEST_FIELDS:
        raise WorkerError(
            "invalid_fields",
            "Request fields do not exactly match the protocol",
            fatal=True,
        )
    try:
        request_id = validate_identifier(
            "request_id",
            value["request_id"],
            128,
        )
        schema = value["schema"]
        target = validate_identifier(
            "controller_id",
            value["controller_id"],
            128,
        )
        operation = value["op"]
        arguments = value["args"]
    except WorkerError as error:
        error.request_id = request_id
        raise
    if schema != REQUEST_SCHEMA:
        raise WorkerError(
            "wrong_schema",
            "Request schema does not match",
            request_id=request_id,
            fatal=True,
        )
    if target != controller_id:
        raise WorkerError(
            "wrong_controller",
            "Request targets another controller",
            request_id=request_id,
        )
    if operation not in ALLOWED_OPERATIONS:
        raise WorkerError(
            "unsupported_operation",
            "Operation is not supported",
            request_id=request_id,
        )
    if not isinstance(arguments, dict):
        raise WorkerError(
            "invalid_args",
            "args must be a JSON object",
            request_id=request_id,
        )
    if operation == "pulse":
        expected_arguments = frozenset(("action",))
    elif operation == "scan_turn":
        expected_arguments = frozenset(("relative_delta_mdeg",))
    else:
        expected_arguments = frozenset()
    if set(arguments) != expected_arguments:
        raise WorkerError(
            "invalid_args",
            "Arguments do not exactly match the operation",
            request_id=request_id,
        )
    if operation == "pulse":
        try:
            validate_identifier("action", arguments["action"], 32)
        except WorkerError as error:
            error.request_id = request_id
            raise
        if arguments["action"] not in ACTION_SPECS:
            raise WorkerError(
                "invalid_action",
                "Pulse action is not supported",
                request_id=request_id,
            )
    if operation == "scan_turn":
        relative_delta_mdeg = arguments["relative_delta_mdeg"]
        if (
            isinstance(relative_delta_mdeg, bool)
            or not isinstance(relative_delta_mdeg, int)
            or relative_delta_mdeg
            not in SCAN_TURN_ALLOWED_DELTAS_MDEG
        ):
            raise WorkerError(
                "invalid_scan_turn",
                "Relative scan turn is outside the fixed host profile",
                request_id=request_id,
            )
    return {
        "request_id": request_id,
        "op": operation,
        "args": arguments,
    }


def response_object(
    controller_id,
    request_id,
    ok,
    payload,
    state_version,
):
    """Build a correlated response carrying the worker state version."""
    response = {
        "schema": RESPONSE_SCHEMA,
        "controller_id": controller_id,
        "request_id": request_id,
        "ok": ok,
        "state_version": state_version,
    }
    if ok:
        response["result"] = payload
    else:
        response["error"] = payload
    return response


def write_response(response, output_stream=None):
    """Write exactly one canonical JSON response frame."""
    raw = (
        json.dumps(
            response,
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
        + b"\n"
    )
    stream = output_stream if output_stream is not None else sys.stdout
    binary_stream = getattr(stream, "buffer", stream)
    binary_stream.write(raw)
    binary_stream.flush()


def error_payload(error, worker, stop=None):
    payload = {
        "code": getattr(error, "code", "worker_failure"),
        "message": str(error),
        "fatal": bool(getattr(error, "fatal", False)),
    }
    if (
        worker is not None
        and getattr(worker, "last_observation", None) is not None
    ):
        payload["observation"] = copy.deepcopy(
            worker.last_observation
        )
    if stop is not None:
        payload["stop"] = stop
    return payload
