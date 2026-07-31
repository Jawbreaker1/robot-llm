#!/usr/bin/env python3
"""Shared Python 3.5-compatible support for the EV3 supervisor.

This module keeps validation, bound sysfs reads, audit buffering, and stop
evidence handling separate from the motor-owner state machine.  The public
supervisor module imports these names explicitly so existing callers retain
the same API.
"""

from __future__ import print_function

import binascii
import copy
from collections import deque
import io
import json
import os
import stat

if __package__:
    from .robot_hal import SafetyError
else:
    from robot_hal import SafetyError


KNOWN_MOTOR_STATES = frozenset(
    ("running", "ramping", "holding", "stalled", "overloaded")
)


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _validate_positive_int(name, value, maximum=None):
    if not _is_int(value) or value <= 0:
        raise SupervisorError(
            "invalid_{}".format(name),
            "{} must be a positive integer".format(name),
        )
    if maximum is not None and value > maximum:
        raise SupervisorError(
            "invalid_{}".format(name),
            "{} exceeds maximum {}".format(name, maximum),
        )
    return value


def _validate_identifier(name, value, maximum):
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or "\x00" in value
        or "\n" in value
        or "\r" in value
    ):
        raise SupervisorError(
            "invalid_{}".format(name),
            "{} must contain 1..{} safe characters".format(name, maximum),
        )
    return value


def _state_tokens(raw):
    if not isinstance(raw, str):
        raise SupervisorError(
            "invalid_motor_state",
            "Motor state must be text",
        )
    tokens = frozenset(raw.split())
    unknown = tokens - KNOWN_MOTOR_STATES
    if unknown:
        raise SupervisorError(
            "invalid_motor_state",
            "Motor state contains unknown tokens: {}".format(
                sorted(unknown)
            ),
        )
    return tokens


def _default_session_id():
    return binascii.hexlify(os.urandom(16)).decode("ascii")


class SupervisorError(SafetyError):
    def __init__(self, code, message):
        self.code = code
        SafetyError.__init__(self, message)


class _BoundAttributeReader(object):
    """Read one immutable sysfs attribute without reopening it per tick."""

    MAX_BYTES = 256

    def __init__(self, path, kind):
        self.path = path
        self.kind = kind
        self._descriptor = None
        self._identity_token = None
        flags = os.O_RDONLY
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        try:
            self._descriptor = os.open(path, flags)
            metadata = os.fstat(self._descriptor)
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_nlink <= 0
            ):
                raise OSError("attribute is not a linked regular file")
            self._identity_token = (
                metadata.st_dev,
                metadata.st_ino,
            )
            self._verify_identity("hardware_topology_unreadable")
        except BaseException:
            try:
                self.close()
            except BaseException:
                pass
            raise

    @property
    def descriptor(self):
        return self._descriptor

    def _verify_identity(self, error_code):
        if self._descriptor is None:
            raise SupervisorError(
                error_code,
                "{} cached attribute is closed".format(self.kind),
            )
        try:
            descriptor_metadata = os.fstat(self._descriptor)
            path_metadata = os.stat(self.path)
        except (IOError, OSError, ValueError) as error:
            raise SupervisorError(
                error_code,
                "{} identity could not be read: {}".format(
                    self.kind,
                    error,
                ),
            )
        descriptor_token = (
            descriptor_metadata.st_dev,
            descriptor_metadata.st_ino,
        )
        path_token = (
            path_metadata.st_dev,
            path_metadata.st_ino,
        )
        if (
            not stat.S_ISREG(descriptor_metadata.st_mode)
            or descriptor_metadata.st_nlink <= 0
            or descriptor_token != self._identity_token
            or path_token != self._identity_token
        ):
            raise SupervisorError(
                error_code,
                "{} identity changed after binding".format(self.kind),
            )

    def read(self, error_code):
        self._verify_identity(error_code)
        try:
            raw = os.pread(
                self._descriptor,
                self.MAX_BYTES + 1,
                0,
            )
        except (IOError, OSError, ValueError) as error:
            raise SupervisorError(
                error_code,
                "{} could not be read: {}".format(self.kind, error),
            )
        if len(raw) > self.MAX_BYTES:
            raise SupervisorError(
                error_code,
                "{} exceeded the read limit".format(self.kind),
            )
        try:
            return raw.decode("ascii").strip()
        except (AttributeError, UnicodeDecodeError):
            raise SupervisorError(
                error_code,
                "{} was not valid ASCII".format(self.kind),
            )

    def close(self):
        descriptor = self._descriptor
        self._descriptor = None
        if descriptor is not None:
            os.close(descriptor)


class JSONLAuditLog(object):
    """Small append-only JSONL sink with durable transition writes."""

    def __init__(self, path):
        if not isinstance(path, str) or not path:
            raise SupervisorError(
                "invalid_audit_path",
                "Audit path is invalid",
            )
        self.path = path
        flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(path, flags, 0o600)
        os.fchmod(descriptor, 0o600)
        self._handle = io.open(
            descriptor,
            "a",
            encoding="utf-8",
            closefd=True,
        )
        self._closed = False

    def append(self, event):
        if self._closed:
            raise IOError("Audit log is closed")
        encoded = json.dumps(
            event,
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        self._handle.write(encoded + "\n")
        self._handle.flush()
        os.fsync(self._handle.fileno())

    def close(self):
        if self._closed:
            return
        self._handle.close()
        self._closed = True


class AuditBuffer(object):
    """Bounded in-memory audit queue that never performs external I/O."""

    def __init__(self, maximum_events):
        _validate_positive_int(
            "audit_buffer_events",
            maximum_events,
            10000,
        )
        self.maximum_events = maximum_events
        self._events = deque()

    def append(self, event, terminal=False):
        reserved = 0 if terminal else 1
        if len(self._events) >= self.maximum_events - reserved:
            raise SupervisorError(
                "audit_buffer_full",
                "Audit buffer is full",
            )
        self._events.append(copy.deepcopy(event))

    def snapshot(self):
        return [copy.deepcopy(event) for event in self._events]

    def drain(self):
        events = []
        while self._events:
            events.append(self._events.popleft())
        return events


def _failed_stop_result(error):
    return {
        "stop_attempts": [],
        "stop_confirmed": False,
        "states": {},
        "positions": {},
        "fault_tokens": {},
        "errors": [str(error)],
    }


def _stop_result_from_exception(error):
    attached = getattr(error, "supervisor_stop_result", None)
    if isinstance(attached, dict):
        return copy.deepcopy(attached)
    return _failed_stop_result(error)


def _combine_stop_results(labelled_results):
    history = []
    errors = []
    latest = None
    for label, result in labelled_results:
        if result is None:
            continue
        snapshot = copy.deepcopy(result)
        history.append(
            {
                "phase": label,
                "result": snapshot,
            }
        )
        for detail in snapshot.get("errors", []):
            errors.append("{}: {}".format(label, detail))
        latest = snapshot
    if latest is None:
        latest = _failed_stop_result("No stop result")
    latest["errors"] = errors
    latest["stop_history"] = history
    return latest


def _copy_start_failure_evidence(source, target):
    for name in (
        "supervisor_start_cleanup",
        "supervisor_start_evidence",
        "supervisor_start_error",
    ):
        value = getattr(source, name, None)
        if value is not None:
            try:
                setattr(target, name, copy.deepcopy(value))
            except Exception:
                pass
    return target
