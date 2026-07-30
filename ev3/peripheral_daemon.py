#!/usr/bin/env python3
"""Bounded, motor-free EV3 peripheral process over SSH stdio.

This is not a network server.  One authenticated SSH session starts the
process and exchanges strict JSONL frames over stdin/stdout.  The process has
no motor, speech, shell, or network operation.
"""

from __future__ import print_function

import argparse
import errno
import fcntl
import json
import os
import queue
import select
import sys
import threading
import time

if __package__:
    from .peripheral_protocol import (
        MAX_FRAME_BYTES,
        OP_SHUTDOWN,
        PeripheralProtocol,
        ProtocolError,
        decode_request,
        encode_response,
        error_response,
    )
    from .robot_hal import RobotHAL, SafetyError
else:
    from peripheral_protocol import (
        MAX_FRAME_BYTES,
        OP_SHUTDOWN,
        PeripheralProtocol,
        ProtocolError,
        decode_request,
        encode_response,
        error_response,
    )
    from robot_hal import RobotHAL, SafetyError


DEFAULT_MAX_SESSION_MS = 60000
MAX_SESSION_MS = 120000
DEFAULT_MAX_REQUESTS = 128
MAX_REQUESTS = 128
REQUEST_QUEUE_SIZE = 8
OUTPUT_DEADLINE_SECONDS = 0.5


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


class SessionError(RuntimeError):
    def __init__(self, code, message):
        self.code = code
        RuntimeError.__init__(self, message)


class _RequestReader(threading.Thread):
    """Read bounded frames without letting stdin block the session owner."""

    def __init__(self, session, input_stream):
        threading.Thread.__init__(
            self,
            name="ev3-peripheral-request-reader",
        )
        self.daemon = True
        self.session = session
        self.input_stream = input_stream
        self._stop_requested = threading.Event()
        try:
            self.descriptor = input_stream.fileno()
        except (AttributeError, IOError, OSError, ValueError):
            self.descriptor = None

    def stop(self):
        self._stop_requested.set()
        if self.descriptor is not None:
            return
        close = getattr(self.input_stream, "close", None)
        if callable(close):
            try:
                close()
            except BaseException:
                pass

    def should_stop(self):
        return (
            self._stop_requested.is_set()
            or self.session.shutdown_requested()
        )

    def _put(self, kind, value=None, received_at_ms=None):
        try:
            self.session._request_queue.put_nowait(
                (kind, value, received_at_ms)
            )
            return True
        except queue.Full:
            self.session.fail_transport("request_queue_full")
            return False

    def run(self):
        if self.descriptor is not None:
            self._run_descriptor()
            return
        self._run_stream()

    def _run_descriptor(self):
        buffered = b""
        while not self.should_stop():
            try:
                readable, _writable, _errors = select.select(
                    [self.descriptor],
                    [],
                    [],
                    0.05,
                )
            except (IOError, OSError, ValueError):
                if not self.should_stop():
                    self._put("error", "input_select_failed")
                return
            if not readable:
                continue
            try:
                chunk = os.read(
                    self.descriptor,
                    MAX_FRAME_BYTES + 1,
                )
            except (IOError, OSError):
                if not self.should_stop():
                    self._put("error", "input_read_failed")
                return
            if not chunk:
                if self.should_stop():
                    return
                if buffered:
                    self._put(
                        "frame",
                        buffered,
                        received_at_ms=self.session.local_now_ms(),
                    )
                else:
                    self._put("eof")
                return

            buffered += chunk
            while b"\n" in buffered:
                boundary = buffered.index(b"\n") + 1
                raw = buffered[:boundary]
                buffered = buffered[boundary:]
                if not self._put(
                    "frame",
                    raw,
                    received_at_ms=self.session.local_now_ms(),
                ):
                    return
            if len(buffered) > MAX_FRAME_BYTES:
                self._put("oversized")
                return

    def _run_stream(self):
        while not self.should_stop():
            try:
                raw = self.input_stream.readline(
                    MAX_FRAME_BYTES + 1
                )
                received_at_ms = self.session.local_now_ms()
            except BaseException:
                if not self.should_stop():
                    self._put("error", "input_read_failed")
                return
            if raw == b"" or raw == "":
                if not self.should_stop():
                    self._put("eof")
                return
            if not self._put(
                "frame",
                raw,
                received_at_ms=received_at_ms,
            ):
                return


class _BoundedOutput(object):
    """Write one bounded response without an unbounded pipe wait."""

    def __init__(self, output_stream):
        self.output_stream = output_stream
        try:
            descriptor = output_stream.fileno()
        except (AttributeError, IOError, OSError, ValueError):
            descriptor = None
        self.descriptor = descriptor
        if descriptor is not None:
            try:
                flags = fcntl.fcntl(descriptor, fcntl.F_GETFL)
                fcntl.fcntl(
                    descriptor,
                    fcntl.F_SETFL,
                    flags | os.O_NONBLOCK,
                )
            except (IOError, OSError):
                raise SessionError(
                    "output_nonblocking_setup_failed",
                    "Could not make protocol output non-blocking",
                )

    def write(self, wire):
        if self.descriptor is None:
            try:
                self.output_stream.write(wire)
                self.output_stream.flush()
            except BaseException:
                raise SessionError(
                    "output_write_failed",
                    "Could not write protocol response",
                )
            return

        offset = 0
        deadline = time.monotonic() + OUTPUT_DEADLINE_SECONDS
        while offset < len(wire):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SessionError(
                    "output_write_timeout",
                    "Protocol response write timed out",
                )
            try:
                _readable, writable, _errors = select.select(
                    [],
                    [self.descriptor],
                    [],
                    min(0.05, remaining),
                )
            except (IOError, OSError, ValueError):
                raise SessionError(
                    "output_select_failed",
                    "Protocol output select failed",
                )
            if not writable:
                continue
            try:
                written = os.write(
                    self.descriptor,
                    wire[offset:],
                )
            except OSError as error:
                if error.errno in (
                    errno.EAGAIN,
                    errno.EWOULDBLOCK,
                ):
                    continue
                raise SessionError(
                    "output_write_failed",
                    "Could not write protocol response",
                )
            if written <= 0:
                raise SessionError(
                    "output_write_failed",
                    "Protocol response write made no progress",
                )
            offset += written


class PeripheralSession(object):
    """One bounded, sequential request stream to one peripheral protocol."""

    def __init__(
        self,
        protocol,
        input_stream,
        output_stream,
        max_session_ms=DEFAULT_MAX_SESSION_MS,
        max_requests=DEFAULT_MAX_REQUESTS,
        monotonic_fn=time.monotonic,
    ):
        if not isinstance(protocol, PeripheralProtocol):
            raise SessionError(
                "invalid_protocol",
                "Session requires PeripheralProtocol",
            )
        if not hasattr(input_stream, "readline"):
            raise SessionError(
                "invalid_input",
                "Session input stream is invalid",
            )
        if not callable(monotonic_fn):
            raise SessionError(
                "invalid_clock",
                "Session clock is invalid",
            )
        if (
            not _is_int(max_session_ms)
            or max_session_ms <= 0
            or max_session_ms > MAX_SESSION_MS
        ):
            raise SessionError(
                "invalid_session_deadline",
                "Session duration is invalid",
            )
        if (
            not _is_int(max_requests)
            or max_requests <= 0
            or max_requests > MAX_REQUESTS
        ):
            raise SessionError(
                "invalid_request_budget",
                "Session request budget is invalid",
            )
        self.protocol = protocol
        self.input_stream = input_stream
        self.max_session_ms = max_session_ms
        self.max_requests = max_requests
        self._monotonic_fn = monotonic_fn
        self._output = _BoundedOutput(output_stream)
        self._request_queue = queue.Queue(maxsize=REQUEST_QUEUE_SIZE)
        self._shutdown = threading.Event()
        self._failure_lock = threading.Lock()
        self._transport_failure = None
        self._request_count = 0
        self._seen_request_ids = set()
        self._reader = None

    def local_now_ms(self):
        value = self._monotonic_fn()
        if isinstance(value, bool) or not isinstance(
            value,
            (int, float),
        ):
            raise SessionError(
                "invalid_clock",
                "Session clock returned an invalid value",
            )
        milliseconds = int(value * 1000)
        if milliseconds < 0:
            raise SessionError(
                "invalid_clock",
                "Session clock returned an invalid value",
            )
        return milliseconds

    @property
    def transport_failure(self):
        with self._failure_lock:
            return self._transport_failure

    def fail_transport(self, code):
        with self._failure_lock:
            if self._transport_failure is None:
                self._transport_failure = code
        self._shutdown.set()

    def shutdown_requested(self):
        return self._shutdown.is_set()

    @staticmethod
    def _has_line_ending(raw):
        if isinstance(raw, bytes):
            return raw.endswith(b"\n")
        if isinstance(raw, str):
            return raw.endswith("\n")
        return False

    def _write_response(self, response):
        try:
            self._output.write(encode_response(response))
            return True
        except (ProtocolError, SessionError) as error:
            self.fail_transport(
                getattr(error, "code", "output_write_failed")
            )
            return False

    def _write_error(self, error, request_id=None):
        return self._write_response(
            error_response(
                self.protocol.controller_id,
                error,
                request_id=request_id,
            )
        )

    def run(self):
        try:
            return self._run_session()
        finally:
            self._shutdown.set()
            if self._reader is not None:
                self._reader.stop()
                self._reader.join(0.5)
                if self._reader.is_alive():
                    self.fail_transport(
                        "input_reader_not_stopped"
                    )
                    raise SessionError(
                        "input_reader_not_stopped",
                        (
                            "Input reader did not stop before "
                            "process shutdown"
                        ),
                    )

    def _run_session(self):
        started_at_ms = self.local_now_ms()
        wall_deadline = (
            time.monotonic() + (self.max_session_ms / 1000.0)
        )
        close_reason = None
        reader = _RequestReader(self, self.input_stream)
        self._reader = reader
        reader.start()

        while not self.shutdown_requested():
            local_remaining_ms = (
                started_at_ms
                + self.max_session_ms
                - self.local_now_ms()
            )
            wall_remaining = wall_deadline - time.monotonic()
            if local_remaining_ms <= 0 or wall_remaining <= 0:
                close_reason = "session_deadline"
                self._shutdown.set()
                break
            wait_seconds = min(
                0.05,
                local_remaining_ms / 1000.0,
                wall_remaining,
            )
            try:
                kind, raw, received_at_ms = self._request_queue.get(
                    timeout=max(0.001, wait_seconds)
                )
            except queue.Empty:
                continue

            if kind == "eof":
                close_reason = "input_eof"
                self.fail_transport(close_reason)
                break
            if kind == "error":
                close_reason = raw
                self.fail_transport(close_reason)
                break
            if kind == "oversized":
                error = ProtocolError(
                    "frame_too_large",
                    "Request frame exceeds the byte limit",
                    fatal=True,
                )
                self._write_error(error)
                close_reason = error.code
                self.fail_transport(close_reason)
                break
            if kind != "frame":
                close_reason = "invalid_reader_event"
                self.fail_transport(close_reason)
                break
            if not self._has_line_ending(raw):
                error = ProtocolError(
                    "truncated_frame",
                    "Request frame did not end with a newline",
                    fatal=True,
                )
                self._write_error(error)
                close_reason = error.code
                self.fail_transport(close_reason)
                break

            if self._request_count >= self.max_requests:
                error = ProtocolError(
                    "request_budget_exhausted",
                    "Session request budget is exhausted",
                    fatal=True,
                )
                self._write_error(error)
                close_reason = error.code
                self.fail_transport(close_reason)
                break
            self._request_count += 1

            try:
                request = decode_request(raw, received_at_ms)
            except ProtocolError as error:
                if error.request_id is not None:
                    if error.request_id in self._seen_request_ids:
                        error = ProtocolError(
                            "duplicate_request_id",
                            "request_id has already been received",
                            request_id=error.request_id,
                            fatal=True,
                        )
                    else:
                        self._seen_request_ids.add(
                            error.request_id
                        )
                self._write_error(error)
                if error.fatal:
                    close_reason = error.code
                    self.fail_transport(close_reason)
                    break
                continue

            if request.request_id in self._seen_request_ids:
                error = ProtocolError(
                    "duplicate_request_id",
                    "request_id has already been received",
                    request_id=request.request_id,
                    fatal=True,
                )
                self._write_error(
                    error,
                    request_id=request.request_id,
                )
                close_reason = error.code
                self.fail_transport(close_reason)
                break

            self._seen_request_ids.add(request.request_id)
            try:
                response = self.protocol.execute(
                    request,
                    dispatch_at_ms=self.local_now_ms(),
                )
            except ProtocolError as error:
                response = error_response(
                    self.protocol.controller_id,
                    error,
                    request_id=request.request_id,
                )
                if error.fatal:
                    close_reason = error.code
            if not self._write_response(response):
                close_reason = self.transport_failure
                break
            if close_reason is not None:
                self.fail_transport(close_reason)
                break
            if (
                request.operation == OP_SHUTDOWN
                and response.get("ok") is True
            ):
                close_reason = "shutdown"
                self._shutdown.set()
                break

        if close_reason is None:
            close_reason = self.transport_failure or "closed"
        return {
            "status": "closed",
            "close_reason": close_reason,
            "transport_failure": self.transport_failure,
            "requests_received": self._request_count,
        }


def default_config_path():
    return os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "ev3rstorm.json",
        )
    )


def run_daemon(
    robot,
    input_stream,
    output_stream,
    max_session_ms=DEFAULT_MAX_SESSION_MS,
    max_requests=DEFAULT_MAX_REQUESTS,
    monotonic_fn=time.monotonic,
):
    protocol = PeripheralProtocol(robot)
    session = PeripheralSession(
        protocol,
        input_stream,
        output_stream,
        max_session_ms=max_session_ms,
        max_requests=max_requests,
        monotonic_fn=monotonic_fn,
    )
    return session.run()


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Motor-free EV3 peripheral process over strict JSONL "
            "stdin/stdout."
        )
    )
    parser.add_argument("--config", default=default_config_path())
    parser.add_argument(
        "--max-session-ms",
        default=DEFAULT_MAX_SESSION_MS,
        type=int,
    )
    parser.add_argument(
        "--max-requests",
        default=DEFAULT_MAX_REQUESTS,
        type=int,
    )
    return parser


def _binary_stdio():
    stdin = getattr(sys.stdin, "buffer", sys.stdin)
    stdout = getattr(sys.stdout, "buffer", sys.stdout)
    return stdin, stdout


def _safe_error(error):
    try:
        message = " ".join(str(error).split())[:240]
    except Exception:
        message = "Peripheral daemon failed"
    return {
        "status": "failed",
        "error": {
            "code": getattr(error, "code", "peripheral_daemon_failed"),
            "message": message,
        },
    }


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        robot = RobotHAL(args.config)
        input_stream, output_stream = _binary_stdio()
        result = run_daemon(
            robot,
            input_stream,
            output_stream,
            max_session_ms=args.max_session_ms,
            max_requests=args.max_requests,
            monotonic_fn=robot.monotonic_fn,
        )
        return 0 if result["transport_failure"] is None else 1
    except (
        ProtocolError,
        SafetyError,
        SessionError,
        IOError,
        OSError,
        RuntimeError,
        ValueError,
    ) as error:
        print(
            json.dumps(
                _safe_error(error),
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    sys.exit(main())
