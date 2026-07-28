#!/usr/bin/env python3
"""Foreground EV3 supervisor process over authenticated SSH stdio.

The process is intentionally not a network server.  A single SSH session
starts it as a forced or fixed command and exchanges strict JSONL frames over
stdin/stdout.  Reader and writer threads never touch the supervisor; the main
thread remains the sole dispatcher and motor owner.
"""

from __future__ import print_function

import argparse
import errno
import fcntl
import json
import os
import queue
import select
import signal
import sys
import threading
import time

if __package__:
    from .robot_hal import RobotHAL, SafetyError
    from .supervisor import (
        EV3Supervisor,
        EV3SupervisorLoop,
        JSONLAuditLog,
        STATE_CLOSED,
        SupervisorError,
    )
    from .supervisor_cli import default_config_path
    from .supervisor_protocol import (
        MAX_FRAME_BYTES,
        OP_SHUTDOWN,
        STOP_OPERATIONS,
        ProtocolError,
        SupervisorProtocol,
        decode_request,
        encode_response,
        error_response,
    )
else:
    from robot_hal import RobotHAL, SafetyError
    from supervisor import (
        EV3Supervisor,
        EV3SupervisorLoop,
        JSONLAuditLog,
        STATE_CLOSED,
        SupervisorError,
    )
    from supervisor_cli import default_config_path
    from supervisor_protocol import (
        MAX_FRAME_BYTES,
        OP_SHUTDOWN,
        STOP_OPERATIONS,
        ProtocolError,
        SupervisorProtocol,
        decode_request,
        encode_response,
        error_response,
    )


DEFAULT_AUDIT_PATH = "/tmp/robot-llm-supervisor-daemon-audit.jsonl"
DEFAULT_MAX_SESSION_MS = 60000
MAX_SESSION_MS = 120000
MAX_PROCESS_REQUESTS = 128
NORMAL_QUEUE_SIZE = 8
URGENT_QUEUE_SIZE = 4
OUTPUT_QUEUE_SIZE = 16
OUTPUT_DRAIN_TIMEOUT_SECONDS = 1.0


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


class SessionError(RuntimeError):
    def __init__(self, code, message):
        self.code = code
        RuntimeError.__init__(self, message)


class _InboundItem(object):
    def __init__(self, request=None, error=None):
        self.request = request
        self.error = error


class _RequestReader(threading.Thread):
    """Blocking stdin reader that performs bounded parsing only."""

    def __init__(self, session, input_stream):
        threading.Thread.__init__(self, name="ev3-request-reader")
        self.daemon = True
        self.session = session
        self.input_stream = input_stream

    @staticmethod
    def _has_line_ending(raw):
        if isinstance(raw, bytes):
            return raw.endswith(b"\n")
        if isinstance(raw, str):
            return raw.endswith("\n")
        return False

    def run(self):
        try:
            descriptor = self.input_stream.fileno()
        except (AttributeError, IOError, OSError, ValueError):
            descriptor = None
        if descriptor is not None:
            self._run_descriptor(descriptor)
            return
        self._run_stream()

    def _process_raw(self, raw):
        received_at_ms = self.session.local_now_ms()
        try:
            request = decode_request(raw, received_at_ms)
        except ProtocolError as error:
            self.session.enqueue_protocol_error(error)
            if error.fatal:
                self.session.fail_transport(error.code)
                return False
            return True
        return self.session.enqueue_request(request)

    def _run_descriptor(self, descriptor):
        buffered = b""
        while not self.session.shutdown_requested():
            try:
                readable, _writable, _errors = select.select(
                    [descriptor],
                    [],
                    [],
                    0.05,
                )
            except (IOError, OSError, ValueError):
                self.session.fail_transport("input_select_failed")
                return
            if not readable:
                continue
            try:
                chunk = os.read(descriptor, MAX_FRAME_BYTES + 1)
            except (IOError, OSError):
                self.session.fail_transport("input_read_failed")
                return
            if not chunk:
                if self.session.shutdown_requested():
                    return
                self.session.fail_transport("input_eof")
                return
            buffered += chunk
            while b"\n" in buffered:
                boundary = buffered.index(b"\n") + 1
                raw = buffered[:boundary]
                buffered = buffered[boundary:]
                if len(raw) > MAX_FRAME_BYTES:
                    self.session.enqueue_protocol_error(
                        ProtocolError(
                            "frame_too_large",
                            "Request frame exceeds the byte limit",
                            fatal=True,
                        )
                    )
                    self.session.fail_transport("frame_too_large")
                    return
                if not self._process_raw(raw):
                    return
            if len(buffered) > MAX_FRAME_BYTES:
                self.session.enqueue_protocol_error(
                    ProtocolError(
                        "frame_too_large",
                        "Request frame exceeds the byte limit",
                        fatal=True,
                    )
                )
                self.session.fail_transport("frame_too_large")
                return

    def _run_stream(self):
        while not self.session.shutdown_requested():
            try:
                raw = self.input_stream.readline(
                    MAX_FRAME_BYTES + 1
                )
            except BaseException:
                self.session.fail_transport("input_read_failed")
                return

            if raw == b"" or raw == "":
                if self.session.shutdown_requested():
                    return
                self.session.fail_transport("input_eof")
                return
            if not self._has_line_ending(raw):
                self.session.enqueue_protocol_error(
                    ProtocolError(
                        "truncated_frame",
                        "Request frame did not end with a newline",
                        fatal=True,
                    )
                )
                self.session.fail_transport("truncated_frame")
                return

            if not self._process_raw(raw):
                return


class _ResponseWriter(threading.Thread):
    """Only this daemon thread may block on stdout."""

    def __init__(self, session, output_stream):
        threading.Thread.__init__(self, name="ev3-response-writer")
        self.daemon = True
        self.session = session
        self.output_stream = output_stream

    def run(self):
        try:
            descriptor = self.output_stream.fileno()
        except (AttributeError, IOError, OSError, ValueError):
            descriptor = None
        if descriptor is not None:
            try:
                flags = fcntl.fcntl(descriptor, fcntl.F_GETFL)
                fcntl.fcntl(
                    descriptor,
                    fcntl.F_SETFL,
                    flags | os.O_NONBLOCK,
                )
            except (IOError, OSError):
                self.session.fail_transport(
                    "output_nonblocking_setup_failed"
                )
                return

        while True:
            item = self.session._output_queue.get()
            try:
                if item is self.session._writer_sentinel:
                    return
                if descriptor is None:
                    self.output_stream.write(item)
                    self.output_stream.flush()
                else:
                    self._write_descriptor(descriptor, item)
            except BaseException:
                self.session.fail_transport("output_write_failed")
                return
            finally:
                self.session._output_queue.task_done()

    def _write_descriptor(self, descriptor, data):
        offset = 0
        deadline = time.monotonic() + 0.5
        while offset < len(data):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise IOError("Output write deadline expired")
            try:
                _readable, writable, _errors = select.select(
                    [],
                    [descriptor],
                    [],
                    min(0.05, remaining),
                )
            except (IOError, OSError, ValueError):
                raise IOError("Output select failed")
            if not writable:
                continue
            try:
                written = os.write(descriptor, data[offset:])
            except OSError as error:
                if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    continue
                raise
            if written <= 0:
                raise IOError("Output write made no progress")
            offset += written


class ForegroundSupervisorSession(object):
    """One bounded SSH stdio session around one supervisor instance."""

    def __init__(
        self,
        supervisor,
        protocol,
        input_stream,
        output_stream,
        max_session_ms=DEFAULT_MAX_SESSION_MS,
        external_shutdown_event=None,
    ):
        if not isinstance(supervisor, EV3Supervisor):
            raise SessionError(
                "invalid_supervisor",
                "Session requires EV3Supervisor",
            )
        if not isinstance(protocol, SupervisorProtocol):
            raise SessionError(
                "invalid_protocol",
                "Session requires SupervisorProtocol",
            )
        if protocol.supervisor is not supervisor:
            raise SessionError(
                "supervisor_mismatch",
                "Protocol and session supervisors differ",
            )
        if (
            not hasattr(input_stream, "readline")
            or not hasattr(output_stream, "write")
            or not hasattr(output_stream, "flush")
        ):
            raise SessionError(
                "invalid_stream",
                "Session streams are invalid",
            )
        if (
            external_shutdown_event is not None
            and not callable(
                getattr(external_shutdown_event, "is_set", None)
            )
        ):
            raise SessionError(
                "invalid_shutdown_event",
                "External shutdown event is invalid",
            )
        if (
            not _is_int(max_session_ms)
            or max_session_ms <= 0
            or max_session_ms > MAX_SESSION_MS
        ):
            raise SessionError(
                "invalid_session_deadline",
                "Session deadline is invalid",
            )

        self.supervisor = supervisor
        self.protocol = protocol
        self.input_stream = input_stream
        self.output_stream = output_stream
        self.max_session_ms = max_session_ms
        self._external_shutdown_event = external_shutdown_event
        self._normal_queue = queue.Queue(maxsize=NORMAL_QUEUE_SIZE)
        self._urgent_queue = queue.Queue(maxsize=URGENT_QUEUE_SIZE)
        self._output_queue = queue.Queue(maxsize=OUTPUT_QUEUE_SIZE)
        self._shutdown = threading.Event()
        self._transport_failure = None
        self._failure_lock = threading.Lock()
        self._request_count = 0
        self._seen_request_ids = set()
        self._writer_sentinel = object()
        self._loop = None
        self._reader = None
        self._writer = None
        self._started_at_ms = None

    def local_now_ms(self):
        return self.supervisor._now_ms()

    def shutdown_requested(self):
        if (
            self._external_shutdown_event is not None
            and self._external_shutdown_event.is_set()
        ):
            self.request_shutdown()
            return True
        if self._shutdown.is_set():
            return True
        if (
            self._started_at_ms is not None
            and self.local_now_ms() - self._started_at_ms
            >= self.max_session_ms
        ):
            self.fail_transport("session_deadline")
            return True
        return False

    @property
    def transport_failure(self):
        with self._failure_lock:
            return self._transport_failure

    def fail_transport(self, code):
        with self._failure_lock:
            if self._transport_failure is None:
                self._transport_failure = code
        if self._loop is not None:
            self._loop.request_emergency_stop()
        self._shutdown.set()

    def request_shutdown(self):
        if self._loop is not None:
            self._loop.request_emergency_stop()
        self._shutdown.set()

    def enqueue_protocol_error(self, error):
        item = _InboundItem(error=error)
        try:
            self._normal_queue.put_nowait(item)
            return True
        except queue.Full:
            self.fail_transport("request_queue_full")
            return False

    def enqueue_request(self, request):
        self._request_count += 1
        if self._request_count > MAX_PROCESS_REQUESTS:
            self.fail_transport("request_budget_exhausted")
            return False
        if request.request_id in self._seen_request_ids:
            return self.enqueue_protocol_error(
                ProtocolError(
                    "duplicate_request_id",
                    "request_id has already been received",
                    request_id=request.request_id,
                )
            )
        self._seen_request_ids.add(request.request_id)
        target = (
            self._urgent_queue
            if (
                request.operation in STOP_OPERATIONS
                and request.controller_id
                == self.protocol.controller_id
            )
            else self._normal_queue
        )
        if (
            request.operation in STOP_OPERATIONS
            and request.controller_id == self.protocol.controller_id
            and self._loop is not None
        ):
            self._loop.request_emergency_stop()
        try:
            target.put_nowait(_InboundItem(request=request))
            return True
        except queue.Full:
            self.fail_transport("request_queue_full")
            return False

    def _enqueue_response(self, response):
        try:
            wire = encode_response(response)
            self._output_queue.put_nowait(wire)
        except (ProtocolError, ValueError, TypeError, queue.Full):
            self.fail_transport("response_queue_failed")
            return False
        return True

    @staticmethod
    def _take_nowait(source):
        try:
            return source.get_nowait()
        except queue.Empty:
            return None

    def _dispatch_one(self):
        source = self._urgent_queue
        item = self._take_nowait(source)
        if item is None:
            source = self._normal_queue
            item = self._take_nowait(source)
        if item is None:
            return
        try:
            if item.error is not None:
                response = error_response(
                    self.protocol.controller_id,
                    item.error,
                )
            else:
                response = self.protocol.execute(
                    item.request,
                    dispatch_at_ms=self.local_now_ms(),
                    cancellation_requested=(
                        self._motion_start_cancelled
                    ),
                )
            if not self._enqueue_response(response):
                return
            if (
                item.request is not None
                and item.request.operation == OP_SHUTDOWN
                and response.get("ok") is True
            ):
                self.request_shutdown()
        finally:
            source.task_done()

    def _motion_start_cancelled(self):
        return (
            self._shutdown.is_set()
            or (
                self._external_shutdown_event is not None
                and self._external_shutdown_event.is_set()
            )
            or (
                self._loop is not None
                and self._loop.emergency_stop_requested()
            )
        )

    def _wait_for_output(self):
        deadline = (
            time.monotonic() + OUTPUT_DRAIN_TIMEOUT_SECONDS
        )
        while (
            self._output_queue.unfinished_tasks
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)
        return self._output_queue.unfinished_tasks == 0

    def run(self):
        self._started_at_ms = self.local_now_ms()
        self._loop = EV3SupervisorLoop(self.supervisor)
        self._writer = _ResponseWriter(self, self.output_stream)
        self._reader = _RequestReader(self, self.input_stream)
        self._writer.start()
        self._reader.start()

        status = None
        pending_error = None
        try:
            status = self._loop.run_forever(
                self.shutdown_requested,
                dispatch_one=self._dispatch_one,
            )
        except BaseException as error:
            pending_error = error
        finally:
            self._shutdown.set()

        output_complete = self._wait_for_output()
        try:
            self._output_queue.put_nowait(self._writer_sentinel)
        except queue.Full:
            output_complete = False
        self._writer.join(OUTPUT_DRAIN_TIMEOUT_SECONDS)
        self._reader.join(0.25)
        if self._reader.is_alive() and pending_error is None:
            pending_error = SessionError(
                "input_reader_not_stopped",
                "Input reader did not stop before process shutdown",
            )

        if not output_complete:
            if pending_error is None:
                pending_error = SessionError(
                    "output_not_drained",
                    "Response output did not drain before shutdown",
                )
        if pending_error is not None:
            raise pending_error
        return {
            "status": status,
            "transport_failure": self.transport_failure,
            "requests_received": self._request_count,
            "motion_enabled": self.protocol.allow_motion,
            "remaining_motion_budget": (
                self.protocol.remaining_motion_budget
            ),
        }


def run_daemon(
    robot,
    audit_log,
    input_stream,
    output_stream,
    allow_one_drive_test=False,
    max_session_ms=DEFAULT_MAX_SESSION_MS,
    external_shutdown_event=None,
):
    """Run one foreground session and persist audit only after shutdown."""
    if not isinstance(allow_one_drive_test, bool):
        raise SessionError(
            "invalid_motion_mode",
            "allow_one_drive_test must be boolean",
        )
    supervisor = None
    supervisor_initialized = False
    result = None
    pending_error = None
    try:
        supervisor = EV3Supervisor.__new__(EV3Supervisor)
        EV3Supervisor.__init__(supervisor, robot)
        supervisor_initialized = True
        try:
            controller_id = robot.config["controller_id"]
        except (KeyError, TypeError):
            raise SessionError(
                "missing_controller_id",
                "Robot configuration has no controller_id",
            )
        protocol = SupervisorProtocol(
            supervisor,
            controller_id,
            allow_motion=allow_one_drive_test,
            motion_budget=1 if allow_one_drive_test else 0,
            experiment_max_abs_speed_dps=100,
            experiment_max_duration_ms=300,
        )
        session = ForegroundSupervisorSession(
            supervisor,
            protocol,
            input_stream,
            output_stream,
            max_session_ms=max_session_ms,
            external_shutdown_event=external_shutdown_event,
        )
        result = session.run()
    except BaseException as error:
        pending_error = error
    finally:
        if (
            supervisor is not None
            and (
                supervisor_initialized
                or hasattr(supervisor, "_owner")
            )
        ):
            if getattr(supervisor, "state", None) != STATE_CLOSED:
                try:
                    shutdown = supervisor.close()
                    if (
                        shutdown.get("state") != STATE_CLOSED
                        or shutdown.get("audit_complete") is not True
                    ):
                        raise SessionError(
                            "shutdown_not_verified",
                            "Supervisor shutdown was not verified",
                        )
                except BaseException as error:
                    if pending_error is None:
                        pending_error = error
            for event in supervisor.drain_audit_events():
                try:
                    audit_log.append(event)
                except BaseException as error:
                    if pending_error is None:
                        pending_error = error
                    break

    if pending_error is not None:
        raise pending_error
    return result


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Foreground EV3 supervisor over strict JSONL stdin/stdout. "
            "This entrypoint is motion-free and cannot enable motor starts."
        )
    )
    parser.add_argument(
        "--config",
        default=default_config_path(),
    )
    parser.add_argument(
        "--audit-log",
        default=DEFAULT_AUDIT_PATH,
    )
    parser.add_argument(
        "--max-session-ms",
        type=int,
        default=DEFAULT_MAX_SESSION_MS,
    )
    return parser


def _binary_stdio():
    stdin = getattr(sys.stdin, "buffer", sys.stdin)
    stdout = getattr(sys.stdout, "buffer", sys.stdout)
    return stdin, stdout


def main(argv=None):
    args = build_parser().parse_args(argv)
    audit = None
    external_shutdown_event = threading.Event()

    def request_signal_shutdown(_signum, _frame):
        external_shutdown_event.set()

    for signal_name in ("SIGHUP", "SIGINT", "SIGTERM"):
        if hasattr(signal, signal_name):
            signal.signal(
                getattr(signal, signal_name),
                request_signal_shutdown,
            )

    try:
        if (
            not _is_int(args.max_session_ms)
            or args.max_session_ms <= 0
            or args.max_session_ms > MAX_SESSION_MS
        ):
            raise SessionError(
                "invalid_session_deadline",
                "Session deadline is invalid",
            )
        audit = JSONLAuditLog(args.audit_log)
        robot = RobotHAL(args.config)
        input_stream, output_stream = _binary_stdio()

        # ``run_daemon`` owns the session internally.  Signals before the
        # session exists still leave the default fail-closed startup path.
        result = run_daemon(
            robot,
            audit,
            input_stream,
            output_stream,
            allow_one_drive_test=False,
            max_session_ms=args.max_session_ms,
            external_shutdown_event=external_shutdown_event,
        )
        status = result["status"]
        if (
            status.get("state") != STATE_CLOSED
            or status.get("audit_complete") is not True
        ):
            raise SessionError(
                "shutdown_not_verified",
                "Supervisor did not close with terminal audit",
            )
        return 0
    except (
        ProtocolError,
        SafetyError,
        SessionError,
        SupervisorError,
        IOError,
        OSError,
        RuntimeError,
        ValueError,
    ) as error:
        report = {
            "status": "failed",
            "error": {
                "code": getattr(error, "code", "daemon_failure"),
                "message": " ".join(str(error).split())[:240],
            },
        }
        print(
            json.dumps(
                report,
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1
    finally:
        if audit is not None:
            audit.close()


if __name__ == "__main__":
    sys.exit(main())
