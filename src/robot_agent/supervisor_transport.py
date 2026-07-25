"""Bounded host transport for the foreground EV3 supervisor process.

SSH supplies authentication and encryption.  The remote command is fixed;
requests and responses are strict JSONL data and are never interpolated into
shell source.
"""

import json
import queue
import secrets
import subprocess
import threading
import time
from typing import Any, Callable, Dict, Mapping, Optional


PROTOCOL_VERSION = 1
RESPONSE_SCHEMA = "ev3-supervisor-response/v1"
REMOTE_DAEMON = "/home/robot/robot-llm/ev3/supervisor_daemon.py"
MAX_RESPONSE_BYTES = 64 * 1024
MAX_ERROR_BYTES = 16 * 1024

ProcessFactory = Callable[..., Any]


class SupervisorSSHError(RuntimeError):
    pass


class SupervisorSSHConfigurationError(SupervisorSSHError):
    pass


class SupervisorSSHTransportError(SupervisorSSHError):
    pass


class SupervisorSSHTimeoutError(SupervisorSSHTransportError):
    pass


class SupervisorSSHProtocolError(SupervisorSSHError):
    pass


class SupervisorRemoteError(SupervisorSSHError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _validate_identifier(name: str, value: str, maximum: int) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(character in value for character in "\x00\r\n")
    ):
        raise SupervisorSSHConfigurationError(
            "{} is invalid".format(name)
        )
    return value


def _validate_target(target: str) -> str:
    _validate_identifier("SSH target", target, 255)
    if target.startswith("-") or any(
        not (
            character.isalnum()
            or character in "._-@:%+"
        )
        for character in target
    ):
        raise SupervisorSSHConfigurationError(
            "SSH target is invalid"
        )
    return target


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


def _decode_response(
    raw: bytes,
    expected_request_id: str,
    expected_controller_id: str,
) -> Mapping[str, object]:
    if not isinstance(raw, bytes):
        raise SupervisorSSHProtocolError(
            "Supervisor response was not bytes"
        )
    if len(raw) > MAX_RESPONSE_BYTES:
        raise SupervisorSSHProtocolError(
            "Supervisor response exceeded the byte limit"
        )
    if not raw.endswith(b"\n") or raw.count(b"\n") != 1:
        raise SupervisorSSHProtocolError(
            "Supervisor response was not one JSONL frame"
        )
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, TypeError, ValueError):
        raise SupervisorSSHProtocolError(
            "Supervisor returned invalid JSON"
        ) from None
    if not isinstance(value, dict):
        raise SupervisorSSHProtocolError(
            "Supervisor response was not an object"
        )
    common = {
        "schema",
        "request_id",
        "controller_id",
        "ok",
    }
    if value.get("ok") is True:
        expected_fields = common | {"result"}
        if set(value) != expected_fields or not isinstance(
            value.get("result"), dict
        ):
            raise SupervisorSSHProtocolError(
                "Supervisor success response had invalid fields"
            )
    elif value.get("ok") is False:
        expected_fields = common | {"error"}
        error = value.get("error")
        if (
            set(value) != expected_fields
            or not isinstance(error, dict)
            or set(error) != {"code", "message"}
            or not isinstance(error.get("code"), str)
            or not error["code"]
            or not isinstance(error.get("message"), str)
            or not error["message"]
        ):
            raise SupervisorSSHProtocolError(
                "Supervisor error response had invalid fields"
            )
    else:
        raise SupervisorSSHProtocolError(
            "Supervisor response had invalid ok flag"
        )

    if value["schema"] != RESPONSE_SCHEMA:
        raise SupervisorSSHProtocolError(
            "Supervisor response schema was invalid"
        )
    if value["request_id"] != expected_request_id:
        raise SupervisorSSHProtocolError(
            "Supervisor response request_id did not match"
        )
    if value["controller_id"] != expected_controller_id:
        raise SupervisorSSHProtocolError(
            "Supervisor response controller_id did not match"
        )
    return value


class _StdoutPump(threading.Thread):
    def __init__(self, stream, destination, failed):
        super().__init__(name="ev3-supervisor-stdout", daemon=True)
        self._stream = stream
        self._destination = destination
        self._failed = failed

    def run(self) -> None:
        while not self._failed.is_set():
            try:
                raw = self._stream.readline(MAX_RESPONSE_BYTES + 1)
            except BaseException:
                self._failed.set()
                return
            if raw == b"":
                self._destination.put(("eof", None))
                return
            if len(raw) > MAX_RESPONSE_BYTES or not raw.endswith(b"\n"):
                self._destination.put(("invalid", None))
                self._failed.set()
                return
            try:
                self._destination.put_nowait(("line", raw))
            except queue.Full:
                self._failed.set()
                return


class _StderrPump(threading.Thread):
    def __init__(self, stream):
        super().__init__(name="ev3-supervisor-stderr", daemon=True)
        self._stream = stream
        self._chunks = bytearray()
        self._lock = threading.Lock()

    def run(self) -> None:
        while True:
            try:
                chunk = self._stream.read(1024)
            except BaseException:
                return
            if not chunk:
                return
            with self._lock:
                remaining = MAX_ERROR_BYTES - len(self._chunks)
                if remaining > 0:
                    self._chunks.extend(chunk[:remaining])

    def summary(self) -> str:
        with self._lock:
            raw = bytes(self._chunks)
        return " ".join(
            raw.decode("utf-8", errors="replace").split()
        )[:240]


class SupervisorSSHSession:
    """One sequential request stream to one foreground EV3 process."""

    def __init__(
        self,
        target: str,
        controller_id: str,
        process_factory: ProcessFactory = subprocess.Popen,
        connect_timeout_seconds: int = 3,
        response_timeout_seconds: float = 3.0,
        remote_session_ms: int = 15_000,
    ):
        self.target = _validate_target(target)
        self.controller_id = _validate_identifier(
            "controller_id",
            controller_id,
            128,
        )
        if not callable(process_factory):
            raise SupervisorSSHConfigurationError(
                "Process factory is invalid"
            )
        if (
            isinstance(connect_timeout_seconds, bool)
            or not isinstance(connect_timeout_seconds, int)
            or not 1 <= connect_timeout_seconds <= 30
        ):
            raise SupervisorSSHConfigurationError(
                "Connect timeout is invalid"
            )
        if (
            isinstance(response_timeout_seconds, bool)
            or not isinstance(response_timeout_seconds, (int, float))
            or not 0.1 <= response_timeout_seconds <= 30
        ):
            raise SupervisorSSHConfigurationError(
                "Response timeout is invalid"
            )
        if (
            isinstance(remote_session_ms, bool)
            or not isinstance(remote_session_ms, int)
            or not 1000 <= remote_session_ms <= 120_000
        ):
            raise SupervisorSSHConfigurationError(
                "Remote session duration is invalid"
            )

        argv = [
            "ssh",
            "-T",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout={}".format(connect_timeout_seconds),
            "-o",
            "StrictHostKeyChecking=yes",
            self.target,
            "python3",
            REMOTE_DAEMON,
            "--max-session-ms",
            str(remote_session_ms),
        ]
        try:
            self._process = process_factory(
                argv,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except OSError:
            raise SupervisorSSHTransportError(
                "Could not start SSH supervisor process"
            ) from None
        if (
            self._process.stdin is None
            or self._process.stdout is None
            or self._process.stderr is None
        ):
            raise SupervisorSSHTransportError(
                "SSH supervisor process has incomplete pipes"
            )

        self.argv = argv
        self._response_timeout_seconds = float(
            response_timeout_seconds
        )
        self._responses = queue.Queue(maxsize=8)
        self._failed = threading.Event()
        self._stdout_pump = _StdoutPump(
            self._process.stdout,
            self._responses,
            self._failed,
        )
        self._stderr_pump = _StderrPump(self._process.stderr)
        self._stdout_pump.start()
        self._stderr_pump.start()
        self._request_lock = threading.Lock()
        self._request_prefix = secrets.token_hex(8)
        self._request_number = 0
        self._closed = False
        self._shutdown_sent = False

    def _next_request_id(self) -> str:
        self._request_number += 1
        return "{}-{}".format(
            self._request_prefix,
            self._request_number,
        )

    def request(
        self,
        operation: str,
        arguments: Optional[Mapping[str, object]] = None,
        ttl_ms: int = 500,
    ) -> Dict[str, object]:
        if self._closed:
            raise SupervisorSSHTransportError(
                "Supervisor session is closed"
            )
        _validate_identifier("operation", operation, 64)
        if arguments is None:
            arguments = {}
        if not isinstance(arguments, Mapping):
            raise SupervisorSSHConfigurationError(
                "Request arguments must be an object"
            )
        if (
            isinstance(ttl_ms, bool)
            or not isinstance(ttl_ms, int)
            or not 1 <= ttl_ms <= 1000
        ):
            raise SupervisorSSHConfigurationError(
                "Request TTL is invalid"
            )

        with self._request_lock:
            request_id = self._next_request_id()
            payload = {
                "protocol_version": PROTOCOL_VERSION,
                "controller_id": self.controller_id,
                "request_id": request_id,
                "op": operation,
                "queue_ttl_ms": ttl_ms,
                "args": dict(arguments),
            }
            try:
                wire = (
                    json.dumps(
                        payload,
                        allow_nan=False,
                        ensure_ascii=False,
                        separators=(",", ":"),
                        sort_keys=True,
                    ).encode("utf-8")
                    + b"\n"
                )
            except (TypeError, ValueError):
                raise SupervisorSSHConfigurationError(
                    "Request arguments are not strict JSON"
                ) from None
            try:
                self._process.stdin.write(wire)
                self._process.stdin.flush()
            except (BrokenPipeError, OSError):
                raise self._transport_failure(
                    "Could not write supervisor request"
                ) from None

            try:
                kind, raw = self._responses.get(
                    timeout=self._response_timeout_seconds
                )
            except queue.Empty:
                raise SupervisorSSHTimeoutError(
                    "Supervisor response timed out"
                ) from None
            if kind == "eof":
                raise self._transport_failure(
                    "Supervisor process closed the response stream"
                )
            if kind != "line":
                raise SupervisorSSHProtocolError(
                    "Supervisor returned an invalid response frame"
                )
            response = _decode_response(
                raw,
                expected_request_id=request_id,
                expected_controller_id=self.controller_id,
            )
            if response["ok"] is False:
                error = response["error"]
                raise SupervisorRemoteError(
                    error["code"],
                    error["message"],
                )
            if operation == "shutdown":
                self._shutdown_sent = True
            return dict(response["result"])

    def _transport_failure(self, message: str):
        detail = self._stderr_pump.summary()
        if detail:
            message = "{}: {}".format(message, detail)
        return SupervisorSSHTransportError(message)

    def wait_closed(self, timeout_seconds: float = 3.0) -> int:
        try:
            returncode = self._process.wait(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            raise SupervisorSSHTimeoutError(
                "Supervisor process did not exit after shutdown"
            ) from None
        if (
            isinstance(returncode, bool)
            or not isinstance(returncode, int)
            or returncode != 0
        ):
            raise self._transport_failure(
                "Supervisor process exited unsuccessfully"
            )
        return returncode

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._process.poll() is None and not self._shutdown_sent:
                try:
                    self.request("stop", ttl_ms=1000)
                except SupervisorSSHError:
                    pass
                try:
                    self.request("shutdown", ttl_ms=1000)
                except SupervisorSSHError:
                    pass
            try:
                self._process.stdin.close()
            except OSError:
                pass
            if self._process.poll() is None:
                try:
                    self._process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self._process.terminate()
                    try:
                        self._process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self._process.kill()
                        self._process.wait(timeout=2)
        finally:
            self._closed = True
            for stream in (
                self._process.stdout,
                self._process.stderr,
            ):
                close = getattr(stream, "close", None)
                if callable(close):
                    try:
                        close()
                    except OSError:
                        pass

    def __enter__(self):
        return self

    def __exit__(self, _error_type, _error, _traceback):
        self.close()


def run_motion_free_supervisor_preflight(
    session: SupervisorSSHSession,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> Mapping[str, object]:
    """Exercise claim/heartbeat/arm/release without a motion request."""
    if not isinstance(session, SupervisorSSHSession):
        raise SupervisorSSHConfigurationError(
            "Preflight requires SupervisorSSHSession"
        )
    if not callable(sleep_fn):
        raise SupervisorSSHConfigurationError(
            "Preflight sleep function is invalid"
        )

    session_id = None
    shutdown_sent = False
    try:
        description = session.request("describe")
        drive_capability = (
            description.get("capabilities", {})
            .get("differential_drive_timed", {})
        )
        if (
            description.get("controller_id")
            != session.controller_id
            or description.get("motion_enabled") is not False
            or drive_capability.get("enabled") is not False
            or description.get("remaining_motion_budget") != 0
        ):
            raise SupervisorSSHProtocolError(
                "Daemon did not prove motion-free capability mode"
            )

        initial = session.request("status")
        if (
            initial.get("state") != "DISARMED"
            or initial.get("fault") is not None
            or initial.get("motion_allowed") is not False
        ):
            raise SupervisorSSHProtocolError(
                "Supervisor did not start safely disarmed"
            )

        claimed = session.request(
            "claim",
            {"owner_id": "mac-motion-free-preflight"},
        )
        session_id = claimed.get("session_id")
        _validate_identifier("session_id", session_id, 128)

        session.request(
            "heartbeat",
            {"session_id": session_id, "sequence_id": 1},
        )
        ready = None
        for _ in range(10):
            ready = session.request("status")
            if (
                ready.get("touch") == 0
                and ready.get("touch_released_samples", 0) >= 3
            ):
                break
            sleep_fn(0.02)
        else:
            raise SupervisorSSHProtocolError(
                "Touch release did not stabilize"
            )

        armed = session.request(
            "arm",
            {"session_id": session_id, "sequence_id": 2},
        )
        if (
            armed.get("state") != "ARMED_IDLE"
            or armed.get("motion_allowed") is not True
        ):
            raise SupervisorSSHProtocolError(
                "Supervisor did not enter armed idle"
            )

        observed_armed = session.request("status")
        if observed_armed.get("state") != "ARMED_IDLE":
            raise SupervisorSSHProtocolError(
                "Armed state was not observable"
            )

        released = session.request(
            "release",
            {"session_id": session_id, "sequence_id": 3},
        )
        session_id = None
        if (
            released.get("state") != "DISARMED"
            or released.get("motion_allowed") is not False
        ):
            raise SupervisorSSHProtocolError(
                "Supervisor release was not safely disarmed"
            )

        final = session.request("status")
        if (
            final.get("state") != "DISARMED"
            or final.get("session_active") is not False
            or final.get("motion_allowed") is not False
        ):
            raise SupervisorSSHProtocolError(
                "Final supervisor state was not safely disarmed"
            )

        session.request("shutdown", ttl_ms=1000)
        shutdown_sent = True
        session.wait_closed()
        return {
            "status": "completed",
            "mode": "motion-free-daemon-preflight",
            "controller_id": session.controller_id,
            "motion_requests_sent": 0,
            "description": description,
            "states": {
                "initial": initial,
                "armed": observed_armed,
                "released": released,
                "final": final,
            },
        }
    except BaseException:
        if not shutdown_sent:
            try:
                session.request("stop", ttl_ms=1000)
            except SupervisorSSHError:
                pass
            try:
                session.request("shutdown", ttl_ms=1000)
            except SupervisorSSHError:
                pass
        raise
