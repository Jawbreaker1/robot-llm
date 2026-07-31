"""Persistent, cancellable SSH transport for the EV3 speech companion."""

from __future__ import annotations

import json
import queue
import subprocess
import threading
import time
from typing import Callable, Mapping, Optional


REMOTE_SPEECH_WORKER = "/home/robot/robot-llm/ev3/speech_worker_cli.py"
REQUEST_SCHEMA = "ev3-speech-request/v1"
RESPONSE_SCHEMA = "ev3-speech-response/v1"
READY_SCHEMA = "ev3-speech-ready/v1"
MAX_FRAME_BYTES = 4096
MAX_STDERR_BYTES = 8192
DEFAULT_REQUEST_TIMEOUT_SECONDS = 45.0
DEFAULT_CLOSE_TIMEOUT_SECONDS = 0.5
DEFAULT_POLL_SECONDS = 0.05


class EV3SpeechTransportError(RuntimeError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


class EV3SpeechRemoteError(EV3SpeechTransportError):
    def __init__(self, code: str, message: str, fatal: bool):
        self.fatal = fatal
        super().__init__(code, message)


def _target(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or value.startswith("-")
        or len(value) > 255
        or any(
            not (character.isalnum() or character in "._-@:%+")
            for character in value
        )
    ):
        raise EV3SpeechTransportError(
            "invalid_speech_target",
            "EV3 speech SSH target is invalid",
        )
    return value


def _stop_process(process, timeout_seconds: float) -> bool:
    if process is None:
        return True
    try:
        if process.poll() is not None:
            return True
    except (OSError, ProcessLookupError):
        return True
    try:
        process.terminate()
    except (OSError, ProcessLookupError):
        pass
    try:
        process.wait(timeout=timeout_seconds)
        return True
    except subprocess.TimeoutExpired:
        pass
    except (OSError, ProcessLookupError):
        return True
    try:
        process.kill()
    except (OSError, ProcessLookupError):
        pass
    try:
        process.wait(timeout=timeout_seconds)
        return True
    except (OSError, ProcessLookupError):
        return True
    except subprocess.TimeoutExpired:
        return False


def _strict_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError
        value[key] = item
    return value


def _decode(raw: object) -> Mapping[str, object]:
    if (
        not isinstance(raw, bytes)
        or len(raw) > MAX_FRAME_BYTES
        or not raw.endswith(b"\n")
    ):
        raise EV3SpeechTransportError(
            "invalid_speech_response",
            "EV3 speech worker returned an invalid frame",
        )
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=lambda _value: (_ for _ in ()).throw(ValueError()),
        )
    except (TypeError, UnicodeError, ValueError):
        raise EV3SpeechTransportError(
            "invalid_speech_response",
            "EV3 speech worker returned invalid JSON",
        ) from None
    if not isinstance(value, dict):
        raise EV3SpeechTransportError(
            "invalid_speech_response",
            "EV3 speech worker response was not an object",
        )
    return value


class EV3SpeechSSHSession:
    """One foreground, speech-only SSH process for one robot episode."""

    def __init__(
        self,
        target: str,
        *,
        process_factory: Callable[..., object] = subprocess.Popen,
        monotonic: Callable[[], float] = time.monotonic,
        connect_timeout_seconds: int = 5,
        request_timeout_seconds: float = DEFAULT_REQUEST_TIMEOUT_SECONDS,
        poll_seconds: float = DEFAULT_POLL_SECONDS,
        remote_worker_path: str = REMOTE_SPEECH_WORKER,
    ):
        self.target = _target(target)
        if not callable(process_factory) or not callable(monotonic):
            raise EV3SpeechTransportError(
                "invalid_speech_transport",
                "EV3 speech transport dependency is invalid",
            )
        if (
            isinstance(connect_timeout_seconds, bool)
            or not isinstance(connect_timeout_seconds, int)
            or not 1 <= connect_timeout_seconds <= 30
            or isinstance(request_timeout_seconds, bool)
            or not isinstance(request_timeout_seconds, (int, float))
            or not 1 <= float(request_timeout_seconds) <= 120
            or isinstance(poll_seconds, bool)
            or not isinstance(poll_seconds, (int, float))
            or not 0.001 <= float(poll_seconds) <= 1
            or not isinstance(remote_worker_path, str)
            or not remote_worker_path.startswith("/")
            or len(remote_worker_path) > 512
            or any(
                not (character.isalnum() or character in "/._-")
                for character in remote_worker_path
            )
        ):
            raise EV3SpeechTransportError(
                "invalid_speech_transport",
                "EV3 speech transport configuration is invalid",
            )
        self._process_factory = process_factory
        self._monotonic = monotonic
        self._connect_timeout_seconds = connect_timeout_seconds
        self._request_timeout_seconds = float(request_timeout_seconds)
        self._poll_seconds = float(poll_seconds)
        self._remote_worker_path = remote_worker_path
        self._process = None
        self._responses = queue.Queue()
        self._stderr = bytearray()
        self._stderr_lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._sequence = 0
        self._ready = False
        self._aborted = False

    @property
    def argv(self):
        return [
            "ssh",
            "-T",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout={}".format(self._connect_timeout_seconds),
            "-o",
            "StrictHostKeyChecking=yes",
            self.target,
            "python3",
            self._remote_worker_path,
        ]

    def start(self) -> None:
        with self._state_lock:
            if self._process is not None or self._aborted:
                raise EV3SpeechTransportError(
                    "speech_session_not_startable",
                    "EV3 speech session cannot be started",
                )
            try:
                process = self._process_factory(
                    self.argv,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=0,
                )
            except OSError:
                raise EV3SpeechTransportError(
                    "speech_transport_start_failed",
                    "EV3 speech SSH process could not start",
                ) from None
            self._process = process
        threading.Thread(
            target=self._read_stdout,
            name="ev3-speech-stdout",
            daemon=True,
        ).start()
        threading.Thread(
            target=self._read_stderr,
            name="ev3-speech-stderr",
            daemon=True,
        ).start()

    def _read_stdout(self) -> None:
        try:
            while True:
                raw = self._process.stdout.readline(MAX_FRAME_BYTES + 1)
                if raw == b"":
                    self._responses.put(("eof", None))
                    return
                self._responses.put(("line", raw))
        except BaseException:
            self._responses.put(("read_error", None))

    def _read_stderr(self) -> None:
        try:
            while True:
                chunk = self._process.stderr.read(512)
                if not chunk:
                    return
                with self._stderr_lock:
                    remaining = MAX_STDERR_BYTES - len(self._stderr)
                    if remaining > 0:
                        self._stderr.extend(chunk[:remaining])
        except BaseException:
            return

    def _next_request_id(self, prefix: str) -> str:
        self._sequence += 1
        return "{}-{}".format(prefix, self._sequence)

    def _write_request(
        self,
        request_id: str,
        operation: str,
        arguments: Mapping[str, object],
    ) -> None:
        process = self._process
        if process is None or process.stdin is None:
            raise EV3SpeechTransportError(
                "speech_session_not_started",
                "EV3 speech session is not started",
            )
        value = {
            "schema": REQUEST_SCHEMA,
            "request_id": request_id,
            "operation": operation,
            "arguments": dict(arguments),
        }
        raw = (
            json.dumps(
                value,
                ensure_ascii=True,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
        if len(raw) > MAX_FRAME_BYTES:
            raise EV3SpeechTransportError(
                "speech_request_too_large",
                "EV3 speech request is too large",
            )
        try:
            process.stdin.write(raw)
            process.stdin.flush()
        except (IOError, OSError, ValueError):
            raise EV3SpeechTransportError(
                "speech_transport_write_failed",
                "EV3 speech request could not be written",
            ) from None

    def _await_response(
        self,
        request_id: str,
        *,
        timeout_seconds: float,
        cancel_event: Optional[threading.Event] = None,
    ):
        deadline = self._monotonic() + timeout_seconds
        while True:
            if cancel_event is not None and cancel_event.is_set():
                self.abort()
                return None
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                self.abort()
                raise EV3SpeechTransportError(
                    "speech_timeout",
                    "EV3 speech worker timed out",
                )
            try:
                kind, raw = self._responses.get(
                    timeout=min(self._poll_seconds, remaining)
                )
            except queue.Empty:
                continue
            if kind != "line":
                code = (
                    "speech_transport_read_failed"
                    if kind == "read_error"
                    else "speech_transport_closed"
                )
                raise EV3SpeechTransportError(
                    code,
                    "EV3 speech worker closed before replying",
                )
            value = _decode(raw)
            if set(value) == {"schema", "status"}:
                if (
                    value["schema"] != READY_SCHEMA
                    or value["status"] != "ready"
                    or self._ready
                ):
                    raise EV3SpeechTransportError(
                        "invalid_speech_response",
                        "EV3 speech ready frame is invalid",
                    )
                self._ready = True
                continue
            if (
                set(value) not in (
                    {"schema", "request_id", "ok", "result"},
                    {"schema", "request_id", "ok", "error"},
                )
                or value.get("schema") != RESPONSE_SCHEMA
                or value.get("request_id") != request_id
                or type(value.get("ok")) is not bool
            ):
                raise EV3SpeechTransportError(
                    "invalid_speech_response",
                    "EV3 speech response correlation is invalid",
                )
            if value["ok"]:
                result = value.get("result")
                if not isinstance(result, dict):
                    raise EV3SpeechTransportError(
                        "invalid_speech_response",
                        "EV3 speech result is invalid",
                    )
                return result
            error = value.get("error")
            if (
                not isinstance(error, dict)
                or set(error) != {"code", "message", "fatal"}
                or not isinstance(error["code"], str)
                or not error["code"]
                or not isinstance(error["message"], str)
                or not error["message"]
                or type(error["fatal"]) is not bool
            ):
                raise EV3SpeechTransportError(
                    "invalid_speech_response",
                    "EV3 speech error is invalid",
                )
            if error["fatal"]:
                self.abort()
            raise EV3SpeechRemoteError(
                error["code"],
                error["message"],
                error["fatal"],
            )

    def speak(
        self,
        text: str,
        locale: str,
        cancel_event: threading.Event,
    ):
        if (
            not isinstance(text, str)
            or not text
            or len(text) > 160
            or locale not in ("sv", "en")
            or not isinstance(cancel_event, threading.Event)
        ):
            raise EV3SpeechTransportError(
                "invalid_speech_request",
                "EV3 speech request is invalid",
            )
        if cancel_event.is_set():
            return None
        with self._io_lock:
            request_id = self._next_request_id("speech")
            self._write_request(
                request_id,
                "speak",
                {"text": text, "voice": locale},
            )
            result = self._await_response(
                request_id,
                timeout_seconds=self._request_timeout_seconds,
                cancel_event=cancel_event,
            )
        if result is None:
            return None
        if result.get("status") != "completed":
            raise EV3SpeechTransportError(
                "invalid_speech_response",
                "EV3 speech did not complete",
            )
        return result

    def abort(self) -> bool:
        with self._state_lock:
            self._aborted = True
            process = self._process
        return _stop_process(process, DEFAULT_CLOSE_TIMEOUT_SECONDS)

    def close(
        self,
        timeout_seconds: float = DEFAULT_CLOSE_TIMEOUT_SECONDS,
    ) -> bool:
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not 0 <= float(timeout_seconds) <= 5
        ):
            raise ValueError("speech session close timeout is invalid")
        with self._state_lock:
            process = self._process
            aborted = self._aborted
        if process is None:
            return True
        try:
            if process.poll() is not None:
                return True
        except (OSError, ProcessLookupError):
            return True
        if not aborted and float(timeout_seconds) > 0:
            try:
                with self._io_lock:
                    request_id = self._next_request_id("shutdown")
                    self._write_request(request_id, "shutdown", {})
                    result = self._await_response(
                        request_id,
                        timeout_seconds=float(timeout_seconds),
                    )
                if result.get("status") != "shutdown":
                    raise EV3SpeechTransportError(
                        "invalid_speech_response",
                        "EV3 speech shutdown was not acknowledged",
                    )
            except Exception:
                pass
        return _stop_process(process, float(timeout_seconds))


__all__ = (
    "EV3SpeechRemoteError",
    "EV3SpeechSSHSession",
    "EV3SpeechTransportError",
    "REMOTE_SPEECH_WORKER",
)
