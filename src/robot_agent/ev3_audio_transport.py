"""Persistent binary-safe SSH transport for host-generated EV3 speech."""

from __future__ import annotations

import json
import queue
import subprocess
import threading
import time
from typing import Callable, Mapping, Optional

from .host_piper_speech import (
    HostSpeechError,
    WAVMetadata,
    validate_pcm16_mono_wav,
)


REMOTE_AUDIO_WORKER = "/home/robot/robot-llm/ev3/audio_playback_worker_cli.py"
REQUEST_SCHEMA = "ev3-audio-request/v1"
RESPONSE_SCHEMA = "ev3-audio-response/v1"
READY_SCHEMA = "ev3-audio-ready/v1"
MAX_HEADER_BYTES = 512
MAX_STDERR_BYTES = 8192
STARTUP_TIMEOUT_SECONDS = 30.0
POLL_SECONDS = 0.05
CLOSE_TIMEOUT_SECONDS = 0.15


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
        raise ValueError("EV3 audio SSH target is invalid")
    return value


def _remote_path(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value.startswith("/")
        or len(value) > 512
        or any(
            not (character.isalnum() or character in "/._-")
            for character in value
        )
    ):
        raise ValueError("EV3 audio worker path is invalid")
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
        or len(raw) > MAX_HEADER_BYTES
        or not raw.endswith(b"\n")
    ):
        raise HostSpeechError(
            "invalid_playback_response",
            "EV3 audio worker returned an invalid frame",
        )
    try:
        value = json.loads(
            raw.decode("ascii"),
            object_pairs_hook=_strict_object,
            parse_constant=lambda _value: (_ for _ in ()).throw(ValueError()),
        )
    except (TypeError, UnicodeError, ValueError):
        raise HostSpeechError(
            "invalid_playback_response",
            "EV3 audio worker returned invalid JSON",
        ) from None
    if not isinstance(value, dict):
        raise HostSpeechError(
            "invalid_playback_response",
            "EV3 audio worker response was not an object",
        )
    return value


class EV3WAVSSHSession:
    """One lazily started audio-only SSH process for one robot episode."""

    def __init__(
        self,
        target: str,
        *,
        process_factory: Callable[..., object] = subprocess.Popen,
        monotonic: Callable[[], float] = time.monotonic,
        connect_timeout_seconds: int = 5,
        startup_timeout_seconds: float = STARTUP_TIMEOUT_SECONDS,
        poll_seconds: float = POLL_SECONDS,
        remote_worker_path: str = REMOTE_AUDIO_WORKER,
    ):
        self.target = _target(target)
        self.remote_worker_path = _remote_path(remote_worker_path)
        if (
            not callable(process_factory)
            or not callable(monotonic)
            or isinstance(connect_timeout_seconds, bool)
            or not isinstance(connect_timeout_seconds, int)
            or not 1 <= connect_timeout_seconds <= 30
            or isinstance(startup_timeout_seconds, bool)
            or not isinstance(startup_timeout_seconds, (int, float))
            or not 1 <= float(startup_timeout_seconds) <= 120
            or isinstance(poll_seconds, bool)
            or not isinstance(poll_seconds, (int, float))
            or not 0.001 <= float(poll_seconds) <= 1
        ):
            raise ValueError("EV3 audio session configuration is invalid")
        self._process_factory = process_factory
        self._monotonic = monotonic
        self._connect_timeout_seconds = connect_timeout_seconds
        self._startup_timeout_seconds = float(startup_timeout_seconds)
        self._poll_seconds = float(poll_seconds)
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
            self.remote_worker_path,
        ]

    def _ensure_started(self) -> None:
        with self._state_lock:
            if self._aborted:
                raise HostSpeechError(
                    "speech_session_aborted",
                    "EV3 audio session was aborted",
                )
            if self._process is not None:
                return
            try:
                process = self._process_factory(
                    self.argv,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=0,
                )
            except OSError:
                raise HostSpeechError(
                    "speech_playback_unavailable",
                    "EV3 audio SSH process could not start",
                ) from None
            self._process = process
        threading.Thread(
            target=self._read_stdout,
            args=(process,),
            name="ev3-audio-stdout",
            daemon=True,
        ).start()
        threading.Thread(
            target=self._read_stderr,
            args=(process,),
            name="ev3-audio-stderr",
            daemon=True,
        ).start()

    def _read_stdout(self, process) -> None:
        try:
            while True:
                raw = process.stdout.readline(MAX_HEADER_BYTES + 1)
                if raw == b"":
                    self._responses.put(("eof", None))
                    return
                self._responses.put(("line", raw))
        except BaseException:
            self._responses.put(("read_error", None))

    def _read_stderr(self, process) -> None:
        try:
            while True:
                chunk = process.stderr.read(512)
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

    def _write_all(self, raw: bytes, cancel_event=None) -> bool:
        process = self._process
        if process is None or process.stdin is None:
            raise HostSpeechError(
                "speech_session_not_started",
                "EV3 audio session is not started",
            )
        view = memoryview(raw)
        while view:
            if cancel_event is not None and cancel_event.is_set():
                self.abort()
                return False
            try:
                written = process.stdin.write(view[:65536])
            except (IOError, OSError, ValueError):
                self.abort()
                raise HostSpeechError(
                    "speech_transport_write_failed",
                    "EV3 audio request could not be written",
                ) from None
            if not isinstance(written, int) or written <= 0:
                self.abort()
                raise HostSpeechError(
                    "speech_transport_write_failed",
                    "EV3 audio request write made no progress",
                )
            view = view[written:]
        return True

    def _write_request(
        self,
        request_id: str,
        operation: str,
        audio: bytes,
        cancel_event=None,
    ) -> bool:
        header = (
            json.dumps(
                {
                    "schema": REQUEST_SCHEMA,
                    "request_id": request_id,
                    "operation": operation,
                    "byte_count": len(audio),
                },
                ensure_ascii=True,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("ascii")
        if len(header) > MAX_HEADER_BYTES:
            raise HostSpeechError(
                "speech_request_too_large",
                "EV3 audio request header is too large",
            )
        if not self._write_all(header, cancel_event):
            return False
        if audio and not self._write_all(audio, cancel_event):
            return False
        try:
            self._process.stdin.flush()
        except (IOError, OSError, ValueError):
            self.abort()
            raise HostSpeechError(
                "speech_transport_write_failed",
                "EV3 audio request could not be flushed",
            ) from None
        return True

    def _next_frame(self, timeout_seconds: float, cancel_event=None):
        deadline = self._monotonic() + timeout_seconds
        while True:
            if cancel_event is not None and cancel_event.is_set():
                self.abort()
                return None
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                self.abort()
                raise HostSpeechError("speech_timeout", "EV3 audio worker timed out")
            try:
                kind, raw = self._responses.get(
                    timeout=min(self._poll_seconds, remaining)
                )
            except queue.Empty:
                continue
            if kind != "line":
                self.abort()
                raise HostSpeechError(
                    "speech_transport_closed",
                    "EV3 audio worker closed before replying",
                )
            return _decode(raw)

    def _ensure_ready(self, cancel_event) -> bool:
        if self._ready:
            return True
        value = self._next_frame(self._startup_timeout_seconds, cancel_event)
        if value is None:
            return False
        if value != {"schema": READY_SCHEMA, "status": "ready"}:
            self.abort()
            raise HostSpeechError(
                "invalid_playback_response",
                "EV3 audio ready frame is invalid",
            )
        self._ready = True
        return True

    def _await_result(self, request_id, timeout_seconds, cancel_event=None):
        value = self._next_frame(timeout_seconds, cancel_event)
        if value is None:
            return None
        if (
            set(value) not in (
                {"schema", "request_id", "ok", "result"},
                {"schema", "request_id", "ok", "error"},
            )
            or value.get("schema") != RESPONSE_SCHEMA
            or value.get("request_id") != request_id
            or type(value.get("ok")) is not bool
        ):
            self.abort()
            raise HostSpeechError(
                "invalid_playback_response",
                "EV3 audio response correlation is invalid",
            )
        if value["ok"]:
            if not isinstance(value.get("result"), dict):
                raise HostSpeechError(
                    "invalid_playback_response",
                    "EV3 audio result is invalid",
                )
            return value["result"]
        error = value.get("error")
        if (
            not isinstance(error, dict)
            or set(error) != {"code", "fatal"}
            or not isinstance(error["code"], str)
            or not error["code"]
            or type(error["fatal"]) is not bool
        ):
            self.abort()
            raise HostSpeechError(
                "invalid_playback_response",
                "EV3 audio error is invalid",
            )
        if error["fatal"]:
            self.abort()
        raise HostSpeechError(error["code"], "EV3 audio worker rejected playback")

    @staticmethod
    def _receipt(metadata: WAVMetadata):
        return {
            "status": "completed",
            "bytes": metadata.byte_count,
            "channels": metadata.channels,
            "sample_width_bytes": metadata.sample_width_bytes,
            "sample_rate_hz": metadata.sample_rate_hz,
            "frames": metadata.frames,
            "duration_ms": metadata.duration_ms,
        }

    def play(self, raw: bytes, cancel_event: threading.Event):
        metadata = validate_pcm16_mono_wav(raw)
        if not isinstance(cancel_event, threading.Event):
            raise HostSpeechError("invalid_speech_cancel", "Speech cancellation is invalid")
        if cancel_event.is_set():
            return None
        with self._io_lock:
            self._ensure_started()
            if not self._ensure_ready(cancel_event):
                return None
            request_id = self._next_request_id("audio")
            if not self._write_request(
                request_id,
                "play",
                raw,
                cancel_event,
            ):
                return None
            result = self._await_result(
                request_id,
                metadata.duration_ms / 1000.0 + 3.0,
                cancel_event,
            )
        if result is not None and result != self._receipt(metadata):
            self.abort()
            raise HostSpeechError(
                "invalid_playback_response",
                "EV3 audio receipt is invalid",
            )
        return result

    def abort(self) -> bool:
        with self._state_lock:
            self._aborted = True
            process = self._process
        return _stop_process(process, CLOSE_TIMEOUT_SECONDS)

    def close(self, timeout_seconds: float = CLOSE_TIMEOUT_SECONDS) -> bool:
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not 0 <= float(timeout_seconds) <= 1
        ):
            raise ValueError("audio session close timeout is invalid")
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
                    self._write_request(request_id, "shutdown", b"")
                    result = self._await_result(
                        request_id,
                        float(timeout_seconds),
                    )
                if result != {"status": "shutdown"}:
                    raise HostSpeechError(
                        "invalid_playback_response",
                        "EV3 audio shutdown was not acknowledged",
                    )
            except Exception:
                pass
        return _stop_process(process, float(timeout_seconds))


__all__ = (
    "EV3WAVSSHSession",
    "READY_SCHEMA",
    "REMOTE_AUDIO_WORKER",
    "REQUEST_SCHEMA",
    "RESPONSE_SCHEMA",
)
