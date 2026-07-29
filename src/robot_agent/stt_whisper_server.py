"""Lifecycle wrapper for one warm, loopback-only whisper.cpp server."""

from __future__ import annotations

from collections import deque
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import pty
import secrets
import signal
import shutil
import subprocess
import threading
import time
from typing import BinaryIO, Optional

from .http_transport import direct_http_request


DEFAULT_WHISPER_SERVER_PORT = 8178
DEFAULT_WHISPER_STARTUP_TIMEOUT_SECONDS = 30.0
_REQUEST_TOKEN_BYTES = 24
_MAX_CHILD_LOG_LINE_BYTES = 4 * 1024
_CHILD_LOG_TAIL_LINES = 40
_KILL_WAIT_SECONDS = 1.0
_DRAINER_JOIN_SECONDS = 1.0


class WhisperServerError(RuntimeError):
    """The managed local whisper.cpp process could not become ready."""


def _open_stdout_capture():
    """Return a PTY reader and the child's writable stdout descriptor."""

    master_fd, slave_fd = pty.openpty()
    try:
        stream = os.fdopen(master_fd, "rb", buffering=0)
    except Exception:
        os.close(master_fd)
        os.close(slave_fd)
        raise
    return stream, slave_fd


def _close_stream(stream: Optional[BinaryIO]) -> None:
    if stream is None:
        return
    try:
        stream.close()
    except (OSError, ValueError):
        pass


@contextmanager
def _defer_termination_signals():
    """Prevent SIGINT/SIGTERM between child creation and ownership recording."""

    if threading.current_thread() is not threading.main_thread():
        yield
        return
    previous = {}
    pending = []

    def defer(signum, _frame):
        pending.append(signum)

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            handler = signal.getsignal(signum)
            if handler is signal.SIG_IGN:
                continue
            previous[signum] = handler
            signal.signal(signum, defer)
        yield
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        if pending:
            raise KeyboardInterrupt


class WhisperCppServer:
    """Start once, keep the model warm, and stop with the dashboard."""

    def __init__(
        self,
        model_path: str,
        *,
        binary: str = "whisper-server",
        port: int = DEFAULT_WHISPER_SERVER_PORT,
        threads: int = 4,
        use_gpu: bool = True,
        startup_timeout_seconds: float = (
            DEFAULT_WHISPER_STARTUP_TIMEOUT_SECONDS
        ),
    ):
        resolved_binary = (
            shutil.which(binary)
            if isinstance(binary, str)
            else None
        )
        model = (
            Path(model_path).expanduser()
            if isinstance(model_path, str)
            else None
        )
        if (
            resolved_binary is None
            or model is None
            or not model.is_file()
            or model.stat().st_size < 1_000_000
            or isinstance(port, bool)
            or not isinstance(port, int)
            or not 1 <= port <= 65_535
            or isinstance(threads, bool)
            or not isinstance(threads, int)
            or not 1 <= threads <= 64
            or not isinstance(use_gpu, bool)
            or isinstance(startup_timeout_seconds, bool)
            or not isinstance(
                startup_timeout_seconds,
                (int, float),
            )
            or not math.isfinite(float(startup_timeout_seconds))
            or not 1 <= float(startup_timeout_seconds) <= 120
        ):
            raise WhisperServerError(
                "Whisper server configuration is invalid"
            )
        self._binary = resolved_binary
        self._model = model.resolve()
        self._port = port
        self._threads = threads
        self._use_gpu = use_gpu
        self._startup_timeout_seconds = float(
            startup_timeout_seconds
        )
        self._process: Optional[subprocess.Popen] = None
        self._request_prefix: Optional[str] = None
        self._stdout_stream: Optional[BinaryIO] = None
        self._stderr_stream: Optional[BinaryIO] = None
        self._stdout_tail = deque(maxlen=_CHILD_LOG_TAIL_LINES)
        self._stderr_tail = deque(maxlen=_CHILD_LOG_TAIL_LINES)
        self._stdout_thread = None
        self._stderr_thread = None
        self._own_listener_ready = threading.Event()

    @property
    def base_url(self) -> str:
        prefix = self._request_prefix
        if self._process is None or prefix is None:
            raise WhisperServerError(
                "Whisper server has no active private endpoint"
            )
        return "http://127.0.0.1:{}{}".format(
            self._port,
            prefix,
        )

    @property
    def model_id(self) -> str:
        name = self._model.stem
        safe = "".join(
            character
            if character.isascii()
            and (character.isalnum() or character in "-_.")
            else "-"
            for character in name
        )
        return safe[:200] or "whisper-model"

    def _new_request_prefix(self) -> str:
        token = secrets.token_hex(_REQUEST_TOKEN_BYTES)
        if (
            not isinstance(token, str)
            or len(token) != _REQUEST_TOKEN_BYTES * 2
            or not token.isascii()
            or not all(
                character in "0123456789abcdef"
                for character in token
            )
        ):
            raise WhisperServerError(
                "Whisper request prefix generation failed"
            )
        return "/stt-" + token

    def _readiness_line(self) -> bytes:
        return (
            "whisper server listening at http://127.0.0.1:{}".format(
                self._port
            )
        ).encode("ascii")

    def _safe_log_line(self, raw: bytes) -> str:
        value = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        if self._request_prefix:
            value = value.replace(
                self._request_prefix,
                "[private-path]",
            )
        return value

    def _drain_stream(
        self,
        stream: BinaryIO,
        tail,
        *,
        observe_readiness: bool,
    ) -> None:
        expected = self._readiness_line()
        try:
            while True:
                raw = stream.readline(_MAX_CHILD_LOG_LINE_BYTES + 1)
                if not raw:
                    return
                if (
                    observe_readiness
                    and raw.rstrip(b"\r\n") == expected
                ):
                    self._own_listener_ready.set()
                tail.append(self._safe_log_line(raw))
        except (OSError, ValueError):
            return

    def _start_drainers(self) -> None:
        stdout_stream = self._stdout_stream
        stderr_stream = self._stderr_stream
        if stdout_stream is None or stderr_stream is None:
            raise WhisperServerError(
                "Whisper output capture was not initialized"
            )
        self._stdout_thread = threading.Thread(
            target=lambda: self._drain_stream(
                stdout_stream,
                self._stdout_tail,
                observe_readiness=True,
            ),
            name="robot-llm-whisper-stdout",
            daemon=True,
        )
        self._stderr_thread = threading.Thread(
            target=lambda: self._drain_stream(
                stderr_stream,
                self._stderr_tail,
                observe_readiness=False,
            ),
            name="robot-llm-whisper-stderr",
            daemon=True,
        )
        self._stdout_thread.start()
        self._stderr_thread.start()

    def _health_is_ready(self) -> bool:
        try:
            response = direct_http_request(
                "GET",
                self.base_url + "/health",
                {"Accept": "application/json"},
                None,
                0.25,
                4 * 1024,
            )
            payload = json.loads(response.body.decode("utf-8"))
            server_headers = response.header_values("server")
            return (
                response.status_code == 200
                and payload == {"status": "ok"}
                and any(
                    value.lower() == "whisper.cpp"
                    for value in server_headers
                )
            )
        except Exception:
            return False

    def start(self) -> None:
        if self._process is not None:
            raise WhisperServerError(
                "Whisper server is already started"
            )
        self._request_prefix = self._new_request_prefix()
        self._own_listener_ready.clear()
        self._stdout_tail.clear()
        self._stderr_tail.clear()
        stdout_stream = None
        stdout_child_fd = None
        try:
            with _defer_termination_signals():
                stdout_stream, stdout_child_fd = _open_stdout_capture()
                command = [
                        self._binary,
                        "--model",
                        str(self._model),
                        "--host",
                        "127.0.0.1",
                        "--port",
                        str(self._port),
                        "--threads",
                        str(self._threads),
                        "--language",
                        "auto",
                        "--no-timestamps",
                        "--request-path",
                        self._request_prefix,
                    ]
                if not self._use_gpu:
                    command.append("--no-gpu")
                process = subprocess.Popen(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=stdout_child_fd,
                    stderr=subprocess.PIPE,
                    close_fds=True,
                )
                self._process = process
                self._stdout_stream = stdout_stream
                self._stderr_stream = process.stderr
                os.close(stdout_child_fd)
                stdout_child_fd = None
            self._start_drainers()

            deadline = (
                time.monotonic() + self._startup_timeout_seconds
            )
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise WhisperServerError(
                        "Whisper server exited before becoming ready"
                    )
                if (
                    self._own_listener_ready.is_set()
                    and self._health_is_ready()
                    and process.poll() is None
                ):
                    return
                time.sleep(0.05)
            raise WhisperServerError(
                "Whisper server did not become ready before its deadline"
            )
        except BaseException as startup_error:
            if stdout_child_fd is not None:
                try:
                    os.close(stdout_child_fd)
                except OSError:
                    pass
            if self._process is None:
                _close_stream(stdout_stream)
                self._request_prefix = None
            else:
                try:
                    self.stop()
                except BaseException as cleanup_error:
                    raise cleanup_error from startup_error
            raise

    @staticmethod
    def _validated_stop_timeout(timeout_seconds: float) -> float:
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not math.isfinite(float(timeout_seconds))
            or not 0 <= float(timeout_seconds) <= 60
        ):
            raise WhisperServerError(
                "Whisper stop timeout is invalid"
            )
        return float(timeout_seconds)

    def _join_drainers(self) -> bool:
        threads = (
            self._stdout_thread,
            self._stderr_thread,
        )
        _close_stream(self._stdout_stream)
        _close_stream(self._stderr_stream)
        for thread in threads:
            if (
                thread is not None
                and thread is not threading.current_thread()
            ):
                thread.join(_DRAINER_JOIN_SECONDS)
        return not any(
            thread is not None and thread.is_alive()
            for thread in threads
        )

    def _clear_stopped_process(self) -> None:
        self._process = None
        self._request_prefix = None
        self._stdout_stream = None
        self._stderr_stream = None
        self._stdout_thread = None
        self._stderr_thread = None
        self._own_listener_ready.clear()

    def stop(self, timeout_seconds: float = 2.0) -> None:
        timeout = self._validated_stop_timeout(timeout_seconds)
        process = self._process
        if process is None:
            return
        if process.poll() is None:
            try:
                process.terminate()
            except OSError:
                if process.poll() is None:
                    raise WhisperServerError(
                        "Whisper server could not be terminated"
                    ) from None
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                except OSError:
                    if process.poll() is None:
                        raise WhisperServerError(
                            "Whisper server could not be killed"
                        ) from None
                try:
                    process.wait(timeout=_KILL_WAIT_SECONDS)
                except subprocess.TimeoutExpired:
                    raise WhisperServerError(
                        "Whisper server did not stop"
                    ) from None
        if process.poll() is None:
            raise WhisperServerError(
                "Whisper server death could not be verified"
            )
        if not self._join_drainers():
            raise WhisperServerError(
                "Whisper output drainers did not stop"
            )
        self._clear_stopped_process()
