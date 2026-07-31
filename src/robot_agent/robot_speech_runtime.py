"""Bounded asynchronous speech playback for one physical robot episode.

Navigation only offers text to this runtime.  A dedicated worker invokes the
injected speaker, so slow audio generation or playback never blocks the
observe/plan/act loop or motor cleanup.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import subprocess
import threading
import time
import unicodedata
from typing import Callable, Mapping, Optional

from .dashboard_contract import RESPONSE_LOCALES
from .shadow_cli import MAX_SSH_OUTPUT_BYTES, REMOTE_ROBOT_CLI


MAX_SPEECH_CHARACTERS = 160
VOICE_BY_LOCALE = {
    "sv": "sv",
    "en": "en",
}
DEFAULT_PROCESS_POLL_SECONDS = 0.05
DEFAULT_PROCESS_STOP_SECONDS = 0.5
MAX_DEDUPLICATION_UTTERANCES = 64


class RobotSpeechRuntimeError(RuntimeError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class SpeechItem:
    episode_id: str
    sequence: int
    text: str
    locale: str


def _bounded_text(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > MAX_SPEECH_CHARACTERS
        or any(
            ord(character) < 32 and character not in "\n\r\t"
            for character in value
        )
    ):
        raise RobotSpeechRuntimeError(
            "invalid_speech_text",
            "Speech text is invalid",
        )
    return value


def _identifier(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 128
        or not value.isascii()
        or not all(
            character.isalnum() or character in "-_"
            for character in value
        )
    ):
        raise RobotSpeechRuntimeError(
            "invalid_speech_episode",
            "Speech episode identity is invalid",
        )
    return value


def _normalized_speech_text(value: str) -> str:
    """Return a language-agnostic key for audibly equivalent text."""

    words = []
    current = []
    for character in unicodedata.normalize("NFKC", value).casefold():
        if character.isspace() or unicodedata.category(character).startswith(
            "P"
        ):
            if current:
                words.append("".join(current))
                current = []
        else:
            current.append(character)
    if current:
        words.append("".join(current))
    return " ".join(words)


class RobotSpeechRuntime:
    """Latest-pending-wins speech worker with explicit cancellation."""

    def __init__(
        self,
        *,
        speaker: Callable[[str, str, threading.Event], object],
        speaker_close: Optional[Callable[[], object]] = None,
        event_sink: Optional[
            Callable[[Mapping[str, object]], None]
        ] = None,
        thread_name: str = "robot-speech",
    ):
        if not callable(speaker):
            raise ValueError("speech speaker is invalid")
        if speaker_close is not None and not callable(speaker_close):
            raise ValueError("speech speaker close callback is invalid")
        if event_sink is not None and not callable(event_sink):
            raise ValueError("speech event sink is invalid")
        if (
            not isinstance(thread_name, str)
            or not thread_name
            or len(thread_name) > 128
        ):
            raise ValueError("speech thread name is invalid")
        self._speaker = speaker
        self._speaker_close = speaker_close
        self._speaker_closed = False
        self._event_sink = event_sink
        self._condition = threading.Condition()
        self._pending = None
        self._active = None
        self._active_cancel = None
        self._sequence = 0
        self._deduplication_windows = {}
        self._accepting = True
        self._closed = False
        self._started = False
        self._thread = threading.Thread(
            target=self._run,
            name=thread_name,
            daemon=True,
        )

    def _emit(self, status: str, item: SpeechItem, reason=None) -> None:
        if self._event_sink is None:
            return
        event = {
            "event": "speech_{}".format(status),
            "speech_status": status,
            "episode_id": item.episode_id,
            "sequence": item.sequence,
            "locale": item.locale,
            "characters": len(item.text),
        }
        if reason is not None:
            event["reason"] = reason
        try:
            self._event_sink(event)
        except Exception:
            # Observability must never become motion or audio authority.
            return

    def start(self) -> None:
        with self._condition:
            if self._closed:
                raise RobotSpeechRuntimeError(
                    "speech_runtime_closed",
                    "Speech runtime is closed",
                )
            if self._started:
                return
            self._started = True
            self._thread.start()

    def offer(
        self,
        *,
        episode_id: str,
        text: str,
        locale: str,
        progress_revision: int = 0,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> int:
        checked_episode = _identifier(episode_id)
        checked_text = _bounded_text(text)
        if locale not in RESPONSE_LOCALES or locale not in VOICE_BY_LOCALE:
            raise RobotSpeechRuntimeError(
                "unsupported_speech_locale",
                "Speech locale is unsupported",
            )
        if cancel_requested is not None and not callable(cancel_requested):
            raise RobotSpeechRuntimeError(
                "invalid_speech_cancellation_probe",
                "Speech cancellation probe is invalid",
            )
        if (
            isinstance(progress_revision, bool)
            or not isinstance(progress_revision, int)
            or progress_revision < 0
        ):
            raise RobotSpeechRuntimeError(
                "invalid_speech_progress_revision",
                "Speech progress revision is invalid",
            )
        fingerprint = (locale, _normalized_speech_text(checked_text))
        dropped = None
        duplicate = None
        with self._condition:
            if not self._started or not self._accepting or self._closed:
                raise RobotSpeechRuntimeError(
                    "speech_runtime_not_accepting",
                    "Speech runtime is not accepting utterances",
                )
            try:
                cancelled = (
                    cancel_requested is not None
                    and cancel_requested() is True
                )
            except BaseException:
                raise RobotSpeechRuntimeError(
                    "speech_cancellation_probe_failed",
                    "Speech cancellation state could not be read",
                ) from None
            if cancelled:
                raise RobotSpeechRuntimeError(
                    "speech_episode_cancelled",
                    "Cancelled episode cannot accept speech",
                )
            window = self._deduplication_windows.get(checked_episode)
            if window is None or progress_revision > window[0]:
                fingerprints = {}
                self._deduplication_windows[checked_episode] = (
                    progress_revision,
                    fingerprints,
                )
            elif progress_revision < window[0]:
                raise RobotSpeechRuntimeError(
                    "stale_speech_progress_revision",
                    "Speech progress revision moved backwards",
                )
            else:
                fingerprints = window[1]
            self._sequence += 1
            item = SpeechItem(
                episode_id=checked_episode,
                sequence=self._sequence,
                text=checked_text,
                locale=locale,
            )
            if fingerprint in fingerprints:
                duplicate = item
            else:
                if len(fingerprints) >= MAX_DEDUPLICATION_UTTERANCES:
                    del fingerprints[next(iter(fingerprints))]
                fingerprints[fingerprint] = None
                dropped = self._pending
                self._pending = item
                self._condition.notify_all()
        if duplicate is not None:
            self._emit(
                "dropped",
                duplicate,
                reason="duplicate_without_progress",
            )
            return duplicate.sequence
        if dropped is not None:
            self._emit("dropped", dropped, reason="replaced_by_newer")
        self._emit("queued", item)
        return item.sequence

    def cancel_episode(self, episode_id: str) -> None:
        checked = _identifier(episode_id)
        dropped = None
        with self._condition:
            self._deduplication_windows.pop(checked, None)
            if (
                self._pending is not None
                and self._pending.episode_id == checked
            ):
                dropped = self._pending
                self._pending = None
            if (
                self._active is not None
                and self._active.episode_id == checked
            ):
                self._active_cancel.set()
            self._condition.notify_all()
        if dropped is not None:
            self._emit("cancelled", dropped, reason="episode_cancelled")

    def close(
        self,
        *,
        drain: bool = False,
        timeout_seconds: float = 1.0,
    ) -> bool:
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not 0 <= float(timeout_seconds) <= 30
        ):
            raise ValueError("speech close timeout is invalid")
        dropped = None
        with self._condition:
            if self._closed:
                thread = self._thread
            else:
                self._accepting = False
                if not drain:
                    dropped = self._pending
                    self._pending = None
                    if self._active_cancel is not None:
                        self._active_cancel.set()
                self._closed = True
                self._condition.notify_all()
                thread = self._thread
        if dropped is not None:
            self._emit("cancelled", dropped, reason="runtime_closed")
        if (
            self._started
            and thread is not threading.current_thread()
        ):
            thread.join(float(timeout_seconds))
        thread_closed = not self._started or not thread.is_alive()
        backend_closed = True
        if thread_closed and not self._speaker_closed:
            if self._speaker_close is not None:
                try:
                    backend_closed = self._speaker_close() is not False
                except Exception:
                    backend_closed = False
            if backend_closed:
                self._speaker_closed = True
        return thread_closed and backend_closed

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()
                if self._pending is None and self._closed:
                    return
                item = self._pending
                self._pending = None
                cancel_event = threading.Event()
                self._active = item
                self._active_cancel = cancel_event
            self._emit("playing", item)
            try:
                self._speaker(
                    item.text,
                    item.locale,
                    cancel_event,
                )
            except Exception as error:
                if cancel_event.is_set():
                    self._emit(
                        "cancelled",
                        item,
                        reason="playback_cancelled",
                    )
                else:
                    self._emit(
                        "failed",
                        item,
                        reason=getattr(
                            error,
                            "code",
                            type(error).__name__,
                        ),
                    )
            else:
                if cancel_event.is_set():
                    self._emit(
                        "cancelled",
                        item,
                        reason="playback_cancelled",
                    )
                else:
                    self._emit("completed", item)
            finally:
                with self._condition:
                    if self._active is item:
                        self._active = None
                        self._active_cancel = None
                    self._condition.notify_all()


def _stop_process(process, timeout_seconds: float) -> None:
    try:
        process.terminate()
    except (OSError, ProcessLookupError):
        pass
    try:
        process.wait(timeout=timeout_seconds)
        return
    except subprocess.TimeoutExpired:
        pass
    except (OSError, ProcessLookupError):
        return
    try:
        process.kill()
    except (OSError, ProcessLookupError):
        pass
    try:
        process.wait(timeout=timeout_seconds)
    except (OSError, ProcessLookupError, subprocess.TimeoutExpired):
        pass


def _speech_result(stdout: object, stderr: object, returncode: object):
    if (
        isinstance(returncode, bool)
        or not isinstance(returncode, int)
        or not isinstance(stdout, str)
        or not isinstance(stderr, str)
    ):
        raise RobotSpeechRuntimeError(
            "invalid_speech_process_result",
            "EV3 speech process returned an invalid result",
        )
    if (
        len(stdout.encode("utf-8")) > MAX_SSH_OUTPUT_BYTES
        or len(stderr.encode("utf-8")) > MAX_SSH_OUTPUT_BYTES
    ):
        raise RobotSpeechRuntimeError(
            "speech_process_output_too_large",
            "EV3 speech process output was too large",
        )
    if returncode != 0:
        raise RobotSpeechRuntimeError(
            "speech_transport_failed",
            "EV3 speech process failed with status {}".format(returncode),
        )
    try:
        result = json.loads(stdout)
    except (TypeError, ValueError):
        raise RobotSpeechRuntimeError(
            "invalid_speech_response",
            "EV3 speech process returned invalid JSON",
        ) from None
    if not isinstance(result, dict) or result.get("status") != "completed":
        raise RobotSpeechRuntimeError(
            "incomplete_speech_response",
            "EV3 speech did not complete",
        )
    return result


def ev3_ssh_speaker(
    transport,
    *,
    popen_factory: Callable[..., object] = subprocess.Popen,
    monotonic: Callable[[], float] = time.monotonic,
    poll_seconds: float = DEFAULT_PROCESS_POLL_SECONDS,
    stop_timeout_seconds: float = DEFAULT_PROCESS_STOP_SECONDS,
) -> Callable[
    [str, str, threading.Event],
    object,
]:
    """Adapt fixed-command EV3 TTS, terminating its SSH process on cancel."""

    if not callable(getattr(transport, "speak", None)):
        raise ValueError("EV3 speech transport is invalid")
    if not callable(popen_factory) or not callable(monotonic):
        raise ValueError("EV3 speech process dependency is invalid")
    for name, value in (
        ("poll interval", poll_seconds),
        ("stop timeout", stop_timeout_seconds),
    ):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not 0.001 <= float(value) <= 5.0
        ):
            raise ValueError("EV3 speech {} is invalid".format(name))

    argv_builder = getattr(transport, "_argv", None)
    process_timeout = getattr(transport, "_speech_timeout_seconds", None)
    use_cancellable_process = callable(argv_builder)
    if use_cancellable_process and (
        isinstance(process_timeout, bool)
        or not isinstance(process_timeout, (int, float))
        or not 0.1 <= float(process_timeout) <= 120.0
    ):
        raise ValueError("EV3 speech transport timeout is invalid")

    def speak(
        text: str,
        locale: str,
        cancel_event: threading.Event,
    ):
        try:
            voice = VOICE_BY_LOCALE[locale]
        except KeyError:
            raise RobotSpeechRuntimeError(
                "unsupported_speech_locale",
                "Speech locale is unsupported",
            ) from None
        if cancel_event.is_set():
            return None
        if not use_cancellable_process:
            # Compatibility for injected/non-SSH transports. The production
            # EV3SSHTransport path below owns a real terminable Popen.
            return transport.speak(text, voice=voice)

        argv = argv_builder(
            [
                "python3",
                REMOTE_ROBOT_CLI,
                "speak-stdin",
                "--voice",
                voice,
            ]
        )
        try:
            process = popen_factory(
                argv,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except OSError:
            raise RobotSpeechRuntimeError(
                "speech_transport_failed",
                "EV3 speech process could not start",
            ) from None

        deadline = monotonic() + float(process_timeout)
        pending_input = text + "\n"
        while True:
            if cancel_event.is_set():
                _stop_process(process, float(stop_timeout_seconds))
                return None
            remaining = deadline - monotonic()
            if remaining <= 0:
                _stop_process(process, float(stop_timeout_seconds))
                raise RobotSpeechRuntimeError(
                    "speech_timeout",
                    "EV3 speech process timed out",
                )
            try:
                stdout, stderr = process.communicate(
                    input=pending_input,
                    timeout=min(float(poll_seconds), remaining),
                )
                break
            except subprocess.TimeoutExpired:
                # communicate() keeps its internal input buffer after a
                # timeout; retries must not submit the text a second time.
                pending_input = None
        if cancel_event.is_set():
            _stop_process(process, float(stop_timeout_seconds))
            return None
        return _speech_result(stdout, stderr, process.returncode)

    return speak
