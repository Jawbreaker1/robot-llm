"""Best-effort asynchronous speech for motion-free robot turns."""

from __future__ import annotations

import threading
from typing import Callable, Optional

from .robot_speech_runtime import RobotSpeechRuntimeError


LOCK_POLL_SECONDS = 0.05
MAX_REQUEST_ID_CHARACTERS = 128
TURN_SPEECH_EPISODE_ID = "robot-turn-dialogue"
SUPPORTED_LOCALES = ("sv", "en")


def _validate_request_id(request_id: str) -> None:
    if (
        not isinstance(request_id, str)
        or not request_id
        or request_id != request_id.strip()
        or len(request_id) > MAX_REQUEST_ID_CHARACTERS
        or any(ord(character) < 32 for character in request_id)
    ):
        raise RobotSpeechRuntimeError(
            "invalid_speech_request",
            "Speech request identity is invalid",
        )
    try:
        request_id.encode("utf-8")
    except UnicodeEncodeError:
        raise RobotSpeechRuntimeError(
            "invalid_speech_request",
            "Speech request identity is invalid",
        ) from None


def cancellable_serialized_speaker(
    speaker: Callable[[str, str, threading.Event], object],
    lock,
    *,
    poll_seconds: float = LOCK_POLL_SECONDS,
):
    """Serialize a speaker without making cancellation wait for the lock."""

    if (
        not callable(speaker)
        or not callable(getattr(lock, "acquire", None))
        or not callable(getattr(lock, "release", None))
    ):
        raise ValueError("serialized speech dependency is invalid")
    if (
        isinstance(poll_seconds, bool)
        or not isinstance(poll_seconds, (int, float))
        or not 0.001 <= float(poll_seconds) <= 1.0
    ):
        raise ValueError("serialized speech poll interval is invalid")

    def speak(text: str, locale: str, cancel_event: threading.Event):
        if not isinstance(cancel_event, threading.Event):
            raise ValueError("serialized speech cancellation is invalid")
        acquired = False
        try:
            while not cancel_event.is_set():
                acquired = lock.acquire(timeout=float(poll_seconds))
                if acquired:
                    break
            if not acquired or cancel_event.is_set():
                return None
            return speaker(text, locale, cancel_event)
        finally:
            if acquired:
                lock.release()

    return speak


class RobotTurnSpeechSink:
    """Submit independent robot replies to one latest-pending speech worker."""

    def __init__(
        self,
        runtime_factory: Callable[..., object],
        *,
        event_sink: Optional[Callable[[object], None]] = None,
        supported_locales=SUPPORTED_LOCALES,
    ):
        if (
            not callable(runtime_factory)
            or not isinstance(supported_locales, tuple)
            or len(set(supported_locales)) != len(supported_locales)
            or any(locale not in SUPPORTED_LOCALES for locale in supported_locales)
        ):
            raise ValueError("turn speech runtime factory is invalid")
        runtime = runtime_factory(event_sink=event_sink)
        if any(
            not callable(getattr(runtime, name, None))
            for name in ("start", "offer", "cancel_episode", "close")
        ):
            raise ValueError("turn speech runtime is invalid")
        self._runtime = runtime
        self._lock = threading.Lock()
        self._closed = False
        self._supported_locales = supported_locales
        self._progress_revision = 0
        runtime.start()

    def submit(self, request_id: str, text: str, locale: str) -> bool:
        try:
            _validate_request_id(request_id)
            with self._lock:
                if self._closed or locale not in self._supported_locales:
                    return False
                self._progress_revision += 1
                self._runtime.offer(
                    episode_id=TURN_SPEECH_EPISODE_ID,
                    text=text,
                    locale=locale,
                    progress_revision=self._progress_revision,
                )
            return True
        except RobotSpeechRuntimeError:
            return False

    def close(
        self,
        *,
        drain: bool = True,
        timeout_seconds: float = 5.0,
    ) -> bool:
        with self._lock:
            self._closed = True
        try:
            return self._runtime.close(
                drain=drain,
                timeout_seconds=timeout_seconds,
            ) is True
        except Exception:
            return False


__all__ = (
    "RobotTurnSpeechSink",
    "TURN_SPEECH_EPISODE_ID",
    "cancellable_serialized_speaker",
)
