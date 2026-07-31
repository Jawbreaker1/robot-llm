"""Bounded asynchronous speech playback for one physical robot episode.

Navigation only offers text to this runtime.  A dedicated worker invokes the
injected speaker, so slow audio generation or playback never blocks the
observe/plan/act loop or motor cleanup.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Callable, Mapping, Optional

from .dashboard_contract import RESPONSE_LOCALES


MAX_SPEECH_CHARACTERS = 160
VOICE_BY_LOCALE = {
    "sv": "sv",
    "en": "en",
}


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


class RobotSpeechRuntime:
    """Latest-pending-wins speech worker with explicit cancellation."""

    def __init__(
        self,
        *,
        speaker: Callable[[str, str, threading.Event], object],
        event_sink: Optional[
            Callable[[Mapping[str, object]], None]
        ] = None,
        thread_name: str = "robot-speech",
    ):
        if not callable(speaker):
            raise ValueError("speech speaker is invalid")
        if event_sink is not None and not callable(event_sink):
            raise ValueError("speech event sink is invalid")
        if (
            not isinstance(thread_name, str)
            or not thread_name
            or len(thread_name) > 128
        ):
            raise ValueError("speech thread name is invalid")
        self._speaker = speaker
        self._event_sink = event_sink
        self._condition = threading.Condition()
        self._pending = None
        self._active = None
        self._active_cancel = None
        self._sequence = 0
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
    ) -> int:
        checked_episode = _identifier(episode_id)
        checked_text = _bounded_text(text)
        if locale not in RESPONSE_LOCALES or locale not in VOICE_BY_LOCALE:
            raise RobotSpeechRuntimeError(
                "unsupported_speech_locale",
                "Speech locale is unsupported",
            )
        dropped = None
        with self._condition:
            if not self._started or not self._accepting or self._closed:
                raise RobotSpeechRuntimeError(
                    "speech_runtime_not_accepting",
                    "Speech runtime is not accepting utterances",
                )
            self._sequence += 1
            item = SpeechItem(
                episode_id=checked_episode,
                sequence=self._sequence,
                text=checked_text,
                locale=locale,
            )
            dropped = self._pending
            self._pending = item
            self._condition.notify_all()
        if dropped is not None:
            self._emit("dropped", dropped, reason="replaced_by_newer")
        self._emit("queued", item)
        return item.sequence

    def cancel_episode(self, episode_id: str) -> None:
        checked = _identifier(episode_id)
        dropped = None
        with self._condition:
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
        return not self._started or not thread.is_alive()

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
                        reason=type(error).__name__,
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


def ev3_ssh_speaker(transport) -> Callable[
    [str, str, threading.Event],
    object,
]:
    """Adapt the existing fixed-command SSH TTS transport."""

    if not callable(getattr(transport, "speak", None)):
        raise ValueError("EV3 speech transport is invalid")

    def speak(
        text: str,
        locale: str,
        _cancel_event: threading.Event,
    ):
        try:
            voice = VOICE_BY_LOCALE[locale]
        except KeyError:
            raise RobotSpeechRuntimeError(
                "unsupported_speech_locale",
                "Speech locale is unsupported",
            ) from None
        return transport.speak(text, voice=voice)

    return speak
