"""Fail-open speech lifecycle for one BLAST navigation episode."""

from __future__ import annotations

import logging
from typing import Mapping

logger = logging.getLogger(__name__)

MAX_SPEECH_ERROR_CODE_CHARACTERS = 128
SPEECH_FAILED_FALLBACK_CODE = "speech_failed"


def _bounded_speech_error_code(value) -> str:
    if (
        isinstance(value, str)
        and value
        and value == value.strip()
        and len(value) <= MAX_SPEECH_ERROR_CODE_CHARACTERS
        and all(33 <= ord(character) <= 126 for character in value)
    ):
        return value
    return SPEECH_FAILED_FALLBACK_CODE


def _exception_speech_error_code(error: Exception) -> str:
    try:
        value = getattr(error, "code", type(error).__name__)
    except Exception:
        value = type(error).__name__
    return _bounded_speech_error_code(value)


def _exception_type_name(error: Exception) -> str:
    return _bounded_speech_error_code(type(error).__name__)


def blast_episode_cancelled(context) -> bool:
    return (
        context.stop_requested.is_set()
        or context.emergency_stop_requested.is_set()
    )


class BlastEpisodeSpeech:
    """Own optional speech without giving it navigation authority."""

    def __init__(self, *, factory, supported_locales, context) -> None:
        self._factory = factory
        self._supported_locales = supported_locales
        self._context = context
        self._runtime = None

    def _cancelled(self) -> bool:
        return blast_episode_cancelled(self._context)

    def _publish_failed(self, error: Exception, *, stage: str) -> None:
        error_code = _exception_speech_error_code(error)
        logger.warning(
            "BLAST speech failed stage=%s code=%s error_type=%s",
            stage,
            error_code,
            _exception_type_name(error),
        )
        try:
            self._context.publish({
                "speech_status": "failed",
                "speech_error_code": error_code,
            })
        except Exception:
            return

    def _publish_event(self, event) -> None:
        if not isinstance(event, Mapping):
            return
        status = event.get("speech_status")
        if isinstance(status, str):
            update = {
                "speech_status": status,
                "speech_error_code": None,
            }
            if status == "failed":
                error_code = _bounded_speech_error_code(
                    event.get("reason")
                )
                update["speech_error_code"] = error_code
                logger.warning(
                    "BLAST speech failed stage=playback code=%s",
                    error_code,
                )
            try:
                self._context.publish(update)
            except Exception:
                return

    def start(self) -> None:
        if (
            getattr(self._context.settings, "speech_enabled", False) is not True
            or self._factory is None
            or self._context.request.locale not in self._supported_locales
            or self._cancelled()
        ):
            return
        try:
            runtime = self._factory(event_sink=self._publish_event)
            if any(
                not callable(getattr(runtime, name, None))
                for name in ("start", "offer", "cancel_episode", "close")
            ):
                raise TypeError("invalid speech runtime")
            self._runtime = runtime
            runtime.start()
        except Exception as error:
            self.close()
            self._publish_failed(error, stage="start")

    def offer(self, text, *, progress_revision: int):
        if self._runtime is None or text is None:
            return None
        try:
            offer_with_admission = getattr(
                self._runtime, "offer_with_admission", None,
            )
            offer = (
                offer_with_admission
                if callable(offer_with_admission)
                else self._runtime.offer
            )
            admission = offer(
                episode_id=self._context.episode_id,
                text=text,
                locale=self._context.request.locale,
                progress_revision=progress_revision,
                cancel_requested=self._cancelled,
            )
            return admission if callable(offer_with_admission) else None
        except Exception as error:
            # A rejected utterance cannot invalidate a verified action.
            self._publish_failed(error, stage="offer")
            return None

    def await_admission(self, admission, *, cancel_requested=None) -> str:
        if admission is None:
            return "skipped"
        try:
            return admission.wait(
                cancel_requested=cancel_requested or self._cancelled,
            )
        except Exception as error:
            self._publish_failed(error, stage="admission")
            return "failed"

    def cancel(self) -> None:
        if self._runtime is None:
            return
        try:
            self._runtime.cancel_episode(self._context.episode_id)
        except Exception as error:
            logger.warning(
                "BLAST speech failed stage=cancel code=%s error_type=%s",
                _exception_speech_error_code(error),
                _exception_type_name(error),
            )
            return

    def close(self) -> bool:
        if self._runtime is None:
            return True
        self.cancel()
        try:
            closed = self._runtime.close(
                drain=False,
                timeout_seconds=1.0,
            ) is True
            if closed:
                self._runtime = None
            return closed
        except Exception as error:
            logger.warning(
                "BLAST speech failed stage=close code=%s error_type=%s",
                _exception_speech_error_code(error),
                _exception_type_name(error),
            )
            return False


__all__ = ("BlastEpisodeSpeech", "blast_episode_cancelled")
