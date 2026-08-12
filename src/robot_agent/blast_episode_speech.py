"""Fail-open speech lifecycle for one BLAST navigation episode."""

from __future__ import annotations

from typing import Mapping


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

    def _publish_failed(self) -> None:
        try:
            self._context.publish({"speech_status": "failed"})
        except Exception:
            return

    def _publish_event(self, event) -> None:
        if not isinstance(event, Mapping):
            return
        status = event.get("speech_status")
        if isinstance(status, str):
            try:
                self._context.publish({"speech_status": status})
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
        except Exception:
            self.close()
            self._publish_failed()

    def offer(self, text, *, progress_revision: int) -> None:
        if self._runtime is None or text is None:
            return
        try:
            self._runtime.offer(
                episode_id=self._context.episode_id,
                text=text,
                locale=self._context.request.locale,
                progress_revision=progress_revision,
                cancel_requested=self._cancelled,
            )
        except Exception:
            # A rejected utterance cannot invalidate a verified action.
            self._publish_failed()

    def cancel(self) -> None:
        if self._runtime is None:
            return
        try:
            self._runtime.cancel_episode(self._context.episode_id)
        except Exception:
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
        except Exception:
            return False


__all__ = ("BlastEpisodeSpeech", "blast_episode_cancelled")
