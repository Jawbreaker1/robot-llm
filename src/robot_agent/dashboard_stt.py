"""Thin dashboard composition layer for provider-neutral speech input."""

from __future__ import annotations

import threading
import time
from typing import Callable, Mapping, Optional

from .stt_contract import (
    MAX_STT_AUDIO_BYTES,
    MAX_STT_DURATION_MS,
    STT_CHANNELS,
    STT_SAMPLE_RATE_HZ,
    STTContractError,
    TranscriptionRequest,
    validate_pcm16_wav,
)
from .stt_provider import (
    STTProviderProtocolError,
    STTProviderTimeoutError,
    STTProviderUnavailableError,
)
from .stt_runtime import STTRuntime, STTRuntimeError


STT_RUNTIME_SCHEMA = "speech-to-text-runtime/v1"


class DashboardSTTError(RuntimeError):
    """Typed speech failure safe to expose through the dashboard API."""

    def __init__(self, status: int, code: str, message: str):
        self.status = status
        self.code = code
        super().__init__(message)


class DashboardSTT:
    """Own speech availability, runtime state, and the independent worker."""

    def __init__(
        self,
        provider=None,
        *,
        event_sink: Optional[
            Callable[[str, Mapping[str, object]], None]
        ] = None,
    ):
        self._provider = provider
        self._runtime = None
        self._lock = threading.RLock()
        self._state = {
            "schema": STT_RUNTIME_SCHEMA,
            "state": "disabled" if provider is None else "configured",
            "provider_id": (
                getattr(provider, "provider_id", None)
                if provider is not None
                else None
            ),
            "model_id": (
                getattr(provider, "model_id", None)
                if provider is not None
                else None
            ),
            "last_probe_at_unix_ms": None,
            "error_code": None,
        }
        if provider is not None:
            try:
                self._runtime = STTRuntime(
                    provider,
                    event_sink=event_sink,
                )
            except STTRuntimeError as error:
                raise DashboardSTTError(
                    error.status,
                    error.code,
                    str(error),
                ) from None

    @property
    def enabled(self) -> bool:
        return self._runtime is not None

    def capability(self) -> Mapping[str, object]:
        return {
            "enabled": self.enabled,
            "input_format": "audio/wav",
            "encoding": "pcm_s16le",
            "sample_rate_hz": STT_SAMPLE_RATE_HZ,
            "channels": STT_CHANNELS,
            "max_audio_bytes": MAX_STT_AUDIO_BYTES,
            "max_duration_ms": MAX_STT_DURATION_MS,
            "audio_persisted": False,
            "transport": "bounded_utterance",
            "cancellation": self.enabled,
            "terminal_ttl_ms": (
                self._runtime.terminal_ttl_ms
                if self._runtime is not None
                else None
            ),
        }

    def runtime_view(self) -> Mapping[str, object]:
        with self._lock:
            return dict(self._state)

    def submit(
        self,
        request_id: str,
        language_hint: str,
        wav_bytes: bytes,
    ) -> Mapping[str, object]:
        if self._runtime is None:
            raise DashboardSTTError(
                503,
                "stt_unavailable",
                "Speech transcription is not configured",
            )
        try:
            audio = validate_pcm16_wav(wav_bytes)
            request = TranscriptionRequest(
                request_id=request_id,
                language_hint=language_hint,
                audio=audio,
            )
            return self._runtime.submit(request)
        except STTContractError as error:
            raise DashboardSTTError(
                400,
                error.code,
                str(error),
            ) from None
        except STTRuntimeError as error:
            raise DashboardSTTError(
                error.status,
                error.code,
                str(error),
            ) from None

    def get(self, transcription_id: str) -> Mapping[str, object]:
        if self._runtime is None:
            raise DashboardSTTError(
                503,
                "stt_unavailable",
                "Speech transcription is not configured",
            )
        try:
            return self._runtime.get(transcription_id)
        except STTRuntimeError as error:
            raise DashboardSTTError(
                error.status,
                error.code,
                str(error),
            ) from None

    def cancel(self, transcription_id: str) -> Mapping[str, object]:
        if self._runtime is None:
            raise DashboardSTTError(
                503,
                "stt_unavailable",
                "Speech transcription is not configured",
            )
        try:
            return self._runtime.cancel(transcription_id)
        except STTRuntimeError as error:
            raise DashboardSTTError(
                error.status,
                error.code,
                str(error),
            ) from None

    def cancel_request(self, request_id: str) -> Mapping[str, object]:
        if self._runtime is None:
            raise DashboardSTTError(
                503,
                "stt_unavailable",
                "Speech transcription is not configured",
            )
        try:
            return self._runtime.cancel_request(request_id)
        except STTRuntimeError as error:
            raise DashboardSTTError(
                error.status,
                error.code,
                str(error),
            ) from None

    def probe(self) -> Mapping[str, object]:
        if self._provider is None:
            raise DashboardSTTError(
                503,
                "stt_unavailable",
                "Speech transcription is not configured",
            )
        probe = getattr(self._provider, "probe", None)
        if not callable(probe):
            raise DashboardSTTError(
                503,
                "stt_probe_unavailable",
                "Speech provider has no readiness probe",
            )
        state = "online"
        error_code = None
        try:
            probe()
        except STTProviderTimeoutError:
            state = "offline"
            error_code = "stt_provider_timeout"
        except STTProviderUnavailableError:
            state = "offline"
            error_code = "stt_provider_unavailable"
        except STTProviderProtocolError:
            state = "fault"
            error_code = "stt_provider_protocol"
        except Exception:
            state = "fault"
            error_code = "stt_provider_failed"
        with self._lock:
            self._state = {
                **self._state,
                "state": state,
                "last_probe_at_unix_ms": (
                    time.time_ns() // 1_000_000
                ),
                "error_code": error_code,
            }
            view = dict(self._state)
        if error_code is not None:
            raise DashboardSTTError(
                503,
                error_code,
                "Speech provider readiness check failed",
            )
        return view

    def shutdown(self, timeout_seconds: float = 1.0):
        if self._runtime is None:
            return {
                "worker_alive": False,
                "event_worker_alive": False,
                "timed_out": False,
                "queued_remaining": 0,
                "provider_work_pending": 0,
                "event_dropped_total": 0,
            }
        try:
            return self._runtime.shutdown(timeout_seconds)
        except STTRuntimeError as error:
            raise DashboardSTTError(
                error.status,
                error.code,
                str(error),
            ) from None
