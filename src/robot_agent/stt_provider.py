"""Provider boundary for local speech-to-text engines."""

from __future__ import annotations

from typing import Protocol

from .stt_contract import ProviderTranscription, TranscriptionRequest


class STTProviderError(RuntimeError):
    """Base class for expected, safely reportable provider failures."""


class STTProviderUnavailableError(STTProviderError):
    """The configured local provider is not available."""


class STTProviderTimeoutError(STTProviderError):
    """The provider exceeded its absolute request deadline."""


class STTProviderProtocolError(STTProviderError):
    """The provider returned malformed or excessive output."""


class SpeechTranscriber(Protocol):
    provider_id: str
    model_id: str

    def transcribe(
        self,
        request: TranscriptionRequest,
    ) -> ProviderTranscription:
        ...
