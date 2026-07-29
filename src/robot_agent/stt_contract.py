"""Provider-neutral contracts for bounded local speech transcription.

Audio is accepted only as a canonical mono PCM16 WAV segment.  The bytes are
ephemeral input and are never included in public views, logs, or agent
instructions.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import struct
from typing import Optional


TRANSCRIPTION_SCHEMA = "speech-transcription/v1"
STT_SAMPLE_RATE_HZ = 16_000
STT_CHANNELS = 1
STT_SAMPLE_WIDTH_BYTES = 2
MIN_STT_DURATION_MS = 250
MAX_STT_DURATION_MS = 20_000
MAX_STT_AUDIO_BYTES = 768 * 1024
MAX_TRANSCRIPT_CHARACTERS = 4_000
MAX_LANGUAGE_TAG_CHARACTERS = 35


class STTContractError(ValueError):
    """A safely reportable speech contract failure."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _identifier(name: str, value: str, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or not value.isascii()
        or not all(
            character.isalnum() or character in "-_."
            for character in value
        )
    ):
        raise STTContractError(
            "invalid_stt_identifier",
            "{} is invalid".format(name),
        )
    return value


def normalize_language_hint(value: str) -> str:
    """Validate a small BCP-47-shaped hint and return Whisper's primary tag."""

    if not isinstance(value, str):
        raise STTContractError(
            "invalid_stt_language",
            "Speech language hint is invalid",
        )
    normalized = value.strip().lower()
    if value != value.strip():
        raise STTContractError(
            "invalid_stt_language",
            "Speech language hint is invalid",
        )
    if normalized == "auto":
        return normalized
    if (
        not normalized
        or normalized != value.lower()
        or len(normalized) > MAX_LANGUAGE_TAG_CHARACTERS
        or not normalized.isascii()
    ):
        raise STTContractError(
            "invalid_stt_language",
            "Speech language hint is invalid",
        )
    parts = normalized.split("-")
    if (
        not 2 <= len(parts[0]) <= 3
        or not parts[0].isalpha()
        or any(
            not 1 <= len(part) <= 8
            or not part.isalnum()
            for part in parts[1:]
        )
    ):
        raise STTContractError(
            "invalid_stt_language",
            "Speech language hint is invalid",
        )
    return parts[0]


def _read_u16(raw: bytes, offset: int) -> int:
    return struct.unpack_from("<H", raw, offset)[0]


def _read_u32(raw: bytes, offset: int) -> int:
    return struct.unpack_from("<I", raw, offset)[0]


@dataclass(frozen=True)
class PCM16Wav:
    """Validated, canonical microphone audio retained only while queued."""

    wav_bytes: bytes
    duration_ms: int
    sample_count: int
    sha256: str


def validate_pcm16_wav(raw: bytes) -> PCM16Wav:
    """Validate the exact WAV shape produced by the browser capture module."""

    if not isinstance(raw, bytes):
        raise STTContractError(
            "invalid_stt_audio",
            "Speech audio must be bytes",
        )
    if not 44 <= len(raw) <= MAX_STT_AUDIO_BYTES:
        raise STTContractError(
            "invalid_stt_audio_size",
            "Speech audio size is outside the accepted range",
        )
    if (
        raw[:4] != b"RIFF"
        or raw[8:12] != b"WAVE"
        or _read_u32(raw, 4) != len(raw) - 8
        or raw[12:16] != b"fmt "
        or _read_u32(raw, 16) != 16
    ):
        raise STTContractError(
            "invalid_stt_wav",
            "Speech audio is not canonical PCM WAV",
        )
    audio_format = _read_u16(raw, 20)
    channels = _read_u16(raw, 22)
    sample_rate = _read_u32(raw, 24)
    byte_rate = _read_u32(raw, 28)
    block_align = _read_u16(raw, 32)
    bits_per_sample = _read_u16(raw, 34)
    if (
        audio_format != 1
        or channels != STT_CHANNELS
        or sample_rate != STT_SAMPLE_RATE_HZ
        or byte_rate
        != STT_SAMPLE_RATE_HZ
        * STT_CHANNELS
        * STT_SAMPLE_WIDTH_BYTES
        or block_align != STT_CHANNELS * STT_SAMPLE_WIDTH_BYTES
        or bits_per_sample != STT_SAMPLE_WIDTH_BYTES * 8
        or raw[36:40] != b"data"
    ):
        raise STTContractError(
            "unsupported_stt_wav",
            "Speech audio must be mono 16 kHz PCM16 WAV",
        )
    data_size = _read_u32(raw, 40)
    if (
        data_size != len(raw) - 44
        or data_size % STT_SAMPLE_WIDTH_BYTES
    ):
        raise STTContractError(
            "invalid_stt_wav",
            "Speech WAV frame data is inconsistent",
        )
    sample_count = data_size // STT_SAMPLE_WIDTH_BYTES
    duration_ms = (
        sample_count * 1_000 + STT_SAMPLE_RATE_HZ - 1
    ) // STT_SAMPLE_RATE_HZ
    if not MIN_STT_DURATION_MS <= duration_ms <= MAX_STT_DURATION_MS:
        raise STTContractError(
            "invalid_stt_duration",
            "Speech duration is outside the accepted range",
        )
    return PCM16Wav(
        wav_bytes=raw,
        duration_ms=duration_ms,
        sample_count=sample_count,
        sha256=hashlib.sha256(raw).hexdigest(),
    )


@dataclass(frozen=True)
class TranscriptionRequest:
    request_id: str
    language_hint: str
    audio: PCM16Wav

    def __post_init__(self) -> None:
        _identifier("request_id", self.request_id)
        object.__setattr__(
            self,
            "language_hint",
            normalize_language_hint(self.language_hint),
        )
        if not isinstance(self.audio, PCM16Wav):
            raise STTContractError(
                "invalid_stt_audio",
                "Validated speech audio is required",
            )


def _optional_language(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    return normalize_language_hint(value)


@dataclass(frozen=True)
class ProviderTranscription:
    """Untrusted provider output after strict structural validation."""

    text: str
    provider_id: str
    model_id: str
    detected_language: Optional[str] = None
    provider_score: Optional[float] = None

    def __post_init__(self) -> None:
        utf8_safe = False
        if isinstance(self.text, str):
            try:
                self.text.encode("utf-8")
                utf8_safe = True
            except UnicodeEncodeError:
                pass
        if (
            not isinstance(self.text, str)
            or not utf8_safe
            or not self.text.strip()
            or self.text != self.text.strip()
            or len(self.text) > MAX_TRANSCRIPT_CHARACTERS
            or any(
                ord(character) < 32
                and character not in "\n\r\t"
                for character in self.text
            )
        ):
            raise STTContractError(
                "invalid_stt_transcript",
                "Speech provider returned invalid transcript text",
            )
        _identifier("provider_id", self.provider_id)
        _identifier("model_id", self.model_id, 200)
        object.__setattr__(
            self,
            "detected_language",
            _optional_language(self.detected_language),
        )
        score = self.provider_score
        if score is not None and (
            isinstance(score, bool)
            or not isinstance(score, (int, float))
            or not 0.0 <= float(score) <= 1.0
        ):
            raise STTContractError(
                "invalid_stt_provider_score",
                "Speech provider score is invalid",
            )
        if score is not None:
            object.__setattr__(self, "provider_score", float(score))
