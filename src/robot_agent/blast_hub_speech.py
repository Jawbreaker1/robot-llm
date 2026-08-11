"""Bounded host synthesis played through BLAST's own hub speaker."""

from __future__ import annotations

from array import array
from dataclasses import dataclass
import io
import sys
import threading
import wave

from .host_piper_speech import HostSpeechError, validate_pcm16_mono_wav


BLAST_PCM_SAMPLE_RATE_HZ = 8_000
BLAST_PCM_BLOCK_BYTES = 32_000


@dataclass(frozen=True)
class BlastPCM:
    blocks: tuple[bytes, ...]
    duration_ms: int
    sample_count: int


def pcm16_wav_to_blast_pcm(raw: bytes) -> BlastPCM:
    """Convert bounded mono PCM16 WAV to 8 kHz unsigned little-endian PCM."""

    metadata = validate_pcm16_mono_wav(raw)
    try:
        with wave.open(io.BytesIO(raw), "rb") as source:
            payload = source.readframes(metadata.frames)
    except (EOFError, wave.Error):
        raise HostSpeechError(
            "invalid_tts_wav",
            "TTS output is not WAV",
        ) from None

    signed = array("h")
    signed.frombytes(payload)
    if sys.byteorder != "little":
        signed.byteswap()
    output_count = (
        metadata.frames * BLAST_PCM_SAMPLE_RATE_HZ
        + metadata.sample_rate_hz
        - 1
    ) // metadata.sample_rate_hz
    if output_count <= 0:
        raise HostSpeechError(
            "invalid_tts_wav",
            "TTS output contains no audio",
        )

    converted = bytearray(output_count * 2)
    target_rate = BLAST_PCM_SAMPLE_RATE_HZ
    source_rate = metadata.sample_rate_hz
    last_index = metadata.frames - 1
    for output_index in range(output_count):
        position = output_index * source_rate
        left_index, fraction = divmod(position, target_rate)
        if left_index >= last_index:
            value = int(signed[last_index])
        else:
            left = int(signed[left_index])
            right = int(signed[left_index + 1])
            value = (
                left * (target_rate - fraction) + right * fraction
            ) // target_rate
        unsigned = max(0, min(65_535, value + 32_768))
        offset = output_index * 2
        converted[offset] = unsigned & 0xFF
        converted[offset + 1] = unsigned >> 8

    blocks = tuple(
        bytes(converted[offset : offset + BLAST_PCM_BLOCK_BYTES])
        for offset in range(0, len(converted), BLAST_PCM_BLOCK_BYTES)
    )
    return BlastPCM(
        blocks=blocks,
        duration_ms=(output_count * 1_000 + target_rate - 1) // target_rate,
        sample_count=output_count,
    )


class BlastHubSpeaker:
    """Synthesize text on the host and play bounded blocks in BLAST's hub."""

    def __init__(self, synthesizer, controller):
        if not callable(getattr(synthesizer, "synthesize", None)) or not callable(
            getattr(controller, "play_pcm", None)
        ):
            raise ValueError("BLAST speech dependencies are invalid")
        self.synthesizer = synthesizer
        self.controller = controller

    def __call__(self, text: str, locale: str, cancel_event: threading.Event):
        if not isinstance(cancel_event, threading.Event):
            raise HostSpeechError(
                "invalid_speech_cancel",
                "Speech cancellation is invalid",
            )
        raw = self.synthesizer.synthesize(text, locale, cancel_event)
        if raw is None or cancel_event.is_set():
            return None
        pcm = pcm16_wav_to_blast_pcm(raw)
        receipts = []
        for block in pcm.blocks:
            if cancel_event.is_set():
                return None
            receipt = self.controller.play_pcm(
                block,
                cancel_requested=cancel_event.is_set,
            )
            duration_ms = (
                receipt.get("duration_ms")
                if isinstance(receipt, dict)
                else None
            )
            if (
                isinstance(duration_ms, bool)
                or not isinstance(duration_ms, int)
                or not 1 <= duration_ms <= 2_000
            ):
                raise HostSpeechError(
                    "invalid_blast_pcm_receipt",
                    "BLAST returned an invalid sampled-audio duration",
                )
            receipts.append(receipt)
            # The hub has already returned to its BLE command loop. Waiting
            # here keeps speech blocks sequential without holding navigation.
            if cancel_event.wait(duration_ms / 1_000):
                return None
        return tuple(receipts)


__all__ = (
    "BLAST_PCM_BLOCK_BYTES",
    "BLAST_PCM_SAMPLE_RATE_HZ",
    "BlastHubSpeaker",
    "BlastPCM",
    "pcm16_wav_to_blast_pcm",
)
