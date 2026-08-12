"""Bounded host synthesis played through BLAST's own hub speaker."""

from __future__ import annotations

from array import array
from dataclasses import dataclass
import io
import sys
import threading
import wave

from .blast_ble_runtime import blast_adpcm_duration_ms
from .host_piper_speech import (
    HostSpeechError,
    PiperSpeechProfile,
    validate_pcm16_mono_wav,
)


BLAST_ADPCM_SAMPLE_RATE_HZ = 16_000
BLAST_ADPCM_MAX_SAMPLES = 128_000
BLAST_ADPCM_HEADER_BYTES = 7
BLAST_ADPCM_MAX_BYTES = (
    BLAST_ADPCM_HEADER_BYTES + BLAST_ADPCM_MAX_SAMPLES // 2
)
BLAST_PIPER_PROFILE = PiperSpeechProfile(
    voices=(("sv", "lisa-bright"),),
    speed=0.98,
)

_PCM16_MAX = 32_767

_IMA_INDEX_CHANGES = (-1, -1, -1, -1, 2, 4, 6, 8)
_IMA_STEP_SIZES = (
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31,
    34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371,
    408, 449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166,
    1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024,
    3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845,
    8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500,
    20350, 22385, 24623, 27086, 29794, 32767,
)


@dataclass(frozen=True)
class BlastADPCM:
    payload: bytes
    duration_ms: int
    sample_count: int


def _soft_compand_sample(sample: int) -> int:
    """Raise speech loudness without adding a hard clipping plateau."""

    magnitude = abs(int(sample))
    compressed = (
        magnitude * (_PCM16_MAX + _PCM16_MAX)
    ) // (magnitude + _PCM16_MAX)
    return -compressed if sample < 0 else compressed


def _encode_ima_adpcm_stream(samples, initial_step_index=0):
    """Encode one self-contained low-nibble-first IMA ADPCM utterance."""

    sample_count = len(samples)
    if not 1 <= sample_count <= BLAST_ADPCM_MAX_SAMPLES:
        raise ValueError("ADPCM stream must contain 1..128000 samples")
    if not isinstance(initial_step_index, int) or isinstance(
        initial_step_index, bool
    ) or not 0 <= initial_step_index <= 88:
        raise ValueError("ADPCM step index must be from 0 to 88")

    predictor = int(samples[0])
    step_index = initial_step_index
    output = bytearray(
        BLAST_ADPCM_HEADER_BYTES + sample_count // 2
    )
    output[0] = predictor & 0xFF
    output[1] = predictor >> 8 & 0xFF
    output[2] = step_index
    output[3] = sample_count & 0xFF
    output[4] = sample_count >> 8 & 0xFF
    output[5] = sample_count >> 16 & 0xFF
    output[6] = sample_count >> 24

    for code_index, sample in enumerate(samples[1:]):
        step = _IMA_STEP_SIZES[step_index]
        difference = int(sample) - predictor
        code = 0
        if difference < 0:
            code = 8
            difference = -difference

        decoded_difference = step >> 3
        if difference >= step:
            code |= 4
            difference -= step
            decoded_difference += step
        half_step = step >> 1
        if difference >= half_step:
            code |= 2
            difference -= half_step
            decoded_difference += half_step
        quarter_step = step >> 2
        if difference >= quarter_step:
            code |= 1
            decoded_difference += quarter_step

        if code & 8:
            predictor -= decoded_difference
        else:
            predictor += decoded_difference
        predictor = max(-32_768, min(32_767, predictor))
        step_index = max(
            0,
            min(88, step_index + _IMA_INDEX_CHANGES[code & 7]),
        )

        byte_index = BLAST_ADPCM_HEADER_BYTES + code_index // 2
        if code_index % 2:
            output[byte_index] |= code << 4
        else:
            output[byte_index] = code

    return bytes(output), step_index


def pcm16_wav_to_blast_adpcm(
    raw: bytes,
    *,
    loudness_compensation: bool = True,
) -> BlastADPCM:
    """Convert bounded mono PCM16 WAV to one 16 kHz IMA ADPCM utterance."""

    if not isinstance(loudness_compensation, bool):
        raise ValueError("BLAST loudness compensation must be boolean")

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
        metadata.frames * BLAST_ADPCM_SAMPLE_RATE_HZ
        + metadata.sample_rate_hz
        - 1
    ) // metadata.sample_rate_hz
    if output_count <= 0:
        raise HostSpeechError(
            "invalid_tts_wav",
            "TTS output contains no audio",
        )
    if output_count > BLAST_ADPCM_MAX_SAMPLES:
        raise HostSpeechError(
            "tts_audio_too_long",
            "BLAST speech exceeds eight seconds",
        )

    converted = array("h")
    target_rate = BLAST_ADPCM_SAMPLE_RATE_HZ
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
        value = max(-32_768, min(32_767, value))
        if loudness_compensation:
            value = _soft_compand_sample(value)
        converted.append(value)

    encoded, _ = _encode_ima_adpcm_stream(converted)
    return BlastADPCM(
        payload=encoded,
        duration_ms=blast_adpcm_duration_ms(output_count),
        sample_count=output_count,
    )


class BlastHubSpeaker:
    """Preload one bounded utterance, then play it through BLAST's hub."""

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
        audio = pcm16_wav_to_blast_adpcm(raw)
        if cancel_event.is_set():
            return None
        receipt = self.controller.play_pcm(
            audio.payload,
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
            or not 1 <= duration_ms <= 8_000
            or duration_ms != audio.duration_ms
        ):
            raise HostSpeechError(
                "invalid_blast_pcm_receipt",
                "BLAST returned an invalid sampled-audio duration",
            )
        # The hub has already returned to its BLE command loop. This worker
        # waits only so episode cancellation and speech completion stay exact;
        # navigation continues on the monitor's separate owner thread.
        if cancel_event.wait(duration_ms / 1_000):
            return None
        return (receipt,)


__all__ = (
    "BLAST_ADPCM_HEADER_BYTES",
    "BLAST_ADPCM_MAX_BYTES",
    "BLAST_ADPCM_MAX_SAMPLES",
    "BLAST_ADPCM_SAMPLE_RATE_HZ",
    "BLAST_PIPER_PROFILE",
    "BlastADPCM",
    "BlastHubSpeaker",
    "pcm16_wav_to_blast_adpcm",
)
