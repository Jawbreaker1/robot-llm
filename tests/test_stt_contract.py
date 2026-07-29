import hashlib
import math
import struct
import unittest

from robot_agent.stt_contract import (
    MAX_STT_AUDIO_BYTES,
    MAX_TRANSCRIPT_CHARACTERS,
    PCM16Wav,
    ProviderTranscription,
    STTContractError,
    STT_SAMPLE_RATE_HZ,
    TranscriptionRequest,
    normalize_language_hint,
    validate_pcm16_wav,
)


def canonical_wav(duration_ms=250, sample=0):
    sample_count = STT_SAMPLE_RATE_HZ * duration_ms // 1_000
    frame = struct.pack("<h", sample)
    data = frame * sample_count
    return (
        b"RIFF"
        + struct.pack("<I", 36 + len(data))
        + b"WAVE"
        + b"fmt "
        + struct.pack(
            "<IHHIIHH",
            16,
            1,
            1,
            STT_SAMPLE_RATE_HZ,
            STT_SAMPLE_RATE_HZ * 2,
            2,
            16,
        )
        + b"data"
        + struct.pack("<I", len(data))
        + data
    )


def replace(raw, offset, value):
    result = bytearray(raw)
    result[offset : offset + len(value)] = value
    return bytes(result)


class PCM16WavContractTests(unittest.TestCase):
    def test_accepts_minimum_and_maximum_canonical_audio(self):
        for duration_ms in (250, 20_000):
            with self.subTest(duration_ms=duration_ms):
                raw = canonical_wav(duration_ms, sample=123)
                audio = validate_pcm16_wav(raw)

                self.assertIsInstance(audio, PCM16Wav)
                self.assertIs(audio.wav_bytes, raw)
                self.assertEqual(audio.duration_ms, duration_ms)
                self.assertEqual(
                    audio.sample_count,
                    STT_SAMPLE_RATE_HZ * duration_ms // 1_000,
                )
                self.assertEqual(
                    audio.sha256,
                    hashlib.sha256(raw).hexdigest(),
                )

    def test_duration_rounds_up_for_partial_millisecond(self):
        raw = canonical_wav(250) + b"\x00\x00"
        raw = replace(raw, 4, struct.pack("<I", len(raw) - 8))
        raw = replace(raw, 40, struct.pack("<I", len(raw) - 44))

        audio = validate_pcm16_wav(raw)

        self.assertEqual(audio.sample_count, 4_001)
        self.assertEqual(audio.duration_ms, 251)

    def test_rejects_non_bytes_and_size_limits(self):
        cases = (
            (bytearray(canonical_wav()), "invalid_stt_audio"),
            (b"", "invalid_stt_audio_size"),
            (b"\x00" * 43, "invalid_stt_audio_size"),
            (
                b"\x00" * (MAX_STT_AUDIO_BYTES + 1),
                "invalid_stt_audio_size",
            ),
        )
        for raw, code in cases:
            with self.subTest(code=code, size=len(raw)):
                with self.assertRaises(STTContractError) as raised:
                    validate_pcm16_wav(raw)
                self.assertEqual(raised.exception.code, code)

    def test_rejects_noncanonical_container_and_chunk_layout(self):
        valid = canonical_wav()
        cases = (
            (replace(valid, 0, b"RIFX"), "invalid_stt_wav"),
            (
                replace(valid, 4, struct.pack("<I", len(valid) - 7)),
                "invalid_stt_wav",
            ),
            (replace(valid, 8, b"AVI "), "invalid_stt_wav"),
            (replace(valid, 12, b"JUNK"), "invalid_stt_wav"),
            (
                replace(valid, 16, struct.pack("<I", 18)),
                "invalid_stt_wav",
            ),
            (replace(valid, 36, b"LIST"), "unsupported_stt_wav"),
        )
        for raw, expected_code in cases:
            with self.subTest(header=raw[:44]):
                with self.assertRaises(STTContractError) as raised:
                    validate_pcm16_wav(raw)
                self.assertEqual(
                    raised.exception.code,
                    expected_code,
                )

    def test_rejects_unsupported_audio_format(self):
        valid = canonical_wav()
        cases = {
            "float": replace(valid, 20, struct.pack("<H", 3)),
            "stereo": replace(valid, 22, struct.pack("<H", 2)),
            "sample_rate": replace(
                valid,
                24,
                struct.pack("<I", 48_000),
            ),
            "byte_rate": replace(
                valid,
                28,
                struct.pack("<I", STT_SAMPLE_RATE_HZ),
            ),
            "block_align": replace(valid, 32, struct.pack("<H", 1)),
            "sample_width": replace(valid, 34, struct.pack("<H", 8)),
        }
        for name, raw in cases.items():
            with self.subTest(name=name):
                with self.assertRaises(STTContractError) as raised:
                    validate_pcm16_wav(raw)
                self.assertEqual(
                    raised.exception.code,
                    "unsupported_stt_wav",
                )

    def test_rejects_inconsistent_or_partial_frame_data(self):
        valid = canonical_wav()
        cases = (
            replace(valid, 40, struct.pack("<I", len(valid) - 46)),
            replace(valid, 40, struct.pack("<I", len(valid) - 43)),
            valid + b"\x00",
        )
        for raw in cases:
            with self.subTest(size=len(raw)):
                with self.assertRaises(STTContractError) as raised:
                    validate_pcm16_wav(raw)
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_wav",
                )

    def test_rejects_audio_outside_duration_window(self):
        for duration_ms in (249, 20_001):
            with self.subTest(duration_ms=duration_ms):
                with self.assertRaises(STTContractError) as raised:
                    validate_pcm16_wav(canonical_wav(duration_ms))
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_duration",
                )


class LanguageAndRequestContractTests(unittest.TestCase):
    def test_normalizes_language_to_primary_tag(self):
        cases = {
            "auto": "auto",
            "sv": "sv",
            "EN": "en",
            "en-US": "en",
            "zh-Hant-TW": "zh",
        }
        for value, expected in cases.items():
            with self.subTest(value=value):
                self.assertEqual(
                    normalize_language_hint(value),
                    expected,
                )

    def test_rejects_malformed_language_hints(self):
        cases = (
            None,
            "",
            " ",
            "e",
            "engl",
            "en-",
            "-en",
            "en_US",
            " en",
            "en ",
            "sv-å",
            "sv-" + "a" * 9,
            "a" * 36,
        )
        for value in cases:
            with self.subTest(value=value):
                with self.assertRaises(STTContractError) as raised:
                    normalize_language_hint(value)
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_language",
                )

    def test_request_normalizes_language_and_requires_valid_types(self):
        audio = validate_pcm16_wav(canonical_wav())
        request = TranscriptionRequest(
            request_id="voice.turn-1",
            language_hint="SV-se",
            audio=audio,
        )
        self.assertEqual(request.language_hint, "sv")
        self.assertIs(request.audio, audio)

        for request_id in ("", " has-space", "slash/value", "å"):
            with self.subTest(request_id=request_id):
                with self.assertRaises(STTContractError) as raised:
                    TranscriptionRequest(
                        request_id=request_id,
                        language_hint="sv",
                        audio=audio,
                    )
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_identifier",
                )

        with self.assertRaises(STTContractError) as raised:
            TranscriptionRequest(
                request_id="voice-2",
                language_hint="sv",
                audio=b"not-validated",
            )
        self.assertEqual(raised.exception.code, "invalid_stt_audio")


class ProviderTranscriptionContractTests(unittest.TestCase):
    def test_accepts_bounded_multilingual_text_and_optional_metadata(self):
        result = ProviderTranscription(
            text="Vinka två gånger.\nSedan stannar du.",
            provider_id="whisper.cpp",
            model_id="ggml-small",
            detected_language="SV-se",
            provider_score=1,
        )

        self.assertEqual(result.detected_language, "sv")
        self.assertEqual(result.provider_score, 1.0)

    def test_rejects_invalid_transcript_text(self):
        cases = (
            "",
            " ",
            " leading",
            "trailing ",
            "bad\x00text",
            "\ud800",
            "bad\udffftext",
            "x" * (MAX_TRANSCRIPT_CHARACTERS + 1),
        )
        for text in cases:
            with self.subTest(length=len(text)):
                with self.assertRaises(STTContractError) as raised:
                    ProviderTranscription(
                        text=text,
                        provider_id="fixture",
                        model_id="fixture-v1",
                    )
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_transcript",
                )

    def test_rejects_invalid_provider_identity_and_score(self):
        identity_cases = (
            ("bad provider", "model"),
            ("provider", "bad/model"),
            ("provider", ""),
        )
        for provider_id, model_id in identity_cases:
            with self.subTest(
                provider_id=provider_id,
                model_id=model_id,
            ):
                with self.assertRaises(STTContractError) as raised:
                    ProviderTranscription(
                        text="Hello",
                        provider_id=provider_id,
                        model_id=model_id,
                    )
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_identifier",
                )

        for score in (True, -0.01, 1.01, math.inf, math.nan, "0.9"):
            with self.subTest(score=score):
                with self.assertRaises(STTContractError) as raised:
                    ProviderTranscription(
                        text="Hello",
                        provider_id="fixture",
                        model_id="fixture-v1",
                        provider_score=score,
                    )
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_provider_score",
                )


if __name__ == "__main__":
    unittest.main()
