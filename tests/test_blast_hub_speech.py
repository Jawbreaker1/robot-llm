from array import array
import io
import threading
import unittest
from unittest import mock
import wave

from robot_agent.blast_hub_speech import (
    BLAST_PCM_BLOCK_BYTES,
    BLAST_PCM_SAMPLE_RATE_HZ,
    BlastHubSpeaker,
    pcm16_wav_to_blast_pcm,
)
from robot_agent.host_piper_speech import HostSpeechError


def wav_bytes(samples, sample_rate):
    payload = array("h", samples).tobytes()
    output = io.BytesIO()
    with wave.open(output, "wb") as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(sample_rate)
        target.writeframes(payload)
    return output.getvalue()


class BlastHubSpeechTests(unittest.TestCase):
    def test_converts_signed_pcm_to_unsigned_little_endian(self):
        result = pcm16_wav_to_blast_pcm(
            wav_bytes((-32768, 0, 32767), BLAST_PCM_SAMPLE_RATE_HZ)
        )

        self.assertEqual(result.sample_count, 3)
        self.assertEqual(result.duration_ms, 1)
        self.assertEqual(
            result.blocks,
            (b"\x00\x00\x00\x80\xff\xff",),
        )

    def test_resamples_linearly_to_eight_kilohertz(self):
        result = pcm16_wav_to_blast_pcm(
            wav_bytes((0, 8_000, 16_000, 24_000), 16_000)
        )

        self.assertEqual(result.sample_count, 2)
        self.assertEqual(result.blocks[0], b"\x00\x80\x80\xbe")

    def test_splits_only_on_even_bounded_block_boundaries(self):
        frames = BLAST_PCM_BLOCK_BYTES // 2 + 3
        result = pcm16_wav_to_blast_pcm(
            wav_bytes((0,) * frames, BLAST_PCM_SAMPLE_RATE_HZ)
        )

        self.assertEqual([len(block) for block in result.blocks], [32_000, 6])
        self.assertTrue(all(len(block) % 2 == 0 for block in result.blocks))

    def test_speaker_waits_reported_duration_without_holding_controller(self):
        audio = wav_bytes(
            (0,) * (BLAST_PCM_BLOCK_BYTES // 2 + 1),
            BLAST_PCM_SAMPLE_RATE_HZ,
        )

        class Synthesizer:
            def synthesize(self, text, locale, cancel_event):
                self.call = text, locale, cancel_event
                return audio

        class Controller:
            def __init__(self):
                self.calls = []

            def play_pcm(self, payload, *, cancel_requested):
                self.calls.append((payload, cancel_requested))
                return {
                    "byte_count": len(payload),
                    "duration_ms": (
                        (len(payload) // 2) * 1000 + 7999
                    ) // 8000,
                }

        synthesizer = Synthesizer()
        controller = Controller()
        cancel = threading.Event()
        speaker = BlastHubSpeaker(synthesizer, controller)

        with mock.patch.object(cancel, "wait", return_value=False) as waited:
            result = speaker("Hej", "sv", cancel)

        self.assertEqual(synthesizer.call, ("Hej", "sv", cancel))
        self.assertEqual([len(item[0]) for item in controller.calls], [32_000, 2])
        self.assertTrue(all(callable(item[1]) for item in controller.calls))
        self.assertEqual(waited.call_args_list, [mock.call(2.0), mock.call(0.001)])
        self.assertEqual([item["byte_count"] for item in result], [32_000, 2])

    def test_cancel_during_started_block_prevents_later_block_upload(self):
        audio = wav_bytes(
            (0,) * (BLAST_PCM_BLOCK_BYTES // 2 + 1),
            BLAST_PCM_SAMPLE_RATE_HZ,
        )

        class Synthesizer:
            def synthesize(self, _text, _locale, _cancel_event):
                return audio

        class Controller:
            def __init__(self):
                self.blocks = []

            def play_pcm(self, payload, **_kwargs):
                self.blocks.append(payload)
                return {"duration_ms": 2_000}

        controller = Controller()
        cancel = threading.Event()
        with mock.patch.object(cancel, "wait", return_value=True):
            result = BlastHubSpeaker(Synthesizer(), controller)(
                "Två block", "sv", cancel
            )

        self.assertIsNone(result)
        self.assertEqual([len(block) for block in controller.blocks], [32_000])

    def test_speaker_does_not_play_after_synthesis_cancellation(self):
        cancel = threading.Event()
        cancel.set()

        class Synthesizer:
            def synthesize(self, _text, _locale, _cancel_event):
                return b"unused"

        class Controller:
            def play_pcm(self, *_args, **_kwargs):
                raise AssertionError("cancelled speech must not reach BLAST")

        self.assertIsNone(
            BlastHubSpeaker(Synthesizer(), Controller())("Hej", "sv", cancel)
        )

    def test_speaker_rejects_missing_started_duration(self):
        class Synthesizer:
            def synthesize(self, _text, _locale, _cancel_event):
                return wav_bytes((0,), BLAST_PCM_SAMPLE_RATE_HZ)

        class Controller:
            def play_pcm(self, _payload, **_kwargs):
                return {"started": True}

        with self.assertRaisesRegex(
            HostSpeechError,
            "invalid sampled-audio duration",
        ):
            BlastHubSpeaker(Synthesizer(), Controller())(
                "Hej", "sv", threading.Event()
            )


if __name__ == "__main__":
    unittest.main()
