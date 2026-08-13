from array import array
import ast
import gc
import io
from pathlib import Path
import threading
import unittest
from unittest import mock
import wave

from robot_agent.blast_ble_runtime import (
    _adpcm_sample_count,
    _fletcher16,
    blast_adpcm_duration_ms,
)
from robot_agent.blast_hub_speech import (
    BLAST_ADPCM_HEADER_BYTES,
    BLAST_ADPCM_MAX_BYTES,
    BLAST_ADPCM_MAX_SAMPLES,
    BLAST_ADPCM_SAMPLE_RATE_HZ,
    BLAST_PIPER_PROFILE,
    BlastHubSpeaker,
    _encode_ima_adpcm_stream,
    _soft_compand_sample,
    pcm16_wav_to_blast_adpcm,
)
from robot_agent.host_piper_speech import HostSpeechError, PiperSpeechProfile


def wav_bytes(samples, sample_rate):
    payload = array("h", samples).tobytes()
    output = io.BytesIO()
    with wave.open(output, "wb") as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(sample_rate)
        target.writeframes(payload)
    return output.getvalue()


def load_hub_audio_namespace():
    """Compile the real pure hub protocol functions without hardware startup."""

    runtime_path = (
        Path(__file__).resolve().parents[1]
        / "hub_programs"
        / "blast_01"
        / "runtime.py"
    )
    tree = ast.parse(runtime_path.read_text(encoding="utf-8"))
    constant_names = {
        "SAMPLED_AUDIO_HEADER_BYTES",
        "SAMPLED_AUDIO_MAX_BYTES",
        "SAMPLED_AUDIO_MAX_SAMPLES",
        "SAMPLED_AUDIO_SAMPLE_RATE_HZ",
        "SAMPLED_AUDIO_ENCODING",
        "SAMPLED_AUDIO_DMA_CHUNK_SAMPLES",
        "SAMPLED_AUDIO_DMA_CHUNK_DURATION_MS",
    }
    function_names = {
        "validate_pcm_format",
        "begin_pcm",
        "start_pcm",
    }
    selected = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id in constant_names
            for target in node.targets
        ):
            selected.append(node)
        elif (
            isinstance(node, ast.FunctionDef)
            and node.name in function_names
        ):
            selected.append(node)
    namespace = {"gc": gc}
    exec(compile(ast.Module(selected, []), str(runtime_path), "exec"), namespace)
    return namespace


class BlastHubSpeechTests(unittest.TestCase):
    def test_blast_has_its_own_swedish_piper_voice(self):
        self.assertIsInstance(BLAST_PIPER_PROFILE, PiperSpeechProfile)
        self.assertEqual(BLAST_PIPER_PROFILE.model, "piper-sv")
        self.assertEqual(BLAST_PIPER_PROFILE.voices, (("sv", "lisa-bright"),))
        self.assertEqual(BLAST_PIPER_PROFILE.speed, 0.98)
        self.assertEqual(PiperSpeechProfile().voices, (("sv", "nst-deep"),))
        self.assertEqual(PiperSpeechProfile().speed, 1.0)

    def test_soft_companding_is_bounded_symmetric_and_monotonic(self):
        magnitudes = (
            0,
            1,
            100,
            1_000,
            8_000,
            16_000,
            24_000,
            30_000,
            32_766,
            32_767,
        )
        positive = tuple(_soft_compand_sample(value) for value in magnitudes)

        self.assertEqual(positive, tuple(sorted(positive)))
        self.assertEqual(positive[0], 0)
        self.assertEqual(positive[-1], 32_767)
        self.assertTrue(all(
            original <= converted <= 32_767
            for original, converted in zip(magnitudes, positive)
        ))
        self.assertEqual(
            tuple(_soft_compand_sample(-value) for value in magnitudes),
            tuple(-value for value in positive),
        )
        self.assertEqual(_soft_compand_sample(-32_768), -32_767)
        self.assertEqual(positive.count(32_767), 1)

    def test_default_companding_raises_rms_without_changing_stream_shape(self):
        samples = (-12_000, -6_000, -1_500, 0, 1_500, 6_000, 12_000)
        audio = wav_bytes(samples, BLAST_ADPCM_SAMPLE_RATE_HZ)

        plain = pcm16_wav_to_blast_adpcm(
            audio,
            loudness_compensation=False,
        )
        louder = pcm16_wav_to_blast_adpcm(audio)
        converted = tuple(_soft_compand_sample(value) for value in samples)

        self.assertGreater(
            sum(value * value for value in converted),
            sum(value * value for value in samples),
        )
        self.assertEqual(louder.sample_count, plain.sample_count)
        self.assertEqual(louder.duration_ms, plain.duration_ms)
        self.assertEqual(len(louder.payload), len(plain.payload))
        self.assertNotEqual(louder.payload, plain.payload)

    def test_companding_receives_the_resampled_sixteen_kilohertz_values(self):
        resampled = []

        def capture(sample):
            resampled.append(sample)
            return sample

        with mock.patch(
            "robot_agent.blast_hub_speech._soft_compand_sample",
            side_effect=capture,
        ):
            result = pcm16_wav_to_blast_adpcm(
                wav_bytes((0, 8_000, 16_000, 24_000), 8_000)
            )

        self.assertEqual(
            resampled,
            [0, 4_000, 8_000, 12_000, 16_000, 20_000, 24_000, 24_000],
        )
        self.assertEqual(result.sample_count, 8)

    def test_loudness_compensation_seam_rejects_non_boolean_values(self):
        with self.assertRaisesRegex(ValueError, "must be boolean"):
            pcm16_wav_to_blast_adpcm(
                wav_bytes((0,), BLAST_ADPCM_SAMPLE_RATE_HZ),
                loudness_compensation=1,
            )

    def test_duration_includes_the_final_dma_half_buffer(self):
        self.assertEqual(blast_adpcm_duration_ms(1), 16)
        self.assertEqual(blast_adpcm_duration_ms(256), 16)
        self.assertEqual(blast_adpcm_duration_ms(257), 32)
        self.assertEqual(blast_adpcm_duration_ms(82_106), 5_136)
        self.assertEqual(blast_adpcm_duration_ms(128_000), 8_000)
        for invalid in (True, 0, 128_001):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                blast_adpcm_duration_ms(invalid)

    def test_golden_vector_has_exact_v5_stream_header_and_codes(self):
        encoded, final_index = _encode_ima_adpcm_stream(
            array("h", (0, 7, -4)),
            0,
        )

        self.assertEqual(
            encoded,
            bytes.fromhex("00 00 00 03 00 00 00 d4"),
        )
        self.assertEqual(final_index, 6)
        self.assertEqual(_adpcm_sample_count(encoded), 3)

    def test_stream_header_carries_full_u32_sample_count(self):
        encoded, _ = _encode_ima_adpcm_stream(array("h", (0,)) * 70_000)

        self.assertEqual(encoded[3:7], bytes.fromhex("70 11 01 00"))
        self.assertEqual(len(encoded), BLAST_ADPCM_HEADER_BYTES + 35_000)
        self.assertEqual(_adpcm_sample_count(encoded), 70_000)

    def test_extreme_predictors_are_signed_little_endian(self):
        minimum, _ = _encode_ima_adpcm_stream(array("h", (-32768,)), 0)
        maximum, _ = _encode_ima_adpcm_stream(array("h", (32767,)), 0)

        self.assertEqual(minimum, bytes.fromhex("00 80 00 01 00 00 00"))
        self.assertEqual(maximum, bytes.fromhex("ff 7f 00 01 00 00 00"))

    def test_hub_start_delegates_bound_metadata_to_native_stream_player(self):
        events = []

        class Motor:
            @staticmethod
            def done():
                return False

        class Speaker:
            def __init__(self):
                self.played = []

            @staticmethod
            def done():
                events.append("speaker_done")
                return True

            def play_adpcm(self, *args, **kwargs):
                events.append("play_adpcm")
                self.played.append((args, kwargs))

        class Hub:
            def __init__(self):
                self.speaker = Speaker()

        class AppData:
            def __init__(self, payload):
                self.payload = payload

            def get_bytes(self, _key):
                events.append("get_bytes")
                return self.payload

        class GC:
            @staticmethod
            def collect():
                events.append("gc_collect")

        payload = bytes.fromhex("00 00 00 03 00 00 00 d4")
        checksum = _fletcher16(payload)
        namespace = load_hub_audio_namespace()
        namespace.update({
            "hub": Hub(),
            "motors": {"drive": Motor()},
            "sampled_audio_supported": True,
            "sampled_audio_app_data": AppData(payload),
            "sampled_audio_transfer": None,
            "gc": GC(),
        })
        arguments = {
            "sample_rate_hz": 16000,
            "encoding": "ima_adpcm4_mono_stream_v1",
            "sample_count": 3,
            "byte_count": len(payload),
            "fletcher16": checksum,
        }
        begun = namespace["begin_pcm"](7, arguments)

        receipt = namespace["start_pcm"](dict(begun))

        self.assertIsNone(namespace["sampled_audio_transfer"])
        self.assertEqual(
            namespace["hub"].speaker.played,
            [
                (
                    (payload,),
                    {
                        "byte_count": len(payload),
                        "sample_count": 3,
                        "fletcher16": checksum,
                        "sample_rate": 16000,
                        "wait": False,
                    },
                )
            ],
        )
        self.assertEqual(receipt["duration_ms"], 16)
        self.assertEqual(receipt["encoding"], "ima_adpcm4_mono_stream_v1")
        self.assertEqual(
            events[-4:],
            ["speaker_done", "gc_collect", "get_bytes", "play_adpcm"],
        )

    def test_hub_failure_consumes_transfer_and_never_retries_playback(self):
        class Speaker:
            def __init__(self):
                self.calls = 0

            @staticmethod
            def done():
                return True

            def play_adpcm(self, *_args, **_kwargs):
                self.calls += 1
                raise ValueError("sampled audio checksum mismatch")

        class Hub:
            def __init__(self):
                self.speaker = Speaker()

        class AppData:
            @staticmethod
            def get_bytes(_key):
                return bytes.fromhex("00 00 00 01 00 00 00")

        payload = AppData.get_bytes(0)
        namespace = load_hub_audio_namespace()
        namespace.update({
            "hub": Hub(),
            "motors": {},
            "sampled_audio_supported": True,
            "sampled_audio_app_data": AppData(),
            "sampled_audio_transfer": None,
        })
        arguments = {
            "sample_rate_hz": 16000,
            "encoding": "ima_adpcm4_mono_stream_v1",
            "sample_count": 1,
            "byte_count": len(payload),
            "fletcher16": _fletcher16(payload),
        }
        begun = namespace["begin_pcm"](9, arguments)

        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            namespace["start_pcm"](dict(begun))

        self.assertIsNone(namespace["sampled_audio_transfer"])
        self.assertEqual(namespace["hub"].speaker.calls, 1)
        with self.assertRaisesRegex(ValueError, "transfer is invalid"):
            namespace["start_pcm"](dict(begun))
        self.assertEqual(namespace["hub"].speaker.calls, 1)

    def test_hub_changed_start_metadata_is_consumed_before_native_call(self):
        class Speaker:
            calls = 0

            @staticmethod
            def done():
                return True

            @classmethod
            def play_adpcm(cls, *_args, **_kwargs):
                cls.calls += 1

        class Hub:
            speaker = Speaker()

        payload = bytes.fromhex("00 00 00 01 00 00 00")
        namespace = load_hub_audio_namespace()
        namespace.update({
            "hub": Hub(),
            "motors": {},
            "sampled_audio_supported": True,
            "sampled_audio_transfer": None,
        })
        arguments = {
            "sample_rate_hz": 16000,
            "encoding": "ima_adpcm4_mono_stream_v1",
            "sample_count": 1,
            "byte_count": len(payload),
            "fletcher16": _fletcher16(payload),
        }
        begun = namespace["begin_pcm"](11, arguments)
        changed = dict(begun, encoding="wrong")

        with self.assertRaisesRegex(ValueError, "metadata changed"):
            namespace["start_pcm"](changed)

        self.assertIsNone(namespace["sampled_audio_transfer"])
        self.assertEqual(namespace["hub"].speaker.calls, 0)

    def test_resamples_to_sixteen_kilohertz(self):
        result = pcm16_wav_to_blast_adpcm(
            wav_bytes((0, 8_000, 16_000, 24_000), 8_000)
        )

        self.assertEqual(result.sample_count, 8)
        self.assertEqual(result.duration_ms, 16)
        self.assertEqual(
            result.payload[0:7],
            bytes.fromhex("00 00 00 08 00 00 00"),
        )

    def test_eight_seconds_is_one_maximum_sized_payload(self):
        result = pcm16_wav_to_blast_adpcm(
            wav_bytes(
                (0,) * BLAST_ADPCM_MAX_SAMPLES,
                BLAST_ADPCM_SAMPLE_RATE_HZ,
            )
        )

        self.assertEqual(result.sample_count, BLAST_ADPCM_MAX_SAMPLES)
        self.assertEqual(result.duration_ms, 8_000)
        self.assertEqual(len(result.payload), BLAST_ADPCM_MAX_BYTES)
        self.assertEqual(_adpcm_sample_count(result.payload), 128_000)

    def test_more_than_eight_seconds_fails_before_controller_upload(self):
        audio = wav_bytes(
            (0,) * (BLAST_ADPCM_MAX_SAMPLES + 1),
            BLAST_ADPCM_SAMPLE_RATE_HZ,
        )

        class Synthesizer:
            @staticmethod
            def synthesize(_text, _locale, _cancel_event):
                return audio

        class Controller:
            @staticmethod
            def play_pcm(*_args, **_kwargs):
                raise AssertionError("oversized speech must not reach BLAST")

        with self.assertRaises(HostSpeechError) as raised:
            BlastHubSpeaker(Synthesizer(), Controller())(
                "För lång", "sv", threading.Event()
            )

        self.assertEqual(raised.exception.code, "tts_audio_too_long")

    def test_speaker_preloads_once_then_waits_for_whole_utterance(self):
        sample_count = 82_080
        audio = wav_bytes(
            (0,) * sample_count,
            BLAST_ADPCM_SAMPLE_RATE_HZ,
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
                    "accepted": True,
                    "started": True,
                    "byte_count": len(payload),
                    "sample_count": _adpcm_sample_count(payload),
                    "duration_ms": 5_136,
                }

        synthesizer = Synthesizer()
        controller = Controller()
        cancel = threading.Event()

        with mock.patch.object(cancel, "wait", return_value=False) as waited:
            result = BlastHubSpeaker(synthesizer, controller)(
                "En riktig mening", "sv", cancel
            )

        self.assertEqual(
            synthesizer.call,
            ("En riktig mening", "sv", cancel),
        )
        self.assertEqual(len(controller.calls), 1)
        self.assertEqual(
            _adpcm_sample_count(controller.calls[0][0]),
            sample_count,
        )
        self.assertTrue(callable(controller.calls[0][1]))
        waited.assert_called_once_with(5.136)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["duration_ms"], 5_136)

    def test_speaker_marks_started_only_after_validated_pcm_receipt(self):
        audio = wav_bytes((0,) * 256, BLAST_ADPCM_SAMPLE_RATE_HZ)

        class Synthesizer:
            @staticmethod
            def synthesize(_text, _locale, _cancel_event):
                return audio

        class CancelEvent(threading.Event):
            def __init__(self):
                super().__init__()
                self.marked = False

            def mark_playback_started(self):
                self.marked = True

        cancel = CancelEvent()

        class Controller:
            @staticmethod
            def play_pcm(payload, **_kwargs):
                self.assertFalse(cancel.marked)
                return {
                    "accepted": True,
                    "started": True,
                    "duration_ms": 16,
                    "sample_count": _adpcm_sample_count(payload),
                }

        with mock.patch.object(cancel, "wait", return_value=False):
            BlastHubSpeaker(Synthesizer(), Controller())(
                "Hej", "sv", cancel,
            )

        self.assertTrue(cancel.marked)

    def test_cancel_during_started_utterance_finishes_speech_worker(self):
        audio = wav_bytes((0,) * 16_000, BLAST_ADPCM_SAMPLE_RATE_HZ)

        class Synthesizer:
            @staticmethod
            def synthesize(_text, _locale, _cancel_event):
                return audio

        class Controller:
            def __init__(self):
                self.payloads = []

            def play_pcm(self, payload, **_kwargs):
                self.payloads.append(payload)
                return {
                    "accepted": True,
                    "started": True,
                    "duration_ms": 1_008,
                }

        controller = Controller()
        cancel = threading.Event()
        with mock.patch.object(cancel, "wait", return_value=True):
            result = BlastHubSpeaker(Synthesizer(), controller)(
                "Avbryt", "sv", cancel
            )

        self.assertIsNone(result)
        self.assertEqual(len(controller.payloads), 1)

    def test_speaker_does_not_play_after_synthesis_cancellation(self):
        cancel = threading.Event()
        cancel.set()

        class Synthesizer:
            @staticmethod
            def synthesize(_text, _locale, _cancel_event):
                return b"unused"

        class Controller:
            @staticmethod
            def play_pcm(*_args, **_kwargs):
                raise AssertionError("cancelled speech must not reach BLAST")

        self.assertIsNone(
            BlastHubSpeaker(Synthesizer(), Controller())("Hej", "sv", cancel)
        )

    def test_speaker_rejects_duration_that_does_not_match_payload(self):
        class Synthesizer:
            @staticmethod
            def synthesize(_text, _locale, _cancel_event):
                return wav_bytes((0,), BLAST_ADPCM_SAMPLE_RATE_HZ)

        class Controller:
            @staticmethod
            def play_pcm(_payload, **_kwargs):
                return {"duration_ms": 2}

        with self.assertRaisesRegex(
            HostSpeechError,
            "invalid sampled-audio duration",
        ):
            BlastHubSpeaker(Synthesizer(), Controller())(
                "Hej", "sv", threading.Event()
            )


if __name__ == "__main__":
    unittest.main()
