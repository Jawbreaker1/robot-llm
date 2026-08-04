import io
import json
import subprocess
import threading
import time
import unittest
import wave

from robot_agent.host_piper_speech import (
    DEFAULT_ENGLISH_VOICE,
    EV3WAVSSHPlayer,
    HostPiperEV3Speaker,
    HostSpeechError,
    LocaleSpeechSynthesizer,
    MacOSSayWAVSynthesizer,
    PiperLoopbackSynthesizer,
    PiperSpeechProfile,
    validate_pcm16_mono_wav,
)
from robot_agent.robot_speech_runtime import RobotSpeechRuntime


def wav_bytes(frames=2205, rate=22050):
    output = io.BytesIO()
    with wave.open(output, "wb") as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(rate)
        target.writeframes(b"\x00\x00" * frames)
    return output.getvalue()


class RecordingInput:
    def __init__(self):
        self.data = bytearray()
        self.closed = False

    def write(self, value):
        self.data.extend(value)

    def close(self):
        self.closed = True


class ImmediateProcess:
    def __init__(self, stdout=b"", stderr=b"", returncode=0):
        self.stdout_value = stdout
        self.stderr_value = stderr
        self.returncode = returncode
        self.inputs = []
        self.stdin = RecordingInput()
        self.stdout = io.BytesIO(stdout)
        self.stderr = io.BytesIO(stderr)

    def communicate(self, input=None, timeout=None):
        self.inputs.append(input)
        return self.stdout_value, self.stderr_value

    def poll(self):
        return self.returncode


class BlockingProcess(ImmediateProcess):
    def __init__(self):
        super().__init__()
        self.returncode = None
        self.started = threading.Event()
        self.terminated = False

    def communicate(self, input=None, timeout=None):
        self.inputs.append(input)
        self.started.set()
        raise subprocess.TimeoutExpired("process", timeout)

    def terminate(self):
        self.terminated = True
        self.returncode = -15

    def wait(self, timeout=None):
        return self.returncode

    def kill(self):
        self.returncode = -9


class StubbornProcess(BlockingProcess):
    def __init__(self):
        super().__init__()
        self.killed = False

    def terminate(self):
        self.terminated = True

    def wait(self, timeout=None):
        if not self.killed:
            raise subprocess.TimeoutExpired("process", timeout)
        return self.returncode

    def kill(self):
        self.killed = True
        self.returncode = -9


class OversizedStream:
    def __init__(self, total):
        self.remaining = total

    def read(self, count):
        if self.remaining <= 0:
            return b""
        size = min(count, self.remaining)
        self.remaining -= size
        return b"x" * size


class HostPiperSpeechTests(unittest.TestCase):
    def test_default_profile_is_explicit_swedish_nst_deep(self):
        profile = PiperSpeechProfile()
        self.assertEqual(profile.endpoint, "http://127.0.0.1:8179/v1/audio/speech")
        self.assertEqual(profile.model, "piper-sv")
        self.assertEqual(profile.voice_for_locale("sv"), "nst-deep")
        with self.assertRaises(HostSpeechError):
            profile.voice_for_locale("en")
        for url in (
            "https://127.0.0.1:8179/v1",
            "http://localhost:8179/v1",
            "http://speech.example:8179/v1",
        ):
            with self.subTest(url=url), self.assertRaises(ValueError):
                PiperSpeechProfile(base_url=url)

    def test_piper_posts_bounded_json_on_stdin_and_returns_valid_wav(self):
        audio = wav_bytes()
        process = ImmediateProcess(stdout=audio)
        seen = []

        def factory(argv, **kwargs):
            seen.append((list(argv), kwargs))
            return process

        synth = PiperLoopbackSynthesizer(process_factory=factory)
        result = synth.synthesize("Hej robot", "sv", threading.Event())

        self.assertEqual(result, audio)
        argv = seen[0][0]
        self.assertEqual(argv[-1], "http://127.0.0.1:8179/v1/audio/speech")
        self.assertNotIn("Hej robot", argv)
        request = json.loads(bytes(process.stdin.data).decode("utf-8"))
        self.assertEqual(
            request,
            {
                "model": "piper-sv",
                "input": "Hej robot",
                "voice": "nst-deep",
                "response_format": "wav",
                "speed": 1.0,
            },
        )

    def test_piper_cancellation_terminates_only_its_process(self):
        process = BlockingProcess()
        synth = PiperLoopbackSynthesizer(
            process_factory=lambda *_args, **_kwargs: process,
        )
        cancel = threading.Event()
        result = []
        thread = threading.Thread(
            target=lambda: result.append(synth.synthesize("Hej", "sv", cancel)),
            daemon=True,
        )
        thread.start()
        deadline = time.monotonic() + 1
        while not process.stdin.data and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(process.stdin.data)
        cancel.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result, [None])
        self.assertTrue(process.terminated)

    def test_macos_say_writes_text_on_stdin_and_returns_valid_wav(self):
        audio = wav_bytes()
        process = ImmediateProcess()
        seen = []

        def factory(argv, **kwargs):
            seen.append((list(argv), kwargs))
            output = argv[argv.index("-o") + 1]
            with open(output, "wb") as target:
                target.write(audio)
            return process

        synth = MacOSSayWAVSynthesizer(process_factory=factory)
        result = synth.synthesize(
            "Oh, for heaven's sake.",
            "en",
            threading.Event(),
        )

        self.assertEqual(result, audio)
        argv = seen[0][0]
        self.assertEqual(argv[0], "/usr/bin/say")
        self.assertEqual(argv[argv.index("-v") + 1], DEFAULT_ENGLISH_VOICE)
        self.assertNotIn("Oh, for heaven's sake.", argv)
        self.assertEqual(
            bytes(process.stdin.data),
            b"Oh, for heaven's sake.",
        )
        self.assertIn("--data-format=LEI16@22050", argv)
        self.assertIn("--file-format=WAVE", argv)

    def test_macos_say_cancellation_terminates_its_process(self):
        process = BlockingProcess()
        synth = MacOSSayWAVSynthesizer(
            process_factory=lambda *_args, **_kwargs: process,
        )
        cancel = threading.Event()
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                synth.synthesize("Please stop.", "en", cancel)
            ),
            daemon=True,
        )

        thread.start()
        deadline = time.monotonic() + 1
        while not process.stdin.data and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(process.stdin.data)
        cancel.set()
        thread.join(1)

        self.assertFalse(thread.is_alive())
        self.assertEqual(result, [None])
        self.assertTrue(process.terminated)

    def test_locale_router_uses_only_the_configured_provider(self):
        class Synthesizer:
            def __init__(self, name):
                self.name = name
                self.calls = []

            def synthesize(self, text, locale, cancel_event):
                self.calls.append((text, locale, cancel_event))
                return self.name.encode("ascii")

        swedish = Synthesizer("swedish")
        english = Synthesizer("english")
        router = LocaleSpeechSynthesizer(
            {"sv": swedish, "en": english}
        )
        cancel = threading.Event()

        self.assertEqual(router.locales, ("sv", "en"))
        self.assertEqual(router.synthesize("Hej", "sv", cancel), b"swedish")
        self.assertEqual(
            router.synthesize("Hello", "en", cancel),
            b"english",
        )
        self.assertEqual(len(swedish.calls), 1)
        self.assertEqual(len(english.calls), 1)
        with self.assertRaises(HostSpeechError) as caught:
            router.synthesize("Hallo", "de", cancel)
        self.assertEqual(caught.exception.code, "unsupported_tts_locale")

    def test_chunked_style_oversized_output_is_stopped_at_capture_limit(self):
        process = BlockingProcess()
        process.stdout = OversizedStream(4 * 1024 * 1024 + 100_000)
        synth = PiperLoopbackSynthesizer(
            process_factory=lambda *_args, **_kwargs: process,
        )

        with self.assertRaises(HostSpeechError) as caught:
            synth.synthesize("Hej", "sv", threading.Event())

        self.assertEqual(caught.exception.code, "tts_response_too_large")
        self.assertTrue(process.terminated)

    def test_terminate_timeout_escalates_to_kill_and_next_episode_speaks(self):
        audio = wav_bytes()
        stubborn = StubbornProcess()
        healthy = ImmediateProcess(stdout=audio)
        processes = iter((stubborn, healthy))
        synth = PiperLoopbackSynthesizer(
            process_factory=lambda *_args, **_kwargs: next(processes),
        )

        class Player:
            def __init__(self):
                self.played = threading.Event()

            def play(self, _audio, _cancel):
                self.played.set()

        player = Player()
        speaker = HostPiperEV3Speaker(synth, player)
        first = RobotSpeechRuntime(speaker=speaker)
        first.start()
        first.offer(episode_id="episode-one", text="Första", locale="sv")
        deadline = time.monotonic() + 1
        while not stubborn.stdin.data and time.monotonic() < deadline:
            time.sleep(0.01)
        started = time.monotonic()
        self.assertTrue(first.close(timeout_seconds=1.0))
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertTrue(stubborn.terminated)
        self.assertTrue(stubborn.killed)

        second = RobotSpeechRuntime(speaker=speaker)
        second.start()
        second.offer(episode_id="episode-two", text="Andra", locale="sv")
        self.assertTrue(player.played.wait(1))
        self.assertTrue(second.close(timeout_seconds=1.0))

    def test_profile_validation_is_total_for_malformed_voice_and_timing(self):
        invalid = (
            {"voices": None},
            {"voices": (None,)},
            {"voices": (("sv",),)},
            {"voices": (("sv", []),)},
            {"voices": (("sv", "nst"), ("sv", "nst-deep"))},
            {"speed": None},
            {"speed": float("nan")},
            {"connect_timeout_seconds": "2"},
            {"request_timeout_seconds": float("inf")},
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                PiperSpeechProfile(**values)

    def test_ev3_player_streams_wav_to_fixed_remote_cli(self):
        audio = wav_bytes()
        metadata = validate_pcm16_mono_wav(audio)
        receipt = {
            "status": "completed",
            "bytes": metadata.byte_count,
            "channels": metadata.channels,
            "sample_width_bytes": metadata.sample_width_bytes,
            "sample_rate_hz": metadata.sample_rate_hz,
            "frames": metadata.frames,
            "duration_ms": metadata.duration_ms,
        }
        process = ImmediateProcess(
            stdout=(json.dumps(receipt) + "\n").encode("ascii")
        )
        player = EV3WAVSSHPlayer(
            "robot@ev3dev.local",
            process_factory=lambda *_args, **_kwargs: process,
        )

        self.assertEqual(player.play(audio, threading.Event()), receipt)
        self.assertEqual(process.inputs, [audio])
        self.assertEqual(
            player.argv[-2:],
            ["python3", "/home/robot/robot-llm/ev3/audio_playback_cli.py"],
        )

    def test_composed_speaker_never_plays_after_cancelled_synthesis(self):
        class Synthesizer:
            def synthesize(self, _text, _locale, cancel):
                cancel.set()
                return wav_bytes()

        class Player:
            def play(self, _audio, _cancel):
                raise AssertionError("playback must not start")

        cancel = threading.Event()
        speaker = HostPiperEV3Speaker(Synthesizer(), Player())
        self.assertIsNone(speaker("Hej", "sv", cancel))


if __name__ == "__main__":
    unittest.main()
