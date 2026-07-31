import ast
import fcntl
import io
import json
import tempfile
import unittest
import wave
from pathlib import Path

from ev3.audio_playback_cli import (
    AudioValidationError,
    play_wav,
    run,
    validate_wav,
)


def wav_bytes(frames=2205, rate=22050, channels=1, width=2):
    output = io.BytesIO()
    with wave.open(output, "wb") as target:
        target.setnchannels(channels)
        target.setsampwidth(width)
        target.setframerate(rate)
        target.writeframes(b"\x00" * frames * channels * width)
    return output.getvalue()


class Player:
    def __init__(self):
        self.returncode = 0
        self.input = None

    def communicate(self, input=None, timeout=None):
        self.input = input
        return b"", b""

    def poll(self):
        return self.returncode


class EV3AudioPlaybackTests(unittest.TestCase):
    def test_module_remains_python35_compatible(self):
        path = Path(__file__).resolve().parents[1] / "ev3" / "audio_playback_cli.py"
        ast.parse(path.read_text(encoding="utf-8"), feature_version=5)

    def test_validation_accepts_only_bounded_pcm16_mono(self):
        result = validate_wav(wav_bytes())
        self.assertEqual(result["sample_rate_hz"], 22050)
        self.assertEqual(result["duration_ms"], 100)
        for raw in (b"not wav", wav_bytes(channels=2), wav_bytes(width=1)):
            with self.subTest(size=len(raw)), self.assertRaises(AudioValidationError):
                validate_wav(raw)

    def test_playback_uses_audio_lock_and_streams_stdin_without_file(self):
        audio = wav_bytes()
        player = Player()
        calls = []
        with tempfile.TemporaryDirectory() as directory:
            lock_path = str(Path(directory) / "audio.lock")

            class Robot:
                def __init__(self, _config):
                    self.handle = None

                def _acquire_speech_lock(self):
                    self.handle = open(lock_path, "a+")
                    fcntl.flock(
                        self.handle.fileno(),
                        fcntl.LOCK_EX | fcntl.LOCK_NB,
                    )
                    return self.handle

            result = play_wav(
                audio,
                robot_factory=Robot,
                popen_factory=lambda argv, **kwargs: (
                    calls.append((list(argv), kwargs)) or player
                ),
            )

        self.assertEqual(calls[0][0], ["aplay", "--quiet"])
        self.assertEqual(player.input, audio)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["bytes"], len(audio))

    def test_cli_returns_one_machine_readable_receipt(self):
        audio = wav_bytes()
        player = Player()

        class Lock:
            def fileno(self):
                return self._file.fileno()

            def close(self):
                self._file.close()

        with tempfile.TemporaryDirectory() as directory:
            lock_file = str(Path(directory) / "audio.lock")

            class Robot:
                def __init__(self, _config):
                    pass

                def _acquire_speech_lock(self):
                    handle = open(lock_file, "a+")
                    fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                    return handle

            output = io.StringIO()
            status = run(
                input_stream=io.BytesIO(audio),
                output_stream=output,
                robot_factory=Robot,
                popen_factory=lambda *_args, **_kwargs: player,
            )

        self.assertEqual(status, 0)
        receipt = json.loads(output.getvalue())
        self.assertEqual(receipt["status"], "completed")
        self.assertEqual(receipt["bytes"], len(audio))


if __name__ == "__main__":
    unittest.main()
