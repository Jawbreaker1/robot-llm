import ast
import fcntl
import io
import json
import tempfile
import unittest
import wave
from pathlib import Path

from ev3.audio_playback_worker_cli import (
    READY_SCHEMA,
    REQUEST_SCHEMA,
    RESPONSE_SCHEMA,
    run_worker,
)


def wav_bytes(frames=2205):
    output = io.BytesIO()
    with wave.open(output, "wb") as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(22050)
        target.writeframes(b"\x00\x00" * frames)
    return output.getvalue()


def request(request_id, operation, audio=b""):
    header = {
        "schema": REQUEST_SCHEMA,
        "request_id": request_id,
        "operation": operation,
        "byte_count": len(audio),
    }
    return (json.dumps(header, separators=(",", ":")) + "\n").encode("ascii") + audio


class Player:
    def __init__(self):
        self.returncode = 0
        self.audio = None

    def communicate(self, input=None, timeout=None):
        self.audio = input
        return b"", b""

    def poll(self):
        return self.returncode


class EV3AudioPlaybackWorkerTests(unittest.TestCase):
    def test_worker_is_python35_compatible(self):
        path = Path(__file__).resolve().parents[1] / "ev3" / "audio_playback_worker_cli.py"
        source = path.read_text(encoding="utf-8")
        ast.parse(source, feature_version=5)
        self.assertNotIn(".isascii(", source)

    def test_one_worker_plays_multiple_binary_wavs_then_shuts_down(self):
        first = wav_bytes(1000)
        second = wav_bytes(2000)
        raw = (
            request("audio-1", "play", first)
            + request("audio-2", "play", second)
            + request("shutdown-3", "shutdown")
        )
        players = []
        robots = []
        with tempfile.TemporaryDirectory() as directory:
            lock_path = str(Path(directory) / "audio.lock")

            class Robot:
                def __init__(self, _config):
                    robots.append(self)

                def _acquire_speech_lock(self):
                    handle = open(lock_path, "a+")
                    fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                    return handle

            def player_factory(*_args, **_kwargs):
                player = Player()
                players.append(player)
                return player

            output = io.StringIO()
            status = run_worker(
                input_stream=io.BytesIO(raw),
                output_stream=output,
                robot_factory=Robot,
                player_factory=player_factory,
            )

        frames = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(status, 0)
        self.assertEqual(frames[0], {"schema": READY_SCHEMA, "status": "ready"})
        self.assertEqual([frame["schema"] for frame in frames[1:]], [
            RESPONSE_SCHEMA,
            RESPONSE_SCHEMA,
            RESPONSE_SCHEMA,
        ])
        self.assertEqual([frame["request_id"] for frame in frames[1:]], [
            "audio-1",
            "audio-2",
            "shutdown-3",
        ])
        self.assertEqual(len(robots), 1)
        self.assertEqual([player.audio for player in players], [first, second])

    def test_truncated_binary_payload_is_fatal_and_correlated(self):
        audio = wav_bytes()
        header = request("audio-1", "play", audio)[:100]
        output = io.StringIO()

        status = run_worker(
            input_stream=io.BytesIO(header),
            output_stream=output,
            robot_factory=lambda _config: object(),
        )

        frames = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(status, 1)
        self.assertEqual(frames[-1]["request_id"], "audio-1")
        self.assertTrue(frames[-1]["error"]["fatal"])


if __name__ == "__main__":
    unittest.main()
