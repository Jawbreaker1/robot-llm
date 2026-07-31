import io
import json
import queue
import subprocess
import threading
import unittest
import wave

from robot_agent.ev3_audio_transport import (
    EV3WAVSSHSession,
    READY_SCHEMA,
    RESPONSE_SCHEMA,
)
from robot_agent.host_piper_speech import validate_pcm16_mono_wav


def wav_bytes(frames=2205):
    output = io.BytesIO()
    with wave.open(output, "wb") as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(22050)
        target.writeframes(b"\x00\x00" * frames)
    return output.getvalue()


class QueueReader:
    def __init__(self):
        self.values = queue.Queue()

    def readline(self, _limit=-1):
        return self.values.get(timeout=1)

    def read(self, _size=-1):
        return b""


class BinaryRequestWriter:
    def __init__(self, stdout):
        self.stdout = stdout
        self.buffer = bytearray()
        self.pending = None
        self.requests = []
        self.request_written = threading.Event()
        self.respond = True

    def write(self, raw):
        self.buffer.extend(bytes(raw))
        while True:
            if self.pending is None:
                newline = self.buffer.find(b"\n")
                if newline < 0:
                    break
                header = json.loads(bytes(self.buffer[:newline]).decode("ascii"))
                del self.buffer[:newline + 1]
                self.pending = header
                self.requests.append(header)
                self.request_written.set()
            count = self.pending["byte_count"]
            if len(self.buffer) < count:
                break
            audio = bytes(self.buffer[:count])
            del self.buffer[:count]
            header = self.pending
            self.pending = None
            if self.respond:
                if header["operation"] == "shutdown":
                    result = {"status": "shutdown"}
                else:
                    metadata = validate_pcm16_mono_wav(audio)
                    result = {
                        "status": "completed",
                        "bytes": metadata.byte_count,
                        "channels": metadata.channels,
                        "sample_width_bytes": metadata.sample_width_bytes,
                        "sample_rate_hz": metadata.sample_rate_hz,
                        "frames": metadata.frames,
                        "duration_ms": metadata.duration_ms,
                    }
                response = {
                    "schema": RESPONSE_SCHEMA,
                    "request_id": header["request_id"],
                    "ok": True,
                    "result": result,
                }
                self.stdout.values.put(
                    (json.dumps(response) + "\n").encode("ascii")
                )
        return len(raw)

    def flush(self):
        return None


class Process:
    def __init__(self):
        self.stdout = QueueReader()
        self.stdout.values.put(
            (json.dumps({"schema": READY_SCHEMA, "status": "ready"}) + "\n").encode("ascii")
        )
        self.stderr = QueueReader()
        self.stderr.values.put(b"")
        self.stdin = BinaryRequestWriter(self.stdout)
        self.returncode = None
        self.terminated = False

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        self.returncode = -15
        self.stdout.values.put(b"")

    def kill(self):
        self.returncode = -9
        self.stdout.values.put(b"")

    def wait(self, timeout=None):
        if self.returncode is None:
            raise subprocess.TimeoutExpired("ssh", timeout)
        return self.returncode


class EV3AudioTransportTests(unittest.TestCase):
    def test_session_is_lazy_and_one_process_handles_multiple_wavs(self):
        process = Process()
        factory_calls = []
        session = EV3WAVSSHSession(
            "robot@ev3dev.local",
            process_factory=lambda *args, **kwargs: (
                factory_calls.append((args, kwargs)) or process
            ),
            startup_timeout_seconds=1,
            poll_seconds=0.01,
        )
        first = wav_bytes(1000)
        second = wav_bytes(2000)
        cancel = threading.Event()

        self.assertEqual(factory_calls, [])
        self.assertEqual(session.play(first, cancel)["bytes"], len(first))
        self.assertEqual(session.play(second, cancel)["bytes"], len(second))
        self.assertTrue(session.close())

        self.assertEqual(len(factory_calls), 1)
        self.assertEqual(
            [item["operation"] for item in process.stdin.requests],
            ["play", "play", "shutdown"],
        )
        self.assertTrue(process.terminated)

    def test_cancellation_aborts_only_owned_audio_ssh(self):
        process = Process()
        process.stdin.respond = False
        session = EV3WAVSSHSession(
            "robot@ev3dev.local",
            process_factory=lambda *_args, **_kwargs: process,
            startup_timeout_seconds=1,
            poll_seconds=0.01,
        )
        cancel = threading.Event()
        results = []
        thread = threading.Thread(
            target=lambda: results.append(session.play(wav_bytes(), cancel)),
            daemon=True,
        )
        thread.start()
        self.assertTrue(process.stdin.request_written.wait(1))
        cancel.set()
        thread.join(1)

        self.assertFalse(thread.is_alive())
        self.assertEqual(results, [None])
        self.assertTrue(process.terminated)


if __name__ == "__main__":
    unittest.main()
