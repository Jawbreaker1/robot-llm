import json
import queue
import subprocess
import threading
import unittest

from robot_agent.ev3_speech_transport import (
    EV3SpeechSSHSession,
    READY_SCHEMA,
    RESPONSE_SCHEMA,
)


class QueueReader:
    def __init__(self):
        self.values = queue.Queue()

    def readline(self, _limit=-1):
        return self.values.get(timeout=1)

    def read(self, _size=-1):
        return b""


class RequestWriter:
    def __init__(self, stdout):
        self.stdout = stdout
        self.requests = []
        self.request_written = threading.Event()

    def write(self, raw):
        request = json.loads(raw.decode("utf-8"))
        self.requests.append(request)
        self.request_written.set()
        status = "shutdown" if request["operation"] == "shutdown" else "completed"
        response = {
            "schema": RESPONSE_SCHEMA,
            "request_id": request["request_id"],
            "ok": True,
            "result": {"status": status},
        }
        self.stdout.values.put((json.dumps(response) + "\n").encode("utf-8"))
        return len(raw)

    def flush(self):
        return None


class Process:
    def __init__(self):
        self.stdout = QueueReader()
        self.stdout.values.put(
            (json.dumps({"schema": READY_SCHEMA, "status": "ready"}) + "\n").encode("utf-8")
        )
        self.stderr = QueueReader()
        self.stderr.values.put(b"")
        self.stdin = RequestWriter(self.stdout)
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


class EV3SpeechTransportTests(unittest.TestCase):
    def test_one_process_handles_multiple_utterances_and_shutdown(self):
        process = Process()
        factory_calls = []

        def factory(*args, **kwargs):
            factory_calls.append((args, kwargs))
            return process

        session = EV3SpeechSSHSession(
            "robot@ev3dev.local",
            process_factory=factory,
            request_timeout_seconds=1,
        )
        session.start()
        cancel = threading.Event()

        self.assertEqual(session.speak("Hej", "sv", cancel)["status"], "completed")
        self.assertEqual(session.speak("Hello", "en", cancel)["status"], "completed")
        self.assertTrue(session.close(timeout_seconds=0.01))

        self.assertEqual(len(factory_calls), 1)
        self.assertEqual(
            [request["operation"] for request in process.stdin.requests],
            ["speak", "speak", "shutdown"],
        )
        self.assertTrue(process.terminated)

    def test_cancellation_terminates_the_owned_ssh_process(self):
        process = Process()

        def hold_response(raw):
            request = json.loads(raw.decode("utf-8"))
            process.stdin.requests.append(request)
            process.stdin.request_written.set()
            return len(raw)

        process.stdin.write = hold_response
        session = EV3SpeechSSHSession(
            "robot@ev3dev.local",
            process_factory=lambda *args, **kwargs: process,
            request_timeout_seconds=1,
            poll_seconds=0.01,
        )
        session.start()
        cancel = threading.Event()
        results = []
        thread = threading.Thread(
            target=lambda: results.append(
                session.speak("Stopp", "sv", cancel)
            ),
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
