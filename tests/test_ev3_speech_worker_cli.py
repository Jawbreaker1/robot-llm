import ast
import io
import json
import unittest
from pathlib import Path

from ev3.speech_worker_cli import (
    READY_SCHEMA,
    REQUEST_SCHEMA,
    RESPONSE_SCHEMA,
    run_worker,
)


class EV3SpeechWorkerTests(unittest.TestCase):
    def test_worker_preloads_one_hal_and_speaks_multiple_times(self):
        requests = [
            {
                "schema": REQUEST_SCHEMA,
                "request_id": "speech-1",
                "operation": "speak",
                "arguments": {"text": "Hej", "voice": "sv"},
            },
            {
                "schema": REQUEST_SCHEMA,
                "request_id": "speech-2",
                "operation": "speak",
                "arguments": {"text": "Hello", "voice": "en"},
            },
            {
                "schema": REQUEST_SCHEMA,
                "request_id": "shutdown-3",
                "operation": "shutdown",
                "arguments": {},
            },
        ]
        source = io.StringIO(
            "".join(json.dumps(value) + "\n" for value in requests)
        )
        output = io.StringIO()
        instances = []

        class Robot:
            def __init__(self, config_path):
                self.config_path = config_path
                self.calls = []
                instances.append(self)

            def speak(self, text, voice="sv"):
                self.calls.append((text, voice))
                return {"status": "completed", "characters": len(text)}

        self.assertEqual(
            run_worker(
                config_path="config.json",
                input_stream=source,
                output_stream=output,
                robot_factory=Robot,
            ),
            0,
        )
        frames = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(frames[0], {"schema": READY_SCHEMA, "status": "ready"})
        self.assertEqual([frame["schema"] for frame in frames[1:]], [
            RESPONSE_SCHEMA,
            RESPONSE_SCHEMA,
            RESPONSE_SCHEMA,
        ])
        self.assertEqual(len(instances), 1)
        self.assertEqual(
            instances[0].calls,
            [("Hej", "sv"), ("Hello", "en")],
        )

    def test_worker_source_is_python35_compatible(self):
        path = Path(__file__).resolve().parents[1] / "ev3" / "speech_worker_cli.py"
        source = path.read_text(encoding="utf-8")
        ast.parse(source, feature_version=5)
        self.assertNotIn(".isascii(", source)


if __name__ == "__main__":
    unittest.main()
