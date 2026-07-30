import json
import subprocess
import unittest
from dataclasses import dataclass

from robot_agent.shadow_cli import (
    EV3SSHConfigurationError,
    EV3SSHProtocolError,
    EV3SSHTimeoutError,
    EV3SSHTransport,
    PersistentShadowTransport,
    REMOTE_ROBOT_CLI,
    run_shadow_cycle,
)


@dataclass
class Completed:
    returncode: int = 0
    stdout: str = ""
    stderr: str = ""


class ScriptedRunner:
    def __init__(self, results):
        self.results = list(results)
        self.calls = []

    def __call__(self, argv, **kwargs):
        self.calls.append((list(argv), dict(kwargs)))
        result = self.results.pop(0)
        if isinstance(result, BaseException):
            raise result
        return result


class Candidate:
    def __init__(self, text, latency_ms=10):
        self.text = text
        self.latency_ms = latency_ms


class FakeModel:
    def __init__(self, text):
        self.text = text
        self.observations = []

    def comment(self, observation):
        self.observations.append(observation)
        return Candidate(self.text)


def sensor_result(value, timestamp):
    return Completed(
        stdout=json.dumps(
            {
                "role": "infrared",
                "driver": "lego-ev3-ir",
                "mode": "IR-PROX",
                "units": "pct",
                "value0": value,
                "observed_monotonic_ms": timestamp,
            }
        )
    )


class ShadowCLITests(unittest.TestCase):
    def test_persistent_shadow_transport_delegates_sensor_and_speech(self):
        class SensorSession:
            def __init__(self):
                self.roles = []

            def read_sensor(self, role):
                self.roles.append(role)
                return {"role": role, "value0": 22}

        class SpeechTransport:
            def __init__(self):
                self.texts = []

            def speak(self, text):
                self.texts.append(text)
                return {"status": "completed"}

        sensor = SensorSession()
        speech = SpeechTransport()
        transport = PersistentShadowTransport(sensor, speech)

        self.assertEqual(
            transport.read_infrared()["value0"],
            22,
        )
        self.assertEqual(
            transport.speak("Hej"),
            {"status": "completed"},
        )
        self.assertEqual(sensor.roles, ["infrared"])
        self.assertEqual(speech.texts, ["Hej"])

    def test_target_validation_rejects_option_and_shell_characters(self):
        for target in [
            "",
            "-oProxyCommand=bad",
            "robot@host;bad",
            "robot@host bad",
            " robot@host",
        ]:
            with self.subTest(target=target):
                with self.assertRaises(EV3SSHConfigurationError):
                    EV3SSHTransport(target)

    def test_sensor_read_uses_only_fixed_noninteractive_command(self):
        runner = ScriptedRunner([sensor_result(28, 100)])
        transport = EV3SSHTransport(
            "robot@fe80::1234%en9",
            runner=runner,
        )

        reading = transport.read_infrared()

        self.assertEqual(reading["value0"], 28)
        argv, kwargs = runner.calls[0]
        self.assertEqual(
            argv,
            [
                "ssh",
                "-T",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=3",
                "-o",
                "StrictHostKeyChecking=yes",
                "-o",
                "ControlMaster=auto",
                "-o",
                "ControlPath=~/.ssh/robot-llm-%C",
                "-o",
                "ControlPersist=60",
                "robot@fe80::1234%en9",
                "python3",
                REMOTE_ROBOT_CLI,
                "read-sensor",
                "--role",
                "infrared",
            ],
        )
        self.assertIsNone(kwargs["input"])
        self.assertFalse(kwargs["check"])
        self.assertEqual(kwargs["timeout"], 20)

    def test_speech_text_is_stdin_data_and_never_an_argument(self):
        runner = ScriptedRunner(
            [Completed(stdout='{"status":"completed","characters":31}')]
        )
        transport = EV3SSHTransport("robot@ev3dev.local", runner=runner)
        text = "Vad fan är det där; $(absolut inte kod)"

        result = transport.speak(text)

        self.assertEqual(result["status"], "completed")
        argv, kwargs = runner.calls[0]
        self.assertNotIn(text, argv)
        self.assertEqual(kwargs["input"], text + "\n")
        self.assertEqual(argv[-1], "speak-stdin")

    def test_bad_sensor_shape_is_rejected(self):
        bad_payloads = [
            {"role": "touch", "driver": "lego-ev3-ir", "mode": "IR-PROX",
             "units": "pct", "value0": 28, "observed_monotonic_ms": 1},
            {"role": "infrared", "driver": "lego-ev3-ir", "mode": "IR-SEEK",
             "units": "pct", "value0": 28, "observed_monotonic_ms": 1},
            {"role": "infrared", "driver": "lego-ev3-ir", "mode": "IR-PROX",
             "units": "pct", "value0": True, "observed_monotonic_ms": 1},
            {"role": "infrared", "driver": "lego-ev3-ir", "mode": "IR-PROX",
             "units": "pct", "value0": 101, "observed_monotonic_ms": 1},
        ]
        for payload in bad_payloads:
            with self.subTest(payload=payload):
                runner = ScriptedRunner(
                    [Completed(stdout=json.dumps(payload))]
                )
                with self.assertRaises(EV3SSHProtocolError):
                    EV3SSHTransport(
                        "robot@ev3dev.local",
                        runner=runner,
                    ).read_infrared()

    def test_timeout_is_wrapped(self):
        runner = ScriptedRunner(
            [subprocess.TimeoutExpired(cmd="ssh", timeout=5)]
        )
        transport = EV3SSHTransport("robot@ev3dev.local", runner=runner)

        with self.assertRaises(EV3SSHTimeoutError):
            transport.read_infrared()

    def test_full_cycle_reads_three_times_and_speaks_only_fallback(self):
        runner = ScriptedRunner(
            [
                sensor_result(27, 100),
                sensor_result(28, 110),
                sensor_result(29, 120),
                Completed(
                    stdout='{"status":"completed","characters":31}'
                ),
            ]
        )
        transport = EV3SSHTransport("robot@ev3dev.local", runner=runner)
        hallucination = "En hund står femton centimeter bort."
        model = FakeModel(hallucination)

        result = run_shadow_cycle(transport, model)

        self.assertEqual(len(runner.calls), 4)
        self.assertEqual(len(model.observations), 1)
        self.assertEqual(result.model_candidate, hallucination)
        self.assertEqual(
            result.spoken_text,
            "Jag märker något framför mig.",
        )
        speech_argv, speech_kwargs = runner.calls[-1]
        self.assertNotIn(hallucination, speech_argv)
        self.assertNotIn(hallucination, speech_kwargs["input"])
        self.assertEqual(
            speech_kwargs["input"],
            result.fallback_text + "\n",
        )

    def test_timestamp_regression_blocks_model_and_tts(self):
        runner = ScriptedRunner(
            [
                sensor_result(27, 100),
                sensor_result(28, 90),
                sensor_result(29, 120),
            ]
        )
        model = FakeModel("Något är nära.")

        with self.assertRaises(EV3SSHProtocolError):
            run_shadow_cycle(
                EV3SSHTransport("robot@ev3dev.local", runner=runner),
                model,
            )

        self.assertEqual(model.observations, [])
        self.assertEqual(len(runner.calls), 3)


if __name__ == "__main__":
    unittest.main()
