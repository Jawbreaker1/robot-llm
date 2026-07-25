import json
import subprocess
import sys
import unittest
from pathlib import Path

from ev3.ir_gate_probe import (
    APPROACH_PROMPT,
    COMPLETE_PROMPT,
    ENTRY_TIMEOUT_PROMPT,
    EXIT_TIMEOUT_PROMPT,
    RETREAT_PROMPT,
    GatePolicy,
    IRGateProbe,
)
from ev3.robot_hal import SafetyError


PROJECT_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "ev3rstorm.json"


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class SequenceReader:
    def __init__(self, values):
        self.values = list(values)
        self.index = 0

    def __call__(self):
        if self.index < len(self.values):
            value = self.values[self.index]
            self.index += 1
            return value
        return self.values[-1]


class RecordingSpeaker:
    def __init__(self):
        self.messages = []

    def __call__(self, text):
        self.messages.append(text)
        return {"status": "completed"}


def default_policy():
    return GatePolicy(
        immediate_enter_max=16,
        enter_max=35,
        exit_min=40,
        median_window=3,
        enter_consecutive=2,
        exit_consecutive=3,
    )


def run_probe(
    values,
    phase_timeout_seconds=2.0,
    sample_hz=20,
):
    clock = FakeClock()
    speaker = RecordingSpeaker()
    probe = IRGateProbe(
        policy=default_policy(),
        read_value=SequenceReader(values),
        speaker=speaker,
        sample_hz=sample_hz,
        phase_timeout_seconds=phase_timeout_seconds,
        monotonic_fn=clock.monotonic,
        sleep_fn=clock.sleep,
    )
    return probe.run(), speaker


class IRGateProbeTests(unittest.TestCase):
    def test_direct_script_entrypoint_starts(self):
        completed = subprocess.run(
            [
                sys.executable,
                str(PROJECT_ROOT / "ev3" / "ir_gate_probe.py"),
                "--help",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("motor-free", completed.stdout.lower())

    def test_config_policy_matches_physical_probe_defaults(self):
        with CONFIG_PATH.open("r", encoding="utf-8") as handle:
            config = json.load(handle)

        policy = GatePolicy.from_config(config)

        self.assertEqual(policy.to_dict(), default_policy().to_dict())

    def test_two_near_and_three_release_decisions_complete(self):
        values = [
            52,
            52,
            40,
            36,
            35,
            35,
            35,
            34,
            36,
            39,
            40,
            40,
            40,
            41,
        ]

        result, speaker = run_probe(values)

        self.assertEqual(result["status"], "completed")
        self.assertEqual(
            result["entry"]["reason"],
            "stable_filtered_near_returns",
        )
        self.assertEqual(result["entry"]["raw"], 35)
        self.assertEqual(result["entry"]["filtered"], 35)
        self.assertEqual(
            result["exit"]["reason"],
            "stable_filtered_release_returns",
        )
        self.assertEqual(result["exit"]["raw"], 41)
        self.assertEqual(result["exit"]["filtered"], 40)
        self.assertEqual(
            speaker.messages,
            [APPROACH_PROMPT, RETREAT_PROMPT, COMPLETE_PROMPT],
        )

    def test_strong_raw_return_enters_immediately(self):
        result, _ = run_probe([52, 13, 13, 40, 40, 40, 40])

        self.assertEqual(result["status"], "completed")
        self.assertEqual(
            result["entry"]["reason"],
            "immediate_strong_return",
        )
        self.assertEqual(result["entry"]["raw"], 13)

    def test_entry_timeout_is_explicit_and_never_runs_retreat(self):
        result, speaker = run_probe(
            [52],
            phase_timeout_seconds=0.2,
        )

        self.assertEqual(result["status"], "entry_timeout")
        self.assertIsNone(result["entry"])
        self.assertIsNone(result["exit"])
        self.assertEqual(
            speaker.messages,
            [APPROACH_PROMPT, ENTRY_TIMEOUT_PROMPT],
        )

    def test_exit_timeout_is_explicit(self):
        result, speaker = run_probe(
            [13, 13, 13, 13],
            phase_timeout_seconds=0.2,
        )

        self.assertEqual(result["status"], "exit_timeout")
        self.assertIsNotNone(result["entry"])
        self.assertIsNone(result["exit"])
        self.assertEqual(
            speaker.messages,
            [
                APPROACH_PROMPT,
                RETREAT_PROMPT,
                EXIT_TIMEOUT_PROMPT,
            ],
        )

    def test_invalid_sensor_values_are_fail_closed(self):
        for value in [True, -1, 101, 1.5]:
            with self.subTest(value=value):
                clock = FakeClock()
                probe = IRGateProbe(
                    policy=default_policy(),
                    read_value=SequenceReader([value]),
                    speaker=RecordingSpeaker(),
                    phase_timeout_seconds=0.2,
                    monotonic_fn=clock.monotonic,
                    sleep_fn=clock.sleep,
                )
                with self.assertRaises(SafetyError):
                    probe.run()

    def test_invalid_policy_bools_and_even_window_are_rejected(self):
        invalid_values = [
            dict(
                immediate_enter_max=True,
                enter_max=35,
                exit_min=40,
                median_window=3,
                enter_consecutive=2,
                exit_consecutive=3,
            ),
            dict(
                immediate_enter_max=16,
                enter_max=35,
                exit_min=40,
                median_window=4,
                enter_consecutive=2,
                exit_consecutive=3,
            ),
        ]
        for values in invalid_values:
            with self.subTest(values=values):
                with self.assertRaises(SafetyError):
                    GatePolicy(**values)

    def test_sample_rate_and_timeout_validation_reject_bools(self):
        dependencies = {
            "policy": default_policy(),
            "read_value": SequenceReader([52]),
            "speaker": RecordingSpeaker(),
        }
        with self.assertRaises(SafetyError):
            IRGateProbe(sample_hz=True, **dependencies)
        with self.assertRaises(SafetyError):
            IRGateProbe(
                phase_timeout_seconds=True,
                **dependencies
            )

    def test_audit_is_json_serializable_and_has_sample_schema(self):
        result, _ = run_probe(
            [13, 13, 40, 40, 40, 40],
        )

        encoded = json.dumps(result, sort_keys=True)

        self.assertIn('"sample_count"', encoded)
        self.assertEqual(
            result["sample_columns"],
            [
                "elapsed_ms",
                "phase_0_approach_1_retreat",
                "raw",
                "filtered",
            ],
        )
        self.assertEqual(result["requested_hz"], 20)
        self.assertGreater(result["sample_count"], 0)


if __name__ == "__main__":
    unittest.main()
