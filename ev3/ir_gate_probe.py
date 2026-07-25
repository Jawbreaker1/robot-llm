#!/usr/bin/env python3
"""Python 3.5-compatible, motor-free dynamic IR gate probe."""

from __future__ import print_function

import argparse
from collections import deque
import json
import os
from statistics import median
import sys
import time

try:
    from .robot_hal import RobotHAL, SafetyError, read_text
except (ImportError, ValueError, SystemError):
    from robot_hal import RobotHAL, SafetyError, read_text


APPROACH_PROMPT = (
    "För lådan närmare tills jag säger registrerat."
)
RETREAT_PROMPT = (
    "Registrerat. För lådan bort tills jag säger klart."
)
ENTRY_TIMEOUT_PROMPT = "Ingen tydlig träff. Testet avbryts."
EXIT_TIMEOUT_PROMPT = "Ingen tydlig frigivning. Testet avbryts."
COMPLETE_PROMPT = "Klart"


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


class GatePolicy(object):
    """Validated copy of the checked-in provisional evidence policy."""

    def __init__(
        self,
        immediate_enter_max,
        enter_max,
        exit_min,
        median_window,
        enter_consecutive,
        exit_consecutive,
    ):
        values = (
            immediate_enter_max,
            enter_max,
            exit_min,
            median_window,
            enter_consecutive,
            exit_consecutive,
        )
        if not all(_is_int(value) for value in values):
            raise SafetyError("IR gate policy values must be integers")
        if not (
            0
            <= immediate_enter_max
            <= enter_max
            < exit_min
            <= 100
        ):
            raise SafetyError("IR gate thresholds are invalid")
        if median_window <= 0 or median_window % 2 == 0:
            raise SafetyError(
                "IR gate median window must be a positive odd integer"
            )
        if enter_consecutive <= 0 or exit_consecutive <= 0:
            raise SafetyError(
                "IR gate consecutive counts must be positive"
            )

        self.immediate_enter_max = immediate_enter_max
        self.enter_max = enter_max
        self.exit_min = exit_min
        self.median_window = median_window
        self.enter_consecutive = enter_consecutive
        self.exit_consecutive = exit_consecutive

    @classmethod
    def from_config(cls, config):
        try:
            values = config["calibration"]["infrared_proximity"][
                "obstacle_gate"
            ]
            return cls(
                values["immediate_enter_max"],
                values["enter_max"],
                values["exit_min"],
                values["median_window"],
                values["enter_consecutive"],
                values["exit_consecutive"],
            )
        except (KeyError, TypeError):
            raise SafetyError(
                "Configuration has no valid IR obstacle gate"
            )

    def to_dict(self):
        return {
            "immediate_enter_max": self.immediate_enter_max,
            "enter_max": self.enter_max,
            "exit_min": self.exit_min,
            "median_window": self.median_window,
            "enter_consecutive": self.enter_consecutive,
            "exit_consecutive": self.exit_consecutive,
        }


class IRGateProbe(object):
    """Guide one human-driven approach/retreat cycle with no motor calls."""

    def __init__(
        self,
        policy,
        read_value,
        speaker,
        sample_hz=20,
        phase_timeout_seconds=30,
        monotonic_fn=time.monotonic,
        sleep_fn=time.sleep,
    ):
        if not isinstance(policy, GatePolicy):
            raise SafetyError("IR gate policy is invalid")
        if not callable(read_value) or not callable(speaker):
            raise SafetyError("IR probe dependencies are invalid")
        if not callable(monotonic_fn) or not callable(sleep_fn):
            raise SafetyError("IR probe clock dependencies are invalid")
        if (
            not _is_int(sample_hz)
            or sample_hz <= 0
            or sample_hz > 50
        ):
            raise SafetyError("sample_hz must be an integer in 1..50")
        if (
            isinstance(phase_timeout_seconds, bool)
            or not isinstance(phase_timeout_seconds, (int, float))
            or phase_timeout_seconds <= 0
            or phase_timeout_seconds > 120
        ):
            raise SafetyError(
                "phase_timeout_seconds must be in (0, 120]"
            )

        self.policy = policy
        self.read_value = read_value
        self.speaker = speaker
        self.sample_hz = sample_hz
        self.phase_timeout_seconds = phase_timeout_seconds
        self.monotonic_fn = monotonic_fn
        self.sleep_fn = sleep_fn

    def _read(self):
        value = self.read_value()
        if not _is_int(value) or value < 0 or value > 100:
            raise SafetyError("IR sample must be an integer in 0..100")
        return value

    def _append_sample(
        self,
        samples,
        experiment_started,
        phase,
        window,
    ):
        observed = self.monotonic_fn()
        raw = self._read()
        window.append(raw)
        filtered = int(round(median(window)))
        elapsed_ms = int(
            round((observed - experiment_started) * 1000)
        )
        samples.append([elapsed_ms, phase, raw, filtered])
        return raw, filtered, elapsed_ms

    def _sleep_until(self, deadline):
        remaining = deadline - self.monotonic_fn()
        if remaining > 0:
            self.sleep_fn(remaining)

    def _await_entry(self, samples, experiment_started, window):
        hits = 0
        phase_started = self.monotonic_fn()
        next_sample = phase_started
        interval = 1.0 / self.sample_hz

        while (
            self.monotonic_fn() - phase_started
            < self.phase_timeout_seconds
        ):
            raw, filtered, elapsed_ms = self._append_sample(
                samples,
                experiment_started,
                0,
                window,
            )
            if raw <= self.policy.immediate_enter_max:
                return {
                    "elapsed_ms": elapsed_ms,
                    "raw": raw,
                    "filtered": filtered,
                    "reason": "immediate_strong_return",
                }

            if (
                len(window) == self.policy.median_window
                and filtered <= self.policy.enter_max
            ):
                hits += 1
            else:
                hits = 0

            if hits >= self.policy.enter_consecutive:
                return {
                    "elapsed_ms": elapsed_ms,
                    "raw": raw,
                    "filtered": filtered,
                    "reason": "stable_filtered_near_returns",
                }

            next_sample += interval
            self._sleep_until(next_sample)
        return None

    def _await_exit(self, samples, experiment_started, window):
        hits = 0
        phase_started = self.monotonic_fn()
        next_sample = phase_started
        interval = 1.0 / self.sample_hz

        while (
            self.monotonic_fn() - phase_started
            < self.phase_timeout_seconds
        ):
            raw, filtered, elapsed_ms = self._append_sample(
                samples,
                experiment_started,
                1,
                window,
            )
            if (
                len(window) == self.policy.median_window
                and filtered >= self.policy.exit_min
            ):
                hits += 1
            else:
                hits = 0

            if hits >= self.policy.exit_consecutive:
                return {
                    "elapsed_ms": elapsed_ms,
                    "raw": raw,
                    "filtered": filtered,
                    "reason": "stable_filtered_release_returns",
                }

            next_sample += interval
            self._sleep_until(next_sample)
        return None

    def _result(self, status, samples, entry, exit_event):
        raw_values = [sample[2] for sample in samples]
        return {
            "status": status,
            "requested_hz": self.sample_hz,
            "phase_timeout_seconds": self.phase_timeout_seconds,
            "policy": self.policy.to_dict(),
            "sample_columns": [
                "elapsed_ms",
                "phase_0_approach_1_retreat",
                "raw",
                "filtered",
            ],
            "sample_count": len(samples),
            "minimum_raw": min(raw_values) if raw_values else None,
            "maximum_raw": max(raw_values) if raw_values else None,
            "entry": entry,
            "exit": exit_event,
            "samples": samples,
        }

    def run(self):
        samples = []
        window = deque(maxlen=self.policy.median_window)
        experiment_started = self.monotonic_fn()

        self.speaker(APPROACH_PROMPT)
        entry = self._await_entry(
            samples,
            experiment_started,
            window,
        )
        if entry is None:
            self.speaker(ENTRY_TIMEOUT_PROMPT)
            return self._result(
                "entry_timeout",
                samples,
                None,
                None,
            )

        self.speaker(RETREAT_PROMPT)
        exit_event = self._await_exit(
            samples,
            experiment_started,
            window,
        )
        if exit_event is None:
            self.speaker(EXIT_TIMEOUT_PROMPT)
            return self._result(
                "exit_timeout",
                samples,
                entry,
                None,
            )

        self.speaker(COMPLETE_PROMPT)
        return self._result(
            "completed",
            samples,
            entry,
            exit_event,
        )


def default_config_path():
    return os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "ev3rstorm.json",
        )
    )


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Motor-free, voice-guided dynamic IR evidence-gate probe."
        )
    )
    parser.add_argument("--config", default=default_config_path())
    parser.add_argument("--sample-hz", default=20, type=int)
    parser.add_argument(
        "--phase-timeout-seconds",
        default=30,
        type=int,
    )
    parser.add_argument("--no-speech", action="store_true")
    return parser


def main():
    args = build_parser().parse_args()
    robot = RobotHAL(args.config)
    policy = GatePolicy.from_config(robot.config)
    sensor_path = robot._sensor_path_for_role("infrared")
    actual_mode = read_text(os.path.join(sensor_path, "mode"))
    if actual_mode != "IR-PROX":
        raise SafetyError(
            "Infrared sensor must remain in IR-PROX mode"
        )
    value_path = os.path.join(sensor_path, "value0")

    def read_value():
        return int(read_text(value_path))

    def speaker(text):
        if args.no_speech:
            return {"status": "disabled"}
        return robot.speak(text)

    probe = IRGateProbe(
        policy=policy,
        read_value=read_value,
        speaker=speaker,
        sample_hz=args.sample_hz,
        phase_timeout_seconds=args.phase_timeout_seconds,
    )
    print(
        json.dumps(
            probe.run(),
            sort_keys=True,
            separators=(",", ":"),
        )
    )


if __name__ == "__main__":
    try:
        main()
    except (SafetyError, IOError, OSError, RuntimeError) as error:
        print(
            json.dumps(
                {
                    "status": "failed",
                    "error": str(error),
                },
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        sys.exit(1)
