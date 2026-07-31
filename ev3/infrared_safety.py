#!/usr/bin/env python3
"""Python 3.5-compatible, stateful infrared obstacle safety gate."""

from __future__ import print_function

from collections import deque
from statistics import median

if __package__:
    from .robot_config import (
        MAX_IR_CONSECUTIVE_DECISIONS,
        MAX_IR_MEDIAN_WINDOW,
    )
    from .robot_hal import SafetyError
else:
    from robot_config import (
        MAX_IR_CONSECUTIVE_DECISIONS,
        MAX_IR_MEDIAN_WINDOW,
    )
    from robot_hal import SafetyError


REASON_UNVERIFIED_STARTUP = "unverified_startup"
REASON_INVALID_SAMPLE = "invalid_sample"
REASON_WARMING_UP = "warming_up"
REASON_IMMEDIATE_ENTRY = "immediate_strong_return"
REASON_STABLE_ENTRY = "stable_filtered_near_returns"
REASON_STABLE_EXIT = "stable_filtered_release_returns"
REASON_ENTRY_PENDING = "stable_entry_pending"
REASON_EXIT_PENDING = "stable_exit_pending"
REASON_BLOCKED_HOLD = "blocked_hysteresis_hold"
REASON_CLEAR_HOLD = "clear_hysteresis_hold"


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


class InfraredGatePolicy(object):
    """Strict, bounded policy for interpreting EV3 IR-PROX samples."""

    __slots__ = (
        "immediate_enter_max",
        "enter_max",
        "exit_min",
        "median_window",
        "enter_consecutive",
        "exit_consecutive",
    )

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
        if (
            median_window <= 0
            or median_window > MAX_IR_MEDIAN_WINDOW
            or median_window % 2 == 0
        ):
            raise SafetyError(
                "IR gate median window must be a positive odd integer "
                "no greater than {0}".format(MAX_IR_MEDIAN_WINDOW)
            )
        if (
            enter_consecutive <= 0
            or enter_consecutive > MAX_IR_CONSECUTIVE_DECISIONS
            or exit_consecutive <= 0
            or exit_consecutive > MAX_IR_CONSECUTIVE_DECISIONS
        ):
            raise SafetyError(
                "IR gate consecutive counts must be between 1 and "
                "{0}".format(MAX_IR_CONSECUTIVE_DECISIONS)
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


class InfraredObstacleGate(object):
    """Fail-closed IR gate with fast entry and conservative release.

    Startup is blocked until stable higher readings establish release. Each
    observation returns a fixed-size dictionary rather than retaining or
    publishing an unbounded sample history.
    """

    def __init__(self, policy):
        if not isinstance(policy, InfraredGatePolicy):
            raise SafetyError("IR gate policy is invalid")
        self.policy = policy
        self._samples = deque(maxlen=policy.median_window)
        self._raw = None
        self._filtered = None
        self._blocked = True
        self._reason = REASON_UNVERIFIED_STARTUP
        self._sample_count = 0
        self._candidate_blocked = None
        self._candidate_count = 0

    @property
    def blocked(self):
        return self._blocked

    def _reset_candidate(self):
        self._candidate_blocked = None
        self._candidate_count = 0

    def _set_candidate(self, desired_blocked):
        if desired_blocked == self._candidate_blocked:
            self._candidate_count += 1
        else:
            self._candidate_blocked = desired_blocked
            self._candidate_count = 1

    def _fail_closed(self):
        self._samples.clear()
        self._raw = None
        self._filtered = None
        self._blocked = True
        self._reason = REASON_INVALID_SAMPLE
        self._reset_candidate()

    def snapshot(self):
        """Return a bounded, JSON-safe view of the current gate state."""
        return {
            "raw": self._raw,
            "filtered": self._filtered,
            "blocked": self._blocked,
            "reason": self._reason,
            "sample_count": self._sample_count,
        }

    def fail_closed(self):
        """Invalidate dynamic evidence after a sensor or topology failure."""
        self._fail_closed()
        return self.snapshot()

    def observe(self, raw):
        """Consume one IR-PROX sample and return the current gate snapshot."""
        if not _is_int(raw) or raw < 0 or raw > 100:
            self._fail_closed()
            raise SafetyError(
                "IR sample must be an integer in 0..100"
            )

        self._raw = raw
        self._sample_count += 1
        self._samples.append(raw)
        self._filtered = int(round(median(self._samples)))

        if raw <= self.policy.immediate_enter_max:
            self._blocked = True
            self._reason = REASON_IMMEDIATE_ENTRY
            self._reset_candidate()
            return self.snapshot()

        if len(self._samples) < self.policy.median_window:
            self._reason = REASON_WARMING_UP
            self._reset_candidate()
            return self.snapshot()

        if self._filtered <= self.policy.enter_max:
            desired_blocked = True
            required = self.policy.enter_consecutive
            pending_reason = REASON_ENTRY_PENDING
        elif self._filtered >= self.policy.exit_min:
            desired_blocked = False
            required = self.policy.exit_consecutive
            pending_reason = REASON_EXIT_PENDING
        else:
            self._reason = (
                REASON_BLOCKED_HOLD
                if self._blocked
                else REASON_CLEAR_HOLD
            )
            self._reset_candidate()
            return self.snapshot()

        if desired_blocked == self._blocked:
            self._reason = (
                REASON_BLOCKED_HOLD
                if self._blocked
                else REASON_CLEAR_HOLD
            )
            self._reset_candidate()
            return self.snapshot()

        self._set_candidate(desired_blocked)
        if self._candidate_count < required:
            self._reason = pending_reason
            return self.snapshot()

        self._blocked = desired_blocked
        self._reason = (
            REASON_STABLE_ENTRY
            if desired_blocked
            else REASON_STABLE_EXIT
        )
        self._reset_candidate()
        return self.snapshot()
