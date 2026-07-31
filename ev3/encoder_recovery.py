#!/usr/bin/env python3
"""Pure, Python 3.5-compatible policy for bounded encoder recovery.

The policy interprets signed encoder evidence after a paired drive command.
It never reads hardware, starts motors, or exposes a new agent action.  A
motor-owning caller may use its JSON-safe decision to perform one bounded
internal recovery step and then call the policy again with fresh cumulative
encoder evidence.
"""

from __future__ import print_function


DECISION_NO_RECOVERY = "no_recovery"
DECISION_CATCH_UP = "catch_up_lagging_side"
DECISION_RETRY_PAIR = "retry_pair"
DECISION_ABORT = "abort"

REASON_COMMAND_SATISFIED = "command_satisfied"
REASON_LEFT_LAGGING = "left_lagging"
REASON_RIGHT_LAGGING = "right_lagging"
REASON_PAIRED_UNDERTRAVEL = "paired_undertravel"
REASON_ENCODER_DIRECTION_MISMATCH = "encoder_direction_mismatch"
REASON_TOTAL_ATTEMPT_BUDGET = "total_attempt_budget_exhausted"
REASON_CATCH_UP_ATTEMPT_BUDGET = "catch_up_attempt_budget_exhausted"
REASON_PAIR_RETRY_ATTEMPT_BUDGET = (
    "pair_retry_attempt_budget_exhausted"
)
REASON_DURATION_BUDGET = "recovery_duration_budget_exhausted"
REASON_ENCODER_BUDGET = "recovery_encoder_budget_exhausted"


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _integer(value, name, minimum=0, maximum=None):
    if not _is_int(value) or value < minimum:
        raise ValueError(
            "{} must be an integer greater than or equal to {}".format(
                name, minimum
            )
        )
    if maximum is not None and value > maximum:
        raise ValueError(
            "{} must be at most {}".format(name, maximum)
        )
    return value


def _ceil_div(numerator, denominator):
    return (numerator + denominator - 1) // denominator


class EncoderRecoveryBudget(object):
    """Already consumed recovery resources, immutable by convention."""

    __slots__ = (
        "catch_up_attempts",
        "pair_retry_attempts",
        "duration_ms",
        "encoder_degrees",
    )

    def __init__(
        self,
        catch_up_attempts=0,
        pair_retry_attempts=0,
        duration_ms=0,
        encoder_degrees=0,
    ):
        self.catch_up_attempts = _integer(
            catch_up_attempts, "catch_up_attempts"
        )
        self.pair_retry_attempts = _integer(
            pair_retry_attempts, "pair_retry_attempts"
        )
        self.duration_ms = _integer(duration_ms, "duration_ms")
        self.encoder_degrees = _integer(
            encoder_degrees, "encoder_degrees"
        )

    @property
    def total_attempts(self):
        return self.catch_up_attempts + self.pair_retry_attempts

    def to_dict(self):
        return {
            "catch_up_attempts": self.catch_up_attempts,
            "pair_retry_attempts": self.pair_retry_attempts,
            "total_attempts": self.total_attempts,
            "duration_ms": self.duration_ms,
            "encoder_degrees": self.encoder_degrees,
        }

    def consume(self, decision, duration_ms, encoder_degrees):
        if decision == DECISION_CATCH_UP:
            catch_up_attempts = self.catch_up_attempts + 1
            pair_retry_attempts = self.pair_retry_attempts
        elif decision == DECISION_RETRY_PAIR:
            catch_up_attempts = self.catch_up_attempts
            pair_retry_attempts = self.pair_retry_attempts + 1
        else:
            raise ValueError("Only executable recovery decisions consume budget")
        return EncoderRecoveryBudget(
            catch_up_attempts=catch_up_attempts,
            pair_retry_attempts=pair_retry_attempts,
            duration_ms=self.duration_ms + duration_ms,
            encoder_degrees=self.encoder_degrees + encoder_degrees,
        )


class EncoderRecoveryPolicy(object):
    """Deterministically choose at most one bounded recovery step.

    Expected deltas use the physical encoder sign expected from the original
    command.  Observed deltas must use that same coordinate system.  Recovery
    instructions reuse the original per-side speed and provide only a side or
    pair selection, duration, and expected encoder amount.
    """

    __slots__ = (
        "minimum_progress_degrees",
        "catch_up_leader_minimum_degrees",
        "acceptable_completion_percent",
        "maximum_progress_skew_percent",
        "maximum_catch_up_attempts",
        "maximum_pair_retry_attempts",
        "maximum_total_attempts",
        "maximum_step_duration_ms",
        "maximum_total_recovery_duration_ms",
        "maximum_total_recovery_encoder_degrees",
    )

    def __init__(
        self,
        minimum_progress_degrees,
        catch_up_leader_minimum_degrees,
        acceptable_completion_percent,
        maximum_progress_skew_percent,
        maximum_catch_up_attempts,
        maximum_pair_retry_attempts,
        maximum_total_attempts,
        maximum_step_duration_ms,
        maximum_total_recovery_duration_ms,
        maximum_total_recovery_encoder_degrees,
    ):
        self.minimum_progress_degrees = _integer(
            minimum_progress_degrees,
            "minimum_progress_degrees",
            minimum=1,
        )
        self.catch_up_leader_minimum_degrees = _integer(
            catch_up_leader_minimum_degrees,
            "catch_up_leader_minimum_degrees",
            minimum=self.minimum_progress_degrees,
        )
        self.acceptable_completion_percent = _integer(
            acceptable_completion_percent,
            "acceptable_completion_percent",
            minimum=1,
            maximum=100,
        )
        self.maximum_progress_skew_percent = _integer(
            maximum_progress_skew_percent,
            "maximum_progress_skew_percent",
            maximum=100,
        )
        self.maximum_catch_up_attempts = _integer(
            maximum_catch_up_attempts,
            "maximum_catch_up_attempts",
        )
        self.maximum_pair_retry_attempts = _integer(
            maximum_pair_retry_attempts,
            "maximum_pair_retry_attempts",
        )
        self.maximum_total_attempts = _integer(
            maximum_total_attempts,
            "maximum_total_attempts",
        )
        if self.maximum_total_attempts > (
            self.maximum_catch_up_attempts
            + self.maximum_pair_retry_attempts
        ):
            raise ValueError(
                "maximum_total_attempts cannot exceed the sum of the "
                "per-decision attempt budgets"
            )
        self.maximum_step_duration_ms = _integer(
            maximum_step_duration_ms,
            "maximum_step_duration_ms",
            minimum=1,
        )
        self.maximum_total_recovery_duration_ms = _integer(
            maximum_total_recovery_duration_ms,
            "maximum_total_recovery_duration_ms",
            minimum=1,
        )
        self.maximum_total_recovery_encoder_degrees = _integer(
            maximum_total_recovery_encoder_degrees,
            "maximum_total_recovery_encoder_degrees",
            minimum=1,
        )

    def to_dict(self):
        return {
            "minimum_progress_degrees": self.minimum_progress_degrees,
            "catch_up_leader_minimum_degrees": (
                self.catch_up_leader_minimum_degrees
            ),
            "acceptable_completion_percent": (
                self.acceptable_completion_percent
            ),
            "maximum_progress_skew_percent": (
                self.maximum_progress_skew_percent
            ),
            "maximum_catch_up_attempts": self.maximum_catch_up_attempts,
            "maximum_pair_retry_attempts": (
                self.maximum_pair_retry_attempts
            ),
            "maximum_total_attempts": self.maximum_total_attempts,
            "maximum_step_duration_ms": self.maximum_step_duration_ms,
            "maximum_total_recovery_duration_ms": (
                self.maximum_total_recovery_duration_ms
            ),
            "maximum_total_recovery_encoder_degrees": (
                self.maximum_total_recovery_encoder_degrees
            ),
        }

    @staticmethod
    def _expected_for_duration(
        expected_degrees, command_duration_ms, recovery_duration_ms
    ):
        return _ceil_div(
            expected_degrees * recovery_duration_ms,
            command_duration_ms,
        )

    def _encoder_cost(
        self,
        mode,
        side,
        expected,
        command_duration_ms,
        recovery_duration_ms,
    ):
        if recovery_duration_ms <= 0:
            return 0
        if mode == "single_wheel":
            return self._expected_for_duration(
                expected[side],
                command_duration_ms,
                recovery_duration_ms,
            )
        return sum(
            self._expected_for_duration(
                expected[name],
                command_duration_ms,
                recovery_duration_ms,
            )
            for name in ("left", "right")
        )

    def _duration_within_encoder_budget(
        self,
        mode,
        side,
        expected,
        command_duration_ms,
        desired_duration_ms,
        available_encoder_degrees,
    ):
        low = 0
        high = desired_duration_ms
        while low < high:
            middle = (low + high + 1) // 2
            cost = self._encoder_cost(
                mode,
                side,
                expected,
                command_duration_ms,
                middle,
            )
            if cost <= available_encoder_degrees:
                low = middle
            else:
                high = middle - 1
        return low

    @staticmethod
    def _base_result(
        decision,
        reason,
        expected,
        observed,
        completion,
        budget,
    ):
        budget_snapshot = budget.to_dict()
        return {
            "decision": decision,
            "reason": reason,
            "expected_abs_encoder_degrees": dict(expected),
            "observed_abs_encoder_degrees": dict(observed),
            "completion_percent": dict(completion),
            "instruction": None,
            "budget_before": budget_snapshot,
            "budget_after_if_executed": dict(budget_snapshot),
        }

    def _abort(
        self, reason, expected, observed, completion, budget
    ):
        return self._base_result(
            DECISION_ABORT,
            reason,
            expected,
            observed,
            completion,
            budget,
        )

    def _instruction(
        self,
        decision,
        reason,
        mode,
        side,
        desired_duration_ms,
        expected,
        observed,
        completion,
        command_duration_ms,
        budget,
    ):
        if budget.total_attempts >= self.maximum_total_attempts:
            return self._abort(
                REASON_TOTAL_ATTEMPT_BUDGET,
                expected,
                observed,
                completion,
                budget,
            )
        if (
            decision == DECISION_CATCH_UP
            and budget.catch_up_attempts
            >= self.maximum_catch_up_attempts
        ):
            return self._abort(
                REASON_CATCH_UP_ATTEMPT_BUDGET,
                expected,
                observed,
                completion,
                budget,
            )
        if (
            decision == DECISION_RETRY_PAIR
            and budget.pair_retry_attempts
            >= self.maximum_pair_retry_attempts
        ):
            return self._abort(
                REASON_PAIR_RETRY_ATTEMPT_BUDGET,
                expected,
                observed,
                completion,
                budget,
            )

        available_duration_ms = (
            self.maximum_total_recovery_duration_ms - budget.duration_ms
        )
        if available_duration_ms <= 0:
            return self._abort(
                REASON_DURATION_BUDGET,
                expected,
                observed,
                completion,
                budget,
            )
        available_encoder_degrees = (
            self.maximum_total_recovery_encoder_degrees
            - budget.encoder_degrees
        )
        if available_encoder_degrees <= 0:
            return self._abort(
                REASON_ENCODER_BUDGET,
                expected,
                observed,
                completion,
                budget,
            )

        commanded_sides = (
            (side,) if mode == "single_wheel" else ("left", "right")
        )
        minimum_verifiable_duration_ms = max(
            _ceil_div(
                self.minimum_progress_degrees * command_duration_ms,
                expected[name],
            )
            for name in commanded_sides
        )
        duration_ms = min(
            max(desired_duration_ms, minimum_verifiable_duration_ms),
            self.maximum_step_duration_ms,
            available_duration_ms,
        )
        duration_ms = self._duration_within_encoder_budget(
            mode,
            side,
            expected,
            command_duration_ms,
            duration_ms,
            available_encoder_degrees,
        )
        if duration_ms < minimum_verifiable_duration_ms:
            return self._abort(
                REASON_ENCODER_BUDGET,
                expected,
                observed,
                completion,
                budget,
            )

        target = {"left": 0, "right": 0}
        if mode == "single_wheel":
            target[side] = self._expected_for_duration(
                expected[side], command_duration_ms, duration_ms
            )
        else:
            for name in ("left", "right"):
                target[name] = self._expected_for_duration(
                    expected[name], command_duration_ms, duration_ms
                )
        encoder_cost = target["left"] + target["right"]
        after = budget.consume(decision, duration_ms, encoder_cost)
        result = self._base_result(
            decision,
            reason,
            expected,
            observed,
            completion,
            budget,
        )
        result["instruction"] = {
            "mode": mode,
            "side": side,
            "reuse_original_speed": True,
            "duration_ms": duration_ms,
            "target_abs_encoder_degrees": target,
            "bounded": duration_ms != desired_duration_ms,
        }
        result["budget_after_if_executed"] = after.to_dict()
        return result

    def decide(
        self,
        expected_left_delta_degrees,
        expected_right_delta_degrees,
        observed_left_delta_degrees,
        observed_right_delta_degrees,
        command_duration_ms,
        budget,
    ):
        """Return one JSON-safe decision without mutating ``budget``."""
        expected_signed = {
            "left": _integer(
                abs(expected_left_delta_degrees),
                "expected_left_delta_degrees magnitude",
                minimum=self.minimum_progress_degrees,
            ),
            "right": _integer(
                abs(expected_right_delta_degrees),
                "expected_right_delta_degrees magnitude",
                minimum=self.minimum_progress_degrees,
            ),
        }
        for name, value in (
            ("expected_left_delta_degrees", expected_left_delta_degrees),
            ("expected_right_delta_degrees", expected_right_delta_degrees),
            ("observed_left_delta_degrees", observed_left_delta_degrees),
            ("observed_right_delta_degrees", observed_right_delta_degrees),
        ):
            if not _is_int(value):
                raise ValueError("{} must be an integer".format(name))
        command_duration_ms = _integer(
            command_duration_ms,
            "command_duration_ms",
            minimum=1,
        )
        if not isinstance(budget, EncoderRecoveryBudget):
            raise ValueError("budget must be an EncoderRecoveryBudget")

        observed_signed = {
            "left": observed_left_delta_degrees,
            "right": observed_right_delta_degrees,
        }
        expected_directions = {
            "left": 1 if expected_left_delta_degrees > 0 else -1,
            "right": 1 if expected_right_delta_degrees > 0 else -1,
        }
        observed = {
            "left": abs(observed_left_delta_degrees),
            "right": abs(observed_right_delta_degrees),
        }
        completion = {
            name: observed[name] * 100 // expected_signed[name]
            for name in ("left", "right")
        }

        for name in ("left", "right"):
            if (
                observed_signed[name] != 0
                and observed_signed[name] * expected_directions[name] < 0
            ):
                return self._abort(
                    REASON_ENCODER_DIRECTION_MISMATCH,
                    expected_signed,
                    observed,
                    completion,
                    budget,
                )

        skew = abs(completion["left"] - completion["right"])
        completion_ok = all(
            completion[name] >= self.acceptable_completion_percent
            for name in ("left", "right")
        )
        if completion_ok and skew <= self.maximum_progress_skew_percent:
            return self._base_result(
                DECISION_NO_RECOVERY,
                REASON_COMMAND_SATISFIED,
                expected_signed,
                observed,
                completion,
                budget,
            )

        left_started = observed["left"] >= self.minimum_progress_degrees
        right_started = observed["right"] >= self.minimum_progress_degrees
        if left_started != right_started:
            leader = "left" if left_started else "right"
            if observed[leader] >= self.catch_up_leader_minimum_degrees:
                lagging = "right" if leader == "left" else "left"
            else:
                lagging = None
        elif left_started and right_started and skew > (
            self.maximum_progress_skew_percent
        ):
            lagging = (
                "left"
                if completion["left"] < completion["right"]
                else "right"
            )
            leader = "right" if lagging == "left" else "left"
        else:
            lagging = None

        if lagging is not None:
            desired_lagging_degrees = _ceil_div(
                observed[leader] * expected_signed[lagging],
                expected_signed[leader],
            )
            deficit = max(
                1, desired_lagging_degrees - observed[lagging]
            )
            desired_duration_ms = _ceil_div(
                deficit * command_duration_ms,
                expected_signed[lagging],
            )
            reason = (
                REASON_LEFT_LAGGING
                if lagging == "left"
                else REASON_RIGHT_LAGGING
            )
            return self._instruction(
                DECISION_CATCH_UP,
                reason,
                "single_wheel",
                lagging,
                desired_duration_ms,
                expected_signed,
                observed,
                completion,
                command_duration_ms,
                budget,
            )

        desired = {
            name: _ceil_div(
                expected_signed[name]
                * self.acceptable_completion_percent,
                100,
            )
            for name in ("left", "right")
        }
        remaining = {
            name: max(0, desired[name] - observed[name])
            for name in ("left", "right")
        }
        desired_duration_ms = max(
            _ceil_div(
                remaining[name] * command_duration_ms,
                expected_signed[name],
            )
            for name in ("left", "right")
        )
        if desired_duration_ms <= 0:
            return self._base_result(
                DECISION_NO_RECOVERY,
                REASON_COMMAND_SATISFIED,
                expected_signed,
                observed,
                completion,
                budget,
            )
        return self._instruction(
            DECISION_RETRY_PAIR,
            REASON_PAIRED_UNDERTRAVEL,
            "paired",
            None,
            desired_duration_ms,
            expected_signed,
            observed,
            completion,
            command_duration_ms,
            budget,
        )
