#!/usr/bin/env python3
"""Python 3.5-compatible execution of bounded EV3 encoder recovery.

The pure policy in :mod:`encoder_recovery` decides what to try.  This module
executes those private motor corrections through callbacks supplied by the
navigation worker, records their encoder evidence, and returns one strict
slice result.  It never selects a semantic robot action.
"""

from __future__ import print_function

if __package__:
    from .encoder_recovery import (
        DECISION_ABORT,
        DECISION_CATCH_UP,
        DECISION_NO_RECOVERY,
        DECISION_RETRY_PAIR,
        REASON_ENCODER_DIRECTION_MISMATCH,
        EncoderRecoveryBudget,
        EncoderRecoveryPolicy,
    )
    from .navigation_profile import (
        ENCODER_RECOVERY_ACCEPTABLE_COMPLETION_PERCENT,
        ENCODER_RECOVERY_LEADER_MINIMUM_DEGREES,
        ENCODER_RECOVERY_MAXIMUM_CATCH_UP_ATTEMPTS,
        ENCODER_RECOVERY_MAXIMUM_PAIR_RETRY_ATTEMPTS,
        ENCODER_RECOVERY_MAXIMUM_PROGRESS_SKEW_PERCENT,
        ENCODER_RECOVERY_MAXIMUM_STEP_DURATION_MS,
        ENCODER_RECOVERY_MAXIMUM_TOTAL_ATTEMPTS,
        ENCODER_RECOVERY_MAXIMUM_TOTAL_DURATION_MS,
        ENCODER_RECOVERY_MAXIMUM_TOTAL_ENCODER_DEGREES,
        ENCODER_RECOVERY_MINIMUM_PROGRESS_DEGREES,
        MAX_PULSES,
        MAX_PULSE_DURATION_MS,
    )
    from .navigation_worker_protocol import WorkerError
else:
    from encoder_recovery import (
        DECISION_ABORT,
        DECISION_CATCH_UP,
        DECISION_NO_RECOVERY,
        DECISION_RETRY_PAIR,
        REASON_ENCODER_DIRECTION_MISMATCH,
        EncoderRecoveryBudget,
        EncoderRecoveryPolicy,
    )
    from navigation_profile import (
        ENCODER_RECOVERY_ACCEPTABLE_COMPLETION_PERCENT,
        ENCODER_RECOVERY_LEADER_MINIMUM_DEGREES,
        ENCODER_RECOVERY_MAXIMUM_CATCH_UP_ATTEMPTS,
        ENCODER_RECOVERY_MAXIMUM_PAIR_RETRY_ATTEMPTS,
        ENCODER_RECOVERY_MAXIMUM_PROGRESS_SKEW_PERCENT,
        ENCODER_RECOVERY_MAXIMUM_STEP_DURATION_MS,
        ENCODER_RECOVERY_MAXIMUM_TOTAL_ATTEMPTS,
        ENCODER_RECOVERY_MAXIMUM_TOTAL_DURATION_MS,
        ENCODER_RECOVERY_MAXIMUM_TOTAL_ENCODER_DEGREES,
        ENCODER_RECOVERY_MINIMUM_PROGRESS_DEGREES,
        MAX_PULSES,
        MAX_PULSE_DURATION_MS,
    )
    from navigation_worker_protocol import WorkerError


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def validated_start_failure_evidence(
    error,
    duration_ms,
    expected_sides,
    expected_roles,
    stop_is_verified,
):
    """Return complete cleanup evidence, or fail closed if it is absent."""
    evidence = getattr(error, "supervisor_start_evidence", None)
    if not isinstance(evidence, dict) or evidence.get("complete") is not True:
        raise WorkerError(
            "start_evidence_incomplete",
            "A failed motor start lacked complete encoder evidence",
            fatal=True,
        )
    sides = list(expected_sides)
    started_sides = evidence.get("started_sides")
    windows = evidence.get("start_write_windows")
    motors = evidence.get("motors")
    started_at_ms = evidence.get("started_at_ms")
    completed_at_ms = evidence.get("completed_at_ms")
    if (
        evidence.get("duration_ms") != duration_ms
        or not isinstance(started_sides, list)
        or len(started_sides) != len(set(started_sides))
        or any(side not in sides for side in started_sides)
        or not isinstance(windows, list)
        or len(windows) != len(started_sides)
        or not isinstance(motors, list)
        or len(motors) != len(sides)
        or not _is_int(completed_at_ms)
        or not stop_is_verified(evidence.get("stop"))
    ):
        raise WorkerError(
            "start_evidence_incomplete",
            "Failed motor-start evidence was inconsistent",
            fatal=True,
        )
    if started_sides:
        if not _is_int(started_at_ms) or completed_at_ms < started_at_ms:
            raise WorkerError(
                "start_evidence_incomplete",
                "Failed motor-start timestamps were inconsistent",
                fatal=True,
            )
    elif started_at_ms is not None:
        raise WorkerError(
            "start_evidence_incomplete",
            "A denied start reported an impossible start timestamp",
            fatal=True,
        )
    if [window.get("side") for window in windows] != started_sides:
        raise WorkerError(
            "start_evidence_incomplete",
            "Failed motor-start windows did not match started sides",
            fatal=True,
        )
    previous_end = None
    for window in windows:
        side = window.get("side")
        begin_ms = window.get("begin_ms")
        end_ms = window.get("end_ms")
        if (
            window.get("role") != expected_roles[side]
            or not _is_int(begin_ms)
            or not _is_int(end_ms)
            or end_ms < begin_ms
            or (previous_end is not None and begin_ms < previous_end)
        ):
            raise WorkerError(
                "start_evidence_incomplete",
                "Failed motor-start write windows were inconsistent",
                fatal=True,
            )
        previous_end = end_ms
    by_side = {}
    for motor in motors:
        if not isinstance(motor, dict):
            by_side = {}
            break
        side = motor.get("side")
        before = motor.get("position_before")
        after = motor.get("position_after")
        delta = motor.get("position_delta")
        physical_speed = motor.get("physical_speed_dps")
        if (
            side not in sides
            or side in by_side
            or motor.get("role") != expected_roles[side]
            or not _is_int(physical_speed)
            or physical_speed == 0
            or not _is_int(before)
            or not _is_int(after)
            or not _is_int(delta)
            or after - before != delta
            or not isinstance(motor.get("state"), str)
        ):
            by_side = {}
            break
        by_side[side] = motor
    if set(by_side) != set(sides):
        raise WorkerError(
            "start_evidence_incomplete",
            "Failed motor-start encoder receipts were inconsistent",
            fatal=True,
        )
    return evidence


def partial_start_motor_receipts(evidence):
    fields = (
        "side",
        "role",
        "position_before",
        "position_after",
        "position_delta",
        "state",
    )
    return [
        dict((name, motor[name]) for name in fields)
        for motor in evidence["motors"]
    ]


def partial_start_direction_mismatch(evidence):
    return any(
        motor["position_delta"] != 0
        and motor["position_delta"] * motor["physical_speed_dps"] < 0
        for motor in evidence["motors"]
        if motor["side"] in evidence["started_sides"]
    )


def partial_start_verification(motors, started_sides):
    checks = [
        {
            "role": motor["role"],
            "side": motor["side"],
            "position_delta": motor["position_delta"],
            "passed": motor["side"] in started_sides,
        }
        for motor in motors
    ]
    checks.append(
        {
            "role": "paired_start_completion",
            "side": "paired",
            "position_delta": 0,
            "passed": False,
        }
    )
    return {
        "passed": False,
        "error": "paired motor start was incomplete",
        "checks": checks,
    }


def partial_start_segment(
    evidence,
    motors,
    duration_ms,
    segment_index,
    reason,
):
    return {
        "segment_index": segment_index,
        "kind": "partial_start",
        "commanded_sides": list(evidence["started_sides"]),
        "duration_ms": duration_ms,
        "status": "interrupted",
        "reason": reason,
        "started_monotonic_ms": evidence["started_at_ms"],
        "completed_monotonic_ms": evidence["completed_at_ms"],
        "motors": motors,
        "encoder_verification": partial_start_verification(
            motors,
            evidence["started_sides"],
        ),
        "stop": evidence["stop"],
    }


def build_encoder_recovery_policy():
    """Build the fixed recovery policy for this EV3 hardware profile."""
    return EncoderRecoveryPolicy(
        minimum_progress_degrees=(
            ENCODER_RECOVERY_MINIMUM_PROGRESS_DEGREES
        ),
        catch_up_leader_minimum_degrees=(
            ENCODER_RECOVERY_LEADER_MINIMUM_DEGREES
        ),
        acceptable_completion_percent=(
            ENCODER_RECOVERY_ACCEPTABLE_COMPLETION_PERCENT
        ),
        maximum_progress_skew_percent=(
            ENCODER_RECOVERY_MAXIMUM_PROGRESS_SKEW_PERCENT
        ),
        maximum_catch_up_attempts=(
            ENCODER_RECOVERY_MAXIMUM_CATCH_UP_ATTEMPTS
        ),
        maximum_pair_retry_attempts=(
            ENCODER_RECOVERY_MAXIMUM_PAIR_RETRY_ATTEMPTS
        ),
        maximum_total_attempts=(
            ENCODER_RECOVERY_MAXIMUM_TOTAL_ATTEMPTS
        ),
        maximum_step_duration_ms=(
            ENCODER_RECOVERY_MAXIMUM_STEP_DURATION_MS
        ),
        maximum_total_recovery_duration_ms=(
            ENCODER_RECOVERY_MAXIMUM_TOTAL_DURATION_MS
        ),
        maximum_total_recovery_encoder_degrees=(
            ENCODER_RECOVERY_MAXIMUM_TOTAL_ENCODER_DEGREES
        ),
    )


class EncoderRecoveryRuntime(object):
    """Execute policy-selected recovery while the worker owns the motors."""

    __slots__ = (
        "policy",
        "owner",
        "observe",
        "start_guard",
        "wait_for_active",
        "require_verified_finish",
        "now_ms",
        "monotonic",
        "deadline",
        "motion_budget",
        "consume_motion_budget",
        "latch_motion_fault",
        "drive_roles_by_side",
        "forward_speed_sign_by_side",
        "fault_states",
        "stop_is_verified",
    )

    def __init__(
        self,
        policy,
        owner,
        observe,
        start_guard,
        wait_for_active,
        require_verified_finish,
        now_ms,
        monotonic,
        deadline,
        motion_budget,
        consume_motion_budget,
        latch_motion_fault,
        drive_roles_by_side,
        forward_speed_sign_by_side,
        fault_states,
        stop_is_verified,
    ):
        self.policy = policy
        self.owner = owner
        self.observe = observe
        self.start_guard = start_guard
        self.wait_for_active = wait_for_active
        self.require_verified_finish = require_verified_finish
        self.now_ms = now_ms
        self.monotonic = monotonic
        self.deadline = deadline
        self.motion_budget = motion_budget
        self.consume_motion_budget = consume_motion_budget
        self.latch_motion_fault = latch_motion_fault
        self.drive_roles_by_side = dict(drive_roles_by_side)
        self.forward_speed_sign_by_side = {}
        for side in ("left", "right"):
            sign = forward_speed_sign_by_side.get(side)
            if not _is_int(sign) or sign not in (-1, 1):
                raise ValueError(
                    "forward speed sign for {} is invalid".format(side)
                )
            self.forward_speed_sign_by_side[side] = sign
        self.fault_states = frozenset(fault_states)
        self.stop_is_verified = stop_is_verified

    def undertravel_is_recoverable(self, spec, finish):
        """Accept only clean, direction-consistent encoder undertravel."""
        if not self.stop_is_verified(finish.get("stop", {})):
            return False
        motors = finish.get("motors")
        if not isinstance(motors, list) or len(motors) != 2:
            return False
        by_side = {}
        for motor in motors:
            if not isinstance(motor, dict):
                return False
            side = motor.get("side")
            if side not in ("left", "right") or side in by_side:
                return False
            before = motor.get("position_before")
            after = motor.get("position_after")
            delta = motor.get("position_delta")
            state = motor.get("state")
            if (
                not _is_int(before)
                or not _is_int(after)
                or not _is_int(delta)
                or after - before != delta
                or not isinstance(state, str)
                or frozenset(state.split()) & self.fault_states
            ):
                return False
            by_side[side] = motor
        expected_speeds = {
            "left": (
                spec["left_speed_dps"]
                * self.forward_speed_sign_by_side["left"]
            ),
            "right": (
                spec["right_speed_dps"]
                * self.forward_speed_sign_by_side["right"]
            ),
        }
        for side in ("left", "right"):
            delta = by_side[side]["position_delta"]
            if delta != 0 and delta * expected_speeds[side] <= 0:
                return False
        return True

    def transient_pair_fault_is_catch_up_candidate(
        self,
        spec,
        active,
        finish,
        duration_ms,
    ):
        """Accept only one-moving/one-static paired fault evidence.

        A sysfs ``stalled`` or ``overloaded`` token can be transient on the
        assembled EV3RSTORM.  Deferring the latch is nevertheless safe only
        when a verified stop leaves complete, direction-consistent receipts
        showing one genuine leader and one wheel below the normal progress
        floor.  The ordinary recovery policy must independently agree that
        the next bounded instruction is a single-wheel catch-up.
        """
        if not self.undertravel_is_recoverable(spec, finish):
            return False
        motors = finish["motors"]
        observed = dict(
            (motor["side"], abs(motor["position_delta"]))
            for motor in motors
        )
        left_started = (
            observed["left"] >= self.policy.minimum_progress_degrees
        )
        right_started = (
            observed["right"] >= self.policy.minimum_progress_degrees
        )
        if left_started == right_started:
            return False
        leader = "left" if left_started else "right"
        if (
            observed[leader]
            < self.policy.catch_up_leader_minimum_degrees
        ):
            return False
        decision = self._decision(
            active,
            duration_ms,
            motors,
            EncoderRecoveryBudget(),
        )
        return (
            decision["decision"] == DECISION_CATCH_UP
            and decision["instruction"] is not None
            and decision["instruction"]["mode"] == "single_wheel"
        )

    def _drive_receipts_between(self, before, after):
        before_by_role = dict(
            (motor["role"], motor) for motor in before["motors"]
        )
        after_by_role = dict(
            (motor["role"], motor) for motor in after["motors"]
        )
        receipts = []
        for side in ("left", "right"):
            role = self.drive_roles_by_side[side]
            before_position = before_by_role[role]["position"]
            after_motor = after_by_role[role]
            after_position = after_motor["position"]
            receipts.append(
                {
                    "side": side,
                    "role": role,
                    "position_before": before_position,
                    "position_after": after_position,
                    "position_delta": after_position - before_position,
                    "state": after_motor["state"],
                }
            )
        return receipts

    @staticmethod
    def _aggregate_receipts(initial_motors, final_observation):
        final_by_role = dict(
            (motor["role"], motor)
            for motor in final_observation["motors"]
        )
        receipts = []
        for initial in initial_motors:
            final = final_by_role[initial["role"]]
            after = final["position"]
            receipts.append(
                {
                    "side": initial["side"],
                    "role": initial["role"],
                    "position_before": initial["position_before"],
                    "position_after": after,
                    "position_delta": (
                        after - initial["position_before"]
                    ),
                    "state": final["state"],
                }
            )
        return receipts

    @staticmethod
    def _motor_deltas_by_side(motors):
        return dict(
            (motor["side"], motor["position_delta"])
            for motor in motors
        )

    @staticmethod
    def _budget_from_snapshot(snapshot):
        return EncoderRecoveryBudget(
            catch_up_attempts=snapshot["catch_up_attempts"],
            pair_retry_attempts=snapshot["pair_retry_attempts"],
            duration_ms=snapshot["duration_ms"],
            encoder_degrees=snapshot["encoder_degrees"],
        )

    def _decision(self, active, duration_ms, motors, budget):
        physical_speeds = dict(
            (motor["side"], motor["physical_speed_dps"])
            for motor in active["motors"]
        )
        deltas = self._motor_deltas_by_side(motors)
        expected = dict(
            (
                side,
                int(round(physical_speeds[side] * duration_ms / 1000.0)),
            )
            for side in ("left", "right")
        )
        return self.policy.decide(
            expected["left"],
            expected["right"],
            deltas["left"],
            deltas["right"],
            duration_ms,
            budget,
        )

    @staticmethod
    def _checks(decision, motors):
        completion = decision["completion_percent"]
        checks = []
        for motor in motors:
            side = motor["side"]
            checks.append(
                {
                    "role": motor["role"],
                    "side": side,
                    "position_delta": motor["position_delta"],
                    "completion_percent": completion[side],
                    "passed": (
                        completion[side]
                        >= ENCODER_RECOVERY_ACCEPTABLE_COMPLETION_PERCENT
                    ),
                }
            )
        checks.append(
            {
                "role": "paired_drive_balance",
                "side": "paired",
                "position_delta": 0,
                "completion_percent": min(
                    completion["left"], completion["right"]
                ),
                "passed": (
                    abs(completion["left"] - completion["right"])
                    <= ENCODER_RECOVERY_MAXIMUM_PROGRESS_SKEW_PERCENT
                ),
            }
        )
        return checks

    def _budget_allows(self, duration_ms):
        pulse_count, pulse_duration_ms = self.motion_budget()
        return (
            pulse_count + 1 <= MAX_PULSES
            and pulse_duration_ms + duration_ms
            <= MAX_PULSE_DURATION_MS
            and self.monotonic()
            + duration_ms / 1000.0
            + 0.25
            < self.deadline()
        )

    def _execute_instruction(
        self,
        action,
        spec,
        instruction,
        segment_index,
    ):
        duration_ms = instruction["duration_ms"]
        before = self.observe()
        mode = instruction["mode"]
        side = instruction["side"]
        try:
            if mode == "single_wheel":
                speed = spec[side + "_speed_dps"]
                active = self.owner.start_drive_side(
                    side,
                    speed,
                    duration_ms,
                    pre_each_start=lambda _motor, windows: (
                        self.start_guard(action, windows)
                    ),
                )
                commanded_sides = [side]
                kind = side + "_catch_up"
            else:
                active = self.owner.start_drive(
                    spec["left_speed_dps"],
                    spec["right_speed_dps"],
                    duration_ms,
                    pre_each_start=lambda _motor, windows: (
                        self.start_guard(action, windows)
                    ),
                )
                commanded_sides = ["left", "right"]
                kind = "paired_retry"
        except Exception as error:
            expected_sides = (
                (side,) if mode == "single_wheel"
                else ("left", "right")
            )
            evidence = validated_start_failure_evidence(
                error,
                duration_ms,
                expected_sides,
                self.drive_roles_by_side,
                self.stop_is_verified,
            )
            reason = getattr(error, "code", "motion_start_failed")
            if not evidence["started_sides"]:
                if isinstance(error, WorkerError) and error.fatal:
                    raise
                if not isinstance(error, WorkerError):
                    raise WorkerError(
                        "motion_start_failed",
                        "Motor start failed before execution",
                        fatal=True,
                    )
                return None, before, {
                    "status": "interrupted",
                    "reason": reason,
                    "stop": evidence["stop"],
                }
            self.consume_motion_budget(duration_ms)
            if partial_start_direction_mismatch(evidence):
                reason = "encoder_direction_mismatch"
                self.latch_motion_fault()
            if not isinstance(error, WorkerError):
                self.latch_motion_fault()
            after = self.observe()
            motors = self._drive_receipts_between(before, after)
            evidence_by_side = dict(
                (motor["side"], motor) for motor in evidence["motors"]
            )
            for started_side in evidence["started_sides"]:
                actual = dict(
                    (motor["side"], motor) for motor in motors
                )[started_side]
                if (
                    actual["position_before"]
                    != evidence_by_side[started_side]["position_before"]
                    or actual["position_after"]
                    != evidence_by_side[started_side]["position_after"]
                ):
                    raise WorkerError(
                        "start_evidence_incomplete",
                        "Cleanup encoders changed after partial start",
                        fatal=True,
                    )
            segment = partial_start_segment(
                evidence,
                motors,
                duration_ms,
                segment_index,
                reason,
            )
            return segment, after, {
                "status": "interrupted",
                "reason": reason,
                "stop": evidence["stop"],
            }

        self.consume_motion_budget(duration_ms)
        status, reason = self.wait_for_active(action, active)
        finish = self.require_verified_finish(verify_motion=True)
        verification_error = finish.get("verification_error")
        if status == "completed" and verification_error is not None:
            status = "verification_failed"
            reason = "encoder_undertravel_observed"
        after = self.observe()
        segment = {
            "segment_index": segment_index,
            "kind": kind,
            "commanded_sides": commanded_sides,
            "duration_ms": duration_ms,
            "status": status,
            "reason": reason,
            "started_monotonic_ms": active["started_at_ms"],
            "completed_monotonic_ms": self.now_ms(),
            "motors": self._drive_receipts_between(before, after),
            "encoder_verification": {
                "passed": verification_error is None,
                "error": verification_error,
                "checks": finish.get("checks", []),
            },
            "stop": finish["stop"],
        }
        terminal = None
        if status == "interrupted":
            terminal = {
                "status": status,
                "reason": reason,
                "stop": finish["stop"],
            }
        return segment, after, terminal

    def recover_slice(
        self,
        action,
        spec,
        duration_ms,
        active,
        primary_finish,
        primary_status,
        primary_reason,
        started_ms,
    ):
        """Execute bounded corrections and return cumulative encoder proof."""
        primary_verification_error = primary_finish.get(
            "verification_error"
        )
        primary_segment = {
            "segment_index": 1,
            "kind": "paired",
            "commanded_sides": ["left", "right"],
            "duration_ms": duration_ms,
            "status": primary_status,
            "reason": primary_reason,
            "started_monotonic_ms": started_ms,
            "completed_monotonic_ms": self.now_ms(),
            "motors": primary_finish.get("motors", []),
            "encoder_verification": {
                "passed": primary_verification_error is None,
                "error": primary_verification_error,
                "checks": primary_finish.get("checks", []),
            },
            "stop": primary_finish["stop"],
        }
        segments = [primary_segment]
        initial_motors = primary_finish.get("motors", [])
        final_observation = self.observe()
        aggregate = self._aggregate_receipts(
            initial_motors,
            final_observation,
        )
        budget = EncoderRecoveryBudget()
        stop = primary_finish["stop"]
        recovered = False

        while True:
            decision = self._decision(
                active,
                duration_ms,
                aggregate,
                budget,
            )
            choice = decision["decision"]
            if choice == DECISION_NO_RECOVERY:
                status = "completed"
                reason = (
                    "encoder_recovered"
                    if recovered
                    else "duration_elapsed"
                )
                break
            if choice == DECISION_ABORT:
                status = "verification_failed"
                if (
                    decision["reason"]
                    == REASON_ENCODER_DIRECTION_MISMATCH
                ):
                    reason = "encoder_verification_failed"
                    self.latch_motion_fault()
                else:
                    reason = "encoder_recovery_exhausted"
                break

            instruction = decision["instruction"]
            if choice not in (DECISION_CATCH_UP, DECISION_RETRY_PAIR):
                raise WorkerError(
                    "invalid_encoder_recovery_decision",
                    "Encoder recovery returned an unknown decision",
                    fatal=True,
                )
            if not self._budget_allows(instruction["duration_ms"]):
                status = "verification_failed"
                reason = "encoder_recovery_worker_budget_exhausted"
                break
            segment, after, terminal = self._execute_instruction(
                action,
                spec,
                instruction,
                len(segments) + 1,
            )
            if segment is not None:
                segments.append(segment)
                final_observation = after
                aggregate = self._aggregate_receipts(
                    initial_motors,
                    final_observation,
                )
                stop = segment["stop"]
                recovered = True
                budget = self._budget_from_snapshot(
                    decision["budget_after_if_executed"]
                )
            if terminal is not None:
                status = terminal["status"]
                reason = terminal["reason"]
                stop = terminal["stop"]
                break

        final_decision = self._decision(
            active,
            duration_ms,
            aggregate,
            budget,
        )
        if (
            final_decision["decision"] == DECISION_ABORT
            and final_decision["reason"]
            == REASON_ENCODER_DIRECTION_MISMATCH
        ):
            self.latch_motion_fault()
        checks = self._checks(final_decision, aggregate)
        passed = status == "completed" and all(
            check["passed"] for check in checks
        )
        return {
            "status": status,
            "reason": reason,
            "completed_monotonic_ms": segments[-1][
                "completed_monotonic_ms"
            ],
            "motors": aggregate,
            "segments": segments,
            "encoder_verification": {
                "passed": passed,
                "error": (
                    None
                    if passed
                    else "encoder recovery did not satisfy paired motion"
                ),
                "checks": checks,
            },
            "stop": stop,
        }
