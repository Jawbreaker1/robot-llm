#!/usr/bin/env python3
"""Policy-free EV3 motor and sensor worker for autonomous navigation.

The worker never chooses a route or action.  It owns the EV3 motor lock for
its complete bounded lifetime and executes only explicit semantic pulses
selected by the host agent.
"""

from __future__ import print_function

import copy
import os
import time

if __package__:
    from .infrared_safety import (
        InfraredGatePolicy,
        InfraredObstacleGate,
    )
    from .navigation_profile import (
        ACTION_SPECS,
        ALLOWED_OPERATIONS,
        MAX_PROCESS_SECONDS,
        MAX_PULSES,
        MAX_PULSE_DURATION_MS,
        MAX_REQUESTS,
        MAX_START_SKEW_MS,
        POLL_INTERVAL_MS,
        REQUEST_SCHEMA,
        RESPONSE_SCHEMA,
        SCAN_TURN_MAX_SIDE_DIVERGENCE_DEGREES,
        SCAN_SAMPLE_COUNT,
        SCAN_SAMPLE_INTERVAL_MS,
        WORKER_ID,
        scan_turn_profile,
        scan_turn_spec,
        scan_sample_profile,
        validate_action_specs,
    )
    from .navigation_worker_protocol import (
        WorkerError,
        validate_identifier,
    )
    from .robot_hal import RobotHAL
    from .supervisor import (
        EV3Supervisor,
        FAULT_MOTOR_STATES,
        SupervisorMotorOwner,
    )
else:
    from infrared_safety import (
        InfraredGatePolicy,
        InfraredObstacleGate,
    )
    from navigation_profile import (
        ACTION_SPECS,
        ALLOWED_OPERATIONS,
        MAX_PROCESS_SECONDS,
        MAX_PULSES,
        MAX_PULSE_DURATION_MS,
        MAX_REQUESTS,
        MAX_START_SKEW_MS,
        POLL_INTERVAL_MS,
        REQUEST_SCHEMA,
        RESPONSE_SCHEMA,
        SCAN_TURN_MAX_SIDE_DIVERGENCE_DEGREES,
        SCAN_SAMPLE_COUNT,
        SCAN_SAMPLE_INTERVAL_MS,
        WORKER_ID,
        scan_turn_profile,
        scan_turn_spec,
        scan_sample_profile,
        validate_action_specs,
    )
    from navigation_worker_protocol import (
        WorkerError,
        validate_identifier,
    )
    from robot_hal import RobotHAL
    from supervisor import (
        EV3Supervisor,
        FAULT_MOTOR_STATES,
        SupervisorMotorOwner,
    )


PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir)
)
DEFAULT_CONFIG_PATH = os.path.join(
    PROJECT_ROOT,
    "config",
    "ev3rstorm.json",
)
FAULT_STATES = frozenset(FAULT_MOTOR_STATES)


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def stop_is_verified(result):
    return (
        isinstance(result, dict)
        and result.get("stop_confirmed") is True
        and not result.get("errors")
        and not result.get("fault_tokens")
    )


class NavigationWorker(object):
    """One bounded, exclusively owned EV3 navigation session."""

    def __init__(
        self,
        config_path=DEFAULT_CONFIG_PATH,
        cancel_requested=None,
    ):
        if cancel_requested is None:
            cancel_requested = lambda: False
        if not callable(cancel_requested):
            raise ValueError("cancel_requested must be callable")
        self.owner = None
        self.closed = False
        self._cancel_requested = cancel_requested
        self.started_at = time.monotonic()
        self.deadline = self.started_at + MAX_PROCESS_SECONDS
        self.robot = RobotHAL(config_path)
        self.limits = EV3Supervisor._validated_limits(self.robot.config)
        self.limits["max_start_skew_ms"] = MAX_START_SKEW_MS
        self.owner = SupervisorMotorOwner(self.robot, self.limits)
        self.controller_id = validate_identifier(
            "controller_id",
            self.robot.config["controller_id"],
            128,
        )
        self.gate = InfraredObstacleGate(
            InfraredGatePolicy.from_config(self.robot.config)
        )
        self.state_version = 0
        self.request_count = 0
        self.pulse_count = 0
        self.pulse_duration_ms = 0
        self.motion_fault_latched = False
        self.last_outcome = {
            "kind": "startup",
            "status": "initializing",
        }
        self.last_observation = None
        self.seen_request_ids = set()
        self.shutdown_requested = False

        self.owner.bind_topology()
        startup_stop = self.owner.stop_all_verified()
        if not stop_is_verified(startup_stop):
            raise WorkerError(
                "startup_stop_not_confirmed",
                "Initial local motor stop was not verified",
                fatal=True,
            )

        # Five stable samples are required by the configured median and
        # release hysteresis before forward motion can become eligible.
        for index in range(5):
            self._observe()
            if index < 4:
                self.robot.sleep_fn(POLL_INTERVAL_MS / 1000.0)
        self.last_outcome = {
            "kind": "startup",
            "status": "ready",
            "stop_confirmed": True,
        }
        self.last_observation = self._observe()

        try:
            validate_action_specs(self.owner)
        except ValueError as error:
            raise WorkerError(
                "invalid_action_spec",
                str(error),
                fatal=True,
            )

    def _now_ms(self):
        return int(time.monotonic() * 1000)

    def _remaining_process_ms(self):
        return max(0, int((self.deadline - time.monotonic()) * 1000))

    def _cancellation_is_requested(self):
        try:
            return self._cancel_requested() is True
        except BaseException:
            raise WorkerError(
                "cancellation_probe_failed",
                "Cancellation state could not be read",
                fatal=True,
            )

    def _budget_snapshot(self):
        return {
            "pulse_count": self.pulse_count,
            "pulse_count_remaining": MAX_PULSES - self.pulse_count,
            "pulse_duration_ms": self.pulse_duration_ms,
            "pulse_duration_ms_remaining": (
                MAX_PULSE_DURATION_MS - self.pulse_duration_ms
            ),
            "motion_fault_latched": self.motion_fault_latched,
            "process_ms_remaining": self._remaining_process_ms(),
        }

    def _observe(self):
        try:
            touch_value = self.owner.read_touch_value()
            if not _is_int(touch_value) or touch_value not in (0, 1):
                raise WorkerError(
                    "invalid_touch_sample",
                    "Touch must report exactly 0 or 1",
                    fatal=True,
                )
            infrared_value = self.owner.read_infrared_value()
            infrared = self.gate.observe(infrared_value)
            motor_rows = self.owner.snapshot_all()
        except BaseException:
            self.gate.fail_closed()
            raise

        motors = []
        for row in motor_rows:
            motors.append(
                {
                    "role": row["role"],
                    "position": row["position"],
                    "state": row["state"],
                }
            )
        self.state_version += 1
        observation = {
            "state_version": self.state_version,
            "observed_monotonic_ms": self._now_ms(),
            "touch": {
                "value0": touch_value,
                "pressed": touch_value == 1,
            },
            "infrared": infrared,
            "motors": motors,
            "last_outcome": copy.deepcopy(self.last_outcome),
            "budgets": self._budget_snapshot(),
        }
        self.last_observation = observation
        return copy.deepcopy(observation)

    def _require_verified_finish(self, verify_motion=False):
        finish = self.owner.finish_active(verify_motion)
        stop = finish.get("stop", {})
        if not stop_is_verified(stop):
            raise WorkerError(
                "stop_not_confirmed",
                "Local motor stop was not verified",
                fatal=True,
            )
        return finish

    def _deny_pulse(self, action, reason):
        finish = self._require_verified_finish()
        now_ms = self._now_ms()
        requested_slices = ACTION_SPECS[action]["slice_count"]
        self.last_outcome = {
            "kind": "pulse",
            "action": action,
            "status": "denied",
            "reason": reason,
            "started_monotonic_ms": None,
            "completed_monotonic_ms": now_ms,
            "stop_confirmed": True,
            "requested_slice_count": requested_slices,
            "completed_slice_count": 0,
            "slices": [],
            "encoder_verification": {
                "passed": False,
                "verified_slice_count": 0,
                "requested_slice_count": requested_slices,
            },
        }
        return {
            "action": action,
            "outcome": copy.deepcopy(self.last_outcome),
            "observation": self._observe(),
            "stop": finish["stop"],
        }

    def _start_guard(self, action, prior_write_windows):
        if self._cancellation_is_requested():
            self.shutdown_requested = True
            raise WorkerError(
                "cancel_requested",
                "Worker cancellation was requested before motor start",
            )
        if prior_write_windows:
            if time.monotonic() >= self.deadline:
                raise WorkerError(
                    "process_deadline",
                    "Process deadline reached during motor start",
                    fatal=True,
                )
            return

        # Binding already performs complete topology revalidation.  The hot
        # path uses bound-reader identity/mode checks in _observe() instead
        # of re-resolving every device before each short segment.
        observation = self._observe()
        if observation["touch"]["pressed"]:
            raise WorkerError(
                "touch_pressed",
                "Touch became pressed before motor start",
            )
        if (
            action == "ADVANCE"
            and observation["infrared"]["blocked"]
        ):
            raise WorkerError(
                "infrared_blocked",
                "Forward path became blocked before motor start",
            )
        if time.monotonic() >= self.deadline:
            raise WorkerError(
                "process_deadline",
                "Process deadline reached before motor start",
                fatal=True,
            )

    def _execute_slice(
        self,
        action,
        spec,
        duration_ms,
        slice_index,
    ):
        try:
            active = self.owner.start_drive(
                spec["left_speed_dps"],
                spec["right_speed_dps"],
                duration_ms,
                pre_each_start=lambda _motor, windows: (
                    self._start_guard(action, windows)
                ),
            )
        except WorkerError as error:
            start_cleanup = getattr(
                error,
                "supervisor_start_cleanup",
                None,
            )
            if stop_is_verified(start_cleanup):
                finish = {
                    "stop": start_cleanup,
                    "motors": [],
                }
            else:
                finish = self._require_verified_finish()
            self.last_outcome = {
                "kind": "pulse",
                "action": action,
                "status": "denied",
                "reason": error.code,
                "started_monotonic_ms": None,
                "completed_monotonic_ms": self._now_ms(),
                "stop_confirmed": True,
            }
            if error.fatal:
                raise
            return {
                "slice_index": slice_index,
                "slice_count": spec["slice_count"],
                "duration_ms": duration_ms,
                "status": "denied",
                "reason": error.code,
                "started_monotonic_ms": None,
                "completed_monotonic_ms": self._now_ms(),
                "motors": [],
                "encoder_verification": {
                    "passed": False,
                    "error": None,
                    "checks": [],
                },
                "stop": finish["stop"],
            }

        self.pulse_count += 1
        self.pulse_duration_ms += duration_ms
        started_ms = active["started_at_ms"]
        reason = "duration_elapsed"
        status = "completed"
        active_roles = frozenset(
            motor["role"] for motor in active["motors"]
        )
        while True:
            if self._cancellation_is_requested():
                reason = "cancel_requested"
                status = "interrupted"
                self.shutdown_requested = True
                break
            observation = self._observe()
            if observation["touch"]["pressed"]:
                reason = "touch_pressed"
                status = "interrupted"
                break
            if (
                action == "ADVANCE"
                and observation["infrared"]["blocked"]
            ):
                reason = "infrared_blocked"
                status = "interrupted"
                break
            if time.monotonic() >= self.deadline:
                reason = "process_deadline"
                status = "interrupted"
                self.shutdown_requested = True
                break
            motor_fault = False
            for motor in observation["motors"]:
                if motor["role"] not in active_roles:
                    continue
                tokens = frozenset(motor["state"].split())
                if tokens & FAULT_STATES:
                    motor_fault = True
                    break
            if motor_fault:
                reason = "motor_fault"
                status = "interrupted"
                self.motion_fault_latched = True
                break
            now_ms = self._now_ms()
            if now_ms >= active["deadline_ms"]:
                break
            remaining_ms = active["deadline_ms"] - now_ms
            self.robot.sleep_fn(
                min(POLL_INTERVAL_MS, remaining_ms) / 1000.0
            )

        finish = self._require_verified_finish(verify_motion=True)
        verification_error = finish.get("verification_error")
        if status == "completed" and verification_error is not None:
            status = "verification_failed"
            reason = "encoder_verification_failed"
            self.motion_fault_latched = True
        return {
            "slice_index": slice_index,
            "slice_count": spec["slice_count"],
            "duration_ms": duration_ms,
            "status": status,
            "reason": reason,
            "started_monotonic_ms": started_ms,
            "completed_monotonic_ms": self._now_ms(),
            "motors": finish.get("motors", []),
            "encoder_verification": {
                "passed": verification_error is None,
                "error": verification_error,
                "checks": finish.get("checks", []),
            },
            "stop": finish["stop"],
        }

    def _pulse(self, action):
        if action not in ACTION_SPECS:
            raise WorkerError(
                "invalid_action",
                "Pulse action is not supported",
            )
        if self.motion_fault_latched:
            raise WorkerError(
                "motion_fault_latched",
                "Further motion is blocked after a motion verification "
                "or motor-state fault",
            )
        spec = ACTION_SPECS[action]
        requested_slices = spec["slice_count"]
        total_duration_ms = spec["total_duration_ms"]
        if (
            self.pulse_count + requested_slices > MAX_PULSES
            or self.pulse_duration_ms + total_duration_ms
            > MAX_PULSE_DURATION_MS
        ):
            raise WorkerError(
                "pulse_budget_exhausted",
                "The fixed physical-slice budget is exhausted",
                fatal=True,
            )
        # Leave bounded headroom for verified stop and observation between
        # slices, not only the run-timed motor duration.
        if (
            time.monotonic()
            + total_duration_ms / 1000.0
            + requested_slices * 0.25
            >= self.deadline
        ):
            raise WorkerError(
                "process_deadline",
                "Not enough process lifetime remains for this action",
                fatal=True,
            )

        before = self._observe()
        if self._cancellation_is_requested():
            self.shutdown_requested = True
            return self._deny_pulse(action, "cancel_requested")
        if before["touch"]["pressed"]:
            return self._deny_pulse(action, "touch_pressed")
        if action == "ADVANCE" and before["infrared"]["blocked"]:
            return self._deny_pulse(action, "infrared_blocked")

        slices = []
        for slice_index, duration_ms in enumerate(
            spec["slice_durations_ms"],
            1,
        ):
            receipt = self._execute_slice(
                action,
                spec,
                duration_ms,
                slice_index,
            )
            slices.append(receipt)
            if receipt["status"] != "completed":
                break

        completed_slices = sum(
            1 for receipt in slices
            if receipt["status"] == "completed"
        )
        if completed_slices == requested_slices:
            status = "completed"
            reason = "semantic_action_completed"
        else:
            terminal = slices[-1]
            status = terminal["status"]
            reason = terminal["reason"]
            if reason == "cancel_requested" and completed_slices:
                status = "interrupted"
        verified_slices = sum(
            1 for receipt in slices
            if receipt["encoder_verification"]["passed"]
        )
        first_started_ms = None
        for receipt in slices:
            if receipt["started_monotonic_ms"] is not None:
                first_started_ms = receipt["started_monotonic_ms"]
                break
        terminal = slices[-1]
        self.last_outcome = {
            "kind": "pulse",
            "action": action,
            "status": status,
            "reason": reason,
            "started_monotonic_ms": first_started_ms,
            "completed_monotonic_ms": terminal[
                "completed_monotonic_ms"
            ],
            "stop_confirmed": True,
            "requested_slice_count": requested_slices,
            "completed_slice_count": completed_slices,
            "slices": slices,
            "encoder_verification": {
                "passed": verified_slices == requested_slices,
                "verified_slice_count": verified_slices,
                "requested_slice_count": requested_slices,
            },
        }
        return {
            "action": action,
            "outcome": copy.deepcopy(self.last_outcome),
            "observation": self._observe(),
            "stop": terminal["stop"],
        }

    @staticmethod
    def _scan_turn_encoder_totals(slices):
        totals = {"left": 0, "right": 0}
        for receipt in slices:
            seen_sides = set()
            for motor in receipt.get("motors", []):
                side = motor.get("side")
                delta = motor.get("position_delta")
                if (
                    side not in totals
                    or side in seen_sides
                    or not _is_int(delta)
                ):
                    return None
                seen_sides.add(side)
                totals[side] += delta
            if seen_sides != set(("left", "right")):
                return None
        return totals

    def _deny_scan_turn(self, relative_delta_mdeg, spec, reason):
        finish = self._require_verified_finish()
        now_ms = self._now_ms()
        self.last_outcome = {
            "kind": "scan_turn",
            "requested_relative_delta_mdeg": relative_delta_mdeg,
            "status": "denied",
            "reason": reason,
            "profile_id": spec["profile_id"],
            "calibration": spec["calibration"],
            "started_monotonic_ms": None,
            "completed_monotonic_ms": now_ms,
            "stop_confirmed": True,
            "requested_slice_count": spec["slice_count"],
            "completed_slice_count": 0,
            "slices": [],
            "encoder_verification": {
                "passed": False,
                "verified_slice_count": 0,
                "requested_slice_count": spec["slice_count"],
                "left_delta_degrees": None,
                "right_delta_degrees": None,
                "mean_abs_encoder_degrees": None,
                "target_mean_abs_encoder_degrees": spec[
                    "target_mean_abs_encoder_degrees"
                ],
                "max_side_divergence_degrees": (
                    SCAN_TURN_MAX_SIDE_DIVERGENCE_DEGREES
                ),
            },
        }
        return {
            "relative_delta_mdeg": relative_delta_mdeg,
            "outcome": copy.deepcopy(self.last_outcome),
            "observation": self._observe(),
            "stop": finish["stop"],
        }

    def _scan_turn(self, relative_delta_mdeg):
        """Execute one host-only body turn from the fixed scan lattice."""
        try:
            spec = scan_turn_spec(relative_delta_mdeg)
        except ValueError:
            raise WorkerError(
                "invalid_scan_turn",
                "Relative scan turn is outside the fixed host profile",
            )
        if self.motion_fault_latched:
            raise WorkerError(
                "motion_fault_latched",
                "Further motion is blocked after a motion verification "
                "or motor-state fault",
            )
        requested_slices = spec["slice_count"]
        total_duration_ms = spec["total_duration_ms"]
        if (
            self.pulse_count + requested_slices > MAX_PULSES
            or self.pulse_duration_ms + total_duration_ms
            > MAX_PULSE_DURATION_MS
        ):
            raise WorkerError(
                "pulse_budget_exhausted",
                "The fixed physical-slice budget is exhausted",
                fatal=True,
            )
        if (
            time.monotonic()
            + total_duration_ms / 1000.0
            + requested_slices * 0.25
            >= self.deadline
        ):
            raise WorkerError(
                "process_deadline",
                "Not enough process lifetime remains for this scan turn",
                fatal=True,
            )

        before = self._observe()
        if self._cancellation_is_requested():
            self.shutdown_requested = True
            return self._deny_scan_turn(
                relative_delta_mdeg,
                spec,
                "cancel_requested",
            )
        if before["touch"]["pressed"]:
            return self._deny_scan_turn(
                relative_delta_mdeg,
                spec,
                "touch_pressed",
            )

        slices = []
        for slice_index, duration_ms in enumerate(
            spec["slice_durations_ms"],
            1,
        ):
            receipt = self._execute_slice(
                "SCAN_TURN",
                spec,
                duration_ms,
                slice_index,
            )
            slices.append(receipt)
            if receipt["status"] != "completed":
                break

        completed_slices = sum(
            1 for receipt in slices
            if receipt["status"] == "completed"
        )
        verified_slices = sum(
            1 for receipt in slices
            if receipt["encoder_verification"]["passed"]
        )
        totals = self._scan_turn_encoder_totals(slices)
        encoder_passed = (
            totals is not None
            and completed_slices == requested_slices
            and verified_slices == requested_slices
        )
        mean_abs_encoder_degrees = None
        if totals is not None:
            left_delta = totals["left"]
            right_delta = totals["right"]
            mean_abs_encoder_degrees = int(
                round(
                    (
                        abs(left_delta)
                        + abs(right_delta)
                    )
                    / 2.0
                )
            )
            expected_direction = (
                1 if relative_delta_mdeg > 0 else -1
            )
            encoder_passed = (
                encoder_passed
                and left_delta * expected_direction < 0
                and right_delta * expected_direction > 0
                and abs(abs(left_delta) - abs(right_delta))
                <= SCAN_TURN_MAX_SIDE_DIVERGENCE_DEGREES
            )

        if completed_slices == requested_slices and encoder_passed:
            status = "completed"
            reason = "scan_turn_completed"
        elif completed_slices == requested_slices:
            status = "verification_failed"
            reason = "scan_turn_encoder_verification_failed"
            self.motion_fault_latched = True
        else:
            terminal = slices[-1]
            status = terminal["status"]
            reason = terminal["reason"]
        first_started_ms = None
        for receipt in slices:
            if receipt["started_monotonic_ms"] is not None:
                first_started_ms = receipt["started_monotonic_ms"]
                break
        terminal = slices[-1]
        self.last_outcome = {
            "kind": "scan_turn",
            "requested_relative_delta_mdeg": relative_delta_mdeg,
            "status": status,
            "reason": reason,
            "profile_id": spec["profile_id"],
            "calibration": spec["calibration"],
            "started_monotonic_ms": first_started_ms,
            "completed_monotonic_ms": terminal[
                "completed_monotonic_ms"
            ],
            "stop_confirmed": True,
            "requested_slice_count": requested_slices,
            "completed_slice_count": completed_slices,
            "slices": slices,
            "encoder_verification": {
                "passed": encoder_passed,
                "verified_slice_count": verified_slices,
                "requested_slice_count": requested_slices,
                "left_delta_degrees": (
                    None if totals is None else totals["left"]
                ),
                "right_delta_degrees": (
                    None if totals is None else totals["right"]
                ),
                "mean_abs_encoder_degrees": (
                    mean_abs_encoder_degrees
                ),
                "target_mean_abs_encoder_degrees": spec[
                    "target_mean_abs_encoder_degrees"
                ],
                "max_side_divergence_degrees": (
                    SCAN_TURN_MAX_SIDE_DIVERGENCE_DEGREES
                ),
            },
        }
        return {
            "relative_delta_mdeg": relative_delta_mdeg,
            "outcome": copy.deepcopy(self.last_outcome),
            "observation": self._observe(),
            "stop": terminal["stop"],
        }

    def _stop(self, kind):
        finish = self._require_verified_finish()
        self.last_outcome = {
            "kind": kind,
            "status": "completed",
            "completed_monotonic_ms": self._now_ms(),
            "stop_confirmed": True,
        }
        return {
            "outcome": copy.deepcopy(self.last_outcome),
            "observation": self._observe(),
            "stop": finish["stop"],
        }

    def _scan_sample(self):
        """Build one fresh stationary IR batch at the current bearing."""
        finish = self._require_verified_finish()
        self.gate = InfraredObstacleGate(self.gate.policy)
        raw_samples = []
        observation = None
        started_ms = self._now_ms()
        for index in range(SCAN_SAMPLE_COUNT):
            if self._cancellation_is_requested():
                self.shutdown_requested = True
                raise WorkerError(
                    "cancel_requested",
                    "Scan sampling was cancelled",
                )
            observation = self._observe()
            raw_samples.append(observation["infrared"]["raw"])
            if observation["touch"]["pressed"]:
                raise WorkerError(
                    "touch_pressed",
                    "Touch cancelled stationary scan sampling",
                )
            if index < SCAN_SAMPLE_COUNT - 1:
                self.robot.sleep_fn(
                    SCAN_SAMPLE_INTERVAL_MS / 1000.0
                )
        return {
            "sample_count": SCAN_SAMPLE_COUNT,
            "raw_samples": raw_samples,
            "started_monotonic_ms": started_ms,
            "completed_monotonic_ms": self._now_ms(),
            "observation": observation,
            "stop": finish["stop"],
        }

    def _shutdown(self):
        close_result = self.close()
        observation = copy.deepcopy(self.last_observation)
        if observation is None:
            motors = []
            for role in sorted(self.robot.config["motors"]):
                path = self.owner.path_for_role(role)
                motors.append(
                    {
                        "role": role,
                        "position": close_result.get(
                            "positions",
                            {},
                        ).get(path),
                        "state": close_result.get(
                            "states",
                            {},
                        ).get(path),
                    }
                )
            observation = {
                "state_version": self.state_version,
                "observed_monotonic_ms": self._now_ms(),
                "touch": {
                    "value0": None,
                    "pressed": None,
                },
                "infrared": self.gate.snapshot(),
                "motors": motors,
                "last_outcome": copy.deepcopy(
                    self.last_outcome
                ),
                "budgets": self._budget_snapshot(),
            }
        self.last_outcome = {
            "kind": "shutdown",
            "status": "completed",
            "completed_monotonic_ms": self._now_ms(),
            "stop_confirmed": True,
            "motor_owner_closed": True,
        }
        self.state_version += 1
        observation["state_version"] = self.state_version
        observation["observed_monotonic_ms"] = self._now_ms()
        observation["last_outcome"] = copy.deepcopy(
            self.last_outcome
        )
        observation["budgets"] = self._budget_snapshot()
        for motor in observation["motors"]:
            path = self.owner.path_for_role(motor["role"])
            if path in close_result.get("states", {}):
                motor["state"] = close_result["states"][path]
            if path in close_result.get("positions", {}):
                motor["position"] = close_result["positions"][path]
        self.last_observation = copy.deepcopy(observation)
        self.shutdown_requested = True
        return {
            "outcome": copy.deepcopy(self.last_outcome),
            "observation": observation,
            "close": close_result,
        }

    def describe(self):
        return {
            "worker_id": WORKER_ID,
            "demo_only": True,
            "policy_owner": "host",
            "controller_id": self.controller_id,
            "request_schema": REQUEST_SCHEMA,
            "response_schema": RESPONSE_SCHEMA,
            "operations": list(ALLOWED_OPERATIONS),
            "drive_geometry": copy.deepcopy(
                self.robot.config["drive_geometry"]
            ),
            "pulse": {
                "actions": copy.deepcopy(ACTION_SPECS),
                "max_pulses": MAX_PULSES,
                "max_total_duration_ms": MAX_PULSE_DURATION_MS,
            },
            "scan_turn": scan_turn_profile(),
            "scan_sample": scan_sample_profile(),
            "safety": {
                "lifetime_motor_lock": True,
                "bound_hardware_topology": True,
                "touch_interrupts_all_motion": True,
                "infrared_blocks_and_interrupts_advance": True,
                "infrared_does_not_block_turns": True,
                "process_signals_interrupt_active_pulses": True,
                "channel_close_interrupts_active_pulses": True,
                "worker_selects_actions": False,
            },
            "process": {
                "absolute_max_ms": int(MAX_PROCESS_SECONDS * 1000),
                "max_requests": MAX_REQUESTS,
            },
            "observation": self._observe(),
        }

    def execute(self, request):
        operation = request["op"]
        if operation == "describe":
            return self.describe()
        if operation == "observe":
            return {"observation": self._observe()}
        if operation == "pulse":
            return self._pulse(request["args"]["action"])
        if operation == "scan_turn":
            return self._scan_turn(
                request["args"]["relative_delta_mdeg"]
            )
        if operation == "scan_sample":
            return self._scan_sample()
        if operation == "stop":
            return self._stop("stop")
        if operation == "shutdown":
            return self._shutdown()
        raise WorkerError(
            "unsupported_operation",
            "Operation is not supported",
            request_id=request["request_id"],
        )

    def close(self):
        if self.closed:
            return {
                "stop_confirmed": True,
                "already_closed": True,
            }
        if self.owner is None:
            self.closed = True
            return {
                "stop_confirmed": True,
                "no_motor_owner_created": True,
            }
        result = self.owner.close()
        if (
            not stop_is_verified(result)
            or result.get("cleanup_errors")
            or not self.owner.closed
        ):
            raise WorkerError(
                "close_not_confirmed",
                "Final local stop and motor-owner close were not verified",
                fatal=True,
            )
        self.closed = True
        return result
