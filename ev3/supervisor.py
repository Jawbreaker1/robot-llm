#!/usr/bin/env python3
"""Language-blind, Python 3.5-compatible local EV3 motion supervisor.

The supervisor owns the motor flock for its entire lifetime. It never
interprets natural language and never calls an LLM. Its only job is to admit
and monitor already-typed, locally bounded motor primitives.
"""

from __future__ import print_function

import binascii
import copy
from collections import deque
import hashlib
import io
import json
import os
import threading
import time

try:
    from .robot_hal import (
        MotionVerificationError,
        RobotHAL,
        SafetyError,
        read_text,
        write_text,
    )
except (ImportError, ValueError, SystemError):
    from robot_hal import (
        MotionVerificationError,
        RobotHAL,
        SafetyError,
        read_text,
        write_text,
    )


STATE_BOOTING = "BOOTING"
STATE_DISARMED = "DISARMED"
STATE_ARMED_IDLE = "ARMED_IDLE"
STATE_RUNNING = "RUNNING"
STATE_FAULT_LATCHED = "FAULT_LATCHED"
STATE_SHUTTING_DOWN = "SHUTTING_DOWN"
STATE_CLOSED = "CLOSED"

ACTIVE_MOTOR_STATES = frozenset(("running", "ramping", "holding"))
FAULT_MOTOR_STATES = frozenset(("stalled", "overloaded"))
KNOWN_MOTOR_STATES = frozenset(
    ("running", "ramping", "holding", "stalled", "overloaded")
)
AUDIT_SCHEMA = "ev3-supervisor-audit/v1"


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _validate_positive_int(name, value, maximum=None):
    if not _is_int(value) or value <= 0:
        raise SupervisorError(
            "invalid_{}".format(name),
            "{} must be a positive integer".format(name),
        )
    if maximum is not None and value > maximum:
        raise SupervisorError(
            "invalid_{}".format(name),
            "{} exceeds maximum {}".format(name, maximum),
        )
    return value


def _validate_identifier(name, value, maximum):
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or "\x00" in value
        or "\n" in value
        or "\r" in value
    ):
        raise SupervisorError(
            "invalid_{}".format(name),
            "{} must contain 1..{} safe characters".format(name, maximum),
        )
    return value


def _state_tokens(raw):
    if not isinstance(raw, str):
        raise SupervisorError(
            "invalid_motor_state",
            "Motor state must be text",
        )
    tokens = frozenset(raw.split())
    unknown = tokens - KNOWN_MOTOR_STATES
    if unknown:
        raise SupervisorError(
            "invalid_motor_state",
            "Motor state contains unknown tokens: {}".format(
                sorted(unknown)
            ),
        )
    return tokens


def _default_session_id():
    return binascii.hexlify(os.urandom(16)).decode("ascii")


class SupervisorError(SafetyError):
    def __init__(self, code, message):
        self.code = code
        SafetyError.__init__(self, message)


class JSONLAuditLog(object):
    """Small append-only JSONL sink with durable transition writes."""

    def __init__(self, path):
        if not isinstance(path, str) or not path:
            raise SupervisorError(
                "invalid_audit_path",
                "Audit path is invalid",
            )
        self.path = path
        flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(path, flags, 0o600)
        os.fchmod(descriptor, 0o600)
        self._handle = io.open(
            descriptor,
            "a",
            encoding="utf-8",
            closefd=True,
        )
        self._closed = False

    def append(self, event):
        if self._closed:
            raise IOError("Audit log is closed")
        encoded = json.dumps(
            event,
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        self._handle.write(encoded + "\n")
        self._handle.flush()
        os.fsync(self._handle.fileno())

    def close(self):
        if self._closed:
            return
        self._handle.close()
        self._closed = True


class AuditBuffer(object):
    """Bounded in-memory audit queue that never performs external I/O."""

    def __init__(self, maximum_events):
        _validate_positive_int(
            "audit_buffer_events",
            maximum_events,
            10000,
        )
        self.maximum_events = maximum_events
        self._events = deque()

    def append(self, event, terminal=False):
        reserved = 0 if terminal else 1
        if len(self._events) >= self.maximum_events - reserved:
            raise SupervisorError(
                "audit_buffer_full",
                "Audit buffer is full",
            )
        self._events.append(copy.deepcopy(event))

    def snapshot(self):
        return [copy.deepcopy(event) for event in self._events]

    def drain(self):
        events = []
        while self._events:
            events.append(self._events.popleft())
        return events


def _failed_stop_result(error):
    return {
        "stop_attempts": [],
        "stop_confirmed": False,
        "states": {},
        "positions": {},
        "fault_tokens": {},
        "errors": [str(error)],
    }


def _stop_result_from_exception(error):
    attached = getattr(error, "supervisor_stop_result", None)
    if isinstance(attached, dict):
        return copy.deepcopy(attached)
    return _failed_stop_result(error)


def _combine_stop_results(labelled_results):
    history = []
    errors = []
    latest = None
    for label, result in labelled_results:
        if result is None:
            continue
        snapshot = copy.deepcopy(result)
        history.append(
            {
                "phase": label,
                "result": snapshot,
            }
        )
        for detail in snapshot.get("errors", []):
            errors.append("{}: {}".format(label, detail))
        latest = snapshot
    if latest is None:
        latest = _failed_stop_result("No stop result")
    latest["errors"] = errors
    latest["stop_history"] = history
    return latest


class SupervisorMotorOwner(object):
    """Non-blocking motor primitive under a lifetime ownership lock."""

    def __init__(self, robot, limits):
        if not isinstance(robot, RobotHAL):
            raise SupervisorError(
                "invalid_hal",
                "Supervisor requires RobotHAL",
            )
        self.robot = robot
        self.limits = limits
        self._lock_handle = robot._acquire_motor_lock()
        self.active = None
        self.closed = False

    def _require_open(self):
        if self.closed:
            raise SupervisorError(
                "motor_owner_closed",
                "Motor owner is closed",
            )

    def _discovered_motor_paths(self):
        pattern = os.path.join(
            self.robot.sysfs_root,
            "tacho-motor",
            "*",
        )
        return sorted(self.robot_module_glob(pattern))

    @staticmethod
    def robot_module_glob(pattern):
        # Kept as a small seam for deterministic tests.
        import glob

        return glob.glob(pattern)

    def _configured_paths(self):
        paths = []
        for role in sorted(self.robot.config["motors"]):
            path = self.robot._motor_path_for_role(role)
            if path in paths:
                raise SupervisorError(
                    "duplicate_motor_path",
                    "Two configured roles resolve to one motor",
                )
            paths.append(path)
        return paths

    def snapshot_all(self):
        self._require_open()
        configured_paths = self._configured_paths()
        discovered_paths = self._discovered_motor_paths()
        if set(configured_paths) != set(discovered_paths):
            raise SupervisorError(
                "unexpected_motor_inventory",
                "Discovered motors do not exactly match configuration",
            )

        snapshots = []
        for path in discovered_paths:
            snapshots.append(
                {
                    "path": path,
                    "address": read_text(os.path.join(path, "address")),
                    "position": int(
                        read_text(os.path.join(path, "position"))
                    ),
                    "state": read_text(os.path.join(path, "state")),
                }
            )
        return snapshots

    def stop_all_verified(self):
        """Retry stop and require readable, inactive, stable motor state."""
        self._require_open()
        errors = []
        stop_attempts = []
        deferred_interrupt = None
        try:
            paths = self._discovered_motor_paths()
        except BaseException as error:
            paths = []
            errors.append("discover: {}".format(error))
            if (
                not isinstance(error, Exception)
                and deferred_interrupt is None
            ):
                deferred_interrupt = error

        try:
            configured = self._configured_paths()
            for path in configured:
                if path not in paths:
                    paths.append(path)
            paths = sorted(set(paths))
        except BaseException as error:
            errors.append("configured: {}".format(error))
            if (
                not isinstance(error, Exception)
                and deferred_interrupt is None
            ):
                deferred_interrupt = error

        timeout_ms = self.limits["stop_verify_timeout_ms"]
        poll_ms = self.limits["stop_poll_interval_ms"]
        maximum_attempts = max(2, (timeout_ms // poll_ms) + 1)
        last_states = {}
        last_positions = {}
        previous_positions = None
        stop_confirmed = False
        fault_tokens = {}

        for attempt in range(maximum_attempts):
            for path in paths:
                stop_attempt = {
                    "attempt": attempt + 1,
                    "path": path,
                    "succeeded": False,
                }
                try:
                    write_text(
                        os.path.join(path, "stop_action"),
                        "brake",
                    )
                    stop_attempt["stop_action_succeeded"] = True
                except BaseException as error:
                    detail = "{} stop_action: {}".format(path, error)
                    stop_attempt["stop_action_error"] = str(error)
                    if detail not in errors:
                        errors.append(detail)
                    if (
                        not isinstance(error, Exception)
                        and deferred_interrupt is None
                    ):
                        deferred_interrupt = error
                try:
                    write_text(
                        os.path.join(path, "command"),
                        "stop",
                    )
                    stop_attempt["succeeded"] = True
                except BaseException as error:
                    detail = "{} stop: {}".format(path, error)
                    stop_attempt["error"] = str(error)
                    if detail not in errors:
                        errors.append(detail)
                    if (
                        not isinstance(error, Exception)
                        and deferred_interrupt is None
                    ):
                        deferred_interrupt = error
                stop_attempts.append(stop_attempt)

            read_errors = []
            active = []
            fault_tokens = {}
            last_states = {}
            last_positions = {}
            for path in paths:
                try:
                    raw_state = read_text(os.path.join(path, "state"))
                    tokens = _state_tokens(raw_state)
                    last_states[path] = raw_state
                    last_positions[path] = int(
                        read_text(os.path.join(path, "position"))
                    )
                    if tokens & ACTIVE_MOTOR_STATES:
                        active.append(path)
                    if tokens & FAULT_MOTOR_STATES:
                        fault_tokens[path] = sorted(
                            tokens & FAULT_MOTOR_STATES
                        )
                except BaseException as error:
                    read_errors.append(
                        "{} verification: {}".format(path, error)
                    )
                    if (
                        not isinstance(error, Exception)
                        and deferred_interrupt is None
                    ):
                        deferred_interrupt = error

            stable_positions = (
                previous_positions is not None
                and last_positions == previous_positions
            )
            if (
                not active
                and not read_errors
                and not fault_tokens
                and stable_positions
            ):
                stop_confirmed = True
                break
            if not read_errors:
                previous_positions = dict(last_positions)
            if attempt + 1 < maximum_attempts:
                try:
                    self.robot.sleep_fn(poll_ms / 1000.0)
                except BaseException as error:
                    detail = "stop sleep: {}".format(error)
                    if detail not in errors:
                        errors.append(detail)
                    if (
                        not isinstance(error, Exception)
                        and deferred_interrupt is None
                    ):
                        deferred_interrupt = error
            if read_errors and attempt + 1 == maximum_attempts:
                errors.extend(read_errors)

        if not stop_confirmed and paths:
            errors.append("Motor stop was not confirmed before deadline")
        if not paths:
            errors.append("No tacho motors were discovered")

        result = {
            "stop_attempts": stop_attempts,
            "stop_confirmed": stop_confirmed,
            "states": last_states,
            "positions": last_positions,
            "fault_tokens": fault_tokens,
            "errors": errors,
        }
        if deferred_interrupt is not None:
            try:
                deferred_interrupt.supervisor_stop_result = copy.deepcopy(
                    result
                )
            except Exception:
                pass
            raise deferred_interrupt
        return result

    def start_drive(
        self,
        left_speed_dps,
        right_speed_dps,
        duration_ms,
        pre_each_start=None,
    ):
        self._require_open()
        if self.active is not None:
            raise SupervisorError(
                "motion_already_active",
                "A motion command is already active",
            )
        if (
            pre_each_start is not None
            and not callable(pre_each_start)
        ):
            raise SupervisorError(
                "invalid_start_guard",
                "pre_each_start must be callable",
            )

        left_role, right_role, forward_signs = self.validate_drive(
            left_speed_dps,
            right_speed_dps,
            duration_ms,
        )
        left_path = self.robot._motor_path_for_role(left_role)
        right_path = self.robot._motor_path_for_role(right_role)
        if left_path == right_path:
            raise SupervisorError(
                "duplicate_motor_path",
                "Drive roles resolve to one motor",
            )

        motors = [
            {
                "side": "left",
                "role": left_role,
                "path": left_path,
                "logical_speed_dps": left_speed_dps,
                "physical_speed_dps": (
                    left_speed_dps * forward_signs[left_role]
                ),
            },
            {
                "side": "right",
                "role": right_role,
                "path": right_path,
                "logical_speed_dps": right_speed_dps,
                "physical_speed_dps": (
                    right_speed_dps * forward_signs[right_role]
                ),
            },
        ]

        try:
            for motor in motors:
                position = int(
                    read_text(os.path.join(motor["path"], "position"))
                )
                motor["position_before"] = position
                motor["checkpoint_position"] = position

            for motor in motors:
                write_text(
                    os.path.join(motor["path"], "speed_sp"),
                    motor["physical_speed_dps"],
                )
                write_text(
                    os.path.join(motor["path"], "time_sp"),
                    duration_ms,
                )
                write_text(
                    os.path.join(motor["path"], "stop_action"),
                    "brake",
                )

            start_times_ms = []
            for motor in motors:
                if pre_each_start is not None:
                    pre_each_start(
                        copy.deepcopy(motor),
                        tuple(start_times_ms),
                    )
                write_text(
                    os.path.join(motor["path"], "command"),
                    "run-timed",
                )
                started_ms = int(self.robot.monotonic_fn() * 1000)
                motor["start_write_ms"] = started_ms
                start_times_ms.append(started_ms)
        except BaseException as error:
            propagated = error
            try:
                cleanup = self.stop_all_verified()
            except BaseException as cleanup_error:
                cleanup = _stop_result_from_exception(cleanup_error)
                propagated = cleanup_error
                try:
                    propagated.supervisor_start_error = str(error)
                except Exception:
                    pass
            try:
                propagated.supervisor_start_cleanup = copy.deepcopy(
                    cleanup
                )
            except Exception:
                pass
            raise propagated

        started_at_ms = min(start_times_ms)
        for motor in motors:
            motor["checkpoint_at_ms"] = (
                started_at_ms
                + self.limits["stall_startup_grace_ms"]
            )
        self.active = {
            "motors": motors,
            "duration_ms": duration_ms,
            "started_at_ms": started_at_ms,
            "deadline_ms": started_at_ms + duration_ms,
        }
        return self.active_snapshot()

    def validate_drive(
        self,
        left_speed_dps,
        right_speed_dps,
        duration_ms,
    ):
        """Validate every drive argument without touching motor sysfs."""
        self._require_open()
        left_role, right_role, forward_signs = self.robot._drive_roles()
        self.robot._validate_motion(
            left_role,
            left_speed_dps,
            duration_ms,
        )
        self.robot._validate_motion(
            right_role,
            right_speed_dps,
            duration_ms,
        )
        return left_role, right_role, forward_signs

    def active_snapshot(self):
        self._require_open()
        if self.active is None:
            raise SupervisorError(
                "no_active_motion",
                "No motion command is active",
            )
        motors = []
        for configured in self.active["motors"]:
            raw_state = read_text(
                os.path.join(configured["path"], "state")
            )
            motors.append(
                {
                    "side": configured["side"],
                    "role": configured["role"],
                    "path": configured["path"],
                    "physical_speed_dps": configured[
                        "physical_speed_dps"
                    ],
                    "position_before": configured["position_before"],
                    "position": int(
                        read_text(
                            os.path.join(
                                configured["path"],
                                "position",
                            )
                        )
                    ),
                    "state": raw_state,
                }
            )
        return {
            "started_at_ms": self.active["started_at_ms"],
            "deadline_ms": self.active["deadline_ms"],
            "duration_ms": self.active["duration_ms"],
            "motors": motors,
        }

    def update_progress_checkpoint(self, role, position, now_ms):
        if self.active is None:
            return
        for motor in self.active["motors"]:
            if motor["role"] == role:
                motor["checkpoint_position"] = position
                motor["checkpoint_at_ms"] = now_ms
                return
        raise SupervisorError(
            "unknown_active_motor",
            "Active motion has no role {!r}".format(role),
        )

    def progress_checkpoint(self, role):
        if self.active is None:
            raise SupervisorError(
                "no_active_motion",
                "No motion command is active",
            )
        for motor in self.active["motors"]:
            if motor["role"] == role:
                return {
                    "position": motor["checkpoint_position"],
                    "at_ms": motor["checkpoint_at_ms"],
                }
        raise SupervisorError(
            "unknown_active_motor",
            "Active motion has no role {!r}".format(role),
        )

    def finish_active(self, verify_motion):
        self._require_open()
        if self.active is None:
            return {
                "stop": self.stop_all_verified(),
                "checks": [],
                "motors": [],
            }

        active = self.active
        self.active = None
        stop_result = self.stop_all_verified()
        observations = []
        checks = []
        read_errors = []

        for motor in active["motors"]:
            try:
                after = int(
                    read_text(os.path.join(motor["path"], "position"))
                )
                raw_state = read_text(
                    os.path.join(motor["path"], "state")
                )
                observation = {
                    "side": motor["side"],
                    "role": motor["role"],
                    "position_before": motor["position_before"],
                    "position_after": after,
                    "position_delta": (
                        after - motor["position_before"]
                    ),
                    "state": raw_state,
                }
                observations.append(observation)
                if verify_motion:
                    check = self.robot._encoder_check(
                        motor["role"],
                        motor["physical_speed_dps"],
                        active["duration_ms"],
                        motor["position_before"],
                        after,
                    )
                    ratio_minimum = int(
                        round(
                            check["expected_abs_delta"]
                            * self.limits[
                                "min_completion_ratio_percent"
                            ]
                            / 100.0
                        )
                    )
                    check["minimum_abs_delta"] = max(
                        check["minimum_abs_delta"],
                        ratio_minimum,
                    )
                    check["enough_motion"] = (
                        abs(check["position_delta"])
                        >= check["minimum_abs_delta"]
                    )
                    check["passed"] = (
                        check["direction_matches"]
                        and check["enough_motion"]
                    )
                    checks.append(check)
            except (IOError, OSError, ValueError) as error:
                read_errors.append(
                    "{}: {}".format(motor["role"], error)
                )

        stop_result["errors"].extend(read_errors)
        if verify_motion and not read_errors:
            try:
                self.robot._require_encoder_checks(checks)
            except MotionVerificationError as error:
                return {
                    "stop": stop_result,
                    "checks": error.checks,
                    "motors": observations,
                    "verification_error": str(error),
                }
        return {
            "stop": stop_result,
            "checks": checks,
            "motors": observations,
        }

    def close(self):
        if self.closed:
            return copy.deepcopy(self._close_result)
        result = self.finish_active(False)["stop"]
        if (
            not result["stop_confirmed"]
            or result["errors"]
            or result["fault_tokens"]
        ):
            return result
        try:
            import fcntl

            fcntl.flock(
                self._lock_handle.fileno(),
                fcntl.LOCK_UN,
            )
        finally:
            self._lock_handle.close()
            self.closed = True
            self._close_result = copy.deepcopy(result)
        return copy.deepcopy(self._close_result)


class EV3Supervisor(object):
    """Transport-independent deterministic supervisor state machine."""

    def __init__(
        self,
        robot,
        audit_buffer=None,
        session_id_factory=_default_session_id,
    ):
        if not callable(session_id_factory):
            raise SupervisorError(
                "invalid_session_factory",
                "Session factory must be callable",
            )
        self.robot = robot
        self._session_id_factory = session_id_factory
        self.limits = self._validated_limits(robot.config)
        if audit_buffer is None:
            audit_buffer = AuditBuffer(
                self.limits["audit_buffer_events"]
            )
        if not isinstance(audit_buffer, AuditBuffer):
            raise SupervisorError(
                "invalid_audit_buffer",
                "Audit buffer has an invalid type",
            )
        if audit_buffer.maximum_events < 2:
            raise SupervisorError(
                "audit_buffer_too_small",
                "Audit buffer must reserve a terminal event slot",
            )
        if audit_buffer.snapshot():
            raise SupervisorError(
                "audit_buffer_not_empty",
                "Audit buffer must be empty at supervisor startup",
            )
        self._audit_buffer = audit_buffer
        self.state = STATE_BOOTING
        self.fault = None
        self.owner_id = None
        self.session_id = None
        self.last_sequence_id = 0
        self.last_heartbeat_ms = None
        self.last_heartbeat_sequence = None
        self.claimed_at_ms = None
        self.last_poll_ms = None
        self.touch_value = None
        self.touch_released_samples = 0
        self.active_command_id = None
        self._seen_command_ids = set()
        self._event_sequence = 0
        self._closed = False
        self._close_status = None
        self._dispatch_thread_id = threading.current_thread().ident
        self._owner = SupervisorMotorOwner(robot, self.limits)

        stop_result = self._owner.stop_all_verified()
        try:
            startup_snapshots = self._owner.snapshot_all()
            for snapshot in startup_snapshots:
                tokens = _state_tokens(snapshot["state"])
                if tokens & (
                    ACTIVE_MOTOR_STATES | FAULT_MOTOR_STATES
                ):
                    raise SupervisorError(
                        "unsafe_startup_motor_state",
                        "A motor is not safely inactive at startup",
                    )
        except (
            IOError,
            OSError,
            RuntimeError,
            ValueError,
            SupervisorError,
        ) as error:
            stop_result["errors"].append(
                "inventory: {}".format(error)
            )
        if (
            not stop_result["stop_confirmed"]
            or stop_result["errors"]
        ):
            self._enter_fault(
                "startup_stop_failed",
                "Startup stop could not be verified",
                stop_result=stop_result,
                emit=False,
            )
            self._audit(
                "startup_fault",
                stop=stop_result,
            )
        else:
            self.state = STATE_DISARMED
            self._audit(
                "startup_complete",
                stop=stop_result,
            )

    @staticmethod
    def _validated_limits(config):
        try:
            supervisor = dict(config["limits"]["supervisor"])
            heartbeat_timeout_ms = config["limits"]["heartbeat"][
                "timeout_ms"
            ]
            drive_max_speed_dps = config["limits"]["drive"][
                "max_abs_speed_dps"
            ]
            drive_max_duration_ms = config["limits"]["drive"][
                "max_duration_ms"
            ]
        except (KeyError, TypeError, ValueError):
            raise SupervisorError(
                "invalid_supervisor_config",
                "Supervisor configuration is incomplete",
            )

        required = (
            "poll_interval_ms",
            "max_poll_lateness_ms",
            "touch_release_samples",
            "min_abs_drive_speed_dps",
            "stall_startup_grace_ms",
            "stall_window_ms",
            "stall_min_progress_degrees",
            "stall_min_progress_ratio_percent",
            "min_completion_ratio_percent",
            "max_start_skew_ms",
            "stop_verify_timeout_ms",
            "stop_poll_interval_ms",
            "max_commands_per_session",
            "audit_buffer_events",
        )
        for name in required:
            _validate_positive_int(name, supervisor.get(name))
        _validate_positive_int(
            "heartbeat_timeout_ms",
            heartbeat_timeout_ms,
        )
        _validate_positive_int(
            "drive_max_speed_dps",
            drive_max_speed_dps,
        )
        _validate_positive_int(
            "drive_max_duration_ms",
            drive_max_duration_ms,
        )
        if (
            supervisor["stop_poll_interval_ms"]
            > supervisor["stop_verify_timeout_ms"]
        ):
            raise SupervisorError(
                "invalid_supervisor_config",
                "Stop poll interval exceeds stop timeout",
            )
        if (
            supervisor["poll_interval_ms"]
            >= heartbeat_timeout_ms
        ):
            raise SupervisorError(
                "invalid_supervisor_config",
                "Poll interval must be shorter than heartbeat timeout",
            )
        if (
            supervisor["max_poll_lateness_ms"]
            >= heartbeat_timeout_ms
        ):
            raise SupervisorError(
                "invalid_supervisor_config",
                "Poll lateness limit must be shorter than heartbeat timeout",
            )
        if (
            supervisor["min_abs_drive_speed_dps"]
            > drive_max_speed_dps
        ):
            raise SupervisorError(
                "invalid_supervisor_config",
                "Supervised speed floor exceeds drive speed limit",
            )
        for name in (
            "stall_min_progress_ratio_percent",
            "min_completion_ratio_percent",
        ):
            if supervisor[name] > 100:
                raise SupervisorError(
                    "invalid_supervisor_config",
                    "{} must be at most 100".format(name),
                )
        if (
            supervisor["stall_startup_grace_ms"]
            + supervisor["stall_window_ms"]
            > drive_max_duration_ms
        ):
            raise SupervisorError(
                "invalid_supervisor_config",
                "Stall detection cannot run within maximum drive duration",
            )
        supervisor["heartbeat_timeout_ms"] = heartbeat_timeout_ms
        return supervisor

    @property
    def audit_events(self):
        return self._audit_buffer.snapshot()

    def drain_audit_events(self):
        return self._audit_buffer.drain()

    def _now_ms(self):
        return int(self.robot.monotonic_fn() * 1000)

    def bind_to_current_thread(self):
        thread_id = threading.current_thread().ident
        if self._dispatch_thread_id is None:
            self._dispatch_thread_id = thread_id
        elif self._dispatch_thread_id != thread_id:
            raise SupervisorError(
                "wrong_dispatch_thread",
                "Supervisor is bound to another dispatch thread",
            )
        return thread_id

    def _require_dispatch_thread(self):
        if (
            self._dispatch_thread_id is not None
            and self._dispatch_thread_id
            != threading.current_thread().ident
        ):
            raise SupervisorError(
                "wrong_dispatch_thread",
                "Mutation must run on the supervisor dispatch thread",
            )

    def _checked_now_ms(self):
        now_ms = self._now_ms()
        if (
            self.last_poll_ms is not None
            and now_ms < self.last_poll_ms
        ):
            raise SupervisorError(
                "clock_rollback",
                "Monotonic clock moved backwards",
            )
        self.last_poll_ms = now_ms
        return now_ms

    def _audit(self, event_type, **details):
        self._event_sequence += 1
        event = {
            "schema": AUDIT_SCHEMA,
            "event_sequence": self._event_sequence,
            "observed_monotonic_ms": self._now_ms(),
            "event": event_type,
            "state": self.state,
            "owner_id": self.owner_id,
            "session_fingerprint": self._session_fingerprint(),
            "last_sequence_id": self.last_sequence_id,
            "active_command_id": self.active_command_id,
            "fault": self.fault,
        }
        event.update(details)
        try:
            self._audit_buffer.append(
                event,
                terminal=event_type == "supervisor_closed",
            )
        except Exception as error:
            if self.state not in (
                STATE_SHUTTING_DOWN,
                STATE_CLOSED,
            ):
                self._enter_fault(
                    "audit_failure",
                    "Audit sink failed: {}".format(error),
                    emit=False,
                )
            raise SupervisorError(
                "audit_failure",
                "Audit sink failed",
            )
        return event

    def _session_fingerprint(self):
        if self.session_id is None:
            return None
        return hashlib.sha256(
            self.session_id.encode("utf-8")
        ).hexdigest()[:12]

    def _invalidate_session(self):
        self.owner_id = None
        self.session_id = None
        self.last_sequence_id = 0
        self.last_heartbeat_ms = None
        self.last_heartbeat_sequence = None
        self.claimed_at_ms = None
        self._seen_command_ids = set()

    def _enter_fault(
        self,
        code,
        detail,
        stop_result=None,
        emit=True,
    ):
        if self.state == STATE_FAULT_LATCHED:
            deferred_error = None
            try:
                retry = self._owner.finish_active(False)["stop"]
                self.fault["stop_confirmed"] = retry[
                    "stop_confirmed"
                ]
                self.fault["stop_errors"] = list(
                    retry["errors"]
                )
                self.fault["fault_tokens"] = copy.deepcopy(
                    retry.get("fault_tokens", {})
                )
                self.fault.setdefault(
                    "stop_history",
                    [],
                ).append(
                    {
                        "phase": "fault_retry",
                        "result": copy.deepcopy(retry),
                    }
                )
            except BaseException as error:
                retry = _stop_result_from_exception(error)
                self.fault["stop_confirmed"] = retry.get(
                    "stop_confirmed",
                    False,
                )
                self.fault["stop_errors"] = list(
                    retry.get("errors", [])
                )
                self.fault["fault_tokens"] = copy.deepcopy(
                    retry.get("fault_tokens", {})
                )
                self.fault.setdefault(
                    "stop_history",
                    [],
                ).append(
                    {
                        "phase": "fault_retry_interrupted",
                        "result": copy.deepcopy(retry),
                    }
                )
                if not isinstance(error, Exception):
                    deferred_error = error
            if deferred_error is not None:
                raise deferred_error
            return
        deferred_error = None
        if stop_result is None:
            try:
                stop_result = self._owner.finish_active(False)["stop"]
            except BaseException as error:
                stop_result = _stop_result_from_exception(error)
                if not isinstance(error, Exception):
                    deferred_error = error
        self.active_command_id = None
        self._invalidate_session()
        stop_history = copy.deepcopy(
            stop_result.get("stop_history", [])
        )
        if not stop_history:
            history_result = copy.deepcopy(stop_result)
            history_result.pop("stop_history", None)
            stop_history.append(
                {
                    "phase": "fault_entry",
                    "result": history_result,
                }
            )
        self.fault = {
            "code": code,
            "detail": detail,
            "stop_confirmed": stop_result.get(
                "stop_confirmed",
                False,
            ),
            "stop_errors": list(stop_result.get("errors", [])),
            "fault_tokens": copy.deepcopy(
                stop_result.get("fault_tokens", {})
            ),
            "stop_history": stop_history,
        }
        self.state = STATE_FAULT_LATCHED
        if emit:
            try:
                self._audit(
                    "fault_latched",
                    reason_code=code,
                    detail=detail,
                    stop=stop_result,
                )
            except SupervisorError:
                pass
        if deferred_error is not None:
            raise deferred_error

    def _require_state(self, *states):
        if self.state not in states:
            raise SupervisorError(
                "wrong_state",
                "Operation is not allowed in state {}".format(
                    self.state
                ),
            )

    def _require_session(self, session_id):
        _validate_identifier("session_id", session_id, 128)
        if (
            self.session_id is None
            or session_id != self.session_id
        ):
            raise SupervisorError(
                "wrong_session",
                "Session is not active",
            )

    def _consume_sequence(self, sequence_id):
        self._validate_sequence(sequence_id)
        self.last_sequence_id = sequence_id

    def _validate_sequence(self, sequence_id):
        _validate_positive_int(
            "sequence_id",
            sequence_id,
            2147483647,
        )
        if sequence_id <= self.last_sequence_id:
            raise SupervisorError(
                "replayed_sequence",
                "sequence_id must increase strictly",
            )

    def _heartbeat_age_ms(self, now_ms):
        if self.last_heartbeat_ms is None:
            return None
        return now_ms - self.last_heartbeat_ms

    def _heartbeat_is_fresh(self, now_ms):
        age_ms = self._heartbeat_age_ms(now_ms)
        return (
            age_ms is not None
            and age_ms >= 0
            and age_ms < self.limits["heartbeat_timeout_ms"]
        )

    def _read_touch(self):
        reading = self.robot.read_sensor("touch")
        value = reading.get("value0")
        if not _is_int(value) or value not in (0, 1):
            raise SupervisorError(
                "invalid_touch",
                "Touch sensor must report exactly 0 or 1",
            )
        self.touch_value = value
        if value == 0:
            self.touch_released_samples += 1
        else:
            self.touch_released_samples = 0
        return reading

    def _check_idle_motors(self):
        snapshots = self._owner.snapshot_all()
        for snapshot in snapshots:
            tokens = _state_tokens(snapshot["state"])
            if tokens & ACTIVE_MOTOR_STATES:
                raise SupervisorError(
                    "unexpected_external_motion",
                    "Motor {} is active while supervisor is idle".format(
                        snapshot["address"]
                    ),
                )
            if tokens & FAULT_MOTOR_STATES:
                raise SupervisorError(
                    "motor_state_fault",
                    "Motor {} reports {}".format(
                        snapshot["address"],
                        sorted(tokens & FAULT_MOTOR_STATES),
                    ),
                )
        return snapshots

    def _check_running_motion(self):
        snapshot = self._owner.active_snapshot()
        all_motors = self._owner.snapshot_all()
        self._read_touch()
        now_ms = self._checked_now_ms()
        active_paths = frozenset(
            motor["path"] for motor in snapshot["motors"]
        )
        for observed in all_motors:
            tokens = _state_tokens(observed["state"])
            if tokens & FAULT_MOTOR_STATES:
                raise SupervisorError(
                    "motor_state_fault",
                    "Motor {} reports {}".format(
                        observed["address"],
                        sorted(tokens & FAULT_MOTOR_STATES),
                    ),
                )
            if (
                tokens & ACTIVE_MOTOR_STATES
                and observed["path"] not in active_paths
            ):
                raise SupervisorError(
                    "unexpected_external_motion",
                    "A motor outside the active command is running",
                )
        if self.touch_value == 1:
            raise SupervisorError(
                "touch_pressed",
                "Touch stop input is pressed",
            )
        if not self._heartbeat_is_fresh(now_ms):
            raise SupervisorError(
                "heartbeat_timeout",
                "Heartbeat expired during motion",
            )

        for motor in snapshot["motors"]:
            tokens = _state_tokens(motor["state"])
            if tokens & FAULT_MOTOR_STATES:
                raise SupervisorError(
                    "motor_state_fault",
                    "Motor {} reports {}".format(
                        motor["role"],
                        sorted(tokens & FAULT_MOTOR_STATES),
                    ),
                )
            if (
                now_ms
                >= snapshot["started_at_ms"]
                + self.limits["stall_startup_grace_ms"]
                and now_ms
                < snapshot["deadline_ms"]
                - self.limits["poll_interval_ms"]
                and not tokens & ACTIVE_MOTOR_STATES
            ):
                raise SupervisorError(
                    "unexpected_early_stop",
                    "Motor {} stopped before its local deadline".format(
                        motor["role"]
                    ),
                )

            total_delta = (
                motor["position"] - motor["position_before"]
            )
            if (
                abs(total_delta) >= 1
                and total_delta * motor["physical_speed_dps"] < 0
            ):
                raise SupervisorError(
                    "encoder_direction",
                    "Motor {} moved in the wrong direction".format(
                        motor["role"]
                    ),
                )

            checkpoint = self._owner.progress_checkpoint(
                motor["role"]
            )
            checkpoint_delta = (
                motor["position"] - checkpoint["position"]
            )
            direction_progress = (
                checkpoint_delta * motor["physical_speed_dps"] > 0
            )
            expected_window_progress = int(
                round(
                    abs(motor["physical_speed_dps"])
                    * self.limits["stall_window_ms"]
                    / 1000.0
                )
            )
            ratio_progress = int(
                round(
                    expected_window_progress
                    * self.limits[
                        "stall_min_progress_ratio_percent"
                    ]
                    / 100.0
                )
            )
            required_progress = max(
                self.limits["stall_min_progress_degrees"],
                ratio_progress,
            )
            if (
                direction_progress
                and abs(checkpoint_delta)
                >= required_progress
            ):
                self._owner.update_progress_checkpoint(
                    motor["role"],
                    motor["position"],
                    now_ms,
                )
            elif (
                now_ms >= checkpoint["at_ms"]
                and now_ms - checkpoint["at_ms"]
                >= self.limits["stall_window_ms"]
            ):
                raise SupervisorError(
                    "encoder_stall",
                    "Motor {} made no verified progress".format(
                        motor["role"]
                    ),
                )
        snapshot["observed_at_ms"] = now_ms
        return snapshot

    def poll_once(self):
        self._require_dispatch_thread()
        if self._closed:
            raise SupervisorError(
                "supervisor_closed",
                "Supervisor is closed",
            )

        if self.state == STATE_FAULT_LATCHED:
            secondary_errors = []
            try:
                self._checked_now_ms()
            except SupervisorError as error:
                secondary_errors.append(str(error))
            try:
                retry = self._owner.finish_active(False)["stop"]
                self.fault["stop_confirmed"] = retry[
                    "stop_confirmed"
                ]
                self.fault["stop_errors"] = list(retry["errors"])
                self.fault["fault_tokens"] = copy.deepcopy(
                    retry["fault_tokens"]
                )
            except Exception as error:
                self.fault["stop_confirmed"] = False
                self.fault["stop_errors"] = [str(error)]
            try:
                self._read_touch()
                self._checked_now_ms()
            except Exception as error:
                secondary_errors.append(str(error))
            if secondary_errors:
                self.fault["secondary_errors"] = secondary_errors
            return self.status()

        try:
            now_ms = self._checked_now_ms()
            self._read_touch()
            now_ms = self._checked_now_ms()
            if self.state == STATE_RUNNING:
                running = self._check_running_motion()
                now_ms = running["observed_at_ms"]
                if now_ms >= running["deadline_ms"]:
                    result = self._owner.finish_active(True)
                    if (
                        not result["stop"]["stop_confirmed"]
                        or result["stop"]["errors"]
                        or result.get("verification_error")
                    ):
                        self._enter_fault(
                            "motion_completion_failed",
                            "Motion could not be safely verified",
                            stop_result=result["stop"],
                        )
                    else:
                        completed_command_id = self.active_command_id
                        self.active_command_id = None
                        self.state = STATE_ARMED_IDLE
                        self._audit(
                            "motion_completed",
                            command_id=completed_command_id,
                            result=result,
                        )
            elif self.state in (
                STATE_DISARMED,
                STATE_ARMED_IDLE,
            ):
                self._check_idle_motors()
                now_ms = self._checked_now_ms()
                if (
                    self.state == STATE_ARMED_IDLE
                    and self.touch_value == 1
                ):
                    raise SupervisorError(
                        "touch_pressed",
                        "Touch stop input is pressed while armed",
                    )
                if (
                    self.state == STATE_ARMED_IDLE
                    and not self._heartbeat_is_fresh(now_ms)
                ):
                    raise SupervisorError(
                        "heartbeat_timeout",
                        "Heartbeat expired while armed",
                    )
                if (
                    self.state == STATE_DISARMED
                    and self.session_id is not None
                ):
                    lease_reference_ms = (
                        self.last_heartbeat_ms
                        if self.last_heartbeat_ms is not None
                        else self.claimed_at_ms
                    )
                    if (
                        lease_reference_ms is not None
                        and now_ms - lease_reference_ms
                        >= self.limits["heartbeat_timeout_ms"]
                    ):
                        expired_owner = self.owner_id
                        self._invalidate_session()
                        self._audit(
                            "unarmed_lease_expired",
                            expired_owner_id=expired_owner,
                        )
        except (
            IOError,
            OSError,
            RuntimeError,
            ValueError,
            SupervisorError,
        ) as error:
            code = getattr(error, "code", "poll_failure")
            self._enter_fault(code, str(error))
        return self.status()

    def claim(self, owner_id):
        self._require_dispatch_thread()
        self._require_state(STATE_DISARMED)
        if self.session_id is not None:
            raise SupervisorError(
                "owner_exists",
                "A session is already claimed",
            )
        _validate_identifier("owner_id", owner_id, 64)
        session_id = self._session_id_factory()
        _validate_identifier("session_id", session_id, 128)
        self.owner_id = owner_id
        self.session_id = session_id
        self.last_sequence_id = 0
        self.last_heartbeat_ms = None
        self.last_heartbeat_sequence = None
        self.claimed_at_ms = self._checked_now_ms()
        self._seen_command_ids = set()
        self._audit("session_claimed")
        return {
            "status": "claimed",
            "owner_id": owner_id,
            "session_id": session_id,
            "state": self.state,
        }

    def heartbeat(self, session_id, sequence_id):
        self._require_dispatch_thread()
        self._require_state(
            STATE_DISARMED,
            STATE_ARMED_IDLE,
            STATE_RUNNING,
        )
        self._require_session(session_id)
        self._consume_sequence(sequence_id)
        self.last_heartbeat_ms = self._now_ms()
        self.last_heartbeat_sequence = sequence_id
        self._audit(
            "heartbeat",
            sequence_id=sequence_id,
        )
        return {
            "status": "accepted",
            "sequence_id": sequence_id,
            "heartbeat_timeout_ms": self.limits[
                "heartbeat_timeout_ms"
            ],
        }

    def arm(self, session_id, sequence_id):
        self._require_dispatch_thread()
        self._require_state(STATE_DISARMED)
        self._require_session(session_id)
        self._validate_sequence(sequence_id)
        self.poll_once()
        self._require_state(STATE_DISARMED)
        now_ms = self._checked_now_ms()
        if not self._heartbeat_is_fresh(now_ms):
            raise SupervisorError(
                "heartbeat_required",
                "A fresh explicit heartbeat is required",
            )
        if (
            self.touch_value != 0
            or self.touch_released_samples
            < self.limits["touch_release_samples"]
        ):
            raise SupervisorError(
                "touch_not_released",
                "Touch must be released for stable samples",
            )
        self._check_idle_motors()
        now_ms = self._checked_now_ms()
        if not self._heartbeat_is_fresh(now_ms):
            raise SupervisorError(
                "heartbeat_required",
                "Heartbeat expired during arm preflight",
            )
        self._consume_sequence(sequence_id)
        self.state = STATE_ARMED_IDLE
        self._audit(
            "armed",
            sequence_id=sequence_id,
            heartbeat_age_ms=self._heartbeat_age_ms(now_ms),
        )
        return self.status()

    def _guard_individual_motor_start(
        self,
        motor,
        prior_start_times_ms,
    ):
        self._read_touch()
        now_ms = self._checked_now_ms()
        if self.touch_value != 0:
            raise SupervisorError(
                "touch_pressed",
                "Touch stop input changed between motor starts",
            )
        if not self._heartbeat_is_fresh(now_ms):
            raise SupervisorError(
                "heartbeat_timeout",
                "Heartbeat expired between motor starts",
            )
        if (
            prior_start_times_ms
            and now_ms - min(prior_start_times_ms)
            > self.limits["max_start_skew_ms"]
        ):
            raise SupervisorError(
                "start_skew",
                "Second motor start missed the local skew limit",
            )
        allowed_active_paths = set()
        if prior_start_times_ms:
            left_role, right_role, _signs = self.robot._drive_roles()
            for role in (left_role, right_role):
                path = self.robot._motor_path_for_role(role)
                if path != motor["path"]:
                    allowed_active_paths.add(path)
        for observed in self._owner.snapshot_all():
            tokens = _state_tokens(observed["state"])
            if tokens & FAULT_MOTOR_STATES:
                raise SupervisorError(
                    "motor_state_fault",
                    "A motor fault appeared between motor starts",
                )
            if (
                tokens & ACTIVE_MOTOR_STATES
                and observed["path"] not in allowed_active_paths
            ):
                raise SupervisorError(
                    "unexpected_external_motion",
                    "Unexpected motion appeared between motor starts",
                )
        now_ms = self._checked_now_ms()
        if not self._heartbeat_is_fresh(now_ms):
            raise SupervisorError(
                "heartbeat_timeout",
                "Heartbeat expired during motor start guard",
            )
        if (
            prior_start_times_ms
            and now_ms - min(prior_start_times_ms)
            > self.limits["max_start_skew_ms"]
        ):
            raise SupervisorError(
                "start_skew",
                "Motor inventory check exceeded the start skew limit",
            )

    def start_drive(
        self,
        session_id,
        sequence_id,
        command_id,
        reference_heartbeat_sequence,
        left_speed_dps,
        right_speed_dps,
        duration_ms,
    ):
        self._require_dispatch_thread()
        self._require_state(STATE_ARMED_IDLE)
        self._require_session(session_id)
        self._validate_sequence(sequence_id)
        _validate_identifier("command_id", command_id, 128)
        if command_id in self._seen_command_ids:
            raise SupervisorError(
                "duplicate_command_id",
                "command_id has already been consumed",
            )
        if (
            len(self._seen_command_ids)
            >= self.limits["max_commands_per_session"]
        ):
            raise SupervisorError(
                "session_command_budget",
                "Session command budget is exhausted",
            )
        _validate_positive_int(
            "reference_heartbeat_sequence",
            reference_heartbeat_sequence,
            2147483647,
        )
        if (
            reference_heartbeat_sequence
            != self.last_heartbeat_sequence
        ):
            raise SupervisorError(
                "stale_heartbeat_reference",
                "Motion must reference the latest heartbeat",
            )
        for speed in (left_speed_dps, right_speed_dps):
            if (
                not _is_int(speed)
                or abs(speed)
                < self.limits["min_abs_drive_speed_dps"]
            ):
                raise SupervisorError(
                    "drive_speed_floor",
                    "Drive speed is below the supervised minimum",
                )
        try:
            self._owner.validate_drive(
                left_speed_dps,
                right_speed_dps,
                duration_ms,
            )
        except SupervisorError:
            raise
        except SafetyError as error:
            raise SupervisorError(
                "invalid_motion",
                str(error),
            )

        self.poll_once()
        self._require_state(STATE_ARMED_IDLE)
        now_ms = self._checked_now_ms()
        if not self._heartbeat_is_fresh(now_ms):
            raise SupervisorError(
                "heartbeat_timeout",
                "Heartbeat is not fresh",
            )
        if self.touch_value != 0:
            raise SupervisorError(
                "touch_pressed",
                "Touch stop input is pressed",
            )

        self._consume_sequence(sequence_id)
        self._seen_command_ids.add(command_id)
        self._audit(
            "command_accepted",
            command_id=command_id,
            sequence_id=sequence_id,
            reference_heartbeat_sequence=(
                reference_heartbeat_sequence
            ),
            primitive="drive_timed",
            arguments={
                "left_speed_dps": left_speed_dps,
                "right_speed_dps": right_speed_dps,
                "duration_ms": duration_ms,
            },
        )
        try:
            self._read_touch()
            self._check_idle_motors()
            now_ms = self._checked_now_ms()
            if self.touch_value != 0:
                raise SupervisorError(
                    "touch_pressed",
                    "Touch stop input changed before motor start",
                )
            if not self._heartbeat_is_fresh(now_ms):
                raise SupervisorError(
                    "heartbeat_timeout",
                    "Heartbeat expired before motor start",
                )
        except BaseException as error:
            code = getattr(error, "code", "prestart_failure")
            self._enter_fault(code, str(error))
            if not isinstance(error, Exception):
                raise
            raise SupervisorError(
                code,
                "Motion prestart check failed",
            )
        try:
            motion = self._owner.start_drive(
                left_speed_dps,
                right_speed_dps,
                duration_ms,
                pre_each_start=(
                    self._guard_individual_motor_start
                ),
            )
        except BaseException as error:
            first_cleanup = getattr(
                error,
                "supervisor_start_cleanup",
                None,
            )
            try:
                retry_cleanup = self._owner.finish_active(False)[
                    "stop"
                ]
            except BaseException as cleanup_error:
                retry_cleanup = _stop_result_from_exception(
                    cleanup_error
                )
            stop_result = _combine_stop_results(
                (
                    ("motor_start_cleanup", first_cleanup),
                    ("fault_cleanup_retry", retry_cleanup),
                )
            )
            code = getattr(error, "code", "motion_start_failed")
            self._enter_fault(
                code,
                str(error),
                stop_result=stop_result,
            )
            if not isinstance(error, Exception):
                raise
            raise SupervisorError(
                code,
                "Motion start failed",
            )

        self.active_command_id = command_id
        self.state = STATE_RUNNING
        start_times = [
            motor["start_write_ms"]
            for motor in self._owner.active["motors"]
        ]
        start_skew_ms = max(start_times) - min(start_times)
        try:
            running = self._check_running_motion()
            now_ms = running["observed_at_ms"]
            if (
                start_skew_ms > self.limits["max_start_skew_ms"]
            ):
                raise SupervisorError(
                    "start_skew",
                    "Motor start skew exceeded local limit",
                )
            if self.touch_value != 0:
                raise SupervisorError(
                    "touch_pressed",
                    "Touch stop input changed during motor start",
                )
            if not self._heartbeat_is_fresh(now_ms):
                raise SupervisorError(
                    "heartbeat_timeout",
                    "Heartbeat expired during motor start",
                )
        except BaseException as error:
            code = getattr(error, "code", "poststart_failure")
            self._enter_fault(code, str(error))
            if not isinstance(error, Exception):
                raise
            raise SupervisorError(
                code,
                "Motion was stopped during post-start check",
            )
        self._audit(
            "motion_started",
            command_id=command_id,
            start_skew_ms=start_skew_ms,
            motion=motion,
        )
        return self.status()

    def _finish_active_with_retry(self, phase):
        try:
            return self._owner.finish_active(False)["stop"], None
        except BaseException as error:
            first = _stop_result_from_exception(error)
            try:
                retry = self._owner.finish_active(False)["stop"]
            except BaseException as retry_error:
                retry = _stop_result_from_exception(retry_error)
            return (
                _combine_stop_results(
                    (
                        (phase, first),
                        ("{}_retry".format(phase), retry),
                    )
                ),
                error,
            )

    def stop(self):
        """Unauthenticated local emergency stop; always safe to call."""
        self._require_dispatch_thread()
        if self._closed:
            return {
                "status": "closed",
                "state": STATE_CLOSED,
            }
        was_faulted = self.state == STATE_FAULT_LATCHED
        previous_fault = self.fault
        result, stop_error = self._finish_active_with_retry(
            "emergency_stop"
        )
        self.active_command_id = None
        self._invalidate_session()
        if was_faulted:
            self.state = STATE_FAULT_LATCHED
            self.fault = previous_fault
            self.fault["stop_confirmed"] = result["stop_confirmed"]
            self.fault["stop_errors"] = list(result["errors"])
            self.fault["fault_tokens"] = copy.deepcopy(
                result["fault_tokens"]
            )
            self._audit("fault_stop_retried", stop=result)
        elif (
            stop_error is None
            and result["stop_confirmed"]
            and not result["errors"]
        ):
            self.fault = None
            self.state = STATE_DISARMED
            self._audit("emergency_stop", stop=result)
        else:
            self._enter_fault(
                "stop_failed",
                "Emergency stop was not verified",
                stop_result=result,
            )
        if (
            stop_error is not None
            and not isinstance(stop_error, Exception)
        ):
            raise stop_error
        return self.status()

    def release(self, session_id, sequence_id):
        self._require_dispatch_thread()
        self._require_state(
            STATE_DISARMED,
            STATE_ARMED_IDLE,
            STATE_RUNNING,
        )
        self._require_session(session_id)
        self._consume_sequence(sequence_id)
        result, stop_error = self._finish_active_with_retry(
            "session_release"
        )
        released_owner = self.owner_id
        self.active_command_id = None
        self._invalidate_session()
        if (
            stop_error is None
            and result["stop_confirmed"]
            and not result["errors"]
        ):
            self.state = STATE_DISARMED
            self.fault = None
            self._audit(
                "session_released",
                released_owner_id=released_owner,
                stop=result,
            )
        else:
            self._enter_fault(
                "release_stop_failed",
                "Release stop was not verified",
                stop_result=result,
            )
        if (
            stop_error is not None
            and not isinstance(stop_error, Exception)
        ):
            raise stop_error
        return self.status()

    def reset_fault(self):
        self._require_dispatch_thread()
        self._require_state(STATE_FAULT_LATCHED)
        self.poll_once()
        self._require_state(STATE_FAULT_LATCHED)
        if (
            self.touch_value != 0
            or self.touch_released_samples
            < self.limits["touch_release_samples"]
        ):
            raise SupervisorError(
                "touch_not_released",
                "Touch must be stably released before reset",
            )
        stop_result = self._owner.stop_all_verified()
        if (
            not stop_result["stop_confirmed"]
            or stop_result["errors"]
        ):
            raise SupervisorError(
                "stop_not_confirmed",
                "Fault cannot reset before verified stop",
            )
        old_fault = self.fault
        self.fault = None
        self.state = STATE_DISARMED
        self._audit(
            "fault_reset",
            previous_fault=old_fault,
            stop=stop_result,
        )
        return self.status()

    def status(self):
        now_ms = self._now_ms()
        motion_allowed = (
            self.state == STATE_ARMED_IDLE
            and self.touch_value == 0
            and self._heartbeat_is_fresh(now_ms)
        )
        return {
            "status": "ok" if self.fault is None else "fault",
            "state": self.state,
            "motion_allowed": motion_allowed,
            "owner_id": self.owner_id,
            "session_active": self.session_id is not None,
            "last_sequence_id": self.last_sequence_id,
            "last_heartbeat_sequence": self.last_heartbeat_sequence,
            "heartbeat_age_ms": self._heartbeat_age_ms(now_ms),
            "touch": self.touch_value,
            "touch_released_samples": self.touch_released_samples,
            "active_command_id": self.active_command_id,
            "fault": self.fault,
        }

    def _finalize_closed(self, result):
        self.active_command_id = None
        self._invalidate_session()
        self._closed = True
        self.state = STATE_CLOSED
        audit_complete = True
        audit_error = None
        try:
            self._audit(
                "supervisor_closed",
                stop=result,
            )
        except BaseException as error:
            audit_complete = False
            audit_error = error
        self._close_status = self.status()
        self._close_status["audit_complete"] = audit_complete
        completed = copy.deepcopy(self._close_status)
        if (
            audit_error is not None
            and not isinstance(audit_error, Exception)
        ):
            raise audit_error
        return completed

    def close(self):
        self._require_dispatch_thread()
        if self._closed:
            return copy.deepcopy(self._close_status)
        self.state = STATE_SHUTTING_DOWN
        try:
            result = self._owner.close()
        except BaseException as error:
            first = _stop_result_from_exception(error)
            try:
                retry = self._owner.close()
            except BaseException as retry_error:
                retry = _stop_result_from_exception(retry_error)
            if (
                self._owner.closed
                and retry.get("stop_confirmed") is True
                and not retry.get("errors", [])
                and not retry.get("fault_tokens", {})
            ):
                recovered = copy.deepcopy(retry)
                recovered["stop_history"] = [
                    {
                        "phase": "shutdown_interrupted",
                        "result": copy.deepcopy(first),
                    },
                    {
                        "phase": "shutdown_retry",
                        "result": copy.deepcopy(retry),
                    },
                ]
                completed = self._finalize_closed(recovered)
                if not isinstance(error, Exception):
                    raise error
                return completed
            result = _combine_stop_results(
                (
                    ("shutdown", first),
                    ("shutdown_retry", retry),
                )
            )
            self._enter_fault(
                "shutdown_stop_failed",
                "Shutdown was interrupted; lock retained unless closed",
                stop_result=result,
            )
            if not isinstance(error, Exception):
                raise error
            return self.status()
        if (
            not result["stop_confirmed"]
            or result["errors"]
            or result["fault_tokens"]
        ):
            self._enter_fault(
                "shutdown_stop_failed",
                "Shutdown stop was not verified; lock retained",
                stop_result=result,
            )
            return self.status()
        return self._finalize_closed(result)


class EV3SupervisorLoop(object):
    """Absolute-deadline polling loop independent of any remote client."""

    def __init__(self, supervisor):
        if not isinstance(supervisor, EV3Supervisor):
            raise SupervisorError(
                "invalid_supervisor",
                "Loop requires EV3Supervisor",
            )
        self.supervisor = supervisor
        self.interval_ms = supervisor.limits["poll_interval_ms"]
        self.max_lateness_ms = supervisor.limits[
            "max_poll_lateness_ms"
        ]
        previous_safety_check_ms = supervisor.last_poll_ms
        now_ms = supervisor._checked_now_ms()
        self._next_tick_ms = (
            previous_safety_check_ms
            if previous_safety_check_ms is not None
            else now_ms
        )
        self._emergency_stop_requested = threading.Event()

    def request_emergency_stop(self):
        """Thread-safe signal; the dispatch thread performs the stop."""
        self._emergency_stop_requested.set()

    def run_once(self):
        self.supervisor.bind_to_current_thread()
        now_ms = self.supervisor._checked_now_ms()
        pre_poll_lateness_ms = now_ms - self._next_tick_ms
        if pre_poll_lateness_ms > self.max_lateness_ms:
            self.supervisor._enter_fault(
                "poll_deadline_missed",
                "Supervisor woke {} ms after its poll deadline".format(
                    pre_poll_lateness_ms
                ),
            )
            self._next_tick_ms = now_ms + self.interval_ms
            self.supervisor.robot.sleep_fn(
                self.interval_ms / 1000.0
            )
            return self.supervisor.status()

        if self._emergency_stop_requested.is_set():
            self._emergency_stop_requested.clear()
            self.supervisor.stop()

        self.supervisor.poll_once()
        self._next_tick_ms += self.interval_ms
        now_ms = self.supervisor._checked_now_ms()
        lateness_ms = now_ms - self._next_tick_ms
        if lateness_ms > self.max_lateness_ms:
            self.supervisor._enter_fault(
                "poll_deadline_missed",
                "Supervisor poll deadline missed by {} ms".format(
                    lateness_ms
                ),
            )
            self._next_tick_ms = now_ms + self.interval_ms

        remaining_ms = self._next_tick_ms - self.supervisor._now_ms()
        if remaining_ms > 0:
            self.supervisor.robot.sleep_fn(
                remaining_ms / 1000.0
            )
        elif remaining_ms < -self.interval_ms:
            self._next_tick_ms = self.supervisor._now_ms()
        return self.supervisor.status()

    def run_forever(self, shutdown_requested):
        if not callable(shutdown_requested):
            raise SupervisorError(
                "invalid_shutdown_signal",
                "shutdown_requested must be callable",
            )
        self.supervisor.bind_to_current_thread()
        exit_status = None
        loop_error = None
        close_error = None
        try:
            while not shutdown_requested():
                exit_status = self.run_once()
        except BaseException as error:
            loop_error = error
        try:
            exit_status = self.supervisor.close()
        except BaseException as error:
            close_error = error
            if self.supervisor.state == STATE_CLOSED:
                exit_status = self.supervisor.close()
            else:
                exit_status = self.supervisor.status()
        if exit_status.get("state") != STATE_CLOSED:
            detail = "Supervisor could not verify shutdown stop"
            if loop_error is not None:
                detail += " after {}".format(
                    type(loop_error).__name__
                )
            raise SupervisorError(
                "shutdown_stop_failed",
                detail,
            )
        if exit_status.get("audit_complete") is not True:
            raise SupervisorError(
                "shutdown_audit_failed",
                "Supervisor closed without a complete terminal audit",
            )
        if close_error is not None:
            raise close_error
        if loop_error is not None:
            raise loop_error
        return exit_status
