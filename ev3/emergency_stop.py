#!/usr/bin/env python3
"""Conservative, Python 3.5-compatible emergency stop verification."""

from __future__ import print_function

import glob
import os

if __package__:
    from .robot_config import (
        MIN_STOP_STABLE_INTERVALS,
        MIN_STOP_STABLE_WINDOW_MS,
    )
else:
    from robot_config import (
        MIN_STOP_STABLE_INTERVALS,
        MIN_STOP_STABLE_WINDOW_MS,
    )


ACTIVE_MOTOR_STATES = frozenset(("running", "ramping", "holding"))
FAULT_MOTOR_STATES = frozenset(("stalled", "overloaded"))
KNOWN_MOTOR_STATES = ACTIVE_MOTOR_STATES | FAULT_MOTOR_STATES


def _valid_positive_int(value):
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and value > 0
    )


class _EmergencyStopAttempt(object):
    def __init__(
        self,
        sysfs_root,
        sleep_fn,
        read_fn,
        write_fn,
        expected_motors,
        timeout_ms,
        poll_ms,
        configuration_error,
        glob_fn,
    ):
        self.sysfs_root = sysfs_root
        self.sleep_fn = sleep_fn
        self.read_fn = read_fn
        self.write_fn = write_fn
        self.expected_motors = expected_motors
        self.timeout_ms = timeout_ms
        self.poll_ms = poll_ms
        self.configuration_error = configuration_error
        self.glob_fn = glob_fn
        self.configuration_valid = expected_motors is not None
        self.operational_errors = []
        self.topology_errors = []
        self.attempts = []
        self.deferred_interrupt = None
        self.paths = []
        self.records = {}
        self.discovered_ports = {}
        self.inactive_and_stable = False
        self.final_fault_tokens = {}
        self.maximum_attempts = 2
        self.required_stable_intervals = MIN_STOP_STABLE_INTERVALS
        self.stable_intervals = 0

    @staticmethod
    def _append_once(items, value):
        if value not in items:
            items.append(value)

    def _remember_interrupt(self, error):
        if (
            not isinstance(error, Exception)
            and self.deferred_interrupt is None
        ):
            self.deferred_interrupt = error

    def _operational_error(self, detail, error=None):
        self._append_once(self.operational_errors, detail)
        if error is not None:
            self._remember_interrupt(error)

    def _topology_error(self, detail, error=None):
        self._append_once(self.topology_errors, detail)
        if error is not None:
            self._remember_interrupt(error)

    def _validate_poll_settings(self):
        if not _valid_positive_int(self.timeout_ms):
            self._operational_error(
                "stop verification timeout must be a positive integer"
            )
        if not _valid_positive_int(self.poll_ms):
            self._operational_error(
                "stop verification poll interval must be a positive integer"
            )
        if (
            _valid_positive_int(self.timeout_ms)
            and _valid_positive_int(self.poll_ms)
        ):
            if self.poll_ms > self.timeout_ms:
                self._operational_error(
                    "stop verification poll interval exceeds timeout"
                )
            else:
                self.maximum_attempts = max(
                    2, (self.timeout_ms // self.poll_ms) + 1
                )
                self.required_stable_intervals = max(
                    MIN_STOP_STABLE_INTERVALS,
                    (
                        MIN_STOP_STABLE_WINDOW_MS
                        + self.poll_ms
                        - 1
                    )
                    // self.poll_ms,
                )
        if self.configuration_error is not None:
            self._topology_error(
                "configuration: {}".format(self.configuration_error)
            )
        elif not self.configuration_valid:
            self._topology_error(
                "Expected motor topology is unavailable"
            )

    def _discover(self):
        pattern = os.path.join(
            self.sysfs_root, "tacho-motor", "*"
        )
        try:
            self.paths = sorted(set(self.glob_fn(pattern)))
        except BaseException as error:
            self.paths = []
            self._operational_error(
                "discover: {}".format(error), error
            )

    def _record_for_path(self, motor_path):
        record = self.records.get(motor_path)
        if record is None:
            record = {
                "path": motor_path,
                "address": None,
                "port": None,
                "driver": None,
                "state": None,
                "position": None,
                "inactive": False,
                "stable": False,
            }
            self.records[motor_path] = record
        return record

    def _read_identity(self, motor_path):
        record = self._record_for_path(motor_path)
        try:
            address = self.read_fn(
                os.path.join(motor_path, "address")
            )
            record["address"] = address
            port = address.rsplit(":", 1)[-1]
            record["port"] = port
            if port in self.discovered_ports:
                self._topology_error(
                    "Duplicate discovered motor port {} at {} and {}".format(
                        port,
                        self.discovered_ports[port],
                        motor_path,
                    )
                )
            self.discovered_ports[port] = motor_path
        except BaseException as error:
            self._topology_error(
                "{} address: {}".format(motor_path, error), error
            )
        try:
            record["driver"] = self.read_fn(
                os.path.join(motor_path, "driver_name")
            )
        except BaseException as error:
            self._topology_error(
                "{} driver: {}".format(motor_path, error), error
            )

    def _inspect_topology(self):
        for motor_path in self.paths:
            self._read_identity(motor_path)
        if self.expected_motors is None:
            return

        for port, expected in sorted(self.expected_motors.items()):
            motor_path = self.discovered_ports.get(port)
            if motor_path is None:
                self._topology_error(
                    "Configured motor {} on {} was not discovered".format(
                        expected["role"], port
                    )
                )
                continue
            actual_driver = self.records[motor_path]["driver"]
            if actual_driver != expected["driver"]:
                self._topology_error(
                    "Motor {} on {} expected driver {} but found {}".format(
                        expected["role"],
                        port,
                        expected["driver"],
                        actual_driver,
                    )
                )
        for port in sorted(
            set(self.discovered_ports) - set(self.expected_motors)
        ):
            self._topology_error(
                "Unconfigured tacho motor was discovered on {}".format(
                    port
                )
            )

    def _write_stop_commands(self, attempt_number):
        for motor_path in self.paths:
            attempt = {
                "attempt": attempt_number,
                "path": motor_path,
                "stop_action_written": False,
                "stop_command_written": False,
            }
            try:
                self.write_fn(
                    os.path.join(motor_path, "stop_action"), "brake"
                )
                attempt["stop_action_written"] = True
            except BaseException as error:
                detail = "{} stop_action: {}".format(
                    motor_path, error
                )
                attempt["stop_action_error"] = str(error)
                self._operational_error(detail, error)
            try:
                self.write_fn(
                    os.path.join(motor_path, "command"), "stop"
                )
                attempt["stop_command_written"] = True
            except BaseException as error:
                detail = "{} stop: {}".format(motor_path, error)
                attempt["stop_command_error"] = str(error)
                self._operational_error(detail, error)
            self.attempts.append(attempt)

    def _snapshot(self):
        positions = {}
        read_succeeded = True
        active_paths = []
        self.final_fault_tokens = {}
        for motor_path in self.paths:
            record = self._record_for_path(motor_path)
            try:
                raw_state = self.read_fn(
                    os.path.join(motor_path, "state")
                )
                tokens = frozenset(raw_state.split())
                unknown = tokens - KNOWN_MOTOR_STATES
                if unknown:
                    raise ValueError(
                        "unknown state token(s) {}".format(
                            sorted(unknown)
                        )
                    )
                position = int(
                    self.read_fn(
                        os.path.join(motor_path, "position")
                    )
                )
                record["state"] = raw_state
                record["position"] = position
                positions[motor_path] = position
                if tokens & ACTIVE_MOTOR_STATES:
                    active_paths.append(motor_path)
                if tokens & FAULT_MOTOR_STATES:
                    self.final_fault_tokens[motor_path] = sorted(
                        tokens & FAULT_MOTOR_STATES
                    )
                record["inactive"] = not bool(
                    tokens
                    & (ACTIVE_MOTOR_STATES | FAULT_MOTOR_STATES)
                )
            except BaseException as error:
                read_succeeded = False
                self._operational_error(
                    "{} verification: {}".format(motor_path, error),
                    error,
                )
        return read_succeeded, positions, active_paths

    def _poll_until_stable(self):
        previous_positions = None
        previous_inactive = False
        for attempt_number in range(1, self.maximum_attempts + 1):
            self._write_stop_commands(attempt_number)
            read_succeeded, positions, active_paths = self._snapshot()
            currently_inactive = (
                read_succeeded
                and not active_paths
                and not self.final_fault_tokens
            )
            unchanged_inactive_interval = (
                currently_inactive
                and previous_inactive
                and previous_positions is not None
                and positions == previous_positions
            )
            if unchanged_inactive_interval:
                self.stable_intervals += 1
            else:
                self.stable_intervals = 0
            stable_positions = (
                self.stable_intervals
                >= self.required_stable_intervals
            )
            for motor_path in self.paths:
                self.records[motor_path]["stable"] = (
                    stable_positions and motor_path in positions
                )
            if currently_inactive and stable_positions:
                self.inactive_and_stable = True
                return
            if read_succeeded:
                previous_positions = dict(positions)
                previous_inactive = currently_inactive
            else:
                previous_inactive = False
            if attempt_number < self.maximum_attempts:
                try:
                    self.sleep_fn(self.poll_ms / 1000.0)
                except BaseException as error:
                    self._operational_error(
                        "stop verification sleep: {}".format(error),
                        error,
                    )
                    return

    def _result(self):
        if not self.paths:
            self._operational_error(
                "No tacho motors were discovered"
            )
        elif not self.inactive_and_stable:
            self._operational_error(
                "Motor stop was not verified before deadline"
            )

        hardware_stop_verified = (
            self.inactive_and_stable
            and not self.operational_errors
        )
        stop_confirmed = (
            self.configuration_valid
            and hardware_stop_verified
            and not self.topology_errors
        )
        return {
            "status": "stopped" if stop_confirmed else "failed",
            "configuration_valid": self.configuration_valid,
            "hardware_stop_verified": hardware_stop_verified,
            "stop_confirmed": stop_confirmed,
            "motor_count": len(self.paths),
            "addresses": [
                self.records[path]["address"]
                for path in self.paths
                if self.records[path]["address"] is not None
            ],
            "motors": [
                self.records[path] for path in self.paths
            ],
            "stop_attempts": self.attempts,
            "minimum_stable_window_ms": MIN_STOP_STABLE_WINDOW_MS,
            "required_stable_intervals": (
                self.required_stable_intervals
            ),
            "observed_stable_intervals": self.stable_intervals,
            "fault_tokens": self.final_fault_tokens,
            "errors": (
                self.topology_errors + self.operational_errors
            ),
        }

    def run(self):
        self._validate_poll_settings()
        self._discover()
        self._poll_until_stable()
        # Identity/topology reads are deliberately after the first stop writes
        # so corrupt or slow descriptive sysfs files cannot delay the stop.
        self._inspect_topology()
        result = self._result()
        if self.deferred_interrupt is not None:
            try:
                self.deferred_interrupt.stop_result = result
            except Exception:
                pass
            raise self.deferred_interrupt
        return result


def verified_emergency_stop(
    sysfs_root,
    sleep_fn,
    read_fn,
    write_fn,
    expected_motors=None,
    timeout_ms=200,
    poll_ms=10,
    configuration_error=None,
    glob_fn=glob.glob,
):
    """Stop every discovered motor and return explicit verification evidence.

    Passing no expected topology deliberately enables a configless emergency
    attempt. Its hardware result remains visible, but ``stop_confirmed`` stays
    false because the expected motor flock could not be established.
    """
    return _EmergencyStopAttempt(
        sysfs_root=sysfs_root,
        sleep_fn=sleep_fn,
        read_fn=read_fn,
        write_fn=write_fn,
        expected_motors=expected_motors,
        timeout_ms=timeout_ms,
        poll_ms=poll_ms,
        configuration_error=configuration_error,
        glob_fn=glob_fn,
    ).run()
