#!/usr/bin/env python3
"""Language-blind, Python 3.5-compatible local EV3 motion supervisor.

The supervisor owns the motor flock for its entire lifetime. It never
interprets natural language and never calls an LLM. Its only job is to admit
and monitor already-typed, locally bounded motor primitives.
"""

from __future__ import print_function

import copy
import hashlib
import os
import threading
import time

if __package__:
    from .infrared_safety import (
        InfraredGatePolicy,
        InfraredObstacleGate,
    )
    from .robot_hal import (
        MotionVerificationError,
        RobotHAL,
        SafetyError,
        read_text,
        write_text,
    )
    from .supervisor_support import (
        AuditBuffer,
        JSONLAuditLog,
        KNOWN_MOTOR_STATES,
        SupervisorError,
        _BoundAttributeReader,
        _combine_stop_results,
        _copy_start_failure_evidence,
        _default_session_id,
        _failed_stop_result,
        _is_int,
        _state_tokens,
        _stop_result_from_exception,
        _validate_identifier,
        _validate_positive_int,
    )
else:
    from infrared_safety import (
        InfraredGatePolicy,
        InfraredObstacleGate,
    )
    from robot_hal import (
        MotionVerificationError,
        RobotHAL,
        SafetyError,
        read_text,
        write_text,
    )
    from supervisor_support import (
        AuditBuffer,
        JSONLAuditLog,
        KNOWN_MOTOR_STATES,
        SupervisorError,
        _BoundAttributeReader,
        _combine_stop_results,
        _copy_start_failure_evidence,
        _default_session_id,
        _failed_stop_result,
        _is_int,
        _state_tokens,
        _stop_result_from_exception,
        _validate_identifier,
        _validate_positive_int,
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
AUDIT_SCHEMA = "ev3-supervisor-audit/v1"
IR_ROAMER_RUNTIME_PROFILE = "ir-roamer-v1"
IR_ROAMER_POLL_INTERVAL_MS = 150
IR_ROAMER_MAX_POLL_LATENESS_MS = 400




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
        self._motor_bindings = None
        self._motor_binding_by_role = None
        self._touch_binding = None
        self._infrared_binding = None
        self._bound_readers = ()

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

    @staticmethod
    def _attribute_text(path, attribute, kind, error_code):
        try:
            return read_text(os.path.join(path, attribute))
        except (IOError, OSError, ValueError) as error:
            raise SupervisorError(
                error_code,
                "{} {} could not be read: {}".format(
                    kind,
                    attribute,
                    error,
                ),
            )

    @staticmethod
    def _identity_token(path, kind, error_code):
        try:
            metadata = os.stat(path)
        except (IOError, OSError, ValueError) as error:
            raise SupervisorError(
                error_code,
                "{} identity could not be read: {}".format(
                    kind,
                    error,
                ),
            )
        return (metadata.st_dev, metadata.st_ino)

    @staticmethod
    def _close_readers(readers):
        pending_error = None
        for reader in reversed(tuple(readers)):
            try:
                reader.close()
            except BaseException as error:
                if pending_error is None:
                    pending_error = error
        if pending_error is not None:
            raise pending_error

    @staticmethod
    def _open_bound_reader(path, attribute, kind):
        reader = _BoundAttributeReader(
            os.path.join(path, attribute),
            "{} {}".format(kind, attribute),
        )
        try:
            reader.read("hardware_topology_unreadable")
        except BaseException:
            try:
                reader.close()
            except BaseException:
                pass
            raise
        return reader

    def _revalidate_bindings(
        self,
        motor_bindings,
        touch,
        infrared,
    ):
        discovered_paths = self._discovered_motor_paths()
        expected_paths = set(
            binding["path"] for binding in motor_bindings
        )
        if set(discovered_paths) != expected_paths:
            raise SupervisorError(
                "hardware_topology_changed",
                "Discovered motor set changed after startup",
            )

        for binding in motor_bindings:
            role = binding["role"]
            try:
                resolved = self.robot._motor_path_for_role(role)
            except (
                IOError,
                OSError,
                RuntimeError,
                ValueError,
            ) as error:
                raise SupervisorError(
                    "hardware_topology_changed",
                    "Motor {} could not be revalidated: {}".format(
                        role,
                        error,
                    ),
                )
            address = self._attribute_text(
                binding["path"],
                "address",
                "Motor {}".format(role),
                "hardware_topology_unreadable",
            )
            driver = self._attribute_text(
                binding["path"],
                "driver_name",
                "Motor {}".format(role),
                "hardware_topology_unreadable",
            )
            identity_token = self._identity_token(
                binding["path"],
                "Motor {}".format(role),
                "hardware_topology_unreadable",
            )
            for reader in binding["readers"].values():
                reader._verify_identity("hardware_topology_changed")
            if (
                resolved != binding["path"]
                or address != binding["address"]
                or driver != binding["driver"]
                or identity_token != binding["identity_token"]
            ):
                raise SupervisorError(
                    "hardware_topology_changed",
                    "Motor {} identity changed after startup".format(role),
                )

        try:
            touch_path = self.robot._sensor_path_for_role("touch")
        except (
            IOError,
            OSError,
            RuntimeError,
            ValueError,
        ) as error:
            raise SupervisorError(
                "hardware_topology_changed",
                "Touch sensor could not be revalidated: {}".format(
                    error
                ),
            )
        address = self._attribute_text(
            touch["path"],
            "address",
            "Touch sensor",
            "hardware_topology_unreadable",
        )
        driver = self._attribute_text(
            touch["path"],
            "driver_name",
            "Touch sensor",
            "hardware_topology_unreadable",
        )
        mode = self._attribute_text(
            touch["path"],
            "mode",
            "Touch sensor",
            "hardware_topology_unreadable",
        )
        identity_token = self._identity_token(
            touch["path"],
            "Touch sensor",
            "hardware_topology_unreadable",
        )
        for reader in touch["readers"].values():
            reader._verify_identity("hardware_topology_changed")
        cached_mode = touch["readers"]["mode"].read(
            "hardware_topology_changed"
        )
        if (
            touch_path != touch["path"]
            or address != touch["address"]
            or driver != touch["driver"]
            or mode != touch["mode"]
            or cached_mode != touch["mode"]
            or identity_token != touch["identity_token"]
        ):
            raise SupervisorError(
                "hardware_topology_changed",
                "Touch sensor identity changed after startup",
            )

        try:
            infrared_path = self.robot._sensor_path_for_role("infrared")
        except (
            IOError,
            OSError,
            RuntimeError,
            ValueError,
        ) as error:
            raise SupervisorError(
                "hardware_topology_changed",
                "Infrared sensor could not be revalidated: {}".format(
                    error
                ),
            )
        address = self._attribute_text(
            infrared["path"],
            "address",
            "Infrared sensor",
            "hardware_topology_unreadable",
        )
        driver = self._attribute_text(
            infrared["path"],
            "driver_name",
            "Infrared sensor",
            "hardware_topology_unreadable",
        )
        mode = self._attribute_text(
            infrared["path"],
            "mode",
            "Infrared sensor",
            "hardware_topology_unreadable",
        )
        identity_token = self._identity_token(
            infrared["path"],
            "Infrared sensor",
            "hardware_topology_unreadable",
        )
        for reader in infrared["readers"].values():
            reader._verify_identity("hardware_topology_changed")
        cached_mode = infrared["readers"]["mode"].read(
            "hardware_topology_changed"
        )
        if (
            infrared_path != infrared["path"]
            or address != infrared["address"]
            or driver != infrared["driver"]
            or mode != infrared["mode"]
            or cached_mode != infrared["mode"]
            or identity_token != infrared["identity_token"]
        ):
            raise SupervisorError(
                "hardware_topology_changed",
                "Infrared sensor identity changed after startup",
            )

    def bind_topology(self):
        """Bind immutable device identity once while the motor lock is held."""

        self._require_open()
        if (
            self._motor_bindings is not None
            or self._motor_binding_by_role is not None
            or self._touch_binding is not None
            or self._infrared_binding is not None
            or self._bound_readers
        ):
            raise SupervisorError(
                "hardware_topology_already_bound",
                "Hardware topology is already bound",
            )
        bindings = []
        seen_paths = set()
        provisional_readers = []
        try:
            for role in sorted(self.robot.config["motors"]):
                configured = self.robot.config["motors"][role]
                path = self.robot._motor_path_for_role(role)
                if path in seen_paths:
                    raise SupervisorError(
                        "hardware_topology_changed",
                        "Two configured roles resolve to one motor",
                    )
                seen_paths.add(path)
                kind = "Motor {}".format(role)
                bindings.append(
                    {
                        "role": role,
                        "path": path,
                        "address": self._attribute_text(
                            path,
                            "address",
                            kind,
                            "hardware_topology_unreadable",
                        ),
                        "driver": self._attribute_text(
                            path,
                            "driver_name",
                            kind,
                            "hardware_topology_unreadable",
                        ),
                        "identity_token": self._identity_token(
                            path,
                            kind,
                            "hardware_topology_unreadable",
                        ),
                        "readers": {},
                    }
                )
            discovered_paths = self._discovered_motor_paths()
            if seen_paths != set(discovered_paths):
                raise SupervisorError(
                    "hardware_topology_changed",
                    "Discovered motors do not exactly match configuration",
                )

            touch_config = self.robot.config["sensors"]["touch"]
            touch_path = self.robot._sensor_path_for_role("touch")
            touch_address = self._attribute_text(
                touch_path,
                "address",
                "Touch sensor",
                "hardware_topology_unreadable",
            )
            touch_driver = self._attribute_text(
                touch_path,
                "driver_name",
                "Touch sensor",
                "hardware_topology_unreadable",
            )
            touch_mode = self._attribute_text(
                touch_path,
                "mode",
                "Touch sensor",
                "hardware_topology_unreadable",
            )
            if (
                touch_driver != touch_config["driver"]
                or touch_mode != touch_config["mode"]
            ):
                raise SupervisorError(
                    "hardware_topology_changed",
                    "Touch sensor identity does not match configuration",
                )
            touch = {
                "path": touch_path,
                "address": touch_address,
                "driver": touch_driver,
                "mode": touch_mode,
                "identity_token": self._identity_token(
                    touch_path,
                    "Touch sensor",
                    "hardware_topology_unreadable",
                ),
                "readers": {},
            }

            infrared_config = self.robot.config["sensors"]["infrared"]
            infrared_path = self.robot._sensor_path_for_role("infrared")
            infrared_address = self._attribute_text(
                infrared_path,
                "address",
                "Infrared sensor",
                "hardware_topology_unreadable",
            )
            infrared_driver = self._attribute_text(
                infrared_path,
                "driver_name",
                "Infrared sensor",
                "hardware_topology_unreadable",
            )
            infrared_mode = self._attribute_text(
                infrared_path,
                "mode",
                "Infrared sensor",
                "hardware_topology_unreadable",
            )
            if (
                infrared_config.get("port") != "in4"
                or infrared_driver != infrared_config["driver"]
                or infrared_mode != infrared_config["mode"]
                or infrared_mode != "IR-PROX"
            ):
                raise SupervisorError(
                    "hardware_topology_changed",
                    "Infrared sensor identity does not match "
                    "the fixed in4 IR-PROX configuration",
                )
            infrared = {
                "path": infrared_path,
                "address": infrared_address,
                "driver": infrared_driver,
                "mode": infrared_mode,
                "identity_token": self._identity_token(
                    infrared_path,
                    "Infrared sensor",
                    "hardware_topology_unreadable",
                ),
                "readers": {},
            }

            for binding in bindings:
                kind = "Motor {}".format(binding["role"])
                for attribute in ("state", "position"):
                    reader = self._open_bound_reader(
                        binding["path"],
                        attribute,
                        kind,
                    )
                    provisional_readers.append(reader)
                    binding["readers"][attribute] = reader
                _state_tokens(
                    binding["readers"]["state"].read(
                        "hardware_topology_unreadable"
                    )
                )
                try:
                    int(
                        binding["readers"]["position"].read(
                            "hardware_topology_unreadable"
                        )
                    )
                except ValueError:
                    raise SupervisorError(
                        "hardware_topology_unreadable",
                        "{} position is not an integer".format(kind),
                    )

            for attribute in ("mode", "value0"):
                reader = self._open_bound_reader(
                    touch_path,
                    attribute,
                    "Touch sensor",
                )
                provisional_readers.append(reader)
                touch["readers"][attribute] = reader
            if (
                touch["readers"]["mode"].read(
                    "hardware_topology_unreadable"
                )
                != touch_mode
            ):
                raise SupervisorError(
                    "hardware_topology_changed",
                    "Touch sensor mode changed while binding",
                )
            try:
                int(
                    touch["readers"]["value0"].read(
                        "hardware_topology_unreadable"
                    )
                )
            except ValueError:
                raise SupervisorError(
                    "hardware_topology_unreadable",
                    "Touch sensor value is not an integer",
                )

            for attribute in ("mode", "value0"):
                reader = self._open_bound_reader(
                    infrared_path,
                    attribute,
                    "Infrared sensor",
                )
                provisional_readers.append(reader)
                infrared["readers"][attribute] = reader
            if (
                infrared["readers"]["mode"].read(
                    "hardware_topology_unreadable"
                )
                != infrared_mode
            ):
                raise SupervisorError(
                    "hardware_topology_changed",
                    "Infrared sensor mode changed while binding",
                )
            try:
                infrared_value = int(
                    infrared["readers"]["value0"].read(
                        "hardware_topology_unreadable"
                    )
                )
            except ValueError:
                raise SupervisorError(
                    "hardware_topology_unreadable",
                    "Infrared sensor value is not an integer",
                )
            if infrared_value < 0 or infrared_value > 100:
                raise SupervisorError(
                    "hardware_topology_unreadable",
                    "Infrared sensor value is outside 0..100",
                )

            # Close the open-vs-hotplug race before publishing the complete
            # immutable binding set.
            self._revalidate_bindings(
                tuple(bindings),
                touch,
                infrared,
            )
        except BaseException as error:
            try:
                self._close_readers(provisional_readers)
            except BaseException:
                pass
            if isinstance(error, SupervisorError):
                raise
            if isinstance(
                error,
                (KeyError, IOError, OSError, RuntimeError, ValueError),
            ):
                raise SupervisorError(
                    "hardware_topology_changed",
                    "Hardware topology could not be bound: {}".format(
                        error
                    ),
                )
            raise

        # Publish only after every descriptor and fresh path has passed.
        self._motor_bindings = tuple(bindings)
        self._motor_binding_by_role = dict(
            (binding["role"], binding)
            for binding in bindings
        )
        self._touch_binding = touch
        self._infrared_binding = infrared
        self._bound_readers = tuple(provisional_readers)

    def _require_bound_topology(self):
        if (
            self._motor_bindings is None
            or self._motor_binding_by_role is None
            or self._touch_binding is None
            or self._infrared_binding is None
        ):
            raise SupervisorError(
                "hardware_topology_unbound",
                "Supervisor hardware topology is not bound",
            )

    def revalidate_topology(self):
        """Re-resolve every immutable identity before arming or motion."""

        self._require_open()
        self._require_bound_topology()
        return self._revalidate_bindings(
            self._motor_bindings,
            self._touch_binding,
            self._infrared_binding,
        )

    def path_for_role(self, role):
        self._require_bound_topology()
        try:
            return self._motor_binding_by_role[role]["path"]
        except KeyError:
            raise SupervisorError(
                "hardware_topology_changed",
                "Motor role {!r} is not bound".format(role),
            )

    def read_touch_value(self):
        self._require_open()
        self._require_bound_topology()
        touch = self._touch_binding
        identity_token = self._identity_token(
            touch["path"],
            "Touch sensor",
            "hardware_topology_changed",
        )
        mode = touch["readers"]["mode"].read(
            "hardware_read_failed"
        )
        if (
            identity_token != touch["identity_token"]
            or mode != touch["mode"]
        ):
            raise SupervisorError(
                "hardware_topology_changed",
                "Touch sensor identity changed during polling",
            )
        raw = touch["readers"]["value0"].read(
            "hardware_read_failed"
        )
        try:
            return int(raw)
        except ValueError:
            raise SupervisorError(
                "hardware_read_failed",
                "Touch sensor value is not an integer",
            )

    def read_infrared_value(self):
        self._require_open()
        self._require_bound_topology()
        infrared = self._infrared_binding
        identity_token = self._identity_token(
            infrared["path"],
            "Infrared sensor",
            "hardware_topology_changed",
        )
        mode = infrared["readers"]["mode"].read(
            "hardware_read_failed"
        )
        if (
            identity_token != infrared["identity_token"]
            or mode != infrared["mode"]
            or mode != "IR-PROX"
        ):
            raise SupervisorError(
                "hardware_topology_changed",
                "Infrared sensor identity changed during polling",
            )
        raw = infrared["readers"]["value0"].read(
            "hardware_read_failed"
        )
        try:
            value = int(raw)
        except ValueError:
            raise SupervisorError(
                "hardware_read_failed",
                "Infrared sensor value is not an integer",
            )
        if value < 0 or value > 100:
            raise SupervisorError(
                "hardware_read_failed",
                "Infrared sensor value is outside 0..100",
            )
        return value

    def dynamic_motor_snapshots(self, position_roles=()):
        """Read one consolidated dynamic snapshot from bound devices."""

        self._require_open()
        self._require_bound_topology()
        requested = set(position_roles)
        if not requested <= set(self._motor_binding_by_role):
            raise SupervisorError(
                "hardware_topology_changed",
                "Dynamic snapshot requested an unbound motor role",
            )

        # Retain hot-plug detection without resolving ports, drivers,
        # addresses or modes again on every safety tick.
        discovered_paths = set(self._discovered_motor_paths())
        expected_paths = set(
            binding["path"] for binding in self._motor_bindings
        )
        if discovered_paths != expected_paths:
            raise SupervisorError(
                "hardware_topology_changed",
                "Discovered motor set changed during polling",
            )

        snapshots = []
        for binding in self._motor_bindings:
            role = binding["role"]
            identity_token = self._identity_token(
                binding["path"],
                "Motor {}".format(role),
                "hardware_topology_changed",
            )
            if identity_token != binding["identity_token"]:
                raise SupervisorError(
                    "hardware_topology_changed",
                    "Motor {} identity changed during polling".format(
                        role
                    ),
                )
            snapshot = {
                "role": role,
                "path": binding["path"],
                "address": binding["address"],
                "state": binding["readers"]["state"].read(
                    "hardware_read_failed"
                ),
            }
            if role in requested:
                raw_position = binding["readers"]["position"].read(
                    "hardware_read_failed"
                )
                try:
                    snapshot["position"] = int(raw_position)
                except ValueError:
                    raise SupervisorError(
                        "hardware_read_failed",
                        "Motor {} position is not an integer".format(role),
                    )
            snapshots.append(snapshot)
        return snapshots

    def snapshot_all(self):
        """Compatibility snapshot with dynamic state and all positions."""

        self._require_bound_topology()
        return self.dynamic_motor_snapshots(
            tuple(self._motor_binding_by_role)
        )

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

    def _start_bound_motors(
        self,
        motors,
        duration_ms,
        pre_each_start,
        forward_motion,
    ):
        start_write_windows = []
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

            for motor in motors:
                if pre_each_start is not None:
                    pre_each_start(
                        copy.deepcopy(motor),
                        tuple(copy.deepcopy(start_write_windows)),
                    )
                write_begin_ms = int(
                    self.robot.monotonic_fn() * 1000
                )
                if (
                    start_write_windows
                    and write_begin_ms
                    - start_write_windows[0]["begin_ms"]
                    > self.limits["max_start_skew_ms"]
                ):
                    raise SupervisorError(
                        "start_skew",
                        "Second motor start missed the local skew limit",
                    )
                motor["start_write_begin_ms"] = write_begin_ms
                write_text(
                    os.path.join(motor["path"], "command"),
                    "run-timed",
                )
                write_end_ms = int(
                    self.robot.monotonic_fn() * 1000
                )
                # A successful write means this motor may already have
                # moved.  Record that fact before any later validation can
                # raise so cleanup evidence cannot accidentally erase real
                # motion from host odometry.
                start_write_windows.append(
                    {
                        "side": motor["side"],
                        "role": motor["role"],
                        "begin_ms": write_begin_ms,
                        "end_ms": write_end_ms,
                    }
                )
                if write_end_ms < write_begin_ms:
                    raise SupervisorError(
                        "clock_rollback",
                        "Monotonic clock moved backwards during motor start",
                    )
                motor["start_write_end_ms"] = write_end_ms
                # Preserve the original field as the conservative earliest
                # instant at which the kernel could have observed the write.
                motor["start_write_ms"] = write_begin_ms
                if (
                    len(start_write_windows) == 2
                    and write_end_ms
                    - start_write_windows[0]["begin_ms"]
                    > self.limits["max_start_skew_ms"]
                ):
                    raise SupervisorError(
                        "start_skew",
                        "Motor command-write window exceeded local limit",
                    )
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
            try:
                completed_at_ms = int(
                    self.robot.monotonic_fn() * 1000
                )
            except BaseException:
                completed_at_ms = None
            motor_receipts = []
            cleanup_positions = (
                cleanup.get("positions", {})
                if isinstance(cleanup, dict)
                else {}
            )
            cleanup_states = (
                cleanup.get("states", {})
                if isinstance(cleanup, dict)
                else {}
            )
            for motor in motors:
                before = motor.get("position_before")
                after = cleanup_positions.get(motor["path"])
                state = cleanup_states.get(motor["path"])
                motor_receipts.append(
                    {
                        "side": motor["side"],
                        "role": motor["role"],
                        "physical_speed_dps": motor[
                            "physical_speed_dps"
                        ],
                        "position_before": before,
                        "position_after": after,
                        "position_delta": (
                            after - before
                            if isinstance(before, int)
                            and not isinstance(before, bool)
                            and isinstance(after, int)
                            and not isinstance(after, bool)
                            else None
                        ),
                        "state": state,
                    }
                )
            evidence_complete = (
                isinstance(cleanup, dict)
                and cleanup.get("stop_confirmed") is True
                and not cleanup.get("errors")
                and not cleanup.get("fault_tokens")
                and isinstance(completed_at_ms, int)
                and not isinstance(completed_at_ms, bool)
                and all(
                    isinstance(receipt["position_before"], int)
                    and not isinstance(
                        receipt["position_before"], bool
                    )
                    and isinstance(receipt["position_after"], int)
                    and not isinstance(
                        receipt["position_after"], bool
                    )
                    and isinstance(receipt["position_delta"], int)
                    and not isinstance(
                        receipt["position_delta"], bool
                    )
                    and isinstance(receipt["state"], str)
                    for receipt in motor_receipts
                )
                and all(
                    isinstance(window.get("begin_ms"), int)
                    and not isinstance(window.get("begin_ms"), bool)
                    and isinstance(window.get("end_ms"), int)
                    and not isinstance(window.get("end_ms"), bool)
                    and window["end_ms"] >= window["begin_ms"]
                    for window in start_write_windows
                )
                and (
                    not start_write_windows
                    or completed_at_ms
                    >= start_write_windows[0]["begin_ms"]
                )
            )
            start_evidence = {
                "complete": evidence_complete,
                "duration_ms": duration_ms,
                "started_at_ms": (
                    start_write_windows[0]["begin_ms"]
                    if start_write_windows
                    else None
                ),
                "completed_at_ms": completed_at_ms,
                "started_sides": [
                    window["side"] for window in start_write_windows
                ],
                "start_write_windows": copy.deepcopy(
                    start_write_windows
                ),
                "motors": motor_receipts,
                "stop": copy.deepcopy(cleanup),
            }
            try:
                propagated.supervisor_start_evidence = copy.deepcopy(
                    start_evidence
                )
            except Exception:
                pass
            raise propagated

        started_at_ms = start_write_windows[0]["begin_ms"]
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
            "start_write_window_ms": (
                start_write_windows[-1]["end_ms"]
                - start_write_windows[0]["begin_ms"]
            ),
            "forward_motion": (
                forward_motion
            ),
        }
        return self.active_snapshot()

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
        left_path = self.path_for_role(left_role)
        right_path = self.path_for_role(right_role)
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
        return self._start_bound_motors(
            motors,
            duration_ms,
            pre_each_start,
            left_speed_dps > 0 and right_speed_dps > 0,
        )

    def start_drive_side(
        self,
        side,
        logical_speed_dps,
        duration_ms,
        pre_each_start=None,
    ):
        """Start one internal drive-wheel correction under the same lock."""
        self._require_open()
        if self.active is not None:
            raise SupervisorError(
                "motion_already_active",
                "A motion command is already active",
            )
        if side not in ("left", "right"):
            raise SupervisorError(
                "invalid_drive_side",
                "Drive side must be left or right",
            )
        if (
            pre_each_start is not None
            and not callable(pre_each_start)
        ):
            raise SupervisorError(
                "invalid_start_guard",
                "pre_each_start must be callable",
            )
        left_role, right_role, forward_signs = self.robot._drive_roles()
        role = left_role if side == "left" else right_role
        self.robot._validate_motion(
            role,
            logical_speed_dps,
            duration_ms,
        )
        motor = {
            "side": side,
            "role": role,
            "path": self.path_for_role(role),
            "logical_speed_dps": logical_speed_dps,
            "physical_speed_dps": (
                logical_speed_dps * forward_signs[role]
            ),
        }
        return self._start_bound_motors(
            [motor],
            duration_ms,
            pre_each_start,
            logical_speed_dps > 0,
        )

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

    def active_snapshot(self, observations=None):
        self._require_open()
        if self.active is None:
            raise SupervisorError(
                "no_active_motion",
                "No motion command is active",
            )
        active_roles = tuple(
            configured["role"]
            for configured in self.active["motors"]
        )
        if observations is None:
            observations = self.dynamic_motor_snapshots(active_roles)
        observed_by_path = dict(
            (observed["path"], observed)
            for observed in observations
        )
        motors = []
        for configured in self.active["motors"]:
            try:
                observed = observed_by_path[configured["path"]]
                position = observed["position"]
            except KeyError:
                raise SupervisorError(
                    "hardware_read_failed",
                    "Active motor is absent from the dynamic snapshot",
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
                    "position": position,
                    "state": observed["state"],
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
        cleanup_errors = []
        try:
            self._close_readers(self._bound_readers)
        except BaseException as error:
            cleanup_errors.append(str(error))
        self._bound_readers = ()
        if cleanup_errors:
            result["cleanup_errors"] = cleanup_errors
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
        runtime_profile=None,
    ):
        if not callable(session_id_factory):
            raise SupervisorError(
                "invalid_session_factory",
                "Session factory must be callable",
            )
        self.robot = robot
        self._session_id_factory = session_id_factory
        self.limits = self._validated_limits(robot.config)
        if runtime_profile not in (None, IR_ROAMER_RUNTIME_PROFILE):
            raise SupervisorError(
                "invalid_runtime_profile",
                "Supervisor runtime profile is not supported",
            )
        self.runtime_profile = runtime_profile
        if runtime_profile == IR_ROAMER_RUNTIME_PROFILE:
            self.limits["poll_interval_ms"] = (
                IR_ROAMER_POLL_INTERVAL_MS
            )
            self.limits["max_poll_lateness_ms"] = (
                IR_ROAMER_MAX_POLL_LATENESS_MS
            )
            if (
                self.limits["poll_interval_ms"]
                >= self.limits["heartbeat_timeout_ms"]
                or self.limits["max_poll_lateness_ms"]
                >= self.limits["heartbeat_timeout_ms"]
            ):
                raise SupervisorError(
                    "invalid_supervisor_config",
                    "IR roamer timing must remain inside heartbeat timeout",
                )
        self._infrared_gate = InfraredObstacleGate(
            InfraredGatePolicy.from_config(robot.config)
        )
        self._infrared_observed_ms = None
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
            self._owner.bind_topology()
            startup_snapshots = (
                self._owner.dynamic_motor_snapshots()
            )
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
        supervisor["drive_max_speed_dps"] = drive_max_speed_dps
        supervisor["drive_max_duration_ms"] = (
            drive_max_duration_ms
        )
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
        value = self._owner.read_touch_value()
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
        return {
            "observed_monotonic_ms": self._now_ms(),
            "value0": value,
        }

    def _read_infrared(self):
        observed_ms = self._checked_now_ms()
        try:
            value = self._owner.read_infrared_value()
        except BaseException:
            self._infrared_gate.fail_closed()
            self._infrared_observed_ms = observed_ms
            raise
        try:
            snapshot = self._infrared_gate.observe(value)
        except SafetyError as error:
            self._infrared_observed_ms = observed_ms
            raise SupervisorError(
                "infrared_invalid_sample",
                str(error),
            )
        self._infrared_observed_ms = observed_ms
        return snapshot

    def _infrared_status(self, now_ms):
        snapshot = self._infrared_gate.snapshot()
        observed_ms = self._infrared_observed_ms
        age_ms = (
            None
            if observed_ms is None
            else now_ms - observed_ms
        )
        freshness_limit_ms = (
            self.limits["poll_interval_ms"]
            + self.limits["max_poll_lateness_ms"]
        )
        fresh = (
            age_ms is not None
            and age_ms >= 0
            and age_ms <= freshness_limit_ms
            and snapshot["raw"] is not None
        )
        snapshot.update(
            {
                "observed_monotonic_ms": observed_ms,
                "age_ms": age_ms,
                "fresh": fresh,
            }
        )
        return snapshot

    def _require_forward_infrared_clear(self):
        now_ms = self._checked_now_ms()
        infrared = self._infrared_status(now_ms)
        if not infrared["fresh"]:
            raise SupervisorError(
                "infrared_not_fresh",
                "Forward motion requires a fresh local IR observation",
            )
        if infrared["blocked"]:
            raise SupervisorError(
                "infrared_obstacle",
                "Forward motion is blocked by the local IR gate",
            )
        return infrared

    def _stop_forward_for_infrared(self):
        command_id = self.active_command_id
        infrared = self._infrared_status(self._checked_now_ms())
        result, stop_error = self._finish_active_with_retry(
            "infrared_obstacle_stop"
        )
        self.active_command_id = None
        if (
            stop_error is None
            and result["stop_confirmed"]
            and not result["errors"]
            and not result.get("fault_tokens")
        ):
            self.state = STATE_ARMED_IDLE
            self._audit(
                "infrared_obstacle_stop",
                command_id=command_id,
                infrared=infrared,
                stop=result,
            )
            return self.status()
        self._enter_fault(
            "infrared_stop_failed",
            "IR obstacle stop was not verified",
            stop_result=result,
        )
        if (
            stop_error is not None
            and not isinstance(stop_error, Exception)
        ):
            raise stop_error
        return self.status()

    def _check_idle_motors(self):
        snapshots = self._owner.dynamic_motor_snapshots()
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

    def _check_running_motion(self, touch_already_read=False):
        if not touch_already_read:
            self._read_touch()
        active_roles = tuple(
            motor["role"]
            for motor in self._owner.active["motors"]
        )
        all_motors = self._owner.dynamic_motor_snapshots(
            active_roles
        )
        snapshot = self._owner.active_snapshot(all_motors)
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
                self._read_infrared()
                self._checked_now_ms()
            except Exception as error:
                secondary_errors.append(str(error))
            if secondary_errors:
                self.fault["secondary_errors"] = secondary_errors
            return self.status()

        try:
            now_ms = self._checked_now_ms()
            self._read_touch()
            self._read_infrared()
            now_ms = self._checked_now_ms()
            if self.state == STATE_RUNNING:
                if (
                    self._owner.active is not None
                    and self._owner.active.get("forward_motion") is True
                    and self._infrared_gate.blocked
                ):
                    return self._stop_forward_for_infrared()
                running = self._check_running_motion(
                    touch_already_read=True
                )
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

    def _revalidate_hardware_topology(self, phase):
        try:
            return self._owner.revalidate_topology()
        except (
            IOError,
            OSError,
            RuntimeError,
            ValueError,
            SupervisorError,
        ) as error:
            code = getattr(
                error,
                "code",
                "hardware_topology_changed",
            )
            detail = "{} topology revalidation failed: {}".format(
                phase,
                error,
            )
            self._enter_fault(code, detail)
            raise SupervisorError(code, detail)

    def arm(self, session_id, sequence_id):
        self._require_dispatch_thread()
        self._require_state(STATE_DISARMED)
        self._require_session(session_id)
        self._validate_sequence(sequence_id)
        self._revalidate_hardware_topology("arm")
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
        prior_start_write_windows,
        require_infrared_clear=False,
    ):
        if not prior_start_write_windows:
            # Motor parameters have already been staged, but neither
            # run-timed command has been written.  Take the final full local
            # safety snapshot at this last atomic boundary.  The second
            # guard below deliberately uses only these cached results so the
            # two command writes are not separated by slow sysfs reads.
            self._read_touch()
            self._read_infrared()
            self._check_idle_motors()
        now_ms = self._checked_now_ms()
        if self.touch_value != 0:
            raise SupervisorError(
                "touch_pressed",
                "Cached touch input is not released before motor start",
            )
        if not self._heartbeat_is_fresh(now_ms):
            raise SupervisorError(
                "heartbeat_timeout",
                "Heartbeat expired between motor starts",
            )
        if require_infrared_clear:
            self._require_forward_infrared_clear()
        if (
            prior_start_write_windows
            and now_ms
            - prior_start_write_windows[0]["begin_ms"]
            > self.limits["max_start_skew_ms"]
        ):
            raise SupervisorError(
                "start_skew",
                "Second motor start missed the local skew limit",
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
        external_start_guard=None,
    ):
        self._require_dispatch_thread()
        self._require_state(STATE_ARMED_IDLE)
        self._require_session(session_id)
        self._validate_sequence(sequence_id)
        if (
            external_start_guard is not None
            and not callable(external_start_guard)
        ):
            raise SupervisorError(
                "invalid_start_guard",
                "external_start_guard must be callable",
            )
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

        forward_motion = (
            left_speed_dps > 0 and right_speed_dps > 0
        )
        self._revalidate_hardware_topology("drive")
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
        if forward_motion:
            self._require_forward_infrared_clear()
        if external_start_guard is not None:
            external_start_guard()

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
            self._read_infrared()
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
            if forward_motion:
                self._require_forward_infrared_clear()
            if external_start_guard is not None:
                external_start_guard()
        except BaseException as error:
            code = getattr(error, "code", "prestart_failure")
            if code in (
                "infrared_not_fresh",
                "infrared_obstacle",
            ):
                self._audit(
                    "infrared_forward_denied",
                    command_id=command_id,
                    infrared=self._infrared_status(
                        self._checked_now_ms()
                    ),
                )
                if not isinstance(error, Exception):
                    raise
                raise _copy_start_failure_evidence(
                    error,
                    SupervisorError(code, str(error)),
                )
            self._enter_fault(code, str(error))
            if not isinstance(error, Exception):
                raise
            raise SupervisorError(
                code,
                "Motion prestart check failed",
            )
        try:
            def guarded_individual_start(
                motor,
                prior_start_write_windows,
            ):
                self._guard_individual_motor_start(
                    motor,
                    prior_start_write_windows,
                    require_infrared_clear=forward_motion,
                )
                if external_start_guard is not None:
                    external_start_guard()

            motion = self._owner.start_drive(
                left_speed_dps,
                right_speed_dps,
                duration_ms,
                pre_each_start=guarded_individual_start,
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
            if (
                code in (
                    "infrared_not_fresh",
                    "infrared_obstacle",
                )
                and stop_result.get("stop_confirmed") is True
                and not stop_result.get("errors")
                and not stop_result.get("fault_tokens")
            ):
                self.active_command_id = None
                self.state = STATE_ARMED_IDLE
                self._audit(
                    "infrared_obstacle_stop",
                    command_id=command_id,
                    infrared=self._infrared_status(
                        self._checked_now_ms()
                    ),
                    stop=stop_result,
                )
                if not isinstance(error, Exception):
                    raise
                raise _copy_start_failure_evidence(
                    error,
                    SupervisorError(code, str(error)),
                )
            self._enter_fault(
                code,
                str(error),
                stop_result=stop_result,
            )
            if not isinstance(error, Exception):
                raise
            raise _copy_start_failure_evidence(
                error,
                SupervisorError(
                    code,
                    "Motion start failed",
                ),
            )

        self.active_command_id = command_id
        self.state = STATE_RUNNING
        start_skew_ms = self._owner.active[
            "start_write_window_ms"
        ]
        try:
            self._read_infrared()
            if forward_motion and self._infrared_gate.blocked:
                self._stop_forward_for_infrared()
                raise SupervisorError(
                    "infrared_obstacle",
                    "Forward motion was stopped by the local IR gate",
                )
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
            if external_start_guard is not None:
                external_start_guard()
        except BaseException as error:
            code = getattr(error, "code", "poststart_failure")
            if (
                code in (
                    "infrared_not_fresh",
                    "infrared_obstacle",
                )
                and self.state == STATE_ARMED_IDLE
            ):
                if not isinstance(error, Exception):
                    raise
                raise SupervisorError(code, str(error))
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
        if (
            isinstance(self.fault, dict)
            and self.fault.get("code")
            in (
                "hardware_read_failed",
                "hardware_topology_changed",
                "hardware_topology_unreadable",
                "infrared_invalid_sample",
            )
        ):
            raise SupervisorError(
                "supervisor_restart_required",
                "Hardware binding faults require a supervisor restart",
            )
        self.poll_once()
        self._require_state(STATE_FAULT_LATCHED)
        if (
            isinstance(self.fault, dict)
            and self.fault.get("secondary_errors")
        ):
            raise SupervisorError(
                "supervisor_restart_required",
                "Secondary safety-read failures require a restart",
            )
        try:
            self._owner.revalidate_topology()
            self._read_touch()
            self._read_infrared()
        except (
            IOError,
            OSError,
            RuntimeError,
            ValueError,
            SupervisorError,
        ):
            raise SupervisorError(
                "supervisor_restart_required",
                "Fresh hardware binding could not be verified",
            )
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
            "infrared": self._infrared_status(now_ms),
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
        cleanup_errors = list(result.get("cleanup_errors", []))
        self._close_status["cleanup_complete"] = not cleanup_errors
        self._close_status["cleanup_errors"] = cleanup_errors
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
        self._emergency_stop_request_lock = threading.Lock()
        self._request_bound_emergency_stop = False
        self._preverified_stop_status = None

    def request_emergency_stop(self):
        """Thread-safe signal; the dispatch thread performs the stop."""
        with self._emergency_stop_request_lock:
            self._emergency_stop_requested.set()

    def _request_protocol_emergency_stop(self):
        """Signal a correctly targeted, admitted protocol stop request."""
        with self._emergency_stop_request_lock:
            self._request_bound_emergency_stop = True
            self._emergency_stop_requested.set()

    def emergency_stop_requested(self):
        return self._emergency_stop_requested.is_set()

    @staticmethod
    def _status_proves_verified_stop(status):
        if not isinstance(status, dict):
            return False
        state = status.get("state")
        if state == STATE_CLOSED:
            return True
        if (
            status.get("session_active") is not False
            or status.get("active_command_id") is not None
        ):
            return False
        if state == STATE_DISARMED:
            return True
        fault = status.get("fault")
        return (
            state == STATE_FAULT_LATCHED
            and isinstance(fault, dict)
            and fault.get("stop_confirmed") is True
            and not fault.get("stop_errors")
            and not fault.get("fault_tokens")
        )

    def _take_preverified_stop_status(self, supervisor):
        """Consume one loop-local stop proof for this exact supervisor."""

        status = self._preverified_stop_status
        self._preverified_stop_status = None
        if supervisor is not self.supervisor:
            raise SupervisorError(
                "stop_proof_mismatch",
                "Stop proof belongs to another supervisor loop",
            )
        if (
            status is None
            or not self._status_proves_verified_stop(status)
        ):
            return None
        current_status = self.supervisor.status()
        if not self._status_proves_verified_stop(current_status):
            return None
        return current_status

    def _perform_requested_emergency_stop(self):
        request_bound = False
        with self._emergency_stop_request_lock:
            if not self._emergency_stop_requested.is_set():
                return None
            self._emergency_stop_requested.clear()
            request_bound = self._request_bound_emergency_stop
            self._request_bound_emergency_stop = False

        stop_started_ms = self.supervisor._checked_now_ms()
        status = self.supervisor.stop()
        stop_completed_ms = self.supervisor._checked_now_ms()
        if self._status_proves_verified_stop(status):
            if request_bound:
                # A locally verified urgent STOP/SHUTDOWN is mandatory
                # safety work, not scheduler starvation.  Shift only by
                # the measured stop duration so any debt that existed
                # before the request remains fully visible.
                self._next_tick_ms += (
                    stop_completed_ms - stop_started_ms
                )
            self._preverified_stop_status = status
            return self._external_stop_error()
        self._preverified_stop_status = None
        return self._stop_not_confirmed_error()

    @staticmethod
    def _external_stop_error():
        return SupervisorError(
            "external_stop_requested",
            "An emergency stop was requested before response publication",
        )

    @staticmethod
    def _stop_not_confirmed_error():
        return SupervisorError(
            "stop_not_confirmed",
            "An emergency stop could not be locally verified",
        )

    def _dispatch_one(self, dispatch_one):
        if dispatch_one is None:
            self._preverified_stop_status = None
            return (None, None)
        try:
            completion = dispatch_one()
        except BaseException as error:
            self._preverified_stop_status = None
            code = getattr(error, "code", "dispatch_failure")
            self.supervisor._enter_fault(code, str(error))
            raise
        self._preverified_stop_status = None
        post_dispatch_error = None
        stop_outcome = self._perform_requested_emergency_stop()
        if stop_outcome is not None:
            post_dispatch_error = stop_outcome
            self._preverified_stop_status = None
        return (completion, post_dispatch_error)

    def _complete_dispatch(
        self,
        completion,
        post_dispatch_error=None,
    ):
        stop_outcome = self._perform_requested_emergency_stop()
        if stop_outcome is not None:
            post_dispatch_error = self._prefer_dispatch_error(
                stop_outcome,
                post_dispatch_error,
            )
            self._preverified_stop_status = None
        if completion is None:
            return
        if not callable(completion):
            error = SupervisorError(
                "invalid_dispatch_completion",
                "Dispatch completion must be callable",
            )
            self.supervisor._enter_fault(error.code, str(error))
            raise error
        try:
            completion(post_dispatch_error)
        except BaseException as error:
            code = getattr(
                error,
                "code",
                "dispatch_completion_failure",
            )
            self.supervisor._enter_fault(code, str(error))
            raise
        late_stop_outcome = self._perform_requested_emergency_stop()
        self._preverified_stop_status = None
        if (
            late_stop_outcome is not None
            and getattr(
                late_stop_outcome,
                "code",
                None,
            )
            == "stop_not_confirmed"
        ):
            # The response was already linearized by its bounded, nonblocking
            # queue insertion.  A later failed stop is a subsequent safety
            # transition, but it must still terminate the loop loudly.
            raise late_stop_outcome

    @staticmethod
    def _prefer_dispatch_error(first, second):
        if (
            first is not None
            and getattr(first, "code", None)
            == "stop_not_confirmed"
        ):
            return first
        if (
            second is not None
            and getattr(second, "code", None)
            == "stop_not_confirmed"
        ):
            return second
        if first is not None:
            return first
        return second

    def _sleep_after_deadline_fault(self):
        """Keep fault response service bounded without spinning."""
        now_ms = self.supervisor._now_ms()
        remaining_ms = self._next_tick_ms - now_ms
        if remaining_ms <= 0:
            self._next_tick_ms = now_ms + self.interval_ms
            remaining_ms = self.interval_ms
        self.supervisor.robot.sleep_fn(remaining_ms / 1000.0)

    def run_once(self, dispatch_one=None):
        if dispatch_one is not None and not callable(dispatch_one):
            raise SupervisorError(
                "invalid_dispatch_hook",
                "dispatch_one must be callable",
            )
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
            # The fault is latched and motion/session state has been
            # invalidated before any queued request is serviced.  Dispatch
            # exactly one item so stop/shutdown and diagnostic/error
            # responses cannot starve under repeated late wakeups.  Normal
            # motion, claim, and arm requests remain fail-closed behind the
            # supervisor's FAULT_LATCHED state guards.
            self._perform_requested_emergency_stop()
            completion, dispatch_error = self._dispatch_one(
                dispatch_one
            )
            self._complete_dispatch(
                completion,
                post_dispatch_error=dispatch_error,
            )
            self._sleep_after_deadline_fault()
            return self.supervisor.status()

        self._perform_requested_emergency_stop()

        self.supervisor.poll_once()
        self._next_tick_ms += self.interval_ms
        self._perform_requested_emergency_stop()

        now_ms = self.supervisor._checked_now_ms()
        post_poll_lateness_ms = now_ms - self._next_tick_ms
        if post_poll_lateness_ms > self.max_lateness_ms:
            self.supervisor._enter_fault(
                "poll_deadline_missed",
                "Supervisor poll deadline missed by {} ms".format(
                    post_poll_lateness_ms
                ),
            )
            self._next_tick_ms = now_ms + self.interval_ms
            self._perform_requested_emergency_stop()
            completion, dispatch_error = self._dispatch_one(
                dispatch_one
            )
            self._complete_dispatch(
                completion,
                post_dispatch_error=dispatch_error,
            )
            self._sleep_after_deadline_fault()
            return self.supervisor.status()

        # Dispatch at most one item after a completed safety poll even when
        # the exact interval has been consumed.  The configured lateness
        # budget remains authoritative, and any dispatch-caused overrun
        # below faults and stops before another request can be admitted.
        completion, dispatch_error = self._dispatch_one(dispatch_one)
        post_dispatch_error = dispatch_error

        now_ms = self.supervisor._checked_now_ms()
        lateness_ms = now_ms - self._next_tick_ms
        if lateness_ms > self.max_lateness_ms:
            deadline_error = SupervisorError(
                "poll_deadline_missed",
                "Supervisor poll deadline missed by {} ms".format(
                    lateness_ms
                ),
            )
            self.supervisor._enter_fault(
                deadline_error.code,
                str(deadline_error),
            )
            self._next_tick_ms = now_ms + self.interval_ms
            deadline_stop_outcome = (
                self._perform_requested_emergency_stop()
            )
            self._complete_dispatch(
                completion,
                post_dispatch_error=self._prefer_dispatch_error(
                    deadline_error,
                    self._prefer_dispatch_error(
                        deadline_stop_outcome,
                        post_dispatch_error,
                    ),
                ),
            )
            self._sleep_after_deadline_fault()
            return self.supervisor.status()

        self._complete_dispatch(
            completion,
            post_dispatch_error=post_dispatch_error,
        )
        now_ms = self.supervisor._checked_now_ms()
        completion_lateness_ms = now_ms - self._next_tick_ms
        if completion_lateness_ms > self.max_lateness_ms:
            self.supervisor._enter_fault(
                "poll_deadline_missed",
                "Supervisor completion deadline missed by {} ms".format(
                    completion_lateness_ms
                ),
            )
            self._next_tick_ms = now_ms + self.interval_ms
            late_stop_outcome = (
                self._perform_requested_emergency_stop()
            )
            if (
                late_stop_outcome is not None
                and getattr(
                    late_stop_outcome,
                    "code",
                    None,
                )
                == "stop_not_confirmed"
            ):
                raise late_stop_outcome
            self._sleep_after_deadline_fault()
            return self.supervisor.status()

        remaining_ms = self._next_tick_ms - now_ms
        if remaining_ms > 0:
            self.supervisor.robot.sleep_fn(
                remaining_ms / 1000.0
            )
        return self.supervisor.status()

    def run_forever(self, shutdown_requested, dispatch_one=None):
        if not callable(shutdown_requested):
            raise SupervisorError(
                "invalid_shutdown_signal",
                "shutdown_requested must be callable",
            )
        if dispatch_one is not None and not callable(dispatch_one):
            raise SupervisorError(
                "invalid_dispatch_hook",
                "dispatch_one must be callable",
            )
        self.supervisor.bind_to_current_thread()
        exit_status = None
        loop_error = None
        close_error = None
        try:
            while not shutdown_requested():
                exit_status = self.run_once(
                    dispatch_one=dispatch_one,
                )
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
