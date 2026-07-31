#!/usr/bin/env python3
"""Small Python 3.5-compatible hardware layer for ev3dev-stretch."""

from __future__ import print_function

import fcntl
import glob
import io
import os
import signal
import subprocess
import sys
import threading
import time

if __package__:
    from .emergency_stop import verified_emergency_stop
    from .robot_config import load_robot_config
else:
    from emergency_stop import verified_emergency_stop
    from robot_config import load_robot_config


class SafetyError(ValueError):
    pass


class MotorBusyError(RuntimeError):
    pass


class SpeechBusyError(RuntimeError):
    pass


class SpeechInterruptedError(RuntimeError):
    pass


class MotionVerificationError(RuntimeError):
    def __init__(self, checks):
        self.checks = checks
        details = []
        for check in checks:
            details.append(
                "{} delta {} (minimum {}, expected {} direction)".format(
                    check["role"],
                    check["position_delta"],
                    check["minimum_abs_delta"],
                    check["expected_direction"],
                )
            )
        RuntimeError.__init__(
            self,
            "Encoder verification failed: {}".format("; ".join(details)),
        )


def read_text(path):
    with io.open(path, "r", encoding="ascii") as handle:
        return handle.read().strip()


def write_text(path, value):
    with io.open(path, "w", encoding="ascii") as handle:
        handle.write(str(value))
        handle.flush()


class RobotHAL(object):
    def __init__(
        self,
        config_path,
        sysfs_root="/sys/class",
        lock_path="/tmp/robot-llm-motors.lock",
        sleep_fn=time.sleep,
        monotonic_fn=time.monotonic,
        speech_lock_path="/tmp/robot-llm-audio.lock",
    ):
        try:
            self.config = load_robot_config(config_path)
        except (TypeError, ValueError) as error:
            raise SafetyError(
                "Invalid robot configuration: {}".format(error)
            )
        self.sysfs_root = sysfs_root
        self.lock_path = lock_path
        self.speech_lock_path = speech_lock_path
        self.sleep_fn = sleep_fn
        self.monotonic_fn = monotonic_fn

    def _device_by_port(self, device_class, port):
        pattern = os.path.join(self.sysfs_root, device_class, "*")
        expected_suffix = ":" + port
        for device_path in sorted(glob.glob(pattern)):
            address_path = os.path.join(device_path, "address")
            try:
                address = read_text(address_path)
            except (IOError, OSError):
                continue
            if address == port or address.endswith(expected_suffix):
                return device_path
        raise RuntimeError(
            "No {} device found on port {}".format(device_class, port)
        )

    def _motor_path_for_role(self, role):
        try:
            configured = self.config["motors"][role]
        except KeyError:
            raise SafetyError("Unknown motor role {!r}".format(role))
        motor_path = self._device_by_port("tacho-motor", configured["port"])
        actual_driver = read_text(os.path.join(motor_path, "driver_name"))
        if actual_driver != configured["driver"]:
            raise SafetyError(
                "Motor {} expected driver {} but found {}".format(
                    role, configured["driver"], actual_driver
                )
            )
        return motor_path

    def _sensor_path_for_role(self, role):
        try:
            configured = self.config["sensors"][role]
        except KeyError:
            raise SafetyError("Unknown sensor role {!r}".format(role))
        sensor_path = self._device_by_port("lego-sensor", configured["port"])
        actual_driver = read_text(os.path.join(sensor_path, "driver_name"))
        if actual_driver != configured["driver"]:
            raise SafetyError(
                "Sensor {} expected driver {} but found {}".format(
                    role, configured["driver"], actual_driver
                )
            )
        return sensor_path

    def _limit_for_role(self, role):
        try:
            limit_name = self.config["motors"][role]["limit_profile"]
            return self.config["limits"][limit_name]
        except KeyError:
            raise SafetyError("Unknown motor role {!r}".format(role))

    def _drive_roles(self):
        try:
            geometry = self.config["drive_geometry"]
            left_role = geometry["left_motor_role"]
            right_role = geometry["right_motor_role"]
            forward_signs = geometry["forward_speed_sign"]
        except KeyError as error:
            raise SafetyError(
                "Incomplete drive geometry: missing {}".format(error)
            )

        if left_role == right_role:
            raise SafetyError("Left and right drive roles must be different")

        for role in (left_role, right_role):
            if (
                role not in self.config["motors"]
                or self.config["motors"][role].get("limit_profile")
                != "drive"
            ):
                raise SafetyError(
                    "Drive geometry contains invalid role {!r}".format(role)
                )
            sign = forward_signs.get(role)
            if isinstance(sign, bool) or sign not in (-1, 1):
                raise SafetyError(
                    "Forward speed sign for {} must be -1 or 1".format(role)
                )

        left_port = self.config["motors"][left_role]["port"]
        right_port = self.config["motors"][right_role]["port"]
        if left_port == right_port:
            raise SafetyError("Left and right drive motors must use different ports")

        return left_role, right_role, forward_signs

    def _validate_motion(self, role, speed_dps, duration_ms):
        limit = self._limit_for_role(role)
        if isinstance(speed_dps, bool) or not isinstance(speed_dps, int):
            raise SafetyError("speed_dps must be an integer")
        if speed_dps == 0:
            raise SafetyError("Use stop_all instead of zero speed")
        if abs(speed_dps) > limit["max_abs_speed_dps"]:
            raise SafetyError(
                "speed_dps {} exceeds limit {}".format(
                    speed_dps, limit["max_abs_speed_dps"]
                )
            )
        if isinstance(duration_ms, bool) or not isinstance(duration_ms, int):
            raise SafetyError("duration_ms must be an integer")
        if duration_ms <= 0 or duration_ms > limit["max_duration_ms"]:
            raise SafetyError(
                "duration_ms {} is outside 1..{}".format(
                    duration_ms, limit["max_duration_ms"]
                )
            )
        if abs(speed_dps) * duration_ms < 3000:
            raise SafetyError(
                "Requested motion is too small for encoder verification"
            )


    def _encoder_check(
        self, role, physical_speed_dps, duration_ms, before, after
    ):
        position_delta = after - before
        expected_abs_delta = int(
            round(abs(physical_speed_dps) * duration_ms / 1000.0)
        )
        minimum_abs_delta = 3
        expected_direction = (
            "positive" if physical_speed_dps > 0 else "negative"
        )
        direction_matches = position_delta * physical_speed_dps > 0
        enough_motion = abs(position_delta) >= minimum_abs_delta
        return {
            "role": role,
            "passed": direction_matches and enough_motion,
            "position_delta": position_delta,
            "expected_abs_delta": expected_abs_delta,
            "minimum_abs_delta": minimum_abs_delta,
            "expected_direction": expected_direction,
            "direction_matches": direction_matches,
            "enough_motion": enough_motion,
        }

    def _require_encoder_checks(self, checks):
        failed = [check for check in checks if not check["passed"]]
        if failed:
            raise MotionVerificationError(failed)

    def _acquire_motor_lock(self):
        lock_handle = io.open(self.lock_path, "a+")
        try:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except IOError:
            lock_handle.close()
            raise MotorBusyError("Another process owns the motor lock")
        return lock_handle

    def _acquire_speech_lock(self):
        lock_handle = io.open(self.speech_lock_path, "a+")
        try:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except IOError:
            lock_handle.close()
            raise SpeechBusyError("Another process owns the audio output")
        return lock_handle

    def run_timed(self, role, speed_dps, duration_ms):
        """Run one motor with a kernel-enforced local duration."""
        self._validate_motion(role, speed_dps, duration_ms)
        motor_path = self._motor_path_for_role(role)
        lock_handle = self._acquire_motor_lock()
        started = self.monotonic_fn()
        before = None

        try:
            before = int(read_text(os.path.join(motor_path, "position")))
            write_text(os.path.join(motor_path, "speed_sp"), speed_dps)
            write_text(os.path.join(motor_path, "time_sp"), duration_ms)
            write_text(os.path.join(motor_path, "stop_action"), "brake")
            write_text(os.path.join(motor_path, "command"), "run-timed")

            # The kernel motor driver owns the actual timeout. Waiting here only
            # lets us collect a post-action encoder observation.
            self.sleep_fn((duration_ms + 100) / 1000.0)
        finally:
            try:
                write_text(os.path.join(motor_path, "command"), "stop")
            finally:
                fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
                lock_handle.close()

        after = int(read_text(os.path.join(motor_path, "position")))
        result = {
            "status": "completed",
            "role": role,
            "port": self.config["motors"][role]["port"],
            "speed_dps": speed_dps,
            "duration_ms": duration_ms,
            "position_before": before,
            "position_after": after,
            "position_delta": after - before,
            "elapsed_ms": int((self.monotonic_fn() - started) * 1000),
            "state": read_text(os.path.join(motor_path, "state")),
        }
        check = self._encoder_check(
            role, speed_dps, duration_ms, before, after
        )
        result["verification"] = check
        self._require_encoder_checks([check])
        return result

    def drive_timed(self, left_speed_dps, right_speed_dps, duration_ms):
        """Run both drive motors as one bounded, exclusively owned action.

        Speeds are logical wheel speeds: positive means forward for each side.
        The two sysfs start writes are sequential, so start_skew_ms is reported.
        Both motors retain the kernel-enforced timeout and are always stopped
        together on normal completion or a partial failure.
        """
        left_role, right_role, forward_signs = self._drive_roles()

        # Reject every bad argument before acquiring ownership or touching
        # motor sysfs.
        self._validate_motion(left_role, left_speed_dps, duration_ms)
        self._validate_motion(right_role, right_speed_dps, duration_ms)

        lock_handle = self._acquire_motor_lock()
        motors = []
        started = self.monotonic_fn()
        start_times = {}
        before = {}

        try:
            # Resolve and verify both physical devices under the same ownership
            # lock. No motor settings have been written if either check fails.
            left_path = self._motor_path_for_role(left_role)
            right_path = self._motor_path_for_role(right_role)
            if left_path == right_path:
                raise SafetyError(
                    "Left and right drive roles resolved to one motor"
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

            for motor in motors:
                before[motor["side"]] = int(
                    read_text(os.path.join(motor["path"], "position"))
                )

            # Configure both motors fully before starting either one.
            for motor in motors:
                write_text(
                    os.path.join(motor["path"], "speed_sp"),
                    motor["physical_speed_dps"],
                )
                write_text(
                    os.path.join(motor["path"], "time_sp"), duration_ms
                )
                write_text(
                    os.path.join(motor["path"], "stop_action"), "brake"
                )

            for motor in motors:
                write_text(
                    os.path.join(motor["path"], "command"), "run-timed"
                )
                start_times[motor["side"]] = self.monotonic_fn()

            # ev3dev's kernel driver owns each motor timeout. This wait is only
            # for collecting post-action encoder observations.
            self.sleep_fn((duration_ms + 100) / 1000.0)
        finally:
            active_exception = sys.exc_info()[0] is not None
            stop_errors = []
            for motor in motors:
                try:
                    write_text(os.path.join(motor["path"], "command"), "stop")
                except (IOError, OSError) as error:
                    stop_errors.append(
                        "{}: {}".format(motor["role"], error)
                    )

            # A failed targeted stop triggers a second best-effort emergency
            # stop across every physically discovered tacho motor.
            if stop_errors:
                self.stop_all()

            try:
                fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
            finally:
                lock_handle.close()

            if stop_errors and not active_exception:
                raise RuntimeError(
                    "Failed to stop motor(s): {}".format(
                        "; ".join(stop_errors)
                    )
                )

        observations = {}
        checks = []
        for motor in motors:
            side = motor["side"]
            after = int(read_text(os.path.join(motor["path"], "position")))
            observations[side] = {
                "role": motor["role"],
                "port": self.config["motors"][motor["role"]]["port"],
                "logical_speed_dps": motor["logical_speed_dps"],
                "physical_speed_dps": motor["physical_speed_dps"],
                "position_before": before[side],
                "position_after": after,
                "position_delta": after - before[side],
                "state": read_text(os.path.join(motor["path"], "state")),
            }
            check = self._encoder_check(
                motor["role"],
                motor["physical_speed_dps"],
                duration_ms,
                before[side],
                after,
            )
            observations[side]["verification"] = check
            checks.append(check)

        result = {
            "status": "completed",
            "action": "drive_timed",
            "duration_ms": duration_ms,
            "elapsed_ms": int((self.monotonic_fn() - started) * 1000),
            "start_skew_ms": int(
                abs(start_times["right"] - start_times["left"]) * 1000
            ),
            "motors": observations,
        }
        self._require_encoder_checks(checks)
        return result

    def stop_all(self):
        """Stop all tacho motors and return conservative verification."""
        expected_by_port = {}
        for role, configured in self.config["motors"].items():
            expected_by_port[configured["port"]] = {
                "role": role,
                "driver": configured["driver"],
            }
        limits = self.config["limits"]["supervisor"]
        return verified_emergency_stop(
            sysfs_root=self.sysfs_root,
            sleep_fn=self.sleep_fn,
            read_fn=read_text,
            write_fn=write_text,
            expected_motors=expected_by_port,
            timeout_ms=limits["stop_verify_timeout_ms"],
            poll_ms=limits["stop_poll_interval_ms"],
            glob_fn=glob.glob,
        )

    @staticmethod
    def emergency_stop_unconfigured(
        configuration_error,
        sysfs_root="/sys/class",
        sleep_fn=time.sleep,
    ):
        """Attempt a stop without trusting or requiring robot config."""
        return verified_emergency_stop(
            sysfs_root=sysfs_root,
            sleep_fn=sleep_fn,
            read_fn=read_text,
            write_fn=write_text,
            expected_motors=None,
            configuration_error=configuration_error,
            glob_fn=glob.glob,
        )

    def inventory(self):
        motors = {}
        for role, configured in sorted(self.config["motors"].items()):
            motor_path = self._motor_path_for_role(role)
            motors[role] = {
                "port": configured["port"],
                "driver": read_text(os.path.join(motor_path, "driver_name")),
                "position": int(read_text(os.path.join(motor_path, "position"))),
                "state": read_text(os.path.join(motor_path, "state")),
                "max_speed": int(read_text(os.path.join(motor_path, "max_speed"))),
            }

        sensors = {}
        for role, configured in sorted(self.config["sensors"].items()):
            sensor_path = self._sensor_path_for_role(role)
            sensors[role] = {
                "port": configured["port"],
                "driver": read_text(os.path.join(sensor_path, "driver_name")),
                "mode": read_text(os.path.join(sensor_path, "mode")),
                "value0": int(read_text(os.path.join(sensor_path, "value0"))),
                "units": read_text(os.path.join(sensor_path, "units")),
            }

        battery_root = os.path.join(
            self.sysfs_root, "power_supply", "lego-ev3-battery"
        )
        battery = {
            "voltage_uv": int(read_text(os.path.join(battery_root, "voltage_now"))),
            "current_ua": int(read_text(os.path.join(battery_root, "current_now"))),
        }
        return {
            "observed_monotonic_ms": int(self.monotonic_fn() * 1000),
            "motors": motors,
            "sensors": sensors,
            "battery": battery,
        }

    def read_sensor(self, role):
        """Read one configured sensor without changing its operating mode."""
        try:
            configured = self.config["sensors"][role]
        except KeyError:
            raise SafetyError("Unknown sensor role {!r}".format(role))

        sensor_path = self._sensor_path_for_role(role)
        actual_mode = read_text(os.path.join(sensor_path, "mode"))
        expected_mode = configured.get("mode")
        if expected_mode and actual_mode != expected_mode:
            raise SafetyError(
                "Sensor {} expected mode {} but found {}".format(
                    role, expected_mode, actual_mode
                )
            )

        return {
            "observed_monotonic_ms": int(self.monotonic_fn() * 1000),
            "role": role,
            "port": configured["port"],
            "driver": configured["driver"],
            "mode": actual_mode,
            "value0": int(read_text(os.path.join(sensor_path, "value0"))),
            "units": read_text(os.path.join(sensor_path, "units")),
        }

    def _validate_speech(self, text, voice, rate_wpm, amplitude):
        speech_limit = self.config["limits"]["speech"]
        if not isinstance(text, str):
            raise SafetyError("Speech text must be a string")
        if "\x00" in text:
            raise SafetyError("Speech text must not contain NUL")

        normalized_text = " ".join(text.split())
        if not normalized_text or len(normalized_text) > speech_limit[
            "max_characters"
        ]:
            raise SafetyError(
                "Speech text must contain 1..{} characters".format(
                    speech_limit["max_characters"]
                )
            )

        if not isinstance(voice, str) or voice not in speech_limit[
            "allowed_voices"
        ]:
            raise SafetyError("Speech voice is not allowed")

        if isinstance(rate_wpm, bool) or not isinstance(rate_wpm, int):
            raise SafetyError("Speech rate must be an integer")
        if rate_wpm < speech_limit["min_rate_wpm"] or rate_wpm > speech_limit[
            "max_rate_wpm"
        ]:
            raise SafetyError("Speech rate is outside configured limits")

        if isinstance(amplitude, bool) or not isinstance(amplitude, int):
            raise SafetyError("Speech amplitude must be an integer")
        if amplitude < speech_limit["min_amplitude"] or amplitude > speech_limit[
            "max_amplitude"
        ]:
            raise SafetyError("Speech amplitude is outside configured limits")
        return normalized_text

    def _terminate_process(self, process):
        if process is None:
            return
        try:
            if process.poll() is None:
                process.kill()
        except (IOError, OSError):
            pass
        try:
            process.wait(timeout=1.0)
        except (IOError, OSError, subprocess.TimeoutExpired):
            pass

    def speak(self, text, voice="sv", rate_wpm=135, amplitude=None):
        """Speak bounded text locally without involving the motor lock.

        Text is supplied over stdin, never interpreted as shell syntax or an
        eSpeak command-line option. The independent audio lock prevents
        overlapping utterances, and the whole pipeline has a hard deadline.
        """
        if amplitude is None:
            amplitude = self.config["limits"]["speech"][
                "default_amplitude"
            ]
        normalized_text = self._validate_speech(
            text, voice, rate_wpm, amplitude
        )
        speech_limit = self.config["limits"]["speech"]
        timeout_seconds = speech_limit["max_runtime_ms"] / 1000.0
        lock_handle = self._acquire_speech_lock()
        producer = None
        consumer = None
        previous_signal_handlers = {}
        started = self.monotonic_fn()

        try:
            if threading.current_thread() is threading.main_thread():
                def interrupt_speech(signum, _frame):
                    raise SpeechInterruptedError(
                        "Speech interrupted by signal {}".format(signum)
                    )

                for signum in (signal.SIGHUP, signal.SIGTERM):
                    previous_signal_handlers[signum] = signal.getsignal(
                        signum
                    )
                    signal.signal(signum, interrupt_speech)
            producer = subprocess.Popen(
                [
                    "espeak",
                    "-v",
                    voice,
                    "-s",
                    str(rate_wpm),
                    "-a",
                    str(amplitude),
                    "-b",
                    "1",
                    "--stdout",
                    "--stdin",
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            consumer = subprocess.Popen(
                ["aplay", "--quiet"],
                stdin=producer.stdout,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            producer.stdout.close()
            producer.stdin.write(
                (normalized_text + "\n").encode("utf-8")
            )
            producer.stdin.close()

            playback_output, playback_error = consumer.communicate(
                timeout=timeout_seconds
            )
            producer_result = producer.wait(timeout=1.0)
            producer_error = producer.stderr.read()

            if producer_result != 0 or consumer.returncode != 0:
                raise RuntimeError(
                    "Speech failed: espeak={}, aplay={}, detail={!r}".format(
                        producer_result,
                        consumer.returncode,
                        (
                            producer_error + playback_error + playback_output
                        )[:240].decode("utf-8", "replace"),
                    )
                )
        except subprocess.TimeoutExpired:
            self._terminate_process(consumer)
            self._terminate_process(producer)
            raise RuntimeError(
                "Speech exceeded the configured {} ms deadline".format(
                    speech_limit["max_runtime_ms"]
                )
            )
        except Exception:
            self._terminate_process(consumer)
            self._terminate_process(producer)
            raise
        finally:
            try:
                for signum, handler in previous_signal_handlers.items():
                    try:
                        signal.signal(signum, handler)
                    except (ValueError, OSError, RuntimeError):
                        pass
            finally:
                try:
                    fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
                finally:
                    lock_handle.close()

        return {
            "status": "completed",
            "characters": len(normalized_text),
            "voice": voice,
            "rate_wpm": rate_wpm,
            "amplitude": amplitude,
            "elapsed_ms": int((self.monotonic_fn() - started) * 1000),
        }
