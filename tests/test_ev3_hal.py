import fcntl
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import ev3.robot_hal as robot_hal_module
from ev3.robot_hal import (
    MotionVerificationError,
    MotorBusyError,
    RobotHAL,
    SafetyError,
    SpeechBusyError,
    read_text,
)


PROJECT_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "ev3rstorm.json"


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(value), encoding="ascii")


class FakeSysfs:
    def __init__(self, root):
        self.root = Path(root)
        self.motors = {}
        self.sensors = {}

    def add_motor(self, name, port, driver, position=0, max_speed=1050):
        path = self.root / "tacho-motor" / name
        values = {
            "address": "ev3-ports:{}".format(port),
            "driver_name": driver,
            "position": position,
            "state": "",
            "max_speed": max_speed,
            "speed_sp": 0,
            "time_sp": 0,
            "stop_action": "coast",
            "command": "",
        }
        for filename, value in values.items():
            write(path / filename, value)
        self.motors[port] = path

    def add_sensor(self, name, port, driver, mode, value, units=""):
        path = self.root / "lego-sensor" / name
        values = {
            "address": "ev3-ports:{}".format(port),
            "driver_name": driver,
            "mode": mode,
            "value0": value,
            "units": units,
        }
        for filename, item in values.items():
            write(path / filename, item)
        self.sensors[port] = path

    def add_battery(self):
        root = self.root / "power_supply" / "lego-ev3-battery"
        write(root / "voltage_now", 9_011_800)
        write(root / "current_now", 156_000)


class RobotHALTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.sysfs = FakeSysfs(Path(self.temp.name) / "class")
        self.sysfs.add_motor(
            "motor0", "outA", "lego-ev3-m-motor", position=778, max_speed=1560
        )
        self.sysfs.add_motor("motor1", "outC", "lego-ev3-l-motor")
        self.sysfs.add_motor("motor2", "outB", "lego-ev3-l-motor", position=14)
        self.sysfs.add_sensor(
            "sensor0", "in1", "lego-ev3-touch", "TOUCH", 0
        )
        self.sysfs.add_sensor(
            "sensor1", "in4", "lego-ev3-ir", "IR-PROX", 90, "pct"
        )
        self.sysfs.add_sensor(
            "sensor2", "in3", "lego-ev3-color", "COL-REFLECT", 4, "pct"
        )
        self.sysfs.add_battery()
        self.lock_path = str(Path(self.temp.name) / "motor.lock")
        self.speech_lock_path = str(Path(self.temp.name) / "audio.lock")
        self.now = 1.0

    def tearDown(self):
        self.temp.cleanup()

    def hal(self, sleep_fn=lambda _seconds: None):
        return RobotHAL(
            str(CONFIG_PATH),
            sysfs_root=str(self.sysfs.root),
            lock_path=self.lock_path,
            sleep_fn=sleep_fn,
            monotonic_fn=lambda: self.now,
            speech_lock_path=self.speech_lock_path,
        )

    def test_inventory_maps_by_physical_port_not_kernel_index(self):
        inventory = self.hal().inventory()
        self.assertEqual(inventory["motors"]["drive_b"]["port"], "outB")
        self.assertEqual(inventory["motors"]["drive_b"]["position"], 14)
        self.assertEqual(
            inventory["sensors"]["infrared"]["driver"], "lego-ev3-ir"
        )
        self.assertEqual(inventory["battery"]["voltage_uv"], 9_011_800)

    def test_read_sensor_is_timestamped_and_mode_checked(self):
        result = self.hal().read_sensor("infrared")

        self.assertEqual(result["role"], "infrared")
        self.assertEqual(result["port"], "in4")
        self.assertEqual(result["mode"], "IR-PROX")
        self.assertEqual(result["value0"], 90)
        self.assertEqual(result["units"], "pct")
        self.assertEqual(result["observed_monotonic_ms"], 1000)

        write(self.sysfs.sensors["in4"] / "mode", "IR-REMOTE")
        with self.assertRaises(SafetyError):
            self.hal().read_sensor("infrared")

        with self.assertRaises(SafetyError):
            self.hal().read_sensor("imaginary")

    def test_run_timed_always_finishes_with_explicit_stop(self):
        motor_path = self.sysfs.motors["outB"]

        def simulate_kernel_motion(_seconds):
            write(motor_path / "position", 134)
            self.now += 0.7

        result = self.hal(simulate_kernel_motion).run_timed("drive_b", 200, 600)

        self.assertEqual(result["position_delta"], 120)
        self.assertEqual(read_text(str(motor_path / "speed_sp")), "200")
        self.assertEqual(read_text(str(motor_path / "time_sp")), "600")
        self.assertEqual(read_text(str(motor_path / "stop_action")), "brake")
        self.assertEqual(read_text(str(motor_path / "command")), "stop")
        self.assertTrue(result["verification"]["passed"])

    def test_run_timed_zero_encoder_delta_is_failed(self):
        motor_path = self.sysfs.motors["outB"]

        with self.assertRaises(MotionVerificationError) as context:
            self.hal().run_timed("drive_b", 100, 300)

        check = context.exception.checks[0]
        self.assertEqual(check["role"], "drive_b")
        self.assertEqual(check["position_delta"], 0)
        self.assertFalse(check["enough_motion"])
        self.assertEqual(read_text(str(motor_path / "command")), "stop")

    def test_run_timed_wrong_encoder_direction_is_failed(self):
        motor_path = self.sysfs.motors["outB"]

        def simulate_wrong_direction(_seconds):
            write(motor_path / "position", -16)
            self.now += 0.4

        with self.assertRaises(MotionVerificationError) as context:
            self.hal(simulate_wrong_direction).run_timed(
                "drive_b", 100, 300
            )

        check = context.exception.checks[0]
        self.assertEqual(check["position_delta"], -30)
        self.assertFalse(check["direction_matches"])
        self.assertEqual(read_text(str(motor_path / "command")), "stop")

    def test_unsafe_motion_is_rejected_before_command_write(self):
        motor_path = self.sysfs.motors["outB"]
        hal = self.hal()
        unsafe_speed = (
            hal.config["limits"]["drive"]["max_abs_speed_dps"] + 1
        )
        with self.assertRaises(SafetyError):
            hal.run_timed("drive_b", unsafe_speed, 600)
        self.assertEqual(read_text(str(motor_path / "command")), "")

    def test_wrong_motor_driver_is_rejected_before_motion(self):
        motor_path = self.sysfs.motors["outB"]
        write(motor_path / "driver_name", "lego-ev3-m-motor")
        with self.assertRaises(SafetyError):
            self.hal().run_timed("drive_b", 100, 100)
        self.assertEqual(read_text(str(motor_path / "command")), "")

    def test_motor_lock_prevents_competing_owner(self):
        lock_handle = open(self.lock_path, "a+")
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            with self.assertRaises(MotorBusyError):
                self.hal().run_timed("drive_b", 100, 100)
        finally:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
            lock_handle.close()

    def test_drive_timed_starts_and_stops_both_motors(self):
        left_path = self.sysfs.motors["outB"]
        right_path = self.sysfs.motors["outC"]

        def simulate_kernel_motion(_seconds):
            self.assertEqual(
                read_text(str(left_path / "command")), "run-timed"
            )
            self.assertEqual(
                read_text(str(right_path / "command")), "run-timed"
            )
            write(left_path / "position", 44)
            write(right_path / "position", 30)
            self.now += 0.4

        result = self.hal(simulate_kernel_motion).drive_timed(100, 100, 300)

        self.assertEqual(result["action"], "drive_timed")
        self.assertEqual(result["motors"]["left"]["role"], "drive_b")
        self.assertEqual(result["motors"]["right"]["role"], "drive_c")
        self.assertEqual(result["motors"]["left"]["position_delta"], 30)
        self.assertEqual(result["motors"]["right"]["position_delta"], 30)
        self.assertEqual(result["start_skew_ms"], 0)
        self.assertEqual(read_text(str(left_path / "speed_sp")), "100")
        self.assertEqual(read_text(str(right_path / "speed_sp")), "100")
        self.assertEqual(read_text(str(left_path / "time_sp")), "300")
        self.assertEqual(read_text(str(right_path / "time_sp")), "300")
        self.assertEqual(read_text(str(left_path / "stop_action")), "brake")
        self.assertEqual(read_text(str(right_path / "stop_action")), "brake")
        self.assertEqual(read_text(str(left_path / "command")), "stop")
        self.assertEqual(read_text(str(right_path / "command")), "stop")
        self.assertTrue(result["motors"]["right"]["verification"]["passed"])

    def test_drive_timed_rejects_either_unsafe_speed_before_writes(self):
        left_path = self.sysfs.motors["outB"]
        right_path = self.sysfs.motors["outC"]
        hal = self.hal()
        unsafe_speed = (
            hal.config["limits"]["drive"]["max_abs_speed_dps"] + 1
        )

        with self.assertRaises(SafetyError):
            hal.drive_timed(100, unsafe_speed, 300)

        self.assertEqual(read_text(str(left_path / "command")), "")
        self.assertEqual(read_text(str(right_path / "command")), "")

    def test_drive_timed_resolves_both_motors_before_writes(self):
        left_path = self.sysfs.motors["outB"]
        right_path = self.sysfs.motors["outC"]
        write(right_path / "driver_name", "lego-ev3-m-motor")

        with self.assertRaises(SafetyError):
            self.hal().drive_timed(100, 100, 300)

        self.assertEqual(read_text(str(left_path / "command")), "")
        self.assertEqual(read_text(str(right_path / "command")), "")

    def test_drive_timed_partial_start_failure_stops_both_motors(self):
        left_path = self.sysfs.motors["outB"]
        right_path = self.sysfs.motors["outC"]
        original_write_text = robot_hal_module.write_text

        def fail_right_start(path, value):
            if (
                path == str(right_path / "command")
                and value == "run-timed"
            ):
                raise IOError("injected right motor start failure")
            original_write_text(path, value)

        with patch.object(
            robot_hal_module, "write_text", side_effect=fail_right_start
        ):
            with self.assertRaises(IOError):
                self.hal().drive_timed(100, 100, 300)

        self.assertEqual(read_text(str(left_path / "command")), "stop")
        self.assertEqual(read_text(str(right_path / "command")), "stop")

    def test_drive_timed_zero_delta_on_one_side_is_failed(self):
        left_path = self.sysfs.motors["outB"]
        right_path = self.sysfs.motors["outC"]

        def simulate_one_sided_motion(_seconds):
            write(left_path / "position", 44)
            self.now += 0.4

        with self.assertRaises(MotionVerificationError) as context:
            self.hal(simulate_one_sided_motion).drive_timed(100, 100, 300)

        self.assertEqual(len(context.exception.checks), 1)
        check = context.exception.checks[0]
        self.assertEqual(check["role"], "drive_c")
        self.assertEqual(check["position_delta"], 0)
        self.assertEqual(read_text(str(left_path / "command")), "stop")
        self.assertEqual(read_text(str(right_path / "command")), "stop")

    def test_drive_timed_lock_contention_writes_nothing(self):
        left_path = self.sysfs.motors["outB"]
        right_path = self.sysfs.motors["outC"]
        lock_handle = open(self.lock_path, "a+")
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            with self.assertRaises(MotorBusyError):
                self.hal().drive_timed(100, 100, 300)
        finally:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
            lock_handle.close()

        self.assertEqual(read_text(str(left_path / "command")), "")
        self.assertEqual(read_text(str(right_path / "command")), "")

    def test_drive_timed_stop_failure_still_stops_other_motor(self):
        left_path = self.sysfs.motors["outB"]
        right_path = self.sysfs.motors["outC"]
        original_write_text = robot_hal_module.write_text
        left_stop_attempts = []

        def fail_first_left_stop(path, value):
            if path == str(left_path / "command") and value == "stop":
                left_stop_attempts.append(path)
                if len(left_stop_attempts) == 1:
                    raise IOError("injected left motor stop failure")
            original_write_text(path, value)

        with patch.object(
            robot_hal_module,
            "write_text",
            side_effect=fail_first_left_stop,
        ):
            with self.assertRaises(RuntimeError):
                self.hal().drive_timed(100, 100, 300)

        self.assertGreaterEqual(len(left_stop_attempts), 2)
        self.assertEqual(read_text(str(left_path / "command")), "stop")
        self.assertEqual(read_text(str(right_path / "command")), "stop")

    def test_drive_timed_applies_configured_forward_sign(self):
        left_path = self.sysfs.motors["outB"]
        right_path = self.sysfs.motors["outC"]

        def simulate_signed_motion(_seconds):
            write(left_path / "position", -16)
            write(right_path / "position", 30)
            self.now += 0.4

        robot = self.hal(simulate_signed_motion)
        robot.config["drive_geometry"]["forward_speed_sign"]["drive_b"] = -1

        result = robot.drive_timed(100, 100, 300)

        self.assertEqual(read_text(str(left_path / "speed_sp")), "-100")
        self.assertEqual(
            result["motors"]["left"]["logical_speed_dps"], 100
        )
        self.assertEqual(
            result["motors"]["left"]["physical_speed_dps"], -100
        )

    def test_drive_timed_rejects_duplicate_geometry_before_writes(self):
        left_path = self.sysfs.motors["outB"]
        right_path = self.sysfs.motors["outC"]
        robot = self.hal()
        robot.config["drive_geometry"]["right_motor_role"] = "drive_b"

        with self.assertRaises(SafetyError):
            robot.drive_timed(100, 100, 300)

        self.assertEqual(read_text(str(left_path / "command")), "")
        self.assertEqual(read_text(str(right_path / "command")), "")

    def test_stop_all_writes_stop_to_every_motor(self):
        result = self.hal().stop_all()
        self.assertEqual(result["status"], "stopped")
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(
            result["addresses"],
            ["ev3-ports:outA", "ev3-ports:outC", "ev3-ports:outB"],
        )
        self.assertEqual(result["motor_count"], 3)
        self.assertEqual(result["errors"], [])
        self.assertEqual(result["required_stable_intervals"], 5)
        self.assertEqual(
            len(result["stop_attempts"]),
            3 * (result["required_stable_intervals"] + 1),
        )
        for motor_path in self.sysfs.motors.values():
            self.assertEqual(read_text(str(motor_path / "command")), "stop")
            self.assertEqual(
                read_text(str(motor_path / "stop_action")), "brake"
            )

    def test_stop_all_reports_partial_write_failure_and_continues(self):
        failed_path = self.sysfs.motors["outB"]
        other_paths = [
            self.sysfs.motors["outA"],
            self.sysfs.motors["outC"],
        ]
        original_write_text = robot_hal_module.write_text

        def fail_one_motor(path, value):
            if (
                path == str(failed_path / "command")
                and value == "stop"
            ):
                raise IOError("injected persistent stop failure")
            original_write_text(path, value)

        with patch.object(
            robot_hal_module,
            "write_text",
            side_effect=fail_one_motor,
        ):
            result = self.hal().stop_all()

        self.assertEqual(result["status"], "failed")
        self.assertFalse(result["stop_confirmed"])
        self.assertTrue(
            any("persistent stop failure" in item for item in result["errors"])
        )
        failed_attempts = [
            item
            for item in result["stop_attempts"]
            if item["path"] == str(failed_path)
        ]
        self.assertTrue(failed_attempts)
        self.assertTrue(
            all(
                not item["stop_command_written"]
                for item in failed_attempts
            )
        )
        for motor_path in other_paths:
            self.assertEqual(
                read_text(str(motor_path / "command")), "stop"
            )

    def test_stop_all_reports_verification_read_failure(self):
        failed_path = self.sysfs.motors["outC"]
        original_read_text = robot_hal_module.read_text

        def fail_state_read(path):
            if path == str(failed_path / "state"):
                raise IOError("injected state read failure")
            return original_read_text(path)

        with patch.object(
            robot_hal_module,
            "read_text",
            side_effect=fail_state_read,
        ):
            result = self.hal().stop_all()

        self.assertEqual(result["status"], "failed")
        self.assertFalse(result["stop_confirmed"])
        self.assertTrue(
            any("state read failure" in item for item in result["errors"])
        )
        for motor_path in self.sysfs.motors.values():
            self.assertEqual(
                read_text(str(motor_path / "command")), "stop"
            )

    def test_stop_all_never_confirms_identity_read_failure(self):
        failed_path = self.sysfs.motors["outB"]
        original_read_text = robot_hal_module.read_text

        def fail_address_read(path):
            if path == str(failed_path / "address"):
                raise IOError("injected address read failure")
            return original_read_text(path)

        with patch.object(
            robot_hal_module,
            "read_text",
            side_effect=fail_address_read,
        ):
            result = self.hal().stop_all()

        self.assertTrue(result["hardware_stop_verified"])
        self.assertFalse(result["stop_confirmed"])
        self.assertTrue(
            any("address read failure" in item for item in result["errors"])
        )

    def test_stop_all_does_not_accept_zero_zero_one_coasting_pattern(self):
        motor_path = self.sysfs.motors["outB"]
        scripted_positions = iter(
            [14, 14] + list(range(15, 40))
        )
        original_read_text = robot_hal_module.read_text
        position_reads = []

        def read_coasting_position(path):
            if path == str(motor_path / "position"):
                value = next(scripted_positions)
                position_reads.append(value)
                return str(value)
            return original_read_text(path)

        with patch.object(
            robot_hal_module,
            "read_text",
            side_effect=read_coasting_position,
        ):
            result = self.hal().stop_all()

        self.assertGreater(len(position_reads), 2)
        self.assertEqual(position_reads[:3], [14, 14, 15])
        self.assertFalse(result["hardware_stop_verified"])
        self.assertFalse(result["stop_confirmed"])
        self.assertEqual(result["observed_stable_intervals"], 0)

    def test_stop_commands_precede_descriptive_topology_reads(self):
        original_read_text = robot_hal_module.read_text
        identity_reads = []

        def require_stopped_before_identity(path):
            if path.endswith("/address") or path.endswith("/driver_name"):
                identity_reads.append(path)
                for motor_path in self.sysfs.motors.values():
                    self.assertEqual(
                        original_read_text(str(motor_path / "command")),
                        "stop",
                    )
            return original_read_text(path)

        with patch.object(
            robot_hal_module,
            "read_text",
            side_effect=require_stopped_before_identity,
        ):
            result = self.hal().stop_all()

        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(len(identity_reads), 6)

    def test_stop_all_does_not_confirm_active_or_faulted_motor(self):
        motor_path = self.sysfs.motors["outC"]
        write(motor_path / "state", "running stalled")

        result = self.hal().stop_all()

        self.assertEqual(result["status"], "failed")
        self.assertFalse(result["stop_confirmed"])
        self.assertEqual(
            result["fault_tokens"][str(motor_path)], ["stalled"]
        )
        self.assertTrue(
            any(
                "not verified before deadline" in item
                for item in result["errors"]
            )
        )

    def test_stop_all_does_not_confirm_missing_configured_motor(self):
        missing_path = self.sysfs.motors["outA"]
        for child in missing_path.iterdir():
            child.unlink()
        missing_path.rmdir()

        result = self.hal().stop_all()

        self.assertEqual(result["status"], "failed")
        self.assertFalse(result["stop_confirmed"])
        self.assertTrue(
            any(
                "Configured motor arm on outA was not discovered" in item
                for item in result["errors"]
            )
        )
        for port in ("outB", "outC"):
            self.assertEqual(
                read_text(str(self.sysfs.motors[port] / "command")),
                "stop",
            )

    def test_unconfigured_emergency_stop_attempt_is_never_fully_confirmed(self):
        result = RobotHAL.emergency_stop_unconfigured(
            "configuration contains duplicate keys",
            sysfs_root=str(self.sysfs.root),
            sleep_fn=lambda _seconds: None,
        )

        self.assertEqual(result["status"], "failed")
        self.assertFalse(result["configuration_valid"])
        self.assertTrue(result["hardware_stop_verified"])
        self.assertFalse(result["stop_confirmed"])
        self.assertTrue(
            any(
                "configuration contains duplicate keys" in item
                for item in result["errors"]
            )
        )
        for motor_path in self.sysfs.motors.values():
            self.assertEqual(
                read_text(str(motor_path / "command")), "stop"
            )

    def _speech_processes(self):
        producer = unittest.mock.MagicMock()
        producer.stdin = unittest.mock.MagicMock()
        producer.stdout = unittest.mock.MagicMock()
        producer.stderr = unittest.mock.MagicMock()
        producer.stderr.read.return_value = b""
        producer.wait.return_value = 0
        producer.poll.return_value = 0
        producer.returncode = 0

        consumer = unittest.mock.MagicMock()
        consumer.communicate.return_value = (b"", b"")
        consumer.poll.return_value = 0
        consumer.returncode = 0
        return producer, consumer

    def test_speak_uses_stdin_and_returns_bounded_result(self):
        producer, consumer = self._speech_processes()
        with patch.object(
            robot_hal_module.subprocess,
            "Popen",
            side_effect=[producer, consumer],
        ) as popen:
            result = self.hal().speak(
                "-det här är svenska", "sv", 125
            )

        producer_args = popen.call_args_list[0][0][0]
        self.assertIn("--stdin", producer_args)
        self.assertNotIn("-det här är svenska", producer_args)
        producer.stdin.write.assert_called_once_with(
            "-det här är svenska\n".encode("utf-8")
        )
        producer.stdin.close.assert_called_once_with()
        producer.stdout.close.assert_called_once_with()
        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["characters"], 19)
        self.assertEqual(result["amplitude"], 140)

    def test_speak_rejects_invalid_values_before_starting_process(self):
        robot = self.hal()
        invalid_calls = [
            (b"bytes", "sv", 135, 100),
            ("", "sv", 135, 100),
            ("   ", "sv", 135, 100),
            ("x" * 161, "sv", 135, 100),
            ("hej", "unknown", 135, 100),
            ("hej", "sv", True, 100),
            ("hej", "sv", 99, 100),
            ("hej", "sv", 221, 100),
            ("hej", "sv", 135, True),
            ("hej", "sv", 135, 161),
        ]
        with patch.object(robot_hal_module.subprocess, "Popen") as popen:
            for values in invalid_calls:
                with self.subTest(values=values):
                    with self.assertRaises(SafetyError):
                        robot.speak(*values)
            popen.assert_not_called()

    def test_speak_timeout_kills_both_processes_and_releases_lock(self):
        producer, consumer = self._speech_processes()
        producer.poll.return_value = None
        consumer.poll.return_value = None
        consumer.communicate.side_effect = subprocess.TimeoutExpired(
            "aplay", 20
        )

        with patch.object(
            robot_hal_module.subprocess,
            "Popen",
            side_effect=[producer, consumer],
        ):
            with self.assertRaises(RuntimeError):
                self.hal().speak("Hej", amplitude=100)

        producer.kill.assert_called_once_with()
        consumer.kill.assert_called_once_with()

        next_producer, next_consumer = self._speech_processes()
        with patch.object(
            robot_hal_module.subprocess,
            "Popen",
            side_effect=[next_producer, next_consumer],
        ):
            self.hal().speak("Låset släpptes", amplitude=100)

    def test_speak_signal_interrupt_kills_children_and_restores_handler(self):
        producer, consumer = self._speech_processes()
        producer.poll.return_value = None
        consumer.poll.return_value = None
        previous = robot_hal_module.signal.getsignal(
            robot_hal_module.signal.SIGHUP
        )

        def interrupt_playback(timeout=None):
            handler = robot_hal_module.signal.getsignal(
                robot_hal_module.signal.SIGHUP
            )
            handler(robot_hal_module.signal.SIGHUP, None)

        consumer.communicate.side_effect = interrupt_playback
        with patch.object(
            robot_hal_module.subprocess,
            "Popen",
            side_effect=[producer, consumer],
        ):
            with self.assertRaises(RuntimeError):
                self.hal().speak("Avbryt mig", amplitude=100)

        producer.kill.assert_called_once_with()
        consumer.kill.assert_called_once_with()
        self.assertIs(
            robot_hal_module.signal.getsignal(robot_hal_module.signal.SIGHUP),
            previous,
        )

    def test_speech_lock_prevents_overlapping_audio(self):
        lock_handle = open(self.speech_lock_path, "a+")
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            with patch.object(robot_hal_module.subprocess, "Popen") as popen:
                with self.assertRaises(SpeechBusyError):
                    self.hal().speak("Hej", amplitude=100)
                popen.assert_not_called()
        finally:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
            lock_handle.close()

    def test_speak_reports_subprocess_failure(self):
        producer, consumer = self._speech_processes()
        producer.wait.return_value = 1
        producer.stderr.read.return_value = b"voice failed"

        with patch.object(
            robot_hal_module.subprocess,
            "Popen",
            side_effect=[producer, consumer],
        ):
            with self.assertRaises(RuntimeError) as context:
                self.hal().speak("Hej", amplitude=100)
        self.assertIn("voice failed", str(context.exception))


if __name__ == "__main__":
    unittest.main()
