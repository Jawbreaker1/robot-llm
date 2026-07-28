import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import ev3.robot_hal as robot_hal_module
from ev3.robot_hal import RobotHAL, SafetyError


PROJECT_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "ev3rstorm.json"


class RobotConfigTests(unittest.TestCase):
    def config(self):
        return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))

    def write_config(self, directory, config):
        path = Path(directory) / "ev3-config.json"
        path.write_text(json.dumps(config), encoding="utf-8")
        return path

    def assert_invalid_config(self, config, expected_detail=None):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_config(directory, config)
            with self.assertRaises(SafetyError) as context:
                RobotHAL(str(path))
        if expected_detail is not None:
            self.assertIn(expected_detail, str(context.exception))

    def assert_invalid_raw_config(self, raw, expected_detail=None):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ev3-config.json"
            path.write_text(raw, encoding="utf-8")
            with self.assertRaises(SafetyError) as context:
                RobotHAL(str(path))
        if expected_detail is not None:
            self.assertIn(expected_detail, str(context.exception))

    def test_checked_in_config_passes_strict_validation(self):
        robot = RobotHAL(str(CONFIG_PATH))

        self.assertEqual(robot.config["schema_version"], 1)
        self.assertEqual(
            robot._limit_for_role("drive_b"),
            robot.config["limits"]["drive"],
        )
        self.assertEqual(
            robot._limit_for_role("arm"),
            robot.config["limits"]["arm"],
        )

    def test_limit_profile_is_authoritative_for_renamed_roles(self):
        config = self.config()
        config["motors"]["left_leg"] = config["motors"].pop("drive_b")
        config["motors"]["right_leg"] = config["motors"].pop("drive_c")
        config["drive_geometry"]["left_motor_role"] = "left_leg"
        config["drive_geometry"]["right_motor_role"] = "right_leg"
        config["drive_geometry"]["forward_speed_sign"] = {
            "left_leg": 1,
            "right_leg": -1,
        }

        with tempfile.TemporaryDirectory() as directory:
            path = self.write_config(directory, config)
            robot = RobotHAL(str(path))

        left, right, signs = robot._drive_roles()
        self.assertEqual((left, right), ("left_leg", "right_leg"))
        self.assertEqual(signs["right_leg"], -1)
        self.assertEqual(
            robot._limit_for_role("left_leg"),
            robot.config["limits"]["drive"],
        )

    def test_duplicate_json_keys_are_rejected_at_any_depth(self):
        raw = CONFIG_PATH.read_text(encoding="utf-8")
        cases = (
            raw.replace(
                '"schema_version": 1,',
                '"schema_version": 1, "schema_version": 1,',
                1,
            ),
            raw.replace(
                '"port": "outA",',
                '"port": "outA", "port": "outD",',
                1,
            ),
        )
        for candidate in cases:
            with self.subTest(candidate=candidate[:80]):
                self.assert_invalid_raw_config(
                    candidate, "Duplicate configuration key"
                )

    def test_non_finite_numbers_are_rejected_including_overflow(self):
        raw = CONFIG_PATH.read_text(encoding="utf-8")
        cases = ("NaN", "Infinity", "-Infinity", "1e999")
        for value in cases:
            with self.subTest(value=value):
                candidate = raw.replace(
                    '"approx_encoder_degrees_per_body_degree": 7.58',
                    (
                        '"approx_encoder_degrees_per_body_degree": {}'
                    ).format(value),
                    1,
                )
                self.assert_invalid_raw_config(candidate, "configuration")

    def test_wrong_root_schema_and_unknown_keys_are_rejected(self):
        self.assert_invalid_raw_config(
            "{", "Invalid robot configuration"
        )
        self.assert_invalid_raw_config(
            "[]", "configuration must be an object"
        )

        missing = self.config()
        del missing["motors"]
        self.assert_invalid_config(missing, "missing required key")

        unknown = self.config()
        unknown["limtis"] = unknown["limits"]
        self.assert_invalid_config(unknown, "unknown key")

        wrong_version = self.config()
        wrong_version["schema_version"] = True
        self.assert_invalid_config(
            wrong_version, "schema_version must be an integer"
        )

    def test_ports_profiles_and_geometry_are_cross_validated(self):
        cases = []

        duplicate_motor_port = self.config()
        duplicate_motor_port["motors"]["arm"]["port"] = "outB"
        cases.append((duplicate_motor_port, "share port"))

        duplicate_sensor_port = self.config()
        duplicate_sensor_port["sensors"]["touch"]["port"] = "in4"
        cases.append((duplicate_sensor_port, "share port"))

        wrong_profile = self.config()
        wrong_profile["motors"]["drive_b"]["limit_profile"] = "arm"
        cases.append((wrong_profile, "must use the drive limit profile"))

        same_role = self.config()
        same_role["drive_geometry"]["right_motor_role"] = "drive_b"
        same_role["drive_geometry"]["forward_speed_sign"] = {
            "drive_b": 1
        }
        cases.append((same_role, "must be different"))

        bad_sign = self.config()
        bad_sign["drive_geometry"]["forward_speed_sign"]["drive_b"] = True
        cases.append((bad_sign, "must be -1 or 1"))

        exposed_drive = self.config()
        exposed_drive["agent_api"]["move_motor_roles"] = ["drive_b"]
        cases.append((exposed_drive, "may not be exposed"))

        for config, detail in cases:
            with self.subTest(detail=detail):
                self.assert_invalid_config(config, detail)

    def test_safety_critical_touch_sensor_is_required_and_typed(self):
        cases = []

        missing_touch = self.config()
        del missing_touch["sensors"]["touch"]
        cases.append((missing_touch, "sensors.touch is required"))

        wrong_driver = self.config()
        wrong_driver["sensors"]["touch"]["driver"] = "lego-ev3-color"
        cases.append((wrong_driver, "must be lego-ev3-touch"))

        wrong_mode = self.config()
        wrong_mode["sensors"]["touch"]["mode"] = "COL-REFLECT"
        cases.append((wrong_mode, "sensors.touch.mode must be TOUCH"))

        for config, detail in cases:
            with self.subTest(detail=detail):
                self.assert_invalid_config(config, detail)

    def test_motion_speech_and_supervisor_limits_fail_closed(self):
        cases = []

        bool_speed = self.config()
        bool_speed["limits"]["drive"]["max_abs_speed_dps"] = True
        cases.append((bool_speed, "positive integer"))

        float_speed = self.config()
        float_speed["limits"]["drive"]["max_abs_speed_dps"] = 250.0
        cases.append((float_speed, "positive integer"))

        zero_duration = self.config()
        zero_duration["limits"]["arm"]["max_duration_ms"] = 0
        cases.append((zero_duration, "positive integer"))

        negative_heartbeat = self.config()
        negative_heartbeat["limits"]["heartbeat"]["timeout_ms"] = -1
        cases.append((negative_heartbeat, "positive integer"))

        inverted_rate = self.config()
        inverted_rate["limits"]["speech"]["min_rate_wpm"] = 221
        cases.append((inverted_rate, "rate minimum exceeds"))

        invalid_default = self.config()
        invalid_default["limits"]["speech"]["default_amplitude"] = 161
        cases.append((invalid_default, "outside the configured range"))

        duplicate_voice = self.config()
        duplicate_voice["limits"]["speech"]["allowed_voices"] = [
            "sv",
            "sv",
        ]
        cases.append((duplicate_voice, "must be unique"))

        slow_stop_poll = self.config()
        slow_stop_poll["limits"]["supervisor"][
            "stop_poll_interval_ms"
        ] = 100
        slow_stop_poll["limits"]["supervisor"][
            "stop_verify_timeout_ms"
        ] = 50
        cases.append((slow_stop_poll, "exceeds stop timeout"))

        heartbeat_race = self.config()
        heartbeat_race["limits"]["supervisor"][
            "poll_interval_ms"
        ] = 50
        heartbeat_race["limits"]["heartbeat"]["timeout_ms"] = 50
        cases.append((heartbeat_race, "shorter than heartbeat"))

        insufficient_settling = self.config()
        insufficient_settling["limits"]["supervisor"][
            "stop_verify_timeout_ms"
        ] = 40
        cases.append((insufficient_settling, "required settling window"))

        excessive_ratio = self.config()
        excessive_ratio["limits"]["supervisor"][
            "min_completion_ratio_percent"
        ] = 101
        cases.append((excessive_ratio, "must be at most 100"))

        for config, detail in cases:
            with self.subTest(detail=detail):
                self.assert_invalid_config(config, detail)

    def test_absolute_safety_ceilings_accept_boundary_values(self):
        config = self.config()
        config["limits"]["drive"]["max_abs_speed_dps"] = 1050
        config["limits"]["arm"]["max_abs_speed_dps"] = 1560
        config["limits"]["drive"]["max_duration_ms"] = 5000
        config["limits"]["arm"]["max_duration_ms"] = 5000
        config["limits"]["speech"]["max_runtime_ms"] = 60000
        config["limits"]["heartbeat"]["timeout_ms"] = 2000
        config["limits"]["supervisor"]["poll_interval_ms"] = 100
        config["limits"]["supervisor"]["max_poll_lateness_ms"] = 500
        config["limits"]["supervisor"]["max_start_skew_ms"] = 100
        config["limits"]["supervisor"]["stop_verify_timeout_ms"] = 2000
        config["limits"]["supervisor"]["stop_poll_interval_ms"] = 100

        with tempfile.TemporaryDirectory() as directory:
            path = self.write_config(directory, config)
            robot = RobotHAL(str(path))

        self.assertEqual(
            robot.config["limits"]["drive"]["max_abs_speed_dps"],
            1050,
        )
        self.assertEqual(
            robot.config["limits"]["heartbeat"]["timeout_ms"],
            2000,
        )

    def test_absolute_safety_ceilings_reject_one_above_boundary(self):
        mutations = (
            (
                ("limits", "drive", "max_abs_speed_dps"),
                1051,
                "driver maximum 1050",
            ),
            (
                ("limits", "arm", "max_abs_speed_dps"),
                1561,
                "must be at most 1560",
            ),
            (
                ("limits", "drive", "max_duration_ms"),
                5001,
                "must be at most 5000",
            ),
            (
                ("limits", "speech", "max_runtime_ms"),
                60001,
                "must be at most 60000",
            ),
            (
                ("limits", "heartbeat", "timeout_ms"),
                2001,
                "must be at most 2000",
            ),
            (
                ("limits", "supervisor", "poll_interval_ms"),
                101,
                "must be at most 100",
            ),
            (
                ("limits", "supervisor", "max_start_skew_ms"),
                101,
                "must be at most 100",
            ),
            (
                ("limits", "supervisor", "stop_verify_timeout_ms"),
                2001,
                "must be at most 2000",
            ),
            (
                ("limits", "supervisor", "stop_poll_interval_ms"),
                101,
                "must be at most 100",
            ),
        )
        for path, value, detail in mutations:
            config = self.config()
            target = config
            for name in path[:-1]:
                target = target[name]
            target[path[-1]] = value
            with self.subTest(path=path):
                self.assert_invalid_config(config, detail)

    def test_ir_gate_count_ceilings_accept_boundary_values(self):
        config = self.config()
        gate = config["calibration"]["infrared_proximity"][
            "obstacle_gate"
        ]
        gate["median_window"] = 31
        gate["enter_consecutive"] = 20
        gate["exit_consecutive"] = 20

        with tempfile.TemporaryDirectory() as directory:
            path = self.write_config(directory, config)
            robot = RobotHAL(str(path))

        loaded_gate = robot.config["calibration"]["infrared_proximity"][
            "obstacle_gate"
        ]
        self.assertEqual(loaded_gate["median_window"], 31)
        self.assertEqual(loaded_gate["enter_consecutive"], 20)
        self.assertEqual(loaded_gate["exit_consecutive"], 20)

    def test_ir_gate_count_ceilings_reject_one_above_boundary(self):
        mutations = (
            ("median_window", 32, "must be at most 31"),
            ("enter_consecutive", 21, "must be at most 20"),
            ("exit_consecutive", 21, "must be at most 20"),
        )
        for name, value, detail in mutations:
            config = self.config()
            config["calibration"]["infrared_proximity"][
                "obstacle_gate"
            ][name] = value
            with self.subTest(name=name):
                self.assert_invalid_config(config, detail)

    def test_invalid_config_is_rejected_before_sysfs_or_lock_effects(self):
        config = self.config()
        config["limits"]["drive"]["max_duration_ms"] = "800"

        with tempfile.TemporaryDirectory() as directory:
            path = self.write_config(directory, config)
            lock_path = Path(directory) / "motor.lock"
            with patch.object(
                robot_hal_module.glob, "glob"
            ) as discover, patch.object(
                robot_hal_module, "write_text"
            ) as sysfs_write, patch.object(
                robot_hal_module.fcntl, "flock"
            ) as flock:
                with self.assertRaises(SafetyError):
                    RobotHAL(str(path), lock_path=str(lock_path))

        discover.assert_not_called()
        sysfs_write.assert_not_called()
        flock.assert_not_called()
        self.assertFalse(lock_path.exists())


if __name__ == "__main__":
    unittest.main()
