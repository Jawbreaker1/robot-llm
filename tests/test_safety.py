import json
import unittest
from pathlib import Path
import tempfile
from types import MappingProxyType

from robot_agent import (
    MotionCommand,
    SafetyLimits,
    SafetyPolicy,
    SafetyViolation,
    SimulatedRobot,
)


CONFIG_PATH = Path(__file__).parents[1] / "config" / "ev3rstorm.json"


class MutableClock:
    def __init__(self, now_ms=10_000):
        self.now_ms = now_ms

    def __call__(self):
        return self.now_ms


class SafetyPolicyTests(unittest.TestCase):
    def setUp(self):
        self.limits = SafetyLimits.from_file(CONFIG_PATH)
        self.policy = SafetyPolicy(self.limits)
        self.clock = MutableClock()

    def command(self, **overrides):
        values = {
            "command_id": "cmd-1",
            "motor_role": "drive_b",
            "speed_dps": 200,
            "duration_ms": 600,
            "issued_at_ms": self.clock(),
        }
        values.update(overrides)
        return MotionCommand(**values)

    def assert_violation(self, code, **overrides):
        with self.assertRaises(SafetyViolation) as context:
            self.policy.validate_motion(self.command(**overrides), self.clock())
        self.assertEqual(context.exception.code, code)

    def test_verified_drive_b_command_is_allowed(self):
        self.policy.validate_motion(self.command(), self.clock())

    def test_speed_limit_is_enforced(self):
        self.assert_violation("speed_limit", speed_dps=251)
        self.assert_violation("speed_limit", speed_dps=-251)

    def test_duration_limit_is_enforced(self):
        self.assert_violation("duration_limit", duration_ms=801)

    def test_unknown_motor_is_rejected(self):
        self.assert_violation("unknown_motor", motor_role="imaginary_motor")

    def test_stale_and_future_commands_are_rejected(self):
        self.assert_violation("stale_command", issued_at_ms=8_999)
        self.assert_violation("future_command", issued_at_ms=10_001)

    def test_bool_is_not_accepted_as_integer(self):
        self.assert_violation("invalid_speed", speed_dps=True)
        self.assert_violation("invalid_duration", duration_ms=True)


class SimulatedRobotTests(unittest.TestCase):
    def setUp(self):
        self.clock = MutableClock()
        self.robot = SimulatedRobot(
            SafetyPolicy(SafetyLimits.from_file(CONFIG_PATH)), self.clock
        )

    def test_motion_updates_encoder_deterministically(self):
        command = MotionCommand(
            command_id="cmd-motion",
            motor_role="drive_b",
            speed_dps=200,
            duration_ms=600,
            issued_at_ms=self.clock(),
        )
        result = self.robot.execute_motion(command)

        self.assertEqual(result.status, "completed")
        self.assertEqual(result.completed_at_ms, result.started_at_ms)
        self.assertEqual(
            result.detail,
            "accelerated synchronous simulated motion",
        )
        self.assertEqual(result.position_before, 0)
        self.assertEqual(result.position_after, 120)
        self.assertEqual(
            self.robot.read_state().motors["drive_b"].position_degrees, 120
        )

    def test_duplicate_command_id_is_rejected(self):
        command = MotionCommand(
            command_id="same-id",
            motor_role="drive_b",
            speed_dps=100,
            duration_ms=100,
            issued_at_ms=self.clock(),
        )
        self.robot.execute_motion(command)
        with self.assertRaises(SafetyViolation) as context:
            self.robot.execute_motion(command)
        self.assertEqual(context.exception.code, "duplicate_command")

    def test_sensor_state_is_explicit(self):
        self.robot.set_sensor("touch", True)
        self.assertTrue(self.robot.read_state().sensors["touch"])


class SafetyConfigurationTests(unittest.TestCase):
    def config(self):
        with CONFIG_PATH.open("r", encoding="utf-8") as handle:
            return json.load(handle)

    def write_config(self, config):
        temporary = tempfile.TemporaryDirectory()
        path = Path(temporary.name) / "config.json"
        path.write_text(json.dumps(config), encoding="utf-8")
        return temporary, path

    def test_loaded_mappings_are_immutable(self):
        limits = SafetyLimits.from_file(CONFIG_PATH)

        self.assertIsInstance(limits.motor_roles, MappingProxyType)
        self.assertIsInstance(limits.role_limits, MappingProxyType)
        with self.assertRaises(TypeError):
            limits.motor_roles["arm"] = "outD"
        with self.assertRaises(TypeError):
            limits.role_limits["arm"] = limits.drive

    def test_duplicate_ports_and_missing_profiles_fail_closed(self):
        cases = []
        duplicate_port = self.config()
        duplicate_port["motors"]["arm"]["port"] = "outB"
        cases.append(duplicate_port)
        missing_profile = self.config()
        del missing_profile["motors"]["arm"]["limit_profile"]
        cases.append(missing_profile)
        invalid_heartbeat = self.config()
        invalid_heartbeat["limits"]["heartbeat"]["timeout_ms"] = True
        cases.append(invalid_heartbeat)

        for config in cases:
            temporary, path = self.write_config(config)
            try:
                with self.assertRaises(ValueError):
                    SafetyLimits.from_file(path)
            finally:
                temporary.cleanup()


if __name__ == "__main__":
    unittest.main()
