import unittest
from pathlib import Path

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


if __name__ == "__main__":
    unittest.main()
