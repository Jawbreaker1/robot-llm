import contextlib
import io
import json
import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import ev3.robot_cli as robot_cli
from ev3.robot_hal import SafetyError


PROJECT_ROOT = Path(__file__).parents[1]


class RobotCLITests(unittest.TestCase):
    def run_stop(self, robot_factory):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with patch.object(
            robot_cli, "RobotHAL", robot_factory
        ), patch.object(
            sys,
            "argv",
            ["robot_cli.py", "--config", "test-config.json", "stop"],
        ), contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(
            stderr
        ):
            exit_code = robot_cli.main()
        return exit_code, stdout.getvalue(), stderr.getvalue()

    def test_direct_help_is_python_entrypoint_safe(self):
        completed = subprocess.run(
            [
                sys.executable,
                "-W",
                "error",
                str(PROJECT_ROOT / "ev3" / "robot_cli.py"),
                "--help",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("motor-test", completed.stdout)
        self.assertIn("stop", completed.stdout)

    def test_stop_prints_confirmed_result_only_on_full_success(self):
        robot = MagicMock()
        robot.stop_all.return_value = {
            "status": "stopped",
            "configuration_valid": True,
            "hardware_stop_verified": True,
            "stop_confirmed": True,
            "errors": [],
        }
        factory = MagicMock(return_value=robot)

        exit_code, stdout, stderr = self.run_stop(factory)

        self.assertEqual(exit_code, 0)
        self.assertEqual(stderr, "")
        self.assertTrue(json.loads(stdout)["stop_confirmed"])
        robot.stop_all.assert_called_once_with()

    def test_partial_stop_failure_is_stderr_and_nonzero(self):
        robot = MagicMock()
        robot.stop_all.return_value = {
            "status": "failed",
            "configuration_valid": True,
            "hardware_stop_verified": False,
            "stop_confirmed": False,
            "errors": ["outB stop: injected failure"],
        }
        factory = MagicMock(return_value=robot)

        exit_code, stdout, stderr = self.run_stop(factory)

        self.assertEqual(exit_code, 1)
        self.assertEqual(stdout, "")
        result = json.loads(stderr)
        self.assertEqual(result["status"], "failed")
        self.assertFalse(result["stop_confirmed"])

    def test_corrupt_config_still_attempts_unconfigured_emergency_stop(self):
        result = {
            "status": "failed",
            "configuration_valid": False,
            "hardware_stop_verified": True,
            "stop_confirmed": False,
            "errors": ["configuration: duplicate key"],
        }
        factory = MagicMock(side_effect=SafetyError("duplicate key"))
        factory.emergency_stop_unconfigured.return_value = result

        exit_code, stdout, stderr = self.run_stop(factory)

        self.assertEqual(exit_code, 1)
        self.assertEqual(stdout, "")
        reported = json.loads(stderr)
        self.assertTrue(reported["hardware_stop_verified"])
        self.assertFalse(reported["configuration_valid"])
        self.assertFalse(reported["stop_confirmed"])
        factory.emergency_stop_unconfigured.assert_called_once_with(
            "duplicate key"
        )

    def test_recursion_error_still_attempts_unconfigured_emergency_stop(self):
        result = {
            "status": "failed",
            "configuration_valid": False,
            "hardware_stop_verified": True,
            "stop_confirmed": False,
            "errors": ["configuration: maximum recursion depth"],
        }
        factory = MagicMock(
            side_effect=RecursionError("maximum recursion depth")
        )
        factory.emergency_stop_unconfigured.return_value = result

        exit_code, stdout, stderr = self.run_stop(factory)

        self.assertEqual(exit_code, 1)
        self.assertEqual(stdout, "")
        self.assertFalse(json.loads(stderr)["stop_confirmed"])
        factory.emergency_stop_unconfigured.assert_called_once_with(
            "maximum recursion depth"
        )

    def test_corrupt_config_blocks_non_stop_command_without_fallback(self):
        factory = MagicMock(side_effect=SafetyError("duplicate key"))
        with patch.object(
            robot_cli, "RobotHAL", factory
        ), patch.object(
            sys,
            "argv",
            ["robot_cli.py", "--config", "bad.json", "inventory"],
        ):
            with self.assertRaises(SafetyError):
                robot_cli.main()

        factory.emergency_stop_unconfigured.assert_not_called()


if __name__ == "__main__":
    unittest.main()
