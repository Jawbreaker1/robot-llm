import fcntl
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import ev3.supervisor as supervisor_module
from ev3.robot_hal import RobotHAL
from ev3.supervisor_cli import run_preflight


PROJECT_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "ev3rstorm.json"


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(value), encoding="ascii")


class ListAudit:
    def __init__(self):
        self.events = []

    def append(self, event):
        self.events.append(event)


class SupervisorCLITests(unittest.TestCase):
    def test_direct_help_is_python_entrypoint_safe(self):
        completed = subprocess.run(
            [
                sys.executable,
                str(PROJECT_ROOT / "ev3" / "supervisor_cli.py"),
                "--help",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("motion-free", completed.stdout.lower())

    def test_preflight_starts_no_motor_and_releases_lock(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "class"
            motors = [
                ("motor0", "outA", "lego-ev3-m-motor"),
                ("motor1", "outC", "lego-ev3-l-motor"),
                ("motor2", "outB", "lego-ev3-l-motor"),
            ]
            for name, port, driver in motors:
                path = root / "tacho-motor" / name
                for filename, value in {
                    "address": "ev3-ports:{}".format(port),
                    "driver_name": driver,
                    "position": 0,
                    "state": "",
                    "max_speed": 1050,
                    "speed_sp": 0,
                    "time_sp": 0,
                    "stop_action": "coast",
                    "command": "",
                }.items():
                    write(path / filename, value)

            sensors = [
                (
                    "sensor0",
                    "in1",
                    "lego-ev3-touch",
                    "TOUCH",
                    0,
                    "",
                ),
                (
                    "sensor1",
                    "in4",
                    "lego-ev3-ir",
                    "IR-PROX",
                    50,
                    "pct",
                ),
                (
                    "sensor2",
                    "in3",
                    "lego-ev3-color",
                    "COL-REFLECT",
                    4,
                    "pct",
                ),
            ]
            for name, port, driver, mode, value, units in sensors:
                path = root / "lego-sensor" / name
                for filename, item in {
                    "address": "ev3-ports:{}".format(port),
                    "driver_name": driver,
                    "mode": mode,
                    "value0": value,
                    "units": units,
                }.items():
                    write(path / filename, item)

            lock_path = str(Path(temp) / "motor.lock")
            robot = RobotHAL(
                str(CONFIG_PATH),
                sysfs_root=str(root),
                lock_path=lock_path,
                sleep_fn=lambda _seconds: None,
                monotonic_fn=lambda: 10.0,
            )
            audit = ListAudit()
            writes = []
            real_write = supervisor_module.write_text

            def record_write(path, value):
                writes.append((path, value))
                real_write(path, value)

            with patch.object(
                supervisor_module,
                "write_text",
                side_effect=record_write,
            ), patch(
                "ev3.supervisor_cli.time.sleep",
                return_value=None,
            ):
                result = run_preflight(robot, audit)

            self.assertEqual(result["status"], "completed")
            self.assertEqual(result["motor_start_commands"], 0)
            self.assertEqual(result["shutdown"]["state"], "CLOSED")
            self.assertTrue(result["shutdown"]["audit_complete"])
            self.assertFalse(result["supervisor"]["motion_allowed"])
            self.assertFalse(
                any(value == "run-timed" for _, value in writes)
            )
            self.assertTrue(
                all(
                    value == "stop"
                    for path, value in writes
                    if path.endswith("/command")
                )
            )
            self.assertIn(
                "supervisor_closed",
                [event["event"] for event in audit.events],
            )

            real_stop_verified = (
                supervisor_module.SupervisorMotorOwner.stop_all_verified
            )
            interrupted = {"value": False}

            def interrupt_startup_once(owner):
                if not interrupted["value"]:
                    interrupted["value"] = True
                    raise KeyboardInterrupt()
                return real_stop_verified(owner)

            with patch.object(
                supervisor_module.SupervisorMotorOwner,
                "stop_all_verified",
                new=interrupt_startup_once,
            ):
                with self.assertRaises(KeyboardInterrupt):
                    run_preflight(robot, ListAudit())

            lock_handle = open(lock_path, "a+")
            try:
                fcntl.flock(
                    lock_handle.fileno(),
                    fcntl.LOCK_EX | fcntl.LOCK_NB,
                )
            finally:
                fcntl.flock(
                    lock_handle.fileno(),
                    fcntl.LOCK_UN,
                )
                lock_handle.close()

    def test_preflight_cannot_succeed_when_shutdown_is_not_closed(self):
        class CloseFailureSupervisor:
            limits = {
                "poll_interval_ms": 20,
                "touch_release_samples": 3,
            }

            def __init__(self, _robot):
                self.events = []

            def poll_once(self):
                return self.status()

            def status(self):
                return {
                    "state": "DISARMED",
                    "fault": None,
                    "motion_allowed": False,
                    "touch": 0,
                }

            def close(self):
                return {
                    "state": "FAULT_LATCHED",
                    "fault": {
                        "code": "shutdown_stop_failed",
                    },
                }

            def drain_audit_events(self):
                return list(self.events)

        with patch(
            "ev3.supervisor_cli.EV3Supervisor",
            CloseFailureSupervisor,
        ), patch(
            "ev3.supervisor_cli.time.sleep",
            return_value=None,
        ):
            result = run_preflight(object(), ListAudit())

        self.assertEqual(result["status"], "failed")
        self.assertEqual(
            result["shutdown"]["state"],
            "FAULT_LATCHED",
        )

    def test_preflight_requires_stable_touch_release_samples(self):
        class UnstableTouchSupervisor:
            limits = {
                "poll_interval_ms": 20,
                "touch_release_samples": 3,
            }

            def __init__(self, _robot):
                pass

            def poll_once(self):
                return self.status()

            def status(self):
                return {
                    "state": "DISARMED",
                    "fault": None,
                    "motion_allowed": False,
                    "touch": 0,
                    "touch_released_samples": 1,
                }

            def close(self):
                return {
                    "state": "CLOSED",
                    "audit_complete": True,
                }

            def drain_audit_events(self):
                return []

        with patch(
            "ev3.supervisor_cli.EV3Supervisor",
            UnstableTouchSupervisor,
        ), patch(
            "ev3.supervisor_cli.time.sleep",
            return_value=None,
        ):
            result = run_preflight(object(), ListAudit())

        self.assertEqual(result["status"], "failed")


if __name__ == "__main__":
    unittest.main()
