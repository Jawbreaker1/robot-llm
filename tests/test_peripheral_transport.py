import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from robot_agent.peripheral_transport import (
    PeripheralRemoteError,
    PeripheralSSHChannelPoisonedError,
    PeripheralSSHConfigurationError,
    PeripheralSSHProtocolError,
    PeripheralSSHSession,
    PeripheralSSHTimeoutError,
    REMOTE_DAEMON,
    decode_peripheral_response,
)


PROJECT_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "ev3rstorm.json"
FAKE_DAEMON = (
    PROJECT_ROOT
    / "tests"
    / "fixtures"
    / "run_fake_peripheral_daemon.py"
)
CONTROLLER_ID = "ev3rstorm-01.ev3-main"


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(value), encoding="ascii")


def add_sensor(root, name, port, driver, mode, value, units=""):
    path = Path(root) / "lego-sensor" / name
    for filename, item in {
        "address": "ev3-ports:{}".format(port),
        "driver_name": driver,
        "mode": mode,
        "value0": value,
        "units": units,
    }.items():
        write(path / filename, item)
    return path


def response_wire(
    request_id="request-1",
    ok=True,
    result=None,
    error=None,
    **overrides
):
    value = {
        "schema": "ev3-peripheral-response/v1",
        "request_id": request_id,
        "controller_id": CONTROLLER_ID,
        "ok": ok,
    }
    if ok:
        value["result"] = {} if result is None else result
    else:
        value["error"] = (
            {"code": "failed", "message": "failed"}
            if error is None
            else error
        )
    value.update(overrides)
    return (
        json.dumps(value, separators=(",", ":")).encode("utf-8")
        + b"\n"
    )


class PeripheralResponseTests(unittest.TestCase):
    def test_strict_response_rejects_wrong_identity_duplicates_and_nan(self):
        valid_result = {"status": "closed"}
        invalid = (
            response_wire(
                operation="extra",
                result=valid_result,
            ),
            response_wire(
                request_id="wrong",
                result=valid_result,
            ),
            (
                b'{"schema":"ev3-peripheral-response/v1",'
                b'"request_id":"request-1","request_id":"again",'
                b'"controller_id":"ev3rstorm-01.ev3-main",'
                b'"ok":true,"result":{"status":"closed"}}\n'
            ),
            (
                b'{"schema":"ev3-peripheral-response/v1",'
                b'"request_id":"request-1",'
                b'"controller_id":"ev3rstorm-01.ev3-main",'
                b'"ok":true,"result":{"value":NaN}}\n'
            ),
            response_wire(
                result={
                    "protocol_version": True,
                    "robot_id": "ev3rstorm-01",
                    "controller_id": CONTROLLER_ID,
                    "peripheral_instance_id": "instance-1",
                    "motion_enabled": False,
                    "speech_enabled": False,
                    "capabilities": {
                        "configured_sensor_read": {
                            "enabled": True,
                            "roles": ["infrared"],
                        },
                    },
                },
            ),
        )
        for raw in invalid:
            with self.subTest(raw=raw[:80]):
                with self.assertRaises(
                    PeripheralSSHProtocolError
                ):
                    decode_peripheral_response(
                        raw,
                        "request-1",
                        CONTROLLER_ID,
                        "shutdown",
                        {},
                    )

    def test_correlated_remote_error_is_not_a_protocol_error(self):
        with self.assertRaises(PeripheralRemoteError) as context:
            decode_peripheral_response(
                response_wire(
                    ok=False,
                    error={
                        "code": "unknown_sensor_role",
                        "message": "Sensor is not configured",
                    },
                ),
                "request-1",
                CONTROLLER_ID,
                "read_sensor",
                {"role": "camera"},
            )
        self.assertEqual(
            context.exception.code,
            "unknown_sensor_role",
        )


class PeripheralSessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.sysfs_root = self.root / "class"
        add_sensor(
            self.sysfs_root,
            "sensor0",
            "in1",
            "lego-ev3-touch",
            "TOUCH",
            0,
        )
        self.infrared = add_sensor(
            self.sysfs_root,
            "sensor1",
            "in4",
            "lego-ev3-ir",
            "IR-PROX",
            52,
            "pct",
        )
        add_sensor(
            self.sysfs_root,
            "sensor2",
            "in3",
            "lego-ev3-color",
            "COL-REFLECT",
            8,
            "pct",
        )
        self.motor_traps = {}
        for filename, value in (
            ("command", "do-not-touch"),
            ("speed_sp", "unchanged-speed"),
            ("time_sp", "unchanged-time"),
            ("stop_action", "unchanged-stop"),
        ):
            path = (
                self.sysfs_root
                / "tacho-motor"
                / "motor0"
                / filename
            )
            write(path, value)
            self.motor_traps[path] = value
        self.processes = []
        self.remote_argvs = []

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=2)
        self.temp.cleanup()

    def process_factory(self, remote_argv, **kwargs):
        self.remote_argvs.append(list(remote_argv))
        environment = dict(os.environ)
        existing = environment.get("PYTHONPATH")
        environment["PYTHONPATH"] = (
            str(PROJECT_ROOT)
            if not existing
            else str(PROJECT_ROOT) + os.pathsep + existing
        )
        process = subprocess.Popen(
            [
                sys.executable,
                str(FAKE_DAEMON),
                "--config",
                str(CONFIG_PATH),
                "--sysfs-root",
                str(self.sysfs_root),
            ],
            env=environment,
            **kwargs,
        )
        self.processes.append(process)
        return process

    def session(self, **kwargs):
        return PeripheralSSHSession(
            "robot@fake.local",
            CONTROLLER_ID,
            process_factory=self.process_factory,
            response_timeout_seconds=1,
            startup_response_timeout_seconds=2,
            remote_session_ms=10000,
            remote_max_requests=32,
            **kwargs
        )

    def assert_motor_traps_untouched(self):
        for path, value in self.motor_traps.items():
            self.assertEqual(
                path.read_text(encoding="ascii"),
                value,
            )

    def test_one_process_serves_many_fresh_reads_and_closes_cleanly(self):
        session = self.session()
        try:
            description = session.describe()
            self.assertFalse(description["motion_enabled"])
            self.assertFalse(description["speech_enabled"])
            values = []
            for index in range(5):
                write(self.infrared / "value0", 20 + index)
                values.append(
                    session.read_sensor("infrared")["value0"]
                )
        finally:
            session.close()

        self.assertEqual(values, [20, 21, 22, 23, 24])
        self.assertEqual(len(self.processes), 1)
        self.assertEqual(len(self.remote_argvs), 1)
        self.assertEqual(
            self.remote_argvs[0],
            [
                "ssh",
                "-T",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=3",
                "-o",
                "StrictHostKeyChecking=yes",
                "robot@fake.local",
                "python3",
                REMOTE_DAEMON,
                "--max-session-ms",
                "10000",
                "--max-requests",
                "32",
            ],
        )
        self.assertEqual(session.lifecycle_state, "CLOSED")
        self.assert_motor_traps_untouched()

    def test_disallowed_operation_is_rejected_without_remote_write(self):
        session = self.session()
        try:
            with self.assertRaises(
                PeripheralSSHConfigurationError
            ):
                session.request("drive_timed")
            self.assertEqual(session.describe()["motion_enabled"], False)
        finally:
            session.close()
        self.assert_motor_traps_untouched()

    def test_sensor_read_requires_correlated_description_first(self):
        session = self.session()
        try:
            with self.assertRaises(
                PeripheralSSHConfigurationError
            ):
                session.read_sensor("infrared")
            session.describe()
            self.assertEqual(
                session.read_sensor("infrared")["role"],
                "infrared",
            )
        finally:
            session.close()

    def test_timeout_poisons_channel_before_any_later_request(self):
        processes = []

        def sleeping_factory(_argv, **kwargs):
            process = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    "import time; time.sleep(2)",
                ],
                **kwargs
            )
            processes.append(process)
            return process

        session = PeripheralSSHSession(
            "robot@fake.local",
            CONTROLLER_ID,
            process_factory=sleeping_factory,
            response_timeout_seconds=0.1,
            startup_response_timeout_seconds=0.1,
        )
        try:
            with self.assertRaises(PeripheralSSHTimeoutError):
                session.describe()
            with self.assertRaises(
                PeripheralSSHChannelPoisonedError
            ):
                session.read_sensor("infrared")
            self.assertEqual(session.lifecycle_state, "POISONED")
        finally:
            session.close()
            for process in processes:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=2)
        self.assert_motor_traps_untouched()

    def test_target_and_limits_are_validated_before_process_start(self):
        for target in (
            "",
            "-oProxyCommand=bad",
            "robot@host;bad",
            " robot@host",
        ):
            with self.subTest(target=target):
                with self.assertRaises(
                    PeripheralSSHConfigurationError
                ):
                    PeripheralSSHSession(
                        target,
                        CONTROLLER_ID,
                        process_factory=self.process_factory,
                    )
        self.assertEqual(self.processes, [])


if __name__ == "__main__":
    unittest.main()
