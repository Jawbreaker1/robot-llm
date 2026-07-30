import ast
import contextlib
import io
import json
import subprocess
import unittest
from dataclasses import dataclass
from unittest.mock import patch

from robot_agent.ev3_wifi_preflight import (
    DEFAULT_COMMAND_TIMEOUT_SECONDS,
    EV3WiFiPreflightConfigurationError,
    EV3WiFiPreflightProtocolError,
    EV3WiFiPreflightTransportError,
    MAX_COMMAND_TIMEOUT_SECONDS,
    REMOTE_PREFLIGHT_PROGRAM,
    run_ev3_wifi_preflight,
)
from robot_agent.ev3_wifi_preflight_cli import main


@dataclass
class Completed:
    returncode: int = 0
    stdout: str = ""
    stderr: str = ""


def valid_result():
    return {
        "schema_version": 1,
        "status": "observed",
        "effects": "read_only",
        "identity": {
            "hostname": "ev3dev",
            "machine_id": "brick-01",
        },
        "system": {
            "kernel_release": "4.14-test",
            "os_release": {"id": "debian"},
        },
        "usb_devices": [],
        "network": {
            "interfaces": [],
            "wireless_interfaces": [],
            "ath9k_htc_interfaces": [],
        },
        "ath9k_htc": {
            "module_loaded": False,
            "module_files": [],
            "firmware_candidates": [],
            "firmware_present": [],
        },
        "connman": {
            "available": True,
            "wifi_technology_present": True,
            "technologies": {
                "available": True,
                "returncode": 0,
                "output": "Type = wifi",
            },
            "services": {},
        },
        "onboarding_ready": False,
    }


def ready_result(driver_module="ath9k_htc"):
    result = valid_result()
    result["network"] = {
        "interfaces": [
            {
                "name": "wlan0",
                "wireless": True,
                "operstate": "down",
                "address": "00:11:22:33:44:55",
                "driver": "ath9k_hif_usb",
                "driver_module": driver_module,
            }
        ],
        "wireless_interfaces": ["wlan0"],
        "ath9k_htc_interfaces": (
            ["wlan0"] if driver_module == "ath9k_htc" else []
        ),
    }
    result["onboarding_ready"] = driver_module == "ath9k_htc"
    return result


class RecordingRunner:
    def __init__(self, completed):
        self.completed = completed
        self.calls = []

    def __call__(self, argv, **kwargs):
        self.calls.append((list(argv), dict(kwargs)))
        if isinstance(self.completed, BaseException):
            raise self.completed
        return self.completed


class EV3WiFiPreflightTests(unittest.TestCase):
    def test_remote_program_is_python35_and_read_only(self):
        ast.parse(REMOTE_PREFLIGHT_PROGRAM, feature_version=5)
        lowered = REMOTE_PREFLIGHT_PROGRAM.lower()
        for forbidden in (
            "connmanctl\", \"connect",
            "connmanctl\", \"enable",
            "connmanctl\", \"scan",
            "modprobe",
            "sudo",
        ):
            self.assertNotIn(forbidden, lowered)

    def test_preflight_uses_fixed_strict_multiplexed_ssh_command(self):
        runner = RecordingRunner(
            Completed(stdout=json.dumps(valid_result()))
        )

        result = run_ev3_wifi_preflight(
            "robot@fe80::1234%en9",
            runner=runner,
        )

        self.assertEqual(result["status"], "observed")
        argv, kwargs = runner.calls[0]
        self.assertEqual(
            argv,
            [
                "ssh",
                "-T",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=3",
                "-o",
                "StrictHostKeyChecking=yes",
                "-o",
                "ControlMaster=auto",
                "-o",
                "ControlPath=~/.ssh/robot-llm-%C",
                "-o",
                "ControlPersist=60",
                "robot@fe80::1234%en9",
                "python3",
                "-",
            ],
        )
        self.assertEqual(kwargs["input"], REMOTE_PREFLIGHT_PROGRAM)
        self.assertTrue(kwargs["text"])
        self.assertFalse(kwargs["check"])
        self.assertEqual(
            kwargs["timeout"],
            DEFAULT_COMMAND_TIMEOUT_SECONDS,
        )

    def test_cli_uses_bounded_default_and_forwards_override(self):
        for arguments, expected_timeout in (
            (
                ["--ssh-target", "robot@ev3dev.local"],
                DEFAULT_COMMAND_TIMEOUT_SECONDS,
            ),
            (
                [
                    "--ssh-target",
                    "robot@ev3dev.local",
                    "--command-timeout-seconds",
                    str(MAX_COMMAND_TIMEOUT_SECONDS),
                ],
                MAX_COMMAND_TIMEOUT_SECONDS,
            ),
        ):
            with self.subTest(arguments=arguments):
                output = io.StringIO()
                with patch(
                    "robot_agent.ev3_wifi_preflight_cli."
                    "run_ev3_wifi_preflight",
                    return_value=valid_result(),
                ) as preflight:
                    with contextlib.redirect_stdout(output):
                        status = main(arguments)

                self.assertEqual(status, 0)
                preflight.assert_called_once_with(
                    "robot@ev3dev.local",
                    command_timeout_seconds=expected_timeout,
                )
                self.assertEqual(
                    json.loads(output.getvalue())["status"],
                    "observed",
                )

    def test_ready_requires_wireless_interface_bound_to_ath9k_htc(self):
        ready = run_ev3_wifi_preflight(
            "robot@ev3dev.local",
            runner=RecordingRunner(
                Completed(stdout=json.dumps(ready_result()))
            ),
        )
        self.assertTrue(ready["onboarding_ready"])

        wrong_driver = run_ev3_wifi_preflight(
            "robot@ev3dev.local",
            runner=RecordingRunner(
                Completed(
                    stdout=json.dumps(
                        ready_result("different_wifi_module")
                    )
                )
            ),
        )
        self.assertFalse(wrong_driver["onboarding_ready"])

        inconsistent = ready_result("different_wifi_module")
        inconsistent["network"]["ath9k_htc_interfaces"] = [
            "wlan0"
        ]
        inconsistent["onboarding_ready"] = True
        with self.assertRaises(EV3WiFiPreflightProtocolError):
            run_ev3_wifi_preflight(
                "robot@ev3dev.local",
                runner=RecordingRunner(
                    Completed(stdout=json.dumps(inconsistent))
                ),
            )

    def test_target_and_timeouts_are_validated_before_ssh(self):
        runner = RecordingRunner(
            Completed(stdout=json.dumps(valid_result()))
        )
        for target in (
            "",
            "-oProxyCommand=bad",
            "robot@host;bad",
            " robot@host",
        ):
            with self.subTest(target=target):
                with self.assertRaises(
                    EV3WiFiPreflightConfigurationError
                ):
                    run_ev3_wifi_preflight(target, runner=runner)
        with self.assertRaises(EV3WiFiPreflightConfigurationError):
            run_ev3_wifi_preflight(
                "robot@ev3dev.local",
                runner=runner,
                command_timeout_seconds=0,
            )
        with self.assertRaises(EV3WiFiPreflightConfigurationError):
            run_ev3_wifi_preflight(
                "robot@ev3dev.local",
                runner=runner,
                command_timeout_seconds=(
                    MAX_COMMAND_TIMEOUT_SECONDS + 1
                ),
            )
        self.assertEqual(runner.calls, [])

    def test_timeout_and_nonzero_exit_are_wrapped(self):
        with self.assertRaisesRegex(
            EV3WiFiPreflightTransportError,
            "30-second command deadline",
        ):
            run_ev3_wifi_preflight(
                "robot@ev3dev.local",
                runner=RecordingRunner(
                    subprocess.TimeoutExpired(
                        "ssh",
                        DEFAULT_COMMAND_TIMEOUT_SECONDS,
                    )
                ),
            )
        with self.assertRaises(EV3WiFiPreflightTransportError):
            run_ev3_wifi_preflight(
                "robot@ev3dev.local",
                runner=RecordingRunner(
                    Completed(
                        returncode=255,
                        stderr="host key verification failed",
                    )
                ),
            )

    def test_invalid_or_oversized_schema_is_rejected(self):
        invalid_payloads = (
            "not json",
            "[]",
            json.dumps({"schema_version": 1}),
            json.dumps({
                **valid_result(),
                "onboarding_ready": "yes",
            }),
            json.dumps({
                **valid_result(),
                "schema_version": True,
            }),
            json.dumps({
                **valid_result(),
                "schema_version": 1.0,
            }),
            json.dumps({
                **valid_result(),
                "onboarding_ready": True,
            }),
            (
                '{"schema_version":1,"schema_version":1,'
                '"status":"observed"}'
            ),
            (
                '{"schema_version":1,"status":"observed",'
                '"effects":"read_only","identity":{},'
                '"network":{},"ath9k_htc":{},'
                '"connman":{"value":NaN},'
                '"onboarding_ready":false}'
            ),
            " " * (64 * 1024 + 1),
        )
        for payload in invalid_payloads:
            with self.subTest(payload_size=len(payload)):
                with self.assertRaises(EV3WiFiPreflightProtocolError):
                    run_ev3_wifi_preflight(
                        "robot@ev3dev.local",
                        runner=RecordingRunner(
                            Completed(stdout=payload)
                        ),
                    )


if __name__ == "__main__":
    unittest.main()
