import ast
import contextlib
import hashlib
import io
import json
import os
import shutil
import stat
import subprocess
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import robot_agent.ev3_runtime_preflight as preflight_module
from robot_agent.ev3_runtime_preflight import (
    ALL_MANIFEST_PATHS,
    DEFAULT_COMMAND_TIMEOUT_SECONDS,
    EV3RuntimeDeploymentMismatchError,
    EV3RuntimePreflightConfigurationError,
    EV3RuntimePreflightProtocolError,
    EV3RuntimePreflightTransportError,
    MAX_COMMAND_TIMEOUT_SECONDS,
    MAX_OUTPUT_BYTES,
    NAVIGATION_WORKER_MANIFEST,
    PERIPHERAL_MANIFEST,
    REMOTE_PREFLIGHT_PROGRAM,
    REMOTE_PROJECT_ROOT,
    SUPERVISOR_ADDITIONS,
    SUPERVISOR_MANIFEST,
    run_ev3_runtime_preflight,
)
from robot_agent.ev3_runtime_preflight_cli import main
from robot_agent.peripheral_transport import (
    REMOTE_DAEMON as PERIPHERAL_REMOTE_DAEMON,
)
from robot_agent.supervisor_transport import (
    REMOTE_DAEMON as SUPERVISOR_REMOTE_DAEMON,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Completed:
    returncode: int = 0
    stdout: str = ""
    stderr: str = ""


class RecordingRunner:
    def __init__(self, completed):
        self.completed = completed
        self.calls = []

    def __call__(self, argv, **kwargs):
        self.calls.append((list(argv), dict(kwargs)))
        if isinstance(self.completed, BaseException):
            raise self.completed
        return self.completed


def matching_remote_result(root=PROJECT_ROOT):
    files = []
    for relative_path in ALL_MANIFEST_PATHS:
        content = (root / relative_path).read_bytes()
        files.append(
            {
                "path": relative_path,
                "status": "ok",
                "size": len(content),
                "sha256": hashlib.sha256(content).hexdigest(),
            }
        )
    return {
        "schema_version": 1,
        "effects": "read_only",
        "files": files,
    }


def completed_for(result=None):
    return Completed(
        stdout=json.dumps(
            matching_remote_result() if result is None else result,
            separators=(",", ":"),
        )
    )


class EV3RuntimePreflightTests(unittest.TestCase):
    def test_fixed_profiles_contain_complete_dependencies_once(self):
        self.assertEqual(
            PERIPHERAL_MANIFEST,
            (
                "ev3/peripheral_daemon.py",
                "ev3/peripheral_protocol.py",
                "ev3/robot_hal.py",
                "ev3/robot_config.py",
                "ev3/emergency_stop.py",
                "config/ev3rstorm.json",
            ),
        )
        self.assertEqual(
            SUPERVISOR_ADDITIONS,
            (
                "ev3/supervisor_daemon.py",
                "ev3/supervisor_protocol.py",
                "ev3/supervisor_support.py",
                "ev3/supervisor.py",
                "ev3/infrared_safety.py",
                "ev3/supervisor_cli.py",
            ),
        )
        self.assertEqual(
            SUPERVISOR_MANIFEST,
            PERIPHERAL_MANIFEST + SUPERVISOR_ADDITIONS,
        )
        self.assertEqual(
            len(SUPERVISOR_MANIFEST),
            len(set(SUPERVISOR_MANIFEST)),
        )
        self.assertEqual(
            NAVIGATION_WORKER_MANIFEST,
            (
                "ev3/navigation_worker_cli.py",
                "ev3/audio_playback_cli.py",
                "ev3/audio_playback_worker_cli.py",
                "ev3/speech_worker_cli.py",
                "ev3/robot_cli.py",
                "ev3/navigation_worker.py",
                "ev3/navigation_worker_protocol.py",
                "ev3/navigation_profile.py",
                "ev3/encoder_recovery.py",
                "ev3/encoder_recovery_runtime.py",
                "ev3/infrared_safety.py",
                "ev3/supervisor_support.py",
                "ev3/supervisor.py",
                "ev3/robot_hal.py",
                "ev3/robot_config.py",
                "ev3/emergency_stop.py",
                "config/ev3rstorm.json",
            ),
        )
        self.assertEqual(
            len(NAVIGATION_WORKER_MANIFEST),
            len(set(NAVIGATION_WORKER_MANIFEST)),
        )
        self.assertNotIn("ev3/robot_cli.py", PERIPHERAL_MANIFEST)
        self.assertNotIn("ev3/robot_cli.py", SUPERVISOR_MANIFEST)

    def test_transport_daemons_are_in_corresponding_manifests(self):
        peripheral_remote_paths = {
            REMOTE_PROJECT_ROOT + "/" + path
            for path in PERIPHERAL_MANIFEST
        }
        supervisor_remote_paths = {
            REMOTE_PROJECT_ROOT + "/" + path
            for path in SUPERVISOR_MANIFEST
        }
        self.assertIn(
            PERIPHERAL_REMOTE_DAEMON,
            peripheral_remote_paths,
        )
        self.assertIn(
            SUPERVISOR_REMOTE_DAEMON,
            supervisor_remote_paths,
        )

    def test_remote_program_is_python35_read_only_and_fixed(self):
        ast.parse(REMOTE_PREFLIGHT_PROGRAM, feature_version=5)
        self.assertIn(
            json.dumps(
                list(ALL_MANIFEST_PATHS),
                separators=(",", ":"),
            ),
            REMOTE_PREFLIGHT_PROGRAM,
        )
        for forbidden in (
            "subprocess",
            "os.O_WRONLY",
            "os.O_RDWR",
            "os.remove",
            "os.rename",
            "os.unlink",
            "__import__",
            "exec(",
            "motor",
            "run-timed",
            "speak-stdin",
        ):
            self.assertNotIn(forbidden, REMOTE_PREFLIGHT_PROGRAM)
        self.assertIn('"ev3/robot_cli.py"', REMOTE_PREFLIGHT_PROGRAM)
        self.assertIn("compile(", REMOTE_PREFLIGHT_PROGRAM)

    def test_happy_profiles_use_fixed_strict_ssh_command(self):
        for profile, expected_count in (
            ("peripheral", len(PERIPHERAL_MANIFEST)),
            ("supervisor", len(SUPERVISOR_MANIFEST)),
            (
                "navigation-worker",
                len(NAVIGATION_WORKER_MANIFEST),
            ),
        ):
            with self.subTest(profile=profile):
                runner = RecordingRunner(completed_for())

                result = run_ev3_runtime_preflight(
                    "robot@ev3dev.local",
                    profile=profile,
                    local_root=PROJECT_ROOT,
                    runner=runner,
                )

                self.assertEqual(result["status"], "ready")
                self.assertEqual(result["effects"], "read_only")
                self.assertEqual(result["profile"], profile)
                self.assertEqual(result["file_count"], expected_count)
                self.assertNotIn(
                    "ev3dev.local",
                    json.dumps(result),
                )
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
                        "robot@ev3dev.local",
                        "python3",
                        "-",
                    ],
                )
                self.assertEqual(
                    kwargs["input"],
                    REMOTE_PREFLIGHT_PROGRAM,
                )
                self.assertTrue(kwargs["text"])
                self.assertFalse(kwargs["check"])
                self.assertEqual(
                    kwargs["timeout"],
                    DEFAULT_COMMAND_TIMEOUT_SECONDS,
                )

    def test_missing_remote_file_fails_closed(self):
        remote = matching_remote_result()
        remote["files"][0] = {
            "path": ALL_MANIFEST_PATHS[0],
            "status": "missing",
            "size": None,
            "sha256": None,
        }

        with self.assertRaises(
            EV3RuntimeDeploymentMismatchError
        ) as raised:
            run_ev3_runtime_preflight(
                "robot@ev3dev.local",
                local_root=PROJECT_ROOT,
                runner=RecordingRunner(completed_for(remote)),
            )

        self.assertEqual(raised.exception.code, "remote_missing")

    def test_stale_remote_hash_fails_closed(self):
        remote = matching_remote_result()
        remote["files"][0]["sha256"] = "0" * 64

        with self.assertRaises(
            EV3RuntimeDeploymentMismatchError
        ) as raised:
            run_ev3_runtime_preflight(
                "robot@ev3dev.local",
                local_root=PROJECT_ROOT,
                runner=RecordingRunner(completed_for(remote)),
            )

        self.assertEqual(raised.exception.code, "hash_mismatch")

    def test_symlinked_remote_file_fails_closed(self):
        remote = matching_remote_result()
        remote["files"][0] = {
            "path": ALL_MANIFEST_PATHS[0],
            "status": "symlink",
            "size": None,
            "sha256": None,
        }

        with self.assertRaises(
            EV3RuntimeDeploymentMismatchError
        ) as raised:
            run_ev3_runtime_preflight(
                "robot@ev3dev.local",
                local_root=PROJECT_ROOT,
                runner=RecordingRunner(completed_for(remote)),
            )

        self.assertEqual(raised.exception.code, "remote_symlink")

    def test_non_regular_and_oversized_remote_files_fail_closed(self):
        for remote_status in ("non_regular", "oversized"):
            with self.subTest(remote_status=remote_status):
                remote = matching_remote_result()
                remote["files"][0] = {
                    "path": ALL_MANIFEST_PATHS[0],
                    "status": remote_status,
                    "size": None,
                    "sha256": None,
                }

                with self.assertRaises(
                    EV3RuntimeDeploymentMismatchError
                ) as raised:
                    run_ev3_runtime_preflight(
                        "robot@ev3dev.local",
                        local_root=PROJECT_ROOT,
                        runner=RecordingRunner(
                            completed_for(remote)
                        ),
                    )

                self.assertEqual(
                    raised.exception.code,
                    "remote_{}".format(remote_status),
                )

    def test_remote_unsafe_ancestor_fails_closed(self):
        remote = matching_remote_result()
        remote["files"][0] = {
            "path": ALL_MANIFEST_PATHS[0],
            "status": "unsafe_ancestor",
            "size": None,
            "sha256": None,
        }

        with self.assertRaises(
            EV3RuntimeDeploymentMismatchError
        ) as raised:
            run_ev3_runtime_preflight(
                "robot@ev3dev.local",
                local_root=PROJECT_ROOT,
                runner=RecordingRunner(completed_for(remote)),
            )

        self.assertEqual(
            raised.exception.code,
            "remote_unsafe_ancestor",
        )

    def test_async_generator_is_rejected_by_ev3_compile_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copytree(PROJECT_ROOT / "ev3", root / "ev3")
            shutil.copytree(
                PROJECT_ROOT / "config",
                root / "config",
            )
            incompatible_path = "ev3/peripheral_daemon.py"
            (root / incompatible_path).write_text(
                "async def observations():\n"
                "    yield 1\n",
                encoding="utf-8",
            )
            remote = matching_remote_result(root)
            remote["files"][0] = {
                "path": incompatible_path,
                "status": "python_incompatible",
                "size": None,
                "sha256": None,
            }
            runner = RecordingRunner(completed_for(remote))

            with self.assertRaises(
                EV3RuntimeDeploymentMismatchError
            ) as raised:
                run_ev3_runtime_preflight(
                    "robot@ev3dev.local",
                    local_root=root,
                    runner=runner,
                )

            self.assertEqual(
                raised.exception.code,
                "remote_python_incompatible",
            )
            self.assertEqual(len(runner.calls), 1)

    def test_python35_incompatible_selected_local_file_stops_before_ssh(
        self,
    ):
        for profile, relative_path in (
            ("peripheral", "ev3/peripheral_daemon.py"),
            ("supervisor", "ev3/supervisor_daemon.py"),
            (
                "navigation-worker",
                "ev3/robot_cli.py",
            ),
        ):
            with self.subTest(profile=profile):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    shutil.copytree(
                        PROJECT_ROOT / "ev3",
                        root / "ev3",
                    )
                    shutil.copytree(
                        PROJECT_ROOT / "config",
                        root / "config",
                    )
                    (root / relative_path).write_text(
                        "message = f'incompatible {1}'\n",
                        encoding="utf-8",
                    )
                    runner = RecordingRunner(completed_for())

                    with self.assertRaises(
                        EV3RuntimePreflightConfigurationError
                    ) as raised:
                        run_ev3_runtime_preflight(
                            "robot@ev3dev.local",
                            profile=profile,
                            local_root=root,
                            runner=runner,
                        )

                    self.assertIn(
                        "Python 3.5 compatible",
                        str(raised.exception),
                    )
                    self.assertEqual(runner.calls, [])

    def test_navigation_tts_companion_remote_python35_failure_is_fatal(
        self,
    ):
        remote = matching_remote_result()
        for index, entry in enumerate(remote["files"]):
            if entry["path"] == "ev3/robot_cli.py":
                remote["files"][index] = {
                    "path": "ev3/robot_cli.py",
                    "status": "python_incompatible",
                    "size": None,
                    "sha256": None,
                }
                break
        else:
            self.fail("robot_cli.py is absent from the fixed manifest")

        with self.assertRaises(
            EV3RuntimeDeploymentMismatchError
        ) as raised:
            run_ev3_runtime_preflight(
                "robot@ev3dev.local",
                profile="navigation-worker",
                local_root=PROJECT_ROOT,
                runner=RecordingRunner(completed_for(remote)),
            )

        self.assertEqual(
            raised.exception.code,
            "remote_python_incompatible",
        )

    def test_malformed_selected_local_config_stops_before_ssh(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copytree(PROJECT_ROOT / "ev3", root / "ev3")
            shutil.copytree(
                PROJECT_ROOT / "config",
                root / "config",
            )
            (root / "config" / "ev3rstorm.json").write_text(
                '{"duplicate":1,"duplicate":2}\n',
                encoding="utf-8",
            )
            runner = RecordingRunner(completed_for())

            with self.assertRaises(
                EV3RuntimePreflightConfigurationError
            ) as raised:
                run_ev3_runtime_preflight(
                    "robot@ev3dev.local",
                    local_root=root,
                    runner=runner,
                )

            self.assertIn("strict JSON", str(raised.exception))
            self.assertEqual(runner.calls, [])

    def test_symlinked_local_manifest_ancestor_stops_before_ssh(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            os.symlink(
                str(PROJECT_ROOT / "ev3"),
                str(root / "ev3"),
                target_is_directory=True,
            )
            shutil.copytree(
                PROJECT_ROOT / "config",
                root / "config",
            )
            runner = RecordingRunner(completed_for())

            with self.assertRaises(
                EV3RuntimePreflightConfigurationError
            ) as raised:
                run_ev3_runtime_preflight(
                    "robot@ev3dev.local",
                    local_root=root,
                    runner=runner,
                )

            self.assertIn("ancestor is unsafe", str(raised.exception))
            self.assertEqual(runner.calls, [])

    def test_local_file_path_is_re_lstat_after_hashing(self):
        target = str(PROJECT_ROOT / PERIPHERAL_MANIFEST[0])
        real_lstat = os.lstat
        target_calls = []

        def replaced_after_hash(path):
            evidence = real_lstat(path)
            if str(path) == target:
                target_calls.append(str(path))
                if len(target_calls) == 2:
                    return SimpleNamespace(
                        st_mode=stat.S_IFLNK | 0o777,
                        st_dev=evidence.st_dev,
                        st_ino=evidence.st_ino,
                        st_size=evidence.st_size,
                    )
            return evidence

        runner = RecordingRunner(completed_for())
        with patch.object(
            preflight_module.os,
            "lstat",
            side_effect=replaced_after_hash,
        ):
            with self.assertRaises(
                EV3RuntimePreflightConfigurationError
            ):
                run_ev3_runtime_preflight(
                    "robot@ev3dev.local",
                    local_root=PROJECT_ROOT,
                    runner=runner,
                )

        self.assertEqual(len(target_calls), 2)
        self.assertEqual(runner.calls, [])

    def test_malformed_and_oversized_replies_are_rejected(self):
        invalid_status_type = matching_remote_result()
        invalid_status_type["files"][0]["status"] = []
        malformed_replies = (
            "{",
            json.dumps(
                {
                    "schema_version": 1,
                    "effects": "read_only",
                    "files": [],
                }
            ),
            json.dumps(
                dict(
                    matching_remote_result(),
                    identity={"hostname": "should-not-be-accepted"},
                )
            ),
            json.dumps(invalid_status_type),
            "x" * (MAX_OUTPUT_BYTES + 1),
        )
        for reply in malformed_replies:
            with self.subTest(reply_size=len(reply)):
                with self.assertRaises(
                    EV3RuntimePreflightProtocolError
                ):
                    run_ev3_runtime_preflight(
                        "robot@ev3dev.local",
                        local_root=PROJECT_ROOT,
                        runner=RecordingRunner(
                            Completed(stdout=reply)
                        ),
                    )

    def test_target_and_timeout_injection_are_rejected_before_ssh(self):
        runner = RecordingRunner(completed_for())
        invalid_calls = (
            {
                "target": "robot@ev3dev.local;touch /tmp/pwned",
            },
            {
                "target": "robot@ev3dev.local",
                "profile": [],
            },
            {
                "target": "robot@ev3dev.local",
                "command_timeout_seconds": 0,
            },
            {
                "target": "robot@ev3dev.local",
                "command_timeout_seconds": (
                    MAX_COMMAND_TIMEOUT_SECONDS + 1
                ),
            },
            {
                "target": "robot@ev3dev.local",
                "connect_timeout_seconds": True,
            },
        )
        for kwargs in invalid_calls:
            with self.subTest(kwargs=kwargs):
                target = kwargs.pop("target")
                with self.assertRaises(
                    EV3RuntimePreflightConfigurationError
                ):
                    run_ev3_runtime_preflight(
                        target,
                        local_root=PROJECT_ROOT,
                        runner=runner,
                        **kwargs
                    )
        self.assertEqual(runner.calls, [])

    def test_whole_command_timeout_is_fail_closed(self):
        runner = RecordingRunner(
            subprocess.TimeoutExpired(["ssh"], timeout=7)
        )

        with self.assertRaises(
            EV3RuntimePreflightTransportError
        ):
            run_ev3_runtime_preflight(
                "robot@ev3dev.local",
                local_root=PROJECT_ROOT,
                runner=runner,
                command_timeout_seconds=7,
            )

        self.assertEqual(runner.calls[0][1]["timeout"], 7)

    def test_runner_unicode_decode_failure_is_typed(self):
        runner = RecordingRunner(
            UnicodeDecodeError(
                "utf-8",
                b"\xff",
                0,
                1,
                "invalid start byte",
            )
        )

        with self.assertRaises(
            EV3RuntimePreflightProtocolError
        ):
            run_ev3_runtime_preflight(
                "robot@ev3dev.local",
                local_root=PROJECT_ROOT,
                runner=runner,
            )

    def test_cli_emits_bounded_identifier_free_failure(self):
        error = EV3RuntimeDeploymentMismatchError(
            "remote_missing",
            "Remote deployment file is missing: "
            "ev3/peripheral_daemon.py",
        )
        stderr = io.StringIO()
        with patch(
            "robot_agent.ev3_runtime_preflight_cli."
            "run_ev3_runtime_preflight",
            side_effect=error,
        ) as preflight:
            with contextlib.redirect_stderr(stderr):
                exit_code = main(
                    [
                        "--ssh-target",
                        "robot@ev3dev.local",
                        "--profile",
                        "peripheral",
                    ]
                )

        self.assertEqual(exit_code, 1)
        payload = json.loads(stderr.getvalue())
        self.assertEqual(payload["error_code"], "remote_missing")
        self.assertEqual(payload["effects"], "read_only")
        self.assertNotIn("ev3dev.local", stderr.getvalue())
        preflight.assert_called_once_with(
            "robot@ev3dev.local",
            profile="peripheral",
            local_root=".",
            command_timeout_seconds=(
                DEFAULT_COMMAND_TIMEOUT_SECONDS
            ),
        )


if __name__ == "__main__":
    unittest.main()
