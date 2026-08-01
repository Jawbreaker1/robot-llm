import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
START_SCRIPT = PROJECT_ROOT / "scripts" / "start_lab_console.sh"


class StartLabConsoleScriptTests(unittest.TestCase):
    def _captured_launch(self, extra_environment=None, *arguments):
        with tempfile.TemporaryDirectory() as directory:
            fake_python = Path(directory) / "fake-python"
            fake_python.write_text(
                "#!/bin/sh\n"
                "printf 'PYTHONPATH=%s\\n' \"$PYTHONPATH\"\n"
                "for argument do\n"
                "    printf 'ARG=%s\\n' \"$argument\"\n"
                "done\n",
                encoding="utf-8",
            )
            fake_python.chmod(
                stat.S_IRUSR
                | stat.S_IWUSR
                | stat.S_IXUSR
            )
            environment = os.environ.copy()
            environment["ROBOT_LLM_PYTHON"] = str(fake_python)
            environment.pop("ROBOT_LLM_STT_URL", None)
            environment.pop("ROBOT_LLM_STT_INFERENCE_PATH", None)
            environment.pop("ROBOT_LLM_STT_MODEL_ID", None)
            if extra_environment:
                environment.update(extra_environment)

            completed = subprocess.run(
                [str(START_SCRIPT), *arguments],
                cwd=directory,
                env=environment,
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        lines = completed.stdout.splitlines()
        python_path = lines[0].removeprefix("PYTHONPATH=")
        launch_arguments = [
            line.removeprefix("ARG=")
            for line in lines[1:]
        ]
        return python_path, launch_arguments

    def test_default_profile_reuses_large_v3_turbo_service(self):
        python_path, arguments = self._captured_launch(
            None,
            "--simulation-map-demo",
        )

        self.assertEqual(
            python_path.split(os.pathsep)[0],
            str(PROJECT_ROOT / "src"),
        )
        self.assertEqual(
            arguments,
            [
                "-m",
                "robot_agent.dashboard_cli",
                "--stt-url",
                "http://127.0.0.1:8178/v1",
                "--stt-inference-path",
                "/audio/transcriptions",
                "--stt-model-id",
                "ggml-large-v3-turbo-q5_0",
                "--console-access-key-file",
                "~/.robot-llm/dashboard-access-key",
                "--simulation-map-demo",
            ],
        )

    def test_profile_values_are_portably_overridable(self):
        _python_path, arguments = self._captured_launch(
            {
                "ROBOT_LLM_STT_URL": "http://127.0.0.1:9180/v1",
                "ROBOT_LLM_STT_INFERENCE_PATH": "/audio/transcribe",
                "ROBOT_LLM_STT_MODEL_ID": "alternate-model",
            },
            "--port",
            "8877",
        )

        self.assertEqual(
            arguments,
            [
                "-m",
                "robot_agent.dashboard_cli",
                "--stt-url",
                "http://127.0.0.1:9180/v1",
                "--stt-inference-path",
                "/audio/transcribe",
                "--stt-model-id",
                "alternate-model",
                "--console-access-key-file",
                "~/.robot-llm/dashboard-access-key",
                "--port",
                "8877",
            ],
        )

    def test_empty_service_url_allows_explicit_managed_fallback(self):
        _python_path, arguments = self._captured_launch(
            {"ROBOT_LLM_STT_URL": ""},
            "--stt-model",
            "models/ggml-large-v3-turbo-q5_0.bin",
        )

        self.assertEqual(
            arguments,
            [
                "-m",
                "robot_agent.dashboard_cli",
                "--console-access-key-file",
                "~/.robot-llm/dashboard-access-key",
                "--stt-model",
                "models/ggml-large-v3-turbo-q5_0.bin",
            ],
        )

    def test_console_access_key_path_is_portably_overridable(self):
        _python_path, arguments = self._captured_launch(
            {
                "ROBOT_LLM_STT_URL": "",
                "ROBOT_LLM_CONSOLE_ACCESS_KEY_FILE": "/tmp/test-console-key",
            },
        )

        self.assertEqual(
            arguments,
            [
                "-m",
                "robot_agent.dashboard_cli",
                "--console-access-key-file",
                "/tmp/test-console-key",
            ],
        )


if __name__ == "__main__":
    unittest.main()
