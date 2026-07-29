import io
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from robot_agent.http_transport import DirectHTTPResponse
from robot_agent.stt_whisper_server import (
    WhisperCppServer,
    WhisperServerError,
)


PRIVATE_TOKEN = "a" * 48
PRIVATE_PREFIX = "/stt-" + PRIVATE_TOKEN


class ImmediateThread:
    def __init__(self, *, target, name, daemon):
        self.target = target
        self.name = name
        self.daemon = daemon
        self.started = False
        self.join_calls = []

    def start(self):
        self.started = True
        self.target()

    def join(self, timeout=None):
        self.join_calls.append(timeout)

    def is_alive(self):
        return False


class FakeProcess:
    def __init__(
        self,
        returncode=None,
        wait_times_out=False,
        never_dies=False,
        stderr=b"private provider detail\n",
    ):
        self.returncode = returncode
        self.wait_times_out = wait_times_out
        self.never_dies = never_dies
        self.stderr = io.BytesIO(stderr)
        self.terminated = False
        self.killed = False
        self.wait_calls = []

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True

    def wait(self, timeout):
        self.wait_calls.append(timeout)
        if self.never_dies:
            raise subprocess.TimeoutExpired("whisper-server", timeout)
        if self.wait_times_out and not self.killed:
            raise subprocess.TimeoutExpired("whisper-server", timeout)
        self.returncode = -9 if self.killed else 0
        return self.returncode

    def kill(self):
        self.killed = True


def health_response(
    status=200,
    body=b'{"status":"ok"}',
    server="whisper.cpp",
):
    return DirectHTTPResponse(
        status_code=status,
        headers=(("Server", server),),
        body=body,
    )


def stdout_capture(data=b""):
    child_fd = os.open(os.devnull, os.O_WRONLY)
    return io.BytesIO(data), child_fd


def listener_line(port):
    return (
        "\nwhisper server listening at "
        "http://127.0.0.1:{}\n\n".format(port)
    ).encode("ascii")


class WhisperServerTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.model = Path(self.temporary_directory.name) / (
            "ggml small multilingual.bin"
        )
        self.model.write_bytes(b"\x00" * 1_000_001)

    def make_server(self, **kwargs):
        with patch(
            "robot_agent.stt_whisper_server.shutil.which",
            return_value="/opt/local/bin/whisper-server",
        ):
            return WhisperCppServer(str(self.model), **kwargs)

    def start_patches(self, process, stdout):
        return (
            patch(
                "robot_agent.stt_whisper_server.subprocess.Popen",
                return_value=process,
            ),
            patch(
                "robot_agent.stt_whisper_server.threading.Thread",
                ImmediateThread,
            ),
            patch(
                "robot_agent.stt_whisper_server._open_stdout_capture",
                return_value=stdout_capture(stdout),
            ),
            patch(
                "robot_agent.stt_whisper_server.secrets.token_hex",
                return_value=PRIVATE_TOKEN,
            ),
        )

    def test_configuration_is_bounded_and_model_identity_is_ascii_safe(self):
        server = self.make_server(port=9001, threads=8)

        with self.assertRaisesRegex(
            WhisperServerError,
            "no active private endpoint",
        ):
            _ = server.base_url
        self.assertEqual(
            server.model_id,
            "ggml-small-multilingual",
        )

        unicode_model = Path(self.temporary_directory.name) / (
            "grön språkmodell.bin"
        )
        unicode_model.write_bytes(b"\x00" * 1_000_001)
        with patch(
            "robot_agent.stt_whisper_server.shutil.which",
            return_value="/opt/local/bin/whisper-server",
        ):
            unicode_server = WhisperCppServer(str(unicode_model))
        self.assertEqual(
            unicode_server.model_id,
            "gr-n-spr-kmodell",
        )
        self.assertTrue(unicode_server.model_id.isascii())

        missing = Path(self.temporary_directory.name) / "missing.bin"
        small = Path(self.temporary_directory.name) / "small.bin"
        small.write_bytes(b"\x00" * 100)
        cases = (
            {"model_path": str(missing)},
            {"model_path": str(small)},
            {"model_path": str(self.model), "port": True},
            {"model_path": str(self.model), "port": 0},
            {"model_path": str(self.model), "threads": 0},
            {"model_path": str(self.model), "use_gpu": "yes"},
            {
                "model_path": str(self.model),
                "startup_timeout_seconds": 0,
            },
            {
                "model_path": str(self.model),
                "startup_timeout_seconds": 121,
            },
            {
                "model_path": str(self.model),
                "startup_timeout_seconds": float("nan"),
            },
        )
        for values in cases:
            with self.subTest(values=values):
                with patch(
                    "robot_agent.stt_whisper_server.shutil.which",
                    return_value="/opt/local/bin/whisper-server",
                ):
                    with self.assertRaises(WhisperServerError):
                        WhisperCppServer(**values)

        with patch(
            "robot_agent.stt_whisper_server.shutil.which",
            return_value=None,
        ):
            with self.assertRaises(WhisperServerError):
                WhisperCppServer(str(self.model))

    def test_start_uses_private_prefix_own_stdout_and_exact_health(self):
        server = self.make_server(port=9002, threads=6)
        process = FakeProcess(
            stderr=(
                "diagnostic {}\n".format(PRIVATE_PREFIX)
            ).encode("ascii")
        )
        popen_calls = []

        def popen(argv, **kwargs):
            popen_calls.append((argv, kwargs))
            return process

        with (
            patch(
                "robot_agent.stt_whisper_server.subprocess.Popen",
                side_effect=popen,
            ),
            patch(
                "robot_agent.stt_whisper_server.threading.Thread",
                ImmediateThread,
            ),
            patch(
                "robot_agent.stt_whisper_server._open_stdout_capture",
                return_value=stdout_capture(listener_line(9002)),
            ),
            patch(
                "robot_agent.stt_whisper_server.secrets.token_hex",
                return_value=PRIVATE_TOKEN,
            ),
            patch(
                "robot_agent.stt_whisper_server.direct_http_request",
                return_value=health_response(),
            ) as request,
        ):
            server.start()

        self.assertEqual(len(popen_calls), 1)
        argv, kwargs = popen_calls[0]
        self.assertEqual(
            argv,
            [
                "/opt/local/bin/whisper-server",
                "--model",
                str(self.model.resolve()),
                "--host",
                "127.0.0.1",
                "--port",
                "9002",
                "--threads",
                "6",
                "--language",
                "auto",
                "--no-timestamps",
                "--request-path",
                PRIVATE_PREFIX,
            ],
        )
        self.assertIs(kwargs["stdin"], subprocess.DEVNULL)
        self.assertIsInstance(kwargs["stdout"], int)
        self.assertIs(kwargs["stderr"], subprocess.PIPE)
        self.assertTrue(kwargs["close_fds"])
        self.assertEqual(
            server.base_url,
            "http://127.0.0.1:9002" + PRIVATE_PREFIX,
        )
        request.assert_called_once_with(
            "GET",
            (
                "http://127.0.0.1:9002"
                + PRIVATE_PREFIX
                + "/health"
            ),
            {"Accept": "application/json"},
            None,
            0.25,
            4 * 1024,
        )
        self.assertNotIn(PRIVATE_PREFIX, "\n".join(server._stderr_tail))
        self.assertIn(
            "[private-path]",
            "\n".join(server._stderr_tail),
        )

        with self.assertRaisesRegex(
            WhisperServerError,
            "already started",
        ):
            server.start()

        stdout_thread = server._stdout_thread
        stderr_thread = server._stderr_thread
        server.stop()
        self.assertTrue(process.terminated)
        self.assertEqual(process.wait_calls, [2.0])
        self.assertEqual(stdout_thread.join_calls, [1.0])
        self.assertEqual(stderr_thread.join_calls, [1.0])
        self.assertIsNone(server._process)
        with self.assertRaises(WhisperServerError):
            _ = server.base_url

    def test_cpu_mode_passes_explicit_no_gpu_flag(self):
        server = self.make_server(port=9006, use_gpu=False)
        process = FakeProcess()
        patches = self.start_patches(
            process,
            listener_line(9006),
        )
        with (
            patches[0] as popen,
            patches[1],
            patches[2],
            patches[3],
            patch(
                "robot_agent.stt_whisper_server.direct_http_request",
                return_value=health_response(),
            ),
        ):
            server.start()

        command = popen.call_args.args[0]
        self.assertEqual(command[-1], "--no-gpu")
        server.stop()

    def test_old_server_health_never_substitutes_for_own_stdout(self):
        process = FakeProcess()
        server = self.make_server(
            port=9003,
            startup_timeout_seconds=1,
        )
        patches = self.start_patches(
            process,
            b"whisper server listening at http://127.0.0.1:9004\n",
        )
        with (
            patches[0],
            patches[1],
            patches[2],
            patches[3],
            patch(
                "robot_agent.stt_whisper_server.direct_http_request",
                return_value=health_response(),
            ) as request,
            patch(
                "robot_agent.stt_whisper_server.time.monotonic",
                side_effect=(0.0, 2.0),
            ),
        ):
            with self.assertRaisesRegex(
                WhisperServerError,
                "deadline",
            ):
                server.start()

        request.assert_not_called()
        self.assertTrue(process.terminated)
        self.assertIsNone(server._process)

    def test_preexisting_port_bind_collision_fails_closed(self):
        process = FakeProcess(
            returncode=1,
            stderr=(
                b"couldn't bind to server socket: "
                b"hostname=127.0.0.1 port=9005\n"
            ),
        )
        server = self.make_server(port=9005)
        patches = self.start_patches(process, b"")
        with (
            patches[0],
            patches[1],
            patches[2],
            patches[3],
            patch(
                "robot_agent.stt_whisper_server.direct_http_request",
                return_value=health_response(),
            ) as request,
        ):
            with self.assertRaisesRegex(
                WhisperServerError,
                "exited before becoming ready",
            ):
                server.start()

        request.assert_not_called()
        self.assertIsNone(server._process)

    def test_start_rejects_readiness_timeout_and_bad_prefix_factory(self):
        alive = FakeProcess()
        server = self.make_server(startup_timeout_seconds=1)
        patches = self.start_patches(alive, b"")
        with (
            patches[0],
            patches[1],
            patches[2],
            patches[3],
            patch(
                "robot_agent.stt_whisper_server.time.monotonic",
                side_effect=(0.0, 2.0),
            ),
        ):
            with self.assertRaisesRegex(
                WhisperServerError,
                "deadline",
            ):
                server.start()
        self.assertTrue(alive.terminated)
        self.assertIsNone(server._process)

        for invalid in ("short", "g" * 48, None):
            with self.subTest(invalid=invalid):
                server = self.make_server()
                with patch(
                    "robot_agent.stt_whisper_server.secrets.token_hex",
                    return_value=invalid,
                ):
                    with self.assertRaisesRegex(
                        WhisperServerError,
                        "prefix generation failed",
                    ):
                        server.start()
                self.assertIsNone(server._process)

    def test_start_requires_status_schema_server_header_and_live_child(self):
        invalid_responses = (
            health_response(status=503),
            health_response(body=b'{"status":"loading"}'),
            health_response(body=b'{"status":"ok","extra":true}'),
            health_response(server="not-whisper"),
            health_response(server="prefix whisper.cpp suffix"),
        )
        for invalid_response in invalid_responses:
            with self.subTest(response=invalid_response):
                process = FakeProcess()
                server = self.make_server(startup_timeout_seconds=1)
                patches = self.start_patches(
                    process,
                    listener_line(server._port),
                )
                with (
                    patches[0],
                    patches[1],
                    patches[2],
                    patches[3],
                    patch(
                        (
                            "robot_agent.stt_whisper_server."
                            "direct_http_request"
                        ),
                        return_value=invalid_response,
                    ),
                    patch(
                        "robot_agent.stt_whisper_server.time.monotonic",
                        side_effect=(0.0, 0.5, 2.0),
                    ),
                    patch(
                        "robot_agent.stt_whisper_server.time.sleep",
                    ),
                ):
                    with self.assertRaises(WhisperServerError):
                        server.start()
                self.assertTrue(process.terminated)

    def test_start_interrupt_after_spawn_still_stops_owned_child(self):
        process = FakeProcess()
        server = self.make_server()
        patches = self.start_patches(process, b"")
        with (
            patches[0],
            patches[1],
            patches[2],
            patches[3],
            patch.object(
                server,
                "_start_drainers",
                side_effect=KeyboardInterrupt,
            ),
        ):
            with self.assertRaises(KeyboardInterrupt):
                server.start()

        self.assertTrue(process.terminated)
        self.assertIsNone(server._process)

    def test_signal_during_spawn_is_deferred_until_child_is_owned(self):
        process = FakeProcess()
        server = self.make_server()
        patches = self.start_patches(process, b"")

        def interrupted_spawn(*_args, **_kwargs):
            signal.raise_signal(signal.SIGTERM)
            return process

        with (
            patch(
                "robot_agent.stt_whisper_server.subprocess.Popen",
                side_effect=interrupted_spawn,
            ),
            patches[1],
            patches[2],
            patches[3],
        ):
            with self.assertRaises(KeyboardInterrupt):
                server.start()

        self.assertTrue(process.terminated)
        self.assertIsNone(server._process)

    def test_stop_escalates_from_terminate_to_kill(self):
        server = self.make_server()
        process = FakeProcess(wait_times_out=True)
        server._process = process

        server.stop(timeout_seconds=0.01)

        self.assertTrue(process.terminated)
        self.assertTrue(process.killed)
        self.assertEqual(process.wait_calls, [0.01, 1.0])
        self.assertIsNone(server._process)
        server.stop()

    def test_stop_retains_process_when_death_cannot_be_proven(self):
        server = self.make_server()
        process = FakeProcess(never_dies=True)
        server._process = process

        with self.assertRaisesRegex(
            WhisperServerError,
            "did not stop",
        ):
            server.stop(timeout_seconds=0)

        self.assertTrue(process.terminated)
        self.assertTrue(process.killed)
        self.assertIs(server._process, process)

    def test_stop_rejects_invalid_timeout_without_mutating_process(self):
        invalid = (True, -1, 61, float("nan"), "1")
        for timeout in invalid:
            with self.subTest(timeout=timeout):
                server = self.make_server()
                process = FakeProcess()
                server._process = process
                with self.assertRaisesRegex(
                    WhisperServerError,
                    "timeout is invalid",
                ):
                    server.stop(timeout)
                self.assertIs(server._process, process)
                self.assertFalse(process.terminated)


if __name__ == "__main__":
    unittest.main()
