import io
import json
from pathlib import Path
import socket
import threading
import unittest
from unittest import mock

from http.server import ThreadingHTTPServer

from robot_agent.dashboard_cli import (
    _LoopbackThreadingHTTPServer,
    _handler_class,
    _parser,
    _raise_termination_interrupt,
    _run,
    main,
)
from robot_agent.dashboard_http import DashboardHTTPResponse
from robot_agent.robot_input_service import RobotInputService


class FakeSocket:
    def __init__(self):
        self.closed = False
        self.sent = []
        self.shutdown_calls = []
        self.timeout = None

    def sendall(self, data):
        self.sent.append(data)

    def shutdown(self, how):
        self.shutdown_calls.append(how)

    def close(self):
        self.closed = True

    def settimeout(self, timeout):
        self.timeout = timeout


class FakeHeaders:
    def __init__(self, values):
        self._values = list(values)

    def __iter__(self):
        return iter(name for name, _value in self._values)

    def get_all(self, name):
        return [
            value
            for candidate, value in self._values
            if candidate == name
        ]

    def get(self, name, default=None):
        for candidate, value in self._values:
            if candidate.lower() == name.lower():
                return value
        return default


class RejectingRouter:
    def __init__(self):
        self.preflight_calls = []
        self.handle_calls = []

    def preflight(self, method, path, headers):
        self.preflight_calls.append((method, path, headers))
        return DashboardHTTPResponse(
            status=403,
            headers=(("Content-Type", "application/json"),),
            body=b'{"error":{"code":"session_token_rejected"}}',
        )

    def handle(self, *args):
        self.handle_calls.append(args)
        raise AssertionError("rejected request reached router.handle")


class AcceptingRouter:
    def __init__(self, body_limit):
        self.body_limit = body_limit
        self.limit_calls = []
        self.handle_calls = []

    def preflight(self, _method, _path, _headers):
        return None

    def request_body_limit(self, method, path):
        self.limit_calls.append((method, path))
        return self.body_limit

    def handle(self, method, path, headers, body):
        self.handle_calls.append((method, path, headers, body))
        return DashboardHTTPResponse(
            status=202,
            headers=(("Content-Type", "application/json"),),
            body=b"{}",
        )


class DashboardCLITests(unittest.TestCase):
    def test_sigterm_handler_routes_through_keyboard_interrupt_cleanup(self):
        with self.assertRaises(KeyboardInterrupt):
            _raise_termination_interrupt(15, None)

    @staticmethod
    def bare_server(slots=1):
        server = object.__new__(_LoopbackThreadingHTTPServer)
        server._request_slots = threading.BoundedSemaphore(slots)
        server._read_timeout_seconds = 0.25
        return server

    def test_request_thread_count_is_bounded_before_thread_spawn(self):
        server = self.bare_server(slots=1)
        first = FakeSocket()
        overloaded = FakeSocket()

        with mock.patch.object(
            ThreadingHTTPServer,
            "process_request",
        ) as parent:
            server.process_request(first, ("127.0.0.1", 10001))
            server.process_request(overloaded, ("127.0.0.1", 10002))

        self.assertEqual(parent.call_count, 1)
        self.assertFalse(first.closed)
        self.assertTrue(overloaded.closed)
        self.assertEqual(
            overloaded.shutdown_calls,
            [socket.SHUT_RDWR],
        )
        self.assertIn(b"503 Service Unavailable", overloaded.sent[0])
        server._request_slots.release()

    def test_request_slot_is_released_when_worker_finishes(self):
        server = self.bare_server(slots=1)
        request = FakeSocket()
        self.assertTrue(server._request_slots.acquire(blocking=False))

        with mock.patch.object(
            ThreadingHTTPServer,
            "process_request_thread",
        ):
            server.process_request_thread(
                request,
                ("127.0.0.1", 10001),
            )

        self.assertTrue(server._request_slots.acquire(blocking=False))
        server._request_slots.release()

    def test_accepted_socket_receives_finite_read_timeout(self):
        server = self.bare_server()
        request = FakeSocket()
        with mock.patch.object(
            ThreadingHTTPServer,
            "get_request",
            return_value=(request, ("127.0.0.1", 10001)),
        ):
            returned, address = server.get_request()

        self.assertIs(returned, request)
        self.assertEqual(address, ("127.0.0.1", 10001))
        self.assertEqual(request.timeout, 0.25)

    def test_preflight_rejection_happens_before_request_body_read(self):
        router = RejectingRouter()
        handler_class = _handler_class(router)
        handler = object.__new__(handler_class)
        handler.command = "POST"
        handler.path = "/api/v1/conversations"
        handler.headers = FakeHeaders(
            (
                ("Host", "127.0.0.1:8765"),
                ("Content-Length", "16000"),
            )
        )
        sent = []
        handler._send = sent.append
        handler._request_body = mock.Mock(
            side_effect=AssertionError("body was read before authentication")
        )

        handler._dispatch()

        handler._request_body.assert_not_called()
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0].status, 403)
        self.assertEqual(len(router.preflight_calls), 1)
        self.assertEqual(router.handle_calls, [])

    def test_authenticated_route_selects_body_limit_before_read(self):
        router = AcceptingRouter(body_limit=800_000)
        handler_class = _handler_class(router)
        handler = object.__new__(handler_class)
        handler.command = "POST"
        handler.path = "/api/v1/stt/transcriptions"
        handler.headers = FakeHeaders(
            (
                ("Host", "127.0.0.1:8765"),
                ("Content-Length", "4"),
            )
        )
        handler.rfile = io.BytesIO(b"WAVE")
        sent = []
        handler._send = sent.append

        handler._dispatch()

        self.assertEqual(
            router.limit_calls,
            [("POST", "/api/v1/stt/transcriptions")],
        )
        self.assertEqual(router.handle_calls[0][3], b"WAVE")
        self.assertEqual(sent[0].status, 202)

    def test_access_log_is_quiet_and_never_copies_session_path(self):
        handler_class = _handler_class(RejectingRouter())
        handler = object.__new__(handler_class)
        secret_path = "GET /session/SECRET/assets/app.js HTTP/1.1"

        output = io.StringIO()
        with mock.patch("sys.stderr", output):
            handler.log_message('"%s" %s %s', secret_path, "200", "10")
            handler.log_message('"%s" %s %s', secret_path, "403", "10")

        logged = output.getvalue()
        self.assertEqual(logged, "[dashboard] HTTP error status 403\n")
        self.assertNotIn("SECRET", logged)

    def test_simulation_map_demo_is_explicit_opt_in(self):
        defaults = _parser().parse_args([])
        enabled = _parser().parse_args(["--simulation-map-demo"])

        self.assertFalse(defaults.simulation_map_demo)
        self.assertTrue(enabled.simulation_map_demo)
        self.assertIsNone(defaults.blast_hub_name)

    def test_run_wires_optional_blast_observer_into_dashboard(self):
        monitor = mock.Mock()
        dashboard_service = mock.Mock()
        control_service = mock.Mock()
        http_server = mock.Mock()
        router = mock.Mock(session_path="/live/token/")

        with (
            mock.patch(
                "robot_agent.dashboard_cli.BlastObservationMonitor",
                return_value=monitor,
            ) as monitor_type,
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=dashboard_service,
            ) as dashboard_type,
            mock.patch(
                "robot_agent.dashboard_cli.RobotControlService",
                return_value=control_service,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(http_server, router),
            ) as server_type,
            mock.patch("sys.stdout", new_callable=io.StringIO) as stdout,
        ):
            result = _run(["--blast-hub-name", "BLAST-TEST"])

        self.assertEqual(result, 0)
        monitor_type.assert_called_once_with(hub_name="BLAST-TEST")
        monitor.start.assert_called_once_with()
        self.assertEqual(
            dashboard_type.call_args.kwargs[
                "controller_runtime_providers"
            ],
            (monitor,),
        )
        self.assertEqual(
            server_type.call_args.kwargs[
                "controller_control_services"
            ],
            {"blast-01.hub": monitor},
        )
        monitor.close.assert_called_once_with()
        ready = json.loads(stdout.getvalue())
        self.assertTrue(ready["blast_observation_enabled"])

    def test_run_injects_robot_adapter_into_separate_control_service(self):
        adapter = mock.Mock()
        dashboard_service = mock.Mock()
        control_service = mock.Mock()
        http_server = mock.Mock()
        router = mock.Mock()
        router.session_path = "/live/token/"

        with (
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=dashboard_service,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.RobotControlService",
                return_value=control_service,
            ) as control_factory,
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(http_server, router),
            ) as server_factory,
            mock.patch("sys.stdout", new_callable=io.StringIO) as stdout,
        ):
            result = _run([], robot_runtime_adapter=adapter)

        self.assertEqual(result, 0)
        self.assertIs(
            control_factory.call_args.args[0],
            adapter,
        )
        self.assertEqual(
            control_factory.call_args.kwargs["settings"].model,
            _parser().parse_args([]).model,
        )
        server_factory.assert_called_once()
        server_args = server_factory.call_args
        self.assertEqual(server_args.args, (dashboard_service, 8765))
        self.assertIs(
            server_args.kwargs["robot_control_service"],
            control_service,
        )
        self.assertIsInstance(
            server_args.kwargs["robot_input_service"],
            RobotInputService,
        )
        control_service.shutdown.assert_called_once_with()
        self.assertTrue(
            json.loads(stdout.getvalue())["physical_control_enabled"]
        )

    def test_run_reuses_configured_live_console_access_key(self):
        adapter = mock.Mock()
        dashboard_service = mock.Mock()
        control_service = mock.Mock()
        http_server = mock.Mock()
        router = mock.Mock(session_path="/live/stable-key/")

        with (
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=dashboard_service,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.RobotControlService",
                return_value=control_service,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.load_or_create_dashboard_access_key",
                return_value="a" * 64,
            ) as key_loader,
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(http_server, router),
            ) as server_factory,
            mock.patch("sys.stdout", new_callable=io.StringIO),
        ):
            result = _run(
                ["--console-access-key-file", "~/.robot-llm/test-key"],
                robot_runtime_adapter=adapter,
            )

        self.assertEqual(result, 0)
        key_loader.assert_called_once_with(
            Path("~/.robot-llm/test-key"),
        )
        server_factory.assert_called_once()
        server_args = server_factory.call_args
        self.assertEqual(server_args.args, (dashboard_service, 8765))
        self.assertIs(
            server_args.kwargs["robot_control_service"],
            control_service,
        )
        self.assertIsInstance(
            server_args.kwargs["robot_input_service"],
            RobotInputService,
        )
        self.assertEqual(
            server_args.kwargs["session_token"],
            "a" * 64,
        )

    def test_speech_source_is_explicit_and_mutually_exclusive(self):
        defaults = _parser().parse_args([])
        managed = _parser().parse_args(
            [
                "--stt-model",
                "models/ggml-base.bin",
                "--stt-port",
                "8123",
                "--stt-threads",
                "6",
            ]
        )
        external = _parser().parse_args(
            [
                "--stt-url",
                "http://127.0.0.1:8178",
                "--stt-model-id",
                "ggml-small",
                "--stt-inference-path",
                "/audio/transcriptions",
            ]
        )

        self.assertIsNone(defaults.stt_model)
        self.assertIsNone(defaults.stt_url)
        self.assertEqual(defaults.stt_inference_path, "/inference")
        self.assertEqual(managed.stt_model, "models/ggml-base.bin")
        self.assertEqual(managed.stt_port, 8123)
        self.assertEqual(managed.stt_threads, 6)
        self.assertFalse(managed.stt_cpu)
        self.assertEqual(
            external.stt_url,
            "http://127.0.0.1:8178",
        )
        self.assertEqual(external.stt_model_id, "ggml-small")
        self.assertEqual(
            external.stt_inference_path,
            "/audio/transcriptions",
        )
        with (
            self.assertRaises(SystemExit),
            mock.patch("sys.stderr", new_callable=io.StringIO),
        ):
            _parser().parse_args(
                [
                    "--stt-model",
                    "models/ggml-base.bin",
                    "--stt-url",
                    "http://127.0.0.1:8178",
                ]
            )

    def test_main_wires_and_probes_external_speech_server(self):
        transcriber = mock.Mock()
        service = mock.Mock()
        http_server = mock.Mock()
        router = mock.Mock()
        router.session_path = "/live/token/"
        events = []
        transcriber.probe.side_effect = lambda: events.append("probe")

        def build_service(**kwargs):
            events.append("service")
            return service

        with (
            mock.patch(
                "robot_agent.stt_whisper_cpp.WhisperCppTranscriber",
                return_value=transcriber,
            ) as transcriber_factory,
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                side_effect=build_service,
            ) as service_factory,
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(http_server, router),
            ),
            mock.patch("sys.stdout", new_callable=io.StringIO) as stdout,
        ):
            result = main(
                [
                    "--stt-url",
                    "http://127.0.0.1:8178/v1",
                    "--stt-inference-path",
                    "/audio/transcriptions",
                    "--stt-model-id",
                    "ggml-large-v3-turbo-q5_0",
                ]
            )

        self.assertEqual(result, 0)
        transcriber_factory.assert_called_once_with(
            base_url="http://127.0.0.1:8178/v1",
            inference_path="/audio/transcriptions",
            model_id="ggml-large-v3-turbo-q5_0",
            require_opaque_path=True,
        )
        transcriber.probe.assert_called_once_with()
        self.assertEqual(events, ["probe", "service"])
        self.assertIs(
            service_factory.call_args.kwargs["speech_transcriber"],
            transcriber,
        )
        http_server.serve_forever.assert_called_once_with(
            poll_interval=0.25
        )
        http_server.server_close.assert_called_once_with()
        service.shutdown.assert_called_once_with()
        ready = json.loads(stdout.getvalue())
        self.assertTrue(ready["speech_to_text_enabled"])

    def test_external_speech_probe_failure_prevents_ready(self):
        transcriber = mock.Mock()
        transcriber.probe.side_effect = RuntimeError(
            "Speech provider is unavailable"
        )

        with (
            mock.patch(
                "robot_agent.stt_whisper_cpp.WhisperCppTranscriber",
                return_value=transcriber,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
            ) as service_factory,
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
            ) as server_factory,
            mock.patch("sys.stdout", new_callable=io.StringIO) as stdout,
            mock.patch("sys.stderr", new_callable=io.StringIO) as stderr,
        ):
            result = main(
                [
                    "--stt-url",
                    "http://127.0.0.1:8178/v1",
                    "--stt-inference-path",
                    "/audio/transcriptions",
                ]
            )

        self.assertEqual(result, 2)
        transcriber.probe.assert_called_once_with()
        service_factory.assert_not_called()
        server_factory.assert_not_called()
        self.assertEqual(stdout.getvalue(), "")
        failure = json.loads(stderr.getvalue())
        self.assertEqual(failure["status"], "failed")
        self.assertEqual(
            failure["error"],
            "Speech provider is unavailable",
        )

    def test_main_wires_and_stops_managed_speech_server(self):
        speech_server = mock.Mock()
        private_prefix = "/stt-" + "a" * 48
        speech_server.base_url = (
            "http://127.0.0.1:8178" + private_prefix
        )
        speech_server.model_id = "ggml-base"
        transcriber = object()
        service = mock.Mock()
        http_server = mock.Mock()
        router = mock.Mock()
        router.session_path = "/live/token/"

        with (
            mock.patch(
                "robot_agent.stt_whisper_server.WhisperCppServer",
                return_value=speech_server,
            ) as server_factory,
            mock.patch(
                "robot_agent.stt_whisper_cpp.WhisperCppTranscriber",
                return_value=transcriber,
            ) as transcriber_factory,
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=service,
            ) as service_factory,
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(http_server, router),
            ),
            mock.patch("sys.stdout", new_callable=io.StringIO) as stdout,
        ):
            result = main(
                [
                    "--port",
                    "8765",
                    "--stt-model",
                    "models/ggml-base.bin",
                    "--stt-port",
                    "8178",
                    "--stt-threads",
                    "6",
                    "--stt-cpu",
                ]
            )

        self.assertEqual(result, 0)
        server_factory.assert_called_once_with(
            "models/ggml-base.bin",
            binary="whisper-server",
            port=8178,
            threads=6,
            use_gpu=False,
        )
        speech_server.start.assert_called_once_with()
        transcriber_factory.assert_called_once_with(
            base_url="http://127.0.0.1:8178" + private_prefix,
            model_id="ggml-base",
            require_opaque_path=True,
        )
        self.assertIs(
            service_factory.call_args.kwargs["speech_transcriber"],
            transcriber,
        )
        http_server.serve_forever.assert_called_once_with(
            poll_interval=0.25
        )
        http_server.server_close.assert_called_once_with()
        service.shutdown.assert_called_once_with()
        speech_server.stop.assert_called_once_with()
        ready_output = stdout.getvalue()
        self.assertNotIn(private_prefix, ready_output)
        ready = json.loads(ready_output)
        self.assertTrue(ready["speech_to_text_enabled"])

    def test_ready_interrupt_still_closes_every_owned_resource(self):
        speech_server = mock.Mock()
        speech_server.base_url = (
            "http://127.0.0.1:8178/stt-" + "a" * 48
        )
        speech_server.model_id = "ggml-base"
        service = mock.Mock()
        http_server = mock.Mock()
        router = mock.Mock()
        router.session_path = "/live/token/"

        with (
            mock.patch(
                "robot_agent.stt_whisper_server.WhisperCppServer",
                return_value=speech_server,
            ),
            mock.patch(
                "robot_agent.stt_whisper_cpp.WhisperCppTranscriber",
                return_value=object(),
            ),
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=service,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(http_server, router),
            ),
            mock.patch(
                "builtins.print",
                side_effect=KeyboardInterrupt,
            ),
        ):
            result = main(
                [
                    "--stt-model",
                    "models/ggml-base.bin",
                    "--stt-cpu",
                ]
            )

        self.assertEqual(result, 0)
        http_server.serve_forever.assert_not_called()
        http_server.server_close.assert_called_once_with()
        service.shutdown.assert_called_once_with()
        speech_server.stop.assert_called_once_with()

    def test_cleanup_failure_does_not_skip_managed_speech_stop(self):
        speech_server = mock.Mock()
        speech_server.base_url = (
            "http://127.0.0.1:8178/stt-" + "a" * 48
        )
        speech_server.model_id = "ggml-base"
        service = mock.Mock()
        service.shutdown.side_effect = RuntimeError("shutdown failed")
        http_server = mock.Mock()
        router = mock.Mock()
        router.session_path = "/live/token/"

        with (
            mock.patch(
                "robot_agent.stt_whisper_server.WhisperCppServer",
                return_value=speech_server,
            ),
            mock.patch(
                "robot_agent.stt_whisper_cpp.WhisperCppTranscriber",
                return_value=object(),
            ),
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=service,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(http_server, router),
            ),
            mock.patch("sys.stdout", new_callable=io.StringIO),
        ):
            with self.assertRaisesRegex(
                RuntimeError,
                "shutdown failed",
            ):
                main(
                [
                    "--stt-model",
                    "models/ggml-base.bin",
                    "--stt-cpu",
                ]
                )

        speech_server.stop.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
