"""Run the local, motion-free Robot LLM dashboard on macOS."""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import signal
import socket
import sys
import threading
from typing import Optional, Sequence

from .dashboard_http import (
    DashboardHTTPResponse,
    DashboardRouter,
    new_session_token,
)
from .dashboard_service import DashboardService
from .lm_studio import DEFAULT_BASE_URL, DEFAULT_MODEL


LOOPBACK_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
MAX_HTTP_REQUEST_THREADS = 16
HTTP_READ_TIMEOUT_SECONDS = 5.0


class _LoopbackThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        server_address,
        request_handler_class,
        *,
        max_request_threads: int = MAX_HTTP_REQUEST_THREADS,
        read_timeout_seconds: float = HTTP_READ_TIMEOUT_SECONDS,
    ):
        if (
            isinstance(max_request_threads, bool)
            or not isinstance(max_request_threads, int)
            or not 1 <= max_request_threads <= 256
            or isinstance(read_timeout_seconds, bool)
            or not isinstance(read_timeout_seconds, (int, float))
            or not math.isfinite(float(read_timeout_seconds))
            or not 0 < float(read_timeout_seconds) <= 60
        ):
            raise ValueError("Dashboard HTTP limits are invalid")
        self._request_slots = threading.BoundedSemaphore(
            max_request_threads
        )
        self._read_timeout_seconds = float(read_timeout_seconds)
        super().__init__(server_address, request_handler_class)

    def get_request(self):
        request, client_address = super().get_request()
        request.settimeout(self._read_timeout_seconds)
        return request, client_address

    def process_request(self, request, client_address):
        if not self._request_slots.acquire(blocking=False):
            try:
                request.sendall(
                    b"HTTP/1.1 503 Service Unavailable\r\n"
                    b"Connection: close\r\n"
                    b"Cache-Control: no-store\r\n"
                    b"Content-Length: 0\r\n\r\n"
                )
            except OSError:
                pass
            try:
                request.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            request.close()
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self._request_slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self._request_slots.release()


def _handler_class(router: DashboardRouter):
    class DashboardHandler(BaseHTTPRequestHandler):
        server_version = "RobotLLMDashboard/1"
        sys_version = ""

        def log_message(self, _fmt, *_args):
            # Polling is intentionally quiet, and the capability-bearing
            # session URL must never be copied into a terminal log.
            status = str(_args[1]) if len(_args) > 1 else ""
            if status.isascii() and status.isdigit() and int(status) >= 400:
                sys.stderr.write(
                    "[dashboard] HTTP error status {}\n".format(status)
                )

        def _send(self, response: DashboardHTTPResponse):
            self.send_response(response.status)
            for name, value in response.headers:
                self.send_header(name, value)
            self.send_header("Content-Length", str(len(response.body)))
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(response.body)

        def _request_body(self, max_bytes: int) -> bytes:
            raw_length = self.headers.get("Content-Length")
            if raw_length is None:
                return b""
            if (
                not raw_length.isascii()
                or not raw_length.isdigit()
                or len(raw_length) > 8
            ):
                raise ValueError("invalid content length")
            length = int(raw_length)
            if length > max_bytes:
                raise OverflowError("request body too large")
            body = self.rfile.read(length)
            if len(body) != length:
                raise ValueError("incomplete request body")
            return body

        def _dispatch(self):
            headers = {}
            for name in self.headers:
                values = self.headers.get_all(name)
                lowered = name.lower()
                if (
                    not values
                    or len(values) != 1
                    or lowered in headers
                ):
                    self._send(
                        router._response(
                            400,
                            {
                                "error": {
                                    "code": "duplicate_header",
                                    "message": "Request headers are invalid",
                                }
                            },
                        )
                    )
                    return
                headers[lowered] = values[0]

            preflight = router.preflight(
                self.command,
                self.path,
                headers,
            )
            if preflight is not None:
                self._send(preflight)
                return
            try:
                body = self._request_body(
                    router.request_body_limit(
                        self.command,
                        self.path,
                    )
                )
            except OverflowError:
                self._send(
                    router._response(
                        413,
                        {
                            "error": {
                                "code": "request_too_large",
                                "message": "Request body is too large",
                            }
                        },
                    )
                )
                return
            except (socket.timeout, TimeoutError):
                self._send(
                    router._response(
                        408,
                        {
                            "error": {
                                "code": "request_timeout",
                                "message": "Request body timed out",
                            }
                        },
                    )
                )
                return
            except ValueError:
                self._send(
                    router._response(
                        400,
                        {
                            "error": {
                                "code": "invalid_http_framing",
                                "message": "Request framing is invalid",
                            }
                        },
                    )
                )
                return

            try:
                response = router.handle(
                    self.command,
                    self.path,
                    headers,
                    body,
                )
            except Exception:
                response = router._response(
                    500,
                    {
                        "error": {
                            "code": "internal_error",
                            "message": "Dashboard request failed",
                        }
                    },
                )
            self._send(response)

        do_GET = _dispatch
        do_POST = _dispatch
        do_PUT = _dispatch
        do_DELETE = _dispatch
        do_PATCH = _dispatch
        do_OPTIONS = _dispatch
        do_HEAD = _dispatch

    return DashboardHandler


def build_server(
    service: DashboardService,
    port: int = DEFAULT_PORT,
    session_token: Optional[str] = None,
):
    if (
        isinstance(port, bool)
        or not isinstance(port, int)
        or not 1 <= port <= 65_535
    ):
        raise ValueError("Dashboard port is invalid")
    token = session_token or new_session_token()
    expected_host = "{}:{}".format(LOOPBACK_HOST, port)
    router = DashboardRouter(
        service=service,
        session_token=token,
        expected_host=expected_host,
    )
    server = _LoopbackThreadingHTTPServer(
        (LOOPBACK_HOST, port),
        _handler_class(router),
    )
    return server, router


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Starta Robot LLM:s lokala, rörelsefria Mac-dashboard."
        )
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help="Loopback-port (default: %(default)s)",
    )
    parser.add_argument(
        "--lm-studio-url",
        default=DEFAULT_BASE_URL,
        help="Loopback-URL till LM Studio (default: %(default)s)",
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL,
        help="Exakt modell-ID som LM Studio ska använda",
    )
    parser.add_argument(
        "--simulation-map-demo",
        action="store_true",
        help=(
            "Bygg en riktig simulatorisk navigationskarta före start "
            "och visa den skrivskyddat i GUI:t"
        ),
    )
    stt_source = parser.add_mutually_exclusive_group()
    stt_source.add_argument(
        "--stt-model",
        help=(
            "Sökväg till en flerspråkig whisper.cpp-modell; "
            "dashboarden startar då en varm lokal server"
        ),
    )
    stt_source.add_argument(
        "--stt-url",
        help=(
            "Loopback-bas-URL till en redan startad whisper.cpp-server"
        ),
    )
    parser.add_argument(
        "--stt-model-id",
        default="whisper-multilingual",
        help=(
            "Säkert visnings-ID för modellen vid --stt-url "
            "(default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--stt-inference-path",
        default="/inference",
        help=(
            "Validerad relativ inferenssökväg vid --stt-url "
            "(default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--stt-whisper-binary",
        default="whisper-server",
        help="whisper-server-binär (default: %(default)s)",
    )
    parser.add_argument(
        "--stt-port",
        type=int,
        default=8178,
        help="Loopback-port för hanterad whisper-server (default: %(default)s)",
    )
    parser.add_argument(
        "--stt-threads",
        type=int,
        default=4,
        help="Whisper-trådar för hanterad server (default: %(default)s)",
    )
    parser.add_argument(
        "--stt-cpu",
        action="store_true",
        help=(
            "Kör hanterad whisper-server på CPU om Metal/GPU redan används"
        ),
    )
    return parser


def _close_resources(
    server,
    service,
    map_runtime,
    whisper_runtime,
    *,
    drain_map: bool,
) -> None:
    """Attempt every owned cleanup even if an earlier one is interrupted."""

    try:
        if server is not None:
            server.server_close()
    finally:
        try:
            if service is not None:
                service.shutdown()
        finally:
            try:
                if map_runtime is not None:
                    map_runtime.close(drain=drain_map)
            finally:
                if whisper_runtime is not None:
                    whisper_runtime.stop()


def _run(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    map_runtime = None
    whisper_runtime = None
    service = None
    server = None
    try:
        if args.simulation_map_demo:
            from .spatial_mapping_demo import (
                build_simulation_map_demo,
            )

            navigation, plant, map_runtime = (
                build_simulation_map_demo()
            )
            if (
                not navigation.completed
                or not navigation.terminal_stop_verified
                or plant.collision_count
            ):
                raise RuntimeError(
                    "Simulator map demo did not complete safely"
                )
        speech_transcriber = None
        if args.stt_model:
            from .stt_whisper_cpp import WhisperCppTranscriber
            from .stt_whisper_server import WhisperCppServer

            whisper_runtime = WhisperCppServer(
                args.stt_model,
                binary=args.stt_whisper_binary,
                port=args.stt_port,
                threads=args.stt_threads,
                use_gpu=not args.stt_cpu,
            )
            whisper_runtime.start()
            speech_transcriber = WhisperCppTranscriber(
                base_url=whisper_runtime.base_url,
                model_id=whisper_runtime.model_id,
                require_opaque_path=True,
            )
        elif args.stt_url:
            from .stt_whisper_cpp import WhisperCppTranscriber

            speech_transcriber = WhisperCppTranscriber(
                base_url=args.stt_url,
                inference_path=args.stt_inference_path,
                model_id=args.stt_model_id,
                require_opaque_path=True,
            )
            speech_transcriber.probe()
        service = DashboardService(
            base_url=args.lm_studio_url,
            model=args.model,
            spatial_map_provider=map_runtime,
            speech_transcriber=speech_transcriber,
        )
        server, _router = build_server(service, args.port)

        address = "http://{}:{}{}".format(
            LOOPBACK_HOST,
            args.port,
            _router.session_path,
        )
        print(
            json.dumps(
                {
                    "status": "ready",
                    "url": address,
                    "physical_control_enabled": False,
                    "speech_to_text_enabled": (
                        speech_transcriber is not None
                    ),
                    "spatial_map_mode": (
                        "simulation_demo"
                        if map_runtime is not None
                        else "unavailable"
                    ),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError) as error:
        print(
            json.dumps(
                {
                    "status": "failed",
                    "error": str(error),
                },
                ensure_ascii=False,
            ),
            file=sys.stderr,
        )
        return 2
    finally:
        _close_resources(
            server,
            service,
            map_runtime,
            whisper_runtime,
            drain_map=server is not None,
        )
    return 0


def _raise_termination_interrupt(_signum, _frame) -> None:
    """Route normal service-manager termination through runtime cleanup."""

    raise KeyboardInterrupt


def main(argv: Optional[Sequence[str]] = None) -> int:
    previous_sigterm = None
    if threading.current_thread() is threading.main_thread():
        previous_sigterm = signal.signal(
            signal.SIGTERM,
            _raise_termination_interrupt,
        )
    try:
        return _run(argv)
    finally:
        if previous_sigterm is not None:
            signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    raise SystemExit(main())
