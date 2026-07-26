"""Run the local, motion-free Robot LLM dashboard on macOS."""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import socket
import sys
import threading
from typing import Optional, Sequence

from .dashboard_http import (
    DashboardHTTPResponse,
    DashboardRouter,
    MAX_REQUEST_BYTES,
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

        def _request_body(self) -> bytes:
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
            if length > MAX_REQUEST_BYTES:
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
                body = self._request_body()
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
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    try:
        service = DashboardService(
            base_url=args.lm_studio_url,
            model=args.model,
        )
        server, _router = build_server(service, args.port)
    except (OSError, RuntimeError, ValueError) as error:
        if "service" in locals():
            service.shutdown()
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
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        service.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
