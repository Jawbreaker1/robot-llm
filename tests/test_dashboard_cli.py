import io
import socket
import threading
import unittest
from unittest import mock

from http.server import ThreadingHTTPServer

from robot_agent.dashboard_cli import (
    _LoopbackThreadingHTTPServer,
    _handler_class,
    _parser,
)
from robot_agent.dashboard_http import DashboardHTTPResponse


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


class DashboardCLITests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
