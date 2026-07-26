import os
import threading
import time
import unittest
from unittest import mock

from robot_agent.http_transport import (
    DirectHTTPTimeoutError,
    direct_http_request,
)


class FakeResponse:
    def __init__(self, status=200, body=b'{"ok":true}', headers=()):
        self.status = status
        self._body = body
        self._headers = tuple(headers)

    def read(self, _limit):
        return self._body

    def getheaders(self):
        return self._headers


class FakeConnection:
    response = FakeResponse()
    block_until_closed = False
    instances = []

    def __init__(self, host, port=None, timeout=None):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.requests = []
        self.closed = threading.Event()
        type(self).instances.append(self)

    def request(self, method, target, body=None, headers=None):
        self.requests.append((method, target, body, headers))

    def connect(self):
        self.sock = FakeSocket(self.closed)

    def getresponse(self):
        if type(self).block_until_closed:
            self.closed.wait(5)
            raise OSError("connection closed")
        return type(self).response

    def close(self):
        self.closed.set()


class FakeSocket:
    def __init__(self, closed):
        self._closed = closed

    def shutdown(self, _how):
        self._closed.set()

    def close(self):
        self._closed.set()


class DirectHTTPTransportTests(unittest.TestCase):
    def setUp(self):
        FakeConnection.response = FakeResponse()
        FakeConnection.block_until_closed = False
        FakeConnection.instances = []
        patcher = mock.patch(
            "robot_agent.http_transport.http.client.HTTPConnection",
            FakeConnection,
        )
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_post_is_direct_even_with_ambient_proxy(self):
        previous_proxy = os.environ.get("HTTP_PROXY")
        previous_no_proxy = os.environ.get("NO_PROXY")
        os.environ["HTTP_PROXY"] = "http://proxy.invalid:8080"
        os.environ["NO_PROXY"] = ""
        try:
            response = direct_http_request(
                "POST",
                "http://127.0.0.1:1234/chat",
                {"Content-Type": "application/json"},
                b'{"question":"private"}',
                1.0,
                1_024,
            )
        finally:
            if previous_proxy is None:
                os.environ.pop("HTTP_PROXY", None)
            else:
                os.environ["HTTP_PROXY"] = previous_proxy
            if previous_no_proxy is None:
                os.environ.pop("NO_PROXY", None)
            else:
                os.environ["NO_PROXY"] = previous_no_proxy

        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(FakeConnection.instances), 1)
        connection = FakeConnection.instances[0]
        self.assertEqual(connection.host, "127.0.0.1")
        self.assertEqual(connection.port, 1234)
        self.assertEqual(
            connection.requests,
            [
                (
                    "POST",
                    "/chat",
                    b'{"question":"private"}',
                    {"Content-Type": "application/json"},
                )
            ],
        )

    def test_redirect_is_returned_and_never_followed(self):
        FakeConnection.response = FakeResponse(
            status=302,
            body=b"",
            headers=(("Location", "http://external.invalid/probe"),),
        )

        response = direct_http_request(
            "POST",
            "http://127.0.0.1:1234/chat",
            {},
            b"x",
            1.0,
            1_024,
        )

        self.assertEqual(response.status_code, 302)
        self.assertEqual(len(FakeConnection.instances), 1)
        self.assertEqual(
            len(FakeConnection.instances[0].requests),
            1,
        )

    def test_blocked_response_cannot_extend_absolute_deadline(self):
        FakeConnection.block_until_closed = True
        started = time.monotonic()

        with self.assertRaises(DirectHTTPTimeoutError):
            direct_http_request(
                "GET",
                "http://127.0.0.1:1234/slow",
                {},
                None,
                0.05,
                1_024,
            )

        elapsed = time.monotonic() - started
        self.assertLess(elapsed, 0.25)
        self.assertTrue(FakeConnection.instances[0].closed.is_set())


if __name__ == "__main__":
    unittest.main()
