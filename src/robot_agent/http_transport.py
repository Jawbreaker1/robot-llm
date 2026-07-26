"""Small direct HTTP transport with an absolute caller-visible deadline.

The transport deliberately bypasses ambient proxies and never follows
redirects.  A daemon worker owns the blocking standard-library connection;
the calling thread closes that connection and returns at the absolute
deadline even if DNS, headers, or a slow-drip body remain blocked.
"""

from dataclasses import dataclass
import http.client
import math
import socket
import threading
import time
from typing import Mapping, Optional, Tuple
from urllib.parse import urlsplit


class DirectHTTPTimeoutError(TimeoutError):
    """The absolute request deadline expired."""


class DirectHTTPTransportError(OSError):
    """A direct HTTP connection or response failed."""


@dataclass(frozen=True)
class DirectHTTPResponse:
    status_code: int
    headers: Tuple[Tuple[str, str], ...]
    body: bytes

    def header_values(self, name: str) -> Tuple[str, ...]:
        lowered = name.lower()
        return tuple(
            value
            for key, value in self.headers
            if key.lower() == lowered
        )


def _request_target(parsed) -> str:
    target = parsed.path or "/"
    if parsed.query:
        target += "?" + parsed.query
    return target


def direct_http_request(
    method: str,
    url: str,
    headers: Mapping[str, str],
    body: Optional[bytes],
    timeout_seconds: float,
    max_response_bytes: int,
) -> DirectHTTPResponse:
    """Perform one direct request without proxy or redirect behavior."""

    if (
        method not in ("GET", "POST")
        or not isinstance(url, str)
        or not isinstance(headers, Mapping)
        or body is not None
        and not isinstance(body, bytes)
        or isinstance(timeout_seconds, bool)
        or not isinstance(timeout_seconds, (int, float))
        or not math.isfinite(float(timeout_seconds))
        or timeout_seconds <= 0
        or isinstance(max_response_bytes, bool)
        or not isinstance(max_response_bytes, int)
        or max_response_bytes <= 0
    ):
        raise DirectHTTPTransportError(
            "Direct HTTP request configuration is invalid"
        )
    try:
        parsed = urlsplit(url)
        port = parsed.port
    except ValueError:
        raise DirectHTTPTransportError(
            "Direct HTTP URL is invalid"
        ) from None
    if (
        parsed.scheme not in ("http", "https")
        or parsed.hostname is None
        or parsed.username is not None
        or parsed.password is not None
        or parsed.fragment
    ):
        raise DirectHTTPTransportError(
            "Direct HTTP URL is invalid"
        )
    for key, value in headers.items():
        if (
            not isinstance(key, str)
            or not key
            or not isinstance(value, str)
            or "\r" in key
            or "\n" in key
            or "\r" in value
            or "\n" in value
        ):
            raise DirectHTTPTransportError(
                "Direct HTTP headers are invalid"
            )

    connection_class = (
        http.client.HTTPSConnection
        if parsed.scheme == "https"
        else http.client.HTTPConnection
    )
    connection = connection_class(
        parsed.hostname,
        port=port,
        timeout=float(timeout_seconds),
    )
    outcome = []
    cancelled = threading.Event()
    active_socket = []

    def perform() -> None:
        try:
            connection.connect()
            active_socket.append(connection.sock)
            # Prevent ``HTTPConnection.send`` from reconnecting if the
            # deadline thread closes the socket between this check and send.
            connection.auto_open = 0
            if cancelled.is_set():
                raise DirectHTTPTimeoutError(
                    "Direct HTTP request timed out"
                )
            connection.request(
                method,
                _request_target(parsed),
                body=body,
                headers=dict(headers),
            )
            response = connection.getresponse()
            if cancelled.is_set():
                raise DirectHTTPTimeoutError(
                    "Direct HTTP request timed out"
                )
            response_body = response.read(max_response_bytes + 1)
            if cancelled.is_set():
                raise DirectHTTPTimeoutError(
                    "Direct HTTP request timed out"
                )
            outcome.append(
                (
                    "response",
                    DirectHTTPResponse(
                        status_code=response.status,
                        headers=tuple(response.getheaders()),
                        body=response_body,
                    ),
                )
            )
        except Exception as error:
            outcome.append(("error", error))
        finally:
            try:
                connection.close()
            except OSError:
                pass

    started = time.monotonic()
    worker = threading.Thread(
        target=perform,
        name="robot-llm-direct-http",
        daemon=True,
    )
    worker.start()
    worker.join(float(timeout_seconds))
    elapsed = time.monotonic() - started
    if worker.is_alive() or elapsed >= float(timeout_seconds):
        cancelled.set()
        if active_socket and active_socket[0] is not None:
            request_socket = active_socket[0]
            try:
                request_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                request_socket.close()
            except OSError:
                pass
        raise DirectHTTPTimeoutError(
            "Direct HTTP request timed out"
        )
    if not outcome:
        raise DirectHTTPTransportError(
            "Direct HTTP worker returned no result"
        )
    kind, value = outcome[0]
    if kind == "response":
        return value
    if isinstance(value, (socket.timeout, TimeoutError)):
        raise DirectHTTPTimeoutError(
            "Direct HTTP request timed out"
        ) from None
    if isinstance(value, (OSError, http.client.HTTPException)):
        raise DirectHTTPTransportError(
            "Direct HTTP request failed"
        ) from None
    raise value
