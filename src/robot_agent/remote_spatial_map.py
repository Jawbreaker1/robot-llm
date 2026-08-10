"""Read one peer dashboard's local spatial map over loopback only.

The peer connection is deliberately narrow: callers supply only a TCP port
and an access key, while this provider owns the HTTP scheme, loopback address,
path, method, response limit, and redirect-free transport.  It is a read-only
source for the shared-map compositor and has no robot-control surface.
"""

from __future__ import annotations

from email.message import Message
import math
from typing import Callable

from .dashboard_contract import strict_json_loads
from .dashboard_http import TOKEN_HEADER
from .http_transport import DirectHTTPResponse, direct_http_request
from .spatial_map_contract import DASHBOARD_SPATIAL_MAP_SCHEMA


MAX_REMOTE_SPATIAL_MAP_BYTES = 4 * 1024 * 1024
DEFAULT_REMOTE_SPATIAL_MAP_TIMEOUT_SECONDS = 1.0
MAX_REMOTE_SPATIAL_MAP_TIMEOUT_SECONDS = 5.0
_MIN_ACCESS_KEY_CHARACTERS = 32
_MAX_ACCESS_KEY_CHARACTERS = 128
_UNAVAILABLE_MESSAGE = "Remote spatial map is unavailable"

RemoteSpatialMapTransport = Callable[..., DirectHTTPResponse]


class RemoteSpatialMapError(RuntimeError):
    """A configuration or peer failure safe to report without details."""


def _configuration_error() -> RemoteSpatialMapError:
    return RemoteSpatialMapError(
        "Remote spatial map configuration is invalid"
    )


def _valid_access_key(value: object) -> bool:
    return (
        isinstance(value, str)
        and _MIN_ACCESS_KEY_CHARACTERS <= len(value)
        <= _MAX_ACCESS_KEY_CHARACTERS
        and value.isascii()
        and all(
            character.isalnum() or character in "-_"
            for character in value
        )
    )


def _json_content_type(response: DirectHTTPResponse) -> bool:
    values = response.header_values("Content-Type")
    if len(values) != 1:
        return False
    metadata = Message()
    metadata["Content-Type"] = values[0]
    charset = metadata.get_content_charset()
    return (
        metadata.get_content_type() == "application/json"
        and (charset is None or charset.lower() == "utf-8")
    )


class RemoteSpatialMapProvider:
    """Fetch exact v1, read-only map snapshots from one loopback peer."""

    def __init__(
        self,
        peer_port: int,
        access_key: str,
        *,
        timeout_seconds: float = (
            DEFAULT_REMOTE_SPATIAL_MAP_TIMEOUT_SECONDS
        ),
        transport: RemoteSpatialMapTransport = direct_http_request,
    ):
        if (
            type(peer_port) is not int
            or not 1 <= peer_port <= 65_535
            or not _valid_access_key(access_key)
            or isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not math.isfinite(float(timeout_seconds))
            or not 0 < float(timeout_seconds)
            <= MAX_REMOTE_SPATIAL_MAP_TIMEOUT_SECONDS
            or not callable(transport)
        ):
            raise _configuration_error()

        self._url = "http://127.0.0.1:{}/api/v1/map".format(peer_port)
        self._access_key = access_key
        self._timeout_seconds = float(timeout_seconds)
        self._transport = transport

    def snapshot(self):
        """Return one fresh decoded snapshot or one generic peer failure."""

        try:
            response = self._transport(
                "GET",
                self._url,
                {
                    "Accept": "application/json",
                    TOKEN_HEADER: self._access_key,
                },
                None,
                self._timeout_seconds,
                MAX_REMOTE_SPATIAL_MAP_BYTES,
            )
            if (
                not isinstance(response, DirectHTTPResponse)
                or type(response.status_code) is not int
                or response.status_code != 200
                or not _json_content_type(response)
                or not isinstance(response.body, bytes)
                or not response.body
                or len(response.body) > MAX_REMOTE_SPATIAL_MAP_BYTES
            ):
                raise ValueError

            envelope = strict_json_loads(response.body)
            if (
                not isinstance(envelope, dict)
                or set(envelope) != {"map"}
                or not isinstance(envelope["map"], dict)
                or envelope["map"].get("schema")
                != DASHBOARD_SPATIAL_MAP_SCHEMA
                or envelope["map"].get("read_only") is not True
            ):
                raise ValueError
            return envelope["map"]
        except Exception:
            # Raise after leaving the handler so even ``__context__`` cannot
            # retain a transport error containing peer or credential detail.
            pass
        raise RemoteSpatialMapError(_UNAVAILABLE_MESSAGE)


__all__ = (
    "DEFAULT_REMOTE_SPATIAL_MAP_TIMEOUT_SECONDS",
    "MAX_REMOTE_SPATIAL_MAP_BYTES",
    "MAX_REMOTE_SPATIAL_MAP_TIMEOUT_SECONDS",
    "RemoteSpatialMapError",
    "RemoteSpatialMapProvider",
)
