import json
from unittest import TestCase

from robot_agent.http_transport import DirectHTTPResponse
from robot_agent.remote_spatial_map import (
    MAX_REMOTE_SPATIAL_MAP_BYTES,
    RemoteSpatialMapError,
    RemoteSpatialMapProvider,
)


ACCESS_KEY = "a" * 64


def response(value, *, status=200, headers=None):
    if headers is None:
        headers = (("Content-Type", "application/json; charset=utf-8"),)
    body = (
        value
        if isinstance(value, bytes)
        else json.dumps(value, separators=(",", ":")).encode("utf-8")
    )
    return DirectHTTPResponse(status, tuple(headers), body)


def local_map(**changes):
    value = {
        "schema": "robot-spatial-map/v1",
        "read_only": True,
        "status": "pose_only",
        "robot_id": "blast-01",
    }
    value.update(changes)
    return value


class RecordingTransport:
    def __init__(self, result=None, error=None):
        self.result = result
        self.error = error
        self.calls = []

    def __call__(self, *args):
        self.calls.append(args)
        if self.error is not None:
            raise self.error
        return self.result


class RemoteSpatialMapProviderTests(TestCase):
    def test_fetches_only_the_authenticated_loopback_v1_map(self):
        transport = RecordingTransport(response({"map": local_map()}))
        provider = RemoteSpatialMapProvider(
            8_766,
            ACCESS_KEY,
            timeout_seconds=0.75,
            transport=transport,
        )

        value = provider.snapshot()

        self.assertEqual(value, local_map())
        self.assertEqual(
            transport.calls,
            [
                (
                    "GET",
                    "http://127.0.0.1:8766/api/v1/map",
                    {
                        "Accept": "application/json",
                        "x-robot-dashboard-token": ACCESS_KEY,
                    },
                    None,
                    0.75,
                    MAX_REMOTE_SPATIAL_MAP_BYTES,
                )
            ],
        )

    def test_constructor_accepts_no_host_path_or_unsafe_credentials(self):
        invalid = (
            (True, ACCESS_KEY, 1.0, RecordingTransport()),
            (0, ACCESS_KEY, 1.0, RecordingTransport()),
            (65_536, ACCESS_KEY, 1.0, RecordingTransport()),
            ("8766", ACCESS_KEY, 1.0, RecordingTransport()),
            (8_766, "short", 1.0, RecordingTransport()),
            (8_766, "a" * 63 + "\n", 1.0, RecordingTransport()),
            (8_766, ACCESS_KEY, True, RecordingTransport()),
            (8_766, ACCESS_KEY, 0, RecordingTransport()),
            (8_766, ACCESS_KEY, float("inf"), RecordingTransport()),
            (8_766, ACCESS_KEY, 5.01, RecordingTransport()),
            (8_766, ACCESS_KEY, 1.0, None),
        )
        for port, key, timeout, transport in invalid:
            with self.subTest(port=port, timeout=timeout):
                with self.assertRaisesRegex(
                    RemoteSpatialMapError,
                    "^Remote spatial map configuration is invalid$",
                ):
                    RemoteSpatialMapProvider(
                        port,
                        key,
                        timeout_seconds=timeout,
                        transport=transport,
                    )

    def test_redirect_and_http_errors_are_not_followed_or_exposed(self):
        for peer_response in (
            response(
                b"",
                status=302,
                headers=(("Location", "http://attacker.invalid/map"),),
            ),
            response(
                {"error": {"message": "private-peer-detail"}},
                status=403,
            ),
        ):
            transport = RecordingTransport(peer_response)
            with self.subTest(status=peer_response.status_code):
                with self.assertRaisesRegex(
                    RemoteSpatialMapError,
                    "^Remote spatial map is unavailable$",
                ) as raised:
                    RemoteSpatialMapProvider(
                        8_766, ACCESS_KEY, transport=transport
                    ).snapshot()
                self.assertNotIn("private", str(raised.exception))
                self.assertNotIn("attacker", str(raised.exception))
                self.assertEqual(len(transport.calls), 1)

    def test_transport_failure_does_not_leak_secret_or_peer_details(self):
        secret = "peer failed with key {}".format(ACCESS_KEY)
        provider = RemoteSpatialMapProvider(
            8_766,
            ACCESS_KEY,
            transport=RecordingTransport(error=OSError(secret)),
        )

        with self.assertRaisesRegex(
            RemoteSpatialMapError,
            "^Remote spatial map is unavailable$",
        ) as raised:
            provider.snapshot()

        self.assertNotIn(ACCESS_KEY, str(raised.exception))
        self.assertIsNone(raised.exception.__cause__)
        self.assertIsNone(raised.exception.__context__)

    def test_requires_one_json_content_type_with_utf8_if_declared(self):
        invalid_headers = (
            (),
            (("Content-Type", "text/html"),),
            (("Content-Type", "application/json; charset=latin-1"),),
            (
                ("Content-Type", "application/json"),
                ("content-type", "application/json"),
            ),
        )
        for headers in invalid_headers:
            with self.subTest(headers=headers):
                provider = RemoteSpatialMapProvider(
                    8_766,
                    ACCESS_KEY,
                    transport=RecordingTransport(response(
                        {"map": local_map()}, headers=headers
                    )),
                )
                with self.assertRaises(RemoteSpatialMapError):
                    provider.snapshot()

    def test_rejects_oversized_malformed_or_non_exact_envelopes(self):
        cases = (
            b"x" * (MAX_REMOTE_SPATIAL_MAP_BYTES + 1),
            b"\xff",
            b'{"map":{},"map":{}}',
            b'{"map":{"schema":"robot-spatial-map/v1",'
            b'"read_only":true,"number":NaN}}',
            [],
            {},
            {"map": local_map(), "extra": True},
        )
        for value in cases:
            with self.subTest(value_type=type(value).__name__):
                provider = RemoteSpatialMapProvider(
                    8_766,
                    ACCESS_KEY,
                    transport=RecordingTransport(response(value)),
                )
                with self.assertRaisesRegex(
                    RemoteSpatialMapError,
                    "^Remote spatial map is unavailable$",
                ):
                    provider.snapshot()

    def test_requires_exact_v1_schema_and_boolean_read_only(self):
        invalid_maps = (
            None,
            {},
            local_map(schema="robot-spatial-map/v2"),
            local_map(read_only=False),
            local_map(read_only=1),
        )
        for value in invalid_maps:
            with self.subTest(value=value):
                provider = RemoteSpatialMapProvider(
                    8_766,
                    ACCESS_KEY,
                    transport=RecordingTransport(response({"map": value})),
                )
                with self.assertRaises(RemoteSpatialMapError):
                    provider.snapshot()

    def test_wrong_transport_response_shape_is_generic_failure(self):
        provider = RemoteSpatialMapProvider(
            8_766,
            ACCESS_KEY,
            transport=RecordingTransport(object()),
        )

        with self.assertRaisesRegex(
            RemoteSpatialMapError,
            "^Remote spatial map is unavailable$",
        ):
            provider.snapshot()
