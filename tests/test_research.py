from dataclasses import FrozenInstanceError, replace
import hashlib
import json
import socket
import unittest
from unittest.mock import patch
from urllib.parse import parse_qs, urlsplit

import robot_agent.research as research
from robot_agent.http_transport import DirectHTTPResponse
from robot_agent.research import (
    ATTRIBUTION_URL,
    CURRENT_WEATHER_ENDPOINT,
    GEOCODING_ENDPOINT,
    GEOCODING_EVIDENCE_TTL_MS,
    LICENSE_URL,
    MAX_REQUEST_TTL_MS,
    MAX_RESPONSE_BYTES,
    PROVENANCE_POLICY_VERSION,
    WEATHER_EVIDENCE_TTL_MS,
    CurrentWeather,
    EvidenceProvenance,
    LocationEvidence,
    OpenMeteoWeatherTool,
    ResearchClockError,
    ResearchConfigurationError,
    ResearchContractError,
    ResearchDeadlineExceeded,
    ResearchHTTPError,
    ResearchLocationNotFound,
    ResearchProtocolError,
    ResearchResponseTooLarge,
    ResearchTransportError,
    ResolvedLocation,
    WeatherEvidence,
    WeatherResearchRequest,
    WeatherResearchResult,
)


def geocoding_body(result=None, **top_overrides):
    location = {
        "id": 2673730,
        "name": "Stockholm",
        "latitude": 59.32938,
        "longitude": 18.06871,
        "elevation": 17.0,
        "feature_code": "PPLC",
        "country_code": "SE",
        "admin1_id": 2673722,
        "timezone": "Europe/Stockholm",
        "population": 1515017,
        "postcodes": ["100 04", "111 20"],
        "country_id": 2661886,
        "country": "Sweden",
        "admin1": "Stockholm",
    }
    if result is not None:
        location = result
    value = {
        "results": [location],
        "generationtime_ms": 0.45,
    }
    value.update(top_overrides)
    return json.dumps(
        value,
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")


def weather_body(current=None, units=None, **top_overrides):
    current_value = {
        "time": "2026-07-26T14:15",
        "interval": 900,
        "temperature_2m": 22.4,
        "apparent_temperature": 21.8,
        "precipitation": 0.0,
        "weather_code": 2,
        "cloud_cover": 63,
        "wind_speed_10m": 11.2,
        "is_day": 1,
    }
    if current is not None:
        current_value = current
    units_value = {
        "time": "iso8601",
        "interval": "seconds",
        "temperature_2m": "°C",
        "apparent_temperature": "°C",
        "precipitation": "mm",
        "weather_code": "wmo code",
        "cloud_cover": "%",
        "wind_speed_10m": "km/h",
        "is_day": "",
    }
    if units is not None:
        units_value = units
    value = {
        "latitude": 59.33,
        "longitude": 18.07,
        "generationtime_ms": 0.12,
        "utc_offset_seconds": 0,
        "timezone": "GMT",
        "timezone_abbreviation": "GMT",
        "elevation": 20.0,
        "current_units": units_value,
        "current": current_value,
    }
    value.update(top_overrides)
    return json.dumps(
        value,
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")


class MutableClock:
    def __init__(self, value):
        self.value = value

    def __call__(self):
        return self.value


class RecordingTransport:
    def __init__(self, responses=None, error=None, after_call=None):
        self.responses = list(
            responses
            if responses is not None
            else (geocoding_body(), weather_body())
        )
        self.error = error
        self.after_call = after_call
        self.calls = []

    def __call__(
        self,
        url,
        headers,
        timeout_seconds,
        max_response_bytes,
    ):
        self.calls.append(
            (
                url,
                headers,
                timeout_seconds,
                max_response_bytes,
            )
        )
        if self.error is not None:
            raise self.error
        result = self.responses.pop(0)
        if self.after_call is not None:
            self.after_call(len(self.calls))
        return result


class ResearchTests(unittest.TestCase):
    def setUp(self):
        self.monotonic = MutableClock(10_000)
        self.wall = MutableClock(1_700_000_000_000)
        self.transport = RecordingTransport()
        self.tool = OpenMeteoWeatherTool(
            transport=self.transport,
            monotonic_clock_ms=self.monotonic,
            wall_clock_ms=self.wall,
        )

    def request(self, **overrides):
        values = {
            "request_id": "weather-request-1",
            "location_query": "Stockholm",
            "issued_at_monotonic_ms": 10_000,
            "valid_until_monotonic_ms": 15_000,
        }
        values.update(overrides)
        return WeatherResearchRequest(**values)

    def test_current_weather_returns_typed_immutable_evidence(self):
        result = self.tool.current(self.request())

        self.assertIsInstance(result, WeatherResearchResult)
        self.assertEqual(result.request_id, "weather-request-1")
        self.assertEqual(result.location.name, "Stockholm")
        self.assertEqual(result.location.country_code, "SE")
        self.assertEqual(result.location.administrative_area, "Stockholm")
        self.assertEqual(result.weather.temperature_c, 22.4)
        self.assertEqual(result.weather.weather_code, 2)
        self.assertTrue(result.weather.is_day)
        self.assertEqual(
            result.location_evidence.provenance.ttl_ms,
            GEOCODING_EVIDENCE_TTL_MS,
        )
        self.assertEqual(
            result.weather_evidence.provenance.ttl_ms,
            WEATHER_EVIDENCE_TTL_MS,
        )
        self.assertEqual(
            result.location_evidence.provenance.raw_sha256,
            hashlib.sha256(geocoding_body()).hexdigest(),
        )
        self.assertEqual(
            result.weather_evidence.provenance.raw_sha256,
            hashlib.sha256(weather_body()).hexdigest(),
        )
        self.assertEqual(
            result.location_evidence.provenance.byte_count,
            len(geocoding_body()),
        )
        self.assertEqual(
            result.weather_evidence.provenance.attribution_url,
            ATTRIBUTION_URL,
        )
        self.assertEqual(
            result.weather_evidence.provenance.license_url,
            LICENSE_URL,
        )
        self.assertEqual(
            result.weather_evidence.provenance.policy_version,
            PROVENANCE_POLICY_VERSION,
        )
        self.assertEqual(
            result.valid_until_unix_ms,
            self.wall.value + WEATHER_EVIDENCE_TTL_MS,
        )
        self.assertEqual(
            result.evidence,
            (result.location_evidence, result.weather_evidence),
        )
        with self.assertRaises(FrozenInstanceError):
            result.weather.temperature_c = 99.0
        with self.assertRaises(FrozenInstanceError):
            result.request.location_query = "Other"

    def test_requests_use_only_fixed_official_https_endpoints(self):
        result = self.tool.current(self.request())

        self.assertEqual(len(self.transport.calls), 2)
        geocoding_call, weather_call = self.transport.calls
        self.assertTrue(
            geocoding_call[0].startswith(GEOCODING_ENDPOINT + "?")
        )
        self.assertTrue(
            weather_call[0].startswith(CURRENT_WEATHER_ENDPOINT + "?")
        )
        for url, headers, timeout, body_limit in self.transport.calls:
            parsed = urlsplit(url)
            self.assertEqual(parsed.scheme, "https")
            self.assertIsNone(parsed.username)
            self.assertIsNone(parsed.password)
            self.assertEqual(headers["Accept"], "application/json")
            self.assertEqual(timeout, 3.0)
            self.assertEqual(body_limit, MAX_RESPONSE_BYTES)

        geocoding_query = parse_qs(urlsplit(geocoding_call[0]).query)
        self.assertEqual(
            geocoding_query,
            {
                "name": ["Stockholm"],
                "count": ["1"],
                "language": ["en"],
                "format": ["json"],
            },
        )
        weather_query = parse_qs(urlsplit(weather_call[0]).query)
        self.assertEqual(
            weather_query["current"],
            [
                (
                    "temperature_2m,apparent_temperature,precipitation,"
                    "weather_code,cloud_cover,wind_speed_10m,is_day"
                )
            ],
        )
        self.assertEqual(weather_query["timezone"], ["UTC"])
        self.assertEqual(
            result.location_evidence.provenance.source_url,
            geocoding_call[0],
        )
        self.assertEqual(
            result.weather_evidence.provenance.source_url,
            weather_call[0],
        )

    def test_result_to_dict_preserves_provenance_and_freshness(self):
        result = self.tool.current(self.request())

        value = result.to_dict()

        self.assertEqual(value["request_id"], result.request_id)
        self.assertEqual(value["location"]["location_id"], 2673730)
        self.assertEqual(value["weather"]["cloud_cover_percent"], 63)
        self.assertEqual(len(value["evidence"]), 2)
        self.assertEqual(
            value["evidence"][1]["provenance"]["provider"],
            "open-meteo",
        )
        self.assertEqual(
            value["evidence"][1]["provenance"][
                "retrieved_at_unix_ms"
            ],
            self.wall.value,
        )
        self.assertEqual(
            value["evidence"][1]["provenance"]["valid_until_unix_ms"],
            result.valid_until_unix_ms,
        )

    def test_request_contract_is_strict(self):
        invalid_values = (
            {"request_id": ""},
            {"request_id": True},
            {"location_query": " X"},
            {"location_query": "X"},
            {"location_query": "X\nY"},
            {"issued_at_monotonic_ms": True},
            {"valid_until_monotonic_ms": 10_000},
            {
                "valid_until_monotonic_ms": (
                    10_000 + MAX_REQUEST_TTL_MS + 1
                )
            },
        )
        for overrides in invalid_values:
            with self.subTest(overrides=overrides):
                with self.assertRaises(ResearchContractError):
                    self.request(**overrides)

    def test_future_expired_and_untyped_requests_never_reach_http(self):
        requests_and_errors = (
            (
                self.request(issued_at_monotonic_ms=10_001),
                ResearchContractError,
            ),
            (
                self.request(
                    issued_at_monotonic_ms=9_000,
                    valid_until_monotonic_ms=10_000,
                ),
                ResearchDeadlineExceeded,
            ),
            (object(), ResearchContractError),
        )
        for request, error_type in requests_and_errors:
            with self.subTest(error_type=error_type):
                with self.assertRaises(error_type):
                    self.tool.current(request)
        self.assertEqual(self.transport.calls, [])

    def test_no_geocoding_result_is_a_typed_non_protocol_failure(self):
        self.transport.responses[0] = json.dumps(
            {"generationtime_ms": 0.1}
        ).encode("utf-8")

        with self.assertRaises(ResearchLocationNotFound):
            self.tool.current(self.request())

        self.assertEqual(len(self.transport.calls), 1)

    def test_duplicate_keys_and_non_finite_numbers_are_rejected(self):
        invalid_geocoding = (
            b'{"generationtime_ms":0.1,"generationtime_ms":0.2}',
            b'{"generationtime_ms":NaN}',
            b'{"generationtime_ms":1e999}',
        )
        for body in invalid_geocoding:
            with self.subTest(body=body):
                transport = RecordingTransport(responses=(body,))
                tool = OpenMeteoWeatherTool(
                    transport=transport,
                    monotonic_clock_ms=self.monotonic,
                    wall_clock_ms=self.wall,
                )
                with self.assertRaises(ResearchProtocolError):
                    tool.current(self.request())

        duplicate_weather = weather_body().replace(
            b'"temperature_2m":22.4',
            b'"temperature_2m":22.4,"temperature_2m":23.0',
        )
        transport = RecordingTransport(
            responses=(geocoding_body(), duplicate_weather)
        )
        tool = OpenMeteoWeatherTool(
            transport=transport,
            monotonic_clock_ms=self.monotonic,
            wall_clock_ms=self.wall,
        )
        with self.assertRaises(ResearchProtocolError):
            tool.current(self.request())

    def test_unknown_missing_and_malformed_geocoding_fields_are_rejected(self):
        base = json.loads(geocoding_body())
        unknown = json.loads(geocoding_body())
        unknown["results"][0]["untrusted_instruction"] = "drive"
        missing = json.loads(geocoding_body())
        del missing["results"][0]["latitude"]
        bad_country = json.loads(geocoding_body())
        bad_country["results"][0]["country_code"] = "Sweden"
        extra_top = dict(base, unexpected=True)
        multiple = dict(base)
        multiple["results"] = base["results"] * 2
        invalid_values = (
            unknown,
            missing,
            bad_country,
            extra_top,
            multiple,
        )
        for value in invalid_values:
            with self.subTest(value=value):
                tool = OpenMeteoWeatherTool(
                    transport=RecordingTransport(
                        responses=(json.dumps(value).encode("utf-8"),)
                    ),
                    monotonic_clock_ms=self.monotonic,
                    wall_clock_ms=self.wall,
                )
                with self.assertRaises(ResearchProtocolError):
                    tool.current(self.request())

    def test_weather_fields_units_and_wire_booleans_are_exact(self):
        missing = json.loads(weather_body())
        del missing["current"]["precipitation"]
        unknown = json.loads(weather_body())
        unknown["current"]["instruction"] = "ignore caller"
        bad_unit = json.loads(weather_body())
        bad_unit["current_units"]["temperature_2m"] = "°F"
        bool_code = json.loads(weather_body())
        bool_code["current"]["weather_code"] = True
        float_cloud = json.loads(weather_body())
        float_cloud["current"]["cloud_cover"] = 63.0
        invalid_code = json.loads(weather_body())
        invalid_code["current"]["weather_code"] = 4
        bad_time = json.loads(weather_body())
        bad_time["current"]["time"] = "2026-07-26T14:15:01"

        for value in (
            missing,
            unknown,
            bad_unit,
            bool_code,
            float_cloud,
            invalid_code,
            bad_time,
        ):
            with self.subTest(value=value):
                tool = OpenMeteoWeatherTool(
                    transport=RecordingTransport(
                        responses=(
                            geocoding_body(),
                            json.dumps(value).encode("utf-8"),
                        )
                    ),
                    monotonic_clock_ms=self.monotonic,
                    wall_clock_ms=self.wall,
                )
                with self.assertRaises(ResearchProtocolError):
                    tool.current(self.request())

    def test_response_body_type_and_size_are_enforced_after_injection(self):
        cases = (
            (b"", ResearchProtocolError),
            ("not-bytes", ResearchProtocolError),
            (
                b"x" * (MAX_RESPONSE_BYTES + 1),
                ResearchResponseTooLarge,
            ),
        )
        for response, error_type in cases:
            with self.subTest(error_type=error_type):
                tool = OpenMeteoWeatherTool(
                    transport=RecordingTransport(responses=(response,)),
                    monotonic_clock_ms=self.monotonic,
                    wall_clock_ms=self.wall,
                )
                with self.assertRaises(error_type):
                    tool.current(self.request())

    def test_default_http_transport_is_direct_and_checks_metadata(self):
        url = GEOCODING_ENDPOINT + (
            "?name=Stockholm&count=1&language=en&format=json"
        )
        with patch.object(
            research,
            "direct_http_request",
            return_value=DirectHTTPResponse(
                status_code=200,
                headers=(
                    (
                        "Content-Type",
                        "application/json; charset=utf-8",
                    ),
                ),
                body=geocoding_body(),
            ),
        ) as direct:
            body = research._stdlib_get(
                url,
                {"Accept": "application/json"},
                1.25,
                MAX_RESPONSE_BYTES,
            )

        self.assertEqual(body, geocoding_body())
        direct.assert_called_once_with(
            "GET",
            url,
            {"Accept": "application/json"},
            None,
            1.25,
            MAX_RESPONSE_BYTES,
        )

    def test_default_http_transport_rejects_redirects_and_non_json(self):
        url = GEOCODING_ENDPOINT + (
            "?name=Stockholm&count=1&language=en&format=json"
        )
        cases = (
            (
                DirectHTTPResponse(
                    status_code=302,
                    headers=(
                        (
                            "Location",
                            "https://example.com/redirected",
                        ),
                    ),
                    body=b"",
                ),
                ResearchHTTPError,
            ),
            (
                DirectHTTPResponse(
                    status_code=200,
                    headers=(
                        (
                            "Content-Type",
                            "text/html; charset=utf-8",
                        ),
                    ),
                    body=geocoding_body(),
                ),
                ResearchProtocolError,
            ),
            (
                DirectHTTPResponse(
                    status_code=200,
                    headers=(
                        (
                            "Content-Type",
                            "application/json; charset=iso-8859-1",
                        ),
                    ),
                    body=geocoding_body(),
                ),
                ResearchProtocolError,
            ),
        )
        for response, error_type in cases:
            with self.subTest(error_type=error_type):
                with patch.object(
                    research,
                    "direct_http_request",
                    return_value=response,
                ):
                    with self.assertRaises(error_type):
                        research._stdlib_get(
                            url,
                            {"Accept": "application/json"},
                            1.0,
                            MAX_RESPONSE_BYTES,
                        )

    def test_transport_failures_are_typed_and_do_not_echo_query(self):
        cases = (
            (socket.timeout("private Stockholm"), ResearchDeadlineExceeded),
            (OSError("private Stockholm"), ResearchTransportError),
        )
        for error, error_type in cases:
            with self.subTest(error_type=error_type):
                tool = OpenMeteoWeatherTool(
                    transport=RecordingTransport(error=error),
                    monotonic_clock_ms=self.monotonic,
                    wall_clock_ms=self.wall,
                )
                with self.assertRaises(error_type) as raised:
                    tool.current(self.request())
                self.assertNotIn("Stockholm", str(raised.exception))
                self.assertNotIn("private", str(raised.exception))

    def test_deadline_is_rechecked_after_each_http_call(self):
        def expire(_call_number):
            self.monotonic.value = 15_000

        tool = OpenMeteoWeatherTool(
            transport=RecordingTransport(after_call=expire),
            monotonic_clock_ms=self.monotonic,
            wall_clock_ms=self.wall,
        )

        with self.assertRaises(ResearchDeadlineExceeded):
            tool.current(self.request())

    def test_each_http_timeout_is_capped_by_remaining_episode_time(self):
        self.monotonic.value = 10_000

        def advance(call_number):
            self.monotonic.value += 500 if call_number == 1 else 200

        transport = RecordingTransport(after_call=advance)
        tool = OpenMeteoWeatherTool(
            transport=transport,
            monotonic_clock_ms=self.monotonic,
            wall_clock_ms=self.wall,
        )
        request = self.request(valid_until_monotonic_ms=12_000)

        tool.current(request)

        self.assertEqual(transport.calls[0][2], 2.0)
        self.assertEqual(transport.calls[1][2], 1.5)

    def test_invalid_or_regressing_clocks_fail_closed(self):
        invalid_tool = OpenMeteoWeatherTool(
            transport=self.transport,
            monotonic_clock_ms=lambda: True,
            wall_clock_ms=self.wall,
        )
        with self.assertRaises(ResearchClockError):
            invalid_tool.current(self.request())

        calls = iter((10_000, 9_999))
        regressing_tool = OpenMeteoWeatherTool(
            transport=self.transport,
            monotonic_clock_ms=lambda: next(calls),
            wall_clock_ms=self.wall,
        )
        with self.assertRaises(ResearchClockError):
            regressing_tool.current(self.request())

        invalid_wall_tool = OpenMeteoWeatherTool(
            transport=RecordingTransport(),
            monotonic_clock_ms=self.monotonic,
            wall_clock_ms=lambda: -1,
        )
        with self.assertRaises(ResearchClockError):
            invalid_wall_tool.current(self.request())

    def test_provenance_rejects_non_official_or_mutated_urls(self):
        valid = self.tool.current(self.request())
        provenance = valid.weather_evidence.provenance
        invalid_urls = (
            "https://example.com/v1/forecast?latitude=1",
            provenance.source_url.replace("https://", "http://", 1),
            provenance.source_url + "&url=https%3A%2F%2Fexample.com",
            provenance.source_url.replace("timezone=UTC", "timezone=auto"),
        )
        for url in invalid_urls:
            with self.subTest(url=url):
                with self.assertRaises(ResearchContractError):
                    replace(provenance, source_url=url)

    def test_evidence_and_result_identity_cannot_be_cross_wired(self):
        result = self.tool.current(self.request())
        with self.assertRaises(ResearchContractError):
            LocationEvidence(
                request_id=result.request_id,
                location=result.location,
                provenance=result.weather_evidence.provenance,
            )
        with self.assertRaises(ResearchContractError):
            WeatherEvidence(
                request_id=result.request_id,
                weather=result.weather,
                provenance=result.location_evidence.provenance,
            )
        with self.assertRaises(ResearchContractError):
            replace(
                result,
                weather_evidence=replace(
                    result.weather_evidence,
                    request_id="other-request",
                ),
            )

    def test_configuration_and_http_errors_are_safe_and_typed(self):
        invalid_configurations = (
            {"transport": None},
            {"monotonic_clock_ms": None},
            {"wall_clock_ms": None},
            {"request_timeout_seconds": True},
            {"request_timeout_seconds": 0},
            {"request_timeout_seconds": float("inf")},
            {"request_timeout_seconds": 31},
        )
        for values in invalid_configurations:
            with self.subTest(values=values):
                with self.assertRaises(ResearchConfigurationError):
                    OpenMeteoWeatherTool(**values)

        error = ResearchHTTPError(503)
        self.assertEqual(error.status_code, 503)
        self.assertEqual(
            str(error),
            "Research endpoint returned HTTP status 503",
        )

    def test_public_value_contracts_reject_bool_and_nonfinite_values(self):
        result = self.tool.current(self.request())
        with self.assertRaises(ResearchContractError):
            replace(result.location, latitude=float("nan"))
        with self.assertRaises(ResearchContractError):
            replace(result.weather, precipitation_mm=-0.1)
        with self.assertRaises(ResearchContractError):
            replace(result.weather, is_day=1)
        with self.assertRaises(ResearchContractError):
            replace(result.weather, weather_code=True)
        with self.assertRaises(ResearchContractError):
            replace(result.weather, cloud_cover_percent=True)
        with self.assertRaises(ResearchContractError):
            replace(
                result.location_evidence.provenance,
                retrieved_at_unix_ms=True,
            )


if __name__ == "__main__":
    unittest.main()
