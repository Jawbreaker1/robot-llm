"""Typed, bounded research tools for fresh external observations.

This module contains no natural-language classification.  A planner may choose
the weather tool, but the tool itself accepts only a typed location request and
returns typed evidence from two fixed Open-Meteo HTTPS endpoints.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from datetime import datetime
from email.message import Message
import hashlib
import json
import math
import socket
import time
from typing import Callable, Mapping, Optional, Tuple
from urllib.parse import parse_qsl, urlencode, urlsplit

from .http_transport import (
    DirectHTTPTimeoutError,
    DirectHTTPTransportError,
    direct_http_request,
)


GEOCODING_ENDPOINT = "https://geocoding-api.open-meteo.com/v1/search"
CURRENT_WEATHER_ENDPOINT = "https://api.open-meteo.com/v1/forecast"

REQUEST_TIMEOUT_SECONDS = 3.0
MAX_REQUEST_TTL_MS = 10_000
MAX_RESPONSE_BYTES = 64 * 1024
GEOCODING_EVIDENCE_TTL_MS = 24 * 60 * 60 * 1_000
WEATHER_EVIDENCE_TTL_MS = 10 * 60 * 1_000
ATTRIBUTION_URL = "https://open-meteo.com/"
LICENSE_URL = "https://creativecommons.org/licenses/by/4.0/"
PROVENANCE_POLICY_VERSION = "open-meteo-cc-by-4.0/v1"

_PROVIDER = "open-meteo"
_GEOCODING_SOURCE = "geocoding"
_CURRENT_WEATHER_SOURCE = "current_weather"
_CURRENT_VARIABLES = (
    "temperature_2m",
    "apparent_temperature",
    "precipitation",
    "weather_code",
    "cloud_cover",
    "wind_speed_10m",
    "is_day",
)
_WMO_WEATHER_CODES = frozenset(
    (
        0,
        1,
        2,
        3,
        45,
        48,
        51,
        53,
        55,
        56,
        57,
        61,
        63,
        65,
        66,
        67,
        71,
        73,
        75,
        77,
        80,
        81,
        82,
        85,
        86,
        95,
        96,
        99,
    )
)

HTTPTransport = Callable[
    [str, Mapping[str, str], float, int],
    bytes,
]
MillisecondClock = Callable[[], int]


class ResearchError(RuntimeError):
    """Base class for expected research-tool failures."""


class ResearchContractError(ResearchError):
    """A typed local request or result contract was invalid."""


class ResearchConfigurationError(ResearchError):
    """The tool was constructed with an invalid dependency or limit."""


class ResearchClockError(ResearchError):
    """An injected clock returned an invalid or regressing value."""


class ResearchDeadlineExceeded(ResearchError):
    """The request deadline expired before a complete result existed."""


class ResearchTransportError(ResearchError):
    """An HTTPS request could not be completed."""


class ResearchHTTPError(ResearchTransportError):
    """An official endpoint returned a non-success status."""

    def __init__(self, status_code: Optional[int]):
        self.status_code = status_code
        if (
            isinstance(status_code, int)
            and not isinstance(status_code, bool)
        ):
            message = "Research endpoint returned HTTP status {}".format(
                status_code
            )
        else:
            message = "Research endpoint returned an HTTP error"
        super().__init__(message)


class ResearchResponseTooLarge(ResearchError):
    """A response exceeded the pre-parse byte limit."""


class ResearchProtocolError(ResearchError):
    """A response did not satisfy the fixed provider contract."""


class ResearchLocationNotFound(ResearchError):
    """The fixed geocoder returned no location."""


def _identifier(name: str, value: str, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or not value.isprintable()
    ):
        raise ResearchContractError("{} is invalid".format(name))
    return value


def _integer(
    name: str,
    value: int,
    minimum: int,
    maximum: int,
) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise ResearchContractError("{} is invalid".format(name))
    return value


def _float(
    name: str,
    value: float,
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, float)
        or not math.isfinite(value)
        or minimum is not None
        and value < minimum
        or maximum is not None
        and value > maximum
    ):
        raise ResearchContractError("{} is invalid".format(name))
    return value


def _location_query(value: str) -> str:
    if (
        not isinstance(value, str)
        or value != value.strip()
        or len(value) < 2
        or len(value) > 200
        or not value.isprintable()
    ):
        raise ResearchContractError("location_query is invalid")
    return value


def _display_text(name: str, value: str, maximum: int = 200) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or not value.isprintable()
    ):
        raise ResearchContractError("{} is invalid".format(name))
    return value


def _country_code(value: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 2
        or value != value.upper()
        or any(character not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ" for character in value)
    ):
        raise ResearchContractError("country_code is invalid")
    return value


def _iso_minute(value: str) -> str:
    if not isinstance(value, str):
        raise ResearchContractError("observed_at is invalid")
    try:
        parsed = datetime.fromisoformat(value)
    except ValueError:
        raise ResearchContractError("observed_at is invalid") from None
    if (
        parsed.tzinfo is not None
        or parsed.second != 0
        or parsed.microsecond != 0
        or parsed.isoformat(timespec="minutes") != value
    ):
        raise ResearchContractError("observed_at is invalid")
    return value


@dataclass(frozen=True)
class WeatherResearchRequest:
    request_id: str
    location_query: str
    issued_at_monotonic_ms: int
    valid_until_monotonic_ms: int

    def __post_init__(self) -> None:
        _identifier("request_id", self.request_id)
        _location_query(self.location_query)
        _integer(
            "issued_at_monotonic_ms",
            self.issued_at_monotonic_ms,
            0,
            2**63 - 1,
        )
        _integer(
            "valid_until_monotonic_ms",
            self.valid_until_monotonic_ms,
            1,
            2**63 - 1,
        )
        ttl_ms = (
            self.valid_until_monotonic_ms
            - self.issued_at_monotonic_ms
        )
        if not 1 <= ttl_ms <= MAX_REQUEST_TTL_MS:
            raise ResearchContractError("request TTL is invalid")

    def to_dict(self) -> Mapping[str, object]:
        return {
            "request_id": self.request_id,
            "location_query": self.location_query,
            "issued_at_monotonic_ms": self.issued_at_monotonic_ms,
            "valid_until_monotonic_ms": self.valid_until_monotonic_ms,
        }


def _expected_query(
    source_kind: str,
    pairs: Tuple[Tuple[str, str], ...],
) -> None:
    keys = tuple(key for key, _value in pairs)
    values = dict(pairs)
    if len(values) != len(pairs):
        raise ResearchContractError("source_url is invalid")

    if source_kind == _GEOCODING_SOURCE:
        if keys != ("name", "count", "language", "format"):
            raise ResearchContractError("source_url is invalid")
        if (
            values["count"] != "1"
            or values["language"] != "en"
            or values["format"] != "json"
        ):
            raise ResearchContractError("source_url is invalid")
        _location_query(values["name"])
        return

    if source_kind == _CURRENT_WEATHER_SOURCE:
        if keys != (
            "latitude",
            "longitude",
            "current",
            "temperature_unit",
            "wind_speed_unit",
            "precipitation_unit",
            "timeformat",
            "timezone",
        ):
            raise ResearchContractError("source_url is invalid")
        if (
            values["current"] != ",".join(_CURRENT_VARIABLES)
            or values["temperature_unit"] != "celsius"
            or values["wind_speed_unit"] != "kmh"
            or values["precipitation_unit"] != "mm"
            or values["timeformat"] != "iso8601"
            or values["timezone"] != "UTC"
        ):
            raise ResearchContractError("source_url is invalid")
        try:
            latitude = float(values["latitude"])
            longitude = float(values["longitude"])
        except ValueError:
            raise ResearchContractError("source_url is invalid") from None
        if (
            not math.isfinite(latitude)
            or not -90.0 <= latitude <= 90.0
            or not math.isfinite(longitude)
            or not -180.0 <= longitude <= 180.0
        ):
            raise ResearchContractError("source_url is invalid")
        return

    raise ResearchContractError("source_kind is invalid")


def _validate_source_url(source_kind: str, source_url: str) -> str:
    if not isinstance(source_url, str):
        raise ResearchContractError("source_url is invalid")
    try:
        parsed = urlsplit(source_url)
        parsed.port
    except ValueError:
        raise ResearchContractError("source_url is invalid") from None

    endpoint = (
        GEOCODING_ENDPOINT
        if source_kind == _GEOCODING_SOURCE
        else CURRENT_WEATHER_ENDPOINT
        if source_kind == _CURRENT_WEATHER_SOURCE
        else None
    )
    if endpoint is None:
        raise ResearchContractError("source_kind is invalid")
    expected = urlsplit(endpoint)
    if (
        parsed.scheme != "https"
        or parsed.netloc != expected.netloc
        or parsed.path != expected.path
        or parsed.username is not None
        or parsed.password is not None
        or parsed.fragment
        or not parsed.query
    ):
        raise ResearchContractError("source_url is invalid")
    try:
        pairs = tuple(
            parse_qsl(
                parsed.query,
                keep_blank_values=True,
                strict_parsing=True,
            )
        )
    except ValueError:
        raise ResearchContractError("source_url is invalid") from None
    if urlencode(pairs) != parsed.query:
        raise ResearchContractError("source_url is invalid")
    _expected_query(source_kind, pairs)
    return source_url


@dataclass(frozen=True)
class EvidenceProvenance:
    provider: str
    source_kind: str
    source_url: str
    retrieved_at_unix_ms: int
    ttl_ms: int
    raw_sha256: str
    byte_count: int
    attribution_url: str
    license_url: str
    policy_version: str

    def __post_init__(self) -> None:
        if self.provider != _PROVIDER:
            raise ResearchContractError("provider is invalid")
        if self.source_kind not in (
            _GEOCODING_SOURCE,
            _CURRENT_WEATHER_SOURCE,
        ):
            raise ResearchContractError("source_kind is invalid")
        _validate_source_url(self.source_kind, self.source_url)
        _integer(
            "retrieved_at_unix_ms",
            self.retrieved_at_unix_ms,
            0,
            2**63 - 1,
        )
        expected_ttl = (
            GEOCODING_EVIDENCE_TTL_MS
            if self.source_kind == _GEOCODING_SOURCE
            else WEATHER_EVIDENCE_TTL_MS
        )
        if self.ttl_ms != expected_ttl:
            raise ResearchContractError("ttl_ms is invalid")
        if self.retrieved_at_unix_ms > 2**63 - 1 - self.ttl_ms:
            raise ResearchContractError("evidence validity overflowed")
        if (
            not isinstance(self.raw_sha256, str)
            or len(self.raw_sha256) != 64
            or any(
                character not in "0123456789abcdef"
                for character in self.raw_sha256
            )
        ):
            raise ResearchContractError("raw_sha256 is invalid")
        _integer(
            "byte_count",
            self.byte_count,
            1,
            MAX_RESPONSE_BYTES,
        )
        if (
            self.attribution_url != ATTRIBUTION_URL
            or self.license_url != LICENSE_URL
            or self.policy_version != PROVENANCE_POLICY_VERSION
        ):
            raise ResearchContractError(
                "evidence attribution policy is invalid"
            )

    @property
    def valid_until_unix_ms(self) -> int:
        return self.retrieved_at_unix_ms + self.ttl_ms

    def to_dict(self) -> Mapping[str, object]:
        return {
            "provider": self.provider,
            "source_kind": self.source_kind,
            "source_url": self.source_url,
            "retrieved_at_unix_ms": self.retrieved_at_unix_ms,
            "ttl_ms": self.ttl_ms,
            "valid_until_unix_ms": self.valid_until_unix_ms,
            "raw_sha256": self.raw_sha256,
            "byte_count": self.byte_count,
            "attribution_url": self.attribution_url,
            "license_url": self.license_url,
            "policy_version": self.policy_version,
        }


@dataclass(frozen=True)
class ResolvedLocation:
    location_id: int
    name: str
    latitude: float
    longitude: float
    elevation_m: float
    feature_code: str
    country_code: str
    country_name: str
    timezone: str
    administrative_area: Optional[str] = None

    def __post_init__(self) -> None:
        _integer("location_id", self.location_id, 1, 2**63 - 1)
        _display_text("name", self.name)
        _float("latitude", self.latitude, -90.0, 90.0)
        _float("longitude", self.longitude, -180.0, 180.0)
        _float("elevation_m", self.elevation_m)
        _identifier("feature_code", self.feature_code, 32)
        _country_code(self.country_code)
        _display_text("country_name", self.country_name)
        _identifier("timezone", self.timezone, 128)
        if self.administrative_area is not None:
            _display_text(
                "administrative_area",
                self.administrative_area,
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "location_id": self.location_id,
            "name": self.name,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "elevation_m": self.elevation_m,
            "feature_code": self.feature_code,
            "country_code": self.country_code,
            "country_name": self.country_name,
            "timezone": self.timezone,
            "administrative_area": self.administrative_area,
        }


@dataclass(frozen=True)
class CurrentWeather:
    observed_at: str
    interval_seconds: int
    grid_latitude: float
    grid_longitude: float
    grid_elevation_m: float
    temperature_c: float
    apparent_temperature_c: float
    precipitation_mm: float
    weather_code: int
    cloud_cover_percent: int
    wind_speed_kmh: float
    is_day: bool

    def __post_init__(self) -> None:
        _iso_minute(self.observed_at)
        _integer("interval_seconds", self.interval_seconds, 1, 86_400)
        _float("grid_latitude", self.grid_latitude, -90.0, 90.0)
        _float("grid_longitude", self.grid_longitude, -180.0, 180.0)
        _float("grid_elevation_m", self.grid_elevation_m)
        _float("temperature_c", self.temperature_c)
        _float("apparent_temperature_c", self.apparent_temperature_c)
        _float("precipitation_mm", self.precipitation_mm, 0.0)
        _integer("weather_code", self.weather_code, 0, 99)
        if self.weather_code not in _WMO_WEATHER_CODES:
            raise ResearchContractError("weather_code is invalid")
        _integer(
            "cloud_cover_percent",
            self.cloud_cover_percent,
            0,
            100,
        )
        _float("wind_speed_kmh", self.wind_speed_kmh, 0.0)
        if type(self.is_day) is not bool:
            raise ResearchContractError("is_day is invalid")

    def to_dict(self) -> Mapping[str, object]:
        return {
            "observed_at": self.observed_at,
            "interval_seconds": self.interval_seconds,
            "grid_latitude": self.grid_latitude,
            "grid_longitude": self.grid_longitude,
            "grid_elevation_m": self.grid_elevation_m,
            "temperature_c": self.temperature_c,
            "apparent_temperature_c": self.apparent_temperature_c,
            "precipitation_mm": self.precipitation_mm,
            "weather_code": self.weather_code,
            "cloud_cover_percent": self.cloud_cover_percent,
            "wind_speed_kmh": self.wind_speed_kmh,
            "is_day": self.is_day,
        }


@dataclass(frozen=True)
class LocationEvidence:
    request_id: str
    location: ResolvedLocation
    provenance: EvidenceProvenance

    def __post_init__(self) -> None:
        _identifier("request_id", self.request_id)
        if not isinstance(self.location, ResolvedLocation):
            raise ResearchContractError("location evidence is invalid")
        if (
            not isinstance(self.provenance, EvidenceProvenance)
            or self.provenance.source_kind != _GEOCODING_SOURCE
        ):
            raise ResearchContractError("location provenance is invalid")

    def to_dict(self) -> Mapping[str, object]:
        return {
            "request_id": self.request_id,
            "kind": _GEOCODING_SOURCE,
            "location": self.location.to_dict(),
            "provenance": self.provenance.to_dict(),
        }


@dataclass(frozen=True)
class WeatherEvidence:
    request_id: str
    weather: CurrentWeather
    provenance: EvidenceProvenance

    def __post_init__(self) -> None:
        _identifier("request_id", self.request_id)
        if not isinstance(self.weather, CurrentWeather):
            raise ResearchContractError("weather evidence is invalid")
        if (
            not isinstance(self.provenance, EvidenceProvenance)
            or self.provenance.source_kind != _CURRENT_WEATHER_SOURCE
        ):
            raise ResearchContractError("weather provenance is invalid")

    def to_dict(self) -> Mapping[str, object]:
        return {
            "request_id": self.request_id,
            "kind": _CURRENT_WEATHER_SOURCE,
            "weather": self.weather.to_dict(),
            "provenance": self.provenance.to_dict(),
        }


@dataclass(frozen=True)
class WeatherResearchResult:
    request: WeatherResearchRequest
    location_evidence: LocationEvidence
    weather_evidence: WeatherEvidence
    completed_at_monotonic_ms: int

    def __post_init__(self) -> None:
        if not isinstance(self.request, WeatherResearchRequest):
            raise ResearchContractError("result request is invalid")
        if (
            not isinstance(self.location_evidence, LocationEvidence)
            or not isinstance(self.weather_evidence, WeatherEvidence)
            or self.location_evidence.request_id != self.request.request_id
            or self.weather_evidence.request_id != self.request.request_id
        ):
            raise ResearchContractError("result evidence is invalid")
        _integer(
            "completed_at_monotonic_ms",
            self.completed_at_monotonic_ms,
            self.request.issued_at_monotonic_ms,
            self.request.valid_until_monotonic_ms - 1,
        )

    @property
    def request_id(self) -> str:
        return self.request.request_id

    @property
    def location(self) -> ResolvedLocation:
        return self.location_evidence.location

    @property
    def weather(self) -> CurrentWeather:
        return self.weather_evidence.weather

    @property
    def evidence(self) -> Tuple[LocationEvidence, WeatherEvidence]:
        return (self.location_evidence, self.weather_evidence)

    @property
    def valid_until_unix_ms(self) -> int:
        return min(
            self.location_evidence.provenance.valid_until_unix_ms,
            self.weather_evidence.provenance.valid_until_unix_ms,
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "request": self.request.to_dict(),
            "request_id": self.request_id,
            "location": self.location.to_dict(),
            "weather": self.weather.to_dict(),
            "evidence": [
                item.to_dict() for item in self.evidence
            ],
            "completed_at_monotonic_ms": self.completed_at_monotonic_ms,
            "valid_until_unix_ms": self.valid_until_unix_ms,
        }


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


def _finite_float_token(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("non-finite number")
    return result


def _decode_json(body: bytes) -> object:
    if not isinstance(body, bytes):
        raise ResearchProtocolError("Research response body was not bytes")
    if not body:
        raise ResearchProtocolError("Research response body was empty")
    if len(body) > MAX_RESPONSE_BYTES:
        raise ResearchResponseTooLarge("Research response was too large")
    try:
        return json.loads(
            body.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
            parse_float=_finite_float_token,
        )
    except (UnicodeDecodeError, TypeError, ValueError):
        raise ResearchProtocolError(
            "Research endpoint returned invalid JSON"
        ) from None


def _remote_object(
    value: object,
    required: frozenset,
    allowed: frozenset,
) -> Mapping[str, object]:
    if (
        not isinstance(value, dict)
        or not required.issubset(value)
        or not set(value).issubset(allowed)
    ):
        raise ResearchProtocolError(
            "Research endpoint returned invalid fields"
        )
    return value


def _wire_integer(value: object, minimum: int, maximum: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise ResearchProtocolError(
            "Research endpoint returned an invalid integer"
        )
    return value


def _wire_float(
    value: object,
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
    ):
        raise ResearchProtocolError(
            "Research endpoint returned an invalid number"
        )
    result = float(value)
    if (
        not math.isfinite(result)
        or minimum is not None
        and result < minimum
        or maximum is not None
        and result > maximum
    ):
        raise ResearchProtocolError(
            "Research endpoint returned an invalid number"
        )
    return result


def _wire_text(
    value: object,
    maximum: int = 200,
    allow_empty: bool = False,
) -> str:
    if (
        not isinstance(value, str)
        or not allow_empty
        and not value
        or value != value.strip()
        and not (allow_empty and value == "")
        or len(value) > maximum
        or value
        and not value.isprintable()
    ):
        raise ResearchProtocolError(
            "Research endpoint returned invalid text"
        )
    return value


_GEOCODING_RESULT_REQUIRED = frozenset(
    (
        "id",
        "name",
        "latitude",
        "longitude",
        "elevation",
        "feature_code",
        "country_code",
        "timezone",
        "country_id",
        "country",
    )
)
_GEOCODING_RESULT_ALLOWED = frozenset(
    (
        "id",
        "name",
        "latitude",
        "longitude",
        "elevation",
        "feature_code",
        "country_code",
        "admin1_id",
        "admin2_id",
        "admin3_id",
        "admin4_id",
        "timezone",
        "population",
        "postcodes",
        "country_id",
        "country",
        "admin1",
        "admin2",
        "admin3",
        "admin4",
    )
)


def _decode_location(body: bytes) -> ResolvedLocation:
    value = _remote_object(
        _decode_json(body),
        frozenset(("generationtime_ms",)),
        frozenset(("generationtime_ms", "results")),
    )
    _wire_float(value["generationtime_ms"], 0.0)
    results = value.get("results")
    if results is None or results == []:
        raise ResearchLocationNotFound(
            "Open-Meteo geocoding returned no location"
        )
    if not isinstance(results, list) or len(results) != 1:
        raise ResearchProtocolError(
            "Geocoding response must contain exactly one location"
        )
    location = _remote_object(
        results[0],
        _GEOCODING_RESULT_REQUIRED,
        _GEOCODING_RESULT_ALLOWED,
    )

    for field in (
        "admin1_id",
        "admin2_id",
        "admin3_id",
        "admin4_id",
        "population",
    ):
        if field in location:
            _wire_integer(location[field], 0, 2**63 - 1)
    for field in ("admin1", "admin2", "admin3", "admin4"):
        if field in location:
            _wire_text(location[field], allow_empty=True)
    if "postcodes" in location:
        postcodes = location["postcodes"]
        if (
            not isinstance(postcodes, list)
            or any(
                not isinstance(postcode, str)
                or not postcode
                or len(postcode) > 64
                or not postcode.isprintable()
                for postcode in postcodes
            )
        ):
            raise ResearchProtocolError(
                "Geocoding response contained invalid postcodes"
            )

    location_id = _wire_integer(location["id"], 1, 2**63 - 1)
    _wire_integer(location["country_id"], 1, 2**63 - 1)
    name = _wire_text(location["name"])
    feature_code = _wire_text(location["feature_code"], 32)
    country_code = _wire_text(location["country_code"], 2)
    country_name = _wire_text(location["country"])
    timezone = _wire_text(location["timezone"], 128)
    administrative_area = (
        _wire_text(location["admin1"])
        if location.get("admin1")
        else None
    )
    try:
        return ResolvedLocation(
            location_id=location_id,
            name=name,
            latitude=_wire_float(
                location["latitude"],
                -90.0,
                90.0,
            ),
            longitude=_wire_float(
                location["longitude"],
                -180.0,
                180.0,
            ),
            elevation_m=_wire_float(location["elevation"]),
            feature_code=feature_code,
            country_code=country_code,
            country_name=country_name,
            timezone=timezone,
            administrative_area=administrative_area,
        )
    except ResearchContractError:
        raise ResearchProtocolError(
            "Geocoding response contained invalid location data"
        ) from None


_WEATHER_TOP_FIELDS = frozenset(
    (
        "latitude",
        "longitude",
        "generationtime_ms",
        "utc_offset_seconds",
        "timezone",
        "timezone_abbreviation",
        "elevation",
        "current_units",
        "current",
    )
)
_CURRENT_FIELDS = frozenset(("time", "interval") + _CURRENT_VARIABLES)
_EXPECTED_CURRENT_UNITS = {
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


def _decode_weather(body: bytes) -> CurrentWeather:
    value = _remote_object(
        _decode_json(body),
        _WEATHER_TOP_FIELDS,
        _WEATHER_TOP_FIELDS,
    )
    _wire_float(value["generationtime_ms"], 0.0)
    _wire_integer(value["utc_offset_seconds"], 0, 0)
    _wire_text(value["timezone"], 64)
    _wire_text(value["timezone_abbreviation"], 32)

    units = _remote_object(
        value["current_units"],
        frozenset(_EXPECTED_CURRENT_UNITS),
        frozenset(_EXPECTED_CURRENT_UNITS),
    )
    if units != _EXPECTED_CURRENT_UNITS:
        raise ResearchProtocolError(
            "Weather response units were not the requested units"
        )

    current = _remote_object(
        value["current"],
        _CURRENT_FIELDS,
        _CURRENT_FIELDS,
    )
    observed_at = _wire_text(current["time"], 32)
    interval_seconds = _wire_integer(
        current["interval"],
        1,
        86_400,
    )
    weather_code = _wire_integer(current["weather_code"], 0, 99)
    cloud_cover = _wire_integer(current["cloud_cover"], 0, 100)
    is_day_integer = _wire_integer(current["is_day"], 0, 1)

    try:
        return CurrentWeather(
            observed_at=observed_at,
            interval_seconds=interval_seconds,
            grid_latitude=_wire_float(
                value["latitude"],
                -90.0,
                90.0,
            ),
            grid_longitude=_wire_float(
                value["longitude"],
                -180.0,
                180.0,
            ),
            grid_elevation_m=_wire_float(value["elevation"]),
            temperature_c=_wire_float(current["temperature_2m"]),
            apparent_temperature_c=_wire_float(
                current["apparent_temperature"]
            ),
            precipitation_mm=_wire_float(
                current["precipitation"],
                0.0,
            ),
            weather_code=weather_code,
            cloud_cover_percent=cloud_cover,
            wind_speed_kmh=_wire_float(
                current["wind_speed_10m"],
                0.0,
            ),
            is_day=bool(is_day_integer),
        )
    except ResearchContractError:
        raise ResearchProtocolError(
            "Weather response contained invalid current conditions"
        ) from None


def _stdlib_get(
    url: str,
    headers: Mapping[str, str],
    timeout_seconds: float,
    max_response_bytes: int,
) -> bytes:
    try:
        response = direct_http_request(
            "GET",
            url,
            headers,
            None,
            timeout_seconds,
            max_response_bytes,
        )
    except DirectHTTPTimeoutError:
        raise ResearchDeadlineExceeded(
            "Research endpoint request timed out"
        ) from None
    except DirectHTTPTransportError:
        raise ResearchTransportError(
            "Research endpoint request failed"
        ) from None
    if not 200 <= response.status_code < 300:
        raise ResearchHTTPError(response.status_code)

    content_type_values = response.header_values("Content-Type")
    if len(content_type_values) != 1:
        raise ResearchProtocolError(
            "Research endpoint returned invalid HTTP metadata"
        )
    metadata = Message()
    metadata["Content-Type"] = content_type_values[0]
    content_type = metadata.get_content_type()
    charset = metadata.get_content_charset()
    if (
        content_type != "application/json"
        or charset is not None
        and charset.lower() != "utf-8"
    ):
        raise ResearchProtocolError(
            "Research endpoint returned a non-JSON response"
        )

    body = response.body
    if len(body) > max_response_bytes:
        raise ResearchResponseTooLarge("Research response was too large")
    return body


def _default_monotonic_ms() -> int:
    return int(time.monotonic() * 1_000)


def _default_wall_clock_ms() -> int:
    return int(time.time() * 1_000)


def _coordinate(value: float) -> str:
    if value == 0.0:
        return "0"
    return "{:.6f}".format(value).rstrip("0").rstrip(".")


def _geocoding_url(query: str) -> str:
    return GEOCODING_ENDPOINT + "?" + urlencode(
        (
            ("name", query),
            ("count", "1"),
            ("language", "en"),
            ("format", "json"),
        )
    )


def _weather_url(location: ResolvedLocation) -> str:
    return CURRENT_WEATHER_ENDPOINT + "?" + urlencode(
        (
            ("latitude", _coordinate(location.latitude)),
            ("longitude", _coordinate(location.longitude)),
            ("current", ",".join(_CURRENT_VARIABLES)),
            ("temperature_unit", "celsius"),
            ("wind_speed_unit", "kmh"),
            ("precipitation_unit", "mm"),
            ("timeformat", "iso8601"),
            ("timezone", "UTC"),
        )
    )


class WeatherTool(ABC):
    """Typed boundary exposed to an agent coordinator."""

    @abstractmethod
    def current(
        self,
        request: WeatherResearchRequest,
    ) -> WeatherResearchResult:
        raise NotImplementedError


class OpenMeteoWeatherTool(WeatherTool):
    """Two-call current-weather tool with fixed provider endpoints."""

    def __init__(
        self,
        transport: HTTPTransport = _stdlib_get,
        monotonic_clock_ms: MillisecondClock = _default_monotonic_ms,
        wall_clock_ms: MillisecondClock = _default_wall_clock_ms,
        request_timeout_seconds: float = REQUEST_TIMEOUT_SECONDS,
    ):
        if (
            not callable(transport)
            or not callable(monotonic_clock_ms)
            or not callable(wall_clock_ms)
            or isinstance(request_timeout_seconds, bool)
            or not isinstance(request_timeout_seconds, (int, float))
            or not math.isfinite(float(request_timeout_seconds))
            or not 0.0 < float(request_timeout_seconds) <= 30.0
        ):
            raise ResearchConfigurationError(
                "Research tool configuration is invalid"
            )
        self._transport = transport
        self._monotonic_clock_ms = monotonic_clock_ms
        self._wall_clock_ms = wall_clock_ms
        self._request_timeout_seconds = float(request_timeout_seconds)

    def _monotonic_now(self) -> int:
        value = self._monotonic_clock_ms()
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 0 <= value <= 2**63 - 1
        ):
            raise ResearchClockError("Monotonic clock returned invalid time")
        return value

    def _wall_now(self) -> int:
        value = self._wall_clock_ms()
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 0 <= value <= 2**63 - 1
        ):
            raise ResearchClockError("Wall clock returned invalid time")
        return value

    def _check_deadline(
        self,
        request: WeatherResearchRequest,
        previous_monotonic_ms: int,
    ) -> int:
        now_ms = self._monotonic_now()
        if now_ms < previous_monotonic_ms:
            raise ResearchClockError("Monotonic clock regressed")
        if now_ms >= request.valid_until_monotonic_ms:
            raise ResearchDeadlineExceeded(
                "Research request deadline expired"
            )
        return now_ms

    def _fetch(
        self,
        source_kind: str,
        url: str,
        request: WeatherResearchRequest,
        previous_monotonic_ms: int,
    ) -> Tuple[bytes, int, EvidenceProvenance]:
        _validate_source_url(source_kind, url)
        started_ms = self._check_deadline(
            request,
            previous_monotonic_ms,
        )
        remaining_seconds = (
            request.valid_until_monotonic_ms - started_ms
        ) / 1_000.0
        timeout_seconds = min(
            self._request_timeout_seconds,
            remaining_seconds,
        )
        try:
            body = self._transport(
                url,
                {
                    "Accept": "application/json",
                    "User-Agent": "robot-llm-research/1",
                },
                timeout_seconds,
                MAX_RESPONSE_BYTES,
            )
        except ResearchError:
            raise
        except (socket.timeout, TimeoutError):
            raise ResearchDeadlineExceeded(
                "Research endpoint request timed out"
            ) from None
        except OSError:
            raise ResearchTransportError(
                "Research endpoint request failed"
            ) from None
        completed_ms = self._check_deadline(request, started_ms)
        if not isinstance(body, bytes):
            raise ResearchProtocolError(
                "Research response body was not bytes"
            )
        if not body:
            raise ResearchProtocolError(
                "Research response body was empty"
            )
        if len(body) > MAX_RESPONSE_BYTES:
            raise ResearchResponseTooLarge(
                "Research response was too large"
            )
        retrieved_at_unix_ms = self._wall_now()
        ttl_ms = (
            GEOCODING_EVIDENCE_TTL_MS
            if source_kind == _GEOCODING_SOURCE
            else WEATHER_EVIDENCE_TTL_MS
        )
        provenance = EvidenceProvenance(
            provider=_PROVIDER,
            source_kind=source_kind,
            source_url=url,
            retrieved_at_unix_ms=retrieved_at_unix_ms,
            ttl_ms=ttl_ms,
            raw_sha256=hashlib.sha256(body).hexdigest(),
            byte_count=len(body),
            attribution_url=ATTRIBUTION_URL,
            license_url=LICENSE_URL,
            policy_version=PROVENANCE_POLICY_VERSION,
        )
        return body, completed_ms, provenance

    def current(
        self,
        request: WeatherResearchRequest,
    ) -> WeatherResearchResult:
        if not isinstance(request, WeatherResearchRequest):
            raise ResearchContractError("Weather request is invalid")
        initial_ms = self._monotonic_now()
        if initial_ms < request.issued_at_monotonic_ms:
            raise ResearchContractError(
                "Weather request was issued in the future"
            )
        if initial_ms >= request.valid_until_monotonic_ms:
            raise ResearchDeadlineExceeded(
                "Research request deadline expired"
            )

        location_body, last_ms, location_provenance = self._fetch(
            _GEOCODING_SOURCE,
            _geocoding_url(request.location_query),
            request,
            initial_ms,
        )
        location = _decode_location(location_body)
        last_ms = self._check_deadline(request, last_ms)

        weather_body, last_ms, weather_provenance = self._fetch(
            _CURRENT_WEATHER_SOURCE,
            _weather_url(location),
            request,
            last_ms,
        )
        weather = _decode_weather(weather_body)
        completed_ms = self._check_deadline(request, last_ms)

        return WeatherResearchResult(
            request=request,
            location_evidence=LocationEvidence(
                request_id=request.request_id,
                location=location,
                provenance=location_provenance,
            ),
            weather_evidence=WeatherEvidence(
                request_id=request.request_id,
                weather=weather,
                provenance=weather_provenance,
            ),
            completed_at_monotonic_ms=completed_ms,
        )
