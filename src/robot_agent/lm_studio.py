"""Small, fail-closed client for LM Studio's native chat endpoint.

The client deliberately exposes one operation and no tools.  Model output is
returned as untrusted data for the shadow loop to validate; it never carries
motor authority.
"""

from dataclasses import dataclass
import ipaddress
import json
import socket
import time
from typing import Callable, Mapping, Optional
from urllib.parse import urlsplit

from .commentary import FALLBACK_PHRASES, ProximityObservation
from .http_transport import (
    DirectHTTPTimeoutError,
    DirectHTTPTransportError,
    direct_http_request,
)


DEFAULT_BASE_URL = "http://127.0.0.1:1234"
DEFAULT_MODEL = "qwen/qwen3.8-27b"
CHAT_PATH = "/api/v1/chat"
REQUEST_TIMEOUT_SECONDS = 3.0
MAX_RESPONSE_BYTES = 64 * 1024
MAX_OUTPUT_TOKENS = 48

_SYSTEM_PROMPT = (
    "Du är den griniga men harmlösa LEGO-roboten EV3RSTORM. "
    "Svara med exakt en kort svensk replik, högst 80 tecken och 14 ord. "
    "Kommentera bara den angivna relativa IR-zonen. Hitta aldrig på ett "
    "exakt avstånd eller vilket föremål som finns där. Ingen markdown."
)

Transport = Callable[[str, bytes, Mapping[str, str], float, int], bytes]
Clock = Callable[[], float]


class LMStudioError(RuntimeError):
    """Base class for expected, safely reportable LM Studio failures."""


class LMStudioConfigurationError(LMStudioError):
    """The client was configured with an unsafe or invalid value."""


class LMStudioInputError(LMStudioError):
    """An observation cannot be represented by the fixed prompt."""


class LMStudioTransportError(LMStudioError):
    """The local server could not be reached or read."""


class LMStudioTimeoutError(LMStudioTransportError):
    """The fixed request deadline expired."""


class LMStudioHTTPError(LMStudioTransportError):
    """The local server returned a non-success HTTP status."""

    def __init__(self, status_code: Optional[int]):
        self.status_code = status_code
        if isinstance(status_code, int):
            message = "LM Studio returned HTTP status {}".format(status_code)
        else:
            message = "LM Studio returned an HTTP error"
        super().__init__(message)


class LMStudioResponseTooLargeError(LMStudioError):
    """The response exceeded the pre-parse body limit."""


class LMStudioProtocolError(LMStudioError):
    """The response did not satisfy the strict shadow-loop contract."""


@dataclass(frozen=True)
class ModelCandidate:
    """Untrusted language proposed by the fixed local model."""

    text: str
    latency_ms: int
    model_instance_id: str


def _safe_base_url(value: str) -> str:
    if not isinstance(value, str):
        raise LMStudioConfigurationError("LM Studio base URL is invalid")

    try:
        parsed = urlsplit(value)
        hostname = parsed.hostname
        # Accessing ``port`` validates malformed and out-of-range ports.
        parsed.port
    except ValueError:
        raise LMStudioConfigurationError("LM Studio base URL is invalid") from None

    if (
        parsed.scheme not in ("http", "https")
        or not parsed.netloc
        or parsed.username is not None
        or parsed.password is not None
        or parsed.path not in ("", "/")
        or parsed.query
        or parsed.fragment
        or hostname is None
    ):
        raise LMStudioConfigurationError("LM Studio base URL is invalid")

    if hostname.lower() != "localhost":
        try:
            address = ipaddress.ip_address(hostname)
        except ValueError:
            raise LMStudioConfigurationError(
                "LM Studio base URL must use a loopback host"
            ) from None
        if not address.is_loopback:
            raise LMStudioConfigurationError(
                "LM Studio base URL must use a loopback host"
            )

    return value.rstrip("/")


def _safe_model(value: str) -> str:
    if (
        not isinstance(value, str)
        or not value.strip()
        or value != value.strip()
        or len(value) > 200
        or any(ord(character) < 32 for character in value)
    ):
        raise LMStudioConfigurationError("LM Studio model identifier is invalid")
    return value


def _stdlib_post(
    url: str,
    body: bytes,
    headers: Mapping[str, str],
    timeout_seconds: float,
    max_response_bytes: int,
) -> bytes:
    try:
        response = direct_http_request(
            "POST",
            url,
            headers,
            body,
            timeout_seconds,
            max_response_bytes,
        )
    except DirectHTTPTimeoutError:
        raise LMStudioTimeoutError("LM Studio request timed out") from None
    except DirectHTTPTransportError:
        raise LMStudioTransportError("LM Studio request failed") from None
    if not 200 <= response.status_code < 300:
        raise LMStudioHTTPError(response.status_code)

    result = response.body
    if len(result) > max_response_bytes:
        raise LMStudioResponseTooLargeError("LM Studio response was too large")
    return result


def _decode_response(body: bytes) -> tuple[str, str]:
    if not isinstance(body, bytes):
        raise LMStudioProtocolError("LM Studio response body was not bytes")
    if len(body) > MAX_RESPONSE_BYTES:
        raise LMStudioResponseTooLargeError("LM Studio response was too large")

    def reject_constant(_: str) -> None:
        raise ValueError

    try:
        decoded = json.loads(
            body.decode("utf-8"),
            parse_constant=reject_constant,
        )
    except (UnicodeDecodeError, ValueError, TypeError):
        raise LMStudioProtocolError("LM Studio returned invalid JSON") from None

    if not isinstance(decoded, dict):
        raise LMStudioProtocolError("LM Studio response was not an object")
    if "response_id" in decoded:
        raise LMStudioProtocolError("LM Studio unexpectedly stored the response")

    model_instance_id = decoded.get("model_instance_id")
    if not isinstance(model_instance_id, str) or not model_instance_id.strip():
        raise LMStudioProtocolError(
            "LM Studio response had no model instance identifier"
        )

    output = decoded.get("output")
    if not isinstance(output, list) or len(output) != 1:
        raise LMStudioProtocolError(
            "LM Studio response must contain exactly one output item"
        )
    message = output[0]
    if not isinstance(message, dict) or message.get("type") != "message":
        raise LMStudioProtocolError(
            "LM Studio response must contain exactly one message"
        )
    content = message.get("content")
    if not isinstance(content, str) or not content.strip():
        raise LMStudioProtocolError("LM Studio message was empty")

    stats = decoded.get("stats")
    if not isinstance(stats, dict):
        raise LMStudioProtocolError("LM Studio response had no token statistics")
    reasoning_tokens = stats.get("reasoning_output_tokens")
    if (
        isinstance(reasoning_tokens, bool)
        or not isinstance(reasoning_tokens, int)
        or reasoning_tokens != 0
    ):
        raise LMStudioProtocolError(
            "LM Studio produced or omitted reasoning tokens"
        )

    return content, model_instance_id


class NativeLMStudioClient:
    """Stateless, tool-free LM Studio client with a model fixed at construction."""

    def __init__(
        self,
        base_url: str = DEFAULT_BASE_URL,
        model: str = DEFAULT_MODEL,
        transport: Transport = _stdlib_post,
        clock: Clock = time.monotonic,
    ):
        if not callable(transport) or not callable(clock):
            raise LMStudioConfigurationError("LM Studio client dependency is invalid")
        self._base_url = _safe_base_url(base_url)
        self._model = _safe_model(model)
        self._transport = transport
        self._clock = clock

    @property
    def model(self) -> str:
        return self._model

    def comment(self, observation: ProximityObservation) -> ModelCandidate:
        try:
            zone = observation.zone
            filtered_percent = observation.filtered_percent
        except AttributeError:
            raise LMStudioInputError("IR observation is invalid") from None

        if zone not in FALLBACK_PHRASES:
            raise LMStudioInputError("IR observation zone is invalid")
        if (
            isinstance(filtered_percent, bool)
            or not isinstance(filtered_percent, int)
            or not 0 <= filtered_percent <= 100
        ):
            raise LMStudioInputError("IR observation value is invalid")

        user_input = (
            "IR-zon={}; relativt_reflektionsvärde={}. "
            "Skriv endast repliken."
        ).format(zone, filtered_percent)
        payload = {
            "model": self._model,
            "input": user_input,
            "system_prompt": _SYSTEM_PROMPT,
            "reasoning": "off",
            "store": False,
            "stream": False,
            "integrations": [],
            "temperature": 0,
            "max_output_tokens": MAX_OUTPUT_TOKENS,
        }
        body = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        headers = {
            "Accept": "application/json",
            "Content-Type": "application/json; charset=utf-8",
        }

        started = self._clock()
        try:
            response_body = self._transport(
                self._base_url + CHAT_PATH,
                body,
                headers,
                REQUEST_TIMEOUT_SECONDS,
                MAX_RESPONSE_BYTES,
            )
        except LMStudioError:
            raise
        except (socket.timeout, TimeoutError):
            raise LMStudioTimeoutError("LM Studio request timed out") from None
        except OSError:
            raise LMStudioTransportError("LM Studio request failed") from None
        completed = self._clock()
        elapsed_seconds = max(0.0, completed - started)
        if elapsed_seconds > REQUEST_TIMEOUT_SECONDS:
            raise LMStudioTimeoutError("LM Studio request timed out")

        text, model_instance_id = _decode_response(response_body)
        latency_ms = int(round(elapsed_seconds * 1000))
        return ModelCandidate(
            text=text,
            latency_ms=latency_ms,
            model_instance_id=model_instance_id,
        )
