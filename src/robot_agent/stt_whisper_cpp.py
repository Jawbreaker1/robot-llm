"""Strict, loopback-only adapter for a warm whisper.cpp HTTP server."""

from __future__ import annotations

import ipaddress
import json
import math
import secrets
import socket
from typing import Callable, Mapping
from urllib.parse import urlsplit

from .http_transport import (
    DirectHTTPTimeoutError,
    DirectHTTPTransportError,
    direct_http_request,
)
from .stt_contract import (
    ProviderTranscription,
    STTContractError,
    TranscriptionRequest,
)
from .stt_provider import (
    STTProviderProtocolError,
    STTProviderTimeoutError,
    STTProviderUnavailableError,
)


DEFAULT_WHISPER_CPP_URL = "http://127.0.0.1:8178"
DEFAULT_STT_TIMEOUT_SECONDS = 12.0
MAX_STT_PROVIDER_RESPONSE_BYTES = 64 * 1024
MIN_OPAQUE_PATH_CHARACTERS = 22
MAX_OPAQUE_PATH_CHARACTERS = 128

Transport = Callable[..., object]


def _safe_opaque_path(path: str) -> str:
    if path in ("", "/"):
        return ""
    segment = path[1:] if path.startswith("/") else ""
    if (
        not segment
        or "/" in segment
        or not MIN_OPAQUE_PATH_CHARACTERS
        <= len(segment)
        <= MAX_OPAQUE_PATH_CHARACTERS
        or not segment.isascii()
        or not all(
            character.isalnum() or character in "-_"
            for character in segment
        )
    ):
        raise ValueError("Whisper server URL path is invalid")
    return "/" + segment


def _safe_loopback_base_url(value: str) -> str:
    if not isinstance(value, str):
        raise ValueError("Whisper server URL is invalid")
    try:
        parsed = urlsplit(value)
        hostname = parsed.hostname
        parsed.port
    except ValueError:
        raise ValueError("Whisper server URL is invalid") from None
    if (
        parsed.scheme != "http"
        or not parsed.netloc
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
        or hostname is None
    ):
        raise ValueError("Whisper server URL is invalid")
    if hostname.lower() != "localhost":
        try:
            address = ipaddress.ip_address(hostname)
        except ValueError:
            raise ValueError(
                "Whisper server must use a loopback host"
            ) from None
        if not address.is_loopback:
            raise ValueError(
                "Whisper server must use a loopback host"
            )
    safe_path = _safe_opaque_path(parsed.path)
    return "{}://{}{}".format(
        parsed.scheme,
        parsed.netloc,
        safe_path,
    )


def _raise_for_provider_status(status, operation: str) -> None:
    if not isinstance(status, int) or isinstance(status, bool):
        raise STTProviderProtocolError(
            "Speech provider returned an invalid HTTP status"
        )
    if 400 <= status < 500:
        raise STTProviderProtocolError(
            "Speech provider rejected the {}".format(operation)
        )
    if 500 <= status < 600:
        raise STTProviderUnavailableError(
            "Speech provider is unavailable"
        )
    raise STTProviderProtocolError(
        "Speech provider returned an unexpected HTTP status"
    )


def _safe_display_id(name: str, value: str, maximum: int = 200) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or not value.isascii()
        or not all(
            character.isalnum() or character in "-_."
            for character in value
        )
    ):
        raise ValueError("{} is invalid".format(name))
    return value


def _multipart(
    request: TranscriptionRequest,
    boundary: str,
) -> bytes:
    if (
        not isinstance(boundary, str)
        or not boundary
        or not boundary.isascii()
        or not boundary.isalnum()
    ):
        raise ValueError("Multipart boundary is invalid")

    marker = ("--" + boundary).encode("ascii")
    chunks = []

    def field(name: str, value: str) -> None:
        chunks.extend(
            (
                marker,
                (
                    'Content-Disposition: form-data; name="{}"'.format(
                        name
                    )
                ).encode("ascii"),
                b"",
                value.encode("ascii"),
            )
        )

    chunks.extend(
        (
            marker,
            (
                b'Content-Disposition: form-data; name="file"; '
                b'filename="utterance.wav"'
            ),
            b"Content-Type: audio/wav",
            b"",
            request.audio.wav_bytes,
        )
    )
    field("response_format", "json")
    field("language", request.language_hint)
    field("temperature", "0")
    field("temperature_inc", "0")
    field("beam_size", "1")
    field("best_of", "1")
    field("no_timestamps", "true")
    field("translate", "false")
    chunks.extend((marker + b"--", b""))
    return b"\r\n".join(chunks)


def _strict_json_object(raw: bytes) -> Mapping[str, object]:
    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > MAX_STT_PROVIDER_RESPONSE_BYTES
    ):
        raise STTProviderProtocolError(
            "Speech provider response size is invalid"
        )

    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise ValueError("duplicate key")
            result[key] = value
        return result

    def constant(_value):
        raise ValueError("non-finite number")

    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=pairs,
            parse_constant=constant,
        )
    except (UnicodeDecodeError, ValueError, TypeError):
        raise STTProviderProtocolError(
            "Speech provider returned invalid JSON"
        ) from None
    if not isinstance(value, dict):
        raise STTProviderProtocolError(
            "Speech provider response is not an object"
        )
    return value


class WhisperCppTranscriber:
    """Transcribe short WAV utterances through a persistent local server."""

    provider_id = "whisper.cpp"

    def __init__(
        self,
        *,
        base_url: str = DEFAULT_WHISPER_CPP_URL,
        model_id: str = "whisper-multilingual",
        timeout_seconds: float = DEFAULT_STT_TIMEOUT_SECONDS,
        transport: Transport = direct_http_request,
        boundary_factory=lambda: secrets.token_hex(16),
        require_opaque_path: bool = False,
    ):
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not math.isfinite(float(timeout_seconds))
            or not 0 < float(timeout_seconds) <= 60
            or not callable(transport)
            or not callable(boundary_factory)
            or not isinstance(require_opaque_path, bool)
        ):
            raise ValueError("Whisper transcriber configuration is invalid")
        self._base_url = _safe_loopback_base_url(base_url)
        if (
            require_opaque_path
            and not urlsplit(self._base_url).path
        ):
            raise ValueError(
                "Whisper server URL requires a private request path"
            )
        self.model_id = _safe_display_id("model_id", model_id)
        self._timeout_seconds = float(timeout_seconds)
        self._transport = transport
        self._boundary_factory = boundary_factory

    @property
    def base_url(self) -> str:
        return self._base_url

    def probe(self) -> Mapping[str, object]:
        try:
            response = self._transport(
                "GET",
                self._base_url + "/health",
                {"Accept": "application/json"},
                None,
                min(self._timeout_seconds, 2.0),
                4 * 1024,
            )
        except DirectHTTPTimeoutError:
            raise STTProviderTimeoutError(
                "Speech provider probe timed out"
            ) from None
        except (DirectHTTPTransportError, OSError, socket.timeout):
            raise STTProviderUnavailableError(
                "Speech provider is unavailable"
            ) from None
        status = getattr(response, "status_code", None)
        if status != 200:
            _raise_for_provider_status(status, "readiness probe")
        value = _strict_json_object(getattr(response, "body", b""))
        if value != {"status": "ok"}:
            raise STTProviderProtocolError(
                "Speech provider readiness response is invalid"
            )
        return {
            "state": "online",
            "provider_id": self.provider_id,
            "model_id": self.model_id,
        }

    def transcribe(
        self,
        request: TranscriptionRequest,
    ) -> ProviderTranscription:
        if not isinstance(request, TranscriptionRequest):
            raise STTContractError(
                "invalid_stt_request",
                "Speech transcription request is invalid",
            )
        boundary = self._boundary_factory()
        body = _multipart(request, boundary)
        headers = {
            "Accept": "application/json",
            "Content-Type": (
                "multipart/form-data; boundary={}".format(boundary)
            ),
        }
        try:
            response = self._transport(
                "POST",
                self._base_url + "/inference",
                headers,
                body,
                self._timeout_seconds,
                MAX_STT_PROVIDER_RESPONSE_BYTES,
            )
        except DirectHTTPTimeoutError:
            raise STTProviderTimeoutError(
                "Speech transcription timed out"
            ) from None
        except (DirectHTTPTransportError, OSError, socket.timeout):
            raise STTProviderUnavailableError(
                "Speech provider is unavailable"
            ) from None
        status = getattr(response, "status_code", None)
        if not isinstance(status, int) or not 200 <= status < 300:
            _raise_for_provider_status(status, "transcription")
        raw = getattr(response, "body", b"")
        if len(raw) > MAX_STT_PROVIDER_RESPONSE_BYTES:
            raise STTProviderProtocolError(
                "Speech provider response was too large"
            )
        value = _strict_json_object(raw)
        text = value.get("text")
        if not isinstance(text, str) or not text.strip():
            raise STTProviderProtocolError(
                "Speech provider returned no transcript"
            )
        try:
            return ProviderTranscription(
                text=text.strip(),
                provider_id=self.provider_id,
                model_id=self.model_id,
            )
        except STTContractError:
            raise STTProviderProtocolError(
                "Speech provider transcript is invalid"
            ) from None
