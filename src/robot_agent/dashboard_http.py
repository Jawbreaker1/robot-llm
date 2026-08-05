"""Loopback-only HTTP boundary for the Robot LLM dashboard.

The router is deliberately independent from sockets so its complete security
surface can be exercised in unit tests.  The thin ``BaseHTTPRequestHandler``
adapter owns framing only; it never interprets natural language.  Physical
control routes, when configured, are delegated to a separate typed router.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import secrets
from typing import Dict, Mapping, Optional, Tuple
from urllib.parse import parse_qs, urlsplit

from .dashboard_contract import DashboardContractError, strict_json_loads
from .stt_contract import MAX_STT_AUDIO_BYTES


API_VERSION = "robot-dashboard/v1"
MAX_REQUEST_BYTES = 16 * 1024
MAX_EVENT_LIMIT = 500
MAX_SPATIAL_MAP_RESPONSE_BYTES = 4 * 1024 * 1024
TOKEN_HEADER = "x-robot-dashboard-token"
TOKEN_PLACEHOLDER = b"__ROBOT_DASHBOARD_TOKEN__"
STT_REQUEST_ID_HEADER = "x-robot-stt-request-id"
STT_LANGUAGE_HEADER = "x-robot-stt-language"
STT_TRANSCRIPTIONS_PATH = "/api/v1/stt/transcriptions"
STT_REQUESTS_PATH = "/api/v1/stt/requests"


class DashboardHTTPError(RuntimeError):
    """Safely reportable HTTP boundary failure."""

    def __init__(self, status: int, code: str, message: str):
        self.status = status
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class DashboardHTTPResponse:
    status: int
    headers: Tuple[Tuple[str, str], ...]
    body: bytes


def _json_bytes(value: Mapping[str, object]) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _lower_headers(headers: Mapping[str, str]) -> Dict[str, str]:
    lowered = {}
    for raw_name, raw_value in headers.items():
        if not isinstance(raw_name, str) or not isinstance(raw_value, str):
            raise DashboardHTTPError(
                400,
                "invalid_headers",
                "Request headers are invalid",
            )
        name = raw_name.lower()
        if (
            not name
            or name in lowered
            or "\r" in name
            or "\n" in name
            or "\r" in raw_value
            or "\n" in raw_value
        ):
            raise DashboardHTTPError(
                400,
                "invalid_headers",
                "Request headers are invalid",
            )
        lowered[name] = raw_value
    return lowered


def _integer_query(
    query: Mapping[str, object],
    name: str,
    default: int,
    minimum: int,
    maximum: int,
) -> int:
    values = query.get(name)
    if values is None:
        return default
    if (
        not isinstance(values, list)
        or len(values) != 1
        or not isinstance(values[0], str)
        or not values[0]
        or not values[0].isascii()
        or not values[0].isdigit()
    ):
        raise DashboardHTTPError(
            400,
            "invalid_query",
            "Query parameter is invalid",
        )
    value = int(values[0])
    if not minimum <= value <= maximum:
        raise DashboardHTTPError(
            400,
            "invalid_query",
            "Query parameter is invalid",
        )
    return value


def _exact_object(
    value: object,
    required,
    optional=(),
) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise DashboardHTTPError(
            400,
            "invalid_request",
            "JSON body must be an object",
        )
    keys = set(value)
    required_keys = set(required)
    allowed = required_keys | set(optional)
    if not required_keys <= keys or not keys <= allowed:
        raise DashboardHTTPError(
            400,
            "invalid_request_fields",
            "JSON body fields are invalid",
        )
    return value


class DashboardRouter:
    """Exact-route, exact-method router for one local dashboard instance."""

    _STATIC_ROUTES = {
        "assets/styles.css": ("styles.css", "text/css; charset=utf-8"),
        "assets/i18n.js": (
            "i18n.js",
            "text/javascript; charset=utf-8",
        ),
        "assets/dashboard_logic.js": (
            "dashboard_logic.js",
            "text/javascript; charset=utf-8",
        ),
        "assets/controller_panel.js": (
            "controller_panel.js",
            "text/javascript; charset=utf-8",
        ),
        "assets/spatial_map_presenter.js": (
            "spatial_map_presenter.js",
            "text/javascript; charset=utf-8",
        ),
        "assets/robot_mission_panel.js": (
            "robot_mission_panel.js",
            "text/javascript; charset=utf-8",
        ),
        "assets/robot_control.js": (
            "robot_control.js",
            "text/javascript; charset=utf-8",
        ),
        "assets/speech_input_logic.js": (
            "speech_input_logic.js",
            "text/javascript; charset=utf-8",
        ),
        "assets/microphone_input.js": (
            "microphone_input.js",
            "text/javascript; charset=utf-8",
        ),
        "assets/pcm_capture_worklet.js": (
            "pcm_capture_worklet.js",
            "text/javascript; charset=utf-8",
        ),
        "assets/app.js": (
            "app.js",
            "text/javascript; charset=utf-8",
        ),
        "assets/robot-llm-mascot.png": (
            "robot-llm-mascot.png",
            "image/png",
        ),
        "assets/robot-llm-head.png": (
            "robot-llm-head.png",
            "image/png",
        ),
    }

    def __init__(
        self,
        service,
        session_token: str,
        expected_host: str,
        web_root: Optional[Path] = None,
        robot_control_router=None,
    ):
        if (
            service is None
            or not isinstance(session_token, str)
            or len(session_token) < 32
            or len(session_token) > 128
            or not session_token.isascii()
            or not all(
                character.isalnum() or character in "-_"
                for character in session_token
            )
            or not isinstance(expected_host, str)
            or not expected_host
        ):
            raise ValueError("Dashboard router configuration is invalid")
        self._service = service
        self._session_token = session_token
        self._access_path = "/live/{}/".format(session_token)
        self._legacy_session_path = "/session/{}/".format(session_token)
        self._expected_host = expected_host
        self._expected_origin = "http://" + expected_host
        self._robot_control_router = robot_control_router
        self._web_root = (
            Path(web_root)
            if web_root is not None
            else Path(__file__).with_name("dashboard_web")
        )
        self._assets = self._load_assets()

    @property
    def access_path(self) -> str:
        """Canonical capability-bearing path for the live console."""

        return self._access_path

    @property
    def session_path(self) -> str:
        """Backward-compatible name for the canonical live-console path."""

        return self._access_path

    def _load_assets(self):
        assets = {}
        index = (self._web_root / "index.html").read_bytes()
        if index.count(TOKEN_PLACEHOLDER) != 1:
            raise ValueError(
                "Dashboard index must contain one access-key placeholder"
            )
        assets[self._access_path] = (
            index.replace(
                TOKEN_PLACEHOLDER,
                self._session_token.encode("ascii"),
            ),
            "text/html; charset=utf-8",
        )
        for route, (filename, content_type) in self._STATIC_ROUTES.items():
            path = self._web_root / filename
            if not path.is_file():
                if filename.encode("utf-8") in index:
                    raise ValueError(
                        "Dashboard index references a missing asset"
                    )
                continue
            assets[self._access_path + route] = (
                path.read_bytes(),
                content_type,
            )
        return assets

    def _legacy_location(self, path: str) -> Optional[str]:
        """Map an allowlisted legacy session path to its live URL."""

        if path == self._legacy_session_path:
            return self._access_path
        for route in self._STATIC_ROUTES:
            if path == self._legacy_session_path + route:
                return self._access_path + route
        return None

    @staticmethod
    def _security_headers(content_type: str):
        return (
            ("Content-Type", content_type),
            ("Cache-Control", "no-store, max-age=0"),
            ("Pragma", "no-cache"),
            ("X-Content-Type-Options", "nosniff"),
            ("X-Frame-Options", "DENY"),
            ("Referrer-Policy", "no-referrer"),
            (
                "Content-Security-Policy",
                "default-src 'self'; "
                "script-src 'self'; "
                "style-src 'self'; "
                "img-src 'self' data:; "
                "connect-src 'self'; "
                "worker-src 'self'; "
                "object-src 'none'; "
                "base-uri 'none'; "
                "frame-ancestors 'none'; "
                "form-action 'self'",
            ),
            ("Cross-Origin-Resource-Policy", "same-origin"),
            ("Permissions-Policy", "microphone=(self), camera=()"),
        )

    def _response(
        self,
        status: int,
        value: Mapping[str, object],
    ) -> DashboardHTTPResponse:
        body = _json_bytes(value)
        return DashboardHTTPResponse(
            status=status,
            headers=self._security_headers(
                "application/json; charset=utf-8"
            ),
            body=body,
        )

    def _error(self, error: DashboardHTTPError):
        return self._response(
            error.status,
            {
                "error": {
                    "code": error.code,
                    "message": str(error),
                }
            },
        )

    def _service_call(self, operation):
        try:
            return operation()
        except DashboardContractError as error:
            raise DashboardHTTPError(
                getattr(error, "status", 400),
                getattr(error, "code", "invalid_request"),
                str(error),
            ) from None
        except Exception as error:
            status = getattr(error, "status", None)
            code = getattr(error, "code", None)
            if (
                isinstance(status, int)
                and isinstance(code, str)
                and code
            ):
                raise DashboardHTTPError(
                    status,
                    code,
                    str(error),
                ) from None
            raise

    def _authorize(
        self,
        method: str,
        path: str,
        headers: Mapping[str, str],
    ) -> None:
        if headers.get("host") != self._expected_host:
            raise DashboardHTTPError(
                421,
                "host_rejected",
                "Request host is not accepted",
            )

        if path.startswith(("/live/", "/session/")):
            parts = path.split("/")
            path_token = parts[2] if len(parts) > 2 else ""
            if not secrets.compare_digest(
                path_token,
                self._session_token,
            ):
                raise DashboardHTTPError(
                    403,
                    "session_token_rejected",
                    "Live-console access key is invalid",
                )
            return

        if not path.startswith("/api/"):
            return
        token = headers.get(TOKEN_HEADER, "")
        if not secrets.compare_digest(token, self._session_token):
            raise DashboardHTTPError(
                403,
                "session_token_rejected",
                "Live-console access key is invalid",
            )
        origin = headers.get("origin")
        if origin not in (None, self._expected_origin):
            raise DashboardHTTPError(
                403,
                "origin_rejected",
                "Request origin is not accepted",
            )
        if "transfer-encoding" in headers:
            raise DashboardHTTPError(
                400,
                "transfer_encoding_rejected",
                "Transfer-Encoding is not accepted",
            )
        if "content-encoding" in headers:
            raise DashboardHTTPError(
                415,
                "content_encoding_rejected",
                "Content-Encoding is not accepted",
            )
        if method in ("POST", "PUT"):
            expected_content_type = (
                "audio/wav"
                if method == "POST"
                and path == STT_TRANSCRIPTIONS_PATH
                else "application/json"
            )
            if headers.get("content-type") != expected_content_type:
                raise DashboardHTTPError(
                    415,
                    "content_type_rejected",
                    "Content-Type is not accepted for this route",
                )

    def _body_object(self, body: bytes):
        if not isinstance(body, bytes) or len(body) > MAX_REQUEST_BYTES:
            raise DashboardHTTPError(
                413,
                "request_too_large",
                "Request body is too large",
            )
        try:
            return strict_json_loads(body)
        except DashboardContractError as error:
            raise DashboardHTTPError(
                400,
                getattr(error, "code", "invalid_json"),
                str(error),
            ) from None

    @staticmethod
    def _path_segments(path: str):
        if (
            not path.startswith("/")
            or "\\" in path
            or "\x00" in path
            or "//" in path
        ):
            raise DashboardHTTPError(
                404,
                "route_not_found",
                "Route was not found",
            )
        return tuple(segment for segment in path.split("/") if segment)

    @staticmethod
    def _parsed_target(target: str):
        try:
            parsed = urlsplit(target)
        except (TypeError, ValueError):
            raise DashboardHTTPError(
                400,
                "invalid_target",
                "Request target is invalid",
            ) from None
        if parsed.scheme or parsed.netloc or parsed.fragment:
            raise DashboardHTTPError(
                400,
                "invalid_target",
                "Request target is invalid",
            )
        DashboardRouter._path_segments(parsed.path)
        return parsed

    def _preflight(
        self,
        method: str,
        target: str,
        headers: Mapping[str, str],
    ):
        if method not in ("GET", "POST", "PUT", "DELETE"):
            raise DashboardHTTPError(
                405,
                "method_not_allowed",
                "HTTP method is not allowed",
            )
        request_headers = _lower_headers(headers)
        parsed = self._parsed_target(target)
        if method == "DELETE":
            segments = self._path_segments(parsed.path)
            if (
                len(segments) != 5
                or segments[:4]
                not in (
                    ("api", "v1", "stt", "transcriptions"),
                    ("api", "v1", "stt", "requests"),
                )
            ):
                raise DashboardHTTPError(
                    405,
                    "method_not_allowed",
                    "HTTP method is not allowed",
                )
        self._authorize(method, parsed.path, request_headers)
        return parsed

    def preflight(
        self,
        method: str,
        target: str,
        headers: Mapping[str, str],
    ) -> Optional[DashboardHTTPResponse]:
        """Validate routing headers before an HTTP adapter reads a body."""

        try:
            self._preflight(method, target, headers)
        except DashboardHTTPError as error:
            return self._error(error)
        return None

    def request_body_limit(self, method: str, target: str) -> int:
        """Return the exact post-preflight body cap for one route."""

        parsed = self._parsed_target(target)
        if (
            method == "POST"
            and parsed.path == STT_TRANSCRIPTIONS_PATH
            and not parsed.query
        ):
            return MAX_STT_AUDIO_BYTES
        return MAX_REQUEST_BYTES

    def handle(
        self,
        method: str,
        target: str,
        headers: Mapping[str, str],
        body: bytes = b"",
    ) -> DashboardHTTPResponse:
        try:
            return self._handle(method, target, headers, body)
        except DashboardHTTPError as error:
            return self._error(error)

    def _handle(
        self,
        method: str,
        target: str,
        headers: Mapping[str, str],
        body: bytes,
    ) -> DashboardHTTPResponse:
        parsed = self._preflight(method, target, headers)
        path = parsed.path
        segments = self._path_segments(path)

        if method == "GET" and path == "/" and not parsed.query:
            return DashboardHTTPResponse(
                status=307,
                headers=(
                    self._security_headers("text/plain; charset=utf-8")
                    + (("Location", self._access_path),)
                ),
                body=b"",
            )

        legacy_location = self._legacy_location(path)
        if method == "GET" and not parsed.query and legacy_location is not None:
            return DashboardHTTPResponse(
                status=308,
                headers=(
                    self._security_headers("text/plain; charset=utf-8")
                    + (("Location", legacy_location),)
                ),
                body=b"",
            )

        if method == "GET" and not parsed.query and path in self._assets:
            asset, content_type = self._assets[path]
            return DashboardHTTPResponse(
                status=200,
                headers=self._security_headers(content_type),
                body=asset,
            )

        if method == "GET" and path == "/api/v1/health":
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Health endpoint accepts no query",
                )
            return self._response(
                200,
                {
                    "status": "ok",
                    "api_version": API_VERSION,
                },
            )

        if method == "GET" and path == "/api/v1/bootstrap":
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Bootstrap endpoint accepts no query",
                )
            value = self._service_call(self._service.bootstrap)
            return self._response(200, value)

        if method == "GET" and path == "/api/v1/settings":
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Settings endpoint accepts no query",
                )
            value = self._service_call(self._service.settings)
            return self._response(200, {"settings": value})

        if method == "PUT" and path == "/api/v1/settings":
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Settings endpoint accepts no query",
                )
            request = _exact_object(
                self._body_object(body),
                ("expected_revision", "changes"),
            )
            value = self._service_call(
                lambda: self._service.update_settings(
                    request["expected_revision"],
                    request["changes"],
                )
            )
            return self._response(200, {"settings": value})

        if method == "GET" and path == "/api/v1/registry":
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Registry endpoint accepts no query",
                )
            value = self._service_call(self._service.registry)
            return self._response(200, {"registry": value})

        if method == "GET" and path == "/api/v1/map":
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Map endpoint accepts no query",
                )
            value = self._service_call(self._service.spatial_map)
            response = self._response(200, {"map": value})
            if len(response.body) > MAX_SPATIAL_MAP_RESPONSE_BYTES:
                raise DashboardHTTPError(
                    503,
                    "spatial_map_unavailable",
                    "Spatial map response exceeds its byte capacity",
                )
            return response

        if method == "POST" and path == STT_TRANSCRIPTIONS_PATH:
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Speech transcription endpoint accepts no query",
                )
            request_headers = _lower_headers(headers)
            request_id = request_headers.get(STT_REQUEST_ID_HEADER)
            language_hint = request_headers.get(STT_LANGUAGE_HEADER)
            if request_id is None or language_hint is None:
                raise DashboardHTTPError(
                    400,
                    "invalid_request_headers",
                    "Speech transcription headers are incomplete",
                )
            if len(body) > MAX_STT_AUDIO_BYTES:
                raise DashboardHTTPError(
                    413,
                    "request_too_large",
                    "Speech audio is too large",
                )
            value = self._service_call(
                lambda: self._service.submit_transcription(
                    request_id,
                    language_hint,
                    body,
                )
            )
            return self._response(
                202,
                {"transcription": value},
            )

        if (
            method == "GET"
            and len(segments) == 5
            and segments[:4] == (
                "api",
                "v1",
                "stt",
                "transcriptions",
            )
        ):
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Speech transcription endpoint accepts no query",
                )
            value = self._service_call(
                lambda: self._service.get_transcription(
                    segments[4]
                )
            )
            return self._response(
                200,
                {"transcription": value},
            )

        if (
            method == "DELETE"
            and len(segments) == 5
            and segments[:4] == (
                "api",
                "v1",
                "stt",
                "transcriptions",
            )
        ):
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Speech transcription endpoint accepts no query",
                )
            if body:
                raise DashboardHTTPError(
                    400,
                    "invalid_request",
                    "Speech cancellation accepts no request body",
                )
            value = self._service_call(
                lambda: self._service.cancel_transcription(
                    segments[4]
                )
            )
            return self._response(
                200,
                {"transcription": value},
            )

        if (
            method == "DELETE"
            and len(segments) == 5
            and segments[:4] == (
                "api",
                "v1",
                "stt",
                "requests",
            )
        ):
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Speech cancellation endpoint accepts no query",
                )
            if body:
                raise DashboardHTTPError(
                    400,
                    "invalid_request",
                    "Speech cancellation accepts no request body",
                )
            value = self._service_call(
                lambda: self._service.cancel_transcription_request(
                    segments[4]
                )
            )
            return self._response(
                200,
                {"cancellation": value},
            )

        if method == "POST" and path == "/api/v1/conversations":
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Conversation endpoint accepts no query",
                )
            request = _exact_object(
                self._body_object(body),
                (),
                ("title",),
            )
            conversation = self._service_call(
                lambda: self._service.create_conversation(
                    request.get("title")
                )
            )
            return self._response(
                201,
                {"conversation": conversation},
            )

        if (
            method == "GET"
            and len(segments) == 4
            and segments[:3] == ("api", "v1", "conversations")
        ):
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Conversation endpoint accepts no query",
                )
            conversation = self._service_call(
                lambda: self._service.get_conversation(segments[3])
            )
            return self._response(
                200,
                {"conversation": conversation},
            )

        if (
            method == "POST"
            and len(segments) == 5
            and segments[:3] == ("api", "v1", "conversations")
            and segments[4] == "turns"
        ):
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Turn endpoint accepts no query",
                )
            request = _exact_object(
                self._body_object(body),
                (
                    "client_request_id",
                    "expected_conversation_version",
                    "content",
                    "mode",
                    "response_locale",
                ),
            )
            turn = self._service_call(
                lambda: self._service.submit_turn(
                    segments[3],
                    request["client_request_id"],
                    request["expected_conversation_version"],
                    request["content"],
                    request["mode"],
                    request["response_locale"],
                )
            )
            return self._response(202, {"turn": turn})

        if (
            method == "GET"
            and len(segments) == 4
            and segments[:3] == ("api", "v1", "turns")
        ):
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Turn endpoint accepts no query",
                )
            turn = self._service_call(
                lambda: self._service.get_turn(segments[3])
            )
            return self._response(200, {"turn": turn})

        if method == "GET" and path == "/api/v1/events":
            try:
                query = parse_qs(
                    parsed.query,
                    keep_blank_values=True,
                    strict_parsing=True,
                    max_num_fields=2,
                )
            except (TypeError, ValueError):
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Event query is invalid",
                ) from None
            if not set(query) <= {"after_sequence", "limit"}:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Event query is invalid",
                )
            after_sequence = _integer_query(
                query,
                "after_sequence",
                0,
                0,
                2**63 - 1,
            )
            limit = _integer_query(
                query,
                "limit",
                100,
                1,
                MAX_EVENT_LIMIT,
            )
            value = self._service_call(
                lambda: self._service.events(
                    after_sequence,
                    limit,
                )
            )
            return self._response(200, value)

        if (
            method == "POST"
            and path == "/api/v1/runtime/lm-studio/probe"
        ):
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Probe endpoint accepts no query",
                )
            _exact_object(self._body_object(body), ())
            value = self._service_call(self._service.probe_lm_studio)
            return self._response(200, {"lm_studio": value})

        if (
            method == "POST"
            and path == "/api/v1/runtime/stt/probe"
        ):
            if parsed.query:
                raise DashboardHTTPError(
                    400,
                    "invalid_query",
                    "Probe endpoint accepts no query",
                )
            _exact_object(self._body_object(body), ())
            value = self._service_call(
                self._service.probe_speech_transcriber
            )
            return self._response(
                200,
                {"speech_to_text": value},
            )

        if (
            self._robot_control_router is not None
            and self._robot_control_router.handles(path)
        ):
            try:
                value = self._robot_control_router.handle(
                    method,
                    path,
                    parsed.query,
                    body,
                )
            except Exception as error:
                status = getattr(error, "status", None)
                code = getattr(error, "code", None)
                if (
                    isinstance(status, int)
                    and isinstance(code, str)
                    and code
                ):
                    raise DashboardHTTPError(
                        status,
                        code,
                        str(error),
                    ) from None
                raise
            return self._response(value.status, value.body)

        raise DashboardHTTPError(
            404,
            "route_not_found",
            "Route was not found",
        )


def new_session_token() -> str:
    """Return a random 256-bit access key (legacy public function name)."""

    return secrets.token_hex(32)
