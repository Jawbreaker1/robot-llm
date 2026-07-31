"""Exact HTTP route adapter for :mod:`robot_control_service`.

Authentication, origin checks, framing, and response headers remain owned by
``DashboardRouter``.  This delegated router only validates robot route shapes
and translates them into typed service calls.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping
from urllib.parse import parse_qs

from .dashboard_contract import DashboardContractError, strict_json_loads
from .robot_control_service import RobotControlServiceError


ROBOT_API_PREFIX = "/api/v1/robot/"
MAX_ROBOT_REQUEST_BYTES = 16 * 1024
MAX_ROBOT_PAGE_LIMIT = 500


class RobotControlHTTPError(RuntimeError):
    """Safely reportable delegated route failure."""

    def __init__(self, status: int, code: str, message: str):
        self.status = status
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class RobotControlHTTPResponse:
    status: int
    body: Mapping[str, object]


def _exact_object(value, required=(), optional=()):
    if not isinstance(value, dict):
        raise RobotControlHTTPError(
            400,
            "invalid_robot_request",
            "Robot request body must be an object",
        )
    keys = set(value)
    required_keys = set(required)
    if (
        not required_keys <= keys
        or not keys <= required_keys | set(optional)
    ):
        raise RobotControlHTTPError(
            400,
            "invalid_robot_request_fields",
            "Robot request fields are invalid",
        )
    return value


def _body_object(body: bytes):
    if (
        not isinstance(body, bytes)
        or not body
        or len(body) > MAX_ROBOT_REQUEST_BYTES
    ):
        raise RobotControlHTTPError(
            400 if body == b"" else 413,
            (
                "invalid_robot_request"
                if body == b""
                else "robot_request_too_large"
            ),
            (
                "Robot request body is required"
                if body == b""
                else "Robot request body is too large"
            ),
        )
    try:
        return strict_json_loads(body)
    except DashboardContractError as error:
        raise RobotControlHTTPError(
            400,
            error.code,
            str(error),
        ) from None


def _integer_query(query, name, default):
    values = query.get(name)
    if values is None:
        return default
    if (
        not isinstance(values, list)
        or len(values) != 1
        or not values[0].isascii()
        or not values[0].isdigit()
    ):
        raise RobotControlHTTPError(
            400,
            "invalid_robot_query",
            "Robot query is invalid",
        )
    return int(values[0])


class RobotControlHTTPRouter:
    """Delegate only the explicitly enumerated robot control routes."""

    def __init__(self, service):
        if service is None:
            raise ValueError("Robot control HTTP service is required")
        self._service = service

    @staticmethod
    def handles(path: str) -> bool:
        return isinstance(path, str) and path.startswith(ROBOT_API_PREFIX)

    def _call(self, operation):
        try:
            return operation()
        except RobotControlHTTPError:
            raise
        except RobotControlServiceError as error:
            raise RobotControlHTTPError(
                error.status,
                error.code,
                str(error),
            ) from None
        except DashboardContractError as error:
            raise RobotControlHTTPError(
                400,
                error.code,
                str(error),
            ) from None

    @staticmethod
    def _no_query(query: str, endpoint: str) -> None:
        if query:
            raise RobotControlHTTPError(
                400,
                "invalid_robot_query",
                "{} accepts no query".format(endpoint),
            )

    @staticmethod
    def _empty_command(body: bytes):
        return _exact_object(_body_object(body))

    def handle(
        self,
        method: str,
        path: str,
        query: str,
        body: bytes,
    ) -> RobotControlHTTPResponse:
        if not self.handles(path):
            raise RobotControlHTTPError(
                404,
                "robot_route_not_found",
                "Robot route was not found",
            )

        if method == "GET" and path == "/api/v1/robot/status":
            self._no_query(query, "Robot status endpoint")
            value = self._call(self._service.status)
            return RobotControlHTTPResponse(200, {"control": value})

        if method == "GET" and path == "/api/v1/robot/settings":
            self._no_query(query, "Robot settings endpoint")
            value = self._call(self._service.settings)
            return RobotControlHTTPResponse(200, {"settings": value})

        if method == "PUT" and path == "/api/v1/robot/settings":
            self._no_query(query, "Robot settings endpoint")
            request = _exact_object(
                _body_object(body),
                ("expected_revision", "changes"),
            )
            value = self._call(
                lambda: self._service.update_settings(
                    request["expected_revision"],
                    request["changes"],
                )
            )
            return RobotControlHTTPResponse(200, {"settings": value})

        if method == "POST" and path == "/api/v1/robot/episodes":
            self._no_query(query, "Robot episode endpoint")
            request = _exact_object(
                _body_object(body),
                (
                    "goal",
                    "locale",
                    "client_request_id",
                    "expected_revision",
                ),
            )
            value = self._call(
                lambda: self._service.start(
                    request["goal"],
                    request["locale"],
                    request["client_request_id"],
                    request["expected_revision"],
                )
            )
            return RobotControlHTTPResponse(202, {"episode": value})

        if method == "POST" and path == "/api/v1/robot/stop":
            self._no_query(query, "Robot stop endpoint")
            self._empty_command(body)
            value = self._call(self._service.stop)
            return RobotControlHTTPResponse(200, {"control": value})

        if (
            method == "POST"
            and path == "/api/v1/robot/emergency-stop"
        ):
            self._no_query(query, "Robot emergency-stop endpoint")
            self._empty_command(body)
            value = self._call(self._service.emergency_stop)
            return RobotControlHTTPResponse(200, {"control": value})

        if method == "GET" and path in (
            "/api/v1/robot/events",
            "/api/v1/robot/snapshots",
        ):
            try:
                parsed = parse_qs(
                    query,
                    keep_blank_values=True,
                    strict_parsing=True,
                    max_num_fields=2,
                )
            except (TypeError, ValueError):
                raise RobotControlHTTPError(
                    400,
                    "invalid_robot_query",
                    "Robot query is invalid",
                ) from None
            if not set(parsed) <= {"after_sequence", "limit"}:
                raise RobotControlHTTPError(
                    400,
                    "invalid_robot_query",
                    "Robot query is invalid",
                )
            after_sequence = _integer_query(
                parsed,
                "after_sequence",
                0,
            )
            limit = _integer_query(parsed, "limit", 100)
            if (
                after_sequence > 2**63 - 1
                or not 1 <= limit <= MAX_ROBOT_PAGE_LIMIT
            ):
                raise RobotControlHTTPError(
                    400,
                    "invalid_robot_query",
                    "Robot query is invalid",
                )
            if path.endswith("/events"):
                value = self._call(
                    lambda: self._service.events(
                        after_sequence,
                        limit,
                    )
                )
            else:
                value = self._call(
                    lambda: self._service.snapshots(
                        after_sequence,
                        limit,
                    )
                )
            return RobotControlHTTPResponse(200, value)

        raise RobotControlHTTPError(
            404,
            "robot_route_not_found",
            "Robot route was not found",
        )
