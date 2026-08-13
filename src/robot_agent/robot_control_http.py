"""Exact HTTP route adapter for :mod:`robot_control_service`.

Authentication, origin checks, framing, and response headers remain owned by
``DashboardRouter``.  This delegated router only validates robot route shapes
and translates them into typed service calls.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Mapping
from urllib.parse import parse_qs

from .dashboard_contract import DashboardContractError, strict_json_loads
from .dashboard_service import strict_spatial_map_snapshot
from .robot_control_service import (
    MAX_ROBOT_PAGE_RESPONSE_BYTES,
    RobotControlServiceError,
)


ROBOT_API_PREFIX = "/api/v1/robot/"
ROBOTS_DIRECTORY_PATH = "/api/v1/robots"
ROBOTS_API_PREFIX = ROBOTS_DIRECTORY_PATH + "/"
CONTROLLER_API_PREFIX = "/api/v1/controllers/"
BLAST_COMMAND_PATH = "/api/v1/controllers/blast-01.hub/commands"
BLAST_CONNECTION_PATH = "/api/v1/controllers/blast-01.hub/connection"
EV3_CONTROLLER_ID = "ev3rstorm-01.ev3-main"
EV3_REACHABILITY_PATH = (
    "/api/v1/controllers/ev3rstorm-01.ev3-main/reachability"
)
MAX_ROBOT_REQUEST_BYTES = 16 * 1024
MAX_ROBOT_PAGE_LIMIT = 500
BLAST_COMMANDS = frozenset((
    "drive_forward",
    "drive_reverse",
    "turn_left",
    "turn_right",
    "claw_open",
    "claw_close",
    "body_left",
    "body_right",
    "stop",
))
CONTROLLER_CONNECTION_ACTIONS = frozenset((
    "connect",
    "disconnect",
    "retry",
))
CONTROLLER_ERROR_STATUS = {
    "controller_busy": 409,
    "controller_command_interrupted": 409,
    "stale_controller_command": 409,
    "controller_unavailable": 503,
    "controller_command_failed": 502,
    "controller_motion_not_stopped": 503,
    "controller_command_timeout": 504,
    "controller_connection_failed": 502,
}


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


def _response_size(value: Mapping[str, object]) -> int:
    return len(json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8"))


class RobotControlHTTPRouter:
    """Delegate only the explicitly enumerated robot control routes."""

    def __init__(
        self,
        service,
        input_service=None,
        controller_services=None,
    ):
        if service is None:
            raise ValueError("Robot control HTTP service is required")
        services = {} if controller_services is None else controller_services
        if (
            not isinstance(services, Mapping)
            or any(
                not isinstance(controller_id, str)
                or controller is None
                or (
                    controller_id == "blast-01.hub"
                    and not callable(getattr(controller, "command", None))
                )
                or (
                    controller_id == EV3_CONTROLLER_ID
                    and not callable(getattr(controller, "check", None))
                )
                or controller_id not in {
                    "blast-01.hub",
                    EV3_CONTROLLER_ID,
                }
                for controller_id, controller in services.items()
            )
        ):
            raise ValueError("Controller services are invalid")
        self._service = service
        self._input_service = input_service
        self._controller_services = dict(services)

    @staticmethod
    def handles(path: str) -> bool:
        return isinstance(path, str) and path.startswith(
            (ROBOT_API_PREFIX, CONTROLLER_API_PREFIX)
        )

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
    def _controller_call(
        operation,
        *,
        default_code="controller_command_failed",
        message="Controller command failed",
    ):
        try:
            return operation()
        except RobotControlHTTPError:
            raise
        except Exception as error:
            code = getattr(error, "code", default_code)
            if code not in CONTROLLER_ERROR_STATUS:
                code = default_code
            raise RobotControlHTTPError(
                CONTROLLER_ERROR_STATUS[code],
                code,
                message,
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

        if path.startswith(CONTROLLER_API_PREFIX):
            if method != "POST" or path not in {
                BLAST_COMMAND_PATH,
                BLAST_CONNECTION_PATH,
                EV3_REACHABILITY_PATH,
            }:
                raise RobotControlHTTPError(
                    404,
                    "controller_route_not_found",
                    "Controller route was not found",
                )
            endpoint = (
                "EV3 reachability endpoint"
                if path == EV3_REACHABILITY_PATH
                else (
                    "Controller connection endpoint"
                    if path == BLAST_CONNECTION_PATH
                    else "Controller command endpoint"
                )
            )
            self._no_query(query, endpoint)
            controller_id = (
                EV3_CONTROLLER_ID
                if path == EV3_REACHABILITY_PATH
                else "blast-01.hub"
            )
            controller = self._controller_services.get(controller_id)
            if controller is None:
                raise RobotControlHTTPError(
                    503,
                    "controller_unavailable",
                    "Controller is not configured",
                )
            if path == EV3_REACHABILITY_PATH:
                self._empty_command(body)
                operation = getattr(controller, "check", None)
                if not callable(operation):
                    raise RobotControlHTTPError(
                        503,
                        "controller_connection_unavailable",
                        "EV3 reachability check is not configured",
                    )
                value = self._controller_call(
                    operation,
                    default_code="controller_connection_failed",
                    message="EV3 reachability check failed",
                )
                return RobotControlHTTPResponse(
                    200,
                    {"reachability": value},
                )
            if path == BLAST_CONNECTION_PATH:
                request = _exact_object(
                    _body_object(body),
                    ("action",),
                )
                action = request["action"]
                if (
                    not isinstance(action, str)
                    or action not in CONTROLLER_CONNECTION_ACTIONS
                ):
                    raise RobotControlHTTPError(
                        400,
                        "invalid_controller_connection_action",
                        "Controller connection action is invalid",
                    )
                operation = getattr(controller, action, None)
                if not callable(operation):
                    raise RobotControlHTTPError(
                        503,
                        "controller_connection_unavailable",
                        "Controller connection lifecycle is not configured",
                    )
                value = self._controller_call(
                    operation,
                    default_code="controller_connection_failed",
                    message="Controller connection action failed",
                )
                return RobotControlHTTPResponse(
                    200,
                    {"connection": value},
                )
            request = _exact_object(
                _body_object(body),
                ("command",),
            )
            command = request["command"]
            if not isinstance(command, str) or command not in BLAST_COMMANDS:
                raise RobotControlHTTPError(
                    400,
                    "invalid_controller_command",
                    "Controller command is invalid",
                )
            value = self._controller_call(
                lambda: controller.command(command)
            )
            return RobotControlHTTPResponse(200, {"result": value})

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

        if method == "POST" and path == "/api/v1/robot/turns":
            self._no_query(query, "Robot turn endpoint")
            if self._input_service is None:
                raise RobotControlHTTPError(
                    503,
                    "robot_input_disabled",
                    "Robot input service is not configured",
                )
            request = _exact_object(
                _body_object(body),
                (
                    "text",
                    "locale",
                    "client_request_id",
                    "expected_revision",
                ),
            )
            value = self._call(
                lambda: self._input_service.dispatch(
                    request["text"],
                    request["locale"],
                    request["client_request_id"],
                    request["expected_revision"],
                )
            )
            status = 202 if value.get("episode") is not None else 200
            return RobotControlHTTPResponse(status, {"turn": value})

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
            if _response_size(value) > MAX_ROBOT_PAGE_RESPONSE_BYTES:
                raise RobotControlHTTPError(
                    500,
                    "robot_history_page_too_large",
                    "Robot history page exceeds its HTTP byte capacity",
                )
            return RobotControlHTTPResponse(200, value)

        raise RobotControlHTTPError(
            404,
            "robot_route_not_found",
            "Robot route was not found",
        )


class RobotControlHTTPDirectoryRouter:
    """Select one existing one-robot router by its immutable robot ID."""

    _OPERATIONS = frozenset((
        "status",
        "settings",
        "episodes",
        "turns",
        "stop",
        "emergency-stop",
        "events",
        "snapshots",
    ))
    _SPATIAL_MAP_OPERATION = "spatial-map"

    def __init__(
        self,
        routers: Mapping[str, RobotControlHTTPRouter],
        *,
        default_router: RobotControlHTTPRouter,
        default_robot_id=None,
        spatial_map_providers=None,
    ):
        map_providers = (
            {} if spatial_map_providers is None else spatial_map_providers
        )
        if (
            not isinstance(routers, Mapping)
            or not isinstance(default_router, RobotControlHTTPRouter)
            or not isinstance(map_providers, Mapping)
            or any(
                not isinstance(robot_id, str)
                or not robot_id
                or robot_id != robot_id.strip()
                or "/" in robot_id
                or not isinstance(router, RobotControlHTTPRouter)
                for robot_id, router in routers.items()
            )
            or (
                default_robot_id is not None
                and default_robot_id not in routers
            )
            or not set(map_providers) <= set(routers)
        ):
            raise ValueError("Robot control HTTP directory is invalid")
        self._routers = dict(routers)
        self._default_router = default_router
        self._default_robot_id = default_robot_id
        self._spatial_map_providers = dict(map_providers)

    @staticmethod
    def handles(path: str) -> bool:
        return isinstance(path, str) and (
            path == ROBOTS_DIRECTORY_PATH
            or path.startswith(ROBOTS_API_PREFIX)
            or RobotControlHTTPRouter.handles(path)
        )

    def _directory(self, method: str, query: str, body: bytes):
        if method != "GET":
            raise RobotControlHTTPError(
                404,
                "robot_route_not_found",
                "Robot route was not found",
            )
        RobotControlHTTPRouter._no_query(
            query,
            "Robot directory endpoint",
        )
        controls = []
        for robot_id in sorted(self._routers):
            response = self._routers[robot_id].handle(
                "GET",
                "/api/v1/robot/status",
                "",
                body,
            )
            control = response.body.get("control")
            target = (
                control.get("target")
                if isinstance(control, Mapping)
                else None
            )
            if (
                not isinstance(target, Mapping)
                or target.get("robot_id") != robot_id
            ):
                raise RobotControlHTTPError(
                    500,
                    "robot_directory_target_mismatch",
                    "Robot directory target does not match its service",
                )
            controls.append(control)
        return RobotControlHTTPResponse(
            200,
            {
                "schema": "robot-control-directory/v1",
                "controls": controls,
            },
        )

    def _scoped_router(self, path: str):
        remainder = path[len(ROBOTS_API_PREFIX):]
        robot_id, separator, operation = remainder.partition("/")
        if (
            not separator
            or not robot_id
            or operation not in self._OPERATIONS
        ):
            raise RobotControlHTTPError(
                404,
                "robot_route_not_found",
                "Robot route was not found",
            )
        router = self._routers.get(robot_id)
        if router is None:
            raise RobotControlHTTPError(
                404,
                "robot_target_not_found",
                "Robot target was not found",
            )
        return router, "/api/v1/robot/{}".format(operation)

    def _scoped_spatial_map(
        self,
        method: str,
        path: str,
        query: str,
    ):
        remainder = path[len(ROBOTS_API_PREFIX):]
        robot_id, separator, operation = remainder.partition("/")
        if (
            not separator
            or operation != self._SPATIAL_MAP_OPERATION
        ):
            return None
        if robot_id not in self._routers:
            raise RobotControlHTTPError(
                404,
                "robot_target_not_found",
                "Robot target was not found",
            )
        if method != "GET":
            raise RobotControlHTTPError(
                404,
                "robot_route_not_found",
                "Robot route was not found",
            )
        RobotControlHTTPRouter._no_query(
            query,
            "Robot spatial map endpoint",
        )
        provider = self._spatial_map_providers.get(robot_id)
        if provider is None:
            raise RobotControlHTTPError(
                503,
                "spatial_map_unavailable",
                "Spatial map snapshot is unavailable",
            )
        try:
            snapshot = strict_spatial_map_snapshot(provider)
        except Exception as error:
            status = getattr(error, "status", None)
            code = getattr(error, "code", None)
            if isinstance(status, int) and isinstance(code, str) and code:
                raise RobotControlHTTPError(
                    status,
                    code,
                    str(error),
                ) from None
            raise
        value = {"map": snapshot}
        if _response_size(value) > MAX_ROBOT_PAGE_RESPONSE_BYTES:
            raise RobotControlHTTPError(
                503,
                "spatial_map_unavailable",
                "Spatial map response exceeds its byte capacity",
            )
        return RobotControlHTTPResponse(200, value)

    def handle(
        self,
        method: str,
        path: str,
        query: str,
        body: bytes,
    ) -> RobotControlHTTPResponse:
        if path == ROBOTS_DIRECTORY_PATH:
            return self._directory(method, query, body)
        if path.startswith(ROBOTS_API_PREFIX):
            spatial_map = self._scoped_spatial_map(
                method,
                path,
                query,
            )
            if spatial_map is not None:
                return spatial_map
            router, delegated_path = self._scoped_router(path)
            return router.handle(method, delegated_path, query, body)
        if RobotControlHTTPRouter.handles(path):
            return self._default_router.handle(method, path, query, body)
        raise RobotControlHTTPError(
            404,
            "robot_route_not_found",
            "Robot route was not found",
        )
