"""Run the local Robot LLM dashboard and its delegated control plane."""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import socket
import sys
import threading
from typing import Mapping, Optional, Sequence

from .dashboard_access_key import load_or_create_dashboard_access_key
from .dashboard_http import (
    DashboardHTTPResponse,
    DashboardRouter,
    new_session_token,
)
from .dashboard_service import DashboardService
from .blast_observation_monitor import (
    CONTROLLER_ID as BLAST_CONTROLLER_ID,
    ROBOT_ID as BLAST_ROBOT_ID,
    BlastObservationMonitor,
)
from .blast_episode_adapter import (
    BLAST_PROFILE_ID,
    BlastEpisodeRuntimeAdapter,
)
from .blast_hub_speech import BLAST_PIPER_PROFILE, BlastHubSpeaker
from .blast_personality import BLAST_PERSONA_BY_LOCALE
from .ev3rstorm_profile import (
    DEFAULT_EV3RSTORM_MEMORY_PATH,
    EV3RSTORM_PROFILE_ID,
    EV3RSTORMProfile,
    EV3SSHBinding,
)
from .lm_studio import DEFAULT_BASE_URL, DEFAULT_MODEL
from .lm_studio_navigation import LMStudioNavigationPlanner
from .lm_studio_controller_action import LMStudioControllerActionPlanner
from .lm_studio_robot_input import (
    LMStudioRobotInputModel,
    REQUEST_TIMEOUT_SECONDS as ROBOT_INPUT_TIMEOUT_SECONDS,
)
from .host_piper_speech import (
    LocaleSpeechSynthesizer,
    MacOSSayWAVSynthesizer,
    PiperLoopbackSynthesizer,
)
from .robot_control_contract import (
    RobotControlSettings,
    RobotControlTarget,
)
from .robot_control_http import (
    EV3_CONTROLLER_ID,
    RobotControlHTTPDirectoryRouter,
    RobotControlHTTPRouter,
)
from .robot_control_service import RobotControlService, RobotEpisodeGate
from .robot_input_service import RobotInputService
from .robot_speech_runtime import RobotSpeechRuntime
from .robot_turn_speech import RobotTurnSpeechSink
from .remote_spatial_map import RemoteSpatialMapProvider
from .shared_fixed_start_map import FixedStartSharedMapProvider
from .shared_frame_transform import MAX_FRAME_COORDINATE_MM


LOOPBACK_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
MAX_HTTP_REQUEST_THREADS = 16
HTTP_READ_TIMEOUT_SECONDS = 5.0
ROBOT_PROFILE_DISABLED = "disabled"
ROBOT_PROFILE_CHOICES = (
    ROBOT_PROFILE_DISABLED,
    EV3RSTORM_PROFILE_ID,
    BLAST_PROFILE_ID,
)
# BLAST can upload at most eight seconds of ADPCM.  Keep navigation remarks
# short enough to leave synthesis headroom for natural Swedish pauses.
BLAST_MAX_NAVIGATION_UTTERANCE_CHARS = 72


def _configured_robot_control_target(
    profile_id: str,
) -> Optional[RobotControlTarget]:
    """Return the one physical addressee selected for this server."""

    if profile_id == ROBOT_PROFILE_DISABLED:
        return None
    if profile_id == BLAST_PROFILE_ID:
        return RobotControlTarget(
            robot_id=BLAST_ROBOT_ID,
            display_name="BLAST",
        )
    if profile_id == EV3RSTORM_PROFILE_ID:
        return RobotControlTarget(
            robot_id=EV3RSTORM_PROFILE_ID,
            display_name="EV3RSTORM",
        )
    raise ValueError("physical robot profile is unsupported")


class _LoopbackThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        server_address,
        request_handler_class,
        *,
        max_request_threads: int = MAX_HTTP_REQUEST_THREADS,
        read_timeout_seconds: float = HTTP_READ_TIMEOUT_SECONDS,
    ):
        if (
            isinstance(max_request_threads, bool)
            or not isinstance(max_request_threads, int)
            or not 1 <= max_request_threads <= 256
            or isinstance(read_timeout_seconds, bool)
            or not isinstance(read_timeout_seconds, (int, float))
            or not math.isfinite(float(read_timeout_seconds))
            or not 0 < float(read_timeout_seconds) <= 60
        ):
            raise ValueError("Dashboard HTTP limits are invalid")
        self._request_slots = threading.BoundedSemaphore(
            max_request_threads
        )
        self._read_timeout_seconds = float(read_timeout_seconds)
        super().__init__(server_address, request_handler_class)

    def get_request(self):
        request, client_address = super().get_request()
        request.settimeout(self._read_timeout_seconds)
        return request, client_address

    def process_request(self, request, client_address):
        if not self._request_slots.acquire(blocking=False):
            try:
                request.sendall(
                    b"HTTP/1.1 503 Service Unavailable\r\n"
                    b"Connection: close\r\n"
                    b"Cache-Control: no-store\r\n"
                    b"Content-Length: 0\r\n\r\n"
                )
            except OSError:
                pass
            try:
                request.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            request.close()
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self._request_slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self._request_slots.release()


def _handler_class(router: DashboardRouter):
    class DashboardHandler(BaseHTTPRequestHandler):
        server_version = "RobotLLMDashboard/1"
        sys_version = ""

        def log_message(self, _fmt, *_args):
            # Polling is intentionally quiet, and the capability-bearing
            # session URL must never be copied into a terminal log.
            status = str(_args[1]) if len(_args) > 1 else ""
            if status.isascii() and status.isdigit() and int(status) >= 400:
                sys.stderr.write(
                    "[dashboard] HTTP error status {}\n".format(status)
                )

        def _send(self, response: DashboardHTTPResponse):
            self.send_response(response.status)
            for name, value in response.headers:
                self.send_header(name, value)
            self.send_header("Content-Length", str(len(response.body)))
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(response.body)

        def _request_body(self, max_bytes: int) -> bytes:
            raw_length = self.headers.get("Content-Length")
            if raw_length is None:
                return b""
            if (
                not raw_length.isascii()
                or not raw_length.isdigit()
                or len(raw_length) > 8
            ):
                raise ValueError("invalid content length")
            length = int(raw_length)
            if length > max_bytes:
                raise OverflowError("request body too large")
            body = self.rfile.read(length)
            if len(body) != length:
                raise ValueError("incomplete request body")
            return body

        def _dispatch(self):
            headers = {}
            for name in self.headers:
                values = self.headers.get_all(name)
                lowered = name.lower()
                if (
                    not values
                    or len(values) != 1
                    or lowered in headers
                ):
                    self._send(
                        router._response(
                            400,
                            {
                                "error": {
                                    "code": "duplicate_header",
                                    "message": "Request headers are invalid",
                                }
                            },
                        )
                    )
                    return
                headers[lowered] = values[0]

            preflight = router.preflight(
                self.command,
                self.path,
                headers,
            )
            if preflight is not None:
                self._send(preflight)
                return
            try:
                body = self._request_body(
                    router.request_body_limit(
                        self.command,
                        self.path,
                    )
                )
            except OverflowError:
                self._send(
                    router._response(
                        413,
                        {
                            "error": {
                                "code": "request_too_large",
                                "message": "Request body is too large",
                            }
                        },
                    )
                )
                return
            except (socket.timeout, TimeoutError):
                self._send(
                    router._response(
                        408,
                        {
                            "error": {
                                "code": "request_timeout",
                                "message": "Request body timed out",
                            }
                        },
                    )
                )
                return
            except ValueError:
                self._send(
                    router._response(
                        400,
                        {
                            "error": {
                                "code": "invalid_http_framing",
                                "message": "Request framing is invalid",
                            }
                        },
                    )
                )
                return

            try:
                response = router.handle(
                    self.command,
                    self.path,
                    headers,
                    body,
                )
            except Exception:
                response = router._response(
                    500,
                    {
                        "error": {
                            "code": "internal_error",
                            "message": "Dashboard request failed",
                        }
                    },
                )
            self._send(response)

        do_GET = _dispatch
        do_POST = _dispatch
        do_PUT = _dispatch
        do_DELETE = _dispatch
        do_PATCH = _dispatch
        do_OPTIONS = _dispatch
        do_HEAD = _dispatch

    return DashboardHandler


def build_server(
    service: DashboardService,
    port: int = DEFAULT_PORT,
    session_token: Optional[str] = None,
    robot_control_service=None,
    robot_input_service=None,
    controller_control_services=None,
    robot_control_services=None,
    robot_input_services=None,
    default_robot_id=None,
    robot_spatial_map_providers=None,
):
    if (
        isinstance(port, bool)
        or not isinstance(port, int)
        or not 1 <= port <= 65_535
    ):
        raise ValueError("Dashboard port is invalid")
    token = session_token or new_session_token()
    expected_host = "{}:{}".format(LOOPBACK_HOST, port)
    control_service = (
        robot_control_service
        if robot_control_service is not None
        else RobotControlService()
    )
    default_control_router = RobotControlHTTPRouter(
        control_service,
        robot_input_service,
        controller_control_services,
    )
    if robot_control_services is None:
        snapshot = control_service.status()
        target = (
            snapshot.get("target")
            if isinstance(snapshot, Mapping)
            else None
        )
        inferred_robot_id = (
            target.get("robot_id")
            if isinstance(target, Mapping)
            else None
        )
        scoped_control_services = (
            {inferred_robot_id: control_service}
            if isinstance(inferred_robot_id, str)
            else {}
        )
        scoped_input_services = (
            {inferred_robot_id: robot_input_service}
            if (
                isinstance(inferred_robot_id, str)
                and robot_input_service is not None
            )
            else {}
        )
        if default_robot_id is None:
            default_robot_id = inferred_robot_id
        scoped_map_providers = (
            {inferred_robot_id: service}
            if (
                robot_spatial_map_providers is None
                and isinstance(inferred_robot_id, str)
            )
            else robot_spatial_map_providers
        )
    else:
        scoped_control_services = robot_control_services
        scoped_input_services = (
            {} if robot_input_services is None else robot_input_services
        )
        scoped_map_providers = robot_spatial_map_providers
    if scoped_map_providers is None:
        scoped_map_providers = {}
    if (
        not isinstance(scoped_control_services, Mapping)
        or not isinstance(scoped_input_services, Mapping)
        or not isinstance(scoped_map_providers, Mapping)
        or not set(scoped_input_services) <= set(scoped_control_services)
        or not set(scoped_map_providers) <= set(scoped_control_services)
    ):
        raise ValueError("Robot control service directory is invalid")
    scoped_routers = {
        robot_id: RobotControlHTTPRouter(
            scoped_control_service,
            scoped_input_services.get(robot_id),
        )
        for robot_id, scoped_control_service
        in scoped_control_services.items()
    }
    router = DashboardRouter(
        service=service,
        session_token=token,
        expected_host=expected_host,
        robot_control_router=RobotControlHTTPDirectoryRouter(
            scoped_routers,
            default_router=default_control_router,
            default_robot_id=default_robot_id,
            spatial_map_providers=scoped_map_providers,
        ),
    )
    server = _LoopbackThreadingHTTPServer(
        (LOOPBACK_HOST, port),
        _handler_class(router),
    )
    return server, router


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Starta Robot LLM:s lokala Mac-dashboard."
        )
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help="Loopback-port (default: %(default)s)",
    )
    parser.add_argument(
        "--console-access-key-file",
        help=(
            "Owner-only file used to keep the private live-console URL "
            "stable across server restarts"
        ),
    )
    parser.add_argument(
        "--lm-studio-url",
        default=DEFAULT_BASE_URL,
        help="Loopback-URL till LM Studio (default: %(default)s)",
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL,
        help="Exakt modell-ID som LM Studio ska använda",
    )
    parser.add_argument(
        "--simulation-map-demo",
        action="store_true",
        help=(
            "Bygg en riktig simulatorisk navigationskarta före start "
            "och visa den skrivskyddat i GUI:t"
        ),
    )
    parser.add_argument(
        "--robot-profile",
        choices=ROBOT_PROFILE_CHOICES,
        default=ROBOT_PROFILE_DISABLED,
        help=(
            "Explicit physical controller profile; disabled by default "
            "(choices: %(choices)s)"
        ),
    )
    parser.add_argument(
        "--blast-hub-name",
        help=(
            "Connect one Robot Inventor hub for telemetry and bounded manual "
            "tests; required when --robot-profile blast-01 is selected"
        ),
    )
    parser.add_argument(
        "--robot-target",
        help=(
            "SSH target for ev3rstorm-01, for example robot@ev3dev.local; "
            "required only when that physical profile is selected"
        ),
    )
    parser.add_argument(
        "--robot-memory-path",
        default=str(DEFAULT_EV3RSTORM_MEMORY_PATH),
        help=(
            "Host-local physical navigation memory file "
            "(default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--robot-reset-memory",
        action="store_true",
        help=(
            "Compatibility flag; every EV3 episode already starts with "
            "fresh navigation memory"
        ),
    )
    parser.add_argument(
        "--robot-planner-timeout-seconds",
        type=float,
        default=30.0,
        help="Structured physical planner timeout (default: %(default)s)",
    )
    parser.add_argument(
        "--robot-input-timeout-seconds",
        type=float,
        default=ROBOT_INPUT_TIMEOUT_SECONDS,
        help=(
            "Interactive robot message timeout, independent of the physical "
            "planner (default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--shared-peer-port",
        type=int,
        help=(
            "Loopback port of the opposite robot dashboard whose local "
            "map should be projected into this fixed-start shared map"
        ),
    )
    parser.add_argument(
        "--shared-peer-access-key-file",
        help="Owner-only access-key file used by the peer dashboard",
    )
    parser.add_argument(
        "--shared-peer-x-mm",
        type=int,
        help="Peer start X in this robot's start frame, in millimetres",
    )
    parser.add_argument(
        "--shared-peer-y-mm",
        type=int,
        help="Peer start Y in this robot's start frame, in millimetres",
    )
    parser.add_argument(
        "--shared-peer-yaw-mdeg",
        type=int,
        help="Peer start heading in this robot's frame, in millidegrees",
    )
    stt_source = parser.add_mutually_exclusive_group()
    stt_source.add_argument(
        "--stt-model",
        help=(
            "Sökväg till en flerspråkig whisper.cpp-modell; "
            "dashboarden startar då en varm lokal server"
        ),
    )
    stt_source.add_argument(
        "--stt-url",
        help=(
            "Loopback-bas-URL till en redan startad whisper.cpp-server"
        ),
    )
    parser.add_argument(
        "--stt-model-id",
        default="whisper-multilingual",
        help=(
            "Säkert visnings-ID för modellen vid --stt-url "
            "(default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--stt-inference-path",
        default="/inference",
        help=(
            "Validerad relativ inferenssökväg vid --stt-url "
            "(default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--stt-whisper-binary",
        default="whisper-server",
        help="whisper-server-binär (default: %(default)s)",
    )
    parser.add_argument(
        "--stt-port",
        type=int,
        default=8178,
        help="Loopback-port för hanterad whisper-server (default: %(default)s)",
    )
    parser.add_argument(
        "--stt-threads",
        type=int,
        default=4,
        help="Whisper-trådar för hanterad server (default: %(default)s)",
    )
    parser.add_argument(
        "--stt-cpu",
        action="store_true",
        help=(
            "Kör hanterad whisper-server på CPU om Metal/GPU redan används"
        ),
    )
    return parser


def _configured_robot_runtime_adapter(args, *, blast_monitor=None):
    """Compose an opt-in adapter without connecting to physical hardware."""

    if args.robot_profile == ROBOT_PROFILE_DISABLED:
        if args.robot_target is not None or args.robot_reset_memory:
            raise ValueError(
                "EV3 target/reset options require --robot-profile "
                "ev3rstorm-01"
            )
        return None
    if args.robot_profile == BLAST_PROFILE_ID:
        if args.robot_target is not None or args.robot_reset_memory:
            raise ValueError(
                "EV3 target/reset options cannot be used with "
                "--robot-profile blast-01"
            )
        return _configured_blast_runtime_adapter(args, blast_monitor)
    if args.robot_profile != EV3RSTORM_PROFILE_ID:
        raise ValueError("physical robot profile is unsupported")
    if args.robot_target is None:
        raise ValueError(
            "--robot-target is required with --robot-profile ev3rstorm-01"
        )
    if (
        not math.isfinite(args.robot_planner_timeout_seconds)
        or not 0.5 <= args.robot_planner_timeout_seconds <= 60.0
    ):
        raise ValueError("physical planner timeout is invalid")

    profile = EV3RSTORMProfile()
    binding = EV3SSHBinding(
        profile_id=profile.descriptor.profile_id,
        target=args.robot_target,
        memory_path=Path(args.robot_memory_path),
        reset_memory=args.robot_reset_memory,
    )

    def planner_factory(model):
        return LMStudioNavigationPlanner(
            base_url=args.lm_studio_url,
            model=model,
            timeout_seconds=args.robot_planner_timeout_seconds,
        )

    return profile.build_adapter(
        binding,
        planner_factory=planner_factory,
    )


def _configured_blast_runtime_adapter(args, blast_monitor):
    """Build BLAST's existing adapter for standalone or combined use."""

    if blast_monitor is None:
        raise ValueError(
            "--blast-hub-name is required with --robot-profile blast-01"
        )
    if (
        not math.isfinite(args.robot_planner_timeout_seconds)
        or not 0.5 <= args.robot_planner_timeout_seconds <= 60.0
    ):
        raise ValueError("physical planner timeout is invalid")

    def planner_factory(model):
        return LMStudioControllerActionPlanner(
            base_url=args.lm_studio_url,
            model=model,
            timeout_seconds=args.robot_planner_timeout_seconds,
            utterance_persona_by_locale=BLAST_PERSONA_BY_LOCALE,
            max_utterance_chars=(
                BLAST_MAX_NAVIGATION_UTTERANCE_CHARS
            ),
        )

    def speech_runtime_factory(*, event_sink):
        synthesizer = LocaleSpeechSynthesizer(
            {
                "sv": PiperLoopbackSynthesizer(
                    profile=BLAST_PIPER_PROFILE,
                ),
                "en": MacOSSayWAVSynthesizer(),
            }
        )
        return RobotSpeechRuntime(
            speaker=BlastHubSpeaker(
                synthesizer,
                blast_monitor,
            ),
            event_sink=event_sink,
            thread_name="blast-01-speech",
        )

    return BlastEpisodeRuntimeAdapter(
        controller=blast_monitor,
        planner_factory=planner_factory,
        max_decisions=64,
        speech_runtime_factory=speech_runtime_factory,
        speech_locales=("sv", "en"),
    )


def _configured_shared_spatial_map(args, *, local_map_provider):
    """Compose an explicit one-shot fixed-start peer map, if requested."""

    option_values = (
        args.shared_peer_port,
        args.shared_peer_access_key_file,
        args.shared_peer_x_mm,
        args.shared_peer_y_mm,
        args.shared_peer_yaw_mdeg,
    )
    if not any(value is not None for value in option_values):
        return None
    if not all(value is not None for value in option_values):
        raise ValueError(
            "Fixed-start shared peer options must be supplied together"
        )
    if args.robot_profile not in (BLAST_PROFILE_ID, EV3RSTORM_PROFILE_ID):
        raise ValueError(
            "Fixed-start shared mapping requires one physical robot profile"
        )
    if local_map_provider is None:
        raise ValueError(
            "Fixed-start shared mapping requires a local physical map"
        )
    if (
        not 1 <= args.shared_peer_port <= 65_535
        or args.shared_peer_port == args.port
        or not -MAX_FRAME_COORDINATE_MM
        <= args.shared_peer_x_mm
        <= MAX_FRAME_COORDINATE_MM
        or not -MAX_FRAME_COORDINATE_MM
        <= args.shared_peer_y_mm
        <= MAX_FRAME_COORDINATE_MM
        or not -180_000 <= args.shared_peer_yaw_mdeg <= 179_999
    ):
        raise ValueError("Fixed-start shared peer geometry is invalid")

    access_key = load_or_create_dashboard_access_key(
        Path(args.shared_peer_access_key_file),
    )
    peer_provider = RemoteSpatialMapProvider(
        args.shared_peer_port,
        access_key,
    )
    if args.robot_profile == BLAST_PROFILE_ID:
        local_identity = BLAST_ROBOT_ID, BLAST_CONTROLLER_ID
        peer_identity = EV3RSTORM_PROFILE_ID, EV3_CONTROLLER_ID
    else:
        local_identity = EV3RSTORM_PROFILE_ID, EV3_CONTROLLER_ID
        peer_identity = BLAST_ROBOT_ID, BLAST_CONTROLLER_ID
    return FixedStartSharedMapProvider(
        local_provider=local_map_provider,
        peer_provider=peer_provider,
        local_robot_id=local_identity[0],
        local_controller_id=local_identity[1],
        peer_robot_id=peer_identity[0],
        peer_controller_id=peer_identity[1],
        peer_tx_mm=args.shared_peer_x_mm,
        peer_ty_mm=args.shared_peer_y_mm,
        peer_yaw_mdeg=args.shared_peer_yaw_mdeg,
    )


def _close_resources(
    server,
    service,
    robot_control_service,
    robot_turn_speech,
    map_runtime,
    blast_monitor,
    whisper_runtime,
    *,
    drain_map: bool,
) -> None:
    """Attempt every owned cleanup even if an earlier one is interrupted."""

    def close_all(resources, operation) -> None:
        if resources is None:
            return
        values = (
            resources
            if isinstance(resources, (tuple, list))
            else (resources,)
        )
        unique = []
        for value in values:
            if value is not None and all(
                value is not existing for existing in unique
            ):
                unique.append(value)
        first_error = None
        for value in unique:
            try:
                operation(value)
            except BaseException as error:
                if first_error is None:
                    first_error = error
        if first_error is not None:
            raise first_error

    try:
        if server is not None:
            server.server_close()
    finally:
        try:
            close_all(
                robot_control_service,
                lambda value: value.shutdown(),
            )
        finally:
            try:
                close_all(
                    robot_turn_speech,
                    lambda value: value.close(drain=False),
                )
            finally:
                try:
                    if service is not None:
                        service.shutdown()
                finally:
                    try:
                        close_all(
                            map_runtime,
                            lambda value: value.close(drain=drain_map),
                        )
                    finally:
                        try:
                            if blast_monitor is not None:
                                blast_monitor.close()
                        finally:
                            if whisper_runtime is not None:
                                whisper_runtime.stop()


def _run(
    argv: Optional[Sequence[str]] = None,
    *,
    robot_runtime_adapter=None,
) -> int:
    args = _parser().parse_args(argv)
    injected_robot_runtime = robot_runtime_adapter is not None
    map_runtime = None
    shared_map_runtime = None
    blast_monitor = None
    whisper_runtime = None
    service = None
    robot_control_service = None
    robot_input_service = None
    robot_turn_speech = None
    server = None
    runtime_entries = []
    robot_control_resources = []
    robot_turn_speech_resources = []
    map_runtime_resources = []
    try:
        if (
            not math.isfinite(args.robot_input_timeout_seconds)
            or not 0.1 <= args.robot_input_timeout_seconds <= 60.0
        ):
            raise ValueError("interactive robot input timeout is invalid")
        if injected_robot_runtime:
            if (
                args.robot_profile != ROBOT_PROFILE_DISABLED
                or args.robot_target is not None
                or args.robot_reset_memory
            ):
                raise ValueError(
                    "Injected robot runtime cannot be combined with CLI "
                    "physical profile options"
                )
        else:
            if args.robot_profile == BLAST_PROFILE_ID and not (
                args.blast_hub_name
            ):
                raise ValueError(
                    "--blast-hub-name is required with "
                    "--robot-profile blast-01"
                )
            if args.blast_hub_name:
                blast_monitor = BlastObservationMonitor(
                    hub_name=args.blast_hub_name,
                )
            robot_runtime_adapter = _configured_robot_runtime_adapter(
                args,
                blast_monitor=blast_monitor,
            )
        if robot_runtime_adapter is not None:
            primary_profile_id = (
                None if injected_robot_runtime else args.robot_profile
            )
            runtime_entries.append(
                (primary_profile_id, robot_runtime_adapter)
            )
        if (
            not injected_robot_runtime
            and args.robot_profile == EV3RSTORM_PROFILE_ID
            and blast_monitor is not None
        ):
            runtime_entries.append((
                BLAST_PROFILE_ID,
                _configured_blast_runtime_adapter(args, blast_monitor),
            ))
        if args.simulation_map_demo and robot_runtime_adapter is not None:
            raise ValueError(
                "--simulation-map-demo cannot be combined with a physical "
                "robot runtime"
            )
        if args.simulation_map_demo:
            from .spatial_mapping_demo import (
                build_simulation_map_demo,
            )

            navigation, plant, map_runtime = (
                build_simulation_map_demo()
            )
            if (
                not navigation.completed
                or not navigation.terminal_stop_verified
                or plant.collision_count
            ):
                raise RuntimeError(
                    "Simulator map demo did not complete safely"
                )
        elif robot_runtime_adapter is not None:
            # Physical map publication is a separate, observation-only
            # capability.  It never travels through RobotRuntimeUpdate and
            # owns no motor authority.  Concrete adapters expose it as an
            # instance attribute so generic injected test doubles cannot
            # accidentally manufacture a provider through dynamic getattr.
            try:
                adapter_state = vars(robot_runtime_adapter)
            except TypeError:
                adapter_state = {}
            candidate = adapter_state.get("spatial_map_provider")
            if (
                candidate is not None
                and callable(getattr(candidate, "snapshot", None))
                and callable(getattr(candidate, "close", None))
            ):
                map_runtime = candidate
                map_runtime_resources.append(candidate)
        for _profile_id, additional_adapter in runtime_entries[1:]:
            try:
                additional_state = vars(additional_adapter)
            except TypeError:
                additional_state = {}
            candidate = additional_state.get("spatial_map_provider")
            if (
                candidate is not None
                and callable(getattr(candidate, "snapshot", None))
                and callable(getattr(candidate, "close", None))
            ):
                map_runtime_resources.append(candidate)
        shared_map_runtime = _configured_shared_spatial_map(
            args,
            local_map_provider=map_runtime,
        )
        speech_transcriber = None
        if args.stt_model:
            from .stt_whisper_cpp import WhisperCppTranscriber
            from .stt_whisper_server import WhisperCppServer

            whisper_runtime = WhisperCppServer(
                args.stt_model,
                binary=args.stt_whisper_binary,
                port=args.stt_port,
                threads=args.stt_threads,
                use_gpu=not args.stt_cpu,
            )
            whisper_runtime.start()
            speech_transcriber = WhisperCppTranscriber(
                base_url=whisper_runtime.base_url,
                model_id=whisper_runtime.model_id,
                require_opaque_path=True,
            )
        elif args.stt_url:
            from .stt_whisper_cpp import WhisperCppTranscriber

            speech_transcriber = WhisperCppTranscriber(
                base_url=args.stt_url,
                inference_path=args.stt_inference_path,
                model_id=args.stt_model_id,
                require_opaque_path=True,
            )
            speech_transcriber.probe()
        if args.blast_hub_name and blast_monitor is None:
            blast_monitor = BlastObservationMonitor(
                hub_name=args.blast_hub_name,
            )
        entry_states = []
        for profile_id, adapter in runtime_entries:
            try:
                state = vars(adapter)
            except TypeError:
                state = {}
            target = (
                state.get("robot_control_target")
                if profile_id is None
                else _configured_robot_control_target(profile_id)
            )
            local_map = state.get("spatial_map_provider")
            if not callable(getattr(local_map, "snapshot", None)):
                local_map = None
            entry_states.append((profile_id, adapter, state, target, local_map))

        controller_runtime_providers = []
        if blast_monitor is not None:
            controller_runtime_providers.append(blast_monitor)
        for _profile_id, _adapter, state, _target, _local_map in entry_states:
            provider = state.get("controller_runtime_provider")
            if (
                provider is not None
                and all(
                    provider is not existing
                    for existing in controller_runtime_providers
                )
            ):
                controller_runtime_providers.append(provider)
        service = DashboardService(
            base_url=args.lm_studio_url,
            model=args.model,
            spatial_map_provider=map_runtime,
            shared_spatial_map_provider=shared_map_runtime,
            speech_transcriber=speech_transcriber,
            controller_runtime_providers=tuple(
                controller_runtime_providers
            ),
        )

        def input_model_factory(profile_id):
            def build_input_model(model):
                options = {
                    "base_url": args.lm_studio_url,
                    "model": model,
                    "timeout_seconds": args.robot_input_timeout_seconds,
                }
                if profile_id == BLAST_PROFILE_ID:
                    options["reply_persona_by_locale"] = (
                        BLAST_PERSONA_BY_LOCALE
                    )
                return LMStudioRobotInputModel(**options)

            return build_input_model

        robot_control_services = {}
        robot_input_services = {}
        robot_spatial_map_providers = {}
        default_robot_id = None
        shared_episode_gate = (
            RobotEpisodeGate() if len(entry_states) > 1 else None
        )
        for profile_id, adapter, state, target, local_map in entry_states:
            control_options = {
                "settings": RobotControlSettings(model=args.model),
                "target": target,
            }
            if shared_episode_gate is not None:
                control_options["episode_gate"] = shared_episode_gate
            control = RobotControlService(
                adapter,
                **control_options,
            )
            robot_control_resources.append(control)
            speech = None
            speech_factory = state.get("speech_runtime_factory")
            speech_locales = state.get("speech_locales", ())
            if callable(speech_factory) and speech_locales:
                speech = RobotTurnSpeechSink(
                    speech_factory,
                    supported_locales=speech_locales,
                )
                robot_turn_speech_resources.append(speech)
            input_service = RobotInputService(
                control_service=control,
                model_factory=input_model_factory(profile_id),
                spatial_map_provider=local_map,
                speech_sink=(
                    speech.submit if speech is not None else None
                ),
            )
            robot_id = target.robot_id
            robot_control_services[robot_id] = control
            robot_input_services[robot_id] = input_service
            if local_map is not None:
                robot_spatial_map_providers[robot_id] = local_map
            if robot_control_service is None:
                robot_control_service = control
                robot_input_service = input_service
                robot_turn_speech = speech
                default_robot_id = robot_id

        if robot_control_service is None:
            robot_control_service = RobotControlService(
                settings=RobotControlSettings(model=args.model),
            )
            robot_control_resources.append(robot_control_service)
            robot_input_service = RobotInputService(
                control_service=robot_control_service,
                model_factory=input_model_factory(None),
                spatial_map_provider=map_runtime,
            )
        controller_control_services = {}
        if blast_monitor is not None:
            controller_control_services["blast-01.hub"] = blast_monitor
        for _profile_id, _adapter, state, _target, _local_map in entry_states:
            reachability = state.get("controller_reachability_service")
            if reachability is not None:
                controller_control_services[
                    "ev3rstorm-01.ev3-main"
                ] = reachability
        server_options = {
            "robot_control_service": robot_control_service,
            "robot_input_service": robot_input_service,
            "controller_control_services": controller_control_services,
            "robot_control_services": robot_control_services,
            "robot_input_services": robot_input_services,
            "robot_spatial_map_providers": (
                robot_spatial_map_providers
            ),
            "default_robot_id": default_robot_id,
        }
        if args.console_access_key_file:
            server_options["session_token"] = (
                load_or_create_dashboard_access_key(
                    Path(args.console_access_key_file),
                )
            )
        server, _router = build_server(
            service,
            args.port,
            **server_options,
        )

        address = "http://{}:{}{}".format(
            LOOPBACK_HOST,
            args.port,
            _router.session_path,
        )
        print(
            json.dumps(
                {
                    "status": "ready",
                    "url": address,
                    "physical_control_enabled": (
                        robot_runtime_adapter is not None
                    ),
                    "robot_profile": (
                        "injected"
                        if injected_robot_runtime
                        else args.robot_profile
                    ),
                    "speech_to_text_enabled": (
                        speech_transcriber is not None
                    ),
                    "blast_observation_enabled": (
                        blast_monitor is not None
                    ),
                    "spatial_map_mode": (
                        "simulation_demo"
                        if args.simulation_map_demo
                        else (
                            "physical_live"
                            if map_runtime is not None
                            else "unavailable"
                        )
                    ),
                    "shared_spatial_map_mode": (
                        "fixed_start_peer"
                        if shared_map_runtime is not None
                        else "unavailable"
                    ),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError) as error:
        print(
            json.dumps(
                {
                    "status": "failed",
                    "error": str(error),
                },
                ensure_ascii=False,
            ),
            file=sys.stderr,
        )
        return 2
    finally:
        _close_resources(
            server,
            service,
            tuple(robot_control_resources),
            tuple(robot_turn_speech_resources),
            (
                tuple(map_runtime_resources)
                if map_runtime_resources
                else map_runtime
            ),
            blast_monitor,
            whisper_runtime,
            drain_map=server is not None,
        )
    return 0


def _raise_termination_interrupt(_signum, _frame) -> None:
    """Route normal service-manager termination through runtime cleanup."""

    raise KeyboardInterrupt


def main(argv: Optional[Sequence[str]] = None) -> int:
    previous_sigterm = None
    if threading.current_thread() is threading.main_thread():
        previous_sigterm = signal.signal(
            signal.SIGTERM,
            _raise_termination_interrupt,
        )
    try:
        return _run(argv)
    finally:
        if previous_sigterm is not None:
            signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    raise SystemExit(main())
