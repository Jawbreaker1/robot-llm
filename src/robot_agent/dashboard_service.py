"""Application service for the local, motion-free Robot LLM dashboard.

All long-running model and research work is serialized through one bounded
worker.  The service can describe robots and accept conversational goals, but
it deliberately has no dependency on RobotAPI, SSH, TTS, the EV3 supervisor,
or any motor-capable module.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import queue
import secrets
import threading
import time
from typing import Callable, Mapping, Optional, Tuple

from .dashboard_contract import (
    CHAT_MODES,
    RESPONSE_LOCALES,
    ChatMessage,
    ChatTurn,
    DashboardContractError,
    DashboardSettings,
    ExperimentDescriptor,
    NodeDescriptor,
    RobotDescriptor,
    strict_json_object,
)
from .dashboard_state import (
    ConversationStore,
    DashboardStateError,
    EventLog,
    NodeRegistry,
    SettingsStore,
)
from .dashboard_stt import DashboardSTT
from .http_transport import direct_http_request
from .lm_studio import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    _safe_base_url,
    _safe_model,
)
from .lm_studio_research import LMStudioResearchPlanner
from .research import OpenMeteoWeatherTool
from .research_loop import (
    ANSWERED,
    BUDGET_EXHAUSTED,
    CLARIFICATION_REQUIRED,
    INVALID_RESEARCH_GOAL,
    PLANNER_ABORTED,
    PLANNER_FAILED,
    ResearchEpisodeResult,
    ResearchEvidenceEnvelope,
    ResearchGoal,
    ResearchLimits,
    ResearchLoop,
    ResearchToolRegistry,
    TOOL_FAILED,
)


SERVICE_SCHEMA = "dashboard-bootstrap/v1"
RUNTIME_SCHEMA = "dashboard-runtime/v1"
SPATIAL_MAP_SCHEMA = "robot-spatial-map/v1"
MAX_CHAT_JOBS = 8
MAX_HISTORY_MESSAGES = 20
MAX_SPATIAL_MAP_BYTES = 2 * 1024 * 1024
LM_PROBE_TIMEOUT_SECONDS = 2.0
LM_PROBE_MAX_BYTES = 64 * 1024
_LEVEL_ORDER = {
    "debug": 10,
    "info": 20,
    "warning": 30,
    "error": 40,
}

EpisodeRunner = Callable[
    [ChatTurn, Tuple[ChatMessage, ...], int, DashboardSettings],
    ResearchEpisodeResult,
]
ProbeTransport = Callable[..., object]
SpatialMapProvider = Callable[[], object]


class DashboardServiceError(RuntimeError):
    """Typed service failure safe to expose through the local HTTP API."""

    def __init__(self, status: int, code: str, message: str):
        self.status = status
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class _ChatJob:
    turn: ChatTurn
    history: Tuple[ChatMessage, ...]
    conversation_version: int
    settings: DashboardSettings


@dataclass(frozen=True)
class _ContextWithHistory:
    """Planner context adapter carrying structured, visible dialogue only."""

    context: object
    conversation_id: str
    conversation_version: int
    history: Tuple[ChatMessage, ...]
    response_locale: str

    def to_dict(self):
        value = dict(self.context.to_dict())
        value["response_locale"] = self.response_locale
        value["conversation_history"] = {
            "schema": "conversation-history/v1",
            "conversation_id": self.conversation_id,
            "conversation_version": self.conversation_version,
            "messages": [
                {
                    "role": message.role,
                    "content": message.content,
                    "turn_id": message.turn_id,
                }
                for message in self.history[-MAX_HISTORY_MESSAGES:]
            ],
        }
        return value


class _InstrumentedWeatherTool:
    def __init__(self, service: "DashboardService", turn: ChatTurn, tool):
        self._service = service
        self._turn = turn
        self._tool = tool

    def current(self, request):
        self._service._event(
            level="info",
            category="research",
            event_type="research.tool_started",
            source_id="weather.current",
            message="Read-only weather tool started",
            request_id=request.request_id,
            conversation_id=self._turn.conversation_id,
            turn_id=self._turn.turn_id,
            tool_call_id=request.request_id,
            data={"tool_name": "weather.current"},
        )
        try:
            result = self._tool.current(request)
        except Exception:
            self._service._event(
                level="error",
                category="research",
                event_type="research.tool_failed",
                source_id="weather.current",
                message="Read-only weather tool failed",
                request_id=request.request_id,
                conversation_id=self._turn.conversation_id,
                turn_id=self._turn.turn_id,
                tool_call_id=request.request_id,
                data={"tool_name": "weather.current"},
            )
            raise
        self._service._event(
            level="info",
            category="research",
            event_type="research.tool_completed",
            source_id="weather.current",
            message="Read-only weather evidence received",
            request_id=request.request_id,
            conversation_id=self._turn.conversation_id,
            turn_id=self._turn.turn_id,
            tool_call_id=request.request_id,
            data={"tool_name": "weather.current"},
        )
        return result


def _default_registry(server_instance_id: str) -> NodeRegistry:
    ev3_nodes = (
        "ev3rstorm-01.ev3-main",
        "ev3rstorm-01.front-camera",
        "ev3rstorm-01.microphone-array",
    )
    composite_nodes = (
        "lab-composite.robot-inventor",
        "lab-composite.boost-hub",
        "lab-composite.vision",
        "lab-composite.audio",
    )
    robots = (
        RobotDescriptor(
            robot_id="ev3rstorm-01",
            display_name="EV3RSTORM",
            robot_kind="lego-ev3rstorm",
            lifecycle="declared",
            node_ids=ev3_nodes,
        ),
        RobotDescriptor(
            robot_id="lab-composite",
            display_name="Composite Lab Robot",
            robot_kind="multi-controller-lego",
            display_name_key="registry.names.composite_lab_robot",
            lifecycle="declared",
            node_ids=composite_nodes,
        ),
    )
    nodes = (
        NodeDescriptor(
            node_id=ev3_nodes[0],
            display_name="EV3 Main",
            node_kind="controller",
            lifecycle="declared",
            robot_id="ev3rstorm-01",
            controller_id="ev3rstorm-01.ev3-main",
            capabilities=(
                "motor.arm",
                "motor.drive.left",
                "motor.drive.right",
                "sensor.touch",
                "sensor.color",
                "sensor.infrared",
                "speaker.tts",
            ),
            status_reason_code="not_observed",
        ),
        NodeDescriptor(
            node_id=ev3_nodes[1],
            display_name="Front Camera",
            node_kind="camera",
            display_name_key="registry.names.front_camera",
            lifecycle="declared",
            robot_id="ev3rstorm-01",
            source_id="ev3rstorm-01.front-camera",
            capabilities=("vision.frames",),
            status_reason_code="future_component",
        ),
        NodeDescriptor(
            node_id=ev3_nodes[2],
            display_name="Microphone Array",
            node_kind="microphone",
            display_name_key="registry.names.microphone_array",
            lifecycle="declared",
            robot_id="ev3rstorm-01",
            source_id="ev3rstorm-01.microphone-array",
            capabilities=("audio.segments", "audio.direction"),
            status_reason_code="future_component",
        ),
        NodeDescriptor(
            node_id=composite_nodes[0],
            display_name="Robot Inventor 51515",
            node_kind="controller",
            lifecycle="declared",
            robot_id="lab-composite",
            controller_id="lab-composite.robot-inventor",
            capabilities=("hub.motors", "hub.sensors"),
            status_reason_code="future_component",
        ),
        NodeDescriptor(
            node_id=composite_nodes[1],
            display_name="BOOST Move Hub",
            node_kind="controller",
            lifecycle="declared",
            robot_id="lab-composite",
            controller_id="lab-composite.boost-hub",
            capabilities=("hub.motors", "hub.sensors"),
            status_reason_code="future_component",
        ),
        NodeDescriptor(
            node_id=composite_nodes[2],
            display_name="Vision Node",
            node_kind="camera",
            display_name_key="registry.names.vision_node",
            lifecycle="declared",
            robot_id="lab-composite",
            source_id="lab-composite.vision",
            capabilities=("vision.frames", "vision.objects"),
            status_reason_code="future_component",
        ),
        NodeDescriptor(
            node_id=composite_nodes[3],
            display_name="Audio Node",
            node_kind="microphone",
            display_name_key="registry.names.audio_node",
            lifecycle="declared",
            robot_id="lab-composite",
            source_id="lab-composite.audio",
            capabilities=("audio.segments", "audio.direction"),
            status_reason_code="future_component",
        ),
        NodeDescriptor(
            node_id="mac-host",
            display_name="Mac Host",
            node_kind="compute",
            display_name_key="registry.names.mac_host",
            lifecycle="declared",
            capabilities=("agent.host", "dashboard.host"),
            status_reason_code="descriptive_only",
        ),
        NodeDescriptor(
            node_id="lm-studio",
            display_name="LM Studio",
            node_kind="model_server",
            lifecycle="declared",
            capabilities=("llm.structured_planning",),
            status_reason_code="probe_not_run",
        ),
        NodeDescriptor(
            node_id="open-meteo",
            display_name="Open-Meteo",
            node_kind="provider",
            lifecycle="declared",
            source_id="weather.current",
            capabilities=("weather.current",),
            status_reason_code="read_only_provider",
        ),
    )
    return NodeRegistry(
        server_instance_id=server_instance_id,
        robots=robots,
        nodes=nodes,
    )


def _empty_spatial_map() -> Mapping[str, object]:
    """Return an honest snapshot until a spatial store is connected."""

    return {
        "schema": SPATIAL_MAP_SCHEMA,
        "status": "unavailable",
        "reason_code": "no_spatial_map_provider",
        "read_only": True,
        "robot_id": None,
        "frame_id": None,
        "map_version": None,
        "based_on_state_version": None,
        "based_on_world_model_version": None,
        "captured_at_unix_ms": None,
        "source_id": None,
        "provenance": None,
        "bounds": None,
        "resolution_mm": None,
        "robot_pose": None,
        "pose_history": [],
        "pose_history_evicted": 0,
        "cells": [],
        "sensor_rays": [],
        "qualitative_observations": [],
        "object_hypotheses": [],
    }


def _service_status_for(code: str) -> int:
    if code in (
        "conversation_not_found",
        "turn_not_found",
    ):
        return 404
    if code in (
        "settings_revision_conflict",
        "conversation_version_conflict",
        "conversation_turn_active",
        "idempotency_conflict",
        "invalid_turn_transition",
        "active_turn_mismatch",
        "duplicate_generated_id",
    ):
        return 409
    return 400


class DashboardService:
    """Thread-safe application facade used by the local HTTP router."""

    def __init__(
        self,
        *,
        base_url: str = DEFAULT_BASE_URL,
        model: str = DEFAULT_MODEL,
        queue_capacity: int = MAX_CHAT_JOBS,
        episode_runner: Optional[EpisodeRunner] = None,
        planner_factory=LMStudioResearchPlanner,
        weather_factory=OpenMeteoWeatherTool,
        probe_transport: ProbeTransport = direct_http_request,
        server_instance_id: Optional[str] = None,
        spatial_map_provider: Optional[SpatialMapProvider] = None,
        speech_transcriber=None,
    ):
        spatial_snapshot = getattr(
            spatial_map_provider,
            "snapshot",
            None,
        )
        if (
            spatial_map_provider is not None
            and not callable(spatial_map_provider)
            and not callable(spatial_snapshot)
        ):
            raise DashboardServiceError(
                500,
                "invalid_service_configuration",
                "Dashboard service configuration is invalid",
            )
        if (
            isinstance(queue_capacity, bool)
            or not isinstance(queue_capacity, int)
            or not 1 <= queue_capacity <= 128
            or episode_runner is not None
            and not callable(episode_runner)
            or not callable(planner_factory)
            or not callable(weather_factory)
            or not callable(probe_transport)
        ):
            raise DashboardServiceError(
                500,
                "invalid_service_configuration",
                "Dashboard service configuration is invalid",
            )
        self._base_url = _safe_base_url(base_url)
        self._model = _safe_model(model)
        self._server_instance_id = (
            server_instance_id
            if server_instance_id is not None
            else "dashboard-{}".format(secrets.token_hex(8))
        )
        self._settings_store = SettingsStore()
        self._events = EventLog(self._server_instance_id)
        self._stt = DashboardSTT(
            speech_transcriber,
            event_sink=self._stt_event,
        )
        self._registry = _default_registry(self._server_instance_id)
        self._conversations = ConversationStore()
        self._episode_runner = episode_runner
        self._planner_factory = planner_factory
        self._weather_factory = weather_factory
        self._probe_transport = probe_transport
        if spatial_map_provider is None:
            self._spatial_map_provider = _empty_spatial_map
        elif callable(spatial_snapshot):
            self._spatial_map_provider = spatial_snapshot
        else:
            self._spatial_map_provider = spatial_map_provider
        self._jobs = queue.Queue(maxsize=queue_capacity)
        self._job_slots = threading.BoundedSemaphore(queue_capacity)
        self._stop_requested = threading.Event()
        self._submit_lock = threading.RLock()
        self._request_index = {}
        self._details_lock = threading.RLock()
        self._turn_details = {}
        self._runtime_lock = threading.RLock()
        self._probe_generation = 0
        self._lm_runtime = {
            "schema": RUNTIME_SCHEMA,
            "state": "unknown",
            "base_url": self._base_url,
            "model": self._model,
            "configured_model_loaded": None,
            "last_probe_at_unix_ms": None,
            "error_code": None,
        }
        self._accepting = True
        self._shutdown_lock = threading.RLock()
        self._cancelled_lock = threading.Lock()
        self._queued_cancelled_total = 0
        self._worker = threading.Thread(
            target=self._work,
            name="robot-llm-dashboard-research",
            daemon=True,
        )
        self._worker.start()
        self._event(
            level="info",
            category="dashboard",
            event_type="dashboard.started",
            source_id="mac-host",
            message="Motion-free dashboard service started",
            data={
                "physical_control_enabled": False,
                "queue_capacity": queue_capacity,
            },
        )

    @property
    def server_instance_id(self) -> str:
        return self._server_instance_id

    def _event(self, **values):
        level = values.get("level")
        minimum = self._settings_store.snapshot().log_level
        if (
            level in _LEVEL_ORDER
            and _LEVEL_ORDER[level] < _LEVEL_ORDER[minimum]
        ):
            return None
        return self._events.append(**values)

    def _stt_event(
        self,
        event_type: str,
        data: Mapping[str, object],
    ) -> None:
        """Record bounded STT metadata without audio or transcript text."""

        safe_data = {
            key: value
            for key, value in data.items()
            if key
            in {
                "transcription_id",
                "language_hint",
                "audio_duration_ms",
                "provider_id",
                "status",
                "error_code",
                "provider_latency_ms",
                "previous_status",
                "reason_code",
                "provider_work_pending",
                "late_result_discarded",
                "cancelled_total",
            }
        }
        request_id = data.get("request_id")
        self._event(
            level=(
                "warning"
                if event_type == "stt.transcription_failed"
                else "info"
            ),
            category="perception",
            event_type=event_type,
            source_id="dashboard.microphone",
            message={
                "stt.transcription_queued": (
                    "Speech transcription queued"
                ),
                "stt.transcription_started": (
                    "Local speech transcription started"
                ),
                "stt.transcription_completed": (
                    "Local speech transcription completed"
                ),
                "stt.transcription_failed": (
                    "Local speech transcription failed"
                ),
                "stt.transcription_cancelled": (
                    "Speech transcription cancelled"
                ),
                "stt.transcription_expired": (
                    "Speech transcription result expired"
                ),
                "stt.transcription_late_result_discarded": (
                    "Late speech provider result discarded"
                ),
                "stt.runtime_shutdown": (
                    "Speech transcription runtime stopping"
                ),
            }.get(event_type, "Speech transcription state changed"),
            request_id=(
                request_id
                if isinstance(request_id, str)
                else None
            ),
            data=safe_data,
        )

    def _translate(self, error: Exception):
        if isinstance(error, DashboardServiceError):
            raise error
        if isinstance(error, (DashboardStateError, DashboardContractError)):
            code = getattr(error, "code", "invalid_request")
            raise DashboardServiceError(
                _service_status_for(code),
                code,
                str(error),
            ) from None
        raise error

    def settings(self):
        return self._settings_store.snapshot().to_dict()

    def update_settings(
        self,
        expected_revision: int,
        changes: Mapping[str, object],
    ):
        try:
            updated = self._settings_store.update(
                expected_revision,
                changes,
            )
        except Exception as error:
            self._translate(error)
        self._event(
            level="info",
            category="dashboard",
            event_type="settings.updated",
            source_id="mac-host",
            message="Dashboard session settings updated",
            data={
                "revision": updated.revision,
                "changed_fields": sorted(changes),
            },
        )
        return updated.to_dict()

    def registry(self):
        return self._registry.snapshot().to_dict()

    def bootstrap(self):
        with self._runtime_lock:
            runtime = dict(self._lm_runtime)
        return {
            "schema": SERVICE_SCHEMA,
            "api_version": "robot-dashboard/v1",
            "server_instance_id": self._server_instance_id,
            "physical_control_enabled": False,
            "capabilities": {
                "chat": True,
                "research": ["weather.current"],
                "spatial_map": "read_only",
                "speech_to_text": self._stt.capability(),
                # This object describes only the conversational workbench.
                # Physical robot control is a separate, explicitly injected
                # service with its own status endpoint and state machine.
                "workbench": {
                    "schema": "dashboard-workbench-capabilities/v1",
                    "tool_effects": "read_only",
                    "physical_control": False,
                    "ssh": False,
                    "tts": False,
                },
                "physical_control": False,
                "ssh": False,
                "tts": False,
            },
            "settings": self.settings(),
            "registry": self.registry(),
            "experiments": self.experiments(),
            "runtime": {
                "lm_studio": runtime,
                "speech_to_text": self._stt.runtime_view(),
                "ev3": {
                    "state": "unobserved",
                    "reason_code": "physical_probe_not_run",
                },
            },
        }

    def submit_transcription(
        self,
        request_id: str,
        language_hint: str,
        wav_bytes: bytes,
    ):
        return self._stt.submit(
            request_id,
            language_hint,
            wav_bytes,
        )

    def get_transcription(self, transcription_id: str):
        return self._stt.get(transcription_id)

    def cancel_transcription(self, transcription_id: str):
        return self._stt.cancel(transcription_id)

    def cancel_transcription_request(self, request_id: str):
        return self._stt.cancel_request(request_id)

    def probe_speech_transcriber(self):
        return self._stt.probe()

    def spatial_map(self):
        """Return one detached, finite JSON map snapshot.

        The provider is observation-only.  Its result is copied through strict
        JSON so callers cannot mutate provider state through the dashboard.
        """

        try:
            supplied = self._spatial_map_provider()
            to_dict = getattr(supplied, "to_dict", None)
            if callable(to_dict):
                supplied = to_dict()
            encoded = json.dumps(
                supplied,
                ensure_ascii=False,
                allow_nan=False,
                separators=(",", ":"),
                sort_keys=True,
            ).encode("utf-8")
            if len(encoded) > MAX_SPATIAL_MAP_BYTES:
                raise ValueError("map snapshot is too large")
            snapshot = strict_json_object(encoded)
            if (
                snapshot.get("schema") != SPATIAL_MAP_SCHEMA
                or snapshot.get("read_only") is not True
            ):
                raise ValueError("map snapshot contract mismatch")
        except DashboardServiceError:
            raise
        except Exception:
            raise DashboardServiceError(
                503,
                "spatial_map_unavailable",
                "Spatial map snapshot is unavailable",
            ) from None
        return snapshot

    @staticmethod
    def experiments():
        descriptors = (
            ExperimentDescriptor(
                experiment_id="EXP-F1-IR-DYN-002",
                title_key="experiments.curated.dynamic_ir.title",
                summary_key="experiments.curated.dynamic_ir.summary",
                status="verified",
                component_ids=(
                    "ev3rstorm-01.ev3-main",
                ),
            ),
            ExperimentDescriptor(
                experiment_id="READONLY-WEATHER-001",
                title_key="experiments.curated.weather_tool.title",
                summary_key="experiments.curated.weather_tool.summary",
                status="verified",
                component_ids=(
                    "lm-studio",
                    "open-meteo",
                ),
            ),
            ExperimentDescriptor(
                experiment_id="EXP-F5-IDLE-SIM-005",
                title_key="experiments.curated.idle_autonomy.title",
                summary_key=(
                    "experiments.curated.idle_autonomy.summary"
                ),
                status="verified",
                component_ids=(
                    "mac-host",
                    "lm-studio",
                ),
            ),
            ExperimentDescriptor(
                experiment_id="EV3-FOREGROUND-PREFLIGHT",
                title_key="experiments.curated.ev3_preflight.title",
                summary_key="experiments.curated.ev3_preflight.summary",
                status="waiting",
                component_ids=(
                    "ev3rstorm-01.ev3-main",
                ),
            ),
        )
        return [descriptor.to_dict() for descriptor in descriptors]

    def create_conversation(self, title: Optional[str] = None):
        try:
            conversation = self._conversations.create(title)
        except Exception as error:
            self._translate(error)
        self._event(
            level="info",
            category="agent",
            event_type="chat.conversation_created",
            source_id="dashboard.chat",
            message="Local conversation created",
            conversation_id=conversation.conversation_id,
            data={"context_mode": conversation.context_mode},
        )
        return conversation.to_dict()

    def get_conversation(self, conversation_id: str):
        try:
            return self._conversations.get(conversation_id).to_dict()
        except Exception as error:
            self._translate(error)

    def submit_turn(
        self,
        conversation_id: str,
        client_request_id: str,
        expected_conversation_version: int,
        content: str,
        mode: str,
        response_locale: str,
    ):
        if mode not in CHAT_MODES:
            raise DashboardServiceError(
                400,
                "invalid_chat_mode",
                "Chat turn mode is unsupported",
            )
        if response_locale not in RESPONSE_LOCALES:
            raise DashboardServiceError(
                400,
                "invalid_response_locale",
                "Chat response locale is unsupported",
            )
        request_key = (conversation_id, client_request_id)
        with self._submit_lock:
            existing_id = self._request_index.get(request_key)
            if existing_id is not None:
                try:
                    existing = self._conversations.get_turn(existing_id)
                except Exception as error:
                    self._translate(error)
                if (
                    existing.content != content
                    or existing.mode != mode
                    or existing.response_locale != response_locale
                ):
                    raise DashboardServiceError(
                        409,
                        "idempotency_conflict",
                        "Client request ID was reused with other content",
                    )
                return self._public_turn(existing)
            if not self._accepting:
                raise DashboardServiceError(
                    503,
                    "service_stopping",
                    "Dashboard service is stopping",
                )
            if not self._job_slots.acquire(blocking=False):
                raise DashboardServiceError(
                    429,
                    "chat_queue_full",
                    "Dashboard chat queue is full",
                )
            try:
                settings = self._settings_store.snapshot()
                before = self._conversations.get(conversation_id)
                updated, turn, created = self._conversations.submit_turn(
                    conversation_id,
                    client_request_id,
                    expected_conversation_version,
                    content,
                    mode,
                    response_locale,
                    settings.revision,
                )
                if not created:
                    self._job_slots.release()
                    return self._public_turn(turn)
                job = _ChatJob(
                    turn=turn,
                    history=before.messages[-MAX_HISTORY_MESSAGES:],
                    conversation_version=before.version,
                    settings=settings,
                )
                self._jobs.put_nowait(job)
                self._request_index[request_key] = turn.turn_id
            except Exception as error:
                self._job_slots.release()
                self._translate(error)
        self._event(
            level="info",
            category="agent",
            event_type="chat.turn_queued",
            source_id="dashboard.chat",
            message="Local chat turn queued",
            request_id=client_request_id,
            conversation_id=conversation_id,
            turn_id=turn.turn_id,
            data={
                "mode": mode,
                "response_locale": response_locale,
                "settings_revision": settings.revision,
                "conversation_version": updated.version,
            },
        )
        return self._public_turn(turn)

    def get_turn(self, turn_id: str):
        try:
            turn = self._conversations.get_turn(turn_id)
        except Exception as error:
            self._translate(error)
        return self._public_turn(turn)

    def _public_turn(self, turn: ChatTurn):
        value = dict(turn.to_dict())
        with self._details_lock:
            details = self._turn_details.get(turn.turn_id)
            if details is not None:
                value["episode"] = details
        return value

    def events(self, after_sequence: int, limit: int):
        try:
            return self._events.page(after_sequence, limit).to_dict()
        except Exception as error:
            self._translate(error)

    def _work(self):
        while True:
            try:
                job = self._jobs.get(timeout=0.1)
            except queue.Empty:
                if self._stop_requested.is_set():
                    break
                continue
            try:
                self._run_job(job)
            finally:
                self._job_slots.release()
                self._jobs.task_done()

    def _fail_queued_job(self, job: _ChatJob) -> bool:
        """Fail one not-yet-started job without invoking its runner."""

        turn = job.turn
        try:
            _conversation, failed = self._conversations.fail_queued(
                turn.turn_id,
                "service_stopping",
            )
        except Exception:
            return False
        with self._cancelled_lock:
            self._queued_cancelled_total += 1
        self._event(
            level="warning",
            category="agent",
            event_type="chat.turn_cancelled",
            source_id="dashboard.chat",
            message="Queued chat turn cancelled during shutdown",
            request_id=turn.client_request_id,
            conversation_id=turn.conversation_id,
            turn_id=turn.turn_id,
            data={
                "status": failed.status,
                "error_code": failed.error_code,
            },
        )
        return True

    def _drain_queued_jobs(self) -> None:
        while True:
            try:
                job = self._jobs.get_nowait()
            except queue.Empty:
                return
            try:
                self._fail_queued_job(job)
            finally:
                self._job_slots.release()
                self._jobs.task_done()

    def _run_job(self, job: _ChatJob):
        turn = job.turn
        try:
            with self._submit_lock:
                if self._stop_requested.is_set():
                    should_cancel = True
                    running = None
                else:
                    should_cancel = False
                    _conversation, running = (
                        self._conversations.mark_running(
                            turn.turn_id
                        )
                    )
            if should_cancel:
                self._fail_queued_job(job)
                return
            self._event(
                level="info",
                category="agent",
                event_type="chat.turn_started",
                source_id="dashboard.chat",
                message="Local chat turn started",
                request_id=turn.client_request_id,
                conversation_id=turn.conversation_id,
                turn_id=turn.turn_id,
                data={"settings_revision": turn.settings_revision},
            )
            if self._episode_runner is None:
                result = self._run_research_episode(
                    running,
                    job.history,
                    job.conversation_version,
                    job.settings,
                )
            else:
                result = self._episode_runner(
                    running,
                    job.history,
                    job.conversation_version,
                    job.settings,
                )
            if not isinstance(result, ResearchEpisodeResult):
                raise TypeError("Episode runner returned invalid result")
            self._validate_episode_result(turn, result, job.settings)
            details = self._episode_view(result)
            with self._details_lock:
                self._turn_details[turn.turn_id] = details
            if (
                result.termination == ANSWERED
                and result.answer_text is not None
            ):
                _conversation, completed = (
                    self._conversations.complete_answer(
                        turn.turn_id,
                        result.answer_text,
                        result.citation_ids,
                    )
                )
                event_type = "chat.turn_answered"
                message = "Local chat turn answered"
            elif (
                result.termination == CLARIFICATION_REQUIRED
                and result.clarification_question is not None
            ):
                _conversation, completed = (
                    self._conversations.complete_clarification(
                        turn.turn_id,
                        result.clarification_question,
                    )
                )
                event_type = "chat.turn_clarification_required"
                message = "Local chat turn needs clarification"
            else:
                error_code = "episode_{}".format(
                    result.termination.lower()
                )[:64]
                _conversation, completed = (
                    self._conversations.complete_failed(
                        turn.turn_id,
                        error_code,
                    )
                )
                event_type = "chat.turn_failed"
                message = "Local chat turn failed closed"
            self._event(
                level=(
                    "info"
                    if completed.status
                    in ("answered", "clarification_required")
                    else "error"
                ),
                category="agent",
                event_type=event_type,
                source_id="dashboard.chat",
                message=message,
                request_id=turn.client_request_id,
                conversation_id=turn.conversation_id,
                turn_id=turn.turn_id,
                data={
                    "status": completed.status,
                    "planner_turns": result.planner_turns,
                    "tool_calls": result.tool_calls,
                    "replans": result.replans,
                },
            )
        except Exception:
            try:
                current = self._conversations.get_turn(turn.turn_id)
                if current.status == "queued":
                    self._conversations.mark_running(turn.turn_id)
                    current = self._conversations.get_turn(turn.turn_id)
                if current.status == "running":
                    self._conversations.complete_failed(
                        turn.turn_id,
                        "episode_failed",
                    )
            except Exception:
                pass
            self._event(
                level="error",
                category="agent",
                event_type="chat.turn_failed",
                source_id="dashboard.chat",
                message="Local chat turn failed closed",
                request_id=turn.client_request_id,
                conversation_id=turn.conversation_id,
                turn_id=turn.turn_id,
                data={"status": "failed"},
            )

    @staticmethod
    def _validate_episode_result(
        turn: ChatTurn,
        result: ResearchEpisodeResult,
        settings: DashboardSettings,
    ) -> None:
        """Revalidate the complete runner result before persisting a turn."""

        if result.turn_id != turn.turn_id:
            raise ValueError(
                "Episode result identity did not match the active turn"
            )
        if (
            result.termination
            not in {
                ANSWERED,
                CLARIFICATION_REQUIRED,
                PLANNER_ABORTED,
                PLANNER_FAILED,
                TOOL_FAILED,
                BUDGET_EXHAUSTED,
                INVALID_RESEARCH_GOAL,
            }
            or
            type(result.completed) is not bool
            or isinstance(result.planner_turns, bool)
            or not isinstance(result.planner_turns, int)
            or not 0 <= result.planner_turns <= 100
            or isinstance(result.tool_calls, bool)
            or not isinstance(result.tool_calls, int)
            or not 0 <= result.tool_calls <= 100
            or isinstance(result.replans, bool)
            or not isinstance(result.replans, int)
            or not 0 <= result.replans <= 100
            or isinstance(result.final_context_version, bool)
            or not isinstance(result.final_context_version, int)
            or not 1 <= result.final_context_version <= 2**63 - 1
            or not isinstance(result.trace, tuple)
            or len(result.trace) > 1_000
            or any(
                not isinstance(item, str)
                or not item
                or len(item) > 128
                for item in result.trace
            )
            or not isinstance(result.evidence, tuple)
            or any(
                not isinstance(item, ResearchEvidenceEnvelope)
                for item in result.evidence
            )
            or not isinstance(result.citation_ids, tuple)
            or len(result.citation_ids) > 32
            or len(set(result.citation_ids))
            != len(result.citation_ids)
            or any(
                not isinstance(item, str)
                or not item
                or len(item) > 128
                for item in result.citation_ids
            )
            or result.planner_turns > settings.max_planner_turns
            or result.tool_calls > settings.max_tool_calls
            or result.replans > settings.max_replans
        ):
            raise ValueError("Episode result contract is invalid")

        evidence_by_id = {
            item.evidence_id: item
            for item in result.evidence
        }
        if len(evidence_by_id) != len(result.evidence):
            raise ValueError("Episode evidence identities are invalid")
        if len(result.evidence) > result.tool_calls:
            raise ValueError("Episode evidence exceeds executed tool calls")
        if any(
            citation_id not in evidence_by_id
            for citation_id in result.citation_ids
        ):
            raise ValueError("Episode citations are not bound to evidence")
        now_ms = time.monotonic_ns() // 1_000_000
        if any(
            not evidence_by_id[citation_id].fresh(now_ms)
            for citation_id in result.citation_ids
        ):
            raise ValueError("Episode citation evidence is stale")

        if result.termination == ANSWERED:
            valid_shape = (
                result.completed
                and isinstance(result.answer_text, str)
                and bool(result.answer_text.strip())
                and result.answer_text == result.answer_text.strip()
                and len(result.answer_text) <= 4_000
                and result.clarification_question is None
                and (
                    turn.mode != "research_required"
                    and result.tool_calls == 0
                    or bool(result.citation_ids)
                )
            )
        elif result.termination == CLARIFICATION_REQUIRED:
            valid_shape = (
                not result.completed
                and result.answer_text is None
                and isinstance(result.clarification_question, str)
                and bool(result.clarification_question.strip())
                and result.clarification_question
                == result.clarification_question.strip()
                and len(result.clarification_question) <= 1_000
                and not result.citation_ids
            )
        else:
            valid_shape = (
                not result.completed
                and result.answer_text is None
                and result.clarification_question is None
                and not result.citation_ids
            )
        if not valid_shape:
            raise ValueError(
                "Episode result fields do not match its termination"
            )

    def _run_research_episode(
        self,
        turn: ChatTurn,
        history: Tuple[ChatMessage, ...],
        conversation_version: int,
        settings: DashboardSettings,
    ) -> ResearchEpisodeResult:
        timeout_seconds = min(
            30.0,
            settings.max_planner_latency_ms / 1_000.0,
        )
        base_planner = self._planner_factory(
            base_url=self._base_url,
            model=self._model,
            timeout_seconds=timeout_seconds,
        )

        def planner(context):
            self._event(
                level="info",
                category="model",
                event_type="research.planner_started",
                source_id="lm-studio",
                message="Local structured planner started",
                request_id=turn.client_request_id,
                conversation_id=turn.conversation_id,
                turn_id=turn.turn_id,
                data={
                    "planner_turn": context.planner_turn,
                    "context_version": context.context_version,
                },
            )
            wrapped = _ContextWithHistory(
                context=context,
                conversation_id=turn.conversation_id,
                conversation_version=conversation_version,
                history=history,
                response_locale=turn.response_locale,
            )
            try:
                decision = base_planner(wrapped)
            except Exception:
                self._event(
                    level="error",
                    category="model",
                    event_type="research.planner_failed",
                    source_id="lm-studio",
                    message="Local structured planner failed",
                    request_id=turn.client_request_id,
                    conversation_id=turn.conversation_id,
                    turn_id=turn.turn_id,
                    data={
                        "planner_turn": context.planner_turn,
                        "context_version": context.context_version,
                    },
                )
                raise
            self._event(
                level="info",
                category="model",
                event_type="research.planner_completed",
                source_id="lm-studio",
                message="Local structured planner completed",
                request_id=turn.client_request_id,
                conversation_id=turn.conversation_id,
                turn_id=turn.turn_id,
                data={
                    "planner_turn": context.planner_turn,
                    "context_version": context.context_version,
                },
            )
            return decision

        weather = _InstrumentedWeatherTool(
            self,
            turn,
            self._weather_factory(),
        )
        loop = ResearchLoop(
            planner=planner,
            tools=ResearchToolRegistry(weather),
            clock_ms=lambda: time.monotonic_ns() // 1_000_000,
            limits=ResearchLimits(
                **dict(settings.to_research_limits_kwargs())
            ),
        )
        return loop.run(
            ResearchGoal(
                turn_id=turn.turn_id,
                user_query=turn.content,
                require_evidence=turn.mode == "research_required",
            )
        )

    @staticmethod
    def _episode_view(result: ResearchEpisodeResult):
        return {
            "termination": result.termination,
            "completed": result.completed,
            "planner_turns": result.planner_turns,
            "tool_calls": result.tool_calls,
            "replans": result.replans,
            "final_context_version": result.final_context_version,
            "citation_ids": list(result.citation_ids),
            "trace": list(result.trace),
            "evidence": [
                evidence.to_dict()
                for evidence in result.evidence
            ],
        }

    def probe_lm_studio(self):
        with self._runtime_lock:
            self._probe_generation += 1
            generation = self._probe_generation
        probed_at = time.time_ns() // 1_000_000
        state = "offline"
        loaded = None
        error_code = "lm_studio_unreachable"
        try:
            response = self._probe_transport(
                "GET",
                self._base_url + "/v1/models",
                {"Accept": "application/json"},
                None,
                LM_PROBE_TIMEOUT_SECONDS,
                LM_PROBE_MAX_BYTES,
            )
            status_code = getattr(response, "status_code", None)
            body = getattr(response, "body", None)
            if status_code != 200 or not isinstance(body, bytes):
                error_code = "lm_studio_probe_http"
            elif len(body) > LM_PROBE_MAX_BYTES:
                error_code = "lm_studio_probe_too_large"
            else:
                payload = strict_json_object(body)
                data = payload.get("data")
                if not isinstance(data, list) or len(data) > 1_000:
                    error_code = "lm_studio_probe_protocol"
                else:
                    model_ids = []
                    for item in data:
                        if (
                            not isinstance(item, dict)
                            or not isinstance(item.get("id"), str)
                            or not item["id"]
                            or len(item["id"]) > 200
                        ):
                            raise DashboardContractError(
                                "lm_studio_probe_protocol",
                                "LM Studio model list is invalid",
                            )
                        model_ids.append(item["id"])
                    state = "online"
                    loaded = self._model in model_ids
                    error_code = None
        except Exception:
            state = "offline"
            loaded = None
            error_code = "lm_studio_unreachable"

        with self._runtime_lock:
            if generation != self._probe_generation:
                return dict(self._lm_runtime)
            previous_state = self._lm_runtime["state"]
            self._lm_runtime = {
                "schema": RUNTIME_SCHEMA,
                "state": state,
                "base_url": self._base_url,
                "model": self._model,
                "configured_model_loaded": loaded,
                "last_probe_at_unix_ms": probed_at,
                "error_code": error_code,
            }
            value = dict(self._lm_runtime)
            if previous_state != state:
                self._event(
                    level=(
                        "info"
                        if state == "online"
                        else "warning"
                    ),
                    category="model",
                    event_type="runtime.lm_studio_state_changed",
                    source_id="lm-studio",
                    message=(
                        "LM Studio is reachable"
                        if state == "online"
                        else "LM Studio is not reachable"
                    ),
                    data={
                        "state": state,
                        "configured_model_loaded": loaded,
                    },
                )
        return value

    def shutdown(self, timeout_seconds: float = 1.0):
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or timeout_seconds < 0
            or not math.isfinite(float(timeout_seconds))
        ):
            raise DashboardServiceError(
                500,
                "invalid_shutdown_timeout",
                "Dashboard shutdown timeout is invalid",
            )
        with self._shutdown_lock:
            with self._submit_lock:
                self._accepting = False
                self._stop_requested.set()
            deadline = time.monotonic() + float(timeout_seconds)
            self._drain_queued_jobs()
            stt_shutdown = self._stt.shutdown(
                timeout_seconds=max(
                    0.0,
                    deadline - time.monotonic(),
                )
            )
            if self._worker is not threading.current_thread():
                self._worker.join(
                    timeout=max(0.0, deadline - time.monotonic())
                )
            worker_alive = self._worker.is_alive()
            stt_worker_alive = stt_shutdown["worker_alive"]
            stt_event_worker_alive = stt_shutdown[
                "event_worker_alive"
            ]
            with self._cancelled_lock:
                cancelled_total = self._queued_cancelled_total
            return {
                "schema": "dashboard-shutdown/v1",
                "accepting": False,
                "stop_requested": True,
                "worker_alive": worker_alive,
                "stt_worker_alive": stt_worker_alive,
                "stt_event_worker_alive": stt_event_worker_alive,
                "timed_out": (
                    worker_alive or stt_shutdown["timed_out"]
                ),
                "queued_cancelled_total": cancelled_total,
                "queued_remaining": self._jobs.qsize(),
                "stt_queued_remaining": stt_shutdown[
                    "queued_remaining"
                ],
                "stt_provider_work_pending": stt_shutdown[
                    "provider_work_pending"
                ],
                "stt_event_dropped_total": stt_shutdown[
                    "event_dropped_total"
                ],
            }
