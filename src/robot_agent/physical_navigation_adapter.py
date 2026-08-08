"""Control-service adapter for the bounded physical navigation runtime."""

import threading
from typing import Callable, Mapping, Optional

from .active_ir_scan_contract import ActiveIrScanCalibration
from .navigation_memory_store import NavigationMemoryStore
from .physical_navigation_runtime import (
    DEFAULT_SCAN_TIMEOUT_SECONDS,
    MAX_SCAN_TIMEOUT_SECONDS,
    PhysicalNavigationRuntime,
    PhysicalNavigationRuntimeConfig,
    PhysicalNavigationRuntimeError,
)
from .navigation_plan_tail import PLAN_TAIL_MAX_AGE_SECONDS
from .robot_control_service import RobotEpisodeOutcome


_SPEECH_RESULT_PROGRESS_EVENTS = frozenset((
    "motion_result",
    "scan_result",
))


class PhysicalNavigationRuntimeAdapter:
    """Factory-backed adapter for ``RobotControlService``'s runner seam."""

    def __init__(
        self,
        *,
        transport_factory: Callable[[], object],
        planner_factory: Callable[[str], object],
        memory_factory: Callable[[], NavigationMemoryStore],
        execution_contract,
        scan_executor_factory: Optional[Callable[[object], object]] = None,
        speech_runtime_factory: Optional[Callable[..., object]] = None,
        speech_locales=(),
        spatial_map_bridge=None,
        minimum_forward_progress_mm: int = 420,
        goal_heading_tolerance_mdeg: int = 5_000,
        startup_timeout_seconds: float = 30.0,
        request_timeout_seconds: float = 8.0,
        scan_timeout_seconds: float = float(DEFAULT_SCAN_TIMEOUT_SECONDS),
        plan_tail_max_age_seconds: float = PLAN_TAIL_MAX_AGE_SECONDS,
        active_scan_calibration: ActiveIrScanCalibration = (
            ActiveIrScanCalibration()
        ),
        event_mapper: Optional[
            Callable[[Mapping[str, object]], Optional[Mapping[str, object]]]
        ] = None,
    ):
        for dependency in (
            transport_factory,
            planner_factory,
            memory_factory,
        ):
            if not callable(dependency):
                raise ValueError("runtime adapter factory is invalid")
        if any(
            not callable(getattr(execution_contract, name, None))
            for name in (
                "parse_description",
                "parse_observation",
                "shutdown_verified",
            )
        ):
            raise ValueError("navigation execution contract is invalid")
        if scan_executor_factory is not None and not callable(
            scan_executor_factory
        ):
            raise ValueError("scan executor factory is invalid")
        if speech_runtime_factory is not None and not callable(
            speech_runtime_factory
        ):
            raise ValueError("speech runtime factory is invalid")
        if (
            not isinstance(speech_locales, tuple)
            or len(set(speech_locales)) != len(speech_locales)
            or any(locale not in ("sv", "en") for locale in speech_locales)
            or speech_locales and speech_runtime_factory is None
        ):
            raise ValueError("runtime speech locales are invalid")
        spatial_map_offer = getattr(spatial_map_bridge, "offer", None)
        spatial_map_snapshot = getattr(
            spatial_map_bridge,
            "snapshot",
            None,
        )
        if spatial_map_bridge is not None and (
            not callable(spatial_map_offer)
            or not callable(spatial_map_snapshot)
        ):
            raise ValueError("spatial map bridge is invalid")
        if (
            isinstance(goal_heading_tolerance_mdeg, bool)
            or not isinstance(goal_heading_tolerance_mdeg, int)
            or not 1_000 <= goal_heading_tolerance_mdeg <= 45_000
        ):
            raise ValueError("runtime heading tolerance is invalid")
        if (
            isinstance(startup_timeout_seconds, bool)
            or not isinstance(startup_timeout_seconds, (int, float))
            or not 0.1 <= float(startup_timeout_seconds) <= 60.0
        ):
            raise ValueError("runtime startup timeout is invalid")
        if (
            isinstance(request_timeout_seconds, bool)
            or not isinstance(request_timeout_seconds, (int, float))
            or not 0.1 <= float(request_timeout_seconds) <= 60.0
        ):
            raise ValueError("runtime request timeout is invalid")
        if (
            isinstance(scan_timeout_seconds, bool)
            or not isinstance(scan_timeout_seconds, (int, float))
            or not DEFAULT_SCAN_TIMEOUT_SECONDS
            <= float(scan_timeout_seconds)
            <= MAX_SCAN_TIMEOUT_SECONDS
        ):
            raise ValueError("runtime scan timeout is invalid")
        if (
            isinstance(plan_tail_max_age_seconds, bool)
            or not isinstance(plan_tail_max_age_seconds, (int, float))
            or not 1.0 <= float(plan_tail_max_age_seconds) <= 120.0
        ):
            raise ValueError("runtime plan tail age is invalid")
        if not isinstance(
            active_scan_calibration,
            ActiveIrScanCalibration,
        ):
            raise ValueError("active scan calibration is invalid")
        self.transport_factory = transport_factory
        self.planner_factory = planner_factory
        self.memory_factory = memory_factory
        self.execution_contract = execution_contract
        self.scan_executor_factory = scan_executor_factory
        self.speech_runtime_factory = speech_runtime_factory
        self.speech_locales = speech_locales
        self.spatial_map_bridge = spatial_map_bridge
        # Dashboard composition receives only the bridge's read capability;
        # physical runtime wiring below retains its non-blocking write seam.
        self.spatial_map_provider = spatial_map_bridge
        self._spatial_map_offer = spatial_map_offer
        self.minimum_forward_progress_mm = minimum_forward_progress_mm
        self.goal_heading_tolerance_mdeg = goal_heading_tolerance_mdeg
        self.startup_timeout_seconds = float(startup_timeout_seconds)
        self.request_timeout_seconds = float(request_timeout_seconds)
        self.scan_timeout_seconds = float(scan_timeout_seconds)
        self.plan_tail_max_age_seconds = float(plan_tail_max_age_seconds)
        self.active_scan_calibration = active_scan_calibration
        self.event_mapper = event_mapper or self._dashboard_update
        self._lock = threading.Lock()
        self._active = None

    @staticmethod
    def _dashboard_update(
        event: Mapping[str, object],
    ) -> Optional[Mapping[str, object]]:
        name = event.get("event")
        if name == "model_decision":
            value = {"model_latency_ms": event["model_latency_ms"]}
            for field in (
                "planner_context_bytes",
                "prompt_tokens",
                "completion_tokens",
                "total_tokens",
            ):
                if field in event:
                    value[field] = event[field]
            return value
        if name == "model_decision_committed":
            value = {
                "current_action": event["action"],
                "plan": event["plan"],
                "message": event["assessment"],
            }
            if event.get("utterance"):
                value["message"] = event["utterance"]
            return value
        if name == "decision_vetoed":
            feedback = event.get("validation_feedback")
            feedback = feedback if isinstance(feedback, Mapping) else {}
            code = feedback.get("code", "decision_validation_failed")
            message = feedback.get("message", "Model proposal was rejected")
            return {
                "current_action": None,
                "plan": [],
                "message": "Decision vetoed: {} — {}".format(
                    code,
                    message,
                )[:1_000],
            }
        if name in ("planner_attempt_failed", "planner_termination"):
            code = event.get(
                "planner_error_code",
                event.get("terminal_reason", "planner_unavailable"),
            )
            return {
                "current_action": None,
                "plan": [],
                "message": "Planner unavailable: {}".format(code)[:1_000],
            }
        if name == "speech_offer_failed":
            return {"speech_status": "failed"}
        if (
            isinstance(name, str)
            and name.startswith("speech_")
            and event.get("speech_status") is not None
        ):
            return {"speech_status": event["speech_status"]}
        if name == "motion_result":
            hazards = event["navigation"].get(
                "navigation_hazard_hypotheses",
                [],
            )
            return {
                "current_action": event["action"],
                "obstacle": hazards[-1] if hazards else None,
            }
        if name == "local_detour_route_updated":
            return {"active_route": event.get("route")}
        if name == "scan_result":
            return {"scan": event["scan"]}
        if name == "episode_stopped":
            return {
                "current_action": None,
                "active_route": None,
                "plan": [],
                "message": event["terminal_reason"],
            }
        return None

    @staticmethod
    def _cancel_speech(speech_runtime, episode_id: str) -> None:
        if speech_runtime is None:
            return
        try:
            speech_runtime.cancel_episode(episode_id)
        except Exception:
            # Speech cancellation must not delay or replace motor stop.
            return

    @classmethod
    def _close_speech(cls, speech_runtime, episode_id: str) -> bool:
        if speech_runtime is None:
            return True
        cls._cancel_speech(speech_runtime, episode_id)
        try:
            return speech_runtime.close(
                drain=False,
                timeout_seconds=1.0,
            ) is True
        except Exception:
            # A broken audio backend cannot fault navigation cleanup.
            return False

    def run(self, context) -> RobotEpisodeOutcome:
        reservation = object()
        with self._lock:
            if self._active is not None:
                raise PhysicalNavigationRuntimeError(
                    "runtime_already_active",
                    "A physical navigation runtime is already active",
                )
            # Reserve ownership before creating any per-episode resource. A
            # stop requested during construction is already represented by
            # the context events and will win before worker startup.
            self._active = (
                reservation,
                None,
                context.episode_id,
            )

        speech_action = None
        speech_progress_revision = 0

        def publish(event):
            nonlocal speech_action, speech_progress_revision
            name = event.get("event")
            if name == "model_decision_committed":
                action = event.get("action")
                if action != speech_action:
                    speech_action = action
                    speech_progress_revision += 1
            elif name in _SPEECH_RESULT_PROGRESS_EVENTS:
                speech_progress_revision += 1
            update = self.event_mapper(event)
            if update:
                context.publish(update)

        speech_runtime = None
        runtime = None
        try:
            transport = self.transport_factory()
            planner = self.planner_factory(context.settings.model)
            memory = self.memory_factory()
            scan_executor = (
                None
                if self.scan_executor_factory is None
                else self.scan_executor_factory(transport)
            )

            if (
                getattr(context.settings, "speech_enabled", False) is True
                and self.speech_runtime_factory is not None
            ):
                speech_runtime = self.speech_runtime_factory(
                    event_sink=publish,
                )
                if any(
                    not callable(getattr(speech_runtime, name, None))
                    for name in (
                        "start",
                        "offer",
                        "cancel_episode",
                        "close",
                    )
                ):
                    raise PhysicalNavigationRuntimeError(
                        "invalid_speech_runtime",
                        "Speech runtime factory returned an invalid runtime",
                    )

            def offer_validated_utterance(text):
                return speech_runtime.offer(
                    episode_id=context.episode_id,
                    text=text,
                    locale=context.request.locale,
                    progress_revision=speech_progress_revision,
                    cancel_requested=lambda: (
                        context.stop_requested.is_set()
                        or context.emergency_stop_requested.is_set()
                    ),
                )

            runtime = PhysicalNavigationRuntime(
                episode_id=context.episode_id,
                config=PhysicalNavigationRuntimeConfig(
                    goal=context.request.goal,
                    locale=context.request.locale,
                    minimum_forward_progress_mm=(
                        self.minimum_forward_progress_mm
                    ),
                    goal_heading_tolerance_mdeg=(
                        self.goal_heading_tolerance_mdeg
                    ),
                    max_episode_seconds=(
                        context.settings.max_episode_ms / 1000.0
                    ),
                    startup_timeout_seconds=self.startup_timeout_seconds,
                    request_timeout_seconds=self.request_timeout_seconds,
                    scan_timeout_seconds=self.scan_timeout_seconds,
                    plan_tail_max_age_seconds=(
                        self.plan_tail_max_age_seconds
                    ),
                ),
                transport=transport,
                transport_factory=self.transport_factory,
                execution_contract=self.execution_contract,
                planner=planner,
                memory=memory,
                active_scan_executor=scan_executor,
                active_scan_executor_factory=self.scan_executor_factory,
                active_scan_calibration=self.active_scan_calibration,
                event_sink=publish,
                observation_sink=self._spatial_map_offer,
                validated_utterance_sink=(
                    offer_validated_utterance
                    if speech_runtime is not None
                    else None
                ),
                cancel_event=context.stop_requested,
                emergency_event=context.emergency_stop_requested,
            )
            with self._lock:
                if (
                    self._active is None
                    or self._active[0] is not reservation
                ):
                    raise PhysicalNavigationRuntimeError(
                        "runtime_reservation_lost",
                        "Physical runtime ownership changed during startup",
                    )
                self._active = (
                    runtime,
                    speech_runtime,
                    context.episode_id,
                )

            if speech_runtime is not None:
                speech_runtime.start()
            result = runtime.run()
            return RobotEpisodeOutcome(
                terminal_reason=result.terminal_reason,
                completed=result.completed,
                runtime_update={
                    "current_action": None,
                    "active_route": None,
                    "plan": [],
                    "model_latency_ms": result.model_latency_ms,
                    "message": result.terminal_reason,
                },
            )
        finally:
            speech_closed = self._close_speech(
                speech_runtime,
                context.episode_id,
            )
            with self._lock:
                if (
                    self._active is not None
                    and (
                        self._active[0] is reservation
                        or self._active[0] is runtime
                    )
                ):
                    if speech_closed:
                        self._active = None
                    else:
                        # Do not allow another physical episode to overlap an
                        # unreaped speech worker. Motor cleanup has already
                        # completed; restarting the host remains the explicit
                        # recovery for a broken audio backend.
                        self._active = (
                            runtime if runtime is not None else reservation,
                            speech_runtime,
                            context.episode_id,
                        )

    def request_stop(self) -> None:
        with self._lock:
            active = self._active
        if active is not None:
            runtime, speech_runtime, episode_id = active
            try:
                request_stop = getattr(runtime, "request_stop", None)
                if callable(request_stop):
                    request_stop()
            finally:
                self._cancel_speech(speech_runtime, episode_id)

    def emergency_stop(self) -> None:
        with self._lock:
            active = self._active
        if active is not None:
            runtime, speech_runtime, episode_id = active
            try:
                emergency_stop = getattr(runtime, "emergency_stop", None)
                if callable(emergency_stop):
                    emergency_stop()
            finally:
                self._cancel_speech(speech_runtime, episode_id)
