"""Thread-safe orchestration boundary for one physical robot episode.

Only the injected runtime adapter may reach the navigation implementation.
The dashboard talks to this service using typed requests and immutable
snapshots, while stop signals remain independent of the background runner.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import secrets
import threading
import time
from typing import Callable, Mapping, Optional

from .dashboard_contract import DashboardContractError
from .robot_control_contract import (
    ACTIVE_STATES,
    DISABLED,
    EVENT_PAGE_SCHEMA,
    FAULTED,
    IDLE,
    RUNNING,
    SNAPSHOT_PAGE_SCHEMA,
    STARTING,
    STOPPING,
    RobotControlSettings,
    RobotControlSnapshot,
    RobotEpisodeStart,
    RobotRuntimeUpdate,
    finite_unix_ms,
)


MAX_EVENTS = 512
MAX_SNAPSHOTS = 128
MAX_REQUEST_HISTORY = 128
MAX_ERROR_MESSAGE_CHARACTERS = 240

Clock = Callable[[], int]
IDFactory = Callable[[], str]


class RobotControlServiceError(RuntimeError):
    """A typed control failure safe to expose through the loopback API."""

    def __init__(self, status: int, code: str, message: str):
        self.status = status
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class RobotEpisodeContext:
    """The runtime adapter's complete per-episode authority."""

    episode_id: str
    request: RobotEpisodeStart
    settings: RobotControlSettings
    stop_requested: threading.Event
    emergency_stop_requested: threading.Event
    publish: Callable[[Mapping[str, object]], None]


@dataclass(frozen=True)
class RobotEpisodeOutcome(Mapping[str, object]):
    """Typed adapter result with an out-of-band episode disposition."""

    terminal_reason: str
    completed: bool
    runtime_update: Mapping[str, object]

    def __post_init__(self) -> None:
        if (
            not isinstance(self.terminal_reason, str)
            or not self.terminal_reason
            or self.terminal_reason != self.terminal_reason.strip()
            or len(self.terminal_reason) > 128
            or any(ord(character) < 32 for character in self.terminal_reason)
            or not isinstance(self.completed, bool)
            or not isinstance(self.runtime_update, Mapping)
            or not self.runtime_update
        ):
            raise ValueError("Robot episode outcome is invalid")
        RobotRuntimeUpdate.from_mapping(self.runtime_update)
        object.__setattr__(self, "runtime_update", dict(self.runtime_update))

    def __getitem__(self, key: str) -> object:
        return self.runtime_update[key]

    def __iter__(self):
        return iter(self.runtime_update)

    def __len__(self) -> int:
        return len(self.runtime_update)


def _default_clock_ms() -> int:
    return time.time_ns() // 1_000_000


def _default_id() -> str:
    return secrets.token_hex(12)


def _validated_id(prefix: str, factory: IDFactory) -> str:
    try:
        suffix = factory()
    except Exception:
        raise RobotControlServiceError(
            500,
            "robot_id_factory_failed",
            "Robot control could not create an identifier",
        ) from None
    value = "{}-{}".format(prefix, suffix)
    if (
        not isinstance(suffix, str)
        or not suffix
        or len(value) > 128
        or not value.isascii()
        or not all(
            character.isalnum() or character in "-_"
            for character in value
        )
    ):
        raise RobotControlServiceError(
            500,
            "robot_id_factory_failed",
            "Robot control could not create an identifier",
        )
    return value


def _safe_error_code(error: Exception) -> str:
    """Return a public snapshot-compatible code for an arbitrary failure."""

    try:
        value = getattr(error, "code", "robot_runtime_failed")
    except Exception:
        value = "robot_runtime_failed"
    if (
        isinstance(value, str)
        and value
        and value == value.strip()
        and len(value) <= 128
        and all(33 <= ord(character) <= 126 for character in value)
    ):
        return value
    return "robot_runtime_failed"


def _safe_error_message(error: Exception) -> str:
    """Create bounded, single-line diagnostic text for loopback events."""

    try:
        value = str(error)
    except Exception:
        value = ""
    printable = "".join(
        character if character.isprintable() else " "
        for character in value
    )
    normalized = " ".join(printable.split())
    if not normalized:
        normalized = "Runtime failed without additional details"
    return normalized[:MAX_ERROR_MESSAGE_CHARACTERS]


def _safe_primary_error_diagnostic(error: Exception):
    """Extract one bounded nested failure without walking exception chains."""

    try:
        primary_error = getattr(error, "primary_error", None)
    except Exception:
        return None, None
    if not isinstance(primary_error, Exception) or primary_error is error:
        return None, None
    return (
        _safe_error_code(primary_error),
        _safe_error_message(primary_error),
    )


class RobotControlService:
    """Own one serialized robot episode and a bounded observation history."""

    def __init__(
        self,
        runtime_adapter=None,
        *,
        settings: Optional[RobotControlSettings] = None,
        event_capacity: int = MAX_EVENTS,
        snapshot_capacity: int = MAX_SNAPSHOTS,
        clock_ms: Clock = _default_clock_ms,
        id_factory: IDFactory = _default_id,
    ):
        if (
            isinstance(event_capacity, bool)
            or not isinstance(event_capacity, int)
            or not 1 <= event_capacity <= 10_000
            or isinstance(snapshot_capacity, bool)
            or not isinstance(snapshot_capacity, int)
            or not 1 <= snapshot_capacity <= 10_000
            or not callable(clock_ms)
            or not callable(id_factory)
        ):
            raise ValueError("Robot control service configuration is invalid")
        if runtime_adapter is not None:
            for name in ("run", "request_stop", "emergency_stop"):
                if not callable(getattr(runtime_adapter, name, None)):
                    raise ValueError(
                        "Robot runtime adapter must implement {}".format(name)
                    )
        self._adapter = runtime_adapter
        self._settings = settings or RobotControlSettings()
        if not isinstance(self._settings, RobotControlSettings):
            raise ValueError("Robot control settings are invalid")
        self._clock_ms = clock_ms
        self._id_factory = id_factory
        self._lock = threading.RLock()
        self._state = IDLE if runtime_adapter is not None else DISABLED
        self._accepting = True
        self._episode_id = None
        self._request = None
        self._started_at_unix_ms = None
        self._terminal_reason = None
        self._last_error_code = None
        self._last_primary_error_code = None
        self._last_primary_error_message = None
        self._runtime = RobotRuntimeUpdate(
            speech_status=(
                "idle"
                if self._settings.speech_enabled
                else "disabled"
            )
        )
        self._stop_requested = None
        self._emergency_stop_requested = None
        self._thread = None
        self._control_signals_inflight = 0
        self._event_capacity = event_capacity
        self._snapshot_capacity = snapshot_capacity
        self._events = deque()
        self._snapshots = deque()
        self._event_sequence = 0
        self._snapshot_sequence = 0
        self._event_dropped_total = 0
        self._snapshot_dropped_total = 0
        self._request_history = {}
        self._request_order = deque()
        with self._lock:
            self._append_event_locked(
                "robot.control_initialized",
                "Robot control service initialized",
                {
                    "state": self._state,
                    "enabled": self._adapter is not None,
                },
            )
            self._record_snapshot_locked()

    def _now(self) -> int:
        try:
            return finite_unix_ms(self._clock_ms())
        except DashboardContractError:
            raise RobotControlServiceError(
                500,
                "robot_clock_failed",
                "Robot control clock failed",
            ) from None

    def _append_bounded(
        self,
        target,
        value,
        capacity: int,
        dropped_attribute: str,
    ) -> None:
        if len(target) == capacity:
            target.popleft()
            setattr(
                self,
                dropped_attribute,
                getattr(self, dropped_attribute) + 1,
            )
        target.append(value)

    def _append_event_locked(
        self,
        event_type: str,
        message: str,
        data: Optional[Mapping[str, object]] = None,
        *,
        level: str = "info",
    ) -> None:
        self._event_sequence += 1
        event = {
            "schema": "robot-control-event/v1",
            "sequence": self._event_sequence,
            "event_id": _validated_id("robot-event", self._id_factory),
            "occurred_at_unix_ms": self._now(),
            "level": level,
            "event_type": event_type,
            "message": message,
            "episode_id": self._episode_id,
            "state": self._state,
            "data": dict(data or {}),
        }
        self._append_bounded(
            self._events,
            event,
            self._event_capacity,
            "_event_dropped_total",
        )

    def _snapshot_value_locked(
        self,
        sequence: int,
    ) -> RobotControlSnapshot:
        return RobotControlSnapshot(
            sequence=sequence,
            state=self._state,
            enabled=self._adapter is not None,
            accepting=self._accepting,
            settings=self._settings,
            episode_id=self._episode_id,
            goal=self._request.goal if self._request is not None else None,
            locale=(
                self._request.locale
                if self._request is not None
                else None
            ),
            started_at_unix_ms=self._started_at_unix_ms,
            updated_at_unix_ms=self._now(),
            terminal_reason=self._terminal_reason,
            last_error_code=self._last_error_code,
            runtime=self._runtime,
            primary_error_code=self._last_primary_error_code,
            primary_error_message=self._last_primary_error_message,
        )

    def _record_snapshot_locked(self) -> RobotControlSnapshot:
        self._snapshot_sequence += 1
        snapshot = self._snapshot_value_locked(self._snapshot_sequence)
        self._append_bounded(
            self._snapshots,
            snapshot,
            self._snapshot_capacity,
            "_snapshot_dropped_total",
        )
        return snapshot

    def _current_snapshot_locked(self) -> RobotControlSnapshot:
        if not self._snapshots:
            return self._record_snapshot_locked()
        return self._snapshots[-1]

    def _transition_locked(
        self,
        state: str,
        event_type: str,
        message: str,
        data: Optional[Mapping[str, object]] = None,
        *,
        level: str = "info",
    ) -> RobotControlSnapshot:
        self._state = state
        self._append_event_locked(
            event_type,
            message,
            data,
            level=level,
        )
        return self._record_snapshot_locked()

    def _finish_stopping_if_quiescent_locked(self) -> None:
        if (
            self._state == STOPPING
            and self._thread is None
            and self._control_signals_inflight == 0
        ):
            if self._terminal_reason is None:
                self._terminal_reason = "stopped"
            self._transition_locked(
                IDLE,
                "robot.episode_finished",
                "Robot episode finished",
                {"terminal_reason": self._terminal_reason},
            )

    def _remember_request_locked(
        self,
        request: RobotEpisodeStart,
        episode_id: str,
    ) -> None:
        if request.client_request_id in self._request_history:
            return
        if len(self._request_order) == MAX_REQUEST_HISTORY:
            oldest = self._request_order.popleft()
            self._request_history.pop(oldest, None)
        self._request_order.append(request.client_request_id)
        self._request_history[request.client_request_id] = (
            request,
            episode_id,
        )

    def status(self):
        with self._lock:
            return self._current_snapshot_locked().to_dict()

    def settings(self):
        with self._lock:
            return self._settings.to_dict()

    def update_settings(
        self,
        expected_revision: int,
        changes: Mapping[str, object],
    ):
        with self._lock:
            if self._state != IDLE or self._control_signals_inflight:
                raise RobotControlServiceError(
                    409,
                    "robot_not_idle",
                    "Robot settings can only change while idle",
                )
            if (
                isinstance(expected_revision, bool)
                or not isinstance(expected_revision, int)
            ):
                raise RobotControlServiceError(
                    400,
                    "invalid_robot_integer",
                    "Robot settings revision is invalid",
                )
            if expected_revision != self._settings.revision:
                raise RobotControlServiceError(
                    409,
                    "robot_settings_revision_conflict",
                    "Robot settings revision does not match",
                )
            try:
                self._settings = self._settings.with_updates(changes)
            except DashboardContractError as error:
                raise RobotControlServiceError(
                    400,
                    error.code,
                    str(error),
                ) from None
            if not self._settings.speech_enabled:
                self._runtime = RobotRuntimeUpdate.from_mapping(
                    {"speech_status": "disabled"},
                    self._runtime,
                )
            elif self._runtime.speech_status == "disabled":
                self._runtime = RobotRuntimeUpdate.from_mapping(
                    {"speech_status": "idle"},
                    self._runtime,
                )
            self._append_event_locked(
                "robot.settings_updated",
                "Robot control settings updated",
                {
                    "revision": self._settings.revision,
                    "changed_fields": sorted(changes),
                },
            )
            self._record_snapshot_locked()
            return self._settings.to_dict()

    def start(
        self,
        goal: str,
        locale: str,
        client_request_id: str,
        expected_revision: int,
    ):
        try:
            request = RobotEpisodeStart(
                goal=goal,
                locale=locale,
                client_request_id=client_request_id,
                expected_revision=expected_revision,
            )
        except DashboardContractError as error:
            raise RobotControlServiceError(
                400,
                error.code,
                str(error),
            ) from None
        with self._lock:
            remembered = self._request_history.get(client_request_id)
            if remembered is not None:
                previous_request, previous_episode_id = remembered
                if previous_request != request:
                    raise RobotControlServiceError(
                        409,
                        "robot_idempotency_conflict",
                        "Robot request ID was reused with other content",
                    )
                return {
                    "accepted_episode_id": previous_episode_id,
                    "idempotent": True,
                    "control": self._current_snapshot_locked().to_dict(),
                }
            if not self._accepting:
                raise RobotControlServiceError(
                    503,
                    "robot_service_stopping",
                    "Robot control service is stopping",
                )
            if self._adapter is None or self._state == DISABLED:
                raise RobotControlServiceError(
                    503,
                    "robot_control_disabled",
                    "No physical robot runtime is configured",
                )
            if self._state != IDLE or self._control_signals_inflight:
                raise RobotControlServiceError(
                    409,
                    "robot_episode_active",
                    "A robot episode is already active",
                )
            if expected_revision != self._settings.revision:
                raise RobotControlServiceError(
                    409,
                    "robot_settings_revision_conflict",
                    "Robot settings revision does not match",
                )
            episode_id = _validated_id("episode", self._id_factory)
            self._episode_id = episode_id
            self._request = request
            self._started_at_unix_ms = self._now()
            self._terminal_reason = None
            self._last_error_code = None
            self._last_primary_error_code = None
            self._last_primary_error_message = None
            self._runtime = RobotRuntimeUpdate(
                speech_status=(
                    "idle"
                    if self._settings.speech_enabled
                    else "disabled"
                )
            )
            self._stop_requested = threading.Event()
            self._emergency_stop_requested = threading.Event()
            self._remember_request_locked(request, episode_id)
            snapshot = self._transition_locked(
                STARTING,
                "robot.episode_starting",
                "Robot episode is starting",
                {
                    "client_request_id": client_request_id,
                    "settings_revision": self._settings.revision,
                    "locale": locale,
                },
            )
            context = RobotEpisodeContext(
                episode_id=episode_id,
                request=request,
                settings=self._settings,
                stop_requested=self._stop_requested,
                emergency_stop_requested=self._emergency_stop_requested,
                publish=lambda update: self._publish(
                    episode_id,
                    update,
                ),
            )
            thread = threading.Thread(
                target=self._run_episode,
                args=(context,),
                name="robot-control-{}".format(episode_id),
                daemon=True,
            )
            self._thread = thread
            thread.start()
            return {
                "accepted_episode_id": episode_id,
                "idempotent": False,
                "control": snapshot.to_dict(),
            }

    def _publish(
        self,
        episode_id: str,
        update: Mapping[str, object],
    ) -> None:
        with self._lock:
            if episode_id != self._episode_id:
                return
            if self._state not in ACTIVE_STATES:
                return
            try:
                self._runtime = RobotRuntimeUpdate.from_mapping(
                    update,
                    self._runtime,
                )
            except DashboardContractError as error:
                raise RobotControlServiceError(
                    500,
                    error.code,
                    str(error),
                ) from None
            self._append_event_locked(
                "robot.runtime_update",
                "Robot runtime status updated",
                {
                    "changed_fields": sorted(update),
                    "current_action": self._runtime.current_action,
                    "model_latency_ms": self._runtime.model_latency_ms,
                    "speech_status": self._runtime.speech_status,
                },
                level="debug",
            )
            self._record_snapshot_locked()

    def _run_episode(self, context: RobotEpisodeContext) -> None:
        try:
            with self._lock:
                if context.episode_id != self._episode_id:
                    return
                if self._state == STOPPING:
                    self._terminal_reason = (
                        "emergency_stopped"
                        if context.emergency_stop_requested.is_set()
                        else "stopped"
                    )
                    if self._control_signals_inflight == 0:
                        self._transition_locked(
                            IDLE,
                            "robot.episode_finished",
                            "Robot episode stopped before runtime start",
                            {
                                "terminal_reason": self._terminal_reason,
                            },
                        )
                    return
                if self._state != STARTING:
                    return
                self._transition_locked(
                    RUNNING,
                    "robot.episode_running",
                    "Robot episode is running",
                )
            result = self._adapter.run(context)
            outcome = (
                result
                if isinstance(result, RobotEpisodeOutcome)
                else None
            )
            if result is not None:
                runtime_update = (
                    outcome.runtime_update
                    if outcome is not None
                    else result
                )
                if not isinstance(runtime_update, Mapping):
                    raise RobotControlServiceError(
                        500,
                        "robot_runtime_result_invalid",
                        "Robot runtime returned an invalid result",
                    )
                self._publish(context.episode_id, runtime_update)
            with self._lock:
                if context.episode_id != self._episode_id:
                    return
                # An emergency-stop transport failure is a latched control
                # fault.  The runner may still unwind successfully after its
                # stop event is set, but that does not prove that the failed
                # emergency stop reached the robot and must never clear the
                # FAULTED state.
                if self._state == FAULTED:
                    return
                if context.emergency_stop_requested.is_set():
                    terminal_reason = "emergency_stopped"
                elif context.stop_requested.is_set():
                    terminal_reason = "stopped"
                elif outcome is not None and not outcome.completed:
                    terminal_reason = outcome.terminal_reason
                else:
                    terminal_reason = "completed"
                self._terminal_reason = terminal_reason
                if self._control_signals_inflight == 0:
                    self._transition_locked(
                        IDLE,
                        "robot.episode_finished",
                        "Robot episode finished",
                        {
                            "terminal_reason": terminal_reason,
                            "completed": terminal_reason == "completed",
                        },
                    )
        except Exception as error:
            with self._lock:
                if context.episode_id != self._episode_id:
                    return
                if self._state == FAULTED:
                    return
                code = _safe_error_code(error)
                error_message = _safe_error_message(error)
                (
                    primary_error_code,
                    primary_error_message,
                ) = _safe_primary_error_diagnostic(error)
                self._last_error_code = code
                self._last_primary_error_code = primary_error_code
                self._last_primary_error_message = primary_error_message
                self._terminal_reason = "faulted"
                # A physical runtime emits its generic terminal marker while
                # unwinding.  It is not the failure diagnosis and must not
                # remain presented as the episode's final message.
                self._runtime = RobotRuntimeUpdate.from_mapping(
                    {"message": None},
                    self._runtime,
                )
                fault_data = {
                    "error_code": code,
                    "error_type": type(error).__name__,
                    "error_message": error_message,
                }
                if primary_error_code is not None:
                    fault_data.update({
                        "primary_error_code": primary_error_code,
                        "primary_error_message": primary_error_message,
                    })
                self._transition_locked(
                    FAULTED,
                    "robot.episode_faulted",
                    "Robot episode faulted",
                    fault_data,
                    level="error",
                )
        finally:
            with self._lock:
                if context.episode_id == self._episode_id:
                    self._thread = None
                    self._stop_requested = None
                    self._emergency_stop_requested = None
                    self._finish_stopping_if_quiescent_locked()

    def stop(self):
        adapter = None
        with self._lock:
            if self._state in (DISABLED, IDLE):
                return self._current_snapshot_locked().to_dict()
            if self._state == FAULTED:
                if (
                    self._thread is None
                    and self._control_signals_inflight == 0
                ):
                    self._last_error_code = None
                    self._last_primary_error_code = None
                    self._last_primary_error_message = None
                    self._terminal_reason = "fault_acknowledged"
                    return self._transition_locked(
                        IDLE,
                        "robot.fault_acknowledged",
                        "Robot fault acknowledged",
                    ).to_dict()
                # Keep the fault latched while the episode runner is still
                # unwinding.  A regular stop remains useful as a best-effort
                # cancellation signal, but it is not evidence that the
                # emergency-stop failure has been resolved.
                if self._stop_requested is not None:
                    self._stop_requested.set()
                self._control_signals_inflight += 1
                self._append_event_locked(
                    "robot.stop_requested_while_faulted",
                    "Robot stop requested while fault remains latched",
                    level="warning",
                )
                self._record_snapshot_locked()
                adapter = self._adapter
                snapshot = self._current_snapshot_locked().to_dict()
            elif self._state == STOPPING:
                return self._current_snapshot_locked().to_dict()
            else:
                if self._stop_requested is not None:
                    self._stop_requested.set()
                self._control_signals_inflight += 1
                self._transition_locked(
                    STOPPING,
                    "robot.stop_requested",
                    "Robot episode stop requested",
                )
                adapter = self._adapter
                snapshot = self._current_snapshot_locked().to_dict()
        try:
            if adapter is not None:
                adapter.request_stop()
        except Exception as error:
            with self._lock:
                self._append_event_locked(
                    "robot.stop_signal_failed",
                    "Robot runtime stop signal failed",
                    {"error_type": type(error).__name__},
                    level="warning",
                )
                self._record_snapshot_locked()
        finally:
            with self._lock:
                self._control_signals_inflight -= 1
                self._finish_stopping_if_quiescent_locked()
        return snapshot

    def emergency_stop(self):
        adapter = None
        with self._lock:
            if self._adapter is None or self._state == DISABLED:
                raise RobotControlServiceError(
                    503,
                    "robot_control_disabled",
                    "No physical robot runtime is configured",
                )
            if self._emergency_stop_requested is not None:
                self._emergency_stop_requested.set()
            if self._stop_requested is not None:
                self._stop_requested.set()
            self._control_signals_inflight += 1
            if self._state in (STARTING, RUNNING):
                self._transition_locked(
                    STOPPING,
                    "robot.emergency_stop_requested",
                    "Robot emergency stop requested",
                    level="warning",
                )
            else:
                self._append_event_locked(
                    "robot.emergency_stop_requested",
                    "Robot emergency stop requested",
                    level="warning",
                )
                self._record_snapshot_locked()
            adapter = self._adapter
        try:
            adapter.emergency_stop()
        except Exception as error:
            with self._lock:
                self._last_error_code = "robot_emergency_stop_failed"
                self._last_primary_error_code = None
                self._last_primary_error_message = None
                self._terminal_reason = "faulted"
                self._transition_locked(
                    FAULTED,
                    "robot.emergency_stop_failed",
                    "Robot emergency stop failed",
                    {"error_type": type(error).__name__},
                    level="error",
                )
                self._control_signals_inflight -= 1
            raise RobotControlServiceError(
                503,
                "robot_emergency_stop_failed",
                "Robot emergency stop failed",
            ) from None
        with self._lock:
            self._control_signals_inflight -= 1
            self._append_event_locked(
                "robot.emergency_stop_sent",
                "Robot emergency stop signal sent",
                level="warning",
            )
            self._record_snapshot_locked()
            self._finish_stopping_if_quiescent_locked()
            snapshot = self._current_snapshot_locked()
            return snapshot.to_dict()

    @staticmethod
    def _page(
        values,
        after_sequence: int,
        limit: int,
        dropped_total: int,
        schema: str,
        serializer,
    ):
        if (
            isinstance(after_sequence, bool)
            or not isinstance(after_sequence, int)
            or after_sequence < 0
            or isinstance(limit, bool)
            or not isinstance(limit, int)
            or not 1 <= limit <= 500
        ):
            raise RobotControlServiceError(
                400,
                "invalid_robot_cursor",
                "Robot event cursor or limit is invalid",
            )
        oldest = values[0]["sequence"] if values and isinstance(
            values[0],
            dict,
        ) else values[0].sequence if values else None
        newest = values[-1]["sequence"] if values and isinstance(
            values[-1],
            dict,
        ) else values[-1].sequence if values else 0
        selected = []
        for value in values:
            sequence = (
                value["sequence"]
                if isinstance(value, dict)
                else value.sequence
            )
            if sequence > after_sequence:
                selected.append(serializer(value))
            if len(selected) == limit:
                break
        next_after = (
            selected[-1]["sequence"]
            if selected
            else after_sequence
        )
        return {
            "schema": schema,
            "after_sequence": after_sequence,
            "oldest_sequence": oldest,
            "newest_sequence": newest,
            "next_after_sequence": next_after,
            "gap": (
                oldest is not None
                and after_sequence < oldest - 1
            ),
            "dropped_total": dropped_total,
            "items": selected,
        }

    def events(self, after_sequence: int, limit: int):
        with self._lock:
            page = self._page(
                tuple(self._events),
                after_sequence,
                limit,
                self._event_dropped_total,
                EVENT_PAGE_SCHEMA,
                lambda value: dict(value),
            )
        page["events"] = page.pop("items")
        return page

    def snapshots(self, after_sequence: int, limit: int):
        with self._lock:
            page = self._page(
                tuple(self._snapshots),
                after_sequence,
                limit,
                self._snapshot_dropped_total,
                SNAPSHOT_PAGE_SCHEMA,
                lambda value: value.to_dict(),
            )
        page["snapshots"] = page.pop("items")
        return page

    def shutdown(self, timeout_seconds: float = 2.0) -> None:
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not 0 <= float(timeout_seconds) <= 30
        ):
            raise ValueError("Robot control shutdown timeout is invalid")
        with self._lock:
            self._accepting = False
            thread = self._thread
            active = self._state in ACTIVE_STATES
        if active:
            try:
                self.emergency_stop()
            except RobotControlServiceError:
                pass
        if thread is not None and thread is not threading.current_thread():
            thread.join(float(timeout_seconds))
        with self._lock:
            if thread is not None and thread.is_alive():
                # Thread liveness is host-process lifecycle evidence, not a
                # physical stop result.  Keep the episode pending while the
                # runner completes its bounded cleanup; _run_episode will
                # transition to FAULTED if that cleanup explicitly reports
                # missing physical proof.
                self._append_event_locked(
                    "robot.control_shutdown_pending",
                    "Robot runtime is still stopping",
                    {"join_timeout_seconds": float(timeout_seconds)},
                    level="warning",
                )
                self._record_snapshot_locked()
            else:
                self._append_event_locked(
                    "robot.control_shutdown",
                    "Robot control service stopped",
                )
                self._record_snapshot_locked()
