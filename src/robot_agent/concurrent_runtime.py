"""Concurrent, simulator-only navigation and object-expression runtime.

The runtime deliberately separates four kinds of work:

* deterministic navigation behaviors publish proposals asynchronously;
* an optional expression planner interprets versioned obstruction snapshots;
* speech playback runs independently from wheel navigation; and
* the propeller arm uses an explicit stop/acknowledge/exclusive/release gate.

Only :class:`NavigationEpisode` and its :class:`MotionSupervisor` own wheel
motion.  Model output remains semantic and cannot select motor speed, motor
duration, port, priority, authority or TTL.

Callback cancellation is cooperative.  The host watchdog can stop the
navigation episode and keep the wheels stopped, but a future physical arm
backend must also enforce its own local motor timeout.
"""

from dataclasses import dataclass
from collections import deque
import math
import queue
import threading
import time
from typing import (
    Callable,
    Deque,
    Dict,
    Mapping,
    Optional,
    Tuple,
    Union,
)

from .interaction_contract import (
    ExpressionProposal,
    InteractionContractError,
    InteractionSnapshot,
    ObjectEvidence,
    decode_expression_proposal,
    expression_proposal_id_for_snapshot,
)
from .navigation_contract import (
    NavigationContractError,
    WaypointGoal,
)
from .navigation_episode import (
    GoalSeekingBehavior,
    NAVIGATION_REFRESH_REQUIRED,
    NavigationEpisode,
    NavigationLimits,
    NavigationResult,
    ObstacleAvoidanceBehavior,
)
from .navigation_simulator import DifferentialDriveSimulator
from .navigation_state import NavigationSnapshot, ProposalInbox
from .navigation_supervisor import MotionSupervisor


ExpressionPlanner = Callable[
    [InteractionSnapshot],
    Union[ExpressionProposal, bytes],
]
Speaker = Callable[[str, str, object], None]
ArmSegmentExecutor = Callable[[int, int, object], None]
TickHook = Callable[[object, float], object]


def _non_bool_int(
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
        raise ValueError("{} is invalid".format(name))
    return value


def _non_negative_seconds(name: str, value: float) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0
        or value > 60
    ):
        raise ValueError("{} is invalid".format(name))
    return float(value)


@dataclass(frozen=True)
class ConcurrentRuntimePolicy:
    """Host-owned limits for one concurrent navigation episode."""

    tick_interval_s: float = 0.01
    behavior_queue_capacity: int = 1
    planner_queue_capacity: int = 2
    speech_queue_capacity: int = 2
    arm_queue_capacity: int = 2
    obstruction_trigger_mm: int = 170
    expression_ttl_ms: int = 5_000
    expression_cooldown_ms: int = 5_000
    max_planner_requests: int = 8
    max_evidence_age_ms: int = 250
    max_speech_utterances: int = 4
    max_speech_time_ms: int = 20_000
    max_gestures: int = 2
    max_arm_time_ms: int = 4_000
    arm_exclusive_timeout_ms: int = 5_000
    gesture_cooldown_ms: int = 1_000
    arm_speed_dps: int = 900
    arm_segment_duration_ms: int = 180
    event_log_capacity: int = 2_048
    shutdown_join_timeout_s: float = 0.2

    def __post_init__(self) -> None:
        _non_negative_seconds("tick_interval_s", self.tick_interval_s)
        _non_negative_seconds(
            "shutdown_join_timeout_s",
            self.shutdown_join_timeout_s,
        )
        for name, value, minimum, maximum in (
            (
                "behavior_queue_capacity",
                self.behavior_queue_capacity,
                1,
                128,
            ),
            (
                "planner_queue_capacity",
                self.planner_queue_capacity,
                1,
                128,
            ),
            (
                "speech_queue_capacity",
                self.speech_queue_capacity,
                1,
                128,
            ),
            ("arm_queue_capacity", self.arm_queue_capacity, 1, 128),
            (
                "obstruction_trigger_mm",
                self.obstruction_trigger_mm,
                1,
                100_000,
            ),
            (
                "expression_ttl_ms",
                self.expression_ttl_ms,
                1,
                60_000,
            ),
            (
                "expression_cooldown_ms",
                self.expression_cooldown_ms,
                0,
                3_600_000,
            ),
            (
                "max_planner_requests",
                self.max_planner_requests,
                0,
                10_000,
            ),
            (
                "max_evidence_age_ms",
                self.max_evidence_age_ms,
                1,
                60_000,
            ),
            (
                "max_speech_utterances",
                self.max_speech_utterances,
                0,
                10_000,
            ),
            (
                "max_speech_time_ms",
                self.max_speech_time_ms,
                0,
                3_600_000,
            ),
            ("max_gestures", self.max_gestures, 0, 10_000),
            (
                "max_arm_time_ms",
                self.max_arm_time_ms,
                0,
                3_600_000,
            ),
            (
                "arm_exclusive_timeout_ms",
                self.arm_exclusive_timeout_ms,
                1,
                3_600_000,
            ),
            (
                "gesture_cooldown_ms",
                self.gesture_cooldown_ms,
                0,
                3_600_000,
            ),
            ("arm_speed_dps", self.arm_speed_dps, 1, 20_000),
            (
                "arm_segment_duration_ms",
                self.arm_segment_duration_ms,
                1,
                10_000,
            ),
            (
                "event_log_capacity",
                self.event_log_capacity,
                1,
                100_000,
            ),
        ):
            _non_bool_int(name, value, minimum, maximum)


@dataclass(frozen=True)
class RuntimeEvent:
    sequence: int
    at_ms: int
    worker: str
    kind: str
    detail: str = ""

    def to_dict(self) -> Mapping[str, object]:
        return {
            "sequence": self.sequence,
            "at_ms": self.at_ms,
            "worker": self.worker,
            "kind": self.kind,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class ConcurrentRuntimeMetrics:
    navigation_snapshots: int
    navigation_snapshot_drops: int
    navigation_proposals: int
    navigation_proposal_failures: int
    planner_requests: int
    planner_budget_drops: int
    planner_cooldown_drops: int
    planner_request_drops: int
    planner_failures: int
    expression_holds: int
    expressions_accepted: int
    stale_expression_drops: int
    duplicate_expression_drops: int
    speech_queue_drops: int
    stale_speech_drops: int
    speech_started: int
    speech_completed: int
    speech_cancellations: int
    speech_failures: int
    speech_elapsed_ms: int
    arm_queue_drops: int
    gestures_started: int
    gestures_completed: int
    arm_cancellations: int
    arm_exclusive_timeouts: int
    gesture_drops: int
    arm_segments: int
    arm_elapsed_ms: int
    navigation_pause_requests: int
    navigation_pause_acks: int
    events_dropped: int

    def to_dict(self) -> Mapping[str, int]:
        return dict(self.__dict__)


@dataclass(frozen=True)
class ConcurrentRuntimeResult:
    """Combined navigation outcome and isolated worker observability."""

    navigation: NavigationResult
    metrics: ConcurrentRuntimeMetrics
    events: Tuple[RuntimeEvent, ...]
    clean_shutdown: bool
    workers_alive: Tuple[str, ...]

    def to_dict(self) -> Mapping[str, object]:
        return {
            "navigation": self.navigation.to_dict(),
            "metrics": self.metrics.to_dict(),
            "events": [event.to_dict() for event in self.events],
            "clean_shutdown": self.clean_shutdown,
            "workers_alive": list(self.workers_alive),
        }


class _EventLog:
    def __init__(self, capacity: int, clock_ms: Callable[[], int]):
        self._events: Deque[RuntimeEvent] = deque(maxlen=capacity)
        self._clock_ms = clock_ms
        self._sequence = 0
        self._dropped = 0
        self._lock = threading.Lock()

    @property
    def dropped(self) -> int:
        with self._lock:
            return self._dropped

    def append(
        self,
        worker: str,
        kind: str,
        detail: str = "",
    ) -> None:
        with self._lock:
            self._sequence += 1
            if len(self._events) == self._events.maxlen:
                self._dropped += 1
            try:
                at_ms = int(self._clock_ms())
            except Exception:
                at_ms = 0
            self._events.append(RuntimeEvent(
                sequence=self._sequence,
                at_ms=max(0, at_ms),
                worker=worker,
                kind=kind,
                detail=detail,
            ))

    def snapshot(self) -> Tuple[RuntimeEvent, ...]:
        with self._lock:
            return tuple(self._events)


class _Metrics:
    _NAMES = (
        "navigation_snapshots",
        "navigation_snapshot_drops",
        "navigation_proposals",
        "navigation_proposal_failures",
        "planner_requests",
        "planner_budget_drops",
        "planner_cooldown_drops",
        "planner_request_drops",
        "planner_failures",
        "expression_holds",
        "expressions_accepted",
        "stale_expression_drops",
        "duplicate_expression_drops",
        "speech_queue_drops",
        "stale_speech_drops",
        "speech_started",
        "speech_completed",
        "speech_cancellations",
        "speech_failures",
        "speech_elapsed_ms",
        "arm_queue_drops",
        "gestures_started",
        "gestures_completed",
        "arm_cancellations",
        "arm_exclusive_timeouts",
        "gesture_drops",
        "arm_segments",
        "arm_elapsed_ms",
        "navigation_pause_requests",
        "navigation_pause_acks",
    )

    def __init__(self):
        self._values: Dict[str, int] = {
            name: 0 for name in self._NAMES
        }
        self._lock = threading.Lock()

    def add(self, name: str, amount: int = 1) -> int:
        with self._lock:
            self._values[name] += amount
            return self._values[name]

    def get(self, name: str) -> int:
        with self._lock:
            return self._values[name]

    def snapshot(self, events_dropped: int) -> ConcurrentRuntimeMetrics:
        with self._lock:
            values = dict(self._values)
        values["events_dropped"] = events_dropped
        return ConcurrentRuntimeMetrics(**values)


@dataclass(frozen=True)
class _PlannerRequest:
    snapshot: InteractionSnapshot
    proposal_id: str
    submitted_at_ms: int
    valid_until_ms: int


@dataclass(frozen=True)
class _ExpressionJob:
    proposal: ExpressionProposal
    basis_snapshot: InteractionSnapshot
    accepted_at_ms: int
    valid_until_ms: int


_STOP = object()


class _InteractionReducer:
    """Reduce fast navigation samples to stable interaction epochs."""

    def __init__(
        self,
        obstruction_trigger_mm: int,
        response_locale: str,
    ):
        self._trigger_mm = obstruction_trigger_mm
        self._response_locale = response_locale
        self._current: Optional[InteractionSnapshot] = None
        self._blocked = False
        self._object_id: Optional[str] = None
        self._world_model_version: Optional[int] = None
        self._interaction_state_version = 0
        self._obstruction_epoch = 0
        self._lock = threading.Lock()

    def current(self) -> Optional[InteractionSnapshot]:
        with self._lock:
            return self._current

    def reduce(
        self,
        snapshot: NavigationSnapshot,
    ) -> Tuple[InteractionSnapshot, bool]:
        clearance = snapshot.clearance
        blocked = bool(
            clearance.near_obstacle_latched
            or (
                clearance.forward_mm is not None
                and clearance.forward_mm <= self._trigger_mm
            )
        )
        object_id = clearance.forward_object_id if blocked else None
        with self._lock:
            first = self._current is None
            changed = bool(
                first
                or blocked != self._blocked
                or object_id != self._object_id
                or snapshot.world_model_version
                != self._world_model_version
            )
            if changed:
                self._interaction_state_version += 1
                if blocked and (
                    first
                    or not self._blocked
                    or object_id != self._object_id
                    or snapshot.world_model_version
                    != self._world_model_version
                ):
                    self._obstruction_epoch += 1
            evidence = None
            if blocked:
                evidence = ObjectEvidence(
                    evidence_id="obstruction-{}-world-{}".format(
                        self._obstruction_epoch,
                        snapshot.world_model_version,
                    ),
                    relation="BLOCKING_PATH",
                    object_id=object_id,
                    source=clearance.source,
                    observed_at_ms=clearance.observed_at_ms,
                    confidence_milli=1_000,
                )
            if blocked:
                drive_phase = "BLOCKED"
            elif snapshot.motors_running:
                drive_phase = "MOVING"
            else:
                drive_phase = "STOPPED"
            value = InteractionSnapshot(
                robot_id=snapshot.robot_id,
                controller_instance_id=snapshot.controller_instance_id,
                goal_id=snapshot.goal_id,
                goal_epoch=snapshot.goal_epoch,
                plan_revision=snapshot.plan_revision,
                response_locale=self._response_locale,
                interaction_state_version=(
                    self._interaction_state_version
                ),
                world_model_version=snapshot.world_model_version,
                captured_at_ms=snapshot.captured_at_host_ms,
                obstruction_epoch=self._obstruction_epoch,
                drive_phase=drive_phase,
                evidence=evidence,
            )
            self._current = value
            self._blocked = blocked
            self._object_id = object_id
            self._world_model_version = snapshot.world_model_version
            return value, changed


class _CallbackCancelEvent:
    """A small Event-compatible cancellation view for callback isolation."""

    def __init__(self, runtime_cancel: threading.Event):
        self._runtime_cancel = runtime_cancel
        self._local_cancel = threading.Event()

    def is_set(self) -> bool:
        return (
            self._runtime_cancel.is_set()
            or self._local_cancel.is_set()
        )

    def set(self) -> None:
        self._local_cancel.set()

    def wait(self, timeout: Optional[float] = None) -> bool:
        if self.is_set():
            return True
        if timeout is None:
            while not self.is_set():
                self._local_cancel.wait(0.05)
            return True
        deadline = time.monotonic() + max(0.0, timeout)
        while not self.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._local_cancel.wait(min(0.05, remaining))
        return True


class ConcurrentBehaviorRuntime:
    """Run bounded navigation, expression, speech and gesture workers.

    The caller supplies an already configured simulator, motion supervisor
    and proposal inbox.  The inbox must allowlist ``goal-seeking`` and
    ``obstacle-avoidance`` when those behaviors are enabled.
    """

    def __init__(
        self,
        plant: DifferentialDriveSimulator,
        supervisor: MotionSupervisor,
        inbox: ProposalInbox,
        goal: WaypointGoal,
        response_locale: str,
        expression_planner: Optional[ExpressionPlanner] = None,
        speaker: Optional[Speaker] = None,
        arm_segment_executor: Optional[ArmSegmentExecutor] = None,
        behaviors: Optional[Tuple[object, ...]] = None,
        navigation_limits: NavigationLimits = NavigationLimits(),
        policy: ConcurrentRuntimePolicy = ConcurrentRuntimePolicy(),
        host_clock_ms: Callable[[], int] = (
            lambda: int(time.monotonic() * 1_000)
        ),
        tick_hook: Optional[TickHook] = None,
    ):
        if not isinstance(plant, DifferentialDriveSimulator):
            raise NavigationContractError(
                "invalid_navigation_plant",
                "Concurrent runtime is simulator-only",
            )
        if not isinstance(supervisor, MotionSupervisor):
            raise NavigationContractError(
                "invalid_motion_supervisor",
                "Concurrent runtime requires MotionSupervisor",
            )
        if not isinstance(inbox, ProposalInbox):
            raise NavigationContractError(
                "invalid_proposal_inbox",
                "Concurrent runtime requires ProposalInbox",
            )
        if not isinstance(goal, WaypointGoal):
            raise NavigationContractError(
                "invalid_goal",
                "Concurrent runtime requires WaypointGoal",
            )
        if (
            not isinstance(response_locale, str)
            or not response_locale
            or response_locale != response_locale.strip()
            or len(response_locale) > 64
            or any(ord(character) < 32 for character in response_locale)
        ):
            raise InteractionContractError(
                "invalid_response_locale",
                "response_locale is invalid",
            )
        if not isinstance(navigation_limits, NavigationLimits):
            raise NavigationContractError(
                "invalid_navigation_limits",
                "Concurrent runtime limits are invalid",
            )
        if not isinstance(policy, ConcurrentRuntimePolicy):
            raise ValueError("policy is invalid")
        if not callable(host_clock_ms):
            raise ValueError("host_clock_ms must be callable")
        for name, callback in (
            ("expression_planner", expression_planner),
            ("speaker", speaker),
            ("arm_segment_executor", arm_segment_executor),
            ("tick_hook", tick_hook),
        ):
            if callback is not None and not callable(callback):
                raise ValueError("{} must be callable".format(name))
        selected = (
            (
                GoalSeekingBehavior(),
                ObstacleAvoidanceBehavior(
                    trigger_mm=policy.obstruction_trigger_mm
                ),
            )
            if behaviors is None
            else behaviors
        )
        if (
            not isinstance(selected, tuple)
            or not selected
            or any(
                type(behavior)
                not in (
                    GoalSeekingBehavior,
                    ObstacleAvoidanceBehavior,
                )
                for behavior in selected
            )
        ):
            raise NavigationContractError(
                "invalid_navigation_behaviors",
                "Concurrent runtime accepts built-in behaviors only",
            )
        source_ids = [behavior.source_id for behavior in selected]
        if len(source_ids) != len(set(source_ids)):
            raise NavigationContractError(
                "duplicate_navigation_behavior",
                "Each behavior source may have one worker",
            )

        self.plant = plant
        self.supervisor = supervisor
        self.inbox = inbox
        self.goal = goal
        self.response_locale = response_locale
        self.expression_planner = expression_planner
        self.speaker = speaker
        self.arm_segment_executor = arm_segment_executor
        self.behaviors = selected
        self.navigation_limits = navigation_limits
        self.policy = policy
        self._host_clock_ms = host_clock_ms
        self._tick_hook = (
            self._default_tick_hook if tick_hook is None else tick_hook
        )

        self._cancel_event = threading.Event()
        self._run_lock = threading.Lock()
        self._has_run = False
        self._events = _EventLog(
            policy.event_log_capacity,
            host_clock_ms,
        )
        self._metrics = _Metrics()
        self._interaction = _InteractionReducer(
            policy.obstruction_trigger_mm,
            response_locale,
        )
        self._submitted_obstruction_epochs = set()
        # One planner worker owns this bounded per-run replay registry.
        # Its maximum size is capped by max_planner_requests.
        self._expression_proposal_ids = set()
        self._planner_request_count = 0
        self._last_expression_request_ms = {}
        self._submission_lock = threading.Lock()

        self._behavior_queues = {
            behavior.source_id: queue.Queue(
                maxsize=policy.behavior_queue_capacity
            )
            for behavior in selected
        }
        self._planner_queue = queue.Queue(
            maxsize=policy.planner_queue_capacity
        )
        self._speech_queue = queue.Queue(
            maxsize=policy.speech_queue_capacity
        )
        self._arm_queue = queue.Queue(
            maxsize=policy.arm_queue_capacity
        )
        self._pause_requested = threading.Event()
        self._pause_acknowledged = threading.Event()
        self._pause_released = threading.Event()
        self._active_callback_events = set()
        self._callback_lock = threading.Lock()
        self._threads = []

    @staticmethod
    def _default_tick_hook(
        cancel_event: object,
        tick_interval_s: float,
    ) -> object:
        return cancel_event.wait(tick_interval_s)

    def cancel(self) -> None:
        """Cooperatively cancel all workers and wake an active arm pause."""

        self._events.append("runtime", "cancel_requested")
        self._cancel_event.set()
        self._pause_acknowledged.set()
        self._pause_released.set()
        with self._callback_lock:
            callbacks = tuple(self._active_callback_events)
        for callback_event in callbacks:
            callback_event.set()
        self._close_queues()

    def _now_ms(self) -> int:
        value = self._host_clock_ms()
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 0
        ):
            raise ValueError("host clock returned an invalid value")
        return value

    def _record_drop(
        self,
        worker: str,
        queue_name: str,
        metric_name: str,
        detail: str,
    ) -> None:
        self._metrics.add(metric_name)
        self._events.append(
            worker,
            "queue_drop",
            "{}:{}".format(queue_name, detail),
        )

    def _put_latest(
        self,
        target: queue.Queue,
        value: object,
        worker: str,
    ) -> None:
        try:
            target.put_nowait(value)
            return
        except queue.Full:
            pass
        try:
            target.get_nowait()
        except queue.Empty:
            pass
        self._record_drop(
            worker,
            "navigation_snapshot",
            "navigation_snapshot_drops",
            "replaced_by_newer_snapshot",
        )
        try:
            target.put_nowait(value)
        except queue.Full:
            self._record_drop(
                worker,
                "navigation_snapshot",
                "navigation_snapshot_drops",
                "concurrent_publish_lost",
            )

    def _put_bounded(
        self,
        target: queue.Queue,
        value: object,
        worker: str,
        queue_name: str,
        drop_metric: str,
    ) -> bool:
        try:
            target.put_nowait(value)
            return True
        except queue.Full:
            self._record_drop(
                worker,
                queue_name,
                drop_metric,
                "capacity_exhausted",
            )
            return False

    def _observation_sink(self, snapshot: NavigationSnapshot) -> None:
        self._metrics.add("navigation_snapshots")
        for source_id, target in self._behavior_queues.items():
            self._put_latest(target, snapshot, source_id)

        interaction, changed = self._interaction.reduce(snapshot)
        self._events.append(
            "motion",
            "navigation_observation",
            "state={} interaction={} obstruction={}".format(
                snapshot.state_version,
                interaction.interaction_state_version,
                interaction.obstruction_epoch,
            ),
        )
        if (
            self.expression_planner is None
            or interaction.evidence is None
            or not changed
        ):
            return
        epoch = interaction.obstruction_epoch
        with self._submission_lock:
            if epoch in self._submitted_obstruction_epochs:
                return
            self._submitted_obstruction_epochs.add(epoch)
            now_ms = self._now_ms()
            evidence = interaction.evidence
            if evidence.object_id is None:
                interaction_key = (
                    "unknown-obstruction",
                    evidence.source,
                    evidence.relation,
                )
            else:
                interaction_key = (
                    "identified-object",
                    evidence.source,
                    evidence.relation,
                    evidence.object_id,
                )
            if (
                self._planner_request_count
                >= self.policy.max_planner_requests
            ):
                self._metrics.add("planner_budget_drops")
                self._events.append(
                    "interaction-reducer",
                    "planner_budget_drop",
                    "obstruction={}".format(epoch),
                )
                return
            previous_request_ms = (
                self._last_expression_request_ms.get(interaction_key)
            )
            if (
                previous_request_ms is not None
                and now_ms - previous_request_ms
                < self.policy.expression_cooldown_ms
            ):
                self._metrics.add("planner_cooldown_drops")
                self._events.append(
                    "interaction-reducer",
                    "planner_cooldown_drop",
                    "obstruction={} key={}".format(
                        epoch,
                        "|".join(interaction_key),
                    ),
                )
                return
            request = _PlannerRequest(
                snapshot=interaction,
                proposal_id=expression_proposal_id_for_snapshot(
                    interaction
                ),
                submitted_at_ms=now_ms,
                valid_until_ms=(
                    now_ms + self.policy.expression_ttl_ms
                ),
            )
            queued = self._put_bounded(
                self._planner_queue,
                request,
                "interaction-reducer",
                "expression_planner",
                "planner_request_drops",
            )
            if queued:
                self._planner_request_count += 1
                self._last_expression_request_ms[
                    interaction_key
                ] = now_ms
                self._metrics.add("planner_requests")

    def _behavior_worker(self, behavior: object) -> None:
        source_id = behavior.source_id
        target = self._behavior_queues[source_id]
        self._events.append(source_id, "worker_started")
        while True:
            item = target.get()
            if item is _STOP:
                break
            if self._cancel_event.is_set():
                break
            try:
                if type(behavior) is GoalSeekingBehavior:
                    proposal = GoalSeekingBehavior.propose(
                        behavior,
                        self.goal,
                        item,
                    )
                else:
                    proposal = ObstacleAvoidanceBehavior.propose(
                        behavior,
                        self.goal,
                        item,
                    )
                if proposal is not None:
                    self.inbox.publish_host(proposal, source_id)
                    self._metrics.add("navigation_proposals")
                    self._events.append(
                        source_id,
                        "proposal_published",
                        proposal.proposal_id,
                    )
            except Exception as error:
                self._metrics.add("navigation_proposal_failures")
                self._events.append(
                    source_id,
                    "worker_failure",
                    type(error).__name__,
                )
        self._events.append(source_id, "worker_stopped")

    def _event_is_current(
        self,
        proposal: ExpressionProposal,
        basis_snapshot: InteractionSnapshot,
        valid_until_ms: int,
    ) -> bool:
        if self._now_ms() > valid_until_ms:
            return False
        try:
            proposal.assert_matches_snapshot(basis_snapshot)
        except InteractionContractError:
            return False
        if (
            proposal.decision == "EXPRESS"
            and proposal.intent.utterance_locale
            != basis_snapshot.response_locale
        ):
            return False
        current = self._interaction.current()
        if current is None:
            return False
        return (
            current.robot_id == basis_snapshot.robot_id
            and current.controller_instance_id
            == basis_snapshot.controller_instance_id
            and current.goal_id == basis_snapshot.goal_id
            and current.goal_epoch == basis_snapshot.goal_epoch
            and current.plan_revision == basis_snapshot.plan_revision
            and current.world_model_version
            == basis_snapshot.world_model_version
            and current.obstruction_epoch
            == basis_snapshot.obstruction_epoch
            and current.response_locale
            == basis_snapshot.response_locale
        )

    def _planner_worker(self) -> None:
        self._events.append("expression-planner", "worker_started")
        while True:
            item = self._planner_queue.get()
            if item is _STOP:
                break
            if self._cancel_event.is_set():
                break
            try:
                value = self.expression_planner(item.snapshot)
                proposal = (
                    decode_expression_proposal(value)
                    if isinstance(value, bytes)
                    else value
                )
                if not isinstance(proposal, ExpressionProposal):
                    raise InteractionContractError(
                        "invalid_expression_result",
                        "Planner must return ExpressionProposal or bytes",
                    )
            except Exception as error:
                self._metrics.add("planner_failures")
                self._events.append(
                    "expression-planner",
                    "planner_failure",
                    type(error).__name__,
                )
                continue
            if self._cancel_event.is_set():
                self._events.append(
                    "expression-planner",
                    "planner_result_cancelled",
                    proposal.proposal_id,
                )
                break
            if proposal.proposal_id != item.proposal_id:
                self._metrics.add("planner_failures")
                self._events.append(
                    "expression-planner",
                    "proposal_id_mismatch_drop",
                    proposal.proposal_id,
                )
                continue
            if proposal.proposal_id in self._expression_proposal_ids:
                self._metrics.add("duplicate_expression_drops")
                self._events.append(
                    "expression-planner",
                    "duplicate_expression_drop",
                    proposal.proposal_id,
                )
                continue
            self._expression_proposal_ids.add(proposal.proposal_id)
            if not self._event_is_current(
                proposal,
                item.snapshot,
                item.valid_until_ms,
            ):
                self._metrics.add("stale_expression_drops")
                self._events.append(
                    "expression-planner",
                    "stale_expression_drop",
                    proposal.proposal_id,
                )
                continue
            if proposal.decision != "EXPRESS":
                self._metrics.add("expression_holds")
                self._events.append(
                    "expression-planner",
                    "expression_not_selected",
                    proposal.decision,
                )
                continue
            self._metrics.add("expressions_accepted")
            job = _ExpressionJob(
                proposal=proposal,
                basis_snapshot=item.snapshot,
                accepted_at_ms=self._now_ms(),
                valid_until_ms=item.valid_until_ms,
            )
            self._events.append(
                "expression-planner",
                "expression_accepted",
                proposal.proposal_id,
            )
            if self.speaker is not None:
                self._put_bounded(
                    self._speech_queue,
                    job,
                    "expression-planner",
                    "speech",
                    "speech_queue_drops",
                )
            if (
                self.arm_segment_executor is not None
                and job.proposal.intent.gesture_kind
                == "PROPELLER_WAVE"
            ):
                self._put_bounded(
                    self._arm_queue,
                    job,
                    "expression-planner",
                    "arm",
                    "arm_queue_drops",
                )
        self._events.append("expression-planner", "worker_stopped")

    def _register_callback(
        self,
        value: _CallbackCancelEvent,
    ) -> None:
        with self._callback_lock:
            self._active_callback_events.add(value)

    def _unregister_callback(
        self,
        value: _CallbackCancelEvent,
    ) -> None:
        with self._callback_lock:
            self._active_callback_events.discard(value)

    def _speech_worker(self) -> None:
        self._events.append("speech", "worker_started")
        used_ms = 0
        while True:
            item = self._speech_queue.get()
            if item is _STOP:
                break
            if self._cancel_event.is_set():
                break
            if (
                self._metrics.get("speech_started")
                >= self.policy.max_speech_utterances
                or used_ms >= self.policy.max_speech_time_ms
            ):
                self._events.append(
                    "speech",
                    "speech_budget_drop",
                    item.proposal.proposal_id,
                )
                continue
            if not self._event_is_current(
                item.proposal,
                item.basis_snapshot,
                item.valid_until_ms,
            ):
                self._metrics.add("stale_speech_drops")
                self._events.append(
                    "speech",
                    "stale_speech_drop",
                    item.proposal.proposal_id,
                )
                continue
            intent = item.proposal.intent
            callback_cancel = _CallbackCancelEvent(
                self._cancel_event
            )
            self._register_callback(callback_cancel)
            started_at = time.monotonic()
            self._metrics.add("speech_started")
            self._events.append(
                "speech",
                "speech_started",
                item.proposal.proposal_id,
            )
            remaining_ms = max(
                0,
                self.policy.max_speech_time_ms - used_ms,
            )
            timer = threading.Timer(
                remaining_ms / 1_000.0,
                callback_cancel.set,
            )
            timer.daemon = True
            timer.start()
            try:
                self.speaker(
                    intent.utterance,
                    intent.utterance_locale,
                    callback_cancel,
                )
                if callback_cancel.is_set():
                    self._metrics.add("speech_cancellations")
                    self._events.append(
                        "speech",
                        "speech_cancelled",
                        item.proposal.proposal_id,
                    )
                else:
                    self._metrics.add("speech_completed")
                    self._events.append(
                        "speech",
                        "speech_completed",
                        item.proposal.proposal_id,
                    )
            except Exception as error:
                self._metrics.add("speech_failures")
                self._events.append(
                    "speech",
                    "speech_failure",
                    type(error).__name__,
                )
            finally:
                timer.cancel()
                self._unregister_callback(callback_cancel)
                elapsed = max(
                    0,
                    int(round(
                        (time.monotonic() - started_at) * 1_000
                    )),
                )
                used_ms += elapsed
                self._metrics.add("speech_elapsed_ms", elapsed)
        self._events.append("speech", "worker_stopped")

    def _arm_revalidation(
        self,
        job: _ExpressionJob,
    ) -> bool:
        if self._now_ms() > job.valid_until_ms:
            return False
        current = self._interaction.current()
        if (
            current is None
            or current.drive_phase != "BLOCKED"
            or current.evidence is None
            or current.captured_at_ms
            - current.evidence.observed_at_ms
            > self.policy.max_evidence_age_ms
        ):
            return False
        try:
            job.proposal.assert_matches_snapshot(current)
        except InteractionContractError:
            return False
        return True

    def _release_navigation_pause(self) -> None:
        self._pause_released.set()
        self._pause_requested.clear()
        self._events.append("arm", "navigation_pause_released")

    def _arm_worker(self) -> None:
        self._events.append("arm", "worker_started")
        used_ms = 0
        last_completed_at_ms: Optional[int] = None
        executed_epochs = set()
        while True:
            item = self._arm_queue.get()
            if item is _STOP:
                break
            if self._cancel_event.is_set():
                break
            epoch = item.proposal.obstruction_epoch
            intent = item.proposal.intent
            if intent.gesture_kind != "PROPELLER_WAVE":
                self._metrics.add("gesture_drops")
                self._events.append(
                    "arm",
                    "unsupported_gesture_drop",
                    item.proposal.proposal_id,
                )
                continue
            required_ms = (
                intent.repetitions
                * 2
                * self.policy.arm_segment_duration_ms
            )
            now_ms = self._now_ms()
            if (
                epoch in executed_epochs
                or self._metrics.get("gestures_started")
                >= self.policy.max_gestures
                or used_ms + required_ms > self.policy.max_arm_time_ms
                or (
                    last_completed_at_ms is not None
                    and now_ms - last_completed_at_ms
                    < self.policy.gesture_cooldown_ms
                )
            ):
                self._metrics.add("gesture_drops")
                self._events.append(
                    "arm",
                    "gesture_budget_or_cooldown_drop",
                    item.proposal.proposal_id,
                )
                continue

            self._pause_acknowledged.clear()
            self._pause_released.clear()
            self._pause_requested.set()
            self._metrics.add("navigation_pause_requests")
            self._events.append(
                "arm",
                "navigation_pause_requested",
                item.proposal.proposal_id,
            )
            self._pause_acknowledged.wait()
            if self._cancel_event.is_set():
                self._release_navigation_pause()
                break
            if not self._arm_revalidation(item):
                self._metrics.add("gesture_drops")
                self._events.append(
                    "arm",
                    "stale_gesture_drop",
                    item.proposal.proposal_id,
                )
                self._release_navigation_pause()
                continue

            executed_epochs.add(epoch)
            self._metrics.add("gestures_started")
            self._events.append(
                "arm",
                "gesture_started",
                item.proposal.proposal_id,
            )
            callback_cancel = _CallbackCancelEvent(
                self._cancel_event
            )
            self._register_callback(callback_cancel)
            completed = True
            try:
                for _repeat in range(intent.repetitions):
                    for direction in (1, -1):
                        if callback_cancel.is_set():
                            completed = False
                            break
                        self._events.append(
                            "arm",
                            "arm_segment_started",
                            str(direction),
                        )
                        used_ms += self.policy.arm_segment_duration_ms
                        self._metrics.add(
                            "arm_elapsed_ms",
                            self.policy.arm_segment_duration_ms,
                        )
                        self._metrics.add("arm_segments")
                        self.arm_segment_executor(
                            direction * self.policy.arm_speed_dps,
                            self.policy.arm_segment_duration_ms,
                            callback_cancel,
                        )
                        if callback_cancel.is_set():
                            completed = False
                            self._metrics.add("arm_cancellations")
                            self._events.append(
                                "arm",
                                "arm_cancelled",
                                item.proposal.proposal_id,
                            )
                            break
                    if not completed:
                        break
            except Exception as error:
                completed = False
                self._metrics.add("gesture_drops")
                self._events.append(
                    "arm",
                    "arm_failure",
                    type(error).__name__,
                )
            finally:
                self._unregister_callback(callback_cancel)
                last_completed_at_ms = self._now_ms()
                if completed:
                    self._metrics.add("gestures_completed")
                    self._events.append(
                        "arm",
                        "gesture_completed",
                        item.proposal.proposal_id,
                    )
                self._release_navigation_pause()
        self._events.append("arm", "worker_stopped")

    def _before_arbitration(
        self,
        snapshot: NavigationSnapshot,
    ) -> None:
        self._events.append(
            "motion",
            "navigation_tick",
            str(snapshot.state_version),
        )
        self._tick_hook(
            self._cancel_event,
            self.policy.tick_interval_s,
        )
        if self._cancel_event.is_set():
            return
        if not self._pause_requested.is_set():
            return
        if snapshot.motors_running:
            self._events.append(
                "motion",
                "navigation_pause_deferred",
                "motors_running",
            )
            return
        self._metrics.add("navigation_pause_acks")
        self._events.append(
            "motion",
            "navigation_pause_ack",
            str(snapshot.state_version),
        )
        self._pause_acknowledged.set()
        released = self._pause_released.wait(
            self.policy.arm_exclusive_timeout_ms / 1_000.0
        )
        if not released:
            self._metrics.add("arm_exclusive_timeouts")
            self._events.append(
                "motion",
                "arm_exclusive_timeout",
                str(snapshot.state_version),
            )
            self.cancel()
            return None
        self._events.append(
            "motion",
            "post_pause_observation_refresh_requested",
            str(snapshot.state_version),
        )
        return NAVIGATION_REFRESH_REQUIRED

    def _start_worker(
        self,
        name: str,
        target: Callable[..., None],
        args: Tuple[object, ...] = (),
    ) -> None:
        thread = threading.Thread(
            name="robot-runtime-{}".format(name),
            target=target,
            args=args,
            daemon=True,
        )
        self._threads.append((name, thread))
        thread.start()

    @staticmethod
    def _offer_stop(target: queue.Queue) -> None:
        try:
            target.put_nowait(_STOP)
            return
        except queue.Full:
            pass
        try:
            target.get_nowait()
        except queue.Empty:
            pass
        try:
            target.put_nowait(_STOP)
        except queue.Full:
            pass

    def _close_queues(self) -> None:
        for target in self._behavior_queues.values():
            self._offer_stop(target)
        self._offer_stop(self._planner_queue)
        self._offer_stop(self._speech_queue)
        self._offer_stop(self._arm_queue)

    def _start_workers(self) -> None:
        for behavior in self.behaviors:
            self._start_worker(
                behavior.source_id,
                self._behavior_worker,
                (behavior,),
            )
        if self.expression_planner is not None:
            self._start_worker(
                "expression-planner",
                self._planner_worker,
            )
        if self.speaker is not None:
            self._start_worker("speech", self._speech_worker)
        if self.arm_segment_executor is not None:
            self._start_worker("arm", self._arm_worker)

    def run(self) -> ConcurrentRuntimeResult:
        """Run one navigation episode and cooperatively stop all workers."""

        with self._run_lock:
            if self._has_run:
                raise RuntimeError(
                    "ConcurrentBehaviorRuntime can only run once"
                )
            self._has_run = True
        self._events.append("runtime", "runtime_started")
        self._start_workers()
        episode = NavigationEpisode(
            self.plant,
            self.supervisor,
            self.inbox,
            local_behaviors=(),
            limits=self.navigation_limits,
            observation_sink=self._observation_sink,
            before_arbitration=self._before_arbitration,
            cancel_event=self._cancel_event,
        )
        try:
            navigation = episode.run(self.goal)
        finally:
            self._cancel_event.set()
            self._pause_acknowledged.set()
            self._pause_released.set()
            with self._callback_lock:
                callbacks = tuple(self._active_callback_events)
            for callback_event in callbacks:
                callback_event.set()
            self._close_queues()
            for _name, thread in self._threads:
                thread.join(self.policy.shutdown_join_timeout_s)
        workers_alive = tuple(
            name for name, thread in self._threads if thread.is_alive()
        )
        self._events.append(
            "runtime",
            "runtime_stopped",
            "alive={}".format(",".join(workers_alive)),
        )
        return ConcurrentRuntimeResult(
            navigation=navigation,
            metrics=self._metrics.snapshot(self._events.dropped),
            events=self._events.snapshot(),
            clean_shutdown=not workers_alive,
            workers_alive=workers_alive,
        )
