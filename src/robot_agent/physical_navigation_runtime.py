"""Bounded host runtime for model-planned, worker-executed EV3 navigation."""

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math
import sys
import threading
import time
from typing import Callable, Mapping, Optional, Tuple

from .active_ir_scan_contract import (
    ActiveIrScanCalibration,
)
from .lm_studio_navigation import (
    MAX_RECENT_COMMITTED_UTTERANCES,
    LMStudioNavigationDecisionError,
    LMStudioNavigationError,
    NavigationPlannerResult,
)
from .maneuver_commitment import (
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
    ManeuverCommitment,
    ManeuverCommitmentError,
)
from .navigation_memory_store import (
    NavigationMemoryError,
    NavigationMemoryStore,
)
from .navigation_plan_tail import (
    PLAN_TAIL_MAX_AGE_SECONDS,
    NavigationPlanTail,
)
from . import physical_action_feasibility
from .local_detour_controller import filter_local_detour_actions
from .local_detour_route import ROUTE_ACTIVE
from .physical_navigation_experience import (
    NavigationExperienceLedger,
    PLANNER_ACTION_SOURCE,
    navigation_evidence_basis,
)
from .physical_observation_progress import (
    observation_information_result,
    observe_without_information_gain,
)
from .physical_navigation_runtime_errors import (
    EpisodeCancelled as _EpisodeCancelled,
    PhysicalNavigationRuntimeError,
)
from .physical_navigation_scan_runtime import (
    DEFAULT_SCAN_BUDGET,
    PhysicalNavigationScanRuntimeMixin,
)
from .physical_navigation_route_runtime import (
    EXECUTION_FAILED,
    PhysicalNavigationRouteRuntimeMixin,
)
from .physical_navigation_plan_tail_runtime import (
    PhysicalNavigationPlanTailRuntimeMixin,
)
from .physical_navigation_contract import (
    ADVANCE,
    EXPECTED_WORKER_SAFETY,
    EXPECTED_WORKER_OPERATIONS,
    EXPECTED_ACTION_SPECS,
    FINISH,
    OBSERVE,
    REVERSE,
    SCAN_FRONT_ARC,
    SCAN_SAMPLE_OPERATION,
    SCAN_TURN_OPERATION,
    NavigationDecision,
    PhysicalNavigationContractError,
    expected_scan_turn_profile,
    expected_scan_sample_profile,
    motion_budget_allows,
    validate_observation,
)
from .physical_navigation_mission import DirectionalMission
from .physical_navigation_motion_runtime import (
    PhysicalNavigationMotionRuntimeMixin,
)
from .physical_odometry import (
    DriveMotorRoles,
)

DEFAULT_MAX_TURNS = 14
MAX_TURNS_PER_EPISODE_SECOND = 4
HARD_MAX_TURNS = 14_400
DEFAULT_MAX_EPISODE_SECONDS = 35.0
MIN_EPISODE_SECONDS = 1.0
MAX_EPISODE_SECONDS = 60.0 * 60.0
SUPPORTED_EPISODE_LOCALES = frozenset(("sv", "en"))
HOST_PER_SLICE_HEADROOM_SECONDS = 0.25
HOST_RESPONSE_HEADROOM_SECONDS = 0.75
DEFAULT_SCAN_TIMEOUT_SECONDS = (
    (DEFAULT_SCAN_BUDGET["minimum_deadline_ms"] + 999) // 1000
)
MAX_SCAN_TIMEOUT_SECONDS = 120.0

class _LogicalEpisodeTermination(Exception):
    def __init__(self, reason: str):
        self.reason = reason
        super().__init__(reason)


@dataclass(frozen=True)
class PhysicalNavigationRuntimeConfig:
    goal: str
    locale: str
    minimum_forward_progress_mm: int = 420
    goal_heading_tolerance_mdeg: int = 5_000
    max_turns: Optional[int] = None
    max_episode_seconds: float = DEFAULT_MAX_EPISODE_SECONDS
    startup_timeout_seconds: float = 30.0
    request_timeout_seconds: float = 8.0
    scan_timeout_seconds: float = float(DEFAULT_SCAN_TIMEOUT_SECONDS)
    plan_tail_max_age_seconds: float = PLAN_TAIL_MAX_AGE_SECONDS
    max_validation_attempts: int = 2

    def __post_init__(self) -> None:
        duration_is_valid = (
            not isinstance(self.max_episode_seconds, bool)
            and isinstance(self.max_episode_seconds, (int, float))
            and MIN_EPISODE_SECONDS
            <= float(self.max_episode_seconds)
            <= MAX_EPISODE_SECONDS
        )
        if self.max_turns is None and duration_is_valid:
            object.__setattr__(
                self,
                "max_turns",
                min(
                    HARD_MAX_TURNS,
                    max(
                        DEFAULT_MAX_TURNS,
                        int(
                            math.ceil(
                                float(self.max_episode_seconds)
                                * MAX_TURNS_PER_EPISODE_SECOND
                            )
                        ),
                    ),
                ),
            )
        if (
            not isinstance(self.goal, str)
            or not self.goal.strip()
            or len(self.goal) > 2_000
            or self.locale not in SUPPORTED_EPISODE_LOCALES
            or isinstance(self.minimum_forward_progress_mm, bool)
            or not isinstance(self.minimum_forward_progress_mm, int)
            or not 1 <= self.minimum_forward_progress_mm <= 2_000
            or isinstance(self.goal_heading_tolerance_mdeg, bool)
            or not isinstance(self.goal_heading_tolerance_mdeg, int)
            or not 1_000 <= self.goal_heading_tolerance_mdeg <= 45_000
            or isinstance(self.max_turns, bool)
            or not isinstance(self.max_turns, int)
            or not 1 <= self.max_turns <= HARD_MAX_TURNS
            or not duration_is_valid
            or isinstance(self.startup_timeout_seconds, bool)
            or not isinstance(self.startup_timeout_seconds, (int, float))
            or not 0.1 <= float(self.startup_timeout_seconds) <= 60.0
            or isinstance(self.request_timeout_seconds, bool)
            or not isinstance(self.request_timeout_seconds, (int, float))
            or not 0.1 <= float(self.request_timeout_seconds) <= 60.0
            or isinstance(self.scan_timeout_seconds, bool)
            or not isinstance(self.scan_timeout_seconds, (int, float))
            or not DEFAULT_SCAN_TIMEOUT_SECONDS
            <= float(self.scan_timeout_seconds)
            <= MAX_SCAN_TIMEOUT_SECONDS
            or isinstance(self.plan_tail_max_age_seconds, bool)
            or not isinstance(
                self.plan_tail_max_age_seconds,
                (int, float),
            )
            or not 1.0 <= float(self.plan_tail_max_age_seconds) <= 120.0
            or isinstance(self.max_validation_attempts, bool)
            or not isinstance(self.max_validation_attempts, int)
            or not 1 <= self.max_validation_attempts <= 3
        ):
            raise ValueError("physical navigation runtime config is invalid")


@dataclass(frozen=True)
class PhysicalNavigationResult:
    terminal_reason: str
    completed: bool
    turns: int
    actions: Tuple[str, ...]
    model_calls: int
    model_latency_ms: int
    plan_tails_completed: int
    plan_tails_cancelled: int
    final_mission: Mapping[str, object]
    final_navigation: Mapping[str, object]
    shutdown_clean: bool

    def to_dict(self) -> Mapping[str, object]:
        return {
            "terminal_reason": self.terminal_reason,
            "completed": self.completed,
            "turns": self.turns,
            "actions": list(self.actions),
            "model_calls": self.model_calls,
            "model_latency_ms": self.model_latency_ms,
            "plan_tails_completed": self.plan_tails_completed,
            "plan_tails_cancelled": self.plan_tails_cancelled,
            "final_mission": deepcopy(self.final_mission),
            "final_navigation": deepcopy(self.final_navigation),
            "shutdown_clean": self.shutdown_clean,
        }


class PhysicalNavigationRuntime(
    PhysicalNavigationMotionRuntimeMixin,
    PhysicalNavigationRouteRuntimeMixin,
    PhysicalNavigationPlanTailRuntimeMixin,
    PhysicalNavigationScanRuntimeMixin,
):
    """Serializes physical execution while model reasoning stays replaceable."""

    def __init__(
        self,
        *,
        episode_id: str,
        config: PhysicalNavigationRuntimeConfig,
        transport,
        transport_factory: Optional[Callable[[], object]] = None,
        planner,
        memory: NavigationMemoryStore,
        active_scan_executor=None,
        active_scan_executor_factory: Optional[
            Callable[[object], object]
        ] = None,
        active_scan_calibration: ActiveIrScanCalibration = (
            ActiveIrScanCalibration()
        ),
        monotonic: Callable[[], float] = time.monotonic,
        unix_ms: Callable[[], int] = lambda: time.time_ns() // 1_000_000,
        event_sink: Optional[Callable[[Mapping[str, object]], None]] = None,
        observation_sink: Optional[Callable[..., object]] = None,
        validated_utterance_sink: Optional[Callable[[str], object]] = None,
        cancel_event=None,
        emergency_event=None,
    ):
        if not isinstance(episode_id, str) or not episode_id:
            raise ValueError("episode_id is invalid")
        if not isinstance(config, PhysicalNavigationRuntimeConfig):
            raise ValueError("runtime config is invalid")
        if any(
            not callable(getattr(transport, name, None))
            for name in ("start", "request", "close")
        ):
            raise ValueError("navigation transport is invalid")
        if transport_factory is not None and not callable(transport_factory):
            raise ValueError("navigation transport factory is invalid")
        if (
            active_scan_executor_factory is not None
            and not callable(active_scan_executor_factory)
        ):
            raise ValueError("active scan executor factory is invalid")
        if not isinstance(
            active_scan_calibration,
            ActiveIrScanCalibration,
        ):
            raise ValueError("active scan calibration is invalid")
        if not callable(getattr(planner, "decide", None)):
            raise ValueError("navigation planner is invalid")
        if not isinstance(memory, NavigationMemoryStore):
            raise ValueError("navigation memory is invalid")
        if not callable(monotonic) or not callable(unix_ms):
            raise ValueError("runtime clocks are invalid")
        if event_sink is not None and not callable(event_sink):
            raise ValueError("runtime event sink is invalid")
        if observation_sink is not None and not callable(observation_sink):
            raise ValueError("runtime observation sink is invalid")
        if (
            validated_utterance_sink is not None
            and not callable(validated_utterance_sink)
        ):
            raise ValueError("validated utterance sink is invalid")
        self.episode_id = episode_id
        self.config = config
        self.transport = transport
        self.transport_factory = transport_factory
        self.planner = planner
        self.memory = memory
        self.active_scan_executor = active_scan_executor
        self.active_scan_executor_factory = active_scan_executor_factory
        self.active_scan_calibration = active_scan_calibration
        self.monotonic = monotonic
        self.unix_ms = unix_ms
        self.event_sink = event_sink
        self.observation_sink = observation_sink
        self.validated_utterance_sink = validated_utterance_sink
        self.cancel_event = cancel_event or threading.Event()
        self.emergency_event = emergency_event or threading.Event()
        self._stop_requested = threading.Event()
        self._cleanup_lock = threading.Lock()
        self._cleanup_started = False
        self._transport_started = False
        self._observation_received_monotonic = None
        self._latest_validated_observation = None
        self._worker_absolute_max_ms = None
        self._all_sessions_clean = True
        self._recent_committed_utterances = deque(
            maxlen=MAX_RECENT_COMMITTED_UTTERANCES
        )
        self._pass_side_probe_attempted_route_ids = set()
        self._restored_scan_progress_barriers = {}
        self._experience_ledger = NavigationExperienceLedger(
            episode_id=episode_id,
        )

    def request_stop(self) -> None:
        self._stop_requested.set()

    def emergency_stop(self) -> None:
        self._stop_requested.set()
        abort = getattr(self.transport, "abort", None)
        if callable(abort):
            abort()

    def _cancelled(self) -> bool:
        return (
            self._stop_requested.is_set()
            or self.cancel_event.is_set()
            or self.emergency_event.is_set()
        )

    def _raise_if_cancelled(self, stage: str) -> None:
        if self._cancelled():
            self._emit("cancellation_observed", stage=stage)
            raise _EpisodeCancelled(stage)

    def _emit(self, event: str, **fields) -> None:
        if self.event_sink is None:
            return
        value = {"event": event, "episode_id": self.episode_id}
        value.update(fields)
        self.event_sink(value)

    def _offer_observation(
        self,
        observation: Mapping[str, object],
        *,
        captured_at_ms: int,
        publication_stage: str,
    ) -> None:
        """Offer committed physical state without granting map authority."""

        if self.observation_sink is None:
            return
        try:
            accepted = self.observation_sink(
                memory=self.memory,
                observation=validate_observation(observation),
                episode_id=self.episode_id,
                captured_at_ms=captured_at_ms,
            )
        except Exception as error:
            try:
                self._emit(
                    "spatial_map_observation_failed",
                    publication_stage=publication_stage,
                    error_type=type(error).__name__,
                )
            except Exception:
                # A broken general telemetry sink cannot reintroduce map
                # authority through this failure-reporting path.
                pass
            return
        if accepted is False:
            try:
                self._emit(
                    "spatial_map_observation_rejected",
                    publication_stage=publication_stage,
                )
            except Exception:
                pass

    def _remember_observation(
        self,
        observation: Mapping[str, object],
    ) -> None:
        """Retain detached evidence for a later localization invalidation."""
        checked = validate_observation(observation)
        self._latest_validated_observation = deepcopy(checked)

    def _offer_invalid_localization(
        self,
        *,
        captured_at_ms: int,
        publication_stage: str,
        observation: Optional[Mapping[str, object]] = None,
    ) -> None:
        """Publish a persisted invalidation without changing its outcome."""

        if self.memory.localization_valid:
            return
        evidence = (
            observation
            if observation is not None
            else self._latest_validated_observation
        )
        if evidence is None:
            try:
                self._emit(
                    "spatial_map_invalidation_unpublished",
                    publication_stage=publication_stage,
                    reason="no_validated_observation",
                )
            except Exception:
                pass
            return
        self._offer_observation(
            evidence,
            captured_at_ms=captured_at_ms,
            publication_stage=publication_stage,
        )

    def _invalidate_localization(
        self,
        reason: str,
        *,
        publication_stage: str,
        observation: Optional[Mapping[str, object]] = None,
    ) -> None:
        captured_at_ms = self.unix_ms()
        self.memory.invalidate_localization(reason, captured_at_ms)
        self._offer_invalid_localization(
            captured_at_ms=captured_at_ms,
            publication_stage=publication_stage,
            observation=observation,
        )

    def _commit_decision(
        self,
        decision: NavigationDecision,
        *,
        turn: int,
    ) -> None:
        """Publish and offer speech only for the action being dispatched."""

        self._raise_if_cancelled("immediately_before_decision_commit")
        self._emit(
            "model_decision_committed",
            turn=turn,
            action=decision.action,
            plan=list(decision.plan),
            assessment=decision.assessment,
            utterance=decision.utterance,
            decision_status="committed",
        )
        if (
            decision.utterance is None
            or self.validated_utterance_sink is None
        ):
            return
        self._raise_if_cancelled(
            "immediately_before_committed_utterance_offer"
        )
        try:
            self.validated_utterance_sink(decision.utterance)
        except Exception as error:
            # Speech is advisory output. It must never become motor authority
            # or turn an otherwise valid physical action into a fault.
            self._emit(
                "speech_offer_failed",
                turn=turn,
                action=decision.action,
                speech_status="failed",
                reason=type(error).__name__,
            )
        else:
            self._recent_committed_utterances.append(decision.utterance)

    def _active_request(
        self,
        operation: str,
        arguments: Mapping[str, object],
        timeout_seconds: float,
    ) -> Mapping[str, object]:
        self._raise_if_cancelled(
            "immediately_before_{}_request".format(operation)
        )
        physical_motion_operation = operation in (
            "pulse",
            SCAN_TURN_OPERATION,
        )
        cancellable_operation = physical_motion_operation or (
            operation == SCAN_SAMPLE_OPERATION
        )
        try:
            response = self.transport.request(
                operation,
                arguments,
                timeout_seconds,
                # Closing the SSH channel is the hard interruption mechanism
                # for a possibly active motor pulse.  Read-only requests can
                # finish their bounded response after STOP, preserving the
                # channel long enough to obtain the verified shutdown receipt.
                cancel_requested=(
                    self._cancelled if cancellable_operation else None
                ),
            )
        except Exception:
            # The SSH transport closes its channel when the callback turns
            # true. Convert that expected transport failure into the episode's
            # cancellation result, while preserving unrelated failures.  A
            # pulse may already have moved either wheel before the channel is
            # closed, so without its correlated encoder receipt the persisted
            # pose is no longer trustworthy.
            if (
                physical_motion_operation
                and self._cancelled()
                and self.memory.localization_valid
            ):
                self._invalidate_localization(
                    "Pulse cancellation lost its correlated encoder receipt",
                    publication_stage="pulse_receipt_lost",
                )
            self._raise_if_cancelled("during_{}_request".format(operation))
            raise
        # If cancellation raced with a response that is already complete, let
        # the caller consume that stopped, validated encoder receipt.  The next
        # cancellation boundary will prevent any later physical action.
        if cancellable_operation and self._cancelled():
            return response
        self._raise_if_cancelled("after_{}_request".format(operation))
        return response

    @staticmethod
    def _description(
        response: Mapping[str, object],
    ) -> Tuple[
        Mapping[str, object],
        Mapping[str, Mapping[str, object]],
        DriveMotorRoles,
        int,
    ]:
        result = response.get("result")
        if not isinstance(result, dict):
            raise PhysicalNavigationRuntimeError(
                "invalid_worker_description",
                "Worker description is missing",
            )
        required = {
            "worker_id",
            "demo_only",
            "policy_owner",
            "controller_id",
            "request_schema",
            "response_schema",
            "operations",
            "pulse",
            "scan_turn",
            "scan_sample",
            "safety",
            "process",
            "observation",
            "drive_geometry",
        }
        if set(result) != required or result["policy_owner"] != "host":
            raise PhysicalNavigationRuntimeError(
                "invalid_worker_description",
                "Worker identity/policy boundary is invalid",
            )
        pulse = result["pulse"]
        safety = result["safety"]
        process = result["process"]
        if (
            not isinstance(process, dict)
            or set(process) != {"absolute_max_ms", "max_requests"}
            or isinstance(process["absolute_max_ms"], bool)
            or not isinstance(process["absolute_max_ms"], int)
            or not 5_000 <= process["absolute_max_ms"] <= 180_000
            or isinstance(process["max_requests"], bool)
            or not isinstance(process["max_requests"], int)
            or process["max_requests"] <= 0
        ):
            raise PhysicalNavigationRuntimeError(
                "invalid_worker_process_contract",
                "Worker process lifetime contract is invalid",
            )
        geometry = result["drive_geometry"]
        if (
            not isinstance(geometry, dict)
            or set(geometry)
            != {
                "left_motor_role",
                "right_motor_role",
                "forward_speed_sign",
            }
            or not isinstance(geometry["forward_speed_sign"], dict)
        ):
            raise PhysicalNavigationRuntimeError(
                "invalid_worker_drive_geometry",
                "Worker drive geometry is invalid",
            )
        drive_roles = DriveMotorRoles(
            left=geometry["left_motor_role"],
            right=geometry["right_motor_role"],
        )
        if (
            set(geometry["forward_speed_sign"])
            != {drive_roles.left, drive_roles.right}
            or geometry["forward_speed_sign"][drive_roles.left] != 1
            or geometry["forward_speed_sign"][drive_roles.right] != 1
        ):
            raise PhysicalNavigationRuntimeError(
                "unsupported_worker_drive_sign",
                "Semantic action profile requires positive-forward drive roles",
            )
        if (
            not isinstance(pulse, dict)
            or pulse.get("actions") != EXPECTED_ACTION_SPECS
            or not isinstance(result["operations"], list)
            or set(result["operations"]) != EXPECTED_WORKER_OPERATIONS
            or result["scan_turn"] != expected_scan_turn_profile()
            or result["scan_sample"] != expected_scan_sample_profile()
            or safety != EXPECTED_WORKER_SAFETY
        ):
            raise PhysicalNavigationRuntimeError(
                "unsafe_worker_contract",
                "Worker safety or semantic action contract is invalid",
            )
        observation = validate_observation(result["observation"])
        return (
            observation,
            deepcopy(pulse["actions"]),
            drive_roles,
            process["absolute_max_ms"],
        )

    def _observation_from_response(
        self,
        operation: str,
        response: Mapping[str, object],
        expected_action: Optional[str] = None,
    ) -> Mapping[str, object]:
        result = response.get("result")
        if not isinstance(result, dict):
            raise PhysicalNavigationRuntimeError(
                "invalid_worker_result",
                "Worker result is missing",
            )
        if operation == "observe":
            if set(result) != {"observation"}:
                raise PhysicalNavigationRuntimeError(
                    "invalid_observe_result",
                    "Observe result fields are invalid",
                )
            observation = validate_observation(result["observation"])
        elif operation == "pulse":
            if (
                set(result) != {"action", "outcome", "observation", "stop"}
                or result["action"] != expected_action
                or not isinstance(result["outcome"], dict)
                or result["outcome"].get("action") != expected_action
                or result["outcome"].get("stop_confirmed") is not True
            ):
                raise PhysicalNavigationRuntimeError(
                    "invalid_pulse_result",
                    "Pulse result is not correlated and stopped",
                )
            observation = validate_observation(result["observation"])
            if observation["last_outcome"] != result["outcome"]:
                raise PhysicalNavigationRuntimeError(
                    "pulse_outcome_mismatch",
                    "Pulse observation lacks its correlated outcome",
                )
        elif operation in (SCAN_TURN_OPERATION, SCAN_SAMPLE_OPERATION):
            # EV3NavigationSSHTransport has already validated the complete
            # operation-specific receipt. Accept only its correlated snapshot.
            observation = validate_observation(result.get("observation"))
        else:
            raise PhysicalNavigationRuntimeError(
                "invalid_observation_operation",
                "Operation has no observation contract",
            )
        if response.get("state_version") != observation["state_version"]:
            raise PhysicalNavigationRuntimeError(
                "worker_state_version_mismatch",
                "Response and observation state versions differ",
            )
        self._remember_observation(observation)
        self._observation_received_monotonic = self.monotonic()
        return observation

    def _goal_state(
        self,
        mission: DirectionalMission,
        observation: Mapping[str, object],
        action_specs: Mapping[str, Mapping[str, object]],
    ) -> Tuple[Mapping[str, object], Mapping[str, object]]:
        geometry = self.memory.hazard_map.goal_geometry(
            pose=self.memory.pose,
            goal_heading_mdeg=mission.reference_heading_mdeg,
            heading_tolerance_mdeg=mission.heading_tolerance_mdeg,
        )
        facts = geometry["facts"]
        target_values = facts[FACT_TARGET_BEHIND]
        mission_value = mission.snapshot(
            pose=self.memory.pose,
            action_specs=action_specs,
            goal_corridor_clear=facts[FACT_GOAL_CORRIDOR_CLEAR],
            all_known_hazards_passed=all(target_values.values()),
            localization_valid=self.memory.localization_valid,
            touch_pressed=observation["touch"]["pressed"],
            calibration=self.memory.odometry_calibration,
        )
        mission_value = dict(mission_value)
        mission_value["user_goal"] = self.config.goal
        fact_values = {
            FACT_GOAL_CORRIDOR_CLEAR: facts[
                FACT_GOAL_CORRIDOR_CLEAR
            ],
            FACT_GOAL_HEADING_ALIGNED: facts[
                FACT_GOAL_HEADING_ALIGNED
            ],
            FACT_TARGET_BEHIND: deepcopy(target_values),
        }
        navigation = dict(self.memory.context())
        navigation["goal_geometry"] = geometry
        navigation["fact_values"] = deepcopy(fact_values)
        feasibility = physical_action_feasibility.navigation_action_feasibility(
            hazard_map=self.memory.hazard_map,
            pose=self.memory.pose,
            action_specs=action_specs,
            odometry_calibration=self.memory.odometry_calibration,
            active_scan_calibration=self.active_scan_calibration,
        )
        navigation["action_feasibility"] = deepcopy(feasibility)
        # Keep the focused field during the dashboard/planner contract
        # transition while publishing every motion action in the generic
        # feasibility table above.
        navigation["scan_front_arc_feasibility"] = deepcopy(
            feasibility["active_scan"]
        )
        navigation["experience_ledger"] = self._experience_ledger.context(
            current_basis=navigation_evidence_basis(
                navigation,
                observation,
            )
        )
        return mission_value, navigation

    def _experience_basis(
        self,
        observation: Mapping[str, object],
    ) -> Mapping[str, object]:
        return navigation_evidence_basis(
            self.memory.context(),
            observation,
        )

    def _record_experience(
        self,
        *,
        turn: int,
        action: str,
        source: str,
        result: Mapping[str, object],
        basis_before: Mapping[str, object],
        observation_after: Mapping[str, object],
    ) -> None:
        entry = self._experience_ledger.record(
            turn=turn,
            action=action,
            source=source,
            result=result,
            basis_before=basis_before,
            basis_after=self._experience_basis(observation_after),
        )
        self._emit(
            "navigation_experience_recorded",
            experience=entry.to_dict(),
            host_ranked_or_selected_action=False,
        )

    @staticmethod
    def _validate_mission_decision(
        decision: NavigationDecision,
        mission: Mapping[str, object],
        navigation: Mapping[str, object],
    ) -> None:
        action = decision.action
        if action == FINISH:
            if (
                decision.plan != (FINISH,)
                or mission["completed"] is not True
            ):
                raise PhysicalNavigationRuntimeError(
                    "premature_mission_finish",
                    "FINISH requires every directional mission fact",
                )
        delta = mission["candidate_action_longitudinal_deltas_mm"].get(action)
        if (
            delta is not None
            and delta < 0
            and not navigation["navigation_hazard_hypotheses"]
        ):
            raise PhysicalNavigationRuntimeError(
                "regression_without_hazard",
                "Negative progress requires a published hazard",
            )
        detour_error = physical_action_feasibility.detour_decision_error(
            action,
            decision.perception_target_hypothesis_id,
            decision.maneuver_commitment,
            navigation,
        )
        if detour_error is not None:
            raise PhysicalNavigationRuntimeError(*detour_error)

    def _remaining_seconds(self, deadline: float) -> float:
        return max(0.0, deadline - self.monotonic())

    def _post_planner_gate(self, deadline: float, stage: str) -> None:
        """Reject planner output that arrived after stop or episode expiry."""

        self._raise_if_cancelled(stage)
        if self.monotonic() >= deadline:
            self._emit(
                "planner_output_discarded",
                stage=stage,
                reason="episode_deadline_elapsed",
            )
            raise _LogicalEpisodeTermination("episode_deadline_elapsed")

    def _execution_veto(
        self,
        *,
        action: str,
        observation: Mapping[str, object],
        action_specs: Mapping[str, Mapping[str, object]],
        deadline: float,
    ) -> Optional[Mapping[str, object]]:
        if not motion_budget_allows(action, observation, action_specs):
            return {
                "code": "worker_budget_insufficient",
                "action": action,
                "host_selected_alternative_action": False,
            }
        swept = self.memory.hazard_map.validate_swept_path(
            self.memory.pose,
            action,
            action_specs,
            self.memory.odometry_calibration,
        )
        if not swept["allowed"]:
            return {
                "code": swept["reason"],
                "action": action,
                "swept_path": swept,
                "host_selected_alternative_action": False,
            }
        spec = action_specs[action]
        required = (
            spec["total_duration_ms"] / 1000.0
            + spec["slice_count"] * HOST_PER_SLICE_HEADROOM_SECONDS
            + HOST_RESPONSE_HEADROOM_SECONDS
        )
        if self._remaining_seconds(deadline) < required:
            return {
                "code": "host_deadline_headroom_insufficient",
                "action": action,
                "required_seconds": required,
                "host_selected_alternative_action": False,
            }
        return None

    def _validated_decision(
        self,
        *,
        turn: int,
        deadline: float,
        observation: Mapping[str, object],
        mission: Mapping[str, object],
        navigation: Mapping[str, object],
        maneuver: ManeuverCommitment,
        available_actions,
        last_tool_result,
        counters,
        validation_feedback=None,
    ) -> Tuple[Optional[NavigationDecision], Optional[Mapping[str, object]]]:
        feedback = validation_feedback
        for attempt in range(1, self.config.max_validation_attempts + 1):
            counters["model_calls"] += 1
            try:
                planner_result = self.planner.decide(
                    episode_id=self.episode_id,
                    turn=turn,
                    locale=self.config.locale,
                    observation=observation,
                    mission=mission,
                    navigation=navigation,
                    maneuver_state=maneuver.state(turn),
                    available_actions=available_actions,
                    last_tool_result=last_tool_result,
                    validation_feedback=feedback,
                    recent_committed_utterances=tuple(
                        self._recent_committed_utterances
                    ),
                )
            except LMStudioNavigationDecisionError as error:
                # The request completed, but the model-authored proposal did
                # not satisfy the decision contract. This is model feedback,
                # not a transport failure and never an invitation for the
                # host to substitute an action.
                counters["model_latency_ms"] += max(0, error.latency_ms)
                self._post_planner_gate(
                    deadline,
                    "after_invalid_planner_return",
                )
                feedback = {
                    "code": error.code,
                    "message": error.feedback_message,
                    "host_selected_alternative_action": False,
                }
                self._emit(
                    "decision_vetoed",
                    turn=turn,
                    attempt=attempt,
                    validation_feedback=feedback,
                )
                continue
            except LMStudioNavigationError as error:
                latency_ms = getattr(error, "latency_ms", 0)
                if (
                    isinstance(latency_ms, bool)
                    or not isinstance(latency_ms, int)
                    or latency_ms < 0
                ):
                    latency_ms = 0
                counters["model_latency_ms"] += latency_ms
                self._emit(
                    "planner_attempt_failed",
                    turn=turn,
                    attempt=attempt,
                    planner_error_code=getattr(
                        error,
                        "code",
                        "lm_studio_navigation_failed",
                    ),
                    model_latency_ms=latency_ms,
                    host_selected_alternative_action=False,
                )
                # A single slow or failed call does not end the sequence.
                # Cancellation and the absolute episode deadline still win
                # before the same model gets another bounded attempt. No
                # model-validation feedback or host-selected action is added.
                self._post_planner_gate(deadline, "after_planner_failure")
                if attempt < self.config.max_validation_attempts:
                    continue
                # The worker is synchronously verified stationary between
                # planner calls, so exhausted planner availability is a
                # logical inability to continue rather than a physical fault.
                self._emit(
                    "planner_termination",
                    turn=turn,
                    attempt=attempt,
                    terminal_reason="planner_unavailable",
                    planner_error_code=getattr(
                        error,
                        "code",
                        "lm_studio_navigation_failed",
                    ),
                    model_latency_ms=latency_ms,
                    host_selected_alternative_action=False,
                )
                raise _LogicalEpisodeTermination("planner_unavailable")
            # Planning may take seconds. A stop requested during that call
            # must win before the returned proposal can change state or start
            # any physical operation.
            if isinstance(planner_result, NavigationPlannerResult):
                decision = planner_result.decision
                latency_ms = planner_result.latency_ms
                served_model = planner_result.served_model
                planner_telemetry = {
                    "planner_context_bytes": planner_result.context_byte_count,
                    "prompt_tokens": planner_result.prompt_tokens,
                    "completion_tokens": planner_result.completion_tokens,
                    "total_tokens": planner_result.total_tokens,
                }
            elif isinstance(planner_result, NavigationDecision):
                decision = planner_result
                latency_ms = 0
                served_model = None
                planner_telemetry = {}
            else:
                self._post_planner_gate(deadline, "after_planner_return")
                raise PhysicalNavigationRuntimeError(
                    "invalid_planner_result",
                    "Planner returned the wrong result type",
                )
            counters["model_latency_ms"] += latency_ms
            self._post_planner_gate(deadline, "after_planner_return")
            self._emit(
                "model_decision",
                turn=turn,
                attempt=attempt,
                action=decision.action,
                plan=list(decision.plan),
                assessment=decision.assessment,
                utterance=decision.utterance,
                model_latency_ms=latency_ms,
                served_model=served_model,
                decision_status="proposed",
                **planner_telemetry,
            )
            try:
                if (
                    decision.action == SCAN_FRONT_ARC
                    and decision.perception_target_hypothesis_id
                    not in navigation.get(
                        "scan_eligible_target_hypothesis_ids",
                        tuple(
                            item["hypothesis_id"]
                            for item in navigation.get(
                                "navigation_hazard_hypotheses",
                                (),
                            )
                        ),
                    )
                ):
                    raise PhysicalNavigationRuntimeError(
                        "scan_target_requires_progress",
                        "Selected scan target requires intervening progress",
                    )
                self._validate_mission_decision(
                    decision,
                    mission,
                    navigation,
                )
                maneuver.apply(
                    decision.maneuver_commitment,
                    action=decision.action,
                    turn=turn,
                    hazard_map=self.memory.hazard_map,
                    pose=self.memory.pose,
                    fact_values=navigation["fact_values"],
                    perception_target_hypothesis_id=(
                        decision.perception_target_hypothesis_id
                    ),
                )
                return decision, None
            except (
                ManeuverCommitmentError,
                PhysicalNavigationRuntimeError,
            ) as error:
                code = getattr(error, "code", "decision_validation_failed")
                feedback = {
                    "code": code,
                    "message": str(error),
                    "host_selected_alternative_action": False,
                }
                self._emit(
                    "decision_vetoed",
                    turn=turn,
                    validation_feedback=feedback,
                )
        return None, feedback

    @staticmethod
    def _transport_is_valid(transport) -> bool:
        return all(
            callable(getattr(transport, name, None))
            for name in ("start", "request", "close")
        )

    def _start_worker_session(
        self,
        *,
        expected_action_specs=None,
        expected_drive_roles=None,
    ):
        self._raise_if_cancelled("immediately_before_worker_start")
        self.transport.start()
        self._transport_started = True
        self._raise_if_cancelled("immediately_before_worker_describe")
        description = self._active_request(
            "describe",
            {},
            self.config.startup_timeout_seconds,
        )
        (
            observation,
            action_specs,
            drive_roles,
            absolute_max_ms,
        ) = self._description(description)
        self._remember_observation(observation)
        if (
            expected_action_specs is not None
            and action_specs != expected_action_specs
        ):
            raise PhysicalNavigationRuntimeError(
                "worker_profile_changed_between_sessions",
                "Renewed worker exposed a different action profile",
            )
        if (
            expected_drive_roles is not None
            and drive_roles != expected_drive_roles
        ):
            raise PhysicalNavigationRuntimeError(
                "worker_geometry_changed_between_sessions",
                "Renewed worker exposed different drive geometry",
            )
        self.memory.bind_drive_roles(drive_roles)
        captured_at_ms = self.unix_ms()
        try:
            self.memory.begin_episode(observation, captured_at_ms)
        except Exception:
            self._offer_invalid_localization(
                captured_at_ms=captured_at_ms,
                publication_stage="worker_session_start_invalidated",
                observation=observation,
            )
            raise
        self._offer_observation(
            observation,
            captured_at_ms=captured_at_ms,
            publication_stage="worker_session_started",
        )
        self._observation_received_monotonic = self.monotonic()
        self._worker_absolute_max_ms = absolute_max_ms
        self._emit(
            "worker_session_started",
            worker_absolute_max_ms=absolute_max_ms,
        )
        return observation, action_specs, drive_roles

    def _shutdown_current_transport(self) -> bool:
        if not self._transport_started:
            return True
        clean = False
        try:
            if not getattr(self.transport, "shutdown_complete", False):
                response = self.transport.request(
                    "shutdown",
                    {},
                    min(4.0, self.config.request_timeout_seconds),
                )
                result = response.get("result", {})
                outcome = result.get("outcome", {})
                clean = (
                    outcome.get("status") == "completed"
                    and outcome.get("stop_confirmed") is True
                    and outcome.get("motor_owner_closed") is True
                )
            else:
                clean = True
        except Exception:
            clean = False
        try:
            self.transport.close()
        except Exception as error:
            # A validated shutdown response above already proves both the
            # physical stop and motor-owner closure.  Failure to reap the
            # local SSH wrapper is transport degradation, not contrary
            # physical evidence.
            self._emit(
                "transport_cleanup_degraded",
                physical_shutdown_verified=clean,
                error_type=type(error).__name__,
            )
        self._transport_started = False
        return clean

    def _session_renewal_headroom_ms(
        self,
        action_specs: Mapping[str, Mapping[str, object]],
        *,
        include_scan: bool,
    ) -> int:
        planner_timeout = getattr(self.planner, "timeout_seconds", 10.0)
        if (
            isinstance(planner_timeout, bool)
            or not isinstance(planner_timeout, (int, float))
            or planner_timeout <= 0
        ):
            planner_timeout = 10.0
        longest_action_seconds = max(
            spec["total_duration_ms"] / 1000.0
            + spec["slice_count"] * HOST_PER_SLICE_HEADROOM_SECONDS
            + HOST_RESPONSE_HEADROOM_SECONDS
            for spec in action_specs.values()
        )
        longest_post_plan_work_seconds = 3 * longest_action_seconds
        if include_scan:
            longest_post_plan_work_seconds = max(
                longest_post_plan_work_seconds,
                self.config.scan_timeout_seconds
                + self.config.request_timeout_seconds,
            )
        # One model call may author three exact actions. Renewal happens only
        # between plans, never in the middle of a model-authored tail or scan.
        return int(
            round(
                (
                    float(planner_timeout)
                    + longest_post_plan_work_seconds
                    + 2.0
                )
                * 1000
            )
        )

    def _effective_worker_process_ms(
        self,
        observation: Mapping[str, object],
    ) -> int:
        received = self._observation_received_monotonic
        elapsed_ms = (
            0
            if received is None
            else max(0, int(round((self.monotonic() - received) * 1000)))
        )
        return max(
            0,
            observation["budgets"]["process_ms_remaining"] - elapsed_ms,
        )

    def _worker_session_needs_renewal(
        self,
        observation: Mapping[str, object],
        action_specs: Mapping[str, Mapping[str, object]],
    ) -> bool:
        budgets = observation["budgets"]
        effective_process_ms = self._effective_worker_process_ms(observation)
        maximum_slices = max(
            spec["slice_count"] for spec in action_specs.values()
        )
        maximum_duration_ms = max(
            spec["total_duration_ms"] for spec in action_specs.values()
        )
        scan_headroom_ms = self._session_renewal_headroom_ms(
            action_specs,
            include_scan=True,
        )
        include_scan = (
            self.active_scan_executor is not None
            and self.transport_factory is not None
            and self._worker_absolute_max_ms is not None
            and scan_headroom_ms <= self._worker_absolute_max_ms
        )
        required_slices = 3 * maximum_slices
        required_duration_ms = 3 * maximum_duration_ms
        if include_scan:
            required_slices = max(
                required_slices,
                DEFAULT_SCAN_BUDGET["turn_slice_count"],
            )
            required_duration_ms = max(
                required_duration_ms,
                DEFAULT_SCAN_BUDGET["turn_duration_ms"],
            )
        return (
            budgets["motion_fault_latched"] is True
            or effective_process_ms
            < self._session_renewal_headroom_ms(
                action_specs,
                include_scan=include_scan,
            )
            or budgets["pulse_count_remaining"] < required_slices
            or budgets["pulse_duration_ms_remaining"]
            < required_duration_ms
        )

    def _renew_worker_session(
        self,
        *,
        action_specs: Mapping[str, Mapping[str, object]],
        drive_roles: DriveMotorRoles,
    ):
        self._raise_if_cancelled("before_worker_session_renewal")
        if self.transport_factory is None:
            raise PhysicalNavigationRuntimeError(
                "worker_session_renewal_unavailable",
                "Logical episode outlived one worker and no renewal factory exists",
            )
        prior_clean = self._shutdown_current_transport()
        self._all_sessions_clean = self._all_sessions_clean and prior_clean
        if not prior_clean:
            raise PhysicalNavigationRuntimeError(
                "worker_session_renewal_cleanup_failed",
                "Worker session could not close cleanly before renewal",
            )
        self._raise_if_cancelled("after_worker_shutdown_before_renewal")
        replacement = self.transport_factory()
        if not self._transport_is_valid(replacement):
            raise PhysicalNavigationRuntimeError(
                "invalid_renewed_transport",
                "Transport factory returned an invalid worker transport",
            )
        self.transport = replacement
        if self.active_scan_executor_factory is not None:
            self.active_scan_executor = self.active_scan_executor_factory(
                replacement
            )
        observation, renewed_specs, renewed_roles = (
            self._start_worker_session(
                expected_action_specs=action_specs,
                expected_drive_roles=drive_roles,
            )
        )
        self._emit("worker_session_renewed")
        return observation, renewed_specs, renewed_roles

    def _cleanup(self) -> bool:
        with self._cleanup_lock:
            if self._cleanup_started:
                return False
            self._cleanup_started = True
        current_clean = self._shutdown_current_transport()
        return self._all_sessions_clean and current_clean

    def run(self) -> PhysicalNavigationResult:
        deadline = self.monotonic() + float(
            self.config.max_episode_seconds
        )
        actions = []
        counters = {
            "model_calls": 0,
            "model_latency_ms": 0,
            "tails_completed": 0,
            "tails_cancelled": 0,
        }
        terminal_reason = "turn_budget_exhausted"
        completed = False
        turns = 0
        final_mission = {}
        observation = None
        action_specs = None
        mission = None
        maneuver = ManeuverCommitment()
        local_route = None
        last_tool_result = None
        pending_validation_feedback, reasoning_deferred = None, False
        shutdown_clean = False
        cleanup_error = None
        try:
            observation, action_specs, drive_roles = (
                self._start_worker_session()
            )
            mission = DirectionalMission.begin(
                episode_id=self.episode_id,
                minimum_forward_progress_mm=(
                    self.config.minimum_forward_progress_mm
                ),
                pose=self.memory.pose,
                heading_tolerance_mdeg=(
                    self.config.goal_heading_tolerance_mdeg
                ),
            )
            self._emit(
                "episode_started",
                observation=deepcopy(observation),
                navigation=self.memory.context(),
            )

            for turn in range(1, self.config.max_turns + 1):
                turns = turn
                if self._cancelled():
                    self._raise_if_cancelled("before_navigation_turn")
                if self.monotonic() >= deadline:
                    terminal_reason = "episode_deadline_elapsed"
                    break
                if self._worker_session_needs_renewal(
                    observation,
                    action_specs,
                ):
                    observation, action_specs, drive_roles = (
                        self._renew_worker_session(
                            action_specs=action_specs,
                            drive_roles=drive_roles,
                        )
                    )
                mission_value, navigation = self._goal_state(
                    mission,
                    observation,
                    action_specs,
                )
                final_mission = mission_value
                route_refresh = self._refresh_authorized_local_detour_route(
                    route=local_route,
                    active_maneuver=maneuver.state(turn)["active"],
                    mission=mission,
                    navigation=navigation,
                    action_specs=action_specs,
                )
                local_route = route_refresh.route
                if (
                    pending_validation_feedback is None
                    and local_route is not None
                    and local_route.status == ROUTE_ACTIVE
                ):
                    route_result = (
                        self._execute_authorized_local_detour_route(
                            turn=turn,
                            deadline=deadline,
                            observation=observation,
                            action_specs=action_specs,
                            mission=mission,
                            route=local_route,
                            active_maneuver=(
                                maneuver.state(turn)["active"]
                            ),
                            last_tool_result=last_tool_result,
                        )
                    )
                    observation = route_result.observation
                    local_route = route_result.route
                    last_tool_result = route_result.last_tool_result
                    actions.extend(route_result.actions)
                    if route_result.outcome == EXECUTION_FAILED:
                        terminal_reason = "episode_deadline_elapsed"
                        break
                    # Route execution may have consumed several physical
                    # pulses before fresh geometry, a veto, or another
                    # handoff condition requires planner attention.  Never
                    # resume an active/rebuilt route merely because at least
                    # one pulse completed: rebuild the planner snapshot from
                    # the post-motion observation and let the model validate
                    # the handoff in this turn.
                    (
                        mission_value,
                        navigation,
                        route_refresh,
                    ) = self._post_route_planner_snapshot(
                        route=local_route,
                        active_maneuver=maneuver.state(turn)["active"],
                        mission=mission,
                        observation=observation,
                        action_specs=action_specs,
                    )
                    local_route = route_refresh.route
                    final_mission = mission_value
                repeated_uninformative_observe = (
                    observe_without_information_gain(last_tool_result)
                )
                scan_blocked_targets = self._scan_blocked_target_ids(
                    observation
                )
                scan_eligible_targets = sorted(
                    hypothesis["hypothesis_id"]
                    for hypothesis in navigation[
                        "navigation_hazard_hypotheses"
                    ]
                    if hypothesis["hypothesis_id"]
                    not in scan_blocked_targets
                )
                available = (
                    physical_action_feasibility.prepare_navigation_availability(
                        navigation,
                        active_maneuver=maneuver.state(turn)["active"],
                        scan_eligible_target_ids=scan_eligible_targets,
                        scan_blocked_target_ids=scan_blocked_targets,
                        scan_budget_available=self._scan_budget_allows(
                            observation,
                            action_specs,
                            deadline,
                        ),
                        reverse_budget_available=motion_budget_allows(
                            REVERSE,
                            observation,
                            action_specs,
                        ),
                        action_specs=action_specs,
                        observation=observation,
                        repeated_uninformative_observe=(
                            repeated_uninformative_observe
                        ),
                    )
                )
                available = filter_local_detour_actions(
                    available,
                    route_refresh.guidance,
                )
                if mission_value["completed"] is not True:
                    available = tuple(a for a in available if a != FINISH)
                decision, pending_validation_feedback = self._validated_decision(
                    turn=turn,
                    deadline=deadline,
                    observation=observation,
                    mission=mission_value,
                    navigation=navigation,
                    maneuver=maneuver,
                    available_actions=available,
                    last_tool_result=last_tool_result,
                    counters=counters,
                    validation_feedback=pending_validation_feedback,
                )
                if decision is None:
                    if reasoning_deferred:
                        self._emit(
                            "planner_termination",
                            turn=turn,
                            attempt=self.config.max_validation_attempts,
                            terminal_reason="reasoning_unavailable",
                            validation_feedback=pending_validation_feedback,
                            host_selected_alternative_action=False,
                        )
                        raise _LogicalEpisodeTermination("reasoning_unavailable")
                    reasoning_deferred = True
                    self._emit(
                        "planner_turn_deferred",
                        turn=turn,
                        attempt=self.config.max_validation_attempts,
                        validation_feedback=pending_validation_feedback,
                        host_selected_alternative_action=False,
                    )
                    continue
                reasoning_deferred = False
                self._post_planner_gate(
                    deadline,
                    "before_planner_decision_dispatch",
                )
                if decision.action == FINISH:
                    self._commit_decision(decision, turn=turn)
                    actions.append(FINISH)
                    completed = True
                    terminal_reason = "goal_completed"
                    break
                if decision.action == OBSERVE:
                    self._raise_if_cancelled("immediately_before_observe")
                    self._commit_decision(decision, turn=turn)
                    previous_observation = observation
                    basis_before = self._experience_basis(observation)
                    response = self._active_request(
                        "observe",
                        {},
                        self.config.request_timeout_seconds,
                    )
                    observation = self._observation_from_response(
                        "observe",
                        response,
                    )
                    captured_at_ms = self.unix_ms()
                    try:
                        self.memory.ingest_stationary_observation(
                            observation,
                            captured_at_ms,
                        )
                    except Exception:
                        self._offer_invalid_localization(
                            captured_at_ms=captured_at_ms,
                            publication_stage=(
                                "stationary_observation_invalidated"
                            ),
                            observation=observation,
                        )
                        raise
                    self._offer_observation(
                        observation,
                        captured_at_ms=captured_at_ms,
                        publication_stage="stationary_observation",
                    )
                    actions.append(OBSERVE)
                    information = observation_information_result(
                        previous_observation,
                        observation,
                        motor_roles=(
                            self.memory.drive_roles.left,
                            self.memory.drive_roles.right,
                        ),
                    )
                    last_tool_result = {
                        "operation": "observe",
                        "status": "observed",
                        "worker_response_state_version": response[
                            "state_version"
                        ],
                        **information,
                    }
                    self._emit(
                        "observation_information_assessed",
                        **information,
                    )
                    self._record_experience(
                        turn=turn,
                        action=OBSERVE,
                        source=PLANNER_ACTION_SOURCE,
                        result=last_tool_result,
                        basis_before=basis_before,
                        observation_after=observation,
                    )
                    continue
                if decision.action == SCAN_FRONT_ARC:
                    self._raise_if_cancelled("immediately_before_scan_dispatch")
                    self._commit_decision(decision, turn=turn)
                    basis_before = self._experience_basis(observation)
                    observation, last_tool_result = self._execute_scan(
                        decision,
                        observation=observation,
                        deadline=deadline,
                    )
                    actions.append(SCAN_FRONT_ARC)
                    self._record_experience(
                        turn=turn,
                        action=SCAN_FRONT_ARC,
                        source=PLANNER_ACTION_SOURCE,
                        result=last_tool_result,
                        basis_before=basis_before,
                        observation_after=observation,
                    )
                    continue

                basis_before = self._experience_basis(observation)
                veto = self._execution_veto(
                    action=decision.action,
                    observation=observation,
                    action_specs=action_specs,
                    deadline=deadline,
                )
                if veto is not None:
                    last_tool_result = {
                        "operation": "pulse",
                        "status": "vetoed",
                        "validation": veto,
                    }
                    self._emit(
                        "execution_vetoed",
                        action=decision.action,
                        validation=veto,
                    )
                    self._record_experience(
                        turn=turn,
                        action=decision.action,
                        source=PLANNER_ACTION_SOURCE,
                        result=last_tool_result,
                        basis_before=basis_before,
                        observation_after=observation,
                    )
                    continue
                tail = None
                if route_refresh.guidance.allowed_motion_actions is None:
                    tail = NavigationPlanTail.from_decision(
                        decision,
                        now_monotonic=self.monotonic(),
                        episode_deadline=deadline,
                        map_context=navigation,
                        observation=observation,
                        maneuver_state=maneuver.state(turn),
                        fact_values=navigation["fact_values"],
                        max_age_seconds=self.config.plan_tail_max_age_seconds,
                    )
                self._commit_decision(decision, turn=turn)
                observation, last_tool_result = self._execute_motion(
                    decision.action,
                    action_specs=action_specs,
                )
                actions.append(decision.action)
                self._record_experience(
                    turn=turn,
                    action=decision.action,
                    source=PLANNER_ACTION_SOURCE,
                    result=last_tool_result,
                    basis_before=basis_before,
                    observation_after=observation,
                )
                if last_tool_result["status"] != "completed":
                    if tail is not None:
                        tail.cancel("first_motion_not_completed")
                        counters["tails_cancelled"] += 1
                    continue

                if tail is not None:
                    tail_result = self._execute_navigation_plan_tail(
                        tail=tail,
                        turn=turn,
                        deadline=deadline,
                        observation=observation,
                        last_tool_result=last_tool_result,
                        action_specs=action_specs,
                        mission=mission,
                        maneuver=maneuver,
                    )
                    observation = tail_result.observation
                    last_tool_result = tail_result.last_tool_result
                    actions.extend(tail_result.actions)
                    counters["tails_completed"] += tail_result.completed
                    counters["tails_cancelled"] += tail_result.cancelled

            if mission is not None and observation is not None:
                final_mission, _navigation = self._goal_state(
                    mission,
                    observation,
                    action_specs,
                )
        except _EpisodeCancelled:
            terminal_reason = (
                "emergency_stopped"
                if self.emergency_event.is_set()
                else "cancelled"
            )
        except _LogicalEpisodeTermination as termination:
            terminal_reason = termination.reason
            completed = False
        except (
            NavigationMemoryError,
            PhysicalNavigationContractError,
            PhysicalNavigationRuntimeError,
        ):
            terminal_reason = "navigation_fault"
            raise
        finally:
            primary_error = sys.exc_info()[1]
            try:
                shutdown_clean = self._cleanup() is True
            except Exception as error:
                cleanup_error = error
                shutdown_clean = False
            if not shutdown_clean:
                terminal_reason = "physical_shutdown_unverified"
            self._emit(
                "episode_stopped",
                terminal_reason=terminal_reason,
                shutdown_clean=shutdown_clean,
                model_calls=counters["model_calls"],
                model_latency_ms=counters["model_latency_ms"],
            )
            if not shutdown_clean:
                raise PhysicalNavigationRuntimeError(
                    "physical_shutdown_unverified",
                    "Physical worker shutdown could not be verified",
                    primary_error=primary_error,
                ) from cleanup_error
        final_navigation = dict(self.memory.context())
        if observation is not None:
            final_navigation["experience_ledger"] = (
                self._experience_ledger.context(
                    current_basis=self._experience_basis(observation),
                )
            )
        return PhysicalNavigationResult(
            terminal_reason=terminal_reason,
            completed=completed,
            turns=turns,
            actions=tuple(actions),
            model_calls=counters["model_calls"],
            model_latency_ms=counters["model_latency_ms"],
            plan_tails_completed=counters["tails_completed"],
            plan_tails_cancelled=counters["tails_cancelled"],
            final_mission=deepcopy(final_mission),
            final_navigation=final_navigation,
            shutdown_clean=shutdown_clean,
        )
