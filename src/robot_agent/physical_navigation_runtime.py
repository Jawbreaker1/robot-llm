"""Bounded host runtime for model-planned, worker-executed EV3 navigation."""

from copy import deepcopy
from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Mapping, Optional, Tuple

from .active_ir_scan_contract import (
    ModelScanChoice,
    build_scan_request,
    validate_scan_result,
    worst_case_scan_budget,
)
from .lm_studio_navigation import NavigationPlannerResult
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
from .navigation_plan_tail import NavigationPlanTail
from .physical_navigation_contract import (
    ACTIONS,
    ADVANCE,
    EXPECTED_WORKER_SAFETY,
    EXPECTED_WORKER_OPERATIONS,
    EXPECTED_ACTION_SPECS,
    FINISH,
    MOTION_ACTIONS,
    OBSERVE,
    SCAN_FRONT_ARC,
    NavigationDecision,
    PhysicalNavigationContractError,
    expected_scan_turn_profile,
    expected_scan_sample_profile,
    motion_budget_allows,
    validate_observation,
)
from .physical_navigation_mission import DirectionalMission
from .physical_odometry import DriveMotorRoles


DEFAULT_MAX_TURNS = 14
MAX_TURNS_PER_EPISODE_SECOND = 4
HARD_MAX_TURNS = 14_400
DEFAULT_MAX_EPISODE_SECONDS = 35.0
MIN_EPISODE_SECONDS = 1.0
MAX_EPISODE_SECONDS = 60.0 * 60.0
SUPPORTED_EPISODE_LOCALES = frozenset(("sv", "en"))
HOST_PER_SLICE_HEADROOM_SECONDS = 0.25
HOST_RESPONSE_HEADROOM_SECONDS = 0.75
DEFAULT_SCAN_BUDGET = worst_case_scan_budget()
DEFAULT_SCAN_TIMEOUT_SECONDS = (
    (DEFAULT_SCAN_BUDGET["minimum_deadline_ms"] + 999) // 1000
)


class PhysicalNavigationRuntimeError(RuntimeError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


class _EpisodeCancelled(Exception):
    def __init__(self, stage: str):
        self.stage = stage
        super().__init__(stage)


@dataclass(frozen=True)
class PhysicalNavigationRuntimeConfig:
    goal: str
    locale: str
    minimum_forward_progress_mm: int = 420
    max_turns: Optional[int] = None
    max_episode_seconds: float = DEFAULT_MAX_EPISODE_SECONDS
    startup_timeout_seconds: float = 30.0
    request_timeout_seconds: float = 8.0
    scan_timeout_seconds: float = float(DEFAULT_SCAN_TIMEOUT_SECONDS)
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
            <= 30.0
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


class PhysicalNavigationRuntime:
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
        monotonic: Callable[[], float] = time.monotonic,
        unix_ms: Callable[[], int] = lambda: time.time_ns() // 1_000_000,
        event_sink: Optional[Callable[[Mapping[str, object]], None]] = None,
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
        if not callable(getattr(planner, "decide", None)):
            raise ValueError("navigation planner is invalid")
        if not isinstance(memory, NavigationMemoryStore):
            raise ValueError("navigation memory is invalid")
        if not callable(monotonic) or not callable(unix_ms):
            raise ValueError("runtime clocks are invalid")
        if event_sink is not None and not callable(event_sink):
            raise ValueError("runtime event sink is invalid")
        self.episode_id = episode_id
        self.config = config
        self.transport = transport
        self.transport_factory = transport_factory
        self.planner = planner
        self.memory = memory
        self.active_scan_executor = active_scan_executor
        self.active_scan_executor_factory = active_scan_executor_factory
        self.monotonic = monotonic
        self.unix_ms = unix_ms
        self.event_sink = event_sink
        self.cancel_event = cancel_event or threading.Event()
        self.emergency_event = emergency_event or threading.Event()
        self._stop_requested = threading.Event()
        self._cleanup_lock = threading.Lock()
        self._cleanup_started = False
        self._transport_started = False
        self._observation_received_monotonic = None
        self._worker_absolute_max_ms = None
        self._all_sessions_clean = True

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

    def _active_request(
        self,
        operation: str,
        arguments: Mapping[str, object],
        timeout_seconds: float,
    ) -> Mapping[str, object]:
        self._raise_if_cancelled(
            "immediately_before_{}_request".format(operation)
        )
        try:
            response = self.transport.request(
                operation,
                arguments,
                timeout_seconds,
                cancel_requested=self._cancelled,
            )
        except Exception:
            # The SSH transport closes its channel when the callback turns
            # true. Convert that expected transport failure into the episode's
            # cancellation result, while preserving unrelated failures.
            self._raise_if_cancelled("during_{}_request".format(operation))
            raise
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
            or not 5_000 <= process["absolute_max_ms"] <= 120_000
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
        return mission_value, navigation

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
                or decision.reason_code != "COMPLETE_GOAL"
                or mission["completed"] is not True
            ):
                raise PhysicalNavigationRuntimeError(
                    "premature_mission_finish",
                    "FINISH requires every directional mission fact",
                )
        elif decision.reason_code == "COMPLETE_GOAL":
            raise PhysicalNavigationRuntimeError(
                "nonterminal_complete_reason",
                "COMPLETE_GOAL is valid only with FINISH",
            )
        delta = mission[
            "candidate_action_longitudinal_deltas_mm"
        ].get(action)
        heading_recovery = (
            delta == 0
            and mission[
                "projected_goal_heading_aligned_after_action"
            ].get(action)
            is True
        )
        if decision.reason_code == "PROGRESS_GOAL" and (
            delta is None or delta < 0 or (delta == 0 and not heading_recovery)
        ):
            raise PhysicalNavigationRuntimeError(
                "nonprogress_action_reason",
                "PROGRESS_GOAL contradicts published mission arithmetic",
            )
        if (
            delta is not None
            and delta < 0
            and not navigation["navigation_hazard_hypotheses"]
        ):
            raise PhysicalNavigationRuntimeError(
                "regression_without_hazard",
                "Negative progress requires a published hazard",
            )

    def _remaining_seconds(self, deadline: float) -> float:
        return max(0.0, deadline - self.monotonic())

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
        observation: Mapping[str, object],
        mission: Mapping[str, object],
        navigation: Mapping[str, object],
        maneuver: ManeuverCommitment,
        available_actions,
        last_tool_result,
        counters,
    ) -> NavigationDecision:
        feedback = None
        for attempt in range(1, self.config.max_validation_attempts + 1):
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
            )
            # Planning may take seconds. A stop requested during that call
            # must win before the returned proposal can change state or start
            # any physical operation.
            self._raise_if_cancelled("after_planner_return")
            if isinstance(planner_result, NavigationPlannerResult):
                decision = planner_result.decision
                latency_ms = planner_result.latency_ms
                served_model = planner_result.served_model
            elif isinstance(planner_result, NavigationDecision):
                decision = planner_result
                latency_ms = 0
                served_model = None
            else:
                raise PhysicalNavigationRuntimeError(
                    "invalid_planner_result",
                    "Planner returned the wrong result type",
                )
            counters["model_calls"] += 1
            counters["model_latency_ms"] += latency_ms
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
            )
            try:
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
                    fact_values=navigation["fact_values"],
                    perception_target_hypothesis_id=(
                        decision.perception_target_hypothesis_id
                    ),
                )
                return decision
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
        raise PhysicalNavigationRuntimeError(
            "model_validation_failed",
            "Model did not correct an invalid decision",
        )

    def _execute_motion(
        self,
        action: str,
        *,
        action_specs: Mapping[str, Mapping[str, object]],
    ) -> Tuple[Mapping[str, object], Mapping[str, object]]:
        self._raise_if_cancelled("immediately_before_motion")
        response = self._active_request(
            "pulse",
            {"action": action},
            self.config.request_timeout_seconds,
        )
        observation = self._observation_from_response(
            "pulse",
            response,
            action,
        )
        result = response["result"]
        self.memory.apply_motion_result(
            action,
            result,
            self.unix_ms(),
        )
        outcome = result["outcome"]
        feedback = {
            "operation": "pulse",
            "requested_action": action,
            "status": outcome["status"],
            "reason": outcome["reason"],
            "worker_response_state_version": response["state_version"],
        }
        self._emit(
            "motion_result",
            action=action,
            outcome=deepcopy(outcome),
            navigation=self.memory.context(),
        )
        return observation, feedback

    def _execute_scan(
        self,
        decision: NavigationDecision,
        *,
        observation: Mapping[str, object],
        deadline: float,
    ) -> Tuple[Mapping[str, object], Mapping[str, object]]:
        self._raise_if_cancelled("immediately_before_scan")
        if self.active_scan_executor is None:
            feedback = {
                "operation": SCAN_FRONT_ARC,
                "status": "unavailable",
                "reason": "physical_active_scan_not_configured",
                "host_selected_route_or_side": False,
            }
            self._emit("scan_unavailable", scan=feedback)
            return observation, feedback
        now_ms = self.unix_ms()
        request = build_scan_request(
            choice=ModelScanChoice(
                decision.perception_target_hypothesis_id
            ),
            frame_id=self.memory.frame_id,
            map_generation_id=self.memory.generation_id,
            map_version=self.memory.hazard_map.revision,
            start_pose=self.memory.pose,
            start_state_version=observation["state_version"],
            created_at_ms=now_ms,
            deadline_ms=min(
                now_ms + int(self.config.scan_timeout_seconds * 1000),
                now_ms + int(self._remaining_seconds(deadline) * 1000),
            ),
        )
        try:
            result = self.active_scan_executor.execute(
                request,
                cancel_requested=self._cancelled,
            )
        except Exception:
            self.memory.invalidate_localization(
                "Active scan failed before heading restoration was verified",
                self.unix_ms(),
            )
            raise
        try:
            validate_scan_result(
                result,
                request,
                current_frame_id=self.memory.frame_id,
                current_map_generation_id=self.memory.generation_id,
                current_map_version=self.memory.hazard_map.revision,
            )
        except Exception:
            self.memory.invalidate_localization(
                "Active scan result could not be validated",
                self.unix_ms(),
            )
            raise
        if not result.restored_start_heading or not result.stop_confirmed:
            self.memory.invalidate_localization(
                "Active scan did not verify restoration to its start heading",
                self.unix_ms(),
            )
            self._raise_if_cancelled("scan_cancelled_without_restoration")
            raise PhysicalNavigationRuntimeError(
                "scan_heading_unrestored",
                "Active scan ended without verified heading restoration",
            )
        if self._cancelled():
            self.memory.invalidate_localization(
                "Scan completed but cancellation prevented encoder re-anchoring",
                self.unix_ms(),
            )
            self._raise_if_cancelled("after_scan_before_encoder_reanchor")
        try:
            self._raise_if_cancelled(
                "immediately_before_post_scan_observe"
            )
            response = self._active_request(
                "observe",
                {},
                self.config.request_timeout_seconds,
            )
            fresh = self._observation_from_response("observe", response)
            self.memory.ingest_verified_scan_completion(
                fresh,
                result,
                self.unix_ms(),
            )
        except _EpisodeCancelled:
            self.memory.invalidate_localization(
                "Cancellation interrupted post-scan encoder re-anchoring",
                self.unix_ms(),
            )
            raise
        except Exception:
            if self.memory.localization_valid:
                self.memory.invalidate_localization(
                    "Post-scan encoder re-anchoring failed",
                    self.unix_ms(),
                )
            raise
        if result.bilateral_complete:
            self.memory.hazard_map.record_scan_boundaries(
                result.target_hypothesis_id,
                completed_at_ms=result.completed_at_ms,
                left_boundary_mdeg=result.left_boundary_mdeg,
                right_boundary_mdeg=result.right_boundary_mdeg,
            )
            self.memory.updated_at_ms = max(
                self.memory.updated_at_ms,
                result.completed_at_ms,
            )
            self.memory.save()
        feedback = {
            "operation": SCAN_FRONT_ARC,
            "status": result.status,
            "reason": result.reason,
            "target_hypothesis_id": result.target_hypothesis_id,
            "bilateral_complete": result.bilateral_complete,
            "scan": result.to_dict(),
        }
        self._emit("scan_result", scan=result.to_dict())
        return fresh, feedback

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
        self.memory.begin_episode(observation, self.unix_ms())
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
        except Exception:
            clean = False
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

    def _scan_budget_allows(
        self,
        observation: Mapping[str, object],
        action_specs: Mapping[str, Mapping[str, object]],
        deadline: float,
    ) -> bool:
        budgets = observation["budgets"]
        scan_headroom_ms = self._session_renewal_headroom_ms(
            action_specs,
            include_scan=True,
        )
        return (
            self.active_scan_executor is not None
            and budgets["pulse_count_remaining"]
            >= DEFAULT_SCAN_BUDGET["turn_slice_count"]
            and budgets["pulse_duration_ms_remaining"]
            >= DEFAULT_SCAN_BUDGET["turn_duration_ms"]
            and self._effective_worker_process_ms(observation)
            >= scan_headroom_ms
            and self._worker_absolute_max_ms is not None
            and scan_headroom_ms <= self._worker_absolute_max_ms
            and self._remaining_seconds(deadline) * 1000
            >= scan_headroom_ms
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
            effective_process_ms
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
        last_tool_result = None
        shutdown_clean = False
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
                available = [
                    action
                    for action in sorted(ACTIONS)
                    if (
                        action not in MOTION_ACTIONS
                        or motion_budget_allows(
                            action,
                            observation,
                            action_specs,
                        )
                    )
                    and (
                        action != SCAN_FRONT_ARC
                        or self._scan_budget_allows(
                            observation,
                            action_specs,
                            deadline,
                        )
                    )
                ]
                decision = self._validated_decision(
                    turn=turn,
                    observation=observation,
                    mission=mission_value,
                    navigation=navigation,
                    maneuver=maneuver,
                    available_actions=available,
                    last_tool_result=last_tool_result,
                    counters=counters,
                )
                if decision.action == FINISH:
                    actions.append(FINISH)
                    completed = True
                    terminal_reason = "goal_completed"
                    break
                if decision.action == OBSERVE:
                    self._raise_if_cancelled("immediately_before_observe")
                    response = self._active_request(
                        "observe",
                        {},
                        self.config.request_timeout_seconds,
                    )
                    observation = self._observation_from_response(
                        "observe",
                        response,
                    )
                    self.memory.ingest_stationary_observation(
                        observation,
                        self.unix_ms(),
                    )
                    actions.append(OBSERVE)
                    last_tool_result = {
                        "operation": "observe",
                        "status": "observed",
                        "worker_response_state_version": response[
                            "state_version"
                        ],
                    }
                    continue
                if decision.action == SCAN_FRONT_ARC:
                    self._raise_if_cancelled("immediately_before_scan_dispatch")
                    observation, last_tool_result = self._execute_scan(
                        decision,
                        observation=observation,
                        deadline=deadline,
                    )
                    actions.append(SCAN_FRONT_ARC)
                    continue

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
                    continue
                tail = NavigationPlanTail.from_decision(
                    decision,
                    now_monotonic=self.monotonic(),
                    episode_deadline=deadline,
                    map_context=navigation,
                    observation=observation,
                    maneuver_state=maneuver.state(turn),
                    fact_values=navigation["fact_values"],
                )
                observation, last_tool_result = self._execute_motion(
                    decision.action,
                    action_specs=action_specs,
                )
                actions.append(decision.action)
                if last_tool_result["status"] != "completed":
                    if tail is not None:
                        tail.cancel("first_motion_not_completed")
                        counters["tails_cancelled"] += 1
                    continue

                while tail is not None and not tail.complete:
                    self._raise_if_cancelled(
                        "before_plan_tail_action_validation"
                    )
                    mission_value, navigation = self._goal_state(
                        mission,
                        observation,
                        action_specs,
                    )
                    next_action = tail.next_action(
                        now_monotonic=self.monotonic(),
                        map_context=navigation,
                        observation=observation,
                        maneuver_state=maneuver.state(turn),
                        fact_values=navigation["fact_values"],
                        localization_valid=self.memory.localization_valid,
                    )
                    if next_action is None:
                        counters["tails_cancelled"] += 1
                        self._emit(
                            "plan_tail_cancelled",
                            reason=tail.cancelled_reason,
                            source_plan=list(tail.source_plan),
                        )
                        break
                    veto = self._execution_veto(
                        action=next_action,
                        observation=observation,
                        action_specs=action_specs,
                        deadline=deadline,
                    )
                    if veto is not None:
                        tail.cancel(veto["code"])
                        counters["tails_cancelled"] += 1
                        last_tool_result = {
                            "operation": "pulse",
                            "status": "tail_vetoed",
                            "validation": veto,
                        }
                        self._emit(
                            "plan_tail_cancelled",
                            reason=tail.cancelled_reason,
                            source_plan=list(tail.source_plan),
                        )
                        break
                    self._raise_if_cancelled(
                        "immediately_before_plan_tail_motion"
                    )
                    observation, last_tool_result = self._execute_motion(
                        next_action,
                        action_specs=action_specs,
                    )
                    actions.append(next_action)
                    if last_tool_result["status"] != "completed":
                        tail.cancel("tail_motion_not_completed")
                        counters["tails_cancelled"] += 1
                        break
                    tail.mark_executed(
                        next_action,
                        map_context=self.memory.context(),
                        observation=observation,
                    )
                    if tail.complete:
                        counters["tails_completed"] += 1
                        self._emit(
                            "plan_tail_completed",
                            source_plan=list(tail.source_plan),
                        )

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
        except (
            NavigationMemoryError,
            PhysicalNavigationContractError,
            PhysicalNavigationRuntimeError,
        ):
            terminal_reason = "navigation_fault"
            raise
        finally:
            shutdown_clean = self._cleanup()
            self._emit(
                "episode_stopped",
                terminal_reason=terminal_reason,
                shutdown_clean=shutdown_clean,
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
            final_navigation=self.memory.context(),
            shutdown_clean=shutdown_clean,
        )


class PhysicalNavigationRuntimeAdapter:
    """Factory-backed adapter for ``RobotControlService``'s runner seam."""

    def __init__(
        self,
        *,
        transport_factory: Callable[[], object],
        planner_factory: Callable[[str], object],
        memory_factory: Callable[[], NavigationMemoryStore],
        scan_executor_factory: Optional[Callable[[object], object]] = None,
        minimum_forward_progress_mm: int = 420,
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
        if scan_executor_factory is not None and not callable(
            scan_executor_factory
        ):
            raise ValueError("scan executor factory is invalid")
        self.transport_factory = transport_factory
        self.planner_factory = planner_factory
        self.memory_factory = memory_factory
        self.scan_executor_factory = scan_executor_factory
        self.minimum_forward_progress_mm = minimum_forward_progress_mm
        self.event_mapper = event_mapper or self._dashboard_update
        self._lock = threading.Lock()
        self._active = None

    @staticmethod
    def _dashboard_update(
        event: Mapping[str, object],
    ) -> Optional[Mapping[str, object]]:
        name = event.get("event")
        if name == "model_decision":
            value = {
                "current_action": event["action"],
                "plan": event["plan"],
                "model_latency_ms": event["model_latency_ms"],
                "message": event["assessment"],
            }
            if event.get("utterance"):
                value["message"] = event["utterance"]
            return value
        if name == "motion_result":
            hazards = event["navigation"].get(
                "navigation_hazard_hypotheses",
                [],
            )
            return {
                "current_action": event["action"],
                "obstacle": hazards[-1] if hazards else None,
            }
        if name == "scan_result":
            return {"scan": event["scan"]}
        if name == "episode_stopped":
            return {
                "current_action": None,
                "plan": [],
                "message": event["terminal_reason"],
            }
        return None

    def run(self, context) -> Mapping[str, object]:
        transport = self.transport_factory()
        planner = self.planner_factory(context.settings.model)
        memory = self.memory_factory()
        scan_executor = (
            None
            if self.scan_executor_factory is None
            else self.scan_executor_factory(transport)
        )

        def publish(event):
            update = self.event_mapper(event)
            if update:
                context.publish(update)

        runtime = PhysicalNavigationRuntime(
            episode_id=context.episode_id,
            config=PhysicalNavigationRuntimeConfig(
                goal=context.request.goal,
                locale=context.request.locale,
                minimum_forward_progress_mm=(
                    self.minimum_forward_progress_mm
                ),
                max_episode_seconds=(
                    context.settings.max_episode_ms / 1000.0
                ),
            ),
            transport=transport,
            transport_factory=self.transport_factory,
            planner=planner,
            memory=memory,
            active_scan_executor=scan_executor,
            active_scan_executor_factory=self.scan_executor_factory,
            event_sink=publish,
            cancel_event=context.stop_requested,
            emergency_event=context.emergency_stop_requested,
        )
        with self._lock:
            if self._active is not None:
                raise PhysicalNavigationRuntimeError(
                    "runtime_already_active",
                    "A physical navigation runtime is already active",
                )
            self._active = runtime
        try:
            result = runtime.run()
            return {
                "current_action": None,
                "plan": [],
                "message": result.terminal_reason,
            }
        finally:
            with self._lock:
                if self._active is runtime:
                    self._active = None

    def request_stop(self) -> None:
        with self._lock:
            runtime = self._active
        if runtime is not None:
            runtime.request_stop()

    def emergency_stop(self) -> None:
        with self._lock:
            runtime = self._active
        if runtime is not None:
            runtime.emergency_stop()
