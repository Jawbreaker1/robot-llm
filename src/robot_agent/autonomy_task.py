"""Internal single-lease executor for bounded idle exploration.

This module owns one acquired idle lease from observation through selection,
mission execution and verified release.  The public service remains in
``autonomy_runtime``, which supplies configuration, IDs, duty-cycle
accounting and session orchestration.
"""

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple, Union

from .autonomy_authority import GoalLease
from .autonomy_contract import (
    ROBOT_BASE_FRAME,
    InterestObservation,
    InterestSelectionContext,
    decode_interest_selection,
)
from .autonomy_perception import ResolvedExplorationCandidate
from .autonomy_runtime_contract import (
    IDLE_HELD,
    IDLE_MISSION_FAILED,
    IDLE_MISSION_STALE,
    IDLE_MODEL_ABORTED,
    IDLE_NO_FEASIBLE_CANDIDATES,
    IDLE_PREEMPTED,
    IDLE_SAFETY_STOP,
    IDLE_SELECTION_FAILED,
    IDLE_SELECTION_STALE,
    IDLE_SESSION_BUDGET_EXHAUSTED,
    IDLE_STOP_UNVERIFIED,
    IDLE_TASK_COMPLETED,
    IDLE_UNAVAILABLE,
    IdleTaskResult,
)
from .autonomy_selector import (
    SELECTOR_BUSY,
    SELECTOR_CANCELLED,
    SELECTOR_COMPLETED,
    SELECTOR_DEADLINE_EXPIRED,
)
from .navigation_contract import (
    NavigationContractError,
    WaypointGoal,
)
from .navigation_mission import MissionRunner
from .navigation_mission_contract import (
    MISSION_ABORTED,
    MISSION_BUDGET_EXHAUSTED,
    MISSION_COMPLETED,
    MISSION_PLAN_REJECTED,
    MISSION_PLAN_STALE,
    MISSION_SAFETY_STOP,
    MissionLeg,
    MissionLimits,
    MissionPlan,
    MissionResult,
)
from .navigation_state import NavigationSnapshot


class _DeadlineCancelEvent:
    def __init__(
        self,
        lease: GoalLease,
        clock_ms: Callable[[], int],
        deadline_ms: int,
    ):
        self._lease = lease
        self._clock_ms = clock_ms
        self._deadline_ms = deadline_ms

    def is_set(self) -> bool:
        return (
            self._lease.cancel_event.is_set()
            or self._clock_ms() >= self._deadline_ms
        )


@dataclass
class _TaskState:
    """Mutable bookkeeping scoped to exactly one acquired idle lease."""

    lease: GoalLease
    boundary_goal: WaypointGoal
    planner_calls: int = 0
    stale_replans: int = 0
    trace: List[str] = field(
        default_factory=lambda: ["LEASE_ACQUIRED", "OBSERVING"]
    )
    observation: Optional[InterestObservation] = None
    candidates: Tuple[ResolvedExplorationCandidate, ...] = ()
    selected_candidate_id: Optional[str] = None
    snapshot: Optional[NavigationSnapshot] = None


@dataclass(frozen=True)
class _SelectedTask:
    """A host-resolved candidate with the context that authorized it."""

    candidate: ResolvedExplorationCandidate
    context: InterestSelectionContext


@dataclass(frozen=True)
class _MissionExecution:
    """The typed output needed to classify one finished mission."""

    plan: MissionPlan
    mission: MissionResult
    deadline_ms: int


_RETRY_SELECTION = object()


class _IdleTaskExecutorMixin:
    """Execute exactly one bounded idle lease for the public service."""

    @staticmethod
    def _boundary_goal(lease: GoalLease) -> WaypointGoal:
        return WaypointGoal(
            goal_id="idle-boundary-{}".format(lease.generation),
            goal_epoch=lease.goal_epoch,
            plan_revision=lease.plan_revision,
            target_x_mm=0,
            target_y_mm=0,
            tolerance_mm=1,
        )

    def _verified_stop(
        self,
        goal: WaypointGoal,
        fallback: Optional[NavigationSnapshot] = None,
    ) -> Tuple[Optional[NavigationSnapshot], bool]:
        try:
            current = self.plant.observe(goal)
            stop = self.supervisor.force_stop(
                current,
                reason_code="idle_terminal_stop",
            )
            after = self.plant.apply(stop, goal)
            return (
                after,
                (
                    after.state_version > current.state_version
                    and not after.motors_running
                ),
            )
        except Exception:
            return fallback, False

    def _finish_without_mission(
        self,
        lease: GoalLease,
        boundary_goal: WaypointGoal,
        termination: str,
        planner_calls: int,
        stale_replans: int,
        observation: Optional[InterestObservation],
        candidates: Tuple[ResolvedExplorationCandidate, ...],
        selected_candidate_id: Optional[str],
        trace,
        fallback_snapshot: Optional[NavigationSnapshot] = None,
    ) -> IdleTaskResult:
        final_snapshot, stopped = self._verified_stop(
            boundary_goal,
            fallback_snapshot,
        )
        safe_release = False
        if final_snapshot is not None:
            try:
                safe_release = self.authority.release(
                    lease,
                    final_snapshot,
                    stopped,
                )
            except NavigationContractError:
                safe_release = False
        if safe_release:
            self.range_tracker.seed(final_snapshot)
        else:
            termination = IDLE_STOP_UNVERIFIED
        trace.append(
            "VERIFIED_STOP" if safe_release else "STOP_UNVERIFIED"
        )
        return IdleTaskResult(
            termination=termination,
            lease_generation=lease.generation,
            planner_calls=planner_calls,
            stale_replans=stale_replans,
            selected_candidate_id=selected_candidate_id,
            observation=observation,
            candidates=tuple(value.view for value in candidates),
            mission=None,
            final_snapshot=final_snapshot,
            terminal_stop_verified=safe_release,
            trace=tuple(trace),
        )

    def _remaining_mission_limits(
        self,
        remaining_actions: int,
        remaining_motion_ms: int,
        remaining_elapsed_ms: int,
    ) -> MissionLimits:
        return MissionLimits(
            max_legs=1,
            max_ticks=self.mission_limits.max_ticks,
            max_elapsed_ms=min(
                self.mission_limits.max_elapsed_ms,
                max(1, remaining_elapsed_ms),
            ),
            max_proposals=self.mission_limits.max_proposals,
            max_replans=self.mission_limits.max_replans,
            max_actions=min(
                self.mission_limits.max_actions,
                remaining_actions,
            ),
            max_total_motion_ms=min(
                self.mission_limits.max_total_motion_ms,
                remaining_motion_ms,
            ),
        )

    def _fresh_selection_snapshot(
        self,
        boundary_goal: WaypointGoal,
        context: InterestSelectionContext,
    ) -> Optional[NavigationSnapshot]:
        if self._now_ms() >= context.valid_until_ms:
            return None
        try:
            snapshot = self.plant.observe(boundary_goal)
        except Exception:
            return None
        if (
            snapshot.state_version != context.state_version
            or snapshot.world_model_version
            != context.world_model_version
            or snapshot.robot_id != context.robot_id
            or snapshot.controller_instance_id
            != context.controller_instance_id
            or snapshot.motors_running
            or snapshot.touch_pressed
            or snapshot.active_faults
        ):
            return None
        return snapshot

    def _finish_state(
        self,
        state: _TaskState,
        termination: str,
    ) -> IdleTaskResult:
        return self._finish_without_mission(
            state.lease,
            state.boundary_goal,
            termination,
            state.planner_calls,
            state.stale_replans,
            state.observation,
            state.candidates,
            state.selected_candidate_id,
            state.trace,
            state.snapshot,
        )

    def _retry_stale_selection(
        self,
        state: _TaskState,
        remaining_stale_replans: int,
    ) -> bool:
        allowed = min(
            self.max_stale_replans_per_task,
            remaining_stale_replans,
        )
        if state.stale_replans >= allowed:
            return False
        state.stale_replans += 1
        return True

    def _build_selection_context(
        self,
        state: _TaskState,
        remaining_tasks: int,
        session_deadline_ms: int,
    ) -> Union[InterestSelectionContext, IdleTaskResult]:
        try:
            state.snapshot = self.plant.observe(state.boundary_goal)
            if (
                state.snapshot.motors_running
                or state.snapshot.touch_pressed
                or state.snapshot.active_faults
            ):
                raise NavigationContractError(
                    "unsafe_idle_activation",
                    "Idle activation snapshot is unsafe",
                )
            captured_at_ms = self._now_ms()
            valid_until_ms = min(
                captured_at_ms + self.selection_ttl_ms,
                session_deadline_ms,
            )
            if valid_until_ms <= captured_at_ms:
                state.trace.append("SELECTION_DEADLINE_EXHAUSTED")
                return self._finish_state(
                    state,
                    IDLE_SELECTION_STALE,
                )
            state.observation = self.range_tracker.capture(
                state.snapshot,
                self._new_id("idle-observation"),
                captured_at_ms,
                valid_until_ms,
            )
            candidate_set_id = self._new_id("candidate-set")
            state.candidates = self.candidate_generator.generate(
                state.snapshot,
                candidate_set_id,
                state.observation,
            )
        except Exception:
            state.trace.append("OBSERVATION_FAILED")
            return self._finish_state(state, IDLE_SAFETY_STOP)

        if not state.candidates:
            state.trace.append("NO_FEASIBLE_CANDIDATES")
            return self._finish_state(
                state,
                IDLE_NO_FEASIBLE_CANDIDATES,
            )
        return InterestSelectionContext(
            proposal_id=self._new_id("interest-proposal"),
            robot_id=state.snapshot.robot_id,
            controller_instance_id=(
                state.snapshot.controller_instance_id
            ),
            autonomy_session_id=self.session_id,
            lease_generation=state.lease.generation,
            candidate_set_id=candidate_set_id,
            frame_id=ROBOT_BASE_FRAME,
            state_version=state.snapshot.state_version,
            world_model_version=state.snapshot.world_model_version,
            captured_at_ms=captured_at_ms,
            valid_until_ms=valid_until_ms,
            remaining_tasks=remaining_tasks,
            observations=(
                ()
                if state.observation is None
                else (state.observation,)
            ),
            candidates=tuple(
                candidate.view for candidate in state.candidates
            ),
        )

    def _request_selection(
        self,
        state: _TaskState,
        context: InterestSelectionContext,
        remaining_stale_replans: int,
    ):
        state.trace.append("SELECTING")
        state.planner_calls += 1
        outcome = self.selector_gate.call(
            context,
            state.lease.cancel_event,
            context.valid_until_ms,
        )
        if outcome.status == SELECTOR_CANCELLED:
            state.trace.append("SELECTION_PREEMPTED")
            return self._finish_state(state, IDLE_PREEMPTED)
        if outcome.status == SELECTOR_DEADLINE_EXPIRED:
            state.trace.append("SELECTION_DEADLINE_EXPIRED")
            if self._retry_stale_selection(
                state,
                remaining_stale_replans,
            ):
                return _RETRY_SELECTION
            return self._finish_state(state, IDLE_SELECTION_STALE)
        if outcome.status == SELECTOR_BUSY:
            state.trace.append("SELECTOR_BUSY")
            return self._finish_state(state, IDLE_UNAVAILABLE)
        if outcome.status != SELECTOR_COMPLETED:
            state.trace.append("SELECTION_FAILED")
            return self._finish_state(state, IDLE_SELECTION_FAILED)

        try:
            selection = decode_interest_selection(outcome.payload)
            context.assert_accepts(selection, self._now_ms())
        except NavigationContractError as error:
            if error.code == "expired_interest_selection":
                state.trace.append("SELECTION_DEADLINE_EXPIRED")
                if self._retry_stale_selection(
                    state,
                    remaining_stale_replans,
                ):
                    return _RETRY_SELECTION
                return self._finish_state(
                    state,
                    IDLE_SELECTION_STALE,
                )
            state.trace.append("SELECTION_FAILED")
            return self._finish_state(state, IDLE_SELECTION_FAILED)
        except Exception:
            state.trace.append("SELECTION_FAILED")
            return self._finish_state(state, IDLE_SELECTION_FAILED)
        return selection

    def _select_task(
        self,
        state: _TaskState,
        remaining_tasks: int,
        remaining_planner_calls: int,
        remaining_stale_replans: int,
        remaining_actions: int,
        remaining_motion_ms: int,
        session_deadline_ms: int,
    ) -> Union[_SelectedTask, IdleTaskResult]:
        while True:
            if not self.authority.is_current_idle(state.lease):
                state.trace.append("PREEMPTED_BEFORE_SELECTION")
                return self._finish_state(state, IDLE_PREEMPTED)
            now_ms = self._now_ms()
            if (
                now_ms >= session_deadline_ms
                or state.planner_calls >= remaining_planner_calls
            ):
                state.trace.append("SELECTION_BUDGET_EXHAUSTED")
                return self._finish_state(
                    state,
                    IDLE_SELECTION_STALE,
                )

            context = self._build_selection_context(
                state,
                remaining_tasks,
                session_deadline_ms,
            )
            if isinstance(context, IdleTaskResult):
                return context
            selection = self._request_selection(
                state,
                context,
                remaining_stale_replans,
            )
            if selection is _RETRY_SELECTION:
                continue
            if isinstance(selection, IdleTaskResult):
                return selection

            if not self.authority.is_current_idle(state.lease):
                state.trace.append("LATE_SELECTION_DROPPED")
                return self._finish_state(state, IDLE_PREEMPTED)
            fresh = self._fresh_selection_snapshot(
                state.boundary_goal,
                context,
            )
            if fresh is None:
                state.trace.append("STALE_SELECTION_DROPPED")
                if self._retry_stale_selection(
                    state,
                    remaining_stale_replans,
                ):
                    continue
                return self._finish_state(
                    state,
                    IDLE_SELECTION_STALE,
                )
            state.snapshot = fresh

            if selection.decision == "HOLD":
                state.trace.append("MODEL_HOLD")
                return self._finish_state(state, IDLE_HELD)
            if selection.decision == "ABORT":
                state.trace.append("MODEL_ABORT")
                return self._finish_state(state, IDLE_MODEL_ABORTED)

            state.selected_candidate_id = (
                selection.selected_candidate_id
            )
            try:
                selected = self.candidate_generator.resolve(
                    state.candidates,
                    state.selected_candidate_id,
                )
            except NavigationContractError:
                state.trace.append("CANDIDATE_RESOLUTION_FAILED")
                return self._finish_state(
                    state,
                    IDLE_SELECTION_FAILED,
                )
            if (
                remaining_actions <= 0
                or remaining_motion_ms <= 0
                or self._now_ms() >= session_deadline_ms
            ):
                state.trace.append("MISSION_BUDGET_EXHAUSTED")
                return self._finish_state(
                    state,
                    IDLE_SELECTION_STALE,
                )

            self.memory.record_attempt(selected.memory_key)
            if not self.authority.is_current_idle(state.lease):
                state.trace.append("PREEMPTED_BEFORE_MISSION")
                return self._finish_state(state, IDLE_PREEMPTED)
            fresh = self._fresh_selection_snapshot(
                state.boundary_goal,
                context,
            )
            if fresh is None:
                state.trace.append("STALE_BEFORE_MISSION_DROPPED")
                if self._retry_stale_selection(
                    state,
                    remaining_stale_replans,
                ):
                    continue
                return self._finish_state(
                    state,
                    IDLE_SELECTION_STALE,
                )
            state.snapshot = fresh
            return _SelectedTask(
                candidate=selected,
                context=context,
            )

    def _dispatch_mission(
        self,
        state: _TaskState,
        selected: _SelectedTask,
        remaining_actions: int,
        remaining_motion_ms: int,
        session_deadline_ms: int,
    ) -> Union[_MissionExecution, IdleTaskResult]:
        if (
            remaining_actions <= 0
            or remaining_motion_ms <= 0
            or self._now_ms() >= session_deadline_ms
            or self._now_ms() >= selected.context.valid_until_ms
        ):
            state.trace.append("MISSION_BUDGET_EXHAUSTED")
            return self._finish_state(state, IDLE_SELECTION_STALE)

        plan = MissionPlan(
            plan_id=self._new_id("idle-plan"),
            robot_id=state.snapshot.robot_id,
            controller_instance_id=(
                state.snapshot.controller_instance_id
            ),
            based_on_state_version=state.snapshot.state_version,
            based_on_world_model_version=(
                state.snapshot.world_model_version
            ),
            plan_revision=state.lease.plan_revision,
            legs=(MissionLeg(
                leg_id=selected.candidate.view.candidate_id,
                target_x_mm=selected.candidate.target_x_mm,
                target_y_mm=selected.candidate.target_y_mm,
                tolerance_mm=selected.candidate.tolerance_mm,
            ),),
        )
        mission_deadline_ms = min(
            session_deadline_ms,
            selected.context.valid_until_ms,
        )
        try:
            self.inbox.drain()
            runner = MissionRunner(
                self.plant,
                self.supervisor,
                self.inbox,
                starting_goal_epoch=state.lease.goal_epoch,
                per_leg_limits=self.per_leg_limits,
                mission_limits=self._remaining_mission_limits(
                    remaining_actions,
                    remaining_motion_ms,
                    max(
                        1,
                        mission_deadline_ms - self._now_ms(),
                    ),
                ),
                cancel_event=_DeadlineCancelEvent(
                    state.lease,
                    self._now_ms,
                    mission_deadline_ms,
                ),
            )
            state.trace.append("MISSION_STARTED")
            mission = runner.run(plan)
            state.trace.append("MISSION_FINISHED")
        except Exception:
            state.trace.append("MISSION_EXCEPTION")
            return self._finish_state(state, IDLE_MISSION_FAILED)
        return _MissionExecution(
            plan=plan,
            mission=mission,
            deadline_ms=mission_deadline_ms,
        )

    def _classify_mission(
        self,
        state: _TaskState,
        selected: _SelectedTask,
        execution: _MissionExecution,
    ) -> IdleTaskResult:
        mission = execution.mission
        plan = execution.plan
        safe_release = False
        try:
            safe_release = self.authority.release(
                state.lease,
                mission.final_snapshot,
                mission.terminal_stop_verified,
            )
        except NavigationContractError:
            safe_release = False

        if not safe_release:
            termination = IDLE_STOP_UNVERIFIED
            state.trace.append("STOP_UNVERIFIED")
        elif mission.completed and mission.termination == MISSION_COMPLETED:
            self.memory.record_completed(
                selected.candidate.memory_key
            )
            self.range_tracker.seed(mission.final_snapshot)
            termination = IDLE_TASK_COMPLETED
            state.trace.append("TASK_COMPLETED")
        elif (
            mission.termination == MISSION_ABORTED
            and state.lease.cancel_event.is_set()
        ):
            termination = IDLE_PREEMPTED
            state.trace.append("TASK_PREEMPTED")
        elif (
            mission.termination == MISSION_ABORTED
            and self._now_ms() >= execution.deadline_ms
        ):
            termination = IDLE_MISSION_STALE
            state.trace.append("MISSION_SELECTION_DEADLINE")
        elif (
            mission.termination == MISSION_PLAN_STALE
            or (
                mission.termination == MISSION_PLAN_REJECTED
                and mission.terminal_stop_verified
                and mission.final_snapshot.robot_id == plan.robot_id
                and mission.final_snapshot.controller_instance_id
                == plan.controller_instance_id
                and not mission.final_snapshot.motors_running
                and not mission.final_snapshot.touch_pressed
                and not mission.final_snapshot.active_faults
                and (
                    mission.final_snapshot.state_version
                    != plan.based_on_state_version
                    or mission.final_snapshot.world_model_version
                    != plan.based_on_world_model_version
                )
            )
        ):
            termination = IDLE_MISSION_STALE
            state.trace.append("MISSION_STALE")
        elif mission.termination == MISSION_SAFETY_STOP:
            termination = IDLE_SAFETY_STOP
            state.trace.append("SAFETY_STOP")
        elif mission.termination == MISSION_BUDGET_EXHAUSTED:
            termination = IDLE_SESSION_BUDGET_EXHAUSTED
            state.trace.append("MISSION_BUDGET_EXHAUSTED")
        else:
            termination = IDLE_MISSION_FAILED
            state.trace.append("MISSION_FAILED")
        return IdleTaskResult(
            termination=termination,
            lease_generation=state.lease.generation,
            planner_calls=state.planner_calls,
            stale_replans=state.stale_replans,
            selected_candidate_id=state.selected_candidate_id,
            observation=state.observation,
            candidates=tuple(
                value.view for value in state.candidates
            ),
            mission=mission,
            final_snapshot=mission.final_snapshot,
            terminal_stop_verified=safe_release,
            trace=tuple(state.trace),
        )

    def _run_task(
        self,
        remaining_tasks: int,
        remaining_planner_calls: int,
        remaining_stale_replans: int,
        remaining_actions: int,
        remaining_motion_ms: int,
        session_deadline_ms: int,
    ) -> IdleTaskResult:
        lease = self.authority.try_acquire_idle()
        if lease is None:
            return IdleTaskResult(
                termination=IDLE_UNAVAILABLE,
                lease_generation=None,
                planner_calls=0,
                stale_replans=0,
                selected_candidate_id=None,
                observation=None,
                candidates=(),
                mission=None,
                final_snapshot=None,
                terminal_stop_verified=False,
                trace=("IDLE_UNAVAILABLE",),
            )
        state = _TaskState(
            lease=lease,
            boundary_goal=self._boundary_goal(lease),
        )
        selected = self._select_task(
            state,
            remaining_tasks,
            remaining_planner_calls,
            remaining_stale_replans,
            remaining_actions,
            remaining_motion_ms,
            session_deadline_ms,
        )
        if isinstance(selected, IdleTaskResult):
            return selected
        execution = self._dispatch_mission(
            state,
            selected,
            remaining_actions,
            remaining_motion_ms,
            session_deadline_ms,
        )
        if isinstance(execution, IdleTaskResult):
            return execution
        return self._classify_mission(
            state,
            selected,
            execution,
        )
