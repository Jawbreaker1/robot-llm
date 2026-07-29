"""Bounded simulator-first idle exploration above navigation authority.

The service wakes only when invited, acquires an exclusive idle goal lease,
offers a local model a host-created candidate menu, revalidates the result and
executes at most one waypoint per lease.  It never grants the model raw
coordinates or direct access to the motion supervisor.
"""

import itertools
import threading
import time
from typing import Callable, Optional

from .autonomy_authority import GoalLeaseCoordinator
from .autonomy_perception import (
    ExplorationMemory,
    ExplorationPolicy,
    RangeObservationTracker,
    SimulatorCandidateGenerator,
)
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
    IdleDutyCycleLimits,
    IdleDutyCycleState,
    IdleSessionLimits,
    IdleSessionResult,
    IdleTaskResult,
)
from .autonomy_selector import SingleFlightSelector
from .autonomy_task import _IdleTaskExecutorMixin
from .navigation_contract import (
    NavigationContractError,
    WaypointGoal,
    identifier,
    integer,
)
from .navigation_episode import NavigationLimits
from .navigation_mission_contract import MissionLimits
from .navigation_simulator import DifferentialDriveSimulator
from .navigation_state import NavigationSnapshot, ProposalInbox
from .navigation_supervisor import MotionSupervisor


def _monotonic_ms() -> int:
    return int(time.monotonic() * 1_000)


class IdleExplorationService(_IdleTaskExecutorMixin):
    """Wake-driven, single-owner idle autonomy for the 2D simulator."""

    def __init__(
        self,
        plant: DifferentialDriveSimulator,
        supervisor: MotionSupervisor,
        inbox: ProposalInbox,
        authority: GoalLeaseCoordinator,
        selector,
        session_id: str = "idle-autonomy-session",
        exploration_policy: ExplorationPolicy = ExplorationPolicy(),
        per_leg_limits: NavigationLimits = NavigationLimits(),
        mission_limits: MissionLimits = MissionLimits(max_legs=1),
        selection_ttl_ms: int = 8_000,
        max_stale_replans_per_task: int = 2,
        clock_ms: Callable[[], int] = _monotonic_ms,
        id_factory: Optional[Callable[[], str]] = None,
        memory: Optional[ExplorationMemory] = None,
        range_tracker: Optional[RangeObservationTracker] = None,
        duty_cycle_limits: IdleDutyCycleLimits = IdleDutyCycleLimits(),
        observation_sink: Optional[
            Callable[[NavigationSnapshot], None]
        ] = None,
    ):
        if not isinstance(plant, DifferentialDriveSimulator):
            raise NavigationContractError(
                "invalid_idle_plant",
                "Idle exploration is simulator-only",
            )
        if not isinstance(supervisor, MotionSupervisor):
            raise NavigationContractError(
                "invalid_idle_supervisor",
                "Idle exploration requires MotionSupervisor",
            )
        if not isinstance(inbox, ProposalInbox):
            raise NavigationContractError(
                "invalid_idle_inbox",
                "Idle exploration requires ProposalInbox",
            )
        if not isinstance(authority, GoalLeaseCoordinator):
            raise NavigationContractError(
                "invalid_goal_authority",
                "Idle exploration requires GoalLeaseCoordinator",
            )
        if (
            authority.robot_id != plant.robot_id
            or authority.controller_instance_id
            != plant.controller_instance_id
        ):
            raise NavigationContractError(
                "idle_authority_identity_mismatch",
                "Idle authority belongs to another controller",
            )
        if not callable(selector):
            raise NavigationContractError(
                "invalid_interest_selector",
                "Idle exploration requires a callable selector",
            )
        identifier("session_id", session_id)
        if not isinstance(exploration_policy, ExplorationPolicy):
            raise NavigationContractError(
                "invalid_exploration_policy",
                "Exploration policy is invalid",
            )
        if not isinstance(per_leg_limits, NavigationLimits):
            raise NavigationContractError(
                "invalid_navigation_limits",
                "Per-leg limits are invalid",
            )
        if not isinstance(mission_limits, MissionLimits):
            raise NavigationContractError(
                "invalid_mission_limits",
                "Mission limits are invalid",
            )
        if mission_limits.max_legs != 1:
            raise NavigationContractError(
                "idle_multi_leg_forbidden",
                "Each idle lease may execute exactly one waypoint",
            )
        integer("selection_ttl_ms", selection_ttl_ms, 100, 60_000)
        integer(
            "max_stale_replans_per_task",
            max_stale_replans_per_task,
            0,
            100,
        )
        if not callable(clock_ms) or (
            id_factory is not None and not callable(id_factory)
        ) or (
            observation_sink is not None
            and not callable(observation_sink)
        ):
            raise NavigationContractError(
                "invalid_idle_dependency",
                "Idle clock, ID factory, or observation sink is invalid",
            )
        if memory is None:
            memory = ExplorationMemory(
                exploration_policy.visit_grid_mm
            )
        if range_tracker is None:
            range_tracker = RangeObservationTracker(
                exploration_policy.range_history_capacity
            )
        if not isinstance(memory, ExplorationMemory):
            raise NavigationContractError(
                "invalid_exploration_memory",
                "Idle exploration memory is invalid",
            )
        if not isinstance(range_tracker, RangeObservationTracker):
            raise NavigationContractError(
                "invalid_range_tracker",
                "Idle range tracker is invalid",
            )
        if not isinstance(duty_cycle_limits, IdleDutyCycleLimits):
            raise NavigationContractError(
                "invalid_idle_duty_cycle_limits",
                "Idle duty-cycle limits are invalid",
            )
        self.plant = plant
        self.supervisor = supervisor
        self.inbox = inbox
        self.authority = authority
        self.selector = selector
        self.selector_gate = SingleFlightSelector(
            selector,
            clock_ms=clock_ms,
        )
        self.session_id = session_id
        self.exploration_policy = exploration_policy
        self.per_leg_limits = per_leg_limits
        self.mission_limits = mission_limits
        self.selection_ttl_ms = selection_ttl_ms
        self.max_stale_replans_per_task = max_stale_replans_per_task
        self.clock_ms = clock_ms
        self.memory = memory
        self.range_tracker = range_tracker
        self.duty_cycle_limits = duty_cycle_limits
        self.observation_sink = observation_sink
        self.candidate_generator = SimulatorCandidateGenerator(
            plant,
            memory,
            exploration_policy,
        )
        self._run_lock = threading.Lock()
        self._id_lock = threading.Lock()
        self._id_counter = itertools.count(1)
        self._external_id_factory = id_factory
        self._duty_generation = 1
        self._duty_started_at_ms = None
        self._duty_task_attempts = 0
        self._duty_planner_calls = 0
        self._duty_stale_replans = 0
        self._duty_actions = 0
        self._duty_total_motion_ms = 0

    def _now_ms(self) -> int:
        return integer(
            "idle_clock_ms",
            self.clock_ms(),
            0,
            2**63 - 1,
        )

    def _new_id(self, prefix: str) -> str:
        with self._id_lock:
            sequence = next(self._id_counter)
        suffix = str(sequence)
        if self._external_id_factory is not None:
            external = self._external_id_factory()
            identifier("idle_id_suffix", external, 48)
            suffix = "{}-{}".format(sequence, external)
        return identifier(
            "idle_generated_id",
            "{}-{}".format(prefix, suffix),
            96,
        )

    def _duty_exhausted_unlocked(self, now_ms: int) -> bool:
        limits = self.duty_cycle_limits
        elapsed_exhausted = (
            self._duty_started_at_ms is not None
            and now_ms - self._duty_started_at_ms
            >= limits.max_elapsed_ms
        )
        return (
            elapsed_exhausted
            or self._duty_task_attempts
            >= limits.max_task_attempts
            or self._duty_planner_calls >= limits.max_planner_calls
            or self._duty_actions >= limits.max_actions
            or self._duty_total_motion_ms
            >= limits.max_total_motion_ms
        )

    def _duty_state_unlocked(self, now_ms: int) -> IdleDutyCycleState:
        return IdleDutyCycleState(
            generation=self._duty_generation,
            started_at_ms=self._duty_started_at_ms,
            task_attempts=self._duty_task_attempts,
            planner_calls=self._duty_planner_calls,
            stale_replans=self._duty_stale_replans,
            actions=self._duty_actions,
            total_motion_ms=self._duty_total_motion_ms,
            exhausted=self._duty_exhausted_unlocked(now_ms),
        )

    @property
    def duty_cycle_state(self) -> IdleDutyCycleState:
        """Return persistent accounting across scheduler invocations."""

        with self._run_lock:
            return self._duty_state_unlocked(self._now_ms())

    def rearm_idle_duty_cycle(self) -> int:
        """Reset persistent budgets only while idle is disabled and stopped."""

        with self._run_lock:
            if self.selector_gate.busy:
                raise NavigationContractError(
                    "unsafe_idle_duty_rearm",
                    "Wait for the outstanding selector before re-arming",
                )
            guard = self.authority.begin_idle_duty_rearm()
            try:
                authority_state = self.authority.state
                probe = WaypointGoal(
                    goal_id="idle-duty-rearm-probe",
                    goal_epoch=max(
                        1,
                        authority_state.last_allocated_goal_epoch,
                    ),
                    plan_revision=max(
                        1,
                        authority_state.last_allocated_plan_revision,
                    ),
                    target_x_mm=0,
                    target_y_mm=0,
                    tolerance_mm=1,
                )
                snapshot = self.plant.observe(probe)
                if (
                    snapshot.motors_running
                    or snapshot.touch_pressed
                    or snapshot.active_faults
                ):
                    raise NavigationContractError(
                        "unsafe_idle_duty_rearm",
                        "Idle duty cycle can only re-arm from a safe stop",
                    )
                self._duty_generation += 1
                self._duty_started_at_ms = None
                self._duty_task_attempts = 0
                self._duty_planner_calls = 0
                self._duty_stale_replans = 0
                self._duty_actions = 0
                self._duty_total_motion_ms = 0
                return self._duty_generation
            finally:
                self.authority.finish_idle_duty_rearm(guard)

    def _duty_remaining_unlocked(self, now_ms: int):
        limits = self.duty_cycle_limits
        if self._duty_started_at_ms is None:
            deadline_ms = now_ms + limits.max_elapsed_ms
        else:
            deadline_ms = (
                self._duty_started_at_ms + limits.max_elapsed_ms
            )
        return {
            "tasks": max(
                0,
                limits.max_task_attempts - self._duty_task_attempts,
            ),
            "planner_calls": max(
                0,
                limits.max_planner_calls - self._duty_planner_calls,
            ),
            "stale_replans": max(
                0,
                limits.max_stale_replans - self._duty_stale_replans,
            ),
            "actions": max(
                0,
                limits.max_actions - self._duty_actions,
            ),
            "motion_ms": max(
                0,
                limits.max_total_motion_ms
                - self._duty_total_motion_ms,
            ),
            "deadline_ms": deadline_ms,
        }

    def _account_duty_task_unlocked(
        self,
        task: IdleTaskResult,
    ) -> None:
        if task.lease_generation is None:
            return
        if self._duty_started_at_ms is None:
            self._duty_started_at_ms = self._now_ms()
        self._duty_task_attempts += 1
        self._duty_planner_calls += task.planner_calls
        self._duty_stale_replans += task.stale_replans
        if task.mission is not None:
            self._duty_actions += task.mission.actions
            self._duty_total_motion_ms += task.mission.total_motion_ms

    def _account_duty_replan_unlocked(self) -> None:
        self._duty_stale_replans += 1

    @staticmethod
    def _empty_task(
        termination: str,
        trace_code: str,
    ) -> IdleTaskResult:
        return IdleTaskResult(
            termination=termination,
            lease_generation=None,
            planner_calls=0,
            stale_replans=0,
            selected_candidate_id=None,
            observation=None,
            candidates=(),
            mission=None,
            final_snapshot=None,
            terminal_stop_verified=False,
            trace=(trace_code,),
        )

    def run_once(
        self,
        limits: IdleSessionLimits = IdleSessionLimits(max_tasks=1),
    ) -> IdleTaskResult:
        """Attempt one bounded idle task; never loop indefinitely."""

        if not isinstance(limits, IdleSessionLimits):
            raise NavigationContractError(
                "invalid_idle_limits",
                "Idle limits are invalid",
            )
        with self._run_lock:
            started = self._now_ms()
            if self._duty_exhausted_unlocked(started):
                return self._empty_task(
                    IDLE_SESSION_BUDGET_EXHAUSTED,
                    "DUTY_CYCLE_BUDGET_EXHAUSTED",
                )
            duty = self._duty_remaining_unlocked(started)
            if (
                duty["tasks"] <= 0
                or duty["planner_calls"] <= 0
                or duty["actions"] <= 0
                or duty["motion_ms"] <= 0
            ):
                return self._empty_task(
                    IDLE_SESSION_BUDGET_EXHAUSTED,
                    "DUTY_CYCLE_BUDGET_EXHAUSTED",
                )
            task = self._run_task(
                remaining_tasks=1,
                remaining_planner_calls=min(
                    limits.max_planner_calls,
                    duty["planner_calls"],
                ),
                remaining_stale_replans=min(
                    limits.max_stale_replans,
                    duty["stale_replans"],
                ),
                remaining_actions=min(
                    limits.max_actions,
                    duty["actions"],
                ),
                remaining_motion_ms=min(
                    limits.max_total_motion_ms,
                    duty["motion_ms"],
                ),
                session_deadline_ms=min(
                    started + limits.max_elapsed_ms,
                    duty["deadline_ms"],
                ),
            )
            if (
                task.lease_generation is not None
                and self._duty_started_at_ms is None
            ):
                self._duty_started_at_ms = started
            self._account_duty_task_unlocked(task)
            return task

    def run_session(
        self,
        limits: IdleSessionLimits = IdleSessionLimits(),
    ) -> IdleSessionResult:
        """Run several one-leg tasks under one cumulative autonomy budget."""

        if not isinstance(limits, IdleSessionLimits):
            raise NavigationContractError(
                "invalid_idle_limits",
                "Idle limits are invalid",
            )
        with self._run_lock:
            started_at_ms = self._now_ms()
            session_deadline_ms = (
                started_at_ms + limits.max_elapsed_ms
            )
            tasks = []
            planner_calls = 0
            stale_replans = 0
            actions = 0
            total_motion_ms = 0
            termination = IDLE_SESSION_BUDGET_EXHAUSTED

            while len(tasks) < limits.max_tasks:
                now_ms = self._now_ms()
                if self._duty_exhausted_unlocked(now_ms):
                    termination = IDLE_SESSION_BUDGET_EXHAUSTED
                    break
                duty = self._duty_remaining_unlocked(now_ms)
                if (
                    now_ms >= session_deadline_ms
                    or planner_calls >= limits.max_planner_calls
                    or actions >= limits.max_actions
                    or total_motion_ms >= limits.max_total_motion_ms
                    or duty["tasks"] <= 0
                    or duty["planner_calls"] <= 0
                    or duty["actions"] <= 0
                    or duty["motion_ms"] <= 0
                ):
                    termination = IDLE_SESSION_BUDGET_EXHAUSTED
                    break
                task = self._run_task(
                    remaining_tasks=limits.max_tasks - len(tasks),
                    remaining_planner_calls=min(
                        limits.max_planner_calls - planner_calls,
                        duty["planner_calls"],
                    ),
                    remaining_stale_replans=min(
                        max(
                            0,
                            limits.max_stale_replans
                            - stale_replans,
                        ),
                        duty["stale_replans"],
                    ),
                    remaining_actions=min(
                        limits.max_actions - actions,
                        duty["actions"],
                    ),
                    remaining_motion_ms=min(
                        limits.max_total_motion_ms
                        - total_motion_ms,
                        duty["motion_ms"],
                    ),
                    session_deadline_ms=min(
                        session_deadline_ms,
                        duty["deadline_ms"],
                    ),
                )
                if (
                    task.lease_generation is not None
                    and self._duty_started_at_ms is None
                ):
                    self._duty_started_at_ms = now_ms
                self._account_duty_task_unlocked(task)
                tasks.append(task)
                planner_calls += task.planner_calls
                stale_replans += task.stale_replans
                if task.mission is not None:
                    actions += task.mission.actions
                    total_motion_ms += task.mission.total_motion_ms
                termination = task.termination
                if task.completed:
                    continue
                if task.termination == IDLE_MISSION_STALE:
                    duty_after = self._duty_remaining_unlocked(
                        self._now_ms()
                    )
                    if (
                        stale_replans < limits.max_stale_replans
                        and duty_after["stale_replans"] > 0
                    ):
                        stale_replans += 1
                        self._account_duty_replan_unlocked()
                        continue
                break
            else:
                termination = IDLE_SESSION_BUDGET_EXHAUSTED

            elapsed_ms = max(0, self._now_ms() - started_at_ms)
            final_snapshot = (
                None if not tasks else tasks[-1].final_snapshot
            )
            terminal_stop_verified = (
                bool(tasks)
                and tasks[-1].terminal_stop_verified
                and final_snapshot is not None
                and not final_snapshot.motors_running
            )
            return IdleSessionResult(
                session_id=self.session_id,
                termination=termination,
                tasks=tuple(tasks),
                planner_calls=planner_calls,
                stale_replans=stale_replans,
                tasks_completed=sum(
                    1 for task in tasks if task.completed
                ),
                actions=actions,
                total_motion_ms=total_motion_ms,
                elapsed_ms=elapsed_ms,
                final_snapshot=final_snapshot,
                terminal_stop_verified=terminal_stop_verified,
            )


__all__ = (
    "IDLE_HELD",
    "IDLE_MISSION_FAILED",
    "IDLE_MISSION_STALE",
    "IDLE_MODEL_ABORTED",
    "IDLE_NO_FEASIBLE_CANDIDATES",
    "IDLE_PREEMPTED",
    "IDLE_SAFETY_STOP",
    "IDLE_SELECTION_FAILED",
    "IDLE_SELECTION_STALE",
    "IDLE_SESSION_BUDGET_EXHAUSTED",
    "IDLE_STOP_UNVERIFIED",
    "IDLE_TASK_COMPLETED",
    "IDLE_UNAVAILABLE",
    "IdleExplorationService",
    "IdleDutyCycleLimits",
    "IdleDutyCycleState",
    "IdleSessionLimits",
    "IdleSessionResult",
    "IdleTaskResult",
)
