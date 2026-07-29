"""Result values and cumulative budgets for idle autonomy execution."""

from dataclasses import dataclass
from typing import Optional, Tuple

from .autonomy_contract import (
    ExplorationCandidate,
    InterestObservation,
)
from .navigation_contract import boolean, integer
from .navigation_mission_contract import MissionResult
from .navigation_state import NavigationSnapshot


IDLE_TASK_COMPLETED = "IDLE_TASK_COMPLETED"
IDLE_HELD = "IDLE_HELD"
IDLE_MODEL_ABORTED = "IDLE_MODEL_ABORTED"
IDLE_PREEMPTED = "IDLE_PREEMPTED"
IDLE_UNAVAILABLE = "IDLE_UNAVAILABLE"
IDLE_NO_FEASIBLE_CANDIDATES = "IDLE_NO_FEASIBLE_CANDIDATES"
IDLE_SELECTION_FAILED = "IDLE_SELECTION_FAILED"
IDLE_SELECTION_STALE = "IDLE_SELECTION_STALE"
IDLE_MISSION_STALE = "IDLE_MISSION_STALE"
IDLE_MISSION_FAILED = "IDLE_MISSION_FAILED"
IDLE_SAFETY_STOP = "IDLE_SAFETY_STOP"
IDLE_STOP_UNVERIFIED = "IDLE_STOP_UNVERIFIED"
IDLE_SESSION_BUDGET_EXHAUSTED = "IDLE_SESSION_BUDGET_EXHAUSTED"


@dataclass(frozen=True)
class IdleDutyCycleLimits:
    """Persistent limits across public scheduler calls.

    Session limits bound one invocation.  These larger limits prevent an
    always-on caller from resetting every budget simply by invoking
    ``run_once`` or ``run_session`` again.  Re-arming is an explicit,
    stopped-state host operation.
    """

    max_task_attempts: int = 32
    max_planner_calls: int = 64
    max_stale_replans: int = 16
    max_elapsed_ms: int = 900_000
    max_actions: int = 5_000
    max_total_motion_ms: int = 300_000

    def __post_init__(self) -> None:
        integer(
            "max_task_attempts",
            self.max_task_attempts,
            1,
            100_000,
        )
        integer(
            "max_planner_calls",
            self.max_planner_calls,
            1,
            100_000,
        )
        integer(
            "max_stale_replans",
            self.max_stale_replans,
            0,
            100_000,
        )
        integer(
            "max_elapsed_ms",
            self.max_elapsed_ms,
            1,
            86_400_000,
        )
        integer("max_actions", self.max_actions, 1, 1_000_000)
        integer(
            "max_total_motion_ms",
            self.max_total_motion_ms,
            1,
            86_400_000,
        )


@dataclass(frozen=True)
class IdleDutyCycleState:
    """Read-only accounting for the currently armed idle duty cycle."""

    generation: int
    started_at_ms: Optional[int]
    task_attempts: int
    planner_calls: int
    stale_replans: int
    actions: int
    total_motion_ms: int
    exhausted: bool

    def __post_init__(self) -> None:
        integer("generation", self.generation, 1, 2**63 - 1)
        if self.started_at_ms is not None:
            integer(
                "started_at_ms",
                self.started_at_ms,
                0,
                2**63 - 1,
            )
        for name, value in (
            ("task_attempts", self.task_attempts),
            ("planner_calls", self.planner_calls),
            ("stale_replans", self.stale_replans),
            ("actions", self.actions),
            ("total_motion_ms", self.total_motion_ms),
        ):
            integer(name, value, 0, 2**63 - 1)
        boolean("exhausted", self.exhausted)


@dataclass(frozen=True)
class IdleSessionLimits:
    """Cumulative limits that cannot reset between one-leg idle tasks."""

    max_tasks: int = 4
    max_planner_calls: int = 8
    max_stale_replans: int = 3
    max_elapsed_ms: int = 90_000
    max_actions: int = 320
    max_total_motion_ms: int = 40_000

    def __post_init__(self) -> None:
        integer("max_tasks", self.max_tasks, 1, 1_000)
        integer(
            "max_planner_calls",
            self.max_planner_calls,
            1,
            10_000,
        )
        integer(
            "max_stale_replans",
            self.max_stale_replans,
            0,
            10_000,
        )
        integer(
            "max_elapsed_ms",
            self.max_elapsed_ms,
            1,
            3_600_000,
        )
        integer("max_actions", self.max_actions, 1, 100_000)
        integer(
            "max_total_motion_ms",
            self.max_total_motion_ms,
            1,
            3_600_000,
        )


@dataclass(frozen=True)
class IdleTaskResult:
    termination: str
    lease_generation: Optional[int]
    planner_calls: int
    stale_replans: int
    selected_candidate_id: Optional[str]
    observation: Optional[InterestObservation]
    candidates: Tuple[ExplorationCandidate, ...]
    mission: Optional[MissionResult]
    final_snapshot: Optional[NavigationSnapshot]
    terminal_stop_verified: bool
    trace: Tuple[str, ...]

    @property
    def completed(self) -> bool:
        return (
            self.termination == IDLE_TASK_COMPLETED
            and self.mission is not None
            and self.mission.completed
            and self.terminal_stop_verified
        )


@dataclass(frozen=True)
class IdleSessionResult:
    session_id: str
    termination: str
    tasks: Tuple[IdleTaskResult, ...]
    planner_calls: int
    stale_replans: int
    tasks_completed: int
    actions: int
    total_motion_ms: int
    elapsed_ms: int
    final_snapshot: Optional[NavigationSnapshot]
    terminal_stop_verified: bool

    def to_dict(self):
        return {
            "schema": "robot-idle-autonomy-session-result/v1",
            "session_id": self.session_id,
            "termination": self.termination,
            "task_attempts": len(self.tasks),
            "tasks_completed": self.tasks_completed,
            "planner_calls": self.planner_calls,
            "stale_replans": self.stale_replans,
            "actions": self.actions,
            "total_motion_ms": self.total_motion_ms,
            "elapsed_ms": self.elapsed_ms,
            "terminal_stop_verified": self.terminal_stop_verified,
            "tasks": [
                {
                    "termination": task.termination,
                    "lease_generation": task.lease_generation,
                    "selected_candidate_id": (
                        task.selected_candidate_id
                    ),
                    "planner_calls": task.planner_calls,
                    "stale_replans": task.stale_replans,
                    "completed": task.completed,
                    "trace": list(task.trace),
                }
                for task in self.tasks
            ],
        }


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
    "IdleDutyCycleLimits",
    "IdleDutyCycleState",
    "IdleSessionLimits",
    "IdleSessionResult",
    "IdleTaskResult",
)
