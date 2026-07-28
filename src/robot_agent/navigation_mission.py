"""Bounded multi-waypoint mission execution over the navigation simulator.

This module is the first plan-execution layer above ``NavigationEpisode``.
It deliberately works with semantic waypoint legs rather than per-tick wheel
commands.  A slow planner can therefore build one immutable, version-bound
plan while the existing deterministic behaviors and ``MotionSupervisor``
remain responsible for short-lived navigation decisions.

The implementation is simulator-only.  It does not import RobotAPI, SSH, the
EV3 HAL, or any physical transport.
"""

from dataclasses import dataclass
import json
import threading
from typing import Mapping, Optional, Tuple

from .navigation_contract import (
    NavigationContractError,
    WaypointGoal,
    identifier,
    integer,
)
from .navigation_episode import (
    NAVIGATION_ABORTED,
    NAVIGATION_BUDGET_EXHAUSTED,
    NAVIGATION_PLAN_STALE,
    NAVIGATION_SAFETY_STOP,
    GoalSeekingBehavior,
    NavigationEpisode,
    NavigationLimits,
    NavigationResult,
    ObstacleAvoidanceBehavior,
)
from .navigation_simulator import DifferentialDriveSimulator
from .navigation_state import (
    NavigationSnapshot,
    ProposalInbox,
)
from .navigation_supervisor import MotionSupervisor


MISSION_PLAN_SCHEMA = "robot-navigation-mission-plan/v1"
MAX_MISSION_PLAN_BYTES = 32 * 1024
MAX_MISSION_LEGS = 8

MISSION_COMPLETED = "MISSION_COMPLETED"
MISSION_ABORTED = "MISSION_ABORTED"
MISSION_SAFETY_STOP = "MISSION_SAFETY_STOP"
MISSION_BUDGET_EXHAUSTED = "MISSION_BUDGET_EXHAUSTED"
MISSION_PLAN_REJECTED = "MISSION_PLAN_REJECTED"
MISSION_PLAN_STALE = "MISSION_PLAN_STALE"
MISSION_LEG_FAILED = "MISSION_LEG_FAILED"


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


@dataclass(frozen=True)
class MissionLeg:
    """One semantic waypoint in an ordered mission plan."""

    leg_id: str
    target_x_mm: int
    target_y_mm: int
    tolerance_mm: int = 35

    def __post_init__(self) -> None:
        identifier("leg_id", self.leg_id, 96)
        integer("target_x_mm", self.target_x_mm, -1_000_000, 1_000_000)
        integer("target_y_mm", self.target_y_mm, -1_000_000, 1_000_000)
        integer("tolerance_mm", self.tolerance_mm, 1, 10_000)

    def to_dict(self) -> Mapping[str, object]:
        return {
            "leg_id": self.leg_id,
            "target_x_mm": self.target_x_mm,
            "target_y_mm": self.target_y_mm,
            "tolerance_mm": self.tolerance_mm,
        }


@dataclass(frozen=True)
class MissionPlan:
    """An immutable plan bound to one stopped simulator snapshot."""

    plan_id: str
    robot_id: str
    controller_instance_id: str
    based_on_state_version: int
    based_on_world_model_version: int
    plan_revision: int
    legs: Tuple[MissionLeg, ...]

    def __post_init__(self) -> None:
        identifier("plan_id", self.plan_id)
        identifier("robot_id", self.robot_id)
        identifier("controller_instance_id", self.controller_instance_id)
        integer(
            "based_on_state_version",
            self.based_on_state_version,
            1,
            2**63 - 1,
        )
        integer(
            "based_on_world_model_version",
            self.based_on_world_model_version,
            1,
            2**63 - 1,
        )
        integer("plan_revision", self.plan_revision, 1, 2**63 - 1)
        if (
            not isinstance(self.legs, tuple)
            or not 1 <= len(self.legs) <= MAX_MISSION_LEGS
            or any(not isinstance(leg, MissionLeg) for leg in self.legs)
        ):
            raise NavigationContractError(
                "invalid_mission_legs",
                "Mission plan requires 1..{} typed legs".format(
                    MAX_MISSION_LEGS
                ),
            )
        leg_ids = tuple(leg.leg_id for leg in self.legs)
        if len(set(leg_ids)) != len(leg_ids):
            raise NavigationContractError(
                "duplicate_mission_leg",
                "Mission leg IDs must be unique",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": MISSION_PLAN_SCHEMA,
            "plan_id": self.plan_id,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "based_on_state_version": self.based_on_state_version,
            "based_on_world_model_version": (
                self.based_on_world_model_version
            ),
            "plan_revision": self.plan_revision,
            "legs": [leg.to_dict() for leg in self.legs],
        }

    def assert_matches_snapshot(
        self,
        snapshot: NavigationSnapshot,
    ) -> None:
        if not isinstance(snapshot, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_snapshot",
                "Mission activation requires NavigationSnapshot",
            )
        if (
            snapshot.robot_id != self.robot_id
            or snapshot.controller_instance_id
            != self.controller_instance_id
            or snapshot.state_version != self.based_on_state_version
            or snapshot.world_model_version
            != self.based_on_world_model_version
            or snapshot.motors_running
            or snapshot.touch_pressed
            or snapshot.active_faults
        ):
            raise NavigationContractError(
                "stale_mission_plan",
                "Mission plan does not match a current safe stopped snapshot",
            )


def decode_mission_plan(raw: bytes) -> MissionPlan:
    """Decode an untrusted strict JSON plan without granting motion."""

    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > MAX_MISSION_PLAN_BYTES
    ):
        raise NavigationContractError(
            "invalid_mission_plan_body",
            "Mission plan body is invalid",
        )
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, TypeError, ValueError):
        raise NavigationContractError(
            "invalid_mission_plan_json",
            "Mission planner returned invalid JSON",
        ) from None
    expected = {
        "schema",
        "plan_id",
        "robot_id",
        "controller_instance_id",
        "based_on_state_version",
        "based_on_world_model_version",
        "plan_revision",
        "legs",
    }
    if not isinstance(value, dict) or set(value) != expected:
        raise NavigationContractError(
            "invalid_mission_plan_fields",
            "Mission plan fields are invalid",
        )
    if value.get("schema") != MISSION_PLAN_SCHEMA:
        raise NavigationContractError(
            "invalid_mission_plan_schema",
            "Mission plan schema is not supported",
        )
    raw_legs = value.get("legs")
    if (
        not isinstance(raw_legs, list)
        or not 1 <= len(raw_legs) <= MAX_MISSION_LEGS
    ):
        raise NavigationContractError(
            "invalid_mission_legs",
            "Mission plan has an invalid leg count",
        )
    legs = []
    for raw_leg in raw_legs:
        if (
            not isinstance(raw_leg, dict)
            or set(raw_leg)
            != {
                "leg_id",
                "target_x_mm",
                "target_y_mm",
                "tolerance_mm",
            }
        ):
            raise NavigationContractError(
                "invalid_mission_leg_fields",
                "Mission leg fields are invalid",
            )
        legs.append(MissionLeg(
            leg_id=raw_leg["leg_id"],
            target_x_mm=raw_leg["target_x_mm"],
            target_y_mm=raw_leg["target_y_mm"],
            tolerance_mm=raw_leg["tolerance_mm"],
        ))
    return MissionPlan(
        plan_id=value["plan_id"],
        robot_id=value["robot_id"],
        controller_instance_id=value["controller_instance_id"],
        based_on_state_version=value["based_on_state_version"],
        based_on_world_model_version=value[
            "based_on_world_model_version"
        ],
        plan_revision=value["plan_revision"],
        legs=tuple(legs),
    )


@dataclass(frozen=True)
class MissionLimits:
    """Global budgets shared across every leg in a mission."""

    max_legs: int = MAX_MISSION_LEGS
    max_ticks: int = 1_200
    max_elapsed_ms: int = 180_000
    max_proposals: int = 2_400
    max_replans: int = 1_200
    max_actions: int = 1_100
    max_total_motion_ms: int = 150_000

    def __post_init__(self) -> None:
        integer("max_legs", self.max_legs, 1, MAX_MISSION_LEGS)
        integer("max_ticks", self.max_ticks, 1, 100_000)
        integer(
            "max_elapsed_ms",
            self.max_elapsed_ms,
            1,
            3_600_000,
        )
        integer("max_proposals", self.max_proposals, 1, 100_000)
        integer("max_replans", self.max_replans, 0, 100_000)
        integer("max_actions", self.max_actions, 1, 100_000)
        integer(
            "max_total_motion_ms",
            self.max_total_motion_ms,
            1,
            3_600_000,
        )


@dataclass(frozen=True)
class MissionLegResult:
    leg_index: int
    leg: MissionLeg
    goal: WaypointGoal
    navigation: NavigationResult

    def to_dict(self) -> Mapping[str, object]:
        return {
            "leg_index": self.leg_index,
            "leg": self.leg.to_dict(),
            "goal": {
                "goal_id": self.goal.goal_id,
                "goal_epoch": self.goal.goal_epoch,
                "plan_revision": self.goal.plan_revision,
            },
            "navigation": self.navigation.to_dict(),
        }


@dataclass(frozen=True)
class MissionResult:
    plan_id: str
    completed: bool
    termination: str
    legs_completed: int
    ticks: int
    proposals: int
    replans: int
    actions: int
    total_motion_ms: int
    elapsed_ms: int
    final_snapshot: NavigationSnapshot
    terminal_stop_verified: bool
    leg_results: Tuple[MissionLegResult, ...]

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": "robot-navigation-mission-result/v1",
            "plan_id": self.plan_id,
            "completed": self.completed,
            "termination": self.termination,
            "legs_completed": self.legs_completed,
            "ticks": self.ticks,
            "proposals": self.proposals,
            "replans": self.replans,
            "actions": self.actions,
            "total_motion_ms": self.total_motion_ms,
            "elapsed_ms": self.elapsed_ms,
            "terminal_stop_verified": self.terminal_stop_verified,
            "final_pose": {
                "x_mm": self.final_snapshot.pose.x_mm,
                "y_mm": self.final_snapshot.pose.y_mm,
                "heading_mdeg": (
                    self.final_snapshot.pose.heading_mdeg
                ),
            },
            "legs": [
                result.to_dict() for result in self.leg_results
            ],
        }


class MissionRunner:
    """Execute one immutable plan as verified, sequential waypoint legs."""

    def __init__(
        self,
        plant: DifferentialDriveSimulator,
        supervisor: MotionSupervisor,
        inbox: ProposalInbox,
        starting_goal_epoch: int,
        per_leg_limits: NavigationLimits = NavigationLimits(),
        mission_limits: MissionLimits = MissionLimits(),
        cancel_event=None,
    ):
        if not isinstance(plant, DifferentialDriveSimulator):
            raise NavigationContractError(
                "invalid_navigation_plant",
                "MissionRunner is simulator-only",
            )
        if not isinstance(supervisor, MotionSupervisor):
            raise NavigationContractError(
                "invalid_motion_supervisor",
                "MissionRunner requires MotionSupervisor",
            )
        if not isinstance(inbox, ProposalInbox):
            raise NavigationContractError(
                "invalid_proposal_inbox",
                "MissionRunner requires ProposalInbox",
            )
        integer(
            "starting_goal_epoch",
            starting_goal_epoch,
            1,
            2**63 - MAX_MISSION_LEGS,
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
        if cancel_event is not None and not callable(
            getattr(cancel_event, "is_set", None)
        ):
            raise NavigationContractError(
                "invalid_cancel_event",
                "Cancel event must expose is_set()",
            )
        self.plant = plant
        self.supervisor = supervisor
        self.inbox = inbox
        self.starting_goal_epoch = starting_goal_epoch
        self.per_leg_limits = per_leg_limits
        self.mission_limits = mission_limits
        self.cancel_event = cancel_event
        self._run_lock = threading.Lock()
        self._has_run = False

    def _goal(self, plan: MissionPlan, index: int) -> WaypointGoal:
        leg = plan.legs[index]
        return WaypointGoal(
            goal_id=leg.leg_id,
            goal_epoch=self.starting_goal_epoch + index,
            plan_revision=plan.plan_revision,
            target_x_mm=leg.target_x_mm,
            target_y_mm=leg.target_y_mm,
            tolerance_mm=leg.tolerance_mm,
        )

    @staticmethod
    def _totals(results: Tuple[MissionLegResult, ...]):
        return {
            "ticks": sum(value.navigation.ticks for value in results),
            "proposals": sum(
                value.navigation.proposals for value in results
            ),
            "replans": sum(
                value.navigation.replans for value in results
            ),
            "actions": sum(
                value.navigation.actions for value in results
            ),
            "total_motion_ms": sum(
                value.navigation.total_motion_ms for value in results
            ),
        }

    def _remaining_limits(
        self,
        totals,
        elapsed_ms: int,
    ) -> Optional[NavigationLimits]:
        remaining = {
            "max_ticks": self.mission_limits.max_ticks - totals["ticks"],
            "max_elapsed_ms": (
                self.mission_limits.max_elapsed_ms - elapsed_ms
            ),
            "max_proposals": (
                self.mission_limits.max_proposals - totals["proposals"]
            ),
            "max_replans": (
                self.mission_limits.max_replans - totals["replans"]
            ),
            "max_actions": (
                self.mission_limits.max_actions - totals["actions"]
            ),
            "max_total_motion_ms": (
                self.mission_limits.max_total_motion_ms
                - totals["total_motion_ms"]
            ),
        }
        if (
            remaining["max_ticks"] <= 0
            or remaining["max_elapsed_ms"] <= 0
            or remaining["max_proposals"] <= 0
            or remaining["max_replans"] < 0
            or remaining["max_actions"] <= 0
            or remaining["max_total_motion_ms"] <= 0
        ):
            return None
        return NavigationLimits(
            max_ticks=min(
                self.per_leg_limits.max_ticks,
                remaining["max_ticks"],
            ),
            max_elapsed_ms=min(
                self.per_leg_limits.max_elapsed_ms,
                remaining["max_elapsed_ms"],
            ),
            max_proposals=min(
                self.per_leg_limits.max_proposals,
                remaining["max_proposals"],
            ),
            max_replans=min(
                self.per_leg_limits.max_replans,
                remaining["max_replans"],
            ),
            max_actions=min(
                self.per_leg_limits.max_actions,
                remaining["max_actions"],
            ),
            max_total_motion_ms=min(
                self.per_leg_limits.max_total_motion_ms,
                remaining["max_total_motion_ms"],
            ),
            max_no_progress_ticks=(
                self.per_leg_limits.max_no_progress_ticks
            ),
            minimum_progress_mm=self.per_leg_limits.minimum_progress_mm,
        )

    def _result(
        self,
        plan: MissionPlan,
        termination: str,
        started_at_ms: int,
        final_snapshot: NavigationSnapshot,
        leg_results,
        terminal_stop_verified: bool,
    ) -> MissionResult:
        values = tuple(leg_results)
        totals = self._totals(values)
        elapsed_ms = max(
            0,
            self.plant.clock_ms() - started_at_ms,
        )
        if (
            termination == MISSION_COMPLETED
            and elapsed_ms > self.mission_limits.max_elapsed_ms
        ):
            termination = MISSION_BUDGET_EXHAUSTED
        completed = (
            termination == MISSION_COMPLETED
            and len(values) == len(plan.legs)
            and all(value.navigation.completed for value in values)
            and terminal_stop_verified
            and not final_snapshot.motors_running
            and not final_snapshot.touch_pressed
            and not final_snapshot.active_faults
            and elapsed_ms <= self.mission_limits.max_elapsed_ms
        )
        return MissionResult(
            plan_id=plan.plan_id,
            completed=completed,
            termination=termination,
            legs_completed=sum(
                1 for value in values if value.navigation.completed
            ),
            ticks=totals["ticks"],
            proposals=totals["proposals"],
            replans=totals["replans"],
            actions=totals["actions"],
            total_motion_ms=totals["total_motion_ms"],
            elapsed_ms=elapsed_ms,
            final_snapshot=final_snapshot,
            terminal_stop_verified=terminal_stop_verified,
            leg_results=values,
        )

    def _verified_stop(
        self,
        goal: WaypointGoal,
        snapshot: NavigationSnapshot,
    ) -> Tuple[NavigationSnapshot, bool]:
        try:
            current = self.plant.observe(goal)
            stop = self.supervisor.force_stop(
                current,
                reason_code="mission_terminal_stop",
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
            return snapshot, False

    def run(self, plan: MissionPlan) -> MissionResult:
        """Execute each leg only after the preceding verified terminal STOP."""

        if not isinstance(plan, MissionPlan):
            raise NavigationContractError(
                "invalid_mission_plan",
                "MissionRunner requires MissionPlan",
            )
        with self._run_lock:
            if self._has_run:
                raise RuntimeError("MissionRunner can only run once")
            self._has_run = True

        if len(plan.legs) > self.mission_limits.max_legs:
            raise NavigationContractError(
                "mission_leg_budget_exceeded",
                "Mission plan exceeds the configured leg budget",
            )

        started_at_ms = self.plant.clock_ms()
        first_goal = self._goal(plan, 0)
        snapshot = self.plant.observe(first_goal)
        try:
            plan.assert_matches_snapshot(snapshot)
        except NavigationContractError:
            snapshot, stopped = self._verified_stop(first_goal, snapshot)
            return self._result(
                plan,
                MISSION_PLAN_REJECTED,
                started_at_ms,
                snapshot,
                (),
                stopped,
            )

        leg_results = []
        for index, leg in enumerate(plan.legs):
            goal = self._goal(plan, index)
            snapshot = self.plant.observe(goal)
            if (
                self.cancel_event is not None
                and self.cancel_event.is_set()
            ):
                snapshot, stopped = self._verified_stop(goal, snapshot)
                return self._result(
                    plan,
                    MISSION_ABORTED,
                    started_at_ms,
                    snapshot,
                    leg_results,
                    stopped,
                )
            if (
                snapshot.world_model_version
                != plan.based_on_world_model_version
            ):
                snapshot, stopped = self._verified_stop(goal, snapshot)
                return self._result(
                    plan,
                    MISSION_PLAN_STALE,
                    started_at_ms,
                    snapshot,
                    leg_results,
                    stopped,
                )
            totals = self._totals(tuple(leg_results))
            elapsed_ms = max(
                0,
                self.plant.clock_ms() - started_at_ms,
            )
            limits = self._remaining_limits(totals, elapsed_ms)
            if limits is None:
                snapshot, stopped = self._verified_stop(goal, snapshot)
                return self._result(
                    plan,
                    MISSION_BUDGET_EXHAUSTED,
                    started_at_ms,
                    snapshot,
                    leg_results,
                    stopped,
                )

            navigation = NavigationEpisode(
                self.plant,
                self.supervisor,
                self.inbox,
                (
                    GoalSeekingBehavior(),
                    ObstacleAvoidanceBehavior(),
                ),
                limits=limits,
                cancel_event=self.cancel_event,
                required_world_model_version=(
                    plan.based_on_world_model_version
                ),
            ).run(goal)
            leg_result = MissionLegResult(
                leg_index=index,
                leg=leg,
                goal=goal,
                navigation=navigation,
            )
            leg_results.append(leg_result)
            snapshot = navigation.final_snapshot
            if not navigation.completed:
                if navigation.termination == NAVIGATION_SAFETY_STOP:
                    termination = MISSION_SAFETY_STOP
                elif navigation.termination == NAVIGATION_BUDGET_EXHAUSTED:
                    termination = MISSION_BUDGET_EXHAUSTED
                elif navigation.termination == NAVIGATION_PLAN_STALE:
                    termination = MISSION_PLAN_STALE
                elif navigation.termination == NAVIGATION_ABORTED:
                    termination = MISSION_ABORTED
                else:
                    termination = MISSION_LEG_FAILED
                return self._result(
                    plan,
                    termination,
                    started_at_ms,
                    snapshot,
                    leg_results,
                    navigation.terminal_stop_verified,
                )
            if not navigation.terminal_stop_verified:
                return self._result(
                    plan,
                    MISSION_LEG_FAILED,
                    started_at_ms,
                    snapshot,
                    leg_results,
                    False,
                )
            boundary_snapshot = self.plant.observe(goal)
            if (
                boundary_snapshot.world_model_version
                != plan.based_on_world_model_version
            ):
                boundary_snapshot, stopped = self._verified_stop(
                    goal,
                    boundary_snapshot,
                )
                return self._result(
                    plan,
                    MISSION_PLAN_STALE,
                    started_at_ms,
                    boundary_snapshot,
                    leg_results,
                    stopped,
                )
            snapshot = boundary_snapshot

        return self._result(
            plan,
            MISSION_COMPLETED,
            started_at_ms,
            snapshot,
            leg_results,
            True,
        )
