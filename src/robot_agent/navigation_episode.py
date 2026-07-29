"""Bounded autonomous behavior loop over the navigation simulator."""

from dataclasses import dataclass
import math
from typing import Callable, Mapping, Optional, Tuple

from .navigation_contract import (
    AdvanceSegment,
    DrivePulse,
    NavigationContractError,
    PlannerProposal,
    TurnSegment,
    WaypointGoal,
    integer,
)
from .navigation_simulator import (
    DifferentialDriveSimulator,
    normalize_heading_mdeg,
)
from .navigation_state import (
    NavigationSnapshot,
    ProposalInbox,
    StateReducer,
)
from .navigation_supervisor import MotionSupervisor


NAVIGATION_GOAL_REACHED = "NAVIGATION_GOAL_REACHED"
NAVIGATION_ABORTED = "NAVIGATION_ABORTED"
NAVIGATION_SAFETY_STOP = "NAVIGATION_SAFETY_STOP"
NAVIGATION_PROGRESS_FAILED = "NAVIGATION_PROGRESS_FAILED"
NAVIGATION_BUDGET_EXHAUSTED = "NAVIGATION_BUDGET_EXHAUSTED"
NAVIGATION_EXECUTION_FAILED = "NAVIGATION_EXECUTION_FAILED"
NAVIGATION_PLAN_STALE = "NAVIGATION_PLAN_STALE"
NAVIGATION_REFRESH_REQUIRED = "NAVIGATION_REFRESH_REQUIRED"


def distance_to_goal_mm(
    snapshot: NavigationSnapshot,
    goal: WaypointGoal,
) -> int:
    return int(round(math.hypot(
        goal.target_x_mm - snapshot.pose.x_mm,
        goal.target_y_mm - snapshot.pose.y_mm,
    )))


def _proposal(
    proposal_id: str,
    goal: WaypointGoal,
    snapshot: NavigationSnapshot,
    decision: str,
    confidence_milli: int,
    segment=None,
    reason_code=None,
) -> PlannerProposal:
    return PlannerProposal(
        proposal_id=proposal_id,
        goal_id=goal.goal_id,
        goal_epoch=goal.goal_epoch,
        plan_revision=goal.plan_revision,
        based_on_state_version=snapshot.state_version,
        based_on_world_model_version=snapshot.world_model_version,
        decision=decision,
        confidence_milli=confidence_milli,
        segment=segment,
        reason_code=reason_code,
    )


class GoalSeekingBehavior:
    """Language-free reference controller for simulator evaluation only."""

    source_id = "goal-seeking"

    def __init__(
        self,
        linear_speed_mm_s: int = 100,
        angular_speed_mdeg_s: int = 90_000,
        heading_tolerance_mdeg: int = 6_000,
        max_advance_mm: int = 120,
        max_turn_mdeg: int = 30_000,
    ):
        integer("linear_speed_mm_s", linear_speed_mm_s, 1, 2_000)
        integer(
            "angular_speed_mdeg_s",
            angular_speed_mdeg_s,
            1,
            720_000,
        )
        integer(
            "heading_tolerance_mdeg",
            heading_tolerance_mdeg,
            1,
            90_000,
        )
        integer("max_advance_mm", max_advance_mm, 1, 10_000)
        integer("max_turn_mdeg", max_turn_mdeg, 1, 180_000)
        self.linear_speed_mm_s = linear_speed_mm_s
        self.angular_speed_mdeg_s = angular_speed_mdeg_s
        self.heading_tolerance_mdeg = heading_tolerance_mdeg
        self.max_advance_mm = max_advance_mm
        self.max_turn_mdeg = max_turn_mdeg
        self._proposal_number = 0

    def propose(
        self,
        goal: WaypointGoal,
        snapshot: NavigationSnapshot,
    ) -> PlannerProposal:
        self._proposal_number += 1
        proposal_id = "goal-seeking-e{}-{}".format(
            goal.goal_epoch,
            self._proposal_number,
        )
        delta_x = goal.target_x_mm - snapshot.pose.x_mm
        delta_y = goal.target_y_mm - snapshot.pose.y_mm
        distance = int(round(math.hypot(delta_x, delta_y)))
        if distance <= goal.tolerance_mm:
            return _proposal(
                proposal_id,
                goal,
                snapshot,
                "HOLD",
                1_000,
                reason_code="goal_candidate_reached",
            )

        desired_heading = int(
            round(math.degrees(math.atan2(delta_y, delta_x)) * 1_000)
        )
        heading_error = normalize_heading_mdeg(
            desired_heading - snapshot.pose.heading_mdeg
        )
        if abs(heading_error) > self.heading_tolerance_mdeg:
            angle = max(
                -self.max_turn_mdeg,
                min(self.max_turn_mdeg, heading_error),
            )
            return _proposal(
                proposal_id,
                goal,
                snapshot,
                "NEXT_SEGMENT",
                900,
                segment=TurnSegment(
                    angle_mdeg=angle,
                    angular_speed_mdeg_s=(
                        self.angular_speed_mdeg_s
                    ),
                ),
            )
        return _proposal(
            proposal_id,
            goal,
            snapshot,
            "NEXT_SEGMENT",
            900,
            segment=AdvanceSegment(
                distance_mm=max(
                    1,
                    min(self.max_advance_mm, distance),
                ),
                speed_mm_s=self.linear_speed_mm_s,
            ),
        )


class ObstacleAvoidanceBehavior:
    """Small reactive behavior that outranks goal seeking in the simulator.

    It turns toward the larger side clearance and then commits to a bounded
    number of short forward segments.  It is an evaluation baseline, not a
    claim of SLAM or robust real-world navigation.
    """

    source_id = "obstacle-avoidance"

    def __init__(
        self,
        trigger_mm: int = 170,
        turn_mdeg: int = 30_000,
        angular_speed_mdeg_s: int = 75_000,
        bypass_segments: int = 18,
        bypass_distance_mm: int = 80,
        bypass_speed_mm_s: int = 80,
    ):
        integer("trigger_mm", trigger_mm, 1, 100_000)
        integer("turn_mdeg", turn_mdeg, 1, 180_000)
        integer(
            "angular_speed_mdeg_s",
            angular_speed_mdeg_s,
            1,
            720_000,
        )
        integer("bypass_segments", bypass_segments, 1, 1_000)
        integer(
            "bypass_distance_mm",
            bypass_distance_mm,
            1,
            10_000,
        )
        integer(
            "bypass_speed_mm_s",
            bypass_speed_mm_s,
            1,
            2_000,
        )
        self.trigger_mm = trigger_mm
        self.turn_mdeg = turn_mdeg
        self.angular_speed_mdeg_s = angular_speed_mdeg_s
        self.bypass_segments = bypass_segments
        self.bypass_distance_mm = bypass_distance_mm
        self.bypass_speed_mm_s = bypass_speed_mm_s
        self._turn_direction = 0
        self._remaining_bypass = 0
        self._proposal_number = 0

    def _next_id(self, goal_epoch: int) -> str:
        self._proposal_number += 1
        return "obstacle-avoidance-e{}-{}".format(
            goal_epoch,
            self._proposal_number,
        )

    def propose(
        self,
        goal: WaypointGoal,
        snapshot: NavigationSnapshot,
    ) -> Optional[PlannerProposal]:
        evidence = snapshot.clearance
        if evidence.source != "simulation_metric":
            return _proposal(
                self._next_id(goal.goal_epoch),
                goal,
                snapshot,
                "HOLD",
                1_000,
                reason_code="metric_clearance_unavailable",
            )
        blocked = (
            evidence.near_obstacle_latched
            or evidence.forward_mm <= self.trigger_mm
        )
        if blocked:
            left = -1 if evidence.left_mm is None else evidence.left_mm
            right = -1 if evidence.right_mm is None else evidence.right_mm
            if left == right == -1:
                return _proposal(
                    self._next_id(goal.goal_epoch),
                    goal,
                    snapshot,
                    "HOLD",
                    1_000,
                    reason_code="avoidance_direction_unknown",
                )
            if self._turn_direction == 0:
                self._turn_direction = 1 if left >= right else -1
            self._remaining_bypass = self.bypass_segments
            return _proposal(
                self._next_id(goal.goal_epoch),
                goal,
                snapshot,
                "NEXT_SEGMENT",
                1_000,
                segment=TurnSegment(
                    angle_mdeg=self._turn_direction * self.turn_mdeg,
                    angular_speed_mdeg_s=(
                        self.angular_speed_mdeg_s
                    ),
                ),
            )
        if self._remaining_bypass > 0:
            self._remaining_bypass -= 1
            proposal = _proposal(
                self._next_id(goal.goal_epoch),
                goal,
                snapshot,
                "NEXT_SEGMENT",
                950,
                segment=AdvanceSegment(
                    distance_mm=self.bypass_distance_mm,
                    speed_mm_s=self.bypass_speed_mm_s,
                ),
            )
            if self._remaining_bypass == 0:
                self._turn_direction = 0
            return proposal
        return None


@dataclass(frozen=True)
class NavigationLimits:
    max_ticks: int = 300
    max_elapsed_ms: int = 45_000
    max_proposals: int = 700
    max_replans: int = 300
    max_actions: int = 280
    max_total_motion_ms: int = 35_000
    max_no_progress_ticks: int = 80
    minimum_progress_mm: int = 2

    def __post_init__(self) -> None:
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
        integer(
            "max_no_progress_ticks",
            self.max_no_progress_ticks,
            1,
            100_000,
        )
        integer(
            "minimum_progress_mm",
            self.minimum_progress_mm,
            1,
            1_000,
        )


@dataclass(frozen=True)
class NavigationStep:
    tick: int
    state_before: int
    state_after: int
    proposal_ids: Tuple[str, ...]
    decision_id: str
    decision_kind: str
    decision_reason: str
    duration_ms: int
    distance_before_mm: int
    distance_after_mm: int
    progress_verified: bool


@dataclass(frozen=True)
class NavigationResult:
    goal_id: str
    completed: bool
    termination: str
    ticks: int
    proposals: int
    replans: int
    actions: int
    total_motion_ms: int
    final_snapshot: NavigationSnapshot
    terminal_stop_verified: bool
    trace: Tuple[str, ...]
    steps: Tuple[NavigationStep, ...]

    def to_dict(self) -> Mapping[str, object]:
        return {
            "goal_id": self.goal_id,
            "completed": self.completed,
            "termination": self.termination,
            "ticks": self.ticks,
            "proposals": self.proposals,
            "replans": self.replans,
            "actions": self.actions,
            "total_motion_ms": self.total_motion_ms,
            "final_pose": {
                "x_mm": self.final_snapshot.pose.x_mm,
                "y_mm": self.final_snapshot.pose.y_mm,
                "heading_mdeg": (
                    self.final_snapshot.pose.heading_mdeg
                ),
            },
            "terminal_stop_verified": self.terminal_stop_verified,
            "trace": list(self.trace),
            "steps": [
                {
                    "tick": step.tick,
                    "state_before": step.state_before,
                    "state_after": step.state_after,
                    "proposal_ids": list(step.proposal_ids),
                    "decision_id": step.decision_id,
                    "decision_kind": step.decision_kind,
                    "decision_reason": step.decision_reason,
                    "duration_ms": step.duration_ms,
                    "distance_before_mm": step.distance_before_mm,
                    "distance_after_mm": step.distance_after_mm,
                    "progress_verified": step.progress_verified,
                }
                for step in self.steps
            ],
        }


class NavigationEpisode:
    """Observe, publish, arbitrate, execute, verify and replan."""

    def __init__(
        self,
        plant: DifferentialDriveSimulator,
        supervisor: MotionSupervisor,
        inbox: ProposalInbox,
        local_behaviors: Tuple[object, ...],
        limits: NavigationLimits = NavigationLimits(),
        observation_sink: Optional[
            Callable[[NavigationSnapshot], None]
        ] = None,
        before_arbitration: Optional[
            Callable[[NavigationSnapshot], None]
        ] = None,
        cancel_event=None,
        required_world_model_version: Optional[int] = None,
    ):
        if not isinstance(plant, DifferentialDriveSimulator):
            raise NavigationContractError(
                "invalid_navigation_plant",
                "NavigationEpisode currently requires simulator plant",
            )
        if not isinstance(supervisor, MotionSupervisor):
            raise NavigationContractError(
                "invalid_motion_supervisor",
                "NavigationEpisode requires MotionSupervisor",
            )
        if not isinstance(inbox, ProposalInbox):
            raise NavigationContractError(
                "invalid_proposal_inbox",
                "NavigationEpisode requires ProposalInbox",
            )
        if (
            not isinstance(local_behaviors, tuple)
            or any(
                type(behavior)
                not in (
                    GoalSeekingBehavior,
                    ObstacleAvoidanceBehavior,
                )
                for behavior in local_behaviors
            )
        ):
            raise NavigationContractError(
                "invalid_navigation_behaviors",
                "Only built-in deterministic reference behaviors "
                "may run inside the simulator tick",
            )
        if not isinstance(limits, NavigationLimits):
            raise NavigationContractError(
                "invalid_navigation_limits",
                "Navigation limits are invalid",
            )
        if observation_sink is not None and not callable(observation_sink):
            raise NavigationContractError(
                "invalid_observation_sink",
                "Observation sink must be callable",
            )
        if before_arbitration is not None and not callable(
            before_arbitration
        ):
            raise NavigationContractError(
                "invalid_tick_hook",
                "Before-arbitration hook must be callable",
            )
        if cancel_event is not None and not callable(
            getattr(cancel_event, "is_set", None)
        ):
            raise NavigationContractError(
                "invalid_cancel_event",
                "Cancel event must expose is_set()",
            )
        if required_world_model_version is not None:
            integer(
                "required_world_model_version",
                required_world_model_version,
                1,
                2**63 - 1,
            )
        self.plant = plant
        self.supervisor = supervisor
        self.inbox = inbox
        self.local_behaviors = local_behaviors
        self.limits = limits
        self.observation_sink = observation_sink
        self.before_arbitration = before_arbitration
        self.cancel_event = cancel_event
        self.required_world_model_version = (
            required_world_model_version
        )

    def _cancelled(self) -> bool:
        return (
            self.cancel_event is not None
            and bool(self.cancel_event.is_set())
        )

    def _publish_observation(
        self,
        snapshot: NavigationSnapshot,
    ) -> None:
        if self.observation_sink is not None:
            self.observation_sink(snapshot)

    def _terminal_stop_cost_ms(self) -> int:
        return self.plant.settings.idle_tick_ms

    def _publish_local(
        self,
        goal: WaypointGoal,
        snapshot: NavigationSnapshot,
    ) -> int:
        count = 0
        for behavior in self.local_behaviors:
            if type(behavior) is GoalSeekingBehavior:
                proposal = GoalSeekingBehavior.propose(
                    behavior,
                    goal,
                    snapshot,
                )
            else:
                proposal = ObstacleAvoidanceBehavior.propose(
                    behavior,
                    goal,
                    snapshot,
                )
            if proposal is None:
                continue
            source_id = behavior.source_id
            self.inbox.publish_host(proposal, source_id)
            count += 1
        return count

    def _verify_progress(
        self,
        before: NavigationSnapshot,
        after: NavigationSnapshot,
        pulse: DrivePulse,
    ) -> bool:
        if pulse.kind == "STOP":
            return (
                after.state_version > before.state_version
                and not after.motors_running
            )
        if (
            after.state_version <= before.state_version
            or after.world_model_version
            < before.world_model_version
            or after.motors_running
        ):
            return False
        left_delta = (
            after.left_encoder_mdeg - before.left_encoder_mdeg
        )
        right_delta = (
            after.right_encoder_mdeg - before.right_encoder_mdeg
        )
        expected_left = (
            pulse.left_speed_dps
            * self.plant.profile.left_motor_sign
        )
        expected_right = (
            pulse.right_speed_dps
            * self.plant.profile.right_motor_sign
        )
        if (
            left_delta == 0
            or right_delta == 0
            or left_delta * expected_left <= 0
            or right_delta * expected_right <= 0
        ):
            return False
        if pulse.reason_code == "authorized_advance":
            same_direction = left_delta * right_delta > 0
            larger = max(abs(left_delta), abs(right_delta))
            smaller = min(abs(left_delta), abs(right_delta))
            pose_progress = math.hypot(
                after.pose.x_mm - before.pose.x_mm,
                after.pose.y_mm - before.pose.y_mm,
            )
            return (
                same_direction
                and smaller * 100 >= larger * 70
                and pose_progress >= self.limits.minimum_progress_mm
            )
        if pulse.reason_code == "authorized_turn":
            heading_delta = normalize_heading_mdeg(
                after.pose.heading_mdeg - before.pose.heading_mdeg
            )
            return (
                left_delta * right_delta < 0
                and heading_delta != 0
                and (
                    heading_delta > 0
                    if expected_right > expected_left
                    else heading_delta < 0
                )
            )
        return False

    def _finish(
        self,
        goal: WaypointGoal,
        termination: str,
        counters,
        trace,
        steps,
        snapshot: NavigationSnapshot,
        already_stopped: bool,
    ) -> NavigationResult:
        self.supervisor.observe_emergency(snapshot)
        terminal_stop_verified = (
            already_stopped and not snapshot.motors_running
        )
        if not terminal_stop_verified:
            trace.append("TERMINAL_STOP")
            try:
                stop = self.supervisor.force_stop(snapshot)
                snapshot = self.plant.apply(stop, goal)
                terminal_stop_verified = (
                    not snapshot.motors_running
                    and snapshot.state_version
                    > stop.based_on_state_version
                )
            except Exception:
                terminal_stop_verified = False
        self.supervisor.observe_emergency(snapshot)
        if snapshot.active_faults or snapshot.touch_pressed:
            termination = NAVIGATION_SAFETY_STOP
        completed = (
            termination == NAVIGATION_GOAL_REACHED
            and terminal_stop_verified
            and distance_to_goal_mm(snapshot, goal)
            <= goal.tolerance_mm
            and not snapshot.active_faults
            and not snapshot.touch_pressed
        )
        trace.append("SUCCEEDED" if completed else "FAILED")
        return NavigationResult(
            goal_id=goal.goal_id,
            completed=completed,
            termination=(
                termination
                if terminal_stop_verified
                else NAVIGATION_EXECUTION_FAILED
            ),
            ticks=counters["ticks"],
            proposals=counters["proposals"],
            replans=counters["replans"],
            actions=counters["actions"],
            total_motion_ms=counters["total_motion_ms"],
            final_snapshot=snapshot,
            terminal_stop_verified=terminal_stop_verified,
            trace=tuple(trace),
            steps=tuple(steps),
        )

    def run(self, goal: WaypointGoal) -> NavigationResult:
        if not isinstance(goal, WaypointGoal):
            raise NavigationContractError(
                "invalid_goal",
                "NavigationEpisode requires WaypointGoal",
            )
        started_at_ms = self.plant.clock_ms()
        snapshot = self.plant.observe(goal)
        reducer = StateReducer(snapshot)
        trace = ["CREATED", "OBSERVING"]
        steps = []
        counters = {
            "ticks": 0,
            "proposals": 0,
            "replans": 0,
            "actions": 0,
            "total_motion_ms": 0,
        }
        best_distance = distance_to_goal_mm(snapshot, goal)
        no_progress_ticks = 0
        stale_dispatch_replans = 0
        stale_retry_granted = False

        try:
            self._publish_observation(snapshot)
        except Exception:
            return self._finish(
                goal,
                NAVIGATION_ABORTED,
                counters,
                trace,
                steps,
                snapshot,
                already_stopped=False,
            )

        while True:
            snapshot = reducer.snapshot()
            current_distance = distance_to_goal_mm(snapshot, goal)
            retrying_stale_dispatch = stale_retry_granted
            stale_retry_granted = False
            if self._cancelled():
                trace.append("CANCELLED")
                return self._finish(
                    goal,
                    NAVIGATION_ABORTED,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )
            if (
                snapshot.touch_pressed
                or snapshot.active_faults
                or self.supervisor.emergency_latched
            ):
                return self._finish(
                    goal,
                    NAVIGATION_SAFETY_STOP,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )
            if (
                self.required_world_model_version is not None
                and snapshot.world_model_version
                != self.required_world_model_version
            ):
                return self._finish(
                    goal,
                    NAVIGATION_PLAN_STALE,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )
            if (
                counters["ticks"] >= self.limits.max_ticks
                or self.plant.clock_ms() < started_at_ms
                or self.plant.clock_ms() - started_at_ms
                >= self.limits.max_elapsed_ms
                or counters["proposals"] >= self.limits.max_proposals
                or (
                    counters["replans"] >= self.limits.max_replans
                    and not retrying_stale_dispatch
                )
                or counters["actions"] >= self.limits.max_actions
                or counters["total_motion_ms"]
                >= self.limits.max_total_motion_ms
            ):
                return self._finish(
                    goal,
                    NAVIGATION_BUDGET_EXHAUSTED,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )
            if current_distance <= goal.tolerance_mm:
                elapsed_ms = self.plant.clock_ms() - started_at_ms
                termination = NAVIGATION_GOAL_REACHED
                if (
                    elapsed_ms + self._terminal_stop_cost_ms()
                    > self.limits.max_elapsed_ms
                ):
                    termination = NAVIGATION_BUDGET_EXHAUSTED
                return self._finish(
                    goal,
                    termination,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )

            trace.append("PUBLISHING")
            try:
                self._publish_local(goal, snapshot)
                tick_directive = None
                if self.before_arbitration is not None:
                    tick_directive = self.before_arbitration(snapshot)
            except Exception:
                return self._finish(
                    goal,
                    NAVIGATION_ABORTED,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )
            if self._cancelled():
                trace.append("CANCELLED")
                return self._finish(
                    goal,
                    NAVIGATION_ABORTED,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )
            proposals = self.inbox.drain()
            counters["proposals"] += len(proposals)
            if counters["proposals"] > self.limits.max_proposals:
                return self._finish(
                    goal,
                    NAVIGATION_BUDGET_EXHAUSTED,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )

            trace.append("ARBITRATING")
            if tick_directive == NAVIGATION_REFRESH_REQUIRED:
                pulse = self.supervisor.force_stop(
                    snapshot,
                    reason_code="post_pause_observation_refresh",
                )
            else:
                pulse = self.supervisor.decide(
                    snapshot,
                    goal,
                    proposals,
                )
            # Goal ownership can change concurrently with arbitration.  A
            # cancellation observed here must revoke the one-shot authority
            # before the already-created pulse reaches the plant.
            if self._cancelled():
                trace.append("CANCELLED_BEFORE_DISPATCH")
                try:
                    self.supervisor.cancel(pulse)
                    termination = NAVIGATION_ABORTED
                except Exception:
                    termination = NAVIGATION_EXECUTION_FAILED
                return self._finish(
                    goal,
                    termination,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )
            if (
                pulse.kind == "DRIVE"
                and (
                    counters["actions"] + 1 > self.limits.max_actions
                    or counters["total_motion_ms"] + pulse.duration_ms
                    > self.limits.max_total_motion_ms
                )
            ):
                try:
                    self.supervisor.cancel(pulse)
                    termination = NAVIGATION_BUDGET_EXHAUSTED
                except Exception:
                    termination = NAVIGATION_EXECUTION_FAILED
                return self._finish(
                    goal,
                    termination,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )
            elapsed_ms = self.plant.clock_ms() - started_at_ms
            dispatch_cost_ms = (
                pulse.duration_ms
                if pulse.kind == "DRIVE"
                else self._terminal_stop_cost_ms()
            )
            terminal_stop_reserve_ms = (
                self._terminal_stop_cost_ms()
                if pulse.kind == "DRIVE"
                else 0
            )
            if (
                elapsed_ms
                + dispatch_cost_ms
                + terminal_stop_reserve_ms
                > self.limits.max_elapsed_ms
            ):
                try:
                    self.supervisor.cancel(pulse)
                    termination = NAVIGATION_BUDGET_EXHAUSTED
                except Exception:
                    termination = NAVIGATION_EXECUTION_FAILED
                return self._finish(
                    goal,
                    termination,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )

            trace.append("EXECUTING")
            before = snapshot
            try:
                after = self.plant.apply(pulse, goal)
                reducer.commit(after)
            except NavigationContractError as error:
                if error.code != "stale_drive_pulse":
                    return self._finish(
                        goal,
                        NAVIGATION_EXECUTION_FAILED,
                        counters,
                        trace,
                        steps,
                        snapshot,
                        already_stopped=False,
                    )
                trace.append("STALE_DISPATCH")
                fresh = None
                try:
                    fresh = self.plant.observe(goal)
                    reducer.commit(fresh)
                    if (
                        self.required_world_model_version is not None
                        and fresh.world_model_version
                        != self.required_world_model_version
                    ):
                        return self._finish(
                            goal,
                            NAVIGATION_PLAN_STALE,
                            counters,
                            trace,
                            steps,
                            fresh,
                            already_stopped=False,
                        )
                    self._publish_observation(fresh)
                except Exception:
                    return self._finish(
                        goal,
                        NAVIGATION_ABORTED,
                        counters,
                        trace,
                        steps,
                        fresh if fresh is not None else snapshot,
                        already_stopped=False,
                    )
                trace.append("STALE_DISPATCH_REOBSERVED")
                if counters["replans"] >= self.limits.max_replans:
                    return self._finish(
                        goal,
                        NAVIGATION_BUDGET_EXHAUSTED,
                        counters,
                        trace,
                        steps,
                        fresh,
                        already_stopped=False,
                    )
                stale_dispatch_replans += 1
                counters["replans"] = (
                    stale_dispatch_replans
                    + max(0, counters["ticks"] - 1)
                )
                stale_retry_granted = True
                trace.append("REPLANNING")
                continue
            except Exception:
                return self._finish(
                    goal,
                    NAVIGATION_EXECUTION_FAILED,
                    counters,
                    trace,
                    steps,
                    snapshot,
                    already_stopped=False,
                )

            counters["ticks"] += 1
            counters["replans"] = (
                stale_dispatch_replans
                + max(0, counters["ticks"] - 1)
            )
            if pulse.kind == "DRIVE":
                counters["actions"] += 1
                counters["total_motion_ms"] += pulse.duration_ms
            trace.append("VERIFYING")
            progress_verified = self._verify_progress(
                before,
                after,
                pulse,
            )
            after_distance = distance_to_goal_mm(after, goal)
            steps.append(
                NavigationStep(
                    tick=counters["ticks"],
                    state_before=before.state_version,
                    state_after=after.state_version,
                    proposal_ids=tuple(sorted(
                        value.proposal.proposal_id
                        for value in proposals
                    )),
                    decision_id=pulse.decision_id,
                    decision_kind=pulse.kind,
                    decision_reason=pulse.reason_code,
                    duration_ms=pulse.duration_ms,
                    distance_before_mm=current_distance,
                    distance_after_mm=after_distance,
                    progress_verified=progress_verified,
                )
            )
            try:
                self._publish_observation(after)
            except Exception:
                # The plant transition is already committed and accounted
                # above.  A downstream observation-delivery failure is a safe
                # orchestration abort, not a reason to erase the action or
                # issue terminal STOP against the stale pre-action snapshot.
                trace.append("OBSERVATION_SINK_FAILED")
                return self._finish(
                    goal,
                    NAVIGATION_ABORTED,
                    counters,
                    trace,
                    steps,
                    after,
                    already_stopped=False,
                )
            if after.touch_pressed or after.active_faults:
                return self._finish(
                    goal,
                    NAVIGATION_SAFETY_STOP,
                    counters,
                    trace,
                    steps,
                    after,
                    already_stopped=(
                        pulse.kind == "STOP"
                        and progress_verified
                        and not after.motors_running
                    ),
                )
            if not progress_verified:
                return self._finish(
                    goal,
                    NAVIGATION_PROGRESS_FAILED,
                    counters,
                    trace,
                    steps,
                    after,
                    already_stopped=False,
                )
            if pulse.kind == "STOP":
                if pulse.reason_code.startswith("abort_"):
                    return self._finish(
                        goal,
                        NAVIGATION_ABORTED,
                        counters,
                        trace,
                        steps,
                        after,
                        already_stopped=True,
                    )
                if (
                    pulse.reason_code
                    != "post_pause_observation_refresh"
                ):
                    no_progress_ticks += 1
            elif (
                after_distance
                <= best_distance - self.limits.minimum_progress_mm
            ):
                best_distance = after_distance
                no_progress_ticks = 0
            else:
                no_progress_ticks += 1
            if no_progress_ticks >= self.limits.max_no_progress_ticks:
                return self._finish(
                    goal,
                    NAVIGATION_PROGRESS_FAILED,
                    counters,
                    trace,
                    steps,
                    after,
                    already_stopped=(
                        pulse.kind == "STOP"
                        and progress_verified
                        and not after.motors_running
                    ),
                )
            trace.append("REPLANNING")
