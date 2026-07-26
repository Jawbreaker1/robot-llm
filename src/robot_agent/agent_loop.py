"""Bounded observe-plan-act-verify loop with no language heuristics."""

from dataclasses import dataclass
import json
import secrets
from typing import Callable, Mapping, Optional, Tuple

from .contract import MotionCommand
from .robot_api import (
    ActionContext,
    ActionReceipt,
    MotionRequest,
    ObservationEnvelope,
    RobotAPI,
    RobotAPIError,
    StopRequest,
)


PROPOSAL_SCHEMA = "robot-agent-decision/v1"
MAX_PROPOSAL_BYTES = 16 * 1024

GOAL_SATISFIED = "GOAL_SATISFIED"
PLANNER_ABORTED = "PLANNER_ABORTED"
INVALID_GOAL = "INVALID_GOAL"
UNSAFE_OBSERVATION = "UNSAFE_OBSERVATION"
PLANNER_FAILED = "PLANNER_FAILED"
EXECUTION_FAILED = "EXECUTION_FAILED"
VERIFICATION_FAILED = "VERIFICATION_FAILED"
BUDGET_EXHAUSTED = "BUDGET_EXHAUSTED"
STOP_FAILED = "STOP_FAILED"


class ProposalError(ValueError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _identifier(name: str, value: str, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise ProposalError(
            "invalid_identifier",
            "{} is invalid".format(name),
        )
    return value


def _integer(name: str, value: int, minimum: int, maximum: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise ProposalError(
            "invalid_integer",
            "{} is invalid".format(name),
        )
    return value


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
class MotorPositionGoal:
    goal_id: str
    instruction: str
    motor_role: str
    target_degrees: int
    tolerance_degrees: int = 2

    def __post_init__(self) -> None:
        _identifier("goal_id", self.goal_id)
        _identifier("motor_role", self.motor_role)
        if (
            not isinstance(self.instruction, str)
            or not self.instruction.strip()
            or len(self.instruction) > 2_000
        ):
            raise ProposalError(
                "invalid_instruction",
                "Goal instruction is invalid",
            )
        _integer(
            "target_degrees",
            self.target_degrees,
            -(2**31),
            2**31 - 1,
        )
        _integer("tolerance_degrees", self.tolerance_degrees, 0, 10_000)

    def position(self, observation: ObservationEnvelope) -> int:
        try:
            position = observation.state.motors[
                self.motor_role
            ].position_degrees
        except KeyError:
            raise ProposalError(
                "missing_goal_motor",
                "Observation does not contain the goal motor",
            ) from None
        return _integer(
            "position_degrees",
            position,
            -(2**63),
            2**63 - 1,
        )

    def error(self, observation: ObservationEnvelope) -> int:
        return abs(self.target_degrees - self.position(observation))

    def satisfied(self, observation: ObservationEnvelope) -> bool:
        return self.error(observation) <= self.tolerance_degrees

    def to_dict(self) -> Mapping[str, object]:
        return {
            "goal_id": self.goal_id,
            "instruction": self.instruction,
            "criterion": {
                "kind": "motor_position",
                "motor_role": self.motor_role,
                "target_degrees": self.target_degrees,
                "tolerance_degrees": self.tolerance_degrees,
            },
        }


@dataclass(frozen=True)
class MotionIntent:
    motor_role: str
    speed_dps: int
    duration_ms: int


@dataclass(frozen=True)
class DecisionProposal:
    proposal_id: str
    goal_id: str
    based_on_state_version: int
    decision: str
    action: Optional[MotionIntent] = None
    abort_code: Optional[str] = None


def decode_decision_proposal(raw: bytes) -> DecisionProposal:
    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > MAX_PROPOSAL_BYTES
    ):
        raise ProposalError(
            "invalid_proposal_body",
            "Planner proposal body is invalid",
        )
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, TypeError, ValueError):
        raise ProposalError(
            "invalid_proposal_json",
            "Planner returned invalid JSON",
        ) from None
    if not isinstance(value, dict):
        raise ProposalError(
            "invalid_proposal_shape",
            "Planner proposal must be an object",
        )
    common = {
        "schema",
        "proposal_id",
        "goal_id",
        "based_on_state_version",
        "decision",
    }
    decision = value.get("decision")
    if decision == "ACT":
        if set(value) != common | {"action"}:
            raise ProposalError(
                "invalid_proposal_fields",
                "ACT proposal fields are invalid",
            )
        action = value["action"]
        if (
            not isinstance(action, dict)
            or set(action)
            != {"type", "motor_role", "speed_dps", "duration_ms"}
            or action.get("type") != "MOVE_MOTOR"
        ):
            raise ProposalError(
                "invalid_action",
                "Planner action is invalid",
            )
        intent = MotionIntent(
            motor_role=_identifier("motor_role", action["motor_role"]),
            speed_dps=_integer(
                "speed_dps",
                action["speed_dps"],
                -100_000,
                100_000,
            ),
            duration_ms=_integer(
                "duration_ms",
                action["duration_ms"],
                1,
                60_000,
            ),
        )
        if intent.speed_dps == 0:
            raise ProposalError(
                "zero_speed",
                "Planner must use ABORT instead of zero speed",
            )
        abort_code = None
    elif decision == "ABORT":
        if set(value) != common | {"abort_code"}:
            raise ProposalError(
                "invalid_proposal_fields",
                "ABORT proposal fields are invalid",
            )
        intent = None
        abort_code = _identifier(
            "abort_code",
            value["abort_code"],
            64,
        )
    else:
        raise ProposalError(
            "invalid_decision",
            "Proposal decision must be ACT or ABORT",
        )
    if value["schema"] != PROPOSAL_SCHEMA:
        raise ProposalError(
            "invalid_proposal_schema",
            "Planner proposal schema is not supported",
        )
    return DecisionProposal(
        proposal_id=_identifier("proposal_id", value["proposal_id"]),
        goal_id=_identifier("goal_id", value["goal_id"]),
        based_on_state_version=_integer(
            "based_on_state_version",
            value["based_on_state_version"],
            1,
            2**63 - 1,
        ),
        decision=decision,
        action=intent,
        abort_code=abort_code,
    )


@dataclass(frozen=True)
class PlannerFeedback:
    code: str
    state_version: int
    action_id: Optional[str] = None

    def to_dict(self) -> Mapping[str, object]:
        return {
            "code": self.code,
            "state_version": self.state_version,
            "action_id": self.action_id,
        }


@dataclass(frozen=True)
class PlanningContext:
    goal: MotorPositionGoal
    observation: ObservationEnvelope
    previous_feedback: Optional[PlannerFeedback]
    planner_turn: int
    remaining_actions: int
    remaining_motion_ms: int

    def to_dict(self) -> Mapping[str, object]:
        return {
            "goal": self.goal.to_dict(),
            "observation": self.observation.to_dict(),
            "previous_feedback": (
                None
                if self.previous_feedback is None
                else self.previous_feedback.to_dict()
            ),
            "planner_turn": self.planner_turn,
            "remaining_actions": self.remaining_actions,
            "remaining_motion_ms": self.remaining_motion_ms,
        }


@dataclass(frozen=True)
class LoopLimits:
    max_elapsed_ms: int = 5_000
    max_planner_latency_ms: int = 1_000
    max_planner_turns: int = 6
    max_actions: int = 4
    max_replans: int = 4
    max_total_motion_ms: int = 1_600
    action_ttl_ms: int = 500
    max_observation_age_ms: int = 500
    terminal_stop_reserve_ms: int = 100

    def __post_init__(self) -> None:
        _integer("max_elapsed_ms", self.max_elapsed_ms, 1, 300_000)
        _integer(
            "max_planner_latency_ms",
            self.max_planner_latency_ms,
            1,
            self.max_elapsed_ms,
        )
        _integer("max_planner_turns", self.max_planner_turns, 1, 100)
        _integer("max_actions", self.max_actions, 1, 100)
        _integer("max_replans", self.max_replans, 0, 100)
        _integer(
            "max_total_motion_ms",
            self.max_total_motion_ms,
            1,
            300_000,
        )
        _integer("action_ttl_ms", self.action_ttl_ms, 1, 1_000)
        _integer(
            "max_observation_age_ms",
            self.max_observation_age_ms,
            1,
            60_000,
        )
        _integer(
            "terminal_stop_reserve_ms",
            self.terminal_stop_reserve_ms,
            1,
            self.max_elapsed_ms - 1,
        )


@dataclass(frozen=True)
class EpisodeStep:
    proposal_id: Optional[str]
    outcome_code: str
    action_id: Optional[str]
    state_before: int
    state_after: int
    error_before: int
    error_after: int


@dataclass(frozen=True)
class EpisodeResult:
    goal_id: str
    completed: bool
    termination: str
    planner_turns: int
    actions: int
    replans: int
    total_motion_ms: int
    final_observation: Optional[ObservationEnvelope]
    trace: Tuple[str, ...]
    steps: Tuple[EpisodeStep, ...]

    def to_dict(self) -> Mapping[str, object]:
        return {
            "goal_id": self.goal_id,
            "completed": self.completed,
            "termination": self.termination,
            "planner_turns": self.planner_turns,
            "actions": self.actions,
            "replans": self.replans,
            "total_motion_ms": self.total_motion_ms,
            "final_observation": (
                None
                if self.final_observation is None
                else self.final_observation.to_dict()
            ),
            "trace": list(self.trace),
            "steps": [
                {
                    "proposal_id": step.proposal_id,
                    "outcome_code": step.outcome_code,
                    "action_id": step.action_id,
                    "state_before": step.state_before,
                    "state_after": step.state_after,
                    "error_before": step.error_before,
                    "error_after": step.error_after,
                }
                for step in self.steps
            ],
        }


Planner = Callable[[PlanningContext], bytes]
Clock = Callable[[], int]
IDFactory = Callable[[], str]


class ClosedLoopAgent:
    def __init__(
        self,
        robot: RobotAPI,
        planner: Planner,
        clock_ms: Clock,
        source_id: str = "local-goal-agent",
        limits: LoopLimits = LoopLimits(),
        id_factory: IDFactory = lambda: secrets.token_hex(8),
    ):
        if not isinstance(robot, RobotAPI):
            raise ProposalError(
                "invalid_robot_api",
                "robot must implement RobotAPI",
            )
        if not callable(planner) or not callable(clock_ms):
            raise ProposalError(
                "invalid_dependency",
                "Planner and clock must be callable",
            )
        _identifier("source_id", source_id)
        if not isinstance(limits, LoopLimits) or not callable(id_factory):
            raise ProposalError(
                "invalid_dependency",
                "Loop limits or ID factory are invalid",
            )
        self._robot = robot
        self._planner = planner
        self._clock_ms = clock_ms
        self._source_id = source_id
        self._limits = limits
        self._id_factory = id_factory

    def _now_ms(self) -> int:
        return _integer("clock_ms", self._clock_ms(), 0, 2**63 - 1)

    @staticmethod
    def _safe_observation(observation: ObservationEnvelope) -> bool:
        if (
            observation.state.active_faults
            or any(
                type(motor.running) is not bool or motor.running
                for motor in observation.state.motors.values()
            )
        ):
            return False
        if "touch" not in observation.state.sensors:
            return False
        touch = observation.state.sensors["touch"]
        return touch is False or type(touch) is int and touch == 0

    @staticmethod
    def _same_controller(
        capabilities,
        observation: ObservationEnvelope,
    ) -> bool:
        return (
            observation.robot_id == capabilities.robot_id
            and observation.controller_id == capabilities.controller_id
            and observation.controller_instance_id
            == capabilities.controller_instance_id
            and observation.host_clock_id == capabilities.host_clock_id
        )

    def _deadline_now(
        self,
        started_at_ms: int,
        required_motion_ms: int = 0,
    ):
        now_ms = self._now_ms()
        exhausted = (
            now_ms < started_at_ms
            or now_ms
            - started_at_ms
            + required_motion_ms
            + self._limits.terminal_stop_reserve_ms
            >= self._limits.max_elapsed_ms
        )
        return now_ms, exhausted

    def _fresh_observation(
        self,
        observation: ObservationEnvelope,
    ) -> bool:
        now_ms = self._now_ms()
        return (
            now_ms >= observation.received_at_host_ms
            and now_ms - observation.received_at_host_ms
            <= self._limits.max_observation_age_ms
        )

    def _stop_request(self, episode_id, capabilities):
        stop_id = "{}-stop".format(episode_id)
        return StopRequest(
            robot_id=capabilities.robot_id,
            controller_id=capabilities.controller_id,
            controller_instance_id=(
                capabilities.controller_instance_id
            ),
            action_id=stop_id,
            segment_id=stop_id,
            source_id=self._source_id,
        )

    def _best_effort_terminal_stop(
        self,
        episode_id,
        capabilities,
    ) -> None:
        try:
            if capabilities is None:
                capabilities = self._robot.capabilities()
            self._stop_and_verify(
                episode_id,
                capabilities,
                previous=None,
            )
            return
        except BaseException:
            pass
        try:
            request = self._stop_request(episode_id, capabilities)
        except BaseException:
            return
        for _ in range(2):
            try:
                self._robot.stop_all(request)
                return
            except Exception:
                pass

    def _stop_and_verify(
        self,
        episode_id,
        capabilities,
        previous,
    ) -> ObservationEnvelope:
        request = self._stop_request(episode_id, capabilities)
        last_error = None
        for _ in range(2):
            try:
                receipt = self._robot.stop_all(request)
                final = self._robot.observe()
                if (
                    not isinstance(receipt, ActionReceipt)
                    or receipt.action_id != request.action_id
                    or receipt.segment_id != request.segment_id
                    or receipt.controller_id != request.controller_id
                    or receipt.controller_instance_id
                    != request.controller_instance_id
                    or receipt.status != "stopped"
                    or receipt.resulting_state_version
                    <= receipt.based_on_state_version
                    or final.state_version
                    != receipt.resulting_state_version
                    or not self._same_controller(capabilities, final)
                    or final.state.active_faults
                    or any(
                        motor.running
                        for motor in final.state.motors.values()
                    )
                    or not self._fresh_observation(final)
                    or previous is not None
                    and (
                        receipt.based_on_state_version
                        < previous.state_version
                        or final.received_at_host_ms
                        < previous.received_at_host_ms
                    )
                ):
                    raise RobotAPIError(
                        "stop_not_verified",
                        "Terminal stop receipt or observation did not match",
                    )
                return final
            except Exception as error:
                last_error = error
        raise RobotAPIError(
            "stop_not_verified",
            "Terminal stop failed after two attempts: {}".format(
                type(last_error).__name__
            ),
        )

    def _finish(
        self,
        goal: MotorPositionGoal,
        episode_id: str,
        termination: str,
        counters: Mapping[str, int],
        trace: list,
        steps: list,
        observation: Optional[ObservationEnvelope],
        capabilities,
    ) -> EpisodeResult:
        trace.append("STOPPING")
        final = observation
        try:
            if capabilities is None:
                capabilities = self._robot.capabilities()
            final = self._stop_and_verify(
                episode_id,
                capabilities,
                observation,
            )
            if (
                termination == GOAL_SATISFIED
                and not self._safe_observation(final)
            ):
                termination = STOP_FAILED
        except Exception:
            termination = STOP_FAILED
        try:
            completed = (
                termination == GOAL_SATISFIED
                and final is not None
                and goal.satisfied(final)
            )
        except ProposalError:
            termination = STOP_FAILED
            completed = False
        trace.append(
            "SUCCEEDED"
            if completed
            else "ABORTED"
            if termination == PLANNER_ABORTED
            else "FAILED"
        )
        return EpisodeResult(
            goal_id=goal.goal_id,
            completed=completed,
            termination=termination,
            planner_turns=counters["planner_turns"],
            actions=counters["actions"],
            replans=counters["replans"],
            total_motion_ms=counters["total_motion_ms"],
            final_observation=final,
            trace=tuple(trace),
            steps=tuple(steps),
        )

    def run(self, goal: MotorPositionGoal) -> EpisodeResult:
        if not isinstance(goal, MotorPositionGoal):
            raise ProposalError(
                "invalid_goal",
                "ClosedLoopAgent requires MotorPositionGoal",
            )
        episode_id = _identifier("episode_id", self._id_factory(), 48)
        controller_ref = []
        try:
            return self._run_episode(goal, episode_id, controller_ref)
        except BaseException:
            self._best_effort_terminal_stop(
                episode_id,
                None if not controller_ref else controller_ref[0],
            )
            raise

    def _run_episode(
        self,
        goal: MotorPositionGoal,
        episode_id: str,
        controller_ref: list,
    ) -> EpisodeResult:
        started_at_ms = self._now_ms()
        counters = {
            "planner_turns": 0,
            "actions": 0,
            "replans": 0,
            "total_motion_ms": 0,
        }
        trace = ["CREATED"]
        steps = []
        seen_proposals = set()
        feedback = None
        observation = None

        try:
            capabilities = self._robot.capabilities()
            controller_ref.append(capabilities)
            capabilities.motor(goal.motor_role)
            trace.append("OBSERVING")
            observation = self._robot.observe()
            if not self._same_controller(capabilities, observation):
                raise RobotAPIError(
                    "identity_mismatch",
                    "Robot identity changed",
                )
            goal.position(observation)
        except (ProposalError, RobotAPIError):
            return self._finish(
                goal,
                episode_id,
                INVALID_GOAL,
                counters,
                trace,
                steps,
                observation,
                capabilities if controller_ref else None,
            )

        while True:
            if (
                not self._same_controller(capabilities, observation)
                or not self._safe_observation(observation)
            ):
                return self._finish(
                    goal,
                    episode_id,
                    UNSAFE_OBSERVATION,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )
            _, deadline_exhausted = self._deadline_now(started_at_ms)
            if (
                deadline_exhausted
                or counters["planner_turns"]
                >= self._limits.max_planner_turns
                or counters["actions"] >= self._limits.max_actions
            ):
                return self._finish(
                    goal,
                    episode_id,
                    BUDGET_EXHAUSTED,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )
            if goal.satisfied(observation):
                return self._finish(
                    goal,
                    episode_id,
                    GOAL_SATISFIED,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )

            trace.append("PLANNING")
            context = PlanningContext(
                goal=goal,
                observation=observation,
                previous_feedback=feedback,
                planner_turn=counters["planner_turns"] + 1,
                remaining_actions=(
                    self._limits.max_actions - counters["actions"]
                ),
                remaining_motion_ms=(
                    self._limits.max_total_motion_ms
                    - counters["total_motion_ms"]
                ),
            )
            planner_started = self._now_ms()
            try:
                raw = self._planner(context)
            except BaseException:
                counters["planner_turns"] += 1
                return self._finish(
                    goal,
                    episode_id,
                    PLANNER_FAILED,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )
            planner_completed = self._now_ms()
            counters["planner_turns"] += 1
            if (
                planner_completed < planner_started
                or planner_completed - planner_started
                >= self._limits.max_planner_latency_ms
            ):
                return self._finish(
                    goal,
                    episode_id,
                    PLANNER_FAILED,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )
            _, deadline_exhausted = self._deadline_now(started_at_ms)
            if deadline_exhausted:
                return self._finish(
                    goal,
                    episode_id,
                    BUDGET_EXHAUSTED,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )

            proposal = None
            try:
                proposal = decode_decision_proposal(raw)
                if proposal.proposal_id in seen_proposals:
                    raise ProposalError(
                        "duplicate_proposal",
                        "Proposal ID was replayed",
                    )
                seen_proposals.add(proposal.proposal_id)
                if proposal.goal_id != goal.goal_id:
                    raise ProposalError(
                        "wrong_goal",
                        "Proposal referenced another goal",
                    )
                if (
                    proposal.based_on_state_version
                    != observation.state_version
                ):
                    raise ProposalError(
                        "stale_state",
                        "Proposal referenced another observation",
                    )
            except ProposalError as error:
                result = self._reject_and_reobserve(
                    goal,
                    episode_id,
                    error.code,
                    None if proposal is None else proposal.proposal_id,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )
                if isinstance(result, EpisodeResult):
                    return result
                observation, feedback = result
                continue

            if proposal.decision == "ABORT":
                return self._finish(
                    goal,
                    episode_id,
                    PLANNER_ABORTED,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )

            trace.append("AUTHORIZING")
            intent = proposal.action
            try:
                current = self._robot.observe()
                if (
                    not self._same_controller(capabilities, current)
                    or current.state_version != observation.state_version
                ):
                    raise ProposalError(
                        "stale_state",
                        "Robot changed while planner was running",
                    )
                _, deadline_exhausted = self._deadline_now(
                    started_at_ms
                )
                if deadline_exhausted:
                    return self._finish(
                        goal,
                        episode_id,
                        BUDGET_EXHAUSTED,
                        counters,
                        trace,
                        steps,
                        current,
                        capabilities,
                    )
                if intent.motor_role != goal.motor_role:
                    raise ProposalError(
                        "wrong_motor",
                        "Proposal targeted another motor",
                    )
                direction_to_goal = (
                    goal.target_degrees - goal.position(current)
                )
                if direction_to_goal * intent.speed_dps <= 0:
                    raise ProposalError(
                        "wrong_direction",
                        "Proposed direction cannot reduce goal error",
                    )
                motor_capability = capabilities.motor(intent.motor_role)
                if not motor_capability.gate.executable:
                    raise ProposalError(
                        "capability_unavailable",
                        "Motor capability is unavailable",
                    )
                if abs(intent.speed_dps) > motor_capability.max_abs_speed_dps:
                    raise ProposalError(
                        "speed_limit",
                        "Proposed speed exceeds capability",
                    )
                if intent.duration_ms > motor_capability.max_duration_ms:
                    raise ProposalError(
                        "duration_limit",
                        "Proposed duration exceeds capability",
                    )
                if (
                    counters["total_motion_ms"] + intent.duration_ms
                    > self._limits.max_total_motion_ms
                ):
                    raise ProposalError(
                        "motion_budget",
                        "Proposal exceeds episode motion budget",
                    )
            except (ProposalError, RobotAPIError) as error:
                result = self._reject_and_reobserve(
                    goal,
                    episode_id,
                    error.code,
                    proposal.proposal_id,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )
                if isinstance(result, EpisodeResult):
                    return result
                observation, feedback = result
                continue

            observation = current
            action_number = counters["actions"] + 1
            action_id = "{}-action-{}".format(episode_id, action_number)
            segment_id = "{}-segment-{}".format(
                episode_id,
                action_number,
            )
            issued_at_ms, deadline_exhausted = self._deadline_now(
                started_at_ms,
                required_motion_ms=intent.duration_ms,
            )
            if deadline_exhausted:
                return self._finish(
                    goal,
                    episode_id,
                    BUDGET_EXHAUSTED,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )
            episode_action_deadline_ms = (
                started_at_ms
                + self._limits.max_elapsed_ms
                - self._limits.terminal_stop_reserve_ms
            )
            request = MotionRequest(
                context=ActionContext(
                    robot_id=capabilities.robot_id,
                    controller_id=capabilities.controller_id,
                    controller_instance_id=(
                        capabilities.controller_instance_id
                    ),
                    action_id=action_id,
                    segment_id=segment_id,
                    source_id=self._source_id,
                    host_clock_id=capabilities.host_clock_id,
                    based_on_state_version=observation.state_version,
                    based_on_received_at_host_ms=(
                        observation.received_at_host_ms
                    ),
                    issued_at_host_ms=issued_at_ms,
                    valid_until_host_ms=min(
                        issued_at_ms + self._limits.action_ttl_ms,
                        episode_action_deadline_ms,
                    ),
                ),
                command=MotionCommand(
                    command_id=segment_id,
                    motor_role=intent.motor_role,
                    speed_dps=intent.speed_dps,
                    duration_ms=intent.duration_ms,
                    issued_at_ms=issued_at_ms,
                ),
            )
            error_before = goal.error(observation)
            counters["actions"] += 1
            counters["total_motion_ms"] += intent.duration_ms
            trace.append("EXECUTING")
            try:
                receipt = self._robot.execute_motion(request)
            except RobotAPIError:
                return self._finish(
                    goal,
                    episode_id,
                    EXECUTION_FAILED,
                    counters,
                    trace,
                    steps,
                    observation,
                    capabilities,
                )

            trace.extend(("OBSERVING", "VERIFYING"))
            post = observation
            try:
                post = self._robot.observe()
                if (
                    not self._same_controller(capabilities, post)
                    or not self._safe_observation(post)
                ):
                    raise RobotAPIError(
                        "unsafe_post_action_observation",
                        "Post-action observation was unsafe",
                    )
                self._verify_receipt(request, receipt, observation, post)
                error_after = goal.error(post)
            except (ProposalError, RobotAPIError):
                return self._finish(
                    goal,
                    episode_id,
                    VERIFICATION_FAILED,
                    counters,
                    trace,
                    steps,
                    post,
                    capabilities,
                )
            steps.append(
                EpisodeStep(
                    proposal_id=proposal.proposal_id,
                    outcome_code="action_completed",
                    action_id=action_id,
                    state_before=observation.state_version,
                    state_after=post.state_version,
                    error_before=error_before,
                    error_after=error_after,
                )
            )
            _, deadline_exhausted = self._deadline_now(started_at_ms)
            if deadline_exhausted:
                return self._finish(
                    goal,
                    episode_id,
                    BUDGET_EXHAUSTED,
                    counters,
                    trace,
                    steps,
                    post,
                    capabilities,
                )
            if not goal.satisfied(post) and error_after >= error_before:
                return self._finish(
                    goal,
                    episode_id,
                    VERIFICATION_FAILED,
                    counters,
                    trace,
                    steps,
                    post,
                    capabilities,
                )
            if goal.satisfied(post):
                observation = post
                continue
            if counters["replans"] >= self._limits.max_replans:
                return self._finish(
                    goal,
                    episode_id,
                    BUDGET_EXHAUSTED,
                    counters,
                    trace,
                    steps,
                    post,
                    capabilities,
                )
            counters["replans"] += 1
            feedback = PlannerFeedback(
                code="verified_progress",
                state_version=post.state_version,
                action_id=action_id,
            )
            trace.extend(("REPLANNING", "OBSERVING"))
            try:
                observation = self._robot.observe()
            except RobotAPIError:
                return self._finish(
                    goal,
                    episode_id,
                    EXECUTION_FAILED,
                    counters,
                    trace,
                    steps,
                    post,
                    capabilities,
                )

    def _reject_and_reobserve(
        self,
        goal,
        episode_id,
        code,
        proposal_id,
        counters,
        trace,
        steps,
        observation,
        capabilities,
    ):
        error = goal.error(observation)
        steps.append(
            EpisodeStep(
                proposal_id=proposal_id,
                outcome_code=code,
                action_id=None,
                state_before=observation.state_version,
                state_after=observation.state_version,
                error_before=error,
                error_after=error,
            )
        )
        if counters["replans"] >= self._limits.max_replans:
            return self._finish(
                goal,
                episode_id,
                BUDGET_EXHAUSTED,
                counters,
                trace,
                steps,
                observation,
                capabilities,
            )
        counters["replans"] += 1
        feedback = PlannerFeedback(code, observation.state_version)
        trace.extend(("REPLANNING", "OBSERVING"))
        try:
            next_observation = self._robot.observe()
        except RobotAPIError:
            return self._finish(
                goal,
                episode_id,
                EXECUTION_FAILED,
                counters,
                trace,
                steps,
                observation,
                capabilities,
            )
        return next_observation, feedback

    @staticmethod
    def _verify_receipt(
        request: MotionRequest,
        receipt: ActionReceipt,
        before: ObservationEnvelope,
        after: ObservationEnvelope,
    ) -> None:
        if (
            receipt.action_id != request.context.action_id
            or receipt.segment_id != request.context.segment_id
            or receipt.controller_id != request.context.controller_id
            or receipt.controller_instance_id
            != request.context.controller_instance_id
            or receipt.status != "completed"
            or receipt.based_on_state_version != before.state_version
            or receipt.resulting_state_version != after.state_version
            or after.state_version <= before.state_version
            or after.state.active_faults
            or any(motor.running for motor in after.state.motors.values())
        ):
            raise RobotAPIError(
                "receipt_mismatch",
                "Action receipt did not match the observed state",
            )
        before_position = before.state.motors[
            request.command.motor_role
        ].position_degrees
        after_position = after.state.motors[
            request.command.motor_role
        ].position_degrees
        if (
            receipt.position_before != before_position
            or receipt.position_after != after_position
        ):
            raise RobotAPIError(
                "position_mismatch",
                "Receipt positions did not match observations",
            )
