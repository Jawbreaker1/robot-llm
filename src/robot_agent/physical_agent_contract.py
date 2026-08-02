"""Frozen contracts for one canonical physical robot controller state.

The module deliberately contains no transport, motor, sensor, map, model, or
dashboard imports.  Planners and evaluators may construct these immutable
events, while ``physical_agent_state`` owns their legal transitions.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional, Tuple, Union


MAX_INT = 2**63 - 1
MAX_PLANNING_TICKET_TTL_MS = 5 * 60_000
MAX_STEP_COMMAND_START_TTL_MS = 60_000
MAX_STEP_COMMAND_SETTLE_MS = 30 * 60_000
PRIMITIVE_ACTIONS = frozenset(
    ("ADVANCE", "REVERSE", "TURN_LEFT_90", "TURN_RIGHT_90")
)
SENSOR_OPERATIONS = frozenset(("OBSERVE", "SCAN_FRONT_ARC"))


class PhysicalAgentStateError(ValueError):
    """A canonical state value or transition violated the contract."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


class AgentPhase(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    STOPPING = "STOPPING"
    TERMINAL = "TERMINAL"


class GoalOutcome(str, Enum):
    SUCCEEDED = "SUCCEEDED"
    CANCELLED = "CANCELLED"
    FAILED = "FAILED"


class PlanningCause(str, Enum):
    NEW_GOAL = "NEW_GOAL"
    UNCERTAINTY = "UNCERTAINTY"
    REPLAN_REQUIRED = "REPLAN_REQUIRED"


class DetourSide(str, Enum):
    LEFT_OF_GOAL = "LEFT_OF_GOAL"
    RIGHT_OF_GOAL = "RIGHT_OF_GOAL"


class ReceiptOutcome(str, Enum):
    COMPLETED = "COMPLETED"
    REJECTED_NOT_STARTED = "REJECTED_NOT_STARTED"
    STOPPED = "STOPPED"


class StepDisposition(str, Enum):
    CONTINUE = "CONTINUE"
    COMPLETE = "COMPLETE"
    BLOCKED = "BLOCKED"


def _identifier(name: str, value: object, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise PhysicalAgentStateError(
            "invalid_{}".format(name),
            "{} is invalid".format(name.replace("_", " ")),
        )
    return value


def _text(name: str, value: object, maximum: int) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(
            ord(character) < 32 and character not in "\n\r\t"
            for character in value
        )
    ):
        raise PhysicalAgentStateError(
            "invalid_{}".format(name),
            "{} is invalid".format(name.replace("_", " ")),
        )
    return value


def _integer(
    name: str,
    value: object,
    minimum: int = 0,
    maximum: int = MAX_INT,
) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise PhysicalAgentStateError(
            "invalid_{}".format(name),
            "{} is invalid".format(name.replace("_", " ")),
        )
    return value


@dataclass(frozen=True)
class ControllerKey:
    robot_id: str
    controller_id: str
    controller_instance_id: str

    def __post_init__(self) -> None:
        _identifier("robot_id", self.robot_id)
        _identifier("controller_id", self.controller_id)
        _identifier("controller_instance_id", self.controller_instance_id)


@dataclass(frozen=True)
class NavigationBasis:
    """Decision-relevant identity and version vector.

    ``navigation_basis_id`` is a host-derived digest of the evidence that can
    affect navigation.  Raw world/controller versions remain available for
    audit and monotonicity, while irrelevant camera or audio updates may keep
    the same digest and therefore need not starve a slow planner response.
    """

    controller_key: ControllerKey
    goal_epoch: int
    controller_state_version: int
    world_generation_id: str
    world_model_version: int
    navigation_basis_id: str
    frame_id: str
    calibration_fingerprint: str

    def __post_init__(self) -> None:
        if not isinstance(self.controller_key, ControllerKey):
            raise PhysicalAgentStateError(
                "invalid_controller_key",
                "navigation basis controller key is invalid",
            )
        _integer("goal_epoch", self.goal_epoch, 1)
        _integer("controller_state_version", self.controller_state_version, 1)
        _identifier("world_generation_id", self.world_generation_id)
        _integer("world_model_version", self.world_model_version, 1)
        _identifier("navigation_basis_id", self.navigation_basis_id)
        _identifier("frame_id", self.frame_id)
        _identifier(
            "calibration_fingerprint",
            self.calibration_fingerprint,
            256,
        )

    def decision_equivalent(self, other: object) -> bool:
        """Whether two snapshots describe the same planning evidence."""

        return isinstance(other, NavigationBasis) and (
            self.controller_key == other.controller_key
            and self.goal_epoch == other.goal_epoch
            and self.world_generation_id == other.world_generation_id
            and self.navigation_basis_id == other.navigation_basis_id
            and self.frame_id == other.frame_id
            and self.calibration_fingerprint
            == other.calibration_fingerprint
        )

    def assert_successor_of(self, previous: "NavigationBasis") -> None:
        if not isinstance(previous, NavigationBasis):
            raise PhysicalAgentStateError(
                "invalid_previous_basis",
                "previous navigation basis is invalid",
            )
        if (
            self.controller_key != previous.controller_key
            or self.goal_epoch != previous.goal_epoch
            or self.world_generation_id != previous.world_generation_id
            or self.frame_id != previous.frame_id
            or self.calibration_fingerprint
            != previous.calibration_fingerprint
        ):
            raise PhysicalAgentStateError(
                "navigation_basis_identity_changed",
                "navigation basis identity, frame, generation, or calibration changed",
            )
        if (
            self.controller_state_version
            < previous.controller_state_version
            or self.world_model_version < previous.world_model_version
        ):
            raise PhysicalAgentStateError(
                "stale_navigation_basis",
                "navigation basis versions regressed",
            )


@dataclass(frozen=True)
class GoalAssignment:
    goal_id: str
    goal_epoch: int
    objective: str
    source: str
    locale: str
    activated_at_ms: int

    def __post_init__(self) -> None:
        _identifier("goal_id", self.goal_id)
        _integer("goal_epoch", self.goal_epoch, 1)
        _text("goal_objective", self.objective, 2_000)
        _identifier("goal_source", self.source)
        _identifier("goal_locale", self.locale, 64)
        _integer("goal_activated_at_ms", self.activated_at_ms)


@dataclass(frozen=True)
class FollowDirectionIntent:
    """Continue the currently assigned directional goal."""


@dataclass(frozen=True)
class ScanTargetIntent:
    target_hypothesis_id: str
    scan_profile_id: str

    def __post_init__(self) -> None:
        _identifier("target_hypothesis_id", self.target_hypothesis_id)
        _identifier("scan_profile_id", self.scan_profile_id)


@dataclass(frozen=True)
class DetourTargetIntent:
    target_hypothesis_id: str
    detour_side: DetourSide

    def __post_init__(self) -> None:
        _identifier("target_hypothesis_id", self.target_hypothesis_id)
        if not isinstance(self.detour_side, DetourSide):
            raise PhysicalAgentStateError(
                "invalid_detour_side",
                "detour side is invalid",
            )


IntentPayload = Union[
    FollowDirectionIntent,
    ScanTargetIntent,
    DetourTargetIntent,
]


@dataclass(frozen=True)
class IntentPolicy:
    """Host-owned finite budget for one persistent intent revision."""

    max_plan_attempts: int = 16
    max_consecutive_no_progress_plans: int = 3

    def __post_init__(self) -> None:
        _integer("max_plan_attempts", self.max_plan_attempts, 1, 1_000)
        _integer(
            "max_consecutive_no_progress_plans",
            self.max_consecutive_no_progress_plans,
            0,
            1_000,
        )


@dataclass(frozen=True)
class IntentProgress:
    """Host-observed progress; it is not authored by the planner."""

    plan_attempts: int
    completed_steps: int
    completed_steps_at_plan_start: int
    consecutive_no_progress_plans: int
    last_progress_basis_id: str

    def __post_init__(self) -> None:
        _integer("intent_plan_attempts", self.plan_attempts, 1)
        _integer("intent_completed_steps", self.completed_steps)
        _integer(
            "intent_completed_steps_at_plan_start",
            self.completed_steps_at_plan_start,
            0,
            self.completed_steps,
        )
        _integer(
            "intent_consecutive_no_progress_plans",
            self.consecutive_no_progress_plans,
        )
        _identifier("intent_last_progress_basis_id", self.last_progress_basis_id)


@dataclass(frozen=True)
class ActiveIntent:
    intent_id: str
    revision: int
    goal_id: str
    goal_epoch: int
    payload: IntentPayload
    accepted_basis: NavigationBasis
    accepted_at_ms: int
    policy: IntentPolicy = IntentPolicy()

    def __post_init__(self) -> None:
        _identifier("intent_id", self.intent_id)
        _integer("intent_revision", self.revision, 1)
        _identifier("intent_goal_id", self.goal_id)
        _integer("intent_goal_epoch", self.goal_epoch, 1)
        if not isinstance(
            self.payload,
            (FollowDirectionIntent, ScanTargetIntent, DetourTargetIntent),
        ):
            raise PhysicalAgentStateError(
                "invalid_intent_payload",
                "intent payload is invalid",
            )
        if not isinstance(self.accepted_basis, NavigationBasis):
            raise PhysicalAgentStateError(
                "invalid_intent_basis",
                "intent basis is invalid",
            )
        if self.accepted_basis.goal_epoch != self.goal_epoch:
            raise PhysicalAgentStateError(
                "intent_basis_mismatch",
                "intent and navigation basis goal epochs differ",
            )
        _integer("intent_accepted_at_ms", self.accepted_at_ms)
        if not isinstance(self.policy, IntentPolicy):
            raise PhysicalAgentStateError(
                "invalid_intent_policy", "intent policy is invalid"
            )


@dataclass(frozen=True)
class PlanBinding:
    controller_key: ControllerKey
    goal_id: str
    goal_epoch: int
    intent_id: str
    intent_revision: int
    frame_id: str
    world_generation_id: str
    calibration_fingerprint: str
    based_on_navigation_basis_id: str
    target_geometry_signatures: Tuple[Tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.controller_key, ControllerKey):
            raise PhysicalAgentStateError(
                "invalid_plan_controller_key",
                "plan controller key is invalid",
            )
        _identifier("plan_goal_id", self.goal_id)
        _integer("plan_goal_epoch", self.goal_epoch, 1)
        _identifier("plan_intent_id", self.intent_id)
        _integer("plan_intent_revision", self.intent_revision, 1)
        _identifier("plan_frame_id", self.frame_id)
        _identifier("plan_world_generation_id", self.world_generation_id)
        _identifier(
            "plan_calibration_fingerprint",
            self.calibration_fingerprint,
            256,
        )
        _identifier(
            "plan_navigation_basis_id",
            self.based_on_navigation_basis_id,
        )
        if (
            not isinstance(self.target_geometry_signatures, tuple)
            or any(
                not isinstance(value, tuple)
                or len(value) != 2
                or _identifier("target_id", value[0]) != value[0]
                or _identifier("target_geometry_signature", value[1], 256)
                != value[1]
                for value in self.target_geometry_signatures
            )
            or tuple(sorted(set(self.target_geometry_signatures)))
            != self.target_geometry_signatures
        ):
            raise PhysicalAgentStateError(
                "invalid_target_geometry_signatures",
                "target geometry signatures must be sorted and unique",
            )

    def assert_matches(
        self,
        *,
        controller_key: ControllerKey,
        goal: GoalAssignment,
        intent: ActiveIntent,
        basis: NavigationBasis,
    ) -> None:
        if (
            self.controller_key != controller_key
            or self.goal_id != goal.goal_id
            or self.goal_epoch != goal.goal_epoch
            or self.intent_id != intent.intent_id
            or self.intent_revision != intent.revision
            or self.frame_id != basis.frame_id
            or self.world_generation_id != basis.world_generation_id
            or self.calibration_fingerprint
            != basis.calibration_fingerprint
        ):
            raise PhysicalAgentStateError(
                "plan_binding_mismatch",
                "plan is not bound to the current controller, goal, intent, or basis",
            )


@dataclass(frozen=True)
class WaypointStep:
    step_id: str
    x_mm: int
    y_mm: int
    heading_mdeg: int
    position_tolerance_mm: int
    heading_tolerance_mdeg: int

    def __post_init__(self) -> None:
        _identifier("waypoint_step_id", self.step_id)
        _integer("waypoint_x_mm", self.x_mm, -1_000_000, 1_000_000)
        _integer("waypoint_y_mm", self.y_mm, -1_000_000, 1_000_000)
        _integer("waypoint_heading_mdeg", self.heading_mdeg, -180_000, 179_999)
        _integer("position_tolerance_mm", self.position_tolerance_mm, 1, 10_000)
        _integer("heading_tolerance_mdeg", self.heading_tolerance_mdeg, 1, 180_000)


@dataclass(frozen=True)
class SensorStep:
    step_id: str
    operation: str
    target_hypothesis_id: Optional[str] = None
    profile_id: Optional[str] = None

    def __post_init__(self) -> None:
        _identifier("sensor_step_id", self.step_id)
        if self.operation not in SENSOR_OPERATIONS:
            raise PhysicalAgentStateError(
                "invalid_sensor_operation",
                "sensor operation is invalid",
            )
        if self.operation == "SCAN_FRONT_ARC":
            _identifier(
                "sensor_target_hypothesis_id",
                self.target_hypothesis_id,
            )
            _identifier("sensor_profile_id", self.profile_id)
        elif self.target_hypothesis_id is not None or self.profile_id is not None:
            raise PhysicalAgentStateError(
                "unexpected_sensor_binding",
                "OBSERVE cannot carry a target or scan profile",
            )


@dataclass(frozen=True)
class PrimitiveStep:
    """Temporary compatibility step for the legacy semantic action tail."""

    step_id: str
    action: str

    def __post_init__(self) -> None:
        _identifier("primitive_step_id", self.step_id)
        if self.action not in PRIMITIVE_ACTIONS:
            raise PhysicalAgentStateError(
                "invalid_primitive_action",
                "primitive action is invalid",
            )


PlanStep = Union[WaypointStep, SensorStep, PrimitiveStep]


@dataclass(frozen=True)
class ExecutionPlan:
    plan_id: str
    revision: int
    binding: PlanBinding
    steps: Tuple[PlanStep, ...]
    cursor: int
    created_at_ms: int

    def __post_init__(self) -> None:
        _identifier("plan_id", self.plan_id)
        _integer("plan_revision", self.revision, 1)
        if not isinstance(self.binding, PlanBinding):
            raise PhysicalAgentStateError(
                "invalid_plan_binding",
                "execution plan binding is invalid",
            )
        if (
            not isinstance(self.steps, tuple)
            or not self.steps
            or len(self.steps) > 64
            or any(
                not isinstance(step, (WaypointStep, SensorStep, PrimitiveStep))
                for step in self.steps
            )
        ):
            raise PhysicalAgentStateError(
                "invalid_plan_steps",
                "execution plan steps are invalid",
            )
        step_ids = tuple(step.step_id for step in self.steps)
        if len(set(step_ids)) != len(step_ids):
            raise PhysicalAgentStateError(
                "duplicate_plan_step_id",
                "execution plan step IDs must be unique",
            )
        _integer("plan_cursor", self.cursor, 0, len(self.steps))
        _integer("plan_created_at_ms", self.created_at_ms)

    @property
    def complete(self) -> bool:
        return self.cursor == len(self.steps)

    @property
    def active_step(self) -> Optional[PlanStep]:
        return None if self.complete else self.steps[self.cursor]


@dataclass(frozen=True)
class PlanStepKey:
    plan_id: str
    plan_revision: int
    cursor: int
    step_id: str

    def __post_init__(self) -> None:
        _identifier("step_key_plan_id", self.plan_id)
        _integer("step_key_plan_revision", self.plan_revision, 1)
        _integer("step_key_cursor", self.cursor)
        _identifier("step_key_step_id", self.step_id)


@dataclass(frozen=True)
class StepCommandAuthorization:
    """Host-issued permission to start one exact plan-step command."""

    action_id: str
    command_id: str
    host_dispatch_sequence: int
    controller_key: ControllerKey
    step_key: PlanStepKey
    based_on_navigation_basis_id: str
    based_on_controller_state_version: int
    command_fingerprint: str
    issued_at_ms: int
    valid_until_ms: int

    def __post_init__(self) -> None:
        _identifier("step_action_id", self.action_id)
        _identifier("step_command_id", self.command_id)
        _integer(
            "host_dispatch_sequence",
            self.host_dispatch_sequence,
            1,
        )
        if not isinstance(self.controller_key, ControllerKey):
            raise PhysicalAgentStateError(
                "invalid_command_controller_key",
                "step authorization controller key is invalid",
            )
        if not isinstance(self.step_key, PlanStepKey):
            raise PhysicalAgentStateError(
                "invalid_command_step_key",
                "step authorization plan-step key is invalid",
            )
        _identifier(
            "command_navigation_basis_id",
            self.based_on_navigation_basis_id,
        )
        _integer(
            "command_controller_state_version",
            self.based_on_controller_state_version,
            1,
        )
        _identifier("command_fingerprint", self.command_fingerprint, 256)
        _integer("command_issued_at_ms", self.issued_at_ms)
        _integer("command_valid_until_ms", self.valid_until_ms)
        if (
            self.valid_until_ms <= self.issued_at_ms
            or self.valid_until_ms - self.issued_at_ms
            > MAX_STEP_COMMAND_START_TTL_MS
        ):
            raise PhysicalAgentStateError(
                "invalid_step_command_ttl",
                "command start expiry must be after issue and within 60 seconds",
            )


@dataclass(frozen=True)
class ActiveDispatch:
    """Host-side command lifecycle; ``dispatched_at_ms`` uses the host clock."""

    authorization: StepCommandAuthorization
    dispatched_at_ms: Optional[int] = None
    settle_by_host_ms: Optional[int] = None
    settlement_expired_at_host_ms: Optional[int] = None

    def __post_init__(self) -> None:
        if not isinstance(self.authorization, StepCommandAuthorization):
            raise PhysicalAgentStateError(
                "invalid_active_authorization",
                "active dispatch authorization is invalid",
            )
        if (self.dispatched_at_ms is None) != (self.settle_by_host_ms is None):
            raise PhysicalAgentStateError(
                "incomplete_active_dispatch",
                "dispatch time and settlement deadline must be recorded together",
            )
        if self.dispatched_at_ms is not None:
            _integer(
                "command_dispatched_at_ms",
                self.dispatched_at_ms,
                self.authorization.issued_at_ms,
            )
            if self.dispatched_at_ms >= self.authorization.valid_until_ms:
                raise PhysicalAgentStateError(
                    "step_command_authorization_expired",
                    "command must be dispatched before authorization expiry",
                )
            _integer(
                "command_settle_by_host_ms",
                self.settle_by_host_ms,
                self.dispatched_at_ms + 1,
            )
            if (
                self.settle_by_host_ms - self.dispatched_at_ms
                > MAX_STEP_COMMAND_SETTLE_MS
            ):
                raise PhysicalAgentStateError(
                    "invalid_step_command_settle_deadline",
                    "command settlement deadline exceeds the finite ceiling",
                )
        if self.settlement_expired_at_host_ms is not None:
            if self.settle_by_host_ms is None:
                raise PhysicalAgentStateError(
                    "invalid_step_command_settlement_expiry",
                    "only a dispatched command can expire",
                )
            _integer(
                "command_settlement_expired_at_host_ms",
                self.settlement_expired_at_host_ms,
                self.settle_by_host_ms,
            )

    @property
    def dispatched(self) -> bool:
        return self.dispatched_at_ms is not None

    @property
    def settlement_expired(self) -> bool:
        return self.settlement_expired_at_host_ms is not None


@dataclass(frozen=True)
class ControllerCommandReceipt:
    """Verified adapter result stamped on ingress with the host clock."""

    outcome: ReceiptOutcome
    controller_key: ControllerKey
    step_key: PlanStepKey
    action_id: str
    command_id: str
    host_dispatch_sequence: int
    command_fingerprint: str
    based_on_navigation_basis_id: str
    based_on_controller_state_version: int
    resulting_controller_state_version: int
    received_at_host_ms: int
    stop_confirmed: bool
    code: str

    def __post_init__(self) -> None:
        if not isinstance(self.outcome, ReceiptOutcome):
            raise PhysicalAgentStateError(
                "invalid_receipt_outcome",
                "controller receipt outcome is invalid",
            )
        if not isinstance(self.controller_key, ControllerKey):
            raise PhysicalAgentStateError(
                "invalid_receipt_controller_key",
                "controller receipt key is invalid",
            )
        if not isinstance(self.step_key, PlanStepKey):
            raise PhysicalAgentStateError(
                "invalid_receipt_step_key",
                "controller receipt plan-step key is invalid",
            )
        _identifier("receipt_action_id", self.action_id)
        _identifier("receipt_command_id", self.command_id)
        _integer(
            "receipt_host_dispatch_sequence",
            self.host_dispatch_sequence,
            1,
        )
        _identifier("receipt_command_fingerprint", self.command_fingerprint, 256)
        _identifier(
            "receipt_navigation_basis_id",
            self.based_on_navigation_basis_id,
        )
        _integer(
            "receipt_based_controller_state_version",
            self.based_on_controller_state_version,
            1,
        )
        _integer(
            "receipt_resulting_controller_state_version",
            self.resulting_controller_state_version,
            self.based_on_controller_state_version,
        )
        _integer("receipt_received_at_host_ms", self.received_at_host_ms)
        if not isinstance(self.stop_confirmed, bool):
            raise PhysicalAgentStateError(
                "invalid_receipt_stop_confirmation",
                "receipt stop confirmation must be a boolean",
            )
        if self.outcome == ReceiptOutcome.STOPPED and not self.stop_confirmed:
            raise PhysicalAgentStateError(
                "missing_receipt_stop_confirmation",
                "STOPPED receipt must confirm that motion stopped",
            )
        _identifier("receipt_code", self.code, 160)


@dataclass(frozen=True)
class PlanningTicket:
    ticket_id: str
    cause: PlanningCause
    basis: NavigationBasis
    created_at_ms: int
    valid_until_ms: int
    consumed_at_ms: Optional[int] = None

    def __post_init__(self) -> None:
        _identifier("planning_ticket_id", self.ticket_id)
        if not isinstance(self.cause, PlanningCause):
            raise PhysicalAgentStateError(
                "invalid_planning_cause",
                "planning cause is invalid",
            )
        if not isinstance(self.basis, NavigationBasis):
            raise PhysicalAgentStateError(
                "invalid_planning_basis",
                "planning ticket basis is invalid",
            )
        _integer("planning_ticket_created_at_ms", self.created_at_ms)
        _integer("planning_ticket_valid_until_ms", self.valid_until_ms)
        if (
            self.valid_until_ms <= self.created_at_ms
            or self.valid_until_ms - self.created_at_ms
            > MAX_PLANNING_TICKET_TTL_MS
        ):
            raise PhysicalAgentStateError(
                "invalid_planning_ticket_ttl",
                "planning ticket expiry must be after creation and within "
                "the TTL limit",
            )
        if self.consumed_at_ms is not None:
            _integer(
                "planning_ticket_consumed_at_ms",
                self.consumed_at_ms,
                self.created_at_ms,
            )
            if self.consumed_at_ms >= self.valid_until_ms:
                raise PhysicalAgentStateError(
                    "planning_ticket_expired",
                    "planning ticket cannot be consumed at or after expiry",
                )

    @property
    def consumed(self) -> bool:
        return self.consumed_at_ms is not None

    def consume(self, consumed_at_ms: int) -> "PlanningTicket":
        if self.consumed:
            raise PhysicalAgentStateError(
                "planning_ticket_already_consumed",
                "planning ticket may be consumed only once",
            )
        _integer(
            "planning_ticket_consumed_at_ms",
            consumed_at_ms,
            self.created_at_ms,
        )
        if consumed_at_ms >= self.valid_until_ms:
            raise PhysicalAgentStateError(
                "planning_ticket_expired",
                "planning ticket cannot be consumed at or after expiry",
            )
        return replace(self, consumed_at_ms=consumed_at_ms)


@dataclass(frozen=True)
class GoalTerminal:
    outcome: GoalOutcome
    reason: str
    completed_at_ms: int

    def __post_init__(self) -> None:
        if not isinstance(self.outcome, GoalOutcome):
            raise PhysicalAgentStateError(
                "invalid_goal_outcome",
                "goal outcome is invalid",
            )
        _identifier("goal_terminal_reason", self.reason, 160)
        _integer("goal_completed_at_ms", self.completed_at_ms)


@dataclass(frozen=True)
class PhysicalAgentState:
    controller_key: ControllerKey
    agent_state_version: int = 1
    phase: AgentPhase = AgentPhase.IDLE
    goal_epoch: int = 0
    plan_revision: int = 0
    last_host_dispatch_sequence: int = 0
    compile_pending: bool = False
    goal: Optional[GoalAssignment] = None
    basis: Optional[NavigationBasis] = None
    intent: Optional[ActiveIntent] = None
    intent_progress: Optional[IntentProgress] = None
    plan: Optional[ExecutionPlan] = None
    active_dispatch: Optional[ActiveDispatch] = None
    planning_ticket: Optional[PlanningTicket] = None
    terminal: Optional[GoalTerminal] = None

    def __post_init__(self) -> None:
        if not isinstance(self.controller_key, ControllerKey):
            raise PhysicalAgentStateError(
                "invalid_state_controller_key",
                "state controller key is invalid",
            )
        _integer("agent_state_version", self.agent_state_version, 1)
        if not isinstance(self.phase, AgentPhase):
            raise PhysicalAgentStateError(
                "invalid_agent_phase",
                "agent phase is invalid",
            )
        _integer("state_goal_epoch", self.goal_epoch)
        _integer("state_plan_revision", self.plan_revision)
        _integer(
            "state_last_host_dispatch_sequence",
            self.last_host_dispatch_sequence,
        )
        if not isinstance(self.compile_pending, bool):
            raise PhysicalAgentStateError(
                "invalid_compile_pending",
                "compile-pending marker must be a boolean",
            )
        if self.goal is not None and not isinstance(self.goal, GoalAssignment):
            raise PhysicalAgentStateError("invalid_state_goal", "state goal is invalid")
        if self.basis is not None and not isinstance(self.basis, NavigationBasis):
            raise PhysicalAgentStateError("invalid_state_basis", "state basis is invalid")
        if self.intent is not None and not isinstance(self.intent, ActiveIntent):
            raise PhysicalAgentStateError("invalid_state_intent", "state intent is invalid")
        if self.intent_progress is not None and not isinstance(
            self.intent_progress, IntentProgress
        ):
            raise PhysicalAgentStateError(
                "invalid_state_intent_progress", "state intent progress is invalid"
            )
        if self.plan is not None and not isinstance(self.plan, ExecutionPlan):
            raise PhysicalAgentStateError("invalid_state_plan", "state plan is invalid")
        if self.active_dispatch is not None and not isinstance(
            self.active_dispatch,
            ActiveDispatch,
        ):
            raise PhysicalAgentStateError(
                "invalid_state_active_dispatch",
                "state active dispatch is invalid",
            )
        if self.planning_ticket is not None and not isinstance(
            self.planning_ticket, PlanningTicket
        ):
            raise PhysicalAgentStateError(
                "invalid_state_planning_ticket",
                "state planning ticket is invalid",
            )
        if self.terminal is not None and not isinstance(self.terminal, GoalTerminal):
            raise PhysicalAgentStateError(
                "invalid_state_terminal",
                "state terminal outcome is invalid",
            )
        self._validate_phase_shape()

    def _validate_common_active(self) -> None:
        if self.goal is None or self.basis is None:
            raise PhysicalAgentStateError(
                "active_state_missing_goal_basis",
                "active state requires a goal and navigation basis",
            )
        if (
            self.goal.goal_epoch != self.goal_epoch
            or self.basis.goal_epoch != self.goal_epoch
            or self.basis.controller_key != self.controller_key
        ):
            raise PhysicalAgentStateError(
                "active_state_binding_mismatch",
                "active state identity or goal epoch is inconsistent",
            )
        if self.intent is not None and (
            self.intent.goal_id != self.goal.goal_id
            or self.intent.goal_epoch != self.goal_epoch
            or self.intent.accepted_basis.controller_key != self.controller_key
        ):
            raise PhysicalAgentStateError(
                "state_intent_binding_mismatch",
                "active intent is not bound to the current goal and controller",
            )
        if (self.intent is None) != (self.intent_progress is None):
            raise PhysicalAgentStateError(
                "intent_progress_binding_mismatch",
                "intent and intent progress must be present together",
            )
        if self.intent_progress is not None and (
            self.intent_progress.plan_attempts > self.intent.policy.max_plan_attempts
            or self.intent_progress.consecutive_no_progress_plans
            > self.intent.policy.max_consecutive_no_progress_plans
        ):
            raise PhysicalAgentStateError(
                "intent_budget_exceeded",
                "intent progress exceeds its host-owned policy",
            )
        if self.planning_ticket is not None and (
            not self.planning_ticket.basis.decision_equivalent(self.basis)
            or self.planning_ticket.basis.goal_epoch != self.goal_epoch
        ):
            raise PhysicalAgentStateError(
                "planning_ticket_basis_mismatch",
                "planning ticket does not match current planning evidence",
            )

    def _validate_phase_shape(self) -> None:
        if self.phase == AgentPhase.IDLE:
            if any(
                value is not None
                for value in (
                    self.goal,
                    self.basis,
                    self.intent,
                    self.intent_progress,
                    self.plan,
                    self.active_dispatch,
                    self.planning_ticket,
                    self.terminal,
                )
            ) or self.plan_revision != 0 or self.compile_pending:
                raise PhysicalAgentStateError(
                    "invalid_idle_state",
                    "IDLE cannot contain active goal state",
                )
            return

        self._validate_common_active()
        if self.phase == AgentPhase.PLANNING:
            if (
                self.plan is not None
                or self.active_dispatch is not None
                or self.terminal is not None
            ):
                raise PhysicalAgentStateError(
                    "invalid_planning_state",
                    "PLANNING cannot contain a plan or terminal outcome",
                )
            if self.compile_pending and (
                self.intent is None
                or self.intent_progress is None
                or self.planning_ticket is not None
            ):
                raise PhysicalAgentStateError(
                    "invalid_compile_pending_state",
                    "deterministic compilation requires an active intent and no ticket",
                )
            return

        if self.phase == AgentPhase.EXECUTING:
            if (
                self.intent is None
                or self.plan is None
                or self.plan.complete
                or self.terminal is not None
                or self.plan.revision != self.plan_revision
                or self.compile_pending
            ):
                raise PhysicalAgentStateError(
                    "invalid_executing_state",
                    "EXECUTING requires one incomplete authoritative plan",
                )
            self.plan.binding.assert_matches(
                controller_key=self.controller_key,
                goal=self.goal,
                intent=self.intent,
                basis=self.basis,
            )
            if self.active_dispatch is not None:
                authorization = self.active_dispatch.authorization
                active_step = self.plan.active_step
                if (
                    authorization.controller_key != self.controller_key
                    or authorization.host_dispatch_sequence
                    != self.last_host_dispatch_sequence
                    or authorization.step_key.plan_id != self.plan.plan_id
                    or authorization.step_key.plan_revision
                    != self.plan.revision
                    or authorization.step_key.cursor != self.plan.cursor
                    or authorization.step_key.step_id != active_step.step_id
                    or authorization.based_on_controller_state_version
                    > self.basis.controller_state_version
                ):
                    raise PhysicalAgentStateError(
                        "active_dispatch_binding_mismatch",
                        "active dispatch is not bound to the current plan step",
                    )
            return

        if self.phase == AgentPhase.STOPPING:
            if (
                self.plan is not None
                or self.planning_ticket is not None
                or self.terminal is None
                or self.compile_pending
                or (
                    self.active_dispatch is not None
                    and (
                        not self.active_dispatch.dispatched
                        or self.active_dispatch.authorization.controller_key
                        != self.controller_key
                        or self.active_dispatch.authorization.host_dispatch_sequence
                        != self.last_host_dispatch_sequence
                    )
                )
            ):
                raise PhysicalAgentStateError(
                    "invalid_stopping_state",
                    "STOPPING requires one pending terminal outcome and no plan",
                )
            return

        if self.phase == AgentPhase.TERMINAL:
            if (
                self.intent is not None
                or self.intent_progress is not None
                or self.plan is not None
                or self.active_dispatch is not None
                or self.planning_ticket is not None
                or self.terminal is None
                or self.compile_pending
            ):
                raise PhysicalAgentStateError(
                    "invalid_terminal_state",
                    "TERMINAL cannot contain active intent or plan state",
                )
            return

        raise PhysicalAgentStateError("invalid_agent_phase", "agent phase is invalid")


# Transition events.  They carry facts only; none owns a motor or calls a model.


@dataclass(frozen=True)
class GoalActivated:
    goal: GoalAssignment
    basis: NavigationBasis
    ticket: PlanningTicket


@dataclass(frozen=True)
class PlanningTicketConsumed:
    ticket_id: str
    based_on_basis: NavigationBasis
    consumed_at_ms: int


@dataclass(frozen=True)
class PlanningTicketExpired:
    ticket_id: str
    based_on_basis: NavigationBasis
    observed_at_ms: int


@dataclass(frozen=True)
class IntentAccepted:
    ticket_id: str
    based_on_basis: NavigationBasis
    intent: ActiveIntent
    plan: ExecutionPlan


@dataclass(frozen=True)
class PlanningAbortRequested:
    ticket_id: str
    based_on_basis: NavigationBasis
    terminal: GoalTerminal


@dataclass(frozen=True)
class PlanningHeld:
    ticket_id: str
    based_on_basis: NavigationBasis


@dataclass(frozen=True)
class PlanningRequested:
    ticket: PlanningTicket


@dataclass(frozen=True)
class NavigationBasisUpdated:
    basis: NavigationBasis


@dataclass(frozen=True)
class StepCommandAuthorized:
    authorization: StepCommandAuthorization


@dataclass(frozen=True)
class StepCommandDispatched:
    """Write-ahead fact; MUST be journaled before the first outbound byte."""

    authorization: StepCommandAuthorization
    dispatched_at_ms: int
    settle_by_host_ms: int


@dataclass(frozen=True)
class StepCommandRevoked:
    authorization: StepCommandAuthorization
    revoked_at_ms: int


@dataclass(frozen=True)
class StepCommandSettlementExpired:
    authorization: StepCommandAuthorization
    observed_at_host_ms: int


@dataclass(frozen=True)
class StepCommandSettled:
    receipt: ControllerCommandReceipt
    resulting_basis: NavigationBasis
    disposition: StepDisposition
    replan_ticket: Optional[PlanningTicket] = None


@dataclass(frozen=True)
class PlanRecompiled:
    plan: ExecutionPlan
    resulting_basis: NavigationBasis


@dataclass(frozen=True)
class ReplanRequested:
    ticket: PlanningTicket
    resulting_basis: NavigationBasis
    reason: str


@dataclass(frozen=True)
class GoalCompletionRequested:
    resulting_basis: NavigationBasis
    terminal: GoalTerminal


@dataclass(frozen=True)
class StopRequested:
    terminal: GoalTerminal


@dataclass(frozen=True)
class StopVerified:
    verified_at_ms: int
    dispatch_receipt: Optional[ControllerCommandReceipt] = None
    resulting_basis: Optional[NavigationBasis] = None


@dataclass(frozen=True)
class TerminalCleared:
    cleared_at_ms: int


PhysicalAgentEvent = Union[
    GoalActivated,
    PlanningTicketConsumed,
    PlanningTicketExpired,
    IntentAccepted,
    PlanningAbortRequested,
    PlanningHeld,
    PlanningRequested,
    NavigationBasisUpdated,
    StepCommandAuthorized,
    StepCommandDispatched,
    StepCommandRevoked,
    StepCommandSettlementExpired,
    StepCommandSettled,
    PlanRecompiled,
    ReplanRequested,
    GoalCompletionRequested,
    StopRequested,
    StopVerified,
    TerminalCleared,
]



__all__ = (
    "ActiveIntent",
    "ActiveDispatch",
    "AgentPhase",
    "ControllerKey",
    "ControllerCommandReceipt",
    "DetourSide",
    "DetourTargetIntent",
    "ExecutionPlan",
    "FollowDirectionIntent",
    "GoalActivated",
    "GoalAssignment",
    "GoalCompletionRequested",
    "GoalOutcome",
    "GoalTerminal",
    "IntentAccepted",
    "IntentPolicy",
    "IntentProgress",
    "MAX_PLANNING_TICKET_TTL_MS",
    "MAX_STEP_COMMAND_SETTLE_MS",
    "MAX_STEP_COMMAND_START_TTL_MS",
    "NavigationBasis",
    "NavigationBasisUpdated",
    "PhysicalAgentEvent",
    "PhysicalAgentState",
    "PhysicalAgentStateError",
    "PlanBinding",
    "PlanRecompiled",
    "PlanStep",
    "PlanStepKey",
    "PlanningAbortRequested",
    "PlanningCause",
    "PlanningHeld",
    "PlanningRequested",
    "PlanningTicket",
    "PlanningTicketConsumed",
    "PlanningTicketExpired",
    "PrimitiveStep",
    "ReceiptOutcome",
    "ReplanRequested",
    "ScanTargetIntent",
    "SensorStep",
    "StopRequested",
    "StopVerified",
    "StepCommandAuthorization",
    "StepCommandAuthorized",
    "StepCommandDispatched",
    "StepCommandRevoked",
    "StepCommandSettlementExpired",
    "StepCommandSettled",
    "StepDisposition",
    "TerminalCleared",
    "WaypointStep",
)
