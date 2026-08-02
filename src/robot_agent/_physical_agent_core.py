"""Private value objects for the canonical physical-agent contract."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional, Tuple, Union


MAX_INT = 2**63 - 1
MAX_PLANNING_TICKET_TTL_MS = 5 * 60_000
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
