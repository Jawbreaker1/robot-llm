"""Pure legacy-decision projection for canonical shadow comparison.

The projection consumes a decision that the legacy host has already accepted
and the maneuver-commitment state *after* that decision was applied.  It does
not call a model, mutate a reducer, authorize an action, or manufacture a
controller receipt.  The resulting intent and plan are comparison artifacts
only; in particular, ``PrimitiveStep`` is not execution-equivalent to a
canonical controller command.
"""

from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
from typing import Mapping, Optional, Sequence, Tuple

from .legacy_control_projection import (
    LegacyControlProjectionError,
    project_active_maneuver_intent,
)
from .physical_agent_state import (
    ActiveIntent,
    DetourTargetIntent,
    ExecutionPlan,
    FollowDirectionIntent,
    GoalAssignment,
    NavigationBasis,
    PhysicalAgentStateError,
    PlanBinding,
    PrimitiveStep,
    ScanTargetIntent,
    SensorStep,
)
from .physical_navigation_contract import (
    ACTIONS,
    FINISH,
    MOTION_ACTIONS,
    OBSERVE,
    SCAN_FRONT_ARC,
    NavigationDecision,
)


SHADOW_PROJECTION_SCHEMA = "legacy-decision-canonical-shadow/v1"

EXACT_CONTRACT = "exact_contract"
LOSSY_COMPATIBILITY = "lossy_compatibility"
NOT_EXECUTION_EQUIVALENT = "not_execution_equivalent"
NOT_EVALUATED = "NOT_EVALUATED"

_MAPPING_CLASSES = frozenset((EXACT_CONTRACT, LOSSY_COMPATIBILITY))
_MAX_INT = 2**63 - 1


class LegacyDecisionShadowProjectionError(ValueError):
    """A validated legacy value could not be represented in the shadow."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _identifier(name: str, value: object, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise LegacyDecisionShadowProjectionError(
            "invalid_{}".format(name),
            "{} is invalid".format(name.replace("_", " ")),
        )
    return value


def _integer(name: str, value: object, minimum: int = 0) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= _MAX_INT
    ):
        raise LegacyDecisionShadowProjectionError(
            "invalid_{}".format(name),
            "{} is invalid".format(name.replace("_", " ")),
        )
    return value


def _step_ids(values: Sequence[str], expected: int) -> Tuple[str, ...]:
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise LegacyDecisionShadowProjectionError(
            "invalid_step_ids", "Step IDs must be an injected sequence"
        )
    result = tuple(values)
    if (
        len(result) != expected
        or any(_identifier("step_id", value) != value for value in result)
        or len(set(result)) != expected
    ):
        raise LegacyDecisionShadowProjectionError(
            "invalid_step_ids",
            "One unique injected step ID is required per projected step",
        )
    return result


def _projection_context(
    *,
    decision: NavigationDecision,
    post_maneuver_state: Mapping[str, object],
    goal: GoalAssignment,
    basis: NavigationBasis,
    accepted_at_ms: int,
    intent_id: str,
    plan_id: str,
    plan_revision: int,
    scan_profile_id: str,
) -> Optional[Mapping[str, object]]:
    if not isinstance(decision, NavigationDecision):
        raise LegacyDecisionShadowProjectionError(
            "invalid_navigation_decision",
            "decision must be a host-validated NavigationDecision",
        )
    if not isinstance(goal, GoalAssignment) or not isinstance(
        basis, NavigationBasis
    ):
        raise LegacyDecisionShadowProjectionError(
            "invalid_projection_context",
            "goal and basis must be canonical values",
        )
    if goal.goal_epoch != basis.goal_epoch:
        raise LegacyDecisionShadowProjectionError(
            "goal_basis_mismatch", "goal and basis epochs differ"
        )
    _integer("accepted_at_ms", accepted_at_ms)
    _identifier("intent_id", intent_id)
    _identifier("plan_id", plan_id)
    _integer("plan_revision", plan_revision, 1)
    _identifier("scan_profile_id", scan_profile_id)

    if (
        not isinstance(post_maneuver_state, Mapping)
        or set(post_maneuver_state) != {"active", "last_terminal"}
        or post_maneuver_state["last_terminal"] is not None
        and not isinstance(post_maneuver_state["last_terminal"], Mapping)
    ):
        raise LegacyDecisionShadowProjectionError(
            "invalid_post_maneuver_state",
            "post-apply maneuver state has invalid fields",
        )
    active = post_maneuver_state["active"]
    if active is not None and not isinstance(active, Mapping):
        raise LegacyDecisionShadowProjectionError(
            "invalid_post_maneuver_state",
            "post-apply active maneuver is invalid",
        )

    plan = decision.plan
    if (
        not isinstance(plan, tuple)
        or not plan
        or decision.action not in ACTIONS
        or plan[0] != decision.action
        or any(action not in ACTIONS for action in plan)
    ):
        raise LegacyDecisionShadowProjectionError(
            "invalid_validated_decision",
            "decision action and plan are not a projectable validated plan",
        )
    if decision.action == FINISH:
        if plan != (FINISH,):
            raise LegacyDecisionShadowProjectionError(
                "invalid_terminal_plan", "FINISH must be a singleton plan"
            )
    elif FINISH in plan:
        raise LegacyDecisionShadowProjectionError(
            "unsupported_terminal_tail",
            "FINISH cannot be represented inside an execution plan",
        )
    return active


def _json_value(value: object) -> object:
    """Serialize only the frozen canonical values emitted by this module."""

    if isinstance(value, Enum):
        return value.value
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if is_dataclass(value) and not isinstance(value, type):
        return {
            item.name: _json_value(getattr(value, item.name))
            for item in fields(value)
        }
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if isinstance(value, Mapping) and all(
        isinstance(key, str) for key in value
    ):
        return {key: _json_value(item) for key, item in value.items()}
    raise LegacyDecisionShadowProjectionError(
        "unsupported_json_value", "shadow value is not JSON serializable"
    )


def _intent_to_dict(value: ActiveIntent) -> Mapping[str, object]:
    payload = value.payload
    if isinstance(payload, FollowDirectionIntent):
        kind = "FOLLOW_DIRECTION"
    elif isinstance(payload, ScanTargetIntent):
        kind = "SCAN_TARGET"
    elif isinstance(payload, DetourTargetIntent):
        kind = "DETOUR_TARGET"
    else:  # pragma: no cover - ActiveIntent already enforces this union.
        raise LegacyDecisionShadowProjectionError(
            "unsupported_intent_payload", "intent payload is not serializable"
        )
    result = _json_value(value)
    result["payload"] = {"kind": kind, **_json_value(payload)}
    return result


def _plan_to_dict(value: ExecutionPlan) -> Mapping[str, object]:
    steps = []
    for step in value.steps:
        if isinstance(step, PrimitiveStep):
            kind = "PRIMITIVE"
        elif isinstance(step, SensorStep):
            kind = "SENSOR"
        else:  # pragma: no cover - this projector emits no waypoint steps.
            raise LegacyDecisionShadowProjectionError(
                "unsupported_projected_step", "plan step is not serializable"
            )
        steps.append({"kind": kind, **_json_value(step)})
    result = _json_value(value)
    result["steps"] = steps
    return result


@dataclass(frozen=True)
class LegacyDecisionShadowClassification:
    """Honest limits of a legacy-to-canonical comparison artifact."""

    mapping_class: str
    intent_mapping_class: str
    plan_mapping_class: str
    execution_equivalence: str = NOT_EXECUTION_EQUIVALENT
    offer_parity: str = NOT_EVALUATED
    receipt_parity: str = NOT_EVALUATED

    def __post_init__(self) -> None:
        if (
            self.mapping_class not in _MAPPING_CLASSES
            or self.intent_mapping_class not in _MAPPING_CLASSES
            or self.plan_mapping_class not in _MAPPING_CLASSES
            or self.execution_equivalence != NOT_EXECUTION_EQUIVALENT
            or self.offer_parity != NOT_EVALUATED
            or self.receipt_parity != NOT_EVALUATED
        ):
            raise LegacyDecisionShadowProjectionError(
                "invalid_shadow_classification",
                "shadow classification is invalid",
            )

    def to_dict(self) -> Mapping[str, str]:
        return {
            "mapping_class": self.mapping_class,
            "intent_mapping_class": self.intent_mapping_class,
            "plan_mapping_class": self.plan_mapping_class,
            "execution_equivalence": self.execution_equivalence,
            "offer_parity": self.offer_parity,
            "receipt_parity": self.receipt_parity,
        }


@dataclass(frozen=True)
class LegacyDecisionShadowProjection:
    """Canonical comparison artifacts correlated to one accepted decision."""

    episode_id: str
    turn: int
    based_on_state_version: int
    source_action: str
    source_plan: Tuple[str, ...]
    terminal: bool
    intent: Optional[ActiveIntent]
    execution_plan: Optional[ExecutionPlan]
    classification: LegacyDecisionShadowClassification

    def __post_init__(self) -> None:
        _identifier("episode_id", self.episode_id)
        _integer("turn", self.turn)
        _integer("based_on_state_version", self.based_on_state_version)
        if (
            self.source_action not in ACTIONS
            or not isinstance(self.source_plan, tuple)
            or not self.source_plan
            or self.source_plan[0] != self.source_action
        ):
            raise LegacyDecisionShadowProjectionError(
                "invalid_source_decision", "source decision is invalid"
            )
        if type(self.terminal) is not bool:
            raise LegacyDecisionShadowProjectionError(
                "invalid_terminal_marker", "terminal must be boolean"
            )
        if self.terminal:
            if (
                self.source_action != FINISH
                or self.intent is not None
                or self.execution_plan is not None
            ):
                raise LegacyDecisionShadowProjectionError(
                    "invalid_terminal_projection",
                    "terminal projection cannot contain intent or plan",
                )
        elif (
            self.source_action == FINISH
            or not isinstance(self.intent, ActiveIntent)
            or not isinstance(self.execution_plan, ExecutionPlan)
        ):
            raise LegacyDecisionShadowProjectionError(
                "invalid_active_projection",
                "non-terminal projection requires intent and plan",
            )
        if not isinstance(
            self.classification, LegacyDecisionShadowClassification
        ):
            raise LegacyDecisionShadowProjectionError(
                "invalid_shadow_classification",
                "classification must be a shadow classification",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": SHADOW_PROJECTION_SCHEMA,
            "source": {
                "episode_id": self.episode_id,
                "turn": self.turn,
                "based_on_state_version": self.based_on_state_version,
                "action": self.source_action,
                "plan": list(self.source_plan),
            },
            "terminal": self.terminal,
            "intent": (
                None if self.intent is None else _intent_to_dict(self.intent)
            ),
            "execution_plan": (
                None
                if self.execution_plan is None
                else _plan_to_dict(self.execution_plan)
            ),
            "classification": self.classification.to_dict(),
        }


def _inactive_intent(
    decision: NavigationDecision,
    *,
    goal: GoalAssignment,
    basis: NavigationBasis,
    intent_id: str,
    scan_profile_id: str,
    accepted_at_ms: int,
) -> Tuple[ActiveIntent, str]:
    if decision.action == SCAN_FRONT_ARC:
        target_id = decision.perception_target_hypothesis_id
        if target_id is None:
            raise LegacyDecisionShadowProjectionError(
                "missing_scan_target",
                "SCAN_FRONT_ARC has no projected target",
            )
        payload = ScanTargetIntent(target_id, scan_profile_id)
        mapping_class = EXACT_CONTRACT
    else:
        payload = FollowDirectionIntent()
        # Several legacy decisions collapse into one goal-level intent.  The
        # exact legacy action remains visible in the compatibility plan.
        mapping_class = LOSSY_COMPATIBILITY
    try:
        return (
            ActiveIntent(
                intent_id=intent_id,
                revision=1,
                goal_id=goal.goal_id,
                goal_epoch=goal.goal_epoch,
                payload=payload,
                accepted_basis=basis,
                accepted_at_ms=accepted_at_ms,
            ),
            mapping_class,
        )
    except PhysicalAgentStateError as error:
        raise LegacyDecisionShadowProjectionError(
            "canonical_intent_rejected",
            "canonical ActiveIntent rejected the projection",
        ) from error


def _execution_plan(
    decision: NavigationDecision,
    *,
    goal: GoalAssignment,
    basis: NavigationBasis,
    intent: ActiveIntent,
    plan_id: str,
    plan_revision: int,
    step_ids: Sequence[str],
    scan_profile_id: str,
    created_at_ms: int,
) -> Tuple[ExecutionPlan, str]:
    ids = _step_ids(step_ids, len(decision.plan))
    scan_target = decision.perception_target_hypothesis_id
    if isinstance(intent.payload, DetourTargetIntent):
        active_target = intent.payload.target_hypothesis_id
        if scan_target is not None and scan_target != active_target:
            raise LegacyDecisionShadowProjectionError(
                "scan_intent_target_mismatch",
                "scan target differs from the post-apply maneuver target",
            )
        scan_target = active_target

    steps = []
    has_primitive = False
    for step_id, action in zip(ids, decision.plan):
        if action in MOTION_ACTIONS:
            has_primitive = True
            steps.append(PrimitiveStep(step_id, action))
        elif action == OBSERVE:
            steps.append(SensorStep(step_id, OBSERVE))
        elif action == SCAN_FRONT_ARC:
            if scan_target is None:
                raise LegacyDecisionShadowProjectionError(
                    "missing_scan_target",
                    "SCAN_FRONT_ARC has no projected target",
                )
            steps.append(
                SensorStep(
                    step_id,
                    SCAN_FRONT_ARC,
                    scan_target,
                    scan_profile_id,
                )
            )
        else:
            raise LegacyDecisionShadowProjectionError(
                "unsupported_plan_action",
                "legacy plan contains an unsupported action",
            )

    try:
        binding = PlanBinding(
            controller_key=basis.controller_key,
            goal_id=goal.goal_id,
            goal_epoch=goal.goal_epoch,
            intent_id=intent.intent_id,
            intent_revision=intent.revision,
            frame_id=basis.frame_id,
            world_generation_id=basis.world_generation_id,
            calibration_fingerprint=basis.calibration_fingerprint,
            based_on_navigation_basis_id=basis.navigation_basis_id,
        )
        return (
            ExecutionPlan(
                plan_id=plan_id,
                revision=plan_revision,
                binding=binding,
                steps=tuple(steps),
                cursor=0,
                created_at_ms=created_at_ms,
            ),
            LOSSY_COMPATIBILITY if has_primitive else EXACT_CONTRACT,
        )
    except PhysicalAgentStateError as error:
        raise LegacyDecisionShadowProjectionError(
            "canonical_plan_rejected",
            "canonical ExecutionPlan rejected the projection",
        ) from error


def project_validated_legacy_decision(
    decision: NavigationDecision,
    post_maneuver_state: Mapping[str, object],
    *,
    goal: GoalAssignment,
    basis: NavigationBasis,
    accepted_at_ms: int,
    intent_id: str,
    plan_id: str,
    plan_revision: int,
    step_ids: Sequence[str],
    scan_profile_id: str,
) -> LegacyDecisionShadowProjection:
    """Project one already accepted legacy decision without granting authority."""

    active = _projection_context(
        decision=decision,
        post_maneuver_state=post_maneuver_state,
        goal=goal,
        basis=basis,
        accepted_at_ms=accepted_at_ms,
        intent_id=intent_id,
        plan_id=plan_id,
        plan_revision=plan_revision,
        scan_profile_id=scan_profile_id,
    )

    projected_active = None
    if active is not None:
        try:
            projected_active = project_active_maneuver_intent(
                post_maneuver_state,
                intent_id=intent_id,
                goal=goal,
                basis=basis,
                accepted_at_ms=accepted_at_ms,
            )
        except LegacyControlProjectionError as error:
            raise LegacyDecisionShadowProjectionError(
                error.code, str(error)
            ) from error

    if decision.action == FINISH:
        try:
            _step_ids(step_ids, 0)
        except LegacyDecisionShadowProjectionError as error:
            raise LegacyDecisionShadowProjectionError(
                "unexpected_terminal_step_ids",
                "FINISH creates no canonical execution steps",
            ) from error
        classification = LegacyDecisionShadowClassification(
            mapping_class=EXACT_CONTRACT,
            intent_mapping_class=EXACT_CONTRACT,
            plan_mapping_class=EXACT_CONTRACT,
        )
        return LegacyDecisionShadowProjection(
            episode_id=decision.episode_id,
            turn=decision.turn,
            based_on_state_version=decision.based_on_state_version,
            source_action=decision.action,
            source_plan=decision.plan,
            terminal=True,
            intent=None,
            execution_plan=None,
            classification=classification,
        )

    if projected_active is None:
        intent, intent_mapping_class = _inactive_intent(
            decision,
            goal=goal,
            basis=basis,
            intent_id=intent_id,
            scan_profile_id=scan_profile_id,
            accepted_at_ms=accepted_at_ms,
        )
    else:
        intent = projected_active
        intent_mapping_class = EXACT_CONTRACT

    plan, plan_mapping_class = _execution_plan(
        decision,
        goal=goal,
        basis=basis,
        intent=intent,
        plan_id=plan_id,
        plan_revision=plan_revision,
        step_ids=step_ids,
        scan_profile_id=scan_profile_id,
        created_at_ms=accepted_at_ms,
    )
    mapping_class = (
        LOSSY_COMPATIBILITY
        if LOSSY_COMPATIBILITY
        in (intent_mapping_class, plan_mapping_class)
        else EXACT_CONTRACT
    )
    return LegacyDecisionShadowProjection(
        episode_id=decision.episode_id,
        turn=decision.turn,
        based_on_state_version=decision.based_on_state_version,
        source_action=decision.action,
        source_plan=decision.plan,
        terminal=False,
        intent=intent,
        execution_plan=plan,
        classification=LegacyDecisionShadowClassification(
            mapping_class=mapping_class,
            intent_mapping_class=intent_mapping_class,
            plan_mapping_class=plan_mapping_class,
        ),
    )


__all__ = (
    "EXACT_CONTRACT",
    "LOSSY_COMPATIBILITY",
    "NOT_EVALUATED",
    "NOT_EXECUTION_EQUIVALENT",
    "SHADOW_PROJECTION_SCHEMA",
    "LegacyDecisionShadowClassification",
    "LegacyDecisionShadowProjection",
    "LegacyDecisionShadowProjectionError",
    "project_validated_legacy_decision",
)
