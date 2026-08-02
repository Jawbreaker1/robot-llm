"""Read-only legacy-to-canonical projections for shadow replay.

These projections grant no execution authority. ``LocalDetourRoute`` has
directed half-plane completion rules that ``WaypointStep`` cannot yet express,
so its projection is not execution-equivalent. ``PrimitiveStep`` is only a
transitional representation of a legacy model-authored motion tail.
"""

from math import isfinite
from typing import Mapping, Optional, Sequence, Tuple

from .local_detour_route import ROUTE_INVALID, LocalDetourRoute
from .maneuver_commitment import DETOUR_SIDES, FACT_KEYS
from .navigation_plan_tail import NavigationPlanTail
from .physical_agent_state import (
    ActiveIntent,
    DetourSide,
    DetourTargetIntent,
    ExecutionPlan,
    GoalAssignment,
    NavigationBasis,
    PhysicalAgentStateError,
    PlanBinding,
    PrimitiveStep,
    WaypointStep,
)
from .physical_navigation_contract import MOTION_ACTIONS


_ACTIVE_FIELDS = frozenset((
    "id", "revision", "objective", "target_hypothesis_id", "detour_side",
    "success_fact_keys", "current_focus_fact_key", "started_turn",
    "last_confirmed_turn",
))


class LegacyControlProjectionError(ValueError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _valid_text(value: object, maximum: int) -> bool:
    return (
        isinstance(value, str) and bool(value) and value == value.strip()
        and len(value) <= maximum
        and not any(ord(character) < 32 for character in value)
    )


def _valid_success_facts(values: object) -> bool:
    if not isinstance(values, (list, tuple)) or not 1 <= len(values) <= len(FACT_KEYS):
        return False
    return (
        all(isinstance(item, str) and item in FACT_KEYS for item in values)
        and len(set(values)) == len(values)
    )


def _context(
    goal: GoalAssignment,
    basis: NavigationBasis,
    intent: Optional[ActiveIntent] = None,
) -> None:
    if not isinstance(goal, GoalAssignment) or not isinstance(
        basis, NavigationBasis
    ):
        raise LegacyControlProjectionError(
            "invalid_projection_context", "Goal or basis is not canonical"
        )
    if goal.goal_epoch != basis.goal_epoch:
        raise LegacyControlProjectionError(
            "goal_basis_mismatch", "Goal and basis epochs differ"
        )
    if intent is None:
        return
    if not isinstance(intent, ActiveIntent):
        raise LegacyControlProjectionError(
            "invalid_projection_context", "Intent is not canonical"
        )
    accepted = intent.accepted_basis
    if (
        intent.goal_id != goal.goal_id
        or intent.goal_epoch != goal.goal_epoch
        or accepted.controller_key != basis.controller_key
        or accepted.frame_id != basis.frame_id
        or accepted.world_generation_id != basis.world_generation_id
        or accepted.calibration_fingerprint != basis.calibration_fingerprint
    ):
        raise LegacyControlProjectionError(
            "intent_basis_mismatch",
            "Intent, goal, controller, frame, generation, or calibration differ",
        )


def _legacy_active(state: Mapping[str, object]) -> Mapping[str, object]:
    if not isinstance(state, Mapping) or set(state) != {"active", "last_terminal"}:
        raise LegacyControlProjectionError(
            "invalid_legacy_maneuver_state", "Legacy maneuver fields are invalid"
        )
    active = state["active"]
    if active is None:
        raise LegacyControlProjectionError(
            "inactive_legacy_maneuver", "An active legacy maneuver is required"
        )
    if not isinstance(active, Mapping) or set(active) != _ACTIVE_FIELDS:
        raise LegacyControlProjectionError(
            "invalid_legacy_active_maneuver", "Active maneuver fields are invalid"
        )
    revision = active["revision"]
    started = active["started_turn"]
    confirmed = active["last_confirmed_turn"]
    success = active["success_fact_keys"]
    invalid = (
        not _valid_text(active["id"], 64)
        or isinstance(revision, bool) or not isinstance(revision, int) or revision < 1
        or not _valid_text(active["objective"], 160)
        or not _valid_text(active["target_hypothesis_id"], 128)
        or active["detour_side"] not in DETOUR_SIDES
        or not _valid_success_facts(success)
        or active["current_focus_fact_key"] not in success
        or isinstance(started, bool) or not isinstance(started, int) or started < 0
        or isinstance(confirmed, bool) or not isinstance(confirmed, int)
        or confirmed < started
    )
    if invalid:
        raise LegacyControlProjectionError(
            "invalid_legacy_active_maneuver", "Active maneuver values are invalid"
        )
    return active


def project_active_maneuver_intent(
    maneuver_state: Mapping[str, object],
    *,
    intent_id: str,
    goal: GoalAssignment,
    basis: NavigationBasis,
    accepted_at_ms: int,
) -> ActiveIntent:
    """Map active commitment state; all host identity and time are injected."""

    _context(goal, basis)
    active = _legacy_active(maneuver_state)
    try:
        return ActiveIntent(
            intent_id=intent_id,
            revision=active["revision"],
            goal_id=goal.goal_id,
            goal_epoch=goal.goal_epoch,
            payload=DetourTargetIntent(
                active["target_hypothesis_id"], DetourSide(active["detour_side"])
            ),
            accepted_basis=basis,
            accepted_at_ms=accepted_at_ms,
        )
    except (PhysicalAgentStateError, TypeError, ValueError) as error:
        raise LegacyControlProjectionError(
            "canonical_intent_rejected", "Canonical ActiveIntent rejected projection"
        ) from error


def _step_ids(values: Sequence[str], expected: int) -> Tuple[str, ...]:
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise LegacyControlProjectionError(
            "invalid_step_ids", "Step IDs must be an injected sequence"
        )
    result = tuple(values)
    if (
        len(result) != expected
        or any(not _valid_text(value, 128) for value in result)
        or len(set(result)) != expected
    ):
        raise LegacyControlProjectionError(
            "invalid_step_ids", "One unique step ID is required per legacy step"
        )
    return result


def _binding(
    goal: GoalAssignment,
    intent: ActiveIntent,
    basis: NavigationBasis,
    signatures: Tuple[Tuple[str, str], ...] = (),
) -> PlanBinding:
    _context(goal, basis, intent)
    try:
        return PlanBinding(
            controller_key=basis.controller_key,
            goal_id=goal.goal_id,
            goal_epoch=goal.goal_epoch,
            intent_id=intent.intent_id,
            intent_revision=intent.revision,
            frame_id=basis.frame_id,
            world_generation_id=basis.world_generation_id,
            calibration_fingerprint=basis.calibration_fingerprint,
            based_on_navigation_basis_id=basis.navigation_basis_id,
            target_geometry_signatures=signatures,
        )
    except PhysicalAgentStateError as error:
        raise LegacyControlProjectionError(
            "canonical_plan_binding_rejected", "Canonical binding rejected projection"
        ) from error


def project_local_detour_execution_plan(
    route: LocalDetourRoute,
    *,
    plan_id: str,
    plan_revision: int,
    step_ids: Sequence[str],
    goal: GoalAssignment,
    intent: ActiveIntent,
    basis: NavigationBasis,
    created_at_ms: int,
) -> ExecutionPlan:
    """Map route state for shadow comparison, never for dispatch."""

    if not isinstance(route, LocalDetourRoute):
        raise LegacyControlProjectionError(
            "invalid_legacy_route", "Expected LocalDetourRoute"
        )
    if route.status == ROUTE_INVALID:
        raise LegacyControlProjectionError(
            "invalidated_legacy_route", "Invalidated route cannot be projected"
        )
    _context(goal, basis, intent)
    if (
        route.frame_id != basis.frame_id
        or route.map_generation_id != basis.world_generation_id
        or route.based_on_map_version > basis.world_model_version
    ):
        raise LegacyControlProjectionError(
            "legacy_route_basis_mismatch", "Route is not supported by basis"
        )
    payload = intent.payload
    if (
        not isinstance(payload, DetourTargetIntent)
        or payload.target_hypothesis_id != route.target_hypothesis_id
        or payload.detour_side.value != route.detour_side
    ):
        raise LegacyControlProjectionError(
            "legacy_route_intent_mismatch", "Route target or side differs from intent"
        )
    ids = _step_ids(step_ids, len(route.waypoints))
    try:
        steps = tuple(
            WaypointStep(
                step_id,
                waypoint.x_mm,
                waypoint.y_mm,
                waypoint.heading_mdeg,
                route.position_tolerance_mm,
                route.heading_tolerance_mdeg,
            )
            for step_id, waypoint in zip(ids, route.waypoints)
        )
        signature = ((route.target_hypothesis_id, route.target_geometry_signature),)
        return ExecutionPlan(
            plan_id,
            plan_revision,
            _binding(goal, intent, basis, signature),
            steps,
            route.active_index,
            created_at_ms,
        )
    except PhysicalAgentStateError as error:
        raise LegacyControlProjectionError(
            "canonical_route_plan_rejected", "Canonical plan rejected route projection"
        ) from error


def project_navigation_plan_tail_execution_plan(
    tail: NavigationPlanTail,
    *,
    plan_id: str,
    plan_revision: int,
    step_ids: Sequence[str],
    goal: GoalAssignment,
    intent: ActiveIntent,
    basis: NavigationBasis,
    created_at_ms: int,
    now_monotonic: float,
) -> ExecutionPlan:
    """Map a live legacy tail using compatibility-only PrimitiveSteps."""

    if not isinstance(tail, NavigationPlanTail):
        raise LegacyControlProjectionError(
            "invalid_legacy_plan_tail", "Expected NavigationPlanTail"
        )
    if tail.cancelled:
        raise LegacyControlProjectionError(
            "cancelled_legacy_plan_tail", "Cancelled tail cannot be projected"
        )
    legacy_times_valid = all(
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and isfinite(float(value))
        for value in (tail.created_monotonic, tail.expires_monotonic)
    )
    invalid_time = (
        not legacy_times_valid
        or isinstance(now_monotonic, bool)
        or not isinstance(now_monotonic, (int, float))
        or not isfinite(float(now_monotonic))
        or (
            legacy_times_valid
            and now_monotonic < tail.created_monotonic
        )
    )
    if invalid_time:
        raise LegacyControlProjectionError(
            "invalid_projection_time", "Injected monotonic time is invalid"
        )
    if now_monotonic >= tail.expires_monotonic:
        raise LegacyControlProjectionError(
            "expired_legacy_plan_tail", "Expired tail cannot be projected"
        )
    _context(goal, basis, intent)
    if (
        tail.map_generation_id != basis.world_generation_id
        or tail.last_map_version > basis.world_model_version
        or tail.last_observation_state_version > basis.controller_state_version
    ):
        raise LegacyControlProjectionError(
            "legacy_plan_tail_basis_mismatch", "Tail is not supported by basis"
        )
    source, remaining = tail.source_plan, tail.remaining_actions
    shaped = (
        isinstance(source, tuple) and bool(source)
        and isinstance(remaining, tuple) and len(remaining) <= len(source)
        and all(isinstance(action, str) for action in source)
    )
    suffix_start = len(source) - len(remaining) if shaped else -1
    if (
        not shaped
        or source[suffix_start:] != remaining
        or any(action not in MOTION_ACTIONS for action in source)
    ):
        raise LegacyControlProjectionError(
            "invalid_legacy_plan_tail", "Source plan and remaining suffix differ"
        )
    active, payload = tail.active_commitment, intent.payload
    if active is not None and (
        not isinstance(active, Mapping)
        or not isinstance(payload, DetourTargetIntent)
        or active.get("revision") != intent.revision
        or active.get("target_hypothesis_id") != payload.target_hypothesis_id
        or active.get("detour_side") != payload.detour_side.value
    ):
        raise LegacyControlProjectionError(
            "legacy_plan_tail_intent_mismatch", "Tail commitment differs from intent"
        )
    ids = _step_ids(step_ids, len(source))
    try:
        steps = tuple(
            PrimitiveStep(step_id, action)
            for step_id, action in zip(ids, source)
        )
        return ExecutionPlan(
            plan_id,
            plan_revision,
            _binding(goal, intent, basis),
            steps,
            len(source) - len(remaining),
            created_at_ms,
        )
    except PhysicalAgentStateError as error:
        raise LegacyControlProjectionError(
            "canonical_tail_plan_rejected", "Canonical plan rejected tail projection"
        ) from error


__all__ = (
    "LegacyControlProjectionError",
    "project_active_maneuver_intent",
    "project_local_detour_execution_plan",
    "project_navigation_plan_tail_execution_plan",
)
