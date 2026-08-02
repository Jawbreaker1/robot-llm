"""Private helpers shared by canonical physical-agent reducers."""

from dataclasses import replace
from typing import Optional, Tuple

from ._physical_agent_core import (
    ActiveIntent,
    AgentPhase,
    ExecutionPlan,
    IntentProgress,
    NavigationBasis,
    PhysicalAgentStateError,
    PlanningCause,
    PlanningTicket,
)
from ._physical_agent_dispatch_contract import ActiveDispatch, PlanStepKey
from ._physical_agent_prepared_contract import PreparedIntentPlan
from ._physical_agent_snapshot import PhysicalAgentState


def _require_phase(
    state: PhysicalAgentState,
    event: object,
    *phases: AgentPhase
) -> None:
    if state.phase not in phases:
        raise PhysicalAgentStateError(
            "illegal_phase_transition",
            "{} is not valid while the agent is {}".format(
                type(event).__name__, state.phase.value
            ),
        )


def _next(state: PhysicalAgentState, **changes) -> PhysicalAgentState:
    return replace(
        state,
        agent_state_version=state.agent_state_version + 1,
        **changes
    )


def _current_ticket(
    state: PhysicalAgentState,
    ticket: PlanningTicket,
    consumed: Optional[bool],
) -> PlanningTicket:
    value = state.planning_ticket
    if (
        not isinstance(ticket, PlanningTicket)
        or value != ticket
        or (consumed is not None and value.consumed is not consumed)
    ):
        raise PhysicalAgentStateError(
            "planning_ticket_mismatch",
            "planning response does not match the current ticket",
        )
    if state.basis is None or not value.basis.decision_equivalent(state.basis):
        raise PhysicalAgentStateError(
            "stale_planning_basis",
            "planning response is stale for the current evidence",
        )
    return value


def _new_ticket(
    state: PhysicalAgentState,
    value: PlanningTicket,
    causes: Tuple[PlanningCause, ...],
) -> None:
    if value.consumed or value.cause not in causes:
        raise PhysicalAgentStateError(
            "invalid_new_planning_ticket",
            "new planning ticket cause or lifecycle is invalid",
        )
    if state.basis is None or value.basis != state.basis:
        raise PhysicalAgentStateError(
            "planning_ticket_basis_mismatch",
            "new planning ticket must bind the exact current basis",
        )


def _successor(state: PhysicalAgentState, value: NavigationBasis) -> None:
    if state.basis is None:
        raise PhysicalAgentStateError(
            "missing_current_basis", "active state has no navigation basis"
        )
    value.assert_successor_of(state.basis)


def _require_no_active_dispatch(
    state: PhysicalAgentState,
    event: object,
) -> None:
    if state.active_dispatch is not None:
        raise PhysicalAgentStateError(
            "active_dispatch_conflict",
            "{} cannot replace a plan while a command is active".format(
                type(event).__name__
            ),
        )


def _require_no_prepared_intent(
    state: PhysicalAgentState,
    event: object,
) -> None:
    if state.prepared_intent_plan is not None:
        raise PhysicalAgentStateError(
            "prepared_intent_conflict",
            "{} cannot bypass a prepared intent plan".format(
                type(event).__name__
            ),
        )


def _active_step_key(state: PhysicalAgentState) -> PlanStepKey:
    active_plan = state.plan
    return PlanStepKey(
        plan_id=active_plan.plan_id,
        plan_revision=active_plan.revision,
        cursor=active_plan.cursor,
        step_id=active_plan.active_step.step_id,
    )




def _ticket_after_basis(
    state: PhysicalAgentState,
    resulting_basis: NavigationBasis,
) -> Optional[PlanningTicket]:
    value = state.planning_ticket
    if value is not None and not value.basis.decision_equivalent(
        resulting_basis
    ):
        return None
    return value


def _prepared_after_basis(
    state: PhysicalAgentState,
    resulting_basis: NavigationBasis,
) -> Optional[PreparedIntentPlan]:
    value = state.prepared_intent_plan
    if value is not None and not value.compilation_basis.decision_equivalent(
        resulting_basis
    ):
        return None
    return value


def _dispatch_for_stopping(state: PhysicalAgentState) -> Optional[ActiveDispatch]:
    value = state.active_dispatch
    return value if value is not None and value.dispatched else None


def _new_plan(
    state: PhysicalAgentState,
    value: ExecutionPlan,
    intent: ActiveIntent,
    nav_basis: NavigationBasis,
) -> None:
    if value.cursor != 0 or value.complete:
        raise PhysicalAgentStateError(
            "new_plan_not_at_start",
            "a newly accepted plan must start at cursor zero",
        )
    if value.revision != state.plan_revision + 1:
        raise PhysicalAgentStateError(
            "invalid_plan_revision", "plan revision must advance exactly once"
        )
    if value.binding.based_on_navigation_basis_id != nav_basis.navigation_basis_id:
        raise PhysicalAgentStateError(
            "plan_basis_mismatch",
            "new plan was not compiled from the supplied navigation basis",
        )
    value.binding.assert_matches(
        controller_key=state.controller_key,
        goal=state.goal,
        intent=intent,
        basis=nav_basis,
    )


def _validate_new_intent_plan(
    state: PhysicalAgentState,
    intent: ActiveIntent,
    plan: ExecutionPlan,
    accepted_basis: NavigationBasis,
    current_basis: NavigationBasis,
) -> None:
    if (
        not isinstance(intent, ActiveIntent)
        or not isinstance(plan, ExecutionPlan)
        or intent.goal_id != state.goal.goal_id
        or intent.goal_epoch != state.goal_epoch
        or intent.accepted_basis != accepted_basis
    ):
        raise PhysicalAgentStateError(
            "invalid_accepted_intent",
            "accepted intent is not bound to the ticket and goal",
        )
    if state.intent is None:
        valid_revision = intent.revision == 1
    elif intent.intent_id == state.intent.intent_id:
        valid_revision = intent.revision == state.intent.revision + 1
    else:
        valid_revision = intent.revision == 1
    if not valid_revision:
        raise PhysicalAgentStateError(
            "invalid_intent_revision",
            "accepted intent revision is invalid",
        )
    _new_plan(state, plan, intent, current_basis)


def _initial_progress(intent: ActiveIntent, nav_basis: NavigationBasis) -> IntentProgress:
    value = IntentProgress(
        plan_attempts=1,
        completed_steps=0,
        completed_steps_at_plan_start=0,
        consecutive_no_progress_plans=0,
        last_progress_basis_id=nav_basis.navigation_basis_id,
    )
    if value.plan_attempts > intent.policy.max_plan_attempts:
        raise PhysicalAgentStateError(
            "intent_plan_attempt_budget_exhausted",
            "intent plan-attempt budget is exhausted",
        )
    return value


def _recompiled_progress(state: PhysicalAgentState) -> IntentProgress:
    current = state.intent_progress
    made_progress = (
        current.completed_steps > current.completed_steps_at_plan_start
    )
    attempts = current.plan_attempts + 1
    no_progress = (
        0
        if made_progress
        else current.consecutive_no_progress_plans + 1
    )
    if attempts > state.intent.policy.max_plan_attempts:
        raise PhysicalAgentStateError(
            "intent_plan_attempt_budget_exhausted",
            "intent plan-attempt budget is exhausted",
        )
    if no_progress > state.intent.policy.max_consecutive_no_progress_plans:
        raise PhysicalAgentStateError(
            "intent_no_progress_budget_exhausted",
            "intent made no progress across too many plans",
        )
    return replace(
        current,
        plan_attempts=attempts,
        completed_steps_at_plan_start=current.completed_steps,
        consecutive_no_progress_plans=no_progress,
    )


def _verified_step_progress(
    state: PhysicalAgentState,
    resulting_basis: NavigationBasis,
) -> IntentProgress:
    return replace(
        state.intent_progress,
        completed_steps=state.intent_progress.completed_steps + 1,
        consecutive_no_progress_plans=0,
        last_progress_basis_id=resulting_basis.navigation_basis_id,
    )
