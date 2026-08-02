"""Private reducer for deterministic plan lifecycle events."""

from dataclasses import replace

from .physical_agent_contract import (
    AgentPhase,
    ExecutionPlan,
    PhysicalAgentEvent,
    PhysicalAgentState,
    PhysicalAgentStateError,
    PlanRecompiled,
    PlanningCause,
    ReplanRequested,
    _identifier,
)
from ._physical_agent_reducer_support import (
    _new_plan,
    _new_ticket,
    _next,
    _recompiled_progress,
    _require_no_active_dispatch,
    _require_phase,
    _successor,
)


def _reduce_plan_lifecycle_event(
    state: PhysicalAgentState,
    event: PhysicalAgentEvent,
) -> PhysicalAgentState:
    if isinstance(event, PlanRecompiled):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
        _require_no_active_dispatch(state, event)
        if state.phase == AgentPhase.PLANNING and not state.compile_pending:
            raise PhysicalAgentStateError(
                "deterministic_compile_not_pending",
                "ticket-free deterministic compilation was not requested",
            )
        _successor(state, event.resulting_basis)
        if not isinstance(event.plan, ExecutionPlan):
            raise PhysicalAgentStateError(
                "invalid_recompiled_plan", "recompiled plan is invalid"
            )
        _new_plan(state, event.plan, state.intent, event.resulting_basis)
        progress = _recompiled_progress(state)
        return _next(
            state,
            phase=AgentPhase.EXECUTING,
            basis=event.resulting_basis,
            compile_pending=False,
            plan=event.plan,
            plan_revision=event.plan.revision,
            intent_progress=progress,
            active_dispatch=None,
            planning_ticket=None,
        )

    if isinstance(event, ReplanRequested):
        _require_phase(state, event, AgentPhase.EXECUTING)
        _require_no_active_dispatch(state, event)
        _identifier("replan_reason", event.reason, 160)
        _successor(state, event.resulting_basis)
        if event.ticket.basis != event.resulting_basis:
            raise PhysicalAgentStateError(
                "planning_ticket_basis_mismatch",
                "replan ticket must bind the resulting basis",
            )
        _new_ticket(
            replace(state, basis=event.resulting_basis),
            event.ticket,
            (PlanningCause.UNCERTAINTY, PlanningCause.REPLAN_REQUIRED),
        )
        return _next(
            state,
            phase=AgentPhase.PLANNING,
            basis=event.resulting_basis,
            compile_pending=False,
            plan=None,
            active_dispatch=None,
            planning_ticket=event.ticket,
        )

    raise PhysicalAgentStateError(
        "unsupported_state_event", "state event type is unsupported"
    )
