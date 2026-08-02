"""Private reducer for goal, ticket, intent, and basis events."""

from .physical_agent_contract import (
    ActiveIntent,
    AgentPhase,
    ExecutionPlan,
    GoalActivated,
    GoalAssignment,
    GoalOutcome,
    GoalTerminal,
    IntentAccepted,
    NavigationBasis,
    NavigationBasisUpdated,
    PhysicalAgentEvent,
    PhysicalAgentState,
    PhysicalAgentStateError,
    PlanningAbortRequested,
    PlanningCause,
    PlanningHeld,
    PlanningRequested,
    PlanningTicket,
    PlanningTicketConsumed,
    PlanningTicketExpired,
    _integer,
)
from ._physical_agent_reducer_support import (
    _current_ticket,
    _dispatch_for_stopping,
    _initial_progress,
    _new_plan,
    _new_ticket,
    _next,
    _require_no_active_dispatch,
    _require_phase,
    _successor,
)


def _reduce_planning_event(
    state: PhysicalAgentState,
    event: PhysicalAgentEvent,
) -> PhysicalAgentState:
    if isinstance(event, GoalActivated):
        _require_phase(state, event, AgentPhase.IDLE, AgentPhase.TERMINAL)
        if (
            not isinstance(event.goal, GoalAssignment)
            or not isinstance(event.basis, NavigationBasis)
            or not isinstance(event.ticket, PlanningTicket)
            or event.goal.goal_epoch <= state.goal_epoch
            or event.basis.controller_key != state.controller_key
            or event.basis.goal_epoch != event.goal.goal_epoch
            or event.ticket.cause != PlanningCause.NEW_GOAL
            or event.ticket.consumed
            or event.ticket.basis != event.basis
        ):
            raise PhysicalAgentStateError(
                "invalid_goal_activation", "goal activation bindings are invalid"
            )
        return _next(
            state,
            phase=AgentPhase.PLANNING,
            goal_epoch=event.goal.goal_epoch,
            plan_revision=0,
            compile_pending=False,
            goal=event.goal,
            basis=event.basis,
            intent=None,
            intent_progress=None,
            plan=None,
            active_dispatch=None,
            planning_ticket=event.ticket,
            terminal=None,
        )

    if isinstance(event, PlanningTicketConsumed):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
        value = _current_ticket(
            state, event.ticket_id, event.based_on_basis, False
        )
        return _next(
            state, planning_ticket=value.consume(event.consumed_at_ms)
        )

    if isinstance(event, PlanningTicketExpired):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
        value = _current_ticket(
            state,
            event.ticket_id,
            event.based_on_basis,
            None,
        )
        _integer("planning_ticket_observed_at_ms", event.observed_at_ms)
        if event.observed_at_ms < value.valid_until_ms:
            raise PhysicalAgentStateError(
                "planning_ticket_not_expired",
                "planning ticket cannot be dismissed before expiry",
            )
        return _next(
            state,
            compile_pending=False,
            planning_ticket=None,
        )

    if isinstance(event, IntentAccepted):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
        _require_no_active_dispatch(state, event)
        _current_ticket(state, event.ticket_id, event.based_on_basis, True)
        value = event.intent
        if (
            not isinstance(value, ActiveIntent)
            or not isinstance(event.plan, ExecutionPlan)
            or value.goal_id != state.goal.goal_id
            or value.goal_epoch != state.goal_epoch
            or value.accepted_basis != event.based_on_basis
        ):
            raise PhysicalAgentStateError(
                "invalid_accepted_intent",
                "accepted intent is not bound to the ticket and goal",
            )
        if state.intent is None:
            valid_revision = value.revision == 1
        elif value.intent_id == state.intent.intent_id:
            valid_revision = value.revision == state.intent.revision + 1
        else:
            valid_revision = value.revision == 1
        if not valid_revision:
            raise PhysicalAgentStateError(
                "invalid_intent_revision", "accepted intent revision is invalid"
            )
        _new_plan(state, event.plan, value, state.basis)
        progress = _initial_progress(value, state.basis)
        return _next(
            state,
            phase=AgentPhase.EXECUTING,
            compile_pending=False,
            intent=value,
            intent_progress=progress,
            plan=event.plan,
            plan_revision=event.plan.revision,
            active_dispatch=None,
            planning_ticket=None,
        )

    if isinstance(event, PlanningAbortRequested):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
        _current_ticket(
            state,
            event.ticket_id,
            event.based_on_basis,
            True,
        )
        if (
            not isinstance(event.terminal, GoalTerminal)
            or event.terminal.outcome == GoalOutcome.SUCCEEDED
        ):
            raise PhysicalAgentStateError(
                "invalid_planning_abort_outcome",
                "planning abort requires CANCELLED or FAILED",
            )
        return _next(
            state,
            phase=AgentPhase.STOPPING,
            compile_pending=False,
            plan=None,
            active_dispatch=_dispatch_for_stopping(state),
            planning_ticket=None,
            terminal=event.terminal,
        )

    if isinstance(event, PlanningHeld):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
        _current_ticket(state, event.ticket_id, event.based_on_basis, True)
        return _next(
            state,
            compile_pending=False,
            planning_ticket=None,
        )

    if isinstance(event, PlanningRequested):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
        if state.planning_ticket is not None:
            raise PhysicalAgentStateError(
                "planning_ticket_already_active",
                "only one planning ticket may be active",
            )
        _new_ticket(
            state,
            event.ticket,
            (PlanningCause.UNCERTAINTY, PlanningCause.REPLAN_REQUIRED),
        )
        return _next(
            state,
            compile_pending=False,
            planning_ticket=event.ticket,
        )

    if isinstance(event, NavigationBasisUpdated):
        _require_phase(state, event, AgentPhase.PLANNING, AgentPhase.EXECUTING)
        _successor(state, event.basis)
        active_ticket = state.planning_ticket
        if active_ticket is not None and not active_ticket.basis.decision_equivalent(
            event.basis
        ):
            active_ticket = None
        return _next(
            state,
            basis=event.basis,
            compile_pending=(
                state.compile_pending
                and state.basis.decision_equivalent(event.basis)
            ),
            planning_ticket=active_ticket,
        )

    raise PhysicalAgentStateError(
        "unsupported_state_event", "state event type is unsupported"
    )
