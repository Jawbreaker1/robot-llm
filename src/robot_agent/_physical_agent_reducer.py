"""Private router for canonical physical-agent transition reducers."""

from .physical_agent_contract import (
    GoalActivated,
    GoalCompletionRequested,
    IntentPrepared,
    NavigationBasisUpdated,
    PhysicalAgentEvent,
    PhysicalAgentState,
    PhysicalAgentStateError,
    PlanRecompiled,
    PlanningAbortRequested,
    PlanningHeld,
    PlanningRequested,
    PlanningTicketConsumed,
    PlanningTicketExpired,
    PreparedIntentAccepted,
    PreparedIntentExpired,
    ReplanRequested,
    StopRequested,
    StopVerified,
    StepCommandAuthorized,
    StepCommandDispatched,
    StepCommandRevoked,
    StepCommandSettlementExpired,
    StepCommandSettled,
    TerminalCleared,
)
from ._physical_agent_command_reducer import _reduce_step_command_event
from ._physical_agent_plan_lifecycle_reducer import (
    _reduce_plan_lifecycle_event,
)
from ._physical_agent_planning_reducer import _reduce_planning_event
from ._physical_agent_prepared_reducer import _reduce_prepared_intent_event
from ._physical_agent_stop_reducer import _reduce_stop_event


_PLANNING_EVENTS = (
    GoalActivated,
    PlanningTicketConsumed,
    PlanningTicketExpired,
    PlanningAbortRequested,
    PlanningHeld,
    PlanningRequested,
    NavigationBasisUpdated,
)
_COMMAND_EVENTS = (
    StepCommandAuthorized,
    StepCommandDispatched,
    StepCommandRevoked,
    StepCommandSettlementExpired,
    StepCommandSettled,
)
_PREPARED_INTENT_EVENTS = (
    IntentPrepared,
    PreparedIntentAccepted,
    PreparedIntentExpired,
)
_PLAN_LIFECYCLE_EVENTS = (PlanRecompiled, ReplanRequested)
_STOP_EVENTS = (
    GoalCompletionRequested,
    StopRequested,
    StopVerified,
    TerminalCleared,
)


def reduce_physical_agent_state(
    state: PhysicalAgentState,
    event: PhysicalAgentEvent,
) -> PhysicalAgentState:
    """Apply exactly one legal event to an immutable controller state."""

    if not isinstance(state, PhysicalAgentState):
        raise PhysicalAgentStateError("invalid_state", "state is invalid")

    if isinstance(event, _PLANNING_EVENTS):
        return _reduce_planning_event(state, event)
    if isinstance(event, _COMMAND_EVENTS):
        return _reduce_step_command_event(state, event)
    if isinstance(event, _PREPARED_INTENT_EVENTS):
        return _reduce_prepared_intent_event(state, event)
    if isinstance(event, _PLAN_LIFECYCLE_EVENTS):
        return _reduce_plan_lifecycle_event(state, event)
    if isinstance(event, _STOP_EVENTS):
        return _reduce_stop_event(state, event)

    raise PhysicalAgentStateError(
        "unsupported_state_event", "state event type is unsupported"
    )
