"""Private reducer for durable prepared-intent activation."""

from ._physical_agent_core import (
    AgentPhase,
    PhysicalAgentStateError,
    _integer,
)
from ._physical_agent_events import (
    IntentPrepared,
    PhysicalAgentEvent,
    PreparedIntentAccepted,
    PreparedIntentExpired,
)
from ._physical_agent_prepared_contract import PreparedIntentPlan
from ._physical_agent_reducer_support import (
    _current_ticket,
    _initial_progress,
    _next,
    _require_no_active_dispatch,
    _require_phase,
    _validate_new_intent_plan,
)
from ._physical_agent_snapshot import PhysicalAgentState


def _current_prepared(
    state: PhysicalAgentState,
    *,
    prepared: PreparedIntentPlan,
) -> PreparedIntentPlan:
    value = state.prepared_intent_plan
    if value is None or value != prepared:
        raise PhysicalAgentStateError(
            "prepared_intent_mismatch",
            "prepared intent event does not match canonical state",
        )
    _current_ticket(
        state,
        prepared.ticket,
        True,
    )
    return value


def _reduce_prepared_intent_event(
    state: PhysicalAgentState,
    event: PhysicalAgentEvent,
) -> PhysicalAgentState:
    if isinstance(event, IntentPrepared):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
        if state.prepared_intent_plan is not None:
            raise PhysicalAgentStateError(
                "prepared_intent_already_exists",
                "only one prepared intent plan may be canonical",
            )
        prepared = event.prepared
        if not isinstance(prepared, PreparedIntentPlan):
            raise PhysicalAgentStateError(
                "invalid_prepared_intent_plan",
                "intent preparation requires PreparedIntentPlan",
            )
        ticket = _current_ticket(
            state,
            prepared.ticket,
            True,
        )
        if (
            prepared.prepared_at_ms < ticket.consumed_at_ms
            or prepared.valid_until_ms > ticket.valid_until_ms
        ):
            raise PhysicalAgentStateError(
                "prepared_intent_ticket_window_mismatch",
                "prepared intent must remain inside its consumed ticket window",
            )
        _validate_new_intent_plan(
            state,
            prepared.intent,
            prepared.plan,
            prepared.ticket_basis,
            prepared.compilation_basis,
        )
        if not prepared.compilation_basis.decision_equivalent(state.basis):
            raise PhysicalAgentStateError(
                "stale_prepared_intent",
                "prepared intent no longer matches decision-relevant evidence",
            )
        state.basis.assert_successor_of(prepared.compilation_basis)
        return _next(state, prepared_intent_plan=prepared)

    if isinstance(event, PreparedIntentAccepted):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
        _require_no_active_dispatch(state, event)
        prepared = _current_prepared(
            state,
            prepared=event.prepared,
        )
        _integer(
            "prepared_intent_accepted_at_ms",
            event.accepted_at_ms,
            prepared.prepared_at_ms,
        )
        if event.accepted_at_ms >= prepared.valid_until_ms:
            raise PhysicalAgentStateError(
                "prepared_intent_expired",
                "prepared intent cannot activate at or after expiry",
            )
        if not prepared.compilation_basis.decision_equivalent(state.basis):
            raise PhysicalAgentStateError(
                "stale_prepared_intent",
                "prepared intent no longer matches decision-relevant evidence",
            )
        _validate_new_intent_plan(
            state,
            prepared.intent,
            prepared.plan,
            prepared.ticket_basis,
            prepared.compilation_basis,
        )
        state.basis.assert_successor_of(prepared.compilation_basis)
        progress = _initial_progress(prepared.intent, state.basis)
        return _next(
            state,
            phase=AgentPhase.EXECUTING,
            compile_pending=False,
            intent=prepared.intent,
            intent_progress=progress,
            plan=prepared.plan,
            plan_revision=prepared.plan.revision,
            prepared_intent_plan=None,
            active_dispatch=None,
            planning_ticket=None,
        )

    if isinstance(event, PreparedIntentExpired):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
        prepared = _current_prepared(
            state,
            prepared=event.prepared,
        )
        _integer(
            "prepared_intent_expired_at_ms",
            event.observed_at_ms,
            prepared.valid_until_ms,
        )
        return _next(
            state,
            compile_pending=False,
            prepared_intent_plan=None,
            planning_ticket=None,
        )

    raise PhysicalAgentStateError(
        "unsupported_state_event", "state event type is unsupported"
    )
