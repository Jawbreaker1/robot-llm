"""Private reducer for the single-flight plan-step command lifecycle."""

from dataclasses import replace
from typing import Optional

from ._physical_agent_core import (
    AgentPhase,
    NavigationBasis,
    PhysicalAgentStateError,
    PlanningCause,
    PlanningTicket,
    _integer,
)
from ._physical_agent_dispatch_contract import (
    ActiveDispatch,
    ControllerCommandReceipt,
    ReceiptOutcome,
    StepCommandAuthorization,
    StepDisposition,
)
from ._physical_agent_events import (
    PhysicalAgentEvent,
    StepCommandAuthorized,
    StepCommandDispatched,
    StepCommandRevoked,
    StepCommandSettlementExpired,
    StepCommandSettled,
)
from ._physical_agent_snapshot import PhysicalAgentState
from ._physical_agent_reducer_support import (
    _active_step_key,
    _new_ticket,
    _next,
    _prepared_after_basis,
    _require_phase,
    _successor,
    _ticket_after_basis,
    _verified_step_progress,
)


def _current_dispatch(
    state: PhysicalAgentState,
    authorization: StepCommandAuthorization,
    *,
    dispatched: bool,
    settlement_expired: Optional[bool] = None,
) -> ActiveDispatch:
    value = state.active_dispatch
    if (
        value is None
        or value.authorization != authorization
        or value.dispatched is not dispatched
        or (
            settlement_expired is not None
            and value.settlement_expired is not settlement_expired
        )
    ):
        raise PhysicalAgentStateError(
            "active_dispatch_mismatch",
            "command event does not match the active dispatch",
        )
    return value


def _validate_receipt(
    active_dispatch: ActiveDispatch,
    receipt: ControllerCommandReceipt,
) -> None:
    if not isinstance(receipt, ControllerCommandReceipt):
        raise PhysicalAgentStateError(
            "invalid_command_receipt",
            "step settlement requires a controller command receipt",
        )
    authorization = active_dispatch.authorization
    if (
        receipt.controller_key != authorization.controller_key
        or receipt.step_key != authorization.step_key
        or receipt.action_id != authorization.action_id
        or receipt.command_id != authorization.command_id
        or receipt.host_dispatch_sequence
        != authorization.host_dispatch_sequence
        or receipt.command_fingerprint != authorization.command_fingerprint
        or receipt.based_on_navigation_basis_id
        != authorization.based_on_navigation_basis_id
        or receipt.based_on_controller_state_version
        != authorization.based_on_controller_state_version
    ):
        raise PhysicalAgentStateError(
            "command_receipt_mismatch",
            "controller receipt does not match the dispatched command",
        )
    if receipt.received_at_host_ms < active_dispatch.dispatched_at_ms:
        raise PhysicalAgentStateError(
            "receipt_before_dispatch",
            "controller receipt predates command dispatch",
        )



def _reduce_step_command_event(
    state: PhysicalAgentState,
    event: PhysicalAgentEvent,
) -> PhysicalAgentState:
    """Reduce the single-flight command lifecycle for the active plan step."""

    if isinstance(event, StepCommandAuthorized):
        _require_phase(state, event, AgentPhase.EXECUTING)
        authorization = event.authorization
        if not isinstance(authorization, StepCommandAuthorization):
            raise PhysicalAgentStateError(
                "invalid_step_command_authorization",
                "step authorization is invalid",
            )
        if state.active_dispatch is not None:
            raise PhysicalAgentStateError(
                "active_dispatch_already_exists",
                "only one step command may be active",
            )
        if state.prepared_intent_plan is not None:
            raise PhysicalAgentStateError(
                "prepared_intent_pending",
                "a prepared intent must activate or expire before a new command",
            )
        if authorization.host_dispatch_sequence != (
            state.last_host_dispatch_sequence + 1
        ):
            raise PhysicalAgentStateError(
                "invalid_host_dispatch_sequence",
                "host dispatch sequence must advance exactly once",
            )
        if (
            authorization.controller_key != state.controller_key
            or authorization.step_key != _active_step_key(state)
            or authorization.based_on_navigation_basis_id
            != state.basis.navigation_basis_id
            or authorization.based_on_controller_state_version
            != state.basis.controller_state_version
        ):
            raise PhysicalAgentStateError(
                "step_command_authorization_mismatch",
                "authorization is not bound to the current controller and plan step",
            )
        return _next(
            state,
            last_host_dispatch_sequence=(
                authorization.host_dispatch_sequence
            ),
            active_dispatch=ActiveDispatch(authorization),
        )

    if isinstance(event, StepCommandDispatched):
        _require_phase(state, event, AgentPhase.EXECUTING)
        active_dispatch = _current_dispatch(
            state,
            event.authorization,
            dispatched=False,
        )
        if (
            state.basis.navigation_basis_id
            != event.authorization.based_on_navigation_basis_id
            or state.basis.controller_state_version
            != event.authorization.based_on_controller_state_version
        ):
            raise PhysicalAgentStateError(
                "stale_step_command_authorization",
                "current navigation evidence no longer matches authorization",
            )
        return _next(
            state,
            active_dispatch=ActiveDispatch(
                authorization=active_dispatch.authorization,
                dispatched_at_ms=event.dispatched_at_ms,
                settle_by_host_ms=event.settle_by_host_ms,
            ),
        )

    if isinstance(event, StepCommandRevoked):
        _require_phase(state, event, AgentPhase.EXECUTING)
        active_dispatch = _current_dispatch(
            state,
            event.authorization,
            dispatched=False,
        )
        _integer(
            "command_revoked_at_ms",
            event.revoked_at_ms,
            active_dispatch.authorization.issued_at_ms,
        )
        return _next(state, active_dispatch=None)

    if isinstance(event, StepCommandSettlementExpired):
        _require_phase(state, event, AgentPhase.EXECUTING)
        active_dispatch = _current_dispatch(
            state,
            event.authorization,
            dispatched=True,
            settlement_expired=False,
        )
        _integer(
            "command_settlement_expired_at_host_ms",
            event.observed_at_host_ms,
            active_dispatch.settle_by_host_ms,
        )
        return _next(
            state,
            active_dispatch=replace(
                active_dispatch,
                settlement_expired_at_host_ms=event.observed_at_host_ms,
            ),
        )

    if not isinstance(event, StepCommandSettled):
        raise PhysicalAgentStateError(
            "unsupported_step_command_event",
            "event is not part of the step-command lifecycle",
        )

    _require_phase(state, event, AgentPhase.EXECUTING)
    active_dispatch = state.active_dispatch
    if active_dispatch is None or not active_dispatch.dispatched:
        raise PhysicalAgentStateError(
            "active_dispatch_mismatch",
            "settlement requires one previously dispatched command",
        )
    _validate_receipt(active_dispatch, event.receipt)
    _successor(state, event.resulting_basis)
    if (
        event.receipt.resulting_controller_state_version
        != event.resulting_basis.controller_state_version
    ):
        raise PhysicalAgentStateError(
            "receipt_resulting_version_mismatch",
            "receipt and resulting basis controller versions differ",
        )
    if not isinstance(event.disposition, StepDisposition):
        raise PhysicalAgentStateError(
            "invalid_step_disposition",
            "step settlement disposition is invalid",
        )
    if (
        not active_dispatch.settlement_expired
        and event.receipt.received_at_host_ms
        >= active_dispatch.settle_by_host_ms
    ):
        raise PhysicalAgentStateError(
            "step_command_settlement_expiry_not_recorded",
            "late command receipt requires an explicit settlement-expired event",
        )
    if (
        active_dispatch.settlement_expired
        and event.receipt.outcome == ReceiptOutcome.COMPLETED
    ):
        raise PhysicalAgentStateError(
            "completed_receipt_after_settlement_expiry",
            "an expired command may not advance from a completed receipt",
        )
    if (
        event.receipt.outcome != ReceiptOutcome.COMPLETED
        and event.disposition != StepDisposition.BLOCKED
    ):
        raise PhysicalAgentStateError(
            "invalid_receipt_disposition",
            "non-completed commands may only block the active plan",
        )

    if event.disposition == StepDisposition.BLOCKED:
        if not isinstance(event.replan_ticket, PlanningTicket):
            raise PhysicalAgentStateError(
                "missing_blocked_replan_ticket",
                "blocked command requires a basis-bound replan ticket",
            )
        if event.replan_ticket.basis != event.resulting_basis:
            raise PhysicalAgentStateError(
                "planning_ticket_basis_mismatch",
                "blocked replan ticket must bind the resulting basis",
            )
        _new_ticket(
            replace(
                state,
                basis=event.resulting_basis,
                prepared_intent_plan=None,
                planning_ticket=None,
            ),
            event.replan_ticket,
            (PlanningCause.UNCERTAINTY, PlanningCause.REPLAN_REQUIRED),
        )
        return _next(
            state,
            phase=AgentPhase.PLANNING,
            basis=event.resulting_basis,
            compile_pending=False,
            plan=None,
            prepared_intent_plan=None,
            active_dispatch=None,
            planning_ticket=event.replan_ticket,
        )

    if event.replan_ticket is not None:
        raise PhysicalAgentStateError(
            "unexpected_step_replan_ticket",
            "only a blocked command may carry a replan ticket",
        )
    if event.disposition == StepDisposition.CONTINUE:
        return _next(
            state,
            basis=event.resulting_basis,
            active_dispatch=None,
            prepared_intent_plan=_prepared_after_basis(
                state,
                event.resulting_basis,
            ),
            planning_ticket=_ticket_after_basis(
                state,
                event.resulting_basis,
            ),
        )

    active_plan = state.plan
    progress = _verified_step_progress(state, event.resulting_basis)
    next_cursor = active_plan.cursor + 1
    if next_cursor < len(active_plan.steps):
        return _next(
            state,
            basis=event.resulting_basis,
            plan=replace(active_plan, cursor=next_cursor),
            active_dispatch=None,
            prepared_intent_plan=_prepared_after_basis(
                state,
                event.resulting_basis,
            ),
            planning_ticket=_ticket_after_basis(
                state,
                event.resulting_basis,
            ),
            intent_progress=progress,
        )
    retained_ticket = _ticket_after_basis(state, event.resulting_basis)
    if retained_ticket is not None:
        return _next(
            state,
            phase=AgentPhase.PLANNING,
            basis=event.resulting_basis,
            compile_pending=False,
            plan=None,
            prepared_intent_plan=_prepared_after_basis(
                state,
                event.resulting_basis,
            ),
            active_dispatch=None,
            planning_ticket=retained_ticket,
            intent_progress=progress,
        )
    return _next(
        state,
        phase=AgentPhase.PLANNING,
        basis=event.resulting_basis,
        compile_pending=True,
        plan=None,
        prepared_intent_plan=None,
        active_dispatch=None,
        planning_ticket=None,
        intent_progress=progress,
    )
