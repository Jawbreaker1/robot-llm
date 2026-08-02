"""Private reducer for physical stop and terminal lifecycle events."""

from .physical_agent_contract import (
    AgentPhase,
    ControllerCommandReceipt,
    GoalCompletionRequested,
    GoalOutcome,
    GoalTerminal,
    NavigationBasis,
    PhysicalAgentEvent,
    PhysicalAgentState,
    PhysicalAgentStateError,
    ReceiptOutcome,
    StopRequested,
    StopVerified,
    TerminalCleared,
    _integer,
)
from ._physical_agent_command_reducer import _validate_receipt
from ._physical_agent_reducer_support import (
    _dispatch_for_stopping,
    _next,
    _require_phase,
    _successor,
)


def _reduce_stop_event(
    state: PhysicalAgentState,
    event: PhysicalAgentEvent,
) -> PhysicalAgentState:
    if isinstance(event, GoalCompletionRequested):
        _require_phase(state, event, AgentPhase.PLANNING, AgentPhase.EXECUTING)
        _successor(state, event.resulting_basis)
        if (
            not isinstance(event.terminal, GoalTerminal)
            or event.terminal.outcome != GoalOutcome.SUCCEEDED
        ):
            raise PhysicalAgentStateError(
                "invalid_goal_completion",
                "goal completion requires a SUCCEEDED terminal outcome",
            )
        return _next(
            state,
            phase=AgentPhase.STOPPING,
            basis=event.resulting_basis,
            compile_pending=False,
            plan=None,
            active_dispatch=_dispatch_for_stopping(state),
            planning_ticket=None,
            terminal=event.terminal,
        )

    if isinstance(event, StopRequested):
        _require_phase(state, event, AgentPhase.PLANNING, AgentPhase.EXECUTING)
        if (
            not isinstance(event.terminal, GoalTerminal)
            or event.terminal.outcome == GoalOutcome.SUCCEEDED
        ):
            raise PhysicalAgentStateError(
                "invalid_stop_outcome",
                "external stop requires CANCELLED or FAILED",
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

    if isinstance(event, StopVerified):
        _require_phase(state, event, AgentPhase.STOPPING)
        _integer("stop_verified_at_ms", event.verified_at_ms)
        if event.verified_at_ms < state.terminal.completed_at_ms:
            raise PhysicalAgentStateError(
                "stop_verified_before_terminal",
                "stop verification predates the terminal request",
            )
        resulting_basis = state.basis
        if state.active_dispatch is None:
            if (
                event.dispatch_receipt is not None
                or event.resulting_basis is not None
            ):
                raise PhysicalAgentStateError(
                    "unexpected_stop_dispatch_receipt",
                    "stop without an active dispatch cannot carry a command receipt",
                )
        else:
            if (
                not isinstance(event.dispatch_receipt, ControllerCommandReceipt)
                or not isinstance(event.resulting_basis, NavigationBasis)
            ):
                raise PhysicalAgentStateError(
                    "missing_stop_dispatch_receipt",
                    "stop with an active dispatch requires its receipt and basis",
                )
            _validate_receipt(state.active_dispatch, event.dispatch_receipt)
            if (
                event.dispatch_receipt.outcome != ReceiptOutcome.STOPPED
                or not event.dispatch_receipt.stop_confirmed
            ):
                raise PhysicalAgentStateError(
                    "invalid_stop_dispatch_receipt",
                    "active dispatch must be reconciled by a confirmed STOPPED receipt",
                )
            _successor(state, event.resulting_basis)
            if (
                event.dispatch_receipt.resulting_controller_state_version
                != event.resulting_basis.controller_state_version
            ):
                raise PhysicalAgentStateError(
                    "receipt_resulting_version_mismatch",
                    "receipt and resulting basis controller versions differ",
                )
            if (
                event.dispatch_receipt.received_at_host_ms
                < state.terminal.completed_at_ms
                or event.verified_at_ms
                < event.dispatch_receipt.received_at_host_ms
            ):
                raise PhysicalAgentStateError(
                    "stop_receipt_time_mismatch",
                    "stop receipt and verification must follow the stop request",
                )
            resulting_basis = event.resulting_basis
        return _next(
            state,
            phase=AgentPhase.TERMINAL,
            basis=resulting_basis,
            compile_pending=False,
            intent=None,
            intent_progress=None,
            active_dispatch=None,
        )

    if isinstance(event, TerminalCleared):
        _require_phase(state, event, AgentPhase.TERMINAL)
        _integer("terminal_cleared_at_ms", event.cleared_at_ms)
        if event.cleared_at_ms < state.terminal.completed_at_ms:
            raise PhysicalAgentStateError(
                "terminal_cleared_before_completion",
                "terminal state cannot be cleared before completion",
            )
        return _next(
            state,
            phase=AgentPhase.IDLE,
            plan_revision=0,
            compile_pending=False,
            goal=None,
            basis=None,
            intent=None,
            intent_progress=None,
            plan=None,
            active_dispatch=None,
            planning_ticket=None,
            terminal=None,
        )

    raise PhysicalAgentStateError(
        "unsupported_state_event", "state event type is unsupported"
    )
