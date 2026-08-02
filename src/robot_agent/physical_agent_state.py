"""Pure reducer and stable public import surface for physical agent state."""

from dataclasses import replace
import threading
from typing import Optional, Tuple

from .physical_agent_contract import (
    ActiveDispatch,
    ActiveIntent,
    AgentPhase,
    ControllerCommandReceipt,
    ControllerKey,
    DetourSide,
    DetourTargetIntent,
    ExecutionPlan,
    FollowDirectionIntent,
    GoalActivated,
    GoalAssignment,
    GoalCompletionRequested,
    GoalOutcome,
    GoalTerminal,
    IntentAccepted,
    IntentPolicy,
    IntentProgress,
    MAX_PLANNING_TICKET_TTL_MS,
    MAX_STEP_COMMAND_SETTLE_MS,
    MAX_STEP_COMMAND_START_TTL_MS,
    NavigationBasis,
    NavigationBasisUpdated,
    PhysicalAgentEvent,
    PhysicalAgentState,
    PhysicalAgentStateError,
    PlanBinding,
    PlanRecompiled,
    PlanStep,
    PlanStepKey,
    PlanningAbortRequested,
    PlanningCause,
    PlanningHeld,
    PlanningRequested,
    PlanningTicket,
    PlanningTicketConsumed,
    PlanningTicketExpired,
    PrimitiveStep,
    ReceiptOutcome,
    ReplanRequested,
    ScanTargetIntent,
    SensorStep,
    StopRequested,
    StopVerified,
    StepCommandAuthorization,
    StepCommandAuthorized,
    StepCommandDispatched,
    StepCommandRevoked,
    StepCommandSettlementExpired,
    StepCommandSettled,
    StepDisposition,
    TerminalCleared,
    WaypointStep,
    _identifier,
    _integer,
)


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
    ticket_id: str,
    based_on_basis: NavigationBasis,
    consumed: Optional[bool],
) -> PlanningTicket:
    value = state.planning_ticket
    if (
        value is None
        or value.ticket_id != ticket_id
        or value.basis != based_on_basis
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


def _active_step_key(state: PhysicalAgentState) -> PlanStepKey:
    active_plan = state.plan
    return PlanStepKey(
        plan_id=active_plan.plan_id,
        plan_revision=active_plan.revision,
        cursor=active_plan.cursor,
        step_id=active_plan.active_step.step_id,
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


def _dispatch_for_stopping(state: PhysicalAgentState) -> Optional[ActiveDispatch]:
    value = state.active_dispatch
    return value if value is not None and value.dispatched else None


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
            replace(state, basis=event.resulting_basis),
            event.replan_ticket,
            (PlanningCause.UNCERTAINTY, PlanningCause.REPLAN_REQUIRED),
        )
        return _next(
            state,
            phase=AgentPhase.PLANNING,
            basis=event.resulting_basis,
            compile_pending=False,
            plan=None,
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
            planning_ticket=_ticket_after_basis(
                state,
                event.resulting_basis,
            ),
            intent_progress=progress,
        )
    return _next(
        state,
        phase=AgentPhase.PLANNING,
        basis=event.resulting_basis,
        compile_pending=True,
        plan=None,
        active_dispatch=None,
        planning_ticket=None,
        intent_progress=progress,
    )


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


def reduce_physical_agent_state(
    state: PhysicalAgentState, event: PhysicalAgentEvent
) -> PhysicalAgentState:
    """Apply exactly one legal event to an immutable controller state."""

    if not isinstance(state, PhysicalAgentState):
        raise PhysicalAgentStateError("invalid_state", "state is invalid")

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

    if isinstance(
        event,
        (
            StepCommandAuthorized,
            StepCommandDispatched,
            StepCommandRevoked,
            StepCommandSettlementExpired,
            StepCommandSettled,
        ),
    ):
        return _reduce_step_command_event(state, event)

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


class PhysicalAgentStateReducer:
    """Thread-safe single writer around the pure transition function."""

    def __init__(self, initial: PhysicalAgentState):
        if not isinstance(initial, PhysicalAgentState):
            raise PhysicalAgentStateError(
                "invalid_initial_state", "initial physical state is invalid"
            )
        self._state = initial
        self._lock = threading.Lock()

    def snapshot(self) -> PhysicalAgentState:
        with self._lock:
            return self._state

    def apply(self, event: PhysicalAgentEvent) -> PhysicalAgentState:
        with self._lock:
            self._state = reduce_physical_agent_state(self._state, event)
            return self._state


__all__ = (
    "ActiveDispatch",
    "ActiveIntent",
    "AgentPhase",
    "ControllerCommandReceipt",
    "ControllerKey",
    "DetourSide",
    "DetourTargetIntent",
    "ExecutionPlan",
    "FollowDirectionIntent",
    "GoalActivated",
    "GoalAssignment",
    "GoalCompletionRequested",
    "GoalOutcome",
    "GoalTerminal",
    "IntentAccepted",
    "IntentPolicy",
    "IntentProgress",
    "MAX_PLANNING_TICKET_TTL_MS",
    "MAX_STEP_COMMAND_SETTLE_MS",
    "MAX_STEP_COMMAND_START_TTL_MS",
    "NavigationBasis",
    "NavigationBasisUpdated",
    "PhysicalAgentEvent",
    "PhysicalAgentState",
    "PhysicalAgentStateError",
    "PhysicalAgentStateReducer",
    "PlanBinding",
    "PlanRecompiled",
    "PlanStep",
    "PlanStepKey",
    "PlanningAbortRequested",
    "PlanningCause",
    "PlanningHeld",
    "PlanningRequested",
    "PlanningTicket",
    "PlanningTicketConsumed",
    "PlanningTicketExpired",
    "PrimitiveStep",
    "ReceiptOutcome",
    "ReplanRequested",
    "ScanTargetIntent",
    "SensorStep",
    "StopRequested",
    "StopVerified",
    "StepCommandAuthorization",
    "StepCommandAuthorized",
    "StepCommandDispatched",
    "StepCommandRevoked",
    "StepCommandSettlementExpired",
    "StepCommandSettled",
    "StepDisposition",
    "TerminalCleared",
    "WaypointStep",
    "reduce_physical_agent_state",
)
