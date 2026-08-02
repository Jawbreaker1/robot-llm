"""Pure reducer and stable public import surface for physical agent state."""

from dataclasses import replace
import threading
from typing import Optional, Tuple

from .physical_agent_contract import (
    ActiveIntent,
    AgentPhase,
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
    NavigationBasis,
    NavigationBasisUpdated,
    PhysicalAgentEvent,
    PhysicalAgentState,
    PhysicalAgentStateError,
    PlanBinding,
    PlanFinished,
    PlanRecompiled,
    PlanStep,
    PlanStepAdvanced,
    PlanningAbortRequested,
    PlanningCause,
    PlanningHeld,
    PlanningRequested,
    PlanningTicket,
    PlanningTicketConsumed,
    PlanningTicketExpired,
    PrimitiveStep,
    ReplanRequested,
    ScanTargetIntent,
    SensorStep,
    StopRequested,
    StopVerified,
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

    if isinstance(event, PlanStepAdvanced):
        _require_phase(state, event, AgentPhase.EXECUTING)
        _successor(state, event.resulting_basis)
        active_plan = state.plan
        if (
            event.plan_id != active_plan.plan_id
            or event.plan_revision != active_plan.revision
            or event.expected_cursor != active_plan.cursor
        ):
            raise PhysicalAgentStateError(
                "plan_cursor_mismatch",
                "step receipt does not match the active plan cursor",
            )
        next_cursor = active_plan.cursor + 1
        if next_cursor >= len(active_plan.steps):
            raise PhysicalAgentStateError(
                "final_step_requires_transition",
                "the final step must transition to planning or stopping",
            )
        active_ticket = state.planning_ticket
        if active_ticket is not None and not active_ticket.basis.decision_equivalent(
            event.resulting_basis
        ):
            active_ticket = None
        return _next(
            state,
            basis=event.resulting_basis,
            plan=replace(active_plan, cursor=next_cursor),
            planning_ticket=active_ticket,
            intent_progress=_verified_step_progress(
                state,
                event.resulting_basis,
            ),
        )

    if isinstance(event, PlanFinished):
        _require_phase(state, event, AgentPhase.EXECUTING)
        _successor(state, event.resulting_basis)
        active_plan = state.plan
        if (
            event.plan_id != active_plan.plan_id
            or event.plan_revision != active_plan.revision
            or event.expected_cursor != active_plan.cursor
        ):
            raise PhysicalAgentStateError(
                "plan_cursor_mismatch",
                "final step receipt does not match the active plan cursor",
            )
        if active_plan.cursor != len(active_plan.steps) - 1:
            raise PhysicalAgentStateError(
                "plan_not_at_final_step",
                "plan may finish only while its final step is active",
            )
        return _next(
            state,
            phase=AgentPhase.PLANNING,
            basis=event.resulting_basis,
            compile_pending=True,
            plan=None,
            planning_ticket=None,
            intent_progress=_verified_step_progress(
                state,
                event.resulting_basis,
            ),
        )

    if isinstance(event, PlanRecompiled):
        _require_phase(
            state,
            event,
            AgentPhase.PLANNING,
            AgentPhase.EXECUTING,
        )
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
            planning_ticket=None,
        )

    if isinstance(event, ReplanRequested):
        _require_phase(state, event, AgentPhase.EXECUTING)
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
        return _next(
            state,
            phase=AgentPhase.TERMINAL,
            compile_pending=False,
            intent=None,
            intent_progress=None,
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
    "ActiveIntent",
    "AgentPhase",
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
    "NavigationBasis",
    "NavigationBasisUpdated",
    "PhysicalAgentEvent",
    "PhysicalAgentState",
    "PhysicalAgentStateError",
    "PhysicalAgentStateReducer",
    "PlanBinding",
    "PlanFinished",
    "PlanRecompiled",
    "PlanStep",
    "PlanStepAdvanced",
    "PlanningAbortRequested",
    "PlanningCause",
    "PlanningHeld",
    "PlanningRequested",
    "PlanningTicket",
    "PlanningTicketConsumed",
    "PlanningTicketExpired",
    "PrimitiveStep",
    "ReplanRequested",
    "ScanTargetIntent",
    "SensorStep",
    "StopRequested",
    "StopVerified",
    "TerminalCleared",
    "WaypointStep",
    "reduce_physical_agent_state",
)
