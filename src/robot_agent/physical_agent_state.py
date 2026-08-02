"""Pure reducer and stable public import surface for physical agent state."""

import threading

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
)
from ._physical_agent_reducer import reduce_physical_agent_state


# Keep the historical public function path for introspection and serialization.
reduce_physical_agent_state.__module__ = __name__


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
