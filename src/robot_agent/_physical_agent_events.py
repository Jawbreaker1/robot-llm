"""Private fact events accepted by the canonical physical-agent reducer."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Union

from ._physical_agent_core import (
    ActiveIntent,
    ExecutionPlan,
    GoalAssignment,
    GoalTerminal,
    NavigationBasis,
    PlanningTicket,
)
from ._physical_agent_dispatch_contract import (
    ControllerCommandReceipt,
    StepCommandAuthorization,
    StepDisposition,
)
from ._physical_agent_prepared_contract import PreparedIntentPlan


@dataclass(frozen=True)
class GoalActivated:
    goal: GoalAssignment
    basis: NavigationBasis
    ticket: PlanningTicket


@dataclass(frozen=True)
class PlanningTicketConsumed:
    ticket: PlanningTicket
    consumed_at_ms: int


@dataclass(frozen=True)
class PlanningTicketExpired:
    ticket: PlanningTicket
    observed_at_ms: int


@dataclass(frozen=True)
class IntentAccepted:
    """Legacy value retained for deserialization, not a live state event."""

    ticket_id: str
    based_on_basis: NavigationBasis
    intent: ActiveIntent
    plan: ExecutionPlan


@dataclass(frozen=True)
class IntentPrepared:
    prepared: PreparedIntentPlan


@dataclass(frozen=True)
class PreparedIntentAccepted:
    prepared: PreparedIntentPlan
    accepted_at_ms: int


@dataclass(frozen=True)
class PreparedIntentExpired:
    prepared: PreparedIntentPlan
    observed_at_ms: int


@dataclass(frozen=True)
class PlanningAbortRequested:
    ticket: PlanningTicket
    proposal_id: Optional[str]
    terminal: GoalTerminal


@dataclass(frozen=True)
class PlanningHeld:
    ticket: PlanningTicket
    proposal_id: Optional[str]


@dataclass(frozen=True)
class PlanningRequested:
    ticket: PlanningTicket


@dataclass(frozen=True)
class NavigationBasisUpdated:
    basis: NavigationBasis


@dataclass(frozen=True)
class StepCommandAuthorized:
    authorization: StepCommandAuthorization


@dataclass(frozen=True)
class StepCommandDispatched:
    """Write-ahead fact; MUST be journaled before the first outbound byte."""

    authorization: StepCommandAuthorization
    dispatched_at_ms: int
    settle_by_host_ms: int


@dataclass(frozen=True)
class StepCommandRevoked:
    authorization: StepCommandAuthorization
    revoked_at_ms: int


@dataclass(frozen=True)
class StepCommandSettlementExpired:
    authorization: StepCommandAuthorization
    observed_at_host_ms: int


@dataclass(frozen=True)
class StepCommandSettled:
    receipt: ControllerCommandReceipt
    resulting_basis: NavigationBasis
    disposition: StepDisposition
    replan_ticket: Optional[PlanningTicket] = None


@dataclass(frozen=True)
class PlanRecompiled:
    plan: ExecutionPlan
    resulting_basis: NavigationBasis


@dataclass(frozen=True)
class ReplanRequested:
    ticket: PlanningTicket
    resulting_basis: NavigationBasis
    reason: str


@dataclass(frozen=True)
class GoalCompletionRequested:
    resulting_basis: NavigationBasis
    terminal: GoalTerminal


@dataclass(frozen=True)
class StopRequested:
    terminal: GoalTerminal


@dataclass(frozen=True)
class StopVerified:
    verified_at_ms: int
    dispatch_receipt: Optional[ControllerCommandReceipt] = None
    resulting_basis: Optional[NavigationBasis] = None


@dataclass(frozen=True)
class TerminalCleared:
    cleared_at_ms: int


PhysicalAgentEvent = Union[
    GoalActivated,
    PlanningTicketConsumed,
    PlanningTicketExpired,
    IntentPrepared,
    PreparedIntentAccepted,
    PreparedIntentExpired,
    PlanningAbortRequested,
    PlanningHeld,
    PlanningRequested,
    NavigationBasisUpdated,
    StepCommandAuthorized,
    StepCommandDispatched,
    StepCommandRevoked,
    StepCommandSettlementExpired,
    StepCommandSettled,
    PlanRecompiled,
    ReplanRequested,
    GoalCompletionRequested,
    StopRequested,
    StopVerified,
    TerminalCleared,
]
