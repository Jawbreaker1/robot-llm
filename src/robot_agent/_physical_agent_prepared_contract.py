"""Private durable contract for one compiled intent awaiting activation."""

from dataclasses import dataclass

from ._physical_agent_core import (
    ActiveIntent,
    ExecutionPlan,
    NavigationBasis,
    PhysicalAgentStateError,
    PlanningTicket,
    _identifier,
    _integer,
)


MAX_PREPARED_INTENT_TTL_MS = 60_000


@dataclass(frozen=True)
class PreparedIntentPlan:
    """A validated, compiled result durably bound to one planning ticket."""

    ticket: PlanningTicket
    proposal_id: str
    compilation_basis: NavigationBasis
    intent: ActiveIntent
    plan: ExecutionPlan
    prepared_at_ms: int
    valid_until_ms: int

    def __post_init__(self) -> None:
        if not isinstance(self.ticket, PlanningTicket) or not self.ticket.consumed:
            raise PhysicalAgentStateError(
                "invalid_prepared_ticket",
                "prepared intent requires the exact consumed ticket",
            )
        _identifier("prepared_proposal_id", self.proposal_id)
        if not isinstance(self.compilation_basis, NavigationBasis):
            raise PhysicalAgentStateError(
                "invalid_prepared_compilation_basis",
                "prepared intent compilation basis is invalid",
            )
        if not isinstance(self.intent, ActiveIntent):
            raise PhysicalAgentStateError(
                "invalid_prepared_intent",
                "prepared intent is invalid",
            )
        if not isinstance(self.plan, ExecutionPlan):
            raise PhysicalAgentStateError(
                "invalid_prepared_plan",
                "prepared execution plan is invalid",
            )
        _integer("intent_prepared_at_ms", self.prepared_at_ms)
        _integer("prepared_intent_valid_until_ms", self.valid_until_ms)
        if (
            self.valid_until_ms <= self.prepared_at_ms
            or self.valid_until_ms - self.prepared_at_ms
            > MAX_PREPARED_INTENT_TTL_MS
            or self.prepared_at_ms < self.ticket.consumed_at_ms
            or self.valid_until_ms > self.ticket.valid_until_ms
        ):
            raise PhysicalAgentStateError(
                "invalid_prepared_intent_ttl",
                "prepared intent expiry must be after preparation and bounded",
            )
        binding = self.plan.binding
        ticket_basis = self.ticket.basis
        compilation_basis = self.compilation_basis
        if not ticket_basis.decision_equivalent(compilation_basis):
            raise PhysicalAgentStateError(
                "prepared_intent_basis_mismatch",
                "ticket and compilation basis are not decision-equivalent",
            )
        compilation_basis.assert_successor_of(ticket_basis)
        if (
            self.intent.accepted_basis != ticket_basis
            or binding.controller_key != compilation_basis.controller_key
            or binding.goal_id != self.intent.goal_id
            or binding.goal_epoch != self.intent.goal_epoch
            or binding.intent_id != self.intent.intent_id
            or binding.intent_revision != self.intent.revision
            or binding.frame_id != compilation_basis.frame_id
            or binding.world_generation_id
            != compilation_basis.world_generation_id
            or binding.calibration_fingerprint
            != compilation_basis.calibration_fingerprint
            or binding.based_on_navigation_basis_id
            != compilation_basis.navigation_basis_id
        ):
            raise PhysicalAgentStateError(
                "prepared_intent_binding_mismatch",
                "prepared intent and plan do not share one exact basis binding",
            )

    @property
    def ticket_id(self) -> str:
        return self.ticket.ticket_id

    @property
    def ticket_basis(self) -> NavigationBasis:
        return self.ticket.basis


__all__ = ("PreparedIntentPlan",)
