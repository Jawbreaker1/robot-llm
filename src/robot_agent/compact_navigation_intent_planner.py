"""Thin callable bridge from canonical planning tickets to compact LM intent.

The bridge deliberately owns no retry policy, execution loop, motion, or
controller state.  It builds one host offer and, only when that offer leaves
an actual choice, makes one compact LM Studio request.
"""

from dataclasses import dataclass
import time
from typing import Callable, Optional, Tuple

from .lm_studio_navigation_intent import (
    DEFAULT_PROPOSAL_TTL_MS,
    LMStudioNavigationIntentResult,
)
from .navigation_intent_context import NavigationIntentPrompt
from .navigation_intent_proposal import (
    ABORT,
    DETOUR_TARGET,
    FOLLOW_DIRECTION,
    HOLD,
    MAX_NAVIGATION_INTENT_TTL_MS,
    SCAN_TARGET,
    NavigationIntentEnvelope,
    NavigationIntentOffer,
    NavigationIntentProposal,
    bind_navigation_intent_proposal,
)
from .physical_intent_contract import IntentPlanningRequest


_MAX_INT = 2**63 - 1


class CompactNavigationIntentPlannerError(ValueError):
    """A bridge dependency or its result violated the compact contract."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _concrete_proposals(
    offer: NavigationIntentOffer,
) -> Tuple[NavigationIntentProposal, ...]:
    proposals = []
    for intent in offer.offered_intents:
        if intent == FOLLOW_DIRECTION:
            proposals.append(NavigationIntentProposal(intent=intent))
        elif intent == SCAN_TARGET:
            proposals.extend(
                NavigationIntentProposal(intent=intent, target_id=target_id)
                for target_id in offer.scan_target_ids
            )
        elif intent == DETOUR_TARGET:
            proposals.extend(
                NavigationIntentProposal(
                    intent=intent,
                    target_id=target_id,
                    side=side,
                )
                for target_id in offer.detour_target_ids
                for side in offer.detour_sides
            )
        elif intent == HOLD:
            proposals.extend(
                NavigationIntentProposal(intent=intent, reason=reason)
                for reason in offer.hold_reasons
            )
        elif intent == ABORT:
            proposals.extend(
                NavigationIntentProposal(intent=intent, reason=reason)
                for reason in offer.abort_reasons
            )
    return tuple(proposals)


@dataclass(frozen=True)
class _Dependencies:
    offer_builder: Callable[[IntentPlanningRequest], NavigationIntentOffer]
    prompt_builder: Callable[
        [IntentPlanningRequest, NavigationIntentOffer],
        NavigationIntentPrompt,
    ]
    decide: Callable[..., LMStudioNavigationIntentResult]
    clock_ms: Callable[[], int]
    telemetry: Optional[Callable[[LMStudioNavigationIntentResult], None]]


class CompactNavigationIntentPlanner:
    """Make the compact client usable as a coordinator intent planner."""

    def __init__(
        self,
        *,
        offer_builder: Callable[
            [IntentPlanningRequest], NavigationIntentOffer
        ],
        prompt_builder: Callable[
            [IntentPlanningRequest, NavigationIntentOffer],
            NavigationIntentPrompt,
        ],
        client: object,
        clock_ms: Callable[[], int] = (
            lambda: time.time_ns() // 1_000_000
        ),
        proposal_ttl_ms: int = DEFAULT_PROPOSAL_TTL_MS,
        telemetry: Optional[
            Callable[[LMStudioNavigationIntentResult], None]
        ] = None,
    ):
        decide = getattr(client, "decide", None)
        if (
            not callable(offer_builder)
            or not callable(prompt_builder)
            or not callable(decide)
            or not callable(clock_ms)
            or (telemetry is not None and not callable(telemetry))
        ):
            raise CompactNavigationIntentPlannerError(
                "invalid_dependency",
                "Planner dependencies must be callable",
            )
        if (
            isinstance(proposal_ttl_ms, bool)
            or not isinstance(proposal_ttl_ms, int)
            or not 1 <= proposal_ttl_ms <= MAX_NAVIGATION_INTENT_TTL_MS
        ):
            raise CompactNavigationIntentPlannerError(
                "invalid_proposal_ttl",
                "Proposal TTL is outside the compact intent limit",
            )
        self._dependencies = _Dependencies(
            offer_builder=offer_builder,
            prompt_builder=prompt_builder,
            decide=decide,
            clock_ms=clock_ms,
            telemetry=telemetry,
        )
        self._proposal_ttl_ms = proposal_ttl_ms

    def _now_ms(self) -> int:
        value = self._dependencies.clock_ms()
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 0 <= value <= _MAX_INT
        ):
            raise CompactNavigationIntentPlannerError(
                "invalid_clock",
                "Host clock must return a non-negative integer timestamp",
            )
        return value

    @staticmethod
    def _validate_offer(
        request: IntentPlanningRequest,
        value: object,
    ) -> NavigationIntentOffer:
        if not isinstance(value, NavigationIntentOffer):
            raise CompactNavigationIntentPlannerError(
                "invalid_offer_result",
                "Offer builder must return NavigationIntentOffer",
            )
        if (
            request.state.basis != request.ticket.basis
            or value.ticket_id != request.ticket.ticket_id
            or value.basis != request.ticket.basis
        ):
            raise CompactNavigationIntentPlannerError(
                "offer_binding_mismatch",
                "Offer must bind the exact current ticket and basis",
            )
        return value

    def _bind_single_choice(
        self,
        *,
        request: IntentPlanningRequest,
        offer: NavigationIntentOffer,
        proposal: NavigationIntentProposal,
    ) -> NavigationIntentEnvelope:
        received_at_ms = self._now_ms()
        if received_at_ms > _MAX_INT - self._proposal_ttl_ms:
            raise CompactNavigationIntentPlannerError(
                "invalid_clock",
                "Host clock cannot represent the configured proposal TTL",
            )
        return bind_navigation_intent_proposal(
            proposal,
            offer=offer,
            proposal_id=request.proposal_id,
            received_at_ms=received_at_ms,
            valid_until_ms=received_at_ms + self._proposal_ttl_ms,
        )

    def __call__(
        self,
        request: IntentPlanningRequest,
    ) -> NavigationIntentEnvelope:
        if not isinstance(request, IntentPlanningRequest):
            raise CompactNavigationIntentPlannerError(
                "invalid_planning_request",
                "Planner requires IntentPlanningRequest",
            )

        offer = self._validate_offer(
            request,
            self._dependencies.offer_builder(request),
        )
        proposals = _concrete_proposals(offer)
        if len(proposals) == 1:
            return self._bind_single_choice(
                request=request,
                offer=offer,
                proposal=proposals[0],
            )

        prompt = self._dependencies.prompt_builder(request, offer)
        if not isinstance(prompt, NavigationIntentPrompt):
            raise CompactNavigationIntentPlannerError(
                "invalid_prompt_result",
                "Prompt builder must return NavigationIntentPrompt",
            )
        result = self._dependencies.decide(
            prompt,
            offer=offer,
            proposal_id=request.proposal_id,
        )
        if not isinstance(result, LMStudioNavigationIntentResult):
            raise CompactNavigationIntentPlannerError(
                "invalid_client_result",
                "Compact client must return LMStudioNavigationIntentResult",
            )
        envelope = result.envelope
        if (
            not isinstance(envelope, NavigationIntentEnvelope)
            or envelope.proposal_id != request.proposal_id
            or envelope.ticket_id != request.ticket.ticket_id
            or envelope.basis != request.ticket.basis
        ):
            raise CompactNavigationIntentPlannerError(
                "client_binding_mismatch",
                "Compact client result does not match the planning request",
            )
        offer.assert_allows(envelope.proposal)
        envelope.assert_current(
            proposal_id=request.proposal_id,
            ticket_id=request.ticket.ticket_id,
            basis=request.state.basis,
            now_ms=self._now_ms(),
        )
        if self._dependencies.telemetry is not None:
            try:
                self._dependencies.telemetry(result)
            except Exception:
                pass
        return envelope


__all__ = (
    "CompactNavigationIntentPlanner",
    "CompactNavigationIntentPlannerError",
)
