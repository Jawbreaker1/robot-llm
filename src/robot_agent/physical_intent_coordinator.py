"""One-shot coordination between navigation intent planning and agent state.

The coordinator owns no loop and performs no motion.  One ``run_once`` call
may consume one planning ticket, invoke the injected planner exactly once,
validate its host-bound envelope, compile one deterministic execution plan,
and durably prepare then activate it through canonical reducer events.  A
failed external call is held in an explicit, ticket-free PLANNING state; a
later retry requires a new host ``PlanningRequested`` event.
"""

from dataclasses import dataclass
from typing import Callable, Optional, Union

from .navigation_intent_proposal import (
    ABORT,
    DETOUR_TARGET,
    FOLLOW_DIRECTION,
    HOLD,
    LEFT,
    NavigationIntentEnvelope,
    NavigationIntentProposal,
    NavigationIntentProposalError,
    RIGHT,
    SCAN_TARGET,
)
from .physical_agent_state import (
    ActiveIntent,
    AgentPhase,
    DetourSide,
    DetourTargetIntent,
    ExecutionPlan,
    FollowDirectionIntent,
    GoalOutcome,
    GoalTerminal,
    IntentPrepared,
    IntentPolicy,
    PhysicalAgentState,
    PhysicalAgentStateError,
    PhysicalAgentStateReducer,
    PlanningAbortRequested,
    PlanningHeld,
    PlanningTicket,
    PlanningTicketConsumed,
    PlanningTicketExpired,
    PreparedIntentAccepted,
    PreparedIntentExpired,
    PreparedIntentPlan,
    ScanTargetIntent,
)
from .physical_intent_contract import (
    CoordinatorOutcome,
    IntentCompilationEvidence,
    IntentCompilationRequest,
    IntentPlanningRequest,
    MAX_COMPILATION_EVIDENCE_BYTES,
    MAX_TARGET_GEOMETRY_SIGNATURES,
    PhysicalIntentCoordinatorError,
    PhysicalIntentCoordinatorResult,
    _identifier,
    _integer,
)


def _exception_code(error: Exception, fallback: str) -> str:
    value = getattr(error, "code", None)
    if (
        isinstance(value, str)
        and value
        and value == value.strip()
        and len(value) <= 160
        and not any(ord(character) < 32 for character in value)
    ):
        return value
    return fallback


@dataclass(frozen=True)
class _TicketAcquisition:
    ticket: PlanningTicket
    consumed_state: PhysicalAgentState


@dataclass(frozen=True)
class _ValidatedPlannerResult:
    ticket: PlanningTicket
    proposal_id: str
    envelope: NavigationIntentEnvelope
    current_state: PhysicalAgentState
    evaluated_at_ms: int


@dataclass(frozen=True)
class _CapturedCompilationEvidence:
    state: PhysicalAgentState
    evidence: IntentCompilationEvidence
    captured_at_ms: int


@dataclass(frozen=True)
class _CompiledIntentPlan:
    intent: ActiveIntent
    plan: ExecutionPlan


class PhysicalIntentCoordinator:
    """Advance at most one unconsumed canonical planning ticket."""

    def __init__(
        self,
        *,
        reducer: PhysicalAgentStateReducer,
        intent_planner: Callable[
            [IntentPlanningRequest], NavigationIntentEnvelope
        ],
        compilation_evidence_provider: Callable[
            [PhysicalAgentState, NavigationIntentProposal],
            IntentCompilationEvidence,
        ],
        plan_compiler: Callable[
            [IntentCompilationRequest], ExecutionPlan
        ],
        clock_ms: Callable[[], int],
        id_factory: Callable[[str], str],
        default_scan_profile_id: str,
        abort_outcome: GoalOutcome = GoalOutcome.FAILED,
    ):
        if not isinstance(reducer, PhysicalAgentStateReducer):
            raise PhysicalIntentCoordinatorError(
                "invalid_reducer",
                "reducer must be PhysicalAgentStateReducer",
            )
        for name, value in (
            ("intent_planner", intent_planner),
            (
                "compilation_evidence_provider",
                compilation_evidence_provider,
            ),
            ("plan_compiler", plan_compiler),
            ("clock_ms", clock_ms),
            ("id_factory", id_factory),
        ):
            if not callable(value):
                raise PhysicalIntentCoordinatorError(
                    "invalid_dependency",
                    "{} must be callable".format(name),
                )
        _identifier("default_scan_profile_id", default_scan_profile_id)
        if abort_outcome not in (GoalOutcome.CANCELLED, GoalOutcome.FAILED):
            raise PhysicalIntentCoordinatorError(
                "invalid_abort_outcome",
                "ABORT must map to host-owned CANCELLED or FAILED",
            )
        self._reducer = reducer
        self._intent_planner = intent_planner
        self._compilation_evidence_provider = (
            compilation_evidence_provider
        )
        self._plan_compiler = plan_compiler
        self._clock_ms = clock_ms
        self._id_factory = id_factory
        self._default_scan_profile_id = default_scan_profile_id
        self._abort_outcome = abort_outcome

    def _now_ms(self) -> int:
        return _integer("clock_ms", self._clock_ms())

    def _new_id(self, namespace: str) -> str:
        return _identifier(
            "{}_id".format(namespace),
            self._id_factory(namespace),
        )

    @staticmethod
    def _current_consumed_ticket(
        state: PhysicalAgentState,
        ticket: PlanningTicket,
    ) -> bool:
        current = state.planning_ticket
        return (
            state.phase in (AgentPhase.PLANNING, AgentPhase.EXECUTING)
            and current is not None
            and current == ticket
            and current.consumed
        )

    def _result(
        self,
        outcome: CoordinatorOutcome,
        *,
        ticket: Optional[PlanningTicket] = None,
        proposal_id: Optional[str] = None,
        error_code: Optional[str] = None,
    ) -> PhysicalIntentCoordinatorResult:
        return PhysicalIntentCoordinatorResult(
            outcome=outcome,
            state=self._reducer.snapshot(),
            ticket_id=None if ticket is None else ticket.ticket_id,
            proposal_id=proposal_id,
            error_code=error_code,
        )

    def _hold_if_current(
        self,
        *,
        outcome: CoordinatorOutcome,
        ticket: PlanningTicket,
        proposal_id: Optional[str],
        error_code: str,
    ) -> PhysicalIntentCoordinatorResult:
        current = self._reducer.snapshot()
        if not self._current_consumed_ticket(current, ticket):
            return self._result(
                CoordinatorOutcome.SUPERSEDED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=error_code,
            )
        try:
            self._reducer.apply(
                PlanningHeld(
                    ticket=ticket,
                    proposal_id=proposal_id,
                )
            )
        except PhysicalAgentStateError:
            return self._result(
                CoordinatorOutcome.SUPERSEDED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=error_code,
            )
        return self._result(
            outcome,
            ticket=ticket,
            proposal_id=proposal_id,
            error_code=error_code,
        )

    def _expire_ticket(
        self,
        *,
        ticket: PlanningTicket,
        observed_at_ms: int,
        proposal_id: Optional[str] = None,
    ) -> PhysicalIntentCoordinatorResult:
        try:
            self._reducer.apply(
                PlanningTicketExpired(
                    ticket=ticket,
                    observed_at_ms=observed_at_ms,
                )
            )
        except PhysicalAgentStateError:
            return self._result(
                CoordinatorOutcome.SUPERSEDED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code="planning_ticket_expired",
            )
        return self._result(
            CoordinatorOutcome.TICKET_EXPIRED,
            ticket=ticket,
            proposal_id=proposal_id,
            error_code="planning_ticket_expired",
        )

    def _expire_prepared_intent(
        self,
        *,
        prepared: PreparedIntentPlan,
        observed_at_ms: int,
    ) -> PhysicalIntentCoordinatorResult:
        try:
            state = self._reducer.apply(
                PreparedIntentExpired(
                    prepared=prepared,
                    observed_at_ms=observed_at_ms,
                )
            )
        except PhysicalAgentStateError as exc:
            return self._result(
                CoordinatorOutcome.SUPERSEDED,
                ticket=self._reducer.snapshot().planning_ticket,
                proposal_id=prepared.proposal_id,
                error_code=exc.code,
            )
        return PhysicalIntentCoordinatorResult(
            outcome=CoordinatorOutcome.PROPOSAL_REJECTED,
            state=state,
            ticket_id=prepared.ticket_id,
            proposal_id=prepared.proposal_id,
            error_code="expired_proposal",
        )

    def _accept_prepared_intent(
        self,
        *,
        state: PhysicalAgentState,
        prepared: PreparedIntentPlan,
        accepted_at_ms: int,
    ) -> PhysicalIntentCoordinatorResult:
        ticket = state.planning_ticket
        if state.active_dispatch is not None:
            return self._result(
                CoordinatorOutcome.DEFERRED,
                ticket=ticket,
                proposal_id=prepared.proposal_id,
            )
        try:
            accepted = self._reducer.apply(
                PreparedIntentAccepted(
                    prepared=prepared,
                    accepted_at_ms=accepted_at_ms,
                )
            )
        except PhysicalAgentStateError as exc:
            return self._result(
                CoordinatorOutcome.SUPERSEDED,
                ticket=ticket,
                proposal_id=prepared.proposal_id,
                error_code=exc.code,
            )
        return PhysicalIntentCoordinatorResult(
            outcome=CoordinatorOutcome.INTENT_ACCEPTED,
            state=accepted,
            ticket_id=prepared.ticket_id,
            proposal_id=prepared.proposal_id,
        )

    def _resume_prepared_intent(
        self,
    ) -> Optional[PhysicalIntentCoordinatorResult]:
        current = self._reducer.snapshot()
        prepared = current.prepared_intent_plan
        if prepared is None:
            return None
        ticket = current.planning_ticket
        try:
            now_ms = self._now_ms()
        except Exception as exc:
            return self._result(
                CoordinatorOutcome.HOST_FAILED,
                ticket=ticket,
                proposal_id=prepared.proposal_id,
                error_code=_exception_code(exc, "clock_failed"),
            )
        if now_ms >= ticket.valid_until_ms:
            return self._expire_ticket(
                ticket=ticket,
                observed_at_ms=now_ms,
                proposal_id=prepared.proposal_id,
            )
        if now_ms >= prepared.valid_until_ms:
            return self._expire_prepared_intent(
                prepared=prepared,
                observed_at_ms=now_ms,
            )
        return self._accept_prepared_intent(
            state=current,
            prepared=prepared,
            accepted_at_ms=now_ms,
        )

    @staticmethod
    def _require_live_ticket(ticket: PlanningTicket, now_ms: int) -> None:
        if now_ms >= ticket.valid_until_ms:
            raise PhysicalIntentCoordinatorError(
                "planning_ticket_expired",
                "planning ticket expired before its result could commit",
            )

    def _validate_envelope(
        self,
        envelope: object,
        *,
        proposal_id: str,
        ticket: PlanningTicket,
        state: PhysicalAgentState,
        now_ms: int,
    ) -> NavigationIntentEnvelope:
        self._require_live_ticket(ticket, now_ms)
        if not isinstance(envelope, NavigationIntentEnvelope):
            raise NavigationIntentProposalError(
                "invalid_planner_result",
                "Intent planner must return NavigationIntentEnvelope",
            )
        if envelope.basis != ticket.basis:
            raise NavigationIntentProposalError(
                "planning_ticket_basis_mismatch",
                "Envelope basis does not exactly match its planning ticket",
            )
        envelope.assert_current(
            proposal_id=proposal_id,
            ticket_id=ticket.ticket_id,
            basis=state.basis,
            now_ms=now_ms,
        )
        return envelope

    def _payload(self, proposal: NavigationIntentProposal):
        if proposal.intent == FOLLOW_DIRECTION:
            return FollowDirectionIntent()
        if proposal.intent == SCAN_TARGET:
            return ScanTargetIntent(
                target_hypothesis_id=proposal.target_id,
                scan_profile_id=self._default_scan_profile_id,
            )
        if proposal.intent == DETOUR_TARGET:
            sides = {
                LEFT: DetourSide.LEFT_OF_GOAL,
                RIGHT: DetourSide.RIGHT_OF_GOAL,
            }
            return DetourTargetIntent(
                target_hypothesis_id=proposal.target_id,
                detour_side=sides[proposal.side],
            )
        raise PhysicalIntentCoordinatorError(
            "invalid_executable_intent",
            "Only executable proposals have intent payloads",
        )

    def _active_intent(
        self,
        *,
        state: PhysicalAgentState,
        ticket: PlanningTicket,
        proposal: NavigationIntentProposal,
        accepted_at_ms: int,
    ) -> ActiveIntent:
        payload = self._payload(proposal)
        current = state.intent
        if current is not None and current.payload == payload:
            intent_id = current.intent_id
            revision = current.revision + 1
            policy = current.policy
        else:
            intent_id = self._new_id("intent")
            revision = 1
            policy = IntentPolicy()
        return ActiveIntent(
            intent_id=intent_id,
            revision=revision,
            goal_id=state.goal.goal_id,
            goal_epoch=state.goal_epoch,
            payload=payload,
            accepted_basis=ticket.basis,
            accepted_at_ms=accepted_at_ms,
            policy=policy,
        )

    @staticmethod
    def _validate_plan_evidence(
        plan: ExecutionPlan,
        evidence: IntentCompilationEvidence,
    ) -> None:
        binding = plan.binding
        basis = evidence.basis
        if (
            binding.controller_key != basis.controller_key
            or binding.frame_id != basis.frame_id
            or binding.world_generation_id != basis.world_generation_id
            or binding.calibration_fingerprint
            != basis.calibration_fingerprint
            or binding.based_on_navigation_basis_id
            != basis.navigation_basis_id
        ):
            raise PhysicalIntentCoordinatorError(
                "compiled_plan_basis_mismatch",
                "compiled plan is not bound to its captured evidence basis",
            )
        if (
            binding.target_geometry_signatures
            != evidence.target_geometry_signatures
        ):
            raise PhysicalIntentCoordinatorError(
                "compiled_plan_geometry_mismatch",
                "compiled plan geometry signatures differ from its snapshot",
            )

    def _acquire_planning_ticket(
        self,
    ) -> Union[_TicketAcquisition, PhysicalIntentCoordinatorResult]:
        initial = self._reducer.snapshot()
        ticket = initial.planning_ticket
        if (
            initial.phase not in (AgentPhase.PLANNING, AgentPhase.EXECUTING)
            or ticket is None
        ):
            return self._result(
                CoordinatorOutcome.NO_WORK,
                ticket=ticket,
            )

        try:
            consumed_at_ms = self._now_ms()
            if consumed_at_ms >= ticket.valid_until_ms:
                return self._expire_ticket(
                    ticket=ticket,
                    observed_at_ms=consumed_at_ms,
                )
            if ticket.consumed:
                return self._result(
                    CoordinatorOutcome.NO_WORK,
                    ticket=ticket,
                )
            consumed = self._reducer.apply(
                PlanningTicketConsumed(
                    ticket=ticket,
                    consumed_at_ms=consumed_at_ms,
                )
            )
        except (PhysicalAgentStateError, PhysicalIntentCoordinatorError) as exc:
            return self._result(
                CoordinatorOutcome.SUPERSEDED,
                ticket=ticket,
                error_code=_exception_code(exc, "ticket_consume_failed"),
            )
        return _TicketAcquisition(
            ticket=consumed.planning_ticket,
            consumed_state=consumed,
        )

    def _call_and_validate_planner(
        self,
        acquisition: _TicketAcquisition,
    ) -> Union[_ValidatedPlannerResult, PhysicalIntentCoordinatorResult]:
        ticket = acquisition.ticket
        try:
            proposal_id = self._new_id("proposal")
        except Exception:
            return self._hold_if_current(
                outcome=CoordinatorOutcome.HOST_FAILED,
                ticket=ticket,
                proposal_id=None,
                error_code="proposal_id_failed",
            )

        planner_request = IntentPlanningRequest(
            proposal_id=proposal_id,
            state=acquisition.consumed_state,
            ticket=acquisition.consumed_state.planning_ticket,
        )
        try:
            envelope = self._intent_planner(planner_request)
        except Exception:
            return self._hold_if_current(
                outcome=CoordinatorOutcome.PLANNER_FAILED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code="intent_planner_failed",
            )

        try:
            evaluated_at_ms = self._now_ms()
            current = self._reducer.snapshot()
            if not self._current_consumed_ticket(current, ticket):
                return self._result(
                    CoordinatorOutcome.SUPERSEDED,
                    ticket=ticket,
                    proposal_id=proposal_id,
                    error_code="planning_ticket_superseded",
                )
            envelope = self._validate_envelope(
                envelope,
                proposal_id=proposal_id,
                ticket=ticket,
                state=current,
                now_ms=evaluated_at_ms,
            )
        except (NavigationIntentProposalError, PhysicalIntentCoordinatorError) as exc:
            if exc.code == "planning_ticket_expired":
                return self._expire_ticket(
                    ticket=ticket,
                    observed_at_ms=evaluated_at_ms,
                    proposal_id=proposal_id,
                )
            return self._hold_if_current(
                outcome=CoordinatorOutcome.PROPOSAL_REJECTED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=exc.code,
            )
        return _ValidatedPlannerResult(
            ticket=ticket,
            proposal_id=proposal_id,
            envelope=envelope,
            current_state=current,
            evaluated_at_ms=evaluated_at_ms,
        )

    def _apply_proposal_disposition(
        self,
        planner_result: _ValidatedPlannerResult,
    ) -> Optional[PhysicalIntentCoordinatorResult]:
        ticket = planner_result.ticket
        proposal_id = planner_result.proposal_id
        proposal = planner_result.envelope.proposal
        if proposal.intent == HOLD:
            try:
                state = self._reducer.apply(
                    PlanningHeld(
                        ticket=ticket,
                        proposal_id=proposal_id,
                    )
                )
            except PhysicalAgentStateError as exc:
                return self._result(
                    CoordinatorOutcome.SUPERSEDED,
                    ticket=ticket,
                    proposal_id=proposal_id,
                    error_code=exc.code,
                )
            return PhysicalIntentCoordinatorResult(
                outcome=CoordinatorOutcome.HELD,
                state=state,
                ticket_id=ticket.ticket_id,
                proposal_id=proposal_id,
            )

        if proposal.intent == ABORT:
            try:
                state = self._reducer.apply(
                    PlanningAbortRequested(
                        ticket=ticket,
                        proposal_id=proposal_id,
                        terminal=GoalTerminal(
                            outcome=self._abort_outcome,
                            reason=proposal.reason,
                            completed_at_ms=planner_result.evaluated_at_ms,
                        ),
                    )
                )
            except PhysicalAgentStateError as exc:
                return self._result(
                    CoordinatorOutcome.SUPERSEDED,
                    ticket=ticket,
                    proposal_id=proposal_id,
                    error_code=exc.code,
                )
            return PhysicalIntentCoordinatorResult(
                outcome=CoordinatorOutcome.ABORTED,
                state=state,
                ticket_id=ticket.ticket_id,
                proposal_id=proposal_id,
            )
        return None

    def _capture_compilation_evidence(
        self,
        planner_result: _ValidatedPlannerResult,
    ) -> Union[
        _CapturedCompilationEvidence,
        PhysicalIntentCoordinatorResult,
    ]:
        ticket = planner_result.ticket
        proposal_id = planner_result.proposal_id
        envelope = planner_result.envelope
        current = planner_result.current_state
        proposal = envelope.proposal
        captured_at_ms = None
        try:
            evidence = self._compilation_evidence_provider(
                current,
                proposal,
            )
            if not isinstance(evidence, IntentCompilationEvidence):
                raise PhysicalIntentCoordinatorError(
                    "invalid_compilation_evidence",
                    "provider must return IntentCompilationEvidence",
                )
            evidence.assert_covers(proposal)
            captured = self._reducer.snapshot()
            if not self._current_consumed_ticket(captured, ticket):
                return self._result(
                    CoordinatorOutcome.SUPERSEDED,
                    ticket=ticket,
                    proposal_id=proposal_id,
                    error_code="planning_ticket_superseded",
                )
            if evidence.basis != current.basis or evidence.basis != captured.basis:
                raise PhysicalIntentCoordinatorError(
                    "compilation_evidence_basis_mismatch",
                    "evidence capture does not exactly match current basis",
                )
            captured_at_ms = self._now_ms()
            self._require_live_ticket(ticket, captured_at_ms)
            envelope.assert_current(
                proposal_id=proposal_id,
                ticket_id=ticket.ticket_id,
                basis=captured.basis,
                now_ms=captured_at_ms,
            )
        except NavigationIntentProposalError as exc:
            return self._hold_if_current(
                outcome=CoordinatorOutcome.PROPOSAL_REJECTED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=exc.code,
            )
        except PhysicalIntentCoordinatorError as exc:
            if (
                exc.code == "planning_ticket_expired"
                and captured_at_ms is not None
            ):
                return self._expire_ticket(
                    ticket=ticket,
                    observed_at_ms=captured_at_ms,
                    proposal_id=proposal_id,
                )
            return self._hold_if_current(
                outcome=CoordinatorOutcome.EVIDENCE_FAILED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=exc.code,
            )
        except Exception as exc:
            return self._hold_if_current(
                outcome=CoordinatorOutcome.EVIDENCE_FAILED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=_exception_code(
                    exc,
                    "compilation_evidence_failed",
                ),
            )
        return _CapturedCompilationEvidence(
            state=captured,
            evidence=evidence,
            captured_at_ms=captured_at_ms,
        )

    def _compile_execution_plan(
        self,
        planner_result: _ValidatedPlannerResult,
        capture: _CapturedCompilationEvidence,
    ) -> Union[_CompiledIntentPlan, PhysicalIntentCoordinatorResult]:
        ticket = planner_result.ticket
        proposal_id = planner_result.proposal_id
        proposal = planner_result.envelope.proposal
        try:
            active_intent = self._active_intent(
                state=capture.state,
                ticket=ticket,
                proposal=proposal,
                accepted_at_ms=capture.captured_at_ms,
            )
            plan_id = self._new_id("plan")
            compilation_request = IntentCompilationRequest(
                state=capture.state,
                intent=active_intent,
                proposal=proposal,
                evidence=capture.evidence,
                plan_id=plan_id,
                plan_revision=capture.state.plan_revision + 1,
                created_at_ms=capture.captured_at_ms,
            )
            plan = self._plan_compiler(compilation_request)
            if not isinstance(plan, ExecutionPlan):
                raise PhysicalIntentCoordinatorError(
                    "invalid_compiled_plan",
                    "Plan compiler must return ExecutionPlan",
                )
        except Exception as exc:
            return self._hold_if_current(
                outcome=CoordinatorOutcome.COMPILER_FAILED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=_exception_code(exc, "plan_compiler_failed"),
            )
        return _CompiledIntentPlan(intent=active_intent, plan=plan)

    def _commit_intent(
        self,
        planner_result: _ValidatedPlannerResult,
        capture: _CapturedCompilationEvidence,
        compiled: _CompiledIntentPlan,
    ) -> PhysicalIntentCoordinatorResult:
        ticket = planner_result.ticket
        proposal_id = planner_result.proposal_id
        try:
            commit_at_ms = self._now_ms()
            latest = self._reducer.snapshot()
            if not self._current_consumed_ticket(latest, ticket):
                return self._result(
                    CoordinatorOutcome.SUPERSEDED,
                    ticket=ticket,
                    proposal_id=proposal_id,
                    error_code="planning_ticket_superseded",
                )
            self._require_live_ticket(ticket, commit_at_ms)
            planner_result.envelope.assert_current(
                proposal_id=proposal_id,
                ticket_id=ticket.ticket_id,
                basis=latest.basis,
                now_ms=commit_at_ms,
            )
            if not capture.evidence.basis.decision_equivalent(latest.basis):
                raise PhysicalIntentCoordinatorError(
                    "stale_compilation_evidence",
                    "compilation evidence is stale before commit",
                )
            self._validate_plan_evidence(compiled.plan, capture.evidence)
            prepared = PreparedIntentPlan(
                ticket=ticket,
                proposal_id=proposal_id,
                compilation_basis=capture.evidence.basis,
                intent=compiled.intent,
                plan=compiled.plan,
                prepared_at_ms=commit_at_ms,
                valid_until_ms=min(
                    ticket.valid_until_ms,
                    planner_result.envelope.valid_until_ms,
                ),
            )
            prepared_state = self._reducer.apply(IntentPrepared(prepared))
        except NavigationIntentProposalError as exc:
            return self._hold_if_current(
                outcome=CoordinatorOutcome.PROPOSAL_REJECTED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=exc.code,
            )
        except PhysicalIntentCoordinatorError as exc:
            if exc.code == "planning_ticket_expired":
                return self._expire_ticket(
                    ticket=ticket,
                    observed_at_ms=commit_at_ms,
                    proposal_id=proposal_id,
                )
            return self._hold_if_current(
                outcome=CoordinatorOutcome.COMPILER_FAILED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=exc.code,
            )
        except PhysicalAgentStateError as exc:
            current = self._reducer.snapshot()
            if current.prepared_intent_plan is not None:
                return self._result(
                    CoordinatorOutcome.SUPERSEDED,
                    ticket=ticket,
                    proposal_id=proposal_id,
                    error_code=exc.code,
                )
            return self._hold_if_current(
                outcome=CoordinatorOutcome.COMPILER_FAILED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=exc.code,
            )
        except Exception as exc:
            return self._hold_if_current(
                outcome=CoordinatorOutcome.COMPILER_FAILED,
                ticket=ticket,
                proposal_id=proposal_id,
                error_code=_exception_code(exc, "plan_compiler_failed"),
            )
        return self._accept_prepared_intent(
            state=prepared_state,
            prepared=prepared,
            accepted_at_ms=commit_at_ms,
        )

    def run_once(self) -> PhysicalIntentCoordinatorResult:
        """Process one eligible ticket; never retries or starts a loop."""

        prepared = self._resume_prepared_intent()
        if prepared is not None:
            return prepared

        acquisition = self._acquire_planning_ticket()
        if isinstance(acquisition, PhysicalIntentCoordinatorResult):
            return acquisition

        planner_result = self._call_and_validate_planner(acquisition)
        if isinstance(planner_result, PhysicalIntentCoordinatorResult):
            return planner_result

        disposition = self._apply_proposal_disposition(planner_result)
        if disposition is not None:
            return disposition

        capture = self._capture_compilation_evidence(planner_result)
        if isinstance(capture, PhysicalIntentCoordinatorResult):
            return capture

        compiled = self._compile_execution_plan(planner_result, capture)
        if isinstance(compiled, PhysicalIntentCoordinatorResult):
            return compiled

        return self._commit_intent(planner_result, capture, compiled)


__all__ = (
    "CoordinatorOutcome",
    "IntentCompilationEvidence",
    "IntentCompilationRequest",
    "IntentPlanningRequest",
    "PhysicalIntentCoordinator",
    "PhysicalIntentCoordinatorError",
    "PhysicalIntentCoordinatorResult",
    "MAX_COMPILATION_EVIDENCE_BYTES",
    "MAX_TARGET_GEOMETRY_SIGNATURES",
)
