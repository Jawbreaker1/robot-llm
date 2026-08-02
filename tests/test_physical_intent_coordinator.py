from dataclasses import replace
import unittest

import robot_agent.physical_intent_contract as physical_intent_contract
import robot_agent.physical_intent_coordinator as physical_intent_coordinator
from robot_agent.navigation_intent_proposal import (
    ABORT,
    DETOUR_TARGET,
    FOLLOW_DIRECTION,
    HOLD,
    LEFT,
    NavigationIntentOffer,
    NavigationIntentProposal,
    RIGHT,
    SCAN_TARGET,
    bind_navigation_intent_proposal,
)
from robot_agent.physical_agent_state import (
    AgentPhase,
    ControllerKey,
    DetourSide,
    DetourTargetIntent,
    ExecutionPlan,
    FollowDirectionIntent,
    GoalActivated,
    GoalAssignment,
    GoalOutcome,
    NavigationBasis,
    NavigationBasisUpdated,
    PhysicalAgentState,
    PhysicalAgentStateReducer,
    PlanBinding,
    PlanningCause,
    PlanningHeld,
    PlanningRequested,
    PlanningTicket,
    PlanningTicketConsumed,
    PrimitiveStep,
    ReplanRequested,
    ScanTargetIntent,
    SensorStep,
)
from robot_agent.physical_intent_coordinator import (
    CoordinatorOutcome,
    IntentCompilationEvidence,
    MAX_COMPILATION_EVIDENCE_BYTES,
    PhysicalIntentCoordinator,
    PhysicalIntentCoordinatorError,
)


NOW_MS = 1_000


class PhysicalIntentContractImportTests(unittest.TestCase):
    def test_coordinator_reexports_the_public_contract(self):
        for name in (
            "CoordinatorOutcome",
            "IntentCompilationEvidence",
            "IntentCompilationRequest",
            "IntentPlanningRequest",
            "PhysicalIntentCoordinatorError",
            "PhysicalIntentCoordinatorResult",
        ):
            with self.subTest(name=name):
                self.assertIs(
                    getattr(physical_intent_coordinator, name),
                    getattr(physical_intent_contract, name),
                )

        for name in (
            "MAX_COMPILATION_EVIDENCE_BYTES",
            "MAX_TARGET_GEOMETRY_SIGNATURES",
        ):
            with self.subTest(name=name):
                self.assertEqual(
                    getattr(physical_intent_coordinator, name),
                    getattr(physical_intent_contract, name),
                )


def controller():
    return ControllerKey(
        robot_id="ev3rstorm-1",
        controller_id="drive-1",
        controller_instance_id="controller-instance-1",
    )


def navigation_basis(
    *,
    key=None,
    controller_version=1,
    world_version=1,
    basis_id="basis-1",
):
    return NavigationBasis(
        controller_key=key or controller(),
        goal_epoch=1,
        controller_state_version=controller_version,
        world_generation_id="world-generation-1",
        world_model_version=world_version,
        navigation_basis_id=basis_id,
        frame_id="robot-local-1",
        calibration_fingerprint="drive-calibration-a",
    )


def planning_reducer(*, valid_until_ms=10_000):
    key = controller()
    basis = navigation_basis(key=key)
    goal = GoalAssignment(
        goal_id="goal-1",
        goal_epoch=1,
        objective="Move forward while handling obstacles",
        source="USER",
        locale="sv",
        activated_at_ms=100,
    )
    ticket = PlanningTicket(
        ticket_id="ticket-1",
        cause=PlanningCause.NEW_GOAL,
        basis=basis,
        created_at_ms=101,
        valid_until_ms=valid_until_ms,
    )
    reducer = PhysicalAgentStateReducer(PhysicalAgentState(key))
    reducer.apply(GoalActivated(goal, basis, ticket))
    return reducer


class IdFactory:
    def __init__(self):
        self.calls = []
        self.counts = {}

    def __call__(self, namespace):
        self.calls.append(namespace)
        count = self.counts.get(namespace, 0) + 1
        self.counts[namespace] = count
        return "{}-{}".format(namespace, count)


class SequenceClock:
    def __init__(self, values):
        self.values = list(values)
        self.last = self.values[-1]

    def __call__(self):
        if self.values:
            self.last = self.values.pop(0)
        return self.last


class RecordingCompiler:
    def __init__(self, error=None, before_return=None):
        self.calls = []
        self.error = error
        self.before_return = before_return

    def __call__(self, request):
        self.calls.append(request)
        if self.error is not None:
            raise self.error
        state = request.state
        basis = state.basis
        intent = request.intent
        binding = PlanBinding(
            controller_key=state.controller_key,
            goal_id=state.goal.goal_id,
            goal_epoch=state.goal_epoch,
            intent_id=intent.intent_id,
            intent_revision=intent.revision,
            frame_id=basis.frame_id,
            world_generation_id=basis.world_generation_id,
            calibration_fingerprint=basis.calibration_fingerprint,
            based_on_navigation_basis_id=basis.navigation_basis_id,
            target_geometry_signatures=(
                request.evidence.target_geometry_signatures
            ),
        )
        if isinstance(intent.payload, ScanTargetIntent):
            steps = (
                SensorStep(
                    step_id="scan-step-1",
                    operation="SCAN_FRONT_ARC",
                    target_hypothesis_id=(
                        intent.payload.target_hypothesis_id
                    ),
                    profile_id=intent.payload.scan_profile_id,
                ),
            )
        else:
            steps = (PrimitiveStep("advance-step-1", "ADVANCE"),)
        plan = ExecutionPlan(
            plan_id=request.plan_id,
            revision=request.plan_revision,
            binding=binding,
            steps=steps,
            cursor=0,
            created_at_ms=request.created_at_ms,
        )
        if self.before_return is not None:
            self.before_return(request)
        return plan


class RecordingEvidenceProvider:
    def __init__(self, error=None, before_return=None):
        self.calls = []
        self.error = error
        self.before_return = before_return

    def __call__(self, state, proposal):
        self.calls.append((state, proposal))
        if self.error is not None:
            raise self.error
        if proposal.intent in (SCAN_TARGET, DETOUR_TARGET):
            signatures = ((proposal.target_id, "geometry-signature-1"),)
            targets = {
                proposal.target_id: {
                    "geometry_signature": "geometry-signature-1",
                },
            }
        else:
            signatures = ()
            targets = {}
        evidence = IntentCompilationEvidence.capture(
            basis=state.basis,
            snapshot={
                "navigation_basis_id": state.basis.navigation_basis_id,
                "targets": targets,
            },
            target_geometry_signatures=signatures,
        )
        if self.before_return is not None:
            self.before_return(state, proposal)
        return evidence


def envelope_for(
    request,
    proposal,
    *,
    proposal_id=None,
    ticket_id=None,
    basis=None,
    received_at_ms=NOW_MS,
    valid_until_ms=NOW_MS + 1_000,
):
    choices = {
        "ticket_id": ticket_id or request.ticket.ticket_id,
        "basis": basis or request.ticket.basis,
        "offered_intents": (proposal.intent,),
        "scan_target_ids": (),
        "detour_target_ids": (),
        "detour_sides": (),
        "hold_reasons": (),
        "abort_reasons": (),
    }
    if proposal.intent == SCAN_TARGET:
        choices["scan_target_ids"] = (proposal.target_id,)
    elif proposal.intent == DETOUR_TARGET:
        choices["detour_target_ids"] = (proposal.target_id,)
        choices["detour_sides"] = (proposal.side,)
    elif proposal.intent == HOLD:
        choices["hold_reasons"] = (proposal.reason,)
    elif proposal.intent == ABORT:
        choices["abort_reasons"] = (proposal.reason,)
    offer = NavigationIntentOffer(**choices)
    return bind_navigation_intent_proposal(
        proposal,
        offer=offer,
        proposal_id=proposal_id or request.proposal_id,
        received_at_ms=received_at_ms,
        valid_until_ms=valid_until_ms,
    )


def coordinator(
    reducer,
    planner,
    compiler=None,
    ids=None,
    evidence_provider=None,
    clock=None,
    *,
    abort_outcome=GoalOutcome.FAILED,
):
    return PhysicalIntentCoordinator(
        reducer=reducer,
        intent_planner=planner,
        compilation_evidence_provider=(
            evidence_provider or RecordingEvidenceProvider()
        ),
        plan_compiler=compiler or RecordingCompiler(),
        clock_ms=clock or (lambda: NOW_MS),
        id_factory=ids or IdFactory(),
        default_scan_profile_id="front-arc-profile-a",
        abort_outcome=abort_outcome,
    )


class IntentCompilationEvidenceTests(unittest.TestCase):
    def test_capture_is_canonical_bounded_and_detached_from_mutation(self):
        source = {
            "targets": {
                "hazard-1": {
                    "right_mm": 80,
                    "left_mm": -40,
                },
            },
            "pose": {"y_mm": 0, "x_mm": 10},
        }

        evidence = IntentCompilationEvidence.capture(
            basis=navigation_basis(),
            snapshot=source,
            target_geometry_signatures=(
                ("hazard-1", "geometry-signature-1"),
            ),
        )
        source["pose"]["x_mm"] = 999
        decoded = evidence.snapshot()
        decoded["pose"]["x_mm"] = 500

        self.assertEqual(evidence.snapshot()["pose"]["x_mm"], 10)
        self.assertEqual(
            evidence.snapshot_json,
            (
                b'{"pose":{"x_mm":10,"y_mm":0},"targets":'
                b'{"hazard-1":{"left_mm":-40,"right_mm":80}}}'
            ),
        )

    def test_rejects_noncanonical_oversized_or_unordered_evidence(self):
        invalid_values = (
            {
                "snapshot_json": b'{"value": 1}',
                "target_geometry_signatures": (),
            },
            {
                "snapshot_json": b"{" + b" " * MAX_COMPILATION_EVIDENCE_BYTES,
                "target_geometry_signatures": (),
            },
            {
                "snapshot_json": b"{}",
                "target_geometry_signatures": (
                    ("hazard-b", "signature-b"),
                    ("hazard-a", "signature-a"),
                ),
            },
        )
        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(PhysicalIntentCoordinatorError):
                    IntentCompilationEvidence(
                        basis=navigation_basis(),
                        **value
                    )


class PhysicalIntentCoordinatorEligibilityTests(unittest.TestCase):
    def test_planner_is_called_only_for_one_unconsumed_planning_ticket(self):
        reducer = planning_reducer()
        calls = []

        def planner(request):
            calls.append(request)
            current = reducer.snapshot()
            self.assertTrue(current.planning_ticket.consumed)
            self.assertEqual(current, request.state)
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            )

        compiler = RecordingCompiler()
        worker = coordinator(reducer, planner, compiler)

        result = worker.run_once()
        second = worker.run_once()

        self.assertEqual(result.outcome, CoordinatorOutcome.INTENT_ACCEPTED)
        self.assertEqual(result.state.phase, AgentPhase.EXECUTING)
        self.assertEqual(len(calls), 1)
        self.assertEqual(len(compiler.calls), 1)
        self.assertEqual(second.outcome, CoordinatorOutcome.NO_WORK)
        self.assertEqual(len(calls), 1)

    def test_idle_held_and_already_consumed_states_do_not_call_planner(self):
        calls = []

        def planner(_request):
            calls.append(1)
            raise AssertionError("planner must not be called")

        idle = PhysicalAgentStateReducer(PhysicalAgentState(controller()))
        self.assertEqual(
            coordinator(idle, planner).run_once().outcome,
            CoordinatorOutcome.NO_WORK,
        )

        consumed = planning_reducer()
        ticket = consumed.snapshot().planning_ticket
        consumed.apply(
            PlanningTicketConsumed(ticket.ticket_id, ticket.basis, NOW_MS)
        )
        self.assertEqual(
            coordinator(consumed, planner).run_once().outcome,
            CoordinatorOutcome.NO_WORK,
        )
        consumed.apply(PlanningHeld(ticket.ticket_id, ticket.basis))
        self.assertEqual(
            coordinator(consumed, planner).run_once().outcome,
            CoordinatorOutcome.NO_WORK,
        )
        self.assertEqual(calls, [])

    def test_reentrant_run_cannot_duplicate_the_planner_call(self):
        reducer = planning_reducer()
        calls = []
        nested = []
        holder = {}

        def planner(request):
            calls.append(request)
            nested.append(holder["worker"].run_once())
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            )

        holder["worker"] = coordinator(reducer, planner)

        result = holder["worker"].run_once()

        self.assertEqual(result.outcome, CoordinatorOutcome.INTENT_ACCEPTED)
        self.assertEqual(len(calls), 1)
        self.assertEqual(nested[0].outcome, CoordinatorOutcome.NO_WORK)

    def test_parallel_hold_keeps_the_authoritative_plan_executing(self):
        reducer = planning_reducer()

        def initial_planner(request):
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            )

        initial = coordinator(reducer, initial_planner).run_once()
        active_plan = initial.state.plan
        basis = initial.state.basis
        parallel_ticket = PlanningTicket(
            ticket_id="parallel-ticket-1",
            cause=PlanningCause.UNCERTAINTY,
            basis=basis,
            created_at_ms=1_001,
            valid_until_ms=10_000,
        )
        reducer.apply(PlanningRequested(parallel_ticket))
        calls = []

        def parallel_planner(request):
            calls.append(request)
            self.assertEqual(request.state.phase, AgentPhase.EXECUTING)
            self.assertEqual(request.state.plan, active_plan)
            return envelope_for(
                request,
                NavigationIntentProposal(
                    intent=HOLD,
                    reason="WAIT_FOR_EVIDENCE",
                ),
                received_at_ms=1_002,
                valid_until_ms=2_002,
            )

        result = coordinator(
            reducer,
            parallel_planner,
            clock=lambda: 1_002,
        ).run_once()

        self.assertEqual(result.outcome, CoordinatorOutcome.HELD)
        self.assertEqual(result.state.phase, AgentPhase.EXECUTING)
        self.assertEqual(result.state.plan, active_plan)
        self.assertIsNone(result.state.planning_ticket)
        self.assertEqual(len(calls), 1)

    def test_expired_ticket_is_cleared_without_calling_the_model(self):
        for consumed in (False, True):
            with self.subTest(consumed=consumed):
                reducer = planning_reducer()
                ticket = reducer.snapshot().planning_ticket
                if consumed:
                    reducer.apply(
                        PlanningTicketConsumed(
                            ticket.ticket_id,
                            ticket.basis,
                            NOW_MS,
                        )
                    )
                calls = []
                result = coordinator(
                    reducer,
                    lambda request: calls.append(request),
                    clock=lambda: ticket.valid_until_ms,
                ).run_once()

                self.assertEqual(
                    result.outcome,
                    CoordinatorOutcome.TICKET_EXPIRED,
                )
                self.assertEqual(result.error_code, "planning_ticket_expired")
                self.assertIsNone(result.state.planning_ticket)
                self.assertEqual(calls, [])


class PhysicalIntentCoordinatorMappingTests(unittest.TestCase):
    def _run(self, proposal, *, abort_outcome=GoalOutcome.FAILED):
        reducer = planning_reducer()
        planner_calls = []

        def planner(request):
            planner_calls.append(request)
            return envelope_for(request, proposal)

        compiler = RecordingCompiler()
        result = coordinator(
            reducer,
            planner,
            compiler,
            abort_outcome=abort_outcome,
        ).run_once()
        return result, planner_calls, compiler.calls

    def test_follow_direction_maps_to_typed_active_intent(self):
        result, planner_calls, compiler_calls = self._run(
            NavigationIntentProposal(intent=FOLLOW_DIRECTION)
        )

        self.assertEqual(result.outcome, CoordinatorOutcome.INTENT_ACCEPTED)
        self.assertIsInstance(result.state.intent.payload, FollowDirectionIntent)
        self.assertEqual(
            result.state.intent.accepted_basis,
            planner_calls[0].ticket.basis,
        )
        self.assertEqual(result.state.intent.revision, 1)
        self.assertEqual(compiler_calls[0].proposal.intent, FOLLOW_DIRECTION)

    def test_scan_uses_the_injected_default_profile(self):
        result, _planner_calls, compiler_calls = self._run(
            NavigationIntentProposal(
                intent=SCAN_TARGET,
                target_id="hazard-1",
            )
        )

        payload = result.state.intent.payload
        self.assertIsInstance(payload, ScanTargetIntent)
        self.assertEqual(payload.target_hypothesis_id, "hazard-1")
        self.assertEqual(payload.scan_profile_id, "front-arc-profile-a")
        self.assertIsInstance(result.state.plan.active_step, SensorStep)
        self.assertEqual(len(compiler_calls), 1)
        evidence = compiler_calls[0].evidence
        self.assertEqual(
            evidence.snapshot()["targets"]["hazard-1"][
                "geometry_signature"
            ],
            "geometry-signature-1",
        )
        self.assertEqual(
            result.state.plan.binding.target_geometry_signatures,
            evidence.target_geometry_signatures,
        )

    def test_detour_side_maps_without_text_classification(self):
        cases = (
            (LEFT, DetourSide.LEFT_OF_GOAL),
            (RIGHT, DetourSide.RIGHT_OF_GOAL),
        )
        for proposed_side, expected_side in cases:
            with self.subTest(side=proposed_side):
                result, _planner_calls, compiler_calls = self._run(
                    NavigationIntentProposal(
                        intent=DETOUR_TARGET,
                        target_id="hazard-1",
                        side=proposed_side,
                    )
                )
                payload = result.state.intent.payload
                self.assertIsInstance(payload, DetourTargetIntent)
                self.assertEqual(payload.target_hypothesis_id, "hazard-1")
                self.assertEqual(payload.detour_side, expected_side)
                self.assertEqual(len(compiler_calls), 1)

    def test_hold_consumes_ticket_and_requires_explicit_replanning(self):
        result, planner_calls, compiler_calls = self._run(
            NavigationIntentProposal(
                intent=HOLD,
                reason="WAIT_FOR_EVIDENCE",
            )
        )

        self.assertEqual(result.outcome, CoordinatorOutcome.HELD)
        self.assertEqual(result.state.phase, AgentPhase.PLANNING)
        self.assertIsNone(result.state.planning_ticket)
        self.assertEqual(len(planner_calls), 1)
        self.assertEqual(compiler_calls, [])

    def test_abort_outcome_is_host_owned_and_never_compiles_motion(self):
        for host_outcome in (GoalOutcome.CANCELLED, GoalOutcome.FAILED):
            with self.subTest(outcome=host_outcome):
                result, planner_calls, compiler_calls = self._run(
                    NavigationIntentProposal(
                        intent=ABORT,
                        reason="NO_SAFE_ROUTE",
                    ),
                    abort_outcome=host_outcome,
                )
                self.assertEqual(result.outcome, CoordinatorOutcome.ABORTED)
                self.assertEqual(result.state.phase, AgentPhase.STOPPING)
                self.assertEqual(result.state.terminal.outcome, host_outcome)
                self.assertEqual(
                    result.state.terminal.reason,
                    "NO_SAFE_ROUTE",
                )
                self.assertIsNone(result.state.plan)
                self.assertEqual(len(planner_calls), 1)
                self.assertEqual(compiler_calls, [])

    def test_succeeded_is_not_a_valid_host_abort_policy(self):
        with self.assertRaises(PhysicalIntentCoordinatorError):
            coordinator(
                planning_reducer(),
                lambda _request: None,
                abort_outcome=GoalOutcome.SUCCEEDED,
            )


class PhysicalIntentCoordinatorFreshnessTests(unittest.TestCase):
    def test_ticket_ttl_is_rechecked_after_model_and_before_commit(self):
        cases = (
            ((NOW_MS, 1_050), 0),
            ((NOW_MS, NOW_MS, NOW_MS, 1_050), 1),
        )
        for clock_values, expected_compiler_calls in cases:
            with self.subTest(clock_values=clock_values):
                reducer = planning_reducer(valid_until_ms=1_050)
                planner_calls = []

                def planner(request):
                    planner_calls.append(request)
                    return envelope_for(
                        request,
                        NavigationIntentProposal(intent=FOLLOW_DIRECTION),
                    )

                compiler = RecordingCompiler()
                result = coordinator(
                    reducer,
                    planner,
                    compiler,
                    clock=SequenceClock(clock_values),
                ).run_once()

                self.assertEqual(
                    result.outcome,
                    CoordinatorOutcome.TICKET_EXPIRED,
                )
                self.assertEqual(result.error_code, "planning_ticket_expired")
                self.assertIsNone(result.state.planning_ticket)
                self.assertEqual(len(planner_calls), 1)
                self.assertEqual(len(compiler.calls), expected_compiler_calls)

    def test_accepts_decision_equivalent_basis_advance_during_planning(self):
        reducer = planning_reducer()

        def planner(request):
            current = reducer.snapshot().basis
            reducer.apply(
                NavigationBasisUpdated(
                    replace(
                        current,
                        controller_state_version=(
                            current.controller_state_version + 1
                        ),
                        world_model_version=current.world_model_version + 1,
                    )
                )
            )
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            )

        compiler = RecordingCompiler()
        result = coordinator(reducer, planner, compiler).run_once()

        self.assertEqual(result.outcome, CoordinatorOutcome.INTENT_ACCEPTED)
        self.assertEqual(
            result.state.basis.controller_state_version,
            2,
        )
        self.assertEqual(
            compiler.calls[0].state.basis.controller_state_version,
            2,
        )

    def test_non_equivalent_basis_supersedes_result_without_compiling(self):
        reducer = planning_reducer()

        def planner(request):
            current = reducer.snapshot().basis
            reducer.apply(
                NavigationBasisUpdated(
                    replace(
                        current,
                        controller_state_version=(
                            current.controller_state_version + 1
                        ),
                        world_model_version=current.world_model_version + 1,
                        navigation_basis_id="basis-changed",
                    )
                )
            )
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            )

        compiler = RecordingCompiler()
        result = coordinator(reducer, planner, compiler).run_once()

        self.assertEqual(result.outcome, CoordinatorOutcome.SUPERSEDED)
        self.assertEqual(result.state.phase, AgentPhase.PLANNING)
        self.assertIsNone(result.state.planning_ticket)
        self.assertEqual(compiler.calls, [])

    def test_evidence_capture_requires_the_exact_current_basis(self):
        reducer = planning_reducer()

        def planner(request):
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            )

        def mismatched_provider(state, _proposal):
            return IntentCompilationEvidence.capture(
                basis=replace(
                    state.basis,
                    controller_state_version=(
                        state.basis.controller_state_version + 1
                    ),
                    world_model_version=state.basis.world_model_version + 1,
                ),
                snapshot={"route": "clear"},
            )

        compiler = RecordingCompiler()
        result = coordinator(
            reducer,
            planner,
            compiler,
            evidence_provider=mismatched_provider,
        ).run_once()

        self.assertEqual(result.outcome, CoordinatorOutcome.EVIDENCE_FAILED)
        self.assertEqual(
            result.error_code,
            "compilation_evidence_basis_mismatch",
        )
        self.assertEqual(result.state.phase, AgentPhase.PLANNING)
        self.assertIsNone(result.state.planning_ticket)
        self.assertEqual(compiler.calls, [])

    def test_basis_change_during_capture_is_not_silently_rebound(self):
        reducer = planning_reducer()

        def planner(request):
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            )

        def advance_basis(_state, _proposal):
            current = reducer.snapshot().basis
            reducer.apply(
                NavigationBasisUpdated(
                    replace(
                        current,
                        controller_state_version=(
                            current.controller_state_version + 1
                        ),
                        world_model_version=current.world_model_version + 1,
                    )
                )
            )

        evidence_provider = RecordingEvidenceProvider(
            before_return=advance_basis
        )
        compiler = RecordingCompiler()
        result = coordinator(
            reducer,
            planner,
            compiler,
            evidence_provider=evidence_provider,
        ).run_once()

        self.assertEqual(result.outcome, CoordinatorOutcome.EVIDENCE_FAILED)
        self.assertEqual(
            result.error_code,
            "compilation_evidence_basis_mismatch",
        )
        self.assertEqual(compiler.calls, [])

    def test_decision_equivalent_change_after_capture_can_commit(self):
        reducer = planning_reducer()

        def planner(request):
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            )

        def advance_basis(_request):
            current = reducer.snapshot().basis
            reducer.apply(
                NavigationBasisUpdated(
                    replace(
                        current,
                        controller_state_version=(
                            current.controller_state_version + 1
                        ),
                        world_model_version=current.world_model_version + 1,
                    )
                )
            )

        compiler = RecordingCompiler(before_return=advance_basis)
        result = coordinator(reducer, planner, compiler).run_once()

        self.assertEqual(result.outcome, CoordinatorOutcome.INTENT_ACCEPTED)
        self.assertEqual(result.state.phase, AgentPhase.EXECUTING)
        self.assertEqual(result.state.basis.controller_state_version, 2)
        self.assertEqual(
            compiler.calls[0].evidence.basis.controller_state_version,
            1,
        )

    def test_target_evidence_and_plan_signatures_must_match(self):
        proposal = NavigationIntentProposal(
            intent=DETOUR_TARGET,
            target_id="hazard-1",
            side=LEFT,
        )

        def planner(request):
            return envelope_for(request, proposal)

        def missing_target_provider(state, _proposal):
            return IntentCompilationEvidence.capture(
                basis=state.basis,
                snapshot={"targets": {}},
            )

        missing_reducer = planning_reducer()
        missing_compiler = RecordingCompiler()
        missing = coordinator(
            missing_reducer,
            planner,
            missing_compiler,
            evidence_provider=missing_target_provider,
        ).run_once()

        self.assertEqual(missing.outcome, CoordinatorOutcome.EVIDENCE_FAILED)
        self.assertEqual(
            missing.error_code,
            "missing_target_geometry_signature",
        )
        self.assertEqual(missing_compiler.calls, [])

        class SignatureDroppingCompiler(RecordingCompiler):
            def __call__(self, request):
                plan = super().__call__(request)
                return replace(
                    plan,
                    binding=replace(
                        plan.binding,
                        target_geometry_signatures=(),
                    ),
                )

        mismatch_reducer = planning_reducer()
        mismatch_compiler = SignatureDroppingCompiler()
        mismatch = coordinator(
            mismatch_reducer,
            planner,
            mismatch_compiler,
        ).run_once()

        self.assertEqual(mismatch.outcome, CoordinatorOutcome.COMPILER_FAILED)
        self.assertEqual(
            mismatch.error_code,
            "compiled_plan_geometry_mismatch",
        )
        self.assertEqual(mismatch.state.phase, AgentPhase.PLANNING)
        self.assertIsNone(mismatch.state.planning_ticket)

    def test_envelope_ttl_is_rechecked_after_compilation(self):
        reducer = planning_reducer()

        def planner(request):
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
                received_at_ms=NOW_MS,
                valid_until_ms=NOW_MS + 1_000,
            )

        compiler = RecordingCompiler()
        result = coordinator(
            reducer,
            planner,
            compiler,
            clock=SequenceClock(
                (NOW_MS, NOW_MS, NOW_MS, NOW_MS + 1_000)
            ),
        ).run_once()

        self.assertEqual(result.outcome, CoordinatorOutcome.PROPOSAL_REJECTED)
        self.assertEqual(result.error_code, "expired_proposal")
        self.assertEqual(result.state.phase, AgentPhase.PLANNING)
        self.assertIsNone(result.state.planning_ticket)
        self.assertEqual(len(compiler.calls), 1)

    def test_rejects_expired_or_misbound_envelopes_and_holds(self):
        def expired(request):
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
                received_at_ms=900,
                valid_until_ms=NOW_MS,
            )

        def wrong_proposal(request):
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
                proposal_id="wrong-proposal-id",
            )

        def wrong_ticket(request):
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
                ticket_id="wrong-ticket-id",
            )

        def wrong_exact_basis(request):
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
                basis=replace(
                    request.ticket.basis,
                    controller_state_version=(
                        request.ticket.basis.controller_state_version + 1
                    ),
                    world_model_version=(
                        request.ticket.basis.world_model_version + 1
                    ),
                ),
            )

        for name, planner in (
            ("expired", expired),
            ("proposal", wrong_proposal),
            ("ticket", wrong_ticket),
            ("basis", wrong_exact_basis),
        ):
            with self.subTest(case=name):
                reducer = planning_reducer()
                compiler = RecordingCompiler()
                result = coordinator(reducer, planner, compiler).run_once()
                self.assertEqual(
                    result.outcome,
                    CoordinatorOutcome.PROPOSAL_REJECTED,
                )
                self.assertEqual(result.state.phase, AgentPhase.PLANNING)
                self.assertIsNone(result.state.planning_ticket)
                self.assertEqual(compiler.calls, [])


class PhysicalIntentCoordinatorFailureTests(unittest.TestCase):
    def test_evidence_provider_failure_holds_without_compiling_or_retrying(self):
        reducer = planning_reducer()
        planner_calls = []

        def planner(request):
            planner_calls.append(request)
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            )

        provider = RecordingEvidenceProvider(
            RuntimeError("map snapshot unavailable")
        )
        compiler = RecordingCompiler()
        worker = coordinator(
            reducer,
            planner,
            compiler,
            evidence_provider=provider,
        )

        first = worker.run_once()
        second = worker.run_once()

        self.assertEqual(first.outcome, CoordinatorOutcome.EVIDENCE_FAILED)
        self.assertEqual(first.error_code, "compilation_evidence_failed")
        self.assertEqual(first.state.phase, AgentPhase.PLANNING)
        self.assertIsNone(first.state.planning_ticket)
        self.assertEqual(second.outcome, CoordinatorOutcome.NO_WORK)
        self.assertEqual(len(planner_calls), 1)
        self.assertEqual(len(provider.calls), 1)
        self.assertEqual(compiler.calls, [])

    def test_host_id_failure_holds_before_any_planner_call(self):
        reducer = planning_reducer()
        planner_calls = []

        def planner(_request):
            planner_calls.append(1)
            raise AssertionError("planner must not be called")

        def broken_ids(_namespace):
            raise RuntimeError("ID service unavailable")

        worker = coordinator(
            reducer,
            planner,
            ids=broken_ids,
        )

        first = worker.run_once()
        second = worker.run_once()

        self.assertEqual(first.outcome, CoordinatorOutcome.HOST_FAILED)
        self.assertEqual(first.error_code, "proposal_id_failed")
        self.assertEqual(first.state.phase, AgentPhase.PLANNING)
        self.assertIsNone(first.state.planning_ticket)
        self.assertEqual(second.outcome, CoordinatorOutcome.NO_WORK)
        self.assertEqual(planner_calls, [])

    def test_planner_failure_is_held_once_without_retry_spin(self):
        reducer = planning_reducer()
        calls = []

        def planner(_request):
            calls.append(1)
            raise RuntimeError("model unavailable")

        worker = coordinator(reducer, planner)
        first = worker.run_once()
        second = worker.run_once()

        self.assertEqual(first.outcome, CoordinatorOutcome.PLANNER_FAILED)
        self.assertEqual(first.error_code, "intent_planner_failed")
        self.assertEqual(first.state.phase, AgentPhase.PLANNING)
        self.assertIsNone(first.state.planning_ticket)
        self.assertEqual(second.outcome, CoordinatorOutcome.NO_WORK)
        self.assertEqual(calls, [1])

    def test_compiler_failure_is_held_once_without_retry_spin(self):
        reducer = planning_reducer()
        planner_calls = []

        def planner(request):
            planner_calls.append(request)
            return envelope_for(
                request,
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            )

        compiler = RecordingCompiler(RuntimeError("geometry unavailable"))
        worker = coordinator(reducer, planner, compiler)
        first = worker.run_once()
        second = worker.run_once()

        self.assertEqual(first.outcome, CoordinatorOutcome.COMPILER_FAILED)
        self.assertEqual(first.error_code, "plan_compiler_failed")
        self.assertEqual(first.state.phase, AgentPhase.PLANNING)
        self.assertIsNone(first.state.planning_ticket)
        self.assertEqual(second.outcome, CoordinatorOutcome.NO_WORK)
        self.assertEqual(len(planner_calls), 1)
        self.assertEqual(len(compiler.calls), 1)

    def test_non_envelope_planner_result_is_rejected_without_compiler(self):
        reducer = planning_reducer()
        compiler = RecordingCompiler()

        result = coordinator(
            reducer,
            lambda _request: {"intent": FOLLOW_DIRECTION},
            compiler,
        ).run_once()

        self.assertEqual(result.outcome, CoordinatorOutcome.PROPOSAL_REJECTED)
        self.assertEqual(result.error_code, "invalid_planner_result")
        self.assertEqual(result.state.phase, AgentPhase.PLANNING)
        self.assertIsNone(result.state.planning_ticket)
        self.assertEqual(compiler.calls, [])


if __name__ == "__main__":
    unittest.main()
