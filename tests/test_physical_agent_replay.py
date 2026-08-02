import unittest
from dataclasses import replace

from robot_agent.navigation_intent_proposal import (
    FOLLOW_DIRECTION,
    NavigationIntentOffer,
    NavigationIntentProposal,
    bind_navigation_intent_proposal,
)
from robot_agent.physical_agent_state import (
    AgentPhase,
    ControllerCommandReceipt,
    ControllerKey,
    ExecutionPlan,
    GoalActivated,
    GoalAssignment,
    GoalOutcome,
    GoalTerminal,
    NavigationBasis,
    PhysicalAgentState,
    PhysicalAgentStateReducer,
    PlanBinding,
    PlanRecompiled,
    PlanStepKey,
    PlanningCause,
    PlanningTicket,
    PrimitiveStep,
    ReceiptOutcome,
    ReplanRequested,
    StepCommandAuthorization,
    StepCommandAuthorized,
    StepCommandDispatched,
    StepCommandSettled,
    StepDisposition,
    StopRequested,
    StopVerified,
)
from robot_agent.physical_intent_coordinator import (
    CoordinatorOutcome,
    IntentCompilationEvidence,
    PhysicalIntentCoordinator,
)


NOW_MS = 1_000


class SequentialIds:
    def __init__(self):
        self.counts = {}

    def __call__(self, namespace):
        count = self.counts.get(namespace, 0) + 1
        self.counts[namespace] = count
        return "{}-{}".format(namespace, count)


class CountingPlanner:
    """Fake the only model boundary and retain every invocation."""

    def __init__(self):
        self.calls = []

    def __call__(self, request):
        self.calls.append(request)
        offer = NavigationIntentOffer(
            ticket_id=request.ticket.ticket_id,
            basis=request.ticket.basis,
            offered_intents=(FOLLOW_DIRECTION,),
        )
        return bind_navigation_intent_proposal(
            NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            offer=offer,
            proposal_id=request.proposal_id,
            received_at_ms=NOW_MS,
            valid_until_ms=NOW_MS + 1_000,
        )


class RecordingCompiler:
    """Compile model intents and ticket-free continuations deterministically."""

    def __init__(self):
        self.model_intent_calls = []
        self.continuation_calls = []

    @staticmethod
    def _plan(state, intent, basis, plan_id, revision, created_at_ms):
        return ExecutionPlan(
            plan_id=plan_id,
            revision=revision,
            binding=PlanBinding(
                controller_key=state.controller_key,
                goal_id=state.goal.goal_id,
                goal_epoch=state.goal_epoch,
                intent_id=intent.intent_id,
                intent_revision=intent.revision,
                frame_id=basis.frame_id,
                world_generation_id=basis.world_generation_id,
                calibration_fingerprint=basis.calibration_fingerprint,
                based_on_navigation_basis_id=basis.navigation_basis_id,
                target_geometry_signatures=(),
            ),
            steps=(
                PrimitiveStep("advance-step", "ADVANCE"),
                PrimitiveStep("observe-step", "ADVANCE"),
            ),
            cursor=0,
            created_at_ms=created_at_ms,
        )

    def __call__(self, request):
        self.model_intent_calls.append(request)
        return self._plan(
            request.state,
            request.intent,
            request.evidence.basis,
            request.plan_id,
            request.plan_revision,
            request.created_at_ms,
        )

    def compile_continuation(self, state, resulting_basis):
        if not state.compile_pending or state.intent is None:
            raise AssertionError("continuation compilation was not pending")
        self.continuation_calls.append((state, resulting_basis))
        return self._plan(
            state,
            state.intent,
            resulting_basis,
            "deterministic-plan-{}".format(state.plan_revision + 1),
            state.plan_revision + 1,
            NOW_MS + 100,
        )


def compilation_evidence(state, _proposal):
    return IntentCompilationEvidence.capture(
        basis=state.basis,
        snapshot={
            "navigation_basis_id": state.basis.navigation_basis_id,
            "route": "clear",
        },
    )


def successor(current, *, controller_version, basis_id):
    return replace(
        current,
        controller_state_version=controller_version,
        world_model_version=controller_version,
        navigation_basis_id=basis_id,
    )


def settle_completed_step(reducer, resulting_basis, *, now_ms):
    state = reducer.snapshot()
    sequence = state.last_host_dispatch_sequence + 1
    authorization = StepCommandAuthorization(
        action_id="replay-action-{}".format(sequence),
        command_id="replay-command-{}".format(sequence),
        host_dispatch_sequence=sequence,
        controller_key=state.controller_key,
        step_key=PlanStepKey(
            plan_id=state.plan.plan_id,
            plan_revision=state.plan.revision,
            cursor=state.plan.cursor,
            step_id=state.plan.active_step.step_id,
        ),
        based_on_navigation_basis_id=state.basis.navigation_basis_id,
        based_on_controller_state_version=(
            state.basis.controller_state_version
        ),
        command_fingerprint="sha256:replay-command-{}".format(sequence),
        issued_at_ms=now_ms,
        valid_until_ms=now_ms + 1_000,
    )
    authorized = reducer.apply(StepCommandAuthorized(authorization))
    dispatched = reducer.apply(
        StepCommandDispatched(
            authorization=authorization,
            dispatched_at_ms=now_ms + 1,
            settle_by_host_ms=now_ms + 30_000,
        )
    )
    receipt = ControllerCommandReceipt(
        outcome=ReceiptOutcome.COMPLETED,
        controller_key=authorization.controller_key,
        step_key=authorization.step_key,
        action_id=authorization.action_id,
        command_id=authorization.command_id,
        host_dispatch_sequence=authorization.host_dispatch_sequence,
        command_fingerprint=authorization.command_fingerprint,
        based_on_navigation_basis_id=(
            authorization.based_on_navigation_basis_id
        ),
        based_on_controller_state_version=(
            authorization.based_on_controller_state_version
        ),
        resulting_controller_state_version=(
            resulting_basis.controller_state_version
        ),
        received_at_host_ms=now_ms + 2,
        stop_confirmed=True,
        code="completed",
    )
    settled = reducer.apply(
        StepCommandSettled(
            receipt=receipt,
            resulting_basis=resulting_basis,
            disposition=StepDisposition.COMPLETE,
        )
    )
    return authorized, dispatched, settled


class PhysicalAgentReplayTests(unittest.TestCase):
    def test_finished_plan_continues_without_model_then_replans_once(self):
        key = ControllerKey(
            robot_id="ev3rstorm-1",
            controller_id="drive-1",
            controller_instance_id="replay-controller-1",
        )
        initial_basis = NavigationBasis(
            controller_key=key,
            goal_epoch=1,
            controller_state_version=1,
            world_generation_id="world-generation-1",
            world_model_version=1,
            navigation_basis_id="basis-goal",
            frame_id="robot-local-1",
            calibration_fingerprint="drive-calibration-a",
        )
        goal = GoalAssignment(
            goal_id="goal-1",
            goal_epoch=1,
            objective="Continue through the room and adapt to obstacles",
            source="USER",
            locale="sv",
            activated_at_ms=100,
        )
        first_ticket = PlanningTicket(
            ticket_id="ticket-new-goal",
            cause=PlanningCause.NEW_GOAL,
            basis=initial_basis,
            created_at_ms=101,
            valid_until_ms=5_000,
        )
        reducer = PhysicalAgentStateReducer(PhysicalAgentState(key))
        planner = CountingPlanner()
        compiler = RecordingCompiler()
        coordinator = PhysicalIntentCoordinator(
            reducer=reducer,
            intent_planner=planner,
            compilation_evidence_provider=compilation_evidence,
            plan_compiler=compiler,
            clock_ms=lambda: NOW_MS,
            id_factory=SequentialIds(),
            default_scan_profile_id="front-arc-profile-a",
        )

        phase_trace = [reducer.snapshot().phase]
        activated = reducer.apply(
            GoalActivated(goal, initial_basis, first_ticket)
        )
        phase_trace.append(activated.phase)

        first_planning = coordinator.run_once()
        phase_trace.append(first_planning.state.phase)
        first_intent = first_planning.state.intent
        first_plan = first_planning.state.plan

        self.assertEqual(first_planning.outcome, CoordinatorOutcome.INTENT_ACCEPTED)
        self.assertEqual(len(planner.calls), 1)
        self.assertEqual(len(compiler.model_intent_calls), 1)
        self.assertEqual(len(first_plan.steps), 2)
        self.assertEqual(first_plan.cursor, 0)
        self.assertEqual(coordinator.run_once().outcome, CoordinatorOutcome.NO_WORK)
        self.assertEqual(len(planner.calls), 1)

        step_basis = successor(
            initial_basis,
            controller_version=2,
            basis_id="basis-step-one",
        )
        authorized, dispatched, advanced = settle_completed_step(
            reducer,
            step_basis,
            now_ms=1_010,
        )
        self.assertEqual(authorized.plan.cursor, 0)
        self.assertEqual(dispatched.plan.cursor, 0)
        phase_trace.append(advanced.phase)
        self.assertEqual(advanced.plan.cursor, 1)
        self.assertFalse(advanced.compile_pending)

        finished_basis = successor(
            step_basis,
            controller_version=3,
            basis_id="basis-plan-finished",
        )
        authorized, dispatched, finished = settle_completed_step(
            reducer,
            finished_basis,
            now_ms=1_020,
        )
        self.assertEqual(authorized.plan.cursor, 1)
        self.assertEqual(dispatched.plan.cursor, 1)
        phase_trace.append(finished.phase)

        self.assertEqual(finished.phase, AgentPhase.PLANNING)
        self.assertTrue(finished.compile_pending)
        self.assertEqual(finished.intent, first_intent)
        self.assertEqual(finished.intent_progress.completed_steps, 2)
        self.assertIsNone(finished.plan)
        self.assertIsNone(finished.planning_ticket)
        self.assertEqual(coordinator.run_once().outcome, CoordinatorOutcome.NO_WORK)
        self.assertEqual(len(planner.calls), 1)

        continuation_basis = successor(
            finished_basis,
            controller_version=4,
            basis_id="basis-continuation",
        )
        continuation_plan = compiler.compile_continuation(
            finished,
            continuation_basis,
        )
        continued = reducer.apply(
            PlanRecompiled(continuation_plan, continuation_basis)
        )
        phase_trace.append(continued.phase)

        self.assertEqual(continued.phase, AgentPhase.EXECUTING)
        self.assertFalse(continued.compile_pending)
        self.assertEqual(continued.intent, first_intent)
        self.assertIs(continued.intent, first_intent)
        self.assertEqual(continued.plan, continuation_plan)
        self.assertEqual(len(compiler.continuation_calls), 1)
        self.assertEqual(len(planner.calls), 1)
        self.assertEqual(coordinator.run_once().outcome, CoordinatorOutcome.NO_WORK)
        self.assertEqual(len(planner.calls), 1)

        replan_basis = successor(
            continuation_basis,
            controller_version=5,
            basis_id="basis-obstacle-changed",
        )
        second_ticket = PlanningTicket(
            ticket_id="ticket-relevant-replan",
            cause=PlanningCause.REPLAN_REQUIRED,
            basis=replan_basis,
            created_at_ms=200,
            valid_until_ms=5_000,
        )
        replanning = reducer.apply(
            ReplanRequested(
                ticket=second_ticket,
                resulting_basis=replan_basis,
                reason="obstacle_geometry_changed",
            )
        )
        phase_trace.append(replanning.phase)
        self.assertFalse(replanning.compile_pending)
        self.assertEqual(replanning.planning_ticket, second_ticket)

        second_planning = coordinator.run_once()
        phase_trace.append(second_planning.state.phase)

        self.assertEqual(second_planning.outcome, CoordinatorOutcome.INTENT_ACCEPTED)
        self.assertEqual(len(planner.calls), 2)
        self.assertEqual(planner.calls[1].state.intent, first_intent)
        self.assertEqual(len(compiler.model_intent_calls), 2)
        self.assertEqual(len(compiler.continuation_calls), 1)
        self.assertEqual(second_planning.state.intent.intent_id, first_intent.intent_id)
        self.assertEqual(
            second_planning.state.intent.revision,
            first_intent.revision + 1,
        )
        self.assertEqual(coordinator.run_once().outcome, CoordinatorOutcome.NO_WORK)
        self.assertEqual(len(planner.calls), 2)

        stopping = reducer.apply(
            StopRequested(
                GoalTerminal(
                    outcome=GoalOutcome.CANCELLED,
                    reason="replay_complete",
                    completed_at_ms=1_100,
                )
            )
        )
        phase_trace.append(stopping.phase)
        terminal = reducer.apply(StopVerified(verified_at_ms=1_101))
        phase_trace.append(terminal.phase)

        self.assertEqual(
            tuple(AgentPhase),
            (
                AgentPhase.IDLE,
                AgentPhase.PLANNING,
                AgentPhase.EXECUTING,
                AgentPhase.STOPPING,
                AgentPhase.TERMINAL,
            ),
        )
        self.assertEqual(
            phase_trace,
            [
                AgentPhase.IDLE,
                AgentPhase.PLANNING,
                AgentPhase.EXECUTING,
                AgentPhase.EXECUTING,
                AgentPhase.PLANNING,
                AgentPhase.EXECUTING,
                AgentPhase.PLANNING,
                AgentPhase.EXECUTING,
                AgentPhase.STOPPING,
                AgentPhase.TERMINAL,
            ],
        )
        self.assertEqual(set(phase_trace), set(AgentPhase))
        self.assertEqual(len(planner.calls), 2)


if __name__ == "__main__":
    unittest.main()
