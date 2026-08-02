from dataclasses import replace
import unittest

from robot_agent.physical_agent_state import (
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
    IntentPrepared,
    IntentPolicy,
    MAX_PLANNING_TICKET_TTL_MS,
    MAX_STEP_COMMAND_SETTLE_MS,
    MAX_STEP_COMMAND_START_TTL_MS,
    NavigationBasis,
    NavigationBasisUpdated,
    PhysicalAgentState,
    PhysicalAgentStateError,
    PhysicalAgentStateReducer,
    PlanBinding,
    PlanRecompiled,
    PlanStepKey,
    PlanningAbortRequested,
    PlanningCause,
    PlanningHeld,
    PlanningRequested,
    PlanningTicket,
    PlanningTicketConsumed,
    PlanningTicketExpired,
    PreparedIntentAccepted,
    PreparedIntentExpired,
    PreparedIntentPlan,
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
    reduce_physical_agent_state,
)


def controller(instance_id="instance-1"):
    return ControllerKey(
        robot_id="robot-1",
        controller_id="drive-1",
        controller_instance_id=instance_id,
    )


def basis(
    *,
    key=None,
    epoch=1,
    controller_version=1,
    world_version=1,
    basis_id="nav-basis-1",
    generation="world-1",
    frame="robot-local-1",
    calibration="calibration-a",
):
    return NavigationBasis(
        controller_key=key or controller(),
        goal_epoch=epoch,
        controller_state_version=controller_version,
        world_generation_id=generation,
        world_model_version=world_version,
        navigation_basis_id=basis_id,
        frame_id=frame,
        calibration_fingerprint=calibration,
    )


def goal(epoch=1, activated_at_ms=100):
    return GoalAssignment(
        goal_id="goal-{}".format(epoch),
        goal_epoch=epoch,
        objective="Move forward while handling obstacles",
        source="USER",
        locale="sv",
        activated_at_ms=activated_at_ms,
    )


def ticket(
    nav_basis,
    *,
    ticket_id="ticket-1",
    cause=PlanningCause.NEW_GOAL,
    created_at_ms=101,
    valid_until_ms=None,
):
    return PlanningTicket(
        ticket_id=ticket_id,
        cause=cause,
        basis=nav_basis,
        created_at_ms=created_at_ms,
        valid_until_ms=(
            created_at_ms + 5_000
            if valid_until_ms is None
            else valid_until_ms
        ),
    )


def active_intent(
    nav_basis,
    assigned_goal,
    *,
    intent_id="intent-1",
    revision=1,
    payload=None,
    policy=None,
    accepted_at_ms=103,
):
    return ActiveIntent(
        intent_id=intent_id,
        revision=revision,
        goal_id=assigned_goal.goal_id,
        goal_epoch=assigned_goal.goal_epoch,
        payload=payload or FollowDirectionIntent(),
        accepted_basis=nav_basis,
        accepted_at_ms=accepted_at_ms,
        policy=policy or IntentPolicy(),
    )


def plan_binding(key, assigned_goal, intent, nav_basis):
    return PlanBinding(
        controller_key=key,
        goal_id=assigned_goal.goal_id,
        goal_epoch=assigned_goal.goal_epoch,
        intent_id=intent.intent_id,
        intent_revision=intent.revision,
        frame_id=nav_basis.frame_id,
        world_generation_id=nav_basis.world_generation_id,
        calibration_fingerprint=nav_basis.calibration_fingerprint,
        based_on_navigation_basis_id=nav_basis.navigation_basis_id,
        target_geometry_signatures=(),
    )


def execution_plan(
    key,
    assigned_goal,
    intent,
    nav_basis,
    *,
    plan_id="plan-1",
    revision=1,
    cursor=0,
    steps=None,
):
    return ExecutionPlan(
        plan_id=plan_id,
        revision=revision,
        binding=plan_binding(key, assigned_goal, intent, nav_basis),
        steps=steps
        or (
            WaypointStep("waypoint-1", 100, 0, 0, 35, 20_000),
            WaypointStep("waypoint-2", 200, 0, 0, 35, 20_000),
        ),
        cursor=cursor,
        created_at_ms=104,
    )


def activated_state(initial=None):
    initial = initial or PhysicalAgentState(controller())
    nav_basis = basis(key=initial.controller_key, epoch=initial.goal_epoch + 1)
    assigned_goal = goal(nav_basis.goal_epoch)
    planning_ticket = ticket(nav_basis)
    state = reduce_physical_agent_state(
        initial,
        GoalActivated(assigned_goal, nav_basis, planning_ticket),
    )
    return state, assigned_goal, nav_basis, planning_ticket


def consumed_planning_state(initial=None):
    state, assigned_goal, nav_basis, planning_ticket = activated_state(initial)
    state = reduce_physical_agent_state(
        state,
        PlanningTicketConsumed(planning_ticket, 102),
    )
    return state, assigned_goal, nav_basis, planning_ticket


def executing_state(initial=None):
    state, assigned_goal, nav_basis, planning_ticket = consumed_planning_state(
        initial
    )
    intent = active_intent(nav_basis, assigned_goal)
    active_plan = execution_plan(
        state.controller_key,
        assigned_goal,
        intent,
        nav_basis,
    )
    state = accept_intent_plan(state, intent, active_plan)
    return state, assigned_goal, nav_basis, intent, active_plan


def prepared_for_state(
    state,
    intent,
    plan,
    *,
    proposal_id="proposal-test",
    compilation_basis=None,
):
    planning_ticket = state.planning_ticket
    prepared_at_ms = max(
        planning_ticket.consumed_at_ms,
        intent.accepted_at_ms,
        plan.created_at_ms,
    )
    return PreparedIntentPlan(
        ticket=planning_ticket,
        proposal_id=proposal_id,
        compilation_basis=compilation_basis or state.basis,
        intent=intent,
        plan=plan,
        prepared_at_ms=prepared_at_ms,
        valid_until_ms=min(
            planning_ticket.valid_until_ms,
            prepared_at_ms + 1_000,
        ),
    )


def accept_intent_plan(state, intent, plan, *, proposal_id="proposal-test"):
    prepared = prepared_for_state(
        state,
        intent,
        plan,
        proposal_id=proposal_id,
    )
    state = reduce_physical_agent_state(state, IntentPrepared(prepared))
    return reduce_physical_agent_state(
        state,
        PreparedIntentAccepted(prepared, prepared.prepared_at_ms),
    )


def step_authorization(
    state,
    *,
    issued_at_ms=None,
    valid_until_ms=None,
    **changes
):
    sequence = state.last_host_dispatch_sequence + 1
    issued_at_ms = (
        102 + sequence * 3
        if issued_at_ms is None
        else issued_at_ms
    )
    value = StepCommandAuthorization(
        action_id="action-{}".format(sequence),
        command_id="command-{}".format(sequence),
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
        command_fingerprint="sha256:command-{}".format(sequence),
        issued_at_ms=issued_at_ms,
        valid_until_ms=(
            issued_at_ms + 1_000
            if valid_until_ms is None
            else valid_until_ms
        ),
    )
    return replace(value, **changes) if changes else value


def dispatch_active_step(
    state,
    *,
    authorization=None,
    dispatched_at_ms=None,
    settle_by_host_ms=None,
):
    authorization = authorization or step_authorization(state)
    dispatched_at_ms = (
        authorization.issued_at_ms + 1
        if dispatched_at_ms is None
        else dispatched_at_ms
    )
    settle_by_host_ms = (
        dispatched_at_ms + 30_000
        if settle_by_host_ms is None
        else settle_by_host_ms
    )
    authorized = reduce_physical_agent_state(
        state,
        StepCommandAuthorized(authorization),
    )
    dispatched = reduce_physical_agent_state(
        authorized,
        StepCommandDispatched(
            authorization,
            dispatched_at_ms,
            settle_by_host_ms,
        ),
    )
    return dispatched, authorization


def command_receipt(
    authorization,
    resulting_basis,
    *,
    outcome=ReceiptOutcome.COMPLETED,
    received_at_host_ms=None,
    stop_confirmed=True,
    code="command_settled",
    **changes
):
    value = ControllerCommandReceipt(
        outcome=outcome,
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
        received_at_host_ms=(
            authorization.issued_at_ms + 2
            if received_at_host_ms is None
            else received_at_host_ms
        ),
        stop_confirmed=stop_confirmed,
        code=code,
    )
    return replace(value, **changes) if changes else value


def settle_active_step(
    state,
    resulting_basis,
    *,
    disposition=StepDisposition.COMPLETE,
    outcome=ReceiptOutcome.COMPLETED,
    replan_ticket=None,
):
    dispatched, authorization = dispatch_active_step(state)
    receipt = command_receipt(
        authorization,
        resulting_basis,
        outcome=outcome,
    )
    settled = reduce_physical_agent_state(
        dispatched,
        StepCommandSettled(
            receipt=receipt,
            resulting_basis=resulting_basis,
            disposition=disposition,
            replan_ticket=replan_ticket,
        ),
    )
    return settled, authorization, receipt, dispatched


def stopping_state():
    state, assigned_goal, nav_basis, intent, active_plan = executing_state()
    terminal = GoalTerminal(GoalOutcome.CANCELLED, "operator_stop", 110)
    state = reduce_physical_agent_state(state, StopRequested(terminal))
    return state, assigned_goal, nav_basis, terminal


def terminal_state():
    state, assigned_goal, nav_basis, terminal = stopping_state()
    state = reduce_physical_agent_state(state, StopVerified(111))
    return state, assigned_goal, nav_basis, terminal


class PhysicalAgentValueTests(unittest.TestCase):
    def test_intent_and_step_union_is_typed_and_small(self):
        scan = ScanTargetIntent("hazard-1", "scan-profile-1")
        detour = DetourTargetIntent(
            "hazard-1",
            DetourSide.LEFT_OF_GOAL,
        )
        sensor = SensorStep(
            "scan-1",
            "SCAN_FRONT_ARC",
            "hazard-1",
            "scan-profile-1",
        )
        primitive = PrimitiveStep("compat-1", "REVERSE")

        self.assertEqual(scan.target_hypothesis_id, "hazard-1")
        self.assertEqual(detour.detour_side, DetourSide.LEFT_OF_GOAL)
        self.assertEqual(sensor.operation, "SCAN_FRONT_ARC")
        self.assertEqual(primitive.action, "REVERSE")

    def test_invalid_payloads_and_steps_are_rejected(self):
        with self.assertRaises(PhysicalAgentStateError):
            ScanTargetIntent("hazard-1", "")
        with self.assertRaises(PhysicalAgentStateError):
            DetourTargetIntent("hazard-1", "LEFT_OF_GOAL")
        with self.assertRaises(PhysicalAgentStateError):
            SensorStep("observe-1", "OBSERVE", "unexpected", None)
        with self.assertRaises(PhysicalAgentStateError):
            PrimitiveStep("compat-1", "FULL_SPEED_FOREVER")

    def test_planning_ticket_requires_a_strict_finite_ttl(self):
        nav_basis = basis()
        valid = ticket(
            nav_basis,
            created_at_ms=100,
            valid_until_ms=100 + MAX_PLANNING_TICKET_TTL_MS,
        )

        self.assertEqual(
            valid.valid_until_ms - valid.created_at_ms,
            MAX_PLANNING_TICKET_TTL_MS,
        )
        for valid_until_ms in (
            100,
            100 + MAX_PLANNING_TICKET_TTL_MS + 1,
        ):
            with self.subTest(valid_until_ms=valid_until_ms):
                with self.assertRaises(PhysicalAgentStateError) as caught:
                    ticket(
                        nav_basis,
                        created_at_ms=100,
                        valid_until_ms=valid_until_ms,
                    )
                self.assertEqual(
                    caught.exception.code,
                    "invalid_planning_ticket_ttl",
                )

        self.assertTrue(valid.consume(valid.valid_until_ms - 1).consumed)
        with self.assertRaises(PhysicalAgentStateError) as expired:
            valid.consume(valid.valid_until_ms)
        self.assertEqual(expired.exception.code, "planning_ticket_expired")

    def test_step_command_start_window_is_finite_and_host_timed(self):
        state, _goal, _basis, _intent, _plan = executing_state()
        issued_at_ms = 1_000
        valid = step_authorization(
            state,
            issued_at_ms=issued_at_ms,
            valid_until_ms=(
                issued_at_ms + MAX_STEP_COMMAND_START_TTL_MS
            ),
        )

        self.assertEqual(
            valid.valid_until_ms - valid.issued_at_ms,
            MAX_STEP_COMMAND_START_TTL_MS,
        )
        for valid_until_ms in (
            issued_at_ms,
            issued_at_ms + MAX_STEP_COMMAND_START_TTL_MS + 1,
        ):
            with self.subTest(valid_until_ms=valid_until_ms):
                with self.assertRaises(PhysicalAgentStateError) as caught:
                    step_authorization(
                        state,
                        issued_at_ms=issued_at_ms,
                        valid_until_ms=valid_until_ms,
                    )
                self.assertEqual(
                    caught.exception.code,
                    "invalid_step_command_ttl",
                )

    def test_step_command_settlement_window_has_a_generous_finite_ceiling(self):
        state, _goal, _basis, _intent, _plan = executing_state()
        authorization = step_authorization(state)
        authorized = reduce_physical_agent_state(
            state,
            StepCommandAuthorized(authorization),
        )
        dispatched_at_ms = authorization.issued_at_ms + 1
        valid = reduce_physical_agent_state(
            authorized,
            StepCommandDispatched(
                authorization,
                dispatched_at_ms,
                dispatched_at_ms + MAX_STEP_COMMAND_SETTLE_MS,
            ),
        )

        self.assertEqual(
            valid.active_dispatch.settle_by_host_ms - dispatched_at_ms,
            MAX_STEP_COMMAND_SETTLE_MS,
        )
        with self.assertRaises(PhysicalAgentStateError) as unbounded:
            reduce_physical_agent_state(
                authorized,
                StepCommandDispatched(
                    authorization,
                    dispatched_at_ms,
                    dispatched_at_ms + MAX_STEP_COMMAND_SETTLE_MS + 1,
                ),
            )
        self.assertEqual(
            unbounded.exception.code,
            "invalid_step_command_settle_deadline",
        )

    def test_plan_binding_requires_sorted_unique_target_signatures(self):
        state, assigned_goal, nav_basis, _planning_ticket = activated_state()
        intent = active_intent(nav_basis, assigned_goal)
        arguments = dict(
            controller_key=state.controller_key,
            goal_id=assigned_goal.goal_id,
            goal_epoch=assigned_goal.goal_epoch,
            intent_id=intent.intent_id,
            intent_revision=intent.revision,
            frame_id=nav_basis.frame_id,
            world_generation_id=nav_basis.world_generation_id,
            calibration_fingerprint=nav_basis.calibration_fingerprint,
            based_on_navigation_basis_id=nav_basis.navigation_basis_id,
        )

        with self.assertRaises(PhysicalAgentStateError):
            PlanBinding(
                target_geometry_signatures=(
                    ("target-b", "signature-b"),
                    ("target-a", "signature-a"),
                ),
                **arguments
            )
        with self.assertRaises(PhysicalAgentStateError):
            PlanBinding(
                target_geometry_signatures=(
                    ("target-a", "signature-a"),
                    ("target-a", "signature-a"),
                ),
                **arguments
            )

    def test_all_five_phase_shapes_are_strict(self):
        idle = PhysicalAgentState(controller())
        planning, assigned_goal, nav_basis, planning_ticket = activated_state()
        executing, _goal, _basis, intent, active_plan = executing_state()
        stopping, _goal, _basis, terminal = stopping_state()
        terminal_state_value, _goal, _basis, _terminal = terminal_state()

        self.assertEqual(
            tuple(
                value.phase
                for value in (
                    idle,
                    planning,
                    executing,
                    stopping,
                    terminal_state_value,
                )
            ),
            tuple(AgentPhase),
        )
        invalid = (
            dict(goal=assigned_goal),
            dict(
                phase=AgentPhase.PLANNING,
                goal_epoch=1,
                goal=assigned_goal,
                basis=nav_basis,
                plan=active_plan,
                planning_ticket=planning_ticket,
            ),
            dict(
                phase=AgentPhase.EXECUTING,
                goal_epoch=1,
                goal=assigned_goal,
                basis=nav_basis,
                intent=intent,
            ),
            dict(
                phase=AgentPhase.STOPPING,
                goal_epoch=1,
                goal=assigned_goal,
                basis=nav_basis,
            ),
            dict(
                phase=AgentPhase.TERMINAL,
                goal_epoch=1,
                goal=assigned_goal,
                basis=nav_basis,
                intent=intent,
                terminal=terminal,
            ),
        )
        for arguments in invalid:
            with self.subTest(arguments=arguments):
                with self.assertRaises(PhysicalAgentStateError):
                    PhysicalAgentState(controller_key=controller(), **arguments)


class PhysicalAgentTransitionTests(unittest.TestCase):
    def test_legacy_direct_intent_acceptance_is_not_a_live_transition(self):
        state, assigned_goal, nav_basis, planning_ticket = (
            consumed_planning_state()
        )
        intent = active_intent(nav_basis, assigned_goal)
        active_plan = execution_plan(
            state.controller_key,
            assigned_goal,
            intent,
            nav_basis,
        )

        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                state,
                IntentAccepted(
                    planning_ticket.ticket_id,
                    nav_basis,
                    intent,
                    active_plan,
                ),
            )

        self.assertEqual(caught.exception.code, "unsupported_state_event")

    def test_complete_legal_lifecycle_is_monotonic(self):
        reducer = PhysicalAgentStateReducer(PhysicalAgentState(controller()))
        nav_basis = basis()
        assigned_goal = goal()
        planning_ticket = ticket(nav_basis)
        versions = [reducer.snapshot().agent_state_version]

        planning = reducer.apply(
            GoalActivated(assigned_goal, nav_basis, planning_ticket)
        )
        versions.append(planning.agent_state_version)
        claimed = reducer.apply(
            PlanningTicketConsumed(planning_ticket, 102)
        )
        versions.append(claimed.agent_state_version)
        intent = active_intent(nav_basis, assigned_goal)
        active_plan = execution_plan(
            controller(), assigned_goal, intent, nav_basis
        )
        prepared = prepared_for_state(
            reducer.snapshot(),
            intent,
            active_plan,
            proposal_id="proposal-lifecycle",
        )
        prepared_state = reducer.apply(IntentPrepared(prepared))
        versions.append(prepared_state.agent_state_version)
        executing = reducer.apply(
            PreparedIntentAccepted(prepared, prepared.prepared_at_ms)
        )
        versions.append(executing.agent_state_version)
        step_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="nav-step-1",
        )
        first_authorization = step_authorization(reducer.snapshot())
        authorized = reducer.apply(
            StepCommandAuthorized(first_authorization)
        )
        versions.append(authorized.agent_state_version)
        dispatched = reducer.apply(
            StepCommandDispatched(
                first_authorization,
                first_authorization.issued_at_ms + 1,
                first_authorization.issued_at_ms + 30_000,
            )
        )
        versions.append(dispatched.agent_state_version)
        advanced = reducer.apply(
            StepCommandSettled(
                command_receipt(first_authorization, step_basis),
                step_basis,
                StepDisposition.COMPLETE,
            )
        )
        versions.append(advanced.agent_state_version)
        final_basis = replace(
            step_basis,
            controller_state_version=3,
            navigation_basis_id="nav-plan-finished",
        )
        final_authorization = step_authorization(reducer.snapshot())
        authorized = reducer.apply(
            StepCommandAuthorized(final_authorization)
        )
        versions.append(authorized.agent_state_version)
        dispatched = reducer.apply(
            StepCommandDispatched(
                final_authorization,
                final_authorization.issued_at_ms + 1,
                final_authorization.issued_at_ms + 30_000,
            )
        )
        versions.append(dispatched.agent_state_version)
        finished = reducer.apply(
            StepCommandSettled(
                command_receipt(final_authorization, final_basis),
                final_basis,
                StepDisposition.COMPLETE,
            )
        )
        versions.append(finished.agent_state_version)
        stopping = reducer.apply(
            GoalCompletionRequested(
                final_basis,
                GoalTerminal(GoalOutcome.SUCCEEDED, "goal_complete", 110),
            )
        )
        versions.append(stopping.agent_state_version)
        terminal = reducer.apply(StopVerified(111))
        versions.append(terminal.agent_state_version)
        idle = reducer.apply(TerminalCleared(112))
        versions.append(idle.agent_state_version)

        self.assertEqual(
            tuple(versions),
            tuple(range(versions[0], versions[0] + len(versions))),
        )
        self.assertEqual(planning.phase, AgentPhase.PLANNING)
        self.assertTrue(claimed.planning_ticket.consumed)
        self.assertEqual(executing.phase, AgentPhase.EXECUTING)
        self.assertEqual(advanced.plan.cursor, 1)
        self.assertEqual(finished.phase, AgentPhase.PLANNING)
        self.assertTrue(finished.compile_pending)
        self.assertIsNone(finished.plan)
        self.assertEqual(finished.intent_progress.completed_steps, 2)
        self.assertEqual(stopping.phase, AgentPhase.STOPPING)
        self.assertEqual(terminal.phase, AgentPhase.TERMINAL)
        self.assertEqual(idle.phase, AgentPhase.IDLE)
        self.assertEqual(idle.goal_epoch, 1)

    def test_hold_does_not_create_another_ticket(self):
        state, assigned_goal, nav_basis, planning_ticket = consumed_planning_state()
        held = reduce_physical_agent_state(
            state,
            PlanningHeld(state.planning_ticket, "proposal-hold"),
        )

        self.assertEqual(held.phase, AgentPhase.PLANNING)
        self.assertIsNone(held.planning_ticket)
        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                held,
                PlanningHeld(state.planning_ticket, "proposal-hold"),
            )
        self.assertEqual(caught.exception.code, "planning_ticket_mismatch")

        next_ticket = ticket(
            nav_basis,
            ticket_id="ticket-2",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=120,
        )
        requested = reduce_physical_agent_state(
            held,
            PlanningRequested(next_ticket),
        )
        self.assertEqual(requested.planning_ticket, next_ticket)

    def test_stale_hold_cannot_clear_reused_ticket_id_and_basis(self):
        state, _goal, nav_basis, _ticket = consumed_planning_state()
        first_consumed = state.planning_ticket
        state = reduce_physical_agent_state(
            state,
            PlanningHeld(first_consumed, "proposal-first"),
        )
        second = ticket(
            nav_basis,
            ticket_id=first_consumed.ticket_id,
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=200,
        )
        state = reduce_physical_agent_state(state, PlanningRequested(second))
        state = reduce_physical_agent_state(
            state,
            PlanningTicketConsumed(second, 201),
        )
        second_consumed = state.planning_ticket

        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                state,
                PlanningHeld(first_consumed, "proposal-first"),
            )

        self.assertEqual(caught.exception.code, "planning_ticket_mismatch")
        self.assertEqual(state.planning_ticket, second_consumed)

    def test_planning_is_orthogonal_to_the_authoritative_execution_plan(self):
        state, _goal, nav_basis, intent, active_plan = executing_state()
        parallel_ticket = ticket(
            nav_basis,
            ticket_id="ticket-parallel",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=120,
        )

        requested = reduce_physical_agent_state(
            state,
            PlanningRequested(parallel_ticket),
        )
        with self.assertRaises(PhysicalAgentStateError) as duplicate:
            reduce_physical_agent_state(
                requested,
                PlanningRequested(
                    ticket(
                        nav_basis,
                        ticket_id="ticket-parallel-duplicate",
                        cause=PlanningCause.UNCERTAINTY,
                        created_at_ms=121,
                    )
                ),
            )
        self.assertEqual(
            duplicate.exception.code,
            "planning_ticket_already_active",
        )
        consumed = reduce_physical_agent_state(
            requested,
            PlanningTicketConsumed(parallel_ticket, 121),
        )
        held = reduce_physical_agent_state(
            consumed,
            PlanningHeld(
                consumed.planning_ticket,
                "proposal-parallel-hold",
            ),
        )

        for current in (requested, consumed, held):
            with self.subTest(version=current.agent_state_version):
                self.assertEqual(current.phase, AgentPhase.EXECUTING)
                self.assertEqual(current.intent, intent)
                self.assertEqual(current.plan, active_plan)
        self.assertEqual(requested.planning_ticket, parallel_ticket)
        self.assertTrue(consumed.planning_ticket.consumed)
        self.assertIsNone(held.planning_ticket)

    def test_parallel_intent_acceptance_atomically_replaces_the_plan(self):
        state, assigned_goal, nav_basis, old_intent, old_plan = executing_state()
        parallel_ticket = ticket(
            nav_basis,
            ticket_id="ticket-parallel-replacement",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=120,
        )
        pending = reduce_physical_agent_state(
            state,
            PlanningRequested(parallel_ticket),
        )
        pending = reduce_physical_agent_state(
            pending,
            PlanningTicketConsumed(parallel_ticket, 121),
        )
        replacement_intent = active_intent(
            nav_basis,
            assigned_goal,
            intent_id="intent-2",
            revision=1,
            payload=DetourTargetIntent(
                "hazard-1",
                DetourSide.LEFT_OF_GOAL,
            ),
            accepted_at_ms=122,
        )
        replacement_plan = execution_plan(
            state.controller_key,
            assigned_goal,
            replacement_intent,
            nav_basis,
            plan_id="plan-2",
            revision=2,
        )

        replaced = accept_intent_plan(
            pending,
            replacement_intent,
            replacement_plan,
            proposal_id="proposal-replacement",
        )

        self.assertEqual(pending.phase, AgentPhase.EXECUTING)
        self.assertEqual(pending.intent, old_intent)
        self.assertEqual(pending.plan, old_plan)
        self.assertEqual(replaced.phase, AgentPhase.EXECUTING)
        self.assertEqual(replaced.intent, replacement_intent)
        self.assertEqual(replaced.plan, replacement_plan)
        self.assertEqual(replaced.plan_revision, 2)
        self.assertIsNone(replaced.planning_ticket)

    def test_plan_can_recompile_without_entering_planning(self):
        state, assigned_goal, nav_basis, intent, old_plan = executing_state()
        next_basis = replace(
            nav_basis,
            controller_state_version=2,
            world_model_version=2,
            navigation_basis_id="nav-basis-2",
        )
        new_plan = execution_plan(
            state.controller_key,
            assigned_goal,
            intent,
            next_basis,
            plan_id="plan-2",
            revision=2,
        )

        recompiled = reduce_physical_agent_state(
            state,
            PlanRecompiled(new_plan, next_basis),
        )

        self.assertEqual(recompiled.phase, AgentPhase.EXECUTING)
        self.assertEqual(recompiled.intent, intent)
        self.assertEqual(recompiled.plan.plan_id, "plan-2")
        self.assertEqual(recompiled.plan_revision, 2)
        self.assertNotEqual(recompiled.plan, old_plan)

    def test_replan_preserves_intent_and_accepts_one_revision(self):
        state, assigned_goal, nav_basis, intent, _old_plan = executing_state()
        next_basis = replace(
            nav_basis,
            controller_state_version=2,
            world_model_version=2,
            navigation_basis_id="nav-basis-2",
        )
        next_ticket = ticket(
            next_basis,
            ticket_id="ticket-2",
            cause=PlanningCause.REPLAN_REQUIRED,
            created_at_ms=120,
        )
        planning = reduce_physical_agent_state(
            state,
            ReplanRequested(next_ticket, next_basis, "geometry_changed"),
        )
        claimed = reduce_physical_agent_state(
            planning,
            PlanningTicketConsumed(next_ticket, 121),
        )
        revised = active_intent(
            next_basis,
            assigned_goal,
            intent_id=intent.intent_id,
            revision=2,
            payload=DetourTargetIntent(
                "hazard-1", DetourSide.RIGHT_OF_GOAL
            ),
            accepted_at_ms=122,
        )
        next_plan = execution_plan(
            state.controller_key,
            assigned_goal,
            revised,
            next_basis,
            plan_id="plan-2",
            revision=2,
        )
        executing = accept_intent_plan(
            claimed,
            revised,
            next_plan,
            proposal_id="proposal-replan",
        )

        self.assertEqual(planning.phase, AgentPhase.PLANNING)
        self.assertEqual(planning.intent, intent)
        self.assertEqual(executing.intent.revision, 2)
        self.assertEqual(executing.plan_revision, 2)
        self.assertEqual(executing.intent_progress.plan_attempts, 1)
        self.assertEqual(executing.intent_progress.completed_steps, 0)

    def test_stop_accepts_only_cancelled_or_failed(self):
        planning, _goal, _basis, _ticket = activated_state()
        cancelled = reduce_physical_agent_state(
            planning,
            StopRequested(
                GoalTerminal(GoalOutcome.CANCELLED, "operator_stop", 110)
            ),
        )
        self.assertEqual(cancelled.phase, AgentPhase.STOPPING)

        executing, _goal, _basis, _intent, _plan = executing_state()
        failed = reduce_physical_agent_state(
            executing,
            StopRequested(
                GoalTerminal(GoalOutcome.FAILED, "controller_fault", 110)
            ),
        )
        self.assertEqual(failed.phase, AgentPhase.STOPPING)

        with self.assertRaises(PhysicalAgentStateError):
            reduce_physical_agent_state(
                planning,
                StopRequested(
                    GoalTerminal(GoalOutcome.SUCCEEDED, "wrong_path", 110)
                ),
            )

    def test_ticket_bound_planning_abort_stops_only_for_current_consumed_ticket(self):
        state, _goal, nav_basis, _intent, _plan = executing_state()
        abort_ticket = ticket(
            nav_basis,
            ticket_id="ticket-abort",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=120,
        )
        pending = reduce_physical_agent_state(
            state,
            PlanningRequested(abort_ticket),
        )
        consumed = reduce_physical_agent_state(
            pending,
            PlanningTicketConsumed(abort_ticket, 121),
        )
        terminal = GoalTerminal(
            GoalOutcome.FAILED,
            "planner_found_no_route",
            122,
        )
        stopping = reduce_physical_agent_state(
            consumed,
            PlanningAbortRequested(
                consumed.planning_ticket,
                "proposal-abort",
                terminal,
            ),
        )

        self.assertEqual(stopping.phase, AgentPhase.STOPPING)
        self.assertEqual(stopping.terminal, terminal)
        self.assertIsNone(stopping.plan)
        self.assertIsNone(stopping.planning_ticket)

        with self.assertRaises(PhysicalAgentStateError) as unconsumed:
            reduce_physical_agent_state(
                pending,
                PlanningAbortRequested(
                    abort_ticket,
                    "proposal-abort",
                    terminal,
                ),
            )
        self.assertEqual(
            unconsumed.exception.code,
            "planning_ticket_mismatch",
        )

        with self.assertRaises(PhysicalAgentStateError) as succeeded:
            reduce_physical_agent_state(
                consumed,
                PlanningAbortRequested(
                    consumed.planning_ticket,
                    "proposal-abort",
                    GoalTerminal(
                        GoalOutcome.SUCCEEDED,
                        "not_an_abort",
                        122,
                    ),
                ),
            )
        self.assertEqual(
            succeeded.exception.code,
            "invalid_planning_abort_outcome",
        )

    def test_stale_planning_abort_cannot_stop_after_motion_supersedes_ticket(self):
        state, _goal, nav_basis, _intent, active_plan = executing_state()
        old_ticket = ticket(
            nav_basis,
            ticket_id="ticket-old-abort",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=120,
        )
        state = reduce_physical_agent_state(
            state,
            PlanningRequested(old_ticket),
        )
        state = reduce_physical_agent_state(
            state,
            PlanningTicketConsumed(old_ticket, 121),
        )
        consumed_old_ticket = state.planning_ticket
        changed_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="basis-after-motion",
        )
        superseded, _authorization, _receipt, _dispatched = (
            settle_active_step(state, changed_basis)
        )

        self.assertEqual(superseded.phase, AgentPhase.EXECUTING)
        self.assertEqual(superseded.plan.cursor, 1)
        self.assertIsNone(superseded.planning_ticket)
        with self.assertRaises(PhysicalAgentStateError) as stale:
            reduce_physical_agent_state(
                superseded,
                PlanningAbortRequested(
                    consumed_old_ticket,
                    "proposal-stale-abort",
                    GoalTerminal(
                        GoalOutcome.FAILED,
                        "stale_abort",
                        122,
                    ),
                ),
            )
        self.assertEqual(stale.exception.code, "planning_ticket_mismatch")
        self.assertEqual(superseded.phase, AgentPhase.EXECUTING)
        self.assertIsNotNone(superseded.plan)

    def test_new_goal_requires_strictly_newer_epoch(self):
        terminal, _goal, _basis, _terminal = terminal_state()
        same_basis = basis(epoch=1)
        same_goal = goal(epoch=1)

        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                terminal,
                GoalActivated(
                    same_goal,
                    same_basis,
                    ticket(same_basis),
                ),
            )
        self.assertEqual(caught.exception.code, "invalid_goal_activation")

        next_basis = basis(epoch=2, basis_id="nav-goal-2")
        next_goal = goal(epoch=2, activated_at_ms=200)
        next_state = reduce_physical_agent_state(
            terminal,
            GoalActivated(
                next_goal,
                next_basis,
                ticket(
                    next_basis,
                    ticket_id="ticket-goal-2",
                    created_at_ms=201,
                ),
            ),
        )
        self.assertEqual(next_state.goal_epoch, 2)
        self.assertEqual(next_state.phase, AgentPhase.PLANNING)


class PhysicalAgentTransitionMatrixTests(unittest.TestCase):
    def test_every_event_rejects_every_disallowed_phase(self):
        idle = PhysicalAgentState(controller())
        planning, assigned_goal, nav_basis, planning_ticket = activated_state()
        consumed, _goal, _basis, _ticket = consumed_planning_state()
        held = reduce_physical_agent_state(
            consumed,
            PlanningHeld(
                consumed.planning_ticket,
                "proposal-matrix-held",
            ),
        )
        executing, _goal, _basis, intent, active_plan = executing_state()
        stopping, _goal, _basis, pending_terminal = stopping_state()
        terminal, _goal, _basis, _terminal = terminal_state()
        states = {
            AgentPhase.IDLE: idle,
            AgentPhase.PLANNING: planning,
            AgentPhase.EXECUTING: executing,
            AgentPhase.STOPPING: stopping,
            AgentPhase.TERMINAL: terminal,
        }
        successor = replace(
            nav_basis,
            controller_state_version=2,
            world_model_version=2,
        )
        replan_ticket = ticket(
            successor,
            ticket_id="ticket-replan",
            cause=PlanningCause.REPLAN_REQUIRED,
            created_at_ms=120,
        )
        intent_plan = execution_plan(
            controller(), assigned_goal, intent, nav_basis
        )
        matrix_prepared = prepared_for_state(
            consumed,
            intent,
            intent_plan,
            proposal_id="proposal-matrix-prepared",
        )
        authorization = step_authorization(executing)
        receipt = command_receipt(authorization, successor)
        events = (
            (
                GoalActivated(assigned_goal, nav_basis, planning_ticket),
                {AgentPhase.IDLE, AgentPhase.TERMINAL},
            ),
            (
                PlanningTicketConsumed(planning_ticket, 102),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                PlanningTicketExpired(
                    planning_ticket,
                    planning_ticket.valid_until_ms,
                ),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                IntentPrepared(matrix_prepared),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                PreparedIntentAccepted(
                    matrix_prepared,
                    matrix_prepared.prepared_at_ms,
                ),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                PreparedIntentExpired(
                    matrix_prepared,
                    matrix_prepared.valid_until_ms,
                ),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                PlanningAbortRequested(
                    consumed.planning_ticket,
                    "proposal-matrix-abort",
                    pending_terminal,
                ),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                PlanningHeld(
                    consumed.planning_ticket,
                    "proposal-matrix-held",
                ),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                PlanningRequested(replan_ticket),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                NavigationBasisUpdated(successor),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                StepCommandAuthorized(authorization),
                {AgentPhase.EXECUTING},
            ),
            (
                StepCommandDispatched(
                    authorization,
                    authorization.issued_at_ms + 1,
                    authorization.issued_at_ms + 30_000,
                ),
                {AgentPhase.EXECUTING},
            ),
            (
                StepCommandRevoked(
                    authorization,
                    authorization.issued_at_ms + 1,
                ),
                {AgentPhase.EXECUTING},
            ),
            (
                StepCommandSettlementExpired(
                    authorization,
                    authorization.issued_at_ms + 30_000,
                ),
                {AgentPhase.EXECUTING},
            ),
            (
                StepCommandSettled(
                    receipt,
                    successor,
                    StepDisposition.COMPLETE,
                ),
                {AgentPhase.EXECUTING},
            ),
            (
                PlanRecompiled(active_plan, successor),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                ReplanRequested(
                    replan_ticket, successor, "representative"
                ),
                {AgentPhase.EXECUTING},
            ),
            (
                GoalCompletionRequested(
                    successor,
                    GoalTerminal(
                        GoalOutcome.SUCCEEDED, "goal_complete", 130
                    ),
                ),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                StopRequested(pending_terminal),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (StopVerified(140), {AgentPhase.STOPPING}),
            (TerminalCleared(140), {AgentPhase.TERMINAL}),
        )

        for event, allowed in events:
            for phase, state in states.items():
                if phase in allowed:
                    continue
                with self.subTest(event=type(event).__name__, phase=phase):
                    with self.assertRaises(PhysicalAgentStateError) as caught:
                        reduce_physical_agent_state(state, event)
                    self.assertEqual(
                        caught.exception.code,
                        "illegal_phase_transition",
                    )


class PhysicalAgentTicketAndCursorTests(unittest.TestCase):
    def test_ticket_is_consumed_at_most_once(self):
        nav_basis = basis()
        planning_ticket = ticket(nav_basis)
        consumed = planning_ticket.consume(102)

        self.assertTrue(consumed.consumed)
        with self.assertRaises(PhysicalAgentStateError) as caught:
            consumed.consume(103)
        self.assertEqual(
            caught.exception.code,
            "planning_ticket_already_consumed",
        )

        state, _goal, _basis, planning_ticket = activated_state()
        state = reduce_physical_agent_state(
            state,
            PlanningTicketConsumed(planning_ticket, 102),
        )
        with self.assertRaises(PhysicalAgentStateError):
            reduce_physical_agent_state(
                state,
                PlanningTicketConsumed(planning_ticket, 103),
            )

    def test_expired_unconsumed_ticket_can_be_dismissed_without_wedging(self):
        state, _goal, nav_basis, planning_ticket = activated_state()

        with self.assertRaises(PhysicalAgentStateError) as early:
            reduce_physical_agent_state(
                state,
                PlanningTicketExpired(
                    planning_ticket,
                    planning_ticket.valid_until_ms - 1,
                ),
            )
        self.assertEqual(early.exception.code, "planning_ticket_not_expired")

        dismissed = reduce_physical_agent_state(
            state,
            PlanningTicketExpired(
                planning_ticket,
                planning_ticket.valid_until_ms,
            ),
        )
        replacement = ticket(
            nav_basis,
            ticket_id="ticket-after-expiry",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=planning_ticket.valid_until_ms,
        )
        resumed = reduce_physical_agent_state(
            dismissed,
            PlanningRequested(replacement),
        )

        self.assertEqual(dismissed.phase, AgentPhase.PLANNING)
        self.assertIsNone(dismissed.planning_ticket)
        self.assertEqual(resumed.planning_ticket, replacement)

    def test_stale_ticket_events_cannot_target_reused_id_and_basis(self):
        state, _goal, nav_basis, first = activated_state()
        state = reduce_physical_agent_state(
            state,
            PlanningTicketExpired(first, first.valid_until_ms),
        )
        second = ticket(
            nav_basis,
            ticket_id=first.ticket_id,
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=first.valid_until_ms,
        )
        state = reduce_physical_agent_state(state, PlanningRequested(second))

        for event in (
            PlanningTicketConsumed(first, first.created_at_ms + 1),
            PlanningTicketExpired(first, second.valid_until_ms),
        ):
            with self.subTest(event=type(event).__name__):
                with self.assertRaises(PhysicalAgentStateError) as caught:
                    reduce_physical_agent_state(state, event)
                self.assertEqual(
                    caught.exception.code,
                    "planning_ticket_mismatch",
                )
                self.assertEqual(state.planning_ticket, second)

    def test_expired_parallel_ticket_is_cleared_without_stopping_plan(self):
        state, _goal, nav_basis, intent, active_plan = executing_state()
        parallel_ticket = ticket(
            nav_basis,
            ticket_id="ticket-expiring-parallel",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=120,
            valid_until_ms=130,
        )
        pending = reduce_physical_agent_state(
            state,
            PlanningRequested(parallel_ticket),
        )
        dismissed = reduce_physical_agent_state(
            pending,
            PlanningTicketExpired(parallel_ticket, 130),
        )

        self.assertEqual(dismissed.phase, AgentPhase.EXECUTING)
        self.assertEqual(dismissed.intent, intent)
        self.assertEqual(dismissed.plan, active_plan)
        self.assertIsNone(dismissed.planning_ticket)

    def test_expired_consumed_ticket_can_be_recovered_after_planner_crash(self):
        state, _goal, nav_basis, planning_ticket = activated_state()
        consumed = reduce_physical_agent_state(
            state,
            PlanningTicketConsumed(
                planning_ticket,
                planning_ticket.created_at_ms + 1,
            ),
        )

        recovered = reduce_physical_agent_state(
            consumed,
            PlanningTicketExpired(
                consumed.planning_ticket,
                planning_ticket.valid_until_ms,
            ),
        )

        self.assertEqual(recovered.phase, AgentPhase.PLANNING)
        self.assertIsNone(recovered.planning_ticket)

    def test_reducer_rejects_ticket_consumption_at_expiry(self):
        nav_basis = basis()
        expiring = ticket(
            nav_basis,
            created_at_ms=101,
            valid_until_ms=110,
        )
        state = reduce_physical_agent_state(
            PhysicalAgentState(controller()),
            GoalActivated(goal(), nav_basis, expiring),
        )

        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                state,
                PlanningTicketConsumed(expiring, 110),
            )
        self.assertEqual(caught.exception.code, "planning_ticket_expired")

    def test_only_one_ticket_can_be_outstanding(self):
        state, _goal, nav_basis, _ticket = activated_state()
        second = ticket(
            nav_basis,
            ticket_id="ticket-2",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=120,
        )
        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(state, PlanningRequested(second))
        self.assertEqual(
            caught.exception.code,
            "planning_ticket_already_active",
        )

    def test_settlement_preserves_only_decision_equivalent_parallel_ticket(self):
        state, _goal, nav_basis, _intent, _active_plan = executing_state()
        parallel_ticket = ticket(
            nav_basis,
            ticket_id="ticket-during-step",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=120,
        )
        pending = reduce_physical_agent_state(
            state,
            PlanningRequested(parallel_ticket),
        )
        equivalent_basis = replace(
            nav_basis,
            controller_state_version=2,
            world_model_version=2,
        )
        equivalent, _authorization, _receipt, _dispatched = (
            settle_active_step(pending, equivalent_basis)
        )
        changed_basis = replace(
            nav_basis,
            controller_state_version=2,
            world_model_version=2,
            navigation_basis_id="basis-decision-changed",
        )
        changed, _authorization, _receipt, _dispatched = (
            settle_active_step(pending, changed_basis)
        )

        self.assertEqual(equivalent.phase, AgentPhase.EXECUTING)
        self.assertEqual(equivalent.plan.cursor, 1)
        self.assertEqual(equivalent.planning_ticket, parallel_ticket)
        self.assertEqual(changed.phase, AgentPhase.EXECUTING)
        self.assertEqual(changed.plan.cursor, 1)
        self.assertIsNone(changed.planning_ticket)

    def test_completed_receipts_advance_then_finish_the_plan_exactly_once(self):
        state, _goal, nav_basis, _intent, _active_plan = executing_state()
        next_basis = replace(nav_basis, controller_state_version=2)
        advanced, _authorization, receipt, _dispatched = settle_active_step(
            state,
            next_basis,
        )

        self.assertEqual(advanced.plan.cursor, 1)
        self.assertEqual(advanced.plan.active_step.step_id, "waypoint-2")
        self.assertEqual(advanced.intent_progress.completed_steps, 1)
        self.assertIsNone(advanced.active_dispatch)
        with self.assertRaises(PhysicalAgentStateError) as duplicate:
            reduce_physical_agent_state(
                advanced,
                StepCommandSettled(
                    receipt,
                    next_basis,
                    StepDisposition.COMPLETE,
                ),
            )
        self.assertEqual(duplicate.exception.code, "active_dispatch_mismatch")

        final_basis = replace(next_basis, controller_state_version=3)
        finished, _authorization, _receipt, _dispatched = settle_active_step(
            advanced,
            final_basis,
        )

        self.assertEqual(finished.phase, AgentPhase.PLANNING)
        self.assertTrue(finished.compile_pending)
        self.assertIsNone(finished.plan)
        self.assertIsNone(finished.active_dispatch)
        self.assertIsNone(finished.planning_ticket)
        self.assertEqual(finished.intent, state.intent)
        self.assertEqual(finished.intent_progress.completed_steps, 2)

    def test_finished_plan_recompiles_same_intent_without_model_ticket(self):
        state, assigned_goal, nav_basis, intent, _active_plan = executing_state()
        step_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="nav-basis-step",
        )
        state, _authorization, _receipt, _dispatched = settle_active_step(
            state,
            step_basis,
        )
        finished_basis = replace(
            step_basis,
            controller_state_version=3,
            navigation_basis_id="nav-basis-plan-finished",
        )
        planning, _authorization, _receipt, _dispatched = settle_active_step(
            state,
            finished_basis,
        )
        self.assertEqual(planning.phase, AgentPhase.PLANNING)
        self.assertTrue(planning.compile_pending)
        self.assertIsNone(planning.planning_ticket)

        continuation_basis = replace(
            finished_basis,
            controller_state_version=4,
            navigation_basis_id="nav-basis-continuation",
        )
        continuation = execution_plan(
            state.controller_key,
            assigned_goal,
            intent,
            continuation_basis,
            plan_id="plan-2",
            revision=2,
        )
        executing = reduce_physical_agent_state(
            planning,
            PlanRecompiled(continuation, continuation_basis),
        )

        self.assertEqual(executing.phase, AgentPhase.EXECUTING)
        self.assertFalse(executing.compile_pending)
        self.assertEqual(executing.intent, intent)
        self.assertEqual(executing.plan, continuation)
        self.assertIsNone(executing.planning_ticket)
        self.assertEqual(executing.intent_progress.plan_attempts, 2)
        self.assertEqual(executing.intent_progress.completed_steps, 2)
        self.assertEqual(
            executing.intent_progress.consecutive_no_progress_plans,
            0,
        )

    def test_hold_cannot_enter_ticket_free_deterministic_compile_path(self):
        state, assigned_goal, nav_basis, intent, _active_plan = executing_state()
        replan_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="nav-basis-replan",
        )
        replan_ticket = ticket(
            replan_basis,
            ticket_id="ticket-held-replan",
            cause=PlanningCause.REPLAN_REQUIRED,
            created_at_ms=120,
        )
        planning = reduce_physical_agent_state(
            state,
            ReplanRequested(replan_ticket, replan_basis, "blocked"),
        )
        planning = reduce_physical_agent_state(
            planning,
            PlanningTicketConsumed(replan_ticket, 121),
        )
        held = reduce_physical_agent_state(
            planning,
            PlanningHeld(
                planning.planning_ticket,
                "proposal-recompile-hold",
            ),
        )
        candidate = execution_plan(
            state.controller_key,
            assigned_goal,
            intent,
            replan_basis,
            plan_id="plan-2",
            revision=2,
        )

        self.assertFalse(held.compile_pending)
        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                held,
                PlanRecompiled(candidate, replan_basis),
            )
        self.assertEqual(
            caught.exception.code,
            "deterministic_compile_not_pending",
        )

    def test_persistent_intent_has_finite_no_progress_recompile_budget(self):
        state, assigned_goal, nav_basis, planning_ticket = consumed_planning_state()
        intent = active_intent(
            nav_basis,
            assigned_goal,
            policy=IntentPolicy(
                max_plan_attempts=4,
                max_consecutive_no_progress_plans=1,
            ),
        )
        first_plan = execution_plan(
            state.controller_key, assigned_goal, intent, nav_basis
        )
        state = accept_intent_plan(
            state,
            intent,
            first_plan,
            proposal_id="proposal-budget",
        )
        second_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="nav-basis-2",
        )
        second_plan = execution_plan(
            state.controller_key,
            assigned_goal,
            intent,
            second_basis,
            plan_id="plan-2",
            revision=2,
        )
        state = reduce_physical_agent_state(
            state, PlanRecompiled(second_plan, second_basis)
        )
        self.assertEqual(
            state.intent_progress.consecutive_no_progress_plans,
            1,
        )

        third_basis = replace(
            second_basis,
            controller_state_version=3,
            navigation_basis_id="nav-basis-3",
        )
        third_plan = execution_plan(
            state.controller_key,
            assigned_goal,
            intent,
            third_basis,
            plan_id="plan-3",
            revision=3,
        )
        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                state, PlanRecompiled(third_plan, third_basis)
            )
        self.assertEqual(
            caught.exception.code,
            "intent_no_progress_budget_exhausted",
        )

    def test_verified_step_resets_no_progress_counter(self):
        state, assigned_goal, nav_basis, intent, active_plan = executing_state()
        next_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="nav-basis-progress",
        )
        state, _authorization, _receipt, _dispatched = settle_active_step(
            state,
            next_basis,
        )
        replacement_basis = replace(
            next_basis,
            controller_state_version=3,
            navigation_basis_id="nav-basis-recompiled",
        )
        replacement_plan = execution_plan(
            state.controller_key,
            assigned_goal,
            intent,
            replacement_basis,
            plan_id="plan-2",
            revision=2,
        )
        state = reduce_physical_agent_state(
            state,
            PlanRecompiled(replacement_plan, replacement_basis),
        )

        self.assertEqual(state.intent_progress.plan_attempts, 2)
        self.assertEqual(state.intent_progress.completed_steps, 1)
        self.assertEqual(
            state.intent_progress.consecutive_no_progress_plans,
            0,
        )


class PhysicalAgentDispatchTests(unittest.TestCase):
    def test_authorization_is_exact_sequenced_and_single_flight(self):
        state, _goal, _basis, _intent, _plan = executing_state()
        authorization = step_authorization(state)
        authorized = reduce_physical_agent_state(
            state,
            StepCommandAuthorized(authorization),
        )

        self.assertEqual(authorized.last_host_dispatch_sequence, 1)
        self.assertEqual(
            authorized.active_dispatch,
            ActiveDispatch(authorization),
        )
        self.assertFalse(authorized.active_dispatch.dispatched)
        with self.assertRaises(PhysicalAgentStateError) as concurrent:
            reduce_physical_agent_state(
                authorized,
                StepCommandAuthorized(step_authorization(authorized)),
            )
        self.assertEqual(
            concurrent.exception.code,
            "active_dispatch_already_exists",
        )

        mismatches = (
            replace(
                authorization,
                controller_key=controller("different-boot"),
            ),
            replace(
                authorization,
                step_key=replace(
                    authorization.step_key,
                    cursor=authorization.step_key.cursor + 1,
                ),
            ),
            replace(
                authorization,
                based_on_navigation_basis_id="different-basis",
            ),
            replace(
                authorization,
                based_on_controller_state_version=(
                    authorization.based_on_controller_state_version + 1
                ),
            ),
        )
        for mismatch in mismatches:
            with self.subTest(mismatch=mismatch):
                with self.assertRaises(PhysicalAgentStateError) as caught:
                    reduce_physical_agent_state(
                        state,
                        StepCommandAuthorized(mismatch),
                    )
                self.assertEqual(
                    caught.exception.code,
                    "step_command_authorization_mismatch",
                )

        with self.assertRaises(PhysicalAgentStateError) as skipped:
            reduce_physical_agent_state(
                state,
                StepCommandAuthorized(
                    replace(authorization, host_dispatch_sequence=2)
                ),
            )
        self.assertEqual(
            skipped.exception.code,
            "invalid_host_dispatch_sequence",
        )

    def test_dispatch_is_exact_current_and_revocation_is_before_send_only(self):
        state, _goal, nav_basis, _intent, _plan = executing_state()
        authorization = step_authorization(state)
        authorized = reduce_physical_agent_state(
            state,
            StepCommandAuthorized(authorization),
        )

        with self.assertRaises(PhysicalAgentStateError) as wrong_command:
            reduce_physical_agent_state(
                authorized,
                StepCommandDispatched(
                    replace(authorization, command_id="different-command"),
                    authorization.issued_at_ms + 1,
                    authorization.issued_at_ms + 30_000,
                ),
            )
        self.assertEqual(
            wrong_command.exception.code,
            "active_dispatch_mismatch",
        )
        with self.assertRaises(PhysicalAgentStateError) as expired:
            reduce_physical_agent_state(
                authorized,
                StepCommandDispatched(
                    authorization,
                    authorization.valid_until_ms,
                    authorization.valid_until_ms + 30_000,
                ),
            )
        self.assertEqual(
            expired.exception.code,
            "step_command_authorization_expired",
        )

        version_advanced = reduce_physical_agent_state(
            authorized,
            NavigationBasisUpdated(
                replace(nav_basis, controller_state_version=2)
            ),
        )
        with self.assertRaises(PhysicalAgentStateError) as stale:
            reduce_physical_agent_state(
                version_advanced,
                StepCommandDispatched(
                    authorization,
                    authorization.issued_at_ms + 1,
                    authorization.issued_at_ms + 30_000,
                ),
            )
        self.assertEqual(
            stale.exception.code,
            "stale_step_command_authorization",
        )
        revoked = reduce_physical_agent_state(
            version_advanced,
            StepCommandRevoked(
                authorization,
                authorization.issued_at_ms + 2,
            ),
        )
        self.assertIsNone(revoked.active_dispatch)
        replacement = step_authorization(revoked)
        self.assertEqual(replacement.host_dispatch_sequence, 2)

        dispatched, authorization = dispatch_active_step(state)
        self.assertTrue(dispatched.active_dispatch.dispatched)
        with self.assertRaises(PhysicalAgentStateError) as after_send:
            reduce_physical_agent_state(
                dispatched,
                StepCommandRevoked(
                    authorization,
                    authorization.issued_at_ms + 2,
                ),
            )
        self.assertEqual(
            after_send.exception.code,
            "active_dispatch_mismatch",
        )

    def test_receipt_requires_exact_dispatch_binding_and_host_time(self):
        state, _goal, nav_basis, _intent, _plan = executing_state()
        dispatched, authorization = dispatch_active_step(state)
        resulting_basis = replace(
            nav_basis,
            controller_state_version=2,
        )
        receipt = command_receipt(authorization, resulting_basis)
        mismatches = (
            replace(receipt, controller_key=controller("different-boot")),
            replace(
                receipt,
                step_key=replace(
                    receipt.step_key,
                    plan_id="different-plan",
                ),
            ),
            replace(
                receipt,
                step_key=replace(
                    receipt.step_key,
                    plan_revision=receipt.step_key.plan_revision + 1,
                ),
            ),
            replace(
                receipt,
                step_key=replace(
                    receipt.step_key,
                    cursor=receipt.step_key.cursor + 1,
                ),
            ),
            replace(
                receipt,
                step_key=replace(
                    receipt.step_key,
                    step_id="different-step",
                ),
            ),
            replace(receipt, action_id="different-action"),
            replace(receipt, command_id="different-command"),
            replace(
                receipt,
                host_dispatch_sequence=receipt.host_dispatch_sequence + 1,
            ),
            replace(receipt, command_fingerprint="sha256:different"),
            replace(
                receipt,
                based_on_navigation_basis_id="different-basis",
            ),
            replace(
                receipt,
                based_on_controller_state_version=2,
            ),
        )
        for mismatch in mismatches:
            with self.subTest(mismatch=mismatch):
                with self.assertRaises(PhysicalAgentStateError) as caught:
                    reduce_physical_agent_state(
                        dispatched,
                        StepCommandSettled(
                            mismatch,
                            resulting_basis,
                            StepDisposition.COMPLETE,
                        ),
                    )
                self.assertEqual(
                    caught.exception.code,
                    "command_receipt_mismatch",
                )

        with self.assertRaises(PhysicalAgentStateError) as wrong_result:
            reduce_physical_agent_state(
                dispatched,
                StepCommandSettled(
                    replace(
                        receipt,
                        resulting_controller_state_version=3,
                    ),
                    resulting_basis,
                    StepDisposition.COMPLETE,
                ),
            )
        self.assertEqual(
            wrong_result.exception.code,
            "receipt_resulting_version_mismatch",
        )
        with self.assertRaises(PhysicalAgentStateError) as too_early:
            reduce_physical_agent_state(
                dispatched,
                StepCommandSettled(
                    replace(
                        receipt,
                        received_at_host_ms=(
                            dispatched.active_dispatch.dispatched_at_ms - 1
                        ),
                    ),
                    resulting_basis,
                    StepDisposition.COMPLETE,
                ),
            )
        self.assertEqual(too_early.exception.code, "receipt_before_dispatch")

    def test_expired_settlement_never_advances_and_reconciles_as_blocked(self):
        state, _goal, nav_basis, _intent, active_plan = executing_state()
        authorization = step_authorization(state)
        settle_by_host_ms = authorization.issued_at_ms + 100
        dispatched, authorization = dispatch_active_step(
            state,
            authorization=authorization,
            settle_by_host_ms=settle_by_host_ms,
        )
        resulting_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="basis-after-expired-command",
        )
        late_completed = command_receipt(
            authorization,
            resulting_basis,
            received_at_host_ms=settle_by_host_ms,
        )

        with self.assertRaises(PhysicalAgentStateError) as implicit_expiry:
            reduce_physical_agent_state(
                dispatched,
                StepCommandSettled(
                    late_completed,
                    resulting_basis,
                    StepDisposition.COMPLETE,
                ),
            )
        self.assertEqual(
            implicit_expiry.exception.code,
            "step_command_settlement_expiry_not_recorded",
        )

        with self.assertRaises(PhysicalAgentStateError) as too_early:
            reduce_physical_agent_state(
                dispatched,
                StepCommandSettlementExpired(
                    authorization,
                    settle_by_host_ms - 1,
                ),
            )
        self.assertEqual(
            too_early.exception.code,
            "invalid_command_settlement_expired_at_host_ms",
        )

        expired = reduce_physical_agent_state(
            dispatched,
            StepCommandSettlementExpired(
                authorization,
                settle_by_host_ms,
            ),
        )
        self.assertTrue(expired.active_dispatch.settlement_expired)
        self.assertEqual(expired.plan, active_plan)
        self.assertEqual(expired.plan.cursor, 0)
        self.assertEqual(expired.basis, nav_basis)
        self.assertEqual(expired.intent_progress.completed_steps, 0)

        with self.assertRaises(PhysicalAgentStateError) as completed_after_expiry:
            reduce_physical_agent_state(
                expired,
                StepCommandSettled(
                    late_completed,
                    resulting_basis,
                    StepDisposition.COMPLETE,
                ),
            )
        self.assertEqual(
            completed_after_expiry.exception.code,
            "completed_receipt_after_settlement_expiry",
        )

        rejected = replace(
            late_completed,
            outcome=ReceiptOutcome.REJECTED_NOT_STARTED,
            code="expired_not_started",
        )
        replan_ticket = ticket(
            resulting_basis,
            ticket_id="ticket-expired-command",
            cause=PlanningCause.REPLAN_REQUIRED,
            created_at_ms=settle_by_host_ms,
        )
        reconciled = reduce_physical_agent_state(
            expired,
            StepCommandSettled(
                rejected,
                resulting_basis,
                StepDisposition.BLOCKED,
                replan_ticket,
            ),
        )

        self.assertEqual(reconciled.phase, AgentPhase.PLANNING)
        self.assertIsNone(reconciled.active_dispatch)
        self.assertIsNone(reconciled.plan)
        self.assertEqual(reconciled.planning_ticket, replan_ticket)
        self.assertEqual(reconciled.intent_progress.completed_steps, 0)

    def test_dispositions_are_finite_and_blocked_requires_replanning(self):
        state, _goal, nav_basis, _intent, _plan = executing_state()
        continue_basis = replace(
            nav_basis,
            controller_state_version=2,
        )
        continued, _authorization, _receipt, _dispatched = settle_active_step(
            state,
            continue_basis,
            disposition=StepDisposition.CONTINUE,
        )
        self.assertEqual(continued.phase, AgentPhase.EXECUTING)
        self.assertEqual(continued.plan.cursor, 0)
        self.assertEqual(continued.intent_progress.completed_steps, 0)
        self.assertIsNone(continued.active_dispatch)

        blocked_basis = replace(
            continue_basis,
            controller_state_version=3,
            navigation_basis_id="basis-blocked",
        )
        dispatched, authorization = dispatch_active_step(continued)
        rejected = command_receipt(
            authorization,
            blocked_basis,
            outcome=ReceiptOutcome.REJECTED_NOT_STARTED,
            code="controller_rejected",
        )
        with self.assertRaises(PhysicalAgentStateError) as wrong_disposition:
            reduce_physical_agent_state(
                dispatched,
                StepCommandSettled(
                    rejected,
                    blocked_basis,
                    StepDisposition.COMPLETE,
                ),
            )
        self.assertEqual(
            wrong_disposition.exception.code,
            "invalid_receipt_disposition",
        )
        with self.assertRaises(PhysicalAgentStateError) as missing_ticket:
            reduce_physical_agent_state(
                dispatched,
                StepCommandSettled(
                    rejected,
                    blocked_basis,
                    StepDisposition.BLOCKED,
                ),
            )
        self.assertEqual(
            missing_ticket.exception.code,
            "missing_blocked_replan_ticket",
        )
        replan_ticket = ticket(
            blocked_basis,
            ticket_id="ticket-command-blocked",
            cause=PlanningCause.REPLAN_REQUIRED,
            created_at_ms=130,
        )
        blocked = reduce_physical_agent_state(
            dispatched,
            StepCommandSettled(
                rejected,
                blocked_basis,
                StepDisposition.BLOCKED,
                replan_ticket,
            ),
        )

        self.assertEqual(blocked.phase, AgentPhase.PLANNING)
        self.assertIsNone(blocked.plan)
        self.assertIsNone(blocked.active_dispatch)
        self.assertEqual(blocked.planning_ticket, replan_ticket)
        with self.assertRaises(PhysicalAgentStateError) as unconfirmed_stop:
            command_receipt(
                authorization,
                blocked_basis,
                outcome=ReceiptOutcome.STOPPED,
                stop_confirmed=False,
            )
        self.assertEqual(
            unconfirmed_stop.exception.code,
            "missing_receipt_stop_confirmation",
        )

    def test_plan_replacement_and_recompile_reject_an_active_dispatch(self):
        state, assigned_goal, nav_basis, intent, _plan = executing_state()
        planning_ticket = ticket(
            nav_basis,
            ticket_id="ticket-replacement",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=120,
        )
        pending = reduce_physical_agent_state(
            state,
            PlanningRequested(planning_ticket),
        )
        pending = reduce_physical_agent_state(
            pending,
            PlanningTicketConsumed(planning_ticket, 121),
        )
        authorization = step_authorization(pending)
        active = reduce_physical_agent_state(
            pending,
            StepCommandAuthorized(authorization),
        )
        replacement_intent = active_intent(
            nav_basis,
            assigned_goal,
            intent_id=intent.intent_id,
            revision=2,
            accepted_at_ms=122,
        )
        replacement_plan = execution_plan(
            state.controller_key,
            assigned_goal,
            replacement_intent,
            nav_basis,
            plan_id="plan-2",
            revision=2,
        )
        prepared = prepared_for_state(
            active,
            replacement_intent,
            replacement_plan,
            proposal_id="proposal-active-dispatch",
        )
        prepared_active = reduce_physical_agent_state(
            active,
            IntentPrepared(prepared),
        )
        with self.assertRaises(PhysicalAgentStateError) as activation:
            reduce_physical_agent_state(
                prepared_active,
                PreparedIntentAccepted(
                    prepared,
                    prepared.prepared_at_ms,
                ),
            )
        self.assertEqual(
            activation.exception.code,
            "active_dispatch_conflict",
        )

        events = (
            PlanRecompiled(replacement_plan, nav_basis),
            ReplanRequested(
                ticket(
                    nav_basis,
                    ticket_id="ticket-blocking-replan",
                    cause=PlanningCause.REPLAN_REQUIRED,
                    created_at_ms=123,
                ),
                nav_basis,
                "plan_changed",
            ),
        )
        for event in events:
            with self.subTest(event=type(event).__name__):
                with self.assertRaises(PhysicalAgentStateError) as caught:
                    reduce_physical_agent_state(active, event)
                self.assertEqual(
                    caught.exception.code,
                    "active_dispatch_conflict",
                )

    def test_stop_drops_unsent_authorization_but_retains_sent_dispatch(self):
        state, _goal, nav_basis, _intent, _plan = executing_state()
        terminal = GoalTerminal(
            GoalOutcome.CANCELLED,
            "operator_stop",
            130,
        )
        authorization = step_authorization(state)
        authorized = reduce_physical_agent_state(
            state,
            StepCommandAuthorized(authorization),
        )
        unsent_stop = reduce_physical_agent_state(
            authorized,
            StopRequested(terminal),
        )
        self.assertEqual(unsent_stop.phase, AgentPhase.STOPPING)
        self.assertIsNone(unsent_stop.active_dispatch)

        dispatched, authorization = dispatch_active_step(state)
        sent_stop = reduce_physical_agent_state(
            dispatched,
            StopRequested(terminal),
        )
        self.assertEqual(sent_stop.phase, AgentPhase.STOPPING)
        self.assertEqual(sent_stop.active_dispatch, dispatched.active_dispatch)
        self.assertTrue(sent_stop.active_dispatch.dispatched)
        with self.assertRaises(PhysicalAgentStateError):
            replace(
                sent_stop,
                active_dispatch=ActiveDispatch(authorization),
            )

        with self.assertRaises(PhysicalAgentStateError) as missing_receipt:
            reduce_physical_agent_state(sent_stop, StopVerified(131))
        self.assertEqual(
            missing_receipt.exception.code,
            "missing_stop_dispatch_receipt",
        )

        stopped_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="basis-stop-confirmed",
        )
        stopped_receipt = command_receipt(
            authorization,
            stopped_basis,
            outcome=ReceiptOutcome.STOPPED,
            received_at_host_ms=131,
            code="stop_confirmed",
        )
        with self.assertRaises(PhysicalAgentStateError) as mismatch:
            reduce_physical_agent_state(
                sent_stop,
                StopVerified(
                    132,
                    replace(stopped_receipt, command_id="wrong-command"),
                    stopped_basis,
                ),
            )
        self.assertEqual(mismatch.exception.code, "command_receipt_mismatch")

        with self.assertRaises(PhysicalAgentStateError) as completed_receipt:
            reduce_physical_agent_state(
                sent_stop,
                StopVerified(
                    132,
                    replace(stopped_receipt, outcome=ReceiptOutcome.COMPLETED),
                    stopped_basis,
                ),
            )
        self.assertEqual(
            completed_receipt.exception.code,
            "invalid_stop_dispatch_receipt",
        )
        with self.assertRaises(PhysicalAgentStateError) as before_stop_request:
            reduce_physical_agent_state(
                sent_stop,
                StopVerified(
                    132,
                    replace(stopped_receipt, received_at_host_ms=129),
                    stopped_basis,
                ),
            )
        self.assertEqual(
            before_stop_request.exception.code,
            "stop_receipt_time_mismatch",
        )

        stopped = reduce_physical_agent_state(
            sent_stop,
            StopVerified(132, stopped_receipt, stopped_basis),
        )
        self.assertEqual(stopped.phase, AgentPhase.TERMINAL)
        self.assertEqual(stopped.basis, stopped_basis)
        self.assertIsNone(stopped.active_dispatch)

    def test_stop_without_active_dispatch_rejects_command_evidence(self):
        executing, _goal, nav_basis, _intent, _plan = executing_state()
        authorization = step_authorization(executing)
        resulting_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="basis-unexpected-stop-receipt",
        )
        receipt = command_receipt(
            authorization,
            resulting_basis,
            outcome=ReceiptOutcome.STOPPED,
            received_at_host_ms=131,
        )
        stopping, _goal, _basis, _terminal = stopping_state()

        with self.assertRaises(PhysicalAgentStateError) as unexpected:
            reduce_physical_agent_state(
                stopping,
                StopVerified(132, receipt, resulting_basis),
            )
        self.assertEqual(
            unexpected.exception.code,
            "unexpected_stop_dispatch_receipt",
        )


class PhysicalAgentFreshnessTests(unittest.TestCase):
    def test_wrong_controller_identity_is_rejected(self):
        state = PhysicalAgentState(controller())
        wrong_basis = basis(key=controller("instance-2"))

        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                state,
                GoalActivated(
                    goal(),
                    wrong_basis,
                    ticket(wrong_basis),
                ),
            )
        self.assertEqual(caught.exception.code, "invalid_goal_activation")

    def test_relevant_basis_change_discards_inflight_ticket(self):
        state, _goal, nav_basis, planning_ticket = activated_state()
        state = reduce_physical_agent_state(
            state,
            PlanningTicketConsumed(planning_ticket, 102),
        )
        changed = replace(
            nav_basis,
            controller_state_version=2,
            world_model_version=2,
            navigation_basis_id="different-navigation-evidence",
        )
        updated = reduce_physical_agent_state(
            state, NavigationBasisUpdated(changed)
        )

        self.assertIsNone(updated.planning_ticket)
        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                updated,
                PlanningHeld(
                    replace(
                        planning_ticket,
                        consumed_at_ms=102,
                    ),
                    "proposal-stale-hold",
                ),
            )
        self.assertEqual(caught.exception.code, "planning_ticket_mismatch")

    def test_irrelevant_world_update_does_not_starve_planner_result(self):
        state, assigned_goal, nav_basis, planning_ticket = activated_state()
        state = reduce_physical_agent_state(
            state,
            PlanningTicketConsumed(planning_ticket, 102),
        )
        same_evidence = replace(nav_basis, world_model_version=20)
        updated = reduce_physical_agent_state(
            state, NavigationBasisUpdated(same_evidence)
        )
        intent = active_intent(nav_basis, assigned_goal)
        active_plan = execution_plan(
            state.controller_key,
            assigned_goal,
            intent,
            same_evidence,
        )

        accepted = accept_intent_plan(
            updated,
            intent,
            active_plan,
            proposal_id="proposal-irrelevant-update",
        )

        self.assertEqual(accepted.phase, AgentPhase.EXECUTING)
        self.assertEqual(accepted.basis.world_model_version, 20)

    def test_regressed_or_rebound_basis_is_rejected(self):
        state, _goal, nav_basis, _intent, _plan = executing_state()
        advanced = replace(
            nav_basis,
            controller_state_version=4,
            world_model_version=4,
        )
        state = reduce_physical_agent_state(
            state, NavigationBasisUpdated(advanced)
        )
        candidates = (
            replace(advanced, controller_state_version=3),
            replace(advanced, world_model_version=3),
            replace(advanced, controller_key=controller("instance-2")),
            replace(advanced, calibration_fingerprint="calibration-b"),
            replace(advanced, world_generation_id="world-2"),
        )

        for candidate in candidates:
            with self.subTest(candidate=candidate):
                with self.assertRaises(PhysicalAgentStateError):
                    reduce_physical_agent_state(
                        state,
                        NavigationBasisUpdated(candidate),
                    )

    def test_plan_compiled_for_another_basis_is_rejected(self):
        state, assigned_goal, nav_basis, planning_ticket = consumed_planning_state()
        intent = active_intent(nav_basis, assigned_goal)
        wrong_basis = replace(
            nav_basis,
            navigation_basis_id="wrong-evidence",
        )
        wrong_plan = execution_plan(
            state.controller_key,
            assigned_goal,
            intent,
            wrong_basis,
        )

        with self.assertRaises(PhysicalAgentStateError) as caught:
            prepared_for_state(state, intent, wrong_plan)
        self.assertEqual(
            caught.exception.code,
            "prepared_intent_binding_mismatch",
        )


if __name__ == "__main__":
    unittest.main()
