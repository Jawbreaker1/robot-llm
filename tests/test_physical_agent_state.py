from dataclasses import replace
import unittest

from robot_agent.physical_agent_state import (
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
    MAX_PLANNING_TICKET_TTL_MS,
    NavigationBasis,
    NavigationBasisUpdated,
    PhysicalAgentState,
    PhysicalAgentStateError,
    PhysicalAgentStateReducer,
    PlanBinding,
    PlanFinished,
    PlanRecompiled,
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
        PlanningTicketConsumed(
            planning_ticket.ticket_id,
            planning_ticket.basis,
            102,
        ),
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
    state = reduce_physical_agent_state(
        state,
        IntentAccepted(
            planning_ticket.ticket_id,
            planning_ticket.basis,
            intent,
            active_plan,
        ),
    )
    return state, assigned_goal, nav_basis, intent, active_plan


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
            PlanningTicketConsumed("ticket-1", nav_basis, 102)
        )
        versions.append(claimed.agent_state_version)
        intent = active_intent(nav_basis, assigned_goal)
        active_plan = execution_plan(
            controller(), assigned_goal, intent, nav_basis
        )
        executing = reducer.apply(
            IntentAccepted("ticket-1", nav_basis, intent, active_plan)
        )
        versions.append(executing.agent_state_version)
        step_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="nav-step-1",
        )
        advanced = reducer.apply(
            PlanStepAdvanced(
                active_plan.plan_id,
                active_plan.revision,
                0,
                step_basis,
            )
        )
        versions.append(advanced.agent_state_version)
        final_basis = replace(
            step_basis,
            controller_state_version=3,
            navigation_basis_id="nav-plan-finished",
        )
        finished = reducer.apply(
            PlanFinished(
                active_plan.plan_id,
                active_plan.revision,
                1,
                final_basis,
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
            PlanningHeld(planning_ticket.ticket_id, nav_basis),
        )

        self.assertEqual(held.phase, AgentPhase.PLANNING)
        self.assertIsNone(held.planning_ticket)
        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                held,
                PlanningHeld(planning_ticket.ticket_id, nav_basis),
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
            PlanningTicketConsumed(
                parallel_ticket.ticket_id,
                nav_basis,
                121,
            ),
        )
        held = reduce_physical_agent_state(
            consumed,
            PlanningHeld(parallel_ticket.ticket_id, nav_basis),
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
            PlanningTicketConsumed(
                parallel_ticket.ticket_id,
                nav_basis,
                121,
            ),
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

        replaced = reduce_physical_agent_state(
            pending,
            IntentAccepted(
                parallel_ticket.ticket_id,
                nav_basis,
                replacement_intent,
                replacement_plan,
            ),
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
            PlanningTicketConsumed("ticket-2", next_basis, 121),
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
        executing = reduce_physical_agent_state(
            claimed,
            IntentAccepted("ticket-2", next_basis, revised, next_plan),
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
            PlanningTicketConsumed(
                abort_ticket.ticket_id,
                nav_basis,
                121,
            ),
        )
        terminal = GoalTerminal(
            GoalOutcome.FAILED,
            "planner_found_no_route",
            122,
        )
        stopping = reduce_physical_agent_state(
            consumed,
            PlanningAbortRequested(
                abort_ticket.ticket_id,
                nav_basis,
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
                    abort_ticket.ticket_id,
                    nav_basis,
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
                    abort_ticket.ticket_id,
                    nav_basis,
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
            PlanningTicketConsumed(
                old_ticket.ticket_id,
                nav_basis,
                121,
            ),
        )
        changed_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="basis-after-motion",
        )
        superseded = reduce_physical_agent_state(
            state,
            PlanStepAdvanced(
                active_plan.plan_id,
                active_plan.revision,
                0,
                changed_basis,
            ),
        )

        self.assertEqual(superseded.phase, AgentPhase.EXECUTING)
        self.assertEqual(superseded.plan.cursor, 1)
        self.assertIsNone(superseded.planning_ticket)
        with self.assertRaises(PhysicalAgentStateError) as stale:
            reduce_physical_agent_state(
                superseded,
                PlanningAbortRequested(
                    old_ticket.ticket_id,
                    nav_basis,
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
            PlanningHeld(planning_ticket.ticket_id, nav_basis),
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
        events = (
            (
                GoalActivated(assigned_goal, nav_basis, planning_ticket),
                {AgentPhase.IDLE, AgentPhase.TERMINAL},
            ),
            (
                PlanningTicketConsumed("ticket-1", nav_basis, 102),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                PlanningTicketExpired(
                    "ticket-1",
                    nav_basis,
                    planning_ticket.valid_until_ms,
                ),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                IntentAccepted(
                    "ticket-1", nav_basis, intent, intent_plan
                ),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                PlanningAbortRequested(
                    "ticket-1",
                    nav_basis,
                    pending_terminal,
                ),
                {AgentPhase.PLANNING, AgentPhase.EXECUTING},
            ),
            (
                PlanningHeld("ticket-1", nav_basis),
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
                PlanStepAdvanced(
                    active_plan.plan_id,
                    active_plan.revision,
                    active_plan.cursor,
                    successor,
                ),
                {AgentPhase.EXECUTING},
            ),
            (
                PlanFinished(
                    active_plan.plan_id,
                    active_plan.revision,
                    active_plan.cursor,
                    successor,
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

        state, _goal, _basis, _ticket = activated_state()
        state = reduce_physical_agent_state(
            state,
            PlanningTicketConsumed("ticket-1", nav_basis, 102),
        )
        with self.assertRaises(PhysicalAgentStateError):
            reduce_physical_agent_state(
                state,
                PlanningTicketConsumed("ticket-1", nav_basis, 103),
            )

    def test_expired_unconsumed_ticket_can_be_dismissed_without_wedging(self):
        state, _goal, nav_basis, planning_ticket = activated_state()

        with self.assertRaises(PhysicalAgentStateError) as early:
            reduce_physical_agent_state(
                state,
                PlanningTicketExpired(
                    planning_ticket.ticket_id,
                    nav_basis,
                    planning_ticket.valid_until_ms - 1,
                ),
            )
        self.assertEqual(early.exception.code, "planning_ticket_not_expired")

        dismissed = reduce_physical_agent_state(
            state,
            PlanningTicketExpired(
                planning_ticket.ticket_id,
                nav_basis,
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
            PlanningTicketExpired(
                parallel_ticket.ticket_id,
                nav_basis,
                130,
            ),
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
                planning_ticket.ticket_id,
                nav_basis,
                planning_ticket.created_at_ms + 1,
            ),
        )

        recovered = reduce_physical_agent_state(
            consumed,
            PlanningTicketExpired(
                planning_ticket.ticket_id,
                nav_basis,
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
                PlanningTicketConsumed(
                    expiring.ticket_id,
                    nav_basis,
                    110,
                ),
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

    def test_step_keeps_only_a_decision_equivalent_parallel_ticket(self):
        state, _goal, nav_basis, _intent, active_plan = executing_state()
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
        equivalent = reduce_physical_agent_state(
            pending,
            PlanStepAdvanced(
                active_plan.plan_id,
                active_plan.revision,
                0,
                equivalent_basis,
            ),
        )

        changed_basis = replace(
            nav_basis,
            controller_state_version=2,
            world_model_version=2,
            navigation_basis_id="basis-decision-changed",
        )
        changed = reduce_physical_agent_state(
            pending,
            PlanStepAdvanced(
                active_plan.plan_id,
                active_plan.revision,
                0,
                changed_basis,
            ),
        )

        self.assertEqual(equivalent.phase, AgentPhase.EXECUTING)
        self.assertEqual(equivalent.plan.cursor, 1)
        self.assertEqual(equivalent.planning_ticket, parallel_ticket)
        self.assertEqual(changed.phase, AgentPhase.EXECUTING)
        self.assertEqual(changed.plan.cursor, 1)
        self.assertIsNone(changed.planning_ticket)

    def test_plan_finished_clears_even_a_still_fresh_parallel_ticket(self):
        state, _goal, nav_basis, _intent, active_plan = executing_state()
        parallel_ticket = ticket(
            nav_basis,
            ticket_id="ticket-until-plan-finished",
            cause=PlanningCause.UNCERTAINTY,
            created_at_ms=120,
        )
        pending = reduce_physical_agent_state(
            state,
            PlanningRequested(parallel_ticket),
        )
        step_basis = replace(nav_basis, controller_state_version=2)
        pending = reduce_physical_agent_state(
            pending,
            PlanStepAdvanced(
                active_plan.plan_id,
                active_plan.revision,
                0,
                step_basis,
            ),
        )
        final_basis = replace(step_basis, controller_state_version=3)
        finished = reduce_physical_agent_state(
            pending,
            PlanFinished(
                active_plan.plan_id,
                active_plan.revision,
                1,
                final_basis,
            ),
        )

        self.assertEqual(pending.planning_ticket, parallel_ticket)
        self.assertEqual(finished.phase, AgentPhase.PLANNING)
        self.assertTrue(finished.compile_pending)
        self.assertIsNone(finished.plan)
        self.assertIsNone(finished.planning_ticket)

    def test_cursor_advances_once_and_final_step_needs_transition(self):
        state, _goal, nav_basis, _intent, active_plan = executing_state()
        next_basis = replace(nav_basis, controller_state_version=2)
        event = PlanStepAdvanced(
            active_plan.plan_id,
            active_plan.revision,
            0,
            next_basis,
        )
        advanced = reduce_physical_agent_state(state, event)

        self.assertEqual(advanced.plan.cursor, 1)
        self.assertEqual(advanced.plan.active_step.step_id, "waypoint-2")
        with self.assertRaises(PhysicalAgentStateError) as stale:
            reduce_physical_agent_state(advanced, event)
        self.assertEqual(stale.exception.code, "plan_cursor_mismatch")

        final_basis = replace(next_basis, controller_state_version=3)
        with self.assertRaises(PhysicalAgentStateError) as final:
            reduce_physical_agent_state(
                advanced,
                PlanStepAdvanced(
                    active_plan.plan_id,
                    active_plan.revision,
                    1,
                    final_basis,
                ),
            )
        self.assertEqual(
            final.exception.code,
            "final_step_requires_transition",
        )

        finished = reduce_physical_agent_state(
            advanced,
            PlanFinished(
                active_plan.plan_id,
                active_plan.revision,
                1,
                final_basis,
            ),
        )
        self.assertEqual(finished.phase, AgentPhase.PLANNING)
        self.assertTrue(finished.compile_pending)
        self.assertIsNone(finished.plan)
        self.assertIsNone(finished.planning_ticket)
        self.assertEqual(finished.intent, state.intent)
        self.assertEqual(finished.intent_progress.completed_steps, 2)

    def test_plan_finished_validates_final_cursor_binding_and_basis(self):
        state, _goal, nav_basis, _intent, active_plan = executing_state()
        next_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="nav-basis-final-cursor",
        )
        with self.assertRaises(PhysicalAgentStateError) as not_final:
            reduce_physical_agent_state(
                state,
                PlanFinished(
                    active_plan.plan_id,
                    active_plan.revision,
                    0,
                    next_basis,
                ),
            )
        self.assertEqual(not_final.exception.code, "plan_not_at_final_step")

        advanced = reduce_physical_agent_state(
            state,
            PlanStepAdvanced(
                active_plan.plan_id,
                active_plan.revision,
                0,
                next_basis,
            ),
        )
        final_basis = replace(
            next_basis,
            controller_state_version=3,
            navigation_basis_id="nav-basis-finished",
        )
        mismatches = (
            ("wrong-plan", active_plan.revision, 1),
            (active_plan.plan_id, active_plan.revision + 1, 1),
            (active_plan.plan_id, active_plan.revision, 0),
        )
        for plan_id, revision, cursor in mismatches:
            with self.subTest(
                plan_id=plan_id,
                revision=revision,
                cursor=cursor,
            ):
                with self.assertRaises(PhysicalAgentStateError) as mismatch:
                    reduce_physical_agent_state(
                        advanced,
                        PlanFinished(
                            plan_id,
                            revision,
                            cursor,
                            final_basis,
                        ),
                    )
                self.assertEqual(
                    mismatch.exception.code,
                    "plan_cursor_mismatch",
                )

        with self.assertRaises(PhysicalAgentStateError) as stale:
            reduce_physical_agent_state(
                advanced,
                PlanFinished(
                    active_plan.plan_id,
                    active_plan.revision,
                    1,
                    nav_basis,
                ),
            )
        self.assertEqual(stale.exception.code, "stale_navigation_basis")

    def test_finished_plan_recompiles_same_intent_without_model_ticket(self):
        state, assigned_goal, nav_basis, intent, active_plan = executing_state()
        step_basis = replace(
            nav_basis,
            controller_state_version=2,
            navigation_basis_id="nav-basis-step",
        )
        state = reduce_physical_agent_state(
            state,
            PlanStepAdvanced(
                active_plan.plan_id,
                active_plan.revision,
                0,
                step_basis,
            ),
        )
        finished_basis = replace(
            step_basis,
            controller_state_version=3,
            navigation_basis_id="nav-basis-plan-finished",
        )
        planning = reduce_physical_agent_state(
            state,
            PlanFinished(
                active_plan.plan_id,
                active_plan.revision,
                1,
                finished_basis,
            ),
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
            PlanningTicketConsumed(
                replan_ticket.ticket_id,
                replan_basis,
                121,
            ),
        )
        held = reduce_physical_agent_state(
            planning,
            PlanningHeld(replan_ticket.ticket_id, replan_basis),
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
        state = reduce_physical_agent_state(
            state,
            IntentAccepted(
                planning_ticket.ticket_id,
                nav_basis,
                intent,
                first_plan,
            ),
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
        state = reduce_physical_agent_state(
            state,
            PlanStepAdvanced(
                active_plan.plan_id,
                active_plan.revision,
                0,
                next_basis,
            ),
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
            PlanningTicketConsumed(
                planning_ticket.ticket_id, nav_basis, 102
            ),
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
                PlanningHeld(planning_ticket.ticket_id, nav_basis),
            )
        self.assertEqual(caught.exception.code, "planning_ticket_mismatch")

    def test_irrelevant_world_update_does_not_starve_planner_result(self):
        state, assigned_goal, nav_basis, planning_ticket = activated_state()
        state = reduce_physical_agent_state(
            state,
            PlanningTicketConsumed(
                planning_ticket.ticket_id, nav_basis, 102
            ),
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

        accepted = reduce_physical_agent_state(
            updated,
            IntentAccepted(
                planning_ticket.ticket_id,
                nav_basis,
                intent,
                active_plan,
            ),
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
            reduce_physical_agent_state(
                state,
                IntentAccepted(
                    planning_ticket.ticket_id,
                    nav_basis,
                    intent,
                    wrong_plan,
                ),
            )
        self.assertEqual(caught.exception.code, "plan_basis_mismatch")


if __name__ == "__main__":
    unittest.main()
