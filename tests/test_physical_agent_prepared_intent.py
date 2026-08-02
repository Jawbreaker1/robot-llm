from dataclasses import replace
import unittest

from robot_agent.physical_agent_state import (
    AgentPhase,
    IntentPrepared,
    NavigationBasisUpdated,
    PhysicalAgentStateError,
    PlanningAbortRequested,
    PlanningCause,
    PlanningHeld,
    PlanningRequested,
    PlanningTicketConsumed,
    PreparedIntentAccepted,
    PreparedIntentExpired,
    PreparedIntentPlan,
    StepCommandAuthorized,
    StepCommandSettled,
    StepDisposition,
    StopRequested,
    GoalOutcome,
    GoalTerminal,
    reduce_physical_agent_state,
)
from tests.test_physical_agent_state import (
    active_intent,
    command_receipt,
    dispatch_active_step,
    executing_state,
    execution_plan,
    step_authorization,
    ticket,
)


def successor(state, *, relevant=False, controller_version=2):
    return replace(
        state.basis,
        controller_state_version=controller_version,
        world_model_version=controller_version,
        navigation_basis_id=(
            "nav-basis-relevant-change"
            if relevant
            else state.basis.navigation_basis_id
        ),
    )


def with_parallel_ticket(state, *, valid_until_ms=5_000):
    planning_ticket = ticket(
        state.basis,
        ticket_id="parallel-ticket-1",
        cause=PlanningCause.UNCERTAINTY,
        created_at_ms=200,
        valid_until_ms=valid_until_ms,
    )
    state = reduce_physical_agent_state(
        state,
        PlanningRequested(planning_ticket),
    )
    state = reduce_physical_agent_state(
        state,
        PlanningTicketConsumed(planning_ticket, 201),
    )
    return state, planning_ticket


def prepared_intent_plan(
    state,
    assigned_goal,
    *,
    proposal_id="proposal-2",
    plan_id="prepared-plan-2",
    prepared_at_ms=220,
    valid_until_ms=1_500,
    compilation_basis=None,
):
    compilation_basis = compilation_basis or state.basis
    current = state.intent
    intent = active_intent(
        state.planning_ticket.basis,
        assigned_goal,
        intent_id=current.intent_id,
        revision=current.revision + 1,
        accepted_at_ms=prepared_at_ms,
    )
    plan = execution_plan(
        state.controller_key,
        assigned_goal,
        intent,
        compilation_basis,
        plan_id=plan_id,
        revision=state.plan_revision + 1,
    )
    return PreparedIntentPlan(
        ticket=state.planning_ticket,
        proposal_id=proposal_id,
        compilation_basis=compilation_basis,
        intent=intent,
        plan=plan,
        prepared_at_ms=prepared_at_ms,
        valid_until_ms=valid_until_ms,
    )


def settle_existing_dispatch(
    state,
    authorization,
    resulting_basis,
    *,
    disposition=StepDisposition.CONTINUE,
    replan_ticket=None,
):
    return reduce_physical_agent_state(
        state,
        StepCommandSettled(
            receipt=command_receipt(
                authorization,
                resulting_basis,
                received_at_host_ms=230,
            ),
            resulting_basis=resulting_basis,
            disposition=disposition,
            replan_ticket=replan_ticket,
        ),
    )


class PreparedIntentValueTests(unittest.TestCase):
    def test_prepared_value_has_one_finite_exact_binding(self):
        state, assigned_goal, _basis, _intent, _plan = executing_state()
        state, planning_ticket = with_parallel_ticket(
            state,
            valid_until_ms=100_000,
        )
        prepared = prepared_intent_plan(state, assigned_goal)

        self.assertEqual(prepared.ticket_id, planning_ticket.ticket_id)
        self.assertEqual(prepared.valid_until_ms, 1_500)
        with self.assertRaises(PhysicalAgentStateError):
            replace(prepared, valid_until_ms=prepared.prepared_at_ms)
        with self.assertRaises(PhysicalAgentStateError):
            replace(
                prepared,
                valid_until_ms=prepared.prepared_at_ms + 60_001,
            )
        with self.assertRaises(PhysicalAgentStateError):
            replace(
                prepared,
                plan=replace(
                    prepared.plan,
                    binding=replace(
                        prepared.plan.binding,
                        based_on_navigation_basis_id="another-basis",
                    ),
                ),
            )

    def test_ticket_and_exact_compilation_basis_are_both_retained(self):
        state, assigned_goal, _basis, _intent, _plan = executing_state()
        state, _planning_ticket = with_parallel_ticket(state)
        compilation_basis = successor(state, controller_version=2)
        state = reduce_physical_agent_state(
            state,
            NavigationBasisUpdated(compilation_basis),
        )

        prepared = prepared_intent_plan(
            state,
            assigned_goal,
            compilation_basis=compilation_basis,
        )
        prepared_state = reduce_physical_agent_state(
            state,
            IntentPrepared(prepared),
        )

        self.assertEqual(
            prepared_state.prepared_intent_plan.ticket_basis,
            state.planning_ticket.basis,
        )
        self.assertEqual(
            prepared_state.prepared_intent_plan.compilation_basis,
            compilation_basis,
        )


class PreparedIntentReducerTests(unittest.TestCase):
    def _prepared_during_dispatch(self, *, final_step=False):
        state, assigned_goal, _basis, _intent, _plan = executing_state()
        if final_step:
            first_basis = successor(state, controller_version=2)
            dispatched, authorization = dispatch_active_step(state)
            state = settle_existing_dispatch(
                dispatched,
                authorization,
                first_basis,
                disposition=StepDisposition.COMPLETE,
            )
        state, _planning_ticket = with_parallel_ticket(state)
        dispatched, authorization = dispatch_active_step(state)
        prepared = prepared_intent_plan(dispatched, assigned_goal)
        prepared_state = reduce_physical_agent_state(
            dispatched,
            IntentPrepared(prepared),
        )
        return prepared_state, prepared, authorization, assigned_goal

    def test_prepare_is_single_and_dispatch_can_settle_before_activation(self):
        state, prepared, authorization, _goal = self._prepared_during_dispatch()

        self.assertEqual(state.prepared_intent_plan, prepared)
        with self.assertRaisesRegex(
            PhysicalAgentStateError,
            "only one prepared intent plan",
        ):
            reduce_physical_agent_state(state, IntentPrepared(prepared))

        resulting_basis = successor(state)
        settled = settle_existing_dispatch(
            state,
            authorization,
            resulting_basis,
        )
        self.assertEqual(settled.phase, AgentPhase.EXECUTING)
        self.assertEqual(settled.prepared_intent_plan, prepared)
        self.assertIsNone(settled.active_dispatch)

        with self.assertRaisesRegex(
            PhysicalAgentStateError,
            "prepared intent must activate",
        ):
            reduce_physical_agent_state(
                settled,
                StepCommandAuthorized(step_authorization(settled)),
            )

        accepted = reduce_physical_agent_state(
            settled,
            PreparedIntentAccepted(prepared, 240),
        )
        self.assertEqual(accepted.phase, AgentPhase.EXECUTING)
        self.assertEqual(accepted.intent, prepared.intent)
        self.assertEqual(accepted.plan, prepared.plan)
        self.assertIsNone(accepted.prepared_intent_plan)
        self.assertIsNone(accepted.planning_ticket)

    def test_decision_equivalent_update_retains_but_relevant_update_discards(self):
        state, prepared, _authorization, _goal = self._prepared_during_dispatch()
        equivalent = successor(state)
        retained = reduce_physical_agent_state(
            state,
            NavigationBasisUpdated(equivalent),
        )
        self.assertEqual(retained.prepared_intent_plan, prepared)
        self.assertIsNotNone(retained.planning_ticket)

        relevant = replace(
            equivalent,
            controller_state_version=3,
            world_model_version=3,
            navigation_basis_id="nav-basis-relevant-change",
        )
        discarded = reduce_physical_agent_state(
            retained,
            NavigationBasisUpdated(relevant),
        )
        self.assertIsNone(discarded.prepared_intent_plan)
        self.assertIsNone(discarded.planning_ticket)

    def test_blocked_receipt_and_stop_clear_prepared_result(self):
        state, _prepared, authorization, _goal = self._prepared_during_dispatch()
        blocked_basis = successor(state, relevant=True)
        replan = ticket(
            blocked_basis,
            ticket_id="blocked-ticket",
            cause=PlanningCause.REPLAN_REQUIRED,
            created_at_ms=230,
        )
        blocked = settle_existing_dispatch(
            state,
            authorization,
            blocked_basis,
            disposition=StepDisposition.BLOCKED,
            replan_ticket=replan,
        )
        self.assertIsNone(blocked.prepared_intent_plan)
        self.assertEqual(blocked.planning_ticket, replan)

        state, _prepared, _authorization, _goal = self._prepared_during_dispatch()
        stopped = reduce_physical_agent_state(
            state,
            StopRequested(
                GoalTerminal(GoalOutcome.CANCELLED, "operator_stop", 240)
            ),
        )
        self.assertEqual(stopped.phase, AgentPhase.STOPPING)
        self.assertIsNone(stopped.prepared_intent_plan)
        self.assertIsNone(stopped.planning_ticket)

    def test_late_hold_or_abort_cannot_erase_prepared_winner(self):
        state, prepared, _authorization, _goal = (
            self._prepared_during_dispatch()
        )

        for event in (
            PlanningHeld(
                state.planning_ticket,
                "proposal-losing-hold",
            ),
            PlanningAbortRequested(
                state.planning_ticket,
                "proposal-losing-abort",
                GoalTerminal(
                    GoalOutcome.FAILED,
                    "losing_abort",
                    240,
                ),
            ),
        ):
            with self.subTest(event=type(event).__name__):
                with self.assertRaises(PhysicalAgentStateError) as caught:
                    reduce_physical_agent_state(state, event)
                self.assertEqual(
                    caught.exception.code,
                    "prepared_intent_conflict",
                )
                self.assertEqual(state.prepared_intent_plan, prepared)

    def test_final_completed_step_preserves_ticket_and_prepared_for_accept(self):
        state, prepared, authorization, _goal = self._prepared_during_dispatch(
            final_step=True
        )
        resulting_basis = successor(state, controller_version=3)
        settled = settle_existing_dispatch(
            state,
            authorization,
            resulting_basis,
            disposition=StepDisposition.COMPLETE,
        )

        self.assertEqual(settled.phase, AgentPhase.PLANNING)
        self.assertFalse(settled.compile_pending)
        self.assertIsNone(settled.plan)
        self.assertEqual(settled.prepared_intent_plan, prepared)
        self.assertTrue(settled.planning_ticket.consumed)

        accepted = reduce_physical_agent_state(
            settled,
            PreparedIntentAccepted(prepared, 240),
        )
        self.assertEqual(accepted.phase, AgentPhase.EXECUTING)
        self.assertEqual(accepted.plan, prepared.plan)

    def test_final_step_preserves_consumed_ticket_while_planner_is_in_flight(self):
        state, assigned_goal, _basis, _intent, _plan = executing_state()
        first_basis = successor(state, controller_version=2)
        dispatched, authorization = dispatch_active_step(state)
        state = settle_existing_dispatch(
            dispatched,
            authorization,
            first_basis,
            disposition=StepDisposition.COMPLETE,
        )
        state, planning_ticket = with_parallel_ticket(state)
        dispatched, authorization = dispatch_active_step(state)
        final_basis = successor(dispatched, controller_version=3)

        waiting = settle_existing_dispatch(
            dispatched,
            authorization,
            final_basis,
            disposition=StepDisposition.COMPLETE,
        )

        self.assertEqual(waiting.phase, AgentPhase.PLANNING)
        self.assertFalse(waiting.compile_pending)
        self.assertIsNone(waiting.plan)
        self.assertIsNone(waiting.prepared_intent_plan)
        self.assertEqual(
            waiting.planning_ticket.ticket_id,
            planning_ticket.ticket_id,
        )
        self.assertTrue(waiting.planning_ticket.consumed)

        prepared = prepared_intent_plan(waiting, assigned_goal)
        prepared_state = reduce_physical_agent_state(
            waiting,
            IntentPrepared(prepared),
        )
        accepted = reduce_physical_agent_state(
            prepared_state,
            PreparedIntentAccepted(prepared, 240),
        )
        self.assertEqual(accepted.phase, AgentPhase.EXECUTING)
        self.assertEqual(accepted.plan, prepared.plan)

    def test_expiry_is_explicit_bounded_and_clears_ticket(self):
        state, prepared, authorization, _goal = self._prepared_during_dispatch()
        resulting_basis = successor(state)
        settled = settle_existing_dispatch(state, authorization, resulting_basis)

        with self.assertRaisesRegex(
            PhysicalAgentStateError,
            "prepared intent expired at",
        ):
            reduce_physical_agent_state(
                settled,
                PreparedIntentExpired(
                    prepared,
                    prepared.valid_until_ms - 1,
                ),
            )
        expired = reduce_physical_agent_state(
            settled,
            PreparedIntentExpired(prepared, prepared.valid_until_ms),
        )
        self.assertIsNone(expired.prepared_intent_plan)
        self.assertIsNone(expired.planning_ticket)
        self.assertEqual(expired.plan, settled.plan)

    def test_stale_full_object_cannot_accept_reused_ids_and_basis(self):
        state, first, authorization, assigned_goal = (
            self._prepared_during_dispatch()
        )
        state = settle_existing_dispatch(
            state,
            authorization,
            successor(state),
        )
        state = reduce_physical_agent_state(
            state,
            PreparedIntentExpired(first, first.valid_until_ms),
        )
        state, _ticket = with_parallel_ticket(
            state,
            valid_until_ms=3_000,
        )
        second = prepared_intent_plan(
            state,
            assigned_goal,
            proposal_id=first.proposal_id,
            plan_id="different-plan",
            prepared_at_ms=1_620,
            valid_until_ms=2_000,
        )
        state = reduce_physical_agent_state(state, IntentPrepared(second))

        with self.assertRaises(PhysicalAgentStateError) as caught:
            reduce_physical_agent_state(
                state,
                PreparedIntentAccepted(first, 1_640),
            )

        self.assertEqual(caught.exception.code, "prepared_intent_mismatch")
        self.assertEqual(state.prepared_intent_plan, second)
        with self.assertRaises(PhysicalAgentStateError) as expired:
            reduce_physical_agent_state(
                state,
                PreparedIntentExpired(first, 1_640),
            )
        self.assertEqual(
            expired.exception.code,
            "prepared_intent_mismatch",
        )
        self.assertEqual(state.prepared_intent_plan, second)


if __name__ == "__main__":
    unittest.main()
