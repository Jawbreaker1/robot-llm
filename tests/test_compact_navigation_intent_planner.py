import unittest

from robot_agent.compact_navigation_intent_planner import (
    CompactNavigationIntentPlanner,
    CompactNavigationIntentPlannerError,
)
from robot_agent.lm_studio_navigation_intent import (
    LMStudioNavigationIntentResult,
)
from robot_agent.navigation_intent_context import NavigationIntentPrompt
from robot_agent.navigation_intent_proposal import (
    FOLLOW_DIRECTION,
    NavigationIntentOffer,
    NavigationIntentProposal,
    SCAN_TARGET,
    bind_navigation_intent_proposal,
)
from robot_agent.physical_agent_state import (
    AgentPhase,
    ControllerKey,
    GoalAssignment,
    NavigationBasis,
    PhysicalAgentState,
    PlanningCause,
    PlanningTicket,
)
from robot_agent.physical_intent_contract import IntentPlanningRequest


NOW_MS = 2_000


def basis(**changes):
    values = {
        "controller_key": ControllerKey("robot-a", "drive-a", "boot-a"),
        "goal_epoch": 1,
        "controller_state_version": 3,
        "world_generation_id": "world-a",
        "world_model_version": 4,
        "navigation_basis_id": "basis-a",
        "frame_id": "frame-a",
        "calibration_fingerprint": "calibration-a",
    }
    values.update(changes)
    return NavigationBasis(**values)


def planning_request():
    current_basis = basis()
    goal = GoalAssignment(
        goal_id="goal-a",
        goal_epoch=1,
        objective="Move through the room",
        source="USER",
        locale="en",
        activated_at_ms=100,
    )
    ticket = PlanningTicket(
        ticket_id="ticket-a",
        cause=PlanningCause.NEW_GOAL,
        basis=current_basis,
        created_at_ms=110,
        valid_until_ms=10_000,
        consumed_at_ms=120,
    )
    state = PhysicalAgentState(
        controller_key=current_basis.controller_key,
        phase=AgentPhase.PLANNING,
        goal_epoch=1,
        goal=goal,
        basis=current_basis,
        planning_ticket=ticket,
    )
    return IntentPlanningRequest(
        proposal_id="proposal-a",
        state=state,
        ticket=ticket,
    )


def offer(request, **changes):
    values = {
        "ticket_id": request.ticket.ticket_id,
        "basis": request.ticket.basis,
        "offered_intents": (FOLLOW_DIRECTION,),
    }
    values.update(changes)
    return NavigationIntentOffer(**values)


def prompt():
    return NavigationIntentPrompt(
        system_prompt="test",
        context={},
        response_schema={},
        context_bytes=2,
        accounted_bytes=2,
    )


def result(envelope):
    return LMStudioNavigationIntentResult(
        envelope=envelope,
        latency_ms=12,
        served_model="model-a",
        prompt_tokens=10,
        completion_tokens=3,
        total_tokens=13,
        server_tokens_per_second=80.0,
        server_time_to_first_token_seconds=0.1,
        context_bytes=100,
        accounted_bytes=200,
    )


class RecordingClient:
    def __init__(self, returned=None):
        self.returned = returned
        self.calls = []

    def decide(self, built_prompt, *, offer, proposal_id):
        self.calls.append((built_prompt, offer, proposal_id))
        return self.returned


class CompactNavigationIntentPlannerTests(unittest.TestCase):
    def test_single_concrete_choice_is_bound_locally_without_lm_call(self):
        request = planning_request()
        client = RecordingClient()
        prompt_calls = []

        def prompt_builder(*args):
            prompt_calls.append(args)
            raise AssertionError("single choice must not build an LM prompt")

        planner = CompactNavigationIntentPlanner(
            offer_builder=lambda current: offer(current),
            prompt_builder=prompt_builder,
            client=client,
            clock_ms=lambda: NOW_MS,
            proposal_ttl_ms=700,
        )

        envelope = planner(request)

        self.assertEqual(envelope.proposal.intent, FOLLOW_DIRECTION)
        self.assertEqual(envelope.proposal_id, request.proposal_id)
        self.assertEqual(envelope.ticket_id, request.ticket.ticket_id)
        self.assertEqual(envelope.basis, request.ticket.basis)
        self.assertEqual(envelope.received_at_ms, NOW_MS)
        self.assertEqual(envelope.valid_until_ms, NOW_MS + 700)
        self.assertEqual(prompt_calls, [])
        self.assertEqual(client.calls, [])

    def test_multiple_choices_make_exactly_one_client_call(self):
        request = planning_request()
        current_offer = offer(
            request,
            offered_intents=(FOLLOW_DIRECTION, SCAN_TARGET),
            scan_target_ids=("hazard-a",),
        )
        chosen = bind_navigation_intent_proposal(
            NavigationIntentProposal(
                intent=SCAN_TARGET,
                target_id="hazard-a",
            ),
            offer=current_offer,
            proposal_id=request.proposal_id,
            received_at_ms=NOW_MS - 10,
            valid_until_ms=NOW_MS + 500,
        )
        client = RecordingClient(result(chosen))
        built_prompt = prompt()
        prompt_calls = []
        telemetry_calls = []

        def prompt_builder(current_request, value):
            prompt_calls.append((current_request, value))
            return built_prompt

        def broken_telemetry(value):
            telemetry_calls.append(value)
            raise RuntimeError("telemetry is observational")

        planner = CompactNavigationIntentPlanner(
            offer_builder=lambda _request: current_offer,
            prompt_builder=prompt_builder,
            client=client,
            clock_ms=lambda: NOW_MS,
            telemetry=broken_telemetry,
        )

        envelope = planner(request)

        self.assertEqual(envelope, chosen)
        self.assertEqual(prompt_calls, [(request, current_offer)])
        self.assertEqual(
            client.calls,
            [(built_prompt, current_offer, request.proposal_id)],
        )
        self.assertEqual(telemetry_calls, [client.returned])

    def test_mismatched_offer_is_rejected_before_prompt_or_client(self):
        request = planning_request()
        client = RecordingClient()
        prompt_calls = []
        mismatched = offer(
            request,
            ticket_id="different-ticket",
            offered_intents=(FOLLOW_DIRECTION, SCAN_TARGET),
            scan_target_ids=("hazard-a",),
        )
        planner = CompactNavigationIntentPlanner(
            offer_builder=lambda _request: mismatched,
            prompt_builder=lambda *args: prompt_calls.append(args),
            client=client,
            clock_ms=lambda: NOW_MS,
        )

        with self.assertRaises(CompactNavigationIntentPlannerError) as raised:
            planner(request)

        self.assertEqual(raised.exception.code, "offer_binding_mismatch")
        self.assertEqual(prompt_calls, [])
        self.assertEqual(client.calls, [])

    def test_malformed_dependencies_and_results_fail_closed(self):
        request = planning_request()
        current_offer = offer(
            request,
            offered_intents=(FOLLOW_DIRECTION, SCAN_TARGET),
            scan_target_ids=("hazard-a",),
        )

        with self.assertRaises(CompactNavigationIntentPlannerError) as raised:
            CompactNavigationIntentPlanner(
                offer_builder=None,
                prompt_builder=prompt,
                client=RecordingClient(),
            )
        self.assertEqual(raised.exception.code, "invalid_dependency")

        invalid_offer = CompactNavigationIntentPlanner(
            offer_builder=lambda _request: object(),
            prompt_builder=lambda *_args: prompt(),
            client=RecordingClient(),
        )
        with self.assertRaises(CompactNavigationIntentPlannerError) as raised:
            invalid_offer(request)
        self.assertEqual(raised.exception.code, "invalid_offer_result")

        invalid_prompt_client = RecordingClient()
        invalid_prompt = CompactNavigationIntentPlanner(
            offer_builder=lambda _request: current_offer,
            prompt_builder=lambda *_args: object(),
            client=invalid_prompt_client,
        )
        with self.assertRaises(CompactNavigationIntentPlannerError) as raised:
            invalid_prompt(request)
        self.assertEqual(raised.exception.code, "invalid_prompt_result")
        self.assertEqual(invalid_prompt_client.calls, [])

        invalid_result_client = RecordingClient(object())
        invalid_result = CompactNavigationIntentPlanner(
            offer_builder=lambda _request: current_offer,
            prompt_builder=lambda *_args: prompt(),
            client=invalid_result_client,
        )
        with self.assertRaises(CompactNavigationIntentPlannerError) as raised:
            invalid_result(request)
        self.assertEqual(raised.exception.code, "invalid_client_result")
        self.assertEqual(len(invalid_result_client.calls), 1)


if __name__ == "__main__":
    unittest.main()
