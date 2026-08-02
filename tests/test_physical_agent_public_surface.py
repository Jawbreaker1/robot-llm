import pickle
import unittest
from typing import get_type_hints

from robot_agent import _physical_agent_core as core
from robot_agent import _physical_agent_dispatch_contract as dispatch_contract
from robot_agent import _physical_agent_events as events
from robot_agent import _physical_agent_reducer as reducer
from robot_agent import _physical_agent_snapshot as snapshot
from robot_agent import physical_agent_contract as contract
from robot_agent import physical_agent_state as state


EXPECTED_CONTRACT_EXPORTS = (
    "ActiveIntent",
    "ActiveDispatch",
    "AgentPhase",
    "ControllerKey",
    "ControllerCommandReceipt",
    "DetourSide",
    "DetourTargetIntent",
    "ExecutionPlan",
    "FollowDirectionIntent",
    "GoalActivated",
    "GoalAssignment",
    "GoalCompletionRequested",
    "GoalOutcome",
    "GoalTerminal",
    "IntentAccepted",
    "IntentPolicy",
    "IntentProgress",
    "MAX_PLANNING_TICKET_TTL_MS",
    "MAX_STEP_COMMAND_SETTLE_MS",
    "MAX_STEP_COMMAND_START_TTL_MS",
    "NavigationBasis",
    "NavigationBasisUpdated",
    "PhysicalAgentEvent",
    "PhysicalAgentState",
    "PhysicalAgentStateError",
    "PlanBinding",
    "PlanRecompiled",
    "PlanStep",
    "PlanStepKey",
    "PlanningAbortRequested",
    "PlanningCause",
    "PlanningHeld",
    "PlanningRequested",
    "PlanningTicket",
    "PlanningTicketConsumed",
    "PlanningTicketExpired",
    "PrimitiveStep",
    "ReceiptOutcome",
    "ReplanRequested",
    "ScanTargetIntent",
    "SensorStep",
    "StopRequested",
    "StopVerified",
    "StepCommandAuthorization",
    "StepCommandAuthorized",
    "StepCommandDispatched",
    "StepCommandRevoked",
    "StepCommandSettlementExpired",
    "StepCommandSettled",
    "StepDisposition",
    "TerminalCleared",
    "WaypointStep",
)

EXPECTED_STATE_EXPORTS = (
    "ActiveDispatch",
    "ActiveIntent",
    "AgentPhase",
    "ControllerCommandReceipt",
    "ControllerKey",
    "DetourSide",
    "DetourTargetIntent",
    "ExecutionPlan",
    "FollowDirectionIntent",
    "GoalActivated",
    "GoalAssignment",
    "GoalCompletionRequested",
    "GoalOutcome",
    "GoalTerminal",
    "IntentAccepted",
    "IntentPolicy",
    "IntentProgress",
    "MAX_PLANNING_TICKET_TTL_MS",
    "MAX_STEP_COMMAND_SETTLE_MS",
    "MAX_STEP_COMMAND_START_TTL_MS",
    "NavigationBasis",
    "NavigationBasisUpdated",
    "PhysicalAgentEvent",
    "PhysicalAgentState",
    "PhysicalAgentStateError",
    "PhysicalAgentStateReducer",
    "PlanBinding",
    "PlanRecompiled",
    "PlanStep",
    "PlanStepKey",
    "PlanningAbortRequested",
    "PlanningCause",
    "PlanningHeld",
    "PlanningRequested",
    "PlanningTicket",
    "PlanningTicketConsumed",
    "PlanningTicketExpired",
    "PrimitiveStep",
    "ReceiptOutcome",
    "ReplanRequested",
    "ScanTargetIntent",
    "SensorStep",
    "StopRequested",
    "StopVerified",
    "StepCommandAuthorization",
    "StepCommandAuthorized",
    "StepCommandDispatched",
    "StepCommandRevoked",
    "StepCommandSettlementExpired",
    "StepCommandSettled",
    "StepDisposition",
    "TerminalCleared",
    "WaypointStep",
    "reduce_physical_agent_state",
)


class PhysicalAgentPublicSurfaceTests(unittest.TestCase):
    def test_export_names_and_order_remain_compatible(self):
        self.assertEqual(contract.__all__, EXPECTED_CONTRACT_EXPORTS)
        self.assertEqual(state.__all__, EXPECTED_STATE_EXPORTS)

    def test_every_contract_export_is_the_private_definition_object(self):
        owners = (core, dispatch_contract, events, snapshot)

        for name in contract.__all__:
            public_value = getattr(contract, name)
            definitions = tuple(
                getattr(owner, name)
                for owner in owners
                if hasattr(owner, name)
            )
            self.assertTrue(definitions, name)
            for definition in definitions:
                self.assertIs(public_value, definition, name)

    def test_every_shared_state_export_is_the_contract_object(self):
        contract_exports = set(contract.__all__)

        for name in state.__all__:
            if name in contract_exports:
                self.assertIs(getattr(state, name), getattr(contract, name), name)

        self.assertIs(
            state.reduce_physical_agent_state,
            reducer.reduce_physical_agent_state,
        )
        self.assertEqual(
            state.PhysicalAgentStateReducer.__module__,
            state.__name__,
        )

    def test_historical_public_class_and_function_paths_are_preserved(self):
        for name in contract.__all__:
            value = getattr(contract, name)
            if isinstance(value, type):
                self.assertEqual(value.__module__, contract.__name__, name)

        self.assertEqual(
            state.reduce_physical_agent_state.__module__,
            state.__name__,
        )
        self.assertEqual(
            state.PhysicalAgentStateReducer.__module__,
            state.__name__,
        )

    def test_public_type_hints_resolve_through_the_contract_facade(self):
        for name in contract.__all__:
            value = getattr(contract, name)
            if isinstance(value, type):
                with self.subTest(name=name):
                    get_type_hints(value)

        state_hints = get_type_hints(contract.PhysicalAgentState)
        receipt_hints = get_type_hints(contract.ControllerCommandReceipt)

        self.assertIs(state_hints["controller_key"], contract.ControllerKey)
        self.assertIs(
            receipt_hints["controller_key"],
            contract.ControllerKey,
        )
        self.assertIs(receipt_hints["step_key"], contract.PlanStepKey)

    def test_public_value_objects_pickle_through_historical_paths(self):
        key = contract.ControllerKey(
            robot_id="robot-1",
            controller_id="controller-1",
            controller_instance_id="boot-1",
        )
        snapshot_value = contract.PhysicalAgentState(controller_key=key)

        restored_key = pickle.loads(pickle.dumps(key))
        restored_snapshot = pickle.loads(pickle.dumps(snapshot_value))

        self.assertIs(type(restored_key), contract.ControllerKey)
        self.assertEqual(restored_key, key)
        self.assertIs(type(restored_snapshot), contract.PhysicalAgentState)
        self.assertEqual(restored_snapshot, snapshot_value)


if __name__ == "__main__":
    unittest.main()
