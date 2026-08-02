"""Private immutable snapshot for one canonical physical agent."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ._physical_agent_core import (
    ActiveIntent,
    AgentPhase,
    ControllerKey,
    ExecutionPlan,
    GoalAssignment,
    GoalTerminal,
    IntentProgress,
    NavigationBasis,
    PhysicalAgentStateError,
    PlanningTicket,
    _integer,
)
from ._physical_agent_dispatch_contract import ActiveDispatch
from ._physical_agent_prepared_contract import PreparedIntentPlan


@dataclass(frozen=True)
class PhysicalAgentState:
    controller_key: ControllerKey
    agent_state_version: int = 1
    phase: AgentPhase = AgentPhase.IDLE
    goal_epoch: int = 0
    plan_revision: int = 0
    last_host_dispatch_sequence: int = 0
    compile_pending: bool = False
    goal: Optional[GoalAssignment] = None
    basis: Optional[NavigationBasis] = None
    intent: Optional[ActiveIntent] = None
    intent_progress: Optional[IntentProgress] = None
    plan: Optional[ExecutionPlan] = None
    active_dispatch: Optional[ActiveDispatch] = None
    planning_ticket: Optional[PlanningTicket] = None
    terminal: Optional[GoalTerminal] = None
    prepared_intent_plan: Optional[PreparedIntentPlan] = None

    def __post_init__(self) -> None:
        if not isinstance(self.controller_key, ControllerKey):
            raise PhysicalAgentStateError(
                "invalid_state_controller_key",
                "state controller key is invalid",
            )
        _integer("agent_state_version", self.agent_state_version, 1)
        if not isinstance(self.phase, AgentPhase):
            raise PhysicalAgentStateError(
                "invalid_agent_phase",
                "agent phase is invalid",
            )
        _integer("state_goal_epoch", self.goal_epoch)
        _integer("state_plan_revision", self.plan_revision)
        _integer(
            "state_last_host_dispatch_sequence",
            self.last_host_dispatch_sequence,
        )
        if not isinstance(self.compile_pending, bool):
            raise PhysicalAgentStateError(
                "invalid_compile_pending",
                "compile-pending marker must be a boolean",
            )
        if self.goal is not None and not isinstance(self.goal, GoalAssignment):
            raise PhysicalAgentStateError("invalid_state_goal", "state goal is invalid")
        if self.basis is not None and not isinstance(self.basis, NavigationBasis):
            raise PhysicalAgentStateError("invalid_state_basis", "state basis is invalid")
        if self.intent is not None and not isinstance(self.intent, ActiveIntent):
            raise PhysicalAgentStateError("invalid_state_intent", "state intent is invalid")
        if self.intent_progress is not None and not isinstance(
            self.intent_progress, IntentProgress
        ):
            raise PhysicalAgentStateError(
                "invalid_state_intent_progress", "state intent progress is invalid"
            )
        if self.plan is not None and not isinstance(self.plan, ExecutionPlan):
            raise PhysicalAgentStateError("invalid_state_plan", "state plan is invalid")
        if self.prepared_intent_plan is not None and not isinstance(
            self.prepared_intent_plan,
            PreparedIntentPlan,
        ):
            raise PhysicalAgentStateError(
                "invalid_state_prepared_intent_plan",
                "state prepared intent plan is invalid",
            )
        if self.active_dispatch is not None and not isinstance(
            self.active_dispatch,
            ActiveDispatch,
        ):
            raise PhysicalAgentStateError(
                "invalid_state_active_dispatch",
                "state active dispatch is invalid",
            )
        if self.planning_ticket is not None and not isinstance(
            self.planning_ticket, PlanningTicket
        ):
            raise PhysicalAgentStateError(
                "invalid_state_planning_ticket",
                "state planning ticket is invalid",
            )
        if self.terminal is not None and not isinstance(self.terminal, GoalTerminal):
            raise PhysicalAgentStateError(
                "invalid_state_terminal",
                "state terminal outcome is invalid",
            )
        self._validate_phase_shape()

    def _validate_common_active(self) -> None:
        if self.goal is None or self.basis is None:
            raise PhysicalAgentStateError(
                "active_state_missing_goal_basis",
                "active state requires a goal and navigation basis",
            )
        if (
            self.goal.goal_epoch != self.goal_epoch
            or self.basis.goal_epoch != self.goal_epoch
            or self.basis.controller_key != self.controller_key
        ):
            raise PhysicalAgentStateError(
                "active_state_binding_mismatch",
                "active state identity or goal epoch is inconsistent",
            )
        if self.intent is not None and (
            self.intent.goal_id != self.goal.goal_id
            or self.intent.goal_epoch != self.goal_epoch
            or self.intent.accepted_basis.controller_key != self.controller_key
        ):
            raise PhysicalAgentStateError(
                "state_intent_binding_mismatch",
                "active intent is not bound to the current goal and controller",
            )
        if (self.intent is None) != (self.intent_progress is None):
            raise PhysicalAgentStateError(
                "intent_progress_binding_mismatch",
                "intent and intent progress must be present together",
            )
        if self.intent_progress is not None and (
            self.intent_progress.plan_attempts > self.intent.policy.max_plan_attempts
            or self.intent_progress.consecutive_no_progress_plans
            > self.intent.policy.max_consecutive_no_progress_plans
        ):
            raise PhysicalAgentStateError(
                "intent_budget_exceeded",
                "intent progress exceeds its host-owned policy",
            )
        if self.planning_ticket is not None and (
            not self.planning_ticket.basis.decision_equivalent(self.basis)
            or self.planning_ticket.basis.goal_epoch != self.goal_epoch
        ):
            raise PhysicalAgentStateError(
                "planning_ticket_basis_mismatch",
                "planning ticket does not match current planning evidence",
            )
        prepared = self.prepared_intent_plan
        if prepared is not None:
            ticket = self.planning_ticket
            if (
                ticket is None
                or not ticket.consumed
                or prepared.ticket != ticket
                or not prepared.compilation_basis.decision_equivalent(
                    self.basis
                )
                or prepared.prepared_at_ms < ticket.consumed_at_ms
                or prepared.valid_until_ms > ticket.valid_until_ms
                or prepared.intent.goal_id != self.goal.goal_id
                or prepared.intent.goal_epoch != self.goal_epoch
                or prepared.plan.revision != self.plan_revision + 1
            ):
                raise PhysicalAgentStateError(
                    "prepared_intent_state_mismatch",
                    "prepared intent plan is not bound to current planning state",
                )
            self.basis.assert_successor_of(prepared.compilation_basis)
            if self.intent is None:
                valid_revision = prepared.intent.revision == 1
            elif prepared.intent.intent_id == self.intent.intent_id:
                valid_revision = (
                    prepared.intent.revision == self.intent.revision + 1
                )
            else:
                valid_revision = prepared.intent.revision == 1
            if not valid_revision:
                raise PhysicalAgentStateError(
                    "invalid_intent_revision",
                    "prepared intent revision is invalid",
                )
            prepared.plan.binding.assert_matches(
                controller_key=self.controller_key,
                goal=self.goal,
                intent=prepared.intent,
                basis=prepared.compilation_basis,
            )

    def _validate_phase_shape(self) -> None:
        if self.phase == AgentPhase.IDLE:
            if any(
                value is not None
                for value in (
                    self.goal,
                    self.basis,
                    self.intent,
                    self.intent_progress,
                    self.plan,
                    self.prepared_intent_plan,
                    self.active_dispatch,
                    self.planning_ticket,
                    self.terminal,
                )
            ) or self.plan_revision != 0 or self.compile_pending:
                raise PhysicalAgentStateError(
                    "invalid_idle_state",
                    "IDLE cannot contain active goal state",
                )
            return

        self._validate_common_active()
        if self.phase == AgentPhase.PLANNING:
            if (
                self.plan is not None
                or self.active_dispatch is not None
                or self.terminal is not None
            ):
                raise PhysicalAgentStateError(
                    "invalid_planning_state",
                    "PLANNING cannot contain a plan or terminal outcome",
                )
            if self.compile_pending and (
                self.intent is None
                or self.intent_progress is None
                or self.planning_ticket is not None
            ):
                raise PhysicalAgentStateError(
                    "invalid_compile_pending_state",
                    "deterministic compilation requires an active intent and no ticket",
                )
            return

        if self.phase == AgentPhase.EXECUTING:
            if (
                self.intent is None
                or self.plan is None
                or self.plan.complete
                or self.terminal is not None
                or self.plan.revision != self.plan_revision
                or self.compile_pending
            ):
                raise PhysicalAgentStateError(
                    "invalid_executing_state",
                    "EXECUTING requires one incomplete authoritative plan",
                )
            self.plan.binding.assert_matches(
                controller_key=self.controller_key,
                goal=self.goal,
                intent=self.intent,
                basis=self.basis,
            )
            if self.active_dispatch is not None:
                authorization = self.active_dispatch.authorization
                active_step = self.plan.active_step
                if (
                    authorization.controller_key != self.controller_key
                    or authorization.host_dispatch_sequence
                    != self.last_host_dispatch_sequence
                    or authorization.step_key.plan_id != self.plan.plan_id
                    or authorization.step_key.plan_revision
                    != self.plan.revision
                    or authorization.step_key.cursor != self.plan.cursor
                    or authorization.step_key.step_id != active_step.step_id
                    or authorization.based_on_controller_state_version
                    > self.basis.controller_state_version
                ):
                    raise PhysicalAgentStateError(
                        "active_dispatch_binding_mismatch",
                        "active dispatch is not bound to the current plan step",
                    )
            return

        if self.phase == AgentPhase.STOPPING:
            if (
                self.plan is not None
                or self.prepared_intent_plan is not None
                or self.planning_ticket is not None
                or self.terminal is None
                or self.compile_pending
                or (
                    self.active_dispatch is not None
                    and (
                        not self.active_dispatch.dispatched
                        or self.active_dispatch.authorization.controller_key
                        != self.controller_key
                        or self.active_dispatch.authorization.host_dispatch_sequence
                        != self.last_host_dispatch_sequence
                    )
                )
            ):
                raise PhysicalAgentStateError(
                    "invalid_stopping_state",
                    "STOPPING requires one pending terminal outcome and no plan",
                )
            return

        if self.phase == AgentPhase.TERMINAL:
            if (
                self.intent is not None
                or self.intent_progress is not None
                or self.plan is not None
                or self.prepared_intent_plan is not None
                or self.active_dispatch is not None
                or self.planning_ticket is not None
                or self.terminal is None
                or self.compile_pending
            ):
                raise PhysicalAgentStateError(
                    "invalid_terminal_state",
                    "TERMINAL cannot contain active intent or plan state",
                )
            return

        raise PhysicalAgentStateError("invalid_agent_phase", "agent phase is invalid")
