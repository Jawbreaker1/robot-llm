"""Private command-authorization and controller-receipt contracts."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from ._physical_agent_core import (
    ControllerKey,
    PhysicalAgentStateError,
    _identifier,
    _integer,
)


MAX_STEP_COMMAND_START_TTL_MS = 60_000
MAX_STEP_COMMAND_SETTLE_MS = 30 * 60_000


class ReceiptOutcome(str, Enum):
    COMPLETED = "COMPLETED"
    REJECTED_NOT_STARTED = "REJECTED_NOT_STARTED"
    STOPPED = "STOPPED"


class StepDisposition(str, Enum):
    CONTINUE = "CONTINUE"
    COMPLETE = "COMPLETE"
    BLOCKED = "BLOCKED"


@dataclass(frozen=True)
class PlanStepKey:
    plan_id: str
    plan_revision: int
    cursor: int
    step_id: str

    def __post_init__(self) -> None:
        _identifier("step_key_plan_id", self.plan_id)
        _integer("step_key_plan_revision", self.plan_revision, 1)
        _integer("step_key_cursor", self.cursor)
        _identifier("step_key_step_id", self.step_id)


@dataclass(frozen=True)
class StepCommandAuthorization:
    """Host-issued permission to start one exact plan-step command."""

    action_id: str
    command_id: str
    host_dispatch_sequence: int
    controller_key: ControllerKey
    step_key: PlanStepKey
    based_on_navigation_basis_id: str
    based_on_controller_state_version: int
    command_fingerprint: str
    issued_at_ms: int
    valid_until_ms: int

    def __post_init__(self) -> None:
        _identifier("step_action_id", self.action_id)
        _identifier("step_command_id", self.command_id)
        _integer(
            "host_dispatch_sequence",
            self.host_dispatch_sequence,
            1,
        )
        if not isinstance(self.controller_key, ControllerKey):
            raise PhysicalAgentStateError(
                "invalid_command_controller_key",
                "step authorization controller key is invalid",
            )
        if not isinstance(self.step_key, PlanStepKey):
            raise PhysicalAgentStateError(
                "invalid_command_step_key",
                "step authorization plan-step key is invalid",
            )
        _identifier(
            "command_navigation_basis_id",
            self.based_on_navigation_basis_id,
        )
        _integer(
            "command_controller_state_version",
            self.based_on_controller_state_version,
            1,
        )
        _identifier("command_fingerprint", self.command_fingerprint, 256)
        _integer("command_issued_at_ms", self.issued_at_ms)
        _integer("command_valid_until_ms", self.valid_until_ms)
        if (
            self.valid_until_ms <= self.issued_at_ms
            or self.valid_until_ms - self.issued_at_ms
            > MAX_STEP_COMMAND_START_TTL_MS
        ):
            raise PhysicalAgentStateError(
                "invalid_step_command_ttl",
                "command start expiry must be after issue and within 60 seconds",
            )


@dataclass(frozen=True)
class ActiveDispatch:
    """Host-side command lifecycle; ``dispatched_at_ms`` uses the host clock."""

    authorization: StepCommandAuthorization
    dispatched_at_ms: Optional[int] = None
    settle_by_host_ms: Optional[int] = None
    settlement_expired_at_host_ms: Optional[int] = None

    def __post_init__(self) -> None:
        if not isinstance(self.authorization, StepCommandAuthorization):
            raise PhysicalAgentStateError(
                "invalid_active_authorization",
                "active dispatch authorization is invalid",
            )
        if (self.dispatched_at_ms is None) != (self.settle_by_host_ms is None):
            raise PhysicalAgentStateError(
                "incomplete_active_dispatch",
                "dispatch time and settlement deadline must be recorded together",
            )
        if self.dispatched_at_ms is not None:
            _integer(
                "command_dispatched_at_ms",
                self.dispatched_at_ms,
                self.authorization.issued_at_ms,
            )
            if self.dispatched_at_ms >= self.authorization.valid_until_ms:
                raise PhysicalAgentStateError(
                    "step_command_authorization_expired",
                    "command must be dispatched before authorization expiry",
                )
            _integer(
                "command_settle_by_host_ms",
                self.settle_by_host_ms,
                self.dispatched_at_ms + 1,
            )
            if (
                self.settle_by_host_ms - self.dispatched_at_ms
                > MAX_STEP_COMMAND_SETTLE_MS
            ):
                raise PhysicalAgentStateError(
                    "invalid_step_command_settle_deadline",
                    "command settlement deadline exceeds the finite ceiling",
                )
        if self.settlement_expired_at_host_ms is not None:
            if self.settle_by_host_ms is None:
                raise PhysicalAgentStateError(
                    "invalid_step_command_settlement_expiry",
                    "only a dispatched command can expire",
                )
            _integer(
                "command_settlement_expired_at_host_ms",
                self.settlement_expired_at_host_ms,
                self.settle_by_host_ms,
            )

    @property
    def dispatched(self) -> bool:
        return self.dispatched_at_ms is not None

    @property
    def settlement_expired(self) -> bool:
        return self.settlement_expired_at_host_ms is not None


@dataclass(frozen=True)
class ControllerCommandReceipt:
    """Verified adapter result stamped on ingress with the host clock."""

    outcome: ReceiptOutcome
    controller_key: ControllerKey
    step_key: PlanStepKey
    action_id: str
    command_id: str
    host_dispatch_sequence: int
    command_fingerprint: str
    based_on_navigation_basis_id: str
    based_on_controller_state_version: int
    resulting_controller_state_version: int
    received_at_host_ms: int
    stop_confirmed: bool
    code: str

    def __post_init__(self) -> None:
        if not isinstance(self.outcome, ReceiptOutcome):
            raise PhysicalAgentStateError(
                "invalid_receipt_outcome",
                "controller receipt outcome is invalid",
            )
        if not isinstance(self.controller_key, ControllerKey):
            raise PhysicalAgentStateError(
                "invalid_receipt_controller_key",
                "controller receipt key is invalid",
            )
        if not isinstance(self.step_key, PlanStepKey):
            raise PhysicalAgentStateError(
                "invalid_receipt_step_key",
                "controller receipt plan-step key is invalid",
            )
        _identifier("receipt_action_id", self.action_id)
        _identifier("receipt_command_id", self.command_id)
        _integer(
            "receipt_host_dispatch_sequence",
            self.host_dispatch_sequence,
            1,
        )
        _identifier("receipt_command_fingerprint", self.command_fingerprint, 256)
        _identifier(
            "receipt_navigation_basis_id",
            self.based_on_navigation_basis_id,
        )
        _integer(
            "receipt_based_controller_state_version",
            self.based_on_controller_state_version,
            1,
        )
        _integer(
            "receipt_resulting_controller_state_version",
            self.resulting_controller_state_version,
            self.based_on_controller_state_version,
        )
        _integer("receipt_received_at_host_ms", self.received_at_host_ms)
        if not isinstance(self.stop_confirmed, bool):
            raise PhysicalAgentStateError(
                "invalid_receipt_stop_confirmation",
                "receipt stop confirmation must be a boolean",
            )
        if self.outcome == ReceiptOutcome.STOPPED and not self.stop_confirmed:
            raise PhysicalAgentStateError(
                "missing_receipt_stop_confirmation",
                "STOPPED receipt must confirm that motion stopped",
            )
        _identifier("receipt_code", self.code, 160)
