"""Validated public contracts for one-shot physical intent coordination."""

from dataclasses import dataclass
from enum import Enum
import json
from typing import Mapping, Optional, Tuple

from .navigation_intent_proposal import (
    DETOUR_TARGET,
    NavigationIntentProposal,
    SCAN_TARGET,
)
from .physical_agent_state import (
    ActiveIntent,
    ExecutionPlan,
    NavigationBasis,
    PhysicalAgentState,
    PlanningTicket,
)


_MAX_INT = 2**63 - 1
MAX_COMPILATION_EVIDENCE_BYTES = 256 * 1024
MAX_TARGET_GEOMETRY_SIGNATURES = 256


class PhysicalIntentCoordinatorError(ValueError):
    """The coordinator configuration or a host dependency is invalid."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


class CoordinatorOutcome(str, Enum):
    NO_WORK = "NO_WORK"
    TICKET_EXPIRED = "TICKET_EXPIRED"
    INTENT_ACCEPTED = "INTENT_ACCEPTED"
    HELD = "HELD"
    ABORTED = "ABORTED"
    PLANNER_FAILED = "PLANNER_FAILED"
    PROPOSAL_REJECTED = "PROPOSAL_REJECTED"
    EVIDENCE_FAILED = "EVIDENCE_FAILED"
    COMPILER_FAILED = "COMPILER_FAILED"
    HOST_FAILED = "HOST_FAILED"
    SUPERSEDED = "SUPERSEDED"


def _identifier(name: str, value: object, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise PhysicalIntentCoordinatorError(
            "invalid_{}".format(name),
            "{} is invalid".format(name.replace("_", " ")),
        )
    return value


def _integer(name: str, value: object) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= _MAX_INT
    ):
        raise PhysicalIntentCoordinatorError(
            "invalid_{}".format(name),
            "{} is invalid".format(name.replace("_", " ")),
        )
    return value


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


def _canonical_snapshot(value: object) -> bytes:
    if not isinstance(value, Mapping):
        raise PhysicalIntentCoordinatorError(
            "invalid_compilation_snapshot",
            "compilation snapshot must be an object",
        )
    try:
        encoded = json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    except (RecursionError, TypeError, ValueError):
        raise PhysicalIntentCoordinatorError(
            "invalid_compilation_snapshot",
            "compilation snapshot must contain finite JSON values",
        ) from None
    if not encoded or len(encoded) > MAX_COMPILATION_EVIDENCE_BYTES:
        raise PhysicalIntentCoordinatorError(
            "compilation_snapshot_too_large",
            "compilation snapshot exceeds its bounded size",
        )
    return encoded


def _decode_canonical_snapshot(raw: bytes) -> Mapping[str, object]:
    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > MAX_COMPILATION_EVIDENCE_BYTES
    ):
        raise PhysicalIntentCoordinatorError(
            "invalid_compilation_snapshot",
            "compilation snapshot bytes are invalid",
        )
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (
        RecursionError,
        UnicodeDecodeError,
        TypeError,
        ValueError,
    ):
        raise PhysicalIntentCoordinatorError(
            "invalid_compilation_snapshot",
            "compilation snapshot is not strict JSON",
        ) from None
    if not isinstance(value, dict) or _canonical_snapshot(value) != raw:
        raise PhysicalIntentCoordinatorError(
            "noncanonical_compilation_snapshot",
            "compilation snapshot must use canonical JSON encoding",
        )
    return value


def _geometry_signatures(
    values: Tuple[Tuple[str, str], ...],
) -> Tuple[Tuple[str, str], ...]:
    if (
        not isinstance(values, tuple)
        or len(values) > MAX_TARGET_GEOMETRY_SIGNATURES
        or any(
            not isinstance(item, tuple)
            or len(item) != 2
            or _identifier("target_id", item[0]) != item[0]
            or _identifier("target_geometry_signature", item[1], 256)
            != item[1]
            for item in values
        )
        or tuple(sorted(set(values))) != values
    ):
        raise PhysicalIntentCoordinatorError(
            "invalid_target_geometry_signatures",
            "target geometry signatures must be sorted and unique",
        )
    return values


@dataclass(frozen=True)
class IntentCompilationEvidence:
    """Immutable world evidence captured for exactly one navigation basis."""

    basis: NavigationBasis
    snapshot_json: bytes
    target_geometry_signatures: Tuple[Tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.basis, NavigationBasis):
            raise PhysicalIntentCoordinatorError(
                "invalid_compilation_basis",
                "compilation evidence requires NavigationBasis",
            )
        _decode_canonical_snapshot(self.snapshot_json)
        _geometry_signatures(self.target_geometry_signatures)

    @classmethod
    def capture(
        cls,
        *,
        basis: NavigationBasis,
        snapshot: Mapping[str, object],
        target_geometry_signatures: Tuple[Tuple[str, str], ...] = (),
    ) -> "IntentCompilationEvidence":
        return cls(
            basis=basis,
            snapshot_json=_canonical_snapshot(snapshot),
            target_geometry_signatures=target_geometry_signatures,
        )

    def snapshot(self) -> Mapping[str, object]:
        """Return a fresh decoded copy; shared mutable data never escapes."""

        return _decode_canonical_snapshot(self.snapshot_json)

    def assert_covers(self, proposal: NavigationIntentProposal) -> None:
        if not isinstance(proposal, NavigationIntentProposal):
            raise PhysicalIntentCoordinatorError(
                "invalid_intent_proposal",
                "proposal must be NavigationIntentProposal",
            )
        if proposal.intent in (SCAN_TARGET, DETOUR_TARGET):
            target_ids = {
                target_id
                for target_id, _signature in self.target_geometry_signatures
            }
            if proposal.target_id not in target_ids:
                raise PhysicalIntentCoordinatorError(
                    "missing_target_geometry_signature",
                    "target intent lacks captured geometry evidence",
                )


@dataclass(frozen=True)
class IntentPlanningRequest:
    """Host-side planner request; it is not the model-visible payload."""

    proposal_id: str
    state: PhysicalAgentState
    ticket: PlanningTicket

    def __post_init__(self) -> None:
        _identifier("proposal_id", self.proposal_id)
        if not isinstance(self.state, PhysicalAgentState):
            raise PhysicalIntentCoordinatorError(
                "invalid_planning_state",
                "state must be PhysicalAgentState",
            )
        if not isinstance(self.ticket, PlanningTicket) or not self.ticket.consumed:
            raise PhysicalIntentCoordinatorError(
                "invalid_planning_ticket",
                "planner request requires a consumed PlanningTicket",
            )
        if self.state.planning_ticket != self.ticket:
            raise PhysicalIntentCoordinatorError(
                "planning_ticket_mismatch",
                "planner request ticket is not current",
            )


@dataclass(frozen=True)
class IntentCompilationRequest:
    """Typed deterministic input to the injected plan compiler."""

    state: PhysicalAgentState
    intent: ActiveIntent
    proposal: NavigationIntentProposal
    evidence: IntentCompilationEvidence
    plan_id: str
    plan_revision: int
    created_at_ms: int

    def __post_init__(self) -> None:
        if not isinstance(self.state, PhysicalAgentState):
            raise PhysicalIntentCoordinatorError(
                "invalid_compilation_state",
                "state must be PhysicalAgentState",
            )
        if not isinstance(self.intent, ActiveIntent):
            raise PhysicalIntentCoordinatorError(
                "invalid_active_intent",
                "intent must be ActiveIntent",
            )
        if not isinstance(self.proposal, NavigationIntentProposal):
            raise PhysicalIntentCoordinatorError(
                "invalid_intent_proposal",
                "proposal must be NavigationIntentProposal",
            )
        if not isinstance(self.evidence, IntentCompilationEvidence):
            raise PhysicalIntentCoordinatorError(
                "invalid_compilation_evidence",
                "evidence must be IntentCompilationEvidence",
            )
        if self.evidence.basis != self.state.basis:
            raise PhysicalIntentCoordinatorError(
                "compilation_evidence_basis_mismatch",
                "evidence must exactly match the compilation state basis",
            )
        self.evidence.assert_covers(self.proposal)
        _identifier("plan_id", self.plan_id)
        _integer("plan_revision", self.plan_revision)
        if self.plan_revision < 1:
            raise PhysicalIntentCoordinatorError(
                "invalid_plan_revision",
                "plan revision must be positive",
            )
        _integer("created_at_ms", self.created_at_ms)


@dataclass(frozen=True)
class PhysicalIntentCoordinatorResult:
    outcome: CoordinatorOutcome
    state: PhysicalAgentState
    ticket_id: Optional[str] = None
    proposal_id: Optional[str] = None
    error_code: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.outcome, CoordinatorOutcome):
            raise PhysicalIntentCoordinatorError(
                "invalid_coordinator_outcome",
                "outcome is invalid",
            )
        if not isinstance(self.state, PhysicalAgentState):
            raise PhysicalIntentCoordinatorError(
                "invalid_result_state",
                "state must be PhysicalAgentState",
            )
        if self.ticket_id is not None:
            _identifier("ticket_id", self.ticket_id)
        if self.proposal_id is not None:
            _identifier("proposal_id", self.proposal_id)
        if self.error_code is not None:
            _identifier("error_code", self.error_code, 160)


__all__ = (
    "CoordinatorOutcome",
    "IntentCompilationEvidence",
    "IntentCompilationRequest",
    "IntentPlanningRequest",
    "PhysicalIntentCoordinatorError",
    "PhysicalIntentCoordinatorResult",
    "MAX_COMPILATION_EVIDENCE_BYTES",
    "MAX_TARGET_GEOMETRY_SIGNATURES",
)
