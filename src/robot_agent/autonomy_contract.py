"""Strict, motion-free contracts for idle autonomy interest selection.

The model sees typed observations and opaque, host-created opportunities.  It
may select one candidate identifier or decline.  Coordinates, goal epochs,
motion settings, authority and execution tools are intentionally absent from
this boundary.
"""

from dataclasses import dataclass
import json
from typing import Mapping, Optional, Tuple

from .navigation_contract import (
    NavigationContractError,
    identifier,
    integer,
)


AUTONOMY_CONTEXT_SCHEMA = "robot-autonomy-interest-context/v1"
AUTONOMY_SELECTION_SCHEMA = "robot-autonomy-interest-selection/v1"
MAX_AUTONOMY_SELECTION_BYTES = 16 * 1024
MAX_INTEREST_OBSERVATIONS = 16
MAX_EXPLORATION_CANDIDATES = 8
MAX_LINKED_OBSERVATIONS = 8

EXPLORE_SPACE = "EXPLORE_SPACE"
INVESTIGATE_OBSERVATION = "INVESTIGATE_OBSERVATION"
FORWARD = "FORWARD"
LEFT = "LEFT"
RIGHT = "RIGHT"
ROBOT_BASE_FRAME = "ROBOT_BASE"


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


def _optional_identifier(
    name: str,
    value: Optional[str],
    maximum: int = 128,
) -> Optional[str]:
    if value is not None:
        identifier(name, value, maximum)
    return value


@dataclass(frozen=True)
class InterestObservation:
    """A generic typed fact transition from a trusted host producer.

    Integer values deliberately carry their unit separately.  This supports
    metric range now and later light, audio-energy or vision scores without
    adding language-specific text interpretation to the control path.
    """

    observation_id: str
    producer_id: str
    subject_robot_id: str
    controller_instance_id: str
    frame_id: str
    modality: str
    kind: str
    channel: str
    observed_at_ms: int
    received_at_host_ms: int
    valid_until_host_ms: int
    state_version: int
    world_model_version: int
    confidence_milli: int
    clock_domain: str = "source_monotonic"
    previous_value: Optional[int] = None
    current_value: Optional[int] = None
    unit: Optional[str] = None
    previous_subject_id: Optional[str] = None
    current_subject_id: Optional[str] = None

    def __post_init__(self) -> None:
        identifier("observation_id", self.observation_id)
        identifier("producer_id", self.producer_id)
        identifier("subject_robot_id", self.subject_robot_id)
        identifier(
            "controller_instance_id",
            self.controller_instance_id,
        )
        identifier("frame_id", self.frame_id, 96)
        identifier("modality", self.modality, 64)
        identifier("kind", self.kind, 64)
        identifier("channel", self.channel, 64)
        integer("observed_at_ms", self.observed_at_ms, 0, 2**63 - 1)
        integer(
            "received_at_host_ms",
            self.received_at_host_ms,
            0,
            2**63 - 2,
        )
        integer(
            "valid_until_host_ms",
            self.valid_until_host_ms,
            self.received_at_host_ms + 1,
            2**63 - 1,
        )
        integer("state_version", self.state_version, 1, 2**63 - 1)
        integer(
            "world_model_version",
            self.world_model_version,
            1,
            2**63 - 1,
        )
        integer("confidence_milli", self.confidence_milli, 0, 1_000)
        identifier("clock_domain", self.clock_domain, 64)
        for name, value in (
            ("previous_value", self.previous_value),
            ("current_value", self.current_value),
        ):
            if value is not None:
                integer(name, value, -(2**63), 2**63 - 1)
        _optional_identifier("unit", self.unit, 64)
        _optional_identifier(
            "previous_subject_id",
            self.previous_subject_id,
        )
        _optional_identifier(
            "current_subject_id",
            self.current_subject_id,
        )
        has_value = (
            self.previous_value is not None
            or self.current_value is not None
        )
        has_subject = (
            self.previous_subject_id is not None
            or self.current_subject_id is not None
        )
        if not has_value and not has_subject:
            raise NavigationContractError(
                "empty_interest_observation",
                "Interest observation must contain a value or subject",
            )
        if has_value != (self.unit is not None):
            raise NavigationContractError(
                "invalid_observation_unit",
                "Numeric observations require exactly one unit",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "observation_id": self.observation_id,
            "producer_id": self.producer_id,
            "subject_robot_id": self.subject_robot_id,
            "controller_instance_id": self.controller_instance_id,
            "frame_id": self.frame_id,
            "modality": self.modality,
            "kind": self.kind,
            "channel": self.channel,
            "observed_at_ms": self.observed_at_ms,
            "received_at_host_ms": self.received_at_host_ms,
            "valid_until_host_ms": self.valid_until_host_ms,
            "clock_domain": self.clock_domain,
            "based_on_state_version": self.state_version,
            "based_on_world_model_version": self.world_model_version,
            "confidence_milli": self.confidence_milli,
            "previous_value": self.previous_value,
            "current_value": self.current_value,
            "unit": self.unit,
            "previous_subject_id": self.previous_subject_id,
            "current_subject_id": self.current_subject_id,
        }


@dataclass(frozen=True)
class ExplorationCandidate:
    """Model-visible view of one host-resolved waypoint opportunity."""

    candidate_id: str
    task_kind: str
    relative_direction: str
    estimated_travel_mm: int
    attempted_visits: int
    completed_visits: int
    linked_observation_ids: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        identifier("candidate_id", self.candidate_id)
        if self.task_kind not in (
            EXPLORE_SPACE,
            INVESTIGATE_OBSERVATION,
        ):
            raise NavigationContractError(
                "invalid_autonomy_task_kind",
                "Autonomy task kind is invalid",
            )
        if self.relative_direction not in (FORWARD, LEFT, RIGHT):
            raise NavigationContractError(
                "invalid_relative_direction",
                "Candidate relative direction is invalid",
            )
        integer(
            "estimated_travel_mm",
            self.estimated_travel_mm,
            1,
            10_000,
        )
        integer(
            "attempted_visits",
            self.attempted_visits,
            0,
            1_000_000,
        )
        integer(
            "completed_visits",
            self.completed_visits,
            0,
            1_000_000,
        )
        if self.completed_visits > self.attempted_visits:
            raise NavigationContractError(
                "invalid_exploration_visit_counts",
                "Completed visits cannot exceed attempted visits",
            )
        if (
            not isinstance(self.linked_observation_ids, tuple)
            or len(self.linked_observation_ids)
            > MAX_LINKED_OBSERVATIONS
            or any(
                identifier("linked_observation_id", value) != value
                for value in self.linked_observation_ids
            )
            or len(set(self.linked_observation_ids))
            != len(self.linked_observation_ids)
        ):
            raise NavigationContractError(
                "invalid_linked_observations",
                "Candidate observation links are invalid",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "candidate_id": self.candidate_id,
            "task_kind": self.task_kind,
            "relative_direction": self.relative_direction,
            "estimated_travel_mm": self.estimated_travel_mm,
            "attempted_visits": self.attempted_visits,
            "completed_visits": self.completed_visits,
            "linked_observation_ids": list(
                self.linked_observation_ids
            ),
        }


@dataclass(frozen=True)
class InterestSelectionProposal:
    """Untrusted model selection with exact host-context bindings."""

    proposal_id: str
    robot_id: str
    controller_instance_id: str
    autonomy_session_id: str
    lease_generation: int
    candidate_set_id: str
    based_on_state_version: int
    based_on_world_model_version: int
    decision: str
    confidence_milli: int
    selected_candidate_id: Optional[str] = None
    reason_code: Optional[str] = None

    def __post_init__(self) -> None:
        identifier("proposal_id", self.proposal_id)
        identifier("robot_id", self.robot_id)
        identifier(
            "controller_instance_id",
            self.controller_instance_id,
        )
        identifier("autonomy_session_id", self.autonomy_session_id)
        integer(
            "lease_generation",
            self.lease_generation,
            1,
            2**63 - 1,
        )
        identifier("candidate_set_id", self.candidate_set_id)
        integer(
            "based_on_state_version",
            self.based_on_state_version,
            1,
            2**63 - 1,
        )
        integer(
            "based_on_world_model_version",
            self.based_on_world_model_version,
            1,
            2**63 - 1,
        )
        integer("confidence_milli", self.confidence_milli, 0, 1_000)
        if self.decision == "SELECT":
            identifier(
                "selected_candidate_id",
                self.selected_candidate_id or "",
            )
            if self.reason_code is not None:
                raise NavigationContractError(
                    "unexpected_selection_reason",
                    "SELECT cannot carry reason_code",
                )
        elif self.decision in ("HOLD", "ABORT"):
            if self.selected_candidate_id is not None:
                raise NavigationContractError(
                    "unexpected_selected_candidate",
                    "{} cannot select a candidate".format(
                        self.decision
                    ),
                )
            identifier("reason_code", self.reason_code or "", 64)
        else:
            raise NavigationContractError(
                "invalid_interest_decision",
                "Decision must be SELECT, HOLD or ABORT",
            )

    def to_dict(self) -> Mapping[str, object]:
        result = {
            "schema": AUTONOMY_SELECTION_SCHEMA,
            "proposal_id": self.proposal_id,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "autonomy_session_id": self.autonomy_session_id,
            "lease_generation": self.lease_generation,
            "candidate_set_id": self.candidate_set_id,
            "based_on_state_version": self.based_on_state_version,
            "based_on_world_model_version": (
                self.based_on_world_model_version
            ),
            "decision": self.decision,
            "confidence_milli": self.confidence_milli,
        }
        if self.decision == "SELECT":
            result["selected_candidate_id"] = (
                self.selected_candidate_id
            )
        else:
            result["reason_code"] = self.reason_code
        return result


@dataclass(frozen=True)
class InterestSelectionContext:
    """Immutable menu and evidence snapshot passed to an interest model."""

    proposal_id: str
    robot_id: str
    controller_instance_id: str
    autonomy_session_id: str
    lease_generation: int
    candidate_set_id: str
    frame_id: str
    state_version: int
    world_model_version: int
    captured_at_ms: int
    valid_until_ms: int
    remaining_tasks: int
    observations: Tuple[InterestObservation, ...]
    candidates: Tuple[ExplorationCandidate, ...]

    def __post_init__(self) -> None:
        identifier("proposal_id", self.proposal_id)
        identifier("robot_id", self.robot_id)
        identifier(
            "controller_instance_id",
            self.controller_instance_id,
        )
        identifier("autonomy_session_id", self.autonomy_session_id)
        integer(
            "lease_generation",
            self.lease_generation,
            1,
            2**63 - 1,
        )
        identifier("candidate_set_id", self.candidate_set_id)
        identifier("frame_id", self.frame_id, 96)
        integer("state_version", self.state_version, 1, 2**63 - 1)
        integer(
            "world_model_version",
            self.world_model_version,
            1,
            2**63 - 1,
        )
        integer("captured_at_ms", self.captured_at_ms, 0, 2**63 - 1)
        integer(
            "valid_until_ms",
            self.valid_until_ms,
            0,
            2**63 - 1,
        )
        if self.valid_until_ms <= self.captured_at_ms:
            raise NavigationContractError(
                "invalid_selection_deadline",
                "Selection deadline must be after capture",
            )
        integer("remaining_tasks", self.remaining_tasks, 1, 100_000)
        if (
            not isinstance(self.observations, tuple)
            or len(self.observations) > MAX_INTEREST_OBSERVATIONS
            or any(
                not isinstance(value, InterestObservation)
                for value in self.observations
            )
        ):
            raise NavigationContractError(
                "invalid_interest_observations",
                "Interest observations are invalid",
            )
        observation_ids = tuple(
            value.observation_id for value in self.observations
        )
        if len(set(observation_ids)) != len(observation_ids):
            raise NavigationContractError(
                "duplicate_interest_observation",
                "Interest observation IDs must be unique",
            )
        if any(
            observation.subject_robot_id != self.robot_id
            or observation.controller_instance_id
            != self.controller_instance_id
            or observation.frame_id != self.frame_id
            or observation.state_version != self.state_version
            or observation.world_model_version
            != self.world_model_version
            or observation.received_at_host_ms > self.captured_at_ms
            or self.captured_at_ms
            >= observation.valid_until_host_ms
            or self.valid_until_ms
            > observation.valid_until_host_ms
            for observation in self.observations
        ):
            raise NavigationContractError(
                "stale_interest_observation",
                "Interest observation does not match its host context",
            )
        if (
            not isinstance(self.candidates, tuple)
            or not 1 <= len(self.candidates)
            <= MAX_EXPLORATION_CANDIDATES
            or any(
                not isinstance(value, ExplorationCandidate)
                for value in self.candidates
            )
        ):
            raise NavigationContractError(
                "invalid_exploration_candidates",
                "Exploration candidates are invalid",
            )
        candidate_ids = tuple(
            value.candidate_id for value in self.candidates
        )
        if len(set(candidate_ids)) != len(candidate_ids):
            raise NavigationContractError(
                "duplicate_exploration_candidate",
                "Exploration candidate IDs must be unique",
            )
        known_observations = set(observation_ids)
        if any(
            not set(candidate.linked_observation_ids).issubset(
                known_observations
            )
            for candidate in self.candidates
        ):
            raise NavigationContractError(
                "unknown_linked_observation",
                "Candidate links an observation outside this context",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": AUTONOMY_CONTEXT_SCHEMA,
            "proposal_id": self.proposal_id,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "autonomy_session_id": self.autonomy_session_id,
            "lease_generation": self.lease_generation,
            "candidate_set_id": self.candidate_set_id,
            "frame_id": self.frame_id,
            "based_on_state_version": self.state_version,
            "based_on_world_model_version": self.world_model_version,
            "captured_at_ms": self.captured_at_ms,
            "valid_until_ms": self.valid_until_ms,
            "remaining_tasks": self.remaining_tasks,
            "observations": [
                value.to_dict() for value in self.observations
            ],
            "candidates": [
                value.to_dict() for value in self.candidates
            ],
        }

    def assert_accepts(
        self,
        proposal: InterestSelectionProposal,
        now_ms: int,
    ) -> None:
        if not isinstance(proposal, InterestSelectionProposal):
            raise NavigationContractError(
                "invalid_interest_selection",
                "Context requires InterestSelectionProposal",
            )
        integer("now_ms", now_ms, 0, 2**63 - 1)
        if now_ms < self.captured_at_ms or now_ms >= self.valid_until_ms:
            raise NavigationContractError(
                "expired_interest_selection",
                "Interest selection is outside its validity window",
            )
        if (
            proposal.proposal_id != self.proposal_id
            or proposal.robot_id != self.robot_id
            or proposal.controller_instance_id
            != self.controller_instance_id
            or proposal.autonomy_session_id
            != self.autonomy_session_id
            or proposal.lease_generation != self.lease_generation
            or proposal.candidate_set_id != self.candidate_set_id
            or proposal.based_on_state_version != self.state_version
            or proposal.based_on_world_model_version
            != self.world_model_version
        ):
            raise NavigationContractError(
                "stale_interest_selection",
                "Interest selection does not match its host context",
            )
        if (
            proposal.decision == "SELECT"
            and proposal.selected_candidate_id
            not in {
                candidate.candidate_id
                for candidate in self.candidates
            }
        ):
            raise NavigationContractError(
                "unknown_selected_candidate",
                "Model selected a candidate outside the host menu",
            )


def decode_interest_selection(raw: bytes) -> InterestSelectionProposal:
    """Strictly decode untrusted model bytes without granting authority."""

    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > MAX_AUTONOMY_SELECTION_BYTES
    ):
        raise NavigationContractError(
            "invalid_interest_selection_body",
            "Interest selection body is invalid",
        )
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, TypeError, ValueError):
        raise NavigationContractError(
            "invalid_interest_selection_json",
            "Interest selector returned invalid JSON",
        ) from None
    common = {
        "schema",
        "proposal_id",
        "robot_id",
        "controller_instance_id",
        "autonomy_session_id",
        "lease_generation",
        "candidate_set_id",
        "based_on_state_version",
        "based_on_world_model_version",
        "decision",
        "confidence_milli",
    }
    if not isinstance(value, dict):
        raise NavigationContractError(
            "invalid_interest_selection_shape",
            "Interest selection must be an object",
        )
    if value.get("schema") != AUTONOMY_SELECTION_SCHEMA:
        raise NavigationContractError(
            "invalid_interest_selection_schema",
            "Interest selection schema is unsupported",
        )
    decision = value.get("decision")
    selected_candidate_id = None
    reason_code = None
    if decision == "SELECT":
        if set(value) != common | {"selected_candidate_id"}:
            raise NavigationContractError(
                "invalid_interest_selection_fields",
                "SELECT fields are invalid",
            )
        selected_candidate_id = value["selected_candidate_id"]
    elif decision in ("HOLD", "ABORT"):
        if set(value) != common | {"reason_code"}:
            raise NavigationContractError(
                "invalid_interest_selection_fields",
                "{} fields are invalid".format(decision),
            )
        reason_code = value["reason_code"]
    else:
        raise NavigationContractError(
            "invalid_interest_decision",
            "Decision must be SELECT, HOLD or ABORT",
        )
    return InterestSelectionProposal(
        proposal_id=value["proposal_id"],
        robot_id=value["robot_id"],
        controller_instance_id=value["controller_instance_id"],
        autonomy_session_id=value["autonomy_session_id"],
        lease_generation=value["lease_generation"],
        candidate_set_id=value["candidate_set_id"],
        based_on_state_version=value["based_on_state_version"],
        based_on_world_model_version=(
            value["based_on_world_model_version"]
        ),
        decision=decision,
        confidence_milli=value["confidence_milli"],
        selected_candidate_id=selected_candidate_id,
        reason_code=reason_code,
    )


__all__ = (
    "AUTONOMY_CONTEXT_SCHEMA",
    "AUTONOMY_SELECTION_SCHEMA",
    "EXPLORE_SPACE",
    "ExplorationCandidate",
    "FORWARD",
    "INVESTIGATE_OBSERVATION",
    "InterestObservation",
    "InterestSelectionContext",
    "InterestSelectionProposal",
    "LEFT",
    "MAX_AUTONOMY_SELECTION_BYTES",
    "MAX_EXPLORATION_CANDIDATES",
    "MAX_INTEREST_OBSERVATIONS",
    "RIGHT",
    "ROBOT_BASE_FRAME",
    "decode_interest_selection",
)
