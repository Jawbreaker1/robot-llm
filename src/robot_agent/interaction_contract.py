"""Strict, motion-free contracts for object reactions.

Perception code owns :class:`ObjectEvidence` and
:class:`InteractionSnapshot`.  A language model may only return an
:class:`ExpressionProposal`: semantic speech and, at most, the single
allowlisted propeller-wave gesture.  Motor roles, ports, speeds, durations,
TTL, priority, authority and source attribution are deliberately absent from
the model-controlled schema.

The contracts are suitable for asynchronous producers.  Every proposal is
bound to immutable interaction, world-model and obstruction versions, and can
be checked against the current snapshot immediately before it is accepted.
"""

from dataclasses import dataclass
import hashlib
import json
from typing import Mapping, Optional


OBJECT_EVIDENCE_SCHEMA = "robot-object-evidence/v1"
INTERACTION_SNAPSHOT_SCHEMA = "robot-interaction-snapshot/v1"
EXPRESSION_PROPOSAL_SCHEMA = "robot-expression-proposal/v1"
MAX_INTERACTION_JSON_BYTES = 16 * 1024

_MAX_INT = 2**63 - 1
_OBJECT_EVIDENCE_FIELDS = {
    "schema",
    "evidence_id",
    "relation",
    "object_id",
    "source",
    "observed_at_ms",
    "confidence_milli",
}
_INTERACTION_SNAPSHOT_FIELDS = {
    "schema",
    "robot_id",
    "controller_instance_id",
    "goal_id",
    "goal_epoch",
    "plan_revision",
    "interaction_state_version",
    "world_model_version",
    "captured_at_ms",
    "obstruction_epoch",
    "drive_phase",
    "response_locale",
    "evidence",
}
_EXPRESSION_PROPOSAL_COMMON_FIELDS = {
    "schema",
    "proposal_id",
    "robot_id",
    "controller_instance_id",
    "goal_id",
    "goal_epoch",
    "plan_revision",
    "based_on_interaction_state_version",
    "based_on_world_model_version",
    "obstruction_epoch",
    "based_on_evidence_id",
    "decision",
    "confidence_milli",
}
_EXPRESSION_INTENT_FIELDS = {
    "utterance",
    "utterance_locale",
    "gesture_kind",
    "affect_label",
    "intensity",
    "repetitions",
}


class InteractionContractError(ValueError):
    """A safely reportable object-interaction contract violation."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


def _identifier(name: str, value: str, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise InteractionContractError(
            "invalid_identifier",
            "{} is invalid".format(name),
        )
    return value


def _optional_identifier(
    name: str,
    value: Optional[str],
    maximum: int = 128,
) -> Optional[str]:
    if value is not None:
        _identifier(name, value, maximum)
    return value


def _integer(name: str, value: int, minimum: int, maximum: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise InteractionContractError(
            "invalid_integer",
            "{} is invalid".format(name),
        )
    return value


def _utterance(value: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 160
        or any(ord(character) < 32 for character in value)
    ):
        raise InteractionContractError(
            "invalid_utterance",
            "utterance must contain 1 to 160 printable characters",
        )
    return value


def _decode_json_object(raw: bytes, body_name: str) -> Mapping[str, object]:
    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > MAX_INTERACTION_JSON_BYTES
    ):
        raise InteractionContractError(
            "invalid_{}_body".format(body_name),
            "{} body is invalid".format(body_name.replace("_", " ")),
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
        raise InteractionContractError(
            "invalid_{}_json".format(body_name),
            "{} is not strict JSON".format(body_name.replace("_", " ")),
        ) from None
    if not isinstance(value, dict):
        raise InteractionContractError(
            "invalid_{}_shape".format(body_name),
            "{} must be an object".format(body_name.replace("_", " ")),
        )
    return value


@dataclass(frozen=True)
class ObjectEvidence:
    """Host-attributed evidence that an object blocks the current path."""

    evidence_id: str
    relation: str
    object_id: Optional[str]
    source: str
    observed_at_ms: int
    confidence_milli: int

    def __post_init__(self) -> None:
        _identifier("evidence_id", self.evidence_id)
        if self.relation != "BLOCKING_PATH":
            raise InteractionContractError(
                "invalid_object_relation",
                "relation must be BLOCKING_PATH",
            )
        _optional_identifier("object_id", self.object_id)
        _identifier("source", self.source)
        _integer("observed_at_ms", self.observed_at_ms, 0, _MAX_INT)
        _integer("confidence_milli", self.confidence_milli, 0, 1_000)

    def to_dict(self):
        return {
            "schema": OBJECT_EVIDENCE_SCHEMA,
            "evidence_id": self.evidence_id,
            "relation": self.relation,
            "object_id": self.object_id,
            "source": self.source,
            "observed_at_ms": self.observed_at_ms,
            "confidence_milli": self.confidence_milli,
        }


def _object_evidence_from_value(value) -> ObjectEvidence:
    if not isinstance(value, dict) or set(value) != _OBJECT_EVIDENCE_FIELDS:
        raise InteractionContractError(
            "invalid_object_evidence_fields",
            "Object evidence fields are invalid",
        )
    if value["schema"] != OBJECT_EVIDENCE_SCHEMA:
        raise InteractionContractError(
            "invalid_object_evidence_schema",
            "Object evidence schema is not supported",
        )
    return ObjectEvidence(
        evidence_id=value["evidence_id"],
        relation=value["relation"],
        object_id=value["object_id"],
        source=value["source"],
        observed_at_ms=value["observed_at_ms"],
        confidence_milli=value["confidence_milli"],
    )


def decode_object_evidence(raw: bytes) -> ObjectEvidence:
    """Decode strict host-owned object evidence."""

    return _object_evidence_from_value(
        _decode_json_object(raw, "object_evidence")
    )


@dataclass(frozen=True)
class InteractionSnapshot:
    """An immutable interaction view captured at one controller instant."""

    robot_id: str
    controller_instance_id: str
    goal_id: str
    goal_epoch: int
    plan_revision: int
    interaction_state_version: int
    world_model_version: int
    captured_at_ms: int
    obstruction_epoch: int
    drive_phase: str
    response_locale: str
    evidence: Optional[ObjectEvidence] = None

    def __post_init__(self) -> None:
        _identifier("robot_id", self.robot_id)
        _identifier("controller_instance_id", self.controller_instance_id)
        _identifier("goal_id", self.goal_id)
        _integer("goal_epoch", self.goal_epoch, 1, _MAX_INT)
        _integer("plan_revision", self.plan_revision, 1, _MAX_INT)
        _integer(
            "interaction_state_version",
            self.interaction_state_version,
            1,
            _MAX_INT,
        )
        _integer(
            "world_model_version",
            self.world_model_version,
            1,
            _MAX_INT,
        )
        _integer("captured_at_ms", self.captured_at_ms, 0, _MAX_INT)
        _integer("obstruction_epoch", self.obstruction_epoch, 0, _MAX_INT)
        if self.drive_phase not in ("STOPPED", "MOVING", "BLOCKED"):
            raise InteractionContractError(
                "invalid_drive_phase",
                "drive_phase must be STOPPED, MOVING or BLOCKED",
            )
        _identifier("response_locale", self.response_locale, 64)
        if self.evidence is not None:
            if not isinstance(self.evidence, ObjectEvidence):
                raise InteractionContractError(
                    "invalid_object_evidence",
                    "evidence must be ObjectEvidence or null",
                )
            if self.evidence.observed_at_ms > self.captured_at_ms:
                raise InteractionContractError(
                    "future_object_evidence",
                    "evidence cannot postdate its interaction snapshot",
                )
            if self.obstruction_epoch == 0:
                raise InteractionContractError(
                    "missing_obstruction_epoch",
                    "evidence requires a non-zero obstruction epoch",
                )

    def to_dict(self):
        return {
            "schema": INTERACTION_SNAPSHOT_SCHEMA,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "goal_id": self.goal_id,
            "goal_epoch": self.goal_epoch,
            "plan_revision": self.plan_revision,
            "interaction_state_version": self.interaction_state_version,
            "world_model_version": self.world_model_version,
            "captured_at_ms": self.captured_at_ms,
            "obstruction_epoch": self.obstruction_epoch,
            "drive_phase": self.drive_phase,
            "response_locale": self.response_locale,
            "evidence": (
                None if self.evidence is None else self.evidence.to_dict()
            ),
        }


def expression_proposal_id_for_snapshot(
    snapshot: InteractionSnapshot,
) -> str:
    """Return the host-owned proposal ID for one exact snapshot."""

    if not isinstance(snapshot, InteractionSnapshot):
        raise InteractionContractError(
            "invalid_interaction_snapshot",
            "snapshot must be InteractionSnapshot",
        )
    canonical = json.dumps(
        snapshot.to_dict(),
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("ascii")
    return "host-expression-{}".format(
        hashlib.sha256(canonical).hexdigest()
    )


def decode_interaction_snapshot(raw: bytes) -> InteractionSnapshot:
    """Decode a strict, version-bound interaction snapshot."""

    value = _decode_json_object(raw, "interaction_snapshot")
    if set(value) != _INTERACTION_SNAPSHOT_FIELDS:
        raise InteractionContractError(
            "invalid_interaction_snapshot_fields",
            "Interaction snapshot fields are invalid",
        )
    if value["schema"] != INTERACTION_SNAPSHOT_SCHEMA:
        raise InteractionContractError(
            "invalid_interaction_snapshot_schema",
            "Interaction snapshot schema is not supported",
        )
    evidence_value = value["evidence"]
    evidence = (
        None
        if evidence_value is None
        else _object_evidence_from_value(evidence_value)
    )
    return InteractionSnapshot(
        robot_id=value["robot_id"],
        controller_instance_id=value["controller_instance_id"],
        goal_id=value["goal_id"],
        goal_epoch=value["goal_epoch"],
        plan_revision=value["plan_revision"],
        interaction_state_version=value["interaction_state_version"],
        world_model_version=value["world_model_version"],
        captured_at_ms=value["captured_at_ms"],
        obstruction_epoch=value["obstruction_epoch"],
        drive_phase=value["drive_phase"],
        response_locale=value["response_locale"],
        evidence=evidence,
    )


@dataclass(frozen=True)
class ExpressionIntent:
    """Semantic expression only; it is never an executable motor command.

    ``affect_label`` is retained for audit and observability.  Deterministic
    execution code must not derive safety limits or authority from it.
    """

    utterance: str
    utterance_locale: str
    gesture_kind: Optional[str]
    affect_label: str
    intensity: int
    repetitions: int

    def __post_init__(self) -> None:
        _utterance(self.utterance)
        # Locale is intentionally a generic identifier, not a language
        # allowlist or language-detection heuristic.
        _identifier("utterance_locale", self.utterance_locale, 64)
        if self.gesture_kind is None:
            _integer("repetitions", self.repetitions, 0, 0)
        elif self.gesture_kind == "PROPELLER_WAVE":
            _integer("repetitions", self.repetitions, 1, 2)
        else:
            raise InteractionContractError(
                "invalid_gesture_kind",
                "gesture_kind must be null or PROPELLER_WAVE",
            )
        _identifier("affect_label", self.affect_label, 64)
        _integer("intensity", self.intensity, 0, 1_000)

    def to_dict(self):
        return {
            "utterance": self.utterance,
            "utterance_locale": self.utterance_locale,
            "gesture_kind": self.gesture_kind,
            "affect_label": self.affect_label,
            "intensity": self.intensity,
            "repetitions": self.repetitions,
        }


def _expression_intent_from_value(value) -> ExpressionIntent:
    if not isinstance(value, dict) or set(value) != _EXPRESSION_INTENT_FIELDS:
        raise InteractionContractError(
            "invalid_expression_intent_fields",
            "Expression intent fields are invalid",
        )
    return ExpressionIntent(
        utterance=value["utterance"],
        utterance_locale=value["utterance_locale"],
        gesture_kind=value["gesture_kind"],
        affect_label=value["affect_label"],
        intensity=value["intensity"],
        repetitions=value["repetitions"],
    )


@dataclass(frozen=True)
class ExpressionProposal:
    """A model proposal bound to one exact interaction snapshot."""

    proposal_id: str
    robot_id: str
    controller_instance_id: str
    goal_id: str
    goal_epoch: int
    plan_revision: int
    based_on_interaction_state_version: int
    based_on_world_model_version: int
    obstruction_epoch: int
    based_on_evidence_id: Optional[str]
    decision: str
    confidence_milli: int
    intent: Optional[ExpressionIntent] = None
    reason_code: Optional[str] = None

    def __post_init__(self) -> None:
        _identifier("proposal_id", self.proposal_id)
        _identifier("robot_id", self.robot_id)
        _identifier("controller_instance_id", self.controller_instance_id)
        _identifier("goal_id", self.goal_id)
        _integer("goal_epoch", self.goal_epoch, 1, _MAX_INT)
        _integer("plan_revision", self.plan_revision, 1, _MAX_INT)
        _integer(
            "based_on_interaction_state_version",
            self.based_on_interaction_state_version,
            1,
            _MAX_INT,
        )
        _integer(
            "based_on_world_model_version",
            self.based_on_world_model_version,
            1,
            _MAX_INT,
        )
        _integer("obstruction_epoch", self.obstruction_epoch, 0, _MAX_INT)
        _optional_identifier(
            "based_on_evidence_id",
            self.based_on_evidence_id,
        )
        _integer("confidence_milli", self.confidence_milli, 0, 1_000)

        if self.decision == "EXPRESS":
            if self.based_on_evidence_id is None:
                raise InteractionContractError(
                    "missing_expression_evidence",
                    "EXPRESS requires an evidence binding",
                )
            if not isinstance(self.intent, ExpressionIntent):
                raise InteractionContractError(
                    "missing_expression_intent",
                    "EXPRESS requires an expression intent",
                )
            if self.reason_code is not None:
                raise InteractionContractError(
                    "unexpected_reason",
                    "EXPRESS cannot contain reason_code",
                )
        elif self.decision in ("HOLD", "ABORT"):
            if self.intent is not None:
                raise InteractionContractError(
                    "unexpected_expression_intent",
                    "{} cannot contain an expression intent".format(
                        self.decision
                    ),
                )
            _identifier("reason_code", self.reason_code or "", 64)
        else:
            raise InteractionContractError(
                "invalid_expression_decision",
                "decision must be EXPRESS, HOLD or ABORT",
            )

    def to_dict(self):
        value = {
            "schema": EXPRESSION_PROPOSAL_SCHEMA,
            "proposal_id": self.proposal_id,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "goal_id": self.goal_id,
            "goal_epoch": self.goal_epoch,
            "plan_revision": self.plan_revision,
            "based_on_interaction_state_version": (
                self.based_on_interaction_state_version
            ),
            "based_on_world_model_version": (
                self.based_on_world_model_version
            ),
            "obstruction_epoch": self.obstruction_epoch,
            "based_on_evidence_id": self.based_on_evidence_id,
            "decision": self.decision,
            "confidence_milli": self.confidence_milli,
        }
        if self.decision == "EXPRESS":
            value["intent"] = self.intent.to_dict()
        else:
            value["reason_code"] = self.reason_code
        return value

    def assert_matches_snapshot(
        self,
        snapshot: InteractionSnapshot,
    ) -> None:
        """Reject stale or cross-controller asynchronous results."""

        if not isinstance(snapshot, InteractionSnapshot):
            raise InteractionContractError(
                "invalid_interaction_snapshot",
                "snapshot must be InteractionSnapshot",
            )
        current_evidence_id = (
            None
            if snapshot.evidence is None
            else snapshot.evidence.evidence_id
        )
        bindings = (
            ("robot_id", self.robot_id, snapshot.robot_id),
            (
                "controller_instance_id",
                self.controller_instance_id,
                snapshot.controller_instance_id,
            ),
            ("goal_id", self.goal_id, snapshot.goal_id),
            ("goal_epoch", self.goal_epoch, snapshot.goal_epoch),
            ("plan_revision", self.plan_revision, snapshot.plan_revision),
            (
                "interaction_state_version",
                self.based_on_interaction_state_version,
                snapshot.interaction_state_version,
            ),
            (
                "world_model_version",
                self.based_on_world_model_version,
                snapshot.world_model_version,
            ),
            (
                "obstruction_epoch",
                self.obstruction_epoch,
                snapshot.obstruction_epoch,
            ),
            (
                "evidence_id",
                self.based_on_evidence_id,
                current_evidence_id,
            ),
        )
        if self.decision == "EXPRESS":
            bindings += (
                (
                    "response_locale",
                    self.intent.utterance_locale,
                    snapshot.response_locale,
                ),
            )
        for name, proposed, current in bindings:
            if proposed != current:
                raise InteractionContractError(
                    "stale_expression_proposal",
                    "{} does not match the current snapshot".format(name),
                )


def decode_expression_proposal(raw: bytes) -> ExpressionProposal:
    """Decode strict model output without granting execution authority."""

    value = _decode_json_object(raw, "expression_proposal")
    if value.get("schema") != EXPRESSION_PROPOSAL_SCHEMA:
        raise InteractionContractError(
            "invalid_expression_proposal_schema",
            "Expression proposal schema is not supported",
        )

    decision = value.get("decision")
    intent = None
    reason_code = None
    if decision == "EXPRESS":
        expected = _EXPRESSION_PROPOSAL_COMMON_FIELDS | {"intent"}
        if set(value) != expected:
            raise InteractionContractError(
                "invalid_expression_proposal_fields",
                "EXPRESS fields are invalid",
            )
        intent = _expression_intent_from_value(value["intent"])
    elif decision in ("HOLD", "ABORT"):
        expected = _EXPRESSION_PROPOSAL_COMMON_FIELDS | {"reason_code"}
        if set(value) != expected:
            raise InteractionContractError(
                "invalid_expression_proposal_fields",
                "{} fields are invalid".format(decision),
            )
        reason_code = value["reason_code"]
    else:
        raise InteractionContractError(
            "invalid_expression_decision",
            "decision must be EXPRESS, HOLD or ABORT",
        )

    return ExpressionProposal(
        proposal_id=value["proposal_id"],
        robot_id=value["robot_id"],
        controller_instance_id=value["controller_instance_id"],
        goal_id=value["goal_id"],
        goal_epoch=value["goal_epoch"],
        plan_revision=value["plan_revision"],
        based_on_interaction_state_version=(
            value["based_on_interaction_state_version"]
        ),
        based_on_world_model_version=value["based_on_world_model_version"],
        obstruction_epoch=value["obstruction_epoch"],
        based_on_evidence_id=value["based_on_evidence_id"],
        decision=value["decision"],
        confidence_milli=value["confidence_milli"],
        intent=intent,
        reason_code=reason_code,
    )
