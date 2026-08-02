"""Small, host-bound model contract for physical navigation intent.

This is a shadow v2 contract.  It deliberately has no runtime integration.
The model returns only one semantic intent selected from a host-authored
offer.  Identity, snapshot basis, receive time and expiry are attached by the
host after strict decoding, so the model cannot copy or manufacture them.
"""

from dataclasses import dataclass
import json
from typing import Mapping, Optional, Sequence, Tuple

from .physical_agent_state import NavigationBasis


FOLLOW_DIRECTION = "FOLLOW_DIRECTION"
SCAN_TARGET = "SCAN_TARGET"
DETOUR_TARGET = "DETOUR_TARGET"
HOLD = "HOLD"
ABORT = "ABORT"

LEFT = "LEFT"
RIGHT = "RIGHT"

INTENT_KINDS = (
    FOLLOW_DIRECTION,
    SCAN_TARGET,
    DETOUR_TARGET,
    HOLD,
    ABORT,
)
DETOUR_SIDES = (LEFT, RIGHT)

MAX_NAVIGATION_INTENT_BYTES = 4 * 1024
MAX_NAVIGATION_INTENT_SCHEMA_BYTES = 3 * 1024
MAX_NAVIGATION_INTENT_TTL_MS = 60_000
_MAX_INT = 2**63 - 1


class NavigationIntentProposalError(ValueError):
    """An untrusted proposal or its host binding is invalid."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _identifier(name: str, value: object, maximum: int = 128) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise NavigationIntentProposalError(
            "invalid_identifier",
            "{} is invalid".format(name),
        )
    return value


def _integer(
    name: str,
    value: object,
    minimum: int,
    maximum: int,
) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise NavigationIntentProposalError(
            "invalid_integer",
            "{} is invalid".format(name),
        )
    return value


def _normalised_enum(
    name: str,
    values: Sequence[str],
    *,
    allowed: Optional[Sequence[str]] = None,
) -> Tuple[str, ...]:
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise NavigationIntentProposalError(
            "invalid_offer",
            "{} must be a sequence".format(name),
        )
    checked = tuple(_identifier(name, value) for value in values)
    if len(set(checked)) != len(checked):
        raise NavigationIntentProposalError(
            "invalid_offer",
            "{} contains duplicates".format(name),
        )
    if allowed is not None and any(value not in allowed for value in checked):
        raise NavigationIntentProposalError(
            "invalid_offer",
            "{} contains an unsupported value".format(name),
        )
    return tuple(sorted(checked))


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


@dataclass(frozen=True)
class NavigationIntentOffer:
    """The exact semantic menu attached to one host inference ticket."""

    ticket_id: str
    basis: NavigationBasis
    offered_intents: Tuple[str, ...]
    scan_target_ids: Tuple[str, ...] = ()
    detour_target_ids: Tuple[str, ...] = ()
    detour_sides: Tuple[str, ...] = ()
    hold_reasons: Tuple[str, ...] = ()
    abort_reasons: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        _identifier("ticket_id", self.ticket_id)
        if not isinstance(self.basis, NavigationBasis):
            raise NavigationIntentProposalError(
                "invalid_basis",
                "basis must be NavigationBasis",
            )

        offered = _normalised_enum(
            "offered_intent",
            self.offered_intents,
            allowed=INTENT_KINDS,
        )
        offered_set = set(offered)
        if not offered:
            raise NavigationIntentProposalError(
                "invalid_offer",
                "At least one intent must be offered",
            )
        offered = tuple(
            intent for intent in INTENT_KINDS if intent in offered_set
        )
        scan_targets = _normalised_enum(
            "scan_target_id",
            self.scan_target_ids,
        )
        detour_targets = _normalised_enum(
            "detour_target_id",
            self.detour_target_ids,
        )
        sides = _normalised_enum(
            "detour_side",
            self.detour_sides,
            allowed=DETOUR_SIDES,
        )
        sides = tuple(side for side in DETOUR_SIDES if side in set(sides))
        hold_reasons = _normalised_enum(
            "hold_reason",
            self.hold_reasons,
        )
        abort_reasons = _normalised_enum(
            "abort_reason",
            self.abort_reasons,
        )

        requirements = (
            (SCAN_TARGET, scan_targets, "scan_target_ids"),
            (DETOUR_TARGET, detour_targets, "detour_target_ids"),
            (DETOUR_TARGET, sides, "detour_sides"),
            (HOLD, hold_reasons, "hold_reasons"),
            (ABORT, abort_reasons, "abort_reasons"),
        )
        for intent, values, field_name in requirements:
            if (intent in offered_set) != bool(values):
                raise NavigationIntentProposalError(
                    "invalid_offer",
                    "{} must be non-empty exactly when {} is offered".format(
                        field_name,
                        intent,
                    ),
                )

        object.__setattr__(self, "offered_intents", offered)
        object.__setattr__(self, "scan_target_ids", scan_targets)
        object.__setattr__(self, "detour_target_ids", detour_targets)
        object.__setattr__(self, "detour_sides", sides)
        object.__setattr__(self, "hold_reasons", hold_reasons)
        object.__setattr__(self, "abort_reasons", abort_reasons)
        schema_bytes = len(
            json.dumps(
                build_navigation_intent_proposal_schema(self),
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        )
        if schema_bytes > MAX_NAVIGATION_INTENT_SCHEMA_BYTES:
            raise NavigationIntentProposalError(
                "offer_schema_too_large",
                "Navigation intent offer exceeds the schema budget",
            )

    def assert_allows(self, proposal: "NavigationIntentProposal") -> None:
        if not isinstance(proposal, NavigationIntentProposal):
            raise NavigationIntentProposalError(
                "invalid_proposal",
                "proposal must be NavigationIntentProposal",
            )
        if proposal.intent not in self.offered_intents:
            raise NavigationIntentProposalError(
                "unoffered_intent",
                "The intent was not offered by the host",
            )
        if (
            proposal.intent == SCAN_TARGET
            and proposal.target_id not in self.scan_target_ids
        ):
            raise NavigationIntentProposalError(
                "unoffered_target",
                "The scan target was not offered by the host",
            )
        if proposal.intent == DETOUR_TARGET:
            if proposal.target_id not in self.detour_target_ids:
                raise NavigationIntentProposalError(
                    "unoffered_target",
                    "The detour target was not offered by the host",
                )
            if proposal.side not in self.detour_sides:
                raise NavigationIntentProposalError(
                    "unoffered_side",
                    "The detour side was not offered by the host",
                )
        if (
            proposal.intent == HOLD
            and proposal.reason not in self.hold_reasons
        ):
            raise NavigationIntentProposalError(
                "unoffered_reason",
                "The hold reason was not offered by the host",
            )
        if (
            proposal.intent == ABORT
            and proposal.reason not in self.abort_reasons
        ):
            raise NavigationIntentProposalError(
                "unoffered_reason",
                "The abort reason was not offered by the host",
            )


@dataclass(frozen=True)
class NavigationIntentProposal:
    """Minimal untrusted model output, before host authority is attached."""

    intent: str
    target_id: Optional[str] = None
    side: Optional[str] = None
    reason: Optional[str] = None

    def __post_init__(self) -> None:
        if self.intent not in INTENT_KINDS:
            raise NavigationIntentProposalError(
                "invalid_intent",
                "Navigation intent is unsupported",
            )
        if self.intent == FOLLOW_DIRECTION:
            expected = (None, None, None)
        elif self.intent == SCAN_TARGET:
            _identifier("target_id", self.target_id)
            expected = (self.target_id, None, None)
        elif self.intent == DETOUR_TARGET:
            _identifier("target_id", self.target_id)
            if self.side not in DETOUR_SIDES:
                raise NavigationIntentProposalError(
                    "invalid_side",
                    "Detour side must be LEFT or RIGHT",
                )
            expected = (self.target_id, self.side, None)
        else:
            _identifier("reason", self.reason, 64)
            expected = (None, None, self.reason)
        if (self.target_id, self.side, self.reason) != expected:
            raise NavigationIntentProposalError(
                "invalid_intent_fields",
                "Intent contains fields that do not belong to it",
            )

    def to_dict(self) -> Mapping[str, object]:
        value = {"intent": self.intent}
        if self.intent == SCAN_TARGET:
            value["target_id"] = self.target_id
        elif self.intent == DETOUR_TARGET:
            value["target_id"] = self.target_id
            value["side"] = self.side
        elif self.intent in (HOLD, ABORT):
            value["reason"] = self.reason
        return value


def _strict_variant(properties: Mapping[str, object]):
    return {
        "type": "object",
        "properties": dict(properties),
        "required": sorted(properties),
        "additionalProperties": False,
    }


def build_navigation_intent_proposal_schema(
    offer: NavigationIntentOffer,
) -> Mapping[str, object]:
    """Build a strict schema containing only the host-offered variants."""

    if not isinstance(offer, NavigationIntentOffer):
        raise NavigationIntentProposalError(
            "invalid_offer",
            "offer must be NavigationIntentOffer",
        )
    variants = []
    for intent in offer.offered_intents:
        properties = {
            "intent": {"type": "string", "const": intent},
        }
        if intent == SCAN_TARGET:
            properties["target_id"] = {
                "type": "string",
                "enum": list(offer.scan_target_ids),
            }
        elif intent == DETOUR_TARGET:
            properties["target_id"] = {
                "type": "string",
                "enum": list(offer.detour_target_ids),
            }
            properties["side"] = {
                "type": "string",
                "enum": list(offer.detour_sides),
            }
        elif intent == HOLD:
            properties["reason"] = {
                "type": "string",
                "enum": list(offer.hold_reasons),
            }
        elif intent == ABORT:
            properties["reason"] = {
                "type": "string",
                "enum": list(offer.abort_reasons),
            }
        variants.append(_strict_variant(properties))
    return {"oneOf": variants}


def decode_navigation_intent_proposal(
    raw: bytes,
    offer: NavigationIntentOffer,
) -> NavigationIntentProposal:
    """Strictly decode model bytes and verify the host-authored menu."""

    if not isinstance(offer, NavigationIntentOffer):
        raise NavigationIntentProposalError(
            "invalid_offer",
            "offer must be NavigationIntentOffer",
        )
    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > MAX_NAVIGATION_INTENT_BYTES
    ):
        raise NavigationIntentProposalError(
            "invalid_proposal_body",
            "Navigation intent proposal body is invalid",
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
        raise NavigationIntentProposalError(
            "invalid_proposal_json",
            "Navigation intent proposal is not strict JSON",
        ) from None
    if not isinstance(value, dict):
        raise NavigationIntentProposalError(
            "invalid_proposal_shape",
            "Navigation intent proposal must be an object",
        )

    intent = value.get("intent")
    if intent == FOLLOW_DIRECTION:
        expected_fields = {"intent"}
    elif intent == SCAN_TARGET:
        expected_fields = {"intent", "target_id"}
    elif intent == DETOUR_TARGET:
        expected_fields = {"intent", "target_id", "side"}
    elif intent in (HOLD, ABORT):
        expected_fields = {"intent", "reason"}
    else:
        raise NavigationIntentProposalError(
            "invalid_intent",
            "Navigation intent is unsupported",
        )
    if set(value) != expected_fields:
        raise NavigationIntentProposalError(
            "invalid_intent_fields",
            "Navigation intent fields are invalid",
        )

    proposal = NavigationIntentProposal(
        intent=intent,
        target_id=value.get("target_id"),
        side=value.get("side"),
        reason=value.get("reason"),
    )
    offer.assert_allows(proposal)
    return proposal


@dataclass(frozen=True)
class NavigationIntentEnvelope:
    """A decoded proposal carrying host-owned freshness and identity."""

    proposal_id: str
    ticket_id: str
    basis: NavigationBasis
    received_at_ms: int
    valid_until_ms: int
    proposal: NavigationIntentProposal

    def __post_init__(self) -> None:
        _identifier("proposal_id", self.proposal_id)
        _identifier("ticket_id", self.ticket_id)
        if not isinstance(self.basis, NavigationBasis):
            raise NavigationIntentProposalError(
                "invalid_basis",
                "basis must be NavigationBasis",
            )
        if not isinstance(self.proposal, NavigationIntentProposal):
            raise NavigationIntentProposalError(
                "invalid_proposal",
                "proposal must be NavigationIntentProposal",
            )
        _integer("received_at_ms", self.received_at_ms, 0, _MAX_INT - 1)
        _integer(
            "valid_until_ms",
            self.valid_until_ms,
            self.received_at_ms + 1,
            _MAX_INT,
        )
        if (
            self.valid_until_ms - self.received_at_ms
            > MAX_NAVIGATION_INTENT_TTL_MS
        ):
            raise NavigationIntentProposalError(
                "invalid_ttl",
                "Navigation intent TTL exceeds the host limit",
            )

    def assert_current(
        self,
        *,
        proposal_id: str,
        ticket_id: str,
        basis: NavigationBasis,
        now_ms: int,
    ) -> None:
        """Reject replayed, cross-controller, stale or expired envelopes."""

        _identifier("proposal_id", proposal_id)
        _identifier("ticket_id", ticket_id)
        if not isinstance(basis, NavigationBasis):
            raise NavigationIntentProposalError(
                "invalid_basis",
                "basis must be NavigationBasis",
            )
        _integer("now_ms", now_ms, 0, _MAX_INT)
        if now_ms < self.received_at_ms:
            raise NavigationIntentProposalError(
                "invalid_current_time",
                "Current time predates proposal receipt",
            )
        if now_ms >= self.valid_until_ms:
            raise NavigationIntentProposalError(
                "expired_proposal",
                "Navigation intent proposal has expired",
            )
        if (
            proposal_id != self.proposal_id
            or ticket_id != self.ticket_id
        ):
            raise NavigationIntentProposalError(
                "proposal_identity_mismatch",
                "Navigation intent identity does not match",
            )
        if not self.basis.decision_equivalent(basis):
            raise NavigationIntentProposalError(
                "stale_navigation_basis",
                "Navigation basis has changed",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "proposal_id": self.proposal_id,
            "ticket_id": self.ticket_id,
            "basis": _navigation_basis_dict(self.basis),
            "received_at_ms": self.received_at_ms,
            "valid_until_ms": self.valid_until_ms,
            "proposal": self.proposal.to_dict(),
        }


def bind_navigation_intent_proposal(
    proposal: NavigationIntentProposal,
    *,
    offer: NavigationIntentOffer,
    proposal_id: str,
    received_at_ms: int,
    valid_until_ms: int,
) -> NavigationIntentEnvelope:
    """Attach host identity and freshness after successful model decoding."""

    if not isinstance(offer, NavigationIntentOffer):
        raise NavigationIntentProposalError(
            "invalid_offer",
            "offer must be NavigationIntentOffer",
        )
    offer.assert_allows(proposal)
    return NavigationIntentEnvelope(
        proposal_id=proposal_id,
        ticket_id=offer.ticket_id,
        basis=offer.basis,
        received_at_ms=received_at_ms,
        valid_until_ms=valid_until_ms,
        proposal=proposal,
    )


def _navigation_basis_dict(basis: NavigationBasis) -> Mapping[str, object]:
    key = basis.controller_key
    return {
        "controller_key": {
            "robot_id": key.robot_id,
            "controller_id": key.controller_id,
            "controller_instance_id": key.controller_instance_id,
        },
        "goal_epoch": basis.goal_epoch,
        "controller_state_version": basis.controller_state_version,
        "world_generation_id": basis.world_generation_id,
        "world_model_version": basis.world_model_version,
        "navigation_basis_id": basis.navigation_basis_id,
        "frame_id": basis.frame_id,
        "calibration_fingerprint": basis.calibration_fingerprint,
    }


def _canonical_json_size(value: object) -> int:
    try:
        encoded = json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    except (TypeError, ValueError):
        raise NavigationIntentProposalError(
            "invalid_size_input",
            "Size metric input must be finite JSON data",
        ) from None
    return len(encoded)


def _reduction_milli(current: int, shadow: int) -> int:
    return ((current - shadow) * 1_000) // current


def size_metrics(
    *,
    current_schema: Mapping[str, object],
    current_output: Mapping[str, object],
    offer: NavigationIntentOffer,
    proposal: NavigationIntentProposal,
) -> Mapping[str, int]:
    """Return deterministic canonical-byte comparisons with the v1 contract."""

    if not isinstance(current_schema, Mapping) or not current_schema:
        raise NavigationIntentProposalError(
            "invalid_size_input",
            "current_schema must be a non-empty mapping",
        )
    if not isinstance(current_output, Mapping) or not current_output:
        raise NavigationIntentProposalError(
            "invalid_size_input",
            "current_output must be a non-empty mapping",
        )
    if not isinstance(offer, NavigationIntentOffer):
        raise NavigationIntentProposalError(
            "invalid_offer",
            "offer must be NavigationIntentOffer",
        )
    offer.assert_allows(proposal)

    shadow_schema = build_navigation_intent_proposal_schema(offer)
    shadow_output = proposal.to_dict()
    current_schema_bytes = _canonical_json_size(current_schema)
    shadow_schema_bytes = _canonical_json_size(shadow_schema)
    current_output_bytes = _canonical_json_size(current_output)
    shadow_output_bytes = _canonical_json_size(shadow_output)
    return {
        "current_schema_bytes": current_schema_bytes,
        "shadow_schema_bytes": shadow_schema_bytes,
        "schema_saved_bytes": current_schema_bytes - shadow_schema_bytes,
        "schema_reduction_milli": _reduction_milli(
            current_schema_bytes,
            shadow_schema_bytes,
        ),
        "current_output_bytes": current_output_bytes,
        "shadow_output_bytes": shadow_output_bytes,
        "output_saved_bytes": current_output_bytes - shadow_output_bytes,
        "output_reduction_milli": _reduction_milli(
            current_output_bytes,
            shadow_output_bytes,
        ),
    }


__all__ = (
    "ABORT",
    "DETOUR_SIDES",
    "DETOUR_TARGET",
    "FOLLOW_DIRECTION",
    "HOLD",
    "INTENT_KINDS",
    "LEFT",
    "MAX_NAVIGATION_INTENT_BYTES",
    "MAX_NAVIGATION_INTENT_SCHEMA_BYTES",
    "MAX_NAVIGATION_INTENT_TTL_MS",
    "NavigationBasis",
    "NavigationIntentEnvelope",
    "NavigationIntentOffer",
    "NavigationIntentProposal",
    "NavigationIntentProposalError",
    "RIGHT",
    "SCAN_TARGET",
    "bind_navigation_intent_proposal",
    "build_navigation_intent_proposal_schema",
    "decode_navigation_intent_proposal",
    "size_metrics",
)
