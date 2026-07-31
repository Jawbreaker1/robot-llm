"""Model-owned obstacle maneuver commitments with strict lifecycle checks."""

from copy import deepcopy
from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

from .physical_navigation_contract import FINISH, SCAN_FRONT_ARC
from .provisional_hazard_map import ProvisionalHazardMap


TRANSITIONS = frozenset(
    ("NONE", "START", "CONTINUE", "REVISE", "COMPLETE", "ABANDON")
)
DETOUR_SIDES = frozenset(("LEFT_OF_GOAL", "RIGHT_OF_GOAL"))
FACT_GOAL_CORRIDOR_CLEAR = "GOAL_CORRIDOR_CLEAR"
FACT_GOAL_HEADING_ALIGNED = "GOAL_HEADING_ALIGNED"
FACT_TARGET_BEHIND = "TARGET_ENVELOPE_BEHIND_GOAL_ORIGIN"
FACT_KEYS = frozenset(
    (
        FACT_GOAL_CORRIDOR_CLEAR,
        FACT_GOAL_HEADING_ALIGNED,
        FACT_TARGET_BEHIND,
    )
)
COMMITMENT_TTL_TURNS = 14
FIELDS = {
    "id",
    "revision",
    "transition",
    "objective",
    "target_hypothesis_id",
    "detour_side",
    "success_fact_keys",
    "current_focus_fact_key",
    "revision_reason",
}


class ManeuverCommitmentError(ValueError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def empty_commitment() -> Mapping[str, object]:
    return {
        "id": None,
        "revision": 0,
        "transition": "NONE",
        "objective": None,
        "target_hypothesis_id": None,
        "detour_side": None,
        "success_fact_keys": [],
        "current_focus_fact_key": None,
        "revision_reason": None,
    }


def _text(value, maximum):
    return (
        isinstance(value, str)
        and bool(value)
        and value == value.strip()
        and len(value) <= maximum
        and not any(ord(character) < 32 for character in value)
    )


@dataclass(frozen=True)
class ActiveManeuver:
    commitment_id: str
    revision: int
    objective: str
    target_hypothesis_id: str
    detour_side: str
    success_fact_keys: Tuple[str, ...]
    current_focus_fact_key: str
    started_turn: int
    last_confirmed_turn: int

    def prompt_dict(self) -> Mapping[str, object]:
        return {
            "id": self.commitment_id,
            "revision": self.revision,
            "objective": self.objective,
            "target_hypothesis_id": self.target_hypothesis_id,
            "detour_side": self.detour_side,
            "success_fact_keys": list(self.success_fact_keys),
            "current_focus_fact_key": self.current_focus_fact_key,
            "started_turn": self.started_turn,
            "last_confirmed_turn": self.last_confirmed_turn,
        }


class ManeuverCommitment:
    """Validate model transitions without selecting a route or action."""

    def __init__(self, active: Optional[ActiveManeuver] = None):
        self.active = active
        self.last_terminal = None

    def state(self, turn: int) -> Mapping[str, object]:
        expired = (
            self.active is not None
            and turn - self.active.last_confirmed_turn
            > COMMITMENT_TTL_TURNS
        )
        if expired:
            self.last_terminal = {
                "transition": "EXPIRED",
                "id": self.active.commitment_id,
                "turn": turn,
                "success_claimed": False,
            }
            self.active = None
        return {
            "active": (
                None if self.active is None else self.active.prompt_dict()
            ),
            "last_terminal": deepcopy(self.last_terminal),
        }

    def _shape(self, value: object) -> Mapping[str, object]:
        if not isinstance(value, dict) or set(value) != FIELDS:
            raise ManeuverCommitmentError(
                "invalid_commitment_fields",
                "Maneuver commitment fields are invalid",
            )
        transition = value["transition"]
        if transition not in TRANSITIONS:
            raise ManeuverCommitmentError(
                "invalid_commitment_transition",
                "Maneuver transition is invalid",
            )
        if transition == "NONE":
            if value != empty_commitment():
                raise ManeuverCommitmentError(
                    "invalid_none_sentinel",
                    "NONE must be the exact empty commitment sentinel",
                )
            return value
        if (
            not _text(value["id"], 64)
            or isinstance(value["revision"], bool)
            or not isinstance(value["revision"], int)
            or value["revision"] <= 0
            or not _text(value["objective"], 160)
            or not _text(value["target_hypothesis_id"], 128)
            or value["detour_side"] not in DETOUR_SIDES
            or not isinstance(value["success_fact_keys"], list)
            or not 1 <= len(value["success_fact_keys"]) <= len(FACT_KEYS)
            or len(set(value["success_fact_keys"]))
            != len(value["success_fact_keys"])
            or any(key not in FACT_KEYS for key in value["success_fact_keys"])
        ):
            raise ManeuverCommitmentError(
                "invalid_active_commitment",
                "Active maneuver commitment is invalid",
            )
        focus = value["current_focus_fact_key"]
        if transition in ("COMPLETE", "ABANDON"):
            if focus is not None:
                raise ManeuverCommitmentError(
                    "terminal_commitment_has_focus",
                    "Terminal commitment focus must be null",
                )
        elif focus not in value["success_fact_keys"]:
            raise ManeuverCommitmentError(
                "invalid_commitment_focus",
                "Active focus must name one authored success fact",
            )
        reason = value["revision_reason"]
        if transition in ("REVISE", "ABANDON"):
            if not _text(reason, 160):
                raise ManeuverCommitmentError(
                    "missing_revision_reason",
                    "Revision or abandonment needs a reason",
                )
        elif reason is not None:
            raise ManeuverCommitmentError(
                "unexpected_revision_reason",
                "This transition cannot include a revision reason",
            )
        return value

    @staticmethod
    def _same_revision(
        value: Mapping[str, object],
        active: ActiveManeuver,
    ) -> bool:
        return (
            value["id"] == active.commitment_id
            and value["revision"] == active.revision
            and value["objective"] == active.objective
            and value["target_hypothesis_id"]
            == active.target_hypothesis_id
            and value["detour_side"] == active.detour_side
            and tuple(value["success_fact_keys"])
            == active.success_fact_keys
        )

    @staticmethod
    def _scan_ready(
        target_id: str,
        hazard_map: ProvisionalHazardMap,
    ) -> bool:
        hazard = hazard_map.get(target_id)
        return hazard is not None and hazard.bilateral_scan_complete

    def apply(
        self,
        value: object,
        *,
        action: str,
        turn: int,
        hazard_map: ProvisionalHazardMap,
        fact_values: Mapping[str, object],
        perception_target_hypothesis_id: Optional[str] = None,
    ) -> Mapping[str, object]:
        proposal = self._shape(value)
        transition = proposal["transition"]
        published = frozenset(hazard_map.hazard_ids)

        if action == SCAN_FRONT_ARC:
            if self.active is None and transition != "NONE":
                raise ManeuverCommitmentError(
                    "route_choice_before_scan",
                    "A first scan must keep the maneuver commitment NONE",
                )
            if self.active is not None and (
                transition != "CONTINUE"
                or not self._same_revision(proposal, self.active)
                or perception_target_hypothesis_id
                != self.active.target_hypothesis_id
            ):
                raise ManeuverCommitmentError(
                    "route_changed_during_scan",
                    "Scanning may not also change the active route",
                )

        if self.active is None:
            if transition == "NONE":
                return self.state(turn)
            if transition != "START":
                raise ManeuverCommitmentError(
                    "transition_without_active_commitment",
                    "Only START or NONE is valid without an active maneuver",
                )
            if proposal["revision"] != 1:
                raise ManeuverCommitmentError(
                    "invalid_start_revision",
                    "START revision must be one",
                )
            target = proposal["target_hypothesis_id"]
            if target not in published:
                raise ManeuverCommitmentError(
                    "unknown_maneuver_target",
                    "START target is not in the current map",
                )
            if not self._scan_ready(target, hazard_map):
                raise ManeuverCommitmentError(
                    "bilateral_scan_required",
                    "Route commitment requires a completed bilateral scan",
                )
            self.active = ActiveManeuver(
                commitment_id=proposal["id"],
                revision=1,
                objective=proposal["objective"],
                target_hypothesis_id=target,
                detour_side=proposal["detour_side"],
                success_fact_keys=tuple(proposal["success_fact_keys"]),
                current_focus_fact_key=proposal[
                    "current_focus_fact_key"
                ],
                started_turn=turn,
                last_confirmed_turn=turn,
            )
            return self.state(turn)

        active = self.active
        if (
            active.target_hypothesis_id not in published
            and transition != "ABANDON"
        ):
            raise ManeuverCommitmentError(
                "evicted_target_requires_abandon",
                "A disappeared target can only be abandoned",
            )
        if transition == "NONE" or proposal["id"] != active.commitment_id:
            raise ManeuverCommitmentError(
                "active_commitment_lost",
                "The active commitment must be explicitly continued",
            )
        if transition == "CONTINUE":
            if not self._same_revision(proposal, active):
                raise ManeuverCommitmentError(
                    "continue_changed_revision",
                    "CONTINUE must preserve the active revision",
                )
            self.active = ActiveManeuver(
                commitment_id=active.commitment_id,
                revision=active.revision,
                objective=active.objective,
                target_hypothesis_id=active.target_hypothesis_id,
                detour_side=active.detour_side,
                success_fact_keys=active.success_fact_keys,
                current_focus_fact_key=proposal[
                    "current_focus_fact_key"
                ],
                started_turn=active.started_turn,
                last_confirmed_turn=turn,
            )
        elif transition == "REVISE":
            if proposal["revision"] != active.revision + 1:
                raise ManeuverCommitmentError(
                    "invalid_revision_increment",
                    "REVISE must increment the revision by one",
                )
            target = proposal["target_hypothesis_id"]
            if target not in published:
                raise ManeuverCommitmentError(
                    "unknown_revised_target",
                    "Revised target is not in the current map",
                )
            route_changed = (
                target != active.target_hypothesis_id
                or proposal["detour_side"] != active.detour_side
            )
            if route_changed and not self._scan_ready(target, hazard_map):
                raise ManeuverCommitmentError(
                    "bilateral_scan_required",
                    "A changed route requires bilateral scan evidence",
                )
            self.active = ActiveManeuver(
                commitment_id=active.commitment_id,
                revision=proposal["revision"],
                objective=proposal["objective"],
                target_hypothesis_id=target,
                detour_side=proposal["detour_side"],
                success_fact_keys=tuple(proposal["success_fact_keys"]),
                current_focus_fact_key=proposal[
                    "current_focus_fact_key"
                ],
                started_turn=active.started_turn,
                last_confirmed_turn=turn,
            )
        elif transition == "COMPLETE":
            if not self._same_revision(proposal, active):
                raise ManeuverCommitmentError(
                    "complete_changed_revision",
                    "COMPLETE must preserve the active revision",
                )
            if action != FINISH:
                raise ManeuverCommitmentError(
                    "complete_requires_finish",
                    "COMPLETE is only valid with FINISH",
                )
            missing = [
                key
                for key in active.success_fact_keys
                if (
                    fact_values.get(key, {}).get(
                        active.target_hypothesis_id
                    )
                    is not True
                    if key == FACT_TARGET_BEHIND
                    else fact_values.get(key) is not True
                )
            ]
            if missing:
                raise ManeuverCommitmentError(
                    "maneuver_facts_not_complete",
                    "Maneuver success facts are not all true",
                )
            self.last_terminal = {
                "transition": "COMPLETE",
                "id": active.commitment_id,
                "turn": turn,
                "success_claimed": True,
            }
            self.active = None
        elif transition == "ABANDON":
            if not self._same_revision(proposal, active):
                raise ManeuverCommitmentError(
                    "abandon_changed_revision",
                    "ABANDON must preserve the active revision",
                )
            self.last_terminal = {
                "transition": "ABANDON",
                "id": active.commitment_id,
                "turn": turn,
                "success_claimed": False,
                "reason": proposal["revision_reason"],
            }
            self.active = None
        else:
            raise ManeuverCommitmentError(
                "invalid_active_transition",
                "Transition is invalid for an active maneuver",
            )
        return self.state(turn)
