"""Bounded episode-local evidence about physical navigation attempts.

The ledger records what was tried and what the robot actually observed.  It
does not score, rank, recommend, or select actions.  A planner can therefore
distinguish an exact retry from the same action after a verified change in
pose or evidence without relying on natural-language summaries.
"""

from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass
import hashlib
from typing import Mapping, Optional, Tuple

from .physical_navigation_contract import (
    ACTIONS,
    SCAN_FRONT_ARC,
    json_bytes,
    validate_observation,
)
from .physical_observation_progress import observation_progress_signature


EXPERIENCE_LEDGER_SCHEMA = "robot-physical-navigation-experience/v1"
EXPERIENCE_BASIS_SCHEMA = "robot-physical-navigation-evidence-basis/v1"
MAX_EXPERIENCE_ENTRIES = 64
MAX_EXPERIENCE_CONTEXT_BYTES = 64 * 1024
# A runtime turn can execute at most the model-selected action plus two plan
# tail actions.  14,400 is the runtime's hard turn ceiling, so this bounded
# index cannot forget an exact action/evidence basis during one legal episode.
MAX_EXPERIENCE_SEEN_KEYS = 43_200

FIRST_ATTEMPT = "FIRST_ATTEMPT"
UNCHANGED_BASIS_REPEAT = "UNCHANGED_BASIS_REPEAT"
RETRY_AFTER_BASIS_CHANGE = "RETRY_AFTER_BASIS_CHANGE"

PLANNER_ACTION_SOURCE = "PLANNER_ACTION"
PLAN_TAIL_ACTION_SOURCE = "PLAN_TAIL_ACTION"
ACTION_SOURCES = frozenset((PLANNER_ACTION_SOURCE, PLAN_TAIL_ACTION_SOURCE))


class NavigationExperienceError(ValueError):
    pass


def _required_mapping(value, fields, label):
    if not isinstance(value, Mapping) or any(
        field not in value for field in fields
    ):
        raise NavigationExperienceError("{} is invalid".format(label))
    return value


def _pose_basis(navigation: Mapping[str, object]) -> Mapping[str, object]:
    pose = _required_mapping(
        navigation.get("pose"),
        ("x_mm", "y_mm", "heading_mdeg"),
        "navigation pose",
    )
    result = {
        "x_mm": pose["x_mm"],
        "y_mm": pose["y_mm"],
        "heading_mdeg": pose["heading_mdeg"],
    }
    if any(
        isinstance(value, bool) or not isinstance(value, int)
        for value in result.values()
    ):
        raise NavigationExperienceError("navigation pose is invalid")
    return result


def _scan_basis(
    hazard: Mapping[str, object],
) -> Tuple[Mapping[str, object], ...]:
    history = hazard.get("scan_evidence_history", [])
    if not isinstance(history, list):
        raise NavigationExperienceError("hazard scan evidence is invalid")
    values = {}
    for attempt in history:
        checked = _required_mapping(
            attempt,
            (
                "status",
                "observation_pattern",
                "arc_coverage",
                "boundary_coverage",
                "hypothesis_relation",
            ),
            "scan evidence",
        )
        raw_pose = checked.get("scan_pose")
        if raw_pose is None:
            scan_pose = None
        else:
            pose = _required_mapping(
                raw_pose,
                ("x_mm", "y_mm", "heading_mdeg"),
                "scan evidence pose",
            )
            scan_pose = {
                key: pose[key]
                for key in ("x_mm", "y_mm", "heading_mdeg")
            }
        raw_rays = checked.get("rays", [])
        if not isinstance(raw_rays, list):
            raise NavigationExperienceError("scan evidence rays are invalid")
        rays = []
        for ray in raw_rays:
            value = _required_mapping(
                ray,
                (
                    "requested_relative_bearing_mdeg",
                    "actual_relative_bearing_mdeg",
                    "blocked",
                ),
                "scan evidence ray",
            )
            rays.append({
                "requested_relative_bearing_mdeg": value[
                    "requested_relative_bearing_mdeg"
                ],
                "actual_relative_bearing_mdeg": value[
                    "actual_relative_bearing_mdeg"
                ],
                "blocked": value["blocked"],
            })
        fact = {
            "status": checked["status"],
            "observation_pattern": checked["observation_pattern"],
            "arc_coverage": checked["arc_coverage"],
            "boundary_coverage": checked["boundary_coverage"],
            "hypothesis_relation": checked["hypothesis_relation"],
            "left_boundary_mdeg": checked.get("left_boundary_mdeg"),
            "right_boundary_mdeg": checked.get("right_boundary_mdeg"),
            "scan_pose": scan_pose,
            "rays": sorted(
                rays,
                key=lambda item: (
                    item["requested_relative_bearing_mdeg"],
                    item["actual_relative_bearing_mdeg"],
                ),
            ),
        }
        # IDs, timestamps, and raw reflection jitter prove freshness, not new
        # spatial information.  Collapse exact duplicate evidence so it does
        # not turn another identical attempt into an "informed" retry.
        values[json_bytes(fact)] = fact
    return tuple(values[key] for key in sorted(values))


def _hazard_basis(
    navigation: Mapping[str, object],
) -> Tuple[Mapping[str, object], ...]:
    hazards = navigation.get("navigation_hazard_hypotheses")
    if not isinstance(hazards, list):
        raise NavigationExperienceError("navigation hazards are invalid")
    values = []
    for hazard in hazards:
        checked = _required_mapping(
            hazard,
            (
                "hypothesis_id",
                "centroid_x_mm",
                "centroid_y_mm",
                "radius_mm",
                "scan_left_boundary_mdeg",
                "scan_right_boundary_mdeg",
            ),
            "navigation hazard",
        )
        values.append({
            "hypothesis_id": checked["hypothesis_id"],
            "centroid_x_mm": checked["centroid_x_mm"],
            "centroid_y_mm": checked["centroid_y_mm"],
            "radius_mm": checked["radius_mm"],
            "scan_left_boundary_mdeg": checked[
                "scan_left_boundary_mdeg"
            ],
            "scan_right_boundary_mdeg": checked[
                "scan_right_boundary_mdeg"
            ],
            "scan_evidence": list(_scan_basis(checked)),
        })
    return tuple(sorted(values, key=lambda item: item["hypothesis_id"]))


def navigation_evidence_basis(
    navigation: Mapping[str, object],
    observation: Mapping[str, object],
) -> Mapping[str, object]:
    """Return decision-relevant physical facts, excluding freshness noise."""

    checked_observation = validate_observation(observation)
    if not isinstance(navigation, Mapping):
        raise NavigationExperienceError("navigation context is invalid")
    map_generation_id = navigation.get("map_generation_id")
    localization_valid = navigation.get("localization_valid")
    if (
        not isinstance(map_generation_id, str)
        or not map_generation_id
        or type(localization_valid) is not bool
    ):
        raise NavigationExperienceError("navigation identity is invalid")
    drive_roles = navigation.get("drive_motor_roles")
    if drive_roles is None:
        progress_motor_roles = None
    elif (
        isinstance(drive_roles, Mapping)
        and set(drive_roles) == {"left", "right"}
        and all(
            isinstance(role, str) and role
            for role in drive_roles.values()
        )
        and drive_roles["left"] != drive_roles["right"]
    ):
        progress_motor_roles = (
            drive_roles["left"],
            drive_roles["right"],
        )
    else:
        raise NavigationExperienceError("drive motor roles are invalid")
    observation_facts = dict(observation_progress_signature(
        checked_observation,
        motor_roles=progress_motor_roles,
    ))
    return {
        "schema": EXPERIENCE_BASIS_SCHEMA,
        "map_generation_id": map_generation_id,
        "localization_valid": localization_valid,
        "pose": _pose_basis(navigation),
        "observation_facts": observation_facts,
        "hazards": list(_hazard_basis(navigation)),
    }


def _basis_id(basis: Mapping[str, object]) -> str:
    raw = json_bytes(basis)
    return "basis-{}".format(hashlib.sha256(raw).hexdigest()[:20])


def _basis_change_codes(
    before: Mapping[str, object],
    after: Mapping[str, object],
) -> Tuple[str, ...]:
    changes = []
    if before["map_generation_id"] != after["map_generation_id"]:
        changes.append("MAP_GENERATION_CHANGED")
    if before["localization_valid"] != after["localization_valid"]:
        changes.append("LOCALIZATION_VALIDITY_CHANGED")
    if before["pose"] != after["pose"]:
        changes.append("VERIFIED_POSE_CHANGED")
    if before["observation_facts"] != after["observation_facts"]:
        changes.append("DECISION_RELEVANT_OBSERVATION_CHANGED")

    before_hazards = {
        item["hypothesis_id"]: item for item in before["hazards"]
    }
    after_hazards = {
        item["hypothesis_id"]: item for item in after["hazards"]
    }
    if set(before_hazards) != set(after_hazards):
        changes.append("HAZARD_SET_CHANGED")
    common = sorted(set(before_hazards) & set(after_hazards))
    if any(
        {
            key: before_hazards[hypothesis_id][key]
            for key in (
                "centroid_x_mm",
                "centroid_y_mm",
                "radius_mm",
                "scan_left_boundary_mdeg",
                "scan_right_boundary_mdeg",
            )
        }
        != {
            key: after_hazards[hypothesis_id][key]
            for key in (
                "centroid_x_mm",
                "centroid_y_mm",
                "radius_mm",
                "scan_left_boundary_mdeg",
                "scan_right_boundary_mdeg",
            )
        }
        for hypothesis_id in common
    ):
        changes.append("HAZARD_GEOMETRY_CHANGED")
    if any(
        before_hazards[hypothesis_id]["scan_evidence"]
        != after_hazards[hypothesis_id]["scan_evidence"]
        for hypothesis_id in common
    ):
        changes.append("SCAN_EVIDENCE_CHANGED")
    return tuple(changes)


def _basis_summary(basis: Mapping[str, object]) -> Mapping[str, object]:
    return {
        "map_generation_id": basis["map_generation_id"],
        "localization_valid": basis["localization_valid"],
        "pose": deepcopy(basis["pose"]),
        "observation_facts": deepcopy(basis["observation_facts"]),
        "hazards": [
            {
                "hypothesis_id": item["hypothesis_id"],
                "scan_evidence_fact_count": len(item["scan_evidence"]),
            }
            for item in basis["hazards"]
        ],
    }


def _optional_code(value, field):
    candidate = value.get(field)
    if candidate is None:
        return None
    if (
        not isinstance(candidate, str)
        or not candidate
        or len(candidate) > 160
        or any(ord(character) < 32 for character in candidate)
    ):
        raise NavigationExperienceError("{} is invalid".format(field))
    return candidate


def _outcome_facts(result: Mapping[str, object]) -> Mapping[str, object]:
    if not isinstance(result, Mapping):
        raise NavigationExperienceError("action result is invalid")
    operation = _optional_code(result, "operation")
    status = _optional_code(result, "status")
    if operation is None or status is None:
        raise NavigationExperienceError("action result is incomplete")
    facts = {
        "operation": operation,
        "status": status,
        "reason_code": _optional_code(result, "reason"),
    }
    for field in (
        "information_gain",
        "evidence_disposition",
        "target_hypothesis_id",
    ):
        value = _optional_code(result, field)
        if value is not None:
            facts[field] = value
    if "bilateral_complete" in result:
        if type(result["bilateral_complete"]) is not bool:
            raise NavigationExperienceError(
                "bilateral scan completion is invalid"
            )
        facts["bilateral_complete"] = result["bilateral_complete"]
    changed = result.get("changed_facts")
    if changed is not None:
        if (
            not isinstance(changed, list)
            or len(changed) > 16
            or any(
                not isinstance(item, str) or not item or len(item) > 80
                for item in changed
            )
        ):
            raise NavigationExperienceError("changed facts are invalid")
        facts["changed_facts"] = list(changed)
    encoder = result.get("encoder_observation")
    if encoder is not None:
        checked = _required_mapping(
            encoder,
            (
                "left_encoder_delta_degrees",
                "right_encoder_delta_degrees",
                "verified_slice_count",
                "requested_slice_count",
                "command_completed",
            ),
            "encoder observation",
        )
        facts["encoder_observation"] = {
            key: checked[key]
            for key in (
                "left_encoder_delta_degrees",
                "right_encoder_delta_degrees",
                "verified_slice_count",
                "requested_slice_count",
                "command_completed",
            )
        }
    validation = result.get("validation")
    if validation is not None:
        if not isinstance(validation, Mapping):
            raise NavigationExperienceError("action validation is invalid")
        code = _optional_code(validation, "code")
        if code is not None:
            facts["validation_code"] = code
    scan_evidence = result.get("scan_evidence")
    if scan_evidence is not None:
        checked = _required_mapping(
            scan_evidence,
            (
                "scan_id",
                "observation_pattern",
                "arc_coverage",
                "boundary_coverage",
                "hypothesis_relation",
            ),
            "scan evidence outcome",
        )
        scan_facts = {
            key: _optional_code(checked, key)
            for key in (
                "scan_id",
                "observation_pattern",
                "arc_coverage",
                "boundary_coverage",
                "hypothesis_relation",
            )
        }
        if any(value is None for value in scan_facts.values()):
            raise NavigationExperienceError(
                "scan evidence outcome is invalid"
            )
        facts["scan_evidence"] = scan_facts
    return facts


def _outcome_classification(
    outcome: Mapping[str, object],
) -> Mapping[str, object]:
    """Return stable categorical facts for a compact result distribution."""

    fields = (
        "operation",
        "status",
        "reason_code",
        "information_gain",
        "evidence_disposition",
        "validation_code",
        "bilateral_complete",
        "target_hypothesis_id",
    )
    return {
        field: deepcopy(outcome[field])
        for field in fields
        if field in outcome
    }


def _attempt_identity(
    action: str,
    outcome: Mapping[str, object],
) -> Mapping[str, object]:
    """Return the typed parameters that make one attempt comparable.

    Most physical actions have no planner-selected parameter.  An active scan
    does: scanning hazard A is not a retry of scanning hazard B, even when the
    robot and all other evidence are unchanged.
    """

    value = {"action": action}
    if action == SCAN_FRONT_ARC:
        value["target_hypothesis_id"] = outcome.get(
            "target_hypothesis_id"
        )
    return value


def _rollup_outcome(outcome: Mapping[str, object]) -> Mapping[str, object]:
    """Return the bounded typed latest-outcome view used by rollups."""

    return _outcome_classification(outcome)


@dataclass(frozen=True)
class NavigationExperience:
    sequence: int
    turn: int
    action: str
    source: str
    attempt_identity: Mapping[str, object]
    basis_before_id: str
    basis_after_id: str
    basis_before: Mapping[str, object]
    basis_after: Mapping[str, object]
    basis_change_codes: Tuple[str, ...]
    attempt_relation: str
    prior_same_action_sequence: Optional[int]
    prior_same_basis_sequence: Optional[int]
    outcome: Mapping[str, object]

    def to_dict(self) -> Mapping[str, object]:
        return {
            "sequence": self.sequence,
            "turn": self.turn,
            "action": self.action,
            "source": self.source,
            "attempt_identity": deepcopy(self.attempt_identity),
            "basis_before_id": self.basis_before_id,
            "basis_after_id": self.basis_after_id,
            "basis_before": deepcopy(self.basis_before),
            "basis_after": deepcopy(self.basis_after),
            "basis_change_codes": list(self.basis_change_codes),
            "attempt_relation": self.attempt_relation,
            "prior_same_action_sequence": self.prior_same_action_sequence,
            "prior_same_basis_sequence": self.prior_same_basis_sequence,
            "outcome": deepcopy(self.outcome),
        }


class NavigationExperienceLedger:
    """Episode-scoped factual action/result history with a byte ceiling."""

    def __init__(
        self,
        *,
        episode_id: str,
        max_entries: int = MAX_EXPERIENCE_ENTRIES,
    ):
        if (
            not isinstance(episode_id, str)
            or not episode_id
            or len(episode_id) > 128
            or any(ord(character) < 32 for character in episode_id)
            or isinstance(max_entries, bool)
            or not isinstance(max_entries, int)
            or not 1 <= max_entries <= MAX_EXPERIENCE_ENTRIES
        ):
            raise NavigationExperienceError(
                "experience ledger config is invalid"
            )
        self.episode_id = episode_id
        self.max_entries = max_entries
        self._entries = []
        self._sequence = 0
        self._last_action_sequence = {}
        self._last_attempt_sequence = {}
        self._seen_attempt_basis = OrderedDict()
        self._action_basis_rollups = OrderedDict()

    @property
    def entries(self) -> Tuple[NavigationExperience, ...]:
        return tuple(self._entries)

    def record(
        self,
        *,
        turn: int,
        action: str,
        source: str,
        result: Mapping[str, object],
        basis_before: Mapping[str, object],
        basis_after: Mapping[str, object],
    ) -> NavigationExperience:
        if (
            isinstance(turn, bool)
            or not isinstance(turn, int)
            or turn <= 0
            or action not in ACTIONS
            or source not in ACTION_SOURCES
        ):
            raise NavigationExperienceError("experience identity is invalid")
        before_id = _basis_id(basis_before)
        after_id = _basis_id(basis_after)
        outcome = _outcome_facts(result)
        attempt_identity = _attempt_identity(action, outcome)
        attempt_key = json_bytes(attempt_identity)
        prior_sequence = self._last_action_sequence.get(action)
        prior_attempt_sequence = self._last_attempt_sequence.get(attempt_key)
        seen_attempt_key = (attempt_key, before_id)
        prior_same_basis_sequence = self._seen_attempt_basis.get(
            seen_attempt_key
        )
        if prior_attempt_sequence is None:
            relation = FIRST_ATTEMPT
        elif prior_same_basis_sequence is not None:
            relation = UNCHANGED_BASIS_REPEAT
        else:
            relation = RETRY_AFTER_BASIS_CHANGE
        self._sequence += 1
        entry = NavigationExperience(
            sequence=self._sequence,
            turn=turn,
            action=action,
            source=source,
            attempt_identity=attempt_identity,
            basis_before_id=before_id,
            basis_after_id=after_id,
            basis_before=_basis_summary(basis_before),
            basis_after=_basis_summary(basis_after),
            basis_change_codes=_basis_change_codes(
                basis_before,
                basis_after,
            ),
            attempt_relation=relation,
            prior_same_action_sequence=(
                prior_sequence
            ),
            prior_same_basis_sequence=(
                prior_same_basis_sequence
            ),
            outcome=outcome,
        )
        self._entries.append(entry)
        self._entries = self._entries[-self.max_entries:]
        self._last_action_sequence[action] = entry.sequence
        self._last_attempt_sequence[attempt_key] = entry.sequence
        self._seen_attempt_basis[seen_attempt_key] = entry.sequence
        self._seen_attempt_basis.move_to_end(seen_attempt_key)
        while len(self._seen_attempt_basis) > MAX_EXPERIENCE_SEEN_KEYS:
            self._seen_attempt_basis.popitem(last=False)
        seen_key = (action, before_id)
        prior_rollup = self._action_basis_rollups.get(seen_key)
        classification = _outcome_classification(outcome)
        classification_key = json_bytes(classification)
        if prior_rollup is None:
            prior_rollup = {
                "attempt_count": 0,
                "first_sequence": entry.sequence,
                "latest_sequence": entry.sequence,
                "outcome_distribution": OrderedDict(),
                "latest_outcome": None,
            }
        prior_rollup["attempt_count"] += 1
        prior_rollup["latest_sequence"] = entry.sequence
        distribution = prior_rollup["outcome_distribution"]
        bucket = distribution.get(classification_key)
        if bucket is None:
            bucket = {"outcome": classification, "count": 0}
            distribution[classification_key] = bucket
        bucket["count"] += 1
        prior_rollup["latest_outcome"] = deepcopy(outcome)
        self._action_basis_rollups[seen_key] = prior_rollup
        self._action_basis_rollups.move_to_end(seen_key)
        while len(self._action_basis_rollups) > MAX_EXPERIENCE_SEEN_KEYS:
            self._action_basis_rollups.popitem(last=False)
        return entry

    def _current_basis_action_rollups(
        self,
        current_basis_id: str,
        retained_bucket_keys=None,
    ) -> list[Mapping[str, object]]:
        retained_bucket_keys = retained_bucket_keys or {}
        values = []
        for action in sorted(ACTIONS):
            rollup = self._action_basis_rollups.get(
                (action, current_basis_id)
            )
            if rollup is None:
                values.append({
                    "action": action,
                    "attempt_count": 0,
                    "first_sequence": None,
                    "latest_sequence": None,
                    "outcome_bucket_count": 0,
                    "outcome_bucket_retained_count": 0,
                    "outcome_bucket_omitted_count": 0,
                    "outcome_attempt_retained_count": 0,
                    "outcome_attempt_omitted_count": 0,
                    "outcome_distribution": [],
                    "latest_outcome": None,
                })
                continue
            distribution = rollup["outcome_distribution"]
            selected = retained_bucket_keys.get(action, frozenset())
            retained = [
                deepcopy(distribution[key])
                for key in sorted(distribution)
                if key in selected
            ]
            retained_attempts = sum(item["count"] for item in retained)
            values.append({
                "action": action,
                "attempt_count": rollup["attempt_count"],
                "first_sequence": rollup["first_sequence"],
                "latest_sequence": rollup["latest_sequence"],
                "outcome_bucket_count": len(distribution),
                "outcome_bucket_retained_count": len(retained),
                "outcome_bucket_omitted_count": (
                    len(distribution) - len(retained)
                ),
                "outcome_attempt_retained_count": retained_attempts,
                "outcome_attempt_omitted_count": (
                    rollup["attempt_count"] - retained_attempts
                ),
                "outcome_distribution": retained,
                "latest_outcome": _rollup_outcome(
                    rollup["latest_outcome"]
                ),
            })
        return values

    def _current_basis_bucket_candidates(self, current_basis_id: str):
        values = []
        for action in sorted(ACTIONS):
            rollup = self._action_basis_rollups.get(
                (action, current_basis_id)
            )
            if rollup is None:
                continue
            values.extend(
                (action, key)
                for key in sorted(rollup["outcome_distribution"])
            )
        return values

    def context(
        self,
        *,
        current_basis: Mapping[str, object],
    ) -> Mapping[str, object]:
        entries = [item.to_dict() for item in self._entries]
        current_basis_id = _basis_id(current_basis)
        retained_bucket_keys = {}

        def value_with(entries_value):
            current_rollups = self._current_basis_action_rollups(
                current_basis_id,
                retained_bucket_keys,
            )
            return {
                "schema": EXPERIENCE_LEDGER_SCHEMA,
                "episode_id": self.episode_id,
                "scope": "EPISODE",
                "persisted": False,
                "host_ranked_or_selected_action": False,
                "capacity": self.max_entries,
                "retained_count": len(entries_value),
                "total_recorded_count": self._sequence,
                "seen_action_basis_capacity": MAX_EXPERIENCE_SEEN_KEYS,
                "seen_action_basis_retained_count": len(
                    self._action_basis_rollups
                ),
                "seen_attempt_basis_capacity": MAX_EXPERIENCE_SEEN_KEYS,
                "seen_attempt_basis_retained_count": len(
                    self._seen_attempt_basis
                ),
                "current_basis_id": current_basis_id,
                "current_basis_action_rollups": current_rollups,
                "entries": entries_value,
            }

        while True:
            value = value_with(entries)
            if len(json_bytes(value)) <= MAX_EXPERIENCE_CONTEXT_BYTES:
                break
            if not entries:
                raise NavigationExperienceError(
                    "experience context exceeded its byte limit"
                )
            entries = entries[1:]

        for action, key in self._current_basis_bucket_candidates(
            current_basis_id
        ):
            selected = retained_bucket_keys.setdefault(action, set())
            selected.add(key)
            candidate = value_with(entries)
            if len(json_bytes(candidate)) <= MAX_EXPERIENCE_CONTEXT_BYTES:
                value = candidate
                continue
            selected.remove(key)
            # Stable prefix retention keeps the projection deterministic and
            # avoids a size-based semantic ranking of later outcome buckets.
            break
        return deepcopy(value)


__all__ = (
    "ACTION_SOURCES",
    "EXPERIENCE_BASIS_SCHEMA",
    "EXPERIENCE_LEDGER_SCHEMA",
    "FIRST_ATTEMPT",
    "MAX_EXPERIENCE_CONTEXT_BYTES",
    "MAX_EXPERIENCE_ENTRIES",
    "MAX_EXPERIENCE_SEEN_KEYS",
    "NavigationExperience",
    "NavigationExperienceError",
    "NavigationExperienceLedger",
    "PLANNER_ACTION_SOURCE",
    "PLAN_TAIL_ACTION_SOURCE",
    "RETRY_AFTER_BASIS_CHANGE",
    "UNCHANGED_BASIS_REPEAT",
    "navigation_evidence_basis",
)
