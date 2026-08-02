"""Bounded, identity-free context for semantic navigation planning."""

from copy import deepcopy
from dataclasses import dataclass
import json
from typing import Mapping, Optional, Tuple

from .navigation_intent_proposal import (
    MAX_NAVIGATION_INTENT_SCHEMA_BYTES,
    NavigationIntentOffer,
    build_navigation_intent_proposal_schema,
)
from .physical_agent_state import (
    ActiveIntent,
    DetourTargetIntent,
    FollowDirectionIntent,
    GoalAssignment,
    IntentProgress,
    ScanTargetIntent,
)


MAX_NAVIGATION_INTENT_CONTEXT_BYTES = 24 * 1024
MAX_NAVIGATION_INTENT_ACCOUNTED_BYTES = 32 * 1024
NAVIGATION_INTENT_WRAPPER_RESERVE_BYTES = 2 * 1024

SYSTEM_PROMPT = """Choose exactly one semantic navigation intent for a harmless LEGO robot.
The host compiles it into short steps and verifies the results. Do not output
motor commands, timings, speech, invented measurements, or internal ticket,
controller, goal, or state IDs. Schema-provided target IDs are allowed.
FOLLOW_DIRECTION pursues the goal. SCAN_TARGET gathers missing evidence about
an offered target. DETOUR_TARGET chooses an offered target and supported side.
HOLD pauses for an offered factual reason. ABORT ends the goal for an offered
reason. Use only schema values. A clear forward reading does not erase a
remembered hazard beside the robot. Do not repeat a tactic when its progress
budget or evidence says it is stuck. Return one matching JSON object only."""


class NavigationIntentContextError(ValueError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _json_bytes(value: object) -> bytes:
    try:
        return json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    except (TypeError, ValueError):
        raise NavigationIntentContextError(
            "non_json_context",
            "Navigation intent context must contain finite JSON values",
        ) from None


def _required_mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise NavigationIntentContextError(
            "invalid_{}".format(name),
            "{} must be a mapping".format(name.replace("_", " ")),
        )
    return value


def _required_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise NavigationIntentContextError(
            "invalid_{}".format(name),
            "{} must be an integer".format(name.replace("_", " ")),
        )
    return value


def _required_bool(value: object, name: str) -> bool:
    if type(value) is not bool:
        raise NavigationIntentContextError(
            "invalid_{}".format(name),
            "{} must be a boolean".format(name.replace("_", " ")),
        )
    return value


def _optional_scalar(value: object):
    if value is None or isinstance(value, (str, int, bool)):
        return value
    return None


def _mission_context(mission: Mapping[str, object]) -> Mapping[str, object]:
    integers = (
        "current_longitudinal_progress_mm",
        "remaining_longitudinal_progress_mm",
        "regression_from_peak_mm",
        "lateral_offset_mm",
    )
    booleans = (
        "goal_heading_aligned",
        "goal_corridor_clear",
        "all_known_hazards_passed",
        "localization_valid",
        "touch_clear",
        "completed",
    )
    return {
        **{
            name: _required_int(mission.get(name), name)
            for name in integers
        },
        **{
            name: _required_bool(mission.get(name), name)
            for name in booleans
        },
    }


def _intent_context(intent: Optional[ActiveIntent]):
    if intent is None:
        return None
    if not isinstance(intent, ActiveIntent):
        raise NavigationIntentContextError(
            "invalid_active_intent", "Active intent is not canonical"
        )
    payload = intent.payload
    if isinstance(payload, FollowDirectionIntent):
        value = {"kind": "FOLLOW_DIRECTION"}
    elif isinstance(payload, ScanTargetIntent):
        value = {
            "kind": "SCAN_TARGET",
            "target_id": payload.target_hypothesis_id,
            "scan_profile": payload.scan_profile_id,
        }
    elif isinstance(payload, DetourTargetIntent):
        value = {
            "kind": "DETOUR_TARGET",
            "target_id": payload.target_hypothesis_id,
            "side": payload.detour_side.value,
        }
    else:
        raise NavigationIntentContextError(
            "invalid_active_intent", "Active intent payload is unsupported"
        )
    value["revision"] = intent.revision
    value["policy"] = {
        "max_plan_attempts": intent.policy.max_plan_attempts,
        "max_consecutive_no_progress_plans": (
            intent.policy.max_consecutive_no_progress_plans
        ),
    }
    return value


def _progress_context(progress: Optional[IntentProgress]):
    if progress is None:
        return None
    if not isinstance(progress, IntentProgress):
        raise NavigationIntentContextError(
            "invalid_intent_progress", "Intent progress is not canonical"
        )
    return {
        "plan_attempts": progress.plan_attempts,
        "completed_steps": progress.completed_steps,
        "consecutive_no_progress_plans": (
            progress.consecutive_no_progress_plans
        ),
    }


def _latest_scan_context(history: object):
    if not isinstance(history, list) or not history:
        return None
    candidates = [item for item in history if isinstance(item, Mapping)]
    if not candidates:
        return None
    latest = max(
        candidates,
        key=lambda item: (
            item.get("completed_at_ms")
            if isinstance(item.get("completed_at_ms"), int)
            and not isinstance(item.get("completed_at_ms"), bool)
            else -1
        ),
    )
    return {
        name: _optional_scalar(latest.get(name))
        for name in (
            "status",
            "observation_pattern",
            "arc_coverage",
            "boundary_coverage",
            "hypothesis_relation",
            "left_boundary_mdeg",
            "right_boundary_mdeg",
        )
    }


def _target_context(
    navigation: Mapping[str, object],
    offer: NavigationIntentOffer,
) -> Tuple[Mapping[str, object], ...]:
    hazards = navigation.get("navigation_hazard_hypotheses")
    if not isinstance(hazards, list) or any(
        not isinstance(item, Mapping) for item in hazards
    ):
        raise NavigationIntentContextError(
            "invalid_navigation_hazards",
            "Navigation hazards must be a list of mappings",
        )
    by_id = {}
    for item in hazards:
        identifier = item.get("hypothesis_id")
        if not isinstance(identifier, str) or not identifier or identifier in by_id:
            raise NavigationIntentContextError(
                "invalid_navigation_hazards",
                "Navigation hazard identities must be unique",
            )
        by_id[identifier] = item
    selected_ids = tuple(sorted(set(
        offer.scan_target_ids + offer.detour_target_ids
    )))
    missing = [identifier for identifier in selected_ids if identifier not in by_id]
    if missing:
        raise NavigationIntentContextError(
            "offered_target_missing",
            "Every offered target needs current evidence",
        )
    conflicts = {
        item.get("hypothesis_id")
        for item in _required_mapping(
            navigation.get("goal_geometry"), "goal_geometry"
        ).get("conflicts", [])
        if isinstance(item, Mapping)
        and item.get("active_for_collision") is True
    }
    result = []
    for identifier in selected_ids:
        hazard = by_id[identifier]
        route = hazard.get("route_evidence")
        route = route if isinstance(route, Mapping) else {}
        result.append({
            "target_id": identifier,
            "goal_conflict": identifier in conflicts,
            "active_for_collision": _required_bool(
                hazard.get("active_for_collision"),
                "hazard_active_for_collision",
            ),
            "geometry_mm": {
                name: _optional_scalar(hazard.get(name))
                for name in (
                    "centroid_x_mm",
                    "centroid_y_mm",
                    "radius_mm",
                )
            },
            "collision_support_count": _optional_scalar(
                hazard.get("collision_support_count")
            ),
            "route_ready": _required_bool(
                hazard.get("route_commitment_ready"),
                "route_commitment_ready",
            ),
            "route_evidence_reason": _optional_scalar(route.get("reason")),
            "latest_scan": _latest_scan_context(
                hazard.get("scan_evidence_history")
            ),
        })
    return tuple(result)


def _outcome_context(value: Optional[Mapping[str, object]]):
    if value is None:
        return None
    value = _required_mapping(value, "latest_outcome")
    return {
        name: _optional_scalar(value.get(name))
        for name in (
            "operation",
            "status",
            "reason",
            "target_hypothesis_id",
            "information_gain",
        )
    }


@dataclass(frozen=True)
class NavigationIntentPrompt:
    system_prompt: str
    context: Mapping[str, object]
    response_schema: Mapping[str, object]
    context_bytes: int
    accounted_bytes: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "context", deepcopy(dict(self.context)))
        object.__setattr__(
            self, "response_schema", deepcopy(dict(self.response_schema))
        )


def build_navigation_intent_prompt(
    *,
    goal: GoalAssignment,
    mission: Mapping[str, object],
    navigation: Mapping[str, object],
    offer: NavigationIntentOffer,
    active_intent: Optional[ActiveIntent] = None,
    intent_progress: Optional[IntentProgress] = None,
    latest_outcome: Optional[Mapping[str, object]] = None,
) -> NavigationIntentPrompt:
    """Project rich host memory into one small, causally relevant prompt."""

    if not isinstance(goal, GoalAssignment):
        raise NavigationIntentContextError(
            "invalid_goal", "Goal must be canonical"
        )
    if not isinstance(offer, NavigationIntentOffer):
        raise NavigationIntentContextError(
            "invalid_offer", "Offer must be canonical"
        )
    if goal.goal_epoch != offer.basis.goal_epoch:
        raise NavigationIntentContextError(
            "goal_offer_mismatch", "Goal and offer epochs differ"
        )
    if (active_intent is None) != (intent_progress is None):
        raise NavigationIntentContextError(
            "intent_progress_mismatch",
            "Active intent and progress must be supplied together",
        )
    mission = _required_mapping(mission, "mission")
    navigation = _required_mapping(navigation, "navigation")
    context = {
        "objective": goal.objective,
        "locale": goal.locale,
        "mission": _mission_context(mission),
        "pose": {
            name: _required_int(
                _required_mapping(
                    navigation.get("pose"), "navigation_pose"
                ).get(name),
                "pose_{}".format(name),
            )
            for name in ("x_mm", "y_mm", "heading_mdeg")
        },
        "known_hazard_count": len(
            navigation.get("navigation_hazard_hypotheses", [])
        ),
        "active_intent": _intent_context(active_intent),
        "intent_progress": _progress_context(intent_progress),
        "offered_target_evidence": list(_target_context(navigation, offer)),
        "latest_outcome": _outcome_context(latest_outcome),
    }
    context_bytes = _json_bytes(context)
    schema = build_navigation_intent_proposal_schema(offer)
    schema_bytes = _json_bytes(schema)
    accounted = (
        len(SYSTEM_PROMPT.encode("utf-8"))
        + len(schema_bytes)
        + len(context_bytes)
        + NAVIGATION_INTENT_WRAPPER_RESERVE_BYTES
    )
    if len(schema_bytes) > MAX_NAVIGATION_INTENT_SCHEMA_BYTES:
        raise NavigationIntentContextError(
            "intent_schema_too_large", "Intent schema exceeds its budget"
        )
    if len(context_bytes) > MAX_NAVIGATION_INTENT_CONTEXT_BYTES:
        raise NavigationIntentContextError(
            "intent_context_too_large", "Intent context exceeds its budget"
        )
    if accounted > MAX_NAVIGATION_INTENT_ACCOUNTED_BYTES:
        raise NavigationIntentContextError(
            "intent_prompt_too_large", "Intent prompt exceeds its budget"
        )
    return NavigationIntentPrompt(
        system_prompt=SYSTEM_PROMPT,
        context=context,
        response_schema=schema,
        context_bytes=len(context_bytes),
        accounted_bytes=accounted,
    )


__all__ = (
    "MAX_NAVIGATION_INTENT_ACCOUNTED_BYTES",
    "MAX_NAVIGATION_INTENT_CONTEXT_BYTES",
    "NAVIGATION_INTENT_WRAPPER_RESERVE_BYTES",
    "NavigationIntentContextError",
    "NavigationIntentPrompt",
    "SYSTEM_PROMPT",
    "build_navigation_intent_prompt",
)
