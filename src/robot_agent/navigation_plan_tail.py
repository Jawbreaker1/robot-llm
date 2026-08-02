"""Freshness guards for exact model-authored conditional motion tails."""

from copy import deepcopy
from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

from .physical_navigation_contract import (
    MOTION_ACTIONS,
    REVERSE,
    NavigationDecision,
    observation_safety_signature,
)


PLAN_TAIL_MAX_AGE_SECONDS = 8.0


def _active_commitment(
    maneuver_state: Mapping[str, object],
) -> Optional[Mapping[str, object]]:
    active = maneuver_state.get("active")
    return None if active is None else deepcopy(active)


def _focus_truth(
    maneuver_state: Mapping[str, object],
    fact_values: Mapping[str, object],
) -> Tuple[Optional[str], object]:
    active = maneuver_state.get("active")
    if active is None:
        return (None, None)
    focus = active["current_focus_fact_key"]
    target = active["target_hypothesis_id"]
    value = fact_values.get(focus)
    if isinstance(value, dict):
        value = value.get(target)
    return (focus, value)


@dataclass
class NavigationPlanTail:
    source_turn: int
    source_plan: Tuple[str, ...]
    remaining_actions: Tuple[str, ...]
    created_monotonic: float
    expires_monotonic: float
    map_generation_id: str
    last_map_version: int
    last_observation_state_version: int
    hazard_ids: Tuple[str, ...]
    safety_signature: Tuple[bool, bool, bool]
    active_commitment: Optional[Mapping[str, object]]
    focus_truth: Tuple[Optional[str], object]
    scan_staging_target_ids: Tuple[str, ...]
    cancelled_reason: Optional[str] = None

    @classmethod
    def from_decision(
        cls,
        decision: NavigationDecision,
        *,
        now_monotonic: float,
        episode_deadline: float,
        map_context: Mapping[str, object],
        observation: Mapping[str, object],
        maneuver_state: Mapping[str, object],
        fact_values: Mapping[str, object],
        max_age_seconds: float = PLAN_TAIL_MAX_AGE_SECONDS,
    ) -> Optional["NavigationPlanTail"]:
        remaining = decision.plan[1:]
        if not remaining:
            return None
        if any(action not in MOTION_ACTIONS for action in decision.plan):
            raise ValueError("plan tail may contain only motion actions")
        if (
            isinstance(max_age_seconds, bool)
            or not isinstance(max_age_seconds, (int, float))
            or not 1.0 <= float(max_age_seconds) <= 120.0
        ):
            raise ValueError("plan tail age is invalid")
        hazards = tuple(
            sorted(
                item["hypothesis_id"]
                for item in map_context[
                    "navigation_hazard_hypotheses"
                ]
            )
        )
        return cls(
            source_turn=decision.turn,
            source_plan=decision.plan,
            remaining_actions=remaining,
            created_monotonic=now_monotonic,
            expires_monotonic=min(
                episode_deadline,
                now_monotonic + float(max_age_seconds),
            ),
            map_generation_id=map_context["map_generation_id"],
            last_map_version=map_context["map_version"],
            last_observation_state_version=observation["state_version"],
            hazard_ids=hazards,
            safety_signature=observation_safety_signature(observation),
            active_commitment=_active_commitment(maneuver_state),
            focus_truth=_focus_truth(maneuver_state, fact_values),
            scan_staging_target_ids=(
                tuple(sorted(map_context.get(
                    "detour_scan_required_target_hypothesis_ids",
                    (),
                )))
                if decision.action == REVERSE
                else ()
            ),
        )

    @property
    def complete(self) -> bool:
        return not self.remaining_actions

    @property
    def cancelled(self) -> bool:
        return self.cancelled_reason is not None

    def guard_failures(
        self,
        *,
        now_monotonic: float,
        map_context: Mapping[str, object],
        observation: Mapping[str, object],
        maneuver_state: Mapping[str, object],
        fact_values: Mapping[str, object],
        localization_valid: bool,
    ) -> Tuple[str, ...]:
        failures = []
        if self.cancelled:
            failures.append("plan_tail_already_cancelled")
        if now_monotonic >= self.expires_monotonic:
            failures.append("plan_tail_deadline_elapsed")
        if localization_valid is not True:
            failures.append("plan_tail_localization_invalid")
        if map_context["map_generation_id"] != self.map_generation_id:
            failures.append("plan_tail_map_generation_changed")
        if map_context["map_version"] <= self.last_map_version:
            failures.append("plan_tail_memory_not_updated")
        if (
            observation["state_version"]
            <= self.last_observation_state_version
        ):
            failures.append("plan_tail_observation_not_fresh")
        hazard_ids = tuple(
            sorted(
                item["hypothesis_id"]
                for item in map_context[
                    "navigation_hazard_hypotheses"
                ]
            )
        )
        if hazard_ids != self.hazard_ids:
            failures.append("plan_tail_hazard_set_changed")
        if (
            observation_safety_signature(observation)
            != self.safety_signature
        ):
            failures.append("plan_tail_touch_ir_safety_changed")
        if _active_commitment(maneuver_state) != self.active_commitment:
            failures.append("plan_tail_commitment_changed")
        if _focus_truth(maneuver_state, fact_values) != self.focus_truth:
            failures.append("plan_tail_focus_truth_changed")
        scan_feasibility = map_context.get("action_feasibility", {}).get(
            "active_scan",
            {},
        )
        if (
            self.scan_staging_target_ids
            and scan_feasibility.get("allowed") is True
        ):
            failures.append("plan_tail_scan_staging_complete")
        return tuple(failures)

    def next_action(
        self,
        *,
        now_monotonic: float,
        map_context: Mapping[str, object],
        observation: Mapping[str, object],
        maneuver_state: Mapping[str, object],
        fact_values: Mapping[str, object],
        localization_valid: bool,
    ) -> Optional[str]:
        failures = self.guard_failures(
            now_monotonic=now_monotonic,
            map_context=map_context,
            observation=observation,
            maneuver_state=maneuver_state,
            fact_values=fact_values,
            localization_valid=localization_valid,
        )
        if failures:
            self.cancelled_reason = ",".join(failures)
            return None
        return None if self.complete else self.remaining_actions[0]

    def mark_executed(
        self,
        action: str,
        *,
        map_context: Mapping[str, object],
        observation: Mapping[str, object],
    ) -> None:
        if self.cancelled or self.complete:
            raise ValueError("plan tail is not executable")
        if action != self.remaining_actions[0]:
            raise ValueError("plan tail action cannot be reordered or substituted")
        self.remaining_actions = self.remaining_actions[1:]
        self.last_map_version = map_context["map_version"]
        self.last_observation_state_version = observation["state_version"]

    def cancel(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason:
            raise ValueError("plan tail cancellation reason is invalid")
        self.cancelled_reason = reason
