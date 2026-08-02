"""One deterministic host gate for physical navigation actions.

The gate owns no policy for *which* allowed action is preferable.  It only
publishes the actions that are currently admissible and repeats the
time-sensitive checks immediately before a motion is dispatched.  The EV3
worker remains a separate, controller-local safety boundary.
"""

from copy import deepcopy
from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

from . import physical_action_feasibility
from .physical_navigation_contract import (
    ACTIONS,
    FINISH,
    MOTION_ACTIONS,
    motion_budget_allows,
)


HOST_PER_SLICE_HEADROOM_SECONDS = 0.25
HOST_RESPONSE_HEADROOM_SECONDS = 0.75


class PhysicalActionGateError(ValueError):
    """The gate received an invalid snapshot or action."""


@dataclass(frozen=True)
class PhysicalActionAvailability:
    """A planner view produced without selecting or ranking an action."""

    navigation: Mapping[str, object]
    actions: Tuple[str, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.navigation, Mapping):
            raise PhysicalActionGateError("gate navigation is invalid")
        if (
            not isinstance(self.actions, tuple)
            or len(set(self.actions)) != len(self.actions)
        ):
            raise PhysicalActionGateError("gate actions are invalid")
        object.__setattr__(self, "navigation", deepcopy(dict(self.navigation)))


@dataclass(frozen=True)
class PhysicalActionGateDecision:
    """The single host-level allow/deny result for one motion proposal."""

    action: str
    allowed: bool
    reason_code: Optional[str]
    details: Optional[Mapping[str, object]] = None

    def __post_init__(self) -> None:
        if self.action not in ACTIONS:
            raise PhysicalActionGateError("gate action is invalid")
        if type(self.allowed) is not bool:
            raise PhysicalActionGateError("gate disposition is invalid")
        if self.allowed:
            if self.reason_code is not None or self.details is not None:
                raise PhysicalActionGateError(
                    "allowed gate decision cannot contain a veto"
                )
            return
        if (
            not isinstance(self.reason_code, str)
            or not self.reason_code
            or len(self.reason_code) > 160
            or any(ord(character) < 32 for character in self.reason_code)
        ):
            raise PhysicalActionGateError("gate reason is invalid")
        if self.details is not None:
            if not isinstance(self.details, Mapping):
                raise PhysicalActionGateError("gate details are invalid")
            object.__setattr__(self, "details", deepcopy(dict(self.details)))

    def veto_mapping(self) -> Optional[Mapping[str, object]]:
        if self.allowed:
            return None
        value = {
            "code": self.reason_code,
            "action": self.action,
            "host_selected_alternative_action": False,
        }
        if self.details:
            value.update(deepcopy(dict(self.details)))
        return value


class PhysicalNavigationActionGate:
    """Compose the existing pure rules behind one public gate boundary."""

    def describe_navigation_feasibility(
        self,
        *,
        hazard_map,
        pose,
        action_specs,
        odometry_calibration,
        active_scan_calibration,
    ) -> Mapping[str, object]:
        """Publish the gate's detached geometric feasibility snapshot."""

        return deepcopy(
            physical_action_feasibility.navigation_action_feasibility(
                hazard_map=hazard_map,
                pose=pose,
                action_specs=action_specs,
                odometry_calibration=odometry_calibration,
                active_scan_calibration=active_scan_calibration,
            )
        )

    def evaluate_planner_decision(
        self,
        decision,
        *,
        mission: Mapping[str, object],
        navigation: Mapping[str, object],
    ) -> PhysicalActionGateDecision:
        """Validate one legacy planner proposal without choosing a substitute."""

        action = getattr(decision, "action", None)
        if action not in ACTIONS:
            raise PhysicalActionGateError("gate action is invalid")
        if action == FINISH:
            if (
                getattr(decision, "plan", None) != (FINISH,)
                or getattr(decision, "reason_code", None) != "COMPLETE_GOAL"
                or mission.get("completed") is not True
            ):
                return PhysicalActionGateDecision(
                    action=action,
                    allowed=False,
                    reason_code="premature_mission_finish",
                    details={
                        "message": "FINISH requires every directional mission fact"
                    },
                )
        elif getattr(decision, "reason_code", None) == "COMPLETE_GOAL":
            return PhysicalActionGateDecision(
                action=action,
                allowed=False,
                reason_code="nonterminal_complete_reason",
                details={"message": "COMPLETE_GOAL is valid only with FINISH"},
            )

        delta = mission["candidate_action_longitudinal_deltas_mm"].get(
            action
        )
        heading_recovery = delta == 0 and mission[
            "projected_goal_heading_aligned_after_action"
        ].get(action) is True
        if getattr(decision, "reason_code", None) == "PROGRESS_GOAL" and (
            delta is None
            or delta < 0
            or (delta == 0 and not heading_recovery)
        ):
            return PhysicalActionGateDecision(
                action=action,
                allowed=False,
                reason_code="nonprogress_action_reason",
                details={
                    "message": (
                        "PROGRESS_GOAL contradicts published mission arithmetic"
                    )
                },
            )
        if (
            delta is not None
            and delta < 0
            and not navigation["navigation_hazard_hypotheses"]
        ):
            return PhysicalActionGateDecision(
                action=action,
                allowed=False,
                reason_code="regression_without_hazard",
                details={"message": "Negative progress requires a published hazard"},
            )
        detour_error = physical_action_feasibility.detour_decision_error(
            action,
            getattr(decision, "perception_target_hypothesis_id", None),
            getattr(decision, "maneuver_commitment", None),
            navigation,
        )
        if detour_error is not None:
            return PhysicalActionGateDecision(
                action=action,
                allowed=False,
                reason_code=detour_error[0],
                details={"message": detour_error[1]},
            )
        return PhysicalActionGateDecision(
            action=action,
            allowed=True,
            reason_code=None,
        )

    def prepare(
        self,
        navigation: Mapping[str, object],
        *,
        active_maneuver,
        scan_eligible_target_ids,
        scan_blocked_target_ids,
        scan_budget_available: bool,
        reverse_budget_available: bool,
        action_specs,
        observation,
        repeated_uninformative_observe: bool,
    ) -> PhysicalActionAvailability:
        if not isinstance(navigation, Mapping):
            raise PhysicalActionGateError("gate navigation is invalid")
        planner_navigation = deepcopy(dict(navigation))
        actions = physical_action_feasibility.prepare_navigation_availability(
            planner_navigation,
            active_maneuver=active_maneuver,
            scan_eligible_target_ids=scan_eligible_target_ids,
            scan_blocked_target_ids=scan_blocked_target_ids,
            scan_budget_available=scan_budget_available,
            reverse_budget_available=reverse_budget_available,
            action_specs=action_specs,
            observation=observation,
            repeated_uninformative_observe=repeated_uninformative_observe,
        )
        return PhysicalActionAvailability(
            navigation=planner_navigation,
            actions=tuple(actions),
        )

    def evaluate_motion(
        self,
        action: str,
        *,
        observation: Mapping[str, object],
        action_specs: Mapping[str, Mapping[str, object]],
        hazard_map,
        pose,
        odometry_calibration,
        remaining_seconds: float,
    ) -> PhysicalActionGateDecision:
        if action not in MOTION_ACTIONS:
            raise PhysicalActionGateError("gate action is invalid")
        if (
            isinstance(remaining_seconds, bool)
            or not isinstance(remaining_seconds, (int, float))
        ):
            raise PhysicalActionGateError("gate deadline is invalid")
        if not motion_budget_allows(action, observation, action_specs):
            return PhysicalActionGateDecision(
                action=action,
                allowed=False,
                reason_code="worker_budget_insufficient",
            )
        swept = hazard_map.validate_swept_path(
            pose,
            action,
            action_specs,
            odometry_calibration,
        )
        if not swept["allowed"]:
            return PhysicalActionGateDecision(
                action=action,
                allowed=False,
                reason_code=swept["reason"],
                details={"swept_path": swept},
            )
        spec = action_specs[action]
        required = (
            spec["total_duration_ms"] / 1000.0
            + spec["slice_count"] * HOST_PER_SLICE_HEADROOM_SECONDS
            + HOST_RESPONSE_HEADROOM_SECONDS
        )
        if float(remaining_seconds) < required:
            return PhysicalActionGateDecision(
                action=action,
                allowed=False,
                reason_code="host_deadline_headroom_insufficient",
                details={"required_seconds": required},
            )
        return PhysicalActionGateDecision(
            action=action,
            allowed=True,
            reason_code=None,
        )


__all__ = (
    "HOST_PER_SLICE_HEADROOM_SECONDS",
    "HOST_RESPONSE_HEADROOM_SECONDS",
    "PhysicalActionAvailability",
    "PhysicalActionGateDecision",
    "PhysicalActionGateError",
    "PhysicalNavigationActionGate",
)
