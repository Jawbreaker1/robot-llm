"""Bounded evidence-recovery state for one BLAST navigation episode."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Mapping

from .physical_navigation_contract import (
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from .blast_navigation_state import PlannerNavigationState
from .blast_side_search_geometry import (
    maximum_side_search_required_slots,
    recovery_rebase_completed,
    recovery_rebase_waypoint,
    side_search_required_slots,
)


ROUTE_BINDING_UNAVAILABLE = "detour_route_unavailable"
TARGET_REACQUISITION_COLLECTED = (
    "target_reacquisition_observation_collected"
)
TARGET_REACQUISITION_UNRESOLVED = "target_reacquisition_unresolved"
RECOVERABLE_EVIDENCE_REASONS = frozenset((
    ROUTE_BINDING_UNAVAILABLE,
    TARGET_REACQUISITION_COLLECTED,
    TARGET_REACQUISITION_UNRESOLVED,
))
_SIDE_ACTION = {"LEFT": TURN_LEFT_90, "RIGHT": TURN_RIGHT_90}


@dataclass
class BlastIterationBudget:
    """Bound Gemma calls and deterministic host actions independently."""

    planner_limit: int
    host_limit: int
    planner_count: int = 0
    host_count: int = 0

    @property
    def total_limit(self) -> int:
        return self.planner_limit + self.host_limit

    def reserve(self, *, planner_owned: bool) -> bool:
        name = "planner_count" if planner_owned else "host_count"
        limit = self.planner_limit if planner_owned else self.host_limit
        value = getattr(self, name)
        if value >= limit:
            return False
        setattr(self, name, value + 1)
        return True

    def remaining_host_actions(self) -> int:
        return self.host_limit - self.host_count


@dataclass(frozen=True)
class BlastAgenticRecovery:
    """Remember tried evidence tactics without granting motion authority."""

    max_replans: int = 3
    replans: int = 0
    attempted_sides: tuple[str, ...] = ()
    latest_reason: str | None = None
    latest_evidence: Mapping[str, object] | None = None

    def __post_init__(self) -> None:
        if (
            isinstance(self.max_replans, bool)
            or not isinstance(self.max_replans, int)
            or not 1 <= self.max_replans <= 8
            or isinstance(self.replans, bool)
            or not isinstance(self.replans, int)
            or not 0 <= self.replans <= self.max_replans
            or any(side not in ("LEFT", "RIGHT") for side in self.attempted_sides)
            or len(set(self.attempted_sides)) != len(self.attempted_sides)
            or self.latest_reason is not None
            and self.latest_reason not in RECOVERABLE_EVIDENCE_REASONS
        ):
            raise ValueError("BLAST recovery state is invalid")

    def record_side(self, side: str) -> "BlastAgenticRecovery":
        if side not in ("LEFT", "RIGHT"):
            raise ValueError("BLAST recovery side is invalid")
        if side in self.attempted_sides:
            return self
        return replace(
            self,
            attempted_sides=self.attempted_sides + (side,),
        )

    def recover(
        self,
        *,
        reason: str,
        evidence: Mapping[str, object] | None,
    ) -> "BlastAgenticRecovery" | None:
        if reason not in RECOVERABLE_EVIDENCE_REASONS:
            raise ValueError("BLAST recovery reason is invalid")
        if (
            self.replans >= self.max_replans
            or reason == ROUTE_BINDING_UNAVAILABLE
            and set(self.attempted_sides) == {"LEFT", "RIGHT"}
        ):
            return None
        return replace(
            self,
            replans=self.replans + 1,
            latest_reason=reason,
            latest_evidence=self._evidence_summary(evidence),
        )

    def reset_evidence_cycle(self) -> "BlastAgenticRecovery":
        """Keep the bounded replan count while opening a fresh side cycle."""

        return replace(
            self,
            attempted_sides=(),
            latest_reason=None,
            latest_evidence=None,
        )

    def planner_actions(
        self, available_actions: tuple[str, ...], *, scan_is_current: bool,
    ) -> tuple[str, ...]:
        """Remove only exhausted semantic tactics from host-safe actions."""

        actions = list(available_actions)
        if self.latest_reason is None:
            return tuple(actions)
        if scan_is_current:
            untried_turns = {
                _SIDE_ACTION[side]
                for side in ("LEFT", "RIGHT")
                if side not in self.attempted_sides
            }
            actions = [
                action for action in actions
                if action not in (TURN_LEFT_90, TURN_RIGHT_90)
                or action in untried_turns
            ]
        elif SCAN_FRONT_ARC in actions:
            # A new current scan lets Gemma reassess an alternate side. Motion
            # admission remains wholly owned by the existing safety gates.
            actions = [SCAN_FRONT_ARC]
        return tuple(actions)

    def context(self) -> Mapping[str, object] | None:
        if self.latest_reason is None:
            return None
        return {
            "schema": "blast-agentic-recovery/v1",
            "reason": self.latest_reason,
            "replan_attempt": self.replans,
            "replan_limit": self.max_replans,
            "attempted_sides": list(self.attempted_sides),
            "instruction": (
                "Previous evidence was insufficient, not proof that the goal "
                "is impossible. Choose a different available safe observation "
                "or untried side."
            ),
            "evidence": self.latest_evidence,
        }

    def enrich_planner_iteration(
        self, adapter, observation, available_actions, navigation_state,
        latest_scan_view,
    ):
        """Add bounded recovery facts and restrict the next safe tactic."""

        recovery_context = self.context()
        if recovery_context is None or not isinstance(
            navigation_state, PlannerNavigationState,
        ):
            return observation, available_actions
        observation["navigation_recovery"] = recovery_context
        scan_required = latest_scan_view is None
        if (
            scan_required
            and adapter._current_observation_allows_action(
                SCAN_FRONT_ARC, observation,
            )
        ):
            available_actions = (SCAN_FRONT_ARC,)
        return observation, self.planner_actions(
            available_actions,
            scan_is_current=not scan_required,
        )

    def after_side_rescan(
        self, *, outcome, continuation, final_scan,
    ) -> tuple["BlastAgenticRecovery", bool]:
        """Return new state and whether host rebase should begin."""

        if outcome is not None and outcome.terminal_reason in (
            RECOVERABLE_EVIDENCE_REASONS
        ):
            recovered = self.recover(
                reason=outcome.terminal_reason,
                evidence=final_scan,
            )
            if recovered is not None:
                return recovered, True
        if continuation is not None and not isinstance(continuation, Mapping):
            return self.reset_evidence_cycle(), False
        return self, False

    @staticmethod
    def _evidence_summary(
        evidence: Mapping[str, object] | None,
    ) -> Mapping[str, object] | None:
        if not isinstance(evidence, Mapping):
            return None
        multi = evidence.get("multi_view_observations")
        if not isinstance(multi, Mapping):
            return None
        summary = {
            key: multi.get(key)
            for key in (
                "selected_side",
                "viewpoint_separation_mm",
                "object_association_proven",
                "clearance_proven",
                "passage_proven",
                "route_eligible",
                "target_reacquisition",
            )
            if key in multi
        }
        return summary or None


def _prepare_recovery_rebase(
    adapter, navigation_state, side_view, remaining_slots,
):
    try:
        waypoint = recovery_rebase_waypoint(
            navigation_state.origin_scan_view,
            side_view,
            navigation_state.selected_side,
            navigation_state.host_actions,
        )
    except ValueError:
        return None, adapter._outcome(
            "recovery_rebase_unavailable", False,
            "BLAST cannot bind a verified return corridor",
        )
    required = (
        side_search_required_slots(waypoint)
        + maximum_side_search_required_slots()
    )
    if required > remaining_slots:
        return None, adapter._outcome(
            "recovery_rebase_budget_insufficient", False,
            "The episode cannot return and try the alternate side",
        )
    return waypoint, None


def _rebase_scan_verified(
    navigation_state, latest_scan_view, pose, observation, motion_executor,
    encoder_anchor_correlated, body_matched,
):
    return (
        encoder_anchor_correlated(observation, motion_executor)
        and body_matched(observation["sensors"])
        and recovery_rebase_completed(
            navigation_state.origin_scan_view,
            latest_scan_view,
            navigation_state.waypoint,
            pose,
            navigation_state.host_actions,
            navigation_state.recovery_action_start,
        )
    )


def resolve_side_rescan(
    adapter, *, navigation_state, recovery, latest_scan_view,
    selected_side, side_search_waypoint, local_detour_route, pose,
    result_observation, episode_start_heading, diagnostic_scan,
    mission, remaining_slots, action, map_trace, context,
    encoder_anchor_correlated, body_matched, motion_executor,
):
    """Resolve one side scan, including a bounded return before replanning."""

    if navigation_state.recovery_rebase:
        restored = adapter._with_navigation_reference(
            {"sensors": result_observation}, episode_start_heading,
        )
        if not _rebase_scan_verified(
            navigation_state, latest_scan_view, pose, restored,
            motion_executor, encoder_anchor_correlated, body_matched,
        ):
            return navigation_state, recovery, latest_scan_view, (
                local_detour_route
            ), adapter._outcome(
                "recovery_rebase_unverified", False,
                "BLAST did not re-establish its scan origin",
            )
        map_trace.clear_planned_leg(
            pose=pose, observation=result_observation,
        )
        context.publish({"active_route": None})
        return (
            PlannerNavigationState(), recovery, latest_scan_view,
            None, None,
        )

    final_scan, continuation, outcome = adapter._finish_side_rescan(
        origin_view=navigation_state.origin_scan_view,
        side_view=latest_scan_view,
        selected_side=selected_side,
        waypoint=side_search_waypoint,
        pose=pose,
        result_observation=result_observation,
        episode_start_heading=episode_start_heading,
        diagnostic_scan=diagnostic_scan,
        host_actions=navigation_state.host_actions,
        mission=mission,
        remaining_slots=remaining_slots,
        motion_executor=motion_executor,
    )
    if isinstance(continuation, Mapping):
        side_search_waypoint = continuation
        navigation_state = navigation_state.continue_to_waypoint(
            continuation,
        )
        map_trace.record_action(
            action, pose, result_observation, selected_side,
            side_search_waypoint, None, pose_observed=False,
        )
    elif continuation is not None:
        local_detour_route = continuation
        navigation_state = navigation_state.bind_local_detour(
            local_detour_route,
        )
        map_trace.record_action(
            action, pose, result_observation, selected_side,
            side_search_waypoint, None, route=local_detour_route,
            pose_observed=False,
        )
    if final_scan is not None:
        context.publish({"scan": final_scan})
    recovery, replan = recovery.after_side_rescan(
        outcome=outcome,
        continuation=continuation,
        final_scan=final_scan,
    )
    if not replan:
        return (
            navigation_state, recovery, latest_scan_view,
            local_detour_route, outcome,
        )
    rebase, blocked = _prepare_recovery_rebase(
        adapter, navigation_state, latest_scan_view, remaining_slots,
    )
    if blocked is not None:
        return (
            navigation_state, recovery, latest_scan_view,
            local_detour_route, blocked,
        )
    navigation_state = navigation_state.begin_recovery_rebase(rebase)
    map_trace.clear_route(
        pose=pose, observation=result_observation,
    )
    map_trace.record_action(
        action, pose, result_observation, selected_side,
        rebase, None, pose_observed=False,
    )
    context.publish({"active_route": None})
    return navigation_state, recovery, None, None, None


__all__ = (
    "BlastAgenticRecovery",
    "BlastIterationBudget",
    "RECOVERABLE_EVIDENCE_REASONS",
    "ROUTE_BINDING_UNAVAILABLE",
    "TARGET_REACQUISITION_COLLECTED",
    "TARGET_REACQUISITION_UNRESOLVED",
    "resolve_side_rescan",
)
