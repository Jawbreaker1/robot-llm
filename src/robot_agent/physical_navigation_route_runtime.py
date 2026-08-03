"""Deterministic execution of one model-authorized local detour route.

The mixin deliberately owns no planner and never chooses an obstacle, target,
or detour side.  It may continue an already authorized route only while the
fresh physical state yields exactly one route-progressing motion.  Every
physical pulse is still revalidated by the host runtime and the EV3 worker.
"""

from copy import deepcopy
from dataclasses import dataclass
import math
from typing import Mapping, MutableMapping, Optional, Tuple

from .local_detour_controller import (
    SYNC_INACTIVE,
    SYNC_REBUILT,
    SYNC_UNCHANGED,
    LocalDetourGuidance,
    derive_local_detour_guidance,
    synchronize_local_detour_route,
)
from .local_detour_route import (
    PASS_BEYOND_TARGET,
    ROUTE_ACTIVE,
    ROUTE_COMPLETE,
    ROUTE_INVALID,
    LocalDetourRoute,
)
from .physical_navigation_contract import ADVANCE, MOTION_ACTIONS
from .physical_navigation_experience import ROUTE_EXECUTOR_ACTION_SOURCE
from .physical_navigation_mission import DirectionalMission


EXECUTION_CONTINUE = "CONTINUE"
EXECUTION_REPLAN = "REPLAN"
EXECUTION_COMPLETE = "COMPLETE"
EXECUTION_FAILED = "FAILED"
EXECUTION_OUTCOMES = frozenset((
    EXECUTION_CONTINUE,
    EXECUTION_REPLAN,
    EXECUTION_COMPLETE,
    EXECUTION_FAILED,
))

ROUTE_EXECUTION_REASON_COMPLETE = "ROUTE_COMPLETE"
ROUTE_EXECUTION_REASON_INVALID = "ROUTE_INVALID"
ROUTE_EXECUTION_REASON_MISSING = "ROUTE_MISSING"
ROUTE_EXECUTION_REASON_REPLAN_REQUIRED = "ROUTE_REPLAN_REQUIRED"
ROUTE_EXECUTION_REASON_NO_UNIQUE_MOTION = "NO_UNIQUE_FEASIBLE_MOTION"
ROUTE_EXECUTION_REASON_VETOED = "EXECUTION_VETOED"
ROUTE_EXECUTION_REASON_MOTION_INCOMPLETE = "MOTION_NOT_COMPLETED"
ROUTE_EXECUTION_REASON_DEADLINE = "EPISODE_DEADLINE_ELAPSED"
ROUTE_EXECUTION_REPLAN_REASONS = frozenset((
    ROUTE_EXECUTION_REASON_INVALID,
    ROUTE_EXECUTION_REASON_MISSING,
    ROUTE_EXECUTION_REASON_REPLAN_REQUIRED,
    ROUTE_EXECUTION_REASON_NO_UNIQUE_MOTION,
    ROUTE_EXECUTION_REASON_VETOED,
    ROUTE_EXECUTION_REASON_MOTION_INCOMPLETE,
))
ROUTE_EXECUTION_REASONS = frozenset((
    ROUTE_EXECUTION_REASON_COMPLETE,
    ROUTE_EXECUTION_REASON_DEADLINE,
)).union(ROUTE_EXECUTION_REPLAN_REASONS)
PASS_HEADING_TRIM_MDEG = 15_000
PASS_HEADING_TRIM_DELTAS = {
    "LEFT_OF_GOAL": PASS_HEADING_TRIM_MDEG,
    "RIGHT_OF_GOAL": -PASS_HEADING_TRIM_MDEG,
}


def _valid_execution_outcome_reason(outcome, reason_code) -> bool:
    if outcome == EXECUTION_CONTINUE:
        return reason_code is None
    if outcome == EXECUTION_COMPLETE:
        return reason_code == ROUTE_EXECUTION_REASON_COMPLETE
    if outcome == EXECUTION_FAILED:
        return reason_code == ROUTE_EXECUTION_REASON_DEADLINE
    if outcome == EXECUTION_REPLAN:
        return reason_code in ROUTE_EXECUTION_REPLAN_REASONS
    return False


class PhysicalNavigationRouteRuntimeError(ValueError):
    pass


def _route_summary(route: Optional[LocalDetourRoute]):
    if route is None:
        return None
    waypoint = route.active_waypoint
    return {
        "route_id": route.route_id,
        "version": route.version,
        "status": route.status,
        "target_hypothesis_id": route.target_hypothesis_id,
        "detour_side": route.detour_side,
        "active_waypoint_index": (
            None if waypoint is None else route.active_index
        ),
        "active_waypoint_kind": (
            None if waypoint is None else waypoint.kind
        ),
    }


def _guidance_summary(guidance):
    return {
        "reason": guidance.reason,
        "allowed_motion_actions": sorted(
            guidance.allowed_motion_actions or ()
        ),
        "active_waypoint_index": guidance.active_waypoint_index,
        "active_waypoint_kind": guidance.active_waypoint_kind,
        "distance_to_waypoint_mm": guidance.distance_to_waypoint_mm,
        "heading_error_mdeg": guidance.heading_error_mdeg,
    }


def _monotonic_target_growth(
    previous: LocalDetourRoute,
    refreshed: "PhysicalNavigationRouteRefresh",
) -> bool:
    """Allow only conservative growth of the already authorized obstacle."""

    current = refreshed.route
    if (
        current is None
        or refreshed.sync_event != SYNC_REBUILT
        or refreshed.sync_reason != "TARGET_GEOMETRY_MISMATCH"
        or current.target_hypothesis_id
        != previous.target_hypothesis_id
        or current.detour_side != previous.detour_side
        or current.frame_id != previous.frame_id
        or current.map_generation_id != previous.map_generation_id
        or current.goal_origin_x_mm != previous.goal_origin_x_mm
        or current.goal_origin_y_mm != previous.goal_origin_y_mm
        or current.goal_heading_mdeg != previous.goal_heading_mdeg
        or current.target_centroid_x_mm
        != previous.target_centroid_x_mm
        or current.target_centroid_y_mm
        != previous.target_centroid_y_mm
        or current.based_on_map_version < previous.based_on_map_version
        or current.target_radius_mm < previous.target_radius_mm
        or not set(previous.target_support_points).issubset(
            current.target_support_points
        )
    ):
        return False
    geometry_grew = (
        set(previous.target_support_points)
        != set(current.target_support_points)
        or current.target_radius_mm > previous.target_radius_mm
    )
    side_sign = 1 if current.detour_side == "LEFT_OF_GOAL" else -1
    return (
        geometry_grew
        and current.inflated_lateral_clearance_mm
        >= previous.inflated_lateral_clearance_mm
        and current.inflated_pass_clearance_mm
        >= previous.inflated_pass_clearance_mm
        and current.pass_longitudinal_offset_mm
        >= previous.pass_longitudinal_offset_mm
        and side_sign
        * (
            current.route_lateral_offset_mm
            - previous.route_lateral_offset_mm
        )
        >= 0
    )


def _with_handoff(
    last_tool_result,
    *,
    outcome: str,
    reason_code: Optional[str],
    route: Optional[LocalDetourRoute],
    action_count: int,
    detail=None,
):
    value = (
        deepcopy(dict(last_tool_result))
        if isinstance(last_tool_result, Mapping)
        else {}
    )
    if "operation" not in value:
        value["operation"] = "local_detour_route"
    if "status" not in value:
        value["status"] = "planner_handoff"
    if "reason" not in value and reason_code is not None:
        value["reason"] = reason_code
    value["target_hypothesis_id"] = (
        None if route is None else route.target_hypothesis_id
    )
    handoff = {
        "status": "planner_handoff",
        "outcome": outcome,
        "reason_code": reason_code,
        "executed_action_count": action_count,
        "route": _route_summary(route),
        "host_selected_route_or_side": False,
    }
    if detail is not None:
        handoff["detail"] = deepcopy(detail)
    value["route_execution"] = handoff
    return value


@dataclass(frozen=True)
class PhysicalNavigationRouteRuntimeResult:
    """Compact handoff from deterministic route following to the planner."""

    observation: Mapping[str, object]
    route: Optional[LocalDetourRoute]
    last_tool_result: Mapping[str, object]
    actions: Tuple[str, ...]
    outcome: str
    reason_code: Optional[str]

    def __post_init__(self) -> None:
        if (
            not isinstance(self.observation, Mapping)
            or (
                self.route is not None
                and not isinstance(self.route, LocalDetourRoute)
            )
            or not isinstance(self.last_tool_result, Mapping)
            or not isinstance(self.actions, tuple)
            or any(action not in MOTION_ACTIONS for action in self.actions)
            or self.outcome not in EXECUTION_OUTCOMES
            or not _valid_execution_outcome_reason(
                self.outcome,
                self.reason_code,
            )
        ):
            raise PhysicalNavigationRouteRuntimeError(
                "route runtime result is invalid"
            )


@dataclass(frozen=True)
class PhysicalNavigationRouteRefresh:
    """One route synchronization plus its fresh deterministic guidance."""

    route: Optional[LocalDetourRoute]
    guidance: LocalDetourGuidance
    sync_event: str
    sync_reason: Optional[str]


class PhysicalNavigationRouteRuntimeMixin:
    """Follow one active local route without consulting or replacing the LLM."""

    def _post_route_planner_snapshot(
        self,
        *,
        route,
        active_maneuver,
        mission,
        observation,
        action_specs,
    ):
        """Rebuild planner state after route execution returns control."""

        mission_value, navigation = self._goal_state(
            mission,
            observation,
            action_specs,
        )
        refreshed = self._refresh_authorized_local_detour_route(
            route=route,
            active_maneuver=active_maneuver,
            mission=mission,
            navigation=navigation,
            action_specs=action_specs,
        )
        return mission_value, navigation, refreshed

    def _emit_local_detour_route_update(self, synchronized) -> None:
        if synchronized.event in (SYNC_INACTIVE, SYNC_UNCHANGED):
            return
        self._emit(
            "local_detour_route_updated",
            sync_event=synchronized.event,
            sync_reason=synchronized.reason,
            previous_route_id=synchronized.previous_route_id,
            route=(
                None
                if synchronized.route is None
                else synchronized.route.to_dict()
            ),
            host_selected_route_or_side=False,
        )

    def _route_runtime_result(
        self,
        *,
        observation,
        route,
        last_tool_result,
        actions,
        outcome,
        reason_code,
        detail=None,
    ) -> PhysicalNavigationRouteRuntimeResult:
        updated_tool_result = _with_handoff(
            last_tool_result,
            outcome=outcome,
            reason_code=reason_code,
            route=route,
            action_count=len(actions),
            detail=detail,
        )
        self._emit(
            "local_detour_route_handoff",
            outcome=outcome,
            reason_code=reason_code,
            route=_route_summary(route),
            executed_actions=list(actions),
            detail=deepcopy(detail),
            host_selected_route_or_side=False,
        )
        return PhysicalNavigationRouteRuntimeResult(
            observation=observation,
            route=route,
            last_tool_result=updated_tool_result,
            actions=tuple(actions),
            outcome=outcome,
            reason_code=reason_code,
        )

    def _route_deadline_elapsed(self, deadline: float, stage: str) -> bool:
        self._raise_if_cancelled(stage)
        return self.monotonic() >= deadline

    def _refresh_authorized_local_detour_route(
        self,
        *,
        route: Optional[LocalDetourRoute],
        active_maneuver,
        mission: DirectionalMission,
        navigation: MutableMapping[str, object],
        action_specs: Mapping[str, Mapping[str, object]],
    ) -> PhysicalNavigationRouteRefresh:
        """Synchronize, derive guidance, and publish one planner projection."""

        if (
            not isinstance(navigation, MutableMapping)
            or not isinstance(action_specs, Mapping)
            or not isinstance(mission, DirectionalMission)
            or (
                route is not None
                and not isinstance(route, LocalDetourRoute)
            )
        ):
            raise PhysicalNavigationRouteRuntimeError(
                "route refresh arguments are invalid"
            )
        feasibility = navigation.get("action_feasibility")
        motion_feasibility = (
            feasibility.get("motion_actions")
            if isinstance(feasibility, Mapping)
            else None
        )
        if not isinstance(motion_feasibility, Mapping):
            raise PhysicalNavigationRouteRuntimeError(
                "fresh route motion feasibility is invalid"
            )
        synchronized = synchronize_local_detour_route(
            route,
            active_maneuver=active_maneuver,
            current_pose=self.memory.pose,
            mission=mission,
            hazard_map=self.memory.hazard_map,
        )
        self._emit_local_detour_route_update(synchronized)
        guidance = derive_local_detour_guidance(
            synchronized.route,
            current_pose=self.memory.pose,
            motion_feasibility=motion_feasibility,
            action_specs=action_specs,
            odometry_calibration=self.memory.odometry_calibration,
        )
        route_value = guidance.route
        navigation["local_detour_route"] = (
            None if route_value is None else route_value.to_dict()
        )
        navigation["local_detour_guidance"] = _guidance_summary(guidance)
        return PhysicalNavigationRouteRefresh(
            route=route_value,
            guidance=guidance,
            sync_event=synchronized.event,
            sync_reason=synchronized.reason,
        )

    def _execute_authorized_local_detour_route(
        self,
        *,
        turn: int,
        deadline: float,
        observation: Mapping[str, object],
        action_specs: Mapping[str, Mapping[str, object]],
        mission: DirectionalMission,
        route: Optional[LocalDetourRoute],
        active_maneuver,
        last_tool_result=None,
    ) -> PhysicalNavigationRouteRuntimeResult:
        """Execute unique route-progressing pulses until the planner is needed.

        ``active_maneuver`` is the model-owned authorization for the supplied
        route.  Synchronization may continue a conservative route extension
        when fresh evidence only grows the same obstacle.  Any other rebuild
        returns to the planner.  This method never invents or changes the
        authorized detour side.
        """

        if (
            isinstance(turn, bool)
            or not isinstance(turn, int)
            or turn <= 0
            or isinstance(deadline, bool)
            or not isinstance(deadline, (int, float))
            or not math.isfinite(deadline)
            or not isinstance(observation, Mapping)
            or not isinstance(action_specs, Mapping)
            or not isinstance(mission, DirectionalMission)
            or (
                route is not None
                and not isinstance(route, LocalDetourRoute)
            )
            or (
                last_tool_result is not None
                and not isinstance(last_tool_result, Mapping)
            )
        ):
            raise PhysicalNavigationRouteRuntimeError(
                "route runtime arguments are invalid"
            )

        current_observation = observation
        current_route = route
        current_tool_result = last_tool_result
        executed_actions = []
        self._emit(
            "local_detour_route_started",
            route=_route_summary(current_route),
            host_selected_route_or_side=False,
        )

        while True:
            if self._route_deadline_elapsed(
                deadline,
                "before_local_detour_route_refresh",
            ):
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_FAILED,
                    reason_code=ROUTE_EXECUTION_REASON_DEADLINE,
                )
            if current_route is None:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=None,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_REPLAN,
                    reason_code=ROUTE_EXECUTION_REASON_MISSING,
                )
            if current_route.status == ROUTE_COMPLETE:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_COMPLETE,
                    reason_code=ROUTE_EXECUTION_REASON_COMPLETE,
                )
            if current_route.status != ROUTE_ACTIVE:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_REPLAN,
                    reason_code=ROUTE_EXECUTION_REASON_INVALID,
                )
            if getattr(self.memory, "localization_valid", None) is not True:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_REPLAN,
                    reason_code=ROUTE_EXECUTION_REASON_REPLAN_REQUIRED,
                    detail={"reason": "LOCALIZATION_INVALID"},
                )

            _mission_value, navigation = self._goal_state(
                mission,
                current_observation,
                action_specs,
            )
            if self._route_deadline_elapsed(
                deadline,
                "after_local_detour_route_refresh",
            ):
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_FAILED,
                    reason_code=ROUTE_EXECUTION_REASON_DEADLINE,
                )
            refreshed = self._refresh_authorized_local_detour_route(
                route=current_route,
                active_maneuver=active_maneuver,
                mission=mission,
                navigation=navigation,
                action_specs=action_specs,
            )
            previous_route = current_route
            previous_route_id = previous_route.route_id
            current_route = refreshed.route
            transition_detail = {
                "sync_event": refreshed.sync_event,
                "reason": refreshed.sync_reason,
            }
            if current_route is None:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=None,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_REPLAN,
                    reason_code=ROUTE_EXECUTION_REASON_MISSING,
                    detail=transition_detail,
                )
            if current_route.status == ROUTE_COMPLETE:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_COMPLETE,
                    reason_code=ROUTE_EXECUTION_REASON_COMPLETE,
                )
            if current_route.status == ROUTE_INVALID:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_REPLAN,
                    reason_code=ROUTE_EXECUTION_REASON_INVALID,
                    detail=transition_detail,
                )
            if (
                current_route.route_id != previous_route_id
                and not _monotonic_target_growth(
                    previous_route,
                    refreshed,
                )
            ):
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_REPLAN,
                    reason_code=ROUTE_EXECUTION_REASON_REPLAN_REQUIRED,
                    detail=transition_detail,
                )

            guidance = refreshed.guidance
            waypoint_kind = guidance.active_waypoint_kind
            allowed = guidance.allowed_motion_actions
            if allowed is None or len(allowed) != 1:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_REPLAN,
                    reason_code=ROUTE_EXECUTION_REASON_NO_UNIQUE_MOTION,
                    detail={
                        "guidance": _guidance_summary(guidance),
                        "allowed_motion_actions": sorted(allowed or ()),
                    },
                )
            action = next(iter(allowed))
            basis_before = self._experience_basis(current_observation)
            veto = self._execution_veto(
                action=action,
                observation=current_observation,
                action_specs=action_specs,
                deadline=deadline,
            )
            if veto is not None:
                current_tool_result = {
                    "operation": "local_detour_route",
                    "status": "route_action_vetoed",
                    "reason": veto.get("code", "route_action_vetoed"),
                    "validation": deepcopy(veto),
                    "target_hypothesis_id": (
                        current_route.target_hypothesis_id
                    ),
                    "host_selected_route_or_side": False,
                }
                self._record_experience(
                    turn=turn,
                    action=action,
                    source=ROUTE_EXECUTOR_ACTION_SOURCE,
                    result=current_tool_result,
                    basis_before=basis_before,
                    observation_after=current_observation,
                )
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_REPLAN,
                    reason_code=ROUTE_EXECUTION_REASON_VETOED,
                    detail={"action": action, "veto": veto},
                )
            if self._route_deadline_elapsed(
                deadline,
                "immediately_before_local_detour_route_motion",
            ):
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_FAILED,
                    reason_code=ROUTE_EXECUTION_REASON_DEADLINE,
                )

            self._emit(
                "local_detour_route_action_started",
                action=action,
                route=_route_summary(current_route),
                guidance=_guidance_summary(guidance),
                host_selected_route_or_side=False,
            )
            pass_heading_trim_eligible = (
                action == ADVANCE
                and waypoint_kind == PASS_BEYOND_TARGET
                and current_route.route_id
                not in self._pass_heading_trim_attempted_route_ids
            )
            if pass_heading_trim_eligible:
                self._pass_heading_trim_attempted_route_ids.add(
                    current_route.route_id
                )
            next_observation, current_tool_result = self._execute_motion(
                action,
                action_specs=action_specs,
                defer_infrared_hazard=pass_heading_trim_eligible,
            )
            executed_actions.append(action)
            current_observation = next_observation
            self._record_experience(
                turn=turn,
                action=action,
                source=ROUTE_EXECUTOR_ACTION_SOURCE,
                result=current_tool_result,
                basis_before=basis_before,
                observation_after=current_observation,
            )
            if current_tool_result.get("status") != "completed":
                if (
                    pass_heading_trim_eligible
                    and current_tool_result.get("reason")
                    == "infrared_blocked"
                ):
                    if self._route_deadline_elapsed(
                        deadline,
                        "before_pass_heading_trim",
                    ):
                        return self._route_runtime_result(
                            observation=current_observation,
                            route=current_route,
                            last_tool_result=current_tool_result,
                            actions=executed_actions,
                            outcome=EXECUTION_FAILED,
                            reason_code=ROUTE_EXECUTION_REASON_DEADLINE,
                            detail={"action": action},
                        )
                    interrupted_motion = deepcopy(current_tool_result)
                    relative_delta_mdeg = PASS_HEADING_TRIM_DELTAS[
                        current_route.detour_side
                    ]
                    (
                        current_observation,
                        heading_trim,
                    ) = self._execute_pass_heading_trim(
                        observation=current_observation,
                        relative_delta_mdeg=relative_delta_mdeg,
                    )
                    if (
                        not isinstance(heading_trim, Mapping)
                        or heading_trim.get("status")
                        not in ("completed", "blocked", "failed")
                        or type(heading_trim.get("opening_clear")) is not bool
                    ):
                        raise PhysicalNavigationRouteRuntimeError(
                            "pass heading trim result is invalid"
                        )
                    current_tool_result = deepcopy(dict(heading_trim))
                    current_tool_result.setdefault(
                        "operation",
                        "pass_heading_trim",
                    )
                    current_tool_result["trigger_motion"] = (
                        interrupted_motion
                    )
                    current_tool_result["target_hypothesis_id"] = (
                        current_route.target_hypothesis_id
                    )
                    current_tool_result["host_selected_route_or_side"] = (
                        False
                    )
                    self._emit(
                        "local_detour_pass_heading_trimmed",
                        route=_route_summary(current_route),
                        relative_delta_mdeg=relative_delta_mdeg,
                        result=deepcopy(current_tool_result),
                        host_selected_route_or_side=False,
                    )
                    if (
                        current_tool_result["status"] == "completed"
                        and current_tool_result["opening_clear"] is True
                    ):
                        continue
                    return self._route_runtime_result(
                        observation=current_observation,
                        route=current_route,
                        last_tool_result=current_tool_result,
                        actions=executed_actions,
                        outcome=EXECUTION_REPLAN,
                        reason_code=(
                            ROUTE_EXECUTION_REASON_MOTION_INCOMPLETE
                        ),
                        detail={
                            "action": action,
                            "pass_heading_trim": deepcopy(
                                dict(heading_trim)
                            ),
                        },
                    )
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    outcome=EXECUTION_REPLAN,
                    reason_code=ROUTE_EXECUTION_REASON_MOTION_INCOMPLETE,
                    detail={"action": action},
                )


__all__ = (
    "EXECUTION_COMPLETE",
    "EXECUTION_CONTINUE",
    "EXECUTION_FAILED",
    "EXECUTION_OUTCOMES",
    "EXECUTION_REPLAN",
    "PhysicalNavigationRouteRuntimeError",
    "PhysicalNavigationRouteRuntimeMixin",
    "PhysicalNavigationRouteRefresh",
    "PhysicalNavigationRouteRuntimeResult",
    "PASS_HEADING_TRIM_MDEG",
    "ROUTE_EXECUTION_REASON_COMPLETE",
    "ROUTE_EXECUTION_REASON_DEADLINE",
    "ROUTE_EXECUTION_REASON_INVALID",
    "ROUTE_EXECUTION_REASON_MISSING",
    "ROUTE_EXECUTION_REASON_MOTION_INCOMPLETE",
    "ROUTE_EXECUTION_REASON_NO_UNIQUE_MOTION",
    "ROUTE_EXECUTION_REASON_REPLAN_REQUIRED",
    "ROUTE_EXECUTION_REASON_VETOED",
    "ROUTE_EXECUTION_REASONS",
)
