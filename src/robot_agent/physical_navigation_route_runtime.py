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
    SYNC_ADVANCED,
    SYNC_COMPLETED,
    SYNC_INACTIVE,
    SYNC_REBUILT,
    SYNC_UNCHANGED,
    LocalDetourGuidance,
    derive_local_detour_guidance,
    synchronize_local_detour_route,
)
from .local_detour_route import (
    ROUTE_ACTIVE,
    ROUTE_COMPLETE,
    ROUTE_INVALID,
    LocalDetourRoute,
)
from .physical_navigation_contract import MOTION_ACTIONS
from .physical_navigation_experience import ROUTE_EXECUTOR_ACTION_SOURCE
from .physical_navigation_mission import DirectionalMission


HANDOFF_ROUTE_COMPLETE = "ROUTE_COMPLETE"
HANDOFF_ROUTE_INVALID = "ROUTE_INVALID"
HANDOFF_ROUTE_MISSING = "ROUTE_MISSING"
HANDOFF_ROUTE_REPLAN_REQUIRED = "ROUTE_REPLAN_REQUIRED"
HANDOFF_NO_UNIQUE_FEASIBLE_MOTION = "NO_UNIQUE_FEASIBLE_MOTION"
HANDOFF_EXECUTION_VETOED = "EXECUTION_VETOED"
HANDOFF_MOTION_NOT_COMPLETED = "MOTION_NOT_COMPLETED"
HANDOFF_EPISODE_DEADLINE_ELAPSED = "EPISODE_DEADLINE_ELAPSED"
HANDOFF_REASONS = frozenset((
    HANDOFF_ROUTE_COMPLETE,
    HANDOFF_ROUTE_INVALID,
    HANDOFF_ROUTE_MISSING,
    HANDOFF_ROUTE_REPLAN_REQUIRED,
    HANDOFF_NO_UNIQUE_FEASIBLE_MOTION,
    HANDOFF_EXECUTION_VETOED,
    HANDOFF_MOTION_NOT_COMPLETED,
    HANDOFF_EPISODE_DEADLINE_ELAPSED,
))


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


def _with_handoff(
    last_tool_result,
    *,
    reason: str,
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
    if "reason" not in value:
        value["reason"] = reason
    value["target_hypothesis_id"] = (
        None if route is None else route.target_hypothesis_id
    )
    handoff = {
        "status": "planner_handoff",
        "reason": reason,
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
    handoff_reason: str

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
            or self.handoff_reason not in HANDOFF_REASONS
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
        handoff_reason,
        detail=None,
    ) -> PhysicalNavigationRouteRuntimeResult:
        updated_tool_result = _with_handoff(
            last_tool_result,
            reason=handoff_reason,
            route=route,
            action_count=len(actions),
            detail=detail,
        )
        self._emit(
            "local_detour_route_handoff",
            handoff_reason=handoff_reason,
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
            handoff_reason=handoff_reason,
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
        route.  Synchronization may expose a rebuilt route after fresh map
        evidence, but this method returns it to the planner without executing
        it.  It never invents or changes the authorized detour side.
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
                    handoff_reason=HANDOFF_EPISODE_DEADLINE_ELAPSED,
                )
            if current_route is None:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=None,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_ROUTE_MISSING,
                )
            if current_route.status == ROUTE_COMPLETE:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_ROUTE_COMPLETE,
                )
            if current_route.status != ROUTE_ACTIVE:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_ROUTE_INVALID,
                )
            if getattr(self.memory, "localization_valid", None) is not True:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_ROUTE_REPLAN_REQUIRED,
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
                    handoff_reason=HANDOFF_EPISODE_DEADLINE_ELAPSED,
                )
            refreshed = self._refresh_authorized_local_detour_route(
                route=current_route,
                active_maneuver=active_maneuver,
                mission=mission,
                navigation=navigation,
                action_specs=action_specs,
            )
            current_route = refreshed.route
            if refreshed.sync_event == SYNC_COMPLETED:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_ROUTE_COMPLETE,
                )
            if refreshed.sync_event == SYNC_REBUILT:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_ROUTE_REPLAN_REQUIRED,
                    detail={
                        "sync_event": refreshed.sync_event,
                        "reason": refreshed.sync_reason,
                    },
                )
            if refreshed.sync_event not in (SYNC_UNCHANGED, SYNC_ADVANCED):
                reason = (
                    HANDOFF_ROUTE_MISSING
                    if current_route is None
                    else HANDOFF_ROUTE_INVALID
                    if current_route.status == ROUTE_INVALID
                    else HANDOFF_ROUTE_REPLAN_REQUIRED
                )
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=reason,
                    detail={
                        "sync_event": refreshed.sync_event,
                        "reason": refreshed.sync_reason,
                    },
                )
            guidance = refreshed.guidance
            if current_route is not None and current_route.status == ROUTE_COMPLETE:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_ROUTE_COMPLETE,
                )
            if current_route is None:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=None,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_ROUTE_MISSING,
                )
            if current_route.status == ROUTE_INVALID:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_ROUTE_INVALID,
                )

            allowed = guidance.allowed_motion_actions
            if allowed is None or len(allowed) != 1:
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_NO_UNIQUE_FEASIBLE_MOTION,
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
                    handoff_reason=HANDOFF_EXECUTION_VETOED,
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
                    handoff_reason=HANDOFF_EPISODE_DEADLINE_ELAPSED,
                )

            self._emit(
                "local_detour_route_action_started",
                action=action,
                route=_route_summary(current_route),
                guidance=_guidance_summary(guidance),
                host_selected_route_or_side=False,
            )
            next_observation, current_tool_result = self._execute_motion(
                action,
                action_specs=action_specs,
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
                return self._route_runtime_result(
                    observation=current_observation,
                    route=current_route,
                    last_tool_result=current_tool_result,
                    actions=executed_actions,
                    handoff_reason=HANDOFF_MOTION_NOT_COMPLETED,
                    detail={"action": action},
                )


__all__ = (
    "HANDOFF_EPISODE_DEADLINE_ELAPSED",
    "HANDOFF_EXECUTION_VETOED",
    "HANDOFF_MOTION_NOT_COMPLETED",
    "HANDOFF_NO_UNIQUE_FEASIBLE_MOTION",
    "HANDOFF_REASONS",
    "HANDOFF_ROUTE_COMPLETE",
    "HANDOFF_ROUTE_INVALID",
    "HANDOFF_ROUTE_MISSING",
    "HANDOFF_ROUTE_REPLAN_REQUIRED",
    "PhysicalNavigationRouteRuntimeError",
    "PhysicalNavigationRouteRuntimeMixin",
    "PhysicalNavigationRouteRefresh",
    "PhysicalNavigationRouteRuntimeResult",
)
