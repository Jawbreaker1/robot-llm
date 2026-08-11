"""Explicit ownership state for one BLAST navigation episode."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Mapping, Union

from .local_detour_route import LocalDetourRoute


@dataclass(frozen=True)
class PlannerNavigationState:
    """The language-model planner owns the next semantic action."""

    def begin_side_search(
        self,
        *,
        selected_side: str,
        waypoint: Mapping,
        origin_scan_view: Mapping,
    ) -> "SideSearchNavigationState":
        return SideSearchNavigationState(
            selected_side=selected_side,
            waypoint=waypoint,
            origin_scan_view=origin_scan_view,
        )


@dataclass(frozen=True)
class SideSearchNavigationState:
    """The host owns bounded motion to one verified side viewpoint."""

    selected_side: str
    waypoint: Mapping
    origin_scan_view: Mapping
    reorientation_attempted: bool = False
    previous_outbound_distance_mm: int | None = None
    host_actions: tuple[str, ...] = ()

    def mark_reorientation_attempted(self) -> "SideSearchNavigationState":
        return replace(self, reorientation_attempted=True)

    def record_host_action(
        self,
        action: str,
        *,
        outbound_distance_mm: int | None = None,
    ) -> "SideSearchNavigationState":
        distance = (
            self.previous_outbound_distance_mm
            if outbound_distance_mm is None
            else outbound_distance_mm
        )
        return replace(
            self,
            previous_outbound_distance_mm=distance,
            host_actions=self.host_actions + (action,),
        )

    def continue_to_waypoint(
        self, waypoint: Mapping,
    ) -> "SideSearchNavigationState":
        return replace(
            self,
            waypoint=waypoint,
            reorientation_attempted=False,
            previous_outbound_distance_mm=None,
        )

    def bind_local_detour(
        self, route: LocalDetourRoute,
    ) -> "LocalDetourNavigationState":
        return LocalDetourNavigationState(
            selected_side=self.selected_side,
            waypoint=self.waypoint,
            route=route,
        )


@dataclass(frozen=True)
class LocalDetourNavigationState:
    """The host owns one bound local-detour route."""

    selected_side: str
    waypoint: Mapping
    route: LocalDetourRoute
    pass_scan_complete: bool = False

    def with_route(
        self, route: LocalDetourRoute,
    ) -> "LocalDetourNavigationState":
        return replace(self, route=route)

    def mark_pass_scan_complete(self) -> "LocalDetourNavigationState":
        return replace(self, pass_scan_complete=True)


BlastNavigationState = Union[
    PlannerNavigationState,
    SideSearchNavigationState,
    LocalDetourNavigationState,
]


__all__ = (
    "BlastNavigationState",
    "LocalDetourNavigationState",
    "PlannerNavigationState",
    "SideSearchNavigationState",
)
