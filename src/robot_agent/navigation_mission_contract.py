"""Strict contracts and value objects for bounded navigation missions.

This module owns the untrusted JSON boundary and the immutable values shared
with mission execution.  It deliberately contains no mission runner, plant,
supervisor, transport, or motor-control logic.
"""

from dataclasses import dataclass
import json
from typing import Mapping, Tuple, TYPE_CHECKING

from .navigation_contract import (
    NavigationContractError,
    WaypointGoal,
    identifier,
    integer,
)
from .navigation_state import NavigationSnapshot

if TYPE_CHECKING:
    from .navigation_episode import NavigationResult


MISSION_PLAN_SCHEMA = "robot-navigation-mission-plan/v1"
MAX_MISSION_PLAN_BYTES = 32 * 1024
MAX_MISSION_LEGS = 8

MISSION_COMPLETED = "MISSION_COMPLETED"
MISSION_ABORTED = "MISSION_ABORTED"
MISSION_SAFETY_STOP = "MISSION_SAFETY_STOP"
MISSION_BUDGET_EXHAUSTED = "MISSION_BUDGET_EXHAUSTED"
MISSION_PLAN_REJECTED = "MISSION_PLAN_REJECTED"
MISSION_PLAN_STALE = "MISSION_PLAN_STALE"
MISSION_LEG_FAILED = "MISSION_LEG_FAILED"


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
class MissionLeg:
    """One semantic waypoint in an ordered mission plan."""

    leg_id: str
    target_x_mm: int
    target_y_mm: int
    tolerance_mm: int = 35

    def __post_init__(self) -> None:
        identifier("leg_id", self.leg_id, 96)
        integer("target_x_mm", self.target_x_mm, -1_000_000, 1_000_000)
        integer("target_y_mm", self.target_y_mm, -1_000_000, 1_000_000)
        integer("tolerance_mm", self.tolerance_mm, 1, 10_000)

    def to_dict(self) -> Mapping[str, object]:
        return {
            "leg_id": self.leg_id,
            "target_x_mm": self.target_x_mm,
            "target_y_mm": self.target_y_mm,
            "tolerance_mm": self.tolerance_mm,
        }


@dataclass(frozen=True)
class MissionPlan:
    """An immutable plan bound to one stopped simulator snapshot."""

    plan_id: str
    robot_id: str
    controller_instance_id: str
    based_on_state_version: int
    based_on_world_model_version: int
    plan_revision: int
    legs: Tuple[MissionLeg, ...]

    def __post_init__(self) -> None:
        identifier("plan_id", self.plan_id)
        identifier("robot_id", self.robot_id)
        identifier("controller_instance_id", self.controller_instance_id)
        integer(
            "based_on_state_version",
            self.based_on_state_version,
            1,
            2**63 - 1,
        )
        integer(
            "based_on_world_model_version",
            self.based_on_world_model_version,
            1,
            2**63 - 1,
        )
        integer("plan_revision", self.plan_revision, 1, 2**63 - 1)
        if (
            not isinstance(self.legs, tuple)
            or not 1 <= len(self.legs) <= MAX_MISSION_LEGS
            or any(not isinstance(leg, MissionLeg) for leg in self.legs)
        ):
            raise NavigationContractError(
                "invalid_mission_legs",
                "Mission plan requires 1..{} typed legs".format(
                    MAX_MISSION_LEGS
                ),
            )
        leg_ids = tuple(leg.leg_id for leg in self.legs)
        if len(set(leg_ids)) != len(leg_ids):
            raise NavigationContractError(
                "duplicate_mission_leg",
                "Mission leg IDs must be unique",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": MISSION_PLAN_SCHEMA,
            "plan_id": self.plan_id,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "based_on_state_version": self.based_on_state_version,
            "based_on_world_model_version": (
                self.based_on_world_model_version
            ),
            "plan_revision": self.plan_revision,
            "legs": [leg.to_dict() for leg in self.legs],
        }

    def assert_matches_snapshot(
        self,
        snapshot: NavigationSnapshot,
    ) -> None:
        if not isinstance(snapshot, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_snapshot",
                "Mission activation requires NavigationSnapshot",
            )
        if (
            snapshot.robot_id != self.robot_id
            or snapshot.controller_instance_id
            != self.controller_instance_id
            or snapshot.state_version != self.based_on_state_version
            or snapshot.world_model_version
            != self.based_on_world_model_version
            or snapshot.motors_running
            or snapshot.touch_pressed
            or snapshot.active_faults
        ):
            raise NavigationContractError(
                "stale_mission_plan",
                "Mission plan does not match a current safe stopped snapshot",
            )


def decode_mission_plan(raw: bytes) -> MissionPlan:
    """Decode an untrusted strict JSON plan without granting motion."""

    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > MAX_MISSION_PLAN_BYTES
    ):
        raise NavigationContractError(
            "invalid_mission_plan_body",
            "Mission plan body is invalid",
        )
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, TypeError, ValueError):
        raise NavigationContractError(
            "invalid_mission_plan_json",
            "Mission planner returned invalid JSON",
        ) from None
    expected = {
        "schema",
        "plan_id",
        "robot_id",
        "controller_instance_id",
        "based_on_state_version",
        "based_on_world_model_version",
        "plan_revision",
        "legs",
    }
    if not isinstance(value, dict) or set(value) != expected:
        raise NavigationContractError(
            "invalid_mission_plan_fields",
            "Mission plan fields are invalid",
        )
    if value.get("schema") != MISSION_PLAN_SCHEMA:
        raise NavigationContractError(
            "invalid_mission_plan_schema",
            "Mission plan schema is not supported",
        )
    raw_legs = value.get("legs")
    if (
        not isinstance(raw_legs, list)
        or not 1 <= len(raw_legs) <= MAX_MISSION_LEGS
    ):
        raise NavigationContractError(
            "invalid_mission_legs",
            "Mission plan has an invalid leg count",
        )
    legs = []
    for raw_leg in raw_legs:
        if (
            not isinstance(raw_leg, dict)
            or set(raw_leg)
            != {
                "leg_id",
                "target_x_mm",
                "target_y_mm",
                "tolerance_mm",
            }
        ):
            raise NavigationContractError(
                "invalid_mission_leg_fields",
                "Mission leg fields are invalid",
            )
        legs.append(MissionLeg(
            leg_id=raw_leg["leg_id"],
            target_x_mm=raw_leg["target_x_mm"],
            target_y_mm=raw_leg["target_y_mm"],
            tolerance_mm=raw_leg["tolerance_mm"],
        ))
    return MissionPlan(
        plan_id=value["plan_id"],
        robot_id=value["robot_id"],
        controller_instance_id=value["controller_instance_id"],
        based_on_state_version=value["based_on_state_version"],
        based_on_world_model_version=value[
            "based_on_world_model_version"
        ],
        plan_revision=value["plan_revision"],
        legs=tuple(legs),
    )


@dataclass(frozen=True)
class MissionLimits:
    """Global budgets shared across every leg in a mission."""

    max_legs: int = MAX_MISSION_LEGS
    max_ticks: int = 1_200
    max_elapsed_ms: int = 180_000
    max_proposals: int = 2_400
    max_replans: int = 1_200
    max_actions: int = 1_100
    max_total_motion_ms: int = 150_000

    def __post_init__(self) -> None:
        integer("max_legs", self.max_legs, 1, MAX_MISSION_LEGS)
        integer("max_ticks", self.max_ticks, 1, 100_000)
        integer(
            "max_elapsed_ms",
            self.max_elapsed_ms,
            1,
            3_600_000,
        )
        integer("max_proposals", self.max_proposals, 1, 100_000)
        integer("max_replans", self.max_replans, 0, 100_000)
        integer("max_actions", self.max_actions, 1, 100_000)
        integer(
            "max_total_motion_ms",
            self.max_total_motion_ms,
            1,
            3_600_000,
        )


@dataclass(frozen=True)
class MissionLegResult:
    leg_index: int
    leg: MissionLeg
    goal: WaypointGoal
    navigation: "NavigationResult"

    def to_dict(self) -> Mapping[str, object]:
        return {
            "leg_index": self.leg_index,
            "leg": self.leg.to_dict(),
            "goal": {
                "goal_id": self.goal.goal_id,
                "goal_epoch": self.goal.goal_epoch,
                "plan_revision": self.goal.plan_revision,
            },
            "navigation": self.navigation.to_dict(),
        }


@dataclass(frozen=True)
class MissionResult:
    plan_id: str
    completed: bool
    termination: str
    legs_completed: int
    ticks: int
    proposals: int
    replans: int
    actions: int
    total_motion_ms: int
    elapsed_ms: int
    final_snapshot: NavigationSnapshot
    terminal_stop_verified: bool
    leg_results: Tuple[MissionLegResult, ...]

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": "robot-navigation-mission-result/v1",
            "plan_id": self.plan_id,
            "completed": self.completed,
            "termination": self.termination,
            "legs_completed": self.legs_completed,
            "ticks": self.ticks,
            "proposals": self.proposals,
            "replans": self.replans,
            "actions": self.actions,
            "total_motion_ms": self.total_motion_ms,
            "elapsed_ms": self.elapsed_ms,
            "terminal_stop_verified": self.terminal_stop_verified,
            "final_pose": {
                "x_mm": self.final_snapshot.pose.x_mm,
                "y_mm": self.final_snapshot.pose.y_mm,
                "heading_mdeg": (
                    self.final_snapshot.pose.heading_mdeg
                ),
            },
            "legs": [
                result.to_dict() for result in self.leg_results
            ],
        }


__all__ = (
    "MAX_MISSION_LEGS",
    "MAX_MISSION_PLAN_BYTES",
    "MISSION_ABORTED",
    "MISSION_BUDGET_EXHAUSTED",
    "MISSION_COMPLETED",
    "MISSION_LEG_FAILED",
    "MISSION_PLAN_REJECTED",
    "MISSION_PLAN_SCHEMA",
    "MISSION_PLAN_STALE",
    "MISSION_SAFETY_STOP",
    "MissionLeg",
    "MissionLegResult",
    "MissionLimits",
    "MissionPlan",
    "MissionResult",
    "decode_mission_plan",
)
