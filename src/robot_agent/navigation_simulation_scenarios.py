"""Route-free scenarios for multi-robot navigation validation."""

from __future__ import annotations

from dataclasses import dataclass

from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .multi_robot_navigation_simulator import (
    MultiRobotNavigationSimulator,
    RectangleObstacle,
    SimulatedRobot,
    SimulationGoal,
)
from .navigation_state import PoseEstimate
from .physical_footprint import RobotFootprint


EV3_SIMULATION_FOOTPRINT = RobotFootprint(
    front_extent_mm=110,
    rear_extent_mm=90,
    left_extent_mm=100,
    right_extent_mm=130,
    clearance_margin_mm=10,
    calibration_status="operator-measured-current-build",
    calibration_evidence="config/ev3rstorm.json physical footprint",
)


@dataclass(frozen=True)
class NavigationSimulationScenario:
    """Only ground truth: world, bodies, starts and final goals."""

    scenario_id: str
    bounds: tuple[int, int, int, int]
    obstacles: tuple[RectangleObstacle, ...]
    robots: tuple[SimulatedRobot, ...]
    goals: tuple[SimulationGoal, ...]

    def __post_init__(self) -> None:
        if (
            not isinstance(self.scenario_id, str)
            or not self.scenario_id
            or self.scenario_id != self.scenario_id.strip()
        ):
            raise ValueError("simulation scenario id is invalid")
        # Build once at definition time so invalid starts and overlapping
        # robot bodies fail before a model run begins.
        self.build()

    def build(self) -> MultiRobotNavigationSimulator:
        return MultiRobotNavigationSimulator(
            bounds=self.bounds,
            obstacles=self.obstacles,
            robots=self.robots,
            goals=self.goals,
        )


def _robots(
    blast_pose: PoseEstimate,
    ev3_pose: PoseEstimate,
    *,
    blast_goal: tuple[int, int],
    ev3_goal: tuple[int, int],
):
    blast_footprint, _sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    )
    return (
        (
            SimulatedRobot(
                "blast",
                blast_pose,
                blast_footprint,
                2_000,
            ),
            SimulatedRobot(
                "ev3",
                ev3_pose,
                EV3_SIMULATION_FOOTPRINT,
                1_000,
            ),
        ),
        (
            SimulationGoal("blast", *blast_goal),
            SimulationGoal("ev3", *ev3_goal),
        ),
    )


def concurrent_clear_room() -> NavigationSimulationScenario:
    robots, goals = _robots(
        PoseEstimate(0, -350, 0),
        PoseEstimate(0, 350, 0),
        blast_goal=(1_600, -350),
        ev3_goal=(1_600, 350),
    )
    return NavigationSimulationScenario(
        scenario_id="concurrent-clear-room",
        bounds=(-800, -1_200, 2_400, 1_200),
        obstacles=(),
        robots=robots,
        goals=goals,
    )


def shared_box_detour() -> NavigationSimulationScenario:
    robots, goals = _robots(
        PoseEstimate(0, -350, 0),
        PoseEstimate(0, 350, 0),
        blast_goal=(1_800, -350),
        ev3_goal=(1_800, 350),
    )
    return NavigationSimulationScenario(
        scenario_id="shared-box-detour",
        bounds=(-800, -1_500, 2_600, 1_500),
        obstacles=(
            RectangleObstacle("shared-box", 450, -550, 850, 550),
        ),
        robots=robots,
        goals=goals,
    )


def staggered_room() -> NavigationSimulationScenario:
    robots, goals = _robots(
        PoseEstimate(-200, -500, 0),
        PoseEstimate(-200, 500, 0),
        blast_goal=(2_300, 500),
        ev3_goal=(2_300, -500),
    )
    return NavigationSimulationScenario(
        scenario_id="staggered-multi-obstacle-room",
        bounds=(-900, -1_600, 3_000, 1_600),
        obstacles=(
            RectangleObstacle("box-1", 300, -750, 700, -150),
            RectangleObstacle("box-2", 700, 250, 1_100, 850),
            RectangleObstacle("box-3", 1_250, -500, 1_650, 100),
            RectangleObstacle("box-4", 1_750, 450, 2_100, 1_000),
        ),
        robots=robots,
        goals=goals,
    )


def dead_end_room() -> NavigationSimulationScenario:
    """A route may require reversing and choosing another opening."""

    robots, goals = _robots(
        PoseEstimate(-300, -550, 0),
        PoseEstimate(-300, 550, 0),
        blast_goal=(2_400, -550),
        ev3_goal=(2_400, 550),
    )
    return NavigationSimulationScenario(
        scenario_id="dead-end-and-backtrack-room",
        bounds=(-1_000, -1_700, 3_200, 1_700),
        obstacles=(
            RectangleObstacle("dead-end-top", 300, -200, 1_500, 50),
            RectangleObstacle("dead-end-bottom", 300, -1_200, 1_500, -950),
            RectangleObstacle("dead-end-cap", 1_250, -950, 1_500, -200),
            RectangleObstacle("upper-block", 900, 300, 1_350, 950),
            RectangleObstacle("far-block", 1_800, -250, 2_150, 500),
        ),
        robots=robots,
        goals=goals,
    )


def navigation_validation_scenarios():
    return (
        concurrent_clear_room(),
        shared_box_detour(),
        staggered_room(),
        dead_end_room(),
    )


def _blast_gemma_scenario(
    scenario_id: str,
    obstacles: tuple[RectangleObstacle, ...],
) -> NavigationSimulationScenario:
    """Build one route-free BLAST case around its current 800 mm mission."""

    blast_footprint, _sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    )
    return NavigationSimulationScenario(
        scenario_id=scenario_id,
        bounds=(-500, -900, 1_250, 900),
        obstacles=obstacles,
        robots=(SimulatedRobot(
            "blast",
            PoseEstimate(0, 0, 0),
            blast_footprint,
            2_000,
        ),),
        goals=(SimulationGoal("blast", 800, 0),),
    )


def blast_box_front() -> NavigationSimulationScenario:
    return _blast_gemma_scenario(
        "blast-box-front",
        (RectangleObstacle("front-box", 320, -180, 520, 180),),
    )


def blast_box_at_side() -> NavigationSimulationScenario:
    return _blast_gemma_scenario(
        "blast-box-at-side",
        (RectangleObstacle("side-box", 250, 220, 550, 500),),
    )


def blast_boxes_both_sides() -> NavigationSimulationScenario:
    return _blast_gemma_scenario(
        "blast-boxes-both-sides",
        (
            RectangleObstacle("left-box", 150, 260, 650, 650),
            RectangleObstacle("right-box", 150, -650, 650, -260),
        ),
    )


def blast_straight_corridor() -> NavigationSimulationScenario:
    return _blast_gemma_scenario(
        "blast-straight-corridor",
        (
            RectangleObstacle("left-wall", -200, 350, 1_050, 700),
            RectangleObstacle("right-wall", -200, -700, 1_050, -350),
        ),
    )


def blast_bent_corridor() -> NavigationSimulationScenario:
    """A dogleg requires leaving the goal axis and returning after a corner."""

    return _blast_gemma_scenario(
        "blast-bent-corridor",
        (
            RectangleObstacle("upper-wall", -200, 450, 1_050, 700),
            RectangleObstacle("lower-entry-wall", -200, -700, 0, -450),
            RectangleObstacle("corner-block", 450, -100, 650, 450),
            RectangleObstacle("lower-exit-wall", 650, -800, 1_050, -650),
        ),
    )


def blast_gemma_validation_scenarios():
    """Small strategic cases; none contains a route or waypoint answer."""

    return (
        blast_box_front(),
        blast_box_at_side(),
        blast_boxes_both_sides(),
        blast_straight_corridor(),
        blast_bent_corridor(),
    )


__all__ = (
    "EV3_SIMULATION_FOOTPRINT",
    "NavigationSimulationScenario",
    "blast_bent_corridor",
    "blast_box_at_side",
    "blast_box_front",
    "blast_boxes_both_sides",
    "blast_gemma_validation_scenarios",
    "blast_straight_corridor",
    "concurrent_clear_room",
    "dead_end_room",
    "navigation_validation_scenarios",
    "shared_box_detour",
    "staggered_room",
)
