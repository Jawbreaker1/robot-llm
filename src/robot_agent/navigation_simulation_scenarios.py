"""Route-free scenarios for multi-robot navigation validation."""

from __future__ import annotations

from dataclasses import dataclass, replace

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


def blast_open_floor() -> NavigationSimulationScenario:
    """Open room whose forward wall stays beyond ultrasonic range."""
    return replace(
        _blast_gemma_scenario("blast-open-floor", ()),
        bounds=(-1_200, -1_200, 5_000, 1_200),
    )


def blast_box_front() -> NavigationSimulationScenario:
    return _blast_gemma_scenario(
        "blast-box-front",
        (RectangleObstacle("front-box", 320, -180, 520, 180),),
    )


def blast_measured_box_and_chair() -> NavigationSimulationScenario:
    """Regression world based on the 2026-09-02 physical BLAST run.

    The box face was about 280 mm ahead of the forward-mounted sensor. A chair
    foot sat farther to the right, where an unnecessarily wide detour collided
    with it. The useful route is the short gap between those two obstacles.
    """

    blast_footprint, _sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    )
    return NavigationSimulationScenario(
        scenario_id="blast-measured-box-and-chair",
        bounds=(-600, -1_400, 1_400, 1_000),
        obstacles=(
            RectangleObstacle("front-box", 390, -190, 620, 260),
            RectangleObstacle("chair-foot", -80, -760, 120, -680),
        ),
        robots=(SimulatedRobot(
            "blast",
            PoseEstimate(0, 0, 0),
            blast_footprint,
            2_000,
        ),),
        goals=(SimulationGoal("blast", 800, 0),),
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
        blast_open_floor(),
        blast_box_front(),
        blast_measured_box_and_chair(),
        blast_box_at_side(),
        blast_boxes_both_sides(),
        blast_straight_corridor(),
        blast_bent_corridor(),
    )


def ev3_validation_scenarios():
    """Route-free EV3 worlds for the production episode runner.

    Reuse the shared layouts, with single-robot starts and straight-ahead goals
    within the current two-metre mission contract. Nothing here chooses a route.
    """
    from .ev3rstorm_profile import EV3RSTORMProfile
    footprint = EV3RSTORMProfile().hazard_calibration.robot_footprint

    def case(name, bounds, obstacles, *, start=PoseEstimate(0, 0, 0), distance=800):
        return NavigationSimulationScenario(
            scenario_id=name, bounds=bounds, obstacles=tuple(obstacles),
            robots=(SimulatedRobot("ev3", start, footprint, 1_000),),
            goals=(SimulationGoal("ev3", start.x_mm + distance, start.y_mm, tolerance_mm=50),),
        )

    bounds = (-1_000, -1_000, 2_000, 1_000)
    box = RectangleObstacle("front", 200, -100, 400, 100)
    corridor, bend = blast_straight_corridor(), blast_bent_corridor()
    staggered, dead_end = staggered_room(), dead_end_room()
    return (
        case("open", bounds, ()),
        case("front", bounds, (box,)),
        case("front-right", bounds, (box, RectangleObstacle("right", -100, -400, 100, -200))),
        case("front-left", bounds, (box, RectangleObstacle("left", -100, 200, 100, 400))),
        case("corridor", corridor.bounds, corridor.obstacles),
        case("bend", bend.bounds, bend.obstacles),
        case("staggered", staggered.bounds, staggered.obstacles,
             start=PoseEstimate(-200, -500, 0), distance=2_000),
        case("dead-end", dead_end.bounds, dead_end.obstacles,
             start=PoseEstimate(-300, -550, 0), distance=2_000),
        case("many-boxes", staggered.bounds, (*staggered.obstacles,
             RectangleObstacle("box-5", -650, 200, -350, 600),
             RectangleObstacle("box-6", 0, 900, 300, 1_200),
             RectangleObstacle("box-7", 850, -1_200, 1_150, -900),
             RectangleObstacle("box-8", 1_750, -1_200, 2_100, -850)),
             start=PoseEstimate(-200, -500, 0), distance=2_000),
        case("maze", (-700, -1_200, 2_400, 1_200), (
             RectangleObstacle("top-wall", -700, 900, 2_400, 1_050),
             RectangleObstacle("bottom-wall", -700, -1_050, 2_400, -900),
             RectangleObstacle("baffle-1", 250, -900, 400, 150),
             RectangleObstacle("baffle-2", 800, -150, 950, 900),
             RectangleObstacle("baffle-3", 1_350, -900, 1_500, 150)),
             start=PoseEstimate(-200, 0, 0), distance=2_000),
    )


__all__ = (
    "ev3_validation_scenarios",
    "EV3_SIMULATION_FOOTPRINT",
    "NavigationSimulationScenario",
    "blast_bent_corridor",
    "blast_box_at_side",
    "blast_box_front",
    "blast_measured_box_and_chair",
    "blast_open_floor",
    "blast_boxes_both_sides",
    "blast_gemma_validation_scenarios",
    "blast_straight_corridor",
    "concurrent_clear_room",
    "dead_end_room",
    "navigation_validation_scenarios",
    "shared_box_detour",
    "staggered_room",
)
