"""Policy-free 2D world for concurrent robot navigation validation.

Scenarios describe only geometry, robot bodies, starting poses and goals.
There are deliberately no waypoints, routes or obstacle-side choices here;
those must come from the production agent connected to each simulated robot.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import math
import threading
from typing import Callable, Mapping, Sequence

from .navigation_state import PoseEstimate
from .physical_footprint import RobotFootprint
from .physical_odometry import normalize_heading_mdeg


@dataclass(frozen=True)
class RectangleObstacle:
    obstacle_id: str
    min_x_mm: int
    min_y_mm: int
    max_x_mm: int
    max_y_mm: int

    def __post_init__(self) -> None:
        if (
            not isinstance(self.obstacle_id, str)
            or not self.obstacle_id
            or self.obstacle_id != self.obstacle_id.strip()
            or any(type(value) is not int for value in (
                self.min_x_mm,
                self.min_y_mm,
                self.max_x_mm,
                self.max_y_mm,
            ))
            or self.min_x_mm >= self.max_x_mm
            or self.min_y_mm >= self.max_y_mm
        ):
            raise ValueError("simulation obstacle is invalid")


@dataclass(frozen=True)
class SimulatedRobot:
    robot_id: str
    pose: PoseEstimate
    footprint: RobotFootprint
    sensor_range_mm: int

    def __post_init__(self) -> None:
        if (
            not isinstance(self.robot_id, str)
            or not self.robot_id
            or self.robot_id != self.robot_id.strip()
            or not isinstance(self.pose, PoseEstimate)
            or not isinstance(self.footprint, RobotFootprint)
            or type(self.sensor_range_mm) is not int
            or not 100 <= self.sensor_range_mm <= 10_000
        ):
            raise ValueError("simulated robot is invalid")


@dataclass(frozen=True)
class SimulationGoal:
    robot_id: str
    x_mm: int
    y_mm: int
    tolerance_mm: int = 150

    def __post_init__(self) -> None:
        if (
            not isinstance(self.robot_id, str)
            or not self.robot_id
            or any(type(value) is not int for value in (
                self.x_mm,
                self.y_mm,
                self.tolerance_mm,
            ))
            or not 1 <= self.tolerance_mm <= 2_000
        ):
            raise ValueError("simulation goal is invalid")


@dataclass(frozen=True)
class SimulationEvent:
    sequence: int
    robot_id: str
    kind: str
    pose: PoseEstimate
    value: int = 0


class MultiRobotNavigationSimulator:
    """One synchronized world shared by all simulated robots.

    A conservative circle around each measured robot footprint is used for
    collision checks.  This intentionally leaves LEGO-scale breathing room
    without pretending that millimetre-perfect motion is realistic.
    """

    def __init__(
        self,
        *,
        bounds: tuple[int, int, int, int],
        obstacles: Sequence[RectangleObstacle],
        robots: Sequence[SimulatedRobot],
        goals: Sequence[SimulationGoal],
    ) -> None:
        if (
            not isinstance(bounds, tuple)
            or len(bounds) != 4
            or any(type(value) is not int for value in bounds)
        ):
            raise ValueError("simulation bounds are invalid")
        min_x, min_y, max_x, max_y = bounds
        if min_x >= max_x or min_y >= max_y:
            raise ValueError("simulation bounds are empty")
        self.bounds = bounds
        self.obstacles = tuple(obstacles)
        self._robot_specs = {robot.robot_id: robot for robot in robots}
        self.goals = {goal.robot_id: goal for goal in goals}
        if (
            len(self._robot_specs) != len(tuple(robots))
            or len(self.goals) != len(tuple(goals))
            or set(self._robot_specs) != set(self.goals)
            or len({item.obstacle_id for item in self.obstacles})
            != len(self.obstacles)
        ):
            raise ValueError("simulation robot, goal or obstacle ids differ")
        self._poses = {
            robot.robot_id: robot.pose for robot in self._robot_specs.values()
        }
        self._events: list[SimulationEvent] = []
        self._lock = threading.RLock()
        for robot_id in self._poses:
            if self._pose_collides(robot_id, self._poses[robot_id]):
                raise ValueError("initial robot pose collides")

    @staticmethod
    def _radius(spec: SimulatedRobot) -> float:
        return (
            spec.footprint.maximum_corner_radius_mm
            + spec.footprint.clearance_margin_mm
        )

    @staticmethod
    def _point_rectangle_distance(
        x_mm: float,
        y_mm: float,
        obstacle: RectangleObstacle,
    ) -> float:
        dx = max(obstacle.min_x_mm - x_mm, 0.0, x_mm - obstacle.max_x_mm)
        dy = max(obstacle.min_y_mm - y_mm, 0.0, y_mm - obstacle.max_y_mm)
        return math.hypot(dx, dy)

    def _pose_collides(self, robot_id: str, pose: PoseEstimate) -> bool:
        radius = self._radius(self._robot_specs[robot_id])
        min_x, min_y, max_x, max_y = self.bounds
        if (
            pose.x_mm - radius <= min_x
            or pose.y_mm - radius <= min_y
            or pose.x_mm + radius >= max_x
            or pose.y_mm + radius >= max_y
        ):
            return True
        if any(
            self._point_rectangle_distance(pose.x_mm, pose.y_mm, obstacle)
            <= radius
            for obstacle in self.obstacles
        ):
            return True
        for peer_id, peer_pose in self._poses.items():
            if peer_id == robot_id:
                continue
            peer_radius = self._radius(self._robot_specs[peer_id])
            if math.hypot(
                pose.x_mm - peer_pose.x_mm,
                pose.y_mm - peer_pose.y_mm,
            ) <= radius + peer_radius:
                return True
        return False

    def _record(self, robot_id: str, kind: str, value: int = 0) -> None:
        with self._lock:
            self._events.append(SimulationEvent(
                sequence=len(self._events) + 1,
                robot_id=robot_id,
                kind=kind,
                pose=self._poses[robot_id],
                value=value,
            ))

    @property
    def events(self) -> tuple[SimulationEvent, ...]:
        with self._lock:
            return tuple(self._events)

    def pose(self, robot_id: str) -> PoseEstimate:
        with self._lock:
            return self._poses[robot_id]

    def goal_reached(self, robot_id: str) -> bool:
        with self._lock:
            pose = self._poses[robot_id]
            goal = self.goals[robot_id]
            return math.hypot(
                pose.x_mm - goal.x_mm,
                pose.y_mm - goal.y_mm,
            ) <= goal.tolerance_mm

    def rotate(self, robot_id: str, delta_mdeg: int) -> PoseEstimate:
        if type(delta_mdeg) is not int or not -360_000 <= delta_mdeg <= 360_000:
            raise ValueError("simulation rotation is invalid")
        with self._lock:
            current = self._poses[robot_id]
            updated = PoseEstimate(
                x_mm=current.x_mm,
                y_mm=current.y_mm,
                heading_mdeg=normalize_heading_mdeg(
                    current.heading_mdeg + delta_mdeg
                ),
            )
            self._poses[robot_id] = updated
            self._record(robot_id, "rotate", delta_mdeg)
            return updated

    def move(self, robot_id: str, distance_mm: int) -> int:
        """Move until the requested distance or the first blocked substep."""

        if type(distance_mm) is not int or not -2_000 <= distance_mm <= 2_000:
            raise ValueError("simulation movement is invalid")
        with self._lock:
            start = self._poses[robot_id]
            direction = 1 if distance_mm >= 0 else -1
            remaining = abs(distance_mm)
            moved = 0
            heading = math.radians(start.heading_mdeg / 1_000.0)
            while remaining:
                step = min(10, remaining)
                attempted = moved + direction * step
                candidate = PoseEstimate(
                    x_mm=round(start.x_mm + attempted * math.cos(heading)),
                    y_mm=round(start.y_mm + attempted * math.sin(heading)),
                    heading_mdeg=start.heading_mdeg,
                )
                if self._pose_collides(robot_id, candidate):
                    self._record(robot_id, "blocked", moved)
                    return moved
                self._poses[robot_id] = candidate
                moved = attempted
                remaining -= step
            self._record(robot_id, "move", moved)
            return moved

    def scan(
        self,
        robot_id: str,
        relative_headings_mdeg: Sequence[int],
    ) -> tuple[tuple[int, int, str | None], ...]:
        """Return ``(bearing, clearance, object_id)`` without route hints."""

        with self._lock:
            if not relative_headings_mdeg or any(
                type(value) is not int or not -180_000 <= value <= 180_000
                for value in relative_headings_mdeg
            ):
                raise ValueError("simulation scan bearings are invalid")
            readings = tuple(
                self._ray_reading(robot_id, relative)
                for relative in relative_headings_mdeg
            )
            self._record(robot_id, "scan", len(readings))
            return readings

    def _ray_reading(
        self,
        robot_id: str,
        relative_heading_mdeg: int,
    ) -> tuple[int, int, str | None]:
        pose = self._poses[robot_id]
        spec = self._robot_specs[robot_id]
        absolute = math.radians(
            (pose.heading_mdeg + relative_heading_mdeg) / 1_000.0
        )
        dx, dy = math.cos(absolute), math.sin(absolute)
        step_mm = 10
        for distance in range(0, spec.sensor_range_mm + 1, step_mm):
            x_mm = pose.x_mm + distance * dx
            y_mm = pose.y_mm + distance * dy
            min_x, min_y, max_x, max_y = self.bounds
            if not (min_x <= x_mm <= max_x and min_y <= y_mm <= max_y):
                return relative_heading_mdeg, distance, "world-boundary"
            for obstacle in self.obstacles:
                if (
                    obstacle.min_x_mm <= x_mm <= obstacle.max_x_mm
                    and obstacle.min_y_mm <= y_mm <= obstacle.max_y_mm
                ):
                    return relative_heading_mdeg, distance, obstacle.obstacle_id
            for peer_id, peer_pose in self._poses.items():
                if peer_id == robot_id:
                    continue
                if math.hypot(x_mm - peer_pose.x_mm, y_mm - peer_pose.y_mm) <= (
                    self._radius(self._robot_specs[peer_id])
                ):
                    return relative_heading_mdeg, distance, peer_id
        return relative_heading_mdeg, spec.sensor_range_mm, None

    def run_concurrently(
        self,
        runners: Mapping[str, Callable[["MultiRobotNavigationSimulator"], object]],
    ) -> Mapping[str, object]:
        """Start one real agent runner per robot at the same barrier."""

        if set(runners) != set(self._robot_specs) or any(
            not callable(runner) for runner in runners.values()
        ):
            raise ValueError("one simulation runner per robot is required")
        barrier = threading.Barrier(len(runners))

        def run_one(robot_id: str, runner):
            barrier.wait()
            self._record(robot_id, "agent_started")
            # No runner may finish before every robot has entered the same
            # simulation window. The agents still choose and execute their
            # own actions independently after this point.
            barrier.wait()
            result = runner(self)
            self._record(robot_id, "agent_stopped")
            return result

        with ThreadPoolExecutor(max_workers=len(runners)) as executor:
            futures = {
                robot_id: executor.submit(run_one, robot_id, runner)
                for robot_id, runner in runners.items()
            }
            return {
                robot_id: future.result()
                for robot_id, future in futures.items()
            }


__all__ = (
    "MultiRobotNavigationSimulator",
    "RectangleObstacle",
    "SimulatedRobot",
    "SimulationEvent",
    "SimulationGoal",
)
