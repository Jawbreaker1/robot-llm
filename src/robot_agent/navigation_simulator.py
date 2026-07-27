"""Deterministic 2D differential-drive plant for navigation experiments."""

from dataclasses import dataclass
from collections import deque
import math
import threading
from typing import Optional, Tuple

from .navigation_contract import (
    DriveCalibrationProfile,
    DrivePulse,
    MotionAuthority,
    NavigationContractError,
    WaypointGoal,
    identifier,
    integer,
)
from .navigation_state import (
    ClearanceEvidence,
    NavigationSnapshot,
    PoseEstimate,
)


def normalize_heading_mdeg(value: int) -> int:
    value = value % 360_000
    if value >= 180_000:
        value -= 360_000
    return value


@dataclass(frozen=True)
class CircleObstacle:
    obstacle_id: str
    x_mm: int
    y_mm: int
    radius_mm: int

    def __post_init__(self) -> None:
        identifier("obstacle_id", self.obstacle_id)
        integer("x_mm", self.x_mm, 0, 1_000_000)
        integer("y_mm", self.y_mm, 0, 1_000_000)
        integer("radius_mm", self.radius_mm, 1, 100_000)


@dataclass(frozen=True)
class SimulationWorld:
    width_mm: int
    height_mm: int
    obstacles: Tuple[CircleObstacle, ...] = ()

    def __post_init__(self) -> None:
        integer("width_mm", self.width_mm, 100, 1_000_000)
        integer("height_mm", self.height_mm, 100, 1_000_000)
        if (
            not isinstance(self.obstacles, tuple)
            or any(
                not isinstance(obstacle, CircleObstacle)
                for obstacle in self.obstacles
            )
            or len(
                {obstacle.obstacle_id for obstacle in self.obstacles}
            )
            != len(self.obstacles)
        ):
            raise NavigationContractError(
                "invalid_obstacles",
                "World obstacles must be a unique tuple",
            )
        for obstacle in self.obstacles:
            if (
                obstacle.x_mm - obstacle.radius_mm < 0
                or obstacle.y_mm - obstacle.radius_mm < 0
                or obstacle.x_mm + obstacle.radius_mm > self.width_mm
                or obstacle.y_mm + obstacle.radius_mm > self.height_mm
            ):
                raise NavigationContractError(
                    "obstacle_outside_world",
                    "Obstacle must fit inside the world",
                )


@dataclass(frozen=True)
class SimulationSettings:
    robot_radius_mm: int = 65
    range_max_mm: int = 1_000
    near_threshold_mm: int = 120
    idle_tick_ms: int = 20
    max_substep_ms: int = 5
    trace_capacity: int = 2_048

    def __post_init__(self) -> None:
        integer("robot_radius_mm", self.robot_radius_mm, 1, 10_000)
        integer("range_max_mm", self.range_max_mm, 1, 100_000)
        integer(
            "near_threshold_mm",
            self.near_threshold_mm,
            1,
            self.range_max_mm,
        )
        integer("idle_tick_ms", self.idle_tick_ms, 1, 1_000)
        integer("max_substep_ms", self.max_substep_ms, 1, 100)
        integer("trace_capacity", self.trace_capacity, 1, 100_000)


class DifferentialDriveSimulator:
    """A synchronous plant with swept collision checks.

    Ground-truth collision geometry is independent of the ray-based sensor
    surface.  A coarse or faulty sensor therefore cannot let a pulse tunnel
    through an obstacle unnoticed by the test oracle.
    """

    def __init__(
        self,
        world: SimulationWorld,
        profile: DriveCalibrationProfile,
        initial_pose: PoseEstimate,
        motion_authority: MotionAuthority,
        expected_arbiter_id: str = "navigation-motion-supervisor",
        robot_id: str = "ev3rstorm-sim",
        controller_instance_id: str = "nav-sim-instance-1",
        settings: SimulationSettings = SimulationSettings(),
        start_ms: int = 10_000,
    ):
        if not isinstance(world, SimulationWorld):
            raise NavigationContractError(
                "invalid_world",
                "Simulator requires SimulationWorld",
            )
        if not isinstance(profile, DriveCalibrationProfile):
            raise NavigationContractError(
                "invalid_calibration",
                "Simulator requires DriveCalibrationProfile",
            )
        if profile.status != "simulation_only":
            raise NavigationContractError(
                "physical_profile_forbidden",
                "Navigation simulator accepts simulation_only profiles",
            )
        profile.require_complete_geometry()
        if not isinstance(initial_pose, PoseEstimate):
            raise NavigationContractError(
                "invalid_initial_pose",
                "Simulator requires PoseEstimate",
            )
        if not isinstance(motion_authority, MotionAuthority):
            raise NavigationContractError(
                "missing_motion_authority",
                "Simulator requires MotionAuthority",
            )
        if not isinstance(settings, SimulationSettings):
            raise NavigationContractError(
                "invalid_simulation_settings",
                "Simulator settings are invalid",
            )
        identifier("expected_arbiter_id", expected_arbiter_id)
        identifier("robot_id", robot_id)
        identifier("controller_instance_id", controller_instance_id)
        integer("start_ms", start_ms, 0, 2**63 - 1)

        self.world = world
        self.profile = profile
        self.settings = settings
        self.expected_arbiter_id = expected_arbiter_id
        self._motion_authority = motion_authority
        self.robot_id = robot_id
        self.controller_instance_id = controller_instance_id
        self._x_mm = float(initial_pose.x_mm)
        self._y_mm = float(initial_pose.y_mm)
        self._heading_rad = math.radians(
            initial_pose.heading_mdeg / 1_000.0
        )
        self._left_encoder_mdeg = 0.0
        self._right_encoder_mdeg = 0.0
        self._state_version = 1
        self._world_model_version = 1
        self._now_ms = start_ms
        self._touch_pressed = False
        self._active_faults = set()
        self._collision_count = 0
        self._motion_lock = threading.RLock()
        self._applied_decision_ids = set()
        self._applied_decision_order = deque()
        self._applied_pulses = deque(
            maxlen=self.settings.trace_capacity
        )
        if self._collides(self._x_mm, self._y_mm):
            raise NavigationContractError(
                "initial_collision",
                "Initial pose collides with world geometry",
            )

    def clock_ms(self) -> int:
        with self._motion_lock:
            return self._now_ms

    @property
    def collision_count(self) -> int:
        with self._motion_lock:
            return self._collision_count

    @property
    def applied_pulses(self) -> Tuple[DrivePulse, ...]:
        with self._motion_lock:
            return tuple(self._applied_pulses)

    def update_world(self, world: SimulationWorld) -> None:
        """Atomically replace simulated geometry and invalidate old pulses."""

        with self._motion_lock:
            self._update_world_locked(world)

    def _update_world_locked(self, world: SimulationWorld) -> None:
        if not isinstance(world, SimulationWorld):
            raise NavigationContractError(
                "invalid_world",
                "World update requires SimulationWorld",
            )
        previous = self.world
        self.world = world
        if self._collides(self._x_mm, self._y_mm):
            self.world = previous
            raise NavigationContractError(
                "world_update_collision",
                "Updated world collides with the current robot pose",
            )
        self._world_model_version += 1
        self._state_version += 1
        self._now_ms += self.settings.idle_tick_ms

    def _collides(self, x_mm: float, y_mm: float) -> bool:
        radius = self.settings.robot_radius_mm
        if (
            x_mm <= radius
            or y_mm <= radius
            or x_mm >= self.world.width_mm - radius
            or y_mm >= self.world.height_mm - radius
        ):
            return True
        for obstacle in self.world.obstacles:
            minimum = radius + obstacle.radius_mm
            if (
                (x_mm - obstacle.x_mm) ** 2
                + (y_mm - obstacle.y_mm) ** 2
                <= minimum**2
            ):
                return True
        return False

    def _ray_clearance_observation(
        self,
        angle_rad: float,
    ) -> Tuple[int, Optional[str]]:
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        radius = self.settings.robot_radius_mm
        boundary_candidates = []

        if dx > 1e-12:
            boundary_candidates.append(
                (self.world.width_mm - radius - self._x_mm) / dx
            )
        elif dx < -1e-12:
            boundary_candidates.append((radius - self._x_mm) / dx)
        if dy > 1e-12:
            boundary_candidates.append(
                (self.world.height_mm - radius - self._y_mm) / dy
            )
        elif dy < -1e-12:
            boundary_candidates.append((radius - self._y_mm) / dy)

        obstacle_candidates = []
        for obstacle in self.world.obstacles:
            expanded = radius + obstacle.radius_mm
            offset_x = obstacle.x_mm - self._x_mm
            offset_y = obstacle.y_mm - self._y_mm
            projection = offset_x * dx + offset_y * dy
            if projection < 0:
                continue
            perpendicular_sq = (
                offset_x**2 + offset_y**2 - projection**2
            )
            if perpendicular_sq > expanded**2:
                continue
            half_chord = math.sqrt(
                max(0.0, expanded**2 - perpendicular_sq)
            )
            entry = projection - half_chord
            if entry >= 0:
                obstacle_candidates.append(
                    (entry, obstacle.obstacle_id)
                )

        positive_boundaries = [
            value for value in boundary_candidates if value >= 0
        ]
        positive_obstacles = [
            candidate
            for candidate in obstacle_candidates
            if candidate[0] >= 0
        ]
        positive = positive_boundaries + [
            value for value, _obstacle_id in positive_obstacles
        ]
        clearance = (
            self.settings.range_max_mm
            if not positive
            else min(self.settings.range_max_mm, min(positive))
        )
        nearest_boundary = (
            min(positive_boundaries)
            if positive_boundaries
            else math.inf
        )
        nearest_obstacle = (
            min(
                positive_obstacles,
                key=lambda candidate: (candidate[0], candidate[1]),
            )
            if positive_obstacles
            else None
        )
        forward_object_id = None
        if (
            nearest_obstacle is not None
            and nearest_obstacle[0] <= nearest_boundary
            and nearest_obstacle[0] <= self.settings.range_max_mm
        ):
            forward_object_id = nearest_obstacle[1]
        return (
            max(0, int(math.floor(clearance))),
            forward_object_id,
        )

    def _ray_clearance_mm(self, angle_rad: float) -> int:
        clearance_mm, _object_id = self._ray_clearance_observation(
            angle_rad
        )
        return clearance_mm

    def _pose(self) -> PoseEstimate:
        return PoseEstimate(
            x_mm=int(round(self._x_mm)),
            y_mm=int(round(self._y_mm)),
            heading_mdeg=normalize_heading_mdeg(
                int(round(math.degrees(self._heading_rad) * 1_000))
            ),
        )

    def observe(self, goal: WaypointGoal) -> NavigationSnapshot:
        with self._motion_lock:
            return self._observe_locked(goal)

    def _observe_locked(
        self,
        goal: WaypointGoal,
    ) -> NavigationSnapshot:
        if not isinstance(goal, WaypointGoal):
            raise NavigationContractError(
                "invalid_goal",
                "Simulator observation requires WaypointGoal",
            )
        forward, forward_object_id = self._ray_clearance_observation(
            self._heading_rad
        )
        left = self._ray_clearance_mm(
            self._heading_rad + math.radians(45)
        )
        right = self._ray_clearance_mm(
            self._heading_rad - math.radians(45)
        )
        return NavigationSnapshot(
            robot_id=self.robot_id,
            controller_instance_id=self.controller_instance_id,
            goal_id=goal.goal_id,
            goal_epoch=goal.goal_epoch,
            plan_revision=goal.plan_revision,
            state_version=self._state_version,
            world_model_version=self._world_model_version,
            captured_at_host_ms=self._now_ms,
            state_observed_at_ms=self._now_ms,
            pose=self._pose(),
            left_encoder_mdeg=int(round(self._left_encoder_mdeg)),
            right_encoder_mdeg=int(round(self._right_encoder_mdeg)),
            motors_running=False,
            touch_pressed=self._touch_pressed,
            active_faults=tuple(sorted(self._active_faults)),
            clearance=ClearanceEvidence(
                source="simulation_metric",
                observed_at_ms=self._now_ms,
                near_obstacle_latched=(
                    forward <= self.settings.near_threshold_mm
                ),
                forward_mm=forward,
                left_mm=left,
                right_mm=right,
                forward_object_id=forward_object_id,
            ),
        )

    def apply(
        self,
        pulse: DrivePulse,
        goal: WaypointGoal,
    ) -> NavigationSnapshot:
        with self._motion_lock:
            return self._apply_locked(pulse, goal)

    def _apply_locked(
        self,
        pulse: DrivePulse,
        goal: WaypointGoal,
    ) -> NavigationSnapshot:
        if not isinstance(pulse, DrivePulse):
            raise NavigationContractError(
                "invalid_drive_pulse",
                "Simulator only accepts DrivePulse",
            )
        if not isinstance(goal, WaypointGoal):
            raise NavigationContractError(
                "invalid_goal",
                "Simulator apply requires WaypointGoal",
            )
        if pulse.decision_id in self._applied_decision_ids:
            raise NavigationContractError(
                "replayed_drive_pulse",
                "Drive decision has already been applied",
            )
        self._motion_authority.consume(pulse)
        if (
            pulse.arbiter_id != self.expected_arbiter_id
            or pulse.robot_id != self.robot_id
            or pulse.controller_instance_id
            != self.controller_instance_id
        ):
            raise NavigationContractError(
                "unauthorized_motion_owner",
                "Drive pulse did not come from the expected supervisor",
            )
        if (
            pulse.goal_id != goal.goal_id
            or pulse.goal_epoch != goal.goal_epoch
            or pulse.plan_revision != goal.plan_revision
            or pulse.based_on_state_version != self._state_version
            or pulse.based_on_world_model_version
            != self._world_model_version
        ):
            raise NavigationContractError(
                "stale_drive_pulse",
                "Drive pulse is not bound to current state",
            )
        if pulse.kind == "DRIVE" and (
            abs(pulse.left_speed_dps)
            > self.profile.max_wheel_speed_dps
            or abs(pulse.right_speed_dps)
            > self.profile.max_wheel_speed_dps
            or pulse.duration_ms > self.profile.max_pulse_ms
        ):
            raise NavigationContractError(
                "drive_pulse_out_of_bounds",
                "Drive pulse exceeds simulation calibration",
            )

        if (
            len(self._applied_decision_order)
            >= self.settings.trace_capacity
        ):
            expired_id = self._applied_decision_order.popleft()
            self._applied_decision_ids.discard(expired_id)
        self._applied_decision_ids.add(pulse.decision_id)
        self._applied_decision_order.append(pulse.decision_id)
        self._applied_pulses.append(pulse)
        if pulse.kind == "STOP":
            self._state_version += 1
            self._now_ms += self.settings.idle_tick_ms
            return self.observe(goal)

        logical_left_dps = (
            pulse.left_speed_dps * self.profile.left_motor_sign
        )
        logical_right_dps = (
            pulse.right_speed_dps * self.profile.right_motor_sign
        )
        encoder_per_mm = self.profile.encoder_mdeg_per_mm
        encoder_per_body_degree = (
            self.profile.encoder_mdeg_per_body_degree
        )
        left_mm_s = logical_left_dps * 1_000.0 / encoder_per_mm
        right_mm_s = logical_right_dps * 1_000.0 / encoder_per_mm
        linear_mm_s = (left_mm_s + right_mm_s) / 2.0
        angular_deg_s = (
            logical_right_dps - logical_left_dps
        ) * 500.0 / encoder_per_body_degree

        estimated_distance = (
            abs(linear_mm_s) * pulse.duration_ms / 1_000.0
        )
        estimated_angle = (
            abs(angular_deg_s) * pulse.duration_ms / 1_000.0
        )
        steps = max(
            1,
            int(
                math.ceil(
                    pulse.duration_ms / self.settings.max_substep_ms
                )
            ),
            int(math.ceil(estimated_distance)),
            int(math.ceil(estimated_angle / 0.5)),
        )
        dt_s = pulse.duration_ms / 1_000.0 / steps
        executed_steps = 0

        for _ in range(steps):
            next_heading = (
                self._heading_rad + math.radians(angular_deg_s) * dt_s
            )
            mid_heading = (
                self._heading_rad + math.radians(angular_deg_s) * dt_s / 2
            )
            next_x = (
                self._x_mm
                + linear_mm_s * math.cos(mid_heading) * dt_s
            )
            next_y = (
                self._y_mm
                + linear_mm_s * math.sin(mid_heading) * dt_s
            )
            if self._collides(next_x, next_y):
                self._touch_pressed = True
                self._active_faults.add("collision_oracle")
                self._collision_count += 1
                break
            self._x_mm = next_x
            self._y_mm = next_y
            self._heading_rad = next_heading
            self._left_encoder_mdeg += logical_left_dps * 1_000 * dt_s
            self._right_encoder_mdeg += (
                logical_right_dps * 1_000 * dt_s
            )
            executed_steps += 1

        elapsed_ms = max(
            1,
            int(round(executed_steps * dt_s * 1_000)),
        )
        self._state_version += 1
        self._now_ms += elapsed_ms
        return self.observe(goal)
