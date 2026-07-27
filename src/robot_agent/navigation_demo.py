"""Run the autonomous navigation slice without robot hardware."""

import argparse
import itertools
import json
from pathlib import Path
from typing import Mapping, Optional, Sequence

from .navigation_contract import (
    DriveCalibrationProfile,
    MotionAuthority,
    NavigationContractError,
    WaypointGoal,
)
from .navigation_episode import (
    GoalSeekingBehavior,
    NavigationEpisode,
    NavigationLimits,
    ObstacleAvoidanceBehavior,
)
from .navigation_simulator import (
    CircleObstacle,
    DifferentialDriveSimulator,
    SimulationSettings,
    SimulationWorld,
)
from .navigation_state import (
    PoseEstimate,
    ProposalInbox,
    ProposalSourcePolicy,
)
from .navigation_supervisor import MotionPolicy, MotionSupervisor


DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "config"
    / "navigation_simulation.json"
)
MAX_CONFIG_BYTES = 64 * 1024


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _object(
    name: str,
    value,
    fields,
) -> Mapping[str, object]:
    if not isinstance(value, dict) or set(value) != set(fields):
        raise NavigationContractError(
            "invalid_demo_config",
            "{} fields are invalid".format(name),
        )
    return value


def load_demo_config(path: Path):
    if not isinstance(path, Path):
        raise NavigationContractError(
            "invalid_demo_config_path",
            "Demo config path must be Path",
        )
    raw = path.read_bytes()
    if not raw or len(raw) > MAX_CONFIG_BYTES:
        raise NavigationContractError(
            "invalid_demo_config",
            "Demo config size is invalid",
        )
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object,
        )
    except (UnicodeDecodeError, ValueError):
        raise NavigationContractError(
            "invalid_demo_config",
            "Demo config is not strict JSON",
        ) from None
    root = _object(
        "root",
        value,
        {"schema", "status", "warning", "calibration", "world"},
    )
    if (
        root["schema"] != "robot-navigation-simulation/v1"
        or root["status"] != "simulation_only"
        or not isinstance(root["warning"], str)
        or not root["warning"]
    ):
        raise NavigationContractError(
            "invalid_demo_config",
            "Demo config must be explicitly simulation_only",
        )
    calibration = _object(
        "calibration",
        root["calibration"],
        {
            "calibration_id",
            "surface",
            "left_motor_sign",
            "right_motor_sign",
            "encoder_mdeg_per_mm",
            "encoder_mdeg_per_body_degree",
            "max_wheel_speed_dps",
            "max_pulse_ms",
        },
    )
    world = _object(
        "world",
        root["world"],
        {
            "width_mm",
            "height_mm",
            "robot_radius_mm",
            "range_max_mm",
            "near_threshold_mm",
            "start",
            "goal",
            "obstacle",
        },
    )
    start = _object(
        "start",
        world["start"],
        {"x_mm", "y_mm", "heading_mdeg"},
    )
    goal_value = _object(
        "goal",
        world["goal"],
        {
            "goal_id",
            "goal_epoch",
            "plan_revision",
            "target_x_mm",
            "target_y_mm",
            "tolerance_mm",
        },
    )
    obstacle = _object(
        "obstacle",
        world["obstacle"],
        {"obstacle_id", "x_mm", "y_mm", "radius_mm"},
    )
    return {
        "profile": DriveCalibrationProfile(
            calibration_id=calibration["calibration_id"],
            status="simulation_only",
            surface=calibration["surface"],
            left_motor_sign=calibration["left_motor_sign"],
            right_motor_sign=calibration["right_motor_sign"],
            encoder_mdeg_per_mm=calibration[
                "encoder_mdeg_per_mm"
            ],
            encoder_mdeg_per_body_degree=calibration[
                "encoder_mdeg_per_body_degree"
            ],
            max_wheel_speed_dps=calibration[
                "max_wheel_speed_dps"
            ],
            max_pulse_ms=calibration["max_pulse_ms"],
        ),
        "world": SimulationWorld(
            width_mm=world["width_mm"],
            height_mm=world["height_mm"],
            obstacles=(
                CircleObstacle(
                    obstacle_id=obstacle["obstacle_id"],
                    x_mm=obstacle["x_mm"],
                    y_mm=obstacle["y_mm"],
                    radius_mm=obstacle["radius_mm"],
                ),
            ),
        ),
        "settings": SimulationSettings(
            robot_radius_mm=world["robot_radius_mm"],
            range_max_mm=world["range_max_mm"],
            near_threshold_mm=world["near_threshold_mm"],
        ),
        "start": PoseEstimate(
            x_mm=start["x_mm"],
            y_mm=start["y_mm"],
            heading_mdeg=start["heading_mdeg"],
        ),
        "goal": WaypointGoal(
            goal_id=goal_value["goal_id"],
            goal_epoch=goal_value["goal_epoch"],
            plan_revision=goal_value["plan_revision"],
            target_x_mm=goal_value["target_x_mm"],
            target_y_mm=goal_value["target_y_mm"],
            tolerance_mm=goal_value["tolerance_mm"],
        ),
    }


def run_demo(
    config_path: Path = DEFAULT_CONFIG_PATH,
    with_obstacle: bool = True,
):
    config = load_demo_config(config_path)
    world = config["world"]
    if not with_obstacle:
        world = SimulationWorld(
            width_mm=world.width_mm,
            height_mm=world.height_mm,
            obstacles=(),
        )
    motion_authority = MotionAuthority()
    plant = DifferentialDriveSimulator(
        world,
        config["profile"],
        config["start"],
        motion_authority,
        settings=config["settings"],
    )
    ids = itertools.count(1)
    supervisor = MotionSupervisor(
        plant.profile,
        plant.clock_ms,
        plant.robot_id,
        plant.controller_instance_id,
        motion_authority,
        policy=MotionPolicy(
            max_pulse_ms=plant.profile.max_pulse_ms,
        ),
        id_factory=lambda: "demo-decision-{}".format(next(ids)),
    )
    inbox = ProposalInbox(
        (
            ProposalSourcePolicy(
                "goal-seeking",
                authority_rank=10,
                priority=50,
                ttl_ms=200,
            ),
            ProposalSourcePolicy(
                "obstacle-avoidance",
                authority_rank=20,
                priority=100,
                ttl_ms=120,
            ),
        ),
        plant.clock_ms,
    )
    episode = NavigationEpisode(
        plant,
        supervisor,
        inbox,
        (
            GoalSeekingBehavior(),
            ObstacleAvoidanceBehavior(),
        ),
        limits=NavigationLimits(
            max_ticks=500,
            max_elapsed_ms=60_000,
            max_proposals=1_000,
            max_replans=500,
            max_actions=480,
            max_total_motion_ms=55_000,
            max_no_progress_ticks=120,
        ),
    )
    return episode.run(config["goal"])


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the simulator-only navigation experiment."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
    )
    parser.add_argument(
        "--scenario",
        choices=("obstacle", "clear"),
        default="obstacle",
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="Include the complete per-tick trace.",
    )
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args(argv)
    result = run_demo(
        config_path=args.config,
        with_obstacle=args.scenario == "obstacle",
    )
    payload = result.to_dict()
    if not args.full:
        payload = {
            "goal_id": result.goal_id,
            "completed": result.completed,
            "termination": result.termination,
            "ticks": result.ticks,
            "proposals": result.proposals,
            "actions": result.actions,
            "total_motion_ms": result.total_motion_ms,
            "terminal_stop_verified": (
                result.terminal_stop_verified
            ),
            "final_pose": payload["final_pose"],
            "active_faults": list(
                result.final_snapshot.active_faults
            ),
        }
    print(
        json.dumps(
            payload,
            indent=None if args.compact else 2,
            sort_keys=True,
        )
    )
    return 0 if result.completed else 1


if __name__ == "__main__":
    raise SystemExit(main())
