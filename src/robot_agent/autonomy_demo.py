"""CLI for bounded self-directed exploration in the 2D simulator."""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
from typing import Optional, Sequence

from .autonomy_authority import GoalLeaseCoordinator
from .autonomy_contract import (
    InterestSelectionContext,
    InterestSelectionProposal,
)
from .autonomy_runtime import (
    IdleExplorationService,
    IdleSessionLimits,
    IdleTaskResult,
)
from .lm_studio import DEFAULT_BASE_URL, DEFAULT_MODEL, LMStudioError
from .lm_studio_autonomy import LMStudioInterestSelector
from .navigation_contract import MotionAuthority, WaypointGoal
from .navigation_demo import DEFAULT_CONFIG_PATH, load_demo_config
from .navigation_simulator import (
    CircleObstacle,
    DifferentialDriveSimulator,
    SimulationWorld,
)
from .navigation_state import ProposalInbox, ProposalSourcePolicy
from .navigation_supervisor import MotionPolicy, MotionSupervisor


class DeterministicInterestSelector:
    """Strict test oracle; not the semantic classifier used by the agent."""

    def __call__(self, context: InterestSelectionContext) -> bytes:
        if not isinstance(context, InterestSelectionContext):
            raise TypeError("fixture selector requires typed context")
        # The fixture makes repeatable tests possible.  Real interest
        # classification is delegated to LMStudioInterestSelector.
        selected = min(
            context.candidates,
            key=lambda candidate: (
                0 if candidate.linked_observation_ids else 1,
                candidate.attempted_visits,
                candidate.completed_visits,
                candidate.candidate_id,
            ),
        )
        proposal = InterestSelectionProposal(
            proposal_id=context.proposal_id,
            robot_id=context.robot_id,
            controller_instance_id=context.controller_instance_id,
            autonomy_session_id=context.autonomy_session_id,
            lease_generation=context.lease_generation,
            candidate_set_id=context.candidate_set_id,
            based_on_state_version=context.state_version,
            based_on_world_model_version=context.world_model_version,
            decision="SELECT",
            confidence_milli=1_000,
            selected_candidate_id=selected.candidate_id,
        )
        return json.dumps(
            proposal.to_dict(),
            separators=(",", ":"),
        ).encode("utf-8")


def _build_stack(config_path: Path, with_obstacle: bool):
    config = load_demo_config(config_path)
    world = config["world"]
    if not with_obstacle:
        world = SimulationWorld(
            width_mm=world.width_mm,
            height_mm=world.height_mm,
        )
    motion_authority = MotionAuthority()
    plant = DifferentialDriveSimulator(
        world,
        config["profile"],
        config["start"],
        motion_authority,
        settings=config["settings"],
    )
    decision_ids = itertools.count(1)
    supervisor = MotionSupervisor(
        plant.profile,
        plant.clock_ms,
        plant.robot_id,
        plant.controller_instance_id,
        motion_authority,
        policy=MotionPolicy(
            max_pulse_ms=plant.profile.max_pulse_ms,
        ),
        id_factory=lambda: "idle-demo-decision-{}".format(
            next(decision_ids)
        ),
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
    authority = GoalLeaseCoordinator(
        plant.robot_id,
        plant.controller_instance_id,
        starting_goal_epoch=100,
        starting_plan_revision=100,
        idle_enabled=True,
    )
    return plant, supervisor, inbox, authority


def run_demo(
    selector=None,
    tasks: int = 3,
    config_path: Path = DEFAULT_CONFIG_PATH,
    with_obstacle: bool = True,
):
    if selector is None:
        selector = DeterministicInterestSelector()
    plant, supervisor, inbox, authority = _build_stack(
        config_path,
        with_obstacle,
    )
    service = IdleExplorationService(
        plant,
        supervisor,
        inbox,
        authority,
        selector,
        session_id="idle-autonomy-demo",
    )
    result = service.run_session(IdleSessionLimits(
        max_tasks=tasks,
        max_planner_calls=max(tasks * 2, 2),
        max_stale_replans=max(tasks, 1),
        max_elapsed_ms=90_000,
        max_actions=320,
        max_total_motion_ms=40_000,
    ))
    return result, plant


def run_range_change_demo(
    selector=None,
    config_path: Path = DEFAULT_CONFIG_PATH,
):
    """Run one task, move a synthetic object, then plan again."""

    if selector is None:
        selector = DeterministicInterestSelector()
    plant, supervisor, inbox, authority = _build_stack(
        config_path,
        with_obstacle=True,
    )
    probe = WaypointGoal(
        goal_id="range-change-setup",
        goal_epoch=1,
        plan_revision=1,
        target_x_mm=0,
        target_y_mm=0,
        tolerance_mm=1,
    )
    initial = plant.observe(probe)
    side_blockers = []
    for obstacle_id, offset_mdeg in (
        ("range-demo-left-block", 60_000),
        ("range-demo-right-block", -60_000),
    ):
        angle = math.radians(
            (
                initial.pose.heading_mdeg + offset_mdeg
            )
            / 1_000.0
        )
        side_blockers.append(CircleObstacle(
            obstacle_id=obstacle_id,
            x_mm=int(round(initial.pose.x_mm + math.cos(angle) * 120)),
            y_mm=int(round(initial.pose.y_mm + math.sin(angle) * 120)),
            radius_mm=30,
        ))
    plant.update_world(SimulationWorld(
        width_mm=plant.world.width_mm,
        height_mm=plant.world.height_mm,
        obstacles=plant.world.obstacles + tuple(side_blockers),
    ))
    service = IdleExplorationService(
        plant,
        supervisor,
        inbox,
        authority,
        selector,
        session_id="idle-range-change-demo",
    )
    limits = IdleSessionLimits(
        max_tasks=1,
        max_planner_calls=3,
        max_stale_replans=1,
        max_elapsed_ms=30_000,
        max_actions=200,
        max_total_motion_ms=20_000,
    )
    first = service.run_once(limits)
    if not first.completed or not plant.world.obstacles:
        return (first,), plant

    obstacle = plant.world.obstacles[0]
    shifted_x = obstacle.x_mm + 150
    if shifted_x + obstacle.radius_mm > plant.world.width_mm:
        shifted_x = obstacle.x_mm - 150
    moved = CircleObstacle(
        obstacle_id=obstacle.obstacle_id,
        x_mm=shifted_x,
        y_mm=obstacle.y_mm,
        radius_mm=obstacle.radius_mm,
    )
    plant.update_world(SimulationWorld(
        width_mm=plant.world.width_mm,
        height_mm=plant.world.height_mm,
        obstacles=(moved,) + tuple(
            value
            for value in plant.world.obstacles[1:]
            if value.obstacle_id
            not in {
                "range-demo-left-block",
                "range-demo-right-block",
            }
        ),
    ))
    second = service.run_once(limits)
    return (first, second), plant


def _task_report(task: IdleTaskResult):
    selected = next(
        (
            candidate
            for candidate in task.candidates
            if candidate.candidate_id == task.selected_candidate_id
        ),
        None,
    )
    observation = task.observation
    return {
        "termination": task.termination,
        "completed": task.completed,
        "planner_calls": task.planner_calls,
        "stale_replans": task.stale_replans,
        "selected_candidate_id": task.selected_candidate_id,
        "selected_task_kind": (
            None if selected is None else selected.task_kind
        ),
        "selected_direction": (
            None if selected is None else selected.relative_direction
        ),
        "observation": (
            None
            if observation is None
            else {
                "kind": observation.kind,
                "previous_value": observation.previous_value,
                "current_value": observation.current_value,
                "unit": observation.unit,
                "previous_subject_id": (
                    observation.previous_subject_id
                ),
                "current_subject_id": (
                    observation.current_subject_id
                ),
            }
        ),
        "actions": (
            0 if task.mission is None else task.mission.actions
        ),
        "terminal_stop_verified": task.terminal_stop_verified,
        "trace": list(task.trace),
    }


def _report(result, plant):
    payload = result.to_dict()
    payload["collision_count"] = plant.collision_count
    payload["drive_pulses"] = sum(
        1 for pulse in plant.applied_pulses if pulse.kind == "DRIVE"
    )
    payload["stop_pulses"] = sum(
        1 for pulse in plant.applied_pulses if pulse.kind == "STOP"
    )
    payload["final_pose"] = (
        None
        if result.final_snapshot is None
        else {
            "x_mm": result.final_snapshot.pose.x_mm,
            "y_mm": result.final_snapshot.pose.y_mm,
            "heading_mdeg": (
                result.final_snapshot.pose.heading_mdeg
            ),
        }
    )
    payload["selected_opportunities"] = [
        {
            "candidate_id": task.selected_candidate_id,
            "task_kind": (
                None
                if task.selected_candidate_id is None
                else next(
                    (
                        candidate.task_kind
                        for candidate in task.candidates
                        if candidate.candidate_id
                        == task.selected_candidate_id
                    ),
                    None,
                )
            ),
            "observation_kind": (
                None
                if task.observation is None
                else task.observation.kind
            ),
        }
        for task in result.tasks
    ]
    return payload


def _range_change_report(tasks, plant):
    return {
        "schema": "robot-idle-range-change-report/v1",
        "tasks": [_task_report(task) for task in tasks],
        "tasks_completed": sum(1 for task in tasks if task.completed),
        "collision_count": plant.collision_count,
        "drive_pulses": sum(
            1
            for pulse in plant.applied_pulses
            if pulse.kind == "DRIVE"
        ),
        "terminal_stop_verified": (
            bool(tasks)
            and tasks[-1].terminal_stop_verified
            and tasks[-1].final_snapshot is not None
            and not tasks[-1].final_snapshot.motors_running
        ),
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run bounded self-directed exploration in the simulator."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
    )
    parser.add_argument(
        "--scenario",
        choices=("obstacle", "clear", "range-change"),
        default="obstacle",
    )
    parser.add_argument("--tasks", type=int, default=3)
    parser.add_argument(
        "--lm-studio",
        action="store_true",
        help="Let the configured local model select each opportunity.",
    )
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--timeout-seconds", type=float, default=10.0)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args(argv)
    if not 1 <= args.tasks <= 20:
        parser.error("--tasks must be between 1 and 20")

    selector = DeterministicInterestSelector()
    if args.lm_studio:
        try:
            selector = LMStudioInterestSelector(
                base_url=args.base_url,
                model=args.model,
                timeout_seconds=args.timeout_seconds,
            )
        except (LMStudioError, ValueError) as error:
            parser.error(str(error))
    try:
        if args.scenario == "range-change":
            tasks, plant = run_range_change_demo(
                selector=selector,
                config_path=args.config,
            )
            payload = _range_change_report(tasks, plant)
            print(json.dumps(
                payload,
                indent=None if args.compact else 2,
                sort_keys=True,
            ))
            return (
                0
                if (
                    len(tasks) == 2
                    and all(task.completed for task in tasks)
                    and payload["terminal_stop_verified"]
                    and plant.collision_count == 0
                )
                else 1
            )
        result, plant = run_demo(
            selector=selector,
            tasks=args.tasks,
            config_path=args.config,
            with_obstacle=args.scenario == "obstacle",
        )
    except LMStudioError as error:
        print(
            json.dumps({
                "error": type(error).__name__,
                "message": str(error),
            })
        )
        return 2
    payload = _report(result, plant)
    print(json.dumps(
        payload,
        indent=None if args.compact else 2,
        sort_keys=True,
    ))
    return (
        0
        if (
            len(result.tasks) == args.tasks
            and result.tasks_completed == args.tasks
            and all(task.completed for task in result.tasks)
            and result.terminal_stop_verified
            and plant.collision_count == 0
        )
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
