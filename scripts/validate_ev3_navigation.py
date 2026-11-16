"""Bounded real-model EV3 route check; stdout is a JSONL diagnostic artifact.

Run with PYTHONPATH=.:src .venv/bin/python scripts/validate_ev3_navigation.py SCENE.
World geometry stays inside the sensor/physics adapter, never in Qwen context.
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import threading

from robot_agent.ev3_navigation_body import EV3NavigationBody
from robot_agent.ev3_episode_adapter import EV3EpisodeAdapter
from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge
from robot_agent.robot_control_contract import RobotControlSettings, RobotEpisodeStart
from robot_agent.robot_control_service import RobotEpisodeContext
from robot_agent.ev3_navigation_transport import EV3NavigationSSHTransport
from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
from robot_agent.lm_studio_controller_action import LMStudioControllerActionPlanner, REASONING_EFFORTS, navigation_reasoning_effort
from robot_agent.navigation_simulation_scenarios import ev3_validation_scenarios
from robot_agent.navigation_diagnostics import enable_navigation_diagnostics, record_navigation_diagnostic
from robot_agent.simulation_robot_adapters import SharedWorldEV3Transport


class ValidatedTransport(SharedWorldEV3Transport):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.validator = EV3NavigationSSHTransport(target="unused.local", controller_id="ev3", remote_worker_path="/unused.py")

    def request(self, operation, arguments, timeout, cancel_requested=None):
        response = super().request(operation, arguments, timeout, cancel_requested)
        self.validator._validate_success_result(operation, arguments, response)
        return response


def main():
    scenarios = {scene.scenario_id: scene for scene in ev3_validation_scenarios()}
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", choices=tuple(scenarios))
    parser.add_argument("--model", default="qwen/qwen3.8-27b", help="LM Studio model or codex/gpt-6-astra")
    parser.add_argument("--ir-surface", choices=("white", "dark"), default="white")
    parser.add_argument("--decisions", type=int, default=12)
    parser.add_argument("--output-tokens", type=int, default=4096)
    parser.add_argument("--reasoning-effort", choices=REASONING_EFFORTS, default=None,
                        help="Override navigation reasoning (default: Astra high, Qwen low)")
    parser.add_argument("--planner-timeout", type=float, default=60)
    parser.add_argument("--time-budget", type=float, default=300)
    parser.add_argument("--diagnostics", type=Path, default=Path("local-artifacts/navigation-ev3-simulation.jsonl"),
                        help="Full planner requests/responses, separate from the simulation event log")
    args = parser.parse_args()
    args.reasoning_effort = navigation_reasoning_effort(args.model, args.reasoning_effort)
    enable_navigation_diagnostics(args.diagnostics)
    record_navigation_diagnostic("runtime_started", source="ev3_simulation", scene=args.scene,
                                 model=args.model, reasoning_effort=args.reasoning_effort, max_output_tokens=args.output_tokens)
    profile = EV3RSTORMProfile()
    scenario = scenarios[args.scene]
    start, target = scenario.robots[0].pose, scenario.goals[0]
    goal_distance_mm = target.x_mm - start.x_mm
    # Measured anchors, with deliberately explicit assumptions between/beyond
    # them. These curves expose missed detections, not certify room perception.
    ir_response = (((0, 0), (200, 44), (300, 51), (1_000, 100)) if args.ir_surface == "white"
                   else ((0, 20), (20, 30), (200, 71), (1_000, 100)))
    world = scenario.build()
    workers = []
    bodies = []
    def transport_factory():
        transport = ValidatedTransport(world, world_robot_id="ev3", odometry=profile.odometry_calibration,
                                       ir_response=ir_response)
        workers.append(transport)
        return transport
    def body_factory(**options):
        body = EV3NavigationBody(transport_factory, calibration=profile.odometry_calibration, **options)
        bodies.append(body)
        return body
    decisions = completed_steps = 0
    stop = threading.Event()
    def publish(event):
        nonlocal decisions, completed_steps
        if event["event"] == "decision":
            decisions += 1
        if event["event"] == "result":
            completed_steps += 1
            if completed_steps >= args.decisions and event["action"] != "COMPLETE":
                stop.set()
        print(json.dumps({"scene": args.scene, **event}), flush=True)
    bridge = PhysicalSpatialMapBridge(robot_id=profile.descriptor.robot_id, controller_instance_id=profile.descriptor.controller_id)
    adapter = EV3EpisodeAdapter(
        body_factory=body_factory, planner_factory=lambda model: LMStudioControllerActionPlanner(model=model, reasoning_effort=args.reasoning_effort,
                                                          max_output_tokens=args.output_tokens, timeout_seconds=args.planner_timeout),
        descriptor=profile.descriptor, footprint=profile.hazard_calibration.robot_footprint,
        minimum_forward_progress_mm=goal_distance_mm,
        spatial_map_provider=bridge, speech_runtime_factory=None, controller_lease=threading.Lock(), event_sink=publish,
    )
    context = RobotEpisodeContext("ev3-qwen-simulation",
        RobotEpisodeStart(f"Reach the goal {goal_distance_mm / 10:g} cm straight ahead of your starting position.", "en", "simulation", 1),
        RobotControlSettings(model=args.model, max_episode_ms=round(args.time_budget * 1_000), speech_enabled=False),
        stop, threading.Event(), lambda update: publish({"event": "ui_update", "update": update, "map": bridge.snapshot()}))
    failure = None
    outcome = None
    publish({"event": "configuration", "model": args.model, "reasoning_effort": args.reasoning_effort, "output_tokens": args.output_tokens,
             "world": {"bounds": scenario.bounds, "obstacles": [asdict(item) for item in scenario.obstacles],
                       "start": asdict(start), "goal": asdict(target)},
             "ir_surface": args.ir_surface, "ir_response": ir_response,
             "sensor_assumptions": "Piecewise-linear response; zero and 1000 mm endpoints assumed; dark 20 mm reading from contact run is approximate; sensor origin at drive origin; ideal ray and encoder motion, contact visible to physics. Not full hardware fidelity.",
             "planner_timeout_seconds": args.planner_timeout, "time_budget_seconds": args.time_budget})
    try:
        outcome = adapter.run(context)
    except Exception as error:
        failure = repr(error)
    body = bodies[0]
    completed = outcome is not None and outcome.completed
    world_goal_reached = world.goal_reached("ev3")
    goal = (bridge.snapshot().get("navigation_trace") or {}).get("final_goal", {})
    blocked = [asdict(event) for event in world.events if "blocked" in event.kind]
    publish({"event": "summary", "completed": completed, "failure": failure,
             "clean_route": completed and world_goal_reached and not blocked,
             "world_goal_reached": world_goal_reached,
             "stop_reason": "error" if failure else outcome.terminal_reason,
             "decisions": decisions, "completed_steps": completed_steps,
             "worker_sessions": len(workers), "workers_stopped": all(worker.shutdown_complete for worker in workers),
             "distance_to_goal_mm": goal.get("distance_to_goal_mm"),
             "estimated_pose": body.pose.to_dict(), "world_pose": asdict(world.pose("ev3")),
             "physics_events": blocked})
    return 0 if completed and world_goal_reached and not blocked and failure is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
