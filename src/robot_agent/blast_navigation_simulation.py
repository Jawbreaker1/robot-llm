"""Run BLAST's production Gemma episode against route-free 2D worlds."""

from __future__ import annotations

import argparse
import json
import math
import threading
from typing import Callable

from . import lm_studio as _lm
from .blast_episode_adapter import BlastEpisodeRuntimeAdapter
from .lm_studio_controller_action import LMStudioControllerActionPlanner
from .lm_studio_controller_action import REASONING_EFFORTS
from .navigation_simulation_scenarios import (
    NavigationSimulationScenario,
    blast_gemma_validation_scenarios,
)
from .robot_control_contract import RobotControlSettings, RobotEpisodeStart
from .robot_control_service import RobotEpisodeContext
from .simulation_robot_adapters import SharedWorldBlastController


DEFAULT_MODEL = "qwen/qwen3.8-27b"
DEFAULT_GOAL = (
    "Navigate to the goal 800 mm straight ahead of the starting position. "
    "Use the map and replan the route if an obstacle blocks the direct path."
)


def _provider_diagnostics(raw: bytes | None) -> dict[str, object]:
    if not isinstance(raw, bytes):
        return {}
    try:
        envelope = json.loads(raw.decode("utf-8"))
        choice = envelope["choices"][0]
        message = choice["message"]
    except (IndexError, KeyError, TypeError, UnicodeDecodeError, ValueError):
        return {}
    result = {"provider_finish_reason": choice.get("finish_reason")}
    reasoning = message.get("reasoning_content")
    if isinstance(reasoning, str) and reasoning.strip():
        result["provider_reasoning_tail"] = reasoning.strip()[-900:]
    return result


class _RecordingPlanner:
    def __init__(self, planner, records: list[dict[str, object]]) -> None:
        self._planner = planner
        self._records = records

    def decide(self, context):
        context_chars = len(json.dumps(
            context.to_dict(), ensure_ascii=False, separators=(",", ":"),
        ))
        result = self._planner.decide(context)
        decision = getattr(result, "decision", result)
        evidence = context.local_map_evidence or {}
        local_map_chars = len(json.dumps(
            evidence, ensure_ascii=False, separators=(",", ":"),
        ))
        coarse_grid = evidence.get("coarse_grid") or {}
        self._records.append({
            "action": getattr(decision, "action", None),
            "context_chars": context_chars,
            "local_map_chars": local_map_chars,
            "local_map_keys": sorted(evidence),
            "obstacle_region_count": len(
                evidence.get("obstacle_regions") or ()
            ),
            "assessment": getattr(decision, "assessment", None),
            "model_reasoning": getattr(
                result, "reasoning_content", None
            ),
            "waypoint": getattr(decision, "waypoint", None),
            "following_waypoints": list(
                getattr(decision, "following_waypoints", ())
            ),
            "available_actions": list(context.available_actions),
            "robot_pose": evidence.get("robot_pose"),
            "direct_goal_blockage": evidence.get(
                "direct_goal_blockage"
            ),
            "latest_route_rejection": evidence.get(
                "latest_route_rejection"
            ),
            "coarse_grid_rows": coarse_grid.get("rows"),
        })
        return result


def run_blast_gemma_scenario(
    scenario: NavigationSimulationScenario,
    *,
    model: str = DEFAULT_MODEL,
    max_decisions: int = 32,
    reasoning_effort: str = "none",
    max_output_tokens: int = 512,
    timeout_seconds: float = 20.0,
    planner_factory: Callable[[str], object] | None = None,
) -> dict[str, object]:
    """Run the real BLAST episode loop; only hardware is simulated."""

    if (
        not isinstance(scenario, NavigationSimulationScenario)
        or {robot.robot_id for robot in scenario.robots} != {"blast"}
        or {goal.robot_id for goal in scenario.goals} != {"blast"}
    ):
        raise ValueError("BLAST Gemma scenario must contain only BLAST")
    last_raw_response: list[bytes | None] = [None]
    if planner_factory is None:
        def recording_transport(*args):
            raw = _lm._stdlib_post(*args)
            last_raw_response[0] = raw
            return raw

        planner_factory = lambda selected_model: (
            LMStudioControllerActionPlanner(
                model=selected_model,
                timeout_seconds=timeout_seconds,
                reasoning_effort=reasoning_effort,
                max_output_tokens=max_output_tokens,
                transport=recording_transport,
            )
        )

    world = scenario.build()
    controller = SharedWorldBlastController(
        world,
        world_robot_id="blast",
    )
    updates: list[dict[str, object]] = []
    planner_records: list[dict[str, object]] = []
    context = RobotEpisodeContext(
        episode_id="simulation-{}".format(scenario.scenario_id),
        request=RobotEpisodeStart(
            goal=DEFAULT_GOAL,
            locale="en",
            client_request_id="simulation-{}".format(
                scenario.scenario_id
            ),
            expected_revision=1,
        ),
        settings=RobotControlSettings(
            model=model,
            max_episode_ms=10 * 60 * 1_000,
            speech_enabled=False,
        ),
        stop_requested=threading.Event(),
        emergency_stop_requested=threading.Event(),
        publish=lambda update: updates.append(dict(update)),
    )
    adapter = BlastEpisodeRuntimeAdapter(
        controller=controller,
        planner_factory=lambda selected_model: _RecordingPlanner(
            planner_factory(selected_model),
            planner_records,
        ),
        max_decisions=max_decisions,
        minimum_forward_progress_mm=800,
        monotonic_ms=controller.monotonic_ms,
    )
    outcome = None
    error = None
    map_snapshot = {}
    try:
        outcome = adapter.run(context)
    except Exception as caught:
        error = caught
    finally:
        map_snapshot = adapter.spatial_map_provider.snapshot()
        adapter.spatial_map_provider.close()

    pose = world.pose("blast")
    goal = world.goals["blast"]
    route_rejections = sum(
        update.get("message")
        == "Gemma route crosses a known coarse keep-out cell; replanning"
        for update in updates
    )
    trace = []
    previous = None
    for update in updates:
        current = {
            key: update.get(key)
            for key in ("current_action", "message", "plan", "obstacle")
            if key in update
        }
        if current and current != previous:
            trace.append(current)
            previous = current
    error_code = getattr(error, "code", None) if error is not None else None
    obstacle_regions = [
        {
            key: hypothesis.get(key)
            for key in (
                "x_mm",
                "y_mm",
                "support_radius_mm",
                "evidence_count",
            )
        } | ({
            "support_bounds_mm": {
                "min_x": min(
                    point["x_mm"] for point in hypothesis["support_points"]
                ),
                "max_x": max(
                    point["x_mm"] for point in hypothesis["support_points"]
                ),
                "min_y": min(
                    point["y_mm"] for point in hypothesis["support_points"]
                ),
                "max_y": max(
                    point["y_mm"] for point in hypothesis["support_points"]
                ),
            },
        } if hypothesis.get("support_points") else {})
        for hypothesis in map_snapshot.get("object_hypotheses", [])
    ]
    return {
        "scenario_id": scenario.scenario_id,
        "completed": outcome.completed if outcome is not None else False,
        "terminal_reason": (
            outcome.terminal_reason
            if outcome is not None else "simulation_runtime_error"
        ),
        "error_code": error_code,
        "error_type": type(error).__name__ if error is not None else None,
        "error_message": str(error)[:240] if error is not None else None,
        "goal_reached": world.goal_reached("blast"),
        "final_pose": {
            "x_mm": pose.x_mm,
            "y_mm": pose.y_mm,
            "heading_mdeg": pose.heading_mdeg,
        },
        "distance_to_goal_mm": round(math.hypot(
            pose.x_mm - goal.x_mm,
            pose.y_mm - goal.y_mm,
        )),
        "route_rejections": route_rejections,
        "commands": len(controller.commands),
        "scans": controller.scan_count,
        "blocked_moves": sum(
            event.kind == "blocked" for event in world.events
        ),
        "world_event_count": len(world.events),
        "map_quality": map_snapshot.get("map_quality"),
        "obstacle_regions": obstacle_regions,
        "commands_tail": controller.commands[-24:],
        "planner_decisions": planner_records,
        "trace_tail": trace[-24:],
    } | _provider_diagnostics(last_raw_response[0])


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run model-directed BLAST navigation in route-free simulation",
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--max-decisions", type=int, default=32)
    parser.add_argument(
        "--reasoning-effort",
        choices=REASONING_EFFORTS,
        default="none",
    )
    parser.add_argument("--max-output-tokens", type=int, default=512)
    parser.add_argument("--timeout-seconds", type=float, default=20.0)
    parser.add_argument(
        "--compact",
        action="store_true",
        help="Print decisions without the repeated coarse-grid rows",
    )
    parser.add_argument(
        "--scenario",
        action="append",
        help="Scenario id; omit to run the complete small suite",
    )
    return parser


def _compact_cli_result(result: dict[str, object]) -> dict[str, object]:
    compact = dict(result)
    decisions = []
    for decision in result.get("planner_decisions", []):
        item = {
            key: value
            for key, value in decision.items()
            if key in (
                "action",
                "assessment",
                "context_chars",
                "local_map_chars",
                "local_map_keys",
                "obstacle_region_count",
                "robot_pose",
                "waypoint",
                "following_waypoints",
                "latest_route_rejection",
                "model_reasoning",
            )
        }
        reasoning = item.get("model_reasoning")
        if isinstance(reasoning, str) and len(reasoning) > 900:
            item["model_reasoning"] = reasoning[:900] + "…"
        if decisions and all(
            decisions[-1].get(key) == value
            for key, value in item.items()
        ):
            decisions[-1]["repeat_count"] += 1
        else:
            item["repeat_count"] = 1
            decisions.append(item)
    compact["planner_decisions"] = decisions
    compact.pop("trace_tail", None)
    return compact


def main() -> None:
    args = _parser().parse_args()
    scenarios = blast_gemma_validation_scenarios()
    selected_ids = set(args.scenario or ())
    if selected_ids:
        unknown = selected_ids - {
            scenario.scenario_id for scenario in scenarios
        }
        if unknown:
            raise SystemExit(
                "unknown scenario: {}".format(", ".join(sorted(unknown)))
            )
        scenarios = tuple(
            scenario for scenario in scenarios
            if scenario.scenario_id in selected_ids
        )
    for scenario in scenarios:
        result = run_blast_gemma_scenario(
            scenario,
            model=args.model,
            max_decisions=args.max_decisions,
            reasoning_effort=args.reasoning_effort,
            max_output_tokens=args.max_output_tokens,
            timeout_seconds=args.timeout_seconds,
        )
        if args.compact:
            result = _compact_cli_result(result)
        print(json.dumps(result, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
