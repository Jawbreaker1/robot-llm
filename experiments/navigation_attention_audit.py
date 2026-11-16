"""Offline causal probes; no fitting, labels, gradients or simulation truth."""
import argparse
import copy
import json
from pathlib import Path
import statistics
import time

import numpy as np

from experiments.navigation_attention_policy import AttentionPolicy
from experiments.local_navigation import ACTIONS


def fixture(robot):
    from experiments.navigation_weight_space import author_request
    info = json.loads(author_request()["messages"][1]["content"])["robots"]["EV3" if robot == "ev3" else "Blast"]
    waypoint = {"x_mm": 650, "y_mm": 0}
    packet = {"available_actions": list(ACTIONS[:-1]), "active_waypoint": waypoint,
        "active_waypoint_plan": [waypoint], "waypoint_reached_radius_mm": 50,
        "observation": {"directional_goal": {"x_mm": 1400, "y_mm": 0}},
        "execution": {"advance_step_mm": info["advance_step_mm"], "reverse_step_mm": info["advance_step_mm"],
                      "forward_admission": {"permitted": True}},
        "local_map_evidence": {"robot_pose": {"x_mm": 0, "y_mm": 0, "heading_deg": 0},
            "robot_footprint": info["body"], "route_execution_memory": {"event_count": 0, "records": []}}}
    if robot == "ev3":
        packet["observation"]["infrared"] = {"filtered": 100}
        packet["local_map_evidence"]["ir_survey"] = {"complete": True, "sweep_degrees": 360,
            "samples": [{"filtered_ir_proximity": 100, "ray_heading_deg_in_episode": angle, "settled": True,
                         "observed_from": {"x_mm": 0, "y_mm": 0}} for angle in (-60, -30, 0, 30, 60)]}
    else:
        packet["observation"]["sensors"] = {"distance_mm": 1800, "range_state": "MEASURED"}
        packet["local_map_evidence"]["range_observations"] = [
            {"distance_mm": 1800, "range_state": "MEASURED", "ray_heading_deg_in_episode": angle,
             "settled": True, "observed_from": {"x_mm": 110, "y_mm": 80}} for angle in (-60,-30,0,30,60)]
    return packet


def probes(robot):
    p = fixture(robot)
    values = {"clear": p}
    for direction, y in (("left", 650), ("right", -650)):
        q = copy.deepcopy(p)
        q["active_waypoint"] = {"x_mm": 0, "y_mm": y}
        q["active_waypoint_plan"] = [q["active_waypoint"]]
        values["waypoint_" + direction] = q
    q = copy.deepcopy(p)
    if robot == "ev3":
        q["local_map_evidence"]["ir_survey"]["samples"][2]["filtered_ir_proximity"] = 20
        q["observation"]["infrared"]["filtered"] = 20
    else:
        q["local_map_evidence"]["range_observations"][2]["distance_mm"] = 40
        q["observation"]["sensors"]["distance_mm"] = 40
    values["near_return"] = q
    for kind, displacement in (("same_place_failure", 0), ("elsewhere_failure", 2000)):
        q = copy.deepcopy(p)
        q["local_map_evidence"]["route_execution_memory"] = {"event_count": 14, "records": [{
            "waypoint_plan": [{"x_mm": 650+displacement, "y_mm": 0}],
            "pose_before": {"x_mm": displacement, "y_mm": 0},
            "pose_after": {"x_mm": displacement, "y_mm": 0},
            "completed": False, "made_progress": False, "reason": "near_obstacle",
            "action": "ADVANCE", "primitive": "ADVANCE", "count": 13, "last_seen": 13,
            "measured_translation_mm": 0, "route_interruption": {"reason": "near_obstacle"}}]}
        values[kind] = q
    return values


def audit(author, previous, output):
    report = {"training": False, "robot_results": {}}
    for robot in ("ev3", "blast"):
        model = AttentionPolicy.load(author / (robot + ".npz"))
        examples = {name: {"action": model.predict(p)[0], "scores": model.scores(p)}
                    for name, p in probes(robot).items()}
        packets = []
        with (previous / "validation" / robot / "events.jsonl").open() as stream:
            for line in stream:
                event = json.loads(line)
                if event.get("event") == "decision_pair" and event["packet"].get("active_waypoint"):
                    packets.append(event["packet"])
        variants = {k: {"score_changes": 0, "action_changes": 0} for k in ("without_rays", "without_memory", "flipped_goal")}
        timings = []
        for p in packets:
            started = time.perf_counter()
            scores, action = model.scores(p), model.predict(p)[0]
            timings.append((time.perf_counter()-started)*500)  # two forward passes
            for kind, totals in variants.items():
                q = copy.deepcopy(p)
                e = q["local_map_evidence"]
                if kind == "without_rays":
                    if "ir_survey" in e:
                        e["ir_survey"]["samples"] = []
                    else:
                        e["range_observations"] = []
                elif kind == "without_memory":
                    e["route_execution_memory"] = {"records": [], "event_count": 0}
                else:
                    pose = e.get("robot_pose", {})
                    for origin in (q.get("observation", {}), e):
                        goal = origin.get("directional_goal")
                        if goal:
                            for axis in ("x", "y"):
                                key = axis+"_mm" if axis+"_mm" in goal else "target_"+axis+"_mm"
                                goal[key] = 2*pose.get(axis+"_mm", 0)-goal.get(key, 0)
                totals["score_changes"] += not np.allclose(scores, model.scores(q), atol=1e-5, rtol=1e-5)
                totals["action_changes"] += action != model.predict(q)[0]
        blocks = {str(i): sum(int(np.count_nonzero(v)) for k,v in model.weights.items() if k.startswith(f"b{i}_"))
                  for i in range(3)}
        report["robot_results"][robot] = {"probes": examples, "recorded_packets": len(packets),
            "ablations": variants, "median_forward_ms": statistics.median(timings), "block_nonzero": blocks}
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("author", "previous", "output"):
        parser.add_argument("--"+key, type=Path, required=True)
    args = parser.parse_args()
    audit(args.author, args.previous, args.output)
