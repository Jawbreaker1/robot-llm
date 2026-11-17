"""Frozen paired simulation, old small networks versus new attention networks."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys

from robot_agent.codex_subscription import CODEX_MODEL
from experiments.navigation_specialization import source_hashes, digest
from scripts.local_navigation_experiment import TEST, ATTENTION_HELD_OUT

REPO = Path(__file__).resolve().parents[1]


def report(root):
    manifest = json.loads((root / "manifest.json").read_text())
    rows, worlds = [], {}
    for job in manifest["jobs"]:
        path = root / job["name"] / "summary.json"
        if not path.exists():
            rows.append(dict(job, status="pending"))
            continue
        result = json.loads(path.read_text())
        with path.with_name("events.jsonl").open() as stream:
            config = json.loads(next(stream))
            dispatches = [json.loads(line) for line in stream if '"event": "policy_dispatch"' in line]
        world_hash = hashlib.sha256(json.dumps(config["world"], sort_keys=True).encode()).hexdigest()
        key = (job["robot"], job["scene"])
        if worlds.setdefault(key, world_hash) != world_hash:
            raise ValueError("Paired simulations differ in hidden world")
        key = job["robot"] + "-" + job["variant"]
        if config["model_sha256"] != manifest["weight_hashes"][key]:
            raise ValueError("Trial did not use frozen model")
        scores = result.get("local_decision_ms", [])
        rows.append(dict(job, status="complete", **{k: result.get(k) for k in (
            "clean", "goal_reached", "blocked_moves", "distance_to_goal_mm", "decisions", "astra_calls",
            "local_calls", "wall_seconds", "terminal_reason", "error_type", "error_message")},
            local_median_ms=round(statistics.median(scores), 3) if scores else None,
            local_selections={action: sum(d["selection"] == action for d in dispatches) for action in
                sorted({d["selection"] for d in dispatches})}, world_sha256=world_hash))
    result = {"runs": rows, "physical_robots": False, "training": False,
        "limitations": ["Astra routes are stochastic, not controlled identical trajectories.",
            "Six previously evaluated regression layouts per robot; not independent holdouts or a reliability guarantee.",
            "Ideal encoders/range rays and approximate white-surface IR, not physical validation.",
            "Simulator collision stops count as failures even if goal is ultimately reached."]}
    (root / "comparison.json").write_text(json.dumps(result, indent=2))
    return result


def compare(args):
    paths = {} if args.new_only else {robot + "-old": args.previous / ("author-" + robot) / "policy-direct.json"
                                     for robot in ("ev3", "blast")}
    paths.update({robot + "-new": args.author / (robot + ".npz") for robot in ("ev3", "blast")})
    hashes = {k: digest(v) for k, v in paths.items()}
    sources = source_hashes()
    jobs = []
    for index, scene in enumerate(TEST + ATTENTION_HELD_OUT):
        for rindex, robot in enumerate(("ev3", "blast")):
            variants = ("new",) if args.new_only else (("new", "old") if (index+rindex) % 2 else ("old", "new"))
            for variant in variants:
                jobs.append({"robot": robot, "scene": scene, "variant": variant,
                    "held_out": False, "name": f"{robot}-{scene}-{variant}"})
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {"jobs": jobs, "weight_hashes": hashes, "source_hashes": sources,
        "weights": {k: str(v) for k,v in paths.items()}, "decisions": args.decisions,
        "model": CODEX_MODEL, "reasoning": "high", "workers": 2,
        "frozen_before_evaluation": True}
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2))

    def one(job):
        if source_hashes() != sources or any(digest(p) != hashes[k] for k,p in paths.items()):
            raise ValueError("Code or frozen weights changed during comparison")
        key = job["robot"] + "-" + job["variant"]
        command = [sys.executable, "scripts/local_navigation_experiment.py", "run", "--robot", job["robot"],
                   "--scene", job["scene"], "--weights", str(paths[key]), "--decisions", str(args.decisions),
                   "--output", str(args.output / job["name"])]
        environment = dict(os.environ, PYTHONPATH=".:src", PYTHONDONTWRITEBYTECODE="1",
                           OPENBLAS_NUM_THREADS="1", VECLIB_MAXIMUM_THREADS="1", OMP_NUM_THREADS="1")
        with (args.output / (job["name"] + ".log")).open("w") as stream:
            result = subprocess.run(command, cwd=REPO, env=environment, stdout=stream, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError("Trial failed: " + job["name"])
        result = json.loads((args.output / job["name"] / "summary.json").read_text())
        print(json.dumps(dict(job, **{k: result.get(k) for k in
            ("clean", "goal_reached", "blocked_moves", "decisions", "astra_calls", "wall_seconds")})), flush=True)

    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(one, jobs))
    report(args.output)
    print(json.dumps({"complete": len(jobs), "report": str(args.output / "comparison.json")}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("compare", "report"))
    parser.add_argument("--author", type=Path)
    parser.add_argument("--previous", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--decisions", type=int, default=96)
    parser.add_argument("--new-only", action="store_true")
    args = parser.parse_args()
    if args.command == "report":
        result = report(args.output)
        print(json.dumps([{k:r.get(k) for k in ("name", "status", "clean", "goal_reached", "blocked_moves", "decisions")}
                          for r in result["runs"]]))
    else:
        compare(args)
