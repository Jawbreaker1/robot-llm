"""Offline comparison of robot-specific numeric weights, never hardware wiring.

Author each candidate once, freeze all three artifacts, then run the unchanged
simulator/planner entry point. No training, route rule, action override or new
inference layer is introduced here.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import inspect
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time

from experiments import local_navigation as encoder
from experiments.navigation_set_author import expand, request_for
from experiments.navigation_set_policy import SetPolicy
from robot_agent.codex_subscription import CodexSubscription
from robot_agent.navigation_diagnostics import enable_navigation_diagnostics

REPO = Path(__file__).resolve().parents[1]
ROBOTS = ("ev3", "blast")
SCENES = ("box", "bend", "many-boxes")
NUMERIC_KEYS = tuple(name+suffix for name in ("ray", "memory", "head", "output")
                     for suffix in ("_weights", "_bias"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hashes():
    paths = [p for folder in ("src/robot_agent", "experiments")
             for p in (REPO/folder).glob("*.py")]
    paths += [REPO/"scripts/local_navigation_experiment.py",
              REPO/"scripts/validate_ev3_navigation.py"]
    return {str(p.relative_to(REPO)):digest(p) for p in sorted(paths)}


def specialization_request(robot, specification):
    if robot not in ROBOTS:
        raise ValueError("Unknown robot")
    request = request_for({"previous_specification":specification})
    contract = json.loads(request["messages"][1]["content"])
    name = "EV3" if robot == "ev3" else "Blast"
    contract["robots"] = {name:contract["robots"][name]}
    contract["task"] = (
        "Replace the previous numeric policy for "+name+" with one that actually "
        "uses goal/waypoint geometry, positioned sensing and spatial route memory "
        "to choose local navigation actions. The previous policy mostly switched "
        "between following a host motion hint and delegating to Astra; do not "
        "preserve that arbitration or merely add weak connections to it. You may "
        "rewrite every weight and bias within the SAME architecture and existing "
        "action interface. Astra still authors waypoint routes; the local network "
        "owns its choice among execution, investigation, recovery and delegation. "
        "No training, executable rules, new layers, tools or environment access."
    )
    contract["audit_findings"] = {
        "same_inputs": "350 retained-route packets: all score vectors changed but zero raw or masked winner changes after specialization.",
        "zero_connections": "All three previous artifacts had zero weights for global goal/waypoint XY, global route vertices, ray-to-waypoint geometry and memory leg/pose XY.",
        "observed_role": "339/350 selections were FOLLOW_WAYPOINT or REPLAN. Identical recorded Astra responses produced identical outcomes with either old weight file.",
        "runtime": "OBSERVE is now available and executes the existing motorless settled-sampling command for Blast as well as EV3. Known-body/pose, cancellation and motion-admission contracts remain unchanged.",
    }
    contract["global_encoder_source"] = "\n\n".join(inspect.getsource(f) for f in (
        encoder._number, encoder._category, encoder._boolean, encoder._motion, encoder.features))
    contract["execution_contract"] = [s.replace("waypoint axis", "waypoint bearing")
                                      for s in contract["execution_contract"]]
    contract["execution_contract"] += [
        "Both followers track the retained endpoint using measured-pose feedback. "
        "The next_waypoint_motion hint describes the REQUIRED motion independently "
        "of admission; available actions and forward_admission report permission. "
        "Motorless action outcomes are retained in route memory. Last motor motion "
        "may predate the latest blocked action. Mission arrival remains 50 mm."]
    contract["execution_contract"] += [
        "FOLLOW_WAYPOINT can run multiple measured motor pulses until the active "
        "endpoint or interruption. The local policy is not queried during each "
        "pulse. Direct turns/advance/reverse and OBSERVE retain, but do not follow, "
        "the route; the model is queried again afterwards. OBSERVE returns a fresh "
        "stationary sample without sweeping or declaring space free."]
    contract["geometry_objective"] = (
        "Use the existing coordinate features as decision-relevant information, "
        "not ornamental nonzero coefficients. Goal progress, the actual waypoint "
        "bearing and the geometry of subsequent route legs can differ from the "
        "host's immediate motion hint. Positioned rays must remain tied to their "
        "own geometry; compare sensed space with this robot's actual body. For "
        "IR use direction, return strength and observation origins without "
        "inventing metric endpoints. Route-memory positions distinguish failures "
        "near the currently attempted passage from failures elsewhere or on old "
        "legs; identity flags alone are insufficient. Keep successful progress "
        "and failed attempts distinct. Choose the numeric tradeoffs yourself; "
        "no host rule, preferred side, target action examples or test layout is "
        "provided. Nonzero connections alone will not count as functional use: "
        "after freezing we will test score AND action sensitivity to meaningful "
        "changes in goal, sensing and memory, then run both robots in simulation."
    )
    contract["protocol"] = (
        "Write one COMPLETE numeric candidate in the supplied schema. All omitted "
        "numbers are zero, not inherited. The host only expands and validates "
        "numeric arrays. No gradients, fitting, tools, training, executable policy, "
        "forward passes, environment interaction, new rules or architecture changes. "
        "Only the structural audit above and previous numeric specification are "
        "provided, not upcoming trial feedback or action labels. This is an "
        "audit-informed re-authoring, not zero-shot or guaranteed generalization."
    )
    request["messages"][1]["content"] = json.dumps(contract)
    return request


def author(args):
    spec = json.loads(args.shared.with_name("authored-weights.json").read_text())
    shared = json.loads(args.shared.read_text())
    expanded = expand(spec)
    if any(shared[k] != expanded[k] for k in NUMERIC_KEYS):
        raise ValueError("Shared weights do not match their authored specification")
    initial_sources = source_hashes()
    args.output.mkdir(parents=True, exist_ok=False)
    request = specialization_request(args.robot, spec)
    (args.output/"request.json").write_text(json.dumps(request))
    enable_navigation_diagnostics(args.output/"diagnostics.jsonl")
    started = time.monotonic()
    response = json.loads(CodexSubscription().complete(request, timeout=600, maximum=2*1024*1024))
    (args.output/"response.json").write_text(json.dumps(response))
    if response.get("resolved_model") != "gpt-6-astra":
        raise ValueError("Weight author was not the requested Astra model")
    authored = json.loads(response["choices"][0]["message"]["content"])
    artifact = expand(authored)
    artifact.update(author_model=response["resolved_model"], author_reasoning_effort="high",
                    specialized_robot=args.robot, method="astra-direct-spatial-policy-v2")
    if source_hashes() != initial_sources:
        raise ValueError("Code changed during authoring; candidate must not be compared")
    (args.output/"authored-weights.json").write_text(json.dumps(authored,indent=2))
    path = args.output/"policy-direct.json"
    path.write_text(json.dumps(artifact,separators=(",", ":")))
    provenance = {"robot":args.robot,"model":response["resolved_model"],"reasoning":"high",
        "seconds":round(time.monotonic()-started,2),"usage":response.get("usage"),
        "policy_sha256":digest(path),"parent_policy_sha256":digest(args.shared),
        "request_sha256":digest(args.output/"request.json"),"source_hashes":initial_sources,
        "training_rows":0,"gradient_steps":0,"author_forward_passes":0,
        "author_environment_rollouts":0,"completed_candidates":1,
        "design_summary":authored["design_summary"],"limitations":authored["limitations"]}
    (args.output/"provenance.json").write_text(json.dumps(provenance,indent=2))
    print(json.dumps({k:v for k,v in provenance.items() if k not in (
        "source_hashes","design_summary","limitations")}),flush=True)


def schedule():
    jobs = []
    for repeat, scenes in ((0, SCENES), (1, ("many-boxes",))):
        for scene_index, scene in enumerate(scenes):
            for robot_index, robot in enumerate(ROBOTS):
                variants = ("shared", "specialized")
                if (repeat+scene_index+robot_index) % 2:
                    variants = variants[::-1]
                for variant in variants:
                    jobs.append({"robot":robot,"scene":scene,"repeat":repeat+1,"variant":variant,
                                 "name":f"{robot}-{scene}-r{repeat+1}-{variant}"})
    return jobs


def summarize(root):
    manifest = json.loads((root/"manifest.json").read_text())
    rows = []
    for job in manifest["jobs"]:
        path = root/job["name"]/"summary.json"
        if not path.exists():
            rows.append(dict(job,status="missing summary"))
            continue
        result = json.loads(path.read_text())
        row = dict(job,status="completed",**{k:result.get(k) for k in (
            "clean","goal_reached","blocked_moves","distance_to_goal_mm","decisions",
            "astra_calls","local_calls","wall_seconds","terminal_reason","error_type","error_message")})
        for kind in ("local", "astra"):
            samples = result.get(kind+"_decision_ms") or []
            row[kind+"_median_ms"] = round(statistics.median(samples),3) if samples else None
        events = [json.loads(l) for l in path.with_name("events.jsonl").read_text().splitlines()]
        row["planner_retries"] = sum(e.get("payload",{}).get("event") == "planner_retry" for e in events)
        row["policy_sha256"] = events[0]["model_sha256"]
        row["world_sha256"] = hashlib.sha256(json.dumps(events[0]["world"],sort_keys=True).encode()).hexdigest()
        rows.append(row)
    worlds = {}
    for row in rows:
        if row["status"] != "completed":
            continue
        key = (row["robot"],row["scene"],row["repeat"])
        if worlds.setdefault(key,row["world_sha256"]) != row["world_sha256"]:
            raise ValueError("Paired trials used different worlds")
    report = {"protocol":"frozen shared versus robot-specialized weights; same code and worlds",
        "training":False,"physical_robots":False,"runs":rows,
        "limitations":["Small repeated regression set, not held-out generalization evidence.",
            "Astra route decisions are stochastic; no seed-controlled identical route is imposed.",
            "Wall times include shared-provider load; decision counts and collision-free goal success are primary.",
            "Simulator uses approximate white-surface IR and ideal range/encoder measurements."]}
    (root/"comparison.json").write_text(json.dumps(report,indent=2))
    return report


def compare(args):
    weights = {"shared":args.shared.resolve(), "ev3":args.ev3.resolve(), "blast":args.blast.resolve()}
    for key,path in weights.items():
        artifact = json.loads(path.read_text())
        SetPolicy(artifact)
        if key != "shared" and artifact.get("specialized_robot") != key:
            raise ValueError("Specialized artifact does not match robot")
    hashes, sources, jobs = {k:digest(p) for k,p in weights.items()}, source_hashes(), schedule()
    args.output.mkdir(parents=True,exist_ok=False)
    manifest = {"weights":{k:str(v) for k,v in weights.items()},"weight_hashes":hashes,
        "source_hashes":sources,"max_decisions":48,"workers":2,"jobs":jobs,
        "no_post_evaluation_weight_revision":True,"model":"codex/gpt-6-astra","reasoning":"high"}
    (args.output/"manifest.json").write_text(json.dumps(manifest,indent=2))

    def run_job(job):
        if source_hashes() != sources or {k:digest(p) for k,p in weights.items()} != hashes:
            raise ValueError("Frozen comparison code or weights changed")
        model = weights["shared" if job["variant"] == "shared" else job["robot"]]
        output = args.output/job["name"]
        command = [sys.executable,"scripts/local_navigation_experiment.py","run",
            "--robot",job["robot"],"--scene",job["scene"],"--weights",str(model),
            "--output",str(output.resolve()),"--decisions","48"]
        environment = dict(os.environ,PYTHONPATH=".:src",PYTHONDONTWRITEBYTECODE="1")
        with (args.output/(job["name"]+".log")).open("w") as log:
            result = subprocess.run(command,cwd=REPO,env=environment,stdout=log,stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError(f"Trial process failed: {job['name']} ({result.returncode})")
        summary = json.loads((output/"summary.json").read_text())
        print(json.dumps(dict(job,**{k:summary.get(k) for k in (
            "clean","goal_reached","blocked_moves","decisions","astra_calls","wall_seconds")})),flush=True)
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(run_job,jobs))
    if source_hashes() != sources:
        raise ValueError("Code changed during comparison")
    report = summarize(args.output)
    print(json.dumps({"completed_runs":len(report["runs"]),"report":str(args.output/"comparison.json")}),flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command",required=True)
    p = sub.add_parser("author")
    p.add_argument("--robot",choices=ROBOTS,required=True)
    p.add_argument("--shared",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p = sub.add_parser("compare")
    for name in ("shared","ev3","blast","output"):
        p.add_argument("--"+name,type=Path,required=True)
    p = sub.add_parser("report")
    p.add_argument("--output",type=Path,required=True)
    args = parser.parse_args()
    if args.command == "report":
        summarize(args.output)
    else:
        {"author":author,"compare":compare}[args.command](args)


if __name__ == "__main__":
    main()
