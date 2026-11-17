"""Astra writes a Python tensor constructor; inference executes numeric arrays only.

Request, review, materialize and freeze are separate steps. No gradients, fitting,
optimizer or model-generated runtime policy. Never expose held-out test worlds.
"""
import argparse
import ast
import hashlib
import inspect
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from experiments import navigation_attention_policy as network
from experiments.navigation_weight_space import author_request, feature_layout
from experiments.navigation_set_policy import observations, LAST_ACTION_FIELDS, GLOBAL_SIZE
from experiments.local_navigation import features, _category, _boolean, _number, _motion
from robot_agent.route_execution_memory import latest_action_feedback, route_execution_memory
from robot_agent.codex_subscription import ASTRA_MODEL_ID, CODEX_MODEL, CodexSubscription

REPO = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit_constructor(source):
    tree = ast.parse(source)
    # This is a syntax guard, NOT a security sandbox. Review candidate.py before
    # running freeze; construction runs once offline and is never loaded at inference.
    forbidden = {"open", "exec", "eval", "compile", "__import__", "input", "getattr", "setattr",
                 "load", "save", "savez", "savez_compressed", "loadtxt", "savetxt", "fromfile", "tofile"}
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module]
            if any(n not in ("numpy", "math", "hashlib") for n in names):
                raise ValueError("Constructor imports outside numeric tools")
        if isinstance(node, ast.Name) and (node.id in forbidden or node.id.startswith("__")):
            raise ValueError("Forbidden constructor name")
        if isinstance(node, ast.Attribute) and (node.attr in forbidden or node.attr.startswith("__")):
            raise ValueError("Forbidden constructor attribute")
    if not any(isinstance(n, ast.FunctionDef) and n.name == "build" for n in tree.body):
        raise ValueError("Missing build(robot) constructor")
    return tree


def evidence(root):
    result = {}
    for robot in ("ev3", "blast"):
        summary = json.loads((root / "validation" / robot / "summary.json").read_text())
        provenance = json.loads((root / ("author-" + robot) / "authored-weights.json").read_text())
        result[robot] = {"results": {k: summary.get(k) for k in
            ("goal_reached", "blocked_moves", "decisions", "action_counts")},
            "prior_analysis": provenance.get("failure_analysis"),
            "prior_limitations": provenance.get("limitations")}
    return result


def request(args):
    args.output.mkdir(parents=True, exist_ok=False)
    base = json.loads(author_request()["messages"][1]["content"])
    contract = {
        "task": "Author TWO new, substantially larger neural navigation policies with Python: build('ev3') and build('blast'). Replace the previous tiny pooled model. No training or optimizer. You may program numeric matrices algorithmically. The returned code only constructs arrays offline; the fixed neural forward pass below is the ONLY runtime policy.",
        "robots": base["robots"], "execution_contract": base["execution_contract"],
        "objective": base["objective"],
        "architecture": {"width": network.WIDTH, "heads": network.HEADS, "blocks": network.BLOCKS,
            "ff_width": network.FF_WIDTH, "shapes": network.shapes(), "actions": list(network.ACTIONS),
            "formula": "One global token, every supplied ray token, every supplied route-memory-leg token. Linear per-type embedding plus type embedding. Three residual multi-head scaled dot-product attention + residual ReLU feedforward blocks, NO layer normalization. Output is global token @ out_w + out_b. No pooling or truncation. All tokens attend all tokens. No positional index embedding: geometric position, route leg index and event age are explicit input fields. Row permutations do not change the global output.",
            "global_fields": [r for r in feature_layout() if r["start"] < GLOBAL_SIZE]
                + [{"start": GLOBAL_SIZE+i, "name": name, "size": 1}
                   for i,name in enumerate(LAST_ACTION_FIELDS)],
            "ray_fields": dict(enumerate(network.RAY_FIELDS)),
            "memory_fields": dict(enumerate(network.MEMORY_FIELDS)),
            "map_fields": dict(enumerate(network.MAP_FIELDS)),
            "appended_execution_fields": dict(enumerate(network.EXECUTION_FIELDS, start=303))},
        "observation_source": inspect.getsource(observations) + "\n" + inspect.getsource(network.observations),
        "observation_call": "network.observations(packet) returns global, rays, memory, observed-map tokens",
        "global_and_category_sources": "\n\n".join(inspect.getsource(f) for f in
            (features, _category, _boolean, _number, _motion)),
        "shared_receipt_sources": "\n\n".join(inspect.getsource(f) for f in
            (latest_action_feedback, route_execution_memory)),
        "forward_source": inspect.getsource(network.AttentionPolicy.scores),
        "previous_results": evidence(args.previous),
        "important_semantics": [
            "NEW explicit action feedback: global indices 252..302 encode the latest recorded operation, even when no route or motor motion exists. Memory indices 70..73 are action_completed true/false and observation_acquired true/false. Legacy memory completed/progress at 12..15 are MOTION facts, not action completion. Boolean unknown is [0,0], false [0,1], true [1,0]. Never interpret a successful OBSERVE without motor motion as failed travel.",
            "OBSERVE completed=true, observation_acquired=true means a settled stationary measurement was taken; the measurement can still be no-return/unknown/blocked. A failed or unsettled OBSERVE has both false. SCAN action completion includes heading restoration, while acquired samples are distinct from completing the sweep. Completion describes the recorded operation/primitive, NOT arrival or full-route success. EV3 FOLLOW can return heading_aligned with action_completed=false and completed last-motion turn; Blast records its FOLLOW primitives separately. Use both action and primitive, not false completion alone as proof of an obstacle.",
            "All positions are robot-relative metres. Do not infer an obstacle extent/edge from a single IR return. Metric endpoint and range-state masks matter. Unknown is neither confirmed blockage nor confirmed full body clearance.",
            "FOLLOW_WAYPOINT may execute several motor pulses before returning. Direct ADVANCE is one pulse, not a waypoint follower. It preserves the route. OBSERVE is stationary sampling. SCAN_FRONT_ARC returns to original heading when successful; its completion is not translation progress. Use requested action and actual primitive to distinguish stalled travel from successful scans/turns.",
            "Route memory keeps full episode grouped outcomes and count; one row may stand for many failures. Old failures elsewhere are not local blockage. A partial turn with progress is not a failed passage. Memory includes route leg coordinates, before/after pose, age, repetition count and outcomes. No four-record cutoff.",
            "Astra still owns waypoint routes and is called with the full context on REPLAN. Local model must contribute useful measured execution, sensing and recovery rather than always delegate. Geometry and relevant memory must change decisions, not just negligible score decimals.",
            "This is a larger representation, not a target to fill with inert/random weights. Use multiple attention heads/blocks meaningfully to bind sensor and memory geometry to the active route. Do not simply embed the old 16-unit gate into a larger zero network. Explain effective usage and limitations honestly."],
        "protocol": "Return JSON with python_source, design_summary and limitations. python_source defines build(robot) returning a dict of every named np.float32 tensor with exactly the supplied shapes. It may use numpy, math, hashlib only, no file/network IO, no reading previous tensors, no optimizer/gradient/training/fitting, no random padding, no environment-specific maps or obstacle coordinates. Helpers and loops for constructing coefficients are allowed. Use zero initialization and deliberate coefficients. All action selection is computed by the fixed neural forward pass, never a returned Python runtime function. Up to one bounded revision can use offline forward-probe results before final freeze; unseen evaluation worlds will not be supplied. No training is allowed even during revision. Provide concise design rationale, not private chain of thought.",
    }
    if args.revision:
        contract["previous_constructor"] = (args.revision / "author-revised/candidate.py").read_text()
        contract["revision_feedback"] = {
            "observed_failure": "EV3 many-boxes chose OBSERVE 84 times. Successful observations were only a text reason observation_refreshed; motion.completed and made_progress stayed null. Both previous networks ignored all reason bits, and their successful-sensing detector required motion.completed=true. The shared adapters now explicitly report the action receipt without inventing movement. Rewrite the contradictory utility construction, do not add after-N-scan rules or a host-side action override.",
            "scope": "Rewrite both robot weights using actual sensing completion, geometry and local route failure memory. No training or world-specific coordinates. All six evaluation worlds have been tested before, so they are regressions, not pristine held-out tests.",
        }
    payload = {"model": CODEX_MODEL, "reasoning_effort": "high", "max_tokens": 32768,
        "messages": [{"role": "system", "content": "Write an offline Python constructor for neural-network numeric weights. No training. Follow the fixed model contract."},
                     {"role": "user", "content": json.dumps(contract)}],
        "response_format": {"type": "json_schema", "json_schema": {"name": "attention_constructor", "strict": True,
            "schema": {"type": "object", "properties": {key: {"type": "string"} for key in
                ("python_source", "design_summary", "limitations")}, "required": ["python_source", "design_summary", "limitations"],
                "additionalProperties": False}}}}
    (args.output / "request.json").write_text(json.dumps(payload))
    started = time.monotonic()
    response = json.loads(CodexSubscription().complete(payload, timeout=900, maximum=3*1024*1024))
    if response.get("resolved_model") != ASTRA_MODEL_ID:
        raise ValueError("Unexpected author model")
    (args.output / "response.json").write_text(json.dumps(response))
    authored = json.loads(response["choices"][0]["message"]["content"])
    audit_constructor(authored["python_source"])
    source = authored["python_source"].rstrip() + "\n"
    patch = "*** Begin Patch\n*** Add File: " + str((args.output / "candidate.py").resolve()) + "\n"
    patch += "".join("+" + line + "\n" for line in source.splitlines()) + "*** End Patch\n"
    subprocess.run(["apply_patch"], input=patch, text=True, check=True, capture_output=True)
    (args.output / "author.json").write_text(json.dumps({"model": response["resolved_model"],
        "reasoning": "high", "seconds": time.monotonic()-started, "usage": response.get("usage"),
        "design_summary": authored["design_summary"], "limitations": authored["limitations"],
        "training": False, "source_sha256": digest(args.output / "candidate.py")}, indent=2))
    print(json.dumps({"author_complete": True, "source": str(args.output / "candidate.py"),
                      "seconds": round(time.monotonic()-started)}), flush=True)


def freeze(args):
    source = (args.output / "candidate.py").read_text()
    audit_constructor(source)
    namespace = {}
    exec(compile(source, str(args.output / "candidate.py"), "exec"), namespace)
    report = {"training": False, "source_sha256": digest(args.output / "candidate.py"), "robots": {}}
    for robot in ("ev3", "blast"):
        weights = namespace["build"](robot)
        repeated = namespace["build"](robot)
        if set(weights) != set(repeated) or any(not np.array_equal(weights[k], repeated[k]) for k in weights):
            raise ValueError("Constructor must be deterministic")
        path = args.output / (robot + ".npz")
        if path.exists():
            raise ValueError("Refusing to overwrite frozen weights")
        network.save_weights(path, weights, robot=robot)
        report["robots"][robot] = {"path": str(path), "sha256": digest(path),
            "parameters": sum(v.size for v in weights.values()),
            "nonzero": sum(int(np.count_nonzero(v)) for v in weights.values()),
            "tensor_bytes": sum(v.nbytes for v in weights.values()), "file_bytes": path.stat().st_size}
    (args.output / "frozen.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("request", "freeze"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--previous", type=Path)
    parser.add_argument("--revision", type=Path)
    arguments = parser.parse_args()
    {"request": request, "freeze": freeze}[arguments.command](arguments)
