"""One Astra-authored replacement set network; no training or online repair."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
from copy import deepcopy

from experiments import navigation_set_policy as net
from experiments.navigation_weight_space import author_request, compact_packet, feature_layout
from robot_agent.codex_subscription import CODEX_MODEL, CodexSubscription
from robot_agent.navigation_diagnostics import enable_navigation_diagnostics

METHOD = "astra-direct-weights-point-geometry-repair-v2"


def schema():
    def obj(properties):
        return {"type":"object", "properties":properties, "required":list(properties), "additionalProperties":False}
    num = {"type":"number"}
    edge = obj({"feature":{"type":"integer"}, "weight":num})
    unit = obj({"index":{"type":"integer"}, "bias":num,
                "inputs":{"type":"array", "items":edge}, "purpose":{"type":"string"}})
    layer = {"type":"array", "items":unit}
    return obj({"design_summary":{"type":"string"}, "limitations":{"type":"string"},
                "ray":layer, "memory":layer, "head":layer, "output":layer})


def expand(spec):
    result = {"version":net.VERSION, "actions":list(net.ACTIONS), "method":METHOD}
    for name, size, width in (("ray",len(net.RAY_FIELDS),net.SET_UNITS),
        ("memory",len(net.MEMORY_FIELDS),net.SET_UNITS),
        ("head",net.GLOBAL_SIZE+4*net.SET_UNITS+2,net.HEAD_UNITS),
        ("output",net.HEAD_UNITS,len(net.ACTIONS))):
        weights, bias, seen = [[0.0]*width for _ in range(size)], [0.0]*width, set()
        for unit in spec[name]:
            j = unit["index"]
            if type(j) is not int or not 0 <= j < width or j in seen:
                raise ValueError("Duplicate/invalid unit index")
            seen.add(j)
            bias[j] = unit["bias"]
            inputs = set()
            for edge in unit["inputs"]:
                i = edge["feature"]
                if type(i) is not int or not 0 <= i < size or i in inputs:
                    raise ValueError("Duplicate/invalid input index")
                inputs.add(i)
                weights[i][j] = edge["weight"]
        result[name+"_weights"], result[name+"_bias"] = weights, bias
    net.SetPolicy(result)  # shape/finite validation only; no forward pass
    return result


def recorded_evidence(root):
    cases = []
    categories = set()
    def collect_categories(obj):
        if isinstance(obj,dict):
            for key,value in obj.items():
                if key in ("reason","result","action_error","motion_outcome","range_state") and isinstance(value,str):
                    categories.add(value)
                collect_categories(value)
        elif isinstance(obj,list):
            for value in obj:
                collect_categories(value)
    for path in sorted((root/"test").glob("*-local/summary.json")):
        summary = json.loads(path.read_text())
        rows = [json.loads(l) for l in path.with_name("events.jsonl").read_text().splitlines()]
        pairs = [r for r in rows if r["event"] == "decision_pair"]
        indices = {0, len(pairs)-1}
        for i,pair in enumerate(pairs):
            p = pair["packet"]
            history = p.get("history") or []
            if any(h.get("route_rejection") or (h.get("last_motion") or {}).get("completed") is False for h in history):
                indices.update(range(i,min(len(pairs),i+3)))
                break
        snapshots = []
        for i in sorted(indices):
            p = compact_set_packet(pairs[i]["packet"])
            collect_categories(p)
            # No new memory was recorded then. Do not fabricate missing routes.
            snapshots.append({"step":i+1, "packet":p, "old_decision":pairs[i]["decision"]})
        cases.append({"robot":summary["robot"], "summary":{k:summary.get(k) for k in
            ("clean","goal_reached","blocked_moves","decisions")}, "snapshots":snapshots,
            "recording_sha256":hashlib.sha256(path.with_name("events.jsonl").read_bytes()).hexdigest()})
    return {"previous_specification":json.loads((root/"author/authored-weights.json").read_text()),
        "previous_policy_sha256":hashlib.sha256((root/"author/policy-direct.json").read_bytes()).hexdigest(),
        "cases":cases, "category_codes":{name:net._category(name) for name in sorted(categories)},
        "note":"Previous frozen policy, recorded observations/actions only. No target labels or hidden geometry. Preserve recorded persistent route memory; missing memory in older recordings must not be inferred as an empty mission history. Category codes mechanically expose the encoder values, not assigned responses."}


def compact_set_packet(packet):
    """Preserve the set model's inputs, not the legacy fixed-history projection."""
    result = compact_packet(packet)
    source = packet.get("local_map_evidence") or {}
    evidence = result["local_map_evidence"]
    if "route_execution_memory" in source:
        evidence["route_execution_memory"] = deepcopy(source["route_execution_memory"])
    if "ir_survey" in source:
        evidence["ir_survey"]["samples"] = deepcopy(source["ir_survey"].get("samples", []))
    else:
        evidence["range_observations"] = deepcopy(source.get("range_observations", []))
    execution = packet.get("execution") or {}
    if "reverse_step_mm" in execution:
        result["execution"]["reverse_step_mm"] = execution["reverse_step_mm"]
    if net.observations(result) != net.observations(packet):
        raise ValueError("Author evidence dropped an encoded set-model observation")
    return result


def request_for(repair):
    base = json.loads(author_request()["messages"][1]["content"])
    contract = {
        "task":"Replace the prior numeric weights for the new set-network observation contract. Use all supplied sensor geometry and persistent route outcome memory. No training, no new rules.",
        "robots":base["robots"], "execution_contract":base["execution_contract"],
        "objective":base["objective"], "repair_evidence":repair,
        "architecture":{
            "global_features":net.GLOBAL_SIZE, "global_layout":[r for r in feature_layout() if r["start"]<net.GLOBAL_SIZE],
            "ray_features":dict(enumerate(net.RAY_FIELDS)), "memory_features":dict(enumerate(net.MEMORY_FIELDS)),
            "set_units":net.SET_UNITS, "head_units":net.HEAD_UNITS, "actions":list(net.ACTIONS),
            "head_layout":{"global":[0,251],"rays_max":[252,267],"rays_mean":[268,283],
                           "memory_max":[284,299],"memory_mean":[300,315],"log1p_ray_count":316,"log1p_memory_row_count":317},
            "formula":"For each ray: tanh(ray @ ray_weights + ray_bias), same weights for EVERY supplied ray. Concatenate per-unit max and mean (empty set zeros). Same for ALL memory rows. Concatenate global, ray pool, memory pool, log1p counts; h=tanh(concat @ head_weights + head_bias); logits=h @ output_weights + output_bias. Argmax available actions plus REPLAN. No confidence threshold or veto.",
        },
        "source":Path(net.__file__).read_text(),
        "memory_source":(Path(net.__file__).parents[1]/"src/robot_agent/route_execution_memory.py").read_text(),
        "semantics":[
            "All packet rays are processed, with no extra truncation. Current production packets retain at most 128. Set pooling is invariant to ray ordering. Metric endpoints are explicit coordinate transforms, NOT obstacle avoidance rules. IR has no metric endpoint.",
            "Each metric endpoint also has signed distances to the actual asymmetric body, the next advance/reverse pulse's swept rectangle, and the enclosing turn circle. Positive is outside; negative is inside. Margin is included once. These are geometric measurements, not action permissions or clearance guarantees. No return and IR have zero placeholder distances with metric_endpoint_present=0. Separate half-space maxima may originate from different points: do not treat their combination as a single nearby hit.",
            "All route-outcome records from the complete episode journal are retained; equivalent repeated route/outcome pairs aggregate count. No four-step or twelve-step expiry. A successful scan or reverse does not erase prior outcomes. Current route legs are also rows, with is_current_route=1; no four-waypoint truncation in that set.",
            "Incomplete motion alone is not proof of a bad route: partial turns/scans may make useful progress. Records preserve reasons, primitive, route identity and actual movement. Same_active_leg_endpoint only compares coordinates, it does not assert blockage or choose a response. A revised route may legitimately share other endpoints with a previous route.",
            "Head global indices retain the old encoder layout, including four recent route points but NOT its four-history or fixed-ray input. The new memory set provides all route legs and outcomes. The global last_motion/rejection reflects immediate feedback. Category codes are the existing SHA256 bit encoding, not ordinal numbers.",
        ],
        "protocol":"Return exactly one COMPLETE numeric candidate: ray, memory, head, output unit arrays. Each index addresses the OUTPUT unit in that layer and its inputs list INPUT feature indices. Omitted numbers are zero. All nonzero weights/biases are authored by you, the host only assigns arrays and checks shapes/finiteness. No gradients, fitting, generated executable policy, environment interaction, candidate forward passes, or new navigation rules. Upcoming rerun data is unavailable. This is a recorded-feedback repair plus representation replacement, not a zero-shot/independent held-out claim. Describe limitations honestly; choose weights yourself.",
    }
    return {"model":CODEX_MODEL, "reasoning_effort":"high", "max_tokens":32768,
        "messages":[{"role":"system","content":"Author neural network numeric weights directly. No training or executable code."},
                    {"role":"user","content":json.dumps(contract)}],
        "response_format":{"type":"json_schema","json_schema":{"name":"navigation_set_weights","strict":True,"schema":schema()}}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--repair-from",type=Path,required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    repair = recorded_evidence(args.repair_from)
    request = request_for(repair)
    (args.output/"request.json").write_text(json.dumps(request))
    enable_navigation_diagnostics(args.output/"diagnostics.jsonl")
    start = time.monotonic()
    raw = CodexSubscription().complete(request,timeout=600,maximum=2*1024*1024)
    (args.output/"response.json").write_bytes(raw)
    response = json.loads(raw)
    spec = json.loads(response["choices"][0]["message"]["content"])
    (args.output/"authored-weights.json").write_text(json.dumps(spec,indent=2))
    artifact = expand(spec)
    artifact.update(author_model=response["resolved_model"],author_reasoning_effort="high")
    model_path = args.output/"policy-direct.json"
    model_path.write_text(json.dumps(artifact,separators=(",",":")))
    source = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
        Path(net.__file__),Path(__file__),Path(net.__file__).parents[1]/"src/robot_agent/route_execution_memory.py")}
    provenance = {"method":METHOD,"resolved_model":response["resolved_model"],"usage":response.get("usage"),
        "seconds":round(time.monotonic()-start,2),"policy_sha256":hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "request_sha256":hashlib.sha256((args.output/"request.json").read_bytes()).hexdigest(),
        "source_hashes":source,"parent_policy_sha256":repair["previous_policy_sha256"],
        "training_rows":0,"gradient_steps":0,"author_forward_passes":0,"author_environment_rollouts":0,
        "completed_candidates":1,"repeated_evaluation_not_independent":True,
        "design_summary":spec["design_summary"],"author_limitations":spec["limitations"]}
    (args.output/"provenance.json").write_text(json.dumps(provenance,indent=2))
    print(json.dumps(provenance),flush=True)


if __name__ == "__main__":
    main()
