"""Direct numeric weight authoring at the existing local-policy boundary.

The author supplies numeric weights, not executable code. Expansion only copies
those numbers into zero-filled matrices. No fitting, forward pass, simulation,
old policy, episode archive or test geometry is used in zero-shot mode. Repair
mode exposes the previous weights and recorded observation-only failure windows,
not hidden world geometry. Neither mode trains or runs the candidate before freeze.
"""
import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path
import time
from copy import deepcopy

from experiments import local_navigation as network
from robot_agent.codex_subscription import CodexSubscription
from robot_agent.navigation_diagnostics import enable_navigation_diagnostics

METHOD = "astra-direct-weights-zero-shot-v1"
REPAIR_METHOD = "astra-direct-weights-recorded-repair-v1"
HIDDEN = 32
MODEL = "codex/gpt-6-astra"


def feature_layout():
    """Documentation of the unchanged encoder; checked against it in tests."""
    rows, cursor = [], 0
    def add(name, count, meaning):
        nonlocal cursor
        rows.append({"name":name,"start":cursor,"count":count,"meaning":meaning})
        cursor += count
    def motion(prefix):
        add(prefix+".present",1,"0/1")
        add(prefix+".reason",32,"category code")
        add(prefix+".action",7,"one-hot ACTIONS[0:7]")
        add(prefix+".completed",2,"true,false flags; unknown=0,0")
        add(prefix+".made_progress",2,"true,false flags; unknown=0,0")
        add(prefix+".measurements",5,"translation/1000, measured turn/180, requested turn/180, sweep/360, log1p(no-progress count)")
    def rejection(prefix):
        add(prefix+".present",1,"0/1")
        add(prefix+".reason",32,"category code")
        add(prefix+".geometry",5,"leg_index/8, clearance/300, blocker present, blocker relative forward/1000,left/1000")
    add("is_ir",1,"1=EV3 IR, 0=Blast ultrasound")
    add("active_waypoint_relative",2,"ego forward,left /1000")
    add("goal_relative",2,"ego forward,left /1000")
    add("route_status",2,"active waypoint present, route length/8")
    for i in range(4):
        add(f"route[{i}]",3,"present, relative forward/1000,left/1000; zero when absent")
    add("body",6,"front,rear,left,right,margin,turn-sweep-radius /300; all measured from drive origin")
    add("execution_dimensions",2,"advance step/100, waypoint radius/100")
    add("sensor_mount",3,"numeric offset known, forward/300,left/300")
    add("current_range",2,"value present, IR/100 or ultrasound/2000; not equivalent units")
    add("touch",2,"true,false flags")
    add("range_state",32,"category code")
    add("available_actions",7,"one-hot availability for ACTIONS[0:7]; REPLAN always available")
    motion("last_motion")
    add("forward_admission.permitted",2,"true,false flags, only immediate primitive admission; NOT path clearance")
    add("forward_admission.reason",32,"category code")
    add("next_waypoint_motion.present",1,"0/1")
    add("next_waypoint_motion.action",7,"one-hot ACTIONS[0:7]")
    add("next_waypoint_motion.geometry",3,"remaining turn/180, projected forward travel/1000, projected waypoint miss/300")
    rejection("latest_route_rejection")
    add("latest_route_rejection.repeat_count",1,"log1p(count)")
    add("latest_route_rejection.pose_or_evidence_changed",2,"true,false flags")
    add("latest_route_rejection.same_route",1,"rejected and active route coordinates match")
    add("ir_survey.complete",2,"true,false flags")
    add("ir_survey.geometry",2,"sweep degrees/360, movement commands since scan/10")
    add("ir_survey.reason",32,"category code")
    add("previous_action",7,"one-hot ACTIONS[0:7]")
    for i in range(4):
        prefix = f"history[{i}]"
        add(prefix+".pose",5,"present, relative forward/1000,left/1000,sin(relative heading),cos(relative heading)")
        add(prefix+".action",7,"one-hot ACTIONS[0:7]")
        for name in ("result","action_error","motion_outcome","route_interruption.reason"):
            add(prefix+"."+name,32,"category code")
        add(prefix+".route_interruption.distance",1,"distance/2000")
        add(prefix+".scan.state",32,"category code")
        add(prefix+".scan.result",32,"category code")
        add(prefix+".scan.restoration_verified",2,"true,false flags")
        add(prefix+".scan.all_observations_settled",2,"true,false flags")
        add(prefix+".scan.coverage_and_repeat",2,"sweep/360, log1p(repeat count)")
        motion(prefix+".last_motion")
        rejection(prefix+".route_rejection")
    add("rays",128*8,"128 blocks, oldest first, zero padding at front. Each: present, raw value present, IR/100 or ultrasound/2000, origin relative forward/1000,left/1000,sin(relative ray heading),cos(relative ray heading),settled")
    if cursor != len(network.features({})):
        raise ValueError("Feature documentation no longer matches encoder size")
    return rows


def response_schema():
    def obj(properties):
        return {"type":"object","properties":properties,"required":list(properties),"additionalProperties":False}
    number = {"type":"number"}
    inputs = obj({"feature":{"type":"integer"},"weight":number})
    outputs = obj({"action":{"type":"integer"},"weight":number})
    unit = obj({"index":{"type":"integer"},"bias":number,
                "inputs":{"type":"array","items":inputs},
                "outputs":{"type":"array","items":outputs},"purpose":{"type":"string"}})
    return obj({"design_summary":{"type":"string"},"limitations":{"type":"string"},
                "units":{"type":"array","items":unit},
                "output_bias":{"type":"array","items":number}})


def expand_weights(specification):
    """Mechanical sparse-to-dense copy only; never interprets policy purpose."""
    size, actions = len(network.features({})), len(network.ACTIONS)
    w1, b1 = [[0.0]*HIDDEN for _ in range(size)], [0.0]*HIDDEN
    w2 = [[0.0]*actions for _ in range(HIDDEN)]
    def value(number):
        if isinstance(number,bool) or not isinstance(number,(int,float)) or not math.isfinite(number):
            raise ValueError("Weight must be finite and numeric")
        return float(number)
    def index(number, maximum):
        if type(number) is not int or not 0 <= number < maximum:
            raise ValueError("Weight index outside network dimensions")
        return number
    def assign(entries, field, maximum, setter):
        seen = set()
        for entry in entries:
            i = index(entry[field],maximum)
            if i in seen:
                raise ValueError("Duplicate sparse weight index")
            seen.add(i)
            setter(i,value(entry["weight"]))
    seen = set()
    for unit in specification["units"]:
        j = index(unit["index"],HIDDEN)
        if j in seen:
            raise ValueError("Duplicate hidden unit")
        seen.add(j)
        b1[j] = value(unit["bias"])
        assign(unit["inputs"],"feature",size,lambda i,v:w1[i].__setitem__(j,v))
        assign(unit["outputs"],"action",actions,lambda k,v:w2[j].__setitem__(k,v))
    b2 = [value(v) for v in specification["output_bias"]]
    if len(b2) != actions:
        raise ValueError("Output bias has incorrect dimension")
    artifact = {"version":network.VERSION,"actions":list(network.ACTIONS),"method":METHOD,
                "feature_count":size,"hidden_units":HIDDEN,"parameters":size*HIDDEN+HIDDEN+HIDDEN*actions+actions,
                "training_rows":0,"gradient_steps":0,"author_environment_rollouts":0,
                "author_forward_passes":0,"initialization":"all zero; sparse assignments authored by Astra",
                "w1":w1,"b1":b1,"w2":w2,"b2":b2}
    network.LocalPolicy(artifact)  # Shape/finiteness validation, NOT inference.
    return artifact


def compact_packet(packet):
    """Authoring-only lossless projection onto the UNCHANGED encoder's fields."""
    def pick(obj, keys):
        return {k:deepcopy(obj[k]) for k in keys.split() if k in (obj or {})}
    def motion(obj):
        return pick(obj,"reason action completed made_progress measured_translation_mm measured_turn_deg requested_turn_deg sweep_coverage_deg consecutive_no_progress_attempts")
    def point(obj):
        return pick(obj,"x_mm y_mm")
    result = pick(packet, "available_actions active_waypoint active_waypoint_plan waypoint_reached_radius_mm previous_decision")
    observation = packet.get("observation") or {}
    result["observation"] = pick(observation, "directional_goal infrared touch_pressed last_motion")
    if "infrared" in result["observation"]:
        result["observation"]["infrared"] = pick(observation["infrared"],"filtered")
    if "last_motion" in result["observation"]:
        result["observation"]["last_motion"] = motion(observation["last_motion"])
    if "active_waypoint" in result:
        result["active_waypoint"] = point(result["active_waypoint"])
    if "active_waypoint_plan" in result:
        result["active_waypoint_plan"] = [point(p) for p in (result["active_waypoint_plan"] or [])]
    if "previous_decision" in result:
        result["previous_decision"] = pick(result["previous_decision"],"action")
    result["observation"]["sensors"] = pick(observation.get("sensors"), "distance_mm range_state")
    evidence = packet.get("local_map_evidence") or {}
    target = result["local_map_evidence"] = pick(evidence,
        "robot_pose directional_goal robot_footprint sensor_layout latest_route_rejection")
    if "robot_footprint" in target:
        target["robot_footprint"] = pick(target["robot_footprint"],
            "front_extent_mm rear_extent_mm left_extent_mm right_extent_mm clearance_margin_mm turn_sweep_radius_mm")
    if "sensor_layout" in target:
        target["sensor_layout"] = pick(target["sensor_layout"],"forward_offset_mm left_offset_mm")
    is_ir = "ir_survey" in evidence
    if is_ir:
        survey = evidence.get("ir_survey") or {}
        target["ir_survey"] = pick(survey,"complete sweep_degrees movement_commands_since_scan reason")
        destination, name = target["ir_survey"], "samples"
        rays = survey.get("samples", [])
    else:
        destination, name = target, "range_observations"
        rays = evidence.get(name, [])
    destination[name] = [pick(ray, "observed_from sensor_origin ray_heading_deg_in_episode heading_deg filtered_ir_proximity distance_mm settled")
                         for ray in rays[-network.RAY_COUNT:]]
    result["execution"] = pick(packet.get("execution"), "advance_step_mm forward_admission next_waypoint_motion")
    hint = result["execution"].get("next_waypoint_motion")
    if hint:
        result["execution"]["next_waypoint_motion"] = {
            **pick(hint,"action remaining_turn_deg"),
            "straight_motion_projection":pick(hint.get("straight_motion_projection"),"travel_mm waypoint_miss_mm")}
    result["history"] = [pick(item,
        "pose action result action_error motion_outcome route_interruption scan_state scan sweep_coverage_deg repeat_count last_motion route_rejection")
        for item in (packet.get("history") or [])[-4:]]
    for item in result["history"]:
        if "last_motion" in item:
            item["last_motion"] = motion(item["last_motion"])
        if "route_interruption" in item:
            item["route_interruption"] = pick(item["route_interruption"],"reason distance_mm")
        if "scan" in item:
            item["scan"] = pick(item["scan"],"state result restoration_verified all_observations_settled sweep_coverage_deg")
    if network.features(result) != network.features(packet):
        raise ValueError("Repair evidence dropped an encoded observation")
    return result


def repair_evidence(root):
    """Bounded contiguous failure windows, deduplicating rays without changing them.

    Selection is offline evidence collection, never a runtime navigation rule.
    Recorded actions are observations of the OLD policy, not target labels.
    """
    root = Path(root)
    policy = json.loads((root/"author/policy-direct.json").read_text())
    specification = json.loads((root/"author/authored-weights.json").read_text())
    reconstructed = expand_weights(specification)
    if any(policy[k] != reconstructed[k] for k in ("w1","b1","w2","b2")):
        raise ValueError("Previous authored specification does not match weights")
    source_hash = hashlib.sha256(Path(network.__file__).read_bytes()).hexdigest()
    if policy["local_policy_source_sha256"] != source_hash:
        raise ValueError("Encoder/executor policy changed since recorded experiment")
    cases, ray_table, ray_lookup, categories = [], [], {}, set()
    def find_categories(obj):
        if isinstance(obj, dict):
            for key,value in obj.items():
                if key in {"reason","result","action_error","motion_outcome","scan_state","range_state","state"} and isinstance(value,str):
                    categories.add(value)
                find_categories(value)
        elif isinstance(obj,list):
            for value in obj:
                find_categories(value)
    for path in sorted((root/"test").glob("*-local/summary.json")):
        summary = json.loads(path.read_text())
        if summary["clean"]:
            continue
        with (path.parent/"events.jsonl").open() as stream:
            events = [json.loads(line) for line in stream]
        pairs = [r for r in events if r["event"] == "decision_pair"]
        dispatches = [r for r in events if r["event"] == "policy_dispatch"]
        if len(pairs) != len(dispatches):
            raise ValueError("Recorded decision provenance is incomplete")
        def failed(row):
            p = row["packet"]
            motion = (p.get("observation") or {}).get("last_motion") or {}
            h = (p.get("history") or [{}])[-1]
            return motion.get("completed") is False or bool(h.get("route_rejection"))
        first = next((i for i,r in enumerate(pairs) if failed(r)), 0)
        indices = sorted(set(range(max(0,first-2),min(len(pairs),first+6))) | set(range(max(0,len(pairs)-4),len(pairs))))
        snapshots = []
        for i in indices:
            p = compact_packet(pairs[i]["packet"])
            find_categories(p)
            evidence = p["local_map_evidence"]
            holder, name = (evidence["ir_survey"],"samples") if "ir_survey" in evidence else (evidence,"range_observations")
            ids = []
            for ray in holder.pop(name):
                encoded = json.dumps(ray,sort_keys=True)
                if encoded not in ray_lookup:
                    ray_lookup[encoded] = len(ray_table)
                    ray_table.append(ray)
                ids.append(ray_lookup[encoded])
            holder[name+"_ids"] = ids
            snapshots.append({"step":i+1,"packet":p,"old_source":dispatches[i]["source"],
                "old_selection":dispatches[i]["selection"],
                "old_executed_action":pairs[i]["decision"]["action"]})
        cases.append({"recording":path.parent.name,
            "recording_sha256":hashlib.sha256((path.parent/"events.jsonl").read_bytes()).hexdigest(),
            "robot":summary["robot"],"outcome":{k:summary.get(k) for k in ("clean","goal_reached","blocked_moves","decisions")},
            "snapshots":snapshots})
    if not cases:
        raise ValueError("No recorded failed runs to inspect")
    return {"previous_specification":specification,
        "previous_policy_sha256":hashlib.sha256((root/"author/policy-direct.json").read_bytes()).hexdigest(),
        "ray_table":ray_table,"cases":cases,
        "representation":"Expand samples_ids/range_observations_ids through ray_table in the same order before applying features(). Each projected snapshot was checked to encode identically to its original packet. Indices are data references, never a policy feature. Steps omitted between windows are explicitly absent, not consecutive.",
        "category_codes":{name:network._category(name) for name in sorted(categories)},
        "category_note":"These are existing encoder SHA256 codes, mechanically computed for observed strings, not new features or assigned responses. No training labels, gradients, simulator obstacle geometry, or target actions are supplied."}


def author_request(repair=None):
    from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
    from robot_agent.blast_navigation_calibration import BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
    # Base contract contains only interface/source definitions.
    functions = (network._number,network._category,network._boolean,network._motion,
                 network.features,network.LocalPolicy.predict,network.LocalPlanner.decide)
    contract = {
        "task":"Write numeric weights from scratch for local robot-navigation decisions on a retained route.",
        "architecture":{"features":len(network.features({})),"hidden":HIDDEN,"activation":"tanh",
                        "outputs":list(network.ACTIONS),"selection":"argmax over available actions plus REPLAN"},
        "dimensions":"W1[input][hidden], h=tanh(x W1+b1), logits=h W2+b2; no other learned layers or state.",
        "encoding":"All indices are zero-based. Numeric scalings clamp to [-4,4] except log1p counters and trigonometric values. Boolean pairs are [is True,is False], unknown is [0,0]. Category vectors are SHA256 bits, not ordered magnitudes. History is left-padded and history[3] is most recent.",
        "layout":feature_layout(),
        "source":"\n\n".join(inspect.getsource(f) for f in functions),
        "robots":{
            "EV3":{"body":EV3RSTORMProfile().hazard_calibration.robot_footprint.planner_view(),
                   "range":"Head-mounted IR, unitless 0..100; lower is stronger return, not metric distance. Material/angle/height affect it. 100 may be weak/no return. No measured numeric head mounting offset.","advance_step_mm":52},
            "Blast":{"body":BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.robot_footprint.planner_view(),
                     "range":"Ultrasound millimetres from sensor face, 110 mm forward and 80 mm left of drive origin. Missing/no echo is neither a confirmed obstacle nor confirmed clearance.","advance_step_mm":45}},
        "execution_contract":[
            "Astra owns goal interpretation and route creation/revision. This network selects among the eight actions; REPLAN delegates the unchanged current context to Astra. No action outside the listed interface is possible.",
            "Without an active route the existing dispatcher calls Astra. Completion is checked separately by the existing goal contract. Neither is changed in this experiment.",
            "FOLLOW_WAYPOINT preserves route geometry and executes its next turn/forward motion, checking observations between pulses. The route executor rejects non-cardinal legs and, for Blast, known echo-clearance intersections. EV3 IR does not provide a metric obstacle map.",
            "ADVANCE and REVERSE move one robot-specific pulse. TURN_LEFT_90 and TURN_RIGHT_90 turn the body; FOLLOW_WAYPOINT uses the measured remaining turn to its waypoint axis. These primitives do not author new waypoints.",
            "SCAN_FRONT_ARC observes a limited arc by turning the body and attempting to return. OBSERVE samples without navigation movement. These actions do not replace a retained route.",
            "Execution returns measured progress/completion, not requested progress. It can interrupt motion, refuse a route, or return incomplete/missing sensor evidence. A remembered last_motion can predate the latest action rejection; history contains action outcomes.",
            "Historical rejection/evidence may remain after route/pose changes. Observations are partial and historical, not a complete world map. A clear sensor ray does not prove whole-body passage clearance.",
            "All positions/waypoints refer to the drive origin. The neural vector rotates coordinates into the current body frame. Robot body asymmetry, dimensions, primitive sizes and sensor units differ and are encoded.",
        ],
        "objective":"Reach goals without simulator collision stops. Among safe goal-reaching behaviours, reduce expensive Astra calls and unnecessary scans/actions. Assess uncertainty and feedback using the supplied interface; no route or tactical side is prescribed.",
        "author_protocol":"One completed candidate, with no training data, examples, old weights, gradients, tools, environment interaction, forward passes, evaluation layouts or result feedback. Reason from the supplied specification only. Do not request files or propose a training loop. All omitted weights start and remain zero. You author every nonzero weight and bias; the host only copies them into fixed matrices. Inference cannot execute your prose or code.",
        "response_format":"Give units with unique index 0..31; each lists unique input feature indices and output action indices with explicit numeric weights, plus its bias. Unlisted weights and units are zero. Give all 8 output biases. Purpose/design_summary are concise descriptions, not code and not detailed reasoning. Describe limitations honestly. Do not output a conventional rule function or wrapper.",
    }
    if repair is not None:
        contract.update(task="Correct the previous directly authored numeric weights to use positioned sensor-ray geometry and avoid resuming an unchanged failed route after a successful recovery primitive.",
            repair_evidence=repair,
            author_protocol="One corrected numeric candidate. Inspect the supplied old weights and recorded observation-only failure windows. No training, optimization loop, tools, candidate forward passes, environment rollouts, hidden obstacle layouts, or new runtime rules. The fixed 2468->32 tanh->8 network, encoder, dispatcher and route executor must remain unchanged. Output a COMPLETE replacement specification: omitted weights become zero, not inherited. No result from the upcoming rerun is available. This is a recorded-feedback repair, not zero-shot or independent held-out evaluation.",
            repair_objective="Make spatial evidence and robot dimensions genuinely affect decisions, rather than merely attaching inert weights to rays. Avoid confusing completed scans/backing with a repaired route. Recent failure history remains relevant after a primitive succeeds. Unknown/no-return must not imply either obstacle or certified free space. Preserve useful local execution, not unconditional REPLAN or endless scans. Choose the numeric solution yourself; do not add a wrapper or prescribe new execution rules. State honestly what this fixed architecture cannot represent.")
    return {"model":MODEL,"reasoning_effort":"high","max_tokens":32768,
            "messages":[{"role":"system","content":"You are authoring a neural policy's numeric weights directly, not training it. Follow the experiment protocol exactly."},
                        {"role":"user","content":json.dumps(contract,ensure_ascii=False)}],
            "response_format":{"type":"json_schema","json_schema":{
                "name":"direct_navigation_weights","strict":True,"schema":response_schema()}}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--repair-from",type=Path,help="Prior direct-weight experiment; inspect recorded failures without training")
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    repair = repair_evidence(args.repair_from) if args.repair_from else None
    request = author_request(repair)
    (args.output/"request.json").write_text(json.dumps(request,indent=2))
    enable_navigation_diagnostics(args.output/"diagnostics.jsonl")
    start = time.monotonic()
    raw = CodexSubscription().complete(request,timeout=600 if repair else 240,maximum=2*1024*1024)
    response = json.loads(raw)
    (args.output/"response.json").write_bytes(raw)
    specification = json.loads(response["choices"][0]["message"]["content"])
    (args.output/"authored-weights.json").write_text(json.dumps(specification,indent=2))
    artifact = expand_weights(specification)
    if repair:
        artifact.update(method=REPAIR_METHOD, parent_policy_sha256=repair["previous_policy_sha256"],
                        author_recorded_cases=len(repair["cases"]))
    source_hash = hashlib.sha256(Path(network.__file__).read_bytes()).hexdigest()
    artifact.update(author_model=response["resolved_model"],author_reasoning_effort="high",
                    local_policy_source_sha256=source_hash,
                    author_specification_sha256=hashlib.sha256(json.dumps(specification,sort_keys=True).encode()).hexdigest())
    path = args.output/"policy-direct.json"
    path.write_text(json.dumps(artifact,separators=(",",":")))
    provenance = {"method":artifact["method"],"resolved_model":response["resolved_model"],"usage":response.get("usage"),
        "seconds":round(time.monotonic()-start,2),"policy_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
        "request_sha256":hashlib.sha256((args.output/"request.json").read_bytes()).hexdigest(),
        "local_policy_source_sha256":source_hash,"training_rows":0,"gradient_steps":0,
        "author_forward_passes":0,"author_environment_rollouts":0,"completed_candidates":1,
        "hidden_units_assigned":len(specification["units"]),
        "nonzero_parameters":sum(v!=0 for key in ("w1","w2") for row in artifact[key] for v in row)
            +sum(v!=0 for key in ("b1","b2") for v in artifact[key]),
        "design_summary":specification["design_summary"],"author_limitations":specification["limitations"]}
    if repair:
        provenance.update(parent_policy_sha256=repair["previous_policy_sha256"],
            recorded_cases=[{"recording":c["recording"],"sha256":c["recording_sha256"],
                             "steps":[s["step"] for s in c["snapshots"]]} for c in repair["cases"]],
            repeated_evaluation_not_independent=True)
    (args.output/"provenance.json").write_text(json.dumps(provenance,indent=2))
    print(json.dumps(provenance),flush=True)


if __name__ == "__main__":
    main()
