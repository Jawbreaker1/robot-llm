"""Isolated learned-local/Astra comparison on the production simulation adapters.

PYTHONPATH=.:src python scripts/local_navigation_experiment.py --help
No physical transports, UI setting or production default are changed here.

Reproduce (run from the repository with PYTHONPATH=.:src):
  python scripts/local_navigation_experiment.py batch --split train --output OUT/train
  python scripts/local_navigation_experiment.py batch --split recovery --output OUT/recovery --decisions 48
  python scripts/local_navigation_experiment.py fit --data OUT/train --recovery-data OUT/recovery --weights OUT/policy.json
  python scripts/local_navigation_experiment.py batch --split test --output OUT/test --weights OUT/policy.json --paired
  python scripts/local_navigation_experiment.py report --output OUT/test

Training requires experiments/requirements-local-navigation.txt. Optional
--recorded-box-root accepts the explicit four corrected October box runs.
Freeze the model before starting held-out tests; do not fit on their results.
All trials use the existing production execution adapters and Astra high.
One controlled comparison, not an assertion that behavioural cloning can
outperform its teacher. It cannot learn skills missing from the demonstrations.
Recovery collection executes Astra through complete recoveries with finite
simulator faults. An optional recorded TRAINING prefix reproduces a real route
rejection before Astra takes over. Neither prefix actions nor deliberately
corrupted commands enter training. Only collision-free goal-reaching runs do.
The audit separately requires observed faults before counting a recovery: an
unobserved fault can supply normal continuation data, not recovery coverage.
Failed trials remain in the audit. Test runs never inject training faults.
"""

import argparse
from collections import Counter
from contextlib import nullcontext
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from experiments.local_navigation import LocalPlanner, LocalPolicy, features, target, train
from experiments.navigation_recovery_data import FAULTS, RecoveryFault
from experiments.navigation_student_recovery import StudentRecovery, PROTOCOL as STUDENT_PROTOCOL
from robot_agent.blast_navigation_simulation import run_blast_gemma_scenario
from robot_agent.lm_studio_controller_action import (LMStudioControllerActionPlanner, ControllerActionContext,
    ControllerActionDecision, ControllerActionPlannerResult)
from robot_agent.navigation_diagnostics import enable_navigation_diagnostics
from robot_agent.navigation_simulation_scenarios import blast_box_front, ev3_validation_scenarios
from robot_agent.multi_robot_navigation_simulator import RectangleObstacle

MODEL = "codex/gpt-6-astra"
REPO = Path(__file__).resolve().parents[1]
TRAIN = ("train-open", "train-left", "train-right")
RECOVERY = ("train-right-corner", "train-left-corner", "train-staggered")
TEST = ("box", "bend", "many-boxes", "maze")
ATTENTION_HELD_OUT = ("novel-offset-box", "novel-chicane")
RECOVERY_TEST = ("test-box-pair", "test-dogleg", "test-islands", "test-culdesac")
STUDENT_TEST = ("student-test-offset-pair", "student-test-chicane", "student-test-islands", "student-test-pocket")
DENSE_TEST = ("dense-test-box-sides", "dense-test-corridor", "dense-test-islands", "dense-test-pocket")
MAP_TRAIN = tuple(f"map-train-{i}" for i in range(8))
MAP_DEV = ("map-dev-left", "map-dev-right")
MAP_TEST = ("map-test-box", "map-test-corridor", "map-test-islands")
LEARNER_TRAIN = ("learner-train-flanks", "learner-train-flanks-offset",
                 "learner-train-left-pair", "learner-train-right-pair")
LEARNER_TEST = ("learner-test-box", "learner-test-corridor", "learner-test-islands")
PASSAGE_TEST = ("passage-test-box", "passage-test-corridor", "passage-test-islands")

# Environment-only fixtures. No obstacle geometry is put in the mission prompt.
MAP_LAYOUTS = {
    "passage-test-box": (1350, ((450,-150,690,230),(980,-570,1180,-330))),
    "passage-test-corridor": (1850, ((-350,810,2400,970),(-350,-970,2400,-810),
                                     (610,-810,800,-120),(1290,200,1480,810))),
    "passage-test-islands": (1800, ((420,-210,630,130),(970,120,1160,450),(1430,-450,1610,-130))),
    "learner-train-flanks": (1350, ((380,-180,600,180),(820,300,1010,530),(820,-530,1010,-300))),
    "learner-train-flanks-offset": (1450, ((420,-220,650,140),(950,280,1130,520),(880,-580,1070,-340))),
    "learner-train-left-pair": (1400, ((340,-240,570,80),(870,130,1060,440),(-140,-650,100,-420))),
    "learner-train-right-pair": (1450, ((380,-100,610,250),(920,-470,1120,-150),(-170,430,110,660))),
    "learner-test-box": (1300, ((410,-210,650,170),(940,-610,1150,-370))),
    "learner-test-corridor": (1800, ((-350,790,2350,950),(-350,-950,2350,-790),
                                    (580,70,760,790),(1240,-790,1420,-170))),
    "learner-test-islands": (1750, ((390,-170,590,170),(930,-430,1110,-100),(1390,150,1560,430))),
    "map-train-0": (800, ()),
    "map-train-1": (1000, ((300,-180,500,130),)),
    "map-train-2": (1100, ((360,-120,570,220),(-100,-560,100,-340))),
    "map-train-3": (1100, ((340,-250,540,100),(-80,350,130,540))),
    "map-train-4": (1400, ((-300,650,1800,800),(-300,-800,1800,-650),(540,50,720,650))),
    "map-train-5": (1400, ((-300,680,1800,840),(-300,-840,1800,-680),(510,-680,690,-80))),
    "map-train-6": (1500, ((360,-230,540,90),(980,60,1160,350))),
    "map-train-7": (1500, ((350,20,550,310),(960,-400,1150,-70))),
    "map-dev-left": (1200, ((410,-170,640,160),(-120,-650,180,-440))),
    "map-dev-right": (1300, ((440,-210,650,140),(-160,420,130,620))),
    "map-test-box": (1200, ((370,-200,610,200),(880,390,1050,600))),
    "map-test-corridor": (1750, ((-400,760,2300,920),(-400,-920,2300,-760),
                                (520,-760,700,-80),(1190,160,1360,760))),
    "map-test-islands": (1700, ((340,-130,530,190),(850,-390,1050,-70),(1330,110,1510,380))),
}


def scenario(robot, name):
    base = {s.scenario_id: s for s in ev3_validation_scenarios()}
    if name in MAP_LAYOUTS:
        scene = base['open']
        distance, boxes = MAP_LAYOUTS[name]
        scene = replace(scene, scenario_id=name, bounds=(-900,-1200,2600,1200),
            obstacles=tuple(RectangleObstacle(f'obstacle-{i}',*box) for i,box in enumerate(boxes)),
            goals=(replace(scene.goals[0],x_mm=distance),))
    elif name in DENSE_TEST:
        scene=base['open']
        layouts={
            'dense-test-box-sides':((330,-190,580,210),(-180,420,150,600)),
            'dense-test-corridor':((-500,780,2550,950),(-500,-950,2550,-780),
                                   (520,-780,710,-20),(1240,120,1450,780)),
            'dense-test-islands':((360,-60,570,290),(910,-480,1120,-100),(1430,170,1640,520)),
            'dense-test-pocket':((420,350,1360,510),(420,-510,1360,-350),(1170,-510,1360,510)),
        }
        distances={'dense-test-box-sides':1400,'dense-test-corridor':1950,'dense-test-islands':2000,'dense-test-pocket':1950}
        scene=replace(scene,scenario_id=name,bounds=(-950,-1250,2900,1250),
            obstacles=tuple(RectangleObstacle(f'obstacle-{i}',*b) for i,b in enumerate(layouts[name])),
            goals=(replace(scene.goals[0],x_mm=distances[name]),))
    elif name in STUDENT_TEST:
        scene = base["open"]
        layouts = {
            "student-test-offset-pair": ((380,-220,660,180), (1040,260,1220,600)),
            "student-test-chicane": ((-500,740,2500,900), (-500,-900,2500,-740),
                                     (540,0,730,740), (1310,-740,1500,-100)),
            "student-test-islands": ((400,-280,620,90), (920,40,1130,430), (1450,-510,1650,-160)),
            "student-test-pocket": ((350,320,1320,480), (350,-480,1320,-320), (1120,-480,1320,480)),
        }
        scene = replace(scene, scenario_id=name, bounds=(-900,-1200,2800,1200),
            obstacles=tuple(RectangleObstacle(f"obstacle-{i}",*b) for i,b in enumerate(layouts[name])),
            goals=(replace(scene.goals[0],x_mm=1650 if name=='student-test-offset-pair' else 2000),))
    elif name in RECOVERY_TEST:
        # Frozen before recovery training. These are worlds, never routes or
        # planner instructions; no test trajectory is admitted to the trainer.
        scene = base["open"]
        layouts = {
            "test-box-pair": ((360,-160,600,220), (950,-520,1140,-170)),
            "test-dogleg": ((-400,650,2200,820), (-400,-820,2200,-650),
                            (450,-650,620,40), (1120,80,1290,650)),
            "test-islands": ((340,-100,510,240), (760,-510,990,-120), (1220,40,1450,400)),
            "test-culdesac": ((300,260,1170,420), (300,-420,1170,-260), (1000,-420,1170,420)),
        }
        distance = 1550 if name == "test-box-pair" else 1850
        scene = replace(scene, scenario_id=name, bounds=(-700,-1100,2600,1100),
                        obstacles=tuple(RectangleObstacle(f"obstacle-{i}",*box)
                                        for i,box in enumerate(layouts[name])),
                        goals=(replace(scene.goals[0],x_mm=distance),))
    elif name in ATTENTION_HELD_OUT:
        # Defined independently of the author, never put in its input. Worlds
        # only: neither the model nor Astra receives obstacle coordinates.
        scene = base["open"]
        if name == "novel-offset-box":
            obstacles = (RectangleObstacle("wide-offset", 390, -260, 700, 170),
                         RectangleObstacle("side-pillar", 40, -700, 230, -460))
            distance = 1300
        else:
            obstacles = (RectangleObstacle("upper-wall", -400, 720, 2400, 900),
                         RectangleObstacle("lower-wall", -400, -900, 2400, -720),
                         RectangleObstacle("first-baffle", 380, -180, 550, 720),
                         RectangleObstacle("second-baffle", 1080, -720, 1250, 180))
            distance = 1900
        scene = replace(scene, scenario_id=name, bounds=(-700, -1100, 2600, 1100),
                        obstacles=obstacles, goals=(replace(scene.goals[0], x_mm=distance),))
    elif name in RECOVERY:
        # New training worlds, not the held-out bend/many-boxes/maze layouts.
        scene = base["open"]
        if name == "train-staggered":
            obstacles = (RectangleObstacle("first", 280, -240, 450, 100),
                         RectangleObstacle("second", 720, 30, 900, 480),
                         RectangleObstacle("third", 1090, -550, 1260, -200))
            scene = replace(scene, goals=(replace(scene.goals[0], x_mm=1500),))
        else:
            obstacles = (RectangleObstacle("corner", 320, -120, 530, 420),
                         RectangleObstacle("lower-wall", -150, -720, 900, -560))
            if name == "train-left-corner":
                obstacles = tuple(RectangleObstacle(o.obstacle_id, o.min_x_mm, -o.max_y_mm,
                    o.max_x_mm, -o.min_y_mm) for o in obstacles)
        scene = replace(scene, scenario_id=name, obstacles=obstacles)
    elif name in TRAIN or name == "box":
        scene = base["open"]
        if name != "train-open":
            # Only worlds, not routes. The held-out box is wider and further away.
            box = (RectangleObstacle("box", 310, -140, 550, 140) if name == "box" else
                   RectangleObstacle("box", 250, -90, 430, 90))
            side = (-1 if name == "train-right" else 1)
            other = RectangleObstacle("side", -100, min(side * 260, side * 460),
                                      120, max(side * 260, side * 460))
            scene = replace(scene, obstacles=(box,) if name == "box" else (box, other))
        scene = replace(scene, scenario_id=name)
    else:
        scene = base[name]
    if robot == "blast":
        body = replace(blast_box_front().robots[0], pose=scene.robots[0].pose)
        scene = replace(scene, robots=(body,), goals=(replace(scene.goals[0], robot_id=robot),))
    return scene


def run_ev3(scene, planner, maximum, emit, *, speech_runtime_factory=None):
    from scripts.validate_ev3_navigation import ValidatedTransport
    from robot_agent.ev3_navigation_body import EV3NavigationBody
    from robot_agent.ev3_episode_adapter import EV3EpisodeAdapter
    from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
    from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge
    from robot_agent.robot_control_contract import RobotControlSettings, RobotEpisodeStart
    from robot_agent.robot_control_service import RobotEpisodeContext
    profile, world, stop = EV3RSTORMProfile(), scene.build(), threading.Event()
    count = 0

    def publish(event):
        nonlocal count
        emit({"event": "execution", "payload": {k:v for k,v in event.items() if k != "context"}})
        if event["event"] == "result":
            count += 1
            if count >= maximum and event["action"] != "COMPLETE":
                stop.set()

    distance = scene.goals[0].x_mm - scene.robots[0].pose.x_mm
    bridge = PhysicalSpatialMapBridge(robot_id=profile.descriptor.robot_id,
                                     controller_instance_id=profile.descriptor.controller_id)
    adapter = EV3EpisodeAdapter(
        body_factory=lambda **kwargs: EV3NavigationBody(lambda: ValidatedTransport(
            world, world_robot_id="ev3", odometry=profile.odometry_calibration,
            ir_response=((0, 0), (200, 44), (300, 51), (1000, 100))),
            calibration=profile.odometry_calibration, **kwargs),
        planner_factory=lambda model: planner, descriptor=profile.descriptor,
        footprint=profile.hazard_calibration.robot_footprint, spatial_map_provider=bridge,
        speech_runtime_factory=speech_runtime_factory, controller_lease=threading.Lock(),
        minimum_forward_progress_mm=distance, event_sink=publish)
    context = RobotEpisodeContext("local-policy-experiment", RobotEpisodeStart(
        f"Reach the goal {distance / 10:g} cm straight ahead of your starting position.", "en", "simulation", 1),
        RobotControlSettings(model=MODEL, speech_enabled=speech_runtime_factory is not None),
        stop, threading.Event(), lambda update: None)
    outcome = adapter.run(context)
    pose, goal = world.pose("ev3"), scene.goals[0]
    return {"completed": outcome.completed, "goal_reached": world.goal_reached("ev3"),
            "blocked_moves": sum(e.kind == "blocked" for e in world.events),
            "distance_to_goal_mm": round(math.hypot(pose.x_mm-goal.x_mm, pose.y_mm-goal.y_mm)),
            "terminal_reason": outcome.terminal_reason, "execution_steps": count,
            "final_pose": asdict(pose)}


def recovery_prefix(path, *, robot, scene, source_hashes):
    """Replay a recorded TRAINING prefix up to its first actual rejection.

    No prefix action is a training label. All recovery decisions come from new
    Astra calls, after the existing executor reproduces the rejected route.
    """
    with path.open() as stream:
        events = [json.loads(line) for line in stream]
    config = events[0]
    if (config.get("scene") not in TRAIN + RECOVERY or config.get("scene") != scene
            or config.get("robot") != robot):
        raise ValueError("Recovery prefix must match this robot and a training world")
    previous = json.loads(path.with_name("source-hashes.json").read_text())
    if any(previous.get(k) != v for k,v in source_hashes.items() if k.startswith("src/")):
        raise ValueError("Recovery prefix production code no longer matches")
    rows = []
    for event in events:
        if event["event"] != "decision_pair":
            continue
        p = event["packet"]
        if (p["local_map_evidence"].get("latest_route_rejection")
                or any(h.get("route_rejection") for h in p.get("history",[]))):
            if not rows:
                raise ValueError("Recovery prefix must start before the rejection")
            return rows
        rows.append(event)
    raise ValueError("Recovery prefix has no recorded route rejection")


def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    enable_navigation_diagnostics(output / "diagnostics.jsonl")
    collecting = getattr(args, "collect", False)
    student_recovery = getattr(args, "student_recovery", False)
    if student_recovery and (not collecting or not args.weights or
                            getattr(args,'fault','none') != 'none' or getattr(args,'replay_prefix',None)):
        raise ValueError('Student recovery requires training collection, weights, and no injected/replayed faults')
    if collecting and ((args.weights and not student_recovery) or args.scene not in TRAIN + RECOVERY + MAP_TRAIN + MAP_DEV + LEARNER_TRAIN):
        raise ValueError("Recovery collection requires Astra only and a training world")
    if not collecting and getattr(args, "fault", "none") != "none":
        raise ValueError("Fault injection is only permitted in recovery collection")
    replay_path = getattr(args,"replay_prefix",None)
    if replay_path and (not collecting or getattr(args,"fault","none") != "none"):
        raise ValueError("Prefix replay requires collection without additional faults")
    mode = "collect" if collecting else "local" if args.weights else "astra"
    scene = scenario(args.robot, args.scene)
    source_hashes = {str(p.relative_to(REPO)):hashlib.sha256(p.read_bytes()).hexdigest()
                     for directory in (REPO/"src/robot_agent", REPO/"experiments", REPO/"ev3")
                     for p in directory.glob("*.py")}
    source_hashes["config/ev3rstorm.json"] = hashlib.sha256((REPO/"config/ev3rstorm.json").read_bytes()).hexdigest()
    source_hashes["scripts/local_navigation_experiment.py"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (output/"source-hashes.json").write_text(json.dumps(source_hashes, indent=2))
    prefix = recovery_prefix(replay_path, robot=args.robot, scene=args.scene,
                             source_hashes=source_hashes) if replay_path else []
    records, dispatches, speech_counts = [], [], Counter()
    started = time.monotonic()
    with (output/"events.jsonl").open("w") as stream:
        def emit(event):
            stream.write(json.dumps(event, ensure_ascii=False) + "\n")
            stream.flush()
            if event["event"] == "policy_dispatch":
                dispatches.append(event)
            if "speech" in event["event"] or "commentary" in event["event"]:
                speech_counts[event["event"]] += 1
        emit({"event": "configuration", "mode": mode, "robot": args.robot,
              "scene": args.scene, "model": MODEL, "reasoning": "high", "world": asdict(scene),
              "max_decisions": args.decisions, "max_output_tokens":8192, "planner_timeout_seconds":60,
              "commentary":bool(getattr(args,"commentary",False)), "audio_playback":"simulated_only",
              "commentary_reasoning":"low",
              "model_sha256": hashlib.sha256(args.weights.read_bytes()).hexdigest() if args.weights else None,
              "collection_protocol":STUDENT_PROTOCOL if student_recovery else "complete-astra-recovery-v1" if collecting else None,
              "fault":getattr(args,"fault","none"),
              "fault_missing_reads":getattr(args,"fault_reads",32),
              "fault_stalled_attempts":getattr(args,"fault_stalls",1),
              "replay_prefix":str(replay_path) if replay_path else None,
              "replay_prefix_sha256":hashlib.sha256(replay_path.read_bytes()).hexdigest() if replay_path else None,
              "replay_prefix_decisions":len(prefix),
              "limitations": "Approximate white-surface IR, ideal range rays/encoders. No real robots. No hidden world geometry in policy input."})
        from robot_agent.blast_personality import BLAST_PERSONA_BY_LOCALE, EV3_PERSONA_BY_LOCALE
        from robot_agent.social_expression import EV3_GESTURES
        teacher = LMStudioControllerActionPlanner(model=MODEL, reasoning_effort="high",
            max_output_tokens=8192, timeout_seconds=60,
            social_expressions=True,
            **({"social_gestures":EV3_GESTURES, "social_faces":("neutral",)} if args.robot == "ev3" else {}),
            utterance_persona_by_locale=EV3_PERSONA_BY_LOCALE if args.robot == "ev3" else BLAST_PERSONA_BY_LOCALE)
        policy = LocalPolicy.load(args.weights) if args.weights else None
        selected = LocalPlanner(teacher, policy, emit) if policy else teacher
        if student_recovery:
            selected = StudentRecovery(teacher, policy, emit)
        fault = RecoveryFault(getattr(args,"fault","none"), emit,
                              missing_reads=getattr(args,"fault_reads",32),
                              stalled_attempts=getattr(args,"fault_stalls",1))

        class RecordingPlanner:
            def comment(self, packet):
                emit({"event": "commentary_packet", "packet": packet})
                text = teacher.comment(packet)
                emit({"event": "commentary_generated", "text": text})
                return text

            def decide(self, context):
                began = time.perf_counter()
                if student_recovery:
                    result = selected.decide(context)
                elif len(records) < len(prefix):
                    recorded = prefix[len(records)]
                    if features(context.to_dict()) != features(recorded["packet"]):
                        raise ValueError("Recorded recovery prefix state did not reproduce")
                    result = ControllerActionPlannerResult(ControllerActionDecision(**recorded["decision"]),0)
                    emit({"event":"replayed_prefix_decision", "index":len(records),
                          "teacher_label_excluded":True})
                elif collecting:
                    observed = fault.observe(context.to_dict())
                    if prefix and len(records) == len(prefix) and "route_rejected" not in observed:
                        raise ValueError("Recorded route rejection did not reproduce")
                    result = teacher.decide(context)
                    fault.before_action(result.decision.action)
                    transmitted = fault.transmitted_decision(result.decision, context.to_dict())
                    if transmitted is result.decision:
                        emit({"event":"teacher_label", "packet":context.to_dict(),
                              "decision":asdict(result.decision), "feedback":observed})
                    else:
                        emit({"event":"teacher_command_corrupted", "packet":context.to_dict(),
                              "teacher_decision":asdict(result.decision),
                              "transmitted_decision":asdict(transmitted)})
                        result = replace(result, decision=transmitted)
                else:
                    result = selected.decide(context)
                record = {"event": "decision_pair", "packet": context.to_dict(),
                          "decision": asdict(result.decision), "elapsed_ms": (time.perf_counter()-began)*1000}
                records.append(record)
                emit(record)
                if not args.weights:
                    emit({"event": "policy_dispatch", "source": "replay" if len(records) <= len(prefix) else "astra", "selection": result.decision.action,
                          "elapsed_ms": record["elapsed_ms"]})
                return result
        try:
            planner = RecordingPlanner()
            speech_factory = None
            if getattr(args, "commentary", False):
                from robot_agent.robot_speech_runtime import RobotSpeechRuntime
                def speech_factory(*, event_sink):
                    def speaker(text, locale, cancel, *, expression=None):
                        emit({"event": "simulated_speech", "text": text, "locale": locale,
                              "expression": expression})
                    def speech_event(event):
                        emit(dict(event))
                        event_sink(event)
                    return RobotSpeechRuntime(speaker=speaker, event_sink=speech_event)
            with selected if student_recovery else fault if collecting else nullcontext():
                if args.robot == "ev3":
                    result = run_ev3(scene, planner, args.decisions, emit,
                                     speech_runtime_factory=speech_factory)
                else:
                    distance = scene.goals[0].x_mm-scene.robots[0].pose.x_mm
                    result = run_blast_gemma_scenario(scene, model=MODEL, max_decisions=args.decisions,
                        planner_factory=lambda model: planner, minimum_forward_progress_mm=distance,
                        speech_runtime_factory=speech_factory,
                        goal=f"Reach the goal {distance / 10:g} cm straight ahead of your starting position.")
                    result = {k:result[k] for k in ("completed", "goal_reached", "blocked_moves", "distance_to_goal_mm",
                               "terminal_reason", "error_type", "error_message", "commands", "scans", "final_pose")}
        except Exception as error:
            result = {"completed": False, "goal_reached": False, "blocked_moves": None,
                      "error_type": type(error).__name__, "error_message": str(error)[:240]}
        result.update({"robot": args.robot, "scene": args.scene, "mode": mode,
                       "decisions": len(records), "wall_seconds": round(time.monotonic()-started, 2),
                       "astra_calls": sum(d["source"] == "astra" for d in dispatches),
                       "local_calls": sum(d["source"] == "local" for d in dispatches),
                       "goal_contract_calls": sum(d["source"] == "goal_contract" for d in dispatches),
                       "local_decision_ms": [d["elapsed_ms"] for d in dispatches if d["source"] == "local"],
                       "astra_decision_ms": [d["elapsed_ms"] for d in dispatches if d["source"] == "astra"],
                       "decision_seconds": sum(d["elapsed_ms"] for d in dispatches)/1000,
                       "action_counts": dict(Counter(r["decision"]["action"] for r in records))})
        result["speech_counts"] = dict(speech_counts)
        result["clean"] = bool(result["completed"] and result["goal_reached"] and
                               result["blocked_moves"] == 0 and not result.get("error_type"))
        if student_recovery:
            result.update(selected.summary(result))
        elif collecting:
            result.update(collection_protocol="complete-astra-recovery-v1", fault=fault.kind,
                          fault_triggered=fault.triggered, observed_feedback=sorted(fault.feedback),
                          recovery_complete=result["clean"] and fault.observed)
        emit({"event": "summary", **result})
    (output/"summary.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)


def batch(args):
    scenes = TRAIN if args.split == "train" else ("train-right","train-left") if args.split == "recovery" else TEST
    if args.paired and (args.split != "test" or not args.weights):
        raise ValueError("Paired evaluation needs a frozen model and the test split")
    if args.split == "recovery" and args.weights:
        raise ValueError("Recovery collection now uses only Astra, not mixed student actions")
    modes = (("collect",) if args.split == "recovery" else
             ("astra", "local") if args.paired else (("local",) if args.weights else ("astra",)))
    faults = FAULTS if args.split == "recovery" else ("none",)
    cases = [(r, s, m, f) for s in scenes for f in faults for r in ("ev3", "blast") for m in modes]
    args.output.mkdir(parents=True, exist_ok=True)
    def one(case):
        robot, scene, mode, fault = case
        name = f"{robot}-{scene}-{mode}" + (f"-{fault}" if mode == "collect" else "")
        cmd = [sys.executable, str(Path(__file__).resolve()), "run", "--robot", robot, "--scene", scene,
               "--output", str(args.output/name), "--decisions", str(args.decisions)]
        if mode == "local":
            cmd += ["--weights", str(args.weights.resolve())]
        if mode == "collect":
            cmd += ["--collect", "--fault", fault]
        print(json.dumps({"run": name, "state": "started"}), flush=True)
        with (args.output/(name+".log")).open("w") as log:
            try:
                outcome = subprocess.run(cmd, cwd=REPO, env=dict(os.environ, PYTHONPATH=".:src", PYTHONDONTWRITEBYTECODE="1"),
                                         stdout=log, stderr=log, timeout=1100)
                status = outcome.returncode
            except subprocess.TimeoutExpired:
                status = "harness_timeout"
        print(json.dumps({"run": name, "state": "finished", "exit": status}), flush=True)
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(one, cases))


def fit(args):
    runs, excluded = [], []
    for folder in sorted(args.data.iterdir()):
        if not folder.is_dir() or not (folder/"summary.json").exists():
            continue
        summary = json.loads((folder/"summary.json").read_text())
        if summary["scene"] not in TRAIN or summary["mode"] != "astra":
            raise ValueError("Test/local runs are not permitted in training data")
        if not summary["clean"]:
            excluded.append(folder.name)
            continue
        with (folder/"events.jsonl").open() as stream:
            rows = [(r["packet"],r["decision"]) for line in stream
                    if (r:=json.loads(line))["event"] == "decision_pair" and target(r["packet"],r["decision"]) is not None]
        runs.append((folder.name, rows))
    if args.recorded_box_root:
        # Explicit, previously recorded simple-box demonstrations only. Never
        # discover/glob the corridor, maze or many-boxes evaluation archives.
        audit = json.loads((args.recorded_box_root/"validation.json").read_text())
        for name in ("corrected-ev3-1", "corrected-blast-1", "corrected-ev3-2", "corrected-blast-2"):
            summary = next(r["result"] for r in audit["runs"] if r["run"] == name)
            blocks = len(summary["physics_events"]) if "physics_events" in summary else summary["blocked_moves"]
            reached = summary.get("world_goal_reached", summary.get("goal_reached"))
            if not summary["completed"] or not reached or blocks:
                excluded.append(name)
                continue
            packet, rows = None, []
            for line in (args.recorded_box_root/name/"diagnostics.jsonl").open():
                event = json.loads(line)
                if event["event"] == "planner_request":
                    packet = json.loads(event["request"]["messages"][-1]["content"])
                elif event["event"] == "planner_response" and packet:
                    decision = json.loads(json.loads(event["raw_response"])["choices"][0]["message"]["content"])
                    if target(packet, decision) is not None:
                        rows.append((packet, decision))
                    packet = None
            runs.append((name, rows))
    recovery_roots = getattr(args, "recovery_data", None) or []
    if isinstance(recovery_roots, Path):
        recovery_roots = [recovery_roots]
    for recovery_root in recovery_roots:
        for folder in sorted(recovery_root.iterdir()):
            if not folder.is_dir() or not (folder/"summary.json").exists():
                continue
            summary = json.loads((folder/"summary.json").read_text())
            if summary["scene"] not in TRAIN + RECOVERY or summary["mode"] != "collect":
                raise ValueError("Only independent recovery collection worlds can be fitted")
            if (summary.get("collection_protocol") != "complete-astra-recovery-v1"
                    or not summary.get("clean")):
                excluded.append(folder.name)
                continue
            with (folder/"events.jsonl").open() as stream:
                rows = [(r["packet"], r["decision"]) for l in stream
                        if (r:=json.loads(l))["event"] == "teacher_label"
                        and target(r["packet"], r["decision"]) is not None]
            runs.append((folder.name, rows))
    # Predeclared: left-side obstacle episodes validate; other layouts train.
    is_validation = lambda name: "train-left" in name or name.endswith("-2")
    training = [row for name, rows in runs if not is_validation(name) for row in rows]
    validation = [row for name, rows in runs if is_validation(name) for row in rows]
    if args.queries:
        for line in args.queries.open():
            row = json.loads(line)
            if row["source_scene"] not in TRAIN:
                raise ValueError("Held-out scene in training queries")
            if target(row["packet"],row["decision"]) is not None:
                (validation if is_validation(row["source_run"]) else training).append((row["packet"],row["decision"]))
    artifact = train(training, validation)
    artifact["training_runs"] = [n for n,r in runs if not is_validation(n)]
    artifact["validation_runs"] = [n for n,r in runs if is_validation(n)]
    artifact["excluded_runs"] = excluded
    artifact["recovery_collection_protocol"] = "complete-astra-recovery-v1"
    artifact["recovery_data"] = [str(p) for p in recovery_roots]
    artifact["test_scenes"] = list(TEST)
    artifact["supplemental_queries"] = str(args.queries) if args.queries else None
    artifact["training_rows_sha256"] = hashlib.sha256(json.dumps(training,sort_keys=True).encode()).hexdigest()
    artifact["validation_rows_sha256"] = hashlib.sha256(json.dumps(validation,sort_keys=True).encode()).hexdigest()
    model = LocalPolicy(artifact)
    artifact["validation_correct"] = sum(model.predict(p)[0] == target(p,d) for p,d in validation)
    artifact["by_robot"] = {}
    for name in ("ev3", "blast"):
        belongs = lambda p: ("ir_survey" in (p.get("local_map_evidence") or {})) == (name == "ev3")
        rows = [(p,d) for p,d in validation if belongs(p)]
        artifact["by_robot"][name] = {"training_labels":dict(Counter(target(p,d) for p,d in training if belongs(p))),
            "validation_labels":dict(Counter(target(p,d) for p,d in rows)),
            "validation_correct":sum(model.predict(p)[0] == target(p,d) for p,d in rows),
            "validation_rows":len(rows)}
    args.weights.parent.mkdir(parents=True, exist_ok=True)
    args.weights.write_text(json.dumps(artifact, separators=(",", ":")))
    print(json.dumps({k:v for k,v in artifact.items() if k not in ("w1","b1","w2","b2")}), flush=True)


def queries(args):
    """Teacher labels for proposing the direct goal as an unverified retained route.

    Use only training-world START observations; no new/fake sensor measurements,
    no simulator truth, no execution and no labels authored by this program.
    This supplies route-revision examples absent from clean route-following.
    """
    from robot_agent.physical_odometry import PhysicalPose
    from robot_agent.physical_footprint import RobotFootprint
    from robot_agent.waypoint_geometry import waypoint_geometry, waypoint_follow_motion_action, waypoint_motion_hint
    args.output.parent.mkdir(parents=True,exist_ok=True)
    enable_navigation_diagnostics(args.output.with_suffix(".diagnostics.jsonl"))
    teacher = LMStudioControllerActionPlanner(model=MODEL,reasoning_effort="high",max_output_tokens=8192,timeout_seconds=60)
    done = {json.loads(line)["source_run"] for line in args.output.open()} if args.output.exists() else set()
    with args.output.open("a") as stream:
        for robot in ("ev3","blast"):
            for scene in TRAIN:
                folder = args.data/f"{robot}-{scene}-astra"
                if folder.name in done:
                    continue
                row = next(r for l in (folder/"events.jsonl").open() if (r:=json.loads(l))["event"]=="decision_pair")
                p = row["packet"]
                evidence = p["local_map_evidence"]
                point = p["observation"].get("directional_goal") or evidence["directional_goal"]
                wp = {"x_mm":point.get("x_mm",point.get("target_x_mm")),
                      "y_mm":point.get("y_mm",point.get("target_y_mm")),"purpose":"Unverified direct goal route"}
                pose = evidence["robot_pose"]
                pose = PhysicalPose(pose["x_mm"],pose["y_mm"],round(pose["heading_deg"]*1000))
                geom = waypoint_geometry(pose,wp)
                p.update(active_waypoint=wp,active_waypoint_plan=[wp],active_waypoint_geometry={
                    "distance_mm":geom["distance_mm"],"bearing_deg":round(geom["bearing_mdeg"]/1000),
                    "heading_error_deg":round(geom["heading_error_mdeg"]/1000)})
                footprint = RobotFootprint(**{k:evidence["robot_footprint"][k] for k in (
                    "front_extent_mm","rear_extent_mm","left_extent_mm","right_extent_mm","clearance_margin_mm")})
                p["execution"]["next_waypoint_motion"] = waypoint_motion_hint(pose,wp,
                    waypoint_follow_motion_action(pose,wp,p["available_actions"]),footprint=footprint)
                for key in ("available_actions","plan_actions","history","active_waypoint_plan","obstacle_estimates"):
                    if p.get(key) is not None:
                        p[key] = tuple(p[key])
                context = ControllerActionContext(**p)
                result = teacher.decide(context)
                record = {"source_run":folder.name,"source_scene":scene,"packet":context.to_dict(),"decision":asdict(result.decision)}
                stream.write(json.dumps(record)+"\n")
                stream.flush()
                print(json.dumps({"run":folder.name,"label":target(record["packet"],record["decision"])}),flush=True)


def report(args):
    """Include failures and missing runs; never promote success-after-contact."""
    reference_root = getattr(args, "reference_root", None) or args.output
    weights = getattr(args, "weights", None)
    artifact = json.loads(weights.read_text()) if weights else {}
    method = artifact.get("method", "behavioural-cloning")
    rows = []
    for scene in TEST:
        for robot in ("ev3", "blast"):
            row = {"robot":robot, "scene":scene}
            for mode in ("astra", "local"):
                root = reference_root if mode == "astra" else args.output
                path = root/f"{robot}-{scene}-{mode}"/"summary.json"
                row[mode] = json.loads(path.read_text()) if path.exists() else {"missing":True,"clean":False}
                diagnostic = path.parent/"diagnostics.jsonl"
                if diagnostic.exists():
                    requests, resolved, efforts = 0, set(), set()
                    for line in diagnostic.open():
                        event = json.loads(line)
                        if event["event"] == "planner_request":
                            requests += 1
                            efforts.add(event["request"].get("reasoning_effort"))
                        elif event["event"] == "provider_response":
                            resolved.add(event.get("resolved_model"))
                    row[mode].update(astra_requests_including_retries=requests,
                                     resolved_models=sorted(resolved),reasoning_efforts=sorted(efforts))
            rows.append(row)
    comparisons = {"physical_robots_run":False, "model":MODEL, "reasoning":"high",
        "local_policy_method":method,
        "local_weights_sha256":hashlib.sha256(weights.read_bytes()).hexdigest() if weights else None,
        "astra_reference_root":str(reference_root.resolve()),
        "astra_reference_reused":reference_root.resolve() != args.output.resolve(),
        "test_cases":len(rows), "clean_astra":sum(r["astra"]["clean"] for r in rows),
        "clean_local":sum(r["local"]["clean"] for r in rows), "results":rows,
        "limitations":["One rollout per arm and case, not a reliability estimate; LLM decisions are stochastic.",
            ("One frozen, directly repaired numeric weight candidate using previous failed evaluation observations; no training. This is a development rerun, not an independent held-out test."
             if method in ("astra-direct-weights-recorded-repair-v1", "astra-direct-weights-set-repair-v1") else
             "One frozen, directly authored numeric weight candidate; no training or evaluation feedback to its author."
             if method.startswith("astra-direct-weights") else
             "Behavioural cloning with complete successful teacher recoveries, not reinforcement learning or weight-space programming."),
            ("Set network processes every supplied ray and every persistent route-outcome record; production packets currently cap rays at 128. Free-form explanations are not encoded."
             if method == "astra-direct-weights-set-repair-v1" else
             "Structured observation projection retains up to 128 rays and four recent action outcomes; free-form explanations are not encoded."),
            "Ideal range rays/encoder simulation with approximate white-surface IR, not full physical fidelity.",
            "The local model never sees hidden world geometry; only execution/evaluation does.",
            "Completion uses the existing goal contract and is counted separately from neural decisions."]}
    (args.output/"comparison.json").write_text(json.dumps(comparisons,indent=2))
    for row in rows:
        print(json.dumps({"robot":row["robot"],"scene":row["scene"],
            **{mode:{k:row[mode].get(k) for k in ("clean","goal_reached","blocked_moves","astra_calls","local_calls","wall_seconds","missing")}
               for mode in ("astra","local")}}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("run", "batch"):
        p = sub.add_parser(command)
        p.add_argument("--output", type=Path, required=True)
        p.add_argument("--weights", type=Path)
        p.add_argument("--decisions", type=int, default=48)
        p.add_argument("--commentary", action="store_true",
                       help="Generate real background commentary; log playback without hardware audio")
        if command == "run":
            p.add_argument("--collect", action="store_true")
            p.add_argument("--student-recovery", action="store_true",
                           help="Training-only: one teacher handoff from a student's actual failed state")
            p.add_argument("--fault", choices=("none",*FAULTS), default="none")
            p.add_argument("--replay-prefix", type=Path,
                           help="Replay an independent recorded training prefix, then collect real Astra recovery")
            p.add_argument("--fault-reads", type=int, default=32)
            p.add_argument("--fault-stalls", type=int, default=1)
            p.add_argument("--robot", choices=("ev3","blast"), required=True)
            p.add_argument("--scene", choices=TRAIN+RECOVERY+TEST+ATTENTION_HELD_OUT+RECOVERY_TEST+STUDENT_TEST+DENSE_TEST+MAP_TRAIN+MAP_DEV+MAP_TEST+LEARNER_TRAIN+LEARNER_TEST+PASSAGE_TEST, required=True)
        else:
            p.add_argument("--split", choices=("train","recovery","test"), required=True)
            p.add_argument("--paired", action="store_true", help="Alternate baseline/local on identical held-out worlds")
    p = sub.add_parser("fit")
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--weights", type=Path, required=True)
    p.add_argument("--recorded-box-root", type=Path)
    p.add_argument("--queries", type=Path)
    p.add_argument("--recovery-data", type=Path, action="append",
                   help="Complete teacher collection root; repeat to retain earlier recovery data")
    p = sub.add_parser("queries")
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("report")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--reference-root", type=Path, help="Explicitly reuse an existing Astra evaluation root")
    p.add_argument("--weights", type=Path, help="Frozen local model metadata; never runs or trains it")
    args = parser.parse_args()
    {"run":run, "batch":batch, "fit":fit, "queries":queries, "report":report}[args.command](args)


if __name__ == "__main__":
    main()
