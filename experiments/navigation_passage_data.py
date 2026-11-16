"""Offline action consequences, through the real adapters. Never imported at runtime.

Replay only training/development trajectories, branch each physical action from
the same observed state, and record actual simulator collision stops. Hidden
world geometry supplies labels, never model inputs. No counterfactual teacher
response is invented for REPLAN; only a recorded delegation can label that slot.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np

from experiments.local_navigation import ACTIONS, target
from experiments.navigation_attention_policy import observations
from experiments.navigation_recovery_benchmark import runtime_hashes, world_hash
from scripts.local_navigation_experiment import (
    MAP_TRAIN, MAP_DEV, LEARNER_TRAIN, PASSAGE_TEST, scenario, run_ev3,
    run_blast_gemma_scenario, ControllerActionDecision, ControllerActionPlannerResult,
)
from robot_agent.multi_robot_navigation_simulator import MultiRobotNavigationSimulator

VERSION = 'counterfactual-passage-v1'


def input_digest(packet):
    digest = hashlib.sha256()
    for values in observations(packet):
        array = np.asarray(values, dtype=np.float32)
        digest.update(str(array.shape).encode())
        digest.update(array.tobytes())
    digest.update(json.dumps(packet['available_actions']).encode())
    return digest.hexdigest()


class ProbeComplete(Exception):
    pass


def probe(robot, scene_name, decisions, index, action):
    """One isolated branch; startup and prefix run unchanged in production adapters."""
    state = {'calls': 0, 'blocked': 0}
    original = MultiRobotNavigationSimulator._record

    def record(world, robot_id, kind, value=0):
        if kind == 'blocked':
            state['blocked'] += 1
        return original(world, robot_id, kind, value)

    class Planner:
        def decide(self, context):
            call = state['calls']
            state['calls'] += 1
            if call > index:
                state['finished'] = True
                raise ProbeComplete()
            if call < index:
                decision = ControllerActionDecision(**decisions[call]['decision'])
            else:
                state['packet'] = context.to_dict()
                state['blocked_before'] = state['blocked']
                decision = (ControllerActionDecision(**decisions[index]['decision']) if action == 'REPLAN'
                            else ControllerActionDecision(action=action,confidence_milli=1000,
                                assessment='Offline counterfactual action.',plan=(action,),utterance=None))
            return ControllerActionPlannerResult(decision, 0)

    scene = scenario(robot, scene_name)
    try:
        with patch.object(MultiRobotNavigationSimulator, '_record', record):
            if robot == 'ev3':
                outcome = run_ev3(scene, Planner(), index+2, lambda event: None)
            else:
                distance = scene.goals[0].x_mm-scene.robots[0].pose.x_mm
                outcome = run_blast_gemma_scenario(scene, max_decisions=index+2,
                    planner_factory=lambda _: Planner(), minimum_forward_progress_mm=distance,
                    goal=f'Reach the goal {distance / 10:g} cm straight ahead of your starting position.')
    except ProbeComplete:
        outcome = {}
    if 'packet' not in state or ('finished' not in state and not outcome.get('completed')):
        raise ValueError('Counterfactual did not finish its action: '+str(outcome.get('error_message', outcome)))
    return state['packet'], state['blocked']-state['blocked_before']


def selected_indices(decisions):
    """Uniform temporal coverage plus the states before/after reported failures."""
    selected = set(range(0, len(decisions), 4))
    for index, row in enumerate(decisions):
        packet = row['packet']
        last = (packet.get('observation') or {}).get('last_action') or {}
        rejected = (packet.get('local_map_evidence') or {}).get('latest_route_rejection')
        if last.get('completed') is False or rejected:
            selected.update(range(max(0,index-2),index+1))
    return sorted(i for i in selected if decisions[i]['decision']['action'] != 'COMPLETE')


def sources(roots, split, robot, limit):
    allowed = set(MAP_DEV if split == 'development' else (*MAP_TRAIN,*LEARNER_TRAIN))
    groups = []
    for root in roots:
        for path in sorted((root/split).glob('*/events.jsonl')):
            events = [json.loads(line) for line in path.read_text().splitlines()]
            config = events[0]
            if config['robot'] != robot:
                continue
            if config['scene'] not in allowed or config['mode'] != 'collect':
                raise ValueError('Only declared training/development worlds may supply risk labels')
            if config.get('fault') != 'none' or config.get('replay_prefix'):
                continue  # Fault injection cannot be reproduced by an ordinary prefix.
            decisions = [e for e in events if e['event'] == 'decision_pair']
            groups.append([(path, config, decisions, i) for i in selected_indices(decisions)])
    # Round-robin episodes, not first-N frames from the first world.
    selected = []
    while any(groups) and len(selected) < limit:
        for group in groups:
            if group and len(selected) < limit:
                selected.append(group.pop(0))
    return selected


def collect(output, roots, train_limit=160, dev_limit=48):
    output.mkdir(parents=True, exist_ok=False)
    base = json.loads((roots[-1]/'protocol.json').read_text())
    current = runtime_hashes()
    for root in roots:
        prior = json.loads((root/'protocol.json').read_text())
        if any(prior['runtime_hashes'].get(k) != v for k,v in current.items() if k.startswith('src/')):
            raise ValueError('Production changed since recorded prefixes')
    plan = {**base, 'version': VERSION, 'jobs': [], 'runtime_hashes': current,
        'test_scenes': list(PASSAGE_TEST), 'test_decisions': 128,
        'test_world_hashes': {r+'-'+s:world_hash(r,s) for r in ('ev3','blast') for s in PASSAGE_TEST},
        'risk_sources': {str(r.resolve()):hashlib.sha256((r/'protocol.json').read_bytes()).hexdigest() for r in roots},
        'risk_sampling': {'train_limit_per_robot':train_limit,'development_limit_per_robot':dev_limit,
                          'temporal_stride':4, 'before_failed_receipt':2},
        'training': {**base['training'], 'risk_weight':1.0},
        'risk_labels': 'Actual collision stops of alternative actions replayed from training/development prefixes. Hidden world is labels only; no inference change.',
        'selection': 'Existing successful Astra demonstrations plus counterfactual action outcomes. Development combined imitation/risk loss selects weights. Test worlds frozen before collection/fit.'}
    (output/'protocol.json').write_text(json.dumps(plan, indent=2))
    audit = []
    for split,limit in (('train',train_limit),('development',dev_limit)):
        for robot in ('ev3','blast'):
            counts = Counter()
            path = output/f'{robot}-{split}-risk.jsonl'
            with path.open('w') as stream:
                for number,(source,config,decisions,index) in enumerate(sources(roots,split,robot,limit)):
                    packet = decisions[index]['packet']
                    # No waypoint: ordinary LocalPlanner delegates and cannot choose a physical action.
                    if not packet.get('active_waypoint'):
                        continue
                    actions = [a for a in ACTIONS[:-1] if a in packet['available_actions']]
                    if target(packet,decisions[index]['decision']) == 'REPLAN':
                        actions.append('REPLAN')
                    outcomes,branch_digest = {},None
                    for action in actions:
                        observed,blocked = probe(robot,config['scene'],decisions,index,action)
                        actual_digest=input_digest(observed)
                        if branch_digest is not None and actual_digest!=branch_digest:
                            raise ValueError(f'Counterfactual branches differ: {source.parent.name}/{index}/{action}')
                        branch_digest=actual_digest
                        outcomes[action] = {'collision_stops':blocked}
                    row = {'version':VERSION,'robot':robot,'split':split,'scene':config['scene'],
                           'source':str(source.resolve()),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                           'decision_index':index,'input_sha256':branch_digest,'packet':observed,
                           'original_input_sha256':input_digest(packet),
                           'source_observation_reproduced':branch_digest==input_digest(packet),
                           'outcomes':outcomes}
                    stream.write(json.dumps(row)+'\n'); stream.flush()
                    counts['states'] += 1
                    counts['source_observation_differences']+=not row['source_observation_reproduced']
                    for action,result in outcomes.items():
                        counts[action+('-blocked' if result['collision_stops'] else '-clear')] += 1
                    if number % 10 == 0:
                        print(json.dumps({'robot':robot,'split':split,'sample':number,**counts}),flush=True)
            audit.append({'robot':robot,'split':split,'counts':dict(counts),
                          'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    (output/'risk-audit.json').write_text(json.dumps(audit,indent=2))
    print(json.dumps({'complete':True,'audit':audit}),flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--source',type=Path,action='append',required=True)
    parser.add_argument('--train-limit',type=int,default=160)
    parser.add_argument('--dev-limit',type=int,default=48)
    args = parser.parse_args()
    collect(args.output,args.source,args.train_limit,args.dev_limit)
