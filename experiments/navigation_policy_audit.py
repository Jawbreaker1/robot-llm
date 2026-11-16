"""Offline causal audit of frozen weights; no authoring, training or hardware.

Recompute pre/post-mask choices and replay each recorded Astra response stream
with both weight files. The teacher is fixed so its stochastic route choices
cannot be mistaken for an effect of different local weights.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from experiments.local_navigation import ACTIONS, LocalPlanner, LocalPolicy
from experiments.navigation_set_policy import observations
from robot_agent.blast_navigation_simulation import run_blast_gemma_scenario
from robot_agent.lm_studio_controller_action import ControllerActionDecision, ControllerActionPlannerResult
from scripts.local_navigation_experiment import run_ev3, scenario


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def events(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def decision_rows(rows):
    dispatch = None
    for row in rows:
        if row['event'] == 'policy_dispatch':
            dispatch = row
        elif row['event'] == 'decision_pair':
            assert dispatch is not None
            yield dispatch, row
            dispatch = None


def choices(model, packet):
    scores = model.scores(packet)
    raw = ACTIONS[max(range(len(ACTIONS)), key=scores.__getitem__)]
    selected, probability = model.predict(packet)
    allowed = set(packet['available_actions']) | {'REPLAN'}
    ordered = sorted((s for a, s in zip(ACTIONS, scores) if a in allowed), reverse=True)
    return dict(raw=raw, selected=selected, probability=probability,
                margin=ordered[0]-ordered[1], scores=scores)


def differences(a, b, path=()):
    if isinstance(a, (tuple,list)) and isinstance(b, (tuple,list)):
        if len(a) != len(b):
            return [(path,'length',len(a),len(b))]
        return [d for i,(x,y) in enumerate(zip(a,b)) for d in differences(x,y,(*path,i))][:8]
    return [] if a == b else [(path,a,b)]


def replay(job, rows, model, maximum):
    teacher_rows = [r for d, r in decision_rows(rows) if d['source'] == 'astra']
    dispatches, packets, decisions, execution = [], [], [], []

    class RecordedTeacher:
        calls = 0
        def decide(self, context):
            record = teacher_rows[self.calls]  # Exhaustion is a failed audit.
            self.calls += 1
            return ControllerActionPlannerResult(ControllerActionDecision(**record['decision']), 0)

    teacher = RecordedTeacher()
    local = LocalPlanner(teacher, model, dispatches.append)

    class RecordingPlanner:
        def decide(self, context):
            packets.append(observations(context.to_dict()))
            result = local.decide(context)
            decisions.append(result.decision.action)
            return result

    scene = scenario(job['robot'], job['scene'])
    if job['robot'] == 'ev3':
        result = run_ev3(scene, RecordingPlanner(), maximum, execution.append)
    else:
        distance = scene.goals[0].x_mm-scene.robots[0].pose.x_mm
        result = run_blast_gemma_scenario(scene, planner_factory=lambda _:RecordingPlanner(),
            max_decisions=maximum, minimum_forward_progress_mm=distance,
            goal=f'Reach the goal {distance / 10:g} cm straight ahead of your starting position.')
    assert not result.get('error_type'), result.get('error_message')
    original_packets = [observations(r['packet']) for _, r in decision_rows(rows)]
    assert len(packets) == len(original_packets), (job['name'], 'decision count diverged')
    assert decisions == [r['decision']['action'] for _, r in decision_rows(rows)]
    # Instant replay can renew a timed EV3 worker at a different point than the
    # original slow LLM run. Report any historical input discrepancy; do not
    # silently call it identical. The paired NEW runs must match in full below.
    historical_differences = [dict(step=i+1,differences=differences(p,q))
                              for i,(p,q) in enumerate(zip(packets,original_packets)) if p != q]
    assert teacher.calls == len(teacher_rows)
    return dict(decisions=decisions, selections=[d['selection'] for d in dispatches],
                encoded_inputs_sha256=hashlib.sha256(json.dumps(packets).encode()).hexdigest(),
                historical_input_differences=historical_differences,
                astra_calls=teacher.calls, result={k:result.get(k) for k in (
                    'completed', 'goal_reached', 'blocked_moves', 'distance_to_goal_mm',
                    'terminal_reason', 'final_pose')})


def audit(root):
    manifest = json.loads((root/'comparison/manifest.json').read_text())
    repo = Path(__file__).resolve().parents[1]
    for name, expected in manifest['source_hashes'].items():
        assert digest(repo/name) == expected, ('source changed since trial', name)
    for key, path in manifest['weights'].items():
        assert digest(Path(path)) == manifest['weight_hashes'][key]
    models = {key:LocalPolicy.load(path) for key, path in manifest['weights'].items()}
    report = dict(verified_weight_hashes=manifest['weight_hashes'],
                  verified_source_files=len(manifest['source_hashes']),
                  no_training=True, no_hardware=True, no_new_llm_calls=True,
                  counts={robot:Counter() for robot in ('ev3','blast')}, examples=[],
                  input_connections={}, fixed_teacher_replays=[])
    for key, model in models.items():
        a = model.artifact
        report['input_connections'][key] = {
            'goal_and_waypoint_xy':sum(v != 0 for row in a['head_weights'][1:5] for v in row),
            'global_route_vertices':sum(v != 0 for row in a['head_weights'][7:19] for v in row),
            'ray_waypoint_geometry':sum(v != 0 for row in a['ray_weights'][12:17] for v in row),
            'memory_leg_and_pose_xy':sum(v != 0 for row in a['memory_weights'][4:12] for v in row),
        }
    for job in manifest['jobs']:
        rows = events(root/'comparison'/job['name']/'events.jsonl')
        robot = job['robot']
        key = 'shared' if job['variant'] == 'shared' else robot
        assert rows[0]['model_sha256'] == manifest['weight_hashes'][key]
        counts = report['counts'][robot]
        for step, (dispatch, row) in enumerate(decision_rows(rows), 1):
            packet = row['packet']
            if not packet.get('active_waypoint') or packet.get('completion_allowed'):
                continue
            a, b = (choices(models[k], packet) for k in ('shared',robot))
            current = a if key == 'shared' else b
            assert current['selected'] == dispatch['selection']
            assert abs(current['probability']-dispatch['probability']) < 1e-9
            if dispatch['source'] == 'local':
                assert current['selected'] == row['decision']['action']
            counts['packets'] += 1
            counts['different_scores'] += a['scores'] != b['scores']
            counts['different_raw_winner'] += a['raw'] != b['raw']
            counts['different_selected_action'] += a['selected'] != b['selected']
            counts['raw_'+a['raw']] += 1
            counts['raw_to_selected_'+a['raw']+'->'+a['selected']] += 1
            for action in ACTIONS[:-1]:
                counts['available_'+action] += action in packet['available_actions']
            counts['mask_changes_shared'] += a['raw'] != a['selected']
            counts['mask_changes_specialized'] += b['raw'] != b['selected']
            counts['selected_'+a['selected']] += 1
            if step == 2 and job['scene'] == 'box':
                report['examples'].append(dict(run=job['name'],step=step,shared=a,specialized=b))
        # Fix each of the sixteen teacher streams, then change ONLY weights.
        shared = replay(job, rows, models['shared'], manifest['max_decisions'])
        specialized = replay(job, rows, models[robot], manifest['max_decisions'])
        assert shared == specialized, (job['name'], 'weight-only outcomes differ')
        original = json.loads((root/'comparison'/job['name']/'summary.json').read_text())
        assert shared['result'] == {k:original.get(k) for k in shared['result']}, (
            job['name'], 'historical outcome did not reproduce')
        report['fixed_teacher_replays'].append(dict(run=job['name'], identical=True, **shared))
        print(job['name'], 'fixed-teacher replays identical', flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.root.resolve())
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result['counts'], indent=2))
