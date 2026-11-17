"""Bounded collection and frozen, paired tests; no robot navigation policy here."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

from robot_agent.codex_subscription import CODEX_MODEL, CODEX_PREFIX
from experiments.navigation_specialization import source_hashes, digest
from scripts.local_navigation_experiment import TRAIN, RECOVERY_TEST, STUDENT_TEST, DENSE_TEST, MAP_TRAIN, MAP_DEV, MAP_TEST, LEARNER_TRAIN, LEARNER_TEST, scenario

REPO = Path(__file__).resolve().parents[1]
CORE = {'scripts/local_navigation_experiment.py','experiments/local_navigation.py',
        'experiments/navigation_set_policy.py','experiments/navigation_attention_policy.py',
        'experiments/navigation_recovery_data.py','experiments/navigation_student_recovery.py'}


def runtime_hashes():
    return {k:v for k,v in source_hashes().items() if k.startswith('src/') or k in CORE}


def world_hash(robot,scene):
    return hashlib.sha256(json.dumps(asdict(scenario(robot,scene)),sort_keys=True).encode()).hexdigest()


def execute(root, jobs, plan):
    def one(job):
        if runtime_hashes()!=plan['runtime_hashes']:
            raise ValueError('Runtime changed during experiment')
        folder = root/job['split']/job['name']
        folder.parent.mkdir(parents=True,exist_ok=True)
        command = [sys.executable,'scripts/local_navigation_experiment.py','run',
                   '--robot',job['robot'],'--scene',job['scene'],'--decisions',str(job['decisions']),
                   '--output',str(folder)]
        if job.get('weights'):
            if digest(Path(job['weights']))!=job['weight_sha256']:
                raise ValueError('Frozen weights changed')
            command += ['--weights',job['weights']]
        if job['split']!='test':
            command += ['--collect','--fault',job['fault']]
            if job.get('student_recovery'): command += ['--student-recovery']
            if 'fault_reads' in job:
                command += ['--fault-reads',str(job['fault_reads'])]
        env = dict(os.environ,PYTHONPATH='.:src',PYTHONDONTWRITEBYTECODE='1',
                   OPENBLAS_NUM_THREADS='1',VECLIB_MAXIMUM_THREADS='1',OMP_NUM_THREADS='1')
        with folder.with_suffix('.log').open('w') as stream:
            result = subprocess.run(command,cwd=REPO,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=3600)
        if result.returncode:
            raise RuntimeError('Trial process failed: '+job['name'])
        summary = json.loads((folder/'summary.json').read_text())
        print(json.dumps({'case':job['name'],**{k:summary.get(k) for k in
            ('clean','recovery_complete','recovery_collision_stops','goal_reached','blocked_moves','decisions','astra_calls','wall_seconds')}}),flush=True)
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(one,jobs))


def collect(root):
    root.mkdir(parents=True,exist_ok=False)
    jobs = []
    for scene in TRAIN:
        faults = ('none',) if scene=='train-open' else ('none','stall','range','scan','route')
        for fault in faults:
            for robot in ('ev3','blast'):
                jobs.append({'split':'train','robot':robot,'scene':scene,'fault':fault,
                             'name':f'{robot}-{scene}-{fault}','decisions':48})
    for scene in ('train-left-corner','train-right-corner'):
        for fault in ('stall','route'):
            for robot in ('ev3','blast'):
                jobs.append({'split':'development','robot':robot,'scene':scene,'fault':fault,
                             'name':f'{robot}-{scene}-{fault}','decisions':48})
    plan = {'version':'attention-recovery-training-v1','train_scenes':list(TRAIN),
            'development_scenes':['train-left-corner','train-right-corner'],
            'test_scenes':list(RECOVERY_TEST),'jobs':jobs,'runtime_hashes':runtime_hashes(),
            'test_world_hashes':{r+'-'+s:world_hash(r,s) for r in ('ev3','blast') for s in RECOVERY_TEST},
            'training':{'seed':23,'epochs':60,'patience':12,'learning_rate':1e-5,'batch_size':8},
            'teacher':CODEX_MODEL,'reasoning':'high','physical_robots':False,
            'selection':'Complete collision-free teacher runs only. Fault must actually be observed before counting recovery coverage.',
            'split':'Whole independent layouts; no test results or test packets may be used for fitting or model selection.'}
    (root/'protocol.json').write_text(json.dumps(plan,indent=2))
    execute(root,jobs,plan)


def collect_map(root):
    """Fresh broad data: same precision, action boundary and map inputs as tests."""
    root.mkdir(parents=True,exist_ok=False)
    jobs=[]
    for scene,fault in zip(MAP_TRAIN,('none','stall','range','scan','route','none','none','none')):
        for robot in ('ev3','blast'):
            jobs.append({'split':'train','robot':robot,'scene':scene,'fault':fault,
                         'name':f'{robot}-{scene}-{fault}','decisions':96})
    for scene in MAP_DEV:
        for robot in ('ev3','blast'):
            jobs.append({'split':'development','robot':robot,'scene':scene,'fault':'none',
                         'name':f'{robot}-{scene}','decisions':96})
    plan={'version':'observed-map-bounded-action-v3','train_scenes':list(MAP_TRAIN),
          'development_scenes':list(MAP_DEV),'test_scenes':list(MAP_TEST),
          'jobs':jobs,'runtime_hashes':runtime_hashes(),
          'test_world_hashes':{r+'-'+s:world_hash(r,s) for r in ('ev3','blast') for s in MAP_TEST},
          'training':{'initialization':'dense-centered-xavier-he-v1','seed':23,'epochs':120,'patience':20,
                      'learning_rate':1e-4,'weight_decay':1e-4,'batch_size':8},
          'teacher':CODEX_MODEL,'reasoning':'high','physical_robots':False,
          'selection':'Complete collision-free teacher runs, or the successful suffix after the last reported failed operation of a goal-reaching teacher run. Unsafe prefixes never train. Development loss selects weights.',
          'budget':'96 decisions per trial; no mission wall-clock timeout. A separate 3600-second process watchdog is an infrastructure failure, not a navigation result.',
          'split':'Eight training geometries, two development geometries, three independent test geometries. No old-physics data.'}
    (root/'protocol.json').write_text(json.dumps(plan,indent=2))
    execute(root,jobs,plan)


def compare(root,weights,reference=None):
    plan = json.loads((root/'protocol.json').read_text())
    if (root/'test').exists():
        raise ValueError('Refusing to overwrite independent test results')
    jobs = []
    for i,scene in enumerate(plan['test_scenes']):
        for j,robot in enumerate(('ev3','blast')):
            if world_hash(robot,scene)!=plan['test_world_hashes'][robot+'-'+scene]:
                raise ValueError('Test world changed after training declaration')
            arms=('astra','local') if (i+j)%2==0 else ('local','astra')
            if reference is not None:
                arms=(*arms,'previous')
            for arm in arms:
                job = {'split':'test','robot':robot,'scene':scene,'arm':arm,
                       'name':f'{robot}-{scene}-{arm}','decisions':plan.get('test_decisions',96)}
                if arm!='astra':
                    path = (reference if arm=='previous' else weights)/(robot+'.npz')
                    job.update(weights=str(path),weight_sha256=digest(path))
                jobs.append(job)
    (root/'evaluation.json').write_text(json.dumps({'jobs':jobs,'source_hashes':source_hashes()},indent=2))
    execute(root,jobs,plan)
    report(root)


def collect_student(root,weights,base_data):
    if base_data is None:
        raise ValueError('Student collection requires the explicit base-data protocol')
    base=json.loads((base_data/'protocol.json').read_text())
    if any(base['runtime_hashes'].get(k)!=v for k,v in runtime_hashes().items() if k.startswith('src/')):
        raise ValueError('Production runtime changed since the base collection')
    root.mkdir(parents=True,exist_ok=False)
    jobs=[]
    for scene in (*LEARNER_TRAIN,'map-train-2','map-train-3'):
        for robot in ('ev3','blast'):
            path=weights/(robot+'.npz')
            jobs.append({'split':'train','robot':robot,'scene':scene,'fault':'none',
                'name':f'{robot}-{scene}-student','decisions':128,'student_recovery':True,
                'weights':str(path),'weight_sha256':digest(path)})
    plan={'version':'map-student-recovery-v2',
        'train_scenes':[*base['train_scenes'],*LEARNER_TRAIN],
        'development_scenes':base['development_scenes'],'test_scenes':list(LEARNER_TEST),
        'jobs':jobs,'runtime_hashes':runtime_hashes(),
        'test_world_hashes':{r+'-'+s:world_hash(r,s) for r in ('ev3','blast') for s in LEARNER_TEST},
        'test_decisions':128,'base_data':str(base_data.resolve()),
        'training':base['training'],
        'teacher':CODEX_MODEL,'reasoning':'high','physical_robots':False,
        'selection':'Only real Astra labels after one same-episode student-state handoff; goal reached and zero new collision stops in teacher suffix. Prefix actions are never labels.',
        'required_training_contexts':{'ev3':['near_obstacle','infrared_blocked'],
                                      'blast':['KNOWN_ECHO_CLEARANCE_INTERSECTION']},
        'coverage_requirement':'Before fitting, each robot must have accepted teacher labels for at least one of its listed critical rejection reasons. Report exact counts and distinct episodes; a generic fault name is not coverage.',
        'split':'Reuse only previous training/development data, add independent learner training worlds, and freeze three entirely new test worlds. No fitting on any test packet.',
        'handoff':'Training-data selection only: native failed outcome after local action, third stationary action at same approximate pose, or third movement away from the same retained waypoint. No range-missing-only trigger.'}
    (root/'protocol.json').write_text(json.dumps(plan,indent=2))
    execute(root,jobs,plan)


def report(root):
    evaluation = json.loads((root/'evaluation.json').read_text())
    rows=[]
    for job in evaluation['jobs']:
        folder=root/'test'/job['name']
        if not (folder/'summary.json').exists():
            rows.append({**job,'status':'pending'})
            continue
        result=json.loads((folder/'summary.json').read_text())
        with (folder/'events.jsonl').open() as stream:
            config=json.loads(next(stream))
        assert config['model_sha256']==job.get('weight_sha256')
        assert hashlib.sha256(json.dumps(config['world'],sort_keys=True).encode()).hexdigest()==world_hash(job['robot'],job['scene'])
        audit=audit_trial(folder,job,json.loads((root/'protocol.json').read_text()))
        rows.append({**job,'status':'complete',**{k:result.get(k) for k in
            ('clean','goal_reached','blocked_moves','distance_to_goal_mm','decisions','astra_calls','local_calls',
             'wall_seconds','decision_seconds','terminal_reason','error_type','error_message')},'audit':audit})
    arms=tuple(dict.fromkeys(job['arm'] for job in evaluation['jobs']))
    totals={r:{a:{'runs':len(selected:=[x for x in rows if x['robot']==r and x['arm']==a and x['status']=='complete']),
        'clean':sum(bool(x['clean']) for x in selected),'goal_reached':sum(bool(x['goal_reached']) for x in selected),
        'collision_stops':sum(x['blocked_moves'] or 0 for x in selected),
        'astra_calls':sum(x['astra_calls'] or 0 for x in selected),
        'decision_seconds':sum(x['decision_seconds'] or 0 for x in selected)} for a in arms} for r in ('ev3','blast')}
    result={'results':rows,'totals':totals,'physical_robots':False,'training_test_overlap':False,
            'limits':['One stochastic rollout per arm/layout; not a reliability guarantee.',
                      'Local arm is trained actions plus Astra route planning, not an independent full navigator.',
                      'Ideal encoders/range rays and approximate white-surface IR; no physical validation.',
                      'Any simulator collision stop disqualifies a clean run; goal radius is 50mm.']}
    (root/'comparison.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(totals),flush=True)


def audit_trial(folder,job,plan):
    """Verify the real provider and replay the actual learned dispatch, not a proxy."""
    from experiments.local_navigation import LocalPolicy
    recorded=json.loads((folder/'source-hashes.json').read_text())
    if any(recorded.get(k)!=v for k,v in plan['runtime_hashes'].items()):
        raise ValueError('Trial runtime differs from frozen protocol')
    models,efforts=set(),set()
    for line in (folder/'diagnostics.jsonl').read_text().splitlines():
        row=json.loads(line)
        if row.get('event')=='provider_response':models.add(row.get('resolved_model'))
        if row.get('event')=='planner_request':efforts.add(row['request'].get('reasoning_effort'))
    if models!={plan['teacher'].removeprefix(CODEX_PREFIX)} or efforts!={'high'}:
        raise ValueError('Trial provider was not verified as Astra high')
    policy=LocalPolicy.load(Path(job['weights'])) if job.get('weights') else None
    dispatch=None
    checked=0
    for line in (folder/'events.jsonl').read_text().splitlines():
        row=json.loads(line)
        if row['event']=='policy_dispatch':dispatch=row
        if row['event']!='decision_pair' or policy is None:continue
        packet=row['packet']
        if packet.get('completion_allowed'):
            expected='COMPLETE'
        elif not packet.get('active_waypoint'):
            expected='REPLAN'
        else:
            expected=policy.predict(packet)[0]
            checked+=1
        if dispatch is None or dispatch['selection']!=expected:
            raise ValueError('Recorded policy dispatch does not match exported weights')
        if expected!='REPLAN' and row['decision']['action']!=expected:
            raise ValueError('Selected local action was replaced')
    return {'models':sorted(models),'reasoning_efforts':sorted(efforts),
            'learned_decisions_replayed':checked,'dispatch_mismatches':0,'runtime_hashes_match':True}


def prepare_dense(root):
    """Freeze new worlds and training settings before any dense-model fitting."""
    root.mkdir(parents=True,exist_ok=False)
    plan={'version':'dense-initialization-training-v1','train_scenes':[*TRAIN,'train-staggered'],
        'development_scenes':['train-left-corner','train-right-corner'],'test_scenes':list(DENSE_TEST),
        'jobs':[],'runtime_hashes':runtime_hashes(),
        'test_world_hashes':{r+'-'+s:world_hash(r,s) for r in ('ev3','blast') for s in DENSE_TEST},
        'training':{'initialization':'dense-centered-xavier-he-v1','seed':23,'epochs':200,'patience':30,
                    'learning_rate':1e-4,'weight_decay':1e-4,'batch_size':8},
        'teacher':CODEX_MODEL,'reasoning':'high','physical_robots':False,
        'selection':'Reuse accepted teacher training/development data only. Dense checkpoints selected by development loss; previous sparse weights are a reported reference, not a fallback candidate.',
        'split':'Four new geometric layouts frozen before fitting; no old or new test packets admitted to training.'}
    (root/'protocol.json').write_text(json.dumps(plan,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=('collect','collect-map','collect-student','prepare-dense','compare','report'))
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--weights',type=Path)
    p.add_argument('--base-data',type=Path)
    p.add_argument('--reference',type=Path,help='Optional previous weights tested on the identical held-out worlds')
    a=p.parse_args()
    if a.command=='collect': collect(a.output)
    elif a.command=='collect-map': collect_map(a.output)
    elif a.command=='collect-student': collect_student(a.output,a.weights,a.base_data)
    elif a.command=='prepare-dense': prepare_dense(a.output)
    elif a.command=='compare': compare(a.output,a.weights,a.reference)
    else: report(a.output)
