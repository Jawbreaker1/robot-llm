"""Offline supervised training of the EXISTING attention network.

No added inference layer: Torch is only an autograd implementation of the NumPy
forward pass. Exported numeric NPZ files use the same runtime and action slot.
Whole teacher episodes, not randomly split frames, define train/development sets.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from robot_agent.codex_subscription import ASTRA_MODEL_ID
from experiments.navigation_attention_policy import (
    AttentionPolicy, observations, save_weights, WIDTH, HEADS, BLOCKS,
    GLOBAL_SIZE, RAY_FIELDS, MEMORY_FIELDS, MAP_FIELDS, shapes,
)
from experiments.local_navigation import ACTIONS, target
from scripts.local_navigation_experiment import TRAIN, RECOVERY, MAP_TRAIN, MAP_DEV, LEARNER_TRAIN
from experiments.navigation_student_recovery import PROTOCOL as STUDENT_PROTOCOL

METHOD = "map-aware-supervised-recovery-v3"
INITIALIZATION = "dense-centered-xavier-he-v1"


def initialize_weights(seed):
    """Dense, symmetry-breaking initialization; no navigation knowledge/rules.

    All matrices use x @ W, so PyTorch fan-in initializers receive W.T.
    He initialization serves ReLU inputs; Xavier serves the linear projections.
    Residual output matrices are scaled for the existing three-block stack.
    https://docs.pytorch.org/docs/2.15/nn.init.html
    """
    generator = torch.Generator().manual_seed(seed)
    weights = {}
    for key,shape in shapes().items():
        tensor = torch.zeros(shape,dtype=torch.float32)
        if key.endswith('_w'):
            if key.endswith('_ff1_w'):
                nn.init.kaiming_uniform_(tensor.T,nonlinearity='relu',generator=generator)
            else:
                nn.init.xavier_uniform_(tensor.T,generator=generator)
            if key.endswith(('_o_w','_ff2_w')):
                tensor /= math.sqrt(2*BLOCKS)
        elif key=='type_embedding':
            nn.init.normal_(tensor,std=.02,generator=generator)
        weights[key]=tensor.numpy().copy()
    return weights


def encode(packet):
    g, rays, memory, map_tokens = observations(packet)
    inputs = tuple(torch.tensor(v, dtype=torch.float32).reshape(-1, size)
                   for v,size in (([g],GLOBAL_SIZE),(rays,len(RAY_FIELDS)),(memory,len(MEMORY_FIELDS)),(map_tokens,len(MAP_FIELDS))))
    allowed = set(packet['available_actions']) | {'REPLAN'}
    mask = torch.tensor([a in allowed for a in ACTIONS], dtype=torch.bool)
    return inputs, mask


class TrainableAttention(nn.Module):
    def __init__(self, weights):
        super().__init__()
        validated = AttentionPolicy(weights)
        self.w = nn.ParameterDict({k:nn.Parameter(torch.from_numpy(v.copy()))
                                   for k,v in validated.weights.items()})

    def forward(self, inputs, preactivations=None):
        w = self.w
        rows = [x @ w[name+'_w'] + w[name+'_b'] + w['type_embedding'][kind]
                for kind,(name,x) in enumerate(zip(('global','ray','memory','map'),inputs)) if len(x)]
        x = torch.cat(rows, dim=0)
        n, d = len(x), WIDTH//HEADS
        for i in range(BLOCKS):
            p = f'b{i}_'
            q,k,v = [(x @ w[p+name+'_w'] + w[p+name+'_b']).reshape(n,HEADS,d).transpose(0,1)
                     for name in ('q','k','v')]
            attention = torch.softmax(q @ k.transpose(-2,-1) / math.sqrt(d), dim=-1)
            mixed = (attention @ v).transpose(0,1).reshape(n,WIDTH)
            x = x + mixed @ w[p+'o_w'] + w[p+'o_b']
            hidden = x @ w[p+'ff1_w'] + w[p+'ff1_b']
            if preactivations is not None:
                preactivations.append(hidden[0].detach())
            x = x + torch.relu(hidden) @ w[p+'ff2_w'] + w[p+'ff2_b']
        return x[0] @ w['out_w'] + w['out_b']

    def arrays(self):
        return {k:v.detach().cpu().numpy().copy() for k,v in self.w.items()}


@torch.no_grad()
def initialize_model(rows, seed):
    """Center each existing ReLU input using TRAINING observations only.

    The same formula initializes all units, sequentially by block. This is a
    one-time bias initialization, not a normalization layer or runtime rule.
    Center the global readout token: last-block non-global outputs cannot affect
    this model's readout. A small positive mean also handles constant features.
    """
    if not rows:
        raise ValueError('Training observations are required for initialization')
    model=TrainableAttention(initialize_weights(seed))
    for block in range(BLOCKS):
        total=torch.zeros_like(model.w[f'b{block}_ff1_b'])
        for inputs,_,_,_ in rows:
            preactivations=[]
            model(inputs,preactivations)
            total += preactivations[block]
        model.w[f'b{block}_ff1_b'].add_(.01-total/len(rows))
    return model


def gradient_audit(model, rows):
    """Measure actual training-data gradient flow, without taking any steps.

    Zero-valued input features may have no data-driven gradient. Common key
    biases are softmax-shift redundant; neither is confused with dead ReLU units.
    Optional input streams can be empty in individual observations. Measure
    participation across the dataset and report any never-used parameters.
    Synthetic unit tests separately exercise all input dimensions.
    """
    seen={k:torch.zeros_like(v,dtype=torch.bool) for k,v in model.w.items()}
    participated=set()
    for inputs,mask,label,_ in rows:
        model.zero_grad(set_to_none=True)
        loss=F.cross_entropy(model(inputs).masked_fill(~mask,-1e9)[None],torch.tensor([label]))
        loss.backward()
        for k,v in model.w.items():
            if v.grad is None:
                continue
            if not torch.isfinite(v.grad).all():
                raise ValueError('Nonfinite gradient: '+k)
            participated.add(k)
            seen[k] |= v.grad.abs()>1e-12
    units=[]; heads=[]; zero_units=[]
    for i in range(BLOCKS):
        p=f'b{i}_'
        units.append(int((seen[p+'ff1_w'].any(dim=0)|seen[p+'ff1_b']).sum()))
        zero_units.append(int(((model.w[p+'ff1_w']==0).all(dim=0)&(model.w[p+'ff1_b']==0)).sum()))
        heads.append({name:int(seen[p+name+'_w'].any(dim=0).reshape(HEADS,-1).any(dim=1).sum())
                      for name in ('q','k','v')})
    model.zero_grad(set_to_none=True)
    return {'rows':len(rows),'threshold':1e-12,'incoming_gradient_units_per_block':units,
            'parameters_unused_by_dataset':sorted(set(model.w)-participated),
            'structurally_zero_relu_units_per_block':zero_units,'attention_heads_with_gradient':heads,
            'matrix_gradient_counts':{k:int(v.sum()) for k,v in seen.items() if k.endswith('_w')},
            'gradient_fraction_all_slots':sum(int(v.sum()) for v in seen.values())/sum(v.numel() for v in seen.values()),
            'note':'All functional paths can train; scalar gradient coverage also depends on observed features. Key biases are softmax-shift redundant.'}


def successful_teacher_suffix(summary, events):
    """Select a demonstrated recovery, never the command which caused failure.

    A label's packet is the observation BEFORE its proposed action. Therefore
    the first label seeing the last failed operation can be used as recovery
    supervision. This requires actual goal arrival and reported completion.
    Missing receipts do not establish a recovery boundary.
    """
    if not (summary.get('completed') and summary.get('goal_reached')
            and not summary.get('error_type') and summary.get('blocked_moves',0)>0):
        return None
    failures=[]
    for index,event in enumerate(events):
        if event.get('event') != 'teacher_label':
            continue
        last=(event['packet'].get('observation') or {}).get('last_action') or {}
        if last.get('completed') is False:
            failures.append(index)
    return failures[-1] if failures else None


def load_episodes(root, robot, scenes):
    """Explicit whitelist; test/local/corrupted-command labels are never fitted."""
    if not set(scenes) <= set(TRAIN+RECOVERY+MAP_TRAIN+MAP_DEV+LEARNER_TRAIN):
        raise ValueError('Test scenes are forbidden in training/development')
    rows, audit = [], []
    for path in sorted(root.glob('*/summary.json')):
        summary = json.loads(path.read_text())
        if summary['robot'] != robot:
            continue
        if summary['scene'] not in scenes:
            raise ValueError('Unexpected scene in split: '+summary['scene'])
        student = summary.get('collection_protocol') == STUDENT_PROTOCOL
        if summary.get('mode') != 'collect' or summary.get('collection_protocol') not in ('complete-astra-recovery-v1',STUDENT_PROTOCOL):
            raise ValueError('Only complete Astra teacher collections may train this model')
        events=[json.loads(line) for line in path.with_name('events.jsonl').read_text().splitlines()]
        suffix = successful_teacher_suffix(summary,events) if not student and not summary.get('clean') else None
        accepted = bool(summary.get('recovery_success') and summary.get('handoff') and
                        summary.get('recovery_collision_stops')==0 and summary.get('completed') and
                        summary.get('goal_reached') and not summary.get('error_type')) if student else bool(summary.get('clean') or suffix is not None)
        record = {'run':path.parent.name, 'data_root':str(root.resolve()), 'scene':summary['scene'], 'accepted':accepted,
                  'fault':'native' if suffix is not None else summary.get('fault'),
                  'recovery_complete':True if suffix is not None else summary.get('recovery_complete'),
                  'successful_suffix_start_event':suffix,
                  'observed_feedback':summary.get('observed_feedback'), 'labels':0,
                  'events_sha256':hashlib.sha256(path.with_name('events.jsonl').read_bytes()).hexdigest()}
        audit.append(record)
        if not accepted:
            continue
        models, efforts = set(), set()
        with path.with_name('diagnostics.jsonl').open() as stream:
            for line in stream:
                diagnostic = json.loads(line)
                if diagnostic.get('event') == 'provider_response':
                    models.add(diagnostic.get('resolved_model'))
                elif diagnostic.get('event') == 'planner_request':
                    efforts.add(diagnostic['request'].get('reasoning_effort'))
        if models not in ({'gpt-6-astra'}, {ASTRA_MODEL_ID}) or efforts != {'high'}:
            raise ValueError('Teacher provenance is not Astra high')
        record['teacher_models'],record['reasoning_efforts'] = sorted(models),sorted(efforts)
        labels, recovery_labels, context_labels = Counter(), Counter(), Counter()
        fault_seen = suffix is not None
        for event_index,event in enumerate(events):
            if event.get('event') == 'injected_fault':
                fault_seen = True
            if event.get('event') == 'student_handoff':
                fault_seen = True
            if event.get('event') != 'teacher_label':
                continue
            if suffix is not None and event_index < suffix:
                continue
            if student and (not fault_seen or event.get('phase')!='recovery'):
                raise ValueError('Student prefix or unverified suffix cannot be a teacher label')
            packet, decision = event['packet'], event['decision']
            label = target(packet, decision)
            if label is None:
                continue
            actual_robot = 'ev3' if 'ir_survey' in packet['local_map_evidence'] else 'blast'
            if actual_robot != robot:
                raise ValueError('Robot identity changed in training packet')
            inputs, mask = encode(packet)
            index = ACTIONS.index(label)
            if not mask[index]:
                raise ValueError('Teacher label unavailable in its observed state')
            rows.append((inputs, mask, index, packet))
            labels[label] += 1
            reason=((packet.get('observation') or {}).get('last_action') or {}).get('reason')
            if reason:
                context_labels[reason] += 1
            if fault_seen:
                recovery_labels.update(event.get('feedback') or [])
                if student: recovery_labels.update(['student_state'])
                if suffix is not None: recovery_labels.update(['native_failure'])
        record['labels'] = sum(labels.values())
        record['label_counts'] = dict(labels)
        record['last_action_reason_label_counts'] = dict(context_labels)
        record['post_fault_feedback_labels'] = dict(recovery_labels)
        expected = {'stall':'motion_interrupted','range':'range_missing','scan':'partial_scan','route':'route_rejected',
                    'student':'student_state','native':'native_failure'}
        record['actionable_recovery_labels'] = recovery_labels.get(expected.get(record['fault']),0)
    return rows, audit


@torch.no_grad()
def evaluate(model, rows, class_weights):
    total, correct, confusion = 0.0, 0, np.zeros((len(ACTIONS),len(ACTIONS)),dtype=int)
    for inputs, mask, label, _ in rows:
        scores = model(inputs).masked_fill(~mask, -1e9)
        loss = F.cross_entropy(scores[None],torch.tensor([label]),reduction='none')[0]
        total += float(loss * class_weights[label])
        prediction = int(scores.argmax())
        confusion[label,prediction] += 1
        correct += prediction == label
    return {'loss':total/len(rows),'accuracy':correct/len(rows),'rows':len(rows),
            'recall':{a:float(confusion[i,i]/confusion[i].sum()) for i,a in enumerate(ACTIONS) if confusion[i].sum()},
            'confusion':confusion.tolist()}


def passage_loss(scores, known, safe):
    """Move probability from observed collision actions to observed safe actions.

    Unknown actions (especially an unqueried Astra delegation) supply no label.
    This loss trains the existing logits; it adds no runtime mask or risk model.
    """
    if not safe.any() or not (known & ~safe).any() or (safe & ~known).any():
        raise ValueError('Passage loss requires known safe and unsafe alternatives')
    return torch.logsumexp(scores[known],0)-torch.logsumexp(scores[safe],0)


def load_passages(root, robot, split, plan):
    from experiments.navigation_passage_data import VERSION, input_digest
    path = root/f'{robot}-{split}-risk.jsonl'
    if not path.exists():
        if plan.get('version') == VERSION:
            raise ValueError('Missing passage outcome data')
        return [], {}
    audit = json.loads((root/'risk-audit.json').read_text())
    expected = next(r['sha256'] for r in audit if r['robot']==robot and r['split']==split)
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError('Passage dataset changed after collection')
    allowed = plan['train_scenes' if split=='train' else 'development_scenes']
    rows, counts, checked = [], Counter(), set()
    for line in path.read_text().splitlines():
        row = json.loads(line)
        source = Path(row['source'])
        if (row['version']!=VERSION or row['robot']!=robot or row['split']!=split or
                row['scene'] not in allowed or row['scene'] in plan['test_scenes'] or
                source.parent.parent.name != split or
                str(source.parent.parent.parent) not in plan['risk_sources']):
            raise ValueError('Invalid passage data provenance/split')
        if source not in checked:
            if hashlib.sha256(source.read_bytes()).hexdigest()!=row['source_sha256']:
                raise ValueError('Passage source trajectory changed')
            checked.add(source)
        packet = row['packet']
        if input_digest(packet)!=row['input_sha256']:
            raise ValueError('Passage observation changed')
        inputs, available = encode(packet)
        if not row['outcomes'] or set(row['outcomes'])-set(ACTIONS):
            raise ValueError('Invalid tested actions')
        tested=torch.tensor([a in row['outcomes'] for a in ACTIONS])
        counts['unavailable_outcomes_excluded']+=int((tested & ~available).sum())
        known=tested & available
        safe=known & torch.tensor([a in row['outcomes'] and row['outcomes'][a]['collision_stops']==0 for a in ACTIONS])
        if any(r['collision_stops']<0 for r in row['outcomes'].values()):
            raise ValueError('Invalid collision label')
        for action,result in row['outcomes'].items():
            counts[action+('-blocked' if result['collision_stops'] else '-clear')]+=1
        rows.append((inputs,known,safe,packet))
    return rows, {'rows':len(rows),'counts':dict(counts),'sha256':expected,
                  'source_episodes':len(checked),'test_data_used':False}


@torch.no_grad()
def evaluate_passages(model, rows):
    losses, known_choices, blocked_choices, unknown_choices = [],0,0,0
    for inputs,known,safe,packet in rows:
        scores=model(inputs)
        if safe.any() and (known & ~safe).any():
            losses.append(float(passage_loss(scores,known,safe)))
        _,available=encode(packet)
        selected=int(scores.masked_fill(~available,-1e9).argmax())
        if known[selected]:
            known_choices+=1
            blocked_choices+=int(not safe[selected])
        else:
            unknown_choices+=1
    return {'loss':sum(losses)/len(losses) if losses else 0.0,
            'rows':len(rows),'informative_rows':len(losses),'known_choices':known_choices,
            'collision_choices':blocked_choices,'unknown_choices':unknown_choices}


def fit(args):
    plan = json.loads((args.data/'protocol.json').read_text())
    train_scenes, dev_scenes = plan['train_scenes'],plan['development_scenes']
    if set(train_scenes) & set(dev_scenes) or (set(train_scenes)|set(dev_scenes)) & set(plan['test_scenes']):
        raise ValueError('Episode split leakage')
    roots = [*getattr(args,'base_data',[]),args.data]
    for root in roots:
        source_plan = json.loads((root/'protocol.json').read_text())
        if (not set(source_plan['train_scenes']) <= set(train_scenes) or
                not set(source_plan['development_scenes']) <= set(dev_scenes)):
            raise ValueError('Base collection splits do not match')
        if any(not (root/job['split']/job['name']/'summary.json').exists() for job in source_plan['jobs']):
            raise ValueError('Declared collection is not complete')
        if any(source_plan['runtime_hashes'].get(k)!=v for k,v in plan['runtime_hashes'].items() if k.startswith('src/')):
            raise ValueError('Production adapter changed between collection rounds')
        for split in ('train','development'):
            for source in (root/split).glob('*/source-hashes.json'):
                recorded = json.loads(source.read_text())
                if any(recorded.get(k)!=v for k,v in source_plan['runtime_hashes'].items()):
                    raise ValueError('Collection runtime does not match its frozen protocol')
    args.output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(2)
    torch.use_deterministic_algorithms(True)
    configuration = plan['training']
    if configuration.get('initialization') != INITIALIZATION:
        raise ValueError('Declare dense initialization in the training protocol')
    for robot in ('ev3','blast'):
        torch.manual_seed(configuration['seed'])
        rng = np.random.default_rng(configuration['seed'])
        train_rows,train_audit,dev_rows,dev_audit = [],[],[],[]
        for root in roots:
            rows,audit=load_episodes(root/'train',robot,train_scenes)
            train_rows.extend(rows); train_audit.extend(audit)
            rows,audit=load_episodes(root/'development',robot,dev_scenes)
            dev_rows.extend(rows); dev_audit.extend(audit)
        recovery = [r for r in train_audit if r['accepted'] and r['fault']!='none'
                    and r['recovery_complete'] and r['actionable_recovery_labels']>0]
        required=plan.get('required_training_contexts',{}).get(robot,[])
        coverage=Counter()
        for run in train_audit:
            if run['accepted']:
                coverage.update(run.get('last_action_reason_label_counts',{}))
        if required and not any(coverage[reason] for reason in required):
            raise ValueError('Missing actual recovery labels for '+robot+': '+', '.join(required))
        if len(train_rows)<10 or not dev_rows or not recovery:
            raise ValueError('Insufficient complete recovery/development episodes for '+robot)
        risk_train,risk_train_audit=load_passages(args.data,robot,'train',plan)
        risk_dev,risk_dev_audit=load_passages(args.data,robot,'development',plan)
        informative=[r for r in risk_train if r[2].any() and (r[1] & ~r[2]).any()]
        risk_weight=configuration.get('risk_weight',0.0)
        if risk_weight and not informative:
            raise ValueError('No contrasting passage outcomes in training')
        reference_path=args.reference/(robot+'.npz') if args.reference else None
        reference=AttentionPolicy.load(reference_path) if reference_path else None
        model = initialize_model(train_rows+risk_train,configuration['seed'])
        initial = AttentionPolicy(model.arrays())
        for inputs,_,_,packet in train_rows+dev_rows+risk_train+risk_dev:
            np.testing.assert_allclose(model(inputs).detach().numpy(),initial.scores(packet),rtol=2e-4,atol=.005)
        counts = np.bincount([r[2] for r in train_rows],minlength=len(ACTIONS))
        cw = np.sqrt(len(train_rows)/np.maximum(counts,1))
        cw /= cw[counts>0].mean()
        weights = torch.tensor(cw,dtype=torch.float32)
        initial_path=args.output/(robot+'-initial.npz')
        save_weights(initial_path,initial.weights,robot=robot,method=INITIALIZATION+'-untrained')
        initial_gradients=gradient_audit(model,train_rows)
        optimizer = torch.optim.AdamW(model.parameters(),lr=configuration['learning_rate'],
                                     weight_decay=configuration['weight_decay'])
        baseline = evaluate(model,dev_rows,weights)
        risk_baseline=evaluate_passages(model,risk_dev)
        reference_metrics=evaluate(TrainableAttention(reference.weights),dev_rows,weights) if reference else None
        reference_risk=evaluate_passages(TrainableAttention(reference.weights),risk_dev) if reference else None
        curve, best, best_epoch, best_loss = [], model.arrays(), 0, baseline['loss']+risk_weight*risk_baseline['loss']
        for epoch in range(1,configuration['epochs']+1):
            optimizer.zero_grad()
            order = rng.permutation(len(train_rows)+len(informative))
            for offset, index in enumerate(order):
                if index < len(train_rows):
                    inputs,mask,label,_ = train_rows[index]
                    scores = model(inputs).masked_fill(~mask,-1e9)
                    loss = F.cross_entropy(scores[None],torch.tensor([label]),reduction='none')[0]*weights[label]
                    loss *= len(order)/len(train_rows)
                else:
                    inputs,known,safe,_=informative[index-len(train_rows)]
                    loss=passage_loss(model(inputs),known,safe)*risk_weight*len(order)/len(informative)
                if not torch.isfinite(loss):
                    raise ValueError('Nonfinite training loss')
                size = min(configuration['batch_size'],len(order)-(offset//configuration['batch_size'])*configuration['batch_size'])
                (loss/size).backward()
                if (offset+1)%configuration['batch_size']==0 or offset+1==len(order):
                    nn.utils.clip_grad_norm_(model.parameters(),1.0,error_if_nonfinite=True)
                    optimizer.step()
                    optimizer.zero_grad()
            metrics = evaluate(model,dev_rows,weights)
            risk_metrics=evaluate_passages(model,risk_dev)
            combined=metrics['loss']+risk_weight*risk_metrics['loss']
            curve.append({'epoch':epoch,**metrics,'passage':risk_metrics,'combined_loss':combined})
            if combined < best_loss:
                best,best_epoch,best_loss = model.arrays(),epoch,combined
            print(json.dumps({'robot':robot,'epoch':epoch,'dev_loss':metrics['loss'],'dev_accuracy':metrics['accuracy'],
                              'passage_loss':risk_metrics['loss'],'collision_choices':risk_metrics['collision_choices']}),flush=True)
            if epoch-best_epoch >= configuration['patience']:
                break
        selected = TrainableAttention(best)
        selected_gradients=gradient_audit(selected,train_rows)
        exported = AttentionPolicy(best)
        for inputs,_,_,packet in train_rows+dev_rows+risk_train+risk_dev:
            _,mask=encode(packet)
            torch_scores = selected(inputs).detach().numpy()
            np.testing.assert_allclose(torch_scores,exported.scores(packet),rtol=2e-4,atol=.005)
            assert int(torch.tensor(torch_scores).masked_fill(~mask,-1e9).argmax()) == ACTIONS.index(exported.predict(packet)[0])
        path = args.output/(robot+'.npz')
        method='map-aware-passage-supervision-v1' if risk_weight else METHOD
        save_weights(path,best,robot=robot,method=method)
        report = {'robot':robot,'method':method,'configuration':configuration,'selected_epoch':best_epoch,
            'training_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'protocol_sha256':hashlib.sha256((args.data/'protocol.json').read_bytes()).hexdigest(),
            'data_protocols':{str(root.resolve()):hashlib.sha256((root/'protocol.json').read_bytes()).hexdigest() for root in roots},
            'initial_sha256':hashlib.sha256(initial_path.read_bytes()).hexdigest(),
            'reference_sha256':hashlib.sha256(reference_path.read_bytes()).hexdigest() if reference_path else None,
            'reference_development':reference_metrics,
            'passage_training_data':risk_train_audit,'passage_development_data':risk_dev_audit,
            'passage_initial_development':risk_baseline,'passage_reference_development':reference_risk,
            'passage_development_has_contrasts':risk_baseline['informative_rows']>0,
            'passage_trained_development':evaluate_passages(selected,risk_dev),
            'passage_trained_fit':evaluate_passages(selected,risk_train),
            'initial_gradient_audit':initial_gradients,'selected_gradient_audit':selected_gradients,
            'sha256':hashlib.sha256(path.read_bytes()).hexdigest(), 'baseline_development':baseline,
            'trained_development':evaluate(selected,dev_rows,weights), 'trained_fit':evaluate(selected,train_rows,weights),
            'train_episodes':train_audit,'development_episodes':dev_audit,'curve':curve,
            'train_label_counts':dict(Counter(ACTIONS[r[2]] for r in train_rows)),
            'train_last_action_reason_label_counts':dict(coverage),
            'complete_recovery_faults':dict(Counter(r['fault'] for r in recovery)),
            'changed_parameters':sum(int(np.count_nonzero(best[k]!=v)) for k,v in initial.weights.items()),
            'test_data_used':False,'physical_robots':False,
            'limits':'Limited observed-state demonstrations and offline action outcomes. Unknown geometry and unqueried Astra delegations remain unknown; not proof of reliable navigation.'}
        (args.output/(robot+'-training.json')).write_text(json.dumps(report,indent=2))


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data',type=Path,required=True)
    parser.add_argument('--base-data',type=Path,action='append',default=[])
    parser.add_argument('--reference',type=Path,
                        help='Previous models for reporting only, never used as initialization or fallback')
    parser.add_argument('--output',type=Path,required=True)
    fit(parser.parse_args())
