"""Offline supervision must not change policy inputs or execution semantics."""
import copy
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiments.navigation_passage_data import (
    VERSION, input_digest, probe, sources, selected_indices,
    ControllerActionDecision, ControllerActionPlannerResult,
)
from scripts.local_navigation_experiment import scenario,run_ev3,run_blast_gemma_scenario,PASSAGE_TEST,MAP_LAYOUTS


class PassageDataTests(unittest.TestCase):
    def test_only_temporal_sampling_no_environment_rules(self):
        rows=[{'packet':{},'decision':{'action':'OBSERVE'}} for _ in range(12)]
        rows[7]['packet']={'observation':{'last_action':{'completed':False}}}
        rows[-1]['decision']['action']='COMPLETE'
        self.assertEqual(selected_indices(rows),[0,4,5,6,7,8])

    def test_test_scenes_are_novel_whole_worlds(self):
        previous={value for key,value in MAP_LAYOUTS.items() if key not in PASSAGE_TEST}
        for name in PASSAGE_TEST:
            self.assertNotIn(MAP_LAYOUTS[name],previous)
            self.assertNotEqual(scenario('ev3',name).robots[0].footprint,
                                scenario('blast',name).robots[0].footprint)

    def test_outcome_labels_never_become_model_inputs(self):
        from experiments.navigation_attention_audit import fixture
        p=fixture('ev3'); expected=input_digest(p)
        p.update(world={'obstacles':[1,2,3]},outcomes={'ADVANCE':{'collision_stops':9}},scene='secret')
        self.assertEqual(input_digest(p),expected)

    def test_test_collection_cannot_be_replayed_for_training(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); path=root/'train'/'bad'; path.mkdir(parents=True)
            (path/'events.jsonl').write_text(json.dumps({'robot':'ev3','scene':PASSAGE_TEST[0],
                'mode':'collect','fault':'none'})+'\n')
            with self.assertRaisesRegex(ValueError,'training/development'):
                sources([root],'train','ev3',5)

    def test_previous_new_and_astra_share_the_same_frozen_test_worlds_and_budget(self):
        from experiments.navigation_recovery_benchmark import compare,world_hash
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for name in ('new','previous'):
                (root/name).mkdir()
                for robot in ('ev3','blast'):
                    (root/name/(robot+'.npz')).write_bytes((name+robot).encode())
            (root/'protocol.json').write_text(json.dumps({'test_scenes':list(PASSAGE_TEST),
                'test_decisions':128,'test_world_hashes':{r+'-'+s:world_hash(r,s)
                    for r in ('ev3','blast') for s in PASSAGE_TEST}}))
            with patch('experiments.navigation_recovery_benchmark.execute'),patch('builtins.print'):
                compare(root,root/'new',root/'previous')
            jobs=json.loads((root/'evaluation.json').read_text())['jobs']
            self.assertEqual(len(jobs),18)
            self.assertEqual({j['decisions'] for j in jobs},{128})
            for robot in ('ev3','blast'):
                for scene in PASSAGE_TEST:
                    selected=[j for j in jobs if j['robot']==robot and j['scene']==scene]
                    self.assertEqual({j['arm'] for j in selected},{'astra','local','previous'})
                    self.assertEqual(len({j.get('weight_sha256') for j in selected}),3)

    def test_trial_audit_checks_provider_weights_and_actual_executed_choice(self):
        from experiments.navigation_recovery_benchmark import audit_trial
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            (root/'source-hashes.json').write_text(json.dumps({'src/runtime.py':'frozen'}))
            (root/'diagnostics.jsonl').write_text('\n'.join(json.dumps(r) for r in (
                {'event':'provider_response','resolved_model':'gpt-6-astra'},
                {'event':'planner_request','request':{'reasoning_effort':'high'}})))
            events=[{'event':'policy_dispatch','selection':'OBSERVE'},
                    {'event':'decision_pair','packet':{'active_waypoint':{'x_mm':800}},
                     'decision':{'action':'OBSERVE'}}]
            def save(): (root/'events.jsonl').write_text('\n'.join(map(json.dumps,events)))
            save()
            with patch('experiments.local_navigation.LocalPolicy.load',return_value=SimpleNamespace(predict=lambda p:('OBSERVE',1))):
                result=audit_trial(root,{'weights':'unused'}, {'teacher':'codex/gpt-6-astra','runtime_hashes':{'src/runtime.py':'frozen'}})
                self.assertEqual(result['learned_decisions_replayed'],1)
                events[-1]['decision']['action']='ADVANCE';save()
                with self.assertRaisesRegex(ValueError,'replaced'):
                    audit_trial(root,{'weights':'unused'},{'teacher':'codex/gpt-6-astra','runtime_hashes':{'src/runtime.py':'frozen'}})

    def test_real_adapter_prefix_and_single_action_reproduce_for_both_robots(self):
        # Capture one ordinary initial observation; replay it into one bounded OBSERVE.
        for robot in ('ev3','blast'):
            with self.subTest(robot=robot):
                captured=[]
                class Planner:
                    def decide(self,context):
                        decision=ControllerActionDecision('OBSERVE',1000,'test',('OBSERVE',),None)
                        captured.append({'packet':context.to_dict(),'decision':asdict(decision)})
                        return ControllerActionPlannerResult(decision,0)
                scene=scenario(robot,'map-train-0')
                if robot=='ev3':
                    run_ev3(scene,Planner(),1,lambda event:None)
                else:
                    run_blast_gemma_scenario(scene,max_decisions=1,planner_factory=lambda _:Planner(),
                        minimum_forward_progress_mm=800,
                        goal='Reach the goal 80 cm straight ahead of your starting position.')
                observed,blocked=probe(robot,'map-train-0',captured,0,'OBSERVE')
                self.assertEqual(blocked,0)
                self.assertEqual(input_digest(observed),input_digest(captured[0]['packet']))
                corrupted=copy.deepcopy(captured)
                corrupted[0]['packet']['local_map_evidence']['robot_pose']['x_mm']=999
                fresh,_=probe(robot,'map-train-0',corrupted,0,'OBSERVE')
                self.assertEqual(input_digest(fresh),input_digest(observed))
                self.assertNotEqual(input_digest(fresh),input_digest(corrupted[0]['packet']))


@unittest.skipUnless(importlib.util.find_spec('torch'),'Optional offline training dependency')
class PassageTrainingTests(unittest.TestCase):
    def test_loss_reduces_unsafe_action_without_supervising_unknown_delegation(self):
        import torch
        from experiments.navigation_attention_train import passage_loss
        logits=torch.tensor([2.,1.,3.],requires_grad=True)
        loss=passage_loss(logits,torch.tensor([True,True,False]),torch.tensor([False,True,False]))
        loss.backward()
        self.assertGreater(float(logits.grad[0]),0)
        self.assertLess(float(logits.grad[1]),0)
        self.assertEqual(float(logits.grad[2]),0)
        with self.assertRaises(ValueError):
            passage_loss(logits,torch.tensor([True,False,False]),torch.tensor([False,False,False]))

    def test_unknown_action_is_reported_not_counted_as_safe(self):
        import torch
        from experiments.navigation_attention_train import encode,evaluate_passages
        from experiments.navigation_attention_audit import fixture
        packet=fixture('ev3'); inputs,_=encode(packet)
        class Model:
            def __call__(self,inputs):return torch.tensor([0.,0.,0.,0.,0.,0.,0.,10.])
        metrics=evaluate_passages(Model(),[(inputs,torch.tensor([True]*7+[False]),
            torch.tensor([False,True,True,True,True,True,True,False]),packet)])
        self.assertEqual(metrics['unknown_choices'],1)
        self.assertEqual(metrics['known_choices'],0)

    def test_loader_enforces_split_and_original_observations(self):
        from experiments.navigation_attention_train import load_passages
        from experiments.navigation_attention_audit import fixture
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); source=root/'original'/'train'/'episode'/'events.jsonl'
            source.parent.mkdir(parents=True);source.write_text('original trajectory')
            packet=fixture('ev3')
            row={'version':VERSION,'robot':'ev3','split':'train','scene':'map-train-0',
                 'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                 'packet':packet,'input_sha256':input_digest(packet),
                 'outcomes':{'OBSERVE':{'collision_stops':0},'FOLLOW_WAYPOINT':{'collision_stops':1}}}
            path=root/'ev3-train-risk.jsonl'
            def save():
                path.write_text(json.dumps(row)+'\n')
                (root/'risk-audit.json').write_text(json.dumps([{'robot':'ev3','split':'train',
                    'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}]))
            plan={'train_scenes':['map-train-0'],'test_scenes':list(PASSAGE_TEST),
                  'risk_sources':{str(root/'original'):'hash'}}
            save(); rows,audit=load_passages(root,'ev3','train',plan)
            self.assertEqual(len(rows),1);self.assertFalse(audit['test_data_used'])
            row['packet']['available_actions']=[a for a in packet['available_actions'] if a!='FOLLOW_WAYPOINT']
            row['input_sha256']=input_digest(row['packet']);save()
            rows,audit=load_passages(root,'ev3','train',plan)
            self.assertFalse(rows[0][1][0])
            self.assertEqual(audit['counts']['unavailable_outcomes_excluded'],1)
            row['scene']=PASSAGE_TEST[0];save()
            with self.assertRaisesRegex(ValueError,'provenance/split'):
                load_passages(root,'ev3','train',plan)
