import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

@unittest.skipUnless(importlib.util.find_spec('torch'), 'Optional offline training dependency')
class AttentionTrainingTests(unittest.TestCase):
    def setUp(self):
        import torch
        torch.set_num_threads(1)
        from experiments.navigation_attention_audit import fixture
        self.packet=fixture('ev3')

    def test_torch_forward_matches_unmodified_numpy_runtime(self):
        import numpy as np
        from experiments.navigation_attention_policy import AttentionPolicy,empty_weights
        from experiments.navigation_attention_train import TrainableAttention,encode
        rng=np.random.default_rng(6)
        weights={k:rng.normal(0,.01,v.shape).astype(np.float32) for k,v in empty_weights().items()}
        model=TrainableAttention(weights)
        for packet in (self.packet,{}, {**self.packet,'local_map_evidence':{}}):
            p=copy.deepcopy(packet)
            p.setdefault('available_actions',[])
            inputs,_=encode(p)
            np.testing.assert_allclose(model(inputs).detach().numpy(),AttentionPolicy(weights).scores(p),rtol=1e-5,atol=1e-5)

    def test_dense_initialization_is_reproducible_and_has_no_zero_relu_units(self):
        import numpy as np
        from experiments.navigation_attention_train import initialize_weights
        from experiments.navigation_attention_policy import shapes,BLOCKS,AttentionPolicy
        a,b,c=initialize_weights(23),initialize_weights(23),initialize_weights(24)
        self.assertEqual(set(a),set(shapes()))
        self.assertEqual(sum(v.size for v in a.values()),553352)
        for key in a:
            np.testing.assert_array_equal(a[key],b[key])
            self.assertFalse(np.shares_memory(a[key],b[key]))
            if key.endswith('_w'):
                self.assertTrue(np.all(np.any(a[key]!=0,axis=0)))
                self.assertFalse(np.array_equal(a[key],c[key]))
        for i in range(BLOCKS):
            self.assertFalse(np.any(np.all(a[f'b{i}_ff1_w']==0,axis=0)))
        self.assertTrue(np.isfinite(AttentionPolicy(a).scores(self.packet)).all())

    def test_all_hidden_units_and_attention_heads_receive_gradient_and_update(self):
        import numpy as np
        import torch
        from experiments.navigation_attention_train import initialize_model,gradient_audit
        from experiments.navigation_attention_policy import GLOBAL_SIZE,RAY_FIELDS,MEMORY_FIELDS,MAP_FIELDS,BLOCKS
        generator=torch.Generator().manual_seed(19)
        rows=[(tuple(torch.randn((n,d),generator=generator) for n,d in
                     ((1,GLOBAL_SIZE),(16,len(RAY_FIELDS)),(16,len(MEMORY_FIELDS)),(16,len(MAP_FIELDS)))),
               torch.ones(8,dtype=torch.bool),i%8,{}) for i in range(16)]
        model=initialize_model(rows,23); initial=model.arrays()
        repeated=initialize_model(rows,23).arrays()
        for key in initial:np.testing.assert_array_equal(initial[key],repeated[key])
        audit=gradient_audit(model,rows)
        self.assertEqual(audit['incoming_gradient_units_per_block'],[384]*3)
        self.assertEqual(audit['structurally_zero_relu_units_per_block'],[0]*3)
        self.assertTrue(all(all(v==4 for v in heads.values()) for heads in audit['attention_heads_with_gradient']))
        self.assertTrue(all(v>0 for v in audit['matrix_gradient_counts'].values()))
        optimizer=torch.optim.AdamW(model.parameters(),lr=1e-4,weight_decay=0)
        for inputs,mask,label,_ in rows:
            torch.nn.functional.cross_entropy(model(inputs)[None],torch.tensor([label])).backward()
        optimizer.step()
        for i in range(BLOCKS):
            self.assertTrue(np.all(np.any(model.arrays()[f'b{i}_ff1_w']!=initial[f'b{i}_ff1_w'],axis=0)))

    def test_gradient_audit_handles_empty_optional_streams_across_dataset(self):
        import torch
        from experiments.navigation_attention_train import initialize_model,gradient_audit
        from experiments.navigation_attention_policy import GLOBAL_SIZE,RAY_FIELDS,MEMORY_FIELDS,MAP_FIELDS
        generator=torch.Generator().manual_seed(17)
        dims=(GLOBAL_SIZE,len(RAY_FIELDS),len(MEMORY_FIELDS),len(MAP_FIELDS))
        full=tuple(torch.randn((1 if i==0 else 4,d),generator=generator) for i,d in enumerate(dims))
        empty=(full[0],*(torch.empty((0,d)) for d in dims[1:]))
        mask=torch.ones(8,dtype=torch.bool)
        rows=[(empty,mask,0,{}),(full,mask,6,{})]
        model=initialize_model(rows,23)
        missing=gradient_audit(model,rows[:1])['parameters_unused_by_dataset']
        self.assertEqual(set(missing),{name+suffix for name in ('ray','memory','map') for suffix in ('_w','_b')})
        audit=gradient_audit(model,rows)
        self.assertEqual(audit['parameters_unused_by_dataset'],[])
        self.assertGreater(audit['matrix_gradient_counts']['map_w'],0)

    def test_gradient_update_exports_to_same_numeric_model(self):
        import numpy as np
        import torch
        from experiments.navigation_attention_policy import AttentionPolicy,empty_weights,save_weights
        from experiments.navigation_attention_train import TrainableAttention,encode,METHOD
        model=TrainableAttention(empty_weights())
        inputs,mask=encode(self.packet)
        optimizer=torch.optim.AdamW(model.parameters(),lr=.01,weight_decay=0)
        before=float(model(inputs)[6].detach())
        for _ in range(3):
            optimizer.zero_grad()
            loss=torch.nn.functional.cross_entropy(model(inputs).masked_fill(~mask,-1e9)[None],torch.tensor([6]))
            loss.backward()
            optimizer.step()
        self.assertGreater(float(model(inputs)[6].detach()),before)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'trained.npz'
            save_weights(path,model.arrays(),robot='ev3',method=METHOD)
            exported=AttentionPolicy.load(path)
            np.testing.assert_allclose(exported.scores(self.packet),model(inputs).detach().numpy(),atol=1e-5)
            with np.load(path,allow_pickle=False) as archive:
                self.assertEqual(json.loads(str(archive['metadata']))['method'],METHOD)

    def test_only_successful_suffix_after_last_failed_operation_is_recovery_data(self):
        from experiments.navigation_attention_train import successful_teacher_suffix
        summary={'completed':True,'goal_reached':True,'blocked_moves':1}
        def label(completed):
            return {'event':'teacher_label','packet':{'observation':{'last_action':{'completed':completed}}}}
        events=[label(True),label(False),label(True),label(False),label(True)]
        self.assertEqual(successful_teacher_suffix(summary,events),3)
        self.assertIsNone(successful_teacher_suffix({**summary,'goal_reached':False},events))
        self.assertIsNone(successful_teacher_suffix({**summary,'completed':False},events))
        self.assertIsNone(successful_teacher_suffix({**summary,'error_type':'RuntimeError'},events))
        self.assertIsNone(successful_teacher_suffix(summary,[label(None),label(True)]))

    def test_failed_runs_and_corrupted_commands_are_not_teacher_labels(self):
        from experiments.navigation_attention_train import load_episodes
        from experiments.local_navigation import ACTIONS
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for clean,name in ((True,'success'),(False,'failure')):
                folder=root/name
                folder.mkdir()
                (folder/'summary.json').write_text(json.dumps({'robot':'ev3','scene':'train-open',
                    'mode':'collect','collection_protocol':'complete-astra-recovery-v1','clean':clean,'fault':'stall',
                    'recovery_complete':clean,'observed_feedback':['motion_interrupted']}))
                row={'packet':self.packet,'decision':{'action':'OBSERVE'}}
                (folder/'events.jsonl').write_text('\n'.join(json.dumps({'event':event,**row}) for event in
                    ('teacher_label','decision_pair','teacher_command_corrupted'))+'\n')
                (folder/'diagnostics.jsonl').write_text('\n'.join(json.dumps(row) for row in (
                    {'event':'provider_response','resolved_model':'gpt-6-astra'},
                    {'event':'planner_request','request':{'reasoning_effort':'high'}}))+'\n')
            rows,audit=load_episodes(root,'ev3',['train-open'])
            self.assertEqual(len(rows),1)
            self.assertEqual(rows[0][2],ACTIONS.index('OBSERVE'))
            self.assertEqual(sum(r['accepted'] for r in audit),1)
            with self.assertRaisesRegex(ValueError,'Test scenes'):
                load_episodes(root,'ev3',['test-dogleg'])

    def test_new_test_worlds_are_separate_from_training_and_have_robot_footprints(self):
        from scripts.local_navigation_experiment import TRAIN,RECOVERY,RECOVERY_TEST,DENSE_TEST,STUDENT_TEST,TEST,ATTENTION_HELD_OUT,scenario
        self.assertFalse(set(RECOVERY_TEST)&set(TRAIN+RECOVERY))
        for scene in RECOVERY_TEST:
            ev3,blast=scenario('ev3',scene),scenario('blast',scene)
            self.assertEqual(ev3.obstacles,blast.obstacles)
            self.assertNotEqual(ev3.robots[0].footprint,blast.robots[0].footprint)
            self.assertEqual(ev3.goals[0].x_mm,blast.goals[0].x_mm)
        geometry=lambda scene:tuple((o.min_x_mm,o.min_y_mm,o.max_x_mm,o.max_y_mm) for o in scene.obstacles)
        old=[geometry(scenario('ev3',name)) for name in TRAIN+RECOVERY+RECOVERY_TEST+STUDENT_TEST+TEST+ATTENTION_HELD_OUT]
        for name in DENSE_TEST:
            ev3,blast=scenario('ev3',name),scenario('blast',name)
            self.assertNotIn(geometry(ev3),old)
            self.assertEqual(ev3.obstacles,blast.obstacles)
            self.assertNotEqual(ev3.robots[0].footprint,blast.robots[0].footprint)

if __name__=='__main__': unittest.main()
