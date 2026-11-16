import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from experiments.navigation_attention_audit import fixture
from experiments.navigation_student_recovery import StudentRecovery, PROTOCOL
from robot_agent.lm_studio_controller_action import ControllerActionDecision, ControllerActionPlannerResult


class StudentRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.packet=fixture('blast')
        self.packet['observation']={'sensors':{'distance_mm':900},'last_motion':{'completed':True}}
        self.packet['history']=[]
        self.packet['local_map_evidence'].pop('latest_route_rejection',None)
        self.events=[]
        self.contexts=[]
        self.teacher=SimpleNamespace(decide=lambda c:self.teach(c))
        self.collector=StudentRecovery(self.teacher,SimpleNamespace(predict=lambda p:('FOLLOW_WAYPOINT',1)),self.events.append)

    def teach(self, context):
        self.contexts.append(context)
        return ControllerActionPlannerResult(ControllerActionDecision(action='OBSERVE',confidence_milli=900,
            assessment='Real teacher stub',plan=('OBSERVE',),utterance=None),0)

    def context(self):
        return SimpleNamespace(to_dict=lambda:self.packet,completion_allowed=False,active_waypoint=self.packet['active_waypoint'])

    def test_same_episode_handoff_preserves_context_and_excludes_student_labels(self):
        self.collector.decide(self.context())
        self.assertFalse(any(e['event']=='teacher_label' for e in self.events))
        self.packet['observation']['last_motion']={'completed':False,'kind':'translation'}
        context=self.context(); before=copy.deepcopy(self.packet)
        self.collector.blocked=3
        self.collector.decide(context)
        self.assertIs(self.contexts[0],context)
        self.assertEqual(self.packet,before)
        self.assertEqual(self.collector.handoff['prefix_collision_stops'],3)
        self.packet['observation']['last_motion']={'completed':True}
        self.collector.decide(self.context())
        self.assertEqual(len(self.contexts),2)
        labels=[e for e in self.events if e['event']=='teacher_label']
        self.assertTrue(all(e['phase']=='recovery' for e in labels))
        self.assertTrue(self.collector.summary({'completed':True,'goal_reached':True})['recovery_success'])
        self.collector.blocked+=1
        self.assertFalse(self.collector.summary({'completed':True,'goal_reached':True})['recovery_success'])

    def test_no_echo_alone_does_not_trigger_handoff(self):
        self.collector.last_source='local';self.collector.last_action='OBSERVE'
        self.packet['observation']['sensors']['distance_mm']=None
        self.assertIsNone(self.collector.failure(self.packet))
        self.assertIsNone(self.collector.failure(self.packet))
        self.assertEqual(self.collector.failure(self.packet),'repeated_stationary_action')

    def test_sensor_stop_hands_off_with_stale_successful_motion_intact(self):
        self.collector.decide(self.context())
        self.packet['observation']['last_action']={'action':'FOLLOW_WAYPOINT','completed':False,'reason':'near_obstacle'}
        self.packet['history']=[{'action':'FOLLOW_WAYPOINT','result':'near_obstacle','action_completed':False}]
        self.packet['execution']={'forward_admission':{'permitted':False,'reason':'blocked_hysteresis_hold'}}
        before=copy.deepcopy(self.packet)
        self.collector.decide(self.context())
        self.assertEqual(self.collector.handoff['reason'],'motion_interrupted')
        self.assertEqual(self.packet,before)
        self.assertTrue(self.packet['observation']['last_motion']['completed'])
        self.assertEqual(len(self.contexts),1)

    def test_incomplete_turn_with_verified_progress_is_not_a_failure(self):
        self.collector.last_source='local';self.collector.last_action='FOLLOW_WAYPOINT'
        self.packet['observation']['last_action']={'action':'FOLLOW_WAYPOINT','completed':False,'reason':None}
        self.packet['observation']['last_motion']={'action':'TURN_LEFT_90','kind':'turn',
            'requested_turn_deg':110.4,'measured_turn_deg':88.2,'completed':False,'made_progress':True,'reason':None}
        self.assertIsNone(self.collector.failure(self.packet))
        self.assertIsNone(self.collector.handoff)

    def test_successful_observation_does_not_reuse_older_motion_failure(self):
        self.collector.last_source='local';self.collector.last_action='OBSERVE'
        self.packet['observation']['last_action']={'action':'OBSERVE','completed':True,'observation_acquired':True}
        self.packet['observation']['last_motion']={'kind':'translation','completed':False,'made_progress':False}
        self.assertIsNone(self.collector.failure(self.packet))

    def test_geometric_rejection_hands_off_without_fabricated_motion_failure(self):
        self.collector.decide(self.context())
        rejection={'reason':'KNOWN_ECHO_CLEARANCE_INTERSECTION','blocking_echo_point':{'x_mm':410,'y_mm':90}}
        self.packet['observation']['last_action']={'action':'FOLLOW_WAYPOINT','completed':None,'reason':rejection['reason']}
        self.packet['history']=[{'action':'FOLLOW_WAYPOINT','route_rejection':rejection}]
        self.packet['local_map_evidence']['latest_route_rejection']={'rejection':rejection,'repeat_count':1}
        before=copy.deepcopy(self.packet)
        self.collector.decide(self.context())
        self.assertEqual(self.collector.handoff['reason'],'route_rejected')
        self.assertEqual(self.packet,before)
        self.assertEqual(len(self.contexts),1)

    def test_collision_counter_does_not_change_simulator_events(self):
        from scripts.local_navigation_experiment import scenario
        world=scenario('blast','train-open').build()
        count=len(world.events)
        with self.collector:
            world._record('blast','blocked',0)
        self.assertEqual(self.collector.blocked,1)
        self.assertEqual(len(world.events),count+1)

    def test_loader_uses_only_successful_teacher_suffix(self):
        from experiments.navigation_attention_train import load_episodes
        self.packet['observation']['last_action']={'action':'FOLLOW_WAYPOINT','completed':None,
            'reason':'KNOWN_ECHO_CLEARANCE_INTERSECTION'}
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); folder=root/'run';folder.mkdir()
            summary={'robot':'blast','scene':'train-left','mode':'collect','collection_protocol':PROTOCOL,
                'fault':'student','clean':False,'blocked_moves':3,'handoff':{'decision_index':2},
                'recovery_success':True,'recovery_complete':True,'recovery_collision_stops':0,
                'completed':True,'goal_reached':True}
            (folder/'summary.json').write_text(json.dumps(summary))
            row={'packet':self.packet,'decision':{'action':'OBSERVE'}}
            events=[{'event':'decision_pair',**row},{'event':'student_handoff'},
                    {'event':'teacher_label','phase':'recovery',**row}]
            (folder/'events.jsonl').write_text('\n'.join(map(json.dumps,events)))
            (folder/'diagnostics.jsonl').write_text('\n'.join(map(json.dumps,[
                {'event':'provider_response','resolved_model':'gpt-6-astra'},
                {'event':'planner_request','request':{'reasoning_effort':'high'}}])))
            rows,audit=load_episodes(root,'blast',['train-left'])
            self.assertEqual(len(rows),1)
            self.assertEqual(audit[0]['actionable_recovery_labels'],1)
            self.assertEqual(audit[0]['last_action_reason_label_counts'],{'KNOWN_ECHO_CLEARANCE_INTERSECTION':1})
            events[0]={'event':'teacher_label','phase':'recovery',**row}
            (folder/'events.jsonl').write_text('\n'.join(map(json.dumps,events)))
            with self.assertRaisesRegex(ValueError,'prefix'):
                load_episodes(root,'blast',['train-left'])

    def test_new_learner_worlds_are_separate_from_every_previous_split(self):
        from scripts.local_navigation_experiment import (LEARNER_TRAIN,LEARNER_TEST,MAP_TRAIN,MAP_DEV,MAP_TEST,
            TRAIN,RECOVERY,TEST,ATTENTION_HELD_OUT,RECOVERY_TEST,STUDENT_TEST,DENSE_TEST,scenario)
        old=TRAIN+RECOVERY+TEST+ATTENTION_HELD_OUT+RECOVERY_TEST+STUDENT_TEST+DENSE_TEST+MAP_TRAIN+MAP_DEV+MAP_TEST
        signature=lambda s:tuple((o.min_x_mm,o.min_y_mm,o.max_x_mm,o.max_y_mm) for o in s.obstacles)
        previous={signature(scenario('ev3',s)) for s in old};new=[]
        for name in LEARNER_TRAIN+LEARNER_TEST:
            ev3,blast=scenario('ev3',name),scenario('blast',name)
            self.assertEqual(ev3.obstacles,blast.obstacles)
            self.assertNotEqual(ev3.robots[0].footprint,blast.robots[0].footprint)
            shape=signature(ev3)
            self.assertNotIn(shape,previous)
            self.assertNotIn(shape,new)
            new.append(shape)

    def test_student_protocol_freezes_new_tests_and_reuses_only_base_training_splits(self):
        from unittest.mock import patch
        from experiments.navigation_recovery_benchmark import collect_student
        from scripts.local_navigation_experiment import MAP_TRAIN,MAP_DEV,LEARNER_TRAIN,LEARNER_TEST
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);base=root/'base';base.mkdir();weights=root/'weights';weights.mkdir()
            for robot in ('ev3','blast'):(weights/(robot+'.npz')).write_bytes(b'fixture')
            (base/'protocol.json').write_text(json.dumps({'runtime_hashes':{'src/example.py':'same'},
                'train_scenes':list(MAP_TRAIN),'development_scenes':list(MAP_DEV),'training':{'seed':23}}))
            with patch('experiments.navigation_recovery_benchmark.runtime_hashes',return_value={'src/example.py':'same'}), \
                    patch('experiments.navigation_recovery_benchmark.execute') as execute:
                collect_student(root/'new',weights,base)
            plan=json.loads((root/'new/protocol.json').read_text())
            self.assertEqual(set(plan['train_scenes']),set(MAP_TRAIN+LEARNER_TRAIN))
            self.assertEqual(plan['development_scenes'],list(MAP_DEV))
            self.assertEqual(plan['test_scenes'],list(LEARNER_TEST))
            self.assertEqual(plan['test_decisions'],128)
            from robot_agent.blast_episode_adapter import BlastEpisodeRuntimeAdapter
            BlastEpisodeRuntimeAdapter(controller=SimpleNamespace(snapshot=lambda:None,command=lambda *a,**k:None),
                planner_factory=lambda _:None,max_decisions=plan['test_decisions'])
            self.assertEqual(len(plan['jobs']),12)
            self.assertTrue(all(j['student_recovery'] and j['fault']=='none' and j['split']=='train' for j in plan['jobs']))
            self.assertFalse(set(j['scene'] for j in plan['jobs']) & set(LEARNER_TEST))
            execute.assert_called_once()

    def test_final_layouts_are_new_and_shared_between_robot_bodies(self):
        from scripts.local_navigation_experiment import STUDENT_TEST,TRAIN,RECOVERY,TEST,ATTENTION_HELD_OUT,RECOVERY_TEST,scenario
        old=TRAIN+RECOVERY+TEST+ATTENTION_HELD_OUT+RECOVERY_TEST
        signatures=lambda scene:tuple((o.min_x_mm,o.min_y_mm,o.max_x_mm,o.max_y_mm) for o in scene.obstacles)
        for name in STUDENT_TEST:
            ev3,blast=scenario('ev3',name),scenario('blast',name)
            self.assertEqual(ev3.obstacles,blast.obstacles)
            self.assertNotEqual(ev3.robots[0].footprint,blast.robots[0].footprint)
            self.assertNotIn(signatures(ev3),[signatures(scenario('ev3',s)) for s in old])


if __name__=='__main__':unittest.main()
