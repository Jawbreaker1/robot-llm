import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from experiments.navigation_attention_policy import empty_weights,save_weights
from experiments.navigation_physical_trial import build_adapter,MODEL
from robot_agent.lm_studio_controller_action import ControllerActionDecision,ControllerActionPlannerResult


class PhysicalLocalTrialTests(unittest.TestCase):
    def test_composition_uses_physical_profile_without_connecting_and_reuses_local_planner(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);path=root/'ev3.npz'
            weights=empty_weights();weights['out_b'][6]=10
            save_weights(path,weights,robot='ev3',method='unit-test')
            events=[]
            with patch('robot_agent.ev3rstorm_profile.EV3NavigationSSHTransport') as transport:
                adapter=build_adapter(path,'robot@test.local',root,lambda event,**kw:events.append((event,kw)))
                transport.assert_not_called()
            with patch('experiments.navigation_physical_trial.LMStudioControllerActionPlanner') as teacher:
                planner=adapter.navigation_adapter.planner_factory(MODEL)
                packet={'available_actions':['OBSERVE'],'active_waypoint':{'x_mm':800,'y_mm':0}}
                context=SimpleNamespace(completion_allowed=False,active_waypoint=packet['active_waypoint'],to_dict=lambda:packet)
                result=planner.decide(context)
                self.assertEqual(result.decision.action,'OBSERVE')
                teacher.return_value.decide.assert_not_called()
                self.assertEqual(teacher.call_args.kwargs['reasoning_effort'],'high')
                self.assertEqual(teacher.call_args.kwargs['max_output_tokens'],8192)
                expected=ControllerActionPlannerResult(ControllerActionDecision('OBSERVE',1000,'teacher',(),None),1)
                teacher.return_value.decide.return_value=expected
                context.active_waypoint=None;packet.pop('active_waypoint')
                self.assertIs(planner.decide(context),expected)
                teacher.return_value.decide.assert_called_once()
                context.completion_allowed=True
                self.assertEqual(planner.decide(context).decision.action,'COMPLETE')
            self.assertEqual([e[1]['source'] for e in events if e[0]=='policy_dispatch'],['local','astra','goal_contract'])
            self.assertEqual(len([e for e in events if e[0]=='decision_pair']),3)
            self.assertFalse(events[0][1]['automatic_mission_start'])
            adapter.spatial_map_provider.close()

    def test_rejects_another_robots_weights_before_constructing_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);path=root/'wrong.npz'
            save_weights(path,empty_weights(),robot='blast',method='unit-test')
            with patch('experiments.navigation_physical_trial.EV3RSTORMProfile') as profile:
                with self.assertRaisesRegex(ValueError,'EV3 weights'):
                    build_adapter(path,'robot@test.local',root)
                profile.assert_not_called()
