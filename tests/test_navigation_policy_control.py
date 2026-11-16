"""Causal weights -> planner -> real adapters -> simulated movement checks.

Constant networks are test probes, not navigation candidates or success claims.
No model service, training or physical transport is used.
"""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from experiments.local_navigation import ACTIONS, LocalPlanner, LocalPolicy
from experiments.navigation_set_author import expand
from experiments.navigation_set_policy import SetPolicy, observations
from robot_agent.blast_navigation_simulation import run_blast_gemma_scenario
from robot_agent.lm_studio_controller_action import ControllerActionDecision, ControllerActionPlannerResult
from scripts.local_navigation_experiment import run_ev3, scenario


def policy_for(action):
    spec = {key:[] for key in ('ray','memory','head','output')}
    spec['output'] = [{'index':ACTIONS.index(action),'bias':100,'inputs':[]}]
    return SetPolicy(expand(spec))


def response(action, waypoint=None):
    return ControllerActionPlannerResult(ControllerActionDecision(
        action=action, confidence_milli=1000, assessment='Diagnostic only',
        plan=(), utterance=None, waypoint=waypoint), 0)


class NavigationPolicyControlTests(unittest.TestCase):
    def test_availability_filter_is_visible_and_does_not_mutate_raw_scores(self):
        policy = policy_for('ADVANCE')
        packet = {'available_actions':['ADVANCE','REVERSE']}
        scores = policy.scores(packet)
        self.assertEqual(policy.predict(packet)[0], 'ADVANCE')
        packet['available_actions'] = ['REVERSE']
        self.assertEqual(policy.scores(packet), scores)
        self.assertEqual(policy.predict(packet)[0], 'REVERSE')

    def test_loading_distinct_set_weights_changes_choices_without_dispatch_override(self):
        packet = {'available_actions':list(ACTIONS[:-1]),
                  'active_waypoint':{'x_mm':600,'y_mm':0,'purpose':'test'}}
        context = SimpleNamespace(to_dict=lambda:packet, completion_allowed=False,
                                  active_waypoint=packet['active_waypoint'])
        with tempfile.TemporaryDirectory() as folder:
            for action in ACTIONS:
                with self.subTest(action=action):
                    path = Path(folder)/(action+'.json')
                    path.write_text(json.dumps(policy_for(action).artifact))
                    loaded = LocalPolicy.load(path)
                    self.assertIsInstance(loaded, SetPolicy)
                    teacher = Mock()
                    teacher.decide.return_value = response('OBSERVE')
                    dispatches = []
                    result = LocalPlanner(teacher, loaded, dispatches.append).decide(context)
                    self.assertEqual(dispatches[-1]['selection'], action)
                    if action == 'REPLAN':
                        teacher.decide.assert_called_once_with(context)
                        self.assertIs(result, teacher.decide.return_value)
                    else:
                        teacher.decide.assert_not_called()
                        self.assertEqual(result.decision.action, action)

    def test_host_contracts_are_explicit_not_hidden_model_votes(self):
        for complete, waypoint, expected in ((False,None,'REPLAN'),
                                             (True,{'x_mm':0,'y_mm':0},'COMPLETE')):
            with self.subTest(expected=expected):
                policy, teacher = Mock(), Mock()
                context = SimpleNamespace(to_dict=lambda:{}, completion_allowed=complete,
                                          active_waypoint=waypoint)
                events = []
                LocalPlanner(teacher, policy, events.append).decide(context)
                policy.predict.assert_not_called()
                self.assertEqual(events[-1]['selection'], expected)

    def test_only_output_bias_changes_actual_motion_for_both_robot_adapters(self):
        targets = ('TURN_LEFT_90','TURN_RIGHT_90','ADVANCE','REVERSE','FOLLOW_WAYPOINT','OBSERVE')
        for robot in ('ev3','blast'):
            previous_input = None
            for action in targets:
                with self.subTest(robot=robot, action=action):
                    teacher = Mock()
                    teacher.decide.return_value = response('SCAN_FRONT_ARC',
                        {'x_mm':600,'y_mm':0,'purpose':'Diagnostic route'})
                    dispatches, inputs, outputs = [], [], []
                    local = LocalPlanner(teacher, policy_for(action), dispatches.append)
                    class RecordingPlanner:
                        def decide(self, context):
                            inputs.append(observations(context.to_dict()))
                            result = local.decide(context)
                            outputs.append(result.decision.action)
                            return result
                    scene = scenario(robot, 'train-open')
                    if robot == 'ev3':
                        result = run_ev3(scene, RecordingPlanner(), 2, lambda _:None)
                    else:
                        result = run_blast_gemma_scenario(scene, planner_factory=lambda _:RecordingPlanner(),
                            max_decisions=2, minimum_forward_progress_mm=scene.goals[0].x_mm,
                            goal='Reach the goal straight ahead of your starting position.')
                    self.assertIsNone(result.get('error_type'), result.get('error_message'))
                    teacher.decide.assert_called_once()
                    self.assertEqual(outputs, ['SCAN_FRONT_ARC',action])
                    self.assertEqual(dispatches[-1]['selection'], action)
                    if previous_input is not None:
                        self.assertEqual(inputs[-1], previous_input)
                    previous_input = inputs[-1]
                    pose = result['final_pose']
                    if action.startswith('TURN_'):
                        sign = 1 if action == 'TURN_LEFT_90' else -1
                        self.assertAlmostEqual(pose['heading_mdeg'], sign*90_000, delta=3_000)
                        self.assertEqual(pose['x_mm'], 0)
                    elif action == 'FOLLOW_WAYPOINT':
                        self.assertAlmostEqual(pose['x_mm'],52 if robot=='ev3' else 45,delta=2)
                    elif action == 'OBSERVE':
                        self.assertEqual((pose['x_mm'],pose['y_mm']), (0,0))
                        self.assertAlmostEqual(pose['heading_mdeg'],0,delta=1_000)
                        if robot == 'blast':
                            self.assertEqual(result['commands_tail'],
                                ['scan_surroundings','scan_front_arc','observe_settled'])
                            self.assertEqual(result['scans'],2)
                    else:
                        pulse = 52 if robot == 'ev3' else 45
                        self.assertEqual(pose['x_mm'], pulse if action == 'ADVANCE' else -pulse)


if __name__ == '__main__':
    unittest.main()
