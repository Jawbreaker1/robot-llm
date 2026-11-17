"""Opt-in EV3 trial of the SAME learned planner used in simulation.

Only composition and logging live here. The ordinary dashboard, hardware
adapter, motor controller, map, stop controls and LocalPlanner are unchanged.
Starting the console never starts a mission or moves a motor.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import signal

import numpy as np

from experiments.local_navigation import LocalPlanner, LocalPolicy
from robot_agent import dashboard_cli
from robot_agent.ev3rstorm_profile import EV3RSTORMProfile, EV3SSHBinding
from robot_agent.lm_studio_controller_action import LMStudioControllerActionPlanner
from robot_agent.navigation_diagnostics import enable_navigation_diagnostics, record_navigation_diagnostic
from robot_agent.social_expression import EV3_GESTURES
from robot_agent.blast_personality import EV3_PERSONA_BY_LOCALE

MODEL = 'codex/gpt-6-astra'


def build_adapter(weights, target, output, emit=record_navigation_diagnostic):
    with np.load(weights, allow_pickle=False) as archive:
        metadata = json.loads(str(archive['metadata']))
    if metadata.get('robot') != 'ev3':
        raise ValueError('EV3 physical trial requires EV3 weights')
    policy = LocalPolicy.load(weights)

    def planner_factory(model):
        if model != MODEL:
            raise ValueError('This validation trial requires Astra high')
        teacher = LMStudioControllerActionPlanner(model=model, reasoning_effort='high',
            max_output_tokens=8192, timeout_seconds=60, social_expressions=True,
            social_gestures=EV3_GESTURES, social_faces=('neutral',),
            utterance_persona_by_locale=EV3_PERSONA_BY_LOCALE)
        selected = LocalPlanner(teacher, policy,
            lambda event: emit(event['event'], **{k:v for k,v in event.items() if k!='event'}))

        class RecordingPlanner:
            def comment(self, packet):
                return selected.comment(packet)

            def decide(self, context):
                result = selected.decide(context)
                emit('decision_pair', packet=context.to_dict(), decision=asdict(result.decision))
                return result
        return RecordingPlanner()

    profile = EV3RSTORMProfile()
    adapter = profile.build_adapter(EV3SSHBinding(profile_id=profile.descriptor.profile_id,
        target=target, memory_path=output/'memory.json'), planner_factory=planner_factory)
    adapter.robot_control_target = dashboard_cli._configured_robot_control_target(profile.descriptor.profile_id)
    emit('physical_local_policy_configured', robot='ev3', model=MODEL, reasoning_effort='high',
         weights=str(weights.resolve()), weight_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
         metadata=metadata, footprint=asdict(profile.hazard_calibration.robot_footprint),
         automatic_mission_start=False)
    return adapter


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights',type=Path,required=True)
    parser.add_argument('--target',default='robot@ev3dev.local')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--console-access-key-file',default=str(Path.home()/'.robot-llm/dashboard-access-key'))
    args=parser.parse_args(argv)
    args.output.mkdir(parents=True,exist_ok=False)
    enable_navigation_diagnostics(args.output/'diagnostics.jsonl')
    adapter=build_adapter(args.weights,args.target,args.output)
    previous=signal.signal(signal.SIGTERM,dashboard_cli._raise_termination_interrupt)
    try:
        return dashboard_cli._run(['--model',MODEL,'--port',str(args.port),
            '--console-access-key-file',args.console_access_key_file],robot_runtime_adapter=adapter)
    finally:
        signal.signal(signal.SIGTERM,previous)


if __name__=='__main__':
    raise SystemExit(main())
