"""Simulator-only data collection from student states; not a runtime fallback.

One irreversible handoff to the teacher in the SAME episode. Native failed
outcomes, repeated stationary actions, or repeated movement away from the same
waypoint select data. No poses, maps, memories, commands or sensors are edited.
Only the teacher suffix may become labels, and only if it reaches the goal
without any additional simulator collision stops.
"""
import math
import time
from dataclasses import asdict
from unittest.mock import patch

from experiments.local_navigation import LocalPlanner
from experiments.navigation_recovery_data import feedback_kinds
from robot_agent.multi_robot_navigation_simulator import MultiRobotNavigationSimulator

PROTOCOL = 'student-state-astra-recovery-v1'


class StudentRecovery:
    def __init__(self, teacher, policy, emit):
        self.teacher, self.emit = teacher, emit
        self.student = LocalPlanner(teacher, policy, self.dispatch)
        self.last_source = self.last_action = None
        self.handoff = None
        self.blocked = 0
        self.visits = {}
        self.last_target = self.last_distance = None
        self.receding = 0
        self.decisions = 0
        self.teacher_labels = 0

    def dispatch(self, event):
        self.last_source, self.last_action = event['source'], event['selection']
        self.emit(event)

    def __enter__(self):
        original = MultiRobotNavigationSimulator._record
        def record(world, robot_id, kind, value=0):
            if kind == 'blocked': self.blocked += 1
            return original(world, robot_id, kind, value)
        self.record_patch = patch.object(MultiRobotNavigationSimulator, '_record', record)
        self.record_patch.start()
        return self

    def __exit__(self, *args):
        self.record_patch.stop()

    def failure(self, packet):
        if self.last_source != 'local':
            self.visits.clear()
            self.receding = 0
            self.last_target = self.last_distance = None
            return None
        observation = packet.get('observation') or {}
        motion = observation.get('last_motion') or {}
        action = observation.get('last_action') or {}
        recent = (packet.get('history') or [{}])[-1]
        # Incomplete target alignment with verified progress is not a failure.
        # Nor is an older failed motion after a successful current observation.
        failed_motion = (action.get('completed') is not True and
            motion.get('completed') is False and motion.get('made_progress') is not True)
        failed_action = action.get('completed') is False and bool(action.get('reason'))
        native_failure = (failed_motion or failed_action
            or recent.get('route_rejection') or recent.get('result') in
            ('near_obstacle','infrared_blocked','no_measured_progress'))
        kinds = feedback_kinds(packet) & {'motion_interrupted','partial_scan','route_rejected'} if native_failure else set()
        if kinds: return '+'.join(sorted(kinds))
        pose = packet['local_map_evidence']['robot_pose']
        target = packet.get('active_waypoint')
        key = (self.last_action, round(pose['x_mm']/20), round(pose['y_mm']/20),
               round(pose.get('heading_deg',0)/10))
        self.visits[key] = self.visits.get(key,0)+1
        if self.visits[key] >= 3: return 'repeated_stationary_action'
        if target:
            xy = (target['x_mm'],target['y_mm'])
            distance = math.hypot(xy[0]-pose['x_mm'],xy[1]-pose['y_mm'])
            if xy != self.last_target:
                self.receding = 0
            elif self.last_distance is not None and abs(distance-self.last_distance)>10:
                self.receding = self.receding+1 if distance>self.last_distance else 0
            self.last_target,self.last_distance = xy,distance
            if self.receding >= 3: return 'repeated_movement_away_from_waypoint'
        return None

    def decide(self, context):
        packet = context.to_dict()
        reason = None if context.completion_allowed or self.handoff else self.failure(packet)
        if reason:
            self.handoff = {'decision_index':self.decisions,'reason':reason,
                            'prefix_collision_stops':self.blocked}
            self.emit({'event':'student_handoff', **self.handoff, 'packet':packet})
        self.decisions += 1
        if not self.handoff:
            return self.student.decide(context)
        began = time.perf_counter()
        result = self.teacher.decide(context)
        self.teacher_labels += 1
        self.emit({'event':'teacher_label','packet':packet,'decision':asdict(result.decision),
                   'feedback':sorted(feedback_kinds(packet)), 'phase':'recovery'})
        self.emit({'event':'policy_dispatch','source':'astra','selection':result.decision.action,
                   'elapsed_ms':(time.perf_counter()-began)*1000})
        return result

    def summary(self, result):
        suffix_stops = self.blocked-self.handoff['prefix_collision_stops'] if self.handoff else None
        success = bool(self.handoff and result.get('completed') and result.get('goal_reached')
                       and suffix_stops == 0 and not result.get('error_type'))
        return {'collection_protocol':PROTOCOL,'handoff':self.handoff,
                'teacher_suffix_labels':self.teacher_labels,'recovery_collision_stops':suffix_stops,
                'recovery_complete':success,'recovery_success':success,'fault':'student'}
