import unittest
from unittest.mock import Mock

from robot_agent.ev3_navigation_session import EV3NavigationSession
from robot_agent.lm_studio import LMStudioTimeoutError
from robot_agent.lm_studio_controller_action import ControllerActionDecision, ControllerActionPlannerResult, COMPLETE, FOLLOW_WAYPOINT
from robot_agent.physical_navigation_contract import ADVANCE, SCAN_FRONT_ARC
from test_ev3_navigation_body import ev3_test_body


def choice(action, waypoint=None, following=()):
    return ControllerActionPlannerResult(ControllerActionDecision(
        action, 900, "Test execution, not model planning", (action,), None,
        waypoint, following,
    ), 0)


class EV3NavigationSessionTests(unittest.TestCase):
    def session(self, **kwargs):
        body, profile, mission = ev3_test_body(**kwargs)
        self.addCleanup(body.close)
        planner = Mock()
        session = EV3NavigationSession(body=body, planner=planner, descriptor=profile.descriptor,
            footprint=profile.hazard_calibration.robot_footprint, mission=mission,
            goal="Reach the goal 80 cm ahead of the starting position.")
        return session, planner

    def test_one_model_follow_finishes_clear_leg_with_feedback_between_pulses(self):
        session, planner = self.session(front=False)
        planner.decide.side_effect = [choice(FOLLOW_WAYPOINT, {"x_mm": 800, "y_mm": 0, "purpose": "goal"}), choice(COMPLETE)]
        result = session.step()
        self.assertGreater(result["motion_pulses"], 5)
        self.assertEqual(planner.decide.call_count, 1)
        self.assertLessEqual(session.mission.distance_to_target_mm(session.body.pose), 50)
        session.step()
        self.assertTrue(session.completed)

    def test_recovery_advance_uses_current_heading_and_retains_route(self):
        session, planner = self.session(front=False)
        session.body.scan()
        session.body.execute("TURN_LEFT_90")
        route = ({"x_mm": 800, "y_mm": 0, "purpose": "retained goal"},)
        session.waypoints = route
        planner.decide.return_value = choice(ADVANCE)
        result = session.step()
        self.assertEqual(result["motion_pulses"], 1)
        self.assertEqual(result["last_motion"]["kind"], "translation")
        self.assertGreater(session.body.pose.y_mm, 40)
        self.assertLess(abs(session.body.pose.x_mm), 10)
        self.assertEqual(session.waypoints, route)
        self.assertFalse(session.completed)

    def test_route_tail_survives_scan_and_arrival_without_host_replacement(self):
        session, planner = self.session(side="right", missing_sample=4)
        first = {"x_mm": 0, "y_mm": 350, "purpose": "side"}
        tail = ({"x_mm": 650, "y_mm": 350, "purpose": "pass box"},
                {"x_mm": 650, "y_mm": 0, "purpose": "return"})
        planner.decide.side_effect = [choice(FOLLOW_WAYPOINT, first, tail), choice(SCAN_FRONT_ARC), choice(FOLLOW_WAYPOINT)]
        session.step()
        self.assertGreater(session.body.pose.y_mm, 300)
        session.step()
        self.assertEqual(session.waypoints, tail)
        self.assertEqual(planner.decide.call_args.args[0].active_waypoint_plan, tail)
        self.assertEqual(session.mission.target_point(), (800, 0))
        # 650 lies between fixed EV3 pulse endpoints. It must finish this leg,
        # not shuttle endlessly between the two nearest stops.
        result = session.step()
        self.assertEqual(result["result"], "waypoint_reached")
        self.assertLessEqual(abs(session.body.pose.x_mm - 650), session.body.waypoint_radius_mm)

    def test_worker_denial_is_measured_no_motion_and_does_not_erase_route(self):
        session, planner = self.session()
        pose = session.body.pose
        result = session.body.execute(ADVANCE)
        self.assertFalse(result["completed"])
        self.assertEqual(result["reason"], "infrared_blocked")
        self.assertEqual((session.body.pose.x_mm, session.body.pose.y_mm), (pose.x_mm, pose.y_mm))
        target = {"x_mm": 800, "y_mm": 0, "purpose": "goal"}
        planner.decide.return_value = choice(FOLLOW_WAYPOINT, target)
        result = session.step()
        self.assertEqual(result["result"], "near_obstacle")
        self.assertEqual(session.waypoints, (target,))
        self.assertFalse(session.completed)
        self.assertIsNone(result["last_motion"])

    def test_ir_change_during_a_pulse_retains_verified_partial_motion(self):
        session, _ = self.session()
        session.body.transport.ir_response = ((0, 20), (170, 20), (180, 60), (2_000, 60))
        for _ in range(5):
            session.body.observe()
        result = session.body.execute(ADVANCE)
        self.assertFalse(result["completed"])
        self.assertEqual(result["reason"], "infrared_blocked")
        self.assertGreater(session.body.pose.x_mm, 0)
        self.assertLessEqual(session.body.pose.x_mm, 50)

    def test_follow_goal_finishes_heading_in_place_without_requesting_another_plan(self):
        session, planner = self.session(front=False)
        session.body.scan()
        for _ in range(15):
            session.body.execute(ADVANCE)
        session.body.execute("TURN_LEFT_90")
        before = session.body.pose
        self.assertLessEqual(session.mission.distance_to_target_mm(before), 50)
        self.assertFalse(session.mission.heading_aligned(before))
        planner.decide.side_effect = [choice(FOLLOW_WAYPOINT, {"x_mm": 800, "y_mm": 0, "purpose": "goal"}),
                                      choice(COMPLETE)]
        result = session.step()
        self.assertEqual(result["result"], "waypoint_reached")
        self.assertEqual(result["last_motion"]["kind"], "turn")
        self.assertEqual((session.body.pose.x_mm, session.body.pose.y_mm), (before.x_mm, before.y_mm))
        self.assertTrue(session.mission.heading_aligned(session.body.pose))
        self.assertEqual(planner.decide.call_count, 1)
        session.step()
        self.assertTrue(session.completed)

    def test_model_timeout_keeps_route_and_retries_without_scan_or_motion(self):
        session, planner = self.session(front=False)
        session.body.scan()
        route = ({"x_mm": 400, "y_mm": 0, "purpose": "first leg"},
                 {"x_mm": 800, "y_mm": 0, "purpose": "goal"})
        session.waypoints = route
        pose = session.body.pose
        request_count = len(session.body.transport.requests)
        planner.decide.side_effect = [LMStudioTimeoutError("test timeout"), choice(FOLLOW_WAYPOINT)]
        self.assertEqual(session.step()["event"], "planner_retry")
        self.assertEqual(session.waypoints, route)
        self.assertEqual(session.body.pose, pose)
        self.assertEqual([r[0] for r in session.body.transport.requests[request_count:]], ["observe", "shutdown"])
        self.assertEqual(session.step()["result"], "waypoint_reached")

    def test_replay_qwen_wider_detour_keeps_partial_alignment_receipt_and_route(self):
        # Replay the actual three model decisions from the IR-only box run.
        # This isolates execution/receipt handling; it is not another LLM trial.
        session, planner = self.session(side="right")
        first = {"x_mm": 0, "y_mm": 150, "purpose": "side"}
        second = {"x_mm": 800, "y_mm": 150, "purpose": "pass"}
        wider = {"x_mm": 72, "y_mm": 300, "purpose": "more side clearance"}
        planner.decide.side_effect = [choice(FOLLOW_WAYPOINT, first, (second,)),
                                      choice(FOLLOW_WAYPOINT), choice(FOLLOW_WAYPOINT, wider)]
        self.assertEqual(session.step()["result"], "waypoint_reached")
        drive = session.step()
        self.assertEqual(drive["result"], "simulation_body_blocked")
        self.assertEqual(drive["last_motion"]["kind"], "translation")
        before = session.body.pose
        result = session.step()
        self.assertEqual(result["result"], "simulation_body_blocked")
        self.assertEqual(result["last_motion"]["kind"], "turn")
        self.assertEqual(result["last_motion"]["requested_turn_deg"], 90)
        self.assertLess(result["last_motion"]["measured_turn_deg"], 5)
        self.assertEqual(result["last_motion"]["measured_translation_mm"], 0)
        self.assertFalse(result["last_motion"]["completed"])
        self.assertEqual(session.context().history[-1]["last_motion"], result["last_motion"])
        self.assertEqual(session.waypoints, (wider,))
        self.assertFalse(session.completed)
        # With polling-scale contact simulation, this turn can be stopped
        # immediately; zero measured rotation is a valid partial receipt.
        self.assertGreaterEqual(session.body.pose.heading_mdeg, before.heading_mdeg)
        truth = session.body.transport.simulation.pose("ev3")
        self.assertLessEqual(abs(session.body.pose.heading_mdeg - truth.heading_mdeg), 132)
        self.assertEqual((session.body.pose.x_mm, session.body.pose.y_mm), (truth.x_mm, truth.y_mm))


if __name__ == "__main__":
    unittest.main()
