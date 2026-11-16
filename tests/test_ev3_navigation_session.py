import unittest
from unittest.mock import Mock

from robot_agent.ev3_navigation_session import EV3NavigationSession
from robot_agent.lm_studio import LMStudioTimeoutError
from robot_agent.lm_studio_controller_action import ControllerActionDecision, ControllerActionPlannerResult, COMPLETE, FOLLOW_WAYPOINT
from robot_agent.physical_navigation_contract import ADVANCE, OBSERVE, SCAN_FRONT_ARC
from test_ev3_navigation_body import ev3_test_body
from test_ev3_navigation_body import inject_unrecorded_turn


def choice(action, waypoint=None, following=()):
    return ControllerActionPlannerResult(ControllerActionDecision(
        action, 900, "Test execution, not model planning", (action,), None,
        waypoint, following,
    ), 0)


def finish_leg(session):
    """Test driver explicitly requests each bounded step from its fake planner."""
    for _ in range(40):
        result = session.step()
        if result.get("result") not in ("step_completed", "heading_aligned"):
            return result
    raise AssertionError("Leg did not finish within 40 explicit decisions")


class EV3NavigationSessionTests(unittest.TestCase):
    def test_decision_memory_keeps_intent_separate_from_measured_result(self):
        session, planner = self.session(front=False)
        first = choice(ADVANCE)
        second = choice(OBSERVE)
        planner.decide.side_effect = [first, second]
        session.step()
        view = session.context()
        self.assertEqual(view.previous_decision, first.decision.memory())
        self.assertGreater(view.observation["odometry"]["x_mm"], 0)
        self.assertTrue(view.observation["last_motion"]["made_progress"])
        self.assertIsNone(planner.decide.call_args_list[0].args[0].previous_decision)
        session.step()
        self.assertEqual(session.context().previous_decision, second.decision.memory())
        self.assertEqual(view.previous_decision, first.decision.memory())
        fresh_session, _ = self.session(front=False)
        self.assertIsNone(fresh_session.previous_decision)

    def test_ambiguous_non_near_ir_does_not_require_clearance_or_extra_scans(self):
        session, planner = self.session(front=False, ir_response=((0, 39), (2_000, 39)))
        planner.decide.return_value = choice(FOLLOW_WAYPOINT, {"x_mm": 800, "y_mm": 0, "purpose": "goal"})
        result = session.step()
        self.assertEqual(result["result"], "step_completed")
        self.assertEqual(result["motion_attempts"], 1)
        self.assertEqual(planner.decide.call_count, 1)
        self.assertEqual(sum(op == "scan_sample" for op, _ in session.body.transport.requests), 13)
        self.assertEqual(session.body.observation["infrared"]["filtered"], 39)

    def test_model_can_refresh_sensing_without_turning_or_losing_route(self):
        session, planner = self.session(front=False)
        session.body.scan()
        pose, survey = session.body.pose, session.body.survey
        route = ({"x_mm": 800, "y_mm": 0, "purpose": "goal"},)
        session.waypoints = route
        session.body.transport._ir_gate.fail_closed()
        request_count = len(session.body.transport.requests)
        planner.decide.return_value = choice(OBSERVE)
        result = session.step()
        self.assertEqual(result["result"], "observation_refreshed")
        self.assertEqual(result["motion_attempts"], 0)
        self.assertTrue(result["action_completed"])
        self.assertTrue(result["observation_acquired"])
        self.assertIsNone(result["last_motion"])
        feedback = session.context().observation["last_action"]
        self.assertEqual(feedback["action"], OBSERVE)
        self.assertTrue(feedback["completed"])
        remembered = session.context().local_map_evidence["route_execution_memory"]["records"][-1]
        self.assertTrue(remembered["action_completed"])
        self.assertIsNone(remembered["made_progress"])
        self.assertEqual(session.body.pose, pose)
        self.assertIs(session.body.survey, survey)
        self.assertEqual(session.waypoints, route)
        self.assertIn(OBSERVE, planner.decide.call_args.args[0].available_actions)
        self.assertFalse(session.body.observation["infrared"]["blocked"])
        self.assertEqual([op for op, _ in session.body.transport.requests[request_count:]], ["observe", "scan_sample"])

    def test_scan_completion_and_missing_sample_are_independent_receipts(self):
        session, planner = self.session(front=False)
        session.body.scan()
        session.body.transport.missing_sample = session.body.transport.sample_count + 4
        planner.decide.return_value = choice(SCAN_FRONT_ARC)
        result = session.step()
        self.assertTrue(result["action_completed"])
        self.assertFalse(result["observation_acquired"])
        self.assertTrue(result["last_motion"]["completed"])
        self.assertFalse(session.completed)

    def test_interrupted_scan_is_not_reported_as_complete(self):
        session, planner = self.session(ir_response=((0, 0), (200, 44), (300, 51), (1000, 100)))
        session.body.scan()
        session.body.execute(ADVANCE)
        planner.decide.return_value = choice(SCAN_FRONT_ARC)
        result = session.step()
        self.assertFalse(result["action_completed"])
        self.assertFalse(result["last_motion"]["completed"])
        self.assertFalse(session.completed)

    def test_current_pose_and_history_use_the_same_angle_units(self):
        session, planner = self.session(front=False)
        planner.decide.return_value = choice(FOLLOW_WAYPOINT,
            {"x_mm": 0, "y_mm": 300, "purpose": "side"})
        session.step()
        context = session.context()
        prior = context.history[-1]
        self.assertEqual(prior["pose"]["heading_deg"], context.observation["odometry"]["heading_deg"])
        self.assertIn("heading_error_deg", context.active_waypoint_geometry)
        self.assertNotIn("heading_mdeg", prior["pose"])
        self.assertIn("heading_mdeg", session.history[-1]["pose"])
        self.assertNotIn("waypoint_plan", prior)
        self.assertEqual(context.active_waypoint_plan, session.waypoints)
        self.assertIn("waypoint_plan", session.history[-1])

    def test_failed_stationary_sample_keeps_mission_and_can_be_retried(self):
        session, planner = self.session(front=False, missing_sample=14)
        session.body.scan()
        pose = session.body.pose
        route = ({"x_mm": 800, "y_mm": 0, "purpose": "goal"},)
        session.waypoints = route
        planner.decide.return_value = choice(OBSERVE)
        failed = session.step()
        self.assertEqual(failed["result"], "sample_missing")
        self.assertFalse(failed["action_completed"])
        self.assertFalse(failed["observation_acquired"])
        self.assertFalse(session.completed)
        self.assertEqual(session.body.pose, pose)
        self.assertEqual(session.waypoints, route)
        self.assertEqual(session.step()["result"], "observation_refreshed")
        self.assertEqual(session.body.pose, pose)

    def session(self, **kwargs):
        body, profile, mission = ev3_test_body(**kwargs)
        self.addCleanup(body.close)
        planner = Mock()
        session = EV3NavigationSession(body=body, planner=planner, descriptor=profile.descriptor,
            footprint=profile.hazard_calibration.robot_footprint, mission=mission,
            goal="Reach the goal 80 cm ahead of the starting position.")
        return session, planner

    def test_one_model_follow_executes_one_pulse_and_returns_fresh_feedback(self):
        session, planner = self.session(front=False)
        planner.decide.return_value = choice(FOLLOW_WAYPOINT, {"x_mm": 800, "y_mm": 0, "purpose": "goal"})
        result = session.step()
        self.assertEqual(result["motion_attempts"], 1)
        self.assertEqual(planner.decide.call_count, 1)
        self.assertEqual(result["result"], "step_completed")
        self.assertTrue(result["action_completed"])
        self.assertFalse(session.context().completion_allowed)
        self.assertGreater(session.mission.distance_to_target_mm(session.body.pose), 700)
        finish_leg(session)
        self.assertLessEqual(session.mission.distance_to_target_mm(session.body.pose), 50)
        planner.decide.return_value = choice(COMPLETE)
        session.step()
        self.assertTrue(session.completed)

    def test_follow_corrects_small_measured_heading_offset_without_new_waypoints(self):
        session, planner = self.session(front=False)
        session.body.scan()
        inject_unrecorded_turn(session.body.transport, motor_degrees=60)
        session.body.observe()
        self.assertGreater(abs(session.body.pose.heading_mdeg), 5_000)
        self.assertLess(abs(session.body.pose.heading_mdeg), 12_000)
        target = {"x_mm": 600, "y_mm": 0, "purpose": "Unchanged intermediate endpoint"}
        planner.decide.return_value = choice(FOLLOW_WAYPOINT, target)
        result = finish_leg(session)
        self.assertEqual(result["result"], "waypoint_reached")
        self.assertGreater(planner.decide.call_count, 1)
        self.assertEqual(session.waypoints, (target,))
        self.assertLessEqual(abs(session.body.pose.x_mm - 600), session.body.waypoint_radius_mm)
        self.assertLessEqual(abs(session.body.pose.y_mm), session.body.waypoint_radius_mm)

    def test_follow_stops_on_mission_arrival_before_extra_waypoint_pulse(self):
        session, planner = self.session(front=False)
        session.mission.minimum_forward_progress_mm = 815
        planner.decide.return_value = choice(FOLLOW_WAYPOINT,
            {"x_mm":815,"y_mm":0,"purpose":"Goal"})
        result = finish_leg(session)
        distance = session.mission.distance_to_target_mm(session.body.pose)
        self.assertEqual(result["result"], "goal_reached")
        self.assertGreater(distance, session.body.waypoint_radius_mm)
        self.assertLessEqual(distance, 50)
        self.assertTrue(session.context().completion_allowed)
        self.assertEqual(planner.decide.call_count, 15)

    def test_recovery_advance_uses_current_heading_and_retains_route(self):
        session, planner = self.session(front=False)
        session.body.scan()
        session.body.execute("TURN_LEFT_90")
        route = ({"x_mm": 800, "y_mm": 0, "purpose": "retained goal"},)
        session.waypoints = route
        planner.decide.return_value = choice(ADVANCE)
        result = session.step()
        self.assertEqual(result["motion_attempts"], 1)
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
        planner.decide.return_value = choice(FOLLOW_WAYPOINT, first, tail)
        self.assertEqual(session.step()["result"], "heading_aligned")
        planner.decide.return_value = choice(FOLLOW_WAYPOINT)
        self.assertEqual(finish_leg(session)["result"], "waypoint_reached")
        self.assertGreater(session.body.pose.y_mm, 300)
        planner.decide.return_value = choice(SCAN_FRONT_ARC)
        session.step()
        planner.decide.return_value = choice(FOLLOW_WAYPOINT)
        self.assertEqual(session.waypoints, tail)
        self.assertEqual(planner.decide.call_args.args[0].active_waypoint_plan, tail)
        self.assertEqual(session.mission.target_point(), (800, 0))
        # 650 lies between fixed EV3 pulse endpoints. It must finish this leg,
        # not shuttle endlessly between the two nearest stops.
        self.assertEqual(session.step()["result"], "heading_aligned")
        result = finish_leg(session)
        self.assertEqual(result["result"], "waypoint_reached")
        self.assertLessEqual(abs(session.body.pose.x_mm - 650), session.body.waypoint_radius_mm)

    def test_inspection_turn_and_scan_preserve_route_and_historical_viewpoints(self):
        session, planner = self.session(front=False)
        session.body.scan()
        session.body.execute("TURN_RIGHT_90")
        route = ({"x_mm": 0, "y_mm": -350, "purpose": "Retained route"},)
        session.waypoints = route
        before = session.context().local_map_evidence["ir_survey"]["samples"]
        planner.decide.side_effect = [choice("TURN_LEFT_90"), choice(SCAN_FRONT_ARC), choice(ADVANCE)]
        turn = session.step()
        self.assertGreater(abs(turn["last_motion"]["measured_turn_deg"]), 80)
        session.step()
        self.assertEqual(session.waypoints, route)
        session.step()
        after = session.context().local_map_evidence["ir_survey"]["samples"]
        self.assertTrue(all(sample in after for sample in before))
        self.assertEqual(session.waypoints, route)

    def test_worker_denial_is_measured_no_motion_and_does_not_erase_route(self):
        # A denied attempt is not proof of a physical motor pulse.
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

    def test_follow_checks_arrival_after_refreshing_encoder_position(self):
        session, _ = self.session(front=False)
        session.body.transport.request("pulse", {"action": ADVANCE}, 30)
        target = {"x_mm": 52, "y_mm": 0, "purpose": "checkpoint"}
        reason, attempts = session._follow(target)
        self.assertEqual(reason, "waypoint_reached")
        self.assertEqual(attempts, 0)
        self.assertEqual(session.body.pose.x_mm, 52)

    def test_follow_does_not_accept_stale_arrival_before_reading_encoders(self):
        session, _ = self.session(front=False)
        session.body.execute(ADVANCE)
        session.body.transport.request("pulse", {"action": "REVERSE"}, 30)
        target = {"x_mm": 52, "y_mm": 0, "purpose": "checkpoint"}
        reason, attempts = session._follow(target)
        self.assertEqual(reason, "waypoint_reached")
        self.assertEqual(attempts, 1)
        self.assertEqual(session.body.transport.simulation.pose("ev3").x_mm, 52)

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

    def test_follow_goal_does_not_add_an_unrequested_arrival_turn(self):
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
        self.assertIsNone(result["last_motion"])
        self.assertEqual((session.body.pose.x_mm, session.body.pose.y_mm), (before.x_mm, before.y_mm))
        self.assertEqual(session.body.pose, before)
        self.assertTrue(session.context().completion_allowed)
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
        self.assertEqual([r[0] for r in session.body.transport.requests[request_count:]], ["observe"])
        self.assertEqual(session.step()["result"], "step_completed")

    def test_planning_retains_a_healthy_worker_and_its_gate(self):
        # Ordinary planning does not close a healthy worker or reset its gate.
        session, planner = self.session(front=False)
        session.body.scan()
        transport = session.body.transport
        request_count = len(transport.requests)
        planner.decide.return_value = choice(FOLLOW_WAYPOINT,
            {"x_mm": 100, "y_mm": 0, "purpose": "observation"})
        session.step()
        self.assertIs(session.body.transport, transport)
        self.assertNotIn("shutdown", [r[0] for r in transport.requests[request_count:]])

    def test_unverified_ir_is_recoverable_and_not_reported_as_an_obstacle(self):
        session, planner = self.session(front=False)
        session.body.scan()
        session.body.transport._ir_gate.fail_closed()
        target = {"x_mm": 100, "y_mm": 0, "purpose": "observation"}
        planner.decide.return_value = choice(FOLLOW_WAYPOINT, target)
        result = session.step()
        self.assertEqual(result["result"], "infrared_unverified")
        self.assertEqual(result["motion_attempts"], 0)
        self.assertEqual(session.waypoints, (target,))
        self.assertFalse(session.completed)
        context = planner.decide.call_args.args[0]
        self.assertFalse(context.execution["forward_admission"]["permitted"])
        self.assertEqual(context.execution["forward_admission"]["reason"], "warming_up")
        # More stationary samples settle the same gate; no forced rescan/replan.
        for _ in range(5):
            session.body.observe()
        self.assertEqual(finish_leg(session)["result"], "waypoint_reached")

    def test_replay_qwen_wider_detour_keeps_partial_alignment_receipt_and_route(self):
        # Replay the three route decisions from the IR-only box run, with
        # explicit continuation after the newly exposed alignment boundary.
        # This isolates execution/receipt handling; it is not another LLM trial.
        session, planner = self.session(side="right")
        first = {"x_mm": 0, "y_mm": 150, "purpose": "side"}
        second = {"x_mm": 800, "y_mm": 150, "purpose": "pass"}
        wider = {"x_mm": 72, "y_mm": 300, "purpose": "more side clearance"}
        planner.decide.return_value = choice(FOLLOW_WAYPOINT, first, (second,))
        self.assertEqual(session.step()["result"], "heading_aligned")
        planner.decide.return_value = choice(FOLLOW_WAYPOINT)
        self.assertEqual(finish_leg(session)["result"], "waypoint_reached")
        self.assertEqual(session.step()["result"], "heading_aligned")
        drive = finish_leg(session)
        self.assertEqual(drive["result"], "simulation_body_blocked")
        self.assertEqual(drive["last_motion"]["kind"], "translation")
        before = session.body.pose
        planner.decide.return_value = choice(FOLLOW_WAYPOINT, wider)
        result = session.step()
        self.assertEqual(result["result"], "simulation_body_blocked")
        self.assertEqual(result["last_motion"]["kind"], "turn")
        self.assertEqual(result["last_motion"]["requested_turn_deg"], 90)
        self.assertLess(result["last_motion"]["measured_turn_deg"], 5)
        self.assertEqual(result["last_motion"]["measured_translation_mm"], 0)
        self.assertFalse(result["last_motion"]["completed"])
        self.assertEqual(session.context().execution["next_waypoint_motion"]["action"], "TURN_LEFT_90")
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
