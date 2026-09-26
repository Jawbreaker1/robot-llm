import threading
from dataclasses import replace
import unittest
from unittest.mock import Mock

from robot_agent.ev3_navigation_body import EV3NavigationBody
from robot_agent.ev3_navigation_context import ev3_navigation_context
from robot_agent.ev3_navigation_transport import EV3NavigationRemoteError, EV3NavigationSSHTransport, EV3NavigationTimeoutError, EV3NavigationTransportError
from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
from robot_agent.multi_robot_navigation_simulator import MultiRobotNavigationSimulator, RectangleObstacle, SimulatedRobot, SimulationGoal
from robot_agent.navigation_state import PoseEstimate
from robot_agent.physical_odometry import normalize_heading_mdeg
from robot_agent.physical_navigation_contract import ADVANCE, SCAN_TURN_OPERATION
from robot_agent.physical_navigation_mission import DirectionalMission

from robot_agent.simulation_robot_adapters import SharedWorldEV3Transport


class ValidatedEV3SimulationTransport(SharedWorldEV3Transport):
    """Check simulator output with the real SSH worker receipt validator."""
    def __init__(self, *args, missing_sample=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.requests = []
        self.sample_count = 0
        self.missing_sample = missing_sample
        self.validator = EV3NavigationSSHTransport(target="unused.local", controller_id="ev3",
                                                    remote_worker_path="/unused.py")

    def request(self, operation, arguments, timeout, cancel_requested=None):
        self.requests.append((operation, dict(arguments)))
        response = super().request(operation, arguments, timeout, cancel_requested)
        self.validator._validate_success_result(operation, arguments, response)
        if operation == "scan_sample":
            self.sample_count += 1
            if self.sample_count == self.missing_sample:
                raise EV3NavigationRemoteError("sample_missing", "Missing sample", False,
                                               observation=response["result"]["observation"],
                                               stop=response["result"]["stop"])
        return response


# Deliberately strong-return fixture for execution tests, not navigation evidence.
EXECUTION_IR_RESPONSE = ((0, 20), (210, 20), (220, 60), (2_000, 60))


def ev3_test_body(*, side=None, front=True, missing_sample=None, cancel_requested=lambda: False,
                  ir_response=EXECUTION_IR_RESPONSE):
    profile = EV3RSTORMProfile()
    obstacles = [RectangleObstacle("front", 200, -100, 400, 100)] if front else []
    if side is not None:
        obstacles.append(RectangleObstacle("side", -100, -400 if side == "right" else 200,
                                            100, -200 if side == "right" else 400))
    world = MultiRobotNavigationSimulator(
        bounds=(-1_000, -1_000, 2_000, 1_000), obstacles=tuple(obstacles),
        robots=(SimulatedRobot("ev3", PoseEstimate(0, 0, 0), profile.hazard_calibration.robot_footprint, 1_000),),
        goals=(SimulationGoal("ev3", 800, 0),),
    )
    requests = []
    def transport_factory():
        transport = ValidatedEV3SimulationTransport(world, world_robot_id="ev3",
                                                   odometry=profile.odometry_calibration,
                                                   ir_response=ir_response,
                                                   missing_sample=missing_sample)
        transport.requests = requests
        return transport
    body = EV3NavigationBody(transport_factory, calibration=profile.odometry_calibration,
                             cancel_requested=cancel_requested)
    body.start()
    mission = DirectionalMission.begin(episode_id="ev3-test", minimum_forward_progress_mm=800,
                                       pose=body.pose, heading_tolerance_mdeg=20_000)
    return body, profile, mission


def inject_unrecorded_turn(transport, motor_degrees=76):
    """Move the simulated robot without producing a motion receipt."""
    transport._left -= motor_degrees
    transport._right += motor_degrees
    turn_mdeg = motor_degrees * transport.odometry.turn_mdeg_per_opposed_encoder_degree
    transport.simulation.rotate(transport.world_robot_id, turn_mdeg)
    return turn_mdeg


class EV3NavigationBodyTests(unittest.TestCase):
    def test_startup_timeout_retries_without_losing_scan_pose_or_encoder_anchor(self):
        body, _, _ = ev3_test_body(front=False)
        self.addCleanup(body.close)
        body.scan()
        body.execute(ADVANCE)
        body.release_for_planning()
        pose, survey = body.pose, body.survey
        factory = body.transport_factory
        failed = Mock()
        failed.request.side_effect = EV3NavigationTimeoutError("describe", 60, "")
        body.transport_factory = Mock(side_effect=[failed, factory()])
        events = []
        body.publish = events.append
        body.observe()
        failed.request.assert_called_once_with("describe", {}, 60.0, cancel_requested=body.cancel_requested)
        failed.abort.assert_called_once()
        failed.close.assert_called_once()
        self.assertEqual(body.transport_factory.call_count, 2)
        self.assertEqual(body.pose, pose)
        self.assertIs(body.survey, survey)
        self.assertEqual(events[0]["event"], "worker_start_retry")

    def test_startup_retry_is_bounded_and_does_not_hide_protocol_or_cancel_errors(self):
        for kind in ("timeout", "protocol", "cancelled"):
            with self.subTest(kind=kind):
                body, _, _ = ev3_test_body(front=False)
                body.release_for_planning()
                failed = Mock()
                error = (EV3NavigationTransportError("invalid description") if kind == "protocol"
                         else EV3NavigationTimeoutError("describe", 60, ""))
                failed.request.side_effect = error
                body.transport_factory = Mock(return_value=failed)
                body.cancel_requested = lambda: kind == "cancelled"
                with self.assertRaises(type(error)):
                    body.start()
                expected = 2 if kind == "timeout" else 1
                self.assertEqual(body.transport_factory.call_count, expected)
                self.assertEqual(failed.abort.call_count, expected)
                self.assertEqual(failed.close.call_count, expected)
                self.assertFalse(body._started)

    def test_lost_motion_receipt_is_never_replayed_as_a_startup_retry(self):
        body, _, _ = ev3_test_body(front=False)
        transport = body.transport
        request = transport.request
        calls = []
        def fail_pulse(operation, *args, **kwargs):
            calls.append(operation)
            if operation == "pulse":
                raise EV3NavigationTimeoutError("pulse", 30, "")
            return request(operation, *args, **kwargs)
        transport.request = fail_pulse
        with self.assertRaises(EV3NavigationTimeoutError):
            body.execute(ADVANCE)
        self.assertEqual(calls, ["pulse"])
        self.assertIs(body.transport, transport)
        body.close()

    def test_blocked_multi_slice_turn_does_not_invent_completed_slices(self):
        body, _, _ = ev3_test_body(ir_response=((0, 0), (200, 44), (300, 51), (1_000, 100)))
        self.addCleanup(body.close)
        body.execute(ADVANCE)
        first = body.execute("TURN_LEFT_90")
        self.assertFalse(first["completed"])
        pose = body.pose
        second = body.execute("TURN_LEFT_90")
        self.assertFalse(second["completed"])
        self.assertEqual(second["reason"], "simulation_body_blocked")
        self.assertEqual(body.pose, pose)

    def test_measured_box_reflections_survive_without_a_near_gate_trigger(self):
        # Measured white/dark returns at ~20 cm. A sensor miss must remain
        # possible in simulation; it must not be repaired by world geometry.
        for raw in (44, 71):
            with self.subTest(raw=raw):
                body, profile, mission = ev3_test_body(ir_response=((0, 20), (200, raw), (1_000, 100)))
                self.addCleanup(body.close)
                body.scan()
                context = ev3_navigation_context(body=body, descriptor=profile.descriptor,
                    footprint=profile.hazard_calibration.robot_footprint, mission=mission, goal="Reach goal")
                first = context.local_map_evidence["ir_survey"]["samples"][0]
                self.assertEqual(first["raw_ir_proximity"], raw)
                self.assertFalse(body.survey["samples"][0]["near_obstacle"])
                self.assertNotIn("forward_motion_blocked", first)
                self.assertNotIn("forward_motion_blocked", context.observation["infrared"])
                self.assertNotIn("near_obstacle", first)
                self.assertIn("ir_reference_observations", context.local_map_evidence)
                self.assertFalse(context.local_map_evidence["metric_distance_available"])
                layout = context.local_map_evidence["sensor_layout"]
                self.assertEqual(layout["infrared_mount"], "head, facing forward")
                self.assertEqual(layout["color_sensor_mount"], "right hand; not a range sensor")
                self.assertIn("approximate origin", layout["ray_origin_basis"])
                self.assertEqual(context.local_map_evidence["robot_footprint"],
                                 profile.hazard_calibration.robot_footprint.planner_view())
                if raw == 71:
                    # Dark target: the gate must still be open when physical
                    # contact occurs, just as the live run exposed.
                    receipts = [body.execute(ADVANCE) for _ in range(3)]
                    self.assertTrue(any(item["reason"] == "simulation_body_blocked" for item in receipts))
                    self.assertFalse(body.observation["infrared"]["blocked"])

    def test_full_survey_preserves_real_bearings_and_both_side_openings(self):
        for side in ("right", "left"):
            with self.subTest(side=side):
                body, profile, mission = ev3_test_body(side=side)
                self.addCleanup(body.close)
                survey = body.scan()
                self.assertTrue(survey["complete"])
                self.assertEqual(len(survey["samples"]), 13)
                self.assertEqual([s["requested_bearing_deg"] for s in survey["samples"]], list(range(0, 361, 30)))
                self.assertLess(abs(body.pose.heading_mdeg), 7_500)
                self.assertEqual(body.pose.heading_mdeg, body.transport.simulation.pose("ev3").heading_mdeg)
                rays = {s["requested_bearing_deg"]: s for s in survey["samples"]}
                self.assertTrue(rays[0]["near_obstacle"])
                self.assertEqual(rays[90]["near_obstacle"], side == "left")
                self.assertEqual(rays[270]["near_obstacle"], side == "right")
                context = ev3_navigation_context(body=body, descriptor=profile.descriptor,
                    footprint=profile.hazard_calibration.robot_footprint, mission=mission,
                    goal="Reach the point 80 cm ahead of the start.")
                self.assertNotIn(ADVANCE, context.available_actions)
                self.assertFalse(context.completion_allowed)
                self.assertFalse(context.local_map_evidence["metric_distance_available"])
                self.assertEqual(context.observation["directional_goal"]["x_mm"], 800)

    def test_full_survey_does_not_add_a_correction_pulse_when_pose_drifts(self):
        body, _, _ = ev3_test_body(front=False)
        self.addCleanup(body.close)
        original_execute = body.execute
        commands = []

        def underreporting_execute(action, *, turn_mdeg=None):
            if turn_mdeg is not None:
                commands.append(turn_mdeg)
            result = original_execute(action, turn_mdeg=turn_mdeg)
            if turn_mdeg is not None:
                # Mimic a small encoder/odometry under-report after each
                # physical step.  A residual-based scan would eventually
                # round that drift into a 45-degree correction.
                body.pose = replace(
                    body.pose,
                    heading_mdeg=normalize_heading_mdeg(body.pose.heading_mdeg - 4_000),
                )
            return result

        body.execute = underreporting_execute
        survey = body.scan()

        self.assertTrue(survey["complete"])
        self.assertEqual(commands, [30_000] * 12)
        self.assertEqual(len(survey["samples"]), 13)

    def test_missing_sample_keeps_survey_and_model_authored_route(self):
        body, profile, mission = ev3_test_body(missing_sample=4)
        self.addCleanup(body.close)
        survey = body.scan()
        self.assertTrue(survey["complete"])
        self.assertIsNone(survey["samples"][3]["near_obstacle"])
        waypoints = ({"x_mm": 0, "y_mm": 350, "purpose": "side clearance"},
                     {"x_mm": 700, "y_mm": 350, "purpose": "pass box"})
        context = ev3_navigation_context(body=body, descriptor=profile.descriptor,
            footprint=profile.hazard_calibration.robot_footprint, mission=mission,
            goal="Reach the goal", waypoints=waypoints)
        self.assertEqual(context.active_waypoint_plan, waypoints)
        self.assertGreater(context.active_waypoint_geometry["heading_error_deg"], 0)
        self.assertTrue(any(item["raw_ir_proximity"] is None for item in context.local_map_evidence["ir_survey"]["samples"]))

    def test_front_half_survey_returns_to_start_and_forward_uses_measured_encoders(self):
        body, _, _ = ev3_test_body(front=False)
        self.addCleanup(body.close)
        scan = body.scan(full=False)
        self.assertTrue(scan["complete"])
        self.assertLess(abs(body.pose.heading_mdeg), 7_500)
        result = body.execute(ADVANCE)
        self.assertTrue(result["completed"])
        self.assertGreater(body.pose.x_mm, 0)
        truth = body.transport.simulation.pose("ev3")
        self.assertLessEqual(abs(body.pose.x_mm - truth.x_mm), 2)
        self.assertLessEqual(abs(body.pose.y_mm - truth.y_mm), 2)

    def test_old_scan_stays_at_its_original_position_and_heading_after_a_turn_and_move(self):
        body, profile, mission = ev3_test_body(side="right")
        self.addCleanup(body.close)
        body.scan()
        body.execute("TURN_LEFT_90")
        body.execute(ADVANCE)
        context = ev3_navigation_context(body=body, descriptor=profile.descriptor,
            footprint=profile.hazard_calibration.robot_footprint, mission=mission,
            goal="Reach the goal", waypoints=({"x_mm": 800, "y_mm": body.pose.y_mm, "purpose": "pass"},))
        survey = context.local_map_evidence["ir_survey"]
        self.assertEqual(survey["movement_commands_since_scan"], 2)
        self.assertEqual(survey["samples"][0]["observed_from"], {"x_mm": 0, "y_mm": 0})
        self.assertEqual(survey["samples"][0]["ray_heading_deg_in_episode"], 0)
        self.assertEqual(survey["samples"][0]["raw_ir_proximity"], 20)
        self.assertGreater(context.observation["odometry"]["y_mm"], 0)
        self.assertAlmostEqual(context.observation["infrared"]["ray_heading_deg_in_episode"], 90, delta=1)
        self.assertEqual(context.active_waypoint_geometry["bearing_deg"], 0)
        self.assertAlmostEqual(context.active_waypoint_geometry["heading_error_deg"], -90, delta=1)
        self.assertNotIn("bearing_deg", survey["samples"][0])

    def test_cancelled_survey_still_shuts_down_worker(self):
        cancelled = threading.Event()
        body, _, _ = ev3_test_body(cancel_requested=cancelled.is_set)
        cancelled.set()
        self.assertEqual(body.scan()["reason"], "cancelled")
        body.close()
        self.assertTrue(body.transport.shutdown_complete)

    def test_worker_limits_match_deployed_profile_and_are_enforced(self):
        from ev3.navigation_profile import MAX_PROCESS_SECONDS, MAX_REQUESTS, MAX_PULSES, MAX_PULSE_DURATION_MS
        body, _, _ = ev3_test_body(front=False)
        self.addCleanup(body.close)
        transport = body.transport
        self.assertEqual((transport.PROCESS_LIMIT_MS, transport.REQUEST_LIMIT, transport.PULSE_LIMIT, transport.MOTION_LIMIT_MS),
                         (MAX_PROCESS_SECONDS * 1_000, MAX_REQUESTS, MAX_PULSES, MAX_PULSE_DURATION_MS))
        transport._pulse_count = MAX_PULSES
        with self.assertRaises(EV3NavigationRemoteError) as error:
            transport.request("pulse", {"action": ADVANCE}, 30)
        self.assertEqual(error.exception.code, "pulse_budget_exhausted")
        self.assertEqual(body.pose.x_mm, 0)

    def test_worker_renewal_keeps_pose_survey_and_only_executes_next_command_once(self):
        body, _, _ = ev3_test_body(front=False)
        self.addCleanup(body.close)
        body.scan()
        body.execute(ADVANCE)
        pose, survey, old = body.pose, body.survey, body.transport
        old._pulse_count = old.PULSE_LIMIT
        body.observe()
        result = body.execute(ADVANCE)
        self.assertTrue(old.shutdown_complete)
        self.assertIsNot(body.transport, old)
        self.assertIs(body.survey, survey)
        self.assertTrue(result["completed"])
        self.assertEqual(body.pose.verified_motion_count, pose.verified_motion_count + 1)
        self.assertEqual(body.pose.x_mm - pose.x_mm, 52)
        truth = body.transport.simulation.pose("ev3")
        self.assertEqual((body.pose.x_mm, body.pose.y_mm), (truth.x_mm, truth.y_mm))

    def test_observe_reconciles_encoder_motion_between_receipts(self):
        body, _, _ = ev3_test_body(front=False)
        self.addCleanup(body.close)
        injected = inject_unrecorded_turn(body.transport)
        body.observe()
        truth = body.transport.simulation.pose("ev3")
        self.assertEqual(body.pose.heading_mdeg, truth.heading_mdeg)
        self.assertEqual(body.pose.heading_mdeg, injected)
        self.assertEqual(body.pose.verified_motion_count, 0)

    def test_scan_reconciles_encoder_motion_during_sampling(self):
        body, _, _ = ev3_test_body(front=False)
        self.addCleanup(body.close)
        transport = body.transport
        original_request = transport.request
        injected = []

        def request(operation, *args, **kwargs):
            if operation == "scan_sample":
                injected.append(inject_unrecorded_turn(transport))
            return original_request(operation, *args, **kwargs)

        transport.request = request
        survey = body.scan()
        truth = transport.simulation.pose("ev3")
        self.assertTrue(survey["complete"])
        self.assertEqual(body.pose.heading_mdeg, truth.heading_mdeg)
        self.assertEqual(
            body.pose.total_turn_mdeg - sum(injected),
            360_036,
        )

    def test_execute_reconciles_encoder_gap_inside_motion_receipt(self):
        body, _, _ = ev3_test_body(front=False)
        self.addCleanup(body.close)
        transport = body.transport
        original_request = transport.request
        mutated = False

        def request(operation, *args, **kwargs):
            nonlocal mutated
            response = original_request(operation, *args, **kwargs)
            if operation == SCAN_TURN_OPERATION and not mutated:
                result = response["result"]
                shift = {"drive_b": -76, "drive_c": 76}
                for motor in result["observation"]["motors"]:
                    if motor["role"] in shift:
                        motor["position"] += shift[motor["role"]]
                mutated = True
            return response

        transport.request = request
        result = body.execute(SCAN_TURN_OPERATION, turn_mdeg=30_000)
        self.assertTrue(result["completed"])
        self.assertEqual(body.pose.total_turn_mdeg, 40_415)
        self.assertEqual(body.pose.heading_mdeg, 40_415)

    def test_worker_renewal_preserves_absolute_encoder_anchor(self):
        body, _, _ = ev3_test_body(front=False)
        self.addCleanup(body.close)
        body.execute(ADVANCE)
        pose = body.pose
        old = body.transport
        body._requests_remaining = 2
        body.observe()
        self.assertIsNot(body.transport, old)
        self.assertEqual(body.pose, pose)

    def test_model_wait_releases_worker_and_resumes_sensing_without_rescanning(self):
        body, _, _ = ev3_test_body(front=False)
        self.addCleanup(body.close)
        body.scan()
        body.execute(ADVANCE)
        pose, survey, old = body.pose, body.survey, body.transport
        body.release_for_planning()
        self.assertTrue(old.shutdown_complete)
        request_count = len(old.requests)
        # The model takes longer than an entire worker lifetime.
        old._started_clock -= 300
        body.observe()
        self.assertIsNot(body.transport, old)
        self.assertEqual(body.pose, pose)
        self.assertIs(body.survey, survey)
        self.assertEqual([r[0] for r in old.requests[request_count:]], ["describe", "observe"])

    def test_time_and_request_budget_renewal_happen_before_next_sensor_request(self):
        for budget in ("time", "requests"):
            with self.subTest(budget=budget):
                body, _, _ = ev3_test_body(front=False)
                self.addCleanup(body.close)
                old = body.transport
                if budget == "time":
                    old._started_clock -= 155
                    body._deadline -= 155
                else:
                    body._requests_remaining = 2
                    old._request_count = old.REQUEST_LIMIT - 2
                body.observe()
                self.assertTrue(old.shutdown_complete)
                self.assertIsNot(body.transport, old)
                self.assertEqual(body.pose.verified_motion_count, 0)


if __name__ == "__main__":
    unittest.main()
