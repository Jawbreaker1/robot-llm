import unittest
import json
from pathlib import Path
from dataclasses import replace
from itertools import product
from unittest.mock import Mock, patch

from robot_agent.blast_navigation_simulation import run_blast_gemma_scenario
from robot_agent.ev3_navigation_transport import EV3NavigationSSHTransport
from robot_agent.navigation_simulation_scenarios import (
    blast_box_front, blast_measured_box_and_chair, blast_open_floor,
)
from robot_agent.lm_studio_controller_action import (
    COMPLETE, FOLLOW_WAYPOINT, ControllerActionDecision, ControllerActionPlannerResult,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE, REVERSE, SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90,
    expected_scan_turn_spec,
)

from robot_agent.multi_robot_navigation_simulator import (
    MultiRobotNavigationSimulator,
    RectangleObstacle,
    SimulatedRobot,
    SimulationGoal,
)
from robot_agent.navigation_state import PoseEstimate
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_navigation_execution_contract import (
    EV3NavigationExecutionContract,
)
from robot_agent.physical_odometry import (
    DriveMotorRoles,
    OdometryCalibration,
    verified_motion_from_result,
)
from robot_agent.simulation_robot_adapters import (
    SharedWorldBlastController,
    SharedWorldEV3Transport,
)


EV3_ODOMETRY = OdometryCalibration(
    linear_mm_per_encoder_degree=0.35,
    turn_mdeg_per_opposed_encoder_degree=132,
)


def footprint(*, rear=60, right=100):
    return RobotFootprint(
        front_extent_mm=110,
        rear_extent_mm=rear,
        left_extent_mm=105,
        right_extent_mm=right,
        clearance_margin_mm=10,
        calibration_status="simulation-test",
        calibration_evidence="current LEGO robot dimensions",
    )


def shared_world(*, obstacles=()):
    return MultiRobotNavigationSimulator(
        bounds=(-1_000, -1_000, 2_000, 1_000),
        obstacles=obstacles,
        robots=(
            SimulatedRobot(
                "blast",
                PoseEstimate(0, -350, 0),
                footprint(),
                1_500,
            ),
            SimulatedRobot(
                "ev3",
                PoseEstimate(0, 350, 0),
                footprint(rear=90, right=130),
                1_000,
            ),
        ),
        goals=(
            SimulationGoal("blast", 1_000, -350),
            SimulationGoal("ev3", 1_000, 350),
        ),
    )


class SimulationRobotAdapterTests(unittest.TestCase):
    def test_long_blast_simulation_uses_the_configured_mission_distance(self):
        scenario = blast_open_floor()
        scenario = replace(scenario, goals=(SimulationGoal("blast", 2000, 0),))
        contexts = []

        class Planner:
            def decide(self, context):
                contexts.append(context)
                return ControllerActionPlannerResult(ControllerActionDecision(
                    action=ADVANCE, confidence_milli=1000, assessment="Test mission distance",
                    plan=(), utterance=None,
                ), latency_ms=0)

        result = run_blast_gemma_scenario(
            scenario, minimum_forward_progress_mm=2000,
            goal="Reach the goal 200 cm straight ahead.", max_decisions=1,
            planner_factory=lambda _: Planner(),
        )
        self.assertIsNone(result["error_type"])
        self.assertEqual(contexts[0].local_map_evidence["directional_goal"]["target_x_mm"], 2000)
        self.assertFalse(result["goal_reached"])

    def test_default_planner_uses_shared_model_transport_for_both_providers(self):
        # Scripted reply checks provider wiring, not model navigation ability.
        for model in ("codex/gpt-6-astra", "qwen/qwen3.8-27b"):
            with self.subTest(model=model):
                def reply(url, body, headers, timeout, maximum):
                    request = json.loads(body)
                    self.assertEqual(request["model"], model)
                    return json.dumps({"object": "chat.completion", "model": model,
                        "choices": [{"index": 0, "finish_reason": "stop", "message": {
                            "role": "assistant", "content": json.dumps({
                                "action": FOLLOW_WAYPOINT, "confidence_milli": 1000,
                                "assessment": "Provider wiring test", "plan": [], "utterance": None,
                                "waypoint": {"x_mm": 0, "y_mm": 300, "purpose": "Wiring test"},
                                "following_waypoints": [],
                            }),
                        }}]}).encode()
                with patch("robot_agent.blast_navigation_simulation.robot_model_transport",
                           side_effect=reply) as transport:
                    result = run_blast_gemma_scenario(blast_open_floor(), model=model, max_decisions=1)
                self.assertIsNone(result["error_type"])
                self.assertEqual(len(result["planner_decisions"]), 1)
                self.assertEqual(transport.call_count, 1)

    def test_scan_blocked_before_first_turn_retains_center_evidence(self):
        # Actual pose from the Qwen blocked-corner replay, not ideal free space.
        scenario = blast_box_front()
        pose = PoseEstimate(575, 302, -91818)
        scenario = replace(scenario, robots=(replace(scenario.robots[0], pose=pose),))
        world = scenario.build()
        controller = SharedWorldBlastController(world, world_robot_id="blast")
        result = controller._scan_result(surroundings=False)
        self.assertEqual(world.pose("blast"), pose)
        scan = result["scan"]
        self.assertEqual(scan["state"], "partial")
        self.assertEqual(scan["sweep_coverage_deg"], 0)
        self.assertEqual(result["receipt"]["turn_count"], 1)
        self.assertFalse(scan["restoration_verified"])
        self.assertEqual([ray["side"] for ray in scan["rays"]], ["center"])




    def test_front_scan_at_recorded_stall_returns_partial_without_spinning_on(self):
        scenario = blast_measured_box_and_chair()
        scenario = replace(scenario, robots=(replace(
            scenario.robots[0], pose=PoseEstimate(277, -294, 5673),
        ),))
        controller = SharedWorldBlastController(scenario.build(), world_robot_id="blast")
        result = controller._scan_result(surroundings=False)
        self.assertEqual(result["scan"]["state"], "partial")
        self.assertLess(result["receipt"]["turn_count"], 16)
        self.assertFalse(result["scan"]["restoration_verified"])
        self.assertFalse(result["observation"]["motion_active"])

    def test_interrupted_waypoint_returns_to_model_with_route_and_allows_retreat(self):
        # Execution near a recorded corner, not a model navigation pass.
        scan = json.loads((Path(__file__).parent / 'fixtures' /
                           'blast_box_scan_20260902.json').read_text())['scans'][0]
        route = [
            {"x_mm": 0, "y_mm": -300, "purpose": "detour_right"},
            {"x_mm": 600, "y_mm": -300, "purpose": "advance_past_obstacle"},
            {"x_mm": 277, "y_mm": -450, "purpose": "detour_right_clear_obstacle"},
        ]
        following = (
            {"x_mm": 600, "y_mm": -450, "purpose": "pass_obstacle"},
            {"x_mm": 600, "y_mm": 0, "purpose": "return_to_centerline"},
            {"x_mm": 800, "y_mm": 0, "purpose": "goal"},
        )
        contexts = []

        class Planner:
            def decide(self, context):
                contexts.append(context)
                index = (0 if not context.active_waypoint or context.active_waypoint == route[0]
                         else 1 if context.active_waypoint == route[1] else 3)
                if (index == 1 and context.history
                        and ADVANCE not in context.available_actions):
                    index = 2
                tail = following
                if index == 0:
                    tail = (route[1], *following[1:])
                elif index == 1:
                    tail = following[1:]
                return ControllerActionPlannerResult(ControllerActionDecision(
                    action=(FOLLOW_WAYPOINT if index < 2 else
                            TURN_RIGHT_90 if index == 2 else REVERSE if context.previous_decision["action"] == TURN_RIGHT_90 else "OBSERVE"),
                    confidence_milli=1000, assessment="Try the route; retreat if the turn stalls",
                    plan=(), utterance=None, waypoint=route[min(index, 2)],
                    following_waypoints=tail,
                ), latency_ms=0)

        result = run_blast_gemma_scenario(
            blast_measured_box_and_chair(), startup_scan=scan, range_dropout_reads=4,
            planner_factory=lambda _: Planner(), max_decisions=64,
        )
        self.assertEqual(result['terminal_reason'], 'decision_budget_exhausted')
        returned = next(c for c in contexts if c.active_waypoint == route[2])
        self.assertEqual(returned.goal, contexts[0].goal)
        self.assertEqual(returned.active_waypoint, route[2])
        self.assertEqual(returned.active_waypoint_plan, (route[2], *following))
        # Endpoint tracking now stops this approach earlier, while there is
        # still turning room. The interruption and retained recovery route must
        # survive the model's subsequent turn; a stalled turn is not required.
        interrupted = next(c for c in contexts if c.active_waypoint == route[1] and ADVANCE not in c.available_actions)
        self.assertIn(REVERSE, interrupted.available_actions)
        self.assertEqual(returned.history[-1]['action'], TURN_RIGHT_90)
        self.assertIn(REVERSE, returned.available_actions)
        self.assertLess(result['final_pose']['x_mm'], returned.observation['odometry']['x_mm'])
        self.assertEqual(result['scans'], 1)
        self.assertLess(result['blocked_moves'], 10)

    def test_waypoint_tracking_corrects_heading_drift_without_new_decision(self):
        for side in (-1, 1):
            with self.subTest(side=side):
                class HeadingDriftController(SharedWorldBlastController):
                    drift_applied = False

                    def _apply_encoder_motion(self, *, left_delta_deg, right_delta_deg):
                        if left_delta_deg == right_delta_deg and not self.drift_applied:
                            # A real heading disturbance, visible through the IMU;
                            # do not change the navigation code's calibration.
                            heading = self.simulation.pose(self.world_robot_id).heading_mdeg
                            self.simulation.rotate(self.world_robot_id, side * 101_000 - heading)
                            self.drift_applied = True
                        return super()._apply_encoder_motion(
                            left_delta_deg=left_delta_deg, right_delta_deg=right_delta_deg,
                        )

                contexts = []
                class Planner:
                    def decide(self, context):
                        contexts.append(context)
                        return ControllerActionPlannerResult(ControllerActionDecision(
                            action=FOLLOW_WAYPOINT if len(contexts)==1 or context.active_waypoint else "OBSERVE", confidence_milli=1000,
                            assessment="Follow the chosen side", plan=(), utterance=None,
                            waypoint={"x_mm": 0, "y_mm": side * 750, "purpose": "Side target"} if len(contexts)==1 else None,
                        ), latency_ms=0)

                with patch("robot_agent.blast_navigation_simulation.SharedWorldBlastController",
                           HeadingDriftController):
                    result = run_blast_gemma_scenario(
                        replace(blast_box_front(), obstacles=(), bounds=(-500, -1200, 1250, 1200)),
                        planner_factory=lambda _: Planner(), max_decisions=64,
                    )
                self.assertIsNone(result["error_code"])
                self.assertGreater(len(contexts), 2)  # Initial route and post-turn review.
                self.assertEqual(result["scans"], 1)
                self.assertEqual(result["blocked_moves"], 0)
                self.assertLess(abs(result["final_pose"]["x_mm"]), 75)
                self.assertLess(abs(result["final_pose"]["y_mm"] - side * 750), 30)
                self.assertTrue(any(abs(c.observation["odometry"]["heading_deg"]-side*90)>5 for c in contexts[2:]))

    def test_course_correction_preserves_the_model_selected_box_side(self):
        route = [
            {"x_mm": 0, "y_mm": 300, "purpose": "Side entry"},
            {"x_mm": 600, "y_mm": 300, "purpose": "Pass box"},
        ]
        contexts = []
        class Planner:
            def decide(self, context):
                contexts.append(context)
                return ControllerActionPlannerResult(ControllerActionDecision(
                    action=FOLLOW_WAYPOINT if len(contexts)==1 or context.active_waypoint else "OBSERVE", confidence_milli=1000,
                    assessment="Replay the two real Qwen route legs", plan=(), utterance=None,
                    waypoint=route[0] if len(contexts)==1 else None,
                    following_waypoints=(route[1],) if len(contexts)==1 else (),
                ), latency_ms=0)
        result = run_blast_gemma_scenario(
            blast_box_front(), planner_factory=lambda _: Planner(), max_decisions=32,
        )
        self.assertIsNone(result["error_code"])
        self.assertEqual(len(contexts), 32)
        self.assertEqual(result["blocked_moves"], 0)
        self.assertEqual(result["scans"], 1)
        self.assertLess(abs(result["final_pose"]["x_mm"] - 600), 30)
        self.assertLess(abs(result["final_pose"]["y_mm"] - 300), 75)
        # The corrected startup scan can remove the old heading disturbance.
        # Drift correction itself is exercised with an explicit disturbance above.

    def test_blocked_rotation_reports_partial_motion_for_both_adapters(self):
        for robot_id in ("blast", "ev3"):
            with self.subTest(robot_id=robot_id):
                world = MultiRobotNavigationSimulator(
                    bounds=(-1000, -1000, 1000, 1000),
                    obstacles=(RectangleObstacle("corner", 130, -110, 155, -75),),
                    robots=(SimulatedRobot(robot_id, PoseEstimate(0, 0, 0), footprint(), 1000),),
                    goals=(SimulationGoal(robot_id, 800, 0),),
                )
                if robot_id == "blast":
                    controller = SharedWorldBlastController(world, world_robot_id=robot_id)
                    observation = controller.command("turn_left")["observation"]
                    self.assertLess(abs(observation["motor_angles_deg"]["left_drive"]), 45)
                else:
                    transport = SharedWorldEV3Transport(
                        world, world_robot_id=robot_id, odometry=EV3_ODOMETRY,
                    )
                    transport.start()
                    response = transport.request("pulse", {"action": TURN_LEFT_90}, 1.0)
                    validator = EV3NavigationSSHTransport(target="unused.local", controller_id="ev3", remote_worker_path="/unused.py")
                    validator._validate_success_result("pulse", {"action": TURN_LEFT_90}, response)
                    outcome = response["result"]["outcome"]
                    self.assertEqual(outcome["status"], "interrupted")
                    # Verified partial travel is not successful completion of
                    # the requested turn, nor an invented encoder fault.
                    self.assertTrue(all(item["encoder_verification"]["passed"] for item in outcome["slices"]))
                    self.assertEqual(outcome["encoder_verification"]["verified_slice_count"], len(outcome["slices"]))
                    self.assertFalse(outcome["encoder_verification"]["passed"])
                    self.assertLess(outcome["completed_slice_count"], outcome["requested_slice_count"])
                self.assertTrue(any(event.kind == "blocked" for event in world.events))
                self.assertLess(abs(world.pose(robot_id).heading_mdeg), 22_050)

    def test_final_turn_with_missing_echo_keeps_route_and_reaches_goal(self):
        # A regression of the physical final approach, not an LLM planning test.
        class FinalApproachDropout(SharedWorldBlastController):
            def _observation(self):
                observation = super()._observation()
                pose = self.simulation.pose(self.world_robot_id)
                if (missing_echo and len(dropouts) < 3
                        and pose.x_mm > 600 and abs(pose.y_mm) < 50
                        and abs(pose.heading_mdeg) < 20_000):
                    observation["distance_mm"] = 2_000
                    dropouts.append(pose)
                return observation

        for missing_echo in (False, True):
            with self.subTest(missing_echo=missing_echo):
                dropouts = []
                side = 450
                route = [
                    {"x_mm": 0, "y_mm": side, "purpose": "Clear box"},
                    # Leave actual body clearance behind the 520 mm box edge.
                    # The old 650 mm corner only worked because drift carried
                    # the body farther forward than the requested endpoint.
                    {"x_mm": 720, "y_mm": side, "purpose": "Pass box"},
                    {"x_mm": 720, "y_mm": 0, "purpose": "Return behind box"},
                    {"x_mm": 800, "y_mm": 0, "purpose": "Reach goal"},
                ]
                contexts = []
                class Planner:
                    def decide(self, context):
                        contexts.append(context)
                        index = (0 if len(contexts) == 1 else
                                 next((i for i, point in enumerate(route)
                                       if context.active_waypoint is not None
                                       and point["purpose"] == context.active_waypoint["purpose"]), len(route)))
                        if index == 2:
                            # Use the encoder position for the final perpendicular
                            # leg; the simulator retains real turn/heading residue.
                            route[2] = dict(route[2], x_mm=context.observation["odometry"]["x_mm"])
                        point = route[index] if index < len(route) else None
                        action = FOLLOW_WAYPOINT if point else COMPLETE
                        if point is None and not context.completion_allowed:
                            action = (TURN_LEFT_90 if context.observation["odometry"]["heading_deg"] < 0
                                      else TURN_RIGHT_90)
                        return ControllerActionPlannerResult(ControllerActionDecision(
                            action=action,
                            confidence_milli=1000, assessment="Follow retained route",
                            plan=(), utterance=None, waypoint=point,
                            following_waypoints=tuple(route[index + 1:]),
                        ), latency_ms=0)
                with patch("robot_agent.blast_navigation_simulation.SharedWorldBlastController",
                           FinalApproachDropout):
                    result = run_blast_gemma_scenario(
                        blast_box_front(), planner_factory=lambda _: Planner(),
                        max_decisions=64,
                    )
                self.assertIsNone(result["error_code"])
                self.assertTrue(result["completed"], result["terminal_reason"])
                self.assertEqual(result["scans"], 1)
                self.assertEqual(result["blocked_moves"], 0)
                self.assertEqual(contexts[2].active_waypoint_plan[-1]["purpose"], "Reach goal")
                self.assertEqual(len(dropouts), 3 if missing_echo else 0)
                self.assertEqual(contexts[-1].observation["sensors"]["range_state"], "MEASURED")
                self.assertLessEqual(result["distance_to_goal_mm"], 50)
                # Reproduces the missing final leg: a turn behind the box is
                # not arrival. The wheels must then drive toward the goal.
                self.assertGreater(result["final_pose"]["x_mm"], 700)

    def test_recorded_open_floor_no_echo_follows_waypoint_to_goal(self):
        contexts = []
        waypoint = {"x_mm": 800, "y_mm": 0, "purpose": "Goal"}
        scan = json.loads((Path(__file__).parent / "fixtures" /
                           "blast_open_floor_scan_20260910.json").read_text())["scans"][0]
        # Execution regression using physical startup data, not an LLM score.
        class Planner:
            def decide(self, context):
                contexts.append(context)
                action = COMPLETE if context.completion_allowed else FOLLOW_WAYPOINT
                if action == FOLLOW_WAYPOINT and action not in context.available_actions:
                    raise AssertionError("Following vanished on open floor")
                return ControllerActionPlannerResult(ControllerActionDecision(
                    action=action, confidence_milli=1000, assessment="Follow the route",
                    plan=(action,), utterance=None,
                    waypoint=waypoint if action == FOLLOW_WAYPOINT else None,
                ), latency_ms=0)
        result = run_blast_gemma_scenario(
            blast_open_floor(), startup_scan=scan,
            planner_factory=lambda _: Planner(), max_decisions=64,
        )
        self.assertTrue(result["completed"], result.get("error_message"))
        self.assertTrue(result["goal_reached"])
        self.assertEqual(result["scans"], 1)
        self.assertEqual(result["blocked_moves"], 0)
        self.assertNotIn("drive_reverse", result["commands_tail"])
        self.assertGreater(len(contexts), 2)
        for context in contexts:
            self.assertIsNone(context.observation["sensors"]["distance_mm"])
            self.assertEqual(context.observation["sensors"]["range_state"], "NO_VALID_DISTANCE")

    def test_retained_waypoint_keeps_its_side_through_transient_range_loss(self):
        # Execution contract only. Real Qwen decisions are validated separately.
        for front_x in (200, 320):
            for y_mm, dropout_reads in product((-450, 450), (0, 4)):
                with self.subTest(front_x=front_x, y_mm=y_mm, dropout_reads=dropout_reads):
                    contexts = []
                    waypoint = {"x_mm": 0, "y_mm": y_mm, "purpose": "Side contract"}
                    class Planner:
                        def decide(self, context):
                            contexts.append(context)
                            return ControllerActionPlannerResult(ControllerActionDecision(
                                action=FOLLOW_WAYPOINT if len(contexts)==1 or context.active_waypoint else "OBSERVE", confidence_milli=1000,
                                assessment="Follow the chosen point", plan=(FOLLOW_WAYPOINT,),
                                utterance=None, waypoint=waypoint if len(contexts)==1 else None,
                            ), latency_ms=0)
                    result = run_blast_gemma_scenario(
                        replace(blast_box_front(), obstacles=(
                            RectangleObstacle("box", front_x, -180, front_x + 200, 180),
                        )),
                        planner_factory=lambda _: Planner(), max_decisions=64,
                        range_dropout_reads=dropout_reads,
                    )
                    self.assertIsNone(result["error_code"])
                    self.assertGreater(len(contexts), 2)
                    if front_x == 200:
                        self.assertNotIn(ADVANCE, contexts[0].available_actions)
                    self.assertEqual(result["scans"], 1)
                    self.assertEqual(result["blocked_moves"], 0)
                    self.assertLess(abs(result["final_pose"]["y_mm"] - y_mm), 30)
                    for coordinate in ("x_mm", "y_mm"):
                        self.assertLessEqual(abs(
                            result["estimated_pose"][coordinate]
                            - result["final_pose"][coordinate]
                        ), 10)
                    self.assertEqual(
                        result["estimated_pose"]["heading_mdeg"],
                        result["final_pose"]["heading_mdeg"],
                    )
                    self.assertTrue(all(c.observation["odometry"]["y_mm"] * (1 if y_mm>0 else -1) >= -25 for c in contexts))

    def test_blast_and_ev3_commands_move_in_the_same_concurrent_world(self):
        world = shared_world()
        blast = SharedWorldBlastController(
            world,
            world_robot_id="blast",
        )
        ev3 = SharedWorldEV3Transport(
            world,
            world_robot_id="ev3",
            odometry=EV3_ODOMETRY,
        )

        def run_blast(_simulation):
            return blast.command("drive_forward")

        def run_ev3(_simulation):
            ev3.start()
            description = ev3.request("describe", {}, 1.0)
            EV3NavigationExecutionContract.parse_description(description)
            return ev3.request("pulse", {"action": "ADVANCE"}, 1.0)

        results = world.run_concurrently({
            "blast": run_blast,
            "ev3": run_ev3,
        })

        ev3_observation = EV3NavigationExecutionContract.parse_observation(
            "pulse",
            results["ev3"],
            "ADVANCE",
        )
        motion = verified_motion_from_result(
            "ADVANCE",
            results["ev3"]["result"],
            DriveMotorRoles(left="drive_b", right="drive_c"),
        )
        self.assertEqual(world.pose("blast").x_mm, 45)
        self.assertEqual(world.pose("ev3").x_mm, 52)
        self.assertEqual(
            results["blast"]["observation"]["motor_angles_deg"][
                "left_drive"
            ],
            90,
        )
        self.assertEqual(ev3_observation["state_version"], 2)
        self.assertEqual(motion.left_encoder_delta_degrees, 150)

    def test_both_adapters_observe_the_same_obstacle_without_route_hints(self):
        obstacles = (
            RectangleObstacle("blast-box", 250, -450, 500, -250),
            RectangleObstacle("ev3-box", 200, 250, 450, 450),
        )
        world = shared_world(obstacles=obstacles)
        blast = SharedWorldBlastController(world, world_robot_id="blast")
        ev3 = SharedWorldEV3Transport(
            world,
            world_robot_id="ev3",
            odometry=EV3_ODOMETRY,
        )
        ev3.start()

        blast_observation = blast.snapshot()["observation"]
        ev3_description = EV3NavigationExecutionContract.parse_description(
            ev3.request("describe", {}, 1.0)
        )[0]

        self.assertLess(blast_observation["distance_mm"], 200)
        # The measured white-box return is visible but does not trip the
        # actual gate; unlike ultrasound it must not get an oracle stop.
        self.assertGreater(ev3_description["infrared"]["raw"], 35)
        self.assertLess(ev3_description["infrared"]["raw"], 60)
        self.assertFalse(ev3_description["infrared"]["blocked"])
        self.assertFalse(hasattr(blast, "route"))
        self.assertFalse(hasattr(ev3, "waypoints"))

    def test_blast_trim_completes_one_surroundings_rotation(self):
        world = shared_world()
        blast = SharedWorldBlastController(world, world_robot_id="blast")
        angles = blast.snapshot()["observation"]["motor_angles_deg"]
        permit = blast.issue_no_return_scan_permit(
            expected_drive_angles=angles,
        )

        result = blast.scan_surroundings(action_permit=permit)

        self.assertEqual(result["scan"]["sweep_coverage_deg"], 360.15)
        self.assertEqual(result["receipt"], {
            "turn_count": 17,
            "coverage_complete": True,
        })
        self.assertEqual(world.pose("blast").heading_mdeg, 150)

    def test_blast_body_block_retains_partial_encoder_progress(self):
        world = MultiRobotNavigationSimulator(
            bounds=(-1_000, -1_000, 2_000, 1_000),
            obstacles=(RectangleObstacle("box", 180, -100, 400, 100),),
            robots=(SimulatedRobot(
                "blast", PoseEstimate(0, 0, 0), footprint(), 1_500,
            ),),
            goals=(SimulationGoal("blast", 1_000, 0),),
        )
        blast = SharedWorldBlastController(world, world_robot_id="blast")

        partial = blast.command("drive_forward")
        stopped = blast.command("drive_forward")

        self.assertEqual(world.pose("blast").x_mm, 55)
        self.assertEqual(
            partial["observation"]["motor_angles_deg"]["left_drive"],
            90,
        )
        self.assertEqual(
            stopped["observation"]["motor_angles_deg"]["left_drive"],
            110,
        )
        self.assertEqual(
            sum(event.kind == "blocked" for event in world.events),
            1,
        )


if __name__ == "__main__":
    unittest.main()
