"""One recovery contract exercised through both production episode adapters.

The planner is scripted: these are execution/lifecycle tests, NOT evidence of
Qwen's navigation ability. Sensor readings and motor receipts come from the
existing shared-world simulator. Faults are injected at the I/O boundary, not
by replacing the navigation loop. No physical robot or model server is used.
"""

from copy import deepcopy
from dataclasses import replace
import json
import math
import threading
import unittest

from robot_agent.blast_controller_contract import BlastControllerError
from robot_agent.blast_episode_adapter import BlastEpisodeRuntimeAdapter
from robot_agent.ev3_episode_adapter import EV3EpisodeAdapter
from robot_agent.ev3_navigation_body import EV3NavigationBody
from robot_agent.ev3_navigation_transport import EV3NavigationTimeoutError
from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
from robot_agent.lm_studio import LMStudioProtocolError, LMStudioTimeoutError
from robot_agent.lm_studio_controller_action import COMPLETE, FOLLOW_WAYPOINT
from robot_agent.physical_navigation_contract import ADVANCE, SCAN_FRONT_ARC
from robot_agent.multi_robot_navigation_simulator import (
    MultiRobotNavigationSimulator, SimulatedRobot, SimulationGoal,
)
from robot_agent.navigation_simulation_scenarios import blast_open_floor, blast_box_front
from robot_agent.navigation_state import PoseEstimate
from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge
from robot_agent.robot_control_contract import RobotControlSettings, RobotEpisodeStart, RobotRuntimeUpdate
from robot_agent.robot_control_service import RobotEpisodeContext
from robot_agent.robot_speech_runtime import RobotSpeechRuntime
from robot_agent.simulation_robot_adapters import SharedWorldBlastController
from test_ev3_navigation_body import ValidatedEV3SimulationTransport
from test_ev3_navigation_session import choice


GOAL = "Reach the goal 800 mm straight ahead of the starting position."
ROUTE = (
    {"x_mm": 400, "y_mm": 0, "purpose": "First leg"},
    {"x_mm": 800, "y_mm": 0, "purpose": "Goal"},
)


class RecoveryEpisode:
    """Test wiring only; each robot keeps its real adapter and motor logic."""

    def __init__(self, robot, *, planner_errors=(), fault=None, stop_on_error=False):
        self.robot, self.fault = robot, fault
        self.fault_code = "worker_response_timeout" if robot == "ev3" else "controller_command_timeout"
        self.planner_errors = list(planner_errors)
        self.stop_on_error = stop_on_error
        self.contexts, self.trace, self.updates = [], [], []
        self.fault_index = None
        self.fault_pose = None
        self.context = RobotEpisodeContext(
            "recovery-contract-" + robot, RobotEpisodeStart(GOAL, "en", "contract", 1),
            RobotControlSettings(speech_enabled=False),
            threading.Event(), threading.Event(), self.updates.append,
        )
        if robot == "blast":
            self.world = blast_open_floor().build()
            controller = SharedWorldBlastController(self.world, world_robot_id=robot)
            command, snapshot = controller.command, controller.snapshot

            def observed_snapshot():
                result = snapshot()
                self.trace.append("observation")
                return result

            def request(name, **kwargs):
                return self.io(
                    motion=name.startswith(("drive_", "turn_")),
                    forward=name == "drive_forward",
                    read=name == "observe_settled",
                    execute=lambda: command(name, **kwargs),
                )

            controller.snapshot, controller.command = observed_snapshot, request
            self.adapter = BlastEpisodeRuntimeAdapter(
                controller=controller, planner_factory=lambda model: self,
                max_decisions=64, minimum_forward_progress_mm=800,
                monotonic_ms=controller.monotonic_ms,
            )
        else:
            profile = EV3RSTORMProfile()
            self.world = MultiRobotNavigationSimulator(
                bounds=(-1_200, -1_200, 5_000, 1_200), obstacles=(),
                robots=(SimulatedRobot(robot, PoseEstimate(0, 0, 0),
                                       profile.hazard_calibration.robot_footprint, 1_000),),
                goals=(SimulationGoal(robot, 800, 0),),
            )

            def transport_factory():
                transport = ValidatedEV3SimulationTransport(
                    self.world, world_robot_id=robot, odometry=profile.odometry_calibration,
                )
                request = transport.request

                def wrapped(operation, arguments, timeout, cancel_requested=None):
                    return self.io(
                        motion=operation in ("pulse", "scan_turn"),
                        forward=operation == "pulse" and arguments.get("action") == "ADVANCE",
                        read=operation in ("describe", "observe"),
                        execute=lambda: request(operation, arguments, timeout, cancel_requested),
                        abort=transport.abort,
                    )

                transport.request = wrapped
                return transport

            bridge = PhysicalSpatialMapBridge(
                robot_id=profile.descriptor.robot_id,
                controller_instance_id=profile.descriptor.controller_id,
            )
            self.adapter = EV3EpisodeAdapter(
                body_factory=lambda **options: EV3NavigationBody(
                    transport_factory, calibration=profile.odometry_calibration, **options),
                planner_factory=lambda model: self, descriptor=profile.descriptor,
                footprint=profile.hazard_calibration.robot_footprint,
                spatial_map_provider=bridge, controller_lease=threading.Lock(),
                speech_runtime_factory=lambda event_sink: RobotSpeechRuntime(
                    speaker=lambda *args, **kwargs: None, event_sink=event_sink),
            )

    def io(self, *, motion, forward, read, execute, abort=lambda: None):
        inject = forward and self.fault is not None and self.fault_index is None
        if motion:
            self.trace.append("motion_request")
        # The lost-receipt case really moves the simulator and retains its
        # absolute encoders. The before-motion case does not execute at all.
        result = None if inject and self.fault == "before_motion" else execute()
        if inject:
            self.fault_pose = self.world.pose(self.robot)
            self.fault_index = len(self.trace)
            self.trace.append("fault")
            abort()  # An EV3 SSH timeout closes the channel in production too.
            if self.robot == "ev3":
                raise EV3NavigationTimeoutError("pulse", 30, "injected lost response")
            raise BlastControllerError(
                self.fault_code, "Injected lost response",
                motion_started=self.fault == "lost_receipt", evidence_uncertain=True,
            )
        if read:
            self.trace.append("observation")
        return result

    def decide(self, context):
        self.contexts.append(deepcopy(context))
        self.trace.append("planner")
        if len(self.contexts) > 64:
            self.context.stop_requested.set()
            raise AssertionError("Recovery failed to converge within 64 planner calls")
        # Fail only after the first route leg: there is now real progress and
        # an uncompleted route tail to preserve, not just an empty initial state.
        if len(self.contexts) > 1 and self.planner_errors:
            error = self.planner_errors.pop(0)
            if self.stop_on_error:
                self.context.stop_requested.set()
            self.trace.append("planner_error")
            raise error
        if context.completion_allowed:
            return choice(COMPLETE)
        route = context.active_waypoint_plan if len(self.contexts) > 1 else ROUTE
        if not route:
            self.context.stop_requested.set()
            raise AssertionError("The unfinished route was lost")
        return choice(FOLLOW_WAYPOINT, route[0], route[1:])

    def run(self):
        try:
            return self.adapter.run(self.context)
        finally:
            self.adapter.spatial_map_provider.close()


class RecoveryContract:
    """Inherited unchanged by both robot suites: assertions must not diverge."""

    def episode(self, **options):
        return RecoveryEpisode(self.robot, **options)

    def completed(self, episode):
        try:
            outcome = episode.run()
        except Exception as error:
            self.fail(f"Mission escaped instead of recovering: {type(error).__name__}: {error}")
        for update in episode.updates:
            RobotRuntimeUpdate.from_mapping(update)
        self.assertTrue(outcome.completed, outcome.terminal_reason)
        self.assertLessEqual(abs(episode.world.pose(self.robot).x_mm - 800), 50)
        self.assertLessEqual(abs(episode.world.pose(self.robot).y_mm), 10)
        self.assertTrue(all(context.goal == GOAL for context in episode.contexts))
        self.assertIsNone(episode.contexts[0].previous_decision)
        for context in episode.contexts[1:]:
            self.assertEqual(set(context.previous_decision), {"action", "assessment", "plan"})
            self.assertEqual(context.previous_decision["assessment"], "Test execution, not model planning")
        for context in episode.contexts:
            target = context.observation.get("directional_goal")
            if target is None:
                target = context.local_map_evidence["directional_goal"]
                target = {"x_mm": target["target_x_mm"], "y_mm": target["target_y_mm"]}
            self.assertEqual((target["x_mm"], target["y_mm"]), (800, 0))

    def test_healthy_route_reaches_goal(self):
        episode = self.episode()
        self.completed(episode)
        self.assertGreaterEqual(len(episode.contexts), 3)
        self.assertEqual(episode.contexts[1].active_waypoint_plan, ROUTE)
        motion = episode.contexts[1].execution["next_waypoint_motion"]
        self.assertEqual(motion["action"], ADVANCE)
        projection = motion["straight_motion_projection"]
        self.assertGreater(projection["travel_mm"], 300)
        self.assertGreater(projection["swept_body_bounds_mm"]["max_x_mm"], 400)
        self.assertNotIn("clear", projection)

    def test_advance_is_bounded_and_follow_executes_the_remaining_leg(self):
        episode = self.episode()
        replies = iter((choice(ADVANCE), choice(FOLLOW_WAYPOINT, ROUTE[-1])))
        def decide(view):
            episode.contexts.append(deepcopy(view))
            return (choice(COMPLETE) if view.completion_allowed else next(replies, choice(FOLLOW_WAYPOINT)))
        episode.decide = decide
        self.completed(episode)
        progress = episode.contexts[1].observation["odometry"]["x_mm"]
        self.assertGreater(progress, 0)
        self.assertLess(progress, 60)
        self.assertFalse(episode.contexts[1].completion_allowed)

    def test_scan_and_follow_can_reuse_the_retained_route(self):
        episode = self.episode()
        replies = iter((choice(FOLLOW_WAYPOINT, ROUTE[0], ROUTE[1:]),
                        choice(SCAN_FRONT_ARC), choice(FOLLOW_WAYPOINT)))
        def decide(view):
            episode.contexts.append(deepcopy(view))
            return (choice(COMPLETE) if view.completion_allowed else next(replies, choice(FOLLOW_WAYPOINT)))
        episode.decide = decide
        self.completed(episode)
        self.assertEqual(episode.contexts[1].active_waypoint_plan, ROUTE)
        self.assertEqual(episode.contexts[2].active_waypoint_plan, ROUTE)
        feedback = episode.contexts[2].observation["last_motion"]
        self.assertEqual(feedback["action"], SCAN_FRONT_ARC)
        self.assertEqual(feedback["kind"], "scan")
        self.assertTrue(feedback["completed"])
        self.assertTrue(feedback["made_progress"])
        self.assertEqual(feedback["measured_translation_mm"], 0)
        self.assertEqual(feedback["consecutive_no_progress_attempts"], 0)

    def test_diagonal_route_is_reported_before_motion_then_model_can_replace_it(self):
        episode = self.episode()
        diagonal = {"x_mm": 300, "y_mm": 300, "purpose": "invalid diagonal"}
        replies = iter((choice(FOLLOW_WAYPOINT, diagonal),
                        choice(FOLLOW_WAYPOINT, ROUTE[0], ROUTE[1:]),
                        choice(FOLLOW_WAYPOINT)))
        def decide(view):
            episode.contexts.append(deepcopy(view))
            return (choice(COMPLETE) if view.completion_allowed else next(replies, choice(FOLLOW_WAYPOINT)))
        episode.decide = decide
        self.completed(episode)
        rejected = episode.contexts[1]
        pose = rejected.observation["odometry"]
        self.assertEqual((pose["x_mm"], pose["y_mm"]), (0, 0))
        self.assertIn("NON_ORTHOGONAL_ROUTE_LEG", json.dumps(rejected.history))

    def test_mission_survives_more_than_64_decisions_until_operator_stops(self):
        episode = self.episode()
        if self.robot == "blast":
            episode.adapter.max_decisions = None
        count = 0
        def decide(view):
            nonlocal count
            count += 1
            if count == 66:
                episode.context.stop_requested.set()
            return choice(FOLLOW_WAYPOINT, {"x_mm": 300, "y_mm": 300, "purpose": "invalid diagonal"})
        episode.decide = decide
        outcome = episode.run()
        self.assertEqual(count, 66)
        self.assertEqual(outcome.terminal_reason, "stopped")
        self.assertEqual((episode.world.pose(self.robot).x_mm, episode.world.pose(self.robot).y_mm), (0, 0))

    def check_planner_recovery(self, errors):
        episode = self.episode(planner_errors=errors)
        self.completed(episode)
        waiting = episode.contexts[1:len(errors) + 2]
        self.assertEqual(len(waiting), len(errors) + 1)
        for context in waiting:
            self.assertEqual(context.active_waypoint_plan, ROUTE)
            # Fresh sensor timestamps/readings are allowed; measured motion
            # and the model-authored route must not change during the outage.
            self.assertEqual(context.observation["odometry"], waiting[0].observation["odometry"])
            self.assertEqual(context.history, waiting[0].history)
            self.assertEqual(context.previous_decision, waiting[0].previous_decision)
            self.assertEqual(context.previous_decision, choice(FOLLOW_WAYPOINT).decision.memory())
        first_error = episode.trace.index("planner_error")
        last_error = len(episode.trace) - 1 - episode.trace[::-1].index("planner_error")
        next_plan = episode.trace.index("planner", last_error + 1)
        self.assertNotIn("motion_request", episode.trace[first_error:next_plan])

    def test_model_timeout_preserves_progress_and_route(self):
        self.check_planner_recovery([LMStudioTimeoutError("Transient model timeout")])

    def test_repeated_invalid_replies_do_not_end_mission(self):
        self.check_planner_recovery([LMStudioProtocolError("Invalid reply") for _ in range(2)])

    def check_transport_recovery(self, fault):
        episode = self.episode(fault=fault)
        self.completed(episode)
        self.assertIsNotNone(episode.fault_index, "The transport fault was not exercised")
        if fault == "lost_receipt":
            self.assertGreater(episode.fault_pose.x_mm, 0)
        else:
            self.assertEqual(episode.fault_pose.x_mm, 0)
        after = episode.trace[episode.fault_index + 1:]
        # Reconcile physical state and report back to the planner BEFORE a
        # subsequent motor request. A hidden automatic pulse retry cannot pass.
        self.assertLess(after.index("observation"), after.index("planner"))
        self.assertLess(after.index("planner"), after.index("motion_request"))
        recovered = episode.contexts[1]
        self.assertEqual(recovered.active_waypoint_plan, ROUTE)
        self.assertIn(episode.fault_code, json.dumps(recovered.history),
                      "The failed action must reach the planner, not only a debug log")
        pose = recovered.observation["odometry"]
        self.assertLessEqual(abs(pose["x_mm"] - episode.fault_pose.x_mm), 5)
        self.assertLessEqual(abs(pose["y_mm"] - episode.fault_pose.y_mm), 5)

    def test_failure_before_motion_returns_to_planner_with_route(self):
        self.check_transport_recovery("before_motion")

    def test_lost_motion_receipt_reconciles_without_blind_replay(self):
        self.check_transport_recovery("lost_receipt")

    def test_operator_stop_wins_during_model_outage(self):
        episode = self.episode(
            planner_errors=[LMStudioTimeoutError("Transient model timeout")], stop_on_error=True,
        )
        outcome = episode.run()
        self.assertFalse(outcome.completed)
        self.assertEqual(outcome.terminal_reason, "stopped")
        self.assertEqual(len(episode.contexts), 2)
        after_stop = episode.trace[episode.trace.index("planner_error") + 1:]
        self.assertNotIn("motion_request", after_stop)


class BlastRecoveryContractTests(RecoveryContract, unittest.TestCase):
    robot = "blast"

    def test_recorded_partial_scan_replaces_old_translation_feedback(self):
        episode = self.episode()
        episode.world = blast_box_front().build()
        episode.adapter.controller.simulation = episode.world
        route = tuple({"x_mm": x, "y_mm": y, "purpose": "recorded Astra corner"}
                      for x, y in ((0, 300), (600, 300), (600, 0)))
        replies = iter((choice(FOLLOW_WAYPOINT, route[0], route[1:]),
                        choice(FOLLOW_WAYPOINT), choice(FOLLOW_WAYPOINT),
                        choice(FOLLOW_WAYPOINT), choice(SCAN_FRONT_ARC)))
        def decide(view):
            episode.contexts.append(deepcopy(view))
            if any(c.previous_decision and c.previous_decision["action"] == SCAN_FRONT_ARC for c in episode.contexts):
                episode.context.stop_requested.set()
                return choice(SCAN_FRONT_ARC)
            if view.active_waypoint == route[-1]:
                return choice(SCAN_FRONT_ARC)
            return choice(FOLLOW_WAYPOINT, route[0], route[1:]) if len(episode.contexts)==1 else choice(FOLLOW_WAYPOINT)
        episode.decide = decide
        self.assertEqual(episode.run().terminal_reason, "stopped")
        before, after = episode.contexts[-2:]
        self.assertEqual(before.observation["last_motion"]["action"], ADVANCE)
        feedback = after.observation["last_motion"]
        self.assertEqual(feedback["action"], SCAN_FRONT_ARC)
        self.assertEqual(feedback["kind"], "scan")
        self.assertFalse(feedback["completed"])
        self.assertEqual(feedback["reason"], "coverage_incomplete")
        self.assertEqual(feedback["measured_translation_mm"], 0)
        self.assertLess(feedback["measured_turn_deg"], -30)
        self.assertGreater(feedback["measured_turn_deg"], -90)
        self.assertGreater(feedback["sweep_coverage_deg"], 0)
        self.assertLess(feedback["sweep_coverage_deg"], 180)
        self.assertEqual(after.active_waypoint_plan, route[-1:])
        self.assertEqual(after.history[-1]["scan"]["state"], "partial")

    def test_recorded_blocked_corner_reports_alignment_not_another_full_turn(self):
        episode = self.episode()
        episode.world = blast_box_front().build()
        episode.adapter.controller.simulation = episode.world
        route = tuple({"x_mm": x, "y_mm": y, "purpose": "recorded Qwen corner"}
                      for x, y in ((0, 300), (600, 300), (600, 0)))
        replies = iter((choice(FOLLOW_WAYPOINT, route[0], route[1:]),
                        choice(FOLLOW_WAYPOINT), choice(FOLLOW_WAYPOINT),
                        choice(FOLLOW_WAYPOINT), choice(FOLLOW_WAYPOINT),
                        choice(FOLLOW_WAYPOINT)))
        def decide(view):
            episode.contexts.append(deepcopy(view))
            last = view.observation.get("last_motion") or {}
            if last.get("kind") == "turn" and last.get("made_progress") is False:
                episode.context.stop_requested.set()
                return choice(FOLLOW_WAYPOINT)
            return choice(FOLLOW_WAYPOINT, route[0], route[1:]) if len(episode.contexts)==1 else choice(FOLLOW_WAYPOINT)
        episode.decide = decide
        self.assertEqual(episode.run().terminal_reason, "stopped")
        before, after = episode.contexts[-2:]
        self.assertIn(ADVANCE, before.available_actions)
        self.assertIn("TURN_LEFT_90", before.available_actions)
        self.assertIn("TURN_RIGHT_90", before.available_actions)
        hint = before.execution["next_waypoint_motion"]
        self.assertEqual(hint["action"], "TURN_RIGHT_90")
        pose = before.observation["odometry"]
        waypoint = before.active_waypoint
        bearing = math.degrees(math.atan2(waypoint["y_mm"] - pose["y_mm"],
                                         waypoint["x_mm"] - pose["x_mm"]))
        self.assertAlmostEqual(hint["target_heading_deg"], bearing, delta=.1)
        self.assertLess(abs(hint["remaining_turn_deg"]), 30)
        motion = after.observation["last_motion"]
        self.assertFalse(motion["made_progress"])
        self.assertEqual(motion["measured_translation_mm"], 0)
        self.assertAlmostEqual(motion["requested_turn_deg"], hint["remaining_turn_deg"], delta=.1)
        # The under-travelled pulse already returns control to the model;
        # execution no longer adds another zero-progress pulse of its own.
        self.assertEqual(motion["consecutive_no_progress_attempts"], 1)
        self.assertEqual(after.active_waypoint_plan, route[-1:])

    def test_failed_scan_keeps_route_and_measured_pose_then_completes(self):
        for partial in (False, True):
            with self.subTest(partial=partial):
                episode = self.episode()
                controller = episode.adapter.controller
                command = controller.command
                injected = False
                def request(name, **kwargs):
                    nonlocal injected
                    if name == "scan_front_arc" and not injected:
                        injected = True
                        if partial:
                            command("turn_left", **kwargs)
                        raise BlastControllerError("scan_sweep_observation_unverified",
                            "Injected scan failure", motion_started=partial)
                    return command(name, **kwargs)
                controller.command = request
                def decide(view):
                    episode.contexts.append(deepcopy(view))
                    if len(episode.contexts) == 1:
                        return choice(FOLLOW_WAYPOINT, ROUTE[0], ROUTE[1:])
                    if len(episode.contexts) == 2:
                        return choice(SCAN_FRONT_ARC)
                    return choice(COMPLETE if view.completion_allowed else FOLLOW_WAYPOINT)
                episode.decide = decide
                self.completed(episode)
                recovered = episode.contexts[2]
                self.assertEqual(recovered.previous_decision, choice(SCAN_FRONT_ARC).decision.memory())
                self.assertEqual(recovered.active_waypoint_plan, ROUTE)
                self.assertEqual(recovered.history[-1]["action_error"], "scan_sweep_observation_unverified")
                self.assertEqual(recovered.observation["odometry"]["heading_deg"] != 0, partial)
                feedback = recovered.observation["last_motion"]
                self.assertEqual(feedback["action"], SCAN_FRONT_ARC)
                self.assertFalse(feedback["completed"])
                self.assertEqual(feedback["reason"], "scan_sweep_observation_unverified")
                self.assertEqual(feedback["made_progress"], partial)

    def test_waiting_for_observations_preserves_route_and_resumes(self):
        for stop in (False, True):
            with self.subTest(stop=stop):
                episode = self.episode()
                available = episode.adapter._available_actions
                waiting = []
                # Inject temporary admission loss AFTER a verified first leg.
                episode.adapter._available_actions = lambda *args: (
                    () if len(episode.contexts) == 1 and not waiting else available(*args))
                publish = episode.context.publish
                def update(value):
                    publish(value)
                    if value.get("message", "").startswith("Waiting for"):
                        waiting.append(value)
                        if stop:
                            episode.context.stop_requested.set()
                episode.context = replace(episode.context, publish=update)
                if stop:
                    outcome = episode.run()
                    self.assertEqual(outcome.terminal_reason, "stopped")
                    self.assertEqual(len(episode.contexts), 1)
                else:
                    self.completed(episode)
                    self.assertEqual(episode.contexts[1].active_waypoint_plan, ROUTE)
                self.assertEqual(len(waiting), 1)


class EV3RecoveryContractTests(RecoveryContract, unittest.TestCase):
    robot = "ev3"


if __name__ == "__main__":
    unittest.main()
