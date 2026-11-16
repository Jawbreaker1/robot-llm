import json
import subprocess
import threading
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock

from robot_agent.ev3_episode_adapter import EV3EpisodeAdapter
from robot_agent.ev3_navigation_body import EV3NavigationBody
from robot_agent.ev3_navigation_transport import EV3NavigationTimeoutError
from robot_agent.lm_studio_controller_action import COMPLETE, FOLLOW_WAYPOINT
from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge
from robot_agent.robot_control_contract import RobotEpisodeStart, RobotControlSettings, RobotRuntimeUpdate
from robot_agent.robot_control_service import RobotEpisodeContext
from robot_agent.robot_speech_runtime import RobotSpeechRuntime
from test_ev3_navigation_body import ev3_test_body
from test_ev3_navigation_session import choice


class EV3EpisodeAdapterTests(unittest.TestCase):
    def setup_episode(self, *, front=False, speech_failure=False):
        seed, profile, _ = ev3_test_body(front=front)
        seed.close()
        bodies, updates, maps, events, spoken = [], [], [], [], []
        bridge = PhysicalSpatialMapBridge(robot_id=profile.descriptor.robot_id, controller_instance_id=profile.descriptor.controller_id)
        def body_factory(**options):
            body = EV3NavigationBody(seed.transport_factory, calibration=profile.odometry_calibration, **options)
            bodies.append(body)
            return body
        def publish(update):
            RobotRuntimeUpdate.from_mapping(update)
            updates.append(update)
            maps.append(bridge.snapshot())
        def speaker(*args, **kwargs):
            if speech_failure:
                raise RuntimeError("Test playback failure")
            spoken.append((args, kwargs))
        planner = Mock()
        adapter = EV3EpisodeAdapter(body_factory=body_factory, planner_factory=lambda model: planner,
            descriptor=profile.descriptor, footprint=profile.hazard_calibration.robot_footprint,
            spatial_map_provider=bridge, speech_runtime_factory=lambda event_sink: RobotSpeechRuntime(speaker=speaker, event_sink=event_sink),
            controller_lease=threading.Lock(), event_sink=events.append)
        context = RobotEpisodeContext("ev3-live-test", RobotEpisodeStart("Reach the goal 80 cm ahead", "en", "test", 1),
                                      RobotControlSettings(max_episode_ms=60_000), threading.Event(), threading.Event(), publish)
        return adapter, context, planner, bodies, maps, spoken, events

    def test_whole_application_path_map_route_speech_and_final_heading(self):
        adapter, context, planner, bodies, maps, spoken, events = self.setup_episode()
        go = choice(FOLLOW_WAYPOINT, {"x_mm": 800, "y_mm": 0, "purpose": "Goal"})
        go = replace(go, decision=replace(go.decision, utterance="On my way."))
        planner.decide.side_effect = [go, *[choice(FOLLOW_WAYPOINT) for _ in range(14)], choice(COMPLETE)]
        outcome = adapter.run(context)
        self.assertTrue(outcome.completed)
        self.assertEqual(outcome.terminal_reason, "completed")
        self.assertTrue(spoken)
        self.assertTrue(bodies[0].transport.shutdown_complete)
        self.assertFalse(adapter.controller_lease.locked())
        self.assertTrue(any(m["navigation_trace"]["local_detour_route"] for m in maps))
        snapshot = adapter.spatial_map_provider.snapshot()
        self.assertLessEqual(snapshot["navigation_trace"]["final_goal"]["distance_to_goal_mm"], 50)
        self.assertGreater(len(snapshot["qualitative_observations"]), 13)
        self.assertTrue(all("anchor_pose" in obs for obs in snapshot["qualitative_observations"]))
        self.assertTrue(all(obs["relation"] != "NO_NEAR_REFLECTION"
            for obs in snapshot["qualitative_observations"]))
        self.assertEqual(snapshot["sensor_rays"], [])  # No invented metric IR endpoints.
        self.assertEqual(len([e for e in events if e["event"] == "survey"]), 1)
        # Exercise the real browser normalization, not just the Python payload.
        logic = Path(__file__).resolve().parents[1] / "src/robot_agent/dashboard_web/dashboard_logic.js"
        program = "const fs=require('fs'),vm=require('vm'),path=require('path'),c={}; for(const f of ['blast_map_semantics.js','dashboard_logic.js'])vm.runInNewContext(fs.readFileSync(path.join(path.dirname(process.argv[1]),f),'utf8'),c); const m=c.RobotDashboardLogic.normalizeSpatialMap(JSON.parse(fs.readFileSync(0,'utf8')),Date.now()); console.log(JSON.stringify({valid:m.contractValid,trace:!!m.navigationTrace,anchors:m.qualitativeObservations.filter(x=>x.anchorPose).length}));"
        result = subprocess.run(["node", "-e", program, str(logic)], input=json.dumps(snapshot), text=True, capture_output=True, check=True)
        normalized = json.loads(result.stdout)
        self.assertEqual(normalized, {"valid": True, "trace": True, "anchors": len(snapshot["qualitative_observations"])})

    def test_long_wait_only_expires_an_explicit_mission_time_limit(self):
        for duration in (None, 60_000):
            with self.subTest(duration=duration):
                adapter, context, planner, bodies, _, _, events = self.setup_episode()
                context = replace(context, settings=RobotControlSettings(
                    max_episode_ms=duration, speech_enabled=False))
                now = [0.0]
                adapter.clock = lambda: now[0]

                def decide(view):
                    now[0] = 7_200.0  # Two hours, without waiting in real time.
                    return choice(COMPLETE) if view.completion_allowed else choice(
                        FOLLOW_WAYPOINT, {"x_mm": 800, "y_mm": 0, "purpose": "Goal"})

                planner.decide.side_effect = decide
                result = adapter.run(context)
                self.assertEqual(result.completed, duration is None)
                self.assertEqual(result.terminal_reason, "completed" if duration is None else "time_limit")
                if duration is None:
                    self.assertLessEqual(abs(bodies[0].pose.x_mm - 800), 50)
                else:
                    self.assertFalse(any(event["event"] == "motion" for event in events))

    def test_stop_while_model_answers_cannot_start_the_chosen_motion(self):
        adapter, context, planner, bodies, _, _, events = self.setup_episode()
        def decide(_context):
            context.stop_requested.set()
            return choice(FOLLOW_WAYPOINT, {"x_mm": 800, "y_mm": 0, "purpose": "Goal"})
        planner.decide.side_effect = decide
        outcome = adapter.run(context)
        self.assertEqual(outcome.terminal_reason, "stopped")
        self.assertEqual((bodies[0].pose.x_mm, bodies[0].pose.y_mm), (0, 0))
        self.assertFalse(any(e["event"] == "motion" for e in events))
        self.assertTrue(bodies[0].transport.shutdown_complete)
        self.assertFalse(adapter.controller_lease.locked())

    def test_repeated_worker_startup_timeouts_preserve_the_mission(self):
        adapter, context, planner, bodies, _, _, events = self.setup_episode()
        factory = adapter.body_factory
        attempts = []
        def body_factory(**options):
            body = factory(**options)
            start = body.start
            def flaky_start():
                attempts.append(True)
                if len(attempts) <= 3:
                    raise EV3NavigationTimeoutError("describe", 60, "cold startup")
                return start()
            body.start = flaky_start
            return body
        adapter.body_factory = body_factory
        adapter._stop.wait = Mock(return_value=False)
        planner.decide.side_effect = [choice(FOLLOW_WAYPOINT, {"x_mm": 800, "y_mm": 0, "purpose": "Goal"}), *[choice(FOLLOW_WAYPOINT) for _ in range(14)], choice(COMPLETE)]
        self.assertTrue(adapter.run(context).completed)
        self.assertEqual(len(bodies), 1)
        self.assertEqual(len([event for event in events if event["event"] == "transport_retry"]), 3)
        self.assertEqual(len([event for event in events if event["event"] == "started"]), 1)
        self.assertLessEqual(abs(bodies[0].pose.x_mm - 800), 50)
        self.assertFalse(adapter.controller_lease.locked())

    def test_cancel_during_worker_startup_does_not_start_navigation(self):
        adapter, context, planner, bodies, _, _, events = self.setup_episode()
        factory = adapter.body_factory
        def body_factory(**options):
            body = factory(**options)
            def cancelled_start():
                context.stop_requested.set()
                raise EV3NavigationTimeoutError("describe", 60, "cancelled startup")
            body.start = cancelled_start
            return body
        adapter.body_factory = body_factory
        adapter._stop.wait = Mock(return_value=False)
        result = adapter.run(context)
        self.assertEqual(result.terminal_reason, "stopped")
        planner.decide.assert_not_called()
        self.assertFalse(any(event["event"] == "motion" for event in events))
        self.assertFalse(adapter.controller_lease.locked())

    def test_optional_speech_failure_does_not_cancel_navigation(self):
        adapter, context, planner, bodies, _, _, _ = self.setup_episode(speech_failure=True)
        go = choice(FOLLOW_WAYPOINT, {"x_mm": 800, "y_mm": 0, "purpose": "Goal"})
        planner.decide.side_effect = [replace(go, decision=replace(go.decision, utterance="On my way.")), *[choice(FOLLOW_WAYPOINT) for _ in range(14)], choice(COMPLETE)]
        self.assertTrue(adapter.run(context).completed)
        self.assertTrue(bodies[0].transport.shutdown_complete)

    def test_mission_waits_for_accessory_and_can_be_cancelled_while_waiting(self):
        for cancel in (False, True):
            with self.subTest(cancel=cancel):
                adapter, context, planner, bodies, _, _, _ = self.setup_episode()
                calls = []
                def acquire(**kwargs):
                    calls.append(kwargs)
                    if len(calls) == 1:
                        if cancel:
                            context.stop_requested.set()
                        return False
                    return True
                adapter.controller_lease = Mock(acquire=acquire)
                planner.decide.side_effect = [choice(FOLLOW_WAYPOINT, {"x_mm": 800, "y_mm": 0, "purpose": "Goal"}), *[choice(FOLLOW_WAYPOINT) for _ in range(14)], choice(COMPLETE)]
                result = adapter.run(context)
                self.assertEqual(result.completed, not cancel)
                if cancel:
                    self.assertEqual(bodies, [])
                    adapter.controller_lease.release.assert_not_called()
                else:
                    adapter.controller_lease.release.assert_called_once()


if __name__ == "__main__":
    unittest.main()
