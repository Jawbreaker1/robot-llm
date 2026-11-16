"""Obstacle beliefs stay visible without becoming fabricated sensor geometry."""
import json
from pathlib import Path
import subprocess
import unittest
from unittest.mock import Mock

from robot_agent.ev3_navigation_session import EV3NavigationSession
from robot_agent.lm_studio import LMStudioProtocolError
from robot_agent.lm_studio_controller_action import (
    ControllerActionDecision, ControllerActionPlannerResult, LMStudioControllerActionPlanner,
    FOLLOW_WAYPOINT,
)
from robot_agent.physical_navigation_contract import SCAN_FRONT_ARC
from robot_agent.navigation_obstacle_estimates import decode_obstacle_estimates
from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge
from test_ev3_navigation_body import ev3_test_body
from test_lm_studio_controller_action import Transport, completion, context, MODEL


def estimate(**edges):
    return {"label": "Forward reflection", "basis": "Strong return; lateral edges not observed.",
            "confidence_milli": 400, "bounds_mm": {
                "min_x_mm": None, "max_x_mm": None, "min_y_mm": None, "max_y_mm": None, **edges}}


class ObstacleEstimatesTests(unittest.TestCase):
    def test_model_chosen_edge_scan_after_lateral_leg_retains_route_without_advancing(self):
        body, profile, mission = ev3_test_body(ir_response=((0, 0), (200, 44), (300, 51), (1_000, 100)))
        self.addCleanup(body.close)
        body.scan()
        body.execute("TURN_LEFT_90")
        for _ in range(4):
            self.assertTrue(body.execute("ADVANCE")["completed"])
        body.execute("TURN_RIGHT_90")
        planner = Mock()
        planner.decide.return_value = ControllerActionPlannerResult(ControllerActionDecision(
            SCAN_FRONT_ARC, 700, "Inspect the obstacle-facing edge before passing.", (SCAN_FRONT_ARC,), None), 0)
        session = EV3NavigationSession(body=body, planner=planner, descriptor=profile.descriptor,
            footprint=profile.hazard_calibration.robot_footprint, mission=mission, goal="Reach goal")
        session.waypoints = ({"x_mm": 800, "y_mm": 200, "purpose": "pass"},
                             {"x_mm": 800, "y_mm": 0, "purpose": "goal"})
        route, before = session.waypoints, body.pose
        request_count, ray_count = len(body.transport.requests), len(body.ir_observations)
        result = session.step()
        self.assertEqual(result["action"], SCAN_FRONT_ARC)
        self.assertEqual((body.pose.x_mm, body.pose.y_mm), (before.x_mm, before.y_mm))
        self.assertLess(abs(body.pose.heading_mdeg - before.heading_mdeg), 1_000)
        self.assertEqual(session.waypoints, route)
        self.assertGreater(len(body.ir_observations), ray_count)
        self.assertFalse(any(op == "pulse" and args["action"] == "ADVANCE"
            for op, args in body.transport.requests[request_count:]))
        rays = {sample["requested_bearing_deg"]: sample["raw_ir_proximity"] for sample in body.survey["samples"]}
        self.assertLess(rays[-30], rays[0])  # Obstacle side differs from the clear centre.

    def test_actual_motion_feedback_remains_without_a_second_obstacle_interpretation(self):
        session = self.session()
        session.body.scan()
        belief = estimate(min_x_mm=100, min_y_mm=-50, max_y_mm=50)
        belief["confidence_milli"] = 1000
        session.obstacle_estimates = (belief,)
        session.history = [{"action": FOLLOW_WAYPOINT, "result": "simulation_body_blocked",
                            "pose": {"x_mm": 88, "y_mm": 220, "heading_mdeg": -1471},
                            "last_motion": {"action": "ADVANCE", "reason": "simulation_body_blocked",
                                            "made_progress": True, "completed": False}}]
        context = session.context()
        self.assertEqual(context.history[0]["result"], "simulation_body_blocked")
        self.assertEqual(context.history[0]["pose"]["heading_deg"], -1.5)
        # Existing feedback preserves the last actual motion across stationary
        # observations, even when the compact history no longer includes it.
        session.history += [{"action": "OBSERVE", "result": "observation_refreshed"}] * 20
        context = session.context()
        self.assertEqual(context.observation["last_motion"]["reason"], "simulation_body_blocked")
        self.assertFalse(context.observation["last_motion"]["completed"])
        self.assertTrue(context.observation["last_motion"]["made_progress"])
        self.assertEqual(context.obstacle_estimates, (belief,))
        self.assertNotIn("obstacle_review", context.local_map_evidence)
        bridge = PhysicalSpatialMapBridge(robot_id="ev3", controller_instance_id="ev3-controller")
        bridge.offer_navigation(body=session.body, mission=session.mission, waypoints=(),
            footprint=session.footprint, obstacle_estimates=session.obstacle_estimates)
        snapshot = bridge.snapshot()
        self.assertNotIn("obstacle_review", snapshot["navigation_trace"])
        self.assertEqual(snapshot["navigation_trace"]["obstacle_estimates"], [belief])
        self.assertEqual(snapshot["object_hypotheses"], [])
        self.assertEqual(snapshot["cells"], [])

    def test_map_renders_dimensions_without_inventing_unknown_edges_or_sensor_hits(self):
        session = self.session()
        session.body.scan()
        beliefs = [estimate(min_x_mm=200, max_x_mm=400, min_y_mm=-100, max_y_mm=100), estimate()]
        bridge = PhysicalSpatialMapBridge(robot_id="ev3", controller_instance_id="ev3-controller")
        bridge.offer_navigation(body=session.body, mission=session.mission, waypoints=(),
            footprint=session.footprint, obstacle_estimates=beliefs)
        script = r'''
const fs = require("fs"), vm = require("vm");
const ctx = {};
for (const file of process.argv.slice(1)) vm.runInNewContext(fs.readFileSync(file, "utf8"), ctx);
class Node {
  constructor() { this.children = []; this.attributes = {}; this._text = ""; }
  set textContent(value) { this._text = String(value); this.children = []; }
  get textContent() { return this._text + this.children.map(c => c.textContent).join(""); }
  appendChild(n) { this.children.push(n); return n; }
  replaceChildren(...nodes) { this.children = nodes; this._text = ""; }
  setAttribute(k,v) { this.attributes[k] = String(v); }
}
const nodes = {};
const document = { getElementById(id) { return nodes[id] ||= new Node(); },
  createElement() { return new Node(); }, createElementNS() { return new Node(); } };
const presenter = ctx.RobotSpatialMapPresenter.create({ document,
  normalizeSpatialMap: ctx.RobotDashboardLogic.normalizeSpatialMap,
  translate: (k,v) => k + (v ? JSON.stringify(v) : ""), formatNumber: String });
const map = JSON.parse(fs.readFileSync(0, "utf8"));
presenter.render(map, "connected", map.captured_at_unix_ms);
const flatten = node => [node, ...node.children.flatMap(flatten)];
const first = flatten(nodes["map-local-odometry-layer"]);
const text = nodes["map-object-list"].textContent;
const shapes = first.filter(n => n.attributes.class === "map-model-obstacle");
map.navigation_trace.obstacle_estimates = [{label: "bad", basis: "bad", bounds_mm: {min_x_mm: 0}}];
presenter.render(map, "connected", map.captured_at_unix_ms);
process.stdout.write(JSON.stringify({ shapes: shapes.length,
  attributes: shapes[0]?.attributes, text,
  afterInvalid: flatten(nodes["map-local-odometry-layer"]).filter(n => n.attributes.class === "map-model-obstacle").length,
  stillDrawable: nodes["map-local-odometry-layer"].children.length > 0,
  sensorRays: nodes["map-ray-layer"].children.length }));
'''
        web = Path(__file__).resolve().parents[1] / "src/robot_agent/dashboard_web"
        response = subprocess.run(["node", "-e", script, *[str(web / name) for name in
            ("blast_map_semantics.js", "dashboard_logic.js", "spatial_map_presenter.js")]],
            input=json.dumps(bridge.snapshot()), capture_output=True, text=True, check=True)
        result = json.loads(response.stdout)
        self.assertEqual(result["shapes"], 1)
        self.assertEqual(result["attributes"]["data-measured"], "false")
        self.assertIn("≈200 mm", result["text"])
        self.assertIn("map.model_obstacle.unknown", result["text"])
        self.assertIn("Strong return; lateral edges not observed.", result["text"])
        self.assertEqual(result["afterInvalid"], 0)
        self.assertTrue(result["stillDrawable"])
        self.assertEqual(result["sensorRays"], 0)

    def test_unknown_edges_remain_unknown_and_are_detached(self):
        raw = [estimate(min_x_mm=200)]
        value = decode_obstacle_estimates(raw)
        raw[0]["bounds_mm"]["min_x_mm"] = 900
        self.assertEqual(value[0]["bounds_mm"]["min_x_mm"], 200)
        self.assertIsNone(value[0]["bounds_mm"]["max_y_mm"])
        self.assertEqual(decode_obstacle_estimates([]), ())
        with self.assertRaises(ValueError):
            decode_obstacle_estimates(None)

    def test_invalid_or_reversed_edges_are_rejected(self):
        for invalid in (estimate(min_x_mm=True), estimate(max_x_mm="300"),
                        estimate(min_y_mm=20, max_y_mm=10), estimate(max_y_mm=100001)):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                decode_obstacle_estimates([invalid])

    def test_shared_planner_round_trips_only_when_context_requests_beliefs(self):
        raw = [estimate()]
        response = {"action": SCAN_FRONT_ARC, "plan": [SCAN_FRONT_ARC],
                    "confidence_milli": 400, "assessment": "Width remains unknown.",
                    "utterance": None, "obstacle_estimates": raw}
        transport = Transport(completion(response))
        planner = LMStudioControllerActionPlanner(model=MODEL, transport=transport)
        ctx = context(available_actions=(SCAN_FRONT_ARC,), obstacle_estimates=(), local_map_evidence={
            "ir_survey": {"samples": []}})
        result = planner.decide(ctx)
        self.assertEqual(result.decision.obstacle_estimates, tuple(raw))
        payload = json.loads(transport.calls[0][1])
        supplied = json.loads(payload["messages"][1]["content"])
        self.assertEqual(supplied["obstacle_estimates"], [])
        self.assertNotIn("obstacle_estimates", supplied["local_map_evidence"])
        self.assertIn("NOT that the surroundings have been verified empty", payload["messages"][0]["content"])
        self.assertIn("obstacle_estimates", payload["response_format"]["json_schema"]["schema"]["required"])
        self.assertIn("null for edges not established", payload["messages"][0]["content"])
        # The extra output is not silently accepted for an unrelated consumer.
        with self.assertRaises(LMStudioProtocolError):
            planner.decide(context(available_actions=(SCAN_FRONT_ARC,)))

    def session(self):
        body, profile, mission = ev3_test_body(front=False)
        self.addCleanup(body.close)
        session = EV3NavigationSession(body=body, planner=Mock(), descriptor=profile.descriptor,
            footprint=profile.hazard_calibration.robot_footprint, mission=mission,
            goal="Drive 800 mm ahead.")
        return session

    def test_alignment_returns_new_forward_evidence_before_any_translation(self):
        session = self.session()
        target = {"x_mm": 0, "y_mm": 300, "purpose": "explore left"}
        decision = ControllerActionDecision(FOLLOW_WAYPOINT, 400, "Inspect left", (FOLLOW_WAYPOINT,), None, target)
        session.planner.decide.return_value = ControllerActionPlannerResult(decision, 0)
        result = session.step()
        self.assertEqual(result["result"], "heading_aligned")
        self.assertLess(abs(session.body.pose.y_mm), 5)
        self.assertEqual(session.waypoints, (target,))
        self.assertAlmostEqual(session.context().observation["infrared"]["ray_heading_deg_in_episode"], 90, delta=5)
        session.step()
        # Same target: one explicitly requested pulse, no automatic rescan.
        self.assertGreater(session.body.pose.y_mm, 40)
        self.assertLess(session.body.pose.y_mm, 60)
        self.assertEqual(session.planner.decide.call_count, 2)
        self.assertEqual(sum(op == "scan_sample" for op, _ in session.body.transport.requests), 13)

    def test_one_belief_list_is_retained_for_context_and_map_not_collision_geometry(self):
        session = self.session()
        belief = estimate(min_x_mm=200, max_x_mm=400, min_y_mm=-100, max_y_mm=100)
        def result(beliefs):
            return ControllerActionPlannerResult(ControllerActionDecision(
                SCAN_FRONT_ARC, 400, "Inspect", (SCAN_FRONT_ARC,), None, obstacle_estimates=beliefs), 0)
        session.planner.decide.side_effect = [result((belief,)), result(None), result(())]
        session.step()
        session.step()
        evidence = session.context().local_map_evidence
        self.assertNotIn("obstacle_estimates", evidence)
        self.assertEqual(session.context().obstacle_estimates, (belief,))
        self.assertNotIn("ir_reference_observations", evidence)
        bridge = PhysicalSpatialMapBridge(robot_id="ev3", controller_instance_id="ev3-controller")
        bridge.offer_navigation(body=session.body, mission=session.mission, waypoints=session.waypoints,
            footprint=session.footprint, obstacle_estimates=session.obstacle_estimates)
        snapshot = bridge.snapshot()
        self.assertEqual(snapshot["navigation_trace"]["obstacle_estimates"], [belief])
        for key in ("cells", "sensor_rays", "object_hypotheses"):
            self.assertEqual(snapshot[key], [])
        session.step()
        self.assertEqual(session.obstacle_estimates, ())
        # Previously published frames do not mutate on the next decision.
        self.assertEqual(snapshot["navigation_trace"]["obstacle_estimates"], [belief])
