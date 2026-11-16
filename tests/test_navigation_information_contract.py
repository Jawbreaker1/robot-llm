"""Current map producers through status, shared coordinates and GUI consumers.

No hardware, model or speech calls: the fixtures are measured state and model
beliefs fed to the same publishers used by the running robot adapters.
"""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace
from unittest import TestCase

from robot_agent.blast_episode_map_trace import _BlastEpisodeMapTrace
from robot_agent.blast_spatial_map import BlastSpatialMapBridge
from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
from robot_agent.physical_navigation_mission import DirectionalMission
from robot_agent.physical_odometry import PhysicalPose
from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge
from robot_agent.robot_status_facts import project_robot_status_facts
from robot_agent.shared_spatial_map import SharedSpatialMapCompositor
from test_blast_spatial_map import observation
from test_shared_spatial_map import frame_transform, WORLD_FRAME_ID, WORLD_GENERATION_ID

WEB = Path(__file__).resolve().parents[1] / "src/robot_agent/dashboard_web"
WAYPOINTS = ({"x_mm": 0, "y_mm": 300, "purpose": "Investigate left edge"},)
BELIEFS = [{"label": "Possible front obstacle", "bounds_mm": {
    "min_x_mm": 200, "max_x_mm": None, "min_y_mm": None, "max_y_mm": None,
}, "confidence_milli": 400, "basis": "Forward reflection; far and side edges unknown."},
{"label": "Estimated box", "bounds_mm": {
    "min_x_mm": 200, "max_x_mm": 400, "min_y_mm": -100, "max_y_mm": 100,
}, "confidence_milli": 600, "basis": "Model estimate, not measured IR distance."}]


def ev3_map():
    profile = EV3RSTORMProfile()
    bridge = PhysicalSpatialMapBridge(robot_id=profile.descriptor.robot_id,
        controller_instance_id=profile.descriptor.controller_id, clock_ms=lambda: 1000)
    body = SimpleNamespace(pose=PhysicalPose(), ir_observations=[{
        "pose": PhysicalPose().to_dict(), "raw_ir_proximity": 50,
        "near_obstacle": False, "observed_at_unix_ms": 900,
    }])
    mission = DirectionalMission.begin(episode_id="contract-ev3", pose=body.pose,
        minimum_forward_progress_mm=800)
    bridge.offer_navigation(body=body, mission=mission, waypoints=WAYPOINTS,
        footprint=profile.hazard_calibration.robot_footprint, obstacle_estimates=BELIEFS)
    return bridge


def blast_map():
    bridge = BlastSpatialMapBridge(unix_clock_ms=lambda: 1000, monotonic_clock_ms=lambda: 1000)
    trace = _BlastEpisodeMapTrace(bridge=bridge, episode_id="contract-blast",
        pose=PhysicalPose(), observation=observation(), observed_at_unix_ms=1000,
        episode_start_heading=0.0, minimum_forward_progress_mm=800)
    assert trace.set_advisory_waypoint_plan(WAYPOINTS, pose=PhysicalPose(),
        observation=observation(), observed_at_unix_ms=1000)
    return bridge


def shared_map(bridge, *, yaw_mdeg=90000):
    value = bridge.snapshot()
    transform = frame_transform(value["robot_id"], value["controller_instance_id"],
        value["frame_id"], value["local_generation_id"], tx_mm=1000, ty_mm=2000,
        yaw_mdeg=yaw_mdeg)
    return SharedSpatialMapCompositor(world_frame_id=WORLD_FRAME_ID,
        world_generation_id=WORLD_GENERATION_ID, bindings=((bridge, transform),)).snapshot()


class NavigationInformationContractTests(TestCase):
    def test_blast_keeps_ray_origins_no_returns_and_unsettled_hits_after_motion(self):
        from robot_agent.blast_scan_planar_projection import project_blast_scan_planar_surfaces
        from test_blast_scan_planar_projection import scan_result
        scan = scan_result((300, 2000, 400, 2000, 500))
        pose = PhysicalPose()
        trace = _BlastEpisodeMapTrace(bridge=None, episode_id="ray-memory", pose=pose,
            observation=observation(), observed_at_unix_ms=1000, episode_start_heading=0,
            minimum_forward_progress_mm=800)
        trace.record(pose=pose, observation=observation(), pose_observed=False, scan_view={
            "scan": scan, "scan_pose": pose.to_dict(),
            "planar_projection": project_blast_scan_planar_surfaces(scan=scan, scan_pose=pose),
        })
        before = trace.planner_local_map_evidence(pose)
        retained_echoes = trace._coarse_navigation_observations()[0]
        rays = before["range_observations"]
        self.assertEqual(len(rays), 5)
        self.assertEqual(rays[0]["observed_from"], {"x_mm": 110, "y_mm": 80})
        self.assertEqual(rays[0]["distance_mm"], 300)
        self.assertIsNone(rays[1]["distance_mm"])
        self.assertEqual(rays[1]["range_state"], "NO_VALID_DISTANCE")
        self.assertEqual(rays[-1]["distance_mm"], 500)
        self.assertTrue(rays[-1]["settled"])
        moved = PhysicalPose(x_mm=100, y_mm=300, heading_mdeg=90_000)
        trace.record(pose=moved, observation={**observation(), "distance_mm": 250},
            pose_observed=True, scan_view=None, observation_settled=False)
        after = trace.planner_local_map_evidence(moved)
        self.assertEqual(after["range_observations"][:-1], rays)
        latest = after["range_observations"][-1]
        self.assertEqual(latest["observed_from"], {"x_mm": 20, "y_mm": 410})
        self.assertEqual(latest["ray_heading_deg_in_episode"], 90)
        self.assertFalse(latest["settled"])
        self.assertEqual(latest["distance_mm"], 250)
        self.assertEqual(after["sensor_layout"]["left_offset_mm"], 80)
        # Model-input storage is bounded and independent of obstacle retention.
        for i in range(140):
            trace.record(pose=moved, observation={**observation(), "distance_mm": 300 + i},
                         pose_observed=True, scan_view=None)
        self.assertEqual(len(trace.range_observations), 128)
        self.assertEqual(trace._coarse_navigation_observations()[0], retained_echoes)

    def test_current_ev3_beliefs_and_incomplete_scan_reach_conversation(self):
        source = ev3_map().snapshot()
        facts = project_robot_status_facts({"runtime": {"scan": {
            "complete": False, "sweep_degrees": 120,
        }}}, source, captured_at_unix_ms=1100)
        self.assertEqual(facts["control"]["runtime"]["scan"],
                         {"complete": False, "sweep_degrees": 120})
        trace = facts["spatial_map"]["navigation_trace"]
        self.assertEqual(trace["frame_id"], source["frame_id"])
        self.assertEqual(trace["obstacle_estimates"], BELIEFS)
        self.assertEqual(trace["advisory_waypoint"]["purpose"], WAYPOINTS[0]["purpose"])
        self.assertEqual(facts["spatial_map"]["object_hypotheses"], [])
        trace["obstacle_estimates"][0]["bounds_mm"]["max_x_mm"] = 900
        self.assertIsNone(source["navigation_trace"]["obstacle_estimates"][0]["bounds_mm"]["max_x_mm"])

    def test_both_current_producers_pass_shared_map_without_discarding_state(self):
        for factory in (ev3_map, blast_map):
            with self.subTest(robot=factory.__name__):
                bridge = factory()
                before = bridge.snapshot()
                robot = shared_map(bridge)["robots"][0]
                self.assertEqual(robot["status"], "available", robot.get("reason_code"))
                trace = robot["navigation_trace"]
                self.assertEqual(trace["advisory_waypoint"]["x_mm"], 700)
                self.assertEqual(trace["advisory_waypoint"]["y_mm"], 2000)
                self.assertEqual(trace["local_obstacle_estimates"],
                                 before["navigation_trace"].get("obstacle_estimates", []))
                self.assertEqual(trace["local_frame_id"], before["frame_id"])
                self.assertEqual(bridge.snapshot(), before)

    def test_actual_local_and_shared_outputs_reach_gui_with_honest_geometry(self):
        script = r'''
const fs = require("fs");
require(process.argv[1] + "/blast_map_semantics.js");
require(process.argv[1] + "/dashboard_logic.js");
require(process.argv[1] + "/spatial_map_presenter.js");
class Node {
  constructor(tag) {this.tag = tag; this.children = []; this.attrs = {}; this._text = "";}
  set textContent(v) {this._text = String(v); this.children = [];}
  get textContent() {return this._text + this.children.map(n => n.textContent).join("");}
  appendChild(n) {this.children.push(n); return n;}
  replaceChildren(...nodes) {this.children = nodes; this._text = "";}
  setAttribute(k,v) {this.attrs[k] = v;}
}
const inputs = JSON.parse(fs.readFileSync(0, "utf8"));
const result = inputs.map(input => {
  const nodes = {};
  const document = {getElementById: id => nodes[id] ||= new Node("div"),
    createElement: tag => new Node(tag), createElementNS: (_,tag) => new Node(tag)};
  const presenter = RobotSpatialMapPresenter.create({document,
    normalizeSpatialMap: RobotDashboardLogic.normalizeSpatialMap,
    translate: key => key, formatNumber: String});
  presenter.render(input, "connected", 1000);
  const map = RobotDashboardLogic.normalizeSpatialMap(input, 1000);
  const trace = map.navigationTrace || map.robots[0].navigationTrace;
  const points = [];
  RobotBlastMapSemantics.appendEstimatePoints(trace.obstacleEstimates,
    (x,y) => points.push([Math.round(x),Math.round(y)]), trace.obstacleEstimateTransform);
  const layer = {children: [], appendChild(x) {this.children.push(x);}};
  const ui = {svg: (tag, attrs) => ({tag, attrs, children: [],
    appendChild(x) {this.children.push(x);}}), title: () => {},
    t: key => key, format: String};
  RobotBlastMapSemantics.renderObstacleEstimates(layer, trace.obstacleEstimates,
    {point: (x,y) => ({x,y})}, ui, trace.obstacleEstimateTransform);
  return {status: map.status, waypoint: trace.advisoryWaypoint,
    estimates: trace.obstacleEstimates, points, shapes: layer.children,
    objectCount: nodes["map-object-count"].textContent,
    objectText: nodes["map-object-list"].textContent};
});
process.stdout.write(JSON.stringify(result));
'''
        for factory in (ev3_map, blast_map):
            with self.subTest(robot=factory.__name__):
                bridge = factory()
                output = subprocess.run(["node", "-e", script, str(WEB)],
                    input=json.dumps([bridge.snapshot(), shared_map(bridge), shared_map(bridge, yaw_mdeg=45000)]),
                    text=True, capture_output=True)
                self.assertEqual(output.returncode, 0, output.stderr)
                local, shared, oblique = json.loads(output.stdout)
                for result in (local, shared, oblique):
                    self.assertEqual(result["waypoint"]["purpose"], WAYPOINTS[0]["purpose"])
                    self.assertEqual(result["waypoint"]["source"], "MODEL")
                self.assertEqual(shared["estimates"], local["estimates"])
                self.assertEqual(oblique["estimates"], local["estimates"])
                if factory is ev3_map:
                    self.assertEqual(shared["objectCount"], "2")
                    self.assertIn(bridge.robot_id, shared["objectText"])
                    self.assertIn(bridge.snapshot()["frame_id"], shared["objectText"])
                    self.assertEqual(len(local["estimates"]), 2)
                    self.assertEqual(shared["estimates"][0]["confidenceMilli"], 400)
                    self.assertIsNone(shared["estimates"][0]["bounds"])
                    self.assertEqual(sorted(shared["points"]),
                        [[900,2200], [900,2400], [1100,2200], [1100,2400]])
                    self.assertEqual(sorted(oblique["points"]),
                        [[1071,2212], [1212,2071], [1212,2354], [1354,2212]])
                    self.assertEqual(len(shared["shapes"]), 1)  # unknown stays unknown
                    self.assertEqual(shared["shapes"][0]["attrs"]["data-measured"], "false")
                    self.assertEqual(shared["shapes"][0]["children"][0]["tag"], "polygon")

    def test_invalid_beliefs_do_not_bypass_shared_validation(self):
        bridge = ev3_map()
        invalid = deepcopy(bridge.snapshot())
        invalid["navigation_trace"]["obstacle_estimates"][0]["bounds_mm"]["max_x_mm"] = -1
        bridge.snapshot = lambda: invalid
        robot = shared_map(bridge)["robots"][0]
        self.assertEqual(robot["status"], "unavailable")
        self.assertEqual(robot["reason_code"], "source_navigation_trace_invalid")
