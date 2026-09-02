import json
from pathlib import Path
import subprocess
import unittest


WEB_ROOT = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "robot_agent"
    / "dashboard_web"
)


class SpatialMapPresenterRuntimeTests(unittest.TestCase):
    def test_blast_full_route_and_provisional_obstacle_render_separately(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
class FakeNode {
  constructor(tag, id = null) {
    this.tag = tag; this.id = id; this.className = ""; this.hidden = false;
    this.attributes = {}; this.children = []; this._text = "";
  }
  set textContent(value) { this._text = String(value); this.children = []; }
  get textContent() {
    return this._text + this.children.map((child) => child.textContent).join("");
  }
  appendChild(child) { this.children.push(child); return child; }
  replaceChildren(...children) { this._text = ""; this.children = children; }
  setAttribute(name, value) { this.attributes[name] = String(value); }
}
const ids = [
  "map-connection-status", "map-frame-label", "map-empty-state",
  "map-empty-title", "map-empty-body", "map-metadata",
  "map-qualitative-list", "map-qualitative-count", "map-object-list",
  "map-object-count", "map-local-odometry-layer", "map-cell-layer",
  "map-path-layer", "map-ray-layer", "map-object-layer", "map-robot-layer",
];
const nodes = Object.fromEntries(ids.map((id) => [id, new FakeNode("div", id)]));
const document = {
  getElementById(id) { return nodes[id]; },
  createElement(tag) { return new FakeNode(tag); },
  createElementNS(_namespace, tag) { return new FakeNode(tag); },
};
const context = {};
for (const filename of process.argv.slice(1)) {
  vm.runInNewContext(fs.readFileSync(filename, "utf8"), context, { filename });
}
const translations = {
  "common.missing": "—",
  "map.objects.ultrasonic_obstacle_cluster": "PROVISIONAL ULTRASONIC OBSTACLE",
  "map.objects.ultrasonic_obstacle_short": "PROVISIONAL OBSTACLE",
  "map.objects.provisional_inference": "PROVISIONAL INFERENCE",
  "mission.route.waypoint.LATERAL_CLEARANCE": "LATERAL CLEARANCE",
  "mission.route.waypoint.REACQUIRE_GOAL_HEADING": "REACQUIRE GOAL HEADING",
  "mission.route.waypoint.PASS_BEYOND_TARGET": "PASS BEYOND TARGET",
};
const translate = (key, args = {}) => {
  if (key === "map.objects.ultrasonic_support") return `${args.count}/${args.radius}`;
  if (key === "map.objects.source_scans") return args.scans;
  if (key === "map.navigation_trace.route_title") return `ROUTE ${args.side}`;
  if (key === "map.navigation_trace.route_waypoint_pose") return `${args.x}/${args.y}/${args.heading}`;
  return translations[key] || key;
};
const presenter = context.RobotSpatialMapPresenter.create({
  document,
  normalizeSpatialMap: context.RobotDashboardLogic.normalizeSpatialMap,
  translate,
  formatNumber: (value) => String(value),
});
const waypoints = [
  { ordinal: 0, kind: "LATERAL_CLEARANCE", x_mm: 0, y_mm: -225,
    heading_mdeg: -90000, fact_key: null, status: "COMPLETED" },
  { ordinal: 1, kind: "REACQUIRE_GOAL_HEADING", x_mm: 0, y_mm: -225,
    heading_mdeg: 0, fact_key: "GOAL_HEADING_ALIGNED", status: "ACTIVE" },
  { ordinal: 2, kind: "PASS_BEYOND_TARGET", x_mm: 500, y_mm: -225,
    heading_mdeg: 0, fact_key: "TARGET_BEHIND", status: "UPCOMING" },
];
const route = {
  schema: "robot-local-detour-route/v1", read_only: true, provisional: true,
  route_id: "route-a", version: 2, status: "ACTIVE",
  detour_side: "RIGHT_OF_GOAL", active_index: 1, waypoints,
};
const point = {
  side: "center", measured_range_mm: 100, relative_bearing_mdeg: 0,
  sensor_origin_x_mm: 0, sensor_origin_y_mm: 0, beam_heading_mdeg: 0,
  nominal_echo_x_mm: 100, nominal_echo_y_mm: 0,
};
const obstacle = {
  hypothesis_id: "blast-ultrasonic-a",
  classification: "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER",
  label: "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER",
  x_mm: 100, y_mm: 0,
  geometry_kind: "PROVISIONAL_ULTRASONIC_ECHO_CLUSTER",
  support_radius_mm: 35,
  support_points: [{ side: "center", x_mm: 100, y_mm: 0,
    measured_range_mm: 100, relative_bearing_mdeg: 0 }],
  source_scan_ids: ["dense-scan"], bearing: "FRONT",
  relation: "FRONT_OF_SCAN", evidence_count: 1, confidence_milli: 200,
  source_id: "blast-settled-measured-planar-projection",
  provenance: "SETTLED_MEASURED_ULTRASONIC + PROVISIONAL_YAW_ONLY",
  quality: "PROVISIONAL_YAW_ONLY", settled_measured_only: true,
  provisional: true, read_only: true, observed_at_unix_ms: 1900, age_ms: 0,
};
const map = {
  schema: "robot-spatial-map/v1", read_only: true,
  status: "qualitative_only", frame_id: "blast-frame",
  frame_kind: "LOCAL_ODOMETRY", bounds: null,
  robot_pose: { x_mm: 0, y_mm: -225, heading_mdeg: 0 },
  pose_history: [{ x_mm: 0, y_mm: 0, heading_mdeg: 0 }],
  cells: [], sensor_rays: [], qualitative_observations: [],
  scan_evidence_history: [], object_hypotheses: [obstacle],
  navigation_trace: {
    schema: "robot-navigation-trace/v1", read_only: true,
    frame_id: "blast-frame",
    provenance: "PROVISIONAL_ENCODER_ODOMETRY + PROVISIONAL_YAW_ONLY",
    final_goal: { kind: "DIRECTIONAL_HEADING", navigation_enforced: true,
      origin_x_mm: 0, origin_y_mm: 0, target_x_mm: 420, target_y_mm: 0,
      desired_heading_mdeg: 0, minimum_forward_progress_mm: 420,
      heading_tolerance_mdeg: 5000, current_forward_progress_mm: 45,
      current_lateral_offset_mm: -225,
      goal_radius_mm: 120, distance_to_goal_mm: 437,
      remaining_forward_progress_mm: 375 },
    planned_leg: { kind: "REACQUIRE_GOAL_HEADING",
      scope: "LOCAL_DETOUR_ROUTE", clearance_proven: false,
      passage_proven: false, route_eligible: true, selected_side: "RIGHT",
      bind_pose: { x_mm: 0, y_mm: -225, heading_mdeg: -90000 },
      waypoint: { x_mm: 0, y_mm: -225, heading_mdeg: 0 } },
    coarse_grid: {
      schema: "robot-coarse-navigation-grid/v1",
      frame: "EPISODE_START", cell_size_mm: 150,
      top_is: "START_FORWARD", left_is: "START_LEFT",
      rows: [
        "...........", "...........", "...........", "...........",
        ".....G.....", "...........", "......?....", ".....B.....",
        "...........", "...........", "...........",
      ],
      robots: [{ symbol: "B", robot_id: "blast-01", row: 7,
        column: 5, heading: "UP" }],
      legend: ".=UNKNOWN o=OBSERVED_CLEAR_RAY #=ROBOT_KEEP_OUT ?=POSSIBLE_OBSTACLE G=GOAL W=WAYPOINT X=GOAL_AND_WAYPOINT x=WAYPOINT_ON_BLOCKED B=BLAST E=EV3 2=BOTH_ROBOTS",
      cropped: false,
    },
    imu_heading: null, local_detour_route: route,
    planar_scan_views: [{ scan_id: "dense-scan", observed_at_unix_ms: 1900,
      scan_pose: { x_mm: 0, y_mm: 0, heading_mdeg: 0 },
      projection: { schema: "blast-planar-scan-projection/v1",
        frame: "EPISODE_LOCAL_ODOMETRY", quality: "PROVISIONAL_YAW_ONLY",
        vertical_pitch_compensated: false,
        ultrasonic_beam_width_modeled: false,
        scan_turn_translation_compensated: false, points: [point] } },
      // NO_VALID_DISTANCE is intentionally absent from validated points.
      { scan_id: "nvd-only-scan", observed_at_unix_ms: 1951,
        scan_pose: { x_mm: 0, y_mm: 0, heading_mdeg: 0 },
        projection: { schema: "blast-planar-scan-projection/v1",
          frame: "EPISODE_LOCAL_ODOMETRY", quality: "PROVISIONAL_YAW_ONLY",
          vertical_pitch_compensated: false,
          ultrasonic_beam_width_modeled: false,
          scan_turn_translation_compensated: false, points: [] } }],
  },
};
presenter.render(map, "connected", 2000);
function descendants(root) {
  return [root, ...root.children.flatMap(descendants)];
}
const rendered = descendants(nodes["map-local-odometry-layer"]);
const withClass = (fragment) => rendered.filter((node) => (
  String(node.attributes.class || "").includes(fragment)
));
const exactClass = (name) => rendered.filter((node) => (
  String(node.attributes.class || "") === name
));
const obstacleItem = nodes["map-object-list"].children[0];
process.stdout.write(JSON.stringify({
  route: withClass("map-local-detour-route")[0].attributes,
  waypointStatuses: withClass("map-local-detour-waypoint ").map((node) => (
    node.attributes["data-status"]
  )),
  waypointLabels: withClass("map-local-detour-waypoint-label").map((node) => (
    node.textContent
  )),
  obstacle: withClass("map-provisional-ultrasonic-obstacle")[0].attributes,
  obstacleCount: exactClass("map-provisional-ultrasonic-obstacle").length,
  coarseObstacleCount: withClass("map-coarse-obstacle-cell").length,
  scanViewCount: exactClass("map-blast-scan-view").length,
  rawRayCount: exactClass("map-blast-scan-ray").length,
  obstacleItemText: obstacleItem.textContent,
  metadataText: nodes["map-metadata"].textContent,
}));
"""
        completed = subprocess.run(
            [
                "node", "--input-type=commonjs", "-e", script,
                str(WEB_ROOT / "blast_map_semantics.js"),
                str(WEB_ROOT / "dashboard_logic.js"),
                str(WEB_ROOT / "spatial_map_presenter.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        result = json.loads(completed.stdout)
        self.assertEqual(result["route"]["data-route-id"], "route-a")
        self.assertEqual(
            result["waypointStatuses"],
            ["COMPLETED", "ACTIVE", "UPCOMING"],
        )
        self.assertEqual(len(result["waypointLabels"]), 3)
        self.assertIn("PASS BEYOND TARGET", result["waypointLabels"][2])
        self.assertEqual(
            result["obstacle"]["data-classification"],
            "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER",
        )
        self.assertEqual(
            result["obstacle"]["data-settled-measured-only"],
            "true",
        )
        self.assertEqual(result["rawRayCount"], 1)
        self.assertEqual(result["scanViewCount"], 2)
        self.assertEqual(result["obstacleCount"], 1)
        self.assertEqual(result["coarseObstacleCount"], 1)
        self.assertIn("PROVISIONAL INFERENCE", result["obstacleItemText"])
        self.assertNotIn("nvd-only-scan", result["obstacleItemText"])
        self.assertIn(".....G.....", result["metadataText"])
        self.assertIn(".....B.....", result["metadataText"])
        self.assertIn("B blast-01 UP", result["metadataText"])

    def test_asymmetric_footprint_and_pose_based_scan_cues_are_honest(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");

class FakeNode {
  constructor(tag, id = null) {
    this.tag = tag;
    this.id = id;
    this.className = "";
    this.hidden = false;
    this.attributes = {};
    this.children = [];
    this._text = "";
  }
  set textContent(value) {
    this._text = String(value);
    this.children = [];
  }
  get textContent() {
    return this._text + this.children
      .map((child) => child.textContent)
      .join("");
  }
  appendChild(child) {
    this.children.push(child);
    return child;
  }
  replaceChildren(...children) {
    this._text = "";
    this.children = children;
  }
  setAttribute(name, value) {
    this.attributes[name] = String(value);
  }
}

const requiredIds = [
  "map-connection-status",
  "map-frame-label",
  "map-empty-state",
  "map-empty-title",
  "map-empty-body",
  "map-metadata",
  "map-qualitative-list",
  "map-qualitative-count",
  "map-object-list",
  "map-object-count",
  "map-local-odometry-layer",
  "map-cell-layer",
  "map-path-layer",
  "map-ray-layer",
  "map-object-layer",
  "map-robot-layer",
];
const nodes = Object.fromEntries(
  requiredIds.map((id) => [id, new FakeNode("div", id)]),
);
const document = {
  getElementById(id) { return nodes[id]; },
  createElement(tag) { return new FakeNode(tag); },
  createElementNS(_namespace, tag) { return new FakeNode(tag); },
};
const context = {};
for (const filename of process.argv.slice(1)) {
  vm.runInNewContext(
    fs.readFileSync(filename, "utf8"),
    context,
    { filename },
  );
}
const translations = {
  "common.missing": "—",
  "map.footprint.label": "BODY ENVELOPE",
  "map.footprint.no_contact_inference": "No contact inference",
  "map.scan.attempt_count": ({ count }) => `${count} retained attempts`,
  "map.scan.bearing_center": "0° centre",
  "map.scan.bearing_left": ({ value }) => `${value}° left`,
  "map.scan.bearing_right": ({ value }) => `${value}° right`,
  "map.scan.blocked": "reflection",
  "map.scan.clear": "clear",
  "map.scan.label": ({ count }) => `IR SCAN · ${count} attempts`,
  "map.scan.nonmetric": "No measured distance",
  "map.scan.pattern.all_clear": "all clear evidence",
  "map.scan.ray_title": ({ bearing, state }) => `${bearing} · ${state}`,
  "map.scan.title": ({ count }) => `${count} retained attempts`,
};
function translate(key, args = {}) {
  const value = translations[key];
  return typeof value === "function" ? value(args) : (value || key);
}
const presenter = context.RobotSpatialMapPresenter.create({
  document,
  normalizeSpatialMap: context.RobotDashboardLogic.normalizeSpatialMap,
  translate,
  formatNumber: (value) => String(value),
});
const commonScan = {
  target_hypothesis_id: "hazard-1",
  frame_id: "local-odometry",
  hypothesis_anchor_pose: {
    x_mm: 10,
    y_mm: 20,
    heading_mdeg: 30000,
  },
  status: "CANCELLED",
  reason: "bilateral_boundaries_not_observed",
  bearing_convention: "POSITIVE_LEFT_NEGATIVE_RIGHT",
  geometry_kind: "ANGULAR_NONMETRIC_IR_SCAN",
  arc_coverage: "BILATERAL_ARC",
  boundary_coverage: "NO_BOUNDARIES",
  left_boundary_mdeg: null,
  right_boundary_mdeg: null,
  provisional: true,
  read_only: true,
};
const map = presenter.render({
  schema: "robot-spatial-map/v1",
  read_only: true,
  status: "qualitative_only",
  robot_id: "robot-1",
  frame_id: "local-odometry",
  frame_kind: "LOCAL_ODOMETRY",
  map_version: 5,
  robot_pose: {
    x_mm: 0,
    y_mm: 0,
    heading_mdeg: 0,
  },
  collision_geometry: {
    geometry: "ASYMMETRIC_RECTANGLE",
    reference_point: "DIFFERENTIAL_DRIVE_ORIGIN",
    front_extent_mm: 110,
    rear_extent_mm: 90,
    left_extent_mm: 105,
    right_extent_mm: 160,
    clearance_margin_mm: 10,
    calibration_status: "provisional",
    calibration_evidence: "assembled right arm observed",
  },
  object_hypotheses: [{
    hypothesis_id: "hazard-1",
    label: "UNKNOWN",
    anchor_pose: {
      x_mm: 10,
      y_mm: 20,
      heading_mdeg: 30000,
    },
    geometry_kind: "QUALITATIVE_FORWARD_ENVELOPE",
    bearing: "FORWARD",
    relation: "NEAR_OBSTACLE",
    confidence_milli: 250,
    provisional: true,
  }],
  scan_evidence_history: [{
    ...commonScan,
    scan_id: "scan-with-actual-pose",
    completed_at_unix_ms: 1900,
    scan_pose: {
      x_mm: 80,
      y_mm: -35,
      heading_mdeg: 45000,
    },
    based_on_map_version: 3,
    observation_pattern: "MIXED",
    hypothesis_relation: "SUPPORTS_BLOCKED_HYPOTHESIS",
    rays: [{
      requested_relative_bearing_mdeg: -30000,
      actual_relative_bearing_mdeg: -28500,
      blocked: true,
      raw_ir_proximity: 24,
      filtered_ir_proximity: 25,
    }, {
      requested_relative_bearing_mdeg: 30000,
      actual_relative_bearing_mdeg: 31500,
      blocked: false,
      raw_ir_proximity: 56,
      filtered_ir_proximity: 55,
    }],
  }, {
    ...commonScan,
    scan_id: "legacy-without-pose",
    completed_at_unix_ms: 1950,
    scan_pose: null,
    based_on_map_version: null,
    observation_pattern: "ALL_CLEAR",
    hypothesis_relation: "CONFLICTS_BLOCKED_HYPOTHESIS",
    rays: [],
  }],
}, "connected", 2000);

function descendants(root) {
  return [root, ...root.children.flatMap(descendants)];
}
const rendered = descendants(nodes["map-local-odometry-layer"]);
const scans = rendered.filter((node) => (
  node.attributes["data-scan-id"]
));
const footprint = rendered.find((node) => (
  node.attributes.class === "map-local-robot-footprint"
));
const rayGroups = rendered.filter((node) => (
  String(node.attributes.class).startsWith("map-local-scan-ray ")
));
const lengths = rayGroups.map((group) => {
  const line = group.children.find((node) => node.tag === "line");
  return Math.hypot(
    Number(line.attributes.x2) - Number(line.attributes.x1),
    Number(line.attributes.y2) - Number(line.attributes.y1),
  );
});
process.stdout.write(JSON.stringify({
  collisionGeometry: map.collisionGeometry,
  scanHistory: map.scanEvidenceHistory,
  scanAttributes: scans.map((node) => node.attributes),
  footprintAttributes: footprint.attributes,
  rayAttributes: rayGroups.map((node) => node.attributes),
  rayClasses: rayGroups.map((node) => node.attributes.class),
  lengths,
  objectText: nodes["map-object-list"].textContent,
  localText: nodes["map-local-odometry-layer"].textContent,
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "blast_map_semantics.js"),
                str(WEB_ROOT / "dashboard_logic.js"),
                str(WEB_ROOT / "spatial_map_presenter.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        result = json.loads(completed.stdout)

        self.assertEqual(
            result["collisionGeometry"]["rightExtentMm"],
            160,
        )
        self.assertEqual(len(result["scanHistory"]), 2)
        self.assertTrue(result["scanHistory"][0]["spatiallyRenderable"])
        self.assertFalse(result["scanHistory"][1]["spatiallyRenderable"])
        self.assertEqual(len(result["scanAttributes"]), 1)
        self.assertEqual(
            result["scanAttributes"][0]["data-scan-id"],
            "scan-with-actual-pose",
        )
        self.assertEqual(
            result["scanAttributes"][0]["data-attempt-count"],
            "2",
        )
        self.assertEqual(
            (
                result["scanAttributes"][0]["data-origin-x-mm"],
                result["scanAttributes"][0]["data-origin-y-mm"],
                result["scanAttributes"][0][
                    "data-origin-heading-mdeg"
                ],
            ),
            ("80", "-35", "45000"),
        )
        self.assertEqual(
            result["scanAttributes"][0][
                "data-based-on-map-version"
            ],
            "3",
        )
        self.assertEqual(
            result["scanAttributes"][0]["data-metric-distance"],
            "none",
        )
        self.assertEqual(
            result["footprintAttributes"]["data-right-extent-mm"],
            "160",
        )
        self.assertEqual(
            result["footprintAttributes"]["data-left-extent-mm"],
            "105",
        )
        self.assertEqual(len(result["rayAttributes"]), 2)
        self.assertEqual(
            {
                item["data-actual-bearing-mdeg"]
                for item in result["rayAttributes"]
            },
            {"-28500", "31500"},
        )
        self.assertTrue(all(
            item["data-metric-distance"] == "none"
            for item in result["rayAttributes"]
        ))
        self.assertIn("is-blocked", result["rayClasses"][0])
        self.assertIn("is-clear", result["rayClasses"][1])
        self.assertAlmostEqual(result["lengths"][0], 86)
        self.assertAlmostEqual(result["lengths"][1], 86)
        self.assertIn("2 retained attempts", result["objectText"])
        self.assertIn("all clear evidence", result["objectText"])
        self.assertIn("BODY ENVELOPE", result["localText"])

    def test_local_odometry_layer_is_visible_and_metric_svg_stays_honest(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");

class FakeNode {
  constructor(tag, id = null) {
    this.tag = tag;
    this.id = id;
    this.className = "";
    this.hidden = false;
    this.attributes = {};
    this.children = [];
    this._text = "";
  }
  set textContent(value) {
    this._text = String(value);
    this.children = [];
  }
  get textContent() {
    return this._text + this.children
      .map((child) => child.textContent)
      .join("");
  }
  appendChild(child) {
    this.children.push(child);
    return child;
  }
  replaceChildren(...children) {
    this._text = "";
    this.children = children;
  }
  setAttribute(name, value) {
    this.attributes[name] = String(value);
  }
}

const requiredIds = [
  "map-connection-status",
  "map-frame-label",
  "map-empty-state",
  "map-empty-title",
  "map-empty-body",
  "map-metadata",
  "map-qualitative-list",
  "map-qualitative-count",
  "map-object-list",
  "map-object-count",
  "map-local-odometry-layer",
  "map-cell-layer",
  "map-path-layer",
  "map-ray-layer",
  "map-object-layer",
  "map-robot-layer",
];
const nodes = Object.fromEntries(
  requiredIds.map((id) => [id, new FakeNode("div", id)]),
);
const document = {
  getElementById(id) {
    if (!nodes[id]) {
      throw new Error(`missing fixture node ${id}`);
    }
    return nodes[id];
  },
  createElement(tag) {
    return new FakeNode(tag);
  },
  createElementNS(_namespace, tag) {
    return new FakeNode(tag);
  },
};

const context = {};
for (const filename of process.argv.slice(1)) {
  vm.runInNewContext(
    fs.readFileSync(filename, "utf8"),
    context,
    { filename },
  );
}
const translations = {
  "common.missing": "—",
  "map.age.milliseconds": ({ value }) => `${value} ms`,
  "map.age.seconds": ({ value }) => `${value} s`,
  "map.age.minutes": ({ value }) => `${value} min`,
  "map.details.status": "Status",
  "map.details.reason": "Reason",
  "map.details.path_points": "Path points",
  "map.details.path_points_truncated": ({ count, evicted }) => (
    `${count} shown · ${evicted} older removed`
  ),
  "map.details.hazard_retention": "Hazard hypotheses",
  "map.details.hazard_retention_value": ({ retained, capacity, evicted }) => (
    `${retained} / ${capacity} retained · ${evicted} evicted`
  ),
  "map.details.scan_attempts": "Retained scan attempts",
  "map.details.scan_attempts_truncated": ({ count, evicted, total }) => (
    `${count} retained · ${evicted} evicted · ${total} observed`
  ),
  "map.details.scan_memory_retention": "Persistent scan memory",
  "map.details.scan_memory_retention_value": ({
    retained,
    mapCapacity,
    perHazardCapacity,
    evicted,
    reason,
  }) => (
    `${retained} / ${mapCapacity} retained · up to `
    + `${perHazardCapacity} per hazard · ${evicted} evicted`
    + (reason ? ` · latest reason: ${reason}` : "")
  ),
  "map.details.source": "Source",
  "map.details.state_version": "State version",
  "map.details.age": "Age",
  "map.details.provenance": "Provenance",
  "map.details.version": "Version",
  "map.details.world_version": "World version",
  "map.details.robot": "Robot",
  "map.empty.body": "No map body",
  "map.empty.title": "No map",
  "map.empty.pose_body": "Pose body",
  "map.empty.pose_title": "Pose only",
  "map.empty.qualitative_body": "Qualitative evidence panel",
  "map.empty.qualitative_title": "No metric map",
  "map.frame.unavailable": "No frame",
  "map.legend.robot": "Robot",
  "map.legend.sensor_ray": "Ray",
  "map.local_odometry.ir_label": "PROVISIONAL IR",
  "map.local_odometry.ir_title": ({ relation }) => (
    `Provisional IR cue · ${relation}`
  ),
  "map.local_odometry.layer_label": "PROVISIONAL LOCAL ODOMETRY",
  "map.local_odometry.layer_note": "Angular IR cue · no measured distance",
  "map.local_odometry.localization_lost_note": (
    "Last verified path and scan · current pose unknown"
  ),
  "map.local_odometry.nonmetric": "No metric IR distance",
  "map.local_odometry.robot_title": "Provisional robot pose",
  "map.path.title": "Estimated path from odometry",
  "map.objects.confidence": ({ value }) => `${value}% confidence`,
  "map.objects.empty": "No objects",
  "map.objects.no_metadata": "No metadata",
  "map.objects.unnamed": "Unnamed",
  "map.qualitative.age": "Age",
  "map.qualitative.bearing": ({ bearing }) => `bearing ${bearing}`,
  "map.qualitative.confidence": "Confidence",
  "map.qualitative.empty": "No qualitative observations",
  "map.qualitative.provisional": "Provisional",
  "map.qualitative.raw": "Raw IR",
  "map.qualitative.raw_value": ({ value }) => `${value} / 100`,
  "map.qualitative.retention": ({ shown, retained, evicted, total }) => (
    `${shown} shown · ${retained} retained · ${evicted} evicted · ${total} observed`
  ),
  "map.qualitative.relation.near_obstacle": "Near reflection",
  "map.qualitative.relation.no_near_reflection": "No near reflection",
  "map.qualitative.relation.unknown": "Unknown relation",
  "map.read_only": "Read only",
  "map.status.degraded": "Degraded",
  "map.status.empty": "No map",
  "map.status.invalid": "Invalid",
  "map.status.live": "Live",
  "map.status.localization_lost": "Localization lost · history retained",
  "map.status.offline": "Offline",
  "map.status.pose_only": "Pose only",
  "map.status.qualitative_only": "Qualitative IR available",
  "map.status.waiting": "Waiting",
  "map.tooltip.age": ({ age }) => `age ${age}`,
  "map.tooltip.provenance": ({ provenance }) => (
    `provenance ${provenance}`
  ),
  "map.tooltip.source": ({ source }) => `source ${source}`,
  "map.cell.free": "Free",
};
function translate(key, args = {}) {
  const value = translations[key];
  return typeof value === "function" ? value(args) : (value || key);
}
const presenter = context.RobotSpatialMapPresenter.create({
  document,
  normalizeSpatialMap: (
    context.RobotDashboardLogic.normalizeSpatialMap
  ),
  translate,
  formatNumber: (value) => String(value),
});
const qualitativeHistory = Array.from({ length: 150 }, (_value, index) => ({
  bearing: "FORWARD",
  relation: "NEAR_OBSTACLE",
  raw_ir_proximity: index % 101,
  confidence_milli: 250,
  source_id: "physical_ir_reflection",
  provenance: "PROVISIONAL_IR",
  provisional: true,
  observed_at_unix_ms: 1900,
  age_ms: 1,
}));

const qualitative = presenter.render({
  schema: "robot-spatial-map/v1",
  read_only: true,
  status: "qualitative_only",
  robot_id: "robot-1",
  frame_id: "local-odometry",
  frame_kind: "LOCAL_ODOMETRY",
  map_version: 1,
  bounds: null,
  cells: [],
  sensor_rays: [],
  object_hypotheses: [{
    hypothesis_id: "provisional-object-1",
    label: "UNKNOWN",
    x_mm: null,
    y_mm: null,
    bounds: null,
    anchor_pose: {
      x_mm: 10,
      y_mm: 20,
      heading_mdeg: 60000,
    },
    geometry_kind: "QUALITATIVE_FORWARD_ENVELOPE",
    bearing: "FORWARD",
    relation: "NEAR_OBSTACLE",
    confidence_milli: 250,
    source_id: "physical_ir_reflection",
    provenance: "LOCAL_ODOMETRY_POSE | physical_ir_reflection",
    provisional: true,
    observed_at_unix_ms: 1900,
  }],
  qualitative_observations: qualitativeHistory,
  qualitative_observations_evicted: 17,
  scan_evidence_history: [],
  scan_evidence_history_evicted: 4,
  hazard_retention: {
    capacity: 64,
    retained_count: 9,
    evicted_count: 2,
    last_eviction_reason: "MAP_CAPACITY_OLDEST_HAZARD",
  },
  scan_attempt_retention: {
    per_hazard_capacity: 16,
    map_capacity: 64,
    retained_count: 31,
    evicted_count: 6,
    last_eviction_reason: "PER_HAZARD_CAPACITY_DIVERSITY_RETENTION",
  },
}, "connected", 2000);
const qualitativeResult = {
  status: qualitative.status,
  age: qualitative.qualitativeObservations[0].ageMs,
  connection: nodes["map-connection-status"].textContent,
  emptyHidden: nodes["map-empty-state"].hidden,
  emptyTitle: nodes["map-empty-title"].textContent,
  emptyBody: nodes["map-empty-body"].textContent,
  count: nodes["map-qualitative-count"].textContent,
  renderedCount: nodes["map-qualitative-list"].children.length,
  metadataText: nodes["map-metadata"].textContent,
  panelText: nodes["map-qualitative-list"].textContent,
  objectCount: nodes["map-object-count"].textContent,
  objectPanelText: nodes["map-object-list"].textContent,
  metricLayerCounts: [
    nodes["map-cell-layer"].children.length,
    nodes["map-ray-layer"].children.length,
    nodes["map-object-layer"].children.length,
    nodes["map-robot-layer"].children.length,
  ],
  localAttributes: nodes["map-local-odometry-layer"].attributes,
  localTags: nodes["map-local-odometry-layer"].children
    .map((node) => node.tag),
  localText: nodes["map-local-odometry-layer"].textContent,
  cueAttributes: nodes["map-local-odometry-layer"]
    .children[2].attributes,
  cueTags: nodes["map-local-odometry-layer"]
    .children[2].children.map((node) => node.tag),
  wedgeScreenLength: (() => {
    const path = nodes["map-local-odometry-layer"]
      .children[2].children[1].attributes.d.split(" ");
    return Math.hypot(
      Number(path[4]) - Number(path[1]),
      Number(path[5]) - Number(path[2]),
    );
  })(),
};

presenter.render({
  schema: "robot-spatial-map/v1",
  read_only: true,
  status: "pose_only",
  robot_id: "robot-1",
  frame_id: "LOCAL_ODOMETRY",
  frame_kind: "LOCAL_ODOMETRY",
  map_version: 2,
  bounds: null,
  robot_pose: {
    x_mm: 10,
    y_mm: 20,
    heading_mdeg: 90000,
  },
  pose_history: [{
    x_mm: -20,
    y_mm: 20,
    heading_mdeg: 0,
    frame_id: "LOCAL_ODOMETRY",
    observed_at_unix_ms: 1800,
  }, {
    x_mm: 10,
    y_mm: 20,
    heading_mdeg: 90000,
    frame_id: "LOCAL_ODOMETRY",
    observed_at_unix_ms: 1900,
  }],
  pose_history_evicted: 2,
  cells: [],
  sensor_rays: [],
  object_hypotheses: [],
  qualitative_observations: [],
}, "connected", 2000);
const poseResult = {
  connection: nodes["map-connection-status"].textContent,
  emptyHidden: nodes["map-empty-state"].hidden,
  emptyTitle: nodes["map-empty-title"].textContent,
  emptyBody: nodes["map-empty-body"].textContent,
  metricLayerCounts: [
    nodes["map-cell-layer"].children.length,
    nodes["map-ray-layer"].children.length,
    nodes["map-object-layer"].children.length,
    nodes["map-robot-layer"].children.length,
  ],
  metadataText: nodes["map-metadata"].textContent,
  localTags: nodes["map-local-odometry-layer"].children
    .map((node) => node.tag),
  pathAttributes: nodes["map-local-odometry-layer"]
    .children[2].attributes,
  robotAttributes: nodes["map-local-odometry-layer"]
    .children[4].attributes,
  robotTags: nodes["map-local-odometry-layer"]
    .children[4].children.map((node) => node.tag),
  headingLength: (() => {
    const heading = nodes["map-local-odometry-layer"]
      .children[4].children[2].attributes;
    return Math.hypot(
      Number(heading.x2) - Number(heading.x1),
      Number(heading.y2) - Number(heading.y1),
    );
  })(),
};

presenter.render({
  schema: "robot-spatial-map/v1",
  read_only: true,
  status: "unavailable",
  reason_code: "localization_lost",
  robot_id: "robot-1",
  frame_id: "LOCAL_ODOMETRY",
  frame_kind: "LOCAL_ODOMETRY",
  map_version: 3,
  bounds: null,
  robot_pose: null,
  pose_history: [{
    x_mm: 0,
    y_mm: 0,
    heading_mdeg: 0,
    frame_id: "LOCAL_ODOMETRY",
  }, {
    x_mm: 90,
    y_mm: 0,
    heading_mdeg: 0,
    frame_id: "LOCAL_ODOMETRY",
  }],
  cells: [],
  sensor_rays: [],
  object_hypotheses: [],
  qualitative_observations: [],
}, "connected", 2000);
const localizationLostResult = {
  connection: nodes["map-connection-status"].textContent,
  emptyHidden: nodes["map-empty-state"].hidden,
  localTags: nodes["map-local-odometry-layer"].children
    .map((node) => node.tag),
  localText: nodes["map-local-odometry-layer"].textContent,
};

presenter.render({
  schema: "robot-spatial-map/v1",
  read_only: true,
  status: "available",
  robot_id: "robot-1",
  frame_id: "SIM_WORLD",
  frame_kind: "SIMULATION_WORLD",
  map_version: 2,
  based_on_state_version: 19,
  based_on_world_model_version: 4,
  resolution_mm: 50,
  bounds: {
    min_x_mm: 0,
    min_y_mm: 0,
    max_x_mm: 100,
    max_y_mm: 100,
  },
  robot_pose: {
    x_mm: 25,
    y_mm: 25,
    heading_mdeg: 0,
  },
  pose_history: [{
    x_mm: 10,
    y_mm: 10,
    heading_mdeg: 0,
    frame_id: "SIM_WORLD",
  }, {
    x_mm: 25,
    y_mm: 25,
    heading_mdeg: 0,
    frame_id: "SIM_WORLD",
  }],
  cells: [{
    x_mm: 25,
    y_mm: 25,
    size_mm: 50,
    state: "FREE",
  }],
  sensor_rays: [{
    origin_x_mm: 25,
    origin_y_mm: 25,
    end_x_mm: 75,
    end_y_mm: 25,
    valid_until_unix_ms: 2100,
  }],
  object_hypotheses: [{
    hypothesis_id: "object-1",
    label: "UNKNOWN",
    x_mm: 75,
    y_mm: 75,
  }],
  qualitative_observations: [],
}, "connected", 2000);
const metricResult = {
  connection: nodes["map-connection-status"].textContent,
  emptyHidden: nodes["map-empty-state"].hidden,
  qualitativeText: nodes["map-qualitative-list"].textContent,
  metadataText: nodes["map-metadata"].textContent,
  cellTags: nodes["map-cell-layer"].children.map((node) => node.tag),
  rayTags: nodes["map-ray-layer"].children.map((node) => node.tag),
  objectTags: nodes["map-object-layer"].children.map((node) => node.tag),
  robotTags: nodes["map-robot-layer"].children.map((node) => node.tag),
  pathTags: nodes["map-path-layer"].children.map((node) => node.tag),
  localLayerCount: nodes["map-local-odometry-layer"].children.length,
};

let invalidDependenciesRejected = false;
try {
  context.RobotSpatialMapPresenter.create({});
} catch (error) {
  invalidDependenciesRejected = error && error.name === "TypeError";
}

process.stdout.write(JSON.stringify({
  qualitativeResult,
  poseResult,
  localizationLostResult,
  metricResult,
  invalidDependenciesRejected,
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "blast_map_semantics.js"),
                str(WEB_ROOT / "dashboard_logic.js"),
                str(WEB_ROOT / "spatial_map_presenter.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        result = json.loads(completed.stdout)
        qualitative = result["qualitativeResult"]
        self.assertEqual(qualitative["status"], "qualitative_only")
        self.assertEqual(qualitative["age"], 100)
        self.assertEqual(
            qualitative["connection"],
            "Qualitative IR available",
        )
        self.assertTrue(qualitative["emptyHidden"])
        self.assertEqual(qualitative["emptyTitle"], "No metric map")
        self.assertEqual(
            qualitative["emptyBody"],
            "Qualitative evidence panel",
        )
        self.assertEqual(
            qualitative["count"],
            "100 shown · 150 retained · 17 evicted · 167 observed",
        )
        self.assertEqual(qualitative["renderedCount"], 100)
        self.assertIn(
            "0 retained · 4 evicted · 4 observed",
            qualitative["metadataText"],
        )
        self.assertIn(
            "9 / 64 retained · 2 evicted",
            qualitative["metadataText"],
        )
        self.assertIn(
            "31 / 64 retained · up to 16 per hazard · 6 evicted",
            qualitative["metadataText"],
        )
        self.assertIn(
            "PER_HAZARD_CAPACITY_DIVERSITY_RETENTION",
            qualitative["metadataText"],
        )
        self.assertEqual(qualitative["objectCount"], "1")
        for expected in (
            "UNKNOWN",
            "physical_ir_reflection",
            "LOCAL_ODOMETRY_POSE",
            "25% confidence",
            "Near reflection",
            "bearing FORWARD",
        ):
            with self.subTest(expected=expected):
                self.assertIn(
                    expected,
                    qualitative["objectPanelText"],
                )
        for expected in (
            "Near reflection",
            "81 / 100",
            "25% confidence",
            "100 ms",
            "physical_ir_reflection",
            "PROVISIONAL IR",
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, qualitative["panelText"])
        self.assertEqual(qualitative["metricLayerCounts"], [0, 0, 0, 0])
        self.assertEqual(
            qualitative["localAttributes"],
            {
                "data-provisional": "true",
                "data-geometry": "screen-space-nonmetric",
            },
        )
        self.assertEqual(qualitative["localTags"], ["text", "text", "g"])
        self.assertEqual(
            qualitative["cueAttributes"]["data-metric-distance"],
            "none",
        )
        self.assertEqual(
            qualitative["cueTags"],
            ["title", "path", "line", "circle", "text"],
        )
        self.assertAlmostEqual(qualitative["wedgeScreenLength"], 64)
        for expected in (
            "PROVISIONAL LOCAL ODOMETRY",
            "no measured distance",
            "PROVISIONAL IR",
            "Near reflection",
            "No metric IR distance",
        ):
            with self.subTest(local_layer_text=expected):
                self.assertIn(expected, qualitative["localText"])

        pose = result["poseResult"]
        self.assertEqual(pose["connection"], "Pose only")
        self.assertTrue(pose["emptyHidden"])
        self.assertEqual(pose["emptyTitle"], "Pose only")
        self.assertEqual(pose["emptyBody"], "Pose body")
        self.assertEqual(pose["metricLayerCounts"], [0, 0, 0, 0])
        self.assertEqual(
            pose["localTags"],
            ["text", "text", "path", "circle", "g"],
        )
        self.assertEqual(pose["pathAttributes"]["class"], "map-path")
        self.assertTrue(pose["pathAttributes"]["d"].startswith("M "))
        self.assertEqual(
            pose["robotAttributes"]["data-provisional"],
            "true",
        )
        self.assertEqual(
            pose["robotTags"],
            ["title", "circle", "line", "circle"],
        )
        self.assertAlmostEqual(pose["headingLength"], 58)
        self.assertIn(
            "Path points2 shown · 2 older removed",
            pose["metadataText"],
        )

        localization_lost = result["localizationLostResult"]
        self.assertEqual(
            localization_lost["connection"],
            "Localization lost · history retained",
        )
        self.assertTrue(localization_lost["emptyHidden"])
        self.assertEqual(
            localization_lost["localTags"],
            ["text", "text", "path", "circle"],
        )
        self.assertIn(
            "current pose unknown",
            localization_lost["localText"],
        )

        metric = result["metricResult"]
        self.assertEqual(metric["connection"], "Live")
        self.assertTrue(metric["emptyHidden"])
        self.assertEqual(
            metric["qualitativeText"],
            "No qualitative observations",
        )
        self.assertEqual(metric["cellTags"], ["rect"])
        self.assertEqual(metric["rayTags"], ["line", "circle"])
        self.assertEqual(metric["objectTags"], ["g"])
        self.assertEqual(metric["robotTags"], ["g"])
        self.assertEqual(metric["pathTags"], ["path", "circle"])
        self.assertEqual(metric["localLayerCount"], 0)
        self.assertIn("State version19", metric["metadataText"])
        self.assertIn("World version4", metric["metadataText"])
        self.assertIn("Path points2", metric["metadataText"])
        self.assertTrue(result["invalidDependenciesRejected"])

    def test_blast_navigation_trace_is_strict_and_uses_the_existing_map(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
class FakeNode {
  constructor(tag, id = null) {
    this.tag = tag;
    this.id = id;
    this.className = "";
    this.hidden = false;
    this.attributes = {};
    this.children = [];
    this._text = "";
  }
  set textContent(value) { this._text = String(value); this.children = []; }
  get textContent() {
    return this._text + this.children.map((child) => child.textContent).join("");
  }
  appendChild(child) { this.children.push(child); return child; }
  replaceChildren(...children) { this._text = ""; this.children = children; }
  setAttribute(name, value) { this.attributes[name] = String(value); }
}
const ids = [
  "map-connection-status", "map-frame-label", "map-empty-state",
  "map-empty-title", "map-empty-body", "map-metadata",
  "map-qualitative-list", "map-qualitative-count", "map-object-list",
  "map-object-count", "map-local-odometry-layer", "map-cell-layer",
  "map-path-layer", "map-ray-layer", "map-object-layer", "map-robot-layer",
];
const nodes = Object.fromEntries(ids.map((id) => [id, new FakeNode("div", id)]));
const document = {
  getElementById(id) { return nodes[id]; },
  createElement(tag) { return new FakeNode(tag); },
  createElementNS(_namespace, tag) { return new FakeNode(tag); },
};
const context = {};
for (const filename of process.argv.slice(1)) {
  vm.runInNewContext(fs.readFileSync(filename, "utf8"), context, { filename });
}
const translations = {
  "common.missing": "—",
  "map.navigation_trace.final_goal_title": "FINAL GOAL",
  "map.navigation_trace.goal_distance": ({ distance, radius }) => (
    `DISTANCE ${distance} RADIUS ${radius}`
  ),
  "map.navigation_trace.goal_progress": ({ current, target, remaining }) => (
    `${current}/${target}/${remaining}`
  ),
  "map.navigation_trace.goal_heading": ({ heading }) => `GOAL ${heading}`,
  "map.navigation_trace.final_goal_label": ({ distance }) => `GOAL ${distance}`,
  "map.navigation_trace.advisory_waypoint_label": "GEMMA WAYPOINT",
  "map.navigation_trace.advisory_waypoint_pose": ({ x, y }) => `${x}/${y}`,
  "map.navigation_trace.advisory_waypoint_read_only": "READ ONLY",
  "map.navigation_trace.advisory_waypoint_title": "ADVISORY WAYPOINT",
  "map.navigation_trace.planned_leg_title": ({ side }) => `PLAN ${side}`,
  "map.navigation_trace.search_position_only": "SEARCH POSITION ONLY",
  "map.navigation_trace.planned_waypoint_label": "WAYPOINT",
  "map.navigation_trace.scan_title": ({ count }) => `SCAN ${count}`,
  "map.navigation_trace.scan_limitations": "PROVISIONAL_YAW_ONLY",
  "map.navigation_trace.scan_ray_title": ({ range }) => `RANGE ${range}`,
  "map.navigation_trace.scan_label": ({ count }) => `VIEW ${count}`,
  "map.navigation_trace.imu_title": ({ heading }) => `IMU ${heading}`,
  "map.navigation_trace.imu_reference": "EPISODE RELATIVE",
  "map.navigation_trace.layer_label": "BLAST TRACE",
  "map.navigation_trace.layer_note": "PLAN / ENCODER / IMU / ECHO",
  "map.navigation_trace.odometry_title": "PROVISIONAL_ENCODER_ODOMETRY",
};
function translate(key, args = {}) {
  const value = translations[key];
  return typeof value === "function" ? value(args) : (value || key);
}
const logic = context.RobotDashboardLogic;
const presenter = context.RobotSpatialMapPresenter.create({
  document,
  normalizeSpatialMap: logic.normalizeSpatialMap,
  translate,
  formatNumber: (value) => String(value),
});
const point = {
  side: "center",
  measured_range_mm: 400,
  relative_bearing_mdeg: 0,
  sensor_origin_x_mm: 11,
  sensor_origin_y_mm: 0,
  beam_heading_mdeg: 0,
  nominal_echo_x_mm: 411,
  nominal_echo_y_mm: 0,
};
const scan = {
  scan_id: "origin-scan",
  observed_at_unix_ms: 1900,
  scan_pose: { x_mm: 0, y_mm: 0, heading_mdeg: 0 },
  projection: {
    schema: "blast-planar-scan-projection/v1",
    frame: "EPISODE_LOCAL_ODOMETRY",
    quality: "PROVISIONAL_YAW_ONLY",
    vertical_pitch_compensated: false,
    ultrasonic_beam_width_modeled: false,
    scan_turn_translation_compensated: false,
    points: [point],
  },
};
const trace = {
  schema: "robot-navigation-trace/v1",
  read_only: true,
  frame_id: "blast-episode-1",
  provenance: "PROVISIONAL_ENCODER_ODOMETRY + PROVISIONAL_YAW_ONLY",
  final_goal: {
    kind: "DIRECTIONAL_HEADING",
    navigation_enforced: false,
    origin_x_mm: 0,
    origin_y_mm: 0,
    target_x_mm: 600,
    target_y_mm: 0,
    desired_heading_mdeg: 0,
    minimum_forward_progress_mm: 600,
    heading_tolerance_mdeg: 5000,
    current_forward_progress_mm: 20,
    current_lateral_offset_mm: 210,
    remaining_forward_progress_mm: 580,
    goal_radius_mm: 120,
    distance_to_goal_mm: 617,
  },
  imu_heading: {
    heading_mdeg: 95000,
    reference: "EPISODE_START",
    observed_at_unix_ms: 1950,
  },
  planned_leg: {
    kind: "SIDE_SEARCH",
    scope: "SEARCH_POSITION_ONLY",
    clearance_proven: false,
    passage_proven: false,
    route_eligible: false,
    selected_side: "LEFT",
    bind_pose: { x_mm: 0, y_mm: 0, heading_mdeg: 90000 },
    waypoint: { x_mm: 0, y_mm: 225, heading_mdeg: 90000 },
  },
  advisory_waypoint: {
    x_mm: 120,
    y_mm: -280,
    purpose: "Pass the obstacle on its open right side",
    source: "GEMMA_MODEL",
    read_only: true,
  },
  planar_scan_views: [scan],
};
const rawMap = {
  schema: "robot-spatial-map/v1",
  read_only: true,
  status: "pose_only",
  robot_id: "blast-01",
  frame_id: "blast-episode-1",
  frame_kind: "LOCAL_ODOMETRY",
  bounds: null,
  robot_pose: { x_mm: 20, y_mm: 210, heading_mdeg: 0 },
  pose_history: [
    { x_mm: 0, y_mm: 0, heading_mdeg: 0 },
    { x_mm: 20, y_mm: 210, heading_mdeg: 0 },
  ],
  navigation_trace: trace,
};
const renderedMap = presenter.render(rawMap, "connected", 2000);
function descendants(root) {
  return [root, ...root.children.flatMap(descendants)];
}
const rendered = descendants(nodes["map-local-odometry-layer"]);
const byClass = (name) => rendered.find((node) => node.attributes.class === name);
const lineLength = (line) => Math.hypot(
  Number(line.attributes.x2) - Number(line.attributes.x1),
  Number(line.attributes.y2) - Number(line.attributes.y1),
);
const goalLine = byClass("map-final-goal-line");
const rayLine = byClass("map-blast-scan-ray-line");
const invalidFrame = logic.normalizeSpatialMap({
  ...rawMap,
  navigation_trace: { ...trace, frame_id: "another-episode" },
}, 2000);
const invalidEcho = logic.normalizeSpatialMap({
  ...rawMap,
  navigation_trace: {
    ...trace,
    planar_scan_views: [{
      ...scan,
      projection: {
        ...scan.projection,
        points: [{ ...point, nominal_echo_x_mm: 900 }],
      },
    }],
  },
}, 2000);
const overCapacity = logic.normalizeSpatialMap({
  ...rawMap,
  navigation_trace: {
    ...trace,
    planar_scan_views: Array.from({ length: 17 }, (_unused, index) => ({
      ...scan,
      scan_id: `scan-${index}`,
    })),
  },
}, 2000);
process.stdout.write(JSON.stringify({
  trace: renderedMap.navigationTrace,
  traceFrozen: Object.isFrozen(renderedMap.navigationTrace)
    && Object.isFrozen(renderedMap.navigationTrace.planarScanViews),
  layerAttributes: nodes["map-local-odometry-layer"].attributes,
  goalAttributes: byClass("map-final-goal").attributes,
  goalZoneAttributes: byClass("map-final-goal-zone").attributes,
  advisoryAttributes: byClass("map-advisory-waypoint").attributes,
  hasAdvisoryMarker: Boolean(byClass("map-advisory-waypoint-marker")),
  legAttributes: byClass("map-planned-leg").attributes,
  imuAttributes: byClass("map-local-imu-heading").attributes,
  rayAttributes: byClass("map-blast-scan-ray").attributes,
  hasEcho: Boolean(byClass("map-blast-scan-echo")),
  topLayer: nodes["map-local-odometry-layer"].children.at(-1).attributes.class,
  distanceRatio: lineLength(rayLine) / lineLength(goalLine),
  localText: nodes["map-local-odometry-layer"].textContent,
  invalidFrame: invalidFrame.navigationTrace,
  invalidEcho: invalidEcho.navigationTrace,
  overCapacity: overCapacity.navigationTrace,
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "blast_map_semantics.js"),
                str(WEB_ROOT / "dashboard_logic.js"),
                str(WEB_ROOT / "spatial_map_presenter.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        result = json.loads(completed.stdout)
        self.assertTrue(result["traceFrozen"])
        self.assertEqual(result["trace"]["finalGoal"]["targetX"], 600)
        self.assertFalse(
            result["trace"]["finalGoal"]["navigationEnforced"]
        )
        self.assertEqual(
            result["layerAttributes"]["data-geometry"],
            "mixed-local-odometry",
        )
        self.assertEqual(
            result["goalAttributes"]["data-remaining-forward-progress-mm"],
            "580",
        )
        self.assertEqual(
            result["goalAttributes"]["data-distance-to-goal-mm"],
            "617",
        )
        self.assertEqual(
            result["goalAttributes"]["data-goal-radius-mm"],
            "120",
        )
        self.assertGreater(float(result["goalZoneAttributes"]["r"]), 11)
        self.assertEqual(
            result["goalAttributes"]["data-navigation-enforced"],
            "false",
        )
        self.assertEqual(
            result["legAttributes"]["data-scope"],
            "SEARCH_POSITION_ONLY",
        )
        self.assertEqual(
            result["legAttributes"]["data-clearance-proven"],
            "false",
        )
        self.assertEqual(result["imuAttributes"]["data-heading-mdeg"], "95000")
        self.assertEqual(
            result["advisoryAttributes"]["data-source"],
            "GEMMA_MODEL",
        )
        self.assertEqual(result["advisoryAttributes"]["data-read-only"], "true")
        self.assertTrue(result["hasAdvisoryMarker"])
        self.assertEqual(
            result["rayAttributes"]["data-quality"],
            "PROVISIONAL_YAW_ONLY",
        )
        self.assertEqual(
            result["rayAttributes"]["data-measured-range-mm"],
            "400",
        )
        self.assertTrue(result["hasEcho"])
        self.assertEqual(
            result["topLayer"],
            "map-navigation-overlay-layer",
        )
        self.assertAlmostEqual(result["distanceRatio"], 2 / 3)
        self.assertIn("PROVISIONAL_ENCODER_ODOMETRY", result["localText"])
        self.assertIn("SEARCH POSITION ONLY", result["localText"])
        self.assertIn("GEMMA WAYPOINT", result["localText"])
        self.assertIsNone(result["invalidFrame"])
        self.assertIsNone(result["invalidEcho"])
        self.assertIsNone(result["overCapacity"])

    def test_shared_world_renders_separate_robots_without_local_evidence(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
class FakeNode {
  constructor(tag, id = null) {
    this.tag = tag;
    this.id = id;
    this.className = "";
    this.hidden = false;
    this.attributes = {};
    this.children = [];
    this._text = "";
  }
  set textContent(value) { this._text = String(value); this.children = []; }
  get textContent() {
    return this._text + this.children.map((child) => child.textContent).join("");
  }
  appendChild(child) { this.children.push(child); return child; }
  replaceChildren(...children) { this._text = ""; this.children = children; }
  setAttribute(name, value) { this.attributes[name] = String(value); }
}
const ids = [
  "map-connection-status", "map-frame-label", "map-empty-state",
  "map-empty-title", "map-empty-body", "map-metadata",
  "map-qualitative-list", "map-qualitative-count", "map-object-list",
  "map-object-count", "map-local-odometry-layer", "map-cell-layer",
  "map-path-layer", "map-ray-layer", "map-object-layer", "map-robot-layer",
];
const nodes = Object.fromEntries(ids.map((id) => [id, new FakeNode("div", id)]));
const document = {
  getElementById(id) { return nodes[id]; },
  createElement(tag) { return new FakeNode(tag); },
  createElementNS(_namespace, tag) { return new FakeNode(tag); },
};
const context = {};
for (const filename of process.argv.slice(1)) {
  vm.runInNewContext(fs.readFileSync(filename, "utf8"), context, { filename });
}
function pose(robotId, localFrameId, xMm, yMm, headingMdeg, stateVersion) {
  return {
    x_mm: xMm,
    y_mm: yMm,
    heading_mdeg: headingMdeg,
    frame_id: "shared-world",
    local_frame_id: localFrameId,
    state_version: stateVersion,
    source_id: `${robotId}-odometry`,
    provenance: "CALIBRATED_FIXED_START_SE2_PROJECTION",
    observed_at_unix_ms: 1800 + stateVersion,
    age_ms: 100,
  };
}
function robot({
  robotId, controllerId, localFrameId, generationId, poses, geometry,
}) {
  return {
    read_only: true,
    status: "available",
    reason_code: "pose_transformed",
    robot_id: robotId,
    controller_instance_id: controllerId,
    local_frame_id: localFrameId,
    local_generation_id: generationId,
    robot_pose: poses[poses.length - 1],
    pose_history: poses,
    pose_history_evicted: 0,
    collision_geometry: geometry,
    frame_transform: {
      source_robot_id: robotId,
      source_controller_id: controllerId,
      source_frame_id: localFrameId,
      source_generation_id: generationId,
      world_frame_id: "shared-world",
      world_generation_id: "world-gen-1",
      tx_mm: 0,
      ty_mm: 0,
      yaw_mdeg: 0,
      position_uncertainty_mm: 10,
      yaw_uncertainty_mdeg: 500,
      provenance: ["FIXED_START_MEASUREMENT"],
    },
    source_map_id: `${robotId}-map`,
    source_map_version: 3,
    source_status: "pose_only",
    captured_at_unix_ms: 2000,
    source_age_ms: 100,
  };
}
const rectangle = {
  geometry: "ASYMMETRIC_RECTANGLE",
  reference_point: "DIFFERENTIAL_DRIVE_ORIGIN",
  front_extent_mm: 110,
  rear_extent_mm: 90,
  left_extent_mm: 105,
  right_extent_mm: 160,
  clearance_margin_mm: 10,
};
const circle = {
  geometry: "SYMMETRIC_CIRCLE",
  reference_point: "DIFFERENTIAL_DRIVE_ORIGIN",
  radius_mm: 120,
};
const ev3 = robot({
  robotId: "ev3rstorm-01",
  controllerId: "ev3-controller",
  localFrameId: "ev3-local",
  generationId: "ev3-gen",
  poses: [
    pose("ev3rstorm-01", "ev3-local", 0, 0, 0, 1),
    pose("ev3rstorm-01", "ev3-local", 200, 0, 0, 2),
  ],
  geometry: rectangle,
});
const blast = robot({
  robotId: "blast-01",
  controllerId: "blast-controller",
  localFrameId: "blast-local",
  generationId: "blast-gen",
  poses: [
    pose("blast-01", "blast-local", 1000, 500, 90000, 1),
    pose("blast-01", "blast-local", 1000, 700, 90000, 2),
  ],
  geometry: circle,
});
blast.navigation_trace = {
  schema: "robot-navigation-trace/v1",
  read_only: true,
  frame_id: "shared-world",
  world_generation_id: "world-gen-1",
  local_frame_id: "blast-local",
  local_generation_id: "blast-gen",
  source_robot_id: "blast-01",
  source_controller_instance_id: "blast-controller",
  provenance: "PROVISIONAL_ENCODER_ODOMETRY + PROVISIONAL_YAW_ONLY",
  transform_provenance: ["FIXED_START_MEASUREMENT"],
  final_goal: {
    kind: "DIRECTIONAL_HEADING",
    navigation_enforced: false,
    origin_x_mm: 1000,
    origin_y_mm: 500,
    target_x_mm: 1000,
    target_y_mm: 1100,
    desired_heading_mdeg: 90000,
    minimum_forward_progress_mm: 600,
    heading_tolerance_mdeg: 5000,
    current_forward_progress_mm: 200,
    current_lateral_offset_mm: 0,
    remaining_forward_progress_mm: 400,
    goal_radius_mm: 120,
    distance_to_goal_mm: 400,
  },
  imu_heading: null,
  planned_leg: {
    kind: "SIDE_SEARCH",
    scope: "SEARCH_POSITION_ONLY",
    clearance_proven: false,
    passage_proven: false,
    route_eligible: false,
    selected_side: "LEFT",
    bind_pose: { x_mm: 1000, y_mm: 700, heading_mdeg: 90000 },
    waypoint: { x_mm: 700, y_mm: 700, heading_mdeg: -180000 },
  },
  planar_scan_views: [{
    scan_id: "shared-blast-scan",
    observed_at_unix_ms: 1950,
    scan_pose: { x_mm: 1000, y_mm: 700, heading_mdeg: 90000 },
    projection: {
      schema: "blast-planar-scan-projection/v1",
      frame: "SHARED_FIXED_START",
      local_frame: "EPISODE_LOCAL_ODOMETRY",
      quality: "PROVISIONAL_YAW_ONLY",
      vertical_pitch_compensated: false,
      ultrasonic_beam_width_modeled: false,
      scan_turn_translation_compensated: false,
      points: [{
        side: "left_near",
        measured_range_mm: 400,
        relative_bearing_mdeg: 90000,
        sensor_origin_x_mm: 1000,
        sensor_origin_y_mm: 700,
        beam_heading_mdeg: -180000,
        nominal_echo_x_mm: 600,
        nominal_echo_y_mm: 700,
      }],
    },
  }],
};
blast.object_hypotheses = [{
  hypothesis_id: "blast-ultrasonic-shared",
  classification: "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER",
  label: "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER",
  x_mm: 600, y_mm: 700,
  geometry_kind: "PROVISIONAL_ULTRASONIC_ECHO_CLUSTER",
  support_radius_mm: 35,
  support_points: [{ side: "left_near", x_mm: 600, y_mm: 700,
    measured_range_mm: 400, relative_bearing_mdeg: 90000 }],
  source_scan_ids: ["shared-blast-scan"], bearing: "LEFT",
  relation: "LEFT_OF_SCAN", evidence_count: 1, confidence_milli: 200,
  source_id: "blast-settled-measured-planar-projection",
  provenance: "SETTLED_MEASURED_ULTRASONIC + PROVISIONAL_YAW_ONLY",
  quality: "PROVISIONAL_YAW_ONLY", settled_measured_only: true,
  provisional: true, read_only: true, observed_at_unix_ms: 1950, age_ms: 0,
}];
blast.source_status = "qualitative_only";
function shared(robots) {
  return {
    schema: "robot-spatial-map/v2",
    read_only: true,
    status: "available",
    reason_code: "all_sources_available",
    map_id: "shared-world.shared-fixed-start.world-gen-1",
    frame_id: "shared-world",
    frame_kind: "SHARED_FIXED_START",
    world_generation_id: "world-gen-1",
    source_id: "shared-spatial-map-compositor",
    provenance: "CALIBRATED_FIXED_START_SE2_PROJECTION",
    snapshot_semantics: "LATEST_AVAILABLE_NOT_ATOMIC",
    robots,
    bounds: null,
    cells: [],
    sensor_rays: [],
    qualitative_observations: [],
    scan_evidence_history: [],
    object_hypotheses: [],
    navigation_authority: null,
    captured_at_unix_ms: 2000,
  };
}
function translate(key) { return key; }
const presenter = context.RobotSpatialMapPresenter.create({
  document,
  normalizeSpatialMap: context.RobotDashboardLogic.normalizeSpatialMap,
  translate,
  formatNumber: (value) => String(value),
});
const rendered = presenter.render(shared([ev3, blast]), "connected", 2100);
function descendants(root) {
  return [root, ...root.children.flatMap(descendants)];
}
const tracePathGroups = nodes["map-path-layer"].children.filter((node) => (
  String(node.attributes.class).includes("map-shared-navigation-trace")
));
const traceRayGroups = nodes["map-ray-layer"].children.filter((node) => (
  String(node.attributes.class).includes("map-shared-navigation-trace")
));
const traceNodes = tracePathGroups.flatMap(descendants);
const rayNodes = traceRayGroups.flatMap(descendants);
const goalTarget = traceNodes.find((node) => (
  node.attributes.class === "map-final-goal-target"
));
const firstRender = {
  schema: rendered.schema,
  connectionClass: nodes["map-connection-status"].className,
  frame: nodes["map-frame-label"].textContent,
  emptyHidden: nodes["map-empty-state"].hidden,
  pathGroups: nodes["map-path-layer"].children.filter((node) => (
    String(node.attributes.class).includes("map-shared-robot-path")
  )).map((node) => ({
    robotId: node.attributes["data-robot-id"],
    tags: node.children.map((child) => child.tag),
  })),
  robotGroups: nodes["map-robot-layer"].children.map((node) => ({
    robotId: node.attributes["data-robot-id"],
    generation: node.attributes["data-local-generation-id"],
    tags: node.children.map((child) => child.tag),
    classes: node.children.map((child) => child.attributes.class),
    text: node.textContent,
    bodyCx: node.children.find((child) => (
      String(child.attributes.class).includes("map-shared-body")
    )).attributes.cx,
  })),
  cellCount: nodes["map-cell-layer"].children.length,
  tracePaths: tracePathGroups.map((node) => ({
    robotId: node.attributes["data-robot-id"],
    className: node.attributes.class,
    provisional: node.attributes["data-provisional"],
    text: node.textContent,
  })),
  traceRays: traceRayGroups.map((node) => ({
    robotId: node.attributes["data-robot-id"],
    className: node.attributes.class,
    provisional: node.attributes["data-provisional"],
  })),
  goalAttributes: traceNodes.find((node) => (
    node.attributes.class === "map-final-goal"
  )).attributes,
  legAttributes: traceNodes.find((node) => (
    node.attributes.class === "map-planned-leg"
  )).attributes,
  scanAttributes: rayNodes.find((node) => (
    node.attributes.class === "map-blast-scan-view"
  )).attributes,
  echoCount: rayNodes.filter((node) => (
    node.attributes.class === "map-blast-scan-echo"
  )).length,
  fittedGoalTarget: {
    cx: Number(goalTarget.attributes.cx),
    cy: Number(goalTarget.attributes.cy),
  },
  rayCount: nodes["map-ray-layer"].children.length,
  objectCount: nodes["map-object-layer"].children.length,
  obstacleGroup: nodes["map-object-layer"].children[0].attributes,
  obstacleAttributes: nodes["map-object-layer"].children[0]
    .children[0].attributes,
  objectListText: nodes["map-object-list"].textContent,
  objectListCount: nodes["map-object-count"].textContent,
  localCount: nodes["map-local-odometry-layer"].children.length,
};
presenter.render(shared([ev3, {
  ...blast,
  navigation_trace: {
    ...blast.navigation_trace,
    local_generation_id: "stale-trace-generation",
  },
}]), "connected", 2100);
const invalidTraceRender = {
  robotIds: nodes["map-robot-layer"].children.map(
    (node) => node.attributes["data-robot-id"],
  ),
  traceRobotIds: nodes["map-path-layer"].children.filter((node) => (
    String(node.attributes.class).includes("map-shared-navigation-trace")
  )).map((node) => node.attributes["data-robot-id"]),
  rayCount: nodes["map-ray-layer"].children.length,
};
presenter.render(shared([ev3, {
  ...blast,
  frame_transform: {
    ...blast.frame_transform,
    source_generation_id: "stale-blast-gen",
  },
}]), "connected", 2100);
process.stdout.write(JSON.stringify({
  firstRender,
  invalidTraceRender,
  fencedRobotIds: nodes["map-robot-layer"].children.map(
    (node) => node.attributes["data-robot-id"],
  ),
  fencedPathIds: nodes["map-path-layer"].children.map(
    (node) => node.attributes["data-robot-id"],
  ),
  fencedLocalCount: nodes["map-local-odometry-layer"].children.length,
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "blast_map_semantics.js"),
                str(WEB_ROOT / "dashboard_logic.js"),
                str(WEB_ROOT / "spatial_map_presenter.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        result = json.loads(completed.stdout)
        rendered = result["firstRender"]
        self.assertEqual(rendered["schema"], "robot-spatial-map/v2")
        self.assertEqual(rendered["connectionClass"], "state-chip state-ready")
        self.assertEqual(rendered["frame"], "shared-world")
        self.assertTrue(rendered["emptyHidden"])
        self.assertEqual(
            [group["robotId"] for group in rendered["pathGroups"]],
            ["ev3rstorm-01", "blast-01"],
        )
        self.assertTrue(all(
            group["tags"] == ["path", "circle"]
            for group in rendered["pathGroups"]
        ))
        self.assertEqual(
            [group["robotId"] for group in rendered["robotGroups"]],
            ["ev3rstorm-01", "blast-01"],
        )
        self.assertEqual(
            [group["generation"] for group in rendered["robotGroups"]],
            ["ev3-gen", "blast-gen"],
        )
        self.assertIn("polygon", rendered["robotGroups"][0]["tags"])
        self.assertIn("circle", rendered["robotGroups"][1]["tags"])
        self.assertTrue(all(
            any("map-shared-heading" in str(class_name) for class_name in group["classes"])
            for group in rendered["robotGroups"]
        ))
        self.assertIn("ev3rstorm-01", rendered["robotGroups"][0]["text"])
        self.assertIn("blast-01", rendered["robotGroups"][1]["text"])
        self.assertNotEqual(
            rendered["robotGroups"][0]["bodyCx"],
            rendered["robotGroups"][1]["bodyCx"],
        )
        self.assertEqual(
            [group["robotId"] for group in rendered["tracePaths"]],
            ["blast-01"],
        )
        self.assertEqual(
            [group["robotId"] for group in rendered["traceRays"]],
            ["blast-01"],
        )
        self.assertIn(
            "map-shared-robot-1",
            rendered["tracePaths"][0]["className"],
        )
        self.assertIn(
            "map-shared-robot-1",
            rendered["traceRays"][0]["className"],
        )
        self.assertEqual(rendered["tracePaths"][0]["provisional"], "true")
        self.assertEqual(rendered["traceRays"][0]["provisional"], "true")
        self.assertIn(
            "map.navigation_trace.layer_label",
            rendered["tracePaths"][0]["text"],
        )
        self.assertEqual(
            rendered["goalAttributes"]["data-navigation-enforced"],
            "false",
        )
        self.assertEqual(
            rendered["legAttributes"]["data-scope"],
            "SEARCH_POSITION_ONLY",
        )
        self.assertEqual(
            rendered["legAttributes"]["data-route-eligible"],
            "false",
        )
        self.assertEqual(
            rendered["scanAttributes"]["data-projection-frame"],
            "SHARED_FIXED_START",
        )
        self.assertEqual(rendered["echoCount"], 1)
        self.assertGreaterEqual(rendered["fittedGoalTarget"]["cx"], 46)
        self.assertLessEqual(rendered["fittedGoalTarget"]["cx"], 914)
        self.assertGreaterEqual(rendered["fittedGoalTarget"]["cy"], 46)
        self.assertLessEqual(rendered["fittedGoalTarget"]["cy"], 554)
        self.assertEqual(rendered["cellCount"], 0)
        self.assertEqual(rendered["rayCount"], 1)
        self.assertEqual(rendered["objectCount"], 1)
        self.assertEqual(
            rendered["obstacleGroup"]["data-robot-id"], "blast-01"
        )
        self.assertEqual(
            rendered["obstacleAttributes"]["data-classification"],
            "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER",
        )
        self.assertEqual(
            rendered["obstacleAttributes"]["data-provisional"], "true"
        )
        self.assertIn(
            "map.objects.provisional_inference",
            rendered["objectListText"],
        )
        self.assertGreaterEqual(
            rendered["objectListText"].count("map.tooltip.source"), 2
        )
        self.assertEqual(rendered["objectListCount"], "1")
        self.assertEqual(rendered["localCount"], 0)
        self.assertEqual(
            result["invalidTraceRender"]["robotIds"],
            ["ev3rstorm-01", "blast-01"],
        )
        self.assertEqual(result["invalidTraceRender"]["traceRobotIds"], [])
        self.assertEqual(result["invalidTraceRender"]["rayCount"], 0)
        self.assertEqual(result["fencedRobotIds"], ["ev3rstorm-01"])
        self.assertEqual(result["fencedPathIds"], ["ev3rstorm-01"])
        self.assertEqual(result["fencedLocalCount"], 0)


if __name__ == "__main__":
    unittest.main()
