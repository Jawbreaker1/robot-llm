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
  "map.local_odometry.nonmetric": "No metric IR distance",
  "map.local_odometry.robot_title": "Provisional robot pose",
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
  "map.qualitative.relation.near_obstacle": "Near reflection",
  "map.qualitative.relation.no_near_reflection": "No near reflection",
  "map.qualitative.relation.unknown": "Unknown relation",
  "map.read_only": "Read only",
  "map.status.degraded": "Degraded",
  "map.status.empty": "No map",
  "map.status.invalid": "Invalid",
  "map.status.live": "Live",
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
  qualitative_observations: [{
    bearing: "FORWARD",
    relation: "NEAR_OBSTACLE",
    raw_ir_proximity: 81,
    confidence_milli: 250,
    source_id: "physical_ir_reflection",
    provenance: "PROVISIONAL_IR",
    provisional: true,
    observed_at_unix_ms: 1900,
    age_ms: 1,
  }],
}, "connected", 2000);
const qualitativeResult = {
  status: qualitative.status,
  age: qualitative.qualitativeObservations[0].ageMs,
  connection: nodes["map-connection-status"].textContent,
  emptyHidden: nodes["map-empty-state"].hidden,
  emptyTitle: nodes["map-empty-title"].textContent,
  emptyBody: nodes["map-empty-body"].textContent,
  count: nodes["map-qualitative-count"].textContent,
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
  localTags: nodes["map-local-odometry-layer"].children
    .map((node) => node.tag),
  robotAttributes: nodes["map-local-odometry-layer"]
    .children[2].attributes,
  robotTags: nodes["map-local-odometry-layer"]
    .children[2].children.map((node) => node.tag),
  headingLength: (() => {
    const heading = nodes["map-local-odometry-layer"]
      .children[2].children[2].attributes;
    return Math.hypot(
      Number(heading.x2) - Number(heading.x1),
      Number(heading.y2) - Number(heading.y1),
    );
  })(),
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
        self.assertEqual(qualitative["count"], "1")
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
        self.assertEqual(pose["localTags"], ["text", "text", "g"])
        self.assertEqual(
            pose["robotAttributes"]["data-provisional"],
            "true",
        )
        self.assertEqual(
            pose["robotTags"],
            ["title", "circle", "line", "circle"],
        )
        self.assertAlmostEqual(pose["headingLength"], 58)

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
        self.assertEqual(metric["localLayerCount"], 0)
        self.assertIn("State version19", metric["metadataText"])
        self.assertIn("World version4", metric["metadataText"])
        self.assertTrue(result["invalidDependenciesRejected"])


if __name__ == "__main__":
    unittest.main()
