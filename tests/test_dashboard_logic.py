import json
from pathlib import Path
import subprocess
import unittest


LOGIC_ASSET = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "robot_agent"
    / "dashboard_web"
    / "dashboard_logic.js"
)


class DashboardLogicRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const context = {};
vm.runInNewContext(
  source,
  context,
  { filename: process.argv[1] },
);
const logic = context.RobotDashboardLogic;
if (
  !logic
  || !logic.TURN_POLL_POLICY
  || logic.SPATIAL_MAP_SCHEMA !== "robot-spatial-map/v1"
  || typeof logic.normalizeSpatialMap !== "function"
  || typeof logic.replaceRenderedItems !== "function"
  || typeof logic.transitionTurnPoll !== "function"
) {
  throw new Error("dashboard_logic.js did not expose its runtime contract");
}

function containerWith(children) {
  return {
    children: [...children],
    replacements: 0,
    replaceChildren(...next) {
      this.replacements += 1;
      this.children = next;
    },
  };
}

const emptyContainer = containerWith([{ id: "stale" }]);
const renderedWhileEmpty = [];
const normalizedEmpty = logic.replaceRenderedItems(
  emptyContainer,
  [],
  (item) => {
    renderedWhileEmpty.push(item);
    return item;
  },
);

const nonArrayContainer = containerWith([{ id: "also-stale" }]);
const normalizedNonArray = logic.replaceRenderedItems(
  nonArrayContainer,
  null,
  (item) => item,
);

const populatedContainer = containerWith([]);
const renderedIds = [];
const normalizedPopulated = logic.replaceRenderedItems(
  populatedContainer,
  [{ id: "experiment-a" }, { id: "experiment-b" }],
  (item) => {
    renderedIds.push(item.id);
    return { cardFor: item.id };
  },
);

let poll = { failures: 0, connection: "connected" };
const failures = [];
for (let index = 0; index < 9; index += 1) {
  poll = logic.transitionTurnPoll(poll, { type: "failure" });
  failures.push(poll);
}
const recoveredActive = logic.transitionTurnPoll(
  poll,
  { type: "success", turn: { status: "running" } },
);
const recoveredTerminal = logic.transitionTurnPoll(
  poll,
  { type: "success", turn: { status: "answered" } },
);
let invalidEventRejected = false;
try {
  logic.transitionTurnPoll(poll, { type: "timeout" });
} catch (error) {
  invalidEventRejected = Boolean(error && error.name === "TypeError");
}

const spatialNow = 2000;
const normalizedMap = logic.normalizeSpatialMap({
  schema: "robot-spatial-map/v1",
  read_only: true,
  status: "available",
  robot_id: "ev3rstorm-01",
  frame_id: "SIM_WORLD",
  map_version: 7,
  based_on_state_version: 19,
  based_on_world_model_version: 4,
  captured_at_unix_ms: 1800,
  source_id: "simulator",
  provenance: "SIMULATION",
  resolution_mm: 50,
  bounds: {
    min_x_mm: 0,
    min_y_mm: 0,
    max_x_mm: 1000,
    max_y_mm: 800,
    future_field: "ignored",
  },
  robot_pose: {
    x_mm: 100,
    y_mm: 200,
    heading_mdeg: 90000,
  },
  cells: [
    {
      x_mm: 150,
      y_mm: 250,
      state: "FREE",
      source_id: "occupancy",
      provenance: "PROVISIONAL_IR",
      observed_at_unix_ms: 1900,
    },
    { x_mm: 300, y_mm: 400, state: "future-state" },
    { x_mm: "bad", y_mm: 400, state: "OCCUPIED" },
  ],
  sensor_rays: [
    {
      origin_x_mm: 100,
      origin_y_mm: 200,
      end_x_mm: 500,
      end_y_mm: 200,
      observed_at_unix_ms: 1950,
      valid_until_unix_ms: 2100,
      provenance: "PROVISIONAL_IR",
    },
    {
      origin_x_mm: 100,
      origin_y_mm: 200,
      end_x_mm: 100,
      end_y_mm: 500,
      valid_until_unix_ms: 2000,
    },
  ],
  qualitative_observations: [
    {
      bearing: "FORWARD",
      relation: "NEAR_OBSTACLE",
      raw_ir_proximity: 81,
      confidence_milli: 250,
      source_id: "physical_ir_reflection",
      provenance: "PROVISIONAL_IR",
      provisional: true,
      observed_at_unix_ms: 1875,
      age_ms: 42,
    },
    {
      bearing: "FORWARD",
      raw_ir_proximity: 40,
    },
  ],
  object_hypotheses: [
    {
      hypothesis_id: "object-1",
      label: "box",
      x_mm: 500,
      y_mm: 300,
      observed_at_unix_ms: 1750,
      confidence_milli: 850,
      source_id: "vision",
      provenance: "SIMULATION",
    },
    {
      hypothesis_id: "stale",
      x_mm: 700,
      y_mm: 300,
      valid_until_unix_ms: 1999,
    },
  ],
  future_top_level: { ignored: true },
}, spatialNow);
const emptyMap = logic.normalizeSpatialMap(null, spatialNow);
const wrongSchemaMap = logic.normalizeSpatialMap({
  schema: "future-map/v9",
  read_only: true,
  cells: [{ x_mm: 1, y_mm: 2, state: "FREE" }],
}, spatialNow);
const qualitativeOnlyMap = logic.normalizeSpatialMap({
  schema: "robot-spatial-map/v1",
  read_only: true,
  status: "qualitative_only",
  frame_id: "ROBOT_BASE",
  bounds: null,
  qualitative_observations: [{
    bearing: "FORWARD",
    relation: "NO_NEAR_REFLECTION",
    raw_ir_proximity: 12,
    confidence_milli: 200,
    provisional: true,
    age_ms: 90,
  }],
}, spatialNow);

process.stdout.write(JSON.stringify({
  exports: Object.keys(logic).sort(),
  frozen: Object.isFrozen(logic),
  policy: logic.TURN_POLL_POLICY,
  policyFrozen: Object.isFrozen(logic.TURN_POLL_POLICY),
  collections: {
    empty: {
      normalizedLength: normalizedEmpty.length,
      children: emptyContainer.children,
      replacements: emptyContainer.replacements,
      renderedCount: renderedWhileEmpty.length,
    },
    nonArray: {
      normalizedLength: normalizedNonArray.length,
      children: nonArrayContainer.children,
      replacements: nonArrayContainer.replacements,
    },
    populated: {
      normalizedLength: normalizedPopulated.length,
      children: populatedContainer.children,
      replacements: populatedContainer.replacements,
      renderedIds,
    },
  },
  polling: {
    failures,
    recoveredActive,
    recoveredTerminal,
    invalidEventRejected,
  },
  spatial: {
    normalizedMap,
    normalizedFrozen: Object.isFrozen(normalizedMap),
    cellsFrozen: Object.isFrozen(normalizedMap.cells),
    qualitativeFrozen: Object.isFrozen(
      normalizedMap.qualitativeObservations,
    ),
    emptyMap,
    wrongSchemaMap,
    qualitativeOnlyMap,
  },
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(LOGIC_ASSET),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        if completed.returncode != 0:
            raise AssertionError(completed.stderr)
        cls.runtime = json.loads(completed.stdout)

    def test_experiment_collection_replaces_stale_content(self):
        collections = self.runtime["collections"]
        self.assertEqual(
            collections["empty"],
            {
                "normalizedLength": 0,
                "children": [],
                "replacements": 1,
                "renderedCount": 0,
            },
        )
        self.assertEqual(
            collections["nonArray"],
            {
                "normalizedLength": 0,
                "children": [],
                "replacements": 1,
            },
        )
        self.assertEqual(
            collections["populated"],
            {
                "normalizedLength": 2,
                "children": [
                    {"cardFor": "experiment-a"},
                    {"cardFor": "experiment-b"},
                ],
                "replacements": 1,
                "renderedIds": ["experiment-a", "experiment-b"],
            },
        )

    def test_turn_polling_keeps_unknown_connection_nonterminal(self):
        self.assertEqual(
            self.runtime["exports"],
            [
                "SPATIAL_MAP_SCHEMA",
                "TURN_POLL_POLICY",
                "normalizeSpatialMap",
                "replaceRenderedItems",
                "transitionTurnPoll",
            ],
        )
        self.assertTrue(self.runtime["frozen"])
        self.assertTrue(self.runtime["policyFrozen"])
        self.assertEqual(
            self.runtime["policy"],
            {
                "unknownAfterFailures": 8,
                "baseDelayMs": 800,
                "maxDelayMs": 5000,
            },
        )
        polling = self.runtime["polling"]
        failures = polling["failures"]
        self.assertEqual(len(failures), 9)
        self.assertEqual(
            failures[0],
            {
                "failures": 1,
                "connection": "retrying",
                "becameUnknown": False,
                "recovered": False,
                "terminal": False,
                "retry": True,
                "retryDelayMs": 800,
            },
        )
        self.assertEqual(failures[6]["retryDelayMs"], 5000)
        self.assertEqual(
            failures[7],
            {
                "failures": 8,
                "connection": "unknown",
                "becameUnknown": True,
                "recovered": False,
                "terminal": False,
                "retry": True,
                "retryDelayMs": 5000,
            },
        )
        self.assertEqual(failures[8]["connection"], "unknown")
        self.assertFalse(failures[8]["becameUnknown"])
        self.assertTrue(failures[8]["retry"])
        self.assertNotIn("turn", failures[8])
        self.assertEqual(
            polling["recoveredActive"],
            {
                "failures": 0,
                "connection": "connected",
                "becameUnknown": False,
                "recovered": True,
                "terminal": False,
                "retry": False,
                "retryDelayMs": None,
            },
        )
        self.assertTrue(polling["recoveredTerminal"]["recovered"])
        self.assertTrue(polling["recoveredTerminal"]["terminal"])
        self.assertFalse(polling["recoveredTerminal"]["retry"])
        self.assertTrue(polling["invalidEventRejected"])

    def test_spatial_map_normalization_is_bounded_fresh_and_defensive(self):
        spatial = self.runtime["spatial"]
        normalized = spatial["normalizedMap"]

        self.assertTrue(spatial["normalizedFrozen"])
        self.assertTrue(spatial["cellsFrozen"])
        self.assertTrue(spatial["qualitativeFrozen"])
        self.assertTrue(normalized["contractValid"])
        self.assertEqual(normalized["schema"], "robot-spatial-map/v1")
        self.assertEqual(normalized["ageMs"], 200)
        self.assertEqual(normalized["basedOnStateVersion"], 19)
        self.assertEqual(normalized["basedOnWorldModelVersion"], 4)
        self.assertEqual(normalized["provenance"], "SIMULATION")
        self.assertEqual(
            normalized["bounds"],
            {
                "minX": 0,
                "minY": 0,
                "maxX": 1000,
                "maxY": 800,
            },
        )
        self.assertEqual(len(normalized["cells"]), 2)
        self.assertEqual(normalized["cells"][0]["state"], "FREE")
        self.assertEqual(
            normalized["cells"][0]["provenance"],
            "PROVISIONAL IR",
        )
        self.assertEqual(normalized["cells"][0]["ageMs"], 100)
        self.assertEqual(normalized["cells"][1]["state"], "UNKNOWN")
        self.assertEqual(len(normalized["sensorRays"]), 1)
        self.assertEqual(
            normalized["sensorRays"][0]["validUntilUnixMs"],
            2100,
        )
        self.assertEqual(len(normalized["objectHypotheses"]), 1)
        self.assertEqual(
            normalized["objectHypotheses"][0]["hypothesisId"],
            "object-1",
        )
        self.assertEqual(normalized["robotPose"]["ageMs"], 200)
        self.assertEqual(len(normalized["qualitativeObservations"]), 1)
        self.assertEqual(
            normalized["qualitativeObservations"][0],
            {
                "bearing": "FORWARD",
                "relation": "NEAR_OBSTACLE",
                "rawIrProximity": 81,
                "confidenceMilli": 250,
                "sourceId": "physical_ir_reflection",
                "provenance": "PROVISIONAL IR",
                "provisional": True,
                "ageMs": 125,
            },
        )

        empty = spatial["emptyMap"]
        self.assertFalse(empty["contractValid"])
        self.assertEqual(empty["cells"], [])
        self.assertEqual(empty["sensorRays"], [])
        self.assertEqual(empty["objectHypotheses"], [])
        self.assertEqual(empty["qualitativeObservations"], [])

        wrong = spatial["wrongSchemaMap"]
        self.assertFalse(wrong["contractValid"])
        self.assertEqual(wrong["cells"], [])
        self.assertEqual(wrong["qualitativeObservations"], [])

        qualitative = spatial["qualitativeOnlyMap"]
        self.assertTrue(qualitative["contractValid"])
        self.assertEqual(qualitative["status"], "qualitative_only")
        self.assertIsNone(qualitative["bounds"])
        self.assertEqual(len(qualitative["qualitativeObservations"]), 1)
        self.assertEqual(
            qualitative["qualitativeObservations"][0]["ageMs"],
            90,
        )


if __name__ == "__main__":
    unittest.main()
