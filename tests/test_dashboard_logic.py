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
  || typeof logic.createDashboardRequest !== "function"
  || typeof logic.createSessionGuard !== "function"
  || typeof logic.isTerminalSessionError !== "function"
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
  frame_id: "local-odometry",
  bounds: null,
  object_hypotheses: [{
    hypothesis_id: "provisional-object-1",
    label: "UNKNOWN",
    x_mm: 999,
    y_mm: 999,
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
    age_ms: 90,
  }],
  qualitative_observations: [{
    bearing: "FORWARD",
    relation: "NO_NEAR_REFLECTION",
    raw_ir_proximity: 12,
    confidence_milli: 200,
    provisional: true,
    age_ms: 90,
  }],
}, spatialNow);
const physicalEvidenceMap = logic.normalizeSpatialMap({
  schema: "robot-spatial-map/v1",
  read_only: true,
  status: "qualitative_only",
  frame_id: "local-odometry",
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
  qualitative_observations_evicted: 23,
  scan_evidence_history_evicted: 7,
  hazard_retention: {
    capacity: 64,
    retained_count: 12,
    evicted_count: 3,
    last_eviction_reason: "MAP_CAPACITY_OLDEST_HAZARD",
  },
  scan_attempt_retention: {
    per_hazard_capacity: 16,
    map_capacity: 64,
    retained_count: 41,
    evicted_count: 9,
    last_eviction_reason: "MAP_CAPACITY_OLDEST_ATTEMPT",
  },
  scan_evidence_history: [{
    target_hypothesis_id: "hazard-1",
    frame_id: "local-odometry",
    hypothesis_anchor_pose: {
      x_mm: 10,
      y_mm: 20,
      heading_mdeg: 30000,
    },
    scan_pose: {
      x_mm: 80,
      y_mm: -35,
      heading_mdeg: 45000,
    },
    based_on_map_version: 3,
    scan_id: "scan-with-pose",
    completed_at_unix_ms: 1900,
    status: "CANCELLED",
    reason: "bilateral_boundaries_not_observed",
    bearing_convention: "POSITIVE_LEFT_NEGATIVE_RIGHT",
    geometry_kind: "ANGULAR_NONMETRIC_IR_SCAN",
    observation_pattern: "MIXED",
    arc_coverage: "BILATERAL_ARC",
    boundary_coverage: "NO_BOUNDARIES",
    hypothesis_relation: "SUPPORTS_BLOCKED_HYPOTHESIS",
    left_boundary_mdeg: null,
    right_boundary_mdeg: null,
    provisional: true,
    read_only: true,
    rays: [{
      requested_relative_bearing_mdeg: -30000,
      actual_relative_bearing_mdeg: -28500,
      blocked: true,
      raw_ir_proximity: 24,
      filtered_ir_proximity: 25,
      measured_distance_mm: 999,
    }, {
      requested_relative_bearing_mdeg: 30000,
      actual_relative_bearing_mdeg: 31500,
      blocked: false,
      raw_ir_proximity: 56,
      filtered_ir_proximity: 55,
    }],
  }, {
    target_hypothesis_id: "hazard-1",
    frame_id: "local-odometry",
    hypothesis_anchor_pose: {
      x_mm: 10,
      y_mm: 20,
      heading_mdeg: 30000,
    },
    scan_pose: null,
    based_on_map_version: null,
    scan_id: "legacy-scan-without-pose",
    completed_at_unix_ms: 1800,
    status: "CANCELLED",
    reason: "bilateral_boundaries_not_observed",
    bearing_convention: "POSITIVE_LEFT_NEGATIVE_RIGHT",
    geometry_kind: "ANGULAR_NONMETRIC_IR_SCAN",
    observation_pattern: "ALL_CLEAR",
    arc_coverage: "BILATERAL_ARC",
    boundary_coverage: "NO_BOUNDARIES",
    hypothesis_relation: "CONFLICTS_BLOCKED_HYPOTHESIS",
    left_boundary_mdeg: null,
    right_boundary_mdeg: null,
    provisional: true,
    read_only: true,
    rays: [],
  }],
}, spatialNow);

process.stdout.write(JSON.stringify({
  exports: Object.keys(logic).sort(),
  limits: {
    poseHistory: logic.MAX_POSE_HISTORY,
    qualitativeObservations: logic.MAX_QUALITATIVE_OBSERVATIONS,
    scanEvidence: logic.MAX_SCAN_EVIDENCE,
  },
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
    physicalEvidenceMap,
    physicalEvidenceFrozen: (
      Object.isFrozen(physicalEvidenceMap.collisionGeometry)
      && Object.isFrozen(physicalEvidenceMap.scanEvidenceHistory)
      && Object.isFrozen(
        physicalEvidenceMap.scanEvidenceHistory[0].rays,
      )
    ),
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
                "MAX_POSE_HISTORY",
                "MAX_QUALITATIVE_OBSERVATIONS",
                "MAX_SCAN_EVIDENCE",
                "SPATIAL_MAP_SCHEMA",
                "TURN_POLL_POLICY",
                "createDashboardRequest",
                "createSessionGuard",
                "isTerminalSessionError",
                "normalizeSpatialMap",
                "replaceRenderedItems",
                "transitionTurnPoll",
            ],
        )
        self.assertEqual(
            self.runtime["limits"],
            {
                "poseHistory": 2_048,
                "qualitativeObservations": 1_024,
                "scanEvidence": 64,
            },
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

    def test_rejected_session_is_a_terminal_request_circuit_breaker(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const context = {};
vm.runInNewContext(source, context, { filename: process.argv[1] });
const logic = context.RobotDashboardLogic;

class FakeAbortController {
  constructor() {
    this.signal = {
      aborted: false,
      addEventListener() {},
      removeEventListener() {},
    };
  }
  abort() {
    this.signal.aborted = true;
  }
}

function response(status, payload) {
  return {
    ok: status >= 200 && status < 300,
    status,
    async text() {
      return JSON.stringify(payload);
    },
  };
}

(async () => {
  const replies = [
    response(500, { error: { code: "server_busy" } }),
    response(200, { ready: true }),
    response(403, { error: { code: "origin_rejected" } }),
    new Error("temporary network failure"),
    response(403, {
      error: { code: "session_token_rejected" },
    }),
  ];
  const calls = [];
  const guard = logic.createSessionGuard();
  let expiryAnnouncements = 0;
  guard.subscribe(() => {
    expiryAnnouncements += 1;
  });
  const request = logic.createDashboardRequest({
    sessionToken: "a".repeat(64),
    sessionGuard: guard,
    fetchRequest: async (path, options) => {
      calls.push({ path, headers: options.headers });
      const reply = replies.shift();
      if (reply instanceof Error) {
        throw reply;
      }
      return reply;
    },
    AbortController: FakeAbortController,
    setTimeout: () => 1,
    clearTimeout() {},
  });
  const errors = [];
  for (let index = 0; index < 5; index += 1) {
    try {
      await request("/api/v1/bootstrap");
    } catch (error) {
      errors.push({ code: error.code, status: error.status });
    }
  }
  const fetchesAtExpiry = calls.length;
  for (let index = 0; index < 100; index += 1) {
    try {
      await request("/api/v1/map");
    } catch (error) {
      if (
        error.code !== "session_token_rejected"
        || error.status !== 403
      ) {
        throw error;
      }
    }
  }
  let lateAnnouncement = 0;
  guard.subscribe(() => {
    lateAnnouncement += 1;
  });
  process.stdout.write(JSON.stringify({
    calls: calls.length,
    errors,
    expired: guard.isExpired(),
    expiryAnnouncements,
    fetchesAtExpiry,
    frozen: Object.isFrozen(guard),
    lateAnnouncement,
    terminalChecks: {
      exact: logic.isTerminalSessionError({
        code: "session_token_rejected",
        status: 403,
      }),
      wrongCode: logic.isTerminalSessionError({
        code: "origin_rejected",
        status: 403,
      }),
      wrongStatus: logic.isTerminalSessionError({
        code: "session_token_rejected",
        status: 401,
      }),
    },
    tokenHeader: calls[0].headers["X-Robot-Dashboard-Token"],
  }));
})().catch((error) => {
  process.stderr.write(String(error && error.stack || error));
  process.exitCode = 1;
});
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
        self.assertEqual(completed.returncode, 0, completed.stderr)
        result = json.loads(completed.stdout)
        self.assertEqual(result["calls"], 5)
        self.assertEqual(result["fetchesAtExpiry"], 5)
        self.assertTrue(result["expired"])
        self.assertTrue(result["frozen"])
        self.assertEqual(result["expiryAnnouncements"], 1)
        self.assertEqual(result["lateAnnouncement"], 1)
        self.assertEqual(
            result["errors"],
            [
                {"code": "server_busy", "status": 500},
                {"code": "origin_rejected", "status": 403},
                {"code": "network_error", "status": None},
                {
                    "code": "session_token_rejected",
                    "status": 403,
                },
            ],
        )
        self.assertEqual(result["tokenHeader"], "a" * 64)
        self.assertEqual(
            result["terminalChecks"],
            {"exact": True, "wrongCode": False, "wrongStatus": False},
        )

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
        self.assertEqual(len(qualitative["objectHypotheses"]), 1)
        hypothesis = qualitative["objectHypotheses"][0]
        self.assertEqual(
            hypothesis["hypothesisId"],
            "provisional-object-1",
        )
        self.assertTrue(hypothesis["provisional"])
        self.assertIsNone(hypothesis["xMm"])
        self.assertIsNone(hypothesis["yMm"])
        self.assertEqual(
            hypothesis["geometryKind"],
            "QUALITATIVE_FORWARD_ENVELOPE",
        )
        self.assertEqual(
            hypothesis["anchorPose"],
            {
                "xMm": 10,
                "yMm": 20,
                "headingMdeg": 60000,
            },
        )
        self.assertEqual(
            qualitative["qualitativeObservations"][0]["ageMs"],
            90,
        )

        physical = spatial["physicalEvidenceMap"]
        self.assertTrue(spatial["physicalEvidenceFrozen"])
        self.assertEqual(
            physical["collisionGeometry"]["geometry"],
            "ASYMMETRIC_RECTANGLE",
        )
        self.assertEqual(
            physical["collisionGeometry"]["rightExtentMm"],
            160,
        )
        self.assertGreater(
            physical["collisionGeometry"]["rightExtentMm"],
            physical["collisionGeometry"]["leftExtentMm"],
        )
        self.assertEqual(len(physical["scanEvidenceHistory"]), 2)
        self.assertEqual(physical["scanEvidenceHistoryEvicted"], 7)
        self.assertEqual(physical["qualitativeObservationsEvicted"], 23)
        self.assertEqual(physical["hazardRetention"], {
            "capacity": 64,
            "retainedCount": 12,
            "evictedCount": 3,
            "lastEvictionReason": "MAP_CAPACITY_OLDEST_HAZARD",
        })
        self.assertEqual(physical["scanAttemptRetention"], {
            "perHazardCapacity": 16,
            "mapCapacity": 64,
            "retainedCount": 41,
            "evictedCount": 9,
            "lastEvictionReason": "MAP_CAPACITY_OLDEST_ATTEMPT",
        })
        current, legacy = physical["scanEvidenceHistory"]
        self.assertTrue(current["spatiallyRenderable"])
        self.assertEqual(current["scanPose"], {
            "xMm": 80,
            "yMm": -35,
            "headingMdeg": 45000,
        })
        self.assertNotEqual(
            current["scanPose"],
            current["hypothesisAnchorPose"],
        )
        self.assertEqual(current["ageMs"], 100)
        self.assertNotIn("measuredDistanceMm", current["rays"][0])
        self.assertFalse(legacy["spatiallyRenderable"])
        self.assertIsNone(legacy["scanPose"])

    def test_blast_navigation_trace_normalizes_enforced_local_detours(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const context = {};
vm.runInNewContext(source, context, { filename: process.argv[1] });
const logic = context.RobotDashboardLogic;

const goal = {
  kind: "DIRECTIONAL_HEADING",
  navigation_enforced: false,
  origin_x_mm: 0,
  origin_y_mm: 0,
  target_x_mm: 420,
  target_y_mm: 0,
  desired_heading_mdeg: 0,
  minimum_forward_progress_mm: 420,
  heading_tolerance_mdeg: 5000,
  current_forward_progress_mm: 45,
  remaining_forward_progress_mm: 375,
};
const sideSearch = {
  kind: "SIDE_SEARCH",
  scope: "SEARCH_POSITION_ONLY",
  clearance_proven: false,
  passage_proven: false,
  route_eligible: false,
  selected_side: "LEFT",
  bind_pose: { x_mm: 45, y_mm: 0, heading_mdeg: 0 },
  waypoint: { x_mm: 45, y_mm: 210, heading_mdeg: 90000 },
};
const localDetour = {
  ...sideSearch,
  kind: "PASS_BEYOND_TARGET",
  scope: "LOCAL_DETOUR_ROUTE",
  route_eligible: true,
};
function mapFor(finalGoal, plannedLeg) {
  return {
    schema: "robot-spatial-map/v1",
    read_only: true,
    status: "pose_only",
    frame_id: "episode-a-local-odometry",
    frame_kind: "LOCAL_ODOMETRY",
    bounds: null,
    cells: [],
    sensor_rays: [],
    qualitative_observations: [],
    object_hypotheses: [],
    scan_evidence_history: [],
    navigation_trace: {
      schema: "robot-navigation-trace/v1",
      read_only: true,
      frame_id: "episode-a-local-odometry",
      provenance: "PROVISIONAL_ENCODER_ODOMETRY + PROVISIONAL_YAW_ONLY",
      final_goal: finalGoal,
      planned_leg: plannedLeg,
      imu_heading: null,
      planar_scan_views: [],
    },
  };
}
const reference = logic.normalizeSpatialMap(mapFor(goal, sideSearch), 2000)
  .navigationTrace;
const enforced = logic.normalizeSpatialMap(mapFor(
  { ...goal, navigation_enforced: true },
  localDetour,
), 2000).navigationTrace;
const invalidClaim = logic.normalizeSpatialMap(mapFor(
  { ...goal, navigation_enforced: true },
  { ...localDetour, clearance_proven: true },
), 2000).navigationTrace;
const invalidKind = logic.normalizeSpatialMap(mapFor(
  { ...goal, navigation_enforced: true },
  { ...localDetour, kind: "SIDE_SEARCH" },
), 2000).navigationTrace;
const invalidEnforcement = logic.normalizeSpatialMap(mapFor(
  { ...goal, navigation_enforced: "true" },
  localDetour,
), 2000).navigationTrace;
const mismatchedReference = logic.normalizeSpatialMap(mapFor(
  goal,
  localDetour,
), 2000).navigationTrace;
const mismatchedEnforced = logic.normalizeSpatialMap(mapFor(
  { ...goal, navigation_enforced: true },
  sideSearch,
), 2000).navigationTrace;

process.stdout.write(JSON.stringify({
  reference: {
    navigationEnforced: reference.finalGoal.navigationEnforced,
    plannedLeg: reference.plannedLeg,
  },
  enforced: {
    navigationEnforced: enforced.finalGoal.navigationEnforced,
    plannedLeg: enforced.plannedLeg,
    frozen: Object.isFrozen(enforced.plannedLeg),
  },
  invalidClaim,
  invalidKind,
  invalidEnforcement,
  mismatchedReference,
  mismatchedEnforced,
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
        self.assertEqual(completed.returncode, 0, completed.stderr)
        result = json.loads(completed.stdout)
        self.assertFalse(result["reference"]["navigationEnforced"])
        self.assertEqual(
            result["reference"]["plannedLeg"]["scope"],
            "SEARCH_POSITION_ONLY",
        )
        self.assertFalse(
            result["reference"]["plannedLeg"]["routeEligible"]
        )
        self.assertTrue(result["enforced"]["navigationEnforced"])
        self.assertEqual(
            result["enforced"]["plannedLeg"]["kind"],
            "PASS_BEYOND_TARGET",
        )
        self.assertEqual(
            result["enforced"]["plannedLeg"]["scope"],
            "LOCAL_DETOUR_ROUTE",
        )
        self.assertTrue(
            result["enforced"]["plannedLeg"]["routeEligible"]
        )
        self.assertFalse(
            result["enforced"]["plannedLeg"]["clearanceProven"]
        )
        self.assertFalse(
            result["enforced"]["plannedLeg"]["passageProven"]
        )
        self.assertTrue(result["enforced"]["frozen"])
        self.assertIsNone(result["invalidClaim"])
        self.assertIsNone(result["invalidKind"])
        self.assertIsNone(result["invalidEnforcement"])
        self.assertIsNone(result["mismatchedReference"])
        self.assertIsNone(result["mismatchedEnforced"])


if __name__ == "__main__":
    unittest.main()
