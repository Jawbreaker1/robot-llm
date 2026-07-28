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
                "TURN_POLL_POLICY",
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


if __name__ == "__main__":
    unittest.main()
