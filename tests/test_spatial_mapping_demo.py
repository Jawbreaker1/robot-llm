import json
from pathlib import Path
import subprocess
import sys
import unittest

from robot_agent.spatial_mapping_demo import (
    build_simulation_map_demo,
)


class SpatialMappingDemoTests(unittest.TestCase):
    def test_demo_navigation_builds_an_honest_stopped_map(self):
        result, plant, runtime = build_simulation_map_demo()
        try:
            snapshot = runtime.raw_snapshot()
            view = runtime.snapshot()

            self.assertTrue(result.completed)
            self.assertTrue(result.terminal_stop_verified)
            self.assertFalse(result.final_snapshot.motors_running)
            self.assertEqual(plant.collision_count, 0)
            self.assertEqual(
                snapshot.based_on_state_version,
                result.final_snapshot.state_version,
            )
            self.assertEqual(
                runtime.state().ignored_updates,
                0,
            )
            self.assertEqual(view["status"], "available")
            self.assertEqual(
                view["map_quality"],
                "SIMULATION_METRIC",
            )
            self.assertEqual(
                view["provenance"],
                "SIMULATION",
            )
            self.assertTrue(view["cells"])
            self.assertTrue(view["object_hypotheses"])
            self.assertTrue(any(
                item["trusted_simulator_object_id"] == "demo-box"
                for item in view["object_hypotheses"]
            ))
        finally:
            runtime.close()

    def test_compact_cli_report_is_simulation_only(self):
        completed = subprocess.run(
            [
                sys.executable,
                "-m",
                "robot_agent.spatial_mapping_demo",
            ],
            check=False,
            capture_output=True,
            text=True,
            env={
                "PYTHONPATH": "src",
                "PYTHONDONTWRITEBYTECODE": "1",
            },
            timeout=10,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        report = json.loads(completed.stdout)
        self.assertTrue(report["simulation_only"])
        self.assertTrue(report["navigation"]["completed"])
        self.assertEqual(report["navigation"]["collision_count"], 0)
        self.assertGreater(report["map"]["cell_count"], 0)
        self.assertGreater(
            report["map"]["object_hypothesis_count"],
            0,
        )

    def test_real_map_view_normalizes_in_dashboard_javascript(self):
        _result, _plant, runtime = build_simulation_map_demo()
        try:
            view = runtime.snapshot()
        finally:
            runtime.close()
        logic_path = (
            Path(__file__).resolve().parents[1]
            / "src"
            / "robot_agent"
            / "dashboard_web"
            / "dashboard_logic.js"
        )
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const context = {};
vm.runInNewContext(source, context, { filename: process.argv[1] });
const input = JSON.parse(fs.readFileSync(0, "utf8"));
const map = context.RobotDashboardLogic.normalizeSpatialMap(
  input,
  Date.now(),
);
process.stdout.write(JSON.stringify({
  contractValid: map.contractValid,
  status: map.status,
  cells: map.cells.length,
  rays: map.sensorRays.length,
  objects: map.objectHypotheses.length,
  pose: Boolean(map.robotPose),
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(logic_path),
            ],
            input=json.dumps(view),
            capture_output=True,
            text=True,
            check=False,
            timeout=10,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        normalized = json.loads(completed.stdout)
        self.assertTrue(normalized["contractValid"])
        self.assertEqual(normalized["status"], "available")
        self.assertGreater(normalized["cells"], 0)
        self.assertEqual(normalized["rays"], 3)
        self.assertGreater(normalized["objects"], 0)
        self.assertTrue(normalized["pose"])


if __name__ == "__main__":
    unittest.main()
