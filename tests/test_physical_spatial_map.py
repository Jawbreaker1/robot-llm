import json
from pathlib import Path
import tempfile
import unittest

from robot_agent.navigation_memory_store import NavigationMemoryStore
from robot_agent.physical_odometry import DriveMotorRoles, PhysicalPose
from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge
from robot_agent.provisional_hazard_map import ProvisionalHazardMap


def observation(
    version=1,
    *,
    blocked=False,
    raw=60,
    left=10,
    right=20,
):
    return {
        "state_version": version,
        "observed_monotonic_ms": version * 10,
        "touch": {"value0": 0, "pressed": False},
        "infrared": {
            "raw": raw,
            "filtered": raw,
            "blocked": blocked,
            "reason": "blocked" if blocked else "clear",
            "sample_count": 5,
        },
        "motors": [
            {"role": "left_drive", "position": left, "state": ""},
            {"role": "right_drive", "position": right, "state": ""},
        ],
        "last_outcome": {"kind": "observe", "status": "completed"},
        "budgets": {
            "pulse_count": 0,
            "pulse_count_remaining": 40,
            "pulse_duration_ms": 0,
            "pulse_duration_ms_remaining": 32_000,
            "process_ms_remaining": 40_000,
            "motion_fault_latched": False,
        },
    }


class FixedClock:
    def __init__(self, value):
        self.value = value

    def __call__(self):
        return self.value


class PhysicalSpatialMapBridgeTests(unittest.TestCase):
    def memory(
        self,
        directory,
        *,
        generation="generation-1",
        frame="ev3-local-1",
        pose=PhysicalPose(x_mm=25, y_mm=-10, heading_mdeg=30_000),
        localization_valid=True,
        map_revision=0,
    ):
        return NavigationMemoryStore(
            path=Path(directory) / "memory.json",
            robot_id="ev3rstorm-01",
            controller_instance_id="ev3-main",
            frame_id=frame,
            generation_id=generation,
            pose=pose,
            hazard_map=ProvisionalHazardMap(
                frame_id=frame,
                map_generation_id=generation,
                revision=map_revision,
            ),
            drive_roles=DriveMotorRoles(),
            localization_valid=localization_valid,
            localization_error=(
                None if localization_valid else "encoder anchor lost"
            ),
        )

    def test_projects_authoritative_hazard_without_metric_claims(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory)
            blocked = observation(blocked=True, raw=20)
            hazard = memory.hazard_map.record_observation(
                memory.pose,
                blocked,
                1_000,
            )
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(1_100),
            )

            self.assertTrue(bridge.offer(
                memory=memory,
                observation=blocked,
                episode_id="episode-1",
                captured_at_ms=1_000,
            ))
            snapshot = bridge.snapshot()

        self.assertEqual(snapshot["schema"], "robot-spatial-map/v1")
        self.assertTrue(snapshot["read_only"])
        self.assertEqual(snapshot["status"], "qualitative_only")
        self.assertEqual(snapshot["frame_kind"], "LOCAL_ODOMETRY")
        self.assertEqual(
            snapshot["robot_pose"],
            {
                "x_mm": 25,
                "y_mm": -10,
                "heading_mdeg": 30_000,
                "frame_id": "ev3-local-1",
                "state_version": 1,
                "source_id": "navigation-pose",
                "provenance": "LOCAL_ODOMETRY",
                "observed_at_unix_ms": 1_000,
                "age_ms": 100,
            },
        )
        self.assertEqual(snapshot["cells"], [])
        self.assertIsNone(snapshot["bounds"])
        self.assertEqual(len(snapshot["sensor_rays"]), 1)
        ray = snapshot["sensor_rays"][0]
        for field in (
            "origin_x_mm",
            "origin_y_mm",
            "end_x_mm",
            "end_y_mm",
        ):
            self.assertIsNone(ray[field])
        self.assertEqual(len(snapshot["object_hypotheses"]), 1)
        projected = snapshot["object_hypotheses"][0]
        self.assertEqual(projected["hypothesis_id"], hazard.hypothesis_id)
        self.assertEqual(
            projected["geometry_kind"],
            "QUALITATIVE_FORWARD_ENVELOPE",
        )
        self.assertIsNone(projected["x_mm"])
        self.assertIsNone(projected["y_mm"])
        self.assertNotIn("simulation", json.dumps(snapshot).lower())

    def test_bridge_versions_survive_worker_reset_and_generation_swap(self):
        with tempfile.TemporaryDirectory() as directory:
            first = self.memory(directory)
            bridge = PhysicalSpatialMapBridge(
                robot_id=first.robot_id,
                controller_instance_id=first.controller_instance_id,
                clock_ms=FixedClock(2_000),
            )
            self.assertTrue(bridge.offer(
                memory=first,
                observation=observation(version=9),
                episode_id="episode-1",
                captured_at_ms=1_000,
            ))
            first_snapshot = bridge.snapshot()
            first.hazard_map.record_observation(
                first.pose,
                observation(version=1),
                1_100,
            )
            self.assertTrue(bridge.offer(
                memory=first,
                observation=observation(version=1),
                episode_id="episode-1",
                captured_at_ms=1_100,
            ))
            renewed = bridge.snapshot()
            second = self.memory(
                directory,
                generation="generation-2",
                frame="ev3-local-2",
            )
            self.assertTrue(bridge.offer(
                memory=second,
                observation=observation(version=1),
                episode_id="episode-2",
                captured_at_ms=1_200,
            ))
            reset = bridge.snapshot()

        self.assertEqual(first_snapshot["based_on_state_version"], 1)
        self.assertEqual(renewed["based_on_state_version"], 2)
        self.assertEqual(reset["based_on_state_version"], 3)
        self.assertEqual(first_snapshot["based_on_world_model_version"], 1)
        self.assertEqual(renewed["based_on_world_model_version"], 1)
        self.assertEqual(reset["based_on_world_model_version"], 2)
        self.assertEqual(reset["frame_id"], "ev3-local-2")
        self.assertEqual(reset["object_hypotheses"], [])

    def test_rejects_non_increasing_same_generation_map_revisions(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory)
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(2_000),
            )
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=1, raw=60),
                episode_id="episode-1",
                captured_at_ms=1_000,
            ))
            first = bridge.snapshot()

            self.assertFalse(bridge.offer(
                memory=memory,
                observation=observation(
                    version=2,
                    blocked=True,
                    raw=20,
                ),
                episode_id="episode-1",
                captured_at_ms=1_100,
            ))
            self.assertEqual(bridge.snapshot(), first)

            memory.hazard_map.record_observation(
                memory.pose,
                observation(version=2, raw=55),
                1_100,
            )
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=2, raw=55),
                episode_id="episode-1",
                captured_at_ms=1_100,
            ))
            second = bridge.snapshot()
            stale = self.memory(
                directory,
                generation=memory.generation_id,
                frame=memory.frame_id,
                map_revision=0,
            )
            self.assertFalse(bridge.offer(
                memory=stale,
                observation=observation(version=99, raw=5),
                episode_id="episode-1",
                captured_at_ms=1_200,
            ))

        self.assertEqual(bridge.snapshot(), second)
        self.assertEqual(second["map_version"], 1)
        self.assertEqual(second["based_on_state_version"], 2)

    def test_rejects_retired_generation_even_with_a_newer_revision(self):
        with tempfile.TemporaryDirectory() as directory:
            first = self.memory(
                directory,
                generation="generation-1",
                frame="ev3-local-1",
            )
            bridge = PhysicalSpatialMapBridge(
                robot_id=first.robot_id,
                controller_instance_id=first.controller_instance_id,
                clock_ms=FixedClock(2_000),
            )
            self.assertTrue(bridge.offer(
                memory=first,
                observation=observation(version=1),
                episode_id="episode-1",
                captured_at_ms=1_000,
            ))
            second = self.memory(
                directory,
                generation="generation-2",
                frame="ev3-local-2",
            )
            self.assertTrue(bridge.offer(
                memory=second,
                observation=observation(version=1),
                episode_id="episode-2",
                captured_at_ms=1_100,
            ))
            current = bridge.snapshot()
            retired = self.memory(
                directory,
                generation="generation-1",
                frame="ev3-local-1",
                map_revision=50,
            )
            self.assertFalse(bridge.offer(
                memory=retired,
                observation=observation(version=50),
                episode_id="episode-1",
                captured_at_ms=1_200,
            ))

        self.assertEqual(bridge.snapshot(), current)
        self.assertEqual(current["frame_id"], "ev3-local-2")
        self.assertEqual(current["based_on_world_model_version"], 2)

    def test_rejects_regressing_capture_instead_of_clamping_it(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory)
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(1_200),
            )
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=1, raw=60),
                episode_id="episode-1",
                captured_at_ms=1_000,
            ))
            memory.hazard_map.record_observation(
                memory.pose,
                observation(version=2, raw=55),
                1_100,
            )
            self.assertFalse(bridge.offer(
                memory=memory,
                observation=observation(version=2, raw=55),
                episode_id="episode-1",
                captured_at_ms=900,
            ))
            unchanged = bridge.snapshot()
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=2, raw=55),
                episode_id="episode-1",
                captured_at_ms=1_100,
            ))
            updated = bridge.snapshot()

        self.assertEqual(unchanged["captured_at_unix_ms"], 1_000)
        self.assertEqual(unchanged["based_on_state_version"], 1)
        self.assertEqual(updated["captured_at_unix_ms"], 1_100)
        self.assertEqual(updated["based_on_state_version"], 2)
        self.assertEqual(
            [
                item["observed_at_unix_ms"]
                for item in updated["qualitative_observations"]
            ],
            [1_000, 1_100],
        )

    def test_snapshots_are_detached_and_invalid_localization_is_not_drawn(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory, localization_valid=False)
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(1_000),
            )
            bridge.offer(
                memory=memory,
                observation=observation(),
                episode_id="episode-1",
                captured_at_ms=1_000,
            )
            first = bridge.snapshot()
            first["qualitative_observations"].clear()
            second = bridge.snapshot()

        self.assertEqual(first["status"], "degraded")
        self.assertEqual(
            first["reason_code"],
            "physical_localization_invalid",
        )
        self.assertIsNone(first["robot_pose"])
        self.assertEqual(len(second["qualitative_observations"]), 1)

    def test_empty_and_closed_bridge_are_honest(self):
        bridge = PhysicalSpatialMapBridge(
            robot_id="ev3rstorm-01",
            controller_instance_id="ev3-main",
        )
        empty = bridge.snapshot()
        self.assertEqual(empty["status"], "unavailable")
        self.assertEqual(empty["reason_code"], "no_physical_observations")
        self.assertTrue(bridge.close())
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory)
            self.assertFalse(bridge.offer(
                memory=memory,
                observation=observation(),
                episode_id="episode-1",
                captured_at_ms=1_000,
            ))


if __name__ == "__main__":
    unittest.main()
