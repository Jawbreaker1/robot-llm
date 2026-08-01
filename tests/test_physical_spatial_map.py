import json
from pathlib import Path
import tempfile
import unittest

from robot_agent.navigation_memory_store import NavigationMemoryStore
from robot_agent.physical_odometry import DriveMotorRoles, PhysicalPose
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_scan_evidence import (
    MAX_SCAN_ATTEMPTS_PER_HAZARD,
    MAX_SCAN_ATTEMPTS_PER_MAP,
    ScanAttemptEvidence,
    ScanRayEvidence,
)
from robot_agent.physical_spatial_map import (
    MAX_QUALITATIVE_OBSERVATIONS,
    PhysicalSpatialMapBridge,
)
from robot_agent.provisional_hazard_map import (
    HAZARD_CAPACITY_EVICTION,
    MAX_HAZARDS_PER_MAP,
    PER_HAZARD_SCAN_EVICTION,
    HazardMapCalibration,
    ProvisionalHazard,
    ProvisionalHazardMap,
)
from robot_agent.spatial_map_contract import MAX_POSE_HISTORY


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
        self.assertEqual(snapshot["pose_history"], [snapshot["robot_pose"]])
        self.assertEqual(snapshot["pose_history_evicted"], 0)
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
        self.assertEqual(snapshot["hazard_retention"], {
            "capacity": MAX_HAZARDS_PER_MAP,
            "retained_count": 1,
            "evicted_count": 0,
            "last_eviction_reason": None,
        })
        self.assertEqual(snapshot["scan_attempt_retention"], {
            "per_hazard_capacity": MAX_SCAN_ATTEMPTS_PER_HAZARD,
            "map_capacity": MAX_SCAN_ATTEMPTS_PER_MAP,
            "retained_count": 0,
            "evicted_count": 0,
            "last_eviction_reason": None,
        })
        projected = snapshot["object_hypotheses"][0]
        self.assertEqual(projected["hypothesis_id"], hazard.hypothesis_id)
        self.assertEqual(
            projected["geometry_kind"],
            "QUALITATIVE_FORWARD_ENVELOPE",
        )
        self.assertIsNone(projected["x_mm"])
        self.assertIsNone(projected["y_mm"])
        self.assertNotIn("simulation", json.dumps(snapshot).lower())

    def test_projects_authoritative_hazard_eviction_state(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory, map_revision=7)
            memory.hazard_map = ProvisionalHazardMap(
                frame_id=memory.frame_id,
                map_generation_id=memory.generation_id,
                revision=7,
                hazards_evicted=3,
                hazards_eviction_reason=HAZARD_CAPACITY_EVICTION,
                scan_attempts_evicted=5,
                scan_attempts_eviction_reason=(
                    PER_HAZARD_SCAN_EVICTION
                ),
            )
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(1_100),
            )

            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(),
                episode_id="episode-retention",
                captured_at_ms=1_000,
            ))
            snapshot = bridge.snapshot()

        self.assertEqual(snapshot["hazard_retention"], {
            "capacity": MAX_HAZARDS_PER_MAP,
            "retained_count": 0,
            "evicted_count": 3,
            "last_eviction_reason": HAZARD_CAPACITY_EVICTION,
        })
        self.assertEqual(snapshot["scan_attempt_retention"], {
            "per_hazard_capacity": MAX_SCAN_ATTEMPTS_PER_HAZARD,
            "map_capacity": MAX_SCAN_ATTEMPTS_PER_MAP,
            "retained_count": 0,
            "evicted_count": 5,
            "last_eviction_reason": PER_HAZARD_SCAN_EVICTION,
        })

    def test_projects_authoritative_asymmetric_collision_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory)
            footprint = RobotFootprint(
                front_extent_mm=110,
                rear_extent_mm=90,
                left_extent_mm=105,
                right_extent_mm=160,
                clearance_margin_mm=10,
                calibration_status="provisional",
                calibration_evidence="assembled right arm observed",
            )
            memory.hazard_map = ProvisionalHazardMap(
                frame_id=memory.frame_id,
                map_generation_id=memory.generation_id,
                calibration=HazardMapCalibration(
                    robot_footprint=footprint,
                ),
            )
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(1_100),
            )

            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(),
                episode_id="episode-footprint",
                captured_at_ms=1_000,
            ))
            snapshot = bridge.snapshot()

        geometry = snapshot["collision_geometry"]
        self.assertEqual(geometry, footprint.to_dict())
        self.assertGreater(
            geometry["right_extent_mm"],
            geometry["left_extent_mm"],
        )
        self.assertNotIn("obstacle_distance_mm", geometry)
        self.assertEqual(snapshot["scan_evidence_history"], [])

    def test_scan_history_uses_scan_pose_and_never_hazard_anchor_as_origin(
        self,
    ):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory)
            attempt = ScanAttemptEvidence(
                scan_id="scan-pose-proof",
                completed_at_ms=950,
                status="CANCELLED",
                reason="bilateral_boundaries_not_observed",
                rays=(
                    ScanRayEvidence(-30_000, -28_500, True, 24, 25),
                    ScanRayEvidence(30_000, 31_500, False, 56, 55),
                ),
                left_boundary_mdeg=None,
                right_boundary_mdeg=None,
                scan_pose=PhysicalPose(
                    x_mm=80,
                    y_mm=-35,
                    heading_mdeg=45_000,
                ),
                based_on_map_version=3,
            )
            hazard = ProvisionalHazard(
                hypothesis_id="hazard-original-anchor",
                frame_id=memory.frame_id,
                anchor_x_mm=25,
                anchor_y_mm=-10,
                anchor_heading_mdeg=30_000,
                centroid_x_mm=140,
                centroid_y_mm=0,
                radius_mm=70,
                first_seen_at_ms=900,
                last_seen_at_ms=900,
                evidence_count=1,
                last_state_version=1,
                last_raw_ir_proximity=24,
                last_filtered_ir_proximity=25,
                scan_evidence_history=(attempt,),
            )
            memory.hazard_map = ProvisionalHazardMap(
                frame_id=memory.frame_id,
                map_generation_id=memory.generation_id,
                hazards=(hazard,),
                revision=4,
            )
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(1_100),
            )

            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(),
                episode_id="episode-scan-pose",
                captured_at_ms=1_000,
            ))
            snapshot = bridge.snapshot()

        self.assertEqual(len(snapshot["scan_evidence_history"]), 1)
        self.assertEqual(snapshot["scan_evidence_history_evicted"], 0)
        evidence = snapshot["scan_evidence_history"][0]
        self.assertEqual(evidence["hypothesis_anchor_pose"], {
            "x_mm": 25,
            "y_mm": -10,
            "heading_mdeg": 30_000,
        })
        self.assertEqual(evidence["scan_pose"], {
            "x_mm": 80,
            "y_mm": -35,
            "heading_mdeg": 45_000,
        })
        self.assertEqual(evidence["based_on_map_version"], 3)
        self.assertEqual(evidence["age_ms"], 150)
        self.assertNotEqual(
            evidence["scan_pose"],
            evidence["hypothesis_anchor_pose"],
        )
        encoded_rays = json.dumps(evidence["rays"])
        self.assertNotIn("distance", encoded_rays)
        self.assertNotIn("endpoint", encoded_rays)

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
        self.assertEqual(len(reset["pose_history"]), 1)
        self.assertEqual(
            reset["pose_history"][0]["frame_id"],
            "ev3-local-2",
        )
        self.assertEqual(reset["pose_history_evicted"], 0)

    def test_frame_change_resets_history_even_with_same_generation_id(self):
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
                generation="generation-1",
                frame="ev3-local-2",
                pose=PhysicalPose(x_mm=500),
            )
            self.assertTrue(bridge.offer(
                memory=second,
                observation=observation(version=2),
                episode_id="episode-2",
                captured_at_ms=1_100,
            ))
            snapshot = bridge.snapshot()

        self.assertEqual(snapshot["frame_id"], "ev3-local-2")
        self.assertEqual(len(snapshot["pose_history"]), 1)
        self.assertEqual(snapshot["pose_history"][0]["x_mm"], 500)
        self.assertEqual(snapshot["pose_history_evicted"], 0)

    def test_rejects_retired_frame_within_the_active_generation(self):
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
                generation="generation-1",
                frame="ev3-local-2",
            )
            self.assertTrue(bridge.offer(
                memory=second,
                observation=observation(version=2),
                episode_id="episode-1",
                captured_at_ms=1_100,
            ))
            current = bridge.snapshot()
            rolled_back = self.memory(
                directory,
                generation="generation-1",
                frame="ev3-local-1",
                map_revision=50,
            )

            self.assertFalse(bridge.offer(
                memory=rolled_back,
                observation=observation(version=50),
                episode_id="episode-1",
                captured_at_ms=1_200,
            ))

        self.assertEqual(bridge.snapshot(), current)
        self.assertEqual(current["frame_id"], "ev3-local-2")

    def test_pose_history_keeps_only_changed_valid_local_odometry(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory)
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(1_500),
            )
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=1),
                episode_id="episode-1",
                captured_at_ms=1_000,
            ))
            memory.hazard_map.record_observation(
                memory.pose,
                observation(version=2),
                1_100,
            )
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=2),
                episode_id="episode-1",
                captured_at_ms=1_100,
            ))
            memory.pose = PhysicalPose(
                x_mm=75,
                y_mm=-10,
                heading_mdeg=30_000,
            )
            memory.hazard_map.record_observation(
                memory.pose,
                observation(version=3),
                1_200,
            )
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=3),
                episode_id="episode-1",
                captured_at_ms=1_200,
            ))
            memory.pose = PhysicalPose(
                x_mm=75,
                y_mm=-10,
                heading_mdeg=45_000,
            )
            memory.hazard_map.record_observation(
                memory.pose,
                observation(version=4),
                1_300,
            )
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=4),
                episode_id="episode-1",
                captured_at_ms=1_300,
            ))
            memory.localization_valid = False
            memory.localization_error = "encoder anchor lost"
            memory.hazard_map.record_observation(
                memory.pose,
                observation(version=5),
                1_400,
            )
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=5),
                episode_id="episode-1",
                captured_at_ms=1_400,
            ))
            snapshot = bridge.snapshot()

        self.assertIsNone(snapshot["robot_pose"])
        self.assertEqual(
            [
                (
                    item["x_mm"],
                    item["y_mm"],
                    item["heading_mdeg"],
                )
                for item in snapshot["pose_history"]
            ],
            [
                (25, -10, 30_000),
                (75, -10, 30_000),
                (75, -10, 45_000),
            ],
        )
        self.assertEqual(
            [item["state_version"] for item in snapshot["pose_history"]],
            [1, 3, 4],
        )
        self.assertEqual(
            [item["age_ms"] for item in snapshot["pose_history"]],
            [500, 300, 200],
        )

    def test_pose_history_is_hard_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(
                directory,
                pose=PhysicalPose(x_mm=1),
            )
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(10_000),
            )
            for version in range(1, MAX_POSE_HISTORY + 3):
                memory.pose = PhysicalPose(x_mm=version)
                if version > 1:
                    memory.hazard_map.record_observation(
                        memory.pose,
                        observation(version=version),
                        1_000 + version,
                    )
                self.assertTrue(bridge.offer(
                    memory=memory,
                    observation=observation(version=version),
                    episode_id="episode-1",
                    captured_at_ms=1_000 + version,
                ))
            snapshot = bridge.snapshot()

        self.assertEqual(len(snapshot["pose_history"]), MAX_POSE_HISTORY)
        self.assertEqual(snapshot["pose_history_evicted"], 2)
        self.assertEqual(snapshot["pose_history"][0]["x_mm"], 3)
        self.assertEqual(
            len(snapshot["qualitative_observations"]),
            MAX_QUALITATIVE_OBSERVATIONS,
        )
        self.assertEqual(
            snapshot["qualitative_observations_evicted"],
            MAX_POSE_HISTORY + 2 - MAX_QUALITATIVE_OBSERVATIONS,
        )

    def test_scan_projection_counts_evidence_that_leaves_the_hot_view(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory)
            attempt = ScanAttemptEvidence(
                scan_id="scan-before-eviction",
                completed_at_ms=950,
                status="CANCELLED",
                reason="bilateral_boundaries_not_observed",
                rays=(ScanRayEvidence(0, 0, True, 24, 25),),
                left_boundary_mdeg=None,
                right_boundary_mdeg=None,
                scan_pose=PhysicalPose(),
                based_on_map_version=1,
            )
            hazard = ProvisionalHazard(
                hypothesis_id="hazard-with-scan",
                frame_id=memory.frame_id,
                anchor_x_mm=0,
                anchor_y_mm=0,
                anchor_heading_mdeg=0,
                centroid_x_mm=140,
                centroid_y_mm=0,
                radius_mm=70,
                first_seen_at_ms=900,
                last_seen_at_ms=900,
                evidence_count=1,
                last_state_version=1,
                last_raw_ir_proximity=24,
                last_filtered_ir_proximity=25,
                scan_evidence_history=(attempt,),
            )
            memory.hazard_map = ProvisionalHazardMap(
                frame_id=memory.frame_id,
                map_generation_id=memory.generation_id,
                hazards=(hazard,),
                revision=2,
            )
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(2_000),
            )
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=1),
                episode_id="episode-scan-eviction",
                captured_at_ms=1_000,
            ))
            memory.hazard_map = ProvisionalHazardMap(
                frame_id=memory.frame_id,
                map_generation_id=memory.generation_id,
                revision=3,
            )
            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=2),
                episode_id="episode-scan-eviction",
                captured_at_ms=1_100,
            ))
            snapshot = bridge.snapshot()

        self.assertEqual(snapshot["scan_evidence_history"], [])
        self.assertEqual(snapshot["scan_evidence_history_evicted"], 1)

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

    def test_rejects_retired_generation_even_with_a_new_frame(self):
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
                frame="ev3-local-reintroduced",
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

    def test_invalid_drive_metadata_does_not_mutate_or_block_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = self.memory(directory)
            bridge = PhysicalSpatialMapBridge(
                robot_id=memory.robot_id,
                controller_instance_id=memory.controller_instance_id,
                clock_ms=FixedClock(2_000),
            )
            incomplete = observation(version=1)
            incomplete["motors"] = incomplete["motors"][:1]

            with self.assertRaisesRegex(
                ValueError,
                "physical map drive encoder is missing",
            ):
                bridge.offer(
                    memory=memory,
                    observation=incomplete,
                    episode_id="episode-1",
                    captured_at_ms=1_000,
                )
            self.assertIsNone(bridge.snapshot()["map_version"])

            self.assertTrue(bridge.offer(
                memory=memory,
                observation=observation(version=1),
                episode_id="episode-1",
                captured_at_ms=1_000,
            ))
            snapshot = bridge.snapshot()

        self.assertEqual(snapshot["map_version"], 0)
        self.assertEqual(len(snapshot["pose_history"]), 1)
        self.assertEqual(snapshot["based_on_state_version"], 1)

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
            first["pose_history"].append({"x_mm": 999})
            first["collision_geometry"]["radius_mm"] = 999
            second = bridge.snapshot()

        self.assertEqual(first["status"], "degraded")
        self.assertEqual(
            first["reason_code"],
            "physical_localization_invalid",
        )
        self.assertIsNone(first["robot_pose"])
        self.assertEqual(len(second["qualitative_observations"]), 1)
        self.assertEqual(second["pose_history"], [])
        self.assertEqual(second["collision_geometry"]["radius_mm"], 70)

    def test_empty_and_closed_bridge_are_honest(self):
        bridge = PhysicalSpatialMapBridge(
            robot_id="ev3rstorm-01",
            controller_instance_id="ev3-main",
        )
        empty = bridge.snapshot()
        self.assertEqual(empty["status"], "unavailable")
        self.assertEqual(empty["reason_code"], "no_physical_observations")
        self.assertEqual(empty["pose_history"], [])
        self.assertEqual(empty["pose_history_evicted"], 0)
        self.assertIsNone(empty["collision_geometry"])
        self.assertEqual(empty["scan_evidence_history"], [])
        self.assertEqual(empty["scan_evidence_history_evicted"], 0)
        self.assertEqual(empty["scan_attempt_retention"], {
            "per_hazard_capacity": MAX_SCAN_ATTEMPTS_PER_HAZARD,
            "map_capacity": MAX_SCAN_ATTEMPTS_PER_MAP,
            "retained_count": 0,
            "evicted_count": 0,
            "last_eviction_reason": None,
        })
        self.assertEqual(empty["qualitative_observations_evicted"], 0)
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
