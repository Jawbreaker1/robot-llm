from dataclasses import FrozenInstanceError, replace
import json
import unittest

from robot_agent.navigation_contract import NavigationContractError
from robot_agent.spatial_map_contract import (
    CELL_OCCUPIED,
    LOCAL_ODOMETRY,
    LOCAL_ODOMETRY_POSE,
    MAP_PROVISIONAL_IR,
    MAX_POSE_HISTORY,
    METRIC_FUSED,
    ObjectHypothesis,
    PHYSICAL_IR_REFLECTION,
    PROVISIONAL_QUALITATIVE,
    QUALITATIVE_FORWARD_ENVELOPE,
    ProvisionalObjectHypothesis,
    QualitativeObstacleEvidence,
    OccupancyCell,
    SpatialMapSnapshot,
    SpatialCollisionGeometry,
    SpatialScanEvidence,
    SpatialScanRayEvidence,
    SpatialRobotPose,
    SpatialSensorRay,
)


def physical_evidence():
    return QualitativeObstacleEvidence(
        evidence_id="ir-1",
        robot_id="robot-1",
        controller_instance_id="controller-1",
        frame_id="ROBOT_BASE",
        source=PHYSICAL_IR_REFLECTION,
        bearing="FORWARD",
        relation="NEAR_OBSTACLE",
        observed_at_ms=100,
        captured_at_host_ms=105,
        state_version=2,
        world_model_version=1,
        confidence_milli=250,
        raw_ir_proximity=80,
    )


def physical_map_snapshot():
    pose = SpatialRobotPose(
        frame_id="local-odometry",
        x_mm=10,
        y_mm=20,
        heading_mdeg=30_000,
        observed_at_ms=100,
        captured_at_host_ms=105,
        state_version=2,
        world_model_version=1,
    )
    return SpatialMapSnapshot(
        map_id="map-1",
        robot_id="robot-1",
        controller_instance_id="controller-1",
        frame_id="local-odometry",
        frame_kind=LOCAL_ODOMETRY,
        map_quality=MAP_PROVISIONAL_IR,
        evidence_sources=(PHYSICAL_IR_REFLECTION,),
        resolution_mm=50,
        capacity=10,
        map_version=1,
        created_at_ms=0,
        updated_at_ms=105,
        last_observed_at_ms=100,
        based_on_state_version=2,
        based_on_world_model_version=1,
        cells_evicted=0,
        bounds=None,
        latest_robot_pose=pose,
        sensor_rays=(),
        cells=(),
        qualitative_evidence=(physical_evidence(),),
        object_hypotheses=(),
    )


def provisional_hypothesis():
    return ProvisionalObjectHypothesis(
        hypothesis_id="provisional-object-1",
        robot_id="robot-1",
        controller_instance_id="controller-1",
        frame_id="local-odometry",
        semantic_label="UNKNOWN",
        source=PHYSICAL_IR_REFLECTION,
        geometry_kind=QUALITATIVE_FORWARD_ENVELOPE,
        bearing="FORWARD",
        relation="NEAR_OBSTACLE",
        anchor_x_mm=10,
        anchor_y_mm=20,
        anchor_heading_mdeg=30_000,
        first_seen_at_ms=100,
        last_seen_at_ms=100,
        last_state_version=2,
        last_world_model_version=1,
        evidence_count=1,
        confidence_milli=250,
        provenance=(
            LOCAL_ODOMETRY_POSE,
            PHYSICAL_IR_REFLECTION,
        ),
    )


class SpatialEvidenceContractTests(unittest.TestCase):
    def test_asymmetric_collision_geometry_preserves_right_side_extent(self):
        geometry = SpatialCollisionGeometry.from_mapping({
            "geometry": "ASYMMETRIC_RECTANGLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "front_extent_mm": 110,
            "rear_extent_mm": 90,
            "left_extent_mm": 105,
            "right_extent_mm": 160,
            "clearance_margin_mm": 10,
            "calibration_status": "provisional",
            "calibration_evidence": "assembled right arm observed",
        })

        self.assertEqual(geometry.right_extent_mm, 160)
        self.assertGreater(
            geometry.right_extent_mm,
            geometry.left_extent_mm,
        )
        self.assertEqual(
            geometry.to_dict()["reference_point"],
            "DIFFERENTIAL_DRIVE_ORIGIN",
        )
        with self.assertRaises(NavigationContractError):
            SpatialCollisionGeometry.from_mapping({
                "geometry": "ASYMMETRIC_RECTANGLE",
                "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
                "front_extent_mm": 110,
                "rear_extent_mm": 90,
                "left_extent_mm": 105,
                "right_extent_mm": 160,
                "clearance_margin_mm": 10,
                "calibration_status": "provisional",
                "calibration_evidence": "assembled right arm observed",
                "measured_obstacle_distance_mm": 40,
            })

    def test_scan_evidence_separates_actual_scan_pose_from_hypothesis_anchor(
        self,
    ):
        scan = SpatialScanEvidence(
            target_hypothesis_id="hazard-1",
            frame_id="local-odometry",
            hypothesis_anchor_x_mm=10,
            hypothesis_anchor_y_mm=20,
            hypothesis_anchor_heading_mdeg=30_000,
            scan_x_mm=80,
            scan_y_mm=-35,
            scan_heading_mdeg=45_000,
            based_on_map_version=3,
            scan_id="scan-1",
            completed_at_unix_ms=100,
            status="CANCELLED",
            reason="bilateral_boundaries_not_observed",
            observation_pattern="MIXED",
            arc_coverage="BILATERAL_ARC",
            boundary_coverage="NO_BOUNDARIES",
            hypothesis_relation="SUPPORTS_BLOCKED_HYPOTHESIS",
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
            rays=(
                SpatialScanRayEvidence(
                    requested_relative_bearing_mdeg=-30_000,
                    actual_relative_bearing_mdeg=-28_500,
                    blocked=True,
                    raw_ir_proximity=24,
                    filtered_ir_proximity=25,
                ),
                SpatialScanRayEvidence(
                    requested_relative_bearing_mdeg=30_000,
                    actual_relative_bearing_mdeg=31_500,
                    blocked=False,
                    raw_ir_proximity=56,
                    filtered_ir_proximity=55,
                ),
            ),
        )

        payload = scan.to_dict()
        self.assertNotEqual(
            payload["scan_pose"],
            payload["hypothesis_anchor_pose"],
        )
        self.assertEqual(payload["based_on_map_version"], 3)
        self.assertNotIn("distance", json.dumps(payload["rays"]))
        legacy = replace(
            scan,
            scan_x_mm=None,
            scan_y_mm=None,
            scan_heading_mdeg=None,
            based_on_map_version=None,
        )
        self.assertIsNone(legacy.to_dict()["scan_pose"])
        with self.assertRaises(NavigationContractError):
            replace(scan, scan_x_mm=None)

    def test_physical_ir_contract_is_low_confidence_and_non_metric(self):
        evidence = physical_evidence()
        ray = SpatialSensorRay(
            direction="FORWARD",
            frame_id="ROBOT_BASE",
            source=PHYSICAL_IR_REFLECTION,
            observed_at_ms=100,
            captured_at_host_ms=105,
            state_version=2,
            world_model_version=1,
            confidence_milli=250,
            provisional=True,
            relation="NEAR_OBSTACLE",
            raw_ir_proximity=80,
        )

        self.assertEqual(
            evidence.quality,
            PROVISIONAL_QUALITATIVE,
        )
        self.assertNotIn("measured_range_mm", ray.to_dict())
        self.assertNotIn("end_x_mm", ray.to_dict())
        json.dumps(evidence.to_dict(), allow_nan=False)
        json.dumps(ray.to_dict(), allow_nan=False)

    def test_physical_ir_cannot_claim_metric_geometry_or_high_confidence(self):
        invalid_rays = (
            {
                "confidence_milli": 401,
            },
            {
                "origin_x_mm": 0,
                "origin_y_mm": 0,
                "end_x_mm": 10,
                "end_y_mm": 0,
                "measured_range_mm": 10,
                "max_range_mm": 100,
                "endpoint_occupied": True,
            },
        )
        base = {
            "direction": "FORWARD",
            "frame_id": "ROBOT_BASE",
            "source": PHYSICAL_IR_REFLECTION,
            "observed_at_ms": 100,
            "captured_at_host_ms": 105,
            "state_version": 2,
            "world_model_version": 1,
            "confidence_milli": 250,
            "provisional": True,
            "relation": "NEAR_OBSTACLE",
        }
        for changes in invalid_rays:
            with self.subTest(changes=changes):
                value = dict(base)
                value.update(changes)
                with self.assertRaises(NavigationContractError):
                    SpatialSensorRay(**value)

        with self.assertRaises(NavigationContractError):
            replace(physical_evidence(), confidence_milli=401)
        with self.assertRaises(NavigationContractError):
            replace(physical_evidence(), provisional=False)

    def test_provisional_object_has_anchor_but_no_metric_object_bounds(self):
        hypothesis = provisional_hypothesis()
        payload = hypothesis.to_dict()

        self.assertIsNone(payload["bounds_mm"])
        self.assertEqual(payload["anchor_pose"]["x_mm"], 10)
        self.assertNotIn("centroid_mm", payload)
        self.assertNotIn("cell_count", payload)
        self.assertNotIn("measured_range_mm", payload)
        self.assertLessEqual(hypothesis.confidence_milli, 400)

        with self.assertRaises(NavigationContractError):
            replace(hypothesis, confidence_milli=401)
        with self.assertRaises(NavigationContractError):
            replace(
                hypothesis,
                provenance=(PHYSICAL_IR_REFLECTION,),
            )


class SpatialSnapshotContractTests(unittest.TestCase):
    def test_snapshot_is_frozen_and_json_friendly(self):
        cell = OccupancyCell(
            grid_x=1,
            grid_y=2,
            center_x_mm=75,
            center_y_mm=125,
            classification=CELL_OCCUPIED,
            occupancy_milli=650,
            first_seen_at_ms=100,
            last_seen_at_ms=100,
            last_state_version=1,
            last_world_model_version=1,
            evidence_count=1,
            free_evidence_count=0,
            occupied_evidence_count=1,
            provenance=("SIMULATION_CONFIGURATION_SPACE:FORWARD",),
            quality=METRIC_FUSED,
        )
        snapshot = physical_map_snapshot()

        with self.assertRaises(FrozenInstanceError):
            snapshot.map_version = 2
        encoded = json.dumps(snapshot.to_dict(), allow_nan=False)
        self.assertIn("robot-spatial-map-snapshot/v1", encoded)
        self.assertIsInstance(snapshot.cells, tuple)
        self.assertEqual(cell.to_dict()["provenance"], [
            "SIMULATION_CONFIGURATION_SPACE:FORWARD"
        ])
        self.assertEqual(snapshot.pose_history, ())
        self.assertEqual(snapshot.pose_history_evicted, 0)
        self.assertEqual(snapshot.to_dict()["pose_history"], [])
        self.assertIsNone(snapshot.to_dict()["collision_geometry"])
        self.assertEqual(snapshot.to_dict()["scan_evidence_history"], [])

    def test_snapshot_accepts_a_bounded_monotonic_pose_history(self):
        snapshot = physical_map_snapshot()
        first = SpatialRobotPose(
            frame_id=snapshot.frame_id,
            x_mm=0,
            y_mm=0,
            heading_mdeg=0,
            observed_at_ms=90,
            captured_at_host_ms=95,
            state_version=1,
            world_model_version=1,
        )
        retained = replace(
            snapshot,
            pose_history=(first, snapshot.latest_robot_pose),
            pose_history_evicted=3,
        )

        self.assertEqual(len(retained.pose_history), 2)
        self.assertEqual(retained.pose_history_evicted, 3)
        self.assertEqual(
            retained.to_dict()["pose_history"][0]["x_mm"],
            0,
        )

    def test_snapshot_rejects_invalid_pose_history_boundaries(self):
        snapshot = physical_map_snapshot()
        first = SpatialRobotPose(
            frame_id=snapshot.frame_id,
            x_mm=0,
            y_mm=0,
            heading_mdeg=0,
            observed_at_ms=90,
            captured_at_host_ms=95,
            state_version=1,
            world_model_version=1,
        )
        duplicate_geometry = replace(
            snapshot.latest_robot_pose,
            x_mm=first.x_mm,
            y_mm=first.y_mm,
            heading_mdeg=first.heading_mdeg,
        )
        cases = (
            ("not-a-tuple", "invalid_pose_history"),
            (
                tuple(first for _ in range(MAX_POSE_HISTORY + 1)),
                "invalid_pose_history",
            ),
            (
                (replace(first, frame_id="foreign-frame"),),
                "inconsistent_pose_history",
            ),
            (
                (replace(first, world_model_version=2),),
                "inconsistent_pose_history",
            ),
            (
                (first, duplicate_geometry),
                "inconsistent_pose_history_order",
            ),
        )
        for history, code in cases:
            with self.subTest(code=code):
                with self.assertRaises(NavigationContractError) as caught:
                    replace(snapshot, pose_history=history)
                self.assertEqual(caught.exception.code, code)

        with self.assertRaises(NavigationContractError) as caught:
            replace(snapshot, pose_history=(first,))
        self.assertEqual(
            caught.exception.code,
            "inconsistent_latest_pose_history",
        )

    def test_snapshot_rejects_cross_identity_or_version_children(self):
        snapshot = physical_map_snapshot()

        with self.assertRaises(NavigationContractError):
            replace(
                snapshot,
                qualitative_evidence=(
                    replace(physical_evidence(), robot_id="other-robot"),
                ),
            )
        with self.assertRaises(NavigationContractError):
            replace(
                snapshot,
                qualitative_evidence=(
                    replace(physical_evidence(), state_version=3),
                ),
            )

    def test_snapshot_rejects_foreign_metric_ray_and_hypothesis_frames(self):
        snapshot = physical_map_snapshot()
        foreign_ray = SpatialSensorRay(
            direction="FORWARD",
            frame_id="foreign-frame",
            source="simulation_metric",
            observed_at_ms=100,
            captured_at_host_ms=105,
            state_version=2,
            world_model_version=1,
            confidence_milli=900,
            provisional=False,
            origin_x_mm=0,
            origin_y_mm=0,
            end_x_mm=100,
            end_y_mm=0,
            measured_range_mm=100,
            max_range_mm=200,
            endpoint_occupied=True,
        )
        foreign_hypothesis = ObjectHypothesis(
            hypothesis_id="object-foreign",
            frame_id="foreign-frame",
            semantic_label="UNKNOWN",
            min_x_mm=50,
            min_y_mm=50,
            max_x_mm=100,
            max_y_mm=100,
            centroid_x_mm=75,
            centroid_y_mm=75,
            cell_count=1,
            first_seen_at_ms=100,
            last_seen_at_ms=100,
            evidence_count=1,
            confidence_milli=650,
            provenance=(
                "SIMULATION_CONFIGURATION_SPACE:FORWARD",
            ),
        )

        with self.assertRaises(NavigationContractError):
            replace(snapshot, sensor_rays=(foreign_ray,))
        with self.assertRaises(NavigationContractError):
            replace(
                snapshot,
                object_hypotheses=(foreign_hypothesis,),
            )

    def test_snapshot_accepts_local_provisional_hypothesis(self):
        snapshot = replace(
            physical_map_snapshot(),
            object_hypotheses=(provisional_hypothesis(),),
        )

        self.assertEqual(
            snapshot.object_hypotheses[0].hypothesis_id,
            "provisional-object-1",
        )


if __name__ == "__main__":
    unittest.main()
