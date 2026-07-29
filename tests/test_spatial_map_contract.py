from dataclasses import FrozenInstanceError, replace
import json
import unittest

from robot_agent.navigation_contract import NavigationContractError
from robot_agent.spatial_map_contract import (
    CELL_OCCUPIED,
    LOCAL_ODOMETRY,
    MAP_PROVISIONAL_IR,
    METRIC_FUSED,
    ObjectHypothesis,
    PHYSICAL_IR_REFLECTION,
    PROVISIONAL_QUALITATIVE,
    QualitativeObstacleEvidence,
    OccupancyCell,
    SpatialMapSnapshot,
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


class SpatialEvidenceContractTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
