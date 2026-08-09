from copy import deepcopy
from unittest import TestCase

from robot_agent.shared_frame_transform import CalibratedFrameTransform
from robot_agent.shared_spatial_map import (
    MAX_SHARED_POSE_HISTORY,
    SharedSpatialMapCompositor,
    SharedSpatialMapError,
)


WORLD_FRAME_ID = "shared-world"
WORLD_GENERATION_ID = "shared-generation-1"


class Provider:
    def __init__(self, value=None, error=None):
        self.value = value
        self.error = error
        self.snapshot_calls = 0
        self.write_calls = 0
        self.close_calls = 0

    def snapshot(self):
        self.snapshot_calls += 1
        if self.error is not None:
            raise self.error
        return self.value

    def offer(self, *args, **kwargs):
        self.write_calls += 1
        raise AssertionError("shared compositor attempted a source write")

    def close(self, *args, **kwargs):
        self.close_calls += 1
        raise AssertionError("shared compositor attempted to close a source")


def frame_transform(
    robot_id,
    controller_id,
    frame_id,
    generation_id,
    *,
    tx_mm=0,
    ty_mm=0,
    yaw_mdeg=0,
    position_uncertainty_mm=10,
    yaw_uncertainty_mdeg=1_000,
    provenance=("FIXED_START_POSE",),
    world_frame_id=WORLD_FRAME_ID,
    world_generation_id=WORLD_GENERATION_ID,
):
    return CalibratedFrameTransform(
        source_robot_id=robot_id,
        source_controller_id=controller_id,
        source_frame_id=frame_id,
        source_generation_id=generation_id,
        world_frame_id=world_frame_id,
        world_generation_id=world_generation_id,
        tx_mm=tx_mm,
        ty_mm=ty_mm,
        yaw_mdeg=yaw_mdeg,
        position_uncertainty_mm=position_uncertainty_mm,
        yaw_uncertainty_mdeg=yaw_uncertainty_mdeg,
        provenance=provenance,
    )


def pose(
    frame_id,
    x_mm,
    y_mm,
    heading_mdeg,
    *,
    state_version=1,
    source_id="navigation-pose",
    provenance="LOCAL_ODOMETRY",
    observed_at_unix_ms=1_000,
    age_ms=0,
):
    return {
        "x_mm": x_mm,
        "y_mm": y_mm,
        "heading_mdeg": heading_mdeg,
        "frame_id": frame_id,
        "state_version": state_version,
        "source_id": source_id,
        "provenance": provenance,
        "observed_at_unix_ms": observed_at_unix_ms,
        "age_ms": age_ms,
    }


def local_map(
    robot_id,
    controller_id,
    frame_id,
    generation_id,
    *,
    current_pose=None,
    history=None,
    collision_geometry=None,
    map_version=1,
    captured_at_unix_ms=1_000,
):
    if current_pose is None:
        current_pose = pose(frame_id, 0, 0, 0)
    if history is None:
        history = [current_pose]
    return {
        "schema": "robot-spatial-map/v1",
        "read_only": True,
        "status": "pose_only",
        "map_id": "{}-local-map".format(robot_id),
        "robot_id": robot_id,
        "controller_instance_id": controller_id,
        "frame_id": frame_id,
        "local_generation_id": generation_id,
        "frame_kind": "LOCAL_ODOMETRY",
        "map_version": map_version,
        "robot_pose": current_pose,
        "pose_history": history,
        "pose_history_evicted": 0,
        "collision_geometry": collision_geometry,
        "captured_at_unix_ms": captured_at_unix_ms,
        "age_ms": 0,
        "cells": [],
        "sensor_rays": [],
        "object_hypotheses": [],
    }


def compositor(*bindings):
    return SharedSpatialMapCompositor(
        world_frame_id=WORLD_FRAME_ID,
        world_generation_id=WORLD_GENERATION_ID,
        bindings=tuple(bindings),
    )


class SharedSpatialMapCompositorTests(TestCase):
    def test_exact_two_source_fixture_is_canonical_and_world_aligned(self):
        ev3_transform = frame_transform(
            "ev3rstorm-01",
            "ev3-controller",
            "ev3-local",
            "ev3-generation-4",
            provenance=("MEASURED_FIXED_START",),
        )
        blast_transform = frame_transform(
            "blast-01",
            "blast-01.hub",
            "blast-local",
            "blast-episode-7",
            tx_mm=1_000,
            ty_mm=500,
            yaw_mdeg=90_000,
            position_uncertainty_mm=12,
            yaw_uncertainty_mdeg=2_000,
        )
        circle = {
            "geometry": "SYMMETRIC_CIRCLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "radius_mm": 90,
        }
        rectangle = {
            "geometry": "ASYMMETRIC_RECTANGLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "front_extent_mm": 150,
            "rear_extent_mm": 90,
            "left_extent_mm": 80,
            "right_extent_mm": 80,
            "clearance_margin_mm": 15,
            "calibration_status": "PROVISIONAL",
            "calibration_evidence": "MEASURED_BODY_EXTENTS",
        }
        ev3_current = pose("ev3-local", 40, -10, 5_000)
        ev3_source = Provider(local_map(
            "ev3rstorm-01",
            "ev3-controller",
            "ev3-local",
            "ev3-generation-4",
            current_pose=ev3_current,
            history=[pose("ev3-local", 0, 0, 0), ev3_current],
            collision_geometry=circle,
            map_version=8,
            captured_at_unix_ms=1_100,
        ))
        blast_current = pose(
            "blast-local",
            100,
            0,
            -90_000,
            source_id="blast-navigation-motion-executor",
            provenance="PROVISIONAL_ENCODER_ODOMETRY",
        )
        blast_source = Provider(local_map(
            "blast-01",
            "blast-01.hub",
            "blast-local",
            "blast-episode-7",
            current_pose=blast_current,
            history=[pose("blast-local", 0, 0, 0), blast_current],
            collision_geometry=rectangle,
            map_version=3,
            captured_at_unix_ms=1_200,
        ))

        value = compositor(
            (ev3_source, ev3_transform),
            (blast_source, blast_transform),
        ).snapshot()

        self.assertEqual(value["schema"], "robot-spatial-map/v2")
        self.assertTrue(value["read_only"])
        self.assertEqual(value["status"], "available")
        self.assertEqual(value["reason_code"], "all_sources_available")
        self.assertEqual(value["frame_id"], WORLD_FRAME_ID)
        self.assertEqual(value["frame_kind"], "SHARED_FIXED_START")
        self.assertEqual(
            value["snapshot_semantics"], "LATEST_AVAILABLE_NOT_ATOMIC"
        )
        self.assertEqual(
            value["provenance"],
            "CALIBRATED_FIXED_START_SE2_PROJECTION",
        )
        self.assertEqual(
            value["world_generation_id"], WORLD_GENERATION_ID
        )
        self.assertEqual(value["captured_at_unix_ms"], 1_200)
        self.assertEqual(
            [robot["robot_id"] for robot in value["robots"]],
            ["blast-01", "ev3rstorm-01"],
        )

        blast = value["robots"][0]
        self.assertEqual(blast["status"], "available")
        self.assertEqual(blast["local_generation_id"], "blast-episode-7")
        self.assertEqual(blast["captured_at_unix_ms"], 1_200)
        self.assertEqual(blast["source_age_ms"], 0)
        self.assertEqual(
            (
                blast["robot_pose"]["x_mm"],
                blast["robot_pose"]["y_mm"],
                blast["robot_pose"]["heading_mdeg"],
            ),
            (1_000, 600, 0),
        )
        self.assertEqual(
            (
                blast["pose_history"][0]["x_mm"],
                blast["pose_history"][0]["y_mm"],
                blast["pose_history"][0]["heading_mdeg"],
            ),
            (1_000, 500, 90_000),
        )
        self.assertEqual(blast["collision_geometry"], rectangle)
        self.assertEqual(
            blast["frame_transform"]["position_uncertainty_mm"], 12
        )
        self.assertEqual(
            blast["frame_transform"]["yaw_uncertainty_mdeg"], 2_000
        )
        self.assertEqual(
            blast["frame_transform"]["provenance"],
            ["FIXED_START_POSE"],
        )

        ev3 = value["robots"][1]
        self.assertEqual(
            (
                ev3["robot_pose"]["x_mm"],
                ev3["robot_pose"]["y_mm"],
                ev3["robot_pose"]["heading_mdeg"],
            ),
            (40, -10, 5_000),
        )
        self.assertEqual(ev3["collision_geometry"], circle)

    def test_frame_or_generation_mismatch_isolated_as_unavailable(self):
        calibration = frame_transform(
            "blast-01",
            "blast-01.hub",
            "blast-local",
            "episode-current",
        )
        for field, wrong in (
            ("robot_id", "other-robot"),
            ("controller_instance_id", "other-controller"),
            ("frame_id", "retired-frame"),
            ("local_generation_id", "retired-episode"),
        ):
            value = local_map(
                "blast-01",
                "blast-01.hub",
                "blast-local",
                "episode-current",
            )
            value[field] = wrong
            with self.subTest(field=field):
                snapshot = compositor(
                    (Provider(value), calibration)
                ).snapshot()
                self.assertEqual(snapshot["status"], "unavailable")
                self.assertEqual(
                    snapshot["robots"][0]["reason_code"],
                    "source_identity_mismatch",
                )
                self.assertIsNone(snapshot["robots"][0]["robot_pose"])

    def test_incomplete_source_contract_is_not_drawn_as_available(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        for missing in (
            "map_id", "map_version", "captured_at_unix_ms", "age_ms",
        ):
            value = local_map(
                "blast-01", "blast-hub", "blast-local", "episode-a"
            )
            del value[missing]
            with self.subTest(missing=missing):
                robot = compositor(
                    (Provider(value), calibration)
                ).snapshot()["robots"][0]
                self.assertEqual(robot["status"], "unavailable")
                self.assertEqual(
                    robot["reason_code"], "source_contract_mismatch"
                )

        for missing in (
            "state_version", "source_id", "provenance",
            "observed_at_unix_ms", "age_ms",
        ):
            value = local_map(
                "blast-01", "blast-hub", "blast-local", "episode-a"
            )
            del value["robot_pose"][missing]
            with self.subTest(missing=missing):
                robot = compositor(
                    (Provider(value), calibration)
                ).snapshot()["robots"][0]
                self.assertEqual(robot["status"], "unavailable")
                self.assertEqual(robot["reason_code"], "source_pose_invalid")

    def test_one_broken_source_does_not_hide_the_healthy_robot(self):
        healthy_transform = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        broken_transform = frame_transform(
            "ev3rstorm-01", "ev3-controller", "ev3-local", "generation-a"
        )
        healthy = Provider(local_map(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        ))
        broken = Provider(error=RuntimeError("source disconnected"))

        value = compositor(
            (broken, broken_transform),
            (healthy, healthy_transform),
        ).snapshot()

        self.assertEqual(value["status"], "degraded")
        self.assertEqual(value["reason_code"], "some_sources_unavailable")
        self.assertEqual(value["robots"][0]["status"], "available")
        self.assertEqual(value["robots"][1]["status"], "unavailable")
        self.assertEqual(
            value["robots"][1]["reason_code"], "source_snapshot_failed"
        )

    def test_duplicate_bindings_and_invalid_provider_are_rejected(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        provider = Provider(local_map(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        ))
        with self.assertRaisesRegex(
            SharedSpatialMapError, "duplicate shared map binding"
        ):
            compositor(
                (provider, calibration),
                (provider, calibration),
            )
        with self.assertRaisesRegex(
            SharedSpatialMapError, "provider snapshot"
        ):
            compositor((object(), calibration))

    def test_history_is_bounded_and_reports_local_eviction(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        history = [
            pose("blast-local", index, 0, 0, state_version=index + 1)
            for index in range(MAX_SHARED_POSE_HISTORY + 3)
        ]
        value = local_map(
            "blast-01",
            "blast-hub",
            "blast-local",
            "episode-a",
            current_pose=history[-1],
            history=history,
        )
        value["pose_history_evicted"] = 2

        robot = compositor(
            (Provider(value), calibration)
        ).snapshot()["robots"][0]

        self.assertEqual(len(robot["pose_history"]), MAX_SHARED_POSE_HISTORY)
        self.assertEqual(robot["pose_history"][0]["x_mm"], 3)
        self.assertEqual(robot["pose_history_evicted"], 5)

    def test_snapshot_only_reads_and_never_writes_or_closes_sources(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        provider = Provider(local_map(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        ))
        shared = compositor((provider, calibration))

        first = shared.snapshot()
        second = shared.snapshot()

        self.assertEqual(first["status"], "available")
        self.assertEqual(second["status"], "available")
        self.assertEqual(provider.snapshot_calls, 2)
        self.assertEqual(provider.write_calls, 0)
        self.assertEqual(provider.close_calls, 0)

    def test_returned_snapshot_is_deeply_detached(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        geometry = {
            "geometry": "SYMMETRIC_CIRCLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "radius_mm": 100,
        }
        source_value = local_map(
            "blast-01",
            "blast-hub",
            "blast-local",
            "episode-a",
            collision_geometry=geometry,
        )
        source_before = deepcopy(source_value)
        shared = compositor((Provider(source_value), calibration))

        first = shared.snapshot()
        first["robots"][0]["robot_pose"]["x_mm"] = 999
        first["robots"][0]["collision_geometry"]["radius_mm"] = 999
        first["robots"][0]["frame_transform"]["provenance"].append(
            "CHANGED"
        )

        second = shared.snapshot()
        self.assertEqual(second["robots"][0]["robot_pose"]["x_mm"], 0)
        self.assertEqual(
            second["robots"][0]["collision_geometry"]["radius_mm"], 100
        )
        self.assertEqual(
            second["robots"][0]["frame_transform"]["provenance"],
            ["FIXED_START_POSE"],
        )
        self.assertEqual(source_value, source_before)

    def test_no_cells_rays_hypotheses_or_geometry_facts_are_invented(self):
        calibration = frame_transform(
            "blast-01", "blast-hub", "blast-local", "episode-a"
        )
        geometry = {
            "geometry": "SYMMETRIC_CIRCLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "radius_mm": 100,
        }
        value = local_map(
            "blast-01",
            "blast-hub",
            "blast-local",
            "episode-a",
            collision_geometry=geometry,
        )
        value.update({
            "bounds": {"min_x_mm": -100, "max_x_mm": 100},
            "cells": [{"x_mm": 10, "y_mm": 20, "state": "OCCUPIED"}],
            "sensor_rays": [{"state": "MEASURED"}],
            "qualitative_observations": [{"relation": "NEAR_OBSTACLE"}],
            "scan_evidence_history": [{"scan_id": "scan-a"}],
            "object_hypotheses": [{"hypothesis_id": "object-a"}],
            "navigation_trace": {"planned_leg": "ADVANCE"},
        })

        shared = compositor((Provider(value), calibration)).snapshot()

        self.assertIsNone(shared["bounds"])
        self.assertEqual(shared["cells"], [])
        self.assertEqual(shared["sensor_rays"], [])
        self.assertEqual(shared["qualitative_observations"], [])
        self.assertEqual(shared["scan_evidence_history"], [])
        self.assertEqual(shared["object_hypotheses"], [])
        self.assertIsNone(shared["navigation_authority"])
        robot = shared["robots"][0]
        self.assertEqual(robot["collision_geometry"], geometry)
        for forbidden in (
            "bounds",
            "cells",
            "sensor_rays",
            "qualitative_observations",
            "scan_evidence_history",
            "object_hypotheses",
            "navigation_trace",
            "navigation_authority",
        ):
            self.assertNotIn(forbidden, robot)


if __name__ == "__main__":
    import unittest

    unittest.main()
