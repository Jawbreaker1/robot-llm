import math
from dataclasses import FrozenInstanceError, replace
from unittest import TestCase

from robot_agent.physical_odometry import PhysicalPose
from robot_agent.shared_frame_transform import (
    CalibratedFrameTransform,
    FrameTransformError,
    MAX_FRAME_COORDINATE_MM,
)


SOURCE = {
    "source_robot_id": "blast-01",
    "source_controller_id": "blast-01.hub",
    "source_frame_id": "blast-local",
    "source_generation_id": "blast-generation-7",
}
WORLD = {
    "world_frame_id": "shared-world",
    "world_generation_id": "world-generation-3",
}


def transform(**changes):
    values = {
        **SOURCE,
        **WORLD,
        "tx_mm": 0,
        "ty_mm": 0,
        "yaw_mdeg": 0,
        "position_uncertainty_mm": 12,
        "yaw_uncertainty_mdeg": 2_000,
        "provenance": ("FIXED_START_POSE",),
    }
    values.update(changes)
    return CalibratedFrameTransform(**values)


class SharedFrameTransformTests(TestCase):
    def test_identity_preserves_point_heading_pose_totals_and_is_frozen(self):
        calibration = transform()
        pose = PhysicalPose(
            x_mm=42,
            y_mm=-17,
            heading_mdeg=30_000,
            verified_motion_count=4,
            total_forward_mm=210,
            total_turn_mdeg=90_000,
        )

        self.assertEqual(
            calibration.to_world_point(42, -17, **SOURCE),
            (42, -17),
        )
        self.assertEqual(
            calibration.to_world_heading(30_000, **SOURCE),
            30_000,
        )
        self.assertEqual(calibration.to_world_pose(pose, **SOURCE), pose)
        with self.assertRaises(FrozenInstanceError):
            calibration.tx_mm = 1

    def test_translation_and_requested_blast_plus_90_fixture_are_exact(self):
        translated = transform(tx_mm=250, ty_mm=-80)
        self.assertEqual(
            translated.to_world_point(20, 30, **SOURCE),
            (270, -50),
        )

        calibration = transform(tx_mm=1_000, ty_mm=500, yaw_mdeg=90_000)
        pose = calibration.to_world_pose(
            PhysicalPose(x_mm=100, y_mm=0, heading_mdeg=-90_000),
            **SOURCE,
        )
        self.assertEqual(
            (pose.x_mm, pose.y_mm, pose.heading_mdeg),
            (1_000, 600, 0),
        )
        self.assertEqual(
            calibration.to_world_point(200, -50, **SOURCE),
            (1_050, 700),
        )

    def test_minus_90_rotation_and_heading_wrap(self):
        calibration = transform(yaw_mdeg=-90_000)
        self.assertEqual(
            calibration.to_world_point(200, -50, **SOURCE),
            (-50, -200),
        )
        self.assertEqual(
            transform(yaw_mdeg=30_000).to_world_heading(
                170_000,
                **SOURCE,
            ),
            -160_000,
        )
        self.assertEqual(
            transform(yaw_mdeg=-30_000).to_world_heading(
                -170_000,
                **SOURCE,
            ),
            160_000,
        )

    def test_inverse_round_trip_and_world_generation_fence(self):
        calibration = transform(
            tx_mm=1_000,
            ty_mm=500,
            yaw_mdeg=90_000,
        )
        point = calibration.to_world_point(200, -50, **SOURCE)
        heading = calibration.to_world_heading(-90_000, **SOURCE)

        self.assertEqual(
            calibration.to_source_point(*point, **WORLD),
            (200, -50),
        )
        self.assertEqual(
            calibration.to_source_heading(heading, **WORLD),
            -90_000,
        )
        with self.assertRaisesRegex(FrameTransformError, "World identity"):
            calibration.to_source_point(
                *point,
                world_frame_id=WORLD["world_frame_id"],
                world_generation_id="retired-world-generation",
            )

    def test_composition_matches_sequential_transforms(self):
        local_to_staging = transform(
            tx_mm=100,
            ty_mm=20,
            yaw_mdeg=90_000,
            world_frame_id="staging-frame",
            world_generation_id="staging-generation",
            provenance=("LOCAL_BIND",),
        )
        staging_to_world = transform(
            source_frame_id="staging-frame",
            source_generation_id="staging-generation",
            tx_mm=1_000,
            ty_mm=500,
            yaw_mdeg=-90_000,
            position_uncertainty_mm=5,
            yaw_uncertainty_mdeg=1_000,
            provenance=("WORLD_BIND",),
        )
        composed = local_to_staging.then(staging_to_world)
        point = (40, -30)
        staging_point = local_to_staging.to_world_point(*point, **SOURCE)
        expected = staging_to_world.to_world_point(
            *staging_point,
            source_robot_id=SOURCE["source_robot_id"],
            source_controller_id=SOURCE["source_controller_id"],
            source_frame_id="staging-frame",
            source_generation_id="staging-generation",
        )

        self.assertEqual(composed.to_world_point(*point, **SOURCE), expected)
        self.assertEqual(composed.yaw_mdeg, 0)
        self.assertEqual(composed.position_uncertainty_mm, 17)
        self.assertEqual(composed.yaw_uncertainty_mdeg, 3_000)
        self.assertEqual(
            composed.provenance,
            ("LOCAL_BIND", "WORLD_BIND"),
        )

    def test_source_identity_and_generation_must_match_exactly(self):
        calibration = transform()
        for field, wrong in (
            ("source_robot_id", "ev3rstorm-01"),
            ("source_controller_id", "other.controller"),
            ("source_frame_id", "other-frame"),
            ("source_generation_id", "retired-generation"),
        ):
            presented = dict(SOURCE)
            presented[field] = wrong
            with self.subTest(field=field):
                with self.assertRaisesRegex(
                    FrameTransformError,
                    "Source identity",
                ):
                    calibration.to_world_point(0, 0, **presented)

    def test_validation_rejects_non_integer_non_finite_and_out_of_bounds(self):
        invalid_changes = (
            {"tx_mm": True},
            {"tx_mm": 1.0},
            {"tx_mm": math.inf},
            {"tx_mm": math.nan},
            {"tx_mm": MAX_FRAME_COORDINATE_MM + 1},
            {"yaw_mdeg": 180_000},
            {"position_uncertainty_mm": -1},
            {"yaw_uncertainty_mdeg": 180_001},
            {"provenance": ()},
            {"source_generation_id": " stale"},
        )
        for changes in invalid_changes:
            with self.subTest(changes=changes):
                with self.assertRaises(FrameTransformError):
                    transform(**changes)

        calibration = transform()
        for coordinate in (True, 1.0, math.inf, math.nan):
            with self.subTest(coordinate=coordinate):
                with self.assertRaises(FrameTransformError):
                    calibration.to_world_point(coordinate, 0, **SOURCE)

    def test_to_dict_retains_bindings_uncertainty_and_provenance(self):
        calibration = transform(tx_mm=1_000, ty_mm=500, yaw_mdeg=90_000)

        self.assertEqual(
            calibration.to_dict(),
            {
                **SOURCE,
                **WORLD,
                "tx_mm": 1_000,
                "ty_mm": 500,
                "yaw_mdeg": 90_000,
                "position_uncertainty_mm": 12,
                "yaw_uncertainty_mdeg": 2_000,
                "provenance": ["FIXED_START_POSE"],
            },
        )

    def test_composition_rejects_wrong_intermediate_generation(self):
        inner = transform(
            world_frame_id="staging-frame",
            world_generation_id="staging-generation",
        )
        outer = transform(
            source_frame_id="staging-frame",
            source_generation_id="other-generation",
        )

        with self.assertRaisesRegex(
            FrameTransformError,
            "intermediate identity",
        ):
            inner.then(outer)


class SharedFrameTransformDataclassTests(TestCase):
    def test_replace_revalidates_immutable_calibration(self):
        calibration = transform()

        with self.assertRaises(FrameTransformError):
            replace(calibration, yaw_mdeg=180_000)
