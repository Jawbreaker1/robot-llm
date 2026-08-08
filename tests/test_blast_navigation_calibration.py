import json
from dataclasses import replace
from pathlib import Path
import unittest

from robot_agent.blast_navigation_calibration import (
    BLAST_NAVIGATION_EVIDENCE_ID,
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
    BlastNavigationCalibration,
    BlastRangeSensorExtrinsics,
)
from robot_agent.physical_footprint import RobotFootprint


EVIDENCE_PATH = (
    Path(__file__).resolve().parents[1]
    / "docs"
    / "data"
    / "EXP-BLAST-NAV-CALIBRATION-20260808-001.json"
)


class BlastNavigationCalibrationTests(unittest.TestCase):
    def test_live_motion_scale_and_measured_geometry_are_typed(self):
        calibration = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION

        self.assertEqual(
            calibration.odometry.linear_mm_per_encoder_degree,
            0.5,
        )
        self.assertEqual(
            calibration.odometry.turn_mdeg_per_opposed_encoder_degree,
            490,
        )
        footprint, sensor = calibration.require_complete()
        self.assertEqual(
            (
                footprint.front_extent_mm,
                footprint.rear_extent_mm,
                footprint.left_extent_mm,
                footprint.right_extent_mm,
                footprint.clearance_margin_mm,
            ),
            (110, 60, 105, 100, 10),
        )
        self.assertEqual(
            (
                sensor.forward_offset_mm,
                sensor.left_offset_mm,
                sensor.yaw_mdeg,
            ),
            (110, 80, 0),
        )
        self.assertTrue(calibration.complete)

    def test_partial_range_sensor_measurement_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "extrinsics are partial"):
            BlastRangeSensorExtrinsics(
                forward_offset_mm=100,
                left_offset_mm=None,
                yaw_mdeg=0,
                calibration_status="test",
                calibration_evidence="partial fixture",
            )

    def test_complete_fixture_can_cross_the_activation_gate(self):
        footprint = RobotFootprint(
            front_extent_mm=100,
            rear_extent_mm=80,
            left_extent_mm=90,
            right_extent_mm=90,
            clearance_margin_mm=10,
            calibration_status="test",
            calibration_evidence="complete fixture",
        )
        sensor = BlastRangeSensorExtrinsics(
            forward_offset_mm=80,
            left_offset_mm=60,
            yaw_mdeg=0,
            calibration_status="test",
            calibration_evidence="complete fixture",
        )
        source = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        calibration = BlastNavigationCalibration(
            odometry=source.odometry,
            odometry_status=source.odometry_status,
            odometry_evidence=source.odometry_evidence,
            robot_footprint=footprint,
            footprint_status="test",
            footprint_evidence="complete fixture",
            range_sensor_extrinsics=sensor,
            evidence_id="test-evidence",
        )

        self.assertTrue(calibration.complete)
        self.assertEqual(calibration.require_complete(), (footprint, sensor))

    def test_both_geometry_parts_are_required_for_activation(self):
        source = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        assert source.robot_footprint is not None
        incomplete_sensor = BlastRangeSensorExtrinsics(
            forward_offset_mm=None,
            left_offset_mm=None,
            yaw_mdeg=None,
            calibration_status="unknown",
            calibration_evidence="incomplete fixture",
        )

        for calibration in (
            replace(source, robot_footprint=None),
            replace(source, range_sensor_extrinsics=incomplete_sensor),
        ):
            with self.subTest(calibration=calibration):
                self.assertFalse(calibration.complete)
                with self.assertRaisesRegex(
                    ValueError,
                    "geometry is incomplete",
                ):
                    calibration.require_complete()

    def test_evidence_record_matches_the_typed_values(self):
        evidence = json.loads(EVIDENCE_PATH.read_text(encoding="utf-8"))
        calibration = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION

        self.assertEqual(evidence["experiment_id"], BLAST_NAVIGATION_EVIDENCE_ID)
        self.assertEqual(
            evidence["drive"]["derived_linear_mm_per_encoder_degree"],
            calibration.odometry.linear_mm_per_encoder_degree,
        )
        self.assertEqual(
            evidence["turn"][
                "selected_turn_mdeg_per_actual_opposed_encoder_degree"
            ],
            calibration.odometry.turn_mdeg_per_opposed_encoder_degree,
        )
        samples = evidence["turn"]["body_turn_samples_degrees"]
        sample_mean = sum(samples) / len(samples)
        self.assertEqual(len(samples), evidence["turn"]["imu_sample_count"])
        self.assertAlmostEqual(
            sample_mean,
            evidence["turn"]["mean_body_turn_degrees"],
            places=12,
        )
        self.assertEqual(
            min(samples),
            evidence["turn"]["minimum_body_turn_degrees"],
        )
        self.assertEqual(
            max(samples),
            evidence["turn"]["maximum_body_turn_degrees"],
        )
        self.assertEqual(
            round(
                sample_mean
                / evidence["turn"][
                    "commanded_opposed_encoder_degrees_per_wheel"
                ]
                * 1_000
            ),
            evidence["turn"]["commanded_angle_ratio_mdeg_per_degree"],
        )
        validation = evidence["turn"]["quarter_turn_validation"]
        self.assertEqual(
            validation[
                "selected_provisional_turn_mdeg_per_actual_opposed_encoder_degree"
            ],
            calibration.odometry.turn_mdeg_per_opposed_encoder_degree,
        )
        self.assertEqual(
            validation["rounded_expected_encoder_degrees_per_action"],
            193,
        )
        self.assertEqual(
            validation["left"]["actual_opposed_encoder_degrees"],
            194.0,
        )
        self.assertEqual(
            validation["right_return"][
                "actual_opposed_encoder_degrees"
            ],
            191.5,
        )
        self.assertEqual(
            evidence["geometry"]["robot_footprint"]["status"],
            calibration.robot_footprint.calibration_status,
        )
        self.assertEqual(
            evidence["geometry"]["range_sensor_extrinsics"]["status"],
            calibration.range_sensor_extrinsics.calibration_status,
        )
        self.assertEqual(
            evidence["geometry"]["robot_footprint"]["extents_mm"],
            {
                "front": calibration.robot_footprint.front_extent_mm,
                "rear": calibration.robot_footprint.rear_extent_mm,
                "left": calibration.robot_footprint.left_extent_mm,
                "right": calibration.robot_footprint.right_extent_mm,
            },
        )
        self.assertEqual(
            evidence["geometry"]["robot_footprint"][
                "clearance_margin_mm"
            ],
            calibration.robot_footprint.clearance_margin_mm,
        )
        self.assertEqual(
            evidence["geometry"]["range_sensor_extrinsics"][
                "planar_pose"
            ],
            {
                "forward_offset_mm": (
                    calibration.range_sensor_extrinsics.forward_offset_mm
                ),
                "left_offset_mm": (
                    calibration.range_sensor_extrinsics.left_offset_mm
                ),
                "yaw_mdeg": calibration.range_sensor_extrinsics.yaw_mdeg,
            },
        )
        self.assertEqual(
            evidence["geometry"]["range_sensor_extrinsics"][
                "centre_height_mm_approx"
            ],
            205,
        )
        self.assertEqual(
            evidence["geometry"]["range_sensor_extrinsics"][
                "vertical_pitch"
            ],
            "not measured",
        )


if __name__ == "__main__":
    unittest.main()
