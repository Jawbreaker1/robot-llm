import json
import tempfile
import unittest
from pathlib import Path

from robot_agent.ev3rstorm_profile import (
    EV3RSTORM_PROFILE_ID,
    EV3RSTORMProfile,
    EV3SSHBinding,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    EXPECTED_ACTION_SPECS,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_odometry import PhysicalPose
from robot_agent.physical_scan_evidence import (
    ScanAttemptEvidence,
    ScanRayEvidence,
)
from robot_agent.provisional_hazard_map import (
    HazardMapCalibration,
    ProvisionalHazard,
    ProvisionalHazardMap,
)


def hazard_map(*, footprint=None, x_mm=100, y_mm=-120, radius_mm=10):
    frame_id = "test-frame"
    return ProvisionalHazardMap(
        frame_id=frame_id,
        map_generation_id="test-generation",
        calibration=HazardMapCalibration(robot_footprint=footprint),
        hazards=(
            ProvisionalHazard(
                hypothesis_id="box",
                frame_id=frame_id,
                anchor_x_mm=0,
                anchor_y_mm=0,
                anchor_heading_mdeg=0,
                centroid_x_mm=x_mm,
                centroid_y_mm=y_mm,
                radius_mm=radius_mm,
                first_seen_at_ms=1,
                last_seen_at_ms=1,
                evidence_count=1,
                last_state_version=1,
                last_raw_ir_proximity=30,
                last_filtered_ir_proximity=30,
            ),
        ),
    )


class ProvisionalHazardFootprintTests(unittest.TestCase):
    def setUp(self):
        self.pose = PhysicalPose()
        self.asymmetric = RobotFootprint(
            front_extent_mm=50,
            rear_extent_mm=40,
            left_extent_mm=50,
            right_extent_mm=140,
        )

    def test_default_circle_keeps_legacy_centerline_corridor(self):
        result = hazard_map().validate_swept_path(
            self.pose,
            ADVANCE,
            EXPECTED_ACTION_SPECS,
        )

        self.assertTrue(result["allowed"])
        self.assertEqual(
            result["collision_geometry"]["geometry"],
            "SYMMETRIC_CIRCLE",
        )

    def test_right_arm_vetoes_advance_that_centerline_would_allow(self):
        right = hazard_map(
            footprint=self.asymmetric,
        ).validate_swept_path(
            self.pose,
            ADVANCE,
            EXPECTED_ACTION_SPECS,
        )
        left = hazard_map(
            footprint=self.asymmetric,
            y_mm=120,
        ).validate_swept_path(
            self.pose,
            ADVANCE,
            EXPECTED_ACTION_SPECS,
        )

        self.assertFalse(right["allowed"])
        self.assertEqual(right["hazard_ids"], ["box"])
        self.assertTrue(left["allowed"])
        self.assertEqual(
            right["collision_geometry"]["right_extent_mm"],
            140,
        )

    def test_right_arm_has_directional_turn_sweep(self):
        obstacle = hazard_map(
            footprint=self.asymmetric,
            x_mm=100,
            y_mm=-100,
        )

        left_turn = obstacle.validate_swept_path(
            self.pose,
            TURN_LEFT_90,
            EXPECTED_ACTION_SPECS,
        )
        right_turn = obstacle.validate_swept_path(
            self.pose,
            TURN_RIGHT_90,
            EXPECTED_ACTION_SPECS,
        )

        self.assertFalse(left_turn["allowed"])
        self.assertTrue(right_turn["allowed"])

    def test_ev3_near_envelope_can_continue_monotonic_reverse_escape(self):
        profile = EV3RSTORMProfile()
        obstacle = hazard_map(
            footprint=profile.hazard_calibration.robot_footprint,
            x_mm=140,
            y_mm=0,
            radius_mm=70,
        )

        reverse = obstacle.validate_swept_path(
            PhysicalPose(x_mm=-51, heading_mdeg=66),
            REVERSE,
            EXPECTED_ACTION_SPECS,
            profile.odometry_calibration,
        )

        self.assertTrue(reverse["allowed"])
        self.assertEqual(reverse["monotonic_escape_hazard_ids"], ["box"])

    def test_blocked_scan_bearing_extends_collision_hypothesis(self):
        scan = ScanAttemptEvidence(
            scan_id="scan-wide-box-right-edge",
            completed_at_ms=2_000,
            status="CANCELLED",
            reason="bilateral_boundaries_not_observed",
            rays=(
                ScanRayEvidence(
                    requested_relative_bearing_mdeg=-30_000,
                    actual_relative_bearing_mdeg=-30_000,
                    blocked=True,
                    raw=30,
                    filtered=30,
                ),
            ),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
            scan_pose=PhysicalPose(),
            based_on_map_version=1,
        )
        distant_primary = ProvisionalHazard(
            hypothesis_id="wide-box",
            frame_id="test-frame",
            anchor_x_mm=0,
            anchor_y_mm=0,
            anchor_heading_mdeg=0,
            centroid_x_mm=500,
            centroid_y_mm=500,
            radius_mm=20,
            first_seen_at_ms=1,
            last_seen_at_ms=1,
            evidence_count=1,
            last_state_version=1,
            last_raw_ir_proximity=30,
            last_filtered_ir_proximity=30,
            scan_evidence_history=(scan,),
        )
        calibration = HazardMapCalibration(
            provisional_hazard_offset_mm=140,
            provisional_hazard_radius_mm=20,
            robot_footprint=RobotFootprint(
                front_extent_mm=50,
                rear_extent_mm=40,
                left_extent_mm=50,
                right_extent_mm=100,
            ),
        )
        without_scan = ProvisionalHazardMap(
            frame_id="test-frame",
            map_generation_id="test-generation",
            calibration=calibration,
            hazards=(
                ProvisionalHazard(
                    **{
                        **distant_primary.__dict__,
                        "scan_evidence_history": (),
                        "collision_supports": (),
                    }
                ),
            ),
        )
        with_scan = ProvisionalHazardMap(
            frame_id="test-frame",
            map_generation_id="test-generation",
            calibration=calibration,
            hazards=(distant_primary,),
        )

        self.assertTrue(without_scan.validate_swept_path(
            self.pose,
            ADVANCE,
            EXPECTED_ACTION_SPECS,
        )["allowed"])
        blocked = with_scan.validate_swept_path(
            self.pose,
            ADVANCE,
            EXPECTED_ACTION_SPECS,
        )
        self.assertFalse(blocked["allowed"])
        self.assertEqual(blocked["hazard_ids"], ["wide-box"])
        context = with_scan.context()["navigation_hazard_hypotheses"][0]
        self.assertEqual(context["collision_support_count"], 2)
        self.assertTrue(context["active_for_collision"])

        detail_pruned = ProvisionalHazardMap(
            frame_id="test-frame",
            map_generation_id="test-generation",
            calibration=calibration,
            hazards=(
                ProvisionalHazard(
                    **{
                        **distant_primary.__dict__,
                        "scan_evidence_history": (),
                    }
                ),
            ),
        )
        retained = detail_pruned.validate_swept_path(
            self.pose,
            ADVANCE,
            EXPECTED_ACTION_SPECS,
        )
        self.assertFalse(retained["allowed"])
        self.assertEqual(retained["hazard_ids"], ["wide-box"])

    def test_scan_rotation_detects_arm_sweep_but_circle_adds_no_sweep(self):
        asymmetric = hazard_map(
            footprint=self.asymmetric,
            x_mm=100,
            y_mm=-100,
        ).validate_in_place_rotation(
            self.pose,
            (-60_000, -30_000, 0, 30_000, 60_000),
            alignment_tolerance_mdeg=10_000,
        )
        default_circle = hazard_map(
            x_mm=70,
            y_mm=0,
        ).validate_in_place_rotation(
            self.pose,
            (-60_000, 0, 60_000),
        )

        self.assertFalse(asymmetric["allowed"])
        self.assertEqual(
            asymmetric["reason"],
            "provisional_hazard_rotation_sweep_collision",
        )
        self.assertEqual(
            asymmetric["minimum_relative_heading_mdeg"],
            -70_000,
        )
        self.assertEqual(
            asymmetric["maximum_relative_heading_mdeg"],
            70_000,
        )
        self.assertTrue(default_circle["allowed"])

    def test_new_ev3_hazard_requires_backing_before_body_scan(self):
        profile = EV3RSTORMProfile()
        map_value = ProvisionalHazardMap(
            frame_id="live-frame",
            map_generation_id="live-generation",
            calibration=profile.hazard_calibration,
        )
        map_value.record_observation(
            self.pose,
            {
                "state_version": 1,
                "infrared": {
                    "blocked": True,
                    "raw": 30,
                    "filtered": 30,
                },
            },
            observed_at_ms=1,
        )

        advance = map_value.validate_swept_path(
            self.pose,
            ADVANCE,
            EXPECTED_ACTION_SPECS,
        )
        reverse = map_value.validate_swept_path(
            self.pose,
            REVERSE,
            EXPECTED_ACTION_SPECS,
        )
        close_scan = map_value.validate_in_place_rotation(
            self.pose,
            (-60_000, 0, 60_000),
            alignment_tolerance_mdeg=10_000,
        )
        backed_scan = map_value.validate_in_place_rotation(
            PhysicalPose(x_mm=-58),
            (-60_000, 0, 60_000),
            alignment_tolerance_mdeg=10_000,
        )

        self.assertFalse(advance["allowed"])
        self.assertTrue(reverse["allowed"])
        self.assertFalse(close_scan["allowed"])
        self.assertTrue(backed_scan["allowed"])

    def test_ev3_profile_injects_operator_measured_asymmetric_geometry(self):
        profile = EV3RSTORMProfile()
        footprint = profile.hazard_calibration.robot_footprint

        self.assertIsNotNone(footprint)
        self.assertGreater(
            footprint.right_extent_mm,
            footprint.left_extent_mm,
        )
        with tempfile.TemporaryDirectory() as directory:
            adapter = profile.build_adapter(
                EV3SSHBinding(
                    profile_id=EV3RSTORM_PROFILE_ID,
                    target="robot@ev3dev.local",
                    memory_path=Path(directory) / "memory.json",
                ),
                planner_factory=lambda _model: object(),
            )
            memory = adapter.memory_factory()

        self.assertIs(
            memory.hazard_map.calibration,
            profile.hazard_calibration,
        )
        self.assertEqual(
            memory.context()["collision_geometry"]["right_extent_mm"],
            footprint.right_extent_mm,
        )
        self.assertEqual(
            memory.context()["collision_geometry"][
                "calibration_status"
            ],
            "operator-measured-current-build",
        )
        self.assertIn(
            "100 mm left and 130 mm right",
            memory.context()["collision_geometry"][
                "calibration_evidence"
            ],
        )

    def test_schema_v1_config_without_footprint_uses_circle_fallback(self):
        checked_in = EV3RSTORMProfile()
        value = json.loads(
            checked_in.config_path.read_text(encoding="utf-8")
        )
        del value["calibration"]["physical_footprint"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy-ev3rstorm.json"
            path.write_text(json.dumps(value), encoding="utf-8")
            legacy = EV3RSTORMProfile(path)

        self.assertIsNone(legacy.hazard_calibration.robot_footprint)
        self.assertEqual(
            legacy.hazard_calibration.collision_geometry()["geometry"],
            "SYMMETRIC_CIRCLE",
        )


if __name__ == "__main__":
    unittest.main()
