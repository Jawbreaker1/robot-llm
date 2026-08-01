import unittest

from robot_agent.active_ir_scan_contract import ActiveIrScanCalibration
from robot_agent.physical_action_feasibility import (
    navigation_action_feasibility,
)
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    EXPECTED_ACTION_SPECS,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from robot_agent.physical_odometry import OdometryCalibration, PhysicalPose
from robot_agent.provisional_hazard_map import (
    HazardMapCalibration,
    ProvisionalHazard,
    ProvisionalHazardMap,
)


def mapped_hazard(*, footprint=None):
    return ProvisionalHazardMap(
        frame_id="frame-a",
        map_generation_id="generation-a",
        calibration=HazardMapCalibration(robot_footprint=footprint),
        hazards=(
            ProvisionalHazard(
                hypothesis_id="box-a",
                frame_id="frame-a",
                anchor_x_mm=0,
                anchor_y_mm=0,
                anchor_heading_mdeg=0,
                centroid_x_mm=140,
                centroid_y_mm=0,
                radius_mm=70,
                first_seen_at_ms=1,
                last_seen_at_ms=1,
                evidence_count=1,
                last_state_version=1,
                last_raw_ir_proximity=31,
                last_filtered_ir_proximity=32,
            ),
        ),
    )


class PhysicalActionFeasibilityTests(unittest.TestCase):
    def test_asymmetric_body_publishes_only_reverse_near_hazard(self):
        footprint = RobotFootprint(
            front_extent_mm=110,
            rear_extent_mm=90,
            left_extent_mm=105,
            right_extent_mm=160,
            clearance_margin_mm=10,
            calibration_status="provisional-unmeasured",
            calibration_evidence="operator observed right-arm contact",
        )

        result = navigation_action_feasibility(
            hazard_map=mapped_hazard(footprint=footprint),
            pose=PhysicalPose(),
            action_specs=EXPECTED_ACTION_SPECS,
            odometry_calibration=OdometryCalibration(),
            active_scan_calibration=ActiveIrScanCalibration(
                alignment_tolerance_mdeg=10_000,
            ),
        )

        self.assertFalse(result["motion_actions"][ADVANCE]["allowed"])
        self.assertTrue(result["motion_actions"][REVERSE]["allowed"])
        self.assertFalse(
            result["motion_actions"][TURN_LEFT_90]["allowed"]
        )
        self.assertFalse(
            result["motion_actions"][TURN_RIGHT_90]["allowed"]
        )
        self.assertFalse(result["active_scan"]["allowed"])
        self.assertEqual(
            result["collision_geometry"]["right_extent_mm"],
            160,
        )
        self.assertFalse(result["host_ranked_or_selected_action"])

    def test_retreat_changes_scan_feasibility_without_host_route_choice(self):
        footprint = RobotFootprint(
            front_extent_mm=110,
            rear_extent_mm=90,
            left_extent_mm=105,
            right_extent_mm=160,
            clearance_margin_mm=10,
        )
        result = navigation_action_feasibility(
            hazard_map=mapped_hazard(footprint=footprint),
            pose=PhysicalPose(x_mm=-180),
            action_specs=EXPECTED_ACTION_SPECS,
            odometry_calibration=OdometryCalibration(),
            active_scan_calibration=ActiveIrScanCalibration(
                alignment_tolerance_mdeg=10_000,
            ),
        )

        self.assertTrue(result["active_scan"]["allowed"])
        self.assertEqual(
            result["active_scan"]["reason"],
            "in_place_rotation_clear",
        )

    def test_legacy_circle_does_not_block_in_place_scan(self):
        result = navigation_action_feasibility(
            hazard_map=mapped_hazard(),
            pose=PhysicalPose(),
            action_specs=EXPECTED_ACTION_SPECS,
            odometry_calibration=OdometryCalibration(),
            active_scan_calibration=ActiveIrScanCalibration(),
        )

        self.assertTrue(result["active_scan"]["allowed"])
        self.assertEqual(
            result["collision_geometry"]["geometry"],
            "SYMMETRIC_CIRCLE",
        )


if __name__ == "__main__":
    unittest.main()
