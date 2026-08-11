import copy
import unittest

from robot_agent.blast_navigation_action_profile import (
    BLAST_NAVIGATION_ACTION_SPECS,
)
from robot_agent.blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from robot_agent.blast_observation_monitor import (
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    SCAN_RESULT_SCHEMA,
)
from robot_agent.blast_side_search_geometry import (
    TARGET_REACQUISITION_SEARCH_BASIS,
    side_search_progress,
    side_search_required_slots,
    side_search_scan_sweep_is_clear,
    target_reacquisition_resolved,
    target_reacquisition_waypoint,
)
from robot_agent.blast_scan_planar_projection import (
    project_blast_scan_planar_surfaces,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from robot_agent.physical_odometry import PhysicalPose, nominal_effect


def scan_result(distances):
    sides = ("center", "left_near", "left_far", "right_near", "right_far")
    headings = (0.0, -22.0, -45.0, 24.0, 47.0)
    return {
        "schema": SCAN_RESULT_SCHEMA,
        "state": "complete",
        "result": "restored",
        "start_heading_deg": 0.0,
        "final_heading_deg": 0.0,
        "restoration_error_deg": 0.0,
        "restoration_verified": True,
        "all_observations_settled": True,
        "rays": [
            {
                "side": side,
                "distance_mm": distance,
                "range_state": (
                    RANGE_STATE_NO_VALID_DISTANCE
                    if distance == 2_000 else RANGE_STATE_MEASURED
                ),
                "body_motor_angle_deg": 158,
                "heading_deg": heading,
                "relative_heading_deg": heading,
                "observation_settled": True,
            }
            for side, heading, distance in zip(sides, headings, distances)
        ],
    }


def view(pose, distances, points):
    return {
        "scan_pose": pose.to_dict(),
        "scan": scan_result(distances),
        "planar_projection": {
            "schema": "blast-planar-scan-projection/v1",
            "frame": "EPISODE_LOCAL_ODOMETRY",
            "quality": "PROVISIONAL_YAW_ONLY",
            "points": [
                {
                    "side": side,
                    "measured_range_mm": 300.0,
                    "nominal_echo_x_mm": x_mm,
                    "nominal_echo_y_mm": y_mm,
                }
                for side, x_mm, y_mm in points
            ],
        },
    }


def nominal(pose, action):
    return nominal_effect(
        pose,
        action,
        BLAST_NAVIGATION_ACTION_SPECS,
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
    )[0]


class BlastTargetReacquisitionGeometryTests(unittest.TestCase):
    def setUp(self):
        self.origin = view(
            PhysicalPose(),
            (245, 300, 2_000, 300, 2_000),
            (
                ("center", 355, 80),
                ("left_near", 335, 228),
                ("right_near", 343, -69),
            ),
        )
        self.current = PhysicalPose(
            x_mm=-12,
            y_mm=368,
            heading_mdeg=-1_960,
            verified_motion_count=10,
            total_forward_mm=360,
            total_turn_mdeg=190_000,
        )
        self.failed = view(
            self.current,
            (1_352, 1_420, 2_000, 2_000, 2_000),
            (("center", 1_452, 398), ("left_near", 1_383, 1_002)),
        )

    def test_live_support_selects_six_pulse_inward_view(self):
        waypoint = target_reacquisition_waypoint(
            self.origin, self.failed, "LEFT", self.current
        )

        self.assertEqual(
            waypoint["search_basis"], TARGET_REACQUISITION_SEARCH_BASIS
        )
        self.assertEqual(waypoint["travel_side"], "RIGHT")
        self.assertEqual(waypoint["reacquisition_advance_count"], 6)
        self.assertEqual(waypoint["required_action_slots"], 9)
        self.assertEqual(side_search_required_slots(waypoint), 9)
        self.assertLessEqual(abs(waypoint["target_y_mm"] - 99), 3)
        self.assertLessEqual(
            abs(waypoint["predicted_target_bearing_mdeg"]), 30_000
        )
        self.assertEqual(waypoint["frozen_target_centroid_x_mm"], 355)
        self.assertEqual(waypoint["frozen_target_centroid_y_mm"], 80)
        self.assertGreaterEqual(waypoint["frozen_target_radius_mm"], 149)
        for fact in ("clearance_proven", "passage_proven", "route_eligible"):
            self.assertFalse(waypoint[fact])

    def test_live_reacquisition_pose_clears_two_pulse_scan_sweep(self):
        self.assertTrue(side_search_scan_sweep_is_clear(
            self.origin, self.current
        ))

    def test_scan_sweep_rejects_remembered_target_intersection(self):
        self.assertFalse(side_search_scan_sweep_is_clear(
            self.origin,
            PhysicalPose(x_mm=40, y_mm=40),
        ))

    def test_progress_orients_retraces_restores_and_rescans(self):
        waypoint = target_reacquisition_waypoint(
            self.origin, self.failed, "LEFT", self.current
        )

        progress = side_search_progress(self.current, waypoint)
        self.assertEqual(progress["phase"], "ORIENT_INWARD")
        self.assertEqual(progress["required_action"], TURN_RIGHT_90)

        pose = nominal(self.current, TURN_RIGHT_90)
        for _index in range(6):
            progress = side_search_progress(pose, waypoint)
            self.assertEqual(progress["phase"], "OUTBOUND")
            self.assertEqual(progress["required_action"], ADVANCE)
            pose = nominal(pose, ADVANCE)

        progress = side_search_progress(pose, waypoint)
        self.assertEqual(progress["phase"], "REORIENT")
        self.assertEqual(progress["required_action"], TURN_LEFT_90)
        pose = nominal(pose, TURN_LEFT_90)
        progress = side_search_progress(
            pose, waypoint, reorientation_attempted=True
        )
        self.assertEqual(progress["phase"], "RESCAN")
        self.assertEqual(progress["required_action"], SCAN_FRONT_ARC)

    def test_requires_exact_no_return_on_both_target_facing_rays(self):
        measured = copy.deepcopy(self.failed)
        measured["scan"]["rays"][3].update({
            "distance_mm": 500,
            "range_state": RANGE_STATE_MEASURED,
        })

        with self.assertRaises(ValueError):
            target_reacquisition_waypoint(
                self.origin, measured, "LEFT", self.current
            )

        unsettled = copy.deepcopy(self.failed)
        unsettled["scan"]["all_observations_settled"] = False
        unsettled["scan"]["rays"][3].update({
            "observation_settled": False,
            "evidence_use": "SWEEP_CONTINUATION_ONLY",
        })
        with self.assertRaises(ValueError):
            target_reacquisition_waypoint(
                self.origin, unsettled, "LEFT", self.current
            )

    def test_reacquisition_advance_vetoes_frozen_target_sweep(self):
        waypoint = target_reacquisition_waypoint(
            self.origin, self.failed, "LEFT", self.current
        )
        pose = nominal(self.current, TURN_RIGHT_90)
        blocked = dict(waypoint)
        blocked.update({
            "frozen_target_centroid_x_mm": pose.x_mm,
            "frozen_target_centroid_y_mm": pose.y_mm,
            "frozen_target_radius_mm": 1,
        })

        progress = side_search_progress(pose, blocked)

        self.assertEqual(progress["phase"], "OUTBOUND")
        self.assertIsNone(progress["required_action"])

    def test_reacquisition_turn_vetoes_frozen_target_sweep(self):
        scan = scan_result((55, 55, 2_000, 80, 2_000))
        origin = {
            "scan_pose": PhysicalPose().to_dict(),
            "scan": scan,
            "planar_projection": project_blast_scan_planar_surfaces(
                scan=scan, scan_pose=PhysicalPose(),
            ),
        }
        pose = PhysicalPose(y_mm=281)
        failed = view(
            pose,
            (1_352, 1_420, 2_000, 2_000, 2_000),
            (("center", 1_452, 311), ("left_near", 1_383, 900)),
        )
        waypoint = target_reacquisition_waypoint(
            origin, failed, "LEFT", pose,
        )

        progress = side_search_progress(pose, waypoint)

        self.assertEqual(progress["phase"], "ORIENT_INWARD")
        self.assertIsNone(progress["required_action"])

    def test_resolved_requires_compatible_target_facing_echo(self):
        waypoint = target_reacquisition_waypoint(
            self.origin, self.failed, "LEFT", self.current
        )
        scan_pose = PhysicalPose(
            x_mm=waypoint["target_x_mm"],
            y_mm=waypoint["target_y_mm"],
            heading_mdeg=self.current.heading_mdeg,
        )
        compatible = view(
            scan_pose,
            (900, 900, 2_000, 300, 2_000),
            (("center", 900, 100), ("right_near", 350, 70)),
        )
        background = copy.deepcopy(compatible)
        background["planar_projection"]["points"][1].update({
            "nominal_echo_x_mm": 900,
            "nominal_echo_y_mm": 500,
        })

        self.assertTrue(target_reacquisition_resolved(
            self.origin, compatible, "LEFT"
        ))
        self.assertFalse(target_reacquisition_resolved(
            self.origin, background, "LEFT"
        ))


if __name__ == "__main__":
    unittest.main()
