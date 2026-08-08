import unittest
from dataclasses import replace

from robot_agent.blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
    BlastRangeSensorExtrinsics,
)
from robot_agent.blast_scan_route_candidates import (
    CANDIDATE_SCHEMA,
    build_blast_scan_route_candidates,
)
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_odometry import PhysicalPose


RAY_NAMES = (
    "center",
    "left_near",
    "left_far",
    "right_near",
    "right_far",
)
RAY_HEADINGS = (0, -30, -60, 30, 60)


def scan(*, distances=(300, 900, 2_000, 900, 2_000)):
    return {
        "schema": "blast-scan-front-arc/v1",
        "state": "complete",
        "result": "restored",
        "restoration_verified": True,
        "all_observations_settled": True,
        "rays": [
            {
                "side": side,
                "distance_mm": distance,
                "relative_heading_deg": heading,
                "observation_settled": True,
            }
            for side, distance, heading in zip(
                RAY_NAMES,
                distances,
                RAY_HEADINGS,
            )
        ],
    }


def candidates(value=None, *, pose=PhysicalPose(), calibration=None):
    return build_blast_scan_route_candidates(
        scan() if value is None else value,
        scan_pose=pose,
        frame_id="blast-local-odometry",
        map_generation_id="episode-1",
        map_version=1,
        calibration=(
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
            if calibration is None
            else calibration
        ),
    )


class BlastScanRouteCandidateTests(unittest.TestCase):
    def test_projects_both_sides_with_blast_sign_and_per_ray_origin(self):
        result = candidates()

        self.assertEqual(result["schema"], CANDIDATE_SCHEMA)
        self.assertFalse(result["route_execution_authorized"])
        self.assertFalse(
            result["target"]["full_depth_clearance_proven"]
        )
        by_side = {
            item["detour_side"]: item
            for item in result["candidates"]
        }
        self.assertEqual(
            set(by_side),
            {"LEFT_OF_GOAL", "RIGHT_OF_GOAL"},
        )
        self.assertEqual(
            by_side["LEFT_OF_GOAL"]["projected_endpoint_mm"],
            [835, 574],
        )
        self.assertEqual(
            by_side["RIGHT_OF_GOAL"]["projected_endpoint_mm"],
            [915, -436],
        )
        self.assertGreater(
            by_side["LEFT_OF_GOAL"]["route"][
                "route_lateral_offset_mm"
            ],
            0,
        )
        self.assertLess(
            by_side["RIGHT_OF_GOAL"]["route"][
                "route_lateral_offset_mm"
            ],
            0,
        )

    def test_no_return_is_unknown_and_never_proves_an_opening(self):
        result = candidates(scan(distances=(300, 2_000, 2_000, 900, 2_000)))

        self.assertEqual(
            [item["detour_side"] for item in result["candidates"]],
            ["RIGHT_OF_GOAL"],
        )
        self.assertNotIn(
            [2_000, 0],
            result["target"]["support_points_mm"],
        )
        self.assertIsNone(candidates(scan(distances=(2_000,) * 5)))

    def test_measured_ray_must_reach_the_route_pass_plane(self):
        result = candidates(scan(distances=(300, 500, 2_000, 500, 2_000)))

        self.assertEqual(result["candidates"], [])

    def test_near_zero_sweep_does_not_claim_either_side(self):
        value = scan()
        for ray, heading in zip(
            value["rays"],
            (0, -0.01, -0.02, 0.01, 0.02),
        ):
            ray["relative_heading_deg"] = heading

        self.assertEqual(candidates(value)["candidates"], [])

    def test_outer_measured_ray_can_contradict_a_near_opening(self):
        for distances, expected_side in (
            ((300, 900, 50, 900, 2_000), "RIGHT_OF_GOAL"),
            ((300, 900, 2_000, 900, 50), "LEFT_OF_GOAL"),
        ):
            with self.subTest(distances=distances):
                result = candidates(scan(distances=distances))
                self.assertEqual(
                    [
                        item["detour_side"]
                        for item in result["candidates"]
                    ],
                    [expected_side],
                )

    def test_geometry_is_calibration_driven_not_box_specific(self):
        baseline = candidates()
        calibration = replace(
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
            robot_footprint=RobotFootprint(
                front_extent_mm=90,
                rear_extent_mm=50,
                left_extent_mm=80,
                right_extent_mm=70,
                clearance_margin_mm=5,
                calibration_status="test-measured",
                calibration_evidence="test fixture",
            ),
            range_sensor_extrinsics=BlastRangeSensorExtrinsics(
                forward_offset_mm=80,
                left_offset_mm=20,
                yaw_mdeg=0,
                calibration_status="test-measured",
                calibration_evidence="test fixture",
            ),
        )

        changed = candidates(calibration=calibration)

        self.assertNotEqual(
            baseline["target"]["centroid_mm"],
            changed["target"]["centroid_mm"],
        )
        self.assertNotEqual(
            baseline["candidates"][0]["route"][
                "route_lateral_offset_mm"
            ],
            changed["candidates"][0]["route"][
                "route_lateral_offset_mm"
            ],
        )

    def test_rejects_unrestored_unsettled_or_malformed_scans(self):
        changes = []
        unrestored = scan()
        unrestored["restoration_verified"] = False
        changes.append(unrestored)
        unsettled = scan()
        unsettled["rays"][2]["observation_settled"] = False
        changes.append(unsettled)
        wrong_order = scan()
        wrong_order["rays"][1], wrong_order["rays"][2] = (
            wrong_order["rays"][2],
            wrong_order["rays"][1],
        )
        changes.append(wrong_order)
        invalid_distance = scan()
        invalid_distance["rays"][0]["distance_mm"] = True
        changes.append(invalid_distance)
        reversed_bearings = scan()
        for ray, heading in zip(
            reversed_bearings["rays"],
            (0, 30, 60, -30, -60),
        ):
            ray["relative_heading_deg"] = heading
        changes.append(reversed_bearings)
        left_not_left = scan()
        for ray, heading in zip(
            left_not_left["rays"],
            (1, 0.5, 0, 2, 3),
        ):
            ray["relative_heading_deg"] = heading
        changes.append(left_not_left)

        for value in changes:
            with self.subTest(scan=value):
                self.assertIsNone(candidates(value))

    def test_candidate_is_frozen_to_the_scan_pose_and_bounded(self):
        pose = PhysicalPose(x_mm=25, y_mm=10)

        result = candidates(pose=pose)

        self.assertEqual(result["source_scan_pose"], pose.to_dict())
        self.assertLessEqual(len(result["candidates"]), 2)
        for item in result["candidates"]:
            self.assertEqual(
                item["route"]["created_pose"],
                pose.to_dict(),
            )
            self.assertLessEqual(len(item["route"]["waypoints"]), 5)


if __name__ == "__main__":
    unittest.main()
