from copy import deepcopy
from dataclasses import replace
from unittest import TestCase, mock

from robot_agent.blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from robot_agent.blast_observation_monitor import (
    SCAN_RAY_SIDES,
    SCAN_RESULT_SCHEMA,
    blast_range_state,
)
from robot_agent.blast_scan_planar_projection import (
    project_blast_scan_planar_surfaces,
)
from robot_agent.physical_odometry import PhysicalPose


def scan_result(ranges=(300.0, 2_000.0, 2_000.0, 2_000.0, 2_000.0)):
    relative = (0.0, -45.0, -90.0, 45.0, 90.0)
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
                "range_state": blast_range_state(distance),
                "body_motor_angle_deg": 158,
                "heading_deg": heading,
                "relative_heading_deg": heading,
                "observation_settled": True,
            }
            for side, distance, heading in zip(
                SCAN_RAY_SIDES,
                ranges,
                relative,
            )
        ],
    }


def projected_points(scan=None, pose=None):
    return project_blast_scan_planar_surfaces(
        scan=scan_result() if scan is None else scan,
        scan_pose=PhysicalPose() if pose is None else pose,
    )["points"]


_GEOMETRY_FIELDS = (
    "sensor_origin_x_mm",
    "sensor_origin_y_mm",
    "beam_heading_mdeg",
    "nominal_echo_x_mm",
    "nominal_echo_y_mm",
)
_SIDE_FIELDS = (
    "side",
    "relative_bearing_mdeg",
    "sensor_origin_x_mm",
    "sensor_origin_y_mm",
    "nominal_echo_x_mm",
    "nominal_echo_y_mm",
)


class BlastScanPlanarProjectionTests(TestCase):
    def test_center_echo_uses_extrinsics_and_pose_heading(self):
        cases = (
            (PhysicalPose(), (110, 80, 0, 410, 80)),
            (PhysicalPose(heading_mdeg=90_000), (-80, 110, 90_000, -80, 410)),
        )
        for pose, expected in cases:
            with self.subTest(pose=pose):
                point = projected_points(pose=pose)[0]
                self.assertEqual(
                    tuple(point[field] for field in _GEOMETRY_FIELDS),
                    expected,
                )
                self.assertEqual(point["measured_range_mm"], 300.0)

    def test_each_side_rotates_its_own_sensor_origin(self):
        scan = scan_result((2_000.0, 2_000.0, 300.0, 2_000.0, 300.0))
        left, right = projected_points(scan)

        self.assertEqual(
            tuple(left[field] for field in _SIDE_FIELDS),
            ("left_far", 90_000, -80, 110, -80, 410),
        )
        self.assertEqual(
            tuple(right[field] for field in _SIDE_FIELDS),
            ("right_far", -90_000, 80, -110, 80, -410),
        )

    def test_heading_wrap_is_normalized(self):
        scan = scan_result((2_000.0, 100.0, 2_000.0, 2_000.0, 2_000.0))
        scan["rays"][1]["heading_deg"] = -30.0
        scan["rays"][1]["relative_heading_deg"] = -30.0
        point = projected_points(
            scan,
            PhysicalPose(heading_mdeg=170_000),
        )[0]

        self.assertEqual(point["relative_bearing_mdeg"], 30_000)
        self.assertEqual(point["beam_heading_mdeg"], -160_000)

    def test_sensor_yaw_rotates_beam_but_not_mounted_origin(self):
        calibration = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        sensor = replace(
            calibration.range_sensor_extrinsics,
            yaw_mdeg=90_000,
        )
        with mock.patch(
            "robot_agent.blast_scan_planar_projection."
            "BLAST_PROVISIONAL_NAVIGATION_CALIBRATION",
            replace(calibration, range_sensor_extrinsics=sensor),
        ):
            point = projected_points()[0]

        self.assertEqual(
            tuple(point[field] for field in _GEOMETRY_FIELDS),
            (110, 80, 90_000, 110, 380),
        )

    def test_only_measured_ranges_create_points_in_ray_order(self):
        scan = scan_result((1_999.0, 2_000.0, None, 100.0, 2_000.0))
        original = deepcopy(scan)

        points = projected_points(scan)

        self.assertEqual(
            [point["side"] for point in points],
            ["center", "right_near"],
        )
        self.assertEqual(points[0]["measured_range_mm"], 1_999.0)
        self.assertEqual(scan, original)
        self.assertEqual(
            projected_points(scan_result((2_000.0,) * 5)),
            [],
        )

    def test_unready_scan_or_sensor_pose_is_rejected(self):
        mutations = (
            lambda value: value.update(restoration_verified=False),
            lambda value: value.update(all_observations_settled=False),
            lambda value: value["rays"][2].update(observation_settled=False),
            lambda value: value["rays"][2].update(body_motor_angle_deg=160),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                scan = scan_result()
                mutate(scan)
                with self.assertRaises(ValueError):
                    project_blast_scan_planar_surfaces(
                        scan=scan,
                        scan_pose=PhysicalPose(),
                    )
        with self.assertRaises(ValueError):
            project_blast_scan_planar_surfaces(
                scan=scan_result(),
                scan_pose="not-a-pose",
            )

    def test_body_pose_tolerance_boundary_is_projection_ready(self):
        scan = scan_result((100.0,) * 5)
        for ray in scan["rays"]:
            ray["body_motor_angle_deg"] = 159

        self.assertEqual(
            len(project_blast_scan_planar_surfaces(
                scan=scan,
                scan_pose=PhysicalPose(),
            )["points"]),
            5,
        )

    def test_heading_topology_and_v3_contract_fail_closed(self):
        mutations = (
            lambda value: value.update(schema="blast-scan-front-arc/v2"),
            lambda value: value["rays"][0].update(relative_heading_deg=1.0),
            lambda value: value["rays"][1].update(relative_heading_deg=1.0),
            lambda value: value["rays"][2].update(relative_heading_deg=-20.0),
            lambda value: value["rays"][3].update(relative_heading_deg=float("nan")),
            lambda value: value["rays"][4].update(range_state="MEASURED"),
            lambda value: value["rays"][1].update(heading_deg=-44.0),
            lambda value: value.update(final_heading_deg=1.0),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                scan = scan_result()
                mutate(scan)
                with self.assertRaises(ValueError):
                    project_blast_scan_planar_surfaces(
                        scan=scan,
                        scan_pose=PhysicalPose(),
                    )

    def test_output_contract_contains_no_map_claims(self):
        projection = project_blast_scan_planar_surfaces(
            scan=scan_result(),
            scan_pose=PhysicalPose(),
        )
        self.assertEqual(projection["quality"], "PROVISIONAL_YAW_ONLY")
        self.assertFalse(projection["vertical_pitch_compensated"])
        fields = set(projection) | set(projection["points"][0])
        self.assertTrue(
            fields.isdisjoint({"clear", "occupied", "radius", "target_id"})
        )
