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


DENSE_NINE_SIDES = (
    "center", "left_1", "left_2", "left_3", "left_4",
    "right_1", "right_2", "right_3", "right_4",
)


def scan_result(ranges=(300.0, 2_000.0, 2_000.0, 2_000.0, 2_000.0)):
    relative = (0.0, -44.1, -88.2, 44.1, 88.2)
    turn_scale = 0.490

    def encoder_delta(bearing):
        opposed = round(abs(bearing) / turn_scale)
        if bearing < 0:
            return {"left_drive": -opposed, "right_drive": opposed}
        if bearing > 0:
            return {"left_drive": opposed, "right_drive": -opposed}
        return {"left_drive": 0, "right_drive": 0}

    return {
        "schema": SCAN_RESULT_SCHEMA,
        "state": "complete",
        "result": "restored",
        "bearing_source": "DRIVE_ENCODER_ODOMETRY",
        "bearing_frame": "ROBOT_RELATIVE_AT_SCAN_START",
        "start_heading_deg": 0.0,
        "final_heading_deg": 0.0,
        "restoration_error_deg": 0.0,
        "restoration_verified": True,
        "encoder_start_angles_deg": {
            "left_drive": 100, "right_drive": 200,
        },
        "encoder_final_angles_deg": {
            "left_drive": 100, "right_drive": 200,
        },
        "encoder_restoration": {
            "common_mode_residue_mm": 0.0,
            "opposed_residue_deg": 0.0,
            "motion_stopped": True,
            "observation_settled": True,
            "body_pose_verified": True,
        },
        "imu_heading_diagnostics": {
            "authority": "DIAGNOSTIC_ONLY",
            "start_heading_deg": 0.0,
            "final_heading_deg": 0.0,
            "restoration_error_deg": 0.0,
        },
        "all_observations_settled": True,
        "rays": [
            {
                "side": side,
                "distance_mm": distance,
                "range_state": blast_range_state(distance),
                "body_motor_angle_deg": 158,
                "heading_deg": heading,
                "relative_heading_deg": heading,
                "imu_heading_deg": heading,
                "drive_encoder_delta_deg": encoder_delta(heading),
                "observation_settled": True,
            }
            for side, distance, heading in zip(
                SCAN_RAY_SIDES,
                ranges,
                relative,
            )
        ],
    }


def dense_scan_result(ranges, relative):
    value = scan_result()
    turn_scale = 0.490

    def encoder_evidence(bearing):
        opposed = round(abs(bearing) / turn_scale)
        if bearing < 0:
            delta = {"left_drive": -opposed, "right_drive": opposed}
            return delta, -opposed * turn_scale
        if bearing > 0:
            delta = {"left_drive": opposed, "right_drive": -opposed}
            return delta, opposed * turn_scale
        return {"left_drive": 0, "right_drive": 0}, 0.0

    value["angular_rays"] = [
        {
            "side": side,
            "distance_mm": distance,
            "range_state": blast_range_state(distance),
            "body_motor_angle_deg": 158,
            "heading_deg": encoder_evidence(heading)[1],
            "relative_heading_deg": encoder_evidence(heading)[1],
            "imu_heading_deg": heading,
            "drive_encoder_delta_deg": encoder_evidence(heading)[0],
            "observation_settled": True,
        }
        for side, distance, heading in zip(
            DENSE_NINE_SIDES, ranges, relative,
        )
    ]
    value["rays"] = [
        {
            **deepcopy(value["angular_rays"][dense]),
            "side": side,
        }
        for dense, side in (
            (0, "center"),
            (2, "left_near"),
            (4, "left_far"),
            (6, "right_near"),
            (8, "right_far"),
        )
    ]
    return value


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
    def test_calibrated_wide_scan_projects_both_ninety_five_degree_edges(self):
        scan = dense_scan_result(
            (300,) * 9,
            (0.0, -23.52, -47.04, -70.56, -94.57,
             23.52, 47.04, 70.56, 94.57),
        )

        points = projected_points(scan)

        self.assertEqual(
            [points[index]["relative_bearing_mdeg"] for index in (4, 8)],
            [94_570, -94_570],
        )

    def test_scan_bearing_beyond_full_circle_half_plane_is_rejected(self):
        scan = dense_scan_result(
            (300,) * 9,
            (0.0, -45.0, -90.0, -135.0, -180.45,
             45.0, 90.0, 135.0, 180.45),
        )

        with self.assertRaises(ValueError):
            projected_points(scan)

    def test_live_dense_scan_projects_all_nine_settled_measured_rays(self):
        scan = dense_scan_result(
            (265, 285, 310, 351, 379, 250, 250, 251, 1_228),
            (0.0, -9.98, -21.16, -32.58, -43.90,
             11.48, 22.21, 33.75, 44.56),
        )

        points = projected_points(scan)

        self.assertEqual(
            [point["side"] for point in points],
            list(DENSE_NINE_SIDES),
        )
        right_three = points[7]
        self.assertEqual(
            (
                right_three["relative_bearing_mdeg"],
                right_three["nominal_echo_x_mm"],
                right_three["nominal_echo_y_mm"],
            ),
            (-33_810, 344, -134),
        )

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
            ("left_far", 88_200, -77, 112, -67, 412),
        )
        self.assertEqual(
            tuple(right[field] for field in _SIDE_FIELDS),
            ("right_far", -88_200, 83, -107, 93, -407),
        )

    def test_heading_wrap_is_normalized(self):
        scan = scan_result((2_000.0, 100.0, 2_000.0, 2_000.0, 2_000.0))
        scan["rays"][1]["heading_deg"] = -29.89
        scan["rays"][1]["relative_heading_deg"] = -29.89
        scan["rays"][1]["drive_encoder_delta_deg"] = {
            "left_drive": -61, "right_drive": 61,
        }
        point = projected_points(
            scan,
            PhysicalPose(heading_mdeg=170_000),
        )[0]

        self.assertEqual(point["relative_bearing_mdeg"], 29_890)
        self.assertEqual(point["beam_heading_mdeg"], -160_110)

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

    def test_unsettled_far_ray_is_excluded_from_partial_projection(self):
        scan = scan_result((300.0, 1_489.0, 500.0, 600.0, 700.0))
        scan["all_observations_settled"] = False
        scan["rays"][1].update({
            "observation_settled": False,
            "evidence_use": "SWEEP_CONTINUATION_ONLY",
        })

        points = projected_points(scan)

        self.assertEqual(
            [point["side"] for point in points],
            ["center", "left_far", "right_near", "right_far"],
        )
        self.assertNotIn(1_489.0, (
            point["measured_range_mm"] for point in points
        ))

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
