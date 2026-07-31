import unittest

from robot_agent.navigation_state import (
    ClearanceEvidence,
    NavigationSnapshot,
    PoseEstimate,
)
from robot_agent.spatial_dashboard import spatial_dashboard_view
from robot_agent.spatial_map_contract import (
    DASHBOARD_SPATIAL_MAP_SCHEMA,
    LOCAL_ODOMETRY,
    SIMULATION_WORLD,
)
from robot_agent.spatial_mapping import (
    BoundedOccupancyGrid,
    SpatialMappingPolicy,
)


def mapped_snapshot():
    navigation = NavigationSnapshot(
        robot_id="robot-1",
        controller_instance_id="controller-1",
        goal_id="mapping",
        goal_epoch=1,
        plan_revision=1,
        state_version=1,
        world_model_version=1,
        captured_at_host_ms=10_000,
        state_observed_at_ms=9_990,
        pose=PoseEstimate(100, 200, 0),
        left_encoder_mdeg=0,
        right_encoder_mdeg=0,
        motors_running=False,
        touch_pressed=False,
        active_faults=(),
        clearance=ClearanceEvidence(
            source="simulation_metric",
            observed_at_ms=9_995,
            near_obstacle_latched=True,
            forward_mm=100,
            left_mm=200,
            right_mm=200,
            forward_object_id="box-1",
        ),
    )
    grid = BoundedOccupancyGrid(
        map_id="map-1",
        robot_id="robot-1",
        controller_instance_id="controller-1",
        frame_id="sim-world",
        frame_kind=SIMULATION_WORLD,
        policy=SpatialMappingPolicy(
            resolution_mm=50,
            range_max_mm=200,
        ),
    )
    grid.ingest(navigation)
    return grid.snapshot()


class SpatialDashboardViewTests(unittest.TestCase):
    def test_read_only_view_matches_dashboard_shape(self):
        value = spatial_dashboard_view(
            mapped_snapshot(),
            now_unix_ms=2_000_000,
        )

        self.assertEqual(
            value["schema"],
            DASHBOARD_SPATIAL_MAP_SCHEMA,
        )
        self.assertTrue(value["read_only"])
        self.assertEqual(value["status"], "available")
        self.assertEqual(value["based_on_state_version"], 1)
        self.assertEqual(value["based_on_world_model_version"], 1)
        self.assertIsNotNone(value["bounds"])
        self.assertEqual(value["robot_pose"]["x_mm"], 100)
        self.assertEqual(len(value["sensor_rays"]), 3)
        self.assertTrue(value["cells"])
        self.assertTrue(value["object_hypotheses"])
        cell = value["cells"][0]
        self.assertTrue({
            "x_mm",
            "y_mm",
            "size_mm",
            "state",
            "confidence_milli",
            "source_id",
            "provenance",
            "observed_at_unix_ms",
        }.issubset(cell))
        core_cell = mapped_snapshot().cells[0]
        self.assertEqual(cell["x_mm"], core_cell.center_x_mm)
        self.assertEqual(cell["y_mm"], core_cell.center_y_mm)

    def test_monotonic_times_are_not_mislabeled_as_unix(self):
        value = spatial_dashboard_view(
            mapped_snapshot(),
            now_unix_ms=2_000_000,
        )

        self.assertIsNone(value["observed_at_unix_ms"])
        self.assertIsNone(
            value["robot_pose"]["observed_at_unix_ms"]
        )
        self.assertTrue(all(
            item["observed_at_unix_ms"] is None
            for item in value["sensor_rays"]
        ))
        self.assertTrue(all(
            item["valid_until_unix_ms"] is None
            for item in value["sensor_rays"]
        ))
        self.assertTrue(all(
            item["observed_at_unix_ms"] is None
            for item in value["cells"]
        ))

    def test_explicit_age_bridge_produces_auditable_unix_times(self):
        value = spatial_dashboard_view(
            mapped_snapshot(),
            now_unix_ms=2_000_000,
            observed_age_ms=250,
        )

        self.assertEqual(value["observed_age_ms"], 250)
        self.assertEqual(
            value["observed_at_unix_ms"],
            1_999_750,
        )
        self.assertEqual(
            value["sensor_rays"][0]["age_ms"],
            250,
        )
        self.assertEqual(
            value["sensor_rays"][0]["observed_at_unix_ms"],
            1_999_750,
        )

    def test_physical_ir_is_visible_but_never_drawn_as_metric(self):
        navigation = NavigationSnapshot(
            robot_id="robot-1",
            controller_instance_id="controller-1",
            goal_id="mapping",
            goal_epoch=1,
            plan_revision=1,
            state_version=1,
            world_model_version=1,
            captured_at_host_ms=10_000,
            state_observed_at_ms=9_990,
            pose=PoseEstimate(100, 200, 0),
            left_encoder_mdeg=0,
            right_encoder_mdeg=0,
            motors_running=False,
            touch_pressed=False,
            active_faults=(),
            clearance=ClearanceEvidence(
                source="physical_ir_reflection",
                observed_at_ms=9_995,
                near_obstacle_latched=True,
                raw_ir_proximity=82,
            ),
        )
        grid = BoundedOccupancyGrid(
            map_id="physical-map",
            robot_id="robot-1",
            controller_instance_id="controller-1",
            frame_id="local-odometry",
            frame_kind=LOCAL_ODOMETRY,
        )
        grid.ingest(navigation)

        value = spatial_dashboard_view(
            grid.snapshot(),
            now_unix_ms=2_000_000,
            observed_age_ms=250,
            ray_ttl_ms=5_000,
        )

        self.assertEqual(value["status"], "qualitative_only")
        self.assertEqual(
            value["reason_code"],
            "provisional_ir_only",
        )
        self.assertIsNone(value["bounds"])
        self.assertEqual(value["cells"], [])
        self.assertEqual(len(value["qualitative_observations"]), 1)
        self.assertEqual(len(value["object_hypotheses"]), 1)
        observation = value["qualitative_observations"][0]
        self.assertEqual(observation["relation"], "NEAR_OBSTACLE")
        self.assertEqual(observation["raw_ir_proximity"], 82)
        self.assertEqual(observation["provenance"], "PROVISIONAL_IR")
        self.assertTrue(observation["provisional"])
        ray = value["sensor_rays"][0]
        self.assertIsNone(ray["origin_x_mm"])
        self.assertIsNone(ray["end_x_mm"])
        self.assertEqual(
            value["robot_pose"]["provenance"],
            "LOCAL_ODOMETRY",
        )
        hypothesis = value["object_hypotheses"][0]
        self.assertTrue(hypothesis["provisional"])
        self.assertIsNone(hypothesis["x_mm"])
        self.assertIsNone(hypothesis["y_mm"])
        self.assertIsNone(hypothesis["bounds"])
        self.assertEqual(
            hypothesis["geometry_kind"],
            "QUALITATIVE_FORWARD_ENVELOPE",
        )
        self.assertEqual(
            hypothesis["anchor_pose"],
            {
                "x_mm": 100,
                "y_mm": 200,
                "heading_mdeg": 0,
            },
        )
        self.assertEqual(
            hypothesis["source_id"],
            "physical_ir_reflection",
        )
        self.assertIn(
            "LOCAL_ODOMETRY_POSE",
            hypothesis["provenance"],
        )
        self.assertLessEqual(hypothesis["confidence_milli"], 400)


if __name__ == "__main__":
    unittest.main()
