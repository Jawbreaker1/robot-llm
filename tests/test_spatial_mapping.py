from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import json
import unittest

from robot_agent.navigation_contract import NavigationContractError
from robot_agent.navigation_state import (
    ClearanceEvidence,
    NavigationSnapshot,
    PoseEstimate,
)
from robot_agent.spatial_map_contract import (
    CELL_FREE,
    CELL_OCCUPIED,
    CELL_UNKNOWN,
    LOCAL_ODOMETRY,
    MAP_PROVISIONAL_IR,
    MAP_SIMULATION_METRIC,
    LOCAL_ODOMETRY_POSE,
    PHYSICAL_IR_REFLECTION,
    QUALITATIVE_FORWARD_ENVELOPE,
    ProvisionalObjectHypothesis,
    SEMANTIC_UNKNOWN,
    SIMULATION_WORLD,
    STALE_STATE_VERSION,
)
from robot_agent.spatial_mapping import (
    BoundedOccupancyGrid,
    SpatialMappingPolicy,
)


def metric_snapshot(
    state_version=1,
    world_model_version=1,
    captured_at_ms=None,
    pose=PoseEstimate(0, 0, 0),
    forward_mm=100,
    left_mm=200,
    right_mm=200,
    object_id="box-1",
):
    captured = (
        state_version * 100
        if captured_at_ms is None
        else captured_at_ms
    )
    return NavigationSnapshot(
        robot_id="robot-1",
        controller_instance_id="controller-1",
        goal_id="mapping-probe",
        goal_epoch=1,
        plan_revision=1,
        state_version=state_version,
        world_model_version=world_model_version,
        captured_at_host_ms=captured,
        state_observed_at_ms=captured,
        pose=pose,
        left_encoder_mdeg=0,
        right_encoder_mdeg=0,
        motors_running=False,
        touch_pressed=False,
        active_faults=(),
        clearance=ClearanceEvidence(
            source="simulation_metric",
            observed_at_ms=captured,
            near_obstacle_latched=forward_mm < 200,
            forward_mm=forward_mm,
            left_mm=left_mm,
            right_mm=right_mm,
            forward_object_id=object_id,
        ),
    )


def physical_snapshot(
    state_version=1,
    captured_at_ms=None,
    near=True,
    raw_ir_proximity=82,
    pose=PoseEstimate(10, 20, 30_000),
    world_model_version=1,
):
    captured = (
        state_version * 100
        if captured_at_ms is None
        else captured_at_ms
    )
    return NavigationSnapshot(
        robot_id="robot-1",
        controller_instance_id="controller-1",
        goal_id="mapping-probe",
        goal_epoch=1,
        plan_revision=1,
        state_version=state_version,
        world_model_version=world_model_version,
        captured_at_host_ms=captured,
        state_observed_at_ms=captured,
        pose=pose,
        left_encoder_mdeg=0,
        right_encoder_mdeg=0,
        motors_running=False,
        touch_pressed=False,
        active_faults=(),
        clearance=ClearanceEvidence(
            source="physical_ir_reflection",
            observed_at_ms=captured,
            near_obstacle_latched=near,
            raw_ir_proximity=raw_ir_proximity,
        ),
    )


def simulator_grid(max_cells=128, resolution_mm=50):
    return BoundedOccupancyGrid(
        map_id="map-1",
        robot_id="robot-1",
        controller_instance_id="controller-1",
        frame_id="sim-world",
        frame_kind=SIMULATION_WORLD,
        policy=SpatialMappingPolicy(
            resolution_mm=resolution_mm,
            range_max_mm=200,
            max_cells=max_cells,
            max_qualitative_evidence=4,
        ),
    )


class MetricSpatialMappingTests(unittest.TestCase):
    def test_correlated_rays_update_each_cell_once_with_endpoint_dominance(
        self,
    ):
        grid = simulator_grid(resolution_mm=100)

        update = grid.ingest(metric_snapshot(
            forward_mm=10,
            left_mm=200,
            right_mm=200,
        ))
        snapshot = grid.snapshot()
        origin = {
            (cell.grid_x, cell.grid_y): cell
            for cell in snapshot.cells
        }[(0, 0)]

        self.assertEqual(update.cells_touched, len(snapshot.cells))
        self.assertEqual(origin.classification, CELL_OCCUPIED)
        self.assertEqual(origin.occupancy_milli, 650)
        self.assertEqual(origin.evidence_count, 1)
        self.assertEqual(origin.occupied_evidence_count, 1)
        self.assertEqual(origin.free_evidence_count, 0)
        self.assertEqual(set(origin.provenance), {
            "SIMULATION_CONFIGURATION_SPACE:FORWARD",
            "SIMULATION_CONFIGURATION_SPACE:LEFT",
            "SIMULATION_CONFIGURATION_SPACE:RIGHT",
        })
        self.assertEqual(len(snapshot.object_hypotheses), 1)
        self.assertEqual(
            snapshot.object_hypotheses[
                0
            ].trusted_simulator_object_id,
            "box-1",
        )

    def test_three_metric_rays_clear_paths_and_occupy_short_endpoints(self):
        grid = simulator_grid()
        update = grid.ingest(metric_snapshot(
            forward_mm=100,
            left_mm=200,
            right_mm=150,
        ))
        snapshot = grid.snapshot()
        by_coordinate = {
            (cell.grid_x, cell.grid_y): cell
            for cell in snapshot.cells
        }

        self.assertTrue(update.applied)
        self.assertEqual(len(snapshot.sensor_rays), 3)
        self.assertEqual(
            {
                ray.direction: ray.endpoint_occupied
                for ray in snapshot.sensor_rays
            },
            {
                "FORWARD": True,
                "LEFT": False,
                "RIGHT": True,
            },
        )
        self.assertEqual(
            by_coordinate[(2, 0)].classification,
            CELL_OCCUPIED,
        )
        self.assertEqual(
            by_coordinate[(1, 0)].classification,
            CELL_FREE,
        )
        self.assertEqual(snapshot.map_quality, MAP_SIMULATION_METRIC)
        self.assertEqual(snapshot.frame_kind, SIMULATION_WORLD)
        self.assertIsNotNone(snapshot.bounds)
        self.assertTrue(snapshot.object_hypotheses)
        forward_object = next(
            item
            for item in snapshot.object_hypotheses
            if item.trusted_simulator_object_id == "box-1"
        )
        self.assertEqual(
            forward_object.semantic_label,
            SEMANTIC_UNKNOWN,
        )
        self.assertIn(
            "SIMULATION_CONFIGURATION_SPACE:FORWARD",
            forward_object.provenance,
        )

    def test_connected_cells_form_stable_opaque_object_hypothesis(self):
        grid = simulator_grid(
            resolution_mm=100,
        )
        first = metric_snapshot(
            state_version=1,
            pose=PoseEstimate(0, 0, 0),
            forward_mm=100,
            left_mm=None,
            right_mm=None,
        )
        second = metric_snapshot(
            state_version=2,
            pose=PoseEstimate(0, 100, 0),
            forward_mm=100,
            left_mm=None,
            right_mm=None,
        )

        grid.ingest(first)
        first_id = grid.snapshot().object_hypotheses[0].hypothesis_id
        grid.ingest(second)
        hypothesis = grid.snapshot().object_hypotheses[0]

        self.assertEqual(hypothesis.hypothesis_id, first_id)
        self.assertTrue(hypothesis.hypothesis_id.startswith("object-"))
        self.assertNotIn("box-1", hypothesis.hypothesis_id)
        self.assertEqual(hypothesis.cell_count, 2)
        self.assertEqual(hypothesis.evidence_count, 2)
        self.assertEqual(
            hypothesis.trusted_simulator_object_id,
            "box-1",
        )
        self.assertEqual(hypothesis.min_x_mm, 100)
        self.assertEqual(hypothesis.max_x_mm, 200)
        self.assertEqual(hypothesis.min_y_mm, 0)
        self.assertEqual(hypothesis.max_y_mm, 200)

    def test_hypothesis_id_is_stable_when_component_grows_left_or_down(
        self,
    ):
        cases = (
            (
                "left",
                PoseEstimate(200, 0, 90_000),
                PoseEstimate(100, 0, 90_000),
            ),
            (
                "down",
                PoseEstimate(100, 200, 0),
                PoseEstimate(100, 100, 0),
            ),
        )
        for direction, first_pose, second_pose in cases:
            for object_id in ("box-1", None):
                with self.subTest(
                    direction=direction,
                    trusted=object_id is not None,
                ):
                    grid = simulator_grid(resolution_mm=100)
                    grid.ingest(metric_snapshot(
                        state_version=1,
                        pose=first_pose,
                        forward_mm=100,
                        left_mm=None,
                        right_mm=None,
                        object_id=object_id,
                    ))
                    first = grid.snapshot().object_hypotheses[0]

                    grid.ingest(metric_snapshot(
                        state_version=2,
                        pose=second_pose,
                        forward_mm=100,
                        left_mm=None,
                        right_mm=None,
                        object_id=object_id,
                    ))
                    hypotheses = grid.snapshot().object_hypotheses

                    self.assertEqual(len(hypotheses), 1)
                    self.assertEqual(
                        hypotheses[0].hypothesis_id,
                        first.hypothesis_id,
                    )
                    self.assertEqual(hypotheses[0].cell_count, 2)
                    self.assertTrue(
                        hypotheses[0].hypothesis_id.startswith(
                            "object-"
                        )
                    )
                    if object_id is not None:
                        self.assertNotIn(
                            object_id,
                            hypotheses[0].hypothesis_id,
                        )

    def test_new_world_generation_resets_geometric_and_qualitative_evidence(
        self,
    ):
        grid = simulator_grid()
        grid.ingest(metric_snapshot(
            state_version=1,
            world_model_version=1,
            pose=PoseEstimate(0, 0, 0),
            forward_mm=100,
            left_mm=None,
            right_mm=None,
            object_id="removed-box",
        ))
        grid.ingest(physical_snapshot(
            state_version=2,
            captured_at_ms=200,
        ))
        before = grid.snapshot()
        self.assertTrue(before.cells)
        self.assertTrue(before.qualitative_evidence)
        self.assertEqual(before.map_version, 2)

        update = grid.ingest(metric_snapshot(
            state_version=3,
            world_model_version=2,
            pose=PoseEstimate(0, 200, 0),
            forward_mm=100,
            left_mm=None,
            right_mm=None,
            object_id="new-box",
        ))
        after = grid.snapshot()

        self.assertTrue(update.applied)
        self.assertEqual(update.map_version, 3)
        self.assertEqual(after.map_version, 3)
        self.assertEqual(after.based_on_world_model_version, 2)
        self.assertEqual(
            after.evidence_sources,
            ("simulation_metric",),
        )
        self.assertEqual(after.qualitative_evidence, ())
        self.assertTrue(after.cells)
        self.assertTrue(all(
            cell.last_world_model_version == 2
            for cell in after.cells
        ))
        self.assertEqual(
            {
                item.trusted_simulator_object_id
                for item in after.object_hypotheses
            },
            {"new-box"},
        )

    def test_reused_sensor_timestamp_updates_pose_without_refusion(
        self,
    ):
        grid = simulator_grid()
        grid.ingest(metric_snapshot(
            state_version=1,
            captured_at_ms=100,
            pose=PoseEstimate(0, 0, 0),
            forward_mm=100,
            left_mm=None,
            right_mm=None,
            object_id="old-box",
        ))
        before = grid.snapshot()
        duplicate = metric_snapshot(
            state_version=2,
            captured_at_ms=200,
            pose=PoseEstimate(0, 100, 0),
            forward_mm=100,
            left_mm=None,
            right_mm=None,
            object_id="old-box",
        )
        duplicate = replace(
            duplicate,
            clearance=replace(
                duplicate.clearance,
                observed_at_ms=100,
            ),
        )

        duplicate_update = grid.ingest(duplicate)
        after_duplicate = grid.snapshot()

        self.assertTrue(duplicate_update.applied)
        self.assertEqual(duplicate_update.cells_touched, 0)
        self.assertEqual(after_duplicate.map_version, 2)
        self.assertEqual(after_duplicate.cells, before.cells)
        self.assertEqual(
            after_duplicate.sensor_rays,
            before.sensor_rays,
        )
        self.assertEqual(
            after_duplicate.latest_robot_pose.x_mm,
            0,
        )
        self.assertEqual(
            after_duplicate.latest_robot_pose.y_mm,
            100,
        )
        self.assertEqual(
            after_duplicate.latest_robot_pose.state_version,
            2,
        )
        self.assertEqual(
            after_duplicate.based_on_state_version,
            2,
        )

        new_generation = metric_snapshot(
            state_version=3,
            world_model_version=2,
            captured_at_ms=300,
            pose=PoseEstimate(0, 200, 0),
            forward_mm=100,
            left_mm=None,
            right_mm=None,
            object_id="new-box",
        )
        new_generation = replace(
            new_generation,
            clearance=replace(
                new_generation.clearance,
                observed_at_ms=100,
            ),
        )

        generation_update = grid.ingest(new_generation)
        after_generation = grid.snapshot()

        self.assertTrue(generation_update.applied)
        self.assertGreater(generation_update.cells_touched, 0)
        self.assertEqual(after_generation.map_version, 3)
        self.assertTrue(all(
            cell.last_world_model_version == 2
            for cell in after_generation.cells
        ))
        self.assertEqual(
            after_generation.sensor_rays[0].origin_y_mm,
            200,
        )
        self.assertEqual(
            {
                item.trusted_simulator_object_id
                for item in after_generation.object_hypotheses
            },
            {"new-box"},
        )

    def test_duplicate_and_stale_state_versions_are_non_mutating(self):
        grid = simulator_grid()
        accepted = grid.ingest(metric_snapshot(state_version=2))
        before = grid.snapshot()
        duplicate = grid.ingest(metric_snapshot(state_version=2))
        stale = grid.ingest(metric_snapshot(state_version=1))
        after = grid.snapshot()

        self.assertTrue(accepted.applied)
        self.assertFalse(duplicate.applied)
        self.assertFalse(stale.applied)
        self.assertEqual(
            duplicate.reason_code,
            STALE_STATE_VERSION,
        )
        self.assertEqual(stale.reason_code, STALE_STATE_VERSION)
        self.assertEqual(duplicate.cells_touched, 0)
        self.assertEqual(after, before)

    def test_conflicting_rays_pass_through_unknown_before_flipping(self):
        grid = simulator_grid()
        grid.ingest(metric_snapshot(
            state_version=1,
            forward_mm=100,
            left_mm=None,
            right_mm=None,
            object_id="old-box",
        ))

        grid.ingest(metric_snapshot(
            state_version=2,
            forward_mm=200,
            left_mm=None,
            right_mm=None,
            object_id=None,
        ))
        after_one_clear = {
            (cell.grid_x, cell.grid_y): cell
            for cell in grid.snapshot().cells
        }[(2, 0)]

        self.assertEqual(
            after_one_clear.classification,
            CELL_UNKNOWN,
        )
        self.assertEqual(after_one_clear.occupancy_milli, 400)
        self.assertEqual(grid.snapshot().object_hypotheses, ())

        for version in (3, 4, 5):
            grid.ingest(metric_snapshot(
                state_version=version,
                forward_mm=200,
                left_mm=None,
                right_mm=None,
                object_id=None,
            ))
        cleared = {
            (cell.grid_x, cell.grid_y): cell
            for cell in grid.snapshot().cells
        }[(2, 0)]
        self.assertEqual(cleared.classification, CELL_FREE)
        self.assertLessEqual(cleared.occupancy_milli, -250)

        for version in (6, 7):
            grid.ingest(metric_snapshot(
                state_version=version,
                forward_mm=100,
                left_mm=None,
                right_mm=None,
                object_id="new-box",
            ))
        hypotheses = grid.snapshot().object_hypotheses
        self.assertEqual(len(hypotheses), 1)
        self.assertEqual(
            hypotheses[0].trusted_simulator_object_id,
            "new-box",
        )

    def test_capacity_is_hard_bounded_with_deterministic_eviction(self):
        grid = simulator_grid(max_cells=5)
        update = grid.ingest(metric_snapshot(
            forward_mm=200,
            left_mm=200,
            right_mm=200,
            object_id=None,
        ))
        snapshot = grid.snapshot()

        self.assertLessEqual(len(snapshot.cells), 5)
        self.assertGreater(update.cells_evicted, 0)
        self.assertEqual(
            snapshot.cells_evicted,
            update.cells_evicted,
        )
        json.dumps(snapshot.to_dict(), allow_nan=False)

    def test_identity_mismatch_fails_closed(self):
        grid = simulator_grid()
        foreign = metric_snapshot()
        foreign = NavigationSnapshot(
            robot_id="other-robot",
            controller_instance_id=foreign.controller_instance_id,
            goal_id=foreign.goal_id,
            goal_epoch=foreign.goal_epoch,
            plan_revision=foreign.plan_revision,
            state_version=foreign.state_version,
            world_model_version=foreign.world_model_version,
            captured_at_host_ms=foreign.captured_at_host_ms,
            state_observed_at_ms=foreign.state_observed_at_ms,
            pose=foreign.pose,
            left_encoder_mdeg=foreign.left_encoder_mdeg,
            right_encoder_mdeg=foreign.right_encoder_mdeg,
            motors_running=foreign.motors_running,
            touch_pressed=foreign.touch_pressed,
            active_faults=foreign.active_faults,
            clearance=foreign.clearance,
        )

        with self.assertRaises(NavigationContractError) as caught:
            grid.ingest(foreign)

        self.assertEqual(caught.exception.code, "spatial_identity_mismatch")


class QualitativeSpatialMappingTests(unittest.TestCase):
    def test_physical_ir_never_updates_metric_cells_or_claims_range(self):
        grid = BoundedOccupancyGrid(
            map_id="map-physical",
            robot_id="robot-1",
            controller_instance_id="controller-1",
            frame_id="local-odometry",
            frame_kind=LOCAL_ODOMETRY,
            policy=SpatialMappingPolicy(
                max_qualitative_evidence=2,
            ),
        )
        update = grid.ingest(physical_snapshot())
        snapshot = grid.snapshot()
        evidence = snapshot.qualitative_evidence[0]
        ray = snapshot.sensor_rays[0]

        self.assertTrue(update.applied)
        self.assertEqual(update.cells_touched, 0)
        self.assertEqual(snapshot.cells, ())
        self.assertEqual(len(snapshot.object_hypotheses), 1)
        self.assertEqual(snapshot.map_quality, MAP_PROVISIONAL_IR)
        self.assertTrue(evidence.provisional)
        self.assertLessEqual(evidence.confidence_milli, 400)
        self.assertEqual(evidence.frame_id, "ROBOT_BASE")
        self.assertTrue(ray.provisional)
        self.assertIsNone(ray.measured_range_mm)
        self.assertIsNone(ray.end_x_mm)
        for value in (evidence.to_dict(), ray.to_dict()):
            self.assertFalse(
                any(key.endswith("_mm") for key in value),
                value,
            )
        hypothesis = snapshot.object_hypotheses[0]
        self.assertIsInstance(
            hypothesis,
            ProvisionalObjectHypothesis,
        )
        self.assertEqual(
            hypothesis.geometry_kind,
            QUALITATIVE_FORWARD_ENVELOPE,
        )
        self.assertEqual(hypothesis.anchor_x_mm, 10)
        self.assertEqual(hypothesis.anchor_y_mm, 20)
        self.assertEqual(hypothesis.anchor_heading_mdeg, 30_000)
        self.assertEqual(hypothesis.to_dict()["bounds_mm"], None)
        self.assertIsNone(snapshot.bounds)
        self.assertEqual(
            set(hypothesis.provenance),
            {LOCAL_ODOMETRY_POSE, PHYSICAL_IR_REFLECTION},
        )
        self.assertLessEqual(hypothesis.confidence_milli, 400)
        self.assertNotIn(
            "measured_range_mm",
            hypothesis.to_dict(),
        )
        self.assertNotIn("centroid_mm", hypothesis.to_dict())

    def test_clear_after_heading_change_keeps_same_encounter_hypothesis(self):
        grid = BoundedOccupancyGrid(
            map_id="map-physical",
            robot_id="robot-1",
            controller_instance_id="controller-1",
            frame_id="local-odometry",
            frame_kind=LOCAL_ODOMETRY,
        )
        grid.ingest(physical_snapshot(
            state_version=1,
            pose=PoseEstimate(10, 20, 0),
            near=True,
        ))
        first = grid.snapshot().object_hypotheses[0]

        grid.ingest(physical_snapshot(
            state_version=2,
            pose=PoseEstimate(10, 20, 60_000),
            near=False,
            raw_ir_proximity=55,
        ))
        after_turn = grid.snapshot()

        self.assertEqual(len(after_turn.object_hypotheses), 1)
        self.assertEqual(
            after_turn.object_hypotheses[0].hypothesis_id,
            first.hypothesis_id,
        )
        self.assertEqual(after_turn.object_hypotheses[0], first)
        self.assertEqual(
            after_turn.latest_robot_pose.heading_mdeg,
            60_000,
        )
        self.assertEqual(
            after_turn.qualitative_evidence[-1].relation,
            "NO_NEAR_REFLECTION",
        )
        self.assertEqual(after_turn.cells, ())
        self.assertIsNone(after_turn.bounds)

    def test_new_near_episode_gets_new_handle_without_erasing_old_one(self):
        grid = BoundedOccupancyGrid(
            map_id="map-physical",
            robot_id="robot-1",
            controller_instance_id="controller-1",
            frame_id="local-odometry",
            frame_kind=LOCAL_ODOMETRY,
            policy=SpatialMappingPolicy(
                max_provisional_object_hypotheses=2,
            ),
        )
        grid.ingest(physical_snapshot(state_version=1, near=True))
        first_id = grid.snapshot().object_hypotheses[0].hypothesis_id
        grid.ingest(physical_snapshot(
            state_version=2,
            near=False,
            raw_ir_proximity=50,
        ))
        grid.ingest(physical_snapshot(
            state_version=3,
            near=True,
            pose=PoseEstimate(20, 30, 90_000),
        ))
        hypotheses = grid.snapshot().object_hypotheses

        self.assertEqual(len(hypotheses), 2)
        self.assertIn(
            first_id,
            {item.hypothesis_id for item in hypotheses},
        )
        self.assertEqual(len({
            item.hypothesis_id for item in hypotheses
        }), 2)

    def test_qualitative_history_is_bounded(self):
        grid = BoundedOccupancyGrid(
            map_id="map-physical",
            robot_id="robot-1",
            controller_instance_id="controller-1",
            frame_id="local-odometry",
            frame_kind=LOCAL_ODOMETRY,
            policy=SpatialMappingPolicy(
                max_qualitative_evidence=2,
            ),
        )
        for version in range(1, 5):
            grid.ingest(physical_snapshot(state_version=version))

        evidence = grid.snapshot().qualitative_evidence

        self.assertEqual(len(evidence), 2)
        self.assertEqual(
            tuple(item.state_version for item in evidence),
            (3, 4),
        )


class ConcurrentSpatialMappingTests(unittest.TestCase):
    def test_concurrent_ingest_and_snapshot_remain_bounded_and_immutable(self):
        grid = simulator_grid(max_cells=32)
        values = tuple(
            metric_snapshot(
                state_version=version,
                pose=PoseEstimate(version, version, 0),
            )
            for version in range(1, 51)
        )

        with ThreadPoolExecutor(max_workers=8) as pool:
            futures = [
                pool.submit(grid.ingest, value)
                for value in reversed(values)
            ]
            snapshots = [
                pool.submit(grid.snapshot)
                for _index in range(20)
            ]
            for future in futures:
                future.result()
            observed = [future.result() for future in snapshots]

        final = grid.snapshot()
        self.assertEqual(final.based_on_state_version, 50)
        self.assertLessEqual(len(final.cells), 32)
        self.assertTrue(
            all(len(item.cells) <= 32 for item in observed)
        )
        frozen = final
        ignored = grid.ingest(values[-1])
        self.assertFalse(ignored.applied)
        self.assertEqual(frozen, grid.snapshot())


if __name__ == "__main__":
    unittest.main()
