import threading
import unittest

from robot_agent.navigation_state import (
    ClearanceEvidence,
    NavigationSnapshot,
    PoseEstimate,
)
from robot_agent.spatial_map_contract import SIMULATION_WORLD
from robot_agent.spatial_map_runtime import SpatialMapRuntime
from robot_agent.spatial_mapping import (
    BoundedOccupancyGrid,
    SpatialMappingPolicy,
)


def navigation_snapshot(state_version):
    observed_at_ms = state_version * 100
    return NavigationSnapshot(
        robot_id="robot-1",
        controller_instance_id="controller-1",
        goal_id="map-runtime",
        goal_epoch=1,
        plan_revision=1,
        state_version=state_version,
        world_model_version=1,
        captured_at_host_ms=observed_at_ms,
        state_observed_at_ms=observed_at_ms,
        pose=PoseEstimate(state_version * 10, 100, 0),
        left_encoder_mdeg=0,
        right_encoder_mdeg=0,
        motors_running=False,
        touch_pressed=False,
        active_faults=(),
        clearance=ClearanceEvidence(
            source="simulation_metric",
            observed_at_ms=observed_at_ms,
            near_obstacle_latched=True,
            forward_mm=100,
            left_mm=200,
            right_mm=200,
            forward_object_id="box-1",
        ),
    )


def grid(grid_class=BoundedOccupancyGrid):
    return grid_class(
        map_id="map-runtime",
        robot_id="robot-1",
        controller_instance_id="controller-1",
        frame_id="sim-world",
        frame_kind=SIMULATION_WORLD,
        policy=SpatialMappingPolicy(
            resolution_mm=50,
            range_max_mm=200,
            max_cells=128,
        ),
    )


class FixedClock:
    def __init__(self, value):
        self.value = value

    def __call__(self):
        return self.value


class BlockingGrid(BoundedOccupancyGrid):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.started = threading.Event()
        self.release = threading.Event()

    def ingest(self, snapshot):
        if snapshot.state_version == 1:
            self.started.set()
            if not self.release.wait(5):
                raise RuntimeError("blocking map fixture timed out")
        return super().ingest(snapshot)


class FailingGrid(BoundedOccupancyGrid):
    def ingest(self, _snapshot):
        raise RuntimeError("fixture mapper failure")


class TwoStageBlockingGrid(BoundedOccupancyGrid):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.first_started = threading.Event()
        self.release_first = threading.Event()
        self.third_started = threading.Event()
        self.release_third = threading.Event()

    def ingest(self, snapshot):
        if snapshot.state_version == 1:
            self.first_started.set()
            if not self.release_first.wait(5):
                raise RuntimeError("first map fixture timed out")
        elif snapshot.state_version == 3:
            self.third_started.set()
            if not self.release_third.wait(5):
                raise RuntimeError("third map fixture timed out")
        return super().ingest(snapshot)


class BlockingProjectionGrid(BoundedOccupancyGrid):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.block_snapshot = False
        self.snapshot_started = threading.Event()
        self.release_snapshot = threading.Event()

    def snapshot(self):
        if self.block_snapshot:
            self.snapshot_started.set()
            if not self.release_snapshot.wait(5):
                raise RuntimeError("map projection fixture timed out")
        return super().snapshot()


class SpatialMapRuntimeTests(unittest.TestCase):
    def make_runtime(self, target_grid, **kwargs):
        runtime = SpatialMapRuntime(target_grid, **kwargs)
        self.addCleanup(runtime.close)
        return runtime

    def test_worker_builds_dashboard_view_away_from_offer_path(self):
        monotonic = FixedClock(10_000)
        unix = FixedClock(2_000_000)
        runtime = self.make_runtime(
            grid(),
            monotonic_clock_ms=monotonic,
            unix_clock_ms=unix,
        )

        self.assertTrue(runtime.offer_nowait(navigation_snapshot(1)))
        self.assertTrue(runtime.flush())
        value = runtime.snapshot()

        self.assertEqual(value["schema"], "robot-spatial-map/v1")
        self.assertEqual(value["status"], "available")
        self.assertEqual(value["age_ms"], 0)
        self.assertTrue(value["cells"])
        self.assertEqual(len(value["sensor_rays"]), 3)
        self.assertTrue(all(
            ray["valid_until_unix_ms"] == 2_005_000
            for ray in value["sensor_rays"]
        ))
        self.assertEqual(value["runtime"]["applied_updates"], 1)
        self.assertFalse(value["runtime"]["incomplete"])
        value["cells"].clear()
        self.assertTrue(runtime.snapshot()["cells"])

    def test_full_relay_drops_oldest_without_waiting_for_mapper(self):
        target_grid = grid(BlockingGrid)
        runtime = self.make_runtime(
            target_grid,
            queue_capacity=2,
        )

        self.assertTrue(runtime.offer_nowait(navigation_snapshot(1)))
        self.assertTrue(target_grid.started.wait(2))
        self.assertTrue(runtime.offer_nowait(navigation_snapshot(2)))
        self.assertTrue(runtime.offer_nowait(navigation_snapshot(3)))
        self.assertTrue(runtime.offer_nowait(navigation_snapshot(4)))

        state_while_blocked = runtime.state()
        self.assertEqual(state_while_blocked.dropped_total, 1)
        self.assertEqual(state_while_blocked.queue_depth, 2)
        target_grid.release.set()
        self.assertTrue(runtime.flush())

        state = runtime.state()
        self.assertEqual(state.applied_updates, 3)
        self.assertEqual(state.dropped_total, 1)
        self.assertEqual(
            runtime.raw_snapshot().based_on_state_version,
            4,
        )
        dashboard = runtime.snapshot()
        self.assertEqual(dashboard["status"], "degraded")
        self.assertEqual(
            dashboard["reason_code"],
            "observation_gap",
        )

    def test_mapper_failure_is_contained_and_visible(self):
        runtime = self.make_runtime(grid(FailingGrid))

        self.assertTrue(runtime.offer_nowait(navigation_snapshot(1)))
        self.assertTrue(runtime.flush())

        state = runtime.state()
        self.assertEqual(state.failure_total, 1)
        self.assertEqual(state.applied_updates, 0)
        dashboard = runtime.snapshot()
        self.assertEqual(dashboard["status"], "degraded")
        self.assertEqual(
            dashboard["reason_code"],
            "mapping_failure",
        )
        self.assertEqual(dashboard["cells"], [])

    def test_abortive_close_does_not_settle_an_inflight_ingest(self):
        target_grid = grid(BlockingGrid)
        runtime = SpatialMapRuntime(
            target_grid,
            queue_capacity=2,
        )
        self.addCleanup(target_grid.release.set)
        self.addCleanup(runtime.close)

        self.assertTrue(runtime.offer_nowait(navigation_snapshot(1)))
        self.assertTrue(target_grid.started.wait(2))
        self.assertTrue(runtime.offer_nowait(navigation_snapshot(2)))

        self.assertFalse(runtime.close(drain=False, timeout_s=0.01))
        self.assertEqual(runtime.state().settled_sequence, 0)
        self.assertFalse(runtime.flush(timeout_s=0.01))

        target_grid.release.set()
        self.assertTrue(runtime.flush())
        self.assertEqual(runtime.state().settled_sequence, 2)

    def test_dropped_target_settles_before_later_inflight_work(self):
        target_grid = grid(TwoStageBlockingGrid)
        runtime = self.make_runtime(
            target_grid,
            queue_capacity=1,
        )
        self.addCleanup(target_grid.release_first.set)
        self.addCleanup(target_grid.release_third.set)

        self.assertTrue(runtime.offer_nowait(navigation_snapshot(1)))
        self.assertTrue(target_grid.first_started.wait(2))
        self.assertTrue(runtime.offer_nowait(navigation_snapshot(2)))
        self.assertTrue(runtime.offer_nowait(navigation_snapshot(3)))
        self.assertEqual(runtime.state().dropped_total, 1)

        target_grid.release_first.set()
        self.assertTrue(target_grid.third_started.wait(2))
        state = runtime.state()
        self.assertEqual(state.settled_sequence, 2)
        self.assertEqual(state.publication_sequence, 3)

        target_grid.release_third.set()
        self.assertTrue(runtime.flush())
        self.assertEqual(runtime.state().settled_sequence, 3)

    def test_dashboard_snapshot_uses_cache_while_projection_is_busy(self):
        target_grid = grid(BlockingProjectionGrid)
        runtime = self.make_runtime(target_grid)
        self.addCleanup(target_grid.release_snapshot.set)
        target_grid.block_snapshot = True

        self.assertTrue(runtime.offer_nowait(navigation_snapshot(1)))
        self.assertTrue(target_grid.snapshot_started.wait(2))
        values = []
        reader = threading.Thread(
            target=lambda: values.append(runtime.snapshot())
        )
        reader.start()
        reader.join(0.5)

        self.assertFalse(reader.is_alive())
        self.assertEqual(values[0]["status"], "unavailable")
        target_grid.release_snapshot.set()
        self.assertTrue(runtime.flush())
        self.assertEqual(runtime.snapshot()["status"], "available")

    def test_rejects_invalid_or_post_close_observations(self):
        runtime = SpatialMapRuntime(grid())

        self.assertFalse(runtime.offer_nowait(object()))
        rejected = runtime.snapshot()
        self.assertEqual(rejected["status"], "degraded")
        self.assertEqual(
            rejected["reason_code"],
            "observation_rejected",
        )
        self.assertTrue(runtime.close())
        self.assertFalse(runtime.offer_nowait(navigation_snapshot(1)))
        self.assertEqual(runtime.state().rejected_total, 2)


if __name__ == "__main__":
    unittest.main()
