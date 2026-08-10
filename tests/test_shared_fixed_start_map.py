from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from unittest import TestCase

from robot_agent.shared_fixed_start_map import (
    FIXED_START_REBIND_REQUIRED,
    FIXED_START_SOURCES_PENDING,
    FixedStartSharedMapProvider,
)
from robot_agent.shared_spatial_map import SharedSpatialMapError


LOCAL_IDENTITY = ("blast-01", "blast-01.hub")
PEER_IDENTITY = ("ev3rstorm-01", "ev3rstorm-01.ev3-main")


class Provider:
    def __init__(self, value=None, error=None):
        self.value = value
        self.error = error
        self.snapshot_calls = 0
        self.write_calls = 0
        self.close_calls = 0

    def snapshot(self):
        self.snapshot_calls += 1
        if self.error is not None:
            raise self.error
        return deepcopy(self.value)

    def offer(self, *args, **kwargs):
        self.write_calls += 1
        raise AssertionError("fixed-start provider attempted a source write")

    def close(self):
        self.close_calls += 1
        raise AssertionError("fixed-start provider attempted to close a source")


def pose(frame_id, x_mm=0, y_mm=0, heading_mdeg=0):
    return {
        "x_mm": x_mm,
        "y_mm": y_mm,
        "heading_mdeg": heading_mdeg,
        "frame_id": frame_id,
        "state_version": 1,
        "source_id": "navigation-pose",
        "provenance": "LOCAL_ODOMETRY",
        "observed_at_unix_ms": 1_000,
        "age_ms": 0,
    }


def local_map(
    identity,
    frame_id,
    generation_id,
    *,
    x_mm=0,
    y_mm=0,
    heading_mdeg=0,
    status="pose_only",
):
    robot_id, controller_id = identity
    current_pose = pose(frame_id, x_mm, y_mm, heading_mdeg)
    return {
        "schema": "robot-spatial-map/v1",
        "read_only": True,
        "status": status,
        "map_id": "{}-map".format(robot_id),
        "robot_id": robot_id,
        "controller_instance_id": controller_id,
        "frame_id": frame_id,
        "local_generation_id": generation_id,
        "frame_kind": "LOCAL_ODOMETRY",
        "map_version": 1,
        "robot_pose": current_pose,
        "pose_history": [current_pose],
        "pose_history_evicted": 0,
        "collision_geometry": None,
        "captured_at_unix_ms": 1_000,
        "age_ms": 0,
        "cells": [],
        "sensor_rays": [],
        "object_hypotheses": [],
    }


def provider(local, peer, **changes):
    values = {
        "local_provider": local,
        "peer_provider": peer,
        "local_robot_id": LOCAL_IDENTITY[0],
        "local_controller_id": LOCAL_IDENTITY[1],
        "peer_robot_id": PEER_IDENTITY[0],
        "peer_controller_id": PEER_IDENTITY[1],
        "peer_tx_mm": 1_000,
        "peer_ty_mm": 500,
        "peer_yaw_mdeg": 90_000,
        "world_frame_id": "shared-world",
        "world_generation_id": "world-generation-1",
        "position_uncertainty_mm": 20,
        "yaw_uncertainty_mdeg": 2_000,
    }
    values.update(changes)
    return FixedStartSharedMapProvider(**values)


class FixedStartSharedMapProviderTests(TestCase):
    def test_pending_is_valid_v2_and_first_complete_pair_binds(self):
        local = Provider(local_map(
            LOCAL_IDENTITY,
            "blast-frame-1",
            "blast-generation-1",
        ))
        peer = Provider(error=RuntimeError("peer offline"))
        shared = provider(local, peer)

        pending = shared.snapshot()

        self.assertEqual(pending["schema"], "robot-spatial-map/v2")
        self.assertTrue(pending["read_only"])
        self.assertEqual(pending["status"], "unavailable")
        self.assertEqual(
            pending["reason_code"],
            FIXED_START_SOURCES_PENDING,
        )
        self.assertEqual(pending["frame_kind"], "SHARED_FIXED_START")
        self.assertEqual(pending["frame_id"], "shared-world")
        self.assertEqual(
            pending["world_generation_id"],
            "world-generation-1",
        )
        self.assertEqual(pending["robots"], [])
        self.assertEqual((local.snapshot_calls, peer.snapshot_calls), (1, 1))

        pending["robots"].append({"not": "provider state"})
        peer.error = None
        peer.value = local_map(
            PEER_IDENTITY,
            "ev3-frame-1",
            "ev3-generation-1",
            x_mm=100,
        )
        bound = shared.snapshot()

        self.assertEqual(bound["status"], "available")
        self.assertEqual(bound["reason_code"], "all_sources_available")
        self.assertEqual(len(bound["robots"]), 2)
        self.assertEqual((local.snapshot_calls, peer.snapshot_calls), (2, 2))
        self.assertEqual((local.write_calls, peer.write_calls), (0, 0))
        self.assertEqual((local.close_calls, peer.close_calls), (0, 0))

        robots = {robot["robot_id"]: robot for robot in bound["robots"]}
        blast_transform = robots[LOCAL_IDENTITY[0]]["frame_transform"]
        self.assertEqual(
            (
                blast_transform["tx_mm"],
                blast_transform["ty_mm"],
                blast_transform["yaw_mdeg"],
            ),
            (0, 0, 0),
        )
        self.assertEqual(
            blast_transform["source_generation_id"],
            "blast-generation-1",
        )
        ev3 = robots[PEER_IDENTITY[0]]
        self.assertEqual(
            (
                ev3["frame_transform"]["tx_mm"],
                ev3["frame_transform"]["ty_mm"],
                ev3["frame_transform"]["yaw_mdeg"],
            ),
            (1_000, 500, 90_000),
        )
        self.assertEqual(
            (
                ev3["robot_pose"]["x_mm"],
                ev3["robot_pose"]["y_mm"],
                ev3["robot_pose"]["heading_mdeg"],
            ),
            (1_000, 600, 90_000),
        )

    def test_incomplete_or_wrong_expected_identity_cannot_consume_binding(self):
        local = Provider(local_map(
            ("unexpected-robot", LOCAL_IDENTITY[1]),
            "wrong-frame",
            "wrong-generation",
        ))
        peer = Provider(local_map(
            PEER_IDENTITY,
            "ev3-frame-1",
            "ev3-generation-1",
        ))
        shared = provider(local, peer)

        wrong = shared.snapshot()
        self.assertEqual(wrong["reason_code"], FIXED_START_SOURCES_PENDING)

        incomplete = local_map(
            LOCAL_IDENTITY,
            "blast-frame-1",
            "blast-generation-1",
        )
        incomplete["pose_history"] = []
        local.value = incomplete
        not_complete = shared.snapshot()
        self.assertEqual(
            not_complete["reason_code"],
            FIXED_START_SOURCES_PENDING,
        )

        local.value = local_map(
            LOCAL_IDENTITY,
            "blast-frame-2",
            "blast-generation-2",
        )
        bound = shared.snapshot()
        self.assertEqual(bound["status"], "available")
        blast = next(
            robot for robot in bound["robots"]
            if robot["robot_id"] == LOCAL_IDENTITY[0]
        )
        self.assertEqual(blast["local_frame_id"], "blast-frame-2")
        self.assertEqual(
            blast["local_generation_id"],
            "blast-generation-2",
        )

    def test_generation_change_latches_rebind_and_never_auto_rebinds(self):
        local = Provider(local_map(
            LOCAL_IDENTITY,
            "blast-frame-1",
            "blast-generation-1",
        ))
        peer = Provider(local_map(
            PEER_IDENTITY,
            "ev3-frame-1",
            "ev3-generation-1",
        ))
        shared = provider(local, peer)
        self.assertEqual(shared.snapshot()["status"], "available")

        peer.value = local_map(
            PEER_IDENTITY,
            "ev3-frame-2",
            "ev3-generation-2",
        )
        changed = shared.snapshot()

        self.assertEqual(changed["status"], "unavailable")
        self.assertEqual(
            changed["reason_code"],
            FIXED_START_REBIND_REQUIRED,
        )
        robots = {robot["robot_id"]: robot for robot in changed["robots"]}
        self.assertEqual(robots[LOCAL_IDENTITY[0]]["status"], "available")
        self.assertEqual(robots[PEER_IDENTITY[0]]["status"], "unavailable")
        self.assertEqual(
            robots[PEER_IDENTITY[0]]["reason_code"],
            "source_identity_mismatch",
        )
        calls_at_rebind = (local.snapshot_calls, peer.snapshot_calls)

        # Even a return to the old generation or a later valid generation
        # cannot silently create a new physical-world calibration.
        peer.value = local_map(
            PEER_IDENTITY,
            "ev3-frame-1",
            "ev3-generation-1",
        )
        changed["robots"].clear()
        still_blocked = shared.snapshot()
        self.assertEqual(
            still_blocked["reason_code"],
            FIXED_START_REBIND_REQUIRED,
        )
        self.assertEqual(len(still_blocked["robots"]), 2)
        self.assertEqual(
            (local.snapshot_calls, peer.snapshot_calls),
            calls_at_rebind,
        )

    def test_same_generation_transient_failure_recovers_without_rebind(self):
        local = Provider(local_map(
            LOCAL_IDENTITY,
            "blast-frame-1",
            "blast-generation-1",
        ))
        peer = Provider(local_map(
            PEER_IDENTITY,
            "ev3-frame-1",
            "ev3-generation-1",
        ))
        shared = provider(local, peer)
        self.assertEqual(shared.snapshot()["status"], "available")

        peer.error = RuntimeError("temporary peer timeout")
        degraded = shared.snapshot()
        self.assertEqual(degraded["status"], "degraded")
        self.assertEqual(degraded["reason_code"], "some_sources_unavailable")
        self.assertEqual(
            next(
                robot for robot in degraded["robots"]
                if robot["robot_id"] == PEER_IDENTITY[0]
            )["reason_code"],
            "source_snapshot_failed",
        )

        peer.error = None
        recovered = shared.snapshot()
        self.assertEqual(recovered["status"], "available")
        self.assertEqual((local.snapshot_calls, peer.snapshot_calls), (3, 3))

    def test_first_binding_is_thread_safe_and_world_generation_is_stable(self):
        local = Provider(local_map(
            LOCAL_IDENTITY,
            "blast-frame-1",
            "blast-generation-1",
        ))
        peer = Provider(local_map(
            PEER_IDENTITY,
            "ev3-frame-1",
            "ev3-generation-1",
        ))
        shared = provider(
            local,
            peer,
            world_generation_id=None,
        )

        with ThreadPoolExecutor(max_workers=8) as pool:
            snapshots = list(pool.map(lambda _: shared.snapshot(), range(16)))

        generations = {
            snapshot["world_generation_id"] for snapshot in snapshots
        }
        self.assertEqual(len(generations), 1)
        self.assertTrue(next(iter(generations)).startswith("fixed-start-"))
        self.assertTrue(all(
            snapshot["status"] == "available" for snapshot in snapshots
        ))
        self.assertEqual((local.snapshot_calls, peer.snapshot_calls), (16, 16))
        for snapshot in snapshots:
            self.assertEqual(
                {
                    robot["local_generation_id"]
                    for robot in snapshot["robots"]
                },
                {"blast-generation-1", "ev3-generation-1"},
            )

    def test_constructor_rejects_ambiguous_or_invalid_configuration(self):
        source = Provider(local_map(
            LOCAL_IDENTITY,
            "blast-frame-1",
            "blast-generation-1",
        ))
        with self.assertRaisesRegex(
            SharedSpatialMapError,
            "providers are invalid",
        ):
            provider(source, source)

        peer = Provider(local_map(
            PEER_IDENTITY,
            "ev3-frame-1",
            "ev3-generation-1",
        ))
        with self.assertRaisesRegex(
            SharedSpatialMapError,
            "configuration is invalid",
        ):
            provider(source, peer, peer_yaw_mdeg=180_000)


if __name__ == "__main__":
    import unittest

    unittest.main()
