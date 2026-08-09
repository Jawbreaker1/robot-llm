from dataclasses import FrozenInstanceError, replace
import unittest

from robot_agent.local_detour_collision_snapshot import (
    LocalDetourCollisionSnapshot,
)
from robot_agent.physical_footprint import RobotFootprint


def footprint():
    return RobotFootprint(
        front_extent_mm=110,
        rear_extent_mm=60,
        left_extent_mm=105,
        right_extent_mm=100,
        clearance_margin_mm=10,
        calibration_status="test",
        calibration_evidence="collision snapshot fixture",
    )


def snapshot():
    return LocalDetourCollisionSnapshot(
        frame_id="frame-a",
        map_generation_id="generation-a",
        map_version=7,
        target_hypothesis_id="target-a",
        target_centroid_x_mm=300,
        target_centroid_y_mm=0,
        target_envelope_radius_mm=70,
        target_support_points=((300, 0), (300, 50)),
        robot_footprint=footprint(),
        lateral_clearance_margin_mm=15,
    )


class LocalDetourCollisionSnapshotTests(unittest.TestCase):
    def test_valid_snapshot_is_frozen(self):
        value = snapshot()

        self.assertEqual(value.target_support_points, ((300, 0), (300, 50)))
        with self.assertRaises(FrozenInstanceError):
            value.map_version = 8

    def test_rejects_invalid_identity_geometry_and_map_state(self):
        cases = (
            {"frame_id": ""},
            {"map_generation_id": " generation-a"},
            {"target_hypothesis_id": "target\na"},
            {"map_version": -1},
            {"target_centroid_x_mm": True},
            {"target_envelope_radius_mm": 0},
            {"target_support_points": ()},
            {"target_support_points": ((300, 50), (300, 0))},
            {"target_support_points": ((300, 0), (300, 0))},
            {"target_support_points": ((300, 50),)},
            {"robot_footprint": object()},
            {"lateral_clearance_margin_mm": 501},
        )
        for changes in cases:
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    replace(snapshot(), **changes)

    def test_missing_footprint_remains_explicit(self):
        value = replace(snapshot(), robot_footprint=None)

        self.assertIsNone(value.robot_footprint)

    def test_snapshot_preserves_more_supports_than_one_route_can_consume(self):
        supports = tuple((300, offset) for offset in range(4_160))

        value = replace(snapshot(), target_support_points=supports)

        self.assertEqual(value.target_support_points, supports)
        with self.assertRaises(ValueError):
            replace(
                snapshot(),
                target_support_points=supports + ((300, 4_160),),
            )


if __name__ == "__main__":
    unittest.main()
