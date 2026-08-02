import unittest

from robot_agent.legacy_shadow_runtime_context import (
    LegacyShadowRuntimeContextError,
    build_legacy_shadow_basis,
    build_legacy_shadow_goal,
    calibration_fingerprint,
    stable_shadow_id,
)
from tests.test_physical_navigation_core import observation


def navigation(*, map_version=0):
    return {
        "robot_id": "ev3rstorm-01",
        "controller_instance_id": "ev3-main",
        "map_generation_id": "mapgen-a",
        "map_version": map_version,
        "frame_id": "frame-a",
        "localization_valid": True,
        "pose": {"x_mm": 0, "y_mm": 0, "heading_mdeg": 0},
        "drive_motor_roles": {
            "left": "left_drive",
            "right": "right_drive",
        },
        "navigation_hazard_hypotheses": [],
    }


class LegacyShadowRuntimeContextTests(unittest.TestCase):
    def test_goal_and_ids_are_stable_and_bounded(self):
        first = build_legacy_shadow_goal(
            episode_id="episode-a",
            objective="Navigate around the box",
            locale="en",
            activated_at_ms=1_000,
        )
        second = build_legacy_shadow_goal(
            episode_id="episode-a",
            objective="Navigate around the box",
            locale="en",
            activated_at_ms=1_000,
        )

        self.assertEqual(first, second)
        self.assertEqual(first.goal_epoch, 1)
        self.assertEqual(
            stable_shadow_id("shadow-plan", "episode-a", 2, 1),
            stable_shadow_id("shadow-plan", "episode-a", 2, 1),
        )
        self.assertNotEqual(
            stable_shadow_id("shadow-plan", "episode-a", 2, 1),
            stable_shadow_id("shadow-plan", "episode-a", 2, 2),
        )

    def test_basis_preserves_controller_and_zero_based_map_progress(self):
        fingerprint = calibration_fingerprint({"profile": "ev3-a"})
        first = build_legacy_shadow_basis(
            robot_id="ev3rstorm-01",
            controller_id="ev3-main",
            controller_instance_id="ev3-boot-a",
            goal_epoch=1,
            observation=observation(1),
            navigation=navigation(map_version=0),
            calibration_fingerprint_value=fingerprint,
        )
        second_navigation = navigation(map_version=1)
        second_navigation["pose"] = {
            "x_mm": 25,
            "y_mm": 0,
            "heading_mdeg": 0,
        }
        second = build_legacy_shadow_basis(
            robot_id="ev3rstorm-01",
            controller_id="ev3-main",
            controller_instance_id="ev3-boot-a",
            goal_epoch=1,
            observation=observation(
                2,
                left_position=50,
                right_position=50,
            ),
            navigation=second_navigation,
            calibration_fingerprint_value=fingerprint,
        )

        self.assertEqual(first.world_model_version, 1)
        self.assertEqual(second.world_model_version, 2)
        self.assertNotEqual(
            first.navigation_basis_id,
            second.navigation_basis_id,
        )
        second.assert_successor_of(first)

    def test_calibration_key_order_does_not_change_fingerprint(self):
        self.assertEqual(
            calibration_fingerprint({"b": 2, "a": 1}),
            calibration_fingerprint({"a": 1, "b": 2}),
        )

    def test_invalid_context_has_typed_failure(self):
        with self.assertRaises(LegacyShadowRuntimeContextError) as caught:
            build_legacy_shadow_basis(
                robot_id="ev3rstorm-01",
                controller_id="ev3-main",
                controller_instance_id="ev3-boot-a",
                goal_epoch=1,
                observation=observation(1),
                navigation={"map_version": 0},
                calibration_fingerprint_value="fingerprint-a",
            )

        self.assertEqual(caught.exception.code, "invalid_shadow_basis")


if __name__ == "__main__":
    unittest.main()
