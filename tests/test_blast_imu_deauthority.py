import copy
import unittest

from robot_agent.blast_episode_adapter import (
    BlastEpisodeError,
    BlastEpisodeRuntimeAdapter,
    _side_search_encoder_correlated,
)
from robot_agent.blast_navigation_motion_execution import (
    BlastNavigationMotionExecutor,
)
from robot_agent.blast_observation_monitor import CONTROLLER_ID, ROBOT_ID
from robot_agent.physical_navigation_contract import (
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
)


class SnapshotController:
    def __init__(self, imu):
        self.clock = 1_000
        self.observation = {
            "distance_mm": 500,
            "motion_active": False,
            "motor_angles_deg": {
                "left_drive": 10,
                "right_drive": 20,
                "body": 158,
            },
            "imu": dict(imu),
        }

    def snapshot(self):
        return {
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "state": "online",
            "last_observed_at_unix_ms": self.clock,
            "last_observed_at_monotonic_ms": self.clock,
            "observation": copy.deepcopy(self.observation),
        }

    def command(self, *_args, **_kwargs):
        raise AssertionError("test must remain motorless")


class BlastIMUDeauthorityTests(unittest.TestCase):
    @staticmethod
    def runtime(imu):
        controller = SnapshotController(imu)
        adapter = BlastEpisodeRuntimeAdapter(
            controller=controller,
            planner_factory=lambda _model: object(),
            monotonic_ms=lambda: controller.clock,
        )
        executor = BlastNavigationMotionExecutor(
            controller=controller,
            initial_observation=controller.observation,
        )
        return controller, adapter, executor

    def test_turn_and_scan_availability_ignore_imu_yaw_state(self):
        for imu in (
            {"ready": False, "stationary": False},
            {"ready": True, "stationary": False, "heading_deg": 38.96},
        ):
            with self.subTest(imu=imu):
                _controller, adapter, _executor = self.runtime(imu)
                observation = adapter._observation()
                self.assertTrue(adapter._current_observation_allows_action(
                    TURN_LEFT_90, observation,
                ))
                self.assertTrue(adapter._current_observation_allows_action(
                    SCAN_FRONT_ARC, observation,
                ))

    def test_side_and_pre_action_authority_is_the_encoder_anchor(self):
        controller, adapter, executor = self.runtime({
            "ready": True,
            "stationary": False,
            "heading_deg": 38.96,
        })
        observation = adapter._observation()
        self.assertTrue(_side_search_encoder_correlated(
            observation, executor,
        ))
        admitted = adapter._fresh_planner_action_observation(
            action=TURN_LEFT_90,
            selects_detour_side=True,
            episode_start_heading=0.0,
            motion_executor=executor,
            cancel_requested=lambda: False,
        )
        self.assertEqual(admitted["sensors"]["imu"]["heading_deg"], 38.96)

        controller.observation["motor_angles_deg"]["left_drive"] += 2
        drifted = adapter._observation()
        self.assertFalse(_side_search_encoder_correlated(drifted, executor))
        with self.assertRaises(BlastEpisodeError) as raised:
            adapter._fresh_planner_action_observation(
                action=TURN_LEFT_90,
                selects_detour_side=True,
                episode_start_heading=0.0,
                motion_executor=executor,
                cancel_requested=lambda: False,
            )
        self.assertEqual(
            raised.exception.code, "blast_action_start_unverified",
        )

    def test_availability_rejects_malformed_encoders_even_without_imu(self):
        controller, adapter, _executor = self.runtime({"ready": False})
        del controller.observation["motor_angles_deg"]["right_drive"]
        observation = adapter._observation()

        self.assertFalse(adapter._current_observation_allows_action(
            TURN_LEFT_90, observation,
        ))
        self.assertFalse(adapter._current_observation_allows_action(
            SCAN_FRONT_ARC, observation,
        ))


if __name__ == "__main__":
    unittest.main()
