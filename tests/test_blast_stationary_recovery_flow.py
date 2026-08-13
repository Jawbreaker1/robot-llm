import copy
import unittest
from types import SimpleNamespace

from robot_agent.blast_episode_adapter import BlastEpisodeRuntimeAdapter
from robot_agent.blast_navigation_motion_execution import (
    BlastNavigationMotionExecutor,
)
from robot_agent.blast_observation_monitor import (
    COMMAND_RESULT_SCHEMA,
    CONTROLLER_ID,
    ROBOT_ID,
)
from robot_agent.blast_stationary_evidence import (
    BlastStationaryEvidenceStatus,
)
from robot_agent.blast_stationary_recovery_flow import (
    collect_episode_stationary_evidence,
)


class Controller:
    def __init__(self, distances=(500,), *, clock=None):
        self.distances = list(distances)
        self.clock = clock if clock is not None else [1_000]
        self.generation = 1
        self.commands = []
        self.snapshot_value = {
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "state": "online",
            "last_observed_at_unix_ms": 1_000,
            "last_observed_at_monotonic_ms": self.clock[0],
            "observation": self.observation(500),
        }

    @staticmethod
    def observation(distance):
        return {
            "distance_mm": distance,
            "motion_active": False,
            "motor_angles_deg": {
                "left_drive": 0,
                "right_drive": 0,
                "body": 158,
            },
            "imu": {"heading_deg": 0.0},
        }

    def snapshot(self):
        return copy.deepcopy(self.snapshot_value)

    def runtime_generation(self):
        return self.generation

    def command(self, command, *, cancel_requested=None):
        self.commands.append(command)
        if cancel_requested is not None and cancel_requested():
            raise AssertionError("stationary command ignored control")
        distance = self.distances.pop(0)
        self.clock[0] += 1
        observation = self.observation(distance)
        self.snapshot_value.update({
            "last_observed_at_monotonic_ms": self.clock[0],
            "observation": observation,
        })
        return {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": command,
            "accepted": True,
            "completed": True,
            "observation_settled": True,
            "observation": copy.deepcopy(observation),
        }


def context():
    return SimpleNamespace(
        episode_id="episode-1",
        request=SimpleNamespace(goal="forward", locale="en"),
        settings=SimpleNamespace(model="local/model"),
        stop_requested=SimpleNamespace(is_set=lambda: False),
        emergency_stop_requested=SimpleNamespace(is_set=lambda: False),
        publish=lambda _update: None,
    )


class Deadline:
    def __init__(self, result=None):
        self.result = result

    def outcome(self, **_values):
        return self.result


class BlastStationaryRecoveryFlowTests(unittest.TestCase):
    @staticmethod
    def adapter(controller):
        return BlastEpisodeRuntimeAdapter(
            controller=controller,
            planner_factory=lambda _model: object(),
            monotonic_ms=lambda: controller.clock[0],
        )

    def test_measured_recovery_returns_episode_observation_and_receipt(self):
        controller = Controller((480,))
        adapter = self.adapter(controller)
        executor = BlastNavigationMotionExecutor(
            controller=controller,
            initial_observation=controller.snapshot_value["observation"],
        )

        result = collect_episode_stationary_evidence(
            adapter,
            context=context(),
            deadline_ms=Deadline(),
            motion_executor=executor,
            episode_start_heading=0.0,
        )

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.MEASURED_SAFE,
        )
        self.assertEqual(result.observation["sensors"]["distance_mm"], 480)
        self.assertEqual(result.observation["odometry"], executor.pose.to_dict())
        self.assertEqual(
            result.prior_receipt["stationary_evidence_status"],
            "MEASURED_SAFE",
        )
        self.assertEqual(controller.commands, ["observe_settled"])

    def test_missing_imu_heading_does_not_veto_exact_stationary_anchor(self):
        class IMUUnavailableController(Controller):
            @staticmethod
            def observation(distance):
                value = Controller.observation(distance)
                value["imu"] = {"ready": False, "stationary": False}
                return value

        controller = IMUUnavailableController((480,))
        adapter = self.adapter(controller)
        executor = BlastNavigationMotionExecutor(
            controller=controller,
            initial_observation=controller.snapshot_value["observation"],
        )

        result = collect_episode_stationary_evidence(
            adapter,
            context=context(),
            deadline_ms=Deadline(),
            motion_executor=executor,
            episode_start_heading=None,
        )

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.MEASURED_SAFE,
        )
        self.assertNotIn(
            "heading_deg", result.observation["sensors"]["imu"],
        )
        self.assertFalse(result.observation["sensors"]["imu"]["stationary"])

    def test_exact_nvd_is_preserved_and_never_becomes_measured_clearance(self):
        controller = Controller((2_000,))

        result = collect_episode_stationary_evidence(
            self.adapter(controller),
            context=context(),
            deadline_ms=Deadline(),
        )

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.EXACT_NVD,
        )
        self.assertNotEqual(
            result.status, BlastStationaryEvidenceStatus.MEASURED_SAFE,
        )
        self.assertEqual(result.observation["sensors"]["distance_mm"], 2_000)

    def test_stop_or_deadline_wins_before_any_stationary_command(self):
        controller = Controller((500,))
        deadline = Deadline(("stopped", "Stop requested"))

        result = collect_episode_stationary_evidence(
            self.adapter(controller),
            context=context(),
            deadline_ms=deadline,
        )

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.CONTROLLED,
        )
        self.assertEqual(result.control.terminal_reason, "stopped")
        self.assertEqual(controller.commands, [])

    def test_missing_generation_fails_closed_without_command(self):
        controller = Controller((500,))
        controller.runtime_generation = None

        result = collect_episode_stationary_evidence(
            self.adapter(controller),
            context=context(),
            deadline_ms=Deadline(),
        )

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.EXHAUSTED,
        )
        self.assertEqual(
            result.evidence.reason, "session_generation_unavailable",
        )
        self.assertEqual(controller.commands, [])

    def test_close_anchor_fails_before_any_new_settled_command(self):
        controller = Controller((500,))
        controller.snapshot_value["observation"]["distance_mm"] = 40
        adapter = self.adapter(controller)
        executor = BlastNavigationMotionExecutor(
            controller=controller,
            initial_observation=controller.snapshot_value["observation"],
        )

        result = collect_episode_stationary_evidence(
            adapter,
            context=context(),
            deadline_ms=Deadline(),
            motion_executor=executor,
            episode_start_heading=0.0,
        )

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.EXHAUSTED,
        )
        self.assertEqual(
            result.evidence.reason, "stationary_anchor_not_recoverable",
        )
        self.assertEqual(result.evidence.settled_attempts, 0)
        self.assertEqual(controller.commands, [])


if __name__ == "__main__":
    unittest.main()
