import unittest

from robot_agent.active_ir_scan import ActiveIrScanExecutor
from robot_agent.active_ir_scan_contract import (
    ActiveIrScanCalibration,
    ModelScanChoice,
    build_scan_request,
)
from robot_agent.physical_odometry import PhysicalPose


class RecordingTransitionRig:
    def __init__(self):
        self.now = 1_000
        self.heading_mdeg = 0
        self.state_version = 1
        self.turn_deltas_mdeg = []
        self.sampled_bearings_mdeg = []

    def turn_relative_mdeg(self, delta, _calibration, _deadline_ms):
        self.turn_deltas_mdeg.append(delta)
        self.heading_mdeg += delta
        self.now += 1
        return {
            "requested_delta_mdeg": delta,
            "actual_delta_mdeg": delta,
            "completed_at_ms": self.now,
            "stop_confirmed": True,
        }

    def read_snapshot(self, _deadline_ms):
        self.sampled_bearings_mdeg.append(self.heading_mdeg)
        self.state_version += 1
        self.now += 1
        blocked = abs(self.heading_mdeg) < 45_000
        reading = 20 if blocked else 60
        return {
            "state_version": self.state_version,
            "observed_at_ms": self.now,
            "pose_heading_mdeg": self.heading_mdeg,
            "touch_pressed": False,
            "motion_fault_latched": False,
            "infrared": {
                "raw": reading,
                "filtered": reading,
                "blocked": blocked,
            },
        }

    def stop(self):
        return {"stop_confirmed": True}


class ActiveIrScanScheduleTests(unittest.TestCase):
    def test_fine_rays_sweep_from_current_bearing_without_changing_set(self):
        rig = RecordingTransitionRig()
        request = build_scan_request(
            choice=ModelScanChoice("target-a"),
            frame_id="frame-a",
            map_generation_id="generation-a",
            map_version=1,
            start_pose=PhysicalPose(),
            start_state_version=1,
            created_at_ms=rig.now,
            deadline_ms=30_000,
            calibration=ActiveIrScanCalibration(
                estimated_turn_ms_per_degree=1,
            ),
        )

        result = ActiveIrScanExecutor(
            rig=rig,
            clock_ms=lambda: rig.now,
        ).execute(request)

        requested = tuple(
            ray.requested_relative_bearing_mdeg for ray in result.rays
        )
        expected_ray_set = {
            -60_000,
            -45_000,
            -30_000,
            0,
            30_000,
            45_000,
            60_000,
        }
        self.assertEqual(set(requested), expected_ray_set)
        self.assertEqual(len(requested), len(expected_ray_set))
        self.assertEqual(
            requested,
            (
                0,
                -30_000,
                -60_000,
                30_000,
                60_000,
                45_000,
                -45_000,
            ),
        )
        self.assertEqual(
            tuple(rig.sampled_bearings_mdeg),
            requested + (0,),
        )
        self.assertTrue(result.bilateral_complete)
        self.assertEqual(result.left_boundary_mdeg, 37_500)
        self.assertEqual(result.right_boundary_mdeg, -37_500)

        legacy_targets = (
            0,
            -30_000,
            -60_000,
            30_000,
            60_000,
            -45_000,
            45_000,
            0,
        )
        legacy_turn_degrees = sum(
            abs(target - current)
            for current, target in zip(
                legacy_targets,
                legacy_targets[1:],
            )
        )
        legacy_turn_requests = sum(
            target != current
            for current, target in zip(
                legacy_targets,
                legacy_targets[1:],
            )
        )
        scheduled_turn_degrees = sum(
            abs(delta) for delta in rig.turn_deltas_mdeg
        )
        minimum_turn_requests = len(expected_ray_set - {0}) + 1

        self.assertEqual(len(rig.turn_deltas_mdeg), legacy_turn_requests)
        self.assertEqual(len(rig.turn_deltas_mdeg), minimum_turn_requests)
        self.assertEqual(scheduled_turn_degrees, 330_000)
        self.assertEqual(legacy_turn_degrees, 420_000)
        self.assertLess(scheduled_turn_degrees, legacy_turn_degrees)
        self.assertEqual(rig.heading_mdeg, 0)


if __name__ == "__main__":
    unittest.main()
