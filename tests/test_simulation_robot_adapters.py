import unittest

from robot_agent.active_ir_scan_contract import (
    ModelScanChoice,
    build_scan_request,
)
from robot_agent.multi_robot_navigation_simulator import (
    MultiRobotNavigationSimulator,
    RectangleObstacle,
    SimulatedRobot,
    SimulationGoal,
)
from robot_agent.navigation_state import PoseEstimate
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_navigation_execution_contract import (
    EV3NavigationExecutionContract,
)
from robot_agent.physical_odometry import (
    DriveMotorRoles,
    OdometryCalibration,
    PhysicalPose,
    verified_motion_from_result,
)
from robot_agent.simulation_robot_adapters import (
    SharedWorldBlastController,
    SharedWorldEV3Transport,
)


EV3_ODOMETRY = OdometryCalibration(
    linear_mm_per_encoder_degree=0.35,
    turn_mdeg_per_opposed_encoder_degree=132,
)


def footprint(*, rear=60, right=100):
    return RobotFootprint(
        front_extent_mm=110,
        rear_extent_mm=rear,
        left_extent_mm=105,
        right_extent_mm=right,
        clearance_margin_mm=10,
        calibration_status="simulation-test",
        calibration_evidence="current LEGO robot dimensions",
    )


def shared_world(*, obstacles=()):
    return MultiRobotNavigationSimulator(
        bounds=(-1_000, -1_000, 2_000, 1_000),
        obstacles=obstacles,
        robots=(
            SimulatedRobot(
                "blast",
                PoseEstimate(0, -350, 0),
                footprint(),
                1_500,
            ),
            SimulatedRobot(
                "ev3",
                PoseEstimate(0, 350, 0),
                footprint(rear=90, right=130),
                1_000,
            ),
        ),
        goals=(
            SimulationGoal("blast", 1_000, -350),
            SimulationGoal("ev3", 1_000, 350),
        ),
    )


class SimulationRobotAdapterTests(unittest.TestCase):
    def test_blast_and_ev3_commands_move_in_the_same_concurrent_world(self):
        world = shared_world()
        blast = SharedWorldBlastController(
            world,
            world_robot_id="blast",
        )
        ev3 = SharedWorldEV3Transport(
            world,
            world_robot_id="ev3",
            odometry=EV3_ODOMETRY,
        )

        def run_blast(_simulation):
            return blast.command("drive_forward")

        def run_ev3(_simulation):
            ev3.start()
            description = ev3.request("describe", {}, 1.0)
            EV3NavigationExecutionContract.parse_description(description)
            return ev3.request("pulse", {"action": "ADVANCE"}, 1.0)

        results = world.run_concurrently({
            "blast": run_blast,
            "ev3": run_ev3,
        })

        ev3_observation = EV3NavigationExecutionContract.parse_observation(
            "pulse",
            results["ev3"],
            "ADVANCE",
        )
        motion = verified_motion_from_result(
            "ADVANCE",
            results["ev3"]["result"],
            DriveMotorRoles(left="drive_b", right="drive_c"),
        )
        self.assertEqual(world.pose("blast").x_mm, 45)
        self.assertEqual(world.pose("ev3").x_mm, 52)
        self.assertEqual(
            results["blast"]["observation"]["motor_angles_deg"][
                "left_drive"
            ],
            90,
        )
        self.assertEqual(ev3_observation["state_version"], 2)
        self.assertEqual(motion.left_encoder_delta_degrees, 150)

    def test_ev3_active_scan_uses_world_geometry_and_restores_heading(self):
        world = MultiRobotNavigationSimulator(
            bounds=(-1_000, -1_000, 2_000, 1_000),
            obstacles=(RectangleObstacle("box", 200, -100, 400, 100),),
            robots=(SimulatedRobot(
                "ev3",
                PoseEstimate(0, 0, 0),
                footprint(rear=90, right=130),
                1_000,
            ),),
            goals=(SimulationGoal("ev3", 1_000, 0),),
        )
        ev3 = SharedWorldEV3Transport(
            world,
            world_robot_id="ev3",
            odometry=EV3_ODOMETRY,
        )
        ev3.start()
        monotonic_ms = ev3.clock_ms()
        request = build_scan_request(
            choice=ModelScanChoice("hazard-1"),
            frame_id="simulation-frame",
            map_generation_id="simulation-map",
            map_version=0,
            start_pose=PhysicalPose(),
            start_state_version=1,
            created_at_ms=1_700_000_000_000,
            deadline_ms=1_700_000_060_000,
            created_monotonic_ms=monotonic_ms,
            deadline_monotonic_ms=monotonic_ms + 60_000,
        )

        result = ev3.build_scan_executor().execute(request)

        self.assertEqual(result.status, "COMPLETED")
        self.assertTrue(result.bilateral_complete)
        self.assertTrue(any(ray.blocked for ray in result.rays))
        self.assertTrue(any(not ray.blocked for ray in result.rays))
        self.assertEqual(world.pose("ev3").heading_mdeg, 0)

    def test_both_adapters_observe_the_same_obstacle_without_route_hints(self):
        obstacles = (
            RectangleObstacle("blast-box", 250, -450, 500, -250),
            RectangleObstacle("ev3-box", 200, 250, 450, 450),
        )
        world = shared_world(obstacles=obstacles)
        blast = SharedWorldBlastController(world, world_robot_id="blast")
        ev3 = SharedWorldEV3Transport(
            world,
            world_robot_id="ev3",
            odometry=EV3_ODOMETRY,
        )
        ev3.start()

        blast_observation = blast.snapshot()["observation"]
        ev3_description = EV3NavigationExecutionContract.parse_description(
            ev3.request("describe", {}, 1.0)
        )[0]

        self.assertLess(blast_observation["distance_mm"], 200)
        self.assertTrue(ev3_description["infrared"]["blocked"])
        self.assertFalse(hasattr(blast, "route"))
        self.assertFalse(hasattr(ev3, "waypoints"))

    def test_blast_trim_completes_one_surroundings_rotation(self):
        world = shared_world()
        blast = SharedWorldBlastController(world, world_robot_id="blast")
        angles = blast.snapshot()["observation"]["motor_angles_deg"]
        permit = blast.issue_no_return_scan_permit(
            expected_drive_angles=angles,
        )

        result = blast.scan_surroundings(action_permit=permit)

        self.assertEqual(result["scan"]["sweep_coverage_deg"], 360.15)
        self.assertEqual(result["receipt"], {
            "turn_count": 17,
            "coverage_complete": True,
        })
        self.assertEqual(world.pose("blast").heading_mdeg, 150)

    def test_blast_body_block_retains_partial_encoder_progress(self):
        world = MultiRobotNavigationSimulator(
            bounds=(-1_000, -1_000, 2_000, 1_000),
            obstacles=(RectangleObstacle("box", 180, -100, 400, 100),),
            robots=(SimulatedRobot(
                "blast", PoseEstimate(0, 0, 0), footprint(), 1_500,
            ),),
            goals=(SimulationGoal("blast", 1_000, 0),),
        )
        blast = SharedWorldBlastController(world, world_robot_id="blast")

        partial = blast.command("drive_forward")
        stopped = blast.command("drive_forward")

        self.assertEqual(world.pose("blast").x_mm, 10)
        self.assertEqual(
            partial["observation"]["motor_angles_deg"]["left_drive"],
            20,
        )
        self.assertEqual(
            stopped["observation"]["motor_angles_deg"]["left_drive"],
            20,
        )
        self.assertEqual(
            sum(event.kind == "blocked" for event in world.events),
            2,
        )


if __name__ == "__main__":
    unittest.main()
