import threading
import unittest

from robot_agent.multi_robot_navigation_simulator import (
    MultiRobotNavigationSimulator,
    RectangleObstacle,
    SimulatedRobot,
    SimulationGoal,
)
from robot_agent.navigation_state import PoseEstimate
from robot_agent.physical_footprint import RobotFootprint


def footprint(front=110, rear=70, left=100, right=100):
    return RobotFootprint(
        front_extent_mm=front,
        rear_extent_mm=rear,
        left_extent_mm=left,
        right_extent_mm=right,
        clearance_margin_mm=10,
        calibration_status="simulation-test",
        calibration_evidence="measured robot-sized test body",
    )


def two_robot_world(*, obstacles=()):
    return MultiRobotNavigationSimulator(
        bounds=(-1_000, -1_000, 2_000, 1_000),
        obstacles=obstacles,
        robots=(
            SimulatedRobot(
                robot_id="blast",
                pose=PoseEstimate(0, -350, 0),
                footprint=footprint(),
                sensor_range_mm=1_500,
            ),
            SimulatedRobot(
                robot_id="ev3",
                pose=PoseEstimate(0, 350, 0),
                footprint=footprint(rear=90, right=130),
                sensor_range_mm=1_000,
            ),
        ),
        goals=(
            SimulationGoal("blast", 1_000, -350),
            SimulationGoal("ev3", 1_000, 350),
        ),
    )


class MultiRobotNavigationSimulatorTests(unittest.TestCase):
    def test_world_contains_no_route_or_waypoint_policy(self):
        world = two_robot_world()

        self.assertFalse(hasattr(world, "route"))
        self.assertFalse(hasattr(world, "waypoints"))
        self.assertEqual(world.goals["blast"].x_mm, 1_000)

    def test_measured_robot_body_stops_before_rectangular_obstacle(self):
        box = RectangleObstacle("box", 300, -500, 600, -200)
        world = two_robot_world(obstacles=(box,))

        moved = world.move("blast", 800)

        self.assertLess(moved, 300)
        self.assertEqual(world.events[-1].kind, "blocked")
        self.assertEqual(world.pose("blast").x_mm, moved)

    def test_robot_is_visible_and_blocks_a_peer(self):
        world = MultiRobotNavigationSimulator(
            bounds=(-1_000, -1_000, 2_000, 1_000),
            obstacles=(),
            robots=(
                SimulatedRobot(
                    "blast",
                    PoseEstimate(0, 0, 0),
                    footprint(),
                    1_500,
                ),
                SimulatedRobot(
                    "ev3",
                    PoseEstimate(600, 0, 180_000 - 1),
                    footprint(rear=90, right=130),
                    1_000,
                ),
            ),
            goals=(
                SimulationGoal("blast", 1_000, 0),
                SimulationGoal("ev3", -500, 0),
            ),
        )

        reading = world.scan("blast", (0,))[0]
        moved = world.move("blast", 1_000)

        self.assertEqual(reading[2], "ev3")
        self.assertLess(moved, 600)
        self.assertEqual(world.events[-1].kind, "blocked")

    def test_blast_and_ev3_runners_share_one_concurrent_world(self):
        world = two_robot_world()
        release = threading.Barrier(2)

        def blast_runner(simulator):
            release.wait()
            simulator.move("blast", 300)
            return simulator.scan("blast", (0, 90_000, -90_000))

        def ev3_runner(simulator):
            release.wait()
            simulator.move("ev3", 300)
            return simulator.scan("ev3", (0, 90_000, -90_000))

        results = world.run_concurrently({
            "blast": blast_runner,
            "ev3": ev3_runner,
        })

        self.assertEqual(set(results), {"blast", "ev3"})
        self.assertEqual(world.pose("blast").x_mm, 300)
        self.assertEqual(world.pose("ev3").x_mm, 300)
        kinds = [event.kind for event in world.events]
        stopped_at = kinds.index("agent_stopped")
        self.assertEqual(kinds[:stopped_at].count("agent_started"), 2)


if __name__ == "__main__":
    unittest.main()
