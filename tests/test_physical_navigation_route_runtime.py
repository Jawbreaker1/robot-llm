from collections import deque
import unittest

from robot_agent.local_detour_controller import (
    synchronize_local_detour_route,
)
from robot_agent.local_detour_route import ROUTE_COMPLETE, ROUTE_INVALID
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_navigation_contract import (
    EXPECTED_ACTION_SPECS,
    MOTION_ACTIONS,
)
from robot_agent.physical_navigation_experience import (
    ROUTE_EXECUTOR_ACTION_SOURCE,
)
from robot_agent.physical_navigation_mission import DirectionalMission
from robot_agent.physical_navigation_route_runtime import (
    HANDOFF_EPISODE_DEADLINE_ELAPSED,
    HANDOFF_EXECUTION_VETOED,
    HANDOFF_MOTION_NOT_COMPLETED,
    HANDOFF_NO_UNIQUE_FEASIBLE_MOTION,
    HANDOFF_ROUTE_COMPLETE,
    HANDOFF_ROUTE_INVALID,
    HANDOFF_ROUTE_MISSING,
    HANDOFF_ROUTE_REPLAN_REQUIRED,
    PhysicalNavigationRouteRuntimeMixin,
)
from robot_agent.physical_odometry import (
    OdometryCalibration,
    PhysicalPose,
    nominal_effect,
)
from robot_agent.physical_scan_evidence import AngularCollisionSupport
from robot_agent.provisional_hazard_map import (
    HazardMapCalibration,
    ProvisionalHazard,
    ProvisionalHazardMap,
)


TARGET_ID = "box-a"


def footprint():
    return RobotFootprint(
        front_extent_mm=70,
        rear_extent_mm=60,
        left_extent_mm=80,
        right_extent_mm=100,
        clearance_margin_mm=20,
        calibration_status="test",
        calibration_evidence="route runtime fixture",
    )


def hazard(*, collision_supports=()):
    return ProvisionalHazard(
        hypothesis_id=TARGET_ID,
        frame_id="frame-a",
        anchor_x_mm=0,
        anchor_y_mm=0,
        anchor_heading_mdeg=0,
        centroid_x_mm=200,
        centroid_y_mm=0,
        radius_mm=50,
        first_seen_at_ms=1,
        last_seen_at_ms=1,
        evidence_count=1,
        last_state_version=1,
        last_raw_ir_proximity=30,
        last_filtered_ir_proximity=30,
        collision_supports=collision_supports,
    )


def hazard_map(*, value=None, revision=7, generation_id="generation-a"):
    return ProvisionalHazardMap(
        frame_id="frame-a",
        map_generation_id=generation_id,
        hazards=((value or hazard()),),
        revision=revision,
        calibration=HazardMapCalibration(robot_footprint=footprint()),
    )


def mission():
    return DirectionalMission.begin(
        episode_id="episode-a",
        minimum_forward_progress_mm=600,
        pose=PhysicalPose(),
        heading_tolerance_mdeg=20_000,
    )


def authorization(*, side="LEFT_OF_GOAL"):
    return {
        "target_hypothesis_id": TARGET_ID,
        "detour_side": side,
    }


def active_route(*, world=None, side="LEFT_OF_GOAL"):
    world = world or hazard_map()
    synchronized = synchronize_local_detour_route(
        None,
        active_maneuver=authorization(side=side),
        current_pose=PhysicalPose(),
        mission=mission(),
        hazard_map=world,
    )
    if synchronized.route is None:
        raise AssertionError("route fixture did not build")
    return synchronized.route


def all_motion_feasible(allowed=True):
    return {
        action: {"allowed": allowed}
        for action in MOTION_ACTIONS
    }


class FakeCancellation(Exception):
    pass


class FakeMemory:
    def __init__(self, world):
        self.pose = PhysicalPose()
        self.hazard_map = world
        self.odometry_calibration = OdometryCalibration()
        self.localization_valid = True


class RouteRuntimeHost(PhysicalNavigationRouteRuntimeMixin):
    def __init__(self, *, world=None):
        self.memory = FakeMemory(world or hazard_map())
        self.clock = 0.0
        self.cancelled = False
        self.mission_completed = False
        self.goal_calls = 0
        self.navigations = []
        self.cancel_stages = []
        self.events = []
        self.executed_actions = []
        self.experiences = []
        self.vetoes = deque()
        self.motion_statuses = deque()
        self.feasibility_factory = lambda _host: all_motion_feasible()
        self.after_motion = None

    def monotonic(self):
        return self.clock

    def _raise_if_cancelled(self, stage):
        self.cancel_stages.append(stage)
        if self.cancelled:
            raise FakeCancellation(stage)

    def _emit(self, event, **fields):
        self.events.append({"event": event, **fields})

    def _goal_state(self, mission_value, observation, action_specs):
        del mission_value, observation, action_specs
        self.goal_calls += 1
        navigation = {
            "action_feasibility": {
                "motion_actions": self.feasibility_factory(self),
            }
        }
        self.navigations.append(navigation)
        return (
            {"completed": self.mission_completed},
            navigation,
        )

    def _experience_basis(self, observation):
        return {
            "state_version": observation["state_version"],
            "pose": self.memory.pose.to_dict(),
        }

    def _execution_veto(
        self,
        *,
        action,
        observation,
        action_specs,
        deadline,
    ):
        del action, observation, action_specs, deadline
        return self.vetoes.popleft() if self.vetoes else None

    def _execute_motion(
        self,
        action,
        *,
        action_specs,
        turn=None,
        source=None,
    ):
        del turn, source
        if len(self.executed_actions) >= 100:
            raise AssertionError("route runtime did not converge")
        self.executed_actions.append(action)
        status = (
            self.motion_statuses.popleft()
            if self.motion_statuses
            else "completed"
        )
        if status == "completed":
            self.memory.pose = nominal_effect(
                self.memory.pose,
                action,
                action_specs,
                self.memory.odometry_calibration,
            )[0]
        observation = {"state_version": len(self.executed_actions) + 1}
        result = {
            "operation": "pulse",
            "requested_action": action,
            "status": status,
            "reason": "fake_{}".format(status),
            "resulting_pose": self.memory.pose.to_dict(),
        }
        if self.after_motion is not None:
            self.after_motion(self, action)
        return observation, result

    def _record_experience(self, **fields):
        self.experiences.append(fields)


def execute(host, route, **changes):
    values = {
        "turn": 3,
        "deadline": 100.0,
        "observation": {"state_version": 1},
        "action_specs": EXPECTED_ACTION_SPECS,
        "mission": mission(),
        "route": route,
        "active_maneuver": authorization(),
    }
    values.update(changes)
    return host._execute_authorized_local_detour_route(**values)


class PhysicalNavigationRouteRuntimeTests(unittest.TestCase):
    def test_executes_unique_route_actions_until_the_route_is_complete(self):
        host = RouteRuntimeHost()

        result = execute(host, active_route())

        self.assertEqual(result.handoff_reason, HANDOFF_ROUTE_COMPLETE)
        self.assertEqual(result.route.status, ROUTE_COMPLETE)
        self.assertEqual(result.actions, tuple(host.executed_actions))
        self.assertGreater(len(result.actions), 4)
        self.assertEqual(host.goal_calls, len(result.actions) + 1)
        self.assertEqual(len(host.experiences), len(result.actions))
        self.assertTrue(all(
            item["source"] == ROUTE_EXECUTOR_ACTION_SOURCE
            for item in host.experiences
        ))
        self.assertEqual(
            result.last_tool_result["route_execution"]["reason"],
            HANDOFF_ROUTE_COMPLETE,
        )
        self.assertFalse(
            result.last_tool_result["route_execution"][
                "host_selected_route_or_side"
            ]
        )
        updates = [
            item for item in host.events
            if item["event"] == "local_detour_route_updated"
        ]
        self.assertTrue(updates)
        self.assertEqual(updates[-1]["sync_event"], "COMPLETED")
        self.assertEqual(updates[-1]["route"]["status"], ROUTE_COMPLETE)
        self.assertEqual(
            host.navigations[-1]["local_detour_route"]["status"],
            ROUTE_COMPLETE,
        )
        self.assertNotIn(
            "route",
            host.navigations[-1]["local_detour_guidance"],
        )

    def test_missing_or_invalid_route_never_reaches_goal_state(self):
        host = RouteRuntimeHost()
        missing = execute(host, None)
        invalid_route = active_route().invalidate("TARGET_MISSING")
        invalid = execute(host, invalid_route)

        self.assertEqual(missing.handoff_reason, HANDOFF_ROUTE_MISSING)
        self.assertEqual(invalid.handoff_reason, HANDOFF_ROUTE_INVALID)
        self.assertEqual(invalid.route.status, ROUTE_INVALID)
        self.assertEqual(host.goal_calls, 0)
        self.assertEqual(host.executed_actions, [])

    def test_completed_route_returns_without_an_extra_pulse(self):
        completed = active_route()
        for waypoint in completed.waypoints:
            completed = completed.advance_reached(PhysicalPose(
                x_mm=waypoint.x_mm,
                y_mm=waypoint.y_mm,
                heading_mdeg=waypoint.heading_mdeg,
            ))
        self.assertEqual(completed.status, ROUTE_COMPLETE)
        host = RouteRuntimeHost()

        result = execute(host, completed)

        self.assertEqual(result.handoff_reason, HANDOFF_ROUTE_COMPLETE)
        self.assertEqual(host.goal_calls, 0)
        self.assertEqual(result.actions, ())

    def test_ambiguous_or_empty_guidance_returns_control_to_planner(self):
        for mode in ("ambiguous", "empty"):
            with self.subTest(mode=mode):
                host = RouteRuntimeHost()
                route = active_route()
                if mode == "ambiguous":
                    host.memory.pose = PhysicalPose(heading_mdeg=-90_000)
                else:
                    host.feasibility_factory = (
                        lambda _host: all_motion_feasible(False)
                    )

                result = execute(host, route)

                self.assertEqual(
                    result.handoff_reason,
                    HANDOFF_NO_UNIQUE_FEASIBLE_MOTION,
                )
                self.assertEqual(result.actions, ())
                self.assertEqual(host.executed_actions, [])
                allowed = result.last_tool_result["route_execution"][
                    "detail"
                ]["allowed_motion_actions"]
                self.assertEqual(len(allowed), 2 if mode == "ambiguous" else 0)

    def test_fresh_execution_veto_is_recorded_without_dispatching_motion(self):
        host = RouteRuntimeHost()
        host.vetoes.append({
            "code": "swept_path_blocked",
            "hazard_ids": ["new-box"],
        })

        result = execute(host, active_route())

        self.assertEqual(result.handoff_reason, HANDOFF_EXECUTION_VETOED)
        self.assertEqual(result.actions, ())
        self.assertEqual(host.executed_actions, [])
        self.assertEqual(len(host.experiences), 1)
        self.assertEqual(
            host.experiences[0]["result"]["status"],
            "route_action_vetoed",
        )
        self.assertEqual(
            result.last_tool_result["validation"]["hazard_ids"],
            ["new-box"],
        )

    def test_motion_noncompletion_stops_after_the_dispatched_pulse(self):
        host = RouteRuntimeHost()
        host.motion_statuses.append("interrupted")

        result = execute(host, active_route())

        self.assertEqual(result.handoff_reason, HANDOFF_MOTION_NOT_COMPLETED)
        self.assertEqual(len(result.actions), 1)
        self.assertEqual(result.actions, tuple(host.executed_actions))
        self.assertEqual(result.last_tool_result["status"], "interrupted")
        self.assertEqual(len(host.experiences), 1)

    def test_new_target_geometry_returns_rebuilt_route_before_another_pulse(self):
        host = RouteRuntimeHost()
        original = active_route()

        def expand_target(runtime, _action):
            support = AngularCollisionSupport(
                source_scan_id="scan-a",
                completed_at_ms=2,
                pose_x_mm=0,
                pose_y_mm=0,
                pose_heading_mdeg=0,
                actual_relative_bearing_mdeg=90_000,
                based_on_map_version=7,
            )
            runtime.memory.hazard_map = hazard_map(
                value=hazard(collision_supports=(support,)),
                revision=8,
            )

        host.after_motion = expand_target

        result = execute(host, original)

        self.assertEqual(
            result.handoff_reason,
            HANDOFF_ROUTE_REPLAN_REQUIRED,
        )
        self.assertEqual(len(result.actions), 1)
        self.assertNotEqual(result.route.route_id, original.route_id)
        self.assertEqual(result.route.detour_side, original.detour_side)
        self.assertEqual(
            result.last_tool_result["route_execution"]["detail"][
                "reason"
            ],
            "TARGET_GEOMETRY_MISMATCH",
        )
        updates = [
            item for item in host.events
            if item["event"] == "local_detour_route_updated"
        ]
        self.assertEqual(updates[-1]["sync_event"], "REBUILT")
        self.assertEqual(
            updates[-1]["route"]["route_id"],
            result.route.route_id,
        )

    def test_structural_invalidation_emits_the_full_invalid_route(self):
        original = active_route()
        host = RouteRuntimeHost(world=hazard_map(
            generation_id="generation-b",
        ))

        result = execute(host, original)

        self.assertEqual(result.handoff_reason, HANDOFF_ROUTE_INVALID)
        self.assertEqual(result.route.status, ROUTE_INVALID)
        updates = [
            item for item in host.events
            if item["event"] == "local_detour_route_updated"
        ]
        self.assertEqual(len(updates), 1)
        self.assertEqual(updates[0]["sync_event"], "INVALIDATED")
        self.assertEqual(updates[0]["route"], result.route.to_dict())
        self.assertEqual(host.executed_actions, [])

    def test_lost_localization_requires_planner(self):
        host = RouteRuntimeHost()
        host.memory.localization_valid = False

        result = execute(host, active_route())

        self.assertEqual(result.handoff_reason, HANDOFF_ROUTE_REPLAN_REQUIRED)
        self.assertEqual(result.actions, ())
        self.assertEqual(
            result.last_tool_result["route_execution"]["detail"]["reason"],
            "LOCALIZATION_INVALID",
        )

    def test_mission_completion_does_not_skip_merge_and_resume_waypoints(self):
        host = RouteRuntimeHost()
        host.mission_completed = True

        result = execute(host, active_route())

        self.assertEqual(result.handoff_reason, HANDOFF_ROUTE_COMPLETE)
        self.assertEqual(result.route.status, ROUTE_COMPLETE)
        self.assertGreater(len(result.actions), 4)

    def test_deadline_and_cancellation_are_checked_before_state_or_motion(self):
        deadline_host = RouteRuntimeHost()
        deadline_host.clock = 10.0

        result = execute(deadline_host, active_route(), deadline=10.0)

        self.assertEqual(
            result.handoff_reason,
            HANDOFF_EPISODE_DEADLINE_ELAPSED,
        )
        self.assertEqual(deadline_host.goal_calls, 0)
        self.assertEqual(deadline_host.executed_actions, [])

        cancelled_host = RouteRuntimeHost()
        cancelled_host.cancelled = True
        with self.assertRaises(FakeCancellation):
            execute(cancelled_host, active_route())
        self.assertEqual(cancelled_host.goal_calls, 0)
        self.assertEqual(cancelled_host.executed_actions, [])

    def test_active_authorization_change_rebuilds_but_does_not_execute_it(self):
        host = RouteRuntimeHost()
        original = active_route(side="LEFT_OF_GOAL")

        result = execute(
            host,
            original,
            active_maneuver=authorization(side="RIGHT_OF_GOAL"),
        )

        self.assertEqual(
            result.handoff_reason,
            HANDOFF_ROUTE_REPLAN_REQUIRED,
        )
        self.assertEqual(result.route.detour_side, "RIGHT_OF_GOAL")
        self.assertEqual(result.actions, ())
        self.assertEqual(host.executed_actions, [])
        self.assertEqual(
            result.last_tool_result["route_execution"]["detail"][
                "reason"
            ],
            "MODEL_ROUTE_CHANGED",
        )


if __name__ == "__main__":
    unittest.main()
