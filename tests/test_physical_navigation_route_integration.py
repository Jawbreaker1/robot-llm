import copy
from pathlib import Path
import tempfile
import unittest
import uuid

from robot_agent.active_ir_scan_contract import ActiveIrRay, ActiveIrScanResult
from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
from robot_agent.lm_studio_navigation import (
    LMStudioNavigationDecisionError,
)
from robot_agent.maneuver_commitment import (
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
)
from robot_agent.navigation_memory_store import NavigationMemoryStore
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    FINISH,
    MOTION_ACTIONS,
    OBSERVE,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    NavigationDecision,
)
from robot_agent.physical_navigation_runtime import (
    PhysicalNavigationRuntime,
    PhysicalNavigationRuntimeConfig,
)
from robot_agent.physical_navigation_route_runtime import (
    EXECUTION_REPLAN,
    ROUTE_EXECUTION_REASON_REPLAN_REQUIRED,
)
from robot_agent.physical_odometry import (
    DriveMotorRoles,
    OdometryCalibration,
)
from robot_agent.provisional_hazard_map import HazardMapCalibration
from tests.test_physical_navigation_core import (
    FakeRuntimeTransport,
    decision_mapping,
    observation,
)


class RouteAwareRuntimeTransport(FakeRuntimeTransport):
    """Return direction-correct encoder receipts for every semantic motion."""

    DELTAS = {
        ADVANCE: (200, 200),
        REVERSE: (-200, -200),
        TURN_LEFT_90: (-682, 682),
        TURN_RIGHT_90: (682, -682),
    }

    def request(
        self,
        operation,
        arguments,
        timeout,
        cancel_requested=None,
    ):
        before_left = self.left
        before_right = self.right
        response = super().request(
            operation,
            arguments,
            timeout,
            cancel_requested=cancel_requested,
        )
        if operation != "pulse":
            return response
        left_delta, right_delta = self.DELTAS[arguments["action"]]
        self.left = before_left + left_delta
        self.right = before_right + right_delta
        result = response["result"]
        receipt = result["outcome"]["slices"][0]
        for motor, before, after, delta in zip(
            receipt["motors"],
            (before_left, before_right),
            (self.left, self.right),
            (left_delta, right_delta),
        ):
            motor.update({
                "position_before": before,
                "position_after": after,
                "position_delta": delta,
            })
        for motor, position in zip(
            result["observation"]["motors"],
            (self.left, self.right),
        ):
            motor["position"] = position
        return response


class RouteAuthorizationPlanner:
    def __init__(self):
        self.calls = 0
        self.contexts = []

    def decide(self, **context):
        self.calls += 1
        self.contexts.append(copy.deepcopy(context))
        active = context["maneuver_state"]["active"]
        target_ids = tuple(
            item["hypothesis_id"]
            for item in context["navigation"][
                "navigation_hazard_hypotheses"
            ]
        )
        if active is None:
            target = target_ids[0]
            commitment = {
                "id": "route-integration",
                "revision": 1,
                "transition": "START",
                "objective": "Pass the remembered obstacle",
                "target_hypothesis_id": target,
                "detour_side": "LEFT_OF_GOAL",
                "success_fact_keys": [
                    FACT_GOAL_CORRIDOR_CLEAR,
                    FACT_GOAL_HEADING_ALIGNED,
                    FACT_TARGET_BEHIND,
                ],
                "current_focus_fact_key": FACT_GOAL_CORRIDOR_CLEAR,
                "revision_reason": None,
            }
            value = decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=OBSERVE,
                plan=[OBSERVE],
                reason_code="HANDLE_OBSTACLE",
                commitment=commitment,
            )
        elif (
            context["last_tool_result"]
            .get("route_execution", {})
            .get("reason_code")
            == ROUTE_EXECUTION_REASON_REPLAN_REQUIRED
        ):
            commitment = {
                "id": active["id"],
                "revision": active["revision"],
                "transition": "CONTINUE",
                "objective": active["objective"],
                "target_hypothesis_id": active["target_hypothesis_id"],
                "detour_side": active["detour_side"],
                "success_fact_keys": active["success_fact_keys"],
                "current_focus_fact_key": active["current_focus_fact_key"],
                "revision_reason": None,
            }
            value = decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=OBSERVE,
                plan=[OBSERVE],
                reason_code="HANDLE_OBSTACLE",
                commitment=commitment,
            )
        else:
            commitment = {
                "id": active["id"],
                "revision": active["revision"],
                "transition": "COMPLETE",
                "objective": active["objective"],
                "target_hypothesis_id": active["target_hypothesis_id"],
                "detour_side": active["detour_side"],
                "success_fact_keys": active["success_fact_keys"],
                "current_focus_fact_key": None,
                "revision_reason": None,
            }
            value = decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=FINISH,
                plan=[FINISH],
                reason_code="COMPLETE_GOAL",
                commitment=commitment,
            )
        return NavigationDecision.from_mapping(
            value,
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=target_ids,
        )


class ReplanAfterOneRoutePulseRuntime(PhysicalNavigationRuntime):
    """Expose a geometry-change handoff after one real route pulse."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.route_attempts = 0

    def _execute_authorized_local_detour_route(
        self,
        *,
        turn,
        deadline,
        observation,
        action_specs,
        mission,
        route,
        active_maneuver,
        last_tool_result=None,
    ):
        del deadline, mission, active_maneuver, last_tool_result
        self.route_attempts += 1
        self._emit(
            "local_detour_route_action_started",
            action=ADVANCE,
            route={"route_id": route.route_id},
            host_selected_route_or_side=False,
        )
        next_observation, result = self._execute_motion(
            ADVANCE,
            action_specs=action_specs,
        )
        return self._route_runtime_result(
            observation=next_observation,
            route=route,
            last_tool_result=result,
            actions=[ADVANCE],
            outcome=EXECUTION_REPLAN,
            reason_code=ROUTE_EXECUTION_REASON_REPLAN_REQUIRED,
            detail={"reason": "test_geometry_changed_after_pulse"},
        )


def route_ready_memory(
    path: Path,
    *,
    hazard_calibration=None,
    odometry_calibration=OdometryCalibration(),
):
    if hazard_calibration is None:
        footprint = RobotFootprint(
            front_extent_mm=45,
            rear_extent_mm=40,
            left_extent_mm=45,
            right_extent_mm=55,
            clearance_margin_mm=10,
            calibration_status="test",
            calibration_evidence="runtime route integration fixture",
        )
        hazard_calibration = HazardMapCalibration(
            provisional_hazard_offset_mm=300,
            provisional_hazard_radius_mm=40,
            robot_footprint=footprint,
        )
    memory = NavigationMemoryStore.load(
        path=path,
        robot_id="ev3rstorm-01",
        controller_instance_id="ev3-main",
        reset=True,
        clock_ms=lambda: 1_000,
        uuid_factory=lambda: uuid.UUID(int=551),
        hazard_calibration=hazard_calibration,
        odometry_calibration=odometry_calibration,
    )
    memory.bind_drive_roles(DriveMotorRoles("drive_b", "drive_c"))
    memory.begin_episode(observation(
        1,
        blocked=True,
        left_role="drive_b",
        right_role="drive_c",
    ), 1_001)
    target = memory.hazard_map.hazard_ids[0]
    scan_basis = memory.hazard_map.revision
    memory.ingest_stationary_observation(observation(
        2,
        blocked=True,
        left_role="drive_b",
        right_role="drive_c",
    ), 2_000)
    rays = tuple(
        ActiveIrRay(
            ordinal=index,
            requested_relative_bearing_mdeg=bearing,
            actual_relative_bearing_mdeg=bearing,
            observed_at_ms=2_100 + index,
            state_version=2 + index,
            raw=60 if clear else 20,
            filtered=60 if clear else 20,
            blocked=not clear,
        )
        for index, (bearing, clear) in enumerate(
            ((-30_000, True), (-10_000, False),
             (10_000, False), (30_000, True)),
            start=1,
        )
    )
    memory.hazard_map.record_scan_result(
        ActiveIrScanResult(
            scan_id="route-integration-scan",
            target_hypothesis_id=target,
            frame_id=memory.frame_id,
            map_generation_id=memory.generation_id,
            based_on_map_version=scan_basis,
            started_at_ms=2_100,
            completed_at_ms=2_200,
            status="COMPLETED",
            reason="bilateral_boundaries_observed",
            stop_confirmed=True,
            restored_start_heading=True,
            rays=rays,
            left_boundary_mdeg=20_000,
            right_boundary_mdeg=-20_000,
        ),
        scan_pose=memory.pose,
    )
    memory.save()
    return memory


class PhysicalNavigationRouteIntegrationTests(unittest.TestCase):
    def test_ev3_profile_backs_off_then_completes_authorized_route(self):
        with tempfile.TemporaryDirectory() as directory:
            profile = EV3RSTORMProfile()
            memory = route_ready_memory(
                Path(directory) / "memory.json",
                hazard_calibration=profile.hazard_calibration,
                odometry_calibration=profile.odometry_calibration,
            )
            transport = RouteAwareRuntimeTransport(blocked=False)
            planner = RouteAuthorizationPlanner()
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-ev3-backoff-integration",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Pass the obstacle and resume the original heading",
                    locale="en",
                    minimum_forward_progress_mm=120,
                    max_turns=6,
                    max_episode_seconds=300,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 3_000,
            )

            result = runtime.run()

        self.assertTrue(result.completed)
        self.assertEqual(result.model_calls, 2)
        self.assertEqual(result.actions[1], REVERSE)
        self.assertIn(TURN_LEFT_90, result.actions)
        self.assertEqual(result.actions[-1], FINISH)

    def test_real_runtime_executes_full_route_without_planner_per_pulse(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = route_ready_memory(Path(directory) / "memory.json")
            transport = RouteAwareRuntimeTransport(blocked=False)
            planner = RouteAuthorizationPlanner()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-route-integration",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Pass the obstacle and resume the original heading",
                    locale="en",
                    minimum_forward_progress_mm=120,
                    max_turns=6,
                    max_episode_seconds=300,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 3_000,
                event_sink=events.append,
            )

            result = runtime.run()

        motion_actions = tuple(
            action for action in result.actions if action in MOTION_ACTIONS
        )
        self.assertTrue(result.completed)
        self.assertEqual(result.terminal_reason, "goal_completed")
        self.assertEqual(result.model_calls, 2)
        self.assertEqual(planner.calls, 2)
        self.assertGreater(len(motion_actions), 4)
        self.assertLess(planner.calls, len(motion_actions))
        self.assertEqual(result.actions[0], OBSERVE)
        self.assertEqual(result.actions[-1], FINISH)
        self.assertEqual(
            planner.contexts[1]["navigation"][
                "local_detour_guidance"
            ]["reason"],
            "ROUTE_COMPLETE",
        )
        self.assertIn(ADVANCE, planner.contexts[1]["available_actions"])
        self.assertEqual(
            len([
                call for call in transport.calls if call[0] == "pulse"
            ]),
            len(motion_actions),
        )
        route_updates = [
            event for event in events
            if event["event"] == "local_detour_route_updated"
        ]
        self.assertTrue(route_updates)
        self.assertEqual(route_updates[-1]["route"]["status"], "COMPLETE")
        decisions = [
            index for index, event in enumerate(events)
            if event["event"] == "model_decision"
        ]
        route_motions = [
            index for index, event in enumerate(events)
            if event["event"] == "local_detour_route_action_started"
        ]
        self.assertTrue(route_motions)
        self.assertTrue(all(
            decisions[0] < index < decisions[1] for index in route_motions
        ))

    def test_geometry_change_after_route_pulse_returns_to_planner(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = route_ready_memory(Path(directory) / "memory.json")
            transport = RouteAwareRuntimeTransport(blocked=False)
            planner = RouteAuthorizationPlanner()
            events = []
            runtime = ReplanAfterOneRoutePulseRuntime(
                episode_id="episode-route-replan-integration",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Pass the obstacle and resume the original heading",
                    locale="en",
                    minimum_forward_progress_mm=120,
                    max_turns=2,
                    max_episode_seconds=300,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 3_000,
                event_sink=events.append,
            )

            result = runtime.run()

        self.assertFalse(result.completed)
        self.assertEqual(runtime.route_attempts, 1)
        self.assertEqual(planner.calls, 2)
        self.assertEqual(result.actions, (OBSERVE, ADVANCE, OBSERVE))
        route_motion_index = next(
            index for index, event in enumerate(events)
            if event["event"] == "local_detour_route_action_started"
        )
        planner_indices = [
            index for index, event in enumerate(events)
            if event["event"] == "model_decision"
        ]
        self.assertLess(planner_indices[0], route_motion_index)
        self.assertLess(route_motion_index, planner_indices[1])

    def test_deferred_planner_retry_cannot_resume_active_route(self):
        class InvalidAfterAuthorizationPlanner(RouteAuthorizationPlanner):
            def decide(self, **context):
                if self.calls >= 1:
                    self.calls += 1
                    self.contexts.append(copy.deepcopy(context))
                    raise LMStudioNavigationDecisionError(
                        "invalid_action_reason",
                        "Action and reason disagree",
                        latency_ms=1,
                    )
                return super().decide(**context)

        with tempfile.TemporaryDirectory() as directory:
            runtime = ReplanAfterOneRoutePulseRuntime(
                episode_id="episode-route-deferred-planner",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Pass the obstacle and resume the original heading",
                    locale="en",
                    minimum_forward_progress_mm=120,
                    max_turns=3,
                    max_episode_seconds=300,
                    max_validation_attempts=1,
                ),
                transport=RouteAwareRuntimeTransport(blocked=False),
                planner=InvalidAfterAuthorizationPlanner(),
                memory=route_ready_memory(
                    Path(directory) / "memory.json"
                ),
                monotonic=lambda: 0.0,
                unix_ms=lambda: 3_000,
            )

            result = runtime.run()

        self.assertEqual(runtime.route_attempts, 1)
        self.assertEqual(result.actions, (OBSERVE, ADVANCE))
        self.assertEqual(result.model_calls, 3)
        self.assertEqual(result.terminal_reason, "reasoning_unavailable")


if __name__ == "__main__":
    unittest.main()
