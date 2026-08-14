import copy
import threading
import unittest
from types import SimpleNamespace

from robot_agent.blast_episode_adapter import (
    ACTION_COMMANDS,
    BlastEpisodeError,
    BlastEpisodeRuntimeAdapter,
)
from robot_agent.blast_observation_monitor import BlastControllerError
from robot_agent.blast_observation_monitor import (
    COMMAND_RESULT_SCHEMA,
    CONTROLLER_ID,
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    ROBOT_ID,
    SCAN_RESULT_SCHEMA,
    SETTLED_OBSERVATION_COMMAND,
)
from robot_agent.blast_navigation_motion_execution import (
    BlastNavigationMotionExecutor,
)
from robot_agent.blast_scan_observation import (
    build_blast_encoder_scan,
    build_blast_partial_scan,
    encoder_common_mode_residue_mm,
    encoder_relative_bearing_deg,
)
from robot_agent.blast_mission_completion import (
    blast_directional_completion_allowed,
)
from robot_agent.blast_turn_safety import (
    blast_turn_slice_allows_continuation,
)
from robot_agent.lm_studio_controller_action import (
    COMPLETE,
    ControllerActionDecision,
    ControllerActionPlannerResult,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    PhysicalNavigationContractError,
)
from robot_agent.physical_navigation_mission import DirectionalMission
from robot_agent.physical_odometry import PhysicalPose
from robot_agent.robot_control_contract import RobotRuntimeUpdate
from robot_agent.robot_speech_runtime import SpeechAdmission


def decision(action, *, plan=(), assessment="ok", utterance=None):
    return ControllerActionPlannerResult(
        decision=ControllerActionDecision(
            action=action,
            confidence_milli=900,
            assessment=assessment,
            plan=tuple(plan),
            utterance=utterance,
        ),
        latency_ms=12,
    )


def encoder_scan_bearing_evidence(requested_bearing_deg):
    """Build exact integer-encoder evidence for a synthetic scan ray."""

    opposed = round(abs(requested_bearing_deg) / 0.490)
    if requested_bearing_deg < 0:
        delta = {"left_drive": -opposed, "right_drive": opposed}
    elif requested_bearing_deg > 0:
        delta = {"left_drive": opposed, "right_drive": -opposed}
    else:
        delta = {"left_drive": 0, "right_drive": 0}
    bearing = encoder_relative_bearing_deg(
        {"motor_angles_deg": delta},
        {"left_drive": 0, "right_drive": 0},
    )
    return delta, bearing


def set_scan_ray_bearing(ray, requested_bearing_deg):
    """Keep a bearing mutation correlated with its encoder evidence."""

    encoder_delta, bearing = encoder_scan_bearing_evidence(
        requested_bearing_deg,
    )
    ray.update({
        "heading_deg": bearing,
        "relative_heading_deg": bearing,
        "imu_heading_deg": requested_bearing_deg,
        "drive_encoder_delta_deg": encoder_delta,
    })
    return ray


def correlate_scan_restoration(scan, observation):
    """Correlate synthetic restoration evidence to final drive encoders."""

    final_angles = {
        role: observation["motor_angles_deg"][role]
        for role in ("left_drive", "right_drive")
    }
    scan["encoder_final_angles_deg"] = final_angles
    bearing = encoder_relative_bearing_deg(
        {"motor_angles_deg": final_angles},
        scan["encoder_start_angles_deg"],
    )
    common_residue = encoder_common_mode_residue_mm(
        final_angles, scan["encoder_start_angles_deg"],
    )
    scan["final_heading_deg"] = bearing
    scan["restoration_error_deg"] = bearing
    scan["encoder_restoration"]["common_mode_residue_mm"] = (
        common_residue
    )
    scan["encoder_restoration"]["opposed_residue_deg"] = bearing
    return scan


def scan_result(*, center_distance_mm=500.0):
    distances = (
        ("center", center_distance_mm),
        ("left_near", 900.0),
        ("left_far", 2_000.0),
        ("right_near", 1_200.0),
        ("right_far", 2_000.0),
    )
    requested_headings = (0.0, -22.0, -45.0, 24.0, 47.0)

    evidence = tuple(
        encoder_scan_bearing_evidence(value)
        for value in requested_headings
    )
    return {
        "schema": SCAN_RESULT_SCHEMA,
        "state": "complete",
        "result": "restored",
        "bearing_source": "DRIVE_ENCODER_ODOMETRY",
        "bearing_frame": "ROBOT_RELATIVE_AT_SCAN_START",
        "start_heading_deg": 0.0,
        "final_heading_deg": 0.0,
        "restoration_error_deg": 0.0,
        "restoration_verified": True,
        "encoder_start_angles_deg": {
            "left_drive": 0, "right_drive": 0,
        },
        "encoder_final_angles_deg": {
            "left_drive": 0, "right_drive": 0,
        },
        "encoder_restoration": {
            "common_mode_residue_mm": 0.0,
            "opposed_residue_deg": 0.0,
            "motion_stopped": True,
            "observation_settled": True,
            "body_pose_verified": True,
        },
        "imu_heading_diagnostics": {
            "authority": "DIAGNOSTIC_ONLY",
            "start_heading_deg": 0.0,
            "final_heading_deg": 0.0,
            "restoration_error_deg": 0.0,
        },
        "all_observations_settled": True,
        "rays": [
            {
                "side": side,
                "distance_mm": distance_mm,
                "range_state": (
                    RANGE_STATE_NO_VALID_DISTANCE
                    if distance_mm == 2_000
                    else RANGE_STATE_MEASURED
                ),
                "body_motor_angle_deg": 158,
                "heading_deg": encoder_evidence[1],
                "relative_heading_deg": encoder_evidence[1],
                "imu_heading_deg": requested_heading,
                "drive_encoder_delta_deg": encoder_evidence[0],
                "observation_settled": True,
            }
            for (
                (side, distance_mm), requested_heading, encoder_evidence,
            ) in zip(
                distances, requested_headings, evidence,
            )
        ],
    }


def dense_scan_result(
    ranges, relative_headings, imu_restoration_error=0.0,
):
    sides = (
        "center", "left_1", "left_2", "left_3", "left_4",
        "right_1", "right_2", "right_3", "right_4",
    )
    angular = [{
        "side": side,
        "distance_mm": distance,
        "range_state": (
            RANGE_STATE_NO_VALID_DISTANCE
            if distance == 2_000 else RANGE_STATE_MEASURED
        ),
        "body_motor_angle_deg": 158,
        "heading_deg": encoder_scan_bearing_evidence(heading)[1],
        "relative_heading_deg": encoder_scan_bearing_evidence(heading)[1],
        "imu_heading_deg": heading,
        "drive_encoder_delta_deg": encoder_scan_bearing_evidence(heading)[0],
        "observation_settled": True,
    } for side, distance, heading in zip(
        sides, ranges, relative_headings,
    )]
    final_delta, final_bearing = encoder_scan_bearing_evidence(0.0)
    return {
        "schema": SCAN_RESULT_SCHEMA,
        "state": "complete",
        "result": "restored",
        "bearing_source": "DRIVE_ENCODER_ODOMETRY",
        "bearing_frame": "ROBOT_RELATIVE_AT_SCAN_START",
        "start_heading_deg": 0.0,
        "final_heading_deg": final_bearing,
        "restoration_error_deg": final_bearing,
        "restoration_verified": True,
        "encoder_start_angles_deg": {
            "left_drive": 0, "right_drive": 0,
        },
        "encoder_final_angles_deg": final_delta,
        "encoder_restoration": {
            "common_mode_residue_mm": 0.0,
            "opposed_residue_deg": final_bearing,
            "motion_stopped": True,
            "observation_settled": True,
            "body_pose_verified": True,
        },
        "imu_heading_diagnostics": {
            "authority": "DIAGNOSTIC_ONLY",
            "start_heading_deg": 0.0,
            "final_heading_deg": imu_restoration_error,
            "restoration_error_deg": imu_restoration_error,
        },
        "all_observations_settled": True,
        "rays": [
            {**copy.deepcopy(angular[index]), "side": side}
            for index, side in (
                (0, "center"), (2, "left_near"), (4, "left_far"),
                (6, "right_near"), (8, "right_far"),
            )
        ],
        "angular_rays": angular,
    }


def surroundings_scan_result(
    center, *, left_forward_mm=500, right_forward_mm=500,
):
    """Build one valid full-turn scan with a settled NVD center ray."""

    start = {
        role: center["motor_angles_deg"][role]
        for role in ("left_drive", "right_drive")
    }
    sweep_samples = []
    for index in range(1, 17):
        observation = copy.deepcopy(center)
        observation["motor_angles_deg"].update({
            "left_drive": start["left_drive"] - 45 * index,
            "right_drive": start["right_drive"] + 45 * index,
        })
        observation["distance_mm"] = (
            left_forward_mm if index == 2
            else right_forward_mm if index == 14
            else 2_000 if index == 16
            else 500
        )
        sweep_samples.append(({}, observation, True, "SETTLED_RANGE"))
    final = sweep_samples[-1][1]
    return build_blast_encoder_scan(
        center=center,
        center_settled=True,
        start_drive_angles=start,
        sweep_samples=sweep_samples,
        final=final,
        final_settled=True,
        final_body_verified=True,
    ), final


def anchor_scan_result(scan, observation):
    """Place synthetic encoder-relative scan evidence at its live anchor."""

    angles = observation["motor_angles_deg"]
    anchor = {
        role: angles[role] for role in ("left_drive", "right_drive")
    }
    final_delta = {
        role: scan["encoder_final_angles_deg"][role]
        - scan["encoder_start_angles_deg"][role]
        for role in ("left_drive", "right_drive")
    }
    scan["encoder_start_angles_deg"] = dict(anchor)
    scan["encoder_final_angles_deg"] = {
        role: anchor[role] + final_delta[role]
        for role in ("left_drive", "right_drive")
    }
    return scan


class FakeController:
    def __init__(self, distance_mm=500):
        self.commands = []
        self.generation = 1
        self.snapshot_value = {
            "robot_id": "blast-01",
            "controller_id": "blast-01.hub",
            "state": "online",
            "last_observed_at_unix_ms": 1_000,
            "last_observed_at_monotonic_ms": 1_000,
            "observation": {
                "distance_mm": distance_mm,
                "motion_active": False,
                "motor_angles_deg": {
                    "left_drive": 0,
                    "right_drive": 0,
                    "claw": 0,
                    "body": 158,
                },
                "imu": {"ready": True, "heading_deg": 0},
            },
        }

    def snapshot(self):
        return self.snapshot_value

    def runtime_generation(self):
        return self.generation

    def command(self, command, *, cancel_requested=None):
        if cancel_requested is not None and cancel_requested():
            raise BlastControllerError(
                "controller_command_interrupted",
                "cancelled",
                motion_started=False,
            )
        self.commands.append(command)
        previous = self.snapshot_value["observation"]
        observation = {
            **previous,
            "motor_angles_deg": dict(previous["motor_angles_deg"]),
            "imu": dict(previous.get("imu", {})),
        }
        before = {
            role: observation["motor_angles_deg"][role]
            for role in ("left_drive", "right_drive")
        }
        receipt = {"accepted": True}
        if command == "drive_forward":
            observation["distance_mm"] -= 45
            deltas = (90, 90)
            receipt.update({
                "direction": "forward",
                "speed_dps": 120,
                "angle_deg": 90,
                "before_angles_deg": before,
            })
        elif command == "drive_reverse":
            observation["distance_mm"] += 45
            deltas = (-90, -90)
            receipt.update({
                "direction": "reverse",
                "speed_dps": 120,
                "angle_deg": 90,
                "before_angles_deg": before,
            })
        elif command in ("turn_left", "turn_right"):
            left = command == "turn_left"
            deltas = (-48, 49) if left else (49, -48)
            receipt.update({
                "direction": "left" if left else "right",
                "speed_dps": 180,
                "wheel_angle_deg": 45,
                "before_angles_deg": before,
            })
            heading = observation["imu"].get("heading_deg", 0)
            observation["imu"]["heading_deg"] = heading + (
                -23.765 if left else 23.765
            )
            observation["rotation_sweep_window_verified"] = True
        else:
            deltas = (0, 0)
            if command == "scan_front_arc":
                receipt = {"turn_count": 8}
        for role, delta in zip(("left_drive", "right_drive"), deltas):
            observation["motor_angles_deg"][role] += delta
        self.snapshot_value = {
            **self.snapshot_value,
            "observation": observation,
        }
        return {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": command,
            "accepted": True,
            "completed": True,
            "receipt": receipt,
            "observation": observation,
            "observation_settled": True,
        }


class FakeScanController(FakeController):
    def command(self, command, *, cancel_requested=None):
        result = super().command(
            command,
            cancel_requested=cancel_requested,
        )
        if command == "scan_front_arc":
            scan = result.get("scan", scan_result())
            result["scan"] = anchor_scan_result(
                scan, result["observation"],
            )
        return result


class ScanStartRetryController(FakeScanController):
    def __init__(
        self,
        *,
        distance_mm=500,
        scan_failures=1,
        failure_code="scan_start_clearance_unverified",
        motion_started=False,
        after_failure=None,
        settled_observation=None,
        settled_result=None,
        settled_timestamp_delta=1,
        after_observe=None,
    ):
        super().__init__(distance_mm)
        self.snapshot_value["last_observed_at_monotonic_ms"] = 998
        self.scan_failures = scan_failures
        self.failure_code = failure_code
        self.motion_started = motion_started
        self.after_failure = after_failure
        self.settled_observation = dict(settled_observation or {})
        self.settled_result = dict(settled_result or {})
        self.settled_timestamp_delta = settled_timestamp_delta
        self.after_observe = after_observe
        self.scan_attempts = 0
        self.scan_permits = []

    def command(
        self, command, *, cancel_requested=None, action_permit=None,
    ):
        if command == "scan_front_arc":
            self.scan_attempts += 1
            self.scan_permits.append(action_permit)
            if self.scan_attempts <= self.scan_failures:
                self.commands.append(command)
                if self.after_failure is not None:
                    self.after_failure(self)
                raise BlastControllerError(
                    self.failure_code,
                    "injected scan failure",
                    motion_started=self.motion_started,
                )
        result = super().command(
            command, cancel_requested=cancel_requested,
        )
        if command == SETTLED_OBSERVATION_COMMAND:
            observation = {
                **result["observation"],
                **self.settled_observation,
            }
            result["observation"] = observation
            result.update(self.settled_result)
            self.snapshot_value["observation"] = observation
            self.snapshot_value["last_observed_at_monotonic_ms"] += (
                self.settled_timestamp_delta
            )
            if self.after_observe is not None:
                self.after_observe(self)
        return result


class FreshStationaryController(FakeScanController):
    """Make observe_settled causally fresh for episode recovery tests."""

    def __init__(self, distance_mm=500, *, recovered_distance_mm=500):
        super().__init__(distance_mm)
        self.clock = [1_000]
        self.recovered_distance_mm = recovered_distance_mm
        self.snapshot_count = 0
        self.reconnect_on_snapshot = None

    def snapshot(self):
        self.snapshot_count += 1
        if self.snapshot_count == self.reconnect_on_snapshot:
            self.snapshot_value["state"] = "online"
            self.generation += 1
        return super().snapshot()

    def command(self, command, *, cancel_requested=None):
        result = super().command(
            command, cancel_requested=cancel_requested,
        )
        if command == SETTLED_OBSERVATION_COMMAND:
            self.clock[0] += 1
            result["observation"]["distance_mm"] = (
                self.recovered_distance_mm
            )
            self.snapshot_value["observation"] = result["observation"]
            self.snapshot_value["last_observed_at_monotonic_ms"] = (
                self.clock[0]
            )
        return result



class Planner:
    def __init__(self, decisions):
        self.decisions = list(decisions)
        self.contexts = []

    def decide(self, context):
        self.contexts.append(context)
        return self.decisions.pop(0)


def episode_context():
    updates = []
    value = SimpleNamespace(
        episode_id="episode-1",
        request=SimpleNamespace(goal="Approach the obstacle", locale="en"),
        settings=SimpleNamespace(model="local/model"),
        stop_requested=threading.Event(),
        emergency_stop_requested=threading.Event(),
        publish=updates.append,
    )
    return value, updates


class BlastEpisodeRuntimeAdapterTests(unittest.TestCase):
    @staticmethod
    def _capture_run(adapter, context, results, errors):
        try:
            results.append(adapter.run(context))
        except Exception as error:
            errors.append(error)

    def adapter(self, controller, planner, **changes):
        monotonic_ms = changes.pop("monotonic_ms", lambda: 1_000)
        startup_perception = changes.pop("startup_perception", False)
        changes.pop("enforce_directional_completion", None)
        adapter = BlastEpisodeRuntimeAdapter(
            controller=controller,
            planner_factory=lambda _model: planner,
            monotonic_ms=monotonic_ms,
            **changes,
        )
        if not startup_perception:
            adapter._run_startup_perception = lambda **values: (
                values["observation"],
                values["available_actions"],
                values["scan_allows_turn"],
                values["latest_scan_view"],
                None,
            )
        return adapter

    def test_startup_perception_precedes_first_planner_decision(self):
        controller = FakeScanController(500)
        planner = Planner([decision(ADVANCE)])
        context, updates = episode_context()

        result = self.adapter(
            controller, planner,
            max_decisions=1,
            startup_perception=True,
        ).run(context)

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(len(planner.contexts), 1)
        expected_bootstrap_commands = ["scan_front_arc"]
        self.assertEqual(
            controller.commands,
            [*expected_bootstrap_commands, "drive_forward"],
        )
        first = planner.contexts[0]
        self.assertEqual(
            [item["action"] for item in first.history],
            ["SCAN_SURROUNDINGS"],
        )
        self.assertEqual(
            first.history[0]["action_source"], "STARTUP_PERCEPTION",
        )
        self.assertEqual(first.history[0]["scan_view_count"], 1)
        published_actions = [
            update.get("current_action") for update in updates
            if "current_action" in update
        ]
        self.assertNotIn(TURN_LEFT_90, published_actions)
        self.assertNotIn(TURN_RIGHT_90, published_actions)
        self.assertIn("SCAN_SURROUNDINGS", published_actions)
        self.assertIsNone([
            update["scan"] for update in updates if "scan" in update
        ][-1])
        self.assertNotIn(SCAN_FRONT_ARC, first.available_actions)
        self.assertIn(ADVANCE, first.available_actions)
        self.assertIn(TURN_LEFT_90, first.available_actions)
        self.assertIn(TURN_RIGHT_90, first.available_actions)
        self.assertTrue(
            BlastEpisodeRuntimeAdapter._scan_evidence_is_fresh(
                first.history,
            )
        )
        self.assertFalse(
            BlastEpisodeRuntimeAdapter._scan_evidence_is_fresh((
                *first.history,
                {"action": ADVANCE},
            ))
        )
        self.assertEqual(len(first.local_map_evidence["scan_views"]), 1)
        self.assertEqual(
            first.local_map_evidence["robot_pose"]["x_mm"], 0,
        )
        self.assertEqual(
            first.local_map_evidence["robot_pose"]["y_mm"], 0,
        )
        self.assertEqual(
            first.local_map_evidence["robot_pose"]["heading_mdeg"], 0,
        )

    def test_partial_startup_scan_reaches_gemma_with_true_pose_and_map(self):
        class PartialStartupController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                center = copy.deepcopy(self.snapshot_value["observation"])
                result = FakeController.command(
                    self, command, cancel_requested=cancel_requested,
                )
                if command != "scan_front_arc":
                    return result
                final = copy.deepcopy(result["observation"])
                final["motor_angles_deg"].update({
                    "left_drive": -45, "right_drive": 45,
                })
                self.snapshot_value["observation"] = final
                result.update({
                    "observation": final,
                    "observation_settled": True,
                    "receipt": {"turn_count": 1,
                                "coverage_complete": False},
                    "scan": build_blast_partial_scan(
                        center=center,
                        center_settled=True,
                        start_drive_angles={
                            "left_drive": 0, "right_drive": 0,
                        },
                        sweep_samples=(({}, final, True, "SETTLED_RANGE"),),
                        final=final,
                        final_settled=True,
                        final_body_verified=True,
                    ),
                })
                return result

        controller = PartialStartupController(500)
        planner = Planner([decision(ADVANCE)])
        adapter = self.adapter(
            controller, planner, max_decisions=1,
            startup_perception=True,
        )
        map_pose_at_decision = []
        decide = planner.decide

        def inspect_map(context):
            map_pose_at_decision.append(
                adapter.spatial_map_provider.snapshot()["robot_pose"]
            )
            return decide(context)

        planner.decide = inspect_map

        result = adapter.run(episode_context()[0])

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(len(planner.contexts), 1)
        context = planner.contexts[0]
        self.assertEqual(context.history[0]["scan_state"], "partial")
        self.assertEqual(len(context.local_map_evidence["scan_views"]), 1)
        self.assertEqual(
            context.local_map_evidence["robot_pose"]["heading_mdeg"],
            22_050,
        )
        self.assertEqual(
            map_pose_at_decision[0]["heading_mdeg"], 22_050,
        )
        self.assertEqual(
            adapter.spatial_map_provider.snapshot()["robot_pose"][
                "heading_mdeg"
            ],
            22_050,
        )

    def test_unsafe_startup_perception_stops_before_planner(self):
        class UnsafeAfterFirstScan(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    result["observation"]["distance_mm"] = 40
                    self.snapshot_value["observation"] = result[
                        "observation"
                    ]
                return result

        controller = UnsafeAfterFirstScan(500)
        planner = Planner([decision(ADVANCE)])

        result = self.adapter(
            controller, planner, startup_perception=True,
        ).run(episode_context()[0])

        self.assertEqual(
            result.terminal_reason,
            "no_safe_blast_action",
        )
        self.assertEqual(controller.commands, ["scan_front_arc"])
        self.assertEqual(planner.contexts, [])

    def test_no_return_startup_scan_uses_perception_only_permit(self):
        class NoReturnStartupController(FakeScanController):
            def __init__(self):
                super().__init__(2_000)
                self.permits = []

            def issue_no_return_scan_permit(self, **values):
                self.permits.append(values)
                return object()

            def command(
                self, command, *, cancel_requested=None,
                action_permit=None,
            ):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    result["observation"]["distance_mm"] = 500
                    self.snapshot_value["observation"] = result[
                        "observation"
                    ]
                return result

        controller = NoReturnStartupController()
        planner = Planner([decision(ADVANCE)])

        result = self.adapter(
            controller, planner, max_decisions=1,
            startup_perception=True,
        ).run(episode_context()[0])

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(len(controller.permits), 1)
        self.assertTrue(all(
            permit["perception_only"] is True
            and permit["geometry_checked"] is False
            for permit in controller.permits
        ))
        self.assertEqual(controller.commands.count("scan_front_arc"), 1)
        self.assertEqual(len(planner.contexts), 1)
        self.assertEqual(
            len(planner.contexts[0].local_map_evidence["scan_views"]), 1,
        )

    def test_stop_or_deadline_after_bootstrap_result_prevents_next_action(self):
        for control, expected in (
            ("stop", "stopped"),
            ("deadline", "episode_deadline_elapsed"),
        ):
            with self.subTest(control=control):
                controller = FakeScanController(500)
                planner = Planner([decision(ADVANCE)])
                context, _updates = episode_context()
                clock = [1_000]
                if control == "deadline":
                    context.settings.max_episode_ms = 90_000
                adapter = self.adapter(
                    controller, planner,
                    startup_perception=True,
                    monotonic_ms=lambda: clock[0],
                )
                original_record = adapter._record_episode_action_result

                def interrupt(**values):
                    result = original_record(**values)
                    if control == "stop":
                        context.stop_requested.set()
                    else:
                        clock[0] = 91_000
                    return result

                adapter._record_episode_action_result = interrupt

                result = adapter.run(context)

                self.assertEqual(result.terminal_reason, expected)
                self.assertEqual(controller.commands, ["scan_front_arc"])
                self.assertEqual(planner.contexts, [])

    def test_failed_startup_scan_marks_map_localization_lost(self):
        class FailedScan(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                if command != "scan_front_arc":
                    return super().command(
                        command, cancel_requested=cancel_requested,
                    )
                self.commands.append(command)
                motors = self.snapshot_value["observation"][
                    "motor_angles_deg"
                ]
                motors["left_drive"] -= 45
                motors["right_drive"] += 45
                raise BlastControllerError(
                    "scan_sweep_clearance_lost",
                    "injected post-pulse scan failure",
                    motion_started=True,
                )

        controller = FailedScan(500)
        planner = Planner([decision(ADVANCE)])
        adapter = self.adapter(
            controller, planner, startup_perception=True,
        )

        result = adapter.run(episode_context()[0])

        self.assertEqual(
            result.terminal_reason,
            "blast_startup_perception_incomplete",
        )
        self.assertEqual(planner.contexts, [])
        spatial_map = adapter.spatial_map_provider.snapshot()
        self.assertEqual(spatial_map["status"], "unavailable")
        self.assertEqual(
            spatial_map["reason_code"], "localization_lost",
        )
        self.assertIsNone(spatial_map["robot_pose"])
        self.assertFalse(spatial_map["localization"]["valid"])

    def test_speech_configuration_is_explicit_composition_metadata(self):
        factory = lambda **_kwargs: object()
        adapter = self.adapter(
            FakeController(500),
            Planner([]),
            speech_runtime_factory=factory,
            speech_locales=("sv", "en"),
        )

        self.assertIs(adapter.speech_runtime_factory, factory)
        self.assertEqual(adapter.speech_locales, ("sv", "en"))
        with self.assertRaisesRegex(ValueError, "speech locales"):
            self.adapter(
                FakeController(500),
                Planner([]),
                speech_locales=("sv",),
            )

    def test_validated_planner_utterance_is_offered_before_its_action(self):
        controller = FakeController(500)
        planner = Planner([
            decision(
                "ADVANCE",
                assessment="Move one bounded pulse.",
                utterance="Jag rullar. Försök att hänga med.",
            ),
            decision(COMPLETE, assessment="done"),
        ])
        context, updates = episode_context()
        context.settings.speech_enabled = True

        class SpeechRecorder:
            def __init__(self):
                self.started = False
                self.offers = []
                self.cancelled = []
                self.closed = False

            def start(self):
                self.started = True

            def offer(self, **offer):
                self.assert_action_was_verified = list(controller.commands)
                self.offers.append(dict(offer))
                return len(self.offers)

            def cancel_episode(self, episode_id):
                self.cancelled.append(episode_id)

            def close(self, **_options):
                self.closed = True
                return True

        speech = SpeechRecorder()
        adapter = self.adapter(
            controller,
            planner,
            max_decisions=1,
            speech_runtime_factory=lambda **_kwargs: speech,
            speech_locales=("sv", "en"),
        )

        result = adapter.run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertTrue(speech.started)
        self.assertEqual(speech.assert_action_was_verified, [])
        self.assertEqual(len(speech.offers), 1)
        self.assertEqual(
            speech.offers[0]["text"],
            "Jag rullar. Försök att hänga med.",
        )
        self.assertEqual(speech.offers[0]["progress_revision"], 1)
        self.assertEqual(speech.cancelled, ["episode-1"])
        self.assertTrue(speech.closed)
        self.assertNotIn(
            "failed",
            [update.get("speech_status") for update in updates],
        )

    def test_speech_failure_does_not_invalidate_verified_navigation(self):
        controller = FakeController(500)
        planner = Planner([
            decision("ADVANCE", utterance="Det här går säkert bra."),
            decision(COMPLETE, assessment="done"),
        ])
        context, updates = episode_context()
        context.settings.speech_enabled = True

        class FailingSpeech:
            def start(self):
                return None

            def offer(self, **_offer):
                raise RuntimeError("injected speech failure")

            def cancel_episode(self, _episode_id):
                return None

            def close(self, **_options):
                return True

        result = self.adapter(
            controller,
            planner,
            max_decisions=1,
            speech_runtime_factory=lambda **_kwargs: FailingSpeech(),
            speech_locales=("en",),
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(controller.commands, ["drive_forward"])
        self.assertIn(
            "failed",
            [update.get("speech_status") for update in updates],
        )

    def test_long_speech_admission_precedes_fresh_snapshot_and_motion(self):
        for terminal_state in ("started", "failed"):
            with self.subTest(terminal_state=terminal_state):
                clock = {"now": 1_000}

                class GatedSpeech:
                    def __init__(self):
                        self.admission = SpeechAdmission()
                        self.offered = threading.Event()

                    def start(self): return None
                    def offer(self, **_offer): return 1
                    def offer_with_admission(self, **_offer):
                        self.offered.set()
                        return self.admission
                    def cancel_episode(self, _episode_id): return None
                    def close(self, **_options): return True

                controller = FakeController(500)
                speech = GatedSpeech()
                context, _updates = episode_context()
                context.settings.speech_enabled = True
                result, errors = [], []
                adapter = self.adapter(
                    controller,
                    Planner([
                        decision("ADVANCE", utterance="Nu kör jag."),
                        decision(COMPLETE),
                    ]),
                    max_decisions=1,
                    monotonic_ms=lambda: clock["now"],
                    speech_runtime_factory=lambda **_kwargs: speech,
                    speech_locales=("en",),
                )

                worker = threading.Thread(
                    target=lambda: self._capture_run(
                        adapter, context, result, errors,
                    )
                )
                worker.start()
                self.assertTrue(speech.offered.wait(1))
                self.assertEqual(controller.commands, [])
                clock["now"] += 20_000
                controller.snapshot_value[
                    "last_observed_at_monotonic_ms"
                ] = clock["now"] - 1
                speech.admission.resolve(terminal_state)
                worker.join(1)

                self.assertFalse(worker.is_alive())
                self.assertEqual(errors, [])
                self.assertFalse(result[0].completed)
                self.assertEqual(
                    result[0].terminal_reason,
                    "decision_budget_exhausted",
                )
                self.assertEqual(controller.commands, ["drive_forward"])

    def test_spoken_scan_settles_only_inside_the_scan_command(self):
        class NoDuplicateSettleController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                if command == SETTLED_OBSERVATION_COMMAND:
                    raise AssertionError("duplicate pre-scan settle")
                return super().command(
                    command, cancel_requested=cancel_requested,
                )

        class StartedSpeech:
            def start(self): return None
            def offer(self, **_offer): return None
            def offer_with_admission(self, **_offer):
                admission = SpeechAdmission()
                admission.resolve("started")
                return admission
            def cancel_episode(self, _episode_id): return None
            def close(self, **_options): return True

        controller = NoDuplicateSettleController(500)
        context, _updates = episode_context()
        context.settings.speech_enabled = True

        result = self.adapter(
            controller,
            Planner([decision(
                SCAN_FRONT_ARC, utterance="Jag ser mig omkring.",
            )]),
            max_decisions=1,
            speech_runtime_factory=lambda **_kwargs: StartedSpeech(),
            speech_locales=("en",),
        ).run(context)

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(controller.commands, ["scan_front_arc"])

    def test_obstacle_during_speech_preload_blocks_motor(self):
        clock = {"now": 1_000}

        class FreshController(FakeController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command == SETTLED_OBSERVATION_COMMAND:
                    self.snapshot_value[
                        "last_observed_at_monotonic_ms"
                    ] = clock["now"]
                return result

        class GatedSpeech:
            def __init__(self):
                self.admission = SpeechAdmission()
                self.offered = threading.Event()
            def start(self): return None
            def offer(self, **_offer): return 1
            def offer_with_admission(self, **_offer):
                self.offered.set()
                return self.admission
            def cancel_episode(self, _episode_id): return None
            def close(self, **_options): return True

        controller, speech = FreshController(500), GatedSpeech()
        context, _updates = episode_context()
        context.settings.speech_enabled = True
        result, errors = [], []
        adapter = self.adapter(
            controller,
            Planner([decision("ADVANCE", utterance="Nu kör jag.")]),
            monotonic_ms=lambda: clock["now"],
            speech_runtime_factory=lambda **_kwargs: speech,
            speech_locales=("en",),
        )
        worker = threading.Thread(target=lambda: self._capture_run(
            adapter, context, result, errors,
        ))
        worker.start()
        self.assertTrue(speech.offered.wait(1))
        controller.snapshot_value["observation"]["distance_mm"] = 40
        clock["now"] += 20_000
        controller.snapshot_value["last_observed_at_monotonic_ms"] = (
            clock["now"] - 1
        )
        speech.admission.resolve("started")
        worker.join(1)

        self.assertFalse(worker.is_alive())
        self.assertEqual(result, [])
        self.assertEqual(errors[0].code, "blast_action_start_unverified")
        self.assertEqual(controller.commands, [])

    def test_post_speech_uses_fresh_monitor_observation(self):
        class NoDuplicateSettleController(FakeController):
            def command(self, command, *, cancel_requested=None):
                if command == SETTLED_OBSERVATION_COMMAND:
                    raise AssertionError("duplicate post-speech settle")
                return super().command(
                    command, cancel_requested=cancel_requested,
                )

        class StartedSpeech:
            def start(self): return None
            def offer(self, **_offer): return None
            def offer_with_admission(self, **_offer):
                admission = SpeechAdmission()
                admission.resolve("started")
                return admission
            def cancel_episode(self, _episode_id): return None
            def close(self, **_options): return True

        controller = NoDuplicateSettleController(500)
        context, _updates = episode_context()
        context.settings.speech_enabled = True

        result = self.adapter(
            controller,
            Planner([
                decision("ADVANCE", utterance="Nu kör jag."),
                decision(COMPLETE),
            ]),
            max_decisions=1,
            speech_runtime_factory=lambda **_kwargs: StartedSpeech(),
            speech_locales=("en",),
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(controller.commands, ["drive_forward"])

    def test_stop_or_deadline_during_speech_admission_never_moves(self):
        for interruption in ("stop", "deadline"):
            with self.subTest(interruption=interruption):
                clock = {"now": 1_000}

                class GatedSpeech:
                    def __init__(self):
                        self.admission = SpeechAdmission()
                        self.offered = threading.Event()
                    def start(self): return None
                    def offer(self, **_offer): return 1
                    def offer_with_admission(self, **_offer):
                        self.offered.set()
                        return self.admission
                    def cancel_episode(self, _episode_id): return None
                    def close(self, **_options): return True

                controller, speech = FakeController(500), GatedSpeech()
                context, _updates = episode_context()
                context.settings.speech_enabled = True
                if interruption == "deadline":
                    context.settings.max_episode_ms = 10_000
                results, errors = [], []
                adapter = self.adapter(
                    controller,
                    Planner([
                        decision("ADVANCE", utterance="Nu kör jag."),
                    ]),
                    monotonic_ms=lambda: clock["now"],
                    speech_runtime_factory=lambda **_kwargs: speech,
                    speech_locales=("en",),
                )
                worker = threading.Thread(target=lambda: self._capture_run(
                    adapter, context, results, errors,
                ))
                worker.start()
                self.assertTrue(speech.offered.wait(1))
                if interruption == "stop":
                    context.stop_requested.set()
                else:
                    clock["now"] = 11_000
                worker.join(1)

                self.assertFalse(worker.is_alive())
                self.assertEqual(errors, [])
                self.assertFalse(results[0].completed)
                self.assertEqual(controller.commands, [])

    def test_muted_episode_does_not_construct_speech_runtime(self):
        context, _updates = episode_context()
        context.settings.speech_enabled = False

        def forbidden_factory(**_kwargs):
            raise AssertionError("muted episode constructed speech")

        result = self.adapter(
            FakeController(500),
            Planner([decision(ADVANCE, assessment="move once")]),
            max_decisions=1,
            speech_runtime_factory=forbidden_factory,
            speech_locales=("en",),
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")

    def test_unreaped_speech_disables_only_speech_not_later_navigation(self):
        for start_raises in (False, True):
            with self.subTest(start_raises=start_raises):
                calls = []

                class UnreapedSpeech:
                    def start(self):
                        if start_raises:
                            raise RuntimeError("injected speech start failure")

                    def offer(self, **_offer):
                        return 1

                    def cancel_episode(self, _episode_id):
                        return None

                    def close(self, **_options):
                        return False

                def factory(**_kwargs):
                    calls.append(True)
                    return UnreapedSpeech()

                adapter = self.adapter(
                    FakeController(500),
                    Planner([
                        decision(ADVANCE, assessment="first"),
                        decision(ADVANCE, assessment="second"),
                    ]),
                    max_decisions=1,
                    speech_runtime_factory=factory,
                    speech_locales=("en",),
                )
                first, _updates = episode_context()
                first.settings.speech_enabled = True
                second, _updates = episode_context()
                second.episode_id = "episode-2"
                second.settings.speech_enabled = True

                first_result = adapter.run(first)
                second_result = adapter.run(second)

                self.assertFalse(first_result.completed)
                self.assertFalse(second_result.completed)
                self.assertEqual(
                    first_result.terminal_reason,
                    "decision_budget_exhausted",
                )
                self.assertEqual(
                    second_result.terminal_reason,
                    "decision_budget_exhausted",
                )
                self.assertEqual(calls, [True])

    def test_replans_after_each_bounded_action_without_early_completion(self):
        controller = FakeController(500)
        planner = Planner([
            decision(
                "ADVANCE",
                plan=("ADVANCE", TURN_LEFT_90, COMPLETE),
                assessment="Move one bounded pulse.",
            ),
            decision(
                ADVANCE,
                assessment="Continue toward the directional goal.",
            ),
        ])
        context, updates = episode_context()

        result = self.adapter(
            controller, planner, max_decisions=2,
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(controller.commands, ["drive_forward"] * 2)
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(
            planner.contexts[1].history[0]["action"],
            "ADVANCE",
        )
        self.assertEqual(
            planner.contexts[1].history[0]["plan"],
            ["ADVANCE", TURN_LEFT_90, COMPLETE],
        )
        self.assertIsInstance(
            planner.contexts[1].history[0]["plan"],
            list,
        )
        self.assertIsNot(
            planner.contexts[1].history[0]["plan"],
            updates[0]["plan"],
        )
        self.assertTrue(
            planner.contexts[1].history[0]["observation_settled"]
        )
        self.assertEqual(
            planner.contexts[1].observation["sensors"]["distance_mm"],
            455,
        )
        self.assertEqual(
            updates[0]["plan"],
            ["ADVANCE", TURN_LEFT_90, COMPLETE],
        )
        self.assertEqual(
            planner.contexts[0].observation["navigation_reference"],
            {
                "episode_start_heading_deg": 0.0,
                "current_heading_deg": 0.0,
                "heading_error_deg": 0.0,
            },
        )
        self.assertEqual(
            planner.contexts[1].history[0]["motion"][
                "left_encoder_delta_degrees"
            ],
            90,
        )
        self.assertEqual(
            planner.contexts[1].observation["odometry"]["x_mm"],
            45,
        )

    def test_live_dense_scan_cannot_complete_with_328_mm_remaining(self):
        class LiveContradictionController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "drive_forward":
                    angles = result["observation"]["motor_angles_deg"]
                    angles["left_drive"] += 3
                    angles["right_drive"] += 1
                    result["observation"]["imu"]["heading_deg"] = (
                        0.49 * self.commands.count("drive_forward")
                    )
                    self.snapshot_value["observation"] = result["observation"]
                if command != "scan_front_arc":
                    return result
                scan = result["scan"]
                values = (
                    ("center", 258, 0.0, True),
                    ("left_1", 281, -12.2687, True),
                    ("left_2", 304, -23.8833, True),
                    ("left_3", 362, -35.0061, True),
                    ("left_4", 411, -46.4686, False),
                    ("right_1", 246, 10.8163, True),
                    ("right_2", 248, 22.6265, True),
                    ("right_3", 248, 33.6900, True),
                    ("right_4", 2_000, 44.7418, True),
                )
                scan["angular_rays"] = [set_scan_ray_bearing({
                    "side": side,
                    "distance_mm": distance,
                    "range_state": (
                        RANGE_STATE_NO_VALID_DISTANCE
                        if distance == 2_000
                        else RANGE_STATE_MEASURED
                    ),
                    "body_motor_angle_deg": 158,
                    "observation_settled": settled,
                    "evidence_use": (
                        "SETTLED_RANGE"
                        if settled
                        else "SWEEP_CONTINUATION_ONLY"
                    ),
                }, heading) for side, distance, heading, settled in values]
                scan["rays"] = [
                    dict(scan["angular_rays"][dense], side=side)
                    for dense, side in (
                        (0, "center"),
                        (2, "left_near"),
                        (4, "left_far"),
                        (6, "right_near"),
                        (8, "right_far"),
                    )
                ]
                scan["all_observations_settled"] = False
                return result

        assessment = (
            "Vänta nu... jag har redan rullat framåt 92 mm och skannat "
            "vägen. Vägen är helt fri! Men målet var 420 mm... Jag har "
            "inte nått det än. Jag måste fortsätta gasa!"
        )
        controller = LiveContradictionController(350)
        planner = Planner([
            decision("ADVANCE"),
            decision("ADVANCE"),
            decision(SCAN_FRONT_ARC),
            decision(COMPLETE, assessment=assessment),
        ])
        adapter = self.adapter(
            controller,
            planner,
            enforce_directional_completion=True,
            minimum_forward_progress_mm=420,
        )

        with self.assertRaises(BlastEpisodeError) as rejected:
            adapter.run(episode_context()[0])

        self.assertEqual(
            rejected.exception.code,
            "blast_planner_action_invalid",
        )
        self.assertEqual(
            controller.commands,
            ["drive_forward", "drive_forward", "scan_front_arc"],
        )
        final_context = planner.contexts[-1]
        self.assertEqual(final_context.observation["odometry"]["x_mm"], 92)
        self.assertEqual(
            final_context.observation["odometry"]["heading_mdeg"],
            -980,
        )
        self.assertFalse(final_context.completion_allowed)
        self.assertEqual(
            final_context.available_actions,
            (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90),
        )
        final_goal = adapter.spatial_map_provider.snapshot()[
            "navigation_trace"
        ]["final_goal"]
        self.assertEqual(final_goal["current_forward_progress_mm"], 92)
        self.assertEqual(final_goal["minimum_forward_progress_mm"], 420)
        self.assertEqual(final_goal["remaining_forward_progress_mm"], 328)

    def test_directional_completion_requires_progress_heading_and_localization(
        self,
    ):
        mission = DirectionalMission.begin(
            episode_id="episode-1",
            minimum_forward_progress_mm=420,
            pose=PhysicalPose(),
        )

        self.assertFalse(blast_directional_completion_allowed(
            mission=mission,
            pose=PhysicalPose(x_mm=419),
            localization_valid=True,
            scan_fresh=True,
        ))
        self.assertFalse(blast_directional_completion_allowed(
            mission=mission,
            pose=PhysicalPose(x_mm=420, heading_mdeg=5_001),
            localization_valid=True,
            scan_fresh=True,
        ))
        self.assertFalse(blast_directional_completion_allowed(
            mission=mission,
            pose=PhysicalPose(x_mm=420),
            localization_valid=False,
            scan_fresh=True,
        ))
        self.assertFalse(blast_directional_completion_allowed(
            mission=mission,
            pose=PhysicalPose(x_mm=420),
            localization_valid=True,
            scan_fresh=False,
        ))
        self.assertTrue(blast_directional_completion_allowed(
            mission=mission,
            pose=PhysicalPose(x_mm=420),
            localization_valid=True,
            scan_fresh=True,
        ))

    def test_directional_episode_completes_after_verified_minimum_progress(self):
        controller = FakeController(1_000)
        planner = Planner(
            [decision("ADVANCE") for _index in range(10)]
            + [decision(COMPLETE, assessment="Verified goal reached.")]
        )

        result = self.adapter(
            controller,
            planner,
            enforce_directional_completion=True,
            minimum_forward_progress_mm=420,
        ).run(episode_context()[0])

        self.assertTrue(result.completed)
        self.assertEqual(result.terminal_reason, "completed")
        self.assertEqual(controller.commands, ["drive_forward"] * 10)
        self.assertFalse(planner.contexts[-2].completion_allowed)
        self.assertTrue(planner.contexts[-1].completion_allowed)
        self.assertEqual(
            planner.contexts[-1].observation["odometry"]["x_mm"],
            450,
        )

    def test_directional_episode_can_complete_with_no_safe_motion_at_goal(self):
        class BlockedAtGoalController(FakeController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if (
                    command == "drive_forward"
                    and self.commands.count("drive_forward") == 10
                ):
                    result["observation"]["distance_mm"] = 40
                    self.snapshot_value["observation"] = result["observation"]
                return result

        controller = BlockedAtGoalController(1_000)
        planner = Planner(
            [decision("ADVANCE") for _index in range(10)]
            + [decision(COMPLETE, assessment="Verified blocked goal reached.")]
        )

        result = self.adapter(
            controller,
            planner,
            enforce_directional_completion=True,
            minimum_forward_progress_mm=420,
        ).run(episode_context()[0])

        self.assertTrue(result.completed)
        self.assertEqual(result.terminal_reason, "completed")
        self.assertEqual(controller.commands, ["drive_forward"] * 10)
        terminal_context = planner.contexts[-1]
        self.assertEqual(
            terminal_context.observation["odometry"]["x_mm"],
            450,
        )
        self.assertEqual(
            terminal_context.observation["sensors"]["distance_mm"],
            40,
        )
        self.assertEqual(terminal_context.available_actions, ())
        self.assertTrue(terminal_context.completion_allowed)

    def test_existing_map_shows_goal_and_encoder_odometry_without_authority(self):
        controller = FakeController(500)
        planner = Planner([decision("ADVANCE"), decision(COMPLETE)])
        adapter = self.adapter(controller, planner, max_decisions=1)

        result = adapter.run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        spatial_map = adapter.spatial_map_provider.snapshot()
        self.assertEqual(spatial_map["schema"], "robot-spatial-map/v1")
        self.assertEqual(spatial_map["status"], "pose_only")
        self.assertEqual(
            [pose["x_mm"] for pose in spatial_map["pose_history"]],
            [0, 45],
        )
        trace = spatial_map["navigation_trace"]
        self.assertEqual(trace["final_goal"]["target_x_mm"], 420)
        self.assertEqual(
            trace["final_goal"]["current_forward_progress_mm"], 45
        )
        self.assertEqual(
            trace["final_goal"]["remaining_forward_progress_mm"], 375
        )
        self.assertIsNone(trace["planned_leg"])
        self.assertEqual(trace["imu_heading"]["heading_mdeg"], 0)
        self.assertEqual(spatial_map["cells"], [])
        self.assertEqual(spatial_map["object_hypotheses"], [])

    def test_map_sink_failure_does_not_change_actions_or_outcome(self):
        class BrokenMap:
            def begin_episode(self, **_values):
                raise RuntimeError("map unavailable")

            offer_pose = begin_episode
            offer_trace = begin_episode

            def snapshot(self):
                raise RuntimeError("map unavailable")

            def close(self, **_values):
                return True

        controller = FakeController(500)
        planner = Planner([decision("ADVANCE"), decision(COMPLETE)])

        result = self.adapter(
            controller,
            planner,
            max_decisions=1,
            spatial_map_bridge=BrokenMap(),
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(controller.commands, ["drive_forward"])
        self.assertEqual(len(planner.contexts), 1)

    def test_scripted_semantic_actions_use_four_pulse_turn_and_carry_pose(self):
        controller = FakeScanController(1_000)
        planner = Planner([
            decision("ADVANCE"),
            decision("ADVANCE"),
            decision(TURN_LEFT_90),
            decision("ADVANCE"),
            decision("ADVANCE"),
        ])
        context, updates = episode_context()

        result = self.adapter(
            controller, planner, max_decisions=5,
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands,
            ["drive_forward"] * 2
            + ["turn_left"] * 4
            + ["drive_forward"] * 2,
        )
        final_pose = planner.contexts[-1].observation["odometry"]
        self.assertLessEqual(abs(final_pose["x_mm"] - 90), 10)
        self.assertLessEqual(abs(final_pose["y_mm"] - 45), 10)
        self.assertEqual(final_pose["verified_motion_count"], 4)
        self.assertNotIn("pose", updates[-2])
        runtime_update = None
        for update in updates:
            runtime_update = RobotRuntimeUpdate.from_mapping(
                update,
                runtime_update,
            )

    def test_scan_is_one_agent_action_with_stable_heading_reference(self):
        class ScanController(FakeScanController):
            def __init__(self):
                super().__init__(300)
                self.snapshot_value["observation"]["imu"][
                    "heading_deg"
                ] = 179

            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command != "scan_front_arc":
                    return result
                observation = {
                    **result["observation"],
                    "imu": {
                        **result["observation"]["imu"],
                        "heading_deg": -179,
                    },
                }
                result["observation"] = observation
                result["scan"] = anchor_scan_result(
                    scan_result(center_distance_mm=300.0), observation,
                )
                self.snapshot_value = {
                    **self.snapshot_value,
                    "observation": observation,
                }
                return result

        controller = ScanController()
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision(TURN_LEFT_90, assessment="Turn from the scan."),
        ])
        context, updates = episode_context()

        result = self.adapter(
            controller, planner, max_decisions=2,
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands, ["scan_front_arc"] + ["turn_left"] * 4,
        )
        scan = planner.contexts[1].history[0]["scan"]
        self.assertTrue(scan["restoration_verified"])
        self.assertEqual(
            [ray["range_state"] for ray in scan["rays"]],
            [
                RANGE_STATE_MEASURED,
                RANGE_STATE_MEASURED,
                RANGE_STATE_NO_VALID_DISTANCE,
                RANGE_STATE_MEASURED,
                RANGE_STATE_NO_VALID_DISTANCE,
            ],
        )
        runtime_scan = [
            update["scan"] for update in updates if "scan" in update
        ][0]
        self.assertNotIn("planar_projection", scan)
        projection = runtime_scan["planar_projection"]
        self.assertEqual(projection["quality"], "PROVISIONAL_YAW_ONLY")
        self.assertFalse(projection["vertical_pitch_compensated"])
        self.assertEqual(
            [point["side"] for point in projection["points"]],
            ["center", "left_near", "right_near"],
        )
        self.assertIn(
            SCAN_FRONT_ARC,
            planner.contexts[0].available_actions,
        )
        self.assertNotIn(
            SCAN_FRONT_ARC,
            planner.contexts[1].available_actions,
        )
        self.assertEqual(
            planner.contexts[1].available_actions,
            (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90),
        )
        side_scan = planner.contexts[1].robot_relative_side_scan
        self.assertIsNone(
            side_scan["rays"]["left"][1]["distance_mm"]
        )
        self.assertEqual(
            side_scan["rays"]["left"][1]["range_state"],
            RANGE_STATE_NO_VALID_DISTANCE,
        )
        self.assertEqual(
            planner.contexts[1].observation["navigation_reference"],
            {
                "episode_start_heading_deg": 179.0,
                "current_heading_deg": -179.0,
                "heading_error_deg": 2.0,
            },
        )
        runtime_update = None
        for update in updates:
            runtime_update = RobotRuntimeUpdate.from_mapping(
                update,
                runtime_update,
            )

    def test_live_scan_vector_is_summarized_by_physical_side_for_gemma(self):
        class LiveVectorScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    for ray, distance in zip(
                        result["scan"]["rays"],
                        (209, 246, 347, 202, 1_002),
                    ):
                        ray["distance_mm"] = distance
                        ray["range_state"] = RANGE_STATE_MEASURED
                return result

        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(ADVANCE, assessment="Continue from scan evidence."),
        ])

        result = self.adapter(
            LiveVectorScanController(), planner, max_decisions=2,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertIsNone(planner.contexts[0].robot_relative_side_scan)
        self.assertEqual(
            planner.contexts[1].available_actions,
            (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90),
        )
        side_scan = planner.contexts[1].robot_relative_side_scan
        self.assertEqual(
            side_scan,
            {
                "schema": "blast-robot-relative-side-scan/v2",
                "frame": "ROBOT_RELATIVE_AT_SCAN_START",
                "physical_side_labels_authoritative": True,
                "rays": {
                    "left": [{
                        "range_state": RANGE_STATE_MEASURED,
                        "distance_mm": 246,
                        "absolute_bearing_deg": 22.05,
                    }, {
                        "range_state": RANGE_STATE_MEASURED,
                        "distance_mm": 347,
                        "absolute_bearing_deg": 45.08,
                    }],
                    "right": [{
                        "range_state": RANGE_STATE_MEASURED,
                        "distance_mm": 202,
                        "absolute_bearing_deg": 24.01,
                    }, {
                        "range_state": RANGE_STATE_MEASURED,
                        "distance_mm": 1_002,
                        "absolute_bearing_deg": 47.04,
                    }],
                },
            },
        )
        for side_rays in side_scan["rays"].values():
            self.assertEqual(len(side_rays), 2)
            for ray in side_rays:
                self.assertEqual(
                    set(ray),
                    {
                        "range_state",
                        "distance_mm",
                        "absolute_bearing_deg",
                    },
                )

    def test_dense_live_scan_gives_gemma_four_rays_per_physical_side(self):
        class DenseScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command != "scan_front_arc":
                    return result
                scan = result["scan"]
                values = (
                    ("center", 247, 0.0),
                    ("left_1", 307, -11.0),
                    ("left_2", 2_000, -22.0),
                    ("left_3", 210, -33.0),
                    ("left_4", 156, -44.0),
                    ("right_1", 226, 11.0),
                    ("right_2", 400, 22.0),
                    ("right_3", 700, 33.0),
                    ("right_4", 1_000, 44.0),
                )
                scan["angular_rays"] = [set_scan_ray_bearing({
                    "side": side,
                    "distance_mm": distance,
                    "range_state": (
                        RANGE_STATE_NO_VALID_DISTANCE
                        if distance == 2_000
                        else RANGE_STATE_MEASURED
                    ),
                    "body_motor_angle_deg": 158,
                    "observation_settled": True,
                }, heading) for side, distance, heading in values]
                scan["rays"] = [
                    dict(scan["angular_rays"][index], side=side)
                    for index, side in (
                        (0, "center"),
                        (2, "left_near"),
                        (4, "left_far"),
                        (6, "right_near"),
                        (8, "right_far"),
                    )
                ]
                return result

        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(ADVANCE, assessment="Continue from dense evidence."),
        ])

        context, updates = episode_context()
        result = self.adapter(
            DenseScanController(), planner, max_decisions=2,
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        side_rays = planner.contexts[1].robot_relative_side_scan["rays"]
        self.assertEqual(
            [ray["distance_mm"] for ray in side_rays["left"]],
            [307, None, 210, 156],
        )
        self.assertEqual(
            [ray["distance_mm"] for ray in side_rays["right"]],
            [226, 400, 700, 1_000],
        )
        self.assertEqual(
            [ray["absolute_bearing_deg"] for ray in side_rays["left"]],
            [10.78, 22.05, 32.83, 44.1],
        )
        planner_dense = planner.contexts[1].history[0]["scan"][
            "angular_rays"
        ]
        self.assertIsNone(planner_dense[2]["distance_mm"])
        self.assertEqual(
            planner_dense[2]["range_state"],
            RANGE_STATE_NO_VALID_DISTANCE,
        )
        runtime_dense = next(
            update["scan"]["angular_rays"]
            for update in updates if "scan" in update
        )
        self.assertEqual(runtime_dense[2]["distance_mm"], 2_000)
        self.assertEqual(
            runtime_dense[2]["range_state"],
            RANGE_STATE_NO_VALID_DISTANCE,
        )
        for ray in (*side_rays["left"], *side_rays["right"]):
            self.assertEqual(
                set(ray),
                {
                    "range_state",
                    "distance_mm",
                    "absolute_bearing_deg",
                },
            )

    def test_no_valid_live_scan_distance_is_withheld_from_planner_history(self):
        class LiveNoValidScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    values = (
                        (259, RANGE_STATE_MEASURED, 0.0),
                        (300, RANGE_STATE_MEASURED, -22.9793309),
                        (2_000, RANGE_STATE_NO_VALID_DISTANCE, -45.8837289),
                        (251, RANGE_STATE_MEASURED, 24.8248911),
                        (1_029, RANGE_STATE_MEASURED, 49.0489911),
                    )
                    for ray, (distance, state, heading) in zip(
                        result["scan"]["rays"], values,
                    ):
                        ray.update({
                            "distance_mm": distance,
                            "range_state": state,
                        })
                        set_scan_ray_bearing(ray, heading)
                return result

        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(ADVANCE, assessment="Continue from live evidence."),
        ])
        context, updates = episode_context()

        result = self.adapter(
            LiveNoValidScanController(), planner, max_decisions=2,
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        planner_rays = planner.contexts[1].history[0]["scan"]["rays"]
        self.assertEqual(
            [ray["distance_mm"] for ray in planner_rays],
            [259, 300, None, 251, 1_029],
        )
        self.assertEqual(
            planner_rays[2]["range_state"],
            RANGE_STATE_NO_VALID_DISTANCE,
        )
        side_scan = planner.contexts[1].robot_relative_side_scan["rays"]
        self.assertIsNone(side_scan["left"][1]["distance_mm"])
        self.assertEqual(
            side_scan["left"][1]["range_state"],
            RANGE_STATE_NO_VALID_DISTANCE,
        )
        self.assertEqual(side_scan["right"][1]["distance_mm"], 1_029)
        runtime_scan = [
            update["scan"] for update in updates if "scan" in update
        ][0]
        self.assertEqual(runtime_scan["rays"][2]["distance_mm"], 2_000)
        self.assertEqual(
            runtime_scan["rays"][2]["range_state"],
            RANGE_STATE_NO_VALID_DISTANCE,
        )

    def test_legacy_scan_schema_is_rejected_before_replanning(self):
        class LegacyScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    result["scan"]["schema"] = (
                        "blast-scan-front-arc/v2"
                    )
                return result

        planner = Planner([decision(SCAN_FRONT_ARC)])
        adapter = self.adapter(LegacyScanController(), planner)

        with self.assertRaises(BlastEpisodeError) as rejected:
            adapter.run(episode_context()[0])

        self.assertEqual(rejected.exception.code, "blast_scan_result_invalid")
        self.assertEqual(len(planner.contexts), 1)
        spatial_map = adapter.spatial_map_provider.snapshot()
        self.assertEqual(spatial_map["reason_code"], "localization_lost")
        self.assertIsNone(spatial_map["robot_pose"])

    def test_malformed_canonical_ray_with_dense_scan_fails_closed(self):
        class MalformedDenseScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    scan = result["scan"]
                    canonical = scan["rays"]
                    scan["angular_rays"] = [
                        dict(canonical[0], side="center"),
                        dict(canonical[1], side="left_1",
                             relative_heading_deg=-11.0),
                        dict(canonical[1], side="left_2"),
                        dict(canonical[2], side="left_3",
                             relative_heading_deg=-33.0),
                        dict(canonical[2], side="left_4"),
                        dict(canonical[3], side="right_1",
                             relative_heading_deg=11.0),
                        dict(canonical[3], side="right_2"),
                        dict(canonical[4], side="right_3",
                             relative_heading_deg=35.0),
                        dict(canonical[4], side="right_4"),
                    ]
                    scan["rays"][1] = 7
                return result

        planner = Planner([decision(SCAN_FRONT_ARC)])

        with self.assertRaises(BlastEpisodeError) as rejected:
            self.adapter(MalformedDenseScanController(), planner).run(
                episode_context()[0]
            )

        self.assertEqual(rejected.exception.code, "blast_scan_result_invalid")
        self.assertEqual(len(planner.contexts), 1)

    def test_inconsistent_scan_range_state_is_rejected(self):
        class InconsistentScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    result["scan"]["rays"][2]["range_state"] = (
                        RANGE_STATE_MEASURED
                    )
                return result

        planner = Planner([decision(SCAN_FRONT_ARC)])

        with self.assertRaises(BlastEpisodeError) as rejected:
            self.adapter(InconsistentScanController(), planner).run(
                episode_context()[0]
            )

        self.assertEqual(rejected.exception.code, "blast_scan_result_invalid")
        self.assertEqual(len(planner.contexts), 1)

    def test_scan_with_wrong_ray_order_is_rejected(self):
        class MisorderedScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    rays = result["scan"]["rays"]
                    rays[1], rays[2] = rays[2], rays[1]
                return result

        planner = Planner([decision(SCAN_FRONT_ARC)])

        with self.assertRaises(BlastEpisodeError) as rejected:
            controller = MisorderedScanController()
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(rejected.exception.code, "blast_scan_result_invalid")
        self.assertEqual(controller.commands, ["scan_front_arc"])
        self.assertEqual(len(planner.contexts), 1)

    def test_unprojectable_heading_keeps_raw_scan_for_replanning(self):
        class InvalidHeadingController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    set_scan_ray_bearing(
                        result["scan"]["rays"][1], 22.0,
                    )
                return result

        controller = InvalidHeadingController()
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(ADVANCE, assessment="Continue with raw scan evidence."),
        ])
        context, updates = episode_context()

        result = self.adapter(
            controller, planner, max_decisions=2,
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands, ["scan_front_arc", "drive_forward"],
        )
        self.assertEqual(len(planner.contexts), 2)
        self.assertIn("ADVANCE", planner.contexts[1].available_actions)
        self.assertIn("scan", planner.contexts[1].history[0])
        runtime_scan = [
            update["scan"] for update in updates if "scan" in update
        ][0]
        self.assertNotIn("planar_projection", runtime_scan)

    def test_sweep_only_range_is_withheld_from_planner_history(self):
        class UnresolvedFarController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command != "scan_front_arc":
                    return result
                if command == "scan_front_arc":
                    result["scan"]["all_observations_settled"] = False
                    result["scan"]["rays"][1].update({
                        "distance_mm": 1_489.0,
                        "range_state": RANGE_STATE_MEASURED,
                        "observation_settled": False,
                        "evidence_use": "SWEEP_CONTINUATION_ONLY",
                    })
                return result

        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(ADVANCE, assessment="Continue after observation."),
        ])
        context, updates = episode_context()

        result = self.adapter(
            UnresolvedFarController(), planner, max_decisions=2,
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        planner_ray = planner.contexts[1].history[0]["scan"]["rays"][1]
        self.assertIsNone(planner_ray["distance_mm"])
        self.assertEqual(
            planner_ray["range_state"], "UNRESOLVED_SWEEP_ONLY",
        )
        summarized_ray = planner.contexts[1].robot_relative_side_scan[
            "rays"
        ]["left"][0]
        self.assertEqual(
            summarized_ray["range_state"], "UNRESOLVED_SWEEP_ONLY"
        )
        self.assertIsNone(summarized_ray["distance_mm"])
        runtime_scan = [
            update["scan"] for update in updates if "scan" in update
        ][0]
        self.assertEqual(runtime_scan["rays"][1]["distance_mm"], 1_489.0)
        self.assertEqual(
            runtime_scan["rays"][1]["evidence_use"],
            "SWEEP_CONTINUATION_ONLY",
        )

    def test_projected_scan_keeps_safe_agentic_motion_choices(self):
        controller = FakeScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision("ADVANCE"),
        ])

        result = self.adapter(
            controller, planner, max_decisions=2,
        ).run(episode_context()[0])

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands, ["scan_front_arc", "drive_forward"],
        )
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(
            planner.contexts[1].available_actions,
            (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90),
        )
        self.assertFalse(planner.contexts[1].completion_allowed)

    def test_planner_regains_control_after_scan_guided_turn(self):
        class LiveCloseAfterLeftTurnController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if (
                    command == "turn_left"
                    and self.commands.count("turn_left") == 4
                ):
                    result["observation"]["distance_mm"] = 70
                    self.snapshot_value["observation"] = result[
                        "observation"
                    ]
                elif (
                    command == "turn_right"
                    and self.commands.count("turn_right") == 4
                ):
                    result["observation"]["distance_mm"] = 500
                    self.snapshot_value["observation"] = result[
                        "observation"
                    ]
                return result

        controller = LiveCloseAfterLeftTurnController(248)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
            decision(TURN_RIGHT_90),
        ])

        result = self.adapter(
            controller, planner, max_decisions=3,
            enforce_directional_completion=True,
        ).run(episode_context()[0])

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(len(planner.contexts), 3)
        self.assertEqual(
            planner.contexts[0].local_map_evidence["scan_views"], [],
        )
        self.assertEqual(
            len(planner.contexts[1].local_map_evidence["scan_views"]), 1,
        )
        self.assertEqual(
            len(planner.contexts[2].local_map_evidence["scan_views"]), 1,
        )
        self.assertNotEqual(
            planner.contexts[2].local_map_evidence[
                "robot_pose"
            ]["heading_mdeg"],
            0,
        )
        self.assertEqual(
            planner.contexts[2].local_map_evidence["unobserved_space"],
            "UNKNOWN_NOT_FREE",
        )
        self.assertEqual(
            planner.contexts[2].available_actions,
            (TURN_LEFT_90, TURN_RIGHT_90, SCAN_FRONT_ARC),
        )
        self.assertNotIn(
            "side_search_progress",
            planner.contexts[2].observation.get("navigation_intent", {}),
        )
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"]
            + ["turn_left"] * 4
            + ["turn_right"] * 4,
        )

    def test_planner_scan_executes_after_turn_at_no_valid_distance(self):
        class NvdAfterTurnController(FakeScanController):
            def __init__(self):
                super().__init__(248)
                self.clock = [1_000]
                self.scan_permits = []
                self.permit_requests = []

            def issue_no_return_scan_permit(self, **values):
                self.permit_requests.append(values)
                return object()

            def command(
                self, command, *, cancel_requested=None,
                action_permit=None,
            ):
                if command == "scan_front_arc":
                    self.scan_permits.append(action_permit)
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if (
                    command == "turn_left"
                    and self.commands.count("turn_left") == 4
                ):
                    result["observation"]["distance_mm"] = 2_000
                    self.snapshot_value["observation"] = result[
                        "observation"
                    ]
                if command == SETTLED_OBSERVATION_COMMAND:
                    self.clock[0] += 1
                    result["observation"]["distance_mm"] = 2_000
                    self.snapshot_value["observation"] = result[
                        "observation"
                    ]
                    self.snapshot_value[
                        "last_observed_at_monotonic_ms"
                    ] = self.clock[0]
                if command == "scan_front_arc":
                    result["scan"] = anchor_scan_result(
                        dense_scan_result(
                            (248, 250, 260, 270, 280,
                             500, 600, 700, 800),
                            (0, -11, -22, -33, -44,
                             11, 22, 33, 44),
                        ),
                        result["observation"],
                    )
                return result

        class ImmediateSpeech:
            def start(self): return None
            def offer(self, **_offer): return None
            def offer_with_admission(self, **_offer):
                admission = SpeechAdmission()
                admission.resolve("started")
                return admission
            def cancel_episode(self, _episode_id): return None
            def close(self, **_options): return True

        controller = NvdAfterTurnController()
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
            decision(SCAN_FRONT_ARC, utterance="Jag skannar igen."),
        ])
        context, _updates = episode_context()
        context.settings.speech_enabled = True

        result = self.adapter(
            controller, planner, max_decisions=3,
            enforce_directional_completion=True,
            monotonic_ms=lambda: controller.clock[0],
            speech_runtime_factory=lambda **_kwargs: ImmediateSpeech(),
            speech_locales=("en",),
        ).run(context)

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            planner.contexts[2].available_actions, (SCAN_FRONT_ARC,),
        )
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"] + ["turn_left"] * 4
            + ["scan_front_arc"],
        )
        self.assertIsNotNone(controller.scan_permits[-1])
        self.assertTrue(controller.permit_requests[-1]["allow_no_return"])
        self.assertTrue(controller.permit_requests[-1]["geometry_checked"])

    def test_scan_with_off_reference_body_pose_is_rejected(self):
        class OffPoseScanController(FakeScanController):
            mismatch_location = "ray"

            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    if self.mismatch_location == "ray":
                        result["scan"]["rays"][2][
                            "body_motor_angle_deg"
                        ] = 160
                    else:
                        result["observation"]["motor_angles_deg"][
                            "body"
                        ] = 160
                return result

        for mismatch_location in ("ray", "final"):
            with self.subTest(mismatch_location=mismatch_location):
                controller = OffPoseScanController()
                controller.mismatch_location = mismatch_location
                planner = Planner([decision(SCAN_FRONT_ARC)])

                with self.assertRaises(BlastEpisodeError) as rejected:
                    self.adapter(controller, planner).run(
                        episode_context()[0]
                    )

                self.assertEqual(
                    rejected.exception.code,
                    "blast_scan_sensor_pose_unverified",
                )
                self.assertEqual(controller.commands, ["scan_front_arc"])
                self.assertEqual(len(planner.contexts), 1)

    def test_restored_scan_residue_reanchors_before_semantic_turn(self):
        class ResidualScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    observation = {
                        **result["observation"],
                        "motor_angles_deg": dict(
                            result["observation"]["motor_angles_deg"]
                        ),
                    }
                    observation["motor_angles_deg"]["left_drive"] += 8
                    observation["motor_angles_deg"]["right_drive"] += 8
                    result["observation"] = observation
                    correlate_scan_restoration(
                        result["scan"], observation,
                    )
                    self.snapshot_value = {
                        **self.snapshot_value,
                        "observation": observation,
                    }
                return result

        controller = ResidualScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        result = self.adapter(
            controller, planner, max_decisions=2,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason,
            "decision_budget_exhausted",
        )
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"]
            + ["turn_left"] * 4,
        )
        self.assertTrue(
            planner.contexts[1].history[0][
                "odometry_reanchored_after_scan"
            ]
        )
        self.assertEqual(
            planner.contexts[1].observation["odometry"]["heading_mdeg"],
            0,
        )
        self.assertEqual(len(planner.contexts), 2)

    def test_scan_supported_side_turn_continues_through_no_valid_range(self):
        class NoReturnDuringSideTurnController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command == "turn_left":
                    result["observation"]["distance_mm"] = 2_000
                    if self.commands.count("turn_left") == 4:
                        # Make the next semantic action fail closed without
                        # changing the four settled pulse results under test.
                        self.snapshot_value["observation"][
                            "distance_mm"
                        ] = None
                return result

        controller = NoReturnDuringSideTurnController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        result = self.adapter(
            controller, planner, max_decisions=2,
        ).run(episode_context()[0])

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands, ["scan_front_arc"] + ["turn_left"] * 4
        )
        self.assertEqual(len(planner.contexts), 2)

    def test_no_valid_turn_eligibility_preserves_other_slice_gates(self):
        controller = FakeController(500)
        result = controller.command("turn_left")

        result["observation"]["distance_mm"] = 2_000
        self.assertFalse(blast_turn_slice_allows_continuation(result))
        self.assertTrue(blast_turn_slice_allows_continuation(
            result,
            allow_no_valid_distance_with_bounded_evidence=True,
        ))

        no_heading = copy.deepcopy(result)
        no_heading["observation"]["imu"].pop("heading_deg")
        self.assertTrue(blast_turn_slice_allows_continuation(
            no_heading,
            allow_no_valid_distance_with_bounded_evidence=True,
        ))

        for fault in ("close", "invalid", "body", "unsettled"):
            with self.subTest(fault=fault):
                candidate = copy.deepcopy(result)
                if fault == "close":
                    candidate["observation"]["distance_mm"] = 40
                elif fault == "invalid":
                    candidate["observation"]["distance_mm"] = None
                elif fault == "body":
                    candidate["observation"]["motor_angles_deg"][
                        "body"
                    ] = 156
                else:
                    candidate["observation"][
                        "rotation_sweep_window_verified"
                    ] = False
                self.assertFalse(blast_turn_slice_allows_continuation(
                    candidate,
                    allow_no_valid_distance_with_bounded_evidence=True,
                ))

    def test_full_surroundings_scan_offers_turns_at_no_return(self):
        controller = FakeController(2_000)
        adapter = self.adapter(controller, Planner([]))
        observation = adapter._observation()
        scan = scan_result(center_distance_mm=2_000)
        scan["sweep_coverage_deg"] = 356.0

        self.assertEqual(
            adapter._available_actions(observation, ({
                "action": SCAN_FRONT_ARC,
                "scan": scan,
            },)),
            (TURN_LEFT_90, TURN_RIGHT_90),
        )

        scan.pop("sweep_coverage_deg")
        self.assertEqual(
            adapter._available_actions(observation, ({
                "action": SCAN_FRONT_ARC,
                "scan": scan,
            },)),
            (),
        )

    def test_full_scan_brackets_one_bounded_advance_at_no_return(self):
        controller = FakeController(2_000)
        adapter = self.adapter(controller, Planner([]))
        observation = adapter._observation()
        scan, _final = surroundings_scan_result(
            copy.deepcopy(observation["sensors"]),
        )
        history = ({"action": SCAN_FRONT_ARC, "scan": scan},)
        latest_scan_view = {"scan": scan}

        self.assertIn(ADVANCE, adapter._available_actions(
            observation, history, latest_scan_view,
        ))
        unknown_flank = copy.deepcopy(scan)
        unknown_flank["angular_rays"][1].update({
            "distance_mm": 2_000,
            "range_state": RANGE_STATE_NO_VALID_DISTANCE,
        })
        self.assertIn(ADVANCE, adapter._available_actions(
            observation,
            ({"action": SCAN_FRONT_ARC, "scan": unknown_flank},),
            {"scan": unknown_flank},
        ))
        self.assertNotIn(ADVANCE, adapter._available_actions(
            observation,
            (*history, {"action": ADVANCE}),
            latest_scan_view,
        ))

        for side, change in (
            ("left_1", {"observation_settled": False}),
            ("right_1", {
                "distance_mm": 100,
                "range_state": RANGE_STATE_MEASURED,
            }),
        ):
            with self.subTest(side=side):
                blocked = copy.deepcopy(scan)
                ray = next(
                    item for item in blocked["angular_rays"]
                    if item["side"] == side
                )
                ray.update(change)
                self.assertNotIn(ADVANCE, adapter._available_actions(
                    observation,
                    ({"action": SCAN_FRONT_ARC, "scan": blocked},),
                    {"scan": blocked},
                ))

    def test_agentic_full_scan_can_advance_once_at_no_return(self):
        class FullNoReturnScanController(FakeController):
            def issue_no_return_scan_permit(self, **_values):
                return object()

            def command(
                self, command, *, cancel_requested=None,
                action_permit=None,
            ):
                if command != "scan_front_arc":
                    result = super().command(
                        command, cancel_requested=cancel_requested,
                    )
                    if command == "drive_forward":
                        result["observation"]["distance_mm"] = 2_000
                        self.snapshot_value["observation"] = result[
                            "observation"
                        ]
                    return result
                self.commands.append(command)
                center = copy.deepcopy(self.snapshot_value["observation"])
                scan, final = surroundings_scan_result(center)
                self.snapshot_value["observation"] = final
                return {
                    "schema": COMMAND_RESULT_SCHEMA,
                    "robot_id": ROBOT_ID,
                    "controller_id": CONTROLLER_ID,
                    "command": command,
                    "accepted": True,
                    "completed": True,
                    "receipt": {"turn_count": 16},
                    "observation": final,
                    "observation_settled": True,
                    "scan": scan,
                }

        controller = FullNoReturnScanController(2_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(ADVANCE),
        ])

        result = self.adapter(
            controller, planner, max_decisions=2,
        ).run(episode_context()[0])

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands, ["scan_front_arc", "drive_forward"],
        )
        self.assertIn(ADVANCE, planner.contexts[1].available_actions)

    def test_full_scan_turns_use_settled_rays_around_unknown_echoes(self):
        adapter = self.adapter(FakeController(), Planner([]))
        scan = scan_result(center_distance_mm=81)
        scan["sweep_coverage_deg"] = 353.29
        scan["all_observations_settled"] = False
        for index in (1, 3):
            scan["rays"][index]["observation_settled"] = False

        history = ({"action": SCAN_FRONT_ARC, "scan": scan},)
        self.assertTrue(adapter._current_scan_allows_quarter_turn(history))

        scan["rays"][4]["observation_settled"] = False
        self.assertFalse(adapter._current_scan_allows_quarter_turn(history))

    def test_agentic_scan_guided_turn_continues_at_no_return(self):
        class NoReturnAfterScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    result["observation"]["distance_mm"] = 2_000
                    self.snapshot_value["observation"] = result["observation"]
                return result

        controller = NoReturnAfterScanController()
        result = self.adapter(
            controller,
            Planner([
                decision(SCAN_FRONT_ARC),
                decision(TURN_RIGHT_90),
            ]),
            max_decisions=2,
        ).run(episode_context()[0])

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"] + ["turn_right"] * 4,
        )

    def test_arbitrary_planner_turn_still_stops_on_no_valid_range(self):
        class NoReturnDuringPlannerTurn(FakeController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command == "turn_left":
                    result["observation"]["distance_mm"] = 2_000
                    self.snapshot_value["observation"] = (
                        result["observation"]
                    )
                return result

        controller = NoReturnDuringPlannerTurn(500)
        result = self.adapter(
            controller, Planner([decision(TURN_LEFT_90)]),
            max_decisions=1,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason, "decision_budget_exhausted",
        )
        self.assertEqual(controller.commands, ["turn_left"])

    def test_unprojectable_current_scan_cannot_authorize_a_side_turn(self):
        class UnsettledScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    result["scan"]["all_observations_settled"] = False
                    result["scan"]["rays"][0].update({
                        "observation_settled": False,
                        "evidence_use": "SWEEP_CONTINUATION_ONLY",
                    })
                return result

        controller = UnsettledScanController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision("ADVANCE"),
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(raised.exception.code, "blast_planner_action_invalid")
        self.assertEqual(
            controller.commands,
            ["scan_front_arc", "drive_forward", "scan_front_arc"],
        )

    def test_scan_without_measured_center_does_not_select_detour(self):
        class NoMeasuredCenterController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    for ray in result["scan"]["rays"]:
                        ray["distance_mm"] = 2_000.0
                        ray["range_state"] = RANGE_STATE_NO_VALID_DISTANCE
                return result

        controller = NoMeasuredCenterController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(
            raised.exception.code,
            "blast_planner_action_invalid",
        )
        self.assertEqual(controller.commands, ["scan_front_arc"])

    def test_close_center_scan_blocks_turn_before_motor_command(self):
        for distance in (0.0, 40.0):
            with self.subTest(distance=distance):
                class CloseScanController(FakeScanController):
                    def command(self, command, *, cancel_requested=None):
                        result = super().command(
                            command,
                            cancel_requested=cancel_requested,
                        )
                        if command == "scan_front_arc":
                            result["scan"] = scan_result(
                                center_distance_mm=distance
                            )
                        return result

                controller = CloseScanController(500)
                planner = Planner([
                    decision(SCAN_FRONT_ARC),
                    decision(TURN_LEFT_90),
                ])

                with self.assertRaises(BlastEpisodeError) as raised:
                    self.adapter(controller, planner).run(
                        episode_context()[0]
                    )

                self.assertEqual(
                    raised.exception.code,
                    "blast_planner_action_invalid",
                )
                self.assertEqual(controller.commands, ["scan_front_arc"])

    def test_turn_without_current_scan_does_not_create_detour_intent(self):
        controller = FakeController(1_000)
        planner = Planner([
            decision(TURN_LEFT_90, plan=(TURN_LEFT_90,)),
            decision(TURN_RIGHT_90, plan=(TURN_RIGHT_90,)),
        ])

        self.adapter(
            controller, planner, max_decisions=2,
        ).run(episode_context()[0])

        self.assertNotIn(
            "navigation_intent",
            planner.contexts[1].observation,
        )

    def test_planned_later_turn_does_not_create_host_detour_intent(self):
        controller = FakeScanController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(
                "ADVANCE",
                plan=("ADVANCE", TURN_LEFT_90),
            ),
        ])

        result = self.adapter(
            controller, planner, max_decisions=2,
        ).run(episode_context()[0])

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands, ["scan_front_arc", "drive_forward"],
        )
        self.assertNotIn(
            "navigation_intent",
            planner.contexts[1].observation,
        )

    def test_partial_scan_guided_turn_stops_without_extra_execution(self):
        controller = FakeScanController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90, utterance="Jag svänger vänster."),
        ])
        context, _updates = episode_context()
        context.settings.speech_enabled = True

        class SpeechRecorder:
            def __init__(self):
                self.offers = []

            def start(self):
                return None

            def offer(self, **offer):
                self.offers.append(dict(offer))

            def cancel_episode(self, _episode_id):
                return None

            def close(self, **_options):
                return True

        speech = SpeechRecorder()

        class OneShotPartialTurn:
            def __init__(self):
                self.fired = False

            def is_set(self):
                if (
                    not self.fired
                    and controller.commands.count("turn_left") == 2
                ):
                    self.fired = True
                    return True
                return False

        context.emergency_stop_requested = OneShotPartialTurn()

        result = self.adapter(
            controller, planner, max_decisions=2,
            speech_runtime_factory=lambda **_kwargs: speech,
            speech_locales=("en",),
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands,
            ["scan_front_arc", "turn_left", "turn_left"],
        )
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(len(speech.offers), 1)
        self.assertEqual(speech.offers[0]["progress_revision"], 2)

    def test_unrestored_scan_stops_before_another_planner_turn(self):
        class UnrestoredScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    scan = result["scan"]
                    observation = {
                        **result["observation"],
                        "motor_angles_deg": dict(
                            result["observation"]["motor_angles_deg"]
                        ),
                    }
                    observation["motor_angles_deg"]["left_drive"] += 22
                    observation["motor_angles_deg"]["right_drive"] -= 22
                    correlate_scan_restoration(scan, observation)
                    scan["restoration_verified"] = False
                    scan["result"] = "restoration_unverified"
                    result["observation"] = observation
                    self.snapshot_value["observation"] = observation
                return result

        controller = UnrestoredScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision("ADVANCE"),
        ])

        with self.assertRaises(PhysicalNavigationContractError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(
            raised.exception.code,
            "blast_scan_restoration_unverified",
        )
        self.assertEqual(controller.commands, ["scan_front_arc"])
        self.assertEqual(len(planner.contexts), 1)

    def test_close_obstacle_removes_forward_and_blind_reverse(self):
        controller = FakeScanController(100)
        planner = Planner([decision(SCAN_FRONT_ARC)])
        context, _updates = episode_context()

        result = self.adapter(
            controller, planner, max_decisions=1,
        ).run(context)

        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")

        self.assertNotIn(
            "ADVANCE",
            planner.contexts[0].available_actions,
        )
        self.assertNotIn("REVERSE", planner.contexts[0].available_actions)
        self.assertIn(
            SCAN_FRONT_ARC,
            planner.contexts[0].available_actions,
        )

    def test_scan_ignores_missing_heading_but_requires_distance(self):
        for missing in ("heading", "distance"):
            controller = FakeScanController()
            if missing == "heading":
                controller.snapshot_value["observation"]["imu"].pop(
                    "heading_deg"
                )
            else:
                controller.snapshot_value["observation"].pop("distance_mm")
            planner = Planner([decision(SCAN_FRONT_ARC)])

            result = self.adapter(
                controller, planner, max_decisions=1,
            ).run(
                episode_context()[0]
            )

            with self.subTest(missing=missing):
                if missing == "heading":
                    self.assertIn(
                        SCAN_FRONT_ARC,
                        planner.contexts[0].available_actions,
                    )
                else:
                    self.assertEqual(
                        result.terminal_reason,
                        "no_safe_blast_action",
                    )
                    self.assertEqual(planner.contexts, [])

    def test_motion_makes_scan_available_again(self):
        adapter = self.adapter(FakeController(), Planner([]))
        observation = {
            "sensors": {
                "distance_mm": 300,
                "imu": {"heading_deg": 0},
                "motor_angles_deg": {
                    "left_drive": 0,
                    "right_drive": 0,
                    "body": 158,
                },
            }
        }

        available = adapter._available_actions(
            observation,
            (
                {"action": SCAN_FRONT_ARC},
                {"action": TURN_LEFT_90},
            ),
        )

        self.assertIn(SCAN_FRONT_ARC, available)

    def test_unobserved_or_too_close_state_authorizes_no_action(self):
        for distance, body, observes in (
            (53, 158, 0),
            (500, 156, 0),
        ):
            with self.subTest(distance=distance, body=body):
                controller = FakeController(distance)
                controller.snapshot_value["observation"][
                    "motor_angles_deg"
                ]["body"] = body
                planner = Planner([decision("ADVANCE")])

                result = self.adapter(controller, planner).run(
                    episode_context()[0]
                )

                self.assertFalse(result.completed)
                self.assertEqual(
                    result.terminal_reason,
                    "no_safe_blast_action",
                )
                self.assertEqual(
                    controller.commands,
                    [SETTLED_OBSERVATION_COMMAND] * observes,
                )
                self.assertEqual(planner.contexts, [])

    def test_no_return_offers_perception_scan_without_remeasure(self):
        class NoReturnController(FakeScanController):
            def __init__(self):
                super().__init__(2_000)
                self.permit_requests = []
                self.scan_permits = []

            def issue_no_return_scan_permit(self, **values):
                self.permit_requests.append(values)
                return object()

            def command(
                self, command, *, cancel_requested=None,
                action_permit=None,
            ):
                if command == "scan_front_arc":
                    self.scan_permits.append(action_permit)
                return super().command(
                    command, cancel_requested=cancel_requested,
                )

        controller = NoReturnController()
        planner = Planner([decision(SCAN_FRONT_ARC)])

        result = self.adapter(
            controller, planner,
            max_decisions=1,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(controller.commands, ["scan_front_arc"])
        self.assertEqual(
            planner.contexts[0].available_actions, (SCAN_FRONT_ARC,),
        )
        self.assertTrue(
            controller.permit_requests[0]["perception_only"],
        )
        self.assertIsNotNone(controller.scan_permits[0])

    def test_advance_to_no_return_offers_only_perception_scan(self):
        class NoReturnAfterAdvanceController(FakeScanController):
            def __init__(self):
                super().__init__(500)
                self.permit_requests = []
                self.scan_permits = []

            def issue_no_return_scan_permit(self, **values):
                self.permit_requests.append(values)
                return object()

            def command(
                self, command, *, cancel_requested=None,
                action_permit=None,
            ):
                if command == "scan_front_arc":
                    self.scan_permits.append(action_permit)
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command == "drive_forward":
                    result["observation"]["distance_mm"] = 2_000
                    self.snapshot_value["observation"] = result[
                        "observation"
                    ]
                return result

        controller = NoReturnAfterAdvanceController()
        planner = Planner([
            decision(ADVANCE),
            decision(SCAN_FRONT_ARC),
        ])

        result = self.adapter(
            controller, planner, max_decisions=2,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands, ["drive_forward", "scan_front_arc"],
        )
        self.assertEqual(
            planner.contexts[1].available_actions, (SCAN_FRONT_ARC,),
        )
        self.assertTrue(controller.permit_requests[-1]["allow_no_return"])
        self.assertTrue(controller.permit_requests[-1]["perception_only"])
        self.assertIsNotNone(controller.scan_permits[-1])

    def test_initial_stale_or_offline_observation_recovers_motorlessly(self):
        for fault in ("stale", "offline"):
            with self.subTest(fault=fault):
                controller = FreshStationaryController(
                    80, recovered_distance_mm=80,
                )
                controller.clock[0] = 5_000
                if fault == "stale":
                    controller.snapshot_value[
                        "last_observed_at_monotonic_ms"
                    ] = 1_000
                else:
                    controller.snapshot_value["state"] = "offline"
                    controller.reconnect_on_snapshot = 3
                planner = Planner([
                    decision(SCAN_FRONT_ARC,
                             assessment="observation recovered"),
                ])

                result = self.adapter(
                    controller, planner,
                    max_decisions=1,
                    monotonic_ms=lambda: controller.clock[0],
                ).run(episode_context()[0])

                self.assertFalse(result.completed)
                self.assertEqual(
                    result.terminal_reason, "decision_budget_exhausted",
                )
                self.assertEqual(
                    controller.commands,
                    [SETTLED_OBSERVATION_COMMAND, "scan_front_arc"],
                )
                self.assertIn(
                    SCAN_FRONT_ARC, planner.contexts[0].available_actions,
                )

    def test_action_safety_is_rechecked_after_model_latency(self):
        for action, change, code in (
            ("ADVANCE", ("distance_mm", 40), "blast_action_start_unverified"),
            (
                TURN_LEFT_90,
                ("distance_mm", 2_000),
                "blast_action_start_unverified",
            ),
            (
                TURN_LEFT_90,
                ("body", 156),
                "blast_action_start_unverified",
            ),
        ):
            with self.subTest(action=action):
                controller = FakeScanController(500)

                class SafetyChangesPlanner(Planner):
                    def decide(self, context):
                        result = super().decide(context)
                        field, value = change
                        if field == "body":
                            controller.snapshot_value["observation"][
                                "motor_angles_deg"
                            ]["body"] = value
                        else:
                            controller.snapshot_value["observation"][
                                field
                            ] = value
                        return result

                planner = SafetyChangesPlanner([decision(action)])

                with self.assertRaises(BlastEpisodeError) as raised:
                    self.adapter(controller, planner).run(
                        episode_context()[0]
                    )

                self.assertEqual(raised.exception.code, code)
                self.assertEqual(controller.commands, [])

    def test_scan_monitor_settling_can_recover_a_transient_snapshot(self):
        class TransientRangePlanner(Planner):
            def decide(self, context):
                result = super().decide(context)
                if result.decision.action == SCAN_FRONT_ARC:
                    controller.snapshot_value["observation"][
                        "distance_mm"
                    ] = 2_000
                return result

        class SettlingScanController(FakeScanController):
            def __init__(self, distance_mm):
                super().__init__(distance_mm)
                self.permit_requests = []

            def issue_no_return_scan_permit(self, **values):
                self.permit_requests.append(values)
                return object()

            def command(
                self, command, *, cancel_requested=None,
                action_permit=None,
            ):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    result["observation"]["distance_mm"] = 500
                    self.snapshot_value["observation"]["distance_mm"] = 500
                return result

        controller = SettlingScanController(500)
        planner = TransientRangePlanner([
            decision(SCAN_FRONT_ARC),
        ])

        result = self.adapter(
            controller, planner, max_decisions=1,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(controller.commands, ["scan_front_arc"])
        self.assertEqual(len(planner.contexts), 1)
        self.assertTrue(controller.permit_requests[0]["perception_only"])

    def test_scan_start_quality_failure_gets_exactly_one_retry(self):
        controller = ScanStartRetryController()
        planner = Planner([
            decision(SCAN_FRONT_ARC),
        ])

        result = self.adapter(controller, planner, max_decisions=1).run(
            episode_context()[0]
        )

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands,
            [
                "scan_front_arc",
                SETTLED_OBSERVATION_COMMAND,
                "scan_front_arc",
            ],
        )
        self.assertEqual(controller.scan_attempts, 2)
        self.assertEqual(len(planner.contexts), 1)
        self.assertEqual(len(planner.contexts[0].history), 0)

        controller = ScanStartRetryController(scan_failures=2)
        planner = Planner([decision(SCAN_FRONT_ARC)])

        result = self.adapter(controller, planner).run(
            episode_context()[0]
        )

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "no_safe_blast_action")
        self.assertEqual(
            controller.commands,
            [
                "scan_front_arc",
                SETTLED_OBSERVATION_COMMAND,
                "scan_front_arc",
            ],
        )
        self.assertEqual(controller.scan_attempts, 2)
        self.assertEqual(len(planner.contexts), 1)

    def test_scan_start_retry_uses_rotation_not_forward_clearance(self):
        controller = ScanStartRetryController(
            distance_mm=80,
            settled_observation={"distance_mm": 80},
        )
        planner = Planner([
            decision(SCAN_FRONT_ARC),
        ])

        result = self.adapter(controller, planner, max_decisions=1).run(
            episode_context()[0]
        )

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands,
            [
                "scan_front_arc",
                SETTLED_OBSERVATION_COMMAND,
                "scan_front_arc",
            ],
        )

    def test_scan_start_retry_requires_safe_stationary_anchor(self):
        cases = (
            ("close", "distance_mm", 40, 0),
            ("invalid", "distance_mm", None, 2),
            ("body", "body", 0, 0),
            ("encoder drift", "left_drive", 2, 0),
            ("moving", "motion_active", True, 0),
            ("stale", "timestamp", -3_001, 2),
        )
        for name, field, value, observes in cases:
            with self.subTest(name=name):
                def change_snapshot(controller):
                    observation = controller.snapshot_value["observation"]
                    if field == "timestamp":
                        controller.snapshot_value[
                            "last_observed_at_monotonic_ms"
                        ] = value
                    elif field == "body":
                        observation["motor_angles_deg"]["body"] = value
                    elif field in ("left_drive", "right_drive"):
                        observation["motor_angles_deg"][field] = value
                    else:
                        observation[field] = value

                controller = ScanStartRetryController(
                    after_failure=change_snapshot,
                )
                planner = Planner([decision(SCAN_FRONT_ARC)])

                result = self.adapter(controller, planner).run(
                    episode_context()[0]
                )

                self.assertFalse(result.completed)
                self.assertEqual(
                    result.terminal_reason, "no_safe_blast_action"
                )
                self.assertEqual(
                    controller.commands,
                    ["scan_front_arc"]
                    + [SETTLED_OBSERVATION_COMMAND] * observes,
                )
                self.assertEqual(controller.scan_attempts, 1)

        class SettledObservationTimeout(ScanStartRetryController):
            def command(
                self, command, *, cancel_requested=None,
                action_permit=None,
            ):
                if command == SETTLED_OBSERVATION_COMMAND:
                    self.commands.append(command)
                    if self.commands.count(command) == 1:
                        self.generation += 1
                    raise BlastControllerError(
                        "controller_command_timeout",
                        "injected settled observation timeout",
                        motion_started=False,
                    )
                return super().command(
                    command,
                    cancel_requested=cancel_requested,
                    action_permit=action_permit,
                )

        controller = SettledObservationTimeout()
        planner = Planner([decision(SCAN_FRONT_ARC)])

        result = self.adapter(controller, planner).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "no_safe_blast_action")
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"] + [SETTLED_OBSERVATION_COMMAND] * 2,
        )
        self.assertEqual(controller.scan_attempts, 1)

    def test_scan_retry_observation_must_be_fresh_settled_and_valid(self):
        drive_angles = {
            "left_drive": 0,
            "right_drive": 0,
            "claw": 0,
            "body": 158,
        }
        cases = (
            ("unsettled", {}, {"observation_settled": False}, 1, 2),
            ("stale", {}, {}, 0, 2),
            ("rejected", {}, {"accepted": False}, 1, 2),
            ("incomplete", {}, {"completed": False}, 1, 2),
            ("missing settled", {}, {"observation_settled": None}, 1, 2),
            ("close", {"distance_mm": 40}, {}, 1, 1),
            ("invalid", {"distance_mm": None}, {}, 1, 2),
            (
                "body",
                {"motor_angles_deg": {**drive_angles, "body": 0}},
                {},
                1,
                1,
            ),
            (
                "encoder drift",
                {"motor_angles_deg": {**drive_angles, "left_drive": 2}},
                {},
                1,
                1,
            ),
        )
        for (
            name, observation, result_update, timestamp_delta, attempts,
        ) in cases:
            with self.subTest(name=name):
                controller = ScanStartRetryController(
                    settled_observation=observation,
                    settled_result=result_update,
                    settled_timestamp_delta=timestamp_delta,
                )
                planner = Planner([decision(SCAN_FRONT_ARC)])

                result = self.adapter(controller, planner).run(
                    episode_context()[0]
                )

                self.assertFalse(result.completed)
                self.assertEqual(
                    result.terminal_reason, "no_safe_blast_action"
                )
                self.assertEqual(
                    controller.commands,
                    ["scan_front_arc"]
                    + [SETTLED_OBSERVATION_COMMAND] * attempts,
                )
                self.assertEqual(controller.scan_attempts, 1)

    def test_scan_retry_excludes_motion_or_non_start_failures(self):
        cases = (
            ("scan_sweep_clearance_lost", False, False),
            ("scan_sweep_observation_unverified", False, False),
            ("scan_start_clearance_unverified", None, False),
            ("scan_start_clearance_unverified", True, False),
            ("controller_command_timeout", False, True),
        )
        for error_code, motion_started, raises in cases:
            with self.subTest(
                error_code=error_code, motion_started=motion_started,
            ):
                controller = ScanStartRetryController(
                    failure_code=error_code,
                    motion_started=motion_started,
                )
                planner = Planner([decision(SCAN_FRONT_ARC)])

                if raises:
                    with self.assertRaises(BlastControllerError) as raised:
                        self.adapter(controller, planner).run(
                            episode_context()[0]
                        )
                    self.assertEqual(raised.exception.code, error_code)
                else:
                    result = self.adapter(controller, planner).run(
                        episode_context()[0]
                    )
                    self.assertFalse(result.completed)
                self.assertEqual(controller.commands, ["scan_front_arc"])
                self.assertEqual(controller.scan_attempts, 1)

    def test_scan_retry_preserves_stop_and_deadline_precedence(self):
        for control, after_observe, expected_commands in (
            ("stop", False, ["scan_front_arc"]),
            (
                "stop",
                True,
                ["scan_front_arc", SETTLED_OBSERVATION_COMMAND],
            ),
            ("deadline", False, ["scan_front_arc"]),
            (
                "deadline",
                True,
                ["scan_front_arc", SETTLED_OBSERVATION_COMMAND],
            ),
        ):
            with self.subTest(control=control, after_observe=after_observe):
                clock = [1_000]
                context, _updates = episode_context()
                context.settings.max_episode_ms = 90_000

                def trigger(_controller):
                    if control == "stop":
                        context.stop_requested.set()
                    else:
                        clock[0] = 91_000

                controller = ScanStartRetryController(**{
                    "after_observe" if after_observe else "after_failure": (
                        trigger
                    ),
                })
                planner = Planner([decision(SCAN_FRONT_ARC)])

                result = self.adapter(
                    controller,
                    planner,
                    monotonic_ms=lambda: clock[0],
                ).run(context)

                self.assertFalse(result.completed)
                self.assertEqual(
                    result.terminal_reason,
                    "stopped"
                    if control == "stop"
                    else "episode_deadline_elapsed",
                )
                self.assertEqual(controller.commands, expected_commands)
                self.assertEqual(controller.scan_attempts, 1)

    def test_planner_scan_retry_can_continue_from_exact_no_return(self):
        def lose_echo(controller):
            controller.snapshot_value["observation"][
                "distance_mm"
            ] = 2_000

        class NoReturnRetryController(ScanStartRetryController):
            def __init__(self):
                super().__init__(
                    after_failure=lose_echo,
                    settled_observation={"distance_mm": 2_000},
                )
                self.permit_requests = []

            def issue_no_return_scan_permit(self, **values):
                self.permit_requests.append(values)
                return object()

        controller = NoReturnRetryController()
        planner = Planner([decision(SCAN_FRONT_ARC)])

        result = self.adapter(
            controller, planner, max_decisions=1,
        ).run(
            episode_context()[0]
        )

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "decision_budget_exhausted")
        self.assertEqual(
            controller.commands,
            [
                "scan_front_arc",
                SETTLED_OBSERVATION_COMMAND,
                "scan_front_arc",
            ],
        )
        self.assertEqual(controller.scan_attempts, 2)
        self.assertEqual(len(controller.scan_permits), 2)
        self.assertTrue(all(
            permit is not None for permit in controller.scan_permits
        ))
        self.assertTrue(controller.permit_requests[-1]["allow_no_return"])
        self.assertTrue(controller.permit_requests[-1]["perception_only"])

    def test_settled_scan_refusal_is_safe_noncomplete(self):
        for error_code in (
            "scan_start_clearance_unverified",
            "scan_sweep_clearance_lost",
            "scan_sweep_observation_unverified",
        ):
            for cancelled, terminal_reason in (
                (False, "no_safe_blast_action"),
                (True, "stopped"),
            ):
                with self.subTest(error_code=error_code, cancelled=cancelled):
                    context, _updates = episode_context()

                    class RefusingScanController(FakeScanController):
                        def command(self, command, *, cancel_requested=None):
                            if command == "scan_front_arc":
                                if cancelled:
                                    context.stop_requested.set()
                                raise BlastControllerError(
                                    error_code,
                                    "settled scan start was unsafe",
                                )
                            return super().command(
                                command,
                                cancel_requested=cancel_requested,
                            )

                    controller = RefusingScanController(500)
                    planner = Planner([decision(SCAN_FRONT_ARC)])

                    result = self.adapter(controller, planner).run(context)

                    self.assertFalse(result.completed)
                    self.assertEqual(result.terminal_reason, terminal_reason)
                    self.assertEqual(controller.commands, [])
                    self.assertEqual(len(planner.contexts), 1)

    def test_scan_guided_motion_consumes_only_agent_decision_budget(self):
        controller = FakeScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision(TURN_LEFT_90, plan=(TURN_LEFT_90,)),
        ])
        context, _updates = episode_context()

        result = self.adapter(
            controller,
            planner,
            max_decisions=2,
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason,
            "decision_budget_exhausted",
        )
        self.assertFalse(planner.contexts[0].completion_allowed)
        self.assertFalse(planner.contexts[1].completion_allowed)
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"] + ["turn_left"] * 4,
        )

    def test_scan_requires_a_structured_controller_result(self):
        controller = FakeController()
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
        ])

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(raised.exception.code, "blast_scan_result_invalid")
        self.assertEqual(controller.commands, ["scan_front_arc"])

    def test_stop_wins_after_planning_but_before_motor_start(self):
        controller = FakeController()
        context, _updates = episode_context()

        class CancellingPlanner:
            def decide(self, _context):
                context.stop_requested.set()
                return decision("ADVANCE", plan=("ADVANCE",))

        result = self.adapter(
            controller,
            CancellingPlanner(),
        ).run(context)

        self.assertEqual(result.terminal_reason, "stopped")
        self.assertFalse(result.completed)
        self.assertEqual(controller.commands, [])

    def test_stop_wins_over_an_invalid_late_planner_result(self):
        controller = FakeController()
        context, _updates = episode_context()

        class InvalidCancellingPlanner:
            def decide(self, _context):
                context.stop_requested.set()
                return object()

        result = self.adapter(
            controller,
            InvalidCancellingPlanner(),
        ).run(context)

        self.assertEqual(result.terminal_reason, "stopped")
        self.assertFalse(result.completed)
        self.assertEqual(controller.commands, [])

    def test_deadline_discards_late_planner_output_before_dispatch(self):
        for final_clock, terminal_reason in (
            (1_999, "episode_deadline_headroom_insufficient"),
            (2_000, "episode_deadline_elapsed"),
        ):
            with self.subTest(final_clock=final_clock):
                clock = [1_000]
                context, _updates = episode_context()
                context.settings.max_episode_ms = 1_000
                controller = FakeController()

                class LatePlanner(Planner):
                    def decide(self, planner_context):
                        result = super().decide(planner_context)
                        clock[0] = final_clock
                        return result

                planner = LatePlanner([decision("ADVANCE")])

                result = self.adapter(
                    controller, planner, monotonic_ms=lambda: clock[0],
                ).run(context)

                self.assertFalse(result.completed)
                self.assertEqual(result.terminal_reason, terminal_reason)
                self.assertEqual(controller.commands, [])
                self.assertEqual(len(planner.contexts), 1)

    def test_control_before_planner_skips_the_model_call(self):
        for control, terminal_reason in (
            ("deadline", "episode_deadline_elapsed"),
            ("stop", "stopped"),
        ):
            with self.subTest(control=control):
                clock = [1_000]
                context, _updates = episode_context()
                context.settings.max_episode_ms = 1_000

                class ControlDuringSnapshotController(FakeController):
                    def snapshot(self):
                        snapshot = super().snapshot()
                        if control == "deadline":
                            clock[0] = 2_000
                        else:
                            context.stop_requested.set()
                        return snapshot

                controller = ControlDuringSnapshotController()
                planner = Planner([decision("ADVANCE")])

                result = self.adapter(
                    controller, planner, monotonic_ms=lambda: clock[0],
                ).run(context)

                self.assertFalse(result.completed)
                self.assertEqual(result.terminal_reason, terminal_reason)
                self.assertEqual(planner.contexts, [])
                self.assertEqual(controller.commands, [])

    def test_deadline_between_turn_pulses_keeps_partial_map_pose(self):
        clock = [1_000]
        context, _updates = episode_context()
        context.settings.max_episode_ms = 5_000

        class DeadlineAfterFirstPulseController(FakeController):
            def __init__(self):
                super().__init__()
                self.invocations = 0

            def command(self, command, *, cancel_requested=None):
                if command == "turn_left":
                    self.invocations += 1
                if command == "turn_left" and self.invocations == 2:
                    clock[0] = 6_000
                return super().command(
                    command, cancel_requested=cancel_requested,
                )

        controller = DeadlineAfterFirstPulseController()
        planner = Planner([decision(TURN_LEFT_90)])
        adapter = self.adapter(
            controller, planner, monotonic_ms=lambda: clock[0],
        )

        result = adapter.run(context)

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason, "episode_deadline_elapsed"
        )
        self.assertEqual(controller.commands, ["turn_left"])
        self.assertEqual(controller.invocations, 2)
        history = adapter.spatial_map_provider.snapshot()["pose_history"]
        self.assertEqual(len(history), 2)
        self.assertGreater(abs(history[-1]["heading_mdeg"]), 0)
        self.assertLess(abs(history[-1]["heading_mdeg"]), 90_000)

    def test_deadline_or_stop_wins_when_late_planner_raises(self):
        for stop, terminal_reason in (
            (False, "episode_deadline_elapsed"),
            (True, "stopped"),
        ):
            with self.subTest(stop=stop):
                clock = [1_000]
                context, _updates = episode_context()
                context.settings.max_episode_ms = 1_000
                controller = FakeController()

                class RaisingPlanner:
                    def decide(self, _planner_context):
                        clock[0] = 2_000
                        if stop:
                            context.stop_requested.set()
                        raise RuntimeError("late planner failure")

                result = self.adapter(
                    controller,
                    RaisingPlanner(),
                    monotonic_ms=lambda: clock[0],
                ).run(context)

                self.assertFalse(result.completed)
                self.assertEqual(result.terminal_reason, terminal_reason)
                self.assertEqual(controller.commands, [])

    def test_request_stop_uses_same_controller_owner(self):
        controller = FakeController()
        entered = threading.Event()
        release = threading.Event()
        context, _updates = episode_context()

        class BlockingPlanner:
            def decide(self, _context):
                entered.set()
                release.wait(2)
                return decision(COMPLETE, assessment="done")

        adapter = self.adapter(controller, BlockingPlanner())
        result = []
        worker = threading.Thread(target=lambda: result.append(
            adapter.run(context)
        ))
        worker.start()
        self.assertTrue(entered.wait(1))

        context.stop_requested.set()
        adapter.request_stop()
        release.set()
        worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(controller.commands, ["stop"])
        self.assertEqual(result[0].terminal_reason, "stopped")

    def test_request_stop_cancels_active_speech_with_same_episode(self):
        controller = FakeController()
        entered = threading.Event()
        release = threading.Event()
        context, _updates = episode_context()
        context.settings.speech_enabled = True

        class BlockingPlanner:
            def decide(self, _context):
                entered.set()
                release.wait(2)
                return decision(COMPLETE, assessment="done")

        class SpeechRecorder:
            def __init__(self):
                self.cancelled = []

            def start(self):
                return None

            def offer(self, **_offer):
                return 1

            def cancel_episode(self, episode_id):
                self.cancelled.append(episode_id)

            def close(self, **_options):
                return True

        speech = SpeechRecorder()
        adapter = self.adapter(
            controller,
            BlockingPlanner(),
            speech_runtime_factory=lambda **_kwargs: speech,
            speech_locales=("en",),
        )
        results = []
        worker = threading.Thread(
            target=lambda: results.append(adapter.run(context))
        )
        worker.start()
        self.assertTrue(entered.wait(1))

        context.stop_requested.set()
        adapter.request_stop()
        release.set()
        worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(controller.commands, ["stop"])
        self.assertIn("episode-1", speech.cancelled)
        self.assertEqual(results[0].terminal_reason, "stopped")

    def test_rejects_stale_or_nonidle_observation(self):
        for changes, code in (
            ({"last_observed_at_monotonic_ms": 1}, "blast_observation_stale"),
            (
                {"observation": {"distance_mm": 500, "motion_active": True}},
                "blast_motion_not_idle",
            ),
        ):
            controller = FakeController()
            controller.snapshot_value = {
                **controller.snapshot_value,
                **changes,
            }
            adapter = self.adapter(
                controller,
                Planner([decision(COMPLETE)]),
                max_observation_age_ms=500,
            )
            with self.subTest(code=code), self.assertRaises(
                BlastEpisodeError
            ) as raised:
                adapter.run(episode_context()[0])
            self.assertEqual(raised.exception.code, code)


if __name__ == "__main__":
    unittest.main()
