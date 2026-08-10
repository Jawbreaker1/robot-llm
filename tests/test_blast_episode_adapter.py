import copy
import threading
import unittest
from types import SimpleNamespace

from robot_agent.blast_episode_adapter import (
    ACTION_COMMANDS,
    BlastEpisodeError,
    BlastEpisodeRuntimeAdapter,
    _side_search_followup_slots,
    _side_search_progress,
    _side_search_required_slots,
    _side_search_waypoint,
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
from robot_agent.blast_turn_safety import (
    blast_turn_slice_allows_continuation,
)
from robot_agent.lm_studio_controller_action import (
    COMPLETE,
    ControllerActionDecision,
    ControllerActionPlannerResult,
)
from robot_agent.physical_navigation_contract import (
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    PhysicalNavigationContractError,
)
from robot_agent.physical_odometry import PhysicalPose
from robot_agent.robot_control_contract import RobotRuntimeUpdate


def decision(action, *, plan=(), assessment="ok"):
    return ControllerActionPlannerResult(
        decision=ControllerActionDecision(
            action=action,
            confidence_milli=900,
            assessment=assessment,
            plan=tuple(plan),
            utterance=None,
        ),
        latency_ms=12,
    )


def scan_result(*, center_distance_mm=500.0):
    distances = (
        ("center", center_distance_mm),
        ("left_near", 900.0),
        ("left_far", 2_000.0),
        ("right_near", 1_200.0),
        ("right_far", 2_000.0),
    )
    relative_headings = (0.0, -22.0, -45.0, 24.0, 47.0)
    return {
        "schema": SCAN_RESULT_SCHEMA,
        "state": "complete",
        "result": "restored",
        "start_heading_deg": 0.0,
        "final_heading_deg": 0.0,
        "restoration_error_deg": 0.0,
        "restoration_verified": True,
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
                "heading_deg": relative_heading,
                "relative_heading_deg": relative_heading,
                "observation_settled": True,
            }
            for (side, distance_mm), relative_heading in zip(
                distances,
                relative_headings,
            )
        ],
    }


class FakeController:
    def __init__(self, distance_mm=500):
        self.commands = []
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
            result["scan"] = scan_result()
        return result


class FullDetourController(FakeScanController):
    def __init__(
        self, *, scan_centers=None, pass_side_distance=1_500.0,
        result_ranges=None,
    ):
        super().__init__(1_000)
        self.scan_count = 0
        self.scan_centers = dict(scan_centers or {})
        self.pass_side_distance = pass_side_distance
        self.result_ranges = dict(result_ranges or {})

    def command(self, command, *, cancel_requested=None):
        result = super().command(
            command, cancel_requested=cancel_requested,
        )
        result["observation"]["distance_mm"] = 1_000
        self.snapshot_value["observation"]["distance_mm"] = 1_000
        if command == "scan_front_arc":
            self.scan_count += 1
            if self.scan_count == 1:
                scan = scan_result(center_distance_mm=310.0)
                scan["rays"][1].update({
                    "distance_mm": 368.0,
                    "heading_deg": -23.322266,
                    "relative_heading_deg": -23.322266,
                })
                scan["rays"][3].update({
                    "distance_mm": 286.0,
                    "heading_deg": 25.093086,
                    "relative_heading_deg": 25.093086,
                })
            else:
                scan = scan_result(center_distance_mm=(
                    self.scan_centers.get(self.scan_count, 1_500.0)
                ))
                if self.scan_count == 2:
                    for index in (1, 3):
                        scan["rays"][index]["distance_mm"] = 1_500.0
                if self.scan_count == 3:
                    for index in (1, 3):
                        scan["rays"][index]["distance_mm"] = (
                            self.pass_side_distance
                        )
            result["scan"] = scan
            if self.scan_count in self.result_ranges:
                distance = self.result_ranges[self.scan_count]
                result["observation"]["distance_mm"] = distance
                self.snapshot_value["observation"]["distance_mm"] = distance
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
    def adapter(self, controller, planner, **changes):
        monotonic_ms = changes.pop("monotonic_ms", lambda: 1_000)
        return BlastEpisodeRuntimeAdapter(
            controller=controller,
            planner_factory=lambda _model: planner,
            monotonic_ms=monotonic_ms,
            **changes,
        )

    def test_replans_after_each_bounded_action_and_completes(self):
        controller = FakeController(500)
        planner = Planner([
            decision(
                "ADVANCE",
                plan=("ADVANCE", TURN_LEFT_90, COMPLETE),
                assessment="Move one bounded pulse.",
            ),
            decision(
                COMPLETE,
                assessment="The requested state is reached.",
            ),
        ])
        context, updates = episode_context()

        result = self.adapter(controller, planner).run(context)

        self.assertTrue(result.completed)
        self.assertEqual(result.terminal_reason, "completed")
        self.assertEqual(controller.commands, ["drive_forward"])
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

    def test_existing_map_shows_goal_and_encoder_odometry_without_authority(self):
        controller = FakeController(500)
        planner = Planner([decision("ADVANCE"), decision(COMPLETE)])
        adapter = self.adapter(controller, planner)

        result = adapter.run(episode_context()[0])

        self.assertTrue(result.completed)
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
            spatial_map_bridge=BrokenMap(),
        ).run(episode_context()[0])

        self.assertTrue(result.completed)
        self.assertEqual(controller.commands, ["drive_forward"])
        self.assertEqual(len(planner.contexts), 2)

    def test_scripted_semantic_actions_use_four_pulse_turn_and_carry_pose(self):
        controller = FakeController(1_000)
        planner = Planner([
            decision("ADVANCE"),
            decision("ADVANCE"),
            decision(TURN_LEFT_90),
            decision("ADVANCE"),
            decision("ADVANCE"),
            decision(COMPLETE, assessment="Fixture sequence ended."),
        ])
        context, updates = episode_context()

        result = self.adapter(controller, planner).run(context)

        self.assertTrue(result.completed)
        self.assertEqual(
            controller.commands,
            ["drive_forward"] * 2
            + ["turn_left"] * 4
            + ["drive_forward"] * 2,
        )
        final_pose = planner.contexts[-1].observation["odometry"]
        self.assertLessEqual(abs(final_pose["x_mm"] - 90), 10)
        self.assertLessEqual(abs(final_pose["y_mm"] - 90), 10)
        self.assertEqual(final_pose["verified_motion_count"], 5)
        self.assertNotIn("pose", updates[-2])
        runtime_update = None
        for update in updates:
            runtime_update = RobotRuntimeUpdate.from_mapping(
                update,
                runtime_update,
            )

    def test_scan_is_one_agent_action_with_stable_heading_reference(self):
        class ScanController(FakeController):
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
                observation = {
                    **result["observation"],
                    "imu": {
                        **result["observation"]["imu"],
                        "heading_deg": -179,
                    },
                }
                result["observation"] = observation
                result["scan"] = scan_result(center_distance_mm=300.0)
                self.snapshot_value = {
                    **self.snapshot_value,
                    "observation": observation,
                }
                return result

        controller = ScanController()
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision(COMPLETE, assessment="The route is understood."),
        ])
        context, updates = episode_context()

        result = self.adapter(controller, planner).run(context)

        self.assertTrue(result.completed)
        self.assertEqual(controller.commands, ["scan_front_arc"])
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
            (TURN_LEFT_90, TURN_RIGHT_90),
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

        with self.assertRaises(BlastEpisodeError) as rejected:
            self.adapter(LegacyScanController(), planner).run(
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
                    result["scan"]["rays"][1][
                        "relative_heading_deg"
                    ] = 22.0
                return result

        controller = InvalidHeadingController()
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(COMPLETE, assessment="Raw scan remains available."),
        ])
        context, updates = episode_context()

        result = self.adapter(controller, planner).run(context)

        self.assertTrue(result.completed)
        self.assertEqual(controller.commands, ["scan_front_arc"])
        self.assertEqual(len(planner.contexts), 2)
        self.assertIn("ADVANCE", planner.contexts[1].available_actions)
        self.assertIn("scan", planner.contexts[1].history[0])
        runtime_scan = [
            update["scan"] for update in updates if "scan" in update
        ][0]
        self.assertNotIn("planar_projection", runtime_scan)

    def test_projected_scan_requires_side_choice_before_more_motion(self):
        controller = FakeScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision("ADVANCE"),
        ])

        with self.assertRaises(BlastEpisodeError) as rejected:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(rejected.exception.code, "blast_planner_action_invalid")
        self.assertEqual(controller.commands, ["scan_front_arc"])
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(
            planner.contexts[1].available_actions,
            (TURN_LEFT_90, TURN_RIGHT_90),
        )
        self.assertTrue(planner.contexts[1].completion_allowed)

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
                    self.snapshot_value = {
                        **self.snapshot_value,
                        "observation": observation,
                    }
                return result

        controller = ResidualScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
            decision("ABORT"),
        ])

        result = self.adapter(controller, planner).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason,
            "side_search_observation_collected",
        )
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"]
            + ["turn_left"] * 4
            + ["drive_forward"] * 5
            + ["turn_right"] * 4
            + ["scan_front_arc"],
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

    def test_scan_guided_side_search_collects_second_viewpoint(self):
        for action, side in (
            (TURN_LEFT_90, "LEFT"),
            (TURN_RIGHT_90, "RIGHT"),
        ):
            with self.subTest(action=action):
                controller = FakeScanController(1_000)
                restore_action = (
                    TURN_RIGHT_90
                    if action == TURN_LEFT_90
                    else TURN_LEFT_90
                )
                planner = Planner([
                    decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
                    decision(action, plan=(action, "ADVANCE")),
                ])
                context, updates = episode_context()

                adapter = self.adapter(
                    controller,
                    planner,
                    max_decisions=9,
                )
                result = adapter.run(context)

                self.assertFalse(result.completed)
                self.assertEqual(
                    result.terminal_reason,
                    "side_search_observation_collected",
                )
                self.assertNotIn(
                    "navigation_intent",
                    planner.contexts[1].observation,
                )
                self.assertEqual(len(planner.contexts), 2)
                self.assertEqual(
                    controller.commands,
                    ["scan_front_arc"]
                    + list(ACTION_COMMANDS[action])
                    + ["drive_forward"] * 5
                    + list(ACTION_COMMANDS[restore_action])
                    + ["scan_front_arc"],
                )
                multi_view = updates[-1]["scan"][
                    "multi_view_observations"
                ]
                self.assertGreaterEqual(
                    multi_view["viewpoint_separation_mm"],
                    190,
                )
                self.assertFalse(multi_view["object_association_proven"])
                self.assertFalse(multi_view["clearance_proven"])
                self.assertFalse(multi_view["passage_proven"])
                self.assertFalse(multi_view["route_eligible"])
                spatial_map = adapter.spatial_map_provider.snapshot()
                trace = spatial_map["navigation_trace"]
                self.assertEqual(
                    trace["planned_leg"]["selected_side"], side
                )
                self.assertEqual(
                    trace["planned_leg"]["scope"],
                    "SEARCH_POSITION_ONLY",
                )
                self.assertFalse(
                    trace["planned_leg"]["clearance_proven"]
                )
                self.assertEqual(len(trace["planar_scan_views"]), 2)
                self.assertGreaterEqual(
                    len(spatial_map["pose_history"]), 8
                )
                self.assertEqual(
                    trace["final_goal"]["target_x_mm"], 420
                )
                self.assertEqual(
                    multi_view["strategy_source"],
                    "PLANNER_ACTION",
                )
                self.assertEqual(
                    multi_view["execution_source"],
                    "HOST_SIDE_SEARCH_ACTION",
                )
                self.assertEqual(
                    multi_view["host_action_trace"],
                    ["ADVANCE"] * 5
                    + [restore_action, SCAN_FRONT_ARC],
                )
                self.assertEqual(multi_view["host_action_count"], 7)
                self.assertEqual(len(multi_view["views"]), 2)
                self.assertIsNot(updates[-1]["scan"], updates[-2]["scan"])
                origin_history_scan = planner.contexts[1].history[0]["scan"]
                origin_view_scan = multi_view["views"][0]["scan"]
                self.assertIsNot(origin_view_scan, origin_history_scan)
                origin_history_scan["rays"][0]["distance_mm"] = 1
                self.assertNotEqual(
                    origin_view_scan["rays"][0]["distance_mm"],
                    1,
                )
                host_updates = [
                    update for update in updates
                    if str(update.get("message", "")).startswith(
                        "Host follows the selected side-search waypoint"
                    )
                ]
                self.assertEqual(len(host_updates), 7)
                self.assertTrue(all(
                    update["plan"] == []
                    and update["model_latency_ms"] is None
                    for update in host_updates
                ))
                runtime_update = None
                for update in updates:
                    runtime_update = RobotRuntimeUpdate.from_mapping(
                        update,
                        runtime_update,
                    )

    def test_host_side_search_respects_the_total_action_budget(self):
        controller = FakeScanController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        result = self.adapter(
            controller,
            planner,
            max_decisions=8,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason,
            "side_search_budget_insufficient",
        )
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"],
        )
        self.assertEqual(controller.commands.count("scan_front_arc"), 1)

    def test_actual_post_turn_pose_rechecks_remaining_budget(self):
        class ShiftedTurnController(FakeScanController):
            def __init__(self):
                super().__init__(1_000)
                self.scan_count = 0

            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    self.scan_count += 1
                    if self.scan_count == 1:
                        scan = scan_result(center_distance_mm=310.0)
                        scan["rays"][1].update({
                            "distance_mm": 368.0,
                            "heading_deg": -23.322266,
                            "relative_heading_deg": -23.322266,
                        })
                        scan["rays"][3].update({
                            "distance_mm": 286.0,
                            "heading_deg": 25.093086,
                            "relative_heading_deg": 25.093086,
                        })
                        result["scan"] = scan
                elif command == "turn_left":
                    before = result["receipt"]["before_angles_deg"]
                    motors = result["observation"]["motor_angles_deg"]
                    motors["left_drive"] = before["left_drive"] - 91
                    motors["right_drive"] = before["right_drive"] + 1
                    self.snapshot_value["observation"] = result["observation"]
                return result

        controller = ShiftedTurnController()
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        result = self.adapter(
            controller,
            planner,
            max_decisions=13,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason,
            "side_search_budget_insufficient",
        )
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"] + ["turn_left"] * 4,
        )
        self.assertNotIn("drive_forward", controller.commands)
        self.assertEqual(len(planner.contexts), 2)

    def test_stop_wins_over_post_turn_waypoint_refusal(self):
        context, _updates = episode_context()

        class CancelOffHeadingController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "turn_left":
                    before = result["receipt"]["before_angles_deg"]
                    motors = result["observation"]["motor_angles_deg"]
                    motors["left_drive"] = before["left_drive"] - 20
                    motors["right_drive"] = before["right_drive"] + 20
                    self.snapshot_value["observation"] = result["observation"]
                    if self.commands.count("turn_left") == 4:
                        context.stop_requested.set()
                return result

        controller = CancelOffHeadingController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        result = self.adapter(controller, planner).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "stopped")
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"] + ["turn_left"] * 4,
        )
        self.assertNotIn("drive_forward", controller.commands)
        self.assertEqual(len(planner.contexts), 2)

    def test_live_surface_extent_drives_nine_bounded_left_steps(self):
        class LiveSurfaceScanController(FakeScanController):
            def __init__(self):
                super().__init__(1_000)
                self.scan_count = 0

            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command != "scan_front_arc":
                    return result
                self.scan_count += 1
                if self.scan_count == 1:
                    scan = scan_result(center_distance_mm=310.0)
                    scan["rays"][1].update({
                        "distance_mm": 368.0,
                        "heading_deg": -23.322266,
                        "relative_heading_deg": -23.322266,
                    })
                    scan["rays"][3].update({
                        "distance_mm": 286.0,
                        "heading_deg": 25.093086,
                        "relative_heading_deg": 25.093086,
                    })
                    result["scan"] = scan
                return result

        controller = LiveSurfaceScanController()
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])
        context, updates = episode_context()

        result = self.adapter(
            controller,
            planner,
            max_decisions=13,
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason,
            "side_search_observation_collected",
        )
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"]
            + ["turn_left"] * 4
            + ["drive_forward"] * 9
            + ["turn_right"] * 4
            + ["scan_front_arc"],
        )
        multi_view = updates[-1]["scan"]["multi_view_observations"]
        self.assertGreaterEqual(multi_view["viewpoint_separation_mm"], 373)
        self.assertEqual(
            multi_view["host_action_trace"],
            ["ADVANCE"] * 9 + [TURN_RIGHT_90, SCAN_FRONT_ARC],
        )
        self.assertFalse(multi_view["object_association_proven"])
        self.assertFalse(multi_view["clearance_proven"])
        self.assertFalse(multi_view["passage_proven"])
        self.assertFalse(multi_view["route_eligible"])

    def test_two_view_side_choice_executes_full_host_detour(self):
        controller = FullDetourController()
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        adapter = self.adapter(
            controller,
            planner,
            max_decisions=51,
            execute_provisional_detour=True,
        )
        context, updates = episode_context()

        result = adapter.run(context)

        self.assertTrue(result.completed)
        self.assertEqual(result.terminal_reason, "completed")
        self.assertEqual(len(planner.contexts), 2)
        self.assertGreaterEqual(controller.commands.count("scan_front_arc"), 4)
        self.assertIn("turn_right", controller.commands)
        self.assertGreater(controller.commands.count("drive_forward"), 9)
        trace = adapter.spatial_map_provider.snapshot()["navigation_trace"]
        self.assertTrue(trace["final_goal"]["navigation_enforced"])
        self.assertGreaterEqual(
            trace["final_goal"]["current_forward_progress_mm"],
            trace["final_goal"]["minimum_forward_progress_mm"],
        )
        self.assertIsNone(trace["planned_leg"])
        self.assertTrue(any(
            str(update.get("message", "")).startswith(
                "Host follows the local-detour route:"
            )
            for update in updates
        ))

    def test_close_pass_or_final_scan_never_completes_detour(self):
        cases = (
            ({3: 54.0}, 1_500.0, {}, 3),
            ({}, 54.0, {}, 3),
            ({4: 54.0}, 1_500.0, {}, 4),
            ({}, 1_500.0, {4: 54.0}, 4),
        )
        for scan_centers, side_distance, result_ranges, scan_number in cases:
            with self.subTest(
                scan_centers=scan_centers,
                side_distance=side_distance,
            ):
                controller = FullDetourController(
                    scan_centers=scan_centers,
                    pass_side_distance=side_distance,
                    result_ranges=result_ranges,
                )
                planner = Planner([
                    decision(SCAN_FRONT_ARC),
                    decision(TURN_LEFT_90),
                ])

                result = self.adapter(
                    controller,
                    planner,
                    max_decisions=64,
                    execute_provisional_detour=True,
                ).run(episode_context()[0])

                self.assertFalse(result.completed)
                self.assertEqual(
                    result.terminal_reason,
                    "detour_verification_unavailable",
                )
                self.assertEqual(controller.scan_count, scan_number)
                self.assertEqual(controller.commands[-1], "scan_front_arc")
                self.assertEqual(len(planner.contexts), 2)

    def test_close_range_stops_host_turn_after_one_pulse(self):
        class CloseDuringMergeTurnController(FullDetourController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "turn_right" and self.scan_count == 3:
                    result["observation"]["distance_mm"] = 40
                    self.snapshot_value["observation"]["distance_mm"] = 40
                return result

        controller = CloseDuringMergeTurnController()
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        result = self.adapter(
            controller,
            planner,
            max_decisions=64,
            execute_provisional_detour=True,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "detour_motion_incomplete")
        self.assertEqual(controller.scan_count, 3)
        pass_scan_index = max(
            index
            for index, command in enumerate(controller.commands)
            if command == "scan_front_arc"
        )
        self.assertEqual(
            controller.commands[pass_scan_index + 1:],
            ["turn_right"],
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

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(raised.exception.code, "blast_side_search_blocked")
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

        for fault in ("close", "invalid", "body", "heading", "unsettled"):
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
                elif fault == "heading":
                    candidate["observation"]["imu"]["heading_deg"] = None
                else:
                    candidate["observation_settled"] = False
                self.assertFalse(blast_turn_slice_allows_continuation(
                    candidate,
                    allow_no_valid_distance_with_bounded_evidence=True,
                ))

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
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "no_safe_blast_action")
        self.assertEqual(controller.commands, ["turn_left"])

    def test_control_wins_during_final_detour_verification(self):
        for control, terminal_reason in (
            ("deadline", "episode_deadline_elapsed"),
            ("stop", "stopped"),
        ):
            with self.subTest(control=control):
                clock = [1_000]
                context, updates = episode_context()
                context.settings.max_episode_ms = 60_000
                controller = FullDetourController()
                planner = Planner([
                    decision(SCAN_FRONT_ARC),
                    decision(TURN_LEFT_90),
                ])
                adapter = self.adapter(
                    controller,
                    planner,
                    monotonic_ms=lambda: clock[0],
                    max_decisions=51,
                    execute_provisional_detour=True,
                )
                original = adapter._detour_scan_verified

                def verifying_with_control(**values):
                    verified = original(**values)
                    if values["role"] == "FINAL":
                        if control == "deadline":
                            clock[0] = 61_000
                        else:
                            context.stop_requested.set()
                    return verified

                adapter._detour_scan_verified = verifying_with_control

                result = adapter.run(context)

                self.assertFalse(result.completed)
                self.assertEqual(result.terminal_reason, terminal_reason)
                self.assertEqual(controller.scan_count, 4)
                self.assertEqual(controller.commands[-1], "scan_front_arc")
                self.assertEqual(len(planner.contexts), 2)
                self.assertTrue(any(
                    update.get("scan", {}).get("state") == "complete"
                    for update in updates
                ))

    def test_full_host_detour_is_symmetric_to_the_right(self):
        controller = FullDetourController()
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_RIGHT_90),
        ])

        result = self.adapter(
            controller,
            planner,
            max_decisions=64,
            execute_provisional_detour=True,
        ).run(episode_context()[0])

        self.assertTrue(result.completed)
        self.assertEqual(len(planner.contexts), 2)
        self.assertGreater(controller.commands.count("drive_forward"), 5)
        self.assertIn("turn_left", controller.commands)
        self.assertEqual(controller.commands[-1], "scan_front_arc")

    def test_short_second_view_refuses_detour_without_pass_motion(self):
        class ShortViewController(FakeScanController):
            def __init__(self):
                super().__init__(1_000)
                self.scan_count = 0

            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    self.scan_count += 1
                    if self.scan_count == 1:
                        scan = scan_result(center_distance_mm=310.0)
                        scan["rays"][1].update({
                            "distance_mm": 368.0,
                            "heading_deg": -23.322266,
                            "relative_heading_deg": -23.322266,
                        })
                        result["scan"] = scan
                return result

        controller = ShortViewController()
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        result = self.adapter(
            controller,
            planner,
            max_decisions=64,
            execute_provisional_detour=True,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "detour_route_unavailable")
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(controller.commands.count("scan_front_arc"), 2)
        self.assertEqual(controller.commands.count("drive_forward"), 9)

    def test_no_return_side_view_runs_one_bounded_target_reacquisition(self):
        class ReacquisitionController(FakeScanController):
            def __init__(
                self, resolved, block_after_side=False,
                selected_side="LEFT",
            ):
                super().__init__(1_000)
                self.scan_count = 0
                self.resolved = resolved
                self.block_after_side = block_after_side
                self.selected_side = selected_side

            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command != "scan_front_arc":
                    return result
                self.scan_count += 1
                if self.scan_count == 1:
                    scan = scan_result(center_distance_mm=245.0)
                    scan["rays"][1]["distance_mm"] = 286.0
                    scan["rays"][3]["distance_mm"] = 232.0
                elif self.scan_count == 2:
                    scan = scan_result(center_distance_mm=1_352.0)
                    outer = 1 if self.selected_side == "LEFT" else 3
                    target = (3, 4) if self.selected_side == "LEFT" else (1, 2)
                    scan["rays"][outer]["distance_mm"] = 1_420.0
                    for index in target:
                        scan["rays"][index].update({
                            "distance_mm": 2_000.0,
                            "range_state": RANGE_STATE_NO_VALID_DISTANCE,
                        })
                else:
                    scan = scan_result(center_distance_mm=900.0)
                    target = 3 if self.selected_side == "LEFT" else 1
                    if self.resolved:
                        scan["rays"][target]["distance_mm"] = 300.0
                    else:
                        indices = (
                            (3, 4) if self.selected_side == "LEFT"
                            else (1, 2)
                        )
                        for index in indices:
                            scan["rays"][index].update({
                                "distance_mm": 2_000.0,
                                "range_state": RANGE_STATE_NO_VALID_DISTANCE,
                            })
                result["scan"] = scan
                if self.scan_count == 2 and self.block_after_side:
                    result["observation"]["distance_mm"] = 2_000
                    self.snapshot_value["observation"][
                        "distance_mm"
                    ] = 2_000
                return result

        for resolved, terminal_reason in (
            (True, "target_reacquisition_observation_collected"),
            (False, "target_reacquisition_unresolved"),
        ):
            with self.subTest(resolved=resolved):
                controller = ReacquisitionController(resolved)
                planner = Planner([
                    decision(SCAN_FRONT_ARC),
                    decision(TURN_LEFT_90),
                ])
                context, updates = episode_context()

                result = self.adapter(
                    controller,
                    planner,
                    max_decisions=64,
                    execute_provisional_detour=True,
                ).run(context)

                self.assertFalse(result.completed)
                self.assertEqual(result.terminal_reason, terminal_reason)
                self.assertEqual(len(planner.contexts), 2)
                self.assertEqual(controller.scan_count, 3)
                second_scan = [
                    index for index, command in enumerate(controller.commands)
                    if command == "scan_front_arc"
                ][1]
                self.assertEqual(
                    controller.commands[second_scan + 1:],
                    ["turn_right"] * 4
                    + ["drive_forward"] * 6
                    + ["turn_left"] * 4
                    + ["scan_front_arc"],
                )
                evidence = next(
                    update["scan"]["multi_view_observations"][
                        "target_reacquisition"
                    ]
                    for update in reversed(updates)
                    if "target_reacquisition" in update.get(
                        "scan", {}
                    ).get("multi_view_observations", {})
                    and update["scan"]["multi_view_observations"][
                        "target_reacquisition"
                    ]["attempted"]
                )
                self.assertIs(evidence["resolved"], resolved)
                trace = self.adapter(
                    ReacquisitionController(resolved),
                    Planner([
                        decision(SCAN_FRONT_ARC),
                        decision(TURN_LEFT_90),
                    ]),
                    max_decisions=21,
                    execute_provisional_detour=True,
                )
                exact = trace.run(episode_context()[0])
                self.assertEqual(exact.terminal_reason, terminal_reason)
                planned = trace.spatial_map_provider.snapshot()[
                    "navigation_trace"
                ]["planned_leg"]
                self.assertLess(planned["waypoint"]["y_mm"], 150)
                self.assertGreater(planned["bind_pose"]["y_mm"], 300)

        controller = ReacquisitionController(True)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])
        result = self.adapter(
            controller,
            planner,
            max_decisions=20,
            execute_provisional_detour=True,
        ).run(episode_context()[0])
        self.assertEqual(
            result.terminal_reason,
            "target_reacquisition_budget_insufficient",
        )
        self.assertEqual(controller.scan_count, 2)
        self.assertEqual(controller.commands[-1], "scan_front_arc")

        controller = ReacquisitionController(
            True, selected_side="RIGHT",
        )
        result = self.adapter(
            controller,
            Planner([
                decision(SCAN_FRONT_ARC), decision(TURN_RIGHT_90),
            ]),
            max_decisions=64,
            execute_provisional_detour=True,
        ).run(episode_context()[0])
        self.assertEqual(
            result.terminal_reason,
            "target_reacquisition_observation_collected",
        )
        second_scan = [
            index for index, command in enumerate(controller.commands)
            if command == "scan_front_arc"
        ][1]
        tail = controller.commands[second_scan + 1:]
        self.assertEqual(tail[:4], ["turn_left"] * 4)
        self.assertEqual(tail[-5:], ["turn_right"] * 4 + ["scan_front_arc"])
        self.assertIn(tail.count("drive_forward"), range(1, 7))

        controller = ReacquisitionController(True, block_after_side=True)
        result = self.adapter(
            controller,
            Planner([
                decision(SCAN_FRONT_ARC), decision(TURN_LEFT_90),
            ]),
            max_decisions=64,
            execute_provisional_detour=True,
        ).run(episode_context()[0])
        self.assertEqual(
            result.terminal_reason, "target_reacquisition_blocked",
        )
        self.assertEqual(controller.scan_count, 2)
        self.assertEqual(controller.commands[-1], "scan_front_arc")

    def test_stop_after_side_rescan_wins_over_collected_outcome(self):
        context, updates = episode_context()

        class StopOnSecondScan(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if (
                    command == "scan_front_arc"
                    and self.commands.count("scan_front_arc") == 2
                ):
                    context.stop_requested.set()
                return result

        controller = StopOnSecondScan(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        result = self.adapter(
            controller,
            planner,
            max_decisions=9,
        ).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(result.terminal_reason, "stopped")
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(controller.commands.count("scan_front_arc"), 2)
        self.assertFalse(any(
            "multi_view_observations" in update.get("scan", {})
            for update in updates
        ))

    def test_side_turn_rechecks_live_safety_after_model_latency(self):
        controller = FakeScanController(500)

        class ObstacleMovesDuringPlanning(Planner):
            def decide(self, context):
                result = super().decide(context)
                if len(self.contexts) == 2:
                    controller.snapshot_value["observation"][
                        "distance_mm"
                    ] = 40
                return result

        planner = ObstacleMovesDuringPlanning([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(raised.exception.code, "blast_side_search_blocked")
        self.assertEqual(controller.commands, ["scan_front_arc"])
        self.assertEqual(len(planner.contexts), 2)

    def test_side_turn_remeasures_one_post_scan_no_return(self):
        class SettledRemeasureController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == SETTLED_OBSERVATION_COMMAND:
                    result["observation"]["distance_mm"] = 500
                    result["observation_settled"] = True
                    self.snapshot_value["observation"] = result[
                        "observation"
                    ]
                    self.snapshot_value[
                        "last_observed_at_monotonic_ms"
                    ] = 1_000
                return result

        controller = SettledRemeasureController(500)
        controller.snapshot_value["last_observed_at_monotonic_ms"] = 999

        class NoReturnDuringSideChoice(Planner):
            def decide(self, context):
                result = super().decide(context)
                if len(self.contexts) == 2:
                    controller.snapshot_value["observation"][
                        "distance_mm"
                    ] = 2_000
                return result

        planner = NoReturnDuringSideChoice([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        result = self.adapter(
            controller,
            planner,
            max_decisions=16,
        ).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason, "side_search_observation_collected"
        )
        self.assertEqual(controller.commands[0], "scan_front_arc")
        self.assertEqual(controller.commands[1], SETTLED_OBSERVATION_COMMAND)
        self.assertEqual(controller.commands[2:6], ["turn_left"] * 4)
        self.assertEqual(len(planner.contexts), 2)

    def test_side_search_waypoint_uses_pre_turn_pose_frame(self):
        pose = PhysicalPose(x_mm=100, y_mm=-50, heading_mdeg=90_000)

        left = _side_search_waypoint(pose, "LEFT")
        right = _side_search_waypoint(pose, "RIGHT")

        self.assertEqual(
            (left["target_x_mm"], left["target_y_mm"]),
            (-125, -50),
        )
        self.assertEqual(left["target_heading_mdeg"], -180_000)
        self.assertEqual(
            (right["target_x_mm"], right["target_y_mm"]),
            (325, -50),
        )
        self.assertEqual(right["target_heading_mdeg"], 0)
        self.assertEqual(left["scope"], "SEARCH_POSITION_ONLY")
        self.assertIs(left["clearance_proven"], False)
        self.assertEqual(left["origin_pose"], pose.to_dict())
        self.assertEqual(right["origin_pose"], pose.to_dict())

    def test_side_search_waypoint_expands_from_same_depth_echoes(self):
        pose = PhysicalPose()

        def view(points, *, value_pose=pose):
            return {
                "scan_pose": value_pose.to_dict(),
                "planar_projection": {
                    "schema": "blast-planar-scan-projection/v1",
                    "frame": "EPISODE_LOCAL_ODOMETRY",
                    "quality": "PROVISIONAL_YAW_ONLY",
                    "points": points,
                },
            }

        live_points = [
            {
                "side": "center",
                "nominal_echo_x_mm": 420,
                "nominal_echo_y_mm": 80,
            },
            {
                "side": "left_near",
                "nominal_echo_x_mm": 407,
                "nominal_echo_y_mm": 263,
            },
            {
                "side": "right_near",
                "nominal_echo_x_mm": 393,
                "nominal_echo_y_mm": -95,
            },
        ]
        left = _side_search_waypoint(
            pose, "LEFT", scan_view=view(live_points)
        )
        right = _side_search_waypoint(
            pose, "RIGHT", scan_view=view(live_points)
        )

        self.assertEqual(
            (left["target_x_mm"], left["target_y_mm"]),
            (0, 408),
        )
        self.assertEqual(
            (right["target_x_mm"], right["target_y_mm"]),
            (0, -245),
        )
        self.assertEqual(
            left["search_basis"],
            "PROVISIONAL_SAME_DEPTH_ECHO_REACH",
        )
        self.assertIs(left["search_target_capped"], False)
        self.assertIs(left["clearance_proven"], False)
        post_turn = _side_search_waypoint(
            pose,
            "LEFT",
            scan_view=view(live_points),
            outbound_pose=PhysicalPose(heading_mdeg=95_060),
        )
        self.assertEqual(
            (post_turn["target_x_mm"], post_turn["target_y_mm"]),
            (-36, 408),
        )
        translated_turn = _side_search_waypoint(
            pose,
            "LEFT",
            scan_view=view(live_points),
            outbound_pose=PhysicalPose(
                x_mm=2,
                y_mm=2,
                heading_mdeg=94_815,
            ),
        )
        self.assertEqual(
            (
                translated_turn["target_x_mm"],
                translated_turn["target_y_mm"],
            ),
            (-32, 408),
        )
        self.assertEqual(
            _side_search_followup_slots(
                PhysicalPose(x_mm=2, y_mm=2, heading_mdeg=94_815),
                translated_turn,
            ),
            11,
        )

        included = [live_points[0], {
            "side": "left_near",
            "nominal_echo_x_mm": 465,
            "nominal_echo_y_mm": 263,
        }]
        excluded = [live_points[0], {
            "side": "left_near",
            "nominal_echo_x_mm": 466,
            "nominal_echo_y_mm": 1_000,
        }]
        self.assertEqual(
            _side_search_waypoint(
                pose, "LEFT", scan_view=view(included)
            )["target_lateral_offset_mm"],
            408,
        )
        self.assertEqual(
            _side_search_waypoint(
                pose, "LEFT", scan_view=view(excluded)
            )["target_lateral_offset_mm"],
            225,
        )
        capped = [live_points[0], {
            "side": "left_near",
            "nominal_echo_x_mm": 420,
            "nominal_echo_y_mm": 1_000,
        }]
        capped_waypoint = _side_search_waypoint(
            pose, "LEFT", scan_view=view(capped)
        )
        self.assertEqual(capped_waypoint["target_lateral_offset_mm"], 450)
        self.assertIs(capped_waypoint["search_target_capped"], True)
        self.assertEqual(_side_search_required_slots(capped_waypoint), 13)
        center_only = _side_search_waypoint(
            pose, "LEFT", scan_view=view([live_points[0]])
        )
        self.assertEqual(center_only["target_lateral_offset_mm"], 225)
        self.assertEqual(center_only["search_basis"], "FOOTPRINT_MINIMUM")
        with self.assertRaises(ValueError):
            _side_search_waypoint(
                pose,
                "LEFT",
                scan_view=view(live_points, value_pose=PhysicalPose(x_mm=1)),
            )
        with self.assertRaises(ValueError):
            _side_search_waypoint(
                pose,
                "LEFT",
                scan_view=view(live_points),
                outbound_pose=PhysicalPose(heading_mdeg=70_000),
            )
        away_from_surface = _side_search_waypoint(
            pose,
            "LEFT",
            scan_view=view(live_points),
            outbound_pose=PhysicalPose(heading_mdeg=110_000),
        )
        self.assertLess(away_from_surface["target_x_mm"], 0)

        adapter = self.adapter(FakeScanController(), Planner([]))
        side, waypoint, outcome = adapter._complete_side_search(
            pose,
            TURN_LEFT_90,
            view(live_points),
            PhysicalPose(x_mm=2, y_mm=2, heading_mdeg=94_815),
            10,
        )
        self.assertIsNone(side)
        self.assertIsNone(waypoint)
        self.assertEqual(outcome.terminal_reason, "side_search_budget_insufficient")
        side, waypoint, outcome = adapter._complete_side_search(
            pose,
            TURN_LEFT_90,
            view(live_points),
            PhysicalPose(heading_mdeg=39_000),
            14,
        )
        self.assertIsNone(side)
        self.assertIsNone(waypoint)
        self.assertEqual(
            outcome.terminal_reason,
            "side_search_observation_unavailable",
        )

    def test_adaptive_side_search_is_invariant_in_rotated_origin_frame(self):
        pose = PhysicalPose(x_mm=100, y_mm=-50, heading_mdeg=90_000)
        scan_view = {
            "scan_pose": pose.to_dict(),
            "planar_projection": {
                "schema": "blast-planar-scan-projection/v1",
                "frame": "EPISODE_LOCAL_ODOMETRY",
                "quality": "PROVISIONAL_YAW_ONLY",
                "points": [
                    {
                        "side": "center",
                        "nominal_echo_x_mm": 20,
                        "nominal_echo_y_mm": 370,
                    },
                    {
                        "side": "left_near",
                        "nominal_echo_x_mm": -163,
                        "nominal_echo_y_mm": 357,
                    },
                ],
            },
        }

        waypoint = _side_search_waypoint(
            pose, "LEFT", scan_view=scan_view
        )

        self.assertEqual(
            (waypoint["target_x_mm"], waypoint["target_y_mm"]),
            (-308, -50),
        )
        self.assertEqual(waypoint["target_lateral_offset_mm"], 408)

    def test_side_search_requires_measured_rotation_clearance_at_waypoint(self):
        class NoRangeAtWaypointController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if self.commands.count("drive_forward") == 5:
                    result["observation"]["distance_mm"] = 2_000
                    self.snapshot_value["observation"] = result["observation"]
                return result

        controller = NoRangeAtWaypointController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
            *(decision("ADVANCE") for _index in range(5)),
        ])

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(raised.exception.code, "blast_side_search_blocked")
        self.assertNotIn("turn_right", controller.commands)
        self.assertEqual(len(planner.contexts), 2)

    def test_side_search_does_not_retry_incomplete_reorientation(self):
        class UndertravelController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "turn_right":
                    before = result["receipt"]["before_angles_deg"]
                    motors = result["observation"]["motor_angles_deg"]
                    motors["left_drive"] = before["left_drive"] + 10
                    motors["right_drive"] = before["right_drive"] - 10
                    self.snapshot_value["observation"] = result["observation"]
                return result

        controller = UndertravelController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
            *(decision("ADVANCE") for _index in range(5)),
            decision(TURN_RIGHT_90),
        ])

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(raised.exception.code, "blast_side_search_blocked")
        self.assertEqual(controller.commands.count("turn_right"), 4)
        self.assertNotIn("scan_front_arc", controller.commands[1:])

    def test_side_search_rechecks_range_after_reorientation(self):
        for distance in (2_000, None, 40):
            with self.subTest(distance=distance):
                class BlockedAfterTurnController(FakeScanController):
                    def command(self, command, *, cancel_requested=None):
                        result = super().command(
                            command,
                            cancel_requested=cancel_requested,
                        )
                        if self.commands.count("turn_right") == 4:
                            result["observation"]["distance_mm"] = distance
                            self.snapshot_value["observation"] = (
                                result["observation"]
                            )
                        return result

                controller = BlockedAfterTurnController(1_000)
                planner = Planner([
                    decision(SCAN_FRONT_ARC),
                    decision(TURN_LEFT_90),
                    *(decision("ADVANCE") for _index in range(5)),
                    decision(TURN_RIGHT_90),
                ])

                with self.assertRaises(BlastEpisodeError) as raised:
                    self.adapter(controller, planner).run(
                        episode_context()[0]
                    )

                self.assertEqual(
                    raised.exception.code,
                    "blast_side_search_blocked",
                )
                self.assertEqual(
                    controller.commands.count("scan_front_arc"),
                    1,
                )

    def test_host_reorient_no_return_finishes_then_scan_rechecks_range(self):
        class NoReturnDuringHostReorient(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command, cancel_requested=cancel_requested,
                )
                if command == "turn_right":
                    result["observation"]["distance_mm"] = 2_000
                    if self.commands.count("turn_right") == 4:
                        self.snapshot_value["observation"][
                            "distance_mm"
                        ] = 2_000
                return result

        controller = NoReturnDuringHostReorient(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
            *(decision("ADVANCE") for _index in range(5)),
            decision(TURN_RIGHT_90),
        ])

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(raised.exception.code, "blast_side_search_blocked")
        self.assertEqual(controller.commands.count("turn_right"), 4)
        self.assertEqual(controller.commands.count("scan_front_arc"), 1)

    def test_side_search_requires_projection_ready_second_scan(self):
        class UnsettledSecondScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if (
                    command == "scan_front_arc"
                    and self.commands.count("scan_front_arc") == 2
                ):
                    result["scan"]["all_observations_settled"] = False
                return result

        controller = UnsettledSecondScanController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
            *(decision("ADVANCE") for _index in range(5)),
            decision(TURN_RIGHT_90),
            decision(SCAN_FRONT_ARC),
        ])

        result = self.adapter(controller, planner).run(episode_context()[0])

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason,
            "side_search_observation_unavailable",
        )
        self.assertEqual(controller.commands.count("scan_front_arc"), 2)

    def test_unprojectable_current_scan_cannot_authorize_a_side_turn(self):
        class UnsettledScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    result["scan"]["all_observations_settled"] = False
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

    def test_side_search_progress_is_derived_from_pose_and_scan(self):
        for side, outbound_heading in (
            ("LEFT", 94_570),
            ("RIGHT", -94_570),
        ):
            with self.subTest(side=side):
                waypoint = _side_search_waypoint(PhysicalPose(), side)
                outbound = _side_search_progress(
                    PhysicalPose(heading_mdeg=outbound_heading),
                    waypoint,
                )
                reached_pose = PhysicalPose(
                    x_mm=waypoint["target_x_mm"],
                    y_mm=waypoint["target_y_mm"],
                    heading_mdeg=outbound_heading,
                )
                self.assertEqual(outbound["phase"], "OUTBOUND")
                self.assertEqual(outbound["required_action"], "ADVANCE")
                self.assertIsNone(_side_search_progress(
                    PhysicalPose(heading_mdeg=45_000),
                    waypoint,
                )["required_action"])
                if side == "LEFT":
                    toward_front_heading = 70_000
                    bounded_drift_heading = 85_961
                    away_from_front_heading = 110_000
                else:
                    toward_front_heading = -70_000
                    bounded_drift_heading = -85_961
                    away_from_front_heading = -110_000
                self.assertIsNone(_side_search_progress(
                    PhysicalPose(heading_mdeg=toward_front_heading),
                    waypoint,
                )["required_action"])
                self.assertEqual(
                    _side_search_progress(
                        PhysicalPose(heading_mdeg=bounded_drift_heading),
                        waypoint,
                    )["required_action"],
                    "ADVANCE",
                )
                self.assertEqual(
                    _side_search_progress(
                        PhysicalPose(heading_mdeg=away_from_front_heading),
                        waypoint,
                    )["required_action"],
                    "ADVANCE",
                )
                self.assertEqual(
                    _side_search_progress(reached_pose, waypoint)["phase"],
                    "REORIENT",
                )
                restored_pose = PhysicalPose(
                    x_mm=waypoint["target_x_mm"],
                    y_mm=waypoint["target_y_mm"],
                    heading_mdeg=waypoint["origin_pose"]["heading_mdeg"],
                )
                self.assertEqual(
                    _side_search_progress(
                        restored_pose,
                        waypoint,
                        reorientation_attempted=True,
                    )["phase"],
                    "RESCAN",
                )
                without_turn = _side_search_progress(
                    restored_pose,
                    waypoint,
                )
                self.assertEqual(without_turn["phase"], "REORIENT")
                self.assertIsNone(without_turn["required_action"])
                moved_after_turn = _side_search_progress(
                    PhysicalPose(
                        x_mm=restored_pose.x_mm + 100,
                        y_mm=restored_pose.y_mm,
                        heading_mdeg=restored_pose.heading_mdeg,
                    ),
                    waypoint,
                    reorientation_attempted=True,
                )
                self.assertEqual(moved_after_turn["phase"], "BLOCKED")
                self.assertIsNone(moved_after_turn["required_action"])
        left = _side_search_waypoint(PhysicalPose(), "LEFT")
        self.assertIsNone(_side_search_progress(
            PhysicalPose(y_mm=300, heading_mdeg=90_000),
            left,
        )["required_action"])

    def test_side_search_does_not_treat_close_or_no_valid_range_as_clear(self):
        for distance in (100, 2_000):
            with self.subTest(distance=distance):
                class RangeChangesAfterTurnController(FakeScanController):
                    def command(self, command, *, cancel_requested=None):
                        result = super().command(
                            command,
                            cancel_requested=cancel_requested,
                        )
                        if self.commands.count("turn_left") == 4:
                            result["observation"]["distance_mm"] = distance
                            self.snapshot_value["observation"][
                                "distance_mm"
                            ] = distance
                        return result

                controller = RangeChangesAfterTurnController(500)
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
                    "blast_side_search_blocked",
                )
                self.assertNotIn("drive_forward", controller.commands)
                self.assertEqual(len(planner.contexts), 2)

    def test_side_search_invalid_range_stops_before_another_action(self):
        class InvalidAfterTurnController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if self.commands.count("turn_left") == 4:
                    result["observation"]["distance_mm"] = None
                    self.snapshot_value["observation"]["distance_mm"] = None
                return result

        controller = InvalidAfterTurnController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
        ])

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(raised.exception.code, "blast_side_search_blocked")
        self.assertNotIn("drive_forward", controller.commands)
        self.assertEqual(len(planner.contexts), 2)

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

    def test_side_search_requires_imu_and_body_reference_correlation(self):
        for fault in ("imu", "body"):
            with self.subTest(fault=fault):
                class CorrelationFaultController(FakeScanController):
                    def command(self, command, *, cancel_requested=None):
                        result = super().command(
                            command,
                            cancel_requested=cancel_requested,
                        )
                        if self.commands.count("turn_left") == 4:
                            if fault == "imu":
                                result["observation"]["imu"][
                                    "heading_deg"
                                ] = 0
                            else:
                                result["observation"]["motor_angles_deg"][
                                    "body"
                                ] = 120
                            self.snapshot_value["observation"] = (
                                result["observation"]
                            )
                        return result

                controller = CorrelationFaultController(500)
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
                    "blast_side_search_blocked",
                )
                self.assertNotIn("drive_forward", controller.commands)
                self.assertEqual(len(planner.contexts), 2)

    def test_turn_without_current_scan_does_not_create_detour_intent(self):
        controller = FakeController(1_000)
        planner = Planner([
            decision(TURN_LEFT_90, plan=(TURN_LEFT_90,)),
            decision("ABORT"),
        ])

        self.adapter(controller, planner).run(episode_context()[0])

        self.assertNotIn(
            "navigation_intent",
            planner.contexts[1].observation,
        )

    def test_planned_later_turn_does_not_authorize_post_scan_advance(self):
        controller = FakeScanController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(
                "ADVANCE",
                plan=("ADVANCE", TURN_LEFT_90),
            ),
            decision("ABORT"),
        ])

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(episode_context()[0])

        self.assertEqual(raised.exception.code, "blast_planner_action_invalid")
        self.assertEqual(controller.commands, ["scan_front_arc"])
        self.assertNotIn(
            "navigation_intent",
            planner.contexts[1].observation,
        )

    def test_partial_scan_guided_turn_does_not_start_host_execution(self):
        controller = FakeScanController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(TURN_LEFT_90),
            decision("ABORT"),
        ])
        context, _updates = episode_context()

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

        context.stop_requested = OneShotPartialTurn()

        result = self.adapter(controller, planner).run(context)

        self.assertFalse(result.completed)
        self.assertEqual(
            result.terminal_reason,
            "side_search_motion_incomplete",
        )
        self.assertEqual(
            controller.commands,
            ["scan_front_arc", "turn_left", "turn_left"],
        )
        self.assertEqual(len(planner.contexts), 2)

    def test_unrestored_scan_stops_before_another_planner_turn(self):
        class UnrestoredScanController(FakeScanController):
            def command(self, command, *, cancel_requested=None):
                result = super().command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if command == "scan_front_arc":
                    result["scan"] = {
                        **result["scan"],
                        "restoration_verified": False,
                    }
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
        controller = FakeController(100)
        planner = Planner([decision(COMPLETE, assessment="Already close")])
        context, _updates = episode_context()

        self.adapter(controller, planner).run(context)

        self.assertNotIn(
            "ADVANCE",
            planner.contexts[0].available_actions,
        )
        self.assertNotIn("REVERSE", planner.contexts[0].available_actions)
        self.assertIn(
            SCAN_FRONT_ARC,
            planner.contexts[0].available_actions,
        )

    def test_scan_is_hidden_without_heading_or_distance(self):
        for missing in ("heading", "distance"):
            controller = FakeController()
            if missing == "heading":
                controller.snapshot_value["observation"]["imu"].pop(
                    "heading_deg"
                )
            else:
                controller.snapshot_value["observation"].pop("distance_mm")
            planner = Planner([decision(COMPLETE)])

            result = self.adapter(controller, planner).run(
                episode_context()[0]
            )

            with self.subTest(missing=missing):
                if missing == "heading":
                    self.assertNotIn(
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
                "motor_angles_deg": {"body": 158},
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
        for distance, body in ((2_000, 158), (53, 158), (500, 156)):
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
                self.assertEqual(controller.commands, [])
                self.assertEqual(planner.contexts, [])

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
            def command(self, command, *, cancel_requested=None):
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
            decision(COMPLETE, assessment="Scan collected."),
        ])

        result = self.adapter(controller, planner).run(episode_context()[0])

        self.assertTrue(result.completed)
        self.assertEqual(controller.commands, ["scan_front_arc"])
        self.assertEqual(len(planner.contexts), 2)

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

    def test_side_rescan_refusal_does_not_publish_multi_view(self):
        for cancelled, terminal_reason in (
            (False, "side_search_observation_unavailable"),
            (True, "stopped"),
        ):
            with self.subTest(cancelled=cancelled):
                context, updates = episode_context()

                class RefusingSecondScanController(FakeScanController):
                    def __init__(self):
                        super().__init__(500)
                        self.scan_attempts = 0

                    def command(self, command, *, cancel_requested=None):
                        if command == "scan_front_arc":
                            self.scan_attempts += 1
                            if self.scan_attempts == 2:
                                if cancelled:
                                    context.stop_requested.set()
                                raise BlastControllerError(
                                    "scan_start_clearance_unverified",
                                    "settled side scan start was unsafe",
                                )
                        return super().command(
                            command,
                            cancel_requested=cancel_requested,
                        )

                controller = RefusingSecondScanController()
                planner = Planner([
                    decision(SCAN_FRONT_ARC),
                    decision(TURN_LEFT_90),
                ])

                result = self.adapter(controller, planner).run(context)

                self.assertFalse(result.completed)
                self.assertEqual(result.terminal_reason, terminal_reason)
                self.assertEqual(controller.scan_attempts, 2)
                self.assertEqual(len(planner.contexts), 2)
                self.assertFalse(any(
                    "multi_view_observations" in update.get("scan", {})
                    for update in updates
                ))

    def test_scan_guided_motion_does_not_reuse_a_planned_completion(self):
        controller = FakeScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision(TURN_LEFT_90, plan=(TURN_LEFT_90,)),
            decision(COMPLETE, assessment="Unreachable invalid completion."),
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
            "side_search_budget_insufficient",
        )
        self.assertTrue(planner.contexts[0].completion_allowed)
        self.assertTrue(planner.contexts[1].completion_allowed)
        self.assertEqual(len(planner.contexts), 2)
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"],
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
