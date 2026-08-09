import threading
import unittest
from types import SimpleNamespace

from robot_agent.blast_episode_adapter import (
    ACTION_COMMANDS,
    BlastEpisodeError,
    BlastEpisodeRuntimeAdapter,
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
)
from robot_agent.lm_studio_controller_action import (
    COMPLETE,
    ControllerActionDecision,
    ControllerActionPlannerResult,
)
from robot_agent.physical_navigation_contract import (
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
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
    return {
        "schema": SCAN_RESULT_SCHEMA,
        "restoration_verified": True,
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
            }
            for side, distance_mm in distances
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
                23.765 if left else -23.765
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
        return BlastEpisodeRuntimeAdapter(
            controller=controller,
            planner_factory=lambda _model: planner,
            monotonic_ms=lambda: 1_000,
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
        self.assertEqual(
            [update["scan"] for update in updates if "scan" in update],
            [scan],
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
            planner.contexts[1].observation["navigation_reference"],
            {
                "episode_start_heading_deg": 179.0,
                "current_heading_deg": -179.0,
                "heading_error_deg": 2.0,
            },
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
                        ] = 159
                    else:
                        result["observation"]["motor_angles_deg"][
                            "body"
                        ] = 159
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
            decision(SCAN_FRONT_ARC),
            decision(COMPLETE, assessment="Fixture sequence ended."),
        ])

        result = self.adapter(controller, planner).run(episode_context()[0])

        self.assertTrue(result.completed)
        self.assertEqual(
            controller.commands,
            ["scan_front_arc"]
            + ["turn_left"] * 4
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
        self.assertEqual(
            planner.contexts[2].observation["odometry"]["heading_mdeg"],
            95_060,
        )
        self.assertEqual(
            planner.contexts[3].observation["odometry"],
            planner.contexts[2].observation["odometry"],
        )

    def test_scan_guided_turn_persists_one_detour_side(self):
        for action, side in (
            (TURN_LEFT_90, "LEFT"),
            ("TURN_RIGHT_90", "RIGHT"),
        ):
            with self.subTest(action=action):
                opposite = (
                    "TURN_RIGHT_90"
                    if action == TURN_LEFT_90
                    else TURN_LEFT_90
                )
                controller = FakeScanController(1_000)
                planner = Planner([
                    decision(SCAN_FRONT_ARC),
                    decision(action),
                    decision("ADVANCE"),
                    decision(opposite),
                    decision("ABORT"),
                ])

                result = self.adapter(controller, planner).run(
                    episode_context()[0]
                )

                self.assertEqual(result.terminal_reason, "planner_aborted")
                self.assertNotIn(
                    "navigation_intent",
                    planner.contexts[1].observation,
                )
                expected = {
                    "selected_detour_side_relative_to_scan": side,
                    "side_search_waypoint": {
                        "kind": "SIDE_SEARCH",
                        "scope": "SEARCH_POSITION_ONLY",
                        "clearance_proven": False,
                        "frame": "EPISODE_LOCAL_ODOMETRY",
                        "origin_pose": PhysicalPose().to_dict(),
                        "target_x_mm": 0,
                        "target_y_mm": 225 if side == "LEFT" else -225,
                        "target_heading_mdeg": (
                            90_000 if side == "LEFT" else -90_000
                        ),
                        "position_tolerance_mm": 35,
                    },
                }
                self.assertEqual(
                    planner.contexts[2].observation["navigation_intent"],
                    expected,
                )
                self.assertEqual(
                    planner.contexts[3].observation["navigation_intent"],
                    expected,
                )
                self.assertEqual(
                    planner.contexts[4].observation["navigation_intent"],
                    expected,
                )
                self.assertEqual(
                    controller.commands,
                    ["scan_front_arc"]
                    + list(ACTION_COMMANDS[action])
                    + ["drive_forward"]
                    + list(ACTION_COMMANDS[opposite]),
                )

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

    def test_planned_later_turn_does_not_create_detour_intent(self):
        controller = FakeScanController(1_000)
        planner = Planner([
            decision(SCAN_FRONT_ARC),
            decision(
                "ADVANCE",
                plan=("ADVANCE", TURN_LEFT_90),
            ),
            decision("ABORT"),
        ])

        self.adapter(controller, planner).run(episode_context()[0])

        self.assertNotIn(
            "navigation_intent",
            planner.contexts[2].observation,
        )

    def test_partial_scan_guided_turn_does_not_create_detour_intent(self):
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

        self.adapter(controller, planner).run(context)

        self.assertEqual(
            controller.commands,
            ["scan_front_arc", "turn_left", "turn_left"],
        )
        self.assertFalse(
            planner.contexts[2].history[1]["motion"][
                "command_completed"
            ]
        )
        self.assertNotIn(
            "navigation_intent",
            planner.contexts[2].observation,
        )

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

    def test_close_obstacle_removes_only_forward_action(self):
        controller = FakeController(100)
        planner = Planner([decision(COMPLETE, assessment="Already close")])
        context, _updates = episode_context()

        self.adapter(controller, planner).run(context)

        self.assertNotIn(
            "ADVANCE",
            planner.contexts[0].available_actions,
        )
        self.assertIn("REVERSE", planner.contexts[0].available_actions)
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

            self.adapter(controller, planner).run(episode_context()[0])

            with self.subTest(missing=missing):
                self.assertNotIn(
                    SCAN_FRONT_ARC,
                    planner.contexts[0].available_actions,
                )

    def test_motion_makes_scan_available_again(self):
        adapter = self.adapter(FakeController(), Planner([]))
        observation = {
            "sensors": {
                "distance_mm": 300,
                "imu": {"heading_deg": 0},
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

    def test_scan_guided_motion_requires_a_fresh_scan_before_completion(self):
        controller = FakeScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision(TURN_LEFT_90, plan=(TURN_LEFT_90,)),
            decision(COMPLETE, assessment="Unreachable invalid completion."),
        ])
        context, _updates = episode_context()

        with self.assertRaises(BlastEpisodeError) as raised:
            self.adapter(controller, planner).run(context)

        self.assertEqual(raised.exception.code, "blast_planner_action_invalid")
        self.assertTrue(planner.contexts[0].completion_allowed)
        self.assertTrue(planner.contexts[1].completion_allowed)
        self.assertFalse(planner.contexts[2].completion_allowed)
        self.assertIn(
            SCAN_FRONT_ARC,
            planner.contexts[2].available_actions,
        )

    def test_fresh_scan_allows_completion_after_scan_guided_motion(self):
        controller = FakeScanController(500)
        planner = Planner([
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision(TURN_LEFT_90, plan=(TURN_LEFT_90,)),
            decision(SCAN_FRONT_ARC, plan=(SCAN_FRONT_ARC,)),
            decision(COMPLETE, assessment="Fresh final scan confirms it."),
        ])
        context, _updates = episode_context()

        result = self.adapter(controller, planner).run(context)

        self.assertTrue(result.completed)
        self.assertEqual(
            controller.commands,
            [
                "scan_front_arc",
                "turn_left",
                "turn_left",
                "turn_left",
                "turn_left",
                "scan_front_arc",
            ],
        )
        self.assertTrue(planner.contexts[3].completion_allowed)

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
