import json
from pathlib import Path
import tempfile
from types import MappingProxyType
import threading
import time
import unittest

from robot_agent.contract import MotionCommand, MotorState, RobotState
from robot_agent.robot_api import (
    ActionContext,
    CapabilityGate,
    MotionRequest,
    RobotAPIContractError,
    RobotActionRejected,
    SimulatedRobotAPI,
    StopRequest,
)
from robot_agent.safety import SafetyLimits


CONFIG_PATH = Path(__file__).parents[1] / "config" / "ev3rstorm.json"


class MutableClock:
    def __init__(self, now_ms=10_000):
        self.now_ms = now_ms

    def __call__(self):
        return self.now_ms


class RobotAPITests(unittest.TestCase):
    def setUp(self):
        self.clock = MutableClock()
        self.api = SimulatedRobotAPI.from_config(
            CONFIG_PATH,
            self.clock,
            controller_instance_id="sim-instance-1",
        )

    def request(self, observation=None, **overrides):
        if observation is None:
            observation = self.api.observe()
        values = {
            "robot_id": observation.robot_id,
            "controller_id": observation.controller_id,
            "controller_instance_id": observation.controller_instance_id,
            "action_id": "action-1",
            "segment_id": "segment-1",
            "source_id": "test-agent",
            "host_clock_id": observation.host_clock_id,
            "based_on_state_version": observation.state_version,
            "based_on_received_at_host_ms": (
                observation.received_at_host_ms
            ),
            "issued_at_host_ms": self.clock(),
            "valid_until_host_ms": self.clock() + 500,
        }
        values.update(overrides.pop("context", {}))
        command = {
            "command_id": values["segment_id"],
            "motor_role": "arm",
            "speed_dps": 125,
            "duration_ms": 400,
            "issued_at_ms": values["issued_at_host_ms"],
        }
        command.update(overrides)
        return MotionRequest(
            context=ActionContext(**values),
            command=MotionCommand(**command),
        )

    def test_capabilities_are_explicit_and_drive_motors_are_not_exposed(self):
        capabilities = self.api.capabilities()

        self.assertTrue(capabilities.observe.executable)
        self.assertTrue(capabilities.emergency_stop.executable)
        self.assertEqual(
            tuple(motor.motor_role for motor in capabilities.motors),
            ("arm",),
        )
        self.assertEqual(
            capabilities.motor("arm").max_abs_speed_dps,
            150,
        )
        self.assertEqual(
            capabilities.motion_retry_semantics,
            "at_most_once",
        )
        self.assertEqual(
            capabilities.motion_execution_model,
            "accelerated_synchronous",
        )
        with self.assertRaises(RobotActionRejected):
            capabilities.motor("drive_b")

    def test_motion_is_snapshot_bound_and_encoder_verified(self):
        before = self.api.observe()
        receipt = self.api.execute_motion(self.request())
        after = self.api.observe()

        self.assertEqual(receipt.status, "completed")
        self.assertEqual(receipt.position_before, 0)
        self.assertEqual(receipt.position_after, 50)
        self.assertEqual(after.state_version, before.state_version + 1)
        self.assertEqual(
            after.state.motors["arm"].position_degrees,
            50,
        )

    def test_stale_future_and_long_ttl_requests_are_rejected(self):
        invalid_contexts = (
            {"based_on_state_version": 2},
            {"issued_at_host_ms": 10_001, "valid_until_host_ms": 10_100},
            {"issued_at_host_ms": 10_000, "valid_until_host_ms": 11_001},
        )
        expected = (
            "stale_state",
            "future_action",
            "ttl_limit",
        )
        for context, code in zip(invalid_contexts, expected):
            with self.subTest(code=code):
                with self.assertRaises(RobotActionRejected) as raised:
                    self.api.execute_motion(
                        self.request(context=context)
                    )
                self.assertEqual(raised.exception.code, code)
        self.assertEqual(
            self.api.observe().state.motors["arm"].position_degrees,
            0,
        )

    def test_expired_request_is_rejected(self):
        self.clock.now_ms = 9_999
        observation = self.api.observe()
        self.clock.now_ms = 10_000

        with self.assertRaises(RobotActionRejected) as raised:
            self.api.execute_motion(
                self.request(
                    observation=observation,
                    context={
                        "issued_at_host_ms": 9_999,
                        "valid_until_host_ms": 10_000,
                    },
                )
            )

        self.assertEqual(raised.exception.code, "stale_action")

    def test_old_observation_is_rejected_even_when_state_is_unchanged(self):
        observation = self.api.observe()
        self.clock.now_ms += 501

        with self.assertRaises(RobotActionRejected) as raised:
            self.api.execute_motion(self.request(observation=observation))

        self.assertEqual(raised.exception.code, "stale_observation")
        self.assertEqual(
            self.api.observe().state.motors["arm"].position_degrees,
            0,
        )

    def test_wrong_identity_and_command_context_never_execute(self):
        cases = (
            (
                self.request(context={"controller_id": "other"}),
                "identity_mismatch",
            ),
            (
                self.request(command_id="other-command"),
                "command_context_mismatch",
            ),
        )
        for request, code in cases:
            with self.subTest(code=code):
                with self.assertRaises(RobotActionRejected) as raised:
                    self.api.execute_motion(request)
                self.assertEqual(raised.exception.code, code)
        self.assertEqual(
            self.api.observe().state.motors["arm"].position_degrees,
            0,
        )

    def test_backend_rechecks_deadline_at_the_dispatch_boundary(self):
        request = self.request(
            context={"valid_until_host_ms": 10_001}
        )
        execute = self.api._robot.execute_motion

        def expire_before_backend_dispatch(command, valid_until_ms=None):
            self.clock.now_ms = valid_until_ms
            return execute(command, valid_until_ms=valid_until_ms)

        self.api._robot.execute_motion = expire_before_backend_dispatch
        with self.assertRaises(RobotActionRejected) as raised:
            self.api.execute_motion(request)

        self.assertEqual(raised.exception.code, "stale_action")
        self.assertEqual(
            self.api.observe().state.motors["arm"].position_degrees,
            0,
        )

    def test_check_execute_and_state_version_are_serialized(self):
        observation = self.api.observe()
        first = self.request(
            observation=observation,
            context={
                "action_id": "concurrent-action-1",
                "segment_id": "concurrent-segment-1",
            },
            speed_dps=100,
            duration_ms=100,
        )
        second = self.request(
            observation=observation,
            context={
                "action_id": "concurrent-action-2",
                "segment_id": "concurrent-segment-2",
            },
            speed_dps=100,
            duration_ms=100,
        )
        validate = self.api._robot.validate_motion
        first_entered = threading.Event()
        release_first = threading.Event()
        second_entered = threading.Event()
        calls = []
        calls_lock = threading.Lock()

        def gated_validate(command, now_ms):
            with calls_lock:
                calls.append(command.command_id)
                call_number = len(calls)
            if call_number == 1:
                first_entered.set()
                self.assertTrue(release_first.wait(timeout=1))
            else:
                second_entered.set()
            return validate(command, now_ms)

        self.api._robot.validate_motion = gated_validate
        receipts = []
        errors = []

        def execute(request):
            try:
                receipts.append(self.api.execute_motion(request))
            except RobotActionRejected as error:
                errors.append(error)

        first_thread = threading.Thread(target=execute, args=(first,))
        second_thread = threading.Thread(target=execute, args=(second,))
        first_thread.start()
        self.assertTrue(first_entered.wait(timeout=1))
        second_thread.start()
        time.sleep(0.02)
        self.assertFalse(second_entered.is_set())
        release_first.set()
        first_thread.join(timeout=1)
        second_thread.join(timeout=1)

        self.assertFalse(first_thread.is_alive())
        self.assertFalse(second_thread.is_alive())
        self.assertEqual(len(receipts), 1)
        self.assertEqual(len(errors), 1)
        self.assertEqual(errors[0].code, "stale_state")
        self.assertEqual(
            self.api.observe().state.motors["arm"].position_degrees,
            10,
        )

    def test_stop_receipt_is_a_fence_against_validated_motion(self):
        request = self.request(speed_dps=100, duration_ms=100)
        execute_motion = self.api._robot.execute_motion
        motion_entered = threading.Event()
        release_motion = threading.Event()

        def gated_execute(command, valid_until_ms=None):
            motion_entered.set()
            self.assertTrue(release_motion.wait(timeout=1))
            return execute_motion(
                command,
                valid_until_ms=valid_until_ms,
            )

        self.api._robot.execute_motion = gated_execute
        motion_receipts = []
        stop_receipts = []
        motion_thread = threading.Thread(
            target=lambda: motion_receipts.append(
                self.api.execute_motion(request)
            )
        )
        stop_request = StopRequest(
            robot_id="ev3rstorm-01",
            controller_id="ev3rstorm-01.ev3-main",
            controller_instance_id="sim-instance-1",
            action_id="concurrent-stop",
            segment_id="concurrent-stop",
            source_id="test-agent",
        )
        stop_thread = threading.Thread(
            target=lambda: stop_receipts.append(
                self.api.stop_all(stop_request)
            )
        )

        motion_thread.start()
        self.assertTrue(motion_entered.wait(timeout=1))
        stop_thread.start()
        time.sleep(0.02)
        self.assertEqual(stop_receipts, [])
        release_motion.set()
        motion_thread.join(timeout=1)
        stop_thread.join(timeout=1)

        self.assertEqual(len(motion_receipts), 1)
        self.assertEqual(len(stop_receipts), 1)
        self.assertEqual(
            stop_receipts[0].based_on_state_version,
            motion_receipts[0].resulting_state_version,
        )
        self.assertGreater(
            stop_receipts[0].resulting_state_version,
            stop_receipts[0].based_on_state_version,
        )
        self.assertEqual(
            self.api.observe().state.motors["arm"].position_degrees,
            10,
        )

    def test_duplicate_segment_is_not_retried(self):
        request = self.request()
        self.api.execute_motion(request)
        with self.assertRaises(RobotActionRejected) as raised:
            self.api.execute_motion(request)
        self.assertEqual(raised.exception.code, "stale_state")
        self.assertEqual(
            self.api.observe().state.motors["arm"].position_degrees,
            50,
        )

        current = self.api.observe()
        replay = self.request(
            context={
                "based_on_state_version": current.state_version,
            }
        )
        with self.assertRaises(RobotActionRejected) as replayed:
            self.api.execute_motion(replay)
        self.assertEqual(replayed.exception.code, "duplicate_segment")

    def test_stop_uses_exact_controller_but_no_snapshot_deadline(self):
        request = StopRequest(
            robot_id="ev3rstorm-01",
            controller_id="ev3rstorm-01.ev3-main",
            controller_instance_id="sim-instance-1",
            action_id="stop-action",
            segment_id="stop-segment",
            source_id="test-agent",
        )
        receipt = self.api.stop_all(request)
        self.assertEqual(receipt.status, "stopped")

        repeated = self.api.stop_all(request)
        self.assertEqual(repeated.status, "stopped")
        self.assertEqual(
            repeated.based_on_state_version,
            receipt.resulting_state_version,
        )

        with self.assertRaises(RobotActionRejected) as raised:
            self.api.stop_all(
                StopRequest(
                    robot_id="ev3rstorm-01",
                    controller_id="other",
                    controller_instance_id="sim-instance-1",
                    action_id="wrong-stop",
                    segment_id="wrong-stop-segment",
                    source_id="test-agent",
                )
            )
        self.assertEqual(raised.exception.code, "identity_mismatch")

        with self.assertRaises(RobotActionRejected) as restarted:
            self.api.stop_all(
                StopRequest(
                    robot_id="ev3rstorm-01",
                    controller_id="ev3rstorm-01.ev3-main",
                    controller_instance_id="other-instance",
                    action_id="wrong-instance-stop",
                    segment_id="wrong-instance-stop-segment",
                    source_id="test-agent",
                )
            )
        self.assertEqual(restarted.exception.code, "identity_mismatch")

    def test_sensor_mutation_mints_a_new_state_version(self):
        before = self.api.observe()
        self.api.set_sensor("touch", True)
        after = self.api.observe()
        self.assertEqual(after.state_version, before.state_version + 1)
        self.assertTrue(after.state.sensors["touch"])

    def test_capability_flags_and_bool_integers_fail_closed(self):
        with self.assertRaises(RobotAPIContractError):
            CapabilityGate(supported=False, enabled=True, available=False)
        with self.assertRaises(RobotAPIContractError):
            ActionContext(
                robot_id="robot",
                controller_id="controller",
                controller_instance_id="instance",
                action_id="action",
                segment_id="segment",
                source_id="source",
                host_clock_id="clock",
                based_on_state_version=True,
                based_on_received_at_host_ms=0,
                issued_at_host_ms=0,
                valid_until_host_ms=1,
            )

    def test_default_controller_instances_are_unique(self):
        first = SimulatedRobotAPI.from_config(CONFIG_PATH, self.clock)
        second = SimulatedRobotAPI.from_config(CONFIG_PATH, self.clock)

        self.assertNotEqual(
            first.capabilities().controller_instance_id,
            second.capabilities().controller_instance_id,
        )

    def test_action_cannot_predate_its_observation(self):
        with self.assertRaises(RobotAPIContractError) as raised:
            self.request(
                context={
                    "issued_at_host_ms": 9_999,
                    "valid_until_host_ms": 10_001,
                }
            )
        self.assertEqual(raised.exception.code, "invalid_causality")

    def test_snapshots_are_deeply_immutable_at_container_boundaries(self):
        observation = self.api.observe()

        self.assertIsInstance(observation.state.motors, MappingProxyType)
        self.assertIsInstance(observation.state.sensors, MappingProxyType)
        with self.assertRaises(TypeError):
            observation.state.sensors["touch"] = True
        with self.assertRaises(TypeError):
            observation.state.motors["arm"] = None

    def test_malformed_nested_motor_state_is_rejected(self):
        with self.assertRaises(ValueError):
            MotorState(
                role="arm",
                port="outA",
                position_degrees=True,
                running=False,
            )
        with self.assertRaises(ValueError):
            MotorState(
                role="arm",
                port="outA",
                position_degrees=0,
                running="",
            )
        with self.assertRaises(ValueError):
            RobotState(
                observed_at_ms=10_000,
                motors={
                    "wrong-key": MotorState(
                        role="arm",
                        port="outA",
                        position_degrees=0,
                    )
                },
            )

    def test_auxiliary_motor_requires_explicit_allowlist(self):
        with CONFIG_PATH.open("r", encoding="utf-8") as handle:
            config = json.load(handle)
        config["motors"]["future_aux"] = dict(config["motors"]["arm"])
        config["motors"]["future_aux"]["port"] = "outD"

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            api = SimulatedRobotAPI.from_config(path, self.clock)

        self.assertEqual(
            tuple(
                motor.motor_role
                for motor in api.capabilities().motors
            ),
            ("arm",),
        )

    def test_role_limits_come_from_geometry_not_role_name(self):
        with CONFIG_PATH.open("r", encoding="utf-8") as handle:
            config = json.load(handle)
        config["motors"] = {
            "left_leg": config["motors"]["drive_b"],
            "right_leg": config["motors"]["drive_c"],
            "drive_named_arm": config["motors"]["arm"],
        }
        config["drive_geometry"]["left_motor_role"] = "left_leg"
        config["drive_geometry"]["right_motor_role"] = "right_leg"
        config["drive_geometry"]["forward_speed_sign"] = {
            "left_leg": 1,
            "right_leg": 1,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            limits = SafetyLimits.from_file(path)

        self.assertIs(limits.limit_for_role("left_leg"), limits.drive)
        self.assertIs(limits.limit_for_role("right_leg"), limits.drive)
        self.assertIs(
            limits.limit_for_role("drive_named_arm"),
            limits.arm,
        )


if __name__ == "__main__":
    unittest.main()
