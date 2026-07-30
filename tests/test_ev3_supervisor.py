import fcntl
import json
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import ev3.supervisor_daemon as supervisor_daemon_module
import ev3.supervisor as supervisor_module
from ev3.robot_hal import MotorBusyError, RobotHAL, SafetyError, read_text
from ev3.supervisor import (
    AuditBuffer,
    EV3Supervisor,
    EV3SupervisorLoop,
    STATE_ARMED_IDLE,
    STATE_CLOSED,
    STATE_DISARMED,
    STATE_FAULT_LATCHED,
    STATE_RUNNING,
    SupervisorError,
)
from ev3.supervisor_protocol import (
    PROTOCOL_VERSION,
    SupervisorProtocol,
    decode_request,
)
from ev3.supervisor_daemon import (
    ForegroundSupervisorSession,
    SessionError,
    build_parser as build_daemon_parser,
    run_daemon,
)


PROJECT_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "ev3rstorm.json"


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(value), encoding="ascii")


class FakeSysfs:
    def __init__(self, root):
        self.root = Path(root)
        self.motors = {}
        self.sensors = {}

    def add_motor(self, name, port, driver, position=0):
        path = self.root / "tacho-motor" / name
        values = {
            "address": "ev3-ports:{}".format(port),
            "driver_name": driver,
            "position": position,
            "state": "",
            "max_speed": 1050,
            "speed_sp": 0,
            "time_sp": 0,
            "stop_action": "coast",
            "command": "",
        }
        for filename, value in values.items():
            write(path / filename, value)
        self.motors[port] = path

    def add_sensor(self, name, port, driver, mode, value, units=""):
        path = self.root / "lego-sensor" / name
        values = {
            "address": "ev3-ports:{}".format(port),
            "driver_name": driver,
            "mode": mode,
            "value0": value,
            "units": units,
        }
        for filename, item in values.items():
            write(path / filename, item)
        self.sensors[port] = path


class FakeClock:
    def __init__(self, now_ms=10_000):
        self.now_ms = now_ms

    def monotonic(self):
        return self.now_ms / 1000.0

    def sleep(self, seconds):
        self.now_ms += int(round(seconds * 1000))

    def advance(self, milliseconds):
        self.now_ms += milliseconds


class InteractiveInput:
    def __init__(self):
        self.items = queue.Queue()

    def readline(self, _maximum):
        return self.items.get(timeout=2)

    def send(self, value):
        self.items.put(value)

    def close(self):
        self.items.put(b"")


class InteractiveOutput:
    def __init__(self, fail_writes=False):
        self.items = queue.Queue()
        self.fail_writes = fail_writes

    def write(self, value):
        if self.fail_writes:
            raise IOError("simulated output failure")
        self.items.put(value)

    def flush(self):
        return None

    def receive(self):
        return json.loads(
            self.items.get(timeout=2).decode("utf-8")
        )


class EV3SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.sysfs = FakeSysfs(Path(self.temp.name) / "class")
        self.sysfs.add_motor(
            "motor0",
            "outA",
            "lego-ev3-m-motor",
            position=100,
        )
        self.sysfs.add_motor(
            "motor1",
            "outC",
            "lego-ev3-l-motor",
        )
        self.sysfs.add_motor(
            "motor2",
            "outB",
            "lego-ev3-l-motor",
        )
        self.sysfs.add_sensor(
            "sensor0",
            "in1",
            "lego-ev3-touch",
            "TOUCH",
            0,
        )
        self.sysfs.add_sensor(
            "sensor1",
            "in4",
            "lego-ev3-ir",
            "IR-PROX",
            50,
            "pct",
        )
        self.sysfs.add_sensor(
            "sensor2",
            "in3",
            "lego-ev3-color",
            "COL-REFLECT",
            4,
            "pct",
        )
        self.clock = FakeClock()
        self.lock_path = str(Path(self.temp.name) / "motor.lock")
        self.hal = RobotHAL(
            str(CONFIG_PATH),
            sysfs_root=str(self.sysfs.root),
            lock_path=self.lock_path,
            sleep_fn=self.clock.sleep,
            monotonic_fn=self.clock.monotonic,
            speech_lock_path=str(Path(self.temp.name) / "audio.lock"),
        )
        real_supervisor_write = supervisor_module.write_text
        self.motor_writes = []

        def simulate_kernel_state(path, value):
            self.motor_writes.append((path, value))
            real_supervisor_write(path, value)
            target = Path(path)
            if target.name == "command":
                state = (
                    "running"
                    if value == "run-timed"
                    else ""
                )
                real_supervisor_write(
                    str(target.parent / "state"),
                    state,
                )

        self.write_patcher = patch.object(
            supervisor_module,
            "write_text",
            side_effect=simulate_kernel_state,
        )
        self.mock_supervisor_write = self.write_patcher.start()
        self.session_number = 0

        def session_id():
            self.session_number += 1
            return "session-{}".format(self.session_number)

        self.supervisor = EV3Supervisor(
            self.hal,
            session_id_factory=session_id,
        )

    def tearDown(self):
        try:
            self.supervisor.close()
        finally:
            try:
                self.write_patcher.stop()
            finally:
                self.temp.cleanup()

    def release_touch_samples(self, count=3):
        for _ in range(count):
            self.supervisor.poll_once()

    def claim_and_arm(self):
        self.release_touch_samples()
        claimed = self.supervisor.claim("host-agent")
        session_id = claimed["session_id"]
        self.supervisor.heartbeat(session_id, 1)
        self.supervisor.arm(session_id, 2)
        self.assertEqual(self.supervisor.state, STATE_ARMED_IDLE)
        return session_id

    def start_drive(
        self,
        session_id,
        sequence_id=3,
        command_id="drive-1",
        heartbeat_sequence=1,
        left_speed=100,
        right_speed=100,
        duration_ms=300,
    ):
        return self.supervisor.start_drive(
            session_id=session_id,
            sequence_id=sequence_id,
            command_id=command_id,
            reference_heartbeat_sequence=heartbeat_sequence,
            left_speed_dps=left_speed,
            right_speed_dps=right_speed,
            duration_ms=duration_ms,
        )

    def move_drive_encoders(self, left_delta, right_delta):
        left_path = self.sysfs.motors["outB"]
        right_path = self.sysfs.motors["outC"]
        left_before = int(read_text(str(left_path / "position")))
        right_before = int(read_text(str(right_path / "position")))
        write(left_path / "position", left_before + left_delta)
        write(right_path / "position", right_before + right_delta)

    def assert_all_stopped(self):
        for motor in self.sysfs.motors.values():
            self.assertEqual(
                read_text(str(motor / "command")),
                "stop",
            )

    def run_timed_write_count(self):
        return sum(
            1
            for _, value in self.motor_writes
            if value == "run-timed"
        )

    def protocol_request(
        self,
        operation,
        arguments=None,
        request_id="request-1",
        controller_id="ev3rstorm-01.ev3-main",
        ttl_ms=250,
        received_at_ms=None,
    ):
        if received_at_ms is None:
            received_at_ms = self.clock.now_ms
        value = {
            "protocol_version": PROTOCOL_VERSION,
            "controller_id": controller_id,
            "request_id": request_id,
            "op": operation,
            "queue_ttl_ms": ttl_ms,
            "args": {} if arguments is None else arguments,
        }
        return decode_request(
            json.dumps(value).encode("utf-8") + b"\n",
            received_at_ms,
        )

    def protocol_wire(
        self,
        operation,
        arguments=None,
        request_id="request-1",
        controller_id="ev3rstorm-01.ev3-main",
        ttl_ms=250,
    ):
        value = {
            "protocol_version": PROTOCOL_VERSION,
            "controller_id": controller_id,
            "request_id": request_id,
            "op": operation,
            "queue_ttl_ms": ttl_ms,
            "args": {} if arguments is None else arguments,
        }
        return json.dumps(value).encode("utf-8") + b"\n"

    def run_interactive_session(
        self,
        client,
        allow_motion=False,
        motion_budget=0,
        output=None,
    ):
        protocol = SupervisorProtocol(
            self.supervisor,
            "ev3rstorm-01.ev3-main",
            allow_motion=allow_motion,
            motion_budget=motion_budget,
            experiment_max_abs_speed_dps=100,
            experiment_max_duration_ms=300,
        )
        input_stream = InteractiveInput()
        if output is None:
            output = InteractiveOutput()
        client_errors = []

        def run_client():
            try:
                client(input_stream, output)
            except BaseException as error:
                client_errors.append(error)
                input_stream.close()

        client_thread = threading.Thread(target=run_client)
        client_thread.start()
        session = ForegroundSupervisorSession(
            self.supervisor,
            protocol,
            input_stream,
            output,
            max_session_ms=120000,
        )
        result = session.run()
        client_thread.join(2)
        self.assertFalse(client_thread.is_alive())
        if client_errors:
            raise client_errors[0]
        return result

    def test_startup_is_disarmed_stopped_and_fail_closed(self):
        status = self.supervisor.status()

        self.assertEqual(status["state"], STATE_DISARMED)
        self.assertFalse(status["motion_allowed"])
        self.assertFalse(status["session_active"])
        self.assert_all_stopped()
        self.assertEqual(
            self.supervisor.audit_events[0]["event"],
            "startup_complete",
        )

    def test_lifetime_lock_blocks_direct_hal_motion(self):
        with self.assertRaises(MotorBusyError):
            self.hal.drive_timed(100, 100, 300)

        self.assert_all_stopped()

    def test_second_supervisor_cannot_take_motor_ownership(self):
        with self.assertRaises(MotorBusyError):
            EV3Supervisor(
                self.hal,
                session_id_factory=lambda: "other-session",
            )
        self.assert_all_stopped()

    def test_claim_is_exclusive_and_old_session_is_rejected(self):
        first = self.supervisor.claim("owner-a")
        with self.assertRaises(SupervisorError) as context:
            self.supervisor.claim("owner-b")
        self.assertEqual(context.exception.code, "owner_exists")

        self.supervisor.heartbeat(first["session_id"], 1)
        self.supervisor.release(first["session_id"], 2)
        second = self.supervisor.claim("owner-b")
        self.assertNotEqual(first["session_id"], second["session_id"])

        with self.assertRaises(SupervisorError) as context:
            self.supervisor.heartbeat(first["session_id"], 3)
        self.assertEqual(context.exception.code, "wrong_session")

    def test_sequence_must_be_positive_integer_and_strictly_increase(self):
        session_id = self.supervisor.claim("owner")["session_id"]
        for invalid in (True, 0, -1, 1.5, "1"):
            with self.subTest(value=invalid):
                with self.assertRaises(SupervisorError):
                    self.supervisor.heartbeat(session_id, invalid)

        self.supervisor.heartbeat(session_id, 4)
        starts_before_duplicate = self.run_timed_write_count()
        with self.assertRaises(SupervisorError) as context:
            self.supervisor.heartbeat(session_id, 4)
        self.assertEqual(context.exception.code, "replayed_sequence")
        with self.assertRaises(SupervisorError):
            self.supervisor.heartbeat(session_id, 3)
        self.assertEqual(
            self.supervisor.last_heartbeat_sequence,
            4,
        )

    def test_replayed_sequence_does_not_refresh_heartbeat(self):
        session_id = self.claim_and_arm()
        original_heartbeat_ms = self.supervisor.last_heartbeat_ms
        self.clock.advance(400)

        with self.assertRaises(SupervisorError):
            self.supervisor.heartbeat(session_id, 1)

        self.assertEqual(
            self.supervisor.last_heartbeat_ms,
            original_heartbeat_ms,
        )
        self.clock.advance(100)
        self.supervisor.poll_once()
        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        self.assertEqual(
            self.supervisor.fault["code"],
            "heartbeat_timeout",
        )

    def test_arm_requires_explicit_heartbeat_and_stable_touch_release(self):
        session_id = self.supervisor.claim("owner")["session_id"]
        with self.assertRaises(SupervisorError) as context:
            self.supervisor.arm(session_id, 1)
        self.assertEqual(context.exception.code, "heartbeat_required")

        self.supervisor.heartbeat(session_id, 2)
        with self.assertRaises(SupervisorError) as context:
            self.supervisor.arm(session_id, 3)
        self.assertEqual(context.exception.code, "touch_not_released")

        self.release_touch_samples(2)
        self.supervisor.arm(session_id, 4)
        self.assertEqual(self.supervisor.state, STATE_ARMED_IDLE)

    def test_heartbeat_expires_at_exact_500_ms_boundary(self):
        self.claim_and_arm()
        self.clock.advance(499)
        self.supervisor.poll_once()
        self.assertEqual(self.supervisor.state, STATE_ARMED_IDLE)

        self.clock.advance(1)
        self.supervisor.poll_once()
        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        self.assertEqual(
            self.supervisor.fault["code"],
            "heartbeat_timeout",
        )
        self.assert_all_stopped()

    def test_motion_command_does_not_refresh_heartbeat(self):
        session_id = self.claim_and_arm()
        heartbeat_ms = self.supervisor.last_heartbeat_ms
        self.start_drive(session_id)

        self.assertEqual(
            self.supervisor.last_heartbeat_ms,
            heartbeat_ms,
        )

    def test_supervised_drive_uses_only_bounded_run_timed(self):
        session_id = self.claim_and_arm()
        status = self.start_drive(
            session_id,
            left_speed=200,
            right_speed=-200,
            duration_ms=800,
        )

        self.assertEqual(status["state"], STATE_RUNNING)
        left = self.sysfs.motors["outB"]
        right = self.sysfs.motors["outC"]
        self.assertEqual(read_text(str(left / "speed_sp")), "200")
        self.assertEqual(read_text(str(right / "speed_sp")), "-200")
        self.assertEqual(read_text(str(left / "time_sp")), "800")
        self.assertEqual(read_text(str(right / "time_sp")), "800")
        self.assertEqual(read_text(str(left / "stop_action")), "brake")
        self.assertEqual(read_text(str(right / "stop_action")), "brake")
        self.assertEqual(read_text(str(left / "command")), "run-timed")
        self.assertEqual(read_text(str(right / "command")), "run-timed")

    def test_speed_floor_and_existing_limits_reject_before_writes(self):
        session_id = self.claim_and_arm()
        left = self.sysfs.motors["outB"]
        right = self.sysfs.motors["outC"]

        with self.assertRaises(SupervisorError) as context:
            self.start_drive(session_id, left_speed=49)
        self.assertEqual(context.exception.code, "drive_speed_floor")
        self.assertEqual(read_text(str(left / "speed_sp")), "0")
        self.assertEqual(read_text(str(right / "speed_sp")), "0")

        with self.assertRaises(SupervisorError):
            self.start_drive(
                session_id,
                sequence_id=4,
                left_speed=251,
            )
        self.assertEqual(self.supervisor.state, STATE_ARMED_IDLE)
        self.assertEqual(read_text(str(left / "speed_sp")), "0")
        self.assertEqual(read_text(str(right / "speed_sp")), "0")
        accepted = [
            event
            for event in self.supervisor.audit_events
            if event["event"] == "command_accepted"
        ]
        self.assertEqual(accepted, [])

    def test_touch_pressed_before_start_faults_without_starting(self):
        session_id = self.claim_and_arm()
        write(self.sysfs.sensors["in1"] / "value0", 1)

        with self.assertRaises(SupervisorError):
            self.start_drive(session_id)

        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        self.assertEqual(
            self.supervisor.fault["code"],
            "touch_pressed",
        )
        self.assertEqual(
            read_text(str(self.sysfs.motors["outB"] / "speed_sp")),
            "0",
        )
        self.assert_all_stopped()

    def test_touch_transition_during_motion_stops_same_poll(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        write(self.sysfs.sensors["in1"] / "value0", 1)

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        self.assertEqual(
            self.supervisor.fault["code"],
            "touch_pressed",
        )
        self.assert_all_stopped()

    def test_invalid_touch_value_fails_closed(self):
        self.claim_and_arm()
        write(self.sysfs.sensors["in1"] / "value0", 2)

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        self.assertEqual(
            self.supervisor.fault["code"],
            "invalid_touch",
        )
        self.assert_all_stopped()

    def test_unknown_motor_state_token_fails_closed(self):
        write(self.sysfs.motors["outA"] / "state", "mystery")

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "invalid_motor_state",
        )
        self.assert_all_stopped()

    def test_holding_motor_is_not_a_verified_stop(self):
        write(self.sysfs.motors["outA"] / "state", "holding")

        def write_without_kernel_state_change(path, value):
            target = Path(path)
            write(target, value)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=write_without_kernel_state_change,
        ):
            result = self.supervisor._owner.stop_all_verified()

        self.assertFalse(result["stop_confirmed"])
        self.assertIn(
            str(self.sysfs.motors["outA"]),
            result["states"],
        )

    def test_heartbeat_expiry_during_motion_stops_same_poll(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        self.clock.advance(500)

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "heartbeat_timeout",
        )
        self.assert_all_stopped()

    def test_motion_completes_only_after_stop_and_encoder_checks(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        self.move_drive_encoders(30, 30)
        self.clock.advance(300)

        status = self.supervisor.poll_once()

        self.assertEqual(status["state"], STATE_ARMED_IDLE)
        self.assertIsNone(status["active_command_id"])
        self.assert_all_stopped()
        events = [
            event["event"]
            for event in self.supervisor.audit_events
        ]
        self.assertIn("motion_completed", events)

    def test_static_encoders_fault_at_stall_boundary(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        self.clock.advance(299)
        self.supervisor.poll_once()
        self.assertEqual(self.supervisor.state, STATE_RUNNING)

        self.clock.advance(1)
        self.supervisor.poll_once()
        self.assertEqual(
            self.supervisor.fault["code"],
            "encoder_stall",
        )
        self.assert_all_stopped()

    def test_one_sided_stall_stops_both_drive_motors(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        self.move_drive_encoders(30, 0)
        self.clock.advance(300)

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "encoder_stall",
        )
        self.assert_all_stopped()

    def test_wrong_encoder_direction_faults_immediately(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        self.move_drive_encoders(-3, 3)
        self.clock.advance(20)

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "encoder_direction",
        )
        self.assert_all_stopped()

    def test_duplicate_command_id_with_new_sequence_never_restarts(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, command_id="same")
        self.move_drive_encoders(30, 30)
        self.clock.advance(300)
        self.supervisor.poll_once()
        self.supervisor.heartbeat(session_id, 4)
        starts_before_duplicate = self.run_timed_write_count()

        with self.assertRaises(SupervisorError) as context:
            self.start_drive(
                session_id,
                sequence_id=5,
                command_id="same",
                heartbeat_sequence=4,
            )

        self.assertEqual(
            context.exception.code,
            "duplicate_command_id",
        )
        self.assertEqual(
            self.run_timed_write_count(),
            starts_before_duplicate,
        )
        self.assert_all_stopped()

    def test_partial_second_motor_start_failure_stops_every_motor(self):
        session_id = self.claim_and_arm()
        right = self.sysfs.motors["outC"]
        original_write = supervisor_module.write_text

        def fail_right_start(path, value):
            if (
                path == str(right / "command")
                and value == "run-timed"
            ):
                raise IOError("injected second start failure")
            original_write(path, value)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=fail_right_start,
        ):
            with self.assertRaises(SupervisorError) as context:
                self.start_drive(session_id)

        self.assertEqual(
            context.exception.code,
            "motion_start_failed",
        )
        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        start_events = [
            (path, value)
            for path, value in self.motor_writes
            if value == "run-timed"
        ]
        self.assertEqual(
            start_events,
            [(str(self.sysfs.motors["outB"] / "command"), "run-timed")],
        )
        self.assert_all_stopped()

    def test_keyboard_interrupt_on_second_start_stops_every_motor(self):
        session_id = self.claim_and_arm()
        right = self.sysfs.motors["outC"]
        original_write = supervisor_module.write_text

        def interrupt_right_start(path, value):
            if (
                path == str(right / "command")
                and value == "run-timed"
            ):
                raise KeyboardInterrupt()
            original_write(path, value)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=interrupt_right_start,
        ):
            with self.assertRaises(KeyboardInterrupt):
                self.start_drive(session_id)

        self.assertEqual(
            self.supervisor.fault["code"],
            "motion_start_failed",
        )
        self.assertEqual(self.run_timed_write_count(), 1)
        self.assert_all_stopped()

    def test_touch_change_between_motor_starts_blocks_second_start(self):
        session_id = self.claim_and_arm()
        left = self.sysfs.motors["outB"]
        original_write = supervisor_module.write_text

        def press_after_left_start(path, value):
            original_write(path, value)
            if (
                path == str(left / "command")
                and value == "run-timed"
            ):
                write(self.sysfs.sensors["in1"] / "value0", 1)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=press_after_left_start,
        ):
            with self.assertRaises(SupervisorError) as context:
                self.start_drive(session_id)

        self.assertEqual(context.exception.code, "touch_pressed")
        self.assertEqual(self.run_timed_write_count(), 1)
        self.assert_all_stopped()

    def test_heartbeat_expiry_between_motor_starts_blocks_second_start(self):
        session_id = self.claim_and_arm()
        left = self.sysfs.motors["outB"]
        original_write = supervisor_module.write_text

        def expire_after_left_start(path, value):
            original_write(path, value)
            if (
                path == str(left / "command")
                and value == "run-timed"
            ):
                self.clock.advance(500)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=expire_after_left_start,
        ):
            with self.assertRaises(SupervisorError) as context:
                self.start_drive(session_id)

        self.assertEqual(context.exception.code, "heartbeat_timeout")
        self.assertEqual(self.run_timed_write_count(), 1)
        self.assert_all_stopped()

    def test_start_skew_guard_blocks_second_motor_before_write(self):
        session_id = self.claim_and_arm()
        left = self.sysfs.motors["outB"]
        original_write = supervisor_module.write_text
        real_read_sensor = self.hal.read_sensor
        first_started = {"value": False}
        delay_applied = {"value": False}

        def mark_left_start(path, value):
            original_write(path, value)
            if (
                path == str(left / "command")
                and value == "run-timed"
            ):
                first_started["value"] = True

        def delayed_touch_read(role):
            if (
                first_started["value"]
                and not delay_applied["value"]
            ):
                self.clock.advance(26)
                delay_applied["value"] = True
            return real_read_sensor(role)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=mark_left_start,
        ), patch.object(
            self.hal,
            "read_sensor",
            side_effect=delayed_touch_read,
        ):
            with self.assertRaises(SupervisorError) as context:
                self.start_drive(session_id)

        self.assertEqual(context.exception.code, "start_skew")
        self.assertEqual(self.run_timed_write_count(), 1)
        self.assert_all_stopped()

    def test_slow_inventory_guard_blocks_second_motor_before_write(self):
        session_id = self.claim_and_arm()
        left = self.sysfs.motors["outB"]
        original_write = supervisor_module.write_text
        real_snapshot = self.supervisor._owner.snapshot_all
        first_started = {"value": False}
        delay_applied = {"value": False}

        def mark_left_start(path, value):
            original_write(path, value)
            if (
                path == str(left / "command")
                and value == "run-timed"
            ):
                first_started["value"] = True

        def slow_snapshot():
            result = real_snapshot()
            if (
                first_started["value"]
                and not delay_applied["value"]
            ):
                self.clock.advance(26)
                delay_applied["value"] = True
            return result

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=mark_left_start,
        ), patch.object(
            self.supervisor._owner,
            "snapshot_all",
            side_effect=slow_snapshot,
        ):
            with self.assertRaises(SupervisorError) as context:
                self.start_drive(session_id)

        self.assertEqual(context.exception.code, "start_skew")
        self.assertEqual(self.run_timed_write_count(), 1)
        self.assert_all_stopped()

    def test_motor_fault_after_first_start_blocks_second_motor(self):
        session_id = self.claim_and_arm()
        left = self.sysfs.motors["outB"]
        original_write = supervisor_module.write_text

        def stall_after_left_start(path, value):
            original_write(path, value)
            if (
                path == str(left / "command")
                and value == "run-timed"
            ):
                write(self.sysfs.motors["outA"] / "state", "stalled")

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=stall_after_left_start,
        ):
            with self.assertRaises(SupervisorError) as context:
                self.start_drive(session_id)

        self.assertEqual(context.exception.code, "motor_state_fault")
        self.assertEqual(self.run_timed_write_count(), 1)
        self.assert_all_stopped()

    def test_emergency_stop_is_unauthenticated_and_invalidates_session(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)

        status = self.supervisor.stop()

        self.assertEqual(status["state"], STATE_DISARMED)
        self.assertFalse(status["session_active"])
        self.assert_all_stopped()
        with self.assertRaises(SupervisorError):
            self.supervisor.heartbeat(session_id, 4)

    def test_fault_does_not_auto_resume_after_touch_release(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        write(self.sysfs.sensors["in1"] / "value0", 1)
        self.supervisor.poll_once()
        write(self.sysfs.sensors["in1"] / "value0", 0)
        self.release_touch_samples(3)

        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        status = self.supervisor.reset_fault()
        self.assertEqual(status["state"], STATE_DISARMED)
        self.assertFalse(status["motion_allowed"])

    def test_emergency_stop_does_not_clear_latched_fault(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        write(self.sysfs.sensors["in1"] / "value0", 1)
        self.supervisor.poll_once()

        status = self.supervisor.stop()

        self.assertEqual(status["state"], STATE_FAULT_LATCHED)
        self.assertEqual(status["fault"]["code"], "touch_pressed")
        self.assert_all_stopped()

    def test_motor_stopping_early_is_a_fault(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        self.move_drive_encoders(20, 20)
        write(self.sysfs.motors["outB"] / "state", "")
        self.clock.advance(150)

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "unexpected_early_stop",
        )
        self.assert_all_stopped()

    def test_idle_motor_fault_token_latches_fault(self):
        self.release_touch_samples()
        write(self.sysfs.motors["outB"] / "state", "stalled")

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "motor_state_fault",
        )

    def test_status_never_reports_motion_allowed_after_stale_heartbeat(self):
        self.claim_and_arm()
        self.clock.advance(500)

        status = self.supervisor.status()

        self.assertEqual(status["state"], STATE_ARMED_IDLE)
        self.assertFalse(status["motion_allowed"])

    def test_claim_without_first_heartbeat_expires(self):
        claimed = self.supervisor.claim("silent-owner")
        self.clock.advance(500)

        status = self.supervisor.poll_once()

        self.assertEqual(status["state"], STATE_DISARMED)
        self.assertFalse(status["session_active"])
        with self.assertRaises(SupervisorError):
            self.supervisor.heartbeat(claimed["session_id"], 1)

    def test_invalid_durations_do_not_consume_sequence_or_start(self):
        session_id = self.claim_and_arm()
        for duration in (801, 0, -1, True):
            with self.subTest(duration=duration):
                starts_before = self.run_timed_write_count()
                with self.assertRaises(SupervisorError):
                    self.start_drive(
                        session_id,
                        sequence_id=3,
                        duration_ms=duration,
                    )
                self.assertEqual(
                    self.supervisor.last_sequence_id,
                    2,
                )
                self.assertEqual(
                    self.run_timed_write_count(),
                    starts_before,
                )
                self.assertEqual(
                    self.supervisor.state,
                    STATE_ARMED_IDLE,
                )

    def test_completion_requires_fraction_of_expected_encoder_motion(self):
        session_id = self.claim_and_arm()
        self.start_drive(
            session_id,
            left_speed=250,
            right_speed=250,
            duration_ms=800,
        )
        self.move_drive_encoders(20, 20)
        self.clock.advance(800)
        self.supervisor.heartbeat(session_id, 4)

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "motion_completion_failed",
        )
        self.assert_all_stopped()

    def test_running_detects_uncommanded_arm_motor(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        write(self.sysfs.motors["outA"] / "state", "running")

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "unexpected_external_motion",
        )
        self.assert_all_stopped()

    def test_running_detects_fault_token_on_uncommanded_arm_motor(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        write(self.sysfs.motors["outA"] / "state", "stalled")

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "motor_state_fault",
        )
        self.assert_all_stopped()

    def test_slow_sensor_io_cannot_hide_expired_heartbeat(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        real_read_sensor = self.hal.read_sensor

        def slow_read(role):
            self.clock.advance(500)
            return real_read_sensor(role)

        with patch.object(
            self.hal,
            "read_sensor",
            side_effect=slow_read,
        ):
            self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "heartbeat_timeout",
        )
        self.assert_all_stopped()

    def test_close_retains_lock_when_stop_is_not_confirmed(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        blocked_path = str(
            self.sysfs.motors["outB"] / "command"
        )
        base_write = supervisor_module.write_text

        def fail_left_stop(path, value):
            if path == blocked_path and value == "stop":
                self.motor_writes.append((path, value))
                raise IOError("injected stop failure")
            base_write(path, value)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=fail_left_stop,
        ):
            status = self.supervisor.close()

            self.assertEqual(
                status["state"],
                STATE_FAULT_LATCHED,
            )
            self.assertEqual(
                status["fault"]["code"],
                "shutdown_stop_failed",
            )
            self.assertFalse(status["fault"]["stop_confirmed"])
            competing = open(self.lock_path, "a+")
            try:
                with self.assertRaises((BlockingIOError, OSError)):
                    fcntl.flock(
                        competing.fileno(),
                        fcntl.LOCK_EX | fcntl.LOCK_NB,
                    )
            finally:
                competing.close()

        recovered = self.supervisor.close()
        self.assertEqual(recovered["state"], STATE_CLOSED)

    def test_release_stop_failure_latches_fault_and_attempts_all_motors(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        blocked_path = str(
            self.sysfs.motors["outB"] / "command"
        )
        base_write = supervisor_module.write_text
        write_start = len(self.motor_writes)

        def fail_left_stop(path, value):
            if path == blocked_path and value == "stop":
                self.motor_writes.append((path, value))
                raise IOError("injected release stop failure")
            base_write(path, value)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=fail_left_stop,
        ):
            status = self.supervisor.release(session_id, 4)

        self.assertEqual(status["state"], STATE_FAULT_LATCHED)
        self.assertEqual(
            status["fault"]["code"],
            "release_stop_failed",
        )
        attempted_stop_paths = {
            path
            for path, value in self.motor_writes[write_start:]
            if value == "stop"
        }
        self.assertEqual(
            attempted_stop_paths,
            {
                str(motor / "command")
                for motor in self.sysfs.motors.values()
            },
        )

    def test_fault_retry_stops_before_touch_read_error(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        write(self.sysfs.sensors["in1"] / "value0", 1)
        blocked_path = str(
            self.sysfs.motors["outB"] / "command"
        )
        base_write = supervisor_module.write_text

        def fail_left_stop(path, value):
            if path == blocked_path and value == "stop":
                raise IOError("injected first stop failure")
            base_write(path, value)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=fail_left_stop,
        ):
            self.supervisor.poll_once()

        self.assertFalse(self.supervisor.fault["stop_confirmed"])
        (self.sysfs.sensors["in1"] / "value0").unlink()
        write(self.sysfs.motors["outB"] / "state", "running")

        self.supervisor.poll_once()

        self.assertTrue(self.supervisor.fault["stop_confirmed"])
        self.assertIn(
            "secondary_errors",
            self.supervisor.fault,
        )
        self.assert_all_stopped()

    def test_fault_is_latched_before_stop_interrupt_is_rethrown(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        write(self.sysfs.sensors["in1"] / "value0", 1)
        interrupted_path = str(
            self.sysfs.motors["outA"] / "command"
        )
        base_write = supervisor_module.write_text
        interrupted = {"value": False}

        def interrupt_first_stop(path, value):
            if (
                path == interrupted_path
                and value == "stop"
                and not interrupted["value"]
            ):
                interrupted["value"] = True
                raise KeyboardInterrupt()
            base_write(path, value)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=interrupt_first_stop,
        ):
            with self.assertRaises(KeyboardInterrupt):
                self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        self.assertEqual(
            self.supervisor.fault["code"],
            "touch_pressed",
        )
        self.assertFalse(self.supervisor.status()["session_active"])
        self.assert_all_stopped()

    def test_session_secret_is_not_exposed_by_status_or_audit(self):
        claimed = self.supervisor.claim("owner")
        secret = claimed["session_id"]

        serialized = json.dumps(
            {
                "status": self.supervisor.status(),
                "audit": self.supervisor.audit_events,
            },
            sort_keys=True,
        )

        self.assertNotIn(secret, serialized)
        self.assertIn("session_fingerprint", serialized)

    def test_supervisor_loop_polls_heartbeat_without_client_activity(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        loop = EV3SupervisorLoop(self.supervisor)

        for _ in range(30):
            self.move_drive_encoders(2, 2)
            loop.run_once()
            if self.supervisor.state == STATE_FAULT_LATCHED:
                break

        self.assertEqual(
            self.supervisor.fault["code"],
            "heartbeat_timeout",
        )
        self.assert_all_stopped()

    def test_protocol_status_is_controller_scoped_and_ttl_bounded(self):
        protocol = SupervisorProtocol(
            self.supervisor,
            "ev3rstorm-01.ev3-main",
        )
        response = protocol.execute(
            self.protocol_request("status"),
            dispatch_at_ms=self.clock.now_ms,
        )
        self.assertTrue(response["ok"])
        self.assertEqual(response["result"]["state"], STATE_DISARMED)

        wrong = protocol.execute(
            self.protocol_request(
                "status",
                controller_id="another-controller",
            ),
            dispatch_at_ms=self.clock.now_ms,
        )
        self.assertFalse(wrong["ok"])
        self.assertEqual(wrong["error"]["code"], "wrong_controller")

        stale = protocol.execute(
            self.protocol_request(
                "status",
                ttl_ms=20,
                received_at_ms=self.clock.now_ms,
            ),
            dispatch_at_ms=self.clock.now_ms + 20,
        )
        self.assertFalse(stale["ok"])
        self.assertEqual(stale["error"]["code"], "stale_request")

    def test_protocol_describes_exact_controller_capabilities(self):
        protocol = SupervisorProtocol(
            self.supervisor,
            "ev3rstorm-01.ev3-main",
        )
        response = protocol.execute(
            self.protocol_request("describe"),
            dispatch_at_ms=self.clock.now_ms,
        )

        self.assertTrue(response["ok"])
        description = response["result"]
        self.assertEqual(description["robot_id"], "ev3rstorm-01")
        self.assertEqual(
            description["controller_id"],
            "ev3rstorm-01.ev3-main",
        )
        self.assertEqual(len(description["controller_instance_id"]), 32)
        self.assertFalse(description["motion_enabled"])
        self.assertEqual(description["remaining_motion_budget"], 0)
        drive = description["capabilities"][
            "differential_drive_timed"
        ]
        self.assertFalse(drive["enabled"])
        self.assertEqual(drive["max_abs_speed_dps"], 100)
        self.assertEqual(drive["max_duration_ms"], 300)
        self.assertNotIn("arm", description["capabilities"])
        self.assertNotIn("speech", description["capabilities"])

    def test_protocol_motion_is_disabled_by_default_before_motor_write(self):
        protocol = SupervisorProtocol(
            self.supervisor,
            "ev3rstorm-01.ev3-main",
        )
        before = self.run_timed_write_count()
        response = protocol.execute(
            self.protocol_request(
                "drive_timed",
                arguments={
                    "session_id": "not-a-session",
                    "sequence_id": 3,
                    "command_id": "drive-1",
                    "reference_heartbeat_sequence": 1,
                    "left_speed_dps": 100,
                    "right_speed_dps": 100,
                    "duration_ms": 300,
                },
            ),
            dispatch_at_ms=self.clock.now_ms,
        )

        self.assertFalse(response["ok"])
        self.assertEqual(response["error"]["code"], "motion_disabled")
        self.assertEqual(self.run_timed_write_count(), before)

    def test_protocol_one_shot_motion_budget_and_experiment_limits(self):
        session_id = self.claim_and_arm()
        protocol = SupervisorProtocol(
            self.supervisor,
            "ev3rstorm-01.ev3-main",
            allow_motion=True,
            motion_budget=1,
            experiment_max_abs_speed_dps=100,
            experiment_max_duration_ms=300,
        )
        common = {
            "session_id": session_id,
            "sequence_id": 3,
            "command_id": "drive-1",
            "reference_heartbeat_sequence": 1,
            "left_speed_dps": 101,
            "right_speed_dps": 100,
            "duration_ms": 300,
        }
        rejected = protocol.execute(
            self.protocol_request(
                "drive_timed",
                arguments=common,
            ),
            dispatch_at_ms=self.clock.now_ms,
        )
        self.assertFalse(rejected["ok"])
        self.assertEqual(
            rejected["error"]["code"],
            "experiment_speed_limit",
        )
        self.assertEqual(self.run_timed_write_count(), 0)
        self.assertEqual(protocol.remaining_motion_budget, 1)

        common["left_speed_dps"] = 100
        accepted = protocol.execute(
            self.protocol_request(
                "drive_timed",
                arguments=common,
            ),
            dispatch_at_ms=self.clock.now_ms,
        )
        self.assertTrue(accepted["ok"])
        self.assertEqual(accepted["result"]["state"], STATE_RUNNING)
        self.assertEqual(self.run_timed_write_count(), 2)
        self.assertEqual(protocol.remaining_motion_budget, 0)

        common["sequence_id"] = 4
        common["command_id"] = "drive-2"
        second = protocol.execute(
            self.protocol_request(
                "drive_timed",
                arguments=common,
                request_id="request-2",
            ),
            dispatch_at_ms=self.clock.now_ms,
        )
        self.assertFalse(second["ok"])
        self.assertEqual(
            second["error"]["code"],
            "motion_budget_exhausted",
        )
        self.assertEqual(self.run_timed_write_count(), 2)

    def test_protocol_rechecks_queue_ttl_immediately_before_start(self):
        session_id = self.claim_and_arm()
        protocol = SupervisorProtocol(
            self.supervisor,
            "ev3rstorm-01.ev3-main",
            allow_motion=True,
            motion_budget=1,
            experiment_max_abs_speed_dps=100,
            experiment_max_duration_ms=300,
        )
        real_idle_check = self.supervisor._check_idle_motors

        def delayed_idle_check():
            result = real_idle_check()
            self.clock.advance(2)
            return result

        request = self.protocol_request(
            "drive_timed",
            arguments={
                "session_id": session_id,
                "sequence_id": 3,
                "command_id": "drive-expired",
                "reference_heartbeat_sequence": 1,
                "left_speed_dps": 100,
                "right_speed_dps": 100,
                "duration_ms": 300,
            },
            ttl_ms=1,
            received_at_ms=self.clock.now_ms,
        )
        with patch.object(
            self.supervisor,
            "_check_idle_motors",
            side_effect=delayed_idle_check,
        ):
            response = protocol.execute(
                request,
                dispatch_at_ms=self.clock.now_ms,
            )

        self.assertFalse(response["ok"])
        self.assertEqual(response["error"]["code"], "stale_request")
        self.assertEqual(self.run_timed_write_count(), 0)
        self.assertEqual(protocol.remaining_motion_budget, 0)

    def test_stop_event_between_motor_starts_blocks_second_start(self):
        session_id = self.claim_and_arm()
        protocol = SupervisorProtocol(
            self.supervisor,
            "ev3rstorm-01.ev3-main",
            allow_motion=True,
            motion_budget=1,
            experiment_max_abs_speed_dps=100,
            experiment_max_duration_ms=300,
        )
        cancelled = threading.Event()
        session = ForegroundSupervisorSession(
            self.supervisor,
            protocol,
            InteractiveInput(),
            InteractiveOutput(),
            max_session_ms=120000,
            external_shutdown_event=cancelled,
        )
        original_side_effect = self.mock_supervisor_write.side_effect

        def cancel_after_first_start(path, value):
            result = original_side_effect(path, value)
            if value == "run-timed":
                cancelled.set()
            return result

        request = self.protocol_request(
            "drive_timed",
            arguments={
                "session_id": session_id,
                "sequence_id": 3,
                "command_id": "drive-cancelled",
                "reference_heartbeat_sequence": 1,
                "left_speed_dps": 100,
                "right_speed_dps": 100,
                "duration_ms": 300,
            },
        )
        self.mock_supervisor_write.side_effect = (
            cancel_after_first_start
        )
        try:
            response = protocol.execute(
                request,
                dispatch_at_ms=self.clock.now_ms,
                cancellation_requested=(
                    session._motion_start_cancelled
                ),
            )
        finally:
            self.mock_supervisor_write.side_effect = (
                original_side_effect
            )

        self.assertFalse(response["ok"])
        self.assertEqual(
            response["error"]["code"],
            "external_stop_requested",
        )
        self.assertEqual(self.run_timed_write_count(), 1)
        self.assertEqual(self.supervisor.state, STATE_FAULT_LATCHED)
        self.assert_all_stopped()
        self.assertEqual(protocol.remaining_motion_budget, 0)

    def test_stale_stop_is_still_prioritized_and_invalidates_session(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        protocol = SupervisorProtocol(
            self.supervisor,
            "ev3rstorm-01.ev3-main",
        )
        response = protocol.execute(
            self.protocol_request(
                "stop",
                ttl_ms=20,
                received_at_ms=self.clock.now_ms,
            ),
            dispatch_at_ms=self.clock.now_ms + 1000,
        )

        self.assertTrue(response["ok"])
        self.assertEqual(response["result"]["state"], STATE_DISARMED)
        self.assertFalse(response["result"]["session_active"])
        self.assert_all_stopped()

    def test_protocol_does_not_mask_unexpected_dispatch_failure(self):
        protocol = SupervisorProtocol(
            self.supervisor,
            "ev3rstorm-01.ev3-main",
        )
        with patch.object(
            self.supervisor,
            "status",
            side_effect=RuntimeError("unexpected"),
        ):
            with self.assertRaises(RuntimeError):
                protocol.execute(
                    self.protocol_request("status"),
                    dispatch_at_ms=self.clock.now_ms,
                )

    def test_loop_dispatches_at_most_one_request_after_safety_poll(self):
        loop = EV3SupervisorLoop(self.supervisor)
        order = []
        real_poll = self.supervisor.poll_once

        def poll():
            order.append("poll")
            return real_poll()

        def dispatch():
            order.append("dispatch")

        with patch.object(
            self.supervisor,
            "poll_once",
            side_effect=poll,
        ):
            loop.run_once(dispatch_one=dispatch)

        self.assertEqual(order, ["poll", "dispatch"])

    def test_loop_skips_dispatch_when_poll_consumes_the_tick(self):
        loop = EV3SupervisorLoop(self.supervisor)
        dispatched = []
        real_poll = self.supervisor.poll_once

        def full_tick_poll():
            result = real_poll()
            self.clock.advance(loop.interval_ms)
            return result

        with patch.object(
            self.supervisor,
            "poll_once",
            side_effect=full_tick_poll,
        ):
            loop.run_once(
                dispatch_one=lambda: dispatched.append(True)
            )

        self.assertEqual(dispatched, [])
        self.assertEqual(self.supervisor.state, STATE_DISARMED)

    def test_loop_faults_and_stops_on_unexpected_dispatch_exception(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        loop = EV3SupervisorLoop(self.supervisor)

        with self.assertRaises(RuntimeError):
            loop.run_once(
                dispatch_one=lambda: (_ for _ in ()).throw(
                    RuntimeError("boom")
                )
            )

        self.assertEqual(self.supervisor.state, STATE_FAULT_LATCHED)
        self.assertEqual(
            self.supervisor.fault["code"],
            "dispatch_failure",
        )
        self.assert_all_stopped()

    def test_foreground_session_completes_motion_free_handshake(self):
        responses = []

        def client(input_stream, output):
            def exchange(operation, arguments=None, number=1):
                input_stream.send(
                    self.protocol_wire(
                        operation,
                        arguments=arguments,
                        request_id="request-{}".format(number),
                    )
                )
                response = output.receive()
                responses.append(response)
                self.assertTrue(response["ok"])
                return response["result"]

            claim = exchange(
                "claim",
                {"owner_id": "mac-manual-client"},
                1,
            )
            session_id = claim["session_id"]
            exchange(
                "heartbeat",
                {"session_id": session_id, "sequence_id": 1},
                2,
            )
            exchange(
                "arm",
                {"session_id": session_id, "sequence_id": 2},
                3,
            )
            status = exchange("status", number=4)
            self.assertEqual(status["state"], STATE_ARMED_IDLE)
            exchange(
                "release",
                {"session_id": session_id, "sequence_id": 3},
                5,
            )
            exchange("shutdown", number=6)
            input_stream.close()

        result = self.run_interactive_session(client)

        self.assertEqual(len(responses), 6)
        self.assertEqual(result["status"]["state"], STATE_CLOSED)
        self.assertTrue(result["status"]["audit_complete"])
        self.assertIsNone(result["transport_failure"])
        self.assertFalse(result["motion_enabled"])
        self.assertEqual(self.run_timed_write_count(), 0)
        self.assert_all_stopped()

    def test_foreground_session_excludes_worker_startup_from_poll_deadline(
        self,
    ):
        responses = []

        def client(input_stream, output):
            input_stream.send(
                self.protocol_wire(
                    "status",
                    request_id="status-after-worker-start",
                )
            )
            responses.append(output.receive())
            input_stream.send(
                self.protocol_wire(
                    "shutdown",
                    request_id="shutdown-after-worker-start",
                )
            )
            responses.append(output.receive())
            input_stream.close()

        writer_start = supervisor_daemon_module._ResponseWriter.start
        reader_start = supervisor_daemon_module._RequestReader.start

        def delayed_writer_start(writer):
            self.clock.advance(17)
            return writer_start(writer)

        def delayed_reader_start(reader):
            self.clock.advance(16)
            return reader_start(reader)

        with patch.object(
            supervisor_daemon_module._ResponseWriter,
            "start",
            delayed_writer_start,
        ), patch.object(
            supervisor_daemon_module._RequestReader,
            "start",
            delayed_reader_start,
        ):
            result = self.run_interactive_session(client)

        self.assertEqual(len(responses), 2)
        self.assertTrue(responses[0]["ok"])
        self.assertEqual(
            responses[0]["result"]["state"],
            STATE_DISARMED,
        )
        self.assertTrue(responses[1]["ok"])
        self.assertEqual(result["status"]["state"], STATE_CLOSED)
        self.assertIsNone(result["status"]["fault"])
        self.assertIsNone(result["transport_failure"])
        self.assertEqual(self.run_timed_write_count(), 0)
        self.assertNotIn(
            "poll_deadline_missed",
            [
                event.get("reason_code")
                for event in self.supervisor.audit_events
            ],
        )
        self.assert_all_stopped()

    def test_foreground_session_rejects_drive_when_motion_disabled(self):
        drive_response = []

        def client(input_stream, output):
            def exchange(operation, arguments=None, number=1):
                input_stream.send(
                    self.protocol_wire(
                        operation,
                        arguments=arguments,
                        request_id="request-{}".format(number),
                    )
                )
                return output.receive()

            claim = exchange(
                "claim",
                {"owner_id": "mac-manual-client"},
                1,
            )["result"]
            session_id = claim["session_id"]
            exchange(
                "heartbeat",
                {"session_id": session_id, "sequence_id": 1},
                2,
            )
            exchange(
                "arm",
                {"session_id": session_id, "sequence_id": 2},
                3,
            )
            drive_response.append(
                exchange(
                    "drive_timed",
                    {
                        "session_id": session_id,
                        "sequence_id": 3,
                        "command_id": "drive-1",
                        "reference_heartbeat_sequence": 1,
                        "left_speed_dps": 100,
                        "right_speed_dps": 100,
                        "duration_ms": 300,
                    },
                    4,
                )
            )
            exchange("shutdown", number=5)
            input_stream.close()

        result = self.run_interactive_session(client)

        self.assertFalse(drive_response[0]["ok"])
        self.assertEqual(
            drive_response[0]["error"]["code"],
            "motion_disabled",
        )
        self.assertEqual(self.run_timed_write_count(), 0)
        self.assertEqual(result["status"]["state"], STATE_CLOSED)

    def test_wrong_controller_shutdown_neither_stops_nor_closes_session(self):
        observations = []

        def client(input_stream, output):
            input_stream.send(
                self.protocol_wire(
                    "claim",
                    {"owner_id": "right-controller-client"},
                )
            )
            claim = output.receive()
            self.assertTrue(claim["ok"])

            input_stream.send(
                self.protocol_wire(
                    "shutdown",
                    request_id="wrong-shutdown",
                    controller_id="another-controller",
                )
            )
            wrong = output.receive()
            observations.append(wrong)
            self.assertFalse(wrong["ok"])

            input_stream.send(
                self.protocol_wire(
                    "status",
                    request_id="status-after-wrong-shutdown",
                )
            )
            status = output.receive()
            observations.append(status)
            self.assertTrue(status["ok"])
            self.assertTrue(status["result"]["session_active"])

            input_stream.send(
                self.protocol_wire(
                    "shutdown",
                    request_id="right-shutdown",
                )
            )
            output.receive()
            input_stream.close()

        result = self.run_interactive_session(client)

        self.assertEqual(
            observations[0]["error"]["code"],
            "wrong_controller",
        )
        self.assertTrue(observations[1]["result"]["session_active"])
        self.assertIsNone(result["transport_failure"])
        self.assertEqual(result["status"]["state"], STATE_CLOSED)

    def test_duplicate_request_id_is_rejected_without_dispatch(self):
        responses = []

        def client(input_stream, output):
            wire = self.protocol_wire(
                "status",
                request_id="same-request",
            )
            input_stream.send(wire)
            responses.append(output.receive())
            input_stream.send(wire)
            responses.append(output.receive())
            input_stream.send(
                self.protocol_wire(
                    "shutdown",
                    request_id="shutdown-request",
                )
            )
            output.receive()
            input_stream.close()

        result = self.run_interactive_session(client)

        self.assertTrue(responses[0]["ok"])
        self.assertFalse(responses[1]["ok"])
        self.assertEqual(
            responses[1]["error"]["code"],
            "duplicate_request_id",
        )
        self.assertEqual(result["status"]["state"], STATE_CLOSED)
        self.assertEqual(self.run_timed_write_count(), 0)

    def test_foreground_session_eof_stops_and_closes_locally(self):
        def client(input_stream, output):
            input_stream.send(
                self.protocol_wire(
                    "claim",
                    {"owner_id": "client-that-dies"},
                )
            )
            claim = output.receive()["result"]
            input_stream.send(
                self.protocol_wire(
                    "heartbeat",
                    {
                        "session_id": claim["session_id"],
                        "sequence_id": 1,
                    },
                    request_id="request-2",
                )
            )
            output.receive()
            input_stream.close()

        result = self.run_interactive_session(client)

        self.assertEqual(result["transport_failure"], "input_eof")
        self.assertEqual(result["status"]["state"], STATE_CLOSED)
        self.assertTrue(result["status"]["audit_complete"])
        self.assertEqual(self.run_timed_write_count(), 0)
        self.assert_all_stopped()

    def test_foreground_session_malformed_frame_fails_closed(self):
        def client(input_stream, _output):
            input_stream.send(b"{broken json\n")

        result = self.run_interactive_session(client)

        self.assertEqual(result["transport_failure"], "invalid_json")
        self.assertEqual(result["status"]["state"], STATE_CLOSED)
        self.assertEqual(self.run_timed_write_count(), 0)
        self.assert_all_stopped()

    def test_foreground_session_output_failure_fails_closed(self):
        output = InteractiveOutput(fail_writes=True)

        def client(input_stream, _output):
            input_stream.send(self.protocol_wire("status"))
            deadline = time.monotonic() + 1
            while (
                self.supervisor.state != STATE_CLOSED
                and time.monotonic() < deadline
            ):
                time.sleep(0.001)
            input_stream.close()

        result = self.run_interactive_session(
            client,
            output=output,
        )

        self.assertEqual(
            result["transport_failure"],
            "output_write_failed",
        )
        self.assertEqual(result["status"]["state"], STATE_CLOSED)
        self.assertEqual(self.run_timed_write_count(), 0)
        self.assert_all_stopped()

    def test_real_full_output_pipe_cannot_block_safety_loop(self):
        read_descriptor, write_descriptor = os.pipe()
        flags = fcntl.fcntl(write_descriptor, fcntl.F_GETFL)
        fcntl.fcntl(
            write_descriptor,
            fcntl.F_SETFL,
            flags | os.O_NONBLOCK,
        )
        try:
            while True:
                try:
                    os.write(write_descriptor, b"x" * 4096)
                except OSError:
                    break
            output = os.fdopen(
                write_descriptor,
                "wb",
                buffering=0,
            )
            write_descriptor = None

            def client(input_stream, _output):
                input_stream.send(self.protocol_wire("status"))
                deadline = time.monotonic() + 2
                while (
                    self.supervisor.state != STATE_CLOSED
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.001)
                input_stream.close()

            result = self.run_interactive_session(
                client,
                output=output,
            )
            output.close()
        finally:
            if write_descriptor is not None:
                os.close(write_descriptor)
            os.close(read_descriptor)

        self.assertEqual(
            result["transport_failure"],
            "output_write_failed",
        )
        self.assertEqual(result["status"]["state"], STATE_CLOSED)
        self.assertTrue(result["status"]["audit_complete"])
        self.assertEqual(self.run_timed_write_count(), 0)
        self.assert_all_stopped()

    def test_run_daemon_rejects_truthy_non_boolean_motion_modes(self):
        for invalid in ("false", 1, None):
            with self.assertRaises(SessionError) as context:
                run_daemon(
                    None,
                    None,
                    None,
                    None,
                    allow_one_drive_test=invalid,
                )
            self.assertEqual(
                context.exception.code,
                "invalid_motion_mode",
            )
        self.assertEqual(self.run_timed_write_count(), 0)

    def test_public_daemon_cli_has_no_motion_enable_flag(self):
        parser = build_daemon_parser()
        option_strings = {
            option
            for action in parser._actions
            for option in action.option_strings
        }
        self.assertNotIn("--allow-one-drive-test", option_strings)

    def test_direct_daemon_help_is_warning_clean(self):
        completed = subprocess.run(
            [
                sys.executable,
                "-W",
                "error",
                str(PROJECT_ROOT / "ev3" / "supervisor_daemon.py"),
                "--help",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("strict jsonl", completed.stdout.lower())

    def test_supervisor_loop_returns_fault_after_missed_deadline(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        loop = EV3SupervisorLoop(self.supervisor)

        def slow_poll():
            self.clock.advance(41)
            return self.supervisor.status()

        with patch.object(
            self.supervisor,
            "poll_once",
            side_effect=slow_poll,
        ):
            status = loop.run_once()

        self.assertEqual(status["state"], STATE_FAULT_LATCHED)
        self.assertEqual(
            status["fault"]["code"],
            "poll_deadline_missed",
        )
        self.assert_all_stopped()

    def test_supervisor_loop_stops_before_poll_when_already_late(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        loop = EV3SupervisorLoop(self.supervisor)
        loop.run_once()
        self.clock.advance(21)

        with patch.object(
            self.supervisor,
            "poll_once",
            wraps=self.supervisor.poll_once,
        ) as poll:
            status = loop.run_once()

        poll.assert_not_called()
        self.assertEqual(status["state"], STATE_FAULT_LATCHED)
        self.assertEqual(
            status["fault"]["code"],
            "poll_deadline_missed",
        )
        self.assert_all_stopped()

    def test_supervisor_loop_checks_deadline_on_first_tick(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        loop = EV3SupervisorLoop(self.supervisor)
        self.clock.advance(21)

        with patch.object(
            self.supervisor,
            "poll_once",
            wraps=self.supervisor.poll_once,
        ) as poll:
            status = loop.run_once()

        poll.assert_not_called()
        self.assertEqual(status["state"], STATE_FAULT_LATCHED)
        self.assertEqual(
            status["fault"]["code"],
            "poll_deadline_missed",
        )
        self.assert_all_stopped()

    def test_loop_construction_does_not_erase_preexisting_gap(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        self.clock.advance(21)
        loop = EV3SupervisorLoop(self.supervisor)

        with patch.object(
            self.supervisor,
            "poll_once",
            wraps=self.supervisor.poll_once,
        ) as poll:
            status = loop.run_once()

        poll.assert_not_called()
        self.assertEqual(status["state"], STATE_FAULT_LATCHED)
        self.assertEqual(
            status["fault"]["code"],
            "poll_deadline_missed",
        )
        self.assert_all_stopped()

    def test_supervisor_loop_closes_on_keyboard_interrupt(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        loop = EV3SupervisorLoop(self.supervisor)

        with patch.object(
            self.supervisor,
            "poll_once",
            side_effect=KeyboardInterrupt(),
        ):
            with self.assertRaises(KeyboardInterrupt):
                loop.run_forever(lambda: False)

        self.assertEqual(self.supervisor.state, STATE_CLOSED)
        self.assert_all_stopped()

    def test_supervisor_loop_surfaces_unverified_shutdown_stop(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        loop = EV3SupervisorLoop(self.supervisor)
        blocked_path = str(
            self.sysfs.motors["outB"] / "command"
        )
        base_write = supervisor_module.write_text

        def fail_left_stop(path, value):
            if path == blocked_path and value == "stop":
                raise IOError("persistent shutdown stop failure")
            base_write(path, value)

        with patch.object(
            self.supervisor,
            "poll_once",
            side_effect=KeyboardInterrupt(),
        ), patch.object(
            supervisor_module,
            "write_text",
            side_effect=fail_left_stop,
        ):
            with self.assertRaises(SupervisorError) as context:
                loop.run_forever(lambda: False)

        self.assertEqual(
            context.exception.code,
            "shutdown_stop_failed",
        )
        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        competing = open(self.lock_path, "a+")
        try:
            with self.assertRaises((BlockingIOError, OSError)):
                fcntl.flock(
                    competing.fileno(),
                    fcntl.LOCK_EX | fcntl.LOCK_NB,
                )
        finally:
            competing.close()

        self.supervisor.close()

    def test_supervisor_loop_rejects_incomplete_shutdown_audit(self):
        loop = EV3SupervisorLoop(self.supervisor)
        real_append = self.supervisor._audit_buffer.append

        def fail_terminal_audit(event, terminal=False):
            if terminal:
                raise SupervisorError(
                    "audit_buffer_full",
                    "injected terminal audit failure",
                )
            return real_append(event, terminal=terminal)

        with patch.object(
            self.supervisor._audit_buffer,
            "append",
            side_effect=fail_terminal_audit,
        ):
            with self.assertRaises(SupervisorError) as context:
                loop.run_forever(lambda: True)

        self.assertEqual(
            context.exception.code,
            "shutdown_audit_failed",
        )
        self.assertEqual(self.supervisor.state, STATE_CLOSED)

    def test_dispatch_thread_binding_rejects_cross_thread_mutation(self):
        session_id = self.supervisor.claim("owner")["session_id"]
        self.supervisor.bind_to_current_thread()
        result = []

        def mutate_from_other_thread():
            try:
                self.supervisor.heartbeat(session_id, 1)
            except SupervisorError as error:
                result.append(error.code)

        worker = threading.Thread(target=mutate_from_other_thread)
        worker.start()
        worker.join()

        self.assertEqual(result, ["wrong_dispatch_thread"])

    def test_monotonic_clock_rollback_faults_and_stops(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        self.clock.advance(20)
        self.supervisor.poll_once()
        self.clock.advance(-1)

        self.supervisor.poll_once()

        self.assertEqual(
            self.supervisor.fault["code"],
            "clock_rollback",
        )
        self.assert_all_stopped()

    def test_audit_failure_during_acceptance_prevents_motor_start(self):
        self.supervisor.close()
        self.supervisor = EV3Supervisor(
            self.hal,
            audit_buffer=AuditBuffer(5),
            session_id_factory=lambda: "audit-session",
        )
        session_id = self.claim_and_arm()

        with self.assertRaises(SupervisorError) as context:
            self.start_drive(session_id)

        self.assertEqual(context.exception.code, "audit_failure")
        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        self.assertEqual(
            read_text(str(self.sysfs.motors["outB"] / "speed_sp")),
            "0",
        )
        self.assert_all_stopped()

    def test_audit_failure_after_motor_start_stops_every_motor(self):
        self.supervisor.close()
        self.supervisor = EV3Supervisor(
            self.hal,
            audit_buffer=AuditBuffer(6),
            session_id_factory=lambda: "poststart-audit-session",
        )
        session_id = self.claim_and_arm()

        with self.assertRaises(SupervisorError) as context:
            self.start_drive(session_id)

        self.assertEqual(context.exception.code, "audit_failure")
        self.assertEqual(self.run_timed_write_count(), 2)
        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        self.assert_all_stopped()

    def test_nonempty_audit_buffer_is_rejected_before_lock(self):
        self.supervisor.close()
        audit_buffer = AuditBuffer(3)
        audit_buffer.append({"event": "old"})

        with self.assertRaises(SupervisorError) as context:
            EV3Supervisor(
                self.hal,
                audit_buffer=audit_buffer,
            )

        self.assertEqual(
            context.exception.code,
            "audit_buffer_not_empty",
        )
        competing = open(self.lock_path, "a+")
        try:
            fcntl.flock(
                competing.fileno(),
                fcntl.LOCK_EX | fcntl.LOCK_NB,
            )
        finally:
            fcntl.flock(competing.fileno(), fcntl.LOCK_UN)
            competing.close()

    def test_keyboard_interrupt_during_stop_attempts_all_motors(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id, duration_ms=800)
        interrupted_path = str(
            self.sysfs.motors["outA"] / "command"
        )
        base_write = supervisor_module.write_text
        interrupted = {"value": False}
        write_start = len(self.motor_writes)

        def interrupt_first_stop(path, value):
            if (
                path == interrupted_path
                and value == "stop"
                and not interrupted["value"]
            ):
                interrupted["value"] = True
                self.motor_writes.append((path, value))
                raise KeyboardInterrupt()
            base_write(path, value)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=interrupt_first_stop,
        ):
            with self.assertRaises(KeyboardInterrupt):
                self.supervisor.stop()

        attempted = {
            path
            for path, value in self.motor_writes[write_start:]
            if value == "stop"
        }
        self.assertEqual(
            attempted,
            {
                str(motor / "command")
                for motor in self.sysfs.motors.values()
            },
        )
        self.assertEqual(
            self.supervisor.state,
            STATE_FAULT_LATCHED,
        )
        self.assert_all_stopped()

    def test_close_is_idempotent_and_releases_lifetime_lock(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)

        first = self.supervisor.close()
        second = self.supervisor.close()

        self.assertEqual(first["state"], STATE_CLOSED)
        self.assertTrue(first["audit_complete"])
        self.assertEqual(second, first)
        self.assert_all_stopped()

        lock_handle = open(self.lock_path, "a+")
        try:
            fcntl.flock(
                lock_handle.fileno(),
                fcntl.LOCK_EX | fcntl.LOCK_NB,
            )
        finally:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
            lock_handle.close()

    def test_close_finalizes_after_one_shot_stop_interrupt(self):
        session_id = self.claim_and_arm()
        self.start_drive(session_id)
        interrupted_path = str(
            self.sysfs.motors["outA"] / "command"
        )
        base_write = supervisor_module.write_text
        interrupted = {"value": False}

        def interrupt_first_stop(path, value):
            if (
                path == interrupted_path
                and value == "stop"
                and not interrupted["value"]
            ):
                interrupted["value"] = True
                raise KeyboardInterrupt()
            base_write(path, value)

        with patch.object(
            supervisor_module,
            "write_text",
            side_effect=interrupt_first_stop,
        ):
            with self.assertRaises(KeyboardInterrupt):
                self.supervisor.close()

        self.assertEqual(self.supervisor.state, STATE_CLOSED)
        recovered = self.supervisor.close()
        self.assertTrue(recovered["audit_complete"])
        lock_handle = open(self.lock_path, "a+")
        try:
            fcntl.flock(
                lock_handle.fileno(),
                fcntl.LOCK_EX | fcntl.LOCK_NB,
            )
        finally:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
            lock_handle.close()

    def test_close_keeps_idempotent_status_after_terminal_interrupt(self):
        real_append = self.supervisor._audit_buffer.append
        interrupted = {"value": False}

        def interrupt_terminal(event, terminal=False):
            if terminal and not interrupted["value"]:
                interrupted["value"] = True
                raise KeyboardInterrupt()
            return real_append(event, terminal=terminal)

        with patch.object(
            self.supervisor._audit_buffer,
            "append",
            side_effect=interrupt_terminal,
        ):
            with self.assertRaises(KeyboardInterrupt):
                self.supervisor.close()

        second = self.supervisor.close()
        self.assertEqual(second["state"], STATE_CLOSED)
        self.assertFalse(second["audit_complete"])

    def test_hal_rejects_bool_as_supervisor_integer_before_startup(self):
        config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        config["limits"]["supervisor"]["poll_interval_ms"] = True
        self.supervisor.close()

        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            suffix=".json",
        ) as handle:
            json.dump(config, handle)
            handle.flush()
            with self.assertRaises(SafetyError) as context:
                RobotHAL(
                    handle.name,
                    sysfs_root=str(self.sysfs.root),
                    lock_path=self.lock_path,
                    sleep_fn=self.clock.sleep,
                    monotonic_fn=self.clock.monotonic,
                )

        self.assertIn(
            "limits.supervisor.poll_interval_ms",
            str(context.exception),
        )


if __name__ == "__main__":
    unittest.main()
