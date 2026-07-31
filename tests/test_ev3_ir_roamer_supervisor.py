import json
import os
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import ev3.supervisor as supervisor_module
from ev3.robot_hal import RobotHAL, read_text
from ev3.supervisor import (
    EV3Supervisor,
    IR_ROAMER_MAX_POLL_LATENESS_MS,
    IR_ROAMER_POLL_INTERVAL_MS,
    IR_ROAMER_RUNTIME_PROFILE,
    STATE_ARMED_IDLE,
    STATE_FAULT_LATCHED,
    STATE_RUNNING,
    SupervisorError,
)
from ev3.supervisor_daemon import (
    ForegroundSupervisorSession,
    IR_ROAMER_MAX_SESSION_MS,
    MAX_PROCESS_REQUESTS,
    SessionError,
    build_parser,
    run_daemon,
)
from ev3.supervisor_protocol import (
    DRIVE_PULSE_DURATION_MS,
    DRIVE_PULSE_MAX_COMMANDS,
    DRIVE_PULSE_MAX_TOTAL_DURATION_MS,
    IR_ROAMER_MAX_PROCESS_REQUESTS,
    MOTION_FREE_MAX_PROCESS_REQUESTS,
    PROTOCOL_VERSION,
    ProtocolError,
    SupervisorProtocol,
    decode_request,
)
from tests.test_ev3_supervisor import (
    FakeClock,
    FakeSysfs,
    InteractiveInput,
    InteractiveOutput,
    write,
)


PROJECT_ROOT = Path(__file__).parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "ev3rstorm.json"
CONTROLLER_ID = "ev3rstorm-01.ev3-main"


class EV3IRRoamerSupervisorTests(unittest.TestCase):
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
        self.hal = RobotHAL(
            str(CONFIG_PATH),
            sysfs_root=str(self.sysfs.root),
            lock_path=str(Path(self.temp.name) / "motor.lock"),
            sleep_fn=self.clock.sleep,
            monotonic_fn=self.clock.monotonic,
            speech_lock_path=str(
                Path(self.temp.name) / "audio.lock"
            ),
        )
        real_write = supervisor_module.write_text
        self.motor_writes = []

        def simulate_kernel_state(path, value):
            self.motor_writes.append((path, value))
            real_write(path, value)
            target = Path(path)
            if target.name == "command":
                real_write(
                    str(target.parent / "state"),
                    "running" if value == "run-timed" else "",
                )

        self.write_patcher = patch.object(
            supervisor_module,
            "write_text",
            side_effect=simulate_kernel_state,
        )
        self.write_patcher.start()
        self.supervisor = EV3Supervisor(
            self.hal,
            session_id_factory=lambda: "session-ir-roamer",
            runtime_profile=IR_ROAMER_RUNTIME_PROFILE,
        )

    def tearDown(self):
        try:
            self.supervisor.close()
        finally:
            self.write_patcher.stop()
            self.temp.cleanup()

    @property
    def infrared_path(self):
        return self.sysfs.sensors["in4"]

    def run_timed_write_count(self):
        return sum(
            1
            for _path, value in self.motor_writes
            if value == "run-timed"
        )

    def clear_gate(self):
        write(self.infrared_path / "value0", 50)
        for _ in range(5):
            self.supervisor.poll_once()
        self.assertFalse(
            self.supervisor.status()["infrared"]["blocked"]
        )

    def claim_and_arm(self, clear=True):
        if clear:
            self.clear_gate()
        else:
            for _ in range(3):
                self.supervisor.poll_once()
        claimed = self.supervisor.claim("host-roamer")
        session_id = claimed["session_id"]
        self.supervisor.heartbeat(session_id, 1)
        self.supervisor.arm(session_id, 2)
        return session_id

    def start(
        self,
        session_id,
        left_speed,
        right_speed,
        command_id="command-1",
    ):
        return self.supervisor.start_drive(
            session_id,
            3,
            command_id,
            1,
            left_speed,
            right_speed,
            DRIVE_PULSE_DURATION_MS,
        )

    @staticmethod
    def wire(
        operation,
        arguments,
        request_id="request-1",
        received_at_ms=10000,
    ):
        raw = json.dumps(
            {
                "protocol_version": PROTOCOL_VERSION,
                "controller_id": CONTROLLER_ID,
                "request_id": request_id,
                "op": operation,
                "queue_ttl_ms": 500,
                "args": arguments,
            },
            separators=(",", ":"),
        ).encode("utf-8")
        return decode_request(raw, received_at_ms)

    @staticmethod
    def wire_bytes(
        operation,
        arguments=None,
        request_id="request-1",
    ):
        return (
            json.dumps(
                {
                    "protocol_version": PROTOCOL_VERSION,
                    "controller_id": CONTROLLER_ID,
                    "request_id": request_id,
                    "op": operation,
                    "queue_ttl_ms": 1000,
                    "args": {} if arguments is None else arguments,
                },
                separators=(",", ":"),
            ).encode("utf-8")
            + b"\n"
        )

    def pulse_arguments(
        self,
        action,
        sequence_id=3,
        command_id="command-1",
    ):
        return {
            "session_id": "session-ir-roamer",
            "sequence_id": sequence_id,
            "command_id": command_id,
            "reference_heartbeat_sequence": 1,
            "action": action,
        }

    def roamer_protocol(self):
        return SupervisorProtocol(
            self.supervisor,
            CONTROLLER_ID,
            allow_motion=True,
            motion_budget=DRIVE_PULSE_MAX_COMMANDS,
            runtime_profile=IR_ROAMER_RUNTIME_PROFILE,
        )

    def test_startup_is_blocked_until_stable_release(self):
        startup = self.supervisor.status()["infrared"]
        self.assertTrue(startup["blocked"])
        self.assertEqual(startup["reason"], "unverified_startup")
        self.assertFalse(startup["fresh"])

        for _ in range(4):
            self.supervisor.poll_once()
            self.assertTrue(
                self.supervisor.status()["infrared"]["blocked"]
            )
        self.supervisor.poll_once()

        released = self.supervisor.status()["infrared"]
        self.assertFalse(released["blocked"])
        self.assertEqual(
            released["reason"],
            "stable_filtered_release_returns",
        )

    def test_every_poll_reads_ir_and_status_has_only_gate_and_age_fields(self):
        with patch.object(
            self.supervisor._owner,
            "read_infrared_value",
            wraps=self.supervisor._owner.read_infrared_value,
        ) as infrared_read:
            self.supervisor.poll_once()
            self.supervisor.poll_once()
        self.assertEqual(infrared_read.call_count, 2)

        status = self.supervisor.status()["infrared"]
        self.assertEqual(
            set(status),
            {
                "raw",
                "filtered",
                "blocked",
                "reason",
                "sample_count",
                "observed_monotonic_ms",
                "age_ms",
                "fresh",
            },
        )
        self.assertNotIn("distance", status)
        self.assertNotIn("cm", status)
        self.assertTrue(status["fresh"])
        self.clock.advance(
            IR_ROAMER_POLL_INTERVAL_MS
            + IR_ROAMER_MAX_POLL_LATENESS_MS
            + 1
        )
        self.assertFalse(
            self.supervisor.status()["infrared"]["fresh"]
        )

    def test_forward_is_denied_before_stable_clear_without_motor_start(self):
        write(self.infrared_path / "value0", 10)
        session_id = self.claim_and_arm(clear=False)

        with self.assertRaises(SupervisorError) as context:
            self.start(session_id, 100, 100)

        self.assertEqual(context.exception.code, "infrared_obstacle")
        self.assertEqual(self.supervisor.state, STATE_ARMED_IDLE)
        self.assertEqual(self.run_timed_write_count(), 0)

    def test_in_place_turn_is_allowed_while_ir_is_blocked(self):
        write(self.infrared_path / "value0", 10)
        session_id = self.claim_and_arm(clear=False)

        status = self.start(session_id, -100, 100)

        self.assertEqual(status["state"], STATE_RUNNING)
        self.assertEqual(self.run_timed_write_count(), 2)

    def test_blocking_sample_stops_forward_in_same_poll_and_audits(self):
        session_id = self.claim_and_arm()
        self.start(session_id, 100, 100)
        self.assertEqual(self.supervisor.state, STATE_RUNNING)
        write(self.infrared_path / "value0", 10)

        status = self.supervisor.poll_once()

        self.assertEqual(status["state"], STATE_ARMED_IDLE)
        self.assertIsNone(status["active_command_id"])
        events = [
            event
            for event in self.supervisor.audit_events
            if event["event"] == "infrared_obstacle_stop"
        ]
        self.assertEqual(len(events), 1)
        self.assertEqual(
            events[0]["infrared"]["observed_monotonic_ms"],
            status["infrared"]["observed_monotonic_ms"],
        )
        for path in self.sysfs.motors.values():
            self.assertEqual(read_text(str(path / "command")), "stop")

    def test_ir_change_in_write_gap_is_caught_immediately_post_start(self):
        session_id = self.claim_and_arm()
        with patch.object(
            self.supervisor._owner,
            "read_infrared_value",
            side_effect=(50, 50, 50, 10),
        ):
            with self.assertRaises(SupervisorError) as context:
                self.start(session_id, 100, 100)

        self.assertEqual(context.exception.code, "infrared_obstacle")
        self.assertEqual(self.run_timed_write_count(), 2)
        self.assertEqual(self.supervisor.state, STATE_ARMED_IDLE)
        for path in self.sysfs.motors.values():
            self.assertEqual(read_text(str(path / "command")), "stop")

    def test_failed_infrared_stop_latches_fault(self):
        session_id = self.claim_and_arm()
        self.start(session_id, 100, 100)
        write(self.infrared_path / "value0", 10)
        failed = {
            "stop_confirmed": False,
            "errors": ["simulated stop failure"],
            "fault_tokens": {},
        }

        with patch.object(
            self.supervisor,
            "_finish_active_with_retry",
            return_value=(failed, None),
        ):
            status = self.supervisor.poll_once()

        self.assertEqual(status["state"], STATE_FAULT_LATCHED)
        self.assertEqual(
            status["fault"]["code"],
            "infrared_stop_failed",
        )

    def assert_ir_fault_requires_restart(self):
        status = self.supervisor.poll_once()
        self.assertEqual(status["state"], STATE_FAULT_LATCHED)
        self.assertTrue(status["infrared"]["blocked"])
        self.assertFalse(status["infrared"]["fresh"])
        with self.assertRaises(SupervisorError) as context:
            self.supervisor.reset_fault()
        self.assertEqual(
            context.exception.code,
            "supervisor_restart_required",
        )

    def test_ir_mode_fault_requires_restart(self):
        self.clear_gate()
        write(self.infrared_path / "mode", "IR-SEEK")
        self.assert_ir_fault_requires_restart()

    def test_ir_malformed_value_fault_requires_restart(self):
        self.clear_gate()
        write(self.infrared_path / "value0", "not-an-int")
        self.assert_ir_fault_requires_restart()

    def test_ir_hotplug_fault_requires_restart(self):
        self.clear_gate()
        value_path = self.infrared_path / "value0"
        value_path.unlink()
        write(value_path, 50)
        self.assert_ir_fault_requires_restart()

    def test_drive_pulse_schema_rejects_numeric_injection(self):
        valid = self.wire(
            "drive_pulse",
            self.pulse_arguments("ADVANCE"),
        )
        self.assertEqual(valid.arguments["action"], "ADVANCE")

        injected = self.pulse_arguments("ADVANCE")
        injected["left_speed_dps"] = 1
        with self.assertRaises(ProtocolError) as context:
            self.wire("drive_pulse", injected)
        self.assertEqual(
            context.exception.code,
            "invalid_arguments_fields",
        )

        invalid = self.pulse_arguments("BACKWARD")
        with self.assertRaises(ProtocolError) as context:
            self.wire("drive_pulse", invalid)
        self.assertEqual(context.exception.code, "invalid_action")

    def test_description_exposes_exact_fixed_profile(self):
        protocol = self.roamer_protocol()
        description = protocol._description()

        self.assertEqual(IR_ROAMER_POLL_INTERVAL_MS, 150)
        self.assertEqual(
            description["runtime_profile"],
            IR_ROAMER_RUNTIME_PROFILE,
        )
        self.assertTrue(description["motion_enabled"])
        self.assertEqual(description["remaining_motion_budget"], 20)
        self.assertEqual(
            description["remaining_motion_duration_ms"],
            3000,
        )
        self.assertEqual(
            description["max_session_ms"],
            IR_ROAMER_MAX_SESSION_MS,
        )
        self.assertEqual(
            description["poll_interval_ms"],
            IR_ROAMER_POLL_INTERVAL_MS,
        )
        self.assertEqual(
            description["max_poll_lateness_ms"],
            IR_ROAMER_MAX_POLL_LATENESS_MS,
        )
        self.assertEqual(
            description["max_process_requests"],
            IR_ROAMER_MAX_PROCESS_REQUESTS,
        )
        self.assertFalse(
            description["capabilities"][
                "differential_drive_timed"
            ]["enabled"]
        )
        self.assertEqual(
            description["capabilities"]["semantic_drive_pulse"],
            {
                "enabled": True,
                "actions": [
                    "ADVANCE",
                    "TURN_LEFT",
                    "TURN_RIGHT",
                ],
                "mapping": {
                    "ADVANCE": {
                        "left_speed_dps": 100,
                        "right_speed_dps": 100,
                    },
                    "TURN_LEFT": {
                        "left_speed_dps": -100,
                        "right_speed_dps": 100,
                    },
                    "TURN_RIGHT": {
                        "left_speed_dps": 100,
                        "right_speed_dps": -100,
                    },
                },
                "duration_ms": 150,
                "max_commands_per_process": 20,
                "max_total_duration_ms": 3000,
            },
        )

    def test_semantic_actions_map_to_fixed_pulses_and_exhaust_budgets(self):
        protocol = self.roamer_protocol()
        captured = []

        def capture(*arguments, **keywords):
            captured.append((arguments, keywords))
            return {"state": STATE_RUNNING}

        with patch.object(
            self.supervisor,
            "start_drive",
            side_effect=capture,
        ):
            for index, action in enumerate(
                ("ADVANCE", "TURN_LEFT", "TURN_RIGHT")
            ):
                request = self.wire(
                    "drive_pulse",
                    self.pulse_arguments(
                        action,
                        sequence_id=index + 3,
                        command_id="command-{}".format(index),
                    ),
                    request_id="request-{}".format(index),
                )
                response = protocol.execute(
                    request,
                    dispatch_at_ms=self.clock.now_ms,
                )
                self.assertTrue(response["ok"])

        self.assertEqual(
            [
                call[0][4:7]
                for call in captured
            ],
            [
                (100, 100, 150),
                (-100, 100, 150),
                (100, -100, 150),
            ],
        )

        protocol = self.roamer_protocol()
        with patch.object(
            self.supervisor,
            "start_drive",
            return_value={"state": STATE_RUNNING},
        ):
            for index in range(20):
                request = self.wire(
                    "drive_pulse",
                    self.pulse_arguments(
                        "TURN_LEFT",
                        sequence_id=index + 3,
                        command_id="budget-{}".format(index),
                    ),
                    request_id="budget-request-{}".format(index),
                )
                self.assertTrue(
                    protocol.execute(
                        request,
                        dispatch_at_ms=self.clock.now_ms,
                    )["ok"]
                )
            exhausted = self.wire(
                "drive_pulse",
                self.pulse_arguments(
                    "TURN_LEFT",
                    sequence_id=23,
                    command_id="budget-exhausted",
                ),
                request_id="budget-request-exhausted",
            )
            response = protocol.execute(
                exhausted,
                dispatch_at_ms=self.clock.now_ms,
            )

        self.assertFalse(response["ok"])
        self.assertEqual(
            response["error"]["code"],
            "motion_budget_exhausted",
        )
        self.assertEqual(protocol.remaining_motion_budget, 0)
        self.assertEqual(protocol.remaining_motion_duration_ms, 0)

    def test_daemon_default_is_motion_free_and_profile_is_explicit(self):
        parser = build_parser()
        defaults = parser.parse_args([])
        roamer = parser.parse_args(
            ["--profile", IR_ROAMER_RUNTIME_PROFILE]
        )
        self.assertEqual(defaults.profile, "motion-free")
        self.assertEqual(
            roamer.profile,
            IR_ROAMER_RUNTIME_PROFILE,
        )
        self.assertIsNone(defaults.max_session_ms)
        with self.assertRaises(SessionError) as context:
            run_daemon(
                None,
                None,
                None,
                None,
                max_session_ms=IR_ROAMER_MAX_SESSION_MS + 1,
                runtime_profile=IR_ROAMER_RUNTIME_PROFILE,
            )
        self.assertEqual(
            context.exception.code,
            "invalid_session_deadline",
        )
        with self.assertRaises(SessionError) as context:
            run_daemon(
                None,
                None,
                None,
                None,
                max_session_ms=15000,
                runtime_profile=IR_ROAMER_RUNTIME_PROFILE,
            )
        self.assertEqual(
            context.exception.code,
            "invalid_session_deadline",
        )

    def test_foreground_request_budget_is_fixed_per_profile(self):
        protocol = self.roamer_protocol()
        session = ForegroundSupervisorSession(
            self.supervisor,
            protocol,
            InteractiveInput(),
            InteractiveOutput(),
            max_session_ms=IR_ROAMER_MAX_SESSION_MS,
            max_process_requests=IR_ROAMER_MAX_PROCESS_REQUESTS,
        )
        self.assertEqual(
            session.max_process_requests,
            IR_ROAMER_MAX_PROCESS_REQUESTS,
        )
        session._request_count = IR_ROAMER_MAX_PROCESS_REQUESTS
        self.assertFalse(
            session.enqueue_request(
                self.wire(
                    "status",
                    {},
                    request_id="ir-over-budget",
                )
            )
        )
        self.assertEqual(
            session.transport_failure,
            "request_budget_exhausted",
        )
        with self.assertRaises(SessionError) as context:
            ForegroundSupervisorSession(
                self.supervisor,
                protocol,
                InteractiveInput(),
                InteractiveOutput(),
                max_session_ms=IR_ROAMER_MAX_SESSION_MS,
                max_process_requests=MOTION_FREE_MAX_PROCESS_REQUESTS,
            )
        self.assertEqual(
            context.exception.code,
            "invalid_request_budget",
        )

        self.supervisor.close()
        self.supervisor = EV3Supervisor(self.hal)
        motion_free = SupervisorProtocol(
            self.supervisor,
            CONTROLLER_ID,
        )
        self.assertEqual(MAX_PROCESS_REQUESTS, 128)
        self.assertEqual(
            motion_free._description()["max_process_requests"],
            128,
        )
        default_session = ForegroundSupervisorSession(
            self.supervisor,
            motion_free,
            InteractiveInput(),
            InteractiveOutput(),
        )
        self.assertEqual(default_session.max_process_requests, 128)
        default_session._request_count = 127
        self.assertTrue(
            default_session.enqueue_request(
                self.wire(
                    "status",
                    {},
                    request_id="motion-free-128",
                )
            )
        )
        queued = default_session._normal_queue.get_nowait()
        self.assertIsNotNone(queued.request)
        default_session._normal_queue.task_done()
        self.assertFalse(
            default_session.enqueue_request(
                self.wire(
                    "status",
                    {},
                    request_id="motion-free-129",
                )
            )
        )
        self.assertEqual(
            default_session.transport_failure,
            "request_budget_exhausted",
        )

    def test_default_motion_free_supervisor_requires_fixed_in4_ir(self):
        self.supervisor.close()
        write(self.infrared_path / "mode", "IR-SEEK")
        self.supervisor = EV3Supervisor(self.hal)

        status = self.supervisor.status()
        self.assertEqual(status["state"], STATE_FAULT_LATCHED)
        self.assertEqual(status["fault"]["code"], "startup_stop_failed")
        self.assertEqual(self.run_timed_write_count(), 0)

    def test_real_pipe_daemon_roamer_describe_handshake_is_motion_free(self):
        self.supervisor.close()
        request_read, request_write = os.pipe()
        response_read, response_write = os.pipe()
        daemon_input = os.fdopen(request_read, "rb", buffering=0)
        daemon_output = os.fdopen(response_write, "wb", buffering=0)
        client_input = os.fdopen(response_read, "rb", buffering=0)
        client_output = os.fdopen(request_write, "wb", buffering=0)
        responses = []
        client_errors = []

        class AuditCollector(object):
            def __init__(self):
                self.events = []

            def append(self, event):
                self.events.append(event)

        audit = AuditCollector()

        def client():
            try:
                client_output.write(
                    self.wire_bytes(
                        "describe",
                        request_id="describe-roamer",
                    )
                )
                client_output.flush()
                responses.append(
                    json.loads(client_input.readline().decode("utf-8"))
                )
                client_output.write(
                    self.wire_bytes(
                        "shutdown",
                        request_id="shutdown-roamer",
                    )
                )
                client_output.flush()
                responses.append(
                    json.loads(client_input.readline().decode("utf-8"))
                )
            except BaseException as error:
                client_errors.append(error)
            finally:
                client_output.close()
                client_input.close()

        client_thread = threading.Thread(target=client)
        client_thread.start()
        try:
            result = run_daemon(
                self.hal,
                audit,
                daemon_input,
                daemon_output,
                max_session_ms=IR_ROAMER_MAX_SESSION_MS,
                runtime_profile=IR_ROAMER_RUNTIME_PROFILE,
            )
        finally:
            daemon_input.close()
            daemon_output.close()
        client_thread.join(2)

        self.assertFalse(client_thread.is_alive())
        self.assertFalse(client_errors)
        self.assertEqual(
            [response["request_id"] for response in responses],
            ["describe-roamer", "shutdown-roamer"],
        )
        self.assertTrue(responses[0]["ok"])
        description = responses[0]["result"]
        self.assertEqual(
            description["runtime_profile"],
            IR_ROAMER_RUNTIME_PROFILE,
        )
        self.assertEqual(
            description["max_process_requests"],
            IR_ROAMER_MAX_PROCESS_REQUESTS,
        )
        self.assertEqual(description["remaining_motion_budget"], 20)
        self.assertEqual(
            description["remaining_motion_duration_ms"],
            3000,
        )
        self.assertTrue(responses[1]["ok"])
        self.assertEqual(result["max_process_requests"], 256)
        self.assertEqual(self.run_timed_write_count(), 0)
        self.assertTrue(audit.events)

    def test_close_releases_ir_readers_and_is_idempotent(self):
        descriptors = [
            reader.descriptor
            for reader in self.supervisor._owner._infrared_binding[
                "readers"
            ].values()
        ]
        first = self.supervisor.close()
        second = self.supervisor.close()

        self.assertEqual(first, second)
        for descriptor in descriptors:
            with self.assertRaises(OSError):
                supervisor_module.os.fstat(descriptor)


if __name__ == "__main__":
    unittest.main()
