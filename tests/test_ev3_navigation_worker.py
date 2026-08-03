import io
import json
import os
import unittest
from unittest.mock import patch

from ev3.infrared_safety import (
    InfraredGatePolicy,
    InfraredObstacleGate,
)
from ev3.encoder_recovery import EncoderRecoveryPolicy
from ev3.navigation_profile import (
    ACTION_SPECS,
    MAX_PULSES,
    REQUEST_SCHEMA,
    RESPONSE_SCHEMA,
    SCAN_SAMPLE_COUNT,
    SCAN_SAMPLE_FILTER_WINDOW,
    SCAN_SAMPLE_SETTLED_DURATION_MS,
    SCAN_TURN_ALLOWED_DELTAS_MDEG,
    WORKER_ID,
    scan_turn_profile,
    scan_turn_spec,
    scan_sample_profile,
    validate_action_specs,
)
from ev3.navigation_worker import NavigationWorker
from ev3.navigation_worker_cli import (
    _binary_input_stream,
    _input_cancel_requested,
)
from ev3.navigation_worker_protocol import (
    WorkerError,
    decode_request,
    response_object,
    write_response,
)


CONTROLLER_ID = "ev3rstorm-01.ev3-main"


class Clock(object):
    def __init__(self):
        self.value = 100.0

    def monotonic(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class FakeRobot(object):
    def __init__(self, clock, owner):
        self.clock = clock
        self.owner = owner

    def sleep_fn(self, seconds):
        self.owner.advance(seconds)
        self.clock.advance(seconds)


class FakeOwner(object):
    def __init__(self, clock):
        self.clock = clock
        self.robot = FakeRobot(clock, self)
        self.touch = 0
        self.infrared = 50
        self.state = ""
        self.positions = {
            "arm": 0,
            "drive_b": 0,
            "drive_c": 0,
        }
        self.speeds = (0, 0)
        self.finish_count = 0
        self.close_count = 0
        self.force_verification_error = False
        self.blocked_roles = set()
        self.wrong_direction_roles = set()
        self.transient_fault_roles = set()
        self.transient_fault_token = "stalled"
        self.transient_fault_after_leader_degrees = None
        self.fail_reads = False
        self.closed = False
        self.validated = []
        self.position_before = {
            "drive_b": 0,
            "drive_c": 0,
        }
        self.active_sides = ("left", "right")
        self.after_first_start = None
        self.after_single_start = None

    def _attach_start_evidence(
        self,
        error,
        duration_ms,
        target_sides,
        started_sides,
        started_ms,
    ):
        roles = {"left": "drive_b", "right": "drive_c"}
        stop = {
            "stop_confirmed": True,
            "errors": [],
            "fault_tokens": {},
        }
        error.supervisor_start_cleanup = stop
        error.supervisor_start_evidence = {
            "complete": True,
            "duration_ms": duration_ms,
            "started_at_ms": started_ms if started_sides else None,
            "completed_at_ms": int(self.clock.value * 1000),
            "started_sides": list(started_sides),
            "start_write_windows": [
                {
                    "side": side,
                    "role": roles[side],
                    "begin_ms": started_ms,
                    "end_ms": started_ms,
                }
                for side in started_sides
            ],
            "motors": [
                {
                    "side": side,
                    "role": roles[side],
                    "physical_speed_dps": (
                        self.speeds[0] if side == "left" else self.speeds[1]
                    ),
                    "position_before": self.position_before[roles[side]],
                    "position_after": self.positions[roles[side]],
                    "position_delta": (
                        self.positions[roles[side]]
                        - self.position_before[roles[side]]
                    ),
                    "state": "",
                }
                for side in target_sides
            ],
            "stop": stop,
        }

    def read_touch_value(self):
        if self.fail_reads:
            raise IOError("reads disabled")
        return self.touch

    def read_infrared_value(self):
        if self.fail_reads:
            raise IOError("reads disabled")
        return self.infrared

    def snapshot_all(self):
        if self.fail_reads:
            raise IOError("reads disabled")
        leader_progress = max(
            abs(
                self.positions[role]
                - self.position_before.get(role, self.positions[role])
            )
            for role in ("drive_b", "drive_c")
        )
        transient_fault_active = (
            self.state
            and self.transient_fault_after_leader_degrees is not None
            and leader_progress
            >= self.transient_fault_after_leader_degrees
        )
        return [
            {
                "role": role,
                "position": position,
                "state": (
                    "{} {}".format(
                        self.state,
                        self.transient_fault_token,
                    ).strip()
                    if role in self.transient_fault_roles
                    and transient_fault_active
                    else self.state if role != "arm" else ""
                ),
            }
            for role, position in sorted(self.positions.items())
        ]

    def validate_drive(self, left_speed, right_speed, duration_ms):
        self.validated.append(
            (left_speed, right_speed, duration_ms)
        )

    def start_drive(
        self,
        left_speed,
        right_speed,
        duration_ms,
        pre_each_start=None,
    ):
        self.position_before = {
            "drive_b": self.positions["drive_b"],
            "drive_c": self.positions["drive_c"],
        }
        self.speeds = (left_speed, right_speed)
        started = int(self.clock.value * 1000)
        if pre_each_start is not None:
            try:
                pre_each_start({}, ())
            except Exception as error:
                self._attach_start_evidence(
                    error, duration_ms, ("left", "right"), (), started
                )
                raise
            try:
                if self.after_first_start is not None:
                    self.after_first_start()
                pre_each_start(
                    {},
                    ({"begin_ms": started},),
                )
            except Exception as error:
                self._attach_start_evidence(
                    error,
                    duration_ms,
                    ("left", "right"),
                    ("left",),
                    started,
                )
                raise
        self.active_sides = ("left", "right")
        self.state = "running"
        return {
            "started_at_ms": started,
            "deadline_ms": started + duration_ms,
            "duration_ms": duration_ms,
            "motors": [
                {
                    "side": "left",
                    "role": "drive_b",
                    "physical_speed_dps": left_speed,
                    "position_before": self.position_before["drive_b"],
                },
                {
                    "side": "right",
                    "role": "drive_c",
                    "physical_speed_dps": right_speed,
                    "position_before": self.position_before["drive_c"],
                },
            ],
        }

    def start_drive_side(
        self,
        side,
        speed,
        duration_ms,
        pre_each_start=None,
    ):
        self.position_before = {
            "drive_b": self.positions["drive_b"],
            "drive_c": self.positions["drive_c"],
        }
        if side == "left":
            self.speeds = (speed, 0)
            role = "drive_b"
        else:
            self.speeds = (0, speed)
            role = "drive_c"
        started = int(self.clock.value * 1000)
        if pre_each_start is not None:
            try:
                pre_each_start({}, ())
            except Exception as error:
                self._attach_start_evidence(
                    error, duration_ms, (side,), (), started
                )
                raise
        if self.after_single_start is not None:
            try:
                self.after_single_start()
            except Exception as error:
                self._attach_start_evidence(
                    error, duration_ms, (side,), (side,), started
                )
                raise
        self.active_sides = (side,)
        self.state = "running"
        return {
            "started_at_ms": started,
            "deadline_ms": started + duration_ms,
            "duration_ms": duration_ms,
            "motors": [
                {
                    "side": side,
                    "role": role,
                    "physical_speed_dps": speed,
                    "position_before": self.position_before[role],
                }
            ],
        }

    def advance(self, seconds):
        for role, speed in zip(
            ("drive_b", "drive_c"),
            self.speeds,
        ):
            if role in self.blocked_roles:
                continue
            direction = -1 if role in self.wrong_direction_roles else 1
            self.positions[role] += direction * int(
                round(speed * seconds)
            )

    def finish_active(self, verify_motion):
        self.finish_count += 1
        self.state = ""
        all_motors = [
            {
                "side": "left",
                "role": "drive_b",
                "position_before": self.position_before["drive_b"],
                "position_after": self.positions["drive_b"],
                "position_delta": (
                    self.positions["drive_b"]
                    - self.position_before["drive_b"]
                ),
                "state": "",
            },
            {
                "side": "right",
                "role": "drive_c",
                "position_before": self.position_before["drive_c"],
                "position_after": self.positions["drive_c"],
                "position_delta": (
                    self.positions["drive_c"]
                    - self.position_before["drive_c"]
                ),
                "state": "",
            },
        ]
        side_speeds = {
            "left": self.speeds[0],
            "right": self.speeds[1],
        }
        motors = [
            motor
            for motor in all_motors
            if motor["side"] in self.active_sides
        ]
        checks = []
        for motor in motors:
            speed = side_speeds[motor["side"]]
            passed = (
                motor["position_delta"] * speed > 0
                and abs(motor["position_delta"]) >= 3
                and not self.force_verification_error
            )
            checks.append({"passed": passed})
        result = {
            "stop": {
                "stop_confirmed": True,
                "errors": [],
                "fault_tokens": {},
            },
            "motors": motors,
            "checks": checks,
        }
        if verify_motion and any(not check["passed"] for check in checks):
            result["verification_error"] = (
                "simulated encoder mismatch"
            )
        self.speeds = (0, 0)
        return result

    def close(self):
        self.close_count += 1
        self.closed = True
        return {
            "stop_confirmed": True,
            "errors": [],
            "fault_tokens": {},
            "states": {},
            "positions": {},
        }

    def path_for_role(self, role):
        return "/fake/{0}".format(role)


def request_frame(
    request_id="request-1",
    operation="observe",
    arguments=None,
    controller_id=CONTROLLER_ID,
):
    return (
        json.dumps(
            {
                "schema": REQUEST_SCHEMA,
                "controller_id": controller_id,
                "request_id": request_id,
                "op": operation,
                "args": arguments if arguments is not None else {},
            }
        ).encode("utf-8")
        + b"\n"
    )


class EV3NavigationWorkerProtocolTests(unittest.TestCase):
    def test_input_channel_readability_or_eof_requests_cancellation(self):
        read_descriptor, write_descriptor = os.pipe()
        raw_reader = os.fdopen(read_descriptor, "rb", buffering=0)
        buffered_reader = io.BufferedReader(raw_reader)
        text_reader = io.TextIOWrapper(buffered_reader)
        writer = os.fdopen(write_descriptor, "wb", buffering=0)
        try:
            reader = _binary_input_stream(text_reader)
            self.assertIs(reader, raw_reader)
            self.assertFalse(
                _input_cancel_requested(reader, [False])
            )
            writer.write(b"first\nstop\n")
            self.assertEqual(reader.readline(), b"first\n")
            self.assertTrue(
                _input_cancel_requested(reader, [False])
            )
            self.assertEqual(reader.readline(), b"stop\n")
            self.assertFalse(
                _input_cancel_requested(reader, [False])
            )
            writer.close()
            self.assertTrue(
                _input_cancel_requested(reader, [False])
            )
        finally:
            text_reader.close()
            if not writer.closed:
                writer.close()

    def test_signal_requests_cancellation_without_reading_input(self):
        self.assertTrue(
            _input_cancel_requested(object(), [True])
        )

    def test_strict_request_is_correlated(self):
        request = decode_request(
            request_frame(
                operation="pulse",
                arguments={"action": "ADVANCE"},
            ),
            CONTROLLER_ID,
        )
        self.assertEqual(request["request_id"], "request-1")
        response = response_object(
            CONTROLLER_ID,
            request["request_id"],
            True,
            {"observation": {"state_version": 17}},
            17,
        )
        self.assertEqual(response["schema"], RESPONSE_SCHEMA)
        self.assertEqual(response["request_id"], "request-1")
        self.assertEqual(response["state_version"], 17)

    def test_duplicate_json_key_is_fatal(self):
        raw = (
            b'{"schema":"ev3-agent-worker-request/v1",'
            b'"schema":"ev3-agent-worker-request/v1",'
            b'"controller_id":"ev3rstorm-01.ev3-main",'
            b'"request_id":"request-1","op":"observe","args":{}}\n'
        )
        with self.assertRaises(WorkerError) as raised:
            decode_request(raw, CONTROLLER_ID)
        self.assertEqual(raised.exception.code, "invalid_json")
        self.assertTrue(raised.exception.fatal)

    def test_invalid_action_preserves_request_id(self):
        with self.assertRaises(WorkerError) as raised:
            decode_request(
                request_frame(
                    operation="pulse",
                    arguments={"action": "NOT VALID"},
                ),
                CONTROLLER_ID,
            )
        self.assertEqual(raised.exception.code, "invalid_action")
        self.assertEqual(
            raised.exception.request_id,
            "request-1",
        )

    def test_scan_turn_accepts_only_the_fixed_host_lattice(self):
        accepted = decode_request(
            request_frame(
                operation="scan_turn",
                arguments={"relative_delta_mdeg": -15_000},
            ),
            CONTROLLER_ID,
        )
        self.assertEqual(
            accepted["args"]["relative_delta_mdeg"],
            -15_000,
        )
        for invalid in (0, 10_000, 135_000, True, "15000"):
            with self.subTest(invalid=invalid):
                with self.assertRaises(WorkerError) as raised:
                    decode_request(
                        request_frame(
                            operation="scan_turn",
                            arguments={
                                "relative_delta_mdeg": invalid
                            },
                        ),
                        CONTROLLER_ID,
                    )
                self.assertEqual(
                    raised.exception.code,
                    "invalid_scan_turn",
                )

    def test_wrong_controller_does_not_become_motion(self):
        with self.assertRaises(WorkerError) as raised:
            decode_request(
                request_frame(controller_id="another-controller"),
                CONTROLLER_ID,
            )
        self.assertEqual(raised.exception.code, "wrong_controller")
        self.assertFalse(raised.exception.fatal)

    def test_response_is_one_canonical_jsonl_frame(self):
        stream = io.BytesIO()
        write_response(
            response_object(
                CONTROLLER_ID,
                "request-1",
                True,
                {"value": 1},
                7,
            ),
            output_stream=stream,
        )
        raw = stream.getvalue()
        self.assertEqual(raw.count(b"\n"), 1)
        self.assertTrue(raw.endswith(b"\n"))
        self.assertEqual(
            json.loads(raw.decode("utf-8"))["state_version"],
            7,
        )


class EV3NavigationProfileTests(unittest.TestCase):
    def test_fixed_profile_matches_validated_semantic_actions(self):
        self.assertEqual(
            set(ACTION_SPECS),
            {
                "ADVANCE",
                "REVERSE",
                "TURN_LEFT_90",
                "TURN_RIGHT_90",
            },
        )
        self.assertEqual(
            ACTION_SPECS["ADVANCE"]["slice_durations_ms"],
            [250],
        )
        self.assertEqual(ACTION_SPECS["ADVANCE"]["left_speed_dps"], 800)
        self.assertEqual(ACTION_SPECS["ADVANCE"]["right_speed_dps"], 800)
        self.assertEqual(
            ACTION_SPECS["TURN_LEFT_90"]["slice_durations_ms"],
            [800, 800, 800, 160],
        )
        self.assertEqual(
            ACTION_SPECS["TURN_RIGHT_90"][
                "target_mean_abs_encoder_degrees"
            ],
            682,
        )

    def test_every_slice_is_validated_without_starting_motion(self):
        owner = FakeOwner(Clock())
        validate_action_specs(owner)
        expected_count = sum(
            spec["slice_count"] for spec in ACTION_SPECS.values()
        )
        expected_count += sum(
            scan_turn_spec(delta)["slice_count"]
            for delta in SCAN_TURN_ALLOWED_DELTAS_MDEG
        )
        self.assertEqual(len(owner.validated), expected_count)
        self.assertEqual(owner.finish_count, 0)

    def test_scan_profile_is_provisional_bounded_and_symmetric(self):
        profile = scan_turn_profile()
        self.assertEqual(
            profile["calibration"],
            "provisional_live_encoder_derived",
        )
        self.assertEqual(
            profile["allowed_relative_deltas_mdeg"],
            list(SCAN_TURN_ALLOWED_DELTAS_MDEG),
        )
        self.assertEqual(
            scan_turn_spec(30_000)["slice_durations_ms"],
            [427, 426],
        )
        for delta in SCAN_TURN_ALLOWED_DELTAS_MDEG:
            spec = scan_turn_spec(delta)
            mirrored = scan_turn_spec(-delta)
            self.assertTrue(
                all(
                    0 < duration <= 800
                    for duration in spec["slice_durations_ms"]
                )
            )
            self.assertEqual(
                spec["slice_durations_ms"],
                mirrored["slice_durations_ms"],
            )
            self.assertEqual(
                spec["left_speed_dps"],
                -mirrored["left_speed_dps"],
            )


class EV3NavigationWorkerSafetyTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.time_patch = patch(
            "ev3.navigation_worker.time.monotonic",
            side_effect=self.clock.monotonic,
        )
        self.time_patch.start()
        self.worker, self.owner = self.build_worker()

    def tearDown(self):
        self.time_patch.stop()

    def build_worker(self, cancel_requested=None):
        owner = FakeOwner(self.clock)
        worker = NavigationWorker.__new__(NavigationWorker)
        worker.owner = owner
        worker.robot = owner.robot
        worker.robot.config = {
            "drive_geometry": {
                "left_motor_role": "drive_b",
                "right_motor_role": "drive_c",
                "forward_speed_sign": {
                    "drive_b": 1,
                    "drive_c": 1,
                },
            }
        }
        worker.controller_id = CONTROLLER_ID
        worker.gate = InfraredObstacleGate(
            InfraredGatePolicy(16, 35, 40, 3, 2, 3)
        )
        worker.started_at = self.clock.value
        worker.deadline = self.clock.value + 45
        worker.state_version = 0
        worker.request_count = 0
        worker.pulse_count = 0
        worker.pulse_duration_ms = 0
        worker.encoder_recovery_policy = EncoderRecoveryPolicy(
            minimum_progress_degrees=3,
            catch_up_leader_minimum_degrees=12,
            acceptable_completion_percent=75,
            maximum_progress_skew_percent=15,
            maximum_catch_up_attempts=2,
            maximum_pair_retry_attempts=1,
            maximum_total_attempts=3,
            maximum_step_duration_ms=800,
            maximum_total_recovery_duration_ms=1600,
            maximum_total_recovery_encoder_degrees=400,
        )
        worker.motion_fault_latched = False
        worker.last_outcome = {
            "kind": "startup",
            "status": "ready",
        }
        worker.last_observation = None
        worker.seen_request_ids = set()
        worker.closed = False
        worker.shutdown_requested = False
        worker._cancel_requested = (
            cancel_requested
            if cancel_requested is not None
            else lambda: False
        )
        for _index in range(5):
            worker._observe()
        return worker, owner

    def test_touch_denies_every_semantic_motion(self):
        self.owner.touch = 1
        denied = self.worker._pulse("TURN_LEFT_90")
        self.assertEqual(denied["outcome"]["status"], "denied")
        self.assertEqual(
            denied["outcome"]["reason"],
            "touch_pressed",
        )
        self.assertTrue(denied["stop"]["stop_confirmed"])
        self.assertEqual(self.worker.pulse_count, 0)

    def test_infrared_denies_advance_but_not_reverse(self):
        self.owner.infrared = 10
        denied = self.worker._pulse("ADVANCE")
        self.assertEqual(
            denied["outcome"]["reason"],
            "infrared_blocked",
        )
        reversed_result = self.worker._pulse("REVERSE")
        self.assertEqual(
            reversed_result["outcome"]["status"],
            "completed",
        )

    def test_infrared_does_not_block_scan_turn(self):
        self.owner.infrared = 10
        result = self.worker._scan_turn(15_000)
        self.assertEqual(result["outcome"]["status"], "completed")
        self.assertTrue(result["stop"]["stop_confirmed"])
        self.assertTrue(
            result["outcome"]["encoder_verification"]["passed"]
        )

    def test_scan_sample_discards_motion_history_and_settles_fresh_batch(self):
        for _index in range(5):
            self.worker.gate.observe(5)
        self.owner.infrared = 70
        started = self.clock.monotonic()

        result = self.worker._scan_sample()

        self.assertEqual(result["sample_count"], SCAN_SAMPLE_COUNT)
        self.assertEqual(result["raw_samples"], [70] * SCAN_SAMPLE_COUNT)
        self.assertEqual(
            result["observation"]["infrared"]["sample_count"],
            SCAN_SAMPLE_COUNT,
        )
        self.assertEqual(result["observation"]["infrared"]["filtered"], 70)
        self.assertFalse(result["observation"]["infrared"]["blocked"])
        self.assertGreaterEqual(
            int(round((self.clock.monotonic() - started) * 1000)),
            SCAN_SAMPLE_SETTLED_DURATION_MS,
        )
        self.assertTrue(result["stop"]["stop_confirmed"])

    def test_scan_sample_filtered_value_uses_published_tail_window(self):
        raw_samples = [33, 33, 34, 34, 33]
        remaining = iter(raw_samples)
        self.owner.read_infrared_value = lambda: next(remaining)

        result = self.worker._scan_sample()

        self.assertEqual(result["raw_samples"], raw_samples)
        self.assertEqual(SCAN_SAMPLE_FILTER_WINDOW, 3)
        self.assertEqual(
            scan_sample_profile()["filter_window_samples"],
            SCAN_SAMPLE_FILTER_WINDOW,
        )
        self.assertEqual(result["observation"]["infrared"]["filtered"], 34)

    def test_scan_turn_uses_fixed_slices_for_required_angles(self):
        for delta in (-60_000, -30_000, -15_000, 15_000, 30_000, 60_000):
            with self.subTest(delta=delta):
                worker, owner = self.build_worker()
                result = worker._scan_turn(delta)
                spec = scan_turn_spec(delta)
                self.assertEqual(
                    [
                        item["duration_ms"]
                        for item in result["outcome"]["slices"]
                    ],
                    spec["slice_durations_ms"],
                )
                self.assertEqual(
                    worker.pulse_count,
                    spec["slice_count"],
                )
                self.assertEqual(owner.state, "")

    def test_scan_turn_undertravel_is_a_canonical_stopped_failure(self):
        self.owner.blocked_roles.add("drive_c")
        single_wheel_starts = []
        self.owner.after_single_start = lambda: single_wheel_starts.append(
            True
        )

        result = self.worker._scan_turn(-30_000)

        self.assertEqual(result["outcome"]["status"], "verification_failed")
        self.assertEqual(
            result["outcome"]["reason"],
            "encoder_verification_failed",
        )
        self.assertEqual(result["outcome"]["completed_slice_count"], 0)
        self.assertEqual(len(result["outcome"]["slices"]), 1)
        self.assertFalse(
            result["outcome"]["slices"][0]["encoder_verification"][
                "passed"
            ]
        )
        self.assertTrue(result["stop"]["stop_confirmed"])
        self.assertEqual(single_wheel_starts, [])
        self.assertEqual(self.worker.pulse_count, 1)
        self.assertTrue(self.worker.motion_fault_latched)
        self.assertEqual(self.owner.state, "")

    def test_scan_turn_clean_completion_is_paired_and_symmetric(self):
        single_wheel_starts = []
        self.owner.after_single_start = lambda: single_wheel_starts.append(
            True
        )
        result = self.worker._scan_turn(-30_000)

        self.assertEqual(result["outcome"]["status"], "completed")
        receipt = result["outcome"]["slices"][0]
        self.assertEqual(receipt["reason"], "duration_elapsed")
        self.assertNotIn("segments", receipt)
        self.assertTrue(receipt["encoder_verification"]["passed"])
        deltas = [
            motor["position_delta"] for motor in receipt["motors"]
        ]
        self.assertEqual(deltas[0], -deltas[1])
        self.assertEqual(single_wheel_starts, [])
        self.assertFalse(self.worker.motion_fault_latched)
        self.assertEqual(self.owner.state, "")

    def test_scan_turn_wrong_direction_never_enters_recovery(self):
        self.owner.wrong_direction_roles.add("drive_c")

        result = self.worker._scan_turn(-30_000)

        self.assertEqual(result["outcome"]["status"], "verification_failed")
        self.assertEqual(
            result["outcome"]["reason"],
            "encoder_verification_failed",
        )
        self.assertEqual(result["outcome"]["completed_slice_count"], 0)
        self.assertTrue(result["stop"]["stop_confirmed"])
        self.assertTrue(self.worker.motion_fault_latched)
        self.assertEqual(self.owner.state, "")

    def test_touch_and_cancellation_interrupt_scan_turn(self):
        self.owner.touch = 1
        denied = self.worker._scan_turn(30_000)
        self.assertEqual(denied["outcome"]["reason"], "touch_pressed")
        self.assertEqual(self.worker.pulse_count, 0)

        cancelled = [False]
        worker, owner = self.build_worker(
            cancel_requested=lambda: cancelled[0]
        )
        original_sleep = owner.robot.sleep_fn

        def cancelling_sleep(seconds):
            original_sleep(seconds)
            cancelled[0] = True

        owner.robot.sleep_fn = cancelling_sleep
        interrupted = worker._scan_turn(-60_000)
        self.assertEqual(
            interrupted["outcome"]["reason"],
            "cancel_requested",
        )
        self.assertEqual(
            interrupted["outcome"]["completed_slice_count"],
            0,
        )
        self.assertTrue(interrupted["stop"]["stop_confirmed"])
        self.assertEqual(owner.state, "")

    def test_completed_advance_is_stopped_and_encoder_verified(self):
        completed = self.worker._pulse("ADVANCE")
        self.assertEqual(
            completed["outcome"]["status"],
            "completed",
        )
        self.assertTrue(
            completed["outcome"]["encoder_verification"]["passed"]
        )
        self.assertTrue(completed["stop"]["stop_confirmed"])
        self.assertEqual(self.worker.pulse_count, 1)
        self.assertEqual(self.worker.pulse_duration_ms, 250)

    def test_turn_runs_exact_slices_and_counts_each_slice(self):
        result = self.worker._pulse("TURN_RIGHT_90")
        self.assertEqual(result["outcome"]["status"], "completed")
        self.assertEqual(
            [
                row["duration_ms"]
                for row in result["outcome"]["slices"]
            ],
            [800, 800, 800, 160],
        )
        self.assertEqual(self.worker.pulse_count, 4)
        self.assertEqual(self.worker.pulse_duration_ms, 2560)

    def test_cancellation_interrupts_an_active_slice_and_stops(self):
        cancelled = [False]
        self.worker._cancel_requested = lambda: cancelled[0]
        original_sleep = self.owner.robot.sleep_fn

        def cancelling_sleep(seconds):
            original_sleep(seconds)
            cancelled[0] = True

        self.owner.robot.sleep_fn = cancelling_sleep
        result = self.worker._pulse("ADVANCE")

        self.assertEqual(result["outcome"]["status"], "interrupted")
        self.assertEqual(
            result["outcome"]["reason"],
            "cancel_requested",
        )
        self.assertTrue(result["stop"]["stop_confirmed"])
        self.assertTrue(self.worker.shutdown_requested)
        self.assertEqual(self.worker.pulse_count, 1)
        self.assertEqual(self.owner.state, "")

    def test_partial_primary_start_preserves_encoder_motion(self):
        cancelled = [False]
        self.worker._cancel_requested = lambda: cancelled[0]

        def move_left_then_cancel():
            self.owner.positions["drive_b"] += 24
            cancelled[0] = True

        self.owner.after_first_start = move_left_then_cancel
        result = self.worker._pulse("ADVANCE")

        receipt = result["outcome"]["slices"][0]
        self.assertEqual(result["outcome"]["status"], "interrupted")
        self.assertEqual(receipt["status"], "interrupted")
        self.assertEqual(
            [motor["position_delta"] for motor in receipt["motors"]],
            [24, 0],
        )
        self.assertEqual(len(receipt["segments"]), 1)
        self.assertEqual(receipt["segments"][0]["kind"], "partial_start")
        self.assertEqual(
            receipt["segments"][0]["commanded_sides"], ["left"]
        )
        self.assertTrue(receipt["stop"]["stop_confirmed"])
        self.assertEqual(self.worker.pulse_count, 1)

    def test_partial_start_wrong_direction_latches_motion(self):
        cancelled = [False]
        self.worker._cancel_requested = lambda: cancelled[0]

        def move_left_backwards_then_cancel():
            self.owner.positions["drive_b"] -= 7
            cancelled[0] = True

        self.owner.after_first_start = move_left_backwards_then_cancel
        result = self.worker._pulse("ADVANCE")

        self.assertEqual(
            result["outcome"]["reason"],
            "encoder_direction_mismatch",
        )
        self.assertTrue(self.worker.motion_fault_latched)
        self.assertEqual(
            result["outcome"]["slices"][0]["motors"][0][
                "position_delta"
            ],
            -7,
        )

    def test_cancellation_between_turn_slices_starts_no_later_slice(self):
        cancelled = [False]
        self.worker._cancel_requested = lambda: cancelled[0]
        original_finish = self.owner.finish_active

        def cancel_after_first_verified_finish(verify_motion):
            result = original_finish(verify_motion)
            if verify_motion:
                cancelled[0] = True
            return result

        self.owner.finish_active = cancel_after_first_verified_finish
        result = self.worker._pulse("TURN_LEFT_90")

        self.assertEqual(result["outcome"]["status"], "interrupted")
        self.assertEqual(
            result["outcome"]["reason"],
            "cancel_requested",
        )
        self.assertEqual(
            result["outcome"]["completed_slice_count"],
            1,
        )
        self.assertEqual(self.worker.pulse_count, 1)
        self.assertEqual(self.worker.pulse_duration_ms, 800)
        self.assertTrue(result["stop"]["stop_confirmed"])
        self.assertTrue(self.worker.shutdown_requested)
        self.assertEqual(self.owner.state, "")

    def test_clean_encoder_undertravel_is_recoverable(self):
        self.owner.blocked_roles.add("drive_c")
        failed = self.worker._pulse("ADVANCE")
        self.assertEqual(
            failed["outcome"]["status"],
            "verification_failed",
        )
        self.assertEqual(
            failed["outcome"]["reason"],
            "encoder_recovery_exhausted",
        )
        self.assertEqual(
            [
                segment["kind"]
                for segment in failed["outcome"]["slices"][0]["segments"]
            ],
            ["paired", "right_catch_up", "right_catch_up"],
        )
        self.assertFalse(self.worker.motion_fault_latched)

        self.owner.blocked_roles.clear()
        recovered = self.worker._pulse("ADVANCE")
        self.assertEqual(recovered["outcome"]["status"], "completed")

    def test_transient_motor_fault_with_verified_pair_does_not_latch(self):
        for token in ("stalled", "overloaded"):
            with self.subTest(token=token):
                worker, owner = self.build_worker()
                owner.transient_fault_roles.add("drive_c")
                owner.transient_fault_token = token
                owner.transient_fault_after_leader_degrees = 160

                result = worker._pulse("ADVANCE")

                self.assertEqual(result["outcome"]["status"], "completed")
                receipt = result["outcome"]["slices"][0]
                self.assertEqual(
                    [segment["kind"] for segment in receipt["segments"]],
                    ["paired"],
                )
                self.assertEqual(
                    receipt["segments"][0]["reason"],
                    "motor_fault_encoder_verified",
                )
                self.assertTrue(receipt["encoder_verification"]["passed"])
                self.assertFalse(worker.motion_fault_latched)

    def test_transient_one_wheel_fault_gets_bounded_catch_up(self):
        self.owner.blocked_roles.add("drive_c")
        self.owner.transient_fault_roles.add("drive_c")
        self.owner.transient_fault_after_leader_degrees = 160
        original_finish = self.owner.finish_active
        finish_calls = [0]

        def release_after_faulted_primary(verify_motion):
            result = original_finish(verify_motion)
            finish_calls[0] += 1
            if finish_calls[0] == 1:
                self.owner.blocked_roles.clear()
                self.owner.transient_fault_roles.clear()
            return result

        self.owner.finish_active = release_after_faulted_primary

        result = self.worker._pulse("ADVANCE")

        self.assertEqual(result["outcome"]["status"], "completed")
        receipt = result["outcome"]["slices"][0]
        self.assertEqual(
            [segment["kind"] for segment in receipt["segments"]],
            ["paired", "right_catch_up"],
        )
        self.assertEqual(
            receipt["segments"][0]["reason"],
            "transient_motor_fault_undertravel",
        )
        self.assertEqual(
            [motor["position_delta"] for motor in receipt["motors"]],
            [160, 160],
        )
        self.assertTrue(receipt["encoder_verification"]["passed"])
        self.assertFalse(self.worker.motion_fault_latched)
        self.assertEqual(self.worker.pulse_count, 2)
        self.assertEqual(self.worker.pulse_duration_ms, 450)

    def test_failed_catch_up_after_transient_motor_fault_latches(self):
        self.owner.blocked_roles.add("drive_c")
        self.owner.transient_fault_roles.add("drive_c")
        self.owner.transient_fault_after_leader_degrees = 160

        failed = self.worker._pulse("ADVANCE")

        self.assertEqual(
            failed["outcome"]["status"],
            "verification_failed",
        )
        self.assertEqual(
            failed["outcome"]["reason"],
            "encoder_recovery_exhausted",
        )
        self.assertEqual(
            [
                segment["kind"]
                for segment in failed["outcome"]["slices"][0]["segments"]
            ],
            ["paired", "right_catch_up", "right_catch_up"],
        )
        self.assertTrue(self.worker.motion_fault_latched)
        with self.assertRaises(WorkerError) as raised:
            self.worker._pulse("ADVANCE")
        self.assertEqual(raised.exception.code, "motion_fault_latched")

    def test_transient_pair_fault_without_a_leader_latches_immediately(self):
        self.owner.blocked_roles.update(("drive_b", "drive_c"))
        self.owner.transient_fault_roles.add("drive_c")
        self.owner.transient_fault_after_leader_degrees = 0

        failed = self.worker._pulse("REVERSE")

        self.assertEqual(failed["outcome"]["status"], "interrupted")
        self.assertEqual(
            failed["outcome"]["reason"],
            "encoder_verification_failed",
        )
        self.assertEqual(
            [
                segment["kind"]
                for segment in failed["outcome"]["slices"][0]["segments"]
            ],
            ["paired"],
        )
        self.assertTrue(self.worker.motion_fault_latched)

    def test_undertravel_gate_uses_physical_motor_direction(self):
        self.worker.robot.config["drive_geometry"][
            "forward_speed_sign"
        ]["drive_c"] = -1
        runtime = self.worker._encoder_recovery_runtime()
        finish = {
            "stop": {
                "stop_confirmed": True,
                "errors": [],
                "fault_tokens": [],
            },
            "motors": [
                {
                    "side": "left",
                    "role": "drive_b",
                    "position_before": 0,
                    "position_after": 100,
                    "position_delta": 100,
                    "state": "",
                },
                {
                    "side": "right",
                    "role": "drive_c",
                    "position_before": 0,
                    "position_after": -100,
                    "position_delta": -100,
                    "state": "",
                },
            ],
        }

        self.assertTrue(
            runtime.undertravel_is_recoverable(
                ACTION_SPECS["ADVANCE"],
                finish,
            )
        )
        finish["motors"][1]["position_after"] = 100
        finish["motors"][1]["position_delta"] = 100
        self.assertFalse(
            runtime.undertravel_is_recoverable(
                ACTION_SPECS["ADVANCE"],
                finish,
            )
        )

    def test_lagging_wheel_is_caught_up_inside_the_same_semantic_slice(self):
        self.owner.blocked_roles.add("drive_c")
        original_finish = self.owner.finish_active
        finish_calls = [0]

        def release_after_primary(verify_motion):
            result = original_finish(verify_motion)
            finish_calls[0] += 1
            if finish_calls[0] == 1:
                self.owner.blocked_roles.clear()
            return result

        self.owner.finish_active = release_after_primary

        result = self.worker._pulse("ADVANCE")

        self.assertEqual(result["outcome"]["status"], "completed")
        receipt = result["outcome"]["slices"][0]
        self.assertEqual(receipt["reason"], "encoder_recovered")
        self.assertEqual(
            [segment["kind"] for segment in receipt["segments"]],
            ["paired", "right_catch_up"],
        )
        self.assertEqual(
            [motor["position_delta"] for motor in receipt["motors"]],
            [201, 202],
        )
        self.assertTrue(receipt["encoder_verification"]["passed"])
        self.assertFalse(self.worker.motion_fault_latched)
        self.assertEqual(self.worker.pulse_count, 2)
        self.assertEqual(self.worker.pulse_duration_ms, 502)

    def test_failed_final_recovery_segment_can_complete_cumulative_motion(
        self,
    ):
        self.owner.advance = lambda _seconds: None
        finish_active = self.owner.finish_active

        def finish_with_edge_positions(verify_motion):
            if self.owner.finish_count == 0:
                self.owner.positions.update(
                    {"drive_b": 160, "drive_c": 148}
                )
            elif self.owner.finish_count == 1:
                self.owner.positions.update(
                    {"drive_b": 162, "drive_c": 150}
                )
            return finish_active(verify_motion)

        self.owner.finish_active = finish_with_edge_positions

        result = self.worker._pulse("ADVANCE")

        receipt = result["outcome"]["slices"][0]
        self.assertEqual(receipt["status"], "completed")
        self.assertTrue(receipt["encoder_verification"]["passed"])
        self.assertEqual(
            [motor["position_delta"] for motor in receipt["motors"]],
            [162, 150],
        )
        self.assertEqual(
            [segment["kind"] for segment in receipt["segments"]],
            ["paired", "paired_retry"],
        )
        self.assertEqual(
            [segment["status"] for segment in receipt["segments"]],
            ["completed", "verification_failed"],
        )
        self.assertFalse(
            receipt["segments"][-1]["encoder_verification"]["passed"]
        )

    def test_partial_single_wheel_recovery_preserves_encoder_motion(self):
        self.owner.blocked_roles.add("drive_c")

        def move_right_then_cancel():
            self.owner.positions["drive_c"] += 11
            raise WorkerError("cancel_requested", "cancel recovery")

        self.owner.after_single_start = move_right_then_cancel
        result = self.worker._pulse("ADVANCE")

        receipt = result["outcome"]["slices"][0]
        self.assertEqual(receipt["status"], "interrupted")
        self.assertEqual(
            [segment["kind"] for segment in receipt["segments"]],
            ["paired", "partial_start"],
        )
        self.assertEqual(
            receipt["segments"][1]["commanded_sides"], ["right"]
        )
        self.assertEqual(
            [motor["position_delta"] for motor in receipt["motors"]],
            [201, 11],
        )
        self.assertTrue(receipt["stop"]["stop_confirmed"])

    def test_partial_paired_recovery_preserves_encoder_motion(self):
        self.owner.blocked_roles.update(("drive_b", "drive_c"))

        def fail_only_after_primary():
            if self.owner.finish_count:
                self.owner.positions["drive_b"] += 13
                raise WorkerError("touch_pressed", "stop retry")

        self.owner.after_first_start = fail_only_after_primary
        result = self.worker._pulse("ADVANCE")

        receipt = result["outcome"]["slices"][0]
        self.assertEqual(receipt["status"], "interrupted")
        self.assertEqual(
            [segment["kind"] for segment in receipt["segments"]],
            ["paired", "partial_start"],
        )
        self.assertEqual(
            receipt["segments"][1]["commanded_sides"], ["left"]
        )
        self.assertEqual(
            [motor["position_delta"] for motor in receipt["motors"]],
            [13, 0],
        )

    def test_wrong_direction_encoder_failure_latches_motion(self):
        self.owner.wrong_direction_roles.add("drive_c")
        failed = self.worker._pulse("TURN_LEFT_90")
        self.assertEqual(
            failed["outcome"]["status"],
            "verification_failed",
        )
        self.assertTrue(self.worker.motion_fault_latched)
        with self.assertRaises(WorkerError) as raised:
            self.worker._pulse("TURN_RIGHT_90")
        self.assertEqual(
            raised.exception.code,
            "motion_fault_latched",
        )

    def test_budget_exhaustion_is_fatal_before_motor_start(self):
        self.worker.pulse_count = MAX_PULSES
        with self.assertRaises(WorkerError) as raised:
            self.worker._pulse("ADVANCE")
        self.assertEqual(
            raised.exception.code,
            "pulse_budget_exhausted",
        )
        self.assertTrue(raised.exception.fatal)
        self.assertEqual(self.owner.finish_count, 0)

    def test_describe_declares_host_policy_and_lifetime_lock(self):
        description = self.worker.describe()
        self.assertEqual(description["worker_id"], WORKER_ID)
        self.assertEqual(description["policy_owner"], "host")
        self.assertTrue(
            description["safety"]["lifetime_motor_lock"]
        )
        self.assertFalse(
            description["safety"]["worker_selects_actions"]
        )
        self.assertEqual(
            description["drive_geometry"]["left_motor_role"],
            "drive_b",
        )
        self.assertEqual(
            description["drive_geometry"]["right_motor_role"],
            "drive_c",
        )
        self.assertTrue(
            description["safety"][
                "process_signals_interrupt_active_pulses"
            ]
        )
        self.assertTrue(
            description["safety"][
                "channel_close_interrupts_active_pulses"
            ]
        )
        self.assertTrue(
            description["safety"][
                "infrared_does_not_block_turns"
            ]
        )
        self.assertEqual(
            description["scan_turn"],
            scan_turn_profile(),
        )
        self.assertEqual(
            description["scan_sample"],
            scan_sample_profile(),
        )

    def test_shutdown_uses_cached_observation_after_read_failure(self):
        self.worker._observe()
        self.owner.fail_reads = True
        shutdown = self.worker._shutdown()
        self.assertTrue(
            shutdown["outcome"]["motor_owner_closed"]
        )
        self.assertEqual(self.owner.close_count, 1)
        self.assertTrue(self.worker.closed)

    def test_close_is_idempotent_after_verified_owner_close(self):
        first = self.worker.close()
        second = self.worker.close()
        self.assertTrue(first["stop_confirmed"])
        self.assertTrue(second["already_closed"])
        self.assertEqual(self.owner.close_count, 1)


if __name__ == "__main__":
    unittest.main()
