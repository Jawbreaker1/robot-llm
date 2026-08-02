import copy
import io
import json
import threading
import unittest

from robot_agent.active_ir_scan_contract import (
    ActiveIrScanCalibration,
    ActiveIrScanContractError,
    ModelScanChoice,
    build_scan_request,
    validate_scan_result,
    worst_case_scan_budget,
)
from robot_agent.ev3_active_ir_scan_rig import (
    EV3ActiveIrScanRig,
    EV3ActiveIrScanWorkerError,
    build_ev3_active_ir_scan_executor,
)
from robot_agent.ev3_navigation_transport import (
    EV3NavigationOperationError,
    EV3NavigationRemoteError,
    EV3NavigationSSHTransport,
    EV3NavigationTransportError,
)
from robot_agent.physical_navigation_contract import (
    EXPECTED_WORKER_SAFETY,
    SCAN_TURN_ALLOWED_DELTAS_MDEG,
    expected_scan_sample_profile,
    expected_scan_turn_profile,
    expected_scan_turn_spec,
)
from robot_agent.physical_odometry import PhysicalPose


def stop_proof():
    return {
        "stop_attempts": [],
        "stop_confirmed": True,
        "states": {},
        "positions": {},
        "fault_tokens": {},
        "errors": [],
    }


class FakeClock:
    def __init__(self):
        self.now = 1_000

    def __call__(self):
        return self.now

    def advance(self, milliseconds):
        self.now += milliseconds


class FakeScanTransport:
    def __init__(
        self,
        clock,
        *,
        encoder_scale=1.0,
        touch_after_turn=False,
        fault_after_turn=False,
        late_by_ms=0,
        use_profile_timing=False,
    ):
        self.clock = clock
        self.encoder_scale = encoder_scale
        self.touch_after_turn = touch_after_turn
        self.fault_after_turn = fault_after_turn
        self.late_by_ms = late_by_ms
        self.use_profile_timing = use_profile_timing
        self.last_state_version = 1
        self.worker_description = {
            "scan_turn": expected_scan_turn_profile(),
            "scan_sample": expected_scan_sample_profile(),
            "drive_geometry": {
                "left_motor_role": "drive_b",
                "right_motor_role": "drive_c",
                "forward_speed_sign": {
                    "drive_b": 1,
                    "drive_c": 1,
                },
            },
        }
        self.aborted = False
        self.calls = []
        self.relative_heading_mdeg = 0
        self.left_position = 0
        self.right_position = 0
        self.last_outcome = {
            "kind": "startup",
            "status": "ready",
        }
        self.turn_count = 0

    def abort(self):
        self.aborted = True

    def _observation(self, version):
        blocked = abs(self.relative_heading_mdeg) <= 20_000
        touch = self.touch_after_turn and self.turn_count > 0
        fault = self.fault_after_turn and self.turn_count > 0
        return {
            "state_version": version,
            "observed_monotonic_ms": version * 10,
            "touch": {
                "value0": 1 if touch else 0,
                "pressed": touch,
            },
            "infrared": {
                "raw": 20 if blocked else 60,
                "filtered": 20 if blocked else 60,
                "blocked": blocked,
                "reason": (
                    "blocked_hysteresis_hold"
                    if blocked
                    else "clear_hysteresis_hold"
                ),
                "sample_count": 5,
            },
            "motors": [
                {
                    "role": "drive_b",
                    "position": self.left_position,
                    "state": "",
                },
                {
                    "role": "drive_c",
                    "position": self.right_position,
                    "state": "",
                },
            ],
            "last_outcome": copy.deepcopy(self.last_outcome),
            "budgets": {
                "pulse_count": 0,
                "pulse_count_remaining": 40,
                "pulse_duration_ms": 0,
                "pulse_duration_ms_remaining": 32_000,
                "process_ms_remaining": 40_000,
                "motion_fault_latched": fault,
            },
        }

    @staticmethod
    def _distributed_encoder_deltas(total, count):
        base = total // count
        values = [base] * count
        values[-1] += total - sum(values)
        return values

    def _scan_turn(self, relative_delta_mdeg):
        spec = expected_scan_turn_spec(relative_delta_mdeg)
        target = int(
            round(
                spec["target_mean_abs_encoder_degrees"]
                * self.encoder_scale
            )
        )
        direction = 1 if relative_delta_mdeg > 0 else -1
        deltas = self._distributed_encoder_deltas(
            target,
            spec["slice_count"],
        )
        slices = []
        for ordinal, (duration_ms, magnitude) in enumerate(
            zip(spec["slice_durations_ms"], deltas),
            1,
        ):
            left_before = self.left_position
            right_before = self.right_position
            left_delta = -direction * magnitude
            right_delta = direction * magnitude
            self.left_position += left_delta
            self.right_position += right_delta
            slices.append(
                {
                    "slice_index": ordinal,
                    "slice_count": spec["slice_count"],
                    "duration_ms": duration_ms,
                    "status": "completed",
                    "reason": "duration_elapsed",
                    "started_monotonic_ms": self.clock(),
                    "completed_monotonic_ms": self.clock() + 1,
                    "motors": [
                        {
                            "side": "left",
                            "role": "drive_b",
                            "position_before": left_before,
                            "position_after": self.left_position,
                            "position_delta": left_delta,
                            "state": "",
                        },
                        {
                            "side": "right",
                            "role": "drive_c",
                            "position_before": right_before,
                            "position_after": self.right_position,
                            "position_delta": right_delta,
                            "state": "",
                        },
                    ],
                    "encoder_verification": {
                        "passed": True,
                        "error": None,
                        "checks": [{"passed": True}],
                    },
                    "stop": stop_proof(),
                }
            )
        actual_delta = direction * int(
            round(target * 90_000 / 682.0)
        )
        self.relative_heading_mdeg += actual_delta
        self.turn_count += 1
        self.last_outcome = {
            "kind": "scan_turn",
            "requested_relative_delta_mdeg": relative_delta_mdeg,
            "status": "completed",
            "reason": "scan_turn_completed",
            "profile_id": spec["profile_id"],
            "calibration": spec["calibration"],
            "started_monotonic_ms": self.clock(),
            "completed_monotonic_ms": self.clock() + 1,
            "stop_confirmed": True,
            "requested_slice_count": spec["slice_count"],
            "completed_slice_count": spec["slice_count"],
            "slices": slices,
            "encoder_verification": {
                "passed": True,
                "verified_slice_count": spec["slice_count"],
                "requested_slice_count": spec["slice_count"],
                "left_delta_degrees": -direction * target,
                "right_delta_degrees": direction * target,
                "mean_abs_encoder_degrees": target,
                "target_mean_abs_encoder_degrees": spec[
                    "target_mean_abs_encoder_degrees"
                ],
                "max_side_divergence_degrees": 80,
            },
        }
        self.clock.advance(
            (
                spec["total_duration_ms"]
                if self.use_profile_timing
                else 10
            )
            + self.late_by_ms
        )
        self.last_state_version += 1
        observation = self._observation(self.last_state_version)
        return {
            "state_version": self.last_state_version,
            "result": {
                "relative_delta_mdeg": relative_delta_mdeg,
                "outcome": copy.deepcopy(self.last_outcome),
                "observation": observation,
                "stop": stop_proof(),
            },
        }

    def request(
        self,
        operation,
        arguments,
        timeout_seconds,
        cancel_requested=None,
    ):
        if self.aborted:
            raise EV3NavigationTransportError("transport aborted")
        self.calls.append((operation, copy.deepcopy(arguments)))
        if cancel_requested is not None and cancel_requested():
            self.abort()
            raise EV3NavigationTransportError("cancelled")
        if operation == "scan_turn":
            return self._scan_turn(arguments["relative_delta_mdeg"])
        if operation == "scan_sample":
            profile = expected_scan_sample_profile()
            started = self.clock()
            raw = 20 if abs(self.relative_heading_mdeg) <= 20_000 else 60
            raw_samples = [raw] * profile["sample_count"]
            self.clock.advance(
                profile["settled_duration_ms"] + self.late_by_ms
            )
            self.last_state_version += 1
            return {
                "state_version": self.last_state_version,
                "result": {
                    "sample_count": profile["sample_count"],
                    "raw_samples": raw_samples,
                    "started_monotonic_ms": started,
                    "completed_monotonic_ms": self.clock(),
                    "observation": self._observation(
                        self.last_state_version
                    ),
                    "stop": stop_proof(),
                },
            }
        self.clock.advance(10 + self.late_by_ms)
        self.last_state_version += 1
        if operation == "observe":
            return {
                "state_version": self.last_state_version,
                "result": {
                    "observation": self._observation(
                        self.last_state_version
                    )
                },
            }
        if operation == "stop":
            self.last_outcome = {
                "kind": "stop",
                "status": "completed",
                "completed_monotonic_ms": self.clock(),
                "stop_confirmed": True,
            }
            return {
                "state_version": self.last_state_version,
                "result": {
                    "outcome": copy.deepcopy(self.last_outcome),
                    "observation": self._observation(
                        self.last_state_version
                    ),
                    "stop": stop_proof(),
                },
            }
        raise AssertionError("unexpected operation {}".format(operation))


class SequencedEncoderScaleScanTransport(FakeScanTransport):
    """Apply deterministic encoder scales to successive scan turns."""

    def __init__(self, clock, encoder_scales):
        super().__init__(clock)
        self.encoder_scales = tuple(encoder_scales)

    def _scan_turn(self, relative_delta_mdeg):
        if self.turn_count < len(self.encoder_scales):
            self.encoder_scale = self.encoder_scales[self.turn_count]
        return super()._scan_turn(relative_delta_mdeg)


class StrictValidatingScanTransport(FakeScanTransport):
    """Run fake scan receipts through the real host wire validator."""

    def __init__(self, clock):
        super().__init__(clock)
        self.validator = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )
        self.sample_count = 0

    def request(
        self,
        operation,
        arguments,
        timeout_seconds,
        cancel_requested=None,
    ):
        response = super().request(
            operation,
            arguments,
            timeout_seconds,
            cancel_requested=cancel_requested,
        )
        if operation == "scan_sample":
            self.sample_count += 1
            if self.sample_count == 1:
                result = response["result"]
                result["raw_samples"] = [33, 33, 34, 34, 33]
                result["observation"]["infrared"].update(
                    raw=33,
                    filtered=34,
                    blocked=True,
                    reason="blocked_hysteresis_hold",
                )
        self.validator._validate_success_result(
            operation,
            arguments,
            response,
        )
        return response


class BlockingScanTransport(FakeScanTransport):
    def __init__(self, clock, block_operation):
        super().__init__(clock)
        self.block_operation = block_operation
        self.entered = threading.Event()

    def request(
        self,
        operation,
        arguments,
        timeout_seconds,
        cancel_requested=None,
    ):
        if self.aborted:
            raise EV3NavigationTransportError("transport aborted")
        if operation != self.block_operation:
            return super().request(
                operation,
                arguments,
                timeout_seconds,
                cancel_requested=cancel_requested,
            )
        self.calls.append((operation, copy.deepcopy(arguments)))
        if not callable(cancel_requested):
            raise AssertionError("scan request lacked cancellation callback")
        self.entered.set()
        while not cancel_requested():
            threading.Event().wait(0.002)
        self.abort()
        raise EV3NavigationTransportError("cancelled")


class LongScanTurnTransport(FakeScanTransport):
    """Reproduce one valid worker response that needs more than 8 seconds."""

    def __init__(self, clock):
        super().__init__(clock, use_profile_timing=True)
        self.long_turn_timeouts = []
        self.long_turn_completed = False

    def request(
        self,
        operation,
        arguments,
        timeout_seconds,
        cancel_requested=None,
    ):
        is_long_turn = (
            operation == "scan_turn"
            and abs(arguments["relative_delta_mdeg"]) == 90_000
            and not self.long_turn_completed
        )
        if not is_long_turn:
            return super().request(
                operation,
                arguments,
                timeout_seconds,
                cancel_requested=cancel_requested,
            )
        self.long_turn_timeouts.append(timeout_seconds)
        if timeout_seconds <= 8.0:
            self.calls.append((operation, copy.deepcopy(arguments)))
            self.abort()
            raise EV3NavigationTransportError(
                "worker response timed out"
            )
        response = super().request(
            operation,
            arguments,
            timeout_seconds,
            cancel_requested=cancel_requested,
        )
        self.clock.advance(9_000)
        self.long_turn_completed = True
        return response


class RemoteSampleErrorTransport(FakeScanTransport):
    def __init__(self, clock, *, code="sensor_read_failed"):
        super().__init__(clock)
        self.code = code
        self.sample_count = 0

    def request(
        self,
        operation,
        arguments,
        timeout_seconds,
        cancel_requested=None,
    ):
        if operation == "scan_sample":
            self.sample_count += 1
            if self.sample_count == 2:
                self.calls.append((operation, copy.deepcopy(arguments)))
                self.last_state_version += 1
                observation = self._observation(
                    self.last_state_version
                )
                raise EV3NavigationRemoteError(
                    self.code,
                    "simulated worker sample error",
                    False,
                    observation=observation,
                    stop=stop_proof(),
                )
        return super().request(
            operation,
            arguments,
            timeout_seconds,
            cancel_requested=cancel_requested,
        )


def scan_request(
    clock,
    *,
    deadline_ms=30_000,
    heading_mdeg=10_000,
    estimated_turn_ms_per_degree=2,
    alignment_tolerance_mdeg=2_500,
):
    return build_scan_request(
        choice=ModelScanChoice("target-a"),
        frame_id="frame-a",
        map_generation_id="generation-a",
        map_version=3,
        start_pose=PhysicalPose(heading_mdeg=heading_mdeg),
        start_state_version=1,
        created_at_ms=clock(),
        deadline_ms=deadline_ms,
        calibration=ActiveIrScanCalibration(
            estimated_turn_ms_per_degree=estimated_turn_ms_per_degree,
            alignment_tolerance_mdeg=alignment_tolerance_mdeg,
        ),
    )


class EV3ActiveIrScanRigTests(unittest.TestCase):
    def test_transport_types_verified_stopped_scan_turn_denial(self):
        clock = FakeClock()
        worker = FakeScanTransport(clock)
        response = worker._scan_turn(30_000)
        outcome = response["result"]["outcome"]
        outcome.update(
            {
                "status": "denied",
                "reason": "policy_denied",
                "started_monotonic_ms": None,
                "completed_slice_count": 0,
                "slices": [],
                "encoder_verification": {
                    "passed": False,
                    "verified_slice_count": 0,
                    "requested_slice_count": outcome[
                        "requested_slice_count"
                    ],
                    "left_delta_degrees": None,
                    "right_delta_degrees": None,
                    "mean_abs_encoder_degrees": None,
                    "target_mean_abs_encoder_degrees": outcome[
                        "encoder_verification"
                    ]["target_mean_abs_encoder_degrees"],
                    "max_side_divergence_degrees": outcome[
                        "encoder_verification"
                    ]["max_side_divergence_degrees"],
                },
            }
        )
        response["result"]["observation"]["last_outcome"] = copy.deepcopy(
            outcome
        )
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )

        with self.assertRaises(EV3NavigationOperationError) as caught:
            transport._validate_success_result(
                "scan_turn",
                {"relative_delta_mdeg": 30_000},
                response,
            )

        self.assertEqual(caught.exception.code, "policy_denied")
        self.assertEqual(caught.exception.observation["state_version"], 2)
        self.assertTrue(caught.exception.stop["stop_confirmed"])
        self.assertIs(caught.exception.result, response["result"])

    def test_transport_preserves_interrupted_turn_encoder_and_stop_proof(self):
        clock = FakeClock()
        worker = FakeScanTransport(clock)
        response = worker._scan_turn(-30_000)
        outcome = response["result"]["outcome"]
        terminal = outcome["slices"][0]
        terminal["status"] = "interrupted"
        terminal["reason"] = "motor_fault"
        totals = {
            motor["side"]: motor["position_delta"]
            for motor in terminal["motors"]
        }
        outcome.update(
            {
                "status": "interrupted",
                "reason": "motor_fault",
                "completed_monotonic_ms": terminal[
                    "completed_monotonic_ms"
                ],
                "completed_slice_count": 0,
                "slices": [terminal],
            }
        )
        outcome["encoder_verification"].update(
            {
                "passed": False,
                "verified_slice_count": 1,
                "left_delta_degrees": totals["left"],
                "right_delta_degrees": totals["right"],
                "mean_abs_encoder_degrees": int(
                    round(
                        (
                            abs(totals["left"])
                            + abs(totals["right"])
                        )
                        / 2.0
                    )
                ),
            }
        )
        response["result"]["observation"]["last_outcome"] = copy.deepcopy(
            outcome
        )
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )

        with self.assertRaises(EV3NavigationOperationError) as caught:
            transport._validate_success_result(
                "scan_turn",
                {"relative_delta_mdeg": -30_000},
                response,
            )

        self.assertEqual(caught.exception.code, "motor_fault")
        self.assertEqual(
            caught.exception.result["outcome"]["encoder_verification"][
                "mean_abs_encoder_degrees"
            ],
            outcome["encoder_verification"]["mean_abs_encoder_degrees"],
        )
        self.assertTrue(caught.exception.stop["stop_confirmed"])

    def test_transport_preserves_stopped_scan_slice_undertravel(self):
        clock = FakeClock()
        worker = FakeScanTransport(clock)
        response = worker._scan_turn(-30_000)
        outcome = response["result"]["outcome"]
        terminal = outcome["slices"][0]
        terminal["status"] = "verification_failed"
        terminal["reason"] = "encoder_verification_failed"
        terminal["encoder_verification"] = {
            "passed": False,
            "error": "right motor did not make minimum progress",
            "checks": [
                {"passed": True},
                {"passed": False},
            ],
        }
        totals = {
            motor["side"]: motor["position_delta"]
            for motor in terminal["motors"]
        }
        outcome.update(
            {
                "status": "verification_failed",
                "reason": "encoder_verification_failed",
                "started_monotonic_ms": terminal["started_monotonic_ms"],
                "completed_monotonic_ms": terminal[
                    "completed_monotonic_ms"
                ],
                "completed_slice_count": 0,
                "slices": [terminal],
            }
        )
        outcome["encoder_verification"].update(
            {
                "passed": False,
                "verified_slice_count": 0,
                "left_delta_degrees": totals["left"],
                "right_delta_degrees": totals["right"],
                "mean_abs_encoder_degrees": int(
                    round(
                        (
                            abs(totals["left"])
                            + abs(totals["right"])
                        )
                        / 2.0
                    )
                ),
            }
        )
        response["result"]["observation"]["last_outcome"] = copy.deepcopy(
            outcome
        )
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )

        with self.assertRaises(EV3NavigationOperationError) as caught:
            transport._validate_success_result(
                "scan_turn",
                {"relative_delta_mdeg": -30_000},
                response,
            )

        self.assertEqual(
            caught.exception.code,
            "encoder_verification_failed",
        )
        self.assertTrue(caught.exception.stop["stop_confirmed"])
        self.assertFalse(caught.exception.fatal)

    def test_remote_worker_stop_and_code_survive_scan_adapter(self):
        clock = FakeClock()
        transport = RemoteSampleErrorTransport(clock)
        request = scan_request(clock)

        result = build_ev3_active_ir_scan_executor(
            transport,
            clock_ms=clock,
        ).execute(request)

        self.assertEqual(result.status, "CANCELLED")
        self.assertEqual(
            result.reason,
            "scan_worker_error:sensor_read_failed",
        )
        self.assertTrue(result.stop_confirmed)
        self.assertTrue(result.restored_start_heading)
        self.assertFalse(transport.aborted)

    def test_remote_turn_without_delta_keeps_heading_conservative(self):
        clock = FakeClock()
        transport = FakeScanTransport(clock)
        rig = EV3ActiveIrScanRig(
            transport=transport,
            clock_ms=clock,
        )
        request = scan_request(clock)
        rig.begin_scan(request)

        def rejected_request(*_args, **_kwargs):
            raise EV3NavigationRemoteError(
                "worker_failure",
                "simulated turn error",
                False,
                observation=transport._observation(2),
                stop=stop_proof(),
            )

        transport.request = rejected_request
        with self.assertRaises(EV3ActiveIrScanWorkerError) as caught:
            rig.turn_relative_mdeg(
                -30_000,
                request.calibration,
                request.deadline_ms,
            )

        self.assertTrue(caught.exception.stop_confirmed)
        self.assertIsNone(
            caught.exception.verified_actual_delta_mdeg
        )
        self.assertTrue(caught.exception.restoration_prohibited)
        self.assertTrue(rig.stop()["stop_confirmed"])

    def test_transport_accepts_encoder_verified_transient_scan_slice(self):
        clock = FakeClock()
        worker = FakeScanTransport(clock)
        response = worker._scan_turn(30_000)
        receipt = response["result"]["outcome"]["slices"][0]
        receipt["reason"] = "motor_fault_encoder_verified"
        response["result"]["observation"]["last_outcome"]["slices"][
            0
        ]["reason"] = "motor_fault_encoder_verified"
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )

        transport._validate_success_result(
            "scan_turn",
            {"relative_delta_mdeg": 30_000},
            response,
        )

        receipt["encoder_verification"]["passed"] = False
        with self.assertRaises(EV3NavigationTransportError):
            transport._validate_success_result(
                "scan_turn",
                {"relative_delta_mdeg": 30_000},
                response,
            )

    def test_partial_turn_touch_never_claims_heading_restoration(self):
        clock = FakeClock()
        transport = FakeScanTransport(clock, touch_after_turn=True)
        executor = build_ev3_active_ir_scan_executor(
            transport,
            clock_ms=clock,
        )

        result = executor.execute(scan_request(clock))

        self.assertEqual(result.status, "CANCELLED")
        self.assertFalse(result.stop_confirmed)
        self.assertFalse(result.restored_start_heading)
        self.assertNotEqual(transport.relative_heading_mdeg, 0)
        self.assertTrue(transport.aborted)

    def test_execute_scoped_stop_aborts_blocked_sample_or_turn(self):
        for blocked_operation in ("scan_sample", "scan_turn"):
            with self.subTest(operation=blocked_operation):
                clock = FakeClock()
                transport = BlockingScanTransport(
                    clock,
                    blocked_operation,
                )
                executor = build_ev3_active_ir_scan_executor(
                    transport,
                    clock_ms=clock,
                )
                cancelled = threading.Event()
                returned = []

                def run_scan():
                    returned.append(
                        executor.execute(
                            scan_request(clock),
                            cancel_requested=cancelled.is_set,
                        )
                    )

                thread = threading.Thread(target=run_scan, daemon=True)
                thread.start()
                self.assertTrue(transport.entered.wait(1.0))
                cancelled.set()
                thread.join(1.0)

                self.assertFalse(thread.is_alive())
                self.assertEqual(len(returned), 1)
                self.assertEqual(returned[0].status, "CANCELLED")
                self.assertFalse(returned[0].restored_start_heading)
                self.assertTrue(transport.aborted)

    def test_real_motor_profile_fits_derived_scan_deadline(self):
        budget = worst_case_scan_budget()
        self.assertEqual(budget["turn_duration_ms"], 11_945)
        self.assertEqual(budget["turn_slice_count"], 22)
        self.assertEqual(budget["request_round_trip_headroom_ms"], 250)
        self.assertEqual(budget["minimum_deadline_ms"], 19_145)
        clock = FakeClock()
        transport = FakeScanTransport(clock, use_profile_timing=True)
        executor = build_ev3_active_ir_scan_executor(
            transport,
            clock_ms=clock,
        )
        deadline_ms = clock() + budget["minimum_deadline_ms"]

        result = executor.execute(
            scan_request(
                clock,
                deadline_ms=deadline_ms,
                estimated_turn_ms_per_degree=30,
            )
        )

        self.assertTrue(result.bilateral_complete)
        self.assertTrue(result.stop_confirmed)
        self.assertLessEqual(result.completed_at_ms, deadline_ms)

    def test_wifi_ev3_budget_outlives_observed_twenty_second_scan(self):
        budget = worst_case_scan_budget(
            request_round_trip_headroom_ms=750,
        )

        self.assertEqual(budget["worker_request_count"], 20)
        self.assertEqual(budget["request_round_trip_headroom_ms"], 750)
        self.assertEqual(budget["minimum_deadline_ms"], 29_145)
        self.assertGreater(budget["minimum_deadline_ms"], 20_071)

    def test_live_wifi_latency_needs_ev3_profile_scan_deadline(self):
        def execute_with_deadline(window_ms):
            clock = FakeClock()
            transport = FakeScanTransport(
                clock,
                use_profile_timing=True,
                # Keep the 20-second regression meaningful after shortening
                # the fine-ray route while retaining a bounded 30-second run.
                late_by_ms=750,
            )
            request = scan_request(
                clock,
                deadline_ms=clock() + window_ms,
                estimated_turn_ms_per_degree=30,
            )
            result = build_ev3_active_ir_scan_executor(
                transport,
                clock_ms=clock,
            ).execute(request)
            return request, result

        short_request, short_result = execute_with_deadline(20_000)
        with self.assertRaises(ActiveIrScanContractError) as caught:
            validate_scan_result(
                short_result,
                short_request,
                current_frame_id=short_request.frame_id,
                current_map_generation_id=(
                    short_request.map_generation_id
                ),
                current_map_version=short_request.based_on_map_version,
            )
        self.assertEqual(
            caught.exception.code,
            "scan_deadline_or_chronology",
        )
        self.assertEqual(short_result.reason, "scan_deadline_exceeded")
        self.assertGreater(
            short_result.completed_at_ms,
            short_request.deadline_ms,
        )

        long_request, long_result = execute_with_deadline(30_000)
        checked = validate_scan_result(
            long_result,
            long_request,
            current_frame_id=long_request.frame_id,
            current_map_generation_id=long_request.map_generation_id,
            current_map_version=long_request.based_on_map_version,
        )
        self.assertIs(checked, long_result)
        self.assertTrue(long_result.bilateral_complete)
        self.assertLessEqual(
            long_result.completed_at_ms,
            long_request.deadline_ms,
        )

    def test_soft_scan_timeout_restores_inside_hard_cleanup_window(self):
        clock = FakeClock()
        transport = FakeScanTransport(
            clock,
            use_profile_timing=True,
            late_by_ms=750,
        )
        request = scan_request(
            clock,
            deadline_ms=clock() + 30_000,
            estimated_turn_ms_per_degree=30,
        )

        result = build_ev3_active_ir_scan_executor(
            transport,
            clock_ms=clock,
            restoration_headroom_ms=15_000,
        ).execute(request)
        checked = validate_scan_result(
            result,
            request,
            current_frame_id=request.frame_id,
            current_map_generation_id=request.map_generation_id,
            current_map_version=request.based_on_map_version,
        )

        self.assertIs(checked, result)
        self.assertEqual(result.status, "CANCELLED")
        self.assertEqual(result.reason, "scan_deadline_exceeded")
        self.assertTrue(result.stop_confirmed)
        self.assertTrue(result.restored_start_heading)
        self.assertGreater(len(result.rays), 0)
        self.assertLessEqual(result.completed_at_ms, request.deadline_ms)
        self.assertLessEqual(
            abs(transport.relative_heading_mdeg),
            request.calibration.alignment_tolerance_mdeg,
        )
        self.assertFalse(transport.aborted)

    def test_ev3_request_timeout_outlives_long_valid_scan_turn(self):
        def execute(request_timeout_seconds):
            clock = FakeClock()
            transport = LongScanTurnTransport(clock)
            request = scan_request(
                clock,
                deadline_ms=clock() + 50_000,
                estimated_turn_ms_per_degree=30,
            )
            result = build_ev3_active_ir_scan_executor(
                transport,
                clock_ms=clock,
                request_timeout_seconds=request_timeout_seconds,
                restoration_headroom_ms=10_000,
            ).execute(request)
            checked = validate_scan_result(
                result,
                request,
                current_frame_id=request.frame_id,
                current_map_generation_id=request.map_generation_id,
                current_map_version=request.based_on_map_version,
            )
            return transport, checked

        short_transport, short_result = execute(8.0)
        self.assertEqual(short_result.status, "CANCELLED")
        self.assertEqual(short_result.reason, "scan_transport_failed")
        self.assertFalse(short_result.stop_confirmed)
        self.assertFalse(short_result.restored_start_heading)
        self.assertTrue(short_transport.aborted)

        long_transport, long_result = execute(30.0)
        self.assertTrue(long_result.bilateral_complete)
        self.assertTrue(long_result.stop_confirmed)
        self.assertTrue(long_result.restored_start_heading)
        self.assertTrue(long_transport.long_turn_completed)
        self.assertGreater(long_transport.long_turn_timeouts[0], 8.0)
        self.assertFalse(long_transport.aborted)

    def test_executor_uses_fixed_lattice_and_restores_encoder_heading(self):
        clock = FakeClock()
        transport = FakeScanTransport(clock)
        executor = build_ev3_active_ir_scan_executor(
            transport,
            clock_ms=clock,
        )
        request = scan_request(clock)
        result = executor.execute(request)

        self.assertTrue(result.bilateral_complete)
        self.assertTrue(result.restored_start_heading)
        deltas = [
            arguments["relative_delta_mdeg"]
            for operation, arguments in transport.calls
            if operation == "scan_turn"
        ]
        self.assertTrue(deltas)
        self.assertTrue(
            all(
                delta in SCAN_TURN_ALLOWED_DELTAS_MDEG
                for delta in deltas
            )
        )
        self.assertLessEqual(
            abs(transport.relative_heading_mdeg),
            request.calibration.alignment_tolerance_mdeg,
        )
        self.assertTrue(
            all(
                set(arguments) == {"relative_delta_mdeg"}
                for operation, arguments in transport.calls
                if operation == "scan_turn"
            )
        )

    def test_ev3_tolerance_accepts_encoder_derived_bilateral_underturn(self):
        clock = FakeClock()
        transport = FakeScanTransport(clock, encoder_scale=0.95)
        request = scan_request(
            clock,
            alignment_tolerance_mdeg=10_000,
        )

        result = build_ev3_active_ir_scan_executor(
            transport,
            clock_ms=clock,
        ).execute(request)
        checked = validate_scan_result(
            result,
            request,
            current_frame_id=request.frame_id,
            current_map_generation_id=request.map_generation_id,
            current_map_version=request.based_on_map_version,
        )

        self.assertIs(checked, result)
        self.assertTrue(result.bilateral_complete)
        self.assertTrue(result.restored_start_heading)
        requested = {
            ray.requested_relative_bearing_mdeg for ray in result.rays
        }
        self.assertTrue(any(value < 0 for value in requested))
        self.assertTrue(any(value > 0 for value in requested))
        self.assertTrue(
            all(
                abs(
                    ray.actual_relative_bearing_mdeg
                    - ray.requested_relative_bearing_mdeg
                )
                <= request.calibration.alignment_tolerance_mdeg
                for ray in result.rays
            )
        )

    def test_center_ray_jitter_survives_strict_transport_and_full_scan(self):
        clock = FakeClock()
        transport = StrictValidatingScanTransport(clock)
        request = scan_request(clock)

        result = build_ev3_active_ir_scan_executor(
            transport,
            clock_ms=clock,
        ).execute(request)

        self.assertTrue(result.bilateral_complete)
        self.assertTrue(result.stop_confirmed)
        self.assertTrue(result.restored_start_heading)
        self.assertEqual(result.rays[0].requested_relative_bearing_mdeg, 0)
        self.assertEqual(result.rays[0].raw, 33)
        self.assertEqual(result.rays[0].filtered, 34)
        self.assertFalse(transport.aborted)

    def test_required_individual_turns_are_encoder_derived(self):
        for delta in (-60_000, -30_000, -15_000, 15_000, 30_000, 60_000):
            with self.subTest(delta=delta):
                clock = FakeClock()
                transport = FakeScanTransport(clock)
                rig = EV3ActiveIrScanRig(
                    transport=transport,
                    clock_ms=clock,
                )
                request = scan_request(clock)
                rig.begin_scan(request)
                receipt = rig.turn_relative_mdeg(
                    delta,
                    request.calibration,
                    request.deadline_ms,
                )
                self.assertEqual(
                    receipt["requested_delta_mdeg"],
                    delta,
                )
                self.assertLessEqual(
                    abs(receipt["actual_delta_mdeg"] - delta),
                    request.calibration.alignment_tolerance_mdeg,
                )
                self.assertTrue(receipt["stop_confirmed"])

    def test_verified_encoder_undertravel_is_returned_for_pose_validation(self):
        clock = FakeClock()
        transport = FakeScanTransport(clock, encoder_scale=0.5)
        rig = EV3ActiveIrScanRig(
            transport=transport,
            clock_ms=clock,
        )
        request = scan_request(clock)
        rig.begin_scan(request)
        receipt = rig.turn_relative_mdeg(
            30_000,
            request.calibration,
            request.deadline_ms,
        )
        snapshot = rig.read_snapshot(request.deadline_ms)

        self.assertGreater(
            abs(receipt["actual_delta_mdeg"] - 30_000),
            request.calibration.alignment_tolerance_mdeg,
        )
        self.assertEqual(
            snapshot["pose_heading_mdeg"],
            request.start_pose.heading_mdeg
            + receipt["actual_delta_mdeg"],
        )
        self.assertFalse(transport.aborted)

    def test_symmetric_underturn_cancels_bad_ray_but_restores_heading(self):
        clock = FakeClock()
        transport = FakeScanTransport(clock, encoder_scale=0.95)
        request = scan_request(clock)

        result = build_ev3_active_ir_scan_executor(
            transport,
            clock_ms=clock,
        ).execute(request)
        checked = validate_scan_result(
            result,
            request,
            current_frame_id=request.frame_id,
            current_map_generation_id=request.map_generation_id,
            current_map_version=request.based_on_map_version,
        )

        self.assertIs(checked, result)
        self.assertEqual(result.status, "CANCELLED")
        self.assertEqual(result.reason, "scan_snapshot_pose_misaligned")
        self.assertTrue(result.stop_confirmed)
        self.assertTrue(result.restored_start_heading)
        self.assertEqual(
            [ray.requested_relative_bearing_mdeg for ray in result.rays],
            [0, -30_000],
        )
        self.assertEqual(
            [
                arguments["relative_delta_mdeg"]
                for operation, arguments in transport.calls
                if operation == "scan_turn"
            ],
            [-30_000, -30_000, 60_000],
        )
        self.assertLessEqual(
            abs(transport.relative_heading_mdeg),
            request.calibration.alignment_tolerance_mdeg,
        )
        self.assertFalse(transport.aborted)

    def test_asymmetric_underturn_keeps_failed_restoration_unverified(self):
        clock = FakeClock()
        transport = SequencedEncoderScaleScanTransport(
            clock,
            encoder_scales=(0.95, 0.95, 0.85),
        )
        request = scan_request(clock)

        result = build_ev3_active_ir_scan_executor(
            transport,
            clock_ms=clock,
        ).execute(request)
        checked = validate_scan_result(
            result,
            request,
            current_frame_id=request.frame_id,
            current_map_generation_id=request.map_generation_id,
            current_map_version=request.based_on_map_version,
        )

        self.assertIs(checked, result)
        self.assertEqual(result.status, "CANCELLED")
        self.assertEqual(
            result.reason,
            "scan_heading_restoration_unverified",
        )
        self.assertTrue(result.stop_confirmed)
        self.assertFalse(result.restored_start_heading)
        self.assertGreater(
            abs(transport.relative_heading_mdeg),
            request.calibration.alignment_tolerance_mdeg,
        )
        self.assertFalse(transport.aborted)

    def test_touch_motion_fault_late_and_cancel_all_fail_closed(self):
        scenarios = (
            ("touch", {"touch_after_turn": True}),
            ("fault", {"fault_after_turn": True}),
            ("late", {"late_by_ms": 40_000}),
        )
        for name, options in scenarios:
            with self.subTest(name=name):
                clock = FakeClock()
                transport = FakeScanTransport(clock, **options)
                rig = EV3ActiveIrScanRig(
                    transport=transport,
                    clock_ms=clock,
                )
                request = scan_request(clock)
                rig.begin_scan(request)
                with self.assertRaises(ActiveIrScanContractError):
                    rig.turn_relative_mdeg(
                        15_000,
                        request.calibration,
                        request.deadline_ms,
                    )
                self.assertTrue(transport.aborted)

        clock = FakeClock()
        transport = FakeScanTransport(clock)
        rig = EV3ActiveIrScanRig(
            transport=transport,
            clock_ms=clock,
            cancel_requested=lambda: True,
        )
        with self.assertRaises(ActiveIrScanContractError) as caught:
            rig.begin_scan(scan_request(clock))
        self.assertEqual(caught.exception.code, "scan_cancelled")
        self.assertTrue(transport.aborted)

    def test_start_state_mismatch_is_rejected_before_turn(self):
        clock = FakeClock()
        transport = FakeScanTransport(clock)
        transport.last_state_version = 2
        rig = EV3ActiveIrScanRig(
            transport=transport,
            clock_ms=clock,
        )
        with self.assertRaises(ActiveIrScanContractError) as caught:
            rig.begin_scan(scan_request(clock))
        self.assertEqual(
            caught.exception.code,
            "scan_start_state_mismatch",
        )
        self.assertEqual(transport.calls, [])


class FakeStdin(io.BytesIO):
    def __init__(self):
        super().__init__()
        self.flush_count = 0
        self.write_count = 0

    def write(self, value):
        self.write_count += 1
        return super().write(value)

    def flush(self):
        self.flush_count += 1


class FakeProcess:
    def __init__(self):
        self.stdin = FakeStdin()
        self.terminated = False

    def terminate(self):
        self.terminated = True


class EV3NavigationCancellationTests(unittest.TestCase):
    @staticmethod
    def _transport_with_process():
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path="/home/robot/navigation_worker.py",
        )
        process = FakeProcess()
        transport._process = process
        return transport, process

    def _assert_ambiguous_response_aborts_and_prohibits_reuse(
        self,
        queued_response,
        *,
        timeout_seconds=1.0,
    ):
        transport, process = self._transport_with_process()
        if queued_response is not None:
            transport._responses.put(queued_response)
        with self.assertRaises(Exception):
            transport.request("observe", {}, timeout_seconds)
        self.assertTrue(transport.aborted)
        self.assertTrue(process.stdin.closed)
        self.assertTrue(process.terminated)
        self.assertEqual(process.stdin.write_count, 1)
        with self.assertRaises(EV3NavigationTransportError) as caught:
            transport.request("observe", {}, 1.0)
        self.assertIn("cannot be reused", str(caught.exception))
        self.assertEqual(process.stdin.write_count, 1)

    def test_timeout_aborts_channel_and_prohibits_reuse(self):
        self._assert_ambiguous_response_aborts_and_prohibits_reuse(
            None,
            timeout_seconds=0.1,
        )

    def test_invalid_worker_output_aborts_channel_and_prohibits_reuse(self):
        invalid_success = {
            "schema": "ev3-agent-worker-response/v2",
            "controller_id": "ev3-main",
            "request_id": "host-0001",
            "ok": True,
            "state_version": 1,
            "result": {"unexpected": True},
        }
        for queued in (
            ("invalid", None),
            ("line", b"{broken\n"),
            (
                "line",
                json.dumps(invalid_success).encode("utf-8") + b"\n",
            ),
        ):
            with self.subTest(kind=queued[0], raw=queued[1]):
                self._assert_ambiguous_response_aborts_and_prohibits_reuse(
                    queued
                )

    def test_wrong_request_id_aborts_channel_and_prohibits_reuse(self):
        clock = FakeClock()
        worker = FakeScanTransport(clock)
        wrong = {
            "schema": "ev3-agent-worker-response/v2",
            "controller_id": "ev3-main",
            "request_id": "host-wrong",
            "ok": True,
            "state_version": 1,
            "result": {"observation": worker._observation(1)},
        }
        self._assert_ambiguous_response_aborts_and_prohibits_reuse(
            (
                "line",
                json.dumps(wrong).encode("utf-8") + b"\n",
            )
        )

    def test_transport_requires_both_process_interrupt_capabilities(self):
        base_result = {
            "scan_turn": expected_scan_turn_profile(),
            "scan_sample": expected_scan_sample_profile(),
            "operations": [
                "describe",
                "observe",
                "pulse",
                "scan_turn",
                "scan_sample",
                "stop",
                "shutdown",
            ],
            "safety": copy.deepcopy(EXPECTED_WORKER_SAFETY),
        }
        for capability in (
            "process_signals_interrupt_active_pulses",
            "channel_close_interrupts_active_pulses",
        ):
            for mutation in ("missing", "false"):
                with self.subTest(capability=capability, mutation=mutation):
                    result = copy.deepcopy(base_result)
                    if mutation == "missing":
                        del result["safety"][capability]
                    else:
                        result["safety"][capability] = False
                    transport = EV3NavigationSSHTransport(
                        target="robot@ev3.local",
                        controller_id="ev3-main",
                        remote_worker_path="/home/robot/navigation_worker.py",
                    )
                    with self.assertRaises(EV3NavigationTransportError):
                        transport._validate_success_result(
                            "describe",
                            {},
                            {"result": result},
                        )

    def test_transport_rejects_tampered_scan_encoder_receipt(self):
        clock = FakeClock()
        worker = FakeScanTransport(clock)
        response = worker._scan_turn(30_000)
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path="/home/robot/navigation_worker.py",
        )
        transport._validate_success_result(
            "scan_turn",
            {"relative_delta_mdeg": 30_000},
            response,
        )

        tampered = copy.deepcopy(response)
        motor = tampered["result"]["outcome"]["slices"][0]["motors"][0]
        motor["position_delta"] += 1
        with self.assertRaises(EV3NavigationTransportError):
            transport._validate_success_result(
                "scan_turn",
                {"relative_delta_mdeg": 30_000},
                tampered,
            )

    def test_transport_accepts_recovered_scan_encoder_receipt(self):
        clock = FakeClock()
        worker = FakeScanTransport(clock)
        response = worker._scan_turn(30_000)
        response["result"]["outcome"]["slices"][0][
            "reason"
        ] = "encoder_recovered"
        response["result"]["observation"]["last_outcome"] = copy.deepcopy(
            response["result"]["outcome"]
        )
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path="/home/robot/navigation_worker.py",
        )

        transport._validate_success_result(
            "scan_turn",
            {"relative_delta_mdeg": 30_000},
            response,
        )

    def test_cancellation_closes_input_and_terminates_ssh_process(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path="/home/robot/navigation_worker.py",
        )
        process = FakeProcess()
        transport._process = process

        with self.assertRaises(EV3NavigationTransportError) as caught:
            transport.request(
                "observe",
                {},
                1.0,
                cancel_requested=lambda: True,
            )

        self.assertIn("cancelled", str(caught.exception))
        self.assertTrue(process.stdin.closed)
        self.assertTrue(process.terminated)
        self.assertTrue(transport.aborted)


if __name__ == "__main__":
    unittest.main()
