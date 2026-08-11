"""Persistent BLAST telemetry and bounded manual dashboard controls."""

from __future__ import annotations

import asyncio
from concurrent.futures import Future, TimeoutError as FutureTimeoutError
from copy import deepcopy
from dataclasses import dataclass
import math
from queue import Empty, Full, Queue
import threading
import time
from typing import Callable, Mapping, Optional

from .blast_ble_runtime import BlastBLERuntime, DEFAULT_HUB_NAME
from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)


SNAPSHOT_SCHEMA = "controller-runtime-observation/v1"
ROBOT_ID = "blast-01"
CONTROLLER_ID = "blast-01.hub"
DEFAULT_POLL_INTERVAL_SECONDS = 1.0
DEFAULT_RECONNECT_INTERVAL_SECONDS = 3.0
DISCONNECT_TIMEOUT_SECONDS = 3.0
COMMAND_TIMEOUT_SECONDS = 15.0
INTERNAL_COMMAND_TIMEOUT_SECONDS = 12.0
SCAN_COMMAND_TIMEOUT_SECONDS = 36.0
SCAN_INTERNAL_COMMAND_TIMEOUT_SECONDS = 33.0
MIN_COMMAND_BUDGET_SECONDS = 2.0
COMMAND_RESPONSE_MARGIN_SECONDS = 0.25
MOTION_TIMEOUT_SECONDS = 4.0
MOTION_POLL_INTERVAL_SECONDS = 0.05
POST_MOTION_SETTLE_TIMEOUT_SECONDS = 1.5
SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS = 3.0
POST_MOTION_SETTLE_SAMPLE_COUNT = 5
POST_MOTION_DISTANCE_RANGE_MM = 5.0
POST_MOTION_TILT_RANGE_DEG = 1.0
COMMAND_RESULT_SCHEMA = "controller-command-result/v1"
SCAN_COMMAND = "scan_front_arc"
SETTLED_OBSERVATION_COMMAND = "observe_settled"
SCAN_RESULT_SCHEMA = "blast-scan-front-arc/v3"
SCAN_RESTORATION_TOLERANCE_DEG = 5.0
PYBRICKS_ULTRASONIC_NO_VALID_DISTANCE_MM = 2_000
RANGE_STATE_MEASURED = "MEASURED"
RANGE_STATE_NO_VALID_DISTANCE = "NO_VALID_DISTANCE"
RANGE_STATE_INVALID = "INVALID"
SCAN_RAY_EVIDENCE_SETTLED = "SETTLED_RANGE"
SCAN_RAY_EVIDENCE_SWEEP_ONLY = "SWEEP_CONTINUATION_ONLY"
SCAN_RAY_SIDES = (
    "center",
    "left_near",
    "left_far",
    "right_near",
    "right_far",
)
COMMANDS = {
    "drive_forward": ("drive_pulse", "forward"),
    "drive_reverse": ("drive_pulse", "reverse"),
    "turn_left": ("turn_pulse", "left"),
    "turn_right": ("turn_pulse", "right"),
    "claw_open": ("claw_pulse", "open"),
    "claw_close": ("claw_pulse", "close"),
    "body_left": ("body_pulse", "left"),
    "body_right": ("body_pulse", "right"),
    SCAN_COMMAND: (None, None),
    SETTLED_OBSERVATION_COMMAND: (None, None),
    "stop": ("stop", None),
}
NAVIGATION_MOTION_COMMANDS = {
    "drive_forward",
    "drive_reverse",
    "turn_left",
    "turn_right",
}


class BlastControllerError(RuntimeError):
    """A bounded BLAST command could not be accepted or verified."""

    def __init__(
        self, code: str, message: str, *, motion_started=None,
    ) -> None:
        self.code = code
        self.motion_started = motion_started
        super().__init__(message)


@dataclass(frozen=True)
class _BlastNoReturnScanPermit:
    """One short-lived host permit for a geometry-checked scan."""

    runtime_generation: int
    expires_at_monotonic_ns: int
    drive_angles_deg: tuple[float, float]
    heading_deg: float


def blast_range_state(distance_mm):
    """Classify Pybricks ultrasonic output without inventing clearance."""

    if (
        isinstance(distance_mm, bool)
        or not isinstance(distance_mm, (int, float))
        or not math.isfinite(float(distance_mm))
        or not 0 <= float(distance_mm) <= (
            PYBRICKS_ULTRASONIC_NO_VALID_DISTANCE_MM
        )
    ):
        return RANGE_STATE_INVALID
    if float(distance_mm) == PYBRICKS_ULTRASONIC_NO_VALID_DISTANCE_MM:
        return RANGE_STATE_NO_VALID_DISTANCE
    return RANGE_STATE_MEASURED


def _body_motor_angle(observation):
    motor_angles = (
        observation.get("motor_angles_deg")
        if isinstance(observation, Mapping)
        else None
    )
    value = (
        motor_angles.get("body")
        if isinstance(motor_angles, Mapping)
        else None
    )
    return value if type(value) is int else None


def validate_blast_scan_ray_contract(scan):
    """Validate ray order, range state, and body-angle telemetry."""

    if not isinstance(scan, Mapping) or scan.get("schema") != (
        SCAN_RESULT_SCHEMA
    ):
        raise ValueError("BLAST scan result is invalid")
    rays = scan.get("rays")
    if (
        not isinstance(rays, list)
        or tuple(
            ray.get("side") if isinstance(ray, Mapping) else None
            for ray in rays
        )
        != SCAN_RAY_SIDES
        or type(scan.get("all_observations_settled")) is not bool
        or scan.get("all_observations_settled") != all(
            isinstance(ray, Mapping)
            and ray.get("observation_settled") is True
            for ray in rays
        )
        or any(
            (
                type(ray.get("observation_settled")) is not bool
                or (
                    ray.get("observation_settled") is True
                    and ray.get(
                        "evidence_use", SCAN_RAY_EVIDENCE_SETTLED
                    ) != SCAN_RAY_EVIDENCE_SETTLED
                )
                or (
                    ray.get("observation_settled") is False
                    and ray.get("evidence_use")
                    != SCAN_RAY_EVIDENCE_SWEEP_ONLY
                )
                or ray.get("range_state") != blast_range_state(
                    ray.get("distance_mm")
                )
                or "body_motor_angle_deg" not in ray
                or (
                    ray.get("body_motor_angle_deg") is not None
                    and type(ray.get("body_motor_angle_deg")) is not int
                )
            )
            for ray in rays
        )
    ):
        raise ValueError("BLAST scan result is invalid")
    return deepcopy(scan)


class BlastObservationMonitor:
    """Own one BLE session for telemetry and serialized fixed commands."""

    def __init__(
        self,
        *,
        hub_name: str = DEFAULT_HUB_NAME,
        poll_interval_seconds: float = DEFAULT_POLL_INTERVAL_SECONDS,
        reconnect_interval_seconds: float = DEFAULT_RECONNECT_INTERVAL_SECONDS,
        runtime_factory: Callable[..., object] = BlastBLERuntime,
    ) -> None:
        intervals = (poll_interval_seconds, reconnect_interval_seconds)
        valid_intervals = all(
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(float(value))
            and 0.05 <= float(value) <= 60.0
            for value in intervals
        )
        if not (
            isinstance(hub_name, str)
            and hub_name.strip()
            and valid_intervals
            and callable(runtime_factory)
        ):
            raise ValueError("BLAST observation configuration is invalid")
        self._hub_name = hub_name.strip()
        self._poll_interval_seconds = float(poll_interval_seconds)
        self._reconnect_interval_seconds = float(reconnect_interval_seconds)
        self._runtime_factory = runtime_factory
        self._lifecycle_lock = threading.RLock()
        self._lock = threading.RLock()
        self._stop_requested = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._loop = None
        self._task = None
        self._runtime_generation = 0
        self._settling_samples = ()
        self._pending_command = None
        self._preempt_stop_request = None
        self._issued_scan_permit = None
        self._command_queue = Queue(maxsize=1)
        self._command_available = threading.Event()
        self._snapshot = {
            "schema": SNAPSHOT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "display_name": "BLAST",
            "hub_name": self._hub_name,
            "state": "configured",
            "reason_code": "observer_not_started",
            "last_observed_at_unix_ms": None,
            "last_observed_at_monotonic_ms": None,
            "ready": None,
            "observation": None,
        }

    def start(self) -> None:
        with self._lifecycle_lock:
            with self._lock:
                if self._thread is not None:
                    raise RuntimeError(
                        "BLAST observation monitor already started"
                    )
                self._stop_requested.clear()
                self._command_available.clear()
                self._set_state("connecting", "connecting")
                self._thread = threading.Thread(
                    target=self._run,
                    name="robot-llm-blast-observer",
                    daemon=True,
                )
                try:
                    self._thread.start()
                except Exception:
                    self._thread = None
                    self._set_state("offline", "observer_start_failed")
                    raise

    def connect(self):
        with self._lifecycle_lock:
            with self._lock:
                running = self._thread is not None
            if not running:
                self.start()
            return self.snapshot()

    def disconnect(self):
        with self._lifecycle_lock:
            self.close()
            return self.snapshot()

    def retry(self):
        with self._lifecycle_lock:
            self.close()
            self.start()
            return self.snapshot()

    def snapshot(self):
        with self._lock:
            return deepcopy(self._snapshot)

    def issue_no_return_scan_permit(
        self, *, pose, prior_receipt, geometry_checked,
    ):
        """Issue one short-lived token from exact same-pose host evidence."""

        with self._lock:
            snapshot = deepcopy(self._snapshot)
            runtime_generation = self._runtime_generation
        observation = snapshot.get("observation")
        previous = prior_receipt.get("result_observation") if isinstance(
            prior_receipt, Mapping
        ) else None
        motors = observation.get("motor_angles_deg") if isinstance(
            observation, Mapping
        ) else None
        previous_motors = previous.get("motor_angles_deg") if isinstance(
            previous, Mapping
        ) else None
        imu = observation.get("imu") if isinstance(observation, Mapping) else None
        heading = imu.get("heading_deg") if isinstance(imu, Mapping) else None
        now_ns = time.monotonic_ns()
        observed_ms = snapshot.get("last_observed_at_monotonic_ms")
        roles = ("left_drive", "right_drive")
        valid_pose = isinstance(pose, Mapping) and all(
            type(pose.get(key)) is int
            for key in ("x_mm", "y_mm", "heading_mdeg")
        )
        if not (
            geometry_checked is True
            and snapshot.get("state") == "online"
            and valid_pose
            and isinstance(observed_ms, int)
            and 0 <= now_ns // 1_000_000 - observed_ms <= 3_000
            and isinstance(prior_receipt, Mapping)
            and prior_receipt.get("observation_settled") is True
            and prior_receipt.get("pose") == dict(pose)
            and (
                not isinstance(prior_receipt.get("motion"), Mapping)
                or prior_receipt["motion"].get("command_completed") is True
            )
            and isinstance(previous, Mapping)
            and previous.get("motion_active") is False
            and blast_range_state(previous.get("distance_mm"))
            == RANGE_STATE_NO_VALID_DISTANCE
            and isinstance(observation, Mapping)
            and observation.get("motion_active") is False
            and blast_range_state(observation.get("distance_mm"))
            == RANGE_STATE_NO_VALID_DISTANCE
            and isinstance(motors, Mapping)
            and isinstance(previous_motors, Mapping)
            and all(
                isinstance(motors.get(role), (int, float))
                and not isinstance(motors.get(role), bool)
                and math.isfinite(float(motors[role]))
                and isinstance(previous_motors.get(role), (int, float))
                and not isinstance(previous_motors.get(role), bool)
                and math.isfinite(float(previous_motors[role]))
                and float(motors[role]) == float(previous_motors[role])
                for role in roles
            )
            and isinstance(heading, (int, float))
            and not isinstance(heading, bool)
            and math.isfinite(float(heading))
        ):
            return None
        permit = _BlastNoReturnScanPermit(
            runtime_generation=runtime_generation,
            expires_at_monotonic_ns=now_ns + 5_000_000_000,
            drive_angles_deg=tuple(float(motors[role]) for role in roles),
            heading_deg=float(heading),
        )
        with self._lock:
            if (
                self._runtime_generation != runtime_generation
                or self._snapshot.get("state") != "online"
            ):
                return None
            self._issued_scan_permit = permit
        return permit

    def command(
        self, command: str, *, cancel_requested=None, action_permit=None,
    ):
        if command not in COMMANDS or (
            cancel_requested is not None
            and not callable(cancel_requested)
        ) or (
            action_permit is not None
            and not (
                command == SCAN_COMMAND
                and isinstance(action_permit, _BlastNoReturnScanPermit)
            )
        ):
            raise ValueError("unsupported BLAST command")
        timeout_seconds = (
            SCAN_COMMAND_TIMEOUT_SECONDS
            if command == SCAN_COMMAND
            else COMMAND_TIMEOUT_SECONDS
        )
        with self._lock:
            issued_permit = (
                self._issued_scan_permit
                if isinstance(action_permit, _BlastNoReturnScanPermit)
                else None
            )
            if action_permit is not None and (
                issued_permit is not action_permit
                or action_permit.runtime_generation
                != self._runtime_generation
                or action_permit.expires_at_monotonic_ns < time.monotonic_ns()
            ):
                raise BlastControllerError(
                    "scan_start_clearance_unverified",
                    "BLAST scan permit is stale or was not issued here",
                    motion_started=False,
                )
            if self._snapshot["state"] != "online":
                raise BlastControllerError(
                    "controller_unavailable",
                    "BLAST is not connected",
                )
            if cancel_requested is not None and cancel_requested():
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST command was cancelled before motor start",
                    motion_started=False,
                )
            preempts_active_command = (
                command == "stop"
                and self._pending_command is not None
                and self._pending_command != "stop"
            )
            if preempts_active_command:
                if self._preempt_stop_request is not None:
                    raise BlastControllerError(
                        "controller_busy",
                        "BLAST already has a pending stop command",
                    )
                result = Future()
                self._preempt_stop_request = (
                    self._runtime_generation,
                    result,
                    time.monotonic() + timeout_seconds,
                )
                self._command_available.set()
            elif (
                self._pending_command is not None
                or self._preempt_stop_request is not None
            ):
                raise BlastControllerError(
                    "controller_busy",
                    "BLAST already has a pending command",
                )
            else:
                self._pending_command = command
                result = Future()
                request = (
                    self._runtime_generation,
                    command,
                    result,
                    time.monotonic() + timeout_seconds,
                    action_permit,
                )
                if action_permit is not None:
                    self._issued_scan_permit = None
                try:
                    self._command_queue.put_nowait(request)
                except Full:
                    self._pending_command = None
                    raise BlastControllerError(
                        "controller_busy",
                        "BLAST command queue is full",
                    ) from None
                self._command_available.set()
        try:
            return result.result(timeout=timeout_seconds)
        except FutureTimeoutError:
            result.cancel()
            raise BlastControllerError(
                "controller_command_timeout",
                "BLAST command did not finish in time",
            ) from None

    def close(self) -> None:
        with self._lifecycle_lock:
            with self._lock:
                thread = self._thread
                self._issued_scan_permit = None
            if thread is None:
                self._set_state("stopped", "observer_stopped")
                return
            self._set_state("offline", "disconnecting")
            self._stop_requested.set()
            self._command_available.set()
            with self._lock:
                loop = self._loop
                task = self._task
            if loop is not None and task is not None:
                try:
                    loop.call_soon_threadsafe(task.cancel)
                except RuntimeError:
                    pass
            thread.join(timeout=12.0)
            if thread.is_alive():
                raise RuntimeError("BLAST observation monitor did not stop")
            self._reject_commands(
                BlastControllerError(
                    "controller_unavailable",
                    "BLAST connection ended",
                )
            )
            with self._lock:
                self._thread = None

    def _set_state(self, state: str, reason_code: Optional[str], **changes) -> None:
        with self._lock:
            snapshot = deepcopy(self._snapshot)
            snapshot.update(changes)
            snapshot["state"] = state
            snapshot["reason_code"] = reason_code
            self._snapshot = snapshot

    def _run(self) -> None:
        async def run_observer() -> None:
            with self._lock:
                self._loop = asyncio.get_running_loop()
                self._task = asyncio.current_task()
            try:
                await self._observe()
            except asyncio.CancelledError:
                pass
            finally:
                with self._lock:
                    self._loop = None
                    self._task = None

        asyncio.run(run_observer())

    async def _observe(self) -> None:
        try:
            while not self._stop_requested.is_set():
                try:
                    await self._observe_session()
                except asyncio.CancelledError:
                    raise
                except Exception:
                    previous = self.snapshot()
                    reason = (
                        "observation_failed"
                        if previous["last_observed_at_unix_ms"] is not None
                        else "connection_failed"
                    )
                    self._set_state("offline", reason)
                if self._stop_requested.is_set():
                    break
                if await self._wait(self._reconnect_interval_seconds):
                    break
                self._set_state("connecting", "reconnecting")
        finally:
            if self._stop_requested.is_set():
                self._set_state("stopped", "observer_stopped")

    async def _observe_session(self) -> None:
        runtime = self._runtime_factory(hub_name=self._hub_name)
        try:
            ready = await runtime.connect()
            if (
                ready.get("robot_id") != ROBOT_ID
                or ready.get("controller_id") != CONTROLLER_ID
            ):
                raise RuntimeError("BLAST controller identity mismatch")
            with self._lock:
                self._runtime_generation += 1
                generation = self._runtime_generation
                self._issued_scan_permit = None
            self._set_state("online", None, ready=ready)
            while not self._stop_requested.is_set():
                preempted = await self._service_preempt_stop(
                    runtime,
                    generation,
                )
                request = self._next_command()
                if preempted and request is not None:
                    self._finish_command(
                        request[2],
                        error=BlastControllerError(
                            "controller_command_interrupted",
                            "BLAST command was interrupted by stop",
                            motion_started=False,
                        ),
                    )
                elif request is None:
                    if not preempted:
                        self._publish_observation(await runtime.observe())
                else:
                    await self._execute_command(
                        runtime,
                        generation,
                        request,
                    )
                if await self._wait_for_work(
                    self._poll_interval_seconds
                ):
                    break
        finally:
            self._reject_commands(
                BlastControllerError(
                    "controller_unavailable",
                    "BLAST connection ended",
                )
            )
            try:
                await asyncio.wait_for(
                    runtime.close(),
                    timeout=DISCONNECT_TIMEOUT_SECONDS,
                )
            except Exception:
                pass

    def _next_command(self):
        try:
            return self._command_queue.get_nowait()
        except Empty:
            return None

    async def _execute_command(self, runtime, generation, request) -> None:
        requested_generation, command, result, expires_at, permit = request
        if result.cancelled() or time.monotonic() >= expires_at:
            self._finish_command(result)
            return
        remaining = expires_at - time.monotonic()
        if remaining < MIN_COMMAND_BUDGET_SECONDS:
            self._finish_command(
                result,
                error=BlastControllerError(
                    "controller_command_timeout",
                    "BLAST command expired before execution",
                ),
            )
            return
        try:
            if requested_generation != generation:
                raise BlastControllerError(
                    "stale_controller_command",
                    "BLAST reconnected before the command ran",
                )
            with self._lock:
                stop_won_before_start = (
                    command != "stop"
                    and self._preempt_stop_request is not None
                )
            if stop_won_before_start:
                await self._service_preempt_stop(runtime, generation)
                self._finish_command(
                    result,
                    error=BlastControllerError(
                        "controller_command_interrupted",
                        "BLAST command was interrupted by stop",
                        motion_started=False,
                    ),
                )
                return
            internal_timeout = (
                SCAN_INTERNAL_COMMAND_TIMEOUT_SECONDS
                if command == SCAN_COMMAND
                else INTERNAL_COMMAND_TIMEOUT_SECONDS
            )
            value = await asyncio.wait_for(
                self._perform_command(
                    runtime, generation, command, action_permit=permit,
                ),
                timeout=min(
                    internal_timeout,
                    remaining - COMMAND_RESPONSE_MARGIN_SECONDS,
                ),
            )
            self._finish_command(result, value=value)
        except asyncio.CancelledError:
            self._finish_command(
                result,
                error=BlastControllerError(
                    "controller_unavailable",
                    "BLAST controller stopped",
                ),
            )
            raise
        except asyncio.TimeoutError:
            failure = BlastControllerError(
                "controller_command_timeout",
                "BLAST command did not finish in time",
            )
            self._finish_command(result, error=failure)
            raise failure
        except Exception as error:
            failure = (
                error
                if isinstance(error, BlastControllerError)
                else BlastControllerError(
                    "controller_command_failed",
                    "BLAST command failed",
                )
            )
            self._finish_command(result, error=failure)
            if failure.code in (
                "controller_command_interrupted",
                "scan_start_clearance_unverified",
                "scan_sweep_clearance_lost",
                "scan_sweep_observation_unverified",
            ):
                return
            raise failure

    @staticmethod
    def _finite_number(value):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            return None
        return float(value)

    @classmethod
    def _scan_heading(cls, observation):
        imu = observation.get("imu")
        return cls._finite_number(
            imu.get("heading_deg") if isinstance(imu, dict) else None
        )

    @staticmethod
    def _scan_heading_delta(heading, reference):
        if heading is None or reference is None:
            return None
        return (heading - reference + 180.0) % 360.0 - 180.0

    @classmethod
    def _scan_ray(
        cls,
        side,
        observation,
        start_heading,
        observation_settled,
        evidence_use=SCAN_RAY_EVIDENCE_SETTLED,
    ):
        heading = cls._scan_heading(observation)
        observed_at_ms = observation.get("observed_at_ms")
        if type(observed_at_ms) is not int:
            observed_at_ms = None
        distance_mm = cls._finite_number(observation.get("distance_mm"))
        return {
            "side": side,
            "distance_mm": distance_mm,
            "range_state": blast_range_state(distance_mm),
            "body_motor_angle_deg": _body_motor_angle(observation),
            "heading_deg": heading,
            "relative_heading_deg": cls._scan_heading_delta(
                heading,
                start_heading,
            ),
            "observation_settled": observation_settled,
            "evidence_use": evidence_use,
            "observed_at_ms": observed_at_ms,
        }

    def _scan_sweep_window_allows_continuation(self, observation) -> bool:
        samples = self._settling_samples
        if (
            observation.get("motion_active") is not False
            or len(samples) != POST_MOTION_SETTLE_SAMPLE_COUNT
            or self._scan_heading(observation) is None
            or not (
                BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
                .range_sensor_extrinsics
                .matches_navigation_body_angle(_body_motor_angle(observation))
            )
        ):
            return False
        tilt_ranges = tuple(
            max(sample[index] for sample in samples)
            - min(sample[index] for sample in samples)
            for index in (1, 2)
        )
        if any(value > POST_MOTION_TILT_RANGE_DEG for value in tilt_ranges):
            return False
        minimum = (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
            .minimum_rotation_clearance_mm()
        )
        return all(
            blast_range_state(sample[0]) == RANGE_STATE_NO_VALID_DISTANCE
            or (
                blast_range_state(sample[0]) == RANGE_STATE_MEASURED
                and sample[0] > minimum
            )
            for sample in samples
        )

    async def _scan_turn(
        self,
        runtime,
        generation,
        direction,
    ):
        if await self._service_preempt_stop(runtime, generation):
            raise BlastControllerError(
                "controller_command_interrupted",
                "BLAST scan was interrupted by stop",
                motion_started=False,
            )
        receipt = await runtime.turn_pulse(direction)
        observation = await self._observe_until_idle(
            runtime,
            generation=generation,
            stop_only=False,
        )
        observation, observation_settled = (
            await self._observe_until_settled(
                runtime,
                generation=generation,
                initial_observation=observation,
                timeout_seconds=SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
            )
        )
        distance = observation.get("distance_mm")
        sweep_only = observation_settled is not True
        if sweep_only and not self._scan_sweep_window_allows_continuation(
            observation
        ):
            raise BlastControllerError(
                "scan_sweep_observation_unverified",
                "BLAST scan could not settle safely between turn pulses",
            )
        sensor = (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
            .range_sensor_extrinsics
        )
        if (
            self._scan_heading(observation) is None
            or not sensor.matches_navigation_body_angle(
                _body_motor_angle(observation)
            )
        ):
            raise BlastControllerError(
                "scan_sweep_observation_unverified",
                "BLAST scan lost its calibrated sensor pose evidence",
            )
        range_state = blast_range_state(distance)
        if range_state == RANGE_STATE_INVALID:
            raise BlastControllerError(
                "scan_sweep_observation_unverified",
                "BLAST scan received invalid settled range evidence",
            )
        if range_state == RANGE_STATE_MEASURED and float(distance) <= (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
            .minimum_rotation_clearance_mm()
        ):
            raise BlastControllerError(
                "scan_sweep_clearance_lost",
                "BLAST scan stopped after a close settled observation",
            )
        return (
            receipt,
            observation,
            observation_settled,
            (
                SCAN_RAY_EVIDENCE_SWEEP_ONLY
                if sweep_only
                else SCAN_RAY_EVIDENCE_SETTLED
            ),
        )

    async def _perform_scan_front_arc(
        self, runtime, generation, *, action_permit=None,
    ):
        center = await self._observe_until_idle(
            runtime,
            generation=generation,
            stop_only=False,
        )
        center, center_settled = await self._observe_until_settled(
            runtime,
            generation=generation,
            initial_observation=center,
            timeout_seconds=SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
        )
        start_heading = self._scan_heading(center)
        sensor = (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
            .range_sensor_extrinsics
        )
        center_distance = center.get("distance_mm")
        center_motors = center.get("motor_angles_deg")
        center_drive = tuple(
            center_motors.get(role)
            if isinstance(center_motors, Mapping) else None
            for role in ("left_drive", "right_drive")
        )
        permit_allows_no_return = (
            isinstance(action_permit, _BlastNoReturnScanPermit)
            and time.monotonic_ns()
            <= action_permit.expires_at_monotonic_ns
            and all(
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and float(value) == expected
                for value, expected in zip(
                    center_drive, action_permit.drive_angles_deg
                )
            )
            and start_heading is not None
            and abs(
                (start_heading - action_permit.heading_deg + 180.0)
                % 360.0 - 180.0
            ) <= SCAN_RESTORATION_TOLERANCE_DEG
        )
        center_state = blast_range_state(center_distance)
        range_allows_start = (
            center_state == RANGE_STATE_MEASURED
            and float(center_distance) > (
                BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
                .minimum_rotation_clearance_mm()
            )
        ) or (
            permit_allows_no_return
            and center_state == RANGE_STATE_NO_VALID_DISTANCE
        )
        if not (
            center_settled
            and start_heading is not None
            and sensor.matches_navigation_body_angle(
                _body_motor_angle(center)
            )
            and range_allows_start
        ):
            raise BlastControllerError(
                "scan_start_clearance_unverified",
                "BLAST scan cannot rotate without a settled measured "
                "front clearance and navigation sensor pose",
            )

        (
            _left_near_receipt,
            left_near,
            left_near_settled,
            left_near_evidence,
        ) = (
            await self._scan_turn(
                runtime,
                generation,
                "left",
            )
        )
        (
            _left_far_receipt,
            left_far,
            left_far_settled,
            left_far_evidence,
        ) = (
            await self._scan_turn(
                runtime,
                generation,
                "left",
            )
        )
        await self._scan_turn(
            runtime,
            generation,
            "right",
        )
        await self._scan_turn(
            runtime,
            generation,
            "right",
        )
        (
            _right_near_receipt,
            right_near,
            right_near_settled,
            right_near_evidence,
        ) = (
            await self._scan_turn(
                runtime,
                generation,
                "right",
            )
        )
        (
            _right_far_receipt,
            right_far,
            right_far_settled,
            right_far_evidence,
        ) = (
            await self._scan_turn(
                runtime,
                generation,
                "right",
            )
        )
        await self._scan_turn(
            runtime,
            generation,
            "left",
        )
        (
            _return_receipt,
            final,
            final_settled,
            _final_evidence,
        ) = await self._scan_turn(runtime, generation, "left")

        final_heading = self._scan_heading(final)
        restoration_error = self._scan_heading_delta(
            final_heading,
            start_heading,
        )
        restoration_verified = (
            restoration_error is not None
            and abs(restoration_error) <= SCAN_RESTORATION_TOLERANCE_DEG
        )
        scan = {
            "schema": SCAN_RESULT_SCHEMA,
            "state": "complete",
            "result": (
                "restored"
                if restoration_verified
                else "restoration_unverified"
            ),
            "start_heading_deg": start_heading,
            "final_heading_deg": final_heading,
            "restoration_error_deg": restoration_error,
            "restoration_verified": restoration_verified,
            "all_observations_settled": all((
                center_settled,
                left_near_settled,
                left_far_settled,
                right_near_settled,
                right_far_settled,
            )),
            "rays": [
                self._scan_ray(
                    "center",
                    center,
                    start_heading,
                    center_settled,
                ),
                self._scan_ray(
                    "left_near",
                    left_near,
                    start_heading,
                    left_near_settled,
                    left_near_evidence,
                ),
                self._scan_ray(
                    "left_far",
                    left_far,
                    start_heading,
                    left_far_settled,
                    left_far_evidence,
                ),
                self._scan_ray(
                    "right_near",
                    right_near,
                    start_heading,
                    right_near_settled,
                    right_near_evidence,
                ),
                self._scan_ray(
                    "right_far",
                    right_far,
                    start_heading,
                    right_far_settled,
                    right_far_evidence,
                ),
            ],
        }
        return {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": SCAN_COMMAND,
            "accepted": True,
            "completed": True,
            "receipt": {"turn_count": 8},
            "observation": final,
            "observation_settled": final_settled,
            "scan": scan,
        }

    async def _perform_command(
        self, runtime, generation, command: str, *, action_permit=None,
    ):
        if command == SCAN_COMMAND:
            return await self._perform_scan_front_arc(
                runtime, generation, action_permit=action_permit,
            )
        if command == SETTLED_OBSERVATION_COMMAND:
            receipt = {"motion_started": False}
        else:
            operation, direction = COMMANDS[command]
            method = getattr(runtime, operation)
            receipt = (
                await method()
                if direction is None
                else await method(direction)
            )
        observation = await self._observe_until_idle(
            runtime,
            generation=generation,
            stop_only=command == "stop",
        )
        observation_settled = None
        if (
            command in NAVIGATION_MOTION_COMMANDS
            or command == SETTLED_OBSERVATION_COMMAND
        ):
            observation, observation_settled = (
                await self._observe_until_settled(
                    runtime,
                    generation=generation,
                    initial_observation=observation,
                    timeout_seconds=(
                        SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS
                        if command in ("turn_left", "turn_right")
                        else None
                    ),
                )
            )
        result = {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": command,
            "accepted": True,
            "completed": True,
            "receipt": receipt,
            "observation": observation,
        }
        if observation_settled is not None:
            result["observation_settled"] = observation_settled
        return result

    async def _observe_until_idle(
        self,
        runtime,
        *,
        generation,
        stop_only: bool,
    ):
        deadline = asyncio.get_running_loop().time() + (
            0.0 if stop_only else MOTION_TIMEOUT_SECONDS
        )
        while True:
            if (
                not stop_only
                and await self._service_preempt_stop(runtime, generation)
            ):
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST command was interrupted by stop",
                )
            observation = await runtime.observe()
            self._publish_observation(observation)
            if observation.get("motion_active") is False:
                return observation
            if stop_only or asyncio.get_running_loop().time() >= deadline:
                if not stop_only:
                    try:
                        await runtime.stop()
                    except Exception:
                        pass
                raise BlastControllerError(
                    "controller_motion_not_stopped",
                    "BLAST motion did not stop as expected",
                )
            await asyncio.sleep(MOTION_POLL_INTERVAL_SECONDS)

    @staticmethod
    def _settling_sample(observation):
        distance = observation.get("distance_mm")
        imu = observation.get("imu")
        tilt = None
        if isinstance(imu, dict):
            for key in ("tilt_deg", "raw_tilt_deg"):
                candidate = imu.get(key)
                if isinstance(candidate, (list, tuple)) and len(candidate) == 2:
                    tilt = candidate
                    break
        values = (
            distance,
            *(tilt if isinstance(tilt, (list, tuple)) else ()),
        )
        if (
            len(values) != 3
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in values
            )
        ):
            return None
        return tuple(float(value) for value in values)

    @staticmethod
    def _settling_window_is_stable(samples) -> bool:
        if len(samples) < POST_MOTION_SETTLE_SAMPLE_COUNT:
            return False
        ranges = tuple(
            max(sample[index] for sample in samples)
            - min(sample[index] for sample in samples)
            for index in range(3)
        )
        return (
            ranges[0] <= POST_MOTION_DISTANCE_RANGE_MM
            and ranges[1] <= POST_MOTION_TILT_RANGE_DEG
            and ranges[2] <= POST_MOTION_TILT_RANGE_DEG
        )

    async def _observe_until_settled(
        self,
        runtime,
        *,
        generation,
        initial_observation,
        timeout_seconds=None,
    ):
        settle_timeout = (
            POST_MOTION_SETTLE_TIMEOUT_SECONDS
            if timeout_seconds is None
            else timeout_seconds
        )
        deadline = (
            asyncio.get_running_loop().time()
            + settle_timeout
        )
        latest = initial_observation
        samples = []
        self._settling_samples = ()
        while True:
            if await self._service_preempt_stop(runtime, generation):
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST command was interrupted by stop",
                )
            sample = self._settling_sample(latest)
            if latest.get("motion_active") is False and sample is not None:
                samples.append(sample)
                del samples[:-POST_MOTION_SETTLE_SAMPLE_COUNT]
            else:
                samples.clear()
            settled = self._settling_window_is_stable(samples)
            timed_out = asyncio.get_running_loop().time() >= deadline
            if settled or timed_out:
                self._settling_samples = tuple(samples)
                if await self._service_preempt_stop(runtime, generation):
                    raise BlastControllerError(
                        "controller_command_interrupted",
                        "BLAST command was interrupted by stop",
                    )
                # Stabilisation improves evidence quality, but must not turn a
                # usable bounded movement into a failed robot episode.
                return latest, settled
            await asyncio.sleep(MOTION_POLL_INTERVAL_SECONDS)
            latest = await runtime.observe()
            self._publish_observation(latest)

    async def _service_preempt_stop(self, runtime, generation) -> bool:
        with self._lock:
            request = self._preempt_stop_request
        if request is None:
            return False
        requested_generation, result, expires_at = request
        if result.cancelled() or time.monotonic() >= expires_at:
            self._finish_preempt_stop(
                result,
                error=(
                    None
                    if result.cancelled()
                    else BlastControllerError(
                        "controller_command_timeout",
                        "BLAST stop command expired before execution",
                    )
                ),
            )
            return False
        try:
            if requested_generation != generation:
                raise BlastControllerError(
                    "stale_controller_command",
                    "BLAST reconnected before stop ran",
                )
            self._finish_preempt_stop(
                result,
                value=await self._perform_command(
                    runtime,
                    generation,
                    "stop",
                ),
            )
            return True
        except asyncio.CancelledError:
            self._finish_preempt_stop(
                result,
                error=BlastControllerError(
                    "controller_unavailable",
                    "BLAST controller stopped",
                ),
            )
            raise
        except Exception as error:
            failure = (
                error
                if isinstance(error, BlastControllerError)
                else BlastControllerError(
                    "controller_command_failed",
                    "BLAST stop command failed",
                )
            )
            self._finish_preempt_stop(result, error=failure)
            raise failure

    def _publish_observation(self, observation) -> None:
        self._set_state(
            "online",
            None,
            observation=observation,
            last_observed_at_unix_ms=time.time_ns() // 1_000_000,
            last_observed_at_monotonic_ms=(
                time.monotonic_ns() // 1_000_000
            ),
        )

    def _finish_command(self, result, *, value=None, error=None):
        with self._lock:
            self._pending_command = None
        if not result.done():
            if error is None:
                result.set_result(value)
            else:
                result.set_exception(error)

    def _finish_preempt_stop(self, result, *, value=None, error=None):
        with self._lock:
            request = self._preempt_stop_request
            if request is not None and request[1] is result:
                self._preempt_stop_request = None
        if not result.done():
            if error is None:
                result.set_result(value)
            else:
                result.set_exception(error)

    def _reject_commands(self, error) -> None:
        with self._lock:
            stop_request = self._preempt_stop_request
        if stop_request is not None:
            self._finish_preempt_stop(stop_request[1], error=error)
        while True:
            request = self._next_command()
            if request is None:
                return
            self._finish_command(
                request[2],
                error=error,
            )

    async def _wait_for_work(self, seconds: float) -> bool:
        await asyncio.to_thread(self._command_available.wait, seconds)
        self._command_available.clear()
        return self._stop_requested.is_set()

    async def _wait(self, seconds: float) -> bool:
        return await asyncio.to_thread(
            self._stop_requested.wait,
            seconds,
        )
