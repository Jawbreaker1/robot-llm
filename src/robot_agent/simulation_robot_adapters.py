"""Hardware-surface adapters for the shared multi-robot simulation world.

These classes translate existing BLAST and EV3 commands into simulated
motion and sensor readings.  They contain no route, waypoint or side-choice
logic; the production episode runtimes and their model planners retain that
responsibility.
"""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time
from typing import Mapping

from .ev3_navigation_transport import EV3NavigationRemoteError, EV3NavigationTransportError
from .blast_navigation_action_profile import (
    DRIVE_ENCODER_DEGREES,
    DRIVE_SPEED_DPS,
    SCAN_TRIM_ENCODER_DEGREES_PER_PULSE,
    TURN_DURATION_MS_PER_PULSE,
    TURN_ENCODER_DEGREES_PER_PULSE,
    TURN_TRIM_ENCODER_DEGREES,
    TURN_SPEED_DPS,
)
from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_observation_monitor import (
    COMMAND_RESULT_SCHEMA,
    CONTROLLER_ID,
    ROBOT_ID,
    SCAN_COMMAND,
    SETTLED_OBSERVATION_COMMAND,
    BlastControllerError,
)
from .blast_scan_observation import (
    SCAN_RAY_EVIDENCE_SETTLED,
    build_blast_encoder_scan,
    build_blast_front_arc_scan,
    build_blast_partial_scan,
    surroundings_scan_next_turn,
)
from .multi_robot_navigation_simulator import (
    MultiRobotNavigationSimulator,
)
from .physical_navigation_contract import (
    ADVANCE,
    EXPECTED_ACTION_SPECS,
    EXPECTED_WORKER_OPERATIONS,
    EXPECTED_WORKER_SAFETY,
    REVERSE,
    SCAN_TURN_OPERATION,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    expected_scan_sample_profile,
    expected_scan_turn_profile,
    expected_scan_turn_spec,
)
from .physical_odometry import OdometryCalibration, normalize_heading_mdeg


def _stop_proof() -> Mapping[str, object]:
    return {
        "stop_confirmed": True,
        "stop_attempts": [],
        "states": {},
        "positions": {},
        "errors": [],
        "fault_tokens": {},
    }


class SharedWorldBlastController:
    """BLAST's physical controller surface over one shared sim world."""

    def __init__(
        self,
        simulation: MultiRobotNavigationSimulator,
        *,
        world_robot_id: str,
        startup_scan: Mapping[str, object] | None = None,
        range_dropout_reads: int = 0,
    ) -> None:
        if type(range_dropout_reads) is not int or not 0 <= range_dropout_reads <= 100:
            raise ValueError("simulation range dropout length is invalid")
        self.simulation = simulation
        self.world_robot_id = world_robot_id
        self._initial_heading_mdeg = simulation.pose(
            world_robot_id
        ).heading_mdeg
        self._left_encoder_deg = 0
        self._right_encoder_deg = 0
        self._observed_ms = time.monotonic_ns() // 1_000_000
        self._generation = 1
        self.commands: list[str] = []
        self.scan_count = 0
        self._startup_scan = deepcopy(startup_scan)
        self._range_dropout_reads = range_dropout_reads
        self._drop_remaining = 0
        self._drop_injected = False

    def monotonic_ms(self) -> int:
        return self._observed_ms

    def runtime_generation(self) -> int:
        return self._generation

    def _distance_mm(self) -> int:
        if self._drop_remaining:
            self._drop_remaining -= 1
            return 2_000  # BLAST's no-valid-return sentinel, never free space.
        _footprint, sensor = (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
        )
        return self.simulation.scan(
            self.world_robot_id,
            (sensor.yaw_mdeg,),
            origin_forward_mm=sensor.forward_offset_mm,
            origin_left_mm=sensor.left_offset_mm,
        )[0][1]

    def _observation(self) -> Mapping[str, object]:
        pose = self.simulation.pose(self.world_robot_id)
        relative_heading = normalize_heading_mdeg(
            pose.heading_mdeg - self._initial_heading_mdeg
        )
        return {
            "distance_mm": self._distance_mm(),
            "motion_active": False,
            "motor_angles_deg": {
                "left_drive": self._left_encoder_deg,
                "right_drive": self._right_encoder_deg,
                "claw": 0,
                "body": 158,
            },
            # Physical BLAST reports the opposite IMU yaw sign.
            "imu": {
                "ready": True,
                "heading_deg": -relative_heading / 1_000.0,
            },
            "observed_at_ms": self._observed_ms,
        }

    def snapshot(self) -> Mapping[str, object]:
        return {
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "state": "online",
            "last_observed_at_unix_ms": (
                1_700_000_000_000 + self._observed_ms
            ),
            "last_observed_at_monotonic_ms": self._observed_ms,
            "observation": self._observation(),
        }

    def issue_no_return_scan_permit(
        self,
        *,
        expected_drive_angles,
        **_values,
    ):
        current = self._observation()["motor_angles_deg"]
        if not isinstance(expected_drive_angles, Mapping) or any(
            current[role] != expected_drive_angles.get(role)
            for role in ("left_drive", "right_drive")
        ):
            return None
        return {"simulation_only": True, "generation": self._generation}

    def _apply_encoder_motion(
        self,
        *,
        left_delta_deg: int,
        right_delta_deg: int,
    ) -> Mapping[str, object]:
        calibration = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
        applied_left_deg = left_delta_deg
        applied_right_deg = right_delta_deg
        if left_delta_deg == right_delta_deg:
            requested_mm = int(round(
                left_delta_deg * calibration.linear_mm_per_encoder_degree
            ))
            moved_mm = self.simulation.move(
                self.world_robot_id,
                requested_mm,
            )
            if moved_mm != requested_mm:
                applied_deg = int(round(
                    moved_mm / calibration.linear_mm_per_encoder_degree
                ))
                applied_left_deg = applied_deg
                applied_right_deg = applied_deg
        else:
            turn_mdeg = int(round(
                (right_delta_deg - left_delta_deg)
                / 2.0
                * calibration.turn_mdeg_per_opposed_encoder_degree
            ))
            before_heading = self.simulation.pose(self.world_robot_id).heading_mdeg
            after = self.simulation.rotate(self.world_robot_id, turn_mdeg)
            actual_turn = normalize_heading_mdeg(after.heading_mdeg - before_heading)
            if actual_turn != turn_mdeg:
                ratio = actual_turn / turn_mdeg if turn_mdeg else 0.0
                applied_left_deg = round(left_delta_deg * ratio)
                applied_right_deg = round(right_delta_deg * ratio)
        self._left_encoder_deg += applied_left_deg
        self._right_encoder_deg += applied_right_deg
        self._observed_ms += TURN_DURATION_MS_PER_PULSE
        return self._observation()

    def _turn_scan_pulse(
        self,
        direction: str,
        wheel_angle_deg: int = TURN_ENCODER_DEGREES_PER_PULSE,
    ):
        left = direction == "left"
        before = self._observation()["motor_angles_deg"]
        observation = self._apply_encoder_motion(
            left_delta_deg=(
                -wheel_angle_deg if left else wheel_angle_deg
            ),
            right_delta_deg=(
                wheel_angle_deg if left else -wheel_angle_deg
            ),
        )
        return (
            {
                "accepted": True,
                "direction": direction,
                "speed_dps": TURN_SPEED_DPS,
                "wheel_angle_deg": wheel_angle_deg,
                "before_angles_deg": {
                    role: before[role]
                    for role in ("left_drive", "right_drive")
                },
            },
            observation,
            True,
            SCAN_RAY_EVIDENCE_SETTLED,
        )

    def _scan_result(self, *, surroundings: bool):
        if surroundings and self._startup_scan is not None:
            return self._recorded_startup_scan_result()
        center = self._observation()
        start = {
            role: center["motor_angles_deg"][role]
            for role in ("left_drive", "right_drive")
        }
        def pulse_completed(sample):
            receipt, observation = sample[:2]
            before = receipt["before_angles_deg"]
            after = observation["motor_angles_deg"]
            return all(abs(after[role] - before[role]) >= receipt["wheel_angle_deg"]
                       for role in before)

        def partial(samples, final):
            return build_blast_partial_scan(
                center=center, center_settled=True, start_drive_angles=start,
                sweep_samples=samples, final=final, final_settled=True,
                final_body_verified=True,
            )

        if surroundings:
            samples, final = [], center
            while (next_turn := surroundings_scan_next_turn(
                center, start, final, len(samples),
            )) is not None:
                sample = self._turn_scan_pulse(
                    next_turn[0], SCAN_TRIM_ENCODER_DEGREES_PER_PULSE
                    if next_turn[1] == "trim" else TURN_ENCODER_DEGREES_PER_PULSE,
                )
                samples.append(sample)
                final = sample[1]
                if not pulse_completed(sample):
                    break
            scan = partial(samples, final) if samples and not pulse_completed(samples[-1]) else build_blast_encoder_scan(
                center=center,
                center_settled=True,
                start_drive_angles=start,
                sweep_samples=samples,
                final=final,
                final_settled=True,
                final_body_verified=True,
                sweep_turn_count=len(samples),
            )
            receipt = {
                "turn_count": len(samples),
                "coverage_complete": (
                    scan["state"] == "complete"
                ),
            }
        else:
            samples = []
            for direction in ("left",) * 4 + ("right",) * 8 + ("left",) * 4:
                sample = self._turn_scan_pulse(direction)
                samples.append(sample)
                if not pulse_completed(sample):
                    break
            final = samples[-1][1]
            # Match physical recovery: a stopped, incomplete sweep is useful
            # partial evidence, never a fabricated complete 16-pulse scan.
            scan = partial(samples, final) if not pulse_completed(samples[-1]) else build_blast_front_arc_scan(
                center=center,
                center_settled=True,
                start_drive_angles=start,
                left_outbound=samples[:4],
                right_outbound=samples[8:12],
                final=final,
                final_settled=True,
                final_body_verified=True,
            )
            receipt = {"turn_count": len(samples), "coverage": "front_arc"}
        return {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": SCAN_COMMAND,
            "accepted": True,
            "completed": True,
            "receipt": receipt,
            "observation": final,
            "observation_settled": True,
            "scan": scan,
        }

    def scan_surroundings(
        self,
        *,
        cancel_requested=None,
        action_permit=None,
    ):
        if cancel_requested is not None and cancel_requested():
            raise BlastControllerError(
                "controller_command_interrupted",
                "BLAST simulation was cancelled",
                motion_started=False,
            )
        if action_permit is None:
            raise BlastControllerError(
                "scan_start_clearance_unverified",
                "BLAST simulation scan requires an anchor permit",
                motion_started=False,
            )
        self.commands.append("scan_surroundings")
        self.scan_count += 1
        return self._scan_result(surroundings=True)

    def _recorded_startup_scan_result(self):
        """Replay measured startup evidence, not a route or later world truth."""

        scan = deepcopy(self._startup_scan)
        scan.pop("planar_projection", None)
        original = scan["encoder_start_angles_deg"]
        current = self._observation()["motor_angles_deg"]
        final = {
            role: current[role] + scan["encoder_final_angles_deg"][role] - original[role]
            for role in original
        }
        scan["encoder_start_angles_deg"] = {role: current[role] for role in original}
        scan["encoder_final_angles_deg"] = final
        self._left_encoder_deg = final["left_drive"]
        self._right_encoder_deg = final["right_drive"]
        imu = scan["imu_heading_diagnostics"]
        self.simulation.rotate(self.world_robot_id, normalize_heading_mdeg(round(
            -(imu["final_heading_deg"] - imu["start_heading_deg"]) * 1_000
        )))
        self._observed_ms += 40_000
        return {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID, "controller_id": CONTROLLER_ID,
            "command": SCAN_COMMAND, "accepted": True, "completed": True,
            "receipt": {"turn_count": scan["sweep_turn_count"], "coverage_complete": True},
            "observation": self._observation(), "observation_settled": True,
            "scan": scan,
        }

    def command(
        self,
        command: str,
        *,
        cancel_requested=None,
        action_permit=None,
    ):
        if cancel_requested is not None and cancel_requested():
            raise BlastControllerError(
                "controller_command_interrupted",
                "BLAST simulation was cancelled",
                motion_started=False,
            )
        self.commands.append(command)
        if command == "drive_forward" and not self._drop_injected:
            self._drop_injected = True
            self._drop_remaining = self._range_dropout_reads
        if command == SETTLED_OBSERVATION_COMMAND:
            self._observed_ms += 1
            observation = self._observation()
            receipt = {"sample_count": 1}
        elif command == SCAN_COMMAND:
            if action_permit is None:
                raise BlastControllerError(
                    "scan_start_clearance_unverified",
                    "BLAST simulation scan requires an anchor permit",
                    motion_started=False,
                )
            self.scan_count += 1
            return self._scan_result(surroundings=False)
        else:
            before = self._observation()["motor_angles_deg"]
            if command == "drive_forward":
                direction, left, right = (
                    "forward",
                    DRIVE_ENCODER_DEGREES,
                    DRIVE_ENCODER_DEGREES,
                )
            elif command == "drive_reverse":
                direction, left, right = (
                    "reverse",
                    -DRIVE_ENCODER_DEGREES,
                    -DRIVE_ENCODER_DEGREES,
                )
            elif command in ("turn_left", "turn_right", "turn_left_trim", "turn_right_trim"):
                is_left = command.startswith("turn_left")
                turn_degrees = TURN_TRIM_ENCODER_DEGREES if command.endswith("_trim") else TURN_ENCODER_DEGREES_PER_PULSE
                direction = "left" if is_left else "right"
                left = (
                    -turn_degrees if is_left else turn_degrees
                )
                right = -left
            else:
                raise BlastControllerError(
                    "unsupported_controller_command",
                    "BLAST simulation command is unsupported",
                    motion_started=False,
                )
            observation = self._apply_encoder_motion(
                left_delta_deg=left,
                right_delta_deg=right,
            )
            turning = command.startswith("turn_")
            receipt = {
                "accepted": True,
                "direction": direction,
                "speed_dps": TURN_SPEED_DPS if turning else DRIVE_SPEED_DPS,
                "wheel_angle_deg" if turning else "angle_deg": (
                    turn_degrees if turning else DRIVE_ENCODER_DEGREES
                ),
                "before_angles_deg": {
                    role: before[role]
                    for role in ("left_drive", "right_drive")
                },
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


class SharedWorldEV3Transport:
    """EV3 worker transport surface backed by the same simulated world."""

    # Mirrored from the deployed ev3/navigation_profile.py; tested for parity.
    PROCESS_LIMIT_MS, REQUEST_LIMIT = 180_000, 256
    PULSE_LIMIT, MOTION_LIMIT_MS = 40, 32_000

    def __init__(
        self,
        simulation: MultiRobotNavigationSimulator,
        *,
        world_robot_id: str,
        odometry: OdometryCalibration,
        ir_response=((0, 0), (200, 44), (300, 51), (1_000, 100)),
        clock=time.monotonic,
    ) -> None:
        self.simulation = simulation
        self.world_robot_id = world_robot_id
        self.odometry = odometry
        # Reuse the deployed gate: world distance must never bypass IR sensing.
        from ev3.infrared_safety import InfraredGatePolicy, InfraredObstacleGate
        config = json.loads((Path(__file__).resolve().parents[2] / "config/ev3rstorm.json").read_text())
        self._ir_gate = InfraredObstacleGate(InfraredGatePolicy.from_config(config))
        self.ir_response = tuple(ir_response)
        if (len(self.ir_response) < 2
                or any(not 0 <= raw <= 100 or distance < 0 for distance, raw in self.ir_response)
                or any(a[0] >= b[0] for a, b in zip(self.ir_response, self.ir_response[1:]))):
            raise ValueError("IR response needs increasing distances and readings in 0..100")
        self.shutdown_complete = False
        self._started = False
        self._aborted = False
        self._version = 1
        # Encoder counters belong to the simulated robot, not to one worker
        # process. A worker renewal must observe the same absolute counters as
        # the worker it replaces, just like the physical EV3.
        encoder_states = getattr(simulation, "_ev3_encoder_states", None)
        if encoder_states is None:
            encoder_states = {}
            setattr(simulation, "_ev3_encoder_states", encoder_states)
        state = encoder_states.setdefault(
            world_robot_id,
            {"left": 0, "right": 0},
        )
        self._encoder_state = state
        self._left = state["left"]
        self._right = state["right"]
        self._now_ms = time.monotonic_ns() // 1_000_000
        self._last_outcome = {"kind": "observe", "status": "completed"}
        self._lock = threading.RLock()
        self._clock = clock
        self._started_clock = clock()
        self._started_ms = self._now_ms
        self._pulse_count = self._motion_ms = self._request_count = 0

    def _process_remaining_ms(self):
        # Motor time is accelerated; model/host waiting still consumes real time.
        return max(0, self.PROCESS_LIMIT_MS - (self._now_ms - self._started_ms)
                   - round((self._clock() - self._started_clock) * 1_000))

    def clock_ms(self) -> int:
        with self._lock:
            return self._now_ms

    def start(self) -> None:
        with self._lock:
            if self._started:
                raise RuntimeError("simulation transport already started")
            self._started = True
            for _ in range(5):
                self._observation()

    def abort(self) -> None:
        with self._lock:
            self._aborted = True

    def close(self) -> None:
        return None

    def _ir_value(self, clearance_mm: int) -> int:
        # Interpolation is a sensor hypothesis, not a metric fact given to Qwen.
        for (d0, r0), (d1, r1) in zip(self.ir_response, self.ir_response[1:]):
            if clearance_mm <= d1:
                fraction = max(0, (clearance_mm - d0) / (d1 - d0))
                return round(r0 + fraction * (r1 - r0))
        return self.ir_response[-1][1]

    def _clearance_mm(self) -> int:
        return self.simulation.scan(self.world_robot_id, (0,))[0][1]

    def _observation(self) -> Mapping[str, object]:
        clearance = self._clearance_mm()
        infrared = self._ir_gate.observe(self._ir_value(clearance))
        return {
            "state_version": self._version,
            "observed_monotonic_ms": self._now_ms,
            "touch": {"value0": 0, "pressed": False},
            "infrared": infrared,
            "motors": [
                {"role": "drive_b", "position": self._left, "state": ""},
                {"role": "drive_c", "position": self._right, "state": ""},
            ],
            "last_outcome": deepcopy(self._last_outcome),
            "budgets": {
                "pulse_count": self._pulse_count,
                "pulse_count_remaining": self.PULSE_LIMIT - self._pulse_count,
                "pulse_duration_ms": self._motion_ms,
                "pulse_duration_ms_remaining": self.MOTION_LIMIT_MS - self._motion_ms,
                "process_ms_remaining": self._process_remaining_ms(),
                "motion_fault_latched": False,
            },
        }

    def _advance_state(self, elapsed_ms: int = 10) -> None:
        self._version += 1
        self._now_ms += elapsed_ms

    def _describe(self) -> Mapping[str, object]:
        return {
            "state_version": self._version,
            "result": {
                "worker_id": "multi-robot-navigation-simulator",
                "demo_only": True,
                "policy_owner": "host",
                "controller_id": self.world_robot_id,
                "request_schema": "ev3-agent-worker-request/v1",
                "response_schema": "ev3-agent-worker-response/v2",
                "operations": sorted(EXPECTED_WORKER_OPERATIONS),
                "pulse": {
                    "actions": deepcopy(EXPECTED_ACTION_SPECS),
                    "max_pulses": self.PULSE_LIMIT,
                    "max_total_duration_ms": self.MOTION_LIMIT_MS,
                },
                "scan_turn": expected_scan_turn_profile(),
                "scan_sample": expected_scan_sample_profile(),
                "safety": deepcopy(EXPECTED_WORKER_SAFETY),
                "process": {
                    "absolute_max_ms": self.PROCESS_LIMIT_MS,
                    "max_requests": self.REQUEST_LIMIT,
                },
                "drive_geometry": {
                    "left_motor_role": "drive_b",
                    "right_motor_role": "drive_c",
                    "forward_speed_sign": {"drive_b": 1, "drive_c": 1},
                },
                "observation": self._observation(),
            },
        }

    @staticmethod
    def _split_delta(total: int, count: int) -> list[int]:
        sign = 1 if total >= 0 else -1
        magnitude, remainder = divmod(abs(total), count)
        return [
            sign * (magnitude + (1 if index < remainder else 0))
            for index in range(count)
        ]

    def _pulse(self, action: str, relative_delta_mdeg=None) -> Mapping[str, object]:
        if action not in EXPECTED_ACTION_SPECS and action != SCAN_TURN_OPERATION:
            raise ValueError("simulation EV3 action is invalid")
        spec = (
            expected_scan_turn_spec(relative_delta_mdeg)
            if action == SCAN_TURN_OPERATION else EXPECTED_ACTION_SPECS[action]
        )
        if (self._pulse_count + spec["slice_count"] > self.PULSE_LIMIT
                or self._motion_ms + spec["total_duration_ms"] > self.MOTION_LIMIT_MS):
            raise EV3NavigationRemoteError("pulse_budget_exhausted", "Worker motion budget exhausted", True,
                                           observation=self._observation(), stop=_stop_proof())
        if action == ADVANCE and self._observation()["infrared"]["blocked"]:
            # Mirror the real worker's preflight denial, not a fictitious move.
            self._advance_state()
            self._last_outcome = {
                "kind": "pulse", "action": action, "status": "denied",
                "reason": "infrared_blocked", "started_monotonic_ms": None,
                "completed_monotonic_ms": self._now_ms, "stop_confirmed": True,
                "requested_slice_count": spec["slice_count"], "completed_slice_count": 0,
                "slices": [], "encoder_verification": {
                    "passed": False, "verified_slice_count": 0,
                    "requested_slice_count": spec["slice_count"],
                },
            }
            return {"state_version": self._version, "result": {
                "action": action, "outcome": deepcopy(self._last_outcome),
                "observation": self._observation(), "stop": _stop_proof(),
            }}
        before_left, before_right = self._left, self._right
        target = spec["target_mean_abs_encoder_degrees"]
        left_delta = int(round(
            spec["left_speed_dps"] * spec["total_duration_ms"] / 1_000
        ))
        right_delta = int(round(
            spec["right_speed_dps"] * spec["total_duration_ms"] / 1_000
        ))
        if target is not None:
            left_delta = (1 if left_delta >= 0 else -1) * target
            right_delta = (1 if right_delta >= 0 else -1) * target

        blocked = False
        blocked_reason = "simulation_body_blocked"
        if action in (ADVANCE, REVERSE):
            requested_mm = int(round(
                (left_delta + right_delta)
                / 2.0
                * self.odometry.linear_mm_per_encoder_degree
            ))
            # Sample at the worker's polling interval, including its actual
            # filtering/hysteresis. Geometry only supplies sensor data/contact.
            from ev3.navigation_profile import POLL_INTERVAL_MS
            polls = math.ceil(spec["total_duration_ms"] / POLL_INTERVAL_MS)
            moved_mm = 0
            for index in range(1, polls + 1):
                if action == ADVANCE and self._observation()["infrared"]["blocked"]:
                    blocked_reason = "infrared_blocked"
                    break
                step = round(requested_mm * index / polls) - moved_mm
                actual = self.simulation.move(self.world_robot_id, step)
                moved_mm += actual
                if actual != step:
                    break
            if moved_mm != requested_mm:
                blocked = True
                ratio = 0.0 if requested_mm == 0 else moved_mm / requested_mm
                left_delta = int(round(left_delta * ratio))
                right_delta = int(round(right_delta * ratio))
        else:
            turn_mdeg = int(round(
                (right_delta - left_delta)
                / 2.0
                * self.odometry.turn_mdeg_per_opposed_encoder_degree
            ))
            from ev3.navigation_profile import POLL_INTERVAL_MS
            polls = math.ceil(spec["total_duration_ms"] / POLL_INTERVAL_MS)
            actual_turn = 0
            for index in range(1, polls + 1):
                step = round(turn_mdeg * index / polls) - actual_turn
                before_heading = self.simulation.pose(self.world_robot_id).heading_mdeg
                after = self.simulation.rotate(self.world_robot_id, step)
                measured = normalize_heading_mdeg(after.heading_mdeg - before_heading)
                actual_turn += measured
                self._observation()
                if measured != step:
                    break
            if actual_turn != turn_mdeg:
                blocked = True
                ratio = actual_turn / turn_mdeg if turn_mdeg else 0.0
                left_delta = round(left_delta * ratio)
                right_delta = round(right_delta * ratio)

        self._left += left_delta
        self._right += right_delta
        self._encoder_state["left"] = self._left
        self._encoder_state["right"] = self._right
        count = spec["slice_count"]
        # A blocked action has only an executed prefix, never fabricated
        # completed zero-motion slices preceding its interrupted slice.
        executed_count = max(1, min(count, math.ceil(count * ratio))) if blocked else count
        left_parts = self._split_delta(left_delta, executed_count)
        right_parts = self._split_delta(right_delta, executed_count)
        slices = []
        running_left, running_right = before_left, before_right
        for index, (duration, left_part, right_part) in enumerate(zip(
            spec["slice_durations_ms"],
            left_parts,
            right_parts,
        ), start=1):
            next_left = running_left + left_part
            next_right = running_right + right_part
            receipt_stop = _stop_proof()
            slices.append({
                "slice_index": index,
                "slice_count": count,
                "duration_ms": duration,
                "status": "completed",
                "reason": "duration_elapsed",
                "started_monotonic_ms": self._now_ms,
                "completed_monotonic_ms": self._now_ms + duration,
                "motors": [
                    {
                        "side": "left",
                        "role": "drive_b",
                        "position_before": running_left,
                        "position_after": next_left,
                        "position_delta": left_part,
                        "state": "",
                    },
                    {
                        "side": "right",
                        "role": "drive_c",
                        "position_before": running_right,
                        "position_after": next_right,
                        "position_delta": right_part,
                        "state": "",
                    },
                ],
                "encoder_verification": {
                    "passed": True,
                    "error": None,
                    "checks": [{"passed": True, "source": "simulated_encoders"}],
                },
                "stop": receipt_stop,
            })
            if action != SCAN_TURN_OPERATION:
                # Drive receipts carry temporal segments on the real EV3 worker.
                segment = {key: deepcopy(slices[-1][key]) for key in (
                    "duration_ms", "status", "reason", "started_monotonic_ms",
                    "completed_monotonic_ms", "motors", "encoder_verification", "stop",
                )}
                segment.update(segment_index=1, kind="paired", commanded_sides=["left", "right"])
                slices[-1]["segments"] = [segment]
            running_left, running_right = next_left, next_right
            self._now_ms += duration
            self._pulse_count += 1
            self._motion_ms += duration
        status = "interrupted" if blocked else "completed"
        completed_count = 0 if blocked else count
        if blocked:
            slices[-1].update(status="interrupted", reason=blocked_reason)
            if action != SCAN_TURN_OPERATION:
                slices[-1]["segments"][-1].update(status="interrupted", reason=blocked_reason)
            completed_count = executed_count - 1
        outcome = {
            "kind": "pulse",
            "action": action,
            "status": status,
            "reason": (
                blocked_reason
                if blocked else "semantic_action_completed"
            ),
            "started_monotonic_ms": slices[0]["started_monotonic_ms"],
            "completed_monotonic_ms": slices[-1]["completed_monotonic_ms"],
            "stop_confirmed": True,
            "requested_slice_count": count,
            "completed_slice_count": completed_count,
            "slices": slices,
            "encoder_verification": {
                "passed": executed_count == count and (not blocked or action != SCAN_TURN_OPERATION),
                "verified_slice_count": executed_count,
                "requested_slice_count": count,
            },
        }
        self._advance_state()
        self._last_outcome = outcome
        if action == SCAN_TURN_OPERATION:
            outcome.pop("action")
            outcome.update({
                "kind": SCAN_TURN_OPERATION,
                "reason": "simulation_body_blocked" if blocked else "scan_turn_completed",
                "requested_relative_delta_mdeg": relative_delta_mdeg,
                "profile_id": spec["profile_id"],
                "calibration": spec["calibration"],
            })
            outcome["encoder_verification"].update({
                "left_delta_degrees": left_delta,
                "right_delta_degrees": right_delta,
                "mean_abs_encoder_degrees": int(round((abs(left_delta) + abs(right_delta)) / 2)),
                "target_mean_abs_encoder_degrees": target,
                "max_side_divergence_degrees": expected_scan_turn_profile()["max_side_divergence_degrees"],
            })
        result = {
            ("relative_delta_mdeg" if action == SCAN_TURN_OPERATION else "action"): (
                relative_delta_mdeg if action == SCAN_TURN_OPERATION else action
            ),
            "outcome": deepcopy(outcome),
            "observation": self._observation(),
            "stop": _stop_proof(),
        }
        return {"state_version": self._version, "result": result}

    def request(
        self,
        operation,
        arguments,
        timeout,
        cancel_requested=None,
    ):
        del timeout
        with self._lock:
            if self._aborted:
                raise RuntimeError("simulation transport was aborted")
            if self.shutdown_complete or self._process_remaining_ms() == 0:
                raise EV3NavigationTransportError("simulation worker process is closed or expired")
            self._request_count += 1
            if self._request_count > self.REQUEST_LIMIT:
                raise EV3NavigationRemoteError("request_budget_exhausted", "Worker request budget exhausted", True,
                                               observation=self._observation(), stop=_stop_proof())
            if cancel_requested is not None and cancel_requested():
                raise RuntimeError("simulation request was cancelled")
            if operation == "describe":
                return self._describe()
            if operation == "observe":
                self._advance_state()
                self._last_outcome = {"kind": "observe", "status": "completed"}
                return {
                    "state_version": self._version,
                    "result": {"observation": self._observation()},
                }
            if operation == "pulse":
                return self._pulse(arguments["action"])
            if operation == SCAN_TURN_OPERATION:
                return self._pulse(operation, arguments["relative_delta_mdeg"])
            if operation == "scan_sample":
                profile = expected_scan_sample_profile()
                self._ir_gate = type(self._ir_gate)(self._ir_gate.policy)
                started = self._now_ms
                self._advance_state(profile["settled_duration_ms"])
                raw_samples = []
                for _ in range(profile["sample_count"]):
                    obs = self._observation()
                    raw_samples.append(obs["infrared"]["raw"])
                return {"state_version": self._version, "result": {
                    "sample_count": profile["sample_count"],
                    "raw_samples": raw_samples,
                    "started_monotonic_ms": started,
                    "completed_monotonic_ms": self._now_ms,
                    "observation": obs,
                    "stop": _stop_proof(),
                }}
            if operation == "shutdown":
                self._advance_state()
                self.shutdown_complete = True
                return {
                    "state_version": self._version,
                    "result": {"outcome": {
                        "kind": "shutdown",
                        "status": "completed",
                        "completed_monotonic_ms": self._now_ms,
                        "stop_confirmed": True,
                        "motor_owner_closed": True,
                    }},
                }
            raise ValueError("unsupported simulated EV3 operation")

__all__ = (
    "SharedWorldBlastController",
    "SharedWorldEV3Transport",
)
