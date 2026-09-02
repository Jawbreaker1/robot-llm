"""Hardware-surface adapters for the shared multi-robot simulation world.

These classes translate existing BLAST and EV3 commands into simulated
motion and sensor readings.  They contain no route, waypoint or side-choice
logic; the production episode runtimes and their model planners retain that
responsibility.
"""

from __future__ import annotations

from copy import deepcopy
import math
import threading
import time
from typing import Mapping

from .active_ir_scan import ActiveIrScanExecutor
from .blast_navigation_action_profile import (
    DRIVE_ENCODER_DEGREES,
    DRIVE_SPEED_DPS,
    SCAN_TRIM_ENCODER_DEGREES_PER_PULSE,
    TURN_DURATION_MS_PER_PULSE,
    TURN_ENCODER_DEGREES_PER_PULSE,
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
    TURN_LEFT_90,
    TURN_RIGHT_90,
    expected_scan_sample_profile,
    expected_scan_turn_profile,
)
from .physical_odometry import OdometryCalibration, normalize_heading_mdeg


def _stop_proof() -> Mapping[str, object]:
    return {
        "stop_confirmed": True,
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
    ) -> None:
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

    def monotonic_ms(self) -> int:
        return self._observed_ms

    def runtime_generation(self) -> int:
        return self._generation

    def _distance_mm(self) -> int:
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
            self.simulation.rotate(self.world_robot_id, turn_mdeg)
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
        center = self._observation()
        start = {
            role: center["motor_angles_deg"][role]
            for role in ("left_drive", "right_drive")
        }
        if surroundings:
            samples = [self._turn_scan_pulse("left") for _ in range(16)]
            trim_sample = self._turn_scan_pulse(
                "left",
                SCAN_TRIM_ENCODER_DEGREES_PER_PULSE,
            )
            final = trim_sample[1]
            scan = build_blast_encoder_scan(
                center=center,
                center_settled=True,
                start_drive_angles=start,
                sweep_samples=samples,
                final=final,
                final_settled=True,
                final_body_verified=True,
                sweep_turn_count=len(samples) + 1,
            )
            receipt = {
                "turn_count": len(samples) + 1,
                "coverage_complete": (
                    scan["sweep_coverage_deg"] >= 360.0
                ),
            }
        else:
            left_outbound = [
                self._turn_scan_pulse("left") for _ in range(4)
            ]
            for _ in range(4):
                self._turn_scan_pulse("right")
            right_outbound = [
                self._turn_scan_pulse("right") for _ in range(4)
            ]
            for _ in range(3):
                self._turn_scan_pulse("left")
            final_sample = self._turn_scan_pulse("left")
            final = final_sample[1]
            scan = build_blast_front_arc_scan(
                center=center,
                center_settled=True,
                start_drive_angles=start,
                left_outbound=left_outbound,
                right_outbound=right_outbound,
                final=final,
                final_settled=True,
                final_body_verified=True,
            )
            receipt = {"turn_count": 16, "coverage": "front_arc"}
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
            elif command in ("turn_left", "turn_right"):
                is_left = command == "turn_left"
                direction = "left" if is_left else "right"
                left = (
                    -TURN_ENCODER_DEGREES_PER_PULSE
                    if is_left else TURN_ENCODER_DEGREES_PER_PULSE
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
                    TURN_ENCODER_DEGREES_PER_PULSE
                    if turning else DRIVE_ENCODER_DEGREES
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

    def __init__(
        self,
        simulation: MultiRobotNavigationSimulator,
        *,
        world_robot_id: str,
        odometry: OdometryCalibration,
        obstacle_threshold_mm: int = 210,
    ) -> None:
        self.simulation = simulation
        self.world_robot_id = world_robot_id
        self.odometry = odometry
        self.obstacle_threshold_mm = obstacle_threshold_mm
        self.shutdown_complete = False
        self._started = False
        self._aborted = False
        self._version = 1
        self._left = 0
        self._right = 0
        self._now_ms = time.monotonic_ns() // 1_000_000
        self._last_outcome = {"kind": "observe", "status": "completed"}
        self._lock = threading.RLock()

    def clock_ms(self) -> int:
        with self._lock:
            return self._now_ms

    def start(self) -> None:
        with self._lock:
            if self._started:
                raise RuntimeError("simulation transport already started")
            self._started = True

    def abort(self) -> None:
        with self._lock:
            self._aborted = True

    def close(self) -> None:
        return None

    @staticmethod
    def _ir_value(clearance_mm: int, threshold_mm: int) -> int:
        return 20 if clearance_mm <= threshold_mm else 60

    def _clearance_mm(self) -> int:
        return self.simulation.scan(self.world_robot_id, (0,))[0][1]

    def _observation(self) -> Mapping[str, object]:
        clearance = self._clearance_mm()
        blocked = clearance <= self.obstacle_threshold_mm
        raw = self._ir_value(clearance, self.obstacle_threshold_mm)
        return {
            "state_version": self._version,
            "observed_monotonic_ms": self._now_ms,
            "touch": {"value0": 0, "pressed": False},
            "infrared": {
                "raw": raw,
                "filtered": raw,
                "blocked": blocked,
                "reason": (
                    "blocked_hysteresis_hold"
                    if blocked else "clear_hysteresis_hold"
                ),
                "sample_count": 5,
            },
            "motors": [
                {"role": "drive_b", "position": self._left, "state": ""},
                {"role": "drive_c", "position": self._right, "state": ""},
            ],
            "last_outcome": deepcopy(self._last_outcome),
            "budgets": {
                "pulse_count": 0,
                "pulse_count_remaining": 1_000,
                "pulse_duration_ms": 0,
                "pulse_duration_ms_remaining": 1_000_000,
                "process_ms_remaining": 180_000,
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
                    "max_pulses": 1_000,
                    "max_total_duration_ms": 1_000_000,
                },
                "scan_turn": expected_scan_turn_profile(),
                "scan_sample": expected_scan_sample_profile(),
                "safety": deepcopy(EXPECTED_WORKER_SAFETY),
                "process": {
                    "absolute_max_ms": 180_000,
                    "max_requests": 10_000,
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

    def _pulse(self, action: str) -> Mapping[str, object]:
        if action not in EXPECTED_ACTION_SPECS:
            raise ValueError("simulation EV3 action is invalid")
        spec = EXPECTED_ACTION_SPECS[action]
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
        if action in (ADVANCE, REVERSE):
            requested_mm = int(round(
                (left_delta + right_delta)
                / 2.0
                * self.odometry.linear_mm_per_encoder_degree
            ))
            moved_mm = self.simulation.move(
                self.world_robot_id,
                requested_mm,
            )
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
            self.simulation.rotate(self.world_robot_id, turn_mdeg)

        self._left += left_delta
        self._right += right_delta
        count = spec["slice_count"]
        left_parts = self._split_delta(left_delta, count)
        right_parts = self._split_delta(right_delta, count)
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
                    "checks": [],
                },
                "stop": receipt_stop,
            })
            running_left, running_right = next_left, next_right
            self._now_ms += duration
        status = "interrupted" if blocked else "completed"
        completed_count = 0 if blocked else count
        outcome = {
            "kind": "pulse",
            "action": action,
            "status": status,
            "reason": (
                "simulation_body_blocked"
                if blocked else "semantic_action_completed"
            ),
            "started_monotonic_ms": slices[0]["started_monotonic_ms"],
            "completed_monotonic_ms": slices[-1]["completed_monotonic_ms"],
            "stop_confirmed": True,
            "requested_slice_count": count,
            "completed_slice_count": completed_count,
            "slices": slices,
            "encoder_verification": {
                "passed": not blocked,
                "verified_slice_count": count,
                "requested_slice_count": count,
            },
        }
        self._advance_state()
        self._last_outcome = outcome
        result = {
            "action": action,
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

    def build_scan_executor(self) -> ActiveIrScanExecutor:
        return ActiveIrScanExecutor(
            rig=_SharedWorldEV3ScanRig(self),
            clock_ms=self.clock_ms,
        )


class _SharedWorldEV3ScanRig:
    def __init__(self, transport: SharedWorldEV3Transport) -> None:
        self.transport = transport
        self._request = None
        self._local_heading_mdeg = 0

    def begin_scan(self, request) -> None:
        self._request = request
        self._local_heading_mdeg = request.start_pose.heading_mdeg

    def turn_relative_mdeg(self, delta, _calibration, _deadline_ms):
        self.transport.simulation.rotate(
            self.transport.world_robot_id,
            delta,
        )
        self._local_heading_mdeg = normalize_heading_mdeg(
            self._local_heading_mdeg + delta
        )
        self.transport._advance_state(max(1, abs(delta) // 100))
        return {
            "requested_delta_mdeg": delta,
            "actual_delta_mdeg": delta,
            "completed_at_ms": self.transport.clock_ms(),
            "stop_confirmed": True,
        }

    def read_snapshot(self, _deadline_ms):
        self.transport._advance_state()
        clearance = self.transport._clearance_mm()
        blocked = clearance <= self.transport.obstacle_threshold_mm
        raw = self.transport._ir_value(
            clearance,
            self.transport.obstacle_threshold_mm,
        )
        request = self._request
        observed_at_ms = self.transport.clock_ms()
        if request is not None:
            observed_at_ms = request.created_at_ms + max(
                0,
                self.transport.clock_ms() - request.created_monotonic_ms,
            )
        return {
            "state_version": self.transport._version,
            "observed_at_ms": observed_at_ms,
            "pose_heading_mdeg": self._local_heading_mdeg,
            "touch_pressed": False,
            "motion_fault_latched": False,
            "infrared": {
                "raw": raw,
                "filtered": raw,
                "blocked": blocked,
            },
        }

    def stop(self):
        return {"stop_confirmed": True}


__all__ = (
    "SharedWorldBlastController",
    "SharedWorldEV3Transport",
)
