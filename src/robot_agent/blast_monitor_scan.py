"""Front and surroundings scans using the monitor's existing motor/session owner."""

from __future__ import annotations

import logging
import time
from typing import Mapping
from .blast_navigation_calibration import (
    BLAST_ENCODER_SETTLING_DEGREES,
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_navigation_action_profile import (
    SCAN_TURN_ENCODER_DEGREES_PER_PULSE,
    SCAN_TRIM_ENCODER_DEGREES_PER_PULSE,
    TURN_SPEED_DPS,
)
from .blast_scan_observation import (
    RANGE_STATE_INVALID,
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    SCAN_RAY_EVIDENCE_SETTLED,
    SCAN_RAY_EVIDENCE_SWEEP_ONLY,
    blast_range_state,
    body_motor_angle as _body_motor_angle,
    build_blast_encoder_scan,
    build_blast_front_arc_scan,
    build_blast_partial_scan,
    scan_sweep_bearing_deg,
    scan_heading,
    surroundings_scan_next_turn,
    drive_encoder_angles,
    encoder_relative_bearing_deg,
)
from .blast_controller_contract import (
    ROBOT_ID,
    CONTROLLER_ID,
    SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
    SCAN_PULSE_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
    COMMAND_RESULT_SCHEMA,
    SCAN_COMMAND,
    _BlastNoReturnScanPermit,
    BlastControllerError,
)


logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())


async def _recover_scan_turn(monitor, runtime, generation, error, sensor):
    """Stop and retain one moved scan pulse when its range is uncertain."""

    close_obstacle = error.code == "scan_sweep_clearance_lost"
    if not error.evidence_uncertain and not close_obstacle:
        raise error
    await runtime.stop()
    final = await monitor._observe_until_idle(
        runtime, generation=generation, stop_only=True,
    )
    final, final_settled = await monitor._observe_until_settled(
        runtime, generation=generation,
        initial_observation=final,
        timeout_seconds=SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
    )
    if (
        final.get("motion_active") is not False
        or drive_encoder_angles(final) is None
        or not sensor.matches_navigation_body_angle(
            _body_motor_angle(final)
        )
    ):
        raise error
    evidence = (
        SCAN_RAY_EVIDENCE_SETTLED
        if final_settled is True
        else SCAN_RAY_EVIDENCE_SWEEP_ONLY
    )
    sample = (
        {"stopped_after_uncertain_evidence": True},
        final,
        final_settled,
        evidence,
    )
    can_continue = (
        error.code != "controller_command_failed"
        and not close_obstacle
        and monitor._scan_sweep_window_allows_continuation(final)
    )
    return sample, can_continue


async def _scan_turn(
    monitor,
    runtime,
    generation,
    direction,
    *,
    start_drive_angles,
    trim=False,
):
    if await monitor._service_preempt_stop(runtime, generation):
        raise BlastControllerError(
            "controller_command_interrupted",
            "BLAST scan was interrupted by stop",
            motion_started=False,
        )
    expected_wheel_angle = (
        SCAN_TRIM_ENCODER_DEGREES_PER_PULSE
        if trim else SCAN_TURN_ENCODER_DEGREES_PER_PULSE
    )
    try:
        receipt = await (
            runtime.scan_trim_pulse(direction)
            if trim else runtime.scan_turn_pulse(direction)
        )
        observation = await monitor._observe_until_idle(
            runtime,
            generation=generation,
            stop_only=False,
        )
        observation, observation_settled = (
            await monitor._observe_until_settled(
                runtime, generation=generation,
                initial_observation=observation,
                timeout_seconds=(
                    SCAN_PULSE_POST_MOTION_SETTLE_TIMEOUT_SECONDS
                ),
            )
        )
    except BlastControllerError as error:
        if error.motion_started is None:
            error.motion_started = True
        raise
    except Exception as error:
        # A pulse or its following read can fail after physical movement.
        # Both use the existing stop/read/partial-scan recovery.
        logger.warning(
            "BLAST scan pulse/read failed direction=%s trim=%s error_type=%s",
            direction, trim, type(error).__name__,
        )
        raise BlastControllerError(
            "controller_command_failed",
            "BLAST scan pulse or observation transport failed",
            evidence_uncertain=True,
        ) from error
    distance = observation.get("distance_mm")
    sweep_only = observation_settled is not True
    sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .range_sensor_extrinsics
    )
    current_drive_angles = drive_encoder_angles(observation)
    before_angles = (
        receipt.get("before_angles_deg")
        if isinstance(receipt, Mapping) else None
    )
    receipt_fields = {
        "accepted", "direction", "speed_dps", "wheel_angle_deg",
        "before_angles_deg",
    }
    receipt_profile_valid = (
        isinstance(receipt, Mapping)
        and set(receipt) == receipt_fields
        and receipt.get("accepted") is True
        and receipt.get("direction") == direction
        and receipt.get("speed_dps") == TURN_SPEED_DPS
        and receipt.get("wheel_angle_deg")
        == expected_wheel_angle
        and isinstance(before_angles, Mapping)
        and set(before_angles) == {"left_drive", "right_drive"}
        and all(type(before_angles.get(role)) is int for role in (
            "left_drive", "right_drive",
        ))
        and current_drive_angles is not None
    )
    if receipt_profile_valid:
        pulse_delta = {
            role: current_drive_angles[role] - before_angles[role]
            for role in ("left_drive", "right_drive")
        }
        expected_signs = (
            {"left_drive": -1, "right_drive": 1}
            if direction == "left"
            else {"left_drive": 1, "right_drive": -1}
        )
        receipt_profile_valid = all(
            pulse_delta[role] * expected_signs[role] > 0
            and abs(pulse_delta[role])
            <= 4 * expected_wheel_angle
            for role in ("left_drive", "right_drive")
        )
    if (
        not receipt_profile_valid
        or encoder_relative_bearing_deg(
            observation, start_drive_angles,
        ) is None
        or not sensor.matches_navigation_body_angle(
            _body_motor_angle(observation)
        )
    ):
        raise BlastControllerError(
            "scan_sweep_observation_unverified",
            "BLAST scan lost correlated encoder or sensor-pose evidence",
            motion_started=True,
        )
    range_state = blast_range_state(distance)
    if range_state == RANGE_STATE_INVALID:
        raise BlastControllerError(
            "scan_sweep_observation_unverified",
            "BLAST scan received invalid settled range evidence",
            motion_started=True,
            evidence_uncertain=True,
        )
    if observation.get("motion_active") is not False:
        raise BlastControllerError(
            "scan_sweep_observation_unverified",
            "BLAST scan pulse did not reach an idle observation",
            motion_started=True, evidence_uncertain=True,
        )
    if range_state == RANGE_STATE_MEASURED and float(distance) <= (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .minimum_rotation_clearance_mm()
    ):
        raise BlastControllerError(
            "scan_sweep_clearance_lost",
            "BLAST scan stopped after a close settled observation",
            motion_started=True,
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
    monitor, runtime, generation, *, action_permit=None,
):
    center = await monitor._observe_until_idle(
        runtime,
        generation=generation,
        stop_only=False,
    )
    center, center_settled = await monitor._observe_until_settled(
        runtime,
        generation=generation,
        initial_observation=center,
        timeout_seconds=SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
    )
    sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .range_sensor_extrinsics
    )
    center_distance = center.get("distance_mm")
    center_motors = center.get("motor_angles_deg")
    center_drive = tuple(center_motors.get(role)
                         if isinstance(center_motors, Mapping) else None
                         for role in ("left_drive", "right_drive"))
    start_drive_angles = drive_encoder_angles(center)
    permit_anchor_matched = (
        isinstance(action_permit, _BlastNoReturnScanPermit)
        and action_permit.runtime_generation == generation
        and time.monotonic_ns()
        <= action_permit.expires_at_monotonic_ns
        and all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and abs(float(value) - expected)
            <= BLAST_ENCODER_SETTLING_DEGREES
            for value, expected in zip(
                center_drive, action_permit.drive_angles_deg
            )
        )
    )
    permit_allows_no_return = (
        permit_anchor_matched and action_permit.allow_no_return
    )
    center_state = blast_range_state(center_distance)
    range_allows_start = (
        permit_anchor_matched
        and center_state == RANGE_STATE_MEASURED
        and float(center_distance) > (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
            .minimum_rotation_clearance_mm()
        )
    ) or (
        permit_allows_no_return
        and center_state == RANGE_STATE_NO_VALID_DISTANCE
    )
    if not (
        start_drive_angles is not None
        and sensor.matches_navigation_body_angle(
            _body_motor_angle(center)
        )
        and range_allows_start
    ):
        raise BlastControllerError(
            "scan_start_clearance_unverified",
            "BLAST scan lacks usable idle range evidence or sensor pose",
            motion_started=False,
        )

    sweep_samples = []
    partial_reason = None
    directions = (
        *("left",) * 4,
        *("right",) * 8,
        *("left",) * 4,
    )
    for direction in directions:
        try:
            sample = await _scan_turn(monitor, runtime, generation, direction,
                start_drive_angles=start_drive_angles,
            )
        except BlastControllerError as error:
            sample, can_continue = await _recover_scan_turn(monitor, runtime, generation, error, sensor,
            )
            if not can_continue:
                partial_reason = error.code
        sweep_samples.append(sample)
        if partial_reason is not None:
            break
    _receipt, final, final_settled, _evidence = sweep_samples[-1]
    if partial_reason is not None:
        scan = build_blast_partial_scan(
            center=center,
            center_settled=center_settled,
            start_drive_angles=start_drive_angles,
            sweep_samples=sweep_samples,
            final=final,
            final_settled=final_settled,
            final_body_verified=True,
        )
        return {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": SCAN_COMMAND,
            "accepted": True,
            "completed": True,
            "receipt": {
                "turn_count": len(sweep_samples),
                "coverage": "partial_front_arc",
                "coverage_complete": False,
                "reason_code": partial_reason,
            },
            "observation": final,
            "observation_settled": final_settled,
            "scan": scan,
        }
    left_outbound = sweep_samples[:4]
    right_outbound = sweep_samples[8:12]
    try:
        scan = build_blast_front_arc_scan(
            center=center,
            center_settled=center_settled,
            start_drive_angles=start_drive_angles,
            left_outbound=left_outbound,
            right_outbound=right_outbound,
            final=final,
            final_settled=final_settled,
            final_body_verified=sensor.matches_navigation_body_angle(
                _body_motor_angle(final)
            ),
        )
    except ValueError as error:
        raise BlastControllerError(
            "scan_sweep_observation_unverified",
            "BLAST front scan encoder geometry was invalid",
            motion_started=True,
        ) from error
    return {
        "schema": COMMAND_RESULT_SCHEMA,
        "robot_id": ROBOT_ID,
        "controller_id": CONTROLLER_ID,
        "command": SCAN_COMMAND,
        "accepted": True,
        "completed": True,
        "receipt": {"turn_count": 16, "coverage": "front_arc"},
        "observation": final,
        "observation_settled": final_settled,
        "scan": scan,
    }


async def _perform_scan_surroundings(
    monitor, runtime, generation, *, action_permit=None,
):
    center = await monitor._observe_until_idle(
        runtime,
        generation=generation,
        stop_only=False,
    )
    center, center_settled = await monitor._observe_until_settled(
        runtime,
        generation=generation,
        initial_observation=center,
        timeout_seconds=SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS,
    )
    sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .range_sensor_extrinsics
    )
    center_distance = center.get("distance_mm")
    center_motors = center.get("motor_angles_deg")
    center_drive = tuple(center_motors.get(role)
                         if isinstance(center_motors, Mapping) else None
                         for role in ("left_drive", "right_drive"))
    start_drive_angles = drive_encoder_angles(center)
    permit_anchor_matched = (
        isinstance(action_permit, _BlastNoReturnScanPermit)
        and action_permit.runtime_generation == generation
        and time.monotonic_ns()
        <= action_permit.expires_at_monotonic_ns
        and all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and abs(float(value) - expected)
            <= BLAST_ENCODER_SETTLING_DEGREES
            for value, expected in zip(
                center_drive, action_permit.drive_angles_deg
            )
        )
    )
    permit_allows_no_return = (
        permit_anchor_matched and action_permit.allow_no_return
    )
    center_state = blast_range_state(center_distance)
    range_allows_start = (
        permit_anchor_matched
        and center_state == RANGE_STATE_MEASURED
        and float(center_distance) > (
            BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
            .minimum_rotation_clearance_mm()
        )
    ) or (
        permit_allows_no_return
        and center_state == RANGE_STATE_NO_VALID_DISTANCE
    )
    if not (
        start_drive_angles is not None
        and sensor.matches_navigation_body_angle(
            _body_motor_angle(center)
        )
        and range_allows_start
    ):
        raise BlastControllerError(
            "scan_start_clearance_unverified",
            "BLAST scan lacks usable idle range evidence or sensor pose",
            motion_started=False,
        )
    sweep_samples = []
    partial_reason = None
    final = center
    while (next_turn := surroundings_scan_next_turn(
        center, start_drive_angles, final, len(sweep_samples),
    )) is not None:
        try:
            sweep_samples.append(await _scan_turn(monitor, runtime, generation, next_turn[0],
                start_drive_angles=start_drive_angles,
                trim=next_turn[1] == "trim",
            ))
        except BlastControllerError as error:
            sample, can_continue = await _recover_scan_turn(monitor, runtime, generation, error, sensor,
            )
            sweep_samples.append(sample)
            if not can_continue:
                partial_reason = error.code
                break
        final = sweep_samples[-1][1]
    if partial_reason is not None:
        final = sweep_samples[-1][1]
        final_settled = sweep_samples[-1][2]
        scan = build_blast_partial_scan(
            center=center, center_settled=center_settled,
            start_drive_angles=start_drive_angles,
            sweep_samples=sweep_samples,
            final=final, final_settled=final_settled,
            final_body_verified=True,
        )
        return {
            "schema": COMMAND_RESULT_SCHEMA,
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": SCAN_COMMAND,
            "accepted": True,
            "completed": True,
            "receipt": {"turn_count": len(sweep_samples),
                        "coverage_complete": False,
                        "reason_code": partial_reason},
            "observation": final,
            "observation_settled": final_settled,
            "scan": scan,
        }
    final_sample = sweep_samples[-1]
    turn_count = len(sweep_samples)
    coverage = abs(scan_sweep_bearing_deg(
        final_sample[1], start_drive_angles, scan_heading(center)) or 0.0)
    _receipt, final, final_settled, _evidence = final_sample
    final_body_verified = sensor.matches_navigation_body_angle(
        _body_motor_angle(final))
    try:
        scan = build_blast_encoder_scan(
            center=center, center_settled=center_settled,
            start_drive_angles=start_drive_angles,
            sweep_samples=sweep_samples,
            final=final, final_settled=final_settled,
            final_body_verified=final_body_verified,
            sweep_turn_count=turn_count,
        )
    except ValueError as error:
        raise BlastControllerError(
            "scan_sweep_observation_unverified",
            "BLAST scan encoder geometry was invalid",
            motion_started=True,
        ) from error
    return {
        "schema": COMMAND_RESULT_SCHEMA,
        "robot_id": ROBOT_ID,
        "controller_id": CONTROLLER_ID,
        "command": SCAN_COMMAND,
        "accepted": True,
        "completed": True,
        "receipt": {
            "turn_count": turn_count,
            "coverage_complete": True,
        },
        "observation": final,
        "observation_settled": final_settled,
        "scan": scan,
    }
