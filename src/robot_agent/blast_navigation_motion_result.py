"""Translate stopped BLAST receipts into shared physical motion evidence."""

from collections.abc import Mapping, Sequence
from copy import deepcopy

from .blast_navigation_action_profile import (
    BLAST_NAVIGATION_ACTION_SPECS,
    BLAST_NAVIGATION_COMMANDS,
    DRIVE_ENCODER_DEGREES,
    DRIVE_SPEED_DPS,
    TURN_ENCODER_DEGREES_PER_PULSE,
    TURN_SPEED_DPS,
)
from .blast_observation_monitor import (
    COMMAND_RESULT_SCHEMA,
    CONTROLLER_ID,
    ROBOT_ID,
)
from .physical_navigation_contract import PhysicalNavigationContractError


_ROLES = ("left_drive", "right_drive")
_SIDES = ("left", "right")
_CLEAN_STOP = {
    "stop_confirmed": True,
    "errors": [],
    "fault_tokens": {},
    "cleanup_errors": [],
}
_RESULT_FIELDS = frozenset((
    "schema", "robot_id", "controller_id", "command", "accepted",
    "completed", "receipt", "observation", "observation_settled",
))
_MAX_INTER_SLICE_SETTLING_DEGREES = 1


def _fail(code, message):
    raise PhysicalNavigationContractError(code, message)


def _angles(value, *, exact=False):
    if (
        not isinstance(value, Mapping)
        or (exact and set(value) != set(_ROLES))
        or any(role not in value for role in _ROLES)
        or any(type(value[role]) is not int for role in _ROLES)
    ):
        _fail(
            "blast_motion_encoder_evidence_invalid",
            "BLAST drive encoder evidence is invalid",
        )
    return tuple(value[role] for role in _ROLES)


def _command_profile(command):
    if command in ("drive_forward", "drive_reverse"):
        direction = command.removeprefix("drive_")
        signs = (1, 1) if direction == "forward" else (-1, -1)
        return direction, DRIVE_SPEED_DPS, "angle_deg", DRIVE_ENCODER_DEGREES, signs
    direction = command.removeprefix("turn_")
    signs = (-1, 1) if direction == "left" else (1, -1)
    return (
        direction, TURN_SPEED_DPS, "wheel_angle_deg",
        TURN_ENCODER_DEGREES_PER_PULSE, signs,
    )


def _decode_result(command, result):
    if (
        not isinstance(result, Mapping)
        or set(result) != _RESULT_FIELDS
        or result["schema"] != COMMAND_RESULT_SCHEMA
        or result["robot_id"] != ROBOT_ID
        or result["controller_id"] != CONTROLLER_ID
        or result["command"] != command
        or result["accepted"] is not True
        or result["completed"] is not True
        or type(result["observation_settled"]) is not bool
    ):
        _fail(
            "blast_motion_result_contract_mismatch",
            "BLAST motion result is not correlated and complete",
        )
    direction, speed, angle_field, angle, signs = _command_profile(command)
    receipt = result["receipt"]
    if (
        not isinstance(receipt, Mapping)
        or set(receipt) != {
            "accepted", "direction", "speed_dps", angle_field,
            "before_angles_deg",
        }
        or receipt["accepted"] is not True
        or receipt["direction"] != direction
        or receipt["speed_dps"] != speed
        or receipt[angle_field] != angle
    ):
        _fail(
            "blast_motion_receipt_profile_mismatch",
            "BLAST motion receipt does not match its fixed profile",
        )
    before = _angles(receipt["before_angles_deg"], exact=True)
    observation = result["observation"]
    if not isinstance(observation, Mapping) or observation.get(
        "motion_active"
    ) is not False:
        _fail(
            "blast_motion_encoder_evidence_invalid",
            "BLAST motion lacks a stopped final observation",
        )
    after = _angles(observation.get("motor_angles_deg"))
    deltas = tuple(end - start for start, end in zip(before, after))
    # A fourfold cap rejects corrupted data without defining a tight travel
    # tolerance before live encoder variance has been measured.
    if any(abs(delta) > angle * 4 for delta in deltas):
        _fail(
            "blast_motion_encoder_evidence_invalid",
            "BLAST encoder travel is physically implausible",
        )
    checks = tuple(delta != 0 and delta * sign > 0 for delta, sign in zip(
        deltas, signs,
    ))
    return before, after, deltas, checks


def _motors(before, after):
    return [
        {
            "side": side,
            "role": role,
            "position_before": start,
            "position_after": end,
            "position_delta": end - start,
            "state": "",
        }
        for side, role, start, end in zip(_SIDES, _ROLES, before, after)
    ]


def _segment(
    *,
    kind,
    commanded_sides,
    before,
    after,
    checks,
    verified,
    error,
    reason,
):
    return {
        "kind": kind,
        "commanded_sides": list(commanded_sides),
        "status": "completed",
        "reason": reason,
        "motors": _motors(before, after),
        "encoder_verification": {
            "passed": verified,
            "error": error,
            "checks": [
                {"side": side, "passed": passed}
                for side, passed in zip(_SIDES, checks)
            ],
        },
        "stop": deepcopy(_CLEAN_STOP),
    }


def _slice(
    index,
    count,
    duration_ms,
    before,
    command_before,
    after,
    checks,
    settling_checks,
):
    verified = all(checks)
    result = {
        "slice_index": index,
        "slice_count": count,
        "duration_ms": duration_ms,
        "status": "completed",
        "reason": "angle_command_completed",
        "motors": _motors(before, after),
        "encoder_verification": {
            "passed": verified,
            "error": None if verified else "encoder direction missing",
            "checks": [
                {"side": side, "passed": passed}
                for side, passed in zip(_SIDES, checks)
            ],
        },
        "stop": deepcopy(_CLEAN_STOP),
    }
    if command_before != before:
        result["segments"] = [
            _segment(
                kind="inter_slice_settling",
                commanded_sides=(),
                before=before,
                after=command_before,
                checks=settling_checks,
                verified=False,
                error="uncommanded encoder settling",
                reason="inter_slice_encoder_settling_observed",
            ),
            _segment(
                kind="commanded",
                commanded_sides=_SIDES,
                before=command_before,
                after=after,
                checks=checks,
                verified=verified,
                error=None if verified else "encoder direction missing",
                reason="angle_command_completed",
            ),
        ]
    return result


def _correlated_observation(value, final_angles, outcome):
    if not isinstance(value, Mapping) or not isinstance(value.get("motors"), list):
        _fail(
            "blast_canonical_observation_invalid",
            "Canonical BLAST observation lacks motor positions",
        )
    matching = [
        motor for motor in value["motors"]
        if isinstance(motor, Mapping) and motor.get("role") in _ROLES
    ]
    positions = {
        motor.get("role"): motor.get("position") for motor in matching
        if type(motor.get("position")) is int
    }
    if len(matching) != 2 or tuple(positions.get(role) for role in _ROLES) != final_angles:
        _fail(
            "blast_canonical_observation_invalid",
            "Canonical BLAST observation is not correlated with motion",
        )
    observation = deepcopy(dict(value))
    observation["last_outcome"] = deepcopy(outcome)
    return observation


def build_blast_navigation_motion_result(
    action: str,
    command_results: Sequence[Mapping[str, object]],
    *,
    expected_start_angles: Mapping[str, int],
    canonical_observation: Mapping[str, object],
):
    """Build a result accepted by existing shared encoder odometry."""

    if action not in BLAST_NAVIGATION_COMMANDS:
        _fail(
            "invalid_blast_motion_action",
            "BLAST cannot translate this semantic action",
        )
    commands = BLAST_NAVIGATION_COMMANDS[action]
    if (
        not isinstance(command_results, Sequence)
        or isinstance(command_results, (str, bytes, bytearray))
        or not 1 <= len(command_results) <= len(commands)
        or (len(commands) == 1 and len(command_results) != 1)
    ):
        _fail(
            "blast_motion_result_count_mismatch",
            "BLAST receipts do not form a valid action prefix",
        )

    previous_after = _angles(expected_start_angles, exact=True)
    durations = BLAST_NAVIGATION_ACTION_SPECS[action]["slice_durations_ms"]
    slices = []
    for index, (command, result) in enumerate(zip(commands, command_results), 1):
        before, after, _deltas, checks = _decode_result(command, result)
        settling_checks = None
        if before != previous_after:
            settling = tuple(
                current - previous
                for previous, current in zip(previous_after, before)
            )
            if index == 1 or any(
                abs(delta) > _MAX_INTER_SLICE_SETTLING_DEGREES
                for delta in settling
            ):
                _fail(
                    "blast_motion_slice_discontinuous",
                    (
                        "BLAST motion has an unobserved encoder gap: "
                        f"action={action} slice={index} "
                        f"previous={previous_after} before={before} "
                        f"delta={settling}"
                    ),
                )
            settling_checks = tuple(
                abs(delta) <= _MAX_INTER_SLICE_SETTLING_DEGREES
                for delta in settling
            )
        slices.append(_slice(
            index,
            len(commands),
            durations[index - 1],
            previous_after,
            before,
            after,
            checks,
            settling_checks,
        ))
        previous_after = after

    verified_count = sum(
        item["encoder_verification"]["passed"] for item in slices
    )
    if verified_count != len(slices):
        status, reason = "verification_failed", "encoder_verification_failed"
    elif len(slices) == len(commands):
        status, reason = "completed", "semantic_action_completed"
    else:
        status, reason = "interrupted", "semantic_action_incomplete"
    outcome = {
        "kind": "pulse",
        "action": action,
        "status": status,
        "reason": reason,
        "stop_confirmed": True,
        "requested_slice_count": len(commands),
        "completed_slice_count": verified_count,
        "slices": slices,
        "encoder_verification": {
            "passed": verified_count == len(commands),
            "verified_slice_count": verified_count,
            "requested_slice_count": len(commands),
        },
    }
    return {
        "action": action,
        "outcome": outcome,
        "observation": _correlated_observation(
            canonical_observation, previous_after, outcome,
        ),
        "stop": deepcopy(_CLEAN_STOP),
    }


__all__ = ("build_blast_navigation_motion_result",)
