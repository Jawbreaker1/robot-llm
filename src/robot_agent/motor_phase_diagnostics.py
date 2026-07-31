"""Pure diagnostics for position-dependent EV3 motor starts.

The worker already returns absolute encoder positions for every temporal
segment.  This module derives a bounded rotational phase from that existing,
validated evidence; it does not add fields to the EV3 wire protocol.
"""

from __future__ import division

from .physical_navigation_contract import EXPECTED_ACTION_SPECS
from .physical_odometry import (
    DriveMotorRoles,
    PhysicalNavigationContractError,
    verified_motion_from_result,
)


PHASE_EVENT_SCHEMA = "robot-motor-phase-event/v1"
DEFAULT_PHASE_PERIOD_DEGREES = 360
DEFAULT_PHASE_BUCKET_DEGREES = 30


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _periods(value):
    if _is_int(value):
        result = {"left": value, "right": value}
    elif isinstance(value, dict) and set(value) == set(("left", "right")):
        result = dict(value)
    else:
        raise ValueError("phase period must cover left and right")
    if any(
        not _is_int(period) or not 1 <= period <= 100000
        for period in result.values()
    ):
        raise ValueError("phase period is outside the supported range")
    return result


def _forward_signs(value):
    if value is None:
        return {"left": 1, "right": 1}
    if (
        not isinstance(value, dict)
        or set(value) != set(("left", "right"))
        or any(sign not in (-1, 1) for sign in value.values())
    ):
        raise ValueError("forward speed signs must be left/right +/-1")
    return dict(value)


def motor_phase_events_from_result(
    action,
    result,
    drive_roles=None,
    phase_period_degrees=DEFAULT_PHASE_PERIOD_DEGREES,
    phase_bucket_degrees=DEFAULT_PHASE_BUCKET_DEGREES,
    forward_speed_signs=None,
):
    """Extract one deterministic diagnostic event per motor and segment."""

    if drive_roles is None:
        drive_roles = DriveMotorRoles()
    # This rejects ambiguous stop, aggregate, continuity, and motor evidence
    # before diagnostics inspect any absolute encoder position.
    verified_motion_from_result(action, result, drive_roles)
    try:
        action_spec = EXPECTED_ACTION_SPECS[action]
    except (KeyError, TypeError):
        raise ValueError("action has no fixed EV3 motion profile")
    periods = _periods(phase_period_degrees)
    signs = _forward_signs(forward_speed_signs)
    if (
        not _is_int(phase_bucket_degrees)
        or phase_bucket_degrees <= 0
        or any(
            period % phase_bucket_degrees != 0
            for period in periods.values()
        )
    ):
        raise ValueError("phase bucket must divide every phase period")

    expected_roles = {
        "left": drive_roles.left,
        "right": drive_roles.right,
    }
    events = []
    for slice_receipt in result["outcome"]["slices"]:
        temporal = slice_receipt.get("segments")
        if temporal is None:
            temporal = [] if slice_receipt.get("motors") == [] else [
                slice_receipt
            ]
        for segment in temporal:
            commanded_sides = segment.get("commanded_sides")
            verification = segment.get("encoder_verification")
            duration_ms = segment.get("duration_ms")
            if (
                not isinstance(commanded_sides, list)
                or len(commanded_sides) != len(set(commanded_sides))
                or any(
                    side not in ("left", "right")
                    for side in commanded_sides
                )
                or not isinstance(verification, dict)
                or type(verification.get("passed")) is not bool
                or not _is_int(duration_ms)
                or duration_ms <= 0
            ):
                raise PhysicalNavigationContractError(
                    "invalid_phase_source",
                    "Temporal segment cannot produce phase diagnostics",
                )
            motors = segment.get("motors")
            by_side = {}
            if isinstance(motors, list):
                by_side = dict(
                    (motor.get("side"), motor)
                    for motor in motors
                    if isinstance(motor, dict)
                )
            if set(by_side) != set(("left", "right")):
                raise PhysicalNavigationContractError(
                    "invalid_phase_source",
                    "Temporal segment lacks both drive encoders",
                )
            for side in ("left", "right"):
                motor = by_side[side]
                position_before = motor.get("position_before")
                position_delta = motor.get("position_delta")
                if (
                    motor.get("role") != expected_roles[side]
                    or not _is_int(position_before)
                    or not _is_int(position_delta)
                ):
                    raise PhysicalNavigationContractError(
                        "invalid_phase_source",
                        "Temporal motor evidence cannot produce a phase",
                    )
                period = periods[side]
                phase = position_before % period
                commanded = side in commanded_sides
                logical_speed = action_spec[side + "_speed_dps"]
                expected_abs_delta = (
                    abs(logical_speed) * duration_ms + 500
                ) // 1000
                actual_abs_delta = abs(position_delta)
                completion_percent = (
                    actual_abs_delta * 100 + expected_abs_delta // 2
                ) // expected_abs_delta
                expected_direction = (
                    (1 if logical_speed > 0 else -1) * signs[side]
                )
                direction_matches = (
                    position_delta == 0
                    or position_delta * expected_direction > 0
                )
                if not commanded:
                    start_outcome = "uncommanded"
                elif verification["passed"]:
                    start_outcome = "verified"
                elif position_delta == 0:
                    start_outcome = "zero_start"
                elif actual_abs_delta < expected_abs_delta:
                    start_outcome = "undertravel"
                else:
                    start_outcome = "unverified"
                events.append(
                    {
                        "schema": PHASE_EVENT_SCHEMA,
                        "action": action,
                        "slice_index": slice_receipt["slice_index"],
                        "segment_index": segment.get("segment_index", 1),
                        "segment_kind": segment.get("kind", "paired"),
                        "segment_status": segment.get("status"),
                        "side": side,
                        "role": motor["role"],
                        "commanded": commanded,
                        "position_before": position_before,
                        "position_delta": position_delta,
                        "phase_period_degrees": period,
                        "phase_degrees": phase,
                        "phase_bucket_degrees": phase_bucket_degrees,
                        "phase_bucket_start_degrees": (
                            phase // phase_bucket_degrees
                            * phase_bucket_degrees
                        ),
                        "expected_abs_delta": expected_abs_delta,
                        "completion_percent": completion_percent,
                        "direction_matches": direction_matches,
                        "verification_passed": verification["passed"],
                        "start_outcome": start_outcome,
                    }
                )
    return tuple(events)


def aggregate_motor_phase_events(events):
    """Aggregate exact phase buckets without guessing failure reasons."""

    fields = {
        "schema",
        "action",
        "slice_index",
        "segment_index",
        "segment_kind",
        "segment_status",
        "side",
        "role",
        "commanded",
        "position_before",
        "position_delta",
        "phase_period_degrees",
        "phase_degrees",
        "phase_bucket_degrees",
        "phase_bucket_start_degrees",
        "expected_abs_delta",
        "completion_percent",
        "direction_matches",
        "verification_passed",
        "start_outcome",
    }
    outcomes = {
        "verified",
        "zero_start",
        "undertravel",
        "unverified",
        "uncommanded",
    }
    grouped = {}
    for event in events:
        if (
            not isinstance(event, dict)
            or set(event) != fields
            or event["schema"] != PHASE_EVENT_SCHEMA
            or event["side"] not in ("left", "right")
            or type(event["commanded"]) is not bool
            or type(event["direction_matches"]) is not bool
            or type(event["verification_passed"]) is not bool
            or event["start_outcome"] not in outcomes
            or any(
                not _is_int(event[name])
                for name in (
                    "phase_period_degrees",
                    "phase_degrees",
                    "phase_bucket_degrees",
                    "phase_bucket_start_degrees",
                    "completion_percent",
                )
            )
            or not 0 <= event["phase_degrees"] < event[
                "phase_period_degrees"
            ]
            or event["phase_bucket_degrees"] <= 0
            or event["phase_bucket_start_degrees"]
            != event["phase_degrees"] // event[
                "phase_bucket_degrees"
            ] * event["phase_bucket_degrees"]
            or event["commanded"]
            != (event["start_outcome"] != "uncommanded")
        ):
            raise ValueError("motor phase event is invalid")
        key = (
            event["side"],
            event["role"],
            event["phase_period_degrees"],
            event["phase_bucket_degrees"],
            event["phase_bucket_start_degrees"],
        )
        row = grouped.setdefault(
            key,
            {
                "side": event["side"],
                "role": event["role"],
                "phase_period_degrees": event["phase_period_degrees"],
                "phase_bucket_degrees": event["phase_bucket_degrees"],
                "phase_bucket_start_degrees": event[
                    "phase_bucket_start_degrees"
                ],
                "commanded_attempts": 0,
                "verified_starts": 0,
                "zero_starts": 0,
                "undertravel_starts": 0,
                "unverified_starts": 0,
                "direction_mismatches": 0,
                "uncommanded_observations": 0,
                "completion_percent_total": 0,
            },
        )
        if not event["commanded"]:
            row["uncommanded_observations"] += 1
            continue
        row["commanded_attempts"] += 1
        row["completion_percent_total"] += event[
            "completion_percent"
        ]
        if not event["direction_matches"]:
            row["direction_mismatches"] += 1
        counter = {
            "verified": "verified_starts",
            "zero_start": "zero_starts",
            "undertravel": "undertravel_starts",
            "unverified": "unverified_starts",
        }[event["start_outcome"]]
        row[counter] += 1

    result = []
    for key in sorted(grouped):
        row = grouped[key]
        attempts = row["commanded_attempts"]
        row["mean_completion_percent"] = (
            (row.pop("completion_percent_total") + attempts // 2)
            // attempts
            if attempts
            else None
        )
        result.append(row)
    return tuple(result)
