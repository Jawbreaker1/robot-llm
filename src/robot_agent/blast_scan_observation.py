"""Pure validation and projection helpers for BLAST scan observations."""

from __future__ import annotations

from copy import deepcopy
import math
from typing import Mapping


SCAN_RESULT_SCHEMA = "blast-scan-front-arc/v3"
SCAN_RESTORATION_TOLERANCE_DEG = 5.0
SCAN_REPEATED_RAY_MAX_DELTA_DEG = 5.0
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


def finite_number(value):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        return None
    return float(value)


def blast_range_state(distance_mm):
    """Classify Pybricks ultrasonic output without inventing clearance."""

    value = finite_number(distance_mm)
    if (
        value is None
        or not 0 <= value <= PYBRICKS_ULTRASONIC_NO_VALID_DISTANCE_MM
    ):
        return RANGE_STATE_INVALID
    if value == PYBRICKS_ULTRASONIC_NO_VALID_DISTANCE_MM:
        return RANGE_STATE_NO_VALID_DISTANCE
    return RANGE_STATE_MEASURED


def body_motor_angle(observation):
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


def scan_heading(observation):
    imu = observation.get("imu")
    return finite_number(
        imu.get("heading_deg") if isinstance(imu, dict) else None
    )


def scan_heading_delta(heading, reference):
    if heading is None or reference is None:
        return None
    return (heading - reference + 180.0) % 360.0 - 180.0


def scan_ray(
    side,
    observation,
    start_heading,
    observation_settled,
    evidence_use=SCAN_RAY_EVIDENCE_SETTLED,
):
    heading = scan_heading(observation)
    observed_at_ms = observation.get("observed_at_ms")
    if type(observed_at_ms) is not int:
        observed_at_ms = None
    distance_mm = finite_number(observation.get("distance_mm"))
    return {
        "side": side,
        "distance_mm": distance_mm,
        "range_state": blast_range_state(distance_mm),
        "body_motor_angle_deg": body_motor_angle(observation),
        "heading_deg": heading,
        "relative_heading_deg": scan_heading_delta(heading, start_heading),
        "observation_settled": observation_settled,
        "evidence_use": evidence_use,
        "observed_at_ms": observed_at_ms,
    }


def aggregate_repeated_scan_ray(
    side,
    start_heading,
    primary,
    repeated,
):
    """Use a settled same-heading retry only when the primary is weak."""

    primary_observation, primary_settled, _primary_evidence = primary
    repeated_observation, repeated_settled, _repeated_evidence = repeated
    primary_state = blast_range_state(primary_observation.get("distance_mm"))
    repeated_state = blast_range_state(repeated_observation.get("distance_mm"))
    repeated_delta = scan_heading_delta(
        scan_heading(repeated_observation),
        scan_heading(primary_observation),
    )
    primary_is_usable = (
        primary_settled is True
        and primary_state == RANGE_STATE_MEASURED
    )
    repeated_improves_evidence = (
        repeated_settled is True
        and repeated_delta is not None
        and abs(repeated_delta) <= SCAN_REPEATED_RAY_MAX_DELTA_DEG
        and (
            repeated_state == RANGE_STATE_MEASURED
            or (
                primary_settled is not True
                and repeated_state == RANGE_STATE_NO_VALID_DISTANCE
            )
        )
    )
    observation, observation_settled, evidence_use = (
        repeated
        if not primary_is_usable and repeated_improves_evidence
        else primary
    )
    return scan_ray(
        side,
        observation,
        start_heading,
        observation_settled,
        evidence_use,
    )


__all__ = (
    "PYBRICKS_ULTRASONIC_NO_VALID_DISTANCE_MM",
    "RANGE_STATE_INVALID",
    "RANGE_STATE_MEASURED",
    "RANGE_STATE_NO_VALID_DISTANCE",
    "SCAN_RAY_EVIDENCE_SETTLED",
    "SCAN_RAY_EVIDENCE_SWEEP_ONLY",
    "SCAN_RAY_SIDES",
    "SCAN_REPEATED_RAY_MAX_DELTA_DEG",
    "SCAN_RESTORATION_TOLERANCE_DEG",
    "SCAN_RESULT_SCHEMA",
    "aggregate_repeated_scan_ray",
    "blast_range_state",
    "body_motor_angle",
    "finite_number",
    "scan_heading",
    "scan_heading_delta",
    "scan_ray",
    "validate_blast_scan_ray_contract",
)
