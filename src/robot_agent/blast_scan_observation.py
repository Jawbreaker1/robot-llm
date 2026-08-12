"""Pure validation and projection helpers for BLAST scan observations."""

from __future__ import annotations

from copy import deepcopy
import math
from typing import Mapping

from .physical_navigation_contract import MOTION_ACTIONS, SCAN_FRONT_ARC


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
SCAN_ANGULAR_RAY_SIDES = (
    "center",
    "left_1",
    "left_2",
    "left_3",
    "left_4",
    "right_1",
    "right_2",
    "right_3",
    "right_4",
)
ROBOT_RELATIVE_SIDE_SCAN_SCHEMA = "blast-robot-relative-side-scan/v2"
ROBOT_RELATIVE_SIDE_RAYS = (
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
    angular_rays = scan.get("angular_rays")
    evidence_rays = angular_rays if isinstance(angular_rays, list) else rays

    def ray_invalid(ray):
        return (
            not isinstance(ray, Mapping)
            or type(ray.get("observation_settled")) is not bool
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

    def same_ray(first, second):
        if not isinstance(first, Mapping) or not isinstance(second, Mapping):
            return False
        return {
            key: value for key, value in first.items() if key != "side"
        } == {
            key: value for key, value in second.items() if key != "side"
        }

    angular_invalid = False
    if angular_rays is not None:
        relative = tuple(
            finite_number(ray.get("relative_heading_deg"))
            if isinstance(ray, Mapping) else None
            for ray in angular_rays
        ) if isinstance(angular_rays, list) else ()
        angular_invalid = (
            not isinstance(angular_rays, list)
            or tuple(
                ray.get("side") if isinstance(ray, Mapping) else None
                for ray in angular_rays
            ) != SCAN_ANGULAR_RAY_SIDES
            or any(ray_invalid(ray) for ray in angular_rays)
            or len(relative) != 9
            or any(value is None for value in relative)
            or not (
                relative[0] == 0.0
                and 0.0 > relative[1] > relative[2]
                > relative[3] > relative[4] >= -90.0
                and 0.0 < relative[5] < relative[6]
                < relative[7] < relative[8] <= 90.0
            )
            or not isinstance(rays, list)
            or len(rays) != 5
            or not all(
                same_ray(rays[canonical], angular_rays[dense])
                for canonical, dense in ((0, 0), (1, 2), (2, 4),
                                         (3, 6), (4, 8))
            )
        )
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
            for ray in evidence_rays
        )
        or any(ray_invalid(ray) for ray in rays)
        or angular_invalid
    ):
        raise ValueError("BLAST scan result is invalid")
    return deepcopy(scan)


def summarize_robot_relative_side_scan(scan):
    """Return concise physical-side evidence without raw heading signs."""

    checked = validate_blast_scan_ray_contract(scan)
    rays = {"left": [], "right": []}
    source = checked.get("angular_rays", checked["rays"])
    for ray in source:
        side = ray["side"]
        physical_side = (
            "left" if side.startswith("left_")
            else "right" if side.startswith("right_")
            else None
        )
        if physical_side is None:
            continue
        state = ray["range_state"]
        settled = ray["observation_settled"] is True
        relative_heading = finite_number(ray.get("relative_heading_deg"))
        if relative_heading is None:
            raise ValueError("BLAST side-scan bearing is invalid")
        rays[physical_side].append({
            "range_state": state if settled else "UNRESOLVED_SWEEP_ONLY",
            "distance_mm": (
                ray["distance_mm"]
                if settled and state == RANGE_STATE_MEASURED
                else None
            ),
            "absolute_bearing_deg": abs(relative_heading),
        })
    return {
        "schema": ROBOT_RELATIVE_SIDE_SCAN_SCHEMA,
        "frame": "ROBOT_RELATIVE_AT_SCAN_START",
        "physical_side_labels_authoritative": True,
        "rays": rays,
    }


def current_side_scan(history, latest_scan_view):
    """Summarize the latest view only while its scan is still current."""

    if not isinstance(latest_scan_view, Mapping):
        return None
    for item in reversed(history):
        action = item.get("action")
        if action == SCAN_FRONT_ARC:
            scan = latest_scan_view.get("scan")
            return (
                summarize_robot_relative_side_scan(scan)
                if isinstance(scan, Mapping)
                else None
            )
        if action in MOTION_ACTIONS:
            return None
    return None


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
    "ROBOT_RELATIVE_SIDE_RAYS",
    "ROBOT_RELATIVE_SIDE_SCAN_SCHEMA",
    "SCAN_ANGULAR_RAY_SIDES",
    "SCAN_RAY_EVIDENCE_SETTLED",
    "SCAN_RAY_EVIDENCE_SWEEP_ONLY",
    "SCAN_RAY_SIDES",
    "SCAN_REPEATED_RAY_MAX_DELTA_DEG",
    "SCAN_RESTORATION_TOLERANCE_DEG",
    "SCAN_RESULT_SCHEMA",
    "aggregate_repeated_scan_ray",
    "blast_range_state",
    "body_motor_angle",
    "current_side_scan",
    "finite_number",
    "scan_heading",
    "scan_heading_delta",
    "scan_ray",
    "summarize_robot_relative_side_scan",
    "validate_blast_scan_ray_contract",
)
