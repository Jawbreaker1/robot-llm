"""Nominal BLAST yaw-only echo points, never occupancy or navigation authority.

Vertical pitch, beam width, and scan-turn translation are unmodeled.
"""

import math
from typing import Mapping

from . import blast_observation_monitor as scan_contract
from .blast_navigation_calibration import BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
from .blast_scan_observation import SCAN_MAX_ABSOLUTE_BEARING_DEG
from .physical_odometry import PhysicalPose, normalize_heading_mdeg


def _finite(value):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise ValueError("BLAST scan projection evidence is invalid")
    return float(value)


def _delta(heading, reference):
    return (heading - reference + 180.0) % 360.0 - 180.0


def _headings(scan):
    start = _finite(scan.get("start_heading_deg"))
    rays = scan.get("angular_rays", scan["rays"])
    values = []
    for ray in rays:
        relative = _finite(ray.get("relative_heading_deg"))
        values.append(relative)
    center = values[0]
    left = [
        value for ray, value in zip(rays[1:], values[1:])
        if ray.get("side", "").startswith("left_")
    ]
    right = [
        value for ray, value in zip(rays[1:], values[1:])
        if ray.get("side", "").startswith("right_")
    ]
    if (
        not 1 <= len(values) <= 9
        or center != 0.0
        or not all(
            -SCAN_MAX_ABSOLUTE_BEARING_DEG <= value < 0.0
            for value in left
        )
        or not all(
            0.0 < value <= SCAN_MAX_ABSOLUTE_BEARING_DEG
            for value in right
        )
        or any(first <= second for first, second in zip(left, left[1:]))
        or any(first >= second for first, second in zip(right, right[1:]))
    ):
        raise ValueError("BLAST scan heading topology is invalid")
    encoder_restoration = scan.get("encoder_restoration")
    error = _finite(
        encoder_restoration.get("opposed_residue_deg")
        if isinstance(encoder_restoration, Mapping) else None
    )
    if (
        scan.get("state") != "partial"
        and scan.get("sweep_direction") is None
        and abs(error) > scan_contract.SCAN_RESTORATION_TOLERANCE_DEG
    ):
        raise ValueError("BLAST scan restoration evidence is inconsistent")
    return rays, tuple(values)


def project_blast_scan_planar_surfaces(
    *, scan: Mapping[str, object], scan_pose: PhysicalPose
) -> Mapping[str, object]:
    """Project measured v3 rays into provisional local-odometry points."""

    if not isinstance(scan_pose, PhysicalPose):
        raise ValueError("BLAST scan pose is invalid")
    checked = scan_contract.validate_blast_scan_ray_contract(scan)
    complete = (
        checked.get("state") == "complete"
        and checked.get("result") == "restored"
        and checked.get("restoration_verified") is True
    )
    partial = (
        checked.get("state") == "partial"
        and checked.get("result") == "coverage_incomplete"
    )
    if not (
        (complete or partial)
        and checked["rays"][0].get("side") == "center"
    ):
        raise ValueError("BLAST scan is not projection-ready")
    projection_rays, raw_headings = _headings(checked)
    sensor = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.range_sensor_extrinsics
    if not (
        sensor.complete
        and all(
            sensor.matches_navigation_body_angle(ray["body_motor_angle_deg"])
            for ray in projection_rays
        )
    ):
        raise ValueError("BLAST scan sensor pose is not verified")
    forward, left, yaw = (
        sensor.forward_offset_mm,
        sensor.left_offset_mm,
        sensor.yaw_mdeg,
    )
    assert forward is not None and left is not None and yaw is not None

    points = []
    for ray, raw_heading in zip(projection_rays, raw_headings):
        if (
            ray.get("observation_settled") is not True
            or ray["range_state"] != scan_contract.RANGE_STATE_MEASURED
        ):
            continue
        magnitude = round(abs(raw_heading) * 1_000)
        if ray["side"] == "center":
            relative = 0
        elif ray["side"].startswith("left_"):
            relative = magnitude
        else:
            relative = -magnitude
        body_heading = normalize_heading_mdeg(scan_pose.heading_mdeg + relative)
        body_angle = math.radians(body_heading / 1_000)
        origin_x = scan_pose.x_mm + forward * math.cos(body_angle) - left * math.sin(body_angle)
        origin_y = scan_pose.y_mm + forward * math.sin(body_angle) + left * math.cos(body_angle)
        beam_heading = normalize_heading_mdeg(body_heading + yaw)
        beam_angle = math.radians(beam_heading / 1_000)
        distance = _finite(ray["distance_mm"])
        points.append({
            "side": ray["side"],
            "measured_range_mm": distance,
            "relative_bearing_mdeg": relative,
            "sensor_origin_x_mm": round(origin_x),
            "sensor_origin_y_mm": round(origin_y),
            "beam_heading_mdeg": beam_heading,
            "nominal_echo_x_mm": round(origin_x + distance * math.cos(beam_angle)),
            "nominal_echo_y_mm": round(origin_y + distance * math.sin(beam_angle)),
        })
    return {
        "schema": "blast-planar-scan-projection/v1",
        "frame": "EPISODE_LOCAL_ODOMETRY",
        "quality": "PROVISIONAL_YAW_ONLY",
        "vertical_pitch_compensated": False,
        "ultrasonic_beam_width_modeled": False,
        "scan_turn_translation_compensated": False,
        "points": points,
    }
