"""Nominal BLAST yaw-only echo points, never occupancy or navigation authority.

Vertical pitch, beam width, and scan-turn translation are unmodeled.
"""

import math
from typing import Mapping

from . import blast_observation_monitor as scan_contract
from .blast_navigation_calibration import BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
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
    values = []
    for ray in scan["rays"]:
        heading = _finite(ray.get("heading_deg"))
        relative = _finite(ray.get("relative_heading_deg"))
        if not math.isclose(relative, _delta(heading, start), abs_tol=1e-9):
            raise ValueError("BLAST scan heading evidence is inconsistent")
        values.append(relative)
    center, left_near, left_far, right_near, right_far = values
    if not (
        center == 0.0
        and -90.0 <= left_far < left_near < 0.0
        and 0.0 < right_near < right_far <= 90.0
    ):
        raise ValueError("BLAST scan heading topology is invalid")
    final = _finite(scan.get("final_heading_deg"))
    error = _finite(scan.get("restoration_error_deg"))
    if (
        not math.isclose(error, _delta(final, start), abs_tol=1e-9)
        or abs(error) > scan_contract.SCAN_RESTORATION_TOLERANCE_DEG
    ):
        raise ValueError("BLAST scan restoration evidence is inconsistent")
    return tuple(values)


def project_blast_scan_planar_surfaces(
    *, scan: Mapping[str, object], scan_pose: PhysicalPose
) -> Mapping[str, object]:
    """Project measured v3 rays into provisional local-odometry points."""

    if not isinstance(scan_pose, PhysicalPose):
        raise ValueError("BLAST scan pose is invalid")
    checked = scan_contract.validate_blast_scan_ray_contract(scan)
    if not (
        checked.get("state") == "complete"
        and checked.get("result") == "restored"
        and checked.get("restoration_verified") is True
        and checked["rays"][0].get("side") == "center"
        and checked["rays"][0].get("observation_settled") is True
    ):
        raise ValueError("BLAST scan is not projection-ready")
    sensor = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.range_sensor_extrinsics
    if not (
        sensor.complete
        and all(
            sensor.matches_navigation_body_angle(ray["body_motor_angle_deg"])
            for ray in checked["rays"]
        )
    ):
        raise ValueError("BLAST scan sensor pose is not verified")
    raw_headings = _headings(checked)
    forward, left, yaw = (
        sensor.forward_offset_mm,
        sensor.left_offset_mm,
        sensor.yaw_mdeg,
    )
    assert forward is not None and left is not None and yaw is not None

    points = []
    for ray, raw_heading in zip(checked["rays"], raw_headings):
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
