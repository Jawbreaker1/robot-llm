"""Pure validation and projection helpers for BLAST scan observations."""

from __future__ import annotations

from copy import deepcopy
import math
from typing import Mapping

from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .physical_navigation_contract import MOTION_ACTIONS, SCAN_FRONT_ARC


SCAN_RESULT_SCHEMA = "blast-scan-front-arc/v3"
# LEGO-scale completion bands: close enough to the scan start to keep the
# provisional map useful, without treating normal backlash as a hard fault.
# Fresh range and motor checks still gate every following physical action.
SCAN_RESTORATION_TOLERANCE_DEG = 10.0
SCAN_RESTORATION_COMMON_MODE_TOLERANCE_MM = 25.0
SCAN_MAX_ABSOLUTE_BEARING_DEG = 180.0
SCAN_BEARING_SOURCE = "DRIVE_ENCODER_ODOMETRY"
SCAN_BEARING_FRAME = "ROBOT_RELATIVE_AT_SCAN_START"
SCAN_IMU_DIAGNOSTIC_AUTHORITY = "DIAGNOSTIC_ONLY"
SCAN_DRIVE_ENCODER_ROLES = ("left_drive", "right_drive")
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


def drive_encoder_angles(observation):
    """Return exact drive anchors; scan geometry never guesses them."""

    motor_angles = (
        observation.get("motor_angles_deg")
        if isinstance(observation, Mapping)
        else None
    )
    if not isinstance(motor_angles, Mapping) or any(
        type(motor_angles.get(role)) is not int
        for role in SCAN_DRIVE_ENCODER_ROLES
    ):
        return None
    return {
        role: motor_angles[role] for role in SCAN_DRIVE_ENCODER_ROLES
    }


def encoder_sweep_bearing_deg(observation, start_drive_angles):
    """Return unwrapped left-negative/right-positive encoder yaw."""

    current = drive_encoder_angles(observation)
    if current is None or not isinstance(start_drive_angles, Mapping) or any(
        type(start_drive_angles.get(role)) is not int
        for role in SCAN_DRIVE_ENCODER_ROLES
    ):
        return None
    left_delta = current["left_drive"] - start_drive_angles["left_drive"]
    right_delta = (
        current["right_drive"] - start_drive_angles["right_drive"]
    )
    opposed_encoder_deg = (right_delta - left_delta) / 2.0
    return -(
        opposed_encoder_deg
        * BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
        .turn_mdeg_per_opposed_encoder_degree
        / 1_000.0
    )


def encoder_relative_bearing_deg(observation, start_drive_angles):
    """Return encoder yaw normalized into one robot-relative circle."""

    value = encoder_sweep_bearing_deg(observation, start_drive_angles)
    return (
        None if value is None
        else round((value + 180.0) % 360.0 - 180.0, 6)
    )


def encoder_common_mode_residue_mm(final_drive_angles, start_drive_angles):
    if not all(isinstance(value, Mapping) for value in (
        final_drive_angles, start_drive_angles,
    )) or any(
        type(angles.get(role)) is not int
        for angles in (final_drive_angles, start_drive_angles)
        for role in SCAN_DRIVE_ENCODER_ROLES
    ):
        return None
    left_delta = (
        final_drive_angles["left_drive"]
        - start_drive_angles["left_drive"]
    )
    right_delta = (
        final_drive_angles["right_drive"]
        - start_drive_angles["right_drive"]
    )
    return (
        (left_delta + right_delta) / 2.0
        * BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
        .linear_mm_per_encoder_degree
    )


def validate_blast_scan_ray_contract(scan):
    """Validate encoder-authoritative bearings and restoration evidence."""

    if not isinstance(scan, Mapping) or scan.get("schema") != (
        SCAN_RESULT_SCHEMA
    ):
        raise ValueError("BLAST scan result is invalid")
    rays = scan.get("rays")
    angular_rays = scan.get("angular_rays")
    evidence_rays = angular_rays if isinstance(angular_rays, list) else rays

    start_encoders = scan.get("encoder_start_angles_deg")
    final_encoders = scan.get("encoder_final_angles_deg")
    restoration = scan.get("encoder_restoration")
    imu_diagnostics = scan.get("imu_heading_diagnostics")
    sweep_coverage = finite_number(scan.get("sweep_coverage_deg"))
    sweep_direction = scan.get("sweep_direction")
    sweep_turn_count = scan.get("sweep_turn_count")
    partial_scan = (
        scan.get("state") == "partial"
        and scan.get("result") == "coverage_incomplete"
    )

    def exact_drive_angles(value):
        return (
            isinstance(value, Mapping)
            and set(value) == set(SCAN_DRIVE_ENCODER_ROLES)
            and all(type(value.get(role)) is int for role in (
                SCAN_DRIVE_ENCODER_ROLES
            ))
        )

    def ray_invalid(ray):
        encoder_delta = (
            ray.get("drive_encoder_delta_deg")
            if isinstance(ray, Mapping) else None
        )
        heading = finite_number(
            ray.get("heading_deg") if isinstance(ray, Mapping) else None
        )
        relative = finite_number(
            ray.get("relative_heading_deg")
            if isinstance(ray, Mapping) else None
        )
        imu_heading = finite_number(
            ray.get("imu_heading_deg")
            if isinstance(ray, Mapping) else None
        )
        expected_bearing = None
        if exact_drive_angles(encoder_delta):
            unwrapped = -(
                (
                    encoder_delta["right_drive"]
                    - encoder_delta["left_drive"]
                )
                / 2.0
                * BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
                .turn_mdeg_per_opposed_encoder_degree
                / 1_000.0
            )
            expected_bearing = (unwrapped + 180.0) % 360.0 - 180.0
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
            or expected_bearing is None
            or heading is None
            or relative is None
            or not math.isclose(
                heading, expected_bearing, abs_tol=1e-9,
            )
            or not math.isclose(
                relative, expected_bearing, abs_tol=1e-9,
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
        labels = tuple(
            ray.get("side") if isinstance(ray, Mapping) else None
            for ray in angular_rays
        ) if isinstance(angular_rays, list) else ()
        left_count = len([
            label for label in labels if isinstance(label, str)
            and label.startswith("left_")
        ])
        right_count = len([
            label for label in labels if isinstance(label, str)
            and label.startswith("right_")
        ])
        expected_partial_labels = (
            ("center",)
            + tuple("left_{}".format(index) for index in range(
                1, left_count + 1,
            ))
            + tuple("right_{}".format(index) for index in range(
                1, right_count + 1,
            ))
        )
        side_count = left_count if partial_scan else 4
        left = relative[1:1 + side_count]
        right = relative[1 + side_count:]
        angular_invalid = (
            not isinstance(angular_rays, list)
            or labels != (
                expected_partial_labels
                if partial_scan else SCAN_ANGULAR_RAY_SIDES
            )
            or any(ray_invalid(ray) for ray in angular_rays)
            or len(relative) != (1 + left_count + right_count)
            or not (partial_scan or len(relative) == 9)
            or left_count > 4
            or right_count > 4
            or any(value is None for value in relative)
            or not (
                relative and relative[0] == 0.0
                and all(
                    0.0 > value >= -SCAN_MAX_ABSOLUTE_BEARING_DEG
                    for value in left
                )
                and all(
                    0.0 < value <= SCAN_MAX_ABSOLUTE_BEARING_DEG
                    for value in right
                )
                and all(
                    first > second for first, second in zip(left, left[1:])
                )
                and all(
                    first < second for first, second in zip(right, right[1:])
                )
            )
            or not isinstance(rays, list)
            or (
                partial_scan and rays != angular_rays
            )
            or (
                not partial_scan and (
                    len(rays) != 5
                    or not all(
                        same_ray(rays[canonical], angular_rays[dense])
                        for canonical, dense in (
                            (0, 0), (1, 2), (2, 4), (3, 6), (4, 8),
                        )
                    )
                )
            )
        )
    canonical_start = finite_number(scan.get("start_heading_deg"))
    canonical_final = finite_number(scan.get("final_heading_deg"))
    canonical_error = finite_number(scan.get("restoration_error_deg"))
    legacy_heading_fields_valid = all(
        finite_number(ray.get("heading_deg")) is not None
        for ray in evidence_rays
    ) if isinstance(evidence_rays, list) else False
    final_bearing = (
        encoder_relative_bearing_deg(
            {"motor_angles_deg": final_encoders}, start_encoders,
        )
        if exact_drive_angles(start_encoders)
        and exact_drive_angles(final_encoders)
        else None
    )
    common_residue = encoder_common_mode_residue_mm(
        final_encoders, start_encoders,
    )
    raw_sweep_bearing = (
        encoder_sweep_bearing_deg(
            {"motor_angles_deg": final_encoders}, start_encoders,
        )
        if exact_drive_angles(start_encoders)
        and exact_drive_angles(final_encoders)
        else None
    )
    sweep_contract = (
        sweep_coverage is None
        and sweep_direction is None
        and sweep_turn_count is None
    ) or (
        sweep_coverage is not None
        and sweep_direction in ("left", "right")
        and type(sweep_turn_count) is int
        and 1 <= sweep_turn_count <= 17
        and raw_sweep_bearing is not None
        and math.isclose(
            sweep_coverage, abs(raw_sweep_bearing), abs_tol=1e-9,
        )
        and (
            raw_sweep_bearing < 0
            if sweep_direction == "left" else raw_sweep_bearing > 0
        )
    )
    restoration_contract = (
        isinstance(restoration, Mapping)
        and set(restoration) == {
            "common_mode_residue_mm",
            "opposed_residue_deg",
            "motion_stopped",
            "observation_settled",
            "body_pose_verified",
        }
        and finite_number(restoration.get("common_mode_residue_mm"))
        is not None
        and finite_number(restoration.get("opposed_residue_deg"))
        is not None
        and all(type(restoration.get(key)) is bool for key in (
            "motion_stopped", "observation_settled",
            "body_pose_verified",
        ))
    )
    legacy_restored = (
        sweep_coverage is None
        and common_residue is not None
        and abs(common_residue)
        <= SCAN_RESTORATION_COMMON_MODE_TOLERANCE_MM
        and abs(final_bearing or 0.0) <= SCAN_RESTORATION_TOLERANCE_DEG
    )
    surroundings_pose_verified = (
        sweep_coverage is not None and 350.0 <= sweep_coverage <= 390.0
    )
    expected_restored = (
        restoration_contract
        and final_bearing is not None
        and restoration["motion_stopped"]
        and restoration["body_pose_verified"]
        and (legacy_restored or surroundings_pose_verified)
    )
    imu_contract = (
        isinstance(imu_diagnostics, Mapping)
        and set(imu_diagnostics) == {
            "authority", "start_heading_deg", "final_heading_deg",
            "restoration_error_deg",
        }
        and imu_diagnostics.get("authority")
        == SCAN_IMU_DIAGNOSTIC_AUTHORITY
    )
    raw_imu_start = (
        imu_diagnostics.get("start_heading_deg")
        if isinstance(imu_diagnostics, Mapping) else None
    )
    raw_imu_final = (
        imu_diagnostics.get("final_heading_deg")
        if isinstance(imu_diagnostics, Mapping) else None
    )
    raw_imu_error = (
        imu_diagnostics.get("restoration_error_deg")
        if isinstance(imu_diagnostics, Mapping) else None
    )
    imu_start = finite_number(raw_imu_start)
    imu_final = finite_number(raw_imu_final)
    imu_error = finite_number(raw_imu_error)
    if (
        not isinstance(rays, list)
        or (
            not partial_scan
            and tuple(
                ray.get("side") if isinstance(ray, Mapping) else None
                for ray in rays
            ) != SCAN_RAY_SIDES
        )
        or type(scan.get("all_observations_settled")) is not bool
        or scan.get("all_observations_settled") != all(
            isinstance(ray, Mapping)
            and ray.get("observation_settled") is True
            for ray in evidence_rays
        )
        or any(ray_invalid(ray) for ray in rays)
        or angular_invalid
        or not sweep_contract
        or (
            partial_scan
            and (
                sweep_coverage is None
                or not 0.0 < sweep_coverage < 350.0
            )
        )
        or (not partial_scan and scan.get("state") != "complete")
        or scan.get("bearing_source") != SCAN_BEARING_SOURCE
        or scan.get("bearing_frame") != SCAN_BEARING_FRAME
        or not exact_drive_angles(start_encoders)
        or not exact_drive_angles(final_encoders)
        or not legacy_heading_fields_valid
        or canonical_start != 0.0
        or canonical_final is None
        or canonical_error is None
        or final_bearing is None
        or not math.isclose(canonical_final, final_bearing, abs_tol=1e-9)
        or not math.isclose(canonical_error, final_bearing, abs_tol=1e-9)
        or not restoration_contract
        or not math.isclose(
            float(restoration["common_mode_residue_mm"]),
            common_residue,
            abs_tol=1e-9,
        )
        or not math.isclose(
            float(restoration["opposed_residue_deg"]),
            final_bearing,
            abs_tol=1e-9,
        )
        or type(scan.get("restoration_verified")) is not bool
        or scan.get("restoration_verified") is not (
            False if partial_scan else expected_restored
        )
        or scan.get("result") != (
            "coverage_incomplete" if partial_scan
            else "restored" if expected_restored
            else "restoration_unverified"
        )
        or not imu_contract
        or any(
            raw is not None and normalized is None
            for raw, normalized in (
                (raw_imu_start, imu_start),
                (raw_imu_final, imu_final),
                (raw_imu_error, imu_error),
            )
        )
        or (
            imu_start is not None
            and imu_final is not None
            and (
                imu_error is None
                or not math.isclose(
                    imu_error,
                    scan_heading_delta(imu_final, imu_start),
                    abs_tol=1e-9,
                )
            )
        )
        or (
            (imu_start is None or imu_final is None)
            and imu_error is not None
        )
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
    start_drive_angles,
    observation_settled,
    evidence_use=SCAN_RAY_EVIDENCE_SETTLED,
):
    heading = encoder_relative_bearing_deg(
        observation, start_drive_angles,
    )
    current_drive_angles = drive_encoder_angles(observation)
    if heading is None or current_drive_angles is None:
        raise ValueError("BLAST scan drive encoder evidence is invalid")
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
        "relative_heading_deg": heading,
        "imu_heading_deg": scan_heading(observation),
        "drive_encoder_delta_deg": {
            role: current_drive_angles[role] - start_drive_angles[role]
            for role in SCAN_DRIVE_ENCODER_ROLES
        },
        "observation_settled": observation_settled,
        "evidence_use": evidence_use,
        "observed_at_ms": observed_at_ms,
    }


def build_blast_encoder_scan(
    *, center, center_settled, start_drive_angles, sweep_samples,
    final, final_settled, final_body_verified,
):
    """Build nine representative rays from one encoder-measured full turn."""

    start_imu_heading = scan_heading(center)
    final_imu_heading = scan_heading(final)
    imu_restoration_error = scan_heading_delta(
        final_imu_heading, start_imu_heading,
    )
    final_drive_angles = drive_encoder_angles(final)
    restoration_error = encoder_relative_bearing_deg(
        final, start_drive_angles,
    )
    sweep_coverage = abs(encoder_sweep_bearing_deg(
        final, start_drive_angles,
    ) or 0.0)
    common_mode_residue = encoder_common_mode_residue_mm(
        final_drive_angles, start_drive_angles,
    )
    # A full turn need not land on its starting heading. Its exact encoder
    # endpoint becomes the next trusted pose; completion means coverage plus
    # a verified stop, not LEGO-perfect mechanical return.
    restoration_verified = (
        restoration_error is not None
        and common_mode_residue is not None
        and final.get("motion_active") is False
        and final_body_verified
        and 350.0 <= sweep_coverage <= 390.0
    )
    candidates = []
    for sample in sweep_samples:
        observation, settled, evidence_use = sample[1:]
        bearing = encoder_relative_bearing_deg(
            observation, start_drive_angles,
        )
        if bearing is not None:
            candidates.append((bearing, observation, settled, evidence_use))

    def representative(side, index, target):
        matching = [
            item for item in candidates
            if (item[0] < 0 if side == "left" else item[0] > 0)
        ]
        if not matching:
            raise ValueError("BLAST surroundings scan coverage is incomplete")
        _bearing, observation, settled, evidence_use = min(
            matching, key=lambda item: abs(item[0] - target),
        )
        return scan_ray(
            "{}_{}".format(side, index), observation,
            start_drive_angles, settled, evidence_use,
        )

    left_rays = [
        representative("left", index, target)
        for index, target in enumerate((-45.0, -90.0, -135.0, -175.0), 1)
    ]
    right_rays = [
        representative("right", index, target)
        for index, target in enumerate((45.0, 90.0, 135.0, 175.0), 1)
    ]
    center_ray = scan_ray(
        "center", center, start_drive_angles, center_settled,
    )
    angular_rays = [center_ray, *left_rays, *right_rays]
    rays = [
        center_ray,
        {**left_rays[1], "side": "left_near"},
        {**left_rays[3], "side": "left_far"},
        {**right_rays[1], "side": "right_near"},
        {**right_rays[3], "side": "right_far"},
    ]
    scan = {
        "schema": SCAN_RESULT_SCHEMA,
        "state": "complete",
        "result": (
            "restored" if restoration_verified
            else "restoration_unverified"
        ),
        "bearing_source": SCAN_BEARING_SOURCE,
        "bearing_frame": SCAN_BEARING_FRAME,
        "start_heading_deg": 0.0,
        "final_heading_deg": restoration_error,
        "restoration_error_deg": restoration_error,
        "restoration_verified": restoration_verified,
        "sweep_coverage_deg": sweep_coverage,
        "sweep_direction": "left",
        "sweep_turn_count": len(sweep_samples),
        "encoder_start_angles_deg": start_drive_angles,
        "encoder_final_angles_deg": final_drive_angles,
        "encoder_restoration": {
            "common_mode_residue_mm": common_mode_residue,
            "opposed_residue_deg": restoration_error,
            "motion_stopped": final.get("motion_active") is False,
            "observation_settled": final_settled is True,
            "body_pose_verified": final_body_verified,
        },
        "imu_heading_diagnostics": {
            "authority": SCAN_IMU_DIAGNOSTIC_AUTHORITY,
            "start_heading_deg": start_imu_heading,
            "final_heading_deg": final_imu_heading,
            "restoration_error_deg": imu_restoration_error,
        },
        "all_observations_settled": all(
            ray["observation_settled"] for ray in angular_rays
        ),
        "rays": rays,
        "angular_rays": angular_rays,
    }
    return validate_blast_scan_ray_contract(scan)


def build_blast_partial_scan(
    *, center, center_settled, start_drive_angles, sweep_samples,
    final, final_settled, final_body_verified,
):
    """Return honest partial evidence when a safe full sweep cannot continue."""

    candidates = []
    for sample in sweep_samples:
        observation, settled, evidence_use = sample[1:]
        bearing = encoder_relative_bearing_deg(
            observation, start_drive_angles,
        )
        if bearing not in (None, 0.0):
            candidates.append((bearing, observation, settled, evidence_use))

    def bounded_side(side):
        values = sorted(
            (
                item for item in candidates
                if (item[0] < 0 if side == "left" else item[0] > 0)
            ),
            key=lambda item: abs(item[0]),
        )
        if len(values) > 4:
            values = [
                values[round(index * (len(values) - 1) / 3)]
                for index in range(4)
            ]
        return [
            scan_ray(
                "{}_{}".format(side, index), item[1],
                start_drive_angles, item[2], item[3],
            )
            for index, item in enumerate(values, 1)
        ]

    center_ray = scan_ray(
        "center", center, start_drive_angles, center_settled,
    )
    angular_rays = [
        center_ray, *bounded_side("left"), *bounded_side("right"),
    ]
    final_drive_angles = drive_encoder_angles(final)
    restoration_error = encoder_relative_bearing_deg(
        final, start_drive_angles,
    )
    common_mode_residue = encoder_common_mode_residue_mm(
        final_drive_angles, start_drive_angles,
    )
    start_imu_heading = scan_heading(center)
    final_imu_heading = scan_heading(final)
    scan = {
        "schema": SCAN_RESULT_SCHEMA,
        "state": "partial",
        "result": "coverage_incomplete",
        "bearing_source": SCAN_BEARING_SOURCE,
        "bearing_frame": SCAN_BEARING_FRAME,
        "start_heading_deg": 0.0,
        "final_heading_deg": restoration_error,
        "restoration_error_deg": restoration_error,
        "restoration_verified": False,
        "sweep_coverage_deg": abs(encoder_sweep_bearing_deg(
            final, start_drive_angles,
        ) or 0.0),
        "sweep_direction": "left",
        "sweep_turn_count": len(sweep_samples),
        "encoder_start_angles_deg": start_drive_angles,
        "encoder_final_angles_deg": final_drive_angles,
        "encoder_restoration": {
            "common_mode_residue_mm": common_mode_residue,
            "opposed_residue_deg": restoration_error,
            "motion_stopped": final.get("motion_active") is False,
            "observation_settled": final_settled is True,
            "body_pose_verified": final_body_verified,
        },
        "imu_heading_diagnostics": {
            "authority": SCAN_IMU_DIAGNOSTIC_AUTHORITY,
            "start_heading_deg": start_imu_heading,
            "final_heading_deg": final_imu_heading,
            "restoration_error_deg": scan_heading_delta(
                final_imu_heading, start_imu_heading,
            ),
        },
        "all_observations_settled": all(
            ray["observation_settled"] for ray in angular_rays
        ),
        "rays": angular_rays,
        "angular_rays": angular_rays,
    }
    return validate_blast_scan_ray_contract(scan)


__all__ = (
    "PYBRICKS_ULTRASONIC_NO_VALID_DISTANCE_MM",
    "RANGE_STATE_INVALID",
    "RANGE_STATE_MEASURED",
    "RANGE_STATE_NO_VALID_DISTANCE",
    "ROBOT_RELATIVE_SIDE_RAYS",
    "ROBOT_RELATIVE_SIDE_SCAN_SCHEMA",
    "SCAN_ANGULAR_RAY_SIDES",
    "SCAN_BEARING_FRAME",
    "SCAN_BEARING_SOURCE",
    "SCAN_DRIVE_ENCODER_ROLES",
    "SCAN_IMU_DIAGNOSTIC_AUTHORITY",
    "SCAN_MAX_ABSOLUTE_BEARING_DEG",
    "SCAN_RAY_EVIDENCE_SETTLED",
    "SCAN_RAY_EVIDENCE_SWEEP_ONLY",
    "SCAN_RAY_SIDES",
    "SCAN_RESTORATION_COMMON_MODE_TOLERANCE_MM",
    "SCAN_RESTORATION_TOLERANCE_DEG",
    "SCAN_RESULT_SCHEMA",
    "blast_range_state",
    "body_motor_angle",
    "build_blast_encoder_scan",
    "build_blast_partial_scan",
    "current_side_scan",
    "drive_encoder_angles",
    "encoder_common_mode_residue_mm",
    "encoder_relative_bearing_deg",
    "encoder_sweep_bearing_deg",
    "finite_number",
    "scan_heading",
    "scan_heading_delta",
    "scan_ray",
    "summarize_robot_relative_side_scan",
    "validate_blast_scan_ray_contract",
)
