"""Fail-closed BLAST post-pulse turn continuation policy."""

from collections.abc import Mapping
import math

from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_observation_monitor import (
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    blast_range_state,
)


def blast_turn_slice_allows_continuation(
    command_result,
    *,
    allow_no_valid_distance_with_bounded_evidence=False,
) -> bool:
    """Admit another pulse only from settled, calibrated telemetry.

    The opt-in is bounded scan/route turn eligibility for a no-return echo,
    never a claim that geometric clearance has been proved.
    """

    if not (
        isinstance(command_result, Mapping)
        and command_result.get("completed") is True
        and command_result.get("observation_settled") is True
    ):
        return False
    sensors = command_result.get("observation")
    motors = sensors.get("motor_angles_deg") if isinstance(
        sensors, Mapping
    ) else None
    imu = sensors.get("imu") if isinstance(sensors, Mapping) else None
    heading = imu.get("heading_deg") if isinstance(imu, Mapping) else None
    if not (
        sensors.get("motion_active") is False
        and BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .range_sensor_extrinsics.matches_navigation_body_angle(
            motors.get("body") if isinstance(motors, Mapping) else None
        )
        and isinstance(heading, (int, float))
        and not isinstance(heading, bool)
        and math.isfinite(float(heading))
    ):
        return False
    distance = sensors.get("distance_mm")
    state = blast_range_state(distance)
    if state == RANGE_STATE_NO_VALID_DISTANCE:
        return allow_no_valid_distance_with_bounded_evidence is True
    return (
        state == RANGE_STATE_MEASURED
        and float(distance) > BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .minimum_rotation_clearance_mm()
    )


__all__ = ("blast_turn_slice_allows_continuation",)
