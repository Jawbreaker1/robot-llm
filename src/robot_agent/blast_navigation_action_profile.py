"""Physical meaning of BLAST's bounded semantic motion actions."""

from copy import deepcopy

from .blast_navigation_calibration import (
    BLAST_NAVIGATION_EVIDENCE_ID,
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .physical_navigation_contract import (
    ADVANCE,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)


DRIVE_SPEED_DPS = 120
DRIVE_ENCODER_DEGREES = 90
# Ideal run-angle time only. A future transport must separately budget BLE
# round trips and post-motion settling.
DRIVE_DURATION_MS = 750
TURN_SPEED_DPS = 180
TURN_ENCODER_DEGREES_PER_PULSE = 45
TURN_DURATION_MS_PER_PULSE = 250
# Four bounded pulses are the closest existing semantic quarter turn. The
# encoder-derived 93.96-degree result remains authoritative over the label.
TURN_PULSES_PER_QUARTER_TURN = 4
TURN_ENCODER_DEGREES_PER_QUARTER_TURN = (
    TURN_ENCODER_DEGREES_PER_PULSE * TURN_PULSES_PER_QUARTER_TURN
)
# Live four-pulse turns delivered 194 and 191.5 mean absolute encoder
# degrees.  The action target describes expected physical encoder evidence,
# while the fixed command sequence above remains exactly 4 x 45 degrees.
TURN_EXPECTED_ACTUAL_OPPOSED_ENCODER_DEGREES_PER_QUARTER_TURN = 193
_ODOMETRY = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
_NOMINAL_ADVANCE_MM = int(round(
    DRIVE_ENCODER_DEGREES * _ODOMETRY.linear_mm_per_encoder_degree
))
_NOMINAL_QUARTER_TURN_DEGREES = round(
    TURN_EXPECTED_ACTUAL_OPPOSED_ENCODER_DEGREES_PER_QUARTER_TURN
    * _ODOMETRY.turn_mdeg_per_opposed_encoder_degree
    / 1_000,
    2,
)

BLAST_NAVIGATION_COMMANDS = {
    ADVANCE: ("drive_forward",),
    REVERSE: ("drive_reverse",),
    TURN_LEFT_90: ("turn_left",) * TURN_PULSES_PER_QUARTER_TURN,
    TURN_RIGHT_90: ("turn_right",) * TURN_PULSES_PER_QUARTER_TURN,
}


def _action_spec(
    *,
    left_speed_dps,
    right_speed_dps,
    durations_ms,
    estimated_body_turn_degrees,
    target_encoder_degrees,
    calibration,
    calibration_evidence,
):
    return {
        "left_speed_dps": left_speed_dps,
        "right_speed_dps": right_speed_dps,
        "slice_durations_ms": list(durations_ms),
        "slice_count": len(durations_ms),
        "total_duration_ms": sum(durations_ms),
        "estimated_body_turn_degrees": estimated_body_turn_degrees,
        "target_mean_abs_encoder_degrees": target_encoder_degrees,
        "calibration_evidence": deepcopy(calibration_evidence),
        "calibration": calibration,
    }


_DRIVE_EVIDENCE = {
    "evidence_id": BLAST_NAVIGATION_EVIDENCE_ID,
    "commanded_encoder_degrees": DRIVE_ENCODER_DEGREES,
    "observed_forward_progress_mm": _NOMINAL_ADVANCE_MM,
}
_TURN_EVIDENCE = {
    "evidence_id": BLAST_NAVIGATION_EVIDENCE_ID,
    "commanded_encoder_degrees_per_pulse": (
        TURN_ENCODER_DEGREES_PER_PULSE
    ),
    "commanded_encoder_degrees_total": (
        TURN_ENCODER_DEGREES_PER_QUARTER_TURN
    ),
    "observed_mean_abs_encoder_degrees": (
        TURN_EXPECTED_ACTUAL_OPPOSED_ENCODER_DEGREES_PER_QUARTER_TURN
    ),
    "pulse_count": TURN_PULSES_PER_QUARTER_TURN,
    "estimated_body_turn_degrees": _NOMINAL_QUARTER_TURN_DEGREES,
}

BLAST_NAVIGATION_ACTION_SPECS = {
    ADVANCE: _action_spec(
        left_speed_dps=DRIVE_SPEED_DPS,
        right_speed_dps=DRIVE_SPEED_DPS,
        durations_ms=(DRIVE_DURATION_MS,),
        estimated_body_turn_degrees=None,
        target_encoder_degrees=DRIVE_ENCODER_DEGREES,
        calibration="provisional-live-range-derived",
        calibration_evidence=_DRIVE_EVIDENCE,
    ),
    REVERSE: _action_spec(
        left_speed_dps=-DRIVE_SPEED_DPS,
        right_speed_dps=-DRIVE_SPEED_DPS,
        durations_ms=(DRIVE_DURATION_MS,),
        estimated_body_turn_degrees=None,
        target_encoder_degrees=DRIVE_ENCODER_DEGREES,
        calibration="mirrored-provisional-live-range-derived",
        calibration_evidence=_DRIVE_EVIDENCE,
    ),
    TURN_LEFT_90: _action_spec(
        left_speed_dps=-TURN_SPEED_DPS,
        right_speed_dps=TURN_SPEED_DPS,
        durations_ms=(TURN_DURATION_MS_PER_PULSE,)
        * TURN_PULSES_PER_QUARTER_TURN,
        estimated_body_turn_degrees=_NOMINAL_QUARTER_TURN_DEGREES,
        target_encoder_degrees=(
            TURN_EXPECTED_ACTUAL_OPPOSED_ENCODER_DEGREES_PER_QUARTER_TURN
        ),
        calibration="provisional-live-encoder-and-reference-derived",
        calibration_evidence=_TURN_EVIDENCE,
    ),
    TURN_RIGHT_90: _action_spec(
        left_speed_dps=TURN_SPEED_DPS,
        right_speed_dps=-TURN_SPEED_DPS,
        durations_ms=(TURN_DURATION_MS_PER_PULSE,)
        * TURN_PULSES_PER_QUARTER_TURN,
        estimated_body_turn_degrees=-_NOMINAL_QUARTER_TURN_DEGREES,
        target_encoder_degrees=(
            TURN_EXPECTED_ACTUAL_OPPOSED_ENCODER_DEGREES_PER_QUARTER_TURN
        ),
        calibration="provisional-live-encoder-and-reference-derived",
        calibration_evidence=_TURN_EVIDENCE,
    ),
}


def blast_navigation_action_specs():
    """Return detached specs ready for a future BLAST transport description."""

    return deepcopy(BLAST_NAVIGATION_ACTION_SPECS)


__all__ = (
    "BLAST_NAVIGATION_ACTION_SPECS",
    "BLAST_NAVIGATION_COMMANDS",
    "DRIVE_ENCODER_DEGREES",
    "TURN_ENCODER_DEGREES_PER_PULSE",
    "TURN_ENCODER_DEGREES_PER_QUARTER_TURN",
    "TURN_EXPECTED_ACTUAL_OPPOSED_ENCODER_DEGREES_PER_QUARTER_TURN",
    "TURN_PULSES_PER_QUARTER_TURN",
    "blast_navigation_action_specs",
)
