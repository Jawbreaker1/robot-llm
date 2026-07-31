#!/usr/bin/env python3
"""Fixed, Python 3.5-compatible motion profile for the EV3 navigator.

This module contains data only.  The host agent may choose one of these
semantic actions, but neither this profile nor the EV3 worker contains route
selection policy.
"""

from __future__ import print_function


REQUEST_SCHEMA = "ev3-agent-worker-request/v1"
RESPONSE_SCHEMA = "ev3-agent-worker-response/v2"
# Keep the identifier wire-compatible with the validated live prototype.
# The implementation now lives in production modules, but changing this
# value would unnecessarily break strict host capability validation.
WORKER_ID = "ev3-demo-agent-worker-v1"

MAX_FRAME_BYTES = 4096
# The 300 MHz EV3 spends a material part of a cold worker session importing
# Python modules, binding sysfs devices, and establishing its initial stop
# proof.  Concurrent audio startup can consume more of that same wall-clock
# budget even though navigation remains asynchronous.  Give this hardware
# profile enough lifetime for one worst-case host plan, bilateral scan, final
# encoder observation, and verified shutdown.  This process lifetime is not a
# motion duration: every pulse and scan slice remains independently bounded
# and self-stopping, SSH EOF still triggers cleanup immediately, and the host
# renews the worker before this absolute backstop is reached.
MAX_PROCESS_SECONDS = 180.0
MAX_REQUESTS = 256
MAX_PULSES = 40
MAX_PULSE_DURATION_MS = 32000
POLL_INTERVAL_MS = 20

# EV3-specific closed-loop compensation.  These limits are intentionally
# local to this hardware profile; faster hubs can publish different values.
ENCODER_RECOVERY_ACTIONS = ("ADVANCE", "REVERSE")
ENCODER_RECOVERY_MINIMUM_PROGRESS_DEGREES = 3
ENCODER_RECOVERY_LEADER_MINIMUM_DEGREES = 12
ENCODER_RECOVERY_ACCEPTABLE_COMPLETION_PERCENT = 75
ENCODER_RECOVERY_MAXIMUM_PROGRESS_SKEW_PERCENT = 15
ENCODER_RECOVERY_MAXIMUM_CATCH_UP_ATTEMPTS = 2
ENCODER_RECOVERY_MAXIMUM_PAIR_RETRY_ATTEMPTS = 1
ENCODER_RECOVERY_MAXIMUM_TOTAL_ATTEMPTS = 3
ENCODER_RECOVERY_MAXIMUM_STEP_DURATION_MS = 800
ENCODER_RECOVERY_MAXIMUM_TOTAL_DURATION_MS = 1600
ENCODER_RECOVERY_MAXIMUM_TOTAL_ENCODER_DEGREES = 400

# The stock 25 ms limit is below the measured two-write sysfs latency on the
# 300 MHz EV3.  This remains bounded while permitting both drive motors to
# start on the real brick.
MAX_START_SKEW_MS = 150

ALLOWED_OPERATIONS = (
    "describe",
    "observe",
    "pulse",
    "scan_turn",
    "scan_sample",
    "stop",
    "shutdown",
)

# Active IR scanning is a host-owned compound tool.  The host scan executor
# may request only these bounded relative body turns; the model never sees
# this low-level operation or supplies its argument.  The 15-degree lattice
# covers every transition between the fixed +/-60 degree coarse rays, the
# optional 15-degree refinement rays, and restoration to the start heading.
SCAN_TURN_ALLOWED_DELTAS_MDEG = tuple(
    value
    for value in range(-120000, 120001, 15000)
    if value != 0
)
SCAN_TURN_PROFILE_ID = "ev3rstorm-provisional-ir-turn-v2"
SCAN_TURN_CALIBRATION = "provisional_live_encoder_derived"
SCAN_TURN_SPEED_DPS = 250
SCAN_TURN_REFERENCE_BODY_MDEG = 90000
SCAN_TURN_REFERENCE_ENCODER_DEGREES = 682
SCAN_TURN_REFERENCE_DURATION_MS = 2560
SCAN_TURN_MAX_SLICE_DURATION_MS = 800
SCAN_TURN_MAX_SIDE_DIVERGENCE_DEGREES = 80
SCAN_SAMPLE_COUNT = 5
SCAN_SAMPLE_FILTER_WINDOW = 3
SCAN_SAMPLE_INTERVAL_MS = 30
SCAN_SAMPLE_SETTLED_DURATION_MS = (
    (SCAN_SAMPLE_COUNT - 1) * SCAN_SAMPLE_INTERVAL_MS
)


def _rounded_ratio(numerator, denominator):
    return int((numerator + denominator // 2) // denominator)


def _slice_durations(total_duration_ms):
    # Split the calibrated total into balanced slices instead of emitting a
    # tiny final pulse (for example 800 + 53 ms for 30 degrees).  The EV3's
    # regulated motors may spend most of such a tail leaving rest, even though
    # the preceding slice already established useful encoder motion.
    count = max(
        1,
        (
            total_duration_ms
            + SCAN_TURN_MAX_SLICE_DURATION_MS
            - 1
        ) // SCAN_TURN_MAX_SLICE_DURATION_MS,
    )
    base, extra = divmod(total_duration_ms, count)
    return [base + (1 if index < extra else 0) for index in range(count)]


def scan_turn_spec(relative_delta_mdeg):
    """Return one immutable-by-convention deterministic scan-turn profile."""
    if (
        isinstance(relative_delta_mdeg, bool)
        or not isinstance(relative_delta_mdeg, int)
        or relative_delta_mdeg
        not in SCAN_TURN_ALLOWED_DELTAS_MDEG
    ):
        raise ValueError("relative scan turn is not allowed")
    magnitude = abs(relative_delta_mdeg)
    duration_ms = _rounded_ratio(
        SCAN_TURN_REFERENCE_DURATION_MS * magnitude,
        SCAN_TURN_REFERENCE_BODY_MDEG,
    )
    target_encoder_degrees = _rounded_ratio(
        SCAN_TURN_REFERENCE_ENCODER_DEGREES * magnitude,
        SCAN_TURN_REFERENCE_BODY_MDEG,
    )
    durations = _slice_durations(duration_ms)
    direction = 1 if relative_delta_mdeg > 0 else -1
    return {
        "relative_delta_mdeg": relative_delta_mdeg,
        "left_speed_dps": -SCAN_TURN_SPEED_DPS * direction,
        "right_speed_dps": SCAN_TURN_SPEED_DPS * direction,
        "slice_durations_ms": durations,
        "slice_count": len(durations),
        "total_duration_ms": duration_ms,
        "target_mean_abs_encoder_degrees": target_encoder_degrees,
        "calibration": SCAN_TURN_CALIBRATION,
        "profile_id": SCAN_TURN_PROFILE_ID,
    }


def scan_turn_profile():
    """Describe the complete provisional scan calibration on the wire."""
    return {
        "profile_id": SCAN_TURN_PROFILE_ID,
        "calibration": SCAN_TURN_CALIBRATION,
        "allowed_relative_deltas_mdeg": list(
            SCAN_TURN_ALLOWED_DELTAS_MDEG
        ),
        "reference_body_turn_mdeg": SCAN_TURN_REFERENCE_BODY_MDEG,
        "reference_mean_abs_encoder_degrees": (
            SCAN_TURN_REFERENCE_ENCODER_DEGREES
        ),
        "reference_duration_ms": SCAN_TURN_REFERENCE_DURATION_MS,
        "speed_dps": SCAN_TURN_SPEED_DPS,
        "max_slice_duration_ms": SCAN_TURN_MAX_SLICE_DURATION_MS,
        "max_side_divergence_degrees": (
            SCAN_TURN_MAX_SIDE_DIVERGENCE_DEGREES
        ),
        "turns": [
            scan_turn_spec(value)
            for value in SCAN_TURN_ALLOWED_DELTAS_MDEG
        ],
    }


def scan_sample_profile():
    """Describe stopped-at-bearing IR sampling on the wire."""
    return {
        "sample_count": SCAN_SAMPLE_COUNT,
        "filter_window_samples": SCAN_SAMPLE_FILTER_WINDOW,
        "sample_interval_ms": SCAN_SAMPLE_INTERVAL_MS,
        "settled_duration_ms": SCAN_SAMPLE_SETTLED_DURATION_MS,
        "motors_stopped_before_sampling": True,
        "filter_history_reset_before_sampling": True,
    }

ACTION_SPECS = {
    "ADVANCE": {
        # The assembled EV3RSTORM intermittently fails to leave one motor
        # phase at the old 250 dps command.  A short high-speed launch keeps
        # approximately the same nominal 200 encoder-degree travel while
        # giving the regulated motor controller substantially more starting
        # authority and shortening each closed-loop pulse.
        "left_speed_dps": 800,
        "right_speed_dps": 800,
        "slice_durations_ms": [250],
        "slice_count": 1,
        "total_duration_ms": 250,
        "estimated_body_turn_degrees": None,
        "target_mean_abs_encoder_degrees": None,
        "calibration_evidence": None,
        "calibration": "not_applicable",
    },
    "REVERSE": {
        "left_speed_dps": -800,
        "right_speed_dps": -800,
        "slice_durations_ms": [250],
        "slice_count": 1,
        "total_duration_ms": 250,
        "estimated_body_turn_degrees": None,
        "target_mean_abs_encoder_degrees": None,
        "calibration_evidence": None,
        "calibration": "not_applicable",
    },
    # The checked-in EV3RSTORM calibration records about 682 encoder degrees
    # for a visually estimated 90 degree body turn.  A later live 250 dps
    # segment produced a mean 214.5 encoder degrees in 800 ms.  Three full
    # slices plus one 160 ms correction target that same encoder travel while
    # remaining within the per-command 800 ms limit.  This is provisional
    # dead reckoning, not exact pose.
    "TURN_LEFT_90": {
        "left_speed_dps": -250,
        "right_speed_dps": 250,
        "slice_durations_ms": [800, 800, 800, 160],
        "slice_count": 4,
        "total_duration_ms": 2560,
        "estimated_body_turn_degrees": 90,
        "target_mean_abs_encoder_degrees": 682,
        "calibration_evidence": {
            "source_action": "live_turn_left_segment",
            "source_speed_dps": 250,
            "source_duration_ms": 800,
            "source_left_encoder_delta_degrees": -210,
            "source_right_encoder_delta_degrees": 219,
            "source_mean_abs_encoder_delta_degrees": 214.5,
            "right_turn_is_mirrored": False,
        },
        "calibration": "provisional_live_encoder_derived",
    },
    "TURN_RIGHT_90": {
        "left_speed_dps": 250,
        "right_speed_dps": -250,
        "slice_durations_ms": [800, 800, 800, 160],
        "slice_count": 4,
        "total_duration_ms": 2560,
        "estimated_body_turn_degrees": -90,
        "target_mean_abs_encoder_degrees": 682,
        "calibration_evidence": {
            "source_action": "live_turn_left_segment",
            "source_speed_dps": 250,
            "source_duration_ms": 800,
            "source_left_encoder_delta_degrees": -210,
            "source_right_encoder_delta_degrees": 219,
            "source_mean_abs_encoder_delta_degrees": 214.5,
            "right_turn_is_mirrored": True,
        },
        "calibration": "provisional_live_encoder_derived",
    },
}


def validate_action_specs(owner):
    """Validate every fixed action against the active robot configuration."""
    for spec in ACTION_SPECS.values():
        durations = spec["slice_durations_ms"]
        if (
            not isinstance(durations, list)
            or not durations
            or spec["slice_count"] != len(durations)
            or spec["total_duration_ms"] != sum(durations)
        ):
            raise ValueError(
                "A fixed action has inconsistent slice metadata"
            )
        for duration_ms in durations:
            owner.validate_drive(
                spec["left_speed_dps"],
                spec["right_speed_dps"],
                duration_ms,
            )
    for relative_delta_mdeg in SCAN_TURN_ALLOWED_DELTAS_MDEG:
        spec = scan_turn_spec(relative_delta_mdeg)
        for duration_ms in spec["slice_durations_ms"]:
            owner.validate_drive(
                spec["left_speed_dps"],
                spec["right_speed_dps"],
                duration_ms,
            )
