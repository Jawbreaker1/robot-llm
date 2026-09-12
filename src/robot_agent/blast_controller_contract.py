"""Shared BLAST controller commands, timing budgets and error contract."""

from __future__ import annotations

from dataclasses import dataclass


SNAPSHOT_SCHEMA = "controller-runtime-observation/v1"


ROBOT_ID = "blast-01"


CONTROLLER_ID = "blast-01.hub"


DEFAULT_POLL_INTERVAL_SECONDS = 1.0


DEFAULT_RECONNECT_INTERVAL_SECONDS = 3.0


DISCONNECT_TIMEOUT_SECONDS = 3.0


COMMAND_TIMEOUT_SECONDS = 15.0


INTERNAL_COMMAND_TIMEOUT_SECONDS = 12.0


SCAN_COMMAND_TIMEOUT_SECONDS = 90.0


SCAN_INTERNAL_COMMAND_TIMEOUT_SECONDS = 75.0


MIN_COMMAND_BUDGET_SECONDS = 2.0


COMMAND_RESPONSE_MARGIN_SECONDS = 0.25


MOTION_TIMEOUT_SECONDS = 4.0


MOTION_POLL_INTERVAL_SECONDS = 0.05


POST_MOTION_SETTLE_TIMEOUT_SECONDS = 1.5


SCAN_POST_MOTION_SETTLE_TIMEOUT_SECONDS = 3.0


SCAN_PULSE_POST_MOTION_SETTLE_TIMEOUT_SECONDS = 1.5


# Confirm motor idle across a fresh read, not millimetre/degree agreement.
POST_MOTION_IDLE_SAMPLE_COUNT = 2


COMMAND_RESULT_SCHEMA = "controller-command-result/v1"


SAMPLED_AUDIO_RESULT_SCHEMA = "controller-sampled-audio-result/v1"


SAMPLED_AUDIO_TIMEOUT_SECONDS = 60.0


SAMPLED_AUDIO_MAX_TOTAL_SECONDS = 15.0 * 60.0


SCAN_COMMAND = "scan_front_arc"


SURROUNDINGS_SCAN_COMMAND = "scan_surroundings"


SETTLED_OBSERVATION_COMMAND = "observe_settled"


# Measured closed reference for this BLAST linkage, not the last arbitrary pose.
CLAW_CLOSED_MOTOR_ANGLE_DEG = 200


# Offsets from the closed reference: actual targets are 200, 325, 200, 325, 200.
DOUBLE_CLAW_POSES = (
    ("claw", 0), ("claw", 125), ("claw", 0), ("claw", 125), ("claw", 0),
)


# Body offsets remain relative to the existing navigation sensor reference.
GESTURE_POSES = {
    "claw_snap": DOUBLE_CLAW_POSES,
    "arm_wave": (("body", -600), ("body", 0)),
    "claw_flourish": (("body", -600),) + DOUBLE_CLAW_POSES + (("body", 0),),
}


COMMANDS = {
    "drive_forward": ("drive_pulse", "forward"),
    "drive_reverse": ("drive_pulse", "reverse"),
    "turn_left": ("turn_pulse", "left"),
    "turn_right": ("turn_pulse", "right"),
    "turn_left_trim": ("turn_trim_pulse", "left"),
    "turn_right_trim": ("turn_trim_pulse", "right"),
    "claw_open": ("claw_pulse", "open"),
    "claw_close": ("claw_pulse", "close"),
    "body_left": ("body_pulse", "left"),
    "body_right": ("body_pulse", "right"),
    SCAN_COMMAND: (None, None),
    SURROUNDINGS_SCAN_COMMAND: (None, None),
    SETTLED_OBSERVATION_COMMAND: (None, None),
    "stop": ("stop", None),
    **{gesture: (None, None) for gesture in GESTURE_POSES},
    **{"face_" + face: ("show_face", face) for face in (
        "idle", "neutral", "happy", "frustrated", "curious", "surprised", "angry",
    )},
}


NAVIGATION_MOTION_COMMANDS = {
    "drive_forward",
    "drive_reverse",
    "turn_left",
    "turn_right",
    "turn_left_trim",
    "turn_right_trim",
}


@dataclass(frozen=True)
class _BlastNoReturnScanPermit:
    """One short-lived scan permit bound to an exact encoder anchor."""

    runtime_generation: int
    expires_at_monotonic_ns: int
    drive_angles_deg: tuple[float, float]
    allow_no_return: bool = True


class BlastControllerError(RuntimeError):
    """A bounded BLAST command could not be accepted or verified."""

    def __init__(
        self, code: str, message: str, *, motion_started=None,
        evidence_uncertain=False,
    ) -> None:
        self.code = code
        self.motion_started = motion_started
        self.evidence_uncertain = evidence_uncertain is True
        super().__init__(message)
