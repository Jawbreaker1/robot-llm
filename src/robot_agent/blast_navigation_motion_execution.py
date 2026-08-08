"""Execute one calibrated BLAST motion action and retain encoder odometry."""

from collections.abc import Mapping
from copy import deepcopy
from typing import NamedTuple

from .blast_navigation_action_profile import BLAST_NAVIGATION_COMMANDS
from .blast_navigation_calibration import BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
from .blast_navigation_motion_result import build_blast_navigation_motion_result
from .blast_observation_monitor import (
    COMMAND_RESULT_SCHEMA,
    CONTROLLER_ID,
    ROBOT_ID,
    SCAN_COMMAND,
    SCAN_RESTORATION_TOLERANCE_DEG,
    BlastControllerError,
)
from .physical_navigation_contract import PhysicalNavigationContractError
from .physical_odometry import (
    PhysicalPose, VerifiedMotion, apply_verified_motion,
    verified_motion_from_result,
)


_DRIVE_ROLES = ("left_drive", "right_drive")
MAX_RESTORED_SCAN_ENCODER_RESIDUE_DEGREES = int(round(
    SCAN_RESTORATION_TOLERANCE_DEG * 1_000 /
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
    .turn_mdeg_per_opposed_encoder_degree
))


def _fail(code: str, message: str) -> None:
    raise PhysicalNavigationContractError(code, message)


def _encoder_angles(observation: Mapping[str, object]):
    if not isinstance(observation, Mapping):
        _fail("blast_navigation_observation_invalid",
              "BLAST navigation requires a controller observation")
    angles = observation.get("motor_angles_deg")
    if (
        not isinstance(angles, Mapping)
        or any(type(angles.get(role)) is not int for role in _DRIVE_ROLES)
    ):
        _fail("blast_navigation_encoder_anchor_invalid",
              "BLAST navigation lacks exact drive encoder anchors")
    return {role: angles[role] for role in _DRIVE_ROLES}


def _canonical_observation(observation: Mapping[str, object]):
    angles = _encoder_angles(observation)
    return {
        "motors": [
            {"role": role, "position": angles[role], "state": ""}
            for role in _DRIVE_ROLES
        ],
        "last_outcome": {},
    }


class BlastNavigationMotionExecution(NamedTuple):
    """One semantic action, its native receipts, and resulting local pose."""

    controller_results: tuple[Mapping[str, object], ...]
    motion: VerifiedMotion
    pose: PhysicalPose


class BlastNavigationMotionExecutor:
    """Serially execute calibrated actions from one correlated encoder anchor."""

    def __init__(self, *, controller, initial_observation) -> None:
        if (
            not callable(getattr(controller, "command", None))
            or not callable(getattr(controller, "snapshot", None))
        ):
            raise ValueError("BLAST navigation controller is invalid")
        self._controller = controller
        self._expected_start_angles = _encoder_angles(initial_observation)
        self._pose = PhysicalPose()
        self._localization_valid = True

    @property
    def pose(self) -> PhysicalPose:
        return self._pose

    @property
    def expected_start_angles(self):
        return dict(self._expected_start_angles)

    @property
    def localization_valid(self) -> bool:
        return self._localization_valid

    def _invalidate(self, code: str, message: str) -> None:
        self._localization_valid = False
        _fail(code, message)

    def reanchor_after_restored_scan(self, command_result) -> bool:
        """Pose-preserving approximation for one verified restored scan."""
        if not self._localization_valid:
            _fail("blast_navigation_localization_invalid",
                  "BLAST encoder localization must be restarted")
        if (
            not isinstance(command_result, Mapping)
            or command_result.get("schema") != COMMAND_RESULT_SCHEMA
            or command_result.get("robot_id") != ROBOT_ID
            or command_result.get("controller_id") != CONTROLLER_ID
            or command_result.get("command") != SCAN_COMMAND
            or command_result.get("accepted") is not True
            or command_result.get("completed") is not True
        ):
            self._invalidate("blast_scan_result_invalid",
                             "BLAST returned an invalid scan result")
        scan = command_result.get("scan")
        observation = command_result.get("observation")
        if (
            not isinstance(scan, Mapping)
            or scan.get("restoration_verified") is not True
            or not isinstance(observation, Mapping)
            or observation.get("motion_active") is not False
        ):
            self._invalidate("blast_scan_restoration_unverified",
                             "BLAST scan did not verify its restored heading")
        try:
            angles = _encoder_angles(observation)
        except Exception:
            self._localization_valid = False
            raise
        if any(
            abs(angles[role] - self._expected_start_angles[role])
            > MAX_RESTORED_SCAN_ENCODER_RESIDUE_DEGREES
            for role in _DRIVE_ROLES
        ):
            self._invalidate("blast_scan_encoder_residue_excessive",
                             "BLAST scan left excessive encoder residue")
        self._expected_start_angles = angles
        return True

    def execute(self, action: str, *, cancel_requested=None):
        if action not in BLAST_NAVIGATION_COMMANDS:
            _fail("invalid_blast_motion_action",
                  "BLAST cannot execute this semantic motion action")
        if cancel_requested is not None and not callable(cancel_requested):
            raise ValueError("BLAST cancellation callback is invalid")
        if not self._localization_valid:
            _fail("blast_navigation_localization_invalid",
                  "BLAST encoder localization must be restarted")
        snapshot = self._controller.snapshot()
        observed_start = _encoder_angles(
            snapshot.get("observation")
            if isinstance(snapshot, Mapping) else None
        )
        if observed_start != self._expected_start_angles:
            self._invalidate(
                "blast_motion_slice_discontinuous",
                "BLAST encoders changed outside a verified motion action",
            )

        results = []
        command_attempted = False
        try:
            for command in BLAST_NAVIGATION_COMMANDS[action]:
                if cancel_requested is not None and cancel_requested():
                    if not results:
                        raise BlastControllerError(
                            "controller_command_interrupted",
                            "BLAST motion was cancelled before motor start",
                        )
                    break
                command_attempted = True
                command_result = self._controller.command(
                    command,
                    cancel_requested=cancel_requested,
                )
                if not isinstance(command_result, Mapping):
                    _fail("blast_command_result_invalid",
                          "BLAST returned an invalid command result")
                results.append(command_result)

            final_observation = results[-1].get("observation")
            final_angles = _encoder_angles(final_observation)
            canonical = build_blast_navigation_motion_result(
                action,
                results,
                expected_start_angles=self._expected_start_angles,
                canonical_observation=_canonical_observation(
                    final_observation
                ),
            )
            motion = verified_motion_from_result(action, canonical)
            pose = apply_verified_motion(
                self._pose,
                motion,
                BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
            )
        except Exception:
            if command_attempted:
                self._localization_valid = False
            raise

        # Only advance the trusted anchor after every boundary above passed.
        self._expected_start_angles = final_angles
        self._pose = pose
        return BlastNavigationMotionExecution(
            controller_results=tuple(deepcopy(results)),
            motion=motion,
            pose=pose,
        )
__all__ = ("MAX_RESTORED_SCAN_ENCODER_RESIDUE_DEGREES",
           "BlastNavigationMotionExecution", "BlastNavigationMotionExecutor")
