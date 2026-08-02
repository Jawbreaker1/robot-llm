"""Bounded, atomic persistence for physical local-odometry navigation state."""

from copy import deepcopy
import json
import os
from pathlib import Path
import tempfile
import time
import uuid
from typing import Callable, Mapping, Optional

from .active_ir_scan_contract import ActiveIrScanResult
from .physical_navigation_contract import (
    MOTION_ACTIONS,
    PhysicalNavigationContractError,
    json_bytes,
    strict_json_loads,
    validate_observation,
)
from .physical_odometry import (
    DriveMotorRoles,
    OdometryCalibration,
    PhysicalPose,
    apply_verified_motion,
    verified_motion_from_result,
)
from .provisional_hazard_map import (
    HazardMapCalibration,
    ProvisionalHazard,
    ProvisionalHazardMap,
)


LEGACY_MEMORY_SCHEMA = "robot-physical-navigation-memory/v1"
MEMORY_SCHEMA = "robot-physical-navigation-memory/v2"
MAX_MEMORY_BYTES = 2 * 1024 * 1024


class NavigationMemoryError(RuntimeError):
    pass


def _motor_positions(
    observation: Mapping[str, object],
    drive_roles: DriveMotorRoles,
) -> Mapping[str, int]:
    allowed = frozenset((drive_roles.left, drive_roles.right))
    return {
        motor["role"]: motor["position"]
        for motor in observation["motors"]
        if motor["role"] in allowed
    }


class NavigationMemoryStore:
    """Own pose, provisional hazards, motor anchors, and atomic persistence."""

    def __init__(
        self,
        *,
        path: Path,
        robot_id: str,
        controller_instance_id: str,
        frame_id: str,
        generation_id: str,
        pose: PhysicalPose,
        hazard_map: ProvisionalHazardMap,
        drive_roles: Optional[DriveMotorRoles] = None,
        motor_positions: Optional[Mapping[str, int]] = None,
        localization_valid: bool = True,
        localization_error: Optional[str] = None,
        updated_at_ms: int = 0,
        odometry_calibration: OdometryCalibration = OdometryCalibration(),
    ):
        self.path = Path(path).expanduser().resolve()
        if self.path.name in ("", ".", ".."):
            raise NavigationMemoryError("navigation memory path is invalid")
        if (
            not isinstance(robot_id, str)
            or not robot_id
            or not isinstance(controller_instance_id, str)
            or not controller_instance_id
            or not isinstance(frame_id, str)
            or not frame_id
            or not isinstance(generation_id, str)
            or not generation_id
        ):
            raise NavigationMemoryError("navigation identity is invalid")
        if hazard_map.frame_id != frame_id:
            raise NavigationMemoryError("hazard map frame mismatch")
        if hazard_map.map_generation_id != generation_id:
            raise NavigationMemoryError("hazard map generation mismatch")
        if type(localization_valid) is not bool:
            raise NavigationMemoryError("localization state is invalid")
        if localization_valid and localization_error is not None:
            raise NavigationMemoryError("valid localization has an error")
        if not localization_valid and (
            not isinstance(localization_error, str) or not localization_error
        ):
            raise NavigationMemoryError("invalid localization needs a reason")
        self.robot_id = robot_id
        self.controller_instance_id = controller_instance_id
        self.frame_id = frame_id
        self.generation_id = generation_id
        self.pose = pose
        self.hazard_map = hazard_map
        if drive_roles is not None and not isinstance(
            drive_roles,
            DriveMotorRoles,
        ):
            raise NavigationMemoryError("drive motor roles are invalid")
        self.drive_roles = drive_roles
        self.motor_positions = dict(motor_positions or {})
        self.localization_valid = localization_valid
        self.localization_error = localization_error
        self.updated_at_ms = int(updated_at_ms)
        self.odometry_calibration = odometry_calibration

    @classmethod
    def load(
        cls,
        *,
        path,
        robot_id: str,
        controller_instance_id: str,
        reset: bool = False,
        clock_ms: Callable[[], int] = lambda: int(time.time() * 1000),
        uuid_factory: Callable[[], uuid.UUID] = uuid.uuid4,
        odometry_calibration: OdometryCalibration = OdometryCalibration(),
        hazard_calibration: HazardMapCalibration = HazardMapCalibration(),
    ):
        resolved = Path(path).expanduser().resolve()
        if reset or not resolved.exists():
            generation = "mapgen-{}".format(uuid_factory().hex)
            frame = "{}-local-odometry-{}".format(
                robot_id,
                generation[-12:],
            )
            return cls(
                path=resolved,
                robot_id=robot_id,
                controller_instance_id=controller_instance_id,
                frame_id=frame,
                generation_id=generation,
                pose=PhysicalPose(),
                hazard_map=ProvisionalHazardMap(
                    frame_id=frame,
                    map_generation_id=generation,
                    calibration=hazard_calibration,
                ),
                updated_at_ms=clock_ms(),
                odometry_calibration=odometry_calibration,
            )
        try:
            size = resolved.stat().st_size
            if size <= 0 or size > MAX_MEMORY_BYTES:
                raise NavigationMemoryError(
                    "navigation memory file size is invalid"
                )
            raw = resolved.read_bytes()
        except OSError as error:
            raise NavigationMemoryError(
                "navigation memory could not be read: {}".format(error)
            ) from error
        try:
            value = strict_json_loads(raw, MAX_MEMORY_BYTES)
            return cls._from_dict(
                value,
                path=resolved,
                expected_robot_id=robot_id,
                expected_controller_instance_id=controller_instance_id,
                odometry_calibration=odometry_calibration,
                hazard_calibration=hazard_calibration,
            )
        except (PhysicalNavigationContractError, TypeError, ValueError) as error:
            raise NavigationMemoryError(
                "navigation memory is invalid: {}".format(error)
            ) from error

    @classmethod
    def _from_dict(
        cls,
        value,
        *,
        path,
        expected_robot_id,
        expected_controller_instance_id,
        odometry_calibration,
        hazard_calibration,
    ):
        legacy_fields = {
            "schema",
            "robot_id",
            "controller_instance_id",
            "frame_id",
            "generation_id",
            "updated_at_ms",
            "pose",
            "map_revision",
            "hazards",
            "motor_positions",
            "drive_roles",
            "localization_valid",
            "localization_error",
        }
        initial_v2_fields = legacy_fields | {
            "hazards_evicted",
            "hazards_eviction_reason",
        }
        current_fields = initial_v2_fields | {
            "scan_attempts_evicted",
            "scan_attempts_eviction_reason",
        }
        if not isinstance(value, dict):
            raise ValueError("memory fields are invalid")
        schema = value.get("schema")
        if schema == LEGACY_MEMORY_SCHEMA:
            expected_fields = legacy_fields
        elif (
            schema == MEMORY_SCHEMA
            and set(value) in (initial_v2_fields, current_fields)
        ):
            expected_fields = set(value)
        else:
            expected_fields = None
        if expected_fields is None or set(value) != expected_fields:
            raise ValueError("memory fields are invalid")
        if (
            value["robot_id"] != expected_robot_id
            or value["controller_instance_id"]
            != expected_controller_instance_id
        ):
            raise ValueError("memory identity does not match this robot")
        hazards = value["hazards"]
        if not isinstance(hazards, list):
            raise ValueError("memory hazards are invalid")
        positions = value["motor_positions"]
        if (
            not isinstance(positions, dict)
            or any(
                not isinstance(role, str)
                or isinstance(position, bool)
                or not isinstance(position, int)
                for role, position in positions.items()
            )
        ):
            raise ValueError("memory motor anchors are invalid")
        raw_roles = value["drive_roles"]
        if raw_roles is None:
            drive_roles = None
        elif (
            isinstance(raw_roles, dict)
            and set(raw_roles) == {"left", "right"}
        ):
            drive_roles = DriveMotorRoles(
                left=raw_roles["left"],
                right=raw_roles["right"],
            )
        else:
            raise ValueError("memory drive role mapping is invalid")
        hazard_map = ProvisionalHazardMap(
            frame_id=value["frame_id"],
            map_generation_id=value["generation_id"],
            hazards=tuple(
                ProvisionalHazard.from_dict(item) for item in hazards
            ),
            revision=value["map_revision"],
            calibration=hazard_calibration,
            hazards_evicted=value.get("hazards_evicted", 0),
            hazards_eviction_reason=value.get(
                "hazards_eviction_reason"
            ),
            scan_attempts_evicted=value.get(
                "scan_attempts_evicted"
            ),
            scan_attempts_eviction_reason=value.get(
                "scan_attempts_eviction_reason"
            ),
        )
        return cls(
            path=path,
            robot_id=value["robot_id"],
            controller_instance_id=value["controller_instance_id"],
            frame_id=value["frame_id"],
            generation_id=value["generation_id"],
            pose=PhysicalPose.from_mapping(value["pose"]),
            hazard_map=hazard_map,
            drive_roles=drive_roles,
            motor_positions=positions,
            localization_valid=value["localization_valid"],
            localization_error=value["localization_error"],
            updated_at_ms=value["updated_at_ms"],
            odometry_calibration=odometry_calibration,
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": MEMORY_SCHEMA,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "frame_id": self.frame_id,
            "generation_id": self.generation_id,
            "updated_at_ms": self.updated_at_ms,
            "pose": self.pose.to_dict(),
            "map_revision": self.hazard_map.revision,
            "hazards_evicted": self.hazard_map.hazards_evicted,
            "hazards_eviction_reason": (
                self.hazard_map.hazards_eviction_reason
            ),
            "scan_attempts_evicted": (
                self.hazard_map.scan_attempts_evicted
            ),
            "scan_attempts_eviction_reason": (
                self.hazard_map.scan_attempts_eviction_reason
            ),
            "hazards": [
                item.to_dict() for item in self.hazard_map.hazards
            ],
            "motor_positions": dict(self.motor_positions),
            "drive_roles": (
                None
                if self.drive_roles is None
                else self.drive_roles.to_dict()
            ),
            "localization_valid": self.localization_valid,
            "localization_error": self.localization_error,
        }

    def save(self) -> None:
        raw = json_bytes(self.to_dict())
        if len(raw) > MAX_MEMORY_BYTES:
            raise NavigationMemoryError(
                "navigation memory exceeded its bounded size"
            )
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            dir=str(self.path.parent),
            prefix=".{}.".format(self.path.name),
        )
        try:
            os.fchmod(descriptor, 0o600)
            with os.fdopen(descriptor, "wb") as handle:
                descriptor = None
                handle.write(raw)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.path)
        except BaseException:
            if descriptor is not None:
                os.close(descriptor)
            try:
                os.unlink(temporary)
            except OSError:
                pass
            raise

    def begin_episode(
        self,
        observation: Mapping[str, object],
        now_ms: int,
    ) -> None:
        checked = validate_observation(observation)
        if self.drive_roles is None:
            raise NavigationMemoryError(
                "drive motor roles were not bound from worker description"
            )
        current = _motor_positions(checked, self.drive_roles)
        if set(current) != {self.drive_roles.left, self.drive_roles.right}:
            raise NavigationMemoryError(
                "worker observation lacks configured drive roles"
            )
        if self.motor_positions and current != self.motor_positions:
            self.invalidate_localization(
                "Drive encoders changed while the host was not observing",
                now_ms,
            )
            raise NavigationMemoryError(
                "persisted motor anchor no longer matches the robot"
            )
        self.motor_positions = dict(current)
        self.updated_at_ms = max(self.updated_at_ms, int(now_ms))
        self.hazard_map.record_observation(self.pose, checked, int(now_ms))
        self.save()

    def invalidate_localization(self, reason: str, now_ms: int) -> None:
        if not isinstance(reason, str) or not reason:
            raise NavigationMemoryError(
                "localization invalidation requires a reason"
            )
        self.localization_valid = False
        self.localization_error = reason
        self.updated_at_ms = max(self.updated_at_ms, int(now_ms))
        self.hazard_map.revision += 1
        self.save()

    def bind_drive_roles(self, drive_roles: DriveMotorRoles) -> None:
        """Bind once to the worker's explicit, validated drive geometry."""

        if not isinstance(drive_roles, DriveMotorRoles):
            raise NavigationMemoryError("drive motor roles are invalid")
        if self.drive_roles is not None and self.drive_roles != drive_roles:
            raise NavigationMemoryError(
                "persisted drive roles do not match the EV3 worker"
            )
        self.drive_roles = drive_roles

    def ingest_stationary_observation(
        self,
        observation: Mapping[str, object],
        now_ms: int,
    ) -> Optional[ProvisionalHazard]:
        checked = validate_observation(observation)
        if self.drive_roles is None:
            raise NavigationMemoryError("drive motor roles are not bound")
        positions = _motor_positions(checked, self.drive_roles)
        if set(positions) != {self.drive_roles.left, self.drive_roles.right}:
            raise NavigationMemoryError(
                "worker observation lacks configured drive roles"
            )
        if self.motor_positions and positions != self.motor_positions:
            self.invalidate_localization(
                "Drive encoders changed during a stationary observation",
                now_ms,
            )
            raise NavigationMemoryError(
                "unobserved physical motion invalidated localization"
            )
        self.motor_positions = dict(positions)
        hazard = self.hazard_map.record_observation(
            self.pose,
            checked,
            int(now_ms),
        )
        self.updated_at_ms = max(self.updated_at_ms, int(now_ms))
        self.save()
        return hazard

    def ingest_verified_scan_completion(
        self,
        observation: Mapping[str, object],
        scan_result: ActiveIrScanResult,
        now_ms: int,
    ) -> Optional[ProvisionalHazard]:
        """Re-anchor drive encoders after a verified closed scan arc."""
        checked = validate_observation(observation)
        if self.drive_roles is None:
            raise NavigationMemoryError("drive motor roles are not bound")
        if (
            not isinstance(scan_result, ActiveIrScanResult)
            or scan_result.restored_start_heading is not True
            or scan_result.stop_confirmed is not True
        ):
            self.invalidate_localization(
                "Active scan did not verify restoration and motor stop",
                now_ms,
            )
            raise NavigationMemoryError(
                "active scan cannot safely re-anchor localization"
            )
        positions = _motor_positions(checked, self.drive_roles)
        if set(positions) != {self.drive_roles.left, self.drive_roles.right}:
            self.invalidate_localization(
                "Post-scan observation lacks configured drive roles",
                now_ms,
            )
            raise NavigationMemoryError(
                "post-scan observation lacks configured drive roles"
            )
        # A closed scan arc intentionally changes both wheel encoders while
        # restoring the body heading. Preserve the world pose and adopt only
        # the freshly observed encoder anchors.
        self.motor_positions = dict(positions)
        hazard = self.hazard_map.record_observation(
            self.pose,
            checked,
            int(now_ms),
        )
        self.updated_at_ms = max(self.updated_at_ms, int(now_ms))
        self.save()
        return hazard

    def apply_motion_result(
        self,
        action: str,
        result: Mapping[str, object],
        now_ms: int,
    ) -> Optional[ProvisionalHazard]:
        if action not in MOTION_ACTIONS:
            raise NavigationMemoryError("action is not motion")
        try:
            if self.drive_roles is None:
                raise NavigationMemoryError(
                    "drive motor roles are not bound"
                )
            motion = verified_motion_from_result(
                action,
                result,
                self.drive_roles,
            )
            checked = validate_observation(result["observation"])
            self.pose = apply_verified_motion(
                self.pose,
                motion,
                self.odometry_calibration,
            )
        except PhysicalNavigationContractError as error:
            self.invalidate_localization(
                "Unverifiable physical motion: {}".format(error.code),
                now_ms,
            )
            raise NavigationMemoryError(
                "motion could not be localized: {}".format(error)
            ) from error
        positions = _motor_positions(checked, self.drive_roles)
        if set(positions) != {self.drive_roles.left, self.drive_roles.right}:
            self.invalidate_localization(
                "Motion result lacks configured drive motor roles",
                now_ms,
            )
            raise NavigationMemoryError(
                "worker motion observation lacks configured drive roles"
            )
        self.motor_positions = dict(positions)
        hazard = self.hazard_map.record_observation(
            self.pose,
            checked,
            int(now_ms),
        )
        self.updated_at_ms = max(self.updated_at_ms, int(now_ms))
        self.save()
        return hazard

    def context(self) -> Mapping[str, object]:
        value = dict(self.hazard_map.context())
        for hypothesis in value["navigation_hazard_hypotheses"]:
            route_evidence = self.hazard_map.route_evidence(
                hypothesis["hypothesis_id"],
                pose=self.pose,
            )
            hypothesis["route_commitment_ready"] = (
                route_evidence["ready"]
                or route_evidence["best_effort_ready"]
            )
            hypothesis["route_commitment_evidence_strength"] = (
                route_evidence["strength"]
            )
            hypothesis["route_evidence"] = route_evidence
        value.update(
            {
                "robot_id": self.robot_id,
                "controller_instance_id": self.controller_instance_id,
                "pose": self.pose.to_dict(),
                "drive_motor_roles": (
                    None
                    if self.drive_roles is None
                    else self.drive_roles.to_dict()
                ),
                "localization_valid": self.localization_valid,
                "localization_error": self.localization_error,
            }
        )
        return deepcopy(value)
