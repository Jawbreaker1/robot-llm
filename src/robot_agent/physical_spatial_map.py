"""Atomic read-only projection of authoritative physical navigation memory.

The EV3 IR-PROX value is reflection strength, not distance.  This bridge
therefore publishes local odometry and qualitative forward evidence only.  It
never creates metric cells, free-space rays, object surfaces, or identities
separate from the hypotheses already used by physical navigation.
"""

from copy import deepcopy
import threading
import time
from typing import Callable, Mapping

from .navigation_memory_store import NavigationMemoryStore
from .physical_navigation_contract import validate_observation
from .spatial_map_contract import (
    DASHBOARD_SPATIAL_MAP_SCHEMA,
    LOCAL_ODOMETRY,
    LOCAL_ODOMETRY_POSE,
    MAX_POSE_HISTORY,
    MAP_EMPTY,
    MAP_PROVISIONAL_IR,
    MAX_SPATIAL_SCAN_EVIDENCE,
    PHYSICAL_IR_REFLECTION,
    PROVISIONAL_QUALITATIVE,
    QUALITATIVE_FORWARD_ENVELOPE,
    SEMANTIC_UNKNOWN,
    SpatialCollisionGeometry,
    SpatialScanEvidence,
)


MAX_QUALITATIVE_OBSERVATIONS = 128
PHYSICAL_IR_CONFIDENCE_MILLI = 250
PHYSICAL_IR_TTL_MS = 5_000


def _unix_ms() -> int:
    return time.time_ns() // 1_000_000


def _identifier(name: str, value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 128
        or any(ord(character) < 32 for character in value)
    ):
        raise ValueError("{} is invalid".format(name))
    return value


def _empty_snapshot(robot_id: str, controller_instance_id: str):
    return {
        "schema": DASHBOARD_SPATIAL_MAP_SCHEMA,
        "read_only": True,
        "status": "unavailable",
        "reason_code": "no_physical_observations",
        "map_id": "{}-physical-map".format(robot_id),
        "robot_id": robot_id,
        "controller_instance_id": controller_instance_id,
        "frame_id": None,
        "frame_kind": LOCAL_ODOMETRY,
        "map_quality": MAP_EMPTY,
        "map_version": None,
        "based_on_state_version": None,
        "based_on_world_model_version": None,
        "resolution_mm": None,
        "capacity": 32,
        "cells_evicted": 0,
        "source_id": "physical-navigation-memory",
        "provenance": None,
        "bounds": None,
        "robot_pose": None,
        "pose_history": [],
        "pose_history_evicted": 0,
        "collision_geometry": None,
        "scan_evidence_history": [],
        "sensor_rays": [],
        "cells": [],
        "qualitative_observations": [],
        "object_hypotheses": [],
        "captured_at_unix_ms": None,
        "observed_at_unix_ms": None,
        "observed_age_ms": None,
        "age_ms": None,
    }


class PhysicalSpatialMapBridge:
    """Cache detached physical snapshots behind an observation-only API."""

    def __init__(
        self,
        *,
        robot_id: str,
        controller_instance_id: str,
        clock_ms: Callable[[], int] = _unix_ms,
    ):
        self.robot_id = _identifier("robot_id", robot_id)
        self.controller_instance_id = _identifier(
            "controller_instance_id",
            controller_instance_id,
        )
        if not callable(clock_ms):
            raise ValueError("physical map clock is invalid")
        self._clock_ms = clock_ms
        self._lock = threading.RLock()
        self._accepting = True
        self._generation_id = None
        self._frame_id = None
        self._generation_revision = None
        self._retired_generation_ids = set()
        self._retired_generation_keys = set()
        self._world_model_version = 0
        self._publication_sequence = 0
        self._last_captured_at_ms = 0
        self._qualitative = []
        self._pose_history = []
        self._pose_history_evicted = 0
        self._snapshot = _empty_snapshot(
            self.robot_id,
            self.controller_instance_id,
        )

    @staticmethod
    def _motor_position(observation, role: str) -> int:
        for motor in observation["motors"]:
            if motor["role"] == role:
                return motor["position"] * 1_000
        raise ValueError("physical map drive encoder is missing")

    @staticmethod
    def _motors_running(observation) -> bool:
        return any(
            token in {"running", "ramping"}
            for motor in observation["motors"]
            for token in motor["state"].split()
        )

    def _generation_changed(self, memory: NavigationMemoryStore) -> bool:
        return (
            self._generation_id,
            self._frame_id,
        ) != (
            memory.generation_id,
            memory.frame_id,
        )

    def _begin_generation(self, memory: NavigationMemoryStore) -> None:
        if self._generation_id is not None:
            self._retired_generation_keys.add((
                self._generation_id,
                self._frame_id,
            ))
            if self._generation_id != memory.generation_id:
                self._retired_generation_ids.add(self._generation_id)
        self._generation_id = memory.generation_id
        self._frame_id = memory.frame_id
        self._generation_revision = memory.hazard_map.revision
        self._world_model_version += 1
        self._qualitative.clear()
        self._pose_history.clear()
        self._pose_history_evicted = 0

    def _retain_pose(self, pose: Mapping[str, object]) -> None:
        """Retain changed valid poses without heuristic thresholds."""

        if self._pose_history:
            previous = self._pose_history[-1]
            if (
                pose["x_mm"],
                pose["y_mm"],
                pose["heading_mdeg"],
            ) == (
                previous["x_mm"],
                previous["y_mm"],
                previous["heading_mdeg"],
            ):
                return
        if len(self._pose_history) == MAX_POSE_HISTORY:
            del self._pose_history[0]
            self._pose_history_evicted += 1
        self._pose_history.append(dict(pose))

    def offer(
        self,
        *,
        memory: NavigationMemoryStore,
        observation: Mapping[str, object],
        episode_id: str,
        captured_at_ms: int,
    ) -> bool:
        """Publish one post-commit physical observation in bounded time."""

        if not isinstance(memory, NavigationMemoryStore):
            raise ValueError("physical map memory is invalid")
        if (
            memory.robot_id != self.robot_id
            or memory.controller_instance_id
            != self.controller_instance_id
        ):
            raise ValueError("physical map identity changed")
        _identifier("episode_id", episode_id)
        if (
            isinstance(captured_at_ms, bool)
            or not isinstance(captured_at_ms, int)
            or captured_at_ms < 0
        ):
            raise ValueError("physical map capture time is invalid")
        checked = validate_observation(dict(observation))
        if memory.drive_roles is None:
            raise ValueError("physical map drive roles are unavailable")
        # Validate all observation-derived metadata before mutating bridge
        # state. A malformed sample must remain safely retryable at the same
        # map revision.
        left_encoder_mdeg = self._motor_position(
            checked,
            memory.drive_roles.left,
        )
        right_encoder_mdeg = self._motor_position(
            checked,
            memory.drive_roles.right,
        )
        motors_running = self._motors_running(checked)
        collision_geometry = SpatialCollisionGeometry.from_mapping(
            memory.hazard_map.calibration.collision_geometry()
        ).to_dict()
        scan_evidence_history = sorted(
            (
                SpatialScanEvidence.from_navigation_evidence(
                    target_hypothesis_id=hazard.hypothesis_id,
                    frame_id=hazard.frame_id,
                    anchor_x_mm=hazard.anchor_x_mm,
                    anchor_y_mm=hazard.anchor_y_mm,
                    anchor_heading_mdeg=hazard.anchor_heading_mdeg,
                    attempt=attempt,
                ).to_dict()
                for hazard in memory.hazard_map.hazards
                for attempt in hazard.scan_evidence_history
            ),
            key=lambda item: (
                item["completed_at_unix_ms"],
                item["scan_id"],
            ),
        )[-MAX_SPATIAL_SCAN_EVIDENCE:]

        with self._lock:
            if not self._accepting:
                return False
            generation_changed = self._generation_changed(memory)
            if (
                memory.generation_id in self._retired_generation_ids
                or (
                    memory.generation_id,
                    memory.frame_id,
                ) in self._retired_generation_keys
            ):
                return False
            if captured_at_ms < self._last_captured_at_ms:
                return False
            if (
                not generation_changed
                and self._generation_revision is not None
                and memory.hazard_map.revision
                <= self._generation_revision
            ):
                return False
            if generation_changed:
                self._begin_generation(memory)
            else:
                self._generation_revision = memory.hazard_map.revision
            captured = captured_at_ms
            self._last_captured_at_ms = captured
            self._publication_sequence += 1
            state_version = self._publication_sequence
            world_version = self._world_model_version
            infrared = checked["infrared"]
            raw_ir = (
                infrared["raw"]
                if infrared["raw"] is not None
                else infrared["filtered"]
            )
            has_current_ir = raw_ir is not None
            relation = (
                "NEAR_OBSTACLE"
                if infrared["blocked"]
                else "NO_NEAR_REFLECTION"
            )
            if has_current_ir:
                self._qualitative.append({
                    "evidence_id": "physical-ir-{}-{}".format(
                        world_version,
                        state_version,
                    ),
                    "bearing": "FORWARD",
                    "relation": relation,
                    "raw_ir_proximity": raw_ir,
                    "confidence_milli": PHYSICAL_IR_CONFIDENCE_MILLI,
                    "source_id": PHYSICAL_IR_REFLECTION,
                    "provenance": "PROVISIONAL_IR",
                    "provisional": True,
                    "observed_at_unix_ms": captured,
                    "age_ms": 0,
                })
                self._qualitative = self._qualitative[
                    -MAX_QUALITATIVE_OBSERVATIONS:
                ]

            hazards = []
            for hazard in memory.hazard_map.hazards:
                hazards.append({
                    "hypothesis_id": hazard.hypothesis_id,
                    "x_mm": None,
                    "y_mm": None,
                    "label": SEMANTIC_UNKNOWN,
                    "bounds": None,
                    "anchor_pose": {
                        "x_mm": hazard.anchor_x_mm,
                        "y_mm": hazard.anchor_y_mm,
                        "heading_mdeg": hazard.anchor_heading_mdeg,
                    },
                    "geometry_kind": QUALITATIVE_FORWARD_ENVELOPE,
                    "bearing": "FORWARD",
                    "relation": "NEAR_OBSTACLE",
                    "evidence_count": hazard.evidence_count,
                    "confidence_milli": PHYSICAL_IR_CONFIDENCE_MILLI,
                    "source_id": PHYSICAL_IR_REFLECTION,
                    "provenance": "{} | {}".format(
                        LOCAL_ODOMETRY_POSE,
                        PHYSICAL_IR_REFLECTION,
                    ),
                    "provisional": True,
                    "quality": PROVISIONAL_QUALITATIVE,
                    "trusted_simulator_object_id": None,
                    "observed_at_unix_ms": hazard.last_seen_at_ms,
                    "age_ms": max(0, captured - hazard.last_seen_at_ms),
                    "scan_boundaries": (
                        None
                        if not hazard.bilateral_scan_complete
                        else {
                            "left_mdeg": hazard.scan_left_boundary_mdeg,
                            "right_mdeg": hazard.scan_right_boundary_mdeg,
                            "completed_at_unix_ms": (
                                hazard.scan_completed_at_ms
                            ),
                        }
                    ),
                })

            has_physical_evidence = bool(
                has_current_ir or self._qualitative or hazards
            )
            localization_valid = memory.localization_valid
            status = (
                "qualitative_only"
                if localization_valid and has_physical_evidence
                else "pose_only" if localization_valid else "degraded"
            )
            reason_code = (
                "provisional_ir_only"
                if status == "qualitative_only"
                else "pose_only"
                if status == "pose_only"
                else "physical_localization_invalid"
            )
            pose = None
            if localization_valid:
                pose = {
                    "x_mm": memory.pose.x_mm,
                    "y_mm": memory.pose.y_mm,
                    "heading_mdeg": memory.pose.heading_mdeg,
                    "frame_id": memory.frame_id,
                    "state_version": state_version,
                    "source_id": "navigation-pose",
                    "provenance": "LOCAL_ODOMETRY",
                    "observed_at_unix_ms": captured,
                    "age_ms": 0,
                }
                self._retain_pose(pose)
            sensor_rays = []
            if has_current_ir:
                sensor_rays.append({
                    "direction": "FORWARD",
                    "origin_x_mm": None,
                    "origin_y_mm": None,
                    "end_x_mm": None,
                    "end_y_mm": None,
                    "state": relation,
                    "confidence_milli": PHYSICAL_IR_CONFIDENCE_MILLI,
                    "source_id": PHYSICAL_IR_REFLECTION,
                    "provenance": "PROVISIONAL_IR",
                    "provisional": True,
                    "observed_at_unix_ms": captured,
                    "valid_until_unix_ms": captured + PHYSICAL_IR_TTL_MS,
                    "age_ms": 0,
                })

            self._snapshot = {
                "schema": DASHBOARD_SPATIAL_MAP_SCHEMA,
                "read_only": True,
                "status": status,
                "reason_code": reason_code,
                "map_id": "{}-physical-map".format(self.robot_id),
                "robot_id": self.robot_id,
                "controller_instance_id": self.controller_instance_id,
                "frame_id": memory.frame_id,
                "frame_kind": LOCAL_ODOMETRY,
                "map_quality": (
                    MAP_PROVISIONAL_IR
                    if has_physical_evidence
                    else MAP_EMPTY
                ),
                "map_version": memory.hazard_map.revision,
                "based_on_state_version": state_version,
                "based_on_world_model_version": world_version,
                "resolution_mm": None,
                "capacity": 32,
                "cells_evicted": 0,
                "source_id": "physical-navigation-memory",
                "provenance": (
                    "LOCAL_ODOMETRY + PROVISIONAL_IR"
                    if has_physical_evidence
                    else "LOCAL_ODOMETRY"
                ),
                "bounds": None,
                "robot_pose": pose,
                "pose_history": deepcopy(self._pose_history),
                "pose_history_evicted": self._pose_history_evicted,
                "collision_geometry": deepcopy(collision_geometry),
                "scan_evidence_history": deepcopy(
                    scan_evidence_history
                ),
                "sensor_rays": sensor_rays,
                "cells": [],
                "qualitative_observations": deepcopy(self._qualitative),
                "object_hypotheses": hazards,
                "captured_at_unix_ms": captured,
                "observed_at_unix_ms": captured,
                "observed_age_ms": 0,
                "age_ms": 0,
                "localization": {
                    "valid": localization_valid,
                    "error": memory.localization_error,
                },
                "drive_observation": {
                    "left_encoder_mdeg": left_encoder_mdeg,
                    "right_encoder_mdeg": right_encoder_mdeg,
                    "motors_running": motors_running,
                    "touch_pressed": checked["touch"]["pressed"],
                    "motion_fault_latched": checked["budgets"][
                        "motion_fault_latched"
                    ],
                },
            }
        return True

    def snapshot(self):
        """Return a detached snapshot with ages refreshed at read time."""

        with self._lock:
            value = deepcopy(self._snapshot)
        captured = value.get("captured_at_unix_ms")
        if not isinstance(captured, int):
            return value
        now = max(captured, int(self._clock_ms()))
        value["observed_age_ms"] = now - captured
        value["age_ms"] = now - captured
        for key in ("robot_pose",):
            item = value.get(key)
            if isinstance(item, dict):
                observed = item.get("observed_at_unix_ms")
                if isinstance(observed, int):
                    item["age_ms"] = max(0, now - observed)
        for collection in (
            value["pose_history"],
            value["sensor_rays"],
            value["qualitative_observations"],
            value["object_hypotheses"],
        ):
            for item in collection:
                observed = item.get("observed_at_unix_ms")
                if isinstance(observed, int):
                    item["age_ms"] = max(0, now - observed)
        for item in value["scan_evidence_history"]:
            completed = item.get("completed_at_unix_ms")
            if isinstance(completed, int):
                item["age_ms"] = max(0, now - completed)
        return value

    def close(self, drain: bool = True, timeout_s: float = 5.0) -> bool:
        """Stop accepting observations; no worker or motor resource is owned."""

        if type(drain) is not bool:
            raise ValueError("physical map drain flag is invalid")
        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not 0 <= float(timeout_s) <= 60
        ):
            raise ValueError("physical map close timeout is invalid")
        with self._lock:
            self._accepting = False
        return True


__all__ = ("PhysicalSpatialMapBridge",)
