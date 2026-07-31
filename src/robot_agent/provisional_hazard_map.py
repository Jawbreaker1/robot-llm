"""Persistent conservative collision envelopes for qualitative EV3 IR.

The EV3 infrared sensor does not measure a trustworthy metric surface and
does not identify an object.  A blocked observation therefore creates or
updates a provisional host hypothesis.  Clear readings never erase it merely
because the robot turned away.
"""

from dataclasses import dataclass, replace
import hashlib
import math
from typing import Iterable, Mapping, Optional, Tuple

from .physical_navigation_contract import MOTION_ACTIONS
from .physical_odometry import (
    OdometryCalibration,
    PhysicalPose,
    nominal_effect,
    normalize_heading_mdeg,
)


PROVISIONAL_QUALITATIVE = "PROVISIONAL_QUALITATIVE"
GEOMETRY_BASIS = "CONSERVATIVE_COLLISION_ENVELOPE_NOT_OBJECT_SURFACE"


@dataclass(frozen=True)
class HazardMapCalibration:
    robot_collision_radius_mm: int = 70
    provisional_hazard_offset_mm: int = 140
    provisional_hazard_radius_mm: int = 70
    hazard_merge_distance_mm: int = 120
    maximum_anchor_heading_drift_mdeg: int = 5_000

    def __post_init__(self) -> None:
        if any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value <= 0
            for value in (
                self.robot_collision_radius_mm,
                self.provisional_hazard_offset_mm,
                self.provisional_hazard_radius_mm,
                self.hazard_merge_distance_mm,
                self.maximum_anchor_heading_drift_mdeg,
            )
        ):
            raise ValueError("hazard map calibration is invalid")


@dataclass(frozen=True)
class ProvisionalHazard:
    hypothesis_id: str
    frame_id: str
    anchor_x_mm: int
    anchor_y_mm: int
    anchor_heading_mdeg: int
    centroid_x_mm: int
    centroid_y_mm: int
    radius_mm: int
    first_seen_at_ms: int
    last_seen_at_ms: int
    evidence_count: int
    last_state_version: int
    last_raw_ir_proximity: Optional[int]
    last_filtered_ir_proximity: Optional[int]
    scan_completed_at_ms: Optional[int] = None
    scan_left_boundary_mdeg: Optional[int] = None
    scan_right_boundary_mdeg: Optional[int] = None

    def __post_init__(self) -> None:
        if (
            not isinstance(self.hypothesis_id, str)
            or not self.hypothesis_id
            or not isinstance(self.frame_id, str)
            or not self.frame_id
        ):
            raise ValueError("hazard identity is invalid")
        integer_fields = (
            self.anchor_x_mm,
            self.anchor_y_mm,
            self.anchor_heading_mdeg,
            self.centroid_x_mm,
            self.centroid_y_mm,
            self.radius_mm,
            self.first_seen_at_ms,
            self.last_seen_at_ms,
            self.evidence_count,
            self.last_state_version,
        )
        if any(isinstance(value, bool) or not isinstance(value, int) for value in integer_fields):
            raise ValueError("hazard values are invalid")
        if (
            self.radius_mm <= 0
            or self.first_seen_at_ms < 0
            or self.last_seen_at_ms < self.first_seen_at_ms
            or self.evidence_count <= 0
            or self.last_state_version <= 0
        ):
            raise ValueError("hazard bounds are invalid")
        for reading in (
            self.last_raw_ir_proximity,
            self.last_filtered_ir_proximity,
        ):
            if reading is not None and (
                isinstance(reading, bool)
                or not isinstance(reading, int)
                or not 0 <= reading <= 100
            ):
                raise ValueError("hazard IR evidence is invalid")
        scan_values = (
            self.scan_completed_at_ms,
            self.scan_left_boundary_mdeg,
            self.scan_right_boundary_mdeg,
        )
        if any(value is not None for value in scan_values) and any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in scan_values
        ):
            raise ValueError("hazard scan evidence is incomplete")

    @property
    def bilateral_scan_complete(self) -> bool:
        return (
            self.scan_completed_at_ms is not None
            and self.scan_left_boundary_mdeg is not None
            and self.scan_right_boundary_mdeg is not None
            and self.scan_left_boundary_mdeg > 0
            and self.scan_right_boundary_mdeg < 0
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "hypothesis_id": self.hypothesis_id,
            "frame_id": self.frame_id,
            "semantic_label": "UNKNOWN",
            "quality": PROVISIONAL_QUALITATIVE,
            "provisional": True,
            "geometry_basis": GEOMETRY_BASIS,
            "anchor_x_mm": self.anchor_x_mm,
            "anchor_y_mm": self.anchor_y_mm,
            "anchor_heading_mdeg": self.anchor_heading_mdeg,
            "centroid_x_mm": self.centroid_x_mm,
            "centroid_y_mm": self.centroid_y_mm,
            "radius_mm": self.radius_mm,
            "first_seen_at_ms": self.first_seen_at_ms,
            "last_seen_at_ms": self.last_seen_at_ms,
            "evidence_count": self.evidence_count,
            "last_state_version": self.last_state_version,
            "last_raw_ir_proximity": self.last_raw_ir_proximity,
            "last_filtered_ir_proximity": self.last_filtered_ir_proximity,
            "scan_completed_at_ms": self.scan_completed_at_ms,
            "scan_left_boundary_mdeg": self.scan_left_boundary_mdeg,
            "scan_right_boundary_mdeg": self.scan_right_boundary_mdeg,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, object]):
        expected = {
            "hypothesis_id",
            "frame_id",
            "semantic_label",
            "quality",
            "provisional",
            "geometry_basis",
            "anchor_x_mm",
            "anchor_y_mm",
            "anchor_heading_mdeg",
            "centroid_x_mm",
            "centroid_y_mm",
            "radius_mm",
            "first_seen_at_ms",
            "last_seen_at_ms",
            "evidence_count",
            "last_state_version",
            "last_raw_ir_proximity",
            "last_filtered_ir_proximity",
            "scan_completed_at_ms",
            "scan_left_boundary_mdeg",
            "scan_right_boundary_mdeg",
        }
        if not isinstance(value, dict) or set(value) != expected:
            raise ValueError("hazard fields are invalid")
        if (
            value["semantic_label"] != "UNKNOWN"
            or value["quality"] != PROVISIONAL_QUALITATIVE
            or value["provisional"] is not True
            or value["geometry_basis"] != GEOMETRY_BASIS
        ):
            raise ValueError("hazard trust boundary is invalid")
        return cls(
            **{
                key: value[key]
                for key in expected
                if key
                not in {
                    "semantic_label",
                    "quality",
                    "provisional",
                    "geometry_basis",
                }
            }
        )


def _point_segment_distance(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> float:
    dx = x2 - x1
    dy = y2 - y1
    squared = dx * dx + dy * dy
    if squared <= 0:
        return math.hypot(px - x1, py - y1)
    projection = ((px - x1) * dx + (py - y1) * dy) / squared
    projection = max(0.0, min(1.0, projection))
    return math.hypot(
        px - (x1 + projection * dx),
        py - (y1 + projection * dy),
    )


class ProvisionalHazardMap:
    """Map-local stable hypotheses and worst-case swept-path validation."""

    def __init__(
        self,
        *,
        frame_id: str,
        map_generation_id: str,
        hazards: Iterable[ProvisionalHazard] = (),
        revision: int = 0,
        calibration: HazardMapCalibration = HazardMapCalibration(),
    ):
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError("frame_id is invalid")
        if not isinstance(map_generation_id, str) or not map_generation_id:
            raise ValueError("map_generation_id is invalid")
        if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
            raise ValueError("map revision is invalid")
        values = tuple(hazards)
        if len(values) > 32 or len({item.hypothesis_id for item in values}) != len(values):
            raise ValueError("hazard set is invalid")
        if any(item.frame_id != frame_id for item in values):
            raise ValueError("hazard frame does not match map frame")
        self.frame_id = frame_id
        self.map_generation_id = map_generation_id
        self.revision = revision
        self.calibration = calibration
        self._hazards = values

    @property
    def hazards(self) -> Tuple[ProvisionalHazard, ...]:
        return tuple(self._hazards)

    @property
    def hazard_ids(self) -> Tuple[str, ...]:
        return tuple(sorted(item.hypothesis_id for item in self._hazards))

    def get(self, hypothesis_id: str) -> Optional[ProvisionalHazard]:
        return next(
            (
                item
                for item in self._hazards
                if item.hypothesis_id == hypothesis_id
            ),
            None,
        )

    def _stable_id(
        self,
        pose: PhysicalPose,
        observed_at_ms: int,
        state_version: int,
    ) -> str:
        raw = "\0".join(
            (
                self.map_generation_id,
                str(pose.x_mm),
                str(pose.y_mm),
                str(pose.heading_mdeg),
                str(observed_at_ms),
                str(state_version),
            )
        ).encode("utf-8")
        return "provisional-object-{}".format(
            hashlib.sha256(raw).hexdigest()[:20]
        )

    def record_observation(
        self,
        pose: PhysicalPose,
        observation: Mapping[str, object],
        observed_at_ms: int,
    ) -> Optional[ProvisionalHazard]:
        """Persist blocked evidence; a clear observation only advances revision."""

        infrared = observation["infrared"]
        state_version = observation["state_version"]
        self.revision += 1
        if not infrared["blocked"]:
            return None
        heading = math.radians(pose.heading_mdeg / 1000.0)
        center_x = pose.x_mm + int(
            round(
                math.cos(heading)
                * self.calibration.provisional_hazard_offset_mm
            )
        )
        center_y = pose.y_mm + int(
            round(
                math.sin(heading)
                * self.calibration.provisional_hazard_offset_mm
            )
        )
        nearest = None
        nearest_distance = None
        for hazard in self._hazards:
            distance = math.hypot(
                center_x - hazard.centroid_x_mm,
                center_y - hazard.centroid_y_mm,
            )
            heading_drift = abs(
                ((pose.heading_mdeg - hazard.anchor_heading_mdeg + 180_000)
                 % 360_000)
                - 180_000
            )
            if (
                distance <= self.calibration.hazard_merge_distance_mm
                and heading_drift
                <= self.calibration.maximum_anchor_heading_drift_mdeg
                and (nearest_distance is None or distance < nearest_distance)
            ):
                nearest = hazard
                nearest_distance = distance
        if nearest is None:
            updated = ProvisionalHazard(
                hypothesis_id=self._stable_id(
                    pose,
                    observed_at_ms,
                    state_version,
                ),
                frame_id=self.frame_id,
                anchor_x_mm=pose.x_mm,
                anchor_y_mm=pose.y_mm,
                anchor_heading_mdeg=pose.heading_mdeg,
                centroid_x_mm=center_x,
                centroid_y_mm=center_y,
                radius_mm=self.calibration.provisional_hazard_radius_mm,
                first_seen_at_ms=observed_at_ms,
                last_seen_at_ms=observed_at_ms,
                evidence_count=1,
                last_state_version=state_version,
                last_raw_ir_proximity=infrared["raw"],
                last_filtered_ir_proximity=infrared["filtered"],
            )
            self._hazards = (self._hazards + (updated,))[-32:]
            return updated
        updated = replace(
            nearest,
            last_seen_at_ms=max(nearest.last_seen_at_ms, observed_at_ms),
            evidence_count=nearest.evidence_count + 1,
            last_state_version=state_version,
            last_raw_ir_proximity=infrared["raw"],
            last_filtered_ir_proximity=infrared["filtered"],
        )
        self._hazards = tuple(
            updated if item.hypothesis_id == nearest.hypothesis_id else item
            for item in self._hazards
        )
        return updated

    def record_scan_boundaries(
        self,
        hypothesis_id: str,
        *,
        completed_at_ms: int,
        left_boundary_mdeg: int,
        right_boundary_mdeg: int,
    ) -> ProvisionalHazard:
        hazard = self.get(hypothesis_id)
        if hazard is None:
            raise ValueError("scan target no longer exists")
        if (
            isinstance(completed_at_ms, bool)
            or not isinstance(completed_at_ms, int)
            or completed_at_ms < hazard.last_seen_at_ms
            or isinstance(left_boundary_mdeg, bool)
            or not isinstance(left_boundary_mdeg, int)
            or isinstance(right_boundary_mdeg, bool)
            or not isinstance(right_boundary_mdeg, int)
            or left_boundary_mdeg <= 0
            or right_boundary_mdeg >= 0
        ):
            raise ValueError("scan boundaries are not bilateral/fresh")
        updated = replace(
            hazard,
            scan_completed_at_ms=completed_at_ms,
            scan_left_boundary_mdeg=left_boundary_mdeg,
            scan_right_boundary_mdeg=right_boundary_mdeg,
        )
        self._hazards = tuple(
            updated if item.hypothesis_id == hypothesis_id else item
            for item in self._hazards
        )
        self.revision += 1
        return updated

    def validate_swept_path(
        self,
        pose: PhysicalPose,
        action: str,
        action_specs: Mapping[str, Mapping[str, object]],
        odometry_calibration: OdometryCalibration = OdometryCalibration(),
    ) -> Mapping[str, object]:
        """Veto any conservative body sweep, but allow monotonic escape."""

        if action not in MOTION_ACTIONS:
            return {
                "allowed": True,
                "reason": "nonmotion_action",
                "hazard_ids": [],
            }
        nominal, maximum = nominal_effect(
            pose,
            action,
            action_specs,
            odometry_calibration,
        )
        del nominal
        colliding = []
        escaping = []
        for hazard in self._hazards:
            clearance_radius = (
                self.calibration.robot_collision_radius_mm + hazard.radius_mm
            )
            start_distance = math.hypot(
                pose.x_mm - hazard.centroid_x_mm,
                pose.y_mm - hazard.centroid_y_mm,
            )
            end_distance = math.hypot(
                maximum.x_mm - hazard.centroid_x_mm,
                maximum.y_mm - hazard.centroid_y_mm,
            )
            swept_distance = _point_segment_distance(
                hazard.centroid_x_mm,
                hazard.centroid_y_mm,
                pose.x_mm,
                pose.y_mm,
                maximum.x_mm,
                maximum.y_mm,
            )
            if start_distance <= clearance_radius:
                travel_x = maximum.x_mm - pose.x_mm
                travel_y = maximum.y_mm - pose.y_mm
                away_x = pose.x_mm - hazard.centroid_x_mm
                away_y = pose.y_mm - hazard.centroid_y_mm
                strictly_away = (
                    travel_x * away_x + travel_y * away_y > 0
                    and end_distance > start_distance
                    and abs(swept_distance - start_distance) < 1e-6
                )
                if strictly_away:
                    escaping.append(hazard.hypothesis_id)
                else:
                    colliding.append(hazard.hypothesis_id)
            elif swept_distance <= clearance_radius:
                colliding.append(hazard.hypothesis_id)
        return {
            "allowed": not colliding,
            "reason": (
                "swept_path_clear"
                if not colliding
                else "provisional_hazard_swept_path_collision"
            ),
            "hazard_ids": sorted(colliding),
            "monotonic_escape_hazard_ids": sorted(escaping),
            "start_pose": pose.to_dict(),
            "maximum_endpoint": maximum.to_dict(),
            "host_selected_alternative_action": False,
        }

    def goal_geometry(
        self,
        *,
        pose: PhysicalPose,
        goal_heading_mdeg: int,
    ) -> Mapping[str, object]:
        """Publish exact boolean maneuver facts from conservative envelopes."""

        angle = math.radians(goal_heading_mdeg / 1000.0)
        goal_x = math.cos(angle)
        goal_y = math.sin(angle)
        rows = []
        conflicts = []
        target_behind = {}
        for hazard in sorted(
            self._hazards,
            key=lambda item: item.hypothesis_id,
        ):
            relative_x = hazard.centroid_x_mm - pose.x_mm
            relative_y = hazard.centroid_y_mm - pose.y_mm
            longitudinal = relative_x * goal_x + relative_y * goal_y
            signed_lateral = -relative_x * goal_y + relative_y * goal_x
            center_clearance = (
                abs(signed_lateral)
                if longitudinal >= 0
                else math.hypot(relative_x, relative_y)
            )
            required = (
                self.calibration.robot_collision_radius_mm
                + hazard.radius_mm
            )
            intersects = center_clearance <= required
            row = {
                "hypothesis_id": hazard.hypothesis_id,
                "longitudinal_offset_mm": int(round(longitudinal)),
                "signed_lateral_offset_mm": int(round(signed_lateral)),
                "half_line_center_clearance_mm": int(
                    round(center_clearance)
                ),
                "required_center_clearance_mm": required,
                "intersects_goal_heading_half_line": intersects,
            }
            rows.append(row)
            if intersects:
                conflicts.append(dict(row))
            target_behind[hazard.hypothesis_id] = (
                longitudinal + required < 0
            )
        heading_error = normalize_heading_mdeg(
            goal_heading_mdeg - pose.heading_mdeg
        )
        return {
            "goal_heading_mdeg": goal_heading_mdeg,
            "heading_error_mdeg": heading_error,
            "hazards": rows,
            "conflicts": conflicts,
            "facts": {
                "GOAL_CORRIDOR_CLEAR": not conflicts,
                "GOAL_HEADING_ALIGNED": abs(heading_error) <= 5_000,
                "TARGET_ENVELOPE_BEHIND_GOAL_ORIGIN": target_behind,
            },
        }

    def context(self) -> Mapping[str, object]:
        return {
            "map_generation_id": self.map_generation_id,
            "map_version": self.revision,
            "frame_id": self.frame_id,
            "navigation_hazard_hypotheses": [
                item.to_dict() for item in self._hazards
            ],
        }
