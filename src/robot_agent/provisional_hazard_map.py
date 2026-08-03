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

from .active_ir_scan_contract import ActiveIrScanResult
from .physical_navigation_contract import MOTION_ACTIONS
from .physical_footprint import (
    RobotFootprint,
    footprint_sweep_intersects,
)
from .physical_odometry import (
    OdometryCalibration,
    PhysicalPose,
    nominal_effect,
    normalize_heading_mdeg,
)
from .physical_scan_evidence import (
    MAX_COLLISION_SUPPORTS_PER_HAZARD,
    MAX_COLLISION_SUPPORTS_PER_MAP,
    MAX_SCAN_ATTEMPTS_PER_HAZARD,
    MAX_SCAN_ATTEMPTS_PER_MAP,
    AngularCollisionSupport,
    ScanAttemptEvidence,
    collision_supports_from_attempts,
    retain_collision_support_diversity,
    retain_scan_attempt_diversity,
)


PROVISIONAL_QUALITATIVE = "PROVISIONAL_QUALITATIVE"
GEOMETRY_BASIS = "CONSERVATIVE_COLLISION_ENVELOPE_NOT_OBJECT_SURFACE"
MAX_HAZARDS_PER_MAP = 64
HAZARD_CAPACITY_EVICTION = "MAP_CAPACITY_OLDEST_HYPOTHESIS"
PER_HAZARD_SCAN_EVICTION = (
    "PER_HAZARD_CAPACITY_DIVERSITY_RETENTION"
)
MAP_SCAN_EVICTION = "MAP_CAPACITY_OLDEST_ATTEMPT"
HAZARD_SCAN_EVICTION = "HAZARD_CAPACITY_WITH_HYPOTHESIS"
SCAN_ATTEMPT_EVICTION_REASONS = frozenset((
    PER_HAZARD_SCAN_EVICTION,
    MAP_SCAN_EVICTION,
    HAZARD_SCAN_EVICTION,
))
PER_HAZARD_SUPPORT_EVICTION = (
    "PER_HAZARD_CAPACITY_OR_STRUCTURAL_DUPLICATE"
)
MAP_SUPPORT_EVICTION = "MAP_CAPACITY_OLDEST_DISTINCT_SUPPORT"
COLLISION_SUPPORT_EVICTION_REASONS = frozenset((
    PER_HAZARD_SUPPORT_EVICTION,
    MAP_SUPPORT_EVICTION,
))

@dataclass(frozen=True)
class HazardMapCalibration:
    robot_collision_radius_mm: int = 70
    provisional_hazard_offset_mm: int = 140
    provisional_hazard_radius_mm: int = 70
    hazard_merge_distance_mm: int = 120
    maximum_anchor_heading_drift_mdeg: int = 5_000
    robot_footprint: Optional[RobotFootprint] = None

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
        if self.robot_footprint is not None and not isinstance(
            self.robot_footprint,
            RobotFootprint,
        ):
            raise ValueError("hazard map calibration is invalid")

    def collision_geometry(self) -> Mapping[str, object]:
        if self.robot_footprint is not None:
            return self.robot_footprint.to_dict()
        return {
            "geometry": "SYMMETRIC_CIRCLE",
            "reference_point": "DIFFERENTIAL_DRIVE_ORIGIN",
            "radius_mm": self.robot_collision_radius_mm,
        }


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
    scan_evidence_history: Tuple[ScanAttemptEvidence, ...] = ()
    scan_attempts_evicted: int = 0
    scan_attempts_eviction_reason: Optional[str] = None
    collision_supports: Tuple[AngularCollisionSupport, ...] = ()
    collision_supports_evicted: int = 0
    collision_supports_eviction_reason: Optional[str] = None
    collision_contested_at_ms: Optional[int] = None

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
        if (
            not isinstance(self.scan_evidence_history, tuple)
            or len(self.scan_evidence_history)
            > MAX_SCAN_ATTEMPTS_PER_HAZARD
            or any(
                not isinstance(item, ScanAttemptEvidence)
                for item in self.scan_evidence_history
            )
            or tuple(
                sorted(
                    self.scan_evidence_history,
                    key=lambda item: item.completed_at_ms,
                )
            )
            != self.scan_evidence_history
        ):
            raise ValueError("hazard scan history is invalid")
        if (
            isinstance(self.scan_attempts_evicted, bool)
            or not isinstance(self.scan_attempts_evicted, int)
            or self.scan_attempts_evicted < 0
            or (
                self.scan_attempts_evicted == 0
                and self.scan_attempts_eviction_reason is not None
            )
            or (
                self.scan_attempts_evicted > 0
                and self.scan_attempts_eviction_reason not in (
                    PER_HAZARD_SCAN_EVICTION,
                    MAP_SCAN_EVICTION,
                )
            )
        ):
            raise ValueError("hazard scan attempt eviction is invalid")
        derived_supports = collision_supports_from_attempts(
            self.scan_evidence_history
        )
        if not self.collision_supports and derived_supports:
            # Backward-compatible materialization for pre-index memories and
            # direct construction from retained detail evidence.
            object.__setattr__(
                self,
                "collision_supports",
                derived_supports,
            )
        if (
            not isinstance(self.collision_supports, tuple)
            or len(self.collision_supports)
            > MAX_COLLISION_SUPPORTS_PER_HAZARD
            or any(
                not isinstance(item, AngularCollisionSupport)
                for item in self.collision_supports
            )
            or tuple(sorted(
                self.collision_supports,
                key=lambda item: (
                    item.completed_at_ms,
                    item.source_scan_id,
                    item.spatial_key,
                ),
            )) != self.collision_supports
            or len({
                item.spatial_key for item in self.collision_supports
            }) != len(self.collision_supports)
        ):
            raise ValueError("hazard collision support index is invalid")
        if (
            isinstance(self.collision_supports_evicted, bool)
            or not isinstance(self.collision_supports_evicted, int)
            or self.collision_supports_evicted < 0
            or (
                self.collision_supports_evicted == 0
                and self.collision_supports_eviction_reason is not None
            )
            or (
                self.collision_supports_evicted > 0
                and self.collision_supports_eviction_reason
                not in COLLISION_SUPPORT_EVICTION_REASONS
            )
        ):
            raise ValueError("hazard collision support eviction is invalid")
        history_conflict = max(
            (
                attempt.completed_at_ms
                for attempt in self.scan_evidence_history
                if attempt.hypothesis_relation
                == "CONFLICTS_BLOCKED_HYPOTHESIS"
            ),
            default=None,
        )
        if self.collision_contested_at_ms is None:
            if history_conflict is not None:
                object.__setattr__(
                    self,
                    "collision_contested_at_ms",
                    history_conflict,
                )
        elif (
            isinstance(self.collision_contested_at_ms, bool)
            or not isinstance(self.collision_contested_at_ms, int)
            or self.collision_contested_at_ms < self.first_seen_at_ms
        ):
            raise ValueError("hazard collision conflict is invalid")
        elif (
            history_conflict is not None
            and history_conflict > self.collision_contested_at_ms
        ):
            object.__setattr__(
                self,
                "collision_contested_at_ms",
                history_conflict,
            )

    @property
    def bilateral_scan_complete(self) -> bool:
        return (
            self.scan_completed_at_ms is not None
            and self.scan_left_boundary_mdeg is not None
            and self.scan_right_boundary_mdeg is not None
            and self.scan_left_boundary_mdeg > 0
            and self.scan_right_boundary_mdeg < 0
            and (
                self.collision_contested_at_ms is None
                or self.scan_completed_at_ms
                > self.collision_contested_at_ms
            )
        )

    @property
    def latest_conflicting_scan_at_ms(self) -> Optional[int]:
        return self.collision_contested_at_ms

    @property
    def active_for_collision(self) -> bool:
        """Keep all-clear evidence without pretending the object vanished.

        A restored bilateral all-clear scan contests the earlier blocked
        envelope.  It suspends that envelope until a later blocked
        observation supports the hypothesis again.  The hypothesis and its
        evidence remain in memory either way.
        """

        conflict_at_ms = self.latest_conflicting_scan_at_ms
        latest_blocked_support_at_ms = max(
            (
                support.completed_at_ms
                for support in self.collision_supports
            ),
            default=-1,
        )
        latest_bilateral_boundary_at_ms = (
            self.scan_completed_at_ms
            if self.bilateral_scan_complete
            else -1
        )
        return (
            conflict_at_ms is None
            or max(
                self.last_seen_at_ms,
                latest_blocked_support_at_ms,
                latest_bilateral_boundary_at_ms,
            ) > conflict_at_ms
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
            "bilateral_scan_complete": self.bilateral_scan_complete,
            "scan_evidence_history": [
                item.to_dict() for item in self.scan_evidence_history
            ],
            "scan_attempts_evicted": self.scan_attempts_evicted,
            "scan_attempts_eviction_reason": (
                self.scan_attempts_eviction_reason
            ),
            "collision_supports": [
                item.to_dict() for item in self.collision_supports
            ],
            "collision_supports_evicted": (
                self.collision_supports_evicted
            ),
            "collision_supports_eviction_reason": (
                self.collision_supports_eviction_reason
            ),
            "collision_contested_at_ms": self.collision_contested_at_ms,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, object]):
        legacy = {
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
        history_fields = legacy | {"scan_evidence_history"}
        expected = history_fields | {"bilateral_scan_complete"}
        indexed = expected | {
            "collision_supports",
            "collision_supports_evicted",
            "collision_supports_eviction_reason",
            "collision_contested_at_ms",
        }
        retained = indexed | {
            "scan_attempts_evicted",
            "scan_attempts_eviction_reason",
        }
        if (
            not isinstance(value, dict)
            or set(value) not in (
                legacy,
                history_fields,
                expected,
                indexed,
                retained,
            )
        ):
            raise ValueError("hazard fields are invalid")
        if (
            value["semantic_label"] != "UNKNOWN"
            or value["quality"] != PROVISIONAL_QUALITATIVE
            or value["provisional"] is not True
            or value["geometry_basis"] != GEOMETRY_BASIS
        ):
            raise ValueError("hazard trust boundary is invalid")
        excluded = {
            "semantic_label",
            "quality",
            "provisional",
            "geometry_basis",
            "scan_evidence_history",
            "bilateral_scan_complete",
            "scan_attempts_evicted",
            "scan_attempts_eviction_reason",
            "collision_supports",
            "collision_supports_evicted",
            "collision_supports_eviction_reason",
            "collision_contested_at_ms",
        }
        arguments = {
            key: value[key]
            for key in legacy
            if key not in excluded
        }
        history = value.get("scan_evidence_history", [])
        if not isinstance(history, list):
            raise ValueError("hazard scan history is invalid")
        arguments["scan_evidence_history"] = tuple(
            ScanAttemptEvidence.from_dict(item) for item in history
        )
        arguments["scan_attempts_evicted"] = value.get(
            "scan_attempts_evicted",
            0,
        )
        arguments["scan_attempts_eviction_reason"] = value.get(
            "scan_attempts_eviction_reason"
        )
        raw_supports = value.get("collision_supports", [])
        if not isinstance(raw_supports, list):
            raise ValueError("hazard collision supports are invalid")
        arguments["collision_supports"] = tuple(
            AngularCollisionSupport.from_dict(item)
            for item in raw_supports
        )
        arguments["collision_supports_evicted"] = value.get(
            "collision_supports_evicted",
            0,
        )
        arguments["collision_supports_eviction_reason"] = value.get(
            "collision_supports_eviction_reason"
        )
        arguments["collision_contested_at_ms"] = value.get(
            "collision_contested_at_ms"
        )
        hazard = cls(**arguments)
        if (
            "bilateral_scan_complete" in value
            and value["bilateral_scan_complete"]
            is not hazard.bilateral_scan_complete
        ):
            raise ValueError("hazard bilateral scan fact is invalid")
        return hazard


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
        hazards_evicted: int = 0,
        hazards_eviction_reason: Optional[str] = None,
        scan_attempts_evicted: Optional[int] = None,
        scan_attempts_eviction_reason: Optional[str] = None,
    ):
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError("frame_id is invalid")
        if not isinstance(map_generation_id, str) or not map_generation_id:
            raise ValueError("map_generation_id is invalid")
        if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
            raise ValueError("map revision is invalid")
        if (
            isinstance(hazards_evicted, bool)
            or not isinstance(hazards_evicted, int)
            or hazards_evicted < 0
            or (
                hazards_evicted == 0
                and hazards_eviction_reason is not None
            )
            or (
                hazards_evicted > 0
                and hazards_eviction_reason != HAZARD_CAPACITY_EVICTION
            )
        ):
            raise ValueError("hazard eviction state is invalid")
        values = tuple(hazards)
        if (
            len(values) > MAX_HAZARDS_PER_MAP
            or len({item.hypothesis_id for item in values}) != len(values)
        ):
            raise ValueError("hazard set is invalid")
        if any(item.frame_id != frame_id for item in values):
            raise ValueError("hazard frame does not match map frame")
        if sum(
            len(item.scan_evidence_history) for item in values
        ) > MAX_SCAN_ATTEMPTS_PER_MAP:
            raise ValueError("hazard scan histories exceed map bound")
        retained_hazard_evictions = sum(
            item.scan_attempts_evicted for item in values
        )
        if scan_attempts_evicted is None:
            scan_attempts_evicted = retained_hazard_evictions
            if scan_attempts_evicted:
                reasons = {
                    item.scan_attempts_eviction_reason
                    for item in values
                    if item.scan_attempts_evicted
                }
                scan_attempts_eviction_reason = (
                    next(iter(reasons)) if len(reasons) == 1 else MAP_SCAN_EVICTION
                )
        if (
            isinstance(scan_attempts_evicted, bool)
            or not isinstance(scan_attempts_evicted, int)
            or scan_attempts_evicted < retained_hazard_evictions
            or (
                scan_attempts_evicted == 0
                and scan_attempts_eviction_reason is not None
            )
            or (
                scan_attempts_evicted > 0
                and scan_attempts_eviction_reason
                not in SCAN_ATTEMPT_EVICTION_REASONS
            )
        ):
            raise ValueError("map scan attempt eviction state is invalid")
        if sum(
            len(item.collision_supports) for item in values
        ) > MAX_COLLISION_SUPPORTS_PER_MAP:
            raise ValueError("hazard collision supports exceed map bound")
        self.frame_id = frame_id
        self.map_generation_id = map_generation_id
        self.revision = revision
        self.calibration = calibration
        self._hazards = values
        self.hazards_evicted = hazards_evicted
        self.hazards_eviction_reason = hazards_eviction_reason
        self.scan_attempts_evicted = scan_attempts_evicted
        self.scan_attempts_eviction_reason = (
            scan_attempts_eviction_reason
        )

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

    def active_collision_support_points(
        self,
        hypothesis_id: str,
    ) -> Tuple[Tuple[int, int], ...]:
        """Expose the exact support geometry used by swept-path checks.

        Route construction and collision validation must reason about the
        same remembered object envelope.  Returning no points means the
        hypothesis is missing or currently contested for collision use; it
        does not silently substitute the centroid.
        """

        hazard = self.get(hypothesis_id)
        if hazard is None:
            return ()
        return self._collision_envelopes(hazard)

    def active_collision_group(
        self,
        hypothesis_id: str,
    ) -> Tuple[ProvisionalHazard, ...]:
        """Return the connected active collision geometry around one target.

        This does not merge object identities.  It only derives the geometry
        a route must clear when conservative hazard envelopes overlap.
        """

        active = {
            hazard.hypothesis_id: (
                hazard,
                self._collision_envelopes(hazard),
            )
            for hazard in self._hazards
        }
        active = {
            key: value
            for key, value in active.items()
            if value[1]
        }
        if hypothesis_id not in active:
            return ()

        connected = {hypothesis_id}
        pending = [hypothesis_id]
        while pending:
            current_id = pending.pop(0)
            current, current_supports = active[current_id]
            for candidate_id in sorted(active):
                if candidate_id in connected:
                    continue
                candidate, candidate_supports = active[candidate_id]
                maximum_distance = current.radius_mm + candidate.radius_mm
                if any(
                    math.hypot(
                        current_x_mm - candidate_x_mm,
                        current_y_mm - candidate_y_mm,
                    ) <= maximum_distance
                    for current_x_mm, current_y_mm in current_supports
                    for candidate_x_mm, candidate_y_mm in candidate_supports
                ):
                    connected.add(candidate_id)
                    pending.append(candidate_id)

        return tuple(active[key][0] for key in sorted(connected))

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
            candidates = self._hazards + (updated,)
            evicted = max(0, len(candidates) - MAX_HAZARDS_PER_MAP)
            discarded = candidates[:evicted]
            self._hazards = candidates[-MAX_HAZARDS_PER_MAP:]
            if evicted:
                self.hazards_evicted += evicted
                self.hazards_eviction_reason = HAZARD_CAPACITY_EVICTION
                scan_attempts_discarded = sum(
                    len(item.scan_evidence_history) for item in discarded
                )
                if scan_attempts_discarded:
                    self.scan_attempts_evicted += scan_attempts_discarded
                    self.scan_attempts_eviction_reason = (
                        HAZARD_SCAN_EVICTION
                    )
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
        evidence_frame_id: str,
        evidence_map_generation_id: str,
        based_on_map_version: int,
        completed_at_ms: int,
        left_boundary_mdeg: int,
        right_boundary_mdeg: int,
    ) -> ProvisionalHazard:
        """Fuse a validated scan after its one mandatory reanchor observe.

        The post-scan observation intentionally advances the map once and may
        refresh the same hypothesis after the scan's completion timestamp.
        Freshness therefore comes from the scan's authoritative map basis and
        the exact one-revision transition, not from rewriting physical time.
        """
        hazard = self.get(hypothesis_id)
        if hazard is None:
            raise ValueError("scan target no longer exists")
        if (
            evidence_frame_id != self.frame_id
            or evidence_map_generation_id != self.map_generation_id
            or hazard.frame_id != evidence_frame_id
        ):
            raise ValueError("scan boundaries belong to a foreign map")
        if (
            isinstance(based_on_map_version, bool)
            or not isinstance(based_on_map_version, int)
            or based_on_map_version < 0
            or self.revision != based_on_map_version + 1
        ):
            raise ValueError("scan boundary map basis is stale")
        if (
            isinstance(completed_at_ms, bool)
            or not isinstance(completed_at_ms, int)
            or completed_at_ms < hazard.first_seen_at_ms
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

    def _prune_scan_histories(
        self,
        *,
        protected_scan_id: Optional[str] = None,
    ) -> None:
        """Keep planner context and persisted memory bounded map-wide."""

        while sum(
            len(hazard.scan_evidence_history) for hazard in self._hazards
        ) > MAX_SCAN_ATTEMPTS_PER_MAP:
            candidates = []
            for hazard in self._hazards:
                signature_counts = {}
                for attempt in hazard.scan_evidence_history:
                    signature_counts[attempt.evidence_signature] = (
                        signature_counts.get(attempt.evidence_signature, 0)
                        + 1
                    )
                for attempt_ordinal, attempt in enumerate(
                    hazard.scan_evidence_history
                ):
                    if attempt.scan_id == protected_scan_id:
                        continue
                    candidates.append((
                        0
                        if signature_counts[attempt.evidence_signature] > 1
                        else 1,
                        attempt.completed_at_ms,
                        hazard.hypothesis_id,
                        attempt.scan_id,
                        attempt_ordinal,
                        attempt,
                    ))
            if not candidates:
                raise ValueError("scan history bound cannot be enforced")
            (
                _redundancy,
                _completed_at_ms,
                hypothesis_id,
                _scan_id,
                _attempt_ordinal,
                discarded,
            ) = min(candidates)
            hazard = self.get(hypothesis_id)
            retained_values = list(hazard.scan_evidence_history)
            retained_values.remove(discarded)
            updated = replace(
                hazard,
                scan_evidence_history=tuple(retained_values),
                scan_attempts_evicted=hazard.scan_attempts_evicted + 1,
                scan_attempts_eviction_reason=MAP_SCAN_EVICTION,
            )
            self._hazards = tuple(
                updated if item.hypothesis_id == hypothesis_id else item
                for item in self._hazards
            )
            self.scan_attempts_evicted += 1
            self.scan_attempts_eviction_reason = MAP_SCAN_EVICTION

    def _prune_collision_supports(
        self,
        *,
        protected_source_scan_id: Optional[str] = None,
    ) -> None:
        """Enforce the map-wide materialized-support bound deterministically.

        Detail history is not consulted.  Each retained support is already a
        distinct angular/geometric fact.  Prefer removing the oldest support
        from a hypothesis that has alternatives, and protect the scan being
        committed so new physical evidence cannot disappear immediately.
        """

        while sum(
            len(hazard.collision_supports) for hazard in self._hazards
        ) > MAX_COLLISION_SUPPORTS_PER_MAP:
            candidates = []
            for hazard in self._hazards:
                for support in hazard.collision_supports:
                    if support.source_scan_id == protected_source_scan_id:
                        continue
                    candidates.append((
                        0 if len(hazard.collision_supports) > 1 else 1,
                        support.completed_at_ms,
                        hazard.hypothesis_id,
                        support.source_scan_id,
                        support.spatial_key,
                        support,
                    ))
            if not candidates:
                raise ValueError(
                    "collision support map bound cannot be enforced"
                )
            (
                _single_support_priority,
                _completed_at_ms,
                hypothesis_id,
                _source_scan_id,
                _spatial_key,
                discarded,
            ) = min(candidates)
            hazard = self.get(hypothesis_id)
            retained = tuple(
                support
                for support in hazard.collision_supports
                if support != discarded
            )
            updated = replace(
                hazard,
                collision_supports=retained,
                collision_supports_evicted=(
                    hazard.collision_supports_evicted + 1
                ),
                collision_supports_eviction_reason=MAP_SUPPORT_EVICTION,
            )
            self._hazards = tuple(
                updated if item.hypothesis_id == hypothesis_id else item
                for item in self._hazards
            )

    def record_scan_result(
        self,
        result: ActiveIrScanResult,
        *,
        scan_pose: PhysicalPose,
    ) -> ProvisionalHazard:
        """Persist every restored scan, including unilateral and all-clear.

        Bilateral boundaries remain the stronger route-commitment fact.  The
        bounded attempt history gives the planner cumulative evidence after
        ``latest_tool_result`` has been replaced by later motion feedback.
        """

        if not isinstance(result, ActiveIrScanResult):
            raise ValueError("scan result has the wrong type")
        hazard = self.get(result.target_hypothesis_id)
        if hazard is None:
            raise ValueError("scan target no longer exists")
        if (
            result.frame_id != self.frame_id
            or result.map_generation_id != self.map_generation_id
            or hazard.frame_id != result.frame_id
        ):
            raise ValueError("scan evidence belongs to a foreign map")
        if (
            isinstance(result.based_on_map_version, bool)
            or result.based_on_map_version < 0
            or self.revision != result.based_on_map_version + 1
        ):
            raise ValueError("scan evidence map basis is stale")
        if (
            result.stop_confirmed is not True
            or result.restored_start_heading is not True
            or result.completed_at_ms < hazard.first_seen_at_ms
        ):
            raise ValueError("scan evidence is not restored/fresh")
        if not isinstance(scan_pose, PhysicalPose):
            raise ValueError("scan evidence pose is invalid")
        attempt = ScanAttemptEvidence.from_scan_result(
            result,
            scan_pose=scan_pose,
        )
        if (
            attempt.observation_pattern == "ALL_CLEAR"
            and attempt.arc_coverage == "BILATERAL_ARC"
            and attempt.reason == "bilateral_boundaries_not_observed"
        ):
            attempt = replace(
                attempt,
                all_clear_arc_covers_target_hypothesis=(
                    self._all_clear_arc_covers_hazard(hazard, attempt)
                ),
            )
        history_candidates = hazard.scan_evidence_history + (attempt,)
        history = retain_scan_attempt_diversity(
            history_candidates,
            MAX_SCAN_ATTEMPTS_PER_HAZARD,
        )
        scan_attempt_evictions = len(history_candidates) - len(history)
        support_candidates = (
            hazard.collision_supports
            + AngularCollisionSupport.from_attempt(attempt)
        )
        collision_supports = retain_collision_support_diversity(
            support_candidates,
            MAX_COLLISION_SUPPORTS_PER_HAZARD,
        )
        support_evictions = (
            len(support_candidates) - len(collision_supports)
        )
        updates = {
            "scan_evidence_history": history,
            "collision_supports": collision_supports,
        }
        if scan_attempt_evictions:
            updates.update({
                "scan_attempts_evicted": (
                    hazard.scan_attempts_evicted
                    + scan_attempt_evictions
                ),
                "scan_attempts_eviction_reason": (
                    PER_HAZARD_SCAN_EVICTION
                ),
            })
            self.scan_attempts_evicted += scan_attempt_evictions
            self.scan_attempts_eviction_reason = (
                PER_HAZARD_SCAN_EVICTION
            )
        if support_evictions:
            updates.update({
                "collision_supports_evicted": (
                    hazard.collision_supports_evicted
                    + support_evictions
                ),
                "collision_supports_eviction_reason": (
                    PER_HAZARD_SUPPORT_EVICTION
                ),
            })
        if result.bilateral_complete:
            updates.update({
                "scan_completed_at_ms": result.completed_at_ms,
                "scan_left_boundary_mdeg": result.left_boundary_mdeg,
                "scan_right_boundary_mdeg": result.right_boundary_mdeg,
            })
        elif attempt.hypothesis_relation == "CONFLICTS_BLOCKED_HYPOTHESIS":
            updates.update({
                "scan_completed_at_ms": None,
                "scan_left_boundary_mdeg": None,
                "scan_right_boundary_mdeg": None,
                "collision_contested_at_ms": max(
                    hazard.collision_contested_at_ms or 0,
                    attempt.completed_at_ms,
                ),
            })
        updated = replace(hazard, **updates)
        self._hazards = tuple(
            updated
            if item.hypothesis_id == result.target_hypothesis_id
            else item
            for item in self._hazards
        )
        self._prune_scan_histories(protected_scan_id=result.scan_id)
        self._prune_collision_supports(
            protected_source_scan_id=result.scan_id
        )
        self.revision += 1
        return self.get(result.target_hypothesis_id)

    @staticmethod
    def _scan_arc_covers_circle(
        attempt: ScanAttemptEvidence,
        *,
        center_x_mm: int,
        center_y_mm: int,
        radius_mm: int,
    ) -> bool:
        """Check angular coverage without making a range claim."""

        if attempt.scan_pose is None or not attempt.rays:
            return False
        ray_bearings = [
            ray.actual_relative_bearing_mdeg for ray in attempt.rays
        ]
        minimum_bearing = min(ray_bearings)
        maximum_bearing = max(ray_bearings)
        alignment_tolerance_mdeg = 1_000
        pose = attempt.scan_pose
        relative_x = center_x_mm - pose.x_mm
        relative_y = center_y_mm - pose.y_mm
        distance = math.hypot(relative_x, relative_y)
        if distance <= radius_mm:
            return False
        center_bearing = normalize_heading_mdeg(
            int(round(math.degrees(math.atan2(relative_y, relative_x))
                      * 1_000))
            - pose.heading_mdeg
        )
        half_width = int(math.ceil(
            math.degrees(math.asin(min(1.0, radius_mm / distance)))
            * 1_000 - 1e-9
        ))
        return not (
            center_bearing - half_width
            < minimum_bearing - alignment_tolerance_mdeg
            or center_bearing + half_width
            > maximum_bearing + alignment_tolerance_mdeg
        )

    def _all_clear_arc_covers_hazard(
        self,
        hazard: ProvisionalHazard,
        attempt: ScanAttemptEvidence,
    ) -> bool:
        """Check an all-clear scan against remembered world geometry."""

        if attempt.scan_pose is None or not attempt.rays:
            return False
        pose = attempt.scan_pose

        def covered(
            support_x_mm,
            support_y_mm,
            radius_mm,
            evidence_pose_x_mm,
            evidence_pose_y_mm,
        ):
            relative_x = support_x_mm - pose.x_mm
            relative_y = support_y_mm - pose.y_mm
            distance = math.hypot(relative_x, relative_y)
            if distance <= radius_mm:
                return False
            # IR-PROX gives no trustworthy metric range.  A clear ray can
            # therefore contradict a remembered blocked support only while
            # that support's envelope still reaches the same provisional
            # near-field zone in which blocked observations are anchored.
            # Merely backing away until the reading crosses the collision
            # threshold must not erase the object from the map.
            evidence_distance = math.hypot(
                support_x_mm - evidence_pose_x_mm,
                support_y_mm - evidence_pose_y_mm,
            )
            if distance > evidence_distance:
                return False
            return self._scan_arc_covers_circle(
                attempt,
                center_x_mm=support_x_mm,
                center_y_mm=support_y_mm,
                radius_mm=radius_mm,
            )

        if not covered(
            hazard.centroid_x_mm,
            hazard.centroid_y_mm,
            hazard.radius_mm,
            hazard.anchor_x_mm,
            hazard.anchor_y_mm,
        ):
            return False
        conflict_cutoff = hazard.collision_contested_at_ms or -1
        for support in hazard.collision_supports:
            if support.completed_at_ms <= conflict_cutoff:
                continue
            heading = math.radians(
                (
                    support.pose_heading_mdeg
                    + support.actual_relative_bearing_mdeg
                )
                / 1_000.0
            )
            support_x_mm = support.pose_x_mm + int(round(
                math.cos(heading)
                * self.calibration.provisional_hazard_offset_mm
            ))
            support_y_mm = support.pose_y_mm + int(round(
                math.sin(heading)
                * self.calibration.provisional_hazard_offset_mm
            ))
            if not covered(
                support_x_mm,
                support_y_mm,
                hazard.radius_mm,
                support.pose_x_mm,
                support.pose_y_mm,
            ):
                return False
        return True

    def _scan_arc_contains_hazard_geometry(
        self,
        hazard: ProvisionalHazard,
        attempt: ScanAttemptEvidence,
    ) -> bool:
        """Check the complete active envelope without contesting the hazard."""

        envelopes = self._collision_envelopes(hazard)
        return bool(envelopes) and all(
            self._scan_arc_covers_circle(
                attempt,
                center_x_mm=center_x_mm,
                center_y_mm=center_y_mm,
                radius_mm=hazard.radius_mm,
            )
            for center_x_mm, center_y_mm in envelopes
        )

    def _collision_envelopes(
        self,
        hazard: ProvisionalHazard,
    ) -> Tuple[Tuple[int, int], ...]:
        """Return conservative support points for one object hypothesis.

        IR-PROX has no trustworthy range, so every support uses the same
        explicitly provisional offset as the original forward envelope.
        Blocked scan bearings extend the hypothesis angularly; they are not
        presented as measured object surfaces or distances.
        """

        if not hazard.active_for_collision:
            return ()
        values = [(hazard.centroid_x_mm, hazard.centroid_y_mm)]
        conflict_cutoff = hazard.collision_contested_at_ms or -1
        for support in hazard.collision_supports:
            if support.completed_at_ms <= conflict_cutoff:
                continue
            heading = math.radians(
                (
                    support.pose_heading_mdeg
                    + support.actual_relative_bearing_mdeg
                )
                / 1000.0
            )
            values.append((
                support.pose_x_mm
                + int(round(
                    math.cos(heading)
                    * self.calibration.provisional_hazard_offset_mm
                )),
                support.pose_y_mm
                + int(round(
                    math.sin(heading)
                    * self.calibration.provisional_hazard_offset_mm
                )),
            ))
        # Exact coordinate de-duplication is deterministic and does not infer
        # object identity beyond the parent hypothesis.
        return tuple(dict.fromkeys(values))

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
        footprint = self.calibration.robot_footprint
        heading_delta_mdeg = normalize_heading_mdeg(
            maximum.heading_mdeg - pose.heading_mdeg
        )
        colliding = []
        escaping = []
        for hazard in self._hazards:
            supports = self._collision_envelopes(hazard)
            if not supports:
                continue
            clearance_radius = (
                self.calibration.robot_collision_radius_mm + hazard.radius_mm
            )
            hazard_collides = False
            hazard_escapes = False
            for support_x_mm, support_y_mm in supports:
                start_distance = math.hypot(
                    pose.x_mm - support_x_mm,
                    pose.y_mm - support_y_mm,
                )
                end_distance = math.hypot(
                    maximum.x_mm - support_x_mm,
                    maximum.y_mm - support_y_mm,
                )
                swept_distance = _point_segment_distance(
                    support_x_mm,
                    support_y_mm,
                    pose.x_mm,
                    pose.y_mm,
                    maximum.x_mm,
                    maximum.y_mm,
                )
                if footprint is None:
                    start_intersects = start_distance <= clearance_radius
                    swept_intersects = swept_distance <= clearance_radius
                else:
                    (
                        start_intersects,
                        swept_intersects,
                    ) = footprint_sweep_intersects(
                        obstacle_x_mm=support_x_mm,
                        obstacle_y_mm=support_y_mm,
                        obstacle_radius_mm=hazard.radius_mm,
                        start=pose,
                        end=maximum,
                        footprint=footprint,
                    )
                if start_intersects:
                    travel_x = maximum.x_mm - pose.x_mm
                    travel_y = maximum.y_mm - pose.y_mm
                    away_x = pose.x_mm - support_x_mm
                    away_y = pose.y_mm - support_y_mm
                    strictly_away = (
                        travel_x * away_x + travel_y * away_y > 0
                        and end_distance > start_distance
                        and abs(swept_distance - start_distance) < 1e-6
                        and (footprint is None or heading_delta_mdeg == 0)
                    )
                    if strictly_away:
                        hazard_escapes = True
                    else:
                        hazard_collides = True
                elif swept_intersects:
                    hazard_collides = True
            if hazard_collides:
                colliding.append(hazard.hypothesis_id)
            elif hazard_escapes:
                escaping.append(hazard.hypothesis_id)
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
            "collision_geometry": (
                self.calibration.collision_geometry()
            ),
            "host_selected_alternative_action": False,
        }

    def validate_in_place_rotation(
        self,
        pose: PhysicalPose,
        relative_heading_offsets_mdeg: Iterable[int],
        alignment_tolerance_mdeg: int = 0,
    ) -> Mapping[str, object]:
        """Validate the body sweep used by a scan or other in-place look.

        The caller supplies the scan's relative bearings.  A scan traverses
        the complete interval between its extrema, so checking that interval
        catches an arm strike even when the centre-mounted IR ray itself is
        clear.
        """

        offsets = tuple(relative_heading_offsets_mdeg)
        if not offsets or any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or not -180_000 <= value <= 180_000
            for value in offsets
        ):
            raise ValueError("rotation sweep offsets are invalid")
        if (
            isinstance(alignment_tolerance_mdeg, bool)
            or not isinstance(alignment_tolerance_mdeg, int)
            or not 0 <= alignment_tolerance_mdeg <= 30_000
        ):
            raise ValueError("rotation sweep tolerance is invalid")
        minimum = max(
            -180_000,
            min(0, min(offsets)) - alignment_tolerance_mdeg,
        )
        maximum = min(
            180_000,
            max(0, max(offsets)) + alignment_tolerance_mdeg,
        )
        footprint = self.calibration.robot_footprint
        colliding = []
        for hazard in self._hazards:
            supports = self._collision_envelopes(hazard)
            if not supports:
                continue
            if footprint is None:
                # A symmetric circle occupies exactly the same volume at
                # every heading.  Rotation introduces no new collision and
                # must not make legacy profiles unable to scan a hypothesis
                # whose conservative envelopes already touch.
                intersects = False
            else:
                intersects = False
                for support_x_mm, support_y_mm in supports:
                    segment_start = pose
                    for relative_heading in (minimum, maximum, 0):
                        segment_end = replace(
                            pose,
                            heading_mdeg=normalize_heading_mdeg(
                                pose.heading_mdeg + relative_heading
                            ),
                        )
                        _starts_inside, segment_intersects = (
                            footprint_sweep_intersects(
                                obstacle_x_mm=support_x_mm,
                                obstacle_y_mm=support_y_mm,
                                obstacle_radius_mm=hazard.radius_mm,
                                start=segment_start,
                                end=segment_end,
                                footprint=footprint,
                            )
                        )
                        if segment_intersects:
                            intersects = True
                            break
                        segment_start = segment_end
                    if intersects:
                        break
            if intersects:
                colliding.append(hazard.hypothesis_id)
        return {
            "allowed": not colliding,
            "reason": (
                "in_place_rotation_clear"
                if not colliding
                else "provisional_hazard_rotation_sweep_collision"
            ),
            "hazard_ids": sorted(colliding),
            "start_pose": pose.to_dict(),
            "minimum_relative_heading_mdeg": minimum,
            "maximum_relative_heading_mdeg": maximum,
            "alignment_tolerance_mdeg": alignment_tolerance_mdeg,
            "collision_geometry": (
                self.calibration.collision_geometry()
            ),
            "host_selected_alternative_action": False,
        }

    def goal_geometry(
        self,
        *,
        pose: PhysicalPose,
        goal_heading_mdeg: int,
        heading_tolerance_mdeg: int = 5_000,
    ) -> Mapping[str, object]:
        """Publish exact boolean maneuver facts from conservative envelopes."""

        if (
            isinstance(heading_tolerance_mdeg, bool)
            or not isinstance(heading_tolerance_mdeg, int)
            or not 1_000 <= heading_tolerance_mdeg <= 45_000
        ):
            raise ValueError("goal heading tolerance is invalid")

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
            footprint = self.calibration.robot_footprint
            if footprint is None:
                behind_clearance = self.calibration.robot_collision_radius_mm
            else:
                behind_clearance = (
                    int(math.ceil(footprint.maximum_corner_radius_mm))
                    + footprint.clearance_margin_mm
                )
            supports = self._collision_envelopes(hazard)
            measurements = []
            for support_x_mm, support_y_mm in supports:
                relative_x = support_x_mm - pose.x_mm
                relative_y = support_y_mm - pose.y_mm
                longitudinal = relative_x * goal_x + relative_y * goal_y
                signed_lateral = -relative_x * goal_y + relative_y * goal_x
                center_clearance = (
                    abs(signed_lateral)
                    if longitudinal >= 0
                    else math.hypot(relative_x, relative_y)
                )
                if footprint is None:
                    body_clearance = (
                        self.calibration.robot_collision_radius_mm
                    )
                else:
                    body_clearance = (
                        max(
                            footprint.left_extent_mm,
                            footprint.right_extent_mm,
                        )
                        if signed_lateral == 0
                        else footprint.left_extent_mm
                        if signed_lateral > 0
                        else footprint.right_extent_mm
                    ) + footprint.clearance_margin_mm
                required = body_clearance + hazard.radius_mm
                measurements.append({
                    "longitudinal": longitudinal,
                    "signed_lateral": signed_lateral,
                    "center_clearance": center_clearance,
                    "required": required,
                    "intersects": center_clearance <= required,
                })
            if measurements:
                worst = min(
                    measurements,
                    key=lambda item: item["center_clearance"]
                    - item["required"],
                )
                intersects = any(
                    item["intersects"] for item in measurements
                )
                longitudinal = worst["longitudinal"]
                signed_lateral = worst["signed_lateral"]
                center_clearance = worst["center_clearance"]
                required = worst["required"]
            else:
                relative_x = hazard.centroid_x_mm - pose.x_mm
                relative_y = hazard.centroid_y_mm - pose.y_mm
                longitudinal = relative_x * goal_x + relative_y * goal_y
                signed_lateral = -relative_x * goal_y + relative_y * goal_x
                center_clearance = math.hypot(relative_x, relative_y)
                required = 0
                intersects = False
            row = {
                "hypothesis_id": hazard.hypothesis_id,
                "active_for_collision": hazard.active_for_collision,
                "collision_support_count": len(supports),
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
                not supports
                or all(
                    item["longitudinal"]
                    + behind_clearance
                    + hazard.radius_mm
                    < 0
                    for item in measurements
                )
            )
        heading_error = normalize_heading_mdeg(
            goal_heading_mdeg - pose.heading_mdeg
        )
        return {
            "goal_heading_mdeg": goal_heading_mdeg,
            "heading_error_mdeg": heading_error,
            "heading_tolerance_mdeg": heading_tolerance_mdeg,
            "hazards": rows,
            "conflicts": conflicts,
            "facts": {
                "GOAL_CORRIDOR_CLEAR": not conflicts,
                "GOAL_HEADING_ALIGNED": (
                    abs(heading_error) <= heading_tolerance_mdeg
                ),
                "TARGET_ENVELOPE_BEHIND_GOAL_ORIGIN": target_behind,
            },
        }

    def route_evidence(
        self,
        hypothesis_id: str,
        *,
        pose: PhysicalPose,
    ) -> Mapping[str, object]:
        """Publish whether stored angular evidence applies at this pose.

        Complementary one-sided attempts may accumulate only when they share
        the exact verified scan pose.  Evidence from another viewpoint stays
        useful to collision geometry, but body-relative boundaries are not
        silently compared as if they came from one viewpoint.
        """

        if not isinstance(pose, PhysicalPose):
            raise ValueError("route evidence pose is invalid")
        hazard = self.get(hypothesis_id)
        if hazard is None:
            return {
                "ready": False,
                "best_effort_ready": False,
                "strength": "NONE",
                "reason": "UNKNOWN_HYPOTHESIS",
                "applicable_scan_ids": [],
                "positive_boundary_scan_ids": [],
                "negative_boundary_scan_ids": [],
            }
        applicable = [
            attempt
            for attempt in hazard.scan_evidence_history
            if attempt.scan_pose == pose
            and attempt.hypothesis_relation
            != "CONFLICTS_BLOCKED_HYPOTHESIS"
        ]
        conflict_cutoff = hazard.collision_contested_at_ms or -1
        latest_collision_evidence_ms = max(
            (hazard.last_seen_at_ms,) + tuple(
                support.completed_at_ms
                for support in hazard.collision_supports
                if support.completed_at_ms > conflict_cutoff
            )
        )
        cross_hypothesis_all_clear = [
            attempt
            for source in self._hazards
            if source.hypothesis_id != hazard.hypothesis_id
            for attempt in source.scan_evidence_history
            if attempt.scan_pose == pose
            and attempt.observation_pattern == "ALL_CLEAR"
            and attempt.arc_coverage == "BILATERAL_ARC"
            and attempt.reason == "bilateral_boundaries_not_observed"
            and latest_collision_evidence_ms <= attempt.completed_at_ms
            and self._scan_arc_contains_hazard_geometry(hazard, attempt)
        ]
        positive = [
            attempt.scan_id
            for attempt in applicable
            if attempt.left_boundary_mdeg is not None
        ]
        negative = [
            attempt.scan_id
            for attempt in applicable
            if attempt.right_boundary_mdeg is not None
        ]
        blocked_arc = any(
            any(ray.blocked for ray in attempt.rays)
            for attempt in applicable
        )
        all_clear_arc = any(
            attempt.observation_pattern == "ALL_CLEAR"
            and attempt.arc_coverage == "BILATERAL_ARC"
            and attempt.reason == "bilateral_boundaries_not_observed"
            for attempt in applicable
        ) or bool(cross_hypothesis_all_clear)
        ready = (
            hazard.active_for_collision
            and bool(positive)
            and bool(negative)
        )
        best_effort_ready = (
            hazard.active_for_collision
            and bool(positive or negative or blocked_arc or all_clear_arc)
        )
        if not hazard.active_for_collision:
            reason = "HYPOTHESIS_CONTESTED_BY_FULL_ALL_CLEAR"
        elif not applicable and not cross_hypothesis_all_clear:
            reason = "NO_SCAN_EVIDENCE_AT_CURRENT_VERIFIED_POSE"
        elif blocked_arc and not (positive or negative):
            reason = "BLOCKED_ARC_WITHOUT_BOUNDARY"
        elif all_clear_arc and not (positive or negative):
            reason = "ALL_CLEAR_ARC_AT_CURRENT_VERIFIED_POSE"
        elif not positive or not negative:
            reason = "COMPLEMENTARY_BOUNDARY_EVIDENCE_REQUIRED"
        else:
            reason = "COMPLEMENTARY_BOUNDARIES_AT_CURRENT_POSE"
        return {
            "ready": ready,
            "best_effort_ready": best_effort_ready,
            "strength": (
                "BILATERAL_BOUNDARIES"
                if ready
                else "UNILATERAL_BOUNDARY"
                if positive or negative
                else "BLOCKED_ARC"
                if best_effort_ready and blocked_arc
                else "ALL_CLEAR_ARC"
                if best_effort_ready and all_clear_arc
                else "NONE"
            ),
            "reason": reason,
            "applicable_scan_ids": [
                attempt.scan_id for attempt in applicable
            ],
            "positive_boundary_scan_ids": positive,
            "negative_boundary_scan_ids": negative,
        }

    def context(self) -> Mapping[str, object]:
        hypotheses = []
        for item in self._hazards:
            value = dict(item.to_dict())
            # The materialized angular index is authoritative persisted state,
            # not repeated planner detail.  Publish its factual health and the
            # derived geometry count while scan history remains the bounded
            # human/model-readable evidence projection.
            value.pop("collision_supports")
            value.update({
                "active_for_collision": item.active_for_collision,
                "collision_support_count": len(
                    self._collision_envelopes(item)
                ),
                "collision_evidence_basis": (
                    "PROVISIONAL_IR_ANGULAR_SUPPORTS_NOT_OBJECT_SURFACE"
                ),
            })
            hypotheses.append(value)
        return {
            "map_generation_id": self.map_generation_id,
            "map_version": self.revision,
            "frame_id": self.frame_id,
            "hazard_retention": self.hazard_retention(),
            "scan_attempt_retention": self.scan_attempt_retention(),
            "collision_geometry": self.calibration.collision_geometry(),
            "navigation_hazard_hypotheses": hypotheses,
        }

    def hazard_retention(self) -> Mapping[str, object]:
        """Publish bounded-map loss explicitly without inventing geometry."""

        return {
            "capacity": MAX_HAZARDS_PER_MAP,
            "retained_count": len(self._hazards),
            "evicted_count": self.hazards_evicted,
            "last_eviction_reason": self.hazards_eviction_reason,
        }

    def scan_attempt_retention(self) -> Mapping[str, object]:
        """Publish exact persisted scan-detail loss and both hard caps."""

        return {
            "per_hazard_capacity": MAX_SCAN_ATTEMPTS_PER_HAZARD,
            "map_capacity": MAX_SCAN_ATTEMPTS_PER_MAP,
            "retained_count": sum(
                len(item.scan_evidence_history) for item in self._hazards
            ),
            "evicted_count": self.scan_attempts_evicted,
            "last_eviction_reason": self.scan_attempts_eviction_reason,
        }
