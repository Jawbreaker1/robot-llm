"""Immutable, JSON-friendly contracts for uncertain spatial mapping.

The map contains geometric occupancy evidence and qualitative proximity
evidence, but no semantic classifier output.  Object identities are opaque
host hypotheses; the only semantic label in this first slice is ``UNKNOWN``.
"""

from dataclasses import dataclass
from typing import Mapping, Optional, Tuple, Union

from .navigation_contract import (
    NavigationContractError,
    identifier,
    integer,
)
from .physical_scan_evidence import (
    BODY_RELATIVE_BEARING_CONVENTION,
    MAX_SCAN_ATTEMPTS_PER_MAP,
    ScanAttemptEvidence,
)
from .spatial_contract_validation import (
    boolean as _boolean,
    is_unique_identifier_tuple,
    normalize_collision_geometry_mapping,
    optional_identifier as _optional_identifier,
    validate_evidence_sources,
)


SPATIAL_MAP_SNAPSHOT_SCHEMA = "robot-spatial-map-snapshot/v1"
SPATIAL_MAP_UPDATE_SCHEMA = "robot-spatial-map-update/v1"
DASHBOARD_SPATIAL_MAP_SCHEMA = "robot-spatial-map/v1"
MAX_POSE_HISTORY = 256
MAX_SPATIAL_SCAN_EVIDENCE = MAX_SCAN_ATTEMPTS_PER_MAP

CELL_UNKNOWN = "UNKNOWN"
CELL_FREE = "FREE"
CELL_OCCUPIED = "OCCUPIED"

SEMANTIC_UNKNOWN = "UNKNOWN"
METRIC_FUSED = "METRIC_FUSED"
PROVISIONAL_QUALITATIVE = "PROVISIONAL_QUALITATIVE"
PHYSICAL_IR_REFLECTION = "physical_ir_reflection"
LOCAL_ODOMETRY_POSE = "LOCAL_ODOMETRY_POSE"
QUALITATIVE_FORWARD_ENVELOPE = "QUALITATIVE_FORWARD_ENVELOPE"

SIMULATION_WORLD = "SIMULATION_WORLD"
LOCAL_ODOMETRY = "LOCAL_ODOMETRY"
MAP_EMPTY = "EMPTY"
MAP_SIMULATION_METRIC = "SIMULATION_METRIC"
MAP_PROVISIONAL_IR = "PROVISIONAL_IR"
MAP_METRIC_WITH_PROVISIONAL_IR = "METRIC_WITH_PROVISIONAL_IR"

UPDATE_APPLIED = "APPLIED"
STALE_STATE_VERSION = "STALE_STATE_VERSION"
STALE_WORLD_MODEL_VERSION = "STALE_WORLD_MODEL_VERSION"
STALE_TIMESTAMP = "STALE_TIMESTAMP"

_MAX_INT = 2**63 - 1

ASYMMETRIC_RECTANGLE = "ASYMMETRIC_RECTANGLE"
SYMMETRIC_CIRCLE = "SYMMETRIC_CIRCLE"
DIFFERENTIAL_DRIVE_ORIGIN = "DIFFERENTIAL_DRIVE_ORIGIN"
ANGULAR_NONMETRIC_IR_SCAN = "ANGULAR_NONMETRIC_IR_SCAN"


@dataclass(frozen=True)
class SpatialBounds:
    """Metric bounds of the currently retained grid cells."""

    min_x_mm: int
    min_y_mm: int
    max_x_mm: int
    max_y_mm: int

    def __post_init__(self) -> None:
        for name, value in (
            ("min_x_mm", self.min_x_mm),
            ("min_y_mm", self.min_y_mm),
            ("max_x_mm", self.max_x_mm),
            ("max_y_mm", self.max_y_mm),
        ):
            integer(name, value, -1_000_000_000, 1_000_000_000)
        if (
            self.min_x_mm >= self.max_x_mm
            or self.min_y_mm >= self.max_y_mm
        ):
            raise NavigationContractError(
                "invalid_spatial_bounds",
                "Spatial bounds must enclose positive area",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "min_x_mm": self.min_x_mm,
            "min_y_mm": self.min_y_mm,
            "max_x_mm": self.max_x_mm,
            "max_y_mm": self.max_y_mm,
        }


@dataclass(frozen=True)
class SpatialRobotPose:
    """Latest metric robot pose in the map frame."""

    frame_id: str
    x_mm: int
    y_mm: int
    heading_mdeg: int
    observed_at_ms: int
    captured_at_host_ms: int
    state_version: int
    world_model_version: int

    def __post_init__(self) -> None:
        identifier("frame_id", self.frame_id, 96)
        integer("x_mm", self.x_mm, -1_000_000, 1_000_000)
        integer("y_mm", self.y_mm, -1_000_000, 1_000_000)
        integer(
            "heading_mdeg",
            self.heading_mdeg,
            -180_000,
            179_999,
        )
        integer("observed_at_ms", self.observed_at_ms, 0, _MAX_INT)
        integer(
            "captured_at_host_ms",
            self.captured_at_host_ms,
            self.observed_at_ms,
            _MAX_INT,
        )
        integer("state_version", self.state_version, 1, _MAX_INT)
        integer(
            "world_model_version",
            self.world_model_version,
            1,
            _MAX_INT,
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "frame_id": self.frame_id,
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
            "heading_mdeg": self.heading_mdeg,
            "observed_at_ms": self.observed_at_ms,
            "captured_at_host_ms": self.captured_at_host_ms,
            "state_version": self.state_version,
            "world_model_version": self.world_model_version,
        }


@dataclass(frozen=True)
class SpatialCollisionGeometry:
    """Trusted robot-body dimensions used by navigation collision checks.

    These dimensions describe the robot around its drive origin.  They are
    deliberately separate from IR evidence: a real body extent is metric,
    while a reflected IR ray still carries no range or endpoint.
    """

    geometry: str
    reference_point: str
    radius_mm: Optional[int] = None
    front_extent_mm: Optional[int] = None
    rear_extent_mm: Optional[int] = None
    left_extent_mm: Optional[int] = None
    right_extent_mm: Optional[int] = None
    clearance_margin_mm: Optional[int] = None
    calibration_status: Optional[str] = None
    calibration_evidence: Optional[str] = None

    def __post_init__(self) -> None:
        if self.reference_point != DIFFERENTIAL_DRIVE_ORIGIN:
            raise NavigationContractError(
                "invalid_collision_geometry",
                "Collision geometry reference point is invalid",
            )
        extents = (
            self.front_extent_mm,
            self.rear_extent_mm,
            self.left_extent_mm,
            self.right_extent_mm,
        )
        if self.geometry == SYMMETRIC_CIRCLE:
            integer("radius_mm", self.radius_mm, 1, 1_000)
            if any(value is not None for value in extents) or any(
                value is not None
                for value in (
                    self.clearance_margin_mm,
                    self.calibration_status,
                    self.calibration_evidence,
                )
            ):
                raise NavigationContractError(
                    "invalid_collision_geometry",
                    "Circular collision geometry has rectangle fields",
                )
            return
        if self.geometry != ASYMMETRIC_RECTANGLE:
            raise NavigationContractError(
                "invalid_collision_geometry",
                "Collision geometry kind is invalid",
            )
        if self.radius_mm is not None:
            raise NavigationContractError(
                "invalid_collision_geometry",
                "Rectangular collision geometry has a radius",
            )
        for name, value in (
            ("front_extent_mm", self.front_extent_mm),
            ("rear_extent_mm", self.rear_extent_mm),
            ("left_extent_mm", self.left_extent_mm),
            ("right_extent_mm", self.right_extent_mm),
        ):
            integer(name, value, 1, 1_000)
        integer(
            "clearance_margin_mm",
            self.clearance_margin_mm,
            0,
            500,
        )
        _optional_identifier(
            "calibration_status",
            self.calibration_status,
            512,
        )
        _optional_identifier(
            "calibration_evidence",
            self.calibration_evidence,
            512,
        )
        if (
            self.calibration_status is None
            or self.calibration_evidence is None
        ):
            raise NavigationContractError(
                "invalid_collision_geometry",
                "Rectangular collision calibration provenance is missing",
            )

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]):
        return cls(**normalize_collision_geometry_mapping(value))

    def to_dict(self) -> Mapping[str, object]:
        if self.geometry == SYMMETRIC_CIRCLE:
            return {
                "geometry": self.geometry,
                "reference_point": self.reference_point,
                "radius_mm": self.radius_mm,
            }
        return {
            "geometry": self.geometry,
            "reference_point": self.reference_point,
            "front_extent_mm": self.front_extent_mm,
            "rear_extent_mm": self.rear_extent_mm,
            "left_extent_mm": self.left_extent_mm,
            "right_extent_mm": self.right_extent_mm,
            "clearance_margin_mm": self.clearance_margin_mm,
            "calibration_status": self.calibration_status,
            "calibration_evidence": self.calibration_evidence,
        }


@dataclass(frozen=True)
class SpatialScanRayEvidence:
    """One body-relative scan direction, explicitly without distance."""

    requested_relative_bearing_mdeg: int
    actual_relative_bearing_mdeg: int
    blocked: bool
    raw_ir_proximity: Optional[int]
    filtered_ir_proximity: Optional[int]

    def __post_init__(self) -> None:
        integer(
            "requested_relative_bearing_mdeg",
            self.requested_relative_bearing_mdeg,
            -90_000,
            90_000,
        )
        integer(
            "actual_relative_bearing_mdeg",
            self.actual_relative_bearing_mdeg,
            -100_000,
            100_000,
        )
        _boolean("blocked", self.blocked)
        for name, value in (
            ("raw_ir_proximity", self.raw_ir_proximity),
            ("filtered_ir_proximity", self.filtered_ir_proximity),
        ):
            if value is not None:
                integer(name, value, 0, 100)

    def to_dict(self) -> Mapping[str, object]:
        # No range, endpoint, or free-space claim belongs in this contract.
        return {
            "requested_relative_bearing_mdeg": (
                self.requested_relative_bearing_mdeg
            ),
            "actual_relative_bearing_mdeg": (
                self.actual_relative_bearing_mdeg
            ),
            "blocked": self.blocked,
            "raw_ir_proximity": self.raw_ir_proximity,
            "filtered_ir_proximity": self.filtered_ir_proximity,
        }


@dataclass(frozen=True)
class SpatialScanEvidence:
    """A bounded, read-only projection of one restored active IR scan."""

    target_hypothesis_id: str
    frame_id: str
    hypothesis_anchor_x_mm: int
    hypothesis_anchor_y_mm: int
    hypothesis_anchor_heading_mdeg: int
    scan_x_mm: Optional[int]
    scan_y_mm: Optional[int]
    scan_heading_mdeg: Optional[int]
    based_on_map_version: Optional[int]
    scan_id: str
    completed_at_unix_ms: int
    status: str
    reason: str
    observation_pattern: str
    arc_coverage: str
    boundary_coverage: str
    hypothesis_relation: str
    left_boundary_mdeg: Optional[int]
    right_boundary_mdeg: Optional[int]
    rays: Tuple[SpatialScanRayEvidence, ...]
    bearing_convention: str = BODY_RELATIVE_BEARING_CONVENTION
    geometry_kind: str = ANGULAR_NONMETRIC_IR_SCAN
    provisional: bool = True
    read_only: bool = True

    def __post_init__(self) -> None:
        identifier("target_hypothesis_id", self.target_hypothesis_id)
        identifier("frame_id", self.frame_id, 96)
        identifier("scan_id", self.scan_id)
        integer(
            "hypothesis_anchor_x_mm",
            self.hypothesis_anchor_x_mm,
            -1_000_000,
            1_000_000,
        )
        integer(
            "hypothesis_anchor_y_mm",
            self.hypothesis_anchor_y_mm,
            -1_000_000,
            1_000_000,
        )
        integer(
            "hypothesis_anchor_heading_mdeg",
            self.hypothesis_anchor_heading_mdeg,
            -180_000,
            179_999,
        )
        scan_pose = (
            self.scan_x_mm,
            self.scan_y_mm,
            self.scan_heading_mdeg,
        )
        has_scan_pose = all(value is not None for value in scan_pose)
        if any(value is not None for value in scan_pose) != has_scan_pose:
            raise NavigationContractError(
                "invalid_spatial_scan_evidence",
                "Spatial scan pose is incomplete",
            )
        if has_scan_pose:
            integer("scan_x_mm", self.scan_x_mm, -1_000_000, 1_000_000)
            integer("scan_y_mm", self.scan_y_mm, -1_000_000, 1_000_000)
            integer(
                "scan_heading_mdeg",
                self.scan_heading_mdeg,
                -180_000,
                179_999,
            )
            integer(
                "based_on_map_version",
                self.based_on_map_version,
                0,
                _MAX_INT,
            )
        elif self.based_on_map_version is not None:
            raise NavigationContractError(
                "invalid_spatial_scan_evidence",
                "Map basis without a scan pose is invalid",
            )
        integer(
            "completed_at_unix_ms",
            self.completed_at_unix_ms,
            0,
            _MAX_INT,
        )
        if (
            self.status not in ("COMPLETED", "CANCELLED")
            or not isinstance(self.reason, str)
            or not self.reason
            or len(self.reason) > 160
            or self.observation_pattern
            not in ("NO_RAYS", "ALL_CLEAR", "ALL_BLOCKED", "MIXED")
            or self.arc_coverage
            not in (
                "NO_ARC",
                "CENTER_ONLY",
                "NEGATIVE_ARC_ONLY",
                "POSITIVE_ARC_ONLY",
                "BILATERAL_ARC",
            )
            or self.boundary_coverage
            not in (
                "NO_BOUNDARIES",
                "POSITIVE_BOUNDARY_ONLY",
                "NEGATIVE_BOUNDARY_ONLY",
                "BILATERAL_BOUNDARIES",
            )
            or self.hypothesis_relation
            not in (
                "NO_EVIDENCE",
                "SUPPORTS_BLOCKED_HYPOTHESIS",
                "CONFLICTS_BLOCKED_HYPOTHESIS",
            )
            or self.bearing_convention
            != BODY_RELATIVE_BEARING_CONVENTION
            or self.geometry_kind != ANGULAR_NONMETRIC_IR_SCAN
            or self.provisional is not True
            or self.read_only is not True
            or not isinstance(self.rays, tuple)
            or len(self.rays) > 16
            or any(
                not isinstance(ray, SpatialScanRayEvidence)
                for ray in self.rays
            )
            or len({
                ray.requested_relative_bearing_mdeg for ray in self.rays
            }) != len(self.rays)
        ):
            raise NavigationContractError(
                "invalid_spatial_scan_evidence",
                "Spatial scan evidence is invalid",
            )
        if self.left_boundary_mdeg is not None:
            integer(
                "left_boundary_mdeg",
                self.left_boundary_mdeg,
                1,
                90_000,
            )
        if self.right_boundary_mdeg is not None:
            integer(
                "right_boundary_mdeg",
                self.right_boundary_mdeg,
                -90_000,
                -1,
            )

    @classmethod
    def from_navigation_evidence(
        cls,
        *,
        target_hypothesis_id: str,
        frame_id: str,
        anchor_x_mm: int,
        anchor_y_mm: int,
        anchor_heading_mdeg: int,
        attempt: ScanAttemptEvidence,
    ):
        if not isinstance(attempt, ScanAttemptEvidence):
            raise NavigationContractError(
                "invalid_spatial_scan_evidence",
                "Navigation scan evidence is invalid",
            )
        return cls(
            target_hypothesis_id=target_hypothesis_id,
            frame_id=frame_id,
            hypothesis_anchor_x_mm=anchor_x_mm,
            hypothesis_anchor_y_mm=anchor_y_mm,
            hypothesis_anchor_heading_mdeg=anchor_heading_mdeg,
            scan_x_mm=(
                None if attempt.scan_pose is None else attempt.scan_pose.x_mm
            ),
            scan_y_mm=(
                None if attempt.scan_pose is None else attempt.scan_pose.y_mm
            ),
            scan_heading_mdeg=(
                None
                if attempt.scan_pose is None
                else attempt.scan_pose.heading_mdeg
            ),
            based_on_map_version=attempt.based_on_map_version,
            scan_id=attempt.scan_id,
            completed_at_unix_ms=attempt.completed_at_ms,
            status=attempt.status,
            reason=attempt.reason,
            observation_pattern=attempt.observation_pattern,
            arc_coverage=attempt.arc_coverage,
            boundary_coverage=attempt.boundary_coverage,
            hypothesis_relation=attempt.hypothesis_relation,
            left_boundary_mdeg=attempt.left_boundary_mdeg,
            right_boundary_mdeg=attempt.right_boundary_mdeg,
            rays=tuple(
                SpatialScanRayEvidence(
                    requested_relative_bearing_mdeg=(
                        ray.requested_relative_bearing_mdeg
                    ),
                    actual_relative_bearing_mdeg=(
                        ray.actual_relative_bearing_mdeg
                    ),
                    blocked=ray.blocked,
                    raw_ir_proximity=ray.raw,
                    filtered_ir_proximity=ray.filtered,
                )
                for ray in attempt.rays
            ),
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "target_hypothesis_id": self.target_hypothesis_id,
            "frame_id": self.frame_id,
            "hypothesis_anchor_pose": {
                "x_mm": self.hypothesis_anchor_x_mm,
                "y_mm": self.hypothesis_anchor_y_mm,
                "heading_mdeg": self.hypothesis_anchor_heading_mdeg,
            },
            "scan_pose": (
                None
                if self.scan_x_mm is None
                else {
                    "x_mm": self.scan_x_mm,
                    "y_mm": self.scan_y_mm,
                    "heading_mdeg": self.scan_heading_mdeg,
                }
            ),
            "based_on_map_version": self.based_on_map_version,
            "scan_id": self.scan_id,
            "completed_at_unix_ms": self.completed_at_unix_ms,
            "status": self.status,
            "reason": self.reason,
            "bearing_convention": self.bearing_convention,
            "geometry_kind": self.geometry_kind,
            "observation_pattern": self.observation_pattern,
            "arc_coverage": self.arc_coverage,
            "boundary_coverage": self.boundary_coverage,
            "hypothesis_relation": self.hypothesis_relation,
            "left_boundary_mdeg": self.left_boundary_mdeg,
            "right_boundary_mdeg": self.right_boundary_mdeg,
            "rays": [ray.to_dict() for ray in self.rays],
            "provisional": self.provisional,
            "read_only": self.read_only,
        }


@dataclass(frozen=True)
class SpatialSensorRay:
    """One latest sensor ray without crossing metric trust boundaries."""

    direction: str
    frame_id: str
    source: str
    observed_at_ms: int
    captured_at_host_ms: int
    state_version: int
    world_model_version: int
    confidence_milli: int
    provisional: bool
    origin_x_mm: Optional[int] = None
    origin_y_mm: Optional[int] = None
    end_x_mm: Optional[int] = None
    end_y_mm: Optional[int] = None
    measured_range_mm: Optional[int] = None
    max_range_mm: Optional[int] = None
    endpoint_occupied: Optional[bool] = None
    relation: Optional[str] = None
    raw_ir_proximity: Optional[int] = None
    trusted_simulator_object_id: Optional[str] = None

    def __post_init__(self) -> None:
        if self.direction not in ("FORWARD", "LEFT", "RIGHT"):
            raise NavigationContractError(
                "invalid_sensor_ray_direction",
                "Spatial sensor ray direction is invalid",
            )
        identifier("frame_id", self.frame_id, 96)
        if self.source not in (
            "simulation_metric",
            PHYSICAL_IR_REFLECTION,
        ):
            raise NavigationContractError(
                "invalid_sensor_ray_source",
                "Spatial sensor ray source is invalid",
            )
        integer("observed_at_ms", self.observed_at_ms, 0, _MAX_INT)
        integer(
            "captured_at_host_ms",
            self.captured_at_host_ms,
            self.observed_at_ms,
            _MAX_INT,
        )
        integer("state_version", self.state_version, 1, _MAX_INT)
        integer(
            "world_model_version",
            self.world_model_version,
            1,
            _MAX_INT,
        )
        integer(
            "confidence_milli",
            self.confidence_milli,
            0,
            1_000,
        )
        _boolean("provisional", self.provisional)
        metric_values = (
            self.origin_x_mm,
            self.origin_y_mm,
            self.end_x_mm,
            self.end_y_mm,
            self.measured_range_mm,
            self.max_range_mm,
        )
        if self.source == "simulation_metric":
            if (
                any(value is None for value in metric_values)
                or self.endpoint_occupied is None
                or self.relation is not None
                or self.raw_ir_proximity is not None
                or self.provisional
            ):
                raise NavigationContractError(
                    "invalid_metric_sensor_ray",
                    "Metric simulator ray fields are invalid",
                )
            for name, value in (
                ("origin_x_mm", self.origin_x_mm),
                ("origin_y_mm", self.origin_y_mm),
                ("end_x_mm", self.end_x_mm),
                ("end_y_mm", self.end_y_mm),
            ):
                integer(
                    name,
                    value,
                    -1_000_000_000,
                    1_000_000_000,
                )
            integer(
                "measured_range_mm",
                self.measured_range_mm,
                0,
                1_000_000,
            )
            integer(
                "max_range_mm",
                self.max_range_mm,
                1,
                1_000_000,
            )
            _boolean(
                "endpoint_occupied",
                self.endpoint_occupied,
            )
        else:
            if (
                any(value is not None for value in metric_values)
                or self.endpoint_occupied is not None
                or self.direction != "FORWARD"
                or self.relation not in (
                    "NEAR_OBSTACLE",
                    "NO_NEAR_REFLECTION",
                )
                or not self.provisional
                or self.confidence_milli > 400
                or self.trusted_simulator_object_id is not None
            ):
                raise NavigationContractError(
                    "metric_claim_from_physical_ir",
                    "Physical IR ray must remain qualitative",
                )
            if self.raw_ir_proximity is not None:
                integer(
                    "raw_ir_proximity",
                    self.raw_ir_proximity,
                    0,
                    100,
                )
        _optional_identifier(
            "trusted_simulator_object_id",
            self.trusted_simulator_object_id,
        )

    def to_dict(self) -> Mapping[str, object]:
        common = {
            "direction": self.direction,
            "frame_id": self.frame_id,
            "source": self.source,
            "observed_at_ms": self.observed_at_ms,
            "captured_at_host_ms": self.captured_at_host_ms,
            "state_version": self.state_version,
            "world_model_version": self.world_model_version,
            "confidence_milli": self.confidence_milli,
            "provisional": self.provisional,
        }
        if self.source == "simulation_metric":
            common.update({
                "origin_x_mm": self.origin_x_mm,
                "origin_y_mm": self.origin_y_mm,
                "end_x_mm": self.end_x_mm,
                "end_y_mm": self.end_y_mm,
                "measured_range_mm": self.measured_range_mm,
                "max_range_mm": self.max_range_mm,
                "endpoint_occupied": self.endpoint_occupied,
                "trusted_simulator_object_id": (
                    self.trusted_simulator_object_id
                ),
            })
        else:
            common.update({
                "relation": self.relation,
                "raw_ir_proximity": self.raw_ir_proximity,
            })
        return common


@dataclass(frozen=True)
class OccupancyCell:
    """One immutable cell from a probabilistic occupancy accumulator."""

    grid_x: int
    grid_y: int
    center_x_mm: int
    center_y_mm: int
    classification: str
    occupancy_milli: int
    first_seen_at_ms: int
    last_seen_at_ms: int
    last_state_version: int
    last_world_model_version: int
    evidence_count: int
    free_evidence_count: int
    occupied_evidence_count: int
    provenance: Tuple[str, ...]
    quality: str = METRIC_FUSED

    def __post_init__(self) -> None:
        integer("grid_x", self.grid_x, -1_000_000, 1_000_000)
        integer("grid_y", self.grid_y, -1_000_000, 1_000_000)
        integer(
            "center_x_mm",
            self.center_x_mm,
            -1_000_000_000,
            1_000_000_000,
        )
        integer(
            "center_y_mm",
            self.center_y_mm,
            -1_000_000_000,
            1_000_000_000,
        )
        if self.classification not in (
            CELL_UNKNOWN,
            CELL_FREE,
            CELL_OCCUPIED,
        ):
            raise NavigationContractError(
                "invalid_cell_classification",
                "Occupancy cell classification is invalid",
            )
        integer(
            "occupancy_milli",
            self.occupancy_milli,
            -1_000,
            1_000,
        )
        if (
            self.classification == CELL_FREE
            and self.occupancy_milli >= 0
        ) or (
            self.classification == CELL_OCCUPIED
            and self.occupancy_milli <= 0
        ):
            raise NavigationContractError(
                "inconsistent_cell_classification",
                "Occupancy score and classification disagree",
            )
        integer(
            "first_seen_at_ms",
            self.first_seen_at_ms,
            0,
            _MAX_INT,
        )
        integer(
            "last_seen_at_ms",
            self.last_seen_at_ms,
            self.first_seen_at_ms,
            _MAX_INT,
        )
        integer(
            "last_state_version",
            self.last_state_version,
            1,
            _MAX_INT,
        )
        integer(
            "last_world_model_version",
            self.last_world_model_version,
            1,
            _MAX_INT,
        )
        integer("evidence_count", self.evidence_count, 1, _MAX_INT)
        integer(
            "free_evidence_count",
            self.free_evidence_count,
            0,
            self.evidence_count,
        )
        integer(
            "occupied_evidence_count",
            self.occupied_evidence_count,
            0,
            self.evidence_count,
        )
        if (
            self.free_evidence_count
            + self.occupied_evidence_count
            != self.evidence_count
        ):
            raise NavigationContractError(
                "invalid_cell_evidence_counts",
                "Cell evidence counts do not add up",
            )
        if self.quality != METRIC_FUSED:
            raise NavigationContractError(
                "invalid_cell_quality",
                "Occupancy cells require fused metric evidence",
            )
        if (
            not is_unique_identifier_tuple(
                "cell_provenance",
                self.provenance,
                require_nonempty=True,
            )
        ):
            raise NavigationContractError(
                "invalid_cell_provenance",
                "Occupancy cell provenance is invalid",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "grid_x": self.grid_x,
            "grid_y": self.grid_y,
            "center_x_mm": self.center_x_mm,
            "center_y_mm": self.center_y_mm,
            "classification": self.classification,
            "occupancy_milli": self.occupancy_milli,
            "first_seen_at_ms": self.first_seen_at_ms,
            "last_seen_at_ms": self.last_seen_at_ms,
            "last_state_version": self.last_state_version,
            "last_world_model_version": (
                self.last_world_model_version
            ),
            "evidence_count": self.evidence_count,
            "free_evidence_count": self.free_evidence_count,
            "occupied_evidence_count": (
                self.occupied_evidence_count
            ),
            "provenance": list(self.provenance),
            "quality": self.quality,
        }


@dataclass(frozen=True)
class QualitativeObstacleEvidence:
    """Provisional, non-metric evidence from the physical EV3 IR sensor."""

    evidence_id: str
    robot_id: str
    controller_instance_id: str
    frame_id: str
    source: str
    bearing: str
    relation: str
    observed_at_ms: int
    captured_at_host_ms: int
    state_version: int
    world_model_version: int
    confidence_milli: int
    raw_ir_proximity: Optional[int]
    provisional: bool = True
    quality: str = PROVISIONAL_QUALITATIVE

    def __post_init__(self) -> None:
        identifier("evidence_id", self.evidence_id)
        identifier("robot_id", self.robot_id)
        identifier(
            "controller_instance_id",
            self.controller_instance_id,
        )
        identifier("frame_id", self.frame_id, 96)
        if self.source != PHYSICAL_IR_REFLECTION:
            raise NavigationContractError(
                "invalid_qualitative_source",
                "Qualitative evidence source is invalid",
            )
        if self.bearing != "FORWARD":
            raise NavigationContractError(
                "invalid_qualitative_bearing",
                "Physical IR evidence is forward-relative",
            )
        if self.relation not in (
            "NEAR_OBSTACLE",
            "NO_NEAR_REFLECTION",
        ):
            raise NavigationContractError(
                "invalid_qualitative_relation",
                "Qualitative obstacle relation is invalid",
            )
        integer("observed_at_ms", self.observed_at_ms, 0, _MAX_INT)
        integer(
            "captured_at_host_ms",
            self.captured_at_host_ms,
            self.observed_at_ms,
            _MAX_INT,
        )
        integer("state_version", self.state_version, 1, _MAX_INT)
        integer(
            "world_model_version",
            self.world_model_version,
            1,
            _MAX_INT,
        )
        integer(
            "confidence_milli",
            self.confidence_milli,
            0,
            400,
        )
        if self.raw_ir_proximity is not None:
            integer(
                "raw_ir_proximity",
                self.raw_ir_proximity,
                0,
                100,
            )
        _boolean("provisional", self.provisional)
        if (
            not self.provisional
            or self.quality != PROVISIONAL_QUALITATIVE
        ):
            raise NavigationContractError(
                "untrusted_qualitative_quality",
                "Physical IR evidence must remain provisional",
            )

    def to_dict(self) -> Mapping[str, object]:
        # Deliberately no distance or millimetre field.
        return {
            "evidence_id": self.evidence_id,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "frame_id": self.frame_id,
            "source": self.source,
            "bearing": self.bearing,
            "relation": self.relation,
            "observed_at_ms": self.observed_at_ms,
            "captured_at_host_ms": self.captured_at_host_ms,
            "state_version": self.state_version,
            "world_model_version": self.world_model_version,
            "confidence_milli": self.confidence_milli,
            "raw_ir_proximity": self.raw_ir_proximity,
            "provisional": self.provisional,
            "quality": self.quality,
        }


@dataclass(frozen=True)
class ObjectHypothesis:
    """One connected component of occupied configuration-space cells.

    Simulator endpoints include the robot-radius collision envelope; bounds
    therefore describe navigation configuration space, not object surfaces.
    """

    hypothesis_id: str
    frame_id: str
    semantic_label: str
    min_x_mm: int
    min_y_mm: int
    max_x_mm: int
    max_y_mm: int
    centroid_x_mm: int
    centroid_y_mm: int
    cell_count: int
    first_seen_at_ms: int
    last_seen_at_ms: int
    evidence_count: int
    confidence_milli: int
    provenance: Tuple[str, ...]
    trusted_simulator_object_id: Optional[str] = None

    def __post_init__(self) -> None:
        identifier("hypothesis_id", self.hypothesis_id)
        identifier("frame_id", self.frame_id, 96)
        if self.semantic_label != SEMANTIC_UNKNOWN:
            raise NavigationContractError(
                "unsupported_semantic_label",
                "Spatial hypotheses are not semantically classified",
            )
        for name, value in (
            ("min_x_mm", self.min_x_mm),
            ("min_y_mm", self.min_y_mm),
            ("max_x_mm", self.max_x_mm),
            ("max_y_mm", self.max_y_mm),
            ("centroid_x_mm", self.centroid_x_mm),
            ("centroid_y_mm", self.centroid_y_mm),
        ):
            integer(name, value, -1_000_000_000, 1_000_000_000)
        if (
            self.min_x_mm > self.max_x_mm
            or self.min_y_mm > self.max_y_mm
            or not self.min_x_mm
            <= self.centroid_x_mm
            <= self.max_x_mm
            or not self.min_y_mm
            <= self.centroid_y_mm
            <= self.max_y_mm
        ):
            raise NavigationContractError(
                "invalid_hypothesis_geometry",
                "Object hypothesis geometry is inconsistent",
            )
        integer("cell_count", self.cell_count, 1, _MAX_INT)
        integer(
            "first_seen_at_ms",
            self.first_seen_at_ms,
            0,
            _MAX_INT,
        )
        integer(
            "last_seen_at_ms",
            self.last_seen_at_ms,
            self.first_seen_at_ms,
            _MAX_INT,
        )
        integer(
            "evidence_count",
            self.evidence_count,
            self.cell_count,
            _MAX_INT,
        )
        integer(
            "confidence_milli",
            self.confidence_milli,
            1,
            1_000,
        )
        if (
            not is_unique_identifier_tuple(
                "hypothesis_provenance",
                self.provenance,
                require_nonempty=True,
            )
        ):
            raise NavigationContractError(
                "invalid_hypothesis_provenance",
                "Object hypothesis provenance is invalid",
            )
        _optional_identifier(
            "trusted_simulator_object_id",
            self.trusted_simulator_object_id,
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "hypothesis_id": self.hypothesis_id,
            "frame_id": self.frame_id,
            "semantic_label": self.semantic_label,
            "bounds_mm": {
                "min_x": self.min_x_mm,
                "min_y": self.min_y_mm,
                "max_x": self.max_x_mm,
                "max_y": self.max_y_mm,
            },
            "centroid_mm": {
                "x": self.centroid_x_mm,
                "y": self.centroid_y_mm,
            },
            "cell_count": self.cell_count,
            "first_seen_at_ms": self.first_seen_at_ms,
            "last_seen_at_ms": self.last_seen_at_ms,
            "evidence_count": self.evidence_count,
            "confidence_milli": self.confidence_milli,
            "provenance": list(self.provenance),
            "trusted_simulator_object_id": (
                self.trusted_simulator_object_id
            ),
        }


@dataclass(frozen=True)
class ProvisionalObjectHypothesis:
    """One persistent physical-IR encounter without invented geometry.

    ``anchor_*`` records the trusted local-odometry pose from which a near
    reflection was observed.  It is not an object position.  The forward
    relation has no range, endpoint, metric bounds, or occupied cells.
    ``hypothesis_id`` is only a stable map-local handle for the encounter,
    never a claim that later reflections identify the same physical object.
    """

    hypothesis_id: str
    robot_id: str
    controller_instance_id: str
    frame_id: str
    semantic_label: str
    source: str
    geometry_kind: str
    bearing: str
    relation: str
    anchor_x_mm: int
    anchor_y_mm: int
    anchor_heading_mdeg: int
    first_seen_at_ms: int
    last_seen_at_ms: int
    last_state_version: int
    last_world_model_version: int
    evidence_count: int
    confidence_milli: int
    provenance: Tuple[str, ...]
    provisional: bool = True
    quality: str = PROVISIONAL_QUALITATIVE

    def __post_init__(self) -> None:
        identifier("hypothesis_id", self.hypothesis_id)
        identifier("robot_id", self.robot_id)
        identifier(
            "controller_instance_id",
            self.controller_instance_id,
        )
        identifier("frame_id", self.frame_id, 96)
        if self.semantic_label != SEMANTIC_UNKNOWN:
            raise NavigationContractError(
                "unsupported_semantic_label",
                "Spatial hypotheses are not semantically classified",
            )
        if self.source != PHYSICAL_IR_REFLECTION:
            raise NavigationContractError(
                "invalid_provisional_hypothesis_source",
                "Provisional hypothesis source is invalid",
            )
        if self.geometry_kind != QUALITATIVE_FORWARD_ENVELOPE:
            raise NavigationContractError(
                "invalid_provisional_hypothesis_geometry",
                "Physical IR hypothesis must remain non-metric",
            )
        if self.bearing != "FORWARD" or self.relation != "NEAR_OBSTACLE":
            raise NavigationContractError(
                "invalid_provisional_hypothesis_relation",
                "Physical IR hypothesis requires a forward near reflection",
            )
        integer(
            "anchor_x_mm",
            self.anchor_x_mm,
            -1_000_000,
            1_000_000,
        )
        integer(
            "anchor_y_mm",
            self.anchor_y_mm,
            -1_000_000,
            1_000_000,
        )
        integer(
            "anchor_heading_mdeg",
            self.anchor_heading_mdeg,
            -180_000,
            179_999,
        )
        integer("first_seen_at_ms", self.first_seen_at_ms, 0, _MAX_INT)
        integer(
            "last_seen_at_ms",
            self.last_seen_at_ms,
            self.first_seen_at_ms,
            _MAX_INT,
        )
        integer("last_state_version", self.last_state_version, 1, _MAX_INT)
        integer(
            "last_world_model_version",
            self.last_world_model_version,
            1,
            _MAX_INT,
        )
        integer("evidence_count", self.evidence_count, 1, _MAX_INT)
        integer("confidence_milli", self.confidence_milli, 1, 400)
        if (
            not is_unique_identifier_tuple(
                "hypothesis_provenance",
                self.provenance,
                required_members=(
                    LOCAL_ODOMETRY_POSE,
                    PHYSICAL_IR_REFLECTION,
                ),
            )
        ):
            raise NavigationContractError(
                "invalid_hypothesis_provenance",
                "Provisional hypothesis provenance is invalid",
            )
        _boolean("provisional", self.provisional)
        if (
            not self.provisional
            or self.quality != PROVISIONAL_QUALITATIVE
        ):
            raise NavigationContractError(
                "untrusted_provisional_hypothesis",
                "Physical IR hypothesis must remain provisional",
            )

    def to_dict(self) -> Mapping[str, object]:
        # The anchor is the observing robot pose, not an object coordinate.
        return {
            "hypothesis_id": self.hypothesis_id,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "frame_id": self.frame_id,
            "semantic_label": self.semantic_label,
            "source": self.source,
            "geometry_kind": self.geometry_kind,
            "bounds_mm": None,
            "anchor_pose": {
                "x_mm": self.anchor_x_mm,
                "y_mm": self.anchor_y_mm,
                "heading_mdeg": self.anchor_heading_mdeg,
            },
            "bearing": self.bearing,
            "relation": self.relation,
            "first_seen_at_ms": self.first_seen_at_ms,
            "last_seen_at_ms": self.last_seen_at_ms,
            "last_state_version": self.last_state_version,
            "last_world_model_version": (
                self.last_world_model_version
            ),
            "evidence_count": self.evidence_count,
            "confidence_milli": self.confidence_milli,
            "provenance": list(self.provenance),
            "provisional": self.provisional,
            "quality": self.quality,
        }


SpatialObjectHypothesis = Union[
    ObjectHypothesis,
    ProvisionalObjectHypothesis,
]


@dataclass(frozen=True)
class SpatialMapSnapshot:
    """A deeply immutable point-in-time view of one bounded grid."""

    map_id: str
    robot_id: str
    controller_instance_id: str
    frame_id: str
    frame_kind: str
    map_quality: str
    evidence_sources: Tuple[str, ...]
    resolution_mm: int
    capacity: int
    map_version: int
    created_at_ms: int
    updated_at_ms: int
    last_observed_at_ms: int
    based_on_state_version: int
    based_on_world_model_version: int
    cells_evicted: int
    bounds: Optional[SpatialBounds]
    latest_robot_pose: Optional[SpatialRobotPose]
    sensor_rays: Tuple[SpatialSensorRay, ...]
    cells: Tuple[OccupancyCell, ...]
    qualitative_evidence: Tuple[QualitativeObstacleEvidence, ...]
    object_hypotheses: Tuple[SpatialObjectHypothesis, ...]
    pose_history: Tuple[SpatialRobotPose, ...] = ()
    pose_history_evicted: int = 0
    collision_geometry: Optional[SpatialCollisionGeometry] = None
    scan_evidence_history: Tuple[SpatialScanEvidence, ...] = ()

    def __post_init__(self) -> None:
        identifier("map_id", self.map_id)
        identifier("robot_id", self.robot_id)
        identifier(
            "controller_instance_id",
            self.controller_instance_id,
        )
        identifier("frame_id", self.frame_id, 96)
        if self.frame_kind not in (
            SIMULATION_WORLD,
            LOCAL_ODOMETRY,
        ):
            raise NavigationContractError(
                "invalid_spatial_frame_kind",
                "Spatial map frame kind is invalid",
            )
        if self.map_quality not in (
            MAP_EMPTY,
            MAP_SIMULATION_METRIC,
            MAP_PROVISIONAL_IR,
            MAP_METRIC_WITH_PROVISIONAL_IR,
        ):
            raise NavigationContractError(
                "invalid_spatial_map_quality",
                "Spatial map quality is invalid",
            )
        validate_evidence_sources(self.evidence_sources, self.map_quality)
        integer("resolution_mm", self.resolution_mm, 1, 100_000)
        integer("capacity", self.capacity, 1, 1_000_000)
        integer("map_version", self.map_version, 0, _MAX_INT)
        integer("created_at_ms", self.created_at_ms, 0, _MAX_INT)
        integer(
            "updated_at_ms",
            self.updated_at_ms,
            self.created_at_ms,
            _MAX_INT,
        )
        integer(
            "last_observed_at_ms",
            self.last_observed_at_ms,
            0,
            self.updated_at_ms,
        )
        integer(
            "based_on_state_version",
            self.based_on_state_version,
            0,
            _MAX_INT,
        )
        integer(
            "based_on_world_model_version",
            self.based_on_world_model_version,
            0,
            _MAX_INT,
        )
        integer("cells_evicted", self.cells_evicted, 0, _MAX_INT)
        if self.bounds is not None and not isinstance(
            self.bounds,
            SpatialBounds,
        ):
            raise NavigationContractError(
                "invalid_spatial_bounds",
                "Spatial map bounds are invalid",
            )
        if self.latest_robot_pose is not None:
            if (
                not isinstance(
                    self.latest_robot_pose,
                    SpatialRobotPose,
                )
                or self.latest_robot_pose.frame_id != self.frame_id
                or self.latest_robot_pose.state_version
                != self.based_on_state_version
                or self.latest_robot_pose.world_model_version
                != self.based_on_world_model_version
                or self.latest_robot_pose.observed_at_ms
                > self.updated_at_ms
                or self.latest_robot_pose.captured_at_host_ms
                > self.updated_at_ms
            ):
                raise NavigationContractError(
                    "invalid_spatial_robot_pose",
                    "Spatial robot pose is invalid",
                )
        integer(
            "pose_history_evicted",
            self.pose_history_evicted,
            0,
            _MAX_INT,
        )
        if (
            not isinstance(self.pose_history, tuple)
            or len(self.pose_history) > MAX_POSE_HISTORY
            or any(
                not isinstance(item, SpatialRobotPose)
                for item in self.pose_history
            )
        ):
            raise NavigationContractError(
                "invalid_pose_history",
                "Spatial pose history is invalid",
            )
        if any(
            item.frame_id != self.frame_id
            or item.state_version > self.based_on_state_version
            or item.world_model_version
            != self.based_on_world_model_version
            or item.observed_at_ms > self.updated_at_ms
            or item.captured_at_host_ms > self.updated_at_ms
            for item in self.pose_history
        ):
            raise NavigationContractError(
                "inconsistent_pose_history",
                "Spatial pose history crosses the map boundary",
            )
        for previous, current in zip(
            self.pose_history,
            self.pose_history[1:],
        ):
            if (
                current.state_version <= previous.state_version
                or current.observed_at_ms < previous.observed_at_ms
                or current.captured_at_host_ms
                < previous.captured_at_host_ms
                or (
                    current.x_mm,
                    current.y_mm,
                    current.heading_mdeg,
                )
                == (
                    previous.x_mm,
                    previous.y_mm,
                    previous.heading_mdeg,
                )
            ):
                raise NavigationContractError(
                    "inconsistent_pose_history_order",
                    "Spatial pose history is not a monotonic trajectory",
                )
        if (
            self.pose_history
            and self.latest_robot_pose is not None
            and (
                self.pose_history[-1].x_mm,
                self.pose_history[-1].y_mm,
                self.pose_history[-1].heading_mdeg,
            )
            != (
                self.latest_robot_pose.x_mm,
                self.latest_robot_pose.y_mm,
                self.latest_robot_pose.heading_mdeg,
            )
        ):
            raise NavigationContractError(
                "inconsistent_latest_pose_history",
                "Latest robot pose does not end the retained trajectory",
            )
        if (
            self.collision_geometry is not None
            and not isinstance(
                self.collision_geometry,
                SpatialCollisionGeometry,
            )
        ):
            raise NavigationContractError(
                "invalid_collision_geometry",
                "Spatial collision geometry is invalid",
            )
        if (
            not isinstance(self.scan_evidence_history, tuple)
            or len(self.scan_evidence_history)
            > MAX_SPATIAL_SCAN_EVIDENCE
            or any(
                not isinstance(item, SpatialScanEvidence)
                for item in self.scan_evidence_history
            )
            or len({
                item.scan_id for item in self.scan_evidence_history
            }) != len(self.scan_evidence_history)
        ):
            raise NavigationContractError(
                "invalid_spatial_scan_history",
                "Spatial scan history is invalid",
            )
        if self.scan_evidence_history and (
            self.frame_kind != LOCAL_ODOMETRY
            or PHYSICAL_IR_REFLECTION not in self.evidence_sources
            or self.collision_geometry is None
            or any(
                item.frame_id != self.frame_id
                or item.completed_at_unix_ms > self.updated_at_ms
                for item in self.scan_evidence_history
            )
        ):
            raise NavigationContractError(
                "inconsistent_spatial_scan_history",
                "Spatial scan history crosses map trust boundaries",
            )
        if (
            not isinstance(self.sensor_rays, tuple)
            or any(
                not isinstance(ray, SpatialSensorRay)
                for ray in self.sensor_rays
            )
        ):
            raise NavigationContractError(
                "invalid_spatial_sensor_rays",
                "Spatial sensor rays are invalid",
            )
        if any(
            (
                ray.source == "simulation_metric"
                and ray.frame_id != self.frame_id
            )
            or ray.source not in self.evidence_sources
            or ray.state_version > self.based_on_state_version
            or ray.world_model_version
            != self.based_on_world_model_version
            or ray.observed_at_ms > self.last_observed_at_ms
            or ray.captured_at_host_ms > self.updated_at_ms
            for ray in self.sensor_rays
        ):
            raise NavigationContractError(
                "inconsistent_spatial_sensor_rays",
                "Spatial sensor rays cross the map frame or version",
            )
        if (
            not isinstance(self.cells, tuple)
            or len(self.cells) > self.capacity
            or any(
                not isinstance(cell, OccupancyCell)
                for cell in self.cells
            )
            or len(
                {(cell.grid_x, cell.grid_y) for cell in self.cells}
            )
            != len(self.cells)
        ):
            raise NavigationContractError(
                "invalid_spatial_cells",
                "Spatial map cells are invalid",
            )
        if any(
            cell.last_state_version > self.based_on_state_version
            or cell.last_world_model_version
            != self.based_on_world_model_version
            or cell.last_seen_at_ms > self.last_observed_at_ms
            for cell in self.cells
        ):
            raise NavigationContractError(
                "inconsistent_spatial_cells",
                "Spatial cells cross the map version boundary",
            )
        if self.cells and "simulation_metric" not in self.evidence_sources:
            raise NavigationContractError(
                "inconsistent_spatial_cell_source",
                "Metric cells require simulator metric evidence",
            )
        if bool(self.cells) != (self.bounds is not None):
            raise NavigationContractError(
                "inconsistent_spatial_bounds",
                "Bounds must exist exactly when cells exist",
            )
        if (
            not isinstance(self.qualitative_evidence, tuple)
            or any(
                not isinstance(item, QualitativeObstacleEvidence)
                for item in self.qualitative_evidence
            )
        ):
            raise NavigationContractError(
                "invalid_qualitative_evidence",
                "Spatial qualitative evidence is invalid",
            )
        if any(
            item.robot_id != self.robot_id
            or item.controller_instance_id
            != self.controller_instance_id
            or item.state_version > self.based_on_state_version
            or item.world_model_version
            != self.based_on_world_model_version
            or item.observed_at_ms > self.last_observed_at_ms
            or item.captured_at_host_ms > self.updated_at_ms
            for item in self.qualitative_evidence
        ):
            raise NavigationContractError(
                "inconsistent_qualitative_evidence",
                "Qualitative evidence crosses map identity or version",
            )
        if (
            self.qualitative_evidence
            and PHYSICAL_IR_REFLECTION not in self.evidence_sources
        ):
            raise NavigationContractError(
                "inconsistent_qualitative_source",
                "Qualitative evidence requires physical IR provenance",
            )
        if (
            not isinstance(self.object_hypotheses, tuple)
            or any(
                not isinstance(
                    item,
                    (ObjectHypothesis, ProvisionalObjectHypothesis),
                )
                for item in self.object_hypotheses
            )
            or len(
                {
                    item.hypothesis_id
                    for item in self.object_hypotheses
                }
            )
            != len(self.object_hypotheses)
        ):
            raise NavigationContractError(
                "invalid_object_hypotheses",
                "Spatial object hypotheses are invalid",
            )
        if any(
            item.frame_id != self.frame_id
            or item.last_seen_at_ms > self.last_observed_at_ms
            for item in self.object_hypotheses
        ):
            raise NavigationContractError(
                "inconsistent_object_hypotheses",
                "Object hypotheses cross the map frame or time boundary",
            )
        metric_hypotheses = tuple(
            item
            for item in self.object_hypotheses
            if isinstance(item, ObjectHypothesis)
        )
        provisional_hypotheses = tuple(
            item
            for item in self.object_hypotheses
            if isinstance(item, ProvisionalObjectHypothesis)
        )
        if metric_hypotheses and (
            "simulation_metric" not in self.evidence_sources
        ):
            raise NavigationContractError(
                "inconsistent_hypothesis_source",
                "Metric hypotheses require simulator metric evidence",
            )
        if provisional_hypotheses and (
            self.frame_kind != LOCAL_ODOMETRY
            or PHYSICAL_IR_REFLECTION not in self.evidence_sources
            or any(
                item.robot_id != self.robot_id
                or item.controller_instance_id
                != self.controller_instance_id
                or item.last_state_version
                > self.based_on_state_version
                or item.last_world_model_version
                != self.based_on_world_model_version
                for item in provisional_hypotheses
            )
        ):
            raise NavigationContractError(
                "inconsistent_provisional_hypothesis",
                "Provisional hypotheses cross map trust boundaries",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": SPATIAL_MAP_SNAPSHOT_SCHEMA,
            "map_id": self.map_id,
            "robot_id": self.robot_id,
            "controller_instance_id": self.controller_instance_id,
            "frame_id": self.frame_id,
            "frame_kind": self.frame_kind,
            "map_quality": self.map_quality,
            "evidence_sources": list(self.evidence_sources),
            "resolution_mm": self.resolution_mm,
            "capacity": self.capacity,
            "map_version": self.map_version,
            "created_at_ms": self.created_at_ms,
            "updated_at_ms": self.updated_at_ms,
            "last_observed_at_ms": self.last_observed_at_ms,
            "based_on_state_version": self.based_on_state_version,
            "based_on_world_model_version": (
                self.based_on_world_model_version
            ),
            "cells_evicted": self.cells_evicted,
            "bounds": (
                None if self.bounds is None else self.bounds.to_dict()
            ),
            "latest_robot_pose": (
                None
                if self.latest_robot_pose is None
                else self.latest_robot_pose.to_dict()
            ),
            "pose_history": [
                item.to_dict() for item in self.pose_history
            ],
            "pose_history_evicted": self.pose_history_evicted,
            "collision_geometry": (
                None
                if self.collision_geometry is None
                else self.collision_geometry.to_dict()
            ),
            "scan_evidence_history": [
                item.to_dict() for item in self.scan_evidence_history
            ],
            "sensor_rays": [
                ray.to_dict() for ray in self.sensor_rays
            ],
            "cells": [cell.to_dict() for cell in self.cells],
            "qualitative_evidence": [
                item.to_dict() for item in self.qualitative_evidence
            ],
            "object_hypotheses": [
                item.to_dict() for item in self.object_hypotheses
            ],
        }



@dataclass(frozen=True)
class SpatialMapUpdate:
    """Result of accepting or ignoring one navigation state version."""

    applied: bool
    reason_code: str
    state_version: int
    world_model_version: int
    map_version: int
    cells_touched: int
    cells_evicted: int
    qualitative_evidence_added: int

    def __post_init__(self) -> None:
        _boolean("applied", self.applied)
        if self.reason_code not in (
            UPDATE_APPLIED,
            STALE_STATE_VERSION,
            STALE_WORLD_MODEL_VERSION,
            STALE_TIMESTAMP,
        ):
            raise NavigationContractError(
                "invalid_spatial_update_reason",
                "Spatial update reason is invalid",
            )
        if self.applied != (self.reason_code == UPDATE_APPLIED):
            raise NavigationContractError(
                "inconsistent_spatial_update",
                "Spatial update status and reason disagree",
            )
        integer("state_version", self.state_version, 1, _MAX_INT)
        integer(
            "world_model_version",
            self.world_model_version,
            1,
            _MAX_INT,
        )
        integer("map_version", self.map_version, 0, _MAX_INT)
        integer("cells_touched", self.cells_touched, 0, _MAX_INT)
        integer("cells_evicted", self.cells_evicted, 0, _MAX_INT)
        integer(
            "qualitative_evidence_added",
            self.qualitative_evidence_added,
            0,
            1,
        )
        if not self.applied and (
            self.cells_touched
            or self.cells_evicted
            or self.qualitative_evidence_added
        ):
            raise NavigationContractError(
                "mutating_ignored_spatial_update",
                "Ignored spatial updates cannot report mutations",
            )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": SPATIAL_MAP_UPDATE_SCHEMA,
            "applied": self.applied,
            "reason_code": self.reason_code,
            "state_version": self.state_version,
            "world_model_version": self.world_model_version,
            "map_version": self.map_version,
            "cells_touched": self.cells_touched,
            "cells_evicted": self.cells_evicted,
            "qualitative_evidence_added": (
                self.qualitative_evidence_added
            ),
        }
