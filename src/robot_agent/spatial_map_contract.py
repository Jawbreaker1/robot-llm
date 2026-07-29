"""Immutable, JSON-friendly contracts for uncertain spatial mapping.

The map contains geometric occupancy evidence and qualitative proximity
evidence, but no semantic classifier output.  Object identities are opaque
host hypotheses; the only semantic label in this first slice is ``UNKNOWN``.
"""

from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

from .navigation_contract import (
    NavigationContractError,
    identifier,
    integer,
)


SPATIAL_MAP_SNAPSHOT_SCHEMA = "robot-spatial-map-snapshot/v1"
SPATIAL_MAP_UPDATE_SCHEMA = "robot-spatial-map-update/v1"
DASHBOARD_SPATIAL_MAP_SCHEMA = "robot-spatial-map/v1"

CELL_UNKNOWN = "UNKNOWN"
CELL_FREE = "FREE"
CELL_OCCUPIED = "OCCUPIED"

SEMANTIC_UNKNOWN = "UNKNOWN"
METRIC_FUSED = "METRIC_FUSED"
PROVISIONAL_QUALITATIVE = "PROVISIONAL_QUALITATIVE"
PHYSICAL_IR_REFLECTION = "physical_ir_reflection"

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


def _optional_identifier(
    name: str,
    value: Optional[str],
    maximum: int = 128,
) -> Optional[str]:
    if value is not None:
        identifier(name, value, maximum)
    return value


def _boolean(name: str, value: bool) -> bool:
    if type(value) is not bool:
        raise NavigationContractError(
            "invalid_boolean",
            "{} is invalid".format(name),
        )
    return value


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
            not isinstance(self.provenance, tuple)
            or not self.provenance
            or any(
                identifier("cell_provenance", item, 96) != item
                for item in self.provenance
            )
            or len(set(self.provenance)) != len(self.provenance)
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
            not isinstance(self.provenance, tuple)
            or not self.provenance
            or any(
                identifier("hypothesis_provenance", item, 96) != item
                for item in self.provenance
            )
            or len(set(self.provenance)) != len(self.provenance)
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
    object_hypotheses: Tuple[ObjectHypothesis, ...]

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
        if (
            not isinstance(self.evidence_sources, tuple)
            or any(
                source not in (
                    "simulation_metric",
                    PHYSICAL_IR_REFLECTION,
                )
                for source in self.evidence_sources
            )
            or len(set(self.evidence_sources))
            != len(self.evidence_sources)
        ):
            raise NavigationContractError(
                "invalid_spatial_evidence_sources",
                "Spatial map evidence sources are invalid",
            )
        expected_sources = {
            MAP_EMPTY: (),
            MAP_SIMULATION_METRIC: ("simulation_metric",),
            MAP_PROVISIONAL_IR: (PHYSICAL_IR_REFLECTION,),
            MAP_METRIC_WITH_PROVISIONAL_IR: (
                PHYSICAL_IR_REFLECTION,
                "simulation_metric",
            ),
        }[self.map_quality]
        if tuple(sorted(self.evidence_sources)) != expected_sources:
            raise NavigationContractError(
                "inconsistent_spatial_map_quality",
                "Map quality and evidence sources disagree",
            )
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
                not isinstance(item, ObjectHypothesis)
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
        if (
            self.object_hypotheses
            and "simulation_metric" not in self.evidence_sources
        ):
            raise NavigationContractError(
                "inconsistent_hypothesis_source",
                "Object hypotheses require simulator metric evidence",
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
