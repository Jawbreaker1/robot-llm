"""Thread-safe, bounded occupancy mapping from typed navigation snapshots.

Trusted simulator ranges update a metric configuration-space grid.  Their
endpoints are inflated by the simulated robot radius and are not claims about
an object's physical surface.  Physical EV3 IR reflections are retained
separately as provisional qualitative evidence and can neither clear nor
occupy a metric cell.
"""

from collections import OrderedDict, deque
from dataclasses import dataclass, field
import threading
from typing import Deque, Optional, Set, Tuple

from .navigation_contract import (
    NavigationContractError,
    identifier,
    integer,
)
from .navigation_state import NavigationSnapshot
from .spatial_map_contract import (
    CELL_FREE,
    CELL_OCCUPIED,
    CELL_UNKNOWN,
    LOCAL_ODOMETRY,
    LOCAL_ODOMETRY_POSE,
    MAX_POSE_HISTORY,
    MAP_EMPTY,
    MAP_METRIC_WITH_PROVISIONAL_IR,
    MAP_PROVISIONAL_IR,
    MAP_SIMULATION_METRIC,
    METRIC_FUSED,
    PHYSICAL_IR_REFLECTION,
    QUALITATIVE_FORWARD_ENVELOPE,
    PROVISIONAL_QUALITATIVE,
    SEMANTIC_UNKNOWN,
    SIMULATION_WORLD,
    STALE_STATE_VERSION,
    STALE_TIMESTAMP,
    STALE_WORLD_MODEL_VERSION,
    UPDATE_APPLIED,
    OccupancyCell,
    ProvisionalObjectHypothesis,
    QualitativeObstacleEvidence,
    SpatialBounds,
    SpatialMapSnapshot,
    SpatialMapUpdate,
    SpatialRobotPose,
    SpatialSensorRay,
)
from .spatial_objects import (
    OccupiedCellEvidence,
    connected_object_hypotheses,
    provisional_object_hypothesis_id,
)
from .spatial_sensor_model import (
    ROBOT_BASE_FRAME,
    SIMULATION_CONFIGURATION_SPACE,
    SIMULATION_METRIC,
    metric_ray_geometry,
    metric_sensor_ray,
    qualitative_evidence_for,
    qualitative_sensor_ray,
)


@dataclass(frozen=True)
class SpatialMappingPolicy:
    """Host-owned geometric limits and evidence weights."""

    resolution_mm: int = 50
    range_max_mm: int = 1_000
    max_cells: int = 4_096
    max_qualitative_evidence: int = 128
    max_provisional_object_hypotheses: int = 128
    free_evidence_step_milli: int = 250
    occupied_evidence_step_milli: int = 650
    free_threshold_milli: int = 250
    occupied_threshold_milli: int = 500
    physical_ir_confidence_milli: int = 250

    def __post_init__(self) -> None:
        integer("resolution_mm", self.resolution_mm, 1, 100_000)
        integer("range_max_mm", self.range_max_mm, 1, 1_000_000)
        integer("max_cells", self.max_cells, 1, 1_000_000)
        integer(
            "max_qualitative_evidence",
            self.max_qualitative_evidence,
            1,
            100_000,
        )
        integer(
            "max_provisional_object_hypotheses",
            self.max_provisional_object_hypotheses,
            1,
            100_000,
        )
        integer(
            "free_evidence_step_milli",
            self.free_evidence_step_milli,
            1,
            1_000,
        )
        integer(
            "occupied_evidence_step_milli",
            self.occupied_evidence_step_milli,
            1,
            1_000,
        )
        integer(
            "free_threshold_milli",
            self.free_threshold_milli,
            1,
            1_000,
        )
        integer(
            "occupied_threshold_milli",
            self.occupied_threshold_milli,
            1,
            1_000,
        )
        integer(
            "physical_ir_confidence_milli",
            self.physical_ir_confidence_milli,
            1,
            400,
        )


@dataclass
class _CellAccumulator:
    grid_x: int
    grid_y: int
    occupancy_milli: int
    first_seen_at_ms: int
    last_seen_at_ms: int
    last_state_version: int
    last_world_model_version: int
    evidence_count: int = 0
    free_evidence_count: int = 0
    occupied_evidence_count: int = 0
    supported_occupied_evidence_count: int = 0
    first_occupied_at_ms: Optional[int] = None
    last_occupied_at_ms: Optional[int] = None
    provenance: Set[str] = field(default_factory=set)
    trusted_simulator_object_ids: Set[str] = field(
        default_factory=set
    )


@dataclass
class _SnapshotCellEvidence:
    """Correlated ray evidence reduced to one cell update per snapshot."""

    occupied: bool = False
    provenance: Set[str] = field(default_factory=set)
    trusted_simulator_object_ids: Set[str] = field(
        default_factory=set
    )


@dataclass
class _ProvisionalObjectAccumulator:
    """Persistent state for one contiguous physical-IR near episode."""

    hypothesis_id: str
    anchor_x_mm: int
    anchor_y_mm: int
    anchor_heading_mdeg: int
    first_seen_at_ms: int
    last_seen_at_ms: int
    last_state_version: int
    last_world_model_version: int
    evidence_count: int = 1


class BoundedOccupancyGrid:
    """One identity-bound spatial map with atomic ingest and snapshots."""

    def __init__(
        self,
        map_id: str,
        robot_id: str,
        controller_instance_id: str,
        frame_id: str,
        frame_kind: str,
        policy: SpatialMappingPolicy = SpatialMappingPolicy(),
        created_at_ms: int = 0,
        qualitative_frame_id: str = ROBOT_BASE_FRAME,
    ):
        identifier("map_id", map_id)
        identifier("robot_id", robot_id)
        identifier(
            "controller_instance_id",
            controller_instance_id,
        )
        identifier("frame_id", frame_id, 96)
        if frame_kind not in (SIMULATION_WORLD, LOCAL_ODOMETRY):
            raise NavigationContractError(
                "invalid_spatial_frame_kind",
                "Spatial map frame kind is invalid",
            )
        identifier(
            "qualitative_frame_id",
            qualitative_frame_id,
            96,
        )
        if not isinstance(policy, SpatialMappingPolicy):
            raise NavigationContractError(
                "invalid_spatial_mapping_policy",
                "Spatial mapping policy is invalid",
            )
        integer("created_at_ms", created_at_ms, 0, 2**63 - 1)
        self.map_id = map_id
        self.robot_id = robot_id
        self.controller_instance_id = controller_instance_id
        self.frame_id = frame_id
        self.frame_kind = frame_kind
        self.qualitative_frame_id = qualitative_frame_id
        self.policy = policy
        self._cells: "OrderedDict[Tuple[int, int], _CellAccumulator]" = (
            OrderedDict()
        )
        self._qualitative: Deque[QualitativeObstacleEvidence] = deque(
            maxlen=policy.max_qualitative_evidence
        )
        self._provisional_objects: "OrderedDict[str, _ProvisionalObjectAccumulator]" = (
            OrderedDict()
        )
        self._active_provisional_object_id: Optional[str] = None
        self._evidence_sources: Set[str] = set()
        self._latest_robot_pose: Optional[SpatialRobotPose] = None
        self._pose_history: Deque[SpatialRobotPose] = deque(
            maxlen=MAX_POSE_HISTORY
        )
        self._pose_history_evicted = 0
        self._sensor_rays: Tuple[SpatialSensorRay, ...] = ()
        self._map_version = 0
        self._created_at_ms = created_at_ms
        self._updated_at_ms = created_at_ms
        self._last_observed_at_ms = 0
        self._last_state_version = 0
        self._last_world_model_version = 0
        self._cells_evicted = 0
        self._lock = threading.RLock()

    def _classification_for(self, occupancy_milli: int) -> str:
        if occupancy_milli >= self.policy.occupied_threshold_milli:
            return CELL_OCCUPIED
        if occupancy_milli <= -self.policy.free_threshold_milli:
            return CELL_FREE
        return CELL_UNKNOWN

    def _touch_metric_cell_locked(
        self,
        coordinate: Tuple[int, int],
        occupied: bool,
        snapshot: NavigationSnapshot,
        provenance: Set[str],
        trusted_simulator_object_ids: Set[str],
    ) -> int:
        evicted = 0
        record = self._cells.get(coordinate)
        if record is None:
            if len(self._cells) >= self.policy.max_cells:
                self._cells.popitem(last=False)
                self._cells_evicted += 1
                evicted = 1
            record = _CellAccumulator(
                grid_x=coordinate[0],
                grid_y=coordinate[1],
                occupancy_milli=0,
                first_seen_at_ms=snapshot.clearance.observed_at_ms,
                last_seen_at_ms=snapshot.clearance.observed_at_ms,
                last_state_version=snapshot.state_version,
                last_world_model_version=(
                    snapshot.world_model_version
                ),
            )
            self._cells[coordinate] = record
        else:
            self._cells.move_to_end(coordinate)

        record.evidence_count += 1
        record.last_seen_at_ms = snapshot.clearance.observed_at_ms
        record.last_state_version = snapshot.state_version
        record.last_world_model_version = (
            snapshot.world_model_version
        )
        record.provenance.update(provenance)
        if occupied:
            record.occupied_evidence_count += 1
            record.supported_occupied_evidence_count += 1
            record.occupancy_milli = min(
                1_000,
                record.occupancy_milli
                + self.policy.occupied_evidence_step_milli,
            )
            if record.first_occupied_at_ms is None:
                record.first_occupied_at_ms = (
                    snapshot.clearance.observed_at_ms
                )
            record.last_occupied_at_ms = (
                snapshot.clearance.observed_at_ms
            )
            record.trusted_simulator_object_ids.update(
                trusted_simulator_object_ids
            )
        else:
            record.free_evidence_count += 1
            record.occupancy_milli = max(
                -1_000,
                record.occupancy_milli
                - self.policy.free_evidence_step_milli,
            )
            if (
                record.occupancy_milli
                <= -self.policy.free_threshold_milli
            ):
                # Positive free-space evidence disproves the old endpoint
                # identity.  A future hit must establish identity again.
                record.trusted_simulator_object_ids.clear()
                record.supported_occupied_evidence_count = 0
                record.first_occupied_at_ms = None
                record.last_occupied_at_ms = None
        return evicted

    def _ingest_metric_locked(
        self,
        snapshot: NavigationSnapshot,
    ) -> Tuple[int, int, Tuple[SpatialSensorRay, ...]]:
        evidence = snapshot.clearance
        rays = (
            (
                "FORWARD",
                0,
                evidence.forward_mm,
                evidence.forward_object_id,
            ),
            ("LEFT", 45, evidence.left_mm, None),
            ("RIGHT", -45, evidence.right_mm, None),
        )
        by_cell = {}
        sensor_rays = []
        for direction, angle_offset, measured_mm, object_id in rays:
            if measured_mm is None:
                continue
            geometry = metric_ray_geometry(
                snapshot,
                angle_offset,
                measured_mm,
                self.policy.resolution_mm,
                self.policy.range_max_mm,
            )
            cells = geometry.cells
            endpoint_occupied = geometry.endpoint_occupied
            free_cells = cells[:-1] if endpoint_occupied else cells
            provenance = "{}:{}".format(
                SIMULATION_CONFIGURATION_SPACE,
                direction,
            )
            for coordinate in free_cells:
                cell_evidence = by_cell.setdefault(
                    coordinate,
                    _SnapshotCellEvidence(),
                )
                cell_evidence.provenance.add(provenance)
            if endpoint_occupied:
                endpoint = cells[-1]
                cell_evidence = by_cell.setdefault(
                    endpoint,
                    _SnapshotCellEvidence(),
                )
                # Rays from one snapshot are correlated.  A measured endpoint
                # dominates another ray merely traversing the same coarse
                # cell, and the cell is updated exactly once below.
                cell_evidence.occupied = True
                cell_evidence.provenance.add(provenance)
                if object_id is not None:
                    cell_evidence.trusted_simulator_object_ids.add(
                        object_id
                    )
            sensor_rays.append(metric_sensor_ray(
                snapshot=snapshot,
                direction=direction,
                frame_id=self.frame_id,
                measured_mm=measured_mm,
                range_max_mm=self.policy.range_max_mm,
                geometry=geometry,
                trusted_simulator_object_id=object_id,
            ))
        evicted = 0
        for coordinate, cell_evidence in by_cell.items():
            evicted += self._touch_metric_cell_locked(
                coordinate,
                cell_evidence.occupied,
                snapshot,
                cell_evidence.provenance,
                cell_evidence.trusted_simulator_object_ids,
            )
        return len(by_cell), evicted, tuple(sensor_rays)

    def _reset_evidence_locked(self) -> None:
        """Invalidate evidence after an authoritative generation change."""

        self._cells.clear()
        self._qualitative.clear()
        self._provisional_objects.clear()
        self._active_provisional_object_id = None
        self._evidence_sources.clear()
        self._sensor_rays = ()
        self._latest_robot_pose = None
        self._pose_history.clear()
        self._pose_history_evicted = 0

    def _retain_pose_locked(self, pose: SpatialRobotPose) -> None:
        """Retain changed pose geometry without stationary duplicates."""

        if self._pose_history:
            previous = self._pose_history[-1]
            if (
                pose.x_mm,
                pose.y_mm,
                pose.heading_mdeg,
            ) == (
                previous.x_mm,
                previous.y_mm,
                previous.heading_mdeg,
            ):
                return
        if len(self._pose_history) == MAX_POSE_HISTORY:
            self._pose_history_evicted += 1
        self._pose_history.append(pose)

    def _ingest_provisional_object_locked(
        self,
        snapshot: NavigationSnapshot,
        evidence: QualitativeObstacleEvidence,
    ) -> None:
        """Retain one map-local hypothesis per contiguous near episode."""

        if self.frame_kind != LOCAL_ODOMETRY:
            return
        if evidence.relation != "NEAR_OBSTACLE":
            # A clear sample ends association with the current encounter, but
            # it cannot erase an encounter anchored at another pose/heading.
            self._active_provisional_object_id = None
            return

        record = None
        if self._active_provisional_object_id is not None:
            record = self._provisional_objects.get(
                self._active_provisional_object_id
            )
        if record is None:
            hypothesis_id = provisional_object_hypothesis_id(
                map_id=self.map_id,
                robot_id=self.robot_id,
                controller_instance_id=self.controller_instance_id,
                frame_id=self.frame_id,
                world_model_version=snapshot.world_model_version,
                first_evidence_id=evidence.evidence_id,
            )
            if (
                len(self._provisional_objects)
                >= self.policy.max_provisional_object_hypotheses
            ):
                evicted_id, _record = (
                    self._provisional_objects.popitem(last=False)
                )
                if evicted_id == self._active_provisional_object_id:
                    self._active_provisional_object_id = None
            record = _ProvisionalObjectAccumulator(
                hypothesis_id=hypothesis_id,
                anchor_x_mm=snapshot.pose.x_mm,
                anchor_y_mm=snapshot.pose.y_mm,
                anchor_heading_mdeg=snapshot.pose.heading_mdeg,
                first_seen_at_ms=evidence.observed_at_ms,
                last_seen_at_ms=evidence.observed_at_ms,
                last_state_version=snapshot.state_version,
                last_world_model_version=(
                    snapshot.world_model_version
                ),
            )
            self._provisional_objects[hypothesis_id] = record
            self._active_provisional_object_id = hypothesis_id
            return

        record.last_seen_at_ms = evidence.observed_at_ms
        record.last_state_version = snapshot.state_version
        record.last_world_model_version = snapshot.world_model_version
        record.evidence_count += 1
        self._provisional_objects.move_to_end(record.hypothesis_id)


    def _ignored_update_locked(
        self,
        snapshot: NavigationSnapshot,
        reason_code: str,
    ) -> SpatialMapUpdate:
        return SpatialMapUpdate(
            applied=False,
            reason_code=reason_code,
            state_version=snapshot.state_version,
            world_model_version=snapshot.world_model_version,
            map_version=self._map_version,
            cells_touched=0,
            cells_evicted=0,
            qualitative_evidence_added=0,
        )

    def ingest(
        self,
        snapshot: NavigationSnapshot,
    ) -> SpatialMapUpdate:
        """Atomically ingest one newer navigation state version."""

        if not isinstance(snapshot, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_spatial_snapshot",
                "Spatial mapping requires NavigationSnapshot",
            )
        if (
            snapshot.robot_id != self.robot_id
            or snapshot.controller_instance_id
            != self.controller_instance_id
        ):
            raise NavigationContractError(
                "spatial_identity_mismatch",
                "Navigation snapshot belongs to another map identity",
            )
        with self._lock:
            if snapshot.state_version <= self._last_state_version:
                return self._ignored_update_locked(
                    snapshot,
                    STALE_STATE_VERSION,
                )
            if (
                snapshot.world_model_version
                < self._last_world_model_version
            ):
                return self._ignored_update_locked(
                    snapshot,
                    STALE_WORLD_MODEL_VERSION,
                )
            if (
                snapshot.captured_at_host_ms < self._updated_at_ms
                or snapshot.clearance.observed_at_ms
                < self._last_observed_at_ms
                or (
                    self._latest_robot_pose is not None
                    and snapshot.state_observed_at_ms
                    < self._latest_robot_pose.observed_at_ms
                )
            ):
                return self._ignored_update_locked(
                    snapshot,
                    STALE_TIMESTAMP,
                )

            world_generation_changed = (
                self._last_world_model_version > 0
                and snapshot.world_model_version
                > self._last_world_model_version
            )
            first_observation = self._map_version == 0
            sensor_observation_is_new = (
                first_observation
                or world_generation_changed
                or snapshot.clearance.observed_at_ms
                > self._last_observed_at_ms
            )
            if world_generation_changed:
                self._reset_evidence_locked()

            cells_touched = 0
            cells_evicted = 0
            qualitative_added = 0
            source = snapshot.clearance.source
            if (
                sensor_observation_is_new
                and source == SIMULATION_METRIC
            ):
                (
                    cells_touched,
                    cells_evicted,
                    self._sensor_rays,
                ) = (
                    self._ingest_metric_locked(snapshot)
                )
                self._evidence_sources.add(SIMULATION_METRIC)
            elif (
                sensor_observation_is_new
                and source == PHYSICAL_IR_REFLECTION
            ):
                evidence = qualitative_evidence_for(
                    map_id=self.map_id,
                    qualitative_frame_id=(
                        self.qualitative_frame_id
                    ),
                    confidence_milli=(
                        self.policy.physical_ir_confidence_milli
                    ),
                    snapshot=snapshot,
                )
                self._qualitative.append(evidence)
                self._ingest_provisional_object_locked(
                    snapshot,
                    evidence,
                )
                self._sensor_rays = (
                    qualitative_sensor_ray(evidence),
                )
                self._evidence_sources.add(
                    PHYSICAL_IR_REFLECTION
                )
                qualitative_added = 1
            elif sensor_observation_is_new:
                self._sensor_rays = ()

            self._latest_robot_pose = SpatialRobotPose(
                frame_id=self.frame_id,
                x_mm=snapshot.pose.x_mm,
                y_mm=snapshot.pose.y_mm,
                heading_mdeg=snapshot.pose.heading_mdeg,
                observed_at_ms=snapshot.state_observed_at_ms,
                captured_at_host_ms=snapshot.captured_at_host_ms,
                state_version=snapshot.state_version,
                world_model_version=snapshot.world_model_version,
            )
            self._retain_pose_locked(self._latest_robot_pose)

            self._map_version += 1
            self._updated_at_ms = snapshot.captured_at_host_ms
            self._last_observed_at_ms = (
                snapshot.clearance.observed_at_ms
            )
            self._last_state_version = snapshot.state_version
            self._last_world_model_version = (
                snapshot.world_model_version
            )
            return SpatialMapUpdate(
                applied=True,
                reason_code=UPDATE_APPLIED,
                state_version=snapshot.state_version,
                world_model_version=snapshot.world_model_version,
                map_version=self._map_version,
                cells_touched=cells_touched,
                cells_evicted=cells_evicted,
                qualitative_evidence_added=qualitative_added,
            )

    def _occupancy_cell_for_locked(
        self,
        record: _CellAccumulator,
    ) -> OccupancyCell:
        resolution = self.policy.resolution_mm
        return OccupancyCell(
            grid_x=record.grid_x,
            grid_y=record.grid_y,
            center_x_mm=(
                record.grid_x * resolution + resolution // 2
            ),
            center_y_mm=(
                record.grid_y * resolution + resolution // 2
            ),
            classification=self._classification_for(
                record.occupancy_milli
            ),
            occupancy_milli=record.occupancy_milli,
            first_seen_at_ms=record.first_seen_at_ms,
            last_seen_at_ms=record.last_seen_at_ms,
            last_state_version=record.last_state_version,
            last_world_model_version=(
                record.last_world_model_version
            ),
            evidence_count=record.evidence_count,
            free_evidence_count=record.free_evidence_count,
            occupied_evidence_count=(
                record.occupied_evidence_count
            ),
            provenance=tuple(sorted(record.provenance)),
            quality=METRIC_FUSED,
        )

    def _object_hypotheses_locked(self):
        occupied = tuple(
            OccupiedCellEvidence(
                grid_x=record.grid_x,
                grid_y=record.grid_y,
                occupancy_milli=record.occupancy_milli,
                first_occupied_at_ms=(
                    record.first_occupied_at_ms
                ),
                last_occupied_at_ms=(
                    record.last_occupied_at_ms
                ),
                occupied_evidence_count=(
                    record.supported_occupied_evidence_count
                ),
                provenance=tuple(sorted(record.provenance)),
                trusted_simulator_object_ids=tuple(sorted(
                    record.trusted_simulator_object_ids
                )),
            )
            for record in self._cells.values()
            if (
                self._classification_for(
                    record.occupancy_milli
                ) == CELL_OCCUPIED
                and record.first_occupied_at_ms is not None
                and record.last_occupied_at_ms is not None
            )
        )
        return connected_object_hypotheses(
            map_id=self.map_id,
            robot_id=self.robot_id,
            controller_instance_id=self.controller_instance_id,
            frame_id=self.frame_id,
            resolution_mm=self.policy.resolution_mm,
            cells=occupied,
        )

    def _provisional_object_hypotheses_locked(self):
        return tuple(
            ProvisionalObjectHypothesis(
                hypothesis_id=record.hypothesis_id,
                robot_id=self.robot_id,
                controller_instance_id=self.controller_instance_id,
                frame_id=self.frame_id,
                semantic_label=SEMANTIC_UNKNOWN,
                source=PHYSICAL_IR_REFLECTION,
                geometry_kind=QUALITATIVE_FORWARD_ENVELOPE,
                bearing="FORWARD",
                relation="NEAR_OBSTACLE",
                anchor_x_mm=record.anchor_x_mm,
                anchor_y_mm=record.anchor_y_mm,
                anchor_heading_mdeg=record.anchor_heading_mdeg,
                first_seen_at_ms=record.first_seen_at_ms,
                last_seen_at_ms=record.last_seen_at_ms,
                last_state_version=record.last_state_version,
                last_world_model_version=(
                    record.last_world_model_version
                ),
                evidence_count=record.evidence_count,
                confidence_milli=(
                    self.policy.physical_ir_confidence_milli
                ),
                provenance=tuple(sorted((
                    LOCAL_ODOMETRY_POSE,
                    PHYSICAL_IR_REFLECTION,
                ))),
                provisional=True,
                quality=PROVISIONAL_QUALITATIVE,
            )
            for record in self._provisional_objects.values()
        )


    def _map_quality_locked(self) -> str:
        has_metric = SIMULATION_METRIC in self._evidence_sources
        has_physical = (
            PHYSICAL_IR_REFLECTION in self._evidence_sources
        )
        if has_metric and has_physical:
            return MAP_METRIC_WITH_PROVISIONAL_IR
        if has_metric:
            return MAP_SIMULATION_METRIC
        if has_physical:
            return MAP_PROVISIONAL_IR
        return MAP_EMPTY

    def _bounds_locked(self) -> Optional[SpatialBounds]:
        if not self._cells:
            return None
        resolution = self.policy.resolution_mm
        coordinates = tuple(self._cells)
        return SpatialBounds(
            min_x_mm=min(item[0] for item in coordinates)
            * resolution,
            min_y_mm=min(item[1] for item in coordinates)
            * resolution,
            max_x_mm=(max(item[0] for item in coordinates) + 1)
            * resolution,
            max_y_mm=(max(item[1] for item in coordinates) + 1)
            * resolution,
        )

    def snapshot(self) -> SpatialMapSnapshot:
        """Return a deeply immutable view built under the map lock."""

        with self._lock:
            cells = tuple(
                self._occupancy_cell_for_locked(record)
                for _coordinate, record in sorted(
                    self._cells.items(),
                    key=lambda item: item[0],
                )
            )
            return SpatialMapSnapshot(
                map_id=self.map_id,
                robot_id=self.robot_id,
                controller_instance_id=(
                    self.controller_instance_id
                ),
                frame_id=self.frame_id,
                frame_kind=self.frame_kind,
                map_quality=self._map_quality_locked(),
                evidence_sources=tuple(sorted(
                    self._evidence_sources
                )),
                resolution_mm=self.policy.resolution_mm,
                capacity=self.policy.max_cells,
                map_version=self._map_version,
                created_at_ms=self._created_at_ms,
                updated_at_ms=self._updated_at_ms,
                last_observed_at_ms=self._last_observed_at_ms,
                based_on_state_version=self._last_state_version,
                based_on_world_model_version=(
                    self._last_world_model_version
                ),
                cells_evicted=self._cells_evicted,
                bounds=self._bounds_locked(),
                latest_robot_pose=self._latest_robot_pose,
                pose_history=tuple(self._pose_history),
                pose_history_evicted=self._pose_history_evicted,
                sensor_rays=self._sensor_rays,
                cells=cells,
                qualitative_evidence=tuple(self._qualitative),
                object_hypotheses=tuple(sorted(
                    (
                        self._object_hypotheses_locked()
                        + self._provisional_object_hypotheses_locked()
                    ),
                    key=lambda item: item.hypothesis_id,
                )),
            )
