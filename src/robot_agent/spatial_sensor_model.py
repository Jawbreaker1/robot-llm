"""Pure sensor projection helpers for uncertain spatial mapping."""

from dataclasses import dataclass
import hashlib
import math
from typing import Optional, Tuple

from .navigation_state import NavigationSnapshot
from .spatial_map_contract import (
    PHYSICAL_IR_REFLECTION,
    PROVISIONAL_QUALITATIVE,
    QualitativeObstacleEvidence,
    SpatialSensorRay,
)


SIMULATION_METRIC = "simulation_metric"
SIMULATION_CONFIGURATION_SPACE = "SIMULATION_CONFIGURATION_SPACE"
ROBOT_BASE_FRAME = "ROBOT_BASE"


@dataclass(frozen=True)
class MetricRayGeometry:
    """Grid traversal and metric endpoint for one trusted simulator ray."""

    cells: Tuple[Tuple[int, int], ...]
    endpoint_occupied: bool
    endpoint_x_mm: int
    endpoint_y_mm: int


def _cell_for(
    x_mm: int,
    y_mm: int,
    resolution_mm: int,
) -> Tuple[int, int]:
    return (
        math.floor(x_mm / resolution_mm),
        math.floor(y_mm / resolution_mm),
    )


def _bresenham_cells(
    start: Tuple[int, int],
    end: Tuple[int, int],
) -> Tuple[Tuple[int, int], ...]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    cells = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x0 += sx
        if doubled <= dx:
            error += dx
            y0 += sy
    return tuple(cells)


def metric_ray_geometry(
    snapshot: NavigationSnapshot,
    angle_offset_degrees: int,
    measured_mm: int,
    resolution_mm: int,
    range_max_mm: int,
) -> MetricRayGeometry:
    """Project a trusted range into map cells without mutating a map."""

    distance_mm = min(measured_mm, range_max_mm)
    angle = math.radians(
        snapshot.pose.heading_mdeg / 1_000.0
        + angle_offset_degrees
    )
    endpoint_x_mm = int(round(
        snapshot.pose.x_mm + math.cos(angle) * distance_mm
    ))
    endpoint_y_mm = int(round(
        snapshot.pose.y_mm + math.sin(angle) * distance_mm
    ))
    start = _cell_for(
        snapshot.pose.x_mm,
        snapshot.pose.y_mm,
        resolution_mm,
    )
    end = _cell_for(
        endpoint_x_mm,
        endpoint_y_mm,
        resolution_mm,
    )
    return MetricRayGeometry(
        cells=_bresenham_cells(start, end),
        endpoint_occupied=measured_mm < range_max_mm,
        endpoint_x_mm=endpoint_x_mm,
        endpoint_y_mm=endpoint_y_mm,
    )


def metric_sensor_ray(
    snapshot: NavigationSnapshot,
    direction: str,
    frame_id: str,
    measured_mm: int,
    range_max_mm: int,
    geometry: MetricRayGeometry,
    trusted_simulator_object_id: Optional[str],
) -> SpatialSensorRay:
    """Build a typed metric ray from already projected geometry."""

    return SpatialSensorRay(
        direction=direction,
        frame_id=frame_id,
        source=SIMULATION_METRIC,
        observed_at_ms=snapshot.clearance.observed_at_ms,
        captured_at_host_ms=snapshot.captured_at_host_ms,
        state_version=snapshot.state_version,
        world_model_version=snapshot.world_model_version,
        confidence_milli=1_000,
        provisional=False,
        origin_x_mm=snapshot.pose.x_mm,
        origin_y_mm=snapshot.pose.y_mm,
        end_x_mm=geometry.endpoint_x_mm,
        end_y_mm=geometry.endpoint_y_mm,
        measured_range_mm=measured_mm,
        max_range_mm=range_max_mm,
        endpoint_occupied=geometry.endpoint_occupied,
        trusted_simulator_object_id=(
            trusted_simulator_object_id
            if geometry.endpoint_occupied
            else None
        ),
    )


def qualitative_evidence_for(
    map_id: str,
    qualitative_frame_id: str,
    confidence_milli: int,
    snapshot: NavigationSnapshot,
) -> QualitativeObstacleEvidence:
    """Represent physical IR as provisional evidence without geometry."""

    identity = "{}\0{}\0{}\0{}".format(
        map_id,
        snapshot.robot_id,
        snapshot.controller_instance_id,
        snapshot.state_version,
    ).encode("utf-8")
    evidence_id = "qualitative-{}".format(
        hashlib.sha256(identity).hexdigest()[:20]
    )
    return QualitativeObstacleEvidence(
        evidence_id=evidence_id,
        robot_id=snapshot.robot_id,
        controller_instance_id=snapshot.controller_instance_id,
        frame_id=qualitative_frame_id,
        source=PHYSICAL_IR_REFLECTION,
        bearing="FORWARD",
        relation=(
            "NEAR_OBSTACLE"
            if snapshot.clearance.near_obstacle_latched
            else "NO_NEAR_REFLECTION"
        ),
        observed_at_ms=snapshot.clearance.observed_at_ms,
        captured_at_host_ms=snapshot.captured_at_host_ms,
        state_version=snapshot.state_version,
        world_model_version=snapshot.world_model_version,
        confidence_milli=confidence_milli,
        raw_ir_proximity=snapshot.clearance.raw_ir_proximity,
        provisional=True,
        quality=PROVISIONAL_QUALITATIVE,
    )


def qualitative_sensor_ray(
    evidence: QualitativeObstacleEvidence,
) -> SpatialSensorRay:
    """Create the non-metric ray view of physical IR evidence."""

    return SpatialSensorRay(
        direction="FORWARD",
        frame_id=evidence.frame_id,
        source=evidence.source,
        observed_at_ms=evidence.observed_at_ms,
        captured_at_host_ms=evidence.captured_at_host_ms,
        state_version=evidence.state_version,
        world_model_version=evidence.world_model_version,
        confidence_milli=evidence.confidence_milli,
        provisional=True,
        relation=evidence.relation,
        raw_ir_proximity=evidence.raw_ir_proximity,
    )
