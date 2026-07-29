"""Read-only dashboard projection for immutable spatial maps.

Source-monotonic timestamps remain ages unless the caller supplies an
explicit age-to-Unix bridge.  This adapter never relabels source time as Unix.
"""

from typing import Mapping, Optional

from .navigation_contract import integer
from .spatial_map_contract import (
    DASHBOARD_SPATIAL_MAP_SCHEMA,
    LOCAL_ODOMETRY,
    MAP_PROVISIONAL_IR,
    MAP_SIMULATION_METRIC,
    MAP_METRIC_WITH_PROVISIONAL_IR,
    SIMULATION_WORLD,
    SpatialMapSnapshot,
)


def spatial_dashboard_view(
    snapshot: SpatialMapSnapshot,
    now_unix_ms: int,
    observed_age_ms: Optional[int] = None,
    ray_ttl_ms: Optional[int] = None,
) -> Mapping[str, object]:
    """Return the read-only UI schema without inventing Unix times.

    ``observed_age_ms`` is an explicit clock bridge supplied by the host.
    When it is absent, all per-item Unix timestamps remain ``null``.
    """

    integer("now_unix_ms", now_unix_ms, 0, 2**63 - 1)
    if observed_age_ms is not None:
        integer(
            "observed_age_ms",
            observed_age_ms,
            0,
            now_unix_ms,
        )
    if ray_ttl_ms is not None:
        integer("ray_ttl_ms", ray_ttl_ms, 1, 60_000)

    def age_for(source_ms: int) -> Optional[int]:
        if observed_age_ms is None:
            return None
        return max(
            0,
            observed_age_ms
            + snapshot.last_observed_at_ms
            - source_ms,
        )

    def unix_for(source_ms: int) -> Optional[int]:
        age = age_for(source_ms)
        if age is None:
            return None
        return max(0, now_unix_ms - age)

    def provenance_for(items) -> Optional[str]:
        if not items:
            return None
        return " | ".join(items)

    dashboard_cells = []
    for cell in snapshot.cells:
        dashboard_cells.append({
            "x_mm": cell.center_x_mm,
            "y_mm": cell.center_y_mm,
            "size_mm": snapshot.resolution_mm,
            "state": cell.classification,
            "confidence_milli": abs(cell.occupancy_milli),
            "source_id": "bounded-occupancy-grid",
            "provenance": provenance_for(cell.provenance),
            "observed_at_unix_ms": unix_for(
                cell.last_seen_at_ms
            ),
            "age_ms": age_for(cell.last_seen_at_ms),
        })

    dashboard_rays = []
    for ray in snapshot.sensor_rays:
        metric = ray.source == "simulation_metric"
        observed_unix_ms = unix_for(ray.observed_at_ms)
        dashboard_rays.append({
            "direction": ray.direction,
            "origin_x_mm": ray.origin_x_mm if metric else None,
            "origin_y_mm": ray.origin_y_mm if metric else None,
            "end_x_mm": ray.end_x_mm if metric else None,
            "end_y_mm": ray.end_y_mm if metric else None,
            "state": (
                (
                    "OCCUPIED_ENDPOINT"
                    if ray.endpoint_occupied
                    else "CLEAR_TO_MAX_RANGE"
                )
                if metric
                else ray.relation
            ),
            "confidence_milli": ray.confidence_milli,
            "source_id": ray.source,
            "provenance": (
                "SIMULATION_CONFIGURATION_SPACE"
                if metric
                else "PROVISIONAL_IR"
            ),
            "provisional": ray.provisional,
            "observed_at_unix_ms": observed_unix_ms,
            "valid_until_unix_ms": (
                observed_unix_ms + ray_ttl_ms
                if (
                    observed_unix_ms is not None
                    and ray_ttl_ms is not None
                )
                else None
            ),
            "age_ms": age_for(ray.observed_at_ms),
        })

    hypotheses = []
    for item in snapshot.object_hypotheses:
        hypotheses.append({
            "hypothesis_id": item.hypothesis_id,
            "x_mm": item.centroid_x_mm,
            "y_mm": item.centroid_y_mm,
            "label": item.semantic_label,
            "bounds": {
                "min_x_mm": item.min_x_mm,
                "min_y_mm": item.min_y_mm,
                "max_x_mm": item.max_x_mm,
                "max_y_mm": item.max_y_mm,
            },
            "cell_count": item.cell_count,
            "evidence_count": item.evidence_count,
            "confidence_milli": item.confidence_milli,
            "source_id": "occupied-component",
            "provenance": provenance_for(item.provenance),
            "trusted_simulator_object_id": (
                item.trusted_simulator_object_id
            ),
            "observed_at_unix_ms": unix_for(
                item.last_seen_at_ms
            ),
            "age_ms": age_for(item.last_seen_at_ms),
        })

    qualitative_observations = []
    for item in snapshot.qualitative_evidence:
        qualitative_observations.append({
            "evidence_id": item.evidence_id,
            "bearing": item.bearing,
            "relation": item.relation,
            "raw_ir_proximity": item.raw_ir_proximity,
            "confidence_milli": item.confidence_milli,
            "source_id": item.source,
            "provenance": "PROVISIONAL_IR",
            "provisional": item.provisional,
            "observed_at_unix_ms": unix_for(
                item.observed_at_ms
            ),
            "age_ms": age_for(item.observed_at_ms),
        })

    pose = None
    if snapshot.latest_robot_pose is not None:
        pose = {
            "x_mm": snapshot.latest_robot_pose.x_mm,
            "y_mm": snapshot.latest_robot_pose.y_mm,
            "heading_mdeg": (
                snapshot.latest_robot_pose.heading_mdeg
            ),
            "frame_id": snapshot.latest_robot_pose.frame_id,
            "state_version": (
                snapshot.latest_robot_pose.state_version
            ),
            "source_id": "navigation-pose",
            "provenance": (
                "SIMULATION"
                if snapshot.frame_kind == SIMULATION_WORLD
                else (
                    "LOCAL_ODOMETRY"
                    if snapshot.frame_kind == LOCAL_ODOMETRY
                    else None
                )
            ),
            "observed_at_unix_ms": unix_for(
                snapshot.latest_robot_pose.observed_at_ms
            ),
            "age_ms": age_for(
                snapshot.latest_robot_pose.observed_at_ms
            ),
        }
    if snapshot.map_quality in (
        MAP_SIMULATION_METRIC,
        MAP_METRIC_WITH_PROVISIONAL_IR,
    ):
        status = "available"
        reason_code = None
    elif snapshot.map_quality == MAP_PROVISIONAL_IR:
        status = "qualitative_only"
        reason_code = "provisional_ir_only"
    elif pose is not None:
        status = "pose_only"
        reason_code = "pose_only"
    else:
        status = "unavailable"
        reason_code = "no_observations"
    return {
        "schema": DASHBOARD_SPATIAL_MAP_SCHEMA,
        "read_only": True,
        "status": status,
        "reason_code": reason_code,
        "map_id": snapshot.map_id,
        "robot_id": snapshot.robot_id,
        "controller_instance_id": snapshot.controller_instance_id,
        "frame_id": snapshot.frame_id,
        "frame_kind": snapshot.frame_kind,
        "map_quality": snapshot.map_quality,
        "map_version": snapshot.map_version,
        "based_on_state_version": snapshot.based_on_state_version,
        "based_on_world_model_version": (
            snapshot.based_on_world_model_version
        ),
        "resolution_mm": snapshot.resolution_mm,
        "capacity": snapshot.capacity,
        "cells_evicted": snapshot.cells_evicted,
        "source_id": "spatial-map-core",
        "provenance": (
            "SIMULATION + PROVISIONAL_IR"
            if len(snapshot.evidence_sources) == 2
            else (
                "SIMULATION"
                if "simulation_metric"
                in snapshot.evidence_sources
                else (
                    "PROVISIONAL_IR"
                    if snapshot.evidence_sources
                    else None
                )
            )
        ),
        "bounds": (
            None if snapshot.bounds is None else snapshot.bounds.to_dict()
        ),
        "robot_pose": pose,
        "sensor_rays": dashboard_rays,
        "cells": dashboard_cells,
        "qualitative_observations": qualitative_observations,
        "object_hypotheses": hypotheses,
        "observed_age_ms": observed_age_ms,
        "age_ms": observed_age_ms,
        "captured_at_unix_ms": unix_for(
            snapshot.updated_at_ms
        ),
        "observed_at_unix_ms": unix_for(
            snapshot.last_observed_at_ms
        ),
    }
