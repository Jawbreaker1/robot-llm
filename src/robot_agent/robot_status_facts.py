"""Compact, detached facts for motion-free robot status answers."""

from copy import deepcopy
from typing import Mapping


ROBOT_STATUS_FACTS_SCHEMA = "robot-status-facts/v1"


def _select(source, names):
    if not isinstance(source, Mapping):
        return {}
    return {
        name: deepcopy(source[name])
        for name in names
        if name in source
    }


def _scan_summary(scan, *, with_rays=True):
    if not isinstance(scan, Mapping):
        return None
    summary = _select(scan, (
        "schema", "scan_id", "state", "status", "result", "reason", "bearing_frame",
        "bearing_convention", "completed_at_unix_ms", "completed_at_ms",
        "geometry_kind", "provisional", "observation_pattern", "arc_coverage",
        "boundary_coverage", "hypothesis_relation",
        "restoration_verified", "all_observations_settled", "sweep_coverage_deg",
        "left_boundary_mdeg", "right_boundary_mdeg",
    ))
    rays = scan.get("angular_rays", scan.get("rays"))
    if isinstance(rays, list):
        summary["ray_count"] = len(rays)
    if with_rays and isinstance(rays, list):
        summary["rays"] = [_select(ray, (
            "side", "relative_heading_deg", "bearing_mdeg", "distance_mm",
            "range_state", "observation_settled", "relation", "raw_ir_proximity",
        )) for ray in rays[:16]]
        for ray in summary["rays"]:
            if ray.get("range_state") == "NO_VALID_DISTANCE":
                ray["distance_mm"] = None  # a sensor sentinel is not a hit
    return summary


def _pose_in_degrees(pose):
    if not isinstance(pose, Mapping):
        return pose
    result = deepcopy(pose)
    heading = result.pop("heading_mdeg", None)
    if type(heading) in (int, float):
        result["heading_deg"] = round(heading / 1_000, 1)
    return result


def project_robot_status_facts(
    control_snapshot,
    spatial_snapshot,
    *,
    captured_at_unix_ms: int,
):
    """Project authoritative control/map snapshots into bounded model facts."""

    control = _select(
        control_snapshot,
        (
            "state",
            "enabled",
            "accepting",
            "updated_at_unix_ms",
            "last_error_code",
            "primary_error_code",
            "primary_error_message",
        ),
    )
    episode = (
        control_snapshot.get("episode", {})
        if isinstance(control_snapshot, Mapping)
        else {}
    )
    control["episode"] = _select(
        episode,
        (
            "episode_id",
            "goal",
            "locale",
            "started_at_unix_ms",
            "terminal_reason",
        ),
    )
    runtime = (
        control_snapshot.get("runtime", {})
        if isinstance(control_snapshot, Mapping)
        else {}
    )
    control["runtime"] = _select(
        runtime,
        (
            "current_action",
            "active_route",
            "obstacle",
            "plan",
            "speech_status",
            "speech_error_code",
            "message",
        ),
    )
    control["runtime"]["scan"] = _scan_summary(
        runtime.get("scan") if isinstance(runtime, Mapping) else None
    )

    if isinstance(spatial_snapshot, Mapping):
        spatial = _select(
            spatial_snapshot,
            (
                "schema",
                "status",
                "reason_code",
                "robot_id",
                "controller_instance_id",
                "frame_id",
                "map_generation_id",
                "map_version",
                "captured_at_unix_ms",
                "observed_at_unix_ms",
                "observed_age_ms",
                "age_ms",
                "localization",
                "robot_pose",
                "drive_observation",
            ),
        )
        for name, maximum in (
            ("pose_history", 8),
            ("qualitative_observations", 8),
        ):
            values = spatial_snapshot.get(name)
            spatial[name] = (
                deepcopy(values[-maximum:])
                if isinstance(values, list)
                else []
            )
        # Keep the obstacle's meaning, uncertainty and location, not its raw
        # support points or nested scan/encoder diagnostics.
        hypotheses = spatial_snapshot.get("object_hypotheses")
        spatial["object_hypotheses"] = [_select(item, (
            "id", "hypothesis_id", "label", "classification", "geometry_kind",
            "x_mm", "y_mm", "anchor_pose", "bearing", "relation",
            "confidence_milli", "provisional", "quality", "evidence_count",
            "observed_at_unix_ms", "age_ms", "scan_boundaries",
        )) for item in hypotheses[-8:]] if isinstance(hypotheses, list) else []
        spatial["robot_pose"] = _pose_in_degrees(spatial.get("robot_pose"))
        spatial["pose_history"] = [_pose_in_degrees(pose)
                                   for pose in spatial["pose_history"]]
        for obstacle in spatial["object_hypotheses"]:
            if "anchor_pose" in obstacle:
                obstacle["anchor_pose"] = _pose_in_degrees(obstacle["anchor_pose"])
        scans = spatial_snapshot.get("scan_evidence_history")
        spatial["scan_evidence_history"] = [
            _scan_summary(scan, with_rays=False) for scan in scans[-4:]
        ] if isinstance(scans, list) else []
        spatial.update({
            "available": spatial_snapshot.get("status") != "unavailable",
            "source": "physical_spatial_map",
            "heading_convention": "Positive degrees turn left (CCW); negative turn right (CW), relative to this map frame.",
        })
    else:
        spatial = {
            "available": False,
            "source": "physical_spatial_map",
            "reason": "not_available",
        }

    return {
        "schema": ROBOT_STATUS_FACTS_SCHEMA,
        "captured_at_unix_ms": captured_at_unix_ms,
        "sensor_observation_scope": "Stored observations, not a live sensor read. The robot may have moved since the last scan; captured_at_unix_ms is the summary time, not the sensor time.",
        "control": {
            "available": isinstance(control_snapshot, Mapping),
            "source": "robot_control_service",
            **control,
        },
        "spatial_map": spatial,
        "camera_vision": {
            "available": False,
            "reason": "not_configured",
        },
    }


__all__ = ("ROBOT_STATUS_FACTS_SCHEMA", "project_robot_status_facts")
