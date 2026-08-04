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
            "scan",
            "speech_status",
            "message",
        ),
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
            ("object_hypotheses", 8),
            ("scan_evidence_history", 4),
        ):
            values = spatial_snapshot.get(name)
            spatial[name] = (
                deepcopy(values[-maximum:])
                if isinstance(values, list)
                else []
            )
        spatial.update({
            "available": spatial_snapshot.get("status") != "unavailable",
            "source": "physical_spatial_map",
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
