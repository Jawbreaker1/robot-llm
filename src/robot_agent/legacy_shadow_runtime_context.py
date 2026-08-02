"""Deterministic canonical identity for passive legacy-runtime projections."""

import hashlib
from typing import Mapping

from .physical_agent_state import (
    ControllerKey,
    GoalAssignment,
    NavigationBasis,
)
from .physical_navigation_contract import json_bytes, validate_observation
from .physical_navigation_experience import navigation_evidence_basis


LEGACY_SHADOW_GOAL_SOURCE = "legacy-control-shadow"


class LegacyShadowRuntimeContextError(ValueError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _digest(prefix: str, value: object) -> str:
    try:
        encoded = json_bytes(value)
    except Exception as error:
        raise LegacyShadowRuntimeContextError(
            "invalid_shadow_context",
            "Shadow identity input is not deterministic JSON",
        ) from error
    return "{}-{}".format(
        prefix,
        hashlib.sha256(encoded).hexdigest()[:24],
    )


def stable_shadow_id(prefix: str, *parts: object) -> str:
    """Build one replay-stable injected identifier from explicit facts."""

    if (
        not isinstance(prefix, str)
        or not prefix
        or prefix != prefix.strip()
        or len(prefix) > 48
        or any(ord(character) < 32 for character in prefix)
    ):
        raise LegacyShadowRuntimeContextError(
            "invalid_shadow_prefix",
            "Shadow identifier prefix is invalid",
        )
    return _digest(prefix, list(parts))


def build_legacy_shadow_goal(
    *,
    episode_id: str,
    objective: str,
    locale: str,
    activated_at_ms: int,
    goal_epoch: int = 1,
) -> GoalAssignment:
    """Project one runtime request into a canonical comparison-only goal."""

    try:
        return GoalAssignment(
            goal_id=stable_shadow_id("shadow-goal", episode_id),
            goal_epoch=goal_epoch,
            objective=objective,
            source=LEGACY_SHADOW_GOAL_SOURCE,
            locale=locale,
            activated_at_ms=activated_at_ms,
        )
    except Exception as error:
        raise LegacyShadowRuntimeContextError(
            "invalid_shadow_goal",
            "Legacy runtime request could not be projected as a goal",
        ) from error


def calibration_fingerprint(calibration: Mapping[str, object]) -> str:
    if not isinstance(calibration, Mapping):
        raise LegacyShadowRuntimeContextError(
            "invalid_shadow_calibration",
            "Shadow calibration must be a mapping",
        )
    return _digest("shadow-calibration", calibration)


def build_legacy_shadow_basis(
    *,
    robot_id: str,
    controller_id: str,
    controller_instance_id: str,
    goal_epoch: int,
    observation: Mapping[str, object],
    navigation: Mapping[str, object],
    calibration_fingerprint_value: str,
) -> NavigationBasis:
    """Bind a canonical basis to exact legacy controller and map evidence."""

    try:
        checked = validate_observation(observation)
        evidence = navigation_evidence_basis(navigation, checked)
        map_generation_id = navigation["map_generation_id"]
        map_version = navigation["map_version"]
        frame_id = navigation["frame_id"]
        if (
            isinstance(map_version, bool)
            or not isinstance(map_version, int)
            or map_version < 0
        ):
            raise ValueError("map version is invalid")
        return NavigationBasis(
            controller_key=ControllerKey(
                robot_id=robot_id,
                controller_id=controller_id,
                controller_instance_id=controller_instance_id,
            ),
            goal_epoch=goal_epoch,
            controller_state_version=checked["state_version"],
            world_generation_id=map_generation_id,
            # Canonical versions start at one; legacy map revision starts at
            # zero.  The offset preserves every monotonic transition.
            world_model_version=map_version + 1,
            navigation_basis_id=_digest("shadow-basis", evidence),
            frame_id=frame_id,
            calibration_fingerprint=calibration_fingerprint_value,
        )
    except LegacyShadowRuntimeContextError:
        raise
    except Exception as error:
        raise LegacyShadowRuntimeContextError(
            "invalid_shadow_basis",
            "Legacy evidence could not be projected as a canonical basis",
        ) from error


__all__ = (
    "LEGACY_SHADOW_GOAL_SOURCE",
    "LegacyShadowRuntimeContextError",
    "build_legacy_shadow_basis",
    "build_legacy_shadow_goal",
    "calibration_fingerprint",
    "stable_shadow_id",
)
