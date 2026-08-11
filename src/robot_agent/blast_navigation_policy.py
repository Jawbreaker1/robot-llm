"""Small, source-local range policy for BLAST no-return observations."""

from __future__ import annotations

from typing import Mapping

from .blast_observation_monitor import (
    RANGE_STATE_NO_VALID_DISTANCE,
    blast_range_state,
)
from .physical_odometry import PhysicalPose


def settled_no_return_at_pose(
    distance_mm, current_pose, prior_receipt,
) -> bool:
    """Whether the current range is settled no-return at this exact pose."""

    if not isinstance(prior_receipt, Mapping) or not isinstance(
        current_pose, PhysicalPose
    ):
        return False
    observation = prior_receipt.get("result_observation")
    motion = prior_receipt.get("motion")
    return (
        blast_range_state(distance_mm) == RANGE_STATE_NO_VALID_DISTANCE
        and isinstance(observation, Mapping)
        and prior_receipt.get("observation_settled") is True
        and prior_receipt.get("pose") == current_pose.to_dict()
        and observation.get("motion_active") is False
        and blast_range_state(observation.get("distance_mm"))
        == RANGE_STATE_NO_VALID_DISTANCE
        and (
            not isinstance(motion, Mapping)
            or motion.get("command_completed") is True
        )
    )


__all__ = ("settled_no_return_at_pose",)
