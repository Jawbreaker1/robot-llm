"""Deterministic action feasibility published before model planning."""

from typing import Mapping

from .active_ir_scan_contract import ActiveIrScanCalibration
from .physical_navigation_contract import MOTION_ACTIONS
from .physical_odometry import OdometryCalibration, PhysicalPose
from .provisional_hazard_map import ProvisionalHazardMap


def navigation_action_feasibility(
    *,
    hazard_map: ProvisionalHazardMap,
    pose: PhysicalPose,
    action_specs: Mapping[str, Mapping[str, object]],
    odometry_calibration: OdometryCalibration,
    active_scan_calibration: ActiveIrScanCalibration,
) -> Mapping[str, object]:
    """Describe geometry feasibility without ranking or selecting an action."""

    if not isinstance(hazard_map, ProvisionalHazardMap):
        raise ValueError("action feasibility hazard map is invalid")
    if not isinstance(pose, PhysicalPose):
        raise ValueError("action feasibility pose is invalid")
    if not isinstance(action_specs, Mapping):
        raise ValueError("action feasibility specs are invalid")
    if not isinstance(odometry_calibration, OdometryCalibration):
        raise ValueError("action feasibility odometry is invalid")
    if not isinstance(active_scan_calibration, ActiveIrScanCalibration):
        raise ValueError("action feasibility scan calibration is invalid")

    motion = {}
    for action in sorted(MOTION_ACTIONS):
        result = hazard_map.validate_swept_path(
            pose,
            action,
            action_specs,
            odometry_calibration,
        )
        motion[action] = {
            "allowed": result["allowed"],
            "reason": result["reason"],
            "hazard_ids": list(result["hazard_ids"]),
            "monotonic_escape_hazard_ids": list(
                result.get("monotonic_escape_hazard_ids", ())
            ),
            "maximum_endpoint": result.get("maximum_endpoint"),
            "host_selected_alternative_action": False,
        }

    scan = hazard_map.validate_in_place_rotation(
        pose,
        active_scan_calibration.coarse_offsets_mdeg,
        alignment_tolerance_mdeg=(
            active_scan_calibration.alignment_tolerance_mdeg
        ),
    )
    return {
        "collision_geometry": hazard_map.calibration.collision_geometry(),
        "motion_actions": motion,
        "active_scan": {
            "allowed": scan["allowed"],
            "reason": scan["reason"],
            "hazard_ids": list(scan["hazard_ids"]),
            "minimum_relative_heading_mdeg": scan[
                "minimum_relative_heading_mdeg"
            ],
            "maximum_relative_heading_mdeg": scan[
                "maximum_relative_heading_mdeg"
            ],
            "host_selected_alternative_action": False,
        },
        "host_ranked_or_selected_action": False,
    }


__all__ = ("navigation_action_feasibility",)
