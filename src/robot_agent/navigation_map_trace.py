"""Display geometry for model-authored routes; no route selection."""

import math

from .physical_odometry import normalize_heading_mdeg
from .spatial_map_contract import ROUTE_ACTIVE, ROUTE_SCHEMA
from .waypoint_geometry import GOAL_RADIUS_MM


def final_goal_trace(mission, pose):
    distance = mission.minimum_forward_progress_mm
    current = mission.longitudinal_progress_mm(pose)
    x, y = mission.target_point()
    return {"kind": "DIRECTIONAL_HEADING", "navigation_enforced": False,
            "origin_x_mm": mission.origin_x_mm, "origin_y_mm": mission.origin_y_mm,
            "target_x_mm": x, "target_y_mm": y, "goal_radius_mm": GOAL_RADIUS_MM,
            "distance_to_goal_mm": mission.distance_to_target_mm(pose),
            "desired_heading_mdeg": mission.reference_heading_mdeg,
            "minimum_forward_progress_mm": distance,
            "heading_tolerance_mdeg": mission.heading_tolerance_mdeg,
            "current_forward_progress_mm": current,
            "current_lateral_offset_mm": mission.lateral_offset_mm(pose),
            "remaining_forward_progress_mm": max(0, distance - current)}


def waypoint_route_trace(mission, pose, waypoints, version=1):
    if not waypoints:
        return None
    previous = (pose.x_mm, pose.y_mm)
    items = []
    heading = math.radians(mission.reference_heading_mdeg / 1_000)
    lateral = 0
    for index, waypoint in enumerate(waypoints):
        dx, dy = waypoint["x_mm"] - previous[0], waypoint["y_mm"] - previous[1]
        items.append({**waypoint, "ordinal": index, "kind": "MODEL_WAYPOINT",
                      "heading_mdeg": normalize_heading_mdeg(round(math.degrees(math.atan2(dy, dx)) * 1_000)),
                      "fact_key": None, "status": "ACTIVE" if index == 0 else "UPCOMING"})
        if not lateral:
            lateral = -(waypoint["x_mm"] - pose.x_mm) * math.sin(heading) + (waypoint["y_mm"] - pose.y_mm) * math.cos(heading)
        previous = (waypoint["x_mm"], waypoint["y_mm"])
    return {"schema": ROUTE_SCHEMA, "read_only": True, "provisional": True,
            "route_id": ("model-waypoints-" + mission.episode_id)[:128], "version": max(1, version),
            "status": ROUTE_ACTIVE, "detour_side": "LEFT_OF_GOAL" if lateral > 0 else "RIGHT_OF_GOAL",
            "active_index": 0, "waypoints": items}
