"""Shared fixed-frame waypoint arithmetic; never chooses a route."""

import math

from .coarse_navigation_grid import MODEL_ROUTE_AXIS_TOLERANCE_MM
from .physical_odometry import normalize_heading_mdeg
from .physical_navigation_contract import ADVANCE, TURN_LEFT_90, TURN_RIGHT_90

# Along-track arrival and cross-track LEGO drift have different tolerances;
# a large circular radius would cut corner legs short.
WAYPOINT_REACHED_RADIUS_MM = 25
WAYPOINT_ALIGNMENT_TRIGGER_DEG = 12.0
GOAL_RADIUS_MM = 50


def waypoint_geometry(pose, waypoint):
    if waypoint is None:
        return None
    dx = waypoint["x_mm"] - pose.x_mm
    dy = waypoint["y_mm"] - pose.y_mm
    bearing = normalize_heading_mdeg(round(math.degrees(math.atan2(dy, dx)) * 1_000))
    return {
        "distance_mm": round(math.hypot(dx, dy)),
        "bearing_mdeg": bearing,
        "heading_error_mdeg": normalize_heading_mdeg(bearing - pose.heading_mdeg),
    }


def waypoint_axis_heading_mdeg(pose, waypoint):
    """Translate an accepted coarse leg into one cardinal heading."""
    dx = waypoint["x_mm"] - pose.x_mm
    dy = waypoint["y_mm"] - pose.y_mm
    if abs(dx) > MODEL_ROUTE_AXIS_TOLERANCE_MM and abs(dy) > MODEL_ROUTE_AXIS_TOLERANCE_MM:
        return waypoint_geometry(pose, waypoint)["bearing_mdeg"]
    if abs(dx) >= abs(dy):
        return 0 if dx >= 0 else -180_000
    return 90_000 if dy >= 0 else -90_000


def waypoint_reached(pose, waypoint, *, mission=None, reached_radius_mm=WAYPOINT_REACHED_RADIUS_MM):
    if waypoint is None:
        return False
    if mission is not None and (waypoint["x_mm"], waypoint["y_mm"]) == mission.target_point():
        return mission.distance_to_target_mm(pose) <= GOAL_RADIUS_MM
    dx = abs(waypoint["x_mm"] - pose.x_mm)
    dy = abs(waypoint["y_mm"] - pose.y_mm)
    heading = math.radians(pose.heading_mdeg / 1_000)
    along, cross = (dx, dy) if abs(math.cos(heading)) >= abs(math.sin(heading)) else (dy, dx)
    return along <= reached_radius_mm and cross <= MODEL_ROUTE_AXIS_TOLERANCE_MM


def consume_reached_waypoints(pose, waypoints, *, mission=None, reached_radius_mm=WAYPOINT_REACHED_RADIUS_MM):
    """Consume only arrived targets; preserve the model's remaining hypothesis."""
    plan = tuple(waypoints)
    count = 0
    while count < len(plan) and waypoint_reached(pose, plan[count], mission=mission, reached_radius_mm=reached_radius_mm):
        count += 1
    return plan[count:], plan[:count]


def waypoint_follow_motion_action(pose, waypoint, available_actions, *, mission=None, reached_radius_mm=WAYPOINT_REACHED_RADIUS_MM):
    """Execute a model-selected leg; never select a side or generate a target."""
    if waypoint is None or waypoint_reached(pose, waypoint, mission=mission, reached_radius_mm=reached_radius_mm):
        return None
    error = normalize_heading_mdeg(waypoint_axis_heading_mdeg(pose, waypoint) - pose.heading_mdeg)
    action = (TURN_LEFT_90 if error > 0 else TURN_RIGHT_90) if abs(error) >= WAYPOINT_ALIGNMENT_TRIGGER_DEG * 1_000 else ADVANCE
    return action if action in available_actions else None
