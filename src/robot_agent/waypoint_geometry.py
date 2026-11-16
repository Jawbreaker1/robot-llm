"""Shared fixed-frame waypoint arithmetic; never chooses a route."""

import math
from dataclasses import replace

from .physical_odometry import normalize_heading_mdeg
from .physical_navigation_contract import ADVANCE, SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90

# Arrival is positional, regardless of which way the body currently points.
WAYPOINT_REACHED_RADIUS_MM = 25
WAYPOINT_ALIGNMENT_TRIGGER_DEG = 12.0
GOAL_RADIUS_MM = 50


def goal_reached(mission, pose):
    """The same arrival region for planning and in-progress execution."""
    return mission.distance_to_target_mm(pose) <= GOAL_RADIUS_MM


def waypoint_motion_hint(pose, waypoint, *, footprint=None,
                         reached_radius_mm=WAYPOINT_REACHED_RADIUS_MM):
    """Describe the required motion; admission separately says if it is allowed."""
    action = waypoint_follow_motion_action(pose, waypoint,
        (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90), reached_radius_mm=reached_radius_mm)
    if action is None:
        return None
    hint = {"action": action}
    if action in (TURN_LEFT_90, TURN_RIGHT_90):
        target = waypoint_target_heading_mdeg(pose, waypoint)
        hint["target_heading_deg"] = target / 1000
        hint["remaining_turn_deg"] = round(normalize_heading_mdeg(target - pose.heading_mdeg) / 1000, 1)
    elif action == ADVANCE and footprint is not None:
        # Describe the actual forward ray, not an ideal line through waypoints.
        # This is geometry only: no obstacle, side or route is chosen here.
        heading = math.radians(pose.heading_mdeg / 1_000)
        dx, dy = math.cos(heading), math.sin(heading)
        distance = max(0, (waypoint["x_mm"] - pose.x_mm) * dx
                       + (waypoint["y_mm"] - pose.y_mm) * dy)
        end = replace(pose, x_mm=round(pose.x_mm + dx * distance),
                      y_mm=round(pose.y_mm + dy * distance))
        corners = footprint.world_corners(pose) + footprint.world_corners(end)
        hint["straight_motion_projection"] = {
            "assumption": "constant heading without feedback corrections; not the tracked route",
            "heading_deg": round(pose.heading_mdeg / 1_000, 1),
            "travel_mm": round(distance),
            "closest_center_to_waypoint_mm": {"x_mm": end.x_mm, "y_mm": end.y_mm},
            "waypoint_miss_mm": waypoint_geometry(end, waypoint)["distance_mm"],
            "swept_body_bounds_mm": {
                "min_x_mm": math.floor(min(x for x, _ in corners)),
                "max_x_mm": math.ceil(max(x for x, _ in corners)),
                "min_y_mm": math.floor(min(y for _, y in corners)),
                "max_y_mm": math.ceil(max(y for _, y in corners)),
            },
        }
    return hint


def measured_motion_feedback(action, before, after, *, completed, reason=None,
                             requested_turn_deg=None, scan_coverage_deg=None):
    """Describe measured execution, not a diagnosis or a recovery route."""
    turn = normalize_heading_mdeg(after.heading_mdeg - before.heading_mdeg)
    distance = round(math.hypot(after.x_mm - before.x_mm, after.y_mm - before.y_mm))
    scanning = action == SCAN_FRONT_ARC
    if requested_turn_deg is not None:
        # Completion refers to the requested alignment, not to consuming every
        # pulse of the nominal TURN_*_90 command. A useful shortened turn must
        # not become a failed-route memory solely because it stopped on target.
        remaining = normalize_heading_mdeg(round(requested_turn_deg * 1_000) - turn)
        completed = (completed or reason is None) and abs(remaining) < WAYPOINT_ALIGNMENT_TRIGGER_DEG * 1_000
    # A completed sweep can return to its starting pose. Net displacement alone
    # cannot distinguish that measured motion from a stalled scan.
    scan_progress = scanning and (completed or after.total_turn_mdeg > before.total_turn_mdeg
                                  or (scan_coverage_deg is not None and scan_coverage_deg > 0))
    feedback = {
        "kind": "scan" if scanning else "turn" if requested_turn_deg is not None else "translation",
        "action": action, "requested_turn_deg": requested_turn_deg,
        "measured_turn_deg": round(turn / 1_000, 1),
        "measured_translation_mm": distance,
        "made_progress": distance != 0 or turn != 0 or scan_progress,
        "completed": completed, "reason": reason,
    }
    if scanning and scan_coverage_deg is not None:
        feedback["sweep_coverage_deg"] = scan_coverage_deg
    return feedback


def latest_motion_feedback(history):
    """Keep a stalled primitive visible without asking the model to count pulses."""
    motions = [item for item in history if item.get("last_motion")]
    if not motions:
        return None
    last = motions[-1]
    feedback = dict(last["last_motion"])
    count = 0
    axes = ("x_mm", "y_mm", "heading_mdeg")
    for item in reversed(motions):
        motion = item["last_motion"]
        if (motion.get("made_progress", True) or motion["action"] != feedback["action"]
                or any(item.get("pose", {}).get(axis) != last.get("pose", {}).get(axis)
                       for axis in axes)):
            break
        count += 1
    feedback["consecutive_no_progress_attempts"] = count
    return feedback


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


def waypoint_target_heading_mdeg(pose, waypoint):
    """Track the chosen endpoint from the measured pose, including drift.

    Route validation owns orthogonal route legs. Snapping this measured bearing
    back to a cardinal axis would discard the correction needed to reach one.
    """
    return waypoint_geometry(pose, waypoint)["bearing_mdeg"]


def waypoint_reached(pose, waypoint, *, reached_radius_mm=WAYPOINT_REACHED_RADIUS_MM):
    if waypoint is None:
        return False
    return max(abs(waypoint["x_mm"] - pose.x_mm),
               abs(waypoint["y_mm"] - pose.y_mm)) <= reached_radius_mm


def consume_reached_waypoints(pose, waypoints, *, reached_radius_mm=WAYPOINT_REACHED_RADIUS_MM):
    """Consume only arrived targets; preserve the model's remaining hypothesis."""
    plan = tuple(waypoints)
    count = 0
    while count < len(plan) and waypoint_reached(pose, plan[count], reached_radius_mm=reached_radius_mm):
        count += 1
    return plan[count:], plan[:count]


def waypoint_follow_motion_action(pose, waypoint, available_actions, *, reached_radius_mm=WAYPOINT_REACHED_RADIUS_MM):
    """Execute a model-selected leg; never select a side or generate a target."""
    if waypoint is None or waypoint_reached(pose, waypoint, reached_radius_mm=reached_radius_mm):
        return None
    error = waypoint_geometry(pose, waypoint)["heading_error_mdeg"]
    action = (TURN_LEFT_90 if error > 0 else TURN_RIGHT_90) if abs(error) >= WAYPOINT_ALIGNMENT_TRIGGER_DEG * 1_000 else ADVANCE
    return action if action in available_actions else None
