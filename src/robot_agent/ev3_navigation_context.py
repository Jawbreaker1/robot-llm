"""Translate EV3 observations into the same waypoint-planner contract as BLAST."""

from copy import deepcopy
from .route_execution_memory import route_execution_memory, latest_action_feedback

from .lm_studio_controller_action import ControllerActionContext, FOLLOW_WAYPOINT, MAX_HISTORY_ITEMS
from .physical_navigation_contract import ADVANCE, OBSERVE, REVERSE, SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90
from .waypoint_geometry import GOAL_RADIUS_MM, goal_reached, waypoint_geometry, latest_motion_feedback, waypoint_motion_hint

def _history_view(events):
    """Visited poses/outcomes, not copies of obsolete route hypotheses."""
    fields = ("event", "action", "result", "last_motion", "pose", "pose_before",
              "route_rejection", "motion_outcome", "action_completed", "observation_acquired")
    history = [{key: deepcopy(event[key]) for key in fields if key in event}
               for event in events[-MAX_HISTORY_ITEMS:]]
    for event in history:
        for key in ("pose", "pose_before"):
            if key in event:
                pose = event[key]
                event[key] = {"x_mm": pose["x_mm"], "y_mm": pose["y_mm"],
                              "heading_deg": round(pose["heading_mdeg"] / 1_000, 1)}
    return tuple(history)


def ev3_navigation_context(*, body, descriptor, footprint, mission, goal, locale="en",
                           waypoints=(), history=(), obstacle_estimates=(), previous_decision=None):
    if body.observation is None or body.survey is None:
        raise ValueError("EV3 planning requires an initial surroundings survey")
    pose = body.pose
    target_x, target_y = mission.target_point()
    waypoint = waypoints[0] if waypoints else None
    geometry = waypoint_geometry(pose, waypoint)
    if geometry is not None:
        geometry = {"distance_mm": geometry["distance_mm"],
                    "bearing_deg": round(geometry["bearing_mdeg"] / 1_000),
                    "heading_error_deg": round(geometry["heading_error_mdeg"] / 1_000)}
    ir = body.observation["infrared"]
    complete = goal_reached(mission, pose)
    actions = (FOLLOW_WAYPOINT, ADVANCE, REVERSE, TURN_LEFT_90, TURN_RIGHT_90, OBSERVE, SCAN_FRONT_ARC)
    available = tuple(action for action in actions if action != ADVANCE or not ir["blocked"])
    pose_view = {"x_mm": pose.x_mm, "y_mm": pose.y_mm,
                 "heading_deg": round(pose.heading_mdeg / 1_000, 1)}
    samples = [{"observed_from": {"x_mm": item["pose"]["x_mm"], "y_mm": item["pose"]["y_mm"]},
                "ray_heading_deg_in_episode": round(item["pose"]["heading_mdeg"] / 1_000, 1),
                "raw_ir_proximity": item["raw_ir_proximity"],
                "filtered_ir_proximity": item["filtered_ir_proximity"]}
               for item in body.ir_observations]
    return ControllerActionContext(
        goal=goal, locale=locale, robot_id=descriptor.robot_id,
        controller_id=descriptor.controller_id,
        available_actions=() if complete else available, plan_actions=actions,
        observation={"odometry": pose_view,
                     "last_motion": latest_motion_feedback(history),
                     "last_action": latest_action_feedback(history),
                     "infrared": {"units": "unitless_proximity", "raw": ir["raw"],
                                  "ray_heading_deg_in_episode": pose_view["heading_deg"],
                                  "filtered": ir["filtered"], "distance_mm": None},
                     "touch_pressed": body.observation["touch"]["pressed"],
                     "directional_goal": {"x_mm": target_x, "y_mm": target_y,
                                          "goal_radius_mm": GOAL_RADIUS_MM},
                     "goal_vector": {"dx_mm": target_x - pose.x_mm, "dy_mm": target_y - pose.y_mm,
                                     "distance_mm": mission.distance_to_target_mm(pose)}},
        local_map_evidence={"frame": "fixed episode frame: +x starting forward, +y starting left; headings 0=+x, 90=+y, -90=-y, 180=-x",
                            "robot_pose": pose_view, "robot_footprint": footprint.planner_view(),
                            "metric_distance_available": False,
                            "route_execution_memory":route_execution_memory(history),
                            "sensor_layout": {
                                "infrared_mount": "head, facing forward",
                                "color_sensor_mount": "right hand; not a range sensor",
                                "coverage": "Head-level IR samples one direction, not the full body or protruding arms.",
                                "ray_origin_basis": "Drive pose is an approximate origin; a numeric head-to-drive mounting offset has not been measured.",
                            },
                            "ir_survey": {"frame": "historical rays in the fixed episode frame, not relative to the current robot",
                                          "movement_commands_since_scan": pose.verified_motion_count - body.survey["end_pose"]["verified_motion_count"],
                                          "complete": body.survey["complete"],
                                          "sweep_degrees": body.survey["sweep_degrees"],
                                          "reason": body.survey["reason"], "samples": samples}},
        execution={"forward_admission": {"permitted": not ir["blocked"] and not body.observation["touch"]["pressed"],
                    "reason": "touch_pressed" if body.observation["touch"]["pressed"] else ir["reason"]},
                   "advance_step_mm": body.advance_step_mm,
                   "reverse_step_mm": body.advance_step_mm,
                   "follow_waypoint_max_forward_pulses": 1,
                   "scan_front_arc": {"relative_bearings_deg": list(body.FRONT_SCAN_BEARINGS_DEG),
                                      "motion": "in_place_body_rotation",
                                      "returns_to_start_heading": True},
                   "next_waypoint_motion": waypoint_motion_hint(pose, waypoint,
                       reached_radius_mm=body.waypoint_radius_mm, footprint=footprint)},
        obstacle_estimates=tuple(deepcopy(obstacle_estimates)),
        active_waypoint=waypoint, active_waypoint_geometry=geometry,
        active_waypoint_plan=tuple(deepcopy(waypoints)),
        waypoint_reached_radius_mm=body.waypoint_radius_mm,
        history=_history_view(history),
        previous_decision=previous_decision,
        completion_allowed=complete, abort_allowed=False,
    )
