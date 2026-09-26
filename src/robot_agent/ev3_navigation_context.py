"""Translate EV3 observations into the same waypoint-planner contract as BLAST."""

from copy import deepcopy

from .lm_studio_controller_action import ControllerActionContext, FOLLOW_WAYPOINT, MAX_HISTORY_ITEMS
from .physical_navigation_contract import ADVANCE, REVERSE, SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90
from .waypoint_geometry import waypoint_geometry


def ev3_navigation_context(*, body, descriptor, footprint, mission, goal, locale="en",
                           waypoints=(), history=()):
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
    complete = mission.distance_to_target_mm(pose) <= 50 and mission.heading_aligned(pose)
    actions = (FOLLOW_WAYPOINT, ADVANCE, REVERSE, TURN_LEFT_90, TURN_RIGHT_90, SCAN_FRONT_ARC)
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
                     "infrared": {"units": "unitless_proximity", "raw": ir["raw"],
                                  "ray_heading_deg_in_episode": pose_view["heading_deg"],
                                  "filtered": ir["filtered"]},
                     "touch_pressed": body.observation["touch"]["pressed"],
                     "directional_goal": {"x_mm": target_x, "y_mm": target_y,
                                          "heading_deg": mission.reference_heading_mdeg / 1_000},
                     "goal_vector": {"dx_mm": target_x - pose.x_mm, "dy_mm": target_y - pose.y_mm,
                                     "distance_mm": mission.distance_to_target_mm(pose)}},
        local_map_evidence={"frame": "fixed episode frame: +x starting forward, +y starting left; headings 0=+x, 90=+y, -90=-y, 180=-x",
                            "robot_pose": pose_view, "robot_footprint": footprint.planner_view(),
                            "metric_distance_available": False,
                            "sensor_layout": {
                                "infrared_mount": "head, facing forward",
                                "color_sensor_mount": "right hand; not a range sensor",
                                "coverage": "Head-level IR samples one direction, not the full body or protruding arms.",
                                "ray_origin_basis": "Drive pose is an approximate origin; a numeric head-to-drive mounting offset has not been measured.",
                            },
                            "ir_reference_observations": {
                                "meaning": "Past stationary measurements, not a distance conversion. Lower values mean stronger/closer reflection; surface and angle affect them. An untriggered motor gate is NOT evidence of free space.",
                                "white_box": [{"approx_sensor_distance_mm": 200, "raw": 44},
                                              {"approx_sensor_distance_mm": 300, "raw": 51}],
                                "dark_box": [{"approx_sensor_distance_mm": 200, "raw": 71}],
                            },
                            "ir_survey": {"frame": "historical rays in the fixed episode frame, not relative to the current robot",
                                          "movement_commands_since_scan": pose.verified_motion_count - body.survey["end_pose"]["verified_motion_count"],
                                          "complete": body.survey["complete"],
                                          "reason": body.survey["reason"], "samples": samples}},
        active_waypoint=waypoint, active_waypoint_geometry=geometry,
        active_waypoint_plan=tuple(deepcopy(waypoints)),
        waypoint_reached_radius_mm=body.waypoint_radius_mm,
        history=tuple(deepcopy(history[-MAX_HISTORY_ITEMS:])),
        completion_allowed=complete, abort_allowed=False,
    )
