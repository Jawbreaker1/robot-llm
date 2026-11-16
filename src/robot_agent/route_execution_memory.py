"""Compact episode facts, not a route veto or recovery controller.

Derived from the complete existing execution journal, before prompt history is
trimmed. A scan/reverse never deletes an earlier outcome. Repeated equivalent
outcomes are grouped; changed routes and later successes remain distinguishable.
"""
from copy import deepcopy
import json


def _pose_view(pose):
    if pose is None:
        return None
    result = deepcopy(pose)
    if "heading_mdeg" in result:
        result["heading_deg"] = result.pop("heading_mdeg") / 1000
    return result


def latest_action_feedback(history):
    """Latest operation receipt, independent of the last wheel movement.

    completed describes the recorded operation, not the whole mission/route.
    observation_acquired means sensing ran and settled, NOT that space is free
    or a valid range was returned. Missing/legacy fields remain unknown.
    Neither field implies translation or progress toward the goal.
    """
    if not history:
        return None
    event = history[-1]
    motion = event.get("last_motion") or {}
    return {
        "action": event.get("requested_action", event.get("action")),
        "primitive": motion.get("action", event.get("action")),
        "completed": event.get("action_completed"),
        "observation_acquired": event.get("observation_acquired"),
        "reason": ((event.get("route_rejection") or {}).get("reason")
                   or (event.get("route_interruption") or {}).get("reason")
                   or event.get("action_error") or event.get("result") or motion.get("reason")),
    }


def route_execution_memory(history):
    groups = {}
    for index, event in enumerate(history):
        route = event.get("waypoint_plan") or ()
        motion = event.get("last_motion") or {}
        rejection = event.get("route_rejection") or {}
        # Admission can reject an action before a motor command exists. That is
        # still a route outcome; omitting it leaves only older successful motion
        # in the compact memory and hides repeated blocked attempts.
        if not route or not (motion or rejection or event.get("route_interruption")
                             or event.get("result") or event.get("action_error")):
            continue
        coordinates = [{"x_mm":p["x_mm"],"y_mm":p["y_mm"]} for p in route]
        outcome = {"action":event.get("requested_action", event.get("action")),
                   "primitive":motion.get("action",event.get("action")),
                   "completed":motion.get("completed"),"made_progress":motion.get("made_progress"),
                   "action_completed":event.get("action_completed"),
                   "observation_acquired":event.get("observation_acquired"),
                   "reason":rejection.get("reason") or (event.get("route_interruption") or {}).get("reason")
                       or event.get("action_error") or event.get("result") or motion.get("reason"),
                   "route_rejection":deepcopy(rejection),
                   "route_interruption":deepcopy(event.get("route_interruption"))}
        key = json.dumps([coordinates,outcome],sort_keys=True)
        previous = groups.get(key)
        groups[key] = {"waypoint_plan":coordinates, **outcome,
            "first_seen":previous["first_seen"] if previous else index,
            "last_seen":index,"count":previous["count"]+1 if previous else 1,
            "pose_before":_pose_view(event.get("pose_before")),
            "pose_after":_pose_view(event.get("pose")),
            "measured_translation_mm":motion.get("measured_translation_mm"),
            "measured_turn_deg":motion.get("measured_turn_deg"),
            "requested_turn_deg":motion.get("requested_turn_deg"),
            "distance_to_waypoint_after_mm":(event.get("active_waypoint_geometry_after") or {}).get("distance_mm")}
    return {"scope":"this episode; all recorded route outcomes, no expiry or automatic invalidation",
            "event_count":len(history),"records":list(groups.values())}
