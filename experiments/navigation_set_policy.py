"""Simulation-only set network: every supplied ray and route outcome is encoded.

No trained parameters, route veto, fallback policy or action-specific rules live
here. All nonzero parameters come from the authored artifact. Geometry transforms
and exact route identity comparisons describe observations, not their response.
"""
import math
from experiments.local_navigation import ACTIONS, features, _category, _boolean

VERSION = "local-navigation-sets-v2"
GLOBAL_SIZE = 252
SET_UNITS = 16
HEAD_UNITS = 32
RAY_FIELDS = (
    "is_ir", "value_present", "raw_ir_div100", "distance_m", "settled",
    "origin_forward_m", "origin_left_m", "direction_forward", "direction_left",
    "metric_endpoint_present", "endpoint_forward_m", "endpoint_left_m",
    "waypoint_forward_m", "waypoint_left_m", "waypoint_distance_m",
    "endpoint_along_waypoint_axis_m", "endpoint_across_waypoint_axis_m",
    "body_front_m", "body_rear_m", "body_left_m", "body_right_m", "margin_m",
    "range_measured", "range_no_valid_distance", "range_invalid",
    "endpoint_body_signed_distance_m", "endpoint_advance_sweep_signed_distance_m",
    "endpoint_reverse_sweep_signed_distance_m", "endpoint_turn_circle_clearance_m",
)
MEMORY_FIELDS = (
    "is_current_route", "leg_fraction", "same_full_route", "same_active_leg_endpoint",
    "leg_start_forward_m", "leg_start_left_m", "leg_end_forward_m", "leg_end_left_m",
    "pose_before_forward_m", "pose_before_left_m", "pose_after_forward_m", "pose_after_left_m",
    "completed_true", "completed_false", "progress_true", "progress_false",
    "rejection_present", "interruption_present", "translation_m", "measured_turn_div180",
    "requested_turn_div180", "waypoint_remaining_m", "repeat_log1p", "age_fraction",
    *("action_"+a for a in ACTIONS[:-1]),
    *("primitive_"+a for a in ACTIONS[:-1]),
    *("reason_bit_"+str(i) for i in range(32)),
)


ACTION_FEEDBACK_FIELDS = ("action_completed_true", "action_completed_false",
                          "observation_acquired_true", "observation_acquired_false")
LAST_ACTION_FIELDS = ("last_action_present", *("last_action_"+a for a in ACTIONS[:-1]),
                     *("last_primitive_"+a for a in ACTIONS[:-1]),
                     *ACTION_FEEDBACK_FIELDS, *("last_reason_bit_"+str(i) for i in range(32)))


def observations(packet, *, include_action_feedback=False):
    evidence = packet.get("local_map_evidence") or {}
    pose = evidence.get("robot_pose") or {"x_mm":0,"y_mm":0,"heading_deg":0}
    heading = math.radians(pose.get("heading_deg", 0))
    c, s = math.cos(heading), math.sin(heading)

    def relative(point):
        point = point or pose
        dx, dy = point.get("x_mm", 0)-pose.get("x_mm", 0), point.get("y_mm", 0)-pose.get("y_mm", 0)
        return [(c*dx+s*dy)/1000, (-s*dx+c*dy)/1000]

    waypoint = packet.get("active_waypoint") or pose
    wf, wl = relative(waypoint)
    length = math.hypot(wf, wl)
    ac, ass = (wf/length, wl/length) if length else (1, 0)
    is_ir = "ir_survey" in evidence
    rays = (evidence.get("ir_survey") or {}).get("samples", []) if is_ir else evidence.get("range_observations", [])
    body = evidence.get("robot_footprint") or {}
    body_values = [float(body.get(k) or 0)/1000 for k in (
        "front_extent_mm", "rear_extent_mm", "left_extent_mm", "right_extent_mm", "clearance_margin_mm")]
    front, rear, left, right, margin = body_values
    advance = float((packet.get("execution") or {}).get("advance_step_mm") or 0)/1000
    reverse = float((packet.get("execution") or {}).get("reverse_step_mm") or 0)/1000

    def signed_distance(x, y, min_x, max_x):
        # Exact point-to-rectangle geometry, computed BEFORE set pooling. Taking
        # separate maxima of rectangle half-spaces would combine different hits
        # and lose the conjunction: no single hit need actually be near the body.
        qx = abs(x-(min_x+max_x)/2)-(max_x-min_x)/2
        qy = abs(y-(left-right)/2)-(left+right+2*margin)/2
        return math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0)
    encoded_rays = []
    for index, ray in enumerate(rays):
        origin = ray.get("observed_from") or ray.get("sensor_origin") or pose
        of, ol = relative(origin)
        angle = math.radians(ray.get("ray_heading_deg_in_episode", ray.get("heading_deg", 0)))-heading
        df, dl = math.cos(angle), math.sin(angle)
        raw = ray.get("filtered_ir_proximity") if is_ir else ray.get("distance_mm")
        present = isinstance(raw, (int, float)) and not isinstance(raw, bool) and math.isfinite(raw)
        state = ray.get("range_state")
        metric = bool(not is_ir and present and raw >= 0 and state in (None, "MEASURED"))
        distance = raw/1000 if metric else 0
        ef, el = (of+distance*df, ol+distance*dl) if metric else (0, 0)
        clearances = [
            signed_distance(ef, el, -rear-margin, front+margin),
            signed_distance(ef, el, -rear-margin, front+margin+advance),
            signed_distance(ef, el, -rear-margin-reverse, front+margin),
            math.hypot(ef, el)-float(body.get("turn_sweep_radius_mm") or 0)/1000,
        ] if metric else [0.0]*4
        encoded_rays.append([float(is_ir), float(present), raw/100 if is_ir and present else 0,
            distance, float(ray.get("settled", True)), of, ol, df, dl, float(metric), ef, el,
            wf, wl, length, ef*ac+el*ass, -ef*ass+el*ac, *body_values,
            float(state == "MEASURED"),
            float(state == "NO_VALID_DISTANCE"), float(state == "INVALID"), *clearances])

    route = packet.get("active_waypoint_plan") or ([] if not packet.get("active_waypoint") else [waypoint])
    coords = lambda points: [(p["x_mm"], p["y_mm"]) for p in points]
    memory = evidence.get("route_execution_memory") or {}
    # Current route legs also use the same set encoder: no four-waypoint limit.
    records = [(True, {"waypoint_plan": route, "pose_before": pose})] if route else []
    records += [(False, r) for r in memory.get("records", [])]
    encoded_memory = []
    for current, record in records:
        points = record["waypoint_plan"]
        start = record.get("pose_before") or record.get("pose_after") or pose
        for index, end in enumerate(points):
            encoded_memory.append([float(current), index/max(1, len(points)-1),
                float(coords(points) == coords(route)), float(coords([end]) == coords([waypoint])),
                *relative(start), *relative(end), *relative(record.get("pose_before")),
                *relative(record.get("pose_after")), *_boolean(record.get("completed")),
                *_boolean(record.get("made_progress")), float(bool(record.get("route_rejection"))),
                float(bool(record.get("route_interruption"))),
                float(record.get("measured_translation_mm") or 0)/1000,
                float(record.get("measured_turn_deg") or 0)/180,
                float(record.get("requested_turn_deg") or 0)/180,
                float(record.get("distance_to_waypoint_after_mm") or 0)/1000,
                math.log1p(record.get("count") or 0),
                (memory.get("event_count", 0)-record.get("last_seen", 0))/max(1, memory.get("event_count", 0)),
                *[float(record.get("action") == a) for a in ACTIONS[:-1]],
                *[float(record.get("primitive") == a) for a in ACTIONS[:-1]],
                *_category(record.get("reason"))])
            if include_action_feedback:
                encoded_memory[-1] += [*_boolean(record.get("action_completed")),
                                       *_boolean(record.get("observation_acquired"))]
            start = end
    global_values = features(packet)[:GLOBAL_SIZE]
    if include_action_feedback:
        receipt = (packet.get("observation") or {}).get("last_action") or {}
        global_values += [float(bool(receipt)),
            *[float(receipt.get("action") == a) for a in ACTIONS[:-1]],
            *[float(receipt.get("primitive") == a) for a in ACTIONS[:-1]],
            *_boolean(receipt.get("completed")), *_boolean(receipt.get("observation_acquired")),
            *_category(receipt.get("reason"))]
    return global_values, encoded_rays, encoded_memory


def _layer(vector, weights, bias):
    return [math.tanh(b+sum(x*row[j] for x, row in zip(vector, weights))) for j,b in enumerate(bias)]


def _pool(rows, weights, bias):
    if not rows:
        return [0.0]*(2*len(bias))
    encoded = [_layer(row, weights, bias) for row in rows]
    return [max(col) for col in zip(*encoded)]+[sum(col)/len(encoded) for col in zip(*encoded)]


class SetPolicy:
    def __init__(self, artifact):
        if artifact.get("version") != VERSION or artifact.get("actions") != list(ACTIONS):
            raise ValueError("Incompatible set-policy artifact")
        self.artifact = artifact
        for prefix, inputs, outputs in (
            ("ray", len(RAY_FIELDS), SET_UNITS), ("memory", len(MEMORY_FIELDS), SET_UNITS),
            ("head", GLOBAL_SIZE+4*SET_UNITS+2, HEAD_UNITS), ("output", HEAD_UNITS, len(ACTIONS))):
            w, b = artifact[prefix+"_weights"], artifact[prefix+"_bias"]
            if len(w) != inputs or any(len(row) != outputs for row in w) or len(b) != outputs:
                raise ValueError("Invalid set-policy dimensions: "+prefix)
            if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
                       for v in [*b, *(v for row in w for v in row)]):
                raise ValueError("Non-finite/non-numeric set-policy weights")

    def scores(self, packet):
        global_values, rays, memory = observations(packet)
        a = self.artifact
        vector = [*global_values,
            *_pool(rays, a["ray_weights"], a["ray_bias"]),
            *_pool(memory, a["memory_weights"], a["memory_bias"]),
            math.log1p(len(rays)), math.log1p(len(memory))]
        hidden = _layer(vector, a["head_weights"], a["head_bias"])
        return [b+sum(x*row[j] for x,row in zip(hidden,a["output_weights"]))
                for j,b in enumerate(a["output_bias"])]

    def predict(self, packet):
        scores = self.scores(packet)
        available = set(packet["available_actions"]) | {"REPLAN"}
        allowed = [i for i,a in enumerate(ACTIONS) if a in available]
        selected = max(allowed, key=lambda i:scores[i])
        norm = sum(math.exp(scores[i]-scores[selected]) for i in allowed)
        return ACTIONS[selected], 1/norm
