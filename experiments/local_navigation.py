"""Simulation-only local policy slot; never imported by hardware wiring.

Exactly one selected neural model replaces local LLM decisions on a retained
route. Astra remains the route owner. No confidence threshold, hand-coded detour
or forced scan. Legacy JSON models remain for comparisons. The replacement NPZ
attention model uses NumPy; its weights may be authored or trained offline.
"""

import json
import hashlib
import math
from pathlib import Path
import time

ACTIONS = ("FOLLOW_WAYPOINT", "ADVANCE", "REVERSE", "TURN_LEFT_90",
           "TURN_RIGHT_90", "SCAN_FRONT_ARC", "OBSERVE", "REPLAN")
VERSION = "local-navigation-clone-v2.1"
RAY_COUNT = 128


def _number(value, scale=1):
    return max(-4.0, min(4.0, float(value or 0) / scale))


def _category(value):
    """Stable categorical code, including previously unseen failure reasons.

    No interpretation/response is assigned to a reason here. Unlike a whitelist,
    this does not silently turn a new executor result into missing information.
    """
    if value is None:
        return [0.0] * 32
    code = int.from_bytes(hashlib.sha256(str(value).encode()).digest()[:4], "big")
    return [1.0 if code & (1 << bit) else -1.0 for bit in range(32)]


def _boolean(value):
    return [float(value is True), float(value is False)]


def _motion(motion):
    motion = motion or {}
    return [float(bool(motion)), *_category(motion.get("reason")),
            *[float(motion.get("action") == a) for a in ACTIONS[:-1]],
            *_boolean(motion.get("completed")), *_boolean(motion.get("made_progress")),
            _number(motion.get("measured_translation_mm"), 1000),
            _number(motion.get("measured_turn_deg"), 180),
            _number(motion.get("requested_turn_deg"), 180),
            _number(motion.get("sweep_coverage_deg"), 360),
            math.log1p(motion.get("consecutive_no_progress_attempts") or 0)]


def features(packet):
    """Fixed, explicit observation-only projection. No truth, scene ID or text.

    Retain up to 128 positioned rays (the production packet's current limit),
    route geometry and actual execution feedback, not an invented occupancy map.
    IR remains unitless; invalid/no-return is separate from a measured value.
    All positions/headings are relative to the current measured drive pose.
    """
    evidence = packet.get("local_map_evidence") or {}
    pose = evidence.get("robot_pose") or {}
    heading = math.radians(pose.get("heading_deg", 0))
    c, s = math.cos(heading), math.sin(heading)

    def relative(point):
        dx, dy = point.get("x_mm", 0) - pose.get("x_mm", 0), point.get("y_mm", 0) - pose.get("y_mm", 0)
        return [_number(c * dx + s * dy, 1000), _number(-s * dx + c * dy, 1000)]

    observation = packet.get("observation") or {}
    execution = packet.get("execution") or {}
    waypoint = packet.get("active_waypoint") or pose
    goal = observation.get("directional_goal") or evidence.get("directional_goal") or {}
    goal_point = {"x_mm": goal.get("x_mm", goal.get("target_x_mm", 0)),
                  "y_mm": goal.get("y_mm", goal.get("target_y_mm", 0))}
    body = evidence.get("robot_footprint") or {}
    is_ir = "ir_survey" in evidence
    result = [float(is_ir), *relative(waypoint), *relative(goal_point)]
    route = packet.get("active_waypoint_plan") or []
    result += [float(bool(packet.get("active_waypoint"))), _number(len(route), 8)]
    for point in list(route[:4]) + [None] * max(0, 4 - len(route)):
        result += [1.0, *relative(point)] if point else [0.0] * 3
    result += [_number(body.get(k), 300) for k in (
        "front_extent_mm", "rear_extent_mm", "left_extent_mm", "right_extent_mm",
        "clearance_margin_mm", "turn_sweep_radius_mm")]
    result += [_number(execution.get("advance_step_mm"), 100),
               _number(packet.get("waypoint_reached_radius_mm"), 100)]
    layout = evidence.get("sensor_layout") or {}
    result += [float(isinstance(layout.get("forward_offset_mm"), (int, float))),
               _number(layout.get("forward_offset_mm"), 300), _number(layout.get("left_offset_mm"), 300)]
    current = ((observation.get("infrared") or {}).get("filtered") if is_ir else
               (observation.get("sensors") or {}).get("distance_mm"))
    result += [float(isinstance(current, (int, float))), _number(current, 100 if is_ir else 2000),
               *_boolean(observation.get("touch_pressed")),
               *_category((observation.get("sensors") or {}).get("range_state"))]
    result += [float(a in packet.get("available_actions", ())) for a in ACTIONS[:-1]]
    result += _motion(observation.get("last_motion"))
    admission = execution.get("forward_admission") or {}
    result += [*_boolean(admission.get("permitted")), *_category(admission.get("reason"))]
    hint = execution.get("next_waypoint_motion") or {}
    projection = hint.get("straight_motion_projection") or {}
    result += [float(bool(hint)), *[float(hint.get("action") == a) for a in ACTIONS[:-1]],
               _number(hint.get("remaining_turn_deg"), 180),
               _number(projection.get("travel_mm"), 1000), _number(projection.get("waypoint_miss_mm"), 300)]

    def rejection_feedback(rejection):
        rejection = rejection or {}
        blocker = rejection.get("blocking_echo_point")
        return [float(bool(rejection)), *_category(rejection.get("reason")),
                _number(rejection.get("leg_index"), 8), _number(rejection.get("clearance_mm"), 300),
                *([1.0, *relative(blocker)] if blocker else [0.0] * 3)]

    rejected = evidence.get("latest_route_rejection") or {}
    result += rejection_feedback(rejected.get("rejection"))
    result += [math.log1p(rejected.get("repeat_count") or 0),
               *_boolean(rejected.get("pose_or_evidence_changed"))]
    rejected_route = rejected.get("rejected_waypoint_plan") or []
    result += [float(bool(rejected_route) and [(p["x_mm"],p["y_mm"]) for p in rejected_route]
                     == [(p["x_mm"],p["y_mm"]) for p in route])]
    survey = evidence.get("ir_survey") or {}
    result += [*_boolean(survey.get("complete")), _number(survey.get("sweep_degrees"), 360),
               _number(survey.get("movement_commands_since_scan"), 10), *_category(survey.get("reason"))]
    previous = packet.get("previous_decision") or {}
    result += [float(previous.get("action") == a) for a in ACTIONS[:-1]]
    # A rejected action may produce no new motion. Keep the most recent action
    # RESULT, not just an older successful last_motion or a repeated pose.
    history = (packet.get("history") or [])[-4:]
    for item in [None] * (4 - len(history)) + list(history):
        p = (item or {}).get("pose") or {}
        angle = math.radians(p.get("heading_deg", pose.get("heading_deg", 0))) - heading
        result += ([1.0, *relative(p), math.sin(angle), math.cos(angle)] if p else [0.0] * 5)
        item = item or {}
        scan = item.get("scan") or {}
        interruption = item.get("route_interruption") or {}
        result += [*[float(item.get("action") == a) for a in ACTIONS[:-1]],
                   *_category(item.get("result")), *_category(item.get("action_error")),
                   *_category(item.get("motion_outcome")), *_category(interruption.get("reason")),
                   _number(interruption.get("distance_mm"), 2000),
                   *_category(item.get("scan_state", scan.get("state"))),
                   *_category(scan.get("result")), *_boolean(scan.get("restoration_verified")),
                   *_boolean(scan.get("all_observations_settled")),
                   _number(item.get("sweep_coverage_deg", scan.get("sweep_coverage_deg")), 360),
                   math.log1p(item.get("repeat_count") or 0)]
        result += _motion(item.get("last_motion")) + rejection_feedback(item.get("route_rejection"))
    if is_ir:
        rays = (evidence.get("ir_survey") or {}).get("samples", [])
    else:
        rays = evidence.get("range_observations", [])
    rays = rays[-RAY_COUNT:]
    for ray in [None] * (RAY_COUNT - len(rays)) + list(rays):
        if ray is None:
            result += [0.0] * 8
            continue
        origin = ray.get("observed_from") or ray.get("sensor_origin") or pose
        angle = math.radians(ray.get("ray_heading_deg_in_episode", ray.get("heading_deg", 0))) - heading
        raw = ray.get("filtered_ir_proximity") if is_ir else ray.get("distance_mm")
        result += [1.0, float(isinstance(raw, (int, float))),
                   _number(raw, 100 if is_ir else 2000), *relative(origin),
                   math.sin(angle), math.cos(angle), float(ray.get("settled", True))]
    return result


def target(packet, decision):
    """Imitate actions only when the teacher did not change route geometry."""
    if not packet.get("active_waypoint") or packet.get("completion_allowed"):
        return None
    proposed = decision.get("waypoint")
    if proposed:
        route = [proposed, *(decision.get("following_waypoints") or [])]
        coords = lambda points: [(p["x_mm"], p["y_mm"]) for p in points]
        if coords(route) != coords(packet.get("active_waypoint_plan") or [packet["active_waypoint"]]):
            return "REPLAN"
    return decision["action"] if decision["action"] in ACTIONS else "REPLAN"


class LocalPolicy:
    def __init__(self, artifact):
        if artifact["version"] != VERSION or artifact["actions"] != list(ACTIONS):
            raise ValueError("Incompatible local navigation model")
        self.artifact = artifact
        self.w1, self.b1, self.w2, self.b2 = (artifact[k] for k in ("w1", "b1", "w2", "b2"))
        size = len(features({}))
        hidden = len(self.b1)
        if (len(self.w1) != size or any(len(row) != hidden for row in self.w1)
                or len(self.w2) != hidden or any(len(row) != len(ACTIONS) for row in self.w2)
                or len(self.b2) != len(ACTIONS)):
            raise ValueError("Invalid local navigation weight dimensions")
        values = [*self.b1, *self.b2, *(v for row in self.w1 + self.w2 for v in row)]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Non-finite local navigation weights")

    @classmethod
    def load(cls, path):
        if Path(path).suffix == ".npz":
            from experiments.navigation_attention_policy import AttentionPolicy
            return AttentionPolicy.load(path)
        artifact = json.loads(Path(path).read_text())
        from experiments.navigation_set_policy import VERSION as SET_VERSION, SetPolicy
        return SetPolicy(artifact) if artifact.get("version") == SET_VERSION else cls(artifact)

    def predict(self, packet):
        vector = features(packet)
        hidden = [math.tanh(b + sum(x * row[j] for x, row in zip(vector, self.w1)))
                  for j, b in enumerate(self.b1)]
        scores = [b + sum(x * row[j] for x, row in zip(hidden, self.w2))
                  for j, b in enumerate(self.b2)]
        available = set(packet["available_actions"]) | {"REPLAN"}
        allowed = [i for i, a in enumerate(ACTIONS) if a in available]
        selected = max(allowed, key=lambda i: scores[i])
        peak = max(scores[i] for i in allowed)
        norm = sum(math.exp(scores[i] - peak) for i in allowed)
        return ACTIONS[selected], math.exp(scores[selected] - peak) / norm


class LocalPlanner:
    """Mutually exclusive dispatch at the EXISTING planner slot, not a veto layer."""
    def __init__(self, astra, policy, publish=lambda event: None):
        self.astra, self.policy, self.publish = astra, policy, publish
        self._route_endpoint = None
        self._reviewed_event_count = -1

    def comment(self, packet):
        return self.astra.comment(packet)

    def decide(self, context):
        from robot_agent.lm_studio_controller_action import ControllerActionDecision, ControllerActionPlannerResult
        started = time.perf_counter()
        packet = context.to_dict()
        endpoint = context.active_waypoint
        feedback = packet.get("observation", {}).get("last_action") or {}
        event_count = ((packet.get("local_map_evidence") or {}).get("route_execution_memory") or {}).get(
            "event_count", len(packet.get("history", ())))
        # Inspection and unsuccessful execution return to the route's owner.
        # This schedules a review; it neither chooses a route nor vetoes one.
        review = event_count > self._reviewed_event_count and (
            feedback.get("completed") is False
            or feedback.get("observation_acquired") is not None
            or feedback.get("reason") == "heading_aligned")
        route_changed = self._route_endpoint is not None and endpoint != self._route_endpoint
        if context.completion_allowed or endpoint is None or route_changed or review:
            action, probability = "REPLAN", 1.0
        else:
            self._route_endpoint = endpoint
            action, probability = self.policy.predict(packet)
        if action == "REPLAN":
            result = self.astra.decide(context)
            self._route_endpoint = result.decision.waypoint or endpoint
            self._reviewed_event_count = event_count
            source = "astra"
        else:
            result = ControllerActionPlannerResult(ControllerActionDecision(
                action=action, confidence_milli=round(probability * 1000),
                assessment="Local neural action; retain the current route.",
                plan=(action,), utterance=None), round((time.perf_counter() - started) * 1000))
            source = "local"
        self.publish({"event": "policy_dispatch", "source": source, "selection": action,
                      "feedback_review": review, "feedback_event_count": event_count,
                      "probability": probability, "elapsed_ms": (time.perf_counter() - started) * 1000})
        return result


def train(train_rows, validation_rows, *, seed=17, epochs=400):
    """One 32-unit tanh MLP, supervised cross-entropy, whole-run validation.

    Pure NumPy Adam. No simulator truth labels, reward function or route teacher
    is introduced. This can compress teacher behaviour, not certify/improve it.
    """
    import numpy as np
    if not train_rows or not validation_rows:
        raise ValueError("Separate nonempty training and validation episodes required")
    rng = np.random.default_rng(seed)
    x = np.asarray([features(p) for p, d in train_rows], dtype=np.float64)
    y = np.asarray([ACTIONS.index(target(p, d)) for p, d in train_rows])
    vx = np.asarray([features(p) for p, d in validation_rows], dtype=np.float64)
    vy = np.asarray([ACTIONS.index(target(p, d)) for p, d in validation_rows])
    # Equal contribution per robot, with modest class balancing within each:
    # long straight legs must not drown out the few recovery/delegation labels.
    def robot_weights(rows):
        family = np.asarray(["ir_survey" in (p.get("local_map_evidence") or {}) for p,d in rows])
        labels = np.asarray([ACTIONS.index(target(p,d)) for p,d in rows])
        result = np.zeros(len(rows))
        for group in set(family):
            mask = family == group
            counts = np.bincount(labels[mask],minlength=len(ACTIONS))
            local = 1 / np.sqrt(counts[labels[mask]])
            result[mask] = local / local.sum() * len(rows) / len(set(family))
        return result
    weights, vweights = robot_weights(train_rows), robot_weights(validation_rows)
    w1 = rng.normal(0, 1 / math.sqrt(x.shape[1]), (x.shape[1], 32))
    b1, w2, b2 = np.zeros(32), rng.normal(0, 1 / math.sqrt(32), (32, len(ACTIONS))), np.zeros(len(ACTIONS))
    params = [w1, b1, w2, b2]
    moment, variance = [np.zeros_like(p) for p in params], [np.zeros_like(p) for p in params]
    best, best_loss, best_epoch, step = None, float("inf"), 0, 0
    for epoch in range(epochs):
        for batch in np.array_split(rng.permutation(len(x)), max(1, math.ceil(len(x) / 32))):
            h = np.tanh(x[batch] @ w1 + b1)
            logits = h @ w2 + b2
            prob = np.exp(logits - logits.max(axis=1, keepdims=True))
            prob /= prob.sum(axis=1, keepdims=True)
            prob[np.arange(len(batch)), y[batch]] -= 1
            prob *= weights[batch, None] / len(batch)
            dh = (prob @ w2.T) * (1 - h * h)
            gradients = [x[batch].T @ dh + 0.001 * w1, dh.sum(axis=0),
                         h.T @ prob + 0.001 * w2, prob.sum(axis=0)]
            step += 1
            for p, g, m, v in zip(params, gradients, moment, variance):
                m[:] = 0.9 * m + 0.1 * g
                v[:] = 0.999 * v + 0.001 * g * g
                p -= 0.003 * (m / (1 - 0.9 ** step)) / (np.sqrt(v / (1 - 0.999 ** step)) + 1e-8)
        logits = np.tanh(vx @ w1 + b1) @ w2 + b2
        shifted = logits - logits.max(axis=1, keepdims=True)
        loss = float(np.mean(vweights * (np.log(np.exp(shifted).sum(axis=1)) - shifted[np.arange(len(vy)), vy])))
        if loss < best_loss:
            best_loss, best_epoch, best = loss, epoch, [p.copy() for p in params]
    return {"version": VERSION, "actions": list(ACTIONS), "seed": seed,
            "feature_count": x.shape[1], "hidden_units": 32, "parameters": sum(p.size for p in params),
            "training_rows": len(x), "validation_rows": len(vx), "best_epoch": best_epoch,
            "validation_cross_entropy": best_loss,
            "loss_weighting": "equal robot weight; inverse-square-root action frequency within each robot",
            "training_label_counts": {a: int(sum(y == i)) for i, a in enumerate(ACTIONS)},
            **{k: p.tolist() for k, p in zip(("w1", "b1", "w2", "b2"), best)}}
