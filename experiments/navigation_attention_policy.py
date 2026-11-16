"""Simulation-only replacement policy: unpooled ray and route-memory tokens.

Only numeric weights influence action scores, whether authored or trained offline.
No route rules, confidence fallback, or hidden world geometry. Map tokens encode
the same observed evidence supplied to Astra, never simulator ground truth.
"""
import json
import math
from pathlib import Path

import numpy as np

from experiments.local_navigation import ACTIONS
from experiments.navigation_set_policy import (
    observations as set_observations, GLOBAL_SIZE as LEGACY_GLOBAL_SIZE, RAY_FIELDS,
    MEMORY_FIELDS as LEGACY_MEMORY_FIELDS, ACTION_FEEDBACK_FIELDS, LAST_ACTION_FIELDS,
)

VERSION = "local-navigation-attention-v3"
EXECUTION_FIELDS = ("advance_m", "reverse_m", "follow_max_pulses", "scan_min_div180",
    "scan_max_div180", "scan_count_div100", "scan_body_rotation", "scan_returns_heading",
    "grid_present", "clearance_present", "route_clearance_m")
MAP_FIELDS = ("cell", "echo", "keepout", "visited", "estimate",
    "unknown", "clear", "blocked", "uncertain", "goal", "waypoint", "robot",
    "metric_bounds_present", *(f"corner_{i}_{axis}_m" for i in range(4) for axis in ("forward", "left")),
    "confidence_present", "confidence", "evidence_log1p", "provisional", "body_expanded")
GLOBAL_SIZE = LEGACY_GLOBAL_SIZE + len(LAST_ACTION_FIELDS) + len(EXECUTION_FIELDS)
MEMORY_FIELDS = LEGACY_MEMORY_FIELDS + ACTION_FEEDBACK_FIELDS
WIDTH, HEADS, BLOCKS, FF_WIDTH = 128, 4, 3, 384


def observations(packet):
    global_values, rays, memory = set_observations(packet, include_action_feedback=True)
    evidence = packet.get("local_map_evidence") or {}
    execution = packet.get("execution") or {}
    scan = execution.get("scan_front_arc") or {}
    bearings = scan.get("relative_bearings_deg", scan.get("nominal_relative_bearings_deg", []))
    grid = evidence.get("coarse_grid") or {}
    clearance = evidence.get("route_clearance_mm")
    global_values += [float(execution.get("advance_step_mm") or 0)/1000,
        float(execution.get("reverse_step_mm") or 0)/1000,
        float(execution.get("follow_waypoint_max_forward_pulses") or 0),
        min(bearings, default=0)/180, max(bearings, default=0)/180, len(bearings)/100,
        float(scan.get("motion") == "in_place_body_rotation"), float(scan.get("returns_to_start_heading") is True),
        float(bool(grid)), float(clearance is not None), float(clearance or 0)/1000]
    pose = evidence.get("robot_pose") or {}
    angle = math.radians(pose.get("heading_deg", 0))
    c, s = math.cos(angle), math.sin(angle)
    tokens = []

    def token(kind, bounds, *, state="", confidence=None, count=0, provisional=False, expanded=False):
        known = all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in bounds)
        corners = []
        if known:
            x0, y0, x1, y1 = bounds
            for x, y in ((x0,y0),(x0,y1),(x1,y1),(x1,y0)):
                dx, dy = x-pose.get("x_mm",0), y-pose.get("y_mm",0)
                corners.extend(((c*dx+s*dy)/1000,(-s*dx+c*dy)/1000))
        else:
            corners = [0.0]*8
        tokens.append([float(kind == k) for k in ("cell","echo","keepout","visited","estimate")] +
            [float(bool(state) and state in chars) for chars in (".","o","#gx","?","GgX","WXx","BE2")] +
            [float(known),*corners,float(confidence is not None),float(confidence or 0),
             math.log1p(count),float(provisional),float(expanded)])

    # Grid coordinates are cell centres; # is already a ROBOT_KEEP_OUT cell.
    half = float(grid.get("cell_size_mm") or 0)/2
    for row in grid.get("rows", []):
        for y, state in zip(grid.get("column_y_mm", []), row["cells"]):
            x = row["x_mm"]
            token("cell", (x-half,y-half,x+half,y+half), state=state, expanded=state in "#gx")
    for cluster in evidence.get("current_echo_clusters", []):
        for name, kind in (("echo_bounds_mm","echo"),("robot_center_keep_out_bounds_mm","keepout")):
            b = cluster.get(name)
            if b:
                token(kind, tuple(b.get(k) for k in ("x_min_mm","y_min_mm","x_max_mm","y_max_mm")),
                      count=cluster.get("evidence_count",0), provisional=cluster.get("provisional",False), expanded=kind=="keepout")
    for point in evidence.get("visited_cells", []):
        token("visited", (point["x_mm"],point["y_mm"],point["x_mm"],point["y_mm"]))
    for estimate in packet.get("obstacle_estimates", []):
        b = estimate.get("bounds_mm") or {}
        confidence = estimate.get("confidence_milli")
        token("estimate", tuple(b.get(k) for k in ("min_x_mm","min_y_mm","max_x_mm","max_y_mm")),
              confidence=confidence/1000 if confidence is not None else None, provisional=True)
    return global_values, rays, memory, tokens


def shapes():
    result = {}
    for name, size in (("global", GLOBAL_SIZE), ("ray", len(RAY_FIELDS)), ("memory", len(MEMORY_FIELDS)), ("map", len(MAP_FIELDS))):
        result[name + "_w"] = (size, WIDTH)
        result[name + "_b"] = (WIDTH,)
    result["type_embedding"] = (4, WIDTH)
    for i in range(BLOCKS):
        for name in ("q", "k", "v", "o"):
            result[f"b{i}_{name}_w"] = (WIDTH, WIDTH)
            result[f"b{i}_{name}_b"] = (WIDTH,)
        result[f"b{i}_ff1_w"] = (WIDTH, FF_WIDTH)
        result[f"b{i}_ff1_b"] = (FF_WIDTH,)
        result[f"b{i}_ff2_w"] = (FF_WIDTH, WIDTH)
        result[f"b{i}_ff2_b"] = (WIDTH,)
    result["out_w"], result["out_b"] = (WIDTH, len(ACTIONS)), (len(ACTIONS),)
    return result


def empty_weights():
    """Zeros only; the author supplies every nonzero coefficient."""
    return {key: np.zeros(shape, dtype=np.float32) for key, shape in shapes().items()}


class AttentionPolicy:
    def __init__(self, weights):
        expected = shapes()
        if set(weights) != set(expected):
            raise ValueError("Unexpected attention tensor names")
        self.weights = {}
        for key, shape in expected.items():
            value = np.asarray(weights[key], dtype=np.float32)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError("Invalid attention tensor: " + key)
            self.weights[key] = value

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(str(archive["metadata"]))
            if metadata.get("version") != VERSION or metadata.get("actions") != list(ACTIONS):
                raise ValueError("Incompatible attention model")
            return cls({k: archive[k] for k in archive.files if k != "metadata"})

    def scores(self, packet):
        global_values, rays, memory, map_tokens = observations(packet)
        w = self.weights
        rows = []
        for kind, name, values, size in ((0, "global", [global_values], GLOBAL_SIZE),
                (1, "ray", rays, len(RAY_FIELDS)), (2, "memory", memory, len(MEMORY_FIELDS)),
                (3, "map", map_tokens, len(MAP_FIELDS))):
            if values:
                x = np.asarray(values, dtype=np.float32).reshape(-1, size)
                rows.append(x @ w[name + "_w"] + w[name + "_b"] + w["type_embedding"][kind])
        x = np.concatenate(rows, axis=0)
        n, d = len(x), WIDTH // HEADS
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            for i in range(BLOCKS):
                prefix = f"b{i}_"
                q, k, v = [(x @ w[prefix + name + "_w"] + w[prefix + name + "_b"])
                           .reshape(n, HEADS, d).transpose(1, 0, 2) for name in ("q", "k", "v")]
                attention = (q @ k.transpose(0, 2, 1)) / math.sqrt(d)
                attention -= attention.max(axis=-1, keepdims=True)
                attention = np.exp(attention)
                attention /= attention.sum(axis=-1, keepdims=True)
                mixed = (attention @ v).transpose(1, 0, 2).reshape(n, WIDTH)
                x = x + mixed @ w[prefix + "o_w"] + w[prefix + "o_b"]
                hidden = np.maximum(x @ w[prefix + "ff1_w"] + w[prefix + "ff1_b"], 0)
                x = x + hidden @ w[prefix + "ff2_w"] + w[prefix + "ff2_b"]
            result = x[0] @ w["out_w"] + w["out_b"]
        if not np.isfinite(result).all():
            raise ValueError("Nonfinite attention scores")
        return result.tolist()

    def predict(self, packet):
        scores = self.scores(packet)
        available = set(packet["available_actions"]) | {"REPLAN"}
        allowed = [i for i, action in enumerate(ACTIONS) if action in available]
        selected = max(allowed, key=lambda i: scores[i])
        return ACTIONS[selected], 1 / sum(math.exp(scores[i] - scores[selected]) for i in allowed)


def save_weights(path, weights, *, robot, method="astra-python-authored-no-training"):
    AttentionPolicy(weights)
    metadata = {"version": VERSION, "actions": list(ACTIONS), "robot": robot,
                "method": method, "parameters": sum(v.size for v in weights.values())}
    np.savez_compressed(Path(path), metadata=json.dumps(metadata), **weights)
