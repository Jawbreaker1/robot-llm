"""Model-owned obstacle beliefs, not sensor returns or collision geometry."""

from copy import deepcopy

EDGES = ("min_x_mm", "max_x_mm", "min_y_mm", "max_y_mm")
MAX_OBSTACLE_ESTIMATES = 8

OBSTACLE_ESTIMATES_PROMPT = (
    "\nReport obstacle_estimates: the obstacle beliefs behind your route, in the fixed "
    "episode frame. Return the updated list, including retained beliefs. An empty list "
    "means no retained hypotheses, NOT that the surroundings have been verified empty. "
    "Include every obstacle influencing this decision even when all its edges are null. "
    "Each item has a label, bounds_mm, confidence_milli and a short evidence basis. "
    "Each of min_x_mm, max_x_mm, min_y_mm, max_y_mm may be null independently. "
    "Use null for edges not established by observations; do not invent a rectangle "
    "to fill the map or fit your route. Numeric edges are estimates, not measured "
    "surfaces or proof of clearance. Explain the supporting observations and missing "
    "information in basis. IR strength alone does not locate an edge in millimetres. "
    "Reaching a side waypoint does not establish that the body's width has cleared "
    "an unknown obstacle edge. Actual observations and motion outcomes outrank "
    "these hypotheses, but a blocked motion alone does not locate an object's "
    "edges or identify which object stopped you.\n"
)


def obstacle_estimates_schema():
    bounds = {edge: {"type": ["integer", "null"], "minimum": -100_000,
                     "maximum": 100_000} for edge in EDGES}
    fields = {
        "label": {"type": "string", "minLength": 1, "maxLength": 80},
        "bounds_mm": {"type": "object", "properties": bounds,
                      "required": list(bounds), "additionalProperties": False},
        "confidence_milli": {"type": "integer", "minimum": 0, "maximum": 1000},
        "basis": {"type": "string", "minLength": 1, "maxLength": 300},
    }
    return {
        "type": "array", "maxItems": MAX_OBSTACLE_ESTIMATES,
        "items": {"type": "object", "properties": fields,
                  "required": list(fields), "additionalProperties": False},
    }


def decode_obstacle_estimates(value):
    if not isinstance(value, list) or len(value) > MAX_OBSTACLE_ESTIMATES:
        raise ValueError("invalid obstacle estimates")
    for item in value:
        if not isinstance(item, dict) or set(item) != {"label", "bounds_mm", "confidence_milli", "basis"}:
            raise ValueError("invalid obstacle estimate fields")
        for field, maximum in (("label", 80), ("basis", 300)):
            if (not isinstance(item[field], str) or not item[field].strip()
                    or len(item[field]) > maximum or any(ord(c) < 32 for c in item[field])):
                raise ValueError("invalid obstacle estimate text")
        if type(item["confidence_milli"]) is not int or not 0 <= item["confidence_milli"] <= 1000:
            raise ValueError("invalid obstacle estimate confidence")
        bounds = item["bounds_mm"]
        if not isinstance(bounds, dict) or set(bounds) != set(EDGES):
            raise ValueError("invalid obstacle estimate bounds")
        for edge in bounds.values():
            if edge is not None and (type(edge) is not int or not -100_000 <= edge <= 100_000):
                raise ValueError("invalid obstacle estimate edge")
        for axis in ("x", "y"):
            low, high = bounds[f"min_{axis}_mm"], bounds[f"max_{axis}_mm"]
            if low is not None and high is not None and low >= high:
                raise ValueError("reversed obstacle estimate edges")
    return tuple(deepcopy(value))
