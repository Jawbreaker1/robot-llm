"""Small episode-relative map view for a navigation model."""

import math
from typing import Mapping

from .physical_odometry import normalize_heading_mdeg


GRID_SCHEMA = "robot-coarse-navigation-grid/v1"
GRID_CELL_SIZE_MM = 150
# BLAST's body radius including its existing margin is about 163 mm. Round up
# to 200 mm for LEGO-scale uncertainty. Orthogonal route legs separately
# prevent the diagonal corner-cutting that previously needed more inflation.
ROUTE_CLEARANCE_MM = 200
# Planned motion is Manhattan-style on the existing coarse map. Half a cell of
# cross-axis drift is accepted so LEGO odometry is not treated as exact geometry,
# without accepting a complete 150 x 150 mm diagonal as an orthogonal leg.
MODEL_ROUTE_AXIS_TOLERANCE_MM = GRID_CELL_SIZE_MM // 2
GRID_SIZE = 11
_FORWARD_CELL_MAX = 7
_FORWARD_CELL_MIN = -3
_LEFT_CELL_MAX = 5
_LEFT_CELL_MIN = -5
_UNKNOWN = "."
_OBSERVED_CLEAR = "o"
_ROBOT_KEEP_OUT = "#"
_POSSIBLE_OBSTACLE = "?"
_GOAL = "G"
_GOAL_ON_BLOCKED = "g"
_WAYPOINT = "W"
_GOAL_AND_WAYPOINT = "X"
_WAYPOINT_ON_BLOCKED = "x"
_ROBOT_HEADINGS = ("UP", "LEFT", "DOWN", "RIGHT")
_LEGEND = (
    ".=UNKNOWN o=OBSERVED_CLEAR_RAY #=ROBOT_KEEP_OUT "
    "?=POSSIBLE_OBSTACLE G=GOAL g=GOAL_ON_BLOCKED W=WAYPOINT "
    "X=GOAL_AND_WAYPOINT "
    "x=WAYPOINT_ON_BLOCKED "
    "B=BLAST E=EV3 2=BOTH_ROBOTS"
)


def _nearest_cell(value_mm):
    return math.floor(float(value_mm) / GRID_CELL_SIZE_MM + 0.5)


def _window_cell_bounds(window_center):
    if window_center is None:
        center_forward_cell = 0
        center_left_cell = 0
    else:
        center_forward_cell = _nearest_cell(window_center[0])
        center_left_cell = _nearest_cell(window_center[1])
    return (
        center_forward_cell + _FORWARD_CELL_MIN,
        center_forward_cell + _FORWARD_CELL_MAX,
        center_left_cell + _LEFT_CELL_MIN,
        center_left_cell + _LEFT_CELL_MAX,
    )


def _grid_position(forward_mm, left_mm, bounds):
    forward_cell = _nearest_cell(forward_mm)
    left_cell = _nearest_cell(left_mm)
    forward_min, forward_max, left_min, left_max = bounds
    if not (
        forward_min <= forward_cell <= forward_max
        and left_min <= left_cell <= left_max
    ):
        return None
    return (
        forward_max - forward_cell,
        left_max - left_cell,
    )


def _robot_heading(heading_mdeg):
    quarter = math.floor(
        (normalize_heading_mdeg(heading_mdeg) + 45_000) / 90_000
    ) % 4
    return _ROBOT_HEADINGS[quarter]


def _keep_out_cell_indexes(possible_obstacles):
    cells = set()
    for point in possible_obstacles:
        forward_cell = _nearest_cell(point[0])
        left_cell = _nearest_cell(point[1])
        for forward_delta in (-1, 0, 1):
            for left_delta in (-1, 0, 1):
                cells.add((
                    forward_cell + forward_delta,
                    left_cell + left_delta,
                ))
    return cells


def build_coarse_navigation_grid(
    *, robots, goal, waypoint=None, possible_obstacles=(), clear_segments=(),
    window_center=None,
):
    """Render a small rolling view without changing episode coordinates."""

    cells = [[_UNKNOWN] * GRID_SIZE for _index in range(GRID_SIZE)]
    cropped = False
    bounds = _window_cell_bounds(window_center)
    forward_min, forward_max, left_min, left_max = bounds
    keep_out_indexes = _keep_out_cell_indexes(possible_obstacles)

    def position(point):
        nonlocal cropped
        result = _grid_position(point[0], point[1], bounds)
        if result is None:
            cropped = True
            return None
        return result

    def place(point, symbol):
        result = position(point)
        if result is None:
            return None
        row, column = result
        cells[row][column] = symbol
        return row, column

    # A measured echo says that the narrow ray before it was observed clear.
    # Sampling at half-cell intervals is enough for this deliberately coarse
    # model and avoids pretending that it is a metric path planner.
    for start, end in clear_segments:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, math.ceil(distance / (GRID_CELL_SIZE_MM / 2)))
        for index in range(steps):
            ratio = index / steps
            result = position((
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
            ))
            if result is not None:
                row, column = result
                if cells[row][column] == _UNKNOWN:
                    cells[row][column] = _OBSERVED_CLEAR

    # One 150 mm neighbour around an echo is a coarse robot-centre keep-out
    # area for BLAST's roughly 200 mm wide body.
    for point in possible_obstacles:
        result = position(point)
        if result is None:
            continue
        row, column = result
        for row_delta in (-1, 0, 1):
            for column_delta in (-1, 0, 1):
                candidate_row = row + row_delta
                candidate_column = column + column_delta
                if (
                    0 <= candidate_row < GRID_SIZE
                    and 0 <= candidate_column < GRID_SIZE
                    and cells[candidate_row][candidate_column]
                    in (_UNKNOWN, _OBSERVED_CLEAR)
                ):
                    cells[candidate_row][candidate_column] = _ROBOT_KEEP_OUT
        cells[row][column] = _POSSIBLE_OBSTACLE

    # Goal, waypoint and robot symbols are visual overlays. Preserve the
    # measured-clear underlay so those markers cannot make a clear axis appear
    # unknown to the planner summary.
    observed_clear_rows = [
        "".join(
            _OBSERVED_CLEAR if cell == _OBSERVED_CLEAR else _UNKNOWN
            for cell in row
        )
        for row in cells
    ]

    goal_position = position(goal)
    if goal_position is not None:
        row, column = goal_position
        cells[row][column] = (
            _GOAL_ON_BLOCKED
            if cells[row][column] in (_POSSIBLE_OBSTACLE, _ROBOT_KEEP_OUT)
            else _GOAL
        )
    if waypoint is not None:
        waypoint_position = position(waypoint)
        if waypoint_position is not None:
            row, column = waypoint_position
            current = cells[row][column]
            cells[row][column] = (
                _GOAL_AND_WAYPOINT if current == _GOAL
                else _WAYPOINT_ON_BLOCKED
                if current in (
                    _POSSIBLE_OBSTACLE, _ROBOT_KEEP_OUT, _GOAL_ON_BLOCKED,
                )
                else _WAYPOINT
            )
    robot_markers = []
    for robot in robots:
        position = _grid_position(
            robot["forward_mm"], robot["left_mm"], bounds,
        )
        symbol = robot["symbol"]
        if position is not None:
            row, column = position
            if cells[row][column] in ("B", "E", "2"):
                cells[row][column] = "2"
            else:
                cells[row][column] = symbol
        else:
            cropped = True
        robot_markers.append({
            "symbol": symbol,
            "robot_id": robot["robot_id"],
            "row": None if position is None else position[0],
            "column": None if position is None else position[1],
            "heading": _robot_heading(robot["heading_mdeg"]),
        })

    return {
        "schema": GRID_SCHEMA,
        "frame": "EPISODE_START",
        "cell_size_mm": GRID_CELL_SIZE_MM,
        "top_is": "START_FORWARD",
        "left_is": "START_LEFT",
        "window": {
            "x_min_mm": forward_min * GRID_CELL_SIZE_MM,
            "x_max_mm": forward_max * GRID_CELL_SIZE_MM,
            "y_min_mm": left_min * GRID_CELL_SIZE_MM,
            "y_max_mm": left_max * GRID_CELL_SIZE_MM,
        },
        "rows": ["".join(row) for row in cells],
        "observed_clear_rows": observed_clear_rows,
        "robot_center_keep_out_cells": [
            {
                "x_mm": forward_cell * GRID_CELL_SIZE_MM,
                "y_mm": left_cell * GRID_CELL_SIZE_MM,
            }
            for forward_cell, left_cell in sorted(keep_out_indexes)
            if (
                forward_min <= forward_cell <= forward_max
                and left_min <= left_cell <= left_max
            )
        ],
        "robots": robot_markers,
        "legend": _LEGEND,
        "cropped": cropped,
    }


def known_clear_axis_reach_mm(grid):
    """Summarize contiguous observed-clear cells from BLAST's grid cell."""

    rows = grid.get("rows") if isinstance(grid, Mapping) else None
    clear_rows = (
        grid.get("observed_clear_rows", rows)
        if isinstance(grid, Mapping) else None
    )
    robots = grid.get("robots") if isinstance(grid, Mapping) else None
    blast = next((
        robot for robot in robots or ()
        if isinstance(robot, Mapping) and robot.get("symbol") == "B"
    ), None)
    if (
        not isinstance(rows, list)
        or not isinstance(clear_rows, list)
        or len(clear_rows) != len(rows)
        or blast is None
        or not isinstance(blast.get("row"), int)
        or not isinstance(blast.get("column"), int)
    ):
        return None
    origin = (blast["row"], blast["column"])

    def reach(row_delta, column_delta):
        cells = 0
        row, column = origin
        while True:
            row += row_delta
            column += column_delta
            if not (
                0 <= row < len(rows)
                and 0 <= column < len(rows[row])
                and clear_rows[row][column] == _OBSERVED_CLEAR
            ):
                return cells * GRID_CELL_SIZE_MM
            cells += 1

    return {
        "episode_forward_mm": reach(-1, 0),
        "episode_left_mm": reach(0, -1),
        "episode_right_mm": reach(0, 1),
        "episode_back_mm": reach(1, 0),
    }


def route_blockage_from_echoes(
    *, start, waypoints, possible_obstacles,
    clearance_mm=ROUTE_CLEARANCE_MM,
):
    """Return the first route leg crossing a known echo clearance circle."""

    if type(clearance_mm) is not int or clearance_mm <= 0:
        raise ValueError("route clearance is invalid")

    previous = start
    for leg_index, waypoint in enumerate(waypoints, start=1):
        delta_x = waypoint[0] - previous[0]
        delta_y = waypoint[1] - previous[1]
        length_squared = delta_x * delta_x + delta_y * delta_y
        if length_squared == 0:
            previous = waypoint
            continue
        nearest_blockage = None
        for echo in possible_obstacles:
            projected_ratio = (
                (echo[0] - previous[0]) * delta_x
                + (echo[1] - previous[1]) * delta_y
            ) / length_squared
            if projected_ratio <= 0:
                # Moving away from a nearby echo is an escape, not a new
                # intersection with that echo's body-clearance area.
                continue
            closest_ratio = min(1.0, projected_ratio)
            closest_x = previous[0] + delta_x * closest_ratio
            closest_y = previous[1] + delta_y * closest_ratio
            distance_squared = (
                (echo[0] - closest_x) ** 2
                + (echo[1] - closest_y) ** 2
            )
            if distance_squared >= clearance_mm ** 2:
                continue

            perpendicular_squared = max(0.0, (
                (echo[0] - previous[0]) ** 2
                + (echo[1] - previous[1]) ** 2
                - projected_ratio ** 2 * length_squared
            ))
            entry_offset = math.sqrt(max(
                0.0,
                (clearance_mm ** 2 - perpendicular_squared)
                / length_squared,
            ))
            first_ratio = max(0.0, projected_ratio - entry_offset)
            if first_ratio > 1:
                continue
            blocking_echo_point = {
                "x_mm": round(echo[0]),
                "y_mm": round(echo[1]),
            }
            candidate = {
                "reason": "KNOWN_ECHO_CLEARANCE_INTERSECTION",
                "leg_index": leg_index,
                "basis": "CONTINUOUS_ECHO_CLEARANCE",
                "clearance_mm": clearance_mm,
                "blocking_echo_point": blocking_echo_point,
            }
            key = (
                first_ratio,
                blocking_echo_point["x_mm"],
                blocking_echo_point["y_mm"],
            )
            if nearest_blockage is None or key < nearest_blockage[0]:
                nearest_blockage = (key, candidate)
        if nearest_blockage is not None:
            return nearest_blockage[1]
        previous = waypoint
    return None


def model_route_blockage(*, start, waypoints, possible_obstacles):
    """Reject diagonal model legs, then apply the existing echo guard."""

    previous = start
    for leg_index, waypoint in enumerate(waypoints, start=1):
        delta_x = waypoint[0] - previous[0]
        delta_y = waypoint[1] - previous[1]
        if (
            abs(delta_x) > MODEL_ROUTE_AXIS_TOLERANCE_MM
            and abs(delta_y) > MODEL_ROUTE_AXIS_TOLERANCE_MM
        ):
            return {
                "reason": "NON_ORTHOGONAL_ROUTE_LEG",
                "leg_index": leg_index,
                "basis": "COARSE_EPISODE_AXES",
                "axis_tolerance_mm": MODEL_ROUTE_AXIS_TOLERANCE_MM,
                "delta_x_mm": round(delta_x),
                "delta_y_mm": round(delta_y),
            }
        previous = waypoint
    return route_blockage_from_echoes(
        start=start,
        waypoints=waypoints,
        possible_obstacles=possible_obstacles,
    )


def coarse_navigation_grid_valid(value):
    """Validate the small read-only grid before publishing it."""

    required_fields = {
        "schema", "frame", "cell_size_mm", "top_is", "left_is",
        "rows", "robots", "legend", "cropped",
    }
    optional_fields = {
        "window", "robot_center_keep_out_cells", "observed_clear_rows",
    }
    if (
        not isinstance(value, Mapping)
        or not required_fields <= set(value) <= required_fields | optional_fields
    ):
        return False
    rows = value["rows"]
    robots = value["robots"]
    keep_out_cells = value.get("robot_center_keep_out_cells", [])
    observed_clear_rows = value.get("observed_clear_rows")
    window = value.get("window", {
        "x_min_mm": _FORWARD_CELL_MIN * GRID_CELL_SIZE_MM,
        "x_max_mm": _FORWARD_CELL_MAX * GRID_CELL_SIZE_MM,
        "y_min_mm": _LEFT_CELL_MIN * GRID_CELL_SIZE_MM,
        "y_max_mm": _LEFT_CELL_MAX * GRID_CELL_SIZE_MM,
    })
    if not (
        value["schema"] == GRID_SCHEMA
        and value["frame"] == "EPISODE_START"
        and value["cell_size_mm"] == GRID_CELL_SIZE_MM
        and value["top_is"] == "START_FORWARD"
        and value["left_is"] == "START_LEFT"
        and value["legend"] == _LEGEND
        and type(value["cropped"]) is bool
        and isinstance(rows, list)
        and len(rows) == GRID_SIZE
        and all(
            isinstance(row, str)
            and len(row) == GRID_SIZE
            and set(row) <= set(".o#?GgWXxEB2")
            for row in rows
        )
        and (
            observed_clear_rows is None
            or (
                isinstance(observed_clear_rows, list)
                and len(observed_clear_rows) == GRID_SIZE
                and all(
                    isinstance(row, str)
                    and len(row) == GRID_SIZE
                    and set(row) <= set(".o")
                    for row in observed_clear_rows
                )
            )
        )
        and isinstance(robots, list)
        and 1 <= len(robots) <= 2
        and isinstance(window, Mapping)
        and set(window) == {
            "x_min_mm", "x_max_mm", "y_min_mm", "y_max_mm",
        }
        and all(type(item) is int for item in window.values())
        and window["x_max_mm"] - window["x_min_mm"]
        == (GRID_SIZE - 1) * GRID_CELL_SIZE_MM
        and window["y_max_mm"] - window["y_min_mm"]
        == (GRID_SIZE - 1) * GRID_CELL_SIZE_MM
        and all(
            item % GRID_CELL_SIZE_MM == 0 for item in window.values()
        )
        and isinstance(keep_out_cells, list)
        and len(keep_out_cells) <= GRID_SIZE * GRID_SIZE
        and all(
            isinstance(cell, Mapping)
            and set(cell) == {"x_mm", "y_mm"}
            and all(type(item) is int for item in cell.values())
            and cell["x_mm"] % GRID_CELL_SIZE_MM == 0
            and cell["y_mm"] % GRID_CELL_SIZE_MM == 0
            and window["x_min_mm"] <= cell["x_mm"] <= window["x_max_mm"]
            and window["y_min_mm"] <= cell["y_mm"] <= window["y_max_mm"]
            for cell in keep_out_cells
        )
        and len({
            (cell["x_mm"], cell["y_mm"])
            for cell in keep_out_cells
        }) == len(keep_out_cells)
    ):
        return False
    for robot in robots:
        if not isinstance(robot, Mapping) or set(robot) != {
            "symbol", "robot_id", "row", "column", "heading",
        }:
            return False
        position = (robot["row"], robot["column"])
        if not (
            robot["symbol"] in ("B", "E")
            and robot["robot_id"] in ("blast-01", "ev3rstorm-01")
            and robot["heading"] in _ROBOT_HEADINGS
            and (
                position == (None, None)
                or all(
                    type(index) is int and 0 <= index < GRID_SIZE
                    for index in position
                )
            )
        ):
            return False
    return len({robot["symbol"] for robot in robots}) == len(robots)


__all__ = (
    "GRID_CELL_SIZE_MM",
    "GRID_SCHEMA",
    "GRID_SIZE",
    "build_coarse_navigation_grid",
    "coarse_navigation_grid_valid",
    "known_clear_axis_reach_mm",
    "model_route_blockage",
    "route_blockage_from_echoes",
)
