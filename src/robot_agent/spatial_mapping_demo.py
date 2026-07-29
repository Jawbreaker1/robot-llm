"""Run a real simulator navigation while building the read-only map."""

import argparse
import json
from pathlib import Path
from typing import Optional, Sequence

from .navigation_demo import (
    DEFAULT_CONFIG_PATH,
    _build_demo_stack,
    _run_demo_stack,
)
from .spatial_map_contract import SIMULATION_WORLD
from .spatial_map_runtime import SpatialMapRuntime
from .spatial_mapping import (
    BoundedOccupancyGrid,
    SpatialMappingPolicy,
)


def build_simulation_map_demo(
    config_path: Path = DEFAULT_CONFIG_PATH,
    with_obstacle: bool = True,
):
    """Return completed navigation evidence and a live map provider."""

    config, plant, supervisor, inbox = _build_demo_stack(
        config_path,
        with_obstacle,
    )
    grid = BoundedOccupancyGrid(
        map_id="navigation-simulation-map",
        robot_id=plant.robot_id,
        controller_instance_id=plant.controller_instance_id,
        frame_id="navigation-simulation-world",
        frame_kind=SIMULATION_WORLD,
        policy=SpatialMappingPolicy(
            resolution_mm=50,
            range_max_mm=plant.settings.range_max_mm,
            max_cells=8_192,
            max_qualitative_evidence=128,
        ),
        created_at_ms=plant.clock_ms(),
    )
    runtime = SpatialMapRuntime(
        grid,
        queue_capacity=512,
        # The simulator finishes before a human can open the dashboard.
        # Keep only its final three rays visible for one honest minute.
        ray_ttl_ms=60_000,
    )
    try:
        result = _run_demo_stack(
            config,
            plant,
            supervisor,
            inbox,
            runtime.offer_nowait,
        )
        if not runtime.flush():
            raise RuntimeError("spatial map demo did not settle")
    except Exception:
        runtime.close(drain=False)
        raise
    return result, plant, runtime


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run simulator navigation and print its accumulated spatial map."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
    )
    parser.add_argument(
        "--scenario",
        choices=("obstacle", "clear"),
        default="obstacle",
    )
    parser.add_argument("--full-map", action="store_true")
    args = parser.parse_args(argv)
    result, plant, runtime = build_simulation_map_demo(
        config_path=args.config,
        with_obstacle=args.scenario == "obstacle",
    )
    try:
        map_view = runtime.snapshot()
        payload = {
            "schema": "robot-spatial-map-demo-report/v1",
            "simulation_only": True,
            "navigation": {
                "completed": result.completed,
                "termination": result.termination,
                "actions": result.actions,
                "collision_count": plant.collision_count,
                "terminal_stop_verified": (
                    result.terminal_stop_verified
                ),
            },
            "map": (
                map_view
                if args.full_map
                else {
                    "status": map_view["status"],
                    "map_version": map_view["map_version"],
                    "map_quality": map_view["map_quality"],
                    "cell_count": len(map_view["cells"]),
                    "object_hypothesis_count": len(
                        map_view["object_hypotheses"]
                    ),
                    "runtime": map_view["runtime"],
                }
            ),
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
        return (
            0
            if (
                result.completed
                and result.terminal_stop_verified
                and plant.collision_count == 0
                and map_view["status"] == "available"
            )
            else 1
        )
    finally:
        runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
