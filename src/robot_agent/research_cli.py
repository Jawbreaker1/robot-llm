"""One-shot, motion-free weather research through local LM Studio.

The command only composes the read-only research planner and the fixed-origin
Open-Meteo tool.  It imports no robot execution module and exposes no physical
capability.
"""

import argparse
import json
import secrets
import sys
import time
from typing import List, Optional

from .lm_studio import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    LMStudioError,
)
from .lm_studio_research import LMStudioResearchPlanner
from .research import OpenMeteoWeatherTool, ResearchError, WeatherTool
from .research_loop import (
    ANSWERED,
    CLARIFICATION_REQUIRED,
    ResearchEpisodeResult,
    ResearchGoal,
    ResearchLimits,
    ResearchLoop,
    ResearchLoopError,
    ResearchPlanner,
    ResearchToolRegistry,
)


DEFAULT_LIMITS = ResearchLimits(
    max_elapsed_ms=30_000,
    max_planner_latency_ms=10_000,
    max_planner_turns=6,
    max_tool_calls=1,
    max_replans=4,
    tool_request_ttl_ms=8_000,
)


def _monotonic_ms() -> int:
    return time.monotonic_ns() // 1_000_000


def run_research_query(
    query: str,
    turn_id: str,
    planner: ResearchPlanner,
    weather_tool: WeatherTool,
    limits: ResearchLimits = DEFAULT_LIMITS,
) -> ResearchEpisodeResult:
    """Run one evidence-required episode with no robot capability."""

    loop = ResearchLoop(
        planner=planner,
        tools=ResearchToolRegistry(weather_tool),
        clock_ms=_monotonic_ms,
        limits=limits,
    )
    return loop.run(
        ResearchGoal(
            turn_id=turn_id,
            user_query=query,
            require_evidence=True,
        )
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Ask local Gemma one current-weather question through a "
            "bounded, read-only Open-Meteo research loop."
        )
    )
    parser.add_argument("query")
    parser.add_argument("--turn-id")
    parser.add_argument("--lm-studio-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--pretty", action="store_true")
    return parser


def _report(
    result: ResearchEpisodeResult,
) -> dict:
    report = dict(result.to_dict())
    report["status"] = (
        "completed"
        if result.termination == ANSWERED
        else "clarification_required"
        if result.termination == CLARIFICATION_REQUIRED
        else "failed"
    )
    report["mode"] = "read_only_research"
    return report


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    turn_id = args.turn_id or "turn-{}".format(secrets.token_hex(8))
    try:
        planner = LMStudioResearchPlanner(
            base_url=args.lm_studio_url,
            model=args.model,
            timeout_seconds=10.0,
        )
        result = run_research_query(
            query=args.query,
            turn_id=turn_id,
            planner=planner,
            weather_tool=OpenMeteoWeatherTool(),
        )
    except (LMStudioError, ResearchError, ResearchLoopError) as error:
        print(
            json.dumps(
                {
                    "status": "failed",
                    "mode": "read_only_research",
                    "error": type(error).__name__,
                    "message": str(error)[:240],
                },
                ensure_ascii=False,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2

    print(
        json.dumps(
            _report(result),
            ensure_ascii=False,
            indent=2 if args.pretty else None,
            sort_keys=True,
        )
    )
    if result.termination == ANSWERED:
        return 0
    if result.termination == CLARIFICATION_REQUIRED:
        return 4
    return 3


if __name__ == "__main__":
    sys.exit(main())
