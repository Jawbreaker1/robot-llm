"""CLI for the read-only EV3 runtime deployment preflight."""

import argparse
import json
import sys
from typing import List, Optional

from .ev3_runtime_preflight import (
    DEFAULT_COMMAND_TIMEOUT_SECONDS,
    EV3RuntimePreflightError,
    MAX_COMMAND_TIMEOUT_SECONDS,
    run_ev3_runtime_preflight,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare a fixed local EV3 runtime profile with its deployed "
            "files over strict SSH. No daemon or motion is started."
        )
    )
    parser.add_argument("--ssh-target", required=True)
    parser.add_argument(
        "--profile",
        choices=("peripheral", "supervisor", "navigation-worker"),
        default="peripheral",
    )
    parser.add_argument(
        "--local-root",
        default=".",
        help="local robot-llm checkout (default: current directory)",
    )
    parser.add_argument(
        "--command-timeout-seconds",
        type=int,
        default=DEFAULT_COMMAND_TIMEOUT_SECONDS,
        metavar="SECONDS",
        help=(
            "bounded whole-command deadline (1-{}, default: {})"
        ).format(
            MAX_COMMAND_TIMEOUT_SECONDS,
            DEFAULT_COMMAND_TIMEOUT_SECONDS,
        ),
    )
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = run_ev3_runtime_preflight(
            args.ssh_target,
            profile=args.profile,
            local_root=args.local_root,
            command_timeout_seconds=args.command_timeout_seconds,
        )
    except EV3RuntimePreflightError as error:
        print(
            json.dumps(
                {
                    "schema_version": 1,
                    "status": "failed",
                    "mode": "ev3-runtime-deployment-preflight",
                    "effects": "read_only",
                    "error_code": error.code,
                    "error": " ".join(str(error).split())[:240],
                },
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1

    print(
        json.dumps(
            result,
            ensure_ascii=False,
            indent=2 if args.pretty else None,
            separators=None if args.pretty else (",", ":"),
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
