"""Command-line entry point for the read-only EV3 Wi-Fi preflight."""

import argparse
import json
import sys
from typing import List, Optional

from .ev3_wifi_preflight import (
    EV3WiFiPreflightError,
    run_ev3_wifi_preflight,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Inventory EV3 Wi-Fi readiness over the existing USB SSH "
            "link without changing network state."
        )
    )
    parser.add_argument("--ssh-target", required=True)
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = run_ev3_wifi_preflight(args.ssh_target)
    except EV3WiFiPreflightError as error:
        print(
            json.dumps(
                {
                    "status": "failed",
                    "mode": "ev3-wifi-read-only-preflight",
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
