"""Mac-side motion-free preflight for the foreground EV3 supervisor."""

import argparse
import json
import sys
from typing import List, Optional

from .supervisor_transport import (
    SupervisorSSHError,
    SupervisorSSHSession,
    run_motion_free_supervisor_preflight,
)


DEFAULT_CONTROLLER_ID = "ev3rstorm-01.ev3-main"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run a motion-free claim/heartbeat/arm/release/shutdown "
            "cycle against the EV3 foreground supervisor."
        )
    )
    parser.add_argument("--ssh-target", required=True)
    parser.add_argument(
        "--controller-id",
        default=DEFAULT_CONTROLLER_ID,
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    session = None
    try:
        session = SupervisorSSHSession(
            target=args.ssh_target,
            controller_id=args.controller_id,
        )
        result = run_motion_free_supervisor_preflight(session)
        print(
            json.dumps(
                result,
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            )
        )
        return 0
    except SupervisorSSHError as error:
        print(
            json.dumps(
                {
                    "status": "failed",
                    "mode": "motion-free-daemon-preflight",
                    "error": {
                        "code": getattr(
                            error,
                            "code",
                            "supervisor_preflight_failed",
                        ),
                        "message": " ".join(str(error).split())[:240],
                    },
                },
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1
    finally:
        if session is not None:
            session.close()


if __name__ == "__main__":
    sys.exit(main())
