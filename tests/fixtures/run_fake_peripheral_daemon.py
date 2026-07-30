#!/usr/bin/env python3
"""Real-pipe harness for the motor-free peripheral host transport."""

import argparse
import sys

from ev3.peripheral_daemon import run_daemon
from ev3.robot_hal import RobotHAL


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--sysfs-root", required=True)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    robot = RobotHAL(
        args.config,
        sysfs_root=args.sysfs_root,
    )
    result = run_daemon(
        robot,
        getattr(sys.stdin, "buffer", sys.stdin),
        getattr(sys.stdout, "buffer", sys.stdout),
        max_session_ms=10000,
        max_requests=32,
    )
    return 0 if result["transport_failure"] is None else 1


if __name__ == "__main__":
    sys.exit(main())
