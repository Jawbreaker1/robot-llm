#!/usr/bin/env python3
"""Subprocess harness for motion-free supervisor transport tests only."""

import argparse
import json
import signal
import sys
import threading

import ev3.supervisor as supervisor_module
from ev3.robot_hal import RobotHAL
from ev3.supervisor import JSONLAuditLog
from ev3.supervisor_daemon import run_daemon


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--sysfs-root", required=True)
    parser.add_argument("--lock-path", required=True)
    parser.add_argument("--audit-log", required=True)
    parser.add_argument("--write-log", required=True)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    shutdown = threading.Event()

    def stop_signal(_signum, _frame):
        shutdown.set()

    for name in ("SIGHUP", "SIGINT", "SIGTERM"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), stop_signal)

    real_write = supervisor_module.write_text

    def recording_write(path, value):
        with open(args.write_log, "a", encoding="utf-8") as handle:
            handle.write(
                json.dumps(
                    {"path": path, "value": value},
                    separators=(",", ":"),
                    sort_keys=True,
                )
                + "\n"
            )
        real_write(path, value)

    supervisor_module.write_text = recording_write
    audit = JSONLAuditLog(args.audit_log)
    try:
        robot = RobotHAL(
            args.config,
            sysfs_root=args.sysfs_root,
            lock_path=args.lock_path,
        )
        result = run_daemon(
            robot,
            audit,
            getattr(sys.stdin, "buffer", sys.stdin),
            getattr(sys.stdout, "buffer", sys.stdout),
            allow_one_drive_test=False,
            max_session_ms=10000,
            external_shutdown_event=shutdown,
        )
        status = result["status"]
        return (
            0
            if (
                status.get("state") == "CLOSED"
                and status.get("audit_complete") is True
            )
            else 1
        )
    finally:
        audit.close()


if __name__ == "__main__":
    sys.exit(main())
