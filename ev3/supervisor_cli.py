#!/usr/bin/env python3
"""Rörelsefri CLI för preflight av den lokala EV3-supervisorn."""

from __future__ import print_function

import argparse
import json
import os
import sys
import time

try:
    from .robot_hal import RobotHAL, SafetyError
    from .supervisor import (
        EV3Supervisor,
        JSONLAuditLog,
        STATE_CLOSED,
        STATE_DISARMED,
    )
except (ImportError, ValueError, SystemError):
    from robot_hal import RobotHAL, SafetyError
    from supervisor import (
        EV3Supervisor,
        JSONLAuditLog,
        STATE_CLOSED,
        STATE_DISARMED,
    )


DEFAULT_AUDIT_PATH = "/tmp/robot-llm-supervisor-audit.jsonl"


def default_config_path():
    return os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "ev3rstorm.json",
        )
    )


def run_preflight(robot, audit_log):
    """Acquire lifetime ownership, verify stopped hardware, then release."""
    supervisor = None
    supervisor_initialized = False
    result = None
    pending_error = None
    shutdown = None
    try:
        supervisor = EV3Supervisor.__new__(EV3Supervisor)
        EV3Supervisor.__init__(supervisor, robot)
        supervisor_initialized = True
        poll_seconds = (
            supervisor.limits["poll_interval_ms"] / 1000.0
        )
        for _ in range(
            supervisor.limits["touch_release_samples"]
        ):
            supervisor.poll_once()
            if poll_seconds > 0:
                time.sleep(poll_seconds)

        status = supervisor.status()
        completed = (
            status["state"] == STATE_DISARMED
            and status["fault"] is None
            and status["motion_allowed"] is False
            and status["touch"] == 0
            and status.get("touch_released_samples", 0)
            >= supervisor.limits["touch_release_samples"]
        )
        result = {
            "status": "completed" if completed else "failed",
            "mode": "motion-free-preflight",
            "motor_start_commands": 0,
            "supervisor": status,
        }
    except BaseException as error:
        pending_error = error
    finally:
        if (
            supervisor is not None
            and (
                supervisor_initialized
                or hasattr(supervisor, "_owner")
            )
        ):
            try:
                shutdown = supervisor.close()
            except BaseException as error:
                if pending_error is None:
                    pending_error = error
            for event in supervisor.drain_audit_events():
                try:
                    audit_log.append(event)
                except BaseException as error:
                    if pending_error is None:
                        pending_error = error
                    break

    if result is not None:
        result["shutdown"] = shutdown
        if (
            shutdown is None
            or shutdown.get("state") != STATE_CLOSED
            or shutdown.get("audit_complete") is not True
        ):
            result["status"] = "failed"
    if pending_error is not None:
        raise pending_error
    return result


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Motion-free EV3 supervisor preflight. "
            "It writes stop, reads touch/motor state, and never starts motion."
        )
    )
    parser.add_argument(
        "--config",
        default=default_config_path(),
    )
    parser.add_argument(
        "--audit-log",
        default=DEFAULT_AUDIT_PATH,
    )
    parser.add_argument(
        "command",
        choices=("preflight",),
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    audit = None
    try:
        audit = JSONLAuditLog(args.audit_log)
        robot = RobotHAL(args.config)
        result = run_preflight(robot, audit)
        print(
            json.dumps(
                result,
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            )
        )
        return 0 if result["status"] == "completed" else 1
    except (
        SafetyError,
        IOError,
        OSError,
        RuntimeError,
        ValueError,
    ) as error:
        print(
            json.dumps(
                {
                    "status": "failed",
                    "mode": "motion-free-preflight",
                    "error": str(error),
                },
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1
    finally:
        if audit is not None:
            audit.close()


if __name__ == "__main__":
    sys.exit(main())
