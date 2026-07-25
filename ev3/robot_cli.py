#!/usr/bin/env python3
"""Manual CLI for bounded EV3 hardware experiments."""

from __future__ import print_function

import argparse
import json
import os
import sys

from robot_hal import (
    MotorBusyError,
    RobotHAL,
    SafetyError,
    SpeechBusyError,
)


def default_config_path():
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "ev3rstorm.json")
    )


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=default_config_path())
    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("inventory")
    subparsers.add_parser("stop")

    sensor = subparsers.add_parser("read-sensor")
    sensor.add_argument("--role", required=True)

    motion = subparsers.add_parser("motor-test")
    motion.add_argument("--role", required=True)
    motion.add_argument("--speed-dps", required=True, type=int)
    motion.add_argument("--duration-ms", required=True, type=int)
    motion.add_argument(
        "--acknowledge-physical-motion",
        action="store_true",
        help="Required explicit acknowledgement for physical motion.",
    )

    drive = subparsers.add_parser("drive-test")
    drive.add_argument("--left-speed-dps", required=True, type=int)
    drive.add_argument("--right-speed-dps", required=True, type=int)
    drive.add_argument("--duration-ms", required=True, type=int)
    drive.add_argument(
        "--acknowledge-physical-motion",
        action="store_true",
        help="Required explicit acknowledgement for physical motion.",
    )

    speech = subparsers.add_parser("speak")
    speech.add_argument("text")
    speech.add_argument("--voice", default="sv")
    speech.add_argument("--rate-wpm", default=135, type=int)

    speech_stdin = subparsers.add_parser("speak-stdin")
    speech_stdin.add_argument("--voice", default="sv")
    speech_stdin.add_argument("--rate-wpm", default=135, type=int)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    if not args.command:
        parser.error("a command is required")

    robot = RobotHAL(args.config)
    if args.command == "inventory":
        result = robot.inventory()
    elif args.command == "stop":
        result = {"status": "stopped", "motors": robot.stop_all()}
    elif args.command == "read-sensor":
        result = robot.read_sensor(args.role)
    elif args.command == "motor-test":
        if not args.acknowledge_physical_motion:
            parser.error("motor-test requires --acknowledge-physical-motion")
        result = robot.run_timed(args.role, args.speed_dps, args.duration_ms)
    elif args.command == "drive-test":
        if not args.acknowledge_physical_motion:
            parser.error("drive-test requires --acknowledge-physical-motion")
        result = robot.drive_timed(
            args.left_speed_dps,
            args.right_speed_dps,
            args.duration_ms,
        )
    elif args.command == "speak":
        result = robot.speak(args.text, args.voice, args.rate_wpm)
    elif args.command == "speak-stdin":
        maximum = robot.config["limits"]["speech"]["max_characters"]
        text = sys.stdin.read(maximum + 1)
        if len(text) > maximum:
            raise SafetyError("Speech stdin exceeds configured limit")
        result = robot.speak(text, args.voice, args.rate_wpm)
    else:
        parser.error("unknown command")

    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    try:
        main()
    except SafetyError as error:
        print(
            json.dumps({"status": "rejected", "error": str(error)}),
            file=sys.stderr,
        )
        sys.exit(2)
    except (MotorBusyError, SpeechBusyError) as error:
        print(
            json.dumps({"status": "busy", "error": str(error)}),
            file=sys.stderr,
        )
        sys.exit(3)
    except (IOError, OSError, RuntimeError) as error:
        print(
            json.dumps({"status": "failed", "error": str(error)}),
            file=sys.stderr,
        )
        sys.exit(1)
