"""Motion-free cold-start and warm sensor RTT benchmark."""

import argparse
import json
import statistics
import sys
import time
from typing import Any, Callable, Dict, List, Optional

from .peripheral_transport import (
    PeripheralSSHError,
    PeripheralSSHSession,
)


DEFAULT_CONTROLLER_ID = "ev3rstorm-01.ev3-main"
Clock = Callable[[], float]


def _sample_count(value: str) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError(
            "samples must be an integer"
        ) from None
    if not 1 <= parsed <= 100:
        raise argparse.ArgumentTypeError(
            "samples must be between 1 and 100"
        )
    return parsed


def summarize_latencies(latencies_ms: List[int]) -> Dict[str, int]:
    if (
        not isinstance(latencies_ms, list)
        or not latencies_ms
        or any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 0
            for value in latencies_ms
        )
    ):
        raise ValueError("latencies_ms is invalid")
    ordered = sorted(latencies_ms)
    p95_index = max(
        0,
        min(
            len(ordered) - 1,
            int((len(ordered) * 0.95) + 0.999999) - 1,
        ),
    )
    return {
        "minimum_ms": ordered[0],
        "median_ms": int(round(statistics.median(ordered))),
        "p95_ms": ordered[p95_index],
        "maximum_ms": ordered[-1],
    }


def run_peripheral_benchmark(
    session: Any,
    role: str,
    samples: int,
    clock: Clock = time.perf_counter,
) -> Dict[str, object]:
    if (
        not isinstance(role, str)
        or not role
        or not isinstance(samples, int)
        or isinstance(samples, bool)
        or not 1 <= samples <= 100
        or not callable(clock)
    ):
        raise ValueError("Benchmark arguments are invalid")

    cold_started = clock()
    description = session.describe()
    cold_start_ms = max(
        0,
        int(round((clock() - cold_started) * 1000)),
    )
    roles = description["capabilities"][
        "configured_sensor_read"
    ]["roles"]
    if role not in roles:
        raise ValueError("Requested sensor role is not advertised")

    latencies = []
    readings = []
    last_observed_at_ms = None
    for _index in range(samples):
        started = clock()
        reading = session.read_sensor(role)
        elapsed_ms = max(
            0,
            int(round((clock() - started) * 1000)),
        )
        observed_at_ms = reading["observed_monotonic_ms"]
        if (
            last_observed_at_ms is not None
            and observed_at_ms < last_observed_at_ms
        ):
            raise ValueError(
                "Peripheral sensor timestamp moved backwards"
            )
        last_observed_at_ms = observed_at_ms
        latencies.append(elapsed_ms)
        readings.append(reading["value0"])

    return {
        "status": "completed",
        "mode": "motion-free-persistent-peripheral",
        "controller_id": description["controller_id"],
        "robot_id": description["robot_id"],
        "peripheral_instance_id": description[
            "peripheral_instance_id"
        ],
        "motion_enabled": description["motion_enabled"],
        "speech_enabled": description["speech_enabled"],
        "sensor_role": role,
        "sample_count": samples,
        "cold_describe_ms": cold_start_ms,
        "warm_sensor_rtt": summarize_latencies(latencies),
        "first_value": readings[0],
        "last_value": readings[-1],
        "single_ssh_process": True,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Measure one cold persistent EV3 peripheral handshake and "
            "warm motion-free sensor round trips."
        )
    )
    parser.add_argument("--ssh-target", required=True)
    parser.add_argument(
        "--controller-id",
        default=DEFAULT_CONTROLLER_ID,
    )
    parser.add_argument("--role", default="infrared")
    parser.add_argument(
        "--samples",
        default=10,
        type=_sample_count,
    )
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    session = None
    try:
        started = time.perf_counter()
        session = PeripheralSSHSession(
            args.ssh_target,
            args.controller_id,
        )
        result = run_peripheral_benchmark(
            session,
            args.role,
            args.samples,
            clock=time.perf_counter,
        )
        result["session_construction_to_result_ms"] = max(
            0,
            int(round((time.perf_counter() - started) * 1000)),
        )
    except (PeripheralSSHError, ValueError) as error:
        print(
            json.dumps(
                {
                    "status": "failed",
                    "mode": "motion-free-persistent-peripheral",
                    "error": "{}: {}".format(
                        type(error).__name__,
                        " ".join(str(error).split())[:200],
                    ),
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
