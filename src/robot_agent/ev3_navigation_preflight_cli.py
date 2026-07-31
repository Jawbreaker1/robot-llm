"""Motion-free host preflight for the foreground EV3 navigation worker.

The fixed operation sequence deliberately exposes no model decision seam and
contains no movement request.  ``EV3NavigationSSHTransport`` remains the
protocol authority; this module only adds preflight-specific invariants and a
sanitized evidence report.
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Callable, List, Mapping, Optional, Sequence

from .ev3_navigation_transport import EV3NavigationSSHTransport
from .physical_navigation_contract import validate_observation


REPORT_SCHEMA = "robot-ev3-navigation-preflight/v1"
ERROR_SCHEMA = "robot-ev3-navigation-preflight-error/v1"
DEFAULT_CONTROLLER_ID = "ev3rstorm-01.ev3-main"
DEFAULT_REMOTE_WORKER_PATH = (
    "/home/robot/robot-llm/ev3/navigation_worker_cli.py"
)
DEFAULT_CONNECT_TIMEOUT_SECONDS = 5
DEFAULT_STARTUP_TIMEOUT_SECONDS = 30.0
DEFAULT_REQUEST_TIMEOUT_SECONDS = 8.0
_ACTIVE_MOTOR_STATES = frozenset(("running", "ramping"))
_SAFE_FAILURE_MESSAGES = {
    "transport_creation_failed": "Navigation transport could not be created",
    "start_failed": "Navigation worker could not be started",
    "describe_failed": "Worker description did not pass preflight",
    "observe_failed": "Stationary observation did not pass preflight",
    "stop_failed": "Verified worker stop did not pass preflight",
    "shutdown_failed": "Verified worker shutdown did not pass preflight",
    "close_failed": "Navigation transport did not close cleanly",
}


class EV3NavigationPreflightError(RuntimeError):
    """Sanitized failure with cleanup evidence safe for CLI output."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        completed_steps: Sequence[str] = (),
        abort_attempted: bool = False,
        abort_succeeded: bool = False,
        close_attempted: bool = False,
        close_succeeded: bool = False,
    ):
        self.code = code
        self.completed_steps = tuple(completed_steps)
        self.abort_attempted = abort_attempted
        self.abort_succeeded = abort_succeeded
        self.close_attempted = close_attempted
        self.close_succeeded = close_succeeded
        super().__init__(message)

    def to_report(self) -> Mapping[str, object]:
        return {
            "schema": ERROR_SCHEMA,
            "status": "failed",
            "mode": "motion_free_navigation_worker_preflight",
            "effects": "motion_free",
            "completed_steps": list(self.completed_steps),
            "cleanup": {
                "abort_attempted": self.abort_attempted,
                "abort_succeeded": self.abort_succeeded,
                "close_attempted": self.close_attempted,
                "close_succeeded": self.close_succeeded,
            },
            "error": {
                "code": self.code,
                "message": str(self),
            },
        }


def _state_version(response: object) -> int:
    if not isinstance(response, Mapping):
        raise ValueError("response is not an object")
    value = response.get("state_version")
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError("response state version is invalid")
    return value


def _result(response: object) -> Mapping[str, object]:
    if not isinstance(response, Mapping):
        raise ValueError("response is not an object")
    value = response.get("result")
    if not isinstance(value, Mapping):
        raise ValueError("response result is invalid")
    return value


def _validated_observation(
    response: object,
    observation: object,
) -> Mapping[str, object]:
    checked = validate_observation(observation)
    if checked["state_version"] != _state_version(response):
        raise ValueError("observation is not correlated")
    active = any(
        frozenset(motor["state"].split()) & _ACTIVE_MOTOR_STATES
        for motor in checked["motors"]
    )
    budgets = checked["budgets"]
    if (
        active
        or budgets["pulse_count"] != 0
        or budgets["pulse_duration_ms"] != 0
        or budgets["motion_fault_latched"] is not False
    ):
        raise ValueError("observation is not motion-free")
    return checked


def _validate_describe(response: object) -> Mapping[str, object]:
    result = _result(response)
    if "observation" not in result:
        raise ValueError("worker description omitted its observation")
    return _validated_observation(response, result["observation"])


def _validate_observe(response: object) -> Mapping[str, object]:
    result = _result(response)
    if set(result) != {"observation"}:
        raise ValueError("worker observation result is invalid")
    return _validated_observation(response, result["observation"])


def _validate_stop_proof(value: object) -> None:
    if (
        not isinstance(value, Mapping)
        or value.get("stop_confirmed") is not True
        or value.get("errors") not in (None, [])
        or value.get("fault_tokens") not in (None, {})
        or value.get("cleanup_errors") not in (None, [])
    ):
        raise ValueError("worker stop proof is invalid")


def _validate_stop(response: object) -> None:
    result = _result(response)
    if set(result) != {"outcome", "observation", "stop"}:
        raise ValueError("worker stop result is invalid")
    outcome = result["outcome"]
    if (
        not isinstance(outcome, Mapping)
        or outcome.get("kind") != "stop"
        or outcome.get("status") != "completed"
        or outcome.get("stop_confirmed") is not True
    ):
        raise ValueError("worker stop outcome is invalid")
    observation = _validated_observation(
        response,
        result["observation"],
    )
    if observation["last_outcome"] != outcome:
        raise ValueError("worker stop outcome is not correlated")
    _validate_stop_proof(result["stop"])


def _validate_shutdown(response: object) -> None:
    result = _result(response)
    if set(result) != {"outcome", "observation", "close"}:
        raise ValueError("worker shutdown result is invalid")
    outcome = result["outcome"]
    expected_outcome_fields = {
        "kind",
        "status",
        "completed_monotonic_ms",
        "stop_confirmed",
        "motor_owner_closed",
    }
    if (
        not isinstance(outcome, Mapping)
        or set(outcome) != expected_outcome_fields
        or outcome["kind"] != "shutdown"
        or outcome["status"] != "completed"
        or outcome["stop_confirmed"] is not True
        or outcome["motor_owner_closed"] is not True
        or isinstance(outcome["completed_monotonic_ms"], bool)
        or not isinstance(outcome["completed_monotonic_ms"], int)
    ):
        raise ValueError("worker shutdown outcome is invalid")
    observation = _validated_observation(
        response,
        result["observation"],
    )
    if observation["last_outcome"] != outcome:
        raise ValueError("worker shutdown outcome is not correlated")
    _validate_stop_proof(result["close"])


def _cleanup_after_failure(transport: object):
    abort_attempted = transport is not None
    abort_succeeded = False
    close_attempted = transport is not None
    close_succeeded = False
    if transport is not None:
        try:
            transport.abort()
            abort_succeeded = True
        except BaseException:
            pass
        try:
            transport.close()
            close_succeeded = True
        except BaseException:
            pass
    return (
        abort_attempted,
        abort_succeeded,
        close_attempted,
        close_succeeded,
    )


def run_ev3_navigation_preflight(
    ssh_target: str,
    *,
    controller_id: str = DEFAULT_CONTROLLER_ID,
    remote_worker_path: str = DEFAULT_REMOTE_WORKER_PATH,
    connect_timeout_seconds: int = DEFAULT_CONNECT_TIMEOUT_SECONDS,
    startup_timeout_seconds: float = DEFAULT_STARTUP_TIMEOUT_SECONDS,
    request_timeout_seconds: float = DEFAULT_REQUEST_TIMEOUT_SECONDS,
    transport_factory: Callable[..., object] = EV3NavigationSSHTransport,
) -> Mapping[str, object]:
    """Run the fixed stationary worker lifecycle and return safe evidence."""

    transport = None
    completed_steps = []
    stage = "transport_creation"
    try:
        transport = transport_factory(
            target=ssh_target,
            controller_id=controller_id,
            remote_worker_path=remote_worker_path,
            connect_timeout_seconds=connect_timeout_seconds,
        )

        stage = "start"
        transport.start()
        completed_steps.append("start")

        stage = "describe"
        describe = transport.request(
            "describe",
            {},
            startup_timeout_seconds,
        )
        describe_observation = _validate_describe(describe)
        completed_steps.append("describe")

        stage = "observe"
        observe = transport.request(
            "observe",
            {},
            request_timeout_seconds,
        )
        observation = _validate_observe(observe)
        completed_steps.append("observe")

        stage = "stop"
        stop = transport.request(
            "stop",
            {},
            request_timeout_seconds,
        )
        _validate_stop(stop)
        completed_steps.append("stop")

        stage = "shutdown"
        shutdown = transport.request(
            "shutdown",
            {},
            request_timeout_seconds,
        )
        _validate_shutdown(shutdown)
        completed_steps.append("shutdown")

        stage = "close"
        transport.close()
        completed_steps.append("close")
    except BaseException as error:
        cleanup = _cleanup_after_failure(transport)
        if not isinstance(error, Exception):
            raise
        code = "{}_failed".format(stage)
        message = _SAFE_FAILURE_MESSAGES.get(
            code,
            "Navigation worker preflight failed",
        )
        raise EV3NavigationPreflightError(
            code,
            message,
            completed_steps=completed_steps,
            abort_attempted=cleanup[0],
            abort_succeeded=cleanup[1],
            close_attempted=cleanup[2],
            close_succeeded=cleanup[3],
        ) from error

    return {
        "schema": REPORT_SCHEMA,
        "status": "passed",
        "mode": "motion_free_navigation_worker_preflight",
        "effects": "motion_free",
        "completed_steps": completed_steps,
        "contract_checks": {
            "description_validated": True,
            "stationary_observation_validated": True,
            "stop_confirmed": True,
            "shutdown_confirmed": True,
            "motor_owner_closed": True,
            "motor_commands_issued": 0,
        },
        "state_versions": {
            "describe": _state_version(describe),
            "observe": _state_version(observe),
            "stop": _state_version(stop),
            "shutdown": _state_version(shutdown),
        },
        "observation": {
            "touch_pressed": observation["touch"]["pressed"],
            "infrared_blocked": observation["infrared"]["blocked"],
            "motor_count": len(observation["motors"]),
            "motors_active": False,
            "motion_fault_latched": False,
            "initial_state_version": describe_observation[
                "state_version"
            ],
        },
    }


def _bounded_connect_timeout(value: str) -> int:
    try:
        checked = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError("must be an integer") from None
    if not 1 <= checked <= 30:
        raise argparse.ArgumentTypeError("must be between 1 and 30")
    return checked


def _bounded_request_timeout(value: str) -> float:
    try:
        checked = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError("must be a number") from None
    if not 0.1 <= checked <= 60.0:
        raise argparse.ArgumentTypeError("must be between 0.1 and 60")
    return checked


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Start the foreground EV3 navigation worker, validate one "
            "stationary lifecycle, and close it without requesting movement."
        )
    )
    parser.add_argument("--ssh-target", required=True)
    parser.add_argument(
        "--controller-id",
        default=DEFAULT_CONTROLLER_ID,
    )
    parser.add_argument(
        "--remote-worker-path",
        default=DEFAULT_REMOTE_WORKER_PATH,
    )
    parser.add_argument(
        "--connect-timeout-seconds",
        type=_bounded_connect_timeout,
        default=DEFAULT_CONNECT_TIMEOUT_SECONDS,
    )
    parser.add_argument(
        "--startup-timeout-seconds",
        type=_bounded_request_timeout,
        default=DEFAULT_STARTUP_TIMEOUT_SECONDS,
        help=(
            "deadline for cold worker startup and describe only "
            "(default: 30)"
        ),
    )
    parser.add_argument(
        "--request-timeout-seconds",
        type=_bounded_request_timeout,
        default=DEFAULT_REQUEST_TIMEOUT_SECONDS,
    )
    parser.add_argument("--pretty", action="store_true")
    return parser


def _write_json(value: object, stream, *, pretty: bool) -> None:
    print(
        json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            indent=2 if pretty else None,
            separators=None if pretty else (",", ":"),
            sort_keys=True,
        ),
        file=stream,
    )


def main(
    argv: Optional[List[str]] = None,
    *,
    transport_factory: Callable[..., object] = EV3NavigationSSHTransport,
) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = run_ev3_navigation_preflight(
            args.ssh_target,
            controller_id=args.controller_id,
            remote_worker_path=args.remote_worker_path,
            connect_timeout_seconds=args.connect_timeout_seconds,
            startup_timeout_seconds=args.startup_timeout_seconds,
            request_timeout_seconds=args.request_timeout_seconds,
            transport_factory=transport_factory,
        )
    except EV3NavigationPreflightError as error:
        _write_json(error.to_report(), sys.stderr, pretty=args.pretty)
        return 1
    _write_json(result, sys.stdout, pretty=args.pretty)
    return 0


if __name__ == "__main__":
    sys.exit(main())
