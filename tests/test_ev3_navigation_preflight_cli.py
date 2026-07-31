import contextlib
import io
import json
import unittest

from robot_agent.ev3_navigation_preflight_cli import (
    DEFAULT_CONNECT_TIMEOUT_SECONDS,
    DEFAULT_CONTROLLER_ID,
    DEFAULT_REMOTE_WORKER_PATH,
    DEFAULT_REQUEST_TIMEOUT_SECONDS,
    DEFAULT_STARTUP_TIMEOUT_SECONDS,
    EV3NavigationPreflightError,
    main,
    run_ev3_navigation_preflight,
)
from robot_agent.ev3_navigation_transport import (
    EV3NavigationTransportError,
)


def observation(version, last_outcome=None):
    return {
        "state_version": version,
        "observed_monotonic_ms": version * 100,
        "touch": {"value0": 0, "pressed": False},
        "infrared": {
            "raw": 54,
            "filtered": 55,
            "blocked": False,
            "reason": "clear_hysteresis_hold",
            "sample_count": 5,
        },
        "motors": [
            {"role": "arm_a", "position": 0, "state": ""},
            {"role": "drive_b", "position": 0, "state": ""},
            {"role": "drive_c", "position": 0, "state": ""},
        ],
        "last_outcome": (
            {"kind": "startup", "status": "ready"}
            if last_outcome is None
            else dict(last_outcome)
        ),
        "budgets": {
            "pulse_count": 0,
            "pulse_count_remaining": 40,
            "pulse_duration_ms": 0,
            "pulse_duration_ms_remaining": 32_000,
            "process_ms_remaining": 40_000,
            "motion_fault_latched": False,
        },
    }


def stop_proof():
    return {
        "stop_attempts": [],
        "stop_confirmed": True,
        "states": {},
        "positions": {},
        "fault_tokens": {},
        "errors": [],
    }


class FakeTransport:
    def __init__(
        self,
        *,
        target,
        controller_id,
        remote_worker_path,
        connect_timeout_seconds,
        fail_on=None,
        invalid_shutdown=False,
        close_fails=False,
    ):
        self.constructor = {
            "target": target,
            "controller_id": controller_id,
            "remote_worker_path": remote_worker_path,
            "connect_timeout_seconds": connect_timeout_seconds,
        }
        self.fail_on = fail_on
        self.invalid_shutdown = invalid_shutdown
        self.close_fails = close_fails
        self.events = []
        self.version = 0

    def start(self):
        self.events.append(("start",))
        if self.fail_on == "start":
            raise EV3NavigationTransportError("private start detail")

    def request(self, operation, arguments, timeout_seconds):
        self.events.append(
            ("request", operation, dict(arguments), timeout_seconds)
        )
        if self.fail_on == operation:
            raise EV3NavigationTransportError(
                "secret target/path/request detail"
            )
        self.version += 1
        if operation == "describe":
            result = {
                "observation": observation(self.version),
            }
        elif operation == "observe":
            result = {
                "observation": observation(self.version),
            }
        elif operation == "stop":
            outcome = {
                "kind": "stop",
                "status": "completed",
                "completed_monotonic_ms": self.version * 100,
                "stop_confirmed": True,
            }
            result = {
                "outcome": outcome,
                "observation": observation(self.version, outcome),
                "stop": stop_proof(),
            }
        elif operation == "shutdown":
            outcome = {
                "kind": "shutdown",
                "status": "completed",
                "completed_monotonic_ms": self.version * 100,
                "stop_confirmed": True,
                "motor_owner_closed": not self.invalid_shutdown,
            }
            result = {
                "outcome": outcome,
                "observation": observation(self.version, outcome),
                "close": stop_proof(),
            }
        else:
            raise AssertionError(operation)
        return {
            "state_version": self.version,
            "result": result,
        }

    def abort(self):
        self.events.append(("abort",))

    def close(self):
        self.events.append(("close",))
        if self.close_fails:
            raise EV3NavigationTransportError("private close detail")


class Factory:
    def __init__(self, **fake_options):
        self.fake_options = fake_options
        self.instances = []

    def __call__(self, **kwargs):
        instance = FakeTransport(**kwargs, **self.fake_options)
        self.instances.append(instance)
        return instance


class EV3NavigationPreflightCLITests(unittest.TestCase):
    def test_happy_path_is_fixed_motion_free_and_sanitized(self):
        factory = Factory()

        result = run_ev3_navigation_preflight(
            "robot@192.168.50.111",
            transport_factory=factory,
        )

        transport = factory.instances[0]
        self.assertEqual(
            transport.constructor,
            {
                "target": "robot@192.168.50.111",
                "controller_id": DEFAULT_CONTROLLER_ID,
                "remote_worker_path": DEFAULT_REMOTE_WORKER_PATH,
                "connect_timeout_seconds": (
                    DEFAULT_CONNECT_TIMEOUT_SECONDS
                ),
            },
        )
        self.assertEqual(
            transport.events,
            [
                ("start",),
                (
                    "request",
                    "describe",
                    {},
                    DEFAULT_STARTUP_TIMEOUT_SECONDS,
                ),
                (
                    "request",
                    "observe",
                    {},
                    DEFAULT_REQUEST_TIMEOUT_SECONDS,
                ),
                (
                    "request",
                    "stop",
                    {},
                    DEFAULT_REQUEST_TIMEOUT_SECONDS,
                ),
                (
                    "request",
                    "shutdown",
                    {},
                    DEFAULT_REQUEST_TIMEOUT_SECONDS,
                ),
                ("close",),
            ],
        )
        serialized = json.dumps(result, sort_keys=True)
        self.assertEqual(result["status"], "passed")
        self.assertEqual(result["effects"], "motion_free")
        self.assertEqual(
            result["completed_steps"],
            ["start", "describe", "observe", "stop", "shutdown", "close"],
        )
        self.assertEqual(
            result["contract_checks"]["motor_commands_issued"],
            0,
        )
        for forbidden in (
            "192.168.50.111",
            "robot@",
            DEFAULT_REMOTE_WORKER_PATH,
            "pulse",
            "scan_turn",
        ):
            self.assertNotIn(forbidden, serialized)

    def test_transport_failure_aborts_closes_and_hides_raw_error(self):
        factory = Factory(fail_on="observe")

        with self.assertRaises(EV3NavigationPreflightError) as caught:
            run_ev3_navigation_preflight(
                "robot@private-host",
                remote_worker_path="/private/worker.py",
                transport_factory=factory,
            )

        transport = factory.instances[0]
        self.assertEqual(
            transport.events[-2:],
            [("abort",), ("close",)],
        )
        operations = [
            event[1]
            for event in transport.events
            if event[0] == "request"
        ]
        self.assertEqual(operations, ["describe", "observe"])
        report = caught.exception.to_report()
        serialized = json.dumps(report, sort_keys=True)
        self.assertEqual(report["error"]["code"], "observe_failed")
        self.assertEqual(
            report["completed_steps"],
            ["start", "describe"],
        )
        self.assertEqual(
            report["cleanup"],
            {
                "abort_attempted": True,
                "abort_succeeded": True,
                "close_attempted": True,
                "close_succeeded": True,
            },
        )
        for forbidden in (
            "private-host",
            "/private/worker.py",
            "secret target/path/request detail",
        ):
            self.assertNotIn(forbidden, serialized)

    def test_invalid_shutdown_proof_fails_closed(self):
        factory = Factory(invalid_shutdown=True)

        with self.assertRaises(EV3NavigationPreflightError) as caught:
            run_ev3_navigation_preflight(
                "robot@ev3.local",
                transport_factory=factory,
            )

        self.assertEqual(
            caught.exception.code,
            "shutdown_failed",
        )
        self.assertEqual(
            caught.exception.completed_steps,
            ("start", "describe", "observe", "stop"),
        )
        self.assertEqual(
            factory.instances[0].events[-2:],
            [("abort",), ("close",)],
        )

    def test_close_failure_still_aborts_and_retries_cleanup_close(self):
        factory = Factory(close_fails=True)

        with self.assertRaises(EV3NavigationPreflightError) as caught:
            run_ev3_navigation_preflight(
                "robot@ev3.local",
                transport_factory=factory,
            )

        self.assertEqual(caught.exception.code, "close_failed")
        self.assertTrue(caught.exception.abort_succeeded)
        self.assertFalse(caught.exception.close_succeeded)
        self.assertEqual(
            factory.instances[0].events[-3:],
            [("close",), ("abort",), ("close",)],
        )

    def test_cli_prints_success_or_sanitized_failure_json(self):
        success_output = io.StringIO()
        success_factory = Factory()
        with contextlib.redirect_stdout(success_output):
            status = main(
                [
                    "--ssh-target",
                    "robot@ev3.local",
                    "--request-timeout-seconds",
                    "3.5",
                    "--startup-timeout-seconds",
                    "24",
                ],
                transport_factory=success_factory,
            )
        self.assertEqual(status, 0)
        self.assertEqual(
            json.loads(success_output.getvalue())["status"],
            "passed",
        )
        request_events = [
            event
            for event in success_factory.instances[0].events
            if event[0] == "request"
        ]
        self.assertEqual(request_events[0][3], 24.0)
        self.assertTrue(
            all(event[3] == 3.5 for event in request_events[1:])
        )

        error_output = io.StringIO()
        failure_factory = Factory(fail_on="start")
        with contextlib.redirect_stderr(error_output):
            status = main(
                ["--ssh-target", "robot@private-host"],
                transport_factory=failure_factory,
            )
        self.assertEqual(status, 1)
        error = json.loads(error_output.getvalue())
        self.assertEqual(error["status"], "failed")
        self.assertEqual(error["error"]["code"], "start_failed")
        self.assertNotIn("private-host", error_output.getvalue())

    def test_active_motor_or_nonzero_motion_budget_is_rejected(self):
        class NonStationaryTransport(FakeTransport):
            def request(self, operation, arguments, timeout_seconds):
                response = super().request(
                    operation,
                    arguments,
                    timeout_seconds,
                )
                if operation == "describe":
                    response["result"]["observation"]["motors"][0][
                        "state"
                    ] = "running"
                    response["result"]["observation"]["budgets"][
                        "pulse_count"
                    ] = 1
                return response

        instances = []

        def factory(**kwargs):
            instance = NonStationaryTransport(**kwargs)
            instances.append(instance)
            return instance

        with self.assertRaises(EV3NavigationPreflightError) as caught:
            run_ev3_navigation_preflight(
                "robot@ev3.local",
                transport_factory=factory,
            )
        self.assertEqual(caught.exception.code, "describe_failed")
        self.assertEqual(instances[0].events[-2:], [("abort",), ("close",)])


if __name__ == "__main__":
    unittest.main()
