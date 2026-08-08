import threading
import unittest

from robot_agent.ev3_navigation_preflight_cli import (
    EV3NavigationPreflightError,
)
from robot_agent.ev3_reachability import (
    EV3ReachabilityError,
    EV3ReachabilityProbe,
)


def passed_report():
    return {
        "status": "passed",
        "effects": "motion_free",
        "contract_checks": {
            "motor_commands_issued": 0,
            "shutdown_confirmed": True,
            "motor_owner_closed": True,
        },
    }


class EV3ReachabilityProbeTests(unittest.TestCase):
    def make_probe(self, preflight, *, lease=None, clock_ms=lambda: 1_000):
        return EV3ReachabilityProbe(
            robot_id="ev3rstorm-01",
            controller_id="ev3rstorm-01.ev3-main",
            display_name="EV3RSTORM",
            controller_lease=lease or threading.Lock(),
            preflight=preflight,
            clock_ms=clock_ms,
        )

    def test_success_is_timestamped_history_not_a_persistent_connection(self):
        observations = []
        probe = None

        def preflight():
            observations.append(probe.snapshot())
            return passed_report()

        probe = self.make_probe(preflight)
        initial = probe.snapshot()
        initial["reachability"]["status"] = "mutated"

        result = probe.check()

        self.assertEqual(
            probe.snapshot()["reachability"]["status"],
            "passed",
        )
        self.assertEqual(observations[0]["reachability"]["status"], "checking")
        self.assertEqual(result["state"], "configured")
        self.assertEqual(result["connection_mode"], "episodic_ssh")
        self.assertEqual(result["reason_code"], "reachability_verified")
        self.assertEqual(result["last_checked_at_unix_ms"], 1_000)
        self.assertEqual(result["last_verified_at_unix_ms"], 1_000)

    def test_failed_check_is_sanitized_and_releases_the_lease(self):
        lease = threading.Lock()

        def preflight():
            raise EV3NavigationPreflightError(
                "start_failed",
                "private SSH detail",
            )

        probe = self.make_probe(preflight, lease=lease)

        with self.assertRaises(EV3ReachabilityError) as raised:
            probe.check()

        self.assertEqual(raised.exception.code, "controller_connection_failed")
        self.assertNotIn("private", str(raised.exception))
        snapshot = probe.snapshot()
        self.assertEqual(snapshot["state"], "configured")
        self.assertEqual(snapshot["reachability"], {
            "status": "failed",
            "error_code": "start_failed",
        })
        self.assertEqual(snapshot["last_checked_at_unix_ms"], 1_000)
        self.assertIsNone(snapshot["last_verified_at_unix_ms"])
        self.assertTrue(lease.acquire(blocking=False))
        lease.release()

    def test_busy_controller_rejects_before_preflight(self):
        lease = threading.Lock()
        called = []
        probe = self.make_probe(
            lambda: called.append(True),
            lease=lease,
        )
        lease.acquire()
        try:
            with self.assertRaises(EV3ReachabilityError) as raised:
                probe.check()
        finally:
            lease.release()

        self.assertEqual(raised.exception.code, "controller_busy")
        self.assertEqual(called, [])
        self.assertEqual(
            probe.snapshot()["reachability"]["status"],
            "not_checked",
        )

    def test_invalid_success_report_fails_closed(self):
        probe = self.make_probe(lambda: {"status": "passed"})

        with self.assertRaises(EV3ReachabilityError):
            probe.check()

        self.assertEqual(probe.snapshot()["reachability"], {
            "status": "failed",
            "error_code": "reachability_check_failed",
        })


if __name__ == "__main__":
    unittest.main()
