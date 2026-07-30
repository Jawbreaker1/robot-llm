import unittest

from robot_agent.peripheral_benchmark_cli import (
    run_peripheral_benchmark,
    summarize_latencies,
)


class SequenceClock:
    def __init__(self, values):
        self.values = list(values)

    def __call__(self):
        return self.values.pop(0)


class FakeSession:
    def __init__(self, readings=None):
        self.readings = list(
            [31, 30, 29] if readings is None else readings
        )
        self.observed = 1000
        self.calls = []

    def describe(self):
        self.calls.append(("describe", None))
        return {
            "controller_id": "ev3rstorm-01.ev3-main",
            "robot_id": "ev3rstorm-01",
            "peripheral_instance_id": "instance-1",
            "motion_enabled": False,
            "speech_enabled": False,
            "capabilities": {
                "configured_sensor_read": {
                    "enabled": True,
                    "roles": ["color", "infrared", "touch"],
                },
            },
        }

    def read_sensor(self, role):
        self.calls.append(("read_sensor", role))
        self.observed += 25
        return {
            "observed_monotonic_ms": self.observed,
            "value0": self.readings.pop(0),
        }


class PeripheralBenchmarkTests(unittest.TestCase):
    def test_summary_uses_nearest_rank_p95(self):
        self.assertEqual(
            summarize_latencies([8, 2, 4, 10, 6]),
            {
                "minimum_ms": 2,
                "median_ms": 6,
                "p95_ms": 10,
                "maximum_ms": 10,
            },
        )

    def test_benchmark_separates_cold_describe_and_warm_reads(self):
        session = FakeSession()
        clock = SequenceClock(
            [
                0.000,
                0.400,
                1.000,
                1.010,
                2.000,
                2.020,
                3.000,
                3.030,
            ]
        )

        result = run_peripheral_benchmark(
            session,
            "infrared",
            3,
            clock=clock,
        )

        self.assertEqual(result["cold_describe_ms"], 400)
        self.assertEqual(
            result["warm_sensor_rtt"],
            {
                "minimum_ms": 10,
                "median_ms": 20,
                "p95_ms": 30,
                "maximum_ms": 30,
            },
        )
        self.assertEqual(result["first_value"], 31)
        self.assertEqual(result["last_value"], 29)
        self.assertTrue(result["single_ssh_process"])
        self.assertEqual(
            session.calls,
            [
                ("describe", None),
                ("read_sensor", "infrared"),
                ("read_sensor", "infrared"),
                ("read_sensor", "infrared"),
            ],
        )

    def test_unadvertised_role_and_timestamp_regression_fail(self):
        with self.assertRaises(ValueError):
            run_peripheral_benchmark(
                FakeSession(),
                "camera",
                1,
                clock=SequenceClock([0, 0.1]),
            )

        session = FakeSession(readings=[1, 2])
        real_read = session.read_sensor

        def regressing(role):
            result = real_read(role)
            if len(session.readings) == 0:
                result["observed_monotonic_ms"] = 1
            return result

        session.read_sensor = regressing
        with self.assertRaises(ValueError):
            run_peripheral_benchmark(
                session,
                "infrared",
                2,
                clock=SequenceClock(
                    [0, 0.1, 1, 1.01, 2, 2.01]
                ),
            )


if __name__ == "__main__":
    unittest.main()
