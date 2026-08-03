import ast
import copy
import unittest
from pathlib import Path

from robot_agent.motor_phase_diagnostics import (
    PHASE_EVENT_SCHEMA,
    aggregate_motor_phase_events,
    motor_phase_events_from_result,
)
from robot_agent.physical_navigation_contract import EXPECTED_ACTION_SPECS
from tests.test_physical_navigation_core import (
    degraded_motion_result,
    partial_start_motion_result,
    recovered_motion_result,
)


def set_segment_start_positions(result, left_before, right_before):
    receipt = result["outcome"]["slices"][0]
    before_by_side = {"left": left_before, "right": right_before}
    for container in (receipt, receipt["segments"][0]):
        for motor in container["motors"]:
            before = before_by_side[motor["side"]]
            motor["position_before"] = before
            motor["position_after"] = before + motor["position_delta"]
    return result


def advance_undertravel_delta():
    spec = EXPECTED_ACTION_SPECS["ADVANCE"]
    expected = (
        abs(spec["left_speed_dps"]) * spec["total_duration_ms"] + 500
    ) // 1000
    return expected - 1


class MotorPhaseDiagnosticsTests(unittest.TestCase):
    def test_extracts_wrapped_phase_from_existing_encoder_receipts(self):
        result = set_segment_start_positions(
            degraded_motion_result(left_delta=advance_undertravel_delta()),
            -1,
            370,
        )

        events = motor_phase_events_from_result("ADVANCE", result)

        self.assertEqual(len(events), 2)
        left, right = events
        self.assertEqual(left["schema"], PHASE_EVENT_SCHEMA)
        self.assertEqual(left["phase_degrees"], 359)
        self.assertEqual(left["phase_bucket_start_degrees"], 330)
        self.assertEqual(left["start_outcome"], "undertravel")
        self.assertEqual(right["phase_degrees"], 10)
        self.assertEqual(right["phase_bucket_start_degrees"], 0)
        self.assertEqual(right["start_outcome"], "zero_start")

    def test_partial_start_distinguishes_commanded_and_uncommanded_side(self):
        result = set_segment_start_positions(
            partial_start_motion_result(),
            725,
            12,
        )

        events = motor_phase_events_from_result("ADVANCE", result)

        left, right = events
        self.assertTrue(left["commanded"])
        self.assertEqual(left["phase_degrees"], 5)
        self.assertEqual(left["start_outcome"], "undertravel")
        self.assertFalse(right["commanded"])
        self.assertEqual(right["start_outcome"], "uncommanded")

    def test_recovery_produces_one_event_per_motor_and_temporal_segment(self):
        events = motor_phase_events_from_result(
            "ADVANCE",
            recovered_motion_result(),
        )

        self.assertEqual(len(events), 4)
        self.assertEqual(
            [event["segment_kind"] for event in events],
            ["paired", "paired", "right_catch_up", "right_catch_up"],
        )
        catch_up_left, catch_up_right = events[2:]
        self.assertEqual(catch_up_left["start_outcome"], "uncommanded")
        self.assertEqual(catch_up_right["start_outcome"], "verified")
        self.assertEqual(catch_up_left["phase_degrees"], 75)
        self.assertEqual(catch_up_right["phase_degrees"], 0)

    def test_aggregation_clusters_equal_phase_across_full_rotations(self):
        first = set_segment_start_positions(
            degraded_motion_result(left_delta=0, right_delta=200),
            5,
            8,
        )
        second = set_segment_start_positions(
            degraded_motion_result(left_delta=0, right_delta=200),
            365,
            368,
        )
        events = (
            motor_phase_events_from_result("ADVANCE", first)
            + motor_phase_events_from_result("ADVANCE", second)
        )

        rows = aggregate_motor_phase_events(events)
        left = next(row for row in rows if row["side"] == "left")
        right = next(row for row in rows if row["side"] == "right")
        self.assertEqual(left["commanded_attempts"], 2)
        self.assertEqual(left["zero_starts"], 2)
        self.assertEqual(left["phase_bucket_start_degrees"], 0)
        self.assertEqual(right["verified_starts"], 0)
        self.assertEqual(right["unverified_starts"], 2)

    def test_strictly_rejects_invalid_periods_and_tampered_events(self):
        result = degraded_motion_result()
        for period, bucket in ((0, 30), (360, 17), (True, 1)):
            with self.subTest(period=period, bucket=bucket):
                with self.assertRaises(ValueError):
                    motor_phase_events_from_result(
                        "ADVANCE",
                        result,
                        phase_period_degrees=period,
                        phase_bucket_degrees=bucket,
                    )

        event = copy.deepcopy(
            motor_phase_events_from_result("ADVANCE", result)[0]
        )
        event["phase_degrees"] = 360
        with self.assertRaises(ValueError):
            aggregate_motor_phase_events((event,))

    def test_module_remains_python35_parseable(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "src"
            / "robot_agent"
            / "motor_phase_diagnostics.py"
        )
        ast.parse(
            path.read_text(encoding="utf-8"),
            filename=str(path),
            feature_version=5,
        )


if __name__ == "__main__":
    unittest.main()
