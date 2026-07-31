import ast
import json
from pathlib import Path
import unittest

from ev3.encoder_recovery import (
    DECISION_ABORT,
    DECISION_CATCH_UP,
    DECISION_NO_RECOVERY,
    DECISION_RETRY_PAIR,
    REASON_CATCH_UP_ATTEMPT_BUDGET,
    REASON_COMMAND_SATISFIED,
    REASON_DURATION_BUDGET,
    REASON_ENCODER_BUDGET,
    REASON_ENCODER_DIRECTION_MISMATCH,
    REASON_LEFT_LAGGING,
    REASON_PAIRED_UNDERTRAVEL,
    REASON_PAIR_RETRY_ATTEMPT_BUDGET,
    REASON_RIGHT_LAGGING,
    REASON_TOTAL_ATTEMPT_BUDGET,
    EncoderRecoveryBudget,
    EncoderRecoveryPolicy,
)


def policy(**overrides):
    values = {
        "minimum_progress_degrees": 3,
        "catch_up_leader_minimum_degrees": 12,
        "acceptable_completion_percent": 75,
        "maximum_progress_skew_percent": 15,
        "maximum_catch_up_attempts": 2,
        "maximum_pair_retry_attempts": 1,
        "maximum_total_attempts": 3,
        "maximum_step_duration_ms": 800,
        "maximum_total_recovery_duration_ms": 1600,
        "maximum_total_recovery_encoder_degrees": 400,
    }
    values.update(overrides)
    return EncoderRecoveryPolicy(**values)


def decide(
    active_policy=None,
    budget=None,
    expected_left=200,
    expected_right=200,
    observed_left=174,
    observed_right=0,
    duration_ms=800,
):
    return (active_policy or policy()).decide(
        expected_left,
        expected_right,
        observed_left,
        observed_right,
        duration_ms,
        budget or EncoderRecoveryBudget(),
    )


class EncoderRecoveryPolicyTests(unittest.TestCase):
    def test_policy_requires_explicit_valid_thresholds_and_budgets(self):
        expected = {
            "minimum_progress_degrees": 3,
            "catch_up_leader_minimum_degrees": 12,
            "acceptable_completion_percent": 75,
            "maximum_progress_skew_percent": 15,
            "maximum_catch_up_attempts": 2,
            "maximum_pair_retry_attempts": 1,
            "maximum_total_attempts": 3,
            "maximum_step_duration_ms": 800,
            "maximum_total_recovery_duration_ms": 1600,
            "maximum_total_recovery_encoder_degrees": 400,
        }
        self.assertEqual(policy().to_dict(), expected)
        json.dumps(expected)

        invalid = [
            {"minimum_progress_degrees": True},
            {"minimum_progress_degrees": 0},
            {"catch_up_leader_minimum_degrees": 2},
            {"acceptable_completion_percent": 0},
            {"acceptable_completion_percent": 101},
            {"maximum_progress_skew_percent": 101},
            {"maximum_catch_up_attempts": -1},
            {"maximum_pair_retry_attempts": False},
            {"maximum_total_attempts": 4},
            {"maximum_step_duration_ms": 0},
            {"maximum_total_recovery_duration_ms": 0},
            {"maximum_total_recovery_encoder_degrees": 0},
        ]
        for override in invalid:
            with self.subTest(override=override):
                with self.assertRaises(ValueError):
                    policy(**override)

    def test_budget_is_validated_serializable_and_consumed_by_copy(self):
        original = EncoderRecoveryBudget()
        consumed = original.consume(DECISION_CATCH_UP, 200, 50)

        self.assertEqual(
            original.to_dict(),
            {
                "catch_up_attempts": 0,
                "pair_retry_attempts": 0,
                "total_attempts": 0,
                "duration_ms": 0,
                "encoder_degrees": 0,
            },
        )
        self.assertEqual(consumed.catch_up_attempts, 1)
        self.assertEqual(consumed.duration_ms, 200)
        self.assertEqual(consumed.encoder_degrees, 50)
        json.dumps(consumed.to_dict())

        for value in (True, -1, 1.5, "1"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    EncoderRecoveryBudget(catch_up_attempts=value)
        with self.assertRaises(ValueError):
            original.consume(DECISION_ABORT, 0, 0)

    def test_balanced_completed_motion_needs_no_recovery(self):
        budget = EncoderRecoveryBudget(
            catch_up_attempts=1,
            duration_ms=100,
            encoder_degrees=25,
        )
        result = decide(
            budget=budget,
            observed_left=174,
            observed_right=160,
        )

        self.assertEqual(result["decision"], DECISION_NO_RECOVERY)
        self.assertEqual(result["reason"], REASON_COMMAND_SATISFIED)
        self.assertIsNone(result["instruction"])
        self.assertEqual(
            result["budget_after_if_executed"], budget.to_dict()
        )

    def test_zero_right_motor_produces_bounded_right_catch_up(self):
        result = decide()

        self.assertEqual(result["decision"], DECISION_CATCH_UP)
        self.assertEqual(result["reason"], REASON_RIGHT_LAGGING)
        self.assertEqual(
            result["instruction"],
            {
                "mode": "single_wheel",
                "side": "right",
                "reuse_original_speed": True,
                "duration_ms": 696,
                "target_abs_encoder_degrees": {
                    "left": 0,
                    "right": 174,
                },
                "bounded": False,
            },
        )
        self.assertEqual(
            result["budget_after_if_executed"],
            {
                "catch_up_attempts": 1,
                "pair_retry_attempts": 0,
                "total_attempts": 1,
                "duration_ms": 696,
                "encoder_degrees": 174,
            },
        )

    def test_zero_left_motor_produces_left_catch_up(self):
        result = decide(observed_left=0, observed_right=174)

        self.assertEqual(result["decision"], DECISION_CATCH_UP)
        self.assertEqual(result["reason"], REASON_LEFT_LAGGING)
        self.assertEqual(result["instruction"]["side"], "left")
        self.assertEqual(result["instruction"]["duration_ms"], 696)

    def test_progress_ratio_handles_different_expected_side_travel(self):
        result = decide(
            expected_left=-100,
            expected_right=200,
            observed_left=-90,
            observed_right=100,
        )

        self.assertEqual(result["decision"], DECISION_CATCH_UP)
        self.assertEqual(result["reason"], REASON_RIGHT_LAGGING)
        self.assertEqual(
            result["completion_percent"], {"left": 90, "right": 50}
        )
        self.assertEqual(
            result["instruction"]["target_abs_encoder_degrees"]["right"],
            80,
        )

    def test_both_static_or_tiny_leader_retries_pair(self):
        for left, right in ((0, 0), (8, 0), (0, 8)):
            with self.subTest(left=left, right=right):
                result = decide(
                    observed_left=left,
                    observed_right=right,
                )
                self.assertEqual(result["decision"], DECISION_RETRY_PAIR)
                self.assertEqual(result["reason"], REASON_PAIRED_UNDERTRAVEL)
                self.assertEqual(result["instruction"]["mode"], "paired")

        static = decide(observed_left=0, observed_right=0)
        self.assertEqual(static["instruction"]["duration_ms"], 600)
        self.assertEqual(
            static["instruction"]["target_abs_encoder_degrees"],
            {"left": 150, "right": 150},
        )

    def test_catch_up_then_fresh_cumulative_evidence_completes(self):
        first = decide()
        used = EncoderRecoveryBudget(
            catch_up_attempts=1,
            duration_ms=696,
            encoder_degrees=174,
        )
        second = decide(
            budget=used,
            observed_left=174,
            observed_right=174,
        )

        self.assertEqual(first["decision"], DECISION_CATCH_UP)
        self.assertEqual(second["decision"], DECISION_NO_RECOVERY)
        self.assertEqual(
            second["budget_after_if_executed"], used.to_dict()
        )

    def test_step_is_shortened_by_encoder_budget(self):
        active_policy = policy(
            maximum_total_recovery_encoder_degrees=100
        )
        result = decide(active_policy=active_policy)

        self.assertEqual(result["decision"], DECISION_CATCH_UP)
        self.assertEqual(result["instruction"]["duration_ms"], 400)
        self.assertEqual(
            result["instruction"]["target_abs_encoder_degrees"],
            {"left": 0, "right": 100},
        )
        self.assertTrue(result["instruction"]["bounded"])

    def test_tiny_catch_up_is_raised_to_encoder_verifiable_duration(self):
        result = decide(
            observed_left=160,
            observed_right=159,
            active_policy=policy(maximum_progress_skew_percent=0),
        )

        self.assertEqual(result["decision"], DECISION_CATCH_UP)
        self.assertEqual(result["instruction"]["side"], "right")
        self.assertEqual(result["instruction"]["duration_ms"], 12)
        self.assertEqual(
            result["instruction"]["target_abs_encoder_degrees"],
            {"left": 0, "right": 3},
        )
        self.assertTrue(result["instruction"]["bounded"])

    def test_encoder_budget_below_verifiable_step_aborts(self):
        result = decide(
            observed_left=160,
            observed_right=159,
            active_policy=policy(
                maximum_progress_skew_percent=0,
                maximum_total_recovery_encoder_degrees=2,
            ),
        )

        self.assertEqual(result["decision"], DECISION_ABORT)
        self.assertEqual(result["reason"], REASON_ENCODER_BUDGET)

    def test_wrong_direction_aborts_without_consuming_budget(self):
        budget = EncoderRecoveryBudget()
        result = decide(budget=budget, observed_left=-2)

        self.assertEqual(result["decision"], DECISION_ABORT)
        self.assertEqual(
            result["reason"], REASON_ENCODER_DIRECTION_MISMATCH
        )
        self.assertIsNone(result["instruction"])
        self.assertEqual(
            result["budget_after_if_executed"], budget.to_dict()
        )

    def test_each_attempt_and_total_attempt_budget_is_hard_bounded(self):
        catch_exhausted = decide(
            budget=EncoderRecoveryBudget(catch_up_attempts=2)
        )
        self.assertEqual(catch_exhausted["decision"], DECISION_ABORT)
        self.assertEqual(
            catch_exhausted["reason"], REASON_CATCH_UP_ATTEMPT_BUDGET
        )

        pair_exhausted = decide(
            budget=EncoderRecoveryBudget(pair_retry_attempts=1),
            observed_left=0,
            observed_right=0,
        )
        self.assertEqual(pair_exhausted["decision"], DECISION_ABORT)
        self.assertEqual(
            pair_exhausted["reason"],
            REASON_PAIR_RETRY_ATTEMPT_BUDGET,
        )

        total_exhausted = decide(
            budget=EncoderRecoveryBudget(
                catch_up_attempts=2,
                pair_retry_attempts=1,
            )
        )
        self.assertEqual(total_exhausted["decision"], DECISION_ABORT)
        self.assertEqual(
            total_exhausted["reason"], REASON_TOTAL_ATTEMPT_BUDGET
        )

    def test_duration_and_encoder_budgets_abort_when_empty(self):
        duration = decide(
            budget=EncoderRecoveryBudget(duration_ms=1600)
        )
        self.assertEqual(duration["decision"], DECISION_ABORT)
        self.assertEqual(duration["reason"], REASON_DURATION_BUDGET)

        encoder = decide(
            budget=EncoderRecoveryBudget(encoder_degrees=400)
        )
        self.assertEqual(encoder["decision"], DECISION_ABORT)
        self.assertEqual(encoder["reason"], REASON_ENCODER_BUDGET)

    def test_decision_is_repeatable_json_safe_and_does_not_mutate_budget(self):
        budget = EncoderRecoveryBudget()
        first = decide(budget=budget)
        second = decide(budget=budget)

        self.assertEqual(first, second)
        self.assertEqual(budget.to_dict()["total_attempts"], 0)
        json.dumps(first)

    def test_invalid_command_evidence_is_rejected(self):
        invalid_calls = [
            {"expected_left": 0},
            {"expected_right": 2},
            {"expected_left": True},
            {"observed_left": True},
            {"observed_right": 1.5},
            {"duration_ms": 0},
            {"duration_ms": False},
        ]
        for arguments in invalid_calls:
            with self.subTest(arguments=arguments):
                with self.assertRaises(ValueError):
                    decide(**arguments)

        with self.assertRaises(ValueError):
            policy().decide(200, 200, 100, 100, 800, {})

    def test_module_remains_python35_parseable(self):
        ev3_root = Path(__file__).resolve().parents[1] / "ev3"
        for filename in (
            "encoder_recovery.py",
            "encoder_recovery_runtime.py",
        ):
            with self.subTest(filename=filename):
                path = ev3_root / filename
                ast.parse(
                    path.read_text(encoding="utf-8"),
                    filename=str(path),
                    feature_version=5,
                )


if __name__ == "__main__":
    unittest.main()
