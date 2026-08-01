import json
from pathlib import Path
import tempfile
import unittest
import uuid

from robot_agent.navigation_memory_store import NavigationMemoryStore
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    OBSERVE,
    SCAN_FRONT_ARC,
)
from robot_agent.physical_navigation_experience import (
    FIRST_ATTEMPT,
    MAX_EXPERIENCE_CONTEXT_BYTES,
    MAX_EXPERIENCE_ENTRIES,
    NavigationExperienceLedger,
    PLANNER_ACTION_SOURCE,
    RETRY_AFTER_BASIS_CHANGE,
    UNCHANGED_BASIS_REPEAT,
    navigation_evidence_basis,
)
from robot_agent.physical_navigation_runtime import (
    PhysicalNavigationRuntime,
    PhysicalNavigationRuntimeConfig,
)
from tests.test_physical_navigation_core import (
    FakeRuntimePlanner,
    FakeRuntimeTransport,
)


def observation(
    version,
    *,
    blocked=False,
    raw=None,
    left_position=0,
    right_position=0,
):
    reading = (20 if blocked else 60) if raw is None else raw
    return {
        "state_version": version,
        "observed_monotonic_ms": version * 10,
        "touch": {"value0": 0, "pressed": False},
        "infrared": {
            "raw": reading,
            "filtered": reading,
            "blocked": blocked,
            "reason": "typed_sensor_reason",
            "sample_count": 5,
        },
        "motors": [
            {
                "role": "left_drive",
                "position": left_position,
                "state": "",
            },
            {
                "role": "right_drive",
                "position": right_position,
                "state": "",
            },
        ],
        "last_outcome": {"kind": "observe", "status": "completed"},
        "budgets": {
            "pulse_count": 0,
            "pulse_count_remaining": 40,
            "pulse_duration_ms": 0,
            "pulse_duration_ms_remaining": 32_000,
            "process_ms_remaining": 40_000,
            "motion_fault_latched": False,
        },
    }


def navigation(*, x_mm=0, map_version=1, scan_history=None):
    hazards = []
    if scan_history is not None:
        hazards.append({
            "hypothesis_id": "hazard-1",
            "centroid_x_mm": 300,
            "centroid_y_mm": 0,
            "radius_mm": 120,
            "scan_left_boundary_mdeg": None,
            "scan_right_boundary_mdeg": None,
            "scan_evidence_history": list(scan_history),
        })
    return {
        "map_generation_id": "mapgen-1",
        "map_version": map_version,
        "localization_valid": True,
        "pose": {
            "x_mm": x_mm,
            "y_mm": 0,
            "heading_mdeg": 0,
            "verified_motion_count": 0,
            "total_forward_mm": x_mm,
            "total_turn_mdeg": 0,
        },
        "navigation_hazard_hypotheses": hazards,
    }


def pulse_result(*, left_delta=0, right_delta=0, status="completed"):
    return {
        "operation": "pulse",
        "status": status,
        "reason": "completed_all_slices",
        "encoder_observation": {
            "action": ADVANCE,
            "left_encoder_delta_degrees": left_delta,
            "right_encoder_delta_degrees": right_delta,
            "verified_slice_count": 1,
            "observed_slice_count": 1,
            "requested_slice_count": 1,
            "command_completed": status == "completed",
        },
        "utterance": "this arbitrary text must not enter the ledger",
    }


def scan_attempt(scan_id):
    return {
        "scan_id": scan_id,
        "status": "COMPLETED",
        "observation_pattern": "MIXED",
        "arc_coverage": "BILATERAL_ARC",
        "boundary_coverage": "POSITIVE_BOUNDARY_ONLY",
        "hypothesis_relation": "SUPPORTS_BLOCKED_HYPOTHESIS",
    }


class NavigationEvidenceBasisTests(unittest.TestCase):
    def test_non_drive_motor_motion_does_not_fake_navigation_progress(self):
        navigation_value = navigation()
        navigation_value["drive_motor_roles"] = {
            "left": "left_drive",
            "right": "right_drive",
        }
        before_observation = observation(1)
        before_observation["motors"].append({
            "role": "arm",
            "position": 0,
            "state": "",
        })
        after_observation = observation(2)
        after_observation["motors"].append({
            "role": "arm",
            "position": 720,
            "state": "",
        })

        before = navigation_evidence_basis(
            navigation_value,
            before_observation,
        )
        after = navigation_evidence_basis(
            navigation_value,
            after_observation,
        )

        self.assertEqual(before, after)

    def test_freshness_and_ir_jitter_do_not_create_a_new_basis(self):
        first = navigation_evidence_basis(
            navigation(map_version=1),
            observation(1, raw=60),
        )
        second = navigation_evidence_basis(
            navigation(map_version=99),
            observation(88, raw=63),
        )

        self.assertEqual(first, second)

    def test_scan_evidence_is_a_typed_basis_change(self):
        before = navigation_evidence_basis(
            navigation(scan_history=[]),
            observation(1, blocked=True),
        )
        after = navigation_evidence_basis(
            navigation(scan_history=[scan_attempt("scan-1")]),
            observation(2, blocked=True),
        )
        ledger = NavigationExperienceLedger(episode_id="episode-1")

        entry = ledger.record(
            turn=1,
            action=SCAN_FRONT_ARC,
            source=PLANNER_ACTION_SOURCE,
            result={
                "operation": SCAN_FRONT_ARC,
                "status": "COMPLETED",
                "reason": "restored",
                "evidence_disposition": "MAP_INTEGRATED",
                "target_hypothesis_id": "hazard-1",
                "bilateral_complete": False,
                "scan_evidence": scan_attempt("scan-1"),
            },
            basis_before=before,
            basis_after=after,
        )

        self.assertIn("SCAN_EVIDENCE_CHANGED", entry.basis_change_codes)
        self.assertEqual(
            entry.outcome["scan_evidence"]["hypothesis_relation"],
            "SUPPORTS_BLOCKED_HYPOTHESIS",
        )

    def test_duplicate_scan_with_a_fresh_id_is_not_new_information(self):
        first = navigation_evidence_basis(
            navigation(scan_history=[scan_attempt("scan-1")]),
            observation(1, blocked=True),
        )
        duplicate = navigation_evidence_basis(
            navigation(scan_history=[
                scan_attempt("scan-1"),
                scan_attempt("scan-2"),
            ]),
            observation(2, blocked=True),
        )

        self.assertEqual(first, duplicate)


class NavigationExperienceLedgerTests(unittest.TestCase):
    def test_scan_repeat_identity_is_scoped_to_its_typed_target(self):
        basis = navigation_evidence_basis(navigation(), observation(1))
        ledger = NavigationExperienceLedger(episode_id="episode-targets")

        def record(turn, target):
            return ledger.record(
                turn=turn,
                action=SCAN_FRONT_ARC,
                source=PLANNER_ACTION_SOURCE,
                result={
                    "operation": SCAN_FRONT_ARC,
                    "status": "DENIED",
                    "reason": "INTERVENING_NAVIGATION_PROGRESS_REQUIRED",
                    "target_hypothesis_id": target,
                },
                basis_before=basis,
                basis_after=basis,
            )

        first_a = record(1, "hazard-a")
        first_b = record(2, "hazard-b")
        repeated_a = record(3, "hazard-a")

        self.assertEqual(first_a.attempt_relation, FIRST_ATTEMPT)
        self.assertEqual(first_b.attempt_relation, FIRST_ATTEMPT)
        self.assertEqual(first_b.prior_same_action_sequence, 1)
        self.assertEqual(
            first_b.attempt_identity,
            {
                "action": SCAN_FRONT_ARC,
                "target_hypothesis_id": "hazard-b",
            },
        )
        self.assertEqual(
            repeated_a.attempt_relation,
            UNCHANGED_BASIS_REPEAT,
        )
        self.assertEqual(repeated_a.prior_same_basis_sequence, 1)
        rollup = next(
            item
            for item in ledger.context(current_basis=basis)[
                "current_basis_action_rollups"
            ]
            if item["action"] == SCAN_FRONT_ARC
        )
        self.assertEqual(
            {
                item["outcome"]["target_hypothesis_id"]
                for item in rollup["outcome_distribution"]
            },
            {"hazard-a", "hazard-b"},
        )

    def test_rollup_distribution_is_bounded_with_explicit_omission_counts(self):
        basis = navigation_evidence_basis(navigation(), observation(1))
        ledger = NavigationExperienceLedger(episode_id="episode-many-outcomes")
        for turn in range(1, 501):
            ledger.record(
                turn=turn,
                action=OBSERVE,
                source=PLANNER_ACTION_SOURCE,
                result={
                    "operation": "observe",
                    "status": "observed",
                    "reason": "OUTCOME_{:04d}_{}".format(
                        turn,
                        "x" * 120,
                    ),
                    "information_gain": "NO_DECISION_RELEVANT_CHANGE",
                },
                basis_before=basis,
                basis_after=basis,
            )

        context = ledger.context(current_basis=basis)
        rollup = next(
            item
            for item in context["current_basis_action_rollups"]
            if item["action"] == OBSERVE
        )

        self.assertLessEqual(
            len(json.dumps(
                context,
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            ).encode("utf-8")),
            MAX_EXPERIENCE_CONTEXT_BYTES,
        )
        self.assertEqual(rollup["attempt_count"], 500)
        self.assertEqual(rollup["outcome_bucket_count"], 500)
        self.assertGreater(rollup["outcome_bucket_omitted_count"], 0)
        self.assertEqual(
            rollup["outcome_bucket_retained_count"]
            + rollup["outcome_bucket_omitted_count"],
            500,
        )
        self.assertEqual(
            rollup["outcome_attempt_retained_count"]
            + rollup["outcome_attempt_omitted_count"],
            500,
        )
        self.assertTrue(
            rollup["latest_outcome"]["reason_code"].startswith(
                "OUTCOME_0500_"
            )
        )

    def test_exact_repeat_is_distinct_from_retry_after_pose_change(self):
        at_origin = navigation_evidence_basis(
            navigation(x_mm=0),
            observation(1),
        )
        farther_forward = navigation_evidence_basis(
            navigation(x_mm=100),
            observation(2, left_position=174, right_position=174),
        )
        ledger = NavigationExperienceLedger(episode_id="episode-1")

        first = ledger.record(
            turn=1,
            action=ADVANCE,
            source=PLANNER_ACTION_SOURCE,
            result=pulse_result(),
            basis_before=at_origin,
            basis_after=at_origin,
        )
        unchanged = ledger.record(
            turn=2,
            action=ADVANCE,
            source=PLANNER_ACTION_SOURCE,
            result=pulse_result(status="verification_failed"),
            basis_before=at_origin,
            basis_after=at_origin,
        )
        informed = ledger.record(
            turn=3,
            action=ADVANCE,
            source=PLANNER_ACTION_SOURCE,
            result=pulse_result(left_delta=174, right_delta=174),
            basis_before=farther_forward,
            basis_after=farther_forward,
        )

        self.assertEqual(first.attempt_relation, FIRST_ATTEMPT)
        self.assertEqual(
            unchanged.attempt_relation,
            UNCHANGED_BASIS_REPEAT,
        )
        self.assertEqual(
            informed.attempt_relation,
            RETRY_AFTER_BASIS_CHANGE,
        )
        self.assertEqual(unchanged.prior_same_action_sequence, 1)
        self.assertEqual(informed.prior_same_action_sequence, 2)

    def test_seen_basis_survives_detailed_history_eviction(self):
        ledger = NavigationExperienceLedger(episode_id="episode-1")
        origin = navigation_evidence_basis(
            navigation(x_mm=0),
            observation(1),
        )
        ledger.record(
            turn=1,
            action=ADVANCE,
            source=PLANNER_ACTION_SOURCE,
            result=pulse_result(),
            basis_before=origin,
            basis_after=origin,
        )
        for turn in range(2, MAX_EXPERIENCE_ENTRIES + 5):
            changed = navigation_evidence_basis(
                navigation(x_mm=turn),
                observation(
                    turn,
                    left_position=turn,
                    right_position=turn,
                ),
            )
            ledger.record(
                turn=turn,
                action=ADVANCE,
                source=PLANNER_ACTION_SOURCE,
                result=pulse_result(),
                basis_before=changed,
                basis_after=changed,
            )
        self.assertNotIn(1, [item.sequence for item in ledger.entries])

        repeated = ledger.record(
            turn=MAX_EXPERIENCE_ENTRIES + 5,
            action=ADVANCE,
            source=PLANNER_ACTION_SOURCE,
            result=pulse_result(),
            basis_before=origin,
            basis_after=origin,
        )

        self.assertEqual(
            repeated.attempt_relation,
            UNCHANGED_BASIS_REPEAT,
        )
        self.assertEqual(repeated.prior_same_basis_sequence, 1)

    def test_current_basis_rollup_survives_detailed_history_eviction(self):
        ledger = NavigationExperienceLedger(episode_id="episode-rollup")
        origin = navigation_evidence_basis(
            navigation(x_mm=0),
            observation(1),
        )
        ledger.record(
            turn=1,
            action=ADVANCE,
            source=PLANNER_ACTION_SOURCE,
            result=pulse_result(status="verification_failed"),
            basis_before=origin,
            basis_after=origin,
        )
        for turn in range(2, MAX_EXPERIENCE_ENTRIES + 5):
            changed = navigation_evidence_basis(
                navigation(x_mm=turn),
                observation(
                    turn,
                    left_position=turn,
                    right_position=turn,
                ),
            )
            ledger.record(
                turn=turn,
                action=OBSERVE,
                source=PLANNER_ACTION_SOURCE,
                result={
                    "operation": "observe",
                    "status": "observed",
                    "information_gain": "DECISION_RELEVANT_CHANGE",
                },
                basis_before=changed,
                basis_after=changed,
            )

        context = ledger.context(current_basis=origin)
        by_action = {
            item["action"]: item
            for item in context["current_basis_action_rollups"]
        }

        self.assertNotIn(1, [item["sequence"] for item in context["entries"]])
        self.assertEqual(by_action[ADVANCE]["attempt_count"], 1)
        self.assertEqual(by_action[ADVANCE]["first_sequence"], 1)
        self.assertEqual(by_action[ADVANCE]["latest_sequence"], 1)
        self.assertEqual(
            by_action[ADVANCE]["latest_outcome"]["status"],
            "verification_failed",
        )
        self.assertEqual(
            by_action[ADVANCE]["outcome_distribution"],
            [{
                "outcome": {
                    "operation": "pulse",
                    "status": "verification_failed",
                    "reason_code": "completed_all_slices",
                },
                "count": 1,
            }],
        )

    def test_current_basis_rollup_counts_typed_outcomes(self):
        ledger = NavigationExperienceLedger(episode_id="episode-rollup")
        basis = navigation_evidence_basis(navigation(), observation(1))
        for turn, status in enumerate(
            ("verification_failed", "verification_failed", "completed"),
            start=1,
        ):
            ledger.record(
                turn=turn,
                action=ADVANCE,
                source=PLANNER_ACTION_SOURCE,
                result=pulse_result(status=status),
                basis_before=basis,
                basis_after=basis,
            )

        rollup = next(
            item
            for item in ledger.context(current_basis=basis)[
                "current_basis_action_rollups"
            ]
            if item["action"] == ADVANCE
        )

        self.assertEqual(rollup["attempt_count"], 3)
        self.assertEqual(rollup["first_sequence"], 1)
        self.assertEqual(rollup["latest_sequence"], 3)
        self.assertEqual(rollup["latest_outcome"]["status"], "completed")
        self.assertEqual(
            {
                item["outcome"]["status"]: item["count"]
                for item in rollup["outcome_distribution"]
            },
            {"completed": 1, "verification_failed": 2},
        )

    def test_published_outcome_is_structured_and_omits_arbitrary_text(self):
        basis = navigation_evidence_basis(navigation(), observation(1))
        ledger = NavigationExperienceLedger(episode_id="episode-1")
        ledger.record(
            turn=1,
            action=ADVANCE,
            source=PLANNER_ACTION_SOURCE,
            result=pulse_result(left_delta=170, right_delta=168),
            basis_before=basis,
            basis_after=basis,
        )

        context = ledger.context(current_basis=basis)
        outcome = context["entries"][0]["outcome"]
        self.assertNotIn("utterance", outcome)
        self.assertEqual(outcome["operation"], "pulse")
        self.assertEqual(
            outcome["encoder_observation"][
                "left_encoder_delta_degrees"
            ],
            170,
        )
        self.assertEqual(
            context["entries"][0]["basis_before"]["pose"]["x_mm"],
            0,
        )
        self.assertIn(
            "infrared_blocked",
            context["entries"][0]["basis_before"][
                "observation_facts"
            ],
        )
        self.assertFalse(context["host_ranked_or_selected_action"])
        self.assertEqual(context["scope"], "EPISODE")
        self.assertFalse(context["persisted"])

    def test_history_and_serialized_context_are_bounded(self):
        ledger = NavigationExperienceLedger(episode_id="episode-1")
        for sequence in range(1, 33):
            before = navigation_evidence_basis(
                navigation(x_mm=sequence),
                observation(
                    sequence,
                    left_position=sequence,
                    right_position=sequence,
                ),
            )
            ledger.record(
                turn=sequence,
                action=ADVANCE,
                source=PLANNER_ACTION_SOURCE,
                result=pulse_result(
                    left_delta=sequence,
                    right_delta=sequence,
                ),
                basis_before=before,
                basis_after=before,
            )

        context = ledger.context(current_basis=before)
        encoded = json.dumps(
            context,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
        self.assertLessEqual(
            len(context["entries"]),
            MAX_EXPERIENCE_ENTRIES,
        )
        self.assertGreater(len(context["entries"]), 0)
        self.assertEqual(context["total_recorded_count"], 32)
        self.assertLessEqual(len(encoded), MAX_EXPERIENCE_CONTEXT_BYTES)
        self.assertEqual(context["entries"][-1]["sequence"], 32)
        self.assertEqual(
            context["entries"][0]["sequence"],
            33 - len(context["entries"]),
        )

    def test_byte_limit_prunes_oldest_oversized_outcomes(self):
        ledger = NavigationExperienceLedger(episode_id="episode-1")
        basis = navigation_evidence_basis(navigation(), observation(1))
        result = {
            "operation": "observe",
            "status": "observed",
            "information_gain": "DECISION_RELEVANT_CHANGE",
            "changed_facts": [
                "fact-{}-{}".format(index, "x" * 64)
                for index in range(16)
            ],
        }
        for turn in range(1, MAX_EXPERIENCE_ENTRIES + 1):
            ledger.record(
                turn=turn,
                action=OBSERVE,
                source=PLANNER_ACTION_SOURCE,
                result=result,
                basis_before=basis,
                basis_after=basis,
            )

        context = ledger.context(current_basis=basis)
        encoded = json.dumps(
            context,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
        self.assertLess(context["retained_count"], MAX_EXPERIENCE_ENTRIES)
        self.assertLessEqual(len(encoded), MAX_EXPERIENCE_CONTEXT_BYTES)


class NavigationExperienceRuntimeTests(unittest.TestCase):
    def test_runtime_publishes_planner_and_plan_tail_results(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=501),
            )
            times = iter((2_001, 2_002, 2_003))
            runtime = PhysicalNavigationRuntime(
                episode_id="experience-runtime",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Move forward",
                    locale="en",
                    minimum_forward_progress_mm=100,
                    max_turns=3,
                    max_episode_seconds=10,
                ),
                transport=FakeRuntimeTransport(),
                planner=FakeRuntimePlanner(),
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: next(times),
            )

            result = runtime.run()

        ledger = result.final_navigation["experience_ledger"]
        self.assertEqual(ledger["total_recorded_count"], 2)
        self.assertEqual(
            [item["source"] for item in ledger["entries"]],
            ["PLANNER_ACTION", "PLAN_TAIL_ACTION"],
        )
        self.assertEqual(
            [item["action"] for item in ledger["entries"]],
            [ADVANCE, ADVANCE],
        )
        self.assertTrue(all(
            "VERIFIED_POSE_CHANGED" in item["basis_change_codes"]
            for item in ledger["entries"]
        ))
        self.assertFalse(ledger["host_ranked_or_selected_action"])


if __name__ == "__main__":
    unittest.main()
