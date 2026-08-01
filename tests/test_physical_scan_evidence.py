import json
import unittest

from robot_agent.navigation_memory_store import MAX_MEMORY_BYTES
from robot_agent.lm_studio_navigation import LMStudioNavigationPlanner
from robot_agent.maneuver_commitment import empty_commitment
from robot_agent.physical_navigation_contract import DECISION_SCHEMA, OBSERVE
from robot_agent.physical_scan_evidence import (
    BODY_RELATIVE_BEARING_CONVENTION,
    MAX_SCAN_ATTEMPTS_PER_HAZARD,
    MAX_SCAN_ATTEMPTS_PER_MAP,
    ScanAttemptEvidence,
    ScanRayEvidence,
    retain_scan_attempt_diversity,
)
from robot_agent.physical_odometry import PhysicalPose
from robot_agent.provisional_hazard_map import (
    ProvisionalHazard,
    ProvisionalHazardMap,
)


def attempt(index):
    rays = tuple(
        ScanRayEvidence(
            requested_relative_bearing_mdeg=(ordinal - 8) * 1_000,
            actual_relative_bearing_mdeg=(ordinal - 8) * 1_000,
            blocked=ordinal % 2 == 0,
            raw=50,
            filtered=50,
        )
        for ordinal in range(16)
    )
    return ScanAttemptEvidence(
        scan_id="scan-{:03d}".format(index),
        completed_at_ms=2_000 + index,
        status="CANCELLED",
        reason="bilateral_boundaries_not_observed",
        rays=rays,
        left_boundary_mdeg=None,
        right_boundary_mdeg=None,
    )


def hazard(index, history=()):
    return ProvisionalHazard(
        hypothesis_id="hazard-{:02d}".format(index),
        frame_id="frame-a",
        anchor_x_mm=0,
        anchor_y_mm=0,
        anchor_heading_mdeg=0,
        centroid_x_mm=140 + index,
        centroid_y_mm=0,
        radius_mm=70,
        first_seen_at_ms=1_000,
        last_seen_at_ms=1_000,
        evidence_count=1,
        last_state_version=index + 1,
        last_raw_ir_proximity=20,
        last_filtered_ir_proximity=20,
        scan_evidence_history=tuple(history),
    )


class PhysicalScanEvidenceTests(unittest.TestCase):
    @staticmethod
    def _one_sided_attempt(scan_id, *, positive, completed_at_ms):
        if positive:
            bearings = ((10_000, True), (30_000, False))
            left_boundary = 20_000
            right_boundary = None
        else:
            bearings = ((-30_000, False), (-10_000, True))
            left_boundary = None
            right_boundary = -20_000
        return ScanAttemptEvidence(
            scan_id=scan_id,
            completed_at_ms=completed_at_ms,
            status="CANCELLED",
            reason="bilateral_boundaries_not_observed",
            rays=tuple(
                ScanRayEvidence(
                    requested_relative_bearing_mdeg=bearing,
                    actual_relative_bearing_mdeg=bearing,
                    blocked=blocked,
                    raw=20 if blocked else 60,
                    filtered=20 if blocked else 60,
                )
                for bearing, blocked in bearings
            ),
            left_boundary_mdeg=left_boundary,
            right_boundary_mdeg=right_boundary,
            scan_pose=PhysicalPose(),
            based_on_map_version=1,
        )

    def test_complementary_attempts_unlock_only_their_verified_pose(self):
        positive = self._one_sided_attempt(
            "scan-positive",
            positive=True,
            completed_at_ms=2_000,
        )
        negative = self._one_sided_attempt(
            "scan-negative",
            positive=False,
            completed_at_ms=2_100,
        )
        hazard_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="map-a",
            hazards=(hazard(0, (positive, negative)),),
        )

        ready = hazard_map.route_evidence(
            "hazard-00",
            pose=PhysicalPose(),
        )
        moved = hazard_map.route_evidence(
            "hazard-00",
            pose=PhysicalPose(x_mm=1),
        )

        self.assertTrue(ready["ready"])
        self.assertEqual(
            ready["reason"],
            "COMPLEMENTARY_BOUNDARIES_AT_CURRENT_POSE",
        )
        self.assertFalse(moved["ready"])
        self.assertEqual(
            moved["reason"],
            "NO_SCAN_EVIDENCE_AT_CURRENT_VERIFIED_POSE",
        )

    def test_duplicate_attempt_does_not_evict_unique_boundary_shape(self):
        positive = self._one_sided_attempt(
            "scan-positive",
            positive=True,
            completed_at_ms=2_000,
        )
        old_negative = self._one_sided_attempt(
            "scan-negative-old",
            positive=False,
            completed_at_ms=2_100,
        )
        new_negative = self._one_sided_attempt(
            "scan-negative-new",
            positive=False,
            completed_at_ms=2_200,
        )

        retained = retain_scan_attempt_diversity(
            (positive, old_negative, new_negative),
            limit=2,
        )

        self.assertEqual(
            [item.scan_id for item in retained],
            ["scan-positive", "scan-negative-new"],
        )

    def test_partial_all_clear_arc_does_not_erase_blocked_hypothesis(self):
        partial = ScanAttemptEvidence(
            scan_id="scan-partial-clear",
            completed_at_ms=3_000,
            status="CANCELLED",
            reason="scan_deadline_exceeded",
            rays=(
                ScanRayEvidence(0, 0, False, 60, 60),
                ScanRayEvidence(30_000, 29_000, False, 61, 60),
            ),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
        )

        self.assertEqual(partial.observation_pattern, "ALL_CLEAR")
        self.assertEqual(partial.arc_coverage, "POSITIVE_ARC_ONLY")
        self.assertEqual(partial.hypothesis_relation, "NO_EVIDENCE")

    def test_deadline_all_clear_bilateral_sample_is_not_full_conflict(self):
        timed_out = ScanAttemptEvidence(
            scan_id="scan-deadline-clear",
            completed_at_ms=3_000,
            status="CANCELLED",
            reason="scan_deadline_exceeded",
            rays=(
                ScanRayEvidence(-30_000, -30_000, False, 60, 60),
                ScanRayEvidence(0, 0, False, 60, 60),
                ScanRayEvidence(30_000, 30_000, False, 60, 60),
            ),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
        )

        self.assertEqual(timed_out.arc_coverage, "BILATERAL_ARC")
        self.assertEqual(timed_out.hypothesis_relation, "NO_EVIDENCE")

    def test_scan_evidence_publishes_body_relative_bearing_convention(self):
        value = attempt(1).to_dict()

        self.assertEqual(
            value["bearing_convention"],
            BODY_RELATIVE_BEARING_CONVENTION,
        )
        self.assertEqual(ScanAttemptEvidence.from_dict(value), attempt(1))

    def test_legacy_persisted_hazard_loads_without_scan_history(self):
        value = dict(hazard(0).to_dict())
        value.pop("scan_evidence_history")
        value.pop("bilateral_scan_complete")

        loaded = ProvisionalHazard.from_dict(value)

        self.assertEqual(loaded.scan_evidence_history, ())
        self.assertFalse(loaded.bilateral_scan_complete)

    def test_attempt_history_is_bounded_per_hazard_and_per_map(self):
        with self.assertRaisesRegex(ValueError, "scan history"):
            hazard(
                0,
                [
                    attempt(index)
                    for index in range(MAX_SCAN_ATTEMPTS_PER_HAZARD + 1)
                ],
            )
        histories = []
        attempt_index = 0
        for hazard_index in range(5):
            count = 2 if hazard_index < 4 else 1
            histories.append(tuple(
                attempt(attempt_index + offset)
                for offset in range(count)
            ))
            attempt_index += count
        with self.assertRaisesRegex(ValueError, "map bound"):
            ProvisionalHazardMap(
                frame_id="frame-a",
                map_generation_id="map-a",
                hazards=tuple(
                    hazard(index, history)
                    for index, history in enumerate(histories)
                ),
            )
        self.assertEqual(MAX_SCAN_ATTEMPTS_PER_MAP, 8)

    def test_worst_legal_history_stays_below_context_and_memory_limits(self):
        hazards = tuple(
            hazard(
                index,
                (
                    (attempt(index * 2), attempt(index * 2 + 1))
                    if index < 4
                    else ()
                ),
            )
            for index in range(32)
        )
        hazard_map = ProvisionalHazardMap(
            frame_id="frame-a",
            map_generation_id="map-a",
            hazards=hazards,
        )
        context_bytes = json.dumps(
            hazard_map.context(),
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        memory_bytes = json.dumps(
            {"hazards": [item.to_dict() for item in hazards]},
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")

        # Leave material headroom for mission, observation and maneuver
        # context inside the planner's 64-KiB envelope.
        self.assertLess(len(context_bytes), 48 * 1024)
        self.assertLess(len(memory_bytes), MAX_MEMORY_BYTES)

        def transport(_url, _body, _headers, _timeout, _maximum):
            decision = {
                "schema": DECISION_SCHEMA,
                "episode_id": "episode-max-scan-history",
                "turn": 1,
                "based_on_state_version": 1,
                "action": OBSERVE,
                "plan": [OBSERVE],
                "reason_code": "VERIFY_RESULT",
                "assessment": "Use the accumulated scan evidence.",
                "utterance": None,
                "perception_target_hypothesis_id": None,
                "maneuver_commitment": empty_commitment(),
            }
            return json.dumps({
                "choices": [{
                    "message": {"content": json.dumps(decision)},
                }],
            }).encode("utf-8")

        planner = LMStudioNavigationPlanner(
            base_url="http://127.0.0.1:1234",
            model="test-model",
            transport=transport,
            clock=lambda: 1.0,
        )
        planned = planner.decide(
            episode_id="episode-max-scan-history",
            turn=1,
            locale="en",
            observation={"state_version": 1},
            mission={
                "completed": False,
                "user_goal": "Inspect accumulated evidence and adapt.",
            },
            navigation=hazard_map.context(),
            maneuver_state={"active": None, "last_terminal": None},
            available_actions=[OBSERVE],
            last_tool_result=None,
        )
        self.assertEqual(planned.decision.action, OBSERVE)


if __name__ == "__main__":
    unittest.main()
