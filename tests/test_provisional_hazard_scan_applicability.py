from dataclasses import replace
import unittest

from robot_agent.active_ir_scan_contract import ActiveIrRay, ActiveIrScanResult
from robot_agent.maneuver_commitment import (
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
    ManeuverCommitment,
)
from robot_agent.physical_navigation_contract import OBSERVE, TURN_RIGHT_90
from robot_agent.physical_odometry import PhysicalPose
from robot_agent.physical_scan_evidence import (
    AngularCollisionSupport,
    ScanAttemptEvidence,
    ScanRayEvidence,
)
from robot_agent.provisional_hazard_map import (
    ProvisionalHazard,
    ProvisionalHazardMap,
)


def hazard_map():
    return ProvisionalHazardMap(
        frame_id="frame-a",
        map_generation_id="generation-a",
        revision=1,
        hazards=(
            ProvisionalHazard(
                hypothesis_id="hazard-a",
                frame_id="frame-a",
                anchor_x_mm=0,
                anchor_y_mm=0,
                anchor_heading_mdeg=0,
                centroid_x_mm=140,
                centroid_y_mm=0,
                radius_mm=70,
                first_seen_at_ms=1_000,
                last_seen_at_ms=1_000,
                evidence_count=1,
                last_state_version=1,
                last_raw_ir_proximity=30,
                last_filtered_ir_proximity=30,
            ),
        ),
    )


def all_clear_result():
    rays = tuple(
        ActiveIrRay(
            ordinal=index,
            requested_relative_bearing_mdeg=bearing,
            actual_relative_bearing_mdeg=bearing,
            observed_at_ms=2_000 + index,
            state_version=2 + index,
            raw=70,
            filtered=70,
            blocked=False,
        )
        for index, bearing in enumerate(
            (-60_000, -30_000, 0, 30_000, 60_000), start=1
        )
    )
    return ActiveIrScanResult(
        scan_id="scan-all-clear",
        target_hypothesis_id="hazard-a",
        frame_id="frame-a",
        map_generation_id="generation-a",
        based_on_map_version=0,
        started_at_ms=2_000,
        completed_at_ms=2_100,
        status="CANCELLED",
        reason="bilateral_boundaries_not_observed",
        stop_confirmed=True,
        restored_start_heading=True,
        rays=rays,
        left_boundary_mdeg=None,
        right_boundary_mdeg=None,
    )


class AllClearScanApplicabilityTests(unittest.TestCase):
    def test_unknown_legacy_clear_applicability_is_not_destructive(self):
        attempt = ScanAttemptEvidence.from_scan_result(
            all_clear_result(),
            scan_pose=PhysicalPose(),
        )

        self.assertIsNone(
            attempt.all_clear_arc_covers_target_hypothesis
        )
        self.assertEqual(attempt.hypothesis_relation, "NO_EVIDENCE")

    def test_live_geometry_target_ninety_degrees_outside_arc_stays_active(self):
        mapped = hazard_map()

        recorded = mapped.record_scan_result(
            all_clear_result(),
            scan_pose=PhysicalPose(heading_mdeg=90_000),
        )

        attempt = recorded.scan_evidence_history[-1]
        self.assertFalse(
            attempt.all_clear_arc_covers_target_hypothesis
        )
        self.assertEqual(attempt.hypothesis_relation, "NO_EVIDENCE")
        self.assertTrue(recorded.active_for_collision)
        self.assertIsNone(recorded.collision_contested_at_ms)
        geometry = mapped.goal_geometry(
            pose=PhysicalPose(heading_mdeg=90_000),
            goal_heading_mdeg=0,
        )
        self.assertEqual(
            [item["hypothesis_id"] for item in geometry["conflicts"]],
            ["hazard-a"],
        )

    def test_all_clear_arc_covering_whole_hazard_can_contest_it(self):
        mapped = hazard_map()

        recorded = mapped.record_scan_result(
            all_clear_result(),
            scan_pose=PhysicalPose(),
        )

        attempt = recorded.scan_evidence_history[-1]
        self.assertTrue(attempt.all_clear_arc_covers_target_hypothesis)
        self.assertEqual(
            attempt.hypothesis_relation,
            "CONFLICTS_BLOCKED_HYPOTHESIS",
        )
        self.assertFalse(recorded.active_for_collision)
        self.assertEqual(recorded.collision_contested_at_ms, 2_100)

    def test_backing_beyond_blocked_range_does_not_erase_hazard(self):
        mapped = hazard_map()
        pose = PhysicalPose(x_mm=-236)

        recorded = mapped.record_scan_result(
            all_clear_result(),
            scan_pose=pose,
        )

        attempt = recorded.scan_evidence_history[-1]
        self.assertFalse(
            attempt.all_clear_arc_covers_target_hypothesis
        )
        self.assertEqual(attempt.hypothesis_relation, "NO_EVIDENCE")
        self.assertTrue(recorded.active_for_collision)
        self.assertIsNone(recorded.collision_contested_at_ms)
        evidence = mapped.route_evidence("hazard-a", pose=pose)
        self.assertFalse(evidence["ready"])
        self.assertTrue(evidence["best_effort_ready"])
        self.assertEqual(evidence["strength"], "ALL_CLEAR_ARC")
        self.assertEqual(
            evidence["reason"],
            "ALL_CLEAR_ARC_AT_CURRENT_VERIFIED_POSE",
        )

        state = ManeuverCommitment().apply(
            {
                "id": "route-all-clear",
                "revision": 1,
                "transition": "START",
                "objective": "Pass the remembered obstacle",
                "target_hypothesis_id": "hazard-a",
                "detour_side": "RIGHT_OF_GOAL",
                "success_fact_keys": [
                    FACT_GOAL_CORRIDOR_CLEAR,
                    FACT_GOAL_HEADING_ALIGNED,
                    FACT_TARGET_BEHIND,
                ],
                "current_focus_fact_key": FACT_GOAL_CORRIDOR_CLEAR,
                "revision_reason": None,
            },
            action=OBSERVE,
            turn=1,
            hazard_map=mapped,
            pose=pose,
            fact_values={},
        )
        self.assertEqual(
            state["active"]["target_hypothesis_id"],
            "hazard-a",
        )

    def test_explicit_scan_applicability_round_trips(self):
        mapped = hazard_map()
        recorded = mapped.record_scan_result(
            all_clear_result(),
            scan_pose=PhysicalPose(heading_mdeg=90_000),
        )

        reloaded = ProvisionalHazard.from_dict(recorded.to_dict())

        self.assertEqual(reloaded, recorded)
        self.assertFalse(
            reloaded.scan_evidence_history[-1]
            .all_clear_arc_covers_target_hypothesis
        )

    def test_partial_all_clear_scan_does_not_authorize_a_route(self):
        mapped = hazard_map()
        pose = PhysicalPose(x_mm=-236)
        result = replace(
            all_clear_result(),
            scan_id="scan-all-clear-timeout",
            reason="scan_deadline_exceeded",
        )

        recorded = mapped.record_scan_result(result, scan_pose=pose)
        evidence = mapped.route_evidence("hazard-a", pose=pose)

        self.assertTrue(recorded.active_for_collision)
        self.assertFalse(evidence["ready"])
        self.assertFalse(evidence["best_effort_ready"])
        self.assertEqual(evidence["strength"], "NONE")

    def test_clear_arc_must_cover_collision_support_radius_too(self):
        mapped = hazard_map()
        original = mapped.hazards[0]
        mapped._hazards = (
            ProvisionalHazard(
                **{
                    **original.__dict__,
                    "collision_supports": (
                        AngularCollisionSupport(
                            source_scan_id="scan-blocked-edge",
                            completed_at_ms=1_500,
                            pose_x_mm=0,
                            pose_y_mm=0,
                            pose_heading_mdeg=0,
                            actual_relative_bearing_mdeg=55_000,
                            based_on_map_version=0,
                        ),
                    ),
                }
            ),
        )

        recorded = mapped.record_scan_result(
            all_clear_result(),
            scan_pose=PhysicalPose(),
        )

        attempt = recorded.scan_evidence_history[-1]
        self.assertFalse(
            attempt.all_clear_arc_covers_target_hypothesis
        )
        self.assertTrue(recorded.active_for_collision)

    def test_unilateral_boundary_allows_labeled_best_effort_commitment(self):
        attempt = ScanAttemptEvidence(
            scan_id="scan-unilateral",
            completed_at_ms=2_000,
            status="CANCELLED",
            reason="bilateral_boundaries_not_observed",
            rays=(
                ScanRayEvidence(10_000, 10_000, True, 30, 30),
                ScanRayEvidence(30_000, 30_000, False, 70, 70),
            ),
            left_boundary_mdeg=20_000,
            right_boundary_mdeg=None,
            scan_pose=PhysicalPose(),
            based_on_map_version=1,
        )
        mapped = hazard_map()
        original = mapped.hazards[0]
        mapped._hazards = (
            ProvisionalHazard(
                **{
                    **original.__dict__,
                    "scan_evidence_history": (attempt,),
                }
            ),
        )
        evidence = mapped.route_evidence("hazard-a", pose=PhysicalPose())

        self.assertFalse(evidence["ready"])
        self.assertTrue(evidence["best_effort_ready"])
        self.assertEqual(evidence["strength"], "UNILATERAL_BOUNDARY")
        proposal = {
            "id": "route-a",
            "revision": 1,
            "transition": "START",
            "objective": "Pass the remembered obstacle",
            "target_hypothesis_id": "hazard-a",
            "detour_side": "RIGHT_OF_GOAL",
            "success_fact_keys": [
                FACT_GOAL_CORRIDOR_CLEAR,
                FACT_GOAL_HEADING_ALIGNED,
                FACT_TARGET_BEHIND,
            ],
            "current_focus_fact_key": FACT_GOAL_CORRIDOR_CLEAR,
            "revision_reason": None,
        }
        state = ManeuverCommitment().apply(
            proposal,
            action=OBSERVE,
            turn=1,
            hazard_map=mapped,
            pose=PhysicalPose(),
            fact_values={},
        )
        self.assertEqual(
            state["active"]["target_hypothesis_id"], "hazard-a"
        )

    def test_blocked_arc_allows_explicit_low_strength_commitment(self):
        attempt = ScanAttemptEvidence(
            scan_id="scan-blocked-arc",
            completed_at_ms=2_000,
            status="CANCELLED",
            reason="bilateral_boundaries_not_observed",
            rays=(
                ScanRayEvidence(-30_000, -30_000, True, 30, 30),
                ScanRayEvidence(0, 0, True, 30, 30),
                ScanRayEvidence(30_000, 30_000, True, 30, 30),
            ),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
            scan_pose=PhysicalPose(),
            based_on_map_version=1,
        )
        mapped = hazard_map()
        original = mapped.hazards[0]
        mapped._hazards = (
            ProvisionalHazard(
                **{
                    **original.__dict__,
                    "scan_evidence_history": (attempt,),
                }
            ),
        )

        evidence = mapped.route_evidence("hazard-a", pose=PhysicalPose())

        self.assertFalse(evidence["ready"])
        self.assertTrue(evidence["best_effort_ready"])
        self.assertEqual(evidence["strength"], "BLOCKED_ARC")
        self.assertEqual(evidence["reason"], "BLOCKED_ARC_WITHOUT_BOUNDARY")


if __name__ == "__main__":
    unittest.main()
