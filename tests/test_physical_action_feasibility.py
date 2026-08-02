import unittest
from unittest import mock

from robot_agent.active_ir_scan_contract import ActiveIrScanCalibration
from robot_agent.maneuver_commitment import (
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
)
from robot_agent.physical_action_feasibility import (
    detour_scan_required_target_ids,
    detour_scan_target_error,
    detour_turn_commitment_error,
    navigation_action_feasibility,
    prepare_navigation_availability,
)
from robot_agent.physical_footprint import RobotFootprint
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    EXPECTED_ACTION_SPECS,
    OBSERVE,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from robot_agent.physical_odometry import OdometryCalibration, PhysicalPose
from robot_agent.provisional_hazard_map import (
    HazardMapCalibration,
    ProvisionalHazard,
    ProvisionalHazardMap,
)


def mapped_hazard(*, footprint=None):
    return ProvisionalHazardMap(
        frame_id="frame-a",
        map_generation_id="generation-a",
        calibration=HazardMapCalibration(robot_footprint=footprint),
        hazards=(
            ProvisionalHazard(
                hypothesis_id="box-a",
                frame_id="frame-a",
                anchor_x_mm=0,
                anchor_y_mm=0,
                anchor_heading_mdeg=0,
                centroid_x_mm=140,
                centroid_y_mm=0,
                radius_mm=70,
                first_seen_at_ms=1,
                last_seen_at_ms=1,
                evidence_count=1,
                last_state_version=1,
                last_raw_ir_proximity=31,
                last_filtered_ir_proximity=32,
            ),
        ),
    )


def detour_navigation(*, hazards, conflicts):
    return {
        "navigation_hazard_hypotheses": list(hazards),
        "goal_geometry": {"conflicts": list(conflicts)},
    }


def detour_hazard(hypothesis_id, *, route_ready=False):
    return {
        "hypothesis_id": hypothesis_id,
        "active_for_collision": True,
        "route_commitment_ready": route_ready,
    }


def goal_conflict(hypothesis_id):
    return {
        "hypothesis_id": hypothesis_id,
        "active_for_collision": True,
    }


class DetourScanGateTests(unittest.TestCase):
    def test_prepare_publishes_json_list_for_planner_scan_targets(self):
        navigation = {
            "action_feasibility": {},
            "navigation_hazard_hypotheses": [],
        }
        with mock.patch(
            "robot_agent.physical_action_feasibility.detour_scan_gate",
            return_value=(True, ()),
        ), mock.patch(
            "robot_agent.physical_action_feasibility.available_navigation_actions",
            return_value=("OBSERVE",),
        ):
            prepare_navigation_availability(
                navigation,
                active_maneuver=None,
                scan_eligible_target_ids={"box-b", "box-a"},
                scan_blocked_target_ids=(),
                scan_budget_available=True,
                reverse_budget_available=True,
                action_specs={},
                observation={},
                repeated_uninformative_observe=False,
            )

        self.assertEqual(
            navigation["scan_eligible_target_hypothesis_ids"],
            ["box-a", "box-b"],
        )
        self.assertIsInstance(
            navigation["scan_eligible_target_hypothesis_ids"],
            list,
        )

    def test_unready_active_goal_conflict_requires_scan(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-a"),),
            conflicts=(goal_conflict("box-a"),),
        )

        result = detour_scan_required_target_ids(
            navigation,
            active_maneuver=None,
            scan_available=True,
            scan_eligible_target_ids=("box-a",),
        )

        self.assertEqual(result, ("box-a",))

    def test_route_ready_goal_conflict_does_not_require_scan(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-a", route_ready=True),),
            conflicts=(goal_conflict("box-a"),),
        )

        result = detour_scan_required_target_ids(
            navigation,
            active_maneuver=None,
            scan_available=True,
            scan_eligible_target_ids=("box-a",),
        )

        self.assertEqual(result, ())

    def test_route_ready_target_is_not_offered_for_rescan(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-a", route_ready=True),),
            conflicts=(goal_conflict("box-a"),),
        )
        navigation["action_feasibility"] = {}

        with mock.patch(
            "robot_agent.physical_action_feasibility.detour_scan_gate",
            return_value=(False, ()),
        ) as scan_gate, mock.patch(
            "robot_agent.physical_action_feasibility.available_navigation_actions",
            return_value=(OBSERVE,),
        ):
            prepare_navigation_availability(
                navigation,
                active_maneuver=None,
                scan_eligible_target_ids=("box-a",),
                scan_blocked_target_ids=(),
                scan_budget_available=True,
                reverse_budget_available=True,
                action_specs={},
                observation={},
                repeated_uninformative_observe=False,
            )

        self.assertEqual(
            navigation["scan_eligible_target_hypothesis_ids"],
            [],
        )
        self.assertEqual(
            scan_gate.call_args.kwargs["scan_eligible_target_ids"],
            [],
        )

    def test_active_maneuver_target_is_exempt(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-a"),),
            conflicts=(goal_conflict("box-a"),),
        )

        result = detour_scan_required_target_ids(
            navigation,
            active_maneuver={"target_hypothesis_id": "box-a"},
            scan_available=True,
            scan_eligible_target_ids=("box-a",),
        )

        self.assertEqual(result, ())

    def test_new_conflict_still_requires_scan_during_active_maneuver(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-a"), detour_hazard("box-b")),
            conflicts=(goal_conflict("box-a"), goal_conflict("box-b")),
        )

        result = detour_scan_required_target_ids(
            navigation,
            active_maneuver={"target_hypothesis_id": "box-a"},
            scan_available=True,
            scan_eligible_target_ids=("box-a", "box-b"),
        )

        self.assertEqual(result, ("box-b",))

    def test_lateral_non_conflict_hazard_does_not_require_scan(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-lateral"),),
            conflicts=(),
        )

        result = detour_scan_required_target_ids(
            navigation,
            active_maneuver=None,
            scan_available=True,
            scan_eligible_target_ids=("box-lateral",),
        )

        self.assertEqual(result, ())

    def test_unavailable_or_ineligible_scan_does_not_gate_turns(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-a"),),
            conflicts=(goal_conflict("box-a"),),
        )

        unavailable = detour_scan_required_target_ids(
            navigation,
            active_maneuver=None,
            scan_available=False,
            scan_eligible_target_ids=("box-a",),
        )
        ineligible = detour_scan_required_target_ids(
            navigation,
            active_maneuver=None,
            scan_available=True,
            scan_eligible_target_ids=(),
        )

        self.assertEqual(unavailable, ())
        self.assertEqual(ineligible, ())

    def test_rotation_blocked_scan_requires_clearance_reverse_before_turn(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-a"),),
            conflicts=(goal_conflict("box-a"),),
        )

        result = detour_scan_required_target_ids(
            navigation,
            active_maneuver=None,
            scan_available=False,
            scan_eligible_target_ids=("box-a",),
            scan_blocked_by_rotation=True,
            clearance_reverse_available=True,
        )

        self.assertEqual(result, ("box-a",))

    def test_rotation_blocked_scan_allows_turn_when_reverse_is_unavailable(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-a"),),
            conflicts=(goal_conflict("box-a"),),
        )

        result = detour_scan_required_target_ids(
            navigation,
            active_maneuver=None,
            scan_available=False,
            scan_eligible_target_ids=("box-a",),
            scan_blocked_by_rotation=True,
            clearance_reverse_available=False,
        )

        self.assertEqual(result, ())

    def test_other_scan_unavailability_preserves_turn_recovery(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-a"),),
            conflicts=(goal_conflict("box-a"),),
        )

        result = detour_scan_required_target_ids(
            navigation,
            active_maneuver=None,
            scan_available=False,
            scan_eligible_target_ids=("box-a",),
            scan_blocked_by_rotation=False,
            clearance_reverse_available=True,
        )

        self.assertEqual(result, ())

    def test_rotation_block_does_not_gate_scan_ineligible_target(self):
        navigation = detour_navigation(
            hazards=(detour_hazard("box-a"),),
            conflicts=(goal_conflict("box-a"),),
        )

        result = detour_scan_required_target_ids(
            navigation,
            active_maneuver=None,
            scan_available=False,
            scan_eligible_target_ids=(),
            scan_blocked_by_rotation=True,
            clearance_reverse_available=True,
        )

        self.assertEqual(result, ())


class DetourCommitmentGateTests(unittest.TestCase):
    def _navigation(self, *, route_ready=True, conflict=True):
        return detour_navigation(
            hazards=(
                detour_hazard("box-a", route_ready=route_ready),
                detour_hazard("scanned-bystander", route_ready=True),
            ),
            conflicts=(goal_conflict("box-a"),) if conflict else (),
        )

    def test_unscanned_conflict_requires_scan_not_commitment(self):
        self.assertIsNone(
            detour_turn_commitment_error(
                TURN_LEFT_90,
                {"transition": "NONE"},
                self._navigation(route_ready=False),
            )
        )

    def test_scanned_conflict_rejects_uncommitted_detour_turn(self):
        self.assertEqual(
            detour_turn_commitment_error(
                TURN_RIGHT_90,
                {"transition": "NONE"},
                self._navigation(),
            )[0],
            "detour_commitment_required",
        )

    def test_start_must_target_scanned_goal_conflict(self):
        navigation = self._navigation()
        valid = {
            "transition": "START",
            "target_hypothesis_id": "box-a",
            "detour_side": "LEFT_OF_GOAL",
            "success_fact_keys": [
                FACT_GOAL_CORRIDOR_CLEAR,
                FACT_GOAL_HEADING_ALIGNED,
                FACT_TARGET_BEHIND,
            ],
            "current_focus_fact_key": FACT_GOAL_CORRIDOR_CLEAR,
        }
        unrelated = {
            **valid,
            "target_hypothesis_id": "scanned-bystander",
        }

        self.assertIsNone(
            detour_turn_commitment_error(
                OBSERVE, valid, navigation
            )
        )
        self.assertEqual(
            detour_turn_commitment_error(
                OBSERVE, unrelated, navigation
            )[0],
            "detour_commitment_target_mismatch",
        )

    def test_nonconflict_or_nonturn_does_not_force_commitment(self):
        none = {"transition": "NONE"}

        self.assertIsNone(
            detour_turn_commitment_error(
                TURN_LEFT_90, none, self._navigation(conflict=False)
            )
        )
        self.assertIsNone(
            detour_turn_commitment_error(
                ADVANCE, none, self._navigation()
            )
        )

    def test_detour_start_requires_observe_and_complete_facts(self):
        navigation = self._navigation()
        incomplete = {
            "transition": "START",
            "target_hypothesis_id": "box-a",
            "detour_side": "LEFT_OF_GOAL",
            "success_fact_keys": [FACT_GOAL_CORRIDOR_CLEAR],
            "current_focus_fact_key": FACT_GOAL_CORRIDOR_CLEAR,
        }
        self.assertEqual(
            detour_turn_commitment_error(
                OBSERVE,
                incomplete,
                navigation,
            )[0],
            "detour_commitment_facts_required",
        )
        self.assertEqual(
            detour_turn_commitment_error(
                TURN_LEFT_90,
                {
                    **incomplete,
                    "success_fact_keys": [
                        FACT_GOAL_CORRIDOR_CLEAR,
                        FACT_GOAL_HEADING_ALIGNED,
                        FACT_TARGET_BEHIND,
                    ],
                },
                navigation,
            )[0],
            "detour_start_requires_observe",
        )

    def test_required_scan_cannot_target_an_unrelated_hazard(self):
        navigation = self._navigation()
        navigation["detour_scan_required_target_hypothesis_ids"] = [
            "box-a"
        ]

        self.assertEqual(
            detour_scan_target_error(
                "SCAN_FRONT_ARC",
                "scanned-bystander",
                navigation,
            )[0],
            "detour_scan_target_mismatch",
        )
        self.assertIsNone(
            detour_scan_target_error(
                "SCAN_FRONT_ARC",
                "box-a",
                navigation,
            )
        )


class PhysicalActionFeasibilityTests(unittest.TestCase):
    def test_asymmetric_body_publishes_only_reverse_near_hazard(self):
        footprint = RobotFootprint(
            front_extent_mm=110,
            rear_extent_mm=90,
            left_extent_mm=105,
            right_extent_mm=160,
            clearance_margin_mm=10,
            calibration_status="provisional-unmeasured",
            calibration_evidence="operator observed right-arm contact",
        )

        result = navigation_action_feasibility(
            hazard_map=mapped_hazard(footprint=footprint),
            pose=PhysicalPose(),
            action_specs=EXPECTED_ACTION_SPECS,
            odometry_calibration=OdometryCalibration(),
            active_scan_calibration=ActiveIrScanCalibration(
                alignment_tolerance_mdeg=10_000,
            ),
        )

        self.assertFalse(result["motion_actions"][ADVANCE]["allowed"])
        self.assertTrue(result["motion_actions"][REVERSE]["allowed"])
        self.assertFalse(
            result["motion_actions"][TURN_LEFT_90]["allowed"]
        )
        self.assertFalse(
            result["motion_actions"][TURN_RIGHT_90]["allowed"]
        )
        self.assertFalse(result["active_scan"]["allowed"])
        self.assertEqual(
            result["collision_geometry"]["right_extent_mm"],
            160,
        )
        self.assertFalse(result["host_ranked_or_selected_action"])

    def test_retreat_changes_scan_feasibility_without_host_route_choice(self):
        footprint = RobotFootprint(
            front_extent_mm=110,
            rear_extent_mm=90,
            left_extent_mm=105,
            right_extent_mm=160,
            clearance_margin_mm=10,
        )
        result = navigation_action_feasibility(
            hazard_map=mapped_hazard(footprint=footprint),
            pose=PhysicalPose(x_mm=-180),
            action_specs=EXPECTED_ACTION_SPECS,
            odometry_calibration=OdometryCalibration(),
            active_scan_calibration=ActiveIrScanCalibration(
                alignment_tolerance_mdeg=10_000,
            ),
        )

        self.assertTrue(result["active_scan"]["allowed"])
        self.assertEqual(
            result["active_scan"]["reason"],
            "in_place_rotation_clear",
        )

    def test_legacy_circle_does_not_block_in_place_scan(self):
        result = navigation_action_feasibility(
            hazard_map=mapped_hazard(),
            pose=PhysicalPose(),
            action_specs=EXPECTED_ACTION_SPECS,
            odometry_calibration=OdometryCalibration(),
            active_scan_calibration=ActiveIrScanCalibration(),
        )

        self.assertTrue(result["active_scan"]["allowed"])
        self.assertEqual(
            result["collision_geometry"]["geometry"],
            "SYMMETRIC_CIRCLE",
        )


if __name__ == "__main__":
    unittest.main()
