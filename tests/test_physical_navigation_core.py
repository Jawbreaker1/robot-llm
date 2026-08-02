import copy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import unittest
from unittest import mock
import uuid

from robot_agent.active_ir_scan import ActiveIrScanExecutor
from robot_agent.active_ir_scan_contract import (
    ActiveIrScanCalibration,
    ActiveIrScanContractError,
    ActiveIrRay,
    ActiveIrScanResult,
    ModelScanChoice,
    build_scan_request,
    validate_scan_result,
)
from robot_agent.ev3_navigation_transport import (
    EV3NavigationRemoteError,
    EV3NavigationSSHTransport,
    EV3NavigationTransportError,
)
from robot_agent.maneuver_commitment import (
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
    ManeuverCommitment,
    ManeuverCommitmentError,
    empty_commitment,
)
from robot_agent.lm_studio_navigation import (
    LMStudioNavigationDecisionError,
    LMStudioNavigationError,
    LMStudioNavigationPlanner,
    _maneuver_schema,
)
from robot_agent.navigation_memory_store import NavigationMemoryStore
from robot_agent.navigation_memory_store import NavigationMemoryError
from robot_agent.navigation_plan_tail import NavigationPlanTail
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    DECISION_SCHEMA,
    EXPECTED_ACTION_SPECS,
    EXPECTED_WORKER_SAFETY,
    FINISH,
    OBSERVE,
    REVERSE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    NavigationDecision,
    PhysicalNavigationContractError,
    expected_scan_turn_profile,
    expected_scan_sample_profile,
    motion_budget_allows,
)
from robot_agent.physical_navigation_mission import DirectionalMission
from robot_agent.physical_observation_progress import (
    RestoredScanProgressBarrier,
    observation_information_result,
    observation_progress_signature,
)
from robot_agent.physical_navigation_runtime import (
    PhysicalNavigationRuntime,
    PhysicalNavigationRuntimeConfig,
    PhysicalNavigationRuntimeError,
)
from robot_agent.physical_navigation_adapter import (
    PhysicalNavigationRuntimeAdapter,
)
from robot_agent.physical_odometry import (
    DriveMotorRoles,
    PhysicalPose,
    apply_verified_motion,
    verified_motion_from_result,
)
from robot_agent.robot_speech_runtime import RobotSpeechRuntime
from robot_agent.provisional_hazard_map import HazardMapCalibration
from robot_agent.physical_footprint import RobotFootprint


def observation(
    version,
    *,
    blocked=False,
    touch=False,
    left_position=0,
    right_position=0,
    left_role="left_drive",
    right_role="right_drive",
    last_outcome=None,
    process_ms_remaining=40_000,
    pulse_count_remaining=40,
    pulse_duration_ms_remaining=32_000,
    motion_fault_latched=False,
):
    return {
        "state_version": version,
        "observed_monotonic_ms": version * 10,
        "touch": {"value0": 1 if touch else 0, "pressed": touch},
        "infrared": {
            "raw": 20 if blocked else 60,
            "filtered": 20 if blocked else 60,
            "blocked": blocked,
            "reason": (
                "blocked_hysteresis_hold"
                if blocked
                else "clear_hysteresis_hold"
            ),
            "sample_count": 5,
        },
        "motors": [
            {
                "role": left_role,
                "position": left_position,
                "state": "",
            },
            {
                "role": right_role,
                "position": right_position,
                "state": "",
            },
        ],
        "last_outcome": last_outcome or {
            "kind": "observe",
            "status": "completed",
        },
        "budgets": {
            "pulse_count": 0,
            "pulse_count_remaining": pulse_count_remaining,
            "pulse_duration_ms": 0,
            "pulse_duration_ms_remaining": pulse_duration_ms_remaining,
            "process_ms_remaining": process_ms_remaining,
            "motion_fault_latched": motion_fault_latched,
        },
    }


def decision_mapping(
    *,
    episode_id,
    turn,
    state_version,
    action,
    plan,
    reason_code,
    commitment=None,
    target=None,
    utterance=None,
):
    return {
        "schema": DECISION_SCHEMA,
        "episode_id": episode_id,
        "turn": turn,
        "based_on_state_version": state_version,
        "action": action,
        "plan": list(plan),
        "reason_code": reason_code,
        "assessment": "The selected action follows the published facts.",
        "utterance": utterance,
        "perception_target_hypothesis_id": target,
        "maneuver_commitment": (
            empty_commitment() if commitment is None else commitment
        ),
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


def degraded_motion_result(
    *,
    action=ADVANCE,
    left_delta=174,
    right_delta=0,
    version=2,
    left_role="left_drive",
    right_role="right_drive",
):
    receipt = {
        "slice_index": 1,
        "slice_count": 1,
        "duration_ms": 250,
        "status": "verification_failed",
        "reason": "encoder_undertravel_observed",
        "started_monotonic_ms": 10,
        "completed_monotonic_ms": 810,
        "motors": [
            {
                "side": "left",
                "role": left_role,
                "position_before": 0,
                "position_after": left_delta,
                "position_delta": left_delta,
                "state": "",
            },
            {
                "side": "right",
                "role": right_role,
                "position_before": 0,
                "position_after": right_delta,
                "position_delta": right_delta,
                "state": "",
            },
        ],
        "encoder_verification": {
            "passed": False,
            "error": "simulated undertravel",
            "checks": [{"passed": False}],
        },
        "stop": stop_proof(),
    }
    receipt["segments"] = [
        {
            "segment_index": 1,
            "kind": "paired",
            "commanded_sides": ["left", "right"],
            "duration_ms": receipt["duration_ms"],
            "status": receipt["status"],
            "reason": receipt["reason"],
            "started_monotonic_ms": receipt["started_monotonic_ms"],
            "completed_monotonic_ms": receipt[
                "completed_monotonic_ms"
            ],
            "motors": copy.deepcopy(receipt["motors"]),
            "encoder_verification": copy.deepcopy(
                receipt["encoder_verification"]
            ),
            "stop": copy.deepcopy(receipt["stop"]),
        }
    ]
    outcome = {
        "kind": "pulse",
        "action": action,
        "status": "verification_failed",
        "reason": "encoder_undertravel_observed",
        "started_monotonic_ms": 10,
        "completed_monotonic_ms": 810,
        "stop_confirmed": True,
        "requested_slice_count": 1,
        "completed_slice_count": 0,
        "slices": [receipt],
        "encoder_verification": {
            "passed": False,
            "verified_slice_count": 0,
            "requested_slice_count": 1,
        },
    }
    return {
        "action": action,
        "outcome": outcome,
        "observation": observation(
            version,
            left_position=left_delta,
            right_position=right_delta,
            left_role=left_role,
            right_role=right_role,
            last_outcome=outcome,
        ),
        "stop": stop_proof(),
    }


def partial_start_motion_result(
    *,
    left_delta=24,
    right_delta=0,
    commanded_sides=("left",),
):
    result = degraded_motion_result(
        left_delta=left_delta,
        right_delta=right_delta,
    )
    outcome = result["outcome"]
    receipt = outcome["slices"][0]
    reason = "cancel_requested"
    checks = [
        {
            "role": motor["role"],
            "side": motor["side"],
            "position_delta": motor["position_delta"],
            "passed": motor["side"] in commanded_sides,
        }
        for motor in receipt["motors"]
    ]
    checks.append(
        {
            "role": "paired_start_completion",
            "side": "paired",
            "position_delta": 0,
            "passed": False,
        }
    )
    failed_proof = {
        "passed": False,
        "error": "paired motor start was incomplete",
        "checks": checks,
    }
    receipt["status"] = "interrupted"
    receipt["reason"] = reason
    receipt["encoder_verification"] = copy.deepcopy(failed_proof)
    segment = receipt["segments"][0]
    segment["kind"] = "partial_start"
    segment["commanded_sides"] = list(commanded_sides)
    segment["status"] = "interrupted"
    segment["reason"] = reason
    segment["encoder_verification"] = copy.deepcopy(failed_proof)
    outcome["status"] = "interrupted"
    outcome["reason"] = reason
    result["observation"]["last_outcome"] = outcome
    return result


def recovered_motion_result():
    primary_motors = [
        {
            "side": "left",
            "role": "left_drive",
            "position_before": 0,
            "position_after": 75,
            "position_delta": 75,
            "state": "",
        },
        {
            "side": "right",
            "role": "right_drive",
            "position_before": 0,
            "position_after": 0,
            "position_delta": 0,
            "state": "",
        },
    ]
    catch_up_motors = [
        {
            "side": "left",
            "role": "left_drive",
            "position_before": 75,
            "position_after": 75,
            "position_delta": 0,
            "state": "",
        },
        {
            "side": "right",
            "role": "right_drive",
            "position_before": 0,
            "position_after": 75,
            "position_delta": 75,
            "state": "",
        },
    ]
    segments = [
        {
            "segment_index": 1,
            "kind": "paired",
            "commanded_sides": ["left", "right"],
            "duration_ms": 250,
            "status": "verification_failed",
            "reason": "encoder_undertravel_observed",
            "started_monotonic_ms": 10,
            "completed_monotonic_ms": 810,
            "motors": primary_motors,
            "encoder_verification": {
                "passed": False,
                "error": "right motor undertravel",
                "checks": [{"passed": True}, {"passed": False}],
            },
            "stop": stop_proof(),
        },
        {
            "segment_index": 2,
            "kind": "right_catch_up",
            "commanded_sides": ["right"],
            "duration_ms": 300,
            "status": "completed",
            "reason": "duration_elapsed",
            "started_monotonic_ms": 820,
            "completed_monotonic_ms": 1_120,
            "motors": catch_up_motors,
            "encoder_verification": {
                "passed": True,
                "error": None,
                "checks": [{"passed": True}],
            },
            "stop": stop_proof(),
        },
    ]
    aggregate_motors = [
        {
            "side": "left",
            "role": "left_drive",
            "position_before": 0,
            "position_after": 75,
            "position_delta": 75,
            "state": "",
        },
        {
            "side": "right",
            "role": "right_drive",
            "position_before": 0,
            "position_after": 75,
            "position_delta": 75,
            "state": "",
        },
    ]
    receipt = {
        "slice_index": 1,
        "slice_count": 1,
        "duration_ms": 250,
        "status": "completed",
        "reason": "encoder_recovered",
        "started_monotonic_ms": 10,
        "completed_monotonic_ms": 1_120,
        "motors": aggregate_motors,
        "segments": segments,
        "encoder_verification": {
            "passed": True,
            "error": None,
            "checks": [
                {"passed": True},
                {"passed": True},
                {"passed": True},
            ],
        },
        "stop": stop_proof(),
    }
    outcome = {
        "kind": "pulse",
        "action": ADVANCE,
        "status": "completed",
        "reason": "semantic_action_completed",
        "started_monotonic_ms": 10,
        "completed_monotonic_ms": 1_120,
        "stop_confirmed": True,
        "requested_slice_count": 1,
        "completed_slice_count": 1,
        "slices": [receipt],
        "encoder_verification": {
            "passed": True,
            "verified_slice_count": 1,
            "requested_slice_count": 1,
        },
    }
    return {
        "action": ADVANCE,
        "outcome": outcome,
        "observation": observation(
            2,
            left_position=75,
            right_position=75,
            last_outcome=outcome,
        ),
        "stop": stop_proof(),
    }


class PhysicalNavigationContractTests(unittest.TestCase):
    def test_latched_motion_fault_blocks_only_motion_budget(self):
        latched = observation(1, motion_fault_latched=True)

        self.assertFalse(motion_budget_allows(
            ADVANCE,
            latched,
            EXPECTED_ACTION_SPECS,
        ))
        self.assertTrue(motion_budget_allows(
            OBSERVE,
            latched,
            EXPECTED_ACTION_SPECS,
        ))

    def test_motion_requires_exact_model_authored_tail(self):
        with self.assertRaises(PhysicalNavigationContractError) as caught:
            NavigationDecision.from_mapping(
                decision_mapping(
                    episode_id="episode-a",
                    turn=1,
                    state_version=1,
                    action=ADVANCE,
                    plan=[ADVANCE],
                    reason_code="PROGRESS_GOAL",
                ),
                episode_id="episode-a",
                turn=1,
                state_version=1,
            )
        self.assertEqual(caught.exception.code, "motion_plan_requires_tail")

        accepted = NavigationDecision.from_mapping(
            decision_mapping(
                episode_id="episode-a",
                turn=1,
                state_version=1,
                action=ADVANCE,
                plan=[ADVANCE, REVERSE, ADVANCE],
                reason_code="PROGRESS_GOAL",
            ),
            episode_id="episode-a",
            turn=1,
            state_version=1,
        )
        self.assertEqual(
            accepted.plan,
            (ADVANCE, REVERSE, ADVANCE),
        )

    def test_nonmotion_plan_is_singleton(self):
        with self.assertRaises(PhysicalNavigationContractError):
            NavigationDecision.from_mapping(
                decision_mapping(
                    episode_id="episode-a",
                    turn=1,
                    state_version=1,
                    action=OBSERVE,
                    plan=[OBSERVE, OBSERVE],
                    reason_code="VERIFY_RESULT",
                ),
                episode_id="episode-a",
                turn=1,
                state_version=1,
            )


class PhysicalMemoryAndMapTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary.name) / "navigation.json"
        self.memory = NavigationMemoryStore.load(
            path=self.path,
            robot_id="ev3rstorm-01",
            controller_instance_id="ev3-main",
            reset=True,
            clock_ms=lambda: 1_000,
            uuid_factory=lambda: uuid.UUID(int=1),
        )
        self.memory.bind_drive_roles(DriveMotorRoles())

    def tearDown(self):
        self.temporary.cleanup()

    def test_blocked_hypothesis_persists_after_clear_turned_view(self):
        self.memory.begin_episode(
            observation(1, blocked=True),
            1_001,
        )
        hazard_ids = self.memory.hazard_map.hazard_ids
        self.assertEqual(len(hazard_ids), 1)
        self.memory.pose = PhysicalPose(heading_mdeg=90_000)
        self.memory.ingest_stationary_observation(
            observation(2, blocked=False),
            1_002,
        )
        self.assertEqual(self.memory.hazard_map.hazard_ids, hazard_ids)

        loaded = NavigationMemoryStore.load(
            path=self.path,
            robot_id="ev3rstorm-01",
            controller_instance_id="ev3-main",
        )
        self.assertEqual(loaded.hazard_map.hazard_ids, hazard_ids)

    def test_scan_boundaries_survive_later_reanchor_observation_time(self):
        self.memory.begin_episode(
            observation(1, blocked=True),
            1_001,
        )
        target = self.memory.hazard_map.hazard_ids[0]
        scan_basis = self.memory.hazard_map.revision

        self.memory.ingest_stationary_observation(
            observation(2, blocked=True),
            3_000,
        )
        refreshed = self.memory.hazard_map.get(target)
        self.assertEqual(refreshed.last_seen_at_ms, 3_000)

        recorded = self.memory.hazard_map.record_scan_boundaries(
            target,
            evidence_frame_id=self.memory.frame_id,
            evidence_map_generation_id=self.memory.generation_id,
            based_on_map_version=scan_basis,
            completed_at_ms=2_000,
            left_boundary_mdeg=20_000,
            right_boundary_mdeg=-20_000,
        )

        self.assertEqual(recorded.scan_completed_at_ms, 2_000)
        self.assertEqual(recorded.last_seen_at_ms, 3_000)
        self.assertTrue(recorded.bilateral_scan_complete)

    def test_restored_unilateral_scan_evidence_persists_actual_bearings(self):
        self.memory.begin_episode(
            observation(1, blocked=True),
            1_001,
        )
        target = self.memory.hazard_map.hazard_ids[0]
        basis = self.memory.hazard_map.revision
        self.memory.ingest_stationary_observation(
            observation(2, blocked=True),
            2_100,
        )
        rays = tuple(
            ActiveIrRay(
                ordinal=index,
                requested_relative_bearing_mdeg=requested,
                actual_relative_bearing_mdeg=actual,
                observed_at_ms=2_000 + index,
                state_version=2 + index,
                raw=31 if blocked else 62,
                filtered=32 if blocked else 61,
                blocked=blocked,
            )
            for index, (requested, actual, blocked) in enumerate(
                (
                    (0, 0, True),
                    (-30_000, -28_500, True),
                    (-60_000, -57_500, True),
                    (15_000, 14_250, False),
                    (30_000, 28_500, False),
                    (60_000, 57_500, False),
                ),
                start=1,
            )
        )
        result = ActiveIrScanResult(
            scan_id="scan-unilateral-live-shape",
            target_hypothesis_id=target,
            frame_id=self.memory.frame_id,
            map_generation_id=self.memory.generation_id,
            based_on_map_version=basis,
            started_at_ms=2_000,
            completed_at_ms=2_050,
            status="CANCELLED",
            reason="bilateral_boundaries_not_observed",
            stop_confirmed=True,
            restored_start_heading=True,
            rays=rays,
            left_boundary_mdeg=7_500,
            right_boundary_mdeg=None,
        )

        recorded = self.memory.hazard_map.record_scan_result(
            result,
            scan_pose=self.memory.pose,
        )
        self.memory.save()
        loaded = NavigationMemoryStore.load(
            path=self.path,
            robot_id="ev3rstorm-01",
            controller_instance_id="ev3-main",
        )
        evidence = loaded.hazard_map.get(
            recorded.hypothesis_id
        ).scan_evidence_history[-1]

        self.assertEqual(evidence.arc_coverage, "BILATERAL_ARC")
        self.assertEqual(
            evidence.boundary_coverage,
            "POSITIVE_BOUNDARY_ONLY",
        )
        self.assertEqual(evidence.observation_pattern, "MIXED")
        self.assertEqual(evidence.scan_pose, self.memory.pose)
        self.assertEqual(evidence.based_on_map_version, basis)
        self.assertEqual(evidence.left_boundary_mdeg, 7_500)
        self.assertIsNone(evidence.right_boundary_mdeg)
        self.assertEqual(
            [ray.actual_relative_bearing_mdeg for ray in evidence.rays],
            [0, -28_500, -57_500, 14_250, 28_500, 57_500],
        )
        self.assertFalse(recorded.bilateral_scan_complete)

    def test_all_clear_scan_conflicts_with_old_bilateral_hypothesis(self):
        self.memory.begin_episode(
            observation(1, blocked=True),
            1_001,
        )
        target = self.memory.hazard_map.hazard_ids[0]
        basis = self.memory.hazard_map.revision
        self.memory.ingest_stationary_observation(
            observation(2, blocked=True),
            2_100,
        )
        mixed_rays = tuple(
            ActiveIrRay(
                ordinal=index,
                requested_relative_bearing_mdeg=bearing,
                actual_relative_bearing_mdeg=bearing,
                observed_at_ms=2_000 + index,
                state_version=2 + index,
                raw=31 if blocked else 62,
                filtered=32 if blocked else 61,
                blocked=blocked,
            )
            for index, (bearing, blocked) in enumerate(
                ((-60_000, False), (-30_000, True), (0, True),
                 (30_000, True), (60_000, False)),
                start=1,
            )
        )
        self.memory.hazard_map.record_scan_result(
            ActiveIrScanResult(
                scan_id="scan-bilateral-before-conflict",
                target_hypothesis_id=target,
                frame_id=self.memory.frame_id,
                map_generation_id=self.memory.generation_id,
                based_on_map_version=basis,
                started_at_ms=2_000,
                completed_at_ms=2_050,
                status="COMPLETED",
                reason="bilateral_boundaries_observed",
                stop_confirmed=True,
                restored_start_heading=True,
                rays=mixed_rays,
                left_boundary_mdeg=45_000,
                right_boundary_mdeg=-45_000,
            ),
            scan_pose=self.memory.pose,
        )
        self.assertTrue(
            self.memory.hazard_map.get(target).bilateral_scan_complete
        )

        second_basis = self.memory.hazard_map.revision
        self.memory.ingest_stationary_observation(
            observation(3, blocked=False),
            3_100,
        )
        clear_rays = tuple(
            ActiveIrRay(
                ordinal=index,
                requested_relative_bearing_mdeg=bearing,
                actual_relative_bearing_mdeg=bearing,
                observed_at_ms=3_000 + index,
                state_version=10 + index,
                raw=62,
                filtered=61,
                blocked=False,
            )
            for index, bearing in enumerate(
                (-60_000, -30_000, 0, 30_000, 60_000),
                start=1,
            )
        )
        recorded = self.memory.hazard_map.record_scan_result(
            ActiveIrScanResult(
                scan_id="scan-all-clear-conflict",
                target_hypothesis_id=target,
                frame_id=self.memory.frame_id,
                map_generation_id=self.memory.generation_id,
                based_on_map_version=second_basis,
                started_at_ms=3_000,
                completed_at_ms=3_050,
                status="CANCELLED",
                reason="bilateral_boundaries_not_observed",
                stop_confirmed=True,
                restored_start_heading=True,
                rays=clear_rays,
                left_boundary_mdeg=None,
                right_boundary_mdeg=None,
            ),
            scan_pose=self.memory.pose,
        )

        self.assertFalse(recorded.bilateral_scan_complete)
        self.assertIsNone(recorded.scan_completed_at_ms)
        self.assertEqual(
            recorded.scan_evidence_history[-1].hypothesis_relation,
            "CONFLICTS_BLOCKED_HYPOTHESIS",
        )
        self.assertFalse(recorded.active_for_collision)
        context_hazard = self.memory.hazard_map.context()[
            "navigation_hazard_hypotheses"
        ][0]
        self.assertFalse(context_hazard["active_for_collision"])
        self.assertEqual(context_hazard["collision_support_count"], 0)
        self.assertFalse(recorded.to_dict()["bilateral_scan_complete"])

    def test_scan_boundary_fusion_rejects_foreign_or_stale_basis(self):
        self.memory.begin_episode(
            observation(1, blocked=True),
            1_001,
        )
        target = self.memory.hazard_map.hazard_ids[0]
        scan_basis = self.memory.hazard_map.revision
        self.memory.ingest_stationary_observation(
            observation(2, blocked=True),
            2_500,
        )
        common = {
            "completed_at_ms": 2_000,
            "left_boundary_mdeg": 20_000,
            "right_boundary_mdeg": -20_000,
        }

        with self.assertRaisesRegex(ValueError, "foreign map"):
            self.memory.hazard_map.record_scan_boundaries(
                target,
                evidence_frame_id="foreign-frame",
                evidence_map_generation_id=self.memory.generation_id,
                based_on_map_version=scan_basis,
                **common,
            )
        with self.assertRaisesRegex(ValueError, "foreign map"):
            self.memory.hazard_map.record_scan_boundaries(
                target,
                evidence_frame_id=self.memory.frame_id,
                evidence_map_generation_id="foreign-generation",
                based_on_map_version=scan_basis,
                **common,
            )

        self.memory.ingest_stationary_observation(
            observation(3, blocked=False),
            3_000,
        )
        with self.assertRaisesRegex(ValueError, "basis is stale"):
            self.memory.hazard_map.record_scan_boundaries(
                target,
                evidence_frame_id=self.memory.frame_id,
                evidence_map_generation_id=self.memory.generation_id,
                based_on_map_version=scan_basis,
                **common,
            )

    def test_swept_path_vetoes_approach_and_allows_monotonic_escape(self):
        self.memory.begin_episode(
            observation(1, blocked=True),
            1_001,
        )
        advance = self.memory.hazard_map.validate_swept_path(
            self.memory.pose,
            ADVANCE,
            EXPECTED_ACTION_SPECS,
        )
        reverse = self.memory.hazard_map.validate_swept_path(
            self.memory.pose,
            REVERSE,
            EXPECTED_ACTION_SPECS,
        )
        self.assertFalse(advance["allowed"])
        self.assertTrue(reverse["allowed"])
        self.assertEqual(
            reverse["monotonic_escape_hazard_ids"],
            list(self.memory.hazard_map.hazard_ids),
        )

    def test_directional_mission_origin_never_moves(self):
        mission = DirectionalMission.begin(
            episode_id="episode-a",
            minimum_forward_progress_mm=100,
            pose=PhysicalPose(x_mm=10, y_mm=20, heading_mdeg=0),
        )
        first = mission.snapshot(
            pose=PhysicalPose(x_mm=80, y_mm=20, heading_mdeg=0),
            action_specs=EXPECTED_ACTION_SPECS,
            goal_corridor_clear=True,
            all_known_hazards_passed=True,
            localization_valid=True,
            touch_pressed=False,
        )
        second = mission.snapshot(
            pose=PhysicalPose(x_mm=120, y_mm=20, heading_mdeg=0),
            action_specs=EXPECTED_ACTION_SPECS,
            goal_corridor_clear=True,
            all_known_hazards_passed=True,
            localization_valid=True,
            touch_pressed=False,
        )
        self.assertEqual(first["origin_x_mm"], 10)
        self.assertEqual(second["origin_x_mm"], 10)
        self.assertEqual(second["current_longitudinal_progress_mm"], 110)
        self.assertTrue(second["completed"])

    def test_directional_mission_uses_controller_heading_tolerance(self):
        mission = DirectionalMission.begin(
            episode_id="episode-heading-tolerance",
            minimum_forward_progress_mm=100,
            pose=PhysicalPose(),
            heading_tolerance_mdeg=20_000,
        )

        self.assertTrue(
            mission.heading_aligned(PhysicalPose(heading_mdeg=16_500))
        )
        self.assertFalse(
            mission.heading_aligned(PhysicalPose(heading_mdeg=20_001))
        )

    def test_real_drive_roles_anchor_and_detect_unobserved_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "real-roles.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=9),
            )
            memory.bind_drive_roles(
                DriveMotorRoles(left="drive_b", right="drive_c")
            )
            memory.begin_episode(
                observation(
                    1,
                    left_role="drive_b",
                    right_role="drive_c",
                ),
                1_001,
            )
            self.assertEqual(
                memory.motor_positions,
                {"drive_b": 0, "drive_c": 0},
            )
            with self.assertRaises(NavigationMemoryError):
                memory.ingest_stationary_observation(
                    observation(
                        2,
                        left_position=12,
                        right_position=12,
                        left_role="drive_b",
                        right_role="drive_c",
                    ),
                    1_002,
                )
            self.assertFalse(memory.localization_valid)

    def test_clean_undertravel_updates_arc_pose_and_keeps_localization(self):
        self.memory.begin_episode(observation(1), 1_001)
        result = degraded_motion_result()

        self.memory.apply_motion_result(ADVANCE, result, 1_002)

        self.assertTrue(self.memory.localization_valid)
        self.assertEqual(self.memory.pose.x_mm, 30)
        self.assertEqual(self.memory.pose.y_mm, -3)
        self.assertEqual(self.memory.pose.heading_mdeg, -11_484)
        self.assertEqual(
            self.memory.motor_positions,
            {"left_drive": 174, "right_drive": 0},
        )


class PhysicalOdometryEvidenceTests(unittest.TestCase):
    def test_partial_start_encoder_motion_reaches_odometry(self):
        motion = verified_motion_from_result(
            ADVANCE,
            partial_start_motion_result(),
        )

        self.assertFalse(motion.complete)
        self.assertEqual(motion.observed_slice_count, 1)
        self.assertEqual(motion.left_encoder_delta_degrees, 24)
        self.assertEqual(motion.right_encoder_delta_degrees, 0)
        pose = apply_verified_motion(PhysicalPose(), motion)
        self.assertEqual(pose.x_mm, 4)
        self.assertEqual(pose.heading_mdeg, -1_584)

    def test_failed_command_retains_clean_encoder_observation(self):
        result = degraded_motion_result()
        motion = verified_motion_from_result(ADVANCE, result)

        self.assertFalse(motion.complete)
        self.assertEqual(motion.verified_slice_count, 0)
        self.assertEqual(motion.observed_slice_count, 1)
        self.assertEqual(motion.left_encoder_delta_degrees, 174)
        self.assertEqual(motion.right_encoder_delta_degrees, 0)

        pose = apply_verified_motion(PhysicalPose(), motion)
        self.assertEqual(pose.x_mm, 30)
        self.assertEqual(pose.y_mm, -3)
        self.assertEqual(pose.heading_mdeg, -11_484)

    def test_zero_zero_undertravel_reanchors_without_false_motion(self):
        result = degraded_motion_result(left_delta=0, right_delta=0)
        motion = verified_motion_from_result(ADVANCE, result)

        self.assertEqual(
            apply_verified_motion(PhysicalPose(), motion),
            PhysicalPose(),
        )

    def test_wrong_direction_observation_is_not_localizable(self):
        result = degraded_motion_result(left_delta=20, right_delta=-5)
        motion = verified_motion_from_result(ADVANCE, result)

        with self.assertRaises(PhysicalNavigationContractError) as caught:
            apply_verified_motion(PhysicalPose(), motion)
        self.assertEqual(caught.exception.code, "encoder_direction_mismatch")

    def test_recovery_segments_preserve_the_real_two_arc_path(self):
        motion = verified_motion_from_result(
            ADVANCE,
            recovered_motion_result(),
        )

        self.assertTrue(motion.complete)
        self.assertEqual(motion.observed_slice_count, 2)
        self.assertEqual(motion.left_encoder_delta_degrees, 75)
        self.assertEqual(motion.right_encoder_delta_degrees, 75)
        pose = apply_verified_motion(PhysicalPose(), motion)
        self.assertEqual(pose.x_mm, 26)
        self.assertEqual(pose.y_mm, -2)
        self.assertEqual(pose.heading_mdeg, 0)

    def test_recovery_segment_totals_must_match_slice_aggregate(self):
        result = recovered_motion_result()
        result["outcome"]["slices"][0]["motors"][0][
            "position_after"
        ] = 76
        result["outcome"]["slices"][0]["motors"][0][
            "position_delta"
        ] = 76

        with self.assertRaises(PhysicalNavigationContractError) as caught:
            verified_motion_from_result(ADVANCE, result)
        self.assertEqual(
            caught.exception.code,
            "motion_segment_aggregate_mismatch",
        )


class PlanTailAndCommitmentTests(unittest.TestCase):
    def _memory(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        memory = NavigationMemoryStore.load(
            path=Path(temporary.name) / "memory.json",
            robot_id="ev3rstorm-01",
            controller_instance_id="ev3-main",
            reset=True,
            clock_ms=lambda: 1_000,
            uuid_factory=lambda: uuid.UUID(int=2),
        )
        memory.bind_drive_roles(DriveMotorRoles())
        memory.begin_episode(observation(1), 1_001)
        return memory

    def test_tail_requires_fresh_unchanged_authoritative_state(self):
        memory = self._memory()
        decision = NavigationDecision.from_mapping(
            decision_mapping(
                episode_id="episode-a",
                turn=1,
                state_version=1,
                action=ADVANCE,
                plan=[ADVANCE, ADVANCE],
                reason_code="PROGRESS_GOAL",
            ),
            episode_id="episode-a",
            turn=1,
            state_version=1,
        )
        maneuver = ManeuverCommitment()
        facts = {
            FACT_GOAL_CORRIDOR_CLEAR: True,
            FACT_GOAL_HEADING_ALIGNED: True,
            FACT_TARGET_BEHIND: {},
        }
        tail = NavigationPlanTail.from_decision(
            decision,
            now_monotonic=0.0,
            episode_deadline=20.0,
            map_context=memory.context(),
            observation=observation(1),
            maneuver_state=maneuver.state(1),
            fact_values=facts,
        )
        stale = tail.next_action(
            now_monotonic=0.1,
            map_context=memory.context(),
            observation=observation(1),
            maneuver_state=maneuver.state(1),
            fact_values=facts,
            localization_valid=True,
        )
        self.assertIsNone(stale)
        self.assertIn(
            "plan_tail_observation_not_fresh",
            tail.cancelled_reason,
        )

    def test_reverse_tail_stops_as_soon_as_scan_rotation_is_clear(self):
        memory = self._memory()
        decision = NavigationDecision.from_mapping(
            decision_mapping(
                episode_id="episode-scan-room",
                turn=1,
                state_version=1,
                action=REVERSE,
                plan=[REVERSE, REVERSE, REVERSE],
                reason_code="HANDLE_OBSTACLE",
            ),
            episode_id="episode-scan-room",
            turn=1,
            state_version=1,
        )
        maneuver = ManeuverCommitment()
        facts = {
            FACT_GOAL_CORRIDOR_CLEAR: False,
            FACT_GOAL_HEADING_ALIGNED: True,
            FACT_TARGET_BEHIND: {},
        }
        initial = dict(memory.context())
        initial["detour_scan_required_target_hypothesis_ids"] = ["box-a"]
        initial["action_feasibility"] = {
            "active_scan": {"allowed": False},
        }
        tail = NavigationPlanTail.from_decision(
            decision,
            now_monotonic=0.0,
            episode_deadline=20.0,
            map_context=initial,
            observation=observation(1),
            maneuver_state=maneuver.state(1),
            fact_values=facts,
        )
        fresh = copy.deepcopy(initial)
        fresh["map_version"] += 1
        fresh["action_feasibility"]["active_scan"]["allowed"] = True

        self.assertIsNone(tail.next_action(
            now_monotonic=0.1,
            map_context=fresh,
            observation=observation(2),
            maneuver_state=maneuver.state(1),
            fact_values=facts,
            localization_valid=True,
        ))
        self.assertIn(
            "plan_tail_scan_staging_complete",
            tail.cancelled_reason,
        )

    def test_route_start_requires_completed_bilateral_scan(self):
        memory = self._memory()
        memory.ingest_stationary_observation(
            observation(2, blocked=True),
            1_002,
        )
        target = memory.hazard_map.hazard_ids[0]
        proposal = {
            "id": "route-a",
            "revision": 1,
            "transition": "START",
            "objective": "Pass the remembered obstacle",
            "target_hypothesis_id": target,
            "detour_side": "LEFT_OF_GOAL",
            "success_fact_keys": [
                FACT_GOAL_CORRIDOR_CLEAR,
                FACT_GOAL_HEADING_ALIGNED,
                FACT_TARGET_BEHIND,
            ],
            "current_focus_fact_key": FACT_GOAL_CORRIDOR_CLEAR,
            "revision_reason": None,
        }
        commitment = ManeuverCommitment()
        with self.assertRaises(ManeuverCommitmentError) as caught:
            commitment.apply(
                proposal,
                action=ADVANCE,
                turn=1,
                hazard_map=memory.hazard_map,
                pose=memory.pose,
                fact_values={},
            )
        self.assertEqual(caught.exception.code, "route_evidence_required")

        scan_basis = memory.hazard_map.revision
        memory.ingest_stationary_observation(
            observation(3, blocked=True),
            2_001,
        )
        rays = tuple(
            ActiveIrRay(
                ordinal=index,
                requested_relative_bearing_mdeg=bearing,
                actual_relative_bearing_mdeg=bearing,
                observed_at_ms=1_900 + index,
                state_version=3 + index,
                raw=60 if clear else 20,
                filtered=60 if clear else 20,
                blocked=not clear,
            )
            for index, (bearing, clear) in enumerate(
                ((-30_000, True), (-10_000, False),
                 (10_000, False), (30_000, True)),
                start=1,
            )
        )
        memory.hazard_map.record_scan_result(
            ActiveIrScanResult(
                scan_id="route-ready-scan",
                target_hypothesis_id=target,
                frame_id=memory.frame_id,
                map_generation_id=memory.generation_id,
                based_on_map_version=scan_basis,
                started_at_ms=1_900,
                completed_at_ms=2_000,
                status="COMPLETED",
                reason="bilateral_boundaries_observed",
                stop_confirmed=True,
                restored_start_heading=True,
                rays=rays,
                left_boundary_mdeg=20_000,
                right_boundary_mdeg=-20_000,
            ),
            scan_pose=memory.pose,
        )
        with self.assertRaises(ManeuverCommitmentError) as caught:
            commitment.apply(
                proposal,
                action=ADVANCE,
                turn=1,
                hazard_map=memory.hazard_map,
                pose=memory.pose,
                fact_values={},
            )
        self.assertEqual(
            caught.exception.code,
            "route_authorization_requires_observe",
        )
        state = commitment.apply(
            proposal,
            action=OBSERVE,
            turn=1,
            hazard_map=memory.hazard_map,
            pose=memory.pose,
            fact_values={},
        )
        self.assertEqual(
            state["active"]["target_hypothesis_id"],
            target,
        )
        revised = {
            **proposal,
            "revision": 2,
            "transition": "REVISE",
            "detour_side": "RIGHT_OF_GOAL",
            "revision_reason": "The other side is now preferable",
        }
        with self.assertRaises(ManeuverCommitmentError) as caught:
            commitment.apply(
                revised,
                action=TURN_RIGHT_90,
                turn=2,
                hazard_map=memory.hazard_map,
                pose=memory.pose,
                fact_values={},
            )
        self.assertEqual(
            caught.exception.code,
            "route_revision_requires_observe",
        )
        abandoned = {
            **proposal,
            "transition": "ABANDON",
            "current_focus_fact_key": None,
            "revision_reason": "The hypothesis no longer blocks the goal",
        }
        with self.assertRaises(ManeuverCommitmentError) as caught:
            commitment.apply(
                abandoned,
                action=TURN_LEFT_90,
                turn=2,
                hazard_map=memory.hazard_map,
                pose=memory.pose,
                fact_values={},
            )
        self.assertEqual(
            caught.exception.code,
            "route_abandon_requires_observe",
        )


class FakeScanRig:
    def __init__(
        self,
        late_final=False,
        touch_after_turn=False,
        evidence_offset_ms=0,
    ):
        self.now = 1_000
        self.evidence_offset_ms = evidence_offset_ms
        self.heading = 0
        self.state_version = 1
        self.read_count = 0
        self.late_final = late_final
        self.touch_after_turn = touch_after_turn

    def turn_relative_mdeg(self, delta, _calibration, _deadline):
        self.now += 50
        self.heading += delta
        return {
            "requested_delta_mdeg": delta,
            "actual_delta_mdeg": delta,
            "completed_at_ms": self.now,
            "stop_confirmed": True,
        }

    def stop(self):
        return {"stop_confirmed": True}

    def read_snapshot(self, _deadline_ms=None):
        self.read_count += 1
        self.now += 10
        if self.late_final and self.heading == 0 and self.read_count > 5:
            self.now += 60_000
        self.state_version += 1
        blocked = abs(self.heading) <= 20_000
        return {
            "state_version": self.state_version,
            "observed_at_ms": self.now + self.evidence_offset_ms,
            "pose_heading_mdeg": self.heading,
            "touch_pressed": (
                self.touch_after_turn and self.heading != 0
            ),
            "motion_fault_latched": False,
            "infrared": {
                "raw": 20 if blocked else 60,
                "filtered": 20 if blocked else 60,
                "blocked": blocked,
            },
        }


class ActiveIrScanTests(unittest.TestCase):
    def test_scan_deadlines_use_monotonic_time_with_epoch_evidence(self):
        rig = FakeScanRig(evidence_offset_ms=999_000)
        request = build_scan_request(
            choice=ModelScanChoice("target-a"),
            frame_id="frame-a",
            map_generation_id="generation-a",
            map_version=3,
            start_pose=PhysicalPose(),
            start_state_version=1,
            created_at_ms=1_000_000,
            deadline_ms=1_029_000,
            created_monotonic_ms=1_000,
            deadline_monotonic_ms=30_000,
            calibration=ActiveIrScanCalibration(
                estimated_turn_ms_per_degree=2
            ),
        )

        result = ActiveIrScanExecutor(
            rig=rig,
            clock_ms=lambda: rig.now,
        ).execute(request)

        self.assertTrue(result.bilateral_complete)
        self.assertGreaterEqual(result.started_at_ms, request.created_at_ms)
        self.assertLessEqual(result.completed_at_ms, request.deadline_ms)
        validate_scan_result(
            result,
            request,
            current_frame_id="frame-a",
            current_map_generation_id="generation-a",
            current_map_version=3,
        )

    def test_late_result_assembly_preserves_verified_restoration(self):
        rig = FakeScanRig()
        final_clock_calls = [0]

        def clock():
            if rig.heading == 0 and rig.read_count >= 8:
                final_clock_calls[0] += 1
                if final_clock_calls[0] == 2:
                    rig.now = 30_001
            return rig.now

        request = build_scan_request(
            choice=ModelScanChoice("target-a"),
            frame_id="frame-a",
            map_generation_id="generation-a",
            map_version=3,
            start_pose=PhysicalPose(),
            start_state_version=1,
            created_at_ms=1_000,
            deadline_ms=30_000,
            calibration=ActiveIrScanCalibration(
                estimated_turn_ms_per_degree=2
            ),
        )

        result = ActiveIrScanExecutor(
            rig=rig,
            clock_ms=clock,
        ).execute(request)

        self.assertEqual(result.status, "CANCELLED")
        self.assertEqual(result.reason, "scan_deadline_exceeded")
        self.assertTrue(result.stop_confirmed)
        self.assertTrue(result.restored_start_heading)
        self.assertLessEqual(result.completed_at_ms, request.deadline_ms)
        validate_scan_result(
            result,
            request,
            current_frame_id="frame-a",
            current_map_generation_id="generation-a",
            current_map_version=3,
        )

    def test_model_cannot_choose_bearings_and_scan_finds_both_boundaries(self):
        with self.assertRaises(Exception):
            ModelScanChoice.from_mapping(
                {
                    "tool": "SCAN_FRONT_ARC",
                    "target_hypothesis_id": "target-a",
                    "policy": "ADAPTIVE_COARSE_TO_FINE",
                    "bearing_mdeg": 30_000,
                }
            )
        rig = FakeScanRig()
        request = build_scan_request(
            choice=ModelScanChoice("target-a"),
            frame_id="frame-a",
            map_generation_id="generation-a",
            map_version=3,
            start_pose=PhysicalPose(),
            start_state_version=1,
            created_at_ms=1_000,
            deadline_ms=30_000,
            calibration=ActiveIrScanCalibration(
                estimated_turn_ms_per_degree=2
            ),
        )
        result = ActiveIrScanExecutor(
            rig=rig,
            clock_ms=lambda: rig.now,
        ).execute(request)
        checked = validate_scan_result(
            result,
            request,
            current_frame_id="frame-a",
            current_map_generation_id="generation-a",
            current_map_version=3,
        )
        self.assertTrue(checked.bilateral_complete)
        self.assertGreater(checked.left_boundary_mdeg, 0)
        self.assertLess(checked.right_boundary_mdeg, 0)
        self.assertEqual(rig.heading, 0)

    def test_late_final_snapshot_fails_closed(self):
        rig = FakeScanRig(late_final=True)
        request = build_scan_request(
            choice=ModelScanChoice("target-a"),
            frame_id="frame-a",
            map_generation_id="generation-a",
            map_version=3,
            start_pose=PhysicalPose(),
            start_state_version=1,
            created_at_ms=1_000,
            deadline_ms=30_000,
            calibration=ActiveIrScanCalibration(
                estimated_turn_ms_per_degree=2
            ),
        )
        result = ActiveIrScanExecutor(
            rig=rig,
            clock_ms=lambda: rig.now,
        ).execute(request)
        self.assertEqual(result.status, "CANCELLED")
        self.assertFalse(result.restored_start_heading)

    def test_restoration_preserves_the_worker_failure_reason(self):
        class RestorationFailureRig(FakeScanRig):
            def turn_relative_mdeg(
                self,
                delta,
                calibration,
                deadline,
            ):
                if self.heading != 0 and self.heading + delta == 0:
                    error = ActiveIrScanContractError(
                        "scan_worker_error",
                        "worker rejected restoration",
                    )
                    error.result_reason = (
                        "scan_worker_error:encoder_recovery_exhausted"
                    )
                    raise error
                return super().turn_relative_mdeg(
                    delta,
                    calibration,
                    deadline,
                )

        rig = RestorationFailureRig()
        request = build_scan_request(
            choice=ModelScanChoice("target-a"),
            frame_id="frame-a",
            map_generation_id="generation-a",
            map_version=3,
            start_pose=PhysicalPose(),
            start_state_version=1,
            created_at_ms=1_000,
            deadline_ms=30_000,
            calibration=ActiveIrScanCalibration(
                estimated_turn_ms_per_degree=2
            ),
        )

        result = ActiveIrScanExecutor(
            rig=rig,
            clock_ms=lambda: rig.now,
        ).execute(request)

        self.assertEqual(result.status, "CANCELLED")
        self.assertEqual(
            result.reason,
            "scan_worker_error:encoder_recovery_exhausted",
        )
        self.assertTrue(result.stop_confirmed)
        self.assertFalse(result.restored_start_heading)


class EV3NavigationTransportTests(unittest.TestCase):
    def test_scan_sample_uses_declared_filter_tail_not_whole_batch(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )
        current = observation(2, blocked=True)
        current["infrared"].update(
            raw=33,
            filtered=34,
            reason="blocked_hysteresis_hold",
            sample_count=5,
        )
        response = {
            "state_version": 2,
            "result": {
                "sample_count": 5,
                "raw_samples": [33, 33, 34, 34, 33],
                "started_monotonic_ms": 1_000,
                "completed_monotonic_ms": 1_120,
                "observation": current,
                "stop": stop_proof(),
            },
        }

        transport._validate_success_result("scan_sample", {}, response)

        malformed = copy.deepcopy(response)
        malformed["result"]["observation"]["infrared"]["filtered"] = 33
        with self.assertRaises(EV3NavigationTransportError):
            transport._validate_success_result(
                "scan_sample",
                {},
                malformed,
            )

    def test_worker_error_with_observation_and_stop_is_typed_remote_error(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path="/home/robot/robot-llm/ev3/navigation_worker.py",
        )
        current = observation(4)
        frame = {
            "schema": "ev3-agent-worker-response/v2",
            "controller_id": "ev3-main",
            "request_id": "host-0001",
            "ok": False,
            "state_version": 4,
            "error": {
                "code": "infrared_blocked",
                "message": "Forward path is blocked",
                "fatal": False,
                "observation": current,
                "stop": stop_proof(),
            },
        }
        with self.assertRaises(EV3NavigationRemoteError) as caught:
            transport._validate_response(frame, "host-0001")
        self.assertEqual(caught.exception.code, "infrared_blocked")
        self.assertEqual(caught.exception.observation, current)
        self.assertEqual(caught.exception.stop, stop_proof())

    def test_clean_degraded_pulse_is_strictly_validated(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )
        result = degraded_motion_result()
        response = {"state_version": 2, "result": result}

        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            response,
        )

        malformed = copy.deepcopy(response)
        malformed["result"]["outcome"]["slices"][0]["motors"][0][
            "position_delta"
        ] = 173
        with self.assertRaises(EV3NavigationTransportError):
            transport._validate_success_result(
                "pulse",
                {"action": ADVANCE},
                malformed,
            )

    def test_partial_start_requires_complete_terminal_failure_evidence(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )
        result = partial_start_motion_result()
        response = {"state_version": 2, "result": result}

        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            response,
        )

        post_write_failure = partial_start_motion_result(
            left_delta=24,
            right_delta=19,
            commanded_sides=("left", "right"),
        )
        post_write_outcome = post_write_failure["outcome"]
        post_write_receipt = post_write_outcome["slices"][0]
        post_write_outcome["reason"] = "clock_rollback"
        post_write_receipt["reason"] = "clock_rollback"
        post_write_receipt["segments"][0][
            "reason"
        ] = "clock_rollback"
        post_write_failure["observation"][
            "last_outcome"
        ] = post_write_outcome
        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            {"state_version": 2, "result": post_write_failure},
        )

        recovery_tail = recovered_motion_result()
        tail_outcome = recovery_tail["outcome"]
        tail_receipt = tail_outcome["slices"][0]
        tail_segment = tail_receipt["segments"][1]
        tail_reason = "cancel_requested"
        tail_segment["kind"] = "partial_start"
        tail_segment["commanded_sides"] = ["right"]
        tail_segment["status"] = "interrupted"
        tail_segment["reason"] = tail_reason
        tail_segment["encoder_verification"] = {
            "passed": False,
            "error": "paired motor start was incomplete",
            "checks": [
                {
                    "role": "left_drive",
                    "side": "left",
                    "position_delta": 0,
                    "passed": False,
                },
                {
                    "role": "right_drive",
                    "side": "right",
                    "position_delta": 75,
                    "passed": True,
                },
                {
                    "role": "paired_start_completion",
                    "side": "paired",
                    "position_delta": 0,
                    "passed": False,
                },
            ],
        }
        tail_receipt["status"] = "interrupted"
        tail_receipt["reason"] = tail_reason
        tail_receipt["encoder_verification"] = {
            "passed": False,
            "error": "encoder recovery did not satisfy paired motion",
            "checks": [{"passed": False}],
        }
        tail_outcome["status"] = "interrupted"
        tail_outcome["reason"] = tail_reason
        tail_outcome["completed_slice_count"] = 0
        tail_outcome["encoder_verification"] = {
            "passed": False,
            "verified_slice_count": 0,
            "requested_slice_count": 1,
        }
        recovery_tail["observation"]["last_outcome"] = tail_outcome
        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            {"state_version": 2, "result": recovery_tail},
        )

        invalid_responses = []
        for commanded_sides in (
            [],
            ["left", "left"],
            ["right", "left"],
            ["right"],
        ):
            malformed = copy.deepcopy(response)
            malformed["result"]["outcome"]["slices"][0]["segments"][
                0
            ]["commanded_sides"] = commanded_sides
            invalid_responses.append(malformed)

        completed = copy.deepcopy(response)
        completed["result"]["outcome"]["slices"][0]["segments"][0][
            "status"
        ] = "completed"
        invalid_responses.append(completed)

        passed = copy.deepcopy(response)
        segment_proof = passed["result"]["outcome"]["slices"][0][
            "segments"
        ][0]["encoder_verification"]
        segment_proof["passed"] = True
        segment_proof["error"] = None
        for check in segment_proof["checks"]:
            check["passed"] = True
        invalid_responses.append(passed)

        incomplete = copy.deepcopy(response)
        incomplete["result"]["outcome"]["slices"][0]["segments"][0][
            "motors"
        ].pop()
        invalid_responses.append(incomplete)

        dirty_stop = copy.deepcopy(response)
        dirty_stop["result"]["outcome"]["slices"][0]["segments"][0][
            "stop"
        ]["errors"] = ["cleanup failed"]
        invalid_responses.append(dirty_stop)

        denied_with_segment = copy.deepcopy(response)
        denied_outcome = denied_with_segment["result"]["outcome"]
        denied_receipt = denied_outcome["slices"][0]
        denied_receipt["status"] = "denied"
        denied_receipt["started_monotonic_ms"] = None
        denied_receipt["motors"] = []
        denied_receipt["encoder_verification"] = {
            "passed": False,
            "error": None,
            "checks": [],
        }
        denied_outcome["status"] = "denied"
        denied_outcome["started_monotonic_ms"] = None
        denied_with_segment["result"]["observation"][
            "last_outcome"
        ] = denied_outcome
        invalid_responses.append(denied_with_segment)

        for malformed in invalid_responses:
            with self.subTest(malformed=malformed):
                with self.assertRaises(EV3NavigationTransportError):
                    transport._validate_success_result(
                        "pulse",
                        {"action": ADVANCE},
                        malformed,
                    )

    def test_interrupted_pulse_can_retain_successful_encoder_evidence(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )
        result = degraded_motion_result(
            left_delta=80,
            right_delta=78,
        )
        receipt = result["outcome"]["slices"][0]
        receipt["status"] = "interrupted"
        receipt["reason"] = "touch_pressed"
        receipt["encoder_verification"] = {
            "passed": True,
            "error": None,
            "checks": [{"passed": True}, {"passed": True}],
        }
        segment = receipt["segments"][0]
        segment["status"] = receipt["status"]
        segment["reason"] = receipt["reason"]
        segment["encoder_verification"] = copy.deepcopy(
            receipt["encoder_verification"]
        )
        outcome = result["outcome"]
        outcome["status"] = "interrupted"
        outcome["reason"] = "touch_pressed"
        outcome["encoder_verification"] = {
            "passed": True,
            "verified_slice_count": 1,
            "requested_slice_count": 1,
        }
        result["observation"]["last_outcome"] = outcome

        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            {"state_version": 2, "result": result},
        )

    def test_completed_pulse_requires_real_profile_encoder_evidence(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )
        result = degraded_motion_result(
            left_delta=200,
            right_delta=200,
        )
        receipt = result["outcome"]["slices"][0]
        receipt["status"] = "completed"
        receipt["reason"] = "duration_elapsed"
        receipt["encoder_verification"] = {
            "passed": True,
            "error": None,
            "checks": [{"passed": True}, {"passed": True}],
        }
        segment = receipt["segments"][0]
        segment["status"] = receipt["status"]
        segment["reason"] = receipt["reason"]
        segment["encoder_verification"] = copy.deepcopy(
            receipt["encoder_verification"]
        )
        outcome = result["outcome"]
        outcome["status"] = "completed"
        outcome["reason"] = "semantic_action_completed"
        outcome["completed_slice_count"] = 1
        outcome["encoder_verification"] = {
            "passed": True,
            "verified_slice_count": 1,
            "requested_slice_count": 1,
        }
        result["observation"]["last_outcome"] = outcome
        response = {"state_version": 2, "result": result}

        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            response,
        )

        invalid_results = []
        missing = copy.deepcopy(response)
        missing_receipt = missing["result"]["outcome"]["slices"][0]
        missing_receipt["started_monotonic_ms"] = None
        missing_receipt["motors"] = []
        missing_receipt["encoder_verification"]["checks"] = []
        invalid_results.append(missing)

        wrong_duration = copy.deepcopy(response)
        wrong_duration["result"]["outcome"]["slices"][0][
            "duration_ms"
        ] = 1
        invalid_results.append(wrong_duration)

        duplicate_role = copy.deepcopy(response)
        duplicate_role["result"]["outcome"]["slices"][0]["motors"][1][
            "role"
        ] = "left_drive"
        invalid_results.append(duplicate_role)

        for malformed in invalid_results:
            with self.subTest(malformed=malformed):
                with self.assertRaises(EV3NavigationTransportError):
                    transport._validate_success_result(
                        "pulse",
                        {"action": ADVANCE},
                        malformed,
                    )

    def test_recovered_pulse_validates_temporal_encoder_continuity(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )
        result = recovered_motion_result()
        response = {"state_version": 2, "result": result}

        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            response,
        )

        settled = copy.deepcopy(response)
        settled_motors = settled["result"]["observation"]["motors"]
        settled_motors[0]["position"] += 5
        settled_motors[1]["position"] -= 5
        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            settled,
        )

        excessive = copy.deepcopy(response)
        excessive["result"]["observation"]["motors"][0][
            "position"
        ] += 6
        with self.assertRaises(EV3NavigationTransportError):
            transport._validate_success_result(
                "pulse",
                {"action": ADVANCE},
                excessive,
            )

        active = copy.deepcopy(response)
        active["result"]["observation"]["motors"][0]["state"] = (
            "running"
        )
        with self.assertRaises(EV3NavigationTransportError):
            transport._validate_success_result(
                "pulse",
                {"action": ADVANCE},
                active,
            )

        reversed_net = degraded_motion_result(
            left_delta=1,
            right_delta=0,
        )
        reversed_net["observation"]["motors"][0]["position"] = -1
        with self.assertRaises(EV3NavigationTransportError):
            transport._validate_success_result(
                "pulse",
                {"action": ADVANCE},
                {"state_version": 2, "result": reversed_net},
            )

        backlash = copy.deepcopy(response)
        backlash_result = backlash["result"]
        backlash_receipt = backlash_result["outcome"]["slices"][0]
        uncommanded = backlash_receipt["segments"][1]["motors"][0]
        uncommanded["position_after"] = 74
        uncommanded["position_delta"] = -1
        aggregate_left = backlash_receipt["motors"][0]
        aggregate_left["position_after"] = 74
        aggregate_left["position_delta"] = 74
        backlash_result["observation"]["motors"][0]["position"] = 74
        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            backlash,
        )

        malformed = copy.deepcopy(response)
        malformed["result"]["outcome"]["slices"][0]["segments"][1][
            "motors"
        ][0]["position_before"] = 74
        malformed["result"]["outcome"]["slices"][0]["segments"][1][
            "motors"
        ][0]["position_delta"] = 1
        with self.assertRaises(EV3NavigationTransportError):
            transport._validate_success_result(
                "pulse",
                {"action": ADVANCE},
                malformed,
            )

    def test_recovery_start_denial_accepts_newer_clean_outer_stop(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )
        result = degraded_motion_result()
        outcome = result["outcome"]
        receipt = outcome["slices"][0]
        receipt["status"] = "interrupted"
        receipt["reason"] = "infrared_blocked"
        outcome["status"] = receipt["status"]
        outcome["reason"] = receipt["reason"]
        cleanup_stop = stop_proof()
        cleanup_stop["positions"] = {"right_drive": 0}
        cleanup_stop["states"] = {"right_drive": ""}
        receipt["stop"] = cleanup_stop
        result["stop"] = cleanup_stop
        result["observation"]["last_outcome"] = outcome

        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            {"state_version": 2, "result": result},
        )

        completed = recovered_motion_result()
        completed_receipt = completed["outcome"]["slices"][0]
        completed_receipt["stop"] = cleanup_stop
        completed["stop"] = cleanup_stop
        completed["observation"]["last_outcome"] = completed["outcome"]
        with self.assertRaises(EV3NavigationTransportError):
            transport._validate_success_result(
                "pulse",
                {"action": ADVANCE},
                {"state_version": 2, "result": completed},
            )

    def test_wrong_direction_terminal_failure_remains_valid_evidence(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )
        result = degraded_motion_result(left_delta=-20, right_delta=0)
        outcome = result["outcome"]
        receipt = outcome["slices"][0]
        outcome["reason"] = "encoder_verification_failed"
        receipt["reason"] = outcome["reason"]
        receipt["segments"][0]["reason"] = outcome["reason"]
        result["observation"]["budgets"][
            "motion_fault_latched"
        ] = True
        result["observation"]["last_outcome"] = outcome

        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            {"state_version": 2, "result": result},
        )


class FakeRuntimeTransport:
    def __init__(
        self,
        *,
        process_ms_remaining=40_000,
        pulse_count_remaining=40,
        pulse_duration_ms_remaining=32_000,
        blocked=False,
    ):
        self.shutdown_complete = False
        self.version = 1
        self.left = 0
        self.right = 0
        self.calls = []
        self.process_ms_remaining = process_ms_remaining
        self.pulse_count_remaining = pulse_count_remaining
        self.pulse_duration_ms_remaining = pulse_duration_ms_remaining
        self.blocked = blocked

    def start(self):
        self.calls.append(("start", None))

    def _observation(self, last_outcome=None):
        return observation(
            self.version,
            left_position=self.left,
            right_position=self.right,
            left_role="drive_b",
            right_role="drive_c",
            last_outcome=last_outcome,
            process_ms_remaining=self.process_ms_remaining,
            pulse_count_remaining=self.pulse_count_remaining,
            pulse_duration_ms_remaining=(
                self.pulse_duration_ms_remaining
            ),
            blocked=self.blocked,
        )

    def request(
        self,
        operation,
        arguments,
        timeout,
        cancel_requested=None,
    ):
        self.calls.append((operation, copy.deepcopy(arguments)))
        if operation == "describe":
            return {
                "state_version": self.version,
                "result": {
                    "worker_id": "navigation-worker-v1",
                    "demo_only": True,
                    "policy_owner": "host",
                    "controller_id": "ev3-main",
                    "request_schema": "ev3-agent-worker-request/v1",
                    "response_schema": "ev3-agent-worker-response/v2",
                    "operations": [
                        "describe",
                        "observe",
                        "pulse",
                        "scan_turn",
                        "scan_sample",
                        "stop",
                        "shutdown",
                    ],
                    "pulse": {
                        "actions": copy.deepcopy(EXPECTED_ACTION_SPECS),
                        "max_pulses": 40,
                        "max_total_duration_ms": 32_000,
                    },
                    "safety": {
                        **copy.deepcopy(EXPECTED_WORKER_SAFETY),
                    },
                    "scan_turn": expected_scan_turn_profile(),
                    "scan_sample": expected_scan_sample_profile(),
                    "process": {
                        "absolute_max_ms": 45_000,
                        "max_requests": 256,
                    },
                    "drive_geometry": {
                        "left_motor_role": "drive_b",
                        "right_motor_role": "drive_c",
                        "forward_speed_sign": {
                            "drive_b": 1,
                            "drive_c": 1,
                        },
                    },
                    "observation": self._observation(),
                },
            }
        if operation == "pulse":
            self.version += 1
            before_left = self.left
            before_right = self.right
            self.left += 200
            self.right += 200
            outcome = {
                "kind": "pulse",
                "action": arguments["action"],
                "status": "completed",
                "reason": "semantic_action_completed",
                "started_monotonic_ms": 1,
                "completed_monotonic_ms": 2,
                "stop_confirmed": True,
                "requested_slice_count": 1,
                "completed_slice_count": 1,
                "slices": [
                    {
                        "slice_index": 1,
                        "slice_count": 1,
                        "duration_ms": 250,
                        "status": "completed",
                        "reason": "duration_elapsed",
                        "started_monotonic_ms": 1,
                        "completed_monotonic_ms": 2,
                        "motors": [
                            {
                                "side": "left",
                                "role": "drive_b",
                                "position_before": before_left,
                                "position_after": self.left,
                                "position_delta": 200,
                                "state": "",
                            },
                            {
                                "side": "right",
                                "role": "drive_c",
                                "position_before": before_right,
                                "position_after": self.right,
                                "position_delta": 200,
                                "state": "",
                            },
                        ],
                        "encoder_verification": {
                            "passed": True,
                            "error": None,
                            "checks": [],
                        },
                        "stop": {
                            "stop_confirmed": True,
                            "errors": [],
                            "fault_tokens": {},
                        },
                    }
                ],
                "encoder_verification": {
                    "passed": True,
                    "verified_slice_count": 1,
                    "requested_slice_count": 1,
                },
            }
            result = {
                "action": arguments["action"],
                "outcome": outcome,
                "observation": self._observation(outcome),
                "stop": {
                    "stop_confirmed": True,
                    "errors": [],
                    "fault_tokens": {},
                },
            }
            return {
                "state_version": self.version,
                "result": result,
            }
        if operation == "observe":
            self.version += 1
            return {
                "state_version": self.version,
                "result": {"observation": self._observation()},
            }
        if operation == "shutdown":
            self.shutdown_complete = True
            return {
                "state_version": self.version + 1,
                "result": {
                    "outcome": {
                        "kind": "shutdown",
                        "status": "completed",
                        "completed_monotonic_ms": 5,
                        "stop_confirmed": True,
                        "motor_owner_closed": True,
                    }
                },
            }
        raise AssertionError(operation)

    def close(self):
        self.calls.append(("close", None))


class BlockingPulseTransport(FakeRuntimeTransport):
    def __init__(self):
        super().__init__()
        self.pulse_entered = threading.Event()
        self.cancel_observed = False

    def request(
        self,
        operation,
        arguments,
        timeout,
        cancel_requested=None,
    ):
        if operation != "pulse":
            return super().request(
                operation,
                arguments,
                timeout,
                cancel_requested=cancel_requested,
            )
        self.calls.append((operation, copy.deepcopy(arguments)))
        if not callable(cancel_requested):
            raise AssertionError("pulse request lacked cancellation callback")
        self.pulse_entered.set()
        while not cancel_requested():
            threading.Event().wait(0.002)
        self.cancel_observed = True
        raise RuntimeError("simulated SSH cancellation")


class DecisionRelevantChangeTransport(FakeRuntimeTransport):
    """Change blocked truth only on the explicit post-scan OBSERVE."""

    def __init__(self):
        super().__init__(blocked=True)
        self.observe_calls = 0

    def request(
        self,
        operation,
        arguments,
        timeout,
        cancel_requested=None,
    ):
        if operation == "observe":
            self.observe_calls += 1
            if self.observe_calls == 2:
                self.blocked = False
        return super().request(
            operation,
            arguments,
            timeout,
            cancel_requested=cancel_requested,
        )


class DegradedFirstPulseTransport(FakeRuntimeTransport):
    def __init__(self):
        super().__init__()
        self.degraded_sent = False

    def request(
        self,
        operation,
        arguments,
        timeout,
        cancel_requested=None,
    ):
        if operation != "pulse" or self.degraded_sent:
            return super().request(
                operation,
                arguments,
                timeout,
                cancel_requested=cancel_requested,
            )
        self.calls.append((operation, copy.deepcopy(arguments)))
        self.degraded_sent = True
        self.version += 1
        self.left += 174
        result = degraded_motion_result(
            action=arguments["action"],
            left_delta=174,
            right_delta=0,
            version=self.version,
            left_role="drive_b",
            right_role="drive_c",
        )
        return {
            "state_version": self.version,
            "result": result,
        }


class FakeRuntimePlanner:
    def __init__(self):
        self.calls = 0

    def decide(self, **context):
        self.calls += 1
        if self.calls == 1:
            value = decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=ADVANCE,
                plan=[ADVANCE, ADVANCE],
                reason_code="PROGRESS_GOAL",
            )
        else:
            self.assert_completed = context["mission"]["completed"]
            value = decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=FINISH,
                plan=[FINISH],
                reason_code="COMPLETE_GOAL",
            )
        return NavigationDecision.from_mapping(
            value,
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=(),
        )


class DegradedFeedbackPlanner:
    def __init__(self):
        self.calls = 0
        self.feedback = None

    def decide(self, **context):
        self.calls += 1
        if self.calls == 1:
            value = decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=ADVANCE,
                plan=[ADVANCE, ADVANCE],
                reason_code="PROGRESS_GOAL",
            )
        else:
            self.feedback = copy.deepcopy(context["last_tool_result"])
            value = decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=OBSERVE,
                plan=[OBSERVE],
                reason_code="VERIFY_RESULT",
            )
        return NavigationDecision.from_mapping(
            value,
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=(),
        )


class SpeechRuntimePlanner(FakeRuntimePlanner):
    def decide(self, **context):
        decision = super().decide(**context)
        if self.calls != 1:
            return decision
        value = decision.to_dict()
        value["utterance"] = "Jag rullar medan jag pratar."
        return NavigationDecision.from_mapping(
            value,
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=(),
        )


class InvalidThenValidSpeechPlanner:
    def __init__(self):
        self.calls = 0
        self.recent_utterances = []

    def decide(self, **context):
        self.calls += 1
        self.recent_utterances.append(
            tuple(context["recent_committed_utterances"])
        )
        if self.calls == 1:
            value = decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=FINISH,
                plan=[FINISH],
                reason_code="COMPLETE_GOAL",
                utterance="Det här förslaget ska aldrig sägas.",
            )
        else:
            value = decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=ADVANCE,
                plan=[ADVANCE, ADVANCE],
                reason_code="PROGRESS_GOAL",
                utterance="Nu kör jag framåt.",
            )
        return NavigationDecision.from_mapping(
            value,
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=(
                tuple(context["available_actions"]) + (FINISH,)
                if self.calls == 1
                else context["available_actions"]
            ),
            published_target_ids=(),
        )


class CommittedSpeechHistoryPlanner:
    def __init__(self):
        self.calls = 0
        self.recent_utterances = []

    def decide(self, **context):
        self.calls += 1
        self.recent_utterances.append(
            tuple(context["recent_committed_utterances"])
        )
        if self.calls == 1:
            action = ADVANCE
            plan = [ADVANCE, ADVANCE]
            reason_code = "PROGRESS_GOAL"
            utterance = "Jaha, då rullar jag väl."
        else:
            action = FINISH
            plan = [FINISH]
            reason_code = "COMPLETE_GOAL"
            utterance = None
        value = decision_mapping(
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            action=action,
            plan=plan,
            reason_code=reason_code,
            utterance=utterance,
        )
        return NavigationDecision.from_mapping(
            value,
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=(),
        )


class VetoThenObserveSpeechHistoryPlanner:
    def __init__(self):
        self.calls = 0
        self.recent_utterances = []

    def decide(self, **context):
        self.calls += 1
        self.recent_utterances.append(
            tuple(context["recent_committed_utterances"])
        )
        targets = tuple(
            item["hypothesis_id"]
            for item in context["navigation"][
                "navigation_hazard_hypotheses"
            ]
        )
        if self.calls == 1:
            action = ADVANCE
            plan = [ADVANCE, ADVANCE]
            reason_code = "PROGRESS_GOAL"
            utterance = "Jag tänker köra rakt genom skiten."
        else:
            action = OBSERVE
            plan = [OBSERVE]
            reason_code = "VERIFY_RESULT"
            utterance = None
        return NavigationDecision.from_mapping(
            decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=action,
                plan=plan,
                reason_code=reason_code,
                utterance=utterance,
            ),
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=targets,
        )


class InvalidModelOutputThenValidPlanner:
    def __init__(self):
        self.calls = 0
        self.feedback = None

    def decide(self, **context):
        self.calls += 1
        if self.calls == 1:
            raise LMStudioNavigationDecisionError(
                "unexpected_perception_target",
                "Only SCAN_FRONT_ARC may name a perception target",
                latency_ms=17,
            )
        self.feedback = copy.deepcopy(context["validation_feedback"])
        return NavigationDecision.from_mapping(
            decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=OBSERVE,
                plan=[OBSERVE],
                reason_code="VERIFY_RESULT",
            ),
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=(),
        )


class VetoedSpeechPlanner:
    def __init__(self, memory):
        self.memory = memory

    def decide(self, **context):
        # Simulate authoritative perception changing after the planner
        # snapshot but before dispatch. The pre-planner feasibility table
        # cannot predict this; the final execution veto still must catch it.
        self.memory.hazard_map.record_observation(
            self.memory.pose,
            observation(99, blocked=True),
            2_001,
        )
        targets = tuple(
            item["hypothesis_id"]
            for item in context["navigation"][
                "navigation_hazard_hypotheses"
            ]
        )
        return NavigationDecision.from_mapping(
            decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=ADVANCE,
                plan=[ADVANCE, ADVANCE],
                reason_code="PROGRESS_GOAL",
                utterance="Jag kör trots hindret.",
            ),
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=targets,
        )


class CancelDuringPlanner:
    def __init__(self, event):
        self.event = event

    def decide(self, **context):
        self.event.set()
        return NavigationDecision.from_mapping(
            decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=ADVANCE,
                plan=[ADVANCE, ADVANCE],
                reason_code="PROGRESS_GOAL",
            ),
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=(),
        )


class SingleScanPlanner:
    def __init__(self):
        self.available_actions = None

    def decide(self, **context):
        self.available_actions = tuple(context["available_actions"])
        targets = tuple(
            item["hypothesis_id"]
            for item in context["navigation"][
                "navigation_hazard_hypotheses"
            ]
        )
        target = targets[-1]
        return NavigationDecision.from_mapping(
            decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=SCAN_FRONT_ARC,
                plan=[SCAN_FRONT_ARC],
                reason_code="HANDLE_OBSTACLE",
                target=target,
            ),
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=targets,
        )


class ObserveThenScanPlanner:
    def __init__(self):
        self.calls = 0
        self.available_history = []
        self.second_feedback = None

    def decide(self, **context):
        self.calls += 1
        self.available_history.append(tuple(context["available_actions"]))
        targets = tuple(
            item["hypothesis_id"]
            for item in context["navigation"][
                "navigation_hazard_hypotheses"
            ]
        )
        if self.calls == 1:
            action = OBSERVE
            reason = "VERIFY_RESULT"
            target = None
        else:
            self.second_feedback = copy.deepcopy(
                context["last_tool_result"]
            )
            action = SCAN_FRONT_ARC
            reason = "HANDLE_OBSTACLE"
            target = targets[0]
        return NavigationDecision.from_mapping(
            decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=action,
                plan=[action],
                reason_code=reason,
                target=target,
            ),
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=targets,
        )


class CaptureAvailablePlanner:
    def __init__(self):
        self.available_actions = None
        self.navigation = None

    def decide(self, **context):
        self.available_actions = tuple(context["available_actions"])
        self.navigation = copy.deepcopy(context["navigation"])
        return NavigationDecision.from_mapping(
            decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=OBSERVE,
                plan=[OBSERVE],
                reason_code="VERIFY_RESULT",
            ),
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=tuple(
                item["hypothesis_id"]
                for item in context["navigation"][
                    "navigation_hazard_hypotheses"
                ]
            ),
        )


class MapChangesAfterScanPlanningPlanner(SingleScanPlanner):
    def __init__(self, memory):
        super().__init__()
        self.memory = memory

    def decide(self, **context):
        if SCAN_FRONT_ARC not in context["available_actions"]:
            raise AssertionError("scan was not feasible before planning")
        hazard = self.memory.hazard_map.hazards[0]
        self.memory.hazard_map._hazards = (
            replace(hazard, centroid_x_mm=100),
        )
        return super().decide(**context)


class ScanExecutorMustNotRun:
    def execute(self, _request, cancel_requested):
        del cancel_requested
        raise AssertionError("rotation-vetoed scan reached executor")


class RuntimeScanExecutor:
    def __init__(self, transport, *, touch_after_turn=False):
        self.transport = transport
        self.touch_after_turn = touch_after_turn
        self.requests = []

    def execute(self, request, cancel_requested):
        self.requests.append(request)
        rig = FakeScanRig(
            touch_after_turn=self.touch_after_turn,
            evidence_offset_ms=(
                request.created_at_ms - request.created_monotonic_ms
            ),
        )
        rig.now = request.created_monotonic_ms
        result = ActiveIrScanExecutor(
            rig=rig,
            clock_ms=lambda: rig.now,
        ).execute(request, cancel_requested=cancel_requested)
        self.transport.left += 7
        self.transport.right -= 6
        return result


class RestoredCancelledRuntimeScanExecutor:
    def __init__(self, transport):
        self.transport = transport

    def execute(self, request, cancel_requested):
        self.transport.left += 7
        self.transport.right -= 6
        return ActiveIrScanResult(
            scan_id=request.scan_id,
            target_hypothesis_id=request.target_hypothesis_id,
            frame_id=request.frame_id,
            map_generation_id=request.map_generation_id,
            based_on_map_version=request.based_on_map_version,
            started_at_ms=request.created_at_ms,
            completed_at_ms=request.created_at_ms + 1,
            status="CANCELLED",
            reason="scan_deadline_exceeded",
            stop_confirmed=True,
            restored_start_heading=True,
            rays=(),
            left_boundary_mdeg=None,
            right_boundary_mdeg=None,
        )


class RestoredEvidenceRuntimeScanExecutor:
    """Return live-shaped unilateral evidence, then optional all-clear."""

    def __init__(self, transport, *, all_clear_after_first=False):
        self.transport = transport
        self.all_clear_after_first = all_clear_after_first
        self.calls = 0

    def execute(self, request, cancel_requested):
        self.calls += 1
        all_clear = self.all_clear_after_first and self.calls > 1
        samples = (
            (0, 0, not all_clear),
            (-30_000, -28_500, not all_clear),
            (-60_000, -57_500, not all_clear),
            (15_000, 14_250, False),
            (30_000, 28_500, False),
            (60_000, 57_500, False),
        )
        rays = tuple(
            ActiveIrRay(
                ordinal=index,
                requested_relative_bearing_mdeg=requested,
                actual_relative_bearing_mdeg=actual,
                observed_at_ms=request.created_at_ms + index,
                state_version=request.start_state_version + index,
                raw=31 if blocked else 62,
                filtered=32 if blocked else 61,
                blocked=blocked,
            )
            for index, (requested, actual, blocked) in enumerate(
                samples,
                start=1,
            )
        )
        self.transport.left += 7
        self.transport.right -= 6
        return ActiveIrScanResult(
            scan_id=request.scan_id,
            target_hypothesis_id=request.target_hypothesis_id,
            frame_id=request.frame_id,
            map_generation_id=request.map_generation_id,
            based_on_map_version=request.based_on_map_version,
            started_at_ms=request.created_at_ms,
            completed_at_ms=request.created_at_ms + len(rays) + 1,
            status="CANCELLED",
            reason="bilateral_boundaries_not_observed",
            stop_confirmed=True,
            restored_start_heading=True,
            rays=rays,
            left_boundary_mdeg=None if all_clear else 7_500,
            right_boundary_mdeg=None,
        )


class ScanThenObservePlanner(SingleScanPlanner):
    def __init__(self):
        super().__init__()
        self.calls = 0
        self.feedback = None
        self.available_history = []

    def decide(self, **context):
        self.calls += 1
        self.available_history.append(tuple(context["available_actions"]))
        if self.calls == 1:
            return super().decide(**context)
        self.feedback = copy.deepcopy(context["last_tool_result"])
        return NavigationDecision.from_mapping(
            decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=OBSERVE,
                plan=[OBSERVE],
                reason_code="VERIFY_RESULT",
            ),
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=context["available_actions"],
            published_target_ids=tuple(
                item["hypothesis_id"]
                for item in context["navigation"][
                    "navigation_hazard_hypotheses"
                ]
            ),
        )


class ScanObserveChangedThenScanPlanner(SingleScanPlanner):
    def __init__(self):
        super().__init__()
        self.calls = 0
        self.available_history = []

    def decide(self, **context):
        self.calls += 1
        available = tuple(context["available_actions"])
        self.available_history.append(available)
        targets = tuple(
            item["hypothesis_id"]
            for item in context["navigation"][
                "navigation_hazard_hypotheses"
            ]
        )
        wants_scan = self.calls in (1, 3)
        action = (
            SCAN_FRONT_ARC
            if wants_scan and SCAN_FRONT_ARC in available
            else OBSERVE
        )
        return NavigationDecision.from_mapping(
            decision_mapping(
                episode_id=context["episode_id"],
                turn=context["turn"],
                state_version=context["observation"]["state_version"],
                action=action,
                plan=[action],
                reason_code=(
                    "HANDLE_OBSTACLE"
                    if action == SCAN_FRONT_ARC
                    else "VERIFY_RESULT"
                ),
                target=targets[0] if action == SCAN_FRONT_ARC else None,
            ),
            episode_id=context["episode_id"],
            turn=context["turn"],
            state_version=context["observation"]["state_version"],
            available_actions=available,
            published_target_ids=targets,
        )


class PhysicalNavigationRuntimeTests(unittest.TestCase):
    def test_incomplete_mission_does_not_publish_finish_as_available(self):
        class AvailabilityPlanner:
            def __init__(self):
                self.available = None

            def decide(self, **context):
                self.available = context["available_actions"]
                return NavigationDecision.from_mapping(
                    decision_mapping(
                        episode_id=context["episode_id"],
                        turn=context["turn"],
                        state_version=context["observation"]["state_version"],
                        action=OBSERVE,
                        plan=[OBSERVE],
                        reason_code="VERIFY_RESULT",
                    ),
                    episode_id=context["episode_id"],
                    turn=context["turn"],
                    state_version=context["observation"]["state_version"],
                    available_actions=context["available_actions"],
                    published_target_ids=(),
                )

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "incomplete-finish-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
            )
            planner = AvailabilityPlanner()
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-incomplete-finish",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Make forward progress",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=10,
                ),
                transport=FakeRuntimeTransport(),
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
            )

            runtime.run()

        self.assertNotIn(FINISH, planner.available)

    def test_diagnostic_reason_does_not_veto_safe_observe(self):
        class DiagnosticObservePlanner:
            def __init__(self, reason_code):
                self.reason_code = reason_code

            def decide(self, **context):
                return NavigationDecision.from_mapping(
                    decision_mapping(
                        episode_id=context["episode_id"],
                        turn=context["turn"],
                        state_version=context["observation"]["state_version"],
                        action=OBSERVE,
                        plan=[OBSERVE],
                        reason_code=self.reason_code,
                    ),
                    episode_id=context["episode_id"],
                    turn=context["turn"],
                    state_version=context["observation"]["state_version"],
                    available_actions=context["available_actions"],
                    published_target_ids=(),
                )

        for reason_code in ("PROGRESS_GOAL", "COMPLETE_GOAL"):
            with self.subTest(reason_code=reason_code):
                with tempfile.TemporaryDirectory() as directory:
                    memory = NavigationMemoryStore.load(
                        path=Path(directory) / "diagnostic-reason-memory.json",
                        robot_id="ev3rstorm-01",
                        controller_instance_id="ev3-main",
                        reset=True,
                    )
                    events = []
                    runtime = PhysicalNavigationRuntime(
                        episode_id="episode-diagnostic-reason",
                        config=PhysicalNavigationRuntimeConfig(
                            goal="Observe",
                            locale="en",
                            max_turns=1,
                            max_episode_seconds=10,
                        ),
                        transport=FakeRuntimeTransport(),
                        planner=DiagnosticObservePlanner(reason_code),
                        memory=memory,
                        monotonic=lambda: 0.0,
                        event_sink=events.append,
                    )

                    result = runtime.run()

                self.assertEqual(result.actions, (OBSERVE,))
                self.assertFalse(
                    any(
                        event["event"] == "decision_vetoed"
                        for event in events
                    )
                )

    def test_completed_finish_does_not_require_reason_label(self):
        class DiagnosticFinishPlanner(FakeRuntimePlanner):
            def decide(self, **context):
                decision = super().decide(**context)
                if decision.action != FINISH:
                    return decision
                value = decision.to_dict()
                value["reason_code"] = "VERIFY_RESULT"
                return NavigationDecision.from_mapping(
                    value,
                    episode_id=context["episode_id"],
                    turn=context["turn"],
                    state_version=context["observation"]["state_version"],
                    available_actions=context["available_actions"],
                    published_target_ids=(),
                )

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "diagnostic-finish-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
            )
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-diagnostic-finish",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Move forward at least 100 mm",
                    locale="en",
                    minimum_forward_progress_mm=100,
                    max_turns=3,
                    max_episode_seconds=10,
                ),
                transport=FakeRuntimeTransport(),
                planner=DiagnosticFinishPlanner(),
                memory=memory,
                monotonic=lambda: 0.0,
                event_sink=events.append,
            )

            result = runtime.run()

        self.assertTrue(result.completed)
        self.assertEqual(result.actions, (ADVANCE, ADVANCE, FINISH))
        self.assertFalse(
            any(event["event"] == "decision_vetoed" for event in events)
        )

    def test_adapter_carries_planner_context_and_token_telemetry(self):
        update = PhysicalNavigationRuntimeAdapter._dashboard_update({
            "event": "model_decision",
            "model_latency_ms": 321,
            "planner_context_bytes": 88_000,
            "prompt_tokens": 21_000,
            "completion_tokens": 120,
            "total_tokens": 21_120,
        })

        self.assertEqual(update, {
            "model_latency_ms": 321,
            "planner_context_bytes": 88_000,
            "prompt_tokens": 21_000,
            "completion_tokens": 120,
            "total_tokens": 21_120,
        })

    def test_adapter_exposes_model_veto_reason(self):
        update = PhysicalNavigationRuntimeAdapter._dashboard_update({
            "event": "decision_vetoed",
            "validation_feedback": {
                "code": "detour_start_requires_observe",
                "message": "Authorize the route before motion",
            },
        })

        self.assertEqual(update, {
            "current_action": None,
            "plan": [],
            "message": (
                "Decision vetoed: detour_start_requires_observe — "
                "Authorize the route before motion"
            ),
        })

    def test_adapter_exposes_bridge_and_passes_only_its_offer_to_runtime(self):
        class Bridge:
            def offer(self, **_kwargs):
                return True

            def snapshot(self):
                return {"schema": "robot-spatial-map/v1"}

        bridge = Bridge()
        adapter = PhysicalNavigationRuntimeAdapter(
            transport_factory=object,
            planner_factory=lambda _model: object(),
            memory_factory=object,
            spatial_map_bridge=bridge,
        )
        context = SimpleNamespace(
            episode_id="episode-map-bridge",
            request=SimpleNamespace(goal="Observe", locale="en"),
            settings=SimpleNamespace(
                model="test-model",
                max_episode_ms=60_000,
                speech_enabled=False,
            ),
            stop_requested=threading.Event(),
            emergency_stop_requested=threading.Event(),
            publish=lambda _update: None,
        )

        with mock.patch(
            "robot_agent.physical_navigation_adapter."
            "PhysicalNavigationRuntime"
        ) as runtime_type:
            runtime_type.return_value.run.return_value = SimpleNamespace(
                terminal_reason="goal_completed",
                completed=True,
                model_latency_ms=0,
            )
            adapter.run(context)

        offered = runtime_type.call_args.kwargs["observation_sink"]
        self.assertIs(adapter.spatial_map_provider, bridge)
        self.assertIs(adapter.spatial_map_bridge, bridge)
        self.assertIs(offered.__self__, bridge)
        self.assertIs(offered.__func__, bridge.offer.__func__)
        self.assertIsNone(
            adapter._dashboard_update({
                "event": "spatial_map_observation_failed",
                "publication_stage": "motion_result",
                "error_type": "RuntimeError",
            })
        )
        for invalid in (object(), SimpleNamespace(offer=lambda **_kw: True)):
            with self.subTest(invalid=type(invalid).__name__):
                with self.assertRaisesRegex(
                    ValueError,
                    "spatial map bridge is invalid",
                ):
                    PhysicalNavigationRuntimeAdapter(
                        transport_factory=object,
                        planner_factory=lambda _model: object(),
                        memory_factory=object,
                        spatial_map_bridge=invalid,
                    )

    def test_adapter_propagates_profile_specific_scan_timeout(self):
        scan_calibration = ActiveIrScanCalibration(
            alignment_tolerance_mdeg=10_000,
        )
        adapter = PhysicalNavigationRuntimeAdapter(
            transport_factory=object,
            planner_factory=lambda _model: object(),
            memory_factory=object,
            request_timeout_seconds=25.0,
            scan_timeout_seconds=30.0,
            active_scan_calibration=scan_calibration,
        )
        context = SimpleNamespace(
            episode_id="episode-profile-scan-timeout",
            request=SimpleNamespace(goal="Scan obstacle", locale="en"),
            settings=SimpleNamespace(
                model="test-model",
                max_episode_ms=60_000,
                speech_enabled=False,
            ),
            stop_requested=threading.Event(),
            emergency_stop_requested=threading.Event(),
            publish=lambda _update: None,
        )

        with mock.patch(
            "robot_agent.physical_navigation_adapter."
            "PhysicalNavigationRuntime"
        ) as runtime_type:
            runtime_type.return_value.run.return_value = SimpleNamespace(
                terminal_reason="goal_completed",
                completed=True,
                model_latency_ms=0,
            )
            adapter.run(context)

        config = runtime_type.call_args.kwargs["config"]
        self.assertEqual(config.request_timeout_seconds, 25.0)
        self.assertEqual(config.scan_timeout_seconds, 30.0)
        self.assertIs(
            runtime_type.call_args.kwargs["active_scan_calibration"],
            scan_calibration,
        )

    def test_committed_start_and_motion_observations_are_detached_offers(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "map-publication-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=301),
            )
            offers = []

            def offer(**value):
                offers.append({
                    "memory": value["memory"],
                    "episode_id": value["episode_id"],
                    "captured_at_ms": value["captured_at_ms"],
                    "memory_updated_at_ms": value["memory"].updated_at_ms,
                    "map_version": value["memory"].hazard_map.revision,
                    "state_version": value["observation"]["state_version"],
                })
                # The planner and safety gates must retain the original clear
                # observation after a telemetry consumer mutates its copy.
                value["observation"]["infrared"]["blocked"] = True
                return True

            runtime_times = iter((2_001, 2_002, 2_003))
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-map-publication",
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
                unix_ms=lambda: next(runtime_times),
                observation_sink=offer,
            )

            result = runtime.run()

        self.assertTrue(result.completed)
        self.assertEqual(result.actions, (ADVANCE, ADVANCE, FINISH))
        self.assertEqual(
            [item["captured_at_ms"] for item in offers],
            [2_001, 2_002, 2_003],
        )
        self.assertEqual(
            [item["memory_updated_at_ms"] for item in offers],
            [2_001, 2_002, 2_003],
        )
        self.assertEqual(
            [item["map_version"] for item in offers],
            [1, 2, 3],
        )
        self.assertEqual(
            [item["state_version"] for item in offers],
            [1, 2, 3],
        )
        self.assertTrue(all(item["memory"] is memory for item in offers))
        self.assertTrue(all(
            item["episode_id"] == "episode-map-publication"
            for item in offers
        ))

    def test_persisted_anchor_mismatch_publishes_invalid_localization(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "stale-anchor-map.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=303),
            )
            memory.bind_drive_roles(
                DriveMotorRoles(left="drive_b", right="drive_c")
            )
            memory.begin_episode(
                observation(
                    1,
                    left_position=12,
                    right_position=12,
                    left_role="drive_b",
                    right_role="drive_c",
                ),
                1_001,
            )
            offers = []

            def offer(**value):
                offers.append({
                    "localization_valid": value[
                        "memory"
                    ].localization_valid,
                    "map_version": value["memory"].hazard_map.revision,
                    "captured_at_ms": value["captured_at_ms"],
                    "state_version": value["observation"]["state_version"],
                })
                return True

            runtime = PhysicalNavigationRuntime(
                episode_id="episode-stale-anchor-map",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Continue",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=10,
                ),
                transport=FakeRuntimeTransport(),
                planner=FakeRuntimePlanner(),
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                observation_sink=offer,
            )

            with self.assertRaises(NavigationMemoryError):
                runtime.run()

        self.assertFalse(memory.localization_valid)
        self.assertEqual(offers, [{
            "localization_valid": False,
            "map_version": 2,
            "captured_at_ms": 2_000,
            "state_version": 1,
        }])

    def test_stationary_observation_offer_failure_and_rejection_are_advisory(self):
        for disposition in ("failed", "rejected"):
            with self.subTest(disposition=disposition):
                with tempfile.TemporaryDirectory() as directory:
                    memory = NavigationMemoryStore.load(
                        path=Path(directory) / "map-advisory.json",
                        robot_id="ev3rstorm-01",
                        controller_instance_id="ev3-main",
                        reset=True,
                        clock_ms=lambda: 1_000,
                        uuid_factory=lambda: uuid.UUID(int=302),
                    )
                    events = []

                    def offer(**_value):
                        if disposition == "failed":
                            raise RuntimeError("map worker unavailable")
                        return False

                    runtime = PhysicalNavigationRuntime(
                        episode_id="episode-map-advisory",
                        config=PhysicalNavigationRuntimeConfig(
                            goal="Observe",
                            locale="en",
                            max_turns=1,
                            max_episode_seconds=10,
                        ),
                        transport=FakeRuntimeTransport(),
                        planner=CaptureAvailablePlanner(),
                        memory=memory,
                        monotonic=lambda: 0.0,
                        unix_ms=lambda: 2_000,
                        event_sink=events.append,
                        observation_sink=offer,
                    )

                    result = runtime.run()

                self.assertEqual(result.actions, (OBSERVE,))
                event_name = "spatial_map_observation_{}".format(
                    disposition
                )
                telemetry = [
                    event for event in events if event["event"] == event_name
                ]
                self.assertEqual(
                    [event["publication_stage"] for event in telemetry],
                    ["worker_session_started", "stationary_observation"],
                )
                if disposition == "failed":
                    self.assertTrue(all(
                        event["error_type"] == "RuntimeError"
                        for event in telemetry
                    ))

    def test_unchanged_observe_is_removed_until_another_action_progresses(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "observe-liveness.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=304),
            )
            transport = FakeRuntimeTransport(blocked=True)
            planner = ObserveThenScanPlanner()
            runtime_times = iter((1_500, 1_800, 2_000, 10_000))
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-observe-liveness",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect and pass the obstacle",
                    locale="en",
                    max_turns=2,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                active_scan_executor=RuntimeScanExecutor(transport),
                monotonic=lambda: 0.0,
                unix_ms=lambda: next(runtime_times),
            )

            result = runtime.run()

        self.assertEqual(result.actions, (OBSERVE, SCAN_FRONT_ARC))
        self.assertIn(OBSERVE, planner.available_history[0])
        self.assertNotIn(OBSERVE, planner.available_history[1])
        self.assertEqual(
            planner.second_feedback["information_gain"],
            "NONE",
        )
        self.assertEqual(planner.second_feedback["changed_facts"], [])
        self.assertTrue(memory.localization_valid)

    def test_runtime_requires_both_worker_interrupt_capabilities(self):
        response = FakeRuntimeTransport().request("describe", {}, 1.0)
        for capability in (
            "process_signals_interrupt_active_pulses",
            "channel_close_interrupts_active_pulses",
        ):
            for mutation in ("missing", "false"):
                with self.subTest(capability=capability, mutation=mutation):
                    changed = copy.deepcopy(response)
                    if mutation == "missing":
                        del changed["result"]["safety"][capability]
                    else:
                        changed["result"]["safety"][capability] = False
                    with self.assertRaises(PhysicalNavigationRuntimeError):
                        PhysicalNavigationRuntime._description(changed)

    def test_runtime_executes_exact_tail_and_replans_to_finish(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=3),
            )
            transport = FakeRuntimeTransport()
            planner = FakeRuntimePlanner()
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-a",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Move forward at least 100 mm",
                    locale="sv",
                    minimum_forward_progress_mm=100,
                    max_turns=3,
                    max_episode_seconds=10,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
            )
            result = runtime.run()

        self.assertTrue(result.completed)
        self.assertEqual(
            result.actions,
            (ADVANCE, ADVANCE, FINISH),
        )
        self.assertEqual(result.plan_tails_completed, 1)
        self.assertEqual(result.model_calls, 2)
        self.assertTrue(result.shutdown_clean)
        self.assertEqual(planner.calls, 2)
        self.assertTrue(planner.assert_completed)
        pulse_actions = [
            arguments["action"]
            for operation, arguments in transport.calls
            if operation == "pulse"
        ]
        self.assertEqual(pulse_actions, [ADVANCE, ADVANCE])
        self.assertEqual(
            [item for item in transport.calls if item[0] == "shutdown"],
            [("shutdown", {})],
        )

    def test_invalid_model_output_gets_bounded_feedback_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "model-output-retry-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=25),
            )
            planner = InvalidModelOutputThenValidPlanner()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-model-output-retry",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Observe and correct an invalid proposal",
                    locale="en",
                    minimum_forward_progress_mm=100,
                    max_turns=1,
                    max_episode_seconds=10,
                ),
                transport=FakeRuntimeTransport(),
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=lambda event: events.append(dict(event)),
            )

            result = runtime.run()

        self.assertEqual(result.actions, (OBSERVE,))
        self.assertEqual(result.model_calls, 2)
        self.assertEqual(result.model_latency_ms, 17)
        self.assertEqual(planner.calls, 2)
        self.assertEqual(
            planner.feedback,
            {
                "code": "unexpected_perception_target",
                "message": (
                    "Only SCAN_FRONT_ARC may name a perception target"
                ),
                "host_selected_alternative_action": False,
            },
        )
        veto = next(
            event for event in events if event["event"] == "decision_vetoed"
        )
        self.assertEqual(veto["attempt"], 1)
        self.assertEqual(veto["validation_feedback"], planner.feedback)

    def test_transport_planner_error_is_not_treated_as_model_feedback(self):
        class TransportFailurePlanner:
            def __init__(self):
                self.calls = 0

            def decide(self, **_context):
                self.calls += 1
                raise LMStudioNavigationError(
                    "LM Studio navigation request failed: offline",
                    code="planner_transport_failed",
                    latency_ms=23,
                )

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "transport-failure-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=26),
            )
            planner = TransportFailurePlanner()
            transport = FakeRuntimeTransport()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-transport-failure",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Observe",
                    locale="en",
                    minimum_forward_progress_mm=100,
                    max_turns=1,
                    max_episode_seconds=10,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            result = runtime.run()

        self.assertEqual(planner.calls, 2)
        self.assertEqual(result.terminal_reason, "planner_unavailable")
        self.assertFalse(result.completed)
        self.assertTrue(result.shutdown_clean)
        self.assertEqual(result.model_calls, 2)
        self.assertEqual(result.model_latency_ms, 46)
        self.assertEqual(result.actions, ())
        self.assertEqual(
            [operation for operation, _arguments in transport.calls],
            ["start", "describe", "shutdown", "close"],
        )
        termination = next(
            event
            for event in events
            if event["event"] == "planner_termination"
        )
        self.assertEqual(
            termination["terminal_reason"],
            "planner_unavailable",
        )
        self.assertEqual(
            termination["planner_error_code"],
            "planner_transport_failed",
        )
        self.assertEqual(
            [
                event["attempt"]
                for event in events
                if event["event"] == "planner_attempt_failed"
            ],
            [1, 2],
        )
        self.assertFalse(
            any(event["event"] == "decision_vetoed" for event in events)
        )

    def test_single_planner_timeout_retries_then_dispatches_valid_decision(self):
        class TimeoutThenObservePlanner:
            def __init__(self):
                self.calls = 0
                self.retry_feedback = object()

            def decide(self, **context):
                self.calls += 1
                if self.calls == 1:
                    raise LMStudioNavigationError(
                        "LM Studio navigation request failed: timed out",
                        code="planner_transport_failed",
                        latency_ms=19,
                    )
                self.retry_feedback = context["validation_feedback"]
                return NavigationDecision.from_mapping(
                    decision_mapping(
                        episode_id=context["episode_id"],
                        turn=context["turn"],
                        state_version=context["observation"]["state_version"],
                        action=OBSERVE,
                        plan=[OBSERVE],
                        reason_code="VERIFY_RESULT",
                    ),
                    episode_id=context["episode_id"],
                    turn=context["turn"],
                    state_version=context["observation"]["state_version"],
                    available_actions=context["available_actions"],
                    published_target_ids=(),
                )

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "planner-timeout-retry-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=260),
            )
            planner = TimeoutThenObservePlanner()
            transport = FakeRuntimeTransport()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-planner-timeout-retry",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Observe",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=10,
                    max_validation_attempts=2,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            result = runtime.run()

        self.assertEqual(planner.calls, 2)
        self.assertIsNone(planner.retry_feedback)
        self.assertEqual(result.actions, (OBSERVE,))
        self.assertEqual(result.model_calls, 2)
        self.assertEqual(result.model_latency_ms, 19)
        self.assertTrue(result.shutdown_clean)
        self.assertEqual(
            [operation for operation, _arguments in transport.calls],
            ["start", "describe", "observe", "shutdown", "close"],
        )
        self.assertEqual(
            [
                event["attempt"]
                for event in events
                if event["event"] == "planner_attempt_failed"
            ],
            [1],
        )
        self.assertFalse(
            any(event["event"] == "planner_termination" for event in events)
        )

    def test_exhausted_invalid_decisions_defer_once_before_termination(self):
        class AlwaysInvalidPlanner:
            def __init__(self):
                self.calls = 0

            def decide(self, **_context):
                self.calls += 1
                raise LMStudioNavigationDecisionError(
                    "invalid_action_reason",
                    "Action and reason disagree",
                    latency_ms=7,
                )

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "invalid-exhausted-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=261),
            )
            planner = AlwaysInvalidPlanner()
            transport = FakeRuntimeTransport()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-invalid-exhausted",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Observe",
                    locale="en",
                    max_turns=2,
                    max_episode_seconds=10,
                    max_validation_attempts=2,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            result = runtime.run()

        self.assertEqual(result.terminal_reason, "reasoning_unavailable")
        self.assertFalse(result.completed)
        self.assertTrue(result.shutdown_clean)
        self.assertEqual(result.model_calls, 4)
        self.assertEqual(result.model_latency_ms, 28)
        self.assertEqual(planner.calls, 4)
        self.assertEqual(result.actions, ())
        self.assertEqual(
            [operation for operation, _arguments in transport.calls],
            ["start", "describe", "shutdown", "close"],
        )
        self.assertEqual(
            [
                event["attempt"]
                for event in events
                if event["event"] == "decision_vetoed"
            ],
            [1, 2, 1, 2],
        )
        self.assertEqual(
            [
                event["turn"]
                for event in events
                if event["event"] == "planner_turn_deferred"
            ],
            [1],
        )
        termination = next(
            event
            for event in events
            if event["event"] == "planner_termination"
        )
        self.assertEqual(
            termination["terminal_reason"],
            "reasoning_unavailable",
        )

    def test_deferred_invalid_decision_recovers_on_next_turn(self):
        class InvalidTwiceThenObservePlanner:
            def __init__(self):
                self.calls = []

            def decide(self, **context):
                self.calls.append({
                    "turn": context["turn"],
                    "feedback": copy.deepcopy(
                        context["validation_feedback"]
                    ),
                })
                if len(self.calls) <= 2:
                    raise LMStudioNavigationDecisionError(
                        "invalid_action_reason",
                        "Action and reason disagree",
                        latency_ms=7,
                    )
                return NavigationDecision.from_mapping(
                    decision_mapping(
                        episode_id=context["episode_id"],
                        turn=context["turn"],
                        state_version=context["observation"][
                            "state_version"
                        ],
                        action=OBSERVE,
                        plan=[OBSERVE],
                        reason_code="VERIFY_RESULT",
                    ),
                    episode_id=context["episode_id"],
                    turn=context["turn"],
                    state_version=context["observation"]["state_version"],
                    available_actions=context["available_actions"],
                    published_target_ids=(),
                )

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "invalid-deferred-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=262),
            )
            planner = InvalidTwiceThenObservePlanner()
            transport = FakeRuntimeTransport()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-invalid-deferred",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Observe",
                    locale="en",
                    max_turns=2,
                    max_episode_seconds=10,
                    max_validation_attempts=2,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            result = runtime.run()

        self.assertEqual(
            [call["turn"] for call in planner.calls],
            [1, 1, 2],
        )
        self.assertIsNone(planner.calls[0]["feedback"])
        self.assertEqual(
            planner.calls[2]["feedback"]["code"],
            "invalid_action_reason",
        )
        self.assertEqual(result.actions, (OBSERVE,))
        self.assertEqual(result.model_calls, 3)
        self.assertEqual(result.model_latency_ms, 14)
        self.assertTrue(result.shutdown_clean)
        self.assertEqual(
            [
                event["turn"]
                for event in events
                if event["event"] == "planner_turn_deferred"
            ],
            [1],
        )
        self.assertFalse(
            any(event["event"] == "planner_termination" for event in events)
        )

    def test_late_planner_output_cannot_dispatch_a_physical_operation(self):
        class LateObservePlanner:
            def __init__(self, clock):
                self.clock = clock

            def decide(self, **context):
                self.clock[0] = 11.0
                return NavigationDecision.from_mapping(
                    decision_mapping(
                        episode_id=context["episode_id"],
                        turn=context["turn"],
                        state_version=context["observation"]["state_version"],
                        action=OBSERVE,
                        plan=[OBSERVE],
                        reason_code="VERIFY_RESULT",
                    ),
                    episode_id=context["episode_id"],
                    turn=context["turn"],
                    state_version=context["observation"]["state_version"],
                    available_actions=context["available_actions"],
                    published_target_ids=(),
                )

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "late-planner-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=262),
            )
            clock = [0.0]
            transport = FakeRuntimeTransport()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-late-planner",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Observe",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=10,
                ),
                transport=transport,
                planner=LateObservePlanner(clock),
                memory=memory,
                monotonic=lambda: clock[0],
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            result = runtime.run()

        self.assertEqual(result.terminal_reason, "episode_deadline_elapsed")
        self.assertFalse(result.completed)
        self.assertTrue(result.shutdown_clean)
        self.assertEqual(result.model_calls, 1)
        self.assertEqual(result.actions, ())
        self.assertEqual(
            [operation for operation, _arguments in transport.calls],
            ["start", "describe", "shutdown", "close"],
        )
        discarded = next(
            event
            for event in events
            if event["event"] == "planner_output_discarded"
        )
        self.assertEqual(discarded["reason"], "episode_deadline_elapsed")

    def test_unverified_final_shutdown_is_a_typed_physical_fault(self):
        class UnverifiedShutdownTransport(FakeRuntimeTransport):
            def request(
                self,
                operation,
                arguments,
                timeout,
                cancel_requested=None,
            ):
                if operation != "shutdown":
                    return super().request(
                        operation,
                        arguments,
                        timeout,
                        cancel_requested=cancel_requested,
                    )
                self.calls.append((operation, copy.deepcopy(arguments)))
                return {
                    "state_version": self.version + 1,
                    "result": {
                        "outcome": {
                            "kind": "shutdown",
                            "status": "completed",
                            "completed_monotonic_ms": 5,
                            "stop_confirmed": False,
                            "motor_owner_closed": True,
                        }
                    },
                }

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "shutdown-fault-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=263),
            )
            transport = UnverifiedShutdownTransport()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-shutdown-fault",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Observe",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=10,
                ),
                transport=transport,
                planner=CaptureAvailablePlanner(),
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            with self.assertRaises(
                PhysicalNavigationRuntimeError
            ) as caught:
                runtime.run()

        self.assertEqual(
            caught.exception.code,
            "physical_shutdown_unverified",
        )
        self.assertEqual(
            [operation for operation, _arguments in transport.calls],
            ["start", "describe", "observe", "shutdown", "close"],
        )
        stopped = next(
            event for event in events if event["event"] == "episode_stopped"
        )
        self.assertEqual(
            stopped["terminal_reason"],
            "physical_shutdown_unverified",
        )
        self.assertFalse(stopped["shutdown_clean"])

    def test_unverified_shutdown_retains_primary_scan_transport_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "masked-scan-fault-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=305),
            )
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-masked-scan-fault",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect obstacle",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=10,
                ),
                transport=FakeRuntimeTransport(),
                planner=CaptureAvailablePlanner(),
                memory=memory,
            )
            scan_error = PhysicalNavigationRuntimeError(
                "scan_transport_failed",
                "Physical scan transport failed while awaiting worker receipt",
            )
            with mock.patch.object(
                runtime,
                "_start_worker_session",
                side_effect=scan_error,
            ), mock.patch.object(runtime, "_cleanup", return_value=False):
                with self.assertRaises(
                    PhysicalNavigationRuntimeError
                ) as caught:
                    runtime.run()

        self.assertEqual(
            caught.exception.code,
            "physical_shutdown_unverified",
        )
        self.assertIs(caught.exception.primary_error, scan_error)
        self.assertEqual(
            caught.exception.primary_error.code,
            "scan_transport_failed",
        )

    def test_verified_shutdown_survives_local_transport_reap_failure(self):
        class ReapFailureTransport(FakeRuntimeTransport):
            def close(self):
                super().close()
                raise EV3NavigationTransportError(
                    "SSH process exited nonzero after verified shutdown"
                )

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "shutdown-reap-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=264),
            )
            transport = ReapFailureTransport()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-shutdown-reap",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Observe",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=10,
                ),
                transport=transport,
                planner=CaptureAvailablePlanner(),
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            result = runtime.run()

        self.assertTrue(result.shutdown_clean)
        degraded = next(
            event
            for event in events
            if event["event"] == "transport_cleanup_degraded"
        )
        self.assertTrue(degraded["physical_shutdown_verified"])
        self.assertEqual(
            degraded["error_type"],
            "EV3NavigationTransportError",
        )
        stopped = next(
            event for event in events if event["event"] == "episode_stopped"
        )
        self.assertTrue(stopped["shutdown_clean"])

    def test_degraded_motion_cancels_tail_and_replans_from_encoder_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "degraded-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=24),
            )
            transport = DegradedFirstPulseTransport()
            planner = DegradedFeedbackPlanner()
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-degraded-motion",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Move forward and adapt to wheel undertravel",
                    locale="en",
                    minimum_forward_progress_mm=100,
                    max_turns=2,
                    max_episode_seconds=10,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
            )

            result = runtime.run()

        self.assertEqual(result.actions, (ADVANCE, OBSERVE))
        self.assertEqual(result.plan_tails_cancelled, 1)
        self.assertEqual(planner.calls, 2)
        self.assertEqual(planner.feedback["status"], "verification_failed")
        self.assertEqual(
            planner.feedback["encoder_observation"],
            {
                "action": ADVANCE,
                "left_encoder_delta_degrees": 174,
                "right_encoder_delta_degrees": 0,
                "verified_slice_count": 0,
                "observed_slice_count": 1,
                "requested_slice_count": 1,
                "command_completed": False,
            },
        )
        self.assertEqual(
            planner.feedback["resulting_pose"]["heading_mdeg"],
            -11_484,
        )
        self.assertTrue(memory.localization_valid)
        pulse_actions = [
            arguments["action"]
            for operation, arguments in transport.calls
            if operation == "pulse"
        ]
        self.assertEqual(pulse_actions, [ADVANCE])

    def test_only_host_committed_decision_can_offer_utterance(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "speech-validation-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=18),
            )
            offered = []
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-speech-validation",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Move forward",
                    locale="sv",
                    minimum_forward_progress_mm=100,
                    max_turns=1,
                    max_episode_seconds=10,
                ),
                transport=FakeRuntimeTransport(),
                planner=InvalidThenValidSpeechPlanner(),
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=lambda event: events.append(dict(event)),
                validated_utterance_sink=offered.append,
            )

            result = runtime.run()

        self.assertEqual(result.actions, (ADVANCE, ADVANCE))
        self.assertEqual(offered, ["Nu kör jag framåt."])
        proposals = [
            event
            for event in events
            if event["event"] == "model_decision"
        ]
        committed = [
            event
            for event in events
            if event["event"] == "model_decision_committed"
        ]
        self.assertEqual(len(proposals), 2)
        self.assertTrue(
            all(
                event["decision_status"] == "proposed"
                for event in proposals
            )
        )
        self.assertEqual(
            [event["utterance"] for event in committed],
            ["Nu kör jag framåt."],
        )
        self.assertEqual(committed[0]["decision_status"], "committed")

    def test_execution_veto_never_offers_utterance(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "speech-veto-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=19),
            )
            offered = []
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-speech-veto",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Move forward",
                    locale="en",
                    minimum_forward_progress_mm=100,
                    max_turns=1,
                    max_episode_seconds=10,
                ),
                transport=FakeRuntimeTransport(blocked=False),
                planner=VetoedSpeechPlanner(memory),
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=lambda event: events.append(dict(event)),
                validated_utterance_sink=offered.append,
            )

            result = runtime.run()

        self.assertEqual(result.actions, ())
        self.assertEqual(offered, [])
        self.assertTrue(
            any(event["event"] == "execution_vetoed" for event in events)
        )
        self.assertFalse(
            any(
                event["event"] == "model_decision_committed"
                for event in events
            )
        )

    def test_adapter_scopes_speech_deduplication_to_action_progress(self):
        class SpeechRecorder:
            def __init__(self):
                self.offers = []

            def start(self):
                return None

            def offer(self, **value):
                self.offers.append(dict(value))
                return len(self.offers)

            def cancel_episode(self, _episode_id):
                return None

            def close(self, **_kwargs):
                return True

        speech = SpeechRecorder()
        adapter = PhysicalNavigationRuntimeAdapter(
            transport_factory=object,
            planner_factory=lambda _model: object(),
            memory_factory=object,
            speech_runtime_factory=lambda **_kwargs: speech,
        )
        context = SimpleNamespace(
            episode_id="episode-speech-progress",
            request=SimpleNamespace(goal="Observe", locale="en"),
            settings=SimpleNamespace(
                model="test-model",
                max_episode_ms=10_000,
                speech_enabled=True,
            ),
            stop_requested=threading.Event(),
            emergency_stop_requested=threading.Event(),
            publish=lambda _update: None,
        )

        with mock.patch(
            "robot_agent.physical_navigation_adapter."
            "PhysicalNavigationRuntime"
        ) as runtime_type:
            def run_runtime():
                runtime = runtime_type.call_args.kwargs
                publish = runtime["event_sink"]
                offer = runtime["validated_utterance_sink"]

                def commit(action):
                    publish({
                        "event": "model_decision_committed",
                        "action": action,
                        "plan": [action],
                        "assessment": "test",
                        "utterance": "Status ready",
                    })

                commit(OBSERVE)
                offer("Status: READY!")
                commit(OBSERVE)
                offer("status ready")
                commit(ADVANCE)
                offer("STATUS READY")
                publish({
                    "event": "motion_result",
                    "action": ADVANCE,
                    "navigation": {
                        "navigation_hazard_hypotheses": [],
                    },
                })
                offer("Status ready.")
                return SimpleNamespace(
                    terminal_reason="goal_completed",
                    completed=True,
                    model_latency_ms=0,
                )

            runtime_type.return_value.run.side_effect = run_runtime
            adapter.run(context)

        self.assertEqual(
            [item["progress_revision"] for item in speech.offers],
            [1, 1, 2, 3],
        )

    def test_adapter_speech_overlaps_motion_and_close_cancels_it(self):
        speech_started = threading.Event()
        speech_cancelled = threading.Event()
        updates = []

        class SpeechAwareTransport(FakeRuntimeTransport):
            def request(
                self,
                operation,
                arguments,
                timeout,
                cancel_requested=None,
            ):
                if operation == "pulse":
                    if not speech_started.wait(1):
                        raise AssertionError("motion waited for no speech")
                    self.assert_speech_still_active = (
                        not speech_cancelled.is_set()
                    )
                return super().request(
                    operation,
                    arguments,
                    timeout,
                    cancel_requested=cancel_requested,
                )

        transport = SpeechAwareTransport()

        def speaker(_text, _locale, cancel_event):
            speech_started.set()
            if cancel_event.wait(2):
                speech_cancelled.set()

        with tempfile.TemporaryDirectory() as directory:
            def memory_factory():
                return NavigationMemoryStore.load(
                    path=Path(directory) / "adapter-speech-memory.json",
                    robot_id="ev3rstorm-01",
                    controller_instance_id="ev3-main",
                    reset=True,
                    clock_ms=lambda: 1_000,
                    uuid_factory=lambda: uuid.UUID(int=20),
                )

            adapter = PhysicalNavigationRuntimeAdapter(
                transport_factory=lambda: transport,
                planner_factory=lambda _model: SpeechRuntimePlanner(),
                memory_factory=memory_factory,
                speech_runtime_factory=(
                    lambda *, event_sink: RobotSpeechRuntime(
                        speaker=speaker,
                        event_sink=event_sink,
                    )
                ),
                minimum_forward_progress_mm=100,
            )
            context = SimpleNamespace(
                episode_id="episode-adapter-speech",
                request=SimpleNamespace(
                    goal="Move forward while speaking",
                    locale="sv",
                ),
                settings=SimpleNamespace(
                    model="test-model",
                    max_episode_ms=10_000,
                    speech_enabled=True,
                ),
                stop_requested=threading.Event(),
                emergency_stop_requested=threading.Event(),
                publish=updates.append,
            )

            result = adapter.run(context)

        self.assertEqual(result["message"], "goal_completed")
        self.assertTrue(transport.assert_speech_still_active)
        self.assertTrue(speech_cancelled.wait(1))
        statuses = [
            update["speech_status"]
            for update in updates
            if "speech_status" in update
        ]
        self.assertIn("queued", statuses)
        self.assertIn("playing", statuses)
        self.assertIn("cancelled", statuses)

    def test_adapter_speech_disabled_constructs_nothing(self):
        with tempfile.TemporaryDirectory() as directory:
            factory_calls = []

            def memory_factory():
                return NavigationMemoryStore.load(
                    path=Path(directory) / "adapter-muted-memory.json",
                    robot_id="ev3rstorm-01",
                    controller_instance_id="ev3-main",
                    reset=True,
                    clock_ms=lambda: 1_000,
                    uuid_factory=lambda: uuid.UUID(int=21),
                )

            def speech_runtime_factory(**_kwargs):
                factory_calls.append(True)
                raise AssertionError("muted episode created speech")

            adapter = PhysicalNavigationRuntimeAdapter(
                transport_factory=FakeRuntimeTransport,
                planner_factory=lambda _model: FakeRuntimePlanner(),
                memory_factory=memory_factory,
                speech_runtime_factory=speech_runtime_factory,
                minimum_forward_progress_mm=100,
            )
            context = SimpleNamespace(
                episode_id="episode-adapter-muted",
                request=SimpleNamespace(goal="Move forward", locale="en"),
                settings=SimpleNamespace(
                    model="test-model",
                    max_episode_ms=10_000,
                    speech_enabled=False,
                ),
                stop_requested=threading.Event(),
                emergency_stop_requested=threading.Event(),
                publish=lambda _update: None,
            )

            result = adapter.run(context)

        self.assertEqual(result["message"], "goal_completed")
        self.assertEqual(factory_calls, [])

    def test_adapter_blocks_next_episode_when_speech_cannot_close(self):
        class UnreapedSpeech:
            def start(self):
                return None

            def offer(self, **_kwargs):
                return 1

            def cancel_episode(self, _episode_id):
                return None

            def close(self, **_kwargs):
                return False

        with tempfile.TemporaryDirectory() as directory:
            factory_calls = []

            def memory_factory():
                factory_calls.append("memory")
                return NavigationMemoryStore.load(
                    path=Path(directory) / "adapter-orphan-memory.json",
                    robot_id="ev3rstorm-01",
                    controller_instance_id="ev3-main",
                    reset=True,
                    clock_ms=lambda: 1_000,
                    uuid_factory=lambda: uuid.UUID(int=23),
                )

            adapter = PhysicalNavigationRuntimeAdapter(
                transport_factory=FakeRuntimeTransport,
                planner_factory=lambda _model: FakeRuntimePlanner(),
                memory_factory=memory_factory,
                speech_runtime_factory=(
                    lambda *, event_sink: UnreapedSpeech()
                ),
                minimum_forward_progress_mm=100,
            )
            context = SimpleNamespace(
                episode_id="episode-adapter-orphan",
                request=SimpleNamespace(goal="Move forward", locale="en"),
                settings=SimpleNamespace(
                    model="test-model",
                    max_episode_ms=10_000,
                    speech_enabled=True,
                ),
                stop_requested=threading.Event(),
                emergency_stop_requested=threading.Event(),
                publish=lambda _update: None,
            )

            result = adapter.run(context)
            with self.assertRaises(PhysicalNavigationRuntimeError) as caught:
                adapter.run(context)

        self.assertEqual(result["message"], "goal_completed")
        self.assertEqual(caught.exception.code, "runtime_already_active")
        self.assertEqual(factory_calls, ["memory"])

    def test_adapter_emergency_cancels_active_speech_and_motion(self):
        speech_started = threading.Event()
        speech_cancelled = threading.Event()
        updates = []
        returned = []
        failures = []
        transport = BlockingPulseTransport()

        def speaker(_text, _locale, cancel_event):
            speech_started.set()
            if cancel_event.wait(2):
                speech_cancelled.set()

        with tempfile.TemporaryDirectory() as directory:
            def memory_factory():
                return NavigationMemoryStore.load(
                    path=Path(directory) / "adapter-emergency-memory.json",
                    robot_id="ev3rstorm-01",
                    controller_instance_id="ev3-main",
                    reset=True,
                    clock_ms=lambda: 1_000,
                    uuid_factory=lambda: uuid.UUID(int=22),
                )

            adapter = PhysicalNavigationRuntimeAdapter(
                transport_factory=lambda: transport,
                planner_factory=lambda _model: SpeechRuntimePlanner(),
                memory_factory=memory_factory,
                speech_runtime_factory=(
                    lambda *, event_sink: RobotSpeechRuntime(
                        speaker=speaker,
                        event_sink=event_sink,
                    )
                ),
                minimum_forward_progress_mm=100,
            )
            context = SimpleNamespace(
                episode_id="episode-adapter-emergency",
                request=SimpleNamespace(goal="Move forward", locale="en"),
                settings=SimpleNamespace(
                    model="test-model",
                    max_episode_ms=60_000,
                    speech_enabled=True,
                ),
                stop_requested=threading.Event(),
                emergency_stop_requested=threading.Event(),
                publish=updates.append,
            )

            def run_adapter():
                try:
                    returned.append(adapter.run(context))
                except BaseException as error:
                    failures.append(error)

            thread = threading.Thread(target=run_adapter, daemon=True)
            thread.start()
            self.assertTrue(speech_started.wait(1))
            self.assertTrue(transport.pulse_entered.wait(1))
            context.emergency_stop_requested.set()
            adapter.emergency_stop()
            thread.join(2)

        self.assertFalse(thread.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(returned[0]["message"], "emergency_stopped")
        self.assertTrue(transport.cancel_observed)
        self.assertTrue(speech_cancelled.wait(1))
        self.assertIn(
            "cancelled",
            [
                update["speech_status"]
                for update in updates
                if "speech_status" in update
            ],
        )

    def test_successful_scan_reanchors_changed_encoders_without_moving_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "scan-reanchor-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=15),
            )
            transport = FakeRuntimeTransport(blocked=True)
            planner = SingleScanPlanner()
            scan_executor = RuntimeScanExecutor(transport)
            scan_calibration = ActiveIrScanCalibration(
                alignment_tolerance_mdeg=10_000,
            )
            runtime_times = iter((1_500, 2_000, 10_000))
            offers = []

            def offer(**value):
                hazards = value["memory"].hazard_map.hazards
                offers.append({
                    "captured_at_ms": value["captured_at_ms"],
                    "map_version": value["memory"].hazard_map.revision,
                    "scan_complete": bool(
                        hazards and hazards[-1].bilateral_scan_complete
                    ),
                    "state_version": value["observation"]["state_version"],
                })
                return True

            runtime = PhysicalNavigationRuntime(
                episode_id="episode-scan-reanchor",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect the obstacle",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                active_scan_executor=scan_executor,
                active_scan_calibration=scan_calibration,
                monotonic=lambda: 0.0,
                unix_ms=lambda: next(runtime_times),
                observation_sink=offer,
            )
            initial_pose = memory.pose

            result = runtime.run()

        self.assertEqual(result.actions, (SCAN_FRONT_ARC,))
        self.assertEqual(len(scan_executor.requests), 1)
        self.assertIs(
            scan_executor.requests[0].calibration,
            scan_calibration,
        )
        self.assertTrue(memory.localization_valid)
        self.assertEqual(memory.pose, initial_pose)
        self.assertEqual(
            memory.motor_positions,
            {"drive_b": 7, "drive_c": -6},
        )
        self.assertIn(SCAN_FRONT_ARC, planner.available_actions)
        hazard = memory.hazard_map.hazards[0]
        self.assertGreater(
            hazard.last_seen_at_ms,
            hazard.scan_completed_at_ms,
        )
        self.assertTrue(hazard.bilateral_scan_complete)
        self.assertEqual(
            [operation for operation, _arguments in transport.calls].count(
                "observe"
            ),
            1,
        )
        self.assertEqual(offers, [
            {
                "captured_at_ms": 1_500,
                "map_version": 1,
                "scan_complete": False,
                "state_version": 1,
            },
            {
                "captured_at_ms": 10_000,
                "map_version": 3,
                "scan_complete": True,
                "state_version": 2,
            },
        ])

    def test_unscanned_forward_hazard_blocks_detour_turns(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "scan-before-detour-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=315),
            )
            planner = CaptureAvailablePlanner()
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-scan-before-detour",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect before choosing a detour",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=60,
                ),
                transport=FakeRuntimeTransport(blocked=True),
                planner=planner,
                memory=memory,
                active_scan_executor=object(),
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
            )

            result = runtime.run()

        self.assertEqual(result.actions, (OBSERVE,))
        self.assertIn(SCAN_FRONT_ARC, planner.available_actions)
        self.assertIn(REVERSE, planner.available_actions)
        self.assertNotIn(TURN_LEFT_90, planner.available_actions)
        self.assertNotIn(TURN_RIGHT_90, planner.available_actions)
        self.assertEqual(
            planner.navigation[
                "detour_scan_required_target_hypothesis_ids"
            ],
            [
                planner.navigation["navigation_hazard_hypotheses"][0][
                    "hypothesis_id"
                ]
            ],
        )

    def test_first_scanned_detour_turn_requires_route_commitment(self):
        decision = NavigationDecision.from_mapping(
            decision_mapping(
                episode_id="episode-detour-commitment",
                turn=1,
                state_version=1,
                action=TURN_RIGHT_90,
                plan=[TURN_RIGHT_90, ADVANCE],
                reason_code="HANDLE_OBSTACLE",
            ),
            episode_id="episode-detour-commitment",
            turn=1,
            state_version=1,
            available_actions=(TURN_RIGHT_90, ADVANCE),
            published_target_ids=("box-a",),
        )
        mission = {
            "completed": False,
            "candidate_action_longitudinal_deltas_mm": {
                TURN_RIGHT_90: 0,
            },
            "projected_goal_heading_aligned_after_action": {
                TURN_RIGHT_90: False,
            },
        }
        navigation = {
            "goal_geometry": {
                "conflicts": [{
                    "hypothesis_id": "box-a",
                    "active_for_collision": True,
                }],
            },
            "navigation_hazard_hypotheses": [{
                "hypothesis_id": "box-a",
                "active_for_collision": True,
                "route_commitment_ready": True,
            }],
        }

        with self.assertRaises(PhysicalNavigationRuntimeError) as caught:
            PhysicalNavigationRuntime._validate_mission_decision(
                decision, mission, navigation
            )

        self.assertEqual(caught.exception.code, "detour_commitment_required")

    def test_restored_soft_scan_timeout_reanchors_and_replans(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "scan-timeout-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=151),
            )
            transport = FakeRuntimeTransport(blocked=True)
            planner = ScanThenObservePlanner()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-scan-soft-timeout",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect the obstacle",
                    locale="en",
                    max_turns=2,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                active_scan_executor=(
                    RestoredCancelledRuntimeScanExecutor(transport)
                ),
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            result = runtime.run()

        self.assertEqual(
            result.actions,
            (SCAN_FRONT_ARC, OBSERVE),
        )
        self.assertTrue(memory.localization_valid)
        self.assertEqual(
            memory.motor_positions,
            {"drive_b": 7, "drive_c": -6},
        )
        self.assertEqual(planner.calls, 2)
        self.assertNotIn(
            SCAN_FRONT_ARC,
            planner.available_history[1],
        )
        self.assertTrue({OBSERVE, REVERSE}.issubset(
            planner.available_history[1]
        ))
        self.assertNotIn(ADVANCE, planner.available_history[1])
        self.assertEqual(planner.feedback["status"], "CANCELLED")
        self.assertEqual(
            planner.feedback["reason"],
            "scan_deadline_exceeded",
        )
        scan_event = next(
            event for event in events if event["event"] == "scan_result"
        )
        self.assertEqual(scan_event["scan"]["status"], "CANCELLED")
        self.assertTrue(scan_event["scan"]["restored_start_heading"])

    def test_restored_unilateral_scan_is_not_repeated_after_progress(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "scan-progress-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=153),
            )
            transport = DecisionRelevantChangeTransport()
            planner = ScanObserveChangedThenScanPlanner()
            scan_executor = RestoredEvidenceRuntimeScanExecutor(
                transport,
                all_clear_after_first=True,
            )
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-scan-progress-gate",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect and adapt around the obstacle",
                    locale="en",
                    max_turns=3,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                active_scan_executor=scan_executor,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
            )

            result = runtime.run()

        self.assertEqual(
            result.actions,
            (SCAN_FRONT_ARC, OBSERVE, OBSERVE),
        )
        self.assertEqual(scan_executor.calls, 1)
        self.assertIn(SCAN_FRONT_ARC, planner.available_history[0])
        self.assertNotIn(SCAN_FRONT_ARC, planner.available_history[1])
        self.assertNotIn(SCAN_FRONT_ARC, planner.available_history[2])
        hazard = memory.hazard_map.hazards[0]
        self.assertEqual(len(hazard.scan_evidence_history), 1)
        first = hazard.scan_evidence_history[0]
        self.assertEqual(first.left_boundary_mdeg, 7_500)
        self.assertIsNone(first.right_boundary_mdeg)

    def test_scan_progress_barrier_rearms_for_pose_or_target_change(self):
        baseline = observation(2, blocked=True)
        baseline["motors"].append({
            "role": "arm",
            "position": 0,
            "state": "",
        })
        motor_roles = ("drive_b", "drive_c")
        barrier = RestoredScanProgressBarrier(
            scan_id="scan-progress-facts",
            target_hypothesis_id="hazard-a",
            map_generation_id="map-a",
            pose=PhysicalPose(),
            hazard_ids=("hazard-a",),
            observation_signature=observation_progress_signature(
                baseline,
                motor_roles=motor_roles,
            ),
            motor_roles=motor_roles,
        )

        arm_only = copy.deepcopy(baseline)
        arm_only["state_version"] = 3
        arm_only["motors"][-1]["position"] = 360
        self.assertIsNone(barrier.rearm_reason(
            map_generation_id="map-a",
            pose=PhysicalPose(),
            hazard_ids=("hazard-a",),
            observation=arm_only,
        ))

        drive_anchor_only = copy.deepcopy(baseline)
        drive_anchor_only["state_version"] = 4
        for motor in drive_anchor_only["motors"]:
            if motor["role"] in motor_roles:
                motor["position"] += 180
        self.assertIsNone(barrier.rearm_reason(
            map_generation_id="map-a",
            pose=PhysicalPose(),
            hazard_ids=("hazard-a",),
            observation=drive_anchor_only,
        ))

        self.assertIsNone(barrier.rearm_reason(
            map_generation_id="map-a",
            pose=PhysicalPose(),
            hazard_ids=("hazard-a",),
            observation=observation(99, blocked=True),
        ))
        self.assertEqual(
            barrier.rearm_reason(
                map_generation_id="map-a",
                pose=PhysicalPose(x_mm=1),
                hazard_ids=("hazard-a",),
                observation=baseline,
            ),
            "VERIFIED_POSE_CHANGED",
        )
        self.assertEqual(
            barrier.rearm_reason(
                map_generation_id="map-a",
                pose=PhysicalPose(),
                hazard_ids=("hazard-a", "hazard-b"),
                observation=baseline,
            ),
            "TARGET_HYPOTHESES_CHANGED",
        )

    def test_arm_position_is_not_navigation_observation_information(self):
        before = observation(1)
        after = observation(2)
        before["motors"].append({
            "role": "arm",
            "position": 0,
            "state": "",
        })
        after["motors"].append({
            "role": "arm",
            "position": 720,
            "state": "",
        })

        result = observation_information_result(
            before,
            after,
            motor_roles=("drive_b", "drive_c"),
        )

        self.assertEqual(result["information_gain"], "NONE")
        self.assertEqual(result["changed_facts"], [])

    def test_rejected_scan_map_fusion_replans_without_physical_fault(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "scan-fusion-rejected.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=152),
            )
            transport = FakeRuntimeTransport(blocked=True)
            planner = ScanThenObservePlanner()
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-scan-fusion-rejected",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect the obstacle",
                    locale="en",
                    max_turns=2,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                active_scan_executor=RuntimeScanExecutor(transport),
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            with mock.patch.object(
                memory.hazard_map,
                "record_scan_result",
                side_effect=ValueError("simulated stale basis"),
            ):
                result = runtime.run()

        self.assertEqual(result.actions, (SCAN_FRONT_ARC, OBSERVE))
        self.assertTrue(memory.localization_valid)
        self.assertEqual(planner.feedback["status"], "CANCELLED")
        self.assertEqual(
            planner.feedback["reason"],
            "scan_boundary_map_integration_rejected",
        )
        self.assertEqual(
            planner.feedback["evidence_disposition"],
            "DISCARDED",
        )
        scan_event = next(
            event
            for event in events
            if event["event"] == "scan_result"
        )
        self.assertEqual(scan_event["evidence_disposition"], "DISCARDED")
        self.assertEqual(
            scan_event["map_integration"]["status"],
            "rejected",
        )

    def test_partial_turn_touch_invalidates_localization_before_more_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "scan-touch-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=16),
            )
            transport = FakeRuntimeTransport(blocked=True)
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-scan-touch",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect the obstacle",
                    locale="sv",
                    max_turns=2,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=SingleScanPlanner(),
                memory=memory,
                active_scan_executor=RuntimeScanExecutor(
                    transport,
                    touch_after_turn=True,
                ),
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            with self.assertRaises(PhysicalNavigationRuntimeError) as caught:
                runtime.run()

        self.assertEqual(caught.exception.code, "scan_heading_unrestored")
        self.assertFalse(memory.localization_valid)
        self.assertIn("restoration", memory.localization_error)
        scan_event = next(
            event for event in events if event["event"] == "scan_result"
        )
        self.assertEqual(scan_event["scan"]["status"], "CANCELLED")
        self.assertEqual(
            scan_event["scan"]["reason"],
            "scan_touch_cancelled",
        )
        self.assertGreater(len(scan_event["scan"]["rays"]), 0)
        self.assertFalse(scan_event["scan"]["restored_start_heading"])
        task_operations = [
            operation
            for operation, _arguments in transport.calls
            if operation in ("observe", "pulse")
        ]
        self.assertEqual(task_operations, [])

    def test_scan_is_omitted_when_worker_slice_budget_is_insufficient(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "scan-budget-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=17),
            )
            transport = FakeRuntimeTransport(
                blocked=True,
                pulse_count_remaining=21,
            )
            planner = CaptureAvailablePlanner()
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-scan-budget",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect the obstacle",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=planner,
                memory=memory,
                active_scan_executor=object(),
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
            )

            result = runtime.run()

        self.assertEqual(result.actions, (OBSERVE,))
        self.assertNotIn(SCAN_FRONT_ARC, planner.available_actions)

    def test_scan_rotation_feasibility_is_published_and_filters_planner(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "scan-footprint-filter.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=154),
                hazard_calibration=HazardMapCalibration(
                    robot_footprint=RobotFootprint(
                        front_extent_mm=100,
                        rear_extent_mm=60,
                        left_extent_mm=150,
                        right_extent_mm=60,
                    ),
                ),
            )
            planner = CaptureAvailablePlanner()
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-scan-footprint-filter",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect the obstacle",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=60,
                ),
                transport=FakeRuntimeTransport(blocked=True),
                planner=planner,
                memory=memory,
                active_scan_executor=ScanExecutorMustNotRun(),
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
            )

            result = runtime.run()

        self.assertEqual(result.actions, (OBSERVE,))
        self.assertNotIn(SCAN_FRONT_ARC, planner.available_actions)
        feasibility = planner.navigation["scan_front_arc_feasibility"]
        self.assertFalse(feasibility["allowed"])
        self.assertEqual(
            feasibility["reason"],
            "provisional_hazard_rotation_sweep_collision",
        )

    def test_rotation_blocked_scan_prefers_clearance_reverse_over_detour(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "scan-clearance-reverse.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=316),
                hazard_calibration=HazardMapCalibration(
                    robot_footprint=RobotFootprint(
                        front_extent_mm=100,
                        rear_extent_mm=60,
                        left_extent_mm=150,
                        right_extent_mm=60,
                    ),
                ),
            )
            memory.hazard_map.record_observation(
                PhysicalPose(), observation(1, blocked=True), 1_000
            )
            memory.pose = PhysicalPose(x_mm=-60)
            planner = CaptureAvailablePlanner()
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-scan-clearance-reverse",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Make room to inspect the obstacle",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=60,
                ),
                transport=FakeRuntimeTransport(blocked=False),
                planner=planner,
                memory=memory,
                active_scan_executor=ScanExecutorMustNotRun(),
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
            )

            result = runtime.run()

        self.assertEqual(result.actions, (OBSERVE,))
        feasibility = planner.navigation["action_feasibility"]
        self.assertFalse(feasibility["active_scan"]["allowed"])
        self.assertTrue(
            feasibility["motion_actions"][TURN_LEFT_90]["allowed"]
        )
        self.assertIn(REVERSE, planner.available_actions)
        self.assertNotIn(TURN_LEFT_90, planner.available_actions)
        self.assertNotIn(TURN_RIGHT_90, planner.available_actions)
        self.assertEqual(
            planner.navigation[
                "detour_scan_required_target_hypothesis_ids"
            ],
            [
                planner.navigation["navigation_hazard_hypotheses"][0][
                    "hypothesis_id"
                ]
            ],
        )

    def test_scan_rotation_feasibility_is_rechecked_before_executor(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "scan-footprint-toctou.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=155),
                hazard_calibration=HazardMapCalibration(
                    provisional_hazard_offset_mm=500,
                    robot_footprint=RobotFootprint(
                        front_extent_mm=100,
                        rear_extent_mm=60,
                        left_extent_mm=150,
                        right_extent_mm=60,
                    ),
                ),
            )
            events = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-scan-footprint-toctou",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Inspect the obstacle",
                    locale="en",
                    max_turns=1,
                    max_episode_seconds=60,
                ),
                transport=FakeRuntimeTransport(blocked=True),
                planner=MapChangesAfterScanPlanningPlanner(memory),
                memory=memory,
                active_scan_executor=ScanExecutorMustNotRun(),
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                event_sink=events.append,
            )

            result = runtime.run()

        self.assertEqual(result.actions, (SCAN_FRONT_ARC,))
        denied = next(
            event for event in events if event["event"] == "scan_denied"
        )
        self.assertEqual(
            denied["scan"]["reason"],
            "provisional_hazard_rotation_sweep_collision",
        )
        self.assertEqual(memory.hazard_map.hazards[0].scan_evidence_history, ())

    def test_dashboard_runner_adapter_uses_injected_model_and_publishes(self):
        with tempfile.TemporaryDirectory() as directory:
            selected_models = []
            updates = []

            def planner_factory(model):
                selected_models.append(model)
                return FakeRuntimePlanner()

            def memory_factory():
                return NavigationMemoryStore.load(
                    path=Path(directory) / "adapter-memory.json",
                    robot_id="ev3rstorm-01",
                    controller_instance_id="ev3-main",
                    reset=True,
                    clock_ms=lambda: 1_000,
                    uuid_factory=lambda: uuid.UUID(int=10),
                )

            adapter = PhysicalNavigationRuntimeAdapter(
                transport_factory=FakeRuntimeTransport,
                planner_factory=planner_factory,
                memory_factory=memory_factory,
                minimum_forward_progress_mm=100,
            )
            context = SimpleNamespace(
                episode_id="episode-adapter",
                request=SimpleNamespace(
                    goal="Move forward while observing the room",
                    locale="en",
                ),
                settings=SimpleNamespace(
                    model="mlx-community/gemma-4-26b-a4b-it",
                    max_episode_ms=10_000,
                ),
                stop_requested=threading.Event(),
                emergency_stop_requested=threading.Event(),
                publish=updates.append,
            )
            result = adapter.run(context)

        self.assertEqual(
            selected_models,
            ["mlx-community/gemma-4-26b-a4b-it"],
        )
        self.assertEqual(result["message"], "goal_completed")
        self.assertTrue(
            any(
                update.get("current_action") == ADVANCE
                and update.get("plan") == [ADVANCE, ADVANCE]
                for update in updates
            )
        )
        self.assertEqual(updates[-1]["message"], "goal_completed")

    def test_emergency_during_planning_starts_no_later_operation(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "cancel-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=11),
            )
            emergency = threading.Event()
            transport = FakeRuntimeTransport()
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-cancel-planner",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Move forward",
                    locale="en",
                    minimum_forward_progress_mm=100,
                    max_turns=3,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=CancelDuringPlanner(emergency),
                memory=memory,
                emergency_event=emergency,
            )
            result = runtime.run()

        self.assertEqual(result.terminal_reason, "emergency_stopped")
        self.assertEqual(result.actions, ())
        task_operations = [
            operation
            for operation, _arguments in transport.calls
            if operation in ("pulse", "observe")
        ]
        self.assertEqual(task_operations, [])
        self.assertEqual(
            [item for item in transport.calls if item[0] == "shutdown"],
            [("shutdown", {})],
        )

    def test_cancellation_between_tail_actions_starts_no_second_pulse(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "tail-cancel-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=12),
            )
            cancelled = threading.Event()
            transport = FakeRuntimeTransport()

            def cancel_after_first_motion(event):
                if event.get("event") == "motion_result":
                    cancelled.set()

            runtime = PhysicalNavigationRuntime(
                episode_id="episode-cancel-tail",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Move forward",
                    locale="sv",
                    minimum_forward_progress_mm=100,
                    max_turns=3,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=FakeRuntimePlanner(),
                memory=memory,
                cancel_event=cancelled,
                event_sink=cancel_after_first_motion,
            )
            result = runtime.run()

        self.assertEqual(result.terminal_reason, "cancelled")
        self.assertEqual(result.actions, (ADVANCE,))
        pulse_actions = [
            arguments["action"]
            for operation, arguments in transport.calls
            if operation == "pulse"
        ]
        self.assertEqual(pulse_actions, [ADVANCE])

    def test_stop_during_observe_preserves_verified_shutdown_channel(self):
        class StopDuringObserveTransport(FakeRuntimeTransport):
            def __init__(self):
                super().__init__()
                self.observe_entered = threading.Event()
                self.release_observe = threading.Event()
                self.observe_cancel_probe = object()
                self.aborted = False

            def request(
                self,
                operation,
                arguments,
                timeout,
                cancel_requested=None,
            ):
                if self.aborted:
                    raise EV3NavigationTransportError(
                        "aborted navigation transport cannot be reused"
                    )
                if operation != "observe":
                    return super().request(
                        operation,
                        arguments,
                        timeout,
                        cancel_requested=cancel_requested,
                    )
                self.calls.append((operation, copy.deepcopy(arguments)))
                self.observe_cancel_probe = cancel_requested
                self.observe_entered.set()
                if not self.release_observe.wait(1.0):
                    raise AssertionError("observe response was not released")
                if (
                    callable(cancel_requested)
                    and cancel_requested() is True
                ):
                    # Match the SSH transport: cancelling a written request
                    # closes the channel, so no shutdown receipt is possible.
                    self.aborted = True
                    raise EV3NavigationTransportError(
                        "worker request cancelled; SSH channel closed"
                    )
                self.version += 1
                return {
                    "state_version": self.version,
                    "result": {"observation": self._observation()},
                }

        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "stop-during-observe.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=265),
            )
            transport = StopDuringObserveTransport()
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-stop-during-observe",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Observe",
                    locale="en",
                    max_turns=3,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=CaptureAvailablePlanner(),
                memory=memory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
            )
            returned = []
            failures = []

            def run_runtime():
                try:
                    returned.append(runtime.run())
                except BaseException as error:
                    failures.append(error)

            thread = threading.Thread(target=run_runtime, daemon=True)
            thread.start()
            self.assertTrue(transport.observe_entered.wait(1.0))
            runtime.request_stop()
            transport.release_observe.set()
            thread.join(1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(len(returned), 1)
        self.assertEqual(returned[0].terminal_reason, "cancelled")
        self.assertTrue(returned[0].shutdown_clean)
        self.assertIsNone(transport.observe_cancel_probe)
        self.assertEqual(
            [operation for operation, _arguments in transport.calls],
            ["start", "describe", "observe", "shutdown", "close"],
        )

    def test_inflight_motion_request_observes_stop_callback(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "inflight-cancel-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=13),
            )
            cancelled = threading.Event()
            transport = BlockingPulseTransport()
            offers = []
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-inflight-cancel",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Move forward",
                    locale="en",
                    minimum_forward_progress_mm=100,
                    max_turns=3,
                    max_episode_seconds=60,
                ),
                transport=transport,
                planner=FakeRuntimePlanner(),
                memory=memory,
                cancel_event=cancelled,
                observation_sink=lambda **value: offers.append({
                    "localization_valid": value[
                        "memory"
                    ].localization_valid,
                    "map_version": value["memory"].hazard_map.revision,
                    "state_version": value["observation"]["state_version"],
                }) or True,
            )
            returned = []
            failures = []

            def run_runtime():
                try:
                    returned.append(runtime.run())
                except BaseException as error:
                    failures.append(error)

            thread = threading.Thread(target=run_runtime, daemon=True)
            thread.start()
            self.assertTrue(transport.pulse_entered.wait(1.0))
            cancelled.set()
            thread.join(1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(len(returned), 1)
        self.assertEqual(returned[0].terminal_reason, "cancelled")
        self.assertTrue(transport.cancel_observed)
        self.assertEqual(returned[0].actions, ())
        self.assertFalse(memory.localization_valid)
        self.assertEqual(
            memory.localization_error,
            "Pulse cancellation lost its correlated encoder receipt",
        )
        self.assertFalse(
            returned[0].final_navigation["localization_valid"]
        )
        self.assertEqual(
            [operation for operation, _arguments in transport.calls],
            ["start", "describe", "pulse", "shutdown", "close"],
        )
        self.assertEqual(offers, [
            {
                "localization_valid": True,
                "map_version": 1,
                "state_version": 1,
            },
            {
                "localization_valid": False,
                "map_version": 2,
                "state_version": 1,
            },
        ])

    def test_long_episode_renews_bounded_worker_between_plans(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "renewal-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
                clock_ms=lambda: 1_000,
                uuid_factory=lambda: uuid.UUID(int=14),
            )
            first = FakeRuntimeTransport(pulse_count_remaining=21)
            second = FakeRuntimeTransport()
            replacements = [second]
            scan_bindings = []
            offers = []

            def transport_factory():
                return replacements.pop(0)

            def scan_executor_factory(transport):
                scan_bindings.append(transport)
                return object()

            runtime = PhysicalNavigationRuntime(
                episode_id="episode-worker-renewal",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Explore for up to one hour",
                    locale="sv",
                    minimum_forward_progress_mm=100,
                    max_episode_seconds=3_600,
                ),
                transport=first,
                transport_factory=transport_factory,
                planner=FakeRuntimePlanner(),
                memory=memory,
                active_scan_executor=object(),
                active_scan_executor_factory=scan_executor_factory,
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
                observation_sink=lambda **value: offers.append({
                    "state_version": value["observation"]["state_version"],
                    "map_version": value["memory"].hazard_map.revision,
                }),
            )
            result = runtime.run()

        self.assertTrue(result.completed)
        self.assertTrue(result.shutdown_clean)
        self.assertEqual(replacements, [])
        self.assertEqual(scan_bindings, [second])
        self.assertEqual(
            [operation for operation, _arguments in first.calls],
            ["start", "describe", "shutdown", "close"],
        )
        self.assertEqual(
            [
                arguments["action"]
                for operation, arguments in second.calls
                if operation == "pulse"
            ],
            [ADVANCE, ADVANCE],
        )
        self.assertEqual(offers, [
            {"state_version": 1, "map_version": 1},
            {"state_version": 1, "map_version": 2},
            {"state_version": 2, "map_version": 3},
            {"state_version": 3, "map_version": 4},
        ])

    def test_latched_motion_fault_requires_worker_session_renewal(self):
        with tempfile.TemporaryDirectory() as directory:
            memory = NavigationMemoryStore.load(
                path=Path(directory) / "latched-renewal-memory.json",
                robot_id="ev3rstorm-01",
                controller_instance_id="ev3-main",
                reset=True,
            )
            runtime = PhysicalNavigationRuntime(
                episode_id="episode-latched-renewal",
                config=PhysicalNavigationRuntimeConfig(
                    goal="Continue after a recoverable motor fault",
                    locale="en",
                    max_episode_seconds=60,
                ),
                transport=FakeRuntimeTransport(),
                planner=FakeRuntimePlanner(),
                memory=memory,
                monotonic=lambda: 0.0,
            )

            self.assertTrue(runtime._worker_session_needs_renewal(
                observation(1, motion_fault_latched=True),
                EXPECTED_ACTION_SPECS,
            ))

    def test_runtime_accepts_full_dashboard_episode_duration_range(self):
        configured = PhysicalNavigationRuntimeConfig(
            goal="Explore",
            locale="en",
            max_episode_seconds=3_600,
            scan_timeout_seconds=120.0,
        )
        self.assertEqual(configured.max_episode_seconds, 3_600)
        self.assertEqual(configured.max_turns, 14_400)
        self.assertEqual(configured.scan_timeout_seconds, 120.0)
        self.assertEqual(configured.goal_heading_tolerance_mdeg, 5_000)
        ev3_tolerance = PhysicalNavigationRuntimeConfig(
            goal="Explore",
            locale="en",
            goal_heading_tolerance_mdeg=20_000,
        )
        self.assertEqual(
            ev3_tolerance.goal_heading_tolerance_mdeg,
            20_000,
        )
        with self.assertRaises(ValueError):
            PhysicalNavigationRuntimeConfig(
                goal="Explore",
                locale="en",
                max_episode_seconds=3_601,
            )
        with self.assertRaises(ValueError):
            PhysicalNavigationRuntimeConfig(
                goal="Explore",
                locale="en",
                scan_timeout_seconds=120.001,
            )
        with self.assertRaises(ValueError):
            PhysicalNavigationRuntimeConfig(
                goal="Explore",
                locale="en",
                goal_heading_tolerance_mdeg=45_001,
            )


class LMStudioNavigationLocaleTests(unittest.TestCase):
    def test_structured_schema_makes_none_commitment_an_exact_sentinel(self):
        schema = _maneuver_schema()

        self.assertEqual(set(schema), {"anyOf"})
        self.assertEqual(len(schema["anyOf"]), 2)
        none_branch, active_branch = schema["anyOf"]
        none = none_branch["properties"]
        active = active_branch["properties"]

        self.assertEqual(none["transition"], {
            "type": "string",
            "const": "NONE",
        })
        self.assertEqual(none["target_hypothesis_id"], {"type": "null"})
        self.assertEqual(none["current_focus_fact_key"], {"type": "null"})
        self.assertEqual(none["success_fact_keys"]["maxItems"], 0)
        self.assertNotIn("NONE", active["transition"]["enum"])
        self.assertEqual(active["target_hypothesis_id"]["type"], "string")

    def test_commitment_schema_opens_only_after_bilateral_scan_evidence(self):
        cases = (
            (False, None, False),
            (True, None, True),
            # A newer all-clear attempt can explicitly invalidate legacy
            # boundary fields retained only as history.
            (True, False, False),
        )
        for scanned, bilateral_fact, schema_opens in cases:
            with self.subTest(
                scanned=scanned,
                bilateral_fact=bilateral_fact,
            ):
                captured = {}

                def transport(_url, body, _headers, _timeout, _maximum):
                    captured["payload"] = json.loads(body.decode("utf-8"))
                    response_decision = decision_mapping(
                        episode_id="episode-scan-schema",
                        turn=1,
                        state_version=1,
                        action=OBSERVE,
                        plan=[OBSERVE],
                        reason_code="VERIFY_RESULT",
                    )
                    return json.dumps({
                        "choices": [{
                            "message": {
                                "content": json.dumps(response_decision),
                            },
                        }],
                    }).encode("utf-8")

                planner = LMStudioNavigationPlanner(
                    base_url="http://127.0.0.1:1234",
                    model="test-model",
                    transport=transport,
                    clock=lambda: 1.0,
                )
                planner.decide(
                    episode_id="episode-scan-schema",
                    turn=1,
                    locale="en",
                    observation=observation(1),
                    mission={"completed": False},
                    navigation={
                        "navigation_hazard_hypotheses": [{
                            "hypothesis_id": "hazard-1",
                            "scan_completed_at_ms": 2_000 if scanned else None,
                            "scan_left_boundary_mdeg": 30_000 if scanned else None,
                            "scan_right_boundary_mdeg": -30_000 if scanned else None,
                            **(
                                {}
                                if bilateral_fact is None
                                else {
                                    "bilateral_scan_complete": (
                                        bilateral_fact
                                    )
                                }
                            ),
                        }],
                    },
                    maneuver_state={"active": None},
                    available_actions=[OBSERVE],
                    last_tool_result=None,
                )
                decision_schema = captured["payload"]["response_format"][
                    "json_schema"
                ]["schema"]
                schema = decision_schema["oneOf"][0]["properties"][
                    "maneuver_commitment"
                ]
                if schema_opens:
                    self.assertEqual(set(schema), {"anyOf"})
                else:
                    self.assertEqual(
                        schema["properties"]["transition"]["const"],
                        "NONE",
                    )

    def test_schema_binds_perception_target_to_scan_action(self):
        captured = {}

        def transport(_url, body, _headers, _timeout, _maximum):
            captured["payload"] = json.loads(body.decode("utf-8"))
            response_decision = decision_mapping(
                episode_id="episode-action-target-schema",
                turn=1,
                state_version=1,
                action=OBSERVE,
                plan=[OBSERVE],
                reason_code="VERIFY_RESULT",
            )
            return json.dumps({
                "choices": [{
                    "message": {
                        "content": json.dumps(response_decision),
                    },
                }],
            }).encode("utf-8")

        planner = LMStudioNavigationPlanner(
            base_url="http://127.0.0.1:1234",
            model="test-model",
            transport=transport,
            clock=lambda: 1.0,
        )
        planner.decide(
            episode_id="episode-action-target-schema",
            turn=1,
            locale="en",
            observation=observation(1, blocked=True),
            mission={"completed": False},
            navigation={
                "navigation_hazard_hypotheses": [{
                    "hypothesis_id": "hazard-1",
                }],
            },
            maneuver_state={"active": None},
            available_actions=[OBSERVE, SCAN_FRONT_ARC],
            last_tool_result=None,
        )

        schema = captured["payload"]["response_format"]["json_schema"][
            "schema"
        ]
        self.assertEqual(set(schema), {"oneOf"})
        self.assertEqual(len(schema["oneOf"]), 2)
        scan, non_scan = schema["oneOf"]
        self.assertEqual(
            scan["properties"]["action"]["const"],
            SCAN_FRONT_ARC,
        )
        self.assertEqual(
            scan["properties"]["perception_target_hypothesis_id"],
            {"type": "string", "enum": ["hazard-1"]},
        )
        self.assertEqual(
            non_scan["properties"]["action"]["enum"],
            [OBSERVE],
        )
        self.assertEqual(
            non_scan["properties"]["perception_target_hypothesis_id"],
            {"type": "null"},
        )

    def test_scan_schema_keeps_other_target_when_one_requires_progress(self):
        captured = {}

        def transport(_url, body, _headers, _timeout, _maximum):
            captured["payload"] = json.loads(body.decode("utf-8"))
            response_decision = decision_mapping(
                episode_id="episode-target-specific-scan",
                turn=1,
                state_version=1,
                action=OBSERVE,
                plan=[OBSERVE],
                reason_code="VERIFY_RESULT",
            )
            return json.dumps({
                "choices": [{
                    "message": {
                        "content": json.dumps(response_decision),
                    },
                }],
            }).encode("utf-8")

        planner = LMStudioNavigationPlanner(
            base_url="http://127.0.0.1:1234",
            model="test-model",
            transport=transport,
            clock=lambda: 1.0,
        )
        planner.decide(
            episode_id="episode-target-specific-scan",
            turn=1,
            locale="en",
            observation=observation(1, blocked=True),
            mission={"completed": False},
            navigation={
                "navigation_hazard_hypotheses": [
                    {"hypothesis_id": "hazard-a"},
                    {"hypothesis_id": "hazard-b"},
                ],
                "scan_eligible_target_hypothesis_ids": ["hazard-b"],
                "scan_progress_blocked_target_hypothesis_ids": [
                    "hazard-a"
                ],
            },
            maneuver_state={"active": None},
            available_actions=[OBSERVE, SCAN_FRONT_ARC],
            last_tool_result=None,
        )

        schema = captured["payload"]["response_format"]["json_schema"][
            "schema"
        ]
        scan = next(
            variant
            for variant in schema["oneOf"]
            if variant["properties"]["action"].get("const")
            == SCAN_FRONT_ARC
        )
        self.assertEqual(
            scan["properties"]["perception_target_hypothesis_id"],
            {"type": "string", "enum": ["hazard-b"]},
        )

    def test_invalid_decision_is_typed_for_runtime_feedback(self):
        def transport(_url, _body, _headers, _timeout, _maximum):
            invalid = decision_mapping(
                episode_id="episode-invalid-target",
                turn=1,
                state_version=1,
                action=OBSERVE,
                plan=[OBSERVE],
                reason_code="VERIFY_RESULT",
                target="hazard-1",
            )
            return json.dumps({
                "choices": [{
                    "message": {"content": json.dumps(invalid)},
                }],
            }).encode("utf-8")

        times = iter((1.0, 1.017))
        planner = LMStudioNavigationPlanner(
            base_url="http://127.0.0.1:1234",
            model="test-model",
            transport=transport,
            clock=lambda: next(times),
        )

        with self.assertRaises(
            LMStudioNavigationDecisionError
        ) as caught:
            planner.decide(
                episode_id="episode-invalid-target",
                turn=1,
                locale="en",
                observation=observation(1, blocked=True),
                mission={"completed": False},
                navigation={
                    "navigation_hazard_hypotheses": [{
                        "hypothesis_id": "hazard-1",
                    }],
                },
                maneuver_state={"active": None},
                available_actions=[OBSERVE, SCAN_FRONT_ARC],
                last_tool_result=None,
            )

        self.assertEqual(caught.exception.code, "unexpected_perception_target")
        self.assertEqual(caught.exception.latency_ms, 17)
        self.assertEqual(
            caught.exception.feedback_message,
            "Only SCAN_FRONT_ARC may name a perception target",
        )

    def test_episode_locale_is_forwarded_without_language_heuristics(self):
        for locale, expected_language in (
            ("sv", "Swedish or null"),
            ("en", "English or null"),
        ):
            with self.subTest(locale=locale):
                captured = {}

                def transport(url, body, headers, timeout, maximum):
                    captured["url"] = url
                    captured["payload"] = json.loads(body.decode("utf-8"))
                    captured["headers"] = headers
                    captured["timeout"] = timeout
                    captured["maximum"] = maximum
                    response_decision = decision_mapping(
                        episode_id="episode-locale",
                        turn=1,
                        state_version=1,
                        action=OBSERVE,
                        plan=[OBSERVE],
                        reason_code="VERIFY_RESULT",
                    )
                    return json.dumps(
                        {
                            "model": "served-model",
                            "choices": [
                                {
                                    "message": {
                                        "content": json.dumps(
                                            response_decision
                                        )
                                    }
                                }
                            ],
                            "usage": {},
                            "stats": {
                                "tokens_per_second": 99.25,
                                "time_to_first_token": 0.11,
                            },
                        }
                    ).encode("utf-8")

                times = iter((1.0, 1.01))
                planner = LMStudioNavigationPlanner(
                    base_url="http://127.0.0.1:1234",
                    model="injected-model",
                    transport=transport,
                    clock=lambda: next(times),
                )
                result = planner.decide(
                    episode_id="episode-locale",
                    turn=1,
                    locale=locale,
                    observation=observation(1),
                    mission={"completed": False},
                    navigation={
                        "navigation_hazard_hypotheses": [],
                    },
                    maneuver_state={"active": None},
                    available_actions=[OBSERVE],
                    last_tool_result=None,
                )
                context = json.loads(
                    captured["payload"]["messages"][1]["content"]
                )
                self.assertEqual(context["episode_locale"], locale)
                self.assertEqual(
                    context["output_languages"]["utterance"],
                    expected_language,
                )
                self.assertEqual(result.decision.action, OBSERVE)
                self.assertEqual(
                    captured["payload"]["model"],
                    "injected-model",
                )
                decision_schema = captured["payload"][
                    "response_format"
                ]["json_schema"]["schema"]
                commitment_schema = decision_schema["oneOf"][0][
                    "properties"
                ]["maneuver_commitment"]
                self.assertEqual(
                    commitment_schema["properties"]["transition"],
                    {"type": "string", "const": "NONE"},
                )
                self.assertEqual(
                    captured["url"],
                    "http://127.0.0.1:1234/v1/chat/completions",
                )
                self.assertEqual(
                    result.stats,
                    {
                        "tokens_per_second": 99.25,
                        "time_to_first_token": 0.11,
                    },
                )


if __name__ == "__main__":
    unittest.main()
