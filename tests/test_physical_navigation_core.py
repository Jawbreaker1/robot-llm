import copy
from pathlib import Path
import tempfile
import unittest
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
    FACT_KEYS,
    FACT_TARGET_BEHIND,
    ManeuverCommitment,
    ManeuverCommitmentError,
    empty_commitment,
)
from robot_agent.navigation_memory_store import NavigationMemoryStore
from robot_agent.navigation_memory_store import NavigationMemoryError
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
from robot_agent.physical_odometry import (
    DriveMotorRoles,
    PhysicalPose,
    apply_verified_motion,
    verified_motion_from_result,
)


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

    def test_episode_start_adopts_one_degree_encoder_settling(self):
        self.memory.begin_episode(observation(1), 1_001)

        self.memory.begin_episode(
            observation(
                2,
                left_position=-1,
                right_position=1,
            ),
            1_002,
        )

        self.assertTrue(self.memory.localization_valid)
        self.assertEqual(
            self.memory.motor_positions,
            {"left_drive": -1, "right_drive": 1},
        )
        self.assertEqual(self.memory.pose, PhysicalPose())

    def test_episode_start_rejects_two_degree_encoder_change(self):
        self.memory.begin_episode(observation(1), 1_001)

        with self.assertRaises(NavigationMemoryError):
            self.memory.begin_episode(
                observation(2, left_position=2),
                1_002,
            )

        self.assertFalse(self.memory.localization_valid)

    def test_stationary_observation_still_rejects_one_degree_change(self):
        self.memory.begin_episode(observation(1), 1_001)

        with self.assertRaises(NavigationMemoryError):
            self.memory.ingest_stationary_observation(
                observation(2, right_position=1),
                1_002,
            )

        self.assertFalse(self.memory.localization_valid)

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

    def test_motion_can_defer_one_ambiguous_hazard_observation(self):
        self.memory.begin_episode(observation(1), 1_001)
        result = degraded_motion_result()
        result["observation"]["infrared"] = observation(
            2,
            blocked=True,
        )["infrared"]

        hazard = self.memory.apply_motion_result(
            ADVANCE,
            result,
            1_002,
            record_hazard_evidence=False,
        )

        self.assertIsNone(hazard)
        self.assertEqual(self.memory.hazard_map.hazard_ids, ())
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

        cumulative_completion = copy.deepcopy(response)
        final_segment = cumulative_completion["result"]["outcome"][
            "slices"
        ][0]["segments"][-1]
        final_segment["status"] = "verification_failed"
        final_segment["reason"] = "encoder_undertravel_observed"
        final_segment["encoder_verification"] = {
            "passed": False,
            "error": "recovery segment undertravel",
            "checks": [{"passed": False}],
        }
        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            cumulative_completion,
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






















































if __name__ == "__main__":
    unittest.main()
