import copy
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import unittest
import uuid

from robot_agent.active_ir_scan import ActiveIrScanExecutor
from robot_agent.active_ir_scan_contract import (
    ActiveIrScanCalibration,
    ModelScanChoice,
    build_scan_request,
    validate_scan_result,
)
from robot_agent.ev3_navigation_transport import (
    EV3NavigationRemoteError,
    EV3NavigationSSHTransport,
)
from robot_agent.maneuver_commitment import (
    FACT_GOAL_CORRIDOR_CLEAR,
    FACT_GOAL_HEADING_ALIGNED,
    FACT_TARGET_BEHIND,
    ManeuverCommitment,
    ManeuverCommitmentError,
    empty_commitment,
)
from robot_agent.lm_studio_navigation import LMStudioNavigationPlanner
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
    NavigationDecision,
    PhysicalNavigationContractError,
    expected_scan_turn_profile,
    expected_scan_sample_profile,
)
from robot_agent.physical_navigation_mission import DirectionalMission
from robot_agent.physical_navigation_runtime import (
    PhysicalNavigationRuntime,
    PhysicalNavigationRuntimeAdapter,
    PhysicalNavigationRuntimeConfig,
    PhysicalNavigationRuntimeError,
)
from robot_agent.physical_odometry import DriveMotorRoles, PhysicalPose


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
            "motion_fault_latched": False,
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
        "utterance": None,
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


class PhysicalNavigationContractTests(unittest.TestCase):
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
                fact_values={},
            )
        self.assertEqual(caught.exception.code, "bilateral_scan_required")

        memory.hazard_map.record_scan_boundaries(
            target,
            completed_at_ms=2_000,
            left_boundary_mdeg=20_000,
            right_boundary_mdeg=-20_000,
        )
        state = commitment.apply(
            proposal,
            action=ADVANCE,
            turn=1,
            hazard_map=memory.hazard_map,
            fact_values={},
        )
        self.assertEqual(
            state["active"]["target_hypothesis_id"],
            target,
        )


class FakeScanRig:
    def __init__(self, late_final=False, touch_after_turn=False):
        self.now = 1_000
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

    def read_snapshot(self):
        self.read_count += 1
        self.now += 10
        if self.late_final and self.heading == 0 and self.read_count > 5:
            self.now += 60_000
        self.state_version += 1
        blocked = abs(self.heading) <= 20_000
        return {
            "state_version": self.state_version,
            "observed_at_ms": self.now,
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


class EV3NavigationTransportTests(unittest.TestCase):
    def test_worker_error_with_observation_and_stop_is_typed_remote_error(self):
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path="/home/robot/robot-llm/ev3/navigation_worker.py",
        )
        current = observation(4)
        frame = {
            "schema": "ev3-agent-worker-response/v1",
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
                    "response_schema": "ev3-agent-worker-response/v1",
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
                        "duration_ms": 800,
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


class CaptureAvailablePlanner:
    def __init__(self):
        self.available_actions = None

    def decide(self, **context):
        self.available_actions = tuple(context["available_actions"])
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


class RuntimeScanExecutor:
    def __init__(self, transport, *, touch_after_turn=False):
        self.transport = transport
        self.touch_after_turn = touch_after_turn

    def execute(self, request, cancel_requested):
        rig = FakeScanRig(touch_after_turn=self.touch_after_turn)
        rig.now = request.created_at_ms
        result = ActiveIrScanExecutor(
            rig=rig,
            clock_ms=lambda: rig.now,
        ).execute(request, cancel_requested=cancel_requested)
        self.transport.left += 7
        self.transport.right -= 6
        return result


class PhysicalNavigationRuntimeTests(unittest.TestCase):
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
                active_scan_executor=RuntimeScanExecutor(transport),
                monotonic=lambda: 0.0,
                unix_ms=lambda: 2_000,
            )
            initial_pose = memory.pose

            result = runtime.run()

        self.assertEqual(result.actions, (SCAN_FRONT_ARC,))
        self.assertTrue(memory.localization_valid)
        self.assertEqual(memory.pose, initial_pose)
        self.assertEqual(
            memory.motor_positions,
            {"drive_b": 7, "drive_c": -6},
        )
        self.assertIn(SCAN_FRONT_ARC, planner.available_actions)
        self.assertEqual(
            [operation for operation, _arguments in transport.calls].count(
                "observe"
            ),
            1,
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
            )

            with self.assertRaises(PhysicalNavigationRuntimeError) as caught:
                runtime.run()

        self.assertEqual(caught.exception.code, "scan_heading_unrestored")
        self.assertFalse(memory.localization_valid)
        self.assertIn("restoration", memory.localization_error)
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
        self.assertEqual(
            [operation for operation, _arguments in transport.calls],
            ["start", "describe", "pulse", "shutdown", "close"],
        )

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

    def test_runtime_accepts_full_dashboard_episode_duration_range(self):
        configured = PhysicalNavigationRuntimeConfig(
            goal="Explore",
            locale="en",
            max_episode_seconds=3_600,
        )
        self.assertEqual(configured.max_episode_seconds, 3_600)
        self.assertEqual(configured.max_turns, 14_400)
        with self.assertRaises(ValueError):
            PhysicalNavigationRuntimeConfig(
                goal="Explore",
                locale="en",
                max_episode_seconds=3_601,
            )


class LMStudioNavigationLocaleTests(unittest.TestCase):
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
