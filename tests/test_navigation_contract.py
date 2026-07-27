from dataclasses import replace
import json
import threading
import unittest

from robot_agent.navigation_contract import (
    NAVIGATION_PROPOSAL_SCHEMA,
    AdvanceSegment,
    DriveCalibrationProfile,
    DrivePulse,
    MotionAuthority,
    NavigationContractError,
    PlannerProposal,
    TurnSegment,
    WaypointGoal,
    decode_navigation_proposal,
)
from robot_agent.navigation_state import (
    ClearanceEvidence,
    NavigationSnapshot,
    PoseEstimate,
    ProposalInbox,
    ProposalSourcePolicy,
    StateReducer,
)


def raw_proposal(**changes):
    value = {
        "schema": NAVIGATION_PROPOSAL_SCHEMA,
        "proposal_id": "proposal-1",
        "goal_id": "waypoint-1",
        "goal_epoch": 1,
        "plan_revision": 1,
        "based_on_state_version": 4,
        "based_on_world_model_version": 2,
        "decision": "NEXT_SEGMENT",
        "confidence_milli": 900,
        "segment": {
            "type": "ADVANCE",
            "distance_mm": 80,
            "speed_mm_s": 100,
        },
    }
    value.update(changes)
    return json.dumps(value).encode("utf-8")


def snapshot(state_version=4, captured_at_ms=10_000):
    return NavigationSnapshot(
        robot_id="robot-sim",
        controller_instance_id="instance-1",
        goal_id="waypoint-1",
        goal_epoch=1,
        plan_revision=1,
        state_version=state_version,
        world_model_version=2,
        captured_at_host_ms=captured_at_ms,
        state_observed_at_ms=captured_at_ms,
        pose=PoseEstimate(200, 200, 0),
        left_encoder_mdeg=0,
        right_encoder_mdeg=0,
        motors_running=False,
        touch_pressed=False,
        active_faults=(),
        clearance=ClearanceEvidence(
            source="simulation_metric",
            observed_at_ms=captured_at_ms,
            near_obstacle_latched=False,
            forward_mm=400,
            left_mm=300,
            right_mm=300,
        ),
    )


class NavigationProposalContractTests(unittest.TestCase):
    def test_decodes_strict_advance_and_turn_segments(self):
        advance = decode_navigation_proposal(raw_proposal())
        turn = decode_navigation_proposal(
            raw_proposal(
                segment={
                    "type": "TURN",
                    "angle_mdeg": -30_000,
                    "angular_speed_mdeg_s": 75_000,
                }
            )
        )

        self.assertIsInstance(advance.segment, AdvanceSegment)
        self.assertEqual(advance.segment.distance_mm, 80)
        self.assertIsInstance(turn.segment, TurnSegment)
        self.assertEqual(turn.segment.angle_mdeg, -30_000)

    def test_hold_has_reason_and_no_segment(self):
        value = json.loads(raw_proposal())
        value.pop("segment")
        value["decision"] = "HOLD"
        value["reason_code"] = "awaiting_perception"

        proposal = decode_navigation_proposal(
            json.dumps(value).encode("utf-8")
        )

        self.assertEqual(proposal.decision, "HOLD")
        self.assertIsNone(proposal.segment)
        self.assertEqual(proposal.reason_code, "awaiting_perception")

    def test_rejects_extra_host_authority_fields_from_model(self):
        with self.assertRaises(NavigationContractError) as caught:
            decode_navigation_proposal(
                raw_proposal(priority=10_000, valid_until_ms=999_999)
            )

        self.assertEqual(caught.exception.code, "invalid_proposal_fields")

    def test_rejects_duplicate_json_key(self):
        raw = raw_proposal().replace(
            b'"proposal_id": "proposal-1",',
            b'"proposal_id": "proposal-1",'
            b'"proposal_id": "proposal-2",',
        )

        with self.assertRaises(NavigationContractError) as caught:
            decode_navigation_proposal(raw)

        self.assertEqual(caught.exception.code, "invalid_proposal_json")

    def test_reverse_advance_is_not_part_of_first_contract(self):
        with self.assertRaises(NavigationContractError):
            decode_navigation_proposal(
                raw_proposal(
                    segment={
                        "type": "ADVANCE",
                        "distance_mm": -20,
                        "speed_mm_s": 80,
                    }
                )
            )

    def test_provisional_physical_profile_is_explicitly_not_ready(self):
        profile = DriveCalibrationProfile(
            calibration_id="ev3-provisional",
            status="provisional",
            surface="unknown",
            left_motor_sign=1,
            right_motor_sign=1,
            encoder_mdeg_per_mm=None,
            encoder_mdeg_per_body_degree=7_580,
            max_wheel_speed_dps=250,
            max_pulse_ms=120,
            physical_stop_latency_verified=False,
        )

        self.assertFalse(profile.ready_for_physical_motion)
        with self.assertRaises(NavigationContractError) as caught:
            profile.require_complete_geometry()
        self.assertEqual(caught.exception.code, "incomplete_calibration")

    def test_motion_authority_replay_memory_is_bounded(self):
        authority = MotionAuthority(max_pending=1, replay_window=2)
        for number in range(5):
            pulse = DrivePulse(
                decision_id="decision-{}".format(number),
                arbiter_id="arbiter",
                robot_id="robot",
                controller_instance_id="instance",
                goal_id="goal",
                goal_epoch=1,
                plan_revision=1,
                based_on_state_version=number + 1,
                based_on_world_model_version=1,
                kind="STOP",
                left_speed_dps=0,
                right_speed_dps=0,
                duration_ms=0,
                reason_code="bounded_replay_test",
            )
            authority.authorize(pulse)
            authority.consume(pulse)

        self.assertEqual(len(authority._pending), 0)
        self.assertEqual(len(authority._consumed), 2)
        self.assertEqual(len(authority._consumed_order), 2)


class NavigationStateTests(unittest.TestCase):
    def setUp(self):
        self.now_ms = 10_000
        self.inbox = ProposalInbox(
            (
                ProposalSourcePolicy(
                    source_id="goal-seeking",
                    authority_rank=10,
                    priority=50,
                    ttl_ms=200,
                ),
            ),
            lambda: self.now_ms,
            capacity=4,
        )

    def proposal(self, proposal_id="proposal-1"):
        return PlannerProposal(
            proposal_id=proposal_id,
            goal_id="waypoint-1",
            goal_epoch=1,
            plan_revision=1,
            based_on_state_version=4,
            based_on_world_model_version=2,
            decision="NEXT_SEGMENT",
            confidence_milli=900,
            segment=AdvanceSegment(80, 100),
        )

    def test_inbox_host_stamps_authority_time_and_ttl(self):
        stamped = self.inbox.publish(
            self.proposal(),
            "goal-seeking",
            1,
        )

        self.assertEqual(stamped.received_at_ms, 10_000)
        self.assertEqual(stamped.valid_until_ms, 10_200)
        self.assertEqual(stamped.authority_rank, 10)
        self.assertEqual(stamped.priority, 50)

    def test_inbox_rejects_id_and_sequence_replay(self):
        self.inbox.publish(self.proposal(), "goal-seeking", 1)
        with self.assertRaises(NavigationContractError) as duplicate:
            self.inbox.publish(
                self.proposal(),
                "goal-seeking",
                2,
            )
        with self.assertRaises(NavigationContractError) as sequence:
            self.inbox.publish(
                self.proposal("proposal-2"),
                "goal-seeking",
                1,
            )

        self.assertEqual(
            duplicate.exception.code,
            "duplicate_proposal_id",
        )
        self.assertEqual(
            sequence.exception.code,
            "replayed_source_sequence",
        )

    def test_drain_consumes_winners_and_losers(self):
        self.inbox.publish(self.proposal(), "goal-seeking", 1)
        self.inbox.publish(
            self.proposal("proposal-2"),
            "goal-seeking",
            2,
        )

        self.assertEqual(len(self.inbox.drain()), 2)
        self.assertEqual(len(self.inbox.drain()), 0)

    def test_replay_id_window_is_bounded(self):
        inbox = ProposalInbox(
            (
                ProposalSourcePolicy(
                    source_id="goal-seeking",
                    authority_rank=10,
                    priority=50,
                    ttl_ms=200,
                ),
            ),
            lambda: self.now_ms,
            capacity=4,
            replay_window=4,
        )
        for number in range(1, 11):
            inbox.publish(
                self.proposal("bounded-{}".format(number)),
                "goal-seeking",
                number,
            )
            inbox.drain()

        self.assertEqual(len(inbox._proposal_ids), 4)
        self.assertEqual(len(inbox._proposal_id_order), 4)

    def test_future_goal_proposal_cannot_poison_inbox_epoch(self):
        future = replace(
            self.proposal("future-goal"),
            goal_epoch=2**63 - 1,
        )
        self.inbox.publish(future, "goal-seeking", 1)
        self.inbox.drain()

        current = self.inbox.publish(
            self.proposal("current-goal"),
            "goal-seeking",
            2,
        )

        self.assertEqual(current.proposal.goal_epoch, 1)

    def test_concurrent_publish_is_bounded_and_sequence_safe(self):
        barrier = threading.Barrier(3)
        outcomes = []

        def publish(number):
            barrier.wait()
            try:
                self.inbox.publish(
                    self.proposal("proposal-{}".format(number)),
                    "goal-seeking",
                    1,
                )
                outcomes.append("accepted")
            except NavigationContractError:
                outcomes.append("rejected")

        threads = [
            threading.Thread(target=publish, args=(number,))
            for number in (1, 2)
        ]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join()

        self.assertEqual(
            sorted(outcomes),
            ["accepted", "rejected"],
        )
        self.assertEqual(len(self.inbox), 1)

    def test_independent_sources_can_publish_concurrently(self):
        inbox = ProposalInbox(
            (
                ProposalSourcePolicy(
                    "goal-seeking",
                    authority_rank=10,
                    priority=50,
                    ttl_ms=200,
                ),
                ProposalSourcePolicy(
                    "obstacle-avoidance",
                    authority_rank=20,
                    priority=100,
                    ttl_ms=120,
                ),
            ),
            lambda: self.now_ms,
        )
        barrier = threading.Barrier(3)
        accepted = []

        def publish(source_id, proposal_id):
            barrier.wait()
            inbox.publish(
                self.proposal(proposal_id),
                source_id,
                1,
            )
            accepted.append(source_id)

        threads = [
            threading.Thread(
                target=publish,
                args=("goal-seeking", "parallel-goal"),
            ),
            threading.Thread(
                target=publish,
                args=("obstacle-avoidance", "parallel-obstacle"),
            ),
        ]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join()

        self.assertEqual(
            sorted(accepted),
            ["goal-seeking", "obstacle-avoidance"],
        )
        self.assertEqual(len(inbox.drain()), 2)

    def test_state_reducer_rejects_state_replay(self):
        reducer = StateReducer(snapshot())
        reducer.commit(snapshot(state_version=5, captured_at_ms=10_001))

        with self.assertRaises(NavigationContractError) as caught:
            reducer.commit(snapshot(state_version=5, captured_at_ms=10_002))

        self.assertEqual(
            caught.exception.code,
            "non_monotonic_snapshot",
        )

    def test_physical_ir_high_value_is_not_metric_clearance(self):
        evidence = ClearanceEvidence(
            source="physical_ir_reflection",
            observed_at_ms=10_000,
            near_obstacle_latched=False,
            raw_ir_proximity=52,
        )

        self.assertFalse(evidence.positively_cleared_for_simulation)
        self.assertIsNone(evidence.forward_mm)
        with self.assertRaises(NavigationContractError):
            replace(evidence, forward_mm=1_000)


if __name__ == "__main__":
    unittest.main()
