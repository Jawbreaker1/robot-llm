from dataclasses import replace
import itertools
import unittest

from robot_agent.navigation_contract import (
    AdvanceSegment,
    DriveCalibrationProfile,
    MotionAuthority,
    NavigationContractError,
    PlannerProposal,
    StampedProposal,
    TurnSegment,
    WaypointGoal,
)
from robot_agent.navigation_state import (
    ClearanceEvidence,
    NavigationSnapshot,
    PoseEstimate,
)
from robot_agent.navigation_supervisor import (
    MotionPolicy,
    MotionSupervisor,
)


def simulation_profile():
    return DriveCalibrationProfile(
        calibration_id="synthetic-nav-fixture-v1",
        status="simulation_only",
        surface="synthetic",
        left_motor_sign=1,
        right_motor_sign=1,
        encoder_mdeg_per_mm=1_800,
        encoder_mdeg_per_body_degree=2_000,
        max_wheel_speed_dps=250,
        max_pulse_ms=120,
    )


def goal(epoch=1):
    return WaypointGoal(
        goal_id="waypoint-1",
        goal_epoch=epoch,
        plan_revision=1,
        target_x_mm=900,
        target_y_mm=200,
        tolerance_mm=30,
    )


def snapshot(
    now_ms=10_000,
    epoch=1,
    state_version=4,
    clearance=None,
    **changes
):
    value = NavigationSnapshot(
        robot_id="ev3rstorm-sim",
        controller_instance_id="nav-sim-instance-1",
        goal_id="waypoint-1",
        goal_epoch=epoch,
        plan_revision=1,
        state_version=state_version,
        world_model_version=2,
        captured_at_host_ms=now_ms,
        state_observed_at_ms=now_ms,
        pose=PoseEstimate(200, 200, 0),
        left_encoder_mdeg=0,
        right_encoder_mdeg=0,
        motors_running=False,
        touch_pressed=False,
        active_faults=(),
        clearance=(
            ClearanceEvidence(
                source="simulation_metric",
                observed_at_ms=now_ms,
                near_obstacle_latched=False,
                forward_mm=500,
                left_mm=400,
                right_mm=400,
            )
            if clearance is None
            else clearance
        ),
    )
    return replace(value, **changes)


def proposal(
    proposal_id="proposal-1",
    segment=None,
    decision="NEXT_SEGMENT",
    reason_code=None,
    epoch=1,
    state_version=4,
):
    return PlannerProposal(
        proposal_id=proposal_id,
        goal_id="waypoint-1",
        goal_epoch=epoch,
        plan_revision=1,
        based_on_state_version=state_version,
        based_on_world_model_version=2,
        decision=decision,
        confidence_milli=900,
        segment=AdvanceSegment(80, 100) if segment is None else segment,
        reason_code=reason_code,
    )


def stamped(
    proposal_value,
    source_id="goal-seeking",
    sequence=1,
    received_at_ms=10_000,
    valid_until_ms=10_200,
    authority_rank=10,
    priority=50,
):
    return StampedProposal(
        proposal=proposal_value,
        source_id=source_id,
        source_sequence=sequence,
        received_at_ms=received_at_ms,
        valid_until_ms=valid_until_ms,
        authority_rank=authority_rank,
        priority=priority,
    )


class MotionSupervisorTests(unittest.TestCase):
    def setUp(self):
        self.now_ms = 10_000
        self.motion_authority = MotionAuthority()
        ids = itertools.count(1)
        self.supervisor = MotionSupervisor(
            simulation_profile(),
            lambda: self.now_ms,
            robot_id="ev3rstorm-sim",
            controller_instance_id="nav-sim-instance-1",
            motion_authority=self.motion_authority,
            id_factory=lambda: "decision-{}".format(next(ids)),
        )

    def test_authorizes_one_short_bounded_advance_pulse(self):
        decision = self.supervisor.decide(
            snapshot(),
            goal(),
            (stamped(proposal()),),
        )

        self.assertEqual(decision.kind, "DRIVE")
        self.assertEqual(decision.reason_code, "authorized_advance")
        self.assertLessEqual(decision.duration_ms, 120)
        self.assertLessEqual(abs(decision.left_speed_dps), 250)
        self.assertEqual(
            decision.left_speed_dps,
            decision.right_speed_dps,
        )

    def test_turn_does_not_require_fake_forward_clearance(self):
        physical_ir = ClearanceEvidence(
            source="physical_ir_reflection",
            observed_at_ms=10_000,
            near_obstacle_latched=False,
            raw_ir_proximity=52,
        )
        decision = self.supervisor.decide(
            snapshot(clearance=physical_ir),
            goal(),
            (
                stamped(
                    proposal(
                        segment=TurnSegment(30_000, 75_000)
                    )
                ),
            ),
        )

        self.assertEqual(decision.kind, "DRIVE")
        self.assertEqual(decision.reason_code, "authorized_turn")
        self.assertLess(decision.left_speed_dps, 0)
        self.assertGreater(decision.right_speed_dps, 0)

    def test_high_physical_ir_never_authorizes_forward(self):
        physical_ir = ClearanceEvidence(
            source="physical_ir_reflection",
            observed_at_ms=10_000,
            near_obstacle_latched=False,
            raw_ir_proximity=52,
        )

        decision = self.supervisor.decide(
            snapshot(clearance=physical_ir),
            goal(),
            (stamped(proposal()),),
        )

        self.assertEqual(decision.kind, "STOP")
        self.assertEqual(
            decision.reason_code,
            "forward_clearance_unknown",
        )

    def test_stale_snapshot_and_safety_evidence_fail_closed(self):
        self.now_ms = 10_201
        stale_state = self.supervisor.decide(
            snapshot(now_ms=10_000),
            goal(),
            (),
        )
        fresh_state_stale_safety = snapshot(
            now_ms=10_201,
            clearance=ClearanceEvidence(
                source="simulation_metric",
                observed_at_ms=10_100,
                near_obstacle_latched=False,
                forward_mm=500,
            ),
        )
        stale_safety = self.supervisor.decide(
            fresh_state_stale_safety,
            goal(),
            (),
        )

        self.assertEqual(
            stale_state.reason_code,
            "stale_navigation_snapshot",
        )
        self.assertEqual(
            stale_safety.reason_code,
            "stale_safety_evidence",
        )

    def test_stale_explicit_touch_or_fault_still_latches(self):
        self.now_ms = 10_201
        stale_touch = snapshot(
            now_ms=10_000,
            touch_pressed=True,
        )

        hazard = self.supervisor.decide(
            stale_touch,
            goal(),
            (),
        )
        later = self.supervisor.decide(
            snapshot(now_ms=10_201, state_version=5),
            goal(),
            (
                stamped(
                    proposal(
                        "later-drive",
                        state_version=5,
                    ),
                    received_at_ms=10_201,
                    valid_until_ms=10_301,
                ),
            ),
        )

        self.assertEqual(hazard.reason_code, "emergency_stop_latched")
        self.assertEqual(later.reason_code, "emergency_stop_latched")
        self.assertTrue(self.supervisor.emergency_latched)

    def test_future_proposal_and_wrong_world_version_do_not_move(self):
        future_snapshot = self.supervisor.decide(
            snapshot(now_ms=10_001),
            goal(),
            (),
        )
        future = self.supervisor.decide(
            snapshot(),
            goal(),
            (
                stamped(
                    proposal(),
                    received_at_ms=10_001,
                    valid_until_ms=10_201,
                ),
            ),
        )
        wrong_world = self.supervisor.decide(
            replace(snapshot(state_version=5), world_model_version=3),
            goal(),
            (
                stamped(
                    proposal(
                        "wrong-world",
                        state_version=5,
                    ),
                    sequence=2,
                ),
            ),
        )

        self.assertEqual(
            future_snapshot.reason_code,
            "stale_navigation_snapshot",
        )
        self.assertEqual(future.kind, "STOP")
        self.assertEqual(wrong_world.kind, "STOP")

    def test_lower_clearance_never_opens_a_longer_forward_pulse(self):
        exact_evidence = ClearanceEvidence(
            source="simulation_metric",
            observed_at_ms=10_000,
            near_obstacle_latched=False,
            forward_mm=82,
        )
        lower_evidence = replace(exact_evidence, forward_mm=81)
        exact = self.supervisor.decide(
            snapshot(clearance=exact_evidence),
            goal(),
            (stamped(proposal()),),
        )
        lower = self.supervisor.decide(
            snapshot(state_version=5, clearance=lower_evidence),
            goal(),
            (
                stamped(
                    proposal(
                        "lower-clearance",
                        state_version=5,
                    ),
                    sequence=2,
                ),
            ),
        )

        self.assertEqual(exact.kind, "DRIVE")
        self.assertEqual(lower.kind, "STOP")
        self.assertEqual(
            lower.reason_code,
            "forward_clearance_insufficient",
        )
        latched_evidence = ClearanceEvidence(
            source="simulation_metric",
            observed_at_ms=10_000,
            near_obstacle_latched=True,
            forward_mm=500,
        )
        ids = itertools.count(1)
        latched_supervisor = MotionSupervisor(
            simulation_profile(),
            lambda: self.now_ms,
            "ev3rstorm-sim",
            "nav-sim-instance-1",
            MotionAuthority(),
            id_factory=lambda: "latched-{}".format(next(ids)),
        )
        latched = latched_supervisor.decide(
            snapshot(clearance=latched_evidence),
            goal(),
            (stamped(proposal("latched-clearance")),),
        )
        self.assertEqual(latched.kind, "STOP")
        self.assertEqual(
            latched.reason_code,
            "forward_clearance_unknown",
        )

    def test_ttl_equality_old_epoch_and_old_state_do_not_move(self):
        expired = self.supervisor.decide(
            snapshot(),
            goal(),
            (
                stamped(
                    proposal(),
                    received_at_ms=self.now_ms - 1,
                    valid_until_ms=self.now_ms,
                ),
            ),
        )
        old_epoch = self.supervisor.decide(
            snapshot(state_version=5),
            goal(),
            (
                stamped(
                    proposal(
                        "proposal-2",
                        epoch=2,
                        state_version=5,
                    ),
                    sequence=2,
                ),
            ),
        )
        old_state = self.supervisor.decide(
            snapshot(state_version=6),
            goal(),
            (
                stamped(
                    proposal(
                        "proposal-3",
                        state_version=5,
                    ),
                    sequence=3,
                ),
            ),
        )

        self.assertEqual(expired.kind, "STOP")
        self.assertEqual(old_epoch.kind, "STOP")
        self.assertEqual(old_state.kind, "STOP")

    def test_touch_latches_until_new_epoch_has_stable_safe_samples(self):
        touched = snapshot(touch_pressed=True)
        first = self.supervisor.decide(touched, goal(), ())
        still_latched = self.supervisor.decide(snapshot(), goal(), ())

        self.assertEqual(first.reason_code, "emergency_stop_latched")
        self.assertEqual(
            still_latched.reason_code,
            "emergency_stop_latched",
        )
        self.now_ms = 10_020
        new_goal = goal(epoch=2)
        safe_one = snapshot(
            now_ms=10_010,
            epoch=2,
            state_version=5,
        )
        safe_two = snapshot(
            now_ms=10_020,
            epoch=2,
            state_version=6,
        )
        self.assertFalse(
            self.supervisor.request_rearm(new_goal, (safe_one,))
        )
        self.assertTrue(
            self.supervisor.request_rearm(
                new_goal,
                (safe_one, safe_two),
            )
        )
        self.assertFalse(self.supervisor.emergency_latched)

    def test_equal_rank_conflict_stops_in_every_arrival_order(self):
        left = stamped(
            proposal(
                "left",
                segment=TurnSegment(30_000, 75_000),
            ),
            source_id="planner-a",
        )
        right = stamped(
            proposal(
                "right",
                segment=TurnSegment(-30_000, 75_000),
            ),
            source_id="planner-b",
        )

        reasons = set()
        for order in ((left, right), (right, left)):
            ids = itertools.count(1)
            supervisor = MotionSupervisor(
                simulation_profile(),
                lambda: self.now_ms,
                "ev3rstorm-sim",
                "nav-sim-instance-1",
                MotionAuthority(),
                id_factory=lambda: "permutation-{}".format(
                    next(ids)
                ),
            )
            decision = supervisor.decide(
                snapshot(),
                goal(),
                order,
            )
            reasons.add((decision.kind, decision.reason_code))

        self.assertEqual(
            reasons,
            {("STOP", "ambiguous_top_priority")},
        )

    def test_duplicate_source_sequence_batch_fails_closed(self):
        first = stamped(
            proposal("first"),
            source_id="same-source",
            sequence=7,
        )
        second = stamped(
            proposal("second"),
            source_id="same-source",
            sequence=7,
        )

        for order in ((first, second), (second, first)):
            ids = itertools.count(1)
            supervisor = MotionSupervisor(
                simulation_profile(),
                lambda: self.now_ms,
                "ev3rstorm-sim",
                "nav-sim-instance-1",
                MotionAuthority(),
                id_factory=lambda: "sequence-{}".format(next(ids)),
            )
            decision = supervisor.decide(
                snapshot(),
                goal(),
                order,
            )
            self.assertEqual(decision.kind, "STOP")
            self.assertEqual(
                decision.reason_code,
                "no_fresh_proposal",
            )

    def test_equivalent_top_proposals_choose_canonical_id(self):
        one = stamped(
            proposal("z-proposal"),
            source_id="z-source",
        )
        two = stamped(
            proposal("a-proposal"),
            source_id="a-source",
        )

        decision = self.supervisor.decide(
            snapshot(),
            goal(),
            (one, two),
        )

        self.assertEqual(decision.kind, "DRIVE")
        self.assertEqual(decision.proposal_id, "a-proposal")

    def test_higher_authority_avoidance_turn_wins(self):
        goal_drive = stamped(proposal(), authority_rank=10)
        avoidance = stamped(
            proposal(
                "avoid",
                segment=TurnSegment(30_000, 75_000),
            ),
            source_id="obstacle-avoidance",
            authority_rank=20,
            priority=100,
        )

        decision = self.supervisor.decide(
            snapshot(),
            goal(),
            (goal_drive, avoidance),
        )

        self.assertEqual(decision.reason_code, "authorized_turn")
        self.assertEqual(decision.proposal_id, "avoid")

    def test_supervisor_refuses_provisional_physical_profile(self):
        provisional = replace(
            simulation_profile(),
            calibration_id="physical-provisional",
            status="provisional",
            encoder_mdeg_per_mm=None,
        )

        with self.assertRaises(NavigationContractError) as caught:
            MotionSupervisor(
                provisional,
                lambda: self.now_ms,
                "ev3rstorm-sim",
                "nav-sim-instance-1",
                MotionAuthority(),
            )

        self.assertEqual(
            caught.exception.code,
            "physical_navigation_disabled",
        )


if __name__ == "__main__":
    unittest.main()
