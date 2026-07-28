from dataclasses import replace
import itertools
import subprocess
import sys
import threading
import unittest

from robot_agent.navigation_contract import (
    DriveCalibrationProfile,
    DrivePulse,
    MotionAuthority,
    NavigationContractError,
    WaypointGoal,
)
from robot_agent.navigation_episode import (
    NAVIGATION_ABORTED,
    NAVIGATION_BUDGET_EXHAUSTED,
    NAVIGATION_EXECUTION_FAILED,
    NAVIGATION_GOAL_REACHED,
    NAVIGATION_PROGRESS_FAILED,
    NAVIGATION_SAFETY_STOP,
    GoalSeekingBehavior,
    NavigationEpisode,
    NavigationLimits,
    ObstacleAvoidanceBehavior,
)
from robot_agent.navigation_simulator import (
    CircleObstacle,
    DifferentialDriveSimulator,
    SimulationSettings,
    SimulationWorld,
)
from robot_agent.navigation_state import (
    PoseEstimate,
    ProposalInbox,
    ProposalSourcePolicy,
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


def waypoint(x_mm=1_000, y_mm=300):
    return WaypointGoal(
        goal_id="sim-waypoint",
        goal_epoch=1,
        plan_revision=1,
        target_x_mm=x_mm,
        target_y_mm=y_mm,
        tolerance_mm=30,
    )


def make_stack(
    obstacles=(),
    plant_class=DifferentialDriveSimulator,
    motion_authority=None,
):
    ids = itertools.count(1)
    if motion_authority is None:
        motion_authority = MotionAuthority()
    plant = plant_class(
        SimulationWorld(
            width_mm=1_300,
            height_mm=700,
            obstacles=tuple(obstacles),
        ),
        simulation_profile(),
        PoseEstimate(150, 300, 0),
        motion_authority,
    )
    supervisor = MotionSupervisor(
        plant.profile,
        plant.clock_ms,
        plant.robot_id,
        plant.controller_instance_id,
        motion_authority,
        policy=MotionPolicy(
            max_snapshot_age_ms=200,
            max_safety_age_ms=100,
            max_proposal_ttl_ms=500,
            max_pulse_ms=120,
            max_linear_speed_mm_s=120,
            max_angular_speed_mdeg_s=90_000,
            forward_reserve_mm=70,
        ),
        id_factory=lambda: "decision-{}".format(next(ids)),
    )
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
        plant.clock_ms,
    )
    return plant, supervisor, inbox, motion_authority


class StallingSimulator(DifferentialDriveSimulator):
    def apply(self, pulse, goal):
        before = self.observe(goal)
        after = super().apply(pulse, goal)
        if pulse.kind == "DRIVE":
            return replace(
                after,
                pose=before.pose,
                left_encoder_mdeg=before.left_encoder_mdeg,
                right_encoder_mdeg=before.right_encoder_mdeg,
            )
        return after


class BrokenStopSimulator(DifferentialDriveSimulator):
    def apply(self, pulse, goal):
        after = super().apply(pulse, goal)
        if pulse.kind == "STOP":
            return replace(after, motors_running=True)
        return after


class FaultAfterDriveSimulator(DifferentialDriveSimulator):
    def apply(self, pulse, goal):
        after = super().apply(pulse, goal)
        if pulse.kind == "DRIVE":
            return replace(
                after,
                active_faults=("injected_motor_fault",),
            )
        return after


class FaultAtGoalSimulator(DifferentialDriveSimulator):
    def observe(self, goal):
        return replace(
            super().observe(goal),
            active_faults=("injected_motor_fault",),
        )


class NavigationSimulatorTests(unittest.TestCase):
    def test_forward_ray_reports_nearest_obstacle_identity(self):
        nearer = CircleObstacle(
            obstacle_id="near-box",
            x_mm=400,
            y_mm=300,
            radius_mm=30,
        )
        farther = CircleObstacle(
            obstacle_id="far-box",
            x_mm=700,
            y_mm=300,
            radius_mm=30,
        )
        plant, _supervisor, _inbox, _authority = make_stack(
            (farther, nearer)
        )

        observed = plant.observe(waypoint())

        self.assertEqual(observed.clearance.forward_mm, 155)
        self.assertEqual(
            observed.clearance.forward_object_id,
            "near-box",
        )

    def test_forward_object_identity_is_none_for_clear_space_and_wall(self):
        clear_plant, _supervisor, _inbox, _authority = make_stack()

        clear_observation = clear_plant.observe(waypoint())

        self.assertEqual(clear_observation.clearance.forward_mm, 1_000)
        self.assertIsNone(
            clear_observation.clearance.forward_object_id
        )

        wall_plant = DifferentialDriveSimulator(
            SimulationWorld(width_mm=500, height_mm=700),
            simulation_profile(),
            PoseEstimate(150, 300, 0),
            MotionAuthority(),
        )

        wall_observation = wall_plant.observe(waypoint())

        self.assertEqual(wall_observation.clearance.forward_mm, 285)
        self.assertIsNone(
            wall_observation.clearance.forward_object_id
        )

    def test_world_update_changes_forward_object_identity(self):
        first = CircleObstacle(
            obstacle_id="first-box",
            x_mm=400,
            y_mm=300,
            radius_mm=30,
        )
        second = CircleObstacle(
            obstacle_id="second-box",
            x_mm=400,
            y_mm=300,
            radius_mm=30,
        )
        plant, _supervisor, _inbox, _authority = make_stack((first,))

        before = plant.observe(waypoint())
        plant.update_world(
            SimulationWorld(
                width_mm=1_300,
                height_mm=700,
                obstacles=(second,),
            )
        )
        after = plant.observe(waypoint())

        self.assertEqual(
            before.clearance.forward_object_id,
            "first-box",
        )
        self.assertEqual(
            after.clearance.forward_object_id,
            "second-box",
        )
        self.assertEqual(
            after.clearance.forward_mm,
            before.clearance.forward_mm,
        )
        self.assertGreater(
            after.world_model_version,
            before.world_model_version,
        )

    def test_goal_seeking_reaches_waypoint_and_verifies_terminal_stop(self):
        plant, supervisor, inbox, _authority = make_stack()
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (
                GoalSeekingBehavior(),
                ObstacleAvoidanceBehavior(),
            ),
        )

        result = episode.run(waypoint())

        self.assertTrue(result.completed)
        self.assertEqual(result.termination, NAVIGATION_GOAL_REACHED)
        self.assertTrue(result.terminal_stop_verified)
        self.assertEqual(plant.collision_count, 0)
        self.assertGreater(result.actions, 1)
        self.assertTrue(
            all(
                pulse.duration_ms <= 120
                for pulse in plant.applied_pulses
                if pulse.kind == "DRIVE"
            )
        )

    def test_new_goal_epoch_runs_with_fresh_behavior_instances(self):
        plant, supervisor, inbox, _authority = make_stack()
        first_episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
        )
        first = first_episode.run(waypoint())
        second_goal = replace(
            waypoint(x_mm=700),
            goal_id="sim-waypoint-2",
            goal_epoch=2,
        )
        second_episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
        )

        second = second_episode.run(second_goal)

        self.assertTrue(first.completed)
        self.assertTrue(second.completed, second.to_dict())
        self.assertGreater(second.actions, 0)
        self.assertEqual(
            second.termination,
            NAVIGATION_GOAL_REACHED,
        )

    def test_reactive_avoidance_routes_around_one_obstacle(self):
        obstacle = CircleObstacle(
            obstacle_id="box",
            x_mm=600,
            y_mm=300,
            radius_mm=70,
        )
        plant, supervisor, inbox, _authority = make_stack((obstacle,))
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (
                GoalSeekingBehavior(),
                ObstacleAvoidanceBehavior(),
            ),
            limits=NavigationLimits(
                max_ticks=500,
                max_elapsed_ms=60_000,
                max_proposals=1_000,
                max_replans=500,
                max_actions=480,
                max_total_motion_ms=55_000,
                max_no_progress_ticks=120,
            ),
        )

        result = episode.run(waypoint())

        self.assertTrue(result.completed, result.to_dict())
        self.assertEqual(plant.collision_count, 0)
        self.assertIn(
            "authorized_turn",
            {step.decision_reason for step in result.steps},
        )
        self.assertTrue(
            any(
                step.proposal_ids
                and any(
                    value.startswith("obstacle-avoidance-")
                    for value in step.proposal_ids
                )
                for step in result.steps
            )
        )

    def test_swept_collision_oracle_prevents_tunneling(self):
        obstacle = CircleObstacle(
            obstacle_id="thin-target",
            x_mm=230,
            y_mm=300,
            radius_mm=1,
        )
        plant, _supervisor, _inbox, authority = make_stack((obstacle,))
        goal = waypoint()
        before = plant.observe(goal)
        pulse = DrivePulse(
            decision_id="forged-but-well-typed",
            arbiter_id=plant.expected_arbiter_id,
            robot_id=plant.robot_id,
            controller_instance_id=plant.controller_instance_id,
            goal_id=goal.goal_id,
            goal_epoch=goal.goal_epoch,
            plan_revision=goal.plan_revision,
            based_on_state_version=before.state_version,
            based_on_world_model_version=before.world_model_version,
            kind="DRIVE",
            left_speed_dps=250,
            right_speed_dps=250,
            duration_ms=120,
            reason_code="test_oracle",
        )
        authority.authorize(pulse)

        after = plant.apply(pulse, goal)

        self.assertEqual(plant.collision_count, 1)
        self.assertTrue(after.touch_pressed)
        self.assertIn("collision_oracle", after.active_faults)
        self.assertLessEqual(after.pose.x_mm, 164)

    def test_motion_bus_rejects_replayed_decision(self):
        plant, _supervisor, _inbox, authority = make_stack()
        goal = waypoint()
        before = plant.observe(goal)
        pulse = DrivePulse(
            decision_id="one-shot-decision",
            arbiter_id=plant.expected_arbiter_id,
            robot_id=plant.robot_id,
            controller_instance_id=plant.controller_instance_id,
            goal_id=goal.goal_id,
            goal_epoch=goal.goal_epoch,
            plan_revision=goal.plan_revision,
            based_on_state_version=before.state_version,
            based_on_world_model_version=before.world_model_version,
            kind="STOP",
            left_speed_dps=0,
            right_speed_dps=0,
            duration_ms=0,
            reason_code="test_stop",
        )
        authority.authorize(pulse)
        plant.apply(pulse, goal)
        self.assertFalse(
            hasattr(
                plant.applied_pulses[-1],
                "authorization_token",
            )
        )

        with self.assertRaises(NavigationContractError) as caught:
            plant.apply(pulse, goal)

        self.assertEqual(caught.exception.code, "replayed_drive_pulse")

    def test_motion_bus_requires_unforgeable_authority_identity(self):
        plant, _supervisor, _inbox, _authority = make_stack()
        goal = waypoint()
        before = plant.observe(goal)
        spoofed = DrivePulse(
            decision_id="spoofed-decision",
            arbiter_id=plant.expected_arbiter_id,
            robot_id=plant.robot_id,
            controller_instance_id=plant.controller_instance_id,
            goal_id=goal.goal_id,
            goal_epoch=goal.goal_epoch,
            plan_revision=goal.plan_revision,
            based_on_state_version=before.state_version,
            based_on_world_model_version=before.world_model_version,
            kind="STOP",
            left_speed_dps=0,
            right_speed_dps=0,
            duration_ms=0,
            reason_code="spoofed_stop",
        )

        with self.assertRaises(NavigationContractError) as caught:
            plant.apply(spoofed, goal)

        self.assertEqual(
            caught.exception.code,
            "unauthorized_motion_owner",
        )
        self.assertFalse(
            hasattr(spoofed, "authorization_token")
        )

    def test_motion_bus_serializes_concurrent_authorized_pulses(self):
        plant, supervisor, inbox, _authority = make_stack()
        goal = waypoint()
        before = plant.observe(goal)
        producer = GoalSeekingBehavior()
        inbox.publish_host(
            producer.propose(goal, before),
            producer.source_id,
        )
        first = supervisor.decide(before, goal, inbox.drain())
        inbox.publish_host(
            producer.propose(goal, before),
            producer.source_id,
        )
        second = supervisor.decide(before, goal, inbox.drain())
        pulses = (first, second)
        barrier = threading.Barrier(3)
        outcomes = []

        def apply_pulse(pulse):
            barrier.wait()
            try:
                plant.apply(pulse, goal)
                outcomes.append("applied")
            except NavigationContractError as error:
                outcomes.append(error.code)

        threads = [
            threading.Thread(target=apply_pulse, args=(pulse,))
            for pulse in pulses
        ]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join()

        self.assertEqual(
            sorted(outcomes),
            ["applied", "stale_drive_pulse"],
        )
        self.assertEqual(len(plant.applied_pulses), 1)

    def test_drive_pulse_is_bound_to_plan_revision(self):
        plant, _supervisor, _inbox, authority = make_stack()
        revision_one = waypoint()
        before = plant.observe(revision_one)
        pulse = DrivePulse(
            decision_id="revision-one-decision",
            arbiter_id=plant.expected_arbiter_id,
            robot_id=plant.robot_id,
            controller_instance_id=plant.controller_instance_id,
            goal_id=revision_one.goal_id,
            goal_epoch=revision_one.goal_epoch,
            plan_revision=revision_one.plan_revision,
            based_on_state_version=before.state_version,
            based_on_world_model_version=before.world_model_version,
            kind="DRIVE",
            left_speed_dps=100,
            right_speed_dps=100,
            duration_ms=120,
            reason_code="test_revision_binding",
        )
        authority.authorize(pulse)
        revision_two = replace(revision_one, plan_revision=2)

        with self.assertRaises(NavigationContractError) as caught:
            plant.apply(pulse, revision_two)

        self.assertEqual(caught.exception.code, "stale_drive_pulse")
        self.assertEqual(
            plant.observe(revision_one).pose,
            before.pose,
        )

    def test_world_update_invalidates_already_authorized_pulse(self):
        plant, supervisor, _inbox, _authority = make_stack()
        goal = waypoint()
        before = plant.observe(goal)
        authorized = supervisor.force_stop(before, "world-binding-test")
        plant.update_world(
            SimulationWorld(
                width_mm=1_300,
                height_mm=700,
                obstacles=(
                    CircleObstacle(
                        "new-obstacle",
                        x_mm=800,
                        y_mm=500,
                        radius_mm=20,
                    ),
                ),
            )
        )

        with self.assertRaises(NavigationContractError) as caught:
            plant.apply(authorized, goal)

        self.assertEqual(caught.exception.code, "stale_drive_pulse")

    def test_stale_dispatch_reobserves_and_routes_around_new_obstacle(self):
        plant, supervisor, inbox, _authority = make_stack()
        observations = []
        updates = []

        def add_obstacle_once(snapshot):
            if updates:
                return
            updates.append(snapshot.state_version)
            plant.update_world(
                SimulationWorld(
                    width_mm=1_300,
                    height_mm=700,
                    obstacles=(
                        CircleObstacle(
                            obstacle_id="appeared-box",
                            x_mm=600,
                            y_mm=300,
                            radius_mm=70,
                        ),
                    ),
                )
            )

        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (
                GoalSeekingBehavior(),
                ObstacleAvoidanceBehavior(),
            ),
            limits=NavigationLimits(
                max_ticks=500,
                max_elapsed_ms=60_000,
                max_proposals=1_000,
                max_replans=500,
                max_actions=480,
                max_total_motion_ms=55_000,
                max_no_progress_ticks=120,
            ),
            observation_sink=observations.append,
            before_arbitration=add_obstacle_once,
        )

        result = episode.run(waypoint())

        self.assertTrue(result.completed, result.to_dict())
        self.assertEqual(result.termination, NAVIGATION_GOAL_REACHED)
        self.assertTrue(result.terminal_stop_verified)
        self.assertEqual(plant.collision_count, 0)
        self.assertEqual(updates, [1])
        self.assertEqual(
            [
                (
                    value.state_version,
                    value.world_model_version,
                )
                for value in observations[:2]
            ],
            [(1, 1), (2, 2)],
        )
        self.assertIn("STALE_DISPATCH", result.trace)
        self.assertIn("STALE_DISPATCH_REOBSERVED", result.trace)
        self.assertTrue(
            all(
                pulse.based_on_world_model_version == 2
                for pulse in plant.applied_pulses
            )
        )
        self.assertEqual(
            plant.applied_pulses[0].based_on_state_version,
            2,
        )
        self.assertEqual(plant.applied_pulses[-1].kind, "STOP")

    def test_repeated_stale_dispatches_exhaust_replans_with_verified_stop(
        self,
    ):
        plant, supervisor, inbox, _authority = make_stack()
        observed_versions = []
        update_count = [0]

        def invalidate_every_dispatch(_snapshot):
            update_count[0] += 1
            plant.update_world(
                SimulationWorld(
                    width_mm=1_300,
                    height_mm=700,
                    obstacles=(),
                )
            )

        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
            limits=NavigationLimits(
                max_ticks=10,
                max_elapsed_ms=10_000,
                max_proposals=10,
                max_replans=2,
                max_actions=10,
                max_total_motion_ms=1_000,
                max_no_progress_ticks=10,
            ),
            observation_sink=lambda value: (
                observed_versions.append(value.state_version)
            ),
            before_arbitration=invalidate_every_dispatch,
        )

        result = episode.run(waypoint())

        self.assertFalse(result.completed)
        self.assertEqual(
            result.termination,
            NAVIGATION_BUDGET_EXHAUSTED,
        )
        self.assertTrue(result.terminal_stop_verified)
        self.assertEqual(result.ticks, 0)
        self.assertEqual(result.actions, 0)
        self.assertEqual(result.replans, 2)
        self.assertEqual(update_count[0], 3)
        self.assertEqual(observed_versions, [1, 2, 3, 4])
        self.assertEqual(plant.collision_count, 0)
        self.assertEqual(
            result.trace.count("STALE_DISPATCH"),
            3,
        )
        self.assertEqual(
            result.trace.count("STALE_DISPATCH_REOBSERVED"),
            3,
        )
        self.assertEqual(
            [pulse.kind for pulse in plant.applied_pulses],
            ["STOP"],
        )
        self.assertEqual(
            plant.applied_pulses[-1].based_on_state_version,
            4,
        )
        self.assertEqual(
            result.final_snapshot.state_version,
            5,
        )

    def test_stall_is_detected_on_first_motion_segment(self):
        plant, supervisor, inbox, _authority = make_stack(
            plant_class=StallingSimulator
        )
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
        )

        result = episode.run(waypoint())

        self.assertFalse(result.completed)
        self.assertEqual(
            result.termination,
            NAVIGATION_PROGRESS_FAILED,
        )
        self.assertEqual(result.actions, 1)
        self.assertTrue(result.terminal_stop_verified)

    def test_fault_after_drive_outranks_progress_classification(self):
        plant, supervisor, inbox, _authority = make_stack(
            plant_class=FaultAfterDriveSimulator
        )
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
        )

        result = episode.run(waypoint())

        self.assertFalse(result.completed)
        self.assertEqual(result.termination, NAVIGATION_SAFETY_STOP)
        self.assertTrue(result.terminal_stop_verified)
        self.assertTrue(supervisor.emergency_latched)
        drive_count = sum(
            pulse.kind == "DRIVE" for pulse in plant.applied_pulses
        )

        retry = episode.run(waypoint())

        self.assertEqual(retry.termination, NAVIGATION_SAFETY_STOP)
        self.assertEqual(retry.actions, 0)
        self.assertEqual(
            sum(
                pulse.kind == "DRIVE"
                for pulse in plant.applied_pulses
            ),
            drive_count,
        )

    def test_fault_at_goal_is_safety_stop_not_goal_reached(self):
        plant, supervisor, inbox, _authority = make_stack(
            plant_class=FaultAtGoalSimulator
        )
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
            limits=NavigationLimits(max_replans=0),
        )
        goal = waypoint(x_mm=150, y_mm=300)

        result = episode.run(goal)

        self.assertFalse(result.completed)
        self.assertEqual(result.termination, NAVIGATION_SAFETY_STOP)
        self.assertTrue(result.terminal_stop_verified)

    def test_failed_stop_observation_is_never_marked_verified(self):
        plant, supervisor, inbox, _authority = make_stack(
            plant_class=BrokenStopSimulator
        )
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (),
            limits=NavigationLimits(
                max_ticks=1,
                max_elapsed_ms=1_000,
                max_proposals=2,
                max_replans=1,
                max_actions=1,
                max_total_motion_ms=120,
                max_no_progress_ticks=1,
            ),
        )

        result = episode.run(waypoint())

        self.assertFalse(result.completed)
        self.assertFalse(result.terminal_stop_verified)
        self.assertEqual(
            result.termination,
            NAVIGATION_EXECUTION_FAILED,
        )
        self.assertTrue(result.final_snapshot.motors_running)

    def test_empty_producer_loop_is_bounded_and_stops(self):
        plant, supervisor, inbox, _authority = make_stack()
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (),
            limits=NavigationLimits(
                max_ticks=3,
                max_elapsed_ms=10_000,
                max_proposals=10,
                max_replans=3,
                max_actions=3,
                max_total_motion_ms=1_000,
                max_no_progress_ticks=10,
            ),
        )

        result = episode.run(waypoint())

        self.assertFalse(result.completed)
        self.assertEqual(
            result.termination,
            NAVIGATION_BUDGET_EXHAUSTED,
        )
        self.assertEqual(result.ticks, 3)
        self.assertTrue(result.terminal_stop_verified)
        self.assertTrue(
            all(
                pulse.kind == "STOP"
                for pulse in plant.applied_pulses
            )
        )

    def test_runtime_hooks_observe_each_committed_tick_before_arbitration(self):
        plant, supervisor, inbox, _authority = make_stack()
        observations = []
        arbitration_versions = []
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
            limits=NavigationLimits(
                max_ticks=2,
                max_elapsed_ms=10_000,
                max_proposals=10,
                max_replans=2,
                max_actions=2,
                max_total_motion_ms=1_000,
                max_no_progress_ticks=10,
            ),
            observation_sink=observations.append,
            before_arbitration=lambda value: (
                arbitration_versions.append(value.state_version)
            ),
        )

        result = episode.run(waypoint())

        self.assertEqual(result.ticks, 2)
        self.assertEqual(
            [value.state_version for value in observations],
            [1, 2, 3],
        )
        self.assertEqual(arbitration_versions, [1, 2])

    def test_post_commit_sink_failure_accounts_and_stops_current_state(
        self,
    ):
        plant, supervisor, inbox, _authority = make_stack()
        observed_versions = []

        def fail_after_first_drive(snapshot):
            observed_versions.append(snapshot.state_version)
            if len(observed_versions) == 2:
                raise RuntimeError("synthetic observation delivery failure")

        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
            observation_sink=fail_after_first_drive,
        )

        result = episode.run(waypoint())
        current = plant.observe(waypoint())
        pulses = plant.applied_pulses

        self.assertEqual(result.termination, NAVIGATION_ABORTED)
        self.assertEqual(observed_versions, [1, 2])
        self.assertEqual(result.ticks, 1)
        self.assertEqual(result.actions, 1)
        self.assertEqual(result.total_motion_ms, pulses[0].duration_ms)
        self.assertEqual(len(result.steps), 1)
        self.assertEqual(result.steps[0].decision_kind, "DRIVE")
        self.assertEqual(
            result.steps[0].state_after,
            observed_versions[-1],
        )
        self.assertEqual([pulse.kind for pulse in pulses], ["DRIVE", "STOP"])
        self.assertTrue(result.terminal_stop_verified)
        self.assertFalse(result.final_snapshot.motors_running)
        self.assertEqual(
            result.final_snapshot.state_version,
            current.state_version,
        )
        self.assertEqual(result.final_snapshot.pose, current.pose)
        self.assertIn("OBSERVATION_SINK_FAILED", result.trace)

    def test_initial_sink_failure_aborts_without_a_navigation_action(self):
        plant, supervisor, inbox, _authority = make_stack()

        def fail_initial_observation(_snapshot):
            raise RuntimeError("synthetic initial delivery failure")

        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
            observation_sink=fail_initial_observation,
        )

        result = episode.run(waypoint())

        self.assertEqual(result.termination, NAVIGATION_ABORTED)
        self.assertEqual(result.ticks, 0)
        self.assertEqual(result.actions, 0)
        self.assertEqual(result.total_motion_ms, 0)
        self.assertEqual(
            [pulse.kind for pulse in plant.applied_pulses],
            ["STOP"],
        )
        self.assertTrue(result.terminal_stop_verified)

    def test_cancel_event_stops_before_any_navigation_action(self):
        plant, supervisor, inbox, _authority = make_stack()
        cancel_event = threading.Event()
        cancel_event.set()
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
            cancel_event=cancel_event,
        )

        result = episode.run(waypoint())

        self.assertEqual(result.termination, NAVIGATION_ABORTED)
        self.assertEqual(result.actions, 0)
        self.assertTrue(result.terminal_stop_verified)
        self.assertIn("CANCELLED", result.trace)

    def test_async_inbox_proposals_count_toward_episode_budget(self):
        plant, supervisor, inbox, _authority = make_stack()
        goal = waypoint()
        current = plant.observe(goal)
        producer = GoalSeekingBehavior()
        inbox.publish(
            producer.propose(goal, current),
            producer.source_id,
            1,
        )
        inbox.publish(
            producer.propose(goal, current),
            producer.source_id,
            2,
        )
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (),
            limits=NavigationLimits(
                max_ticks=5,
                max_elapsed_ms=1_000,
                max_proposals=1,
                max_replans=5,
                max_actions=5,
                max_total_motion_ms=500,
                max_no_progress_ticks=5,
            ),
        )

        result = episode.run(goal)

        self.assertEqual(
            result.termination,
            NAVIGATION_BUDGET_EXHAUSTED,
        )
        self.assertEqual(result.proposals, 2)
        self.assertEqual(result.actions, 0)
        self.assertTrue(result.terminal_stop_verified)

    def test_rejected_motion_budget_cancels_one_shot_authority(self):
        authority = MotionAuthority(max_pending=1, replay_window=4)
        plant, supervisor, inbox, _authority = make_stack(
            motion_authority=authority
        )
        episode = NavigationEpisode(
            plant,
            supervisor,
            inbox,
            (GoalSeekingBehavior(),),
            limits=NavigationLimits(
                max_ticks=5,
                max_elapsed_ms=1_000,
                max_proposals=5,
                max_replans=5,
                max_actions=5,
                max_total_motion_ms=1,
                max_no_progress_ticks=5,
            ),
        )

        results = [episode.run(waypoint()) for _ in range(3)]

        self.assertTrue(
            all(
                result.termination
                == NAVIGATION_BUDGET_EXHAUSTED
                for result in results
            )
        )
        self.assertTrue(
            all(result.terminal_stop_verified for result in results)
        )
        self.assertTrue(all(result.actions == 0 for result in results))
        self.assertEqual(len(authority._pending), 0)

    def test_unknown_inline_producer_is_rejected(self):
        plant, supervisor, inbox, _authority = make_stack()

        class SlowOrExternalProducer:
            source_id = "external"

            def propose(self, _goal, _snapshot):
                raise RuntimeError("must never run in motion tick")

        with self.assertRaises(NavigationContractError) as caught:
            NavigationEpisode(
                plant,
                supervisor,
                inbox,
                (SlowOrExternalProducer(),),
            )

        self.assertEqual(
            caught.exception.code,
            "invalid_navigation_behaviors",
        )

    def test_navigation_import_has_no_physical_transport_side_effect(self):
        code = (
            "import sys;"
            "import robot_agent.navigation_episode;"
            "import robot_agent.navigation_mission;"
            "assert 'robot_agent.supervisor_transport' not in sys.modules;"
            "assert 'robot_agent.robot_api' not in sys.modules"
        )
        completed = subprocess.run(
            [sys.executable, "-c", code],
            check=False,
            env={
                "PYTHONPATH": "src",
                "PYTHONDONTWRITEBYTECODE": "1",
            },
            capture_output=True,
            text=True,
        )

        self.assertEqual(
            completed.returncode,
            0,
            completed.stderr,
        )


if __name__ == "__main__":
    unittest.main()
