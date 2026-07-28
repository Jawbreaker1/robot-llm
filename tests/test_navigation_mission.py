from dataclasses import replace
import itertools
import json
import threading
import unittest

from robot_agent.navigation_contract import (
    DriveCalibrationProfile,
    MotionAuthority,
    NavigationContractError,
    WaypointGoal,
)
from robot_agent.navigation_episode import NavigationLimits
from robot_agent.navigation_mission import (
    MAX_MISSION_PLAN_BYTES,
    MISSION_ABORTED,
    MISSION_BUDGET_EXHAUSTED,
    MISSION_COMPLETED,
    MISSION_LEG_FAILED,
    MISSION_PLAN_REJECTED,
    MISSION_PLAN_STALE,
    MissionLeg,
    MissionLimits,
    MissionPlan,
    MissionRunner,
    decode_mission_plan,
)
from robot_agent.navigation_simulator import (
    DifferentialDriveSimulator,
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


STARTING_EPOCH = 20


def simulation_profile():
    return DriveCalibrationProfile(
        calibration_id="synthetic-mission-fixture-v1",
        status="simulation_only",
        surface="synthetic",
        left_motor_sign=1,
        right_motor_sign=1,
        encoder_mdeg_per_mm=1_800,
        encoder_mdeg_per_body_degree=2_000,
        max_wheel_speed_dps=250,
        max_pulse_ms=120,
    )


class WorldChangingAfterFirstStop(DifferentialDriveSimulator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.changed = False

    def apply(self, pulse, goal):
        after = super().apply(pulse, goal)
        if (
            not self.changed
            and goal.goal_id == "leg-one"
            and pulse.kind == "STOP"
            and pulse.reason_code == "terminal_stop"
        ):
            self.changed = True
            self.update_world(self.world)
        return after


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


class WorldChangingBeforeFirstDrive(DifferentialDriveSimulator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.changed = False

    def apply(self, pulse, goal):
        if pulse.kind == "DRIVE" and not self.changed:
            self.changed = True
            self.update_world(self.world)
        return super().apply(pulse, goal)


def make_stack(plant_class=DifferentialDriveSimulator):
    authority = MotionAuthority()
    plant = plant_class(
        SimulationWorld(width_mm=1_300, height_mm=700),
        simulation_profile(),
        PoseEstimate(150, 300, 0),
        authority,
    )
    decision_ids = itertools.count(1)
    supervisor = MotionSupervisor(
        plant.profile,
        plant.clock_ms,
        plant.robot_id,
        plant.controller_instance_id,
        authority,
        policy=MotionPolicy(max_pulse_ms=120),
        id_factory=lambda: "mission-decision-{}".format(
            next(decision_ids)
        ),
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
    return plant, supervisor, inbox


def mission_plan(plant, legs, revision=1):
    first = legs[0]
    binding_goal = WaypointGoal(
        goal_id=first.leg_id,
        goal_epoch=STARTING_EPOCH,
        plan_revision=revision,
        target_x_mm=first.target_x_mm,
        target_y_mm=first.target_y_mm,
        tolerance_mm=first.tolerance_mm,
    )
    snapshot = plant.observe(binding_goal)
    return MissionPlan(
        plan_id="mission-alpha",
        robot_id=snapshot.robot_id,
        controller_instance_id=snapshot.controller_instance_id,
        based_on_state_version=snapshot.state_version,
        based_on_world_model_version=snapshot.world_model_version,
        plan_revision=revision,
        legs=tuple(legs),
    )


def three_legs():
    return (
        MissionLeg("leg-one", 320, 180, 25),
        MissionLeg("leg-two", 760, 180, 25),
        MissionLeg("leg-three", 1_000, 300, 30),
    )


class MissionContractTests(unittest.TestCase):
    def test_strict_plan_round_trip_and_duplicate_key_rejection(self):
        plant, _supervisor, _inbox = make_stack()
        plan = mission_plan(plant, three_legs())
        raw = json.dumps(plan.to_dict()).encode("utf-8")

        decoded = decode_mission_plan(raw)

        self.assertEqual(decoded, plan)
        with self.assertRaises(NavigationContractError) as caught:
            decode_mission_plan(
                raw.replace(
                    b'"plan_id": "mission-alpha"',
                    (
                        b'"plan_id": "mission-alpha",'
                        b'"plan_id": "mission-replay"'
                    ),
                )
            )
        self.assertEqual(caught.exception.code, "invalid_mission_plan_json")

    def test_empty_oversize_extra_fields_and_duplicate_legs_are_rejected(self):
        plant, _supervisor, _inbox = make_stack()
        value = mission_plan(plant, three_legs()).to_dict()
        invalid_values = []
        empty = dict(value)
        empty["legs"] = []
        invalid_values.append(empty)
        extra = dict(value)
        extra["authority_rank"] = 10_000
        invalid_values.append(extra)
        duplicate = dict(value)
        duplicate["legs"] = [
            dict(value["legs"][0]),
            dict(value["legs"][0]),
        ]
        invalid_values.append(duplicate)
        for invalid in invalid_values:
            with self.subTest(value=invalid):
                with self.assertRaises(NavigationContractError):
                    decode_mission_plan(
                        json.dumps(invalid).encode("utf-8")
                    )
        with self.assertRaises(NavigationContractError) as caught:
            decode_mission_plan(b"x" * (MAX_MISSION_PLAN_BYTES + 1))
        self.assertEqual(
            caught.exception.code,
            "invalid_mission_plan_body",
        )


class MissionRunnerTests(unittest.TestCase):
    def test_three_leg_plan_reaches_every_waypoint_with_verified_stops(self):
        plant, supervisor, inbox = make_stack()
        plan = mission_plan(plant, three_legs())
        runner = MissionRunner(
            plant,
            supervisor,
            inbox,
            starting_goal_epoch=STARTING_EPOCH,
        )

        result = runner.run(plan)

        self.assertTrue(result.completed, result.to_dict())
        self.assertEqual(result.termination, MISSION_COMPLETED)
        self.assertEqual(result.legs_completed, 3)
        self.assertTrue(result.terminal_stop_verified)
        self.assertEqual(plant.collision_count, 0)
        self.assertEqual(
            [
                value.goal.goal_epoch
                for value in result.leg_results
            ],
            [20, 21, 22],
        )
        self.assertEqual(
            [
                value.goal.plan_revision
                for value in result.leg_results
            ],
            [1, 1, 1],
        )
        for leg in result.leg_results:
            matching = [
                pulse
                for pulse in plant.applied_pulses
                if pulse.goal_id == leg.goal.goal_id
                and pulse.goal_epoch == leg.goal.goal_epoch
            ]
            self.assertTrue(matching)
            self.assertEqual(matching[-1].kind, "STOP")
            self.assertTrue(leg.navigation.terminal_stop_verified)

    def test_stale_activation_is_rejected_without_drive_and_still_stops(self):
        plant, supervisor, inbox = make_stack()
        plan = mission_plan(plant, three_legs())
        stale = replace(
            plan,
            based_on_state_version=plan.based_on_state_version + 1,
        )

        result = MissionRunner(
            plant,
            supervisor,
            inbox,
            STARTING_EPOCH,
        ).run(stale)

        self.assertFalse(result.completed)
        self.assertEqual(result.termination, MISSION_PLAN_REJECTED)
        self.assertEqual(result.leg_results, ())
        self.assertTrue(result.terminal_stop_verified)
        self.assertTrue(plant.applied_pulses)
        self.assertTrue(all(
            pulse.kind == "STOP" for pulse in plant.applied_pulses
        ))

    def test_world_change_invalidates_remaining_plan_at_stopped_boundary(self):
        plant, supervisor, inbox = make_stack(
            WorldChangingAfterFirstStop
        )
        plan = mission_plan(plant, three_legs())

        result = MissionRunner(
            plant,
            supervisor,
            inbox,
            STARTING_EPOCH,
        ).run(plan)

        self.assertTrue(plant.changed)
        self.assertFalse(result.completed)
        self.assertEqual(result.termination, MISSION_PLAN_STALE)
        self.assertEqual(result.legs_completed, 1)
        self.assertEqual(len(result.leg_results), 1)
        self.assertTrue(result.terminal_stop_verified)
        self.assertFalse(result.final_snapshot.motors_running)
        self.assertFalse(any(
            pulse.goal_id == "leg-two"
            and pulse.kind == "DRIVE"
            for pulse in plant.applied_pulses
        ))

    def test_world_change_during_leg_stops_before_any_new_world_drive(self):
        plant, supervisor, inbox = make_stack(
            WorldChangingBeforeFirstDrive
        )
        plan = mission_plan(plant, three_legs())

        result = MissionRunner(
            plant,
            supervisor,
            inbox,
            STARTING_EPOCH,
        ).run(plan)

        self.assertTrue(plant.changed)
        self.assertFalse(result.completed)
        self.assertEqual(result.termination, MISSION_PLAN_STALE)
        self.assertEqual(result.actions, 0)
        self.assertTrue(result.terminal_stop_verified)
        self.assertEqual(plant.collision_count, 0)
        self.assertTrue(plant.applied_pulses)
        self.assertTrue(all(
            pulse.kind == "STOP" for pulse in plant.applied_pulses
        ))
        self.assertEqual(
            result.final_snapshot.world_model_version,
            2,
        )

    def test_failed_leg_prevents_later_legs(self):
        plant, supervisor, inbox = make_stack(StallingSimulator)
        legs = (
            MissionLeg("stalled-leg", 500, 300, 20),
            MissionLeg("must-not-run", 300, 300, 20),
        )
        plan = mission_plan(plant, legs)

        result = MissionRunner(
            plant,
            supervisor,
            inbox,
            STARTING_EPOCH,
        ).run(plan)

        self.assertFalse(result.completed)
        self.assertEqual(result.termination, MISSION_LEG_FAILED)
        self.assertEqual(len(result.leg_results), 1)
        self.assertTrue(result.terminal_stop_verified)
        self.assertFalse(any(
            pulse.goal_id == "must-not-run"
            for pulse in plant.applied_pulses
        ))

    def test_global_action_budget_is_enforced_inside_first_leg(self):
        plant, supervisor, inbox = make_stack()
        plan = mission_plan(plant, three_legs())

        result = MissionRunner(
            plant,
            supervisor,
            inbox,
            STARTING_EPOCH,
            mission_limits=MissionLimits(max_actions=3),
        ).run(plan)

        self.assertFalse(result.completed)
        self.assertEqual(
            result.termination,
            MISSION_BUDGET_EXHAUSTED,
        )
        self.assertLessEqual(result.actions, 3)
        self.assertEqual(len(result.leg_results), 1)
        self.assertTrue(result.terminal_stop_verified)
        self.assertFalse(result.final_snapshot.motors_running)

    def test_pre_cancelled_mission_executes_no_drive_and_verifies_stop(self):
        plant, supervisor, inbox = make_stack()
        plan = mission_plan(plant, three_legs())
        cancelled = threading.Event()
        cancelled.set()

        result = MissionRunner(
            plant,
            supervisor,
            inbox,
            STARTING_EPOCH,
            cancel_event=cancelled,
        ).run(plan)

        self.assertEqual(result.termination, MISSION_ABORTED)
        self.assertEqual(result.leg_results, ())
        self.assertTrue(result.terminal_stop_verified)
        self.assertTrue(all(
            pulse.kind == "STOP" for pulse in plant.applied_pulses
        ))

    def test_elapsed_budget_reserves_terminal_stop_before_drive(self):
        for max_elapsed_ms, completed, expected_elapsed in (
            (139, False, 20),
            (140, True, 140),
        ):
            with self.subTest(max_elapsed_ms=max_elapsed_ms):
                plant, supervisor, inbox = make_stack()
                plan = mission_plan(
                    plant,
                    (MissionLeg("short-leg", 162, 300, 1),),
                )

                result = MissionRunner(
                    plant,
                    supervisor,
                    inbox,
                    STARTING_EPOCH,
                    mission_limits=MissionLimits(
                        max_elapsed_ms=max_elapsed_ms
                    ),
                ).run(plan)

                self.assertEqual(result.completed, completed)
                self.assertEqual(result.elapsed_ms, expected_elapsed)
                self.assertTrue(result.terminal_stop_verified)
                if completed:
                    self.assertEqual(
                        result.termination,
                        MISSION_COMPLETED,
                    )
                    self.assertEqual(result.actions, 1)
                else:
                    self.assertEqual(
                        result.termination,
                        MISSION_BUDGET_EXHAUSTED,
                    )
                    self.assertEqual(result.actions, 0)

    def test_already_satisfied_leg_cannot_complete_past_elapsed_budget(self):
        plant, supervisor, inbox = make_stack()
        plan = mission_plan(
            plant,
            (MissionLeg("already-there", 150, 300, 1),),
        )

        result = MissionRunner(
            plant,
            supervisor,
            inbox,
            STARTING_EPOCH,
            mission_limits=MissionLimits(max_elapsed_ms=1),
        ).run(plan)

        self.assertFalse(result.completed)
        self.assertEqual(
            result.termination,
            MISSION_BUDGET_EXHAUSTED,
        )
        self.assertEqual(result.actions, 0)
        self.assertEqual(result.elapsed_ms, 20)
        self.assertTrue(result.terminal_stop_verified)

    def test_runner_is_single_use(self):
        plant, supervisor, inbox = make_stack()
        plan = mission_plan(
            plant,
            (MissionLeg("already-there", 150, 300, 10),),
        )
        runner = MissionRunner(
            plant,
            supervisor,
            inbox,
            STARTING_EPOCH,
        )

        first = runner.run(plan)

        self.assertTrue(first.completed)
        with self.assertRaises(RuntimeError):
            runner.run(plan)


if __name__ == "__main__":
    unittest.main()
