from dataclasses import replace
import unittest

from robot_agent.autonomy_contract import (
    EXPLORE_SPACE,
    FORWARD,
    INVESTIGATE_OBSERVATION,
    LEFT,
    RIGHT,
    ROBOT_BASE_FRAME,
)
from robot_agent.autonomy_perception import (
    ExplorationMemory,
    ExplorationPolicy,
    RangeObservationTracker,
    SimulatorCandidateGenerator,
)
from robot_agent.navigation_contract import (
    DriveCalibrationProfile,
    MotionAuthority,
    NavigationContractError,
    WaypointGoal,
)
from robot_agent.navigation_simulator import (
    CircleObstacle,
    DifferentialDriveSimulator,
    SimulationSettings,
    SimulationWorld,
)
from robot_agent.navigation_state import (
    ClearanceEvidence,
    PoseEstimate,
)


def _profile():
    return DriveCalibrationProfile(
        calibration_id="autonomy-perception-fixture-v1",
        status="simulation_only",
        surface="synthetic",
        left_motor_sign=1,
        right_motor_sign=1,
        encoder_mdeg_per_mm=1_800,
        encoder_mdeg_per_body_degree=2_000,
        max_wheel_speed_dps=250,
        max_pulse_ms=120,
    )


def _goal():
    return WaypointGoal(
        goal_id="idle-observation",
        goal_epoch=1,
        plan_revision=1,
        target_x_mm=650,
        target_y_mm=450,
        tolerance_mm=30,
    )


def _plant(
    obstacles=(),
    pose=PoseEstimate(650, 450, 0),
    settings=SimulationSettings(),
):
    return DifferentialDriveSimulator(
        world=SimulationWorld(
            width_mm=1_300,
            height_mm=900,
            obstacles=tuple(obstacles),
        ),
        profile=_profile(),
        initial_pose=pose,
        motion_authority=MotionAuthority(),
        settings=settings,
    )


def _generator(plant, memory=None):
    if memory is None:
        memory = ExplorationMemory(grid_mm=50)
    return SimulatorCandidateGenerator(
        plant=plant,
        memory=memory,
        policy=ExplorationPolicy(
            step_mm=180,
            minimum_travel_mm=70,
            clearance_reserve_mm=120,
            tolerance_mm=30,
            visit_grid_mm=50,
            max_completed_visits_without_change=1,
        ),
    )


def _capture(tracker, snapshot, observation_id):
    received_at = snapshot.captured_at_host_ms
    return tracker.capture(
        snapshot,
        observation_id,
        received_at_host_ms=received_at,
        valid_until_host_ms=received_at + 1_000,
    )


class RangeObservationTrackerTests(unittest.TestCase):
    def test_identical_sample_is_typed_as_unchanged(self):
        plant = _plant()
        tracker = RangeObservationTracker()
        snapshot = plant.observe(_goal())
        tracker.seed(snapshot)

        observation = _capture(
            tracker,
            snapshot,
            "unchanged-range-1",
        )

        self.assertEqual(observation.kind, "METRIC_UNCHANGED")
        self.assertEqual(
            observation.previous_value,
            observation.current_value,
        )
        self.assertEqual(
            observation.subject_robot_id,
            snapshot.robot_id,
        )
        self.assertEqual(
            observation.controller_instance_id,
            snapshot.controller_instance_id,
        )
        self.assertEqual(observation.frame_id, ROBOT_BASE_FRAME)
        self.assertEqual(
            observation.received_at_host_ms,
            snapshot.captured_at_host_ms,
        )
        self.assertGreater(
            observation.valid_until_host_ms,
            observation.received_at_host_ms,
        )
        self.assertFalse(tracker.is_exact_transition(observation))

    def test_same_pose_emits_exact_range_and_object_transition(self):
        plant = _plant()
        tracker = RangeObservationTracker()
        before = plant.observe(_goal())
        tracker.seed(before)

        plant.update_world(
            SimulationWorld(
                width_mm=1_300,
                height_mm=900,
                obstacles=(
                    CircleObstacle(
                        obstacle_id="new-box",
                        x_mm=1_130,
                        y_mm=450,
                        radius_mm=30,
                    ),
                ),
            )
        )
        after = plant.observe(_goal())
        observation = _capture(tracker, after, "range-change-1")

        self.assertIsNotNone(observation)
        self.assertEqual(observation.kind, "METRIC_TRANSITION")
        self.assertEqual(
            observation.previous_value,
            before.clearance.forward_mm,
        )
        self.assertEqual(
            observation.current_value,
            after.clearance.forward_mm,
        )
        self.assertIsNone(observation.previous_subject_id)
        self.assertEqual(observation.current_subject_id, "new-box")
        self.assertTrue(tracker.is_exact_transition(observation))

    def test_subject_identity_change_is_a_transition_at_equal_range(self):
        first = CircleObstacle("first-box", 1_130, 450, 30)
        plant = _plant((first,))
        tracker = RangeObservationTracker()
        before = plant.observe(_goal())
        tracker.seed(before)

        plant.update_world(
            SimulationWorld(
                width_mm=1_300,
                height_mm=900,
                obstacles=(
                    CircleObstacle("replacement-box", 1_130, 450, 30),
                ),
            )
        )
        observation = _capture(
            tracker,
            plant.observe(_goal()),
            "identity-change-1",
        )

        self.assertEqual(
            observation.previous_value,
            observation.current_value,
        )
        self.assertEqual(observation.previous_subject_id, "first-box")
        self.assertEqual(
            observation.current_subject_id,
            "replacement-box",
        )
        self.assertTrue(tracker.is_exact_transition(observation))

    def test_samples_at_different_poses_are_not_compared(self):
        plant = _plant()
        tracker = RangeObservationTracker()
        first = plant.observe(_goal())
        tracker.seed(first)
        at_another_pose = replace(
            first,
            pose=replace(first.pose, x_mm=first.pose.x_mm + 1),
            clearance=replace(
                first.clearance,
                forward_mm=first.clearance.forward_mm - 100,
            ),
        )

        observation = _capture(
            tracker,
            at_another_pose,
            "different-pose-1",
        )

        self.assertEqual(observation.kind, "METRIC_SAMPLE")
        self.assertIsNone(observation.previous_value)
        self.assertFalse(tracker.is_exact_transition(observation))

    def test_physical_ir_is_neither_metric_observation_nor_candidate(self):
        plant = _plant()
        snapshot = plant.observe(_goal())
        physical = replace(
            snapshot,
            clearance=ClearanceEvidence(
                source="physical_ir_reflection",
                observed_at_ms=snapshot.captured_at_host_ms,
                near_obstacle_latched=False,
                raw_ir_proximity=20,
            ),
        )
        tracker = RangeObservationTracker()

        tracker.seed(physical)
        self.assertIsNone(_capture(
            tracker,
            physical,
            "physical-ir-1",
        ))
        self.assertEqual(
            _generator(plant).generate(
                physical,
                "physical-candidates-1",
                None,
            ),
            (),
        )


class SimulatorCandidateGeneratorTests(unittest.TestCase):
    def test_policy_rejects_zero_progress_and_unsafe_sensor_margins(self):
        with self.assertRaises(NavigationContractError) as caught:
            ExplorationPolicy(
                minimum_travel_mm=70,
                tolerance_mm=70,
            )
        self.assertEqual(
            caught.exception.code,
            "invalid_exploration_tolerance",
        )

        plant = _plant(settings=SimulationSettings(
            near_threshold_mm=150,
        ))
        with self.assertRaises(NavigationContractError) as caught:
            SimulatorCandidateGenerator(
                plant,
                ExplorationMemory(),
                ExplorationPolicy(clearance_reserve_mm=149),
            )
        self.assertEqual(
            caught.exception.code,
            "unsafe_clearance_reserve",
        )

    def test_boundary_candidate_stops_before_avoidance_threshold(self):
        plant = _plant(pose=PoseEstimate(1_140, 407, 72_000))
        policy = ExplorationPolicy()
        generator = SimulatorCandidateGenerator(
            plant,
            ExplorationMemory(),
            policy,
        )
        snapshot = plant.observe(_goal())

        forward = next(
            candidate
            for candidate in generator.generate(
                snapshot,
                "boundary-reachability-1",
                None,
            )
            if candidate.view.relative_direction == FORWARD
        )

        self.assertGreaterEqual(
            policy.clearance_reserve_mm,
            plant.settings.near_threshold_mm,
        )
        self.assertGreaterEqual(
            snapshot.clearance.forward_mm
            - forward.view.estimated_travel_mm,
            plant.settings.near_threshold_mm,
        )

    def test_safe_menu_has_three_opaque_coordinate_free_views(self):
        plant = _plant()
        candidates = _generator(plant).generate(
            plant.observe(_goal()),
            "safe-menu-1",
            None,
        )

        self.assertEqual(
            tuple(item.view.relative_direction for item in candidates),
            (FORWARD, LEFT, RIGHT),
        )
        self.assertTrue(
            all(item.view.task_kind == EXPLORE_SPACE for item in candidates)
        )
        self.assertTrue(
            all(item.view.estimated_travel_mm == 180 for item in candidates)
        )
        expected_view_keys = {
            "candidate_id",
            "task_kind",
            "relative_direction",
            "estimated_travel_mm",
            "attempted_visits",
            "completed_visits",
            "linked_observation_ids",
        }
        for candidate in candidates:
            self.assertEqual(
                set(candidate.view.to_dict()),
                expected_view_keys,
            )
            serialized = repr(candidate.view.to_dict()).lower()
            self.assertNotIn("target_x", serialized)
            self.assertNotIn("target_y", serialized)
            self.assertNotIn("heading", serialized)

    def test_blocked_rays_are_omitted_from_menu(self):
        obstacles = (
            CircleObstacle("front", 770, 450, 30),
            CircleObstacle("left", 735, 535, 30),
            CircleObstacle("right", 735, 365, 30),
        )
        plant = _plant(obstacles)

        candidates = _generator(plant).generate(
            plant.observe(_goal()),
            "blocked-menu-1",
            None,
        )

        self.assertEqual(candidates, ())

    def test_completed_target_is_suppressed_until_exact_change(self):
        plant = _plant()
        memory = ExplorationMemory(grid_mm=50)
        generator = _generator(plant, memory)
        before = plant.observe(_goal())
        tracker = RangeObservationTracker()
        tracker.seed(before)
        original = generator.generate(before, "original-menu-1", None)
        forward = generator.resolve(original, "original-menu-1-c1")
        memory.record_attempt(forward.memory_key)
        memory.record_completed(forward.memory_key)

        without_change = generator.generate(
            before,
            "unchanged-menu-1",
            None,
        )
        self.assertNotIn(
            FORWARD,
            tuple(item.view.relative_direction for item in without_change),
        )

        plant.update_world(
            SimulationWorld(
                width_mm=1_300,
                height_mm=900,
                obstacles=(
                    CircleObstacle(
                        obstacle_id="new-box",
                        x_mm=1_130,
                        y_mm=450,
                        radius_mm=30,
                    ),
                ),
            )
        )
        changed_snapshot = plant.observe(_goal())
        changed = _capture(
            tracker,
            changed_snapshot,
            "range-change-2",
        )
        with_change = generator.generate(
            changed_snapshot,
            "changed-menu-1",
            changed,
        )
        reincluded = generator.resolve(
            with_change,
            "changed-menu-1-c1",
        )

        self.assertEqual(
            reincluded.view.task_kind,
            INVESTIGATE_OBSERVATION,
        )
        self.assertEqual(
            reincluded.view.linked_observation_ids,
            ("range-change-2",),
        )
        self.assertEqual(reincluded.view.completed_visits, 1)
        self.assertEqual(reincluded.memory_key, forward.memory_key)

    def test_attempt_retry_cap_suppresses_cell_until_exact_change(self):
        plant = _plant()
        memory = ExplorationMemory(grid_mm=50)
        generator = _generator(plant, memory)
        snapshot = plant.observe(_goal())
        tracker = RangeObservationTracker()
        tracker.seed(snapshot)
        initial = generator.generate(
            snapshot,
            "attempt-menu-1",
            None,
        )
        forward = generator.resolve(initial, "attempt-menu-1-c1")

        memory.record_attempt(forward.memory_key)
        once = generator.generate(
            snapshot,
            "attempt-menu-2",
            None,
        )
        offered_once = generator.resolve(once, "attempt-menu-2-c1")
        self.assertEqual(offered_once.view.attempted_visits, 1)

        memory.record_attempt(forward.memory_key)
        capped = generator.generate(
            snapshot,
            "attempt-menu-3",
            None,
        )
        self.assertNotIn(
            FORWARD,
            tuple(item.view.relative_direction for item in capped),
        )

        plant.update_world(SimulationWorld(
            width_mm=1_300,
            height_mm=900,
            obstacles=(
                CircleObstacle("new-box", 1_130, 450, 30),
            ),
        ))
        changed_snapshot = plant.observe(_goal())
        changed = _capture(
            tracker,
            changed_snapshot,
            "attempt-range-change-1",
        )
        reconsidered = generator.generate(
            changed_snapshot,
            "attempt-menu-4",
            changed,
        )
        offered_after_change = generator.resolve(
            reconsidered,
            "attempt-menu-4-c1",
        )
        self.assertEqual(
            offered_after_change.view.attempted_visits,
            2,
        )
        self.assertEqual(
            offered_after_change.view.task_kind,
            INVESTIGATE_OBSERVATION,
        )

    def test_candidate_observation_must_match_snapshot_identity_and_frame(self):
        plant = _plant()
        snapshot = plant.observe(_goal())
        observation = _capture(
            RangeObservationTracker(),
            snapshot,
            "candidate-binding-1",
        )
        generator = _generator(plant)
        mutations = (
            {"subject_robot_id": "another-robot"},
            {"controller_instance_id": "another-controller"},
            {"frame_id": "CAMERA_FRAME"},
            {"state_version": snapshot.state_version + 1},
            {
                "world_model_version": (
                    snapshot.world_model_version + 1
                )
            },
        )

        for changes in mutations:
            with self.subTest(changes=changes):
                with self.assertRaises(
                    NavigationContractError
                ) as caught:
                    generator.generate(
                        snapshot,
                        "candidate-binding-menu",
                        replace(observation, **changes),
                    )
                self.assertEqual(
                    caught.exception.code,
                    "stale_candidate_observation",
                )

    def test_target_geometry_is_checked_against_current_world(self):
        obstacle = CircleObstacle("hidden-at-target", 830, 450, 30)
        plant = _plant((obstacle,))
        snapshot = plant.observe(_goal())
        inconsistent_clearance = replace(
            snapshot,
            clearance=replace(
                snapshot.clearance,
                near_obstacle_latched=False,
                forward_mm=1_000,
                left_mm=1_000,
                right_mm=1_000,
                forward_object_id=None,
            ),
        )

        candidates = _generator(plant).generate(
            inconsistent_clearance,
            "world-check-1",
            None,
        )

        self.assertNotIn(
            FORWARD,
            tuple(item.view.relative_direction for item in candidates),
        )

    def test_world_boundary_rejects_inconsistent_forward_target(self):
        plant = _plant(pose=PoseEstimate(1_100, 450, 0))
        snapshot = plant.observe(_goal())
        inconsistent_clearance = replace(
            snapshot,
            clearance=replace(
                snapshot.clearance,
                near_obstacle_latched=False,
                forward_mm=1_000,
            ),
        )

        candidates = _generator(plant).generate(
            inconsistent_clearance,
            "boundary-check-1",
            None,
        )

        self.assertNotIn(
            FORWARD,
            tuple(item.view.relative_direction for item in candidates),
        )

    def test_unsafe_snapshot_is_rejected_before_geometry(self):
        plant = _plant()
        generator = _generator(plant)
        snapshot = plant.observe(_goal())

        for unsafe in (
            replace(snapshot, motors_running=True),
            replace(snapshot, touch_pressed=True),
            replace(snapshot, active_faults=("fault",)),
            replace(snapshot, robot_id="another-robot"),
            replace(
                snapshot,
                controller_instance_id="another-controller",
            ),
        ):
            with self.subTest(snapshot=unsafe):
                with self.assertRaises(NavigationContractError) as caught:
                    generator.generate(
                        unsafe,
                        "unsafe-menu-1",
                        None,
                    )
                self.assertEqual(
                    caught.exception.code,
                    "unsafe_candidate_snapshot",
                )

    def test_unknown_candidate_cannot_be_resolved(self):
        plant = _plant()
        generator = _generator(plant)
        candidates = generator.generate(
            plant.observe(_goal()),
            "resolve-menu-1",
            None,
        )

        with self.assertRaises(NavigationContractError) as caught:
            generator.resolve(candidates, "not-in-menu")

        self.assertEqual(
            caught.exception.code,
            "unresolved_exploration_candidate",
        )

    def test_generation_and_resolution_do_not_mutate_memory(self):
        plant = _plant()
        memory = ExplorationMemory(grid_mm=50)
        generator = _generator(plant, memory)
        candidates = generator.generate(
            plant.observe(_goal()),
            "memory-menu-1",
            None,
        )
        selected = generator.resolve(
            candidates,
            "memory-menu-1-c1",
        )

        self.assertEqual(memory.counts(selected.memory_key), (0, 0))
        generator.generate(
            plant.observe(_goal()),
            "memory-menu-2",
            None,
        )
        self.assertEqual(memory.counts(selected.memory_key), (0, 0))

        memory.record_attempt(selected.memory_key)
        self.assertEqual(memory.counts(selected.memory_key), (1, 0))
        memory.record_completed(selected.memory_key)
        self.assertEqual(memory.counts(selected.memory_key), (1, 1))
        with self.assertRaises(NavigationContractError) as caught:
            memory.record_completed(selected.memory_key)
        self.assertEqual(
            caught.exception.code,
            "completion_without_attempt",
        )


if __name__ == "__main__":
    unittest.main()
