from dataclasses import replace
import itertools
import json
import queue
import threading
import unittest
from unittest import mock

from robot_agent.concurrent_runtime import (
    ConcurrentBehaviorRuntime,
    ConcurrentRuntimePolicy,
)
from robot_agent.interaction_contract import (
    ExpressionIntent,
    ExpressionProposal,
    expression_proposal_id_for_snapshot,
)
from robot_agent.navigation_contract import (
    DriveCalibrationProfile,
    MotionAuthority,
    WaypointGoal,
)
from robot_agent.navigation_episode import (
    NAVIGATION_ABORTED,
    GoalSeekingBehavior,
    NavigationLimits,
    ObstacleAvoidanceBehavior,
)
from robot_agent.navigation_simulator import (
    CircleObstacle,
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


def simulation_profile():
    return DriveCalibrationProfile(
        calibration_id="concurrent-runtime-fixture",
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
        goal_id="concurrent-waypoint",
        goal_epoch=1,
        plan_revision=1,
        target_x_mm=x_mm,
        target_y_mm=y_mm,
        tolerance_mm=30,
    )


class SignalingInbox(ProposalInbox):
    def __init__(self, policies, clock_ms, published):
        super().__init__(policies, clock_ms, capacity=16)
        self.published = published
        self.source_published = {
            policy.source_id: threading.Event()
            for policy in policies
        }

    def publish_host(self, proposal, source_id):
        value = super().publish_host(proposal, source_id)
        self.source_published[source_id].set()
        self.published.set()
        return value


class OverlapCheckingSimulator(DifferentialDriveSimulator):
    def __init__(self, *args, arm_active, **kwargs):
        self.arm_active = arm_active
        self.overlap_detected = False
        self.drive_count = 0
        super().__init__(*args, **kwargs)

    def apply(self, pulse, goal):
        if pulse.kind == "DRIVE":
            self.drive_count += 1
            if self.arm_active.is_set():
                self.overlap_detected = True
        return super().apply(pulse, goal)


def make_stack(
    obstacle=True,
    published=None,
    plant_class=DifferentialDriveSimulator,
    plant_kwargs=None,
):
    if published is None:
        published = threading.Event()
    if plant_kwargs is None:
        plant_kwargs = {}
    authority = MotionAuthority()
    obstacles = (
        (
            CircleObstacle(
                obstacle_id="box",
                x_mm=400,
                y_mm=300,
                radius_mm=30,
            ),
        )
        if obstacle
        else ()
    )
    plant = plant_class(
        SimulationWorld(
            width_mm=1_300,
            height_mm=700,
            obstacles=obstacles,
        ),
        simulation_profile(),
        PoseEstimate(150, 300, 0),
        authority,
        **plant_kwargs,
    )
    ids = itertools.count(1)
    supervisor = MotionSupervisor(
        plant.profile,
        plant.clock_ms,
        plant.robot_id,
        plant.controller_instance_id,
        authority,
        policy=MotionPolicy(
            max_snapshot_age_ms=200,
            max_safety_age_ms=100,
            max_proposal_ttl_ms=500,
            max_pulse_ms=120,
            max_linear_speed_mm_s=120,
            max_angular_speed_mdeg_s=90_000,
            forward_reserve_mm=70,
        ),
        id_factory=lambda: "concurrent-decision-{}".format(next(ids)),
    )
    policies = (
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
    )
    inbox = SignalingInbox(policies, plant.clock_ms, published)
    return plant, supervisor, inbox, published


def expression_for(
    snapshot,
    proposal_id=None,
    gesture_kind="PROPELLER_WAVE",
):
    if proposal_id is None:
        proposal_id = expression_proposal_id_for_snapshot(snapshot)
    repetitions = 1 if gesture_kind == "PROPELLER_WAVE" else 0
    return ExpressionProposal(
        proposal_id=proposal_id,
        robot_id=snapshot.robot_id,
        controller_instance_id=snapshot.controller_instance_id,
        goal_id=snapshot.goal_id,
        goal_epoch=snapshot.goal_epoch,
        plan_revision=snapshot.plan_revision,
        based_on_interaction_state_version=(
            snapshot.interaction_state_version
        ),
        based_on_world_model_version=snapshot.world_model_version,
        obstruction_epoch=snapshot.obstruction_epoch,
        based_on_evidence_id=snapshot.evidence.evidence_id,
        decision="EXPRESS",
        confidence_milli=900,
        intent=ExpressionIntent(
            utterance="Move, box. You are blocking my route.",
            utterance_locale=snapshot.response_locale,
            gesture_kind=gesture_kind,
            affect_label="grumpy",
            intensity=900,
            repetitions=repetitions,
        ),
    )


def short_limits(max_ticks=8):
    return NavigationLimits(
        max_ticks=max_ticks,
        max_elapsed_ms=30_000,
        max_proposals=100,
        max_replans=100,
        max_actions=100,
        max_total_motion_ms=20_000,
        max_no_progress_ticks=100,
    )


def waiting_tick(published):
    def tick(cancel_event, _interval):
        if cancel_event.is_set():
            return
        if not published.wait(1):
            raise AssertionError("navigation producer did not publish")
        published.clear()

    return tick


def run_async(runtime):
    completed = threading.Event()
    values = {}

    def target():
        try:
            values["result"] = runtime.run()
        except BaseException as error:
            values["error"] = error
        finally:
            completed.set()

    thread = threading.Thread(target=target)
    thread.start()
    return thread, completed, values


class RuntimePolicyValidationTests(unittest.TestCase):
    def test_rejects_non_finite_second_values(self):
        for field in (
            "tick_interval_s",
            "shutdown_join_timeout_s",
        ):
            for value in (
                float("nan"),
                float("inf"),
                float("-inf"),
            ):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ValueError):
                        ConcurrentRuntimePolicy(**{field: value})


class PlannerAdmissionPolicyTests(unittest.TestCase):
    def make_runtime(
        self,
        max_planner_requests=8,
        expression_cooldown_ms=5_000,
    ):
        plant, supervisor, inbox, _published = make_stack()
        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=expression_for,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            policy=replace(
                ConcurrentRuntimePolicy(),
                planner_queue_capacity=8,
                max_planner_requests=max_planner_requests,
                expression_cooldown_ms=expression_cooldown_ms,
            ),
            host_clock_ms=lambda: 1_000,
        )
        return runtime, plant.observe(waypoint())

    @staticmethod
    def clear_snapshot(blocked, state_version):
        return replace(
            blocked,
            state_version=state_version,
            clearance=replace(
                blocked.clearance,
                near_obstacle_latched=False,
                forward_mm=1_000,
                forward_object_id=None,
            ),
        )

    @staticmethod
    def blocked_snapshot(original, state_version, object_id):
        return replace(
            original,
            state_version=state_version,
            clearance=replace(
                original.clearance,
                near_obstacle_latched=True,
                forward_mm=100,
                forward_object_id=object_id,
            ),
        )

    @staticmethod
    def metrics(runtime):
        return runtime._metrics.snapshot(runtime._events.dropped)

    def test_same_stable_key_inside_cooldown_queues_one_request(self):
        for object_id in ("demo-box", None):
            with self.subTest(object_id=object_id):
                runtime, original = self.make_runtime()
                first = self.blocked_snapshot(
                    original,
                    state_version=1,
                    object_id=object_id,
                )
                runtime._observation_sink(first)
                runtime._observation_sink(
                    self.clear_snapshot(first, state_version=2)
                )
                runtime._observation_sink(
                    self.blocked_snapshot(
                        first,
                        state_version=3,
                        object_id=object_id,
                    )
                )

                metrics = self.metrics(runtime)
                self.assertEqual(metrics.planner_requests, 1)
                self.assertEqual(metrics.planner_cooldown_drops, 1)
                self.assertEqual(metrics.planner_budget_drops, 0)
                self.assertEqual(runtime._planner_queue.qsize(), 1)
                self.assertTrue(any(
                    event.kind == "planner_cooldown_drop"
                    for event in runtime._events.snapshot()
                ))

    def test_different_object_is_immediately_eligible(self):
        runtime, original = self.make_runtime()
        first = self.blocked_snapshot(
            original,
            state_version=1,
            object_id="first-box",
        )
        runtime._observation_sink(first)
        runtime._observation_sink(
            self.clear_snapshot(first, state_version=2)
        )
        runtime._observation_sink(
            self.blocked_snapshot(
                first,
                state_version=3,
                object_id="second-box",
            )
        )

        metrics = self.metrics(runtime)
        self.assertEqual(metrics.planner_requests, 2)
        self.assertEqual(metrics.planner_cooldown_drops, 0)
        self.assertEqual(metrics.planner_budget_drops, 0)
        self.assertEqual(runtime._planner_queue.qsize(), 2)

    def test_total_planner_request_budget_is_enforced_before_queueing(self):
        runtime, original = self.make_runtime(
            max_planner_requests=2,
            expression_cooldown_ms=0,
        )
        first = self.blocked_snapshot(
            original,
            state_version=1,
            object_id="box-1",
        )
        runtime._observation_sink(first)
        runtime._observation_sink(
            self.clear_snapshot(first, state_version=2)
        )
        second = self.blocked_snapshot(
            first,
            state_version=3,
            object_id="box-2",
        )
        runtime._observation_sink(second)
        runtime._observation_sink(
            self.clear_snapshot(second, state_version=4)
        )
        runtime._observation_sink(
            self.blocked_snapshot(
                second,
                state_version=5,
                object_id="box-3",
            )
        )

        metrics = self.metrics(runtime)
        self.assertEqual(metrics.planner_requests, 2)
        self.assertEqual(metrics.planner_budget_drops, 1)
        self.assertEqual(metrics.planner_cooldown_drops, 0)
        self.assertEqual(runtime._planner_queue.qsize(), 2)
        self.assertTrue(any(
            event.kind == "planner_budget_drop"
            for event in runtime._events.snapshot()
        ))

    def test_default_expression_ttl_is_valid_through_5000_not_5001(self):
        host_now = [0]
        plant, supervisor, inbox, _published = make_stack()
        policy = ConcurrentRuntimePolicy()
        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=expression_for,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            policy=policy,
            host_clock_ms=lambda: host_now[0],
        )
        runtime._observation_sink(plant.observe(waypoint()))
        request = runtime._planner_queue.get_nowait()
        proposal = expression_for(
            request.snapshot,
            gesture_kind=None,
        )

        self.assertEqual(policy.expression_ttl_ms, 5_000)
        self.assertEqual(request.submitted_at_ms, 0)
        self.assertEqual(request.valid_until_ms, 5_000)
        host_now[0] = 5_000
        self.assertTrue(runtime._event_is_current(
            proposal,
            request.snapshot,
            request.valid_until_ms,
        ))
        host_now[0] = 5_001
        self.assertFalse(runtime._event_is_current(
            proposal,
            request.snapshot,
            request.valid_until_ms,
        ))

    def test_new_obstruction_epoch_invalidates_pending_expression(self):
        runtime, original = self.make_runtime(
            expression_cooldown_ms=0,
        )
        first = self.blocked_snapshot(
            original,
            state_version=1,
            object_id="first-box",
        )
        runtime._observation_sink(first)
        request = runtime._planner_queue.get_nowait()
        proposal = expression_for(
            request.snapshot,
            gesture_kind=None,
        )

        runtime._observation_sink(
            self.clear_snapshot(first, state_version=2)
        )
        self.assertTrue(runtime._event_is_current(
            proposal,
            request.snapshot,
            request.valid_until_ms,
        ))

        runtime._observation_sink(self.blocked_snapshot(
            first,
            state_version=3,
            object_id="second-box",
        ))
        current = runtime._interaction.current()
        self.assertGreater(
            current.obstruction_epoch,
            request.snapshot.obstruction_epoch,
        )
        self.assertFalse(runtime._event_is_current(
            proposal,
            request.snapshot,
            request.valid_until_ms,
        ))

    def test_duplicate_expression_proposal_id_is_rejected(self):
        runtime, original = self.make_runtime()
        first = self.blocked_snapshot(
            original,
            state_version=1,
            object_id="box",
        )
        runtime._observation_sink(first)
        request = runtime._planner_queue.get_nowait()
        runtime._planner_queue.put_nowait(request)
        runtime._planner_queue.put_nowait(request)
        runtime._close_queues()

        runtime._planner_worker()

        metrics = self.metrics(runtime)
        self.assertEqual(metrics.expressions_accepted, 1)
        self.assertEqual(metrics.duplicate_expression_drops, 1)
        self.assertTrue(any(
            event.kind == "duplicate_expression_drop"
            and event.detail == request.proposal_id
            for event in runtime._events.snapshot()
        ))

    def test_model_chosen_proposal_id_is_rejected(self):
        runtime, original = self.make_runtime()
        runtime.expression_planner = lambda item: expression_for(
            item,
            proposal_id="model-chosen-id",
        )
        first = self.blocked_snapshot(
            original,
            state_version=1,
            object_id="box",
        )
        runtime._observation_sink(first)
        runtime._close_queues()

        runtime._planner_worker()

        metrics = self.metrics(runtime)
        self.assertEqual(metrics.expressions_accepted, 0)
        self.assertEqual(metrics.planner_failures, 1)
        self.assertEqual(metrics.duplicate_expression_drops, 0)
        self.assertTrue(any(
            event.kind == "proposal_id_mismatch_drop"
            and event.detail == "model-chosen-id"
            for event in runtime._events.snapshot()
        ))


class ConcurrentNavigationTests(unittest.TestCase):
    def test_blocked_expression_planner_does_not_block_navigation(self):
        plant, supervisor, inbox, published = make_stack()
        planner_entered = threading.Event()
        release_planner = threading.Event()
        planner_cancelled = threading.Event()

        def planner(snapshot):
            planner_entered.set()
            release_planner.wait()
            return expression_for(snapshot)

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=planner,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(7),
            policy=replace(
                ConcurrentRuntimePolicy(),
                shutdown_join_timeout_s=0,
            ),
            tick_hook=waiting_tick(published),
        )
        original_events = runtime._events

        class SignalingAuditLog:
            @property
            def dropped(self):
                return original_events.dropped

            def append(self, worker, kind, detail=""):
                original_events.append(worker, kind, detail)
                if kind == "planner_result_cancelled":
                    planner_cancelled.set()

            def snapshot(self):
                return original_events.snapshot()

        runtime._events = SignalingAuditLog()
        thread, completed, values = run_async(runtime)

        self.assertTrue(planner_entered.wait(1))
        self.assertTrue(completed.wait(2))
        self.assertNotIn("error", values)
        self.assertGreater(values["result"].navigation.ticks, 1)
        self.assertGreater(values["result"].navigation.actions, 1)
        self.assertIn(
            "expression-planner",
            values["result"].workers_alive,
        )
        self.assertEqual(
            values["result"].metrics.expressions_accepted,
            0,
        )
        queue_sizes = (
            runtime._speech_queue.qsize(),
            runtime._arm_queue.qsize(),
        )

        release_planner.set()
        self.assertTrue(planner_cancelled.wait(1))
        internal_metrics = runtime._metrics.snapshot(
            runtime._events.dropped
        )
        self.assertEqual(internal_metrics.expressions_accepted, 0)
        self.assertEqual(
            (
                runtime._speech_queue.qsize(),
                runtime._arm_queue.qsize(),
            ),
            queue_sizes,
        )
        thread.join(1)

    def test_blocked_speech_playback_overlaps_later_drive_ticks(self):
        plant, supervisor, inbox, published = make_stack()
        speech_started = threading.Event()
        observed_two_later_ticks = threading.Event()
        arm_called = threading.Event()
        tick_count = [0]

        def speaker(_text, _locale, cancel_event):
            speech_started.set()
            cancel_event.wait()

        def speech_only(snapshot):
            return expression_for(snapshot, gesture_kind=None)

        def tick(cancel_event, _interval):
            tick_count[0] += 1
            if tick_count[0] == 1:
                self.assertTrue(speech_started.wait(1))
            elif speech_started.is_set() and tick_count[0] >= 3:
                observed_two_later_ticks.set()
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=speech_only,
            speaker=speaker,
            arm_segment_executor=lambda *_args: arm_called.set(),
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(7),
            tick_hook=tick,
        )

        result = runtime.run()

        self.assertTrue(observed_two_later_ticks.is_set())
        self.assertGreaterEqual(result.metrics.speech_started, 1)
        self.assertGreater(result.navigation.actions, 1)
        self.assertEqual(result.metrics.speech_failures, 0)
        self.assertFalse(arm_called.is_set())
        self.assertEqual(result.metrics.navigation_pause_requests, 0)
        self.assertEqual(result.metrics.navigation_pause_acks, 0)

    def test_latest_snapshot_mailbox_is_bounded_and_audits_drops(self):
        plant, supervisor, inbox, _published = make_stack(
            obstacle=False
        )
        behavior_entered = threading.Event()
        release_behavior = threading.Event()
        original = GoalSeekingBehavior.propose

        def blocked_propose(behavior, goal, snapshot):
            behavior_entered.set()
            release_behavior.wait()
            return original(behavior, goal, snapshot)

        tick_number = [0]

        def tick(_cancel_event, _interval):
            tick_number[0] += 1
            if tick_number[0] == 1:
                self.assertTrue(behavior_entered.wait(1))

        with mock.patch.object(
            GoalSeekingBehavior,
            "propose",
            blocked_propose,
        ):
            runtime = ConcurrentBehaviorRuntime(
                plant,
                supervisor,
                inbox,
                waypoint(),
                response_locale="en",
                behaviors=(GoalSeekingBehavior(),),
                navigation_limits=short_limits(6),
                policy=replace(
                    ConcurrentRuntimePolicy(),
                    behavior_queue_capacity=1,
                    shutdown_join_timeout_s=0,
                ),
                tick_hook=tick,
            )
            _thread, completed, values = run_async(runtime)
            self.assertTrue(completed.wait(2))
            self.assertNotIn("error", values)

        release_behavior.set()
        self.assertGreater(
            values["result"].metrics.navigation_snapshot_drops,
            0,
        )
        self.assertTrue(any(
            event.kind == "queue_drop"
            and "navigation_snapshot" in event.detail
            for event in values["result"].events
        ))


class ExpressionIsolationTests(unittest.TestCase):
    def test_speech_event_survives_clear_obstruction_in_same_world(self):
        plant, supervisor, inbox, published = make_stack()
        planner_entered = threading.Event()
        release_planner = threading.Event()
        speech_called = threading.Event()
        arm_called = threading.Event()
        runtime_holder = []
        tick_number = [0]
        planner_calls = [0]

        def planner(snapshot):
            planner_calls[0] += 1
            if planner_calls[0] > 1:
                return ExpressionProposal(
                    proposal_id=expression_proposal_id_for_snapshot(
                        snapshot
                    ),
                    robot_id=snapshot.robot_id,
                    controller_instance_id=(
                        snapshot.controller_instance_id
                    ),
                    goal_id=snapshot.goal_id,
                    goal_epoch=snapshot.goal_epoch,
                    plan_revision=snapshot.plan_revision,
                    based_on_interaction_state_version=(
                        snapshot.interaction_state_version
                    ),
                    based_on_world_model_version=(
                        snapshot.world_model_version
                    ),
                    obstruction_epoch=snapshot.obstruction_epoch,
                    based_on_evidence_id=(
                        snapshot.evidence.evidence_id
                    ),
                    decision="HOLD",
                    confidence_milli=1_000,
                    reason_code="already_reacted",
                )
            planner_entered.set()
            release_planner.wait()
            return expression_for(snapshot, gesture_kind=None)

        def tick(cancel_event, _interval):
            tick_number[0] += 1
            self.assertTrue(
                inbox.source_published[
                    "obstacle-avoidance"
                ].wait(1)
            )
            inbox.source_published["obstacle-avoidance"].clear()
            if tick_number[0] == 1:
                self.assertTrue(planner_entered.wait(1))
            elif tick_number[0] == 3:
                current = runtime_holder[0]._interaction.current()
                self.assertIsNone(current.evidence)
                release_planner.set()
                self.assertTrue(speech_called.wait(1))
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=planner,
            speaker=lambda *_args: speech_called.set(),
            arm_segment_executor=lambda *_args: arm_called.set(),
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(6),
            tick_hook=tick,
        )
        runtime_holder.append(runtime)

        result = runtime.run()

        self.assertTrue(speech_called.is_set())
        self.assertEqual(result.metrics.expressions_accepted, 1)
        self.assertEqual(result.metrics.stale_speech_drops, 0)
        self.assertEqual(result.metrics.speech_started, 1)
        self.assertFalse(arm_called.is_set())
        self.assertEqual(result.metrics.navigation_pause_requests, 0)

    def test_stale_expression_is_dropped_after_obstruction_changes(self):
        plant, supervisor, inbox, published = make_stack()
        planner_entered = threading.Event()
        release_planner = threading.Event()
        proposal_validated = threading.Event()
        changed_world_observed = threading.Event()
        tick_number = [0]

        class SignalingExpression(ExpressionProposal):
            def assert_matches_snapshot(self, snapshot):
                try:
                    return super().assert_matches_snapshot(snapshot)
                finally:
                    proposal_validated.set()

        first_snapshot = []

        def planner(snapshot):
            if not first_snapshot:
                first_snapshot.append(snapshot)
            planner_entered.set()
            release_planner.wait()
            original = expression_for(first_snapshot[0])
            return SignalingExpression(**original.__dict__)

        def tick(cancel_event, _interval):
            tick_number[0] += 1
            if tick_number[0] == 1:
                self.assertTrue(planner_entered.wait(1))
                self.assertTrue(
                    inbox.source_published[
                        "obstacle-avoidance"
                    ].wait(1)
                )
                self.assertTrue(changed_world_observed.wait(1))
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=planner,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(8),
            tick_hook=tick,
        )

        thread, completed, values = run_async(runtime)
        self.assertTrue(planner_entered.wait(1))
        original_world_version = plant.observe(
            waypoint()
        ).world_model_version
        plant.update_world(
            SimulationWorld(
                width_mm=1_300,
                height_mm=700,
                obstacles=(),
            )
        )
        updated = plant.observe(waypoint())
        runtime._observation_sink(updated)
        current = runtime._interaction.current()
        self.assertGreater(
            current.world_model_version,
            original_world_version,
        )
        self.assertIsNone(current.evidence)
        release_planner.set()
        self.assertTrue(proposal_validated.wait(1))
        changed_world_observed.set()
        self.assertTrue(completed.wait(2))
        thread.join(1)
        self.assertNotIn("error", values)
        result = values["result"]

        self.assertEqual(
            result.metrics.expressions_accepted,
            0,
            result.to_dict(),
        )
        self.assertGreaterEqual(result.metrics.stale_expression_drops, 1)

    def test_strict_bytes_are_accepted_but_invalid_bytes_are_isolated(self):
        def raw_planner(snapshot):
            value = expression_for(snapshot).to_dict()
            return json.dumps(value).encode("utf-8")

        for planner, accepted, failures in (
            (raw_planner, 1, 0),
            (lambda _snapshot: b'{"not":"the contract"}', 0, 1),
            (lambda _snapshot: (_ for _ in ()).throw(RuntimeError()), 0, 1),
        ):
            with self.subTest(planner=planner):
                plant, supervisor, inbox, published = make_stack()
                runtime = ConcurrentBehaviorRuntime(
                    plant,
                    supervisor,
                    inbox,
                    waypoint(),
                    response_locale="en",
                    expression_planner=planner,
                    behaviors=(
                        GoalSeekingBehavior(),
                        ObstacleAvoidanceBehavior(),
                    ),
                    navigation_limits=short_limits(6),
                    tick_hook=waiting_tick(published),
                )

                result = runtime.run()

                if accepted:
                    self.assertGreaterEqual(
                        result.metrics.expressions_accepted,
                        accepted,
                    )
                else:
                    self.assertEqual(
                        result.metrics.expressions_accepted,
                        0,
                    )
                self.assertGreaterEqual(
                    result.metrics.planner_failures,
                    failures,
                )
                self.assertGreater(result.navigation.actions, 0)

    def test_speaker_failure_isolated_from_navigation(self):
        plant, supervisor, inbox, published = make_stack()
        speaker_called = threading.Event()

        def speaker(_text, _locale, _cancel_event):
            speaker_called.set()
            raise RuntimeError("synthetic audio failure")

        tick_number = [0]

        def tick(cancel_event, _interval):
            tick_number[0] += 1
            if tick_number[0] == 1:
                self.assertTrue(speaker_called.wait(1))
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=expression_for,
            speaker=speaker,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(7),
            tick_hook=tick,
        )

        result = runtime.run()

        self.assertGreaterEqual(result.metrics.speech_failures, 1)
        self.assertGreater(result.navigation.actions, 1)

    def test_cancelled_speech_callback_is_not_counted_completed(self):
        plant, supervisor, inbox, published = make_stack()
        speech_started = threading.Event()

        def planner(snapshot):
            return expression_for(snapshot, gesture_kind=None)

        def speaker(_text, _locale, cancel_event):
            speech_started.set()
            cancel_event.wait()

        def tick(cancel_event, _interval):
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=planner,
            speaker=speaker,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(20),
            tick_hook=tick,
        )
        thread, completed, values = run_async(runtime)

        self.assertTrue(speech_started.wait(1))
        runtime.cancel()
        self.assertTrue(completed.wait(2))
        thread.join(1)
        self.assertNotIn("error", values)
        result = values["result"]

        self.assertEqual(result.metrics.speech_started, 1)
        self.assertEqual(result.metrics.speech_completed, 0)
        self.assertEqual(result.metrics.speech_cancellations, 1)
        self.assertTrue(any(
            event.kind == "speech_cancelled"
            for event in result.events
        ))

    def test_second_queued_speech_is_revalidated_and_dropped_after_ttl(self):
        plant, supervisor, inbox, published = make_stack()
        host_now = [1_000]
        first_speech_started = threading.Event()
        release_first_speech = threading.Event()
        second_speech_queued = threading.Event()
        stale_speech_audited = threading.Event()
        allow_motion = threading.Event()
        speaker_calls = []
        planner_calls = []

        class SignalingSpeechQueue(queue.Queue):
            def __init__(self):
                super().__init__(maxsize=2)
                self.put_count = 0
                self.put_lock = threading.Lock()

            def put_nowait(self, item):
                result = super().put_nowait(item)
                with self.put_lock:
                    self.put_count += 1
                    if self.put_count == 2:
                        second_speech_queued.set()
                return result

        def planner(snapshot):
            planner_calls.append(snapshot)
            return expression_for(
                snapshot,
                gesture_kind=None,
            )

        def speaker(text, locale, cancel_event):
            speaker_calls.append((text, locale))
            if len(speaker_calls) == 1:
                first_speech_started.set()
                while (
                    not release_first_speech.is_set()
                    and not cancel_event.is_set()
                ):
                    release_first_speech.wait(1)

        def tick(cancel_event, _interval):
            if not allow_motion.is_set():
                self.assertTrue(allow_motion.wait(1))
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=planner,
            speaker=speaker,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(6),
            policy=replace(
                ConcurrentRuntimePolicy(),
                expression_ttl_ms=100,
            ),
            host_clock_ms=lambda: host_now[0],
            tick_hook=tick,
        )
        runtime._speech_queue = SignalingSpeechQueue()
        original_events = runtime._events

        class SignalingAuditLog:
            @property
            def dropped(self):
                return original_events.dropped

            def append(self, worker, kind, detail=""):
                original_events.append(worker, kind, detail)
                if kind == "stale_speech_drop":
                    stale_speech_audited.set()

            def snapshot(self):
                return original_events.snapshot()

        runtime._events = SignalingAuditLog()
        thread, completed, values = run_async(runtime)

        self.assertTrue(first_speech_started.wait(1))
        plant.update_world(
            SimulationWorld(
                width_mm=1_300,
                height_mm=700,
                obstacles=(
                    CircleObstacle(
                        obstacle_id="second-box",
                        x_mm=400,
                        y_mm=300,
                        radius_mm=30,
                    ),
                ),
            )
        )
        runtime._observation_sink(plant.observe(waypoint()))
        self.assertTrue(second_speech_queued.wait(1))
        host_now[0] = 1_101
        release_first_speech.set()
        self.assertTrue(stale_speech_audited.wait(1))
        allow_motion.set()
        self.assertTrue(completed.wait(2))
        thread.join(1)
        self.assertNotIn("error", values)
        result = values["result"]

        self.assertEqual(len(planner_calls), 2)
        self.assertEqual(len(speaker_calls), 1)
        self.assertEqual(result.metrics.expressions_accepted, 2)
        self.assertEqual(result.metrics.stale_speech_drops, 1)
        self.assertTrue(any(
            event.kind == "stale_speech_drop"
            and event.detail
            == expression_proposal_id_for_snapshot(planner_calls[1])
            for event in result.events
        ))

    def test_unused_speech_and_arm_workers_shutdown_without_planner(self):
        plant, supervisor, inbox, published = make_stack(
            obstacle=False
        )
        speaker_calls = []
        arm_calls = []
        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(x_mm=260),
            response_locale="en",
            speaker=lambda *args: speaker_calls.append(args),
            arm_segment_executor=lambda *args: arm_calls.append(args),
            behaviors=(GoalSeekingBehavior(),),
            navigation_limits=short_limits(20),
            tick_hook=waiting_tick(published),
        )

        result = runtime.run()

        self.assertTrue(result.clean_shutdown)
        self.assertEqual(result.workers_alive, ())
        self.assertEqual(speaker_calls, [])
        self.assertEqual(arm_calls, [])


class ExclusiveArmTests(unittest.TestCase):
    def test_arm_waits_for_stopped_boundary_never_overlaps_and_resumes(self):
        arm_active = threading.Event()
        arm_started = threading.Event()
        release_first_segment = threading.Event()
        published = threading.Event()
        plant, supervisor, inbox, _published = make_stack(
            published=published,
            plant_class=OverlapCheckingSimulator,
            plant_kwargs={"arm_active": arm_active},
        )
        segments = []
        first = [True]
        runtime_holder = []

        def arm_executor(speed_dps, duration_ms, cancel_event):
            self.assertFalse(cancel_event.is_set())
            arm_active.set()
            arm_started.set()
            segments.append((speed_dps, duration_ms))
            if first[0]:
                first[0] = False
                release_first_segment.wait()
            arm_active.clear()

        tick_number = [0]

        def tick(cancel_event, _interval):
            tick_number[0] += 1
            if tick_number[0] == 1:
                runtime = runtime_holder[0]
                self.assertTrue(runtime._pause_requested.wait(1))
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=expression_for,
            arm_segment_executor=arm_executor,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(7),
            policy=replace(
                ConcurrentRuntimePolicy(),
                gesture_cooldown_ms=0,
                max_gestures=1,
            ),
            tick_hook=tick,
        )
        runtime_holder.append(runtime)
        thread, completed, values = run_async(runtime)

        self.assertTrue(arm_started.wait(1))
        self.assertEqual(plant.drive_count, 0)
        release_first_segment.set()
        self.assertTrue(completed.wait(2))
        thread.join(1)
        self.assertNotIn("error", values)
        result = values["result"]

        self.assertFalse(plant.overlap_detected)
        self.assertGreater(plant.drive_count, 0)
        self.assertEqual(
            segments,
            [(900, 180), (-900, 180)],
        )
        self.assertEqual(result.metrics.navigation_pause_requests, 1)
        self.assertEqual(result.metrics.navigation_pause_acks, 1)
        self.assertEqual(result.metrics.gestures_completed, 1)
        event_kinds = [event.kind for event in result.events]
        self.assertLess(
            event_kinds.index("navigation_pause_ack"),
            event_kinds.index("arm_segment_started"),
        )
        self.assertLess(
            event_kinds.index("gesture_completed"),
            event_kinds.index("navigation_pause_released"),
        )

    def test_cancel_wakes_active_pause_and_navigation_terminal_stops(self):
        arm_active = threading.Event()
        arm_started = threading.Event()
        published = threading.Event()
        plant, supervisor, inbox, _published = make_stack(
            published=published,
            plant_class=OverlapCheckingSimulator,
            plant_kwargs={"arm_active": arm_active},
        )
        runtime_holder = []

        def arm_executor(_speed, _duration, cancel_event):
            arm_active.set()
            arm_started.set()
            cancel_event.wait()
            arm_active.clear()

        def tick(cancel_event, _interval):
            runtime = runtime_holder[0]
            self.assertTrue(runtime._pause_requested.wait(1))
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=expression_for,
            arm_segment_executor=arm_executor,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(30),
            tick_hook=tick,
        )
        runtime_holder.append(runtime)
        thread, completed, values = run_async(runtime)

        self.assertTrue(arm_started.wait(1))
        runtime.cancel()
        self.assertTrue(completed.wait(2))
        thread.join(1)
        self.assertNotIn("error", values)

        result = values["result"]
        self.assertEqual(
            result.navigation.termination,
            NAVIGATION_ABORTED,
        )
        self.assertTrue(result.navigation.terminal_stop_verified)
        self.assertFalse(plant.overlap_detected)
        self.assertEqual(
            plant.applied_pulses[-1].kind,
            "STOP",
        )
        self.assertEqual(result.metrics.gestures_completed, 0)
        self.assertEqual(result.metrics.arm_cancellations, 1)
        self.assertEqual(result.metrics.arm_segments, 1)
        self.assertEqual(result.metrics.arm_elapsed_ms, 180)
        self.assertTrue(any(
            event.kind == "arm_cancelled"
            for event in result.events
        ))

    def test_hung_arm_callback_times_out_and_terminally_stops(self):
        arm_started = threading.Event()
        release_arm = threading.Event()
        arm_returned = threading.Event()
        plant, supervisor, inbox, published = make_stack()
        runtime_holder = []
        tick_number = [0]

        def arm_executor(_speed, _duration, _cancel_event):
            arm_started.set()
            release_arm.wait()
            arm_returned.set()

        def tick(cancel_event, _interval):
            tick_number[0] += 1
            if tick_number[0] == 1:
                self.assertTrue(
                    runtime_holder[0]._pause_requested.wait(1)
                )
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=expression_for,
            arm_segment_executor=arm_executor,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(30),
            policy=replace(
                ConcurrentRuntimePolicy(),
                arm_exclusive_timeout_ms=25,
                shutdown_join_timeout_s=0.01,
            ),
            tick_hook=tick,
        )
        runtime_holder.append(runtime)
        thread, completed, values = run_async(runtime)

        self.assertTrue(arm_started.wait(1))
        self.assertTrue(completed.wait(1))
        thread.join(1)
        self.assertNotIn("error", values)
        result = values["result"]

        self.assertEqual(
            result.navigation.termination,
            NAVIGATION_ABORTED,
        )
        self.assertTrue(result.navigation.terminal_stop_verified)
        self.assertEqual(result.metrics.arm_exclusive_timeouts, 1)
        self.assertIn("arm", result.workers_alive)
        self.assertFalse(result.clean_shutdown)
        self.assertEqual(plant.applied_pulses[-1].kind, "STOP")
        self.assertTrue(any(
            event.kind == "arm_exclusive_timeout"
            for event in result.events
        ))

        release_arm.set()
        self.assertTrue(arm_returned.wait(1))

    def test_failed_arm_dispatch_is_charged_before_callback(self):
        arm_called = threading.Event()
        plant, supervisor, inbox, published = make_stack()
        runtime_holder = []
        tick_number = [0]

        def arm_executor(_speed, _duration, _cancel_event):
            arm_called.set()
            raise RuntimeError("synthetic arm failure")

        def tick(cancel_event, _interval):
            tick_number[0] += 1
            if tick_number[0] == 1:
                self.assertTrue(
                    runtime_holder[0]._pause_requested.wait(1)
                )
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=expression_for,
            arm_segment_executor=arm_executor,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(6),
            policy=replace(
                ConcurrentRuntimePolicy(),
                max_gestures=1,
            ),
            tick_hook=tick,
        )
        runtime_holder.append(runtime)

        result = runtime.run()

        self.assertTrue(arm_called.is_set())
        self.assertEqual(result.metrics.arm_segments, 1)
        self.assertEqual(result.metrics.arm_elapsed_ms, 180)
        self.assertEqual(result.metrics.gestures_completed, 0)
        self.assertEqual(result.metrics.gesture_drops, 1)
        self.assertTrue(any(
            event.kind == "arm_failure"
            for event in result.events
        ))

    def test_one_gesture_per_epoch_and_host_budgets(self):
        planner_calls = []
        segments = []
        published = threading.Event()
        plant, supervisor, inbox, _published = make_stack(
            published=published
        )
        runtime_holder = []

        def planner(snapshot):
            planner_calls.append(snapshot.obstruction_epoch)
            return expression_for(snapshot)

        def arm_executor(speed, duration, _cancel_event):
            segments.append((speed, duration))

        tick_number = [0]

        def tick(cancel_event, _interval):
            tick_number[0] += 1
            if tick_number[0] == 1:
                self.assertTrue(
                    runtime_holder[0]._pause_requested.wait(1)
                )
            if cancel_event.is_set():
                return
            self.assertTrue(published.wait(1))
            published.clear()

        runtime = ConcurrentBehaviorRuntime(
            plant,
            supervisor,
            inbox,
            waypoint(),
            response_locale="en",
            expression_planner=planner,
            arm_segment_executor=arm_executor,
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(8),
            policy=replace(
                ConcurrentRuntimePolicy(),
                max_gestures=1,
                max_arm_time_ms=360,
                gesture_cooldown_ms=0,
            ),
            tick_hook=tick,
        )
        runtime_holder.append(runtime)

        result = runtime.run()

        self.assertTrue(planner_calls)
        self.assertTrue(all(
            planner_calls.count(epoch) == 1
            for epoch in set(planner_calls)
        ))
        self.assertEqual(len(segments), 2)
        self.assertEqual(result.metrics.gestures_started, 1)
        self.assertEqual(result.metrics.arm_elapsed_ms, 360)

        blocked_plant, blocked_supervisor, blocked_inbox, signal = (
            make_stack()
        )
        blocked_calls = []

        blocked_runtime = ConcurrentBehaviorRuntime(
            blocked_plant,
            blocked_supervisor,
            blocked_inbox,
            waypoint(),
            response_locale="en",
            expression_planner=expression_for,
            arm_segment_executor=lambda *args: blocked_calls.append(args),
            behaviors=(GoalSeekingBehavior(), ObstacleAvoidanceBehavior()),
            navigation_limits=short_limits(5),
            policy=replace(
                ConcurrentRuntimePolicy(),
                max_gestures=0,
            ),
            tick_hook=waiting_tick(signal),
        )

        blocked_result = blocked_runtime.run()

        self.assertEqual(blocked_calls, [])
        self.assertGreaterEqual(blocked_result.metrics.gesture_drops, 1)


if __name__ == "__main__":
    unittest.main()
