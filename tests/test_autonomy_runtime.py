import itertools
import json
import subprocess
import sys
import threading
import unittest

from robot_agent.autonomy_authority import (
    USER,
    GoalLeaseCoordinator,
)
from robot_agent.autonomy_contract import FORWARD
from robot_agent.autonomy_perception import ExplorationMemory
from robot_agent.autonomy_runtime import (
    IDLE_HELD,
    IDLE_MISSION_STALE,
    IDLE_NO_FEASIBLE_CANDIDATES,
    IDLE_PREEMPTED,
    IDLE_SELECTION_FAILED,
    IDLE_SELECTION_STALE,
    IDLE_SESSION_BUDGET_EXHAUSTED,
    IDLE_TASK_COMPLETED,
    IdleExplorationService,
    IdleSessionLimits,
)
from robot_agent.autonomy_runtime_contract import IdleDutyCycleLimits
from robot_agent.navigation_contract import (
    DriveCalibrationProfile,
    MotionAuthority,
    NavigationContractError,
    WaypointGoal,
)
from robot_agent.navigation_mission_contract import MissionLimits
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


def _profile():
    return DriveCalibrationProfile(
        calibration_id="idle-runtime-fixture-v1",
        status="simulation_only",
        surface="synthetic",
        left_motor_sign=1,
        right_motor_sign=1,
        encoder_mdeg_per_mm=1_800,
        encoder_mdeg_per_body_degree=2_000,
        max_wheel_speed_dps=250,
        max_pulse_ms=120,
    )


def _stack(plant_class=DifferentialDriveSimulator, world=None):
    motion_authority = MotionAuthority()
    plant = plant_class(
        world=world or SimulationWorld(1_300, 900),
        profile=_profile(),
        initial_pose=PoseEstimate(650, 450, 0),
        motion_authority=motion_authority,
    )
    decision_ids = itertools.count(1)
    supervisor = MotionSupervisor(
        plant.profile,
        plant.clock_ms,
        plant.robot_id,
        plant.controller_instance_id,
        motion_authority,
        policy=MotionPolicy(max_pulse_ms=120),
        id_factory=lambda: "idle-decision-{}".format(
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
    authority = GoalLeaseCoordinator(
        plant.robot_id,
        plant.controller_instance_id,
        starting_goal_epoch=40,
        starting_plan_revision=10,
        idle_enabled=True,
    )
    return plant, supervisor, inbox, authority


def _selection(context, decision="SELECT", candidate_id=None):
    value = {
        "schema": "robot-autonomy-interest-selection/v1",
        "proposal_id": context.proposal_id,
        "robot_id": context.robot_id,
        "controller_instance_id": context.controller_instance_id,
        "autonomy_session_id": context.autonomy_session_id,
        "lease_generation": context.lease_generation,
        "candidate_set_id": context.candidate_set_id,
        "based_on_state_version": context.state_version,
        "based_on_world_model_version": context.world_model_version,
        "decision": decision,
        "confidence_milli": 900,
    }
    if decision == "SELECT":
        value["selected_candidate_id"] = (
            candidate_id or context.candidates[0].candidate_id
        )
    else:
        value["reason_code"] = "fixture_declined"
    return json.dumps(value).encode("utf-8")


def _release_unused_user_lease(
    plant,
    supervisor,
    authority,
    lease,
):
    probe = WaypointGoal(
        goal_id="user-release-probe",
        goal_epoch=lease.goal_epoch,
        plan_revision=lease.plan_revision,
        target_x_mm=0,
        target_y_mm=0,
        tolerance_mm=1,
    )
    current = plant.observe(probe)
    goal = WaypointGoal(
        goal_id="unused-user-goal",
        goal_epoch=lease.goal_epoch,
        plan_revision=lease.plan_revision,
        target_x_mm=current.pose.x_mm,
        target_y_mm=current.pose.y_mm,
        tolerance_mm=1,
    )
    before = plant.observe(goal)
    stop = supervisor.force_stop(
        before,
        reason_code="unused_user_terminal_stop",
    )
    after = plant.apply(stop, goal)
    return authority.release(lease, after, True)


class RecordingSelector:
    def __init__(self, decision="SELECT"):
        self.decision = decision
        self.contexts = []

    def __call__(self, context):
        self.contexts.append(context)
        return _selection(context, self.decision)


class RecordingMemory(ExplorationMemory):
    def __init__(self):
        super().__init__(grid_mm=50)
        self.attempted_keys = []
        self.completed_keys = []

    def record_attempt(self, key):
        self.attempted_keys.append(key)
        super().record_attempt(key)

    def record_completed(self, key):
        self.completed_keys.append(key)
        super().record_completed(key)


class ManualClock:
    def __init__(self, now_ms=10_000):
        self.now_ms = now_ms

    def __call__(self):
        return self.now_ms


class ExpiringAttemptMemory(RecordingMemory):
    def __init__(self, clock):
        super().__init__()
        self.clock = clock
        self.deadline_ms = None

    def record_attempt(self, key):
        super().record_attempt(key)
        if self.deadline_ms is not None:
            self.clock.now_ms = self.deadline_ms


class WorldChangingOnFirstDrive(DifferentialDriveSimulator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.changed = False

    def apply(self, pulse, goal):
        if pulse.kind == "DRIVE" and not self.changed:
            self.changed = True
            self.update_world(self.world)
        return super().apply(pulse, goal)


class WorldChangingOnEveryDrive(DifferentialDriveSimulator):
    def apply(self, pulse, goal):
        if pulse.kind == "DRIVE":
            self.update_world(self.world)
        return super().apply(pulse, goal)


class CallbackOnFirstDrive(DifferentialDriveSimulator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.callback = None
        self.callback_ran = False

    def apply(self, pulse, goal):
        if (
            pulse.kind == "DRIVE"
            and not self.callback_ran
            and self.callback is not None
        ):
            self.callback_ran = True
            self.callback()
        return super().apply(pulse, goal)


class CallbackOnObserve(DifferentialDriveSimulator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.observe_callback = None
        self.callback_ran = False

    def observe(self, goal):
        if (
            self.observe_callback is not None
            and not self.callback_ran
        ):
            self.callback_ran = True
            self.observe_callback()
        return super().observe(goal)


class WorldChangingOnMissionActivation(DifferentialDriveSimulator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.changed = False

    def observe(self, goal):
        if (
            not self.changed
            and goal.goal_id.startswith("candidate-set-")
        ):
            self.changed = True
            self.update_world(self.world)
        return super().observe(goal)


class IdleExplorationRuntimeTests(unittest.TestCase):
    def test_idle_runtime_import_has_no_physical_transport_side_effect(self):
        code = (
            "import sys;"
            "import robot_agent.autonomy_runtime;"
            "assert 'robot_agent.robot_api' not in sys.modules;"
            "assert 'robot_agent.supervisor_transport' not in sys.modules;"
            "assert 'robot_agent.ev3_hal' not in sys.modules"
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

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_result_contract_lazy_exports_do_not_load_executor(self):
        code = (
            "import sys, robot_agent;"
            "_ = robot_agent.IdleSessionLimits;"
            "_ = robot_agent.IdleSessionResult;"
            "_ = robot_agent.IdleTaskResult;"
            "assert 'robot_agent.autonomy_runtime' not in sys.modules;"
            "assert 'robot_agent.autonomy_task' not in sys.modules;"
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

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_selected_host_waypoint_completes_and_stops_safely(self):
        plant, supervisor, inbox, authority = _stack()
        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=_selection,
            clock_ms=plant.clock_ms,
        )

        result = service.run_once()

        self.assertEqual(result.termination, IDLE_TASK_COMPLETED)
        self.assertTrue(result.completed)
        self.assertTrue(result.terminal_stop_verified)
        self.assertFalse(result.final_snapshot.motors_running)
        self.assertEqual(plant.collision_count, 0)
        self.assertTrue(
            any(pulse.kind == "DRIVE" for pulse in plant.applied_pulses)
        )
        self.assertEqual(plant.applied_pulses[-1].kind, "STOP")
        self.assertIsNone(authority.state.active_owner)

    def test_idle_autonomy_propagates_mapping_observations_through_stop(self):
        plant, supervisor, inbox, authority = _stack()
        observations = []
        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=_selection,
            clock_ms=plant.clock_ms,
            observation_sink=observations.append,
        )

        result = service.run_once()

        self.assertTrue(result.completed)
        self.assertTrue(observations)
        self.assertEqual(observations[-1], result.final_snapshot)
        self.assertFalse(observations[-1].motors_running)

    def test_hold_releases_safely_without_driving(self):
        plant, supervisor, inbox, authority = _stack()
        selector = RecordingSelector("HOLD")
        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=selector,
            clock_ms=plant.clock_ms,
        )

        result = service.run_once()

        self.assertEqual(result.termination, IDLE_HELD)
        self.assertEqual(len(selector.contexts), 1)
        self.assertTrue(result.terminal_stop_verified)
        self.assertFalse(result.final_snapshot.motors_running)
        self.assertFalse(
            any(pulse.kind == "DRIVE" for pulse in plant.applied_pulses)
        )
        self.assertIsNone(authority.state.active_owner)

    def test_empty_host_menu_never_calls_selector(self):
        blocked_world = SimulationWorld(
            1_300,
            900,
            (
                CircleObstacle("front", 770, 450, 30),
                CircleObstacle("left", 735, 535, 30),
                CircleObstacle("right", 735, 365, 30),
            ),
        )
        plant, supervisor, inbox, authority = _stack(
            world=blocked_world
        )
        selector = RecordingSelector()
        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=selector,
            clock_ms=plant.clock_ms,
        )

        result = service.run_once()

        self.assertEqual(
            result.termination,
            IDLE_NO_FEASIBLE_CANDIDATES,
        )
        self.assertEqual(selector.contexts, [])
        self.assertEqual(result.planner_calls, 0)
        self.assertTrue(result.terminal_stop_verified)
        self.assertFalse(
            any(pulse.kind == "DRIVE" for pulse in plant.applied_pulses)
        )

    def test_late_selection_after_user_reservation_is_dropped(self):
        plant, supervisor, inbox, authority = _stack()
        selector_entered = threading.Event()
        return_selection = threading.Event()
        selector_contexts = []

        def late_selector(context):
            selector_contexts.append(context)
            selector_entered.set()
            if not return_selection.wait(2):
                raise RuntimeError("fixture selector was not released")
            return _selection(context)

        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=late_selector,
            clock_ms=plant.clock_ms,
        )
        outcome = {}

        def run_idle():
            try:
                outcome["result"] = service.run_once()
            except Exception as error:
                outcome["error"] = error

        worker = threading.Thread(target=run_idle)
        worker.start()
        self.assertTrue(selector_entered.wait(2))
        idle_epoch = authority.state.last_allocated_goal_epoch
        reservation = authority.reserve_user("manual-waypoint")
        with self.assertRaises(NavigationContractError) as caught:
            authority.activate_user(reservation)
        self.assertEqual(caught.exception.code, "goal_owner_still_active")

        return_selection.set()
        worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertNotIn("error", outcome)
        result = outcome["result"]
        self.assertEqual(result.termination, IDLE_PREEMPTED)
        self.assertIn("SELECTION_PREEMPTED", result.trace)
        self.assertTrue(result.terminal_stop_verified)
        self.assertFalse(result.final_snapshot.motors_running)
        self.assertFalse(
            any(pulse.kind == "DRIVE" for pulse in plant.applied_pulses)
        )
        self.assertEqual(
            authority.state.pending_user_request_id,
            "manual-waypoint",
        )

        user_lease = authority.activate_user(reservation)

        self.assertEqual(user_lease.owner, USER)
        self.assertGreater(user_lease.goal_epoch, idle_epoch)
        self.assertEqual(plant.applied_pulses[-1].kind, "STOP")
        self.assertTrue(_release_unused_user_lease(
            plant,
            supervisor,
            authority,
            user_lease,
        ))

    def test_hung_selector_cannot_delay_user_activation(self):
        plant, supervisor, inbox, authority = _stack()
        selector_entered = threading.Event()
        release_selector = threading.Event()

        def hung_selector(context):
            selector_entered.set()
            release_selector.wait(5)
            return _selection(context)

        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=hung_selector,
            clock_ms=plant.clock_ms,
        )
        outcome = {}
        worker = threading.Thread(
            target=lambda: outcome.setdefault(
                "result",
                service.run_once(),
            )
        )
        worker.start()
        self.assertTrue(selector_entered.wait(2))

        reservation = authority.reserve_user("urgent-user-goal")
        worker.join(2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(
            outcome["result"].termination,
            IDLE_PREEMPTED,
        )
        self.assertTrue(outcome["result"].terminal_stop_verified)
        self.assertTrue(service.selector_gate.busy)
        user_lease = authority.activate_user(reservation)
        self.assertEqual(user_lease.owner, USER)
        self.assertFalse(
            any(pulse.kind == "DRIVE" for pulse in plant.applied_pulses)
        )

        release_selector.set()
        self.assertTrue(_release_unused_user_lease(
            plant,
            supervisor,
            authority,
            user_lease,
        ))

    def test_user_preemption_during_motion_stops_after_bounded_pulse(self):
        plant, supervisor, inbox, authority = _stack(
            plant_class=CallbackOnFirstDrive
        )
        reservation = {}
        plant.callback = lambda: reservation.setdefault(
            "value",
            authority.reserve_user("interrupt-active-idle"),
        )
        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=_selection,
            clock_ms=plant.clock_ms,
        )

        result = service.run_once()

        self.assertTrue(plant.callback_ran)
        self.assertEqual(result.termination, IDLE_PREEMPTED)
        self.assertTrue(result.terminal_stop_verified)
        self.assertFalse(result.final_snapshot.motors_running)
        drive_pulses = tuple(
            pulse
            for pulse in plant.applied_pulses
            if pulse.kind == "DRIVE"
        )
        self.assertEqual(len(drive_pulses), 1)
        self.assertEqual(plant.applied_pulses[-1].kind, "STOP")
        user_lease = authority.activate_user(reservation["value"])
        self.assertGreater(
            user_lease.goal_epoch,
            drive_pulses[0].goal_epoch,
        )
        self.assertTrue(_release_unused_user_lease(
            plant,
            supervisor,
            authority,
            user_lease,
        ))

    def test_selector_state_or_world_change_drops_and_replans(self):
        for mutation in ("state", "world"):
            with self.subTest(mutation=mutation):
                plant, supervisor, inbox, authority = _stack()
                contexts = []

                def changing_selector(context):
                    contexts.append(context)
                    if len(contexts) == 1:
                        if mutation == "world":
                            plant.update_world(plant.world)
                        else:
                            bump_goal = WaypointGoal(
                                goal_id="selector-state-bump",
                                goal_epoch=99,
                                plan_revision=99,
                                target_x_mm=650,
                                target_y_mm=450,
                                tolerance_mm=1,
                            )
                            before = plant.observe(bump_goal)
                            stop = supervisor.force_stop(
                                before,
                                reason_code="fixture_state_bump",
                            )
                            plant.apply(stop, bump_goal)
                    return _selection(context)

                service = IdleExplorationService(
                    plant,
                    supervisor,
                    inbox,
                    authority,
                    selector=changing_selector,
                    clock_ms=plant.clock_ms,
                )

                result = service.run_once(
                    IdleSessionLimits(
                        max_tasks=1,
                        max_planner_calls=3,
                        max_stale_replans=2,
                    )
                )

                self.assertEqual(result.termination, IDLE_TASK_COMPLETED)
                self.assertEqual(result.planner_calls, 2)
                self.assertEqual(result.stale_replans, 1)
                self.assertIn("STALE_SELECTION_DROPPED", result.trace)
                self.assertGreater(
                    contexts[1].state_version,
                    contexts[0].state_version,
                )
                if mutation == "world":
                    self.assertGreater(
                        contexts[1].world_model_version,
                        contexts[0].world_model_version,
                    )
                else:
                    self.assertEqual(
                        contexts[1].world_model_version,
                        contexts[0].world_model_version,
                    )
                self.assertEqual(plant.collision_count, 0)

    def test_selection_deadline_is_exclusive_and_never_dispatches(self):
        plant, supervisor, inbox, authority = _stack()
        clock = ManualClock()

        def expires_at_return(context):
            clock.now_ms = context.valid_until_ms
            return _selection(context)

        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=expires_at_return,
            clock_ms=clock,
            selection_ttl_ms=100,
        )

        result = service.run_once(IdleSessionLimits(
            max_tasks=1,
            max_planner_calls=1,
            max_stale_replans=0,
        ))

        self.assertEqual(result.termination, IDLE_SELECTION_STALE)
        self.assertEqual(result.stale_replans, 0)
        self.assertTrue(result.terminal_stop_verified)
        self.assertFalse(
            any(pulse.kind == "DRIVE" for pulse in plant.applied_pulses)
        )

    def test_expiry_after_selection_before_plan_never_dispatches(self):
        plant, supervisor, inbox, authority = _stack()
        clock = ManualClock()
        memory = ExpiringAttemptMemory(clock)

        def records_deadline(context):
            memory.deadline_ms = context.valid_until_ms
            return _selection(context)

        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=records_deadline,
            clock_ms=clock,
            selection_ttl_ms=100,
            memory=memory,
        )

        result = service.run_once(IdleSessionLimits(
            max_tasks=1,
            max_planner_calls=1,
            max_stale_replans=0,
        ))

        self.assertEqual(result.termination, IDLE_SELECTION_STALE)
        self.assertEqual(result.stale_replans, 0)
        self.assertEqual(len(memory.attempted_keys), 1)
        self.assertIn("STALE_BEFORE_MISSION_DROPPED", result.trace)
        self.assertFalse(
            any(pulse.kind == "DRIVE" for pulse in plant.applied_pulses)
        )

    def test_zero_stale_replan_budget_is_never_overcounted(self):
        plant, supervisor, inbox, authority = _stack()

        def changes_world(context):
            plant.update_world(plant.world)
            return _selection(context)

        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=changes_world,
            clock_ms=plant.clock_ms,
        )

        result = service.run_once(IdleSessionLimits(
            max_tasks=1,
            max_planner_calls=2,
            max_stale_replans=0,
        ))

        self.assertEqual(result.termination, IDLE_SELECTION_STALE)
        self.assertEqual(result.stale_replans, 0)
        self.assertFalse(
            any(pulse.kind == "DRIVE" for pulse in plant.applied_pulses)
        )

    def test_world_change_during_drive_retries_as_fresh_idle_task(self):
        plant, supervisor, inbox, authority = _stack(
            plant_class=WorldChangingOnFirstDrive
        )
        selector = RecordingSelector()
        memory = RecordingMemory()
        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=selector,
            clock_ms=plant.clock_ms,
            memory=memory,
        )

        result = service.run_session(
            IdleSessionLimits(
                max_tasks=2,
                max_planner_calls=3,
                max_stale_replans=2,
            )
        )

        self.assertTrue(plant.changed)
        self.assertEqual(len(result.tasks), 2)
        self.assertEqual(
            tuple(task.termination for task in result.tasks),
            (IDLE_MISSION_STALE, IDLE_TASK_COMPLETED),
        )
        self.assertEqual(result.stale_replans, 1)
        self.assertEqual(result.tasks_completed, 1)
        self.assertTrue(result.terminal_stop_verified)
        self.assertEqual(plant.collision_count, 0)
        self.assertEqual(len(memory.attempted_keys), 2)
        self.assertEqual(len(memory.completed_keys), 1)
        self.assertEqual(
            memory.completed_keys[0],
            memory.attempted_keys[-1],
        )

    def test_mission_stale_without_replan_budget_stops_at_zero(self):
        plant, supervisor, inbox, authority = _stack(
            plant_class=WorldChangingOnFirstDrive
        )
        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=_selection,
            clock_ms=plant.clock_ms,
        )

        result = service.run_session(IdleSessionLimits(
            max_tasks=2,
            max_planner_calls=2,
            max_stale_replans=0,
        ))

        self.assertEqual(len(result.tasks), 1)
        self.assertEqual(
            result.tasks[0].termination,
            IDLE_MISSION_STALE,
        )
        self.assertEqual(result.stale_replans, 0)
        self.assertTrue(result.terminal_stop_verified)

    def test_world_change_at_mission_activation_replans_as_stale(self):
        plant, supervisor, inbox, authority = _stack(
            plant_class=WorldChangingOnMissionActivation
        )
        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=RecordingSelector(),
            clock_ms=plant.clock_ms,
        )

        result = service.run_session(IdleSessionLimits(
            max_tasks=2,
            max_planner_calls=3,
            max_stale_replans=2,
        ))

        self.assertTrue(plant.changed)
        self.assertEqual(
            tuple(task.termination for task in result.tasks),
            (IDLE_MISSION_STALE, IDLE_TASK_COMPLETED),
        )
        self.assertEqual(result.stale_replans, 1)
        self.assertEqual(result.tasks_completed, 1)
        self.assertTrue(result.terminal_stop_verified)
        self.assertEqual(plant.collision_count, 0)
        self.assertFalse(
            any(
                pulse.kind == "DRIVE"
                and pulse.based_on_world_model_version == 1
                for pulse in plant.applied_pulses
            )
        )

    def test_malformed_or_unknown_selection_fails_closed(self):
        def malformed_selector(_context):
            return b"{"

        def unknown_selector(context):
            return _selection(
                context,
                candidate_id="candidate-outside-host-menu",
            )

        for label, selector in (
            ("malformed", malformed_selector),
            ("unknown", unknown_selector),
        ):
            with self.subTest(label=label):
                plant, supervisor, inbox, authority = _stack()
                memory = RecordingMemory()
                service = IdleExplorationService(
                    plant,
                    supervisor,
                    inbox,
                    authority,
                    selector=selector,
                    clock_ms=plant.clock_ms,
                    memory=memory,
                )

                result = service.run_once()

                self.assertEqual(
                    result.termination,
                    IDLE_SELECTION_FAILED,
                )
                self.assertTrue(result.terminal_stop_verified)
                self.assertFalse(
                    any(
                        pulse.kind == "DRIVE"
                        for pulse in plant.applied_pulses
                    )
                )
                self.assertEqual(memory.attempted_keys, [])
                self.assertEqual(memory.completed_keys, [])
                self.assertIsNone(authority.state.active_owner)

    def test_completion_memory_is_committed_only_after_success(self):
        stale_plant, supervisor, inbox, authority = _stack(
            plant_class=WorldChangingOnFirstDrive
        )
        stale_memory = RecordingMemory()
        stale_service = IdleExplorationService(
            stale_plant,
            supervisor,
            inbox,
            authority,
            selector=_selection,
            clock_ms=stale_plant.clock_ms,
            memory=stale_memory,
        )

        stale = stale_service.run_once()

        self.assertEqual(stale.termination, IDLE_MISSION_STALE)
        self.assertEqual(len(stale_memory.attempted_keys), 1)
        self.assertEqual(stale_memory.completed_keys, [])

        plant, supervisor, inbox, authority = _stack()
        completed_memory = RecordingMemory()
        completed_service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=_selection,
            clock_ms=plant.clock_ms,
            memory=completed_memory,
        )

        completed = completed_service.run_once()

        self.assertTrue(completed.completed)
        self.assertEqual(len(completed_memory.attempted_keys), 1)
        self.assertEqual(
            completed_memory.completed_keys,
            completed_memory.attempted_keys,
        )

    def test_repeated_failed_cell_is_suppressed_across_public_calls(self):
        plant, supervisor, inbox, authority = _stack(
            plant_class=WorldChangingOnEveryDrive
        )
        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=_selection,
            clock_ms=plant.clock_ms,
        )
        limits = IdleSessionLimits(
            max_tasks=1,
            max_planner_calls=1,
            max_stale_replans=0,
        )

        attempts = tuple(service.run_once(limits) for _ in range(3))
        directions = tuple(
            next(
                candidate.relative_direction
                for candidate in task.candidates
                if candidate.candidate_id
                == task.selected_candidate_id
            )
            for task in attempts
        )

        self.assertEqual(
            tuple(task.termination for task in attempts),
            (IDLE_MISSION_STALE,) * 3,
        )
        self.assertEqual(directions[:2], (FORWARD, FORWARD))
        self.assertNotEqual(directions[2], FORWARD)
        self.assertTrue(
            all(task.terminal_stop_verified for task in attempts)
        )
        self.assertEqual(plant.collision_count, 0)

    def test_session_budgets_are_cumulative_across_idle_leases(self):
        plant, supervisor, inbox, authority = _stack()
        selector = RecordingSelector()
        task_limited = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=selector,
            clock_ms=plant.clock_ms,
        ).run_session(IdleSessionLimits(max_tasks=2))

        self.assertEqual(len(task_limited.tasks), 2)
        self.assertEqual(task_limited.tasks_completed, 2)
        self.assertEqual(
            task_limited.termination,
            IDLE_SESSION_BUDGET_EXHAUSTED,
        )

        plant, supervisor, inbox, authority = _stack()
        planner_limited = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=RecordingSelector(),
            clock_ms=plant.clock_ms,
        ).run_session(
            IdleSessionLimits(
                max_tasks=3,
                max_planner_calls=1,
            )
        )

        self.assertEqual(len(planner_limited.tasks), 1)
        self.assertEqual(planner_limited.planner_calls, 1)
        self.assertEqual(
            planner_limited.termination,
            IDLE_SESSION_BUDGET_EXHAUSTED,
        )

        probe_plant, probe_supervisor, probe_inbox, probe_authority = (
            _stack()
        )
        probe = IdleExplorationService(
            probe_plant,
            probe_supervisor,
            probe_inbox,
            probe_authority,
            selector=_selection,
            clock_ms=probe_plant.clock_ms,
        ).run_once()
        self.assertTrue(probe.completed)

        plant, supervisor, inbox, authority = _stack()
        action_motion_limited = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=RecordingSelector(),
            clock_ms=plant.clock_ms,
            mission_limits=MissionLimits(max_legs=1),
        ).run_session(
            IdleSessionLimits(
                max_tasks=3,
                max_actions=probe.mission.actions,
                max_total_motion_ms=probe.mission.total_motion_ms,
            )
        )

        self.assertEqual(len(action_motion_limited.tasks), 1)
        self.assertEqual(
            action_motion_limited.actions,
            probe.mission.actions,
        )
        self.assertEqual(
            action_motion_limited.total_motion_ms,
            probe.mission.total_motion_ms,
        )
        self.assertLessEqual(
            action_motion_limited.actions,
            probe.mission.actions,
        )
        self.assertLessEqual(
            action_motion_limited.total_motion_ms,
            probe.mission.total_motion_ms,
        )
        self.assertEqual(
            action_motion_limited.termination,
            IDLE_SESSION_BUDGET_EXHAUSTED,
        )

    def test_duty_cycle_budgets_persist_until_explicit_safe_rearm(self):
        plant, supervisor, inbox, authority = _stack(
            plant_class=CallbackOnObserve
        )
        service = IdleExplorationService(
            plant,
            supervisor,
            inbox,
            authority,
            selector=_selection,
            clock_ms=plant.clock_ms,
            duty_cycle_limits=IdleDutyCycleLimits(
                max_task_attempts=1,
                max_planner_calls=3,
                max_stale_replans=1,
                max_elapsed_ms=30_000,
                max_actions=200,
                max_total_motion_ms=20_000,
            ),
        )

        first = service.run_once()
        second = service.run_once()

        self.assertTrue(first.completed)
        self.assertEqual(
            second.termination,
            IDLE_SESSION_BUDGET_EXHAUSTED,
        )
        self.assertEqual(
            second.trace,
            ("DUTY_CYCLE_BUDGET_EXHAUSTED",),
        )
        duty_state = service.duty_cycle_state
        self.assertTrue(duty_state.exhausted)
        self.assertEqual(duty_state.task_attempts, 1)
        self.assertEqual(duty_state.planner_calls, first.planner_calls)
        self.assertEqual(duty_state.actions, first.mission.actions)
        self.assertEqual(
            duty_state.total_motion_ms,
            first.mission.total_motion_ms,
        )
        with self.assertRaises(NavigationContractError) as caught:
            service.rearm_idle_duty_cycle()
        self.assertEqual(
            caught.exception.code,
            "unsafe_idle_duty_rearm",
        )

        authority.set_idle_enabled(False)
        racing_enable = {}

        def attempt_enable_during_rearm():
            try:
                authority.set_idle_enabled(True)
            except NavigationContractError as error:
                racing_enable["error"] = error.code
            racing_enable["lease"] = authority.try_acquire_idle()

        plant.observe_callback = attempt_enable_during_rearm
        plant.callback_ran = False
        old_generation = service.duty_cycle_state.generation
        new_generation = service.rearm_idle_duty_cycle()

        self.assertTrue(plant.callback_ran)
        self.assertEqual(
            racing_enable["error"],
            "idle_duty_rearm_in_progress",
        )
        self.assertIsNone(racing_enable["lease"])
        self.assertFalse(authority.state.idle_enabled)
        self.assertIsNone(authority.state.active_owner)
        authority.set_idle_enabled(True)
        after_rearm = service.run_once()

        self.assertEqual(new_generation, old_generation + 1)
        self.assertTrue(after_rearm.completed)


if __name__ == "__main__":
    unittest.main()
