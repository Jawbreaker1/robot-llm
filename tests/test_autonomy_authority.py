import threading
import unittest

from robot_agent.autonomy_authority import (
    IDLE_EXPLORATION,
    USER,
    GoalLeaseCoordinator,
    IdleDutyRearmGuard,
    UserGoalReservation,
)
from robot_agent.navigation_contract import NavigationContractError
from robot_agent.navigation_state import (
    ClearanceEvidence,
    NavigationSnapshot,
    PoseEstimate,
)


def stopped_snapshot(
    state_version=1,
    touch_pressed=False,
    active_faults=(),
    goal_epoch=40,
    plan_revision=7,
):
    return NavigationSnapshot(
        robot_id="ev3rstorm-sim",
        controller_instance_id="controller-one",
        goal_id="authority-boundary",
        goal_epoch=goal_epoch,
        plan_revision=plan_revision,
        state_version=state_version,
        world_model_version=1,
        captured_at_host_ms=10_000,
        state_observed_at_ms=10_000,
        pose=PoseEstimate(200, 200, 0),
        left_encoder_mdeg=0,
        right_encoder_mdeg=0,
        motors_running=False,
        touch_pressed=touch_pressed,
        active_faults=active_faults,
        clearance=ClearanceEvidence(
            source="simulation_metric",
            observed_at_ms=10_000,
            near_obstacle_latched=False,
            forward_mm=500,
            left_mm=500,
            right_mm=500,
        ),
    )


def coordinator(idle_enabled=False):
    return GoalLeaseCoordinator(
        "ev3rstorm-sim",
        "controller-one",
        starting_goal_epoch=40,
        starting_plan_revision=7,
        idle_enabled=idle_enabled,
    )


class GoalLeaseCoordinatorTests(unittest.TestCase):
    def test_idle_is_opt_in_and_exclusive(self):
        authority = coordinator()

        self.assertIsNone(authority.try_acquire_idle())
        authority.set_idle_enabled(True)
        lease = authority.try_acquire_idle()

        self.assertIsNotNone(lease)
        self.assertEqual(lease.owner, IDLE_EXPLORATION)
        self.assertEqual(lease.goal_epoch, 40)
        self.assertEqual(lease.plan_revision, 7)
        self.assertIsNone(authority.try_acquire_idle())
        self.assertTrue(
            authority.release(lease, stopped_snapshot(), True)
        )
        second = authority.try_acquire_idle()
        self.assertEqual(second.goal_epoch, 41)
        self.assertEqual(second.plan_revision, 8)

    def test_duty_rearm_guard_atomically_blocks_new_goal_claims(self):
        authority = coordinator()
        guard = authority.begin_idle_duty_rearm()

        self.assertIsNone(authority.try_acquire_idle())
        with self.assertRaises(NavigationContractError) as caught:
            authority.set_idle_enabled(True)
        self.assertEqual(
            caught.exception.code,
            "idle_duty_rearm_in_progress",
        )
        with self.assertRaises(NavigationContractError) as caught:
            authority.reserve_user("racing-user")
        self.assertEqual(
            caught.exception.code,
            "idle_duty_rearm_in_progress",
        )
        with self.assertRaises(NavigationContractError) as caught:
            authority.finish_idle_duty_rearm(
                IdleDutyRearmGuard(guard.generation)
            )
        self.assertEqual(
            caught.exception.code,
            "stale_idle_duty_rearm_guard",
        )

        authority.finish_idle_duty_rearm(guard)
        authority.set_idle_enabled(True)
        lease = authority.try_acquire_idle()
        self.assertIsNotNone(lease)
        self.assertTrue(authority.release(
            lease,
            stopped_snapshot(
                goal_epoch=lease.goal_epoch,
                plan_revision=lease.plan_revision,
            ),
            True,
        ))

    def test_user_reservation_cancels_idle_before_new_epoch_activates(self):
        authority = coordinator(idle_enabled=True)
        idle = authority.try_acquire_idle()

        reservation = authority.reserve_user("walk-to-johan")

        self.assertTrue(idle.cancel_event.is_set())
        self.assertIsNone(authority.try_acquire_idle())
        with self.assertRaises(NavigationContractError) as caught:
            authority.activate_user(reservation)
        self.assertEqual(
            caught.exception.code,
            "goal_owner_still_active",
        )

        self.assertTrue(
            authority.release(idle, stopped_snapshot(), True)
        )
        user = authority.activate_user(reservation)

        self.assertEqual(user.owner, USER)
        self.assertGreater(user.goal_epoch, idle.goal_epoch)
        self.assertGreater(user.plan_revision, idle.plan_revision)
        self.assertIsNone(authority.try_acquire_idle())
        self.assertTrue(
            authority.release(
                user,
                stopped_snapshot(
                    state_version=2,
                    goal_epoch=user.goal_epoch,
                    plan_revision=user.plan_revision,
                ),
                True,
            )
        )

    def test_reserving_user_is_idempotent_only_for_same_request(self):
        authority = coordinator(idle_enabled=True)

        first = authority.reserve_user("request-one")
        repeated = authority.reserve_user("request-one")

        self.assertIs(first, repeated)
        with self.assertRaises(NavigationContractError) as caught:
            authority.reserve_user("request-two")
        self.assertEqual(
            caught.exception.code,
            "user_goal_already_pending",
        )

    def test_user_reservation_cancel_is_exact_and_idempotent(self):
        authority = coordinator(idle_enabled=True)
        reservation = authority.reserve_user("request-one")
        equal_but_distinct = UserGoalReservation(
            reservation.request_id,
            reservation.generation,
        )

        with self.assertRaises(NavigationContractError) as caught:
            authority.cancel_user_reservation(equal_but_distinct)
        self.assertEqual(caught.exception.code, "stale_user_reservation")
        self.assertEqual(
            authority.state.pending_user_request_id,
            reservation.request_id,
        )

        self.assertTrue(
            authority.cancel_user_reservation(reservation)
        )
        self.assertFalse(
            authority.cancel_user_reservation(reservation)
        )
        self.assertIsNone(authority.state.pending_user_request_id)

    def test_old_cancel_retry_cannot_cancel_newer_reservation(self):
        authority = coordinator(idle_enabled=True)
        first = authority.reserve_user("request-one")
        self.assertTrue(authority.cancel_user_reservation(first))
        second = authority.reserve_user("request-two")

        self.assertFalse(authority.cancel_user_reservation(first))
        self.assertEqual(
            authority.state.pending_user_request_id,
            second.request_id,
        )
        self.assertTrue(authority.cancel_user_reservation(second))
        self.assertFalse(authority.cancel_user_reservation(first))
        self.assertFalse(authority.cancel_user_reservation(second))

    def test_cancelled_user_claim_does_not_uncancel_idle_lease(self):
        authority = coordinator(idle_enabled=True)
        idle = authority.try_acquire_idle()
        reservation = authority.reserve_user("request-one")

        self.assertTrue(idle.cancel_event.is_set())
        self.assertTrue(
            authority.cancel_user_reservation(reservation)
        )
        self.assertTrue(idle.cancel_event.is_set())
        self.assertFalse(authority.is_current_idle(idle))
        self.assertIsNone(authority.try_acquire_idle())

        self.assertTrue(authority.release(
            idle,
            stopped_snapshot(
                goal_epoch=idle.goal_epoch,
                plan_revision=idle.plan_revision,
            ),
            True,
        ))
        next_idle = authority.try_acquire_idle()
        self.assertIsNotNone(next_idle)
        self.assertTrue(authority.release(
            next_idle,
            stopped_snapshot(
                state_version=2,
                goal_epoch=next_idle.goal_epoch,
                plan_revision=next_idle.plan_revision,
            ),
            True,
        ))

    def test_cancel_and_activate_race_has_exactly_one_winner(self):
        for _attempt in range(30):
            authority = coordinator(idle_enabled=True)
            reservation = authority.reserve_user("race-request")
            barrier = threading.Barrier(3)
            result = {}

            def cancel_user():
                barrier.wait()
                try:
                    result["cancelled"] = (
                        authority.cancel_user_reservation(reservation)
                    )
                except NavigationContractError as error:
                    result["cancel_error"] = error.code

            def activate_user():
                barrier.wait()
                try:
                    result["user"] = authority.activate_user(
                        reservation
                    )
                except NavigationContractError as error:
                    result["activate_error"] = error.code

            cancel_thread = threading.Thread(target=cancel_user)
            activate_thread = threading.Thread(target=activate_user)
            cancel_thread.start()
            activate_thread.start()
            barrier.wait()
            cancel_thread.join()
            activate_thread.join()

            if result.get("cancelled"):
                self.assertEqual(
                    result.get("activate_error"),
                    "stale_user_reservation",
                )
                self.assertNotIn("user", result)
                self.assertIsNone(authority.state.active_owner)
            else:
                self.assertEqual(
                    result.get("cancel_error"),
                    "user_reservation_already_activated",
                )
                user = result["user"]
                self.assertEqual(user.owner, USER)
                self.assertTrue(authority.release(
                    user,
                    stopped_snapshot(
                        goal_epoch=user.goal_epoch,
                        plan_revision=user.plan_revision,
                    ),
                    True,
                ))

    def test_unverified_or_hazardous_release_faults_closed(self):
        unsafe_values = (
            (False, stopped_snapshot()),
            (True, stopped_snapshot(goal_epoch=39)),
            (True, stopped_snapshot(touch_pressed=True)),
            (
                True,
                stopped_snapshot(active_faults=("motor-stall",)),
            ),
        )
        for verified, snapshot in unsafe_values:
            with self.subTest(verified=verified, snapshot=snapshot):
                authority = coordinator(idle_enabled=True)
                lease = authority.try_acquire_idle()

                self.assertFalse(
                    authority.release(lease, snapshot, verified)
                )

                self.assertTrue(authority.state.faulted)
                self.assertIsNone(authority.try_acquire_idle())
                with self.assertRaises(NavigationContractError) as caught:
                    authority.reserve_user("blocked-after-fault")
                self.assertEqual(
                    caught.exception.code,
                    "goal_authority_faulted",
                )

    def test_disabling_idle_cancels_current_lease(self):
        authority = coordinator(idle_enabled=True)
        lease = authority.try_acquire_idle()

        authority.set_idle_enabled(False)

        self.assertTrue(lease.cancel_event.is_set())
        self.assertFalse(authority.is_current_idle(lease))
        self.assertTrue(
            authority.release(lease, stopped_snapshot(), True)
        )
        self.assertIsNone(authority.try_acquire_idle())

    def test_user_and_idle_acquisition_race_never_grants_both(self):
        for _attempt in range(30):
            authority = coordinator(idle_enabled=True)
            barrier = threading.Barrier(3)
            result = {}

            def acquire_idle():
                barrier.wait()
                result["idle"] = authority.try_acquire_idle()

            def reserve_user():
                barrier.wait()
                result["user"] = authority.reserve_user(
                    "race-request"
                )

            idle_thread = threading.Thread(target=acquire_idle)
            user_thread = threading.Thread(target=reserve_user)
            idle_thread.start()
            user_thread.start()
            barrier.wait()
            idle_thread.join()
            user_thread.join()

            self.assertEqual(
                authority.state.pending_user_request_id,
                "race-request",
            )
            self.assertIsNone(authority.try_acquire_idle())
            idle = result["idle"]
            if idle is not None:
                self.assertTrue(idle.cancel_event.is_set())
                self.assertEqual(
                    authority.state.active_owner,
                    IDLE_EXPLORATION,
                )
                self.assertTrue(
                    authority.release(
                        idle,
                        stopped_snapshot(
                            goal_epoch=idle.goal_epoch,
                            plan_revision=idle.plan_revision,
                        ),
                        True,
                    )
                )
            user = authority.activate_user(result["user"])
            self.assertEqual(user.owner, USER)


if __name__ == "__main__":
    unittest.main()
