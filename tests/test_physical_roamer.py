import copy
import threading
import time
import unittest

import robot_agent.physical_roamer as physical_roamer_module
from robot_agent.physical_roamer import (
    ACTION_ADVANCE,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    EXPRESSION_EVENT_TTL_MS,
    HEARTBEAT_INTERVAL_SECONDS,
    MAX_COMMANDED_MOTION_MS,
    MAX_CONTROL_REQUESTS,
    MAX_EPISODE_SECONDS,
    MAX_HEARTBEAT_GAP_SECONDS,
    MAX_INFRARED_AGE_MS,
    MAX_PULSES,
    PULSE_ACCOUNTING_MS,
    PhysicalRoamer,
    REQUEST_TTL_MS,
    RUNTIME_PROFILE,
    TERMINATION_CANCELLED,
    TERMINATION_CLEANUP_FAILED,
    TERMINATION_HEARTBEAT_MISSED,
    TERMINATION_PULSE_BUDGET,
    TERMINATION_REMOTE_FAILURE,
    TERMINATION_REQUEST_BUDGET,
    TERMINATION_SAFETY_FAULT,
    TERMINATION_STALE_OBSERVATION,
    TERMINATION_TIME_BUDGET,
)


class FakeClock:
    def __init__(self):
        self.value = 0.0
        self.lock = threading.Lock()

    def monotonic(self):
        with self.lock:
            return self.value

    def advance(self, seconds):
        with self.lock:
            self.value += seconds

    def sleep(self, seconds):
        self.advance(seconds)


def infrared(blocked=False, **overrides):
    value = {
        "raw": 52 if not blocked else 20,
        "filtered": 52 if not blocked else 20,
        "blocked": blocked,
        "reason": (
            "clear_hysteresis_hold"
            if not blocked
            else "blocked_hysteresis_hold"
        ),
        "sample_count": 5,
        "observed_monotonic_ms": 1,
        "age_ms": 1,
        "fresh": True,
    }
    value.update(overrides)
    return value


def profile_description():
    return {
        "runtime_profile": RUNTIME_PROFILE,
        "motion_enabled": True,
        "remaining_motion_budget": 20,
        "remaining_motion_duration_ms": 3000,
        "max_session_ms": 20000,
        "poll_interval_ms": 150,
        "max_poll_lateness_ms": 400,
        "max_process_requests": 256,
        "capabilities": {
            "differential_drive_timed": {
                "enabled": False,
            },
            "semantic_drive_pulse": {
                "enabled": True,
                "actions": [
                    ACTION_ADVANCE,
                    ACTION_TURN_LEFT,
                    ACTION_TURN_RIGHT,
                ],
                "mapping": {
                    ACTION_ADVANCE: {
                        "left_speed_dps": 100,
                        "right_speed_dps": 100,
                    },
                    ACTION_TURN_LEFT: {
                        "left_speed_dps": -100,
                        "right_speed_dps": 100,
                    },
                    ACTION_TURN_RIGHT: {
                        "left_speed_dps": 100,
                        "right_speed_dps": -100,
                    },
                },
                "duration_ms": 150,
                "max_commands_per_process": 20,
                "max_total_duration_ms": 3000,
            },
        },
    }


class FakeSession:
    def __init__(
        self,
        clock,
        obstacles=(),
        request_cost=0.001,
        fail_operation=None,
        fault_status_index=None,
        stale_status_index=None,
        always_running=False,
        running_until_time=None,
        cancel_on_status_index=None,
        cancel_event=None,
        real_drive_yield=0.0,
        profile=RUNTIME_PROFILE,
        description=None,
        wait_closed_result=0,
    ):
        self.profile = profile
        self.clock = clock
        self.obstacles = list(obstacles)
        self.request_cost = request_cost
        self.fail_operation = fail_operation
        self.fault_status_index = fault_status_index
        self.stale_status_index = stale_status_index
        self.always_running = always_running
        self.running_until_time = running_until_time
        self.cancel_on_status_index = cancel_on_status_index
        self.cancel_event = cancel_event
        self.real_drive_yield = real_drive_yield
        self.description = (
            profile_description()
            if description is None
            else description
        )
        self.wait_closed_result = wait_closed_result
        self.wait_closed_timeouts = []
        self.calls = []
        self.status_count = 0
        self.state = "DISARMED"
        self.session_id = None

    def _gate(self, index):
        if not self.obstacles:
            return infrared(False)
        value = self.obstacles[min(index, len(self.obstacles) - 1)]
        return infrared(value)

    def request(self, operation, arguments=None, ttl_ms=500):
        arguments = {} if arguments is None else dict(arguments)
        called_at = self.clock.monotonic()
        self.calls.append((operation, arguments, ttl_ms, called_at))
        self.clock.advance(self.request_cost)
        if operation == self.fail_operation:
            raise OSError("simulated link failure")
        if operation == "describe":
            return dict(self.description)
        if operation == "claim":
            self.session_id = "fake-session"
            return {
                "status": "claimed",
                "session_id": self.session_id,
                "state": self.state,
            }
        if operation == "heartbeat":
            return {
                "status": "accepted",
                "sequence_id": arguments["sequence_id"],
                "heartbeat_timeout_ms": 500,
            }
        if operation == "status":
            index = self.status_count
            self.status_count += 1
            if (
                self.cancel_on_status_index == index
                and self.cancel_event is not None
            ):
                self.cancel_event.set()
            state = (
                "RUNNING"
                if (
                    self.state == "ARMED_IDLE"
                    and (
                        self.always_running
                        or (
                            self.running_until_time is not None
                            and self.clock.monotonic()
                            < self.running_until_time
                        )
                    )
                )
                else self.state
            )
            result = {
                "status": "ok",
                "state": state,
                "fault": None,
                "motion_allowed": state == "ARMED_IDLE",
                "session_active": self.session_id is not None,
                "active_command_id": (
                    "simulated-running"
                    if state == "RUNNING"
                    else None
                ),
                "heartbeat_age_ms": 1,
                "touch": 0,
                "infrared": self._gate(index),
            }
            if self.fault_status_index == index:
                result["status"] = "fault"
                result["state"] = "FAULT_LATCHED"
                result["fault"] = {"code": "simulated_fault"}
            if self.stale_status_index == index:
                result["infrared"] = infrared(False, stale=True)
            return result
        if operation == "arm":
            self.state = "ARMED_IDLE"
            return {
                "status": "ok",
                "state": self.state,
                "fault": None,
                "motion_allowed": True,
                "session_active": True,
                "active_command_id": None,
                "heartbeat_age_ms": 1,
                "touch": 0,
                "infrared": self._gate(
                    max(0, self.status_count - 1)
                ),
            }
        if operation == "drive_pulse":
            if self.real_drive_yield:
                time.sleep(self.real_drive_yield)
            return {
                "status": "ok",
                "state": "RUNNING",
                "fault": None,
                "motion_allowed": False,
                "session_active": True,
                "active_command_id": arguments["command_id"],
                "heartbeat_age_ms": 1,
                "touch": 0,
                "infrared": self._gate(
                    max(0, self.status_count - 1)
                ),
            }
        if operation == "release":
            self.state = "DISARMED"
            self.session_id = None
            return {
                "status": "ok",
                "state": self.state,
                "fault": None,
                "motion_allowed": False,
                "session_active": False,
                "active_command_id": None,
            }
        if operation in ("stop", "shutdown"):
            self.state = "DISARMED"
            self.session_id = None
            return {
                "status": "ok",
                "state": self.state,
                "fault": None,
                "motion_allowed": False,
                "session_active": False,
                "active_command_id": None,
                "stop_confirmed": True,
            }
        raise AssertionError(operation)

    def wait_closed(self, timeout_seconds=3.0):
        self.wait_closed_timeouts.append(timeout_seconds)
        return self.wait_closed_result

    def operations(self):
        return [call[0] for call in self.calls]

    def drive_calls(self):
        return [
            call for call in self.calls if call[0] == "drive_pulse"
        ]


class PhysicalRoamerTests(unittest.TestCase):
    def test_requires_exact_session_profile_before_any_request(self):
        clock = FakeClock()
        session = FakeSession(clock, profile="motion-free")

        with self.assertRaisesRegex(
            RuntimeError,
            "ir-roamer-v1",
        ):
            PhysicalRoamer(
                session,
                monotonic=clock.monotonic,
                sleep=clock.sleep,
            )

        self.assertFalse(session.calls)

    def test_describe_is_first_and_exact_before_claim(self):
        clock = FakeClock()
        session = FakeSession(clock)

        PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(session.operations()[0], "describe")
        self.assertEqual(session.operations().count("describe"), 1)
        self.assertLess(
            session.operations().index("describe"),
            session.operations().index("claim"),
        )

    def test_profile_description_mismatch_never_claims(self):
        expected_mapping = profile_description()["capabilities"][
            "semantic_drive_pulse"
        ]["mapping"]
        changed_mapping = copy.deepcopy(expected_mapping)
        changed_mapping[ACTION_ADVANCE]["left_speed_dps"] = 101
        mutations = (
            (("runtime_profile",), "other-profile"),
            (("motion_enabled",), False),
            (("remaining_motion_budget",), 19),
            (("remaining_motion_budget",), True),
            (("remaining_motion_duration_ms",), 2999),
            (("max_session_ms",), 19999),
            (("poll_interval_ms",), 149),
            (("max_poll_lateness_ms",), 399),
            (("max_process_requests",), 255),
            (("max_process_requests",), True),
            (
                (
                    "capabilities",
                    "differential_drive_timed",
                    "enabled",
                ),
                True,
            ),
            (
                (
                    "capabilities",
                    "semantic_drive_pulse",
                    "actions",
                ),
                [
                    ACTION_TURN_LEFT,
                    ACTION_ADVANCE,
                    ACTION_TURN_RIGHT,
                ],
            ),
            (
                (
                    "capabilities",
                    "semantic_drive_pulse",
                    "mapping",
                ),
                changed_mapping,
            ),
            (
                (
                    "capabilities",
                    "semantic_drive_pulse",
                    "duration_ms",
                ),
                151,
            ),
            (
                (
                    "capabilities",
                    "semantic_drive_pulse",
                    "max_commands_per_process",
                ),
                21,
            ),
            (
                (
                    "capabilities",
                    "semantic_drive_pulse",
                    "max_total_duration_ms",
                ),
                3001,
            ),
        )
        for path, replacement in mutations:
            with self.subTest(path=path, replacement=replacement):
                description = copy.deepcopy(profile_description())
                target = description
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = replacement
                clock = FakeClock()
                session = FakeSession(
                    clock,
                    description=description,
                )

                result = PhysicalRoamer(
                    session,
                    monotonic=clock.monotonic,
                    sleep=clock.sleep,
                ).run()

                self.assertEqual(
                    result.termination,
                    TERMINATION_REMOTE_FAILURE,
                )
                self.assertEqual(session.operations()[0], "describe")
                self.assertNotIn("claim", session.operations())
                self.assertFalse(session.drive_calls())

    def test_control_request_cap_reserves_direct_cleanup_slots(self):
        clock = FakeClock()
        session = FakeSession(clock)
        roamer = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        roamer._control_request_count = MAX_CONTROL_REQUESTS - 1

        result = roamer.run()

        self.assertEqual(
            result.termination,
            TERMINATION_REQUEST_BUDGET,
        )
        self.assertEqual(
            session.operations(),
            ["describe", "shutdown"],
        )
        self.assertFalse(session.drive_calls())
        shutdown = next(
            outcome
            for outcome in result.cleanup
            if outcome.operation == "shutdown"
        )
        self.assertTrue(shutdown.succeeded)

    def test_turn_is_stable_within_obstacle_and_alternates_next_episode(
        self,
    ):
        clock = FakeClock()
        session = FakeSession(
            clock,
            obstacles=(
                False,
                False,
                True,
                True,
                False,
                True,
                True,
            ),
        )

        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(result.termination, TERMINATION_PULSE_BUDGET)
        actions = [
            call[1]["action"] for call in session.drive_calls()
        ]
        self.assertEqual(
            actions[:6],
            [
                ACTION_ADVANCE,
                ACTION_TURN_LEFT,
                ACTION_TURN_LEFT,
                ACTION_ADVANCE,
                ACTION_TURN_RIGHT,
                ACTION_TURN_RIGHT,
            ],
        )
        calls = session.calls
        for index, call in enumerate(calls):
            if call[0] == "drive_pulse":
                self.assertEqual(calls[index - 1][0], "heartbeat")
                later = [item[0] for item in calls[index + 1:]]
                self.assertTrue(
                    "status" in later or "shutdown" in later
                )
        self.assertEqual(
            session.operations()[-1:],
            ["shutdown"],
        )

    def test_clear_running_observation_ends_blocked_episode(self):
        clock = FakeClock()
        session = FakeSession(clock, obstacles=(True,))
        request = session.request
        status_number = 0

        def clear_while_running(operation, arguments=None, ttl_ms=500):
            nonlocal status_number
            result = dict(request(operation, arguments, ttl_ms))
            if operation == "status":
                status_number += 1
                if status_number == 3:
                    result["state"] = "RUNNING"
                    result["motion_allowed"] = False
                    result["active_command_id"] = (
                        "physical-roamer-001"
                    )
                    result["infrared"] = infrared(False)
            return result

        session.request = clear_while_running
        PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        actions = [
            call[1]["action"] for call in session.drive_calls()
        ]
        self.assertEqual(
            actions[:2],
            [ACTION_TURN_LEFT, ACTION_TURN_RIGHT],
        )

    def test_all_hard_budgets_are_enforced(self):
        clock = FakeClock()
        session = FakeSession(clock, obstacles=(False,))

        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertLessEqual(result.pulses, MAX_PULSES)
        self.assertLessEqual(
            result.commanded_motion_ms,
            MAX_COMMANDED_MOTION_MS,
        )
        self.assertEqual(
            result.commanded_motion_ms,
            result.pulses * PULSE_ACCOUNTING_MS,
        )
        self.assertLessEqual(clock.monotonic(), MAX_EPISODE_SECONDS + 1.0)

    def test_episode_timer_starts_only_after_successful_arm(self):
        clock = FakeClock()
        session = FakeSession(clock)
        request = session.request

        def delayed_onboarding(operation, arguments=None, ttl_ms=500):
            result = request(operation, arguments, ttl_ms)
            if operation == "describe":
                clock.advance(MAX_EPISODE_SECONDS + 5.0)
            return result

        session.request = delayed_onboarding
        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(result.termination, TERMINATION_PULSE_BUDGET)
        self.assertEqual(result.pulses, MAX_PULSES)
        self.assertTrue(session.drive_calls())

    def test_time_budget_stops_episode_without_motion(self):
        clock = FakeClock()
        session = FakeSession(
            clock,
            running_until_time=MAX_EPISODE_SECONDS + 0.3,
            request_cost=0.05,
        )

        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(result.termination, TERMINATION_TIME_BUDGET)
        self.assertEqual(result.pulses, 0)
        self.assertFalse(session.drive_calls())
        self.assertIn("shutdown", session.operations())

    def test_stale_infrared_fails_closed_before_drive(self):
        clock = FakeClock()
        session = FakeSession(clock, stale_status_index=1)

        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(
            result.termination,
            TERMINATION_STALE_OBSERVATION,
        )
        self.assertFalse(session.drive_calls())
        self.assertEqual(
            session.operations()[-1:],
            ["shutdown"],
        )

    def test_infrared_requires_affirmative_bounded_freshness(self):
        invalid_values = (
            {"fresh": False},
            {"fresh": None},
            {"age_ms": MAX_INFRARED_AGE_MS + 1},
            {"age_ms": -1},
            {"age_ms": True},
            {"observed_monotonic_ms": -1},
            {"observed_monotonic_ms": True},
        )
        for invalid in invalid_values:
            with self.subTest(invalid=invalid):
                clock = FakeClock()
                session = FakeSession(clock)
                original_gate = session._gate

                def invalid_gate(index, values=invalid):
                    gate = original_gate(index)
                    gate.update(values)
                    return gate

                session._gate = invalid_gate
                result = PhysicalRoamer(
                    session,
                    monotonic=clock.monotonic,
                    sleep=clock.sleep,
                ).run()

                self.assertEqual(
                    result.termination,
                    TERMINATION_STALE_OBSERVATION,
                )
                self.assertFalse(session.drive_calls())

        clock = FakeClock()
        session = FakeSession(clock)
        original_gate = session._gate

        def missing_freshness(index):
            gate = original_gate(index)
            gate.pop("fresh")
            return gate

        session._gate = missing_freshness
        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()
        self.assertEqual(
            result.termination,
            TERMINATION_STALE_OBSERVATION,
        )
        self.assertFalse(session.drive_calls())

    def test_infrared_reason_must_match_blocked_state(self):
        invalid_pairs = (
            (False, "blocked_hysteresis_hold"),
            (False, "stable_exit_pending"),
            (True, "clear_hysteresis_hold"),
            (True, "stable_entry_pending"),
            (True, "unknown_reason"),
            (True, "unverified_startup"),
            (True, "invalid_sample"),
        )
        for blocked, reason in invalid_pairs:
            with self.subTest(blocked=blocked, reason=reason):
                clock = FakeClock()
                session = FakeSession(clock)

                def invalid_gate(_index, b=blocked, r=reason):
                    return infrared(b, reason=r)

                session._gate = invalid_gate
                result = PhysicalRoamer(
                    session,
                    monotonic=clock.monotonic,
                    sleep=clock.sleep,
                ).run()

                self.assertEqual(
                    result.termination,
                    TERMINATION_STALE_OBSERVATION,
                )
                self.assertFalse(session.drive_calls())

    def test_all_allowlisted_infrared_reason_pairs_can_operate(self):
        valid_pairs = (
            (True, "warming_up"),
            (True, "immediate_strong_return"),
            (True, "stable_filtered_near_returns"),
            (True, "stable_exit_pending"),
            (True, "blocked_hysteresis_hold"),
            (False, "stable_filtered_release_returns"),
            (False, "stable_entry_pending"),
            (False, "clear_hysteresis_hold"),
        )
        for blocked, reason in valid_pairs:
            with self.subTest(blocked=blocked, reason=reason):
                clock = FakeClock()
                session = FakeSession(clock)

                def valid_gate(_index, b=blocked, r=reason):
                    return infrared(b, reason=r)

                session._gate = valid_gate
                result = PhysicalRoamer(
                    session,
                    monotonic=clock.monotonic,
                    sleep=clock.sleep,
                ).run()

                self.assertGreater(result.pulses, 0)
                self.assertTrue(session.drive_calls())

    def test_remote_fault_fails_closed_before_drive(self):
        clock = FakeClock()
        session = FakeSession(clock, fault_status_index=1)

        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(result.termination, TERMINATION_SAFETY_FAULT)
        self.assertFalse(session.drive_calls())
        self.assertEqual(
            session.operations()[-1:],
            ["shutdown"],
        )

    def test_link_failure_attempts_urgent_cleanup(self):
        clock = FakeClock()
        session = FakeSession(clock, fail_operation="drive_pulse")

        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(
            result.termination,
            TERMINATION_REMOTE_FAILURE,
        )
        self.assertIn("shutdown", session.operations())
        self.assertNotIn("release", session.operations())
        self.assertNotIn("stop", session.operations())
        self.assertEqual(session.operations().count("shutdown"), 1)
        self.assertEqual(session.wait_closed_timeouts, [3.0])
        self.assertTrue(result.stopped_cleanly)

    def test_accepted_pulse_is_accounted_if_post_ack_heartbeat_fails(self):
        clock = FakeClock()
        session = FakeSession(clock)
        request = session.request
        heartbeat_count = 0

        def fail_fifth_heartbeat(operation, arguments=None, ttl_ms=500):
            nonlocal heartbeat_count
            result = request(operation, arguments, ttl_ms)
            if operation == "heartbeat":
                heartbeat_count += 1
                if heartbeat_count == 5:
                    raise OSError("heartbeat acknowledgement lost")
            return result

        session.request = fail_fifth_heartbeat
        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(
            result.termination,
            TERMINATION_REMOTE_FAILURE,
        )
        self.assertEqual(len(session.drive_calls()), 1)
        self.assertEqual(result.pulses, 1)
        self.assertEqual(
            result.commanded_motion_ms,
            PULSE_ACCOUNTING_MS,
        )
        self.assertIn("shutdown", session.operations())

    def test_measured_slow_request_costs_still_operate_safely(self):
        self.assertEqual(HEARTBEAT_INTERVAL_SECONDS, 0.200)
        self.assertEqual(REQUEST_TTL_MS, 500)
        for request_cost in (0.18, 0.20):
            with self.subTest(request_cost=request_cost):
                clock = FakeClock()
                session = FakeSession(
                    clock,
                    request_cost=request_cost,
                )

                result = PhysicalRoamer(
                    session,
                    monotonic=clock.monotonic,
                    sleep=clock.sleep,
                ).run()

                self.assertGreater(result.pulses, 0)
                self.assertTrue(session.drive_calls())
                self.assertEqual(
                    result.termination,
                    TERMINATION_TIME_BUDGET,
                )
                heartbeat_times = [
                    call[3]
                    for call in session.calls
                    if call[0] == "heartbeat"
                ]
                self.assertLessEqual(
                    max(
                        right - left
                        for left, right in zip(
                            heartbeat_times,
                            heartbeat_times[1:],
                        )
                    ),
                    MAX_HEARTBEAT_GAP_SECONDS,
                )
                self.assertTrue(
                    all(
                        call[2] == REQUEST_TTL_MS
                        for call in session.calls
                        if call[0] != "shutdown"
                    )
                )

    def test_request_cost_above_host_heartbeat_limit_fails_closed(self):
        clock = FakeClock()
        session = FakeSession(clock, request_cost=0.46)

        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(
            result.termination,
            TERMINATION_HEARTBEAT_MISSED,
        )
        self.assertFalse(session.drive_calls())
        self.assertEqual(
            session.operations()[-1:],
            ["shutdown"],
        )

    def test_every_heartbeat_ack_must_affirm_exact_remote_timeout(self):
        invalid_timeouts = (None, True, 499, 501, "500")
        for invalid_timeout in invalid_timeouts:
            with self.subTest(invalid_timeout=invalid_timeout):
                clock = FakeClock()
                session = FakeSession(clock)
                request = session.request

                def invalid_heartbeat(
                    operation,
                    arguments=None,
                    ttl_ms=500,
                    value=invalid_timeout,
                ):
                    result = dict(
                        request(operation, arguments, ttl_ms)
                    )
                    if operation == "heartbeat":
                        if value is None:
                            result.pop("heartbeat_timeout_ms")
                        else:
                            result["heartbeat_timeout_ms"] = value
                    return result

                session.request = invalid_heartbeat
                result = PhysicalRoamer(
                    session,
                    monotonic=clock.monotonic,
                    sleep=clock.sleep,
                ).run()

                self.assertEqual(
                    result.termination,
                    TERMINATION_REMOTE_FAILURE,
                )
                self.assertFalse(session.drive_calls())
                self.assertIn("shutdown", session.operations())

    def test_malformed_success_responses_never_authorize_motion(self):
        mutations = ("heartbeat", "status")
        for target in mutations:
            with self.subTest(target=target):
                clock = FakeClock()
                session = FakeSession(clock)
                request = session.request

                def malformed(
                    operation,
                    arguments=None,
                    ttl_ms=500,
                    selected=target,
                ):
                    result = request(operation, arguments, ttl_ms)
                    result = dict(result)
                    if operation == selected:
                        if selected == "heartbeat":
                            result["sequence_id"] += 1
                        else:
                            result.pop("heartbeat_age_ms", None)
                    return result

                session.request = malformed
                result = PhysicalRoamer(
                    session,
                    monotonic=clock.monotonic,
                    sleep=clock.sleep,
                ).run()

                self.assertFalse(session.drive_calls())
                self.assertIn(
                    result.termination,
                    {
                        TERMINATION_REMOTE_FAILURE,
                        TERMINATION_HEARTBEAT_MISSED,
                    },
                )

        clock = FakeClock()
        session = FakeSession(clock)
        request = session.request

        def malformed_drive(operation, arguments=None, ttl_ms=500):
            result = dict(request(operation, arguments, ttl_ms))
            if operation == "drive_pulse":
                result["active_command_id"] = "wrong-command"
            return result

        session.request = malformed_drive
        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()
        self.assertEqual(len(session.drive_calls()), 1)
        self.assertEqual(result.pulses, 1)
        self.assertEqual(
            result.commanded_motion_ms,
            PULSE_ACCOUNTING_MS,
        )
        self.assertEqual(
            result.termination,
            TERMINATION_REMOTE_FAILURE,
        )

    def test_arm_must_affirm_exact_armed_idle_state(self):
        clock = FakeClock()
        session = FakeSession(clock)
        request = session.request

        def running_arm(operation, arguments=None, ttl_ms=500):
            result = dict(request(operation, arguments, ttl_ms))
            if operation == "arm":
                result["state"] = "RUNNING"
                result["motion_allowed"] = False
                result["active_command_id"] = "unexpected-command"
            return result

        session.request = running_arm
        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(result.termination, TERMINATION_SAFETY_FAULT)
        self.assertFalse(session.drive_calls())
        self.assertEqual(
            session.operations()[-1:],
            ["shutdown"],
        )

    def test_cleanup_requires_session_and_active_command_proof(self):
        clock = FakeClock()
        session = FakeSession(clock)
        request = session.request

        def malformed_cleanup(operation, arguments=None, ttl_ms=500):
            result = dict(request(operation, arguments, ttl_ms))
            if operation == "shutdown":
                result.pop("session_active", None)
                result.pop("active_command_id", None)
            return result

        session.request = malformed_cleanup
        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(
            result.termination,
            TERMINATION_CLEANUP_FAILED,
        )
        self.assertFalse(result.stopped_cleanly)

    def test_malformed_status_uses_one_urgent_shutdown(self):
        clock = FakeClock()
        session = FakeSession(clock)
        request = session.request

        def malformed_status(operation, arguments=None, ttl_ms=500):
            result = request(operation, arguments, ttl_ms)
            if operation == "status":
                result = dict(result)
                result.pop("heartbeat_age_ms", None)
            return result

        session.request = malformed_status
        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(
            result.termination,
            TERMINATION_HEARTBEAT_MISSED,
        )
        self.assertFalse(session.drive_calls())
        self.assertEqual(session.operations().count("shutdown"), 1)
        self.assertNotIn("release", session.operations())
        self.assertNotIn("stop", session.operations())
        self.assertEqual(session.wait_closed_timeouts, [3.0])
        self.assertTrue(result.stopped_cleanly)

    def test_successful_close_cannot_upgrade_malformed_shutdown_proof(
        self,
    ):
        clock = FakeClock()
        session = FakeSession(clock)
        request = session.request
        close_calls = []

        def malformed_shutdown(operation, arguments=None, ttl_ms=500):
            result = request(operation, arguments, ttl_ms)
            if operation == "shutdown":
                result = dict(result)
                result["session_active"] = True
            return result

        def successful_close():
            close_calls.append(True)

        session.request = malformed_shutdown
        session.close = successful_close
        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertEqual(
            result.termination,
            TERMINATION_CLEANUP_FAILED,
        )
        self.assertEqual(session.operations().count("shutdown"), 1)
        self.assertEqual(session.wait_closed_timeouts, [3.0])
        self.assertEqual(close_calls, [True])
        self.assertFalse(result.stopped_cleanly)
        close_outcome = next(
            outcome
            for outcome in result.cleanup
            if outcome.operation == "close"
        )
        self.assertTrue(close_outcome.succeeded)

    def test_cleanup_failures_always_promote_terminal_result(self):
        for failed_operation in ("shutdown",):
            with self.subTest(failed_operation=failed_operation):
                clock = FakeClock()
                session = FakeSession(
                    clock,
                    fail_operation=failed_operation,
                )

                result = PhysicalRoamer(
                    session,
                    monotonic=clock.monotonic,
                    sleep=clock.sleep,
                ).run()

                self.assertEqual(
                    result.termination,
                    TERMINATION_CLEANUP_FAILED,
                )

        for return_code in (None, True, 1):
            with self.subTest(wait_closed_result=return_code):
                clock = FakeClock()
                session = FakeSession(
                    clock,
                    wait_closed_result=return_code,
                )

                result = PhysicalRoamer(
                    session,
                    monotonic=clock.monotonic,
                    sleep=clock.sleep,
                ).run()

                self.assertEqual(
                    result.termination,
                    TERMINATION_CLEANUP_FAILED,
                )
                self.assertEqual(
                    session.wait_closed_timeouts,
                    [3.0],
                )
                self.assertFalse(result.stopped_cleanly)

        clock = FakeClock()
        session = FakeSession(clock)

        def failed_close():
            raise OSError("simulated close failure")

        session.close = failed_close
        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()
        self.assertEqual(
            result.termination,
            TERMINATION_CLEANUP_FAILED,
        )
        close_outcome = next(
            outcome
            for outcome in result.cleanup
            if outcome.operation == "close"
        )
        self.assertTrue(close_outcome.attempted)
        self.assertFalse(close_outcome.succeeded)

    def test_successful_cleanup_requires_zero_wait_closed_at_three_seconds(
        self,
    ):
        clock = FakeClock()
        session = FakeSession(clock)

        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        self.assertNotEqual(
            result.termination,
            TERMINATION_CLEANUP_FAILED,
        )
        self.assertEqual(session.operations().count("shutdown"), 1)
        self.assertNotIn("release", session.operations())
        self.assertNotIn("stop", session.operations())
        self.assertEqual(session.wait_closed_timeouts, [3.0])
        wait_outcome = next(
            outcome
            for outcome in result.cleanup
            if outcome.operation == "wait_closed"
        )
        self.assertTrue(wait_outcome.mandatory)
        self.assertTrue(wait_outcome.succeeded)

    def test_cancellation_prevents_next_drive_and_stops(self):
        clock = FakeClock()
        cancelled = threading.Event()
        session = FakeSession(
            clock,
            cancel_on_status_index=1,
            cancel_event=cancelled,
        )

        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
            cancel_event=cancelled,
        ).run()

        self.assertEqual(result.termination, TERMINATION_CANCELLED)
        self.assertFalse(session.drive_calls())
        self.assertEqual(
            session.operations()[-1:],
            ["shutdown"],
        )
        self.assertEqual(session.operations().count("shutdown"), 1)
        self.assertNotIn("release", session.operations())
        self.assertNotIn("stop", session.operations())
        self.assertEqual(session.wait_closed_timeouts, [3.0])
        self.assertTrue(result.stopped_cleanly)

    def test_slow_expression_never_stalls_heartbeat_or_motion(self):
        clock = FakeClock()
        started = threading.Event()
        release = threading.Event()

        def expression(_event):
            started.set()
            release.wait(2.0)

        session = FakeSession(
            clock,
            obstacles=(False, True),
            real_drive_yield=0.001,
        )
        before = time.monotonic()
        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
            expression_submit=expression,
        ).run()
        real_elapsed = time.monotonic() - before

        try:
            self.assertTrue(started.wait(0.5))
            self.assertLess(real_elapsed, 0.5)
            self.assertEqual(result.pulses, MAX_PULSES)
            heartbeat_times = [
                call[3]
                for call in session.calls
                if call[0] == "heartbeat"
            ]
            gaps = [
                right - left
                for left, right in zip(
                    heartbeat_times,
                    heartbeat_times[1:],
                )
            ]
            self.assertTrue(gaps)
            self.assertLessEqual(
                max(gaps),
                MAX_HEARTBEAT_GAP_SECONDS,
            )
        finally:
            release.set()

    def test_expression_events_have_bounded_episode_scoped_ttl(self):
        clock = FakeClock()
        events = []
        received_two = threading.Event()

        def expression(event):
            events.append(dict(event))
            if len(events) >= 2:
                received_two.set()

        session = FakeSession(
            clock,
            obstacles=(False, True),
            real_drive_yield=0.002,
        )
        PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
            expression_submit=expression,
        ).run()

        self.assertTrue(received_two.wait(0.5))
        episode_ids = {event["episode_id"] for event in events}
        self.assertEqual(len(episode_ids), 1)
        episode_id = next(iter(episode_ids))
        self.assertTrue(episode_id.startswith("physical-roamer-"))
        for event in events:
            observed = event["observed_monotonic_ms"]
            valid_until = event["valid_until_monotonic_ms"]
            self.assertIs(type(observed), int)
            self.assertIs(type(valid_until), int)
            self.assertEqual(
                valid_until - observed,
                EXPRESSION_EVENT_TTL_MS,
            )
            self.assertEqual(
                set(event),
                {
                    "schema",
                    "episode_id",
                    "observed_monotonic_ms",
                    "valid_until_monotonic_ms",
                    "obstacle",
                    "reason",
                    "action",
                },
            )
        documentation = physical_roamer_module.__doc__
        self.assertIn("separate transport/process", documentation)
        self.assertIn("never call or share", documentation)
        self.assertIn("supervisor session", documentation)

    def test_drive_payload_is_semantic_and_has_no_motor_numbers(self):
        clock = FakeClock()
        session = FakeSession(clock, obstacles=(False, True))

        result = PhysicalRoamer(
            session,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run()

        forbidden_keys = {
            "left_speed_dps",
            "right_speed_dps",
            "duration_ms",
            "motor",
            "distance_mm",
            "heading",
            "x",
            "y",
            "map",
        }
        allowed_keys = {
            "session_id",
            "sequence_id",
            "command_id",
            "reference_heartbeat_sequence",
            "action",
        }
        self.assertTrue(session.drive_calls())
        for _operation, payload, _ttl_ms, _called_at in (
            session.drive_calls()
        ):
            self.assertEqual(set(payload), allowed_keys)
            self.assertTrue(forbidden_keys.isdisjoint(payload))
            self.assertIn(
                payload["action"],
                {
                    ACTION_ADVANCE,
                    ACTION_TURN_LEFT,
                    ACTION_TURN_RIGHT,
                },
            )
            for key, value in payload.items():
                if key in (
                    "sequence_id",
                    "reference_heartbeat_sequence",
                ):
                    continue
                self.assertNotIsInstance(value, (int, float))

        self.assertFalse(
            any(
                field in result.__dict__
                for field in (
                    "x",
                    "y",
                    "heading",
                    "map",
                    "distance_mm",
                )
            )
        )


if __name__ == "__main__":
    unittest.main()
