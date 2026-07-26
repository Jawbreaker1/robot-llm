from dataclasses import replace
import json
from pathlib import Path
import unittest

from robot_agent.agent_loop import (
    BUDGET_EXHAUSTED,
    STOP_FAILED,
    GOAL_SATISFIED,
    PLANNER_ABORTED,
    PLANNER_FAILED,
    PROPOSAL_SCHEMA,
    UNSAFE_OBSERVATION,
    ClosedLoopAgent,
    LoopLimits,
    MotorPositionGoal,
    ProposalError,
    decode_decision_proposal,
)
from robot_agent.robot_api import RobotAPI, SimulatedRobotAPI


CONFIG_PATH = Path(__file__).parents[1] / "config" / "ev3rstorm.json"


class MutableClock:
    def __init__(self, now_ms=10_000):
        self.now_ms = now_ms

    def __call__(self):
        return self.now_ms


def act(context, proposal_id, speed_dps=125, duration_ms=400, **extra):
    value = {
        "schema": PROPOSAL_SCHEMA,
        "proposal_id": proposal_id,
        "goal_id": context.goal.goal_id,
        "based_on_state_version": context.observation.state_version,
        "decision": "ACT",
        "action": {
            "type": "MOVE_MOTOR",
            "motor_role": context.goal.motor_role,
            "speed_dps": speed_dps,
            "duration_ms": duration_ms,
        },
    }
    value.update(extra)
    return json.dumps(value).encode("utf-8")


def abort(context, proposal_id="abort-1"):
    return json.dumps(
        {
            "schema": PROPOSAL_SCHEMA,
            "proposal_id": proposal_id,
            "goal_id": context.goal.goal_id,
            "based_on_state_version": (
                context.observation.state_version
            ),
            "decision": "ABORT",
            "abort_code": "planner_cannot_continue",
        }
    ).encode("utf-8")


class ScriptedPlanner:
    def __init__(self, *steps):
        self.steps = list(steps)
        self.contexts = []

    def __call__(self, context):
        self.contexts.append(context)
        step = self.steps.pop(0)
        return step(context) if callable(step) else step


class RobotAPIProbe(RobotAPI):
    def __init__(
        self,
        delegate,
        raise_on_observe=None,
        stop_failures=0,
        wrong_stop_receipt=False,
        unsafe_terminal=False,
        stale_terminal=False,
        advance_after_execute_ms=0,
        unsafe_stop_observations=0,
    ):
        self.delegate = delegate
        self.raise_on_observe = raise_on_observe
        self.stop_failures = stop_failures
        self.wrong_stop_receipt = wrong_stop_receipt
        self.unsafe_terminal = unsafe_terminal
        self.stale_terminal = stale_terminal
        self.advance_after_execute_ms = advance_after_execute_ms
        self.unsafe_stop_observations = unsafe_stop_observations
        self.observe_calls = 0
        self.execute_calls = 0
        self.stop_calls = 0
        self.stopped = False

    def capabilities(self):
        return self.delegate.capabilities()

    def observe(self):
        self.observe_calls += 1
        if self.observe_calls == self.raise_on_observe:
            raise RuntimeError("unexpected observe failure")
        observation = self.delegate.observe()
        if self.stopped and self.unsafe_stop_observations > 0:
            self.unsafe_stop_observations -= 1
            motors = dict(observation.state.motors)
            motors["arm"] = replace(motors["arm"], running=True)
            observation = replace(
                observation,
                state=replace(observation.state, motors=motors),
            )
        if self.stopped and self.unsafe_terminal:
            sensors = dict(observation.state.sensors)
            sensors["touch"] = True
            observation = replace(
                observation,
                state=replace(observation.state, sensors=sensors),
            )
        if self.stopped and self.stale_terminal:
            observation = replace(
                observation,
                received_at_host_ms=(
                    observation.received_at_host_ms - 501
                ),
            )
        return observation

    def execute_motion(self, request):
        self.execute_calls += 1
        receipt = self.delegate.execute_motion(request)
        self.delegate._clock_ms.now_ms += self.advance_after_execute_ms
        return receipt

    def stop_all(self, request):
        self.stop_calls += 1
        if self.stop_calls <= self.stop_failures:
            raise RuntimeError("unexpected stop failure")
        receipt = self.delegate.stop_all(request)
        self.stopped = True
        if self.wrong_stop_receipt:
            return replace(receipt, action_id="wrong-stop-action")
        return receipt


class AgentLoopTests(unittest.TestCase):
    def setUp(self):
        self.clock = MutableClock()
        self.api = SimulatedRobotAPI.from_config(
            CONFIG_PATH,
            self.clock,
            controller_instance_id="sim-instance-loop",
        )
        self.episode_number = 0

    def agent(self, planner, limits=None, robot=None):
        self.episode_number += 1
        episode_id = "episode-{}".format(self.episode_number)
        return ClosedLoopAgent(
            self.api if robot is None else robot,
            planner,
            self.clock,
            limits=LoopLimits() if limits is None else limits,
            id_factory=lambda: episode_id,
        )

    def goal(self, target=100):
        return MotorPositionGoal(
            goal_id="goal-arm-position",
            instruction="Placera armen vid målets encoderposition.",
            motor_role="arm",
            target_degrees=target,
            tolerance_degrees=2,
        )

    def test_two_actions_replan_and_reach_verified_goal(self):
        planner = ScriptedPlanner(
            lambda context: act(context, "proposal-1"),
            lambda context: act(context, "proposal-2"),
        )

        result = self.agent(planner).run(self.goal())

        self.assertTrue(result.completed)
        self.assertEqual(result.termination, GOAL_SATISFIED)
        self.assertEqual(result.planner_turns, 2)
        self.assertEqual(result.actions, 2)
        self.assertEqual(result.replans, 1)
        self.assertEqual(result.total_motion_ms, 800)
        self.assertEqual(
            result.final_observation.state.motors[
                "arm"
            ].position_degrees,
            100,
        )
        self.assertEqual(
            result.trace,
            (
                "CREATED",
                "OBSERVING",
                "PLANNING",
                "AUTHORIZING",
                "EXECUTING",
                "OBSERVING",
                "VERIFYING",
                "REPLANNING",
                "OBSERVING",
                "PLANNING",
                "AUTHORIZING",
                "EXECUTING",
                "OBSERVING",
                "VERIFYING",
                "STOPPING",
                "SUCCEEDED",
            ),
        )
        self.assertEqual(
            planner.contexts[1].previous_feedback.code,
            "verified_progress",
        )
        json.dumps(result.to_dict())

    def test_initially_satisfied_goal_never_calls_planner(self):
        planner = ScriptedPlanner()
        result = self.agent(planner).run(self.goal(target=0))

        self.assertTrue(result.completed)
        self.assertEqual(result.actions, 0)
        self.assertEqual(result.planner_turns, 0)
        self.assertEqual(planner.contexts, [])

    def test_rejected_speed_is_feedback_then_valid_plan_succeeds(self):
        planner = ScriptedPlanner(
            lambda context: act(
                context,
                "proposal-too-fast",
                speed_dps=151,
            ),
            lambda context: act(
                context,
                "proposal-valid",
                speed_dps=125,
            ),
        )

        result = self.agent(planner).run(self.goal(target=50))

        self.assertTrue(result.completed)
        self.assertEqual(result.actions, 1)
        self.assertEqual(result.replans, 1)
        self.assertEqual(result.steps[0].outcome_code, "speed_limit")
        self.assertEqual(
            planner.contexts[1].previous_feedback.code,
            "speed_limit",
        )

    def test_wrong_motor_and_direction_never_execute(self):
        def wrong_motor(context):
            raw = json.loads(act(context, "wrong-motor"))
            raw["action"]["motor_role"] = "drive_b"
            return json.dumps(raw).encode("utf-8")

        planner = ScriptedPlanner(
            wrong_motor,
            lambda context: act(
                context,
                "wrong-direction",
                speed_dps=-125,
            ),
            abort,
        )
        result = self.agent(planner).run(self.goal(target=50))

        self.assertFalse(result.completed)
        self.assertEqual(result.termination, PLANNER_ABORTED)
        self.assertEqual(result.actions, 0)
        self.assertEqual(
            tuple(step.outcome_code for step in result.steps),
            ("wrong_motor", "wrong_direction"),
        )
        self.assertEqual(
            result.final_observation.state.motors[
                "arm"
            ].position_degrees,
            0,
        )

    def test_duplicate_proposal_id_is_rejected(self):
        planner = ScriptedPlanner(
            lambda context: act(
                context,
                "same-proposal",
                speed_dps=50,
                duration_ms=100,
            ),
            lambda context: act(
                context,
                "same-proposal",
                speed_dps=50,
                duration_ms=100,
            ),
            abort,
        )

        result = self.agent(planner).run(self.goal(target=50))

        self.assertEqual(result.actions, 1)
        self.assertEqual(
            result.steps[1].outcome_code,
            "duplicate_proposal",
        )
        self.assertEqual(result.termination, PLANNER_ABORTED)

    def test_state_change_during_planning_causes_replan(self):
        def mutate_then_propose(context):
            self.api.set_sensor("color_reflected_percent", 10)
            return act(context, "stale-proposal")

        planner = ScriptedPlanner(
            mutate_then_propose,
            lambda context: act(context, "fresh-proposal"),
        )

        result = self.agent(planner).run(self.goal(target=50))

        self.assertTrue(result.completed)
        self.assertEqual(result.actions, 1)
        self.assertEqual(result.steps[0].outcome_code, "stale_state")

    def test_touch_observation_stops_before_planning(self):
        self.api.set_sensor("touch", True)
        planner = ScriptedPlanner()

        result = self.agent(planner).run(self.goal(target=50))

        self.assertEqual(result.termination, UNSAFE_OBSERVATION)
        self.assertEqual(result.actions, 0)
        self.assertEqual(planner.contexts, [])

    def test_planner_abort_is_bounded_and_stopped(self):
        planner = ScriptedPlanner(abort)
        result = self.agent(planner).run(self.goal(target=50))

        self.assertEqual(result.termination, PLANNER_ABORTED)
        self.assertFalse(result.completed)
        self.assertEqual(result.trace[-2:], ("STOPPING", "ABORTED"))

    def test_planner_exception_and_latency_are_fail_closed(self):
        def fails(_context):
            raise RuntimeError("planner exploded")

        failed = self.agent(ScriptedPlanner(fails)).run(
            self.goal(target=50)
        )
        self.assertEqual(failed.termination, PLANNER_FAILED)
        self.assertEqual(failed.actions, 0)

        def slow(context):
            self.clock.now_ms += 1_001
            return act(context, "late-proposal")

        slow_result = self.agent(ScriptedPlanner(slow)).run(
            self.goal(target=50)
        )
        self.assertEqual(slow_result.termination, PLANNER_FAILED)
        self.assertEqual(slow_result.actions, 0)

    def test_motion_and_replan_budgets_are_terminal(self):
        planner = ScriptedPlanner(
            lambda context: act(
                context,
                "over-budget-1",
                duration_ms=400,
            ),
            lambda context: act(
                context,
                "over-budget-2",
                duration_ms=400,
            ),
        )
        limits = LoopLimits(
            max_replans=1,
            max_total_motion_ms=300,
        )

        result = self.agent(planner, limits).run(
            self.goal(target=50)
        )

        self.assertEqual(result.termination, BUDGET_EXHAUSTED)
        self.assertEqual(result.actions, 0)
        self.assertEqual(
            tuple(step.outcome_code for step in result.steps),
            ("motion_budget", "motion_budget"),
        )

    def test_unexpected_post_observe_error_stops_before_rethrow(self):
        robot = RobotAPIProbe(
            self.api,
            raise_on_observe=3,
            unsafe_stop_observations=1,
        )
        planner = ScriptedPlanner(
            lambda context: act(
                context,
                "proposal-before-observe-failure",
            )
        )

        with self.assertRaises(RuntimeError):
            self.agent(planner, robot=robot).run(
                self.goal(target=50)
            )

        self.assertEqual(robot.execute_calls, 1)
        self.assertEqual(robot.stop_calls, 2)
        self.assertGreaterEqual(robot.observe_calls, 5)
        self.assertFalse(
            any(
                motor.running
                for motor in self.api.observe().state.motors.values()
            )
        )

    def test_terminal_stop_retries_and_fails_closed(self):
        transient = RobotAPIProbe(self.api, stop_failures=1)
        recovered = self.agent(
            ScriptedPlanner(),
            robot=transient,
        ).run(self.goal(target=0))
        self.assertTrue(recovered.completed)
        self.assertEqual(transient.stop_calls, 2)

        always_fails = RobotAPIProbe(self.api, stop_failures=2)
        failed = self.agent(
            ScriptedPlanner(),
            robot=always_fails,
        ).run(self.goal(target=0))
        self.assertFalse(failed.completed)
        self.assertEqual(failed.termination, STOP_FAILED)
        self.assertEqual(always_fails.stop_calls, 2)

    def test_terminal_success_requires_safe_fresh_observation(self):
        for robot in (
            RobotAPIProbe(self.api, unsafe_terminal=True),
            RobotAPIProbe(self.api, stale_terminal=True),
        ):
            with self.subTest(robot=robot):
                result = self.agent(
                    ScriptedPlanner(),
                    robot=robot,
                ).run(self.goal(target=0))
                self.assertFalse(result.completed)
                self.assertEqual(result.termination, STOP_FAILED)

    def test_terminal_stop_receipt_must_match(self):
        robot = RobotAPIProbe(
            self.api,
            wrong_stop_receipt=True,
        )

        result = self.agent(
            ScriptedPlanner(),
            robot=robot,
        ).run(self.goal(target=0))

        self.assertFalse(result.completed)
        self.assertEqual(result.termination, STOP_FAILED)
        self.assertEqual(robot.stop_calls, 2)

    def test_global_deadline_is_rechecked_after_planner(self):
        def slow_plan(context):
            self.clock.now_ms += 900
            return act(context, "proposal-after-global-deadline")

        robot = RobotAPIProbe(self.api)
        limits = LoopLimits(
            max_elapsed_ms=1_000,
            max_planner_latency_ms=1_000,
        )
        result = self.agent(
            ScriptedPlanner(slow_plan),
            limits,
            robot,
        ).run(self.goal(target=50))

        self.assertEqual(result.termination, BUDGET_EXHAUSTED)
        self.assertEqual(robot.execute_calls, 0)

    def test_action_duration_must_fit_remaining_elapsed_budget(self):
        def consume_half_budget(context):
            self.clock.now_ms += 500
            return act(
                context,
                "proposal-too-long-for-episode",
                duration_ms=400,
            )

        robot = RobotAPIProbe(self.api)
        limits = LoopLimits(
            max_elapsed_ms=1_000,
            max_planner_latency_ms=900,
        )
        result = self.agent(
            ScriptedPlanner(consume_half_budget),
            limits,
            robot,
        ).run(self.goal(target=50))

        self.assertEqual(result.termination, BUDGET_EXHAUSTED)
        self.assertEqual(robot.execute_calls, 0)

    def test_elapsed_budget_after_execute_cannot_report_success(self):
        def plan(context):
            self.clock.now_ms += 100
            return act(context, "proposal-crosses-deadline")

        robot = RobotAPIProbe(
            self.api,
            advance_after_execute_ms=800,
        )
        limits = LoopLimits(
            max_elapsed_ms=1_000,
            max_planner_latency_ms=900,
        )
        result = self.agent(
            ScriptedPlanner(plan),
            limits,
            robot,
        ).run(self.goal(target=50))

        self.assertFalse(result.completed)
        self.assertEqual(result.termination, BUDGET_EXHAUSTED)
        self.assertEqual(robot.execute_calls, 1)

    def test_replan_counter_never_exceeds_limit(self):
        planner = ScriptedPlanner(
            lambda context: act(
                context,
                "proposal-without-replan-budget",
                speed_dps=151,
            )
        )

        result = self.agent(
            planner,
            LoopLimits(max_replans=0),
        ).run(self.goal(target=50))

        self.assertEqual(result.termination, BUDGET_EXHAUSTED)
        self.assertEqual(result.replans, 0)
        self.assertEqual(result.actions, 0)


class ProposalCodecTests(unittest.TestCase):
    def test_exact_act_and_abort_shapes_decode(self):
        act_value = {
            "schema": PROPOSAL_SCHEMA,
            "proposal_id": "proposal-1",
            "goal_id": "goal-1",
            "based_on_state_version": 1,
            "decision": "ACT",
            "action": {
                "type": "MOVE_MOTOR",
                "motor_role": "arm",
                "speed_dps": 125,
                "duration_ms": 400,
            },
        }
        proposal = decode_decision_proposal(
            json.dumps(act_value).encode("utf-8")
        )
        self.assertEqual(proposal.action.motor_role, "arm")

        act_value.pop("action")
        act_value["decision"] = "ABORT"
        act_value["abort_code"] = "cannot_continue"
        proposal = decode_decision_proposal(
            json.dumps(act_value).encode("utf-8")
        )
        self.assertEqual(proposal.decision, "ABORT")

    def test_duplicate_extra_nonfinite_and_bool_integer_are_rejected(self):
        valid = {
            "schema": PROPOSAL_SCHEMA,
            "proposal_id": "proposal-1",
            "goal_id": "goal-1",
            "based_on_state_version": 1,
            "decision": "ABORT",
            "abort_code": "cannot_continue",
        }
        invalid = (
            (
                b'{"schema":"robot-agent-decision/v1",'
                b'"proposal_id":"a","proposal_id":"b",'
                b'"goal_id":"g","based_on_state_version":1,'
                b'"decision":"ABORT","abort_code":"x"}'
            ),
            json.dumps(dict(valid, extra=True)).encode("utf-8"),
            json.dumps(valid).replace("1", "NaN", 1).encode("utf-8"),
            json.dumps(
                dict(valid, based_on_state_version=True)
            ).encode("utf-8"),
        )
        for raw in invalid:
            with self.subTest(raw=raw):
                with self.assertRaises(ProposalError):
                    decode_decision_proposal(raw)


if __name__ == "__main__":
    unittest.main()
