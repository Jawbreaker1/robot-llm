import contextlib
import io
import json
import os
import subprocess
import sys
import unittest
from unittest import mock

from robot_agent.research_cli import main, run_research_query
from robot_agent.research_loop import (
    ANSWERED,
    BUDGET_EXHAUSTED,
    CLARIFICATION_REQUIRED,
    ResearchEpisodeResult,
)


def episode(termination, answer=None, clarification=None):
    return ResearchEpisodeResult(
        turn_id="turn-1",
        completed=termination == ANSWERED,
        termination=termination,
        answer_text=answer,
        clarification_question=clarification,
        citation_ids=(
            ("evidence-1",)
            if termination == ANSWERED
            else ()
        ),
        planner_turns=2,
        tool_calls=1,
        replans=1,
        final_context_version=2,
        evidence=(),
        trace=("CREATED", termination),
    )


class NeverCalledWeatherTool:
    def current(self, _request):
        raise AssertionError("clarification must not call weather")


class ResearchCLITests(unittest.TestCase):
    def test_cold_research_import_does_not_load_execution_stack(self):
        forbidden = (
            "robot_agent.agent_loop",
            "robot_agent.contract",
            "robot_agent.robot_api",
            "robot_agent.safety",
            "robot_agent.shadow_commentary",
            "robot_agent.simulated_robot",
            "robot_agent.supervisor_transport",
        )
        program = (
            "import json,sys;"
            "import robot_agent.research_cli;"
            "forbidden={};"
            "print(json.dumps([name for name in forbidden "
            "if name in sys.modules]))"
        ).format(repr(forbidden))
        environment = dict(os.environ)
        environment["PYTHONDONTWRITEBYTECODE"] = "1"

        completed = subprocess.run(
            [sys.executable, "-c", program],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
            env=environment,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertEqual(json.loads(completed.stdout), [])

    def test_run_query_always_requires_evidence_and_has_no_motion_path(self):
        captured = []

        def planner(context):
            captured.append(context)
            return json.dumps(
                {
                    "schema": "research-decision/v1",
                    "proposal_id": "proposal-1",
                    "turn_id": "turn-1",
                    "based_on_context_version": 1,
                    "decision": "CLARIFY",
                    "question": "Vilken Stockholm menar du?",
                }
            ).encode("utf-8")

        result = run_research_query(
            query="Hur är vädret?",
            turn_id="turn-1",
            planner=planner,
            weather_tool=NeverCalledWeatherTool(),
        )

        self.assertEqual(result.termination, CLARIFICATION_REQUIRED)
        self.assertTrue(captured[0].require_evidence)
        self.assertEqual(
            captured[0].available_tools,
            ("weather.current",),
        )

    @mock.patch("robot_agent.research_cli.OpenMeteoWeatherTool")
    @mock.patch("robot_agent.research_cli.LMStudioResearchPlanner")
    @mock.patch("robot_agent.research_cli.run_research_query")
    def test_main_prints_answered_audit_json(
        self,
        run_query,
        planner_class,
        weather_class,
    ):
        run_query.return_value = episode(
            ANSWERED,
            answer="Ingen nederbörd just nu.",
        )
        stdout = io.StringIO()

        with contextlib.redirect_stdout(stdout):
            status = main(
                [
                    "Behöver jag paraply i Stockholm?",
                    "--turn-id",
                    "turn-1",
                    "--lm-studio-url",
                    "http://127.0.0.1:1234",
                    "--model",
                    "google/gemma-4-26b-a4b",
                ]
            )

        self.assertEqual(status, 0)
        report = json.loads(stdout.getvalue())
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["mode"], "read_only_research")
        self.assertEqual(report["citation_ids"], ["evidence-1"])
        planner_class.assert_called_once_with(
            base_url="http://127.0.0.1:1234",
            model="google/gemma-4-26b-a4b",
            timeout_seconds=10.0,
        )
        run_query.assert_called_once_with(
            query="Behöver jag paraply i Stockholm?",
            turn_id="turn-1",
            planner=planner_class.return_value,
            weather_tool=weather_class.return_value,
        )

    @mock.patch("robot_agent.research_cli.OpenMeteoWeatherTool")
    @mock.patch("robot_agent.research_cli.LMStudioResearchPlanner")
    @mock.patch("robot_agent.research_cli.run_research_query")
    def test_main_uses_distinct_exit_codes(
        self,
        run_query,
        _planner_class,
        _weather_class,
    ):
        cases = (
            (
                episode(
                    CLARIFICATION_REQUIRED,
                    clarification="Vilken plats?",
                ),
                4,
                "clarification_required",
            ),
            (
                episode(BUDGET_EXHAUSTED),
                3,
                "failed",
            ),
        )
        for result, expected_status, expected_label in cases:
            with self.subTest(result=result.termination):
                run_query.return_value = result
                stdout = io.StringIO()
                with contextlib.redirect_stdout(stdout):
                    status = main(["Väder?", "--turn-id", "turn-1"])
                self.assertEqual(status, expected_status)
                self.assertEqual(
                    json.loads(stdout.getvalue())["status"],
                    expected_label,
                )

    @mock.patch(
        "robot_agent.research_cli.LMStudioResearchPlanner",
        side_effect=ValueError("unexpected"),
    )
    def test_unexpected_programming_error_is_not_hidden(self, _planner):
        with self.assertRaisesRegex(ValueError, "unexpected"):
            main(["Väder?", "--turn-id", "turn-1"])


if __name__ == "__main__":
    unittest.main()
