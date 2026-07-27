import ast
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from robot_agent import concurrent_demo
from robot_agent.concurrent_demo import (
    DeterministicGrumpyExpressionPlanner,
    run_concurrent_demo,
)
from robot_agent.interaction_contract import (
    InteractionSnapshot,
    ObjectEvidence,
)
from robot_agent.navigation_contract import NavigationContractError


def blocked_snapshot(
    locale="sv-SE",
    object_id="demo-box",
):
    evidence = ObjectEvidence(
        evidence_id="evidence-1",
        relation="BLOCKING_PATH",
        object_id=object_id,
        source="simulator_metric",
        observed_at_ms=100,
        confidence_milli=1_000,
    )
    return InteractionSnapshot(
        robot_id="robot-1",
        controller_instance_id="controller-1",
        goal_id="goal-1",
        goal_epoch=1,
        plan_revision=1,
        interaction_state_version=2,
        world_model_version=3,
        captured_at_ms=100,
        obstruction_epoch=1,
        drive_phase="BLOCKED",
        response_locale=locale,
        evidence=evidence,
    )


class DeterministicPlannerTests(unittest.TestCase):
    def test_known_object_produces_exact_typed_wave(self):
        planner = DeterministicGrumpyExpressionPlanner("sv-SE")

        first = planner(blocked_snapshot())
        second = planner(blocked_snapshot())

        self.assertEqual(first, second)
        first.assert_matches_snapshot(blocked_snapshot())
        self.assertEqual(first.decision, "EXPRESS")
        self.assertEqual(first.intent.utterance, "Grrr! demo-box!")
        self.assertEqual(first.intent.utterance_locale, "sv-SE")
        self.assertEqual(first.intent.gesture_kind, "PROPELLER_WAVE")
        self.assertEqual(first.intent.repetitions, 1)

    def test_unknown_object_is_speech_only_without_identity_claim(self):
        snapshot = blocked_snapshot(object_id=None)
        proposal = DeterministicGrumpyExpressionPlanner("en-US")(
            InteractionSnapshot(
                **{
                    **snapshot.__dict__,
                    "response_locale": "en-US",
                }
            )
        )

        proposal.assert_matches_snapshot(
            InteractionSnapshot(
                **{
                    **snapshot.__dict__,
                    "response_locale": "en-US",
                }
            )
        )
        self.assertEqual(proposal.intent.utterance, "Grrr!")
        self.assertIsNone(proposal.intent.gesture_kind)
        self.assertEqual(proposal.intent.repetitions, 0)


class ConcurrentDemoTests(unittest.TestCase):
    def test_lm_studio_mode_binds_the_same_host_locale(self):
        planner, mode, model = concurrent_demo._build_planner(
            True,
            "en-US",
            concurrent_demo.DEFAULT_BASE_URL,
            concurrent_demo.DEFAULT_MODEL,
            1.0,
        )

        self.assertEqual(mode, "lm_studio_structured_output")
        self.assertEqual(model, concurrent_demo.DEFAULT_MODEL)
        self.assertEqual(planner.response_locale, "en-US")

    def test_default_run_is_simulator_only_and_bounded(self):
        outcome = run_concurrent_demo(tick_ms=2)
        report = outcome.to_report(event_limit=48)

        self.assertEqual(report["schema"], concurrent_demo.REPORT_SCHEMA)
        self.assertEqual(report["mode"], "simulation_only")
        self.assertFalse(report["hardware_used"])
        self.assertTrue(report["navigation"]["completed"])
        self.assertTrue(
            report["navigation"]["terminal_stop_verified"]
        )
        self.assertEqual(
            report["planner"]["mode"],
            "deterministic_typed_fixture",
        )
        metrics = report["concurrency"]["metrics"]
        self.assertGreaterEqual(metrics["expressions_accepted"], 1)
        self.assertGreaterEqual(metrics["speech_started"], 1)
        self.assertGreaterEqual(metrics["gestures_started"], 1)
        self.assertEqual(report["concurrency"]["workers_alive"], [])
        self.assertFalse(
            report["concurrency"]["wheel_and_arm_overlap_allowed"]
        )
        self.assertLessEqual(len(report["event_order"]), 48)
        sequences = [
            event["sequence"] for event in report["event_order"]
        ]
        self.assertEqual(sequences, sorted(sequences))
        callback_kinds = {
            callback["kind"]
            for callback in report["virtual_callbacks"]
        }
        self.assertIn("virtual_speech_started", callback_kinds)
        self.assertIn("virtual_arm_segment", callback_kinds)

    def test_main_emits_json_and_uses_navigation_exit_status(self):
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            status = concurrent_demo.main(
                [
                    "--compact",
                    "--tick-ms",
                    "2",
                    "--event-limit",
                    "12",
                ]
            )

        self.assertEqual(status, 0)
        report = json.loads(stdout.getvalue())
        self.assertTrue(report["navigation"]["completed"])
        self.assertLessEqual(len(report["event_order"]), 12)

    def test_clear_scenario_does_not_invent_an_interaction(self):
        report = run_concurrent_demo(
            scenario="clear",
            tick_ms=1,
        ).to_report(20)

        metrics = report["concurrency"]["metrics"]
        self.assertEqual(metrics["planner_requests"], 0)
        self.assertEqual(metrics["expressions_accepted"], 0)
        self.assertEqual(report["virtual_callbacks"], [])


class StrictCliAndIsolationTests(unittest.TestCase):
    def test_invalid_tick_and_model_options_are_rejected(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as tick_error:
                concurrent_demo.main(["--tick-ms", "0"])
            with self.assertRaises(SystemExit) as model_error:
                concurrent_demo.main(["--model", "some/model"])

        self.assertEqual(tick_error.exception.code, 2)
        self.assertEqual(model_error.exception.code, 2)

    def test_config_is_parsed_by_the_strict_existing_loader(self):
        duplicate_config = (
            b'{"schema":"robot-navigation-simulation/v1",'
            b'"schema":"robot-navigation-simulation/v1"}'
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.json"
            path.write_bytes(duplicate_config)
            with self.assertRaises(NavigationContractError):
                run_concurrent_demo(config_path=path)

    def test_module_has_no_physical_transport_imports(self):
        tree = ast.parse(
            Path(concurrent_demo.__file__).read_text(encoding="utf-8")
        )
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imported.add(node.module or "")

        forbidden = ("robot_api", "supervisor_transport", "ev3")
        self.assertFalse(
            any(
                module == name
                or module.endswith("." + name)
                or module.startswith(name + ".")
                for module in imported
                for name in forbidden
            )
        )


if __name__ == "__main__":
    unittest.main()
