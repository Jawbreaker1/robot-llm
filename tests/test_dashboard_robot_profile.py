import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from robot_agent.dashboard_cli import (
    ROBOT_PROFILE_DISABLED,
    _configured_robot_runtime_adapter,
    _parser,
    _run,
)
from robot_agent.ev3rstorm_profile import EV3RSTORM_PROFILE_ID, EV3SSHBinding


class DashboardRobotProfileTests(unittest.TestCase):
    def test_default_profile_remains_disabled(self):
        args = _parser().parse_args([])

        self.assertEqual(args.robot_profile, ROBOT_PROFILE_DISABLED)
        self.assertIsNone(args.robot_target)
        self.assertEqual(args.robot_planner_timeout_seconds, 30.0)
        self.assertEqual(args.robot_input_timeout_seconds, 10.0)
        self.assertIsNone(_configured_robot_runtime_adapter(args))

    def test_ev3_profile_requires_an_explicit_target(self):
        args = _parser().parse_args(
            ["--robot-profile", EV3RSTORM_PROFILE_ID]
        )

        with self.assertRaisesRegex(ValueError, "--robot-target is required"):
            _configured_robot_runtime_adapter(args)

    def test_ev3_profile_builds_binding_and_deferred_planner_factory(self):
        with tempfile.TemporaryDirectory() as directory:
            args = _parser().parse_args(
                [
                    "--robot-profile",
                    EV3RSTORM_PROFILE_ID,
                    "--robot-target",
                    "robot@ev3dev.local",
                    "--robot-memory-path",
                    str(Path(directory) / "memory.json"),
                    "--robot-reset-memory",
                    "--robot-planner-timeout-seconds",
                    "7.5",
                ]
            )
            built_adapter = object()
            profile = mock.Mock()
            profile.descriptor = SimpleNamespace(
                profile_id=EV3RSTORM_PROFILE_ID
            )
            profile.build_adapter.return_value = built_adapter
            planner = object()
            with (
                mock.patch(
                    "robot_agent.dashboard_cli.EV3RSTORMProfile",
                    return_value=profile,
                ),
                mock.patch(
                    "robot_agent.dashboard_cli.LMStudioNavigationPlanner",
                    return_value=planner,
                ) as planner_type,
            ):
                result = _configured_robot_runtime_adapter(args)

                self.assertIs(result, built_adapter)
                binding = profile.build_adapter.call_args.args[0]
                self.assertIsInstance(binding, EV3SSHBinding)
                self.assertEqual(binding.target, "robot@ev3dev.local")
                self.assertTrue(binding.reset_memory)
                planner_factory = profile.build_adapter.call_args.kwargs[
                    "planner_factory"
                ]
                planner_type.assert_not_called()
                self.assertIs(planner_factory("model-a"), planner)
                planner_type.assert_called_once_with(
                    base_url=args.lm_studio_url,
                    model="model-a",
                    timeout_seconds=7.5,
                )

    def test_run_injects_selected_profile_without_starting_adapter(self):
        adapter = mock.Mock()
        dashboard_service = mock.Mock()
        control_service = mock.Mock()
        server = mock.Mock()
        router = mock.Mock(session_path="/session/token/")
        with (
            mock.patch(
                "robot_agent.dashboard_cli."
                "_configured_robot_runtime_adapter",
                return_value=adapter,
            ) as configured,
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=dashboard_service,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.RobotControlService",
                return_value=control_service,
            ) as control_type,
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(server, router),
            ),
            mock.patch("sys.stdout", new_callable=io.StringIO) as stdout,
        ):
            result = _run(
                [
                    "--robot-profile",
                    EV3RSTORM_PROFILE_ID,
                    "--robot-target",
                    "robot@ev3dev.local",
                ]
            )

        self.assertEqual(result, 0)
        configured.assert_called_once()
        self.assertIs(control_type.call_args.args[0], adapter)
        adapter.run.assert_not_called()
        ready = json.loads(stdout.getvalue())
        self.assertTrue(ready["physical_control_enabled"])
        self.assertEqual(ready["robot_profile"], EV3RSTORM_PROFILE_ID)

    def test_run_wires_explicit_physical_map_provider(self):
        adapter = mock.Mock()
        map_provider = mock.Mock()
        map_provider.snapshot = mock.Mock(return_value={})
        map_provider.close = mock.Mock(return_value=True)
        adapter.spatial_map_provider = map_provider
        dashboard_service = mock.Mock()
        control_service = mock.Mock()
        server = mock.Mock()
        router = mock.Mock(session_path="/session/token/")
        with (
            mock.patch(
                "robot_agent.dashboard_cli."
                "_configured_robot_runtime_adapter",
                return_value=adapter,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=dashboard_service,
            ) as dashboard_type,
            mock.patch(
                "robot_agent.dashboard_cli.RobotControlService",
                return_value=control_service,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(server, router),
            ),
            mock.patch("sys.stdout", new_callable=io.StringIO) as stdout,
        ):
            result = _run(
                [
                    "--robot-profile",
                    EV3RSTORM_PROFILE_ID,
                    "--robot-target",
                    "robot@ev3dev.local",
                ]
            )

        self.assertEqual(result, 0)
        self.assertIs(
            dashboard_type.call_args.kwargs["spatial_map_provider"],
            map_provider,
        )
        self.assertEqual(
            json.loads(stdout.getvalue())["spatial_map_mode"],
            "physical_live",
        )
        map_provider.close.assert_called_once_with(drain=True)

    def test_run_keeps_interactive_timeout_independent_of_planner(self):
        adapter = mock.Mock()
        dashboard_service = mock.Mock()
        control_service = mock.Mock()
        server = mock.Mock()
        router = mock.Mock(session_path="/session/token/")
        input_model = object()
        with (
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=dashboard_service,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.RobotControlService",
                return_value=control_service,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.LMStudioRobotInputModel",
                return_value=input_model,
            ) as input_model_type,
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(server, router),
            ) as build_server,
            mock.patch("sys.stdout", new_callable=io.StringIO),
        ):
            result = _run(
                [
                    "--robot-planner-timeout-seconds",
                    "60",
                    "--robot-input-timeout-seconds",
                    "8",
                ],
                robot_runtime_adapter=adapter,
            )

            input_service = build_server.call_args.kwargs[
                "robot_input_service"
            ]
            self.assertIs(
                input_service._model_factory("model-a"),
                input_model,
            )

        self.assertEqual(result, 0)
        input_model_type.assert_called_once_with(
            base_url="http://127.0.0.1:1234",
            model="model-a",
            timeout_seconds=8.0,
        )

    def test_simulation_map_cannot_be_mixed_with_physical_runtime(self):
        output = io.StringIO()
        with mock.patch("sys.stderr", output):
            result = _run(
                ["--simulation-map-demo"],
                robot_runtime_adapter=mock.Mock(),
            )

        self.assertEqual(result, 2)
        failure = json.loads(output.getvalue())
        self.assertEqual(failure["status"], "failed")
        self.assertIn("cannot be combined", failure["error"])


if __name__ == "__main__":
    unittest.main()
