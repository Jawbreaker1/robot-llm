import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from robot_agent.dashboard_cli import (
    BLAST_MAX_NAVIGATION_UTTERANCE_CHARS,
    ROBOT_PROFILE_DISABLED,
    _configured_robot_control_target,
    _configured_robot_runtime_adapter,
    _configured_shared_spatial_map,
    _parser,
    _run,
)
from robot_agent.blast_episode_adapter import BLAST_PROFILE_ID
from robot_agent.blast_hub_speech import BLAST_PIPER_PROFILE
from robot_agent.blast_personality import BLAST_PERSONA_BY_LOCALE
from robot_agent.ev3rstorm_profile import EV3RSTORM_PROFILE_ID, EV3SSHBinding
from robot_agent.robot_control_contract import RobotControlTarget


class DashboardRobotProfileTests(unittest.TestCase):
    def test_default_profile_remains_disabled(self):
        args = _parser().parse_args([])

        self.assertEqual(args.robot_profile, ROBOT_PROFILE_DISABLED)
        self.assertIsNone(args.robot_target)
        self.assertEqual(args.robot_planner_timeout_seconds, 30.0)
        self.assertEqual(args.robot_input_timeout_seconds, 10.0)
        self.assertIsNone(args.shared_peer_port)
        self.assertIsNone(args.shared_peer_access_key_file)
        self.assertIsNone(_configured_robot_runtime_adapter(args))
        self.assertIsNone(
            _configured_robot_control_target(ROBOT_PROFILE_DISABLED)
        )

    def test_physical_profiles_have_authoritative_control_targets(self):
        self.assertEqual(
            _configured_robot_control_target(BLAST_PROFILE_ID).to_dict(),
            {"robot_id": "blast-01", "display_name": "BLAST"},
        )
        self.assertEqual(
            _configured_robot_control_target(
                EV3RSTORM_PROFILE_ID
            ).to_dict(),
            {
                "robot_id": "ev3rstorm-01",
                "display_name": "EV3RSTORM",
            },
        )
        with self.assertRaisesRegex(ValueError, "unsupported"):
            _configured_robot_control_target("another-robot")

    def test_fixed_start_peer_options_are_all_or_none_and_physical(self):
        local_map = mock.Mock()
        partial = _parser().parse_args([
            "--robot-profile",
            BLAST_PROFILE_ID,
            "--blast-hub-name",
            "BLAST-TEST",
            "--shared-peer-port",
            "8766",
        ])
        with self.assertRaisesRegex(ValueError, "supplied together"):
            _configured_shared_spatial_map(
                partial,
                local_map_provider=local_map,
            )

        disabled = _parser().parse_args([
            "--shared-peer-port",
            "8766",
            "--shared-peer-access-key-file",
            "/tmp/peer-key",
            "--shared-peer-x-mm",
            "600",
            "--shared-peer-y-mm",
            "0",
            "--shared-peer-yaw-mdeg",
            "0",
        ])
        with self.assertRaisesRegex(ValueError, "physical robot profile"):
            _configured_shared_spatial_map(
                disabled,
                local_map_provider=local_map,
            )

    def test_fixed_start_peer_configuration_is_loopback_and_opposite(self):
        args = _parser().parse_args([
            "--robot-profile",
            BLAST_PROFILE_ID,
            "--blast-hub-name",
            "BLAST-TEST",
            "--shared-peer-port",
            "8766",
            "--shared-peer-access-key-file",
            "~/.robot-llm/peer-key",
            "--shared-peer-x-mm",
            "600",
            "--shared-peer-y-mm",
            "-25",
            "--shared-peer-yaw-mdeg",
            "90000",
        ])
        local_map = mock.Mock()
        remote = mock.Mock()
        shared = mock.Mock()
        with (
            mock.patch(
                "robot_agent.dashboard_cli."
                "load_or_create_dashboard_access_key",
                return_value="p" * 64,
            ) as key_loader,
            mock.patch(
                "robot_agent.dashboard_cli.RemoteSpatialMapProvider",
                return_value=remote,
            ) as remote_type,
            mock.patch(
                "robot_agent.dashboard_cli.FixedStartSharedMapProvider",
                return_value=shared,
            ) as shared_type,
        ):
            value = _configured_shared_spatial_map(
                args,
                local_map_provider=local_map,
            )

        self.assertIs(value, shared)
        key_loader.assert_called_once_with(Path("~/.robot-llm/peer-key"))
        remote_type.assert_called_once_with(8766, "p" * 64)
        self.assertEqual(
            shared_type.call_args.kwargs,
            {
                "local_provider": local_map,
                "peer_provider": remote,
                "local_robot_id": "blast-01",
                "local_controller_id": "blast-01.hub",
                "peer_robot_id": "ev3rstorm-01",
                "peer_controller_id": "ev3rstorm-01.ev3-main",
                "peer_tx_mm": 600,
                "peer_ty_mm": -25,
                "peer_yaw_mdeg": 90000,
            },
        )

    def test_ev3_anchor_maps_the_blast_peer_in_the_configured_frame(self):
        args = _parser().parse_args([
            "--robot-profile",
            EV3RSTORM_PROFILE_ID,
            "--robot-target",
            "robot@ev3dev.local",
            "--shared-peer-port",
            "8766",
            "--shared-peer-access-key-file",
            "/tmp/peer-key",
            "--shared-peer-x-mm",
            "-500",
            "--shared-peer-y-mm",
            "100",
            "--shared-peer-yaw-mdeg",
            "-90000",
        ])
        with (
            mock.patch(
                "robot_agent.dashboard_cli."
                "load_or_create_dashboard_access_key",
                return_value="p" * 64,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.RemoteSpatialMapProvider",
                return_value=mock.Mock(),
            ),
            mock.patch(
                "robot_agent.dashboard_cli.FixedStartSharedMapProvider",
                return_value=mock.Mock(),
            ) as shared_type,
        ):
            _configured_shared_spatial_map(
                args,
                local_map_provider=mock.Mock(),
            )

        values = shared_type.call_args.kwargs
        self.assertEqual(
            (
                values["local_robot_id"],
                values["local_controller_id"],
                values["peer_robot_id"],
                values["peer_controller_id"],
            ),
            (
                "ev3rstorm-01",
                "ev3rstorm-01.ev3-main",
                "blast-01",
                "blast-01.hub",
            ),
        )

    def test_fixed_start_peer_rejects_same_port_and_missing_local_map(self):
        base = [
            "--robot-profile",
            EV3RSTORM_PROFILE_ID,
            "--robot-target",
            "robot@ev3dev.local",
            "--shared-peer-port",
            "8765",
            "--shared-peer-access-key-file",
            "/tmp/peer-key",
            "--shared-peer-x-mm",
            "0",
            "--shared-peer-y-mm",
            "0",
            "--shared-peer-yaw-mdeg",
            "0",
        ]
        same_port = _parser().parse_args(base)
        with self.assertRaisesRegex(ValueError, "geometry is invalid"):
            _configured_shared_spatial_map(
                same_port,
                local_map_provider=mock.Mock(),
            )

        base[base.index("8765")] = "8766"
        without_map = _parser().parse_args(base)
        with self.assertRaisesRegex(ValueError, "local physical map"):
            _configured_shared_spatial_map(
                without_map,
                local_map_provider=None,
            )

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

    def test_blast_profile_requires_and_reuses_one_monitor(self):
        # The live 120-character navigation remark in
        # episode-2e6cc0931db38a0b178ae61f synthesized beyond BLAST's
        # eight-second PCM envelope. Keep this transport-specific bound
        # materially below the generic/EV3 utterance limit.
        self.assertEqual(BLAST_MAX_NAVIGATION_UTTERANCE_CHARS, 72)
        args = _parser().parse_args(
            ["--robot-profile", BLAST_PROFILE_ID]
        )
        with self.assertRaisesRegex(ValueError, "--blast-hub-name"):
            _configured_robot_runtime_adapter(args)

        args = _parser().parse_args([
            "--robot-profile",
            BLAST_PROFILE_ID,
            "--blast-hub-name",
            "BLAST-TEST",
            "--robot-planner-timeout-seconds",
            "6.5",
        ])
        monitor = object()
        adapter = object()
        planner = object()
        synthesizer = object()
        speaker = object()
        speech_runtime = object()
        event_sink = mock.Mock()
        with (
            mock.patch(
                "robot_agent.dashboard_cli.BlastEpisodeRuntimeAdapter",
                return_value=adapter,
            ) as adapter_type,
            mock.patch(
                "robot_agent.dashboard_cli.LMStudioControllerActionPlanner",
                return_value=planner,
            ) as planner_type,
            mock.patch(
                "robot_agent.dashboard_cli.LocaleSpeechSynthesizer",
                return_value=synthesizer,
            ) as synthesizer_type,
            mock.patch(
                "robot_agent.dashboard_cli.PiperLoopbackSynthesizer",
                return_value="piper",
            ) as piper_type,
            mock.patch(
                "robot_agent.dashboard_cli.MacOSSayWAVSynthesizer",
                return_value="say-wav",
            ),
            mock.patch(
                "robot_agent.dashboard_cli.BlastHubSpeaker",
                return_value=speaker,
            ) as speaker_type,
            mock.patch(
                "robot_agent.dashboard_cli.RobotSpeechRuntime",
                return_value=speech_runtime,
            ) as speech_runtime_type,
        ):
            result = _configured_robot_runtime_adapter(
                args,
                blast_monitor=monitor,
            )
            self.assertIs(result, adapter)
            self.assertIs(
                adapter_type.call_args.kwargs["controller"],
                monitor,
            )
            planner_factory = adapter_type.call_args.kwargs[
                "planner_factory"
            ]
            planner_type.assert_not_called()
            self.assertIs(planner_factory("model-b"), planner)
            planner_type.assert_called_once_with(
                base_url=args.lm_studio_url,
                model="model-b",
                timeout_seconds=6.5,
                utterance_persona_by_locale=BLAST_PERSONA_BY_LOCALE,
                max_utterance_chars=(
                    BLAST_MAX_NAVIGATION_UTTERANCE_CHARS
                ),
            )
            self.assertEqual(
                adapter_type.call_args.kwargs["speech_locales"],
                ("sv", "en"),
            )
            self.assertNotIn(
                "speech_navigation_gate",
                adapter_type.call_args.kwargs,
            )
            runtime_factory = adapter_type.call_args.kwargs[
                "speech_runtime_factory"
            ]
            self.assertIs(
                runtime_factory(event_sink=event_sink),
                speech_runtime,
            )
            synthesizer_type.assert_called_once_with(
                {"sv": "piper", "en": "say-wav"}
            )
            piper_type.assert_called_once_with(
                profile=BLAST_PIPER_PROFILE,
            )
            speaker_type.assert_called_once_with(
                synthesizer,
                monitor,
            )
            speech_runtime_type.assert_called_once_with(
                speaker=speaker,
                event_sink=event_sink,
                thread_name="blast-01-speech",
            )

    def test_run_binds_blast_profile_to_the_observed_controller(self):
        monitor = mock.Mock()
        adapter = mock.Mock()
        service = mock.Mock()
        control_service = mock.Mock()
        server = mock.Mock()
        router = mock.Mock(session_path="/live/token/")
        input_model = object()
        with (
            mock.patch(
                "robot_agent.dashboard_cli.BlastObservationMonitor",
                return_value=monitor,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.BlastEpisodeRuntimeAdapter",
                return_value=adapter,
            ) as adapter_type,
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=service,
            ) as service_type,
            mock.patch(
                "robot_agent.dashboard_cli.RobotControlService",
                return_value=control_service,
            ) as control_type,
            mock.patch(
                "robot_agent.dashboard_cli.LMStudioRobotInputModel",
                return_value=input_model,
            ) as input_model_type,
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(server, router),
            ) as server_type,
            mock.patch("sys.stdout", new_callable=io.StringIO) as stdout,
        ):
            result = _run([
                "--robot-profile",
                BLAST_PROFILE_ID,
                "--blast-hub-name",
                "BLAST-TEST",
            ])
            input_service = server_type.call_args.kwargs[
                "robot_input_service"
            ]
            built_input_model = input_service._model_factory("model-a")

        self.assertEqual(result, 0)
        monitor.start.assert_not_called()
        monitor.close.assert_called_once_with()
        self.assertIs(adapter_type.call_args.kwargs["controller"], monitor)
        self.assertIs(control_type.call_args.args[0], adapter)
        self.assertEqual(
            control_type.call_args.kwargs["target"].to_dict(),
            {"robot_id": "blast-01", "display_name": "BLAST"},
        )
        self.assertEqual(
            service_type.call_args.kwargs["controller_runtime_providers"],
            (monitor,),
        )
        self.assertIs(built_input_model, input_model)
        input_model_type.assert_called_once_with(
            base_url="http://127.0.0.1:1234",
            model="model-a",
            timeout_seconds=10.0,
            reply_persona_by_locale=BLAST_PERSONA_BY_LOCALE,
        )
        ready = json.loads(stdout.getvalue())
        self.assertEqual(ready["robot_profile"], BLAST_PROFILE_ID)
        self.assertTrue(ready["physical_control_enabled"])

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
        self.assertEqual(
            control_type.call_args.kwargs["target"].to_dict(),
            {
                "robot_id": "ev3rstorm-01",
                "display_name": "EV3RSTORM",
            },
        )
        adapter.run.assert_not_called()
        ready = json.loads(stdout.getvalue())
        self.assertTrue(ready["physical_control_enabled"])
        self.assertEqual(ready["robot_profile"], EV3RSTORM_PROFILE_ID)

    def test_run_configures_ev3_and_blast_as_selectable_targets(self):
        adapter = mock.Mock()
        blast_adapter = mock.Mock()
        reachability = mock.Mock(spec=("snapshot", "check"))
        adapter.controller_runtime_provider = reachability
        adapter.controller_reachability_service = reachability
        monitor = mock.Mock()
        dashboard_service = mock.Mock()
        ev3_control_service = mock.Mock()
        blast_control_service = mock.Mock()
        ev3_input_model = object()
        blast_input_model = object()
        server = mock.Mock()
        router = mock.Mock(session_path="/live/token/")
        with (
            mock.patch(
                "robot_agent.dashboard_cli.BlastObservationMonitor",
                return_value=monitor,
            ) as monitor_type,
            mock.patch(
                "robot_agent.dashboard_cli."
                "_configured_robot_runtime_adapter",
                return_value=adapter,
            ),
            mock.patch(
                "robot_agent.dashboard_cli."
                "_configured_blast_runtime_adapter",
                return_value=blast_adapter,
            ) as blast_configured,
            mock.patch(
                "robot_agent.dashboard_cli.DashboardService",
                return_value=dashboard_service,
            ) as service_type,
            mock.patch(
                "robot_agent.dashboard_cli.RobotControlService",
                side_effect=(
                    ev3_control_service,
                    blast_control_service,
                ),
            ) as control_type,
            mock.patch(
                "robot_agent.dashboard_cli.LMStudioRobotInputModel",
                side_effect=(ev3_input_model, blast_input_model),
            ) as input_model_type,
            mock.patch(
                "robot_agent.dashboard_cli.build_server",
                return_value=(server, router),
            ) as server_type,
            mock.patch("sys.stdout", new_callable=io.StringIO),
        ):
            result = _run([
                "--robot-profile",
                EV3RSTORM_PROFILE_ID,
                "--robot-target",
                "robot@ev3dev.local",
                "--blast-hub-name",
                "BLAST-TEST",
            ])
            input_services = server_type.call_args.kwargs[
                "robot_input_services"
            ]
            built_ev3_model = input_services[
                "ev3rstorm-01"
            ]._model_factory("model-a")
            built_blast_model = input_services[
                "blast-01"
            ]._model_factory("model-b")

        self.assertEqual(result, 0)
        monitor_type.assert_called_once_with(hub_name="BLAST-TEST")
        monitor.start.assert_not_called()
        adapter.run.assert_not_called()
        blast_adapter.run.assert_not_called()
        blast_configured.assert_called_once_with(
            mock.ANY,
            monitor,
        )
        self.assertEqual(control_type.call_count, 2)
        ev3_gate = control_type.call_args_list[0].kwargs["episode_gate"]
        blast_gate = control_type.call_args_list[1].kwargs["episode_gate"]
        self.assertIs(ev3_gate, blast_gate)
        self.assertEqual(
            service_type.call_args.kwargs["controller_runtime_providers"],
            (monitor, reachability),
        )
        self.assertEqual(
            server_type.call_args.kwargs["controller_control_services"],
            {
                "blast-01.hub": monitor,
                "ev3rstorm-01.ev3-main": reachability,
            },
        )
        self.assertEqual(
            server_type.call_args.kwargs["robot_control_services"],
            {
                "ev3rstorm-01": ev3_control_service,
                "blast-01": blast_control_service,
            },
        )
        self.assertEqual(
            set(server_type.call_args.kwargs["robot_input_services"]),
            {"ev3rstorm-01", "blast-01"},
        )
        self.assertEqual(
            server_type.call_args.kwargs["default_robot_id"],
            "ev3rstorm-01",
        )
        self.assertIs(built_ev3_model, ev3_input_model)
        self.assertIs(built_blast_model, blast_input_model)
        self.assertEqual(
            input_model_type.call_args_list,
            [
                mock.call(
                    base_url="http://127.0.0.1:1234",
                    model="model-a",
                    timeout_seconds=10.0,
                ),
                mock.call(
                    base_url="http://127.0.0.1:1234",
                    model="model-b",
                    timeout_seconds=10.0,
                    reply_persona_by_locale=BLAST_PERSONA_BY_LOCALE,
                ),
            ],
        )

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
            ) as server_type,
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
            server_type.call_args.kwargs[
                "robot_spatial_map_providers"
            ],
            {"ev3rstorm-01": map_provider},
        )
        self.assertEqual(
            json.loads(stdout.getvalue())["spatial_map_mode"],
            "physical_live",
        )
        map_provider.close.assert_called_once_with(drain=True)

    def test_run_exposes_shared_map_but_keeps_local_map_as_robot_facts(self):
        adapter = mock.Mock()
        local_map = mock.Mock()
        local_map.snapshot = mock.Mock(return_value={})
        local_map.close = mock.Mock(return_value=True)
        adapter.spatial_map_provider = local_map
        peer_map = mock.Mock()
        shared_map = mock.Mock()
        shared_map.close = mock.Mock()
        peer_key = "private-peer-key-" + "p" * 48
        dashboard_service = mock.Mock()
        control_service = mock.Mock()
        server = mock.Mock()
        router = mock.Mock(session_path="/live/token/")
        with (
            mock.patch(
                "robot_agent.dashboard_cli."
                "_configured_robot_runtime_adapter",
                return_value=adapter,
            ),
            mock.patch(
                "robot_agent.dashboard_cli."
                "load_or_create_dashboard_access_key",
                return_value=peer_key,
            ),
            mock.patch(
                "robot_agent.dashboard_cli.RemoteSpatialMapProvider",
                return_value=peer_map,
            ) as peer_type,
            mock.patch(
                "robot_agent.dashboard_cli.FixedStartSharedMapProvider",
                return_value=shared_map,
            ) as shared_type,
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
            ) as server_type,
            mock.patch("sys.stdout", new_callable=io.StringIO) as stdout,
        ):
            result = _run([
                "--robot-profile",
                EV3RSTORM_PROFILE_ID,
                "--robot-target",
                "robot@ev3dev.local",
                "--shared-peer-port",
                "8766",
                "--shared-peer-access-key-file",
                "/tmp/private-peer-key",
                "--shared-peer-x-mm",
                "600",
                "--shared-peer-y-mm",
                "0",
                "--shared-peer-yaw-mdeg",
                "0",
            ])

        self.assertEqual(result, 0)
        peer_type.assert_called_once_with(8766, peer_key)
        self.assertIs(
            shared_type.call_args.kwargs["local_provider"],
            local_map,
        )
        self.assertIs(
            shared_type.call_args.kwargs["peer_provider"],
            peer_map,
        )
        self.assertIs(
            dashboard_type.call_args.kwargs["spatial_map_provider"],
            local_map,
        )
        self.assertIs(
            dashboard_type.call_args.kwargs[
                "shared_spatial_map_provider"
            ],
            shared_map,
        )
        input_service = server_type.call_args.kwargs[
            "robot_input_service"
        ]
        self.assertIs(input_service._map_snapshot, local_map.snapshot)
        ready_output = stdout.getvalue()
        self.assertNotIn(peer_key, ready_output)
        self.assertEqual(
            json.loads(ready_output)["shared_spatial_map_mode"],
            "fixed_start_peer",
        )
        local_map.close.assert_called_once_with(drain=True)
        shared_map.close.assert_not_called()

    def test_run_keeps_interactive_timeout_independent_of_planner(self):
        adapter = mock.Mock()
        adapter.robot_control_target = RobotControlTarget(
            robot_id="injected-robot",
            display_name="Injected Robot",
        )
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
