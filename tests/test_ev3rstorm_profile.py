import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock
from robot_agent.blast_personality import EV3_PERSONA_BY_LOCALE

from robot_agent.controller_runtime_profile import (
    ControllerRuntimeProfileError,
)
from robot_agent.ev3rstorm_profile import (
    EV3RSTORM_PROFILE_ID,
    EV3RSTORM_REMOTE_WORKER_PATH,
    EV3RSTORMProfile,
    EV3SSHBinding,
)
from robot_agent.host_piper_speech import PiperSpeechProfile
from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge
from robot_agent.physical_odometry import OdometryCalibration


class EV3RSTORMProfileTests(unittest.TestCase):
    def setUp(self):
        self.profile = EV3RSTORMProfile()

    def test_checked_in_config_supplies_profile_identity(self):
        descriptor = self.profile.descriptor

        self.assertEqual(descriptor.profile_id, EV3RSTORM_PROFILE_ID)
        self.assertEqual(descriptor.robot_id, "ev3rstorm-01")
        self.assertEqual(
            descriptor.controller_id,
            "ev3rstorm-01.ev3-main",
        )
        self.assertIn("sensor.infrared", descriptor.capabilities)
        self.assertTrue(self.profile.config_path.is_file())
        self.assertEqual(
            self.profile.odometry_calibration,
            OdometryCalibration(
                linear_mm_per_encoder_degree=0.35,
                turn_mdeg_per_opposed_encoder_degree=137,
            ),
        )
        self.assertEqual(
            self.profile.hazard_calibration.provisional_hazard_offset_mm,
            210,
        )
        self.assertEqual(
            self.profile.hazard_calibration.detour_lateral_clearance_margin_mm,
            30,
        )


    def test_legacy_schema_v1_config_keeps_prior_hazard_offset(self):
        config = json.loads(self.profile.config_path.read_text("utf-8"))
        del config["calibration"]["infrared_proximity"][
            "provisional_hazard_offset_mm"
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy-config.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            legacy = EV3RSTORMProfile(path)

        self.assertEqual(
            legacy.hazard_calibration.provisional_hazard_offset_mm,
            140,
        )

    def test_legacy_schema_v1_config_has_no_detour_margin(self):
        config = json.loads(self.profile.config_path.read_text("utf-8"))
        del config["calibration"]["local_detour"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy-config.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            legacy = EV3RSTORMProfile(path)

        self.assertEqual(
            legacy.hazard_calibration.detour_lateral_clearance_margin_mm,
            0,
        )

    def test_binding_validates_location_without_contacting_it(self):
        with tempfile.TemporaryDirectory() as directory:
            binding = EV3SSHBinding(
                profile_id=EV3RSTORM_PROFILE_ID,
                target="robot@192.168.1.42",
                memory_path=Path(directory) / "memory.json",
            )

        self.assertEqual(binding.target, "robot@192.168.1.42")
        self.assertTrue(binding.memory_path.is_absolute())
        self.assertEqual(
            binding.remote_worker_path,
            EV3RSTORM_REMOTE_WORKER_PATH,
        )
        for target in ("", " robot@host", "-host", "robot@host;bad"):
            with self.subTest(target=target):
                with self.assertRaises(ControllerRuntimeProfileError):
                    EV3SSHBinding(
                        profile_id=EV3RSTORM_PROFILE_ID,
                        target=target,
                        memory_path="memory.json",
                    )

    def test_shared_navigation_is_wired_but_hardware_and_speech_stay_lazy(self):
        binding = EV3SSHBinding(
            profile_id=EV3RSTORM_PROFILE_ID,
            target="robot@ev3dev.local",
            memory_path="/tmp/unused-ev3-memory.json",
        )
        with (
            mock.patch("robot_agent.ev3rstorm_profile.EV3NavigationSSHTransport") as transport,
            mock.patch("robot_agent.ev3rstorm_profile.PiperLoopbackSynthesizer") as synth,
            mock.patch("robot_agent.ev3rstorm_profile.EV3WAVSSHSession") as player,
            mock.patch("robot_agent.ev3rstorm_profile.HostPiperEV3Speaker") as speaker,
            mock.patch("robot_agent.ev3rstorm_profile.RobotSpeechRuntime") as speech,
        ):
            adapter = self.profile.build_adapter(binding)
            for factory in (transport, synth, player, speaker, speech):
                factory.assert_not_called()
            self.assertTrue(callable(adapter.navigation_adapter.run))
            self.assertFalse(hasattr(adapter, "navigation_unavailable_reason"))
            self.assertIs(adapter.navigation_adapter.spatial_map_provider, adapter.spatial_map_provider)
            self.assertIs(adapter.navigation_adapter.controller_lease, adapter.controller_lease)
            self.assertFalse(hasattr(adapter, "run"))
            self.assertFalse(hasattr(adapter, "planner_factory"))
            self.assertIsInstance(adapter.spatial_map_provider, PhysicalSpatialMapBridge)
            self.assertEqual(adapter.spatial_map_provider.snapshot()["status"], "unavailable")
            self.assertIs(adapter.controller_runtime_provider, adapter.controller_reachability_service)
            self.assertIs(adapter.controller_lease, adapter.controller_reachability_service._lease)
            self.assertEqual(adapter.controller_runtime_provider.snapshot()["reachability"]["status"], "not_checked")

            self.assertIs(adapter.transport_factory(), transport.return_value)
            transport.assert_called_once_with(
                target="robot@ev3dev.local",
                controller_id="ev3rstorm-01.ev3-main",
                remote_worker_path=EV3RSTORM_REMOTE_WORKER_PATH,
                connect_timeout_seconds=5,
            )
            event_sink = mock.Mock()
            self.assertIs(adapter.speech_runtime_factory(event_sink=event_sink), speech.return_value)
            self.assertEqual(adapter.speech_locales, ("sv", "en"))
            synth.assert_called_once_with(self.profile.speech_profile)
            player.assert_called_once_with("robot@ev3dev.local", connect_timeout_seconds=5)
            speaker.assert_called_once_with(synth.return_value, player.return_value)
            speech.assert_called_once_with(
                speaker=mock.ANY, speaker_close=player.return_value.close,
                event_sink=event_sink, thread_name="ev3rstorm-01.ev3-main-speech",
            )
            self.assertTrue(callable(speech.call_args.kwargs["speaker"]))
            self.assertEqual(adapter.robot_description["robot_id"], "ev3rstorm-01")
            self.assertIn("motor.arm", adapter.robot_description["capabilities"])
            self.assertNotIn("navigation_status", adapter.robot_description)

    def test_profile_uses_shared_controller_action_planner_reasoning_defaults(self):
        adapter = self.profile.build_adapter(
            EV3SSHBinding(profile_id=EV3RSTORM_PROFILE_ID, target="robot@ev3dev.local",
                          memory_path="/tmp/unused-ev3-memory.json"))
        with mock.patch("robot_agent.ev3rstorm_profile.LMStudioControllerActionPlanner") as planner:
            self.assertIs(adapter.navigation_adapter.planner_factory("qwen-test"), planner.return_value)
        planner.assert_called_once_with(model="qwen-test", max_output_tokens=4096, timeout_seconds=60,
            social_expressions=True, social_gestures=("none", "saw_flourish", "saw_celebrate"),
            social_faces=("neutral",),
            utterance_persona_by_locale=EV3_PERSONA_BY_LOCALE)

    def test_reachability_uses_the_bound_motion_free_preflight(self):
        report = {
            "status": "passed",
            "effects": "motion_free",
            "contract_checks": {
                "motor_commands_issued": 0,
                "shutdown_confirmed": True,
                "motor_owner_closed": True,
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            binding = EV3SSHBinding(
                profile_id=EV3RSTORM_PROFILE_ID,
                target="robot@ev3dev.local",
                memory_path=Path(directory) / "memory.json",
            )
            with mock.patch(
                "robot_agent.ev3rstorm_profile."
                "run_ev3_navigation_preflight",
                return_value=report,
            ) as preflight:
                adapter = self.profile.build_adapter(
                    binding,
                )
                preflight.assert_not_called()
                result = adapter.controller_reachability_service.check()

        self.assertEqual(result["reachability"]["status"], "passed")
        preflight.assert_called_once_with(
            "robot@ev3dev.local",
            controller_id="ev3rstorm-01.ev3-main",
            remote_worker_path=EV3RSTORM_REMOTE_WORKER_PATH,
            connect_timeout_seconds=5,
        )

    def test_default_speech_profile_uses_piper_for_both_languages(self):
        self.assertEqual(
            self.profile.speech_profile,
            PiperSpeechProfile(
                voices=(("sv", "nst-deep"), ("en", "john-medium")),
            ),
        )

    def test_all_profile_speech_runtimes_share_one_playback_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            binding = EV3SSHBinding(
                profile_id=EV3RSTORM_PROFILE_ID,
                target="robot@ev3dev.local",
                memory_path=Path(directory) / "memory.json",
            )
            with (
                mock.patch(
                    "robot_agent.ev3rstorm_profile.PiperLoopbackSynthesizer",
                ),
                mock.patch(
                    "robot_agent.ev3rstorm_profile.EV3WAVSSHSession",
                ),
                mock.patch(
                    "robot_agent.ev3rstorm_profile.HostPiperEV3Speaker",
                ),
                mock.patch(
                    "robot_agent.ev3rstorm_profile."
                    "cancellable_serialized_speaker",
                    side_effect=(mock.Mock(), mock.Mock()),
                ) as serialize,
                mock.patch(
                    "robot_agent.ev3rstorm_profile.RobotSpeechRuntime",
                ),
            ):
                adapter = self.profile.build_adapter(
                    binding,
                )
                adapter.speech_runtime_factory(event_sink=None)
                adapter.speech_runtime_factory(event_sink=None)

        self.assertEqual(serialize.call_count, 2)
        self.assertIs(
            serialize.call_args_list[0].args[1],
            serialize.call_args_list[1].args[1],
        )

    def test_profile_rejects_another_binding_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            binding = EV3SSHBinding(
                profile_id="another-profile",
                target="robot@ev3dev.local",
                memory_path=Path(directory) / "memory.json",
            )
            with self.assertRaises(ControllerRuntimeProfileError):
                self.profile.build_adapter(
                    binding,
                    planner_factory=mock.Mock(),
                )


if __name__ == "__main__":
    unittest.main()
