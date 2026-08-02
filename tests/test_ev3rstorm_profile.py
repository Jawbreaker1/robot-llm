import tempfile
import unittest
from pathlib import Path
from unittest import mock

from ev3.navigation_profile import MAX_PROCESS_SECONDS
from robot_agent.controller_runtime_profile import (
    ControllerRuntimeProfileError,
)
from robot_agent.ev3rstorm_profile import (
    EV3RSTORM_ACTIVE_IR_SCAN_CALIBRATION,
    EV3RSTORM_GOAL_HEADING_TOLERANCE_MDEG,
    EV3RSTORM_LEGACY_SHADOW_DIRECTORY_NAME,
    EV3RSTORM_PLAN_TAIL_MAX_AGE_SECONDS,
    EV3RSTORM_PROFILE_ID,
    EV3RSTORM_REMOTE_WORKER_PATH,
    EV3RSTORM_REQUEST_TIMEOUT_SECONDS,
    EV3RSTORM_SCAN_REQUEST_TIMEOUT_SECONDS,
    EV3RSTORM_SCAN_RESTORATION_HEADROOM_MS,
    EV3RSTORM_SCAN_TIMEOUT_SECONDS,
    EV3RSTORM_STARTUP_TIMEOUT_SECONDS,
    EV3RSTORMProfile,
    EV3SSHBinding,
)
from robot_agent.host_piper_speech import PiperSpeechProfile
from robot_agent.physical_spatial_map import PhysicalSpatialMapBridge
from robot_agent.physical_odometry import OdometryCalibration
from robot_agent.persistent_legacy_shadow_journal import (
    PersistentLegacyShadowSession,
)


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
                turn_mdeg_per_opposed_encoder_degree=132,
            ),
        )

    def test_worker_lifetime_can_cover_default_plan_scan_and_reanchor(self):
        # Dashboard planner (30 s) + EV3 scan (80 s) + post-scan request
        # (30 s) + the runtime's fixed 2 s cleanup margin.
        required_seconds = (
            30.0
            + EV3RSTORM_SCAN_TIMEOUT_SECONDS
            + EV3RSTORM_REQUEST_TIMEOUT_SECONDS
            + 2.0
        )
        self.assertGreaterEqual(MAX_PROCESS_SECONDS, required_seconds)

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

    def test_adapter_is_lazy_and_memory_reset_is_one_shot(self):
        with tempfile.TemporaryDirectory() as directory:
            binding = EV3SSHBinding(
                profile_id=EV3RSTORM_PROFILE_ID,
                target="robot@ev3dev.local",
                memory_path=Path(directory) / "memory.json",
                reset_memory=True,
            )
            planner_factory = mock.Mock(name="planner_factory")
            transport_value = mock.Mock(name="transport")
            memory_value = mock.Mock(name="memory")
            scan_value = mock.Mock(name="scan")
            synthesizer_value = mock.Mock(name="synthesizer")
            player_value = mock.Mock(name="player")
            speaker_value = mock.Mock(name="speaker")
            speech_runtime_value = mock.Mock(name="speech_runtime")
            with (
                mock.patch(
                    "robot_agent.ev3rstorm_profile."
                    "EV3NavigationSSHTransport",
                    return_value=transport_value,
                ) as transport_factory,
                mock.patch(
                    "robot_agent.ev3rstorm_profile."
                    "NavigationMemoryStore.load",
                    return_value=memory_value,
                ) as memory_factory,
                mock.patch(
                    "robot_agent.ev3rstorm_profile."
                    "build_ev3_active_ir_scan_executor",
                    return_value=scan_value,
                ) as scan_factory,
                mock.patch(
                    "robot_agent.ev3rstorm_profile."
                    "PiperLoopbackSynthesizer",
                    return_value=synthesizer_value,
                ) as synthesizer_factory,
                mock.patch(
                    "robot_agent.ev3rstorm_profile.EV3WAVSSHSession",
                    return_value=player_value,
                ) as player_factory,
                mock.patch(
                    "robot_agent.ev3rstorm_profile.HostPiperEV3Speaker",
                    return_value=speaker_value,
                ) as speaker_factory,
                mock.patch(
                    "robot_agent.ev3rstorm_profile.RobotSpeechRuntime",
                    return_value=speech_runtime_value,
                ) as speech_runtime_factory,
            ):
                adapter = self.profile.build_adapter(
                    binding,
                    planner_factory=planner_factory,
                )

                transport_factory.assert_not_called()
                memory_factory.assert_not_called()
                scan_factory.assert_not_called()
                synthesizer_factory.assert_not_called()
                player_factory.assert_not_called()
                speaker_factory.assert_not_called()
                speech_runtime_factory.assert_not_called()
                planner_factory.assert_not_called()
                self.assertEqual(
                    adapter.startup_timeout_seconds,
                    EV3RSTORM_STARTUP_TIMEOUT_SECONDS,
                )
                self.assertEqual(
                    adapter.request_timeout_seconds,
                    EV3RSTORM_REQUEST_TIMEOUT_SECONDS,
                )
                self.assertEqual(adapter.request_timeout_seconds, 30.0)
                self.assertEqual(
                    adapter.scan_timeout_seconds,
                    EV3RSTORM_SCAN_TIMEOUT_SECONDS,
                )
                self.assertEqual(adapter.scan_timeout_seconds, 80.0)
                self.assertEqual(
                    adapter.goal_heading_tolerance_mdeg,
                    EV3RSTORM_GOAL_HEADING_TOLERANCE_MDEG,
                )
                self.assertEqual(
                    adapter.plan_tail_max_age_seconds,
                    EV3RSTORM_PLAN_TAIL_MAX_AGE_SECONDS,
                )
                self.assertIs(
                    adapter.active_scan_calibration,
                    EV3RSTORM_ACTIVE_IR_SCAN_CALIBRATION,
                )
                self.assertEqual(
                    adapter.active_scan_calibration.alignment_tolerance_mdeg,
                    10_000,
                )
                self.assertIsInstance(
                    adapter.spatial_map_provider,
                    PhysicalSpatialMapBridge,
                )
                self.assertEqual(
                    adapter.spatial_map_provider.snapshot()["status"],
                    "unavailable",
                )

                self.assertIs(adapter.transport_factory(), transport_value)
                transport_factory.assert_called_once_with(
                    target="robot@ev3dev.local",
                    controller_id="ev3rstorm-01.ev3-main",
                    remote_worker_path=EV3RSTORM_REMOTE_WORKER_PATH,
                    connect_timeout_seconds=5,
                )
                self.assertIs(adapter.memory_factory(), memory_value)
                self.assertIs(adapter.memory_factory(), memory_value)
                self.assertEqual(memory_factory.call_count, 2)
                self.assertEqual(
                    [
                        call.kwargs["reset"]
                        for call in memory_factory.call_args_list
                    ],
                    [True, False],
                )
                memory_value.save.assert_called_once_with()
                for call in memory_factory.call_args_list:
                    self.assertEqual(call.kwargs["path"], binding.memory_path)
                    self.assertEqual(call.kwargs["robot_id"], "ev3rstorm-01")
                    self.assertEqual(
                        call.kwargs["controller_instance_id"],
                        "ev3rstorm-01.ev3-main",
                    )
                    self.assertEqual(
                        call.kwargs["odometry_calibration"],
                        self.profile.odometry_calibration,
                    )
                self.assertIs(
                    adapter.scan_executor_factory(transport_value),
                    scan_value,
                )
                scan_factory.assert_called_once_with(
                    transport_value,
                    request_timeout_seconds=(
                        EV3RSTORM_SCAN_REQUEST_TIMEOUT_SECONDS
                    ),
                    restoration_headroom_ms=(
                        EV3RSTORM_SCAN_RESTORATION_HEADROOM_MS
                    ),
                )
                event_sink = mock.Mock(name="event_sink")
                self.assertIs(
                    adapter.speech_runtime_factory(event_sink=event_sink),
                    speech_runtime_value,
                )
                synthesizer_factory.assert_called_once_with(
                    self.profile.speech_profile,
                )
                player_factory.assert_called_once_with(
                    "robot@ev3dev.local",
                    connect_timeout_seconds=5,
                )
                speaker_factory.assert_called_once_with(
                    synthesizer_value,
                    player_value,
                )
                speech_runtime_factory.assert_called_once_with(
                    speaker=speaker_value,
                    speaker_close=player_value.close,
                    event_sink=event_sink,
                    thread_name="ev3rstorm-01.ev3-main-speech",
                )
                planner_factory.assert_not_called()

    def test_default_speech_profile_is_swedish_nst_deep_only(self):
        self.assertEqual(
            self.profile.speech_profile,
            PiperSpeechProfile(
                voices=(("sv", "nst-deep"),),
            ),
        )

    def test_adapter_persists_one_passive_shadow_journal_per_episode(self):
        with tempfile.TemporaryDirectory() as directory:
            binding = EV3SSHBinding(
                profile_id=EV3RSTORM_PROFILE_ID,
                target="robot@ev3dev.local",
                memory_path=Path(directory) / "memory.json",
            )
            adapter = self.profile.build_adapter(
                binding,
                planner_factory=mock.Mock(),
            )

            self.assertFalse(
                (
                    binding.memory_path.parent
                    / EV3RSTORM_LEGACY_SHADOW_DIRECTORY_NAME
                ).exists()
            )
            shadow = adapter.canonical_shadow_factory(
                episode_id="episode-profile-test",
            )
            self.assertIsInstance(shadow, PersistentLegacyShadowSession)
            self.assertEqual(
                shadow.path.parent,
                binding.memory_path.parent
                / EV3RSTORM_LEGACY_SHADOW_DIRECTORY_NAME,
            )
            self.assertEqual(shadow.path.suffix, ".ndjson")
            self.assertNotIn("episode-profile-test", shadow.path.name)
            shadow.observe("episode_start", physical_authority="legacy")
            self.assertTrue(shadow.close())
            self.assertEqual(
                shadow.path.read_text(encoding="utf-8").count("\n"),
                1,
            )
            second = adapter.canonical_shadow_factory(
                episode_id="episode-profile-test-2",
            )
            self.assertNotEqual(second.path, shadow.path)
            self.assertTrue(second.close())

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
