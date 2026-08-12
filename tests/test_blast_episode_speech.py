import threading
import unittest
from types import SimpleNamespace

from robot_agent.blast_episode_speech import BlastEpisodeSpeech
from robot_agent.dashboard_contract import DashboardContractError
from robot_agent.robot_control_contract import RobotRuntimeUpdate


class _SpeechRuntime:
    def __init__(self, *, offer_error=None):
        self.offer_error = offer_error

    def start(self):
        return None

    def offer(self, **_offer):
        if self.offer_error is not None:
            raise self.offer_error
        return 1

    def cancel_episode(self, _episode_id):
        return None

    def close(self, **_options):
        return True


def _context(updates):
    return SimpleNamespace(
        episode_id="episode-1",
        request=SimpleNamespace(locale="sv"),
        settings=SimpleNamespace(speech_enabled=True),
        stop_requested=threading.Event(),
        emergency_stop_requested=threading.Event(),
        publish=lambda update: updates.append(dict(update)),
    )


class BlastEpisodeSpeechObservabilityTests(unittest.TestCase):
    def test_worker_failure_publishes_bounded_code_and_later_status_clears_it(self):
        updates = []
        captured = {}

        def factory(*, event_sink):
            captured["event_sink"] = event_sink
            return _SpeechRuntime()

        speech = BlastEpisodeSpeech(
            factory=factory,
            supported_locales=("sv",),
            context=_context(updates),
        )
        speech.start()

        with self.assertLogs(
            "robot_agent.blast_episode_speech",
            level="WARNING",
        ) as logged:
            captured["event_sink"]({
                "event": "speech_failed",
                "speech_status": "failed",
                "reason": "tts_audio_too_long",
            })
        captured["event_sink"]({
            "event": "speech_playing",
            "speech_status": "playing",
        })

        self.assertEqual(
            updates[-2],
            {
                "speech_status": "failed",
                "speech_error_code": "tts_audio_too_long",
            },
        )
        self.assertEqual(
            updates[-1],
            {
                "speech_status": "playing",
                "speech_error_code": None,
            },
        )
        self.assertIn("stage=playback", logged.output[0])
        self.assertIn("code=tts_audio_too_long", logged.output[0])

    def test_sync_offer_failure_logs_stage_without_private_message(self):
        updates = []

        class SpeechFailure(RuntimeError):
            code = "controller_unavailable"

        speech = BlastEpisodeSpeech(
            factory=lambda **_options: _SpeechRuntime(
                offer_error=SpeechFailure("private BLE details"),
            ),
            supported_locales=("sv",),
            context=_context(updates),
        )
        speech.start()

        with self.assertLogs(
            "robot_agent.blast_episode_speech",
            level="WARNING",
        ) as logged:
            speech.offer("Hej", progress_revision=1)

        self.assertEqual(
            updates[-1],
            {
                "speech_status": "failed",
                "speech_error_code": "controller_unavailable",
            },
        )
        self.assertIn("stage=offer", logged.output[0])
        self.assertIn("error_type=SpeechFailure", logged.output[0])
        self.assertNotIn("private BLE details", logged.output[0])

    def test_invalid_worker_reason_is_replaced_before_publication_or_logging(self):
        updates = []
        captured = {}

        def factory(*, event_sink):
            captured["event_sink"] = event_sink
            return _SpeechRuntime()

        speech = BlastEpisodeSpeech(
            factory=factory,
            supported_locales=("sv",),
            context=_context(updates),
        )
        speech.start()
        with self.assertLogs(
            "robot_agent.blast_episode_speech",
            level="WARNING",
        ) as logged:
            captured["event_sink"]({
                "speech_status": "failed",
                "reason": "private detail with spaces",
            })

        self.assertEqual(updates[-1]["speech_error_code"], "speech_failed")
        self.assertNotIn("private detail", logged.output[0])

    def test_runtime_contract_preserves_failure_code_and_clears_on_recovery(self):
        failed = RobotRuntimeUpdate.from_mapping({
            "speech_status": "failed",
            "speech_error_code": "tts_audio_too_long",
        })
        self.assertEqual(
            failed.to_dict()["speech_error_code"],
            "tts_audio_too_long",
        )

        recovered = RobotRuntimeUpdate.from_mapping(
            {"speech_status": "playing"},
            failed,
        )
        self.assertIsNone(recovered.speech_error_code)

        with self.assertRaises(DashboardContractError):
            RobotRuntimeUpdate.from_mapping({
                "speech_status": "failed",
                "speech_error_code": "not a bounded code",
            })


if __name__ == "__main__":
    unittest.main()
