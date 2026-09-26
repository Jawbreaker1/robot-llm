import json
import threading
import unittest
from unittest.mock import Mock

from robot_agent.ev3_social_speech import EV3SocialSpeaker, SAW_DURATIONS_MS, SAW_SPEED_DPS
from robot_agent.robot_turn_speech import cancellable_serialized_speaker
from robot_agent.social_expression import EV3_GESTURES


class EV3SocialSpeechTests(unittest.TestCase):
    def make_speaker(self):
        calls = []
        audio = Mock(return_value={"played": True})

        def process_factory(argv, **kwargs):
            calls.append(argv)
            speed = int(argv[argv.index("--speed-dps") + 1])
            duration = int(argv[argv.index("--duration-ms") + 1])
            result = {"status": "completed", "role": "arm", "port": "outA",
                      "speed_dps": speed, "duration_ms": duration, "position_delta": speed * duration / 1000,
                      "verification": {"passed": True}}
            return Mock(returncode=0, communicate=Mock(return_value=(json.dumps(result).encode(), b"")))

        lease = threading.Lock()
        speaker = EV3SocialSpeaker(audio, "robot@ev3dev.local", lease,
                                   process_factory=process_factory, gesture_allowed=lambda: True)
        return speaker, audio, calls, lease

    def test_both_gestures_use_only_existing_arm_command_after_speech(self):
        self.assertEqual(set(EV3_GESTURES) - {"none"}, set(SAW_DURATIONS_MS))
        for gesture, duration in SAW_DURATIONS_MS.items():
            with self.subTest(gesture=gesture):
                speaker, audio, calls, lease = self.make_speaker()
                wrapped = cancellable_serialized_speaker(speaker, threading.Lock())
                result = wrapped("Hmph. Victory.", "en", threading.Event(),
                                 expression={"face": "neutral", "gesture": gesture})
                self.assertEqual(result, {"played": True})
                audio.assert_called_once()
                self.assertEqual([int(call[call.index("--speed-dps") + 1]) for call in calls], [SAW_SPEED_DPS])
                for call in calls:
                    self.assertEqual(call[call.index("--role") + 1], "arm")
                    self.assertEqual(call[call.index("--duration-ms") + 1], str(duration))
                self.assertFalse(lease.locked())

    def test_busy_navigation_cancellation_and_unsupported_gestures_do_not_move_arm(self):
        for condition in ("lease", "busy", "cancelled", "claw"):
            with self.subTest(condition=condition):
                speaker, audio, calls, lease = self.make_speaker()
                cancel = threading.Event()
                if condition == "lease":
                    lease.acquire()
                elif condition == "busy":
                    speaker.gesture_allowed = lambda: False
                elif condition == "cancelled":
                    cancel.set()
                gesture = "claw_snap" if condition == "claw" else "saw_flourish"
                speaker("Hi.", "en", cancel, expression={"face": "neutral", "gesture": gesture})
                self.assertEqual(calls, [])
                self.assertEqual(lease.locked(), condition == "lease")

    def test_failed_arm_receipt_preserves_speech_and_releases_lease(self):
        speaker, audio, calls, lease = self.make_speaker()
        speaker.process_factory = Mock(return_value=Mock(
            returncode=0, communicate=Mock(return_value=(b'{"status":"failed"}', b""))))
        with self.assertLogs("robot_agent.ev3_social_speech", level="WARNING"):
            result = speaker("Hello.", "en", threading.Event(),
                             expression={"face": "neutral", "gesture": "saw_celebrate"})
        self.assertEqual(result, {"played": True})
        speaker.process_factory.assert_called_once()
        self.assertFalse(lease.locked())

    def test_cancellation_after_spin_does_not_start_another(self):
        speaker, audio, calls, lease = self.make_speaker()
        cancel = threading.Event()
        original = speaker._spin

        def segment(speed, event):
            result = original(speed, event)
            cancel.set()
            return result

        speaker._spin = segment
        speaker("Hello.", "en", cancel, expression={"face": "neutral", "gesture": "saw_celebrate"})
        self.assertEqual(len(calls), 1)
        self.assertFalse(lease.locked())
