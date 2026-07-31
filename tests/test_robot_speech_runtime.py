import threading
import unittest

from robot_agent.robot_speech_runtime import (
    RobotSpeechRuntime,
    RobotSpeechRuntimeError,
    ev3_ssh_speaker,
)


class RobotSpeechRuntimeTests(unittest.TestCase):
    def test_slow_speech_is_nonblocking_and_latest_pending_wins(self):
        first_started = threading.Event()
        release_first = threading.Event()
        third_completed = threading.Event()
        spoken = []
        events = []

        def speaker(text, locale, cancel_event):
            spoken.append((text, locale))
            if text == "first":
                first_started.set()
                release_first.wait(1)
            self.assertFalse(cancel_event.is_set())

        def event_sink(event):
            events.append(dict(event))
            if (
                event["event"] == "speech_completed"
                and event["sequence"] == 3
            ):
                third_completed.set()

        runtime = RobotSpeechRuntime(
            speaker=speaker,
            event_sink=event_sink,
        )
        self.addCleanup(runtime.close, timeout_seconds=0.2)
        runtime.start()

        self.assertEqual(
            runtime.offer(
                episode_id="episode-a",
                text="first",
                locale="sv",
            ),
            1,
        )
        self.assertTrue(first_started.wait(1))
        self.assertEqual(
            runtime.offer(
                episode_id="episode-a",
                text="second",
                locale="sv",
            ),
            2,
        )
        self.assertEqual(
            runtime.offer(
                episode_id="episode-a",
                text="third",
                locale="en",
            ),
            3,
        )
        release_first.set()
        self.assertTrue(third_completed.wait(1))

        self.assertEqual(
            spoken,
            [("first", "sv"), ("third", "en")],
        )
        self.assertTrue(
            any(
                event["event"] == "speech_dropped"
                and event["sequence"] == 2
                and event["reason"] == "replaced_by_newer"
                for event in events
            )
        )

    def test_cancelled_episode_cancels_pending_and_active_once(self):
        started = threading.Event()
        finished = threading.Event()
        events = []

        def speaker(_text, _locale, cancel_event):
            started.set()
            self.assertTrue(cancel_event.wait(1))

        def event_sink(event):
            events.append(dict(event))
            if (
                event["event"] == "speech_cancelled"
                and event["sequence"] == 1
            ):
                finished.set()

        runtime = RobotSpeechRuntime(
            speaker=speaker,
            event_sink=event_sink,
        )
        self.addCleanup(runtime.close, timeout_seconds=0.2)
        runtime.start()
        runtime.offer(
            episode_id="episode-a",
            text="active",
            locale="sv",
        )
        self.assertTrue(started.wait(1))
        runtime.offer(
            episode_id="episode-a",
            text="pending",
            locale="sv",
        )

        runtime.cancel_episode("episode-a")

        self.assertTrue(finished.wait(1))
        cancelled = [
            event
            for event in events
            if event["event"] == "speech_cancelled"
        ]
        self.assertEqual(
            sorted(event["sequence"] for event in cancelled),
            [1, 2],
        )

    def test_speaker_failure_is_reported_and_worker_continues(self):
        failed = threading.Event()
        completed = threading.Event()
        calls = []
        events = []

        def speaker(text, _locale, _cancel_event):
            calls.append(text)
            if text == "bad":
                raise RuntimeError("audio failed")

        def event_sink(event):
            events.append(dict(event))
            if event["event"] == "speech_failed":
                failed.set()
            if event["event"] == "speech_completed":
                completed.set()

        runtime = RobotSpeechRuntime(
            speaker=speaker,
            event_sink=event_sink,
        )
        self.addCleanup(runtime.close, timeout_seconds=0.2)
        runtime.start()
        runtime.offer(
            episode_id="episode-a",
            text="bad",
            locale="sv",
        )
        self.assertTrue(failed.wait(1))
        runtime.offer(
            episode_id="episode-a",
            text="good",
            locale="sv",
        )
        self.assertTrue(completed.wait(1))
        self.assertEqual(calls, ["bad", "good"])

    def test_input_and_lifecycle_are_bounded(self):
        runtime = RobotSpeechRuntime(
            speaker=lambda *_args: None,
        )
        with self.assertRaises(RobotSpeechRuntimeError):
            runtime.offer(
                episode_id="episode-a",
                text="not started",
                locale="sv",
            )
        runtime.start()
        with self.assertRaises(RobotSpeechRuntimeError):
            runtime.offer(
                episode_id="episode-a",
                text="hello",
                locale="de",
            )
        with self.assertRaises(RobotSpeechRuntimeError):
            runtime.offer(
                episode_id="../episode",
                text="hello",
                locale="en",
            )
        with self.assertRaises(RobotSpeechRuntimeError):
            runtime.offer(
                episode_id="episode-a",
                text="x" * 161,
                locale="en",
            )
        self.assertTrue(runtime.close(timeout_seconds=1))
        with self.assertRaises(RobotSpeechRuntimeError):
            runtime.offer(
                episode_id="episode-a",
                text="closed",
                locale="en",
            )

    def test_ev3_speaker_maps_locale_to_fixed_voice(self):
        class Transport:
            def __init__(self):
                self.calls = []

            def speak(self, text, voice="sv"):
                self.calls.append((text, voice))
                return {"status": "completed"}

        transport = Transport()
        speaker = ev3_ssh_speaker(transport)
        speaker("Hej", "sv", threading.Event())
        speaker("Hello", "en", threading.Event())
        self.assertEqual(
            transport.calls,
            [("Hej", "sv"), ("Hello", "en")],
        )


if __name__ == "__main__":
    unittest.main()
