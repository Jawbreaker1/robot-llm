import subprocess
import threading
import unittest

from robot_agent.robot_speech_runtime import (
    RobotSpeechRuntime,
    RobotSpeechRuntimeError,
    ev3_ssh_speaker,
)


class RobotSpeechRuntimeTests(unittest.TestCase):
    def test_normalized_duplicates_wait_for_real_progress(self):
        first_started = threading.Event()
        release_first = threading.Event()
        repeated_completed = threading.Event()
        spoken = []
        events = []

        def speaker(text, _locale, _cancel_event):
            spoken.append(text)
            if len(spoken) == 1:
                first_started.set()
                release_first.wait(1)

        def event_sink(event):
            events.append(dict(event))
            if (
                event["event"] == "speech_completed"
                and event["sequence"] == 9
            ):
                repeated_completed.set()

        runtime = RobotSpeechRuntime(
            speaker=speaker,
            event_sink=event_sink,
        )
        self.addCleanup(runtime.close, timeout_seconds=0.2)
        runtime.start()

        variants = (
            "Café status: READY!",
            "CAFÉ STATUS READY",
            "cafe\u0301\tstatus—ready.",
            "Café  status / ready",
            "café status; READY?",
            "CAFÉ\nSTATUS READY",
            "Café (status) ready",
            "cafe\u0301 status... ready",
        )
        for sequence, text in enumerate(variants, start=1):
            self.assertEqual(
                runtime.offer(
                    episode_id="episode-a",
                    text=text,
                    locale="en",
                    progress_revision=4,
                ),
                sequence,
            )
            if sequence == 1:
                self.assertTrue(first_started.wait(1))

        release_first.set()
        runtime.offer(
            episode_id="episode-a",
            text="CAFÉ STATUS READY",
            locale="en",
            progress_revision=5,
        )
        self.assertTrue(repeated_completed.wait(1))

        self.assertEqual(
            spoken,
            ["Café status: READY!", "CAFÉ STATUS READY"],
        )
        duplicates = [
            event
            for event in events
            if event["event"] == "speech_dropped"
            and event.get("reason") == "duplicate_without_progress"
        ]
        self.assertEqual(
            [event["sequence"] for event in duplicates],
            list(range(2, 9)),
        )
        self.assertFalse(any(
            event["event"] == "speech_queued"
            and 2 <= event["sequence"] <= 8
            for event in events
        ))

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

    def test_speaker_failure_surfaces_specific_error_code(self):
        failed = threading.Event()
        events = []

        def speaker(_text, _locale, _cancel_event):
            error = RobotSpeechRuntimeError(
                "speech_playback_failed",
                "private detail",
            )
            raise error

        def event_sink(event):
            events.append(dict(event))
            if event["event"] == "speech_failed":
                failed.set()

        runtime = RobotSpeechRuntime(
            speaker=speaker,
            event_sink=event_sink,
        )
        self.addCleanup(runtime.close, timeout_seconds=0.2)
        runtime.start()
        runtime.offer(
            episode_id="episode-a",
            text="Hej",
            locale="sv",
        )

        self.assertTrue(failed.wait(1))
        event = next(
            event for event in events if event["event"] == "speech_failed"
        )
        self.assertEqual(event["reason"], "speech_playback_failed")
        self.assertNotIn("private", str(event))

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

    def test_cancelled_episode_cannot_race_a_new_speech_offer(self):
        events = []
        runtime = RobotSpeechRuntime(
            speaker=lambda *_args: None,
            event_sink=lambda event: events.append(dict(event)),
        )
        self.addCleanup(runtime.close, timeout_seconds=0.2)
        runtime.start()
        cancelled = threading.Event()
        cancelled.set()

        with self.assertRaises(RobotSpeechRuntimeError) as caught:
            runtime.offer(
                episode_id="episode-a",
                text="Det här ska inte sägas.",
                locale="sv",
                cancel_requested=cancelled.is_set,
            )

        self.assertEqual(caught.exception.code, "speech_episode_cancelled")
        self.assertFalse(
            any(event["event"] == "speech_queued" for event in events)
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

    def test_ev3_ssh_process_is_terminated_when_speech_is_cancelled(self):
        communication_started = threading.Event()

        class Transport:
            _speech_timeout_seconds = 5

            def __init__(self):
                self.remote_arguments = None
                self.fallback_calls = []

            def _argv(self, remote_arguments):
                self.remote_arguments = list(remote_arguments)
                return ["ssh", "robot@ev3dev.local"] + list(
                    remote_arguments
                )

            def speak(self, text, voice="sv"):
                self.fallback_calls.append((text, voice))
                raise AssertionError("blocking fallback must not be used")

        class Process:
            def __init__(self):
                self.returncode = None
                self.inputs = []
                self.terminated = False
                self.killed = False

            def communicate(self, input=None, timeout=None):
                self.inputs.append(input)
                communication_started.set()
                raise subprocess.TimeoutExpired("ssh", timeout)

            def terminate(self):
                self.terminated = True
                self.returncode = -15

            def wait(self, timeout=None):
                return self.returncode

            def kill(self):
                self.killed = True
                self.returncode = -9

        transport = Transport()
        process = Process()
        speaker = ev3_ssh_speaker(
            transport,
            popen_factory=lambda *_args, **_kwargs: process,
            poll_seconds=0.01,
        )
        cancelled = threading.Event()
        failures = []

        def invoke():
            try:
                speaker("Stopp nu", "sv", cancelled)
            except BaseException as error:
                failures.append(error)

        thread = threading.Thread(target=invoke, daemon=True)
        thread.start()
        self.assertTrue(communication_started.wait(1))
        cancelled.set()
        thread.join(1)

        self.assertFalse(thread.is_alive())
        self.assertEqual(failures, [])
        self.assertTrue(process.terminated)
        self.assertFalse(process.killed)
        self.assertEqual(process.inputs[0], "Stopp nu\n")
        self.assertTrue(all(value is None for value in process.inputs[1:]))
        self.assertEqual(transport.fallback_calls, [])
        self.assertEqual(
            transport.remote_arguments[-3:],
            ["speak-stdin", "--voice", "sv"],
        )


if __name__ == "__main__":
    unittest.main()
