import threading
import time
import unittest

from robot_agent.robot_speech_runtime import RobotSpeechRuntime
from robot_agent.robot_turn_speech import (
    RobotTurnSpeechSink,
    TURN_SPEECH_EPISODE_ID,
    cancellable_serialized_speaker,
)


class FakeRuntime:
    def __init__(self):
        self.started = 0
        self.offers = []
        self.cancelled = []
        self.closes = []

    def start(self):
        self.started += 1

    def offer(self, **values):
        self.offers.append(values)
        return len(self.offers)

    def cancel_episode(self, episode_id):
        self.cancelled.append(episode_id)

    def close(self, **values):
        self.closes.append(values)
        return True


class RobotTurnSpeechSinkTests(unittest.TestCase):
    def test_maps_untrusted_request_ids_and_forwards_without_blocking(self):
        runtime = FakeRuntime()
        events = []
        factory_calls = []

        def factory(*, event_sink):
            factory_calls.append(event_sink)
            return runtime

        sink = RobotTurnSpeechSink(factory, event_sink=events.append)

        self.assertTrue(sink.submit("röstfråga / 17?", "Jaha.", "sv"))
        self.assertEqual(runtime.started, 1)
        self.assertEqual(factory_calls, [events.append])
        self.assertEqual(
            runtime.offers[0]["episode_id"],
            TURN_SPEECH_EPISODE_ID,
        )
        self.assertEqual(runtime.offers[0]["progress_revision"], 1)
        self.assertEqual(runtime.offers[0]["text"], "Jaha.")
        self.assertEqual(runtime.offers[0]["locale"], "sv")
        self.assertTrue(sink.close(drain=True, timeout_seconds=2.0))
        self.assertEqual(
            runtime.closes,
            [{"drain": True, "timeout_seconds": 2.0}],
        )
        self.assertFalse(sink.submit("later", "Nej.", "sv"))

    def test_invalid_request_identity_is_rejected_without_queueing(self):
        runtime = FakeRuntime()
        sink = RobotTurnSpeechSink(
            lambda **_kwargs: runtime,
        )
        self.addCleanup(sink.close, drain=False)

        for request_id in (
            "",
            " leading",
            "x" * 129,
            "bad\nrequest",
            "bad\ud800request",
        ):
            with self.subTest(request_id=request_id):
                self.assertFalse(sink.submit(request_id, "Hej.", "sv"))
        self.assertEqual(runtime.offers, [])

    def test_unsupported_profile_locale_is_not_reported_as_queued(self):
        runtime = FakeRuntime()
        sink = RobotTurnSpeechSink(
            lambda **_kwargs: runtime,
            supported_locales=("sv",),
        )
        self.addCleanup(sink.close, drain=False)

        self.assertFalse(sink.submit("english-turn", "Hello.", "en"))
        self.assertEqual(runtime.offers, [])

    def test_distinct_turns_reuse_one_bounded_runtime_episode(self):
        runtime = FakeRuntime()
        sink = RobotTurnSpeechSink(lambda **_kwargs: runtime)
        self.addCleanup(sink.close, drain=False)

        for index in range(100):
            self.assertTrue(
                sink.submit("request-{}".format(index), "Svar {}".format(index), "sv")
            )

        self.assertEqual(
            {offer["episode_id"] for offer in runtime.offers},
            {TURN_SPEECH_EPISODE_ID},
        )
        self.assertEqual(
            [offer["progress_revision"] for offer in runtime.offers],
            list(range(1, 101)),
        )

    def test_reuses_latest_pending_behavior_of_speech_runtime(self):
        first_started = threading.Event()
        release_first = threading.Event()
        third_completed = threading.Event()
        spoken = []

        def speaker(text, _locale, _cancel_event):
            spoken.append(text)
            if text == "first":
                first_started.set()
                release_first.wait(1)

        def event_sink(event):
            if (
                event["event"] == "speech_completed"
                and event["sequence"] == 3
            ):
                third_completed.set()

        sink = RobotTurnSpeechSink(
            lambda **options: RobotSpeechRuntime(
                speaker=speaker,
                **options,
            ),
            event_sink=event_sink,
        )
        self.addCleanup(sink.close, drain=False, timeout_seconds=0.2)

        self.assertTrue(sink.submit("request-1", "first", "sv"))
        self.assertTrue(first_started.wait(1))
        started = time.monotonic()
        self.assertTrue(sink.submit("request-2", "second", "sv"))
        self.assertTrue(sink.submit("request-3", "third", "sv"))
        self.assertLess(time.monotonic() - started, 0.1)
        release_first.set()
        self.assertTrue(third_completed.wait(1))
        self.assertEqual(spoken, ["first", "third"])


class SerializedSpeakerTests(unittest.TestCase):
    def test_waiting_speaker_honors_cancellation(self):
        lock = threading.Lock()
        first_started = threading.Event()
        release_first = threading.Event()
        calls = []

        def first_speaker(text, _locale, _cancel_event):
            calls.append(text)
            first_started.set()
            release_first.wait(1)

        def second_speaker(text, _locale, _cancel_event):
            calls.append(text)

        first = cancellable_serialized_speaker(first_speaker, lock)
        second = cancellable_serialized_speaker(second_speaker, lock)
        first_thread = threading.Thread(
            target=first,
            args=("first", "sv", threading.Event()),
        )
        first_thread.start()
        self.assertTrue(first_started.wait(1))

        cancelled = threading.Event()
        result = []
        second_thread = threading.Thread(
            target=lambda: result.append(second("second", "sv", cancelled)),
        )
        second_thread.start()
        time.sleep(0.06)
        self.assertEqual(calls, ["first"])
        cancelled.set()
        second_thread.join(0.3)
        self.assertFalse(second_thread.is_alive())
        self.assertEqual(result, [None])
        self.assertEqual(calls, ["first"])

        release_first.set()
        first_thread.join(1)
        self.assertFalse(first_thread.is_alive())


if __name__ == "__main__":
    unittest.main()
