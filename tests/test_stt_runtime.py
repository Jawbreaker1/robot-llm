import gc
import json
import threading
import time
import unittest
import weakref

from robot_agent.stt_contract import (
    PCM16Wav,
    ProviderTranscription,
    STTContractError,
    TranscriptionRequest,
    validate_pcm16_wav,
)
from robot_agent.stt_provider import (
    STTProviderProtocolError,
    STTProviderTimeoutError,
    STTProviderUnavailableError,
)
from robot_agent.stt_runtime import (
    AUTO_DELIVERY_TTL_MS,
    STTRuntime,
    STTRuntimeError,
)

from test_stt_contract import canonical_wav


class IncrementingClock:
    def __init__(self, value, step=10):
        self._value = value
        self._step = step
        self._lock = threading.Lock()

    def __call__(self):
        with self._lock:
            result = self._value
            self._value += self._step
            return result


class ManualClock:
    def __init__(self, value):
        self._value = value
        self._lock = threading.Lock()

    def __call__(self):
        with self._lock:
            return self._value

    def advance(self, milliseconds):
        with self._lock:
            self._value += milliseconds


class RecordingProvider:
    provider_id = "fixture"
    model_id = "fixture-v1"

    def __init__(self, result=None, error=None):
        self.result = result or ProviderTranscription(
            text="Vinka två gånger.",
            provider_id=self.provider_id,
            model_id=self.model_id,
            detected_language="sv",
            provider_score=0.75,
        )
        self.error = error
        self.calls = []
        self._lock = threading.Lock()

    def transcribe(self, request):
        with self._lock:
            self.calls.append(request)
        if self.error is not None:
            raise self.error
        return self.result


class BlockingProvider(RecordingProvider):
    def __init__(self):
        super().__init__()
        self.started = threading.Event()
        self.release = threading.Event()

    def transcribe(self, request):
        with self._lock:
            self.calls.append(request)
        self.started.set()
        if not self.release.wait(5):
            raise TimeoutError("test provider was not released")
        return self.result


class WeakRequestProvider(RecordingProvider):
    def __init__(self):
        super().__init__()
        self.request_ref = None

    def transcribe(self, request):
        self.request_ref = weakref.ref(request)
        return self.result


class WeakResultProvider:
    provider_id = "fixture"
    model_id = "fixture-v1"

    def __init__(self):
        self.result_ref = None

    def transcribe(self, _request):
        result = ProviderTranscription(
            text="Kortlivat transkript.",
            provider_id=self.provider_id,
            model_id=self.model_id,
        )
        self.result_ref = weakref.ref(result)
        return result


def transcription_request(
    request_id="voice-1",
    language="sv",
    sample=0,
):
    return TranscriptionRequest(
        request_id=request_id,
        language_hint=language,
        audio=validate_pcm16_wav(
            canonical_wav(duration_ms=250, sample=sample)
        ),
    )


def wait_for_status(runtime, transcription_id, expected, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        view = runtime.get(transcription_id)
        if view["status"] in expected:
            return view
        time.sleep(0.001)
    raise AssertionError(
        "transcription did not reach {}; latest={!r}".format(
            expected,
            runtime.get(transcription_id),
        )
    )


def wait_for_view(runtime, transcription_id, predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        view = runtime.get(transcription_id)
        if predicate(view):
            return view
        time.sleep(0.001)
    raise AssertionError(
        "transcription view did not satisfy predicate; latest={!r}".format(
            runtime.get(transcription_id)
        )
    )


class STTRuntimeSuccessTests(unittest.TestCase):
    def make_runtime(self, provider=None, **kwargs):
        provider = provider or RecordingProvider()
        runtime = STTRuntime(provider, **kwargs)
        self.addCleanup(runtime.shutdown)
        return runtime, provider

    def test_completes_with_latency_and_discards_audio(self):
        events = []
        runtime, provider = self.make_runtime(
            unix_clock_ms=IncrementingClock(1_700_000_000_000),
            monotonic_clock_ms=IncrementingClock(100),
            id_factory=lambda: "job-one",
            event_sink=lambda event_type, data: events.append(
                (event_type, dict(data))
            ),
        )
        source = transcription_request()

        queued = runtime.submit(source)
        completed = wait_for_status(
            runtime,
            queued["transcription_id"],
            {"completed"},
        )

        self.assertEqual(queued["status"], "queued")
        self.assertTrue(queued["audio"]["retained"])
        self.assertEqual(completed["schema"], "speech-transcription/v1")
        self.assertEqual(completed["transcription_id"], "stt-job-one")
        self.assertEqual(completed["request_id"], "voice-1")
        self.assertEqual(completed["status"], "completed")
        self.assertEqual(completed["requested_language"], "sv")
        self.assertEqual(completed["text"], "Vinka två gånger.")
        self.assertEqual(completed["detected_language"], "sv")
        self.assertEqual(completed["provider_score"], 0.75)
        self.assertFalse(completed["audio"]["retained"])
        self.assertEqual(
            completed["provider"],
            {
                "provider_id": "fixture",
                "model_id": "fixture-v1",
            },
        )
        self.assertGreaterEqual(
            completed["timing"]["queue_wait_ms"],
            0,
        )
        self.assertGreaterEqual(
            completed["timing"]["provider_latency_ms"],
            0,
        )
        self.assertEqual(
            completed["timing"]["total_latency_ms"],
            completed["timing"]["queue_wait_ms"]
            + completed["timing"]["provider_latency_ms"],
        )
        self.assertEqual(
            completed["valid_until_unix_ms"],
            completed["timing"]["completed_at_unix_ms"]
            + AUTO_DELIVERY_TTL_MS,
        )
        self.assertEqual(len(provider.calls), 1)
        self.assertEqual(
            provider.calls[0].audio.wav_bytes,
            source.audio.wav_bytes,
        )
        self.assertEqual(
            [event_type for event_type, _data in events],
            [
                "stt.transcription_queued",
                "stt.transcription_started",
                "stt.transcription_completed",
            ],
        )

        public_and_events = json.dumps(
            {"view": completed, "events": events},
            ensure_ascii=False,
        )
        self.assertNotIn(source.audio.sha256, public_and_events)
        self.assertNotIn("wav_bytes", public_and_events)
        self.assertNotIn("Vinka två gånger.", json.dumps(events))

    def test_event_sink_failure_never_changes_job_outcome(self):
        def broken_sink(_event_type, _data):
            raise RuntimeError("observability failed")

        runtime, _provider = self.make_runtime(
            event_sink=broken_sink,
        )
        submitted = runtime.submit(transcription_request())

        completed = wait_for_status(
            runtime,
            submitted["transcription_id"],
            {"completed"},
        )

        self.assertEqual(completed["status"], "completed")

    def test_get_validates_identifier_and_hides_unknown_jobs(self):
        runtime, _provider = self.make_runtime()
        for value in ("", "bad/id", "has space", "å", None):
            with self.subTest(value=value):
                with self.assertRaises(STTRuntimeError) as raised:
                    runtime.get(value)
                self.assertEqual(raised.exception.status, 400)
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_identifier",
                )

        with self.assertRaises(STTRuntimeError) as raised:
            runtime.get("stt-does-not-exist")
        self.assertEqual(raised.exception.status, 404)
        self.assertEqual(raised.exception.code, "stt_not_found")


class STTRuntimeTrustAndExpiryTests(unittest.TestCase):
    def test_submit_revalidates_wav_bytes_and_all_derived_metadata(self):
        trusted = validate_pcm16_wav(canonical_wav())
        malformed_bytes = b"RIFX" + trusted.wav_bytes[4:]
        forged_values = (
            PCM16Wav(
                wav_bytes=malformed_bytes,
                duration_ms=trusted.duration_ms,
                sample_count=trusted.sample_count,
                sha256=trusted.sha256,
            ),
            PCM16Wav(
                wav_bytes=trusted.wav_bytes,
                duration_ms=trusted.duration_ms + 1,
                sample_count=trusted.sample_count,
                sha256=trusted.sha256,
            ),
            PCM16Wav(
                wav_bytes=trusted.wav_bytes,
                duration_ms=trusted.duration_ms,
                sample_count=trusted.sample_count + 1,
                sha256=trusted.sha256,
            ),
            PCM16Wav(
                wav_bytes=trusted.wav_bytes,
                duration_ms=trusted.duration_ms,
                sample_count=trusted.sample_count,
                sha256="0" * 64,
            ),
        )
        runtime = STTRuntime(RecordingProvider())
        self.addCleanup(runtime.shutdown)

        for index, forged in enumerate(forged_values):
            with self.subTest(index=index):
                request = TranscriptionRequest(
                    request_id="forged-{}".format(index),
                    language_hint="sv",
                    audio=forged,
                )
                with self.assertRaises(STTRuntimeError) as raised:
                    runtime.submit(request)
                self.assertEqual(raised.exception.status, 400)
                self.assertIn(
                    raised.exception.code,
                    (
                        "invalid_stt_wav",
                        "invalid_stt_audio_metadata",
                    ),
                )
        self.assertEqual(runtime._jobs, {})
        self.assertEqual(runtime._request_index, {})

        mutated = transcription_request("mutated")
        object.__setattr__(mutated, "audio", object())
        with self.assertRaises(STTRuntimeError) as raised:
            runtime.submit(mutated)
        self.assertEqual(raised.exception.status, 400)
        self.assertEqual(raised.exception.code, "invalid_stt_audio")

    def test_terminal_ttl_uses_monotonic_time_and_redacts_result(self):
        unix_clock = ManualClock(10_000)
        monotonic_clock = ManualClock(500)
        events = []
        source = transcription_request()
        runtime = STTRuntime(
            RecordingProvider(),
            terminal_ttl_ms=50,
            unix_clock_ms=unix_clock,
            monotonic_clock_ms=monotonic_clock,
            event_sink=lambda kind, data: events.append(
                (kind, dict(data))
            ),
        )
        self.addCleanup(runtime.shutdown)
        submitted = runtime.submit(source)
        completed = wait_for_status(
            runtime,
            submitted["transcription_id"],
            {"completed"},
        )
        self.assertEqual(completed["valid_until_unix_ms"], 10_050)

        unix_clock.advance(10_000_000)
        monotonic_clock.advance(49)
        still_fresh = runtime.get(submitted["transcription_id"])
        self.assertEqual(still_fresh["text"], "Vinka två gånger.")

        monotonic_clock.advance(1)
        with self.assertRaises(STTRuntimeError) as expired:
            runtime.get(submitted["transcription_id"])
        self.assertEqual(expired.exception.status, 410)
        self.assertEqual(expired.exception.code, "stt_expired")
        with self.assertRaises(STTRuntimeError) as replay:
            runtime.submit(source)
        self.assertEqual(replay.exception.status, 410)
        self.assertEqual(replay.exception.code, "stt_expired")

        job = runtime._jobs[submitted["transcription_id"]]
        self.assertEqual(job.status, "expired")
        self.assertIsNone(job.result)
        self.assertEqual(job.audio_bytes, b"")
        self.assertIsNone(job.audio_sha256)
        deadline = time.monotonic() + 1
        while not any(
            kind == "stt.transcription_expired"
            for kind, _data in events
        ):
            if time.monotonic() >= deadline:
                self.fail("expiry event was not dispatched")
            time.sleep(0.001)
        serialized_events = json.dumps(events, ensure_ascii=False)
        self.assertNotIn("Vinka två gånger.", serialized_events)
        self.assertNotIn(source.audio.sha256, serialized_events)
        self.assertNotIn("wav_bytes", serialized_events)

    def test_invalid_terminal_ttl_is_rejected(self):
        for value in (True, 0, -1, 86_400_001, "30000"):
            with self.subTest(value=value):
                with self.assertRaises(STTRuntimeError) as raised:
                    STTRuntime(
                        RecordingProvider(),
                        terminal_ttl_ms=value,
                    )
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_configuration",
                )


class STTRuntimeCancellationTests(unittest.TestCase):
    def test_request_cancel_tombstone_wins_race_with_submission(self):
        monotonic_clock = ManualClock(100)
        provider = RecordingProvider()
        runtime = STTRuntime(
            provider,
            terminal_ttl_ms=10,
            monotonic_clock_ms=monotonic_clock,
        )
        self.addCleanup(runtime.shutdown)

        cancelled = runtime.cancel_request("race-request")

        self.assertEqual(cancelled["status"], "cancelled")
        self.assertIsNone(cancelled["transcription_id"])
        with self.assertRaises(STTRuntimeError) as rejected:
            runtime.submit(transcription_request("race-request"))
        self.assertEqual(rejected.exception.status, 409)
        self.assertEqual(
            rejected.exception.code,
            "stt_request_cancelled",
        )
        self.assertEqual(provider.calls, [])

        monotonic_clock.advance(10)
        accepted = runtime.submit(
            transcription_request("race-request")
        )
        wait_for_status(
            runtime,
            accepted["transcription_id"],
            {"completed"},
        )
        self.assertEqual(len(provider.calls), 1)

    def test_repeated_cancel_keeps_tombstones_in_expiry_order(self):
        monotonic_clock = ManualClock(100)
        provider = RecordingProvider()
        runtime = STTRuntime(
            provider,
            terminal_ttl_ms=10,
            monotonic_clock_ms=monotonic_clock,
        )
        self.addCleanup(runtime.shutdown)

        runtime.cancel_request("request-a")
        monotonic_clock.advance(2)
        runtime.cancel_request("request-b")
        monotonic_clock.advance(2)
        runtime.cancel_request("request-a")
        monotonic_clock.advance(9)

        accepted = runtime.submit(
            transcription_request("request-b")
        )

        wait_for_status(
            runtime,
            accepted["transcription_id"],
            {"completed"},
        )
        self.assertEqual(len(provider.calls), 1)
        with self.assertRaises(STTRuntimeError) as rejected:
            runtime.submit(transcription_request("request-a"))
        self.assertEqual(
            rejected.exception.code,
            "stt_request_cancelled",
        )

    def test_request_cancel_redacts_already_completed_transcript(self):
        runtime = STTRuntime(RecordingProvider())
        self.addCleanup(runtime.shutdown)
        submitted = runtime.submit(
            transcription_request("completed-request")
        )
        completed = wait_for_status(
            runtime,
            submitted["transcription_id"],
            {"completed"},
        )
        self.assertIn("text", completed)

        cancelled = runtime.cancel_request("completed-request")

        self.assertEqual(cancelled["status"], "cancelled")
        self.assertNotIn("text", cancelled)
        self.assertFalse(cancelled["audio"]["retained"])

    def test_cancelled_queued_job_immediately_frees_queue_capacity(self):
        provider = BlockingProvider()
        runtime = STTRuntime(
            provider,
            queue_capacity=1,
            job_capacity=4,
        )
        self.addCleanup(runtime.shutdown)
        self.addCleanup(provider.release.set)
        running = runtime.submit(transcription_request("running"))
        self.assertTrue(provider.started.wait(1))
        queued = runtime.submit(transcription_request("queued"))

        runtime.cancel(queued["transcription_id"])
        replacement = runtime.submit(
            transcription_request("replacement")
        )

        self.assertEqual(replacement["status"], "queued")
        self.assertTrue(replacement["audio"]["retained"])
        provider.release.set()
        wait_for_status(
            runtime,
            running["transcription_id"],
            {"completed"},
        )
        wait_for_status(
            runtime,
            replacement["transcription_id"],
            {"completed"},
        )
        self.assertEqual(len(provider.calls), 2)

    def test_cancel_queued_and_running_discards_late_provider_result(self):
        provider = BlockingProvider()
        events = []
        runtime = STTRuntime(
            provider,
            queue_capacity=1,
            job_capacity=4,
            event_sink=lambda kind, data: events.append(
                (kind, dict(data))
            ),
        )
        self.addCleanup(runtime.shutdown)
        self.addCleanup(provider.release.set)
        running = runtime.submit(
            transcription_request("running-secret", sample=111)
        )
        self.assertTrue(provider.started.wait(1))
        queued = runtime.submit(
            transcription_request("queued-secret", sample=222)
        )

        queued_cancelled = runtime.cancel(
            queued["transcription_id"]
        )
        running_cancelled = runtime.cancel(
            running["transcription_id"]
        )
        repeated = runtime.cancel(running["transcription_id"])

        self.assertEqual(queued_cancelled["status"], "cancelled")
        self.assertFalse(queued_cancelled["provider_work_pending"])
        self.assertFalse(queued_cancelled["audio"]["retained"])
        self.assertEqual(running_cancelled["status"], "cancelled")
        self.assertTrue(running_cancelled["provider_work_pending"])
        self.assertTrue(running_cancelled["audio"]["retained"])
        self.assertEqual(repeated, running_cancelled)
        self.assertEqual(len(provider.calls), 1)

        provider.release.set()
        late = wait_for_view(
            runtime,
            running["transcription_id"],
            lambda view: view["late_provider_result_discarded"],
        )
        self.assertEqual(late["status"], "cancelled")
        self.assertFalse(late["provider_work_pending"])
        self.assertFalse(late["audio"]["retained"])
        self.assertNotIn("text", late)
        time.sleep(0.01)
        self.assertEqual(len(provider.calls), 1)
        self.assertEqual(
            runtime.get(queued["transcription_id"])["status"],
            "cancelled",
        )

        deadline = time.monotonic() + 1
        expected_types = (
            "stt.transcription_queued",
            "stt.transcription_started",
            "stt.transcription_queued",
            "stt.transcription_cancelled",
            "stt.transcription_cancelled",
            "stt.transcription_late_result_discarded",
        )
        while len(events) < len(expected_types):
            if time.monotonic() >= deadline:
                self.fail("cancel events were not dispatched")
            time.sleep(0.001)
        self.assertEqual(
            tuple(kind for kind, _data in events[:6]),
            expected_types,
        )
        serialized_events = json.dumps(events, ensure_ascii=False)
        self.assertNotIn(provider.result.text, serialized_events)
        for call in provider.calls:
            self.assertNotIn(call.audio.sha256, serialized_events)
        self.assertNotIn("wav_bytes", serialized_events)
        self.assertNotIn("audio_sha256", serialized_events)

    def test_cancel_rejects_invalid_missing_completed_and_expired_jobs(self):
        monotonic_clock = ManualClock(100)
        runtime = STTRuntime(
            RecordingProvider(),
            terminal_ttl_ms=5,
            monotonic_clock_ms=monotonic_clock,
        )
        self.addCleanup(runtime.shutdown)
        for value, status, code in (
            ("bad/id", 400, "invalid_stt_identifier"),
            ("stt-missing", 404, "stt_not_found"),
        ):
            with self.subTest(value=value):
                with self.assertRaises(STTRuntimeError) as raised:
                    runtime.cancel(value)
                self.assertEqual(raised.exception.status, status)
                self.assertEqual(raised.exception.code, code)

        submitted = runtime.submit(transcription_request())
        wait_for_status(
            runtime,
            submitted["transcription_id"],
            {"completed"},
        )
        with self.assertRaises(STTRuntimeError) as completed:
            runtime.cancel(submitted["transcription_id"])
        self.assertEqual(completed.exception.status, 409)
        self.assertEqual(
            completed.exception.code,
            "stt_not_cancellable",
        )

        monotonic_clock.advance(5)
        with self.assertRaises(STTRuntimeError) as expired:
            runtime.cancel(submitted["transcription_id"])
        self.assertEqual(expired.exception.status, 410)
        self.assertEqual(expired.exception.code, "stt_expired")

    def test_event_sink_runs_outside_core_lock(self):
        holder = {}
        observed = threading.Event()
        failures = []

        def sink(event_type, data):
            if event_type != "stt.transcription_queued":
                return
            try:
                holder["runtime"].get(data["transcription_id"])
            except Exception as error:
                failures.append(error)
            observed.set()

        runtime = STTRuntime(
            RecordingProvider(),
            event_sink=sink,
        )
        holder["runtime"] = runtime
        self.addCleanup(runtime.shutdown)

        submitted = runtime.submit(transcription_request())

        self.assertTrue(observed.wait(1))
        self.assertEqual(failures, [])
        wait_for_status(
            runtime,
            submitted["transcription_id"],
            {"completed"},
        )


class STTRuntimeIdempotencyAndBoundTests(unittest.TestCase):
    def make_blocking_runtime(self, **kwargs):
        provider = BlockingProvider()
        runtime = STTRuntime(provider, **kwargs)
        self.addCleanup(runtime.shutdown)
        self.addCleanup(provider.release.set)
        return runtime, provider

    def test_same_request_is_idempotent_but_changed_input_conflicts(self):
        runtime, provider = self.make_blocking_runtime(
            id_factory=lambda: "stable",
        )
        original = transcription_request()
        first = runtime.submit(original)
        self.assertTrue(provider.started.wait(1))

        repeated = runtime.submit(original)

        self.assertEqual(
            repeated["transcription_id"],
            first["transcription_id"],
        )
        self.assertEqual(repeated["status"], "running")
        self.assertEqual(len(provider.calls), 1)

        for changed in (
            transcription_request(sample=1),
            transcription_request(language="en"),
        ):
            with self.subTest(
                language=changed.language_hint,
                sha=changed.audio.sha256,
            ):
                with self.assertRaises(STTRuntimeError) as raised:
                    runtime.submit(changed)
                self.assertEqual(raised.exception.status, 409)
                self.assertEqual(
                    raised.exception.code,
                    "stt_idempotency_conflict",
                )

        provider.release.set()
        wait_for_status(
            runtime,
            first["transcription_id"],
            {"completed"},
        )
        self.assertEqual(len(provider.calls), 1)

    def test_idle_worker_does_not_retain_completed_audio_request(self):
        provider = WeakRequestProvider()
        runtime = STTRuntime(provider)
        self.addCleanup(runtime.shutdown)
        submitted = runtime.submit(transcription_request())

        wait_for_status(
            runtime,
            submitted["transcription_id"],
            {"completed"},
        )
        gc.collect()

        self.assertIsNotNone(provider.request_ref)
        self.assertIsNone(provider.request_ref())

    def test_idle_worker_does_not_retain_transcript_after_ttl(self):
        monotonic_clock = ManualClock(100)
        provider = WeakResultProvider()
        runtime = STTRuntime(
            provider,
            terminal_ttl_ms=1,
            monotonic_clock_ms=monotonic_clock,
        )
        self.addCleanup(runtime.shutdown)
        submitted = runtime.submit(transcription_request())
        wait_for_status(
            runtime,
            submitted["transcription_id"],
            {"completed"},
        )

        monotonic_clock.advance(1)
        with self.assertRaises(STTRuntimeError):
            runtime.get(submitted["transcription_id"])
        gc.collect()

        self.assertIsNotNone(provider.result_ref)
        self.assertIsNone(provider.result_ref())

    def test_queue_full_rejects_without_retaining_failed_submission(self):
        ids = iter(("one", "two", "three", "four"))
        runtime, provider = self.make_blocking_runtime(
            queue_capacity=1,
            job_capacity=8,
            id_factory=lambda: next(ids),
        )
        first = runtime.submit(transcription_request("request-1"))
        self.assertTrue(provider.started.wait(1))
        second = runtime.submit(transcription_request("request-2"))

        with self.assertRaises(STTRuntimeError) as raised:
            runtime.submit(transcription_request("request-3"))
        self.assertEqual(raised.exception.status, 429)
        self.assertEqual(raised.exception.code, "stt_queue_full")
        with self.assertRaises(STTRuntimeError) as missing:
            runtime.get("stt-three")
        self.assertEqual(missing.exception.code, "stt_not_found")

        provider.release.set()
        wait_for_status(
            runtime,
            first["transcription_id"],
            {"completed"},
        )
        wait_for_status(
            runtime,
            second["transcription_id"],
            {"completed"},
        )

        retried = runtime.submit(transcription_request("request-3"))
        self.assertEqual(retried["transcription_id"], "stt-four")
        wait_for_status(runtime, "stt-four", {"completed"})

    def test_job_store_full_fails_before_creating_untracked_job(self):
        runtime, provider = self.make_blocking_runtime(
            queue_capacity=1,
            job_capacity=2,
        )
        first = runtime.submit(transcription_request("request-1"))
        self.assertTrue(provider.started.wait(1))
        runtime.submit(transcription_request("request-2"))

        with self.assertRaises(STTRuntimeError) as raised:
            runtime.submit(transcription_request("request-3"))
        self.assertEqual(raised.exception.status, 429)
        self.assertEqual(raised.exception.code, "stt_job_store_full")

        provider.release.set()
        wait_for_status(
            runtime,
            first["transcription_id"],
            {"completed"},
        )

    def test_terminal_job_is_evicted_at_capacity(self):
        ids = iter(("first", "second"))
        runtime = STTRuntime(
            RecordingProvider(),
            queue_capacity=1,
            job_capacity=1,
            id_factory=lambda: next(ids),
        )
        self.addCleanup(runtime.shutdown)
        first = runtime.submit(transcription_request("old-request"))
        wait_for_status(runtime, first["transcription_id"], {"completed"})

        second = runtime.submit(transcription_request("new-request"))

        self.assertEqual(second["transcription_id"], "stt-second")
        with self.assertRaises(STTRuntimeError) as raised:
            runtime.get(first["transcription_id"])
        self.assertEqual(raised.exception.code, "stt_not_found")
        wait_for_status(runtime, second["transcription_id"], {"completed"})

    def test_duplicate_or_invalid_generated_ids_fail_closed(self):
        for generated, expected in (
            ("", "invalid_stt_id_factory"),
            ("bad/id", "invalid_stt_id_factory"),
            (None, "invalid_stt_id_factory"),
        ):
            with self.subTest(generated=generated):
                runtime = STTRuntime(
                    RecordingProvider(),
                    id_factory=lambda value=generated: value,
                )
                self.addCleanup(runtime.shutdown)
                with self.assertRaises(STTRuntimeError) as raised:
                    runtime.submit(transcription_request())
                self.assertEqual(raised.exception.code, expected)

        runtime = STTRuntime(
            RecordingProvider(),
            queue_capacity=1,
            job_capacity=2,
            id_factory=lambda: "duplicate",
        )
        self.addCleanup(runtime.shutdown)
        first = runtime.submit(transcription_request("request-1"))
        wait_for_status(runtime, first["transcription_id"], {"completed"})
        with self.assertRaises(STTRuntimeError) as raised:
            runtime.submit(transcription_request("request-2"))
        self.assertEqual(raised.exception.code, "duplicate_stt_id")


class STTRuntimeFailureAndShutdownTests(unittest.TestCase):
    def test_provider_failures_are_safely_classified_and_audio_dropped(self):
        cases = (
            (
                STTProviderTimeoutError("private"),
                "stt_provider_timeout",
            ),
            (
                STTProviderUnavailableError("private"),
                "stt_provider_unavailable",
            ),
            (
                STTProviderProtocolError("private"),
                "stt_provider_protocol",
            ),
            (
                STTContractError("private_code", "private"),
                "stt_provider_protocol",
            ),
            (RuntimeError("private"), "stt_provider_failed"),
        )
        for error, expected_code in cases:
            with self.subTest(error=type(error).__name__):
                events = []
                provider = RecordingProvider(error=error)
                runtime = STTRuntime(
                    provider,
                    event_sink=lambda kind, data: events.append(
                        (kind, dict(data))
                    ),
                )
                try:
                    submitted = runtime.submit(
                        transcription_request()
                    )
                    failed = wait_for_status(
                        runtime,
                        submitted["transcription_id"],
                        {"failed"},
                    )
                    self.assertEqual(
                        failed["error_code"],
                        expected_code,
                    )
                    self.assertFalse(failed["audio"]["retained"])
                    serialized = json.dumps(
                        {"view": failed, "events": events}
                    )
                    self.assertNotIn("private", serialized)
                    self.assertEqual(
                        events[-1][0],
                        "stt.transcription_failed",
                    )
                    self.assertEqual(
                        events[-1][1]["error_code"],
                        expected_code,
                    )
                finally:
                    runtime.shutdown()

    def test_invalid_provider_result_is_protocol_failure(self):
        provider = RecordingProvider(result=object())
        # RecordingProvider uses ``or`` for its fixture default.
        provider.result = object()
        runtime = STTRuntime(provider)
        self.addCleanup(runtime.shutdown)

        submitted = runtime.submit(transcription_request())
        failed = wait_for_status(
            runtime,
            submitted["transcription_id"],
            {"failed"},
        )

        self.assertEqual(
            failed["error_code"],
            "stt_provider_protocol",
        )
        self.assertFalse(failed["audio"]["retained"])

    def test_provider_cannot_spoof_runtime_identity(self):
        cases = (
            ProviderTranscription(
                text="Hello",
                provider_id="other-provider",
                model_id="fixture-v1",
            ),
            ProviderTranscription(
                text="Hello",
                provider_id="fixture",
                model_id="other-model",
            ),
        )
        for result in cases:
            with self.subTest(
                provider_id=result.provider_id,
                model_id=result.model_id,
            ):
                provider = RecordingProvider(result=result)
                runtime = STTRuntime(provider)
                try:
                    submitted = runtime.submit(
                        transcription_request()
                    )
                    failed = wait_for_status(
                        runtime,
                        submitted["transcription_id"],
                        {"failed"},
                    )
                    self.assertEqual(
                        failed["error_code"],
                        "stt_provider_protocol",
                    )
                    self.assertFalse(failed["audio"]["retained"])
                    self.assertNotIn("text", failed)
                finally:
                    runtime.shutdown()

    def test_shutdown_cancels_jobs_and_is_idempotent(self):
        provider = BlockingProvider()
        events = []
        runtime = STTRuntime(
            provider,
            queue_capacity=1,
            job_capacity=4,
            event_sink=lambda kind, data: events.append(
                (kind, dict(data))
            ),
        )
        self.addCleanup(runtime.shutdown)
        self.addCleanup(provider.release.set)
        running = runtime.submit(transcription_request("running"))
        self.assertTrue(provider.started.wait(1))
        queued = runtime.submit(transcription_request("queued"))

        first_shutdown = runtime.shutdown(timeout_seconds=0)

        self.assertTrue(first_shutdown["worker_alive"])
        self.assertTrue(first_shutdown["timed_out"])
        self.assertEqual(first_shutdown["queued_remaining"], 0)
        self.assertEqual(first_shutdown["provider_work_pending"], 1)
        for submitted, expected_retained in (
            (running, True),
            (queued, False),
        ):
            cancelled = runtime.get(submitted["transcription_id"])
            self.assertEqual(cancelled["status"], "cancelled")
            self.assertEqual(
                cancelled["error_code"],
                "stt_stopping",
            )
            self.assertEqual(
                cancelled["audio"]["retained"],
                expected_retained,
            )
            self.assertNotIn("text", cancelled)
        with self.assertRaises(STTRuntimeError) as raised:
            runtime.submit(transcription_request("after-stop"))
        self.assertEqual(raised.exception.status, 503)
        self.assertEqual(raised.exception.code, "stt_stopping")

        provider.release.set()
        deadline = time.monotonic() + 1
        while True:
            late = runtime.get(running["transcription_id"])
            if late["late_provider_result_discarded"]:
                break
            if time.monotonic() >= deadline:
                self.fail("late provider result was not discarded")
            time.sleep(0.001)
        self.assertEqual(late["status"], "cancelled")
        self.assertFalse(late["provider_work_pending"])
        self.assertFalse(late["audio"]["retained"])
        self.assertNotIn("text", late)

        while runtime.shutdown(0)["worker_alive"]:
            if time.monotonic() >= deadline:
                self.fail("STT worker did not stop")
            time.sleep(0.001)

        repeated = runtime.shutdown()
        self.assertFalse(repeated["worker_alive"])
        self.assertFalse(repeated["timed_out"])
        self.assertEqual(repeated["queued_remaining"], 0)
        self.assertEqual(repeated["provider_work_pending"], 0)
        self.assertEqual(repeated["event_dropped_total"], 0)
        expected_tail = (
            "stt.transcription_cancelled",
            "stt.transcription_cancelled",
            "stt.runtime_shutdown",
            "stt.transcription_late_result_discarded",
        )
        self.assertEqual(
            tuple(kind for kind, _data in events[-4:]),
            expected_tail,
        )
        serialized_events = json.dumps(events, ensure_ascii=False)
        self.assertNotIn(provider.result.text, serialized_events)
        self.assertNotIn(
            provider.calls[0].audio.sha256,
            serialized_events,
        )

    def test_rejects_invalid_configuration_submission_and_shutdown(self):
        invalid_configs = (
            {"provider": object()},
            {"provider": RecordingProvider(), "queue_capacity": True},
            {"provider": RecordingProvider(), "queue_capacity": 0},
            {
                "provider": RecordingProvider(),
                "queue_capacity": 2,
                "job_capacity": 1,
            },
            {"provider": RecordingProvider(), "event_sink": object()},
        )
        for values in invalid_configs:
            with self.subTest(values=values):
                provider = values.pop("provider")
                with self.assertRaises(STTRuntimeError) as raised:
                    STTRuntime(provider, **values)
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_configuration",
                )

        runtime = STTRuntime(RecordingProvider())
        self.addCleanup(runtime.shutdown)
        with self.assertRaises(STTRuntimeError) as invalid_request:
            runtime.submit(object())
        self.assertEqual(
            invalid_request.exception.code,
            "invalid_stt_request",
        )
        for timeout in (True, -1, float("nan"), "1"):
            with self.subTest(timeout=timeout):
                with self.assertRaises(STTRuntimeError) as raised:
                    runtime.shutdown(timeout)
                self.assertEqual(
                    raised.exception.code,
                    "invalid_stt_shutdown_timeout",
                )


if __name__ == "__main__":
    unittest.main()
