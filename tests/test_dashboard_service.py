import json
import os
import struct
import subprocess
import sys
import threading
import time
import unittest
from dataclasses import replace

from robot_agent.dashboard_contract import (
    EXPERIMENT_SUMMARY_KEYS,
    EXPERIMENT_TITLE_KEYS,
    REGISTRY_DISPLAY_NAME_KEYS,
)
from robot_agent.dashboard_service import (
    MAX_SPATIAL_MAP_BYTES,
    DashboardService,
    DashboardServiceError,
)
from robot_agent.http_transport import DirectHTTPResponse
from robot_agent.research_loop import (
    ANSWERED,
    CLARIFICATION_REQUIRED,
    PLANNER_FAILED,
    ResearchEvidenceEnvelope,
    ResearchEpisodeResult,
)
from robot_agent.stt_contract import ProviderTranscription


def episode(
    termination,
    *,
    answer_text=None,
    clarification_question=None,
    citation_ids=(),
    completed=None,
    planner_turns=1,
    tool_calls=0,
    replans=0,
    evidence=(),
):
    return ResearchEpisodeResult(
        turn_id="runner-overwrites-no-authority",
        completed=(
            termination == ANSWERED
            if completed is None
            else completed
        ),
        termination=termination,
        answer_text=answer_text,
        clarification_question=clarification_question,
        citation_ids=tuple(citation_ids),
        planner_turns=planner_turns,
        tool_calls=tool_calls,
        replans=replans,
        final_context_version=1,
        evidence=tuple(evidence),
        trace=("CREATED", termination),
    )


class ScriptedRunner:
    def __init__(self, *results):
        self.results = list(results)
        self.calls = []
        self.lock = threading.Lock()

    def __call__(self, turn, history, conversation_version, settings):
        with self.lock:
            self.calls.append(
                {
                    "turn": turn,
                    "history": history,
                    "conversation_version": conversation_version,
                    "settings": settings,
                }
            )
            result = self.results.pop(0)
        if isinstance(result, BaseException):
            raise result
        if result.turn_id == "runner-overwrites-no-authority":
            result = replace(result, turn_id=turn.turn_id)
        return result


class BlockingRunner:
    def __init__(self, result):
        self.result = result
        self.started = threading.Event()
        self.release = threading.Event()
        self.calls = []

    def __call__(self, turn, history, conversation_version, settings):
        self.calls.append((turn, history, conversation_version, settings))
        self.started.set()
        if not self.release.wait(5):
            raise RuntimeError("test runner timed out")
        if self.result.turn_id == "runner-overwrites-no-authority":
            return replace(self.result, turn_id=turn.turn_id)
        return self.result


class ScriptedSpeechProvider:
    provider_id = "fixture-stt"
    model_id = "fixture-multilingual-v1"

    def __init__(self, text="Vinka med höger arm."):
        self.text = text
        self.calls = []
        self.probe_calls = 0

    def transcribe(self, request):
        self.calls.append(request)
        return ProviderTranscription(
            text=self.text,
            provider_id=self.provider_id,
            model_id=self.model_id,
            detected_language="sv",
        )

    def probe(self):
        self.probe_calls += 1
        return {
            "state": "online",
            "provider_id": self.provider_id,
            "model_id": self.model_id,
        }


class BlockingSpeechProvider(ScriptedSpeechProvider):
    def __init__(self, text="Vinka med höger arm."):
        super().__init__(text)
        self.started = threading.Event()
        self.release = threading.Event()

    def transcribe(self, request):
        self.calls.append(request)
        self.started.set()
        if not self.release.wait(5):
            raise RuntimeError("test speech provider timed out")
        return ProviderTranscription(
            text=self.text,
            provider_id=self.provider_id,
            model_id=self.model_id,
            detected_language="sv",
        )


def canonical_wav(duration_ms=250, sample=0):
    sample_count = 16_000 * duration_ms // 1_000
    data = struct.pack("<h", sample) * sample_count
    return (
        b"RIFF"
        + struct.pack("<I", 36 + len(data))
        + b"WAVE"
        + b"fmt "
        + struct.pack(
            "<IHHIIHH",
            16,
            1,
            1,
            16_000,
            32_000,
            2,
            16,
        )
        + b"data"
        + struct.pack("<I", len(data))
        + data
    )


def wait_for_terminal(service, turn_id, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = service.get_turn(turn_id)
        if value["status"] in (
            "answered",
            "clarification_required",
            "failed",
        ):
            return value
        time.sleep(0.005)
    raise AssertionError("turn did not become terminal")


def wait_for_transcription(service, transcription_id, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = service.get_transcription(transcription_id)
        if value["status"] in ("completed", "failed", "cancelled"):
            return value
        time.sleep(0.005)
    raise AssertionError("transcription did not become terminal")


class DashboardServiceTests(unittest.TestCase):
    def setUp(self):
        self.services = []

    def tearDown(self):
        for service in self.services:
            service.shutdown()

    def make_service(self, **kwargs):
        service = DashboardService(**kwargs)
        self.services.append(service)
        return service

    def test_spatial_map_payload_has_generous_bounded_headroom(self):
        self.assertEqual(MAX_SPATIAL_MAP_BYTES, 4 * 1024 * 1024)

    def submit(
        self,
        service,
        conversation,
        request_id,
        content,
        mode,
        response_locale="sv",
    ):
        return service.submit_turn(
            conversation["conversation_id"],
            request_id,
            conversation["version"],
            content,
            mode,
            response_locale,
        )

    def test_answer_clarification_and_failure_are_safe_terminal_views(self):
        runner = ScriptedRunner(
            episode(ANSWERED, answer_text="Hej tillbaka."),
            episode(
                CLARIFICATION_REQUIRED,
                clarification_question="Vilken plats menar du?",
            ),
            episode(PLANNER_FAILED),
        )
        service = self.make_service(episode_runner=runner)
        conversation = service.create_conversation("Test")

        first = self.submit(
            service,
            conversation,
            "request-answer",
            "Hej",
            "conversation",
            response_locale="en",
        )
        answered = wait_for_terminal(service, first["turn_id"])
        self.assertEqual(answered["status"], "answered")
        self.assertEqual(answered["response_locale"], "en")
        self.assertEqual(
            runner.calls[0]["turn"].response_locale,
            "en",
        )
        self.assertEqual(answered["answer_text"], "Hej tillbaka.")
        self.assertEqual(answered["episode"]["termination"], ANSWERED)

        conversation = service.get_conversation(
            conversation["conversation_id"]
        )
        second = self.submit(
            service,
            conversation,
            "request-clarify",
            "Hur är vädret?",
            "research_required",
        )
        clarified = wait_for_terminal(service, second["turn_id"])
        self.assertEqual(clarified["status"], "clarification_required")
        self.assertEqual(
            clarified["clarification_question"],
            "Vilken plats menar du?",
        )

        conversation = service.get_conversation(
            conversation["conversation_id"]
        )
        third = self.submit(
            service,
            conversation,
            "request-fail",
            "Försök igen",
            "conversation",
        )
        failed = wait_for_terminal(service, third["turn_id"])
        self.assertEqual(failed["status"], "failed")
        self.assertEqual(failed["error_code"], "episode_planner_failed")
        self.assertNotIn("answer_text", failed["episode"])

    def test_queue_full_and_idempotent_retry_after_settings_update(self):
        blocker = BlockingRunner(
            episode(ANSWERED, answer_text="Klart.")
        )
        service = self.make_service(
            episode_runner=blocker,
            queue_capacity=1,
        )
        first_conversation = service.create_conversation()
        first = self.submit(
            service,
            first_conversation,
            "stable-request",
            "Samma innehåll",
            "conversation",
        )
        self.assertTrue(blocker.started.wait(2))

        service.update_settings(1, {"log_level": "info"})
        replay = service.submit_turn(
            first_conversation["conversation_id"],
            "stable-request",
            999,
            "Samma innehåll",
            "conversation",
            "sv",
        )
        self.assertEqual(replay["turn_id"], first["turn_id"])
        self.assertEqual(replay["settings_revision"], 1)
        with self.assertRaises(DashboardServiceError) as locale_conflict:
            service.submit_turn(
                first_conversation["conversation_id"],
                "stable-request",
                999,
                "Samma innehåll",
                "conversation",
                "en",
            )
        self.assertEqual(
            locale_conflict.exception.code,
            "idempotency_conflict",
        )

        second_conversation = service.create_conversation()
        with self.assertRaises(DashboardServiceError) as raised:
            self.submit(
                service,
                second_conversation,
                "request-overflow",
                "Ny köpost",
                "conversation",
            )
        self.assertEqual(raised.exception.status, 429)
        self.assertEqual(raised.exception.code, "chat_queue_full")

        blocker.release.set()
        self.assertEqual(
            wait_for_terminal(service, first["turn_id"])["status"],
            "answered",
        )
        self.assertEqual(blocker.calls[0][3].revision, 1)

    def test_unsupported_response_locale_is_rejected_before_queueing(self):
        service = self.make_service(
            episode_runner=ScriptedRunner(
                episode(ANSWERED, answer_text="Unused")
            )
        )
        conversation = service.create_conversation()

        with self.assertRaises(DashboardServiceError) as raised:
            service.submit_turn(
                conversation["conversation_id"],
                "request-fr",
                conversation["version"],
                "Bonjour",
                "conversation",
                "fr",
            )

        self.assertEqual(raised.exception.status, 400)
        self.assertEqual(
            raised.exception.code,
            "invalid_response_locale",
        )

    def test_planner_receives_bounded_typed_history_without_current_user(self):
        contexts = []
        proposal_number = [0]

        def planner_factory(**_kwargs):
            def planner(context):
                payload = context.to_dict()
                contexts.append(payload)
                proposal_number[0] += 1
                return json.dumps(
                    {
                        "schema": "research-decision/v1",
                        "proposal_id": "proposal-{}".format(
                            proposal_number[0]
                        ),
                        "turn_id": payload["turn_id"],
                        "based_on_context_version": payload[
                            "context_version"
                        ],
                        "decision": "ANSWER",
                        "answer": {
                            "text": "Svar {}".format(proposal_number[0]),
                            "evidence_ids": [],
                        },
                    }
                ).encode("utf-8")

            return planner

        class UnusedWeather:
            def current(self, _request):
                raise AssertionError("conversation mode used weather")

        service = self.make_service(
            planner_factory=planner_factory,
            weather_factory=UnusedWeather,
        )
        conversation = service.create_conversation()
        first = self.submit(
            service,
            conversation,
            "history-1",
            "Första frågan",
            "conversation",
        )
        wait_for_terminal(service, first["turn_id"])

        conversation = service.get_conversation(
            conversation["conversation_id"]
        )
        second = self.submit(
            service,
            conversation,
            "history-2",
            "Två gånger till",
            "conversation",
            response_locale="en",
        )
        wait_for_terminal(service, second["turn_id"])

        history = contexts[-1]["conversation_history"]
        self.assertEqual(contexts[-1]["response_locale"], "en")
        self.assertEqual(history["schema"], "conversation-history/v1")
        self.assertEqual(
            [(item["role"], item["content"]) for item in history["messages"]],
            [
                ("user", "Första frågan"),
                ("assistant", "Svar 1"),
            ],
        )
        self.assertNotIn(
            "Två gånger till",
            [item["content"] for item in history["messages"]],
        )
        self.assertLessEqual(len(history["messages"]), 20)

    def test_event_log_never_contains_user_or_exception_text(self):
        user_secret = "SECRET-USER-CONTENT-731"
        exception_secret = "SECRET-STACK-CONTENT-991"
        runner = ScriptedRunner(RuntimeError(exception_secret))
        service = self.make_service(episode_runner=runner)
        conversation = service.create_conversation()
        turn = self.submit(
            service,
            conversation,
            "safe-request-id",
            user_secret,
            "conversation",
        )
        failed = wait_for_terminal(service, turn["turn_id"])
        self.assertEqual(failed["status"], "failed")

        encoded = json.dumps(
            service.events(0, 1_000),
            ensure_ascii=False,
            sort_keys=True,
        )
        self.assertNotIn(user_secret, encoded)
        self.assertNotIn(exception_secret, encoded)

    def test_speech_runs_independently_and_event_log_stays_private(self):
        blocker = BlockingRunner(
            episode(ANSWERED, answer_text="Chatten är klar.")
        )
        transcript_secret = "HEMLIG TRANSKRIBERING 817"
        provider = ScriptedSpeechProvider(transcript_secret)
        service = self.make_service(
            episode_runner=blocker,
            speech_transcriber=provider,
        )
        conversation = service.create_conversation()
        chat_turn = self.submit(
            service,
            conversation,
            "parallel-chat",
            "Blockera forskningsarbetaren",
            "conversation",
        )
        self.assertTrue(blocker.started.wait(1))
        wav = canonical_wav(sample=317)

        submitted = service.submit_transcription(
            "parallel-voice",
            "sv-SE",
            wav,
        )
        transcribed = wait_for_transcription(
            service,
            submitted["transcription_id"],
        )

        self.assertEqual(transcribed["status"], "completed")
        self.assertEqual(transcribed["text"], transcript_secret)
        self.assertEqual(transcribed["detected_language"], "sv")
        self.assertEqual(len(provider.calls), 1)
        self.assertEqual(provider.calls[0].language_hint, "sv")
        self.assertEqual(
            service.get_turn(chat_turn["turn_id"])["status"],
            "running",
        )

        encoded_events = json.dumps(
            service.events(0, 1_000),
            ensure_ascii=False,
            sort_keys=True,
        )
        self.assertNotIn(transcript_secret, encoded_events)
        self.assertNotIn(provider.calls[0].audio.sha256, encoded_events)
        self.assertNotIn("wav_bytes", encoded_events)
        self.assertNotIn("audio_sha256", encoded_events)

        blocker.release.set()
        self.assertEqual(
            wait_for_terminal(service, chat_turn["turn_id"])["status"],
            "answered",
        )

    def test_speech_cancellation_discards_late_result_and_stays_private(self):
        transcript_secret = "CANCELLED TRANSCRIPT MUST NOT LEAK 991"
        provider = BlockingSpeechProvider(transcript_secret)
        service = self.make_service(speech_transcriber=provider)
        submitted = service.submit_transcription(
            "cancel-service-voice",
            "sv",
            canonical_wav(sample=913),
        )
        self.assertTrue(provider.started.wait(1))

        cancelled = service.cancel_transcription(
            submitted["transcription_id"]
        )

        self.assertEqual(cancelled["status"], "cancelled")
        self.assertEqual(cancelled["error_code"], "stt_cancelled")
        self.assertTrue(cancelled["provider_work_pending"])
        self.assertTrue(cancelled["audio"]["retained"])
        self.assertNotIn("text", cancelled)

        provider.release.set()
        deadline = time.monotonic() + 1
        while True:
            final = service.get_transcription(
                submitted["transcription_id"]
            )
            if final["late_provider_result_discarded"]:
                break
            if time.monotonic() >= deadline:
                self.fail("late speech result was not discarded")
            time.sleep(0.001)
        self.assertEqual(final["status"], "cancelled")
        self.assertFalse(final["provider_work_pending"])
        self.assertFalse(final["audio"]["retained"])
        self.assertNotIn("text", final)

        deadline = time.monotonic() + 1
        while True:
            event_page = service.events(0, 1_000)
            event_types = [
                event["event_type"]
                for event in event_page["events"]
            ]
            if "stt.transcription_late_result_discarded" in event_types:
                break
            if time.monotonic() >= deadline:
                self.fail("speech cancellation events were not recorded")
            time.sleep(0.001)
        encoded_events = json.dumps(
            event_page,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.assertIn("stt.transcription_cancelled", event_types)
        self.assertNotIn(transcript_secret, encoded_events)
        self.assertNotIn(provider.calls[0].audio.sha256, encoded_events)
        self.assertNotIn("wav_bytes", encoded_events)
        self.assertNotIn("audio_sha256", encoded_events)

    def test_episode_result_for_another_turn_fails_closed(self):
        stale = replace(
            episode(ANSWERED, answer_text="Fel tur"),
            turn_id="turn-from-another-episode",
        )

        def runner(_turn, _history, _version, _settings):
            return stale

        service = self.make_service(episode_runner=runner)
        conversation = service.create_conversation()
        submitted = self.submit(
            service,
            conversation,
            "identity-check",
            "Svara på rätt tur",
            "conversation",
        )
        result = wait_for_terminal(service, submitted["turn_id"])

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error_code"], "episode_failed")
        self.assertIsNone(result["answer_text"])
        self.assertNotIn("episode", result)

    def test_untrusted_runner_cannot_bypass_result_or_budget_contract(self):
        invalid_results = (
            (
                "answered-but-incomplete",
                episode(
                    ANSWERED,
                    answer_text="Får inte publiceras",
                    completed=False,
                ),
                "conversation",
            ),
            (
                "fabricated-citation",
                episode(
                    ANSWERED,
                    answer_text="Påstått belagt",
                    citation_ids=("evidence-that-does-not-exist",),
                    tool_calls=1,
                ),
                "research_required",
            ),
            (
                "research-answer-without-evidence",
                episode(
                    ANSWERED,
                    answer_text="Obelagt svar",
                ),
                "research_required",
            ),
            (
                "planner-budget-exceeded",
                episode(
                    ANSWERED,
                    answer_text="För dyrt svar",
                    planner_turns=7,
                ),
                "conversation",
            ),
        )

        for name, runner_result, mode in invalid_results:
            with self.subTest(name=name):
                service = self.make_service(
                    episode_runner=ScriptedRunner(runner_result)
                )
                conversation = service.create_conversation()
                submitted = self.submit(
                    service,
                    conversation,
                    "contract-{}".format(name),
                    "Kontrollera resultatet",
                    mode,
                )
                result = wait_for_terminal(
                    service,
                    submitted["turn_id"],
                )

                self.assertEqual(result["status"], "failed")
                self.assertEqual(
                    result["error_code"],
                    "episode_failed",
                )
                self.assertIsNone(result["answer_text"])
                self.assertNotIn("episode", result)

    def test_evidence_cannot_exist_without_a_counted_tool_call(self):
        now_ms = time.monotonic_ns() // 1_000_000
        fabricated = ResearchEvidenceEnvelope(
            evidence_id="evidence-1",
            tool_call_id="tool-call-1",
            tool_name="weather.current",
            produced_from_context_version=1,
            received_at_monotonic_ms=now_ms,
            valid_until_monotonic_ms=now_ms + 60_000,
            payload={"value": "untrusted"},
        )
        runner = ScriptedRunner(
            episode(
                ANSWERED,
                answer_text="Fabricerat belägg",
                citation_ids=("evidence-1",),
                tool_calls=0,
                evidence=(fabricated,),
            )
        )
        service = self.make_service(episode_runner=runner)
        conversation = service.create_conversation()
        submitted = self.submit(
            service,
            conversation,
            "evidence-without-call",
            "Kontrollera evidensen",
            "conversation",
        )

        result = wait_for_terminal(service, submitted["turn_id"])
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error_code"], "episode_failed")

    def test_bootstrap_and_registry_never_expose_physical_control(self):
        service = self.make_service(
            episode_runner=ScriptedRunner(
                episode(ANSWERED, answer_text="ok")
            )
        )
        bootstrap = service.bootstrap()
        registry = service.registry()

        self.assertFalse(bootstrap["physical_control_enabled"])
        self.assertFalse(
            bootstrap["capabilities"]["physical_control"]
        )
        self.assertEqual(
            bootstrap["capabilities"]["workbench"],
            {
                "schema": "dashboard-workbench-capabilities/v1",
                "tool_effects": "read_only",
                "physical_control": False,
                "ssh": False,
                "tts": False,
            },
        )
        self.assertFalse(registry["physical_control_enabled"])
        self.assertTrue(
            all(not node["control_exposed"] for node in registry["nodes"])
        )
        self.assertEqual(bootstrap["runtime"]["controllers"], [])
        self.assertEqual(
            bootstrap["runtime"]["ev3"],
            {
                "state": "unobserved",
                "reason_code": "physical_probe_not_run",
            },
        )
        self.assertFalse(bootstrap["capabilities"]["ssh"])
        self.assertEqual(
            bootstrap["capabilities"]["spatial_map"],
            "read_only",
        )
        self.assertFalse(
            bootstrap["capabilities"]["speech_to_text"]["enabled"]
        )
        self.assertEqual(
            bootstrap["runtime"]["speech_to_text"]["state"],
            "disabled",
        )

    def test_bootstrap_and_probe_describe_local_speech_runtime(self):
        provider = ScriptedSpeechProvider()
        service = self.make_service(
            episode_runner=ScriptedRunner(),
            speech_transcriber=provider,
        )

        bootstrap = service.bootstrap()
        capability = bootstrap["capabilities"]["speech_to_text"]
        probed = service.probe_speech_transcriber()

        self.assertTrue(capability["enabled"])
        self.assertEqual(capability["input_format"], "audio/wav")
        self.assertEqual(capability["encoding"], "pcm_s16le")
        self.assertEqual(capability["sample_rate_hz"], 16_000)
        self.assertEqual(capability["channels"], 1)
        self.assertFalse(capability["audio_persisted"])
        self.assertEqual(
            bootstrap["runtime"]["speech_to_text"]["state"],
            "configured",
        )
        self.assertEqual(probed["state"], "online")
        self.assertEqual(provider.probe_calls, 1)


    def test_spatial_map_defaults_to_an_honest_empty_snapshot(self):
        service = self.make_service(
            episode_runner=ScriptedRunner()
        )

        snapshot = service.spatial_map()

        self.assertEqual(snapshot["schema"], "robot-spatial-map/v1")
        self.assertEqual(snapshot["status"], "unavailable")
        self.assertEqual(
            snapshot["reason_code"],
            "no_spatial_map_provider",
        )
        self.assertIs(snapshot["read_only"], True)
        self.assertIsNone(snapshot["robot_pose"])
        self.assertEqual(snapshot["pose_history"], [])
        self.assertEqual(snapshot["pose_history_evicted"], 0)
        self.assertEqual(snapshot["scan_evidence_history"], [])
        self.assertEqual(snapshot["scan_evidence_history_evicted"], 0)
        self.assertEqual(snapshot["qualitative_observations_evicted"], 0)
        self.assertIsNone(snapshot["hazard_retention"])
        self.assertIsNone(snapshot["scan_attempt_retention"])
        self.assertEqual(snapshot["cells"], [])
        self.assertEqual(snapshot["sensor_rays"], [])
        self.assertEqual(snapshot["object_hypotheses"], [])
        self.assertIsNone(snapshot["source_id"])
        self.assertIsNone(snapshot["provenance"])

    def test_spatial_map_store_is_injected_and_snapshot_is_detached(self):
        supplied = {
            "schema": "robot-spatial-map/v1",
            "status": "available",
            "read_only": True,
            "robot_id": "ev3rstorm-01",
            "frame_id": "SIM_WORLD",
            "source_id": "navigation-simulator",
            "provenance": "SIMULATION",
            "captured_at_unix_ms": 1_785_251_200_000,
            "bounds": {
                "min_x_mm": 0,
                "min_y_mm": 0,
                "max_x_mm": 2_000,
                "max_y_mm": 1_200,
            },
            "robot_pose": {
                "x_mm": 500,
                "y_mm": 400,
                "heading_mdeg": 90_000,
            },
            "cells": [
                {
                    "x_mm": 700,
                    "y_mm": 400,
                    "state": "FREE",
                }
            ],
            "sensor_rays": [],
            "object_hypotheses": [],
        }

        class MapStore:
            def __init__(self):
                self.calls = 0

            def snapshot(self):
                self.calls += 1
                return supplied

        store = MapStore()
        service = self.make_service(
            episode_runner=ScriptedRunner(),
            spatial_map_provider=store,
        )

        first = service.spatial_map()
        first["cells"].append({"state": "OCCUPIED"})
        second = service.spatial_map()

        self.assertEqual(store.calls, 2)
        self.assertEqual(second["cells"], supplied["cells"])
        self.assertIsNot(second, supplied)
        self.assertIsNot(second["bounds"], supplied["bounds"])

    def test_spatial_map_store_prefers_read_capability_over_call(self):
        class CapabilitySeparatedStore:
            def __init__(self):
                self.reads = 0
                self.write_calls = 0

            def __call__(self, _observation=None):
                self.write_calls += 1
                raise AssertionError("write capability was invoked")

            def snapshot(self):
                self.reads += 1
                return {
                    "schema": "robot-spatial-map/v1",
                    "status": "unavailable",
                    "reason_code": "no_observations",
                    "read_only": True,
                    "cells": [],
                    "sensor_rays": [],
                    "object_hypotheses": [],
                }

        store = CapabilitySeparatedStore()
        service = self.make_service(
            episode_runner=ScriptedRunner(),
            spatial_map_provider=store,
        )

        snapshot = service.spatial_map()

        self.assertEqual(snapshot["reason_code"], "no_observations")
        self.assertEqual(store.reads, 1)
        self.assertEqual(store.write_calls, 0)

    def test_spatial_map_provider_fails_closed_without_leaking_errors(self):
        secret = "MAP-PROVIDER-SECRET"

        def broken():
            raise RuntimeError(secret)

        service = self.make_service(
            episode_runner=ScriptedRunner(),
            spatial_map_provider=broken,
        )
        with self.assertRaises(DashboardServiceError) as raised:
            service.spatial_map()

        self.assertEqual(raised.exception.status, 503)
        self.assertEqual(
            raised.exception.code,
            "spatial_map_unavailable",
        )
        self.assertNotIn(secret, str(raised.exception))

        invalid = self.make_service(
            episode_runner=ScriptedRunner(),
            spatial_map_provider=lambda: {
                "schema": "wrong/v1",
                "read_only": True,
            },
        )
        with self.assertRaises(DashboardServiceError) as mismatch:
            invalid.spatial_map()
        self.assertEqual(
            mismatch.exception.code,
            "spatial_map_unavailable",
        )

    def test_bootstrap_uses_typed_catalog_keys_for_curated_copy(self):
        service = self.make_service(
            episode_runner=ScriptedRunner(
                episode(ANSWERED, answer_text="ok")
            )
        )
        bootstrap = service.bootstrap()
        experiments = bootstrap["experiments"]

        self.assertEqual(len(experiments), 4)
        self.assertEqual(
            {item["title_key"] for item in experiments},
            set(EXPERIMENT_TITLE_KEYS),
        )
        self.assertEqual(
            {item["summary_key"] for item in experiments},
            set(EXPERIMENT_SUMMARY_KEYS),
        )
        self.assertEqual(
            {item["schema"] for item in experiments},
            {"dashboard-experiment/v1"},
        )
        self.assertEqual(
            len({item["experiment_id"] for item in experiments}),
            len(experiments),
        )
        for item in experiments:
            self.assertNotIn("title", item)
            self.assertNotIn("summary", item)

    def test_registry_localizes_only_explicit_generic_names(self):
        service = self.make_service(
            episode_runner=ScriptedRunner(
                episode(ANSWERED, answer_text="ok")
            )
        )
        registry = service.registry()
        records = registry["robots"] + registry["nodes"]
        keyed = {
            record["display_name"]: record["display_name_key"]
            for record in records
            if record["display_name_key"] is not None
        }
        raw_names = {
            record["display_name"]
            for record in records
            if record["display_name_key"] is None
        }

        self.assertEqual(set(keyed.values()), set(REGISTRY_DISPLAY_NAME_KEYS))
        self.assertTrue(
            {
                "EV3RSTORM",
                "EV3 Main",
                "BLAST",
                "Robot Inventor Hub",
                "BOOST Move Hub",
                "LM Studio",
                "Open-Meteo",
            }
            <= raw_names
        )

    def test_bootstrap_exposes_read_only_blast_runtime_snapshot(self):
        runtime_snapshot = {
            "schema": "controller-runtime-observation/v1",
            "robot_id": "blast-01",
            "controller_id": "blast-01.hub",
            "display_name": "BLAST",
            "hub_name": "BLAST-01",
            "state": "online",
            "reason_code": None,
            "last_observed_at_unix_ms": 1_000,
            "last_observed_at_monotonic_ms": 900,
            "ready": {"protocol_version": 1},
            "observation": {"distance_mm": 321},
        }
        service = self.make_service(
            episode_runner=ScriptedRunner(),
            controller_runtime_providers=(lambda: runtime_snapshot,),
        )

        bootstrap = service.bootstrap()
        registry = bootstrap["registry"]

        self.assertEqual(
            bootstrap["runtime"]["controllers"],
            [runtime_snapshot],
        )
        self.assertEqual(
            bootstrap["runtime"]["ev3"]["state"],
            "unobserved",
        )
        blast = next(
            robot
            for robot in registry["robots"]
            if robot["robot_id"] == "blast-01"
        )
        self.assertEqual(blast["node_ids"], ["blast-01.hub"])
        controller = next(
            node
            for node in registry["nodes"]
            if node["controller_id"] == "blast-01.hub"
        )
        self.assertFalse(controller["control_exposed"])
        self.assertIn("sensor.imu", controller["capabilities"])

    def test_invalid_controller_runtime_does_not_break_bootstrap(self):
        service = self.make_service(
            episode_runner=ScriptedRunner(),
            controller_runtime_providers=(lambda: {"schema": "wrong"},),
        )

        self.assertEqual(service.bootstrap()["runtime"]["controllers"], [])

    def test_lm_probe_is_fixed_loopback_get_and_bounded(self):
        calls = []
        configured_model = "local/test-model"

        def transport(*args):
            calls.append(args)
            return DirectHTTPResponse(
                status_code=200,
                headers=(("Content-Type", "application/json"),),
                body=json.dumps(
                    {"object": "list", "data": [{"id": configured_model}]}
                ).encode("utf-8"),
            )

        service = self.make_service(
            base_url="http://127.0.0.1:1234",
            model=configured_model,
            probe_transport=transport,
            episode_runner=ScriptedRunner(),
        )
        result = service.probe_lm_studio()

        self.assertEqual(result["state"], "online")
        self.assertTrue(result["configured_model_loaded"])
        self.assertEqual(calls[0][0], "GET")
        self.assertEqual(
            calls[0][1],
            "http://127.0.0.1:1234/v1/models",
        )
        self.assertIsNone(calls[0][3])
        self.assertLessEqual(calls[0][4], 2.0)
        self.assertLessEqual(calls[0][5], 64 * 1024)

    def test_later_started_probe_wins_if_older_probe_finishes_last(self):
        first_started = threading.Event()
        release_first = threading.Event()
        call_lock = threading.Lock()
        call_number = [0]

        def transport(*_args):
            with call_lock:
                call_number[0] += 1
                number = call_number[0]
            if number == 1:
                first_started.set()
                if not release_first.wait(2):
                    raise RuntimeError("probe test timed out")
                raise OSError("older probe is offline")
            return DirectHTTPResponse(
                status_code=200,
                headers=(("Content-Type", "application/json"),),
                body=json.dumps(
                    {
                        "object": "list",
                        "data": [{"id": "local/test-model"}],
                    }
                ).encode("utf-8"),
            )

        service = self.make_service(
            model="local/test-model",
            probe_transport=transport,
            episode_runner=ScriptedRunner(),
        )
        older_result = []
        older = threading.Thread(
            target=lambda: older_result.append(
                service.probe_lm_studio()
            )
        )
        older.start()
        self.assertTrue(first_started.wait(1))

        newer = service.probe_lm_studio()
        release_first.set()
        older.join(2)

        self.assertFalse(older.is_alive())
        self.assertEqual(newer["state"], "online")
        self.assertEqual(older_result[0]["state"], "online")
        self.assertEqual(
            service.bootstrap()["runtime"]["lm_studio"]["state"],
            "online",
        )

    def test_shutdown_is_idempotent_and_rejects_new_work(self):
        service = self.make_service(
            episode_runner=ScriptedRunner(
                episode(ANSWERED, answer_text="ok")
            )
        )
        conversation = service.create_conversation()
        service.shutdown()
        service.shutdown()

        with self.assertRaises(DashboardServiceError) as raised:
            self.submit(
                service,
                conversation,
                "after-shutdown",
                "Hej",
                "conversation",
            )
        self.assertEqual(raised.exception.status, 503)
        self.assertEqual(raised.exception.code, "service_stopping")

    def test_shutdown_cancels_queued_work_and_reports_live_worker(self):
        blocker = BlockingRunner(
            episode(ANSWERED, answer_text="Första klar.")
        )
        service = self.make_service(
            episode_runner=blocker,
            queue_capacity=2,
        )
        first_conversation = service.create_conversation()
        first = self.submit(
            service,
            first_conversation,
            "shutdown-running",
            "Pågående",
            "conversation",
        )
        self.assertTrue(blocker.started.wait(1))
        second_conversation = service.create_conversation()
        second = self.submit(
            service,
            second_conversation,
            "shutdown-queued",
            "Får inte köras",
            "conversation",
        )

        status = service.shutdown(timeout_seconds=0.01)
        cancelled = service.get_turn(second["turn_id"])

        self.assertTrue(status["timed_out"])
        self.assertTrue(status["worker_alive"])
        self.assertEqual(status["queued_cancelled_total"], 1)
        self.assertEqual(status["queued_remaining"], 0)
        self.assertEqual(cancelled["status"], "failed")
        self.assertEqual(
            cancelled["error_code"],
            "service_stopping",
        )
        self.assertEqual(len(blocker.calls), 1)

        blocker.release.set()
        self.assertEqual(
            wait_for_terminal(service, first["turn_id"])["status"],
            "answered",
        )
        completed_shutdown = service.shutdown(timeout_seconds=1)
        self.assertFalse(completed_shutdown["timed_out"])
        self.assertFalse(completed_shutdown["worker_alive"])
        self.assertEqual(
            completed_shutdown["queued_cancelled_total"],
            1,
        )
        self.assertEqual(len(blocker.calls), 1)

    def test_shutdown_wins_before_worker_marks_dequeued_turn_running(self):
        runner = ScriptedRunner(
            episode(ANSWERED, answer_text="Får inte köras.")
        )
        service = self.make_service(
            episode_runner=runner,
            queue_capacity=1,
        )
        conversation = service.create_conversation()

        with service._submit_lock:
            submitted = self.submit(
                service,
                conversation,
                "shutdown-start-race",
                "Stanna i queued",
                "conversation",
            )
            deadline = time.monotonic() + 1
            while service._jobs.qsize() != 0:
                if time.monotonic() >= deadline:
                    self.fail("worker did not dequeue test job")
                time.sleep(0.001)
            status = service.shutdown(timeout_seconds=0.01)

        cancelled = wait_for_terminal(
            service,
            submitted["turn_id"],
        )
        completed_shutdown = service.shutdown(timeout_seconds=1)

        self.assertTrue(status["worker_alive"])
        self.assertTrue(status["timed_out"])
        self.assertEqual(cancelled["status"], "failed")
        self.assertEqual(
            cancelled["error_code"],
            "service_stopping",
        )
        self.assertEqual(runner.calls, [])
        self.assertFalse(completed_shutdown["worker_alive"])

    def test_cold_import_does_not_load_physical_execution_modules(self):
        forbidden = (
            "robot_agent.agent_loop",
            "robot_agent.contract",
            "robot_agent.robot_api",
            "robot_agent.safety",
            "robot_agent.simulated_robot",
            "robot_agent.supervisor_transport",
        )
        program = (
            "import json,sys;"
            "import robot_agent.dashboard_service;"
            "forbidden={};"
            "print(json.dumps([name for name in forbidden "
            "if name in sys.modules]))"
        ).format(repr(forbidden))
        environment = dict(os.environ)
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        source_root = str(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            + "/src"
        )
        environment["PYTHONPATH"] = source_root

        completed = subprocess.run(
            [sys.executable, "-c", program],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
            env=environment,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertEqual(json.loads(completed.stdout), [])


if __name__ == "__main__":
    unittest.main()
