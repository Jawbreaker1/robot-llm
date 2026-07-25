import json
import unittest
from dataclasses import dataclass

from robot_agent.shadow_commentary import (
    SPEECH_SOURCE,
    ShadowSpeechError,
    run_shadow_comment,
)


@dataclass(frozen=True)
class Candidate:
    text: str
    latency_ms: int


class FakeModel:
    def __init__(self, result=None, error=None):
        self.result = result
        self.error = error
        self.observations = []

    def comment(self, observation):
        self.observations.append(observation)
        if self.error is not None:
            raise self.error
        return self.result


class RecordingSpeaker:
    def __init__(self, error=None):
        self.error = error
        self.texts = []

    def __call__(self, text):
        self.texts.append(text)
        if self.error is not None:
            raise self.error
        return {"status": "completed", "characters": len(text)}


class LMStudioTimeout(RuntimeError):
    pass


class LMStudioTransportError(RuntimeError):
    pass


class ShadowCommentaryTests(unittest.TestCase):
    def test_valid_candidate_is_logged_but_fallback_is_spoken(self):
        model = FakeModel(Candidate("Vad fan är det där framför mig?", 187))
        speaker = RecordingSpeaker()

        result = run_shadow_comment([27, 28, 29], 12_345, model, speaker)

        self.assertEqual(result.observation.zone, "near_return")
        self.assertEqual(result.candidate_status, "valid")
        self.assertEqual(
            result.model_candidate,
            "Vad fan är det där framför mig?",
        )
        self.assertEqual(result.model_latency_ms, 187)
        self.assertEqual(result.speech_source, SPEECH_SOURCE)
        self.assertEqual(
            result.spoken_text,
            "Jag märker något framför mig.",
        )
        self.assertEqual(speaker.texts, [result.fallback_text])
        self.assertNotEqual(result.model_candidate, result.spoken_text)

    def test_structurally_valid_hallucination_never_reaches_tts(self):
        model = FakeModel(Candidate("En hund står femton centimeter bort.", 90))
        speaker = RecordingSpeaker()

        result = run_shadow_comment([58, 58, 59], 99, model, speaker)

        self.assertEqual(result.candidate_status, "valid")
        self.assertEqual(
            result.model_candidate,
            "En hund står femton centimeter bort.",
        )
        self.assertEqual(
            speaker.texts,
            ["Jag får ingen tydlig närträff framför mig."],
        )

    def test_invalid_candidates_fall_back_without_blocking_tts(self):
        invalid_candidates = [
            "",
            "rad ett\nrad två",
            "x" * 81,
            object(),
        ]
        for candidate in invalid_candidates:
            with self.subTest(candidate=candidate):
                model = FakeModel(candidate)
                speaker = RecordingSpeaker()

                result = run_shadow_comment([13, 13, 13], 1, model, speaker)

                self.assertEqual(result.candidate_status, "invalid")
                self.assertIsNone(result.model_candidate)
                self.assertEqual(speaker.texts, [result.fallback_text])

    def test_model_timeout_is_audited_and_fallback_is_spoken(self):
        model = FakeModel(error=LMStudioTimeout("deadline exceeded"))
        speaker = RecordingSpeaker()

        result = run_shadow_comment([45, 45, 46], 2, model, speaker)

        self.assertEqual(result.candidate_status, "timeout")
        self.assertIn("LMStudioTimeout", result.model_error)
        self.assertEqual(speaker.texts, [result.fallback_text])

    def test_model_transport_failure_is_audited_as_unavailable(self):
        model = FakeModel(error=LMStudioTransportError("server is off"))
        speaker = RecordingSpeaker()

        result = run_shadow_comment([45, 45, 46], 2, model, speaker)

        self.assertEqual(result.candidate_status, "unavailable")
        self.assertEqual(speaker.texts, [result.fallback_text])

    def test_invalid_sensor_input_calls_neither_model_nor_speaker(self):
        invalid_inputs = [
            ([], 1),
            ([True], 1),
            ([101], 1),
            ([28], True),
            ([28], -1),
        ]
        for samples, observed_at_ms in invalid_inputs:
            with self.subTest(samples=samples, observed_at_ms=observed_at_ms):
                model = FakeModel(Candidate("Något är nära.", 1))
                speaker = RecordingSpeaker()

                with self.assertRaises(ValueError):
                    run_shadow_comment(
                        samples,
                        observed_at_ms,
                        model,
                        speaker,
                    )

                self.assertEqual(model.observations, [])
                self.assertEqual(speaker.texts, [])

    def test_tts_failure_is_not_reported_as_success(self):
        model = FakeModel(Candidate("Något är nära.", 10))
        speaker = RecordingSpeaker(error=RuntimeError("audio busy"))

        with self.assertRaises(ShadowSpeechError) as caught:
            run_shadow_comment([28, 28, 28], 5, model, speaker)

        self.assertEqual(caught.exception.audit["tts_status"], "failed")
        self.assertIn("RuntimeError", caught.exception.audit["tts_error"])
        self.assertEqual(
            caught.exception.audit["speech_source"],
            SPEECH_SOURCE,
        )

    def test_result_is_json_serializable(self):
        result = run_shadow_comment(
            [28, 28, 29],
            42,
            FakeModel(Candidate("Något jävla stör mig.", 15)),
            RecordingSpeaker(),
        )

        encoded = json.dumps(result.to_dict(), sort_keys=True)

        self.assertIn('"candidate_status": "valid"', encoded)
        self.assertIn('"speech_source": "deterministic_fallback"', encoded)


if __name__ == "__main__":
    unittest.main()
