"""Motion-free shadow evaluation for model-generated robot commentary."""

from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, Mapping, Optional

from .commentary import (
    ProximityObservation,
    classify_infrared,
    fallback_comment,
    validate_generated_comment,
)


SPEECH_SOURCE = "deterministic_fallback"


@dataclass(frozen=True)
class ShadowCommentResult:
    """Auditable result from one sensor-to-speech shadow cycle."""

    observation: ProximityObservation
    fallback_text: str
    candidate_status: str
    model_candidate: Optional[str]
    model_latency_ms: Optional[int]
    model_error: Optional[str]
    speech_source: str
    spoken_text: str
    tts_result: Mapping[str, object]

    def to_dict(self) -> Dict[str, object]:
        return {
            "observation": {
                "observed_at_ms": self.observation.observed_at_ms,
                "samples": list(self.observation.samples),
                "filtered_percent": self.observation.filtered_percent,
                "zone": self.observation.zone,
            },
            "fallback_text": self.fallback_text,
            "candidate_status": self.candidate_status,
            "model_candidate": self.model_candidate,
            "model_latency_ms": self.model_latency_ms,
            "model_error": self.model_error,
            "speech_source": self.speech_source,
            "spoken_text": self.spoken_text,
            "tts_result": dict(self.tts_result),
        }


class ShadowSpeechError(RuntimeError):
    """TTS failed after an otherwise auditable shadow decision."""

    def __init__(self, message: str, audit: Mapping[str, object]):
        super().__init__(message)
        self.audit = dict(audit)


def _bounded_error(error: BaseException) -> str:
    detail = "{}: {}".format(type(error).__name__, str(error))
    return detail[:200]


def _failure_status(error: BaseException) -> str:
    name = type(error).__name__.lower()
    if "timeout" in name:
        return "timeout"
    if (
        "unavailable" in name
        or "connection" in name
        or "transport" in name
        or "http" in name
    ):
        return "unavailable"
    if "protocol" in name or "json" in name or "response" in name:
        return "protocol_error"
    return "error"


def _candidate_value(candidate: Any) -> tuple[str, Optional[int]]:
    if isinstance(candidate, str):
        return candidate, None

    text = getattr(candidate, "text", None)
    if not isinstance(text, str):
        text = getattr(candidate, "content", None)
    latency_ms = getattr(candidate, "latency_ms", None)
    if isinstance(latency_ms, bool) or (
        latency_ms is not None and not isinstance(latency_ms, int)
    ):
        raise ValueError("Model latency must be an integer or None")
    if not isinstance(text, str):
        raise ValueError("Model candidate must contain text")
    return text, latency_ms


def run_shadow_comment(
    samples: Iterable[int],
    observed_at_ms: int,
    model_client: Any,
    speaker: Callable[[str], Mapping[str, object]],
) -> ShadowCommentResult:
    """Run one motion-free cycle; model text is never selected for speech.

    Sensor validation happens before either external dependency is called.
    Model failures and invalid candidates are recorded, then the deterministic
    zone phrase is still sent to TTS. A TTS failure is not converted into a
    successful result.
    """
    if isinstance(observed_at_ms, bool) or not isinstance(observed_at_ms, int):
        raise ValueError("observed_at_ms must be an integer")
    if observed_at_ms < 0:
        raise ValueError("observed_at_ms must not be negative")

    observation = classify_infrared(samples, observed_at_ms)
    deterministic_text = fallback_comment(observation)
    candidate_status = "valid"
    model_candidate: Optional[str] = None
    model_latency_ms: Optional[int] = None
    model_error: Optional[str] = None

    try:
        candidate = model_client.comment(observation)
        candidate_text, model_latency_ms = _candidate_value(candidate)
        model_candidate = validate_generated_comment(candidate_text)
    except ValueError as error:
        candidate_status = "invalid"
        model_error = _bounded_error(error)
    except Exception as error:
        candidate_status = _failure_status(error)
        model_error = _bounded_error(error)

    partial_audit: Dict[str, object] = {
        "observation": {
            "observed_at_ms": observation.observed_at_ms,
            "samples": list(observation.samples),
            "filtered_percent": observation.filtered_percent,
            "zone": observation.zone,
        },
        "fallback_text": deterministic_text,
        "candidate_status": candidate_status,
        "model_candidate": model_candidate,
        "model_latency_ms": model_latency_ms,
        "model_error": model_error,
        "speech_source": SPEECH_SOURCE,
        "spoken_text": deterministic_text,
    }

    try:
        raw_tts_result = speaker(deterministic_text)
        if not isinstance(raw_tts_result, Mapping):
            raise TypeError("Speaker result must be a mapping")
        tts_result = dict(raw_tts_result)
    except Exception as error:
        partial_audit["tts_status"] = "failed"
        partial_audit["tts_error"] = _bounded_error(error)
        raise ShadowSpeechError("Shadow TTS failed", partial_audit) from error

    return ShadowCommentResult(
        observation=observation,
        fallback_text=deterministic_text,
        candidate_status=candidate_status,
        model_candidate=model_candidate,
        model_latency_ms=model_latency_ms,
        model_error=model_error,
        speech_source=SPEECH_SOURCE,
        spoken_text=deterministic_text,
        tts_result=tts_result,
    )
