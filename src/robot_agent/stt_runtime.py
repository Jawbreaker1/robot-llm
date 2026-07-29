"""Independent bounded worker for utterance-level speech transcription.

The runtime is a trust boundary.  It revalidates audio even when callers
already hold a :class:`PCM16Wav`, owns one serialized provider worker, and
keeps cancellation, expiry, and shutdown independent from provider latency.
"""

from __future__ import annotations

from collections import deque, OrderedDict
from dataclasses import dataclass
import math
import queue
import secrets
import threading
import time
from typing import Callable, Mapping, Optional

from .stt_contract import (
    PCM16Wav,
    ProviderTranscription,
    STTContractError,
    TRANSCRIPTION_SCHEMA,
    TranscriptionRequest,
    validate_pcm16_wav,
)
from .stt_provider import (
    STTProviderProtocolError,
    STTProviderTimeoutError,
    STTProviderUnavailableError,
)


STT_JOB_STATUSES = (
    "queued",
    "running",
    "completed",
    "failed",
    "cancelled",
)
CANCELLATION_SCHEMA = "speech-transcription-cancellation/v1"
DEFAULT_STT_QUEUE_CAPACITY = 2
DEFAULT_STT_JOB_CAPACITY = 32
AUTO_DELIVERY_TTL_MS = 30_000
MAX_TERMINAL_TTL_MS = 24 * 60 * 60 * 1_000

_TERMINAL_STATUSES = frozenset(("completed", "failed", "cancelled"))
_EXPIRED_STATUS = "expired"

Clock = Callable[[], int]
IDFactory = Callable[[], str]
EventSink = Callable[[str, Mapping[str, object]], None]


class STTRuntimeError(RuntimeError):
    """Typed runtime failure safe to expose through the local dashboard."""

    def __init__(self, status: int, code: str, message: str):
        self.status = status
        self.code = code
        super().__init__(message)


def _unix_ms() -> int:
    return time.time_ns() // 1_000_000


def _monotonic_ms() -> int:
    return time.monotonic_ns() // 1_000_000


def _new_id() -> str:
    return secrets.token_hex(12)


def _clock_value(name: str, clock: Clock) -> int:
    value = clock()
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= 2**63 - 1
    ):
        raise STTRuntimeError(
            500,
            "invalid_stt_clock",
            "{} returned an invalid value".format(name),
        )
    return value


def _valid_identifier(value: object, maximum: int) -> bool:
    return (
        isinstance(value, str)
        and bool(value)
        and value == value.strip()
        and len(value) <= maximum
        and value.isascii()
        and all(
            character.isalnum() or character in "-_."
            for character in value
        )
    )


@dataclass
class _Job:
    transcription_id: str
    request_id: str
    language_hint: str
    audio_sha256: Optional[str]
    audio_duration_ms: int
    audio_sample_count: int
    audio_bytes: bytes
    created_at_unix_ms: int
    created_at_monotonic_ms: int
    status: str = "queued"
    started_at_unix_ms: Optional[int] = None
    started_at_monotonic_ms: Optional[int] = None
    completed_at_unix_ms: Optional[int] = None
    completed_at_monotonic_ms: Optional[int] = None
    expires_at_monotonic_ms: Optional[int] = None
    result: Optional[ProviderTranscription] = None
    error_code: Optional[str] = None
    provider_inflight: bool = False
    late_result_discarded: bool = False


class STTRuntime:
    """Serialize provider access without blocking chat or motion workers."""

    def __init__(
        self,
        provider,
        *,
        queue_capacity: int = DEFAULT_STT_QUEUE_CAPACITY,
        job_capacity: int = DEFAULT_STT_JOB_CAPACITY,
        terminal_ttl_ms: int = AUTO_DELIVERY_TTL_MS,
        unix_clock_ms: Clock = _unix_ms,
        monotonic_clock_ms: Clock = _monotonic_ms,
        id_factory: IDFactory = _new_id,
        event_sink: Optional[EventSink] = None,
    ):
        transcribe = getattr(provider, "transcribe", None)
        provider_id = getattr(provider, "provider_id", None)
        model_id = getattr(provider, "model_id", None)
        if (
            not callable(transcribe)
            or not _valid_identifier(provider_id, 128)
            or not _valid_identifier(model_id, 200)
            or isinstance(queue_capacity, bool)
            or not isinstance(queue_capacity, int)
            or not 1 <= queue_capacity <= 16
            or isinstance(job_capacity, bool)
            or not isinstance(job_capacity, int)
            or not queue_capacity <= job_capacity <= 256
            or isinstance(terminal_ttl_ms, bool)
            or not isinstance(terminal_ttl_ms, int)
            or not 1 <= terminal_ttl_ms <= MAX_TERMINAL_TTL_MS
            or not callable(unix_clock_ms)
            or not callable(monotonic_clock_ms)
            or not callable(id_factory)
            or event_sink is not None
            and not callable(event_sink)
        ):
            raise STTRuntimeError(
                500,
                "invalid_stt_configuration",
                "Speech runtime configuration is invalid",
            )
        self._provider = provider
        self._provider_id = provider_id
        self._model_id = model_id
        self._terminal_ttl_ms = terminal_ttl_ms
        self._queue_capacity = queue_capacity
        self._job_capacity = job_capacity
        self._unix_clock_ms = unix_clock_ms
        self._monotonic_clock_ms = monotonic_clock_ms
        self._id_factory = id_factory
        self._event_sink = event_sink
        self._jobs = {}
        self._request_index = {}
        self._lock = threading.RLock()
        self._pending_condition = threading.Condition(self._lock)
        self._pending = deque()
        self._pre_cancelled_requests = OrderedDict()
        self._accepting = True
        self._shutdown_started = False

        self._event_stop = threading.Event()
        self._event_queue = None
        self._event_worker = None
        self._event_dropped_total = 0
        if event_sink is not None:
            self._event_queue = queue.Queue(
                maxsize=job_capacity * 6 + 16
            )
            self._event_worker = threading.Thread(
                target=self._dispatch_events,
                name="robot-llm-stt-events",
                daemon=True,
            )
            self._event_worker.start()

        self._worker = threading.Thread(
            target=self._work,
            name="robot-llm-stt",
            daemon=True,
        )
        self._worker.start()

    @property
    def provider_id(self) -> str:
        return self._provider_id

    @property
    def model_id(self) -> str:
        return self._model_id

    @property
    def terminal_ttl_ms(self) -> int:
        return self._terminal_ttl_ms

    def _schedule_event_locked(
        self,
        event_type: str,
        data: Mapping[str, object],
    ) -> None:
        """Queue metadata while locked; never invoke the sink while locked."""

        if self._event_queue is None:
            return
        try:
            self._event_queue.put_nowait(
                (event_type, dict(data))
            )
        except queue.Full:
            self._event_dropped_total += 1

    def _dispatch_events(self) -> None:
        event_queue = self._event_queue
        sink = self._event_sink
        if event_queue is None or sink is None:
            return
        while True:
            if self._event_stop.is_set() and event_queue.empty():
                return
            try:
                event_type, data = event_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                sink(event_type, data)
            except Exception:
                # Observability may never change transcription state.
                pass
            finally:
                event_queue.task_done()

    def _expired_error(self) -> STTRuntimeError:
        return STTRuntimeError(
            410,
            "stt_expired",
            "Speech transcription result has expired",
        )

    def _expire_job_locked(self, job: _Job) -> None:
        previous_status = job.status
        job.status = _EXPIRED_STATUS
        job.result = None
        job.audio_bytes = b""
        job.audio_sha256 = None
        self._schedule_event_locked(
            "stt.transcription_expired",
            {
                "transcription_id": job.transcription_id,
                "request_id": job.request_id,
                "provider_id": self._provider_id,
                "previous_status": previous_status,
                "status": _EXPIRED_STATUS,
            },
        )

    def _expire_due_locked(self, now_monotonic_ms: int) -> None:
        for job in self._jobs.values():
            if (
                job.status in _TERMINAL_STATUSES
                and job.expires_at_monotonic_ms is not None
                and now_monotonic_ms >= job.expires_at_monotonic_ms
            ):
                self._expire_job_locked(job)

    def _prune_pre_cancelled_locked(
        self,
        now_monotonic_ms: int,
    ) -> None:
        while self._pre_cancelled_requests:
            request_id, expires_at = next(
                iter(self._pre_cancelled_requests.items())
            )
            if now_monotonic_ms < expires_at:
                return
            self._pre_cancelled_requests.pop(request_id, None)

    def _evict_one_locked(self) -> bool:
        terminal = [
            job
            for job in self._jobs.values()
            if (
                job.status in _TERMINAL_STATUSES
                or job.status == _EXPIRED_STATUS
            )
            and not job.provider_inflight
        ]
        if not terminal:
            return False
        oldest = min(
            terminal,
            key=lambda item: (
                item.completed_at_monotonic_ms
                if item.completed_at_monotonic_ms is not None
                else item.created_at_monotonic_ms
            ),
        )
        oldest.result = None
        oldest.audio_bytes = b""
        oldest.audio_sha256 = None
        self._jobs.pop(oldest.transcription_id, None)
        self._request_index.pop(oldest.request_id, None)
        return True

    @staticmethod
    def _trusted_audio(request: TranscriptionRequest) -> PCM16Wav:
        try:
            trusted = validate_pcm16_wav(request.audio.wav_bytes)
        except STTContractError as error:
            raise STTRuntimeError(
                400,
                error.code,
                str(error),
            ) from None
        supplied = request.audio
        if (
            supplied.duration_ms != trusted.duration_ms
            or supplied.sample_count != trusted.sample_count
            or supplied.sha256 != trusted.sha256
        ):
            raise STTRuntimeError(
                400,
                "invalid_stt_audio_metadata",
                "Speech audio metadata does not match its WAV bytes",
            )
        return trusted

    def submit(
        self,
        request: TranscriptionRequest,
    ) -> Mapping[str, object]:
        if not isinstance(request, TranscriptionRequest):
            raise STTRuntimeError(
                400,
                "invalid_stt_request",
                "Speech transcription request is invalid",
            )
        try:
            request = TranscriptionRequest(
                request_id=request.request_id,
                language_hint=request.language_hint,
                audio=request.audio,
            )
        except (AttributeError, STTContractError) as error:
            raise STTRuntimeError(
                400,
                getattr(error, "code", "invalid_stt_request"),
                "Speech transcription request is invalid",
            ) from None
        trusted_audio = self._trusted_audio(request)
        with self._lock:
            now_monotonic_ms = _clock_value(
                "monotonic_clock_ms",
                self._monotonic_clock_ms,
            )
            self._expire_due_locked(now_monotonic_ms)
            self._prune_pre_cancelled_locked(now_monotonic_ms)
            if request.request_id in self._pre_cancelled_requests:
                raise STTRuntimeError(
                    409,
                    "stt_request_cancelled",
                    "Speech transcription request was cancelled",
                )
            existing_id = self._request_index.get(request.request_id)
            if existing_id is not None:
                existing = self._jobs[existing_id]
                if existing.status == _EXPIRED_STATUS:
                    raise self._expired_error()
                if (
                    existing.audio_sha256 != trusted_audio.sha256
                    or existing.language_hint != request.language_hint
                ):
                    raise STTRuntimeError(
                        409,
                        "stt_idempotency_conflict",
                        "Speech request ID was reused with other input",
                    )
                return self._view_locked(existing)
            if not self._accepting:
                raise STTRuntimeError(
                    503,
                    "stt_stopping",
                    "Speech transcription runtime is stopping",
                )
            while len(self._jobs) >= self._job_capacity:
                if not self._evict_one_locked():
                    raise STTRuntimeError(
                        429,
                        "stt_job_store_full",
                        "Speech transcription job store is full",
                    )
            raw_id = self._id_factory()
            if not _valid_identifier(raw_id, 96):
                raise STTRuntimeError(
                    500,
                    "invalid_stt_id_factory",
                    "Speech job ID factory returned an invalid value",
                )
            transcription_id = "stt-" + raw_id
            if transcription_id in self._jobs:
                raise STTRuntimeError(
                    500,
                    "duplicate_stt_id",
                    "Speech job ID was duplicated",
                )
            job = _Job(
                transcription_id=transcription_id,
                request_id=request.request_id,
                language_hint=request.language_hint,
                audio_sha256=trusted_audio.sha256,
                audio_duration_ms=trusted_audio.duration_ms,
                audio_sample_count=trusted_audio.sample_count,
                audio_bytes=trusted_audio.wav_bytes,
                created_at_unix_ms=_clock_value(
                    "unix_clock_ms",
                    self._unix_clock_ms,
                ),
                created_at_monotonic_ms=now_monotonic_ms,
            )
            self._jobs[transcription_id] = job
            self._request_index[request.request_id] = transcription_id
            if len(self._pending) >= self._queue_capacity:
                job.audio_bytes = b""
                job.audio_sha256 = None
                self._jobs.pop(transcription_id, None)
                self._request_index.pop(request.request_id, None)
                raise STTRuntimeError(
                    429,
                    "stt_queue_full",
                    "Speech transcription queue is full",
                )
            self._pending.append(transcription_id)
            self._pending_condition.notify()
            self._schedule_event_locked(
                "stt.transcription_queued",
                {
                    "transcription_id": transcription_id,
                    "request_id": request.request_id,
                    "language_hint": request.language_hint,
                    "audio_duration_ms": trusted_audio.duration_ms,
                    "provider_id": self._provider_id,
                    "status": job.status,
                },
            )
            return self._view_locked(job)

    @staticmethod
    def _validate_transcription_id(transcription_id: object) -> str:
        if not _valid_identifier(transcription_id, 128):
            raise STTRuntimeError(
                400,
                "invalid_stt_identifier",
                "Speech transcription ID is invalid",
            )
        return transcription_id

    def get(self, transcription_id: str) -> Mapping[str, object]:
        transcription_id = self._validate_transcription_id(
            transcription_id
        )
        with self._lock:
            self._expire_due_locked(
                _clock_value(
                    "monotonic_clock_ms",
                    self._monotonic_clock_ms,
                )
            )
            job = self._jobs.get(transcription_id)
            if job is None:
                raise STTRuntimeError(
                    404,
                    "stt_not_found",
                    "Speech transcription was not found",
                )
            if job.status == _EXPIRED_STATUS:
                raise self._expired_error()
            return self._view_locked(job)

    def cancel(self, transcription_id: str) -> Mapping[str, object]:
        transcription_id = self._validate_transcription_id(
            transcription_id
        )
        with self._lock:
            self._expire_due_locked(
                _clock_value(
                    "monotonic_clock_ms",
                    self._monotonic_clock_ms,
                )
            )
            job = self._jobs.get(transcription_id)
            if job is None:
                raise STTRuntimeError(
                    404,
                    "stt_not_found",
                    "Speech transcription was not found",
                )
            if job.status == _EXPIRED_STATUS:
                raise self._expired_error()
            if job.status == "cancelled":
                return self._view_locked(job)
            if job.status in ("completed", "failed"):
                raise STTRuntimeError(
                    409,
                    "stt_not_cancellable",
                    "Speech transcription is already terminal",
                )
            if job.status == "queued":
                try:
                    self._pending.remove(transcription_id)
                except ValueError:
                    pass
            self._terminalize_locked(
                job,
                status="cancelled",
                error_code="stt_cancelled",
            )
            self._schedule_cancelled_event_locked(
                job,
                reason_code="user_cancelled",
            )
            return self._view_locked(job)

    def cancel_request(self, request_id: str) -> Mapping[str, object]:
        if not _valid_identifier(request_id, 128):
            raise STTRuntimeError(
                400,
                "invalid_stt_identifier",
                "Speech request ID is invalid",
            )
        with self._lock:
            now_monotonic_ms = _clock_value(
                "monotonic_clock_ms",
                self._monotonic_clock_ms,
            )
            self._expire_due_locked(now_monotonic_ms)
            self._prune_pre_cancelled_locked(now_monotonic_ms)
            transcription_id = self._request_index.get(request_id)
            if transcription_id is None:
                while (
                    len(self._pre_cancelled_requests)
                    >= self._job_capacity
                ):
                    self._pre_cancelled_requests.popitem(last=False)
                self._pre_cancelled_requests[request_id] = (
                    now_monotonic_ms + self._terminal_ttl_ms
                )
                self._pre_cancelled_requests.move_to_end(request_id)
                self._schedule_event_locked(
                    "stt.request_cancelled",
                    {
                        "request_id": request_id,
                        "provider_id": self._provider_id,
                        "status": "cancelled",
                        "reason_code": "user_cancelled",
                    },
                )
                now_unix_ms = _clock_value(
                    "unix_clock_ms",
                    self._unix_clock_ms,
                )
                return {
                    "schema": CANCELLATION_SCHEMA,
                    "request_id": request_id,
                    "transcription_id": None,
                    "status": "cancelled",
                    "valid_until_unix_ms": (
                        now_unix_ms + self._terminal_ttl_ms
                    ),
                }
            job = self._jobs.get(transcription_id)
            if job is None:
                raise STTRuntimeError(
                    404,
                    "stt_not_found",
                    "Speech transcription was not found",
                )
            if job.status == _EXPIRED_STATUS:
                raise self._expired_error()
            if job.status == "cancelled":
                return self._view_locked(job)
            if job.status == "queued":
                try:
                    self._pending.remove(transcription_id)
                except ValueError:
                    pass
            self._terminalize_locked(
                job,
                status="cancelled",
                error_code="stt_cancelled",
            )
            self._schedule_cancelled_event_locked(
                job,
                reason_code="user_cancelled_by_request",
            )
            return self._view_locked(job)

    def _view_locked(self, job: _Job) -> Mapping[str, object]:
        queue_wait_ms = None
        provider_latency_ms = None
        total_latency_ms = None
        if job.started_at_monotonic_ms is not None:
            queue_wait_ms = max(
                0,
                job.started_at_monotonic_ms
                - job.created_at_monotonic_ms,
            )
        if (
            job.started_at_monotonic_ms is not None
            and job.completed_at_monotonic_ms is not None
        ):
            provider_latency_ms = max(
                0,
                job.completed_at_monotonic_ms
                - job.started_at_monotonic_ms,
            )
        if job.completed_at_monotonic_ms is not None:
            total_latency_ms = max(
                0,
                job.completed_at_monotonic_ms
                - job.created_at_monotonic_ms,
            )
        value = {
            "schema": TRANSCRIPTION_SCHEMA,
            "transcription_id": job.transcription_id,
            "request_id": job.request_id,
            "status": job.status,
            "requested_language": job.language_hint,
            "audio": {
                "duration_ms": job.audio_duration_ms,
                "retained": (
                    bool(job.audio_bytes)
                    or job.provider_inflight
                ),
            },
            "provider": {
                "provider_id": self._provider_id,
                "model_id": self._model_id,
            },
            "timing": {
                "queued_at_unix_ms": job.created_at_unix_ms,
                "started_at_unix_ms": job.started_at_unix_ms,
                "completed_at_unix_ms": job.completed_at_unix_ms,
                "queue_wait_ms": queue_wait_ms,
                "provider_latency_ms": provider_latency_ms,
                "total_latency_ms": total_latency_ms,
            },
        }
        if job.result is not None:
            value.update(
                {
                    "text": job.result.text,
                    "detected_language": (
                        job.result.detected_language
                    ),
                    "provider_score": job.result.provider_score,
                    "valid_until_unix_ms": (
                        job.completed_at_unix_ms
                        + self._terminal_ttl_ms
                        if job.completed_at_unix_ms is not None
                        else None
                    ),
                }
            )
        if job.error_code is not None:
            value["error_code"] = job.error_code
        if job.status == "cancelled":
            value["provider_work_pending"] = job.provider_inflight
            value["late_provider_result_discarded"] = (
                job.late_result_discarded
            )
        return value

    def _terminalize_locked(
        self,
        job: _Job,
        *,
        status: str,
        error_code: Optional[str],
        result: Optional[ProviderTranscription] = None,
    ) -> None:
        completed_at_unix_ms = _clock_value(
            "unix_clock_ms",
            self._unix_clock_ms,
        )
        completed_at_monotonic_ms = _clock_value(
            "monotonic_clock_ms",
            self._monotonic_clock_ms,
        )
        job.status = status
        job.error_code = error_code
        job.result = result
        job.completed_at_unix_ms = completed_at_unix_ms
        job.completed_at_monotonic_ms = completed_at_monotonic_ms
        job.expires_at_monotonic_ms = (
            completed_at_monotonic_ms + self._terminal_ttl_ms
        )
        job.audio_bytes = b""

    def _schedule_cancelled_event_locked(
        self,
        job: _Job,
        *,
        reason_code: str,
    ) -> None:
        self._schedule_event_locked(
            "stt.transcription_cancelled",
            {
                "transcription_id": job.transcription_id,
                "request_id": job.request_id,
                "audio_duration_ms": job.audio_duration_ms,
                "provider_id": self._provider_id,
                "status": job.status,
                "reason_code": reason_code,
                "provider_work_pending": job.provider_inflight,
            },
        )

    def _work(self) -> None:
        try:
            while True:
                with self._pending_condition:
                    while (
                        not self._pending
                        and not self._shutdown_started
                    ):
                        self._expire_due_locked(
                            _clock_value(
                                "monotonic_clock_ms",
                                self._monotonic_clock_ms,
                            )
                        )
                        self._pending_condition.wait(timeout=0.1)
                    if not self._pending:
                        return
                    transcription_id = self._pending.popleft()
                    job = self._jobs.get(transcription_id)
                    if job is None or job.status != "queued":
                        continue
                    job.status = "running"
                    job.started_at_unix_ms = _clock_value(
                        "unix_clock_ms",
                        self._unix_clock_ms,
                    )
                    job.started_at_monotonic_ms = _clock_value(
                        "monotonic_clock_ms",
                        self._monotonic_clock_ms,
                    )
                    job.provider_inflight = True
                    raw_audio = job.audio_bytes
                    audio = PCM16Wav(
                        wav_bytes=raw_audio,
                        duration_ms=job.audio_duration_ms,
                        sample_count=job.audio_sample_count,
                        sha256=job.audio_sha256 or "",
                    )
                    provider_request = TranscriptionRequest(
                        request_id=job.request_id,
                        language_hint=job.language_hint,
                        audio=audio,
                    )
                    self._schedule_event_locked(
                        "stt.transcription_started",
                        {
                            "transcription_id": (
                                job.transcription_id
                            ),
                            "request_id": job.request_id,
                            "audio_duration_ms": (
                                job.audio_duration_ms
                            ),
                            "provider_id": self._provider_id,
                            "status": job.status,
                        },
                    )
                error_code = None
                result = None
                try:
                    result = self._provider.transcribe(
                        provider_request
                    )
                    if (
                        not isinstance(
                            result,
                            ProviderTranscription,
                        )
                        or result.provider_id != self._provider_id
                        or result.model_id != self._model_id
                    ):
                        raise STTProviderProtocolError(
                            "Speech provider returned an "
                            "invalid result"
                        )
                except STTProviderTimeoutError:
                    error_code = "stt_provider_timeout"
                except STTProviderUnavailableError:
                    error_code = "stt_provider_unavailable"
                except (
                    STTProviderProtocolError,
                    STTContractError,
                ):
                    error_code = "stt_provider_protocol"
                except Exception:
                    error_code = "stt_provider_failed"
                finally:
                    # Do not retain raw audio in the idle worker frame.
                    provider_request = None
                    audio = None
                    raw_audio = b""

                try:
                    with self._lock:
                        current = self._jobs.get(transcription_id)
                        if current is not job:
                            continue
                        job.provider_inflight = False
                        if job.status != "running":
                            job.late_result_discarded = True
                            self._schedule_event_locked(
                                "stt.transcription_late_result_discarded",
                                {
                                    "transcription_id": (
                                        job.transcription_id
                                    ),
                                    "request_id": job.request_id,
                                    "provider_id": self._provider_id,
                                    "status": job.status,
                                    "late_result_discarded": True,
                                },
                            )
                            continue
                        if error_code is not None:
                            self._terminalize_locked(
                                job,
                                status="failed",
                                error_code=error_code,
                            )
                            event_type = "stt.transcription_failed"
                        else:
                            self._terminalize_locked(
                                job,
                                status="completed",
                                error_code=None,
                                result=result,
                            )
                            event_type = "stt.transcription_completed"
                        provider_latency_ms = max(
                            0,
                            job.completed_at_monotonic_ms
                            - job.started_at_monotonic_ms,
                        )
                        self._schedule_event_locked(
                            event_type,
                            {
                                "transcription_id": (
                                    job.transcription_id
                                ),
                                "request_id": job.request_id,
                                "audio_duration_ms": (
                                    job.audio_duration_ms
                                ),
                                "provider_id": self._provider_id,
                                "status": job.status,
                                "error_code": error_code,
                                "provider_latency_ms": (
                                    provider_latency_ms
                                ),
                            },
                        )
                finally:
                    # The authoritative job owns a fresh transcript only
                    # until TTL expiry; the idle worker frame owns none.
                    result = None
        finally:
            with self._lock:
                should_stop_events = self._shutdown_started
            if should_stop_events:
                self._event_stop.set()

    def shutdown(self, timeout_seconds: float = 1.0):
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not math.isfinite(float(timeout_seconds))
            or timeout_seconds < 0
        ):
            raise STTRuntimeError(
                500,
                "invalid_stt_shutdown_timeout",
                "Speech runtime shutdown timeout is invalid",
            )
        deadline = time.monotonic() + float(timeout_seconds)
        with self._lock:
            if not self._shutdown_started:
                self._shutdown_started = True
                self._accepting = False
                cancelled_total = 0
                for job in self._jobs.values():
                    if job.status in ("queued", "running"):
                        self._terminalize_locked(
                            job,
                            status="cancelled",
                            error_code="stt_stopping",
                        )
                        self._schedule_cancelled_event_locked(
                            job,
                            reason_code="runtime_shutdown",
                        )
                        cancelled_total += 1
                self._pending.clear()
                self._pre_cancelled_requests.clear()
                self._schedule_event_locked(
                    "stt.runtime_shutdown",
                    {
                        "provider_id": self._provider_id,
                        "cancelled_total": cancelled_total,
                        "status": "stopping",
                    },
                )
                self._pending_condition.notify_all()
                if not self._worker.is_alive():
                    self._event_stop.set()

        if self._worker is not threading.current_thread():
            self._worker.join(
                max(0.0, deadline - time.monotonic())
            )
        if not self._worker.is_alive():
            self._event_stop.set()
        event_worker = self._event_worker
        if (
            event_worker is not None
            and event_worker is not threading.current_thread()
        ):
            event_worker.join(
                max(0.0, deadline - time.monotonic())
            )
        with self._lock:
            queued_remaining = sum(
                job.status == "queued"
                for job in self._jobs.values()
            )
            running_remaining = sum(
                job.provider_inflight
                for job in self._jobs.values()
            )
            event_dropped_total = self._event_dropped_total
        event_worker_alive = bool(
            event_worker is not None and event_worker.is_alive()
        )
        worker_alive = self._worker.is_alive()
        return {
            "worker_alive": worker_alive,
            "event_worker_alive": event_worker_alive,
            "timed_out": worker_alive or event_worker_alive,
            "queued_remaining": queued_remaining,
            "provider_work_pending": running_remaining,
            "event_dropped_total": event_dropped_total,
        }
