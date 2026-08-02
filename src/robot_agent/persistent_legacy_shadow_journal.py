"""Bounded NDJSON persistence for passive legacy-control shadow records."""

from dataclasses import dataclass, replace
import math
import os
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread, current_thread
from typing import Optional

from .legacy_control_shadow import (
    FailOpenLegacyShadowObserver,
    LegacyControlShadowError,
    LegacyShadowObserverStatus,
    LegacyShadowRecord,
)


DEFAULT_PERSISTENT_SHADOW_QUEUE_CAPACITY = 256
MAX_PERSISTENT_SHADOW_QUEUE_CAPACITY = 4_096
DEFAULT_PERSISTENT_SHADOW_CLOSE_TIMEOUT_SECONDS = 2.0
MAX_PERSISTENT_SHADOW_CLOSE_TIMEOUT_SECONDS = 60.0
_QUEUE_POLL_SECONDS = 0.025


class PersistentLegacyShadowJournalError(RuntimeError):
    """Persistent journal configuration or asynchronous writing failed."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class PersistentLegacyShadowFailure:
    code: str
    error_type: str


@dataclass(frozen=True)
class PersistentLegacyShadowJournalStatus:
    path: str
    capacity: int
    queued_records: int
    written_records: int
    error: Optional[PersistentLegacyShadowFailure]
    closed: bool
    worker_alive: bool


@dataclass(frozen=True)
class PersistentLegacyShadowSessionStatus:
    observer: LegacyShadowObserverStatus
    journal: PersistentLegacyShadowJournalStatus


def _queue_capacity(value: object) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 1 <= value <= MAX_PERSISTENT_SHADOW_QUEUE_CAPACITY
    ):
        raise PersistentLegacyShadowJournalError(
            "invalid_capacity",
            "Persistent shadow queue capacity is invalid",
        )
    return value


def _close_timeout(value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or not 0 <= value <= MAX_PERSISTENT_SHADOW_CLOSE_TIMEOUT_SECONDS
    ):
        raise PersistentLegacyShadowJournalError(
            "invalid_close_timeout",
            "Persistent shadow close timeout is invalid",
        )
    return float(value)


def _journal_path(value: object) -> Path:
    if isinstance(value, bytes):
        raise PersistentLegacyShadowJournalError(
            "invalid_path",
            "Persistent shadow path must be text or path-like",
        )
    try:
        raw = Path(value)
    except (TypeError, ValueError) as error:
        raise PersistentLegacyShadowJournalError(
            "invalid_path",
            "Persistent shadow path is invalid",
        ) from error
    raw_text = str(raw)
    if (
        not raw_text
        or raw.name in ("", ".", "..")
        or any(ord(character) < 32 for character in raw_text)
    ):
        raise PersistentLegacyShadowJournalError(
            "invalid_path",
            "Persistent shadow path must name a file",
        )
    try:
        if raw.is_symlink():
            raise PersistentLegacyShadowJournalError(
                "invalid_path",
                "Persistent shadow path may not be a symbolic link",
            )
        path = raw.absolute()
        parent = path.parent
        if not parent.exists() or not parent.is_dir():
            raise PersistentLegacyShadowJournalError(
                "invalid_parent",
                "Persistent shadow parent directory does not exist",
            )
        if path.exists() and not path.is_file():
            raise PersistentLegacyShadowJournalError(
                "invalid_path",
                "Persistent shadow path must be a regular file",
            )
    except PersistentLegacyShadowJournalError:
        raise
    except (OSError, ValueError) as error:
        raise PersistentLegacyShadowJournalError(
            "invalid_path",
            "Persistent shadow path cannot be inspected",
        ) from error
    return path


def _open_append_stream(path: Path):
    flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(str(path), flags, 0o600)
    try:
        return os.fdopen(
            descriptor,
            "a",
            encoding="utf-8",
            newline="\n",
        )
    except BaseException:
        os.close(descriptor)
        raise


def _failure(code: str, error: Exception) -> PersistentLegacyShadowFailure:
    error_type = type(error).__name__
    if (
        not isinstance(error_type, str)
        or not error_type
        or error_type != error_type.strip()
        or len(error_type) > 128
        or any(ord(character) < 32 for character in error_type)
    ):
        error_type = "Exception"
    return PersistentLegacyShadowFailure(code=code, error_type=error_type)


class PersistentLegacyShadowJournal:
    """Append-only NDJSON sink with one bounded daemon writer.

    ``try_write`` never waits for queue capacity or the journal state lock.
    ``close`` stops admission immediately and waits only for its explicit
    timeout.  On the normal path the worker drains every accepted record,
    flushes the stream, and then closes it.
    """

    def __init__(
        self,
        path: object,
        *,
        capacity: int = DEFAULT_PERSISTENT_SHADOW_QUEUE_CAPACITY,
    ):
        checked_path = _journal_path(path)
        checked_capacity = _queue_capacity(capacity)
        self._path = checked_path
        self._queue = Queue(maxsize=checked_capacity)
        self._closing = Event()
        self._state_lock = Lock()
        self._status = PersistentLegacyShadowJournalStatus(
            path=str(checked_path),
            capacity=checked_capacity,
            queued_records=0,
            written_records=0,
            error=None,
            closed=False,
            worker_alive=True,
        )
        self._worker = Thread(
            target=self._run_writer,
            name="legacy-shadow-ndjson-writer",
            daemon=True,
        )
        self._worker.start()

    @property
    def path(self) -> Path:
        return self._path

    @property
    def status(self) -> PersistentLegacyShadowJournalStatus:
        return replace(
            self._status,
            closed=self._closing.is_set(),
            worker_alive=self._worker.is_alive(),
        )

    def _record_open_error(self, error: Exception) -> None:
        with self._state_lock:
            if self._status.error is None:
                self._status = replace(
                    self._status,
                    error=_failure("open_failed", error),
                )

    def _record_dequeued(self) -> None:
        with self._state_lock:
            self._status = replace(
                self._status,
                queued_records=max(0, self._status.queued_records - 1),
            )

    def _record_write_success(self) -> None:
        with self._state_lock:
            self._status = replace(
                self._status,
                written_records=self._status.written_records + 1,
            )

    def _record_write_error(self, error: Exception) -> None:
        with self._state_lock:
            self._status = replace(
                self._status,
                error=(
                    self._status.error
                    or _failure("write_failed", error)
                ),
            )

    def _record_close_error(self, error: Exception) -> None:
        with self._state_lock:
            if self._status.error is None:
                self._status = replace(
                    self._status,
                    error=_failure("close_failed", error),
                )

    def _should_stop(self) -> bool:
        # Serialize the empty-queue check with the final admission update.  A
        # write already inside try_write when close starts must remain visible
        # to the draining worker before it exits.
        with self._state_lock:
            return self._closing.is_set() and self._queue.empty()

    @staticmethod
    def _write_record(stream, record: LegacyShadowRecord) -> None:
        line = record.to_json()
        if "\n" in line or "\r" in line:
            raise PersistentLegacyShadowJournalError(
                "invalid_record_line",
                "Shadow record JSON must occupy exactly one line",
            )
        payload = line + "\n"
        written = stream.write(payload)
        if written != len(payload):
            raise OSError("short NDJSON write")
        stream.flush()

    def _run_writer(self) -> None:
        try:
            stream = _open_append_stream(self._path)
        except Exception as error:
            self._record_open_error(error)
            return
        try:
            while True:
                if self._should_stop():
                    break
                try:
                    record = self._queue.get(timeout=_QUEUE_POLL_SECONDS)
                except Empty:
                    continue
                self._record_dequeued()
                try:
                    self._write_record(stream, record)
                except Exception as error:
                    self._record_write_error(error)
                    return
                else:
                    self._record_write_success()
                finally:
                    self._queue.task_done()
        finally:
            try:
                stream.close()
            except Exception as error:
                self._record_close_error(error)

    def try_write(self, record: LegacyShadowRecord) -> bool:
        if not isinstance(record, LegacyShadowRecord):
            raise LegacyControlShadowError(
                "invalid_record",
                "Persistent journal accepts only LegacyShadowRecord values",
            )
        if self._closing.is_set():
            return False
        if not self._state_lock.acquire(False):
            raise PersistentLegacyShadowJournalError(
                "admission_busy",
                "Persistent shadow admission state is busy",
            )
        try:
            if self._status.error is not None:
                raise PersistentLegacyShadowJournalError(
                    "writer_failed",
                    "Persistent shadow writer has failed",
                )
            if self._closing.is_set():
                return False
            try:
                self._queue.put_nowait(record)
            except Full:
                raise PersistentLegacyShadowJournalError(
                    "queue_full",
                    "Persistent shadow queue is full",
                )
            self._status = replace(
                self._status,
                queued_records=self._status.queued_records + 1,
            )
            return True
        finally:
            self._state_lock.release()

    def close(
        self,
        timeout_seconds: float = (
            DEFAULT_PERSISTENT_SHADOW_CLOSE_TIMEOUT_SECONDS
        ),
    ) -> bool:
        timeout = _close_timeout(timeout_seconds)
        self.request_close()
        if current_thread() is not self._worker:
            self._worker.join(timeout)
        current = self.status
        return (
            not current.worker_alive
            and current.error is None
            and current.queued_records == 0
        )

    def request_close(self) -> None:
        """Stop admission and let the daemon drain without joining it."""

        self._closing.set()

    def __enter__(self) -> "PersistentLegacyShadowJournal":
        return self

    def __exit__(self, _error_type, _error, _traceback) -> None:
        self.close()


class PersistentLegacyShadowSession:
    """Episode-scoped composition of fail-open observation and persistence."""

    def __init__(
        self,
        *,
        episode_id: str,
        path: object,
        capacity: int = DEFAULT_PERSISTENT_SHADOW_QUEUE_CAPACITY,
    ):
        journal = PersistentLegacyShadowJournal(path, capacity=capacity)
        try:
            observer = FailOpenLegacyShadowObserver(
                episode_id=episode_id,
                try_write=journal.try_write,
            )
        except Exception:
            journal.close()
            raise
        self._journal = journal
        self._observer = observer

    @property
    def path(self) -> Path:
        return self._journal.path

    @property
    def status(self) -> PersistentLegacyShadowSessionStatus:
        return PersistentLegacyShadowSessionStatus(
            observer=self._observer.status,
            journal=self._journal.status,
        )

    def observe(self, stage: str, **facts: object) -> None:
        return self._observer.observe(stage, **facts)

    def disable(self, failed_stage: str, error: Exception) -> None:
        return self._observer.disable(failed_stage, error)

    def close(
        self,
        timeout_seconds: float = (
            DEFAULT_PERSISTENT_SHADOW_CLOSE_TIMEOUT_SECONDS
        ),
    ) -> bool:
        return self._journal.close(timeout_seconds)

    def request_close(self) -> None:
        self._journal.request_close()

    def __enter__(self) -> "PersistentLegacyShadowSession":
        return self

    def __exit__(self, _error_type, _error, _traceback) -> None:
        self.close()


__all__ = (
    "DEFAULT_PERSISTENT_SHADOW_CLOSE_TIMEOUT_SECONDS",
    "DEFAULT_PERSISTENT_SHADOW_QUEUE_CAPACITY",
    "MAX_PERSISTENT_SHADOW_CLOSE_TIMEOUT_SECONDS",
    "MAX_PERSISTENT_SHADOW_QUEUE_CAPACITY",
    "PersistentLegacyShadowFailure",
    "PersistentLegacyShadowJournal",
    "PersistentLegacyShadowJournalError",
    "PersistentLegacyShadowJournalStatus",
    "PersistentLegacyShadowSession",
    "PersistentLegacyShadowSessionStatus",
)
