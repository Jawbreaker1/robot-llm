"""Passive, bounded journal for comparing legacy control with canonical state.

The observer in this module owns no authority and performs no work outside an
in-memory record write.  One observer is bound to one physical-navigation
episode.  If record construction or its injected non-blocking writer fails,
the observer disables itself for that episode and the legacy runtime carries
on unchanged.
"""

from dataclasses import dataclass, field, replace
import json
from threading import Event, Lock
from typing import Callable, Mapping, Optional, Sequence, Tuple


LEGACY_CONTROL_SHADOW_SCHEMA = "robot-legacy-control-shadow"
LEGACY_CONTROL_SHADOW_VERSION = 1
SHADOW_DISABLED_STAGE = "shadow_disabled"

DEFAULT_LEGACY_SHADOW_JOURNAL_CAPACITY = 256
MAX_LEGACY_SHADOW_JOURNAL_CAPACITY = 4_096
MAX_LEGACY_SHADOW_FACT_BYTES = 16 * 1024
MAX_LEGACY_SHADOW_FACT_KEYS = 32
MAX_LEGACY_SHADOW_FACT_NODES = 512
MAX_LEGACY_SHADOW_FACT_DEPTH = 8
MAX_LEGACY_SHADOW_TEXT_LENGTH = 4_096
MAX_LEGACY_SHADOW_IDENTIFIER_LENGTH = 128
_MIN_INT = -(2**63)
_MAX_INT = 2**63 - 1


class LegacyControlShadowError(ValueError):
    """A shadow record violates the bounded replay contract."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _identifier(name: str, value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > MAX_LEGACY_SHADOW_IDENTIFIER_LENGTH
        or any(ord(character) < 32 for character in value)
    ):
        raise LegacyControlShadowError(
            "invalid_identifier",
            "{} is invalid".format(name),
        )
    return value


def _integer(name: str, value: object, minimum: int, maximum: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not minimum <= value <= maximum
    ):
        raise LegacyControlShadowError(
            "invalid_integer",
            "{} is invalid".format(name),
        )
    return value


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("non-finite number")


def _normalise_fact_value(
    value: object,
    *,
    depth: int,
    node_count: list,
) -> object:
    if depth > MAX_LEGACY_SHADOW_FACT_DEPTH:
        raise LegacyControlShadowError(
            "facts_too_deep",
            "Shadow facts exceed the nesting limit",
        )
    node_count[0] += 1
    if node_count[0] > MAX_LEGACY_SHADOW_FACT_NODES:
        raise LegacyControlShadowError(
            "too_many_fact_nodes",
            "Shadow facts exceed the node limit",
        )

    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, int):
        return _integer("fact integer", value, _MIN_INT, _MAX_INT)
    if isinstance(value, str):
        if len(value) > MAX_LEGACY_SHADOW_TEXT_LENGTH:
            raise LegacyControlShadowError(
                "fact_text_too_long",
                "A shadow fact string exceeds the text limit",
            )
        return value
    if isinstance(value, Mapping):
        keys = tuple(value.keys())
        if any(not isinstance(key, str) for key in keys):
            raise LegacyControlShadowError(
                "invalid_fact_key",
                "Shadow fact object keys must be strings",
            )
        result = {}
        for key in sorted(keys):
            _identifier("fact key", key)
            result[key] = _normalise_fact_value(
                value[key],
                depth=depth + 1,
                node_count=node_count,
            )
        return result
    if isinstance(value, Sequence) and not isinstance(
        value, (str, bytes, bytearray)
    ):
        return [
            _normalise_fact_value(
                item,
                depth=depth + 1,
                node_count=node_count,
            )
            for item in value
        ]
    raise LegacyControlShadowError(
        "unsupported_fact_value",
        "Shadow facts support only JSON scalar, object, and array values",
    )


def _canonical_facts(value: Mapping[str, object]) -> str:
    if not isinstance(value, Mapping):
        raise LegacyControlShadowError(
            "invalid_facts",
            "Shadow facts must be a mapping",
        )
    if len(value) > MAX_LEGACY_SHADOW_FACT_KEYS:
        raise LegacyControlShadowError(
            "too_many_facts",
            "Shadow facts exceed the top-level key limit",
        )
    normalised = _normalise_fact_value(
        value,
        depth=0,
        node_count=[0],
    )
    encoded = json.dumps(
        normalised,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    if len(encoded.encode("utf-8")) > MAX_LEGACY_SHADOW_FACT_BYTES:
        raise LegacyControlShadowError(
            "facts_too_large",
            "Shadow facts exceed the byte limit",
        )
    return encoded


@dataclass(frozen=True)
class FrozenLegacyShadowFacts:
    """Canonical JSON storage with a fresh JSON-compatible view per read."""

    canonical_json: str = field(repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self.canonical_json, str):
            raise LegacyControlShadowError(
                "invalid_facts",
                "Canonical shadow facts must be text",
            )
        try:
            decoded = json.loads(
                self.canonical_json,
                object_pairs_hook=_strict_object,
                parse_constant=_reject_constant,
            )
        except (TypeError, ValueError) as error:
            raise LegacyControlShadowError(
                "invalid_facts",
                "Canonical shadow facts are not strict JSON",
            ) from error
        if not isinstance(decoded, dict):
            raise LegacyControlShadowError(
                "invalid_facts",
                "Canonical shadow facts must contain an object",
            )
        if _canonical_facts(decoded) != self.canonical_json:
            raise LegacyControlShadowError(
                "noncanonical_facts",
                "Shadow facts must use their canonical representation",
            )

    @classmethod
    def capture(
        cls,
        facts: Mapping[str, object],
    ) -> "FrozenLegacyShadowFacts":
        return cls(_canonical_facts(facts))

    def to_dict(self) -> Mapping[str, object]:
        return json.loads(self.canonical_json)


@dataclass(frozen=True)
class LegacyShadowRecord:
    """One immutable, schema-versioned fact in deterministic episode order."""

    schema: str
    version: int
    episode_id: str
    sequence: int
    stage: str
    facts: FrozenLegacyShadowFacts

    def __post_init__(self) -> None:
        if self.schema != LEGACY_CONTROL_SHADOW_SCHEMA:
            raise LegacyControlShadowError(
                "invalid_schema",
                "Shadow record schema is unsupported",
            )
        if (
            isinstance(self.version, bool)
            or not isinstance(self.version, int)
            or self.version != LEGACY_CONTROL_SHADOW_VERSION
        ):
            raise LegacyControlShadowError(
                "invalid_version",
                "Shadow record version is unsupported",
            )
        _identifier("episode_id", self.episode_id)
        _integer("sequence", self.sequence, 1, _MAX_INT)
        _identifier("stage", self.stage)
        if not isinstance(self.facts, FrozenLegacyShadowFacts):
            raise LegacyControlShadowError(
                "invalid_facts",
                "Shadow record facts are not frozen",
            )

    @classmethod
    def capture(
        cls,
        *,
        episode_id: str,
        sequence: int,
        stage: str,
        facts: Mapping[str, object],
    ) -> "LegacyShadowRecord":
        return cls(
            schema=LEGACY_CONTROL_SHADOW_SCHEMA,
            version=LEGACY_CONTROL_SHADOW_VERSION,
            episode_id=episode_id,
            sequence=sequence,
            stage=stage,
            facts=FrozenLegacyShadowFacts.capture(facts),
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": self.schema,
            "version": self.version,
            "episode_id": self.episode_id,
            "sequence": self.sequence,
            "stage": self.stage,
            "facts": self.facts.to_dict(),
        }

    def to_json(self) -> str:
        return json.dumps(
            self.to_dict(),
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )


@dataclass(frozen=True)
class LegacyShadowFailureSummary:
    failed_sequence: int
    failed_stage: str
    failure_type: str


@dataclass(frozen=True)
class LegacyShadowObserverStatus:
    episode_id: str
    enabled: bool
    next_sequence: int
    written_records: int
    dropped_records: int
    disabled_record_emitted: bool
    failure: Optional[LegacyShadowFailureSummary]


@dataclass(frozen=True)
class LegacyShadowJournalStatus:
    capacity: int
    retained_records: int
    written_records: int
    evicted_records: int


class InMemoryLegacyShadowJournal:
    """A bounded sink whose write and snapshot paths never wait for its lock."""

    def __init__(
        self,
        capacity: int = DEFAULT_LEGACY_SHADOW_JOURNAL_CAPACITY,
    ):
        checked_capacity = _integer(
            "capacity",
            capacity,
            1,
            MAX_LEGACY_SHADOW_JOURNAL_CAPACITY,
        )
        self._capacity = checked_capacity
        self._records = ()
        self._status = LegacyShadowJournalStatus(
            capacity=checked_capacity,
            retained_records=0,
            written_records=0,
            evicted_records=0,
        )
        self._lock = Lock()

    @property
    def status(self) -> LegacyShadowJournalStatus:
        return self._status

    def snapshot(self) -> Tuple[LegacyShadowRecord, ...]:
        return self._records

    def try_write(self, record: LegacyShadowRecord) -> bool:
        if not isinstance(record, LegacyShadowRecord):
            raise LegacyControlShadowError(
                "invalid_record",
                "Journal accepts only LegacyShadowRecord values",
            )
        if not self._lock.acquire(False):
            return False
        try:
            records = self._records + (record,)
            evicted = max(0, len(records) - self._capacity)
            if evicted:
                records = records[evicted:]
            self._records = records
            self._status = LegacyShadowJournalStatus(
                capacity=self._capacity,
                retained_records=len(records),
                written_records=self._status.written_records + 1,
                evicted_records=self._status.evicted_records + evicted,
            )
            return True
        finally:
            self._lock.release()


class FailOpenLegacyShadowObserver:
    """Episode-bound observer that permanently isolates its first failure."""

    def __init__(
        self,
        *,
        episode_id: str,
        try_write: Callable[[LegacyShadowRecord], bool],
    ):
        checked_episode_id = _identifier("episode_id", episode_id)
        if not callable(try_write):
            raise LegacyControlShadowError(
                "invalid_writer",
                "try_write must be callable",
            )
        self._try_write = try_write
        self._status = LegacyShadowObserverStatus(
            episode_id=checked_episode_id,
            enabled=True,
            next_sequence=1,
            written_records=0,
            dropped_records=0,
            disabled_record_emitted=False,
            failure=None,
        )
        self._lock = Lock()
        self._contention = Event()

    @property
    def status(self) -> LegacyShadowObserverStatus:
        status = self._status
        if self._contention.is_set() and status.enabled:
            return replace(
                status,
                enabled=False,
                failure=LegacyShadowFailureSummary(
                    failed_sequence=status.next_sequence,
                    failed_stage="observer_contention",
                    failure_type="LegacyControlShadowError",
                ),
            )
        return status

    @property
    def enabled(self) -> bool:
        return self.status.enabled

    def _disable_after_contention(self) -> None:
        if not self._contention.is_set() or not self._status.enabled:
            return
        failed_sequence = self._status.next_sequence
        self._status = replace(
            self._status,
            next_sequence=failed_sequence + 1,
        )
        self._disable(
            failed_sequence,
            "observer_contention",
            LegacyControlShadowError(
                "observer_busy",
                "Concurrent shadow observation was rejected",
            ),
        )

    def _disable(
        self,
        failed_sequence: int,
        failed_stage: object,
        error: Exception,
    ) -> None:
        safe_stage = (
            failed_stage
            if isinstance(failed_stage, str)
            and bool(failed_stage)
            and failed_stage == failed_stage.strip()
            and len(failed_stage) <= MAX_LEGACY_SHADOW_IDENTIFIER_LENGTH
            and not any(ord(character) < 32 for character in failed_stage)
            else "invalid_stage"
        )
        failure_type = type(error).__name__
        if (
            not isinstance(failure_type, str)
            or not failure_type
            or failure_type != failure_type.strip()
            or len(failure_type) > MAX_LEGACY_SHADOW_IDENTIFIER_LENGTH
            or any(ord(character) < 32 for character in failure_type)
        ):
            failure_type = "Exception"
        disabled_sequence = self._status.next_sequence
        failure = LegacyShadowFailureSummary(
            failed_sequence=failed_sequence,
            failed_stage=safe_stage,
            failure_type=failure_type,
        )
        self._status = replace(
            self._status,
            enabled=False,
            next_sequence=disabled_sequence + 1,
            failure=failure,
        )
        try:
            disabled_record = LegacyShadowRecord.capture(
                episode_id=self._status.episode_id,
                sequence=disabled_sequence,
                stage=SHADOW_DISABLED_STAGE,
                facts={
                    "failed_sequence": failed_sequence,
                    "failed_stage": safe_stage,
                    "failure_type": failure_type,
                },
            )
            accepted = self._try_write(disabled_record)
        except Exception:
            return
        if accepted:
            self._status = replace(
                self._status,
                written_records=self._status.written_records + 1,
                disabled_record_emitted=True,
            )
        else:
            self._status = replace(
                self._status,
                dropped_records=self._status.dropped_records + 1,
            )

    def observe(self, stage: str, **facts: object) -> None:
        """Best-effort record of one stage; never grants or delays authority."""

        if not self._lock.acquire(False):
            self._contention.set()
            return None
        try:
            self._disable_after_contention()
            if not self._status.enabled:
                return None
            sequence = self._status.next_sequence
            self._status = replace(
                self._status,
                next_sequence=sequence + 1,
            )
            try:
                record = LegacyShadowRecord.capture(
                    episode_id=self._status.episode_id,
                    sequence=sequence,
                    stage=stage,
                    facts=facts,
                )
                accepted = self._try_write(record)
                if accepted is not True:
                    raise LegacyControlShadowError(
                        "write_rejected",
                        "Shadow writer rejected a mandatory record",
                    )
            except Exception as error:
                self._disable(sequence, stage, error)
                return None
            self._status = replace(
                self._status,
                written_records=self._status.written_records + 1,
            )
            self._disable_after_contention()
            return None
        finally:
            self._lock.release()

    def disable(self, failed_stage: str, error: Exception) -> None:
        """Explicitly end a trace when preparation outside the sink fails."""

        if not isinstance(error, Exception):
            error = LegacyControlShadowError(
                "shadow_failed",
                "Shadow preparation failed",
            )
        if not self._lock.acquire(False):
            self._contention.set()
            return None
        try:
            self._disable_after_contention()
            if not self._status.enabled:
                return None
            failed_sequence = self._status.next_sequence
            self._status = replace(
                self._status,
                next_sequence=failed_sequence + 1,
            )
            self._disable(failed_sequence, failed_stage, error)
            return None
        finally:
            self._lock.release()


__all__ = (
    "DEFAULT_LEGACY_SHADOW_JOURNAL_CAPACITY",
    "FailOpenLegacyShadowObserver",
    "FrozenLegacyShadowFacts",
    "InMemoryLegacyShadowJournal",
    "LEGACY_CONTROL_SHADOW_SCHEMA",
    "LEGACY_CONTROL_SHADOW_VERSION",
    "LegacyControlShadowError",
    "LegacyShadowFailureSummary",
    "LegacyShadowJournalStatus",
    "LegacyShadowObserverStatus",
    "LegacyShadowRecord",
    "MAX_LEGACY_SHADOW_FACT_BYTES",
    "MAX_LEGACY_SHADOW_FACT_DEPTH",
    "MAX_LEGACY_SHADOW_FACT_KEYS",
    "MAX_LEGACY_SHADOW_FACT_NODES",
    "MAX_LEGACY_SHADOW_IDENTIFIER_LENGTH",
    "MAX_LEGACY_SHADOW_JOURNAL_CAPACITY",
    "MAX_LEGACY_SHADOW_TEXT_LENGTH",
    "SHADOW_DISABLED_STAGE",
)
