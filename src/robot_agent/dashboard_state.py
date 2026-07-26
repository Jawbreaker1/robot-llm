"""Thread-safe, in-memory state for the motion-free Mac dashboard."""

from collections import deque
from dataclasses import replace
import secrets
import threading
import time
from typing import Callable, Mapping, Optional, Tuple

from .dashboard_contract import (
    CHAT_MODES,
    ChatMessage,
    ChatTurn,
    Conversation,
    DashboardContractError,
    DashboardSettings,
    EventPage,
    NodeDescriptor,
    RegistrySnapshot,
    RobotDescriptor,
    TechnicalEvent,
)


_MAX_INT = 2**63 - 1
Clock = Callable[[], int]
IDFactory = Callable[[], str]


class DashboardStateError(RuntimeError):
    """A typed conflict or lookup failure in dashboard state."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _default_unix_ms() -> int:
    return time.time_ns() // 1_000_000


def _default_monotonic_ms() -> int:
    return time.monotonic_ns() // 1_000_000


def _default_id() -> str:
    return secrets.token_hex(8)


def _clock_value(name: str, clock: Clock) -> int:
    value = clock()
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= _MAX_INT
    ):
        raise DashboardStateError(
            "invalid_clock",
            "{} returned an invalid value".format(name),
        )
    return value


def _new_id(prefix: str, factory: IDFactory) -> str:
    value = factory()
    try:
        # Let the contract validate the completed identifier.
        event = TechnicalEvent(
            server_instance_id="validation",
            sequence=1,
            event_id="{}-{}".format(prefix, value),
            recorded_at_unix_ms=0,
            recorded_at_monotonic_ms=0,
            level="debug",
            category="validation",
            event_type="validation.id",
            source_id="validation",
            message="Identifier validation",
            data={},
        )
    except (DashboardContractError, TypeError):
        raise DashboardStateError(
            "invalid_id_factory",
            "ID factory returned an invalid identifier",
        ) from None
    return event.event_id


class SettingsStore:
    """Atomic session-only settings with optimistic revision checks."""

    def __init__(
        self,
        initial: Optional[DashboardSettings] = None,
    ):
        self._settings = initial or DashboardSettings.defaults()
        if not isinstance(self._settings, DashboardSettings):
            raise DashboardStateError(
                "invalid_settings",
                "Initial dashboard settings are invalid",
            )
        self._lock = threading.RLock()

    def snapshot(self) -> DashboardSettings:
        with self._lock:
            return self._settings

    def update(
        self,
        expected_revision: int,
        changes: Mapping[str, object],
    ) -> DashboardSettings:
        with self._lock:
            if (
                isinstance(expected_revision, bool)
                or not isinstance(expected_revision, int)
                or expected_revision != self._settings.revision
            ):
                raise DashboardStateError(
                    "settings_revision_conflict",
                    "Settings revision does not match",
                )
            updated = self._settings.with_updates(changes)
            self._settings = updated
            return updated

    def replace(
        self,
        expected_revision: int,
        settings: DashboardSettings,
    ) -> DashboardSettings:
        if not isinstance(settings, DashboardSettings):
            raise DashboardStateError(
                "invalid_settings",
                "Replacement settings are invalid",
            )
        with self._lock:
            if expected_revision != self._settings.revision:
                raise DashboardStateError(
                    "settings_revision_conflict",
                    "Settings revision does not match",
                )
            if settings.revision != self._settings.revision + 1:
                raise DashboardStateError(
                    "invalid_settings_revision",
                    "Replacement settings revision is invalid",
                )
            self._settings = settings
            return settings


class EventLog:
    """Bounded event ring with monotonically assigned host sequence IDs."""

    def __init__(
        self,
        server_instance_id: str,
        capacity: int = 2_000,
        *,
        unix_clock_ms: Clock = _default_unix_ms,
        monotonic_clock_ms: Clock = _default_monotonic_ms,
        id_factory: IDFactory = _default_id,
    ):
        if (
            not isinstance(capacity, int)
            or isinstance(capacity, bool)
            or not 1 <= capacity <= 100_000
            or not callable(unix_clock_ms)
            or not callable(monotonic_clock_ms)
            or not callable(id_factory)
        ):
            raise DashboardStateError(
                "invalid_event_log_configuration",
                "Event log configuration is invalid",
            )
        # Contract validation without retaining a synthetic event.
        TechnicalEvent(
            server_instance_id=server_instance_id,
            sequence=1,
            event_id="validation",
            recorded_at_unix_ms=0,
            recorded_at_monotonic_ms=0,
            level="debug",
            category="validation",
            event_type="validation.event",
            source_id="validation",
            message="Event log validation",
            data={},
        )
        self._server_instance_id = server_instance_id
        self._capacity = capacity
        self._unix_clock_ms = unix_clock_ms
        self._monotonic_clock_ms = monotonic_clock_ms
        self._id_factory = id_factory
        self._events = deque()
        self._next_sequence = 1
        self._dropped_total = 0
        self._lock = threading.RLock()

    @property
    def capacity(self) -> int:
        return self._capacity

    def append(
        self,
        *,
        level: str,
        category: str,
        event_type: str,
        source_id: str,
        message: str,
        data: Optional[Mapping[str, object]] = None,
        robot_id: Optional[str] = None,
        node_id: Optional[str] = None,
        request_id: Optional[str] = None,
        conversation_id: Optional[str] = None,
        turn_id: Optional[str] = None,
        tool_call_id: Optional[str] = None,
    ) -> TechnicalEvent:
        with self._lock:
            sequence = self._next_sequence
            event = TechnicalEvent(
                server_instance_id=self._server_instance_id,
                sequence=sequence,
                event_id=_new_id("event", self._id_factory),
                recorded_at_unix_ms=_clock_value(
                    "unix_clock_ms",
                    self._unix_clock_ms,
                ),
                recorded_at_monotonic_ms=_clock_value(
                    "monotonic_clock_ms",
                    self._monotonic_clock_ms,
                ),
                level=level,
                category=category,
                event_type=event_type,
                source_id=source_id,
                message=message,
                data={} if data is None else data,
                robot_id=robot_id,
                node_id=node_id,
                request_id=request_id,
                conversation_id=conversation_id,
                turn_id=turn_id,
                tool_call_id=tool_call_id,
            )
            if len(self._events) == self._capacity:
                self._events.popleft()
                self._dropped_total += 1
            self._events.append(event)
            self._next_sequence += 1
            return event

    def page(
        self,
        after_sequence: int = 0,
        limit: int = 200,
    ) -> EventPage:
        if (
            isinstance(after_sequence, bool)
            or not isinstance(after_sequence, int)
            or after_sequence < 0
            or isinstance(limit, bool)
            or not isinstance(limit, int)
            or not 1 <= limit <= 1_000
        ):
            raise DashboardStateError(
                "invalid_event_cursor",
                "Event cursor or limit is invalid",
            )
        with self._lock:
            oldest = (
                self._events[0].sequence
                if self._events
                else None
            )
            newest = (
                self._events[-1].sequence
                if self._events
                else 0
            )
            gap = oldest is not None and after_sequence < oldest - 1
            selected = tuple(
                event
                for event in self._events
                if event.sequence > after_sequence
            )[:limit]
            next_after = (
                selected[-1].sequence
                if selected
                else after_sequence
            )
            return EventPage(
                server_instance_id=self._server_instance_id,
                after_sequence=after_sequence,
                oldest_sequence=oldest,
                newest_sequence=newest,
                next_after_sequence=next_after,
                gap=gap,
                dropped_total=self._dropped_total,
                events=selected,
            )


class NodeRegistry:
    """Single-writer descriptive registry with no executable capability."""

    def __init__(
        self,
        server_instance_id: str,
        robots: Tuple[RobotDescriptor, ...] = (),
        nodes: Tuple[NodeDescriptor, ...] = (),
        *,
        unix_clock_ms: Clock = _default_unix_ms,
        monotonic_clock_ms: Clock = _default_monotonic_ms,
    ):
        if not callable(unix_clock_ms) or not callable(monotonic_clock_ms):
            raise DashboardStateError(
                "invalid_registry_configuration",
                "Registry clocks are invalid",
            )
        self._server_instance_id = server_instance_id
        self._unix_clock_ms = unix_clock_ms
        self._monotonic_clock_ms = monotonic_clock_ms
        self._lock = threading.RLock()
        self._version = 1
        self._robots = tuple(robots)
        self._nodes = tuple(nodes)
        # Validate all identities and associations eagerly.
        self._make_snapshot()

    def _make_snapshot(self) -> RegistrySnapshot:
        return RegistrySnapshot(
            server_instance_id=self._server_instance_id,
            version=self._version,
            generated_at_unix_ms=_clock_value(
                "unix_clock_ms",
                self._unix_clock_ms,
            ),
            generated_at_monotonic_ms=_clock_value(
                "monotonic_clock_ms",
                self._monotonic_clock_ms,
            ),
            robots=self._robots,
            nodes=self._nodes,
        )

    def snapshot(self) -> RegistrySnapshot:
        with self._lock:
            return self._make_snapshot()

    def replace(
        self,
        expected_version: int,
        robots: Tuple[RobotDescriptor, ...],
        nodes: Tuple[NodeDescriptor, ...],
    ) -> RegistrySnapshot:
        if not isinstance(robots, tuple) or not isinstance(nodes, tuple):
            raise DashboardStateError(
                "invalid_registry",
                "Registry replacements must be tuples",
            )
        with self._lock:
            if expected_version != self._version:
                raise DashboardStateError(
                    "registry_version_conflict",
                    "Registry version does not match",
                )
            # Validate before mutating.
            candidate = RegistrySnapshot(
                server_instance_id=self._server_instance_id,
                version=self._version + 1,
                generated_at_unix_ms=_clock_value(
                    "unix_clock_ms",
                    self._unix_clock_ms,
                ),
                generated_at_monotonic_ms=_clock_value(
                    "monotonic_clock_ms",
                    self._monotonic_clock_ms,
                ),
                robots=robots,
                nodes=nodes,
            )
            self._version += 1
            self._robots = robots
            self._nodes = nodes
            return candidate


class ConversationStore:
    """Versioned chat transcripts with idempotent, serialized turns."""

    def __init__(
        self,
        *,
        unix_clock_ms: Clock = _default_unix_ms,
        id_factory: IDFactory = _default_id,
    ):
        if not callable(unix_clock_ms) or not callable(id_factory):
            raise DashboardStateError(
                "invalid_conversation_store_configuration",
                "Conversation store dependencies are invalid",
            )
        self._unix_clock_ms = unix_clock_ms
        self._id_factory = id_factory
        self._conversations = {}
        self._turns = {}
        self._request_index = {}
        self._lock = threading.RLock()

    def _now(self) -> int:
        return _clock_value("unix_clock_ms", self._unix_clock_ms)

    def create(self, title: Optional[str] = None) -> Conversation:
        with self._lock:
            conversation_id = _new_id(
                "conversation",
                self._id_factory,
            )
            if conversation_id in self._conversations:
                raise DashboardStateError(
                    "duplicate_generated_id",
                    "Conversation ID was generated twice",
                )
            now = self._now()
            conversation = Conversation(
                conversation_id=conversation_id,
                version=1,
                title=title,
                created_at_unix_ms=now,
                updated_at_unix_ms=now,
                messages=(),
            )
            self._conversations[conversation_id] = conversation
            return conversation

    def get(self, conversation_id: str) -> Conversation:
        with self._lock:
            try:
                return self._conversations[conversation_id]
            except (KeyError, TypeError):
                raise DashboardStateError(
                    "conversation_not_found",
                    "Conversation does not exist",
                ) from None

    def get_turn(self, turn_id: str) -> ChatTurn:
        with self._lock:
            try:
                return self._turns[turn_id]
            except (KeyError, TypeError):
                raise DashboardStateError(
                    "turn_not_found",
                    "Chat turn does not exist",
                ) from None

    def history(
        self,
        conversation_id: str,
    ) -> Tuple[ChatMessage, ...]:
        return self.get(conversation_id).messages

    def submit_turn(
        self,
        conversation_id: str,
        client_request_id: str,
        expected_version: int,
        content: str,
        mode: str,
        settings_revision: int = 1,
    ) -> Tuple[Conversation, ChatTurn, bool]:
        if mode not in CHAT_MODES:
            raise DashboardStateError(
                "invalid_chat_mode",
                "Chat turn mode is unsupported",
            )
        with self._lock:
            try:
                conversation = self._conversations[conversation_id]
            except (KeyError, TypeError):
                raise DashboardStateError(
                    "conversation_not_found",
                    "Conversation does not exist",
                ) from None
            request_key = (conversation_id, client_request_id)
            existing_id = self._request_index.get(request_key)
            if existing_id is not None:
                existing = self._turns[existing_id]
                if (
                    existing.content != content
                    or existing.mode != mode
                    or existing.settings_revision != settings_revision
                ):
                    raise DashboardStateError(
                        "idempotency_conflict",
                        "Client request ID was reused with other content",
                    )
                return conversation, existing, False
            if expected_version != conversation.version:
                raise DashboardStateError(
                    "conversation_version_conflict",
                    "Conversation version does not match",
                )
            if conversation.active_turn_id is not None:
                raise DashboardStateError(
                    "conversation_turn_active",
                    "Conversation already has an active turn",
                )
            turn_id = _new_id("turn", self._id_factory)
            message_id = _new_id("message", self._id_factory)
            if (
                turn_id in self._turns
                or any(
                    message.message_id == message_id
                    for message in conversation.messages
                )
            ):
                raise DashboardStateError(
                    "duplicate_generated_id",
                    "Chat ID was generated twice",
                )
            now = self._now()
            turn = ChatTurn(
                turn_id=turn_id,
                conversation_id=conversation_id,
                client_request_id=client_request_id,
                mode=mode,
                settings_revision=settings_revision,
                status="queued",
                content=content,
                created_at_unix_ms=now,
            )
            message = ChatMessage(
                message_id=message_id,
                turn_id=turn_id,
                role="user",
                content=content,
                created_at_unix_ms=now,
            )
            updated = replace(
                conversation,
                version=conversation.version + 1,
                updated_at_unix_ms=now,
                active_turn_id=turn_id,
                messages=conversation.messages + (message,),
            )
            self._conversations[conversation_id] = updated
            self._turns[turn_id] = turn
            self._request_index[request_key] = turn_id
            return updated, turn, True

    def mark_running(
        self,
        turn_id: str,
    ) -> Tuple[Conversation, ChatTurn]:
        with self._lock:
            turn = self.get_turn(turn_id)
            if turn.status == "running":
                return self.get(turn.conversation_id), turn
            if turn.status != "queued":
                raise DashboardStateError(
                    "invalid_turn_transition",
                    "Only a queued turn can start",
                )
            now = self._now()
            running = replace(
                turn,
                status="running",
                started_at_unix_ms=now,
            )
            self._turns[turn_id] = running
            return self.get(turn.conversation_id), running

    def fail_queued(
        self,
        turn_id: str,
        error_code: str,
    ) -> Tuple[Conversation, ChatTurn]:
        """Atomically fail work that was cancelled before execution."""

        with self._lock:
            turn = self.get_turn(turn_id)
            conversation = self.get(turn.conversation_id)
            if turn.status != "queued":
                raise DashboardStateError(
                    "invalid_turn_transition",
                    "Only a queued turn can be cancelled",
                )
            if conversation.active_turn_id != turn_id:
                raise DashboardStateError(
                    "active_turn_mismatch",
                    "Conversation active turn does not match",
                )
            now = self._now()
            failed = replace(
                turn,
                status="failed",
                started_at_unix_ms=now,
                completed_at_unix_ms=now,
                error_code=error_code,
            )
            updated = replace(
                conversation,
                version=conversation.version + 1,
                updated_at_unix_ms=now,
                active_turn_id=None,
            )
            self._turns[turn_id] = failed
            self._conversations[conversation.conversation_id] = updated
            return updated, failed

    def _complete(
        self,
        turn_id: str,
        *,
        status: str,
        answer_text: Optional[str] = None,
        clarification_question: Optional[str] = None,
        citation_ids: Tuple[str, ...] = (),
        error_code: Optional[str] = None,
    ) -> Tuple[Conversation, ChatTurn]:
        with self._lock:
            turn = self.get_turn(turn_id)
            conversation = self.get(turn.conversation_id)
            if turn.status != "running":
                raise DashboardStateError(
                    "invalid_turn_transition",
                    "Only a running turn can complete",
                )
            if conversation.active_turn_id != turn_id:
                raise DashboardStateError(
                    "active_turn_mismatch",
                    "Conversation active turn does not match",
                )
            now = self._now()
            completed = replace(
                turn,
                status=status,
                completed_at_unix_ms=now,
                answer_text=answer_text,
                clarification_question=clarification_question,
                citation_ids=tuple(citation_ids),
                error_code=error_code,
            )
            response_content = (
                answer_text
                if status == "answered"
                else clarification_question
                if status == "clarification_required"
                else None
            )
            messages = conversation.messages
            if response_content is not None:
                messages = messages + (
                    ChatMessage(
                        message_id=_new_id(
                            "message",
                            self._id_factory,
                        ),
                        turn_id=turn_id,
                        role="assistant",
                        content=response_content,
                        created_at_unix_ms=now,
                        citation_ids=(
                            tuple(citation_ids)
                            if status == "answered"
                            else ()
                        ),
                    ),
                )
            updated = replace(
                conversation,
                version=conversation.version + 1,
                updated_at_unix_ms=now,
                active_turn_id=None,
                messages=messages,
            )
            self._turns[turn_id] = completed
            self._conversations[conversation.conversation_id] = updated
            return updated, completed

    def complete_answer(
        self,
        turn_id: str,
        answer_text: str,
        citation_ids: Tuple[str, ...] = (),
    ) -> Tuple[Conversation, ChatTurn]:
        return self._complete(
            turn_id,
            status="answered",
            answer_text=answer_text,
            citation_ids=tuple(citation_ids),
        )

    def complete_clarification(
        self,
        turn_id: str,
        question: str,
    ) -> Tuple[Conversation, ChatTurn]:
        return self._complete(
            turn_id,
            status="clarification_required",
            clarification_question=question,
        )

    def complete_failed(
        self,
        turn_id: str,
        error_code: str,
    ) -> Tuple[Conversation, ChatTurn]:
        return self._complete(
            turn_id,
            status="failed",
            error_code=error_code,
        )
