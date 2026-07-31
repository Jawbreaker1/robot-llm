"""Bounded, qualitative host controller for a physical EV3 roamer.

This module deliberately does not estimate pose, distance, heading, or a
map.  It consumes the EV3 supervisor's already-arbitrated infrared gate and
requests only one of three semantic, fixed-profile drive pulses.  The EV3
owns all numerical motor parameters and the final safety decision.

Expression generation is observational. It runs through a one-item mailbox
on a daemon worker and can never delay heartbeat, observation, or cleanup.
The callback must publish through a separate transport/process; it must
never call or share the sequential supervisor session used by this roamer.
"""

from dataclasses import dataclass
import math
import queue
import secrets
import threading
import time
from typing import Callable, Mapping, Optional, Tuple


ACTION_ADVANCE = "ADVANCE"
ACTION_TURN_LEFT = "TURN_LEFT"
ACTION_TURN_RIGHT = "TURN_RIGHT"
ALLOWED_ACTIONS = frozenset(
    (ACTION_ADVANCE, ACTION_TURN_LEFT, ACTION_TURN_RIGHT)
)
RUNTIME_PROFILE = "ir-roamer-v1"

INFRARED_STATUS_FIELD = "infrared"
PULSE_ACCOUNTING_MS = 150
MAX_EPISODE_SECONDS = 12.0
MAX_PULSES = 20
MAX_COMMANDED_MOTION_MS = 3_000
HEARTBEAT_INTERVAL_SECONDS = 0.200
MAX_HEARTBEAT_GAP_SECONDS = 0.450
MAX_INFRARED_AGE_MS = 300
POLL_INTERVAL_SECONDS = 0.050
REQUEST_TTL_MS = 500
CLEANUP_TTL_MS = 1_000
MAX_OBSERVATIONS = 1_000
EXPRESSION_EVENT_TTL_MS = 1_000
MAX_PROCESS_REQUESTS = 256
MAX_CONTROL_REQUESTS = 240

_EXPECTED_SEMANTIC_CAPABILITY = {
    "enabled": True,
    "actions": [
        ACTION_ADVANCE,
        ACTION_TURN_LEFT,
        ACTION_TURN_RIGHT,
    ],
    "mapping": {
        ACTION_ADVANCE: {
            "left_speed_dps": 100,
            "right_speed_dps": 100,
        },
        ACTION_TURN_LEFT: {
            "left_speed_dps": -100,
            "right_speed_dps": 100,
        },
        ACTION_TURN_RIGHT: {
            "left_speed_dps": 100,
            "right_speed_dps": -100,
        },
    },
    "duration_ms": PULSE_ACCOUNTING_MS,
    "max_commands_per_process": MAX_PULSES,
    "max_total_duration_ms": MAX_COMMANDED_MOTION_MS,
}

TERMINATION_PULSE_BUDGET = "PULSE_BUDGET_EXHAUSTED"
TERMINATION_MOTION_BUDGET = "MOTION_BUDGET_EXHAUSTED"
TERMINATION_TIME_BUDGET = "TIME_BUDGET_EXHAUSTED"
TERMINATION_CANCELLED = "CANCELLED"
TERMINATION_REMOTE_FAILURE = "REMOTE_FAILURE"
TERMINATION_SAFETY_FAULT = "SAFETY_FAULT"
TERMINATION_STALE_OBSERVATION = "STALE_OBSERVATION"
TERMINATION_HEARTBEAT_MISSED = "HEARTBEAT_CADENCE_MISSED"
TERMINATION_OBSERVATION_BUDGET = "OBSERVATION_BUDGET_EXHAUSTED"
TERMINATION_REQUEST_BUDGET = "REQUEST_BUDGET_EXHAUSTED"
TERMINATION_CLEANUP_FAILED = "CLEANUP_FAILED"

_BLOCKED_INFRARED_REASONS = frozenset(
    (
        "warming_up",
        "immediate_strong_return",
        "stable_filtered_near_returns",
        "stable_exit_pending",
        "blocked_hysteresis_hold",
    )
)
_CLEAR_INFRARED_REASONS = frozenset(
    (
        "stable_filtered_release_returns",
        "stable_entry_pending",
        "clear_hysteresis_hold",
    )
)


class PhysicalRoamerError(RuntimeError):
    """Invalid dependency or unsafe response from the remote supervisor."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


class _Terminate(Exception):
    def __init__(self, termination: str):
        self.termination = termination
        super().__init__(termination)


@dataclass(frozen=True)
class CleanupOutcome:
    operation: str
    attempted: bool
    succeeded: bool
    mandatory: bool

    def to_dict(self) -> Mapping[str, object]:
        return {
            "operation": self.operation,
            "attempted": self.attempted,
            "succeeded": self.succeeded,
            "mandatory": self.mandatory,
        }


@dataclass(frozen=True)
class PhysicalRoamerResult:
    termination: str
    pulses: int
    commanded_motion_ms: int
    observations: int
    heartbeats: int
    actions: Tuple[str, ...]
    cleanup: Tuple[CleanupOutcome, ...]
    expression_offers: int
    expression_dropped: int

    @property
    def stopped_cleanly(self) -> bool:
        shutdowns = [
            outcome
            for outcome in self.cleanup
            if outcome.operation == "shutdown"
        ]
        process_exits = [
            outcome
            for outcome in self.cleanup
            if outcome.operation == "wait_closed"
        ]
        return (
            len(shutdowns) == 1
            and shutdowns[0].attempted
            and shutdowns[0].succeeded
            and shutdowns[0].mandatory
            and len(process_exits) == 1
            and process_exits[0].attempted
            and process_exits[0].succeeded
            and process_exits[0].mandatory
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "termination": self.termination,
            "pulses": self.pulses,
            "commanded_motion_ms": self.commanded_motion_ms,
            "observations": self.observations,
            "heartbeats": self.heartbeats,
            "actions": list(self.actions),
            "cleanup": [
                outcome.to_dict() for outcome in self.cleanup
            ],
            "stopped_cleanly": self.stopped_cleanly,
            "expression_offers": self.expression_offers,
            "expression_dropped": self.expression_dropped,
        }


class _ExpressionMailbox:
    """Drop-oldest, single-worker isolation for optional expression work."""

    def __init__(
        self,
        submit: Optional[Callable[[Mapping[str, object]], object]],
    ):
        self._submit = submit
        self._queue = queue.Queue(maxsize=1)
        self._closed = threading.Event()
        self._lock = threading.Lock()
        self.offers = 0
        self.dropped = 0
        self._worker = None
        if submit is not None:
            self._worker = threading.Thread(
                target=self._run,
                name="physical-roamer-expression",
                daemon=True,
            )
            self._worker.start()

    def offer(self, event: Mapping[str, object]) -> None:
        if self._submit is None or self._closed.is_set():
            return
        frozen = dict(event)
        with self._lock:
            self.offers += 1
            try:
                self._queue.put_nowait(frozen)
                return
            except queue.Full:
                pass
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            else:
                self.dropped += 1
            try:
                self._queue.put_nowait(frozen)
            except queue.Full:
                self.dropped += 1

    def close_nowait(self) -> None:
        self._closed.set()

    def _run(self) -> None:
        while not self._closed.is_set():
            try:
                event = self._queue.get(timeout=0.050)
            except queue.Empty:
                continue
            try:
                self._submit(event)
            except BaseException:
                # This worker is an isolation boundary. Expression failures
                # must never reach the physical control thread.
                pass


class PhysicalRoamer:
    """Run one bounded, IR-reactive physical exploration episode."""

    def __init__(
        self,
        session,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        cancel_event=None,
        expression_submit: Optional[
            Callable[[Mapping[str, object]], object]
        ] = None,
        owner_id: str = "physical-ir-roamer",
    ):
        if not callable(getattr(session, "request", None)):
            raise PhysicalRoamerError(
                "invalid_session",
                "Physical roamer requires a sequential supervisor session",
            )
        if getattr(session, "profile", None) != RUNTIME_PROFILE:
            raise PhysicalRoamerError(
                "invalid_runtime_profile",
                "Physical roamer requires the ir-roamer-v1 session profile",
            )
        if not callable(monotonic) or not callable(sleep):
            raise PhysicalRoamerError(
                "invalid_clock",
                "Physical roamer clock dependencies must be callable",
            )
        if (
            cancel_event is not None
            and not callable(getattr(cancel_event, "is_set", None))
        ):
            raise PhysicalRoamerError(
                "invalid_cancel_event",
                "Cancel event must expose is_set()",
            )
        if expression_submit is not None and not callable(
            expression_submit
        ):
            raise PhysicalRoamerError(
                "invalid_expression_submit",
                "Expression submit callback must be callable",
            )
        if (
            not isinstance(owner_id, str)
            or not owner_id
            or owner_id != owner_id.strip()
            or len(owner_id) > 128
            or any(character in owner_id for character in "\x00\r\n")
        ):
            raise PhysicalRoamerError(
                "invalid_owner_id",
                "Physical roamer owner id is invalid",
            )
        self.session = session
        self._monotonic = monotonic
        self._sleep = sleep
        self.cancel_event = cancel_event
        self.owner_id = owner_id
        self._expression = _ExpressionMailbox(expression_submit)
        self._last_now = None
        self._last_heartbeat_at = None
        self._heartbeat_sequence = None
        self._sequence = 0
        self._heartbeats = 0
        self._control_request_count = 0

    def _now(self) -> float:
        try:
            value = self._monotonic()
        except Exception:
            raise PhysicalRoamerError(
                "clock_failed",
                "Physical roamer monotonic clock failed",
            ) from None
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or value < 0
        ):
            raise PhysicalRoamerError(
                "invalid_clock_value",
                "Physical roamer monotonic clock returned an invalid value",
            )
        current = float(value)
        if self._last_now is not None and current < self._last_now:
            raise PhysicalRoamerError(
                "clock_moved_backwards",
                "Physical roamer monotonic clock moved backwards",
            )
        self._last_now = current
        return current

    def _cancelled(self) -> bool:
        if self.cancel_event is None:
            return False
        try:
            return bool(self.cancel_event.is_set())
        except Exception:
            raise PhysicalRoamerError(
                "cancel_event_failed",
                "Physical roamer cancel event failed",
            ) from None

    def _check_cancelled(self) -> None:
        if self._cancelled():
            raise _Terminate(TERMINATION_CANCELLED)

    def _request(
        self,
        operation: str,
        arguments=None,
        ttl_ms: int = REQUEST_TTL_MS,
    ) -> Mapping[str, object]:
        self._check_cancelled()
        if self._control_request_count >= MAX_CONTROL_REQUESTS:
            raise _Terminate(TERMINATION_REQUEST_BUDGET)
        self._control_request_count += 1
        try:
            result = self.session.request(
                operation,
                arguments,
                ttl_ms,
            )
        except _Terminate:
            raise
        except Exception:
            raise _Terminate(TERMINATION_REMOTE_FAILURE) from None
        if not isinstance(result, Mapping):
            raise _Terminate(TERMINATION_REMOTE_FAILURE)
        self._check_cancelled()
        return result

    def _next_sequence(self) -> int:
        self._sequence += 1
        return self._sequence

    @staticmethod
    def _profile_int(description, field: str, expected: int) -> bool:
        value = description.get(field)
        return type(value) is int and value == expected

    @classmethod
    def _validate_description(
        cls,
        description: Mapping[str, object],
    ) -> None:
        capabilities = description.get("capabilities")
        if not isinstance(capabilities, Mapping):
            raise _Terminate(TERMINATION_REMOTE_FAILURE)
        differential = capabilities.get("differential_drive_timed")
        semantic = capabilities.get("semantic_drive_pulse")
        if (
            description.get("runtime_profile") != RUNTIME_PROFILE
            or description.get("motion_enabled") is not True
            or not cls._profile_int(
                description,
                "remaining_motion_budget",
                MAX_PULSES,
            )
            or not cls._profile_int(
                description,
                "remaining_motion_duration_ms",
                MAX_COMMANDED_MOTION_MS,
            )
            or not cls._profile_int(
                description,
                "max_session_ms",
                20_000,
            )
            or not cls._profile_int(
                description,
                "poll_interval_ms",
                150,
            )
            or not cls._profile_int(
                description,
                "max_poll_lateness_ms",
                400,
            )
            or not cls._profile_int(
                description,
                "max_process_requests",
                MAX_PROCESS_REQUESTS,
            )
            or not isinstance(differential, Mapping)
            or differential.get("enabled") is not False
            or type(semantic) is not dict
            or semantic != _EXPECTED_SEMANTIC_CAPABILITY
        ):
            raise _Terminate(TERMINATION_REMOTE_FAILURE)

    def _heartbeat(self, session_id: str) -> None:
        before = self._now()
        if (
            self._last_heartbeat_at is not None
            and before - self._last_heartbeat_at
            > MAX_HEARTBEAT_GAP_SECONDS
        ):
            raise _Terminate(TERMINATION_HEARTBEAT_MISSED)
        sequence = self._next_sequence()
        response = self._request(
            "heartbeat",
            {
                "session_id": session_id,
                "sequence_id": sequence,
            },
        )
        if (
            response.get("status") != "accepted"
            or response.get("sequence_id") != sequence
            or type(response.get("heartbeat_timeout_ms")) is not int
            or response.get("heartbeat_timeout_ms") != 500
        ):
            raise _Terminate(TERMINATION_REMOTE_FAILURE)
        after = self._now()
        if (
            self._last_heartbeat_at is not None
            and after - self._last_heartbeat_at
            > MAX_HEARTBEAT_GAP_SECONDS
        ):
            raise _Terminate(TERMINATION_HEARTBEAT_MISSED)
        self._last_heartbeat_at = after
        self._heartbeat_sequence = sequence
        self._heartbeats += 1

    def _verify_heartbeat_gap(self) -> None:
        if (
            self._last_heartbeat_at is not None
            and self._now() - self._last_heartbeat_at
            > MAX_HEARTBEAT_GAP_SECONDS
        ):
            raise _Terminate(TERMINATION_HEARTBEAT_MISSED)

    @staticmethod
    def _infrared_observation(
        status: Mapping[str, object],
    ) -> Tuple[str, str]:
        infrared = status.get(INFRARED_STATUS_FIELD)
        if not isinstance(infrared, Mapping):
            raise _Terminate(TERMINATION_STALE_OBSERVATION)
        blocked = infrared.get("blocked")
        reason = infrared.get("reason")
        raw = infrared.get("raw")
        filtered = infrared.get("filtered")
        sample_count = infrared.get("sample_count")
        observed_monotonic_ms = infrared.get(
            "observed_monotonic_ms"
        )
        age_ms = infrared.get("age_ms")
        if (
            infrared.get("fresh") is not True
            or infrared.get("stale") is True
            or type(observed_monotonic_ms) is not int
            or observed_monotonic_ms < 0
            or type(age_ms) is not int
            or not 0 <= age_ms <= MAX_INFRARED_AGE_MS
            or not isinstance(blocked, bool)
            or not isinstance(reason, str)
            or (
                blocked
                and reason not in _BLOCKED_INFRARED_REASONS
            )
            or (
                not blocked
                and reason not in _CLEAR_INFRARED_REASONS
            )
            or isinstance(raw, bool)
            or not isinstance(raw, int)
            or not 0 <= raw <= 100
            or isinstance(filtered, bool)
            or not isinstance(filtered, int)
            or not 0 <= filtered <= 100
            or isinstance(sample_count, bool)
            or not isinstance(sample_count, int)
            or sample_count <= 0
        ):
            raise _Terminate(TERMINATION_STALE_OBSERVATION)
        return ("BLOCKED" if blocked else "CLEAR", reason)

    @classmethod
    def _validate_status(
        cls,
        status: Mapping[str, object],
        allow_running: bool = True,
    ) -> Tuple[str, str, str]:
        state = status.get("state")
        allowed_states = (
            frozenset(("ARMED_IDLE", "RUNNING"))
            if allow_running
            else frozenset(("DISARMED",))
        )
        expected_motion_allowed = state == "ARMED_IDLE"
        active_command_id = status.get("active_command_id")
        if (
            status.get("fault") is not None
            or status.get("status") != "ok"
            or state not in allowed_states
            or status.get("session_active") is not True
            or type(status.get("touch")) is not int
            or status.get("touch") != 0
            or status.get("motion_allowed") is not expected_motion_allowed
            or (
                state == "RUNNING"
                and (
                    not isinstance(active_command_id, str)
                    or not active_command_id
                )
            )
            or (
                state != "RUNNING"
                and active_command_id is not None
            )
        ):
            raise _Terminate(TERMINATION_SAFETY_FAULT)
        heartbeat_age_ms = status.get("heartbeat_age_ms")
        if (
            type(heartbeat_age_ms) is not int
            or heartbeat_age_ms < 0
            or heartbeat_age_ms
            > int(MAX_HEARTBEAT_GAP_SECONDS * 1_000)
        ):
            raise _Terminate(TERMINATION_HEARTBEAT_MISSED)
        obstacle, reason = cls._infrared_observation(status)
        return state, obstacle, reason

    def _sleep_poll(self) -> None:
        self._sleep(POLL_INTERVAL_SECONDS)
        self._verify_heartbeat_gap()

    def _observe_armed(
        self,
        session_id: str,
    ) -> Tuple[str, str, str]:
        self._heartbeat(session_id)
        status = self._request("status")
        self._verify_heartbeat_gap()
        return self._validate_status(status)

    def _cleanup_request(
        self,
        operation: str,
        arguments=None,
        mandatory: bool = True,
    ) -> CleanupOutcome:
        try:
            result = self.session.request(
                operation,
                arguments,
                CLEANUP_TTL_MS,
            )
            succeeded = self._cleanup_result_is_safe(
                operation,
                result,
            )
        except BaseException:
            succeeded = False
        return CleanupOutcome(
            operation,
            True,
            succeeded,
            mandatory,
        )

    @staticmethod
    def _cleanup_result_is_safe(operation: str, result) -> bool:
        if not isinstance(result, Mapping):
            return False
        if operation != "shutdown":
            return False
        state = result.get("state")
        if result.get("motion_allowed") is not False:
            return False
        if (
            result.get("session_active") is not False
            or result.get("active_command_id") is not None
        ):
            return False
        if state == "DISARMED":
            return (
                result.get("status") == "ok"
                and result.get("fault") is None
            )
        if state == "CLOSED":
            return result.get("status") == "closed"
        fault = result.get("fault")
        return (
            state == "FAULT_LATCHED"
            and result.get("status") == "fault"
            and isinstance(fault, Mapping)
            and fault.get("stop_confirmed") is True
            and not fault.get("stop_errors")
            and not fault.get("fault_tokens")
        )

    def _cleanup(self) -> Tuple[CleanupOutcome, ...]:
        # Shutdown is itself an urgent operation whose ACK is published only
        # after the EV3 has proved a local stop.  Sending release -> stop ->
        # shutdown would repeat the same expensive proof and can starve the
        # supervisor's safety poll on the 300 MHz brick.
        outcomes = [self._cleanup_request("shutdown")]
        wait_closed = getattr(self.session, "wait_closed", None)
        if callable(wait_closed):
            try:
                return_code = wait_closed(timeout_seconds=3.0)
                succeeded = (
                    type(return_code) is int
                    and return_code == 0
                )
            except BaseException:
                succeeded = False
            outcomes.append(
                CleanupOutcome(
                    "wait_closed",
                    True,
                    succeeded,
                    True,
                )
            )
        else:
            outcomes.append(
                CleanupOutcome(
                    "wait_closed",
                    False,
                    False,
                    True,
                )
            )
        close = getattr(self.session, "close", None)
        if callable(close):
            try:
                close()
                succeeded = True
            except BaseException:
                succeeded = False
            outcomes.append(
                CleanupOutcome(
                    "close",
                    True,
                    succeeded,
                    False,
                )
            )
        return tuple(outcomes)

    def _expression_event(
        self,
        episode_id: str,
        observed_monotonic_ms: int,
        obstacle: str,
        reason: str,
        action: str,
    ) -> None:
        self._expression.offer(
            {
                "schema": "physical-roamer-expression/v1",
                "episode_id": episode_id,
                "observed_monotonic_ms": observed_monotonic_ms,
                "valid_until_monotonic_ms": (
                    observed_monotonic_ms
                    + EXPRESSION_EVENT_TTL_MS
                ),
                "obstacle": obstacle,
                "reason": reason,
                "action": action,
            }
        )

    def run(self) -> PhysicalRoamerResult:
        """Run one episode and always attempt a verified remote shutdown."""

        session_id = None
        episode_started_at = None
        episode_id = None
        pulses = 0
        commanded_motion_ms = 0
        observations = 0
        actions = []
        termination = TERMINATION_REMOTE_FAILURE
        next_blocked_turn = ACTION_TURN_LEFT
        blocked_episode_turn = None

        try:
            description = self._request("describe")
            self._validate_description(description)
            claimed = self._request(
                "claim",
                {"owner_id": self.owner_id},
            )
            candidate_session_id = claimed.get("session_id")
            if (
                claimed.get("status") != "claimed"
                or claimed.get("state") != "DISARMED"
                or not isinstance(candidate_session_id, str)
                or not candidate_session_id
                or candidate_session_id
                != candidate_session_id.strip()
                or len(candidate_session_id) > 128
                or any(
                    character in candidate_session_id
                    for character in "\x00\r\n"
                )
            ):
                raise _Terminate(TERMINATION_REMOTE_FAILURE)
            session_id = candidate_session_id

            self._heartbeat(session_id)
            initial = self._request("status")
            self._verify_heartbeat_gap()
            self._validate_status(initial, allow_running=False)
            self._heartbeat(session_id)
            arm_sequence = self._next_sequence()
            armed = self._request(
                "arm",
                {
                    "session_id": session_id,
                    "sequence_id": arm_sequence,
                },
            )
            armed_state, _armed_obstacle, _armed_reason = (
                self._validate_status(armed)
            )
            if armed_state != "ARMED_IDLE":
                raise _Terminate(TERMINATION_SAFETY_FAULT)
            episode_started_at = self._now()
            episode_id = "physical-roamer-{}".format(
                secrets.token_hex(8)
            )

            while True:
                self._check_cancelled()
                elapsed = self._now() - episode_started_at
                if elapsed + PULSE_ACCOUNTING_MS / 1_000.0 > (
                    MAX_EPISODE_SECONDS
                ):
                    termination = TERMINATION_TIME_BUDGET
                    break
                if pulses >= MAX_PULSES:
                    termination = TERMINATION_PULSE_BUDGET
                    break
                if (
                    commanded_motion_ms + PULSE_ACCOUNTING_MS
                    > MAX_COMMANDED_MOTION_MS
                ):
                    termination = TERMINATION_MOTION_BUDGET
                    break
                if observations >= MAX_OBSERVATIONS:
                    termination = TERMINATION_OBSERVATION_BUDGET
                    break

                state, obstacle, reason = self._observe_armed(
                    session_id
                )
                observed_monotonic_ms = int(self._now() * 1_000)
                observations += 1
                if obstacle == "CLEAR":
                    blocked_episode_turn = None
                elif blocked_episode_turn is None:
                    blocked_episode_turn = next_blocked_turn
                    next_blocked_turn = (
                        ACTION_TURN_RIGHT
                        if next_blocked_turn == ACTION_TURN_LEFT
                        else ACTION_TURN_LEFT
                    )
                if state == "RUNNING":
                    self._sleep_poll()
                    continue

                if obstacle == "CLEAR":
                    action = ACTION_ADVANCE
                else:
                    action = blocked_episode_turn
                if action not in ALLOWED_ACTIONS:
                    raise _Terminate(TERMINATION_SAFETY_FAULT)
                self._check_cancelled()
                self._heartbeat(session_id)
                if (
                    self._now()
                    - episode_started_at
                    + PULSE_ACCOUNTING_MS / 1_000.0
                    > MAX_EPISODE_SECONDS
                ):
                    termination = TERMINATION_TIME_BUDGET
                    break
                if self._heartbeat_sequence is None:
                    raise _Terminate(TERMINATION_HEARTBEAT_MISSED)
                sequence = self._next_sequence()
                command_id = "physical-roamer-{:03d}".format(
                    pulses + 1
                )
                pulses += 1
                commanded_motion_ms += PULSE_ACCOUNTING_MS
                actions.append(action)
                drive_result = self._request(
                    "drive_pulse",
                    {
                        "session_id": session_id,
                        "sequence_id": sequence,
                        "command_id": command_id,
                        "reference_heartbeat_sequence": (
                            self._heartbeat_sequence
                        ),
                        "action": action,
                    },
                )
                drive_state, _drive_obstacle, _drive_reason = (
                    self._validate_status(drive_result)
                )
                if (
                    drive_state != "RUNNING"
                    or drive_result.get("active_command_id")
                    != command_id
                ):
                    raise _Terminate(TERMINATION_REMOTE_FAILURE)
                self._verify_heartbeat_gap()
                self._heartbeat(session_id)
                self._expression_event(
                    episode_id,
                    observed_monotonic_ms,
                    obstacle,
                    reason,
                    action,
                )
                self._sleep_poll()
        except _Terminate as stop:
            termination = stop.termination
        except PhysicalRoamerError:
            termination = TERMINATION_REMOTE_FAILURE
        except Exception:
            termination = TERMINATION_REMOTE_FAILURE
        finally:
            cleanup = self._cleanup()
            self._expression.close_nowait()

        if any(
            (outcome.mandatory or outcome.attempted)
            and not outcome.succeeded
            for outcome in cleanup
        ):
            termination = TERMINATION_CLEANUP_FAILED

        return PhysicalRoamerResult(
            termination=termination,
            pulses=pulses,
            commanded_motion_ms=commanded_motion_ms,
            observations=observations,
            heartbeats=self._heartbeats,
            actions=tuple(actions),
            cleanup=cleanup,
            expression_offers=self._expression.offers,
            expression_dropped=self._expression.dropped,
        )


__all__ = (
    "ACTION_ADVANCE",
    "ACTION_TURN_LEFT",
    "ACTION_TURN_RIGHT",
    "ALLOWED_ACTIONS",
    "CleanupOutcome",
    "EXPRESSION_EVENT_TTL_MS",
    "HEARTBEAT_INTERVAL_SECONDS",
    "INFRARED_STATUS_FIELD",
    "MAX_COMMANDED_MOTION_MS",
    "MAX_CONTROL_REQUESTS",
    "MAX_EPISODE_SECONDS",
    "MAX_HEARTBEAT_GAP_SECONDS",
    "MAX_INFRARED_AGE_MS",
    "MAX_PULSES",
    "MAX_PROCESS_REQUESTS",
    "PULSE_ACCOUNTING_MS",
    "REQUEST_TTL_MS",
    "RUNTIME_PROFILE",
    "PhysicalRoamer",
    "PhysicalRoamerError",
    "PhysicalRoamerResult",
    "TERMINATION_CANCELLED",
    "TERMINATION_CLEANUP_FAILED",
    "TERMINATION_HEARTBEAT_MISSED",
    "TERMINATION_MOTION_BUDGET",
    "TERMINATION_OBSERVATION_BUDGET",
    "TERMINATION_PULSE_BUDGET",
    "TERMINATION_REMOTE_FAILURE",
    "TERMINATION_REQUEST_BUDGET",
    "TERMINATION_SAFETY_FAULT",
    "TERMINATION_STALE_OBSERVATION",
    "TERMINATION_TIME_BUDGET",
)
