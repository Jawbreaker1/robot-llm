"""Composite user-to-robot turns with one explicit physical dispatch branch."""

from collections import OrderedDict
from copy import deepcopy
import threading
import time
from typing import Callable, Mapping, Optional

from .lm_studio_robot_input import (
    CLARIFY,
    PHYSICAL_TASK,
    STOP_TASK,
    RobotInput,
    RobotInputDecision,
)
from .robot_control_service import RobotControlServiceError
from .robot_status_facts import project_robot_status_facts


ROBOT_INPUT_TURN_SCHEMA = "robot-input-turn/v1"
MAX_INPUT_HISTORY = 128


def _clarification(locale: str) -> str:
    if locale == "sv":
        return "Jag är inte säker på vad du menar. Kan du förtydliga?"
    return "I am not sure what you mean. Could you clarify?"


class RobotInputService:
    """Interpret once; only ``PHYSICAL_TASK`` may start a robot episode."""

    def __init__(
        self,
        *,
        control_service,
        model_factory: Callable[[str], object],
        spatial_map_provider=None,
        speech_sink: Optional[Callable[[str, str, str], bool]] = None,
        clock_ms: Callable[[], int] = lambda: time.time_ns() // 1_000_000,
    ):
        if (
            control_service is None
            or not callable(model_factory)
            or not callable(clock_ms)
            or speech_sink is not None
            and not callable(speech_sink)
        ):
            raise ValueError("robot input service configuration is invalid")
        snapshot = getattr(spatial_map_provider, "snapshot", None)
        if spatial_map_provider is not None and not callable(snapshot):
            raise ValueError("robot input spatial map provider is invalid")
        self._control = control_service
        self._model_factory = model_factory
        self._map_snapshot = snapshot
        self._speech_sink = speech_sink
        self._clock_ms = clock_ms
        self._history = OrderedDict()
        self._inflight = set()
        self._lock = threading.Lock()

    def _facts(self):
        captured = self._clock_ms()
        spatial = None
        if self._map_snapshot is not None:
            try:
                spatial = self._map_snapshot()
            except Exception:
                spatial = None
        return project_robot_status_facts(
            self._control.status(),
            spatial,
            captured_at_unix_ms=captured,
        )

    @staticmethod
    def _fallback(input_value: RobotInput) -> RobotInputDecision:
        return RobotInputDecision(
            intent=CLARIFY,
            confidence_milli=0,
            reply_text=_clarification(input_value.locale),
            fallback=True,
        )

    def _interpret(self, model: str, input_value: RobotInput, facts):
        try:
            interpreter = self._model_factory(model)
            decision = interpreter.interpret(input_value, facts)
            if not isinstance(decision, RobotInputDecision):
                raise TypeError
            return decision
        except Exception:
            return self._fallback(input_value)

    def dispatch(
        self,
        text: str,
        locale: str,
        client_request_id: str,
        expected_revision: int,
    ) -> Mapping[str, object]:
        try:
            input_value = RobotInput(client_request_id, text, locale)
        except Exception as error:
            raise RobotControlServiceError(
                400,
                "invalid_robot_input",
                str(error),
            ) from None
        if (
            isinstance(expected_revision, bool)
            or not isinstance(expected_revision, int)
            or expected_revision < 1
        ):
            raise RobotControlServiceError(
                400,
                "invalid_robot_integer",
                "Robot settings revision is invalid",
            )

        signature = (text, locale, expected_revision)
        with self._lock:
            remembered = self._history.get(client_request_id)
            if remembered is not None:
                if remembered[0] != signature:
                    raise RobotControlServiceError(
                        409,
                        "robot_input_idempotency_conflict",
                        "Robot input request ID was reused with other content",
                    )
                return deepcopy(remembered[1])
            if client_request_id in self._inflight:
                raise RobotControlServiceError(
                    409,
                    "robot_input_inflight",
                    "Robot input is already being processed",
                )
            self._inflight.add(client_request_id)

        try:
            settings = self._control.settings()
            facts = self._facts()
            decision = self._interpret(
                settings["model"],
                input_value,
                facts,
            )
            episode = None
            control = None
            speech_queued = False
            if decision.intent == PHYSICAL_TASK:
                episode = self._control.start(
                    text,
                    locale,
                    client_request_id,
                    expected_revision,
                )
            elif decision.intent == STOP_TASK:
                control = self._control.stop()
            elif (
                settings.get("speech_enabled") is True
                and self._speech_sink is not None
            ):
                try:
                    speech_queued = self._speech_sink(
                        client_request_id,
                        decision.reply_text,
                        locale,
                    ) is True
                except Exception:
                    speech_queued = False
            result = {
                "schema": ROBOT_INPUT_TURN_SCHEMA,
                "request_id": client_request_id,
                "intent": decision.intent,
                "confidence_milli": decision.confidence_milli,
                "answer_text": decision.reply_text,
                "episode": episode,
                "control": control,
                "speech_queued": speech_queued,
                "facts_captured_at_unix_ms": facts[
                    "captured_at_unix_ms"
                ],
            }
        finally:
            with self._lock:
                self._inflight.discard(client_request_id)

        with self._lock:
            self._history[client_request_id] = (signature, deepcopy(result))
            self._history.move_to_end(client_request_id)
            while len(self._history) > MAX_INPUT_HISTORY:
                self._history.popitem(last=False)
        return result


__all__ = ("ROBOT_INPUT_TURN_SCHEMA", "RobotInputService")
