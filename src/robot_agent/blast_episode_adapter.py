"""Thin agent-episode adapter over BLAST's single persistent BLE owner."""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Mapping

from .blast_observation_monitor import (
    CONTROLLER_ID,
    ROBOT_ID,
    BlastControllerError,
)
from .lm_studio_controller_action import (
    ABORT,
    COMPLETE,
    ControllerActionContext,
    ControllerActionPlannerResult,
)
from .robot_control_service import RobotEpisodeOutcome


BLAST_PROFILE_ID = ROBOT_ID
ACTION_COMMANDS = {
    "ADVANCE": "drive_forward",
    "REVERSE": "drive_reverse",
    "TURN_LEFT": "turn_left",
    "TURN_RIGHT": "turn_right",
}
DEFAULT_MAX_DECISIONS = 16
DEFAULT_MAX_OBSERVATION_AGE_MS = 3_000
DEFAULT_MIN_FORWARD_CLEARANCE_MM = 120


class BlastEpisodeError(RuntimeError):
    """One safely reportable BLAST episode failure."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)


class BlastEpisodeRuntimeAdapter:
    """Let the shared control service run one model-directed BLAST episode."""

    def __init__(
        self,
        *,
        controller,
        planner_factory: Callable[[str], object],
        max_decisions: int = DEFAULT_MAX_DECISIONS,
        max_observation_age_ms: int = DEFAULT_MAX_OBSERVATION_AGE_MS,
        minimum_forward_clearance_mm: int = (
            DEFAULT_MIN_FORWARD_CLEARANCE_MM
        ),
        monotonic_ms: Callable[[], int] = (
            lambda: time.monotonic_ns() // 1_000_000
        ),
    ) -> None:
        if (
            not callable(getattr(controller, "snapshot", None))
            or not callable(getattr(controller, "command", None))
            or not callable(planner_factory)
            or not callable(monotonic_ms)
            or isinstance(max_decisions, bool)
            or not isinstance(max_decisions, int)
            or not 1 <= max_decisions <= 128
            or isinstance(max_observation_age_ms, bool)
            or not isinstance(max_observation_age_ms, int)
            or not 100 <= max_observation_age_ms <= 60_000
            or isinstance(minimum_forward_clearance_mm, bool)
            or not isinstance(minimum_forward_clearance_mm, int)
            or not 1 <= minimum_forward_clearance_mm <= 2_000
        ):
            raise ValueError("BLAST episode adapter configuration is invalid")
        self.controller = controller
        self.planner_factory = planner_factory
        self.max_decisions = max_decisions
        self.max_observation_age_ms = max_observation_age_ms
        self.minimum_forward_clearance_mm = minimum_forward_clearance_mm
        self.monotonic_ms = monotonic_ms
        self._lock = threading.Lock()
        self._active_episode_id = None

    @staticmethod
    def _cancelled(context) -> bool:
        return (
            context.stop_requested.is_set()
            or context.emergency_stop_requested.is_set()
        )

    def _observation(self):
        snapshot = self.controller.snapshot()
        if (
            not isinstance(snapshot, Mapping)
            or snapshot.get("robot_id") != ROBOT_ID
            or snapshot.get("controller_id") != CONTROLLER_ID
            or snapshot.get("state") != "online"
            or not isinstance(snapshot.get("observation"), Mapping)
            or type(snapshot.get("last_observed_at_monotonic_ms")) is not int
        ):
            raise BlastEpisodeError(
                "blast_observation_unavailable",
                "BLAST has no current online observation",
            )
        age_ms = self.monotonic_ms() - snapshot[
            "last_observed_at_monotonic_ms"
        ]
        if age_ms < 0 or age_ms > self.max_observation_age_ms:
            raise BlastEpisodeError(
                "blast_observation_stale",
                "BLAST observation is stale",
            )
        observation = dict(snapshot["observation"])
        if observation.get("motion_active") is not False:
            raise BlastEpisodeError(
                "blast_motion_not_idle",
                "BLAST is still moving",
            )
        return {
            "observed_at_unix_ms": snapshot.get(
                "last_observed_at_unix_ms"
            ),
            "observed_at_monotonic_ms": snapshot[
                "last_observed_at_monotonic_ms"
            ],
            "age_ms": age_ms,
            "sensors": observation,
        }

    def _available_actions(self, observation) -> tuple[str, ...]:
        available = list(ACTION_COMMANDS)
        distance = observation["sensors"].get("distance_mm")
        if (
            isinstance(distance, (int, float))
            and not isinstance(distance, bool)
            and math.isfinite(float(distance))
            and distance <= self.minimum_forward_clearance_mm
        ):
            available.remove("ADVANCE")
        return tuple(available)

    @staticmethod
    def _outcome(reason: str, completed: bool, message: str):
        return RobotEpisodeOutcome(
            terminal_reason=reason,
            completed=completed,
            runtime_update={
                "current_action": None,
                "plan": [],
                "message": message,
            },
        )

    def run(self, context) -> RobotEpisodeOutcome:
        with self._lock:
            if self._active_episode_id is not None:
                raise BlastEpisodeError(
                    "blast_episode_already_active",
                    "A BLAST episode is already active",
                )
            self._active_episode_id = context.episode_id

        history = []
        try:
            planner = self.planner_factory(context.settings.model)
            if not callable(getattr(planner, "decide", None)):
                raise BlastEpisodeError(
                    "blast_planner_invalid",
                    "BLAST planner is invalid",
                )
            for _index in range(self.max_decisions):
                if self._cancelled(context):
                    return self._outcome("stopped", False, "stopped")
                observation = self._observation()
                available_actions = self._available_actions(observation)
                result = planner.decide(ControllerActionContext(
                    goal=context.request.goal,
                    locale=context.request.locale,
                    robot_id=ROBOT_ID,
                    controller_id=CONTROLLER_ID,
                    available_actions=available_actions,
                    observation=observation,
                    history=tuple(history[-12:]),
                ))
                if not isinstance(result, ControllerActionPlannerResult):
                    raise BlastEpisodeError(
                        "blast_planner_result_invalid",
                        "BLAST planner returned an invalid result",
                    )
                if self._cancelled(context):
                    return self._outcome("stopped", False, "stopped")
                decision = result.decision
                if decision.action not in (
                    available_actions + (COMPLETE, ABORT)
                ):
                    raise BlastEpisodeError(
                        "blast_planner_action_invalid",
                        "BLAST planner selected an unavailable action",
                    )
                context.publish({
                    "current_action": (
                        None
                        if decision.action in (COMPLETE, ABORT)
                        else decision.action
                    ),
                    "plan": list(decision.plan),
                    "model_latency_ms": result.latency_ms,
                    "message": decision.utterance or decision.assessment,
                    "obstacle": {
                        "distance_mm": observation["sensors"].get(
                            "distance_mm"
                        ),
                        "observed_at_monotonic_ms": observation[
                            "observed_at_monotonic_ms"
                        ],
                    },
                })
                if decision.action == COMPLETE:
                    return self._outcome(
                        "completed",
                        True,
                        decision.assessment,
                    )
                if decision.action == ABORT:
                    return self._outcome(
                        "planner_aborted",
                        False,
                        decision.assessment,
                    )
                try:
                    command_result = self.controller.command(
                        ACTION_COMMANDS[decision.action],
                        cancel_requested=lambda: self._cancelled(context),
                    )
                except BlastControllerError as error:
                    if (
                        error.code == "controller_command_interrupted"
                        and self._cancelled(context)
                    ):
                        return self._outcome("stopped", False, "stopped")
                    raise
                if not isinstance(command_result, Mapping):
                    raise BlastEpisodeError(
                        "blast_command_result_invalid",
                        "BLAST returned an invalid command result",
                    )
                result_observation = command_result.get("observation")
                history.append({
                    "action": decision.action,
                    "assessment": decision.assessment,
                    "result_observation": result_observation,
                })
                context.publish({
                    "current_action": decision.action,
                    "obstacle": {
                        "distance_mm": (
                            result_observation.get("distance_mm")
                            if isinstance(result_observation, Mapping)
                            else None
                        )
                    },
                })
            return self._outcome(
                "decision_budget_exhausted",
                False,
                "decision_budget_exhausted",
            )
        finally:
            with self._lock:
                if self._active_episode_id == context.episode_id:
                    self._active_episode_id = None

    def request_stop(self) -> None:
        with self._lock:
            active = self._active_episode_id is not None
        if active:
            self.controller.command("stop")

    def emergency_stop(self) -> None:
        self.controller.command("stop")


__all__ = (
    "ACTION_COMMANDS",
    "BLAST_PROFILE_ID",
    "BlastEpisodeError",
    "BlastEpisodeRuntimeAdapter",
)
