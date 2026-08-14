"""Thin agent-episode adapter over BLAST's single persistent BLE owner."""

from __future__ import annotations

import copy
from functools import partial
import math
import threading
import time
from typing import Callable, Mapping

from .blast_agentic_recovery import (
    BlastAgenticRecovery,
)
from .blast_action_admission import (
    admit_blast_spoken_action,
    fresh_blast_action_observation,
)
from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_episode_deadline import (
    SETTLED_OBSERVATION_HEADROOM_MS, BlastEpisodeDeadline,
    blast_action_deadline_headroom_ms)
from .blast_episode_map_trace import _BlastEpisodeMapTrace
from .blast_episode_speech import BlastEpisodeSpeech, blast_episode_cancelled
from .blast_observation_monitor import (
    CONTROLLER_ID,
    RANGE_STATE_MEASURED,
    RANGE_STATE_NO_VALID_DISTANCE,
    ROBOT_ID,
    BlastControllerError,
    blast_range_state,
    validate_blast_scan_ray_contract,
)
from .blast_scan_observation import current_side_scan
from .blast_scan_planar_projection import project_blast_scan_planar_surfaces
from .blast_scan_safety import (
    BlastScanPermitUnavailable,
    blast_scan_sweep_is_clear,
    issue_blast_scan_permit,
)
from .blast_stationary_recovery_flow import (
    begin_blast_iteration,
    recover_planner_iteration_actions,
    recover_scan_start_observation,
)
from .blast_spatial_map import BlastSpatialMapBridge
from .blast_navigation_action_profile import (
    BLAST_NAVIGATION_COMMANDS,
)
from .blast_navigation_motion_execution import (
    BlastNavigationMotionExecutor,
)
from .blast_mission_completion import blast_directional_completion_allowed
from .blast_navigation_state import (
    PlannerNavigationState,
)
from .blast_turn_safety import blast_turn_slice_allows_continuation
from .lm_studio_controller_action import (
    ABORT,
    COMPLETE,
    ControllerActionContext,
    ControllerActionPlannerResult,
)
from .physical_navigation_contract import (
    ADVANCE,
    REVERSE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from .robot_control_service import RobotEpisodeOutcome
BLAST_PROFILE_ID = ROBOT_ID
ACTION_COMMANDS = {
    action: BLAST_NAVIGATION_COMMANDS[action]
    for action in (ADVANCE, REVERSE, TURN_LEFT_90, TURN_RIGHT_90)
}
DEFAULT_MAX_DECISIONS = 16
DEFAULT_MAX_OBSERVATION_AGE_MS = 3_000
DEFAULT_MIN_FORWARD_CLEARANCE_MM = 120
DEFAULT_MINIMUM_FORWARD_PROGRESS_MM = 420
_PLANNER_ACTION_SOURCE = "PLANNER_ACTION"
_STARTUP_PERCEPTION_ACTION_SOURCE = "STARTUP_PERCEPTION"
_STARTUP_SURROUNDINGS_ACTION = "SCAN_SURROUNDINGS"
_SCAN_REFUSAL_CODES = frozenset(("scan_start_clearance_unverified",
                                 "scan_sweep_clearance_lost",
                                 "scan_sweep_observation_unverified"))


def _side_search_encoder_correlated(observation, motion_executor) -> bool:
    sensors = (
        observation.get("sensors")
        if isinstance(observation, Mapping) else None
    )
    if not isinstance(sensors, Mapping):
        sensors = observation
    matches = getattr(motion_executor, "observation_matches_anchor", None)
    return callable(matches) and matches(sensors)


def _navigation_drive_encoders_available(sensors) -> bool:
    motors = (
        sensors.get("motor_angles_deg")
        if isinstance(sensors, Mapping) else None
    )
    return (
        isinstance(motors, Mapping)
        and all(type(motors.get(role)) is int for role in (
            "left_drive", "right_drive",
        ))
    )


def _navigation_body_matched(sensors) -> bool:
    motors = sensors.get("motor_angles_deg")
    angle = motors.get("body") if isinstance(motors, Mapping) else None
    return (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.range_sensor_extrinsics
        .matches_navigation_body_angle(angle)
    )


def _minimum_rotation_clearance_mm() -> int:
    return (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .minimum_rotation_clearance_mm()
    )


def _planner_scan_geometry_checked(
    action, observation, latest_scan_view, pose,
):
    return (
        action == SCAN_FRONT_ARC
        and blast_range_state(
            observation["sensors"].get("distance_mm")
        ) == RANGE_STATE_NO_VALID_DISTANCE
        and latest_scan_view is not None
        and blast_scan_sweep_is_clear(latest_scan_view, pose)
    )


class BlastEpisodeError(RuntimeError):
    """One safely reportable BLAST episode failure."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)


class BlastEpisodeRuntimeAdapter:
    """Run one BLAST episode with model strategy and verified host progress."""

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
        minimum_forward_progress_mm: int = (
            DEFAULT_MINIMUM_FORWARD_PROGRESS_MM
        ),
        speech_runtime_factory=None,
        speech_locales=(),
        spatial_map_bridge=None,
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
            or isinstance(minimum_forward_progress_mm, bool)
            or not isinstance(minimum_forward_progress_mm, int)
            or not 1 <= minimum_forward_progress_mm <= 2_000
        ):
            raise ValueError("BLAST episode adapter configuration is invalid")
        if speech_runtime_factory is not None and not callable(
            speech_runtime_factory
        ):
            raise ValueError("speech runtime factory is invalid")
        if (
            not isinstance(speech_locales, tuple)
            or len(set(speech_locales)) != len(speech_locales)
            or any(locale not in ("sv", "en") for locale in speech_locales)
            or speech_locales and speech_runtime_factory is None
        ):
            raise ValueError("runtime speech locales are invalid")
        if spatial_map_bridge is None:
            spatial_map_bridge = BlastSpatialMapBridge(
                robot_id=ROBOT_ID,
                controller_instance_id=CONTROLLER_ID,
            )
        if any(
            not callable(getattr(spatial_map_bridge, name, None))
            for name in (
                "begin_episode",
                "offer_pose",
                "offer_trace",
                "snapshot",
                "close",
            )
        ):
            raise ValueError("BLAST spatial map bridge is invalid")
        self.controller = controller
        self.planner_factory = planner_factory
        self.max_decisions = max_decisions
        self.max_observation_age_ms = max_observation_age_ms
        self.minimum_forward_clearance_mm = minimum_forward_clearance_mm
        self.minimum_forward_progress_mm = minimum_forward_progress_mm
        self.speech_runtime_factory = speech_runtime_factory
        self.speech_locales = speech_locales
        self.spatial_map_provider = spatial_map_bridge
        self.monotonic_ms = monotonic_ms
        self._lock = threading.Lock()
        self._active_episode_id = None
        self._active_speech = None
        self._speech_available = True

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

    @staticmethod
    def _scan_is_current(history) -> bool:
        for item in reversed(history):
            action = item.get("action")
            if action == SCAN_FRONT_ARC:
                return True
            if action in ACTION_COMMANDS:
                return False
        return False

    @staticmethod
    def _scan_evidence_is_fresh(history) -> bool:
        """Whether the latest scan evidence still matches the current pose."""

        for item in reversed(history):
            action = item.get("action")
            if action in (SCAN_FRONT_ARC, _STARTUP_SURROUNDINGS_ACTION):
                return True
            if action in ACTION_COMMANDS:
                return False
        return False

    @staticmethod
    def _has_scan_evidence(history) -> bool:
        return any(
            item.get("action") in (
                SCAN_FRONT_ARC, _STARTUP_SURROUNDINGS_ACTION,
            )
            for item in history
        )

    @staticmethod
    def _current_scan_allows_quarter_turn(history) -> bool:
        if not history or history[-1].get("action") != SCAN_FRONT_ARC:
            return False
        scan = history[-1].get("scan")
        rays = scan.get("rays") if isinstance(scan, Mapping) else None
        if not (
            isinstance(rays, list)
            and rays
            and isinstance(rays[0], Mapping)
            and rays[0].get("side") == "center"
            and rays[0].get("observation_settled") is True
            and rays[0].get("range_state") == RANGE_STATE_MEASURED
        ):
            return False
        return (
            float(rays[0]["distance_mm"])
            > _minimum_rotation_clearance_mm()
        )

    @staticmethod
    def _current_range_allows_rotation(observation) -> bool:
        distance = observation["sensors"].get("distance_mm")
        if blast_range_state(distance) != RANGE_STATE_MEASURED:
            return False
        return float(distance) > _minimum_rotation_clearance_mm()

    def _observation_allows_quality_retry(self, observation) -> bool:
        """Whether one idle reading is safe to observe again in place."""

        distance = observation["sensors"].get("distance_mm")
        return (
            blast_range_state(distance) == RANGE_STATE_NO_VALID_DISTANCE
            or self._current_range_allows_rotation(observation)
        )

    def _current_observation_allows_action(self, action, observation) -> bool:
        sensors = observation["sensors"]
        if not _navigation_body_matched(sensors):
            return False
        distance = sensors.get("distance_mm")
        if action == ADVANCE:
            return (
                blast_range_state(distance) == RANGE_STATE_MEASURED
                and float(distance) > self.minimum_forward_clearance_mm
            )
        if action == SCAN_FRONT_ARC:
            return (
                _navigation_drive_encoders_available(sensors)
                and (
                    self._current_range_allows_rotation(observation)
                    or blast_range_state(distance)
                    == RANGE_STATE_NO_VALID_DISTANCE
                )
            )
        if action in (TURN_LEFT_90, TURN_RIGHT_90):
            return (
                _navigation_drive_encoders_available(sensors)
                and self._current_range_allows_rotation(observation)
            )
        return False

    def _available_actions(self, observation, history=()) -> tuple[str, ...]:
        # BLAST has no rear-facing clearance source yet. Keep reverse in the
        # executor contract, but never offer it as a planner action.
        available = [
            action
            for action in (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90)
            if self._current_observation_allows_action(action, observation)
        ]
        if (
            self._current_observation_allows_action(
                SCAN_FRONT_ARC, observation
            )
            and not self._scan_evidence_is_fresh(history)
        ):
            available.append(SCAN_FRONT_ARC)
        return tuple(available)

    @staticmethod
    def _heading(sensors):
        imu = sensors.get("imu") if isinstance(sensors, Mapping) else None
        heading = imu.get("heading_deg") if isinstance(imu, Mapping) else None
        if (
            isinstance(heading, bool)
            or not isinstance(heading, (int, float))
            or not math.isfinite(float(heading))
        ):
            return None
        return float(heading)

    @staticmethod
    def _heading_delta(heading, reference):
        if heading is None or reference is None:
            return None
        return (heading - reference + 180.0) % 360.0 - 180.0

    @classmethod
    def _with_navigation_reference(cls, observation, start_heading):
        enriched = dict(observation)
        current_heading = cls._heading(observation["sensors"])
        enriched["navigation_reference"] = {
            "episode_start_heading_deg": start_heading,
            "current_heading_deg": current_heading,
            "heading_error_deg": cls._heading_delta(
                current_heading,
                start_heading,
            ),
        }
        return enriched

    @staticmethod
    def _outcome(reason: str, completed: bool, message: str):
        return RobotEpisodeOutcome(
            terminal_reason=reason,
            completed=completed,
            runtime_update={
                "current_action": None,
                "active_route": None,
                "plan": [],
                "message": message,
            },
        )

    def _control_outcome(self, context, deadline, headroom_ms=0):
        value = deadline.outcome(
            cancelled=blast_episode_cancelled(context),
            headroom_ms=headroom_ms,
        )
        if value is None:
            return None
        return self._outcome(value[0], False, value[1])

    def _dispatch_action(
        self,
        action,
        motion_executor,
        control_requested,
        *,
        allow_turn_no_valid_with_bounded_evidence=False,
        action_permit=None,
    ):
        if action == SCAN_FRONT_ARC:
            kwargs = {"cancel_requested": control_requested}
            if action_permit is not None:
                kwargs["action_permit"] = action_permit
            result = self.controller.command("scan_front_arc", **kwargs)
            return result, None
        continuation_gate = (
            partial(
                blast_turn_slice_allows_continuation,
                allow_no_valid_distance_with_bounded_evidence=(
                    allow_turn_no_valid_with_bounded_evidence
                ),
            ) if action in (TURN_LEFT_90, TURN_RIGHT_90) else None
        )
        execution = motion_executor.execute(
            action,
            cancel_requested=control_requested,
            continue_requested=continuation_gate,
        )
        return execution.controller_results[-1], execution

    def _scan_failure_outcome(self, code):
        if code not in _SCAN_REFUSAL_CODES:
            return None
        sweep_stopped = code != "scan_start_clearance_unverified"
        message = (
            "BLAST scan stopped between pulses; reposition before retry"
            if sweep_stopped
            else "BLAST scan could not start from settled safety evidence"
        )
        return self._outcome("no_safe_blast_action", False, message)

    def _dispatch_episode_action(
        self, *, action, observation, geometry_checked, motion_executor,
        prior_receipt,
        allow_turn_no_valid_with_bounded_evidence, context, deadline_ms,
        map_trace=None, perception_only_scan=False,
    ):
        outcome = self._control_outcome(
            context, deadline_ms, blast_action_deadline_headroom_ms(action),
        )
        if outcome is not None:
            return None, None, observation, outcome
        control_requested = lambda: self._control_outcome(
            context, deadline_ms) is not None
        for attempt in range(2):
            if not _side_search_encoder_correlated(
                observation, motion_executor,
            ):
                raise BlastEpisodeError(
                    "blast_action_start_unverified",
                    "BLAST drive encoders no longer match its trusted pose",
                )
            action_permit = self._scan_action_permit(
                action=action,
                observation=observation,
                geometry_checked=geometry_checked,
                pose=motion_executor.pose,
                prior_receipt=prior_receipt,
                expected_drive_angles=(
                    motion_executor.expected_start_angles
                ),
                perception_only=perception_only_scan,
            )
            try:
                command_result, execution = self._dispatch_action(
                    action,
                    motion_executor,
                    control_requested,
                    allow_turn_no_valid_with_bounded_evidence=(
                        allow_turn_no_valid_with_bounded_evidence
                    ),
                    action_permit=action_permit,
                )
                return command_result, execution, observation, None
            except BlastControllerError as error:
                if (
                    action == SCAN_FRONT_ARC
                    and error.motion_started is not False
                ):
                    motion_executor.invalidate_after_failed_scan()
                    if map_trace is not None:
                        map_trace.invalidate_localization()
                outcome = self._control_outcome(context, deadline_ms)
                if (
                    error.code in (
                        {"controller_command_interrupted"}
                        | _SCAN_REFUSAL_CODES
                    )
                    and outcome is not None
                ):
                    return None, None, observation, outcome
                retryable = (
                    attempt == 0
                    and error.code == "scan_start_clearance_unverified"
                    and error.motion_started is False
                )
                if retryable:
                    outcome = self._control_outcome(
                        context,
                        deadline_ms,
                        blast_action_deadline_headroom_ms(action),
                    )
                    if outcome is not None:
                        return None, None, observation, outcome
                    try:
                        retry_observation = (
                            recover_scan_start_observation(
                                self,
                                context=context,
                                deadline_ms=deadline_ms,
                                motion_executor=motion_executor,
                                episode_start_heading=(
                                    observation.get(
                                        "navigation_reference", {}
                                    ).get("episode_start_heading_deg")
                                ),
                                allow_no_return=(
                                    perception_only_scan
                                    or action_permit is not None
                                    and blast_range_state(
                                        observation["sensors"].get(
                                            "distance_mm"
                                        )
                                    ) == RANGE_STATE_NO_VALID_DISTANCE
                                ),
                                minimum_safe_distance_mm=(
                                    _minimum_rotation_clearance_mm()
                                ),
                            )
                        )
                        retry_observation, retry_outcome = retry_observation
                    except BlastControllerError:
                        raise
                else:
                    retry_observation, retry_outcome = None, None
                if retry_outcome is not None:
                    return None, None, observation, retry_outcome
                if retry_observation is not None:
                    outcome = self._control_outcome(
                        context,
                        deadline_ms,
                        blast_action_deadline_headroom_ms(action),
                    )
                    if outcome is not None:
                        return None, None, observation, outcome
                    observation = retry_observation
                    continue
                scan_outcome = self._scan_failure_outcome(error.code)
                if scan_outcome is not None:
                    return None, None, observation, scan_outcome
                raise
        raise AssertionError("BLAST action retry loop exhausted")

    def _fresh_planner_action_observation(
        self,
        *,
        action,
        selects_detour_side,
        episode_start_heading,
        motion_executor,
        cancel_requested,
        force_remeasure=False,
        allow_turn_no_valid_with_bounded_evidence=False,
        context=None, deadline_ms=None,
    ):
        return fresh_blast_action_observation(
            self, action=action, selects_detour_side=selects_detour_side,
            episode_start_heading=episode_start_heading,
            motion_executor=motion_executor,
            cancel_requested=cancel_requested,
            episode_error_type=BlastEpisodeError,
            encoder_anchor_correlated=_side_search_encoder_correlated,
            navigation_body_matched=_navigation_body_matched,
            force_remeasure=force_remeasure,
            allow_turn_no_valid_with_bounded_evidence=(
                allow_turn_no_valid_with_bounded_evidence
            ),
            context=context, deadline_ms=deadline_ms,
        )

    def _fresh_planner_observation_or_stop(
        self, action, selects_detour_side, episode_start_heading,
        motion_executor, context, deadline_ms, force_remeasure=False,
        allow_turn_no_valid_with_bounded_evidence=False,
    ):
        control_requested = lambda: (
            self._control_outcome(
                context, deadline_ms, SETTLED_OBSERVATION_HEADROOM_MS,
            ) is not None
        )
        try:
            observation = self._fresh_planner_action_observation(
                action=action,
                selects_detour_side=selects_detour_side,
                episode_start_heading=episode_start_heading,
                motion_executor=motion_executor,
                cancel_requested=control_requested,
                force_remeasure=force_remeasure,
                allow_turn_no_valid_with_bounded_evidence=(
                    allow_turn_no_valid_with_bounded_evidence
                ),
                context=context, deadline_ms=deadline_ms,
            )
        except BlastControllerError as error:
            outcome = self._control_outcome(
                context, deadline_ms, SETTLED_OBSERVATION_HEADROOM_MS,
            )
            if error.code == "controller_command_interrupted" and outcome:
                return None, outcome
            raise
        return observation, self._control_outcome(
            context, deadline_ms, blast_action_deadline_headroom_ms(action),
        )

    def _planner_step(
        self,
        *,
        planner,
        context,
        observation,
        history,
        available_actions,
        completion_allowed,
        scan_allows_turn,
        latest_scan_view,
        motion_executor,
        episode_start_heading,
        deadline_ms,
        abort_allowed,
        local_map_evidence,
    ):
        outcome = self._control_outcome(context, deadline_ms)
        if outcome is not None: return None, outcome
        try:
            result = planner.decide(ControllerActionContext(
                goal=context.request.goal,
                locale=context.request.locale,
                robot_id=ROBOT_ID,
                controller_id=CONTROLLER_ID,
                available_actions=available_actions,
                observation=observation,
                history=tuple(history[-12:]),
                completion_allowed=completion_allowed,
                abort_allowed=abort_allowed,
                robot_relative_side_scan=current_side_scan(history, latest_scan_view),
                local_map_evidence=local_map_evidence,
            ))
        except Exception:
            outcome = self._control_outcome(context, deadline_ms)
            if outcome is not None:
                return None, outcome
            raise
        outcome = self._control_outcome(context, deadline_ms)
        if outcome is not None:
            return None, outcome
        if not isinstance(result, ControllerActionPlannerResult):
            raise BlastEpisodeError(
                "blast_planner_result_invalid",
                "BLAST planner returned an invalid result",
            )
        decision = result.decision
        action = decision.action
        scan_guided_turn = (
            self._scan_is_current(history)
            and scan_allows_turn
            and action in (TURN_LEFT_90, TURN_RIGHT_90)
        )
        terminal_actions = tuple(
            action for action in (COMPLETE, ABORT)
            if (
                action == COMPLETE and completion_allowed
                or action == ABORT and abort_allowed
            )
        )
        if action not in available_actions + terminal_actions:
            raise BlastEpisodeError(
                "blast_planner_action_invalid",
                "BLAST planner selected an unavailable action",
            )
        if action in ACTION_COMMANDS or action == SCAN_FRONT_ARC:
            observation, stopped = self._fresh_planner_observation_or_stop(
                action, scan_guided_turn, episode_start_heading,
                motion_executor, context, deadline_ms,
            )
            if stopped is not None:
                return None, stopped
        assessment = decision.assessment
        context.publish({
            "current_action": None if action in (COMPLETE, ABORT) else action,
            "plan": list(decision.plan),
            "model_latency_ms": result.latency_ms,
            "message": decision.utterance or assessment,
            "obstacle": {
                "distance_mm": observation["sensors"].get("distance_mm"),
                "observed_at_monotonic_ms": observation[
                    "observed_at_monotonic_ms"
                ],
            },
        })
        outcome = self._control_outcome(context, deadline_ms)
        if outcome is not None:
            return None, outcome
        if action == COMPLETE:
            return None, self._outcome("completed", True, assessment)
        if action == ABORT:
            return None, self._outcome(
                "planner_aborted", False, assessment,
            )
        return {
            "action": action,
            "assessment": assessment,
            "utterance": decision.utterance,
            "plan": list(decision.plan),
            "action_source": _PLANNER_ACTION_SOURCE,
            "observation": observation,
            "selects_detour_side": False,
            "bounded_turn_no_valid_eligible": scan_guided_turn,
        }, None
    def _scan_action_permit(
        self, *, action, observation, geometry_checked, pose, prior_receipt,
        expected_drive_angles=None, perception_only=False,
    ):
        try:
            return issue_blast_scan_permit(
                controller=self.controller,
                action=action,
                distance_mm=observation["sensors"].get("distance_mm"),
                geometry_checked=geometry_checked,
                pose=pose,
                prior_receipt=prior_receipt,
                expected_drive_angles=expected_drive_angles,
                perception_only=perception_only,
            )
        except BlastScanPermitUnavailable as error:
            raise BlastEpisodeError(error.code, str(error)) from None

    def _begin_map_trace(
        self, context, pose, observation, episode_start_heading,
    ):
        return _BlastEpisodeMapTrace(
            bridge=self.spatial_map_provider,
            episode_id=context.episode_id,
            pose=pose,
            observation=observation["sensors"],
            observed_at_unix_ms=observation["observed_at_unix_ms"],
            episode_start_heading=episode_start_heading,
            minimum_forward_progress_mm=self.minimum_forward_progress_mm,
        )

    @staticmethod
    def _publish_action_result(
        context, action, result_observation, scan, planar_projection,
    ):
        update = {
            "current_action": action,
            "obstacle": {
                "distance_mm": (
                    result_observation.get("distance_mm")
                    if isinstance(result_observation, Mapping)
                    else None
                )
            },
        }
        diagnostic_scan = None
        if isinstance(scan, Mapping):
            diagnostic_scan = dict(scan)
            if planar_projection is not None:
                diagnostic_scan["planar_projection"] = planar_projection
            update["scan"] = diagnostic_scan
        context.publish(update)
        return diagnostic_scan

    def _record_episode_action_result(
        self, *, action, action_source, assessment, plan, command_result,
        execution, scan_pose, motion_executor, history, map_trace, context,
        published_action=None,
    ):
        """Record one result, invalidating any rejected completed scan."""

        try:
            return self._retain_episode_action_result(
                action=action,
                action_source=action_source,
                assessment=assessment,
                plan=plan,
                command_result=command_result,
                execution=execution,
                scan_pose=scan_pose,
                motion_executor=motion_executor,
                history=history,
                map_trace=map_trace,
                context=context,
                published_action=published_action,
            )
        except Exception:
            if action == SCAN_FRONT_ARC:
                motion_executor.invalidate_after_failed_scan()
                map_trace.invalidate_localization()
            raise

    def _retain_episode_action_result(
        self, *, action, action_source, assessment, plan, command_result,
        execution, scan_pose, motion_executor, history, map_trace, context,
        published_action=None,
    ):
        """Validate and retain one result through the shared scan/map path."""

        if not isinstance(command_result, Mapping):
            raise BlastEpisodeError(
                "blast_command_result_invalid",
                "BLAST returned an invalid command result",
            )
        result_observation = command_result.get("observation")
        history_item = {
            "action": action,
            "assessment": assessment,
            "plan": list(plan),
            "action_source": action_source,
            "result_observation": result_observation,
            "observation_settled": command_result.get(
                "observation_settled"
            ),
            "pose": motion_executor.pose.to_dict(),
        }
        if action in ACTION_COMMANDS:
            if execution is None:
                raise BlastEpisodeError(
                    "blast_command_result_invalid",
                    "BLAST motion returned no verified execution",
                )
            history_item["motion"] = execution.motion.to_dict()
            history_item["pose"] = execution.pose.to_dict()
        scan = command_result.get("scan")
        planar_projection = None
        latest_scan_view = None
        if action == SCAN_FRONT_ARC and not isinstance(scan, Mapping):
            raise BlastEpisodeError(
                "blast_scan_result_invalid",
                "BLAST returned an invalid scan result",
            )
        if isinstance(scan, Mapping):
            try:
                scan = validate_blast_scan_ray_contract(scan)
            except ValueError:
                raise BlastEpisodeError(
                    "blast_scan_result_invalid",
                    "BLAST returned an invalid scan result",
                ) from None
            sensor = (
                BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
                .range_sensor_extrinsics
            )
            final_motors = (
                result_observation.get("motor_angles_deg")
                if isinstance(result_observation, Mapping)
                else None
            )
            body_angles = [ray["body_motor_angle_deg"] for ray in (
                scan.get("angular_rays", scan["rays"])
            )] + [
                final_motors.get("body")
                if isinstance(final_motors, Mapping)
                else None
            ]
            if not all(
                sensor.matches_navigation_body_angle(value)
                for value in body_angles
            ):
                raise BlastEpisodeError(
                    "blast_scan_sensor_pose_unverified",
                    "BLAST scan body encoder did not match its "
                    "provisional navigation reference",
                )
            history_item["odometry_reanchored_after_scan"] = (
                motion_executor.reanchor_after_restored_scan(command_result)
            )
            history_item["pose"] = motion_executor.pose.to_dict()
            try:
                planar_projection = project_blast_scan_planar_surfaces(
                    scan=scan,
                    scan_pose=scan_pose,
                )
            except ValueError:
                planar_projection = None
            planner_scan = copy.deepcopy(scan)
            planner_rays = (
                planner_scan["rays"]
                + planner_scan.get("angular_rays", [])
            )
            for ray in planner_rays:
                if ray["observation_settled"] is not True:
                    ray["distance_mm"] = None
                    ray["range_state"] = "UNRESOLVED_SWEEP_ONLY"
                elif ray["range_state"] != RANGE_STATE_MEASURED:
                    ray["distance_mm"] = None
            history_item["scan"] = planner_scan
            if planar_projection is not None:
                latest_scan_view = {
                    "scan_pose": scan_pose.to_dict(),
                    "scan": copy.deepcopy(scan),
                    "planar_projection": copy.deepcopy(planar_projection),
                }
        map_trace.record_action(
            action, motion_executor.pose, result_observation,
            None, None, latest_scan_view,
            pose_observed=(action in ACTION_COMMANDS or
                           action == SCAN_FRONT_ARC),
        )
        history.append(history_item)
        self._publish_action_result(
            context,
            action if published_action is None else published_action,
            result_observation,
            scan,
            planar_projection,
        )
        return latest_scan_view

    def _run_startup_perception(
        self, *, observation, available_actions, scan_allows_turn,
        latest_scan_view, history, motion_executor, episode_start_heading,
        map_trace, navigation_state, recovery, context, deadline_ms,
    ):
        """Acquire one complete encoder-measured view before Gemma decides."""

        initial_scan_view_count = len(map_trace.planar_scan_views)
        startup_history = []
        action = SCAN_FRONT_ARC
        try:
            observation, outcome = self._fresh_planner_observation_or_stop(
                action, False, episode_start_heading, motion_executor,
                context, deadline_ms,
            )
        except BlastEpisodeError:
            outcome = self._outcome(
                "blast_startup_perception_incomplete", False,
                "BLAST could not safely begin its surroundings scan",
            )
        if outcome is not None:
            return (
                observation, available_actions, scan_allows_turn,
                latest_scan_view, outcome,
            )
        command_result, execution, observation, outcome = (
            self._dispatch_episode_action(
                action=action,
                observation=observation,
                geometry_checked=False,
                motion_executor=motion_executor,
                prior_receipt=history[-1] if history else None,
                allow_turn_no_valid_with_bounded_evidence=False,
                context=context,
                deadline_ms=deadline_ms,
                map_trace=map_trace,
                perception_only_scan=True,
            )
        )
        if outcome is not None:
            startup_outcome = (
                outcome if outcome.terminal_reason in (
                    "stopped", "episode_deadline_elapsed",
                    "episode_deadline_headroom_insufficient",
                ) else self._outcome(
                    "blast_startup_perception_incomplete", False,
                    "BLAST could not safely complete its surroundings scan",
                )
            )
            return (
                observation, available_actions, scan_allows_turn,
                latest_scan_view, startup_outcome,
            )
        latest_scan_view = self._record_episode_action_result(
            action=action,
            action_source=_STARTUP_PERCEPTION_ACTION_SOURCE,
            assessment="Mandatory startup surroundings acquisition",
            plan=(action,),
            command_result=command_result,
            execution=execution,
            scan_pose=motion_executor.pose,
            motion_executor=motion_executor,
            history=startup_history,
            map_trace=map_trace,
            context=context,
            published_action=_STARTUP_SURROUNDINGS_ACTION,
        )
        outcome = self._control_outcome(context, deadline_ms)
        if outcome is not None:
            return (
                observation, available_actions, scan_allows_turn,
                latest_scan_view, outcome,
            )
        if (
            latest_scan_view is None
            or not motion_executor.localization_valid
            or len(map_trace.planar_scan_views)
            != initial_scan_view_count + 1
        ):
            return (
                observation, available_actions, scan_allows_turn,
                latest_scan_view,
                self._outcome(
                    "blast_startup_perception_incomplete", False,
                    "BLAST startup scan produced no localized surroundings",
                ),
            )
        history.append({
            "action": _STARTUP_SURROUNDINGS_ACTION,
            "action_source": _STARTUP_PERCEPTION_ACTION_SOURCE,
            "assessment": "Mandatory startup surroundings acquisition",
            "plan": [_STARTUP_SURROUNDINGS_ACTION],
            "result_observation": startup_history[-1][
                "result_observation"
            ],
            "observation_settled": startup_history[-1][
                "observation_settled"
            ],
            "pose": motion_executor.pose.to_dict(),
            "scan_view_count": 1,
            "scan_state": startup_history[-1]["scan"]["state"],
            "sweep_coverage_deg": startup_history[-1]["scan"].get(
                "sweep_coverage_deg"
            ),
        })
        final_observation = startup_history[-1]["result_observation"]
        context.publish({
            "current_action": None,
            "scan": None,
            "obstacle": {
                "distance_mm": (
                    final_observation.get("distance_mm")
                    if isinstance(final_observation, Mapping)
                    else None
                )
            },
        })
        (
            observation, available_actions, scan_allows_turn,
            _runtime, outcome,
        ) = begin_blast_iteration(
            self, context=context, deadline_ms=deadline_ms,
            index=1, history=history, selected_detour_side=None,
            navigation_state=navigation_state,
            latest_scan_view=latest_scan_view, recovery=recovery,
            motion_executor=motion_executor,
            episode_start_heading=episode_start_heading,
            motion_executor_factory=BlastNavigationMotionExecutor,
            minimum_rotation_clearance_mm=_minimum_rotation_clearance_mm(),
        )
        return (
            observation, available_actions, scan_allows_turn,
            latest_scan_view, outcome,
        )

    def run(self, context) -> RobotEpisodeOutcome:
        with self._lock:
            if self._active_episode_id is not None:
                raise BlastEpisodeError(
                    "blast_episode_already_active",
                    "A BLAST episode is already active",
                )
            self._active_episode_id = context.episode_id
            speech_factory = (
                self.speech_runtime_factory if self._speech_available else None)
        history, episode_start_heading, motion_executor = [], None, None
        navigation_state, recovery, latest_scan_view = PlannerNavigationState(), BlastAgenticRecovery(), None
        map_trace = None
        speech = BlastEpisodeSpeech(
            factory=speech_factory,
            supported_locales=self.speech_locales,
            context=context,
        )
        try:
            deadline_ms = BlastEpisodeDeadline.begin(
                context.settings, self.monotonic_ms)
            planner = self.planner_factory(context.settings.model)
            if not callable(getattr(planner, "decide", None)):
                raise BlastEpisodeError(
                    "blast_planner_invalid",
                    "BLAST planner is invalid",
                )
            with self._lock:
                if self._active_episode_id == context.episode_id:
                    self._active_speech = speech
            speech.start()
            for _index in range(self.max_decisions):
                outcome = self._control_outcome(context, deadline_ms)
                if outcome is not None:
                    return outcome
                (observation, available_actions, scan_allows_turn,
                 iteration_runtime, outcome) = begin_blast_iteration(
                    self, context=context, deadline_ms=deadline_ms,
                    index=_index, history=history,
                    selected_detour_side=None,
                    navigation_state=navigation_state,
                    latest_scan_view=latest_scan_view, recovery=recovery,
                    motion_executor=motion_executor,
                    episode_start_heading=episode_start_heading,
                    motion_executor_factory=BlastNavigationMotionExecutor,
                    minimum_rotation_clearance_mm=(
                        _minimum_rotation_clearance_mm()
                    ))
                if outcome is not None: return outcome
                motion_executor, episode_start_heading = iteration_runtime
                if _index == 0:
                    map_trace = self._begin_map_trace(
                        context, motion_executor.pose, observation,
                        episode_start_heading,
                    )
                    (
                        observation, available_actions, scan_allows_turn,
                        latest_scan_view, outcome,
                    ) = self._run_startup_perception(
                        observation=observation,
                        available_actions=available_actions,
                        scan_allows_turn=scan_allows_turn,
                        latest_scan_view=latest_scan_view,
                        history=history,
                        motion_executor=motion_executor,
                        episode_start_heading=episode_start_heading,
                        map_trace=map_trace,
                        navigation_state=navigation_state,
                        recovery=recovery,
                        context=context,
                        deadline_ms=deadline_ms,
                    )
                    if outcome is not None:
                        return outcome
                completion_allowed = blast_directional_completion_allowed(
                    mission=map_trace.mission, pose=motion_executor.pose,
                    localization_valid=motion_executor.localization_valid,
                    scan_fresh=(
                        not self._has_scan_evidence(history)
                        or self._scan_evidence_is_fresh(history)
                    ),
                )
                if not available_actions and not completion_allowed:
                    observation, available_actions, refreshed_turns, outcome = (
                        recover_planner_iteration_actions(
                        self, observation=observation,
                        available_actions=available_actions,
                        completion_allowed=completion_allowed, context=context,
                        deadline_ms=deadline_ms,
                        motion_executor=motion_executor,
                        episode_start_heading=episode_start_heading,
                        history=history,
                        selected_detour_side=None,
                        navigation_state=navigation_state,
                        latest_scan_view=latest_scan_view, recovery=recovery,
                    ))
                    if outcome is not None: return outcome
                    if refreshed_turns is not None:
                        scan_allows_turn = refreshed_turns
                if not available_actions and not completion_allowed:
                    return self._outcome(
                        "no_safe_blast_action",
                        False,
                        "BLAST has no currently observed safe motion or scan",
                    )
                step, outcome = self._planner_step(
                    planner=planner, context=context,
                    observation=observation, history=history,
                    available_actions=available_actions,
                    completion_allowed=completion_allowed,
                    scan_allows_turn=scan_allows_turn,
                    latest_scan_view=latest_scan_view,
                    motion_executor=motion_executor,
                    episode_start_heading=episode_start_heading,
                    deadline_ms=deadline_ms,
                    abort_allowed=(
                        not available_actions and not completion_allowed
                    ),
                    local_map_evidence=(
                        map_trace.planner_local_map_evidence(
                            motion_executor.pose
                        )
                    ),
                )
                if outcome is not None:
                    return outcome
                action = step["action"]
                assessment = step["assessment"]
                plan = step["plan"]
                action_source = step["action_source"]
                observation = step["observation"]
                scan_pose = motion_executor.pose if action == SCAN_FRONT_ARC else None
                observation, outcome = admit_blast_spoken_action(
                    self, speech, step, observation, motion_executor,
                    episode_start_heading, context, deadline_ms,
                    len(history) + 1)
                if outcome is not None:
                    return outcome
                no_return_scan_geometry_checked = (
                    _planner_scan_geometry_checked(
                        action,
                        observation,
                        latest_scan_view,
                        motion_executor.pose,
                    )
                )
                (
                    command_result,
                    execution,
                    observation,
                    outcome,
                ) = self._dispatch_episode_action(
                    action=action,
                    observation=observation,
                    geometry_checked=no_return_scan_geometry_checked,
                    motion_executor=motion_executor,
                    prior_receipt=(history[-1] if history else None),
                    allow_turn_no_valid_with_bounded_evidence=(
                        step["bounded_turn_no_valid_eligible"]
                    ),
                    context=context,
                    deadline_ms=deadline_ms,
                    map_trace=map_trace,
                    perception_only_scan=(action == SCAN_FRONT_ARC),
                )
                if outcome is not None:
                    return outcome
                new_scan_view = self._record_episode_action_result(
                    action=action,
                    action_source=action_source,
                    assessment=assessment,
                    plan=plan,
                    command_result=command_result,
                    execution=execution,
                    scan_pose=scan_pose,
                    motion_executor=motion_executor,
                    history=history,
                    map_trace=map_trace,
                    context=context,
                )
                if action == SCAN_FRONT_ARC:
                    latest_scan_view = new_scan_view
                outcome = self._control_outcome(context, deadline_ms)
                if outcome is not None:
                    return outcome
            return self._outcome(
                "decision_budget_exhausted",
                False,
                "decision_budget_exhausted",
            )
        finally:
            if map_trace is not None:
                try:
                    map_trace.finalize()
                except Exception:
                    pass
            speech_closed = speech.close()
            with self._lock:
                if self._active_episode_id == context.episode_id:
                    self._active_episode_id = None
                    self._active_speech = None
                    if not speech_closed:
                        # Never overlap audio; navigation stays available.
                        self._speech_available = False

    def request_stop(self) -> None:
        with self._lock:
            episode_id = self._active_episode_id
            speech = self._active_speech
        if episode_id is not None:
            try:
                self.controller.command("stop")
            finally:
                if speech is not None:
                    speech.cancel()

    def emergency_stop(self) -> None:
        with self._lock:
            episode_id = self._active_episode_id
            speech = self._active_speech
        try:
            self.controller.command("stop")
        finally:
            if episode_id is not None and speech is not None:
                speech.cancel()


__all__ = ("ACTION_COMMANDS", "BLAST_PROFILE_ID", "BlastEpisodeError", "BlastEpisodeRuntimeAdapter")
