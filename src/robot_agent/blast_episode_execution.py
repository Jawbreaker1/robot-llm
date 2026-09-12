"""Execute BLAST actions and initial perception through its controller owner."""

from __future__ import annotations

from functools import partial
from typing import Mapping
from .blast_navigation_calibration import BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
from .blast_episode_deadline import blast_action_deadline_headroom_ms
from .blast_observation_monitor import (
    RANGE_STATE_NO_VALID_DISTANCE,
    ROBOT_ID,
    BlastControllerError,
    blast_range_state,
)
from .blast_scan_safety import BlastScanPermitUnavailable, issue_blast_scan_permit
from .blast_stationary_recovery_flow import (
    begin_blast_iteration,
    recover_scan_start_observation,
)
from .blast_navigation_action_profile import BLAST_NAVIGATION_COMMANDS
from .blast_navigation_motion_execution import BlastNavigationMotionExecutor
from .blast_turn_safety import blast_turn_slice_allows_continuation
from .navigation_diagnostics import record_navigation_diagnostic
from .physical_navigation_contract import (
    ADVANCE,
    REVERSE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)


ACTION_COMMANDS = {
    action: BLAST_NAVIGATION_COMMANDS[action]
    for action in (ADVANCE, REVERSE, TURN_LEFT_90, TURN_RIGHT_90)
}

_STARTUP_PERCEPTION_ACTION_SOURCE = "STARTUP_PERCEPTION"

_ROUTE_VALIDATION_ACTION_SOURCE = "ROUTE_VALIDATION"

_STARTUP_SURROUNDINGS_ACTION = "SCAN_SURROUNDINGS"

_STARTUP_HEADING_RESTORATION_TOLERANCE_DEG = 20.0

_SCAN_REFUSAL_CODES = frozenset(("scan_start_clearance_unverified",
                                 "scan_sweep_clearance_lost",
                                 "scan_sweep_observation_unverified"))

def _encoder_anchor_correlated(observation, motion_executor) -> bool:
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

class BlastEpisodeError(RuntimeError):
    """One safely reportable BLAST episode failure."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)

def _dispatch_action(
    adapter,
    action,
    motion_executor,
    control_requested,
    *,
    allow_turn_no_valid_with_bounded_evidence=False,
    action_permit=None,
    surroundings_scan=False,
    turn_continue_requested=None,
    trim_turn=False,
):
    if action == SCAN_FRONT_ARC:
        kwargs = {"cancel_requested": control_requested}
        if action_permit is not None:
            kwargs["action_permit"] = action_permit
        surroundings = getattr(
            adapter.controller, "scan_surroundings", None,
        )
        result = (
            surroundings(**kwargs)
            if surroundings_scan and callable(surroundings)
            else adapter.controller.command("scan_front_arc", **kwargs)
        )
        return result, None
    continuation_gate = None
    if action in (TURN_LEFT_90, TURN_RIGHT_90):
        continuation_gate = (
            turn_continue_requested
            if turn_continue_requested is not None
            else partial(
                blast_turn_slice_allows_continuation,
                allow_no_valid_distance_with_bounded_evidence=(
                    allow_turn_no_valid_with_bounded_evidence
                ),
            )
        )
    execution = motion_executor.execute(
        action,
        cancel_requested=control_requested,
        continue_requested=continuation_gate,
        **({"trim_turn": True} if trim_turn else {}),
    )
    return execution.controller_results[-1], execution

def _scan_failure_outcome(adapter, code):
    if code not in _SCAN_REFUSAL_CODES:
        return None
    sweep_stopped = code != "scan_start_clearance_unverified"
    message = (
        "BLAST scan stopped between pulses; reposition before retry"
        if sweep_stopped
        else "BLAST scan could not start from settled safety evidence"
    )
    return adapter._outcome("no_safe_blast_action", False, message)

def _dispatch_episode_action(
    adapter, *, action, observation, geometry_checked, motion_executor,
    prior_receipt,
    allow_turn_no_valid_with_bounded_evidence, context, deadline_ms,
    map_trace=None, perception_only_scan=False,
    surroundings_scan=False, turn_continue_requested=None,
    scan_refusal_can_replan=False,
    trim_turn=False,
):
    outcome = adapter._control_outcome(
        context, deadline_ms, blast_action_deadline_headroom_ms(action),
    )
    if outcome is not None:
        return None, None, observation, outcome
    control_requested = lambda: adapter._control_outcome(
        context, deadline_ms) is not None
    for attempt in range(2):
        if not _encoder_anchor_correlated(
            observation, motion_executor,
        ):
            raise BlastEpisodeError(
                "blast_action_start_unverified",
                "BLAST drive encoders no longer match its trusted pose",
            )
        action_permit = _scan_action_permit(adapter, action=action,
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
            record_navigation_diagnostic(
                "controller_action_started", episode_id=context.episode_id,
                robot_id=ROBOT_ID, action=action,
                pose=motion_executor.pose.to_dict(),
                observation_before=observation,
            )
            command_result, execution = _dispatch_action(adapter, action,
                motion_executor,
                control_requested,
                allow_turn_no_valid_with_bounded_evidence=(
                    allow_turn_no_valid_with_bounded_evidence
                ),
                action_permit=action_permit,
                surroundings_scan=surroundings_scan,
                turn_continue_requested=turn_continue_requested,
                trim_turn=trim_turn,
            )
            record_navigation_diagnostic(
                "controller_action_completed", episode_id=context.episode_id,
                robot_id=ROBOT_ID, action=action, result=command_result,
            )
            return command_result, execution, observation, None
        except BlastControllerError as error:
            record_navigation_diagnostic(
                "controller_action_failed", episode_id=context.episode_id,
                robot_id=ROBOT_ID, action=action, code=error.code,
                message=str(error), motion_started=error.motion_started,
                evidence_uncertain=error.evidence_uncertain,
                cause=str(error.__cause__) if error.__cause__ else None,
            )
            if (
                action == SCAN_FRONT_ARC
                and error.motion_started is not False
            ):
                motion_executor.invalidate_after_failed_scan()
                if map_trace is not None:
                    map_trace.invalidate_localization()
            outcome = adapter._control_outcome(context, deadline_ms)
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
                outcome = adapter._control_outcome(
                    context,
                    deadline_ms,
                    blast_action_deadline_headroom_ms(action),
                )
                if outcome is not None:
                    return None, None, observation, outcome
                try:
                    retry_observation = (
                        recover_scan_start_observation(
                            adapter,
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
                outcome = adapter._control_outcome(
                    context,
                    deadline_ms,
                    blast_action_deadline_headroom_ms(action),
                )
                if outcome is not None:
                    return None, None, observation, outcome
                observation = retry_observation
                continue
            scan_outcome = _scan_failure_outcome(adapter, error.code)
            if scan_outcome is not None:
                if (
                    scan_refusal_can_replan
                    and error.code
                    == "scan_start_clearance_unverified"
                    and error.motion_started is False
                ):
                    return {
                        "recoverable_scan_refusal": {
                            "code": error.code,
                            "motion_started": False,
                        },
                    }, None, observation, None
                return None, None, observation, scan_outcome
            raise
    raise AssertionError("BLAST action retry loop exhausted")

def _scan_action_permit(
    adapter, *, action, observation, geometry_checked, pose, prior_receipt,
    expected_drive_angles=None, perception_only=False,
):
    try:
        return issue_blast_scan_permit(
            controller=adapter.controller,
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

def _restore_startup_heading(
    adapter, *, scan_item, motion_executor, episode_start_heading,
    map_trace, context, deadline_ms,
):
    """Coarsely return a completed full scan to its IMU start heading."""

    result_observation = scan_item.get("result_observation")
    scan = scan_item.get("scan")
    coverage = (
        scan.get("sweep_coverage_deg")
        if isinstance(scan, Mapping) else None
    )
    if (
        not isinstance(scan, Mapping)
        or scan.get("state") != "complete"
        or isinstance(coverage, bool)
        or not isinstance(coverage, (int, float))
        or float(coverage) < 350.0
    ):
        return result_observation, None, None
    heading_mdeg = motion_executor.pose.heading_mdeg
    tolerance_mdeg = round(
        _STARTUP_HEADING_RESTORATION_TOLERANCE_DEG * 1_000
    )
    correction = {
        "initial_error_mdeg": heading_mdeg,
        "tolerance_mdeg": tolerance_mdeg,
        "action": None,
        "final_error_mdeg": heading_mdeg,
    }
    if abs(heading_mdeg) <= tolerance_mdeg:
        map_trace.record_action(
            SCAN_FRONT_ARC, motion_executor.pose,
            result_observation, None, pose_observed=True,
        )
        return result_observation, None, correction

    observation = adapter._with_navigation_reference(
        adapter._observation(), episode_start_heading,
    )
    observation["odometry"] = motion_executor.pose.to_dict()
    alignment = adapter._desired_heading_turn_alignment(
        0.0, motion_executor.pose,
    )
    if alignment is None:
        return result_observation, None, correction
    action, desired_heading, direction, _error = alignment
    correction["action"] = action
    command_result, execution, _observation, outcome = (
        _dispatch_episode_action(adapter, action=action,
            observation=observation,
            geometry_checked=False,
            motion_executor=motion_executor,
            prior_receipt=scan_item,
            allow_turn_no_valid_with_bounded_evidence=True,
            context=context,
            deadline_ms=deadline_ms,
            map_trace=map_trace,
            turn_continue_requested=(
                adapter._waypoint_alignment_continuation(
                    desired_heading=desired_heading,
                    direction=direction,
                    start_observation=observation,
                    start_heading_mdeg=motion_executor.pose.heading_mdeg,
                    allow_no_valid_distance=True,
                    alignment_trigger_deg=(
                        _STARTUP_HEADING_RESTORATION_TOLERANCE_DEG
                    ),
                )
            ),
        )
    )
    if outcome is not None:
        return result_observation, outcome, correction
    result_observation = command_result.get("observation")
    correction["final_error_mdeg"] = motion_executor.pose.heading_mdeg
    map_trace.record_action(
        action, motion_executor.pose, result_observation, None,
        pose_observed=True,
    )
    context.publish({
        "current_action": None,
        "obstacle": {
            "distance_mm": (
                result_observation.get("distance_mm")
                if isinstance(result_observation, Mapping) else None
            )
        },
    })
    return result_observation, None, correction

def _run_startup_perception(
    adapter, *, observation, available_actions, turns_available,
    latest_scan_view, history, motion_executor, episode_start_heading,
    map_trace, context, deadline_ms,
):
    """Acquire one complete encoder-measured view before Gemma decides."""

    initial_scan_view_count = len(map_trace.planar_scan_views)
    startup_history = []
    action = SCAN_FRONT_ARC
    try:
        observation, outcome = adapter._fresh_planner_observation_or_stop(
            action, episode_start_heading, motion_executor,
            context, deadline_ms,
        )
    except BlastEpisodeError:
        outcome = adapter._outcome(
            "blast_startup_perception_incomplete", False,
            "BLAST could not safely begin its surroundings scan",
        )
    if outcome is not None:
        return (
            observation, available_actions, turns_available,
            latest_scan_view, outcome,
        )
    command_result, execution, observation, outcome = (
        _dispatch_episode_action(adapter, action=action,
            observation=observation,
            geometry_checked=False,
            motion_executor=motion_executor,
            prior_receipt=history[-1] if history else None,
            allow_turn_no_valid_with_bounded_evidence=False,
            context=context,
            deadline_ms=deadline_ms,
            map_trace=map_trace,
            perception_only_scan=True,
            surroundings_scan=True,
        )
    )
    if outcome is not None:
        startup_outcome = (
            outcome if outcome.terminal_reason in (
                "stopped", "episode_deadline_elapsed",
                "episode_deadline_headroom_insufficient",
            ) else adapter._outcome(
                "blast_startup_perception_incomplete", False,
                "BLAST could not safely complete its surroundings scan",
            )
        )
        return (
            observation, available_actions, turns_available,
            latest_scan_view, startup_outcome,
        )
    latest_scan_view = adapter._record_episode_action_result(
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
    startup_scan_item = startup_history[-1]
    outcome = adapter._control_outcome(context, deadline_ms)
    if outcome is not None:
        return (
            observation, available_actions, turns_available,
            latest_scan_view, outcome,
        )
    if (
        latest_scan_view is None
        or not motion_executor.localization_valid
        or len(map_trace.planar_scan_views)
        != initial_scan_view_count + 1
    ):
        return (
            observation, available_actions, turns_available,
            latest_scan_view,
            adapter._outcome(
                "blast_startup_perception_incomplete", False,
                "BLAST startup scan produced no localized surroundings",
            ),
        )
    (
        final_observation, outcome, heading_restoration,
    ) = _restore_startup_heading(adapter, scan_item=startup_scan_item,
        motion_executor=motion_executor,
        episode_start_heading=episode_start_heading,
        map_trace=map_trace,
        context=context,
        deadline_ms=deadline_ms,
    )
    if outcome is not None:
        return (
            observation, available_actions, turns_available,
            latest_scan_view, outcome,
        )
    history.append({
        "action": _STARTUP_SURROUNDINGS_ACTION,
        "action_source": _STARTUP_PERCEPTION_ACTION_SOURCE,
        "assessment": "Mandatory startup surroundings acquisition",
        "plan": [_STARTUP_SURROUNDINGS_ACTION],
        "result_observation": final_observation,
        "observation_settled": startup_scan_item[
            "observation_settled"
        ],
        "pose": motion_executor.pose.to_dict(),
        "scan_view_count": 1,
        "scan_state": startup_scan_item["scan"]["state"],
        "sweep_coverage_deg": startup_scan_item["scan"].get(
            "sweep_coverage_deg"
        ),
        "heading_restoration": heading_restoration,
    })
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
        observation, available_actions, turns_available,
        _runtime, outcome,
    ) = begin_blast_iteration(
        adapter, context=context, deadline_ms=deadline_ms,
        index=1, history=history, latest_scan_view=latest_scan_view,
        motion_executor=motion_executor,
        episode_start_heading=episode_start_heading,
        motion_executor_factory=BlastNavigationMotionExecutor,
        minimum_rotation_clearance_mm=_minimum_rotation_clearance_mm(),
    )
    return (
        observation, available_actions, turns_available,
        latest_scan_view, outcome,
    )
