"""Prepare model context and admit its waypoint plan without selecting a route."""

from __future__ import annotations

import copy
import math
from typing import Mapping
from .blast_action_admission import BlastActionEvidenceChanged
from .episode_speech import episode_cancelled as blast_episode_cancelled
from .blast_observation_monitor import (
    CONTROLLER_ID,
    RANGE_STATE_MEASURED,
    ROBOT_ID,
    blast_range_state,
)
from .blast_scan_observation import current_side_scan
from .lm_studio import LMStudioProtocolError, LMStudioTransportError
from .lm_studio_controller_action import (
    ABORT,
    COMPLETE,
    FOLLOW_WAYPOINT,
    ControllerActionContext,
    ControllerActionPlannerResult,
)
from .navigation_diagnostics import record_navigation_diagnostic
from .route_execution_memory import route_execution_memory, latest_action_feedback
from .physical_navigation_contract import (
    ADVANCE,
    OBSERVE,
    REVERSE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from .blast_episode_execution import (
    ACTION_COMMANDS,
    _ROUTE_VALIDATION_ACTION_SOURCE,
    _STARTUP_SURROUNDINGS_ACTION,
    BlastEpisodeError,
)
from .blast_navigation_calibration import BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
from .blast_navigation_action_profile import (
    DRIVE_ENCODER_DEGREES, SCAN_TURN_ENCODER_DEGREES_PER_PULSE, SCAN_TURN_PULSES_PER_SIDE,
)
from .waypoint_geometry import (
    WAYPOINT_REACHED_RADIUS_MM as _WAYPOINT_REACHED_RADIUS_MM,
    WAYPOINT_ALIGNMENT_TRIGGER_DEG as _WAYPOINT_ALIGNMENT_TRIGGER_DEG,
    consume_reached_waypoints,
    latest_motion_feedback,
    waypoint_motion_hint,
)


BLAST_PLAN_ACTIONS = (
    FOLLOW_WAYPOINT,
    ADVANCE,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    SCAN_FRONT_ARC,
    OBSERVE,
)

_PLANNER_ACTION_SOURCE = "PLANNER_ACTION"
_PLAN_CONTINUATION_ACTION_SOURCE = "PLAN_CONTINUATION"

def _planner_navigation_value(value):
    """Expose coarse episode facts without hardware-scale angle precision."""

    if isinstance(value, Mapping):
        result = {}
        for key, nested in value.items():
            if (
                key == "imu"
                or key == "navigation_reference"
                or key.startswith("imu_")
            ):
                continue
            planner_key = key
            planner_value = _planner_navigation_value(nested)
            if (
                key.endswith("_mdeg")
                and isinstance(nested, (int, float))
                and not isinstance(nested, bool)
            ):
                planner_key = key[:-5] + "_deg"
                planner_value = round(nested / 1_000)
            result[planner_key] = planner_value
        if "motor_angles_deg" in value and "distance_mm" in value:
            result.update(_range_evidence(value["distance_mm"]))
        return result
    if isinstance(value, tuple):
        return tuple(_planner_navigation_value(item) for item in value)
    if isinstance(value, list):
        return [_planner_navigation_value(item) for item in value]
    return value

def _planner_history(history):
    """Project pulse receipts into compact semantic navigation events."""

    retained_keys = (
        "action",
        "requested_action",
        "action_source",
        "action_error",
        "result",
        "action_completed",
        "observation_acquired",
        "motion_outcome",
        "observation_settled",
        "pose",
        "motion",
        "last_motion",
        "odometry_reanchored_after_scan",
        "scan_view_count",
        "scan_state",
        "sweep_coverage_deg",
        "heading_restoration",
        "route_rejection",
        "route_interruption",
        "active_waypoint_geometry_after",
        "waypoint_plan",
    )
    events = []
    for item in history:
        event = {
            key: item[key] for key in retained_keys if key in item
        }
        scan = item.get("scan")
        if isinstance(scan, Mapping):
            event["scan"] = {
                key: scan[key] for key in (
                    "state",
                    "result",
                    "restoration_verified",
                    "sweep_coverage_deg",
                    "all_observations_settled",
                ) if key in scan
            }
        result_observation = item.get("result_observation")
        if isinstance(result_observation, Mapping):
            event.update(_range_evidence(result_observation.get("distance_mm")))
        if (
            item.get("action_source") == _PLAN_CONTINUATION_ACTION_SOURCE
            and events
            and events[-1].get("action") == item.get("action")
            and events[-1].get("action_source") in (
                _PLANNER_ACTION_SOURCE,
                _PLAN_CONTINUATION_ACTION_SOURCE,
            )
        ):
            original_source = events[-1]["action_source"]
            event["action_source"] = original_source
            event["continued"] = True
            events[-1] = event
        elif (
            event.get("route_rejection") is not None
            and events
            and events[-1].get("route_rejection")
            == event["route_rejection"]
            and events[-1].get("waypoint_plan")
            == event.get("waypoint_plan")
            and events[-1].get("pose") == event.get("pose")
        ):
            event["repeat_count"] = events[-1].get(
                "repeat_count", 1,
            ) + 1
            events[-1] = event
        else:
            events.append(event)
    return tuple(events)

def _planner_map_with_route_feedback(local_map_evidence, history):
    """Keep the latest unchanged route refusal beside the current map."""

    if not isinstance(local_map_evidence, Mapping):
        return local_map_evidence
    local_map_evidence = copy.deepcopy(local_map_evidence)
    local_map_evidence["route_execution_memory"] = route_execution_memory(history)
    for event in reversed(_planner_history(history)):
        rejection = event.get("route_rejection")
        if isinstance(rejection, Mapping):
            enriched = copy.deepcopy(local_map_evidence)
            enriched["latest_route_rejection"] = {
                "rejection": copy.deepcopy(rejection),
                "rejected_waypoint_plan": copy.deepcopy(
                    event.get("waypoint_plan", ())
                ),
                "pose_at_rejection": copy.deepcopy(event.get("pose")),
                "repeat_count": event.get("repeat_count", 1),
                "pose_or_evidence_changed": False,
            }
            return enriched
        if event.get("action") in (
            ADVANCE,
            REVERSE,
            TURN_LEFT_90,
            TURN_RIGHT_90,
            SCAN_FRONT_ARC,
            _STARTUP_SURROUNDINGS_ACTION,
        ):
            break
    return local_map_evidence

def _range_evidence(distance):
    state = blast_range_state(distance)
    return {
        "range_state": state,
        "distance_mm": distance if state == RANGE_STATE_MEASURED else None,
    }

def _route_interruption(reason, distance, waypoint):
    evidence = _range_evidence(distance)
    if (
        reason == "FORWARD_CLEARANCE_UNAVAILABLE"
        and evidence["range_state"] != RANGE_STATE_MEASURED
    ):
        reason = "RANGE_MEASUREMENT_UNAVAILABLE"
    return {
        "reason": reason,
        **evidence,
        "waypoint": waypoint,
    }

def _planner_step(
    adapter,
    *,
    planner,
    speech,
    context,
    observation,
    history,
    available_actions,
    completion_allowed,
    turns_available,
    latest_scan_view,
    motion_executor,
    episode_start_heading,
    deadline_ms,
    abort_allowed,
    local_map_evidence,
    active_waypoint,
    active_waypoint_plan,
    waypoint_required,
    previous_decision,
):
    outcome = adapter._control_outcome(context, deadline_ms)
    if outcome is not None: return None, outcome
    try:
        calibration = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry
        scan_step_deg = SCAN_TURN_ENCODER_DEGREES_PER_PULSE * calibration.turn_mdeg_per_opposed_encoder_degree / 1_000
        planner_context = ControllerActionContext(
            goal=context.request.goal,
            locale=context.request.locale,
            robot_id=ROBOT_ID,
            controller_id=CONTROLLER_ID,
            available_actions=available_actions,
            observation=_planner_navigation_value({**observation,
                "last_motion": latest_motion_feedback(history),
                "last_action": latest_action_feedback(history)}),
            execution={
                "advance_step_mm": round(calibration.linear_mm_per_encoder_degree * DRIVE_ENCODER_DEGREES),
                "reverse_step_mm": round(calibration.linear_mm_per_encoder_degree * DRIVE_ENCODER_DEGREES),
                "follow_waypoint_max_forward_pulses": 1,
                "rear_range_sensor_available": False,
                "forward_range_stop_mm": adapter.minimum_forward_clearance_mm,
                "scan_front_arc": {"nominal_relative_bearings_deg": [0] + [round(sign * i * scan_step_deg, 2)
                                       for sign in (1, -1) for i in range(1, SCAN_TURN_PULSES_PER_SIDE + 1)],
                                   "motion": "in_place_body_rotation", "returns_to_start_heading": True},
                "next_waypoint_motion": waypoint_motion_hint(
                    motion_executor.pose, active_waypoint,
                    footprint=BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.robot_footprint)},
            history=_planner_navigation_value(
                _planner_history(history)[-12:]
            ),
            completion_allowed=completion_allowed,
            abort_allowed=abort_allowed,
            robot_relative_side_scan=_planner_navigation_value(
                current_side_scan(history, latest_scan_view)
            ),
            local_map_evidence=_planner_navigation_value(
                local_map_evidence
            ),
            active_waypoint=active_waypoint,
            active_waypoint_geometry=(
                _planner_navigation_value(
                    adapter._active_waypoint_geometry(
                        motion_executor.pose, active_waypoint,
                    )
                )
            ),
            active_waypoint_plan=active_waypoint_plan,
            waypoint_reached_radius_mm=_WAYPOINT_REACHED_RADIUS_MM,
            waypoint_required=waypoint_required,
            plan_actions=BLAST_PLAN_ACTIONS,
            previous_decision=previous_decision,
        )
        # Provider trouble is not a navigation event or a consumed decision.
        # Keep this context until a usable reply, cancellation or the deadline.
        while True:
            outcome = adapter._control_outcome(context, deadline_ms)
            if outcome is not None:
                return None, outcome
            try:
                result = planner.decide(planner_context)
                break
            except (LMStudioProtocolError, LMStudioTransportError) as error:
                outcome = adapter._control_outcome(context, deadline_ms)
                if outcome is not None:
                    return None, outcome
                record_navigation_diagnostic(
                    "planner_reply_retry", robot_id=ROBOT_ID,
                    episode_id=context.episode_id,
                    error_type=type(error).__name__,
                )
                context.publish({"message": "Waiting for the planner; keeping the current goal and route."})
                context.stop_requested.wait(1.0)
    except Exception:
        outcome = adapter._control_outcome(context, deadline_ms)
        if outcome is not None:
            return None, outcome
        raise
    outcome = adapter._control_outcome(context, deadline_ms)
    if outcome is not None:
        return None, outcome
    if not isinstance(result, ControllerActionPlannerResult):
        raise BlastEpisodeError(
            "blast_planner_result_invalid",
            "BLAST planner returned an invalid result",
        )
    decision = result.decision
    action = decision.action
    bounded_turn = (
        turns_available
        and action in (TURN_LEFT_90, TURN_RIGHT_90)
    )
    bounded_no_valid = bounded_turn
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
        observation, stopped = adapter._fresh_planner_observation_or_stop(
            action, episode_start_heading,
            motion_executor, context, deadline_ms,
            allow_no_valid_with_bounded_evidence=bounded_no_valid,
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
    outcome = adapter._control_outcome(context, deadline_ms)
    if outcome is not None:
        return None, outcome
    if action == COMPLETE:
        # Every iteration reaches the planner at rest. The existing gesture
        # executor checks idle again; no wheel action follows this finale.
        speech.offer(
            decision.utterance, progress_revision=len(history) + 1,
            expression=decision.expression,
        )
        speech.close(drain=True)
        if blast_episode_cancelled(context):
            return None, adapter._control_outcome(context, deadline_ms)
        return None, adapter._outcome("completed", True, assessment)
    if action == ABORT:
        return None, adapter._outcome(
            "planner_aborted", False, assessment,
        )
    waypoint_plan = (active_waypoint_plan if decision.waypoint is None else
                     (decision.waypoint, *decision.following_waypoints))
    return {
        "action": action,
        "decision_memory": decision.memory(),
        "assessment": assessment,
        "utterance": decision.utterance,
        "plan": list(decision.plan),
        "action_source": _PLANNER_ACTION_SOURCE,
        "observation": observation,
        "bounded_no_valid_eligible": bounded_no_valid,
        "active_waypoint": waypoint_plan[0] if waypoint_plan else None,
        "waypoint_plan": waypoint_plan,
    }, None


def _admit_waypoint_step(
    adapter, *, step, map_trace, motion_executor, history, context,
    available_actions, turns_available, latest_scan_view,
    episode_start_heading, deadline_ms,
):
    """Check the model's route and prepare its next existing motion action."""
    requested_action = step["action"]
    action = requested_action
    plan = step["plan"]
    observation = step["observation"]
    waypoint_plan = adapter._intermediate_waypoint_plan(
        map_trace.mission,
        motion_executor.pose,
        step["waypoint_plan"],
    )
    waypoint_plan, reached_waypoints = consume_reached_waypoints(
        motion_executor.pose, waypoint_plan,
    )
    if (
        requested_action == FOLLOW_WAYPOINT
        and reached_waypoints
        and not waypoint_plan
    ):
        rejected_waypoint = reached_waypoints[-1]
        distance_mm = round(math.hypot(
            rejected_waypoint["x_mm"] - motion_executor.pose.x_mm,
            rejected_waypoint["y_mm"] - motion_executor.pose.y_mm,
        ))
        history.append({
            "action": FOLLOW_WAYPOINT,
            "requested_action": FOLLOW_WAYPOINT,
            "action_source": _ROUTE_VALIDATION_ACTION_SOURCE,
            "route_rejection": {
                "reason": "WAYPOINT_ALREADY_REACHED",
                "distance_mm": distance_mm,
                "reached_radius_mm": _WAYPOINT_REACHED_RADIUS_MM,
                "waypoint": rejected_waypoint,
            },
            "waypoint_plan": tuple(reached_waypoints),
            "pose": motion_executor.pose.to_dict(),
        })
        active_waypoint = None
        map_trace.set_advisory_waypoint_plan(
            (),
            pose=motion_executor.pose,
            observation=observation["sensors"],
            observed_at_unix_ms=(
                observation["observed_at_unix_ms"]
            ),
        )
        context.publish({
            "current_action": None,
            "plan": list(plan),
            "message": (
                "Gemma waypoint is already reached; replanning"
            ),
        })
        return waypoint_plan, True, None
    active_waypoint = (
        waypoint_plan[0] if waypoint_plan else None
    )
    step["active_waypoint"] = active_waypoint
    step["waypoint_plan"] = waypoint_plan
    map_trace.set_advisory_waypoint_plan(
        waypoint_plan,
        pose=motion_executor.pose,
        observation=observation["sensors"],
        observed_at_unix_ms=observation["observed_at_unix_ms"],
    )
    route_blockage = map_trace.advisory_route_blockage(
        motion_executor.pose,
    )
    if requested_action == FOLLOW_WAYPOINT:
        if route_blockage is not None:
            route_blockage = dict(route_blockage)
        if route_blockage is not None:
            history.append({
                "action": FOLLOW_WAYPOINT,
                "requested_action": FOLLOW_WAYPOINT,
                "action_source": _ROUTE_VALIDATION_ACTION_SOURCE,
                "route_rejection": route_blockage,
                "waypoint_plan": waypoint_plan,
                "pose": motion_executor.pose.to_dict(),
            })
            # Stop physical execution, but keep Gemma's complete
            # rejected hypothesis in context and on the map until
            # the model explicitly revises or replaces it.
            context.publish({
                "current_action": None,
                "plan": list(plan),
                "message": (
                    "Gemma route crosses a known coarse "
                    "keep-out cell; replanning"
                ),
            })
            return waypoint_plan, True, None
        action = adapter._waypoint_follow_motion_action(
            motion_executor.pose,
            active_waypoint,
            available_actions,
        )
        if action is None:
            geometry = adapter._active_waypoint_geometry(
                motion_executor.pose, active_waypoint,
            )
            if geometry is not None:
                history.append({
                    "action": FOLLOW_WAYPOINT,
                    "requested_action": FOLLOW_WAYPOINT,
                    "action_source": (
                        _ROUTE_VALIDATION_ACTION_SOURCE
                    ),
                    "route_interruption": _route_interruption(
                        (
                            "REQUIRED_STEERING_UNAVAILABLE"
                            if abs(
                                geometry[
                                    "heading_error_mdeg"
                                ]
                            ) >= round(
                                _WAYPOINT_ALIGNMENT_TRIGGER_DEG
                                * 1_000
                            )
                            else (
                                "FORWARD_CLEARANCE_UNAVAILABLE"
                            )
                        ),
                        observation[
                            "sensors"
                        ].get("distance_mm"),
                        active_waypoint,
                    ),
                    "pose": motion_executor.pose.to_dict(),
                })
            context.publish({
                "current_action": None,
                "plan": list(plan),
            })
            return waypoint_plan, True, None
        bounded_no_valid = (
            action in (TURN_LEFT_90, TURN_RIGHT_90)
            and turns_available
        )
        try:
            observation, outcome = (
                adapter._fresh_planner_observation_or_stop(
                    action,
                    episode_start_heading,
                    motion_executor,
                    context,
                    deadline_ms,
                    allow_no_valid_with_bounded_evidence=(
                        bounded_no_valid
                    ),
                )
            )
        except BlastActionEvidenceChanged:
            context.publish({
                "current_action": None,
                "plan": list(plan),
            })
            return waypoint_plan, True, None
        if outcome is not None:
            return waypoint_plan, False, outcome
        step["action"] = action
        step["observation"] = observation
        step["bounded_no_valid_eligible"] = bounded_no_valid
    return waypoint_plan, False, None
