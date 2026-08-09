"""Thin agent-episode adapter over BLAST's single persistent BLE owner."""

from __future__ import annotations

import copy
import math
import threading
import time
from typing import Callable, Mapping

from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_observation_monitor import (
    CONTROLLER_ID,
    RANGE_STATE_MEASURED,
    ROBOT_ID,
    BlastControllerError,
    blast_range_state,
    validate_blast_scan_ray_contract,
)
from .blast_scan_planar_projection import (
    project_blast_scan_planar_surfaces,
)
from .blast_navigation_action_profile import (
    BLAST_NAVIGATION_ACTION_SPECS,
    BLAST_NAVIGATION_COMMANDS,
)
from .blast_navigation_motion_execution import (
    BlastNavigationMotionExecutor,
)
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
from .physical_odometry import (
    PhysicalPose,
    nominal_effect,
    normalize_heading_mdeg,
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
_SIDE_SEARCH_POSITION_TOLERANCE_MM = 35
_SIDE_SEARCH_HEADING_TOLERANCE_MDEG = 20_000
_SIDE_SEARCH_IMU_ODOMETRY_TOLERANCE_MDEG = 30_000


def _blast_nominal_pose(pose: PhysicalPose, action: str) -> PhysicalPose:
    return nominal_effect(
        pose,
        action,
        BLAST_NAVIGATION_ACTION_SPECS,
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.odometry,
    )[0]


def _side_search_distance(pose: PhysicalPose, waypoint) -> int:
    return int(round(math.hypot(
        waypoint["target_x_mm"] - pose.x_mm,
        waypoint["target_y_mm"] - pose.y_mm,
    )))


def _side_search_waypoint(pose: PhysicalPose, side: str):
    footprint, _sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    )
    stride_mm = (
        footprint.left_extent_mm
        + footprint.right_extent_mm
        + 2 * footprint.clearance_margin_mm
    )
    side_sign = 1 if side == "LEFT" else -1
    heading_radians = math.radians(pose.heading_mdeg / 1_000.0)
    local_left_x = -math.sin(heading_radians)
    local_left_y = math.cos(heading_radians)
    return {
        "kind": "SIDE_SEARCH",
        "selected_side": side,
        "scope": "SEARCH_POSITION_ONLY",
        "clearance_proven": False,
        "frame": "EPISODE_LOCAL_ODOMETRY",
        "origin_pose": pose.to_dict(),
        "target_x_mm": int(round(
            pose.x_mm + side_sign * stride_mm * local_left_x
        )),
        "target_y_mm": int(round(
            pose.y_mm + side_sign * stride_mm * local_left_y
        )),
        "target_heading_mdeg": normalize_heading_mdeg(
            pose.heading_mdeg + side_sign * 90_000
        ),
        "position_tolerance_mm": _SIDE_SEARCH_POSITION_TOLERANCE_MM,
    }


def _side_search_progress(
    pose: PhysicalPose,
    waypoint: Mapping[str, object],
    *,
    reorientation_attempted: bool = False,
):
    """Derive one bounded action for the selected side observation pose."""

    distance = _side_search_distance(pose, waypoint)
    target_heading = waypoint["target_heading_mdeg"]
    heading_error = normalize_heading_mdeg(
        target_heading - pose.heading_mdeg
    )
    required_action = None
    phase = "BLOCKED"
    origin_heading = waypoint["origin_pose"]["heading_mdeg"]
    origin_heading_error = normalize_heading_mdeg(
        origin_heading - pose.heading_mdeg
    )
    if reorientation_attempted:
        if (
            distance <= waypoint["position_tolerance_mm"]
            and abs(origin_heading_error)
            <= _SIDE_SEARCH_HEADING_TOLERANCE_MDEG
        ):
            phase = "RESCAN"
            heading_error = origin_heading_error
            required_action = SCAN_FRONT_ARC
    elif distance > waypoint["position_tolerance_mm"]:
        phase = "OUTBOUND"
        if abs(heading_error) <= _SIDE_SEARCH_HEADING_TOLERANCE_MDEG:
            if _side_search_distance(
                _blast_nominal_pose(pose, ADVANCE), waypoint
            ) < distance:
                required_action = ADVANCE
    else:
        phase = "REORIENT"
        heading_error = origin_heading_error
        action = (
            TURN_RIGHT_90
            if waypoint["selected_side"] == "LEFT"
            else TURN_LEFT_90
        )
        projected = _blast_nominal_pose(pose, action)
        projected_error = normalize_heading_mdeg(
            origin_heading - projected.heading_mdeg
        )
        if (
            abs(projected_error) < abs(heading_error)
            and abs(projected_error) <= _SIDE_SEARCH_HEADING_TOLERANCE_MDEG
        ):
            required_action = action
    return {
        "phase": phase,
        "distance_remaining_mm": distance,
        "heading_error_mdeg": heading_error,
        "required_action": required_action,
    }


def _side_search_heading_correlated(observation, pose: PhysicalPose) -> bool:
    reference = observation.get("navigation_reference")
    raw_error = (
        reference.get("heading_error_deg")
        if isinstance(reference, Mapping)
        else None
    )
    if (
        isinstance(raw_error, bool)
        or not isinstance(raw_error, (int, float))
        or not math.isfinite(float(raw_error))
    ):
        return False
    # Pybricks IMU heading is clockwise-positive; odometry is left-positive.
    imu_heading = normalize_heading_mdeg(-round(float(raw_error) * 1_000))
    error = normalize_heading_mdeg(imu_heading - pose.heading_mdeg)
    return abs(error) <= _SIDE_SEARCH_IMU_ODOMETRY_TOLERANCE_MDEG


def _navigation_body_matched(sensors) -> bool:
    motors = sensors.get("motor_angles_deg")
    angle = motors.get("body") if isinstance(motors, Mapping) else None
    return (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.range_sensor_extrinsics
        .matches_navigation_body_angle(angle)
    )


def _minimum_rotation_clearance_mm() -> int:
    footprint, sensor = (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.require_complete()
    )
    assert sensor.forward_offset_mm is not None
    return max(0, math.ceil(
        footprint.maximum_corner_radius_mm
        + footprint.clearance_margin_mm
        - sensor.forward_offset_mm
    ))


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

    @classmethod
    def _completion_allowed(cls, history) -> bool:
        scan_used = any(
            item.get("action") == SCAN_FRONT_ARC
            for item in history
        )
        return not scan_used or cls._scan_is_current(history)

    def _available_actions(self, observation, history=()) -> tuple[str, ...]:
        available = list(ACTION_COMMANDS)
        distance = observation["sensors"].get("distance_mm")
        distance_is_finite = (
            isinstance(distance, (int, float))
            and not isinstance(distance, bool)
            and math.isfinite(float(distance))
        )
        if (
            self._heading(observation["sensors"]) is not None
            and distance_is_finite
            and not self._scan_is_current(history)
        ):
            available.append(SCAN_FRONT_ARC)
        if distance_is_finite and distance <= self.minimum_forward_clearance_mm:
            available.remove("ADVANCE")
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
        episode_start_heading = None
        motion_executor = None
        selected_detour_side = None
        side_search_waypoint = None
        latest_scan_view = None
        origin_scan_view = None
        reorientation_attempted = False
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
                if _index == 0:
                    episode_start_heading = self._heading(
                        observation["sensors"]
                    )
                    motion_executor = BlastNavigationMotionExecutor(
                        controller=self.controller,
                        initial_observation=observation["sensors"],
                    )
                observation = self._with_navigation_reference(
                    observation,
                    episode_start_heading,
                )
                observation["odometry"] = motion_executor.pose.to_dict()
                side_search_progress = None
                if selected_detour_side is not None:
                    side_search_progress = _side_search_progress(
                        motion_executor.pose,
                        side_search_waypoint,
                        reorientation_attempted=reorientation_attempted,
                    )
                    observation["navigation_intent"] = {
                        "selected_detour_side_relative_to_scan": (
                            selected_detour_side
                        ),
                        "side_search_waypoint": dict(side_search_waypoint),
                    }
                    evidence_correlated = (
                        _side_search_heading_correlated(
                            observation, motion_executor.pose
                        )
                        and _navigation_body_matched(observation["sensors"])
                    )
                available_actions = self._available_actions(
                    observation,
                    history,
                )
                scan_allows_turn = self._current_scan_allows_quarter_turn(
                    history
                ) and latest_scan_view is not None
                if self._scan_is_current(history) and not scan_allows_turn:
                    available_actions = tuple(
                        action for action in available_actions
                        if action not in (TURN_LEFT_90, TURN_RIGHT_90)
                    )
                if side_search_progress is not None:
                    required_action = side_search_progress["required_action"]
                    if not evidence_correlated:
                        required_action = None
                    if required_action == ADVANCE:
                        if (
                            blast_range_state(
                                observation["sensors"].get("distance_mm")
                            ) != RANGE_STATE_MEASURED
                            or ADVANCE not in available_actions
                        ):
                            required_action = None
                    if required_action in (
                        TURN_LEFT_90,
                        TURN_RIGHT_90,
                        SCAN_FRONT_ARC,
                    ) and not self._current_range_allows_rotation(observation):
                        required_action = None
                    if required_action not in available_actions:
                        raise BlastEpisodeError(
                            "blast_side_search_blocked",
                            "BLAST has no verified side-search progress action",
                        )
                    available_actions = (required_action,)
                if side_search_progress is not None:
                    observation["navigation_intent"][
                        "side_search_progress"
                    ] = dict(side_search_progress)
                completion_allowed = self._completion_allowed(history)
                result = planner.decide(ControllerActionContext(
                    goal=context.request.goal,
                    locale=context.request.locale,
                    robot_id=ROBOT_ID,
                    controller_id=CONTROLLER_ID,
                    available_actions=available_actions,
                    observation=observation,
                    history=tuple(history[-12:]),
                    completion_allowed=completion_allowed,
                ))
                if not isinstance(result, ControllerActionPlannerResult):
                    raise BlastEpisodeError(
                        "blast_planner_result_invalid",
                        "BLAST planner returned an invalid result",
                    )
                if self._cancelled(context):
                    return self._outcome("stopped", False, "stopped")
                decision = result.decision
                selects_detour_side = (
                    selected_detour_side is None
                    and self._scan_is_current(history)
                    and scan_allows_turn
                    and decision.action in (TURN_LEFT_90, TURN_RIGHT_90)
                )
                detour_origin_pose = (
                    motion_executor.pose if selects_detour_side else None
                )
                terminal_actions = (
                    (COMPLETE, ABORT)
                    if completion_allowed
                    else (ABORT,)
                )
                if decision.action not in available_actions + terminal_actions:
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
                scan_pose = (
                    motion_executor.pose
                    if decision.action == SCAN_FRONT_ARC
                    else None
                )
                is_side_search_reorientation = (
                    side_search_progress is not None
                    and side_search_progress["phase"] == "REORIENT"
                    and decision.action
                    in (TURN_LEFT_90, TURN_RIGHT_90)
                )
                is_side_search_rescan = (
                    side_search_progress is not None
                    and side_search_progress["phase"] == "RESCAN"
                    and decision.action == SCAN_FRONT_ARC
                )
                if is_side_search_reorientation:
                    reorientation_attempted = True
                try:
                    if decision.action == SCAN_FRONT_ARC:
                        command_result = self.controller.command(
                            "scan_front_arc",
                            cancel_requested=lambda: self._cancelled(context),
                        )
                    else:
                        execution = motion_executor.execute(
                            decision.action,
                            cancel_requested=lambda: self._cancelled(context),
                        )
                        command_result = execution.controller_results[-1]
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
                history_item = {
                    "action": decision.action,
                    "assessment": decision.assessment,
                    "plan": list(decision.plan),
                    "result_observation": result_observation,
                    "observation_settled": command_result.get(
                        "observation_settled"
                    ),
                }
                if decision.action in ACTION_COMMANDS:
                    history_item["motion"] = execution.motion.to_dict()
                    history_item["pose"] = execution.pose.to_dict()
                    if selects_detour_side and execution.motion.complete:
                        selected_detour_side = (
                            "LEFT"
                            if decision.action == TURN_LEFT_90
                            else "RIGHT"
                        )
                        side_search_waypoint = _side_search_waypoint(
                            detour_origin_pose,
                            selected_detour_side,
                        )
                        origin_scan_view = latest_scan_view
                scan = command_result.get("scan")
                planar_projection = None
                if (
                    decision.action == SCAN_FRONT_ARC
                    and not isinstance(scan, Mapping)
                ):
                    raise BlastEpisodeError(
                        "blast_scan_result_invalid",
                        "BLAST returned an invalid scan result",
                    )
                if isinstance(scan, Mapping):
                    latest_scan_view = None
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
                    body_angles = [
                        ray["body_motor_angle_deg"]
                        for ray in scan["rays"]
                    ] + [
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
                        motion_executor.reanchor_after_restored_scan(
                            command_result
                        )
                    )
                    try:
                        planar_projection = (
                            project_blast_scan_planar_surfaces(
                                scan=scan,
                                scan_pose=scan_pose,
                            )
                        )
                    except ValueError:
                        planar_projection = None
                    history_item["scan"] = scan
                    if (
                        planar_projection is not None
                        and command_result.get("observation_settled") is True
                    ):
                        latest_scan_view = {
                            "scan_pose": scan_pose.to_dict(),
                            "scan": copy.deepcopy(scan),
                            "planar_projection": copy.deepcopy(
                                planar_projection
                            ),
                        }
                history.append(history_item)
                update = {
                    "current_action": decision.action,
                    "obstacle": {
                        "distance_mm": (
                            result_observation.get("distance_mm")
                            if isinstance(result_observation, Mapping)
                            else None
                        )
                    },
                }
                if isinstance(scan, Mapping):
                    diagnostic_scan = dict(scan)
                    if planar_projection is not None:
                        diagnostic_scan["planar_projection"] = (
                            planar_projection
                        )
                    update["scan"] = diagnostic_scan
                context.publish(update)
                if is_side_search_rescan:
                    side_scan_view = latest_scan_view
                    if not (
                        origin_scan_view is not None
                        and side_scan_view is not None
                        and side_scan_view is not origin_scan_view
                        and _side_search_heading_correlated(
                            self._with_navigation_reference(
                                {"sensors": result_observation},
                                episode_start_heading,
                            ),
                            motion_executor.pose,
                        )
                        and _navigation_body_matched(result_observation)
                    ):
                        return self._outcome(
                            "side_search_observation_unavailable",
                            False,
                            "Side observation could not be correlated",
                        )
                    origin_pose = origin_scan_view["scan_pose"]
                    side_pose = side_scan_view["scan_pose"]
                    separation = int(round(math.hypot(
                        side_pose["x_mm"] - origin_pose["x_mm"],
                        side_pose["y_mm"] - origin_pose["y_mm"],
                    )))
                    stride = int(round(math.hypot(
                        side_search_waypoint["target_x_mm"]
                        - origin_pose["x_mm"],
                        side_search_waypoint["target_y_mm"]
                        - origin_pose["y_mm"],
                    )))
                    if separation < (
                        stride - _SIDE_SEARCH_POSITION_TOLERANCE_MM
                    ):
                        return self._outcome(
                            "side_search_observation_unavailable",
                            False,
                            "Side observation viewpoints were not distinct",
                        )
                    final_scan = copy.deepcopy(diagnostic_scan)
                    final_scan["multi_view_observations"] = {
                        "schema": "blast-multi-view-scan-observations/v1",
                        "frame": "EPISODE_LOCAL_ODOMETRY",
                        "quality": "PROVISIONAL_YAW_ONLY",
                        "selected_side": selected_detour_side,
                        "viewpoint_separation_mm": separation,
                        "object_association_proven": False,
                        "clearance_proven": False,
                        "passage_proven": False,
                        "route_eligible": False,
                        "views": [origin_scan_view, side_scan_view],
                    }
                    context.publish({"scan": final_scan})
                    return self._outcome(
                        "side_search_observation_collected",
                        False,
                        "Two scan viewpoints collected; passage is not proven",
                    )
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
