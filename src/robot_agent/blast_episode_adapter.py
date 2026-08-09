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
    RANGE_STATE_NO_VALID_DISTANCE,
    ROBOT_ID,
    SETTLED_OBSERVATION_COMMAND,
    BlastControllerError,
    blast_range_state,
    validate_blast_scan_ray_contract,
)
from .blast_scan_planar_projection import (
    project_blast_scan_planar_surfaces,
)
from .blast_spatial_map import BlastSpatialMapBridge
from .blast_navigation_action_profile import (
    BLAST_NAVIGATION_COMMANDS,
)
from .blast_navigation_motion_execution import (
    BlastNavigationMotionExecutor,
)
from .blast_side_search_geometry import (
    POSITION_TOLERANCE_MM as _SIDE_SEARCH_POSITION_TOLERANCE_MM,
    side_search_distance as _side_search_distance,
    side_search_followup_slots as _side_search_followup_slots,
    side_search_progress as _side_search_progress,
    side_search_required_slots as _side_search_required_slots,
    side_search_waypoint as _side_search_waypoint,
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
    normalize_heading_mdeg,
)
from .physical_navigation_mission import DirectionalMission
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
_SIDE_SEARCH_IMU_ODOMETRY_TOLERANCE_MDEG = 30_000
_PLANNER_ACTION_SOURCE = "PLANNER_ACTION"
_HOST_SIDE_SEARCH_ACTION_SOURCE = "HOST_SIDE_SEARCH_ACTION"


def _map_pose(pose: PhysicalPose):
    return {
        "x_mm": pose.x_mm,
        "y_mm": pose.y_mm,
        "heading_mdeg": pose.heading_mdeg,
    }


class _BlastEpisodeMapTrace:
    """Fail-open projection of one episode into the diagnostic map sink."""

    def __init__(
        self,
        *,
        bridge,
        episode_id,
        pose,
        observation,
        observed_at_unix_ms,
        episode_start_heading,
        minimum_forward_progress_mm,
    ):
        self.bridge = bridge
        self.episode_id = episode_id
        self.episode_start_heading = episode_start_heading
        self.mission = DirectionalMission.begin(
            episode_id=episode_id,
            minimum_forward_progress_mm=minimum_forward_progress_mm,
            pose=pose,
        )
        self.planned_leg = None
        self.planar_scan_views = []
        self._offer(
            "begin_episode",
            episode_id=episode_id,
            pose=pose,
            observation=observation,
        )
        self._offer_trace(pose, observation, observed_at_unix_ms)

    def _offer(self, method, **values):
        try:
            return getattr(self.bridge, method)(**values)
        except Exception:
            return False

    def _final_goal(self, pose):
        heading = math.radians(
            self.mission.reference_heading_mdeg / 1_000.0
        )
        distance = self.mission.minimum_forward_progress_mm
        current = self.mission.longitudinal_progress_mm(pose)
        return {
            "kind": "DIRECTIONAL_HEADING",
            "navigation_enforced": False,
            "origin_x_mm": self.mission.origin_x_mm,
            "origin_y_mm": self.mission.origin_y_mm,
            "target_x_mm": int(round(
                self.mission.origin_x_mm + distance * math.cos(heading)
            )),
            "target_y_mm": int(round(
                self.mission.origin_y_mm + distance * math.sin(heading)
            )),
            "desired_heading_mdeg": self.mission.reference_heading_mdeg,
            "minimum_forward_progress_mm": distance,
            "heading_tolerance_mdeg": self.mission.heading_tolerance_mdeg,
            "current_forward_progress_mm": current,
            "remaining_forward_progress_mm": max(0, distance - current),
        }

    def _imu_heading(self, observation, observed_at_unix_ms):
        imu = observation.get("imu") if isinstance(observation, Mapping) else None
        heading = imu.get("heading_deg") if isinstance(imu, Mapping) else None
        if (
            isinstance(heading, bool)
            or not isinstance(heading, (int, float))
            or not math.isfinite(float(heading))
            or isinstance(self.episode_start_heading, bool)
            or not isinstance(self.episode_start_heading, (int, float))
            or not math.isfinite(float(self.episode_start_heading))
            or type(observed_at_unix_ms) is not int
        ):
            return None
        relative = (
            float(heading) - float(self.episode_start_heading) + 180.0
        ) % 360.0 - 180.0
        return {
            "heading_mdeg": normalize_heading_mdeg(
                -round(relative * 1_000)
            ),
            "reference": "EPISODE_START",
            "observed_at_unix_ms": observed_at_unix_ms,
        }

    def _offer_trace(self, pose, observation, observed_at_unix_ms):
        self._offer(
            "offer_trace",
            episode_id=self.episode_id,
            final_goal=self._final_goal(pose),
            planned_leg=self.planned_leg,
            imu_heading=self._imu_heading(
                observation, observed_at_unix_ms
            ),
            planar_scan_views=tuple(self.planar_scan_views),
        )

    def record(
        self,
        *,
        pose,
        observation,
        pose_observed,
        selected_side,
        waypoint,
        bind_pose,
        scan_view,
    ):
        if not isinstance(observation, Mapping):
            return
        observed_at_unix_ms = time.time_ns() // 1_000_000
        if pose_observed:
            self._offer(
                "offer_pose",
                episode_id=self.episode_id,
                pose=pose,
                observation=observation,
            )
        if (
            self.planned_leg is None
            and selected_side in ("LEFT", "RIGHT")
            and isinstance(waypoint, Mapping)
            and isinstance(bind_pose, PhysicalPose)
        ):
            self.planned_leg = {
                "kind": "SIDE_SEARCH",
                "scope": "SEARCH_POSITION_ONLY",
                "clearance_proven": False,
                "passage_proven": False,
                "route_eligible": False,
                "selected_side": selected_side,
                "bind_pose": _map_pose(bind_pose),
                "waypoint": {
                    "x_mm": waypoint["target_x_mm"],
                    "y_mm": waypoint["target_y_mm"],
                    "heading_mdeg": waypoint["target_heading_mdeg"],
                },
            }
        if isinstance(scan_view, Mapping):
            self.planar_scan_views.append({
                "scan_id": "{}-scan-{}".format(
                    self.episode_id,
                    len(self.planar_scan_views) + 1,
                ),
                "observed_at_unix_ms": observed_at_unix_ms,
                "scan_pose": {
                    key: scan_view["scan_pose"][key]
                    for key in ("x_mm", "y_mm", "heading_mdeg")
                },
                "projection": copy.deepcopy(
                    scan_view["planar_projection"]
                ),
            })
        self._offer_trace(pose, observation, observed_at_unix_ms)


def _scan_refusal(reason_is_side_search: bool):
    if reason_is_side_search:
        return (
            "side_search_observation_unavailable",
            "Side observation scan could not start safely",
        )
    return (
        "no_safe_blast_action",
        "BLAST scan could not start from settled safety evidence",
    )


def _detour_side(action: str) -> str:
    return "LEFT" if action == TURN_LEFT_90 else "RIGHT"


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
    return (
        BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
        .minimum_rotation_clearance_mm()
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
        self.spatial_map_provider = spatial_map_bridge
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
        if action in (TURN_LEFT_90, TURN_RIGHT_90, SCAN_FRONT_ARC):
            return (
                self._heading(sensors) is not None
                and self._current_range_allows_rotation(observation)
            )
        return False

    @classmethod
    def _completion_allowed(cls, history) -> bool:
        scan_used = any(
            item.get("action") == SCAN_FRONT_ARC
            for item in history
        )
        return not scan_used or cls._scan_is_current(history)

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
            and not self._scan_is_current(history)
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
                "plan": [],
                "message": message,
            },
        )

    def _fresh_planner_action_observation(
        self,
        *,
        action,
        selects_detour_side,
        episode_start_heading,
        motion_executor,
        cancel_requested,
    ):
        observation = self._with_navigation_reference(
            self._observation(),
            episode_start_heading,
        )
        observation["odometry"] = motion_executor.pose.to_dict()
        rejected_at = observation["observed_at_monotonic_ms"]
        should_remeasure = (
            selects_detour_side
            and action in (TURN_LEFT_90, TURN_RIGHT_90)
            and blast_range_state(
                observation["sensors"].get("distance_mm")
            ) == RANGE_STATE_NO_VALID_DISTANCE
            and _side_search_heading_correlated(
                observation, motion_executor.pose
            )
            and _navigation_body_matched(observation["sensors"])
        )
        if should_remeasure:
            result = self.controller.command(
                SETTLED_OBSERVATION_COMMAND,
                cancel_requested=cancel_requested,
            )
            if cancel_requested():
                raise BlastControllerError(
                    "controller_command_interrupted",
                    "BLAST observation was cancelled",
                )
            observation = self._with_navigation_reference(
                self._observation(), episode_start_heading
            )
            observation["odometry"] = motion_executor.pose.to_dict()
            if not (
                isinstance(result, Mapping)
                and result.get("command") == SETTLED_OBSERVATION_COMMAND
                and result.get("accepted") is True
                and result.get("completed") is True
                and result.get("observation_settled") is True
                and observation["observed_at_monotonic_ms"] > rejected_at
            ):
                raise BlastEpisodeError(
                    "blast_side_search_blocked",
                    "BLAST side selection has no fresh settled range",
                )
        # The scan monitor settles five fresh samples and performs the final
        # range, heading, and sensor-pose gate before its first wheel pulse.
        # Do not let one unsmoothed adapter snapshot preempt that authority.
        if (
            action != SCAN_FRONT_ARC
            and not self._current_observation_allows_action(
                action, observation
            )
        ):
            if selects_detour_side:
                code = "blast_side_search_blocked"
                message = (
                    "BLAST side selection lost current motion safety evidence"
                )
            else:
                code = "blast_action_start_unverified"
                message = "BLAST action lost current motion safety evidence"
            raise BlastEpisodeError(code, message)
        if selects_detour_side and not (
            _side_search_heading_correlated(
                observation,
                motion_executor.pose,
            )
            and _navigation_body_matched(observation["sensors"])
            and self._current_range_allows_rotation(observation)
        ):
            raise BlastEpisodeError(
                "blast_side_search_blocked",
                "BLAST side selection lost current motion safety evidence",
            )
        return observation

    def _prepare_side_search(
        self,
        *,
        origin_pose,
        side,
        scan_view,
        remaining_slots,
    ):
        try:
            waypoint = _side_search_waypoint(
                origin_pose,
                side,
                scan_view=scan_view,
            )
        except ValueError:
            return None, self._outcome(
                "side_search_observation_unavailable",
                False,
                "Origin scan could not size a side search",
            )
        if _side_search_required_slots(waypoint) > remaining_slots:
            return None, self._outcome(
                "side_search_budget_insufficient",
                False,
                "The bounded episode cannot finish this side observation",
            )
        return waypoint, None

    def _fresh_planner_observation_or_stop(
        self, action, selects_detour_side, episode_start_heading,
        motion_executor, context,
    ):
        try:
            return self._fresh_planner_action_observation(
                action=action,
                selects_detour_side=selects_detour_side,
                episode_start_heading=episode_start_heading,
                motion_executor=motion_executor,
                cancel_requested=lambda: self._cancelled(context),
            ), None
        except BlastControllerError as error:
            if (
                error.code == "controller_command_interrupted"
                and self._cancelled(context)
            ):
                return None, self._outcome("stopped", False, "stopped")
            raise

    def _complete_side_search(
        self,
        origin_pose,
        action,
        scan_view,
        outbound_pose,
        remaining_slots,
    ):
        side = _detour_side(action)
        try:
            waypoint = _side_search_waypoint(
                origin_pose,
                side,
                scan_view=scan_view,
                outbound_pose=outbound_pose,
            )
        except ValueError:
            return None, None, self._outcome(
                "side_search_observation_unavailable",
                False,
                "Verified side turn could not bind the search waypoint",
            )
        if _side_search_followup_slots(
            outbound_pose, waypoint
        ) > remaining_slots:
            return None, None, self._outcome(
                "side_search_budget_insufficient",
                False,
                "The bounded episode cannot finish this side observation",
            )
        return side, waypoint, None

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
    def _record_map_result(
        trace, action, pose, observation, selected_side, waypoint, scan_view,
    ):
        trace.record(
            pose=pose,
            observation=observation,
            pose_observed=action in ACTION_COMMANDS,
            selected_side=selected_side,
            waypoint=waypoint,
            bind_pose=pose,
            scan_view=(scan_view if action == SCAN_FRONT_ARC else None),
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
        previous_outbound_distance_mm = None
        host_side_search_actions = []
        try:
            planner = self.planner_factory(context.settings.model)
            if not callable(getattr(planner, "decide", None)):
                raise BlastEpisodeError(
                    "blast_planner_invalid",
                    "BLAST planner is invalid",
                )
            # Preserve the existing hard episode bound: host-owned waypoint
            # actions consume the same semantic action slots as model actions.
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
                    map_trace = self._begin_map_trace(
                        context, motion_executor.pose, observation,
                        episode_start_heading,
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
                elif (
                    selected_detour_side is None
                    and self._scan_is_current(history)
                    and scan_allows_turn
                ):
                    available_actions = tuple(
                        action for action in available_actions
                        if action in (TURN_LEFT_90, TURN_RIGHT_90)
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
                if side_search_progress is None and not available_actions:
                    return self._outcome(
                        "no_safe_blast_action",
                        False,
                        "BLAST has no currently observed safe motion or scan",
                    )
                completion_allowed = self._completion_allowed(history)
                selects_detour_side = False
                if side_search_progress is None:
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
                    if self._cancelled(context):
                        return self._outcome("stopped", False, "stopped")
                    if not isinstance(result, ControllerActionPlannerResult):
                        raise BlastEpisodeError(
                            "blast_planner_result_invalid",
                            "BLAST planner returned an invalid result",
                        )
                    decision = result.decision
                    action = decision.action
                    assessment = decision.assessment
                    plan = list(decision.plan)
                    action_source = _PLANNER_ACTION_SOURCE
                    selects_detour_side = (
                        self._scan_is_current(history)
                        and scan_allows_turn
                        and action in (TURN_LEFT_90, TURN_RIGHT_90)
                    )
                    if selects_detour_side:
                        detour_origin_pose = motion_executor.pose
                        selected_side_candidate = _detour_side(action)
                        _waypoint, blocked = self._prepare_side_search(
                            origin_pose=detour_origin_pose,
                            side=selected_side_candidate,
                            scan_view=latest_scan_view,
                            remaining_slots=self.max_decisions - _index,
                        )
                        if blocked is not None:
                            return blocked
                    terminal_actions = (
                        (COMPLETE, ABORT)
                        if completion_allowed
                        else (ABORT,)
                    )
                    if action not in available_actions + terminal_actions:
                        raise BlastEpisodeError(
                            "blast_planner_action_invalid",
                            "BLAST planner selected an unavailable action",
                        )
                    if action in ACTION_COMMANDS or action == SCAN_FRONT_ARC:
                        observation, stopped = (
                            self._fresh_planner_observation_or_stop(
                                action, selects_detour_side,
                                episode_start_heading, motion_executor, context,
                            )
                        )
                        if stopped is not None:
                            return stopped
                    context.publish({
                        "current_action": (
                            None if action in (COMPLETE, ABORT) else action
                        ),
                        "plan": list(plan),
                        "model_latency_ms": result.latency_ms,
                        "message": decision.utterance or assessment,
                        "obstacle": {
                            "distance_mm": observation["sensors"].get(
                                "distance_mm"
                            ),
                            "observed_at_monotonic_ms": observation[
                                "observed_at_monotonic_ms"
                            ],
                        },
                    })
                    if action == COMPLETE:
                        return self._outcome("completed", True, assessment)
                    if action == ABORT:
                        return self._outcome(
                            "planner_aborted",
                            False,
                            assessment,
                        )
                else:
                    action = side_search_progress["required_action"]
                    assessment = None
                    plan = []
                    action_source = _HOST_SIDE_SEARCH_ACTION_SOURCE
                    if action == ADVANCE:
                        distance = side_search_progress[
                            "distance_remaining_mm"
                        ]
                        if (
                            previous_outbound_distance_mm is not None
                            and distance >= previous_outbound_distance_mm
                        ):
                            raise BlastEpisodeError(
                                "blast_side_search_not_progressing",
                                "BLAST side-search motion did not reduce the "
                                "waypoint distance",
                            )
                        previous_outbound_distance_mm = distance
                    host_side_search_actions.append(action)
                    context.publish({
                        "current_action": action,
                        "plan": [],
                        "model_latency_ms": None,
                        "message": (
                            "Host follows the selected side-search waypoint: "
                            f"{side_search_progress['phase']} / {action}"
                        ),
                        "obstacle": {
                            "distance_mm": observation["sensors"].get(
                                "distance_mm"
                            ),
                            "observed_at_monotonic_ms": observation[
                                "observed_at_monotonic_ms"
                            ],
                        },
                    })
                scan_pose = (
                    motion_executor.pose
                    if action == SCAN_FRONT_ARC
                    else None
                )
                is_side_search_reorientation = (
                    side_search_progress is not None
                    and side_search_progress["phase"] == "REORIENT"
                    and action in (TURN_LEFT_90, TURN_RIGHT_90)
                )
                is_side_search_rescan = (
                    side_search_progress is not None
                    and side_search_progress["phase"] == "RESCAN"
                    and action == SCAN_FRONT_ARC
                )
                if is_side_search_reorientation:
                    reorientation_attempted = True
                try:
                    if action == SCAN_FRONT_ARC:
                        command_result = self.controller.command(
                            "scan_front_arc",
                            cancel_requested=lambda: self._cancelled(context),
                        )
                    else:
                        execution = motion_executor.execute(
                            action,
                            cancel_requested=lambda: self._cancelled(context),
                        )
                        command_result = execution.controller_results[-1]
                except BlastControllerError as error:
                    if (
                        error.code == "controller_command_interrupted"
                        and self._cancelled(context)
                    ):
                        return self._outcome("stopped", False, "stopped")
                    if error.code == "scan_start_clearance_unverified":
                        if self._cancelled(context):
                            return self._outcome(
                                "stopped", False, "stopped"
                            )
                        reason, message = _scan_refusal(
                            is_side_search_rescan
                        )
                        return self._outcome(
                            reason,
                            False,
                            message,
                        )
                    raise
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
                }
                side_search_setup_outcome = None
                if action in ACTION_COMMANDS:
                    history_item["motion"] = execution.motion.to_dict()
                    history_item["pose"] = execution.pose.to_dict()
                    if selects_detour_side and execution.motion.complete:
                        side_search_setup = self._complete_side_search(
                            detour_origin_pose, action, latest_scan_view,
                            execution.pose, self.max_decisions - _index - 1,
                        )
                        (
                            selected_detour_side,
                            side_search_waypoint,
                            side_search_setup_outcome,
                        ) = side_search_setup
                        if side_search_setup_outcome is None:
                            origin_scan_view = latest_scan_view
                scan = command_result.get("scan")
                planar_projection = None
                if (
                    action == SCAN_FRONT_ARC
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
                self._record_map_result(
                    map_trace, action, motion_executor.pose,
                    result_observation, selected_detour_side,
                    side_search_waypoint, latest_scan_view,
                )
                history.append(history_item)
                diagnostic_scan = self._publish_action_result(
                    context, action, result_observation, scan,
                    planar_projection,
                )
                if self._cancelled(context):
                    return self._outcome("stopped", False, "stopped")
                if (
                    (
                        action_source == _HOST_SIDE_SEARCH_ACTION_SOURCE
                        or selects_detour_side
                    )
                    and action in ACTION_COMMANDS
                    and execution.motion.complete is not True
                ):
                    return self._outcome(
                        "side_search_motion_incomplete",
                        False,
                        "Host side-search motion was incomplete",
                    )
                if side_search_setup_outcome is not None:
                    return side_search_setup_outcome
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
                        "strategy_source": _PLANNER_ACTION_SOURCE,
                        "execution_source": _HOST_SIDE_SEARCH_ACTION_SOURCE,
                        "host_action_count": len(host_side_search_actions),
                        "host_action_trace": list(host_side_search_actions),
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
