"""Thin agent-episode adapter over BLAST's single persistent BLE owner."""

from __future__ import annotations

import copy
from functools import partial
import math
import threading
import time
from typing import Callable, Mapping

from .blast_navigation_calibration import (
    BLAST_PROVISIONAL_NAVIGATION_CALIBRATION,
)
from .blast_detour_route import (
    bind_blast_detour_route,
    blast_detour_required_slots,
)
from .blast_detour_runtime import (
    BlastDetourRuntimeBlocked,
    BlastNoReturnScanPermitUnavailable,
    blast_detour_scan_no_return_allows_progress,
    blast_detour_scan_verified,
    blast_local_detour_step,
    issue_blast_no_return_scan_permit,
)
from .blast_episode_deadline import (
    SETTLED_OBSERVATION_HEADROOM_MS, BlastEpisodeDeadline,
    blast_action_deadline_headroom_ms)
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
from .blast_side_observation import (
    build_blast_multi_view_observation, finish_target_reacquisition,
    plan_target_reacquisition, side_search_action_admission,
    side_search_planned_leg)
from .blast_spatial_map import BlastSpatialMapBridge
from .blast_navigation_action_profile import (
    BLAST_NAVIGATION_COMMANDS,
)
from .blast_navigation_motion_execution import (
    BlastNavigationMotionExecutor,
)
from .blast_navigation_state import (
    LocalDetourNavigationState,
    PlannerNavigationState,
    SideSearchNavigationState,
)
from .blast_turn_safety import blast_turn_slice_allows_continuation
from .blast_side_search_geometry import (
    side_search_distance as _side_search_distance,
    side_search_followup_slots as _side_search_followup_slots,
    side_search_progress as _side_search_progress,
    side_search_required_slots as _side_search_required_slots,
    side_search_scan_sweep_is_clear as _side_search_scan_sweep_is_clear,
    side_search_waypoint as _side_search_waypoint,
)
from .lm_studio_controller_action import (
    ABORT,
    COMPLETE,
    ControllerActionContext,
    ControllerActionPlannerResult,
)
from .local_detour_route import ROUTE_COMPLETE
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
_HOST_LOCAL_DETOUR_ACTION_SOURCE = "HOST_LOCAL_DETOUR_ACTION"
_SCAN_REFUSAL_CODES = frozenset(("scan_start_clearance_unverified",
                                 "scan_sweep_clearance_lost",
                                 "scan_sweep_observation_unverified"))


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
        self.navigation_enforced = False
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
            "navigation_enforced": self.navigation_enforced,
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
        route=None,
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
        next_leg = side_search_planned_leg(
            selected_side, waypoint, bind_pose)
        if next_leg is not None and (
            self.planned_leg is None
            or self.planned_leg.get("waypoint") != next_leg["waypoint"]
        ):
            self.planned_leg = next_leg
        if route is not None:
            self.navigation_enforced = True
            if route.status == ROUTE_COMPLETE:
                self.planned_leg = None
            else:
                active = route.active_waypoint
                self.planned_leg = {
                    "kind": active.kind,
                    "scope": "LOCAL_DETOUR_ROUTE",
                    "clearance_proven": False,
                    "passage_proven": False,
                    "route_eligible": True,
                    "selected_side": (
                        "LEFT"
                        if route.detour_side == "LEFT_OF_GOAL"
                        else "RIGHT"
                    ),
                    "bind_pose": _map_pose(pose),
                    "waypoint": {
                        "x_mm": active.x_mm,
                        "y_mm": active.y_mm,
                        "heading_mdeg": active.heading_mdeg,
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
        execute_provisional_detour: bool = False,
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
            or type(execute_provisional_detour) is not bool
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
        self.execute_provisional_detour = execute_provisional_detour
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

    def _control_outcome(self, context, deadline, headroom_ms=0):
        value = deadline.outcome(
            cancelled=self._cancelled(context),
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

    def _scan_failure_outcome(
        self, code, *, side_search_rescan, detour_verification,
    ):
        if code not in _SCAN_REFUSAL_CODES:
            return None
        sweep_stopped = code != "scan_start_clearance_unverified"
        if detour_verification:
            reason = "detour_verification_unavailable"
            message = (
                "BLAST detour verification scan stopped between pulses; "
                "reposition before retry"
                if sweep_stopped
                else "BLAST detour verification scan could not start safely"
            )
        else:
            reason, message = _scan_refusal(side_search_rescan)
            if sweep_stopped:
                message = (
                    "BLAST scan stopped between pulses; reposition before retry"
                )
        return self._outcome(reason, False, message)

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
        motion_executor, context, deadline_ms,
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

    def _bind_local_detour(
        self,
        *,
        origin_view,
        side_view,
        selected_side,
        side_waypoint,
        mission,
        pose,
        remaining_slots,
    ):
        try:
            route = bind_blast_detour_route(
                origin_view=origin_view,
                side_view=side_view,
                selected_side=selected_side,
                side_waypoint=side_waypoint,
                mission=mission,
                current_pose=pose,
            )
            required = blast_detour_required_slots(
                route, pose, mission,
            )
        except ValueError:
            return None, self._outcome(
                "detour_route_unavailable",
                False,
                "The two BLAST views could not bind a local detour route",
            )
        if required > remaining_slots:
            return None, self._outcome(
                "detour_budget_insufficient",
                False,
                "The bounded episode cannot complete the local detour",
            )
        return route, None

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
        remaining_slots,
        deadline_ms,
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
        selects_side = (
            self._scan_is_current(history)
            and scan_allows_turn
            and action in (TURN_LEFT_90, TURN_RIGHT_90)
        )
        detour_origin_pose = None
        if selects_side:
            detour_origin_pose = motion_executor.pose
            _waypoint, blocked = self._prepare_side_search(
                origin_pose=detour_origin_pose,
                side=_detour_side(action),
                scan_view=latest_scan_view,
                remaining_slots=remaining_slots,
            )
            if blocked is not None:
                return None, blocked
        terminal_actions = (
            (COMPLETE, ABORT) if completion_allowed else (ABORT,)
        )
        if action not in available_actions + terminal_actions:
            raise BlastEpisodeError(
                "blast_planner_action_invalid",
                "BLAST planner selected an unavailable action",
            )
        if action in ACTION_COMMANDS or action == SCAN_FRONT_ARC:
            observation, stopped = self._fresh_planner_observation_or_stop(
                action, selects_side, episode_start_heading,
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
            "plan": list(decision.plan),
            "action_source": _PLANNER_ACTION_SOURCE,
            "observation": observation,
            "selects_detour_side": selects_side,
            "bounded_turn_no_valid_eligible": selects_side,
            "detour_origin_pose": detour_origin_pose,
        }, None

    def _host_step(
        self,
        *,
        context,
        observation,
        available_actions,
        side_search_progress,
        detour_guidance,
        detour_scan_role,
        navigation_state,
    ):
        if detour_guidance is not None:
            action = available_actions[0]
            source = _HOST_LOCAL_DETOUR_ACTION_SOURCE
            phase = (
                detour_scan_role
                or detour_guidance.active_waypoint_kind
                or ROUTE_COMPLETE
            )
            message = f"Host follows the local-detour route: {phase} / {action}"
        else:
            if not isinstance(navigation_state, SideSearchNavigationState):
                raise RuntimeError("BLAST side-search ownership is invalid")
            action = side_search_progress["required_action"]
            source = _HOST_SIDE_SEARCH_ACTION_SOURCE
            distance = None
            if action == ADVANCE:
                distance = side_search_progress["distance_remaining_mm"]
                if (
                    navigation_state.previous_outbound_distance_mm is not None
                    and distance >= (
                        navigation_state.previous_outbound_distance_mm
                    )
                ):
                    raise BlastEpisodeError(
                        "blast_side_search_not_progressing",
                        "BLAST side-search motion did not reduce the waypoint "
                        "distance",
                    )
            navigation_state = navigation_state.record_host_action(
                action,
                outbound_distance_mm=distance,
            )
            message = (
                "Host follows the selected side-search waypoint: "
                f"{side_search_progress['phase']} / {action}"
            )
        context.publish({
            "current_action": action,
            "plan": [],
            "model_latency_ms": None,
            "message": message,
            "obstacle": {
                "distance_mm": observation["sensors"].get("distance_mm"),
                "observed_at_monotonic_ms": observation[
                    "observed_at_monotonic_ms"
                ],
            },
        })
        return {
            "action": action,
            "assessment": None,
            "plan": [],
            "action_source": source,
            "observation": observation,
            "selects_detour_side": False,
            "bounded_turn_no_valid_eligible": True,
            "detour_origin_pose": None,
        }, navigation_state

    def _finish_side_rescan(
        self,
        *,
        origin_view,
        side_view,
        selected_side,
        waypoint,
        pose,
        result_observation,
        episode_start_heading,
        diagnostic_scan,
        host_actions,
        mission,
        remaining_slots,
    ):
        correlated = (
            origin_view is not None
            and side_view is not None
            and side_view is not origin_view
            and _side_search_heading_correlated(
                self._with_navigation_reference(
                    {"sensors": result_observation}, episode_start_heading,
                ),
                pose,
            )
            and _navigation_body_matched(result_observation)
        )
        if not correlated:
            return None, None, self._outcome(
                "side_search_observation_unavailable",
                False,
                "Side observation could not be correlated",
            )
        try:
            final_scan = build_blast_multi_view_observation(
                origin_view=origin_view,
                side_view=side_view,
                selected_side=selected_side,
                waypoint=waypoint,
                pose=pose,
                diagnostic_scan=diagnostic_scan,
                host_actions=host_actions,
            )
        except ValueError:
            return None, None, self._outcome(
                "side_search_observation_unavailable",
                False,
                "Side observation viewpoints were not distinct",
            )
        terminal = finish_target_reacquisition(
            final_scan, origin_view, side_view, selected_side, waypoint)
        if terminal is not None:
            return final_scan, None, self._outcome(
                terminal, False,
                "BLAST completed one bounded target reacquisition view",
            )
        if not self.execute_provisional_detour:
            return final_scan, None, self._outcome(
                "side_search_observation_collected",
                False,
                "Two scan viewpoints collected; passage is not proven",
            )
        route, blocked = self._bind_local_detour(
            origin_view=origin_view,
            side_view=side_view,
            selected_side=selected_side,
            side_waypoint=waypoint,
            mission=mission,
            pose=pose,
            remaining_slots=remaining_slots,
        )
        if blocked is not None:
            if blocked.terminal_reason == "detour_route_unavailable":
                next_waypoint, budget_blocked = plan_target_reacquisition(
                    final_scan, origin_view, side_view, selected_side, pose,
                    remaining_slots)
                if budget_blocked:
                    return final_scan, None, self._outcome(
                        "target_reacquisition_budget_insufficient", False,
                        "The episode cannot finish target reacquisition",
                    )
                if next_waypoint is not None:
                    return final_scan, next_waypoint, None
            return final_scan, None, blocked
        final_scan["multi_view_observations"]["route_eligible"] = True
        return final_scan, route, None

    def _local_detour_runtime_step(
        self, *, route, pose, observation, available_actions,
        pass_scan_complete, mission, prior_receipt,
    ):
        try:
            return blast_local_detour_step(
                route=route,
                pose=pose,
                distance_mm=observation["sensors"].get("distance_mm"),
                available_actions=available_actions,
                pass_scan_complete=pass_scan_complete,
                mission=mission,
                prior_receipt=prior_receipt,
                rotation_allowed=self._current_range_allows_rotation(
                    observation
                ),
                evidence_correlated=(
                    _side_search_heading_correlated(observation, pose)
                    and _navigation_body_matched(observation["sensors"])
                ),
            )
        except BlastDetourRuntimeBlocked as error:
            raise BlastEpisodeError(error.code, str(error)) from None

    def _no_return_scan_action_permit(
        self, *, action, action_source, observation, geometry_checked,
        route, pose, prior_receipt,
    ):
        try:
            return issue_blast_no_return_scan_permit(
                controller=self.controller,
                action=action,
                host_side_scan=(
                    action_source == _HOST_SIDE_SEARCH_ACTION_SOURCE
                ),
                host_detour_scan=(
                    action_source == _HOST_LOCAL_DETOUR_ACTION_SOURCE
                ),
                distance_mm=observation["sensors"].get("distance_mm"),
                side_search_geometry_checked=geometry_checked,
                route=route,
                pose=pose,
                prior_receipt=prior_receipt,
            )
        except BlastNoReturnScanPermitUnavailable as error:
            raise BlastEpisodeError(error.code, str(error)) from None

    def _finish_detour_scan(
        self, *, context, deadline, is_pass_scan, scan_view, selected_side,
        result_observation, observation_settled, episode_start_heading, pose,
        route, navigation_state,
    ):
        scan_role = "PASS" if is_pass_scan else "FINAL"
        restored = self._with_navigation_reference(
            {"sensors": result_observation}, episode_start_heading,
        )
        body_matched = _navigation_body_matched(result_observation)
        heading_correlated = _side_search_heading_correlated(restored, pose)
        verified = self._detour_scan_verified(
            scan_view=scan_view,
            role=scan_role,
            selected_side=selected_side,
            result_observation=result_observation,
            route=route,
            navigation_body_matched=body_matched,
            heading_correlated=heading_correlated,
        )
        no_return_progress = blast_detour_scan_no_return_allows_progress(
            scan_view=scan_view,
            role=scan_role,
            selected_side=selected_side,
            result_observation=result_observation,
            observation_settled=observation_settled,
            minimum_forward_clearance_mm=self.minimum_forward_clearance_mm,
            route=route,
            navigation_body_matched=body_matched,
            heading_correlated=heading_correlated,
        )
        outcome = self._control_outcome(context, deadline)
        if outcome is not None:
            return navigation_state, outcome
        if not (verified or no_return_progress):
            return navigation_state, self._outcome(
                "detour_verification_unavailable",
                False,
                "BLAST detour verification scan was unavailable",
            )
        if is_pass_scan:
            return navigation_state.mark_pass_scan_complete(), None
        return navigation_state, self._outcome(
            "completed",
            True,
            "BLAST completed the host-owned local detour",
        )

    def _detour_scan_verified(
        self, *, scan_view, role, selected_side, result_observation,
        route, navigation_body_matched, heading_correlated,
    ):
        return blast_detour_scan_verified(
            scan_view=scan_view, role=role, selected_side=selected_side,
            result_observation=result_observation,
            minimum_forward_clearance_mm=self.minimum_forward_clearance_mm,
            route=route,
            navigation_body_matched=navigation_body_matched,
            heading_correlated=heading_correlated,
        )

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
        route=None, pose_observed=None,
    ):
        trace.record(
            pose=pose,
            observation=observation,
            pose_observed=(action in ACTION_COMMANDS if pose_observed is None
                           else pose_observed),
            selected_side=selected_side,
            waypoint=waypoint,
            bind_pose=pose,
            scan_view=(scan_view if action == SCAN_FRONT_ARC else None),
            route=route,
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
        history, episode_start_heading, motion_executor = [], None, None
        navigation_state = PlannerNavigationState()
        latest_scan_view = None
        try:
            deadline_ms = BlastEpisodeDeadline.begin(
                context.settings, self.monotonic_ms)
            planner = self.planner_factory(context.settings.model)
            if not callable(getattr(planner, "decide", None)):
                raise BlastEpisodeError(
                    "blast_planner_invalid",
                    "BLAST planner is invalid",
                )
            for _index in range(self.max_decisions):
                outcome = self._control_outcome(context, deadline_ms)
                if outcome is not None:
                    return outcome
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
                side_search_state = navigation_state if isinstance(
                    navigation_state, SideSearchNavigationState) else None
                detour_state = navigation_state if isinstance(
                    navigation_state, LocalDetourNavigationState) else None
                selected_detour_side = getattr(
                    navigation_state, "selected_side", None)
                side_search_waypoint = getattr(
                    navigation_state, "waypoint", None)
                local_detour_route = getattr(
                    navigation_state, "route", None)
                side_search_progress = None
                if side_search_state is not None:
                    side_search_progress = _side_search_progress(
                        motion_executor.pose,
                        side_search_waypoint,
                        reorientation_attempted=(
                            side_search_state.reorientation_attempted
                        ),
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
                no_return_scan_geometry_checked = False
                if side_search_progress is not None:
                    no_return_scan_geometry_checked = (
                        side_search_progress["phase"] == "RESCAN"
                        and _side_search_scan_sweep_is_clear(
                            navigation_state.origin_scan_view,
                            motion_executor.pose,
                        )
                    )
                    available_actions, blocked = side_search_action_admission(
                        side_search_progress, side_search_waypoint,
                        observation["sensors"], available_actions,
                        evidence_correlated,
                        self._current_range_allows_rotation(observation),
                        current_pose=motion_executor.pose,
                        prior_receipt=(history[-1] if history else None),
                        no_return_scan_geometry_checked=(
                            no_return_scan_geometry_checked
                        ),
                    )
                    if blocked == "target_reacquisition_blocked":
                        return self._outcome(blocked, False,
                            "BLAST cannot safely reach the next target view")
                    if blocked is not None:
                        raise BlastEpisodeError(
                            "blast_side_search_blocked",
                            "BLAST has no verified side-search progress action",
                        )
                if side_search_progress is not None:
                    observation["navigation_intent"][
                        "side_search_progress"
                    ] = dict(side_search_progress)
                detour_guidance = None
                detour_scan_role = None
                if local_detour_route is not None:
                    (
                        local_detour_route,
                        detour_guidance,
                        required_action,
                        detour_scan_role,
                    ) = self._local_detour_runtime_step(
                        route=local_detour_route,
                        pose=motion_executor.pose,
                        observation=observation,
                        available_actions=available_actions,
                        pass_scan_complete=detour_state.pass_scan_complete,
                        mission=map_trace.mission,
                        prior_receipt=(history[-1] if history else None),
                    )
                    navigation_state = detour_state.with_route(
                        local_detour_route,
                    )
                    detour_state = navigation_state
                    available_actions = (required_action,)
                    observation["navigation_intent"] = {
                        "selected_detour_side_relative_to_scan": (
                            selected_detour_side
                        ),
                        "local_detour_route": local_detour_route.to_dict(),
                        "local_detour_guidance": detour_guidance.to_dict(),
                    }
                if (
                    side_search_progress is None
                    and detour_guidance is None
                    and not available_actions
                ):
                    return self._outcome(
                        "no_safe_blast_action",
                        False,
                        "BLAST has no currently observed safe motion or scan",
                    )
                completion_allowed = self._completion_allowed(history)
                if side_search_progress is None and detour_guidance is None:
                    step, outcome = self._planner_step(
                        planner=planner,
                        context=context,
                        observation=observation,
                        history=history,
                        available_actions=available_actions,
                        completion_allowed=completion_allowed,
                        scan_allows_turn=scan_allows_turn,
                        latest_scan_view=latest_scan_view,
                        motion_executor=motion_executor,
                        episode_start_heading=episode_start_heading,
                        remaining_slots=self.max_decisions - _index,
                        deadline_ms=deadline_ms,
                    )
                    if outcome is not None:
                        return outcome
                else:
                    step, navigation_state = self._host_step(
                        context=context,
                        observation=observation,
                        available_actions=available_actions,
                        side_search_progress=side_search_progress,
                        detour_guidance=detour_guidance,
                        detour_scan_role=detour_scan_role,
                        navigation_state=navigation_state,
                    )
                action = step["action"]
                assessment = step["assessment"]
                plan = step["plan"]
                action_source = step["action_source"]
                observation = step["observation"]
                selects_detour_side = step["selects_detour_side"]
                if step["detour_origin_pose"] is not None:
                    detour_origin_pose = step["detour_origin_pose"]
                scan_pose = (
                    motion_executor.pose if action == SCAN_FRONT_ARC else None
                )
                is_side_search_reorientation = side_search_progress is not None and (
                    side_search_progress["phase"] == "REORIENT"
                    and action in (TURN_LEFT_90, TURN_RIGHT_90))
                is_side_search_rescan = side_search_progress is not None and (
                    side_search_progress["phase"] == "RESCAN"
                    and action == SCAN_FRONT_ARC)
                is_detour_pass_scan = detour_guidance is not None and (
                    detour_scan_role == "PASS" and action == SCAN_FRONT_ARC)
                is_detour_final_scan = detour_guidance is not None and (
                    detour_scan_role == "FINAL" and action == SCAN_FRONT_ARC)
                if is_side_search_reorientation:
                    navigation_state = navigation_state.mark_reorientation_attempted()
                outcome = self._control_outcome(
                    context, deadline_ms,
                    blast_action_deadline_headroom_ms(action),
                )
                if outcome is not None:
                    return outcome
                control_requested = lambda: self._control_outcome(
                    context, deadline_ms) is not None
                action_permit = self._no_return_scan_action_permit(
                    action=action,
                    action_source=action_source,
                    observation=observation,
                    geometry_checked=no_return_scan_geometry_checked,
                    route=local_detour_route,
                    pose=motion_executor.pose,
                    prior_receipt=(history[-1] if history else None),
                )
                try:
                    command_result, execution = self._dispatch_action(
                        action,
                        motion_executor,
                        control_requested,
                        allow_turn_no_valid_with_bounded_evidence=(
                            step["bounded_turn_no_valid_eligible"]
                        ),
                        action_permit=action_permit,
                    )
                except BlastControllerError as error:
                    outcome = self._control_outcome(context, deadline_ms)
                    if (
                        error.code in (
                            {"controller_command_interrupted"}
                            | _SCAN_REFUSAL_CODES
                        )
                        and outcome is not None
                    ):
                        return outcome
                    scan_outcome = self._scan_failure_outcome(
                        error.code,
                        side_search_rescan=is_side_search_rescan,
                        detour_verification=(
                            is_detour_pass_scan or is_detour_final_scan
                        ),
                    )
                    if scan_outcome is not None:
                        return scan_outcome
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
                    "pose": motion_executor.pose.to_dict(),
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
                            navigation_state = (
                                navigation_state.begin_side_search(
                                    selected_side=selected_detour_side,
                                    waypoint=side_search_waypoint,
                                    origin_scan_view=latest_scan_view,
                                )
                            )
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
                    history_item["pose"] = motion_executor.pose.to_dict()
                    try:
                        planar_projection = (
                            project_blast_scan_planar_surfaces(
                                scan=scan,
                                scan_pose=scan_pose,
                            )
                        )
                    except ValueError:
                        planar_projection = None
                    planner_scan = copy.deepcopy(scan)
                    for ray in planner_scan["rays"]:
                        if ray["observation_settled"] is not True:
                            ray["distance_mm"] = None
                            ray["range_state"] = "UNRESOLVED_SWEEP_ONLY"
                    history_item["scan"] = planner_scan
                    if (
                        planar_projection is not None
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
                    route=local_detour_route,
                )
                history.append(history_item)
                diagnostic_scan = self._publish_action_result(
                    context, action, result_observation, scan,
                    planar_projection,
                )
                outcome = self._control_outcome(context, deadline_ms)
                if outcome is not None:
                    return outcome
                if is_detour_pass_scan or is_detour_final_scan:
                    navigation_state, outcome = self._finish_detour_scan(
                        context=context,
                        deadline=deadline_ms,
                        is_pass_scan=is_detour_pass_scan,
                        scan_view=latest_scan_view,
                        selected_side=selected_detour_side,
                        result_observation=result_observation,
                        observation_settled=command_result.get(
                            "observation_settled"
                        ),
                        episode_start_heading=episode_start_heading,
                        pose=motion_executor.pose,
                        route=local_detour_route,
                        navigation_state=navigation_state,
                    )
                    if outcome is not None:
                        return outcome
                if (
                    (
                        action_source == _HOST_SIDE_SEARCH_ACTION_SOURCE
                        or action_source == _HOST_LOCAL_DETOUR_ACTION_SOURCE
                        or selects_detour_side
                    )
                    and action in ACTION_COMMANDS
                    and execution.motion.complete is not True
                ):
                    detour_motion = (
                        action_source == _HOST_LOCAL_DETOUR_ACTION_SOURCE
                    )
                    return self._outcome(
                        (
                            "detour_motion_incomplete"
                            if detour_motion
                            else "side_search_motion_incomplete"
                        ),
                        False,
                        (
                            "Host local-detour motion was incomplete"
                            if detour_motion
                            else "Host side-search motion was incomplete"
                        ),
                    )
                if side_search_setup_outcome is not None:
                    return side_search_setup_outcome
                if is_side_search_rescan:
                    if not isinstance(
                        navigation_state, SideSearchNavigationState,
                    ):
                        raise RuntimeError(
                            "BLAST side-search ownership is invalid",
                        )
                    final_scan, continuation, outcome = self._finish_side_rescan(
                        origin_view=navigation_state.origin_scan_view,
                        side_view=latest_scan_view,
                        selected_side=selected_detour_side,
                        waypoint=side_search_waypoint,
                        pose=motion_executor.pose,
                        result_observation=result_observation,
                        episode_start_heading=episode_start_heading,
                        diagnostic_scan=diagnostic_scan,
                        host_actions=navigation_state.host_actions,
                        mission=map_trace.mission,
                        remaining_slots=self.max_decisions - _index - 1,
                    )
                    if isinstance(continuation, Mapping):
                        side_search_waypoint = continuation
                        navigation_state = (
                            navigation_state.continue_to_waypoint(
                                continuation,
                            )
                        )
                        self._record_map_result(
                            map_trace, action, motion_executor.pose,
                            result_observation, selected_detour_side,
                            side_search_waypoint, None,
                            pose_observed=False,
                        )
                    elif continuation is not None:
                        local_detour_route = continuation
                        navigation_state = (
                            navigation_state.bind_local_detour(
                                local_detour_route,
                            )
                        )
                        self._record_map_result(
                            map_trace, action, motion_executor.pose,
                            result_observation, selected_detour_side,
                            side_search_waypoint, None,
                            route=local_detour_route,
                            pose_observed=False,
                        )
                    if final_scan is not None:
                        context.publish({"scan": final_scan})
                    if outcome is not None:
                        return outcome
                    continue
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


__all__ = ("ACTION_COMMANDS", "BLAST_PROFILE_ID", "BlastEpisodeError", "BlastEpisodeRuntimeAdapter")
