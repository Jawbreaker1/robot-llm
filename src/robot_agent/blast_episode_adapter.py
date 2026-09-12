"""Thin agent-episode adapter over BLAST's single persistent BLE owner."""

from __future__ import annotations

from .blast_episode_execution import (
    ACTION_COMMANDS,
    _ROUTE_VALIDATION_ACTION_SOURCE,
    _STARTUP_SURROUNDINGS_ACTION,
    _encoder_anchor_correlated,
    _navigation_drive_encoders_available,
    _navigation_body_matched,
    _minimum_rotation_clearance_mm,
    BlastEpisodeError,
    _dispatch_episode_action,
    _run_startup_perception,
)
from .blast_episode_planning import (
    _PLAN_CONTINUATION_ACTION_SOURCE,
    _WAYPOINT_REACHED_RADIUS_MM,
    _WAYPOINT_ALIGNMENT_TRIGGER_DEG,
    _planner_history,
    _planner_map_with_route_feedback,
    _route_interruption,
    _planner_step,
    _admit_waypoint_step,
)

import copy
import math
import threading
import time
from typing import Callable, Mapping

from .blast_action_admission import (
    BlastActionEvidenceChanged,
    admit_blast_spoken_action,
    fresh_blast_action_observation,
)
from .blast_navigation_calibration import BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
from .blast_episode_deadline import (
    SETTLED_OBSERVATION_HEADROOM_MS,
    BlastEpisodeDeadline,
    blast_action_deadline_headroom_ms,
)
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
from .blast_scan_planar_projection import project_blast_scan_planar_surfaces
from .blast_scan_safety import blast_scan_sweep_is_clear
from .blast_stationary_recovery_flow import (
    begin_blast_iteration,
    recover_planner_iteration_actions,
    read_episode_observation,
)
from .blast_spatial_map import BlastSpatialMapBridge
from .coarse_navigation_grid import GRID_CELL_SIZE_MM, MODEL_ROUTE_AXIS_TOLERANCE_MM
from .blast_navigation_motion_execution import BlastNavigationMotionExecutor
from .blast_mission_completion import (
    BLAST_GOAL_HEADING_TOLERANCE_MDEG,
    BLAST_GOAL_RADIUS_MM,
    blast_directional_completion_allowed,
)
from .blast_turn_safety import blast_turn_slice_allows_continuation
from .lm_studio_controller_action import COMPLETE, FOLLOW_WAYPOINT
from .physical_navigation_contract import (
    ADVANCE,
    REVERSE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from .physical_odometry import normalize_heading_mdeg
from .robot_control_service import RobotEpisodeOutcome
BLAST_PROFILE_ID = ROBOT_ID
DEFAULT_MAX_DECISIONS = 16
DEFAULT_MAX_OBSERVATION_AGE_MS = 3_000
DEFAULT_MIN_FORWARD_CLEARANCE_MM = 120
DEFAULT_MINIMUM_FORWARD_PROGRESS_MM = 800
_STRAIGHT_SCAN_REUSE_MM = GRID_CELL_SIZE_MM * 2
# Finish the commanded axis close to its target, while tolerating ordinary
# cross-axis LEGO odometry drift. A large circular radius cut clearance legs
# short and let BLAST turn toward an obstacle before passing its corner.
_WAYPOINT_CROSS_AXIS_TOLERANCE_MM = MODEL_ROUTE_AXIS_TOLERANCE_MM
_ADVANCE_PROGRESS_STALLED = object()
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
            if action in (SCAN_FRONT_ARC, _STARTUP_SURROUNDINGS_ACTION):
                return True
            if action in ACTION_COMMANDS:
                return False
        return False

    @staticmethod
    def _scan_evidence_is_fresh(history) -> bool:
        """Reuse a scan across small straight progress, not a changed view."""

        scan_index = None
        for index in range(len(history) - 1, -1, -1):
            if history[index].get("action") in (
                SCAN_FRONT_ARC, _STARTUP_SURROUNDINGS_ACTION,
            ):
                scan_index = index
                break
        if scan_index is None:
            return False
        later_motion = [
            item for item in history[scan_index + 1:]
            if item.get("action") in ACTION_COMMANDS
        ]
        if not later_motion:
            return True
        if any(item.get("action") != ADVANCE for item in later_motion):
            return False
        scan_pose = history[scan_index].get("pose")
        current_pose = later_motion[-1].get("pose")
        try:
            displacement = math.hypot(
                current_pose["x_mm"] - scan_pose["x_mm"],
                current_pose["y_mm"] - scan_pose["y_mm"],
            )
        except (KeyError, TypeError):
            return False
        return displacement < _STRAIGHT_SCAN_REUSE_MM

    @staticmethod
    def _has_scan_evidence(history) -> bool:
        return any(
            item.get("action") in (
                SCAN_FRONT_ARC, _STARTUP_SURROUNDINGS_ACTION,
            )
            for item in history
        )

    def _current_scan_supports_bounded_reverse(
        self, history, latest_scan_view,
    ) -> bool:
        """Whether Gemma has a current full view before considering reverse."""

        if (
            not self._scan_is_current(history)
            or not isinstance(latest_scan_view, Mapping)
        ):
            return False
        scan = latest_scan_view.get("scan")
        coverage = (
            scan.get("sweep_coverage_deg")
            if isinstance(scan, Mapping) else None
        )
        return (
            isinstance(scan, Mapping)
            and scan.get("state") == "complete"
            and scan.get("result") == "restored"
            and scan.get("restoration_verified") is True
            and isinstance(coverage, (int, float))
            and not isinstance(coverage, bool)
            and 350.0 <= float(coverage) <= 390.0
        )

    @staticmethod
    def _completed_advance_allows_bounded_reverse(history) -> bool:
        """Allow retreat only across verified forward pulses not yet undone."""

        unmatched_reverses = 0
        for item in reversed(history):
            action = item.get("action")
            motion = item.get("motion")
            if isinstance(motion, Mapping) and all(
                motion.get(side + "_encoder_delta_degrees") == 0
                for side in ("left", "right")
            ):
                # A measured zero-motion attempt did not consume the return path.
                continue
            completed = (
                isinstance(motion, Mapping)
                and motion.get("command_completed") is True
            )
            if action == ADVANCE:
                if not completed:
                    return False
                if unmatched_reverses:
                    unmatched_reverses -= 1
                    continue
                return True
            if action == REVERSE:
                if not completed:
                    return False
                unmatched_reverses += 1
                continue
            if action in (SCAN_FRONT_ARC, _STARTUP_SURROUNDINGS_ACTION):
                continue
            verified_slices = (
                motion.get("verified_slice_count")
                if isinstance(motion, Mapping) else None
            )
            observed_slices = (
                motion.get("observed_slice_count")
                if isinstance(motion, Mapping) else None
            )
            if (
                action in (TURN_LEFT_90, TURN_RIGHT_90)
                and type(verified_slices) is int
                and verified_slices >= 1
                and observed_slices == verified_slices
            ):
                continue
            if (
                action == FOLLOW_WAYPOINT
                and item.get("action_source")
                == _ROUTE_VALIDATION_ACTION_SOURCE
            ):
                continue
            return False
        return False

    @staticmethod
    def _current_range_allows_rotation(observation) -> bool:
        """Missing echo permits bounded rotation, not forward clearance."""
        distance = observation["sensors"].get("distance_mm")
        state = blast_range_state(distance)
        return (
            state == RANGE_STATE_NO_VALID_DISTANCE
            or state == RANGE_STATE_MEASURED
            and float(distance) > _minimum_rotation_clearance_mm()
        )

    def _current_observation_allows_action(self, action, observation) -> bool:
        sensors = observation["sensors"]
        if not _navigation_body_matched(sensors):
            return False
        distance = sensors.get("distance_mm")
        if action == ADVANCE:
            # Pybricks' no-echo response is normal on open floor, not a fault
            # or a measured 2 m clearance. Check fresh data before every pulse;
            # the route guard still rejects known obstacles on the chosen leg.
            state = blast_range_state(distance)
            return (
                state == RANGE_STATE_NO_VALID_DISTANCE
                or state == RANGE_STATE_MEASURED
                and float(distance) > self.minimum_forward_clearance_mm
            )
        if action in (SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90):
            return (
                _navigation_drive_encoders_available(sensors)
                and self._current_range_allows_rotation(observation)
            )
        return False

    def _available_actions(
        self, observation, history=(), latest_scan_view=None,
    ) -> tuple[str, ...]:
        available = [
            action
            for action in (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90)
            if self._current_observation_allows_action(action, observation)
        ]
        sensors = observation["sensors"]
        if (
            _navigation_body_matched(sensors)
            and _navigation_drive_encoders_available(sensors)
            and (
                self._current_scan_supports_bounded_reverse(
                    history, latest_scan_view,
                )
                or self._completed_advance_allows_bounded_reverse(history)
            )
        ):
            available.append(REVERSE)
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

    def _fresh_planner_action_observation(
        self,
        *,
        action,
        episode_start_heading,
        motion_executor,
        cancel_requested,
        allow_no_valid_with_bounded_evidence=False,
        observation=None,
    ):
        return fresh_blast_action_observation(
            self, action=action,
            episode_start_heading=episode_start_heading,
            motion_executor=motion_executor,
            cancel_requested=cancel_requested,
            episode_error_type=BlastEpisodeError,
            encoder_anchor_correlated=_encoder_anchor_correlated,
            navigation_body_matched=_navigation_body_matched,
            observation=observation,
            allow_no_valid_with_bounded_evidence=(
                allow_no_valid_with_bounded_evidence
            ),
        )

    def _fresh_planner_observation_or_stop(
        self, action, episode_start_heading,
        motion_executor, context, deadline_ms,
        allow_no_valid_with_bounded_evidence=False,
    ):
        control_requested = lambda: (
            self._control_outcome(
                context, deadline_ms, SETTLED_OBSERVATION_HEADROOM_MS,
            ) is not None
        )
        try:
            observation, outcome = read_episode_observation(
                self, context=context, deadline_ms=deadline_ms,
                # A scan already performs its own stationary observation.
                motion_executor=(
                    motion_executor if action != SCAN_FRONT_ARC else None
                ),
                episode_start_heading=episode_start_heading,
            )
            if outcome is not None:
                return None, outcome
            observation = self._fresh_planner_action_observation(
                action=action,
                episode_start_heading=episode_start_heading,
                motion_executor=motion_executor,
                cancel_requested=control_requested,
                observation=observation,
                allow_no_valid_with_bounded_evidence=(
                    allow_no_valid_with_bounded_evidence
                ),
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
        if published_action is not None and published_action != action:
            history_item["requested_action"] = published_action
        if action in ACTION_COMMANDS:
            if execution is None:
                raise BlastEpisodeError(
                    "blast_command_result_invalid",
                    "BLAST motion returned no verified execution",
                )
            retained_motion = execution.motion.to_dict()
            if (
                action in (TURN_LEFT_90, TURN_RIGHT_90)
                and retained_motion.get("verified_slice_count", 0) > 0
            ):
                retained_motion["interpretation"] = (
                    "BOUNDED_TURN_PROGRESS"
                )
            history_item["motion"] = retained_motion
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
            latest_scan_view,
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

    @staticmethod
    def _semantic_advance_requested(step) -> bool:
        """Whether Gemma asked to advance toward a known target."""

        return (
            step["action"] == ADVANCE
            and (
                step["active_waypoint"] is not None
                or tuple(step["plan"]) == (ADVANCE, COMPLETE)
            )
        )

    @staticmethod
    def _advance_target_is_ahead(mission, pose, waypoint) -> bool:
        """Whether Gemma's next target lies in the robot's front half-plane."""

        target_x, target_y = (
            mission.target_point()
            if waypoint is None
            else (waypoint["x_mm"], waypoint["y_mm"])
        )
        heading = math.radians(pose.heading_mdeg / 1_000.0)
        return (
            (target_x - pose.x_mm) * math.cos(heading)
            + (target_y - pose.y_mm) * math.sin(heading)
        ) > 0

    @staticmethod
    def _waypoint_reached(pose, waypoint, *, mission=None) -> bool:
        if waypoint is None:
            return False
        if mission is not None and (
            waypoint["x_mm"], waypoint["y_mm"]
        ) == mission.target_point():
            return BlastEpisodeRuntimeAdapter._goal_corridor_entered(mission, pose)
        delta_x = abs(waypoint["x_mm"] - pose.x_mm)
        delta_y = abs(waypoint["y_mm"] - pose.y_mm)
        heading = math.radians(pose.heading_mdeg / 1_000.0)
        if abs(math.cos(heading)) >= abs(math.sin(heading)):
            along_axis, cross_axis = delta_x, delta_y
        else:
            along_axis, cross_axis = delta_y, delta_x
        return (
            along_axis <= _WAYPOINT_REACHED_RADIUS_MM
            and cross_axis <= _WAYPOINT_CROSS_AXIS_TOLERANCE_MM
        )

    @staticmethod
    def _intermediate_waypoint_plan(mission, pose, waypoints):
        """Do not reintroduce the final target after entering its corridor."""

        plan = tuple(waypoints)
        if not BlastEpisodeRuntimeAdapter._goal_corridor_entered(
            mission, pose,
        ):
            return plan

        target_x, target_y = mission.target_point()
        return tuple(
            waypoint for waypoint in plan
            if math.hypot(
                waypoint["x_mm"] - target_x,
                waypoint["y_mm"] - target_y,
            ) > BLAST_GOAL_RADIUS_MM
        )

    @staticmethod
    def _goal_corridor_entered(mission, pose) -> bool:
        return mission.distance_to_target_mm(pose) <= BLAST_GOAL_RADIUS_MM

    @staticmethod
    def _active_waypoint_geometry(pose, waypoint):
        if waypoint is None:
            return None
        delta_x = waypoint["x_mm"] - pose.x_mm
        delta_y = waypoint["y_mm"] - pose.y_mm
        bearing_mdeg = normalize_heading_mdeg(round(
            math.degrees(math.atan2(delta_y, delta_x)) * 1_000
        ))
        return {
            "distance_mm": round(math.hypot(delta_x, delta_y)),
            "bearing_mdeg": bearing_mdeg,
            "heading_error_mdeg": normalize_heading_mdeg(
                bearing_mdeg - pose.heading_mdeg
            ),
        }

    @staticmethod
    def _waypoint_axis_heading_mdeg(pose, waypoint):
        """Translate an accepted coarse leg into one cardinal heading."""

        delta_x = waypoint["x_mm"] - pose.x_mm
        delta_y = waypoint["y_mm"] - pose.y_mm
        if (
            abs(delta_x) > MODEL_ROUTE_AXIS_TOLERANCE_MM
            and abs(delta_y) > MODEL_ROUTE_AXIS_TOLERANCE_MM
        ):
            return normalize_heading_mdeg(round(
                math.degrees(math.atan2(delta_y, delta_x)) * 1_000
            ))
        if abs(delta_x) >= abs(delta_y):
            return 0 if delta_x >= 0 else -180_000
        return 90_000 if delta_y >= 0 else -90_000

    @classmethod
    def _waypoint_follow_motion_action(
        cls, pose, waypoint, available_actions, *, mission=None,
    ):
        """Resolve one explicit waypoint-follow request to a bounded primitive."""

        geometry = cls._active_waypoint_geometry(pose, waypoint)
        if geometry is None or cls._waypoint_reached(pose, waypoint, mission=mission):
            return None
        heading_error = normalize_heading_mdeg(
            cls._waypoint_axis_heading_mdeg(pose, waypoint)
            - pose.heading_mdeg
        )
        target_turn = (
            TURN_LEFT_90 if heading_error > 0 else TURN_RIGHT_90
        )
        if abs(heading_error) >= round(
            _WAYPOINT_ALIGNMENT_TRIGGER_DEG * 1_000
        ):
            action = target_turn
        elif ADVANCE in available_actions:
            action = ADVANCE
        else:
            return None
        return action if action in available_actions else None

    @classmethod
    def _desired_heading_turn_alignment(
        cls, desired_heading, pose,
    ):
        error = cls._heading_delta(desired_heading, pose.heading_mdeg / 1000)
        if error is None or error == 0:
            return None
        return (
            TURN_LEFT_90 if error > 0 else TURN_RIGHT_90,
            desired_heading,
            1 if error > 0 else -1,
            abs(error),
        )

    @classmethod
    def _waypoint_turn_alignment(
        cls, pose, waypoint,
        *, allow_reached=False, mission=None,
    ):
        """Return the turn direction and bearing to a model-owned waypoint."""

        if waypoint is None:
            return None
        if not allow_reached and cls._waypoint_reached(pose, waypoint, mission=mission):
            return None
        desired_heading = (
            cls._waypoint_axis_heading_mdeg(pose, waypoint) / 1_000
        )
        return cls._desired_heading_turn_alignment(
            desired_heading, pose,
        )

    @classmethod
    def _waypoint_alignment_continuation(
        cls, *, desired_heading, direction, start_observation, start_heading_mdeg,
        allow_no_valid_distance,
        alignment_trigger_deg=_WAYPOINT_ALIGNMENT_TRIGGER_DEG,
    ):
        """Continue a model-selected turn while its waypoint error is large."""

        start_imu = cls._heading(start_observation["sensors"])

        def continue_requested(command_result):
            if not blast_turn_slice_allows_continuation(
                command_result,
                allow_no_valid_distance_with_bounded_evidence=(
                    allow_no_valid_distance
                ),
            ):
                return False
            sensors = command_result.get("observation")
            current_heading = cls._heading(sensors)
            relative_heading = cls._heading_delta(
                current_heading, start_imu,
            )
            if relative_heading is None:
                return False
            remaining = cls._heading_delta(
                desired_heading, start_heading_mdeg / 1000 - relative_heading,
            )
            return (
                remaining is not None
                and abs(remaining) >= alignment_trigger_deg
                and remaining * direction > 0
            )

        return continue_requested

    def _continue_semantic_advance(
        self, *, step, motion_executor, episode_start_heading,
        history, latest_scan_view, map_trace, context, deadline_ms,
        follow_waypoint=False,
    ):
        """Track the accepted waypoint between pulses until a relevant event."""

        if not self._semantic_advance_requested(step):
            return None
        mission = map_trace.mission
        waypoint = step["active_waypoint"]

        def target_distance():
            if waypoint is None:
                return mission.distance_to_target_mm(motion_executor.pose)
            return math.hypot(
                waypoint["x_mm"] - motion_executor.pose.x_mm,
                waypoint["y_mm"] - motion_executor.pose.y_mm,
            )

        distance = target_distance()
        progress = mission.longitudinal_progress_mm(motion_executor.pose)

        def stalled():
            if history:
                result_observation = history[-1].get("result_observation")
                history[-1]["route_interruption"] = _route_interruption(
                    "MOTION_PROGRESS_STALLED",
                    result_observation.get("distance_mm")
                    if isinstance(result_observation, Mapping) else None,
                    waypoint,
                )
            return _ADVANCE_PROGRESS_STALLED

        def target_pending():
            if waypoint is None:
                return (
                    progress < mission.minimum_forward_progress_mm
                    and mission.heading_aligned(motion_executor.pose)
                )
            return (
                not self._waypoint_reached(motion_executor.pose, waypoint, mission=mission)
                and self._advance_target_is_ahead(
                    mission, motion_executor.pose, waypoint,
                )
            )

        while target_pending():
            outcome = self._control_outcome(context, deadline_ms)
            if outcome is not None:
                return outcome
            geometry = self._active_waypoint_geometry(motion_executor.pose, waypoint)
            # Keep LEGO-scale slack: only correct a course that would miss the
            # waypoint corridor, not every small heading discrepancy.
            correcting = (
                follow_waypoint and geometry is not None
                and abs(geometry["heading_error_mdeg"])
                >= round(_WAYPOINT_ALIGNMENT_TRIGGER_DEG * 1000)
                and abs(target_distance() * math.sin(math.radians(
                    geometry["heading_error_mdeg"] / 1000,
                ))) > _WAYPOINT_CROSS_AXIS_TOLERANCE_MM
            )
            action = (
                TURN_LEFT_90 if geometry["heading_error_mdeg"] > 0 else TURN_RIGHT_90
            ) if correcting else ADVANCE
            try:
                observation, outcome = (
                    self._fresh_planner_observation_or_stop(
                        action,
                        episode_start_heading,
                        motion_executor,
                        context,
                        deadline_ms,
                        allow_no_valid_with_bounded_evidence=correcting,
                    )
                )
            except BlastActionEvidenceChanged:
                return None
            if outcome is not None:
                return outcome
            turn_continuation = None
            if correcting:
                turn_continuation = self._waypoint_alignment_continuation(
                    desired_heading=geometry["bearing_mdeg"] / 1000,
                    direction=1 if action == TURN_LEFT_90 else -1,
                    start_observation=observation,
                    start_heading_mdeg=motion_executor.pose.heading_mdeg,
                    allow_no_valid_distance=True,
                )
            command_result, execution, observation, outcome = (
                _dispatch_episode_action(self, action=action,
                    observation=observation,
                    geometry_checked=False,
                    motion_executor=motion_executor,
                    prior_receipt=history[-1],
                    allow_turn_no_valid_with_bounded_evidence=correcting,
                    context=context,
                    deadline_ms=deadline_ms,
                    map_trace=map_trace,
                    turn_continue_requested=turn_continuation,
                    trim_turn=correcting,
                )
            )
            if outcome is not None:
                return outcome
            self._record_episode_action_result(
                action=action,
                action_source=_PLAN_CONTINUATION_ACTION_SOURCE,
                assessment=step["assessment"],
                plan=step["plan"],
                command_result=command_result,
                execution=execution,
                scan_pose=None,
                motion_executor=motion_executor,
                history=history,
                map_trace=map_trace,
                context=context,
            )
            if correcting:
                remaining = self._active_waypoint_geometry(motion_executor.pose, waypoint)
                if abs(remaining["heading_error_mdeg"]) >= abs(geometry["heading_error_mdeg"]):
                    return stalled()
                distance = target_distance()
                continue
            if waypoint is None:
                next_progress = mission.longitudinal_progress_mm(
                    motion_executor.pose
                )
                if next_progress <= progress:
                    return stalled()
                progress = next_progress
            else:
                next_distance = target_distance()
                if next_distance >= distance:
                    return stalled()
                distance = next_distance
        return None

    def _run_startup_perception(self, **kwargs):
        return _run_startup_perception(self, **kwargs)

    def _selected_turn_alignment(self, action, active_waypoint, pose, mission):
        """Use the existing waypoint or final-heading alignment target."""
        selected_turn_alignment = None
        selected_turn_trigger_deg = _WAYPOINT_ALIGNMENT_TRIGGER_DEG
        if action in (TURN_LEFT_90, TURN_RIGHT_90):
            if active_waypoint is not None:
                selected_turn_alignment = self._waypoint_turn_alignment(
                    pose,
                    active_waypoint,
                    mission=mission,
                )
            if (
                selected_turn_alignment is None
                and (
                    mission.longitudinal_progress_mm(
                        pose
                    ) >= mission.minimum_forward_progress_mm
                    or mission.distance_to_target_mm(
                        pose
                    ) <= BLAST_GOAL_RADIUS_MM
                )
            ):
                selected_turn_alignment = self._desired_heading_turn_alignment(
                    mission.reference_heading_mdeg / 1_000,
                    pose,
                )
                selected_turn_trigger_deg = (
                    BLAST_GOAL_HEADING_TOLERANCE_MDEG / 1_000
                )
        return selected_turn_alignment, selected_turn_trigger_deg

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
        active_waypoint = None
        waypoint_plan = ()
        route_following = False
        latest_scan_view = None
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
            decision_count = 0
            iteration_index = 0
            while route_following or decision_count < self.max_decisions:
                _index = iteration_index
                iteration_index += 1
                outcome = self._control_outcome(context, deadline_ms)
                if outcome is not None:
                    return outcome
                (observation, available_actions, turns_available,
                 iteration_runtime, outcome) = begin_blast_iteration(
                    self, context=context, deadline_ms=deadline_ms,
                    index=_index, history=history,
                    latest_scan_view=latest_scan_view,
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
                        observation, available_actions, turns_available,
                        latest_scan_view, outcome,
                    ) = self._run_startup_perception(
                        observation=observation,
                        available_actions=available_actions,
                        turns_available=turns_available,
                        latest_scan_view=latest_scan_view,
                        history=history,
                        motion_executor=motion_executor,
                        episode_start_heading=episode_start_heading,
                        map_trace=map_trace,
                        context=context,
                        deadline_ms=deadline_ms,
                    )
                    if outcome is not None:
                        return outcome
                if self._waypoint_reached(
                    motion_executor.pose, active_waypoint,
                    mission=map_trace.mission,
                ):
                    waypoint_plan = waypoint_plan[1:]
                    active_waypoint = (
                        waypoint_plan[0] if waypoint_plan else None
                    )
                    # A following waypoint remains Gemma's hypothesis, but it
                    # requires a fresh model decision before execution.
                    route_following = False
                    map_trace.set_advisory_waypoint_plan(
                        waypoint_plan,
                        pose=motion_executor.pose,
                        observation=observation["sensors"],
                        observed_at_unix_ms=observation[
                            "observed_at_unix_ms"
                        ],
                    )
                completion_allowed = blast_directional_completion_allowed(
                    mission=map_trace.mission, pose=motion_executor.pose,
                    localization_valid=motion_executor.localization_valid,
                )
                if completion_allowed:
                    available_actions = ()
                elif (
                    ADVANCE in available_actions
                    and not self._advance_target_is_ahead(
                        map_trace.mission,
                        motion_executor.pose,
                        active_waypoint,
                    )
                ):
                    available_actions = tuple(
                        action for action in available_actions
                        if action != ADVANCE
                    )
                elif (
                    active_waypoint is None
                    and map_trace.mission.longitudinal_progress_mm(
                        motion_executor.pose
                    )
                    >= map_trace.mission.minimum_forward_progress_mm
                ):
                    available_actions = tuple(
                        action for action in available_actions
                        if action != ADVANCE
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
                        latest_scan_view=latest_scan_view,
                    ))
                    if outcome is not None: return outcome
                    if refreshed_turns is not None:
                        turns_available = refreshed_turns
                if not available_actions and not completion_allowed:
                    return self._outcome(
                        "no_safe_blast_action",
                        False,
                        "BLAST has no currently observed safe motion or scan",
                    )
                follow_motion_action = self._waypoint_follow_motion_action(
                    motion_executor.pose,
                    active_waypoint,
                    available_actions,
                    mission=map_trace.mission,
                )
                route_blockage = map_trace.advisory_route_blockage(
                    motion_executor.pose,
                )
                continuing_route = (
                    route_following
                    and active_waypoint is not None
                    and follow_motion_action is not None
                    and route_blockage is None
                )
                if route_following and not continuing_route:
                    geometry = self._active_waypoint_geometry(
                        motion_executor.pose, active_waypoint,
                    )
                    if (
                        geometry is not None
                        and route_blockage is None
                        and ADVANCE not in available_actions
                        and abs(geometry["heading_error_mdeg"]) < round(
                            _WAYPOINT_ALIGNMENT_TRIGGER_DEG * 1_000
                        )
                    ):
                        interruption = _route_interruption(
                            "FORWARD_CLEARANCE_UNAVAILABLE",
                            observation["sensors"].get("distance_mm"),
                            active_waypoint,
                        )
                        if history and history[-1].get("action") == ADVANCE:
                            history[-1]["route_interruption"] = interruption
                        else:
                            history.append({
                                "action": FOLLOW_WAYPOINT,
                                "requested_action": FOLLOW_WAYPOINT,
                                "action_source": (
                                    _ROUTE_VALIDATION_ACTION_SOURCE
                                ),
                                "route_interruption": interruption,
                                "pose": motion_executor.pose.to_dict(),
                            })
                    route_following = False
                if not continuing_route and decision_count >= self.max_decisions:
                    break
                if continuing_route:
                    step = {
                        "action": FOLLOW_WAYPOINT,
                        "assessment": "Continue the model-owned waypoint route",
                        "utterance": None,
                        "plan": [FOLLOW_WAYPOINT],
                        "action_source": _PLAN_CONTINUATION_ACTION_SOURCE,
                        "observation": observation,
                        "bounded_no_valid_eligible": False,
                        "active_waypoint": active_waypoint,
                        "waypoint_plan": waypoint_plan,
                    }
                    outcome = None
                else:
                    planner_local_map_evidence = (
                        map_trace.planner_local_map_evidence(
                            motion_executor.pose
                        )
                    )
                    planner_local_map_evidence = (
                        _planner_map_with_route_feedback(
                            planner_local_map_evidence,
                            history,
                        )
                    )
                    direct_goal_blocked = (
                        isinstance(planner_local_map_evidence, Mapping)
                        and planner_local_map_evidence.get(
                            "direct_goal_blockage"
                        ) is not None
                    )
                    if route_blockage is not None:
                        planner_available_actions = (
                            FOLLOW_WAYPOINT, *available_actions
                        )
                    elif (
                        active_waypoint is not None
                        and follow_motion_action is not None
                    ):
                        planner_available_actions = (
                            FOLLOW_WAYPOINT,
                            *(
                                action for action in available_actions
                                if action in (SCAN_FRONT_ARC, REVERSE)
                            ),
                        )
                    elif active_waypoint is not None:
                        # The retained route cannot currently be executed.
                        # Return control to the model instead of repeatedly
                        # accepting the same blocked waypoint.
                        planner_available_actions = available_actions
                    elif any(
                        action in available_actions
                        for action in (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90)
                    ) and not self._goal_corridor_entered(
                        map_trace.mission, motion_executor.pose,
                    ) and (
                        map_trace.mission.distance_to_target_mm(
                            motion_executor.pose
                        ) > BLAST_GOAL_RADIUS_MM
                    ):
                        planner_available_actions = (
                            FOLLOW_WAYPOINT, *available_actions
                        )
                    else:
                        planner_available_actions = available_actions
                    decision_count += 1
                    try:
                        step, outcome = _planner_step(self, speech=speech,
                            planner=planner, context=context,
                            observation=observation, history=history,
                            available_actions=planner_available_actions,
                            completion_allowed=completion_allowed,
                            turns_available=turns_available,
                            latest_scan_view=latest_scan_view,
                            motion_executor=motion_executor,
                            episode_start_heading=episode_start_heading,
                            deadline_ms=deadline_ms,
                            abort_allowed=(
                                not available_actions and not completion_allowed
                            ),
                            local_map_evidence=(
                                planner_local_map_evidence
                            ),
                            active_waypoint=active_waypoint,
                            active_waypoint_plan=waypoint_plan,
                            waypoint_required=(
                                active_waypoint is not None
                                or route_blockage is not None
                                or direct_goal_blocked
                                or (
                                    not completion_allowed
                                    and ADVANCE not in available_actions
                                    and not self._goal_corridor_entered(
                                        map_trace.mission,
                                        motion_executor.pose,
                                    )
                                )
                            ),
                            # BLAST replans after strategic evidence changes. A
                            # previous motor-action tail is not current evidence.
                            active_plan=(),
                        )
                    except BlastActionEvidenceChanged:
                        route_following = False
                        continue
                if outcome is not None:
                    return outcome
                requested_action = step["action"]
                waypoint_plan, route_following, replan, outcome = _admit_waypoint_step(
                    self, step=step, map_trace=map_trace,
                    motion_executor=motion_executor, history=history,
                    context=context, available_actions=available_actions,
                    turns_available=turns_available, latest_scan_view=latest_scan_view,
                    episode_start_heading=episode_start_heading,
                    deadline_ms=deadline_ms, route_following=route_following,
                )
                active_waypoint = waypoint_plan[0] if waypoint_plan else None
                if outcome is not None:
                    return outcome
                if replan:
                    continue
                action = step["action"]
                assessment = step["assessment"]
                plan = step["plan"]
                action_source = step["action_source"]
                observation = step["observation"]
                selected_turn_continuation = None
                selected_turn_alignment, selected_turn_trigger_deg = self._selected_turn_alignment(
                    action, active_waypoint, motion_executor.pose, map_trace.mission,
                )
                scan_pose = motion_executor.pose if action == SCAN_FRONT_ARC else None
                try:
                    observation, outcome = admit_blast_spoken_action(
                        self, speech, step, observation, motion_executor,
                        episode_start_heading, context, deadline_ms,
                        len(history) + 1)
                except BlastActionEvidenceChanged:
                    route_following = False
                    context.publish({"current_action": None, "plan": []})
                    continue
                if outcome is not None:
                    return outcome
                # An admitted bounded turn may finish through missing echoes.
                # This permission lasts for this turn only; fresh close readings
                # still stop it in blast_turn_slice_allows_continuation.
                allow_turn_no_valid = action in (TURN_LEFT_90, TURN_RIGHT_90) and (
                    step["bounded_no_valid_eligible"]
                    or self._current_observation_allows_action(action, observation)
                )
                if (
                    selected_turn_alignment is not None
                    and selected_turn_alignment[0] == action
                ):
                    _turn, desired_heading, direction, _error = selected_turn_alignment
                    selected_turn_continuation = self._waypoint_alignment_continuation(
                        desired_heading=desired_heading,
                        direction=direction,
                        start_observation=observation,
                        start_heading_mdeg=motion_executor.pose.heading_mdeg,
                        allow_no_valid_distance=allow_turn_no_valid,
                        alignment_trigger_deg=selected_turn_trigger_deg,
                    )
                no_return_scan_geometry_checked = (
                    _planner_scan_geometry_checked(
                        action,
                        observation,
                        latest_scan_view,
                        motion_executor.pose,
                    )
                )
                pose_before_action = motion_executor.pose
                (
                    command_result,
                    execution,
                    observation,
                    outcome,
                ) = _dispatch_episode_action(self, action=action,
                    observation=observation,
                    geometry_checked=no_return_scan_geometry_checked,
                    motion_executor=motion_executor,
                    prior_receipt=(history[-1] if history else None),
                    allow_turn_no_valid_with_bounded_evidence=allow_turn_no_valid,
                    context=context,
                    deadline_ms=deadline_ms,
                    map_trace=map_trace,
                    perception_only_scan=(action == SCAN_FRONT_ARC),
                    turn_continue_requested=selected_turn_continuation,
                    scan_refusal_can_replan=(
                        action == SCAN_FRONT_ARC
                        and self._completed_advance_allows_bounded_reverse(
                            history
                        )
                    ),
                )
                if outcome is not None:
                    return outcome
                scan_refusal = (
                    command_result.get("recoverable_scan_refusal")
                    if isinstance(command_result, Mapping) else None
                )
                if isinstance(scan_refusal, Mapping):
                    route_following = False
                    history.append({
                        "action": SCAN_FRONT_ARC,
                        "requested_action": requested_action,
                        "action_source": action_source,
                        "scan_refusal": dict(scan_refusal),
                        "result_observation": observation["sensors"],
                        "observation_settled": observation["sensors"].get(
                            "motion_active"
                        ) is False,
                        "pose": motion_executor.pose.to_dict(),
                    })
                    context.publish({
                        "current_action": None,
                        "plan": list(plan),
                        "message": (
                            "BLAST scan could not start here; Gemma can "
                            "reposition and replan"
                        ),
                    })
                    continue
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
                    published_action=(
                        FOLLOW_WAYPOINT
                        if requested_action == FOLLOW_WAYPOINT else None
                    ),
                )
                if active_waypoint is not None and history:
                    history[-1]["active_waypoint_geometry_after"] = (
                        self._active_waypoint_geometry(
                            motion_executor.pose, active_waypoint,
                        )
                    )
                if action in ACTION_COMMANDS and all(
                    getattr(motion_executor.pose, axis) == getattr(pose_before_action, axis)
                    for axis in ("x_mm", "y_mm", "heading_mdeg")
                ):
                    # A blocked turn is the same event as a blocked advance:
                    # keep the route, but let the model choose the next attempt.
                    route_following = False
                    history[-1]["route_interruption"] = _route_interruption(
                        "MOTION_PROGRESS_STALLED",
                        observation["sensors"].get("distance_mm"), active_waypoint,
                    )
                    continue
                if action == SCAN_FRONT_ARC:
                    latest_scan_view = new_scan_view
                if action == ADVANCE:
                    advance_result = self._continue_semantic_advance(
                        step=step,
                        follow_waypoint=requested_action == FOLLOW_WAYPOINT,
                        motion_executor=motion_executor,
                        episode_start_heading=episode_start_heading,
                        history=history,
                        latest_scan_view=latest_scan_view,
                        map_trace=map_trace,
                        context=context,
                        deadline_ms=deadline_ms,
                    )
                    if advance_result is _ADVANCE_PROGRESS_STALLED:
                        route_following = False
                    elif advance_result is not None:
                        return advance_result
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
