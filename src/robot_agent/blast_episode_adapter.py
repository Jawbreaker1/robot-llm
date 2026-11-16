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
from .episode_speech import EpisodeSpeech as BlastEpisodeSpeech, episode_cancelled as blast_episode_cancelled
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
from .waypoint_geometry import measured_motion_feedback
from .blast_stationary_recovery_flow import (
    begin_blast_iteration,
    recover_planner_iteration_actions,
    read_episode_observation,
)
from .blast_spatial_map import BlastSpatialMapBridge
from .blast_navigation_motion_execution import BlastNavigationMotionExecutor
from .blast_mission_completion import (
    BLAST_GOAL_RADIUS_MM,
    blast_directional_completion_allowed,
)
from .blast_turn_safety import blast_turn_slice_allows_continuation
from .lm_studio_controller_action import FOLLOW_WAYPOINT
from .physical_navigation_contract import (
    ADVANCE,
    OBSERVE,
    REVERSE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)
from .physical_odometry import normalize_heading_mdeg
from .robot_control_service import RobotEpisodeOutcome
from .waypoint_geometry import (
    waypoint_geometry, waypoint_target_heading_mdeg, waypoint_reached,
    waypoint_follow_motion_action, goal_reached,
)
BLAST_PROFILE_ID = ROBOT_ID
DEFAULT_MAX_DECISIONS = None
DEFAULT_MAX_OBSERVATION_AGE_MS = 3_000
DEFAULT_MIN_FORWARD_CLEARANCE_MM = 120
DEFAULT_MINIMUM_FORWARD_PROGRESS_MM = 800
# Finish the commanded axis close to its target, while tolerating ordinary
# cross-axis LEGO odometry drift. A large circular radius cut clearance legs
# short and let BLAST turn toward an obstacle before passing its corner.
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
        max_decisions: int | None = DEFAULT_MAX_DECISIONS,
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
            or (max_decisions is not None and (
                isinstance(max_decisions, bool)
                or not isinstance(max_decisions, int)
                or not 1 <= max_decisions <= 128
            ))
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
    def _has_scan_evidence(history) -> bool:
        return any(
            item.get("action") in (
                SCAN_FRONT_ARC, _STARTUP_SURROUNDINGS_ACTION,
            )
            for item in history
        )

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
        if action == OBSERVE:
            return True  # Sampling is motorless; close/missing echoes do not forbid it.
        if action == REVERSE:
            # A capability, not a clearance claim. Like EV3, the planner owns
            # bounded retreat; a front sensor cannot certify the space behind.
            return _navigation_drive_encoders_available(sensors)
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
            for action in (ADVANCE, TURN_LEFT_90, TURN_RIGHT_90, REVERSE, OBSERVE)
            if self._current_observation_allows_action(action, observation)
        ]
        if (
            self._current_observation_allows_action(
                SCAN_FRONT_ARC, observation
            )
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
        pose_before=None,
        requested_turn_deg=None,
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
                pose_before=pose_before,
                requested_turn_deg=requested_turn_deg,
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
        pose_before=None,
        requested_turn_deg=None,
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
        if command_result.get("action_error"):
            history_item["action_error"] = command_result["action_error"]
            history_item["motion_outcome"] = command_result["motion_outcome"]
        elif action == OBSERVE:
            history_item["result"] = (
                "observation_refreshed" if command_result.get("observation_settled") is True
                else "observation_unsettled"
            )
        elif action in ACTION_COMMANDS:
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
        if action in ACTION_COMMANDS and pose_before is not None:
            history_item["last_motion"] = measured_motion_feedback(
                action, pose_before, motion_executor.pose,
                completed=execution.motion.complete if execution is not None else False,
                reason=command_result.get("action_error") or (
                    "encoder_target_not_reached"
                    if execution is not None and execution.motion.status == "verification_failed" else None
                ),
                requested_turn_deg=(requested_turn_deg if requested_turn_deg is not None else
                                    90 if action == TURN_LEFT_90 else
                                    -90 if action == TURN_RIGHT_90 else None),
            )
        scan = command_result.get("scan")
        planar_projection = None
        latest_scan_view = None
        if action == SCAN_FRONT_ARC and not isinstance(scan, Mapping) and not command_result.get("action_error"):
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
        if action == SCAN_FRONT_ARC and pose_before is not None:
            history_item["last_motion"] = measured_motion_feedback(
                action, pose_before, motion_executor.pose,
                completed=(isinstance(scan, Mapping) and scan.get("state") == "complete"
                           and scan.get("restoration_verified") is True
                           and not command_result.get("action_error")),
                reason=(command_result.get("action_error") or
                        (scan.get("result") if isinstance(scan, Mapping) and scan.get("state") != "complete" else None)),
                scan_coverage_deg=scan.get("sweep_coverage_deg") if isinstance(scan, Mapping) else None,
            )
        if action == OBSERVE:
            history_item["action_completed"] = (
                not command_result.get("action_error")
                and command_result.get("observation_settled") is True)
            history_item["observation_acquired"] = history_item["action_completed"]
        elif action == SCAN_FRONT_ARC:
            history_item["action_completed"] = (
                not command_result.get("action_error") and isinstance(scan, Mapping)
                and scan.get("state") == "complete" and scan.get("restoration_verified") is True)
            history_item["observation_acquired"] = (
                scan.get("all_observations_settled") if isinstance(scan, Mapping) else None)
        elif action in ACTION_COMMANDS:
            history_item["action_completed"] = (
                not command_result.get("action_error") and execution is not None
                and history_item.get("last_motion", {}).get("completed", execution.motion.complete))
        map_trace.record_action(
            action, motion_executor.pose, result_observation,
            latest_scan_view,
            pose_observed=(action in ACTION_COMMANDS or
                           action in (SCAN_FRONT_ARC, OBSERVE)),
            observation_settled=command_result.get("observation_settled") is True,
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

    _waypoint_reached = staticmethod(waypoint_reached)

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
        return goal_reached(mission, pose)

    _active_waypoint_geometry = staticmethod(waypoint_geometry)

    _waypoint_target_heading_mdeg = staticmethod(waypoint_target_heading_mdeg)

    _waypoint_follow_motion_action = staticmethod(waypoint_follow_motion_action)

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
        *, allow_reached=False,
    ):
        """Return the turn direction and bearing to a model-owned waypoint."""

        if waypoint is None:
            return None
        if not allow_reached and cls._waypoint_reached(pose, waypoint):
            return None
        desired_heading = (
            cls._waypoint_target_heading_mdeg(pose, waypoint) / 1_000
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

    def _run_startup_perception(self, **kwargs):
        return _run_startup_perception(self, **kwargs)

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
        previous_decision = None
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
            while self.max_decisions is None or decision_count < self.max_decisions:
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
                ):
                    waypoint_plan = waypoint_plan[1:]
                    active_waypoint = (
                        waypoint_plan[0] if waypoint_plan else None
                    )
                    # A following waypoint remains Gemma's hypothesis, but it
                    # requires a fresh model decision before execution.
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
                    context.publish({"current_action": None,
                        "message": "Waiting for usable observations; mission and route retained"})
                    context.stop_requested.wait(0.25)
                    continue
                follow_motion_action = self._waypoint_follow_motion_action(
                    motion_executor.pose,
                    active_waypoint,
                    available_actions,
                )
                route_blockage = map_trace.advisory_route_blockage(
                    motion_executor.pose,
                )
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
                    # A retained route is memory, not a ban on recovery
                    # primitives. Each still passes its existing admission.
                    planner_available_actions = (FOLLOW_WAYPOINT, *available_actions)
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
                        previous_decision=previous_decision,
                    )
                    if step is not None:
                        previous_decision = step["decision_memory"]
                except BlastActionEvidenceChanged:
                    continue
                if outcome is not None:
                    return outcome
                requested_action = step["action"]
                waypoint_plan, replan, outcome = _admit_waypoint_step(
                    self, step=step, map_trace=map_trace,
                    motion_executor=motion_executor, history=history,
                    context=context, available_actions=available_actions,
                    turns_available=turns_available, latest_scan_view=latest_scan_view,
                    episode_start_heading=episode_start_heading,
                    deadline_ms=deadline_ms,
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
                # Only FOLLOW_WAYPOINT delegates heading selection to the host.
                # A direct quarter-turn is an investigation/recovery action,
                # independent of the route retained in memory.
                selected_turn_alignment = (
                    self._waypoint_turn_alignment(motion_executor.pose, active_waypoint)
                    if requested_action == FOLLOW_WAYPOINT
                    and action in (TURN_LEFT_90, TURN_RIGHT_90) else None
                )
                scan_pose = motion_executor.pose if action == SCAN_FRONT_ARC else None
                try:
                    observation, outcome = admit_blast_spoken_action(
                        self, speech, step, observation, motion_executor,
                        episode_start_heading, context, deadline_ms,
                        len(history) + 1)
                except BlastActionEvidenceChanged:
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
                )
                if outcome is not None:
                    return outcome
                new_scan_view = self._record_episode_action_result(
                    action=action,
                    action_source=action_source,
                    assessment=assessment,
                    pose_before=pose_before_action,
                    requested_turn_deg=(selected_turn_alignment[2] * selected_turn_alignment[3]
                                        if selected_turn_continuation is not None else None),
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
                history[-1]["waypoint_plan"] = copy.deepcopy(waypoint_plan)
                history[-1]["pose_before"] = pose_before_action.to_dict()
                if command_result.get("action_error"):
                    continue
                if active_waypoint is not None and history:
                    history[-1]["active_waypoint_geometry_after"] = (
                        self._active_waypoint_geometry(
                            motion_executor.pose, active_waypoint,
                        )
                    )
                if action in ACTION_COMMANDS and (
                    execution.motion.status == "verification_failed" or all(
                        getattr(motion_executor.pose, axis) == getattr(pose_before_action, axis)
                        for axis in ("x_mm", "y_mm", "heading_mdeg")
                    )
                ):
                    # A blocked turn is the same event as a blocked advance:
                    # keep the route, but let the model choose the next attempt.
                    history[-1]["route_interruption"] = _route_interruption(
                        "MOTION_PROGRESS_STALLED",
                        observation["sensors"].get("distance_mm"), active_waypoint,
                    )
                    continue
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
