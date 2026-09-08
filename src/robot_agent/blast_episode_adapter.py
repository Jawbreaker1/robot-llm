"""Thin agent-episode adapter over BLAST's single persistent BLE owner."""

from __future__ import annotations

import copy
from functools import partial
import math
import threading
import time
from typing import Callable, Mapping

from .blast_action_admission import (
    BlastActionEvidenceChanged,
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
    read_episode_observation,
)
from .blast_spatial_map import BlastSpatialMapBridge
from .coarse_navigation_grid import (
    GRID_CELL_SIZE_MM,
    MODEL_ROUTE_AXIS_TOLERANCE_MM,
)
from .blast_navigation_action_profile import (
    BLAST_NAVIGATION_COMMANDS,
)
from .blast_navigation_motion_execution import (
    BlastNavigationMotionExecutor,
)
from .blast_mission_completion import (
    BLAST_GOAL_HEADING_TOLERANCE_MDEG,
    BLAST_GOAL_RADIUS_MM,
    blast_directional_completion_allowed,
)
from .blast_turn_safety import blast_turn_slice_allows_continuation
from .lm_studio import LMStudioProtocolError
from .lm_studio_controller_action import (
    ABORT,
    COMPLETE,
    FOLLOW_WAYPOINT,
    ControllerActionContext,
    ControllerActionPlannerResult,
)
from .navigation_diagnostics import record_navigation_diagnostic
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
ACTION_COMMANDS = {
    action: BLAST_NAVIGATION_COMMANDS[action]
    for action in (ADVANCE, REVERSE, TURN_LEFT_90, TURN_RIGHT_90)
}
BLAST_PLAN_ACTIONS = (
    FOLLOW_WAYPOINT,
    ADVANCE,
    REVERSE,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    SCAN_FRONT_ARC,
)
DEFAULT_MAX_DECISIONS = 16
DEFAULT_MAX_OBSERVATION_AGE_MS = 3_000
DEFAULT_MIN_FORWARD_CLEARANCE_MM = 120
DEFAULT_MINIMUM_FORWARD_PROGRESS_MM = 800
_PLANNER_ACTION_SOURCE = "PLANNER_ACTION"
_PLAN_CONTINUATION_ACTION_SOURCE = "PLAN_CONTINUATION"
_STARTUP_PERCEPTION_ACTION_SOURCE = "STARTUP_PERCEPTION"
_ROUTE_VALIDATION_ACTION_SOURCE = "ROUTE_VALIDATION"
_STARTUP_SURROUNDINGS_ACTION = "SCAN_SURROUNDINGS"
_STRAIGHT_SCAN_REUSE_MM = GRID_CELL_SIZE_MM * 2
# Finish the commanded axis close to its target, while tolerating ordinary
# cross-axis LEGO odometry drift. A large circular radius cut clearance legs
# short and let BLAST turn toward an obstacle before passing its corner.
_WAYPOINT_REACHED_RADIUS_MM = 25
_WAYPOINT_CROSS_AXIS_TOLERANCE_MM = MODEL_ROUTE_AXIS_TOLERANCE_MM
_WAYPOINT_ALIGNMENT_TRIGGER_DEG = 12.0
_STARTUP_HEADING_RESTORATION_TOLERANCE_DEG = 20.0
_ADVANCE_PROGRESS_STALLED = object()
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
        "observation_settled",
        "pose",
        "motion",
        "odometry_reanchored_after_scan",
        "scan_view_count",
        "scan_state",
        "sweep_coverage_deg",
        "heading_restoration",
        "route_rejection",
        "route_interruption",
        "active_waypoint_geometry_after",
        "scan_refusal",
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

    def _recent_evidence_allows_bounded_advance(
        self, history, latest_scan_view,
    ) -> bool:
        """Reuse evidence in the actual travel direction, not named scan sides."""

        current_pose = next((
            item["pose"] for item in reversed(history)
            if isinstance(item.get("pose"), Mapping)
        ), None)
        candidates = []
        for item in reversed(history):
            sensors = item.get("result_observation")
            pose = item.get("pose")
            if (
                isinstance(sensors, Mapping)
                and item.get("observation_settled") is True
                and blast_range_state(sensors.get("distance_mm"))
                == RANGE_STATE_MEASURED
                and isinstance(pose, Mapping)
                and current_pose is not None
            ):
                candidates.append((pose, pose.get("heading_mdeg", 0), sensors["distance_mm"]))
            action = item.get("action")
            if action in (SCAN_FRONT_ARC, _STARTUP_SURROUNDINGS_ACTION):
                break
        scan = latest_scan_view.get("scan") if isinstance(latest_scan_view, Mapping) else None
        rays = scan.get("angular_rays") if isinstance(scan, Mapping) else None
        scan_pose = latest_scan_view.get("scan_pose") if isinstance(latest_scan_view, Mapping) else None
        if isinstance(scan_pose, Mapping) and isinstance(rays, list):
            for ray in rays:
                if ray.get("observation_settled") is not True:
                    continue
                bearing = ray.get("relative_heading_deg")
                if not isinstance(bearing, (int, float)) or isinstance(bearing, bool):
                    continue
                state = ray.get("range_state")
                if state not in (RANGE_STATE_MEASURED, RANGE_STATE_NO_VALID_DISTANCE):
                    continue
                candidates.append((
                    scan_pose, scan_pose["heading_mdeg"] - round(bearing * 1000),
                    ray.get("distance_mm") if state == RANGE_STATE_MEASURED else None,
                ))
        if current_pose is None:
            return False
        heading = current_pose.get("heading_mdeg", 0)
        radians = math.radians(heading / 1000)
        for pose, bearing, distance in candidates:
            dx = current_pose["x_mm"] - pose["x_mm"]
            dy = current_pose["y_mm"] - pose["y_mm"]
            if (
                abs(normalize_heading_mdeg(bearing - heading)) > 12_000
                or math.hypot(dx, dy) >= _STRAIGHT_SCAN_REUSE_MM
                or abs(-dx * math.sin(radians) + dy * math.cos(radians))
                > _WAYPOINT_CROSS_AXIS_TOLERANCE_MM
            ):
                continue
            travelled = max(0, dx * math.cos(radians) + dy * math.sin(radians))
            # A settled no-return permits bounded exploration; it never becomes
            # a free map cell. The route guard still retains measured obstacles.
            if distance is None or distance - travelled > self.minimum_forward_clearance_mm:
                return True
        return False

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
            return (
                blast_range_state(distance) == RANGE_STATE_MEASURED
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
            blast_range_state(
                observation["sensors"].get("distance_mm")
            ) == RANGE_STATE_NO_VALID_DISTANCE
        ):
            if self._recent_evidence_allows_bounded_advance(
                history, latest_scan_view,
            ):
                available.append(ADVANCE)
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
        surroundings_scan=False,
        turn_continue_requested=None,
        trim_turn=False,
    ):
        if action == SCAN_FRONT_ARC:
            kwargs = {"cancel_requested": control_requested}
            if action_permit is not None:
                kwargs["action_permit"] = action_permit
            surroundings = getattr(
                self.controller, "scan_surroundings", None,
            )
            result = (
                surroundings(**kwargs)
                if surroundings_scan and callable(surroundings)
                else self.controller.command("scan_front_arc", **kwargs)
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
        surroundings_scan=False, turn_continue_requested=None,
        scan_refusal_can_replan=False,
        trim_turn=False,
    ):
        outcome = self._control_outcome(
            context, deadline_ms, blast_action_deadline_headroom_ms(action),
        )
        if outcome is not None:
            return None, None, observation, outcome
        control_requested = lambda: self._control_outcome(
            context, deadline_ms) is not None
        for attempt in range(2):
            if not _encoder_anchor_correlated(
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
                record_navigation_diagnostic(
                    "controller_action_started", episode_id=context.episode_id,
                    robot_id=ROBOT_ID, action=action,
                    pose=motion_executor.pose.to_dict(),
                    observation_before=observation,
                )
                command_result, execution = self._dispatch_action(
                    action,
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

    def _planner_step(
        self,
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
        active_plan,
    ):
        outcome = self._control_outcome(context, deadline_ms)
        if outcome is not None: return None, outcome
        try:
            planner_context = ControllerActionContext(
                goal=context.request.goal,
                locale=context.request.locale,
                robot_id=ROBOT_ID,
                controller_id=CONTROLLER_ID,
                available_actions=available_actions,
                observation=_planner_navigation_value(observation),
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
                        self._active_waypoint_geometry(
                            motion_executor.pose, active_waypoint,
                        )
                    )
                ),
                active_waypoint_plan=active_waypoint_plan,
                waypoint_reached_radius_mm=_WAYPOINT_REACHED_RADIUS_MM,
                waypoint_required=waypoint_required,
                plan_actions=BLAST_PLAN_ACTIONS,
                active_plan=active_plan,
            )
            # An unusable reply is not a navigation event. Retry once with the
            # same goal, route, pose and observations; do not scan or move.
            for attempt in range(2):
                try:
                    result = planner.decide(planner_context)
                    break
                except LMStudioProtocolError:
                    outcome = self._control_outcome(context, deadline_ms)
                    if outcome is not None:
                        return None, outcome
                    if attempt:
                        raise
                    record_navigation_diagnostic(
                        "planner_reply_retry", robot_id=ROBOT_ID,
                        episode_id=context.episode_id,
                    )
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
        bounded_turn = (
            turns_available
            and action in (TURN_LEFT_90, TURN_RIGHT_90)
        )
        scan_guided_advance = (
            action == ADVANCE
            and self._recent_evidence_allows_bounded_advance(
                history, latest_scan_view,
            )
        )
        bounded_reverse = (
            action == REVERSE
            and (
                self._current_scan_supports_bounded_reverse(
                    history, latest_scan_view,
                )
                or self._completed_advance_allows_bounded_reverse(history)
            )
        )
        bounded_no_valid = (
            bounded_turn or scan_guided_advance or bounded_reverse
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
        outcome = self._control_outcome(context, deadline_ms)
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
                return None, self._control_outcome(context, deadline_ms)
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
            "bounded_no_valid_eligible": bounded_no_valid,
            "active_waypoint": decision.waypoint,
            "waypoint_plan": (
                () if decision.waypoint is None else (
                    decision.waypoint,
                    *decision.following_waypoints,
                )
            ),
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
            bounded_evidence = correcting or self._recent_evidence_allows_bounded_advance(
                history, latest_scan_view,
            )
            try:
                observation, outcome = (
                    self._fresh_planner_observation_or_stop(
                        action,
                        episode_start_heading,
                        motion_executor,
                        context,
                        deadline_ms,
                        allow_no_valid_with_bounded_evidence=bounded_evidence,
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
                self._dispatch_episode_action(
                    action=action,
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

    def _restore_startup_heading(
        self, *, scan_item, motion_executor, episode_start_heading,
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

        observation = self._with_navigation_reference(
            self._observation(), episode_start_heading,
        )
        observation["odometry"] = motion_executor.pose.to_dict()
        alignment = self._desired_heading_turn_alignment(
            0.0, motion_executor.pose,
        )
        if alignment is None:
            return result_observation, None, correction
        action, desired_heading, direction, _error = alignment
        correction["action"] = action
        command_result, execution, _observation, outcome = (
            self._dispatch_episode_action(
                action=action,
                observation=observation,
                geometry_checked=False,
                motion_executor=motion_executor,
                prior_receipt=scan_item,
                allow_turn_no_valid_with_bounded_evidence=True,
                context=context,
                deadline_ms=deadline_ms,
                map_trace=map_trace,
                turn_continue_requested=(
                    self._waypoint_alignment_continuation(
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
        self, *, observation, available_actions, turns_available,
        latest_scan_view, history, motion_executor, episode_start_heading,
        map_trace, context, deadline_ms,
    ):
        """Acquire one complete encoder-measured view before Gemma decides."""

        initial_scan_view_count = len(map_trace.planar_scan_views)
        startup_history = []
        action = SCAN_FRONT_ARC
        try:
            observation, outcome = self._fresh_planner_observation_or_stop(
                action, episode_start_heading, motion_executor,
                context, deadline_ms,
            )
        except BlastEpisodeError:
            outcome = self._outcome(
                "blast_startup_perception_incomplete", False,
                "BLAST could not safely begin its surroundings scan",
            )
        if outcome is not None:
            return (
                observation, available_actions, turns_available,
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
                surroundings_scan=True,
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
                observation, available_actions, turns_available,
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
        startup_scan_item = startup_history[-1]
        outcome = self._control_outcome(context, deadline_ms)
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
                self._outcome(
                    "blast_startup_perception_incomplete", False,
                    "BLAST startup scan produced no localized surroundings",
                ),
            )
        (
            final_observation, outcome, heading_restoration,
        ) = self._restore_startup_heading(
            scan_item=startup_scan_item,
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
            self, context=context, deadline_ms=deadline_ms,
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
                        step, outcome = self._planner_step(
                            speech=speech,
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
                action = requested_action
                assessment = step["assessment"]
                plan = step["plan"]
                action_source = step["action_source"]
                observation = step["observation"]
                waypoint_plan = self._intermediate_waypoint_plan(
                    map_trace.mission,
                    motion_executor.pose,
                    step["waypoint_plan"],
                )
                reached_waypoints = []
                while (
                    waypoint_plan
                    and self._waypoint_reached(
                        motion_executor.pose, waypoint_plan[0],
                        mission=map_trace.mission,
                    )
                ):
                    reached_waypoints.append(waypoint_plan[0])
                    waypoint_plan = waypoint_plan[1:]
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
                    continue
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
                    route_following = active_waypoint is not None
                    if route_blockage is not None:
                        route_blockage = dict(route_blockage)
                    if route_blockage is not None:
                        route_following = False
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
                        continue
                    action = self._waypoint_follow_motion_action(
                        motion_executor.pose,
                        active_waypoint,
                        available_actions,
                        mission=map_trace.mission,
                    )
                    if action is None:
                        route_following = False
                        geometry = self._active_waypoint_geometry(
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
                        continue
                    bounded_no_valid = (
                        action in (TURN_LEFT_90, TURN_RIGHT_90)
                        and turns_available
                        or action == ADVANCE
                        and self._recent_evidence_allows_bounded_advance(
                            history, latest_scan_view,
                        )
                    )
                    try:
                        observation, outcome = (
                            self._fresh_planner_observation_or_stop(
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
                        route_following = False
                        context.publish({
                            "current_action": None,
                            "plan": list(plan),
                        })
                        continue
                    if outcome is not None:
                        return outcome
                    step["action"] = action
                    step["observation"] = observation
                    step["bounded_no_valid_eligible"] = bounded_no_valid
                selected_turn_continuation = None
                selected_turn_alignment = None
                selected_turn_trigger_deg = _WAYPOINT_ALIGNMENT_TRIGGER_DEG
                if action in (TURN_LEFT_90, TURN_RIGHT_90):
                    if active_waypoint is not None:
                        selected_turn_alignment = self._waypoint_turn_alignment(
                            motion_executor.pose,
                            active_waypoint,
                            mission=map_trace.mission,
                        )
                    if (
                        selected_turn_alignment is None
                        and (
                            map_trace.mission.longitudinal_progress_mm(
                                motion_executor.pose
                            ) >= map_trace.mission.minimum_forward_progress_mm
                            or map_trace.mission.distance_to_target_mm(
                                motion_executor.pose
                            ) <= BLAST_GOAL_RADIUS_MM
                        )
                    ):
                        selected_turn_alignment = self._desired_heading_turn_alignment(
                            map_trace.mission.reference_heading_mdeg / 1_000,
                            motion_executor.pose,
                        )
                        selected_turn_trigger_deg = (
                            BLAST_GOAL_HEADING_TOLERANCE_MDEG / 1_000
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
                ) = self._dispatch_episode_action(
                    action=action,
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
