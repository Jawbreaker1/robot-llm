"""EV3 sensing/movement boundary for shared planning, without navigation policy."""

from copy import deepcopy
from math import ceil
from time import monotonic, time_ns

from .ev3_navigation_transport import EV3NavigationOperationError, EV3NavigationRemoteError
from .physical_navigation_contract import ADVANCE, EXPECTED_ACTION_SPECS, SCAN_FRONT_ARC, SCAN_SAMPLE_OPERATION, SCAN_TURN_OPERATION, expected_scan_turn_spec
from .waypoint_geometry import WAYPOINT_REACHED_RADIUS_MM, measured_motion_feedback
from .physical_navigation_execution_contract import EV3NavigationExecutionContract
from .physical_odometry import (
    PhysicalPose,
    apply_encoder_observation_delta,
    apply_verified_motion,
    normalize_heading_mdeg,
    verified_motion_from_result,
)


class EV3NavigationBody:
    """Use the same worker receipts on SSH and in the shared-world simulator."""

    FRONT_SCAN_BEARINGS_DEG = tuple(range(-60, 61, 15))

    def __init__(self, transport_factory, *, calibration, cancel_requested=lambda: False, clock=monotonic,
                 publish=lambda event: None):
        self.transport = None
        self.calibration = calibration
        self.cancel_requested = cancel_requested
        self.publish = publish
        self.ir_observations = []
        self.transport_factory, self.clock = transport_factory, clock
        self._started = False
        self.pose = PhysicalPose()
        self.observation = None
        self.drive_roles = None
        self._last_drive_motor_positions = None
        self.survey = None
        self.last_motion = None
        self._contract = EV3NavigationExecutionContract()
        drive = EXPECTED_ACTION_SPECS[ADVANCE]
        pulse_mm = drive["left_speed_dps"] * drive["total_duration_ms"] / 1_000 * calibration.linear_mm_per_encoder_degree
        self.advance_step_mm = round(pulse_mm)
        # The worker has fixed pulses: arrival must allow at least half a pulse,
        # otherwise some targets fall between two physically reachable stops.
        self.waypoint_radius_mm = max(WAYPOINT_REACHED_RADIUS_MM, ceil(pulse_mm / 2))

    def _request(self, operation, arguments=None):
        if not self._started:
            self.start()
        arguments = arguments or {}
        spec = (expected_scan_turn_spec(arguments["relative_delta_mdeg"]) if operation == SCAN_TURN_OPERATION else
                EXPECTED_ACTION_SPECS[arguments["action"]] if operation == "pulse" else {})
        budgets = self.observation["budgets"]
        exhausted = (min(budgets["process_ms_remaining"], (self._deadline - self.clock()) * 1_000) < 30_000
                     or self._requests_remaining <= 2
                     or budgets["pulse_count_remaining"] < spec.get("slice_count", 0)
                     or budgets["pulse_duration_ms_remaining"] < spec.get("total_duration_ms", 0))
        if exhausted and not budgets["motion_fault_latched"]:
            # Replace the bounded worker only while stopped, never the mission,
            # pose or route. Never replay a request whose result was lost.
            self.close()
            self.start()
        self._requests_remaining -= 1
        return self.transport.request(operation, arguments, 30.0, cancel_requested=self.cancel_requested)

    def start(self):
        if self._started:
            return
        # The episode owns retries. Reopening reads absolute counters without
        # resetting the pose or replaying the previous request.
        self.transport = self.transport_factory()
        try:
            self.transport.start()
            description = self.transport.request("describe", {}, 60.0, cancel_requested=self.cancel_requested)
            observation, _, self.drive_roles, lifetime_ms = self._contract.parse_description(description)
            self._accept_observation(observation)
            self._deadline = self.clock() + min(lifetime_ms, self.observation["budgets"]["process_ms_remaining"]) / 1_000
            self._requests_remaining = description["result"]["process"]["max_requests"] - 1
            self._started = True
        except Exception:
            self.disconnect()
            raise

    def disconnect(self):
        """Discard an uncertain channel, not the mission's measured state."""
        self._started = False
        if self.transport is not None:
            self.transport.abort()
            self.transport.close()

    def _drive_motor_positions(self, observation=None):
        positions = {
            motor["role"]: motor["position"]
            for motor in (self.observation if observation is None else observation)["motors"]
        }
        try:
            return {
                "left": positions[self.drive_roles.left],
                "right": positions[self.drive_roles.right],
            }
        except KeyError as error:
            raise RuntimeError("Worker observation lacks a drive motor") from error

    def _accept_observation(self, observation, motion=None):
        """Commit encoder evidence once, in before/command/after order."""
        current = self._drive_motor_positions(observation)
        previous = self._last_drive_motor_positions
        before = self.pose
        pose = before
        if motion is not None and motion.encoder_start is not None:
            left, right = motion.encoder_start
            if previous is not None:
                pose = apply_encoder_observation_delta(
                    pose, left - previous["left"], right - previous["right"], self.calibration)
            pose = apply_verified_motion(pose, motion, self.calibration)
            previous = {"left": left + motion.left_encoder_delta_degrees,
                        "right": right + motion.right_encoder_delta_degrees}
        if previous is not None:
            pose = apply_encoder_observation_delta(
                pose, current["left"] - previous["left"], current["right"] - previous["right"], self.calibration)
        # Nothing is published or remembered until the entire update succeeds.
        self.pose, self.observation, self._last_drive_motor_positions = pose, observation, current
        if motion is None and previous is not None and current != previous:
            self.publish({"event": "encoder_continuity", "pose_before": before.to_dict(),
                          "pose": pose.to_dict(),
                          "left_encoder_delta_degrees": current["left"] - previous["left"],
                          "right_encoder_delta_degrees": current["right"] - previous["right"]})

    def _record_observation(self, sample=None):
        ir = self.observation["infrared"]
        sample = sample if sample is not None else {
            "raw_ir_proximity": ir["raw"], "filtered_ir_proximity": ir["filtered"],
            "near_obstacle": ir["blocked"] if ir["filtered"] is not None else None}
        item = {"pose": self.pose.to_dict(), **sample}
        previous = self.ir_observations[-1] if self.ir_observations else None
        if previous is None or item != {k: v for k, v in previous.items() if k != "observed_at_unix_ms"}:
            self.ir_observations.append({**item, "observed_at_unix_ms": time_ns() // 1_000_000})
            del self.ir_observations[:-128]
        self.publish({"event": "observation", "pose": self.pose.to_dict(),
                      "infrared": deepcopy(ir)})

    def execute(self, action, *, turn_mdeg=None):
        """Apply measured motion, including a stopped partial result, exactly once."""
        operation = "pulse" if turn_mdeg is None else SCAN_TURN_OPERATION
        arguments = {"action": action} if turn_mdeg is None else {"relative_delta_mdeg": turn_mdeg}
        error_code = None
        try:
            response = self._request(operation, arguments)
        except EV3NavigationOperationError as error:
            response = {"state_version": error.observation["state_version"], "result": error.result}
            error_code = error.code
        observation = self._contract.parse_observation(operation, response, expected_action=action)
        motion = verified_motion_from_result(
            action if operation == "pulse" else operation, response["result"], self.drive_roles,
        )
        before = self.pose
        self._accept_observation(observation, motion)
        completed = motion.complete and error_code is None
        reason = error_code or response["result"]["outcome"].get("reason")
        turning = turn_mdeg is not None or action in ("TURN_LEFT_90", "TURN_RIGHT_90")
        self.last_motion = measured_motion_feedback(
            action, before, self.pose, completed=completed, reason=reason,
            requested_turn_deg=(turn_mdeg / 1_000 if turn_mdeg is not None else
                                (90 if action == "TURN_LEFT_90" else -90) if turning else None),
        )
        self._record_observation()
        return {"completed": completed,
                "error_code": error_code,
                "reason": response["result"]["outcome"].get("reason"),
                "pose": self.pose.to_dict()}

    def observe(self):
        self._accept_observation(self._contract.parse_observation("observe", self._request("observe")))
        self._record_observation()
        return self.observation

    def sample(self):
        """Refresh one settled ray without turning or discarding the survey."""
        try:
            response = self._request(SCAN_SAMPLE_OPERATION)
            self._accept_observation(self._contract.parse_observation(SCAN_SAMPLE_OPERATION, response))
            ir = self.observation["infrared"]
            sample = {"raw_ir_proximity": ir["raw"], "filtered_ir_proximity": ir["filtered"],
                      "near_obstacle": ir["blocked"] if ir["filtered"] is not None else None}
        except EV3NavigationRemoteError as error:
            if error.fatal or not error.stop or error.stop.get("stop_confirmed") is not True:
                raise
            sample = {"raw_ir_proximity": None, "filtered_ir_proximity": None,
                      "near_obstacle": None, "error_code": error.code}
        self._record_observation(sample)
        return {"pose": self.pose.to_dict(), "state_version": self.observation["state_version"], **sample}

    def scan(self, *, full=True):
        """Survey 0..360 or inspect a denser front sector, using actual encoders."""
        if self.drive_roles is None:
            raise RuntimeError("Start EV3 sensing before requesting a scan")
        start_pose = self.pose
        targets = range(0, 361, 30) if full else (*self.FRONT_SCAN_BEARINGS_DEG, 0)
        samples = []
        reason = None
        # A full survey is a physical 360-degree sweep, not a heading
        # correction loop.  Keep it deliberately boring: twelve equal
        # 30-degree commands.  Recomputing a rounded residual from encoder
        # feedback can turn one step into a 45-degree correction (and, with
        # the worker's 800 ms slice limit, two visible pulses), which makes a
        # nominal 360-degree scan overshoot.  The encoder-backed pose remains
        # authoritative for the resulting heading; it is not used to invent
        # another turn during the sweep.
        turned_mdeg = 0
        # Use unwrapped measured extent: a partial sweep is not 120/360 degrees,
        # and a restored full sweep is not zero just because its heading matches.
        swept_min_mdeg = swept_max_mdeg = 0
        for target in targets:
            if self.cancel_requested():
                reason = "cancelled"
                break
            if full:
                command = 0 if target == 0 else 30_000
            else:
                delta = target * 1_000 - turned_mdeg
                # The worker's existing motor profile supports 15-degree increments.
                # Correct accumulated error toward the next target, never reset the pose.
                command = round(delta / 15_000) * 15_000
            if command:
                if abs(command) > 120_000:
                    reason = "turn_progress_incomplete"
                    break
                before_heading = self.pose.heading_mdeg
                result = self.execute(SCAN_TURN_OPERATION, turn_mdeg=command)
                turned_mdeg += normalize_heading_mdeg(self.pose.heading_mdeg - before_heading)
                swept_min_mdeg = min(swept_min_mdeg, turned_mdeg)
                swept_max_mdeg = max(swept_max_mdeg, turned_mdeg)
                if not result["completed"]:
                    reason = result["error_code"] or "turn_progress_incomplete"
                    break
            samples.append({**self.sample(), "requested_bearing_deg": target})
        self.survey = {"sensor": "reflected_infrared", "units": "unitless_proximity",
                       "sweep_degrees": round((swept_max_mdeg - swept_min_mdeg) / 1_000, 1),
                       "metric_distance_available": False, "complete": reason is None,
                       "reason": reason, "start_pose": start_pose.to_dict(),
                       "end_pose": self.pose.to_dict(), "samples": samples}
        self.last_motion = measured_motion_feedback(
            SCAN_FRONT_ARC, start_pose, self.pose, completed=self.survey["complete"], reason=reason,
            scan_coverage_deg=self.survey["sweep_degrees"],
        )
        self.publish({"event": "survey", "complete": self.survey["complete"], "sweep_degrees": self.survey["sweep_degrees"]})
        return deepcopy(self.survey)

    def close(self):
        if not self._started:
            return
        try:
            if not self._contract.shutdown_verified(self.transport.request("shutdown", {}, 30.0)):
                raise RuntimeError("EV3 worker shutdown was not confirmed")
        except Exception:
            self.transport.abort()
            raise
        finally:
            self._started = False
            self.transport.close()
