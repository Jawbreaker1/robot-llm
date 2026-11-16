"""EV3 execution of the shared model's route, independent of SSH or simulation.

The caller owns mission lifetime, cancellation and presentation. This session
never authors a detour and never converts IR proximity to obstacle coordinates.
"""

from copy import deepcopy
from .coarse_navigation_grid import model_route_blockage

from .ev3_navigation_context import ev3_navigation_context
from .ev3_navigation_transport import EV3NavigationTransportError
from .lm_studio import LMStudioTransportError, LMStudioProtocolError
from .lm_studio_controller_action import COMPLETE, FOLLOW_WAYPOINT
from .physical_navigation_contract import ADVANCE, OBSERVE, SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90
from .physical_odometry import normalize_heading_mdeg
from .waypoint_geometry import (
    consume_reached_waypoints, goal_reached, waypoint_target_heading_mdeg,
    waypoint_follow_motion_action, waypoint_geometry, waypoint_reached,
)


class EV3NavigationSession:
    def __init__(self, *, body, planner, descriptor, footprint, mission, goal,
                 locale="en", publish=lambda event: None):
        self.body, self.planner = body, planner
        self.descriptor, self.footprint = descriptor, footprint
        self.mission, self.goal, self.locale = mission, goal, locale
        self.publish = publish
        self.waypoints = ()
        self.obstacle_estimates = ()
        self.previous_decision = None
        self.history = []
        self.completed = False

    def context(self):
        return ev3_navigation_context(
            body=self.body, descriptor=self.descriptor, footprint=self.footprint,
            mission=self.mission, goal=self.goal, locale=self.locale,
            waypoints=self.waypoints, history=self.history,
            obstacle_estimates=self.obstacle_estimates,
            previous_decision=self.previous_decision,
        )

    def _follow(self, waypoint):
        """Finish alignment or one forward pulse; retain the model-owned route."""
        motions = 0
        turned = False
        while not self.body.cancel_requested():
            observation = self.body.observe()
            at_position = waypoint_reached(self.body.pose, waypoint, reached_radius_mm=self.body.waypoint_radius_mm)
            if at_position:
                return "waypoint_reached", motions
            if goal_reached(self.mission, self.body.pose):
                return "goal_reached", motions
            available = (TURN_LEFT_90, TURN_RIGHT_90)
            if not observation["infrared"]["blocked"] and not observation["touch"]["pressed"]:
                available += (ADVANCE,)
            action = waypoint_follow_motion_action(self.body.pose, waypoint, available,
                                                   reached_radius_mm=self.body.waypoint_radius_mm)
            if observation["touch"]["pressed"]:
                return "touch_pressed", motions
            if action is None:
                ir = observation["infrared"]
                if not ir["blocked"]:
                    return "action_unavailable", motions
                unverified = ir["reason"] in ("unverified_startup", "warming_up", "invalid_sample")
                return "infrared_unverified" if unverified else "near_obstacle", motions
            if action == ADVANCE and turned:
                # IR cannot validate the next leg while still facing sideways.
                # The observation above belongs to the completed turn, not the
                # pre-turn pose. Keep the route and let its owner review it.
                return "heading_aligned", motions
            before = self.body.pose
            if action in (TURN_LEFT_90, TURN_RIGHT_90):
                heading = waypoint_target_heading_mdeg(before, waypoint)
                delta = normalize_heading_mdeg(heading - before.heading_mdeg)
                turn = max(-120_000, min(120_000, round(delta / 15_000) * 15_000))
                result = self.body.execute(action, turn_mdeg=turn)
                # Review a newly faced leg, not each feedback correction after
                # translation on the already-approved leg has begun.
                turned = turned or motions == 0
            else:
                result = self.body.execute(action)
            motions += 1
            self.publish({"event": "motion", "action": action, "pose": self.body.pose.to_dict(),
                          "waypoints": deepcopy(self.waypoints), "result": result})
            if not result["completed"]:
                return result["error_code"] or result["reason"] or "partial_motion", motions
            if (self.body.pose.x_mm, self.body.pose.y_mm, self.body.pose.heading_mdeg) == (before.x_mm, before.y_mm, before.heading_mdeg):
                return "no_measured_progress", motions
            # A fixed worker pulse can pass a small target. Return measured
            # progress to Qwen rather than oscillating back and forth forever.
            if action == ADVANCE and waypoint_geometry(self.body.pose, waypoint)["distance_mm"] > waypoint_geometry(before, waypoint)["distance_mm"]:
                return "waypoint_passed", motions
            if action == ADVANCE:
                if goal_reached(self.mission, self.body.pose):
                    return "goal_reached", motions
                if waypoint_reached(self.body.pose, waypoint, reached_radius_mm=self.body.waypoint_radius_mm):
                    return "waypoint_reached", motions
                return "step_completed", motions
        return "cancelled", motions

    def step(self):
        self._action_in_flight = None
        try:
            return self._step()
        except EV3NavigationTransportError as error:
            self.body.disconnect()
            event = {"event": "transport_retry", "action": self._action_in_flight,
                     "result": getattr(error, "code", "worker_transport_error"),
                     "operation": getattr(error, "operation", None),
                     "motion_outcome": "unconfirmed; observe absolute encoders before further motion",
                     "pose": self.body.pose.to_dict(), "waypoint_plan": deepcopy(self.waypoints)}
            self.history.append(event)
            self.publish(event)
            return event

    def _step(self):
        if self.completed or self.body.cancel_requested():
            return None
        self.body.observe()  # Reconnect/reconcile before planning, never replay.
        if self.body.survey is None:
            self.body.scan()
        self.waypoints, _ = consume_reached_waypoints(self.body.pose, self.waypoints,
                                                    reached_radius_mm=self.body.waypoint_radius_mm)
        context = self.context()
        try:
            result = self.planner.decide(context)
        except (LMStudioTransportError, LMStudioProtocolError) as error:
            # No motor command has been issued. Retry from the same measured
            # state; provider trouble is not new evidence about the room.
            event = {"event": "planner_retry", "error": type(error).__name__,
                     "pose": self.body.pose.to_dict(), "waypoints": deepcopy(self.waypoints)}
            self.publish(event)
            return event
        decision = result.decision
        if self.body.cancel_requested():
            return None
        self._action_in_flight = decision.action
        self.previous_decision = decision.memory()
        if decision.obstacle_estimates is not None:
            self.obstacle_estimates = tuple(deepcopy(decision.obstacle_estimates))
        if decision.waypoint is not None:
            self.waypoints = tuple(deepcopy((decision.waypoint, *decision.following_waypoints)))
        before = self.body.pose
        self.publish({"event": "decision", "action": decision.action, "assessment": decision.assessment,
                      "utterance": decision.utterance, "plan": list(decision.plan),
                      "reasoning": result.reasoning_content, "latency_ms": result.latency_ms,
                      "waypoints": deepcopy(self.waypoints),
                      "obstacle_estimates": deepcopy(self.obstacle_estimates), "context": context.to_dict()})
        self.body.last_motion = None
        motions = 0
        action_completed = False
        observation_acquired = None
        route_rejection = model_route_blockage(
            start=(self.body.pose.x_mm, self.body.pose.y_mm),
            waypoints=tuple((item["x_mm"], item["y_mm"]) for item in self.waypoints),
            possible_obstacles=(),  # IR is not a metric obstacle map.
        ) if decision.action == FOLLOW_WAYPOINT else None
        if decision.action == COMPLETE:
            self.completed = context.completion_allowed
            reason = "goal_reached" if self.completed else "goal_not_reached"
            action_completed = self.completed
        elif decision.action == SCAN_FRONT_ARC:
            survey = self.body.scan(full=False)
            reason = survey["reason"] or "survey_completed"
            action_completed = survey["complete"]
            observation_acquired = bool(survey["samples"]) and all(
                not sample.get("error_code") for sample in survey["samples"])
        elif decision.action == OBSERVE:
            sample = self.body.sample()
            action_completed = observation_acquired = not bool(sample.get("error_code"))
            reason = sample.get("error_code") or "observation_refreshed"
        elif decision.action == FOLLOW_WAYPOINT:
            reason, motions = (("route_rejected", 0) if route_rejection is not None else
                               self._follow(self.waypoints[0]) if self.waypoints else ("waypoint_required", 0))
            action_completed = reason in ("waypoint_reached", "goal_reached", "heading_aligned", "step_completed")
        elif decision.action in context.available_actions:
            receipt = self.body.execute(decision.action)
            motions = 1
            reason = receipt["error_code"] or receipt["reason"]
            action_completed = receipt["completed"]
        else:
            reason = "action_unavailable"
        event = {"action": decision.action, "result": reason, "motion_attempts": motions,
                 "action_completed": action_completed, "observation_acquired": observation_acquired,
                 "last_motion": deepcopy(self.body.last_motion),
                 "pose_before": before.to_dict(), "pose": self.body.pose.to_dict(),
                 "waypoint_plan": deepcopy(self.waypoints),
                 "active_waypoint_geometry_after": waypoint_geometry(self.body.pose, self.waypoints[0]) if self.waypoints else None}
        if route_rejection is not None:
            event["route_rejection"] = route_rejection
        self.history.append(event)
        self.publish({"event": "result", **event})
        return event
