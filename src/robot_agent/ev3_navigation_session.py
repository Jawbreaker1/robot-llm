"""EV3 execution of the shared model's route, independent of SSH or simulation.

The caller owns mission lifetime, cancellation and presentation. This session
never authors a detour and never converts IR proximity to obstacle coordinates.
"""

from copy import deepcopy

from .ev3_navigation_context import ev3_navigation_context
from .lm_studio import LMStudioTransportError, LMStudioProtocolError
from .lm_studio_controller_action import COMPLETE, FOLLOW_WAYPOINT
from .physical_navigation_contract import ADVANCE, SCAN_FRONT_ARC, TURN_LEFT_90, TURN_RIGHT_90
from .physical_odometry import normalize_heading_mdeg
from .waypoint_geometry import (
    consume_reached_waypoints, waypoint_axis_heading_mdeg,
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
        self.history = []
        self.completed = False

    def context(self):
        return ev3_navigation_context(
            body=self.body, descriptor=self.descriptor, footprint=self.footprint,
            mission=self.mission, goal=self.goal, locale=self.locale,
            waypoints=self.waypoints, history=self.history,
        )

    def _follow(self, waypoint):
        """Finish one selected leg; refresh sensors between bounded worker pulses."""
        motions = 0
        while not self.body.cancel_requested():
            at_position = waypoint_reached(self.body.pose, waypoint, mission=self.mission, reached_radius_mm=self.body.waypoint_radius_mm)
            final_alignment = (at_position and (waypoint["x_mm"], waypoint["y_mm"]) == self.mission.target_point()
                               and not self.mission.heading_aligned(self.body.pose))
            if at_position and not final_alignment:
                return "waypoint_reached", motions
            observation = self.body.observe()
            available = (TURN_LEFT_90, TURN_RIGHT_90)
            if not observation["infrared"]["blocked"] and not observation["touch"]["pressed"]:
                available += (ADVANCE,)
            action = waypoint_follow_motion_action(self.body.pose, waypoint, available, mission=self.mission,
                                                   reached_radius_mm=self.body.waypoint_radius_mm)
            if final_alignment:
                delta = normalize_heading_mdeg(self.mission.reference_heading_mdeg - self.body.pose.heading_mdeg)
                action = TURN_LEFT_90 if delta > 0 else TURN_RIGHT_90
            if action is None or observation["touch"]["pressed"]:
                return "near_obstacle", motions
            before = self.body.pose
            if action in (TURN_LEFT_90, TURN_RIGHT_90):
                heading = self.mission.reference_heading_mdeg if final_alignment else waypoint_axis_heading_mdeg(before, waypoint)
                delta = normalize_heading_mdeg(heading - before.heading_mdeg)
                turn = max(-120_000, min(120_000, round(delta / 15_000) * 15_000))
                result = self.body.execute(action, turn_mdeg=turn)
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
        return "cancelled", motions

    def step(self):
        if self.completed or self.body.cancel_requested():
            return None
        if self.body.survey is None:
            self.body.scan()
        else:
            self.body.observe()
        self.waypoints, _ = consume_reached_waypoints(self.body.pose, self.waypoints, mission=self.mission,
                                                    reached_radius_mm=self.body.waypoint_radius_mm)
        context = self.context()
        self.body.release_for_planning()
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
        if decision.waypoint is not None:
            self.waypoints = tuple(deepcopy((decision.waypoint, *decision.following_waypoints)))
        before = self.body.pose
        self.publish({"event": "decision", "action": decision.action, "assessment": decision.assessment,
                      "utterance": decision.utterance, "plan": list(decision.plan),
                      "reasoning": result.reasoning_content, "latency_ms": result.latency_ms,
                      "waypoints": deepcopy(self.waypoints), "context": context.to_dict()})
        self.body.last_motion = None
        motions = 0
        if decision.action == COMPLETE:
            self.completed = context.completion_allowed
            reason = "goal_reached" if self.completed else "goal_not_reached"
        elif decision.action == SCAN_FRONT_ARC:
            survey = self.body.scan(full=False)
            reason = survey["reason"] or "survey_completed"
        elif decision.action == FOLLOW_WAYPOINT:
            reason, motions = self._follow(self.waypoints[0]) if self.waypoints else ("waypoint_required", 0)
        elif decision.action in context.available_actions:
            receipt = self.body.execute(decision.action)
            motions = 1
            reason = receipt["error_code"] or receipt["reason"]
        else:
            reason = "action_unavailable"
        event = {"action": decision.action, "result": reason, "motion_pulses": motions,
                 "last_motion": deepcopy(self.body.last_motion),
                 "pose_before": before.to_dict(), "pose": self.body.pose.to_dict(),
                 "waypoint_plan": deepcopy(self.waypoints),
                 "active_waypoint_geometry_after": waypoint_geometry(self.body.pose, self.waypoints[0]) if self.waypoints else None}
        self.history.append(event)
        self.publish({"event": "result", **event})
        return event
