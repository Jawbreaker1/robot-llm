"""Finite fault fixtures for simulator data collection, never robot control.

Faults change simulated measurements, motor progress, or drop one transmitted
waypoint. A corrupted command is explicitly NOT a teacher training label.
Production adapters construct the feedback; no rejection packet is fabricated.
Every patch is scoped to one simulation subprocess and restored afterwards.
"""
from contextlib import ExitStack
from dataclasses import replace
from unittest.mock import patch

from robot_agent.coarse_navigation_grid import model_route_blockage
from robot_agent.multi_robot_navigation_simulator import MultiRobotNavigationSimulator
from robot_agent.simulation_robot_adapters import SharedWorldBlastController, SharedWorldEV3Transport

FAULTS = ("stall", "range", "scan", "route")


def feedback_kinds(packet):
    """Audit/data-selection labels only; never consulted by the action policy."""
    kinds = set()
    observation = packet.get("observation") or {}
    evidence = packet.get("local_map_evidence") or {}
    motion = observation.get("last_motion") or {}
    if motion.get("completed") is False:
        kinds.add("partial_scan" if motion.get("kind") == "scan" else "motion_interrupted")
    if evidence.get("latest_route_rejection"):
        kinds.add("route_rejected")
    if "ir_survey" in evidence:
        ir = observation.get("infrared") or {}
        if "raw" in ir and (ir["raw"] is None or ir["raw"] == 100):
            kinds.add("range_missing")
        if (evidence.get("ir_survey") or {}).get("complete") is False:
            kinds.add("partial_scan")
    elif (observation.get("sensors") or {}).get("distance_mm") is None:
        kinds.add("range_missing")
    for item in (packet.get("history") or [])[-1:]:
        if item.get("result") in ("near_obstacle", "infrared_blocked", "no_measured_progress"):
            kinds.add("motion_interrupted")
        if item.get("route_rejection"):
            kinds.add("route_rejected")
    return kinds


class RecoveryFault:
    def __init__(self, kind="none", emit=lambda event: None, *, after_mm=70, missing_reads=32, stalled_attempts=1):
        if kind not in ("none", *FAULTS):
            raise ValueError("Unknown simulator fault")
        if not 1 <= missing_reads <= 256 or not 1 <= stalled_attempts <= 5 or after_mm < 0:
            raise ValueError("Fault duration/trigger outside the bounded experiment")
        self.kind, self.emit = kind, emit
        self.after_mm, self.missing_reads = after_mm, missing_reads
        self.travel_mm = 0
        self.action = None
        self.scan_turn_mdeg = 0
        self.triggered = False
        self.missing_remaining = 0
        self.stalls_remaining = stalled_attempts
        self.feedback = set()

    def before_action(self, action):
        self.action = action
        self.scan_turn_mdeg = 0

    def transmitted_decision(self, decision, packet):
        """Drop one corner once, then let the real route validator reject it.

        A test fixture for a damaged/stale route, not a recovery policy. Never
        inject a shortcut the validator would accept and physically execute.
        """
        if (self.kind != "route" or self.triggered or decision.action != "FOLLOW_WAYPOINT"
                or decision.waypoint is None or not decision.following_waypoints):
            return decision
        pose = packet["local_map_evidence"]["robot_pose"]
        shortened = decision.following_waypoints
        rejection = model_route_blockage(start=(pose["x_mm"], pose["y_mm"]),
            waypoints=tuple((p["x_mm"], p["y_mm"]) for p in shortened), possible_obstacles=())
        if not rejection or rejection["reason"] != "NON_ORTHOGONAL_ROUTE_LEG":
            return decision
        self._trigger()
        self.emit({"event":"dropped_route_corner", "original_waypoint":decision.waypoint,
                   "transmitted_waypoint":shortened[0], "teacher_label_excluded":True})
        return replace(decision, waypoint=shortened[0], following_waypoints=shortened[1:])

    def observe(self, packet):
        kinds = feedback_kinds(packet)
        if self.triggered or self.kind == "none":
            self.feedback.update(kinds)
        return sorted(kinds)

    def _trigger(self):
        self.triggered = True
        self.emit({"event":"injected_fault", "kind":self.kind,
                   "after_measured_travel_mm":self.travel_mm, "action":self.action})

    def _missing(self):
        if self.kind == "range" and not self.triggered and self.travel_mm >= self.after_mm:
            self._trigger()
            self.missing_remaining = self.missing_reads
        if self.missing_remaining:
            self.missing_remaining -= 1
            return True
        return False

    @property
    def observed(self):
        expected = {"stall":"motion_interrupted", "range":"range_missing", "scan":"partial_scan",
                    "route":"route_rejected"}
        return bool(self.feedback) if self.kind == "none" else self.triggered and expected[self.kind] in self.feedback

    def __enter__(self):
        self.stack = ExitStack()
        move = MultiRobotNavigationSimulator.move
        rotate = MultiRobotNavigationSimulator.rotate
        distance = SharedWorldBlastController._distance_mm
        infrared = SharedWorldEV3Transport._ir_value

        def measured_move(world, robot_id, distance_mm):
            if self.kind == "stall" and self.stalls_remaining and self.travel_mm >= self.after_mm and distance_mm:
                if not self.triggered:
                    self._trigger()
                self.stalls_remaining -= 1
                self.emit({"event":"injected_stall_attempt", "requested_mm":distance_mm,
                           "attempts_remaining":self.stalls_remaining})
                # Zero physical displacement => zero encoder change through the
                # existing adapter. This is a brief jam, not a hidden obstacle.
                world._record(robot_id, "injected_stall", 0)
                return 0
            actual = move(world, robot_id, distance_mm)
            self.travel_mm += abs(actual)
            return actual

        def measured_rotate(world, robot_id, delta_mdeg):
            scanning = self.action == "SCAN_FRONT_ARC"
            if (self.kind == "scan" and not self.triggered and scanning
                    and self.travel_mm >= self.after_mm and self.scan_turn_mdeg >= 10_000):
                self._trigger()
                world._record(robot_id, "injected_scan_interruption", 0)
                return world.pose(robot_id)
            actual = rotate(world, robot_id, delta_mdeg)
            if scanning:
                self.scan_turn_mdeg += abs(delta_mdeg)
            return actual

        def measured_distance(controller):
            return 2000 if self._missing() else distance(controller)

        def measured_ir(transport, clearance_mm):
            # EV3 reports weak/no IR reflection as 100, not a metric distance.
            # None would bypass its real sensor contract and fabricate a crash.
            return 100 if self._missing() else infrared(transport, clearance_mm)

        for owner, name, replacement in (
            (MultiRobotNavigationSimulator, "move", measured_move),
            (MultiRobotNavigationSimulator, "rotate", measured_rotate),
            (SharedWorldBlastController, "_distance_mm", measured_distance),
            (SharedWorldEV3Transport, "_ir_value", measured_ir),
        ):
            self.stack.enter_context(patch.object(owner, name, replacement))
        return self

    def __exit__(self, *args):
        return self.stack.__exit__(*args)
