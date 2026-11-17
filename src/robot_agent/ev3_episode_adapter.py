"""EV3 application wiring: shared planner, map, speech and cancellable body."""

import logging
import threading
import time

from .episode_speech import EpisodeSpeech, episode_cancelled
from .ev3_navigation_session import EV3NavigationSession
from .ev3_navigation_transport import EV3NavigationTransportError
from .navigation_diagnostics import record_navigation_diagnostic
from .navigation_map_trace import waypoint_route_trace
from .physical_navigation_mission import DirectionalMission
from .robot_control_service import RobotEpisodeOutcome

LOGGER = logging.getLogger(__name__)


class EV3EpisodeAdapter:
    def __init__(self, *, body_factory, planner_factory, descriptor, footprint,
                 spatial_map_provider, speech_runtime_factory, controller_lease,
                 minimum_forward_progress_mm=800, clock=time.monotonic, event_sink=lambda event: None):
        self.body_factory, self.planner_factory = body_factory, planner_factory
        self.descriptor, self.footprint = descriptor, footprint
        self.spatial_map_provider = spatial_map_provider
        self.speech_runtime_factory, self.controller_lease = speech_runtime_factory, controller_lease
        self.minimum_forward_progress_mm, self.clock = minimum_forward_progress_mm, clock
        self.event_sink = event_sink
        self._stop = threading.Event()

    def request_stop(self):
        self._stop.set()

    emergency_stop = request_stop

    def run(self, context):
        self._stop.clear()
        duration_ms = context.settings.max_episode_ms
        deadline = None if duration_ms is None else self.clock() + duration_ms / 1_000
        expired = lambda: deadline is not None and self.clock() >= deadline
        cancelled = lambda: self._stop.is_set() or episode_cancelled(context) or expired()
        # A just-started mission waits for the current short arm command or
        # preflight to finish; an accessory must not make navigation fail.
        while not self.controller_lease.acquire(timeout=0.05):
            if cancelled():
                return RobotEpisodeOutcome("stopped", False, {"current_action": "STOP"})
        body = session = mission = planner = None
        finale = None
        speech = EpisodeSpeech(factory=self.speech_runtime_factory, supported_locales=("sv", "en"), context=context)

        def publish(event):
            nonlocal finale
            record_navigation_diagnostic("ev3_episode", episode_id=context.episode_id, payload=event)
            self.event_sink(event)
            if event["event"] == "transport_retry":
                context.publish({"current_action": None,
                                 "message": "Reconnecting to EV3; keeping the current goal and route."})
            if body is None or mission is None:
                return
            route = session.waypoints if session is not None else ()
            try:
                self.spatial_map_provider.offer_navigation(body=body, mission=mission, waypoints=route, footprint=self.footprint,
                    obstacle_estimates=session.obstacle_estimates if session is not None else ())
            except Exception:
                LOGGER.exception("EV3 map publication failed")
            kind = event["event"]
            update = {}
            if kind == "decision":
                update = {"current_action": event["action"], "message": event["assessment"],
                          "model_latency_ms": event["latency_ms"], "plan": event["plan"],
                          "active_route": waypoint_route_trace(mission, body.pose, route)}
                if event["action"] == "COMPLETE":
                    speech.end_commentary()
                    finale = event
                elif not speech.observe(event["context"], commentator=getattr(planner, "comment", None),
                                        progress_revision=len(session.history) + 1):
                    speech.offer(event.get("utterance"), progress_revision=len(session.history) + 1)
            elif kind == "survey":
                update = {"scan": {"complete": event["complete"], "sweep_degrees": event["sweep_degrees"]}}
            elif kind == "result":
                update = {"message": event["result"], "active_route": waypoint_route_trace(mission, body.pose, route)}
            elif kind == "planner_retry":
                update = {"message": "Waiting for the planner; keeping the current goal and route."}
            if update:
                context.publish(update)

        try:
            body = self.body_factory(cancel_requested=cancelled, publish=publish)
            while not cancelled():
                try:
                    body.start()
                    break
                except EV3NavigationTransportError as error:
                    publish({"event": "transport_retry", "result": getattr(error, "code", "worker_transport_error")})
                    self._stop.wait(1)
            if cancelled():
                return RobotEpisodeOutcome("stopped", False, {"current_action": "STOP"})
            mission = DirectionalMission.begin(episode_id=context.episode_id, pose=body.pose,
                minimum_forward_progress_mm=self.minimum_forward_progress_mm, heading_tolerance_mdeg=20_000)
            planner = self.planner_factory(context.settings.model)
            session = EV3NavigationSession(body=body, planner=planner,
                descriptor=self.descriptor, footprint=self.footprint, mission=mission,
                goal=context.request.goal, locale=context.request.locale, publish=publish)
            publish({"event": "started"})
            context.publish({"current_action": "SCAN_360", "message": "Surveying the surroundings before navigation."})
            speech.start()
            while not cancelled() and not session.completed:
                event = session.step()
                if event and event.get("event") in ("planner_retry", "transport_retry"):
                    self._stop.wait(1)
            reason = ("completed" if session.completed else "emergency_stopped" if context.emergency_stop_requested.is_set()
                      else "time_limit" if expired() else "stopped")
            return RobotEpisodeOutcome(reason, session.completed, {
                "current_action": "COMPLETE" if session.completed else "STOP", "active_route": None,
                "message": "Goal reached." if session.completed else "Episode stopped; measured progress retained on the map."})
        finally:
            stopped = False
            try:
                if body is not None:
                    body.close()
                    stopped = True
            finally:
                self.controller_lease.release()
                # The existing social speaker owns the arm only after the drive
                # worker has confirmed shutdown. Never celebrate a proposed or
                # cancelled completion, or an unconfirmed motor stop.
                completed = stopped and session is not None and session.completed and not cancelled()
                if completed and finale is not None:
                    speech.offer(finale.get("utterance"), progress_revision=len(session.history) + 1,
                                 expression=finale.get("expression"))
                speech.close(drain=completed)
