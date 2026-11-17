"""Shared-context speech is optional, read-only and off the movement path."""
import copy
import threading
import unittest
from types import SimpleNamespace

from robot_agent.episode_speech import EpisodeSpeech
from robot_agent.robot_speech_runtime import RobotSpeechRuntime
from robot_agent.blast_navigation_simulation import run_blast_gemma_scenario
from robot_agent.lm_studio_controller_action import ControllerActionDecision, ControllerActionPlannerResult
from scripts.local_navigation_experiment import run_ev3, scenario


class ParallelNavigationSpeechTests(unittest.TestCase):
    def test_generation_and_playback_do_not_block_either_robot(self):
        for robot in ("ev3", "blast"):
            with self.subTest(robot=robot):
                generating, play_started = threading.Event(), threading.Event()
                release_generation, release_playback = threading.Event(), threading.Event()
                packets, received, spoken, expressions = [], [], [], []
                test = self
                class Planner:
                    def comment(self, packet):
                        received.append(copy.deepcopy(packet))
                        generating.set()
                        release_generation.wait(3)
                        return "I have checked the surroundings."

                    def decide(self, context):
                        packets.append(context.to_dict())
                        if len(packets) == 2:
                            # Navigation reached its next decision while text generation waited.
                            test.assertTrue(generating.wait(2), "generation never started")
                            release_generation.set()
                            test.assertTrue(play_started.wait(2), "playback never started")
                        complete = context.completion_allowed
                        if complete:
                            # The whole route also proceeded while the audio was held open.
                            test.assertTrue(play_started.is_set(), "no audio before completion")
                            test.assertFalse(release_playback.is_set(), "audio was released prematurely")
                            release_playback.set()
                        waypoint = ({"x_mm":800,"y_mm":0,"purpose":"Goal"}
                                    if len(packets) == 1 else None)
                        return ControllerActionPlannerResult(ControllerActionDecision(
                            "COMPLETE" if complete else "FOLLOW_WAYPOINT",1000,"Test",(),
                            "Made it!" if complete else None,waypoint=waypoint,
                            expression=({"face":"neutral","gesture":"saw_celebrate" if robot == "ev3" else "claw_flourish"}
                                        if complete else None)),0)

                def speaker(text, locale, cancel, *, expression=None):
                    spoken.append(text)
                    expressions.append(expression)
                    play_started.set()
                    release_playback.wait(3)
                def factory(**kwargs):
                    return RobotSpeechRuntime(speaker=speaker,**kwargs)
                try:
                    scene, planner = scenario(robot,"train-open"), Planner()
                    if robot == "ev3":
                        result = run_ev3(scene,planner,32,lambda _:None,speech_runtime_factory=factory)
                    else:
                        result = run_blast_gemma_scenario(scene,planner_factory=lambda _:planner,
                            max_decisions=32,speech_runtime_factory=factory)
                    self.assertTrue(result["completed"],result.get("error_message"))
                    self.assertTrue(result["goal_reached"])
                    self.assertGreater(len(packets),5)
                    self.assertIn("Made it!",spoken)
                    self.assertEqual(expressions[-1]["gesture"],"saw_celebrate" if robot == "ev3" else "claw_flourish")
                    self.assertTrue(all(value is None for value in expressions[:-1]))
                    self.assertTrue(received)
                    self.assertTrue(all(packet in packets for packet in received))
                finally:
                    release_generation.set()
                    release_playback.set()

    def test_new_route_discards_old_comment_and_only_keeps_latest_snapshot(self):
        entered, release = threading.Event(), threading.Event()
        spoken, snapshots = [], []
        context = SimpleNamespace(episode_id="test",request=SimpleNamespace(locale="en"),
            settings=SimpleNamespace(speech_enabled=True),stop_requested=threading.Event(),
            emergency_stop_requested=threading.Event(),publish=lambda _:None)
        runtime = RobotSpeechRuntime(speaker=lambda text,*_:spoken.append(text))
        speech = EpisodeSpeech(factory=lambda **_:runtime,supported_locales=("en",),context=context)
        speech.start()
        packet = {"robot_id":"ev3", "goal":"forward", "observation":{},
                  "active_waypoint_plan":[{"x_mm":800,"y_mm":0}],
                  "local_map_evidence":{"route_execution_memory":{"records":[{"reason":"near_obstacle"}]}}}
        original = copy.deepcopy(packet)
        def comment(snapshot):
            snapshots.append(snapshot)
            if len(snapshots) == 1:
                entered.set()
                release.wait(2)
                return "Old route."
            return "New route."
        try:
            speech.observe(packet,commentator=comment,progress_revision=1)
            self.assertTrue(entered.wait(1))
            packet["active_waypoint_plan"][0]["y_mm"] = 300
            speech.observe(packet,commentator=comment,progress_revision=2)
            packet["active_waypoint_plan"][0]["y_mm"] = 400
            speech.observe(packet,commentator=comment,progress_revision=3)
            release.set()
            self.assertTrue(runtime.close(drain=True,timeout_seconds=2))
            self.assertEqual(spoken,["New route."])
            self.assertEqual(snapshots[0],original)
            self.assertEqual(snapshots[-1],packet)
            self.assertEqual(len(snapshots),2)
        finally:
            release.set()
            speech.close()

    def test_comment_failure_or_cancellation_cannot_play_late_audio(self):
        for fail in (False,True):
            entered, release = threading.Event(), threading.Event()
            spoken = []
            runtime = RobotSpeechRuntime(speaker=lambda text,*_:spoken.append(text))
            def comment():
                entered.set()
                release.wait(2)
                if fail:
                    raise RuntimeError("model offline")
                return "Too late."
            runtime.start()
            try:
                runtime.offer(episode_id="test",text=None,locale="en",commentary=comment,is_current=lambda:True)
                self.assertTrue(entered.wait(1))
                if not fail:
                    runtime.cancel_episode("test")
                release.set()
                self.assertTrue(runtime.close(drain=True,timeout_seconds=2))
                self.assertEqual(spoken,[])
            finally:
                release.set()
                runtime.close()
