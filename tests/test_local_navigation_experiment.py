"""No network, hardware, optional training package or model artifact needed."""
from copy import deepcopy
from dataclasses import replace
import math
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from experiments.local_navigation import ACTIONS, VERSION, LocalPlanner, LocalPolicy, features, target, train
from robot_agent.lm_studio_controller_action import ControllerActionContext, ControllerActionDecision, ControllerActionPlannerResult


def packet():
    return {"available_actions": ["ADVANCE", "FOLLOW_WAYPOINT", "SCAN_FRONT_ARC"],
            "active_waypoint": {"x_mm":800, "y_mm":0, "purpose":"goal"},
            "local_map_evidence": {"robot_pose":{"x_mm":0,"y_mm":0,"heading_deg":0},
                "directional_goal":{"target_x_mm":800,"target_y_mm":0},
                "robot_footprint":{"front_extent_mm":110,"rear_extent_mm":90,"left_extent_mm":100,"right_extent_mm":130},
                "range_observations":[{"observed_from":{"x_mm":110,"y_mm":80},
                    "ray_heading_deg_in_episode":30,"distance_mm":300,"settled":True}]},
            "observation":{"sensors":{"distance_mm":300}},
            "history":[],"completion_allowed":False}


def biased_policy(action):
    return LocalPolicy({"version":VERSION,"actions":list(ACTIONS),
        "w1":[[0.0] for _ in features({})],"b1":[0.0],
        "w2":[[0.0]*len(ACTIONS)],"b2":[10.0 if a==action else 0.0 for a in ACTIONS]})


class LocalNavigationExperimentTests(unittest.TestCase):
    def test_dense_test_goals_are_accepted_by_the_real_mission_contract(self):
        from scripts.local_navigation_experiment import DENSE_TEST,scenario
        from robot_agent.physical_navigation_mission import DirectionalMission
        from robot_agent.physical_odometry import PhysicalPose
        for robot in ('ev3','blast'):
            for name in DENSE_TEST:
                with self.subTest(robot=robot,scene=name):
                    scene=scenario(robot,name)
                    distance=scene.goals[0].x_mm-scene.robots[0].pose.x_mm
                    mission=DirectionalMission.begin(episode_id=name,
                        minimum_forward_progress_mm=distance,pose=PhysicalPose())
                    self.assertEqual(mission.minimum_forward_progress_mm,distance)

    def test_truth_text_and_scene_identity_do_not_enter_features(self):
        p = packet()
        expected = features(p)
        p.update(world={"obstacles":[{"x_mm":900}]}, scene="maze", goal="secret route", robot_id="new-name")
        p["local_map_evidence"]["obstacle_estimates"] = [{"bounds":"invented"}]
        self.assertEqual(features(p), expected)

    def test_translation_and_quarter_turn_preserve_relative_features(self):
        p, q = packet(), packet()
        q["local_map_evidence"]["robot_pose"] = {"x_mm":200,"y_mm":300,"heading_deg":90}
        q["active_waypoint"].update(x_mm=200,y_mm=1100)
        q["local_map_evidence"]["directional_goal"].update(target_x_mm=200,target_y_mm=1100)
        ray = q["local_map_evidence"]["range_observations"][0]
        ray["observed_from"] = {"x_mm":120,"y_mm":410}
        ray["ray_heading_deg_in_episode"] += 90
        for a,b in zip(features(p), features(q)):
            self.assertAlmostEqual(a,b)

    def test_missing_ray_is_not_zero_distance_or_verified_free(self):
        p, q = packet(), packet()
        p["local_map_evidence"]["range_observations"][0]["distance_mm"] = None
        q["local_map_evidence"]["range_observations"][0]["distance_mm"] = 0
        self.assertNotEqual(features(p), features(q))
        q["local_map_evidence"]["range_observations"] = []
        self.assertNotEqual(features(p), features(q))

    def test_current_measurement_body_and_offset_are_used(self):
        for edit in (lambda p:p["observation"]["sensors"].update(distance_mm=None),
                     lambda p:p["local_map_evidence"]["robot_footprint"].update(right_extent_mm=230),
                     lambda p:p["local_map_evidence"].update(sensor_layout={"forward_offset_mm":110,"left_offset_mm":80})):
            p = packet()
            edit(p)
            self.assertNotEqual(features(p), features(packet()))

    def test_ir_is_not_converted_to_metric_distance(self):
        p = packet()
        p["local_map_evidence"]["ir_survey"] = {"samples":[{
            "observed_from":{"x_mm":0,"y_mm":0},"ray_heading_deg_in_episode":0,
            "filtered_ir_proximity":50}]}
        f = features(p)
        self.assertEqual(f[0], 1)
        self.assertEqual(f[-6], .5)
        self.assertEqual(len(f), len(features(packet())))

    def test_route_change_is_a_learned_replan_label(self):
        p = packet()
        self.assertEqual(target(p, {"action":"ADVANCE","waypoint":None}), "ADVANCE")
        self.assertEqual(target(p, {"action":"FOLLOW_WAYPOINT","waypoint":{"x_mm":0,"y_mm":400}}), "REPLAN")
        self.assertIsNone(target({**p,"active_waypoint":None}, {"action":"ADVANCE"}))

    def test_route_rejection_is_visible_despite_last_successful_scan(self):
        p = packet()
        p["observation"]["last_motion"] = {"action":"SCAN_FRONT_ARC", "completed":True,
                                             "made_progress":True, "measured_translation_mm":0}
        before = features(p)
        p["local_map_evidence"]["latest_route_rejection"] = {
            "rejection":{"reason":"KNOWN_ECHO_CLEARANCE_INTERSECTION", "leg_index":1,
                         "clearance_mm":150, "blocking_echo_point":{"x_mm":600,"y_mm":90}},
            "repeat_count":40, "pose_or_evidence_changed":False,
            "rejected_waypoint_plan":[p["active_waypoint"]]}
        self.assertNotEqual(features(p), before)
        unchanged = features(p)
        p["local_map_evidence"]["latest_route_rejection"]["pose_or_evidence_changed"] = True
        self.assertNotEqual(features(p), unchanged)

    def test_latest_action_outcome_does_not_disappear_without_new_motion(self):
        p = packet()
        p["history"] = [{"action":"FOLLOW_WAYPOINT", "result":"heading_aligned",
                         "pose":{"x_mm":0,"y_mm":0,"heading_deg":0}, "last_motion":None}]
        before = features(p)
        p["history"][-1]["result"] = "near_obstacle"
        self.assertNotEqual(features(p), before)

    def test_motion_reason_admission_and_scan_coverage_are_not_discarded(self):
        changes = [
            lambda p:p["observation"].update(last_motion={"completed":False,"reason":"infrared_blocked"}),
            lambda p:p["observation"].update(last_motion={"completed":False,"reason":"new_transport_error"}),
            lambda p:p.update(execution={"forward_admission":{"permitted":False,"reason":"blocked_hysteresis_hold"}}),
            lambda p:p["local_map_evidence"].update(ir_survey={"samples":[],"complete":False,"sweep_degrees":8.4}),
            lambda p:p["local_map_evidence"].update(ir_survey={"samples":[],"complete":True,"sweep_degrees":120}),
        ]
        encoded = []
        for change in changes:
            p = packet()
            change(p)
            self.assertNotEqual(features(p), features(packet()))
            encoded.append(tuple(features(p)))
        self.assertEqual(len(set(encoded)), len(changes))

    def test_an_older_ray_within_packet_limit_is_not_lost(self):
        p = packet()
        rays = p["local_map_evidence"]["range_observations"]
        rays[:] = [deepcopy(rays[0]) for _ in range(128)]
        before = features(p)
        rays[0]["distance_mm"] = 55
        self.assertNotEqual(features(p), before)

    def test_blast_nested_scan_and_route_interruptions_are_feedback_too(self):
        p = packet()
        p["history"] = [{"action":"SCAN_FRONT_ARC", "scan":{
            "state":"complete","result":"restored","sweep_coverage_deg":120,
            "restoration_verified":True,"all_observations_settled":True}}]
        before = features(p)
        for field,value in (("state","incomplete"),("result","interrupted"),
                            ("restoration_verified",False),("sweep_coverage_deg",8.4)):
            q = deepcopy(p)
            q["history"][-1]["scan"][field] = value
            self.assertNotEqual(features(q),before)
        for field,value in (("action_error","transport_timeout"),
                            ("route_interruption",{"reason":"RANGE_MEASUREMENT_UNAVAILABLE"})):
            q = deepcopy(p)
            q["history"][-1][field] = value
            self.assertNotEqual(features(q),before)

    def test_two_robot_profiles_and_primitive_lengths_remain_distinct(self):
        from scripts.local_navigation_experiment import scenario
        ev3, blast = (scenario(robot,"train-right-corner").robots[0] for robot in ("ev3","blast"))
        self.assertNotEqual(ev3.footprint, blast.footprint)
        p, q = packet(), packet()
        p["execution"] = {"advance_step_mm":52}
        q["execution"] = {"advance_step_mm":45}
        self.assertNotEqual(features(p), features(q))

    def test_held_out_world_cannot_enter_training(self):
        from scripts.local_navigation_experiment import fit
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root/"run").mkdir()
            (root/"run/summary.json").write_text(json.dumps({"scene":"maze","mode":"astra","clean":True}))
            with self.assertRaisesRegex(ValueError,"Test/local runs"):
                fit(SimpleNamespace(data=root,recorded_box_root=None,queries=None,weights=root/"weights.json"))
            self.assertFalse((root/"weights.json").exists())

    def test_only_completed_teacher_trajectories_enter_training(self):
        from scripts.local_navigation_experiment import fit
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            data, recovery = root/"train", root/"recovery"
            data.mkdir()
            recovery.mkdir()
            cases = [(data,"ev3-train-right-astra","train-right","astra",True,"decision_pair","ADVANCE"),
                     (data,"ev3-train-left-astra","train-left","astra",True,"decision_pair","FOLLOW_WAYPOINT"),
                     (recovery,"ev3-train-right-corner-collect","train-right-corner","collect",False,"teacher_label","REVERSE"),
                     (recovery,"blast-train-staggered-collect","train-staggered","collect",True,"teacher_label","SCAN_FRONT_ARC")]
            for parent,name,scene,mode,clean,event,action in cases:
                run = parent/name
                run.mkdir()
                (run/"summary.json").write_text(json.dumps({"scene":scene,"mode":mode,"clean":clean,
                    "collection_protocol":"complete-astra-recovery-v1","recovery_complete":clean}))
                (run/"events.jsonl").write_text(json.dumps({"event":event,"packet":packet(),
                                                           "decision":{"action":action,"waypoint":None}})+"\n")
            with patch("scripts.local_navigation_experiment.train",return_value=biased_policy("ADVANCE").artifact) as learn:
                with patch("builtins.print"):
                    fit(SimpleNamespace(data=data,recovery_data=recovery,recorded_box_root=None,
                                        queries=None,weights=root/"weights.json"))
                training,validation = learn.call_args.args
                self.assertNotIn("REVERSE",[d["action"] for p,d in training])
                self.assertIn("SCAN_FRONT_ARC",[d["action"] for p,d in training])
                self.assertEqual([d["action"] for p,d in validation],["FOLLOW_WAYPOINT"])

    def test_simulator_stall_does_not_invent_motion_and_is_finite(self):
        from experiments.navigation_recovery_data import RecoveryFault
        from scripts.local_navigation_experiment import scenario
        from robot_agent.multi_robot_navigation_simulator import MultiRobotNavigationSimulator
        original = MultiRobotNavigationSimulator.move
        for robot in ("ev3","blast"):
            world = scenario(robot,"train-open").build()
            with RecoveryFault("stall",after_mm=0) as fault:
                start = world.pose(robot)
                self.assertEqual(world.move(robot,20),0)
                self.assertEqual(world.pose(robot),start)
                self.assertEqual(world.move(robot,20),20)
                self.assertTrue(fault.triggered)
                self.assertFalse(any(e.kind=="blocked" for e in world.events))
            self.assertIs(MultiRobotNavigationSimulator.move,original)

    def test_partial_scan_keeps_actual_angle_and_resumes_without_fixture_advice(self):
        from experiments.navigation_recovery_data import RecoveryFault
        from scripts.local_navigation_experiment import scenario
        for robot in ("ev3","blast"):
            world = scenario(robot,"train-open").build()
            with RecoveryFault("scan",after_mm=0) as fault:
                fault.before_action("SCAN_FRONT_ARC")
                world.rotate(robot,15_000)
                start = world.pose(robot)
                self.assertEqual(world.rotate(robot,15_000),start)
                self.assertEqual(world.rotate(robot,15_000).heading_mdeg,30_000)
                self.assertTrue(fault.triggered)

    def test_repeated_stall_can_include_reverse_and_has_a_fixed_end(self):
        from experiments.navigation_recovery_data import RecoveryFault
        from scripts.local_navigation_experiment import scenario
        for robot in ("ev3","blast"):
            world = scenario(robot,"train-open").build()
            with RecoveryFault("stall",after_mm=0,stalled_attempts=3) as fault:
                for requested in (20,-20,-20):
                    self.assertEqual(world.move(robot,requested),0)
                    self.assertEqual(world.pose(robot).x_mm,0)
                self.assertEqual(world.move(robot,20),20)
                self.assertEqual(fault.stalls_remaining,0)

    def test_no_return_uses_each_robots_real_sensor_units(self):
        from experiments.navigation_recovery_data import RecoveryFault
        from robot_agent.simulation_robot_adapters import SharedWorldBlastController, SharedWorldEV3Transport
        from scripts.local_navigation_experiment import scenario
        from robot_agent.ev3rstorm_profile import EV3RSTORMProfile
        blast = SharedWorldBlastController(scenario("blast","train-open").build(),world_robot_id="blast")
        ev3 = SharedWorldEV3Transport(scenario("ev3","train-open").build(),world_robot_id="ev3",
                                      odometry=EV3RSTORMProfile().odometry_calibration)
        for read,missing in ((blast._distance_mm,2000),(lambda:ev3._ir_value(200),100)):
            with RecoveryFault("range",after_mm=0,missing_reads=2) as fault:
                # Resolve patched methods at call time, not bound before entry.
                fn = blast._distance_mm if missing==2000 else read
                self.assertEqual(fn(),missing)
                self.assertEqual(fn(),missing)
                fn()
                self.assertEqual(fault.missing_remaining,0)

    def test_dropped_corner_is_rejected_by_existing_validator_only_once(self):
        from experiments.navigation_recovery_data import RecoveryFault
        from robot_agent.coarse_navigation_grid import model_route_blockage
        decision = ControllerActionDecision("FOLLOW_WAYPOINT", 900, "route", (), None,
            waypoint={"x_mm":0,"y_mm":300},
            following_waypoints=({"x_mm":650,"y_mm":300},{"x_mm":650,"y_mm":0}))
        events = []
        fault = RecoveryFault("route", events.append)
        changed = fault.transmitted_decision(decision, packet())
        self.assertIsNot(changed, decision)
        self.assertEqual(decision.waypoint, {"x_mm":0,"y_mm":300})
        rejected = model_route_blockage(start=(0,0), possible_obstacles=(),
            waypoints=tuple((p["x_mm"],p["y_mm"]) for p in (changed.waypoint,*changed.following_waypoints)))
        self.assertEqual(rejected["reason"],"NON_ORTHOGONAL_ROUTE_LEG")
        self.assertIs(fault.transmitted_decision(decision, packet()),decision)
        self.assertTrue(events[-1]["teacher_label_excluded"])
        p = packet()
        p["history"] = [{"route_rejection":rejected}]
        fault.observe(p)
        self.assertTrue(fault.observed)

    def test_route_fault_never_executes_an_accepted_shortcut(self):
        from experiments.navigation_recovery_data import RecoveryFault
        decision = ControllerActionDecision("FOLLOW_WAYPOINT", 900, "route", (), None,
            waypoint={"x_mm":300,"y_mm":0}, following_waypoints=({"x_mm":650,"y_mm":0},))
        fault = RecoveryFault("route")
        self.assertIs(fault.transmitted_decision(decision, packet()),decision)
        self.assertFalse(fault.triggered)

    def test_recovery_replay_stops_before_first_rejected_input_and_excludes_test_worlds(self):
        from scripts.local_navigation_experiment import recovery_prefix
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            path = root/"events.jsonl"
            (root/"source-hashes.json").write_text(json.dumps({"src/engine.py":"same"}))
            before, after = packet(), packet()
            after["history"] = [{"route_rejection":{"reason":"NON_ORTHOGONAL_ROUTE_LEG"}}]
            rows = [{"event":"configuration","robot":"blast","scene":"train-right-corner"},
                    {"event":"decision_pair","packet":before,"decision":{"action":"FOLLOW_WAYPOINT"}},
                    {"event":"decision_pair","packet":after,"decision":{"action":"SCAN_FRONT_ARC"}}]
            path.write_text("\n".join(json.dumps(r) for r in rows))
            args = dict(robot="blast",scene="train-right-corner",source_hashes={"src/engine.py":"same"})
            self.assertEqual(recovery_prefix(path,**args),rows[1:2])
            with self.assertRaisesRegex(ValueError,"production code"):
                recovery_prefix(path,**{**args,"source_hashes":{"src/engine.py":"changed"}})
            rows[0]["scene"] = "maze"
            path.write_text("\n".join(json.dumps(r) for r in rows))
            with self.assertRaisesRegex(ValueError,"training world"):
                recovery_prefix(path,**{**args,"scene":"maze"})

    def test_held_out_feedback_cannot_be_smuggled_into_recovery_training(self):
        from scripts.local_navigation_experiment import fit
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root/"train").mkdir()
            (root/"recovery/run").mkdir(parents=True)
            (root/"recovery/run/summary.json").write_text(json.dumps({"scene":"maze","mode":"collect","clean":False}))
            with self.assertRaisesRegex(ValueError,"independent recovery"):
                fit(SimpleNamespace(data=root/"train",recovery_data=root/"recovery",recorded_box_root=None,
                                    queries=None,weights=root/"weights.json"))

    def test_unavailable_action_cannot_be_selected(self):
        p = packet()
        selected, _ = biased_policy("REVERSE").predict(p)
        self.assertNotEqual(selected, "REVERSE")

    def test_invalid_weights_rejected(self):
        model = deepcopy(biased_policy("ADVANCE").artifact)
        model["b1"][0] = math.nan
        with self.assertRaises(ValueError):
            LocalPolicy(model)
        model["b1"] = []
        with self.assertRaises(ValueError):
            LocalPolicy(model)

    def context(self):
        p = packet()
        return ControllerActionContext(goal="Move forward",locale="en",robot_id="test",controller_id="test",
            available_actions=tuple(p["available_actions"]), observation=p["observation"],
            local_map_evidence=p["local_map_evidence"],active_waypoint=p["active_waypoint"],
            active_waypoint_plan=(p["active_waypoint"],),completion_allowed=False,abort_allowed=False)

    def test_local_action_replaces_llm_call_and_keeps_route(self):
        astra = Mock()
        result = LocalPlanner(astra,biased_policy("ADVANCE")).decide(self.context())
        astra.decide.assert_not_called()
        self.assertEqual(result.decision.action,"ADVANCE")
        self.assertIsNone(result.decision.waypoint)
        self.assertEqual(result.decision.following_waypoints,())

    def test_no_route_and_learned_replan_each_delegate_once(self):
        for context in (self.context(),replace(self.context(),active_waypoint=None,active_waypoint_plan=())):
            astra = Mock()
            expected = ControllerActionPlannerResult(ControllerActionDecision("ADVANCE",500,"test",("ADVANCE",),None),0)
            astra.decide.return_value = expected
            self.assertIs(LocalPlanner(astra,biased_policy("REPLAN")).decide(context), expected)
            astra.decide.assert_called_once_with(context)

    def test_completion_uses_existing_goal_contract(self):
        astra = Mock()
        context = replace(self.context(),completion_allowed=True)
        result = LocalPlanner(astra,biased_policy("ADVANCE")).decide(context)
        self.assertEqual(result.decision.action,"COMPLETE")
        astra.decide.assert_not_called()

    @unittest.skipUnless(importlib.util.find_spec("numpy"), "Optional experiment training dependency")
    def test_training_changes_weights_and_learns_held_out_synthetic_labels(self):
        def rows(offset):
            samples = []
            for i in range(24):
                p = packet()
                distance = 50 + i * 5 + offset if i < 12 else 1000 + i * 5 + offset
                p["observation"]["sensors"]["distance_mm"] = distance
                p["local_map_evidence"]["range_observations"][0]["distance_mm"] = distance
                action = "SCAN_FRONT_ARC" if i < 12 else "ADVANCE"
                samples.append((p,{"action":action,"waypoint":None}))
            return samples
        training, validation = rows(0), rows(2)
        model = LocalPolicy(train(training,validation,epochs=100))
        self.assertGreaterEqual(sum(model.predict(p)[0]==d["action"] for p,d in validation),23)
        self.assertEqual(model.artifact["training_rows"],24)

    @unittest.skipUnless(importlib.util.find_spec("numpy"), "Optional experiment training dependency")
    def test_training_can_distinguish_recovery_without_changing_pose(self):
        def examples(offset):
            rows = []
            for i in range(12):
                for reason, label in (("heading_aligned","FOLLOW_WAYPOINT"),
                                      ("near_obstacle","REVERSE"),
                                      ("route_rejected","REPLAN")):
                    p = packet()
                    p["available_actions"].append("REVERSE")
                    p["observation"]["sensors"]["distance_mm"] = 300+i+offset
                    p["history"] = [{"action":"FOLLOW_WAYPOINT","result":reason}]
                    decision = {"action":label, "waypoint":None}
                    rows.append((p,decision))
            return rows
        rows = examples(1)
        model = LocalPolicy(train(examples(0),rows,epochs=100))
        self.assertEqual([model.predict(p)[0] for p,d in rows], [d["action"] for p,d in rows])


if __name__ == "__main__":
    unittest.main()
