import copy
import math
import unittest
from unittest.mock import patch

from experiments.navigation_set_author import expand, compact_set_packet
from experiments.navigation_set_policy import observations, SetPolicy, RAY_FIELDS, MEMORY_FIELDS
from robot_agent.route_execution_memory import route_execution_memory
from robot_agent.blast_episode_planning import _planner_map_with_route_feedback


def specification():
    # Inference test numbers only. Never passed to Astra or used in evaluation.
    return {"ray":[{"index":0,"bias":0,"inputs":[{"feature":10,"weight":1}]}],
            "memory":[{"index":0,"bias":0,"inputs":[{"feature":16,"weight":1}]}],
            "head":[{"index":0,"bias":0,"inputs":[{"feature":252,"weight":1},{"feature":284,"weight":1}]}],
            "output":[{"index":7,"bias":0,"inputs":[{"feature":0,"weight":1}]}]}


class NavigationSetPolicyTests(unittest.TestCase):
    def test_shapes_and_geometry_use_sensor_origin(self):
        packet = {"local_map_evidence":{"robot_pose":{"x_mm":100,"y_mm":200,"heading_deg":90},
            "range_observations":[{"observed_from":{"x_mm":20,"y_mm":310},
                "ray_heading_deg_in_episode":90,"distance_mm":1000,"range_state":"MEASURED"}]}}
        global_values, rays, memory = observations(packet)
        self.assertEqual(len(global_values),252)
        self.assertEqual(len(rays[0]),len(RAY_FIELDS))
        self.assertAlmostEqual(rays[0][5],.11)
        self.assertAlmostEqual(rays[0][6],.08)
        self.assertEqual(rays[0][9],1)
        self.assertAlmostEqual(rays[0][10],1.11)
        self.assertAlmostEqual(rays[0][11],.08)
        self.assertEqual(memory,[])

    def test_ir_does_not_invent_metric_endpoint(self):
        p = {"local_map_evidence":{"ir_survey":{"samples":[{"filtered_ir_proximity":30}]}}}
        ray = observations(p)[1][0]
        self.assertEqual(ray[:4],[1,1,.3,0])
        self.assertEqual(ray[9:12],[0,0,0])
        self.assertEqual(ray[25:],[0,0,0,0])

    def test_point_clearance_keeps_body_sides_and_each_hit_together(self):
        p = {"execution":{"advance_step_mm":45,"reverse_step_mm":30},
             "local_map_evidence":{"robot_footprint":{
                 "front_extent_mm":100,"rear_extent_mm":80,
                 "left_extent_mm":150,"right_extent_mm":200,
                 "clearance_margin_mm":10,"turn_sweep_radius_mm":240},
                 "range_observations":[]}}
        def hit(x, y):
            return {"observed_from":{"x_mm":x,"y_mm":y},
                    "distance_mm":0,"range_state":"MEASURED"}
        p["local_map_evidence"]["range_observations"] = [
            hit(140,0), hit(-120,0), hit(0,180), hit(0,-180),
            hit(1000,-1000), hit(-1000,1000)]
        rays = observations(p)[1]
        # Same hit: ahead of current body, inside the next 45 mm forward sweep.
        self.assertAlmostEqual(rays[0][25],.03)
        self.assertAlmostEqual(rays[0][26],-.015)
        # Rear hit intersects reverse, not forward. Asymmetric arm sides remain.
        self.assertAlmostEqual(rays[1][26],.03)
        self.assertAlmostEqual(rays[1][27],0)
        self.assertAlmostEqual(rays[2][25],.02)
        self.assertAlmostEqual(rays[3][25],-.03)
        # Separate far-away points must not masquerade as one nearby obstacle.
        for ray in rays[-2:]:
            self.assertGreater(min(ray[25:]),1)
        self.assertAlmostEqual(rays[0][28],-.1)

    def test_clearance_changes_with_robot_body_without_sensor_unit_conversion(self):
        p = {"local_map_evidence":{"robot_footprint":{
            "front_extent_mm":100,"rear_extent_mm":100,
            "left_extent_mm":100,"right_extent_mm":100},
            "range_observations":[{"distance_mm":200,"range_state":"MEASURED"}]}}
        self.assertAlmostEqual(observations(p)[1][0][25],.1)
        p["local_map_evidence"]["robot_footprint"]["front_extent_mm"] = 250
        self.assertAlmostEqual(observations(p)[1][0][25],-.05)

    def test_all_rays_processed_and_order_invariant(self):
        p = {"available_actions":["FOLLOW_WAYPOINT"],"local_map_evidence":{
            "range_observations":[{"distance_mm":500,"range_state":"MEASURED"} for _ in range(200)]}}
        policy = SetPolicy(expand(specification()))
        initial = policy.scores(p)
        p["local_map_evidence"]["range_observations"][0]["distance_mm"] = 1000
        self.assertNotEqual(initial,policy.scores(p))
        changed = policy.scores(p)
        p["local_map_evidence"]["range_observations"].reverse()
        self.assertEqual(changed,policy.scores(p))
        self.assertEqual(len(observations(p)[1]),200)

    def test_missing_echo_is_not_metric_zero_or_hit(self):
        p = {"local_map_evidence":{"range_observations":[
            {"distance_mm":None,"range_state":"NO_VALID_DISTANCE"},
            {"distance_mm":100,"range_state":"INVALID"}]}}
        rays = observations(p)[1]
        self.assertEqual([r[9] for r in rays],[0,0])
        self.assertNotEqual(rays[0],rays[1])
        p["local_map_evidence"]["range_observations"].append({
            "distance_mm":0,"range_state":"MEASURED","observed_from":{"x_mm":110,"y_mm":80}})
        zero = observations(p)[1][-1]
        self.assertEqual(zero[9:12],[1,.11,.08])

    def test_ev3_context_gets_full_memory_before_history_trimming(self):
        from test_ev3_navigation_body import ev3_test_body, ev3_navigation_context
        body, profile, mission = ev3_test_body()
        self.addCleanup(body.close)
        body.scan()
        event = {"action":"FOLLOW_WAYPOINT","waypoint_plan":[{"x_mm":0,"y_mm":400}],
            "pose":{"x_mm":0,"y_mm":0,"heading_mdeg":90000},
            "route_rejection":{"reason":"test"}}
        history = [event]+[{"action":"OBSERVE"} for _ in range(20)]
        context = ev3_navigation_context(body=body,descriptor=profile.descriptor,
            footprint=profile.hazard_calibration.robot_footprint,mission=mission,
            goal="Reach the goal",history=history)
        self.assertEqual(len(context.history),12)
        record = context.local_map_evidence["route_execution_memory"]["records"][0]
        self.assertEqual(record["reason"],"test")
        self.assertEqual(record["pose_after"]["heading_deg"],90)
        self.assertIn("heading_mdeg",event["pose"])

    def test_memory_survives_scan_reverse_and_history_limits(self):
        route = [{"x_mm":1000,"y_mm":0},{"x_mm":1000,"y_mm":400}]
        bad = {"action":"FOLLOW_WAYPOINT","waypoint_plan":route,
               "last_motion":{"action":"ADVANCE","completed":False,"made_progress":False},
               "route_rejection":{"reason":"clearance"}}
        events = [bad,copy.deepcopy(bad)] + [
            {"action":"SCAN_FRONT_ARC" if i%2 else "REVERSE", "waypoint_plan":route,
             "last_motion":{"action":"REVERSE","completed":True,"made_progress":True}}
            for i in range(20)]
        memory = route_execution_memory(events)
        self.assertEqual(memory["records"][0]["count"],2)
        self.assertEqual(memory["records"][0]["route_rejection"],{"reason":"clearance"})
        changed = [{"x_mm":1000,"y_mm":-300}]
        p = {"active_waypoint":changed[0],"active_waypoint_plan":changed,
             "local_map_evidence":{"route_execution_memory":memory}}
        rows = observations(p)[2]
        self.assertTrue(all(len(r)==len(MEMORY_FIELDS) for r in rows))
        self.assertTrue(any(r[16] == 1 and r[2] == 0 for r in rows))
        self.assertEqual(route_execution_memory([])["records"],[])
        self.assertEqual(_planner_map_with_route_feedback({},events)["route_execution_memory"],memory)

    def test_ev3_denied_forward_keeps_required_motion_visible(self):
        from test_ev3_navigation_body import ev3_test_body, ev3_navigation_context
        body, profile, mission = ev3_test_body()
        self.addCleanup(body.close)
        body.scan()
        body.observation["infrared"].update(blocked=True,reason="blocked_hysteresis_hold")
        for waypoint, action in (({"x_mm":500,"y_mm":0,"purpose":"forward"},"ADVANCE"),
                                 ({"x_mm":0,"y_mm":500,"purpose":"left"},"TURN_LEFT_90")):
            with self.subTest(action=action):
                context = ev3_navigation_context(body=body,descriptor=profile.descriptor,
                    footprint=profile.hazard_calibration.robot_footprint,mission=mission,
                    goal="Reach the goal",waypoints=(waypoint,))
                self.assertFalse(context.execution["forward_admission"]["permitted"])
                self.assertNotIn("ADVANCE",context.available_actions)
                self.assertEqual(context.execution["next_waypoint_motion"]["action"],action)

    def test_partial_turn_not_relabelled_as_blocked(self):
        event = {"action":"FOLLOW_WAYPOINT","waypoint_plan":[{"x_mm":0,"y_mm":500}],
                 "last_motion":{"action":"TURN_LEFT_90","completed":False,"made_progress":True,
                                "reason":"turn_pending","measured_turn_deg":45}}
        record = route_execution_memory([event])["records"][0]
        self.assertEqual(record["reason"],"turn_pending")
        self.assertTrue(record["made_progress"])
        self.assertFalse(record["route_rejection"])

    def test_memory_keeps_results_without_motor_motion(self):
        route = [{"x_mm":0,"y_mm":500}]
        blocked = {"action":"FOLLOW_WAYPOINT", "waypoint_plan":route,
                   "result":"near_obstacle", "last_motion":None,
                   "motion_attempts":0, "pose":{"x_mm":0,"y_mm":0}}
        observed = {"action":"OBSERVE", "waypoint_plan":route,
                    "result":"observation_refreshed", "last_motion":None}
        records = route_execution_memory([blocked, copy.deepcopy(blocked), observed])["records"]
        self.assertEqual(len(records),2)
        self.assertEqual(records[0]["reason"],"near_obstacle")
        self.assertEqual(records[0]["count"],2)
        self.assertEqual(records[0]["last_seen"],1)
        self.assertIsNone(records[0]["completed"])
        self.assertIsNone(records[0]["made_progress"])
        self.assertEqual(records[1]["reason"],"observation_refreshed")

    def test_weight_author_sees_the_same_full_memory_and_range_states_as_model(self):
        route = [{"x_mm":700,"y_mm":0}]
        memory = route_execution_memory([{"action":"FOLLOW_WAYPOINT",
            "waypoint_plan":route, "route_rejection":{"reason":"clearance"}}])
        p = {"active_waypoint":route[0],"active_waypoint_plan":route,
             "execution":{"advance_step_mm":45,"reverse_step_mm":45},
             "local_map_evidence":{"route_execution_memory":memory,
                 "range_observations":[{"distance_mm":2000,"range_state":"NO_VALID_DISTANCE"}]}}
        compact = compact_set_packet(p)
        self.assertEqual(observations(compact), observations(p))
        self.assertEqual(compact["local_map_evidence"]["route_execution_memory"],memory)
        self.assertEqual(compact["local_map_evidence"]["range_observations"][0]["range_state"],
                         "NO_VALID_DISTANCE")

    def test_all_route_legs_present(self):
        route = [{"x_mm":100*i,"y_mm":0} for i in range(20)]
        rows = observations({"active_waypoint_plan":route})[2]
        self.assertEqual(len(rows),20)
        self.assertEqual(rows[-1][6],1.9)

    def test_numeric_compiler_does_not_train_or_run_policy(self):
        with patch.object(SetPolicy,"scores",side_effect=AssertionError("inference")):
            artifact = expand(specification())
        self.assertEqual(artifact["ray_weights"][10][0],1)
        for value in (True,float("nan"),float("inf"),"1"):
            spec = specification()
            spec["head"][0]["bias"] = value
            with self.assertRaises(ValueError):
                expand(spec)
        spec = specification()
        spec["ray"] *= 2
        with self.assertRaises(ValueError):
            expand(spec)


if __name__ == "__main__":
    unittest.main()
