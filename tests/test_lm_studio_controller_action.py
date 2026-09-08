import json
import unittest

from robot_agent.lm_studio import (
    LMStudioConfigurationError,
    LMStudioInputError,
    LMStudioProtocolError,
)
from robot_agent.blast_personality import (
    BLAST_PERSONA_BY_LOCALE,
    MAX_PERSONA_CHARS,
)
from robot_agent.lm_studio_controller_action import (
    COMPLETE,
    ControllerActionContext,
    FOLLOW_WAYPOINT,
    LMStudioControllerActionPlanner,
    MAX_UTTERANCE_CHARS,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    SCAN_FRONT_ARC,
    TURN_LEFT_90,
    TURN_RIGHT_90,
)


MODEL = "local/controller-model"


def context(**changes):
    values = {
        "goal": "Kör mot hindret och stanna ungefär 25 cm ifrån.",
        "locale": "sv",
        "robot_id": "blast-01",
        "controller_id": "blast-01.hub",
        "available_actions": (
            "DRIVE_FORWARD",
            "DRIVE_REVERSE",
            "TURN_LEFT",
            "TURN_RIGHT",
        ),
        "observation": {
            "distance_mm": 480,
            "motion_active": False,
            "imu": {"ready": True, "heading_deg": 0},
        },
        "history": (),
    }
    values.update(changes)
    return ControllerActionContext(**values)


def completion(output, **changes):
    output = {
        "waypoint": None,
        "following_waypoints": [],
        **output,
    }
    value = {
        "object": "chat.completion",
        "model": MODEL,
        "choices": [
            {
                "index": 0,
                "finish_reason": "stop",
                "message": {
                    "role": "assistant",
                    "content": json.dumps(output),
                },
            }
        ],
    }
    value.update(changes)
    return json.dumps(value).encode("utf-8")


class Transport:
    def __init__(self, response):
        self.response = response
        self.calls = []

    def __call__(self, *args):
        self.calls.append(args)
        if isinstance(self.response, BaseException):
            raise self.response
        return self.response


class ControllerActionPlannerTests(unittest.TestCase):
    def planner(self, response, clock_values=(1.0, 1.125), **options):
        transport = Transport(response)
        values = iter(clock_values)
        planner = LMStudioControllerActionPlanner(
            model=MODEL,
            transport=transport,
            clock=lambda: next(values),
            **options,
        )
        return planner, transport

    def test_completion_expression_uses_existing_social_vocabulary(self):
        expression = {"face": "happy", "gesture": "claw_flourish"}
        planner, transport = self.planner(completion({
            "action": COMPLETE, "confidence_milli": 950,
            "assessment": "Final goal reached.", "plan": [],
            "utterance": "Mission crushed!", "expression": expression,
        }), social_expressions=True)
        result = planner.decide(context(completion_allowed=True))
        self.assertEqual(result.decision.expression, expression)
        prompt = json.loads(transport.calls[0][1])["messages"][0]["content"]
        self.assertIn("Do not celebrate intermediate waypoints", prompt)

    def test_completion_expressions_do_not_change_navigation_requests(self):
        output = completion({
            "action": "DRIVE_FORWARD", "confidence_milli": 950,
            "assessment": "Continue.", "plan": ["DRIVE_FORWARD"],
            "utterance": None,
        })
        requests = []
        for enabled in (False, True):
            planner, transport = self.planner(output, social_expressions=enabled)
            self.assertIsNone(planner.decide(context(completion_allowed=False)).decision.expression)
            requests.append(transport.calls[0][1])
        self.assertEqual(*requests)

    def test_completion_expression_rejects_wrong_action_or_invalid_payload(self):
        for action, utterance, expression in (
            ("DRIVE_FORWARD", "Go!", {"face": "happy", "gesture": "arm_wave"}),
            (COMPLETE, None, {"face": "happy", "gesture": "arm_wave"}),
            (COMPLETE, "Done!", {"face": "happy", "gesture": "drive_forward"}),
            (COMPLETE, "Done!", {"face": "happy"}),
        ):
            with self.subTest(action=action, expression=expression):
                planner, _ = self.planner(completion({
                    "action": action, "confidence_milli": 950,
                    "assessment": "Done.", "plan": [],
                    "utterance": utterance, "expression": expression,
                }), social_expressions=True)
                with self.assertRaises(LMStudioProtocolError):
                    planner.decide(context(completion_allowed=True))

    def test_returns_one_observation_bound_action(self):
        planner, transport = self.planner(completion({
            "action": "DRIVE_FORWARD",
            "confidence_milli": 940,
            "assessment": "Det är fortfarande gott om plats framåt.",
            "plan": ["DRIVE_FORWARD", "COMPLETE"],
            "utterance": "Jaja, jag kör väl en bit till då.",
        }))

        result = planner.decide(context())

        self.assertEqual(result.latency_ms, 125)
        self.assertIsNone(result.reasoning_content)
        self.assertEqual(result.decision.action, "DRIVE_FORWARD")
        self.assertEqual(
            result.decision.plan,
            ("DRIVE_FORWARD", "COMPLETE"),
        )
        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        self.assertEqual(supplied["observation"]["distance_mm"], 480)
        self.assertEqual(supplied["goal"], context().goal)
        self.assertTrue(supplied["completion_allowed"])
        self.assertTrue(supplied["abort_allowed"])
        self.assertNotIn("robot_relative_side_scan", supplied)
        action_schema = request["response_format"]["json_schema"][
            "schema"
        ]["properties"]["action"]
        self.assertEqual(
            action_schema["enum"],
            [
                "DRIVE_FORWARD",
                "DRIVE_REVERSE",
                "TURN_LEFT",
                "TURN_RIGHT",
                "COMPLETE",
                "ABORT",
            ],
        )
        self.assertEqual(request["reasoning_effort"], "none")
        self.assertEqual(request["temperature"], 1.0)
        self.assertEqual(request["top_p"], 0.95)
        self.assertEqual(request["top_k"], 20)
        self.assertEqual(request["min_p"], 0.0)
        self.assertEqual(request["presence_penalty"], 0.0)
        self.assertEqual(request["repeat_penalty"], 1.0)

    def test_preserves_provider_reasoning_for_diagnostics(self):
        response = json.loads(completion({
            "action": "DRIVE_FORWARD",
            "confidence_milli": 940,
            "assessment": "The route ahead is clear.",
            "plan": ["DRIVE_FORWARD", "COMPLETE"],
            "utterance": None,
        }))
        response["choices"][0]["message"]["reasoning_content"] = (
            "  " + "The observed route is clear, so advance. " * 150 + "  "
        )
        planner, transport = self.planner(
            json.dumps(response).encode("utf-8")
        )

        result = planner.decide(context())

        self.assertEqual(
            result.reasoning_content,
            ("The observed route is clear, so advance. " * 150).strip(),
        )
        request = json.loads(transport.calls[0][1])
        system_prompt = request["messages"][0]["content"]
        self.assertIn("ADVANCE is semantic forward progress", system_prompt)
        self.assertIn("REVERSE is a bounded retreat", system_prompt)
        self.assertIn("change x or y, not both", system_prompt)
        self.assertIn("RANGE_MEASUREMENT_UNAVAILABLE", system_prompt)
        self.assertIn("following_waypoints are hypotheses", system_prompt)
        self.assertIn("may extend into unknown space", system_prompt)
        self.assertIn("not to limit future hypotheses", system_prompt)
        self.assertNotIn("Do not commit a waypoint leg", system_prompt)
        utterance_schema = request["response_format"]["json_schema"][
            "schema"
        ]["properties"]["utterance"]["oneOf"][0]
        self.assertEqual(
            utterance_schema["maxLength"],
            MAX_UTTERANCE_CHARS,
        )

    def test_default_utterance_limit_keeps_the_request_byte_identical(self):
        response = completion({
            "action": "DRIVE_FORWARD",
            "confidence_milli": 940,
            "assessment": "Det är fritt framåt.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": "Framåt.",
        })
        default_planner, default_transport = self.planner(response)
        explicit_planner, explicit_transport = self.planner(
            response,
            max_utterance_chars=MAX_UTTERANCE_CHARS,
        )

        default_planner.decide(context())
        explicit_planner.decide(context())

        self.assertEqual(
            default_transport.calls[0][1],
            explicit_transport.calls[0][1],
        )

    def test_reasoning_effort_can_match_the_loaded_model(self):
        planner, transport = self.planner(
            completion({
                "action": "DRIVE_FORWARD",
                "confidence_milli": 940,
                "assessment": "Det är fritt framåt.",
                "plan": ["DRIVE_FORWARD"],
                "utterance": None,
            }),
            reasoning_effort="low",
        )

        planner.decide(context())

        request = json.loads(transport.calls[0][1])
        self.assertEqual(request["reasoning_effort"], "low")

    def test_waypoint_arrival_tolerance_is_exposed_to_the_model(self):
        planner, transport = self.planner(completion({
            "action": "DRIVE_FORWARD",
            "confidence_milli": 940,
            "assessment": "The route is clear.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": None,
        }))

        planner.decide(context(waypoint_reached_radius_mm=75))

        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        self.assertEqual(supplied["waypoint_reached_radius_mm"], 75)
        self.assertIn(
            "a corner may begin that far before its coordinates",
            request["messages"][0]["content"],
        )

    def test_output_token_budget_can_expand_for_reasoning_models(self):
        response = json.loads(completion({
            "action": "DRIVE_FORWARD",
            "confidence_milli": 940,
            "assessment": "The route is clear.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": None,
        }))
        reasoning = "Measured geometry. " * 2000
        response["choices"][0]["message"]["reasoning_content"] = reasoning
        planner, transport = self.planner(
            json.dumps(response).encode(),
            max_output_tokens=8_192,
            reasoning_effort="low",
        )

        result = planner.decide(context())

        request = json.loads(transport.calls[0][1])
        self.assertEqual(request["max_tokens"], 8_192)
        self.assertEqual(request["reasoning_effort"], "low")
        self.assertEqual(result.reasoning_content, reasoning.strip())

    def test_invalid_choice_reports_finish_reason_and_bounded_reasoning(self):
        response = json.loads(completion({
            "action": "DRIVE_FORWARD",
            "confidence_milli": 940,
            "assessment": "The route is clear.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": None,
        }))
        response["choices"][0]["finish_reason"] = "length"
        response["choices"][0]["message"]["reasoning_content"] = (
            "I still need to choose a route. " * 150 + "End of reasoning."
        )
        planner, _transport = self.planner(
            json.dumps(response).encode("utf-8")
        )

        with self.assertRaisesRegex(
            LMStudioProtocolError,
            "finish_reason='length'.*I still need to choose a route",
        ):
            with self.assertLogs("robot_agent.navigation_diagnostics", "INFO") as logs:
                planner.decide(context())
        request_record, response_record = [
            json.loads(record.getMessage()) for record in logs.records
        ]
        self.assertEqual(request_record["event"], "planner_request")
        self.assertEqual(response_record["event"], "planner_response")
        self.assertEqual(request_record["request_id"], response_record["request_id"])
        self.assertEqual(json.loads(response_record["raw_response"]), response)

    def test_rejects_invalid_output_token_budget(self):
        for budget in (0, 8_193):
            with self.subTest(budget=budget), self.assertRaises(LMStudioConfigurationError):
                LMStudioControllerActionPlanner(max_output_tokens=budget)

    def test_rejects_unknown_reasoning_effort(self):
        with self.assertRaises(LMStudioConfigurationError):
            LMStudioControllerActionPlanner(reasoning_effort="maximum")

    def test_custom_utterance_limit_is_shared_by_schema_and_decoder(self):
        maximum = 120
        accepted, accepted_transport = self.planner(
            completion({
                "action": "DRIVE_FORWARD",
                "confidence_milli": 940,
                "assessment": "Det är fritt framåt.",
                "plan": ["DRIVE_FORWARD"],
                "utterance": "x" * maximum,
            }),
            max_utterance_chars=maximum,
        )

        result = accepted.decide(context())

        self.assertEqual(len(result.decision.utterance), maximum)
        request = json.loads(accepted_transport.calls[0][1])
        utterance_schema = request["response_format"]["json_schema"][
            "schema"
        ]["properties"]["utterance"]["oneOf"][0]
        self.assertEqual(utterance_schema["maxLength"], maximum)
        self.assertIn(
            "at or below 120 Unicode characters",
            request["messages"][0]["content"],
        )

        rejected, _ = self.planner(
            completion({
                "action": "DRIVE_FORWARD",
                "confidence_milli": 940,
                "assessment": "Det är fritt framåt.",
                "plan": ["DRIVE_FORWARD"],
                "utterance": "x" * (maximum + 1),
            }),
            max_utterance_chars=maximum,
        )
        with self.assertRaises(LMStudioProtocolError):
            rejected.decide(context())

    def test_blast_bound_rejects_the_live_eight_second_utterance(self):
        live_utterance = (
            "Nu rör vi oss! 46 millimeter är en bra början, men vi har "
            "inte ens sett hindren än. Dags att skanna av terrängen så jag激"
        )
        self.assertEqual(len(live_utterance), 120)
        planner, _ = self.planner(
            completion({
                "action": SCAN_FRONT_ARC,
                "confidence_milli": 900,
                "assessment": "Terrängen behöver skannas.",
                "plan": [SCAN_FRONT_ARC],
                "utterance": live_utterance,
            }),
            max_utterance_chars=72,
        )

        with self.assertRaises(LMStudioProtocolError):
            planner.decide(context(
                available_actions=(SCAN_FRONT_ARC,),
            ))

    def test_rejects_invalid_utterance_limits(self):
        for maximum in (
            False,
            0,
            MAX_UTTERANCE_CHARS + 1,
            120.0,
            "120",
        ):
            with self.subTest(maximum=maximum), self.assertRaises(
                LMStudioConfigurationError
            ):
                LMStudioControllerActionPlanner(
                    max_utterance_chars=maximum
                )

    def test_blast_persona_changes_only_the_locale_specific_system_prompt(self):
        response = completion({
            "action": "DRIVE_FORWARD",
            "confidence_milli": 940,
            "assessment": "Det är fritt framåt.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": "Flytta på dig, låda.",
        })
        default_planner, default_transport = self.planner(response)
        blast_planner, blast_transport = self.planner(
            response,
            utterance_persona_by_locale=BLAST_PERSONA_BY_LOCALE,
        )

        default_planner.decide(context())
        blast_planner.decide(context())

        default_payload = json.loads(default_transport.calls[0][1])
        blast_payload = json.loads(blast_transport.calls[0][1])
        default_prompt = default_payload["messages"][0]["content"]
        blast_prompt = blast_payload["messages"][0]["content"]
        self.assertTrue(blast_prompt.startswith(default_prompt))
        self.assertIn(BLAST_PERSONA_BY_LOCALE["sv"], blast_prompt)
        self.assertNotIn(BLAST_PERSONA_BY_LOCALE["en"], blast_prompt)
        for guardrail in (
            "only to the wording and tone of utterance",
            "never influence action",
            "assessment",
            "sensor facts",
            "safety",
            "COMPLETE/ABORT decisions",
        ):
            self.assertIn(guardrail, blast_prompt)
        blast_payload["messages"][0]["content"] = default_prompt
        self.assertEqual(blast_payload, default_payload)

        english_planner, english_transport = self.planner(
            response,
            utterance_persona_by_locale=BLAST_PERSONA_BY_LOCALE,
        )
        english_planner.decide(context(locale="en"))
        english_prompt = json.loads(english_transport.calls[0][1])[
            "messages"
        ][0]["content"]
        self.assertIn(BLAST_PERSONA_BY_LOCALE["en"], english_prompt)
        self.assertNotIn(BLAST_PERSONA_BY_LOCALE["sv"], english_prompt)

    def test_terminal_decision_discards_valid_stale_plan_tail(self):
        planner, _ = self.planner(completion({
            "action": COMPLETE,
            "confidence_milli": 900,
            "assessment": "Målet är uppnått.",
            "plan": [],
            "utterance": None,
        }))
        self.assertEqual(
            planner.decide(context()).decision.action,
            COMPLETE,
        )

        invalid, _ = self.planner(completion({
            "action": COMPLETE,
            "confidence_milli": 900,
            "assessment": "Målet är uppnått.",
            "plan": [COMPLETE],
            "utterance": None,
        }))
        self.assertEqual(
            invalid.decide(context()).decision.plan,
            (),
        )

        stale, _ = self.planner(completion({
            "action": COMPLETE,
            "confidence_milli": 900,
            "assessment": "Målet är uppnått.",
            "plan": ["TURN_LEFT"],
            "utterance": None,
        }))
        self.assertEqual(stale.decide(context()).decision.plan, ())

    def test_completion_can_be_withheld_by_the_host(self):
        planner, transport = self.planner(completion({
            "action": "TURN_LEFT",
            "confidence_milli": 800,
            "assessment": "Jag måste verifiera slutläget först.",
            "plan": ["TURN_LEFT"],
            "utterance": None,
        }))

        planner.decide(context(completion_allowed=False))

        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        action_schema = request["response_format"]["json_schema"][
            "schema"
        ]["properties"]["action"]
        self.assertFalse(supplied["completion_allowed"])
        self.assertNotIn(COMPLETE, action_schema["enum"])
        self.assertIn("ABORT", action_schema["enum"])

        invalid, _ = self.planner(completion({
            "action": COMPLETE,
            "confidence_milli": 900,
            "assessment": "Klart.",
            "plan": [],
            "utterance": None,
        }))
        with self.assertRaises(LMStudioProtocolError):
            invalid.decide(context(completion_allowed=False))

    def test_abort_can_be_withheld_while_safe_actions_remain(self):
        planner, transport = self.planner(completion({
            "action": "TURN_LEFT",
            "confidence_milli": 800,
            "assessment": "Jag provar en annan säker observation.",
            "plan": ["TURN_LEFT"],
            "utterance": None,
        }))

        planner.decide(context(
            completion_allowed=False,
            abort_allowed=False,
        ))

        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        choices = request["response_format"]["json_schema"][
            "schema"
        ]["properties"]["action"]["enum"]
        self.assertFalse(supplied["abort_allowed"])
        self.assertNotIn("ABORT", choices)
        self.assertNotIn(COMPLETE, choices)

        invalid, _ = self.planner(completion({
            "action": "ABORT",
            "confidence_milli": 900,
            "assessment": "Avbryt.",
            "plan": [],
            "utterance": None,
        }))
        with self.assertRaises(LMStudioProtocolError):
            invalid.decide(context(
                completion_allowed=False,
                abort_allowed=False,
            ))

    def test_empty_motion_actions_expose_only_terminal_choices(self):
        planner, transport = self.planner(completion({
            "action": COMPLETE,
            "confidence_milli": 900,
            "assessment": "Målet är verifierat trots den blockerade fronten.",
            "plan": [],
            "utterance": None,
        }))

        result = planner.decide(context(available_actions=()))

        self.assertEqual(result.decision.action, COMPLETE)
        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        action_schema = request["response_format"]["json_schema"][
            "schema"
        ]["properties"]["action"]
        self.assertEqual(supplied["available_actions"], [])
        self.assertEqual(action_schema["enum"], [COMPLETE, "ABORT"])
        with self.assertRaises(LMStudioInputError):
            context(available_actions=(), completion_allowed=False)

    def test_scan_action_is_described_as_a_full_surroundings_sweep(self):
        planner, transport = self.planner(completion({
            "action": SCAN_FRONT_ARC,
            "confidence_milli": 900,
            "assessment": "Jag behöver se båda sidorna.",
            "plan": [SCAN_FRONT_ARC],
            "utterance": None,
        }))

        result = planner.decide(context(available_actions=(
            "TURN_LEFT",
            "TURN_RIGHT",
            SCAN_FRONT_ARC,
        )))

        self.assertEqual(result.decision.action, SCAN_FRONT_ARC)
        request = json.loads(transport.calls[0][1])
        system_prompt = request["messages"][0]["content"]
        self.assertIn(SCAN_FRONT_ARC, system_prompt)
        self.assertIn("front half-space", system_prompt)
        self.assertIn("returns to its starting direction", system_prompt)

    def test_side_scan_context_and_prompt_preserve_gemmas_side_choice(self):
        side_scan = {
            "schema": "blast-robot-relative-side-scan/v2",
            "frame": "ROBOT_RELATIVE_AT_SCAN_START",
            "physical_side_labels_authoritative": True,
            "rays": {
                "left": [{
                    "range_state": "MEASURED",
                    "distance_mm": 246,
                    "absolute_bearing_deg": 23.0,
                }, {
                    "range_state": "MEASURED",
                    "distance_mm": 347,
                    "absolute_bearing_deg": 70.0,
                }],
                "right": [{
                    "range_state": "MEASURED",
                    "distance_mm": 202,
                    "absolute_bearing_deg": 23.0,
                }, {
                    "range_state": "MEASURED",
                    "distance_mm": 1_002,
                    "absolute_bearing_deg": 70.0,
                }],
            },
        }
        planner, transport = self.planner(completion({
            "action": TURN_RIGHT_90,
            "confidence_milli": 900,
            "assessment": "Jag jämför båda sidornas hela scanmönster.",
            "plan": [TURN_RIGHT_90],
            "utterance": None,
        }))

        planner.decide(context(
            available_actions=(TURN_LEFT_90, TURN_RIGHT_90),
            robot_relative_side_scan=side_scan,
        ))

        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        self.assertEqual(supplied["robot_relative_side_scan"], side_scan)
        self.assertEqual(
            supplied["available_actions"],
            [TURN_LEFT_90, TURN_RIGHT_90],
        )
        self.assertNotIn(
            "heading",
            json.dumps(supplied["robot_relative_side_scan"]),
        )
        system_prompt = request["messages"][0]["content"]
        for instruction in (
            "physical left/right at scan start",
            "full angular pattern on both sides",
            "NO_VALID_DISTANCE",
            "mean unknown",
            "you choose the route, detour side",
        ):
            self.assertIn(instruction, system_prompt)

    def test_local_map_evidence_is_optional_echo_only_context(self):
        local_map = {
            "schema": "blast-local-map-evidence/v1",
            "frame": "EPISODE_LOCAL_ODOMETRY",
            "robot_pose": {"x_mm": 45, "y_mm": 0, "heading_mdeg": 0},
            "directional_goal": {
                "target_x_mm": 420,
                "target_y_mm": 0,
                "remaining_forward_progress_mm": 375,
            },
            "scan_views": [{
                "scan_id": "episode-a-scan-1",
                "scan_pose": {"x_mm": 0, "y_mm": 0, "heading_mdeg": 0},
                "echo_points": [{"x_mm": 250, "y_mm": -180}],
            }],
            "unobserved_space": "UNKNOWN_NOT_FREE",
            "occupancy_model": "NONE",
        }
        planner, transport = self.planner(completion({
            "action": SCAN_FRONT_ARC,
            "confidence_milli": 800,
            "assessment": "Jag behöver undersöka den okända korridoren.",
            "plan": [SCAN_FRONT_ARC],
            "utterance": None,
        }))

        planner.decide(context(
            available_actions=(SCAN_FRONT_ARC,),
            local_map_evidence=local_map,
        ))

        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        self.assertEqual(supplied["local_map_evidence"], local_map)
        system_prompt = request["messages"][0]["content"]
        self.assertLess(len(system_prompt), 6_000)
        self.assertIn("SAME fixed episode frame", system_prompt)
        self.assertIn("x is rows[i].x_mm and y is column_y_mm[j]", system_prompt)
        self.assertIn("+x is starting forward", system_prompt)
        self.assertIn("+y is starting left", system_prompt)
        self.assertIn("-y is starting right", system_prompt)
        self.assertIn("direct_goal_blockage", system_prompt)
        self.assertIn("visited_cells", system_prompt)
        self.assertIn("current_echo_clusters", system_prompt)
        self.assertIn("robot_center_keep_out_bounds_mm", system_prompt)
        self.assertIn("direct_detour_axis_candidates", system_prompt)
        self.assertIn("not a chosen route", system_prompt)
        self.assertNotIn("local_map_evidence", context().to_dict())

        with self.assertRaises(LMStudioInputError):
            context(local_map_evidence=[])

    def test_waypoint_round_trips_as_model_owned_advisory_memory(self):
        waypoint = {
            "x_mm": 120,
            "y_mm": -280,
            "purpose": "Pass the obstacle on its open right side",
        }
        following = ({
            "x_mm": 500,
            "y_mm": -280,
            "purpose": "Pass beyond the obstacle",
        }, {
            "x_mm": 800,
            "y_mm": 0,
            "purpose": "Return toward the final goal",
        })
        planner, transport = self.planner(completion({
            "action": "DRIVE_FORWARD",
            "confidence_milli": 850,
            "assessment": "I am still approaching my chosen detour point.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": None,
            "waypoint": waypoint,
            "following_waypoints": list(following),
        }))

        result = planner.decide(context(
            active_waypoint=waypoint,
            active_waypoint_geometry={
                "distance_mm": 304,
                "bearing_deg": -67,
                "heading_error_deg": -60,
            },
            active_waypoint_plan=(waypoint, *following),
        ))

        self.assertEqual(result.decision.waypoint, waypoint)
        self.assertEqual(result.decision.following_waypoints, following)
        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        self.assertEqual(supplied["active_waypoint"], waypoint)
        self.assertEqual(supplied["active_waypoint_geometry"], {
            "distance_mm": 304,
            "bearing_deg": -67,
            "heading_error_deg": -60,
        })
        self.assertEqual(
            supplied["active_waypoint_plan"], [waypoint, *following],
        )
        self.assertIn(
            "Keep the route until reached, blocked or disproved",
            request["messages"][0]["content"],
        )
        self.assertIn(
            "waypoint",
            request["response_format"]["json_schema"]["schema"][
                "required"
            ],
        )
        self.assertIn(
            "following_waypoints",
            request["response_format"]["json_schema"]["schema"][
                "required"
            ],
        )
        with self.assertRaises(LMStudioInputError):
            context(active_waypoint={**waypoint, "host_executes": True})
        with self.assertRaises(LMStudioInputError):
            context(active_waypoint_geometry={
                "distance_mm": 304,
                "bearing_deg": -67,
            })

    def test_adjacent_duplicate_waypoints_are_removed(self):
        waypoint = {
            "x_mm": 0,
            "y_mm": 450,
            "purpose": "Create side clearance",
        }
        later = {
            "x_mm": 600,
            "y_mm": 450,
            "purpose": "Pass the obstacle",
        }
        planner, _transport = self.planner(completion({
            "action": FOLLOW_WAYPOINT,
            "confidence_milli": 850,
            "assessment": "Follow the route.",
            "plan": [FOLLOW_WAYPOINT],
            "utterance": None,
            "waypoint": waypoint,
            "following_waypoints": [waypoint, waypoint, later],
        }))

        result = planner.decide(context(
            available_actions=(FOLLOW_WAYPOINT,),
        ))

        self.assertEqual(result.decision.waypoint, waypoint)
        self.assertEqual(result.decision.following_waypoints, (later,))

        invalid, _ = self.planner(completion({
            "action": "DRIVE_FORWARD",
            "confidence_milli": 850,
            "assessment": "The proposed waypoint is outside the episode.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": None,
            "waypoint": {**waypoint, "x_mm": 5_001},
        }))
        with self.assertRaises(LMStudioProtocolError):
            invalid.decide(context())

    def test_off_axis_context_requires_waypoint_and_defers_complete(self):
        waypoint = {
            "x_mm": 120,
            "y_mm": 300,
            "purpose": "Reach the open side before returning to the goal",
        }
        planner, transport = self.planner(completion({
            "action": "TURN_RIGHT",
            "confidence_milli": 850,
            "assessment": "I need a visible detour target first.",
            "plan": ["TURN_RIGHT", "DRIVE_FORWARD"],
            "utterance": None,
            "waypoint": waypoint,
        }))

        result = planner.decide(context(
            waypoint_required=True,
            completion_allowed=False,
            abort_allowed=False,
        ))

        self.assertEqual(result.decision.waypoint, waypoint)
        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        properties = request["response_format"]["json_schema"]["schema"][
            "properties"
        ]
        self.assertTrue(supplied["waypoint_required"])
        self.assertEqual(properties["waypoint"]["type"], "object")
        self.assertNotIn("oneOf", properties["waypoint"])
        self.assertNotIn("COMPLETE", properties["plan"]["items"]["enum"])

        invalid, _ = self.planner(completion({
            "action": "TURN_RIGHT",
            "confidence_milli": 850,
            "assessment": "I omitted the required detour target.",
            "plan": ["TURN_RIGHT"],
            "utterance": None,
            "waypoint": None,
        }))
        with self.assertRaises(LMStudioProtocolError):
            invalid.decide(context(
                waypoint_required=True,
                completion_allowed=False,
                abort_allowed=False,
            ))
        with self.assertRaises(LMStudioInputError):
            context(waypoint_required="yes")

    def test_follow_waypoint_requires_model_owned_waypoint(self):
        waypoint = {
            "x_mm": 300,
            "y_mm": -300,
            "purpose": "Follow the chosen right-side detour leg",
        }
        planner, transport = self.planner(completion({
            "action": FOLLOW_WAYPOINT,
            "confidence_milli": 850,
            "assessment": "I will execute my current waypoint.",
            "plan": [FOLLOW_WAYPOINT],
            "utterance": None,
            "waypoint": waypoint,
        }))

        result = planner.decide(context(
            available_actions=(FOLLOW_WAYPOINT,),
        ))

        self.assertEqual(result.decision.waypoint, waypoint)
        request = json.loads(transport.calls[0][1])
        prompt = request["messages"][0]["content"]
        self.assertIn("follows only the current waypoint", prompt)
        self.assertIn("you choose the route, detour side, waypoints", prompt)

        invalid, _ = self.planner(completion({
            "action": FOLLOW_WAYPOINT,
            "confidence_milli": 850,
            "assessment": "I omitted the waypoint I asked to follow.",
            "plan": [FOLLOW_WAYPOINT],
            "utterance": None,
            "waypoint": None,
        }))
        with self.assertRaises(LMStudioProtocolError):
            invalid.decide(context(
                available_actions=(FOLLOW_WAYPOINT,),
            ))

    def test_nonterminal_plan_is_aligned_with_selected_action(self):
        planner, _ = self.planner(completion({
            "action": "TURN_LEFT",
            "confidence_milli": 800,
            "assessment": "Jag behöver vrida mig.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": None,
        }))

        self.assertEqual(
            planner.decide(context()).decision.plan,
            ("TURN_LEFT", "DRIVE_FORWARD"),
        )

    def test_empty_nonterminal_plan_is_aligned_with_selected_action(self):
        planner, _ = self.planner(completion({
            "action": "TURN_LEFT",
            "confidence_milli": 800,
            "assessment": "Jag behöver vrida mig.",
            "plan": [],
            "utterance": None,
        }))

        self.assertEqual(
            planner.decide(context()).decision.plan,
            ("TURN_LEFT",),
        )

    def test_minimal_semantic_advance_keeps_bounded_goal_continuation(self):
        planner, _ = self.planner(completion({
            "action": ADVANCE,
            "confidence_milli": 900,
            "assessment": "The direct path remains clear.",
            "plan": [ADVANCE],
            "utterance": None,
        }))

        decision = planner.decide(context(
            available_actions=(ADVANCE,),
            plan_actions=(ADVANCE,),
            completion_allowed=False,
        )).decision

        self.assertEqual(decision.plan, (ADVANCE, COMPLETE))

    def test_harmless_text_whitespace_is_normalized(self):
        planner, _ = self.planner(completion({
            "action": "TURN_LEFT",
            "confidence_milli": 800,
            "assessment": "  Jag behöver vrida mig.  ",
            "plan": ["TURN_LEFT"],
            "utterance": "  Nu svänger jag.  ",
        }))

        decision = planner.decide(context()).decision

        self.assertEqual(decision.assessment, "Jag behöver vrida mig.")
        self.assertEqual(decision.utterance, "Nu svänger jag.")

    def test_first_tail_waypoint_is_promoted_when_current_is_null(self):
        first = {
            "x_mm": 300,
            "y_mm": -300,
            "purpose": "Create lateral clearance",
        }
        second = {
            "x_mm": 600,
            "y_mm": -300,
            "purpose": "Pass the obstacle",
        }
        planner, _ = self.planner(completion({
            "action": "TURN_RIGHT",
            "confidence_milli": 800,
            "assessment": "I will use the open side.",
            "plan": ["TURN_RIGHT", "DRIVE_FORWARD"],
            "utterance": None,
            "waypoint": None,
            "following_waypoints": [first, second],
        }))

        decision = planner.decide(context()).decision

        self.assertEqual(decision.waypoint, first)
        self.assertEqual(decision.following_waypoints, (second,))

    def test_plan_can_hypothesize_actions_not_available_at_current_pose(self):
        future = (TURN_RIGHT_90, "DRIVE_FORWARD", TURN_LEFT_90)
        planner, transport = self.planner(completion({
            "action": TURN_RIGHT_90,
            "confidence_milli": 900,
            "assessment": "Öppningen finns åt höger.",
            "plan": [*future, COMPLETE],
            "utterance": None,
        }))

        result = planner.decide(context(
            available_actions=(TURN_RIGHT_90,),
            plan_actions=future,
            active_plan=("DRIVE_FORWARD", TURN_LEFT_90, COMPLETE),
        ))

        self.assertEqual(result.decision.plan, (*future, COMPLETE))
        request = json.loads(transport.calls[0][1])
        supplied = json.loads(request["messages"][1]["content"])
        self.assertEqual(supplied["available_actions"], [TURN_RIGHT_90])
        self.assertEqual(supplied["plan_actions"], list(future))
        self.assertEqual(
            supplied["active_plan"],
            ["DRIVE_FORWARD", TURN_LEFT_90, COMPLETE],
        )
        plan_schema = request["response_format"]["json_schema"][
            "schema"
        ]["properties"]["plan"]
        self.assertIn("DRIVE_FORWARD", plan_schema["items"]["enum"])

    def test_model_cannot_invent_an_action(self):
        planner, _ = self.planner(completion({
            "action": "JUMP",
            "confidence_milli": 999,
            "assessment": "Hoppa.",
            "plan": ["JUMP"],
            "utterance": None,
        }))
        with self.assertRaises(LMStudioProtocolError):
            planner.decide(context())

    def test_context_rejects_invalid_json_and_action_sets(self):
        invalid = (
            {"observation": {"value": float("nan")}},
            {"observation": {"value": object()}},
            {"available_actions": ("DRIVE_FORWARD", "DRIVE_FORWARD")},
            {"available_actions": ("COMPLETE",)},
            {"available_actions": ([],)},
            {"history": ({"value": object()},)},
        )
        for changes in invalid:
            with self.subTest(changes=changes), self.assertRaises(
                LMStudioInputError
            ):
                context(**changes)

    def test_rejects_invalid_completion_envelopes(self):
        valid = {
            "action": "DRIVE_FORWARD",
            "confidence_milli": 900,
            "assessment": "Fortsätt.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": None,
        }
        invalid = (
            b"{}",
            completion(valid, model="other/model"),
            completion(valid, choices=[]),
            completion(valid, choices=[{
                "index": False,
                "finish_reason": "stop",
                "message": {
                    "role": "assistant",
                    "content": json.dumps(valid),
                },
            }]),
            completion({**valid, "confidence_milli": True}),
            completion({**valid, "extra": "no"}),
            completion({**valid, "utterance": ""}),
        )
        for response in invalid:
            with self.subTest(response=response[:80]):
                planner, _ = self.planner(response)
                with self.assertRaises(LMStudioProtocolError):
                    planner.decide(context())

    def test_rejects_invalid_persona_configuration(self):
        invalid = (
            {},
            {"sv": "Bara svenska."},
            {"sv": "Svenska.", "en": "English.", "de": "Deutsch."},
            ("sv", "en"),
            {"sv": " Svenska.", "en": "English."},
            {"sv": "Svenska.\n", "en": "English."},
            {"sv": "x" * (MAX_PERSONA_CHARS + 1), "en": "English."},
        )
        for persona in invalid:
            with self.subTest(persona=persona), self.assertRaises(
                LMStudioConfigurationError
            ):
                LMStudioControllerActionPlanner(
                    utterance_persona_by_locale=persona
                )


if __name__ == "__main__":
    unittest.main()
