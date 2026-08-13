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
    LMStudioControllerActionPlanner,
    MAX_UTTERANCE_CHARS,
)
from robot_agent.physical_navigation_contract import (
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

    def test_terminal_decision_normalizes_one_redundant_terminal_step(self):
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

        invalid, _ = self.planner(completion({
            "action": COMPLETE,
            "confidence_milli": 900,
            "assessment": "Målet är uppnått.",
            "plan": ["TURN_LEFT"],
            "utterance": None,
        }))
        with self.assertRaises(LMStudioProtocolError):
            invalid.decide(context())

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

    def test_scan_action_is_described_as_a_returning_two_sided_sweep(self):
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
        self.assertIn("both sides", system_prompt)
        self.assertIn("returns near its starting heading", system_prompt)

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
            "left and right arrays",
            "authoritative physical sides",
            "smallest to largest absolute_bearing_deg",
            "Ignore conflicting raw heading signs",
            "complete angular pattern on both sides",
            "larger distance_mm means a farther return",
            "far-angle measured opening",
            "NO_VALID_DISTANCE",
            "mean unknown",
            "host does not rank or choose the turn side",
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
        self.assertIn("accumulated echo points", system_prompt)
        self.assertIn("Unobserved space is unknown, never free", system_prompt)
        self.assertIn("has not selected a corridor", system_prompt)
        self.assertNotIn("local_map_evidence", context().to_dict())

        with self.assertRaises(LMStudioInputError):
            context(local_map_evidence=[])

    def test_nonterminal_plan_must_start_with_selected_action(self):
        planner, _ = self.planner(completion({
            "action": "TURN_LEFT",
            "confidence_milli": 800,
            "assessment": "Jag behöver vrida mig.",
            "plan": ["DRIVE_FORWARD"],
            "utterance": None,
        }))
        with self.assertRaises(LMStudioProtocolError):
            planner.decide(context())

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
