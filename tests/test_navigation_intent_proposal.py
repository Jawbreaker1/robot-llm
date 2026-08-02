import json
import unittest

import robot_agent.navigation_intent_proposal as shadow_contract
from robot_agent.lm_studio_navigation import _response_schema
from robot_agent.maneuver_commitment import empty_commitment
from robot_agent.navigation_intent_proposal import (
    ABORT,
    DETOUR_TARGET,
    FOLLOW_DIRECTION,
    HOLD,
    LEFT,
    MAX_NAVIGATION_INTENT_TTL_MS,
    MAX_NAVIGATION_INTENT_REASON_LENGTH,
    NavigationIntentOffer,
    NavigationIntentProposal,
    NavigationIntentProposalError,
    RIGHT,
    SCAN_TARGET,
    bind_navigation_intent_proposal,
    build_navigation_intent_proposal_schema,
    decode_navigation_intent_proposal,
)
from robot_agent.physical_agent_state import ControllerKey, NavigationBasis
from robot_agent.physical_navigation_contract import (
    ACTIONS,
    ADVANCE,
)


def basis(**changes):
    values = {
        "controller_key": ControllerKey(
            robot_id="ev3rstorm-1",
            controller_id="ev3-navigation",
            controller_instance_id="controller-a",
        ),
        "goal_epoch": 2,
        "controller_state_version": 11,
        "world_generation_id": "map-generation-a",
        "world_model_version": 8,
        "navigation_basis_id": "navigation-evidence-14",
        "frame_id": "map-frame-a",
        "calibration_fingerprint": "ev3-drive-calibration-a",
    }
    values.update(changes)
    return NavigationBasis(**values)


def offer(**changes):
    values = {
        "ticket_id": "ticket-11",
        "basis": basis(),
        "offered_intents": (
            ABORT,
            HOLD,
            DETOUR_TARGET,
            SCAN_TARGET,
            FOLLOW_DIRECTION,
        ),
        "scan_target_ids": ("hazard-2", "hazard-1"),
        "detour_target_ids": ("hazard-2",),
        "detour_sides": (RIGHT, LEFT),
        "hold_reasons": ("WAIT_FOR_EVIDENCE", "NO_CLEAR_ROUTE"),
        "abort_reasons": ("GOAL_WITHDRAWN", "LOCALIZATION_LOST"),
    }
    values.update(changes)
    return NavigationIntentOffer(**values)


def encoded(value):
    return json.dumps(value, separators=(",", ":")).encode("utf-8")


def canonical_size(value):
    return len(
        json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    )


def size_metrics(*, current_schema, current_output, current_offer, proposal):
    current_offer.assert_allows(proposal)
    shadow_schema = build_navigation_intent_proposal_schema(current_offer)
    shadow_output = proposal.to_dict()
    current_schema_bytes = canonical_size(current_schema)
    shadow_schema_bytes = canonical_size(shadow_schema)
    current_output_bytes = canonical_size(current_output)
    shadow_output_bytes = canonical_size(shadow_output)
    return {
        "schema_saved_bytes": current_schema_bytes - shadow_schema_bytes,
        "schema_reduction_milli": (
            (current_schema_bytes - shadow_schema_bytes) * 1_000
        ) // current_schema_bytes,
        "output_saved_bytes": current_output_bytes - shadow_output_bytes,
        "output_reduction_milli": (
            (current_output_bytes - shadow_output_bytes) * 1_000
        ) // current_output_bytes,
        "shadow_schema_bytes": shadow_schema_bytes,
        "shadow_output_bytes": shadow_output_bytes,
    }


class NavigationIntentSchemaTests(unittest.TestCase):
    def test_contract_reexports_the_canonical_navigation_basis(self):
        self.assertIs(shadow_contract.NavigationBasis, NavigationBasis)

    def test_schema_is_strict_deterministic_and_contains_no_host_echo(self):
        first = build_navigation_intent_proposal_schema(offer())
        second = build_navigation_intent_proposal_schema(offer())

        self.assertEqual(first, second)
        self.assertEqual(set(first), {"oneOf"})
        self.assertEqual(len(first["oneOf"]), 5)
        by_intent = {
            variant["properties"]["intent"]["const"]: variant
            for variant in first["oneOf"]
        }
        self.assertEqual(
            list(by_intent),
            [
                FOLLOW_DIRECTION,
                SCAN_TARGET,
                DETOUR_TARGET,
                HOLD,
                ABORT,
            ],
        )
        for variant in first["oneOf"]:
            self.assertIs(variant["additionalProperties"], False)
            self.assertEqual(
                variant["required"],
                sorted(variant["properties"]),
            )
        self.assertEqual(
            by_intent[SCAN_TARGET]["properties"]["target_id"]["enum"],
            ["hazard-1", "hazard-2"],
        )
        self.assertEqual(
            by_intent[DETOUR_TARGET]["properties"]["target_id"]["enum"],
            ["hazard-2"],
        )
        self.assertEqual(
            by_intent[DETOUR_TARGET]["properties"]["side"]["enum"],
            [LEFT, RIGHT],
        )

        serialized = json.dumps(first, sort_keys=True)
        for forbidden in (
            "schema",
            "episode_id",
            "ticket_id",
            "proposal_id",
            "turn",
            "state_version",
            "world_model_version",
            "utterance",
            "assessment",
            "commentary",
        ):
            self.assertNotIn('"{}"'.format(forbidden), serialized)

    def test_schema_contains_only_offered_variants(self):
        current_offer = offer(
            offered_intents=(FOLLOW_DIRECTION, SCAN_TARGET),
            detour_target_ids=(),
            detour_sides=(),
            hold_reasons=(),
            abort_reasons=(),
        )

        schema = build_navigation_intent_proposal_schema(current_offer)

        self.assertEqual(
            [
                item["properties"]["intent"]["const"]
                for item in schema["oneOf"]
            ],
            [FOLLOW_DIRECTION, SCAN_TARGET],
        )

    def test_offer_rejects_missing_or_irrelevant_enum_values(self):
        invalid_changes = (
            {
                "offered_intents": (FOLLOW_DIRECTION, SCAN_TARGET),
                "scan_target_ids": (),
                "detour_target_ids": (),
                "detour_sides": (),
                "hold_reasons": (),
                "abort_reasons": (),
            },
            {
                "offered_intents": (FOLLOW_DIRECTION,),
                "scan_target_ids": ("hazard-1",),
                "detour_target_ids": (),
                "detour_sides": (),
                "hold_reasons": (),
                "abort_reasons": (),
            },
            {"detour_sides": ("UP",)},
            {"offered_intents": (FOLLOW_DIRECTION, FOLLOW_DIRECTION)},
        )
        for changes in invalid_changes:
            with self.subTest(changes=changes):
                with self.assertRaises(NavigationIntentProposalError):
                    offer(**changes)

    def test_offer_cannot_expand_the_model_schema_without_bound(self):
        with self.assertRaises(NavigationIntentProposalError) as caught:
            offer(
                scan_target_ids=tuple(
                    "hazard-{:02d}-{}".format(index, "x" * 100)
                    for index in range(40)
                ),
                detour_target_ids=("hazard-2",),
            )

        self.assertEqual(caught.exception.code, "offer_schema_too_large")

    def test_offer_reason_limit_matches_the_decoded_proposal_contract(self):
        maximum = "R" * MAX_NAVIGATION_INTENT_REASON_LENGTH
        current_offer = offer(
            hold_reasons=(maximum,),
            abort_reasons=(maximum,),
        )

        self.assertEqual(
            decode_navigation_intent_proposal(
                encoded({"intent": HOLD, "reason": maximum}),
                current_offer,
            ),
            NavigationIntentProposal(HOLD, reason=maximum),
        )
        with self.assertRaises(NavigationIntentProposalError) as caught:
            offer(hold_reasons=(maximum + "R",))
        self.assertEqual(caught.exception.code, "invalid_offer")

    def test_detour_sides_cannot_form_ambiguous_cross_target_choices(self):
        with self.assertRaises(NavigationIntentProposalError) as caught:
            offer(detour_target_ids=("hazard-1", "hazard-2"))

        self.assertEqual(caught.exception.code, "invalid_offer")


class NavigationIntentDecodeTests(unittest.TestCase):
    def test_decodes_each_minimal_variant(self):
        cases = (
            (
                {"intent": FOLLOW_DIRECTION},
                NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            ),
            (
                {"intent": SCAN_TARGET, "target_id": "hazard-1"},
                NavigationIntentProposal(
                    intent=SCAN_TARGET,
                    target_id="hazard-1",
                ),
            ),
            (
                {
                    "intent": DETOUR_TARGET,
                    "target_id": "hazard-2",
                    "side": LEFT,
                },
                NavigationIntentProposal(
                    intent=DETOUR_TARGET,
                    target_id="hazard-2",
                    side=LEFT,
                ),
            ),
            (
                {"intent": HOLD, "reason": "NO_CLEAR_ROUTE"},
                NavigationIntentProposal(
                    intent=HOLD,
                    reason="NO_CLEAR_ROUTE",
                ),
            ),
            (
                {"intent": ABORT, "reason": "GOAL_WITHDRAWN"},
                NavigationIntentProposal(
                    intent=ABORT,
                    reason="GOAL_WITHDRAWN",
                ),
            ),
        )
        for value, expected in cases:
            with self.subTest(intent=value["intent"]):
                decoded = decode_navigation_intent_proposal(
                    encoded(value),
                    offer(),
                )
                self.assertEqual(decoded, expected)
                self.assertEqual(decoded.to_dict(), value)

    def test_rejects_non_strict_json_and_exact_field_violations(self):
        cases = (
            b'{"intent":"FOLLOW_DIRECTION","intent":"ABORT"}',
            b'{"intent":"FOLLOW_DIRECTION","utterance":"go"}',
            b'{"intent":"SCAN_TARGET"}',
            b'{"intent":"HOLD","reason":"NO_CLEAR_ROUTE","side":"LEFT"}',
            b'{"intent":NaN}',
            b'[]',
            b'{',
            b'',
        )
        for raw in cases:
            with self.subTest(raw=raw):
                with self.assertRaises(NavigationIntentProposalError):
                    decode_navigation_intent_proposal(raw, offer())
        with self.assertRaises(NavigationIntentProposalError):
            decode_navigation_intent_proposal(
                '{"intent":"FOLLOW_DIRECTION"}',
                offer(),
            )

    def test_rejects_values_outside_host_offered_enums(self):
        cases = (
            {"intent": SCAN_TARGET, "target_id": "hazard-unknown"},
            {
                "intent": DETOUR_TARGET,
                "target_id": "hazard-1",
                "side": LEFT,
            },
            {
                "intent": DETOUR_TARGET,
                "target_id": "hazard-2",
                "side": "UP",
            },
            {"intent": HOLD, "reason": "MODEL_INVENTED_REASON"},
            {"intent": ABORT, "reason": "MODEL_INVENTED_REASON"},
        )
        for value in cases:
            with self.subTest(value=value):
                with self.assertRaises(NavigationIntentProposalError):
                    decode_navigation_intent_proposal(
                        encoded(value),
                        offer(),
                    )

    def test_rejects_an_intent_that_is_well_formed_but_not_offered(self):
        follow_only = offer(
            offered_intents=(FOLLOW_DIRECTION,),
            scan_target_ids=(),
            detour_target_ids=(),
            detour_sides=(),
            hold_reasons=(),
            abort_reasons=(),
        )

        with self.assertRaises(NavigationIntentProposalError) as caught:
            decode_navigation_intent_proposal(
                encoded({
                    "intent": SCAN_TARGET,
                    "target_id": "hazard-1",
                }),
                follow_only,
            )

        self.assertEqual(caught.exception.code, "unoffered_intent")


class NavigationIntentEnvelopeTests(unittest.TestCase):
    def test_host_binds_identity_basis_and_ttl_after_decode(self):
        current_offer = offer()
        proposal = decode_navigation_intent_proposal(
            encoded({"intent": FOLLOW_DIRECTION}),
            current_offer,
        )

        envelope = bind_navigation_intent_proposal(
            proposal,
            offer=current_offer,
            proposal_id="host-proposal-17",
            received_at_ms=10_000,
            valid_until_ms=20_000,
        )

        self.assertEqual(envelope.ticket_id, current_offer.ticket_id)
        self.assertIs(envelope.basis, current_offer.basis)
        self.assertEqual(envelope.proposal, proposal)
        envelope.assert_current(
            proposal_id="host-proposal-17",
            ticket_id="ticket-11",
            basis=current_offer.basis,
            now_ms=19_999,
        )
        self.assertEqual(
            envelope.to_dict()["proposal"],
            {"intent": FOLLOW_DIRECTION},
        )
        self.assertEqual(
            envelope.to_dict()["basis"]["navigation_basis_id"],
            "navigation-evidence-14",
        )

    def test_rejects_invalid_ttl(self):
        proposal = NavigationIntentProposal(intent=FOLLOW_DIRECTION)
        invalid_windows = (
            (10_000, 10_000),
            (10_000, 9_999),
            (10_000, 10_000 + MAX_NAVIGATION_INTENT_TTL_MS + 1),
        )
        for received_at_ms, valid_until_ms in invalid_windows:
            with self.subTest(
                received_at_ms=received_at_ms,
                valid_until_ms=valid_until_ms,
            ):
                with self.assertRaises(NavigationIntentProposalError):
                    bind_navigation_intent_proposal(
                        proposal,
                        offer=offer(),
                        proposal_id="host-proposal-17",
                        received_at_ms=received_at_ms,
                        valid_until_ms=valid_until_ms,
                    )

    def test_rejects_expiry_identity_and_basis_mismatch(self):
        current_offer = offer()
        envelope = bind_navigation_intent_proposal(
            NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            offer=current_offer,
            proposal_id="host-proposal-17",
            received_at_ms=10_000,
            valid_until_ms=20_000,
        )
        cases = (
            {
                "proposal_id": "host-proposal-other",
                "ticket_id": "ticket-11",
                "basis": current_offer.basis,
                "now_ms": 11_000,
            },
            {
                "proposal_id": "host-proposal-17",
                "ticket_id": "ticket-other",
                "basis": current_offer.basis,
                "now_ms": 11_000,
            },
            {
                "proposal_id": "host-proposal-17",
                "ticket_id": "ticket-11",
                "basis": basis(
                    navigation_basis_id="navigation-evidence-changed",
                    controller_state_version=12,
                ),
                "now_ms": 11_000,
            },
            {
                "proposal_id": "host-proposal-17",
                "ticket_id": "ticket-11",
                "basis": current_offer.basis,
                "now_ms": 20_000,
            },
        )
        for values in cases:
            with self.subTest(values=values):
                with self.assertRaises(NavigationIntentProposalError):
                    envelope.assert_current(**values)

    def test_currentness_uses_canonical_decision_equivalence(self):
        current_offer = offer()
        envelope = bind_navigation_intent_proposal(
            NavigationIntentProposal(intent=FOLLOW_DIRECTION),
            offer=current_offer,
            proposal_id="host-proposal-17",
            received_at_ms=10_000,
            valid_until_ms=20_000,
        )

        envelope.assert_current(
            proposal_id="host-proposal-17",
            ticket_id="ticket-11",
            basis=basis(
                controller_state_version=12,
                world_model_version=9,
            ),
            now_ms=11_000,
        )

    def test_binding_revalidates_offer_instead_of_trusting_constructor(self):
        proposal = NavigationIntentProposal(
            intent=DETOUR_TARGET,
            target_id="hazard-1",
            side=LEFT,
        )

        with self.assertRaises(NavigationIntentProposalError) as caught:
            bind_navigation_intent_proposal(
                proposal,
                offer=offer(),
                proposal_id="host-proposal-17",
                received_at_ms=10_000,
                valid_until_ms=20_000,
            )

        self.assertEqual(caught.exception.code, "unoffered_target")


class NavigationIntentSizeTests(unittest.TestCase):
    def test_shadow_contract_is_clearly_smaller_than_current_contract(self):
        current_schema = _response_schema(
            episode_id="episode-7",
            turn=9,
            state_version=11,
            available_actions=sorted(ACTIONS),
            target_ids=("hazard-1", "hazard-2"),
            empty_maneuver_required=False,
        )
        current_output = {
            "schema": "robot-physical-navigation-decision/v1",
            "episode_id": "episode-7",
            "turn": 9,
            "based_on_state_version": 11,
            "action": ADVANCE,
            "plan": [ADVANCE, ADVANCE],
            "reason_code": "PROGRESS_GOAL",
            "assessment": "The current path remains clear.",
            "utterance": None,
            "perception_target_hypothesis_id": None,
            "maneuver_commitment": empty_commitment(),
        }
        current_offer = offer()
        proposal = NavigationIntentProposal(intent=FOLLOW_DIRECTION)

        first = size_metrics(
            current_schema=current_schema,
            current_output=current_output,
            current_offer=current_offer,
            proposal=proposal,
        )
        second = size_metrics(
            current_schema=current_schema,
            current_output=current_output,
            current_offer=current_offer,
            proposal=proposal,
        )

        self.assertEqual(first, second)
        self.assertGreater(first["schema_saved_bytes"], 0)
        self.assertGreaterEqual(first["schema_reduction_milli"], 600)
        self.assertGreater(first["output_saved_bytes"], 0)
        self.assertGreaterEqual(first["output_reduction_milli"], 900)
        self.assertLess(first["shadow_schema_bytes"], 2_000)
        self.assertLess(first["shadow_output_bytes"], 40)

if __name__ == "__main__":
    unittest.main()
