from dataclasses import replace
import json
import unittest

from robot_agent.interaction_contract import (
    EXPRESSION_PROPOSAL_SCHEMA,
    INTERACTION_SNAPSHOT_SCHEMA,
    MAX_INTERACTION_JSON_BYTES,
    OBJECT_EVIDENCE_SCHEMA,
    ExpressionIntent,
    ExpressionProposal,
    InteractionContractError,
    InteractionSnapshot,
    ObjectEvidence,
    decode_expression_proposal,
    decode_interaction_snapshot,
    decode_object_evidence,
)


def evidence():
    return ObjectEvidence(
        evidence_id="evidence-7",
        relation="BLOCKING_PATH",
        object_id="box-3",
        source="simulated-range-fusion",
        observed_at_ms=9_990,
        confidence_milli=940,
    )


def snapshot():
    return InteractionSnapshot(
        robot_id="robot-sim",
        controller_instance_id="controller-1",
        goal_id="waypoint-2",
        goal_epoch=3,
        plan_revision=4,
        interaction_state_version=11,
        world_model_version=8,
        captured_at_ms=10_000,
        obstruction_epoch=2,
        drive_phase="BLOCKED",
        response_locale="en-GB",
        evidence=evidence(),
    )


def proposal_value(decision="EXPRESS"):
    value = {
        "schema": EXPRESSION_PROPOSAL_SCHEMA,
        "proposal_id": "expression-1",
        "robot_id": "robot-sim",
        "controller_instance_id": "controller-1",
        "goal_id": "waypoint-2",
        "goal_epoch": 3,
        "plan_revision": 4,
        "based_on_interaction_state_version": 11,
        "based_on_world_model_version": 8,
        "obstruction_epoch": 2,
        "based_on_evidence_id": "evidence-7",
        "decision": decision,
        "confidence_milli": 880,
    }
    if decision == "EXPRESS":
        value["intent"] = {
            "utterance": "What on earth is blocking my way now?",
            "utterance_locale": "en-GB",
            "gesture_kind": "PROPELLER_WAVE",
            "affect_label": "grumpy",
            "intensity": 800,
            "repetitions": 2,
        }
    else:
        value["reason_code"] = "expression_not_needed"
    return value


def proposal_raw(decision="EXPRESS"):
    return json.dumps(proposal_value(decision)).encode("utf-8")


class ObjectEvidenceContractTests(unittest.TestCase):
    def test_round_trips_strict_evidence_with_optional_object_id(self):
        item = decode_object_evidence(
            json.dumps(evidence().to_dict()).encode("utf-8")
        )
        unidentified = replace(item, object_id=None)

        self.assertEqual(item, evidence())
        self.assertIsNone(unidentified.to_dict()["object_id"])
        self.assertEqual(
            item.to_dict()["schema"],
            OBJECT_EVIDENCE_SCHEMA,
        )

    def test_rejects_unknown_relation_and_invalid_confidence(self):
        for changes in (
            {"relation": "NEARBY"},
            {"confidence_milli": -1},
            {"confidence_milli": 1_001},
            {"confidence_milli": True},
        ):
            value = evidence().to_dict()
            value.update(changes)
            with self.subTest(changes=changes):
                with self.assertRaises(InteractionContractError):
                    decode_object_evidence(
                        json.dumps(value).encode("utf-8")
                    )

    def test_rejects_extra_and_duplicate_evidence_fields(self):
        value = evidence().to_dict()
        value["priority"] = 99
        with self.assertRaises(InteractionContractError) as extra:
            decode_object_evidence(json.dumps(value).encode("utf-8"))
        self.assertEqual(
            extra.exception.code,
            "invalid_object_evidence_fields",
        )

        duplicate = json.dumps(evidence().to_dict()).replace(
            '"evidence_id": "evidence-7"',
            '"evidence_id": "evidence-7", '
            '"evidence_id": "replacement"',
        )
        with self.assertRaises(InteractionContractError) as caught:
            decode_object_evidence(duplicate.encode("utf-8"))
        self.assertEqual(
            caught.exception.code,
            "invalid_object_evidence_json",
        )


class InteractionSnapshotContractTests(unittest.TestCase):
    def test_round_trips_all_stable_bindings(self):
        original = snapshot()
        decoded = decode_interaction_snapshot(
            json.dumps(original.to_dict()).encode("utf-8")
        )

        self.assertEqual(decoded, original)
        self.assertEqual(
            decoded.to_dict()["schema"],
            INTERACTION_SNAPSHOT_SCHEMA,
        )
        self.assertEqual(decoded.interaction_state_version, 11)
        self.assertEqual(decoded.obstruction_epoch, 2)
        self.assertEqual(decoded.response_locale, "en-GB")

    def test_allows_no_current_evidence(self):
        original = replace(
            snapshot(),
            obstruction_epoch=0,
            drive_phase="MOVING",
            evidence=None,
        )

        decoded = decode_interaction_snapshot(
            json.dumps(original.to_dict()).encode("utf-8")
        )

        self.assertIsNone(decoded.evidence)
        self.assertEqual(decoded.drive_phase, "MOVING")

    def test_rejects_bad_phase_future_evidence_and_unversioned_evidence(self):
        invalid_changes = (
            {"drive_phase": "REVERSING"},
            {"captured_at_ms": 9_000},
            {"obstruction_epoch": 0},
            {"response_locale": ""},
            {"response_locale": " en"},
        )
        for changes in invalid_changes:
            with self.subTest(changes=changes):
                with self.assertRaises(InteractionContractError):
                    replace(snapshot(), **changes)

    def test_rejects_extra_snapshot_fields_and_nested_duplicate_keys(self):
        value = snapshot().to_dict()
        value["ttl_ms"] = 500
        with self.assertRaises(InteractionContractError) as extra:
            decode_interaction_snapshot(
                json.dumps(value).encode("utf-8")
            )
        self.assertEqual(
            extra.exception.code,
            "invalid_interaction_snapshot_fields",
        )

        missing_locale = snapshot().to_dict()
        missing_locale.pop("response_locale")
        with self.assertRaises(InteractionContractError) as missing:
            decode_interaction_snapshot(
                json.dumps(missing_locale).encode("utf-8")
            )
        self.assertEqual(
            missing.exception.code,
            "invalid_interaction_snapshot_fields",
        )

        raw = json.dumps(snapshot().to_dict()).replace(
            '"source": "simulated-range-fusion"',
            '"source": "simulated-range-fusion", "source": "spoofed"',
        )
        with self.assertRaises(InteractionContractError) as duplicate:
            decode_interaction_snapshot(raw.encode("utf-8"))
        self.assertEqual(
            duplicate.exception.code,
            "invalid_interaction_snapshot_json",
        )


class ExpressionIntentContractTests(unittest.TestCase):
    def test_accepts_generic_locale_identifiers_without_language_rules(self):
        for locale in ("sv", "en-US", "pl-Latn-PL", "x-demo"):
            with self.subTest(locale=locale):
                intent = ExpressionIntent(
                    utterance="x" * 160,
                    utterance_locale=locale,
                    gesture_kind="PROPELLER_WAVE",
                    affect_label="animated",
                    intensity=0,
                    repetitions=1,
                )
                self.assertEqual(intent.utterance_locale, locale)

    def test_accepts_speech_only_without_gesture_repetitions(self):
        intent = ExpressionIntent(
            utterance="I can complain without waving.",
            utterance_locale="en",
            gesture_kind=None,
            affect_label="grumpy",
            intensity=500,
            repetitions=0,
        )

        self.assertIsNone(intent.gesture_kind)
        self.assertEqual(intent.repetitions, 0)
        self.assertIsNone(intent.to_dict()["gesture_kind"])

    def test_rejects_long_or_non_printable_utterances(self):
        for utterance in ("x" * 161, " leading", "line\nbreak", ""):
            with self.subTest(utterance=utterance):
                with self.assertRaises(InteractionContractError) as caught:
                    ExpressionIntent(
                        utterance=utterance,
                        utterance_locale="en",
                        gesture_kind="PROPELLER_WAVE",
                        affect_label="grumpy",
                        intensity=500,
                        repetitions=1,
                    )
                self.assertEqual(
                    caught.exception.code,
                    "invalid_utterance",
                )

    def test_rejects_unknown_gesture_and_out_of_range_semantics(self):
        changes = (
            {"gesture_kind": "ARM_FLAIL"},
            {"intensity": -1},
            {"intensity": 1_001},
            {"intensity": True},
            {"repetitions": 0},
            {"repetitions": 3},
        )
        base = {
            "utterance": "Move, box.",
            "utterance_locale": "en",
            "gesture_kind": "PROPELLER_WAVE",
            "affect_label": "grumpy",
            "intensity": 500,
            "repetitions": 1,
        }
        for change in changes:
            with self.subTest(change=change):
                values = dict(base)
                values.update(change)
                with self.assertRaises(InteractionContractError):
                    ExpressionIntent(**values)

        invalid_combinations = (
            {"gesture_kind": None, "repetitions": 1},
            {"gesture_kind": None, "repetitions": -1},
            {"gesture_kind": None, "repetitions": False},
        )
        for change in invalid_combinations:
            with self.subTest(change=change):
                values = dict(base)
                values.update(change)
                with self.assertRaises(InteractionContractError):
                    ExpressionIntent(**values)

    def test_affect_label_is_data_not_execution_authority(self):
        intent = ExpressionIntent(
            utterance="I object to this object.",
            utterance_locale="en",
            gesture_kind="PROPELLER_WAVE",
            affect_label="furious",
            intensity=1_000,
            repetitions=2,
        )

        view = intent.to_dict()
        self.assertEqual(view["affect_label"], "furious")
        self.assertFalse(
            {
                "speed",
                "duration",
                "priority",
                "authority",
                "source",
            }
            & set(view)
        )


class ExpressionProposalContractTests(unittest.TestCase):
    def test_decodes_express_and_matches_exact_snapshot(self):
        proposal = decode_expression_proposal(proposal_raw())

        self.assertEqual(proposal.decision, "EXPRESS")
        self.assertEqual(proposal.intent.repetitions, 2)
        self.assertIsNone(proposal.assert_matches_snapshot(snapshot()))
        self.assertEqual(proposal.to_dict(), proposal_value())

    def test_decodes_hold_and_abort_without_intent(self):
        for decision in ("HOLD", "ABORT"):
            with self.subTest(decision=decision):
                proposal = decode_expression_proposal(
                    proposal_raw(decision)
                )
                self.assertEqual(proposal.decision, decision)
                self.assertIsNone(proposal.intent)
                self.assertEqual(
                    proposal.reason_code,
                    "expression_not_needed",
                )
                self.assertIsNone(
                    proposal.assert_matches_snapshot(snapshot())
                )

    def test_express_requires_evidence_and_intent(self):
        no_evidence = proposal_value()
        no_evidence["based_on_evidence_id"] = None
        no_intent = proposal_value()
        no_intent.pop("intent")
        for value in (no_evidence, no_intent):
            with self.subTest(value=value):
                with self.assertRaises(InteractionContractError):
                    decode_expression_proposal(
                        json.dumps(value).encode("utf-8")
                    )

    def test_non_express_requires_reason_and_rejects_intent(self):
        no_reason = proposal_value("HOLD")
        no_reason.pop("reason_code")
        with_intent = proposal_value("ABORT")
        with_intent["intent"] = proposal_value()["intent"]
        for value in (no_reason, with_intent):
            with self.subTest(value=value):
                with self.assertRaises(InteractionContractError):
                    decode_expression_proposal(
                        json.dumps(value).encode("utf-8")
                    )

    def test_rejects_every_stale_or_cross_context_binding(self):
        current = snapshot()
        mismatches = (
            replace(current, robot_id="other-robot"),
            replace(current, controller_instance_id="controller-2"),
            replace(current, goal_id="waypoint-9"),
            replace(current, goal_epoch=4),
            replace(current, plan_revision=5),
            replace(current, interaction_state_version=12),
            replace(current, world_model_version=9),
            replace(current, obstruction_epoch=3),
            replace(current, response_locale="sv"),
            replace(
                current,
                evidence=replace(
                    current.evidence,
                    evidence_id="evidence-8",
                ),
            ),
            replace(current, evidence=None),
        )
        proposal = decode_expression_proposal(proposal_raw())
        for changed in mismatches:
            with self.subTest(changed=changed):
                with self.assertRaises(InteractionContractError) as caught:
                    proposal.assert_matches_snapshot(changed)
                self.assertEqual(
                    caught.exception.code,
                    "stale_expression_proposal",
                )

    def test_rejects_unknown_gesture_from_json(self):
        value = proposal_value()
        value["intent"]["gesture_kind"] = "DRIVE_AT_OBJECT"

        with self.assertRaises(InteractionContractError) as caught:
            decode_expression_proposal(
                json.dumps(value).encode("utf-8")
            )

        self.assertEqual(caught.exception.code, "invalid_gesture_kind")

    def test_rejects_model_control_fields_at_any_level(self):
        forbidden = (
            "motor_role",
            "motor_port",
            "port",
            "speed",
            "speed_dps",
            "duration",
            "duration_ms",
            "ttl_ms",
            "valid_until_ms",
            "priority",
            "authority",
            "authority_rank",
            "source",
            "source_id",
        )
        for field in forbidden:
            for nested in (False, True):
                with self.subTest(field=field, nested=nested):
                    value = proposal_value()
                    target = value["intent"] if nested else value
                    target[field] = 1
                    with self.assertRaises(
                        InteractionContractError
                    ) as caught:
                        decode_expression_proposal(
                            json.dumps(value).encode("utf-8")
                        )
                    self.assertIn(
                        caught.exception.code,
                        (
                            "invalid_expression_proposal_fields",
                            "invalid_expression_intent_fields",
                        ),
                    )

    def test_rejects_duplicate_keys_unknown_decision_and_oversize(self):
        duplicate = proposal_raw().replace(
            b'"proposal_id": "expression-1",',
            b'"proposal_id": "expression-1", '
            b'"proposal_id": "expression-2",',
        )
        with self.assertRaises(InteractionContractError) as caught:
            decode_expression_proposal(duplicate)
        self.assertEqual(
            caught.exception.code,
            "invalid_expression_proposal_json",
        )

        unknown = proposal_value()
        unknown["decision"] = "EXECUTE"
        with self.assertRaises(InteractionContractError) as decision:
            decode_expression_proposal(
                json.dumps(unknown).encode("utf-8")
            )
        self.assertEqual(
            decision.exception.code,
            "invalid_expression_decision",
        )

        with self.assertRaises(InteractionContractError) as oversized:
            decode_expression_proposal(
                b"{" + (b" " * MAX_INTERACTION_JSON_BYTES) + b"}"
            )
        self.assertEqual(
            oversized.exception.code,
            "invalid_expression_proposal_body",
        )


if __name__ == "__main__":
    unittest.main()
