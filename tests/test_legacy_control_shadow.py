from dataclasses import FrozenInstanceError
import json
import unittest

from robot_agent.legacy_control_shadow import (
    FailOpenLegacyShadowObserver,
    FrozenLegacyShadowFacts,
    InMemoryLegacyShadowJournal,
    LEGACY_CONTROL_SHADOW_SCHEMA,
    LEGACY_CONTROL_SHADOW_VERSION,
    LegacyControlShadowError,
    LegacyShadowRecord,
    MAX_LEGACY_SHADOW_FACT_KEYS,
    SHADOW_DISABLED_STAGE,
)


def record(sequence=1, **facts):
    return LegacyShadowRecord.capture(
        episode_id="episode-a",
        sequence=sequence,
        stage="decision_committed",
        facts=facts,
    )


class LegacyShadowRecordTests(unittest.TestCase):
    def test_record_is_versioned_ordered_and_directly_json_serializable(self):
        value = record(
            flag=True,
            count=1,
            label="1",
            nested={"z": None, "a": [False, 2, "2"]},
        )

        payload = value.to_dict()

        self.assertEqual(payload["schema"], LEGACY_CONTROL_SHADOW_SCHEMA)
        self.assertEqual(payload["version"], LEGACY_CONTROL_SHADOW_VERSION)
        self.assertEqual(payload["episode_id"], "episode-a")
        self.assertEqual(payload["sequence"], 1)
        self.assertEqual(payload["stage"], "decision_committed")
        self.assertIs(payload["facts"]["flag"], True)
        self.assertIs(type(payload["facts"]["count"]), int)
        self.assertIs(type(payload["facts"]["label"]), str)
        self.assertIs(payload["facts"]["nested"]["a"][0], False)
        self.assertEqual(json.loads(value.to_json()), payload)

    def test_fact_order_does_not_change_canonical_replay_record(self):
        first = LegacyShadowRecord.capture(
            episode_id="episode-a",
            sequence=7,
            stage="projection",
            facts={"z": {"b": 2, "a": 1}, "a": True},
        )
        second = LegacyShadowRecord.capture(
            episode_id="episode-a",
            sequence=7,
            stage="projection",
            facts={"a": True, "z": {"a": 1, "b": 2}},
        )

        self.assertEqual(first, second)
        self.assertEqual(first.to_json(), second.to_json())

    def test_facts_are_immutable_and_views_are_defensive_copies(self):
        value = record(nested={"values": [1, 2]})
        first = value.to_dict()
        first["facts"]["nested"]["values"].append(3)

        self.assertEqual(
            value.to_dict()["facts"],
            {"nested": {"values": [1, 2]}},
        )
        with self.assertRaises(FrozenInstanceError):
            value.facts.canonical_json = "{}"

    def test_noncanonical_and_unbounded_facts_are_rejected(self):
        cases = (
            (
                lambda: FrozenLegacyShadowFacts('{"b":1, "a":2}'),
                "noncanonical_facts",
            ),
            (
                lambda: record(value=1.5),
                "unsupported_fact_value",
            ),
            (
                lambda: record(value=2**63),
                "invalid_integer",
            ),
            (
                lambda: LegacyShadowRecord.capture(
                    episode_id="episode-a",
                    sequence=1,
                    stage="projection",
                    facts={
                        "key-{}".format(index): index
                        for index in range(MAX_LEGACY_SHADOW_FACT_KEYS + 1)
                    },
                ),
                "too_many_facts",
            ),
        )
        for operation, code in cases:
            with self.subTest(code=code):
                with self.assertRaises(LegacyControlShadowError) as caught:
                    operation()
                self.assertEqual(caught.exception.code, code)

    def test_boolean_cannot_impersonate_integer_schema_version(self):
        with self.assertRaises(LegacyControlShadowError) as caught:
            LegacyShadowRecord(
                schema=LEGACY_CONTROL_SHADOW_SCHEMA,
                version=True,
                episode_id="episode-a",
                sequence=1,
                stage="projection",
                facts=FrozenLegacyShadowFacts.capture({}),
            )

        self.assertEqual(caught.exception.code, "invalid_version")


class InMemoryLegacyShadowJournalTests(unittest.TestCase):
    def test_journal_is_bounded_and_preserves_record_sequence(self):
        journal = InMemoryLegacyShadowJournal(capacity=2)

        self.assertTrue(journal.try_write(record(1)))
        self.assertTrue(journal.try_write(record(2)))
        self.assertTrue(journal.try_write(record(3)))

        self.assertEqual(
            tuple(item.sequence for item in journal.snapshot()),
            (2, 3),
        )
        self.assertEqual(journal.status.capacity, 2)
        self.assertEqual(journal.status.retained_records, 2)
        self.assertEqual(journal.status.written_records, 3)
        self.assertEqual(journal.status.evicted_records, 1)

    def test_busy_journal_drops_immediately_without_mutating_snapshot(self):
        journal = InMemoryLegacyShadowJournal(capacity=2)
        self.assertTrue(journal._lock.acquire(False))
        try:
            self.assertFalse(journal.try_write(record(1)))
        finally:
            journal._lock.release()

        self.assertEqual(journal.snapshot(), ())
        self.assertEqual(journal.status.written_records, 0)


class FailOpenLegacyShadowObserverTests(unittest.TestCase):
    def test_observe_writes_deterministic_sequences_and_returns_none(self):
        journal = InMemoryLegacyShadowJournal()
        observer = FailOpenLegacyShadowObserver(
            episode_id="episode-a",
            try_write=journal.try_write,
        )

        self.assertIsNone(observer.observe("decision", choice="ADVANCE"))
        self.assertIsNone(observer.observe("projection", valid=True))

        records = journal.snapshot()
        self.assertEqual(
            tuple((item.sequence, item.stage) for item in records),
            ((1, "decision"), (2, "projection")),
        )
        self.assertTrue(observer.status.enabled)
        self.assertEqual(observer.status.next_sequence, 3)
        self.assertEqual(observer.status.written_records, 2)
        self.assertEqual(observer.status.dropped_records, 0)
        self.assertIsNone(observer.status.failure)

    def test_false_writer_result_disables_incomplete_episode_trace(self):
        observer = FailOpenLegacyShadowObserver(
            episode_id="episode-a",
            try_write=lambda _record: False,
        )

        self.assertIsNone(observer.observe("decision", choice="HOLD"))

        self.assertFalse(observer.enabled)
        self.assertEqual(observer.status.next_sequence, 3)
        self.assertEqual(observer.status.written_records, 0)
        self.assertEqual(observer.status.dropped_records, 1)
        self.assertEqual(
            observer.status.failure.failure_type,
            "LegacyControlShadowError",
        )

    def test_first_exception_disables_episode_and_emits_one_summary(self):
        calls = []

        def flaky_writer(value):
            calls.append(value)
            if len(calls) == 1:
                raise RuntimeError("unbounded detail must not enter the journal")
            return True

        observer = FailOpenLegacyShadowObserver(
            episode_id="episode-a",
            try_write=flaky_writer,
        )

        self.assertIsNone(observer.observe("projection", candidate="left"))
        self.assertIsNone(observer.observe("later", ignored=True))

        self.assertEqual(len(calls), 2)
        disabled = calls[1]
        self.assertEqual(disabled.sequence, 2)
        self.assertEqual(disabled.stage, SHADOW_DISABLED_STAGE)
        self.assertEqual(
            disabled.to_dict()["facts"],
            {
                "failed_sequence": 1,
                "failed_stage": "projection",
                "failure_type": "RuntimeError",
            },
        )
        self.assertFalse(observer.enabled)
        self.assertEqual(observer.status.next_sequence, 3)
        self.assertEqual(observer.status.written_records, 1)
        self.assertTrue(observer.status.disabled_record_emitted)
        self.assertEqual(observer.status.failure.failure_type, "RuntimeError")

    def test_disabled_summary_is_attempted_at_most_once_when_writer_stays_broken(self):
        calls = []

        def broken_writer(value):
            calls.append(value)
            raise OSError("offline")

        observer = FailOpenLegacyShadowObserver(
            episode_id="episode-a",
            try_write=broken_writer,
        )

        self.assertIsNone(observer.observe("decision", value="ADVANCE"))
        self.assertIsNone(observer.observe("decision", value="ADVANCE"))

        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[1].stage, SHADOW_DISABLED_STAGE)
        self.assertFalse(observer.status.disabled_record_emitted)
        self.assertEqual(observer.status.failure.failure_type, "OSError")

    def test_invalid_fact_disables_without_leaking_invalid_payload(self):
        records = []
        observer = FailOpenLegacyShadowObserver(
            episode_id="episode-a",
            try_write=lambda value: records.append(value) or True,
        )

        self.assertIsNone(observer.observe("projection", invalid={1, 2}))

        self.assertEqual(len(records), 1)
        self.assertEqual(records[0].stage, SHADOW_DISABLED_STAGE)
        self.assertEqual(records[0].sequence, 2)
        self.assertEqual(
            records[0].to_dict()["facts"]["failure_type"],
            "LegacyControlShadowError",
        )
        self.assertFalse(observer.enabled)

    def test_base_exception_is_not_swallowed_or_used_to_disable_shadow(self):
        def interrupting_writer(_value):
            raise KeyboardInterrupt()

        observer = FailOpenLegacyShadowObserver(
            episode_id="episode-a",
            try_write=interrupting_writer,
        )

        with self.assertRaises(KeyboardInterrupt):
            observer.observe("projection", valid=True)

        self.assertTrue(observer.enabled)
        self.assertEqual(observer.status.next_sequence, 2)
        self.assertIsNone(observer.status.failure)

    def test_reentrant_observation_disables_instead_of_hiding_gap(self):
        calls = []
        observer = None

        def reentrant_writer(value):
            calls.append(value)
            observer.observe("nested", ignored=True)
            return True

        observer = FailOpenLegacyShadowObserver(
            episode_id="episode-a",
            try_write=reentrant_writer,
        )

        self.assertIsNone(observer.observe("outer", accepted=True))

        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0].stage, "outer")
        self.assertEqual(calls[1].stage, SHADOW_DISABLED_STAGE)
        self.assertFalse(observer.enabled)
        self.assertEqual(observer.status.next_sequence, 4)
        self.assertEqual(
            observer.status.failure.failed_stage,
            "observer_contention",
        )

    def test_new_episode_observer_is_independent_after_previous_failure(self):
        first = FailOpenLegacyShadowObserver(
            episode_id="episode-a",
            try_write=lambda _record: (_ for _ in ()).throw(RuntimeError()),
        )
        journal = InMemoryLegacyShadowJournal()
        second = FailOpenLegacyShadowObserver(
            episode_id="episode-b",
            try_write=journal.try_write,
        )

        first.observe("projection", valid=True)
        second.observe("projection", valid=True)

        self.assertFalse(first.enabled)
        self.assertTrue(second.enabled)
        self.assertEqual(journal.snapshot()[0].episode_id, "episode-b")


if __name__ == "__main__":
    unittest.main()
