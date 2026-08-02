from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
import time
import unittest
from unittest.mock import patch

from robot_agent.legacy_control_shadow import (
    FailOpenLegacyShadowObserver,
    LegacyShadowRecord,
)
from robot_agent.persistent_legacy_shadow_journal import (
    PersistentLegacyShadowJournal,
    PersistentLegacyShadowJournalError,
    PersistentLegacyShadowSession,
)


def record(sequence, stage="projection", **facts):
    return LegacyShadowRecord.capture(
        episode_id="episode-a",
        sequence=sequence,
        stage=stage,
        facts=facts,
    )


def wait_until(predicate, timeout_seconds=1.0):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


class RecordingStream:
    def __init__(self):
        self.payloads = []
        self.flushes = 0
        self.closed = False

    def write(self, payload):
        self.payloads.append(payload)
        return len(payload)

    def flush(self):
        self.flushes += 1

    def close(self):
        self.closed = True


class BlockingStream(RecordingStream):
    def __init__(self):
        super().__init__()
        self.entered = Event()
        self.release = Event()

    def write(self, payload):
        self.entered.set()
        self.release.wait(2.0)
        return super().write(payload)


class FailingStream(RecordingStream):
    def __init__(self):
        super().__init__()
        self.failed = Event()

    def write(self, _payload):
        self.failed.set()
        raise OSError("disk failure detail is intentionally not retained")


class PersistentLegacyShadowJournalTests(unittest.TestCase):
    def test_close_flushes_deterministic_one_record_per_line_content(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "shadow.ndjson"
            values = (
                record(1, choice="ADVANCE", count=1, active=True),
                record(2, stage="receipt", text="rad ett\nrad två"),
            )
            journal = PersistentLegacyShadowJournal(path, capacity=4)

            self.assertTrue(journal.try_write(values[0]))
            self.assertTrue(journal.try_write(values[1]))
            self.assertTrue(journal.close(timeout_seconds=2.0))

            self.assertEqual(
                path.read_text(encoding="utf-8").splitlines(),
                [value.to_json() for value in values],
            )
            self.assertEqual(journal.status.queued_records, 0)
            self.assertEqual(journal.status.written_records, 2)
            self.assertIsNone(journal.status.error)
            self.assertTrue(journal.status.closed)
            self.assertFalse(journal.status.worker_alive)

    def test_existing_file_is_appended_without_overwrite(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "shadow.ndjson"
            path.write_text("existing-record\n", encoding="utf-8")
            value = record(1, result="projected")
            journal = PersistentLegacyShadowJournal(path)

            self.assertTrue(journal.try_write(value))
            self.assertTrue(journal.close())

            self.assertEqual(
                path.read_text(encoding="utf-8"),
                "existing-record\n{}\n".format(value.to_json()),
            )

    def test_bounded_queue_reports_saturation_without_waiting(self):
        with TemporaryDirectory() as directory:
            stream = BlockingStream()
            with patch(
                "robot_agent.persistent_legacy_shadow_journal."
                "_open_append_stream",
                return_value=stream,
            ):
                journal = PersistentLegacyShadowJournal(
                    Path(directory) / "shadow.ndjson",
                    capacity=1,
                )
                self.assertTrue(journal.try_write(record(1)))
                self.assertTrue(stream.entered.wait(1.0))

                self.assertTrue(journal.try_write(record(2)))
                started = time.monotonic()
                with self.assertRaises(
                    PersistentLegacyShadowJournalError
                ) as caught:
                    journal.try_write(record(3))
                self.assertLess(time.monotonic() - started, 0.1)
                self.assertEqual(caught.exception.code, "queue_full")
                self.assertEqual(journal.status.queued_records, 1)

                stream.release.set()
                self.assertTrue(journal.close(timeout_seconds=2.0))

            self.assertEqual(
                stream.payloads,
                [record(1).to_json() + "\n", record(2).to_json() + "\n"],
            )
            self.assertEqual(journal.status.written_records, 2)

    def test_admission_lock_contention_is_an_explicit_failure(self):
        with TemporaryDirectory() as directory:
            journal = PersistentLegacyShadowJournal(
                Path(directory) / "shadow.ndjson"
            )
            self.assertTrue(journal._state_lock.acquire(False))
            try:
                with self.assertRaises(
                    PersistentLegacyShadowJournalError
                ) as caught:
                    journal.try_write(record(1))
            finally:
                journal._state_lock.release()

            self.assertEqual(caught.exception.code, "admission_busy")
            self.assertTrue(journal.close())

    def test_queue_saturation_disables_episode_session_without_blocking(self):
        with TemporaryDirectory() as directory:
            stream = BlockingStream()
            with patch(
                "robot_agent.persistent_legacy_shadow_journal."
                "_open_append_stream",
                return_value=stream,
            ):
                session = PersistentLegacyShadowSession(
                    episode_id="episode-a",
                    path=Path(directory) / "shadow.ndjson",
                    capacity=1,
                )
                self.assertIsNone(session.observe("first", value=1))
                self.assertTrue(stream.entered.wait(1.0))
                self.assertIsNone(session.observe("second", value=2))

                started = time.monotonic()
                self.assertIsNone(session.observe("third", value=3))
                self.assertLess(time.monotonic() - started, 0.1)
                self.assertFalse(session.status.observer.enabled)
                self.assertEqual(
                    session.status.observer.failure.failure_type,
                    "PersistentLegacyShadowJournalError",
                )
                self.assertFalse(
                    session.status.observer.disabled_record_emitted
                )
                self.assertEqual(session.status.journal.queued_records, 1)

                stream.release.set()
                self.assertTrue(session.close(timeout_seconds=2.0))
                self.assertTrue(session.close(timeout_seconds=0.0))

            self.assertEqual(len(stream.payloads), 2)
            self.assertEqual(session.status.journal.written_records, 2)

    def test_session_composes_observer_journal_status_and_close(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "shadow.ndjson"
            session = PersistentLegacyShadowSession(
                episode_id="episode-a",
                path=path,
                capacity=4,
            )

            self.assertIsNone(session.observe("projection", parity="MATCH"))
            self.assertTrue(session.close())

            status = session.status
            self.assertEqual(status.observer.episode_id, "episode-a")
            self.assertTrue(status.observer.enabled)
            self.assertEqual(status.observer.written_records, 1)
            self.assertEqual(status.journal.written_records, 1)
            self.assertTrue(status.journal.closed)
            self.assertEqual(session.path, path.absolute())

    def test_async_write_error_is_visible_and_rejected_on_next_write(self):
        with TemporaryDirectory() as directory:
            stream = FailingStream()
            with patch(
                "robot_agent.persistent_legacy_shadow_journal."
                "_open_append_stream",
                return_value=stream,
            ):
                journal = PersistentLegacyShadowJournal(
                    Path(directory) / "shadow.ndjson"
                )
                self.assertTrue(journal.try_write(record(1)))
                self.assertTrue(stream.failed.wait(1.0))
                self.assertTrue(
                    wait_until(lambda: journal.status.error is not None)
                )

                with self.assertRaises(
                    PersistentLegacyShadowJournalError
                ) as caught:
                    journal.try_write(record(2))

                self.assertEqual(caught.exception.code, "writer_failed")
                self.assertEqual(journal.status.error.code, "write_failed")
                self.assertEqual(journal.status.error.error_type, "OSError")
                self.assertEqual(journal.status.written_records, 0)
                self.assertEqual(journal.status.queued_records, 0)
                self.assertFalse(journal.close())

    def test_async_error_disables_the_fail_open_observer_on_its_next_record(self):
        with TemporaryDirectory() as directory:
            stream = FailingStream()
            with patch(
                "robot_agent.persistent_legacy_shadow_journal."
                "_open_append_stream",
                return_value=stream,
            ):
                journal = PersistentLegacyShadowJournal(
                    Path(directory) / "shadow.ndjson"
                )
                observer = FailOpenLegacyShadowObserver(
                    episode_id="episode-a",
                    try_write=journal.try_write,
                )

                self.assertIsNone(observer.observe("first", valid=True))
                self.assertTrue(stream.failed.wait(1.0))
                self.assertTrue(
                    wait_until(lambda: journal.status.error is not None)
                )
                self.assertIsNone(observer.observe("second", valid=True))

                self.assertFalse(observer.enabled)
                self.assertEqual(
                    observer.status.failure.failure_type,
                    "PersistentLegacyShadowJournalError",
                )
                self.assertFalse(observer.status.disabled_record_emitted)
                self.assertFalse(journal.close())

    def test_open_error_becomes_an_async_writer_failure(self):
        with TemporaryDirectory() as directory:
            def fail_open(_path):
                raise PermissionError("not writable")

            with patch(
                "robot_agent.persistent_legacy_shadow_journal."
                "_open_append_stream",
                side_effect=fail_open,
            ):
                journal = PersistentLegacyShadowJournal(
                    Path(directory) / "shadow.ndjson"
                )
                self.assertTrue(
                    wait_until(lambda: journal.status.error is not None)
                )

                with self.assertRaises(
                    PersistentLegacyShadowJournalError
                ) as caught:
                    journal.try_write(record(1))

                self.assertEqual(caught.exception.code, "writer_failed")
                self.assertEqual(journal.status.error.code, "open_failed")
                self.assertEqual(
                    journal.status.error.error_type,
                    "PermissionError",
                )
                self.assertFalse(journal.close())

    def test_close_is_bounded_idempotent_and_stops_new_admission(self):
        with TemporaryDirectory() as directory:
            stream = BlockingStream()
            with patch(
                "robot_agent.persistent_legacy_shadow_journal."
                "_open_append_stream",
                return_value=stream,
            ):
                journal = PersistentLegacyShadowJournal(
                    Path(directory) / "shadow.ndjson"
                )
                self.assertTrue(journal.try_write(record(1)))
                self.assertTrue(stream.entered.wait(1.0))

                started = time.monotonic()
                self.assertFalse(journal.close(timeout_seconds=0.01))
                self.assertLess(time.monotonic() - started, 0.2)
                self.assertTrue(journal.status.closed)
                self.assertTrue(journal.status.worker_alive)
                self.assertFalse(journal.try_write(record(2)))

                stream.release.set()
                self.assertTrue(journal.close(timeout_seconds=2.0))
                self.assertTrue(journal.close(timeout_seconds=0.0))

            self.assertTrue(stream.closed)
            self.assertEqual(journal.status.written_records, 1)

    def test_parent_and_target_must_already_be_valid(self):
        with TemporaryDirectory() as directory:
            parent = Path(directory)
            target = parent / "target.ndjson"
            target.write_text("", encoding="utf-8")
            symlink = parent / "shadow-link.ndjson"
            symlink.symlink_to(target)
            cases = (
                (parent / "missing" / "shadow.ndjson", "invalid_parent"),
                (parent, "invalid_path"),
                (symlink, "invalid_path"),
                ("embedded\0nul", "invalid_path"),
            )
            for path, code in cases:
                with self.subTest(code=code):
                    with self.assertRaises(
                        PersistentLegacyShadowJournalError
                    ) as caught:
                        PersistentLegacyShadowJournal(path)
                    self.assertEqual(caught.exception.code, code)


if __name__ == "__main__":
    unittest.main()
