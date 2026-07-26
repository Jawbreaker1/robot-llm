import json
import os
import subprocess
import sys
import threading
import unittest

from robot_agent.dashboard_contract import (
    DashboardContractError,
    DashboardSettings,
    NodeDescriptor,
    RobotDescriptor,
)
from robot_agent.dashboard_state import (
    ConversationStore,
    DashboardStateError,
    EventLog,
    NodeRegistry,
    SettingsStore,
)


class MutableClock:
    def __init__(self, value=1_000):
        self.value = value
        self.lock = threading.Lock()

    def __call__(self):
        with self.lock:
            return self.value

    def advance(self, amount=1):
        with self.lock:
            self.value += amount


class SequentialIDs:
    def __init__(self):
        self.value = 0
        self.lock = threading.Lock()

    def __call__(self):
        with self.lock:
            self.value += 1
            return str(self.value)


class SettingsStoreTests(unittest.TestCase):
    def test_update_is_atomic_and_revision_guarded(self):
        store = SettingsStore()

        updated = store.update(
            1,
            {
                "chat_mode": "research_required",
                "research": {"max_tool_calls": 0},
            },
        )

        self.assertEqual(updated.revision, 2)
        self.assertTrue(updated.require_evidence)
        self.assertEqual(updated.max_tool_calls, 0)
        with self.assertRaisesRegex(
            DashboardStateError,
            "revision",
        ):
            store.update(1, {"log_level": "info"})
        self.assertEqual(store.snapshot(), updated)

    def test_invalid_update_does_not_mutate_snapshot(self):
        store = SettingsStore()

        with self.assertRaises(DashboardContractError):
            store.update(
                1,
                {"research": {"max_elapsed_ms": 0}},
            )

        self.assertEqual(store.snapshot(), DashboardSettings.defaults())


class EventLogTests(unittest.TestCase):
    def make_log(self, capacity=3):
        return EventLog(
            "server-1",
            capacity=capacity,
            unix_clock_ms=MutableClock(10_000),
            monotonic_clock_ms=MutableClock(5_000),
            id_factory=SequentialIDs(),
        )

    def append(self, log, number, payload=None):
        return log.append(
            level="info",
            category="test",
            event_type="test.event",
            source_id="test-suite",
            message="Event {}".format(number),
            data={"number": number} if payload is None else payload,
        )

    def test_ring_reports_gap_and_supports_pagination(self):
        log = self.make_log(capacity=3)
        for number in range(1, 6):
            self.append(log, number)

        first = log.page(after_sequence=0, limit=2)
        self.assertTrue(first.gap)
        self.assertEqual(first.oldest_sequence, 3)
        self.assertEqual(first.newest_sequence, 5)
        self.assertEqual(first.dropped_total, 2)
        self.assertEqual(
            [item.sequence for item in first.events],
            [3, 4],
        )
        self.assertEqual(first.next_after_sequence, 4)

        second = log.page(after_sequence=4, limit=2)
        self.assertFalse(second.gap)
        self.assertEqual(
            [item.sequence for item in second.events],
            [5],
        )

    def test_payload_is_copied_before_caller_can_mutate_it(self):
        log = self.make_log()
        payload = {"nested": [{"safe": True}]}

        stored = self.append(log, 1, payload)
        payload["nested"][0]["safe"] = False

        self.assertTrue(stored.to_dict()["data"]["nested"][0]["safe"])
        self.assertTrue(
            log.page().events[0].to_dict()["data"]["nested"][0]["safe"]
        )

    def test_concurrent_append_has_unique_contiguous_sequences(self):
        log = self.make_log(capacity=1_000)
        barrier = threading.Barrier(11)
        errors = []

        def publish(worker):
            try:
                barrier.wait()
                for number in range(50):
                    self.append(log, worker * 100 + number)
            except BaseException as error:
                errors.append(error)

        threads = [
            threading.Thread(target=publish, args=(worker,))
            for worker in range(10)
        ]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(5)

        self.assertEqual(errors, [])
        self.assertTrue(all(not thread.is_alive() for thread in threads))
        events = log.page(limit=1_000).events
        self.assertEqual(len(events), 500)
        self.assertEqual(
            [event.sequence for event in events],
            list(range(1, 501)),
        )
        self.assertEqual(
            len({event.event_id for event in events}),
            500,
        )

    def test_invalid_cursor_is_rejected(self):
        log = self.make_log()
        for after, limit in ((-1, 1), (0, 0), (True, 1)):
            with self.subTest(after=after, limit=limit):
                with self.assertRaises(DashboardStateError):
                    log.page(after, limit)


class RegistryTests(unittest.TestCase):
    def descriptors(self, suffix="1"):
        robot_id = "robot-{}".format(suffix)
        node_id = "node-{}".format(suffix)
        node = NodeDescriptor(
            node_id=node_id,
            display_name="Controller {}".format(suffix),
            node_kind="controller",
            robot_id=robot_id,
            controller_id="{}.controller".format(robot_id),
        )
        robot = RobotDescriptor(
            robot_id=robot_id,
            display_name="Robot {}".format(suffix),
            robot_kind="lego_ev3",
            node_ids=(node_id,),
        )
        return robot, node

    def test_multi_robot_snapshot_and_replace_are_versioned(self):
        first_robot, first_node = self.descriptors("1")
        registry = NodeRegistry(
            "server-1",
            (first_robot,),
            (first_node,),
            unix_clock_ms=MutableClock(10),
            monotonic_clock_ms=MutableClock(20),
        )
        second_robot, second_node = self.descriptors("2")

        snapshot = registry.replace(
            1,
            (first_robot, second_robot),
            (first_node, second_node),
        )

        self.assertEqual(snapshot.version, 2)
        self.assertEqual(len(snapshot.robots), 2)
        self.assertFalse(snapshot.physical_control_enabled)
        self.assertTrue(
            all(not node.control_exposed for node in snapshot.nodes)
        )
        with self.assertRaisesRegex(
            DashboardStateError,
            "version",
        ):
            registry.replace(1, (), ())

    def test_invalid_replacement_is_not_committed(self):
        robot, node = self.descriptors("1")
        registry = NodeRegistry(
            "server-1",
            (robot,),
            (node,),
            unix_clock_ms=MutableClock(10),
            monotonic_clock_ms=MutableClock(20),
        )

        with self.assertRaises(DashboardContractError):
            registry.replace(1, (), (node,))

        snapshot = registry.snapshot()
        self.assertEqual(snapshot.version, 1)
        self.assertEqual(snapshot.robots, (robot,))


class ConversationStoreTests(unittest.TestCase):
    def make_store(self):
        clock = MutableClock(1_000)
        return (
            ConversationStore(
                unix_clock_ms=clock,
                id_factory=SequentialIDs(),
            ),
            clock,
        )

    def test_full_answered_transition_appends_typed_history(self):
        store, clock = self.make_store()
        conversation = store.create(title="Test")
        clock.advance()

        queued_conversation, turn, created = store.submit_turn(
            conversation.conversation_id,
            "client-1",
            conversation.version,
            "Behöver jag paraply?",
            "research_required",
            settings_revision=3,
        )
        self.assertTrue(created)
        self.assertEqual(turn.status, "queued")
        self.assertEqual(turn.mode, "research_required")
        self.assertEqual(queued_conversation.active_turn_id, turn.turn_id)
        self.assertEqual(
            queued_conversation.messages[0].content,
            "Behöver jag paraply?",
        )

        clock.advance()
        _, running = store.mark_running(turn.turn_id)
        self.assertEqual(running.status, "running")
        clock.advance()
        completed_conversation, answered = store.complete_answer(
            turn.turn_id,
            "Nej, ingen nederbörd just nu.",
            ("evidence-1",),
        )

        self.assertTrue(answered.terminal)
        self.assertEqual(answered.status, "answered")
        self.assertIsNone(completed_conversation.active_turn_id)
        self.assertEqual(
            [message.role for message in completed_conversation.messages],
            ["user", "assistant"],
        )
        self.assertEqual(
            completed_conversation.messages[-1].citation_ids,
            ("evidence-1",),
        )
        self.assertEqual(
            store.history(conversation.conversation_id),
            completed_conversation.messages,
        )

    def test_idempotent_replay_returns_existing_turn(self):
        store, _clock = self.make_store()
        conversation = store.create()
        current, first, created = store.submit_turn(
            conversation.conversation_id,
            "client-1",
            1,
            "Hej",
            "conversation",
        )

        replay_conversation, replay, replay_created = store.submit_turn(
            conversation.conversation_id,
            "client-1",
            999,
            "Hej",
            "conversation",
        )

        self.assertTrue(created)
        self.assertFalse(replay_created)
        self.assertEqual(replay, first)
        self.assertEqual(replay_conversation, current)
        with self.assertRaisesRegex(
            DashboardStateError,
            "other content",
        ):
            store.submit_turn(
                conversation.conversation_id,
                "client-1",
                current.version,
                "Annat",
                "conversation",
            )

    def test_version_and_single_active_turn_are_enforced(self):
        store, _clock = self.make_store()
        conversation = store.create()
        with self.assertRaisesRegex(
            DashboardStateError,
            "version",
        ):
            store.submit_turn(
                conversation.conversation_id,
                "client-1",
                99,
                "Hej",
                "conversation",
            )
        current, _turn, _ = store.submit_turn(
            conversation.conversation_id,
            "client-1",
            1,
            "Hej",
            "conversation",
        )
        with self.assertRaisesRegex(
            DashboardStateError,
            "active turn",
        ):
            store.submit_turn(
                conversation.conversation_id,
                "client-2",
                current.version,
                "En till",
                "conversation",
            )

    def test_queued_turn_can_fail_atomically_without_runner_execution(self):
        store, clock = self.make_store()
        conversation = store.create()
        queued_conversation, turn, _ = store.submit_turn(
            conversation.conversation_id,
            "client-cancelled",
            conversation.version,
            "Kör inte denna",
            "conversation",
        )
        clock.advance()

        failed_conversation, failed = store.fail_queued(
            turn.turn_id,
            "service_stopping",
        )

        self.assertEqual(failed.status, "failed")
        self.assertEqual(failed.error_code, "service_stopping")
        self.assertEqual(
            failed.started_at_unix_ms,
            failed.completed_at_unix_ms,
        )
        self.assertIsNone(failed_conversation.active_turn_id)
        self.assertEqual(
            failed_conversation.messages,
            queued_conversation.messages,
        )
        with self.assertRaisesRegex(
            DashboardStateError,
            "queued",
        ):
            store.fail_queued(turn.turn_id, "service_stopping")

    def test_clarification_and_failure_are_terminal_without_fake_answer(self):
        store, clock = self.make_store()
        conversation = store.create()
        current, clarify_turn, _ = store.submit_turn(
            conversation.conversation_id,
            "client-1",
            1,
            "Hur är vädret?",
            "research_required",
        )
        store.mark_running(clarify_turn.turn_id)
        clock.advance()
        clarified, result = store.complete_clarification(
            clarify_turn.turn_id,
            "Vilken plats menar du?",
        )
        self.assertEqual(result.status, "clarification_required")
        self.assertEqual(clarified.messages[-1].role, "assistant")

        current, failed_turn, _ = store.submit_turn(
            conversation.conversation_id,
            "client-2",
            clarified.version,
            "Försök igen",
            "conversation",
        )
        store.mark_running(failed_turn.turn_id)
        clock.advance()
        failed_conversation, failed = store.complete_failed(
            failed_turn.turn_id,
            "planner_failed",
        )
        self.assertEqual(failed.status, "failed")
        self.assertEqual(failed.error_code, "planner_failed")
        self.assertEqual(
            len(failed_conversation.messages),
            len(current.messages),
        )

    def test_invalid_transition_is_rejected_without_mutation(self):
        store, _clock = self.make_store()
        conversation = store.create()
        current, turn, _ = store.submit_turn(
            conversation.conversation_id,
            "client-1",
            1,
            "Hej",
            "conversation",
        )

        with self.assertRaisesRegex(
            DashboardStateError,
            "running",
        ):
            store.complete_answer(turn.turn_id, "Hej")

        self.assertEqual(store.get_turn(turn.turn_id), turn)
        self.assertEqual(store.get(conversation.conversation_id), current)

    def test_cold_import_does_not_load_execution_stack(self):
        forbidden = (
            "robot_agent.agent_loop",
            "robot_agent.contract",
            "robot_agent.robot_api",
            "robot_agent.safety",
            "robot_agent.simulated_robot",
            "robot_agent.supervisor_transport",
        )
        program = (
            "import json,sys;"
            "import robot_agent.dashboard_contract;"
            "import robot_agent.dashboard_state;"
            "forbidden={};"
            "print(json.dumps([name for name in forbidden "
            "if name in sys.modules]))"
        ).format(repr(forbidden))
        environment = dict(os.environ)
        environment["PYTHONDONTWRITEBYTECODE"] = "1"

        completed = subprocess.run(
            [sys.executable, "-c", program],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
            env=environment,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertEqual(json.loads(completed.stdout), [])


if __name__ == "__main__":
    unittest.main()
