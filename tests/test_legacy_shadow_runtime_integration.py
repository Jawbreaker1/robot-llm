from pathlib import Path
from types import SimpleNamespace
import json
import tempfile
import threading
import unittest
from unittest import mock
import uuid

from robot_agent.legacy_control_shadow import (
    FailOpenLegacyShadowObserver,
    InMemoryLegacyShadowJournal,
)
from robot_agent.navigation_memory_store import NavigationMemoryStore
from robot_agent.physical_navigation_adapter import (
    PhysicalNavigationRuntimeAdapter,
)
from robot_agent.physical_navigation_runtime import (
    PhysicalNavigationRuntime,
    PhysicalNavigationRuntimeConfig,
)
from robot_agent.persistent_legacy_shadow_journal import (
    PersistentLegacyShadowSession,
)
from tests.test_physical_navigation_core import (
    FakeRuntimePlanner,
    FakeRuntimeTransport,
)


def run_episode(directory, *, name, canonical_shadow=None):
    memory = NavigationMemoryStore.load(
        path=Path(directory) / "{}-memory.json".format(name),
        robot_id="ev3rstorm-01",
        controller_instance_id="ev3-main",
        reset=True,
        clock_ms=lambda: 1_000,
        uuid_factory=lambda: uuid.UUID(int=501),
    )
    transport = FakeRuntimeTransport()
    planner = FakeRuntimePlanner()
    runtime = PhysicalNavigationRuntime(
        episode_id="episode-shadow-equivalence",
        config=PhysicalNavigationRuntimeConfig(
            goal="Move forward at least 100 mm",
            locale="en",
            minimum_forward_progress_mm=100,
            max_turns=3,
            max_episode_seconds=10,
        ),
        transport=transport,
        planner=planner,
        memory=memory,
        monotonic=lambda: 0.0,
        unix_ms=lambda: 2_000,
        canonical_shadow=canonical_shadow,
    )
    return runtime.run(), transport, planner


def adapter_context():
    return SimpleNamespace(
        episode_id="episode-adapter-shadow",
        request=SimpleNamespace(goal="Observe", locale="en"),
        settings=SimpleNamespace(
            model="test-model",
            max_episode_ms=10_000,
            speech_enabled=False,
        ),
        stop_requested=threading.Event(),
        emergency_stop_requested=threading.Event(),
        publish=lambda _update: None,
    )


class LegacyShadowRuntimeIntegrationTests(unittest.TestCase):
    def test_real_runtime_payloads_persist_without_disabling_shadow(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.ndjson"
            shadow = PersistentLegacyShadowSession(
                episode_id="episode-shadow-equivalence",
                path=path,
            )
            result, _transport, _planner = run_episode(
                directory,
                name="persistent-shadow",
                canonical_shadow=shadow,
            )
            self.assertTrue(shadow.close())

            records = tuple(
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
            )

        self.assertTrue(result.shutdown_clean)
        self.assertTrue(shadow.status.observer.enabled)
        self.assertIsNone(shadow.status.journal.error)
        self.assertEqual(records[0]["stage"], "episode_start")
        self.assertEqual(records[-1]["stage"], "terminal")
        self.assertNotIn(
            "shadow_disabled",
            tuple(record["stage"] for record in records),
        )

    def test_shadow_observes_and_projects_without_changing_legacy_calls(self):
        journal = InMemoryLegacyShadowJournal(capacity=128)
        observer = FailOpenLegacyShadowObserver(
            episode_id="episode-shadow-equivalence",
            try_write=journal.try_write,
        )

        with tempfile.TemporaryDirectory() as directory:
            baseline, baseline_transport, baseline_planner = run_episode(
                directory,
                name="baseline",
            )
            shadowed, shadow_transport, shadow_planner = run_episode(
                directory,
                name="shadowed",
                canonical_shadow=observer,
            )

        self.assertEqual(shadowed.to_dict(), baseline.to_dict())
        self.assertEqual(shadow_transport.calls, baseline_transport.calls)
        self.assertEqual(shadow_planner.calls, baseline_planner.calls)
        self.assertEqual(shadow_planner.calls, shadowed.model_calls)
        self.assertTrue(observer.enabled)

        records = journal.snapshot()
        stages = tuple(item.stage for item in records)
        self.assertEqual(stages[0], "episode_start")
        self.assertEqual(stages[-1], "terminal")
        self.assertEqual(stages.count("planner_input"), shadowed.model_calls)
        self.assertEqual(
            stages.count("validated_decision"),
            shadowed.model_calls,
        )
        self.assertEqual(
            stages.count("canonical_projection"),
            shadowed.model_calls,
        )
        self.assertEqual(
            stages.count("legacy_execution_observed"),
            2,
        )
        projections = [
            item.to_dict()["facts"]["projection"]
            for item in records
            if item.stage == "canonical_projection"
        ]
        self.assertEqual(projections[-1]["terminal"], True)
        self.assertIsNone(projections[-1]["intent"])
        self.assertEqual(
            projections[0]["classification"]["receipt_parity"],
            "NOT_EVALUATED",
        )
        terminal = records[-1].to_dict()["facts"]
        self.assertEqual(terminal["extra_model_calls"], 0)
        self.assertEqual(terminal["physical_authority"], "legacy")
        self.assertEqual(terminal["canonical_receipt_created"], False)

    def test_exploding_observer_is_disabled_without_changing_episode(self):
        class ExplodingObserver:
            def __init__(self):
                self.calls = []

            def observe(self, stage, **_facts):
                self.calls.append(stage)
                raise RuntimeError("shadow failed")

        exploding = ExplodingObserver()
        with tempfile.TemporaryDirectory() as directory:
            baseline, baseline_transport, baseline_planner = run_episode(
                directory,
                name="baseline-exploding",
            )
            shadowed, shadow_transport, shadow_planner = run_episode(
                directory,
                name="shadow-exploding",
                canonical_shadow=exploding,
            )

        self.assertEqual(shadowed.to_dict(), baseline.to_dict())
        self.assertEqual(shadow_transport.calls, baseline_transport.calls)
        self.assertEqual(shadow_planner.calls, baseline_planner.calls)
        self.assertEqual(
            exploding.calls,
            ["episode_start", "shadow_disabled"],
        )

    def test_shadow_fact_preparation_failure_cannot_change_episode(self):
        journal = InMemoryLegacyShadowJournal(capacity=128)
        observer = FailOpenLegacyShadowObserver(
            episode_id="episode-shadow-equivalence",
            try_write=journal.try_write,
        )
        with tempfile.TemporaryDirectory() as directory:
            baseline, baseline_transport, baseline_planner = run_episode(
                directory,
                name="baseline-preparation-failure",
            )
            with mock.patch.object(
                PhysicalNavigationRuntime,
                "_shadow_navigation_identity",
                side_effect=RuntimeError("shadow preparation failed"),
            ):
                shadowed, shadow_transport, shadow_planner = run_episode(
                    directory,
                    name="shadow-preparation-failure",
                    canonical_shadow=observer,
                )

        self.assertEqual(shadowed.to_dict(), baseline.to_dict())
        self.assertEqual(shadow_transport.calls, baseline_transport.calls)
        self.assertEqual(shadow_planner.calls, baseline_planner.calls)
        self.assertFalse(observer.enabled)

    def test_broken_observe_attribute_is_ignored_during_construction(self):
        class BrokenObserver:
            @property
            def observe(self):
                raise RuntimeError("broken observe property")

        with tempfile.TemporaryDirectory() as directory:
            baseline, baseline_transport, baseline_planner = run_episode(
                directory,
                name="baseline-broken-property",
            )
            shadowed, shadow_transport, shadow_planner = run_episode(
                directory,
                name="shadow-broken-property",
                canonical_shadow=BrokenObserver(),
            )

        self.assertEqual(shadowed.to_dict(), baseline.to_dict())
        self.assertEqual(shadow_transport.calls, baseline_transport.calls)
        self.assertEqual(shadow_planner.calls, baseline_planner.calls)

    def test_adapter_shadow_factory_failure_is_fail_open(self):
        adapter = PhysicalNavigationRuntimeAdapter(
            transport_factory=object,
            planner_factory=lambda _model: object(),
            memory_factory=object,
            canonical_shadow_factory=lambda **_kwargs: (_ for _ in ()).throw(
                RuntimeError("journal unavailable")
            ),
        )

        with mock.patch(
            "robot_agent.physical_navigation_adapter."
            "PhysicalNavigationRuntime"
        ) as runtime_type:
            runtime_type.return_value.run.return_value = SimpleNamespace(
                terminal_reason="goal_completed",
                completed=True,
                model_latency_ms=0,
            )
            result = adapter.run(adapter_context())

        self.assertEqual(result.terminal_reason, "goal_completed")
        self.assertIsNone(
            runtime_type.call_args.kwargs["canonical_shadow"]
        )

    def test_adapter_passes_and_requests_close_when_runtime_fails(self):
        class ClosableShadow:
            def __init__(self):
                self.close_requested = 0

            def observe(self, _stage, **_facts):
                return None

            def request_close(self):
                self.close_requested += 1

        shadow = ClosableShadow()
        adapter = PhysicalNavigationRuntimeAdapter(
            transport_factory=object,
            planner_factory=lambda _model: object(),
            memory_factory=object,
            canonical_shadow_factory=lambda **_kwargs: shadow,
        )

        with mock.patch(
            "robot_agent.physical_navigation_adapter."
            "PhysicalNavigationRuntime"
        ) as runtime_type:
            runtime_type.return_value.run.side_effect = RuntimeError(
                "legacy runtime failed"
            )
            with self.assertRaisesRegex(RuntimeError, "legacy runtime failed"):
                adapter.run(adapter_context())

        self.assertIs(
            runtime_type.call_args.kwargs["canonical_shadow"],
            shadow,
        )
        self.assertEqual(shadow.close_requested, 1)

    def test_broken_close_lookup_cannot_mask_result_or_hold_active_slot(self):
        class BrokenCloseShadow:
            def observe(self, _stage, **_facts):
                return None

            @property
            def request_close(self):
                raise RuntimeError("broken close property")

        shadow = BrokenCloseShadow()
        adapter = PhysicalNavigationRuntimeAdapter(
            transport_factory=object,
            planner_factory=lambda _model: object(),
            memory_factory=object,
            canonical_shadow_factory=lambda **_kwargs: shadow,
        )

        with mock.patch(
            "robot_agent.physical_navigation_adapter."
            "PhysicalNavigationRuntime"
        ) as runtime_type:
            runtime_type.return_value.run.return_value = SimpleNamespace(
                terminal_reason="goal_completed",
                completed=True,
                model_latency_ms=0,
            )
            result = adapter.run(adapter_context())

        self.assertEqual(result.terminal_reason, "goal_completed")
        self.assertIsNone(adapter._active)


if __name__ == "__main__":
    unittest.main()
