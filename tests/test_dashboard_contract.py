import json
import unittest

from robot_agent.dashboard_contract import (
    ChatMessage,
    ChatTurn,
    Conversation,
    DashboardContractError,
    DashboardSettings,
    EventPage,
    ExperimentDescriptor,
    NodeDescriptor,
    RESPONSE_LOCALES,
    RESPONSE_LOCALE_NAMES,
    RegistrySnapshot,
    RobotDescriptor,
    TechnicalEvent,
    strict_json_loads,
    strict_json_object,
)


def event(data=None):
    return TechnicalEvent(
        server_instance_id="server-1",
        sequence=1,
        event_id="event-1",
        recorded_at_unix_ms=100,
        recorded_at_monotonic_ms=50,
        level="info",
        category="chat",
        event_type="chat.turn_queued",
        source_id="dashboard",
        message="Turn queued",
        data={} if data is None else data,
        conversation_id="conversation-1",
        turn_id="turn-1",
    )


class StrictJSONTests(unittest.TestCase):
    def test_decodes_utf8_object_without_interpreting_language(self):
        value = strict_json_object(
            '{"content":"två gånger till","count":2}'.encode("utf-8")
        )

        self.assertEqual(
            value,
            {"content": "två gånger till", "count": 2},
        )

    def test_rejects_duplicate_keys_nan_bad_utf8_and_non_object(self):
        values = (
            b'{"a":1,"a":2}',
            b'{"value":NaN}',
            b"\xff",
        )
        for raw in values:
            with self.subTest(raw=raw):
                with self.assertRaises(DashboardContractError):
                    strict_json_loads(raw)
        with self.assertRaisesRegex(
            DashboardContractError,
            "JSON object",
        ):
            strict_json_object(b"[]")

    def test_response_locale_names_cover_the_exact_allowlist(self):
        self.assertEqual(
            tuple(RESPONSE_LOCALE_NAMES),
            RESPONSE_LOCALES,
        )


class DashboardSettingsTests(unittest.TestCase):
    def test_defaults_are_research_limits_compatible(self):
        settings = DashboardSettings.defaults()

        self.assertEqual(settings.revision, 1)
        self.assertEqual(settings.chat_mode, "conversation")
        self.assertFalse(settings.require_evidence)
        self.assertEqual(
            settings.to_research_limits_kwargs(),
            {
                "max_elapsed_ms": 30_000,
                "max_planner_latency_ms": 10_000,
                "max_planner_turns": 6,
                "max_tool_calls": 1,
                "max_replans": 4,
                "tool_request_ttl_ms": 8_000,
                "evidence_ttl_ms": 600_000,
                "max_weather_observation_skew_ms": 7_200_000,
            },
        )
        view = settings.to_dict()
        self.assertEqual(view["schema"], "dashboard-settings/v1")
        self.assertEqual(view["persistence"], "memory_only")
        self.assertTrue(view["resets_on_restart"])

    def test_update_is_exact_nested_and_returns_new_revision(self):
        original = DashboardSettings.defaults()

        updated = original.with_updates(
            {
                "chat_mode": "research_required",
                "log_level": "warning",
                "research": {
                    "max_elapsed_ms": 20_000,
                    "max_planner_latency_ms": 5_000,
                    "tool_request_ttl_ms": 4_000,
                },
            }
        )

        self.assertEqual(updated.revision, 2)
        self.assertTrue(updated.require_evidence)
        self.assertEqual(updated.max_elapsed_ms, 20_000)
        self.assertEqual(original.revision, 1)
        self.assertEqual(original.chat_mode, "conversation")

    def test_rejects_unknown_settings_and_invalid_cross_field_ranges(self):
        settings = DashboardSettings.defaults()
        invalid = (
            {"model": "anything"},
            {"research": {"unknown": 1}},
            {"research": {}},
            {"research": {"max_planner_latency_ms": 30_001}},
            {"research": {"max_elapsed_ms": True}},
        )
        for changes in invalid:
            with self.subTest(changes=changes):
                with self.assertRaises(DashboardContractError):
                    settings.with_updates(changes)


class EventContractTests(unittest.TestCase):
    def test_event_payload_is_deep_copied_and_immutable(self):
        payload = {"nested": [{"value": 1}]}
        item = event(payload)
        payload["nested"][0]["value"] = 99

        self.assertEqual(
            item.to_dict()["data"],
            {"nested": [{"value": 1}]},
        )
        with self.assertRaises(TypeError):
            item.data["new"] = True
        with self.assertRaises(TypeError):
            item.data["nested"][0]["value"] = 2

    def test_event_view_separates_source_and_correlation(self):
        view = event().to_dict()

        self.assertEqual(view["source"]["source_id"], "dashboard")
        self.assertEqual(
            view["correlation"]["conversation_id"],
            "conversation-1",
        )
        self.assertNotIn("content", json.dumps(view))

    def test_event_page_is_typed(self):
        page = EventPage(
            server_instance_id="server-1",
            after_sequence=0,
            oldest_sequence=1,
            newest_sequence=1,
            next_after_sequence=1,
            gap=False,
            dropped_total=0,
            events=(event(),),
        )

        self.assertEqual(
            page.to_dict()["events"][0]["sequence"],
            1,
        )


class RegistryContractTests(unittest.TestCase):
    def descriptors(self):
        node = NodeDescriptor(
            node_id="ev3-main",
            display_name="EV3 controller",
            node_kind="controller",
            robot_id="ev3rstorm-01",
            controller_id="ev3rstorm-01.ev3-main",
            lifecycle="declared",
            capabilities=("telemetry_declared",),
        )
        robot = RobotDescriptor(
            robot_id="ev3rstorm-01",
            display_name="EV3RSTORM",
            robot_kind="lego_ev3",
            lifecycle="declared",
            node_ids=(node.node_id,),
        )
        return robot, node

    def test_registry_is_descriptive_and_never_exposes_control(self):
        robot, node = self.descriptors()
        snapshot = RegistrySnapshot(
            server_instance_id="server-1",
            version=1,
            generated_at_unix_ms=100,
            generated_at_monotonic_ms=50,
            robots=(robot,),
            nodes=(node,),
        )
        view = snapshot.to_dict()

        self.assertFalse(view["physical_control_enabled"])
        self.assertFalse(view["robots"][0]["control_exposed"])
        self.assertFalse(view["nodes"][0]["control_exposed"])
        self.assertEqual(view["nodes"][0]["lifecycle"], "declared")
        self.assertIsNone(view["nodes"][0]["last_observed_at_unix_ms"])

    def test_control_true_and_inconsistent_association_are_rejected(self):
        with self.assertRaisesRegex(
            DashboardContractError,
            "physical control",
        ):
            NodeDescriptor(
                node_id="bad",
                display_name="Bad node",
                node_kind="controller",
                control_exposed=True,
            )

        robot, node = self.descriptors()
        wrong = RobotDescriptor(
            robot_id=robot.robot_id,
            display_name=robot.display_name,
            robot_kind=robot.robot_kind,
            node_ids=(),
        )
        with self.assertRaisesRegex(
            DashboardContractError,
            "association",
        ):
            RegistrySnapshot(
                server_instance_id="server-1",
                version=1,
                generated_at_unix_ms=100,
                generated_at_monotonic_ms=50,
                robots=(wrong,),
                nodes=(node,),
            )

    def test_observed_lifecycle_requires_both_clock_domains(self):
        with self.assertRaisesRegex(
            DashboardContractError,
            "observation times",
        ):
            NodeDescriptor(
                node_id="camera-1",
                display_name="Camera",
                node_kind="camera",
                lifecycle="observed_online",
                source_id="camera-source-1",
            )

    def test_registry_display_name_key_is_typed_and_optional(self):
        localized = NodeDescriptor(
            node_id="camera-1",
            display_name="Front Camera",
            display_name_key="registry.names.front_camera",
            node_kind="camera",
            source_id="camera-source-1",
        )
        raw_product_name = NodeDescriptor(
            node_id="model-1",
            display_name="LM Studio",
            node_kind="model_server",
        )

        self.assertEqual(
            localized.to_dict()["display_name_key"],
            "registry.names.front_camera",
        )
        self.assertIsNone(raw_product_name.to_dict()["display_name_key"])
        with self.assertRaisesRegex(
            DashboardContractError,
            "display-name catalog key",
        ):
            NodeDescriptor(
                node_id="camera-2",
                display_name="Unknown Camera",
                display_name_key="registry.names.unknown",
                node_kind="camera",
                source_id="camera-source-2",
            )


class ExperimentContractTests(unittest.TestCase):
    def descriptor(self, **changes):
        values = {
            "experiment_id": "EXP-F1-IR-DYN-002",
            "title_key": "experiments.curated.dynamic_ir.title",
            "summary_key": "experiments.curated.dynamic_ir.summary",
            "status": "verified",
            "component_ids": ("ev3rstorm-01.ev3-main",),
        }
        values.update(changes)
        return ExperimentDescriptor(**values)

    def test_serializes_locale_neutral_catalog_keys(self):
        self.assertEqual(
            self.descriptor().to_dict(),
            {
                "schema": "dashboard-experiment/v1",
                "experiment_id": "EXP-F1-IR-DYN-002",
                "title_key": "experiments.curated.dynamic_ir.title",
                "summary_key": "experiments.curated.dynamic_ir.summary",
                "status": "verified",
                "component_ids": ["ev3rstorm-01.ev3-main"],
            },
        )

    def test_rejects_unknown_keys_status_and_invalid_components(self):
        invalid = (
            {"title_key": "experiments.curated.unknown.title"},
            {"summary_key": "experiments.curated.unknown.summary"},
            {"status": "running"},
            {"component_ids": ["not-a-tuple"]},
            {"component_ids": ("duplicate", "duplicate")},
        )
        for changes in invalid:
            with self.subTest(changes=changes):
                with self.assertRaises(DashboardContractError):
                    self.descriptor(**changes)


class ConversationContractTests(unittest.TestCase):
    def test_conversation_returns_typed_history(self):
        message = ChatMessage(
            message_id="message-1",
            turn_id="turn-1",
            role="user",
            content="Hej roboten",
            created_at_unix_ms=100,
        )
        conversation = Conversation(
            conversation_id="conversation-1",
            version=2,
            created_at_unix_ms=100,
            updated_at_unix_ms=100,
            messages=(message,),
            active_turn_id="turn-1",
        )

        view = conversation.to_dict()
        self.assertEqual(view["context_mode"], "typed_history")
        self.assertEqual(view["messages"][0]["role"], "user")
        self.assertEqual(view["messages"][0]["content"], "Hej roboten")

    def test_turn_shape_is_bound_to_status_and_explicit_mode(self):
        queued = ChatTurn(
            turn_id="turn-1",
            conversation_id="conversation-1",
            client_request_id="request-1",
            mode="research_required",
            response_locale="en",
            settings_revision=2,
            status="queued",
            content="Vädret?",
            created_at_unix_ms=100,
        )

        self.assertTrue(queued.to_dict()["require_evidence"])
        self.assertEqual(queued.to_dict()["response_locale"], "en")
        self.assertFalse(queued.terminal)
        with self.assertRaisesRegex(
            DashboardContractError,
            "status",
        ):
            ChatTurn(
                turn_id="turn-1",
                conversation_id="conversation-1",
                client_request_id="request-1",
                mode="conversation",
                response_locale="sv",
                settings_revision=1,
                status="answered",
                content="Hej",
                created_at_unix_ms=100,
                answer_text="Hej",
            )
        with self.assertRaisesRegex(
            DashboardContractError,
            "locale",
        ):
            ChatTurn(
                turn_id="turn-2",
                conversation_id="conversation-1",
                client_request_id="request-2",
                mode="conversation",
                response_locale="fr",
                settings_revision=1,
                status="queued",
                content="Bonjour",
                created_at_unix_ms=100,
            )


if __name__ == "__main__":
    unittest.main()
