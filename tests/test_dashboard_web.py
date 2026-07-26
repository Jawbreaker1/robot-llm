from html.parser import HTMLParser
from pathlib import Path
import unittest


WEB_ROOT = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "robot_agent"
    / "dashboard_web"
)


class AssetParser(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.ids = []
        self.attributes = []
        self.elements = []
        self.scripts = []
        self.links = []
        self._script_without_src = False
        self.inline_script_data = []

    def handle_starttag(self, tag, attrs):
        values = dict(attrs)
        self.elements.append((tag, values))
        self.attributes.extend((tag, name, value) for name, value in attrs)
        if "id" in values:
            self.ids.append(values["id"])
        if tag == "script":
            self.scripts.append(values.get("src"))
            self._script_without_src = "src" not in values
        if tag == "link":
            self.links.append(values.get("href"))

    def handle_endtag(self, tag):
        if tag == "script":
            self._script_without_src = False

    def handle_data(self, data):
        if self._script_without_src and data.strip():
            self.inline_script_data.append(data)


class DashboardWebContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = (WEB_ROOT / "index.html").read_text(encoding="utf-8")
        cls.css = (WEB_ROOT / "styles.css").read_text(encoding="utf-8")
        cls.javascript = (WEB_ROOT / "app.js").read_text(encoding="utf-8")
        cls.parser = AssetParser()
        cls.parser.feed(cls.html)

    def test_html_has_one_token_placeholder_and_external_assets_only(self):
        self.assertEqual(
            self.html.count("__ROBOT_DASHBOARD_TOKEN__"),
            1,
        )
        self.assertEqual(
            self.parser.scripts,
            ["assets/app.js"],
        )
        self.assertIn("assets/styles.css", self.parser.links)
        self.assertEqual(self.parser.inline_script_data, [])
        self.assertNotIn("<style", self.html.lower())

    def test_html_ids_are_unique_and_core_surfaces_exist(self):
        self.assertEqual(
            len(self.parser.ids),
            len(set(self.parser.ids)),
        )
        required = {
            "view-workbench",
            "view-bodies",
            "view-events",
            "view-experiments",
            "view-settings",
            "message-feed",
            "composer-form",
            "registry-tree",
            "event-table-body",
            "settings-form",
            "status-motion",
        }
        self.assertTrue(required <= set(self.parser.ids))
        self.assertIn('lang="sv"', self.html)

    def test_no_inline_handlers_external_urls_or_physical_controls(self):
        for tag, name, value in self.parser.attributes:
            with self.subTest(tag=tag, name=name):
                self.assertFalse(name.lower().startswith("on"))
                self.assertNotEqual(name.lower(), "style")
                if name.lower() in ("href", "src", "action"):
                    self.assertFalse(
                        (value or "").startswith(("http://", "https://", "//"))
                    )
        forbidden_control_ids = (
            "move-button",
            "drive-button",
            "motor-button",
            "ssh-button",
            "tts-button",
            "stop-button",
        )
        for control_id in forbidden_control_ids:
            self.assertNotIn('id="{}"'.format(control_id), self.html)

    def test_javascript_uses_dom_text_and_only_declared_api_shapes(self):
        forbidden = (
            "innerHTML",
            "outerHTML",
            "insertAdjacentHTML",
            "document.write",
            "eval(",
            "new Function",
        )
        for token in forbidden:
            self.assertNotIn(token, self.javascript)
        self.assertIn("textContent", self.javascript)
        self.assertIn("replaceChildren", self.javascript)
        self.assertIn(
            "/api/v1/events?after_sequence=",
            self.javascript,
        )
        self.assertIn(
            "/api/v1/runtime/lm-studio/probe",
            self.javascript,
        )
        for forbidden_route in (
            "/move",
            "/drive",
            "/motor",
            "/ssh",
            "/tts",
            "/stop",
        ):
            self.assertNotIn(forbidden_route, self.javascript)

    def test_ui_does_not_classify_natural_language_with_regex(self):
        self.assertNotIn("RegExp(", self.javascript)
        self.assertNotIn(".match(", self.javascript)
        self.assertNotIn(".test(", self.javascript)
        self.assertIn(
            'mode: byId("turn-mode").value',
            self.javascript,
        )

    def test_css_has_responsive_accessible_motion_aware_layout(self):
        self.assertIn("@media (max-width: 1220px)", self.css)
        self.assertIn("@media (max-width: 820px)", self.css)
        self.assertIn("@media (prefers-reduced-motion: reduce)", self.css)
        self.assertIn(":focus-visible", self.css)
        self.assertIn("-apple-system", self.css)
        self.assertIn("--canvas: #0b0e10", self.css.lower())
        self.assertNotIn("linear-gradient(", self.css)
        self.assertNotIn("@import", self.css)

    def test_server_instance_change_forces_reload_before_reusing_state(self):
        comparison = "previousInstanceId !== nextInstanceId"
        invalidate_polling = "state.turnPollGeneration += 1"
        reload_page = "window.location.reload()"

        self.assertIn("server_instance_id", self.javascript)
        self.assertIn(comparison, self.javascript)
        self.assertIn(invalidate_polling, self.javascript)
        self.assertIn(reload_page, self.javascript)
        self.assertLess(
            self.javascript.index(comparison),
            self.javascript.index(reload_page),
        )

    def test_turn_polling_has_bounded_failure_budget_and_terminal_fallback(self):
        self.assertIn("turnPollFailures: 0", self.javascript)
        self.assertIn("state.turnPollFailures += 1", self.javascript)
        self.assertIn("state.turnPollFailures >= 8", self.javascript)
        self.assertIn('error_code: "turn_poll_failed"', self.javascript)
        self.assertIn("Math.min(", self.javascript)
        self.assertIn(
            "window.setTimeout(() => pollTurn(turnId, generation), retryDelay)",
            self.javascript,
        )

    def test_historical_citations_fetch_their_own_turn_evidence(self):
        self.assertIn("message.turn_id", self.javascript)
        self.assertIn(
            "/api/v1/turns/${encodeURIComponent(message.turn_id)}",
            self.javascript,
        )
        self.assertIn(
            "renderEvidence(safeObject(payload.turn), citationId)",
            self.javascript,
        )

    def test_settings_input_ranges_match_the_backend_contract(self):
        elements_by_id = {
            attrs["id"]: (tag, attrs)
            for tag, attrs in self.parser.elements
            if "id" in attrs
        }
        expected_ranges = {
            "setting-planner-latency-ms": ("1", "300000", "1"),
            "setting-max-planner-turns": ("1", "100", "1"),
            "setting-max-replans": ("0", "100", "1"),
            "setting-max-tool-calls": ("0", "100", "1"),
            "setting-max-elapsed-ms": ("1", "300000", "1"),
            "setting-tool-request-ttl-ms": ("1", "300000", "1"),
            "setting-evidence-ttl-ms": ("1", "86400000", "1"),
            "setting-weather-skew-ms": ("1", "86400000", "1"),
        }

        for element_id, expected in expected_ranges.items():
            with self.subTest(element_id=element_id):
                tag, attrs = elements_by_id[element_id]
                self.assertEqual(tag, "input")
                self.assertEqual(attrs.get("type"), "number")
                self.assertEqual(
                    (attrs.get("min"), attrs.get("max"), attrs.get("step")),
                    expected,
                )

    def test_runtime_requires_the_configured_model_to_be_loaded(self):
        self.assertIn("configured_model_loaded", self.javascript)
        self.assertIn("state.modelReady", self.javascript)
        self.assertIn("modelLoaded === true", self.javascript)
        self.assertIn("state.modelReady === true", self.javascript)
        self.assertIn("konfigurerad modell ej laddad", self.javascript)

    def test_chat_history_uses_a_separate_live_announcer(self):
        elements_by_id = {
            attrs["id"]: attrs
            for _, attrs in self.parser.elements
            if "id" in attrs
        }
        message_feed = elements_by_id["message-feed"]
        announcer = elements_by_id["chat-announcer"]

        self.assertEqual(message_feed.get("role"), "region")
        self.assertNotIn("aria-live", message_feed)
        self.assertEqual(announcer.get("aria-live"), "polite")
        self.assertEqual(announcer.get("aria-atomic"), "true")
        self.assertIn("sr-only", (announcer.get("class") or "").split())
        self.assertIn('byId("chat-announcer").textContent', self.javascript)
        self.assertIn("visibleStateChanged", self.javascript)
        self.assertIn(
            "feed.scrollHeight - feed.scrollTop - feed.clientHeight",
            self.javascript,
        )


if __name__ == "__main__":
    unittest.main()
