import json
from pathlib import Path
import sys
import tempfile
import unittest

from robot_agent.dashboard_http import (
    DashboardRouter,
    MAX_REQUEST_BYTES,
)


TOKEN = "a" * 64
HOST = "127.0.0.1:8765"
ORIGIN = "http://" + HOST


class ServiceFailure(RuntimeError):
    def __init__(self, status, code, message):
        self.status = status
        self.code = code
        super().__init__(message)


class FakeDashboardService:
    def __init__(self):
        self.calls = []

    def bootstrap(self):
        self.calls.append(("bootstrap",))
        return {
            "api_version": "robot-dashboard/v1",
            "server_instance_id": "server-1",
            "physical_control_enabled": False,
        }

    def settings(self):
        self.calls.append(("settings",))
        return {"revision": 1, "chat_mode": "conversation"}

    def update_settings(self, expected_revision, changes):
        self.calls.append(
            ("update_settings", expected_revision, changes)
        )
        if expected_revision == 99:
            raise ServiceFailure(
                409,
                "settings_revision_conflict",
                "Settings changed",
            )
        return {
            "revision": expected_revision + 1,
            **dict(changes),
        }

    def registry(self):
        self.calls.append(("registry",))
        return {
            "schema": "dashboard-registry/v1",
            "robots": [],
        }

    def create_conversation(self, title=None):
        self.calls.append(("create_conversation", title))
        return {
            "conversation_id": "conversation-1",
            "title": title,
            "version": 0,
            "turns": [],
        }

    def get_conversation(self, conversation_id):
        self.calls.append(("get_conversation", conversation_id))
        if conversation_id == "missing":
            raise ServiceFailure(
                404,
                "conversation_not_found",
                "Conversation was not found",
            )
        return {
            "conversation_id": conversation_id,
            "version": 1,
            "turns": [],
        }

    def submit_turn(
        self,
        conversation_id,
        client_request_id,
        expected_conversation_version,
        content,
        mode,
    ):
        self.calls.append(
            (
                "submit_turn",
                conversation_id,
                client_request_id,
                expected_conversation_version,
                content,
                mode,
            )
        )
        return {
            "turn_id": "turn-1",
            "conversation_id": conversation_id,
            "status": "queued",
        }

    def get_turn(self, turn_id):
        self.calls.append(("get_turn", turn_id))
        return {"turn_id": turn_id, "status": "answered"}

    def events(self, after_sequence, limit):
        self.calls.append(("events", after_sequence, limit))
        return {
            "events": [],
            "oldest_sequence": 0,
            "newest_sequence": 0,
            "dropped_total": 0,
            "gap": False,
            "next_after_sequence": after_sequence,
        }

    def probe_lm_studio(self):
        self.calls.append(("probe_lm_studio",))
        return {
            "state": "online",
            "model": "gemma",
            "base_url": "http://127.0.0.1:1234",
        }


def asset_directory():
    temporary = tempfile.TemporaryDirectory()
    root = Path(temporary.name)
    (root / "index.html").write_text(
        '<meta name="robot-dashboard-token" '
        'content="__ROBOT_DASHBOARD_TOKEN__">',
        encoding="utf-8",
    )
    (root / "styles.css").write_text(
        "body { color: white; }",
        encoding="utf-8",
    )
    (root / "app.js").write_text(
        '"use strict";',
        encoding="utf-8",
    )
    return temporary, root


class DashboardHTTPTests(unittest.TestCase):
    def setUp(self):
        self.temporary, web_root = asset_directory()
        self.service = FakeDashboardService()
        self.router = DashboardRouter(
            service=self.service,
            session_token=TOKEN,
            expected_host=HOST,
            web_root=web_root,
        )

    def tearDown(self):
        self.temporary.cleanup()

    @staticmethod
    def headers(mutation=False, authenticated=True, **extra):
        values = {"Host": HOST}
        if authenticated:
            values["X-Robot-Dashboard-Token"] = TOKEN
        if mutation:
            values.update(
                {
                    "Origin": ORIGIN,
                    "Content-Type": "application/json",
                }
            )
        values.update(extra)
        return values

    @staticmethod
    def decoded(response):
        return json.loads(response.body.decode("utf-8"))

    def test_index_substitutes_token_and_sets_strict_headers(self):
        response = self.router.handle(
            "GET",
            self.router.session_path,
            self.headers(),
        )

        self.assertEqual(response.status, 200)
        self.assertIn(TOKEN.encode("ascii"), response.body)
        self.assertNotIn(
            b"__ROBOT_DASHBOARD_TOKEN__",
            response.body,
        )
        headers = dict(response.headers)
        self.assertIn("default-src 'self'", headers["Content-Security-Policy"])
        self.assertIn("frame-ancestors 'none'", headers["Content-Security-Policy"])
        self.assertEqual(headers["Cache-Control"], "no-store, max-age=0")
        self.assertEqual(headers["X-Frame-Options"], "DENY")

    def test_bootstrap_and_registry_are_read_only(self):
        bootstrap = self.router.handle(
            "GET",
            "/api/v1/bootstrap",
            self.headers(),
        )
        registry = self.router.handle(
            "GET",
            "/api/v1/registry",
            self.headers(),
        )

        self.assertEqual(bootstrap.status, 200)
        self.assertFalse(
            self.decoded(bootstrap)["physical_control_enabled"]
        )
        self.assertEqual(registry.status, 200)
        self.assertEqual(
            self.service.calls,
            [("bootstrap",), ("registry",)],
        )

    def test_every_api_read_and_static_bootstrap_require_session(self):
        unauthenticated_api = self.router.handle(
            "GET",
            "/api/v1/events",
            self.headers(authenticated=False),
        )
        wrong_api = self.router.handle(
            "GET",
            "/api/v1/bootstrap",
            self.headers(
                **{"X-Robot-Dashboard-Token": "wrong"},
            ),
        )
        public_root = self.router.handle(
            "GET",
            "/",
            self.headers(authenticated=False),
        )
        old_asset_path = self.router.handle(
            "GET",
            "/assets/app.js",
            self.headers(authenticated=False),
        )
        wrong_session = self.router.handle(
            "GET",
            "/session/{}/".format("b" * 64),
            self.headers(authenticated=False),
        )
        session_asset = self.router.handle(
            "GET",
            self.router.session_path + "assets/app.js",
            self.headers(authenticated=False),
        )

        self.assertEqual(unauthenticated_api.status, 403)
        self.assertEqual(wrong_api.status, 403)
        self.assertEqual(public_root.status, 404)
        self.assertEqual(old_asset_path.status, 404)
        self.assertEqual(wrong_session.status, 403)
        self.assertEqual(session_asset.status, 200)
        self.assertEqual(self.service.calls, [])

    def test_mutations_require_token_origin_and_exact_json_mime(self):
        body = b"{}"
        cases = (
            (
                self.headers(
                    mutation=True,
                    **{"X-Robot-Dashboard-Token": "wrong"},
                ),
                403,
                "session_token_rejected",
            ),
            (
                self.headers(
                    mutation=True,
                    Origin="https://attacker.invalid",
                ),
                403,
                "origin_rejected",
            ),
            (
                self.headers(
                    mutation=True,
                    **{"Content-Type": "text/plain"},
                ),
                415,
                "content_type_rejected",
            ),
            (
                self.headers(
                    mutation=False,
                    authenticated=False,
                ),
                403,
                "session_token_rejected",
            ),
        )
        for headers, status, code in cases:
            with self.subTest(code=code):
                response = self.router.handle(
                    "POST",
                    "/api/v1/conversations",
                    headers,
                    body,
                )
                self.assertEqual(response.status, status)
                self.assertEqual(
                    self.decoded(response)["error"]["code"],
                    code,
                )

    def test_mutation_allows_tokened_non_browser_client_without_origin(self):
        headers = self.headers(mutation=True)
        del headers["Origin"]
        response = self.router.handle(
            "POST",
            "/api/v1/conversations",
            headers,
            b"{}",
        )
        self.assertEqual(response.status, 201)

    def test_host_header_blocks_dns_rebinding_shape(self):
        response = self.router.handle(
            "GET",
            "/api/v1/bootstrap",
            {"Host": "evil.invalid"},
        )
        self.assertEqual(response.status, 421)
        self.assertEqual(
            self.decoded(response)["error"]["code"],
            "host_rejected",
        )

    def test_settings_update_is_exact_and_propagates_revision_conflict(self):
        response = self.router.handle(
            "PUT",
            "/api/v1/settings",
            self.headers(mutation=True),
            json.dumps(
                {
                    "expected_revision": 1,
                    "changes": {
                        "max_planner_turns": 4,
                    },
                }
            ).encode("utf-8"),
        )
        self.assertEqual(response.status, 200)
        self.assertEqual(
            self.decoded(response)["settings"]["revision"],
            2,
        )

        conflict = self.router.handle(
            "PUT",
            "/api/v1/settings",
            self.headers(mutation=True),
            json.dumps(
                {
                    "expected_revision": 99,
                    "changes": {},
                }
            ).encode("utf-8"),
        )
        self.assertEqual(conflict.status, 409)
        self.assertEqual(
            self.decoded(conflict)["error"]["code"],
            "settings_revision_conflict",
        )

    def test_conversation_and_turn_routes_use_host_minted_ids(self):
        created = self.router.handle(
            "POST",
            "/api/v1/conversations",
            self.headers(mutation=True),
            json.dumps({"title": "Kvällslabbet"}).encode("utf-8"),
        )
        submitted = self.router.handle(
            "POST",
            "/api/v1/conversations/conversation-1/turns",
            self.headers(mutation=True),
            json.dumps(
                {
                    "client_request_id": "ui-1",
                    "expected_conversation_version": 0,
                    "content": "Hur är vädret?",
                    "mode": "research_required",
                }
            ).encode("utf-8"),
        )
        fetched = self.router.handle(
            "GET",
            "/api/v1/turns/turn-1",
            self.headers(),
        )

        self.assertEqual(created.status, 201)
        self.assertEqual(submitted.status, 202)
        self.assertEqual(fetched.status, 200)
        self.assertEqual(
            self.service.calls[-2],
            (
                "submit_turn",
                "conversation-1",
                "ui-1",
                0,
                "Hur är vädret?",
                "research_required",
            ),
        )

    def test_strict_json_rejects_duplicates_nonfinite_and_unknown_fields(self):
        bodies = (
            b'{"title":"a","title":"b"}',
            b'{"title":NaN}',
            b'{"unknown":true}',
        )
        for body in bodies:
            with self.subTest(body=body):
                response = self.router.handle(
                    "POST",
                    "/api/v1/conversations",
                    self.headers(mutation=True),
                    body,
                )
                self.assertEqual(response.status, 400)

    def test_request_body_and_content_encoding_are_bounded(self):
        too_large = self.router.handle(
            "POST",
            "/api/v1/conversations",
            self.headers(mutation=True),
            b" " * (MAX_REQUEST_BYTES + 1),
        )
        compressed = self.router.handle(
            "POST",
            "/api/v1/conversations",
            self.headers(
                mutation=True,
                **{"Content-Encoding": "gzip"},
            ),
            b"{}",
        )
        chunked = self.router.handle(
            "POST",
            "/api/v1/conversations",
            self.headers(
                mutation=True,
                **{"Transfer-Encoding": "chunked"},
            ),
            b"{}",
        )
        self.assertEqual(too_large.status, 413)
        self.assertEqual(compressed.status, 415)
        self.assertEqual(chunked.status, 400)

    def test_event_cursor_is_bounded_and_unknown_query_is_rejected(self):
        response = self.router.handle(
            "GET",
            "/api/v1/events?after_sequence=12&limit=25",
            self.headers(),
        )
        invalid = self.router.handle(
            "GET",
            "/api/v1/events?severity=debug",
            self.headers(),
        )
        excessive = self.router.handle(
            "GET",
            "/api/v1/events?limit=501",
            self.headers(),
        )

        self.assertEqual(response.status, 200)
        self.assertEqual(
            self.service.calls[-1],
            ("events", 12, 25),
        )
        self.assertEqual(invalid.status, 400)
        self.assertEqual(excessive.status, 400)

    def test_probe_is_an_explicit_mutation_and_accepts_no_fields(self):
        response = self.router.handle(
            "POST",
            "/api/v1/runtime/lm-studio/probe",
            self.headers(mutation=True),
            b"{}",
        )
        extra = self.router.handle(
            "POST",
            "/api/v1/runtime/lm-studio/probe",
            self.headers(mutation=True),
            b'{"url":"http://example.com"}',
        )
        self.assertEqual(response.status, 200)
        self.assertEqual(extra.status, 400)
        self.assertEqual(
            self.service.calls[-1],
            ("probe_lm_studio",),
        )

    def test_physical_ssh_tts_and_event_injection_routes_do_not_exist(self):
        routes = (
            "/api/v1/robots/ev3rstorm-01/move",
            "/api/v1/robots/ev3rstorm-01/stop",
            "/api/v1/ssh",
            "/api/v1/tts",
            "/api/v1/events",
            "/api/v1/registry",
        )
        for route in routes:
            with self.subTest(route=route):
                response = self.router.handle(
                    "POST",
                    route,
                    self.headers(mutation=True),
                    b"{}",
                )
                self.assertEqual(response.status, 404)

    def test_path_traversal_and_unknown_methods_fail_closed(self):
        for target, expected_status in (
            (
                self.router.session_path
                + "assets/../dashboard_cli.py",
                404,
            ),
            (
                self.router.session_path
                + "assets/%2e%2e/dashboard_cli.py",
                404,
            ),
            ("//assets/app.js", 400),
        ):
            with self.subTest(target=target):
                response = self.router.handle(
                    "GET",
                    target,
                    self.headers(),
                )
                self.assertEqual(response.status, expected_status)
        method = self.router.handle(
            "DELETE",
            "/api/v1/settings",
            self.headers(),
        )
        self.assertEqual(method.status, 405)

    def test_cold_http_import_does_not_load_execution_modules(self):
        forbidden = (
            "robot_agent.agent_loop",
            "robot_agent.contract",
            "robot_agent.robot_api",
            "robot_agent.safety",
            "robot_agent.shadow_commentary",
            "robot_agent.simulated_robot",
            "robot_agent.supervisor_transport",
        )
        script = (
            "import json,sys;"
            "import robot_agent.dashboard_http;"
            "forbidden={};"
            "print(json.dumps([x for x in forbidden if x in sys.modules]))"
        ).format(repr(forbidden))
        import os
        import subprocess

        environment = dict(os.environ)
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        completed = subprocess.run(
            [sys.executable, "-c", script],
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
