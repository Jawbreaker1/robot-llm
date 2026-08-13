import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

from robot_agent.dashboard_http import (
    DashboardRouter,
    MAX_SPATIAL_MAP_RESPONSE_BYTES,
    MAX_REQUEST_BYTES,
    STT_LANGUAGE_HEADER,
    STT_REQUEST_ID_HEADER,
    STT_REQUESTS_PATH,
    STT_TRANSCRIPTIONS_PATH,
)
from robot_agent.stt_contract import MAX_STT_AUDIO_BYTES


TOKEN = "a" * 64
HOST = "127.0.0.1:8765"
ORIGIN = "http://" + HOST


def canonical_wav(duration_ms=250):
    sample_count = 16_000 * duration_ms // 1_000
    data = struct.pack("<h", 0) * sample_count
    return (
        b"RIFF"
        + struct.pack("<I", 36 + len(data))
        + b"WAVE"
        + b"fmt "
        + struct.pack(
            "<IHHIIHH",
            16,
            1,
            1,
            16_000,
            32_000,
            2,
            16,
        )
        + b"data"
        + struct.pack("<I", len(data))
        + data
    )


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
            "experiments": [
                {
                    "schema": "dashboard-experiment/v1",
                    "experiment_id": "EXP-F1-IR-DYN-002",
                    "title_key": "experiments.curated.dynamic_ir.title",
                    "summary_key": (
                        "experiments.curated.dynamic_ir.summary"
                    ),
                    "status": "verified",
                    "component_ids": ["ev3rstorm-01.ev3-main"],
                }
            ],
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

    def spatial_map(self):
        self.calls.append(("spatial_map",))
        return {
            "schema": "robot-spatial-map/v1",
            "status": "unavailable",
            "read_only": True,
            "cells": [],
            "sensor_rays": [],
            "object_hypotheses": [],
        }

    def shared_spatial_map(self):
        self.calls.append(("shared_spatial_map",))
        return {
            "schema": "robot-spatial-map/v2",
            "status": "available",
            "read_only": True,
            "frame_id": "lab-world",
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
        response_locale,
    ):
        self.calls.append(
            (
                "submit_turn",
                conversation_id,
                client_request_id,
                expected_conversation_version,
                content,
                mode,
                response_locale,
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

    def submit_transcription(
        self,
        request_id,
        language_hint,
        wav_bytes,
    ):
        self.calls.append(
            (
                "submit_transcription",
                request_id,
                language_hint,
                wav_bytes,
            )
        )
        return {
            "schema": "speech-transcription/v1",
            "transcription_id": "stt-1",
            "request_id": request_id,
            "status": "queued",
        }

    def get_transcription(self, transcription_id):
        self.calls.append(("get_transcription", transcription_id))
        if transcription_id == "missing":
            raise ServiceFailure(
                404,
                "stt_not_found",
                "Speech transcription was not found",
            )
        if transcription_id == "expired":
            raise ServiceFailure(
                410,
                "stt_expired",
                "Speech transcription result has expired",
            )
        return {
            "schema": "speech-transcription/v1",
            "transcription_id": transcription_id,
            "request_id": "voice-1",
            "status": "completed",
            "text": "Vinka med höger arm.",
        }

    def cancel_transcription(self, transcription_id):
        self.calls.append(("cancel_transcription", transcription_id))
        if transcription_id == "missing":
            raise ServiceFailure(
                404,
                "stt_not_found",
                "Speech transcription was not found",
            )
        if transcription_id == "expired":
            raise ServiceFailure(
                410,
                "stt_expired",
                "Speech transcription result has expired",
            )
        return {
            "schema": "speech-transcription/v1",
            "transcription_id": transcription_id,
            "request_id": "voice-1",
            "status": "cancelled",
            "error_code": "stt_cancelled",
            "audio": {"duration_ms": 250, "retained": False},
        }

    def cancel_transcription_request(self, request_id):
        self.calls.append(("cancel_transcription_request", request_id))
        return {
            "schema": "speech-transcription-cancellation/v1",
            "request_id": request_id,
            "transcription_id": None,
            "status": "cancelled",
        }

    def probe_speech_transcriber(self):
        self.calls.append(("probe_speech_transcriber",))
        return {
            "schema": "speech-to-text-runtime/v1",
            "state": "online",
            "provider_id": "whisper.cpp",
            "model_id": "ggml-small",
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
    (root / "i18n.js").write_text(
        '"use strict";',
        encoding="utf-8",
    )
    (root / "blast_map_semantics.js").write_text(
        '"use strict";\n/* BLAST map semantics fixture */',
        encoding="utf-8",
    )
    (root / "dashboard_logic.js").write_text(
        '"use strict";\n/* dashboard logic fixture */',
        encoding="utf-8",
    )
    (root / "controller_panel.js").write_text(
        '"use strict";\n/* controller panel fixture */',
        encoding="utf-8",
    )
    (root / "spatial_map_presenter.js").write_text(
        '"use strict";\n/* spatial map presenter fixture */',
        encoding="utf-8",
    )
    (root / "robot_mission_panel.js").write_text(
        '"use strict";\n/* robot mission panel fixture */',
        encoding="utf-8",
    )
    (root / "robot_control.js").write_text(
        '"use strict";\n/* robot control fixture */',
        encoding="utf-8",
    )
    (root / "speech_input_logic.js").write_text(
        '"use strict";\n/* speech input logic fixture */',
        encoding="utf-8",
    )
    (root / "microphone_input.js").write_text(
        '"use strict";\n/* microphone input fixture */',
        encoding="utf-8",
    )
    (root / "pcm_capture_worklet.js").write_text(
        '"use strict";\n/* pcm capture worklet fixture */',
        encoding="utf-8",
    )
    (root / "app.js").write_text(
        '"use strict";',
        encoding="utf-8",
    )
    (root / "robot-llm-mascot.png").write_bytes(
        b"\x89PNG\r\n\x1a\nfixture",
    )
    (root / "robot-llm-head.png").write_bytes(
        b"\x89PNG\r\n\x1a\nhead-fixture",
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
        self.assertEqual(
            self.router.access_path,
            "/live/{}/".format(TOKEN),
        )
        self.assertEqual(self.router.session_path, self.router.access_path)
        response = self.router.handle(
            "GET",
            self.router.access_path,
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
        self.assertIn("worker-src 'self'", headers["Content-Security-Policy"])
        self.assertEqual(
            headers["Permissions-Policy"],
            "microphone=(self), camera=()",
        )
        self.assertEqual(headers["Cache-Control"], "no-store, max-age=0")
        self.assertEqual(headers["X-Frame-Options"], "DENY")

    def test_legacy_session_urls_redirect_only_with_the_right_access_key(self):
        legacy_index = self.router.handle(
            "GET",
            "/session/{}/".format(TOKEN),
            self.headers(authenticated=False),
        )
        legacy_asset = self.router.handle(
            "GET",
            "/session/{}/assets/app.js".format(TOKEN),
            self.headers(authenticated=False),
        )
        wrong_live_key = self.router.handle(
            "GET",
            "/live/{}/".format("b" * 64),
            self.headers(authenticated=False),
        )
        wrong_legacy_key = self.router.handle(
            "GET",
            "/session/{}/".format("b" * 64),
            self.headers(authenticated=False),
        )

        self.assertEqual(legacy_index.status, 308)
        self.assertEqual(
            dict(legacy_index.headers)["Location"],
            self.router.access_path,
        )
        self.assertEqual(legacy_asset.status, 308)
        self.assertEqual(
            dict(legacy_asset.headers)["Location"],
            self.router.access_path + "assets/app.js",
        )
        self.assertEqual(wrong_live_key.status, 403)
        self.assertEqual(wrong_legacy_key.status, 403)
        self.assertEqual(
            self.decoded(wrong_live_key)["error"]["code"],
            "session_token_rejected",
        )

    def test_root_redirects_to_the_current_live_console_without_caching(self):
        response = self.router.handle(
            "GET",
            "/",
            self.headers(authenticated=False),
        )
        query = self.router.handle(
            "GET",
            "/?unexpected=true",
            self.headers(authenticated=False),
        )

        self.assertEqual(response.status, 307)
        headers = dict(response.headers)
        self.assertEqual(headers["Location"], self.router.access_path)
        self.assertEqual(headers["Cache-Control"], "no-store, max-age=0")
        self.assertEqual(response.body, b"")
        self.assertEqual(query.status, 404)

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
        self.assertEqual(
            self.decoded(bootstrap)["experiments"],
            [
                {
                    "schema": "dashboard-experiment/v1",
                    "experiment_id": "EXP-F1-IR-DYN-002",
                    "title_key": "experiments.curated.dynamic_ir.title",
                    "summary_key": (
                        "experiments.curated.dynamic_ir.summary"
                    ),
                    "status": "verified",
                    "component_ids": ["ev3rstorm-01.ev3-main"],
                }
            ],
        )
        self.assertEqual(registry.status, 200)
        self.assertEqual(
            self.service.calls,
            [("bootstrap",), ("registry",)],
        )

    def test_spatial_map_is_authenticated_read_only_and_query_free(self):
        response = self.router.handle(
            "GET",
            "/api/v1/map",
            self.headers(),
        )

        self.assertEqual(response.status, 200)
        snapshot = self.decoded(response)["map"]
        self.assertEqual(snapshot["schema"], "robot-spatial-map/v1")
        self.assertIs(snapshot["read_only"], True)
        self.assertEqual(snapshot["cells"], [])
        self.assertEqual(self.service.calls, [("spatial_map",)])

        unauthenticated = self.router.handle(
            "GET",
            "/api/v1/map",
            self.headers(authenticated=False),
        )
        queried = self.router.handle(
            "GET",
            "/api/v1/map?robot_id=ev3rstorm-01",
            self.headers(),
        )
        mutation = self.router.handle(
            "POST",
            "/api/v1/map",
            self.headers(mutation=True),
            b"{}",
        )

        self.assertEqual(unauthenticated.status, 403)
        self.assertEqual(queried.status, 400)
        self.assertEqual(mutation.status, 404)
        self.assertEqual(self.service.calls, [("spatial_map",)])

    def test_spatial_map_limit_applies_to_the_final_http_body(self):
        self.service.spatial_map = lambda: {
            "schema": "robot-spatial-map/v1",
            "read_only": True,
            "padding": "x" * MAX_SPATIAL_MAP_RESPONSE_BYTES,
        }

        response = self.router.handle(
            "GET",
            "/api/v1/map",
            self.headers(),
        )

        self.assertEqual(response.status, 503)
        self.assertLessEqual(len(response.body), MAX_SPATIAL_MAP_RESPONSE_BYTES)
        self.assertEqual(
            self.decoded(response)["error"]["code"],
            "spatial_map_unavailable",
        )

    def test_shared_map_is_authenticated_query_free_and_isolated(self):
        shared = self.router.handle(
            "GET",
            "/api/v1/shared-map",
            self.headers(),
        )
        local = self.router.handle(
            "GET",
            "/api/v1/map",
            self.headers(),
        )

        self.assertEqual(shared.status, 200)
        self.assertEqual(
            self.decoded(shared)["map"]["schema"],
            "robot-spatial-map/v2",
        )
        self.assertEqual(local.status, 200)
        self.assertEqual(
            self.decoded(local)["map"]["schema"],
            "robot-spatial-map/v1",
        )
        self.assertEqual(
            self.service.calls,
            [("shared_spatial_map",), ("spatial_map",)],
        )

        unauthenticated = self.router.handle(
            "GET",
            "/api/v1/shared-map",
            self.headers(authenticated=False),
        )
        queried = self.router.handle(
            "GET",
            "/api/v1/shared-map?robot_id=blast-01",
            self.headers(),
        )
        mutation = self.router.handle(
            "POST",
            "/api/v1/shared-map",
            self.headers(mutation=True),
            b"{}",
        )

        self.assertEqual(unauthenticated.status, 403)
        self.assertEqual(queried.status, 400)
        self.assertEqual(mutation.status, 404)
        self.assertEqual(
            self.service.calls,
            [("shared_spatial_map",), ("spatial_map",)],
        )

    def test_shared_map_limit_applies_to_the_final_http_body(self):
        self.service.shared_spatial_map = lambda: {
            "schema": "robot-spatial-map/v2",
            "read_only": True,
            "padding": "x" * MAX_SPATIAL_MAP_RESPONSE_BYTES,
        }

        response = self.router.handle(
            "GET",
            "/api/v1/shared-map",
            self.headers(),
        )

        self.assertEqual(response.status, 503)
        self.assertLessEqual(len(response.body), MAX_SPATIAL_MAP_RESPONSE_BYTES)
        self.assertEqual(
            self.decoded(response)["error"]["code"],
            "spatial_map_unavailable",
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
        old_i18n_asset_path = self.router.handle(
            "GET",
            "/assets/i18n.js",
            self.headers(authenticated=False),
        )
        old_logic_asset_path = self.router.handle(
            "GET",
            "/assets/dashboard_logic.js",
            self.headers(authenticated=False),
        )
        old_presenter_asset_path = self.router.handle(
            "GET",
            "/assets/spatial_map_presenter.js",
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
        i18n_asset = self.router.handle(
            "GET",
            self.router.session_path + "assets/i18n.js",
            self.headers(authenticated=False),
        )
        logic_asset = self.router.handle(
            "GET",
            self.router.session_path + "assets/dashboard_logic.js",
            self.headers(authenticated=False),
        )
        controller_panel_asset = self.router.handle(
            "GET",
            self.router.session_path + "assets/controller_panel.js",
            self.headers(authenticated=False),
        )
        presenter_asset = self.router.handle(
            "GET",
            self.router.session_path
            + "assets/spatial_map_presenter.js",
            self.headers(authenticated=False),
        )
        mission_panel_asset = self.router.handle(
            "GET",
            self.router.session_path
            + "assets/robot_mission_panel.js",
            self.headers(authenticated=False),
        )
        speech_logic_asset = self.router.handle(
            "GET",
            self.router.session_path
            + "assets/speech_input_logic.js",
            self.headers(authenticated=False),
        )
        microphone_asset = self.router.handle(
            "GET",
            self.router.session_path
            + "assets/microphone_input.js",
            self.headers(authenticated=False),
        )
        worklet_asset = self.router.handle(
            "GET",
            self.router.session_path
            + "assets/pcm_capture_worklet.js",
            self.headers(authenticated=False),
        )
        mascot_asset = self.router.handle(
            "GET",
            self.router.session_path
            + "assets/robot-llm-mascot.png",
            self.headers(authenticated=False),
        )
        head_asset = self.router.handle(
            "GET",
            self.router.session_path
            + "assets/robot-llm-head.png",
            self.headers(authenticated=False),
        )

        self.assertEqual(unauthenticated_api.status, 403)
        self.assertEqual(wrong_api.status, 403)
        self.assertEqual(public_root.status, 307)
        self.assertEqual(
            dict(public_root.headers)["Location"],
            self.router.access_path,
        )
        self.assertEqual(old_asset_path.status, 404)
        self.assertEqual(old_i18n_asset_path.status, 404)
        self.assertEqual(old_logic_asset_path.status, 404)
        self.assertEqual(old_presenter_asset_path.status, 404)
        self.assertEqual(wrong_session.status, 403)
        self.assertEqual(session_asset.status, 200)
        self.assertEqual(
            dict(session_asset.headers)["Content-Type"],
            "text/javascript; charset=utf-8",
        )
        self.assertEqual(i18n_asset.status, 200)
        self.assertEqual(
            dict(i18n_asset.headers)["Content-Type"],
            "text/javascript; charset=utf-8",
        )
        self.assertEqual(
            dict(i18n_asset.headers)["X-Content-Type-Options"],
            "nosniff",
        )
        self.assertEqual(i18n_asset.body, b'"use strict";')
        self.assertEqual(logic_asset.status, 200)
        self.assertEqual(
            dict(logic_asset.headers)["Content-Type"],
            "text/javascript; charset=utf-8",
        )
        self.assertEqual(
            dict(logic_asset.headers)["X-Content-Type-Options"],
            "nosniff",
        )
        self.assertEqual(
            logic_asset.body,
            b'"use strict";\n/* dashboard logic fixture */',
        )
        self.assertEqual(presenter_asset.status, 200)
        self.assertEqual(
            dict(presenter_asset.headers)["Content-Type"],
            "text/javascript; charset=utf-8",
        )
        self.assertEqual(
            presenter_asset.body,
            b'"use strict";\n/* spatial map presenter fixture */',
        )
        self.assertEqual(mission_panel_asset.status, 200)
        self.assertEqual(controller_panel_asset.status, 200)
        self.assertEqual(
            mission_panel_asset.body,
            b'"use strict";\n/* robot mission panel fixture */',
        )
        self.assertEqual(speech_logic_asset.status, 200)
        self.assertEqual(microphone_asset.status, 200)
        self.assertEqual(worklet_asset.status, 200)
        self.assertEqual(
            dict(worklet_asset.headers)["Content-Type"],
            "text/javascript; charset=utf-8",
        )
        static_routes = tuple(DashboardRouter._STATIC_ROUTES)
        self.assertLess(
            static_routes.index("assets/i18n.js"),
            static_routes.index("assets/blast_map_semantics.js"),
        )
        self.assertLess(
            static_routes.index("assets/blast_map_semantics.js"),
            static_routes.index("assets/dashboard_logic.js"),
        )
        self.assertLess(
            static_routes.index("assets/dashboard_logic.js"),
            static_routes.index("assets/controller_panel.js"),
        )
        self.assertLess(
            static_routes.index("assets/controller_panel.js"),
            static_routes.index("assets/spatial_map_presenter.js"),
        )
        self.assertLess(
            static_routes.index("assets/spatial_map_presenter.js"),
            static_routes.index("assets/robot_mission_panel.js"),
        )
        self.assertLess(
            static_routes.index("assets/robot_mission_panel.js"),
            static_routes.index("assets/robot_control.js"),
        )
        self.assertLess(
            static_routes.index("assets/robot_control.js"),
            static_routes.index("assets/speech_input_logic.js"),
        )
        self.assertLess(
            static_routes.index("assets/speech_input_logic.js"),
            static_routes.index("assets/microphone_input.js"),
        )
        self.assertLess(
            static_routes.index("assets/microphone_input.js"),
            static_routes.index("assets/pcm_capture_worklet.js"),
        )
        self.assertLess(
            static_routes.index("assets/pcm_capture_worklet.js"),
            static_routes.index("assets/app.js"),
        )
        self.assertEqual(mascot_asset.status, 200)
        self.assertEqual(
            dict(mascot_asset.headers)["Content-Type"],
            "image/png",
        )
        self.assertEqual(
            mascot_asset.body,
            b"\x89PNG\r\n\x1a\nfixture",
        )
        self.assertEqual(head_asset.status, 200)
        self.assertEqual(
            dict(head_asset.headers)["Content-Type"],
            "image/png",
        )
        self.assertEqual(
            head_asset.body,
            b"\x89PNG\r\n\x1a\nhead-fixture",
        )
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
                    "response_locale": "sv",
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
                "sv",
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

    def test_speech_audio_route_is_bounded_and_provider_neutral(self):
        wav = canonical_wav()
        headers = self.headers(
            mutation=True,
            **{
                "Content-Type": "audio/wav",
                STT_REQUEST_ID_HEADER: "voice-1",
                STT_LANGUAGE_HEADER: "sv",
            },
        )

        submitted = self.router.handle(
            "POST",
            STT_TRANSCRIPTIONS_PATH,
            headers,
            wav,
        )
        fetched = self.router.handle(
            "GET",
            STT_TRANSCRIPTIONS_PATH + "/stt-1",
            self.headers(),
        )

        self.assertEqual(submitted.status, 202)
        self.assertEqual(
            self.decoded(submitted)["transcription"]["status"],
            "queued",
        )
        self.assertEqual(fetched.status, 200)
        self.assertEqual(
            self.decoded(fetched)["transcription"]["text"],
            "Vinka med höger arm.",
        )
        self.assertEqual(
            self.service.calls[-2],
            ("submit_transcription", "voice-1", "sv", wav),
        )
        self.assertEqual(
            self.service.calls[-1],
            ("get_transcription", "stt-1"),
        )

    def test_authenticated_delete_cancels_exact_transcription(self):
        cancelled = self.router.handle(
            "DELETE",
            STT_TRANSCRIPTIONS_PATH + "/stt-1",
            self.headers(),
        )

        self.assertEqual(cancelled.status, 200)
        self.assertEqual(
            self.decoded(cancelled)["transcription"]["status"],
            "cancelled",
        )
        self.assertEqual(
            self.service.calls[-1],
            ("cancel_transcription", "stt-1"),
        )

        missing = self.router.handle(
            "DELETE",
            STT_TRANSCRIPTIONS_PATH + "/missing",
            self.headers(),
        )
        expired = self.router.handle(
            "DELETE",
            STT_TRANSCRIPTIONS_PATH + "/expired",
            self.headers(),
        )
        expired_get = self.router.handle(
            "GET",
            STT_TRANSCRIPTIONS_PATH + "/expired",
            self.headers(),
        )
        self.assertEqual(missing.status, 404)
        self.assertEqual(
            self.decoded(missing)["error"]["code"],
            "stt_not_found",
        )
        self.assertEqual(expired.status, 410)
        self.assertEqual(
            self.decoded(expired)["error"]["code"],
            "stt_expired",
        )
        self.assertEqual(expired_get.status, 410)
        self.assertEqual(
            self.decoded(expired_get)["error"]["code"],
            "stt_expired",
        )

    def test_speech_delete_rejects_bad_auth_query_and_body(self):
        path = STT_TRANSCRIPTIONS_PATH + "/stt-1"

        unauthorized = self.router.handle(
            "DELETE",
            path,
            self.headers(authenticated=False),
        )
        queried = self.router.handle(
            "DELETE",
            path + "?force=true",
            self.headers(),
        )
        with_body = self.router.handle(
            "DELETE",
            path,
            self.headers(),
            b"{}",
        )
        wrong_route = self.router.handle(
            "DELETE",
            "/api/v1/settings",
            self.headers(),
        )

        self.assertEqual(unauthorized.status, 403)
        self.assertEqual(queried.status, 400)
        self.assertEqual(with_body.status, 400)
        self.assertEqual(wrong_route.status, 405)
        self.assertNotIn(
            ("cancel_transcription", "stt-1"),
            self.service.calls,
        )

    def test_authenticated_delete_can_cancel_before_submit_returns(self):
        path = STT_REQUESTS_PATH + "/voice-race-1"
        cancelled = self.router.handle(
            "DELETE",
            path,
            self.headers(),
        )

        self.assertEqual(cancelled.status, 200)
        self.assertEqual(
            self.decoded(cancelled)["cancellation"]["status"],
            "cancelled",
        )
        self.assertEqual(
            self.service.calls[-1],
            ("cancel_transcription_request", "voice-race-1"),
        )

        for target, body, expected in (
            (path + "?force=true", b"", 400),
            (path, b"{}", 400),
        ):
            with self.subTest(target=target, body=body):
                response = self.router.handle(
                    "DELETE",
                    target,
                    self.headers(),
                    body,
                )
                self.assertEqual(response.status, expected)

    def test_speech_route_requires_exact_mime_headers_and_query(self):
        wav = canonical_wav()
        base_headers = {
            STT_REQUEST_ID_HEADER: "voice-1",
            STT_LANGUAGE_HEADER: "auto",
        }
        wrong_mime = self.router.handle(
            "POST",
            STT_TRANSCRIPTIONS_PATH,
            self.headers(mutation=True, **base_headers),
            wav,
        )
        missing_headers = self.router.handle(
            "POST",
            STT_TRANSCRIPTIONS_PATH,
            self.headers(
                mutation=True,
                **{"Content-Type": "audio/wav"},
            ),
            wav,
        )
        queried = self.router.handle(
            "POST",
            STT_TRANSCRIPTIONS_PATH + "?language=sv",
            self.headers(
                mutation=True,
                **{
                    "Content-Type": "audio/wav",
                    **base_headers,
                },
            ),
            wav,
        )

        self.assertEqual(wrong_mime.status, 415)
        self.assertEqual(missing_headers.status, 400)
        self.assertEqual(queried.status, 400)
        self.assertEqual(self.service.calls, [])

    def test_only_exact_speech_route_receives_the_audio_body_limit(self):
        self.assertEqual(
            self.router.request_body_limit(
                "POST",
                STT_TRANSCRIPTIONS_PATH,
            ),
            MAX_STT_AUDIO_BYTES,
        )
        for method, target in (
            ("GET", STT_TRANSCRIPTIONS_PATH),
            ("POST", STT_TRANSCRIPTIONS_PATH + "?debug=1"),
            ("POST", "/api/v1/conversations"),
            ("PUT", "/api/v1/settings"),
        ):
            with self.subTest(method=method, target=target):
                self.assertEqual(
                    self.router.request_body_limit(method, target),
                    MAX_REQUEST_BYTES,
                )

        too_large = self.router.handle(
            "POST",
            STT_TRANSCRIPTIONS_PATH,
            self.headers(
                mutation=True,
                **{
                    "Content-Type": "audio/wav",
                    STT_REQUEST_ID_HEADER: "voice-large",
                    STT_LANGUAGE_HEADER: "auto",
                },
            ),
            b"\x00" * (MAX_STT_AUDIO_BYTES + 1),
        )
        self.assertEqual(too_large.status, 413)

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

    def test_speech_probe_is_explicit_and_accepts_no_fields(self):
        response = self.router.handle(
            "POST",
            "/api/v1/runtime/stt/probe",
            self.headers(mutation=True),
            b"{}",
        )
        extra = self.router.handle(
            "POST",
            "/api/v1/runtime/stt/probe",
            self.headers(mutation=True),
            b'{"url":"http://example.com"}',
        )

        self.assertEqual(response.status, 200)
        self.assertEqual(extra.status, 400)
        self.assertEqual(
            self.decoded(response)["speech_to_text"]["state"],
            "online",
        )
        self.assertEqual(
            self.service.calls[-1],
            ("probe_speech_transcriber",),
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
