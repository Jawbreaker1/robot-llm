import json
import threading
import time
import unittest

from robot_agent.lm_studio_robot_input import (
    PHYSICAL_TASK,
    RobotInputDecision,
)
from robot_agent.robot_control_contract import RobotControlTarget
from robot_agent.robot_control_http import (
    RobotControlHTTPDirectoryRouter,
    RobotControlHTTPError,
    RobotControlHTTPRouter,
)
from robot_agent.robot_control_service import (
    MAX_ROBOT_PAGE_RESPONSE_BYTES,
    RobotControlServiceError,
    RobotControlService,
    RobotEpisodeGate,
)
from robot_agent.robot_input_service import RobotInputService


class FakeRobotControlService:
    def __init__(self, robot_id=None, display_name=None):
        self.calls = []
        self.robot_id = robot_id
        self.display_name = display_name

    def status(self):
        self.calls.append(("status",))
        return {
            "state": "IDLE",
            "target": (
                None
                if self.robot_id is None
                else {
                    "robot_id": self.robot_id,
                    "display_name": self.display_name,
                }
            ),
        }

    def settings(self):
        self.calls.append(("settings",))
        return {"revision": 3, "model": "gemma"}

    def update_settings(self, expected_revision, changes):
        self.calls.append(
            ("update_settings", expected_revision, changes)
        )
        if expected_revision == 99:
            raise RobotControlServiceError(
                409,
                "robot_settings_revision_conflict",
                "Revision conflict",
            )
        return {"revision": expected_revision + 1, **changes}

    def start(
        self,
        goal,
        locale,
        client_request_id,
        expected_revision,
    ):
        self.calls.append(
            (
                "start",
                goal,
                locale,
                client_request_id,
                expected_revision,
            )
        )
        return {
            "accepted_episode_id": "episode-1",
            "idempotent": False,
            "control": {"state": "STARTING"},
        }

    def stop(self):
        self.calls.append(("stop",))
        return {"state": "STOPPING"}

    def emergency_stop(self):
        self.calls.append(("emergency_stop",))
        return {"state": "STOPPING"}

    def events(self, after_sequence, limit):
        self.calls.append(("events", after_sequence, limit))
        return {
            "events": [],
            "next_after_sequence": after_sequence,
        }

    def snapshots(self, after_sequence, limit):
        self.calls.append(("snapshots", after_sequence, limit))
        return {
            "snapshots": [],
            "next_after_sequence": after_sequence,
        }


class FakeRobotInputService:
    def __init__(self, intent="READ_ONLY_TASK"):
        self.intent = intent
        self.calls = []

    def dispatch(self, text, locale, client_request_id, expected_revision):
        self.calls.append(
            (text, locale, client_request_id, expected_revision)
        )
        return {
            "schema": "robot-input-turn/v1",
            "request_id": client_request_id,
            "intent": self.intent,
            "confidence_milli": 900,
            "answer_text": (
                None
                if self.intent == "PHYSICAL_TASK"
                else "Status answer"
            ),
            "episode": (
                {"accepted_episode_id": "episode-2"}
                if self.intent == "PHYSICAL_TASK"
                else None
            ),
            "control": (
                {"state": "STOPPING", "sequence": 8}
                if self.intent == "STOP_TASK"
                else None
            ),
            "speech_queued": self.intent != "PHYSICAL_TASK",
            "facts_captured_at_unix_ms": 1_000,
        }


class FakeControllerService:
    def __init__(self):
        self.commands = []
        self.connection_actions = []

    def command(self, command):
        self.commands.append(command)
        return {
            "schema": "controller-command-result/v1",
            "controller_id": "blast-01.hub",
            "command": command,
            "accepted": True,
            "completed": True,
        }

    def _connection(self, action):
        self.connection_actions.append(action)
        return {
            "schema": "controller-runtime-observation/v1",
            "robot_id": "blast-01",
            "controller_id": "blast-01.hub",
            "state": "stopped" if action == "disconnect" else "connecting",
        }

    def connect(self):
        return self._connection("connect")

    def disconnect(self):
        return self._connection("disconnect")

    def retry(self):
        return self._connection("retry")


def encoded(value):
    return json.dumps(
        value,
        separators=(",", ":"),
    ).encode("utf-8")


class RobotControlHTTPRouterTests(unittest.TestCase):
    def setUp(self):
        self.service = FakeRobotControlService()
        self.router = RobotControlHTTPRouter(self.service)

    def test_status_and_settings_routes(self):
        status = self.router.handle(
            "GET",
            "/api/v1/robot/status",
            "",
            b"",
        )
        settings = self.router.handle(
            "GET",
            "/api/v1/robot/settings",
            "",
            b"",
        )
        self.assertEqual(status.status, 200)
        self.assertEqual(status.body["control"]["state"], "IDLE")
        self.assertEqual(settings.body["settings"]["revision"], 3)

    def test_start_accepts_only_the_four_contract_fields(self):
        response = self.router.handle(
            "POST",
            "/api/v1/robot/episodes",
            "",
            encoded(
                {
                    "goal": "Navigate around the room.",
                    "locale": "en",
                    "client_request_id": "ui-1",
                    "expected_revision": 3,
                }
            ),
        )
        self.assertEqual(response.status, 202)
        self.assertEqual(
            self.service.calls[-1],
            (
                "start",
                "Navigate around the room.",
                "en",
                "ui-1",
                3,
            ),
        )
        with self.assertRaises(RobotControlHTTPError) as raised:
            self.router.handle(
                "POST",
                "/api/v1/robot/episodes",
                "",
                encoded(
                    {
                        "goal": "Move.",
                        "locale": "en",
                        "client_request_id": "ui-2",
                        "expected_revision": 3,
                        "speed": "full",
                    }
                ),
            )
        self.assertEqual(
            raised.exception.code,
            "invalid_robot_request_fields",
        )

    def test_composite_turn_delegates_exact_input_contract(self):
        input_service = FakeRobotInputService()
        router = RobotControlHTTPRouter(self.service, input_service)

        response = router.handle(
            "POST",
            "/api/v1/robot/turns",
            "",
            encoded({
                "text": "Hur går det?",
                "locale": "sv",
                "client_request_id": "ui-turn-1",
                "expected_revision": 3,
            }),
        )

        self.assertEqual(response.status, 200)
        self.assertEqual(
            response.body["turn"]["intent"],
            "READ_ONLY_TASK",
        )
        self.assertEqual(
            input_service.calls,
            [("Hur går det?", "sv", "ui-turn-1", 3)],
        )

        physical = FakeRobotInputService("PHYSICAL_TASK")
        response = RobotControlHTTPRouter(
            self.service,
            physical,
        ).handle(
            "POST",
            "/api/v1/robot/turns",
            "",
            encoded({
                "text": "Sväng höger",
                "locale": "sv",
                "client_request_id": "ui-turn-2",
                "expected_revision": 3,
            }),
        )
        self.assertEqual(response.status, 202)
        self.assertEqual(
            response.body["turn"]["episode"][
                "accepted_episode_id"
            ],
            "episode-2",
        )

    def test_composite_turn_fails_closed_when_service_is_missing(self):
        with self.assertRaises(RobotControlHTTPError) as raised:
            self.router.handle(
                "POST",
                "/api/v1/robot/turns",
                "",
                encoded({
                    "text": "Hej",
                    "locale": "sv",
                    "client_request_id": "ui-turn-3",
                    "expected_revision": 3,
                }),
            )
        self.assertEqual(raised.exception.code, "robot_input_disabled")

    def test_stop_and_emergency_stop_require_empty_json_objects(self):
        stop = self.router.handle(
            "POST",
            "/api/v1/robot/stop",
            "",
            b"{}",
        )
        emergency = self.router.handle(
            "POST",
            "/api/v1/robot/emergency-stop",
            "",
            b"{}",
        )
        self.assertEqual(stop.body["control"]["state"], "STOPPING")
        self.assertEqual(
            emergency.body["control"]["state"],
            "STOPPING",
        )
        with self.assertRaises(RobotControlHTTPError):
            self.router.handle(
                "POST",
                "/api/v1/robot/stop",
                "",
                b"",
            )

    def test_blast_route_accepts_only_fixed_controller_commands(self):
        controller = FakeControllerService()
        router = RobotControlHTTPRouter(
            self.service,
            controller_services={"blast-01.hub": controller},
        )

        response = router.handle(
            "POST",
            "/api/v1/controllers/blast-01.hub/commands",
            "",
            encoded({"command": "claw_close"}),
        )

        self.assertEqual(response.status, 200)
        self.assertEqual(controller.commands, ["claw_close"])
        self.assertTrue(response.body["result"]["completed"])
        for body in (
            {"command": "set_speed"},
            {"command": []},
            {"command": {}},
            {"command": "drive_forward", "speed": 999},
        ):
            with self.subTest(body=body):
                with self.assertRaises(RobotControlHTTPError) as raised:
                    router.handle(
                        "POST",
                        "/api/v1/controllers/blast-01.hub/commands",
                        "",
                        encoded(body),
                    )
                self.assertEqual(raised.exception.status, 400)

    def test_blast_route_fails_closed_when_unavailable_or_unknown(self):
        with self.assertRaises(RobotControlHTTPError) as unavailable:
            self.router.handle(
                "POST",
                "/api/v1/controllers/blast-01.hub/commands",
                "",
                encoded({"command": "stop"}),
            )
        self.assertEqual(unavailable.exception.code, "controller_unavailable")

        with self.assertRaises(RobotControlHTTPError) as unknown:
            self.router.handle(
                "POST",
                "/api/v1/controllers/other/commands",
                "",
                encoded({"command": "stop"}),
            )
        self.assertEqual(unknown.exception.code, "controller_route_not_found")

    def test_blast_connection_route_accepts_only_fixed_actions(self):
        controller = FakeControllerService()
        router = RobotControlHTTPRouter(
            self.service,
            controller_services={"blast-01.hub": controller},
        )

        for action in ("connect", "disconnect", "retry"):
            response = router.handle(
                "POST",
                "/api/v1/controllers/blast-01.hub/connection",
                "",
                encoded({"action": action}),
            )
            self.assertEqual(response.status, 200)
            self.assertEqual(
                response.body["connection"]["controller_id"],
                "blast-01.hub",
            )

        self.assertEqual(
            controller.connection_actions,
            ["connect", "disconnect", "retry"],
        )
        for body in (
            {"action": "restart"},
            {"action": []},
            {"action": "connect", "force": True},
        ):
            with self.subTest(body=body):
                with self.assertRaises(RobotControlHTTPError) as raised:
                    router.handle(
                        "POST",
                        "/api/v1/controllers/blast-01.hub/connection",
                        "",
                        encoded(body),
                    )
                self.assertEqual(raised.exception.status, 400)

    def test_ev3_reachability_route_accepts_only_an_empty_post(self):
        class Reachability:
            def __init__(self):
                self.calls = 0

            def check(self):
                self.calls += 1
                return {
                    "controller_id": "ev3rstorm-01.ev3-main",
                    "state": "configured",
                    "reachability": {"status": "passed"},
                }

        reachability = Reachability()
        router = RobotControlHTTPRouter(
            self.service,
            controller_services={
                "ev3rstorm-01.ev3-main": reachability,
            },
        )

        response = router.handle(
            "POST",
            "/api/v1/controllers/ev3rstorm-01.ev3-main/reachability",
            "",
            encoded({}),
        )

        self.assertEqual(response.status, 200)
        self.assertEqual(reachability.calls, 1)
        self.assertEqual(
            response.body["reachability"]["reachability"]["status"],
            "passed",
        )
        for method, query, body in (
            ("GET", "", encoded({})),
            ("POST", "force=true", encoded({})),
            ("POST", "", encoded({"action": "connect"})),
        ):
            with self.subTest(method=method, query=query, body=body):
                with self.assertRaises(RobotControlHTTPError):
                    router.handle(
                        method,
                        "/api/v1/controllers/"
                        "ev3rstorm-01.ev3-main/reachability",
                        query,
                        body,
                    )

    def test_ev3_reachability_route_is_bounded_when_busy_or_unavailable(self):
        with self.assertRaises(RobotControlHTTPError) as unavailable:
            self.router.handle(
                "POST",
                "/api/v1/controllers/ev3rstorm-01.ev3-main/reachability",
                "",
                encoded({}),
            )
        self.assertEqual(unavailable.exception.status, 503)

        class BusyReachability:
            def check(self):
                error = RuntimeError("private state")
                error.code = "controller_busy"
                raise error

        router = RobotControlHTTPRouter(
            self.service,
            controller_services={
                "ev3rstorm-01.ev3-main": BusyReachability(),
            },
        )
        with self.assertRaises(RobotControlHTTPError) as busy:
            router.handle(
                "POST",
                "/api/v1/controllers/ev3rstorm-01.ev3-main/reachability",
                "",
                encoded({}),
            )
        self.assertEqual(busy.exception.status, 409)
        self.assertEqual(busy.exception.code, "controller_busy")
        self.assertNotIn("private", str(busy.exception))

    def test_blast_controller_errors_have_bounded_http_statuses(self):
        for error_code in (
            "controller_busy",
            "controller_command_interrupted",
        ):
            with self.subTest(error_code=error_code):
                class FailingController(FakeControllerService):
                    def command(self, command):
                        error = RuntimeError("details stay private")
                        error.code = error_code
                        raise error

                router = RobotControlHTTPRouter(
                    self.service,
                    controller_services={
                        "blast-01.hub": FailingController(),
                    },
                )

                with self.assertRaises(RobotControlHTTPError) as raised:
                    router.handle(
                        "POST",
                        "/api/v1/controllers/blast-01.hub/commands",
                        "",
                        encoded({"command": "turn_left"}),
                    )

                self.assertEqual(raised.exception.status, 409)
                self.assertEqual(raised.exception.code, error_code)

    def test_settings_errors_preserve_typed_status_and_code(self):
        with self.assertRaises(RobotControlHTTPError) as raised:
            self.router.handle(
                "PUT",
                "/api/v1/robot/settings",
                "",
                encoded(
                    {
                        "expected_revision": 99,
                        "changes": {"model": "gemma"},
                    }
                ),
            )
        self.assertEqual(raised.exception.status, 409)
        self.assertEqual(
            raised.exception.code,
            "robot_settings_revision_conflict",
        )

    def test_event_and_snapshot_queries_are_bounded(self):
        self.router.handle(
            "GET",
            "/api/v1/robot/events",
            "after_sequence=7&limit=20",
            b"",
        )
        self.router.handle(
            "GET",
            "/api/v1/robot/snapshots",
            "after_sequence=8&limit=10",
            b"",
        )
        self.assertIn(("events", 7, 20), self.service.calls)
        self.assertIn(("snapshots", 8, 10), self.service.calls)
        with self.assertRaises(RobotControlHTTPError):
            self.router.handle(
                "GET",
                "/api/v1/robot/events",
                "limit=501",
                b"",
            )

    def test_history_page_has_a_defensive_http_byte_limit(self):
        self.service.events = lambda _after, _limit: {
            "events": [{
                "sequence": 1,
                "payload": "x" * MAX_ROBOT_PAGE_RESPONSE_BYTES,
            }],
            "next_after_sequence": 1,
        }

        with self.assertRaises(RobotControlHTTPError) as raised:
            self.router.handle(
                "GET",
                "/api/v1/robot/events",
                "after_sequence=0&limit=500",
                b"",
            )

        self.assertEqual(
            raised.exception.code,
            "robot_history_page_too_large",
        )

    def test_unknown_robot_route_fails_closed(self):
        self.assertTrue(
            self.router.handles("/api/v1/robot/status")
        )
        self.assertFalse(self.router.handles("/api/v1/settings"))
        self.assertTrue(
            self.router.handles(
                "/api/v1/controllers/blast-01.hub/commands"
            )
        )
        with self.assertRaises(RobotControlHTTPError) as raised:
            self.router.handle(
                "POST",
                "/api/v1/robot/run-anything",
                "",
                b"{}",
            )
        self.assertEqual(raised.exception.status, 404)


class RobotControlHTTPDirectoryRouterTests(unittest.TestCase):
    def setUp(self):
        self.blast = FakeRobotControlService("blast-01", "BLAST")
        self.ev3 = FakeRobotControlService(
            "ev3rstorm-01",
            "EV3RSTORM",
        )
        self.blast_input = FakeRobotInputService()
        self.ev3_input = FakeRobotInputService()
        self.blast_router = RobotControlHTTPRouter(
            self.blast,
            self.blast_input,
        )
        self.ev3_router = RobotControlHTTPRouter(
            self.ev3,
            self.ev3_input,
        )
        self.router = RobotControlHTTPDirectoryRouter(
            {
                "ev3rstorm-01": self.ev3_router,
                "blast-01": self.blast_router,
            },
            default_router=self.ev3_router,
            default_robot_id="ev3rstorm-01",
        )

    def test_directory_returns_sorted_full_control_snapshots(self):
        response = self.router.handle(
            "GET",
            "/api/v1/robots",
            "",
            b"",
        )

        self.assertEqual(response.status, 200)
        self.assertEqual(
            response.body["schema"],
            "robot-control-directory/v1",
        )
        self.assertEqual(
            [
                control["target"]["robot_id"]
                for control in response.body["controls"]
            ],
            ["blast-01", "ev3rstorm-01"],
        )

    def test_scoped_routes_delegate_to_only_the_selected_robot(self):
        response = self.router.handle(
            "POST",
            "/api/v1/robots/blast-01/turns",
            "",
            encoded({
                "text": "Hur går det?",
                "locale": "sv",
                "client_request_id": "blast-turn-1",
                "expected_revision": 3,
            }),
        )

        self.assertEqual(response.status, 200)
        self.assertEqual(
            self.blast_input.calls,
            [("Hur går det?", "sv", "blast-turn-1", 3)],
        )
        self.assertEqual(self.ev3_input.calls, [])

    def test_singular_routes_remain_aliases_for_the_default_robot(self):
        response = self.router.handle(
            "POST",
            "/api/v1/robot/stop",
            "",
            b"{}",
        )

        self.assertEqual(response.status, 200)
        self.assertIn(("stop",), self.ev3.calls)
        self.assertNotIn(("stop",), self.blast.calls)

    def test_unknown_robot_and_route_fail_closed(self):
        self.assertTrue(self.router.handles("/api/v1/robots"))
        self.assertTrue(
            self.router.handles("/api/v1/robots/blast-01/status")
        )
        with self.assertRaises(RobotControlHTTPError) as unknown:
            self.router.handle(
                "GET",
                "/api/v1/robots/unknown/status",
                "",
                b"",
            )
        self.assertEqual(unknown.exception.code, "robot_target_not_found")
        with self.assertRaises(RobotControlHTTPError) as route:
            self.router.handle(
                "GET",
                "/api/v1/robots/blast-01/run-anything",
                "",
                b"",
            )
        self.assertEqual(route.exception.code, "robot_route_not_found")

    def test_shared_gate_blocks_scoped_episode_and_physical_turn(self):
        class Runtime:
            def __init__(self):
                self.entered = threading.Event()
                self.release = threading.Event()

            def run(self, _context):
                self.entered.set()
                self.release.wait(1.0)
                return {"current_action": "stop"}

            def request_stop(self):
                self.release.set()

            def emergency_stop(self):
                self.release.set()

        class PhysicalModel:
            def interpret(self, _input, _facts):
                return RobotInputDecision(PHYSICAL_TASK, 900, None)

        gate = RobotEpisodeGate()
        ev3_runtime = Runtime()
        blast_runtime = Runtime()
        ev3 = RobotControlService(
            ev3_runtime,
            target=RobotControlTarget("ev3rstorm-01", "EV3RSTORM"),
            episode_gate=gate,
        )
        blast = RobotControlService(
            blast_runtime,
            target=RobotControlTarget("blast-01", "BLAST"),
            episode_gate=gate,
        )
        blast_input = RobotInputService(
            control_service=blast,
            model_factory=lambda _model: PhysicalModel(),
            clock_ms=lambda: 1,
        )
        router = RobotControlHTTPDirectoryRouter(
            {
                "ev3rstorm-01": RobotControlHTTPRouter(ev3),
                "blast-01": RobotControlHTTPRouter(
                    blast,
                    blast_input,
                ),
            },
            default_router=RobotControlHTTPRouter(ev3),
            default_robot_id="ev3rstorm-01",
        )
        try:
            started = router.handle(
                "POST",
                "/api/v1/robots/ev3rstorm-01/episodes",
                "",
                encoded({
                    "goal": "Explore",
                    "locale": "en",
                    "client_request_id": "ev3-start",
                    "expected_revision": 1,
                }),
            )
            self.assertEqual(started.status, 202)
            self.assertTrue(ev3_runtime.entered.wait(1.0))

            with self.assertRaises(RobotControlHTTPError) as episode_busy:
                router.handle(
                    "POST",
                    "/api/v1/robots/blast-01/episodes",
                    "",
                    encoded({
                        "goal": "Explore",
                        "locale": "en",
                        "client_request_id": "blast-episode",
                        "expected_revision": 1,
                    }),
                )
            self.assertEqual(episode_busy.exception.status, 409)
            with self.assertRaises(RobotControlHTTPError) as turn_busy:
                router.handle(
                    "POST",
                    "/api/v1/robots/blast-01/turns",
                    "",
                    encoded({
                        "text": "Kör framåt",
                        "locale": "sv",
                        "client_request_id": "blast-turn",
                        "expected_revision": 1,
                    }),
                )
            self.assertEqual(turn_busy.exception.status, 409)

            ev3_runtime.release.set()
            deadline = time.monotonic() + 1.0
            while ev3.status()["state"] != "IDLE":
                self.assertLess(time.monotonic(), deadline)
                time.sleep(0.005)
            accepted = router.handle(
                "POST",
                "/api/v1/robots/blast-01/episodes",
                "",
                encoded({
                    "goal": "Explore",
                    "locale": "en",
                    "client_request_id": "blast-after-terminal",
                    "expected_revision": 1,
                }),
            )
            self.assertEqual(accepted.status, 202)
        finally:
            blast_runtime.release.set()
            ev3.shutdown(0.2)
            blast.shutdown(0.2)

    def test_scoped_spatial_map_uses_only_selected_robot_provider(self):
        calls = []

        class MapProvider:
            def __init__(self, robot_id):
                self.robot_id = robot_id

            def snapshot(self):
                calls.append(self.robot_id)
                return {
                    "schema": "robot-spatial-map/v1",
                    "read_only": True,
                    "source_id": self.robot_id,
                }

        router = RobotControlHTTPDirectoryRouter(
            {
                "ev3rstorm-01": self.ev3_router,
                "blast-01": self.blast_router,
            },
            default_router=self.ev3_router,
            default_robot_id="ev3rstorm-01",
            spatial_map_providers={
                "ev3rstorm-01": MapProvider("ev3rstorm-01"),
                "blast-01": MapProvider("blast-01"),
            },
        )

        response = router.handle(
            "GET",
            "/api/v1/robots/blast-01/spatial-map",
            "",
            b"",
        )

        self.assertEqual(response.status, 200)
        self.assertEqual(set(response.body), {"map"})
        self.assertEqual(response.body["map"]["source_id"], "blast-01")
        self.assertEqual(calls, ["blast-01"])

    def test_scoped_spatial_map_fails_safely(self):
        missing = self.router
        with self.assertRaises(RobotControlHTTPError) as unavailable:
            missing.handle(
                "GET",
                "/api/v1/robots/blast-01/spatial-map",
                "",
                b"",
            )
        self.assertEqual(unavailable.exception.status, 503)
        self.assertEqual(
            unavailable.exception.code,
            "spatial_map_unavailable",
        )

        invalid = RobotControlHTTPDirectoryRouter(
            {"blast-01": self.blast_router},
            default_router=self.blast_router,
            default_robot_id="blast-01",
            spatial_map_providers={
                "blast-01": lambda: {
                    "schema": "wrong-map/v1",
                    "read_only": True,
                },
            },
        )
        with self.assertRaises(RobotControlHTTPError) as malformed:
            invalid.handle(
                "GET",
                "/api/v1/robots/blast-01/spatial-map",
                "",
                b"",
            )
        self.assertEqual(malformed.exception.status, 503)
        self.assertEqual(malformed.exception.code, "spatial_map_unavailable")

        with self.assertRaises(RobotControlHTTPError) as unknown:
            missing.handle(
                "GET",
                "/api/v1/robots/unknown/spatial-map",
                "",
                b"",
            )
        self.assertEqual(unknown.exception.status, 404)
        self.assertEqual(unknown.exception.code, "robot_target_not_found")

    def test_scoped_spatial_map_is_get_only_and_query_free(self):
        router = RobotControlHTTPDirectoryRouter(
            {"blast-01": self.blast_router},
            default_router=self.blast_router,
            spatial_map_providers={
                "blast-01": lambda: {
                    "schema": "robot-spatial-map/v1",
                    "read_only": True,
                },
            },
        )

        with self.assertRaises(RobotControlHTTPError) as mutation:
            router.handle(
                "POST",
                "/api/v1/robots/blast-01/spatial-map",
                "",
                b"{}",
            )
        self.assertEqual(mutation.exception.status, 404)
        with self.assertRaises(RobotControlHTTPError) as queried:
            router.handle(
                "GET",
                "/api/v1/robots/blast-01/spatial-map",
                "view=shared",
                b"",
            )
        self.assertEqual(queried.exception.status, 400)
        self.assertEqual(queried.exception.code, "invalid_robot_query")

    def test_spatial_map_provider_ids_must_be_registered(self):
        with self.assertRaisesRegex(ValueError, "directory is invalid"):
            RobotControlHTTPDirectoryRouter(
                {"blast-01": self.blast_router},
                default_router=self.blast_router,
                spatial_map_providers={"unknown": lambda: {}},
            )


if __name__ == "__main__":
    unittest.main()
