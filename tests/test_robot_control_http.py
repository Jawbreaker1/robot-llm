import json
import unittest

from robot_agent.robot_control_http import (
    RobotControlHTTPError,
    RobotControlHTTPRouter,
)
from robot_agent.robot_control_service import (
    MAX_ROBOT_PAGE_RESPONSE_BYTES,
    RobotControlServiceError,
)


class FakeRobotControlService:
    def __init__(self):
        self.calls = []

    def status(self):
        self.calls.append(("status",))
        return {"state": "IDLE"}

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
        with self.assertRaises(RobotControlHTTPError) as raised:
            self.router.handle(
                "POST",
                "/api/v1/robot/run-anything",
                "",
                b"{}",
            )
        self.assertEqual(raised.exception.status, 404)


if __name__ == "__main__":
    unittest.main()
