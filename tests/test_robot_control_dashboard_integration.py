import json
import unittest

from robot_agent.dashboard_http import DashboardRouter
from robot_agent.robot_control_http import RobotControlHTTPRouter

from tests.test_dashboard_http import FakeDashboardService
from tests.test_robot_control_http import (
    FakeRobotControlService,
    FakeRobotInputService,
)


TOKEN = "b" * 64
HOST = "127.0.0.1:8765"


class RobotControlDashboardIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobotControlService()
        self.router = DashboardRouter(
            service=FakeDashboardService(),
            session_token=TOKEN,
            expected_host=HOST,
            robot_control_router=RobotControlHTTPRouter(self.robot),
        )

    @staticmethod
    def headers(method="GET"):
        headers = {
            "Host": HOST,
            "X-Robot-Dashboard-Token": TOKEN,
            "Origin": "http://" + HOST,
        }
        if method in ("POST", "PUT"):
            headers["Content-Type"] = "application/json"
        return headers

    @staticmethod
    def body(value):
        return json.dumps(
            value,
            separators=(",", ":"),
        ).encode("utf-8")

    def test_authenticated_status_is_delegated(self):
        response = self.router.handle(
            "GET",
            "/api/v1/robot/status",
            self.headers(),
        )
        self.assertEqual(response.status, 200)
        self.assertEqual(
            json.loads(response.body)["control"]["state"],
            "IDLE",
        )
        self.assertEqual(self.robot.calls, [("status",)])

    def test_start_keeps_dashboard_auth_and_exact_request_contract(self):
        response = self.router.handle(
            "POST",
            "/api/v1/robot/episodes",
            self.headers("POST"),
            self.body(
                {
                    "goal": "Explore the room.",
                    "locale": "en",
                    "client_request_id": "ui-1",
                    "expected_revision": 3,
                }
            ),
        )
        self.assertEqual(response.status, 202)
        self.assertEqual(
            self.robot.calls[-1],
            (
                "start",
                "Explore the room.",
                "en",
                "ui-1",
                3,
            ),
        )
        rejected = self.router.handle(
            "POST",
            "/api/v1/robot/episodes",
            {
                "Host": HOST,
                "Content-Type": "application/json",
            },
            self.body(
                {
                    "goal": "Explore.",
                    "locale": "en",
                    "client_request_id": "ui-2",
                    "expected_revision": 3,
                }
            ),
        )
        self.assertEqual(rejected.status, 403)

    def test_authenticated_composite_robot_turn_is_delegated(self):
        input_service = FakeRobotInputService()
        router = DashboardRouter(
            service=FakeDashboardService(),
            session_token=TOKEN,
            expected_host=HOST,
            robot_control_router=RobotControlHTTPRouter(
                self.robot,
                input_service,
            ),
        )

        response = router.handle(
            "POST",
            "/api/v1/robot/turns",
            self.headers("POST"),
            self.body({
                "text": "Vad ser du?",
                "locale": "sv",
                "client_request_id": "robot-turn-1",
                "expected_revision": 3,
            }),
        )

        self.assertEqual(response.status, 200)
        self.assertEqual(
            json.loads(response.body)["turn"]["intent"],
            "READ_ONLY_TASK",
        )
        self.assertEqual(
            input_service.calls,
            [("Vad ser du?", "sv", "robot-turn-1", 3)],
        )

    def test_emergency_stop_is_an_explicit_json_mutation(self):
        response = self.router.handle(
            "POST",
            "/api/v1/robot/emergency-stop",
            self.headers("POST"),
            b"{}",
        )
        self.assertEqual(response.status, 200)
        wrong_mime = self.router.handle(
            "POST",
            "/api/v1/robot/emergency-stop",
            {
                **self.headers("POST"),
                "Content-Type": "text/plain",
            },
            b"{}",
        )
        self.assertEqual(wrong_mime.status, 415)


if __name__ == "__main__":
    unittest.main()
