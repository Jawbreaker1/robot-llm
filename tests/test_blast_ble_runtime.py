import asyncio
import json
from pathlib import Path
import tempfile
import unittest

from robot_agent.blast_ble_runtime import (
    BlastBLERuntime,
    BlastBLERuntimeError,
)


class FakeHub:
    def __init__(self, ready=None):
        self.connected = False
        self.connect_count = 0
        self.disconnect_count = 0
        self.run_count = 0
        self.writes = []
        self.lines = asyncio.Queue()
        self.ready = ready or {
            "type": "ready",
            "protocol_version": 1,
            "motion_enabled": True,
            "robot_id": "blast-01",
        }

    async def connect(self):
        self.connected = True
        self.connect_count += 1

    async def disconnect(self):
        self.connected = False
        self.disconnect_count += 1

    async def run(self, _path, **_kwargs):
        self.run_count += 1
        await self.lines.put(json.dumps(self.ready))

    async def write_line(self, line):
        request = json.loads(line)
        self.writes.append(request)
        operation = request["op"]
        if operation == "ping":
            result = {"echo": "pong"}
        elif operation == "observe":
            result = {"observed_ms": len(self.writes)}
        elif operation == "stop":
            result = {"stopped": True}
        elif operation == "drive_pulse":
            result = {
                "accepted": True,
                "direction": request["args"]["direction"],
            }
        elif operation == "turn_pulse":
            result = {
                "accepted": True,
                "direction": request["args"]["direction"],
            }
        elif operation == "claw_pulse":
            result = {
                "accepted": True,
                "direction": request["args"]["direction"],
            }
        elif operation == "body_pulse":
            result = {
                "accepted": True,
                "direction": request["args"]["direction"],
            }
        else:
            result = {"shutting_down": True}
        await self.lines.put(
            json.dumps(
                {
                    "id": request["id"],
                    "op": operation,
                    "ok": True,
                    "result": result,
                }
            )
        )

    async def read_line(self):
        return await self.lines.get()

    async def stop_user_program(self):
        raise AssertionError("clean shutdown should not force stop")


class BlastBLERuntimeTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.program_path = Path(self.temporary.name) / "runtime.py"
        self.program_path.write_text("print('test')\n")

    async def asyncTearDown(self):
        self.temporary.cleanup()

    async def test_reuses_one_connection_for_twenty_cycles(self):
        hub = FakeHub()

        async def finder(name):
            self.assertEqual(name, "BLAST-01")
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )

        ready = await runtime.connect()
        for _index in range(20):
            self.assertEqual(await runtime.ping(), {"echo": "pong"})
            self.assertIn("observed_ms", await runtime.observe())
        await runtime.close()

        self.assertEqual(ready["robot_id"], "blast-01")
        self.assertEqual(hub.connect_count, 1)
        self.assertEqual(hub.run_count, 1)
        self.assertEqual(hub.disconnect_count, 1)
        self.assertEqual(len(hub.writes), 41)
        self.assertEqual(hub.writes[-1]["op"], "shutdown")

    async def test_invalid_ready_message_disconnects(self):
        hub = FakeHub(
            ready={
                "type": "ready",
                "protocol_version": 1,
                "motion_enabled": False,
            }
        )

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )

        with self.assertRaisesRegex(
            BlastBLERuntimeError,
            "invalid ready",
        ):
            await runtime.connect()

        self.assertEqual(hub.disconnect_count, 1)

    async def test_disconnect_releases_ble_without_hub_command(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()

        await runtime.disconnect()

        self.assertEqual(hub.disconnect_count, 1)
        self.assertEqual(hub.writes, [])

    async def test_stop_and_fixed_motion_pulses_are_typed(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()

        self.assertEqual(await runtime.stop(), {"stopped": True})
        self.assertEqual(
            await runtime.drive_pulse("forward"),
            {"accepted": True, "direction": "forward"},
        )
        with self.assertRaisesRegex(ValueError, "direction"):
            await runtime.drive_pulse("sideways")
        self.assertEqual(
            await runtime.turn_pulse("left"),
            {"accepted": True, "direction": "left"},
        )
        with self.assertRaisesRegex(ValueError, "direction"):
            await runtime.turn_pulse("forward")
        self.assertEqual(
            await runtime.claw_pulse("open"),
            {"accepted": True, "direction": "open"},
        )
        with self.assertRaisesRegex(ValueError, "direction"):
            await runtime.claw_pulse("left")
        self.assertEqual(
            await runtime.body_pulse("left"),
            {"accepted": True, "direction": "left"},
        )
        with self.assertRaisesRegex(ValueError, "direction"):
            await runtime.body_pulse("forward")
        await runtime.close()

        self.assertEqual(
            hub.writes[1],
            {
                "id": 2,
                "op": "drive_pulse",
                "args": {"direction": "forward"},
            },
        )
        self.assertEqual(
            hub.writes[2],
            {
                "id": 3,
                "op": "turn_pulse",
                "args": {"direction": "left"},
            },
        )
        self.assertEqual(
            hub.writes[3],
            {
                "id": 4,
                "op": "claw_pulse",
                "args": {"direction": "open"},
            },
        )
        self.assertEqual(
            hub.writes[4],
            {
                "id": 5,
                "op": "body_pulse",
                "args": {"direction": "left"},
            },
        )

    async def test_response_timeout_is_bounded(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            timeout_seconds=0.01,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()

        async def no_response(_line):
            return None

        hub.write_line = no_response
        with self.assertRaisesRegex(
            BlastBLERuntimeError,
            "timed out",
        ):
            await runtime.ping()

        hub.write_line = FakeHub.write_line.__get__(hub, FakeHub)
        await runtime.close()


if __name__ == "__main__":
    unittest.main()
