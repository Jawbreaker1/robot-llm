import asyncio
import json
from pathlib import Path
import tempfile
import unittest

from robot_agent.blast_ble_runtime import (
    BlastBLERuntime,
    BlastBLERuntimeError,
    default_program_path,
)


class FakeHub:
    def __init__(self, ready=None):
        self.connected = False
        self.connect_count = 0
        self.disconnect_count = 0
        self.run_count = 0
        self._max_write_size = 100
        self.writes = []
        self.raw_writes = []
        self.fragment_write_counts = []
        self.expected_pcm_bytes = None
        self.pcm_request = None
        self.lines = asyncio.Queue()
        self.ready = ready or {
            "type": "ready",
            "protocol_version": 1,
            "motion_enabled": True,
            "robot_id": "blast-01",
            "capabilities": {
                "sampled_audio_v2": {
                    "sample_rate_hz": 8000,
                    "encoding": "u16le",
                    "max_bytes": 32000,
                    "max_fragment_bytes": 252,
                }
            },
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
        if operation == "play_pcm":
            phase = request["args"]["phase"]
            if phase == "begin":
                self.expected_pcm_bytes = request["args"]["byte_count"]
                self.pcm_request = request
                self.received_pcm_bytes = 0
                result = {
                    "transfer_id": request["id"],
                    "byte_count": self.expected_pcm_bytes,
                    "max_fragment_bytes": 252,
                }
                response_phase = "begun"
            elif phase == "fragment":
                self.pcm_fragment = request
                self.fragment_raw_writes = []
                result = {
                    "transfer_id": request["args"]["transfer_id"],
                    "offset": request["args"]["offset"],
                    "byte_count": request["args"]["byte_count"],
                }
                response_phase = "ready"
            else:
                result = {
                    "transfer_id": request["args"]["transfer_id"],
                    "byte_count": self.expected_pcm_bytes,
                    "sample_rate_hz": 8000,
                    "encoding": "u16le",
                    "duration_ms": (
                        (self.expected_pcm_bytes // 2) * 1000 + 7999
                    ) // 8000,
                }
                response_phase = "started"
            await self.lines.put(
                json.dumps(
                    {
                        "id": request["id"],
                        "op": operation,
                        "phase": response_phase,
                        "ok": True,
                        "result": result,
                    }
                )
            )
            return
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

    async def write(self, payload):
        self.raw_writes.append(bytes(payload))
        self.fragment_raw_writes.append(bytes(payload))
        fragment_bytes = self.pcm_fragment["args"]["byte_count"]
        if sum(map(len, self.fragment_raw_writes)) == fragment_bytes:
            self.fragment_write_counts.append(len(self.fragment_raw_writes))
            self.received_pcm_bytes += fragment_bytes
            await self.lines.put(
                json.dumps(
                    {
                        "id": self.pcm_fragment["id"],
                        "op": "play_pcm",
                        "phase": "received",
                        "ok": True,
                        "result": {
                            "transfer_id": self.pcm_fragment["args"][
                                "transfer_id"
                            ],
                            "offset": self.pcm_fragment["args"]["offset"],
                            "byte_count": fragment_bytes,
                            "received_bytes": self.received_pcm_bytes,
                        },
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

    async def upload_pcm(self, runtime, payload):
        begun = await runtime.begin_pcm(len(payload))
        fragment_bytes = begun["fragment_bytes"]
        for offset in range(0, len(payload), fragment_bytes):
            await runtime.write_pcm_fragment(
                begun["transfer_id"],
                offset,
                payload[offset:offset + fragment_bytes],
            )
        return await runtime.start_pcm(
            begun["transfer_id"],
            len(payload),
        )

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

    async def test_sampled_audio_v2_uploads_bounded_fragments_then_starts(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        ready = await runtime.connect()
        payload = bytes(index % 256 for index in range(32000))

        result = await self.upload_pcm(runtime, payload)

        self.assertIn("sampled_audio_v2", ready["capabilities"])
        self.assertEqual(
            hub.pcm_request["args"],
            {
                "phase": "begin",
                "sample_rate_hz": 8000,
                "encoding": "u16le",
                "byte_count": 32000,
            },
        )
        self.assertEqual(b"".join(hub.raw_writes), payload)
        self.assertTrue(all(len(chunk) <= 63 for chunk in hub.raw_writes))
        fragment_requests = [
            item for item in hub.writes
            if item["op"] == "play_pcm"
            and item["args"]["phase"] == "fragment"
        ]
        self.assertTrue(fragment_requests)
        self.assertTrue(all(
            item["args"]["byte_count"] <= 252
            for item in fragment_requests
        ))
        self.assertTrue(all(
            count <= 4 for count in hub.fragment_write_counts
        ))
        self.assertEqual(
            result,
            {
                "transfer_id": 1,
                "byte_count": 32000,
                "sample_rate_hz": 8000,
                "encoding": "u16le",
                "duration_ms": 2000,
            },
        )
        await runtime.close()

    async def test_pcm_ctrl_c_byte_is_raw_data_and_hub_restores_interrupt(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()
        payload = b"\x03\x80\x00\x03"

        await self.upload_pcm(runtime, payload)

        self.assertEqual(b"".join(hub.raw_writes), payload)
        source = default_program_path().read_text(encoding="utf-8")
        disabled = source.index("micropython.kbd_intr(-1)")
        ready_phase = source.index('"ready",', disabled)
        raw_read = source.index("payload = read_exact(byte_count)", disabled)
        finally_clause = source.index("    finally:", raw_read)
        restored = source.index("micropython.kbd_intr(3)", raw_read)
        playback = source.index("hub.speaker.play_samples(", restored)
        nonblocking = source.index("wait=False", playback)
        self.assertNotIn("from array import array", source)
        self.assertIn('transfer["payload"]', source[playback:nonblocking])
        self.assertLess(disabled, ready_phase)
        self.assertLess(ready_phase, raw_read)
        self.assertLess(raw_read, finally_clause)
        self.assertLess(finally_clause, restored)
        self.assertLess(restored, playback)
        self.assertGreater(nonblocking, playback)
        self.assertIn("stdin.buffer.read(1)", source)
        await runtime.close()

    async def test_sampled_audio_respects_negotiated_write_size(self):
        hub = FakeHub()
        hub._max_write_size = 20

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()

        await self.upload_pcm(runtime, b"\x00\x80" * 20)

        self.assertEqual([len(chunk) for chunk in hub.raw_writes], [19, 19, 2])
        await runtime.close()

    async def test_rejected_begin_remains_line_protocol_aligned(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()

        async def invalid_handshake(line):
            request = json.loads(line)
            hub.writes.append(request)
            await hub.lines.put(
                json.dumps(
                    {
                        "id": request["id"],
                        "op": "play_pcm",
                        "ok": False,
                        "error": "motors busy",
                    }
                )
            )

        hub.write_line = invalid_handshake
        with self.assertRaisesRegex(BlastBLERuntimeError, "invalid play_pcm"):
            await runtime.begin_pcm(2)
        await runtime.close()

        self.assertEqual(
            [item["op"] for item in hub.writes],
            ["play_pcm", "shutdown"],
        )
        self.assertEqual(hub.disconnect_count, 1)

    async def test_cancel_during_fragment_finishes_atomic_raw_frame(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()
        begun = await runtime.begin_pcm(4)
        cancelled = False
        original_write = hub.write

        async def cancel_during_write(payload):
            nonlocal cancelled
            cancelled = True
            await original_write(payload)

        hub.write = cancel_during_write
        result = await runtime.write_pcm_fragment(
            begun["transfer_id"],
            0,
            b"\x00\x80\x00\x80",
            cancel_requested=lambda: cancelled,
        )
        self.assertTrue(runtime.sampled_audio_aligned)
        await runtime.close()

        self.assertEqual(result["received_bytes"], 4)
        self.assertEqual(b"".join(hub.raw_writes), b"\x00\x80\x00\x80")
        self.assertEqual(
            [
                (item["op"], item.get("args", {}).get("phase"))
                for item in hub.writes
            ],
            [
                ("play_pcm", "begin"),
                ("play_pcm", "fragment"),
                ("shutdown", None),
            ],
        )
        self.assertEqual(hub.disconnect_count, 1)

    async def test_invalid_ready_metadata_disconnects_without_shutdown(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()
        begun = await runtime.begin_pcm(4)
        original_write_line = hub.write_line

        async def wrong_ready(line):
            await original_write_line(line)
            queued = json.loads(await hub.lines.get())
            queued["result"]["offset"] = 2
            await hub.lines.put(json.dumps(queued))

        hub.write_line = wrong_ready
        with self.assertRaisesRegex(BlastBLERuntimeError, "ready metadata"):
            await runtime.write_pcm_fragment(
                begun["transfer_id"],
                0,
                b"\x00\x80\x00\x80",
            )
        await runtime.close()

        self.assertEqual(
            [item["args"]["phase"] for item in hub.writes],
            ["begin", "fragment"],
        )
        self.assertEqual(hub.disconnect_count, 1)

    async def test_fragment_ready_timeout_disconnects_without_shutdown(self):
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
        begun = await runtime.begin_pcm(4)

        async def no_ready(line):
            hub.writes.append(json.loads(line))

        hub.write_line = no_ready
        with self.assertRaisesRegex(BlastBLERuntimeError, "timed out"):
            await runtime.write_pcm_fragment(
                begun["transfer_id"],
                0,
                b"\x00\x80\x00\x80",
            )
        await runtime.close()

        self.assertEqual(
            [item["args"]["phase"] for item in hub.writes],
            ["begin", "fragment"],
        )
        self.assertEqual(hub.disconnect_count, 1)

    async def test_sampled_audio_rejects_invalid_size_before_writing(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()

        for byte_count in (0, 1, 32002):
            with self.assertRaisesRegex(ValueError, "byte_count"):
                await runtime.begin_pcm(byte_count)

        self.assertEqual(hub.writes, [])
        await runtime.close()

    async def test_sampled_audio_requires_ready_capability(self):
        hub = FakeHub()
        del hub.ready["capabilities"]

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()

        with self.assertRaisesRegex(
            BlastBLERuntimeError,
            "does not advertise",
        ):
            await runtime.begin_pcm(2)

        self.assertEqual(hub.writes, [])
        await runtime.close()


if __name__ == "__main__":
    unittest.main()
