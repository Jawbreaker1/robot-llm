import asyncio
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

from robot_agent.blast_ble_runtime import (
    BlastBLERuntime,
    BlastBLERuntimeError,
    PYBRICKS_COMMAND_EVENT_UUID,
    PYBRICKS_WRITE_APP_DATA_COMMAND,
    SAMPLED_AUDIO_APP_DATA_READY_POLL_SECONDS,
    SAMPLED_AUDIO_APP_DATA_READY_WAIT_SECONDS,
    SAMPLED_AUDIO_MAX_BYTES,
    SAMPLED_AUDIO_WRITE_PACING_SECONDS,
    _adpcm_sample_count,
    _fletcher16,
    blast_adpcm_duration_ms,
)


def adpcm_block(sample_count, *, predictor=0, step_index=0):
    payload = bytearray(7 + sample_count // 2)
    payload[0] = predictor & 0xFF
    payload[1] = predictor >> 8 & 0xFF
    payload[2] = step_index
    payload[3] = sample_count & 0xFF
    payload[4] = sample_count >> 8 & 0xFF
    payload[5] = sample_count >> 16 & 0xFF
    payload[6] = sample_count >> 24
    return bytes(payload)


def adpcm_block_for_size(byte_count):
    if byte_count == 7:
        return adpcm_block(1)
    return adpcm_block((byte_count - 7) * 2)


class FakeHub:
    def __init__(self, ready=None):
        self.connected = False
        self.connect_count = 0
        self.disconnect_count = 0
        self.run_count = 0
        self._max_write_size = 512
        self.write_without_response_readiness = True
        self.write_without_response_readiness_calls = 0
        peripheral = SimpleNamespace(
            canSendWriteWithoutResponse=(
                self._can_send_write_without_response
            )
        )
        self._client = SimpleNamespace(
            _backend=SimpleNamespace(_peripheral=peripheral)
        )
        self.writes = []
        self.app_data_writes = []
        self.sampled_control_write_groups = []
        self._sampled_control_buffer = bytearray()
        self._sampled_control_chunks = []
        self.expected_pcm_bytes = None
        self.expected_pcm_samples = None
        self.expected_pcm_checksum = None
        self.app_data = bytearray()
        self.pcm_request = None
        self.lines = asyncio.Queue()
        self.ready = ready or {
            "type": "ready",
            "protocol_version": 1,
            "motion_enabled": True,
            "robot_id": "blast-01",
            "capabilities": {
                "sampled_audio_v5": {
                    "sample_rate_hz": 16000,
                    "encoding": "ima_adpcm4_mono_stream_v1",
                    "max_bytes": 64007,
                    "transport": "app_data_v1",
                    "checksum": "fletcher16",
                }
            },
        }

    def _can_send_write_without_response(self):
        self.write_without_response_readiness_calls += 1
        readiness = self.write_without_response_readiness
        if isinstance(readiness, list):
            return readiness.pop(0)
        if isinstance(readiness, BaseException):
            raise readiness
        return readiness

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
                self.expected_pcm_samples = request["args"]["sample_count"]
                self.expected_pcm_checksum = request["args"]["fletcher16"]
                self.pcm_request = request
                self.app_data = bytearray(self.expected_pcm_bytes)
                result = {
                    "transfer_id": request["id"],
                    "sample_rate_hz": 16000,
                    "encoding": "ima_adpcm4_mono_stream_v1",
                    "sample_count": self.expected_pcm_samples,
                    "byte_count": self.expected_pcm_bytes,
                    "fletcher16": self.expected_pcm_checksum,
                }
                response_phase = "begun"
            else:
                result = {
                    "transfer_id": request["args"]["transfer_id"],
                    "byte_count": self.expected_pcm_bytes,
                    "sample_count": self.expected_pcm_samples,
                    "sample_rate_hz": 16000,
                    "encoding": "ima_adpcm4_mono_stream_v1",
                    "fletcher16": self.expected_pcm_checksum,
                    "duration_ms": blast_adpcm_duration_ms(
                        self.expected_pcm_samples
                    ),
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
        elif operation == "scan_turn_pulse":
            result = {
                "accepted": True,
                "direction": request["args"]["direction"],
            }
        elif operation == "scan_trim_pulse":
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
        payload = bytes(payload)
        self._sampled_control_chunks.append(payload)
        self._sampled_control_buffer.extend(payload)
        if b"\n" not in self._sampled_control_buffer:
            return
        line, separator, remainder = self._sampled_control_buffer.partition(
            b"\n"
        )
        if separator != b"\n" or remainder:
            raise AssertionError(
                "expected exactly one sampled control line"
            )
        self.sampled_control_write_groups.append(
            self._sampled_control_chunks
        )
        self._sampled_control_buffer = bytearray()
        self._sampled_control_chunks = []
        await self.write_line(line.decode("utf-8"))

    async def write_gatt_char(self, uuid, frame, response):
        frame = bytes(frame)
        self.app_data_writes.append((uuid, frame, response))
        offset = frame[1] | frame[2] << 8
        payload = frame[3:]
        self.app_data[offset:offset + len(payload)] = payload

    async def read_line(self):
        return await self.lines.get()

    async def stop_user_program(self):
        raise AssertionError("clean shutdown should not force stop")


class BlastBLERuntimeTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.program_path = Path(self.temporary.name) / "runtime.py"
        self.program_path.write_text("print('test')\n")
        self.write_pacer = AsyncMock()
        self.write_pacer_patch = patch(
            "robot_agent.blast_ble_runtime._pace_sampled_audio_write",
            new=self.write_pacer,
        )
        self.write_pacer_patch.start()

    async def asyncTearDown(self):
        self.write_pacer_patch.stop()
        self.temporary.cleanup()

    async def upload_pcm(self, runtime, payload):
        begun = await runtime.begin_pcm(payload)
        batch_bytes = begun["batch_bytes"]
        for offset in range(0, len(payload), batch_bytes):
            await runtime.write_pcm_batch(
                offset,
                payload[offset:offset + batch_bytes],
            )
        return await runtime.start_pcm(
            begun["transfer_id"],
            len(payload),
            begun["fletcher16"],
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
            await runtime.scan_turn_pulse("right"),
            {"accepted": True, "direction": "right"},
        )
        with self.assertRaisesRegex(ValueError, "direction"):
            await runtime.scan_turn_pulse("forward")
        self.assertEqual(
            await runtime.scan_trim_pulse("left"),
            {"accepted": True, "direction": "left"},
        )
        with self.assertRaisesRegex(ValueError, "direction"):
            await runtime.scan_trim_pulse("forward")
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
                "op": "scan_turn_pulse",
                "args": {"direction": "right"},
            },
        )
        self.assertEqual(
            hub.writes[4],
            {
                "id": 5,
                "op": "scan_trim_pulse",
                "args": {"direction": "left"},
            },
        )
        self.assertEqual(
            hub.writes[5],
            {
                "id": 6,
                "op": "claw_pulse",
                "args": {"direction": "open"},
            },
        )
        self.assertEqual(
            hub.writes[6],
            {
                "id": 7,
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

    async def test_sampled_audio_v5_uploads_whole_utterance_then_starts(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        ready = await runtime.connect()
        payload = adpcm_block(128000)

        result = await self.upload_pcm(runtime, payload)

        self.assertEqual(
            ready["capabilities"]["sampled_audio_v5"],
            {
                "sample_rate_hz": 16000,
                "encoding": "ima_adpcm4_mono_stream_v1",
                "max_bytes": 64007,
                "transport": "app_data_v1",
                "checksum": "fletcher16",
            },
        )
        checksum = _fletcher16(payload)
        self.assertEqual(
            hub.pcm_request["args"],
            {
                "phase": "begin",
                "sample_rate_hz": 16000,
                "encoding": "ima_adpcm4_mono_stream_v1",
                "sample_count": 128000,
                "byte_count": 64007,
                "fletcher16": checksum,
            },
        )
        self.assertEqual(bytes(hub.app_data), payload)
        self.assertTrue(all(
            item[1][0] == PYBRICKS_WRITE_APP_DATA_COMMAND
            for item in hub.app_data_writes
        ))
        self.assertEqual(
            result,
            {
                "transfer_id": 1,
                "byte_count": 64007,
                "sample_count": 128000,
                "sample_rate_hz": 16000,
                "encoding": "ima_adpcm4_mono_stream_v1",
                "fletcher16": checksum,
                "duration_ms": 8000,
            },
        )
        self.assertEqual(
            hub.writes[-1]["args"],
            {
                "phase": "start",
                "transfer_id": 1,
                "sample_rate_hz": 16000,
                "encoding": "ima_adpcm4_mono_stream_v1",
                "sample_count": 128000,
                "byte_count": 64007,
                "fletcher16": checksum,
            },
        )
        await runtime.close()

    async def test_app_data_batch_has_offsets_and_one_final_barrier(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()
        payload = adpcm_block_for_size(509 * 8)
        begun = await runtime.begin_pcm(payload)

        self.write_pacer.reset_mock()
        receipt = await runtime.write_pcm_batch(0, payload)

        self.assertEqual(begun["batch_bytes"], 509 * 8)
        self.assertEqual(receipt["received_bytes"], len(payload))
        self.assertEqual(len(hub.app_data_writes), 8)
        self.assertEqual(
            [frame[1] | frame[2] << 8 for _, frame, _ in hub.app_data_writes],
            [index * 509 for index in range(8)],
        )
        self.assertEqual(
            [response for _, _, response in hub.app_data_writes],
            [False] * 7 + [True],
        )
        self.assertTrue(all(
            uuid == PYBRICKS_COMMAND_EVENT_UUID
            for uuid, _, _ in hub.app_data_writes
        ))
        self.write_pacer.assert_not_awaited()
        await runtime.close()

    async def test_control_lines_remain_chunked_and_paced(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()
        payload = adpcm_block_for_size(16)

        await runtime.begin_pcm(payload)

        expected = json.dumps(
            {
                "id": 1,
                "op": "play_pcm",
                "args": {
                    "phase": "begin",
                    "sample_rate_hz": 16000,
                    "encoding": "ima_adpcm4_mono_stream_v1",
                    "sample_count": _adpcm_sample_count(payload),
                    "byte_count": len(payload),
                    "fletcher16": _fletcher16(payload),
                },
            },
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8") + b"\n"
        chunks = hub.sampled_control_write_groups[0]
        self.assertEqual(b"".join(chunks), expected)
        self.assertTrue(all(1 <= len(chunk) <= 60 for chunk in chunks))
        self.assertEqual(self.write_pacer.await_count, len(chunks) - 1)
        self.assertEqual(SAMPLED_AUDIO_WRITE_PACING_SECONDS, 0.02)
        await runtime.close()

    async def test_app_data_waits_for_transient_corebluetooth_backpressure(self):
        hub = FakeHub()
        hub.write_without_response_readiness = [
            True,
            True,
            False,
            True,
            True,
            True,
            True,
            True,
        ]

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()
        payload = adpcm_block_for_size(509 * 8)
        await runtime.begin_pcm(payload)

        await runtime.write_pcm_batch(0, payload)

        self.assertEqual(
            [response for _, _, response in hub.app_data_writes],
            [False] * 7 + [True],
        )
        self.assertEqual(hub.write_without_response_readiness_calls, 8)
        self.assertEqual(
            [frame[1] | frame[2] << 8 for _, frame, _ in hub.app_data_writes],
            [index * 509 for index in range(8)],
        )
        await runtime.close()

    async def test_app_data_readiness_wait_is_bounded_then_uses_barrier(self):
        hub = FakeHub()
        hub.write_without_response_readiness = False

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()
        payload = adpcm_block_for_size(509 * 2)
        await runtime.begin_pcm(payload)

        started_at = asyncio.get_running_loop().time()
        await runtime.write_pcm_batch(0, payload)
        elapsed = asyncio.get_running_loop().time() - started_at

        self.assertEqual(
            [response for _, _, response in hub.app_data_writes],
            [True, True],
        )
        self.assertGreaterEqual(
            hub.write_without_response_readiness_calls,
            2,
        )
        self.assertLess(
            elapsed,
            SAMPLED_AUDIO_APP_DATA_READY_WAIT_SECONDS + 0.1,
        )
        self.assertEqual(SAMPLED_AUDIO_APP_DATA_READY_WAIT_SECONDS, 0.02)
        self.assertEqual(SAMPLED_AUDIO_APP_DATA_READY_POLL_SECONDS, 0.001)
        await runtime.close()

    async def test_cancel_interrupts_app_data_readiness_wait(self):
        hub = FakeHub()
        hub.write_without_response_readiness = False

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()
        payload = adpcm_block_for_size(509 * 2)
        await runtime.begin_pcm(payload)

        with self.assertRaisesRegex(BlastBLERuntimeError, "cancelled"):
            await runtime.write_pcm_batch(
                0,
                payload,
                cancel_requested=lambda: (
                    hub.write_without_response_readiness_calls >= 1
                ),
            )

        self.assertEqual(hub.write_without_response_readiness_calls, 1)
        self.assertEqual(hub.app_data_writes, [])
        self.assertTrue(runtime.sampled_audio_aligned)
        await runtime.close()

    async def test_app_data_missing_corebluetooth_readiness_uses_barriers(self):
        hub = FakeHub()
        hub._client = object()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()
        payload = adpcm_block_for_size(509 * 8)
        await runtime.begin_pcm(payload)

        await runtime.write_pcm_batch(0, payload)

        self.assertEqual(
            [response for _, _, response in hub.app_data_writes],
            [True] * 8,
        )
        await runtime.close()

    async def test_app_data_respects_negotiated_mtu(self):
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
        payload = adpcm_block_for_size(136)
        begun = await runtime.begin_pcm(payload)
        self.write_pacer.reset_mock()

        await runtime.write_pcm_batch(0, payload)

        self.assertEqual(begun["batch_bytes"], 17 * 8)
        self.assertEqual([len(frame) for _, frame, _ in hub.app_data_writes], [20] * 8)
        self.assertEqual(
            [frame[1] | frame[2] << 8 for _, frame, _ in hub.app_data_writes],
            [index * 17 for index in range(8)],
        )
        self.write_pacer.assert_not_awaited()
        await runtime.close()

    async def test_cancel_between_app_data_writes_stays_line_aligned(self):
        hub = FakeHub()

        async def finder(_name):
            return "device"

        runtime = BlastBLERuntime(
            program_path=self.program_path,
            device_finder=finder,
            hub_factory=lambda device: hub,
        )
        await runtime.connect()
        payload = adpcm_block_for_size(509 * 8)
        await runtime.begin_pcm(payload)
        cancelled = False
        original_write = hub.write_gatt_char

        async def cancel_after_first(uuid, frame, response):
            nonlocal cancelled
            await original_write(uuid, frame, response)
            cancelled = True

        hub.write_gatt_char = cancel_after_first
        with self.assertRaisesRegex(BlastBLERuntimeError, "cancelled"):
            await runtime.write_pcm_batch(
                0,
                payload,
                cancel_requested=lambda: cancelled,
            )

        self.assertEqual(len(hub.app_data_writes), 1)
        self.assertFalse(hub.app_data_writes[0][2])
        self.assertTrue(runtime.sampled_audio_aligned)
        self.assertEqual(await runtime.ping(), {"echo": "pong"})
        await runtime.close()

    async def test_fletcher16_known_vector_and_corruption(self):
        payload = b"abcde"
        checksum = _fletcher16(payload)

        self.assertEqual(checksum, 0xC8F0)
        self.assertNotEqual(checksum, _fletcher16(b"abcdf"))

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
            await hub.lines.put(json.dumps({
                "id": request["id"],
                "op": "play_pcm",
                "ok": False,
                "error": "motors busy",
            }))

        hub.write_line = invalid_handshake
        with self.assertRaisesRegex(BlastBLERuntimeError, "invalid play_pcm"):
            await runtime.begin_pcm(adpcm_block(1))
        self.assertTrue(runtime.sampled_audio_aligned)
        await runtime.close()

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

        for payload in (
            b"",
            b"\x00" * 6,
            b"\x00" * (SAMPLED_AUDIO_MAX_BYTES + 1),
            bytes.fromhex("00 00 59 01 00 00 00"),
            bytes.fromhex("00 00 00 00 00 00 00"),
            bytes.fromhex("00 00 00 01 f4 01 00"),
            bytes.fromhex("00 00 00 02 00 00 00 f0"),
        ):
            with self.assertRaises(ValueError):
                await runtime.begin_pcm(payload)

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
            await runtime.begin_pcm(adpcm_block(1))

        self.assertEqual(hub.writes, [])
        await runtime.close()


if __name__ == "__main__":
    unittest.main()
