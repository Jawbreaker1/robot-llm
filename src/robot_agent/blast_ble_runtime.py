"""Persistent bounded Pybricks controller session for BLAST-01."""

import argparse
import asyncio
import json
from pathlib import Path
import sys
from typing import Any, Awaitable, Callable, Dict, List, Optional


PROTOCOL_VERSION = 1
DEFAULT_HUB_NAME = "BLAST-01"
DEFAULT_TIMEOUT_SECONDS = 8.0
DEFAULT_CYCLES = 20
SAMPLED_AUDIO_CAPABILITY = "sampled_audio_v5"
SAMPLED_AUDIO_SAMPLE_RATE_HZ = 16000
SAMPLED_AUDIO_ENCODING = "ima_adpcm4_mono_stream_v1"
SAMPLED_AUDIO_MAX_SAMPLES = 128000
SAMPLED_AUDIO_HEADER_BYTES = 7
SAMPLED_AUDIO_MAX_BYTES = (
    SAMPLED_AUDIO_HEADER_BYTES + SAMPLED_AUDIO_MAX_SAMPLES // 2
)
SAMPLED_AUDIO_DMA_CHUNK_SAMPLES = 256
SAMPLED_AUDIO_DMA_CHUNK_DURATION_MS = 16
SAMPLED_AUDIO_TRANSPORT = "app_data_v1"
SAMPLED_AUDIO_CHECKSUM = "fletcher16"
SAMPLED_AUDIO_APP_DATA_WRITES_PER_BATCH = 8
SAMPLED_AUDIO_APP_DATA_MAX_DATA_BYTES = 509
SAMPLED_AUDIO_CONTROL_MAX_WRITE_CHUNK_BYTES = 60
SAMPLED_AUDIO_WRITE_PACING_SECONDS = 0.02
SAMPLED_AUDIO_APP_DATA_READY_WAIT_SECONDS = 0.02
SAMPLED_AUDIO_APP_DATA_READY_POLL_SECONDS = 0.001
PYBRICKS_COMMAND_EVENT_UUID = "c5f50002-8280-46da-89f4-6d8051e4aeef"
PYBRICKS_WRITE_APP_DATA_COMMAND = 7

DeviceFinder = Callable[[str], Awaitable[Any]]
HubFactory = Callable[[Any], Any]


async def _pace_sampled_audio_write() -> None:
    await asyncio.sleep(SAMPLED_AUDIO_WRITE_PACING_SECONDS)


def _fletcher16(payload: bytes) -> int:
    sum1 = 0
    sum2 = 0
    for byte in payload:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return sum2 << 8 | sum1


def _adpcm_sample_count(payload: bytes) -> int:
    """Validate one canonical self-contained BLAST ADPCM utterance."""

    if type(payload) is not bytes or not (
        SAMPLED_AUDIO_HEADER_BYTES
        <= len(payload)
        <= SAMPLED_AUDIO_MAX_BYTES
    ):
        raise ValueError("sampled audio payload size is invalid")
    step_index = payload[2]
    sample_count = (
        payload[3]
        | payload[4] << 8
        | payload[5] << 16
        | payload[6] << 24
    )
    if step_index > 88 or not 1 <= sample_count <= SAMPLED_AUDIO_MAX_SAMPLES:
        raise ValueError("sampled audio stream header is invalid")
    if len(payload) != SAMPLED_AUDIO_HEADER_BYTES + sample_count // 2:
        raise ValueError("sampled audio stream length is invalid")
    if sample_count % 2 == 0 and payload[-1] & 0xF0:
        raise ValueError("sampled audio padding nibble must be zero")
    return sample_count


def blast_adpcm_duration_ms(sample_count: int) -> int:
    """Return truthful playback time including the final DMA half-buffer."""

    if (
        isinstance(sample_count, bool)
        or not isinstance(sample_count, int)
        or not 1 <= sample_count <= SAMPLED_AUDIO_MAX_SAMPLES
    ):
        raise ValueError("sampled audio sample count is invalid")
    chunks = (
        sample_count + SAMPLED_AUDIO_DMA_CHUNK_SAMPLES - 1
    ) // SAMPLED_AUDIO_DMA_CHUNK_SAMPLES
    return chunks * SAMPLED_AUDIO_DMA_CHUNK_DURATION_MS


class BlastBLERuntimeError(RuntimeError):
    """The persistent BLAST session could not be used reliably."""


def default_program_path() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / "hub_programs"
        / "blast_01"
        / "runtime.py"
    )


class BlastBLERuntime:
    """One request at a time over one Pybricks BLE connection."""

    def __init__(
        self,
        hub_name: str = DEFAULT_HUB_NAME,
        program_path: Optional[Path] = None,
        timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
        device_finder: Optional[DeviceFinder] = None,
        hub_factory: Optional[HubFactory] = None,
    ) -> None:
        if not isinstance(hub_name, str) or not hub_name.strip():
            raise ValueError("hub_name must not be empty")
        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive")
        self.hub_name = hub_name
        self.program_path = Path(
            program_path or default_program_path()
        )
        self.timeout_seconds = float(timeout_seconds)
        self._device_finder = device_finder
        self._hub_factory = hub_factory
        self._hub = None
        self._ready = None
        self._sampled_audio_transfer = None
        self._next_request_id = 1

    @property
    def sampled_audio_aligned(self) -> bool:
        """Whether another line request is safe after sampled-audio I/O."""

        return self._hub is not None

    async def connect(self) -> Dict[str, object]:
        if self._hub is not None:
            raise BlastBLERuntimeError("session is already connected")
        if not self.program_path.is_file():
            raise BlastBLERuntimeError(
                "hub program was not found: {}".format(
                    self.program_path
                )
            )

        device_finder = self._device_finder
        hub_factory = self._hub_factory
        if device_finder is None or hub_factory is None:
            try:
                from pybricksdev.ble import find_device
                from pybricksdev.connections.pybricks import (
                    PybricksHubBLE,
                )
            except ImportError as error:
                raise BlastBLERuntimeError(
                    "install requirements-pybricks.txt first"
                ) from error
            device_finder = device_finder or find_device
            hub_factory = hub_factory or PybricksHubBLE

        device = await device_finder(self.hub_name)
        hub = hub_factory(device)
        self._hub = hub
        try:
            await hub.connect()
            await hub.run(
                str(self.program_path),
                wait=False,
                print_output=False,
                line_handler=True,
            )
            ready = await self._read_message()
            if (
                ready.get("type") != "ready"
                or ready.get("protocol_version")
                != PROTOCOL_VERSION
                or ready.get("motion_enabled") is not True
            ):
                raise BlastBLERuntimeError(
                    "hub sent an invalid ready message"
                )
            self._ready = ready
            return ready
        except BaseException:
            self._hub = None
            self._ready = None
            try:
                await asyncio.wait_for(
                    hub.disconnect(),
                    timeout=self.timeout_seconds,
                )
            except Exception:
                pass
            raise

    async def ping(self) -> Dict[str, object]:
        return await self._request("ping")

    async def observe(self) -> Dict[str, object]:
        return await self._request("observe")

    async def stop(self) -> Dict[str, object]:
        return await self._request("stop")

    async def drive_pulse(self, direction: str) -> Dict[str, object]:
        if direction not in ("forward", "reverse"):
            raise ValueError("direction must be forward or reverse")
        return await self._request(
            "drive_pulse",
            {"direction": direction},
        )

    async def turn_pulse(self, direction: str) -> Dict[str, object]:
        if direction not in ("left", "right"):
            raise ValueError("direction must be left or right")
        return await self._request(
            "turn_pulse",
            {"direction": direction},
        )

    async def scan_turn_pulse(self, direction: str) -> Dict[str, object]:
        if direction not in ("left", "right"):
            raise ValueError("direction must be left or right")
        return await self._request(
            "scan_turn_pulse",
            {"direction": direction},
        )

    async def claw_pulse(self, direction: str) -> Dict[str, object]:
        if direction not in ("open", "close"):
            raise ValueError("direction must be open or close")
        return await self._request(
            "claw_pulse",
            {"direction": direction},
        )

    async def body_pulse(self, direction: str) -> Dict[str, object]:
        if direction not in ("left", "right"):
            raise ValueError("direction must be left or right")
        return await self._request(
            "body_pulse",
            {"direction": direction},
        )

    def _sampled_audio_capability(self):
        capability = (
            self._ready.get("capabilities", {}).get(
                SAMPLED_AUDIO_CAPABILITY
            )
            if isinstance(self._ready, dict)
            else None
        )
        expected = {
            "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
            "encoding": SAMPLED_AUDIO_ENCODING,
            "max_bytes": SAMPLED_AUDIO_MAX_BYTES,
            "transport": SAMPLED_AUDIO_TRANSPORT,
            "checksum": SAMPLED_AUDIO_CHECKSUM,
        }
        if capability != expected:
            raise BlastBLERuntimeError(
                "BLAST hub does not advertise sampled_audio_v5"
            )
        return capability

    def _control_write_size(self) -> int:
        hub = self._hub
        if hub is None:
            raise BlastBLERuntimeError("session is not connected")
        maximum_write_size = getattr(hub, "_max_write_size", 20)
        if not isinstance(maximum_write_size, int):
            maximum_write_size = 20
        return min(
            SAMPLED_AUDIO_CONTROL_MAX_WRITE_CHUNK_BYTES,
            max(1, maximum_write_size - 1),
        )

    def _app_data_write_size(self) -> int:
        hub = self._hub
        if hub is None:
            raise BlastBLERuntimeError("session is not connected")
        maximum_write_size = getattr(hub, "_max_write_size", 20)
        if not isinstance(maximum_write_size, int):
            maximum_write_size = 20
        data_size = min(
            SAMPLED_AUDIO_APP_DATA_MAX_DATA_BYTES,
            maximum_write_size - 3,
        )
        if data_size < 1:
            raise BlastBLERuntimeError(
                "negotiated BLE write size cannot carry AppData"
            )
        return data_size

    @staticmethod
    async def _app_data_without_response_ready(
        hub,
        *,
        cancel_requested=None,
    ) -> bool:
        """Briefly drain CoreBluetooth's WNR queue or request a barrier."""

        try:
            readiness = (
                hub._client._backend._peripheral
                .canSendWriteWithoutResponse
            )
        except Exception:
            return False
        if not callable(readiness):
            return False

        loop = asyncio.get_running_loop()
        deadline = (
            loop.time() + SAMPLED_AUDIO_APP_DATA_READY_WAIT_SECONDS
        )
        while True:
            if cancel_requested is not None and cancel_requested():
                raise BlastBLERuntimeError("sampled audio was cancelled")
            try:
                if readiness() is True:
                    return True
            except Exception:
                return False
            remaining = deadline - loop.time()
            if remaining <= 0:
                return False
            await asyncio.sleep(min(
                SAMPLED_AUDIO_APP_DATA_READY_POLL_SECONDS,
                remaining,
            ))

    async def begin_pcm(
        self,
        payload: bytes,
        *,
        cancel_requested=None,
    ) -> Dict[str, object]:
        """Allocate one bounded, fully preloaded ADPCM utterance."""
        sample_count = _adpcm_sample_count(payload)
        if cancel_requested is not None and not callable(cancel_requested):
            raise ValueError("cancel_requested must be callable")
        self._sampled_audio_capability()
        if cancel_requested is not None and cancel_requested():
            raise BlastBLERuntimeError("sampled audio was cancelled")
        byte_count = len(payload)
        checksum = _fletcher16(payload)
        request_id = self._next_request_id
        self._next_request_id += 1
        await self._write_sampled_audio_request(
            request_id,
            {
                "phase": "begin",
                "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
                "encoding": SAMPLED_AUDIO_ENCODING,
                "sample_count": sample_count,
                "byte_count": byte_count,
                "fletcher16": checksum,
            },
        )
        result = self._validate_sampled_audio_message(
            await self._read_message(),
            request_id=request_id,
            phase="begun",
        )
        if result != {
            "transfer_id": request_id,
            "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
            "encoding": SAMPLED_AUDIO_ENCODING,
            "sample_count": sample_count,
            "byte_count": byte_count,
            "fletcher16": checksum,
        }:
            raise BlastBLERuntimeError(
                "hub sent invalid play_pcm begun metadata: {!r}".format(
                    result
                )
            )
        self._sampled_audio_transfer = dict(result)
        return dict(
            result,
            batch_bytes=(
                self._app_data_write_size()
                * SAMPLED_AUDIO_APP_DATA_WRITES_PER_BATCH
            ),
        )

    async def write_pcm_batch(
        self,
        offset: int,
        payload: bytes,
        *,
        cancel_requested=None,
    ) -> Dict[str, object]:
        """Send up to eight AppData writes, then yield to navigation."""

        self._sampled_audio_capability()
        batch_bytes = (
            self._app_data_write_size()
            * SAMPLED_AUDIO_APP_DATA_WRITES_PER_BATCH
        )
        if (
            isinstance(offset, bool)
            or not isinstance(offset, int)
            or offset < 0
            or type(payload) is not bytes
            or not 1 <= len(payload) <= batch_bytes
            or offset + len(payload) > SAMPLED_AUDIO_MAX_BYTES
            or cancel_requested is not None
            and not callable(cancel_requested)
        ):
            raise ValueError("sampled audio batch is invalid")
        hub = self._hub
        if hub is None:
            raise BlastBLERuntimeError("session is not connected")
        data_size = self._app_data_write_size()
        for chunk_offset in range(0, len(payload), data_size):
            if cancel_requested is not None and cancel_requested():
                raise BlastBLERuntimeError("sampled audio was cancelled")
            frame_offset = offset + chunk_offset
            chunk = payload[chunk_offset:chunk_offset + data_size]
            frame = bytes(
                (
                    PYBRICKS_WRITE_APP_DATA_COMMAND,
                    frame_offset & 0xFF,
                    frame_offset >> 8,
                )
            ) + chunk
            is_last = chunk_offset + len(chunk) == len(payload)
            response = True
            if not is_last:
                response = not await self._app_data_without_response_ready(
                    hub,
                    cancel_requested=cancel_requested,
                )
            await hub.write_gatt_char(
                PYBRICKS_COMMAND_EVENT_UUID,
                frame,
                response,
            )
        return {
            "offset": offset,
            "byte_count": len(payload),
            "received_bytes": offset + len(payload),
        }

    async def start_pcm(
        self,
        transfer_id: int,
        byte_count: int,
        fletcher16: int,
        *,
        cancel_requested=None,
    ) -> Dict[str, object]:
        """Start one verified utterance without waiting for playback."""

        self._sampled_audio_capability()
        transfer = self._sampled_audio_transfer
        if (
            isinstance(transfer_id, bool)
            or not isinstance(transfer_id, int)
            or transfer_id < 1
            or cancel_requested is not None
            and not callable(cancel_requested)
        ):
            raise ValueError("sampled audio start is invalid")
        if (
            not isinstance(transfer, dict)
            or transfer.get("transfer_id") != transfer_id
        ):
            raise ValueError("sampled audio transfer metadata is invalid")
        self._sampled_audio_transfer = None
        if (
            isinstance(byte_count, bool)
            or not isinstance(byte_count, int)
            or not SAMPLED_AUDIO_HEADER_BYTES
            <= byte_count
            <= SAMPLED_AUDIO_MAX_BYTES
            or isinstance(fletcher16, bool)
            or not isinstance(fletcher16, int)
            or not 0 <= fletcher16 <= 0xFFFF
        ):
            raise ValueError("sampled audio start is invalid")
        if cancel_requested is not None and cancel_requested():
            raise BlastBLERuntimeError("sampled audio was cancelled")
        if (
            transfer.get("byte_count") != byte_count
            or transfer.get("fletcher16") != fletcher16
        ):
            raise ValueError("sampled audio transfer metadata is invalid")
        request_id = self._next_request_id
        self._next_request_id += 1
        await self._write_sampled_audio_request(
            request_id,
            dict(transfer, phase="start"),
        )
        result = self._validate_sampled_audio_message(
            await self._read_message(),
            request_id=request_id,
            phase="started",
        )
        expected = {
            "transfer_id": transfer_id,
            "byte_count": byte_count,
            "sample_count": transfer["sample_count"],
            "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
            "encoding": SAMPLED_AUDIO_ENCODING,
            "fletcher16": fletcher16,
            "duration_ms": blast_adpcm_duration_ms(
                transfer["sample_count"]
            ),
        }
        if result != expected:
            raise BlastBLERuntimeError(
                "hub sent invalid play_pcm started metadata: {!r}".format(
                    result
                )
            )
        return result

    async def _write_sampled_audio_request(
        self, request_id: int, arguments: Dict[str, object]
    ) -> None:
        encoded = json.dumps(
            {"id": request_id, "op": "play_pcm", "args": arguments},
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8") + b"\n"
        await self._write_sampled_audio_control_bytes(encoded)

    async def _write_sampled_audio_control_bytes(
        self, payload: bytes,
    ) -> None:
        """Write stdin without overflowing the Prime Hub's 64-byte ring."""

        hub = self._hub
        if hub is None:
            raise BlastBLERuntimeError("session is not connected")
        chunk_size = self._control_write_size()
        for chunk_offset in range(0, len(payload), chunk_size):
            await hub.write(
                payload[chunk_offset:chunk_offset + chunk_size]
            )
            if chunk_offset + chunk_size < len(payload):
                await _pace_sampled_audio_write()

    @staticmethod
    def _validate_sampled_audio_message(
        message,
        *,
        request_id: int,
        phase: str,
    ) -> Dict[str, object]:
        if (
            message.get("id") != request_id
            or message.get("op") != "play_pcm"
            or message.get("phase") != phase
            or message.get("ok") is not True
            or not isinstance(message.get("result"), dict)
        ):
            raise BlastBLERuntimeError(
                "hub sent an invalid play_pcm {} response: {!r}".format(
                    phase,
                    message,
                )
            )
        return message["result"]

    async def disconnect(self) -> None:
        """Release BLE without sending a command to the hub program."""
        hub = self._hub
        if hub is None:
            return
        self._hub = None
        self._ready = None
        self._sampled_audio_transfer = None
        await asyncio.wait_for(
            hub.disconnect(),
            timeout=self.timeout_seconds,
        )

    async def close(self) -> None:
        hub = self._hub
        if hub is None:
            return
        try:
            await self._request("shutdown")
        except Exception:
            try:
                await hub.stop_user_program()
            except Exception:
                pass
        finally:
            await self.disconnect()

    async def _request(
        self,
        operation: str,
        arguments: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        if operation not in (
            "ping",
            "observe",
            "stop",
            "drive_pulse",
            "turn_pulse",
            "scan_turn_pulse",
            "claw_pulse",
            "body_pulse",
            "shutdown",
        ):
            raise ValueError("unsupported BLAST operation")
        hub = self._hub
        if hub is None:
            raise BlastBLERuntimeError("session is not connected")
        request_id = self._next_request_id
        self._next_request_id += 1
        request = {"id": request_id, "op": operation}
        if arguments is not None:
            request["args"] = arguments
        encoded = json.dumps(
            request,
            separators=(",", ":"),
            sort_keys=True,
        )
        await hub.write_line(encoded)
        response = await self._read_message()
        if (
            response.get("id") != request_id
            or response.get("op") != operation
            or response.get("ok") is not True
            or not isinstance(response.get("result"), dict)
        ):
            raise BlastBLERuntimeError(
                "hub sent an invalid {} response: {!r}".format(
                    operation,
                    response,
                )
            )
        return response["result"]

    async def _read_message(self) -> Dict[str, object]:
        hub = self._hub
        if hub is None:
            raise BlastBLERuntimeError("session is not connected")
        try:
            line = await asyncio.wait_for(
                hub.read_line(),
                timeout=self.timeout_seconds,
            )
        except asyncio.TimeoutError:
            raise BlastBLERuntimeError(
                "timed out waiting for BLAST-01"
            ) from None
        try:
            value = json.loads(line)
        except (TypeError, ValueError):
            raise BlastBLERuntimeError(
                "hub sent invalid JSON: {!r}".format(line[:200])
            ) from None
        if not isinstance(value, dict):
            raise BlastBLERuntimeError(
                "hub response was not an object"
            )
        return value


async def run_probe(
    hub_name: str,
    cycles: int,
    program_path: Optional[Path] = None,
) -> Dict[str, object]:
    runtime = BlastBLERuntime(
        hub_name=hub_name,
        program_path=program_path,
    )
    ready = await runtime.connect()
    observations = []
    try:
        for _index in range(cycles):
            await runtime.ping()
            observations.append(await runtime.observe())
    finally:
        await runtime.close()
    return {
        "status": "completed",
        "mode": "blast-motion-free-persistent-ble",
        "hub_name": hub_name,
        "cycles": cycles,
        "single_ble_connection": True,
        "ready": ready,
        "first_observation": observations[0],
        "last_observation": observations[-1],
    }


def _cycles(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError(
            "cycles must be an integer"
        ) from None
    if not 1 <= parsed <= 100:
        raise argparse.ArgumentTypeError(
            "cycles must be between 1 and 100"
        )
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run repeated motor-free observations over one BLAST BLE "
            "connection."
        )
    )
    parser.add_argument("--hub-name", default=DEFAULT_HUB_NAME)
    parser.add_argument(
        "--cycles",
        type=_cycles,
        default=DEFAULT_CYCLES,
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = asyncio.run(
            run_probe(args.hub_name, args.cycles)
        )
    except (BlastBLERuntimeError, OSError) as error:
        print("BLAST probe failed: {}".format(error), file=sys.stderr)
        return 1
    print(
        json.dumps(
            result,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
