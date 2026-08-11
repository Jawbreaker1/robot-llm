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
SAMPLED_AUDIO_CAPABILITY = "sampled_audio_v2"
SAMPLED_AUDIO_SAMPLE_RATE_HZ = 8000
SAMPLED_AUDIO_ENCODING = "u16le"
SAMPLED_AUDIO_MAX_BYTES = 32000
SAMPLED_AUDIO_MAX_FRAGMENT_BYTES = 252
SAMPLED_AUDIO_MAX_WRITE_CHUNK_BYTES = 63
SAMPLED_AUDIO_WRITES_PER_FRAGMENT = 4

DeviceFinder = Callable[[str], Awaitable[Any]]
HubFactory = Callable[[Any], Any]


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
        self._sampled_audio_desynchronized = False
        self._next_request_id = 1

    @property
    def sampled_audio_aligned(self) -> bool:
        """Whether another line request is safe after sampled-audio I/O."""

        return (
            self._hub is not None
            and not self._sampled_audio_desynchronized
        )

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
        self._sampled_audio_desynchronized = False
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
            "max_fragment_bytes": SAMPLED_AUDIO_MAX_FRAGMENT_BYTES,
        }
        if capability != expected:
            raise BlastBLERuntimeError(
                "BLAST hub does not advertise sampled_audio_v2"
            )
        return capability

    def _raw_write_size(self) -> int:
        hub = self._hub
        if hub is None:
            raise BlastBLERuntimeError("session is not connected")
        maximum_write_size = getattr(hub, "_max_write_size", 20)
        if not isinstance(maximum_write_size, int):
            maximum_write_size = 20
        return min(
            SAMPLED_AUDIO_MAX_WRITE_CHUNK_BYTES,
            max(1, maximum_write_size - 1),
        )

    async def begin_pcm(
        self,
        byte_count: int,
        *,
        cancel_requested=None,
    ) -> Dict[str, object]:
        """Allocate one bounded PCM upload while BLAST's motors are idle."""
        if (
            isinstance(byte_count, bool)
            or not isinstance(byte_count, int)
            or not 2 <= byte_count <= SAMPLED_AUDIO_MAX_BYTES
            or byte_count % 2
        ):
            raise ValueError(
                "byte_count must be 2..32000 even bytes of u16le PCM"
            )
        if cancel_requested is not None and not callable(cancel_requested):
            raise ValueError("cancel_requested must be callable")
        self._sampled_audio_capability()
        if cancel_requested is not None and cancel_requested():
            raise BlastBLERuntimeError("sampled audio was cancelled")
        request_id = self._next_request_id
        self._next_request_id += 1
        await self._write_sampled_audio_request(
            request_id,
            {
                "phase": "begin",
                "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
                "encoding": SAMPLED_AUDIO_ENCODING,
                "byte_count": byte_count,
            },
        )
        result = self._validate_sampled_audio_message(
            await self._read_message(),
            request_id=request_id,
            phase="begun",
        )
        if result != {
            "transfer_id": request_id,
            "byte_count": byte_count,
            "max_fragment_bytes": SAMPLED_AUDIO_MAX_FRAGMENT_BYTES,
        }:
            raise BlastBLERuntimeError(
                "hub sent invalid play_pcm begun metadata: {!r}".format(
                    result
                )
            )
        fragment_bytes = min(
            SAMPLED_AUDIO_MAX_FRAGMENT_BYTES,
            self._raw_write_size() * SAMPLED_AUDIO_WRITES_PER_FRAGMENT,
        )
        fragment_bytes -= fragment_bytes % 2
        return dict(result, fragment_bytes=max(2, fragment_bytes))

    async def write_pcm_fragment(
        self,
        transfer_id: int,
        offset: int,
        payload: bytes,
        *,
        cancel_requested=None,
    ) -> Dict[str, object]:
        """Send at most four acknowledged GATT writes, then yield BLE."""

        self._sampled_audio_capability()
        if (
            isinstance(transfer_id, bool)
            or not isinstance(transfer_id, int)
            or transfer_id < 1
            or isinstance(offset, bool)
            or not isinstance(offset, int)
            or offset < 0
            or offset % 2
            or type(payload) is not bytes
            or not 2 <= len(payload) <= SAMPLED_AUDIO_MAX_FRAGMENT_BYTES
            or len(payload) % 2
            or cancel_requested is not None
            and not callable(cancel_requested)
        ):
            raise ValueError("sampled audio fragment is invalid")
        if cancel_requested is not None and cancel_requested():
            raise BlastBLERuntimeError("sampled audio was cancelled")
        hub = self._hub
        if hub is None:
            raise BlastBLERuntimeError("session is not connected")
        request_id = self._next_request_id
        self._next_request_id += 1
        arguments = {
            "phase": "fragment",
            "transfer_id": transfer_id,
            "offset": offset,
            "byte_count": len(payload),
        }
        # Once this header may have reached the hub, a missing or malformed
        # response is ambiguous: the hub may already be waiting for raw PCM.
        # Only a complete received phase restores line-protocol certainty.
        self._sampled_audio_desynchronized = True
        await self._write_sampled_audio_request(request_id, arguments)
        ready = self._validate_sampled_audio_message(
            await self._read_message(),
            request_id=request_id,
            phase="ready",
        )
        expected_ready = {
            "transfer_id": transfer_id,
            "offset": offset,
            "byte_count": len(payload),
        }
        if ready != expected_ready:
            raise BlastBLERuntimeError(
                "hub sent invalid play_pcm ready metadata: {!r}".format(
                    ready
                )
            )
        chunk_size = self._raw_write_size()
        for chunk_offset in range(0, len(payload), chunk_size):
            await hub.write(
                payload[chunk_offset:chunk_offset + chunk_size]
            )
        result = self._validate_sampled_audio_message(
            await self._read_message(),
            request_id=request_id,
            phase="received",
        )
        self._sampled_audio_desynchronized = False
        expected = dict(
            expected_ready,
            received_bytes=offset + len(payload),
        )
        if result != expected:
            raise BlastBLERuntimeError(
                "hub sent invalid play_pcm received metadata: {!r}".format(
                    result
                )
            )
        return result

    async def start_pcm(
        self,
        transfer_id: int,
        byte_count: int,
        *,
        cancel_requested=None,
    ) -> Dict[str, object]:
        """Start one uploaded block without waiting for its DMA playback."""

        self._sampled_audio_capability()
        if (
            isinstance(transfer_id, bool)
            or not isinstance(transfer_id, int)
            or transfer_id < 1
            or isinstance(byte_count, bool)
            or not isinstance(byte_count, int)
            or not 2 <= byte_count <= SAMPLED_AUDIO_MAX_BYTES
            or byte_count % 2
            or cancel_requested is not None
            and not callable(cancel_requested)
        ):
            raise ValueError("sampled audio start is invalid")
        if cancel_requested is not None and cancel_requested():
            raise BlastBLERuntimeError("sampled audio was cancelled")
        request_id = self._next_request_id
        self._next_request_id += 1
        await self._write_sampled_audio_request(
            request_id,
            {"phase": "start", "transfer_id": transfer_id},
        )
        result = self._validate_sampled_audio_message(
            await self._read_message(),
            request_id=request_id,
            phase="started",
        )
        expected = {
            "transfer_id": transfer_id,
            "byte_count": byte_count,
            "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
            "encoding": SAMPLED_AUDIO_ENCODING,
            "duration_ms": (
                (byte_count // 2) * 1000
                + SAMPLED_AUDIO_SAMPLE_RATE_HZ
                - 1
            ) // SAMPLED_AUDIO_SAMPLE_RATE_HZ,
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
        hub = self._hub
        if hub is None:
            raise BlastBLERuntimeError("session is not connected")
        await hub.write_line(
            json.dumps(
                {"id": request_id, "op": "play_pcm", "args": arguments},
                separators=(",", ":"),
                sort_keys=True,
            )
        )

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
        self._sampled_audio_desynchronized = False
        await asyncio.wait_for(
            hub.disconnect(),
            timeout=self.timeout_seconds,
        )

    async def close(self) -> None:
        hub = self._hub
        if hub is None:
            return
        if self._sampled_audio_desynchronized:
            await self.disconnect()
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
