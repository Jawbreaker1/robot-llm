"""Persistent, motion-free Pybricks session for BLAST-01."""

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
        self._next_request_id = 1

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
                or ready.get("motion_enabled") is not False
            ):
                raise BlastBLERuntimeError(
                    "hub sent an invalid ready message"
                )
            return ready
        except BaseException:
            self._hub = None
            try:
                await hub.disconnect()
            except Exception:
                pass
            raise

    async def ping(self) -> Dict[str, object]:
        return await self._request("ping")

    async def observe(self) -> Dict[str, object]:
        return await self._request("observe")

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
            self._hub = None
            await hub.disconnect()

    async def _request(self, operation: str) -> Dict[str, object]:
        if operation not in ("ping", "observe", "shutdown"):
            raise ValueError("unsupported BLAST operation")
        hub = self._hub
        if hub is None:
            raise BlastBLERuntimeError("session is not connected")
        request_id = self._next_request_id
        self._next_request_id += 1
        encoded = json.dumps(
            {"id": request_id, "op": operation},
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
