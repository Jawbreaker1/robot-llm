"""Small state object for one interruptible BLAST utterance upload."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import math
import threading
import time
from typing import Callable

from .blast_ble_runtime import SAMPLED_AUDIO_MAX_BYTES


RESPONSE_MARGIN_SECONDS = 0.25
START_REPLY_TIMEOUT_SECONDS = 8.5


class BlastPCMDeadline:
    """Thread-safe inactivity deadline with an absolute lifetime ceiling."""

    def __init__(
        self,
        *,
        inactivity_seconds: float,
        maximum_seconds: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        values = (inactivity_seconds, maximum_seconds)
        if (
            any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or float(value) <= 0
                for value in values
            )
            or float(maximum_seconds) < float(inactivity_seconds)
            or not callable(clock)
        ):
            raise ValueError("sampled audio deadline is invalid")
        self._inactivity_seconds = float(inactivity_seconds)
        self._clock = clock
        self._lock = threading.Lock()
        started_at = float(clock())
        self._hard_expires_at = started_at + float(maximum_seconds)
        self._progress_expires_at = min(
            self._hard_expires_at,
            started_at + self._inactivity_seconds,
        )
        self._timed_out = False
        self._start_in_flight = False

    def remaining(self) -> float:
        with self._lock:
            if self._timed_out:
                return 0.0
            return max(
                0.0,
                min(
                    self._progress_expires_at,
                    self._hard_expires_at,
                ) - float(self._clock()),
            )

    def expired(self) -> bool:
        return self.remaining() <= 0.0

    def claim_timeout(self) -> bool:
        """Atomically win timeout only if no progress extended the window."""

        with self._lock:
            if self._timed_out:
                return True
            if self._start_in_flight:
                return False
            if float(self._clock()) < min(
                self._progress_expires_at,
                self._hard_expires_at,
            ):
                return False
            self._timed_out = True
            return True

    def begin_start(self) -> bool:
        """Make the final start reply authoritative over caller timeout."""

        with self._lock:
            now = float(self._clock())
            if self._timed_out or now >= min(
                self._progress_expires_at,
                self._hard_expires_at,
            ):
                self._timed_out = True
                return False
            self._start_in_flight = True
            return True

    def finish_start(self) -> None:
        with self._lock:
            self._start_in_flight = False

    def start_in_flight(self) -> bool:
        with self._lock:
            return self._start_in_flight

    def record_progress(self) -> bool:
        """Refresh inactivity after one acknowledged begin/batch/start step."""

        with self._lock:
            now = float(self._clock())
            if self._timed_out or now >= min(
                self._progress_expires_at,
                self._hard_expires_at,
            ):
                self._timed_out = True
                return False
            self._progress_expires_at = min(
                self._hard_expires_at,
                now + self._inactivity_seconds,
            )
            return True


@dataclass
class BlastPCMUpload:
    requested_generation: int
    payload: bytes
    result: object
    deadline: BlastPCMDeadline
    cancel_requested: object
    transfer_id: int | None = None
    batch_bytes: int | None = None
    fletcher16: int | None = None
    offset: int = 0

    @classmethod
    def from_request(cls, request):
        return cls(*request)

    async def advance(self, runtime):
        """Perform exactly one begin, batch, or nonblocking start step."""

        timeout = max(
            0.1,
            self.deadline.remaining() - RESPONSE_MARGIN_SECONDS,
        )
        if self.transfer_id is None:
            begun = await asyncio.wait_for(
                runtime.begin_pcm(
                    self.payload,
                    cancel_requested=self.cancel_requested,
                ),
                timeout=timeout,
            )
            batch_bytes = begun.get("batch_bytes")
            checksum = begun.get("fletcher16")
            if (
                isinstance(batch_bytes, bool)
                or not isinstance(batch_bytes, int)
                or not 1 <= batch_bytes <= SAMPLED_AUDIO_MAX_BYTES
                or isinstance(checksum, bool)
                or not isinstance(checksum, int)
                or not 0 <= checksum <= 0xFFFF
            ):
                raise RuntimeError("invalid sampled audio batch metadata")
            self.transfer_id = begun["transfer_id"]
            self.batch_bytes = batch_bytes
            self.fletcher16 = checksum
            self.deadline.record_progress()
            return None

        if self.offset < len(self.payload):
            batch = self.payload[
                self.offset:self.offset + self.batch_bytes
            ]
            receipt = await asyncio.wait_for(
                runtime.write_pcm_batch(
                    self.offset,
                    batch,
                    cancel_requested=self.cancel_requested,
                ),
                timeout=timeout,
            )
            if receipt.get("received_bytes") != self.offset + len(batch):
                raise RuntimeError("invalid sampled audio batch receipt")
            self.offset += len(batch)
            self.deadline.record_progress()
            return None

        if not self.deadline.begin_start():
            raise asyncio.TimeoutError
        try:
            return await asyncio.wait_for(
                runtime.start_pcm(
                self.transfer_id,
                len(self.payload),
                self.fletcher16,
                cancel_requested=self.cancel_requested,
                ),
                timeout=START_REPLY_TIMEOUT_SECONDS,
            )
        except BaseException:
            self.deadline.finish_start()
            raise


__all__ = ("BlastPCMDeadline", "BlastPCMUpload")
