"""Small state object for one interruptible BLAST utterance upload."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import time

from .blast_ble_runtime import SAMPLED_AUDIO_MAX_BYTES


RESPONSE_MARGIN_SECONDS = 0.25


@dataclass
class BlastPCMUpload:
    requested_generation: int
    payload: bytes
    result: object
    expires_at: float
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
            self.expires_at - time.monotonic() - RESPONSE_MARGIN_SECONDS,
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
            return None

        return await asyncio.wait_for(
            runtime.start_pcm(
                self.transfer_id,
                len(self.payload),
                self.fletcher16,
                cancel_requested=self.cancel_requested,
            ),
            timeout=timeout,
        )


__all__ = ("BlastPCMUpload",)
