"""Small state object for one interruptible BLAST PCM upload."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import time

from .blast_ble_runtime import SAMPLED_AUDIO_MAX_FRAGMENT_BYTES


RESPONSE_MARGIN_SECONDS = 0.25


@dataclass
class BlastPCMUpload:
    requested_generation: int
    payload: bytes
    result: object
    expires_at: float
    cancel_requested: object
    transfer_id: int | None = None
    fragment_bytes: int | None = None
    offset: int = 0

    @classmethod
    def from_request(cls, request):
        return cls(*request)

    @property
    def needs_idle_observation(self) -> bool:
        return self.transfer_id is None

    async def advance(self, runtime):
        """Perform exactly one begin, fragment, or nonblocking start step."""

        timeout = max(
            0.1,
            self.expires_at - time.monotonic() - RESPONSE_MARGIN_SECONDS,
        )
        if self.transfer_id is None:
            begun = await asyncio.wait_for(
                runtime.begin_pcm(
                    len(self.payload),
                    cancel_requested=self.cancel_requested,
                ),
                timeout=timeout,
            )
            fragment_bytes = begun.get("fragment_bytes")
            if (
                isinstance(fragment_bytes, bool)
                or not isinstance(fragment_bytes, int)
                or not 2 <= fragment_bytes
                <= SAMPLED_AUDIO_MAX_FRAGMENT_BYTES
                or fragment_bytes % 2
            ):
                raise RuntimeError("invalid sampled audio fragment size")
            self.transfer_id = begun["transfer_id"]
            self.fragment_bytes = fragment_bytes
            return None

        if self.offset < len(self.payload):
            fragment = self.payload[
                self.offset:self.offset + self.fragment_bytes
            ]
            receipt = await asyncio.wait_for(
                runtime.write_pcm_fragment(
                    self.transfer_id,
                    self.offset,
                    fragment,
                    cancel_requested=self.cancel_requested,
                ),
                timeout=timeout,
            )
            if receipt.get("received_bytes") != self.offset + len(fragment):
                raise RuntimeError("invalid sampled audio fragment receipt")
            self.offset += len(fragment)
            return None

        return await asyncio.wait_for(
            runtime.start_pcm(
                self.transfer_id,
                len(self.payload),
                cancel_requested=self.cancel_requested,
            ),
            timeout=timeout,
        )


__all__ = ("BlastPCMUpload",)
