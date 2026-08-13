import asyncio
import time
import unittest
from unittest import mock

from robot_agent.blast_pcm_upload import (
    PCM_BATCH_TIMEOUT_SECONDS,
    PCM_BEGIN_TIMEOUT_SECONDS,
    PCM_START_REPLY_TIMEOUT_SECONDS,
    BlastPCMDeadline,
    BlastPCMStartTimeout,
    BlastPCMUpload,
)


class HangingRuntime:
    async def begin_pcm(self, payload, *, cancel_requested=None):
        await asyncio.Event().wait()

    async def write_pcm_batch(
        self, offset, payload, *, cancel_requested=None,
    ):
        await asyncio.Event().wait()

    async def start_pcm(
        self,
        transfer_id,
        byte_count,
        fletcher16,
        *,
        cancel_requested=None,
    ):
        await asyncio.Event().wait()


class BlastPCMUploadTests(unittest.IsolatedAsyncioTestCase):
    @staticmethod
    def _upload(*, phase):
        payload = b"1234567"
        upload = BlastPCMUpload(
            requested_generation=1,
            payload=payload,
            result=object(),
            deadline=BlastPCMDeadline(
                inactivity_seconds=60.0,
                maximum_seconds=900.0,
            ),
            cancel_requested=None,
        )
        if phase in ("batch", "start"):
            upload.transfer_id = 1
            upload.batch_bytes = len(payload)
            upload.fletcher16 = 123
        if phase == "start":
            upload.offset = len(payload)
        return upload

    async def test_each_phase_has_its_own_bounded_wall_time_cap(self):
        self.assertEqual(PCM_BEGIN_TIMEOUT_SECONDS, 1.5)
        self.assertEqual(PCM_BATCH_TIMEOUT_SECONDS, 3.0)
        self.assertEqual(PCM_START_REPLY_TIMEOUT_SECONDS, 15.0)

        constants = {
            "begin": "PCM_BEGIN_TIMEOUT_SECONDS",
            "batch": "PCM_BATCH_TIMEOUT_SECONDS",
            "start": "PCM_START_REPLY_TIMEOUT_SECONDS",
        }
        for phase, constant in constants.items():
            with self.subTest(phase=phase), mock.patch(
                "robot_agent.blast_pcm_upload." + constant,
                0.03,
            ):
                upload = self._upload(phase=phase)
                self.assertEqual(upload.current_phase, phase)
                started_at = time.monotonic()
                expected_error = (
                    BlastPCMStartTimeout
                    if phase == "start"
                    else asyncio.TimeoutError
                )
                with self.assertRaises(expected_error) as timed_out:
                    await upload.advance(HangingRuntime())
                self.assertLess(time.monotonic() - started_at, 0.5)
                self.assertFalse(upload.deadline.start_in_flight())
                if phase == "start":
                    self.assertEqual(
                        timed_out.exception.code,
                        "sampled_audio_start_timeout",
                    )

    async def test_start_timeout_replaces_private_transport_message(self):
        class PrivateTimeoutRuntime(HangingRuntime):
            async def start_pcm(
                self,
                transfer_id,
                byte_count,
                fletcher16,
                *,
                cancel_requested=None,
            ):
                raise asyncio.TimeoutError("private BLE transport detail")

        with self.assertRaises(BlastPCMStartTimeout) as timed_out:
            await self._upload(phase="start").advance(
                PrivateTimeoutRuntime()
            )

        self.assertEqual(
            timed_out.exception.code,
            "sampled_audio_start_timeout",
        )
        self.assertNotIn("private", str(timed_out.exception))


if __name__ == "__main__":
    unittest.main()
