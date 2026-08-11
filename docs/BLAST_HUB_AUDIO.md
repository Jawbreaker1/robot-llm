# BLAST hub audio

BLAST uses Pybricks for navigation. Public Pybricks exposes tones on the
Prime/Inventor Hub, but its firmware already contains the sampled DAC driver
used by the hardware. The patch in
`firmware/pybricks/blast_sampled_audio.patch` exposes one bounded Python API:

```python
hub.speaker.play_samples(samples, sample_rate=8000, wait=False)
hub.speaker.done()
hub.speaker.stop()
```

`samples` must be a `bytearray` containing 1–65,535 unsigned 16-bit
little-endian PCM samples. This exactly matches the raw `u16le` payload received
over BLE and does not require MicroPython's intentionally disabled `array`
module. One-shot `DMA_NORMAL` playback continues after `wait=False` returns, so
the normal hub command loop can drive motors and read sensors while sound is
active. The firmware keeps the bytearray alive until DMA completes; polling
`done()` releases it, and `stop()` is immediate and idempotent.

The BLE runtime advertises this behavior only as `sampled_audio_v2`. A 32 kB
speech block is uploaded as fragments of at most four negotiated GATT writes.
The observation monitor checks stop and navigation work between fragments;
speech is always the lower-priority user of the existing BLE session. Upload
starts only while the motors are idle. Once playback has started, navigation
and the speaker DMA run concurrently.

## Pinned proof build

The asynchronous patch has been compiled and linked with `-Werror` against
upstream
Pybricks commit:

```text
v4.0.1 / 4104553405decb0384bcfb030fbfcb4b5a9854cc
firmware.zip SHA-256: 4bdf084db8ae026be2b8f2764dc0c9a610d2562f528625ee891e7804b5e4c271
```

This artifact was flashed to BLAST-01 on 2026-08-11 after confirming the hub
was running v4.0.1. The audio-only smoke completed through the physical hub
speaker without motor activity. The bounded-motion smoke then verified that
sampled playback and both drive motors were active concurrently, playback
completed, repeated cancellation was safe, and legacy beep/notes still worked.
The build artifact is deliberately not committed. A clean, unpatched build of
the same tag must be kept as the rollback artifact during hardware validation.

Apply and build from a clean, recursively initialized Pybricks checkout with
GNU Arm Embedded GCC 13:

```sh
git checkout v4.0.1
git apply /absolute/path/to/robot-llm/firmware/pybricks/blast_sampled_audio.patch
make primehub
```

The firmware package is written to
`bricks/primehub/build/firmware.zip`. Flashing is a separate, explicit
hardware step. Do not flash while a BLAST navigation episode is active.

After flashing, first run the audio-only smoke program:

```sh
.venv/bin/python -m pybricksdev run ble --name BLAST-01 \
  hub_programs/blast_01/speaker_pcm_smoke.py
```

Then place BLAST in a safe bounded-motion test area and run the explicit
concurrency smoke. It moves both drive motors by 90 degrees while a two-second
PCM block is active:

```sh
.venv/bin/python -m pybricksdev run ble --name BLAST-01 \
  hub_programs/blast_01/speaker_pcm_concurrency_smoke.py
```

Its output must match
`hub_programs/blast_01/speaker_pcm_concurrency_smoke.exp`: the exact raw-byte
decode is correct, motor and PCM activity overlap, playback finishes, repeated
stop is safe, and beep/notes still work. Only after both smokes pass should the
normal BLAST runtime be redeployed and its existing observation and
bounded-navigation smokes be repeated.
