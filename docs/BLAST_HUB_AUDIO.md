# BLAST hub audio

BLAST uses Pybricks for navigation. Public Pybricks exposes tones on the
Prime/Inventor Hub, but the firmware already contains the sampled DAC used by
the hardware. The patch in
`firmware/pybricks/blast_sampled_audio.patch` adds two bounded playback APIs.

The low-level PCM API is retained for hardware smoke tests:

```python
hub.speaker.play_samples(samples, sample_rate=16000, wait=False)
hub.speaker.done()
hub.speaker.stop()
```

Production speech uploads one compressed utterance at a time, then uses this
playback API (the internal decoder streams to the DAC, not over Bluetooth):

```python
hub.speaker.play_adpcm(
    payload,
    byte_count=byte_count,
    sample_count=sample_count,
    fletcher16=checksum,
    sample_rate=16000,
    wait=False,
)
```

`play_adpcm()` validates the complete immutable payload before it touches the
current speaker state. It then decodes directly from compressed data into a
one-KiB circular DMA buffer. The DMA interrupt refills one 256-sample half
while the other half plays, so an utterance is continuous and does not depend
on BLE once playback has started. `done()` releases the rooted payload after
the stream ends, and `stop()` is immediate and idempotent.

## Production contract

The runtime advertises exactly `sampled_audio_v5`:

- sample rate: 16 kHz mono;
- encoding: `ima_adpcm4_mono_stream_v1`;
- transport: `app_data_v1`;
- checksum: Fletcher-16;
- maximum: 128,000 samples, eight seconds, and 64,007 encoded bytes.

One seven-byte header precedes the IMA-ADPCM nibbles:

1. signed PCM16 predictor, little-endian;
2. unsigned IMA step index;
3. unsigned 32-bit sample count, little-endian.

The predictor is sample zero. Later samples use the low nibble first. When the
sample count is even, the unused high nibble in the final byte must be zero.
The only valid encoded length is `7 + sample_count // 2`.

The host synthesizes and compresses the entire utterance before calling the
controller. Audio over eight seconds is rejected before `begin`, so no partial
speech can reach BLAST. The complete compressed utterance is uploaded before
`start`; there are no BLE transfers or host-controlled block boundaries in the
middle of a sentence.

Small `begin` and `start` control messages bind the transfer id, rate,
encoding, sample count, byte count, and checksum. The compressed payload uses
Pybricks AppData. Each low-priority upload step contains at most eight
MTU-sized writes, and its final write is acknowledged. Stop is checked between
steps and always has priority. The scheduler runs at most one bounded audio
step per outer monitor turn. A normal episode navigation command is not
admitted while speech is being prepared or uploaded; BLAST remains stationary,
and due telemetry reads may run between audio steps. The complete payload must
be acknowledged before `start`, so native playback remains one continuous
utterance with no partial sentence. Only the correlated, validated `start`
receipt releases speech admission. The adapter then takes a fresh settled
observation and reruns its action and encoder safety gates before it submits a
motor command and starts that command's 15-second timeout. Native playback
releases the BLE session immediately and may overlap motion. A speech request
times out after 60 seconds without
acknowledged audio progress, with a separate 15-minute absolute admission
ceiling. Begin has a 1.5-second wall-time cap, each batch of up to eight
AppData writes gets 3 seconds, and a final start exchange that was claimed
before the ceiling gets at most 15 seconds to collect/snapshot the hub buffer
and return its authoritative playback receipt. A failed audio phase is logged
with its phase, byte offset, and bounded error code. In particular, a missing
final receipt surfaces as `sampled_audio_start_timeout`, without exposing BLE
exception text. The failure is isolated to speech; queued navigation resumes
only after line-protocol alignment has been probed or a fresh BLE session has
been established. Stop is checked between bounded phases; normal drive motion
remains idle during episode upload.

If speech is cancelled during upload, `start` is never sent. Global Stop and
Emergency Stop stop both motors and the speaker. This first version does not
have a separate audio-only cancellation command after native playback has
already started.

## Voice, loudness, and personality

BLAST uses local Piper for both languages: Swedish `lisa-bright` and English
`cori-high` (`en_GB-cori-high`), at speed `0.98`. Cori is a single-speaker UK
English female voice trained at high quality on public-domain LibriVox
recordings; see its [model card](https://huggingface.co/rhasspy/piper-voices/blob/main/en/en_GB/cori/high/MODEL_CARD).
The September 7 correction removes BLAST's separate macOS/Samantha binding;
there is no silent macOS fallback. EV3 and generic speech defaults are unchanged.
Piper's existing API model identifier remains `piper-sv` for compatibility,
but the explicit voice name selects the language model.

Install the three local voice models once with
`scripts/setup_piper_service.sh`, then start the project-owned loopback service
with `scripts/start_piper_service.sh` before launching a physical console.

September 7 validation: 76 focused speech/profile/console tests passed. Three
actual Cori utterances and one Swedish Lisa utterance were synthesized through
the production Piper profile and converted by BLAST's existing ADPCM encoder.
Cori synthesis took 0.28–0.95 seconds for these samples; encoded duration was
2.32–4.50 seconds, within the unchanged eight-second hub limit. Samples are kept
locally under `local-artifacts/blast-voice-20260907/`. This verifies synthesis
and transport format, not a new physical listening test through the hub speaker.

The native ADPCM player already drives the DAC at full numeric scale and does
not use Pybricks' tone/note volume attenuator. To raise perceived speech
volume without hard clipping, the BLAST host applies a bounded, monotonic soft
companding curve after 16 kHz resampling and before ADPCM encoding. It keeps
zero and full scale fixed, preserves sign and sample count, and can be disabled
at the converter seam for an exact A/B test. No firmware gain or sample-rate
pitch manipulation is used.

The BLAST dialogue model and controller-action planner receive the same
host-authored Swedish/English personality: overhyped, cocky, lovably half-mad,
and theatrically combative toward obstacles while warm and harmless toward
people and animals. This deliberately contrasts EV3's grumpy pessimism. Its
prompt scope is explicit:
it may affect only `reply_text` or `utterance`, never intent, confidence,
action, plan, assessment, sensor facts, safety, or completion. A navigation
utterance is admitted after its decision has been validated and before its
corresponding physical action. Speech startup, generation, upload, or playback
failure is fail-open for navigation, but still requires a fresh settled safety
observation before motion. Stop and Emergency Stop cancel the speech worker as
well as stopping the hub speaker.

## Pinned v6 proof build

The patch has been applied to exact upstream Pybricks and built from clean
sources with Arm GNU Toolchain 13.3 and warnings as errors:

```text
Pybricks: v4.0.1 / 4104553405decb0384bcfb030fbfcb4b5a9854cc
firmware patch SHA-256: 41c8b8d549f0a2ac84e63d25fb5ad8c2757ab372914e11bd865c4f08ee0f0037
firmware.zip SHA-256: b3428c3d0660742f8ddee47c1ad18b1d6416537dd5425ef9337712f50d530814
firmware-base.bin SHA-256: e62c1d1da959d30d310607463b551d3738affc89a9b0baab17d10f262e1be88a
```

The local build artifact is:

```text
/private/tmp/blast-primehub-v4.0.1-v6-adpcm.zip
```

The patch also corrects the Prime Hub TIM6 input clock from 48 to 96 MHz. A
motor-free hardware measurement proved the old value played 8,000 requested
samples in 501 ms; the correction made the same playback complete in 1,001 ms.

The v6 patch leaves incoming host connection-parameter negotiation to the
central. An earlier v5 build requested a 15 ms BLE interval, but macOS retained
the effective 30 ms interval and then reproducibly ended three persistent
sessions 39.94 seconds after connection readiness with connection-update status
734. The request provided no observed throughput benefit, so v6 removes only
that request and its Prime Hub configuration macro. The CC2564C controller does
not support BLE Data Length Extension.

Apply and build from a clean recursively initialized checkout with GNU Arm
Embedded GCC 13:

```sh
git checkout v4.0.1
git apply /absolute/path/to/robot-llm/firmware/pybricks/blast_sampled_audio.patch
make -C bricks/primehub clean all
```

The package is written to `bricks/primehub/build/firmware.zip`. Keep a clean
v4.0.1 rollback package while validating the custom build. Never flash during
an active navigation episode.

## v7 repeat-utterance buffer release (2026-09-07)

Two physical speech/eye demos played the first utterance but failed when
allocating the next 64,008-byte AppData copy. The native speaker cleared its
Python `sample_data` reference on completion, but retained the raw
`adpcm_state.data` pointer. MicroPython's conservative collector scans native
heap objects too, so that pointer still kept the previous payload alive.

The correction clears that pointer alongside the existing Python-reference
cleanup: construction, stop, playback completion, `done()`, and replacement
by beeps/notes. It adds six C assignments. The audio format, eight-second
limit, one-utterance transport, DMA buffer, voice and navigation are unchanged.

One regression test compiles the actual patched native completion/stop
functions with hardware stubs. It fails with the old patch and passes with
the correction, checking both references over 20 cycles and retaining them
while playback is active. All 48 focused speech/runtime/expression tests pass.
This checks native reference cleanup, not the physical hub's complete heap.

Built from the same pinned v4.0.1 commit with Arm GNU Toolchain 13.3 and firmware
warnings as errors. The host-only mpy-cross build needs
`CFLAGS_EXTRA=-Wno-gnu-folding-constant` with this Mac's Apple Clang.

```text
firmware patch SHA-256: fbc2dc889cb3adf79c2a6b2fd871de94c0727835d0e79b1576f1676d71564dbc
firmware.zip SHA-256: fec370ff57b92fcd80e6e84c5bfbbee23c1e3427488ed5f31492da6ae92ee856
firmware-base.bin SHA-256: 416df436902eb720de88902bff8d0157dd2b391b0f52097beb3b33654669e06c
local artifact: local-artifacts/firmware/blast-audio-v7/blast-primehub-v4.0.1-v7-audio-release.zip
```

The same directory also holds `blast-primehub-v4.0.1-v6-rollback-rebuilt.zip`,
rebuilt from the committed pre-fix patch against the same upstream source.
Its ZIP SHA-256 is
`1363aedf3d33360ee7a3bac15a770faec417861e81a11ff50f508c7fec9dee25`.
This is a newly built rollback package, not the historical August binary.

Status: installed through USB DFU on BLAST-01 on 2026-09-07; the flasher
completed successfully and retained the `BLAST-01` name. The same four-utterance
Cori/eyes probe then completed in one BLE session (exit 0): all four audio
starts were acknowledged, including the second utterance that previously
failed twice. Durations were 3312, 2032, 3648 and 3104 ms; no allocation error
or disconnect occurred. Observation also succeeded during the idle animation
and after playback. Both drive encoders were unchanged and body angle remained
158 degrees, matching the existing navigation sensor reference. No motor
commands were sent.

This physically reproduces and passes the formerly failing repeat-utterance
sequence; it is not an unlimited-duration memory stress test. The operator
confirmed all four lines and the correctly oriented eyes/idle animation worked
very well, and remarked on the fast delivery. The probe synthesizes its scripted
lines before connecting and sends compressed utterances; it does not measure
Qwen-to-speech response latency. This correction does not change transfer speed.
Evidence: `/tmp/blast-audio-v7-flash-20260907.log`,
`/tmp/blast-face-speech-v7-20260907.jsonl` and its `.stderr` companion.

## Hardware acceptance

The v6 artifact above was flashed successfully to BLAST-01 through USB DFU on
2026-08-12. A motor-free production-runtime probe then completed 90/90 paired
`ping` and `observe` cycles over one uninterrupted 114.9-second session. Every
observation reported `motion_active=false`, stable motor angles, valid IMU and
range data, and battery voltage between 8,282 and 8,291 mV. The raw BLE
disconnect at the end succeeded. This passed well beyond the former
reproducible 39.94-second failure point without a connection update error.

The v5 firmware was flashed successfully to BLAST-01 through USB DFU on
2026-08-12. The production runtime then advertised the exact v5 capability and
played the complete reference utterance in one stream. The host measured:

```text
encoded bytes: 41060
samples: 82106
Fletcher-16: 64127
upload: 7.525 s
start round trip: 0.274 s
DMA-padded playback: 5.136 s
post-playback ping: successful
```

No motor, sensor, observe, or stop command was issued during this first
motion-free run. The listener confirmed that the entire reference sentence
played continuously and sounded good. Heap, mid-playback stop, repetition,
and concurrent-navigation acceptance are still pending.

Validate in this order after flashing:

1. Deploy the normal BLAST runtime and verify the exact
   `sampled_audio_v5` capability.
2. Upload and play one short Swedish phrase without motor commands.
3. Upload and play this continuous reference sentence:
   `Hej Johan, hur är det med dig idag? Jag hoppas det blir bra väder annars
   blir jag arg`.
4. Record upload time, playback duration, free heap while the maximum payload
   is rooted, and DMA completion. The reference fixture is 82,106 samples,
   41,060 encoded bytes, Fletcher-16 64,127, and 5,136 ms DMA-padded
   duration.
5. Verify that drive motion remains idle throughout upload, then begins only
   after the authoritative `start` receipt and a fresh settled safety check.
   Navigation and sensor polling may run during native playback, which must
   contain no repeated or missing audio or visibly jerky drive motion.
6. Test cancellation during upload, then global Stop during playback. The
   former must produce no sound; the latter must brake and silence BLAST.
7. Repeat playback at least 20 times and confirm that the free-heap baseline
   does not decline.

The C ring simulation has reproduced all 82,106 decoded reference samples in
order, followed only by the expected 70 midpoint samples (4.375 ms) needed to
finish the final 256-sample DMA half. Audible continuity, interrupt latency,
heap behavior, and motor coexistence remain physical acceptance gates.
