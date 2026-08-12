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

Production speech uses the compressed streaming API:

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
MTU-sized writes, and its final write is acknowledged. Navigation and stop are
checked between steps and always take priority over the next speech step.
Neither upload nor playback requires idle drive motors.

If speech is cancelled during upload, `start` is never sent. Global Stop and
Emergency Stop stop both motors and the speaker. This first version does not
have a separate audio-only cancellation command after native playback has
already started.

## Voice, loudness, and personality

BLAST has an explicit Swedish Piper profile: `lisa-bright` at speed `0.98`.
This binding exists only in the BLAST dashboard composition; the EV3 and the
generic Piper defaults remain `nst-deep` at speed `1.0`.

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
utterance is queued only after its corresponding physical action result has
been verified. Speech startup, generation, upload, or playback failure is
fail-open for navigation, while Stop and Emergency Stop cancel the speech
worker as well as stopping the hub speaker.

## Pinned proof build

The patch has been applied to exact upstream Pybricks and built from clean
sources with Arm GNU Toolchain 13.3 and warnings as errors:

```text
Pybricks: v4.0.1 / 4104553405decb0384bcfb030fbfcb4b5a9854cc
firmware patch SHA-256: 208fee24764c74cd55080226eb4545414351118f98765a33f12c91a942b4546c
firmware.zip SHA-256: 0fa553600afed4884e6113225f81da4f405f98680511af64d129903bafa49b29
firmware-base.bin SHA-256: b70c62e705f6c172cc826468c4fa5992dc7e27066278159264308ba89eb889d1
```

The local build artifact is:

```text
/private/tmp/blast-primehub-v4.0.1-v5-adpcm.zip
```

The patch also corrects the Prime Hub TIM6 input clock from 48 to 96 MHz. A
motor-free hardware measurement proved the old value played 8,000 requested
samples in 501 ms; the correction made the same playback complete in 1,001 ms.

For incoming host connections the patch requests a 15 ms BLE interval while
preserving the negotiated supervision timeout. macOS accepted the request in
the observed test but retained an effective 30 ms interval. This optimization
is therefore not a correctness dependency. The CC2564C controller does not
support BLE Data Length Extension.

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

## Hardware acceptance

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
5. Run navigation and sensor polling during both upload and playback. Commands
   must win between upload steps, playback must contain no repeated or missing
   audio, and drive motion must not become visibly jerky.
6. Test cancellation during upload, then global Stop during playback. The
   former must produce no sound; the latter must brake and silence BLAST.
7. Repeat playback at least 20 times and confirm that the free-heap baseline
   does not decline.

The C ring simulation has reproduced all 82,106 decoded reference samples in
order, followed only by the expected 70 midpoint samples (4.375 ms) needed to
finish the final 256-sample DMA half. Audible continuity, interrupt latency,
heap behavior, and motor coexistence remain physical acceptance gates.
