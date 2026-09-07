# BLAST social expressions

The aim is Qwen3.8-selected gestures and facial expressions alongside BLAST's
existing speech. The model chooses whether and how to express itself; motor
execution does not infer frustration, celebration or curiosity from obstacles.
No separate mood engine or automatic obstacle-to-gesture rules are planned.

## Three bounded steps

1. **Accessory hardware and neutral pose:** implement and physically verify
   faces, a short claw snap and an arm/body wiggle with sensor-pose restoration.
2. **Model integration:** expose the verified expressions to Qwen in the existing
   dialogue/action path, combined with speech; no model call for each motor pulse.
3. **Interaction and navigation acceptance:** test chosen speech/expressions,
   cancellation and subsequent navigation without losing the goal, route or map.

Stop and report after each step. EV3's expressions are a later hardware-specific
implementation of the same model ownership, not part of the first BLAST slice.

## Step 1 implementation

The existing BLE protocol now provides two accessory operations:

- `set_pose(motor, target_angle_deg)` for `body` or `claw` only. The hub uses
  non-blocking `run_target`, ending with position hold. It preserves encoder and
  IMU references and requires idle motors. Per-command travel is bounded to
  900 motor degrees for the geared arm (`body`), or 180 for the claw. Speeds
  are 500 and 180 motor degrees/second respectively. These are not arm angles
  or a calibration of the mechanism's end stops. Existing Stop remains
  available while the motor moves.
- `show_face(expression)` for `neutral`, `happy`, `frustrated`, `curious`,
  `surprised`, `angry` or `idle`. Expressions use eyes rather than fitting
  a whole face into 5×5 pixels. `angry` is a large frowning mouth; the matrix is
  monochrome. The button light is reserved for runtime status, independent
  of the selected expression.
  `idle` uses a slow, repeating blink/look animation.
  A new expression replaces the previous animation immediately.

The physical probe composes these primitives into a double claw snap and a
small arm/body wiggle in its original version. The body probe now requires an
explicit motor excursion instead of silently defaulting to a tiny wiggle.
These are explicit test sequences, **not Qwen decisions**.
They are not yet added to the production planner or the navigation loop.

The ultrasonic sensor is mounted under the movable left arm. Restoration uses
the existing navigation calibration's body-motor reference (currently 158°),
not a newly zeroed encoder or IMU. The probe reports whether the measured final
angle matches that existing calibration; its tolerance is not changed here.
The claw returns to its observed starting angle. A cancelled/blocked movement
is stopped, not followed by automatic additional motion in a cleanup handler.

## Stationary hardware probe

Run only with room for the arms and with other BLE owners disconnected. The
default mode observes only. Each invocation deploys the normal hub runtime and
closes its connection afterwards. It never requests wheel motion.

```sh
PYTHONPATH=src .venv/bin/python scripts/probe_blast_expressions.py
PYTHONPATH=src .venv/bin/python scripts/probe_blast_expressions.py --gesture faces
PYTHONPATH=src .venv/bin/python scripts/probe_blast_expressions.py --gesture faces --speech
PYTHONPATH=src .venv/bin/python scripts/probe_blast_expressions.py --gesture claw
PYTHONPATH=src .venv/bin/python scripts/probe_blast_expressions.py --gesture restore
# Replace OFFSET with the motor excursion established for this physical build.
PYTHONPATH=src .venv/bin/python scripts/probe_blast_expressions.py --gesture body --body-offset OFFSET
```

Record the observed movement, final body/claw angles, unchanged drive encoders
and operator confirmation of sensor orientation before accepting step 1.

## Validation so far

44 focused tests pass, including actual hub-function tests (without hardware)
and the BLE protocol tests. They cover explicit face images, accessory-only
positioning, non-blocking execution, invalid/busy requests and preserving
encoder references. The first physical connection attempt did not discover
BLAST; no physical expression or accessory motion was issued by that attempt.
A separate read-only Bluetooth discovery also found no advertising LEGO/Pybricks
device. The updated hub program compiles successfully with the existing
MicroPython toolchain.

### Physical probes, 2026-09-07

BLAST subsequently connected and ran the stationary probes on the real hub.
These were scripted hardware checks, not Qwen-selected gestures.

- Faces: all four expressions were acknowledged. The operator did not watch
  the robot, so visual appearance and orientation remain unconfirmed.
- The first claw attempt failed at the BLE write with error `0x81` (busy),
  before a pose acknowledgement. The new JSON position request exceeded the
  hub's 64-byte stdin ring. Ordinary requests now reuse the existing paced
  control writer from speech; no new retry loop or gesture logic was added.
  The regression test verifies a pose command is split into bounded chunks.
- Claw retry: requested `229 → 184 → 229 → 184°`; observed
  `227 → 187 → 227 → 187°`. Both short excursions completed. Final drive
  encoders matched their starting values and the body remained at `157–158°`.
- Body: requested `188 → 128 → 158°`; observed `184 → 132 → 153°`.
  Both excursions completed, but return to the navigation reference did not
  meet its existing ±1° acceptance. A subsequent observation-only deployment
  read `155°`, so successful motor completion alone does not certify sensor
  restoration. No calibration tolerance or encoder reference was changed.
- No drive command was sent. Drive encoder readings varied by at most 1°
  during these probes. The body probe ended with unchanged drive encoders.
- Distance readings were `2000` throughout (no valid return), so these tests
  do not establish sensor orientation from range data.

All 44 focused tests pass after the transport correction. Step 1 remains
partially validated: visual/mechanical confirmation and reliable neutral-pose
restoration are still required. Qwen integration and post-gesture navigation
acceptance remain pending. The final observed body position is `155°`, not a
certified navigation pose; do not start navigation on the strength of this test.

### Observed display/claw repeat, 2026-09-07

The operator then watched a repeat and reported that every face was sideways
toward BLAST's own left. The first correction (`Side.LEFT`) was confirmed
upside down by the operator, resolving the direction ambiguity. `show_face`
now sets `Side.RIGHT` before rendering each image. This uses the hub's
[display-only orientation API](https://docs.pybricks.com/en/stable/hubs/primehub.html#changing-the-display-orientation),
not any IMU, navigation-frame or motor change. Operator confirmation of the
corrected orientation is pending. The probe shows each expression for three
seconds rather than one. All 44 focused tests and hub compilation still pass.

The first observed face repeat acknowledged all four images, but its final
stationary check saw a 3° change on both drive encoders. No drive command was
sent; the cause of that movement was not established. The subsequent claw
repeat completed both excursions, with identical start/end drive encoders:
requested `232 → 187 → 232 → 187°`, observed `230 → 192 → 230 → 192°`,
settling at `189°`. The body remained at `155°`. The body repeat was deferred
while checking the display correction, then resumed after the `Side.RIGHT`
face sequence completed. That face sequence acknowledged all four images with
unchanged drive encoders; final operator confirmation is still pending.

The observed body repeat requested `188 → 128 → 158°` and reported
`185 → 132 → 153°`. Drive readings differed by at most 1° and the claw stayed
at `189°`. This reproduces the neutral-return mismatch, rather than resolving
it. No wheel command, tolerance change or navigation change was made. Sensor
orientation after the gesture still needs operator confirmation and the
neutral-return acceptance remains open. The repeat has not validated Qwen
integration or subsequent navigation.

### Eye-only display and arm-range correction

The operator requested eye-led expressions and a blink/look idle animation,
plus clearly visible full-range arm gestures with the claw presented forward
or upward. The full-face artwork was replaced rather than adding a second
display system. The idle sequence uses the native non-blocking
`display.animate` API (one 11.55-second cycle), not a host timer, sensor scan,
emotion inference or model request for each frame. It starts only when
`show_face("idle")` is explicitly selected; no automatic emotion triggers or
Qwen integration are introduced in this hardware step.

The tiny arm motion came from applying a motor-angle limit without accounting
for the linkage. The [Pybricks BLAST example](https://pybricks.com/projects/sets/mindstorms-robot-inventor/main-models/blast/)
uses roughly 800 motor degrees for its arm exercise after establishing a
mechanical reference. The original 30-degree test excursion and shared
180-degree cap were inappropriate for this geared arm. The cap now separates
arm and claw capabilities, but the example's absolute poses are **not**
assumed to match this build's existing navigation reference. Measured end
positions and the right-arm presentation pose remain to be established before
accepting a full-range gesture with claw movement. No stall-homing or encoder
reset was added, and the navigation-pose tolerance was not relaxed.

46 focused tests pass, including full arm travel versus the claw limit, idle
frame content, replacing idle with a static expression and clearing the red
accent on return to neutral. The hub program compiles successfully.

The physical eye-only demo attempt did not discover BLAST over Bluetooth
(`find_device` timeout, before runtime deployment). No new display or motor
command reached the robot. Eye-only appearance, idle playback on the custom
firmware, and expanded arm travel are therefore not physically accepted yet.

### Eye-only and larger-arm retry after readiness confirmation

The hub connected on the next attempt. All six static expressions, idle and
the final neutral expression were acknowledged. Observation still responded
after 13 seconds of native idle animation, before neutral replaced it. The
operator's visual acceptance remains pending. Drive encoders changed from
`88/232°` to `55/-1°` during this display-only sequence, with no drive command;
the cause of that movement was not established. The old 2° diagnostic cutoff
ended the probe after all display actions had finished. The probe now reports
encoder deltas separately instead of treating possible handling of the robot
as failure of a display command. It still never requests wheel movement.

A larger arm-location test explicitly requested body target `558°` (reference
`158°` plus 400 motor degrees). Starting from `156°`, it reached `361°` but
remained busy at the 4-second deadline. The probe reported that unfinished
position and stopped the motor; it did not retry or increase the target.
This is **not** evidence of a calibrated mechanical end stop. Drive encoders
remained `55/-1°` and the claw remained `189°`. Visual observation is needed
to distinguish the actual mechanism/pose issue.

An explicit restore-only probe then read `290°` before motion and returned
the body to `158°`; the cause of the change from 361 to 290 was not established.
After a three-second hold, the observed body was still `158°`. A separate
observation-only deployment after release again read `158°`, with unchanged
drive/claw values. Navigation-reference matching therefore passed both under
hold and after release for this recovery. No encoder/IMU reset or tolerance
change was needed. No post-gesture navigation or Qwen selection was tested.

The probe now offers `--gesture restore` and holds body targets for three
seconds for operator observation. All 46 focused tests still pass.

### Speech alongside eyes

At the operator's request the face probe now optionally pairs four short
English lines with `happy`, `curious`, `angry` and `idle`. These are explicitly
scripted demonstration lines, not model decisions or production personality
rules. `--speech` is currently limited to the face demo; no motor command is
added. This does not replace the planned Qwen integration.

The probe reuses `BLAST_PIPER_PROFILE` (`cori-high`, speed 0.98), the existing
Piper synthesizer, ADPCM conversion and normal hub audio transport. Synthesis
finishes before connection. For each line, audio is preloaded through the
single BLE owner, then the display is set and native playback is started.
The expression remains visible for the utterance, and idle can keep animating
while the speaker plays. There is no new speech backend or parallel BLE owner.

47 focused tests pass. The new test checks preload → expression → start order,
and verifies a display-only call does not send audio. Real Piper synthesis
produced four valid hub-format clips: happy 3440 ms, curious 2064 ms, angry
3696 ms and idle 3072 ms. Piper is running locally on port 8179.

The combined physical attempt timed out in Bluetooth discovery before hub
deployment. No combined display/audio command reached BLAST. Actual listening
and simultaneous display/audio acceptance therefore remain pending.

### Combined physical speech/display attempt

The next requested attempt connected and displayed neutral, then happy eyes.
The first English Cori utterance received a native playback-start receipt:
`That's more like it. A girl needs a little personality!` (3392 ms, 27035
ADPCM bytes). Frustrated and curious eyes were then acknowledged. Starting
the second utterance failed with `memory allocation failed, allocating 64008
bytes`. The probe closed the session; angry/idle speech was not attempted.
This is partial evidence of combined playback, not acceptance of the sequence.
Operator listening confirmation is still pending. No motor command was sent.

The failing allocation is consistent with `AppData.get_bytes(0)` copying the
entire fixed 64007-byte receive slot, regardless of the shorter utterance.
The runtime already calls `speaker.done()` and `gc.collect()` before that
copy. The exact retention/fragmentation cause has not been measured on the
hub; it must not be described as fixed or blamed on Piper/Qwen. No firmware,
audio-format, display or navigation change was made after this failure.
The next bounded correction should target repeat-utterance memory handling
and revalidate multiple speeches with the display in the same BLE session.

The operator requested one unchanged repeat. It again acknowledged happy eyes
and started the first Cori utterance (3328 ms, 26571 ADPCM bytes). Curious eyes
were then acknowledged, but the second audio start again failed at request 9
with `memory allocation failed, allocating 64008 bytes`. The same failure has
now occurred on two fresh sessions. No runtime/firmware code or motor command
was changed or introduced for the repeat; later utterances were not played.

### Repeat-utterance memory correction (2026-09-07)

Source inspection identified a retained native ADPCM pointer: clearing the
Python payload reference alone did not release it from MicroPython's
conservative collector. The firmware patch now clears both references on
completion/stop and sound replacement (six added C assignments). No new
streaming protocol, buffer scheme, navigation rule or display behavior was
introduced. A native-function regression failed on the old patch and passed
on the correction; all 48 focused tests pass.

The v7 firmware package is saved under
`local-artifacts/firmware/blast-audio-v7/`; source pin and checksums are in
`BLAST_HUB_AUDIO.md`.

### v7 physical repeat accepted by the runtime

USB DFU installation on BLAST-01 completed on 2026-09-07. The unchanged
`--gesture faces --speech` probe then finished successfully in one BLE session:
all four Cori utterances started (3312, 2032, 3648 and 3104 ms), all eight
expression changes were acknowledged, and observation succeeded during the
idle animation and after the sequence. No allocation error occurred. The
previous second-utterance failure therefore did not recur with the correction.

No motor command was sent. Drive encoder deltas were both zero; final body
angle was 158 degrees and `navigation_sensor_pose_matches` was true. The
session closed cleanly. The operator confirmed the four spoken lines and the
right-way-up eyes, blinking and looking animation worked very well. The fast
delivery noted by the operator used pre-synthesized demonstration lines and
compressed transfer, not live Qwen response generation. This was still a
scripted hardware demonstration. Qwen-driven selection and larger arm/claw
gestures remain separate, unfinished steps.

Evidence: `/tmp/blast-audio-v7-flash-20260907.log`,
`/tmp/blast-face-speech-v7-20260907.jsonl` and its `.stderr` companion.

### GUI conversation and display ownership

The operator confirmed microphone-to-BLAST speech through the GUI on
2026-09-07. The console uses Qwen3.8 and the restored, checksum-verified
Whisper `large-v3-turbo-q5_0`; the temporary Small model is no longer active.
Ordinary replies work, but the conversation contract still has no face/arm
output and classifies explicit gestures as unsupported. Do not describe those
replies as model-selected gestures.

The GUI runtime left the matrix showing Pybricks' default program-running
spinner, because only the standalone probe called `show_face`. The hub runtime
now starts `show_face("idle")` before its ready receipt and sets the button
light green. Face changes no longer overwrite that status light. There is no
added host timer, motion,
firmware change or rule mapping conversation text to emotion. Green means the
hub runtime is active, **not** that Qwen is thinking. Processing-state light
changes and Qwen-selected expressions remain part of the conversation
integration step.

49 focused tests pass, including the startup display/light regression, and the
hub program compiles with the installed MicroPython toolchain.

### Larger symbolic eyes and frown

After visual approval of the startup eyes, the operator rejected the white X
and requested an angry mouth and more expressive eyes. The matrix is only
5×5 monochrome pixels: two eyes with a gap have two columns each, so a clearly
enclosed pupil in each is not practical. Neutral eyes are now 2×3 instead of
2×2; looking frames use the same height. Happy keeps its upward arches,
frustrated uses inward-sloping eye shapes, curious raises one eye, and angry
replaces the eyes with one large downward mouth. No extra expression state,
animation timing, firmware, motor command or Qwen rule was added. All 49
focused tests and hub compilation still pass.

The motor-free physical face probe completed all eight expression changes and
responded during idle animation. Final drive encoder deltas were zero and the
body stayed at the existing 158-degree reference. Shape readability still
needs operator judgment; successful receipts alone do not prove it looks
better. Evidence: `/tmp/blast-larger-eyes-20260907.jsonl`.

### Qwen-selected expressions with GUI replies (7 September)

The operator approved the larger symbolic eyes. BLAST's conversation model now
returns `expression: {face, gesture}` alongside its reply in the **same** Qwen
request. Only the BLAST profile enables these fields; EV3's existing contract
is unchanged. There is no second emotion model or text-to-gesture rule table.

Available faces are idle, neutral, happy, frustrated, curious, surprised and
angry. The initial gesture is `claw_snap`: open by 45 motor degrees and return
to the measured starting angle. Qwen can choose `none`. Large arm/body waves
remain unavailable pending the physical range check described above.

The existing speech worker carries the expression with its utterance. After
audio starts, the same BLE owner displays the chosen face and performs the
optional claw gesture; after speech, idle eyes resume. No drive/body command,
encoder reset or sensor recentering is needed for this gesture. Expression
failure is logged without throwing away speech. This slice accompanies audible
dialogue only: muted replies do not execute expressions, and navigation plans
have not acquired gesture actions.

Validation: 71 focused input/speech/expression tests and the existing 88 monitor
tests passed. Seven direct Qwen probes used the already-loaded Qwen3.8-27B with
reasoning disabled, not scripted decisions. Initial responses took 0.5–1.6 s.
The real GUI first received a refusal to “show me your claw”; that wording did
not reliably select the supported action. A second GUI request, “Snap your claw
once and say hello!”, produced a spoken reply and measured claw motion from
189° to 232°, returning to about 191–192°. Body stayed at 158°; drive readings
only fluctuated by 1° with no drive commands. Text reply arrived about 1.1 s
after submission; claw motion began about 12 s later during the audio path.
That delay is not Qwen response time and still merits separate measurement.
Visual/audio quality awaits the operator's observation of this combined test.

### Larger arm gestures (7 September, next bounded slice)

Two additional Qwen choices are implemented in the same gesture executor:
`arm_wave` moves the coupled arms out by -600 motor degrees and returns to the
existing navigation reference; `claw_flourish` adds an open-and-return claw snap
at the extended arm pose before restoring the sensor arm. These use no wheel
command, encoder reset, new planner or emotion-selection rules. Independent
arm positioning and object manipulation are still not implemented.

The default motor controller's 8-degree integral dead zone allowed a repeatable
return error: a -400° excursion returned to 153° instead of the existing 158°
reference. The body motor alone now uses integral_deadzone=1 and a 1-degree
completion tolerance. Gains and drive-motor settings remain unchanged. This
uses the native [motor controller](https://docs.pybricks.com/en/v4.0.0/pupdevices/motor.html#pybricks.pupdevices.Motor.control.pid),
not corrective host pulses or a relaxed navigation calibration.

Physical results: the repeated -400° excursion returned to 158° with unchanged
drive encoders. A wider test reached -442°, but the opposite target 318° was
still at 314° after four seconds and was stopped; that positive extreme is
**not** used by the new gestures. An experimental constructor profile failed
runtime startup and was removed. The final -600° outward-and-back test reached
-443° and returned to 157°, within the unchanged navigation acceptance. Both
drive encoders were unchanged. Operator judgment of arm/claw presentation is
still required; encoder success does not establish which pose looks best.

Three real Qwen probes selected arm_wave, claw_flourish, and none during an
active navigation episode (0.84–1.25 s, no fallback). The dialogue speaker also
checks current control state immediately before a gesture, so a reply generated
earlier does not start an arm motion during an already active run. Concurrent
navigation/gesture acceptance remains a separate unfinished step.

92 focused tests pass and the hub program compiles. The combined physical
executor test and final GUI test could not connect to BLAST. Thus the larger
arm excursion/restoration is physically verified, but the complete Qwen →
speech → arm → claw → restore sequence is **not yet physically accepted**.
The GUI is running with the new gestures and awaits reconnection.

Evidence: `/tmp/blast-arm-minus400-hold-20260907.jsonl`,
`/tmp/blast-arm-wide-sweep-20260907.jsonl`,
`/tmp/blast-arm-minus600-hold-20260907.jsonl`.

### Complete Qwen/arm/claw run after readiness confirmation

The next physical run completed through the GUI's normal robot-turn service,
using the already-loaded Qwen3.8-27B. Qwen answered in 1.22 s with
“Behold the might of BLAST! *claw snap*”, choosing happy and claw_flourish;
the host did not substitute a scripted gesture or start a navigation episode.
Speech was queued, then the production audio path and gesture executor ran.

Telemetry captured body 157 → -443°, claw 192 → 235 → 195°, and body return
to 158°. Motion began about 11.2 s after submission and ended at 18.4 s. At
31.3 s the robot was still idle at 158°; both drive encoders stayed exactly
-17/0° throughout. Range was 587 mm both before and after. No expression or
audio error was recorded. Thus the complete combined execution and sensor
return are now physically verified; operator listening/visual judgment of the
face and claw presentation is still requested. No code was changed for this
repeat, and post-gesture navigation is not claimed as tested.

Evidence: `/tmp/blast-qwen-arm-claw-live-20260907.jsonl`.

The operator subsequently confirmed: “Ser och låter bra” (looks and sounds
good). This combined physical run is therefore accepted for audible speech
and visible expressions/arm/claw movement, as well as the measured return to
the navigation pose. This acceptance does not extend to gestures during
navigation or to a subsequent navigation run, which remain untested here.
