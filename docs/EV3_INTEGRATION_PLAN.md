# EV3 integration: active plan

## Current checkpoint: contract audit before further refactoring (2026-09-26)

This checkpoint preserves the current EV3 integration and removal of the old
physical navigation engine. It is a recovery point, **not** physical acceptance
or a claim that BLAST and EV3 have a unified navigation implementation.

The audit reproduced these outstanding differences without physical motion:

- EV3 transport failure can escape the episode without a result for Qwen;
  BLAST also has terminal paths for repeated invalid replies/no available action.
- The shared prompt describes semantic ADVANCE, but EV3 executes one pulse
  (52 mm in the probe); BLAST can continue toward a target. The decoder's
  implicit ADVANCE/COMPLETE tail is part of this inconsistent behavior.
- BLAST rejects a diagonal route leg; EV3 accepted (300, 300) and began a
  45.1-degree simulated turn through the same model-response decoder.
- A scan without repeated waypoint fields preserves EV3's route but can be
  rejected by BLAST's active-route requirement.
- EV3 simulation uses the production episode/body, but its transport ignores
  request timeouts and does not reproduce slow startup or lost responses.

The recent local startup-timeout retry is included to preserve the exact
working-tree checkpoint. It was not loaded into the live console and is not
the agreed architectural solution. The 203 focused tests passed despite the
audit probes above: existing coverage is not acceptance of these principles.
The complete checkpoint quality gate also passed: JavaScript syntax checks
and 1,988 tests in 84.4 seconds. No physical robot or live-model test was run
for this checkpoint.

Next, establish the same behavioral tests for both bodies, then replace the
conflicting handling in small steps. Preserve mission, route and measured
progress across recoverable failures; report action outcomes to Qwen; never
blindly replay motion whose result was lost. Keep genuine differences in
sensors, body geometry, motor execution and transports. Do not add a third
navigation engine or merge entire loops merely for identical structure.

## Purpose and boundaries

One application owns goals, model-authored routes, conversation and shared
knowledge. BLAST and EV3 supply different bodies and sensing capabilities.
Qwen chooses the route; execution follows it and reports measured results.
Do not recreate EV3's host-authored detours or wrap it in a fake BLAST sensor
interface. Robot-to-robot conversation and shared observations remain required
integration milestones, not capabilities established by two separate chats.

Autonomous navigation acceptance must use only the mission goal, for example
“Drive 800 mm straight ahead from the starting position.” Keep measured body
geometry, sensor capabilities and robot observations in context, but do not
seed the mission with a box's position, color, dimensions, sensor-height caveat
or supposedly open sides. The robot must discover obstacles and choose the
route itself. Operator scene observations belong in the external test record,
not as privileged planner input. A successful seeded run is not acceptance
of autonomous perception and navigation.

## Working baseline

- BLAST's clear-floor arrival and left detour with boxes ahead/right were
  physically confirmed, including map/reality agreement, speech and gestures.
  Baseline commit: `a51c4d1` on `main`.
- EV3's 17 worker deployment files matched the checkout. Its stationary
  preflight passed with no motor commands. A physical left-quarter-turn and
  return completed with verified encoder receipts. The operator saw the
  turn/return but did not confirm exact angles.
- EV3 has IR proximity, touch and motor encoders, not ultrasound or a gyro.
  IR is unitless reflected-light evidence, not a distance measurement.
- Both bodies use shared conversation/classification infrastructure and Piper.
  EV3's hardware descriptor reaches Qwen; John is its English Piper voice.
- The combined dashboard still serializes physical missions. Shared map display
  does not establish shared planning knowledge or coordinated physical motion.

## Recovered hardware evidence and IR display correction (2026-09-14)

The original operator messages were recovered from this task, rather than
inferring sensor placement from the collision envelope:

- 2026-08-03 12:27 UTC: **IR on the head, color sensor on the right arm**.
  In one stationary pose, the box was approximately 210 mm from the IR face
  and 150 mm from the color sensor. This is relative evidence, not a measured
  sensor transform from the drive origin.
- 2026-08-03 13:10 UTC: IR height approximately 300 mm.
- 2026-08-03 16:28 UTC: 130 mm from center to the outer edge of the right-hand
  color sensor, and 100 mm to the left without the removable propeller. The
  operator corrected an initial “130 cm” typo to **130 mm**. These are the
  extents saved in `config/ev3rstorm.json`, not IR sensor coordinates.
- 2026-08-03 16:45 UTC: front of the drive wheels approximately 50 mm ahead
  of the IR face. Wheel front is not the drive-axis origin.

On 2026-09-15 the operator confirmed that **IR is still in the head and the
color sensor is in the right hand**; the previous right-arm IR description
was a mix-up. The earlier assistant assertion that a missing right-arm IR
transform was the cause was premature. The planner now receives this
confirmed sensor layout alongside the measured asymmetric body, explicitly
distinguishing head-level sensing from full-body clearance. No numeric
sensor offset was recovered, and no mount offset or navigation rule has
been changed on that assumption. The old map's 210 mm
hazard offset is a provisional obstacle anchor, **not** a sensor mounting
measurement. Retain the measured asymmetric body in planning and simulation.

Separately, the map provider now distinguishes a reflection from the motor
stop threshold: raw 44 / 71 remain visible reflections; saturation 100 is
unknown, not free space. The UI displays clearer direction rays with raw IR
values, retains their historical observation pose, and places them above the
body overlay. It explicitly labels distance and obstacle width as unknown.
It does not create metric hit points, free cells, or invented box outlines.
Legacy `NO_NEAR_REFLECTION` data remain readable without hiding their raw
reflection values. The stopped physical snapshot was used for browser checks
at a separate read-only preview; no robot motion was started.

Mounting location is confirmed. A numeric head-to-drive transform remains
unmeasured: drive-centered rays are an explicit approximation, not verified
sensor-face coordinates, in the planner, map and simulator. This
display correction alone does not validate box navigation or edge finding.

Validation: 91 focused mapping, browser, contract and EV3 application tests
passed (`/private/tmp/ev3-ir-presentation-verified-20260914.log`); final
recorded-map browser rendering checked after reload. The production console
on port 8765 was **not restarted** and still holds the stopped run. Port 8879
is only the read-only recorded-map preview, with all robot controls disabled.

2026-09-15 follow-up: 58 EV3 body/session and shared-planner tests passed
after adding the confirmed sensor-layout facts. The stopped console was
started with the updated code; port 8765 now serves the revised IR display
and EV3 is `IDLE` with no episode started. No robot movement was requested.

## Code-audit finding: missing encoder continuity (2026-09-20)

Motor-free fault injection reproduced a concrete motion-accounting blind spot
in the current production EV3 path. `EV3NavigationBody.observe()` and the
sampling branch of `scan()` accept new absolute motor positions but do not
reconcile their change with the last accounted pose. `start()` similarly
replaces the observation when a worker is renewed without reconciling the
previous motor counters. `verified_motion_from_result()` checks deltas and
aggregates inside each slice, but not continuity of absolute positions
between slices or with the final stopped observation. The scan transport
validator also accepted that discontinuity before the correction below.

Three focused probes against the current code:

- Inject 10.032 degrees of simulated movement before an `observe` result:
  the result reports the changed motor counters, but host heading remains 0.
- Inject that movement at each of 13 scan samples: host reports a completed
  359.568-degree scan while the absolute counters correspond to 489.984
  degrees. The simulated final heading differs by 130.416 degrees. This is
  deliberate fault injection, not a new physical measurement or LLM result.
- Starting from a real traced scan-turn receipt, inject a 10.032-degree gap
  between its two slices, updating the absolute counters consistently while
  keeping each slice's own delta unchanged. Strict transport validation and
  odometry accept the result as complete and omit the additional turn.

Reproduction: `/private/tmp/ev3_code_audit_20260920.py`; outputs:
`/private/tmp/ev3_code_audit_20260920.jsonl`. No production code or calibration
was changed for this diagnostic step, and no physical movement was requested.

The focused correction now unifies absolute-counter continuity across motion
receipts, observations, sampling and worker renewal. A changed counter pair is
integrated once with the same odometry as a commanded receipt; it is not
counted as a new command, and the simulator now keeps absolute counters across
worker renewal as the physical EV3 does. Regression tests cover an observation
gap, movement during all 13 scan samples, and worker renewal. This prevents the
host map from silently losing measured motion, without adding route rules or
resetting the map to hide error.

The same audit found why EV3's full scan could look mechanically “stuttery”.
The previous 250 dps / 2560 ms turn profile made each recurring 30-degree step
853 ms long, so the 800 ms physical-pulse limit split it into two stopped and
verified pulses (427 + 426 ms). Scan profile v3 preserves the same calibrated
motor work but uses 267 dps / 2400 ms, making each 30-degree step one 800 ms
pulse. EV3 still stops at every bearing for the five-sample IR measurement;
only the unnecessary mid-step stop/start was removed. The simulator and worker
contract now agree on 12 single-slice turns for a full 360-degree sweep.

This still does not establish that the correction alone caused the earlier
operator-observed 520-degree turn: that run's absolute start/end counters and
logged sum matched to 0.066 degrees. A physical failing run is still needed to
connect this code defect to the intermittent overshoot.

## Full-scan residual correction removed (2026-09-21)

The next physical v3 trace identified the remaining deterministic overshoot.
`EV3NavigationBody.scan(full=True)` was recomputing a rounded residual from
the encoder-derived heading before every bearing. Small accumulated
under-reporting eventually produced a 45-degree correction, so the observed
sequence was ten 30-degree steps, one 45-degree step split into two 600 ms
pieces, and one final 30-degree step: 375 commanded degrees for a nominal
360-degree scan.

The full scan is now intentionally simpler: exactly twelve 30-degree commands
and thirteen stationary samples. It does not add a corrective turn based on
temporary odometry drift; the encoder-backed end pose remains the reported
pose. A regression test injects drift and verifies that no 45-degree or extra
pulse appears.

The physical 90-degree comparison then showed that the ordinary 90-degree
action was already visually correct, while the faster v3 scan profile was the
source of the full-scan over-rotation. The v5 calibration keeps the same
single-pulse cadence, uses 2,370 ms for the 90-degree reference and 657 mean
encoder degrees, and shares that encoder-to-angle reference with the normal
turn action. v4's 2,335 ms trial undershot the physical 360-degree sweep, so
v5 is the midpoint calibration to validate next.

## Isolated physical turn comparisons (2026-09-19)

With freshly charged batteries and unchanged code/calibration, three isolated
tests were performed. No navigation planner or subsequent route action ran:

| Test | Encoder-derived body angle | Operator-observed body angle |
| --- | --- | --- |
| One ordinary left-90 action | 87.582 degrees | Approximately 92 degrees; accepted |
| Production left full scan | 360.360 degrees | Approximately 370–375 degrees |
| Mirrored right full scan | 356.598 degrees right | Approximately 520 degrees right; failed |

The right test mirrored the production scan's 30-degree bearings, existing
15-degree command rounding, motor execution and stationary IR sampling in a
temporary diagnostic runner; production navigation code was not changed.
Both scans issued 12 turn commands, totaling 375 commanded degrees in their
respective directions (one 45-degree step corrected accumulated encoder
undertravel). Left raw motor deltas were B=-2746, C=2714; right deltas were
B=2703, C=-2699. Independent stopped sysfs readings after the right test were
still B=-699, C=686, matching its final receipt. There was no additional motor
rotation after the reported scan completion, and the arm did not move.

Read-only checks confirmed matching local/deployed SHA-256 hashes for the
worker, motor owner, hardware layer and scan profile. Both drive motors have
normal polarity, 360 counts per motor revolution, brake stop mode and the
expected opposite 250-dps setpoints. An offline assertion verified that all
supported right/left scan profiles have mirrored speeds and identical timing
and encoder targets, consistent between host and worker.

The physical right turn is a confirmed failure, not a successful 360-degree
scan. The underlying cause remains unresolved: motor encoders verify shaft
travel, not body yaw. Do not apply a global angle multiplier, conceal the
error by resetting heading, or blame Qwen. The earlier 2026-09-15 left scan
also had a reported large overshoot, so these observations do not yet prove a
purely right-direction defect. Next diagnostic: a watched ordinary right-90
action to compare with the accepted left-90 action before another full scan
or route. No further motion or calibration change was made during this audit.

Evidence: `/private/tmp/ev3-single-left90-20260919.jsonl`,
`/private/tmp/ev3-full-scan-20260919.jsonl`, and
`/private/tmp/ev3-clockwise-full-scan-20260919.jsonl`.

Execution-loop audit (same day, no further physical movement): the host
transport writes each request once and aborts on a timeout without replay;
the worker rejects duplicate request IDs. SCAN_TURN is excluded from the
ADVANCE/REVERSE encoder-recovery loop. Replaying both recorded command
sequences through the actual NavigationWorker, SupervisorMotorOwner and
RobotHAL against fake sysfs produced exactly 48 individual `run-timed`
writes per scan: 12 turns x 2 slices x 2 motors. No motor starts occurred
during sensor sampling or shutdown. This tests execution bookkeeping, not
physical turning accuracy. The 81 existing worker/core tests also passed.

Independent cumulative counter differences match the right scan's logged
angle to within 0.066 degrees, including the crossing of +/-180 degrees.
That does not explain the operator's additional approximately 160 degrees.
No competing Python/robot process was present in the later EV3 process
snapshot, but this is not proof of the complete process history.

The root cause is still open. For the next watched short right turn,
`/private/tmp/ev3_motor_audit_worker_20260919.py` is a temporary wrapper for
the installed worker. It records request IDs and every motor-interface
write (including `run-timed`, settings and `stop`) in memory, then saves a
trace after worker cleanup. It issues no requests itself and changes no
calibration or production module. Desktop checks cover success/failure
cleanup, unchanged single starts and Python 3.5 syntax. The isolated
motor-write audit lives at `/private/tmp/ev3_audit_motor_starts_20260919.py`.

Physical traced follow-up (same day): one ordinary RIGHT_90 action was
observed by the operator as a perfect 90-degree turn. Its encoder estimate
was 89.034 degrees. The brick-side trace contains one pulse request, exactly
eight successful motor starts (four paired slices, 800/800/800/160 ms),
and no other motion request. Raw deltas were B=663, C=-686.
Trace: `/private/tmp/ev3-motor-audit-hbhty3hb.json`.

The next traced right full scan completed 12 turns and 13 samples with a
363.858-degree encoder estimate. The operator did not watch the movement but
reported an approximately unchanged final location/orientation; the complete
physical revolution remains unverified. The trace contains exactly the
expected 44 individual motor starts:
two 15-degree turns used one paired slice each, the other ten turns two
paired slices each. All starts belong to their corresponding scan request;
sampling and shutdown issued no starts. No failed motor writes or recorded
request errors. Trace: `/private/tmp/ev3-motor-audit-qlfl3gui.json`; host
record: `/private/tmp/ev3-clockwise-full-scan-audited-20260919.jsonl`.
SSH process close reported exit 255 after successful shutdown. Independent
read-only checks confirmed the motors remained stopped at the final receipt
positions B=2741, C=-2736, and the complete trace was recovered. No command
was replayed. The close error remains separate from the physical angle
question; do not call this run physically validated without operator feedback.

## Observed retry: wrong physical scan angle (2026-09-15)

Episode `episode-bd7452afa8c835a6c74da827` ran unchanged code after the operator
returned EV3 to the start. The operator watched and reported a full turn plus
another half-turn, ending directly away from the box. The scan's odometry
reported 357.588 degrees, ending at (-2, 1) mm, heading -2.412 degrees.

Independent stopped sysfs readings bracket the run: outB -1526 -> -4300,
outC 9328 -> 11972. Their opposed mean is 2709 encoder degrees. Multiplying
by the configured 132 millidegrees gives exactly the recorded 357.588
degrees. No extra unaccounted encoder rotation is needed to explain the
record. The operator's approximately 540-degree physical turn therefore
contradicts the current motor-to-body angle calibration. Its inferred scale
is approximately 1.5 times the configured one; this is a diagnostic estimate,
not a newly verified calibration. Both the worker turn targets and host
odometry must agree on a physically checked scale before another route test.
Do not merely rotate the drawing or zero the scan's end heading.

Historical cross-check after the operator reported that this had only started
today: `/private/tmp/ev3-repeat-full-scan-20260913.jsonl` records the scan the
operator previously described as visually correct. It used 250 dps, 12 turns
and reported 357.786 degrees, equivalent to 2710.5 opposed encoder degrees.
Today's stopped motor counters give 2709 degrees: almost identical travel.
The worker turn profile and its host contract have no working-tree changes;
their turn calibration/speed dates to commit `8c3a254` (2026-08-03). The current
odometry diff adds scan-turn receipt support but does not change its angle
formula. Thus there is no evidence of a newly commanded extra half-turn or
a changed motor-speed constant. The physical observations conflict despite
essentially identical encoder travel. The cause (execution conditions,
mechanical conversion/slip, or an observation/reference mismatch) remains
unresolved; do not apply a 1.5 calibration multiplier without a watched,
isolated turn check. This narrows the diagnosis but does not discount the
operator's observed wrong final heading.

Qwen subsequently selected a left detour, but the worker response timed out
before any post-scan motion was logged. The stop request acknowledged this
already-terminal fault, leaving `IDLE`. This transport failure remains separate
from the observed angle mismatch. No motor or calibration changes were made.

Evidence: `/private/tmp/ev3-observed-box-retry-20260915.jsonl`,
`/private/tmp/ev3-observed-retry-events-20260915.json`, and the baseline/stopped
`ev3-observed-retry-*-20260915.txt` files. This retry also used the overly
informative mission below; it is not a clean autonomous-obstacle test.

## Physical check: reported goal contradicted by reality (2026-09-15)

The operator chose to run without raising the box: approximately 300 mm from
the head IR sensor, with the box top at sensor height. The mission supplied
these operator observations, not a prescribed side or path. That still gave
Qwen privileged obstacle information (color, approximate distance, height
and open-side assumptions) and was a test-design error. Do not repeat that
mission prompt. No navigation, motor, sensor or planner code was changed
during this test.

- First attempt `episode-f53d89a6f39ec50052d34336` completed its full scan
  (encoder total 362.34 degrees). Qwen proposed a 200 mm left detour, but a
  worker-response timeout occurred before any post-scan motion was recorded.
  This was a runtime failure, not an obstacle decision. Independent stationary
  preflight then passed in 18.2 s, with no motion commands and all motors stopped.
- Retry `episode-5d53fd30f40be9c842e052a7` completed in the live application:
  `IDLE`, terminal reason `completed`, no runtime error. Qwen low / 4,096 output
  tokens chose a 300 mm left detour, forward to approximately x=800, then back
  to the goal line. Nine decisions included completion. The original goal
  remained (800, 0) throughout route adjustments.
- The retry used one full scan (encoder total 360.03 degrees), with no further
  surveys. One unusable model response (`LMStudioProtocolError`) was retried
  without losing the route or ending the mission. Final odometry was
  (785, -21) mm, heading -5.448 degrees: approximately 26 mm from the goal.
- Speech timed out during the retry; later utterances reported
  `speech_session_aborted`. Navigation continued and finished. Speech recovery
  and worker reconnect recovery remain unresolved; this is not a fault-free
  end-to-end result.
- The operator subsequently reported EV3 approximately 300–400 mm to the
  left of the start, facing left, at roughly its original distance from the
  box, not at the reported goal. The actual trajectory was not watched.
  **Physical navigation acceptance failed.** The odometric completion above
  is not evidence that EV3 physically reached the goal or passed the box.
  Installed worker/config hashes matched the checkout; stopped drive encoders
  read outB=-1526, outC=9336. These are motor-shaft positions, not independent
  evidence of body position or heading. The cause remains unresolved.
- The operator then returned EV3 to the original position in front of the
  box. Do not compare that manually reset physical pose against the old map.
- IR values varied substantially between scans. The model still made guesses
  about box width and clearance; the actual box contour is not measured by
  this qualitative IR view. The current EV3 publisher supplies no positioned
  hits, obstacle hypotheses or coarse grid. This is an integration gap, not
  merely hidden markers. A tested IR range estimate is needed to supply the
  common map with explicitly approximate hits; do not restore the retired
  deterministic detour engine or silently label raw proximity as millimetres.
  Raw IR numbers were removed from the canvas following operator feedback;
  they remain in hover text and sensor details. This is only a display cleanup,
  not completion of the requested BLAST-equivalent obstacle map.
  The 58 presenter/web tests passed after the cleanup. The idle console was
  restarted to load the updated assets; no robot movement was issued. Its
  old run map was cleared by that restart, consistent with the manual reset.

Evidence: `/private/tmp/ev3-head-height-box-20260915-runs.jsonl` and
`/private/tmp/ev3-head-height-box-20260915-control-events.json`. The main
navigation diagnostic log also retains planner requests/responses. The live
map on port 8765 was checked with EV3 selected and visible raw IR labels.

## Previous physical check: white box, insufficient first detour (2026-09-14)

Episode `episode-061516c548d06e8ffa6faa58` ran the current shared-body
context with Qwen low reasoning / 4,096 output tokens in the live console.
No onboard code or motor calibration was changed. Stationary IR was 44.
Startup survey completed; encoders reported 356.994 degrees, ending at
(-4, 2) mm and -3.006 degrees. Physical heading agreement is unconfirmed.

Qwen recognized the frontal reflection as consistent with the white-box
reference and explicitly considered EV3's 230 mm width / 181 mm turn radius,
but chose (0, -150), (800, -150), (800, 0). The operator confirmed that the
15 cm rightward detour was nowhere near clearing the box edge. The second
leg returned `near_obstacle` at estimated (139, -174) mm, -8.418 degrees;
IR had strengthened to 29. The host operator stop then ended the episode
cleanly (`IDLE`, terminal reason `stopped`, no runtime errors). All three
motor states were independently confirmed stationary. Actual body contact
has not yet been confirmed by the operator.

The pending Qwen response requested `SCAN_FRONT_ARC` to find the obstacle's
extent and retained the route. It arrived after cancellation and was not
executed. Thus the first detour was inadequate, but recovery success/failure
was **not tested**; this was not an automatic mission abort or scan loop.
Evidence: `/private/tmp/ev3-white-box-physical-20260914-01.jsonl` and the
matching request/response records in `local-artifacts/navigation-dashboard.jsonl`.

The operator also confirmed the map still does not communicate box
reflections. The renderer deliberately has no metric IR hit endpoint;
additionally, the map provider still labels raw 44 as `NO_NEAR_REFLECTION`
based on the motor gate, although the planner now receives the raw evidence.
Do not mistake that label for absent reflection, invent box dimensions, or
fix this by hardcoding a wider detour. The next bounded work should make
reflected-strength evidence visible and validate discovering the edge of
an obstacle whose width is not known. No navigation/UI implementation was
changed during this physical test.

## Previous checkpoint: shared body geometry for planning (2026-09-14)

Both planner contexts now use `RobotFootprint.planner_view()` from their own
existing calibration. It retains the measured asymmetric extents and exposes
length, width and the in-place turn's swept radius around the drive origin.
EV3: 200 mm long, 230 mm wide; BLAST: 170 mm long, 205 mm wide. The existing
10 mm margin is unchanged. The derived turn radius includes that margin and
must not be added again to already-expanded metric keep-outs. No motion
calibration, collision thresholds, side choice or route generation changed.

An intermediate dark-box run showed Qwen explicitly using the untriggered
motor gate to justify advancing despite a possible close reflection. Removed
gate status from the planner's IR map/observation: raw/filtered sensor readings
are world evidence; execution owns the unchanged gate. Available actions still
respect it. This replaces the earlier `forward_motion_blocked` presentation
described in the historical checkpoint below.

Real-Qwen validation (low reasoning, 4,096 tokens, production EV3 body/session):

- Initial white-box check with shared dimensions: reached the goal without
  contact, choosing a left offset of 250 mm. Artifact:
  `/private/tmp/ev3-shared-body-white-20260914.jsonl`.
- Intermediate dark-box check **failed** while gate status was still in the
  context; repeated contacts and the time limit remain recorded in
  `/private/tmp/ev3-shared-body-dark-20260914.jsonl`.
- Final dark-box check after removing gate status: Qwen chose a right detour
  at y=-300, passed the box, returned to center and reached the goal. Five
  decisions, 20 mm remaining, no contacts. Artifact:
  `/private/tmp/ev3-shared-body-final-front-dark-20260914.jsonl`.
- Final front-and-right-box check: Qwen chose left, followed the retained
  route and reached the goal. Five decisions, 20 mm remaining, no contacts.
  Artifact: `/private/tmp/ev3-shared-body-final-front-right-white-20260914.jsonl`.

These are individual successful simulations, not a measured success rate.
The IR transfer, sensor origin and contact-model limitations below still apply.
Next: short physical EV3 white-box route, verify real/map direction and obstacle
avoidance. No new physical run, dashboard restart or onboard deployment was
performed at this checkpoint. The physical heading discrepancy is not resolved
by this planner-context change. BLAST's existing execution/clearance is intact;
its live model behavior has not been rerun for this new context.

Validation: 181 focused tests passed. The final full regression rerun passed
**1,979 tests in 136.2 seconds**, including both robot paths:
`/private/tmp/shared-body-final-quality-rerun-20260914.log`.
The preceding full run had intermittent STT event-order and supervisor
`FAULT_LATCHED` failures (`/private/tmp/shared-body-final-quality-20260914.log`).
No STT, supervisor, timing limits or assertions were changed to obtain the
green rerun; the cause of that intermittent failure is not established here.

## Current checkpoint: live application connected; physical acceptance pending

EV3 navigation is now enabled in the combined console. A thin application
adapter connects the existing EV3 body/session to the shared Qwen planner,
mission controls, map publication and speech lifecycle. No host-authored route
or preferred turning side has been introduced; BLAST keeps its execution path.

- The map shows the measured pose, fixed goal and model-authored waypoints.
  IR observations retain their original observation pose across later scans
  and movement. They remain qualitative directional cues, not invented metric
  obstacle positions or ultrasound rays.
- Speech uses the same episode service as BLAST. Playback failure does not
  cancel navigation. Model retries retain the mission and route. Cancellation
  while waiting for a model cannot start the returned movement command.
- The simulator now runs the actual application adapter, including live map
  updates, runtime status and the deployed worker receipt/budget validation.
  Qwen supplies all routes. Physical audio is replaced by a test player.
- Both production and the validation script now default to low reasoning,
  4,096 output tokens and a 60-second planner timeout for EV3. BLAST retains
  its existing output limit. No concurrent model calls were used in these trials.

Full-application Qwen validation:

| Scene | Outcome | Evidence |
| --- | --- | --- |
| Open floor | Clean completion, 20 mm from goal; two decisions, all workers stopped | `/private/tmp/ev3-live-application-open.jsonl` |
| Boxes ahead/right | Completion, 43 mm from goal; eleven decisions, all twelve workers stopped | `/private/tmp/ev3-live-application-box.jsonl` |

**The box run is a recovery result, not a clean pass.** The initial lateral
clearance was too small; three blocked physics events occurred before Qwen
reversed and widened its route. These two trials used a 120-second timeout
cap before it was aligned with the console's 60-second default; observed
responses remained below that production cap. Simulation still simplifies IR,
wheel slip and contact. It cannot establish physical clearance or map accuracy.

The full regression gate passed **1,970 tests**, including BLAST regressions,
application map normalization, shared speech, and cancellation before movement
after a model wait. Final rerun after configuration alignment: **1,970 tests
passed in 82.4 seconds**, `/private/tmp/ev3-live-final-quality.log`.

### Physical clear-floor checkpoint

EV3 reconnected and passed the stationary preflight with zero motor commands
(`/private/tmp/ev3-stationary-preflight-20260913.json`). With fresh operator
placement approval, episode `episode-525a7b5ba8e1dd02ca91e248` then ran through
the live console. The startup survey measured 358.710 degrees total rotation.
Qwen selected checkpoints at (400, 0) and (800, 0), followed both legs without
extra surveys or execution errors, and completed. Three model responses took
8.602, 5.430 and 4.550 seconds with low reasoning. Final encoder pose was
(804, -23) mm, heading -2.346 degrees; runtime returned IDLE with speech complete.
Evidence: the episode's records in `local-artifacts/navigation-dashboard.jsonl`.

The operator confirmed arrival about 80 cm ahead and the spoken announcement,
but reported roughly 5 degrees of leftward deviation. The encoder estimate is
slightly rightward. **Arrival and speech are physically confirmed; heading/map
agreement is not yet accepted.** Establish the source of this discrepancy
before using a box run as acceptance evidence.

The live display also revealed unfinished EV3 presentation: shared trace labels
still mention BLAST/IMU, and repeated fixed-screen-length IR wedges overlap.
These wedges showed observation direction, not reflection endpoints. The
display-only cleanup now removes triangles, observation-origin circles and
repeated labels, retaining thin dashed direction lines with hover details.
Both robots retain the same path, goal, waypoint and footprint renderer. The
heading identifies the actual robot; EV3's legend does not advertise IMU or
ultrasound. The IR context still preserves raw proximity values and original
observation poses without converting them to metric ranges.

Display validation: all 1,970 tests passed in 112.9 seconds
(`/private/tmp/ev3-map-cleanup-quality.log`), including shared-renderer and
BLAST hit/ray regressions. The cleaned view was visually checked with the
captured physical run (`/private/tmp/ev3-completed-run-map-20260913.json`).
No sensor, planning or motor behavior changed in this presentation cleanup.

Robot-to-robot coordination remains a separate acceptance milestone.

### Repeat survey and conversational saw gestures

The operator requested a repeat full survey after suspecting 5–10 degrees of
overshoot. The unchanged production scan used 12 turns and 13 samples, measured
357.786 degrees, and returned to within 2.214 encoder degrees of its start.
The operator confirmed that this repeat was "perfect", starting and stopping
more or less exactly. Evidence: `/private/tmp/ev3-repeat-full-scan-20260913.jsonl`.
No scan-speed or calibration change is justified by that repeat. This does not
yet resolve the separate travel-heading/map discrepancy described above.

EV3 conversation now has hardware-scoped model choices for a one-second saw
flourish or a two-second celebration, beside speech. It reuses
`robot_cli.py motor-test` for the existing encoder-checked arm role on outA.
The operator explicitly approved permanent arm limits of 1,560 dps / 2,000 ms;
1,560 dps is the attached medium motor's reported maximum speed. Each gesture
is one timed spin, not a sequence of small remote commands. Drive limits and
turn calibration are unchanged. No navigation engine or keyword-to-emotion rules were
added. A shared controller lease prevents gestures from overlapping navigation;
a new mission waits cancellably for an ongoing short accessory command instead
of failing as busy. Accessory failure does not discard speech. BLAST retains its own claw/face
choices. EV3 display emotions and navigation-triggered gestures are not part of
this checkpoint. Full-speed physical execution is verified below; the operator's
assessment of the stronger gesture's appearance is still pending.

The real Qwen conversation classifier selected `CONVERSE` and `saw_celebrate`
in 1.55 seconds for a celebration request, with a grumpy spoken reply and no
fallback. This was a model-only check, not physical validation. The robot's
existing `robot_cli.py` and `robot_hal.py` checksums match the checkout; no new
onboard program was needed. Focused checks cover hardware-scoped model choices,
verified arm receipts, cancellation, busy navigation and speech preservation.

Validation gate: **1,976 tests passed in 96.1 seconds**
(`/private/tmp/ev3-social-accepted-quality.log`). The refreshed live console
also accepted a celebration through its real `/turns` endpoint, selected
`saw_celebrate` and queued speech (`/private/tmp/ev3-saw-live-20260913-02.json`).
The operator heard the speech but saw only a small movement. Investigation
confirmed a 60-degree arm movement, with 10.575 seconds needed to start Python,
perform the 400 ms action and return its receipt. This exceeded the old 10-second
host timeout. The host now allows 30 seconds for startup/receipt while the motor
itself is locally limited to one or two seconds.
The first live wording added "Stay where you are; no driving" and Qwen wrongly
classified that as unsupported. The shorter request succeeded. This is a
remaining classifier wording issue separate from the arm timeout, and must not
be hidden by a hardcoded gesture-selection fallback.

Full-speed live check: Qwen selected `CONVERSE` / `saw_celebrate` through the web
application and speech played before the single two-second command. OutA moved
from 119 to 2,700 degrees (2,581 degrees, approximately 7.2 motor revolutions)
and stopped. Both drive encoders were unchanged: outB -3,118 and outC 7,723.
The kernel briefly reported `running overloaded` while trying to maintain its
maximum rated speed under load, then returned to an empty stopped state; no
gesture timeout or playback error was logged. This is an actual movement check,
not a claim that the loaded motor maintained exactly 1,560 dps throughout.
Evidence: `/private/tmp/ev3-saw-fullspeed-live-20260913-01.json` and
`/private/tmp/ev3-saw-fullspeed-encoders.log`. The robot configuration checksum
matches the checkout; its previous configuration was preserved alongside it as
`ev3rstorm.pre-saw-fullspeed-20260913.json`.

After the approved arm-limit change, tests that assumed the old limits were
updated while retaining rejection checks above the configured maximum. The
final full gate passes **all 1,976 tests in 98.9 seconds**
(`/private/tmp/ev3-saw-fullspeed-final-quality.log`).

## Evidence history: execution feedback and real worker budgets

### First physical box attempt: not accepted

Episode `episode-f3f6b36f9bda30cb0e16749d` used the unchanged navigation code,
one front box, both sides open, and the fixed goal 800 mm ahead. Stationary
preflight passed. The complete startup survey recorded 25 IR observations,
raw values 64–95, all represented as `NO_NEAR_REFLECTION`. Qwen incorrectly
treated this as a clear route and selected straight checkpoints (400,0)/(800,0).

During approach, raw proximity fell from about 80 to 63 at encoder x=214 mm,
40 at x=318 mm, and 30 at x=369 mm. The configured entry threshold is 35,
with a three-sample median and two confirming near samples. The operator
reported contact with the dark box and 1–2 further wheel movements; the sensor
was approximately 2 cm from the box when detection occurred. Encoder travel
after contact is not proof of actual body displacement. Qwen did then select
a front-arc scan, but this is late recovery, not a successful box detour.

The drive was stopped for diagnosis. All three motor states were independently
checked empty/stopped over SSH. The application ended `FAULTED` with
`robot_runtime_failed` after stopping during the scan; this separate stop-path
failure is not yet explained. No navigation code or IR thresholds were changed
during the run. Saved map: `/private/tmp/ev3-one-box-stopped-map.json`; startup:
`/private/tmp/ev3-one-box-start-20260913.json`; events remain in the navigation
diagnostic log under the episode ID.

The live browser check also confirmed that EV3 currently renders only
qualitative direction lines, with zero metric sensor rays/hit endpoints.
Missing hit markers are therefore a presentation/data-contract limitation,
not proof that IR observations were lost. Before another drive, compare the
same box at a roughly known stationary sensor distance against a light target.
The old recorded dark-target calibration does not match this encounter; do
not assume a new threshold or invent metric IR hit positions from one contact.

Stationary follow-up: the operator placed the same dark box approximately
20 cm in front of the sensor. Direct read-only sysfs sampling confirmed
`lego-ev3-ir`, port in4, mode `IR-PROX`; nine readings were 71 and one was 70.
These stable readings do not agree with the old recorded dark-target examples
and remain above the current gate threshold. A same-distance light-surface
comparison is requested; no threshold was changed from these data alone.
Artifact: `/private/tmp/ev3-dark-box-20cm-20260913.log`.

White-box follow-up (2026-09-14): after EV3 rebooted, the operator reported a
white box approximately 30 cm ahead. Ten stationary, read-only sysfs samples
in confirmed `IR-PROX` mode on in4 were 51, 50, 51, 50, 52, 51, 52, 50, 51, 50.
The median is 51, still above the near-obstacle entry threshold of 35 and
different from the old light-target 30 cm example (25). This is stable sensor
output, not dropped readings. Surface, distance and reboot all differ from
the preceding dark-box test, so this does not isolate the cause. Request a
same-distance comparison before drawing that conclusion. No motors, sensor
settings or navigation thresholds were changed.
Artifact: `/private/tmp/ev3-white-box-30cm-20260914.log`.

After the request to move the white box to approximately 20 cm from the IR
sensor, the operator said to check again. All ten samples were 44, with in4,
`lego-ev3-ir` and `IR-PROX` confirmed again. The distance is operator-set, not
independently measured. Moving this target closer changed the stable reading
from 50–52 to 44, but still did not meet the current near-obstacle threshold
of 35. The earlier dark-target reading at approximately 20 cm was 70–71;
these observations do not support treating raw IR values as a universal
distance scale. No motor commands, threshold changes or navigation changes.
Artifact: `/private/tmp/ev3-white-box-approx20cm-20260914.log`.

### IR evidence and simulator checkpoint — 2026-09-14

The planner now receives `forward_motion_blocked` instead of the misleading
`near_obstacle` label for the motor gate, along with raw/filtered readings and
the measured white/dark reference examples. An untriggered gate is explicitly
not clear-space evidence. The gate thresholds themselves remain unchanged;
no distance conversion, preferred side or host-generated detour was added.

The EV3 simulator no longer derives its gate directly from an ideal 210 mm
world-distance threshold. It uses the onboard `InfraredObstacleGate` with the
checked-in configuration, startup/scan sampling and polling during movement.
Piecewise-linear response profiles include the white 200/300 mm anchors
(44/51) and dark 200 mm anchor (71). Their other endpoints/interpolation are
assumptions, not measured calibration. The ray still originates at the drive
origin; beam width, surface angle, sensor mounting offset, leg slip and real
contact/encoder behavior are not reproduced. Physics contact remains a test
failure even when the model later recovers. Never call this full hardware
validation.

Removed the unused alternative active-scan simulator adapter and its lone
test; current survey tests exercise `EV3NavigationBody.scan`. Fixed partial
multi-slice simulation receipts to report only their executed prefix, including
an immediately blocked repeated turn, rather than completed zero-motion slices.

Qwen also exposed an execution-contract mismatch: the common prompt defines
`ADVANCE` as current-heading progress, but EV3 redirected it toward the retained
waypoint. Removed that EV3 special case. `FOLLOW_WAYPOINT` still executes a
whole leg with feedback between pulses; explicit recovery `ADVANCE` is bounded
and retains the route. A regression checks recovery perpendicular to the goal.

Real-Qwen checks (low reasoning, 4,096 output tokens; no physical motors):

- White-box first run: recognized the reflection and chose a detour, but hit
  the rear corner when turning; time limit, 293 mm from goal. Artifact:
  `/private/tmp/ev3-white-ir-qwen-20260914.jsonl`.
- White-box repeat: approached too close before turning, recovered backwards,
  then chose insufficient lateral clearance. Exposed the partial-receipt bug
  corrected above; **not** a successful route. Artifact:
  `/private/tmp/ev3-white-ir-qwen-final-20260914.jsonl` (filename does not imply
  acceptance).
- Dark-box run: recognized raw 71 as possible nearby obstruction and chose a
  lateral waypoint, but only 150 mm of offset. Body contact followed; stopped
  at the explicit four-decision test limit, 737 mm from goal. Artifact:
  `/private/tmp/ev3-dark-ir-qwen-20260914.jsonl`.
- Open floor after the corrections: completed with two Qwen decisions, no
  contacts, 20 mm from goal, startup survey only. Artifact:
  `/private/tmp/ev3-open-ir-qwen-20260914.jsonl`.

The data-contract defect is corrected; box navigation is **not accepted**.
Next isolate the remaining body-clearance misunderstanding in model context
and reassessment, without inventing obstacle dimensions or adding a host route
planner. Physical gate calibration and the independently reported heading
discrepancy remain unresolved. These changes have not restarted the live app
or deployed a new worker/configuration to EV3.

Final code regression gate: **1,978 tests passed in 86.0 seconds**, including
BLAST regressions, weak-reflection misses, retained recovery routes and partial
turn receipts. Artifact: `/private/tmp/ev3-ir-final-quality-20260914.log`.
These are code regressions, not 1,978 successful model-driven routes.

The operator again reported slight leftward travel, never the rightward drift
drawn on the map. The renderer's +y-up mapping is consistent with the episode's
+y-left frame; it draws the negative y supplied by odometry. The initial survey
ended at estimated heading -2.742 degrees. Seven forward commands then changed
the estimate to -5.646 degrees and position (369,-24) mm. Thus -2.742 degrees
of the apparent rightward heading was already present before forward travel,
with -2.904 more accumulated from encoder differences. This identifies the
source of the plotted deviation, not its physical cause. The current formula
uses (right encoder delta - left encoder delta) / 2 times the turn calibration,
with B configured left and C right. Physical side mapping, calibration and
leg/ground slip remain to distinguish; EV3 has no independent gyro heading.
Do not mirror the renderer or treat the provisional encoder yaw as verified
body yaw. No motion or odometry changes were made for this diagnostic check.

At this earlier checkpoint, the replacement used the same Qwen planner as
BLAST, but EV3's live navigation endpoint was still disabled. The evidence in
this section concerns the core session, before application integration.

- Each execution result now distinguishes an attempted turn from translation,
  with requested/measured rotation and measured travel. Qwen previously
  interpreted a blocked alignment turn as a blocked path on that entire side.
  No preferred side, route generator or automatic detour was added.
- The EV3 body owns renewal of its bounded worker. It releases the stopped
  worker while waiting for Qwen and opens a new one for the next observation
  or movement. Time/request/movement-budget renewal preserves pose, survey,
  mission and route, and never replays a motion with a missing receipt.
- The simulator now enforces the deployed 180-second, 256-request, 40-slice
  and 32-second motion budgets, instead of reporting permanently full budgets.
  Accelerated motor time and real model waiting both consume worker time.
  Regression tests compare these limits with the actual worker profile.
- Following the final goal waypoint now also finishes the mission's requested
  heading in place. Position arrival does not require another route decision
  just to complete that alignment.

Actual Qwen trials use low reasoning, 4,096 output tokens, one request at a
time, and no supplied route. After the competing game workload stopped,
initial responses were about 7–15 seconds, rather than the earlier 60–120
second timeouts. This is observed timing, not a controlled speed benchmark.

- `/private/tmp/ev3-free-host-front-right.jsonl`: baseline stopped after it
  reproduced the ambiguous-motion feedback problem; not a successful run.
- `/private/tmp/ev3-motion-feedback-front-right.jsonl`: full left detour,
  20 mm from the goal, no blocked physics events. This preceded enforcement
  of the real worker budgets and the final-heading correction.
- `/private/tmp/ev3-bounded-worker-front-left.jsonl`: full right detour and
  completion, 24 mm estimated distance remaining, with real worker budgets.
  **Not a clean route:** two blocked physics events occurred before recovery.
- Final-code rerun: `/private/tmp/ev3-final-front-right.jsonl`.
  Completed the goal in nine model decisions, 42 mm estimated distance
  remaining, after reversing and choosing a wider left detour. **Not a clean
  route:** blocked movement occurred before recovery. All nine worker
  sessions shut down correctly; no worker-budget or transport failure.
- Final regression gate: `/private/tmp/ev3-final-execution-quality.log`:
  **all 1,967 tests pass** (82.1 s), including BLAST regressions, actual-worker
  limit parity, renewal without pose/route reset, model waiting without an
  expiring worker, partial-motion feedback, and final heading in place.
  `git diff --check` passes. No physical motor commands in this checkpoint.

Simulation still uses simplified IR response, contact and wheel behavior.
Successful arrival does not establish physical navigation or reliable
obstacle clearance on every model attempt. Do not hide contact events in a
success count.

At that checkpoint, the next implementation step was the thin live application adapter:
mission cancellation, retained map observations/route display and speech,
using the existing shared services. That wiring is now complete as described
above; the physical survey and clear-floor acceptance remain pending.

## Evidence history: earlier checkpoints

### Checkpoint 1: cleanup and shared planning input

The user approved a temporary EV3 autonomous-navigation gap and explicitly
approved removal of the old engine and dedicated tests. The removal is now done:

- Removed 14 legacy modules: the physical-navigation runtime/adapter and its
  motion, scan, plan-tail and route orchestration; experience/context builders;
  short-plan tail; local detour controller, route generator and collision
  snapshot; old LM Studio navigation planner and its benchmark.
- Removed their nine dedicated test files and tests/helpers for the retired
  engine from mixed files. Kept SSH transport, worker cancellation, encoder,
  odometry, active-scan, shared map and BLAST coverage.
- Removed unused EV3 runtime timing/scan-budget constants. Actual worker motor
  limits, hardware calibration and shared geometry remain.
- BLAST map imports no longer load EV3 detour planning. Display constants live
  in the shared spatial-map contract.
- The legacy code is recoverable from Git. Previous uncommitted work was saved
  in `/private/tmp/ev3-before-cleanup-20260912.patch`.

EV3 remains selectable for conversation, speech and hardware checks, but its
navigation endpoint is explicitly disabled. This is intentional until the
replacement executor is validated. The GUI permits chat without enabling
movement, and Qwen receives the temporary capability limitation.

The first replacement boundary is implemented:

- `EV3NavigationBody` uses the existing SSH worker protocol for sensing and
  movement receipts. Initial full surveys cover nominal 0..360 degrees; front
  surveys cover -90..90 and return. Actual encoder heading is retained rather
  than snapped to the requested angle. Missing stopped samples remain unknown,
  without deleting prior samples or restarting the sweep.
- `ev3_navigation_context` supplies the fixed goal, measured pose, full angular
  IR observations, physical footprint, retained waypoints and history to the
  **same `ControllerActionContext` / `LMStudioControllerActionPlanner` used by
  BLAST**. It does not turn IR into metric obstacles or generate a route.
- Waypoint bearing/axis arithmetic is extracted from BLAST into
  `waypoint_geometry` and reused by EV3. BLAST keeps its existing call sites
  and behavior. The shared planner prompt explains the IR units.
- The shared-world EV3 drive adapter now includes temporal motor segments,
  matching the real worker receipt format. Tests validate scan and drive
  responses with the actual SSH receipt validator.

### What has been validated

The cleanup passed all 1,949 remaining tests before the new boundary was added.
Focused integration checks passed 175 tests, including BLAST episode regression
coverage, full scans with front/left and front/right boxes, unknown samples,
front-half scan restoration, measured forward motion and cancellation/shutdown.
The final full quality gate passes all 1,953 tests (93.3 s); local import checks
find no missing module targets. The 14 deleted engine modules contained 9,276
lines. The three new shared/hardware boundary modules total 199 lines.

Three real-Qwen checks with low reasoning used the production EV3 body/context
code against simulated worker responses:

| Scene | Qwen's first decision | Model latency |
| --- | --- | --- |
| Open floor | Advance toward the fixed goal | 5.0 s |
| Boxes ahead and right | Follow a waypoint to the left | 17.5 s |
| Boxes ahead and left | Follow a waypoint to the right | 13.8 s |

These are **perception-to-first-decision checks, not completed navigation runs**.
No route was executed in those Qwen checks. Proposed lateral distances of
150–200 mm remain hypotheses: correct side selection does not prove adequate
body clearance or passage beyond the box. The next checkpoint must execute,
observe and reassess those legs. No new physical motor commands were issued.

## Checkpoint 2: waypoint execution built; obstacle acceptance still open

`EV3NavigationSession` now connects Qwen decisions to the EV3 body. The shared
waypoint geometry module owns arrival, heading-to-leg resolution and removal
of reached waypoints; BLAST uses these same functions. Its thresholds and
behavior are unchanged. There is no EV3 detour generator or preferred side.

- A selected leg continues through sensor-checked worker pulses, returning to
  Qwen at arrival, a nearby obstacle, partial movement or lack of progress.
- The fixed goal and remaining model-authored waypoints survive scans,
  stopped partial movement and model timeouts. A model transport/protocol
  failure issues no movement or scan; a later step retries the retained state.
- EV3's arrival tolerance accounts for half its fixed drive pulse (27 mm with
  the current calibration). Requiring 25 mm could leave a waypoint between two
  stopping positions and cause repeated corrections. BLAST remains at 25 mm.
- The simulator now honors EV3 IR blocking before and during forward travel.
  Interrupted pulse receipts correlate slice/segment status and preserve valid
  partial encoder evidence. Tests use the actual SSH receipt validator, not
  a more permissive test-only decoder.

Validation distinguishes execution tests from model planning:

- Nine focused body/session checks cover full/front surveys, missing samples,
  multi-pulse advancement, retained route tails, a target between fixed pulse
  endpoints, IR denial/interruption, model-timeout retry and shutdown.
- The full quality gate passes 1,958 tests, including BLAST regressions.
- Real Qwen with low reasoning completed the simulated open-floor mission in
  three decisions, ending 20 mm from the goal with matching estimated/world
  position and no blocked physics events.
- The first box attempt chose the open side but insufficient lateral clearance;
  its interrupted receipt exposed the simulator mismatch corrected above.
  A subsequent attempt enlarged its lateral target from 200 to 350 mm before
  the next model request timed out. The mirrored case timed out before its
  first decision. These are **not successful obstacle-route validations**.
- A final bounded retry check uses low reasoning and 4,096 output tokens. It
  ended after the time budget with one completed lateral leg (156 mm left,
  815 mm from the goal) and repeated model timeouts. Estimated/world poses
  agreed, no blocked physics events occurred, and the remaining route survived
  without extra scans. It does not establish a successful detour.

Reproduction: `PYTHONPATH=src .venv/bin/python scripts/validate_ev3_navigation.py
open` (or `front-right` / `front-left`). JSONL output records the actual model
context, returned reasoning, route, measured execution and ground-truth result.
Each invocation checks a twelve-attempt/five-minute budget between model calls
(an in-flight call can extend that wall time). These are test budgets, not new
production mission-abort rules. Diagnostic artifacts for this
checkpoint are under `/private/tmp/ev3-qwen-*-route*.jsonl`, with the full gate
at `/private/tmp/ev3-session-complete-quality.log`.

The session is deliberately **not wired into the physical dashboard endpoint**
yet. Worker lifetime/renewal, mission cancellation and map/speech presentation
still need the application adapter. The simulator's IR transfer function and
encoders are simplified, and its process budgets do not yet model real worker
expiry. Therefore these results do not qualify a physical EV3 mission. No
physical motor commands were issued during this checkpoint.

## Follow-up: correct the model's evidence contract before more route rules

LM Studio's response logs exposed two concrete context problems. After a
turn, Qwen repeatedly interpreted old survey bearings as current robot-relative
directions. Also, EV3 was receiving the metric-map explanation used by BLAST,
including the claim that execution checks body clearance against mapped echoes.
That claim did not apply to EV3's bearing-only IR evidence.

Corrections now in place:

- IR rays name their original `observed_from` position and
  `ray_heading_deg_in_episode`; the current forward reading names its current
  fixed-frame heading. Survey age is expressed as movement commands since the
  scan. Tests verify that a turn and translation do not relocate old rays.
- The common planner retains one route/action policy. Its map explanation is
  selected from the supplied evidence: IR observations get their actual
  limitations, not instructions for nonexistent metric grids or keep-outs.
  The IR explanation makes clear that the waypoint is the drive origin and a
  clear center ray does not establish clearance for the whole body.
- The shared geometry contract explicitly distinguishes fixed `bearing_deg`
  from the relative `heading_error_deg`. No motor logic, preferred detour side
  or host-generated route was added in this follow-up.
- Slow valid responses are permitted explicitly: the controller planner accepts
  a caller-selected timeout up to 180 s; its default and BLAST configuration
  are unchanged. The simulation defaults to 120 s per request, low reasoning
  and 4,096 output tokens. `--time-budget` controls the separate test budget.

The server spent about 24–30 s processing an initial prompt, and one valid
answer arrived after 95 s. A 60 s timeout could therefore discard a valid
answer. The operator also reported running Minecraft on the model-hosting
computer: these timings are **not an unloaded model-speed benchmark**. Hardware
contention does not negate the independently observed context mistakes.

After the frame correction, Qwen's recorded response correctly located the old
hits at the origin and understood the right turn from +y toward +x. However,
its first offset remained too small for the body. The subsequent IR-only check
also reached the first waypoint, then encountered a simulated body block.
It then replaced the route with a wider lateral waypoint at (72,300), followed
by (800,300) and (800,0). Alignment toward the new waypoint was interrupted
by body contact; its malformed simulated partial-turn receipt failed the real
worker validator. This is not a successful box detour.

That receipt defect is corrected without changing navigation policy: interrupted
scan-turn slices now report their terminal status consistently, just like
interrupted drive pulses. A focused replay of the three actual Qwen decisions
now retains the wider route and the measured partial heading instead of raising
a transport-format error. This replay validates execution, **not a fresh
model-planning success**. The harness now counts accepted model decisions
separately from completed execution steps, including decisions whose execution
fails. Do not add arbitrary clearance margins or scripted detours to turn a
failed model route green.

Before the receipt correction, all 1,961 tests passed, including 49 focused
planner/body/session checks and BLAST regressions. The receipt correction also
passes 24 focused body/session/shared-simulator checks; the final full gate
passes all 1,962 tests (82.9 s). Current artifacts:
`/private/tmp/ev3-ir-contract-qwen-route.jsonl` and
`/private/tmp/ev3-ir-context-final-quality.log`,
`/private/tmp/ev3-partial-alignment-replay.log` and the final gate
`/private/tmp/ev3-context-and-receipts-final-quality.log`. Physical EV3 navigation
remains disabled. Next: rerun real-Qwen obstacle recovery with the corrected
receipts, preferably without the competing GPU workload. Continue auditing
whether simulated contact feedback matches what the real worker can observe.

## Remaining acceptance points — stop and report after each

1. **Physical EV3 acceptance.** Live application wiring and simulation checks
   are complete. First full survey/direction agreement, then
   clear-floor travel, then a box detour. Compare the map with what the operator
   sees. Confirm onboard speech too. Do not claim simulator success establishes
   real IR performance, wheel calibration or arbitrary-room autonomy.
   Continue reporting model-route contacts separately from eventual arrival;
   do not replace a failing Qwen route with a host-authored route.
2. **The pair.** Validate robot-to-robot messages with explicit identities,
   shared observations reaching both planners, aligned maps and independent
   task ownership. Run both production agents in the shared simulator before
   enabling concurrent physical missions. Respect Qwen's two-request limit.

No camera implementation or self-generated long-running missions yet. Keep
those future capabilities in mind without adding their infrastructure now.
The older navigation plans are historical context, not instructions to restore
retired behavior or to reject every uncertain reading.
