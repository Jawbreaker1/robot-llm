# Multi-robot navigation simulation

This simulator validates navigation decisions, not a prewritten route.

## Boundary

The simulation owns only:

- room boundaries and rectangular obstacles;
- each robot's measured body footprint and ground-truth pose;
- range observations and collision prevention;
- final goals; and
- concurrent execution in one shared world.

The simulation does not own waypoints, route selection, obstacle side,
backtracking, scan timing, or replanning. Those decisions must come from each
robot's production agent and its configured model.

BLAST connects through its existing controller command surface. EV3 connects
through its existing worker, encoder, IR, and active-scan surfaces. Hardware
differences remain inside these adapters; both robots see the same physical
world and each other.

## Initial scenarios

The route-free suite contains:

1. a clear shared room;
2. one box blocking both direct goal lines;
3. a room with staggered obstacles; and
4. a dead end that can require reversal and another opening.

Every scenario contains BLAST and EV3 simultaneously. A scenario definition
contains no expected route. A successful run must record which production
model planned each robot and retain its model-authored waypoint history.

## Acceptance

A navigation result is useful only when:

- both production agent loops were active in the same simulation window;
- every robot received only its sensor/map context, never ground truth;
- no collision occurred, including robot-to-robot collision;
- the final goal remained present through every replan;
- waypoints came from the model and were visible in the trace;
- repeated scans required changed evidence or meaningful movement; and
- each robot either reached its goal or returned a bounded, explainable
  failure trace.

Fast scripted planners may still be used in narrow executor unit tests, but
their results must never be reported as navigation validation.

## BLAST full-turn correction

BLAST's sixteen normal scan pulses cover slightly less than one body rotation.
The surroundings command therefore ends with one fixed, short trim pulse. The
trim completes the rotation but is not added as another map ray: the starting
ray already represents that direction.

The shared simulator measures 367.5 degrees of encoder-derived coverage and
ends 7.5 degrees from the starting heading before episode-level restoration.
Production accepts a surroundings
scan only after its own encoders confirm at least one complete turn. Physical
validation on BLAST remains required after deploying the updated hub runtime.

## Physical regression: missing range mistaken for an obstacle (2 September)

The physical box run turned right and moved approximately 47 mm, then reported
an invalid range reading. The episode labelled the interrupted leg
`FORWARD_CLEARANCE_UNAVAILABLE`, and the model prompt called it new blockage
evidence. With the recorded startup scan and the live goal/persona, Qwen
reproduced the failure: it abandoned the right route, declared left waypoints,
then requested a front scan while still facing right.

The correction reuses the existing bounded, stationary range reread. If the
reading remains unavailable, the planner receives `RANGE_MEASUREMENT_UNAVAILABLE`
with a null distance, not an invented wall. A motion that fails to approach its
target is separately reported as `MOTION_PROGRESS_STALLED`; its cause is not
assumed. Route and scan choices remain model-owned. A front scan refers to the
current heading, not a newly declared waypoint.

Reproduce using the loaded Qwen model:

```sh
PYTHONPATH=src .venv/bin/python -m robot_agent.blast_navigation_simulation \
  --scenario blast-measured-box-and-chair \
  --startup-scan-json tests/fixtures/blast_box_scan_20260902.json \
  --range-dropout-reads 4 --max-decisions 16 --timeout-seconds 60 \
  --goal 'Navigate around the box and reach the goal 800 mm straight ahead of your starting position. Use your map to choose the route and briefly explain your plan as you go.' \
  --compact
```

Before the policy cleanup below, two corrected replays with this goal reached
within 51 mm in six model
decisions, with one startup scan, no additional scans, no blocked moves, and no
route refusals. Qwen retained the right-side detour. This verifies the reproduced
sensor-loss failure, not all navigation: a broader box run still needed blocked
move recovery, and a different goal wording produced an unnecessarily long
detour. Do not report these as clean navigation passes.

The initial scan is recorded hardware evidence. Later sensor readings come from
an approximate box/chair world, not measured room ground truth. Physical speech
and angular drift are not validated by this replay. The user's report that the
very first physical waypoints were left while BLAST turned right remains
unresolved: the original first waypoint coordinates were not retained in the
captured trace. A subsequent physical validation must compare the first model
waypoints with the executed turn; do not dismiss this as merely a later replan.

## Rejected policy-cleanup experiment (2 September)

The experiment removed the host's `direct_detour_axis_candidates` calculation, its prompt
instructions, the instruction to retain a particular detour side, duplicate
axis-alignment wording, a second padding instruction, and the blanket scan
after a detour. Observed echo bounds, body clearance, unknown-space semantics,
waypoint execution and model-owned replanning remained. The experiment also
made two turn-alignment paths reuse the physical-IMU-to-episode-heading
conversion.

Coordinate checks cover left and right waypoints, nonzero and wrapped gyro
references, motor direction, estimated versus simulator position, and map
projection. These pass; they do not certify physical calibration.

The simplified prompt did **not** pass the two real-Qwen navigation trials
(`reasoning_effort=none`, 1024 output tokens, four missing range reads):

- Recorded box/chair start: decision budget exhausted at 444 mm from the goal,
  two scans, two blocked moves, six route refusals (12 model decisions).
- Box ahead: no executable action at 453 mm from the goal, one scan, four
  blocked moves, one route refusal (eight model decisions).

Both executed the first waypoint on its selected side. Later motion approached
the box too closely or attempted to return before passing it. A separate
diagnostic with low reasoning and 4096 output tokens showed Qwen planning to
pass the box using the observed front near x=385; the next response hit the
token limit. It is not a completed navigation validation.

No live run or dashboard restart followed this cleanup. After the failed
validation, its behavior-changing removals were restored to the preceding
baseline. Leaving those regressions in the working tree and calling the step
finished was incorrect. The extra coordinate checks were retained. The gyro
refactor was also reverted for an exact baseline comparison: it introduced
millidegree rounding into calculations that previously used raw gyro degrees,
so it was not strictly identical even though the coordinate tests passed.
Restoring the baseline does not establish that its additional route guidance
is the desired final design.

With the instructions restored but the gyro refactor still present, two
replays (four missing reads and no missing reads) reached within 42 mm of the
goal. Both contained two blocked moves; these are not clean navigation passes.
After reverting the gyro refactor as well, the same four-missing-read replay
reached within 32 mm in 11 decisions, with one startup scan, two blocked moves,
and one route refusal. Thus the full restoration recovers goal completion but
does not reproduce the earlier clean passes reliably. The previous baseline
itself is not established as robust. No physical run was performed.

## Subsequent physical test and diagnostic gap (2 September)

The later physical box test chose a right-side waypoint `(0, -150)`, turned
right (confirmed by the user), advanced approximately 48 mm, received a
no-valid-return distance, and requested a front scan while retaining that
waypoint. The episode then stopped with the generic `no_safe_blast_action`
scan outcome. The original controller error was not retained, so this evidence
does not establish which physical scan check failed. That run used reasoning
`none`, not the requested reasoning-enabled diagnostic setup.

The route-free simulator uses the real episode planner/executor but a separate
hardware adapter. Its scan results mark observations as settled and sensor pose
as verified, its gyro follows simulated world pose exactly, and it does not run
the physical monitor's per-pulse scan checks. Recorded-startup replay replaces
only the first scan; later observations again come from simulated geometry.
Consequently, successful simulated routes did not establish reliable physical
navigation, and describing them as that evidence was incorrect.

BLAST live and simulation defaults are now reasoning `low` and 4096 output
tokens. The two CLI entrypoints save local JSONL diagnostics in
`local-artifacts/navigation-dashboard.jsonl` and
`local-artifacts/navigation-simulation.jsonl`. Each planner request has a
correlated raw response saved before parsing, including reasoning and provider
finish reason even when the response cannot be decoded. The previous 4000-character
reasoning truncation is removed; the provider response byte limit still applies.
Controller action results and failures retain the original error code, message,
and chained cause rather than only the episode's generic stop message.

No navigation policy or scan acceptance condition changes in this diagnostic
step. The next physical comparison must trace the first real divergence through
sensor evidence, model reasoning, command, and controller result before further
navigation changes. Model reasoning is explanatory evidence, not ground truth
about the room or proof of successful execution.

Validation of this diagnostic step: 249 relevant unit tests passed. A deliberately
two-decision Qwen trial replayed the latest physical startup scan in the measured
box/chair world with four missing range reads. Both replies completed normally
with reasoning `low` and 4096 output tokens: 2149 and 3880 completion tokens,
approximately 20 and 32 seconds, with 5080 and 9665 reasoning characters retained.
The trial stopped at its two-decision limit, still 651 mm from the goal; this was
a logging/configuration check, not a navigation pass. The dashboard was restarted
with low/4096 and verified IDLE; no new physical episode was started.

### Uncertainty-recovery correction

The existing motorless missing-range retry now lives in `read_episode_observation`
and is reused before motion admission as well as between planner iterations.
Scans retain their existing internal settling, without a duplicate pre-scan retry.
A regression test verifies that a missing pre-action range can recover to a fresh
500 mm reading without movement or another planner call.

After explicit user approval, bounded forward admission now also retains the
latest settled, measured forward clearance in the existing execution history.
It subtracts the distance travelled since that observation and uses the existing
300 mm short-travel reuse limit. A turn without a new measured view, an incomplete
advance, or newly measured close clearance prevents reuse. Missing range remains
`NO_VALID_DISTANCE`/null in model context; it is not rewritten as a clear reading.
The obsolete straight-follow-through helper was folded into this evidence check.

A separate test through the physical `BlastObservationMonitor` reproduces a
front-scan bug: idle, encoder-correlated motion with unsettled range makes the
front-scan builder disagree with its validator and raise
`BLAST front scan encoder geometry was invalid`. Full-surroundings scans already
separated endpoint verification from range settling; the front-scan builder and
executor now do the same. The previously expected-failure test passes through the
physical scan monitor and motion executor with range settling false, verifies
that localization survives, and verifies that unsteady rays remain sweep-only.
Existing partial-scan handling still retains measured progress and returns to
the model; the correction does not treat malformed/missing encoders as verified.

Validation distinguishes recovery from navigation success. The persistent-range-
dropout regression retains its waypoint and advances about 315 mm in simulated
motor pulses before returning control for new information, instead of returning
after the first pulse. A newly measured close obstacle still prevents advance.
The physical-monitor regression exercises the real scan builder and pose
reanchoring, with fake hardware; it is not a physical robot run.
The final relevant unit-test run passed all 365 tests. Dashboard CLI tests now
mock diagnostic-log setup instead of writing to real runtime logs. The dashboard
was reloaded with the correction and verified IDLE, with no episode running.

A real-Qwen replay used the recorded physical startup scan, the measured box/chair
world and four missing range reads, with reasoning `low`, 4096 output tokens and
an eight-decision limit. The first waypoint was reached and the first five model
answers completed. There was one startup scan, no repeated-scan loop, no blocked
moves and no route refusals. However, the sixth answer exhausted 4096 tokens
before completing its JSON decision. The run ended 627 mm from the goal and is
**not a navigation pass**. Later simulated sensing remains idealized; replaying
the startup scan does not establish complete physical/simulator equivalence.

Two replays of that exact failing decision requested shorter reasoning, one
explicitly limiting it to 150 words. They returned complete decisions but still
used 4040 and 3895 completion tokens (3887 and 3727 reasoning tokens). Neither
demonstrated reliably short reasoning, so neither prompt instruction was added
to production. The configured `low`/4096 limits remain unchanged. No physical
navigation episode was started during this recovery correction.

The subsequent user-authorized physical trial
`episode-9c4b4fe543ed16fc386e6bfd` completed its startup surroundings scan
(367.255 degrees by encoders; roughly five degrees from the initial heading by
IMU). The first Qwen request used `low`/4096 and returned after 33.97 seconds
with `finish_reason=length`, 4096 completion tokens and empty final content.
No waypoint was accepted and no translational navigation action ran. The
dashboard reported FAULTED/robot_runtime_failed; the controller remained online
and stationary. Thus the approved post-motion dropout recovery was **not tested
physically** in this trial.

The saved reasoning repeatedly questions whether future route-hypothesis legs
may extend beyond currently observed-clear reach, alongside repeated grid and
clearance analysis. The prompt calls following waypoints a hypothesis but also
says not to commit a waypoint leg beyond observed-clear reach without explicitly
restricting that instruction to the current executable leg. This is a concrete
ambiguity to test against the saved request, not proof that it alone caused the
token exhaustion. No prompt or motion-policy changes were made during this trial.

### Planner input simplification (2 September)

The shared planner no longer forces greedy temperature 0; it sends temperature
1.0, top-p 0.95, top-k 20, min-p 0, presence penalty 0 and repeat penalty 1.0,
while retaining reasoning `low` and the 4096-token BLAST budget. These are
the sampling parameters recommended for thinking mode in the
[Qwen model card](https://huggingface.co/Qwen/Qwen3.8-27B#best-practices).
Before changing the prompt, three replays of the failed physical decision with
only that temperature change all produced complete decisions (3279, 3896 and
3313 tokens). This establishes a useful sampling comparison, not reliable
navigation or proof that the LM Studio reasoning setting was ignored.

The navigation system instruction was reduced from 13627 to 6012 characters
with the same English BLAST persona. Repeated route recipes and repeated
geometry/error instructions were replaced with one account of action execution,
route memory, uncertainty, final-goal tracking and map coordinates. The current
executable leg uses current observed-clear reach; following waypoints may extend
through unknown space as hypotheses, to be rechecked from the new pose. Neither
the available actions, map payload, output schema, motor control nor host
geometry checks changed. The model still chooses each route and turn side.
The fixed map frame now explicitly applies to echo/keep-out bounds as well as
waypoints: a failed corridor response had wrongly rotated already episode-local
echo coordinates using the current robot pose.

Both previously truncated contexts returned complete decisions through the
updated production planner: the first physical decision used 2383 tokens and
the sixth recorded simulation decision 3534. Reasoning remains variable and can
still be long. Full requests and replies for those isolated replays are in the
ignored `local-artifacts/navigation-prompt-validation.jsonl`. Unit tests retain
schema, waypoint-memory and sensor-context coverage; brittle assertions locking
in obsolete prompt paragraphs were removed instead of preserving dead wording.

The intermediate short-prompt/temperature-only full runs did not pass: the
recorded box/chair replay ended 429 mm from the goal after 11 completed model
decisions, with two scans and no blocked moves; the bent corridor ended 213 mm
away after five completed decisions, one scan and one blocked move. Both then
hit the output limit. The corridor's saved failure returned a complete decision
with 1079 tokens using full recommended sampling alone, and with the coordinate
clarification alone also returned a complete decision. However, the saved box
failure still exhausted 4096 with both changes. Do not treat the sampling or
prompt correction as a guarantee against truncation.

Final full-run comparison with the 6012-character prompt and all recommended
sampling parameters (still low/4096):

| Scenario | Completed decisions | Distance remaining | Scans | Blocked moves | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| Recorded box/chair startup + four missing reads | 8 | 436 mm | 1 | 0 | Next response exhausted 4096 tokens |
| Bent corridor | 0 | 800 mm | 1 | 0 | First response exhausted 4096 tokens |

Neither run passed. The box run retained and revised waypoints and moved well
beyond the old first-pulse failure, but did not reach the goal. The full-run
outcomes do not establish a navigation improvement or robust response length,
despite successful isolated replays. The final relevant unit suite passed 365
tests. No physical episode was started during this planner-input step.
Retry handling for incomplete model responses remains outside this step.

Further simplification must separate behavior-preserving code refactoring from
changes to the model's navigation instructions or input. Compare each behavior
change against the same recorded scenario before retaining it; failed cleanup
is not a completed improvement.

### 2026-09-02: preserve waypoint execution after a turn

The BLAST executor had two continuation checks: the normal check against the
next fresh observation, and a second reset after turning that inspected the
**pre-turn** available actions. A blocked original heading therefore ended
`FOLLOW_WAYPOINT` even when the chosen side was clear. Removed that stale reset
(10 production lines); no new navigation rule, route choice or scan policy.

The existing execution-contract test now includes a box 200 mm ahead of the
robot centre, where forward admission is initially blocked, as well as a box
320 mm ahead. Both side waypoints and zero/four missing range reads are checked.
Before the correction all four close-box variants stopped after turning without
translation. Afterward all eight variants reach their 450 mm lateral waypoint
with one planner decision, one startup scan and no collision. A separate
post-turn obstacle test verifies that a newly measured 90 mm range still returns
control to the model, retaining the waypoint and reporting the blockage.

These are scripted execution-contract tests, not evidence that Qwen can plan
and complete the full detour. Physical navigation has not been rerun in this
step; the model-response truncation and full-route issues above remain open.

A bounded real-Qwen check used the same close box (x=200..400, y=-180..180),
four missing range reads, reasoning `low`, 4096 output tokens and at most three
decisions. Qwen produced three complete decisions and selected the route itself:
(0,450), (450,450), (450,0), then the goal (800,0). The first two waypoints were
reached at approximately (-40,450) and (455,516), with only the startup scan.
The third leg encountered the box with the robot body (two simulator blocked
events) and ended at (486,320), 448 mm from the goal, when the three-decision
test budget was exhausted. This is **not a full navigation pass**. The third
response reasoned that no echo clusters blocked x≈450, but that was insufficient
evidence for body clearance around the box's far corner. No second correction
was bundled into this step. The relevant five-module suite passed 241 tests.

The final physical attempt that evening was episode
`episode-44e0116d1eb240d32946781c`, running the corrected adapter with Qwen
`low`/4096 and speech enabled. It did **not** reach the goal. After the single
startup scan, Qwen selected (150,0), (150,-300), (600,-300), (600,0).
Verified odometry reached (136,-11) and then (80,-280): approximately 14 cm
forward followed by 27 cm to the robot's right, without another scan. The
subsequent left turn completed only three of four slices, leaving heading
-33 degrees and no valid range. Pose and waypoints were retained. The requested
leg to (600,-300) was then rejected against the known echo at (380,-149), with
150 mm clearance. Qwen's next response exhausted 4096 tokens with no final
decision (`cd9209015232408895ba17f4aff8e75a`), and the runtime faulted instead of
recovering. The first spoken line played; two later playback attempts failed.
The controller remained online, with motion inactive and battery around 7.30 V.
No second physical run was made immediately after that attempt.
Full requests, reasoning and controller results remain in the ignored
`local-artifacts/navigation-dashboard.jsonl`.

### Follow-up: map labels and interrupted turns

Two operator-approved corrections follow the failed physical attempt:

- The map now draws the remaining route from the current robot position to
  the active waypoint, including when only one waypoint remains. Model-authored
  waypoint purposes are preserved as display data (`MODEL_WAYPOINT`); the map
  no longer invents lateral-clearance/pass/merge roles from list indices.
  Existing fixed-route display types remain supported. This changes no planner
  input, waypoint choice or route geometry.
- Once a bounded turn has passed fresh action admission, its existing
  continuation check may finish that turn through missing range echoes. The
  permission ends with that action, does not authorize forward movement and
  does not override a fresh close obstacle, a bad motor receipt or cancellation.
  Alignment still uses the existing coarse heading tolerance. No new turn loop,
  scan policy or route-selection rule was added.

Before the correction, the reproduced mid-turn missing-echo case stopped after
two pulses. Afterward both left/right and explicit-turn/waypoint-follow variants
complete four pulses with one scripted planner decision and no scan. The
waypoint-follow variants then execute the existing continuous advance. A fresh
40 mm reading still interrupts the turn after one pulse; persistent missing
echoes do not cause repeated quarter turns. These are execution-contract checks,
not real-Qwen navigation validation.

203 relevant tests passed. The actual route renderer was also inspected in an
isolated browser preview using the last physical run's waypoint coordinates,
including the transition after reaching a waypoint. At completion of these code
checks, no physical run or new real-Qwen run had yet been performed. The physical
server was subsequently restarted for the validation below.
The separate route-clearance rejection and truncated-model-response recovery
problems remain open.

### Physical follow-up: box passed, final approach interrupted

Episode `episode-37b371e06f18ff5820e2d3e5` ran the updated server with Qwen
`low`/4096, speech enabled and the goal at (800,0) mm. Qwen selected
(0,-300), (800,-300), (800,0). The first right turn and the following left
turn completed; the operator confirmed that BLAST physically passed the box.
Verified odometry reached approximately (795,-180), heading 11 degrees.
The live map displayed the model's waypoint purposes and the connecting route.
This is progress in physical execution, **not a completed navigation pass**.

Missing range readings after the turns still caused action admission to return
control to Qwen for extra scans. On the long forward leg, cross-axis drift left
the robot about 120 mm from the waypoint's y coordinate, outside the existing
75 mm cross-axis tolerance. Qwen then selected the final goal. Two reverse
pulses left the last verified pose at approximately (706,-196), about 217 mm
from that goal. A direct leg was rejected as `NON_ORTHOGONAL_ROUTE_LEG`, not
because of a newly detected obstacle. Qwen corrected the route to (700,0),
(800,0), retaining the goal and requesting a left turn.

The required turn was unavailable with the missing range reading; the offered
actions were reverse and scan. Qwen chose `SCAN_FRONT_ARC`. That action failed
with `controller_command_failed` ("BLAST command failed"), faulting the episode.
The diagnostic record contains no underlying cause. The controller remained
online and motion inactive, but the failed scan invalidated the current pose;
the last verified coordinates must not be treated as a verified post-failure
pose. All eleven Qwen responses completed within 4096 output tokens in this run.
Speech also had intermittent controller errors.

Remaining issues exposed here are post-turn range-loss admission, heading drift
versus waypoint arrival, misleading generic route-rejection wording, and recovery
from controller/scan failures. No navigation code was changed during this run,
and no second physical episode was started after the failure. Full diagnostics
remain in `local-artifacts/navigation-dashboard.jsonl`.

### 2026-09-03: one rotation rule and recovery of scan reads

The final approach exposed an inconsistent action contract: missing range could
admit a rotating scan, but not the model's quarter turn after translation.
BLAST now uses one current-observation rule for both. A no-return echo can admit
a bounded turn with the normal body/encoder checks; a newly measured close
obstacle still blocks it. Missing range is not converted to measured free space,
and forward movement still needs a current reading or the existing bounded reuse
of recent forward evidence. Mandatory startup perception is unchanged.

The separate scan-history turn-authorisation routine and its iteration filter
were removed. Qwen still selects direction, waypoints and replanning; no route
chooser, retry loop or model prompt was added. Existing tests that required
scan-only action sets on missing echoes now require the selectable turns too.
Old scan coverage can no longer override a current close range.

A second reproducible failure was an exception while reading after a scan pulse:
the pulse-send exception already entered stop/read recovery, but the following
idle/settling reads could bypass it and fault the episode. Both now enter the
same existing recovery. One fresh idle encoder observation can return a partial
scan with its actual heading, preserving localization for the next action.
Unverified encoders/body position or failed recovery still cannot be presented
as a trusted pose. Unhandled command errors now retain their underlying cause
for diagnostics. The prior physical log did not retain that cause, so this is a
reproduced failure path, not a claim that its original transport fault is known.

Validation: 295 relevant tests passed, followed by 163 action/execution tests
after the equivalent common-rule cleanup and two targeted monitor checks after
adding explicit partial-scan localization assertions. The scripted box-route
regression reaches the goal with one startup scan and no collision both with
and without a missing echo before its final turn. Reinstating the old no-return
turn gate makes only the missing-echo variant fail. Separate injected exceptions
in the idle and settling reads return a localized partial scan and permit the
next controller action. These are execution tests, not LLM planning claims.

The first real-Qwen follow-up used `qwen/qwen3.8-27b`, reasoning `low`, 4096
output tokens and an eight-decision cap. Qwen chose its own route around the box,
completed in six decisions with one startup scan and zero blocked moves, ending
135 mm from (800,0), inside the unchanged 150 mm goal radius. All responses were
complete (840–3280 output tokens). However, its shorter route never entered the
injected x>700 mm dropout zone, so this validates ordinary navigation only.
The supplementary run below broadens the dropout zone rather than counting that
first run as missing-range validation.

The one supplementary real-Qwen run injected seven no-return readings at x>500
mm when facing along the goal axis, or near the goal axis while facing sideways.
Qwen selected (0,300), then (500,300), and reached (512,387) without a collision
or extra scan. Its third response ended with provider `finish_reason=length`
without a valid decision, so the simulator stopped with `LMStudioProtocolError`,
482 mm from the goal. The recorded request explicitly contains
`reasoning_effort=low` and `max_tokens=4096`. This is **not** a successful
end-to-end missing-range validation; it exposes the already-known missing
recovery for truncated model replies. No third run or planner/retry redesign
was bundled into this correction.

Result artifacts: `local-artifacts/recovery-qwen-baseline-20260903.json` and
`local-artifacts/recovery-qwen-dropout-20260903.json`; requests are in
`local-artifacts/navigation-simulation.jsonl`. No physical robot was moved in
this step. Restart the dashboard/robot server before physical validation so it
loads the corrected action admission and scan recovery. The truncated-response
recovery is the next separate issue; full physical reliability is not claimed.

### 2026-09-05: checkpoint 1 — waypoint tracking and rotation geometry

BLAST now checks its course between forward pulses while executing
`FOLLOW_WAYPOINT`. It uses the existing turn pulses when both heading error and
the projected sideways miss exceed the existing 12 degree / 75 mm tolerances.
Small discrepancies remain accepted; a plain model-requested `ADVANCE` is not
silently converted into waypoint steering. This changes execution of an accepted
point, not model route selection, scan policy or the cardinal waypoint contract.

The injected-heading test exposed another execution gap: settled IMU readings
were not updating the pose after ordinary motion. Wheel-derived heading could
therefore hide a physical heading disturbance from the waypoint follower. The
existing episode-relative gyro conversion now refreshes heading after verified,
settled motion; encoders still own distance and continuity. Missing/non-finite
headings retain encoder-based heading. This is not absolute room localization,
and errors in measured travel or gyro drift remain possible.

The physics simulator now checks the swept rectangular body during rotation,
using its existing obstacle/boundary/peer-robot collision checks and approximately
10 mm corner sampling. BLAST and EV3 motor adapters report reduced encoder motion
when a rotation is blocked. EV3 scan turns report the actual achieved angle.
The optional recorded-startup fixture is still evidence replay, not a simulated
full sweep; it must not be used as proof of collision-free scan motion.

Validation: 588 BLAST, EV3 execution, scan, geometry and simulation tests passed.
Three focused test methods were added, with mirrored/robot subcases; an existing
partial-scan fixture was corrected to give matching gyro and encoder angles.
The 750 mm waypoint test injects an 11 degree course error independently of the
navigation calibration. Both sides reach the accepted waypoint corridor, with
actual endpoints (45,-748) and (63,748) mm, one planner call, one startup scan,
and zero blocked moves. The run deliberately ends at its one-decision budget:
this validates the chosen waypoint, not completion of the separate final goal.
Existing transient-dropout side routes and final-turn dropout regressions pass.
Rotation tests include a corner hit between otherwise-clear start/end poses and
partial encoder reporting from both hardware adapters.

These are execution regressions with scripted decisions, **not new Qwen planning
successes**. LM Studio responded at `127.0.0.1:1234` but `/v1/models` returned an
empty list. No model was loaded automatically and no physical robot was moved.
Next: bounded Qwen validation, then checkpoint 2 (retained route continuation
and model-response recovery), followed by physical validation when Blast is ready.

### 2026-09-05: real-Qwen follow-up — not a clean pass

See [the bounded real-model validation report](NAVIGATION_VALIDATION_20260905.md).
Open floor completed, but the box case contacted corners, including one contact
caused by the newly added course correction. The disturbed box run subsequently
faulted on a truncated model reply. A separate first-decision probe with 10% turn
slip exposed up to 35.28° error between projected scan rays and gyro angles.
All calls used the loaded Qwen3.8 model with low reasoning and 4096 output tokens.
These findings supersede any inference that the passing execution regressions
establish physical readiness. No production navigation changes or physical
robot movements were made during this validation.

### 2026-09-05: bounded scan and course-correction follow-up

Corrected the two demonstrated execution contracts before proceeding with route
continuity. Measured gyro yaw now owns BLAST scan angles and sweep coverage;
physical observation code and simulator share the sweep-completion rule. The
existing waypoint feedback uses a smaller fixed turn pulse without changing
model route ownership or ordinary 90-degree actions.

The previous box-side execution regression now passes; restoring the old pulse
makes the test fail. All 604 targeted regression tests pass. A real Qwen
rotation-loss probe reports and physically simulates the same 370.44-degree
sweep, with no ray-angle discrepancy beyond rounding (previously up to 35.28
degrees). A separate real Qwen box run reached the first waypoint without
blockage, then stopped on a truncated second reply at 4096 tokens. Neither run
is a successful complete box detour.

See [the follow-up evidence and limits](NAVIGATION_VALIDATION_20260905.md#follow-up-implementation-and-validation--september-5).
The new internal turn operation requires reloading the hub program; nothing was
deployed and no physical robot moved. Pause this implementation checkpoint here.
Next: model-reply recovery with retained episode state, then a complete bounded
real-Qwen rerun before physical validation. No new route planner or maze-memory
layer was added in this checkpoint.

### 2026-09-05: minimal reply retry; one complete real-Qwen simulated detour

BLAST now retries one unusable structured model reply with the identical
navigation context. It neither scans nor moves between those attempts, retains
goal/route/pose, and honors stop/deadline checks. Two bad replies still fail;
there is no generic recovery framework or new navigation rule.

Six fixed-context Qwen comparisons (two each: low/4096, low/8192, off/4096) all
returned valid decisions. Reasoning off took 3.49–4.79 seconds versus
22.38–31.38 seconds with low reasoning. All successful answers fit below 4096,
so the comparison does not demonstrate a benefit from the higher ceiling.
Low reasoning is retained; 8192 is headroom. Live/web and simulation now share
the same 60-second model timeout instead of differing defaults.

In one complete box episode, six real Qwen calls planned and executed the
detour. A previously recorded truncated response was injected after the first
leg, triggering exactly one retry with unchanged context. The robot reached
the simulator's goal acceptance region, 49 mm from its center, with one startup
scan and zero blocked moves. No decisions or startup scan were scripted. Every
real reply finished within 4096 tokens, so success cannot be attributed to the
8192 ceiling. The 674-test targeted gate passes; see
[the detailed report](NAVIGATION_VALIDATION_20260905.md#bounded-reply-recovery-and-output-budget-comparison--september-5).

Checkpoint 2's route-continuation redesign is not implemented by this slice.
Physical validation is next, after reloading the hub program and restarting the
host. No physical motion, model loading, or firmware deployment occurred.

### 2026-09-05: four further real-Qwen cases — physical readiness withdrawn

The operator requested additional validation, especially using physical data.
No production code or settings changed during this series. Sequential runs
used low reasoning, 8192 output tokens, 60-second requests and at most 12 model
decisions per case. Four cases produced **0/4 goal-reaching/completed episodes**:

| Case | Distance remaining | Result |
| --- | --- | --- |
| Recorded September 2 startup scan | 600 mm | Body contact on right-side leg, then model timeout |
| Same scan + four invalid range reads | 600 mm | Same contact; replanned turn repeated without motion until deadline |
| Approximate box/chair world, new simulated scan | 438 mm | Turned back before clearing the box's full extent, then model timeout |
| Synthetic bent corridor | 284 mm | Replanned detour, then another repeated blocked turn until deadline |

All had one scan, not repeated scans. The dropout run recorded 2210 blocked
pulse attempts with only three Qwen calls; the corridor recorded 2340 with
seven calls. The host continued ordinary waypoint turns without the no-progress
handoff already used for forward continuation. This is a reproducible execution
bug, not thousands of model decisions. True and estimated final poses agree in
all four tests. Unknown/echo-free space was also treated as executable in model
reasoning despite explicit unknown map semantics and missing clear reach.

Across the series, 17 real model requests yielded 15 valid replies and two
timeouts. One valid reply used 5224 tokens, confirming that the new ceiling can
help a reply finish, but not that it fixes navigation. The only actual recorded
data is the startup scan; subsequent box/chair geometry remains approximate,
and replay does not exercise physical scan rotation. Full BLE, ultrasound beam,
speech, slip and hub dynamics remain outside these simulations.

Evidence is retained in `local-artifacts/qwen-physical-cases-20260905-*.json`
and `.jsonl`. See [the detailed findings](NAVIGATION_VALIDATION_20260905.md#additional-physical-data-validation--september-5).
Pause physical testing until the narrow blocked-turn execution error is fixed
and revalidated. No new planner or box-specific steering rule is warranted by
these findings, and none was implemented in this validation-only turn.

### 2026-09-05: zero-motion handoff fixed; recorded-data detour self-recovers

The operator clarified that unknown space may be explored and that missing scan
rays must not recreate the old scan/stop loop. The bounded correction therefore
changes execution, not navigation admission: a motor action with unchanged pose
returns `MOTION_PROGRESS_STALLED` to Qwen with the route intact; zero-encoder
attempts do not erase an earlier verified path supporting bounded reverse.

An execution replay of the previous three model waypoint choices detects the
old host loop and now reaches the next planner call, permitting a selected
reverse with no new scan. The 675-test gate passes. Three real-Qwen reruns gave:

- Recorded physical startup + four missing readings, low reasoning: completed
  40 mm from goal, one scan, six blocked pulse attempts, ten model calls. Qwen
  chose to reverse after its blocked turn, changed the detour and reached the
  goal. This is self-correction after contact, not a contact-free run.
- Bent corridor, low: no host turn loop, but a model timeout after the handoff;
  521 mm remained, one scan, six blocked attempts, four model requests.
- Bent corridor, reasoning off: no blocked motion, but 315 mm remained after
  twelve decisions, two route refusals and three scans. Not a completed run.

Full details and limits are in
[the validation report](NAVIGATION_VALIDATION_20260905.md#zero-motion-handoff-correction-and-bounded-reruns--september-5).
The low-reasoning corridor still withheld REVERSE due to an earlier partially
achieved turn; that is outside the zero-motion-only correction. No tighter scan
rules, default detour, planner, model-default change or physical movement was
introduced. Pause here with corridor completion and partial-motion retreat
explicitly outstanding.
