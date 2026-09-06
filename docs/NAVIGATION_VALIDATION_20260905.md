# Real-Qwen navigation validation — 2026-09-05

**Verdict: not approved for a clean physical box-detour claim.** The new
execution behaviour exposes reproducible corner contacts, and scan geometry
still assumes encoder-derived rotation even when the gyro disagrees. No
production navigation code was changed during this validation.

## What was actually exercised

The loaded model was `qwen/qwen3.8-27b`. Recorded requests contain
`reasoning_effort: low` and `max_tokens: 4096`. Runs were sequential. The model
received the production prompt, observations, map and history and selected its
own waypoints. No route or full-world obstacle geometry was supplied to it.

`run_blast_gemma_scenario` uses the same `BlastEpisodeRuntimeAdapter`,
`LMStudioControllerActionPlanner`, map builder, action admission and verified
motion executor as the dashboard BLAST path. Only the controller endpoint is
substituted with `SharedWorldBlastController`. Startup scans were generated from
world sensing and rotation, not replayed from a recorded startup fixture.

The box case uses the existing scenario: box x=320–520, y=-180–180 mm, start
(0,0), goal (800,0), and the project's provisional asymmetric BLAST footprint
and sensor offset. It is not a measured digital twin of the user's room.

## Results

| Run | Model calls / limit | Result |
| --- | --- | --- |
| Open floor, 30 s response timeout | 2 / 8 | Completed, 108 mm from goal, one startup scan, zero blocked motions. This uses the existing 150 mm goal tolerance. |
| Box, diagnostic 60 s timeout | 8 / 8 | 127 mm from goal but episode not completed; six blocked motion attempts. Not a pass, even though inside the goal radius. |
| Box + planned disturbance, 30 s timeout | First request timed out | No motion: the faults were **not exercised**. |
| Box + 11° heading disturbance and seven missing-range reads, diagnostic 60 s timeout | 4 valid decisions, fifth response truncated / 8 | Faults exercised; 638 mm from goal, three blocked motions, one startup scan. Stopped on invalid/truncated model response. |
| Box + 10% less actual chassis rotation than wheel-derived rotation | 1 / 1 | Focused startup/first-leg probe, not a goal-reaching test. Scan-angle inconsistency demonstrated below. |

The box baseline responses took 11.6–46.0 s; three of eight exceeded the physical
CLI's default 30 s timeout. Diagnostic 60 s runs are therefore not equivalent
to that timing configuration. The longer timeout was only a test parameter;
the physical robot configuration was not changed.

## Concrete findings

### 1. The new course correction can steer into the box corner

In the baseline the robot was at (152,339), heading +7.5°. The accepted leg
ended at (600,300). Course feedback applied a -22.05° pulse, leaving heading
-14.55°. After two more forward pulses the body was at (240,317); the next
pulse was blocked by the box corner while the forward sensor still read clear.

A separate geometry-only counterfactual from the **same** starting pose confirms
the causal difference: three forward pulses without that correction reach
(287,357) unblocked; with the correction the third pulse is blocked. This is
not another model navigation run. Reducing bearing error to a point does not
guarantee that the whole body remains in the free corridor along the route.

### 2. Visible front-face echoes are not the box's full extent

The first and fourth real model contexts both contained three box echoes at
x=320–328, y=-91–80. Their padded center keep-out bounds were x=170–478,
y=-241–230. The real simulated box extended to x=520 and y=180.

Qwen selected a return leg at x=600 and treated it as clear. At (607,321), heading
-80.7°, the body's left/front corner blocked the next forward pulse, even while
the sensor ray measured 1150 mm. Qwen repeated that manoeuvre and described
the lack of progress as a transient motor issue before changing route.

The prompt explicitly says echoes are not complete object outlines and marks
unobserved space unknown. That warning alone did not produce the needed new
observation. There was only the startup scan. We must not relabel these failures
as a sensor proving free clearance for the entire body, or fix them by feeding
the simulator's hidden obstacle rectangle into the planner.

### 3. Gyro feedback after motion does not fix scan projection

The rotation-slip probe deliberately preserved commanded wheel encoder motion
but made actual chassis rotation 10% smaller, with an accurate simulated gyro.
The first 17 scan pulses accumulated 367.5° by encoder calculation but only
330.75° of actual rotation. The scan was nevertheless labelled `complete`,
`restored`, `restoration_verified: true`, coverage 367.5°.

Its own IMU diagnostics recorded 29.25° final yaw error, but were marked
`DIAGNOSTIC_ONLY`. Individual projected ray angles disagreed with their gyro
angles by up to 35.28°. The later heading correction brought the robot near its
initial heading; it did not repair the already projected rays or prove full
angular coverage. This is a simulated sensitivity result, not a measurement
that physical BLAST currently has exactly 10% slip.

### 4. Reply recovery is still missing

In the disturbed run the fifth response ended with `finish_reason: length`
without a valid decision. Low reasoning and 4096 output tokens were present in
the actual request. The episode faulted with `LMStudioProtocolError`. This is
separate from the box-contact problem and must not be blamed on range dropout:
the initial short dropout had already been handled without an additional scan.

## Simulator fidelity limits

The simulator now catches rectangular body contacts during both translation and
rotation. In the undisturbed baseline, estimated and true final poses matched.
The fault harness changes only physical motion/sensor output, not navigation
calibration or policy, so successful perfect-calibration tests cannot hide the
scan-angle disagreement demonstrated here.

However, it does not execute BLE transport, hub firmware, actual motor dynamics,
or the real `BlastObservationMonitor` scan/read/settling loop. Its range sensor
is a geometric ray, not the beam/reflection model of a dark physical box. Contact
limits movement/encoders rather than simulating wheels spinning against a box.
Speech is disabled and its clock does not include actual model waiting time.
These tests validate the shared navigation loop under specified simulated faults;
they are not proof of full physical equivalence or of EV3 model navigation.

## Next implementation boundary

Before expanding route continuation, correct the heading/scan-angle contract
and the coarse course-correction behaviour demonstrated above. Then replay the
exact failing inputs, rerun the bounded real-Qwen cases, and physically validate
one box detour. Keep model-response recovery as an explicit remaining issue.
Do not replace model planning with a prewritten detour or enlarge all obstacle
margins to hide these errors.

## Evidence and reproduction

- Real box baseline: `/private/tmp/qwen-box-baseline-20260905.json` and the latest
  run in `local-artifacts/navigation-simulation.jsonl` (eight raw model replies).
- Exact response replay for inspection only:
  `/private/tmp/qwen-20260905-box-forensic-replay.json`; reproduces the same final
  pose and all six blocked events without making additional model requests.
- Open, disturbed and slip results and raw request/response logs:
  `local-artifacts/qwen-20260905-*.json` / `*.jsonl`.
- Repeatable test-only fault harness:
  `local-artifacts/validate_qwen_physics_20260905.py`.

No physical robot was moved, model loaded/unloaded, or production navigation
logic edited during this validation.

## Follow-up implementation and validation — September 5

The statement above describes the initial read-only validation. A subsequent,
bounded implementation corrected findings 1 and 3; findings 2 and 4 remain open.
No physical robot was moved or hub program deployed during this follow-up.

### Changes

- Scan rays and scan coverage use actual gyro yaw when available. Wheel
  encoders still verify motion continuity, disambiguate complete turns, and
  provide fallback when gyro readings are missing. Historical encoder-based
  recordings remain readable under their original bearing-source contract.
- The physical observation monitor and simulated BLAST adapter now call the
  same full-sweep completion rule instead of using separate fixed pulse counts.
  Subsequent estimated heading uses the same measured endpoint as the scan.
- Waypoint course correction uses a fixed 15-degree wheel pulse, approximately
  7.35 degrees of chassis turn, instead of a 45-degree wheel pulse (22.05 degrees
  chassis turn). The existing correction threshold, model-authored waypoints,
  and ordinary 90-degree turn actions are unchanged. This is an internal motion
  primitive, not another model action or route-selection rule.

The new `turn_trim_pulse` hub operation **requires reloading the BLAST hub
program before physical testing**. It has not been deployed in this checkpoint.

### Execution evidence

The exact first two legs from the failing model route, (0,300) then (600,300),
now execute without blockage and with one startup scan. This regression uses
scripted decisions from the earlier real run, not a new planning success. Its
negative control restores the old correction pulse and reproduces a blocked
move, confirming that the test detects the demonstrated regression.

The physical `BlastObservationMonitor` is also exercised with a fake runtime
whose measured chassis yaw is 90% of encoder-predicted yaw. It completes the
measured sweep and projects rays at their measured angles. This tests the real
observation-loop code, not BLE transport or actual motors.

All **604** targeted BLAST, EV3 execution, scan, simulation and map regressions
passed. `git diff --check` passed. These are not 604 model navigation runs.

### New real-Qwen runs

Both used the already loaded `qwen/qwen3.8-27b`, reasoning `low`, 4096 output
tokens, and no recorded startup scan or scripted planner. The diagnostic model
timeout was 60 seconds, not the physical CLI's default 30 seconds.

| Run | Observed result | Assessment |
| --- | --- | --- |
| Box, up to eight decisions | First waypoint reached, one scan, zero blocked moves. Second reply exhausted 4096 tokens without a valid decision; `LMStudioProtocolError`. Ended 886 mm from the goal. | **Not passed.** Reply recovery still required. |
| Box with 10% rotation loss, one-decision probe | Startup sweep used 19 pulses; reported coverage and actual world rotation both 370.44 degrees. All 17 retained ray angles match their gyro measurements to rounding precision. First model action executed with zero blocked moves. | Scan contract validated under this fault; **not a goal-reaching test**. |

Before correction, the same rotation-loss probe reported 367.5 degrees while
actually turning 330.75 degrees, with up to 35.28 degrees of ray-angle error.
After correction the final sweep residual is 10.44 degrees, inside the existing
coarse restoration tolerance; the measured residual is retained in the pose.
This does not claim perfect 360-degree motor precision.

Evidence files:

- `local-artifacts/qwen-20260905-box-t60-corrected.json` and `.jsonl`.
- `local-artifacts/qwen-20260905-box-turn-slip-t60-corrected.json` and `.jsonl`.
- `/private/tmp/scan-trim-complete-regressions.log` (604 passing tests).
- `/private/tmp/old-course-negative-control.log` (expected failure with old pulse).

### Remaining boundary

Do not expand this correction into new route rules. Next, recover an incomplete
model reply while preserving goal, route and position, then rerun a complete
bounded Qwen box case. Unknown obstacle depth and the need to observe a return
leg remain unresolved planning/observation issues from the initial validation.
The simulator still does not reproduce ultrasonic reflections, BLE, hub motor
dynamics, or every physical slip mode. A physical detour is required before
claiming the navigation works on BLAST.

## Bounded reply recovery and output-budget comparison — September 5

The earlier failure used all 4096 completion tokens for reasoning and returned
zero final-content characters. We replayed that exact second-decision request
twice for each setting, changing only reasoning effort and token ceiling. These
six calls used the loaded Qwen model; they were fixed-context response tests,
not complete navigation episodes. No model was loaded or unloaded.

| Setting | Valid structured replies | Completion tokens | Latency |
| --- | --- | --- | --- |
| Low reasoning, 4096 ceiling | 2/2 | 2541, 3025 | 27.62, 31.38 s |
| Low reasoning, 8192 ceiling | 2/2 | 2050, 2745 | 22.38, 29.38 s |
| Reasoning off, 4096 ceiling | 2/2 | 216, 271 | 4.79, 3.49 s |

The server reported zero reasoning tokens with `none`, and nonzero reasoning
tokens with `low`, confirming that these settings affected this loaded model.
All six chose a current waypoint at (800,300). Their validity here means the
production decision decoder accepted them, not that every proposed route was
physically validated. The variation is too large and the sample too small to
claim that 8192 improves planning or response success: every successful answer
in this comparison fit below 4096. The ceiling nevertheless provides room for
reasoning plus the final decision when a response would otherwise be truncated.

### Minimal production change

BLAST's existing planner call now has one bounded retry for an unusable
structured reply (`LMStudioProtocolError`). It resends the same context object;
there is no new state machine, repair prompt, route rule or fallback manoeuvre.
It does not change reasoning mode automatically. Goal, active waypoint,
following waypoints, pose, map and history are unchanged during the retry. The
existing stop/deadline checks run before retrying. Two unusable replies still
raise the original error. Transport timeouts are not retried by this slice.

Low reasoning remains the default. The shared BLAST output ceiling is now 8192;
the raw response byte limit accommodates the reasoning envelope while the
existing final-decision size/schema limits remain unchanged. Web-started
physical BLAST and the simulator now share a 60-second request-timeout default
(previously 30 and 20 seconds respectively). This is a deadline, not a promise
that all 8192 tokens can be generated within it. Already running hosts need a
restart to pick up the changed defaults.

### Full simulated box run with a reply fault

The route-free box scenario used the production BLAST episode, production
planner, gyro-aware full startup scan, simulated body/encoders/sensor and six
real Qwen calls, with low reasoning, 8192 ceiling and 60-second timeout. After
the first executed leg the harness returned the previously recorded truncated
provider response once. This deliberately injected software failure was not a
model-authored decision for this run and did not prescribe any route.

The request before and after that failure was byte-for-byte identical. The
retry produced a valid decision and execution continued. Result:

- Episode completed and the ground-truth simulator goal check passed.
- Final distance to goal: **49 mm**, within the existing acceptance radius.
- **One** startup scan; **zero** blocked moves.
- **Six** real model calls; **one** injected truncated reply; **one** retry.
- Every real answer finished normally, using 373–3471 completion tokens.
- No scripted decisions and no recorded startup scan.

Thus this is evidence for a complete simulated detour and bounded reply
recovery. It is not evidence that doubling the output ceiling caused success,
nor that every box arrangement or physical BLAST is validated. The previously
identified hidden-depth/corner case was not independently closed by this one
successful route. No movement, map, route-selection or scan logic changed in
this slice.

The targeted gate passed **674 tests**. Two focused test methods were added for
route/context preservation, bounded retry and stop handling. Existing tests
also check the larger reasoning envelope and equal live/simulation defaults.
The latter focused 50-test gate passed after the final test assertions were
updated. These counts overlap; they are not hundreds of real model runs.

Local evidence (temporary files, not committed):

- `/private/tmp/qwen-navigation-budget-comparison.jsonl` and
  `/private/tmp/qwen-navigation-budget-comparison-repeat.jsonl`.
- `/private/tmp/qwen-box-reply-recovery-20260905.json` and `.jsonl`.
- `/private/tmp/compare_qwen_navigation_budget.py` and
  `/private/tmp/validate_qwen_reply_recovery.py` (bounded reproduction harnesses).
- `/private/tmp/qwen-reply-recovery-regressions.log` and
  `/private/tmp/qwen-budget-final-tests.log`.

Stop this implementation slice here. Next is a bounded physical box test after
the operator positions BLAST, the updated hub program is loaded and the host is
restarted. No physical robot was moved, firmware deployed, or model loading
configuration changed in this slice.

## Additional physical-data validation — September 5

The operator requested more simulations, especially with real recorded data.
This is validation only: no production code, prompts, tuning, robot firmware or
test definitions were changed between the runs. The earlier single successful
box detour is not sufficient evidence of physical readiness.

All cases use the loaded Qwen3.8 model, low reasoning, an 8192-token ceiling,
60-second request timeout, at most 12 model decisions and a six-minute wall
budget per case. Runs are sequential, with no competing model requests from
this task, no scripted decisions and no injected model replies. The recorded
dropout case injects four invalid range readings around the first forward pulse.

### What is real, and what is simulated

The only recorded sensor fixture used is
`tests/fixtures/blast_box_scan_20260902.json`, from physical episode
`episode-a2176c7372114872743c5688`. It retains its original ranges, invalid
readings, encoder angles and gyro diagnostics under the legacy encoder-bearing
contract. It is **not** silently relabelled as a new gyro-corrected scan.

The recorded-startup adapter replays that evidence and applies its net heading;
it does not execute the full scan sweep through the simulator. Subsequent
motion, contacts and readings use the approximate box/chair geometry. The
recording does not supply a measured complete room or obstacle outline.
Consequently these replays test behaviour with physical sensor evidence, not
full physical equivalence. The box/chair run without a fixture exercises the
current gyro-aware simulated sweep. The bent corridor is synthetic.

### Confirmed findings

**Outcome: 0/4 episodes completed or reached the goal.** Each had one startup
scan. There were 17 actual model requests: 15 valid final replies and two
60-second transport timeouts. No protocol-reply retry occurred in this series.
One corridor reply used 5224 completion tokens and finished in 58.172 seconds,
so the larger ceiling did provide headroom for that answer, but it did not make
the episode successful. Total run time was approximately nine minutes.

| Case | Real-data contribution | Goal distance | Blocked pulse attempts | End condition |
| --- | --- | --- | --- | --- |
| Recorded startup scan | Original physical sensor evidence; later world approximate | 600 mm | 2 | Next model request timed out |
| Recorded scan + four missing range reads | Same evidence plus short dropout injection | 600 mm | 2210 | Repeated blocked turn until episode deadline |
| Box/chair, current simulated sweep | Approximation of the physical setup, no recorded rays | 438 mm | 2 | Next model request timed out |
| Bent corridor | Synthetic geometry, current simulated sweep | 284 mm | 2340 | Repeated blocked turn until episode deadline |

These blocked-attempt counts are simulator contact events, **not thousands of
separate encounters or model decisions**. The host's deadline advances with
simulated sensor/motor time, so a blocked-pulse loop can exhaust a simulated
episode within a short amount of wall time.

1. **No left/right inversion in these replay starts.** Both recorded cases chose
   (0,-300), the robot's initial right, and executed the corresponding right
   turn. Both then retained the forward detour. At the second model call the
   pose was (42,-315), heading approximately -83 degrees. This does not resolve
   every historical coordinate report, but that symptom was not reproduced here.
2. **A too-close leg still reaches the box corner.** The physical scan's box
   echoes span x=374–399, y=-104–256; padded center bounds end at y=-254. Qwen
   considered y=-300 clear. The simulated box extends to y=-190, and the body's
   swept corner contacts it at approximately (277,-294), heading +5.673 degrees.
   The range-dropout and no-dropout cases reached the same pose. The first
   dropout was therefore not sufficient to explain this failure.
3. **Blocked waypoint turns can repeat without model control.** In the dropout
   case Qwen responded to stalled forward motion by choosing (277,-450), farther
   right. That turn could not start because of body contact. The host repeated
   the blocked turn until its episode deadline: 2210 blocked physical pulse
   attempts in total, of which 2208 were turn attempts. Only three model calls
   occurred. Encoder anchors and pose remained unchanged during those turns.
   This is a host execution-loop defect, not thousands of model decisions.
   The waypoint-following loop retains `route_following` after ordinary turn
   execution; unlike forward continuation, that path does not return control
   on zero motion. It is distinct from the smaller in-leg trim feedback.
4. **Unobserved space is still treated as usable by the model.** In the fully
   simulated box/chair case Qwen chose (0,450), (600,450), then (600,0). Before
   the return leg it had pose (590,486), rightward observed-clear reach zero,
   and explicit `UNKNOWN_NOT_FREE` map semantics. Its returned reasoning
   nevertheless treated unknown, echo-free cells as acceptable for this leg.
   The actual box extends to x=620, so the robot was not yet past it. Contact
   stopped it at (594,386), heading -88.05 degrees. The geometric risk is not
   fixed simply by retaining waypoints or raising the response token ceiling.
5. **Model timeout remains distinct from malformed-reply recovery.** The next
   request in both the recorded no-dropout and simulated box/chair runs timed
   out after 60 seconds. No final reply/reasoning was returned for those timed
   out requests; its exact reasoning cannot be inferred. The new one-retry
   policy handles protocol-invalid replies, not transport timeouts.

The bent corridor reproduced the same host turn loop independently. Qwen first
tried y=-150, reacted to a close forward reading by changing the detour to
y=-300, and then chose to return toward the goal at x=650. It was actually at
(634,-230), heading +18.804 degrees, beside the corner block ending at x=650.
The left turn was blocked; the host repeated it without another model call.
There were seven real model decisions in this case, not 2340 decisions. Again,
the detour returned too close to the obstacle end for the robot's whole body.

The true and estimated final poses agreed exactly in all four cases. The
failures therefore do not require a simulator/odometry coordinate inversion to
explain them. All four had one startup scan, not a repeated-scan loop.

### Implementation boundary after this validation

**Operator clarification:** exploring unknown space is intentional, and short
dropouts or incomplete scans must not become a blanket stop or rescan rule.
The earlier wording about unobserved corridors was too categorical. A failed
route is not proof that attempting an unknown passage was itself forbidden.
The corrective priority is to handle the actual failed movement and return
control without erasing useful evidence, not to add stricter map admission.

Do not proceed to physical validation on the strength of the earlier success.
First make the existing no-progress handling consistent for waypoint turns and
forward motion, then reproduce the blocked-turn case. Separately address the
distinction between tentative clearance and actual successful movement without
introducing a fixed box detour, a default side or an unknown-space prohibition.
Keep response-timeout behaviour
an explicit remaining limitation. None of these fixes was implemented during
this validation-only turn.

Detailed results and raw request/response/controller evidence are preserved in
ignored `local-artifacts/qwen-physical-cases-20260905-*.json` / `.jsonl`, with
the bounded harness `local-artifacts/validate_qwen_physical_cases_20260905.py`.

## Zero-motion handoff correction and bounded reruns — September 5

Following the operator's clarification, this slice adds no scan requirement,
unknown-space prohibition, fixed detour, default side or planner. Two local
changes in the existing BLAST episode adapter address the observed failure:

- After a motor action produces no change in position or heading, the host
  stops automatically repeating that action and supplies the existing
  `MOTION_PROGRESS_STALLED` event to Qwen. Goal, waypoint route, map and pose
  remain intact. Qwen chooses the next attempt; the host does not pick reverse.
- An attempt with measured zero encoder displacement no longer invalidates
  an earlier verified forward path when evaluating bounded reverse. A failed
  attempt did not consume or rotate that path. This does not turn missing
  encoder evidence into verified motion.

The regression replays the three waypoint choices from the failed recorded
case. Before correction it exhausted the episode deadline. After correction it
returns to the fourth planner call with the route and stalled-turn event intact,
permits the explicitly selected reverse, and retreats from x=277 to x=232 mm.
There are six blocked pulse attempts rather than 2210, one startup scan, and no
new observation requirement. This is an execution regression with scripted
choices, not a real-Qwen navigation success. The targeted 675-test gate passes;
the final 12-test simulation-adapter gate also passes after refining the fixture
route tails. These counts overlap.

### Real-Qwen reruns

The first two runs retain low reasoning, the 8192-token ceiling, 60-second
requests, 12-decision limit and six-minute wall budget from the failed series.
The third is the operator-authorized reasoning-off comparison; no production
default changed for it. No model answers or routes were injected.

| Case | Outcome | Scans | Blocked pulse attempts | Real model requests |
| --- | --- | --- | --- | --- |
| Recorded physical scan + four missing range reads, low | **Completed**, ground-truth goal check passed, 40 mm from goal | 1 | 6 | 10 |
| Bent corridor, low | Not completed, 521 mm from goal; next request timed out | 1 | 6 | 4 |
| Bent corridor, reasoning off | Not completed, 315 mm from goal at decision limit | 3 | 0 | 12 |

In the successful recorded case Qwen first chose a right detour at y=-300,
contacted the corner, chose a farther side waypoint, received feedback that
the turn had not moved, and **independently chose REVERSE**. It then reached
y=-500, passed the box via x=600 and x=800, returned to the goal line, aligned
and completed. Final true/estimated pose was (769,-25), heading +5.673 degrees.
All ten model replies completed; there were no reply retries. This validates
self-correction after contact in an approximate world with recorded startup
evidence, not a contact-free route or full physical equivalence.

The low-reasoning corridor run also returned control after its blocked turn;
the former 2340-attempt host loop did not recur. At the next request the pose
was (298,-140), heading -32.832 degrees. Its history included a zero-motion turn
and `MOTION_PROGRESS_STALLED`. The request then timed out after 60 seconds, so
no model recovery choice was received. Available actions were FOLLOW_WAYPOINT
and SCAN_FRONT_ARC: an earlier **partly moved** turn still made the existing
reverse-evidence predicate reject REVERSE. The zero-motion-only change does
not resolve that separate partial-motion case, and no unseen reasoning is
claimed to explain the timeout.

Reasoning off produced decisions quickly and no contact events, but this new
stochastic route had two rejected legs and three scans before exhausting twelve
decisions. It is not counted as a completed corridor run, and different route
choices mean it is not a causal comparison of reasoning modes.

Evidence is retained as
`local-artifacts/qwen-physical-cases-20260905-*-corrected*.json` / `.jsonl` and
`local-artifacts/validate_qwen_physical_cases_20260905_corrected.py`. Regression
logs are `local-artifacts/blocked-turn-negative-control.log`,
`local-artifacts/blocked-turn-regressions.log`, and
`local-artifacts/blocked-turn-final-execution-tests.log`.

Stop this implementation checkpoint here. The repeated zero-motion host loop
is corrected and one recorded-data detour now self-recovers to goal. Corridor
completion, reverse availability after partly achieved motion and model timeout
handling remain unvalidated/unresolved. No physical robot was moved, no firmware
was deployed and no navigation restrictions were tightened. The updated hub
program from the earlier trim-pulse change still needs deployment before a
physical test.

## Physical startup regression and bounded correction — September 6

The two physical attempts `episode-145dc696bf80787d607b80a7` and
`episode-28ec3a5d571984dd61e20e41` stopped before any navigation model decision
with `scan_sweep_observation_unverified`. The second run had about 8.4 V battery,
so battery depletion does not explain the repeated validation failure.

An offline reproduction changed one gyro sample by just 0.0000004 degrees:
the original sample passed and the changed one failed. Bearings are serialized
to six decimals, but the validator compared them to unrounded gyro differences
more strictly. Angle comparisons now allow the existing serialization precision;
legacy recorded scans remain readable. This is not a new mechanical margin.

The other correction is limited to closing the startup circle. The earlier
30-motor-degree closing pulse is back to 15 degrees. The shared physical/simulated
completion helper chooses small pulses near the endpoint and can reverse a small
overshoot. Its target is 360 degrees within 5 degrees, with actual measured pose
retained (never replaced by an invented zero). It has a bounded 24-pulse limit.
No route selection, model side choice, or new mandatory-rescan rule was added.

Validation: 88 observation-monitor tests and 156 execution/communication/simulator
tests pass. A new parameterized monitor test covers 20% and 10% underturn, nominal
motion, 15% overturn, fine-decimal gyro readings and a deliberately injected
14-degree endpoint overshoot corrected with a reverse trim. The existing partial
startup test also verifies that incomplete evidence reaches the planner with the
measured pose instead of terminating startup. These are execution tests, not
independent Qwen route successes.

The corrected host and hub program were reloaded. Physical attempt
`episode-0eacdb97b7a5fc279faa17a2` completed startup with 357.51919876 degrees
of measured yaw (2.480801-degree endpoint residue). The scan passed validation
and reached Qwen, which selected a right-side detour. BLAST travelled about
316 mm along that first leg. A later range dropout caused a model-selected
front scan within the same episode, retaining the route and goal; that scan
also passed validation. The full detour FAILED: repeated scans lost the box
from the current map, and BLAST returned towards the start. The operator then
observed sideways travel while the estimate said forward. The run was stopped
at the operator's request to restart/reorient; status confirmed IDLE/stopped.
The measured 357.52-degree endpoint was not an independent physical orientation
measurement and does not establish an accurate full navigation frame. Logs
remain in `local-artifacts/navigation-dashboard.jsonl`.

## Full active BLAST path audit after that failed run — September 6

Scope: the current physical BLAST path, from initialization through observation,
scan projection, map context, model decisions, route admission, turn/advance
execution, waypoint completion and recovery. This is not a new planner, an EV3
migration, a merge, or physical validation of either robot.

Three concrete failures were identified:

1. **Heading ownership:** the recent adapter change replaced the pose heading
   with absolute episode-relative gyro yaw after motion. Nine intervals in this
   run had identical wheel encoders but changing yaw during planner waits. One
   43.085-second pause changed yaw by 4.99758 degrees. This is consistent with
   idle gyro drift accumulating into the navigation frame; it does not alone
   prove the entire physical left/forward discrepancy. The executor now applies
   command-local gyro deltas (encoders when gyro is absent). Turn feedback and
   the map use that same accumulated heading. Raw yaw during an idle wait is no
   longer applied as if the robot had turned. Real motion/scan drift and slip
   still require physical verification; there is no absolute room reference.
2. **Map memory:** `[-1:]` deliberately discarded older obstacle evidence to
   avoid the earlier inflated-box union. That also forgot the actual box when
   later scans faced elsewhere or returned no echo. Retained measured points
   now survive no-return/partial views. Nearby new hits replace previous hits;
   measured rays passing through an old point can contradict it. Missing range
   never clears the map. This uses the existing coarse-map scale, not object
   recognition, a new occupancy framework, or host-selected detours.
3. **Action availability:** reuse of a scan depended on complete scan status,
   three named front/flank rays and an action-name history. This excluded forward
   travel after turns and next to a box even when evidence supported the desired
   direction. Admission now compares measured/viewed rays to the robot's actual
   heading and displacement. A partial scan can contribute usable rays. Settled
   no-return evidence allows bounded exploration without marking space as free;
   currently measured close obstacles still override earlier clearance.

Unchanged ownership: Qwen chooses the side, route, waypoints and replans. Motor
pulses finish its selected intent. Goal coordinates remain anchored to episode
start; waypoint tails remain model-owned. Stop, goal completion, motion failure,
scan results and model timeout are distinct events. No new default turn,
hard-coded box route, mandatory scan cadence or model-prompt rule was added.

Regression source `tests/fixtures/blast_navigation_regression_20260906.json`
contains four actual scan results and nine matching-encoder pause samples from
`episode-0eacdb97b7a5fc279faa17a2`. Tests cover drift during idle waits, true turn
feedback, persistence through those front/partial scans, bounded no-return
waypoint following, repeated-hit replacement and measured contradiction.

The final targeted production/adapter/map/monitor/transport suite passed 340
tests in 118.377 seconds, including recorded-data, memory, turn-feedback,
zero-progress and blocked-scan regressions.
This is NOT a count of successful model-driven routes. The first intermediate
Qwen run stopped on a request timeout 468 mm from goal after two scans and six
blocked moves. It also exposed missing metadata in the new cluster summary;
that issue was caught and corrected before the next run. That run exposed a
simulator/physical mismatch: the simulated front scan always executed sixteen
pulses and built a complete scan even if the world blocked a pulse. This caused
`BLAST scan result is invalid` after contact near the box (600 mm from goal).
The simulator now stops at the blocked pulse and uses the same partial-scan
builder as the physical monitor. A regression at the observed stalled pose
`(277, -294, 5.673 degrees)` verifies that behavior. No route is supplied by the
simulator. Final actual-Qwen validation is recorded separately below. No new
physical run has been started.

### Final bounded real-Qwen result

`local-artifacts/qwen-navigation-audit-20260906-partial-recovery.json` and
`.jsonl` record the final tested code (production file hashes included).
Actual `qwen/qwen3.8-27b`, reasoning low, 8192 output limit: **completed**, 42 mm
from the 800 mm goal, 10 model requests/responses, one startup scan, no subsequent
scan, one model-selected reverse, zero route rejections. World and estimated
final poses both were `(766, -25, 5.673 degrees)`. Runtime was 201 seconds.

This was **not contact-free**: six motor pulses were blocked before Qwen chose
to reverse, make more room and continue. No decision or route was scripted.
Startup was the recorded September 2 scan; four range-dropout reads were
injected. Later range and motion came from the simulated box/chair world, not
from a complete replay of physical BLAST. Today's four scan results and idle
drift samples are separately covered by the recorded-data regressions. Do not
describe either evidence set as a guarantee of physical success.

Stop this code checkpoint here. Next physical acceptance: operator positions
BLAST facing the real goal, reload the changed host, establish a new episode
reference, compare physical orientation against the map before/after startup,
then follow Qwen's detour. Current physical host still has the previous code
loaded and the previous episode is stopped; do not silently resume its map.

## Physical acceptance after reload — September 7

Physical episode `episode-f37cff92ccec6cae569de559` used the exact three production
file hashes of `qwen-navigation-audit-20260906-partial-recovery.json`. The old
host was stopped and reloaded before connecting BLAST. Before movement the hub
reported about 8.08 V, IMU ready/stationary and 259 mm measured forward distance.
No production code was changed during this physical run.

**Result: completed, independently confirmed by the operator.** BLAST drove
around the box on its left and stopped at the intended goal. Final estimated
pose was `(808, -26, approximately -6 degrees)` for target `(800, 0)`, a 27 mm
estimated residual. The operator explicitly confirmed both that startup ended
in approximately the original real heading and that the map matched reality
during travel, then confirmed BLAST was physically at the goal beyond the box.

Qwen selected the waypoint sequence `(150,0)`, `(150,450)`, `(600,450)`, `(600,0)`
and `(800,0)`. Recorded action counts: 39 forward actions, five turn actions,
and three scans total (one mandatory surroundings scan, two model-selected front
scans after range dropout). Startup coverage was 359.339 degrees. The later
front scans were respectively complete and partial; the partial scan ended
about 91 degrees from its start, and Qwen realigned and continued. It did not
terminate the mission or erase its target/remaining waypoint.

All ten actual Qwen requests returned decisions, with low reasoning and the
existing 8192 output limit. Model response times ranged from 4.563 to 55.433 s.
No `controller_action_failed` was logged. Final state was IDLE with terminal
reason `completed`, speech status `completed`, and no speech error. Episode
duration was about 7 min 43 s. No contact-free assertion is inferred merely
from the absence of logged errors.

This accepts the present physical single-box baseline, not every obstacle
course, long-range room localization, EV3, or concurrent multi-robot behavior.
Keep the tested version stable before further navigation changes. This run
did not commit, push or merge the branch.

### Remaining map/heading discrepancy

After arrival, the operator noted that the return leg behind the box looked
more diagonal on the map than the actual travel. Qwen's planned leg was still
orthogonal: `(600,450)` to `(600,0)`. Recorded poses moved from about `(589,445)`
to `(535,-8)`, with headings around -96 to -98 degrees instead of -90 degrees.
The renderer connects those recorded poses with equal axis scales; it does not
introduce a diagonal route. These records do not establish how much of the
remaining discrepancy is physical drift versus heading/odometry error. Preserve
this as a known limitation of the accepted box baseline; no further navigation
code was changed for the commit-and-merge checkpoint.

### Pre-merge quality gate — September 7

The full hardware-free `scripts/quality_check.sh` run completed in about 146
seconds: 2,083 tests, two failures, no errors. JavaScript syntax checks passed.
Both failures are existing structural limits in `test_code_health.py`, not
navigation behavior tests: four oversized production modules and the oversized
BLAST episode `run` function. Current module line counts are 2,699 (episode
adapter), 2,019 (observation monitor), 1,960 (dashboard logic) and 1,949 (map
presenter); their limits are respectively 1,800, 1,800, 1,900 and 1,900. The
episode `run` function is 671 lines against a 500-line limit.

All four modules and the episode function already exceeded these limits at
the previous branch commit `874fd4b`. The limits have not been relaxed and no
production refactoring was added during this checkpoint. The successful
physical run therefore does not constitute a green full quality gate. Commit
and push preserve the tested baseline; merging with this known structural debt
requires an explicit decision after disclosure of these results.
