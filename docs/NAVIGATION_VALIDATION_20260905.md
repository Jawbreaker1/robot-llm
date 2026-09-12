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

## Post-gesture physical regression — September 7

Tested the committed `main` checkout at `d12334d` without changing production
code or navigation settings. BLAST started in front of the box with a goal
800 mm ahead. The running console used Qwen3.8-27B, low reasoning, and the
existing 8192-token output ceiling.

### Gesture preparation required recovery

The normal robot-turn endpoint returned the Qwen-selected greeting
“Ready to roll! Watch this claw snap!”, with `happy` and `claw_flourish`.
The operator reported that the arm looked correctly restored. However, the
controller's four-second completion wait expired during the gesture. Its last
observation showed body angle 156° against the requested 158°, and
`motion_active=true`. The hub's body-motor completion tolerance is 1°.
The controller stopped the motion and reconnected; restoring the idle face
also failed across that connection change.

A separate existing `--gesture restore` probe reproduced the completion
timeout at 156°, without commanding the wheels. A subsequent reconnected
observation reported the arm at 158° and all motors inactive, after which
navigation was started. This preparation therefore did **not** pass as a
hands-off gesture-to-navigation transition. The observed completion sensitivity
and its connection recovery remain unresolved; no tolerance or timeout was
changed to obtain this result.

### Obstacle passage completed; goal acceptance was too early

Episode `episode-f97344bd54a7c1970f18c0b6` ran for approximately 5½ minutes.
Qwen authored the route `(150,0) → (150,-300) → (600,-300) → (600,0) → (800,0)`.
It retained the route through missing range readings and a short reverse;
valid range returned and forward travel resumed. No manual drive command or
replacement goal was injected during the navigation episode.

- One startup surroundings scan, about 361° coverage and 1° restoration error;
  the operator confirmed that the real heading remained approximately correct.
- No subsequent scan. Thirty forward actions, one reverse, and five turn
  actions, including an intermediate heading correction.
- Nine Qwen requests, all returning structured decisions in 6.3–21.1 seconds.
  No navigation controller failure or speech error was logged.
- The operator confirmed passage around the right side and arrival behind the
  box, but noted imperfect turns and a small remaining heading discrepancy.
- Final estimated pose `(657,12,-3.184°)`, target `(800,0)`: 144 mm residual.
  After the last right turn, Qwen chose `COMPLETE` because this was inside the
  existing 150 mm goal radius. There was **no final forward leg**.

The operator independently confirmed that BLAST stood just behind the box,
with her back almost touching it, and had only turned toward the goal without
advancing afterward. Runtime status was `IDLE/completed`, but this does not
validate arrival at the intended goal point. The final-goal acceptance radius
allowed an early finish; it is not evidence that Qwen lost the goal. The cause
of the remaining physical-versus-estimated turn discrepancy is not established.

Result: physical obstacle passage and continuation after range dropout are
confirmed. Automatic recovery from accessory completion errors and the intended
final approach still need correction before accepting the combined test.
Evidence is in `local-artifacts/navigation-dashboard.jsonl` for the episode
above and `/tmp/blast-gui-arm-gestures-20260907.log` for the gesture failure.

## Final-approach correction and heading audit — September 7

Goal arrival is now 50 mm, independent of the 150 mm grid resolution. The final
waypoint uses that same positional check, so the intermediate-waypoint lateral
allowance cannot consume the final target while completion is still forbidden.
Other waypoint tolerances, turn calibration, recovery policy and model-owned
route decisions are unchanged.

The recorded `(657,12,-3.184°)` endpoint is no longer eligible for completion:
its final waypoint remains and requests forward motion. A second regression
covers the final target with 60 mm lateral error, which must request a turn
instead of being discarded as an already-reached intermediate waypoint.
The existing physical-adapter simulation test now explicitly includes a last
forward leg after the return behind the box, both with valid range and three
missing readings. Both reach within 50 mm with one scan and no collision.

Two actual Qwen3.8-27B runs (low reasoning, 8192-token ceiling, at most ten
decisions each, three transient invalid readings) validated the smaller goal
radius: `blast-box-front` completed 50 mm from its goal and
`blast-measured-box-and-chair` completed 5 mm away. Each used one startup scan,
zero blocked moves, and matching simulated/estimated final poses. The second
world is based on the measured box/chair arrangement; these runs used simulated
sensor geometry, not replayed raw scans. The final-waypoint consistency change
was subsequently validated in the focused hardware-free suite (314 tests).

An exploratory variant with permanently missing range on the last approach did
not complete within the scripted planner's seven-decision budget. This is not
a passing permanent-outage test; the correction above targets arrival and
transient dropouts, not unlimited blind travel.

Heading audit: the map and planner use the motion executor's pose, not a second
raw-gyro orientation; raw gyro/reference fields are removed from model context.
Recorded turns ended approximately 3–6° from their intended cardinal headings,
within the current 12° waypoint alignment allowance. That alone does not prove
whether the remaining real-world discrepancy is wheel drift or gyro error.
Recomputing the 30 recorded forward displacements using each pulse's gyro turn
instead of its encoder turn changes the total displacement by only about
`(-1.8,-3.3)` mm (maximum 1.7 mm for one pulse), insufficient to explain the
reported larger skew. No speculative gyro bias, axis flip or calibration
change was added. Physical heading-versus-floor verification is still needed;
this is not a claim of corrected physical angular accuracy.

## Physical regression: heading drift and reverse after a turn — September 8

Episode `episode-7e439e36f0a3e864cc67d6af` tested the local working tree based
on `fead6b0`, after the hardware-free suite passed all 2,107 tests on both
Python 3.9 and 3.13. Qwen3.8-27B used low reasoning and the existing 8,192-token
output ceiling. Piper, the dashboard and the controller used the normal live
path; no manual wheel command or substitute route was injected.

This physical test **failed**. The operator reported that BLAST backed into
EV3 and that the real angular deviation was leftward while the map showed a
rightward deviation. The final approach was therefore not validated.

- The initial surroundings scan completed with 359.23° coverage and 0.77°
  reported restoration error. These are sensor-derived figures, not independent
  physical angle measurements.
- Qwen planned `(0,450) → (600,450) → (600,0)` and later retained `(800,0)`
  as the final waypoint. The robot followed the first two legs without another
  scan. The recorded pose before the return leg was approximately `(579,396)`.
- Six model responses arrived in 22.0–26.4 seconds. There was no model timeout
  or reported speech failure. Actions comprised 23 advances, four turns
  (including one trim), two reverses, the startup scan and one front-arc scan.
- After the turn toward `(600,0)`, repeated `NO_VALID_DISTANCE` readings removed
  `ADVANCE` and `FOLLOW_WAYPOINT` from the model's available actions. The route
  and goal remained in context. Qwen selected two reverse pulses, approximately
  9 cm total by odometry, followed by the front-arc scan.
- That scan reported `restoration_unverified` with -62.65° IMU restoration
  error. The episode faulted with `blast_scan_restoration_unverified`. A stop
  was then sent; the controller confirmed inactive motors and IDLE state.

### Measured heading discrepancy

After the stop, observations at Unix milliseconds `1788901540900` and
`1788901610771` had identical drive encoders (`left_drive=1334`,
`right_drive=2572`) and `motion_active=false`. Reported gyro heading changed
from -256.12570° to -230.27094°: **25.85° in 69.87 seconds**. The operator
explicitly confirmed that BLAST had not been moved by hand. Both samples
reported `imu.ready=true` but `imu.stationary=false`.

Across the 23 advance commands, the recorded wheel deltas imply about 4.17°
net left yaw under the existing calibration, while the command-local gyro
changes imply 6.60° net right yaw. Neither encoder odometry nor gyro alone is
independent ground truth, but the disagreement and confirmed stationary drift
support the operator's observation. The current executor replaces its heading
with the command-local gyro delta whenever that value is finite. Excluding
long model waits does not exclude gyro drift during physical commands. The
cause of the reported gyro drift itself is not yet established; no sign flip,
bias constant, calibration adjustment or navigation patch was applied.

### Separate reverse-admission defect

`_completed_advance_allows_bounded_reverse` skips verified turns while looking
back for unused forward pulses. A forward pulse before a turn therefore still
authorizes reversing in a different direction; it is not evidence of clearance
along that new reverse path. This needs a focused regression using the real
sequence, separately from the heading-quality problem. The physical failure
must not be treated as a successful goal-arrival or recovery validation merely
because the hardware-free suite is green.

### Stationary IMU isolation and chronology — September 8–9

The operator approved a stationary diagnostic, without a navigation run or
firmware change. A standalone program initialized only `InventorHub` with the
existing BLAST orientation, disabled the display and sampled the IMU every two
seconds. No motors, accessories or speech were initialized; no calibration
settings or heading resets were written. The installed firmware reported
`4.0.1`, `local-build-v4.0.1-dirty on 2026-09-07` throughout.

All 31 samples reported `ready=true` and `stationary=true`. Heading changed
from -0.00045° to -1.56881° in 60.523 seconds: **1.57° per minute**, compared
with 25.85° over 69.87 seconds in the earlier normal-runtime observation.
Immediately before isolation, the normal runtime still reported
`stationary=false`, unchanged motor positions and, eventually, `ready=false`.
The probe log is `/private/tmp/blast-gyro-isolation-20260908.log` (temporary
local diagnostic, not a committed test fixture).

This shows that the severe drift was not persistent under the isolated
program with the same installed firmware. It does **not** distinguish a
program-lifecycle/calibration-state effect from accessory activity or an
indirect firmware interaction: changing programs also changes operating
conditions. The normal controller was requested to reconnect afterward but
produced no fresh observation, so that comparison remains incomplete.

The September 6 audit above already recorded 4.99758° of yaw change during
43.085 seconds with unchanged wheel encoders. Therefore drift predates the
September 7 v7 audio-firmware update; an earlier custom-firmware contribution
is not excluded. The hub audio patch does not directly modify the IMU driver,
and the BLAST axis configuration has not changed since August 5. The onset
and root cause remain unproven. No navigation, calibration or firmware fix
was made during this diagnostic.

### Confirmed brake/coast calibration interaction — September 9

Heading reliability is the priority before another navigation acceptance run.
After the operator powered BLAST on, the ordinary runtime reported
`ready=true`, `stationary=true` and inactive motors. This narrows the previous
isolation result: the ordinary program does not itself prevent calibration
immediately after startup.

Review of the exact pinned upstream Pybricks commit
`4104553405decb0384bcfb030fbfcb4b5a9854cc` found that
[`pbio_imu_is_stationary`](https://github.com/pybricks/pybricks-micropython/blob/4104553405decb0384bcfb030fbfcb4b5a9854cc/lib/pbio/src/imu.c#L455)
requires both stable sensor data and `pbio_dcmotor_all_coasting()`.
The bias-update callback returns without updating calibration otherwise.
Thus BRAKE or HOLD blocks calibration even when encoder positions are stable.
Our drive/turn pulses finish in BRAKE, `stop_all()` brakes all motors, and
accessory poses finish in HOLD. `motor.done()` does not establish the motor
state required for calibration.

A 50-second physical A/B/A probe, with no motor-rotation command or firmware
change, reproduced the gate in one program session:

| Motor state | Duration | Stationary samples | Heading change |
| --- | --- | --- | --- |
| Coast before | 10.086 s | 6/6 | +0.2971° |
| Brake | 20.164 s | 0/11 | +1.1816° |
| Coast after | 20.204 s | 11/11 | +1.2810° |

All samples remained `ready=true`; the readiness timeout is ten minutes, so
this short probe does not test expiry. Left-wheel and accessory encoders were
unchanged. The right-wheel reading varied by one motor degree (50–51) and
returned to 50. Evidence: `/private/tmp/blast_gyro_brake_probe_20260909.py` and
`/private/tmp/blast-gyro-brake-probe-20260909.log`.

This proves the calibration-blocking interaction, not that it is the sole
cause of the earlier 25.85° drift. Braking was present since August 5
(`d12047c`); accessory HOLD was added September 7 (`f47b212`). Therefore the
underlying brake condition is not a newly introduced September defect.

The next bounded correction should address the hub motor lifecycle: complete
and brake motion, then release motors at rest so automatic calibration can
resume. Check that the geared sensor arm retains its navigation reference
when released, and validate gyro recovery after movement before attempting
the box again. Do not compensate with a guessed angular bias, discard the
gyro globally or add planner rules. No production fix was applied in this
diagnostic; the normal controller was reconnected afterward.

### Bounded motor-release correction and physical check — September 9

The hub's existing background poll now releases all motors once a commanded
motion has completed and settled for 150 ms. Existing BRAKE endpoints and
accessory HOLD targets remain in place during completion. Any moving motor
defers release for all motors; observation, faces and speech do not postpone
the timer. No gyro thresholds, bias values, angle offsets, planner rules or
firmware were changed. Three focused tests execute the actual hub helper and
dispatch hook, covering all nine motor commands, active motion, one-shot
release and unaffected audio polling.

The real `BlastBLERuntime` deployed this same modified hub program for a
stationary physical probe. It observed the startup baseline, sent normal
`stop`, moved the body motor 80° from its 158° reference and returned it,
then commanded the claw's existing 202° position. No drive or turn command
was sent. This is the normal hardware runtime, not a simulated sensor model.

- After `stop`, `stationary` became true at the first subsequent one-second
  sample (1.14 s); heading changed +0.1206° over 13.745 s.
- After the accessory return, `stationary` similarly became true at 1.11 s
  and stayed true for all remaining samples. Heading changed +0.3417° over
  22.921 s. The body encoder stayed within 157–158°, inside the existing
  navigation-reference tolerance. The claw stayed at 202°.
- Both drive encoders ended at their initial values (-106°, 50°); the right
  reading occasionally varied by one motor degree. The probe finished
  successfully and the ordinary dashboard controller reconnected online,
  with `ready=true`, `stationary=true` and inactive motors.

Evidence: `/private/tmp/blast_validate_idle_release_20260909.py` and
`/private/tmp/blast-validate-idle-release-20260909.log`.

The formerly persistent calibration block is corrected in these physical
stop/accessory cases. This does not yet validate wheel stopping distance,
turn accuracy, full-scan closure or long-duration navigation. The next bounded
physical check is turn/scan accuracy and gyro recovery afterward, before
another box-navigation acceptance attempt.

Validation: `scripts/quality_check.sh` passed all JavaScript syntax checks
and **2,110 tests** in 146.565 seconds on Python 3.13. The three new idle-release
tests also passed on Python 3.9. Focused expression and hub-speech suites
passed (11 and 22 tests respectively). `git diff --check` passed. The final
hub-source change is 25 added / 5 removed lines; no new planner code was added.

### Physical turn and surroundings-scan check — September 9

The operator requested continuation of the bounded hardware check. The probe
used the production `BlastObservationMonitor`, `BlastNavigationMotionExecutor`
and surroundings-scan implementation with the corrected hub runtime: one
four-pulse left quarter-turn, ten seconds at rest, one right quarter-turn,
ten seconds at rest, then one full surroundings scan and twenty seconds at
rest. No translation command or Qwen request was issued. This tests motion
execution and IMU recovery, not autonomous route planning or goal arrival.
The operator-authorized turn test used the existing no-return continuation
opt-in; measured close obstacles still used the production turn gate. The
scan used the ordinary perception-only startup permit, with real sensor data.

- Left turn: command-local gyro change corresponded to **92.079° left**;
  all four pulses completed. Idle yaw changed -0.0262° over 9.271 s.
- Right turn: command-local gyro change corresponded to **91.298° right**;
  all four pulses completed. Idle yaw changed +0.1310° over 9.118 s.
- The sole full scan completed with 17 pulses, reported **361.2198°**
  coverage and **1.2198°** endpoint error, `restoration_verified=true`.
  Idle yaw changed +0.1909° over 19.799 s afterward; 19 of the 21 samples
  reported stationary, with the first two still in post-motion settling.
- The final gyro direction was approximately **1.70°** clockwise from the
  initial direction after the entire sequence, modulo the full revolution.
  The sensor-arm encoder remained at 158°, the claw at 202°, and the final
  observation reported ready, stationary and no active motion.
- The scan retained nine measured ranges and seven no-return rays in its
  sixteen angular samples. No-return values were not converted into obstacle
  distances. The encoder-only scan closure residue was -10.93°, with 3 mm
  common-mode residue: wheel-derived angle and gyro are still not identical.
  This test does not independently calibrate wheel geometry or measure slip.

The probe completed without error, additional scans or replanning, and the
dashboard controller was reconnected online. No production changes were
made during this validation. The operator was asked to confirm the actual
quarter-turns and final physical direction; that independent confirmation
was still pending when these measurements were recorded. Do not label the
sensor-derived angles as independently measured floor angles.

Evidence: `/private/tmp/blast_turn_scan_validation_20260909.py` and
`/private/tmp/blast-turn-scan-validation-20260909.log`.

### Single scan repeated for visual inspection — September 9

The operator had not watched the previous sequence closely, so physical
direction remained unconfirmed. On the explicit request to repeat while
BLAST was powered on, one surroundings scan was run through the same
production controller and scan code, without quarter-turn tests or any
translation command. The dashboard was unavailable at the start; the probe
owned the BLE connection directly.

The scan completed normally with 16 pulses, **356.1274°** gyro-derived
coverage and **3.8726°** endpoint residue, `restoration_verified=true`.
Encoder closure residue was -2.11° with 1 mm common-mode residue. During
4.682 s of post-scan observations yaw changed +0.4402°; the final three of
six samples reported stationary. The sensor-arm encoder remained at 158°
and the final observation was ready and motion-inactive. This is a second
successful scan-execution check, not independent confirmation of a physical
360° rotation; the operator's visual assessment was requested after completion.
No production code was changed for the repeat.

Evidence: `/private/tmp/blast_single_scan_20260909.py` and
`/private/tmp/blast-single-scan-20260909.log`.

After the probe, the existing Piper and Whisper services and BLAST web
console were restarted with their previous settings. The console responded
again at port 8765 and a controller reconnection was requested; no mission
was started.

The operator subsequently confirmed that the single scan looked very
reasonable and estimated the visible deviation to be smaller than the
reported 4°. This is qualitative confirmation of the single scan's physical
return direction, not a measured angular reference or long-run validation.

### Heading-history review and next validation boundary — September 9

The operator highlighted the remaining requirement: direction must remain
consistent with the physical robot over a longer run, without growing error.
Git history confirms a relevant recent change in `d883fae` (September 7):
`BlastNavigationMotionExecutor.execute()` began applying command-local gyro
yaw to its pose heading after each completed movement. In the preceding
committed version, that executor retained its wheel-derived motion heading;
the episode adapter used gyro reanchoring for startup restoration. The
intermediate absolute-heading variant described in the September 6 audit
was superseded by the command-local implementation in the same checkpoint.

This provides a concrete route by which drifting gyro readings can change
both the displayed direction and subsequent steering, even if the wheel
estimate indicates a deviation the other way. The September 8 recording
already contains that disagreement (approximately 4.17° left from wheels
versus 6.60° right from gyro across advance commands). It does not establish
wheel odometry as ground truth or prove every observed discrepancy had that
single cause. The motor-release correction addresses calibration availability;
it has not yet validated accumulated position and heading accuracy.

Next proposed check: a bounded 3–5 minute run with straight travel, multiple
quarter-turns, normal scans and planner waits. At a few visible floor
references, compare actual direction/position with map pose and the existing
per-command gyro/encoder logs. Inspect the first divergence, not just the
final position. Keep the production navigation path and do not introduce
another filter, angle offset or route rule before evidence warrants it.
No longer run or additional production change was performed in this review.

### Open-floor production run — September 10

Episode `episode-e8466e0af05131abc476ebc9` ran with the motor-idle release
correction, the ordinary physical BLAST controller and remote
`qwen/qwen3.8-27b` (low reasoning). No navigation code or goal configuration
was changed during the run. The operator reported a relatively open floor
without boxes.

Test setup discrepancy: the submitted goal text said 1000 mm, but the
authoritative directional mission remained configured for 800 mm. This was
disclosed during the run. This result validates the configured 800 mm target,
not the requested one-metre distance or correct parsing of distances in text.

The episode reached its goal after approximately eight minutes. Final map
pose was `(777, 22) mm`, heading `2.236°`, with 32 mm estimated target
distance inside the configured 50 mm goal radius. The operator confirmed
both the approximately 80 cm physical displacement and the near-original
heading matched the map. This is useful qualitative physical confirmation;
it is not a precision floor measurement or a general long-route guarantee.

Navigation efficiency failed: logs contain 21 ADVANCE commands, four
REVERSE commands, two scan actions (including startup), and a left/right
quarter-turn pair. The operator heard repeated sensor-related complaints
and observed the reversals. Request `fc39d4b20e164cf6a3454bd060968cc8`, for
example, exposes only TURN_LEFT_90, TURN_RIGHT_90, REVERSE and SCAN_FRONT_ARC
in both available_actions and the strict output action enum. Its response
chooses REVERSE while its speech says to charge ahead. Earlier request
`88a56f4c208b4f169b46874f12071388` has the same restricted enum even though
the model's reasoning explicitly selects FOLLOW_WAYPOINT.

Live observations repeatedly report raw distance 2000, normalized in planner
context to null / NO_VALID_DISTANCE. Pybricks documents 2000 as the return
value when no valid distance is obtained. This is not evidence of a broken
sensor: on the operator-confirmed open floor, absent echoes from distant
objects are a plausible normal explanation. It also does not prove every
unobserved cell is free. The demonstrated defect is the handling of this
condition: forward/follow actions disappear and leave incompatible turns,
reversals or scans as the only selectable actions. Do not describe these
reversals as obstacle-driven route choices or successful recovery.

The browser had retained stale mission status. Reloading the existing tab
restored the completed episode, map, grid and trace; the map panel was made
visible. No frontend source change was made in this check.

Evidence: `local-artifacts/navigation-dashboard.jsonl` records from Unix ms
1789049984571 through 1789050471747, plus the read-only partial timeline
`/private/tmp/blast-open-run-watch-20260910.jsonl` and operator confirmation.
Next correction should address coherent no-echo/action availability, not
another planner or steering offset. The one-metre text/configuration mismatch
also remains unresolved. Neither correction was implemented during this run.

### No-echo admission simplification — September 10

The follow-up removes `_recent_evidence_allows_bounded_advance` and its
forward-only exception paths. Fresh Pybricks no-echo responses now permit
an ordinary forward pulse without depending on distance or heading from
an older scan. Current close measurements and genuinely invalid values
(including None, negative and non-finite samples) still refuse that pulse.
The existing body/encoder checks, startup scan, route geometry checks and
between-pulse observations remain. No-echo samples still reach model/map
context as unknown rather than a measured 2000 mm clearance; known mapped
obstacles are not erased. The shared model prompt explains this distinction.
No gyro, motor-idle, turn or hub code was changed in this follow-up.

Two obsolete scan-expiry tests were consolidated into one fresh-range
contract. The existing persistent-no-return simulator test now checks
completion of an 800 mm waypoint using the real September 10 startup scan.
That test uses a scripted planner and is an execution regression, not model
validation. A route-free `blast-open-floor` scenario and the recorded scan
fixture were added for reproducible model validation. Subsequent sensing is
simulated world geometry, not a claim to replay all physical sensor behavior.

Real Qwen validation (`qwen/qwen3.8-27b`, low reasoning, 8192 output tokens):
the open-floor case with that recorded startscan reached the goal, with 39 mm
simulated final distance, one startup scan, no blocked moves and no reversals.
Qwen selected FOLLOW_WAYPOINT directly to (800, 0), then COMPLETE. The
ordinary executor followed that route without returning for more planning
on each no-echo pulse. Evidence:
`/private/tmp/blast-open-floor-qwen-20260910.json` and
`local-artifacts/navigation-simulation.jsonl`.

Validation: `scripts/quality_check.sh` passed 2109 tests in 149.821 seconds,
including JavaScript syntax checks. The final prompt and simulation-adapter
tests were also rerun separately (37 and 13 passing). A new physical run
with this host change has not been performed; the running console must be
restarted to load it. Do not confuse the previous physical gyro validation
with validation of this newly simplified no-echo policy.

The second real-Qwen case used `blast-measured-box-and-chair`, the September
2 recorded box scan, and four injected no-echo reads. It did NOT pass:
decision_budget_exhausted after 12 model decisions, 402 mm from goal,
one startup scan, six blocked simulated moves, no runtime exception.
Qwen chose a left-side detour, passed along the box and tried to descend
behind it. It corrected a rejected diagonal leg but repeatedly retried a
descent after movement stalled. At the repeated stall the front range was
1710 mm / MEASURED, and FOLLOW_WAYPOINT was available. Thus the repeated
attempts in this case are not the removed no-echo action-enum restriction.
The forward ray did not establish clearance for the complete robot body.
Qwen eventually changed its maneuver, but did not reach the goal within the
bounded run. This remains a failed navigation case, not a successful box
validation or a reason to increase the budget until the score turns green.
Evidence: `/private/tmp/blast-box-chair-qwen-20260910.json` and the same
simulation diagnostic log. No follow-on planner/geometry changes were made.

Next bounded step: physically validate the open-floor no-echo correction
after restarting the console, then address the return-leg/stall behavior
separately. The one-metre text/config mismatch and retreat-after-turn
eligibility noted earlier are also still outside this correction.

### Physical open-floor validation — September 12

Episode `episode-e1475d7b4ca7fc422f4d5790` used the restarted production
BLAST console with the September 10 no-echo simplification. The operator
confirmed BLAST was on an open floor without a box. Both the submitted goal
and the configured directional target were 800 mm straight ahead; this
avoids, but does not fix, the earlier arbitrary-distance text/config mismatch.
No production code was changed during the run.

The episode completed in approximately 244 seconds. It executed one startup
surroundings scan and 17 forward pulses, with no reverse, extra scan, logged
controller-action failure or replanning loop. Scan closure was 359.02 degrees
with a 0.98-degree gyro-derived endpoint residue. Actual Qwen
`qwen/qwen3.8-27b` requests used low reasoning and an 8192-token output ceiling.
The two model decisions were ADVANCE with a waypoint at (800, 0), then
COMPLETE. Latencies were 6206 and 4285 ms; completion-token counts were 547
and 281. Ordinary forward execution continued with fresh raw-2000 no-echo
readings, without removing forward actions or asking the model each pulse.

Final map position was (780, -3) mm, with heading +1.903 degrees (left) and
20 mm estimated target distance, inside the existing 50 mm goal radius.
The operator confirmed the approximately 80 cm physical travel and map
agreement, then reported a very small leftward deviation, only a few degrees.
This is qualitative physical confirmation of the open-floor correction,
not a precision position measurement or general obstacle-course acceptance.
The small observed leftward deviation is consistent with the sign of the
final displayed heading; no steering-offset correction was added.

Two limitations remain visible in this run. The first model response preceded
the first forward command by approximately 83 seconds; sampled status showed
speech playing during that interval, and speech ultimately completed without
an error. The exact breakdown of that delay has not been diagnosed. Also,
the condensed startup-scan summary retained five readings (four measured and
one no-echo). The full angular record contained 16 readings, including ten
measured echoes. All were marked SWEEP_CONTINUATION_ONLY, so its planar map
projection contained no hit points. This run therefore validates no-echo forward
continuation on operator-confirmed open floor, not obstacle representation.
Do not infer a fully observed free room from the empty projection.

Evidence: `local-artifacts/navigation-dashboard.jsonl`, records beginning
at Unix ms 1789211704356; production status completed at 1789211947875;
final robot-specific spatial-map response and operator observations.
Next: a separately positioned physical single-box test, checking usable
obstacle evidence before interpreting its route. Keep the speech delay and
the previously failed box/chair simulation as explicit separate follow-ups.

### Preserve uncertain echoes without blocking navigation — September 12

Investigation of that recorded run identified two independent data/display
losses. The projection discarded every measured echo whose settling check
failed. The browser also still accepted only nine scan points and indexed
sides 1–4, although the backend full-sweep contract supports seventeen points
and sides 1–8. The browser limits now match the existing backend contract.

The same projection calculation now retains unsettled measured returns in
an optional `uncertain_points` field. The map renders them as dashed rays
with hollow echo markers, and Qwen receives their positions, timestamp and
explicit quality through `local_map_evidence.uncertain_echoes`. Settled
points remain separate. Uncertain echoes do not populate obstacle memory,
robot-center keep-out cells or verified-free-space calculations. No movement
permission, settling threshold, route policy or mandatory-rescan rule changed.
Shared-map transforms preserve this distinction using the same geometry path.

The historical scan does not contain its settling sample windows, so it
cannot establish whether sample count, distance variation or tilt caused
the failures. Added timeout diagnostics record those quantities for the
next physical test; no hardware failure is inferred from this evidence.

Validation: the full quality check passed 2,110 tests. After correcting the
browser's old point/side limits, the focused projection, renderer, dashboard
normalization and shared-coordinate tests also passed. A read-only browser
replay using the actual scan and production renderer visibly retained all
ten uncertain echoes without obstacle blobs.

Actual Qwen `qwen/qwen3.8-27b`, low reasoning, 8,192 output tokens, completed
one bounded `blast-open-floor` simulation using
`tests/fixtures/blast_open_floor_scan_20260912.json` as its startup scan and
four injected range-dropout reads. Result: goal reached, 39 mm remaining,
three model decisions, one startup scan, no additional scans, blocked moves,
route rejections or reversals. Later sensing used simulated geometry: this
is a recorded-startup/model integration check, not a full physical replay or
box-navigation acceptance. Evidence: `local-artifacts/navigation-simulation.jsonl`.

The physical robot was not moved for this correction, and the running
console was not restarted. These changes take effect after its next restart;
the separate replay does not modify the completed live episode.

### Remove precision settling gates — September 12

The follow-up correction removes the five-sample, 5 mm distance-spread and
1-degree tilt-spread acceptance gate. Post-motion observation now requires
two consecutive motor-idle observations, including a fresh read. Range and
tilt can vary without restarting that confirmation or making its result
unusable. Missing tilt or a no-echo range no longer prevents idle confirmation.
The obsolete distance/tilt comparison code and constants were removed.

The existing `observation_settled` receipt field is retained for compatibility:
it now confirms idle sampling, not constant range/tilt, exact geometry or free
space. Measured echoes remain provisional map evidence. Invalid distances,
nearby obstacles, sensor-arm pose and encoder geometry retain their separate
checks. A turn's sweep-clearance flag is no longer inferred from idle status
alone. Explicit stop/preemption and the existing bounded timeout remain.
No waypoint, planner, gyro, footprint or obstacle-margin policy was changed.

Validation targets the real `BlastObservationMonitor` and scan implementation,
with a fake hardware transport. A full surroundings sweep with alternating
500/540 mm returns and 2–3-degree tilt variation completes with measured,
idle-confirmed rays, correct closure and no recovery stop. Forward movement
also accepts idle samples differing by 40 mm and 2.5 degrees without waiting
for five near-identical readings. Missing idle confirmation stays explicit;
close-obstacle, sensor-pose and stop/preemption regressions still pass. Tests
that demanded five identical readings were updated to the new contract.
The higher-level route simulator bypasses this settling implementation, so
a successful route simulation alone would not validate this correction.

Full quality check: 2,111 tests passed in 83.826 seconds, including the real
monitor regressions above and existing navigation/map tests. JavaScript
syntax checks and `git diff --check` also passed. Test output:
`/private/tmp/robot-quality-idle-settling-20260912.log`.

No physical movement or console restart was performed. Physical confirmation
of the changed sampling behavior remains the next validation step; old scan
records keep their original quality flags and are not retroactively upgraded.

### Physical validation of idle sampling — September 12

Restarted the production console with low-reasoning Qwen and ran the same
800 mm open-floor goal, episode `episode-13f1c1166ff70d89314a3933`, starting
at Unix ms 1789214844484. The physical surroundings scan completed in 15.334
seconds, versus 68.642 seconds in the preceding run. All sixteen angular
readings were idle-confirmed: nine measured echoes and seven no-echo readings.
The live map visibly retained the rays/hits, and the operator confirmed it
looked substantially better and accurate. No additional scan or reverse ran.

This was NOT an end-to-end navigation pass. After twelve forward pulses the
episode faulted at (547, -39) mm, approximately 256 mm from the goal. Every
completed movement had an idle-confirmed receipt. The final event reports:
`blast_motion_slice_discontinuous`, expected encoders `(1094, 4405)`, observed
`(1094, 4403)`, delta `(0, -2)`. The pre-command continuity limit is one motor
degree. At the configured 0.5 mm/encoder-degree scale, this was approximately
1 mm of relaxation at one wheel, not a detected obstacle. The faster sampling
exposes this remaining precision assumption; no tolerance or execution-code
change was made during the physical test.

The operator subsequently questioned the displayed rightward angle because
it was not apparent physically. The scan-end IMU estimate introduced roughly
3.33 degrees of rightward heading; command-local yaw updates during driving
added approximately 1.56 degrees, giving the displayed -4.888-degree heading.
The encoder scan-closure residue was about 0.885 degrees, so these two sources
do not establish the same endpoint angle. After stopping, raw gyro heading
changed from -354.5918 to -354.6892 over 72.989 seconds, with unchanged wheel
encoders. This does not show large ongoing stationary drift, but neither does
it validate the displayed angle against reality. Earlier post-motion sampling
is a hypothesis to examine, not a proven gyro fault or a reason to reintroduce
the range/tilt precision gate.

Evidence: `local-artifacts/navigation-dashboard.jsonl`, episode events (final
sequence 30 at Unix ms 1789214944216), live-map screenshot and operator report.
BLAST remains stopped and online. Next bounded correction: handle small
inter-command wheel settling without discarding localization or terminating
the mission, while investigating scan-end heading separately. No automatic
restart with a new start-relative target was attempted.

### Consistent wheel-settling allowance — September 12

The recorded `(0, -2)` encoder change is now handled by the existing settling
segments, not a new recovery mode. BLAST uses one shared allowance equivalent
to 10 mm of travel per wheel (20 encoder degrees at the provisional scale)
for pre-command continuity, receipt gaps and scan-entry anchors. These small
movements remain explicit odometry segments; they are not discarded, snapped
away or counted as commanded motion. Larger unexplained gaps and malformed
or wrong-direction receipts retain their checks. EV3's default odometry
allowance is unchanged. No gyro, route-planning or obstacle policy changed.

The regression replays the actual expected `(1094, 4405)` and observed
`(1094, 4403)` encoders, checks that the 1 mm wheel relaxation is included in
position accounting, and executes a subsequent forward action successfully.
Scan-entry and receipt tests cover the same allowance and reject its outer
boundary. The full quality check passed: 2,112 tests in 84.522 seconds,
JavaScript syntax checks and `git diff --check`.
Log: `/private/tmp/robot-quality-wheel-settling-complete-20260912.log`.

The operator returned BLAST to the original open-floor starting position.
After reconnecting the powered-on hub, physical episode
`episode-a1858855aff35eecca98edfc` started with the same 800 mm goal and
low-reasoning Qwen.

Physical result: completed without runtime errors. One full surroundings scan
took 15.332 seconds, with sixteen idle-confirmed angular readings: ten measured
echoes and six no-echo readings. Seventeen forward pulses followed, with no
reverse or additional scan. The final map position was (781, 30) mm, 36 mm from
the target inside the existing 50 mm goal radius. Motion finished after about
67 seconds; the episode finished after about 93 seconds including final speech.

This run exercised the actual settling correction: three inter-command wheel
gaps exceeded the former one-degree limit, with deltas `(-1, -3)`, `(-2, -1)`
and `(-2, -1)`. The largest individual wheel relaxation was 1.5 mm. They did
not discard localization or interrupt the route. Other one-degree settling
gaps were present too. Evidence is the production controller receipts and
encoder observations in `local-artifacts/navigation-dashboard.jsonl`.

The operator confirmed that BLAST was approximately 80 cm ahead of the start,
that she had deviated slightly left, and that this was accurately represented
on the live map. This is a physical open-floor pass with operator-confirmed
map alignment, not proof that every gyro condition or obstacle route is solved.
The next bounded physical validation is the box route; no additional movement
was started and no new planner or recovery subsystem was introduced.

### Physical front-and-right box route — September 12

The operator placed boxes ahead and to BLAST's right, leaving the left side
open, and returned her to the start. The goal remained 800 mm straight ahead;
the model was not told which side to choose.

First attempt `episode-aa1d9d4a83dfe3d602f8f930` ended before any wheel motion
or planner request. A front echo at 217 mm was present, but the sensor-arm
encoder was 156 degrees against the 158 +/- 1-degree navigation reference.
The scan reported `scan_start_clearance_unverified` and the episode returned
`blast_startup_perception_incomplete`. This is an unresolved startup
preparation/strictness issue, not a model route-planning failure.

Used the existing stationary `probe_blast_expressions.py --gesture restore`
to restore the sensor position. It reported 157 degrees, 218 mm front range,
and zero changes in both wheel encoders. Reconnected the console for the same
physical route; no planning, calibration or tolerance code was changed.

Retry `episode-bd32c663cf868fd7e2e5b7a0` completed with no controller-action
failures or reverse movement. The initial full sweep took 15.867 seconds and
contained sixteen idle-confirmed angular readings, nine measured echoes.
The live map visibly showed rays, hits, obstacle regions and model waypoints.

The actual first Qwen request included the front cluster (four echoes around
x=296–327 mm, y=5–215 mm), a separate close right-side echo at (21, -241) mm,
the robot footprint clearance representation, goal (800, 0), coarse grid and
known-clear axis reaches (left 450 mm, right 0 mm). Low reasoning and an 8192
output-token cap were retained. The recorded response reasoning explicitly
rejected the right detour because of its nearby echo/keep-out intersection and
chose the left route: (0, 450), (500, 450), (500, 0), (800, 0). No operator
left-turn instruction was added to the model's goal or context.

BLAST followed the first two legs. At approximately (503, 489) mm Qwen revised
the return leg, which still intersected mapped obstacles, to continue to
(800, 489) and then return to (800, 0). One host route interruption reported
`FORWARD_CLEARANCE_UNAVAILABLE` despite a measured 564 mm range. The model
retained the new waypoints and selected a front-arc rescan; five of its nine
angular readings were measured and all were idle-confirmed. It then continued
to the goal instead of aborting. The exact action-availability cause of that
interruption has not yet been resolved; it must not be labelled a sensor fault
solely from the robot's spoken commentary.

Total: two scans (startup 360-degree sweep plus one front-arc refresh), forty
forward pulses, two left-turn and three right-turn actions, eight model
decisions, no reverse and no controller-action failure. The episode completed
in approximately 309 seconds including planning and speech. Final estimated
position was (841, 27) mm, 49 mm from target, heading approximately -18 degrees,
within the existing 50 mm / 20-degree completion tolerances—not an exact
position or heading claim.

The operator confirmed both that BLAST passed the box on her own left with a
matching map, and that she reached the intended goal behind the box. Her slight
leftward drift was visible on the map and handled during the route. This is a
successful physical two-box route after the explicit sensor-arm preparation,
not an unattended-startup pass. Follow-ups remain the startup arm preparation
and unnecessary clearance interruption; no further physical run or code
refactor was started in this validation step.

### Conversation after navigation — September 12

The operator's clear status question received the generic clarification twice.
A motorless replay established the cause before any model call: the status
projector copied full scan diagnostics and obstacle support points, producing
17,209 bytes against the conversation client's 16,384-byte facts limit. The
input service hid that exception behind a misleading clarification reply.

The shared BLAST/EV3 conversation projector now retains a concise latest scan,
obstacle meaning/location/uncertainty, scan boundaries, pose and mission status.
It omits raw support-point lists and encoder diagnostics. No-echo sentinels
become null distances, headings are expressed in degrees with their sign
convention, and stored observations are explicitly distinguished from live
sensor readings. EV3 IR remains qualitative, not centimetres. The actual
post-route context is approximately 8.9 KB. Navigation's map, planner and
execution are unchanged; no new classifier or routing rules were introduced.

Technical fallback replies now describe a technical failure, log its cause,
and expose `fallback: true` in the turn response. Genuine model-selected
clarifications retain the model's question. Status/conversation/failure paths
still cannot start navigation; only the existing physical-task branch can.

Real-Qwen, motorless checks covered status, recent mission outcome, sensor
questions, social conversation, stop, navigation, unsupported physical tasks
and ambiguity. All eight intent checks matched and returned no fallback;
ordinary replies took about 2–2.5 seconds. Additional checks caught and corrected
context ambiguity around no-echo values, IR units and heading sign. The final
BLAST sensor answer described the last scan as stale; EV3 described a nearby
IR reflection without inventing centimetres. EV3 validation here used supplied
status/IR facts, not a physical EV3 run.

The conversation classifier remains the existing single intent-and-reply call
per turn, using robot status, not a new multi-agent dialogue engine. Its existing
reasoning-off/192-token settings were not the cause of this incident and were
left unchanged; navigation still uses low reasoning with 8192 output tokens.
The live console was reloaded to activate the fix. This clears its in-memory
mission view; the physical validation logs above remain retained.

The exact original question was then submitted through the live GUI after
reload. It displayed a normal status reply (idle, awaiting instructions), and
the control service remained IDLE with no navigation episode. The accompanying
model-selected arm_wave reported a separate hub error: `motors must be idle
before setting a pose`. The monitor reconnected afterward. Text conversation
is verified; this does not establish successful gesture or complete audible
delivery. That accessory-execution issue was not folded into this context fix.

Final quality gate: 2,114 tests passed in 100.684 seconds, JavaScript syntax
checks and `git diff --check` passed. The 52 focused conversation/HTTP tests
include dense recorded BLAST scan evidence, EV3's nonmetric observations and
technical-failure reporting. Log:
`/private/tmp/robot-quality-conversation-release-20260912.log`.

### Interrupted dialogue audio — September 12

The user confirmed that the reply above was audibly cut off. Both `arm_wave`
and `claw_flourish` had received a valid hub rejection between accessory poses
(`motors must be idle before setting a pose`). The host misclassified that
rejection as a broken protocol session and restarted the hub, stopping audio.
The arm was left at -443 motor degrees, next to its -442-degree raised target.

Accessory target moves now finish with BRAKE instead of HOLD, retaining the
existing delayed coast release. This avoids continued tiny hold corrections
reopening the busy state after a pose has completed. Speed, target positions,
travel bounds and navigation execution are unchanged. Hub validation failures
are explicitly marked as command rejections; the host reports them without
tearing down the session. Unknown/internal errors and malformed protocol
responses retain the existing failure handling. No automatic gesture retries
or new navigation rules were added.

Regression checks cover a rejected second arm pose during audio, continued
face commands on the same session, and malformed error responses remaining
failures. All 2,116 tests passed in 97.930 seconds, with the full quality gate
passing (`/private/tmp/robot-quality-speech-gesture-20260912.log`).

The live GUI greeting-and-wave test then returned a normal Qwen reply and
finished with no expression failure or reconnect. Body position returned to
157 degrees; wheel encoders stayed exactly at 3811 and 6773 degrees. The user
confirmed complete audible delivery and the returning wave, but also heard
the model's literal `*waves arm*` stage direction. The dialogue prompt now
explicitly separates words read aloud from nonverbal expression fields.
Three real-Qwen, motorless checks then returned clean spoken text and separate
arm-wave, claw-flourish and face-only choices, without fallback. All 13 dialogue
model tests passed after this prompt-only follow-up.
