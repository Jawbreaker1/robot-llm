# Blast / Astra validation after decision-memory changes

## Current result after removing misleading clearance summary

Later follow-up on 2026-10-01: three fresh Astra runs through the production
Blast episode reached their goals without physics-blocked substeps. This
includes two independent model runs in the recorded box/chair scenario with
four range dropouts. No physical robot was moved. A supervised physical test
is now a reasonable next validation step; three simulated passes are not a
general reliability guarantee or a fresh EV3 navigation validation.

| Scenario | Model decisions | Goal distance | Physics-blocked steps | Scans |
| --- | ---: | ---: | ---: | ---: |
| `blast-box-front` | 7 | 31 mm | 0 | 2 |
| Recorded box/chair start + 4 range dropouts | 13 | 22 mm | 0 | 4 |
| Same recorded scenario, fresh model run | 11 | 22 mm | 0 | 3 |

The removed `known_clear_axis_reach_mm` summary counted consecutive 150 mm
grid cells crossed by narrow sensor rays and presented their count as a
distance of known clear travel. A ray crossing a cell does not establish
clearance for the robot's body or its turn sweep. The previous failing run
explicitly relied on its reported 450 mm reach for a 386 mm return leg.
That did not demonstrate room to turn at the leg's endpoint.

The correction deletes this derived summary and its unused grid underlay
generation. The existing grid, measured echoes, route checks and raw current
side scan remain; the prompt now states the grid's narrow-ray meaning and
correctly describes echo clusters as retained observations, not just the
latest scan. Saved maps with the older underlay remain readable. No new
motion gate, fixed detour, obstacle inflation or calibration change was added.
Unknown space remains available for bounded exploration.

In the first recorded-scene rerun, Astra scanned at the return corner near
(690,-394), identified an echo obstructing its proposed return leg and chose
a new corner at x=825 before returning to the goal. The second run also
revised its corner after inspecting the return direction. These were model
choices, not routes supplied by the harness. Behavior improved in this small
sample; stochastic route differences mean it does not isolate the correction
as the sole cause of the improvement.

All 2,080 quality-suite tests passed, including 112 targeted map, context,
recorded-sensor and recovery tests. All 31 planner requests and replies used
`gpt-6-astra`, with low reasoning requested. Their 28 subsequent inputs retained
the previous decision exactly; none contained the removed clearance summary.
Maximum serialized request sizes were 26,196, 29,109 and 28,066 characters.
All three estimated final poses matched the simulated world poses, and no
runtime error was returned.

Artifacts: `/private/tmp/robot-ray-evidence-astra.UamFEl/` contains `front/`,
`replay/` and `replay-repeat/`, each with `result.json`, `run.err` and the full
`local-artifacts/navigation-simulation.jsonl`. Test logs are
`/private/tmp/robot-ray-clearance-focused-20261001.log` and
`/private/tmp/robot-ray-clearance-quality-20261001.log`. The recorded scene
still uses real data only for the startup scan; subsequent sensing comes
from the approximate simulated world described below.

## Earlier result with scan feedback correction only

Follow-up on 2026-10-01: the stale scan-motion summary is corrected in the
shared feedback function and both robot adapters. Fresh Blast trials with
Astra reached both goals, but only the simple box trial was collision-free.
Those results alone did not justify the harder physical box test.

| Scenario | Model decisions | Goal distance | Physics-blocked steps | Scans | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| `blast-box-front` | 9 | 31 mm | 0 | 3 | Clean simulated pass |
| Recorded box/chair start + 4 range dropouts | 17 | 44 mm | 10 | 4 | Goal reached, not collision-free |

The correction reports the latest scan's measured pose change, completion
and failure reason instead of retaining an older ADVANCE result. A completed
scan returning to the same heading counts as scan progress without inventing
translation. EV3 reports the whole scan rather than its last small turn.
No route-selection rule, clearance margin or motor calibration was changed.
The route and final goal remain retained through partial and failed scans.

All 2,080 quality-suite tests passed, including 69 focused feedback, recovery
and EV3 tests. The production-loop regression reproduces the earlier blocked
Blast corner and checks that its partial scan replaces the previous motion
summary, reports about -70 degrees of actual rotation and retains the route.
These scripted tests verify the execution contract, not model ability.

All 26 fresh planner requests and responses resolved to `gpt-6-astra` with
low reasoning requested. All 24 subsequent context packets retained the
preceding action, assessment and plan exactly. Maximum serialized request
sizes were 28,128 and 29,037 characters. The partial scan in the harder trial
now appears correctly in the next packet as an incomplete scan with about
29.6 degrees of rotation. Both final estimated poses match the simulated
world poses, and neither run returned a runtime error.

The clean trial chose a longer forward detour than the earlier run, so one
successful trial does not establish that scan feedback alone caused the
improvement. The harder trial reproduced the same ten blocked substeps as
before. An offline replay of its 17 recorded decisions reproduced every
planner pose and localized all blocks to the final corner behind the box:
seven at (753,19), heading 68.055 degrees; one at the same position, heading
97.641 degrees; and two at (750,39), heading 97.641 degrees. This replay is
diagnostic evidence, not another independent model trial.

Before that corner, the model said the nearest mapped echo was outside its
turn sweep. Its packet contains a nearby measured side return at (594,-187),
but no measured outline of the box's back face. The hidden test-world box
ends at x=620, leaving only 133 mm from the robot's x=753 position to that
face, less than its 163 mm full-turn sweep radius. This explains why clear
distance ahead and sparse point returns were insufficient to establish
turning room. Those hidden dimensions were not given to the planner.

Follow-up artifacts are in `/private/tmp/robot-scan-feedback-astra.VUVCtN/`:
`front/result.json`, `replay/result.json`, each run's full diagnostic log,
and `replay_forensics.py`. Quality-suite output is in
`/private/tmp/robot-scan-feedback-quality-20261001.log`. No physical robot
was moved during this follow-up.

## Earlier baseline before scan feedback correction

Two fresh model-driven trials used the production Blast episode, planner,
context builder and movement executor, with simulated hardware only. Both
resolved the selected `codex/gpt-6-astra` model to `gpt-6-astra`, with low reasoning.
The goal text was only: "Reach the goal 800 mm straight ahead of your starting
position." No route, detour side or hidden box coordinates were supplied.

| Scenario | Model decisions | Goal distance | Physics-blocked steps | Scans | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| `blast-box-front` | 15 | 24 mm | 14 | 3 | Goal reached, not collision-free |
| Recorded box/chair start + 4 range dropouts | 17 | 44 mm | 10 | 4 | Goal reached, not collision-free |

Both ended `completed`, with world-goal confirmation, no runtime exception and
agreement between final estimated and simulated pose. That is **not** a clean
navigation pass: physics blocked movements that would intersect an obstacle.
These counts are blocked substeps, not 14 and 10 independent physical crashes.
The model retained the mission and recovered instead of losing its route or
abandoning the mission. No physical robot was driven. The requested physical
box test is deferred until the corner/recovery problem has been addressed and
the setup is confirmed.

These are Astra trials, not a paired rerun of the former Qwen baseline. They
expose a current problem but do not prove it was introduced by decision memory.

## Small simulator wiring correction

The Blast simulator's response-recording transport still called LM Studio
directly. It now calls the existing `robot_model_transport`, like production,
so selecting Astra actually reaches the subscription provider. This replaces
one import and one call; no navigation, prompt, geometry, motor calibration or
sensor policy was changed for these trials. A regression test checks both
Astra and Qwen selections through that shared transport using scripted replies.
Those scripted replies are wiring checks, not model-navigation results.

## First problematic corner, with exact context evidence

Astra initially chose the left route `(0,300) -> (600,300) -> (600,0) -> (800,0)`.
The robot reached measured `(592,315)`, heading approximately zero. In the
initial/retained echoes the visible box face occupied roughly x=320..328,
y=-91..80; the hidden simulated box actually spans x=320..520, y=-180..180.
The planner is explicitly told that echoes are not complete object outlines.
Nevertheless, the chosen return corner was too close for the body to rotate.

Decision 3 requested a front scan. It stopped partially rotated at about -70
degrees. The next packet accurately retained that pose, the route and a scan
history entry with `state=partial`, `result=coverage_incomplete`, and
`restoration_verified=false`. But `observation.last_motion` still described
the earlier successful 45 mm ADVANCE, with no current scan-motion result.
`latest_motion_feedback()` selects only entries with a `last_motion` field;
the Blast scan history branch updates pose/scan metadata without creating that
field. Thus the most recent action's movement is not represented consistently
in the summary, even though the fuller history contains its changed heading.
This is a concrete information-contract gap, not proof that it caused every
contact: Astra noticed the partial scan and still chose to finish the turn.

The next alignment attempt stalled. Astra then used bounded ADVANCE actions,
updated its route from the resulting position, and eventually reached the goal.

An offline execution replay of the same 15 recorded decisions reproduced the
same planner-pose sequence, final pose and 14 blocked substeps. Those blocks
occurred at three poses:

| Pose (x, y, heading) | Blocked substeps |
| --- | ---: |
| 592 mm, 315 mm, -69.768 degrees | 5 |
| 608 mm, 273 mm, -73.536 degrees | 8 |
| 634 mm, 187 mm, -73.536 degrees | 1 |

This replay localizes the corner problem; it is not another independent model
success. The simulator's collision handling stops interpenetration, so goal
completion after those blocks must not be presented as safe real-world travel.

## Context capture and fidelity limits

All 32 actual planner requests and replies were saved, including complete
instructions, context and response schema. The previous decision matched the
preceding action, assessment and plan in all 30 subsequent inputs. Maximum
serialized application-request sizes were 29,056 and 28,492 characters, not
token counts. The provider rejected the optional 4,096 output cap and the
existing adapter retried without it; low reasoning remained enabled.

The box/chair trial used the recorded 2026-09-02 startup scan and four injected
missing range readings. Later sensing comes from an approximate box/chair
world, not measured room truth. Neither test validates live Bluetooth/USB,
speech, real floor traction or current physical calibration.

Artifacts: `/private/tmp/robot-blast-astra.QSgBy1/` contains `front.json`,
`front.err`, `local-artifacts/navigation-simulation.jsonl`, and the offline
`replay_front_forensics.py`. Its `replay/` subdirectory contains `replay.json`,
`replay.err` and a separate full diagnostic log. Both error files were empty.

Twenty simulator/recorded-regression tests passed after the transport change;
log: `/private/tmp/robot-blast-model-routing-tests-20260929.log`.
The earlier 420 Blast tests and 86 shared tests remain regression evidence,
not substitutes for these fresh model trials.

## Next validation

Confirm Blast's physical setup, then run a supervised box test with a
goal-only instruction and inspect the same captured context and measured
motion feedback. Compare the real body clearance, final position and map
with observation rather than treating simulator completion as physical
proof. Keep route choice and revisions with the model; do not add a fixed
detour or alter calibration to hide any discrepancy.
