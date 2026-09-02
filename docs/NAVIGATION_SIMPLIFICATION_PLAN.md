# Navigation simplification plan

Status: stage 1 cleanup is complete; stage 2 behavioral alignment is in
progress.

Baseline: `fb359f7` on `main`, published on 2026-08-20. This plan starts from
that clean checkpoint. It complements [Navigation ownership](NAVIGATION_OWNERSHIP.md),
which remains the short authority contract.

## Why this plan exists

BLAST and EV3 are small LEGO robots with different sensors and motor hardware,
but they should not have different ideas of what agentic navigation means.
Today BLAST is close to a one-decision-at-a-time agent loop, while EV3 still
contains host-owned route execution, maneuver commitments, and plan tails.
BLAST also retains unreachable code from an older host-owned detour design.

The objective is not a new robotics framework. It is to remove policy that the
host should not own, make both robots follow the same small agent loop, and keep
hardware-specific code below that loop.

## Target architecture

Both robots should follow this semantic cycle:

```text
goal + compact world view + active waypoint + available actions + recent results
                                      |
                                      v
                         Gemma chooses one next step
                                      |
                                      v
                    robot adapter executes a bounded primitive
                                      |
                                      v
                         fresh observation returns to Gemma
```

One semantic action may contain several short motor pulses. Agentic navigation
does not require an LLM call for every pulse. The executor may continue an
`ADVANCE` primitive while its measured continuation conditions remain valid,
but it may not choose a new direction, waypoint, or follow-up action.

### Shared responsibilities

- The first navigation decision starts from a full surroundings view. Scan motor
  order is a robot-specific implementation detail, never a route preference.
- Gemma owns the semantic action, turn direction, tentative waypoint, and path
  strategy.
- The shared agent layer carries the goal, compact world view, available robot
  capabilities, recent results, and the optional active waypoint.
- The host may remove or reject an action that cannot presently be executed. It
  does not replace it with another semantic choice.
- A failed or uncertain ordinary attempt returns to observation and replanning.
- Both planners require the configured non-QAT Gemma model and reject a served
  model with another identity.

### Robot-specific responsibilities

- sensor acquisition and translation into the compact world view;
- supported semantic actions and physical capability descriptions;
- bounded motor primitives and pulse-level continuation checks;
- coarse, measured calibration for the actual LEGO construction;
- controller transport, stop, and motor ownership.

### Failure classes

Keep a small hard-stop class:

- user stop, emergency stop, or episode deadline;
- controller loss or motor state that cannot be shown to be stopped;
- a concrete measured collision condition;
- a command/encoder result so inconsistent that the physical outcome is
  genuinely unknown.

Treat ordinary uncertainty as recoverable agent input:

- no-valid-distance, a stale or missing individual observation;
- an unsettled ray or incomplete surroundings view;
- an invalid model response;
- a blocked tentative waypoint or no useful action in one observation;
- ordinary LEGO-scale settling and odometry noise within measured calibration.

Recoverable uncertainty must be bounded by the existing episode deadline and
decision budget. It must not create a second recovery state machine.

## Fixed implementation order

There are exactly three implementation stages. A later stage does not start
until the previous stage meets its acceptance conditions.

### 1. Remove retired BLAST host-navigation code

This is a behavior-preserving deletion stage.

- Prove production reachability before deleting anything.
- Remove the unreachable side-search, local-detour, host-route, recovery-rebase,
  and legacy navigation-state plumbing from the BLAST adapter.
- Delete BLAST-only modules and tests whose only caller is that retired path.
- Preserve the live neutral scan-permit, scan-sweep, observation, motion, and
  controller-stop primitives.
- Do not change action availability, prompts, calibration, scanning, or live
  navigation behavior in this stage.

Acceptance:

- no production import or branch can enter the retired BLAST route executive;
- the current BLAST scenario outcomes and serialized contracts are unchanged;
- focused BLAST control tests and the full hardware-free suite pass;
- the test count decreases because tests for deleted behavior are deleted, not
  rewritten to preserve it.

### 2. Make both robots meet one behavioral contract

This is the only stage that intentionally changes navigation ownership.
BLAST is closer to the target ownership model, but it is not a reference
implementation and its remaining behavior must not be copied into EV3.

- Define the same four black-box scenarios for both robots: clear path,
  opening-left, opening-right, and transient bad sensor evidence.
- First make BLAST satisfy those outcomes and complete one physical checkpoint.
- Then make EV3 satisfy the same outcomes and complete its physical checkpoint.
- On EV3, remove host policy that selects and follows local-detour waypoints,
  maneuver commitments, or plan tails without a new model decision.
- On both robots, return to Gemma after each completed or interrupted semantic
  primitive. Do not make Gemma decide individual motor pulses.
- Retain each robot's sensor adapter, bounded motion executor, motor supervisor,
  collision checks, and truthful pose updates.

The four scenarios use the same observable contract for both robots:

| Scenario | Required outcome |
| --- | --- |
| Clear path | The first planner context contains the user goal and surroundings evidence. Gemma may choose one bounded `ADVANCE` primitive whose executor emits several short forward pulses, then control returns to Gemma before any different semantic action. |
| Opening left | The world view shows the left opening and keeps the original goal. Gemma chooses the turn or advisory waypoint; the host neither invents a side nor substitutes a route step. |
| Opening right | The mirrored left-opening contract applies without a fixed or preferred first side. |
| Transient bad sensor | No motion starts from invalid evidence. A bounded re-observation returns recovered evidence to Gemma in the same episode instead of terminating the mission or starting a second recovery workflow. |

#### Shared multi-robot simulation gate

The former BLAST-only seeded suite was removed. It supplied complete waypoint
routes before each run and therefore measured route execution while appearing
to validate navigation. Its reported 1000/1000 result is not a navigation
baseline.

The replacement uses one synchronized physical world for BLAST, EV3, and
future robot profiles. Scenarios contain only room bounds, rectangular
obstacles, measured robot bodies, starting poses, and final goals. Robot bodies
are visible dynamic obstacles to each other. Each production agent must create
its own route from sensor and map evidence; the simulator has no waypoint,
side-selection, route, or backtracking policy.

The initial suite progresses from a clear shared room through one common box,
several staggered obstacles, and a dead end that can require reversal and a
different opening. Every scenario starts BLAST and EV3 in the same concurrent
simulation window. A run counts as navigation validation only when the real
model planner participates.

The simulator also checks truthful sensor claims. In particular, BLAST's
current 16-pulse surroundings sweep measures 352.8 degrees in the calibrated
model and must not be reported as complete 360-degree coverage. This is a
known failing acceptance condition until the physical scan ends with verified
full coverage and heading restoration.

The context and waypoint audit removed several real sources of contradictory
behavior: BLAST no longer carries a stale motor-action `active_plan`; planner
history is a compact semantic event log rather than repeated pulse receipts;
raw scan rays live only in the dedicated perception/map fields; and Gemma gets
derived waypoint distance, bearing, and signed heading error. A model-selected
`ADVANCE` is never silently replaced by a host-selected turn. An unreached
waypoint must be retained or explicitly replaced, while `waypoint_required` no
longer conflates an intermediate subgoal with final-heading restoration. When
the direct action is unavailable outside the goal corridor, it instead requires
Gemma to create a detour hypothesis. Final-goal duplicates are removed from the
intermediate list only after the robot has entered the final corridor.

Turn execution is feedback-bounded without selecting a route: Gemma still
chooses left or right and authors an ordered route of up to four waypoints. One
explicit `FOLLOW_WAYPOINT` decision delegates only the mechanical execution of
the current waypoint. The host aligns and advances to that waypoint, then
returns the whole remaining model-authored route to Gemma for confirmation or
revision before another leg can run. A blocked current leg stops physical
execution but remains in context until Gemma replaces it. The host never
invents a waypoint, route, or side. The stable 150 mm `visited_cells` trail
retains the latest 128 traversed cells, including revisits, so Gemma can
recognize and backtrack from explored branches without a host-owned maze
planner. Black-box simulation tests lock these boundaries across multi-waypoint
routes.

Model-authored waypoint legs are now Manhattan-style on the existing episode
axes: each leg changes x or y, not both, with one coarse 150 mm cell of
odometry drift accepted. A diagonal is returned to Gemma before motion as
`NON_ORTHOGONAL_ROUTE_LEG`; Gemma still chooses the side, order, distances,
scans, and replanning. The host does not synthesize the right-angle corner.

Terminal completion now depends only on valid localization, the roomy coarse
goal region, and its 20-degree heading tolerance. Requiring a fresh scan merely
to acknowledge an already reached goal caused pointless scan or
forward/reverse loops and supplied no additional pose evidence, so that host
gate was removed.

The remaining action-boundary ambiguity was removed without adding a route
heuristic. A blocked retained waypoint no longer exposes `FOLLOW_WAYPOINT` as
if it were executable; Gemma must scan, replace the route, reverse, or choose an
available turn. Inside the existing coarse 150 mm final-goal radius, a new
positional route is not offered merely to restore heading. The compact map now
supplies a signed, rounded `directional_goal.heading_error_deg`, matching the
already-derived waypoint geometry. Gemma still chooses the turn direction; the
executor may continue that chosen turn only until the existing generous
20-degree final-heading tolerance.

The earlier deterministic 1000-world numbers are retained only as historical
execution evidence and are not an acceptance baseline. New acceptance results
must state which production model planned each robot, whether both robots ran
concurrently, and whether every final route was model-authored.

Revalidation on 2026-08-30 supersedes that acceptance for the current dirty
branch. The deterministic mechanics gate passed 987/1000 worlds; all 13
failures were mirrored `slalom_right` cases which exhausted the decision
budget without collision. Gemma-in-the-loop validation was then stopped early
instead of claiming a 21-run pass. Clear path and `opening_right` passed, and
targeted reruns showed that both `opening_left` and `slalom_right` can pass.
However, a repeated identical `opening_left` run later exhausted its budget or
reported hidden simulator collisions. The local request already uses
temperature zero, so this trajectory variance cannot be treated as deliberate
sampling.

The audit found and corrected three deterministic host problems: a rejected
route was still presented as active, `FOLLOW_WAYPOINT` was immediately offered
again without changed evidence, and one reverse pulse incorrectly disabled a
second retreat even when several verified forward pulses remained to undo.
Route rejection now includes the responsible measured `blocking_echo_point`, and
the model prompt describes a side-clearance-first detour without selecting a
side. These changes materially improved targeted reruns, but do not yet make
the full model gate reliable. Physical BLAST validation is therefore paused.
The next bounded step is to expose explicit collision/impact feedback to the
same agent context (the simulator currently keeps collision as a hidden oracle,
and physical BLAST has no corresponding model-visible gyro event), then rerun
the deterministic and Gemma gates before returning to hardware.

The later controlled simplification slice on 2026-08-30 found that Gemma's
problem was partly presentation rather than missing raw scan volume. The
controller prompt had grown from about 593 to 1458 words while asking Gemma to
translate an ASCII grid into millimetre route geometry. It is now about 900
words after removing repeated scan, waypoint and recovery instructions. The
same coarse map additionally exposes connected `keep_out_regions` with
symmetric `clear_before_x_mm`, `clear_past_x_mm`, `clear_left_y_mm` and
`clear_right_y_mm` facts. These are geometric observations only: the host still
does not choose a side or construct a route. Directly repeated adjacent model
waypoints are collapsed as redundant input.

The same slice made final-goal state explicit with `corridor_entered` and
`heading_aligned`. Once the directional corridor is entered, the host no longer
offers a positional `FOLLOW_WAYPOINT` which it would immediately discard. This
removed a measured 39-decision no-motion loop. The focused adapter, planner,
map and simulator regression set passes 156/156. The first bounded seven-case
Gemma suite after prompt reduction passed 3/7 with zero collisions in every
case; the failures were replanning/budget failures. Subsequent targeted reruns
passed clear path, both mirrored single openings and both mirrored double
obstacles, but one double-obstacle case failed and then passed unchanged. That
variation means the model gate is still not green, and the slalom pair has not
yet been rerun after the final contract corrections. Physical validation stays
paused. The next gate is the two slalom cases followed by repeated mirrored
routes; no further prompt rule should be added unless their compact traces show
one shared missing fact or deterministic host contradiction.

The bounded slalom gate was run next on 2026-08-30. `slalom_right` repeatedly
completed, while `slalom_left` exposed two concrete contradictions. First, the
coarse route validator rejected an entire model-authored route when only a
later leg crossed a known keep-out cell. It now retains and executes the safe
prefix of Gemma's route and returns control before the blocked suffix; a blocked
first leg is still rejected before motion. Second, an older prompt sentence
asked for several extra cells beyond a clearance bound even though
`clear_left_y_mm` and `clear_right_y_mm` already include coarse body clearance.
That conflict made Gemma add 300 mm to a valid `y=750` boundary and steer toward
the simulated arena edge. The extra-margin sentence was removed; no host route
or side choice was added.

After those two reductions, 157 focused adapter, planner, map and simulation
tests pass. The final two-case Gemma gate still passed only `slalom_right`, in
four decisions with two scans and no collision. `slalom_left` exhausted its
16-decision budget with no collision, four scans, and a sequence of changing
or repeated clearance waypoints instead of a coherent continuation. The gate
is therefore not green. Repeated simpler-route stability runs and physical
BLAST validation were deliberately not started. The next step must inspect the
remaining model/context contract as one bounded slice; it must not add another
navigation heuristic merely to make this one course pass.

A following bounded context audit showed that the failed left case did not
lose its final goal or accumulated evidence. The same goal, current pose,
active plan, route rejection and keep-out regions remained present. The map did
however require Gemma to infer which item in an unsorted region list blocked
the straight final-goal segment. From one identical first-decision context,
Gemma produced the same known-blocked first leg five times. The already-used
coarse intersection calculation is now exposed as `direct_goal_blockage`, with
the matching `blocking_region`; it supplies no turn side or waypoint. From the
same context after that presentation change, Gemma produced the same clear
three-leg detour five times.

The 157 focused tests still pass. A new full slalom pair then passed
`slalom_right` in four decisions with two scans and no collision.
`slalom_left` passed the earlier blocked-route point but later spent 15
`FOLLOW_WAYPOINT` decisions returning an already-reached `(693, 750)` side
clearance point. It ended with no collision and only the startup plus one
additional scan, but made no further progress. The decoder currently checks
that a required waypoint exists, not that it differs usefully from the current
pose; simply rejecting that model response would currently terminate rather
than replan. No such brittle validator was added. Treat no-op waypoint recovery
as a separate bounded contract slice before any physical BLAST run.

The no-op slice now skips already-reached leading points in a newly returned
model route. If Gemma supplied a later point, that next model-authored leg is
retained; if no motion remains, the history receives
`WAYPOINT_ALREADY_REACHED` and control returns for replanning. The host creates
no replacement. The focused regression set passes 159/159, and both mirrored
Gemma slalom cases then completed with two scans and no collision:
`slalom_left` in seven model decisions and `slalom_right` in four.

One deliberately larger gate now complements the seven short templates. The
`large_room` scenario is a 3,000 by 3,000 mm world, asks for 1,800 mm forward
progress, contains eight obstacle circles, and has an empty scripted `route`.
It can be run with:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src \
  python3 -m robot_agent.blast_navigation_gemma_simulation_cli \
  --large-room --max-decisions 16 --summary
```

The exact non-QAT `google/gemma-4-26b-a4b` completed the permanent gate in
seven decisions and three scans with no collision. Ground truth ended at
approximately `(2005, -20)` mm in episode coordinates with the desired heading
restored. Gemma authored and revised the waypoint route; the host supplied no
scripted path. This is one reproducible room-scale success, not yet evidence
that every large-room topology or long-lived map is reliable.

The first maze-readiness slice then corrected route blockage to report the
nearest known intersection on the first blocked leg rather than whichever
echo happened to be stored first. A later controlled simplification removed
the unused host route generator and separated the visual 150 mm grid from the
physical veto: route legs are now checked continuously against measured echo
points with one robot-centre clearance. This removes square-grid false
positives without choosing a detour for Gemma. This
correctness change did not keep the Gemma gate green: `slalom_right` passed,
while two identical `slalom_left` runs exhausted 16 decisions without a
collision. The large-room gate was not rerun.

The compact trace shows why the old green result must not be restored by
returning to scan order. At approximately `(930, 840)` the robot was already
left of a connected keep-out region whose reported bounds were roughly
`x=0..1050`, `y=-150..600`, with `clear_past_x_mm=1200`. Returning directly to
the goal axis crossed that known region, but Gemma repeatedly proposed re-entry
near `x=950` instead of first passing its far x edge. The nearest blocker is
truthful; the remaining problem is that a large connected region and the
robot's relation to it are not presented clearly enough for reliable
backtracking. Physical and maze validation remain paused. The next bounded
slice is the dynamic room-scale map with visited trail and explicit
robot-relative region facts, not another route heuristic.

That map slice is now implemented as a rolling 11x11 window over stable episode
coordinates. The window publishes its global bounds, connected keep-out
regions are no longer clipped to the start area, the planner receives a bounded
coarse visited trail, and `direct_goal_blockage` states the robot's qualitative
x/y relation to the blocking region's clearance edges. The dashboard converts
rolling grid cells back to their episode coordinates. The focused adapter,
planner, map and simulation set passes 162/162.

The model gate remains deliberately red. One isolated `slalom_left` run passed
in ten decisions with three scans and no collision after an obsolete prompt
recipe was removed. The following mirrored gate failed both cases at sixteen
decisions. The decisive failure is now below route selection: Gemma had already
chosen a waypoint near `(450, -300)`, the robot was within roughly 5 degrees of
its bearing, and forward motion was unavailable. The waypoint follower treats
an error below its coarse alignment trigger as `ADVANCE`; when `ADVANCE` is
unavailable it returns no follow action, exposes raw left/right turns, and the
model can alternate one-pulse turns without translating. The next bounded slice
is therefore a mechanical waypoint-follow fallback: while a model-owned
waypoint is active and forward is unavailable, offer one turn pulse toward that
same waypoint and then return fresh feedback. It must not select or modify any
waypoint, side, or route. The slalom pair and large-room gate must pass before
maze scenarios or physical BLAST validation resume.

A controlled Qwen3.8 27B run with Gemma unloaded corrected the earlier
memory-contended comparison. With low reasoning and a 1536-token response
budget, Qwen completed clear path in two decisions. At the first obstacle it
spent the entire response on spatial reasoning and produced no JSON action
within the bounded response. The model is therefore a useful planning-quality
reference but not currently a drop-in real-time controller. Model comparison
remains a bounded quality/latency gate, not a reason to add host-owned
navigation.

Stage 2 starts from an explicit, incomplete baseline rather than compatibility
tests that pretend the target is already met:

- BLAST already carries `goal`, compact map evidence, available actions, and an
  advisory Gemma waypoint on every decision. Its single startup acquisition is
  one encoder-verified 350–390 degree sweep, published as one surroundings
  view; the old `scan_front_arc` command name is retained only for wire
  compatibility.
- EV3 already carries the same user goal as `mission.user_goal` and can execute
  several short pulses for one model-authored forward primitive. It has no
  mandatory startup surroundings acquisition and its local-detour route
  executive still selects follow-up waypoint motion without a new model
  decision.
- The clear-path tests lock goal continuity and bounded same-direction pulse
  batching first. Opening-left, opening-right, and transient-evidence tests are
  added only with the corresponding behavior slice; they are not mocked green
  in advance.

Acceptance:

- EV3 and BLAST expose equivalent goal/world-view/action/waypoint concepts;
- no host component selects a substitute direction or route step for either
  robot;
- clear path, opening-left, opening-right, and transient-bad-sensor scenarios
  finish or make bounded progress without hidden host navigation;
- focused EV3 tests and the full hardware-free suite pass;
- controlled physical BLAST and EV3 validations are completed before stage 3.

### 3. Extract the genuinely shared agent layer

Only extract after the two working loops have the same shape.

- Share the decision contract, compact prompt structure, and episode-cycle
  coordination that are demonstrably identical.
- Keep observation adapters, capability descriptions, calibration, and motor
  executors robot-specific.
- Use one read-only world-view concept for Gemma and the dashboard; do not add
  pathfinding authority to the map.
- Update the architecture documentation to describe the resulting code, not an
  aspirational framework.

Acceptance:

- both production robots use the same semantic decision contract and cycle;
- differences above the adapter boundary are explicit capabilities, not two
  navigation policies;
- all four scenario outcomes run for both robots in hardware-free tests;
- controlled physical BLAST and EV3 validations pass;
- the completion criteria below are met. Then the refactor stops.

## Test pruning

Test quality is the goal; a high test count is not.

Keep:

- a few black-box scenario tests that prove useful robot behavior;
- one positive and one negative contract test at each important boundary;
- controller stop, unknown motor state, and concrete collision tests;
- serialization tests for model and dashboard contracts;
- regressions for faults actually observed on hardware.

Remove:

- tests whose production path is deleted or unreachable;
- near-duplicate permutations of the same mocked sensor condition;
- tests that freeze incidental call order or private helper structure;
- exact millimetre/degree boundary combinations without measured physical
  justification;
- tests that merely bless a terminal outcome instead of checking useful
  recovery or progress.

Do not replace deleted microscopic tests one-for-one. Prefer one scenario test
that crosses the whole agent/adapter/executor boundary.

## Change discipline

For every stage:

1. Start from a clean, published commit and name the one behavior dimension in
   scope.
2. Record the expected outcome before editing.
3. Keep the coding slice within 45 minutes. If the acceptance result gets worse
   or the slice expands into another subsystem, stop and inspect instead of
   stacking another patch.
4. Run focused scenario tests, then the full hardware-free gate.
5. For behavior changes, perform one controlled physical validation before the
   next stage.
6. Commit and push the completed stage separately. Do not mix calibration,
   speech, dashboard polish, or unrelated cleanup into it.

The assistant should explicitly challenge a proposed change when it introduces
host-selected navigation, a new state machine, unmeasured precision, duplicate
robot policies, or a generic abstraction before both concrete loops match.

## Non-goals

- no SLAM, A*, occupancy-grid planner, camera requirement, or fleet framework;
- no LLM call for every motor pulse;
- no random left/right default or hard-coded obstacle side;
- no redesign of speech while navigation is being stabilized;
- no threshold-by-threshold tuning as a substitute for coarse physical
  calibration;
- no shared abstraction whose only purpose is to make unlike implementations
  look alike.

## Completion criteria

The refactor is complete when all of the following are true:

1. BLAST has no reachable or dead host-owned route executive.
2. EV3 and BLAST use the same one-step semantic agent cycle.
3. Gemma owns direction, waypoint, and route strategy for both robots.
4. Each robot adapter contains only its capabilities, observations, calibration,
   and bounded physical execution.
5. Ordinary uncertainty returns to bounded replanning; only the small hard-stop
   class terminates immediately.
6. The retained tests emphasize end-to-end behavior and important boundaries.
7. The architecture and ownership documents match production code.

No fourth refactoring stage is implied. New work after these criteria is a
separate product decision supported by new evidence.

## Product path after the refactor

The larger two-robot goal is intentionally separate from the navigation
refactor. It proceeds through three product checkpoints:

1. **Shared world view.** Keep each robot's raw observations and identity, then
   publish them in one calibrated coordinate frame. Reuse the existing compact
   map evidence; do not introduce SLAM or a fleet planner.
2. **Shared knowledge and dialogue.** Give both robot agents the same bounded
   goal, event, and map summary. Add explicitly addressed robot-to-robot
   messages to the existing conversation services so both robots and the user
   can see who said what. Do not create an unbounded agent-chat loop.
3. **Coordinated simultaneous autonomy.** Keep both robots online, reasoning,
   observing, and talking concurrently. Permit concurrent physical episodes
   only after they share a coordinate frame and a small conflict check can
   reject overlapping motion. This replaces the current global episode gate;
   it is not a general fleet scheduler.

Each checkpoint is separately validated and published. Shared-map or dialogue
work does not start while the two navigation loops still disagree about who
chooses the next semantic action.

## Controlled validation result — 2026-08-31

The latest bounded slice removed two sources of host/model contradiction
without introducing a host-owned route:

- `FOLLOW_WAYPOINT` no longer lets the host continue indefinitely after a
  steering-only response to blocked forward motion. Control returns with the
  model-authored waypoint plan intact.
- A waypoint that is already approximately ahead is not treated as steerable
  when forward motion is unavailable. One coarse turn pulse would overshoot the
  bearing and had produced left/right oscillation.
- Connected echo cells are no longer summarized to Gemma as one filled bounding
  rectangle. In multi-obstacle scans that rectangle filled real gaps with false
  occupied space. The retired region builder and its isolated tests were
  deleted.
- The rolling grid now includes `robot_center_keep_out_cells`, the exact bounded
  x/y list represented by its `#/?` cells. Gemma still selects every side,
  waypoint and route. The prompt states only the essential geometry contract:
  neither a waypoint nor its straight leg may cross those cells.
- A segment starting near an echo may move away from it. The route validator no
  longer mistakes that escape for a new intersection at the segment origin.

The exact non-QAT `google/gemma-4-26b-a4b` then passed both mirrored slalom
scenarios in one 12-decision gate, in seven and nine model decisions
respectively, with no collision. A focused adapter, map, prompt and simulator
gate passes after the changes.

The larger unscripted eight-obstacle room is still red. After the mechanical
oscillation was removed, Gemma continued to produce rejected diagonal legs,
backtracked to previously visited cells and scanned too often. One experiment
that exposed eight unranked straight-leg endpoints did not improve the run and
was removed immediately; no unused candidate-route layer remains.

This is now an architecture decision rather than another prompt-tuning task.
The evidence supports Gemma as the semantic owner of mission, route hypothesis,
replanning, speech and social actions, but does not yet show that free-form
metric waypoint generation is reliable enough for room-scale or maze routing.
Before implementing maze cases or returning to physical BLAST, decide whether
to retain pure model-authored coordinates or introduce a small shared geometry
tool whose suggestions Gemma explicitly selects. Do not hide that choice inside
another host state machine or restore a default obstacle side. Physical
validation remains paused.
