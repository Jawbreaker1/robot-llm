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

Stage 2 starts from an explicit, incomplete baseline rather than compatibility
tests that pretend the target is already met:

- BLAST already carries `goal`, compact map evidence, available actions, and an
  advisory Gemma waypoint on every decision. Its current startup acquisition
  publishes one broad front scan view, not a verified full-surroundings view.
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
