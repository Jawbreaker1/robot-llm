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

## Known failing condition

BLAST's current calibrated surroundings sequence uses sixteen 45-degree wheel
pulses. At 490 body-millidegrees per opposed encoder degree this covers 352.8
body degrees, not 360. The simulator therefore reports
`coverage_complete: false`. A production navigation gate must remain failed
until the physical scan provides verified full coverage and an honest final
heading.
