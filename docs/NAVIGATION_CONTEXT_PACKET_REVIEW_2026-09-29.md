# Per-decision context packet review — 2026-09-29

## Scope and capture

Reviewed all **34 recorded Astra decisions** from the preceding decision-memory
validation: 10 front-box and 24 many-boxes. This is an examination of existing
runs, not another successful navigation trial or a physical test.

The shared planner already records `planner_request` **before** calling its
transport: model settings, complete system/user messages and response schema.
The corresponding raw response is recorded before parsing, with the same
request ID. A timeout therefore retains its input even without an answer.
The dashboard and Blast simulator already enabled this logger; the standalone
EV3 validation script did not. It now enables the same existing logger, by
default at `local-artifacts/navigation-ev3-simulation.jsonl`, overridable with
`--diagnostics PATH`. No second logger or navigation rule was introduced.

These are application planner requests. The Codex transport translates their
messages/schema to provider format; these records are not HTTP packet captures
or visibility into provider-internal state. Provider diagnostics separately
report model resolution, usage and whether the output limit was forwarded.
No credentials or authorization headers are part of the planner request log.
The historical simulations reviewed below contain exact context snapshots, but
did not capture the complete request at the time; newly enabled logging closes
that gap for future EV3 simulations, not retrospectively.

## Checks across every recorded decision

- The previous intent matches the prior action, assessment and plan in all
  **32** noninitial inputs. Each mission starts with no previous decision.
- The latest history outcome matches the preceding execution result in all
  32 transitions. Current x/y matches the last measured execution pose.
- Goal coordinates and body geometry remain unchanged within each run.
- EV3 receives the measured 200 x 230 mm asymmetric footprint, 110 mm front,
  90 mm rear, 100 mm left, 130 mm right, 10 mm margin and 181 mm turning radius.
- The eight-box goal is 2,000 mm ahead, not the 800 mm front-box goal. Known
  simulator rectangles, world pose and IR calibration curve are not included
  as such in these recorded contexts.
- All supplied obstacle-bound coordinates remain null. The model retains
  named hypotheses and written evidence, not established metric box edges.

Source recordings:
`/private/tmp/robot-decision-memory-astra-front-20260929.jsonl` and
`/private/tmp/robot-decision-memory-astra-many-20260929.jsonl`.
The structural checks are reproducible with
`/private/tmp/robot-context-packet-check.jq`.

## Front box — each input

`History` and `rays` count what the model actually received, before its action.
The outcome column is the latest measured result in that input, not a planned
or inferred movement. In-place alignment and a waypoint arrival are distinct.

| Decision | Selected action | Prior outcome supplied | History | Rays |
| --- | --- | --- | ---: | ---: |
| 1 | FOLLOW_WAYPOINT | initial | 0 | 13 |
| 2 | FOLLOW_WAYPOINT | heading_aligned | 1 | 14 |
| 3 | TURN_RIGHT_90 | waypoint_reached | 2 | 25 |
| 4 | SCAN_FRONT_ARC | semantic_action_completed | 3 | 26 |
| 5 | FOLLOW_WAYPOINT | survey_completed | 4 | 36 |
| 6 | TURN_RIGHT_90 | waypoint_reached | 5 | 51 |
| 7 | SCAN_FRONT_ARC | semantic_action_completed | 6 | 52 |
| 8 | FOLLOW_WAYPOINT | survey_completed | 7 | 62 |
| 9 | TURN_LEFT_90 | waypoint_reached | 8 | 69 |
| 10 | COMPLETE | semantic_action_completed | 9 | 70 |

The world goal was reached without contact. No history or ray eviction occurs
in this run.

## Many boxes — each input

| Decision | Selected action | Prior outcome supplied | History | Rays |
| --- | --- | --- | ---: | ---: |
| 1 | FOLLOW_WAYPOINT | initial | 0 | 13 |
| 2 | REVERSE | infrared_blocked | 1 | 22 |
| 3 | REVERSE | semantic_action_completed | 2 | 24 |
| 4 | SCAN_FRONT_ARC | semantic_action_completed | 3 | 26 |
| 5 | FOLLOW_WAYPOINT | survey_completed | 4 | 36 |
| 6 | FOLLOW_WAYPOINT | heading_aligned | 5 | 37 |
| 7 | TURN_LEFT_90 | waypoint_reached | 6 | 47 |
| 8 | SCAN_FRONT_ARC | semantic_action_completed | 7 | 48 |
| 9 | FOLLOW_WAYPOINT | survey_completed | 8 | 58 |
| 10 | REVERSE | infrared_blocked | 9 | 75 |
| 11 | REVERSE | semantic_action_completed | 10 | 77 |
| 12 | SCAN_FRONT_ARC | semantic_action_completed | 11 | 79 |
| 13 | FOLLOW_WAYPOINT | survey_completed | 12 | 89 |
| 14 | SCAN_FRONT_ARC | heading_aligned | 12 | 90 |
| 15 | ADVANCE | survey_completed | 12 | 100 |
| 16 | REVERSE | semantic_action_completed | 12 | 102 |
| 17 | REVERSE | semantic_action_completed | 12 | 104 |
| 18 | TURN_RIGHT_90 | semantic_action_completed | 12 | 106 |
| 19 | SCAN_FRONT_ARC | semantic_action_completed | 12 | 107 |
| 20 | FOLLOW_WAYPOINT | survey_completed | 12 | 117 |
| 21 | SCAN_FRONT_ARC | heading_aligned | 12 | 118 |
| 22 | FOLLOW_WAYPOINT | survey_completed | 12 | 128 |
| 23 | FOLLOW_WAYPOINT | waypoint_reached | 12 | 128 |
| 24 | SCAN_FRONT_ARC | heading_aligned | 12 | 128 |

Important boundaries and observations:

- Decision 14 is the first without the earliest execution result: only the
  latest 12 outcomes are supplied. Older obstacle descriptions and rays still
  exist, so this is not total loss of earlier experience.
- Decision 23 is the first with older rays removed from the 128-sample buffer.
  At 22, its first ray is still the initial heading 0 observation; at 23 it
  starts at heading -120, and at 24 at heading -90. This newly observed eviction
  differs from the earlier pre-memory audit, whose runs stayed below the cap.
- At 13 the model chose a leftward hypothesis from a forward-facing sweep.
  At 14 the fresh left-facing reading was 45, not the weaker diagonal returns;
  the model noticed this and requested a new sweep. At 15 it explicitly chose
  one exploratory step rather than the entire leg. At 16 the reading changed
  from 45 to 37, and the model retreated and subsequently chose the other side.
  Its stated intent and the recorded outcomes support that sequence; it is
  not evidence that the previous plan was silently erased.
- A permitted forward action is an execution admission result, not proof of
  whole-body clearance. The packet contains that distinction. Raw/filtered
  IR values alone do not reproduce the worker's stateful admission result.
- The retained rays still mix full scans, front sweeps and travel readings.
  Individual origins/headings are present; sweep membership/timestamps are
  not. Latest-sweep completion metadata does not describe every older ray.
- After decision 24's completed sweep the **test harness** stopped at its
  requested decision bound. The robot did not independently abandon the
  mission. Goal completion remains unvalidated, with about 1,433 mm remaining;
  there were no physics contacts. This is not a successful complex-route test,
  but neither does it prove the route would never finish with more decisions.

These observations identify what the model did and did not receive. They do
not by themselves establish that buffer eviction or unknown metric bounds
caused the route's lack of completion. No history limit, prompt, sensing policy,
motor setting or navigation behavior was changed in this logging follow-up.

## Verification of capture

**84 focused tests passed**, including new checks of exact full payload logging
for both robot context variants across successive decisions, distinct paired
request IDs, and preservation of a timed-out request. A scripted OBSERVE-only
simulator smoke check exercised the actual EV3 validation entry point and
produced a full request/response log; this was deliberately not a model skill
test. No model was called and no physical robot moved in this follow-up.

Artifacts: `/private/tmp/robot-context-packet-tests-20260929.log`,
`/private/tmp/robot-context-logging-smoke-diagnostics-20260929.jsonl` and
`/private/tmp/robot-context-logging-smoke-events-20260929.jsonl`.
