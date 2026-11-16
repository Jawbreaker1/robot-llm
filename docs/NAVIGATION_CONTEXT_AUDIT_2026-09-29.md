# Navigation context audit — 2026-09-29

## Conclusion and scope

The reviewed failures are not explained by exhaustion of the model context
window. The application deliberately removes planning continuity and flattens
sensor history before the model sees it. Those are concrete representation
limitations, not proof that fixing them will resolve every collision.

Reviewed 58 recorded Astra decisions (front box, eight boxes, maze) and 42
historical Qwen decisions. Reconstructed requests using the current production
request builder; all nine source hashes from the Astra comparison still match.
Historical Qwen contexts are exact recordings, but their original prompt
revisions are not reconstructed by this audit. Two Astra decisions were sent
again to measure provider-reported usage. Their returned actions were **not
executed**. No physical robot, navigation implementation, prompt, motor
calibration or model setting was changed.

## Size: available capacity is not the missing ingredient

| Recorded Astra scenario | Decisions | Complete request size | Maximum rays | Maximum history items |
| --- | ---: | ---: | ---: | ---: |
| Front box | 10 | 12,516–24,981 bytes | 70 | 9 |
| Eight boxes | 24 | 12,510–32,899 bytes | 110 | 12 |
| Maze | 24 | 12,506–31,106 bytes | 102 | 12 |

Actual replays through resolved `gpt-6-astra`, low reasoning:

| Input | Input tokens | Output tokens | Model latency |
| --- | ---: | ---: | ---: |
| First front-box decision | 2,557 | 308 | 10.965 s |
| Largest recorded context: eight boxes, decision 24 | 8,122 | 372 | 13.415 s |

These are provider-reported token counts, not a bytes/4 estimate. They are
Astra counts, **not verified Qwen tokenizer counts or proof of its current
70k hosting configuration**. Both providers use the same application-built
context. The subscription backend rejected the optional output-limit field;
the adapter retried without it, so 4,096 was requested but not enforced there.
Neither measured response was long or truncated.

The app separately caps the serialized request at 65,536 bytes. An oversized
request raises an input error; it is not automatically summarized. Increasing
history constants blindly could hit that cap even with room in a 70k model
window. The cap was not reached in these recordings.

## Confirmed findings

### 1. Planning continuity is discarded, while the route is retained

Each decision is a new system + user request. There is no accumulated chat
transcript or previous-response chain. That design can work, but only the
explicitly retained state is available next time.

EV3 stores waypoints and the latest model-owned obstacle estimates. However,
`decision.plan` and `assessment` are published for diagnostics and not retained
in session history. The context constructor never supplies `active_plan`, so
it is empty in **all 100 reviewed decisions**, including decisions after a
nonempty plan. A waypoint's purpose or an obstacle's short basis can preserve
some intent, but that is indirect and incomplete.

Sources: `ev3_navigation_session.py:129–174`,
`ev3_navigation_context.py:9–21,47–89`,
`lm_studio_controller_action.py:354–368,697–711`.

This is a contract mismatch: we request an advisory action plan but discard
it before the next decision. It is not evidence that the waypoint route itself
is being erased, nor that blindly executing an old action plan would be right.

### 2. Visit/outcome memory is limited to twelve events

The full session history exists locally, but only its last twelve results are
sent to the model. At decision 24, results 1–11 are absent from that history.
There is no separate complete visited-place/dead-end memory in the EV3 context.
Older sensor origins and retained obstacle descriptions still convey some
earlier experience; the model is not completely amnesic.

Sensor memory is separately limited to 128 observations. The largest reviewed
Astra run contains 110, with its initial ray still present. **No sensor eviction
occurred in these three runs**, so that limit does not explain their contacts.
It matters for the requested longer autonomous missions.

Sources: `lm_studio_controller_action.py:32,36`,
`ev3_navigation_context.py:9–21`, `ev3_navigation_body.py:130–139`.

### 3. Scan results are flattened, and their metadata describes only the latest sweep

The model receives each ray's actual observation position, episode heading,
raw value and filtered value. These are useful facts. However, the list combines
the initial 360-degree survey, later front sweeps and readings during travel.
It drops observation timestamps and has no scan ID, scan membership or sample
source. `complete`, `reason` and `sweep_degrees` describe only the latest sweep,
not all the rays under that object. A model must reconstruct which readings
belong together from list order, positions and history.

For example, before the first eight-box contact, the current robot heading
was 91.7 degrees, while the latest completed front sweep had been centered near
1 degree. Both old rays and the newer 91.7-degree reading were in the same list.
Their individual headings were correct. Astra noticed the new reflection and
requested a new scan; that scan then contacted an obstacle. This example does
**not** prove scan grouping caused the contact.

Sources: `ev3_navigation_context.py:42–46,70–74`;
eight-box decisions 13–15 in the comparison artifact.

### 4. EV3 has body geometry, but not Blast's metric map

The context correctly includes the measured asymmetric body: 110 mm front,
90 mm rear, 100 mm left, 130 mm right, 10 mm margin, 181 mm turn-sweep radius.
It identifies the IR sensor in the head and the right-hand color sensor as
non-ranging. The numeric head-to-drive sensor offset is explicitly unmeasured.

The goal, current pose, goal vector, remaining waypoint route and measured
partial-motion outcomes are present in a consistent episode frame. The prompt
says that scanning rotates the entire body. Simulator obstacle rectangles and
the IR response curve are not disclosed to the planner.

Unlike Blast's metric path, EV3 receives **no occupancy grid, measured hit
coordinates, obstacle clusters or calculated body-clearance geometry**. Its
input is unitless reflection data and model-owned hypotheses. Sharing the
planner interface therefore does not mean sharing equivalent spatial evidence.

### 5. The obstacle representation has an unresolved estimation gap

The prompt correctly forbids converting raw IR strength directly into distance
and inventing measured edges. It also asks for millimetre bounds, with null for
edges not established by observations. Numeric estimates are allowed in
principle, and qualitative evidence can be written in the 300-character basis.
Nevertheless, Astra returned only null bounds throughout all 58 decisions.

This is not a formal logical contradiction: sometimes the evidence genuinely
does not locate an edge. But the current contract's only structured shape is
a set of metric bounds. Coarse angular limits and uncertainty live in prose.
The repeated result suggests testing whether this representation encourages
unhelpful all-or-nothing certainty. That is a hypothesis, not a proven cause.
Qwen sometimes emitted numeric bounds; their existence alone did not establish
that those bounds were supported or that its navigation was better.

Sources: `navigation_obstacle_estimates.py:8–40`,
`lm_studio_controller_action.py:133–149`.

## Smallest next refactor to test

1. Repair the existing planning-memory contract: retain the latest concise
   model-authored intent with its measured result, replacing unused/duplicated
   fields rather than adding a second planner. Keep remaining waypoints.
2. Present observations as identifiable sweeps plus current readings, with
   actual origins/headings and completion status. Retain useful visited/recovery
   history under an explicit input budget instead of a blind twelve-event cut.
3. Clarify the existing obstacle-belief representation: distinguish observation,
   approximate hypothesis and unknown extent. Permit useful coarse descriptions
   without promoting them to measured ranges or guaranteed body clearance.

Do not add a deterministic detour selector, guessed distance conversion,
mandatory scan loop or stronger blanket stop rule. Unknown space remains
explorable, and actual obstructions return control to the model.

Validate these changes separately on the same recorded decision states, then
run closed-loop front-box/eight-box/maze trials using unchanged hidden worlds
and production execution. Give the maze sufficient test steps: its previous
24-step result was still progressing without contact. A better one-step answer
does not establish successful navigation.

## Artifacts and measurement reference

- `/private/tmp/robot-context-audit-20260929.jsonl`: request sizes and per-field
  inventory for all 100 recorded contexts.
- `/private/tmp/robot-context-front-replay-20260929.jsonl` and
  `/private/tmp/robot-context-many-replay-20260929.jsonl`: compact actual usage.
- `/private/tmp/robot-context-replay-20260929-diagnostics.jsonl`: exact replay
  requests and responses, with no credentials.
- `/private/tmp/robot_context_audit_20260929.py`: read-only audit/replay harness.
- Original simulations: `/private/tmp/robot-astra-ab.Zvifux/` and the historical
  Qwen files named by the harness. Temporary artifacts are not Git-tracked.

The official [context-window documentation](https://developers.openai.com/api/docs/guides/conversation-state#managing-context-for-text-generation)
distinguishes input, output and reasoning token budgets. This audit uses actual
provider usage rather than assuming that bytes equal tokens or that enlarging
a model window changes application-side memory retention.
