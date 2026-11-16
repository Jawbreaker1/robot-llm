# Information, context and memory review — 2026-09-29

## Executive conclusion

The application has shared model and control interfaces, but not yet a coherent
information contract spanning navigation, conversation, local maps, shared maps
and the GUI. The same word, such as history or plan, currently refers to
different data with different lifetimes. Several consumers silently lose fields;
one shared-map consumer rejects the current EV3 producer entirely.

This is a data-ownership and lifecycle problem before it is a context-window
problem. The next refactor should unify the existing state and its projections,
not introduce another navigation engine, database service, event bus or automatic
route planner. Robot-specific sensing and motor execution remain necessary.

This review made no production code, prompt, settings or physical changes. It
used source inspection, recorded simulator data, offline contract probes and
158 existing tests. It does not claim to have corrected or physically validated
the findings. The earlier [navigation-context audit](NAVIGATION_CONTEXT_AUDIT_2026-09-29.md)
contains measured token usage and detailed planner evidence.

## Current ownership and lifetimes

| Information | Blast today | EV3 today | Model / GUI consequence |
| --- | --- | --- | --- |
| Pose and execution | Per-episode motion executor; encoder/gyro-derived heading | Per-episode body; absolute encoder reconciliation | Shared concepts, separate adapters; retain this hardware distinction |
| Mission and route | Local run variables plus `_BlastEpisodeMapTrace` | `EV3NavigationSession` plus `DirectionalMission` | Waypoints survive ordinary replanning, but no common mission-state contract |
| Sensor memory | Retained metric echoes, 16 scan views, 128 visited cells | Latest 128 IR observations; latest survey metadata | Different sensor capabilities are legitimate; retention semantics differ too |
| Planner history | Last 12 compacted semantic events | Last 12 result events | Both lose earlier intent; both send an empty advisory `active_plan` |
| Obstacle interpretation | Host-derived metric echo clusters and keep-outs | Model hypotheses with nullable metric bounds | Not equivalent evidence; IR cannot honestly become ultrasonic range data |
| Robot conversation | Current text + projected local status | Same | No preceding user or robot dialogue reaches either robot model |
| Visible robot dialogue | 40 messages per target in browser memory | Same | Reload loses it; it is not server/model conversation memory |
| Workbench conversation | Separate server `ConversationStore`, last 20 messages supplied | Not the robot conversation path | A functioning workbench history does not establish robot history |
| Runtime timeline | Per-robot event/snapshot rings; GUI keeps 1,000 of each | Same | Good diagnostic retention, but not model working memory |
| Persistence | Current route/map starts afresh per mission | Same; old file-memory options remain exposed | Neither active navigation flow uses `NavigationMemoryStore` |
| Shared spatial view | Optional fixed-start compositor | Same, currently incompatible with EV3 trace | Read-only display composition, not shared planner knowledge |

Runtime service limits: 4,096 events / 8 MiB, 4,096 snapshots / 32 MiB, explicit
drop counters and cursors. Navigation request cap: 64 KiB. Robot-conversation
facts cap: 16 KiB; complete request cap: 32 KiB. These are byte caps, not model
token budgets. The previous audit measured 2,557–8,122 input tokens in two actual
Astra replays. It did not measure Qwen's tokenizer or current hosting limit.

## Confirmed findings, in priority order

### 1. Robot chat is visually multi-turn but operationally single-turn

`RobotInputService._history` is an idempotency cache, not dialogue memory. It
stores request IDs and completed responses to prevent duplicate effects. The
interpreter receives only `RobotInput` and current status facts. Its request
contains one system message and one user object, with no earlier dialogue.

The GUI independently appends replies to `robotDialogueByTarget`, retaining 40
messages only in browser state. The unrelated workbench service has a real
server conversation store and sends up to 20 messages. This is a genuine
parallel conversation pipeline, not merely a different robot voice.

Offline reproduction: send a unique first message and a canned unique response,
then ask what was just mentioned. Neither earlier text appears anywhere in the
second actual serialized model request. Resending the second request ID correctly
does not call the model again. Preserve that idempotency behavior when adding
dialogue continuity; do not repurpose the cache as authoritative chat history.

Sources: `robot_input_service.py:64–108,229–232`;
`lm_studio_robot_input.py:340–389`; `dashboard_web/app.js:19,94,1843–1860`;
`dashboard_service.py:153–176`; `dashboard_state.py:391–464`.

### 2. Status conversation and GUI do not receive the same obstacle knowledge

EV3 publishes model beliefs at `navigation_trace.obstacle_estimates`.
`project_robot_status_facts` selects `object_hypotheses` instead and omits the
navigation trace. In an actual recorded eight-box map, three retained model
beliefs become zero obstacle hypotheses in conversation facts. The conversation
still gets eight latest IR observations and runtime route/message data: it is
not wholly blind, but it cannot reliably explain the same beliefs shown in GUI.

A second exact mismatch: EV3 runtime scan status is `{complete, sweep_degrees}`,
while `_scan_summary` recognizes other field names. A completed 120-degree EV3
scan becomes `{}` in the conversation facts. The UI runtime timeline retains
the original raw scan object, so two readers of the same run disagree.

Blast's object hypotheses and scan fields are better matched to this projector,
but its planner-specific echo clusters, route-rejection evidence and clearance
view are not the conversation's map either. Summaries may intentionally be
smaller; they must not silently change which knowledge is available.

Sources: `physical_spatial_map.py:222–269`;
`robot_status_facts.py:20–45,133–163`; `ev3_episode_adapter.py:65–73`.

### 3. The shared-map schema is incompatible with today's EV3 producer

Passing the actual recorded EV3 map through `SharedSpatialMapCompositor` yields
`unavailable / source_navigation_trace_invalid`. There are two independent
incompatibilities:

- Its strict allowed-field list rejects `obstacle_estimates`.
- Its imported Blast advisory-waypoint validator requires additional fields
  including the legacy literal `source: GEMMA_MODEL`; EV3 supplies x/y/purpose.

Removing only estimates from an offline copy is insufficient. Removing both
estimates and the advisory waypoint makes that copy available; the goal and
full waypoint-route subcontracts validate. This is an isolation experiment,
**not a proposed fix that throws information away**.

Sources: `shared_spatial_map.py:37–50,272–310,541–646`;
`blast_spatial_map.py:206–223`; offline recorded-map reproduction.

### 4. Planner working memory is narrower than the UI suggests

Both planners are stateless requests with explicitly reconstructed context.
Both preserve waypoint routes, but their generated action plans are displayed
and then omitted from the next planning context. Blast explicitly sends
`active_plan=()`; EV3 leaves it at the empty default. The most recent assessment
and the intent of an inspection are not directly retained as planner memory.

Both cap history at twelve semantic/results events, while keeping more events
locally. Blast has visited cells; EV3 has no equivalent explicit visited-route
memory beyond retained result poses and ray origins. EV3 combines scan and travel
rays into one list without sweep membership. Its 128-ray cap was not reached in
the three recorded Astra trials, so it does not explain those contacts.

This is not an argument to execute stale action tails automatically. Preserve
the model's concise intent and actual result as revisable information, alongside
the remaining route, instead of requesting and then discarding a plan field.

Sources: `blast_episode_planning.py:91–146,238–274`;
`blast_episode_adapter.py:1321`; `ev3_navigation_context.py:9–21,42–89`;
`ev3_navigation_session.py:129–175`.

### 5. Memory lifetime is not yet explicit across missions and restarts

Both active flows create fresh mission-local navigation state. Blast resets its
map generation and pose/hypothesis history in `begin_episode`; EV3 constructs a
new body/session and changes the map's episode frame. Reconnecting within a
mission is different: measured pose and route are retained/reconciled.

The old atomic-file `NavigationMemoryStore` exists and has tests, but neither
current robot navigation path constructs it. EV3 CLI/binding memory-path/reset
options remain, yet `EV3RSTORMProfile.build_adapter` does not consume them.
These names imply persistence that is not present in the active flow.

Do not reconnect the old navigation engine just to recover persistence. Decide
what should survive, then serialize the current state contract. Especially,
remembering an obstacle is not permission to reuse its coordinates after a robot
is manually repositioned or its frame is reset.

Sources: `blast_episode_adapter.py:1040–1045`; `blast_spatial_map.py:823–855`;
`ev3_navigation_body.py:31–38`; `ev3_episode_adapter.py:80–97`;
`ev3rstorm_profile.py:317–340,408–478`; `navigation_memory_store.py:122,331`.

### 6. Evidence identity and freshness need one definition

EV3 assigns GUI evidence IDs from list positions. When the 128-observation ring
evicts its first sample, the same evidence ID denotes the next observation.
An offline retention probe reproduces ID reuse with a different origin. That is
unsafe for later deduplication or references from a model belief; it is not
evidence of an already-observed physical navigation error.

EV3's map publication also stamps the pose with publication time, not a retained
measurement timestamp. A decision/status publication can make unchanged pose
data appear newly observed. Individual IR observation timestamps are preserved.
Separate sensor time, state publication time and model decision time.

Status facts sample map and control separately; map and runtime GUI polling are
also separate. This need not become a transactional database, but projections
need common robot/mission/frame/revision identities so mismatches are detectable.

Sources: `physical_spatial_map.py:222–256`; `robot_input_service.py:74–88`;
`dashboard_web/app.js:1580–1601`.

### 7. Goal text and executable goal are independent today

Both adapters create a directional mission using their configured minimum
forward progress, default 800 mm. User goal text is passed separately to the
model; the robot-input classifier returns an intent, not a resolved geometric
mission. An offline EV3 context can therefore contain “Drive two metres straight
ahead” alongside a target at x=800 mm. This does not explain the earlier tests
where the requested goal was 800 mm, but is an important information-contract
gap before claiming general or dynamically generated missions.

There must be one accepted mission definition with provenance back to the user
request, used consistently for planning, execution, completion and GUI display.
An ambiguous request should be clarified, not silently assigned a hidden target.

Sources: `ev3_episode_adapter.py:22,92–97`; `blast_episode_adapter.py:90,123–124`;
`blast_episode_map_trace.py:64–70`; `physical_navigation_mission.py:32–89`.

### 8. Two robot connections are not yet a shared agent memory

CLI wiring supplies each robot input service its own local map. Neither planner
is supplied the shared compositor's output. The optional shared view requires
explicit fixed-start transforms and a peer server; it is read-only presentation.
The multi-robot console also deliberately installs one `RobotEpisodeGate`, which
allows only one accepted physical mission across the two services at a time.

Thus shared knowledge, robot-to-robot conversation and simultaneous autonomous
missions are not established by today's integration. Do not simply remove the
gate during this refactor; the missing coordination/state contract comes first.

Sources: `dashboard_cli.py:705–766,1036,1066–1099`;
`robot_control_service.py:60–78`; `shared_spatial_map.py:648–722`.

## What already works and should be kept

- Per-robot IDs, episode/frame generations, detached snapshots at map boundaries,
  and request-id idempotency are useful foundations.
- GUI target switching rejects late responses for another selected robot.
  Runtime history has explicit cursor/drop handling rather than silent gaps.
- Shared-frame binding refuses to silently reuse a transform after a frame
  generation changes. This protects against accidentally mixing coordinates.
- Motor permission is separate from clearance. Unknown/no-return observations
  do not have to invalidate the mission; recoverable transport/planner failures
  keep the mission alive.
- The renderer distinguishes measured ultrasonic returns, directional IR cues
  and model estimates. IR cues deliberately have no invented metric endpoint;
  complete estimated rectangles are marked as estimates, unknown edges as unknown.

## Proposed minimal target contract — not implemented

Use one authoritative **per-robot state**, with ordinary read-only projections
for planner, conversation and GUI. Refactor existing owners toward this; do not
create a third store that duplicates both existing map implementations.

| Part | Owner and meaning |
| --- | --- |
| Identity | Robot, controller, mission, frame generation and state revision |
| Observations | Actual pose/sensor/motion evidence, stable IDs, source time, modality, quality; metric range optional |
| Mission | Accepted goal, remaining waypoints, latest intent/result, visited/backtracking memory |
| Beliefs | Model-owned obstacle/passability estimates with supporting evidence references and explicit unknowns |
| Dialogue | Ordered visible user/robot messages, participants, reply/gesture outcome; separate from deduplication |

The robot adapters normalize sensor samples and motor receipts. They keep
hardware-specific IR/ultrasonic semantics, geometry and transport behavior.
The model retains ownership of route, side, investigation and replanning.
The GUI renders the same beliefs and observations, without inventing geometry.

Do not feed every diagnostic receipt to the model. Supply current state, useful
recent observations and explicit durable mission memory. Bound that projection
by an input budget, expose what was summarized/evicted, and reserve output space.
An input-size error must not become an unexplained mission failure. The GUI
already has token/byte fields, but current navigation results do not populate
them; publish measured usage rather than blank counters or estimates as facts.

### Lifetime rules to define explicitly

- **Ordinary movement, partial motion, rescan:** retain mission, route and useful
  evidence; update only what the new result actually changes.
- **Connection loss:** retain beliefs and last verified pose, mark live motion
  uncertain until reconciled; never replay unconfirmed motion blindly.
- **New mission, unchanged verified frame:** retain applicable world knowledge;
  replace the mission, not unrelated conversation history.
- **Manual relocation / new frame:** archive old geometry or explicitly rebind it;
  never reuse numeric coordinates as if they were still current.
- **Application restart:** say exactly what was restored and what is unverified.
  A saved transcript is not a restored physical localization.
- **Cross-robot sharing:** exchange source-attributed observations/beliefs only
  with a known coordinate relationship, or describe them nonmetrically. Sharing
  a model alone does not create shared memory or confer motor authority.

### GUI implications

Keep the normal map simple. In its existing inspector expose: measured versus
estimated versus unknown, data age, active frame/mission, and the state revision
used by the last model decision. The current live map can legitimately be newer
than the model's last input; show that difference rather than imply they are
identical. Give robot dialogue a real server-owned conversation identity so a
reload and follow-up question do not have unrelated memories.

## Refactor order and acceptance checks

1. **Lock down producer-to-consumer contracts with recorded data.** Add tests
   for EV3 and Blast snapshots consumed by planner, status facts, local renderer
   and shared compositor. Fix the field mismatches, not by dropping useful data.
2. **Unify current state/projections and planning continuity.** Keep robot
   sensor differences; remove discarded/duplicate plan fields and legacy memory
   settings that do not do what they claim. Make goal ownership explicit.
3. **Connect dialogue and define retention.** Preserve visible turns per robot,
   useful mission intent, scan identity and visited-route memory. Restore a
   measured input-usage view in the GUI. Define restart/frame behavior before
   adding persistence or robot-to-robot sharing.
4. **Validate behavior.** Replay recorded contexts, then real-model closed-loop
   simulation, then physical navigation. A schema or memory repair is not itself
   a successful box-navigation test.

Minimum cross-layer checks: two-turn recall with no accidental motion;
identical obstacle belief/scan completeness across projections; accepted shared
EV3 and Blast snapshots; stable IDs after eviction; no duplicate motion on
reconnect; explicit new-frame invalidation; matching textual/geometric goals;
long-history requests remaining within budget without killing missions.

## Validation and artifacts

158 existing tests passed across robot input/model, status facts, local maps,
shared maps and GUI. Those tests use independently authored fixtures, including
older EV3-style hypothesis/scan objects. Passing them did not detect the current
producer-to-consumer mismatches demonstrated above.

- `/private/tmp/robot_information_audit_20260929.py`: offline probe harness;
  model replies are canned, no network/speech/motor execution.
- `/private/tmp/robot-information-audit-20260929.jsonl`: compact reproduced facts.
- `/private/tmp/robot-information-tests-20260929.log`: selected-test results.
- `/private/tmp/robot-astra-ab.Zvifux/many-boxes.jsonl`: recorded current EV3 map
  used for the cross-layer probes.

The shared-map removals happened only on deep-copied test data. No production
fields were removed, no physical run was started, and no additional full
navigation simulation was claimed by this review.

## First implementation slice — information contracts

The review above records the pre-change findings. The subsequent implementation
deliberately fixes projection mismatches, not planning or motor behavior:

- Both current robot map publishers use `advisory_waypoint_trace` with the same
  provider-neutral `MODEL` marker. The unused Blast source selector and duplicate
  marker construction were removed. Readers still accept `GEMMA_MODEL` in older
  saved Blast maps.
- Status/conversation facts retain the existing navigation trace's goal, route,
  waypoint and model obstacle estimates. Estimates remain separate from sensor
  hypotheses. EV3 scan `complete` and `sweep_degrees` now survive the summary,
  including `complete: false`.
- The shared compositor validates model estimates with the existing decoder.
  It retains them as `local_obstacle_estimates` in the trace's `local_frame_id`,
  rather than rejecting the robot or inventing rotated axis-aligned edges.
- Local and shared GUI views use the same estimate renderer. Shared drawing
  applies the existing frame calibration to all four corners. Unknown edges
  remain unknown; finite model rectangles are still visibly estimates, never
  sensor hits or collision geometry. The shared inspector identifies the source
  robot and local frame; local coordinates are not shown as world coordinates
  in a transformed polygon's tooltip.

`test_navigation_information_contract.py` exercises actual EV3 and Blast map
publishers through status facts, shared composition, GUI normalization and the
map presenter, including 90-degree and oblique transforms, incomplete scans,
unknown edges, invalid bounds and detached snapshots. The existing context-budget
test now also includes eight maximum-length model obstacle descriptions.

An offline replay checked 134 recorded EV3 publications (24 front, 55 many-boxes,
55 maze). It preserved every obstacle estimate, with maximum status-fact sizes
7,844 / 8,222 / 7,976 bytes respectively, below the existing 16 KiB limit. For
shared projection, old saved raw waypoints were re-published through the current
marker producer; no sensor evidence was changed. This was data-contract replay,
not 134 navigation runs or a fresh live-model validation.

Not changed in this slice: motors, calibration, odometry, scan execution,
clearance/stop policy, planner prompts, planning history, robot dialogue history,
retention IDs, mission goal resolution, persistence or peer coordination. The
remaining audit findings still need their own bounded follow-up; this does not
claim that navigation or conversational memory is now fully corrected.

Validation: the final complete quality check passed **2,071 tests**, plus its
JavaScript syntax checks (`/private/tmp/robot-information-quality-20260929-confirm.log`).
`git diff --check` was clean. During an earlier full run, the unchanged legacy
`test_one_gesture_per_epoch_and_host_budgets` intermittently observed zero gesture
drops. It then passed three isolated repetitions and the final full run. Its tick
hook waits for navigation publication, not arm-queue consumption, so cancellation
can win the race against the metric assertion. This timing-sensitive test remains
a separate follow-up; no runtime workaround was added or test assertion weakened.

## Second implementation slice — decision continuity

Both production robot flows now retain one `previous_decision`: the model's
chosen action, explicit assessment and advisory plan. `ControllerActionDecision.memory()`
and `ControllerActionContext` provide the same bounded, detached contract for
both bodies. The always-empty `active_plan` input and its validation/plumbing
were removed rather than introducing a parallel plan executor.

The retained waypoint route and measured results remain in their existing
fields. Prior intent is not motion evidence, and its plan is never executed as
a queue. The next planner sees the current pose, outcomes and remaining route
alongside its previous intent. The common prompt states that distinction.
Unknown motion after a lost response still requires encoder reconciliation;
storing the prior intention does not turn it into a successful action.

EV3 stores the decision before execution so a transport interruption cannot
erase it. Blast stores the returned planner decision before waypoint admission
and execution, independently of continuation pulses. A proposal discarded
inside planning because fresh evidence changed is not promoted to the accepted
decision memory. Provider errors preserve the last accepted decision. A new
mission starts without another mission's decision memory. Only one decision is
retained, using the existing assessment and plan limits; no hidden reasoning,
conversation transcript, new database or new navigation rule is introduced.

Official [conversation-state documentation](https://developers.openai.com/api/docs/guides/conversation-state#manually-manage-conversation-state)
confirms that independent generation requests do not supply implicit previous
state. This implementation carries explicit application data in the next
request; it does not claim to resume an entire provider conversation or its
reasoning state, and does not change either provider's transport.

Regression coverage includes detached snapshots, prior actions no longer
available at the current pose, bounded memory, multiline assessments accepted
by the existing reply contract, model outages, partial scans and lost motor
receipts through both production adapters. Planning ownership, physical motion,
calibration, sensing and route admission are unchanged. Robot dialogue history,
scan retention identities and richer visited-route memory are still separate
remaining work.

Validation of the second slice:

- Focused regression tests: **204 passed**. Complete quality check: **2,074
  tests passed**, plus JavaScript syntax checks. Logs:
  `/private/tmp/robot-decision-memory-tests-20260929.log` and
  `/private/tmp/robot-decision-memory-quality-20260929.log`.
- Fresh `codex/gpt-6-astra`, low-reasoning simulations used the production EV3
  episode adapter, session, body and planner. Both started with empty decision
  memory; all 32 subsequent planner inputs exactly matched the preceding
  decision's action, assessment and plan. No hidden reasoning was forwarded.
- `front`: reached the world goal in **10 decisions**, with **20 mm estimated
  remaining distance** and no physics contacts. This matches the earlier
  baseline's decision count and remaining distance, so it does not demonstrate
  a navigation-quality improvement by itself.
- `many-boxes`: **not completed within 24 decisions**, approximately **1,433 mm
  estimated distance remaining**, no physics contacts. The model performed
  directed scans at lateral staging points and revised its route after blocked
  progress. The earlier 24-decision run also failed to finish and had three
  contacts. One stochastic run each is not enough to attribute that difference
  to memory, nor to claim that the complex navigation is now solved.
- The many-boxes world starts at `(-200, -500)` whereas navigation starts at
  local `(0, 0)`. Transforming the final local estimate `(825, -821)` into world
  coordinates gives `(625, -1321)`, versus simulated truth `(596, -1332)`:
  approximately **31 mm** position error. Raw world/local coordinates must not
  be compared without that frame offset.
- Maximum serialized planner-context sizes were 15,510 characters (`front`)
  and 25,351 (`many-boxes`); these are character counts, not token counts.
  Logs: `/private/tmp/robot-decision-memory-astra-front-20260929.jsonl` and
  `/private/tmp/robot-decision-memory-astra-many-20260929.jsonl`.

The simulator still uses an explicit approximate IR response curve, ideal rays
and encoder motion. Test-harness decision/time bounds are not a new production
mission deadline. These results verify decision continuity and expose remaining
navigation limitations; they do not establish physical reliability. No physical
robot run was performed for this slice.
