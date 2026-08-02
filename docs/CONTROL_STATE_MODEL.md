# Control State Model

Status: accepted target architecture; migration is incremental.

This decision defines the control-state boundary for autonomous navigation.
It replaces overlapping policy state with one causal path from a user goal to
verified physical evidence. The terms **MUST**, **SHOULD**, and **MAY** are
normative.

## Canonical concepts

### Mission

A `Mission` states the outcome requested by a user or coordinator: objective,
priority, success and failure evidence, and lifecycle. It MUST NOT contain
motor commands, route steps, or transient sensor state. A mission normally
outlives several attempts and intent revisions.

### Intent

An `Intent` is the current persistent semantic tactic for one robot, such as
inspect a hazard, pass it on the left, or resume a goal heading. It MUST name
its mission, revision, target, constraints, required evidence, progress
measure, and supersession conditions. It MUST NOT contain platform-specific
motor details.

The LLM or another reasoning producer MAY propose an intent. The host accepts,
rejects, or supersedes that proposal against the referenced state versions.
Only the accepted intent is canonical. An intent remains valid between model
calls; repeated LLM approval is not required for each motor pulse.

### ExecutionPlan

An `ExecutionPlan` is a short, immutable, revisioned compilation of an accepted
intent against a specific robot and world snapshot. It contains ordered
semantic steps and observable completion predicates. A deterministic executive
owns it.

Plans MUST remain shallow. They MUST NOT become arbitrary workflow graphs,
nested state machines, or containers for dialogue and UI state. A material
change in intent, geometry, topology, calibration, or localization creates a
new plan revision rather than mutating the old plan in place.

### Factual state

`ControllerState` is factual state for one motor-capable device: connection,
controller instance or boot epoch, heartbeat, battery, capabilities,
calibration fingerprint, encoders, active command, receipts, and faults.

`RobotState` describes one physical body: its controllers and sensors,
topology revision, transforms, footprint, fused pose, localization quality,
active mission, accepted intent, and active plan revision.

`WorldState` contains versioned environmental evidence: maps, object tracks,
landmarks, observations, provenance, and uncertainty. It does not own a robot's
mission or motors.

Operational state MUST be reduced by a single writer into immutable snapshots.
Conversation history, experience retrieval, and LLM summaries are advisory
context, not factual control authority.

## Navigation lifecycle

One robot has exactly five causal navigation phases:

- `IDLE`: no active goal;
- `PLANNING`: a goal exists but no executable plan is authoritative;
- `EXECUTING`: one accepted intent and one incomplete plan are active;
- `STOPPING`: a terminal outcome was requested and physical stop is being
  verified;
- `TERMINAL`: the outcome is final and no intent or plan remains active.

These phases describe the navigation lifecycle, not the whole application.
Perception, speech, listening, and deliberation are orthogonal activities and
MAY continue concurrently where their resource claims permit it. They MUST
NOT be added as extra navigation phases or encoded as combinations such as
`MOVING_AND_SPEAKING`.

Within that lifecycle the executive follows one pipeline: integrate evidence,
deliberate only when a valid `PlanningTicket` exists, compile the accepted
intent, authorize and dispatch one bounded step, then verify its correlated
receipt. A current plan MAY keep running while deliberation is pending. The
executive continues, completes, blocks, or requests replanning from verified
evidence; it never infers success from command dispatch alone.

## Ownership

| Concern | Sole authority |
| --- | --- |
| Mission lifecycle | Mission service or user-facing coordinator |
| Planning-ticket creation | Host state reducer/scheduler |
| Intent proposals | LLM and other reasoning producers |
| Accepted intent revision | Host intent authority |
| Execution-plan compilation and progress | Deterministic executive |
| Controller, robot, and world facts | Single-writer reducer/fusion boundary |
| Host action authorization | One logical `ActionGate` |
| A controller's motor bus and hard stop | That controller's local supervisor |
| Speaker playback | Independent audio arbiter |

`ManeuverCommitment` maps to `Intent`. `LocalDetourRoute` maps to
`ExecutionPlan`. `NavigationPlanTail` MUST NOT remain a second physical-plan
authority once migration is complete. UI state MUST be a projection of
canonical facts and events, never another owner of progress.

## Event-driven planning

An LLM call MUST require a finite `PlanningTicket`; the motion tick MUST NOT
call the LLM. A ticket binds at least:

- `ticket_id`, `robot_id`, and mission/intent identifiers;
- the relevant robot, world, map, topology, and transform versions;
- trigger, evidence references, creation time, and `valid_until`.

The immutable proposal offer derived for that ticket binds the permitted
intent transitions, target IDs, reason enums, and response budget. Splitting
the two contracts keeps internal controller identity out of the model-visible
message without weakening the host-side binding.

Tickets are created by decision-relevant events: a new or changed mission, no
usable intent, novel relevant evidence, completed or blocked plan, measured
progress plateau, confidence loss, or user interruption. Sensor jitter and a
periodic control tick are not planning events.

A response based on superseded versions MUST be discarded or explicitly
re-evaluated; it MUST NOT silently revive old intent. At most one intent
transition becomes authoritative for a ticket. Critics and alternative
planners MAY publish evidence or competing proposals, but they never acquire
motor authority.

This preserves agentic behavior: the model chooses and revises tactics,
investigates novelty, comments, and replans from outcomes. Deterministic code
only compiles and safely executes the accepted short-horizon tactic.

## ActionGate and local supervisor

There is one logical host `ActionGate` authority. It MAY be implemented as
small pure rule modules, but there MUST NOT be parallel availability, route,
and runtime veto authorities for the same fact. The gate evaluates freshness,
identity and version binding, capability, resource claims, host policy, and
spatial feasibility. It returns one typed allow or deny result and MUST NOT
silently substitute another action.

Each authorization carries `action_id`, target `robot_id` and `controller_id`,
controller boot epoch, intent and plan revisions, referenced state versions,
an idempotent sequence, and a deadline or TTL.

Plan progress requires a correlated controller receipt. The canonical
execution state records at most one `ActiveDispatch` per controller bus:
first the gate's authorization, then the fact that dispatch began. The
`StepCommandDispatched` event is a write-ahead fact and MUST be durably
journaled before the first outbound command byte. It carries a finite,
profile-selected `settle_by_host_ms` bounded by a generous global ceiling.

A receipt must match the controller boot, action ID, command ID, monotonically
increasing host dispatch sequence, command fingerprint, exact plan step, and
referenced controller state. While `EXECUTING`, at or after the settlement
deadline the host MUST first journal `StepCommandSettlementExpired`; that
event makes no plan or cursor progress. An expired dispatch can never advance
from a `COMPLETED` receipt. It may only be reconciled by an exact non-completed
receipt that blocks the step and creates a basis-bound replan ticket. If
`STOPPING` retains an already-sent dispatch, stop verification instead accepts
only that dispatch's exact `STOPPED` receipt and resulting basis before
entering `TERMINAL`; this reconciliation never advances the plan.

A duplicate, late, cross-controller, or fabricated receipt cannot advance the
plan. Backend-specific tokens such as EV3 request/session sequences remain in
the adapter; they are validated there rather than pretending every controller
shares the same wire protocol. Receipt ingress is timestamped in the host
clock domain, so host deadlines never compare unrelated controller clocks.

The controller-local supervisor is the only process that writes its motor bus.
It rechecks controller identity, epoch, deadline, command sequence, local motor
state, heartbeat, and immediate safety inputs. It may always reject or stop.
It does not interpret missions, intents, maps, dialogue, or personality. Loss
of the host or an expired command stops locally without an LLM round trip.

## Activities and resource claims

Activity is orthogonal state, not a global mode. A robot may move, speak,
listen, and perceive concurrently when resources permit.

Claims MUST be non-blocking leases with bounded TTL and deterministic
preemption. They SHOULD cover scarce actuator resources such as a drive group,
arm, scan actuator, or speaker. Read-only sensors normally use bounded
fan-out, not exclusive claims. Claims MUST NOT grow into a general lock or
workflow system.

Speech uses its own queue and audio arbiter. Playback publishes a speaking
window for echo suppression but does not block navigation. STT publishes
timestamped dialogue events. Camera and vision paths use latest-value or
bounded drop-oldest delivery; a vision result references the source frame,
pose/transform at capture, production time, confidence, and TTL. High-rate
gyro and collision evidence is fused deterministically and never waits for an
LLM.

## Identity and multiple controllers

The following identities MUST remain distinct:

- `world_id`: a shared coordinate and evidence domain;
- `robot_id`: one physical body or independently controlled robot;
- `controller_id`: one hardware control unit and motor bus;
- `controller_instance_id` or boot epoch: one live supervisor incarnation;
- `sensor_id` and `source_id`: observation provenance;
- `action_id`: one semantic physical action;
- `command_id`: one controller-local primitive.

One robot may contain several controllers; one controller belongs to one robot
at a given topology revision. A robot-level coordinator may later compile one
body action into correlated child commands, but every controller retains local
stop authority. Atomic multi-controller motion is not assumed by this
decision.

## LLM contract-size baseline

Sizes below use UTF-8 compact, sorted JSON as produced by `json_bytes`. The
current representative fixture exposes all semantic actions and two hazard
targets. The target column is an acceptance ceiling for the `PlanningTicket`
contract, not a claim that the current implementation already meets it.

| Contract component | Current measured baseline | Target ceiling |
| --- | ---: | ---: |
| System instructions | 5,214 B | 3,072 B |
| Structured response schema | 5,544 B | 3,072 B |
| Wrapper reserve | 2,048 B | 2,048 B |
| Fixed contract subtotal | 12,806 B | 8,192 B |
| Dynamic planner context | 57,344 B target; 65,536 B hard limit | 24,576 B routine ceiling |
| Accounted routine prompt | 70,150 B | 32,768 B |
| Conservative admission estimate | 26,022 tokens | 12,428 tokens |
| Maximum model output | 520 tokens | 320 tokens |

The implemented shadow intent fixture is already substantially smaller than
those ceilings: 758 B of system instructions, a 980–1,025 B response schema,
1,036 B of causally selected context, and 4,822 B accounted in total including
the 2,048 B wrapper reserve. Its minimal `FOLLOW_DIRECTION` output is 29 B.
These are deterministic contract measurements, not live model-latency claims.

The compact planner bridge makes no model call when the host offer contains
exactly one concrete valid proposal. When a genuine choice remains it makes
exactly one model call for the ticket. Retry and backoff remain scheduler
policy; the bridge owns neither a loop nor another planning state machine.

For reference, the current response schema measures 1,557 B for the minimal
`OBSERVE`/empty-maneuver fixture and 6,366 B with all actions and 64 target
identifiers. The target contract sends mission, accepted intent, current plan,
fresh evidence, relevant world slices, and compact outcome history; it does
not send duplicated lifecycle projections. Richer memory remains available on
the host and is selected into a ticket when causally relevant.

These measurements MUST become an automated regression fixture before the new
contract is authoritative. Size reductions MUST NOT remove evidence required
to distinguish a genuine retry from progress after a changed physical basis.

## Incremental migration and replay

Migration MUST be reversible and must not use a big-bang runtime replacement.

1. Record deterministic golden traces containing observations, state versions,
   mission, commitment, route/tail, allowed and vetoed actions, model result,
   command, receipt, scan evidence, and terminal outcome. Replay uses recorded
   model results and never contacts an LLM or hardware.
2. Introduce read-only adapters: `ManeuverCommitment -> Intent` and
   `LocalDetourRoute -> ExecutionPlan`; represent plan tails as a temporary
   legacy plan type.
3. Extract the existing host checks behind the `ActionGate` boundary without
   changing their semantics. Exact compatibility extractions MAY replace their
   former call sites directly when parity tests cover those calls. Any new or
   changed gate semantics MUST first run in shadow mode against identical
   snapshots, comparing allowed action, denial reason, selected plan step,
   route progress, and terminal result with the authoritative path.
4. Before physical authority, define and replay an explicit controller-loss
   transition. A permanently disconnected, powered-off, or rebooted controller
   cannot produce a same-instance command receipt; recovery must invalidate
   the old instance without plan progress and bind any replacement boot as a
   new controller instance. Shadow mode MUST NOT pretend that receipt exists.
5. Make changed gate semantics authoritative behind a feature flag after
   replay parity: simulator first, then a bounded EV3 canary. Keep dual
   decision events long enough for rollback.
6. Stop creating new physical `NavigationPlanTail` values, migrate dashboard
   projections, then remove duplicate state one producer and consumer at a
   time.
7. Add EV3 and 51515 as controller adapters behind the same semantic
   observation, authorization, command, and receipt contracts. Transport
   details MUST remain outside the control-state core.

Replay MUST prove: no command without an authorization; one receipt or expiry
per command; stale epochs and versions are rejected; at most one drive lease
per controller; stop preempts motion; and identical traces reduce to identical
decisions. Clocks, IDs, and randomness must be injected. Large camera and
audio payloads are stored by reference rather than in the event journal.

## Explicit non-goals

This decision does not introduce:

- a general behavior-tree, workflow, or statechart engine;
- a rule DSL for `ActionGate`;
- Kafka, CRDTs, a distributed global blackboard, or microservices;
- distributed transactions for composite-robot motion;
- full SLAM, a universal object ontology, or probabilistic sensor fusion;
- LLM voting, continuous evaluator polling, or an LLM in a safety loop;
- a vector database, arbitrary plugin framework, or unbounded event history.

Those capabilities may be justified by measured future requirements. They are
not prerequisites for EV3 and 51515 to share a clear, replayable control model.
