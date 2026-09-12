# EV3 integration: active plan

## Objective and boundaries

One application owns goals, model-authored routes, conversation and shared
knowledge. BLAST and EV3 supply different bodies and sensing capabilities.
The robots must interact with each other, not only with the user: exchange
observations, ask for help and coordinate tasks through this application.
Reuse working code; do not build a new framework or copy BLAST's whole stack.
Qwen chooses the route. Executors finish its chosen movement and report what
actually happened; they do not invent a detour or request a decision per pulse.

BLAST's current physical checkpoints are clear-floor arrival and a left detour
with boxes ahead and on the right. The operator confirmed map/reality agreement.
Conversation, speech and a returning arm wave were also physically checked.
These are useful baselines, not proof of arbitrary-room or EV3 navigation.

## Inventory

- Shared already: `RobotInputService`, `LMStudioRobotInputModel`,
  `RobotControlService`, dashboard routing and speech-runtime infrastructure.
- Different navigation paths: EV3 uses `LMStudioNavigationPlanner` and
  `PhysicalNavigationRuntimeAdapter`; BLAST uses `LMStudioControllerActionPlanner`
  and `BlastEpisodeRuntimeAdapter`.
- EV3's active `build_local_detour_route` constructs waypoint geometry for the
  model-selected obstacle and side. BLAST accepts model-authored coordinates.
  This is live behavior to replace, not dead code to delete immediately.
- Keep hardware differences: EV3 SSH worker, IR/touch, encoder calibration and
  scan rig versus BLAST BLE, ultrasound and IMU. IR is not metric ultrasound.
- EV3 English speech still selects `MacOSSayWAVSynthesizer`; Swedish uses Piper.
- Both can be configured together, but `RobotEpisodeGate` serializes physical
  missions. A common displayed map does not establish shared planning knowledge.
- The shared-world simulator has both hardware adapters. Existing adapter tests
  do not establish real-Qwen EV3 navigation or simultaneous physical success.

Inventory validation: 39 EV3-profile, dashboard-composition and simulation-adapter
tests passed. No runtime code was changed and no physical commands were issued.

## Checkpoints — stop and report after each

1. **Preserve the BLAST baseline.** Commit and push the validated working tree
   before runtime integration. At inventory time, `main` still has uncommitted
   changes. The previous full quality gate passed 2,116 tests; the prompt-only
   speech follow-up passed its 13 tests and three real-Qwen checks.
2. **Bring up EV3 without changing navigation.** Check its actual connection,
   sensors, encoder directions, GUI conversation and onboard speech. Use the
   existing shared classifier. Replace the English voice path with Piper as
   a small independently tested change. Request physical readiness before
   moving its motors. Do not infer current hardware health from old tests.
3. **Align route ownership.** Reuse the model-authored goal/waypoint contract and
   the genuinely common continuation/replanning behavior, with EV3-specific
   observations and movement execution. First validate clear travel, then left
   and right box openings, then transient sensor loss, using actual Qwen and
   production execution through the simulated EV3 adapter. Do a physical EV3
   checkpoint early. Remove replaced host route-building code and obsolete tests
   only after checking remaining callers; keep BLAST regressions green.
4. **Validate the pair.** Confirm user-to-robot and robot-to-robot conversations,
   shared observations reaching both planners, aligned maps and independent task
   ownership. Include a model-chosen message from BLAST to EV3 and back, with
   explicit sender/recipient identity and evidence that the receiving agent uses
   the message. A shared display or two isolated chats is not acceptance. Test two
   production agents in the shared simulator before changing the single-mission
   restriction and trying both physical robots together. Keep Qwen concurrency
   within the configured two requests.

No camera implementation, self-generated long-running missions, unrelated UI
redesign or wholesale test pruning belongs in these checkpoints. Those remain
future capabilities; the observation and mission boundaries must not prevent them.

The older [navigation plan](NAVIGATION_SIMPLIFICATION_PLAN.md) is historical
context, not an instruction to restore retired behavior or reject every uncertain
reading. Retain useful goal, route and observation state through recoverable
failures; validate model decisions rather than prewritten simulator routes.
