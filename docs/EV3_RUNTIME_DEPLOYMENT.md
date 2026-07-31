# EV3 runtime deployment preflight

Before starting a foreground EV3 process, compare its fixed runtime manifest
with the local checkout:

```sh
PYTHONPATH=src python3 -m robot_agent.ev3_runtime_preflight_cli \
  --ssh-target 'robot@<EV3-host>' \
  --profile peripheral \
  --pretty
```

Use `--profile supervisor` for the complete supervisor manifest. The
supervisor profile contains the shared peripheral dependencies only once.
Use `--profile navigation-worker` for the policy-free autonomous navigation
worker and its exact motor, sensor, safety, and configuration dependencies.
The same fixed profile also includes `ev3/robot_cli.py`, because the physical
application may run the bounded `speak-stdin --voice sv|en` companion while
the navigation worker is active. Speech text is supplied on stdin; the
deployment profile does not add an operator-selected remote command or widen
the navigation worker protocol. This profile verifies deployment only; it
does not start the worker, synthesize speech, or enable motion.

The command uses strict, key-only SSH and reads fixed files below
`/home/robot/robot-llm`. It does not import runtime modules, start a daemon,
write to the EV3, or enable motion. A missing, changed, symlinked,
non-regular, unreadable, or oversized file makes the preflight fail closed.
Deploy the complete selected manifest and repeat the preflight rather than
copying only the reported file.

Every Python file in the selected manifest, including the navigation
worker's TTS companion, is parsed against the Python 3.5 grammar before SSH
and compiled by the fixed read-only program on the EV3. An incompatible
local file prevents the SSH request; an incompatible remote file fails the
deployment comparison.

The parsed response is size-capped, and SSH uses strict host-key checking and
a hard command deadline. The underlying `subprocess.run` pipe capture is not
yet stream-capped before allocation; replacing it with bounded incremental
capture is a documented defense-in-depth follow-up for a compromised but
already trusted endpoint.

## Motion-free navigation-worker protocol preflight

After the `navigation-worker` deployment comparison passes, validate the live
foreground worker lifecycle without requesting movement:

```sh
PYTHONPATH=src python3 -m robot_agent.ev3_navigation_preflight_cli \
  --ssh-target 'robot@<EV3-host>' \
  --pretty
```

This host command uses the production `EV3NavigationSSHTransport` contract and
the fixed sequence `start → describe → observe → stop → shutdown → close`.
It never exposes a planner or sends a movement operation. The observation must
show zero consumed movement budget, no active motor state, and no latched
motion fault. Stop and shutdown must provide correlated state versions and
verified motor-owner cleanup.

Cold startup on the EV3 has a separate 30-second deadline for the first
`describe`; subsequent `observe`, `stop`, and `shutdown` requests retain their
8-second deadline. Override these independently with
`--startup-timeout-seconds` and `--request-timeout-seconds` when collecting
explicit timing evidence.

Movement replies use the strict `ev3-agent-worker-response/v2` contract. A
drive pulse reports the actual encoder deltas even when the semantic action
does not complete. If the EV3 performs a bounded undertravel correction, the
reply preserves every primary, catch-up, or retry segment in execution order;
the host integrates those segments individually instead of pretending their
aggregate was one straight movement. A partial motor start must likewise
carry the started side, timestamps, pre/cleanup positions, and a verified stop
before its movement can be accepted. Wrong-direction movement remains a hard
fault.

This is best-effort encoder odometry, not wheel-to-floor ground truth. It
cannot detect slip, and the recovery budget cannot make a permanently dead
motor or cable move. Live thresholds remain provisional until the assembled
EV3RSTORM has repeated the straight-line fault scenario over Wi-Fi.

The JSON evidence is deliberately sanitized: it contains no SSH target,
remote path, raw stderr, or underlying exception text. Any protocol,
transport, lifecycle, or stationary-state failure aborts the SSH channel and
then attempts a final close before returning a non-zero status.

## Host-generated robot speech

The EV3RSTORM profile synthesizes Swedish speech on the host through
`http://127.0.0.1:8179/v1/audio/speech`. Its explicit default is model
`piper-sv`, voice `nst-deep`, WAV output, and speed `1.0`. Text is limited to
160 characters. Both host and EV3 accept at most 4 MiB and 20 seconds of mono
16-bit PCM WAV at 8–48 kHz.

On the first utterance, the speech thread lazily starts one separate SSH
process running the fixed Python 3.5-compatible
`ev3/audio_playback_worker_cli.py`. The worker emits a ready frame, then loops
over bounded ASCII headers followed by exact-length raw WAV payloads. It stays
alive for the episode, so later utterances do not pay Python, import,
configuration, or SSH startup again. It uses the same exclusive audio lock as
onboard speech and invokes only `aplay --quiet`; it creates no EV3 temporary
file. Episode cancellation terminates this audio SSH process without closing
the navigation worker's SSH process.

There is one bounded cancellation limitation in this first streaming
protocol: if the complete WAV has already reached the remote process before
the SSH channel is cancelled, ev3dev may finish that already-buffered clip.
The validated duration ceiling limits this case to 20 seconds. A future framed
audio protocol can add an explicit remote cancellation acknowledgement.

Only Swedish is mapped by default. English requires its own tested voice/model
mapping; the Swedish NST model is deliberately not an implicit English
fallback. Speech failure remains observable but cannot block navigation or
motor cleanup.

Cancellation during an in-flight motion currently closes the SSH channel so
the EV3 worker can stop locally. If that prevents a correlated encoder receipt
from reaching the host, the persisted localization is invalidated rather than
pretending that no movement occurred. A future protocol revision should add a
graceful in-band cancel that returns verified stop and encoder evidence first,
while retaining channel abort as the bounded fallback.
