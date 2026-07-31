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
