# EV3 runtime deployment preflight

Before starting either foreground daemon, compare its fixed runtime manifest
with the local checkout:

```sh
PYTHONPATH=src python3 -m robot_agent.ev3_runtime_preflight_cli \
  --ssh-target 'robot@<EV3-host>' \
  --profile peripheral \
  --pretty
```

Use `--profile supervisor` for the complete supervisor manifest. The
supervisor profile contains the shared peripheral dependencies only once.

The command uses strict, key-only SSH and reads fixed files below
`/home/robot/robot-llm`. It does not import runtime modules, start a daemon,
write to the EV3, or enable motion. A missing, changed, symlinked,
non-regular, unreadable, or oversized file makes the preflight fail closed.
Deploy the complete selected manifest and repeat the preflight rather than
copying only the reported file.

The parsed response is size-capped, and SSH uses strict host-key checking and
a hard command deadline. The underlying `subprocess.run` pipe capture is not
yet stream-capped before allocation; replacing it with bounded incremental
capture is a documented defense-in-depth follow-up for a compromised but
already trusted endpoint.
