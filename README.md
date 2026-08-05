# Robot LLM Lab 🤖

[![Quality](https://github.com/Jawbreaker1/robot-llm/actions/workflows/ci.yml/badge.svg)](https://github.com/Jawbreaker1/robot-llm/actions/workflows/ci.yml)
![LLM: local](https://img.shields.io/badge/LLM-local%20via%20LM%20Studio-6f42c1)
![EV3 hardware: live](https://img.shields.io/badge/EV3%20hardware-live%20over%20Wi--Fi%2FSSH-2ea44f)
![Robot Inventor hardware: live](https://img.shields.io/badge/Robot%20Inventor%20hardware-live%20over%20BLE-2ea44f)
![Physical navigation: experimental](https://img.shields.io/badge/physical%20navigation-experimental-d29922)

**A real LEGO robot controlled by a local agentic AI that can plan, observe,
speak, and adapt as it goes.**

Robot LLM Lab connects a local AI agent to physical LEGO robots. EV3RSTORM
currently carries the complete navigation loop: give it a goal and the agent
plans, acts, reads sensors and motor feedback, and adapts when reality
disagrees. Robot Inventor 51515 has now completed its first physical bring-up
with Pybricks and direct local deployment over Bluetooth.

The language model chooses semantic intent and expression. The host application
validates and dispatches typed actions, while the active controller worker
remains the sole owner of its motor ports.

<p align="center">
  <img src="src/robot_agent/dashboard_web/robot-llm-mascot.png" alt="Robot LLM Lab's mildly grumpy modular mascot waving" width="280">
</p>

<p align="center"><em>Mildly grumpy by design. Personality shapes the language, not motor authority.</em></p>

## What makes it agentic

Many LLM–robot demos are effectively one-shot remote controls:

```text
prompt → command → execution
```

Robot LLM Lab keeps the goal alive and closes the loop:

```text
goal → plan → action → observe → verify → adapt
```

This means:

- a goal can survive across several actions and observations;
- verified sensor and encoder results inform the next decision;
- the agent can investigate, revise its plan, or stop when reality contradicts
  its assumptions; and
- speech, mapping, and UI updates can progress alongside navigation while
  physical actions remain serialized.

Natural-language intent is handled by the model through strict schemas. The
host does not translate instructions with regular expressions, keyword lists,
or language-specific command menus. The model never receives raw motor access.

## What works today

| Status | Capabilities |
|---|---|
| Working on physical EV3 | ev3dev, Wi-Fi/SSH control, bounded movement and turning, stop, IR, touch, motor encoders, host-generated robot speech, and the goal → plan → act → observe → replan loop |
| Working on physical Robot Inventor | Pybricks firmware on BLAST-01, local BLE deployment, complete port inventory, IMU and battery telemetry, plus verified display, speaker and motor control |
| Working in the application | English/Swedish web dashboard, direct robot conversation and status questions, local push-to-talk STT, technical events, current plan, active route and waypoint, simulator mapping, per-run physical navigation memory, and read-only multi-controller telemetry |
| Experimental | Operator-confirmed physical obstacle passage, active IR scanning, qualitative hazard mapping, model-authorized typed detour routes, body-aware path checks, and recovery from imperfect motor movement |
| Planned | Repeatable autonomous obstacle navigation, Robot Inventor motion control and autonomy in the application, continuous hands-free voice interaction, cameras, vision, sound localization, BOOST, and multi-robot coordination |

EV3 obstacle navigation has succeeded in an operator-confirmed physical trial;
repeatability and broader acceptance runs remain experimental.

The current EV3 map is intentionally qualitative. IR reflection can support
obstacle hypotheses, but it is not vision, object recognition, or precise
metric SLAM. The forward-facing color/light sensor is installed but is not yet
used by the production navigation loop.

EV3 remains the only motion-enabled application worker. BLAST-01 now has its
own registered robot identity and an optional persistent BLE observer in the
dashboard. That first integration is deliberately read-only; BLAST motion
controls and autonomous plans are not exposed through the application yet.

## Architecture

```mermaid
flowchart TD
    U["Goal, question, or voice transcript"] --> H["Host agent"]
    S["Sensors, encoders, and map memory"] --> H
    H --> L["Local LLM<br/>plan and expression"]
    L --> V["Typed proposal"]
    V --> P["Host validation and policy"]
    P --> A["One semantic action"]
    A --> W["Controller worker<br/>sole motor owner"]
    W --> R["Physical LEGO robot"]
    R --> S
    L -. speech .-> T["Host speech worker"]
    T -. audio .-> R
    S --> D["Dashboard and map"]
```

The host owns goals, state, navigation memory, model calls, and validation.
Each controller worker exposes a small set of fixed robot operations and
processes one request at a time. It contains no planner, personality, or
independent goal. The EV3 implementation is application-integrated today; the
Robot Inventor worker is the next controller implementation. Once the model has
authorized a target and detour side, a deterministic route executive may
serialize several freshly checked pulses before asking the model again. New
geometry, ambiguous progress, a veto, or a failed movement returns control to
the agent immediately.

Speech, map publication, and UI delivery already run alongside the physical
loop. Future vision, audio, validation, and planning workers will publish
time-stamped observations or proposals, but they will not control motors
directly. Physical execution remains serialized through the worker that owns
the relevant controller.

More detail is available in [the architecture document](docs/ARCHITECTURE.md).

## Dashboard

![Current English Workbench with the physical EV3 control service ready and Robot selected](docs/images/dashboard-live-workbench-current-en.jpg)

The local web application has two conversation targets:

- **Robot** can answer, report what its current sensors and plan say, or turn a
  text or speech instruction into a physical goal.
- **Workbench** provides ordinary dialogue, configuration, evidence, and
  development tools.

The dashboard shows the current live state rather than offering old physical
runs to resume. It exposes the current goal, plan, action, speech state,
active detour route and waypoint progress, technical events, and a read-only
map. Navigation memory is retained during an EV3 episode and reset before the
next physical run.

The same Map view can display a completed simulator run or qualitative physical
odometry and obstacle hypotheses:

![Map UI populated by the deterministic simulator demo](docs/images/dashboard-simulator-map-current-en.jpg)

See [the dashboard guide](docs/DASHBOARD.md) for STT, settings, persistence,
and UI contracts.

## Quick start without a robot

```sh
git clone https://github.com/Jawbreaker1/robot-llm.git
cd robot-llm
PYTHONPATH=src python3 -m robot_agent.dashboard_cli \
  --simulation-map-demo
```

Open the private live URL printed by the server and choose **Map**. This uses a
deterministic simulator and requires no EV3 or loaded LLM. It demonstrates the
application and mapping pipeline, not physical calibration.

To run the Workbench with a model exposed by LM Studio:

```sh
ROBOT_LLM_STT_URL='' scripts/start_lab_console.sh \
  --model 'EXACT-MODEL-ID-FROM-LM-STUDIO'
```

The empty STT URL starts without speech recognition. Follow the
[dashboard guide](docs/DASHBOARD.md) to add the local whisper.cpp service.

## Running with a physical EV3

The physical path requires an EV3 running ev3dev, a network connection to the
brick, Python 3.9+ on the host, the deployed EV3 worker, and a model served by
LM Studio.

Before allowing movement, verify the robot configuration and complete the
motion-free deployment checks in:

- [EV3 Wi-Fi setup](docs/EV3_WIFI.md)
- [EV3 runtime deployment](docs/EV3_RUNTIME_DEPLOYMENT.md)

Then start the explicit EV3 profile:

```sh
ROBOT_LLM_STT_URL='' scripts/start_ev3rstorm_console.sh \
  --robot-target 'robot@<EV3-host>' \
  --model 'EXACT-MODEL-ID-FROM-LM-STUDIO'
```

Omit the STT override when the local speech-recognition service is configured.
Robot movement begins only after a goal is submitted and explicitly started in
the dashboard.

## Robot Inventor 51515 bring-up

BLAST-01 runs Pybricks and accepts programs directly from the local repository
over Bluetooth. Live diagnostics identify four angular motors, a color sensor,
an ultrasonic sensor, the built-in six-axis IMU and battery telemetry. Display,
speaker and bounded motor actuation are physically verified. A persistent BLE
session exposes observations, stop, drive, turn, claw and body pulses. The
dashboard can keep that session open and show battery, distance, color, IMU and
motor telemetry without granting the web UI motor access.

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-pybricks.txt
./scripts/run_blast_smoke.sh
.venv/bin/python -m pybricksdev run ble --name BLAST-01 \
  hub_programs/blast_01/inventory.py
```

To add the read-only BLAST observer to the dashboard:

```sh
ROBOT_LLM_PYTHON=.venv/bin/python ROBOT_LLM_STT_URL='' \
  scripts/start_lab_console.sh --blast-hub-name BLAST-01
```

This optional toolchain requires Python 3.10 or newer. Disconnect Pybricks Code
from the hub before running the local command because only one BLE client can
own the connection.

## Tests

```sh
sh ./scripts/quality_check.sh
```

The hardware-free suite covers contracts, agent loops, process transports,
mapping, dashboard behavior, simulated EV3 sysfs, and failure cleanup. It does
not replace physical calibration or live stop tests.

## Roadmap

- Make physical obstacle navigation repeatable and complete its live
  calibration and acceptance runs.
- Add color sensing, continuous voice interaction, wireless cameras,
  microphones, vision, and sound-source reasoning.
- Complete the Robot Inventor hub supervisor and host worker, then add BOOST
  and coordinate several LEGO controllers.
- Expand the asynchronous architecture with parallel perception, validation,
  and forward planning while preserving serialized motor ownership.

Long-term vision: hear a dog bark, locate the sound, look for the source,
recognize the dog, turn toward it, and answer, “woof right back at you.”

## Documentation

| Document | Contents |
|---|---|
| [Architecture](docs/ARCHITECTURE.md) | Agent runtime, authority, parallelism, memory, and multi-controller design |
| [Dashboard](docs/DASHBOARD.md) | Live UI, STT, map, settings, and persistence |
| [EV3 Wi-Fi](docs/EV3_WIFI.md) | Network onboarding and recovery |
| [EV3 runtime deployment](docs/EV3_RUNTIME_DEPLOYMENT.md) | Worker deployment, preflight, transport, speech, and physical checks |
| [Experiment plan](docs/EXPERIMENT_PLAN.md) | Test protocols, observations, limitations, and evidence |
| [Navigation benchmark](docs/LM_STUDIO_NAVIGATION_BENCHMARK.md) | Structured-planner benchmark method and results |
| [`docs/data`](docs/data) | Machine-readable experiment artifacts |

## Repository layout

```text
config/                 robot topology and fixed action profiles
docs/                   architecture, setup, experiments, evidence, and images
ev3/                    EV3 HAL, bounded workers, and diagnostic tools
hub_programs/           controller-side programs for non-EV3 LEGO hubs
src/robot_agent/        host agent, navigation, mapping, speech, and dashboard
tests/                  hardware-free scenarios, contracts, and failure tests
```

Robot LLM Lab deliberately avoids a large robotics framework. New abstractions
are introduced when experiments show that they are needed.

No open-source license has been selected yet.

LEGO, MINDSTORMS, EV3, Robot Inventor, and BOOST are trademarks of the LEGO
Group. This independent experimental project is not affiliated with or
endorsed by the LEGO Group.
