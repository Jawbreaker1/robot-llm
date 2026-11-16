# GPT-6 Astra navigation comparison — 2026-09-28

## Result

The model swap improves the simple-box result and the choice to inspect a
passage, but does **not** establish reliable navigation through dense layouts.
No physical robot was driven. No navigation code, prompts, geometry, motor
calibration or sensor curve was changed during this comparison.

| Scenario | Historical Qwen reference | New Astra run |
| --- | --- | --- |
| Front box | 14 decisions, 740 mm estimated distance remaining, 4 body contacts; test time limit | Goal reached in 10 decisions, 20 mm remaining, 0 body contacts |
| Eight boxes | 15 decisions, 1,649 mm remaining, 3 contacts; test time limit | 24 decisions, 1,264 mm remaining, 3 contacts; decision cap |
| Maze | 13 decisions, 1,779 mm remaining, 4 contacts; worker-expiry cleanup error | 24 decisions, 1,202 mm remaining, 0 contacts; decision cap |

The earlier Astra front-box run also reached the true simulator goal without
contacts. Thus the simple box has two clean Astra observations, not a general
reliability guarantee. The two complex runs were **stopped by the test harness
at 24 completed actions**, not abandoned by the model. In particular, the maze
result is incomplete, not evidence that Astra cannot finish it with more steps.
The eight-box contacts already disqualify that run from clean success.

The comparison uses the historical Qwen runs documented in
[the EV3 integration audit](EV3_INTEGRATION_PLAN.md#edge-inspection-validation-not-accepted-2026-09-28).
These are not fresh paired statistical trials: the historical runs used the
documented initial/compact prompt revisions and different elapsed-time outcomes.
The first structured robot context was compared in full and was identical for
each matching scene. Core source hashes were unchanged throughout the Astra
runs. Do not infer that model capability is the sole remaining cause, or that
every remaining issue must be a deterministic execution bug.

## What the decisions show

- **Simple box:** Astra chose a lateral inspection waypoint, turned to face the
  passage, explicitly requested `SCAN_FRONT_ARC`, passed the obstacle, inspected
  the return leg, and completed at the goal. The simulator's hidden world also
  confirmed arrival. It did not receive a ready-made route.
- **Eight boxes:** it passed the first obstacle, detected a further obstruction
  through IR, retreated and scanned. The first body contact occurred while
  rotating during a scan at estimated pose `(856,-404)`; the turn stopped partway
  at about 118 degrees. A later recovery turn and a waypoint leg also contacted
  obstacles. It attempted recovery rather than declaring completion or giving
  up. No transport exception terminated this run.
- **Maze:** it passed the first baffle and moved inward between baffles without
  contact. Many `ADVANCE` decisions each authorize only one approximately 52 mm
  step. At decision 24 it was still progressing and about 1.2 m from the goal.
  The test cap, not a navigation fault, stopped it.

Astra chose 2, 5 and 5 front-arc scans respectively. Average model-call latency
was 11.1, 12.2 and 12.0 seconds. Total model-call time was 111.4, 293.6 and 287.9
seconds; this is not physical travel time.

## Remaining evidence/geometry limitation

All model-returned obstacle bounds remained null in **all three** new runs.
The model receives the measured asymmetric body (front 110, rear 90, left 100,
right 130 mm, 10 mm margin; 181 mm turn-sweep radius), observation origins and
ray bearings. It does not receive the hidden obstacle rectangles or simulated
metric ranges. The IR curve generates unitless readings; the prompt correctly
forbids treating their numeric values as millimetres.

Consequently, requesting a scan is better behavior but is not itself proof of
body or turning clearance. The narrow scan rotates the whole body, so it can
also encounter an obstacle. This was observed in the eight-box run, not merely
hypothesized. Unknown edges alone do not prove why every decision failed, but
they mean we still do not have a usable, model-owned metric obstacle outline
to display or use as a reliable clearance estimate.

The next investigation should separate whether observations can support a
useful edge/passability estimate, how that estimate is retained, and whether
the body has room to perform the inspection. Do not hide this with fabricated
box coordinates, a forced route, or another model-specific detour rule.
For the maze, a larger **test** action allowance is needed before judging
end-to-end completion; that does not require changing production mission limits.

## Reproduction and artifacts

All runs selected exact `codex/gpt-6-astra`, low reasoning, the current production
planner/context, white IR response, 4,096 requested output tokens, 60-second
planner timeout and 24 completed actions maximum. The subscription backend may
reject its optional token-limit field; the adapter's documented behavior then
uses the backend limit. Reasoning settings are supported by the
[official Astra model documentation](https://developers.openai.com/api/docs/models/gpt-6-astra).

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src:. .venv/bin/python \
  scripts/validate_ev3_navigation.py front --model codex/gpt-6-astra \
  --decisions 24 --output-tokens 4096 --planner-timeout 60 --time-budget 420
# Repeat with many-boxes / 540 seconds and maze / 480 seconds.
```

Local artifacts: `/private/tmp/robot-astra-ab.Zvifux/` contains `front.jsonl`,
`many-boxes.jsonl`, `maze.jsonl`, the corresponding empty `.err` files,
`results.json` (compact decision/result evidence), and `source-sha256.txt`.
All three runs shut down their simulated workers. There were no final runtime
exceptions. The simulator's ideal ray, approximate IR response, sensor-origin
and encoder assumptions remain limitations; these results do not certify
physical navigation.
