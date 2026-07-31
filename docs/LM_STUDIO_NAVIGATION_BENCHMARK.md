# Physical-navigation LM Studio benchmark

`robot_agent.lm_studio_navigation_benchmark_cli` measures the exact
schema-constrained planner used by the physical navigation host runtime. It
does not import the EV3 transport, contact a robot, or expose a motor API.

The benchmark deliberately has no default endpoint or model. Both values are
required so an operator must identify the intended LM Studio machine and the
exact loaded model before a request can be sent.

## Identity and endpoint gate

The CLI accepts only:

- a numeric loopback address;
- an RFC1918 IPv4 address (`10/8`, `172.16/12`, or `192.168/16`); or
- an IPv6 loopback or unique-local address.

Hostnames, public addresses, link-local addresses, credentials, paths,
queries, and fragments are rejected. The first request is always
`GET /v1/models`. The requested model ID must exactly match one `data[].id`
entry before the first warmup or measured inference POST is allowed.

## Fixed suite

Each run uses the production navigation planner's system prompt, strict JSON
schema, and `NavigationDecision` validation for three small physical contexts:

1. clear English forward-progress evidence (`ADVANCE` expected);
2. a Swedish unscanned provisional obstacle (`SCAN_FRONT_ARC` expected); and
3. a completed English directional mission (`FINISH` expected).

One warmup is run for each case and excluded from all metrics. Measured calls
never retry, so first-pass schema validity is observable rather than hidden by
repair attempts.

## Invocation

After confirming the IP address of the computer that is actually running LM
Studio and copying the exact ID shown by its `/v1/models` response:

```sh
PYTHONPATH=src python3 -m robot_agent.lm_studio_navigation_benchmark_cli \
  --base-url 'http://192.168.1.50:1234' \
  --model 'EXACT-MODEL-ID-FROM-LM-STUDIO' \
  --repetitions 5 \
  --parallelism 1 \
  --inference-api lmstudio-v0 \
  --pretty
```

`--repetitions` accepts 1–10. `--parallelism` accepts 1–4. Use parallelism 1
for comparable single-request latency and an explicit higher value only when
measuring concurrent planner throughput. The per-request timeout can be set
between 0.5 and 60 seconds with `--timeout-seconds`.

The benchmark defaults to LM Studio's enhanced
`/api/v0/chat/completions` route. It sends the same OpenAI-compatible payload
as production but also retains LM Studio's top-level server timing statistics.
Use `--inference-api openai-v1` as a compatibility fallback; server decode and
time-to-first-token fields will be null when that route does not return
top-level `stats`. This benchmark option does not change the production
planner, whose default remains `/v1/chat/completions`.

`--parallelism` is the maximum number of in-flight client requests. It does
not by itself prove that LM Studio decoded that many requests simultaneously.
The report therefore separates server-reported decode speed from client
end-to-end time and includes per-sample start/finish offsets plus the complete
measured-phase makespan.

Run different quantizations separately with their own exact model IDs. The
tool reports measurements; it does not assume that one quantization is faster
or that two IDs refer to equivalent weights.

## Recorded QAT run

`EXP-GEMMA4-QAT-NAV-001` used the operator-confirmed local `lmlink` endpoint
and the enhanced v0 route for `google/gemma-4-26b-a4b-qat` on 2026-07-31. All
45 measured responses passed the strict schema, selected the expected semantic
action, reported the exact model ID, and included server statistics.

| Client concurrency | Median / p95 E2E latency | Median / p95 server decode | Median / p95 TTFT | Measured aggregate E2E output |
|---:|---:|---:|---:|---:|
| 1 | `3.220 / 3.513 s` | `99.559 / 105.849 tok/s` | `0.107 / 0.204 s` | `90.678 tok/s` |
| 2 | `7.054 / 8.118 s` | `50.828 / 76.796 tok/s` | `0.137 / 0.652 s` | `85.232 tok/s` |
| 4 | `11.920 / 16.839 s` | `27.872 / 42.092 tok/s` | `0.939 / 2.333 s` | `90.408 tok/s` |

For this exact structured production workload, more concurrent requests
increased per-request latency and reduced per-stream server decode speed while
aggregate end-to-end output stayed around `85–91 tok/s`. There is therefore no
measured planner-throughput advantage to four concurrent requests. A separate
unconstrained 192-token diagnostic reached roughly `161–167 tok/s` per stream
at four requests, so this penalty is workload-specific rather than evidence
that `lmlink` universally serializes generation. That diagnostic is not part
of the formal navigation suite.

The evidence record, including makespan and completion-token totals, is stored
in [`docs/data/EXP-GEMMA4-QAT-NAV-001.json`](data/EXP-GEMMA4-QAT-NAV-001.json).

## Report

The JSON report includes:

- sanitized endpoint host, port, and scheme;
- requested model and every model identity reported by inference responses;
- the exact-model-list gate result;
- per-case first-pass schema validity and expected semantic-action agreement;
- failure codes without retrying malformed output;
- median and nearest-rank p95 wall latency;
- median and p95 completion tokens per client end-to-end second when
  `completion_tokens` usage is present;
- LM Studio's server-reported decode tokens per second and time to first token,
  reported separately when top-level `stats` are available;
- measured-phase makespan, total completion tokens, and aggregate completion
  tokens per end-to-end second;
- sanitized per-sample timing, model, token, validity, and action fields; and
- explicit `ev3_contact: false` and `motor_api_exposed: false` scope fields.

Warmup failures are reported separately and never included in measured
latency, agreement, or token-rate distributions.
