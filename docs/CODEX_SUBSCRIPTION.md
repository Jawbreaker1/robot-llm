# Robot model selection

Qwen and GPT-6 Astra use the same robot conversation classifier and navigation
planner contract. Only the model transport changes. Neither provider gets
shell access, hardware tools, repository contents or direct motor control.

## Select a model

In the web console's **Operate** view, select EV3 or BLAST, expand
**Technical details and run settings** in the Robot control panel,
choose **AI model — conversation and navigation**, and save:

- `qwen/qwen3.8-27b`: the existing LM Studio connection, unchanged.
- `codex/gpt-6-astra`: GPT-6 Astra through the host's Codex/ChatGPT login.

Settings are per robot and can be changed only while that robot is idle. The
new selection applies to subsequent inputs and missions, not a running mission.
Like the existing model setting, it lives in the console process; restarting
restores the command-line default. Existing custom model IDs remain displayed.
The general, read-only workbench chat is separate and still uses LM Studio.

For a Codex default on startup:

```sh
scripts/start_robot_console.sh --model codex/gpt-6-astra
```

Selecting Codex sends the supplied conversation, mission and robot observations
to OpenAI. Choosing Qwen keeps model inference at the configured LM Studio host.
The wrapper never silently switches models or falls back to a paid API key.

## Login and compatibility boundary

This follows the inference boundary in
[BirdHackBot's subscription bridge](https://github.com/Jawbreaker1/CodeHackBot/tree/60df93f778502e40a963865a37dff79f70f1054a/internal/subscription).
It is an in-process transport, not another local HTTP service: robot Chat
Completions messages become a tool-free Responses request, and a completed
response returns through the existing strict JSON decoder.

The endpoint is `https://chatgpt.com/backend-api/codex/responses`. It is a
**private Codex compatibility endpoint, not a stable public OpenAI API**.
Compatibility, account access and subscription limits can change. If it stops
working, the error is reported rather than hidden by a model/provider fallback.

Install the Codex CLI and use a ChatGPT file login on the same host/account as
the console. An existing working login needs no change. If login is missing:

```sh
codex -c 'cli_auth_credentials_store="file"' login
codex login status
```

The adapter reads only `auth.json` under `CODEX_HOME` (default `~/.codex`) and
requires ChatGPT authentication, not `OPENAI_API_KEY`. Credentials are excluded
from prompts and diagnostic output. A rejected token is refreshed once through
Codex's `account/read` with `refreshToken: true`; this starts no thread or agent
turn. Keychain-only credentials are not supported by this adapter.
See the [official app-server authentication protocol](https://developers.openai.com/codex/app-server/).

## Request behavior

- At most two subscription requests run concurrently. The caller's deadline
  includes queueing, authentication refresh and inference.
- Navigation uses **high** reasoning for Astra and **low** for Qwen, for both
  EV3 and BLAST and their simulators. The default follows the selected model,
  including changes made in the GUI. Explicit reasoning overrides remain
  available for comparison runs.
- Conversation is unchanged: an Astra conversation request for `none` is
  normalized to `low`; it does not select another model.
- Tools are disabled, `store` is false, and partial/refused/tool-call output is
  rejected. The backend's resolved model must match the requested model.
- Existing prompts and output validation are retained. Nullable, disjoint
  `oneOf` branches are translated to the Responses-compatible `anyOf` form.
- The requested output-token limit is forwarded when supported. If the backend
  explicitly rejects that parameter as unsupported, it is omitted on one retry;
  the backend then owns its token limit. The host still bounds response bytes
  and elapsed time. Do not assume the configured token cap was enforced: the
  `provider_response` diagnostic records `output_limit_forwarded`.
- Provider-supplied reasoning summaries are optional. We log these only when
  returned, plus the structured assessment; absence of a summary does not mean
  reasoning was off. No full hidden reasoning trace is promised.
- HTTP 401, 403 and 429 remain authentication/access/usage failures. Neither
  redirects nor arbitrary schema/model errors trigger a fallback.

The robot session continues to own recovery and mission state. The transport
does not add new navigation rules, impose a mission lifetime, or retry motors.

## Validate without moving hardware

```sh
PYTHONPATH=src:tests .venv/bin/python -m unittest test_codex_subscription
PYTHONPATH=src:. .venv/bin/python scripts/validate_ev3_navigation.py front \
  --model codex/gpt-6-astra --decisions 24 --time-budget 420 --planner-timeout 60
```

The simulator runs the production EV3 adapter/session/body with simulated
encoders and IR readings. The model does not receive the hidden obstacle
geometry. Its IR curve and motion remain approximations, not proof of physical
acceptance.

Initial integration checks (2026-09-28): the backend resolved `gpt-6-astra`;
three replays of the previously missed pre-passage scan all selected
`SCAN_FRONT_ARC`. One complete front-box simulation reached the goal in ten
decisions, with no physics contacts and 21 mm estimated goal distance (20 mm
in the hidden world). The model chose scans before both the outward passage
and the return leg. EV3 and BLAST read-only conversation probes both returned
valid replies without fallback. These are initial results, not a reliability
percentage or physical validation. No real robot moved in these checks.
The full quality suite passed 2,067 tests, including the provider boundary and
model-setting round trips. A text-only EV3 question also completed through the
actual web console with Astra selected; the robot remained idle.

The [follow-up comparison](ASTRA_NAVIGATION_VALIDATION_2026-09-28.md) repeats the
box and adds the eight-box and maze layouts, including failures and test limits.
