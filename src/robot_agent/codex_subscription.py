"""Tool-free subscription inference, following BirdHackBot's provider boundary.

The Codex backend is a compatibility endpoint, not the public OpenAI API.
Codex owns login/refresh. Credentials never enter prompts or diagnostics.
No Codex model turn, agent, local listener, or paid-API fallback is created.
"""

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import select
import subprocess
import tempfile
import threading
import time

from . import lm_studio as lm
from .http_transport import direct_http_request, DirectHTTPTimeoutError, DirectHTTPTransportError
from .navigation_diagnostics import record_navigation_diagnostic

CODEX_PREFIX = "codex/"
ASTRA_MODEL_ID = "gpt-6.2-astra"
CODEX_MODEL = CODEX_PREFIX + ASTRA_MODEL_ID
RESPONSES_URL = "https://chatgpt.com/backend-api/codex/responses"
MAX_STREAM_BYTES = 8 * 1024 * 1024


class CodexSubscriptionError(lm.LMStudioTransportError):
    """A safely reportable provider failure; never contains upstream bodies."""

    def __init__(self, message, *, status_code=None, retry_after=None):
        super().__init__(message)
        self.status_code, self.retry_after = status_code, retry_after


@dataclass(frozen=True)
class _Credentials:
    access_token: str = field(repr=False)
    account_id: str = field(repr=False)


def _remaining(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise lm.LMStudioTimeoutError("Codex subscription request timed out")
    return remaining


class CodexAuth:
    def __init__(self, home=None):
        self.home = Path(home or os.environ.get("CODEX_HOME") or Path.home() / ".codex")
        self._lock = threading.Lock()

    def read(self):
        try:
            with (self.home / "auth.json").open("rb") as stream:
                raw = stream.read(128 * 1024 + 1)
            if len(raw) > 128 * 1024:
                raise ValueError()
            auth = json.loads(raw)
            tokens = auth["tokens"]
            if auth.get("auth_mode") != "chatgpt" or auth.get("OPENAI_API_KEY"):
                raise ValueError()
            values = (tokens["access_token"], tokens["account_id"])
            if any(not isinstance(v, str) or not v or any(ord(c) < 33 for c in v) for v in values):
                raise ValueError()
            return _Credentials(*values)
        except (OSError, ValueError, KeyError, TypeError):
            raise CodexSubscriptionError(
                'Codex ChatGPT file login required; run codex -c \'cli_auth_credentials_store="file"\' login',
                status_code=401) from None

    def refresh(self, rejected, deadline):
        if not self._lock.acquire(timeout=_remaining(deadline)):
            raise lm.LMStudioTimeoutError("Codex credential refresh timed out")
        try:
            current = self.read()
            if current != rejected:
                return current
            self._refresh_account(min(deadline, time.monotonic() + 30))
            return self.read()
        finally:
            self._lock.release()

    def _refresh_account(self, deadline):
        env = {k: v for k, v in os.environ.items()
               if k not in ("OPENAI_API_KEY", "CODEX_API_KEY", "CODEX_ACCESS_TOKEN", "CODEX_HOME")}
        env["CODEX_HOME"] = str(self.home)
        process = None
        try:
            with tempfile.TemporaryDirectory(prefix="robot-codex-auth-") as cwd:
                process = subprocess.Popen(
                    ["codex", "-c", 'cli_auth_credentials_store="file"', "-c", 'model_provider="openai"', "app-server"],
                    cwd=cwd, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL)
                buffer = b""
                def send(value):
                    process.stdin.write(json.dumps(value).encode() + b"\n")
                    process.stdin.flush()
                def receive(identifier):
                    nonlocal buffer
                    while True:
                        while b"\n" not in buffer:
                            if not select.select([process.stdout], [], [], _remaining(deadline))[0]:
                                raise TimeoutError()
                            chunk = os.read(process.stdout.fileno(), 65536)
                            if not chunk or len(buffer) + len(chunk) > 1024 * 1024:
                                raise ValueError()
                            buffer += chunk
                        line, buffer = buffer.split(b"\n", 1)
                        reply = json.loads(line)
                        if reply.get("id") == identifier:
                            if reply.get("error") or "result" not in reply:
                                raise ValueError()
                            return
                send({"id": 0, "method": "initialize", "params": {
                    "clientInfo": {"name": "robot-llm", "version": "1.0"}}})
                receive(0)
                send({"method": "initialized"})
                send({"id": 1, "method": "account/read", "params": {"refreshToken": True}})
                receive(1)
        except (OSError, ValueError, KeyError, TypeError, TimeoutError):
            raise CodexSubscriptionError("Codex subscription refresh failed; sign in again", status_code=401) from None
        finally:
            if process is not None:
                process.kill()
                process.communicate(timeout=2)


def _schema(value):
    """Our nullable oneOf branches are disjoint; Responses uses anyOf."""
    if isinstance(value, list):
        return [_schema(item) for item in value]
    if isinstance(value, dict):
        return {("anyOf" if key == "oneOf" else key): _schema(item) for key, item in value.items()}
    return value

def _request(payload):
    model = payload.get("model", "")
    if not model.startswith(CODEX_PREFIX) or not model[len(CODEX_PREFIX):]:
        raise lm.LMStudioInputError("Codex model selection is invalid")
    if payload.get("tools") or payload.get("stream"):
        raise lm.LMStudioInputError("Robot subscription inference is tool-free and non-streaming")
    instructions, messages = [], []
    for message in payload.get("messages", []):
        if not isinstance(message.get("content"), str):
            raise lm.LMStudioInputError("Robot subscription inference requires text messages")
        role = message.get("role")
        if role in ("system", "developer"):
            instructions.append(message["content"])
        elif role in ("user", "assistant"):
            messages.append({"role": role, "content": message["content"]})
        else:
            raise lm.LMStudioInputError("Unsupported subscription message role")
    if not messages:
        raise lm.LMStudioInputError("Subscription request has no input")
    effort = payload.get("reasoning_effort", "low")
    # Astra has no reasoning-off mode. Do not silently choose a different model.
    if effort in ("none", "minimal"):
        effort = "low"
    request = {"model": model[len(CODEX_PREFIX):], "instructions": "\n\n".join(instructions),
               "input": messages, "store": False, "stream": True, "tools": [], "tool_choice": "none",
               "reasoning": {"effort": effort, "summary": "auto"}}
    if payload.get("max_tokens"):
        request["max_output_tokens"] = payload["max_tokens"]
    output = payload.get("response_format", {})
    if output.get("type") == "json_schema":
        request["text"] = {"format": {"type": "json_schema", **_schema(output["json_schema"])}}
    return request

def _completion(raw, requested_model, maximum):
    if len(raw) > MAX_STREAM_BYTES:
        raise lm.LMStudioProtocolError("Codex subscription response too large")
    items, data = [], []
    try:
        for line in raw.decode("utf-8").splitlines():
            if line.startswith("data:"):
                data.append(line[5:].strip())
                continue
            if line or not data:
                continue
            payload, data = "\n".join(data), []
            if payload == "[DONE]":
                break
            event = json.loads(payload)
            kind = event.get("type")
            if kind == "response.output_item.done":
                items.append(event["item"])
            elif kind in ("error", "response.failed", "response.incomplete"):
                raise ValueError()
            elif kind == "response.completed":
                response = event["response"]
                if response.get("status") != "completed":
                    raise ValueError()
                if response.get("model") != requested_model[len(CODEX_PREFIX):]:
                    raise ValueError()
                text, summaries = [], []
                for item in response.get("output") or items:
                    if item.get("type") == "reasoning":
                        summaries.extend(part["text"] for part in item.get("summary", [])
                                         if part.get("type") == "summary_text")
                        continue
                    if item.get("type") != "message" or item.get("role") != "assistant":
                        raise ValueError()
                    for part in item["content"]:
                        if part.get("type") != "output_text":
                            raise ValueError()
                        text.append(part["text"])
                if not "".join(text).strip():
                    raise ValueError()
                usage = response.get("usage") or {}
                envelope = {"id": response.get("id"), "object": "chat.completion", "model": requested_model,
                    "resolved_model": response.get("model"), "provider": "codex_subscription",
                    "choices": [{"index": 0, "finish_reason": "stop", "message": {
                        "role": "assistant", "content": "".join(text),
                        "reasoning_content": "\n".join(summaries) or None}}],
                    "usage": {"prompt_tokens": usage.get("input_tokens"),
                              "completion_tokens": usage.get("output_tokens"), "total_tokens": usage.get("total_tokens")}}
                encoded = json.dumps(envelope).encode()
                if len(encoded) > maximum:
                    raise ValueError()
                return encoded
    except (UnicodeError, ValueError, KeyError, TypeError, AttributeError):
        raise lm.LMStudioProtocolError("Codex subscription response invalid, refused or incomplete") from None
    raise lm.LMStudioProtocolError("Codex subscription stream ended without a completed response")

class CodexSubscription:
    def __init__(self, auth=None, request=direct_http_request):
        self.auth, self.request = auth or CodexAuth(), request
        self._slots = threading.BoundedSemaphore(2)

    def complete(self, payload, timeout, maximum):
        deadline = time.monotonic() + timeout
        if not self._slots.acquire(timeout=_remaining(deadline)):
            raise lm.LMStudioTimeoutError("Codex subscription queue timed out")
        try:
            request, credentials = _request(payload), self.auth.read()
            refreshed = False
            while True:
                try:
                    response = self.request("POST", RESPONSES_URL, {
                        "Authorization": "Bearer " + credentials.access_token,
                        "ChatGPT-Account-Id": credentials.account_id,
                        "Content-Type": "application/json", "Accept": "text/event-stream",
                        "originator": "robot-llm", "User-Agent": "robot-llm-subscription/1.0"},
                        json.dumps(request).encode(), _remaining(deadline), MAX_STREAM_BYTES)
                except DirectHTTPTimeoutError:
                    raise lm.LMStudioTimeoutError("Codex subscription request timed out") from None
                except DirectHTTPTransportError:
                    raise CodexSubscriptionError("Codex subscription connection failed") from None
                if response.status_code == 401 and not refreshed:
                    credentials = self.auth.refresh(credentials, deadline)
                    refreshed = True
                    continue
                if response.status_code == 400 and "max_output_tokens" in request:
                    # Unlike a generic 400 retry, this only drops an explicitly
                    # rejected optional limit. Schema/model errors stay visible.
                    error = response.body[:8192].lower()
                    if b"max_output_tokens" in error and b"unsupported" in error:
                        del request["max_output_tokens"]
                        continue
                if response.status_code != 200:
                    status = response.status_code
                    messages = {401: "Codex subscription login expired; sign in again",
                                403: "Codex subscription model access denied",
                                429: "Codex subscription usage limit reached"}
                    retry = response.header_values("Retry-After")
                    raise CodexSubscriptionError(messages.get(status, "Codex subscription request rejected (HTTP %s)" % status),
                                                 status_code=status, retry_after=retry[0] if retry else None)
                result = _completion(response.body, payload["model"], maximum)
                record_navigation_diagnostic("provider_response", provider="codex_subscription",
                    requested_model=request["model"], resolved_model=json.loads(result)["resolved_model"],
                    reasoning_effort=request["reasoning"]["effort"],
                    output_limit_forwarded="max_output_tokens" in request,
                    usage=json.loads(result)["usage"])
                return result
        finally:
            self._slots.release()


_subscription = CodexSubscription()

def robot_model_transport(url, body, headers, timeout, maximum):
    """Keep the existing Qwen wire untouched; explicit codex/ selects cloud."""
    payload = json.loads(body)
    if not payload.get("model", "").startswith(CODEX_PREFIX):
        return lm._stdlib_post(url, body, headers, timeout, maximum)
    return _subscription.complete(payload, timeout, maximum)
