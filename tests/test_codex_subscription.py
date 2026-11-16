"""Provider-boundary tests: no credentials, network, tools or physical robots."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from robot_agent import codex_subscription as codex
from robot_agent import lm_studio as lm
from robot_agent.http_transport import DirectHTTPResponse, DirectHTTPTimeoutError
from robot_agent.lm_studio_controller_action import LMStudioControllerActionPlanner
from robot_agent.lm_studio_robot_input import LMStudioRobotInputModel
from test_lm_studio_controller_action import context


def event(value):
    return ("data: " + json.dumps(value) + "\n\n").encode()

def item(text):
    return {"type": "message", "role": "assistant", "content": [{"type": "output_text", "text": text}]}

def completed(text='{"ok":true}', *, output=True):
    return event({"type": "response.output_item.done", "item": item(text)}) + event({
        "type": "response.completed", "response": {"id": "response-test", "status": "completed",
            "model": "gpt-6-astra", "output": [item(text)] if output else [],
            "usage": {"input_tokens": 50, "output_tokens": 20, "total_tokens": 70}}})

def request():
    return {"model": codex.CODEX_MODEL, "messages": [{"role": "system", "content": "Instructions"},
        {"role": "user", "content": "Sensor facts"}], "reasoning_effort": "low", "max_tokens": 4096,
        "temperature": 1, "stream": False, "response_format": {"type": "json_schema", "json_schema": {
            "name": "test", "strict": True, "schema": {"type": "object", "properties": {
                "ok": {"oneOf": [{"type": "boolean"}, {"type": "null"}]}},
                "required": ["ok"], "additionalProperties": False}}}}


class CodexSubscriptionTests(unittest.TestCase):
    def provider(self, *responses):
        auth = Mock()
        auth.read.return_value = codex._Credentials("test-secret", "test-account")
        auth.refresh.return_value = codex._Credentials("refreshed-secret", "test-account")
        transport = Mock(side_effect=list(responses) or [DirectHTTPResponse(200, (), completed())])
        return codex.CodexSubscription(auth=auth, request=transport), auth, transport

    def test_converts_only_provider_format_and_preserves_prompt_and_model(self):
        original = request()
        before = deepcopy(original)
        bridge, _, transport = self.provider()
        result = json.loads(bridge.complete(original, 10, 10000))
        args = transport.call_args.args
        upstream = json.loads(args[3])
        self.assertEqual(args[1], codex.RESPONSES_URL)
        self.assertEqual(upstream["model"], "gpt-6-astra")
        self.assertEqual(upstream["instructions"], "Instructions")
        self.assertEqual(upstream["input"], [original["messages"][1]])
        self.assertEqual(upstream["tools"], [])
        self.assertEqual(upstream["tool_choice"], "none")
        self.assertFalse(upstream["store"])
        self.assertNotIn("temperature", upstream)
        self.assertEqual(upstream["max_output_tokens"], 4096)
        self.assertIn("anyOf", upstream["text"]["format"]["schema"]["properties"]["ok"])
        self.assertEqual(original, before)
        self.assertNotIn("test-secret", json.dumps(upstream))
        self.assertNotIn("test-secret", repr(bridge.auth.read()))
        self.assertEqual(result["model"], codex.CODEX_MODEL)
        self.assertEqual(result["resolved_model"], "gpt-6-astra")
        self.assertEqual(result["usage"]["total_tokens"], 70)

    def test_qwen_wire_is_untouched_and_never_reads_codex_auth(self):
        payload = request()
        payload["model"] = "qwen/qwen3.8-27b"
        body, headers = json.dumps(payload).encode(), {"Content-Type": "application/json"}
        with patch.object(lm, "_stdlib_post", return_value=b"qwen") as local, patch.object(codex, "_subscription") as cloud:
            self.assertEqual(codex.robot_model_transport("http://127.0.0.1:1234/v1/chat/completions", body, headers, 5, 1000), b"qwen")
            local.assert_called_once_with("http://127.0.0.1:1234/v1/chat/completions", body, headers, 5, 1000)
            cloud.complete.assert_not_called()

    def test_planner_and_conversation_share_the_provider_selection(self):
        text = json.dumps({"action": "DRIVE_FORWARD", "confidence_milli": 900, "assessment": "Test only",
                           "plan": ["DRIVE_FORWARD"], "utterance": None, "waypoint": None, "following_waypoints": []})
        bridge, _, _ = self.provider(DirectHTTPResponse(200, (), completed(text)))
        with patch.object(codex, "_subscription", bridge):
            result = LMStudioControllerActionPlanner(model=codex.CODEX_MODEL).decide(context())
        self.assertEqual(result.decision.action, "DRIVE_FORWARD")
        self.assertIs(LMStudioRobotInputModel(model=codex.CODEX_MODEL)._transport, codex.robot_model_transport)

    def test_terminal_may_omit_already_completed_items(self):
        result = json.loads(codex._completion(completed(output=False), codex.CODEX_MODEL, 10000))
        self.assertEqual(result["choices"][0]["message"]["content"], '{"ok":true}')

    def test_a_different_resolved_model_is_not_accepted_as_astra(self):
        raw = completed().replace(b'"model": "gpt-6-astra"', b'"model": "different-model"')
        with self.assertRaises(lm.LMStudioProtocolError):
            codex._completion(raw, codex.CODEX_MODEL, 10000)

    def test_refresh_only_reads_account_and_never_starts_an_agent_turn(self):
        auth = codex.CodexAuth("/tmp/test-codex-home")
        process = Mock()
        replies = [json.dumps({"id": identifier, "result": {}}).encode() + b"\n" for identifier in (0, 1)]
        with patch.object(codex.subprocess, "Popen", return_value=process) as popen, \
                patch.object(codex.select, "select", return_value=([process.stdout], [], [])), \
                patch.object(codex.os, "read", side_effect=replies):
            auth._refresh_account(time.monotonic() + 1)
        sent = [json.loads(call.args[0]) for call in process.stdin.write.call_args_list]
        self.assertEqual([message["method"] for message in sent], ["initialize", "initialized", "account/read"])
        self.assertTrue(sent[-1]["params"]["refreshToken"])
        self.assertEqual(popen.call_args.args[0][-1], "app-server")
        self.assertEqual(popen.call_args.kwargs["env"]["CODEX_HOME"], "/tmp/test-codex-home")
        self.assertNotIn("OPENAI_API_KEY", popen.call_args.kwargs["env"])
        process.kill.assert_called_once()
        process.communicate.assert_called_once()

    def test_summaries_are_optional_and_are_not_promoted_to_final_text(self):
        summary = {"type": "reasoning", "summary": [{"type": "summary_text", "text": "Reported summary"}]}
        raw = event({"type": "response.output_item.done", "item": summary}) + completed(output=False)
        result = json.loads(codex._completion(raw, codex.CODEX_MODEL, 10000))
        self.assertEqual(result["choices"][0]["message"]["reasoning_content"], "Reported summary")

    def test_rejects_partial_refused_tool_and_oversized_output(self):
        bad = [event({"type": "response.output_item.done", "item": item("partial")}),
               event({"type": "response.incomplete"}), b"data: invalid\n\n"]
        for output in ({"type": "function_call", "name": "drive"},
                       {"type": "message", "role": "assistant", "content": [{"type": "refusal", "refusal": "No"}]}):
            bad.append(event({"type": "response.completed", "response": {"status": "completed", "output": [output]}}))
        for raw in bad:
            with self.subTest(raw=raw), self.assertRaises(lm.LMStudioProtocolError):
                codex._completion(raw, codex.CODEX_MODEL, 10000)
        with self.assertRaises(lm.LMStudioProtocolError):
            codex._completion(completed(), codex.CODEX_MODEL, 10)

    def test_auth_refresh_once_only_and_no_secret_in_error(self):
        bridge, auth, transport = self.provider(DirectHTTPResponse(401, (), b"test-secret"),
                                               DirectHTTPResponse(401, (), b"refreshed-secret"))
        with self.assertRaises(codex.CodexSubscriptionError) as failure:
            bridge.complete(request(), 5, 10000)
        self.assertNotIn("secret", str(failure.exception))
        self.assertEqual(auth.refresh.call_count, 1)
        self.assertEqual(transport.call_count, 2)
        self.assertEqual(transport.call_args.args[2]["Authorization"], "Bearer refreshed-secret")

    def test_rate_access_and_redirect_errors_do_not_fall_back(self):
        for status in (302, 400, 403, 429, 500):
            bridge, auth, transport = self.provider(DirectHTTPResponse(status, (("Retry-After", "20"),), b"test-secret"))
            with self.subTest(status=status), self.assertRaises(codex.CodexSubscriptionError) as failure:
                bridge.complete(request(), 5, 10000)
            self.assertEqual(failure.exception.status_code, status)
            self.assertNotIn("test-secret", str(failure.exception))
            self.assertEqual(transport.call_count, 1)
            auth.refresh.assert_not_called()

    def test_only_explicit_output_limit_rejection_is_retried(self):
        bridge, _, transport = self.provider(DirectHTTPResponse(400, (), b'Unsupported parameter: max_output_tokens'),
                                             DirectHTTPResponse(200, (), completed()))
        bridge.complete(request(), 5, 10000)
        self.assertEqual(transport.call_count, 2)
        self.assertNotIn("max_output_tokens", json.loads(transport.call_args.args[3]))

    def test_deadlines_and_concurrency_are_bounded(self):
        bridge, _, _ = self.provider()
        bridge.request.side_effect = DirectHTTPTimeoutError("do not propagate upstream data")
        with self.assertRaises(lm.LMStudioTimeoutError):
            bridge.complete(request(), 1, 10000)
        bridge._slots.acquire()
        bridge._slots.acquire()
        try:
            with self.assertRaises(lm.LMStudioTimeoutError):
                bridge.complete(request(), 0.001, 10000)
        finally:
            bridge._slots.release()
            bridge._slots.release()

    def test_astra_conversation_uses_low_not_unsupported_none(self):
        payload = request()
        payload["reasoning_effort"] = "none"
        self.assertEqual(codex._request(payload)["reasoning"]["effort"], "low")
        for field, value in (("tools", [{"type": "function"}]), ("stream", True)):
            with self.assertRaises(lm.LMStudioInputError):
                codex._request({**payload, field: value})

    def test_auth_accepts_only_chatgpt_file_and_skips_duplicate_refresh(self):
        with tempfile.TemporaryDirectory() as home:
            path = Path(home) / "auth.json"
            auth = codex.CodexAuth(home)
            with self.assertRaises(codex.CodexSubscriptionError):
                auth.read()
            valid = {"auth_mode": "chatgpt", "tokens": {"access_token": "test-secret", "account_id": "account"}}
            for change in ({"auth_mode": "apikey"}, {"OPENAI_API_KEY": "test-key"}, {"tokens": {}}):
                path.write_text(json.dumps({**valid, **change}))
                with self.assertRaises(codex.CodexSubscriptionError):
                    auth.read()
            path.write_text(json.dumps(valid))
            with patch.object(auth, "_refresh_account") as refresh:
                current = auth.refresh(codex._Credentials("outdated", "account"), time.monotonic() + 1)
            self.assertEqual(current.access_token, "test-secret")
            refresh.assert_not_called()
