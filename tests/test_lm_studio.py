import json
import socket
import unittest
from unittest.mock import ANY, patch

import robot_agent.lm_studio as lm_studio
from robot_agent.commentary import ProximityObservation
from robot_agent.http_transport import (
    DirectHTTPResponse,
    DirectHTTPTimeoutError,
)
from robot_agent.lm_studio import (
    CHAT_PATH,
    DEFAULT_MODEL,
    MAX_OUTPUT_TOKENS,
    MAX_RESPONSE_BYTES,
    REQUEST_TIMEOUT_SECONDS,
    LMStudioConfigurationError,
    LMStudioHTTPError,
    LMStudioInputError,
    LMStudioProtocolError,
    LMStudioResponseTooLargeError,
    LMStudioTimeoutError,
    NativeLMStudioClient,
)


def response_body(
    text="Vad fan står där framme?",
    reasoning_tokens=0,
    output=None,
    **extra,
):
    value = {
        "model_instance_id": DEFAULT_MODEL,
        "output": (
            [{"type": "message", "content": text}]
            if output is None
            else output
        ),
        "stats": {
            "input_tokens": 42,
            "total_output_tokens": 8,
            "reasoning_output_tokens": reasoning_tokens,
        },
    }
    value.update(extra)
    return json.dumps(value).encode("utf-8")


def observation(zone="near_return", filtered_percent=28):
    return ProximityObservation(
        observed_at_ms=1_234,
        samples=(filtered_percent,),
        filtered_percent=filtered_percent,
        zone=zone,
    )


class RecordingTransport:
    def __init__(self, result=None, error=None):
        self.result = result if result is not None else response_body()
        self.error = error
        self.calls = []

    def __call__(self, url, body, headers, timeout_seconds, max_response_bytes):
        self.calls.append(
            (url, body, headers, timeout_seconds, max_response_bytes)
        )
        if self.error is not None:
            raise self.error
        return self.result


class FixedClock:
    def __init__(self, *values):
        self._values = iter(values)

    def __call__(self):
        return next(self._values)


class NativeLMStudioClientTests(unittest.TestCase):
    def test_canonical_default_model_is_non_qat_artifact(self):
        self.assertEqual(DEFAULT_MODEL, "google/gemma-4-26b-a4b")

    def test_comment_uses_fixed_safe_native_request(self):
        transport = RecordingTransport()
        client = NativeLMStudioClient(
            transport=transport,
            clock=FixedClock(10.0, 10.125),
        )

        candidate = client.comment(observation())

        self.assertEqual(candidate.text, "Vad fan står där framme?")
        self.assertEqual(candidate.latency_ms, 125)
        self.assertEqual(candidate.model_instance_id, DEFAULT_MODEL)
        self.assertEqual(client.model, DEFAULT_MODEL)
        self.assertEqual(len(transport.calls), 1)

        url, raw_body, headers, timeout, body_limit = transport.calls[0]
        self.assertEqual(url, "http://127.0.0.1:1234" + CHAT_PATH)
        self.assertEqual(timeout, REQUEST_TIMEOUT_SECONDS)
        self.assertEqual(body_limit, MAX_RESPONSE_BYTES)
        self.assertEqual(headers["Accept"], "application/json")
        self.assertEqual(
            headers["Content-Type"],
            "application/json; charset=utf-8",
        )

        payload = json.loads(raw_body.decode("utf-8"))
        self.assertEqual(
            payload,
            {
                "model": DEFAULT_MODEL,
                "input": (
                    "IR-zon=near_return; relativt_reflektionsvärde=28. "
                    "Skriv endast repliken."
                ),
                "system_prompt": ANY,
                "reasoning": "off",
                "store": False,
                "stream": False,
                "integrations": [],
                "temperature": 0,
                "max_output_tokens": MAX_OUTPUT_TOKENS,
            },
        )
        self.assertIn("Hitta aldrig på", payload["system_prompt"])

    def test_model_is_fixed_per_client_and_cannot_be_overridden_per_call(self):
        transport = RecordingTransport()
        client = NativeLMStudioClient(
            model="local/fixed-model",
            transport=transport,
            clock=FixedClock(0.0, 0.0),
        )
        client.comment(observation())

        payload = json.loads(transport.calls[0][1])
        self.assertEqual(payload["model"], "local/fixed-model")
        with self.assertRaises(TypeError):
            client.comment(observation(), model="other/model")

    def test_only_literal_loopback_base_urls_are_accepted(self):
        accepted = [
            "http://localhost:1234",
            "https://LOCALHOST:8443/",
            "http://127.0.0.1",
            "http://127.42.9.1:1234/",
            "http://[::1]:1234",
        ]
        for base_url in accepted:
            with self.subTest(base_url=base_url):
                NativeLMStudioClient(base_url=base_url)

        rejected = [
            "http://192.168.1.10:1234",
            "http://lmstudio.local:1234",
            "http://localhost.example:1234",
            "http://user:secret@localhost:1234",
            "http://localhost:1234/api",
            "http://localhost:1234?token=secret",
            "file:///tmp/server",
            "http://[::1",
            "http://localhost:99999",
        ]
        for base_url in rejected:
            with self.subTest(base_url=base_url):
                with self.assertRaises(LMStudioConfigurationError) as raised:
                    NativeLMStudioClient(base_url=base_url)
                self.assertNotIn("secret", str(raised.exception))

    def test_invalid_model_and_observations_are_rejected_before_transport(self):
        for model in ["", " model", "model\nsecret"]:
            with self.subTest(model=model):
                with self.assertRaises(LMStudioConfigurationError):
                    NativeLMStudioClient(model=model)

        transport = RecordingTransport()
        client = NativeLMStudioClient(transport=transport)
        bad_observations = [
            object(),
            observation(zone="unknown"),
            observation(filtered_percent=True),
            observation(filtered_percent=-1),
            observation(filtered_percent=101),
        ]
        for value in bad_observations:
            with self.subTest(value=value):
                with self.assertRaises(LMStudioInputError):
                    client.comment(value)
        self.assertEqual(transport.calls, [])

    def test_timeout_is_typed_and_has_no_request_data_in_message(self):
        transport = RecordingTransport(error=socket.timeout("super-secret"))
        client = NativeLMStudioClient(
            transport=transport,
            clock=FixedClock(0.0),
        )

        with self.assertRaises(LMStudioTimeoutError) as raised:
            client.comment(observation())
        self.assertNotIn("super-secret", str(raised.exception))
        self.assertNotIn("near_return", str(raised.exception))

    def test_body_limit_is_enforced_even_for_injected_transport(self):
        transport = RecordingTransport(result=b"x" * (MAX_RESPONSE_BYTES + 1))
        client = NativeLMStudioClient(
            transport=transport,
            clock=FixedClock(0.0, 0.1),
        )

        with self.assertRaises(LMStudioResponseTooLargeError):
            client.comment(observation())

    def test_exactly_one_message_is_required(self):
        invalid_outputs = [
            [],
            [{"type": "reasoning", "content": "secret chain"}],
            [
                {"type": "message", "content": "one"},
                {"type": "message", "content": "two"},
            ],
            [{"type": "tool_call", "tool": "drive", "arguments": {}}],
            [{"type": "message", "content": ""}],
        ]
        for output in invalid_outputs:
            with self.subTest(output=output):
                client = NativeLMStudioClient(
                    transport=RecordingTransport(
                        result=response_body(output=output)
                    ),
                    clock=FixedClock(0.0, 0.1),
                )
                with self.assertRaises(LMStudioProtocolError):
                    client.comment(observation())

    def test_reasoning_must_be_explicitly_zero_integer_tokens(self):
        for value in [1, True, 0.0, None]:
            with self.subTest(value=value):
                client = NativeLMStudioClient(
                    transport=RecordingTransport(
                        result=response_body(reasoning_tokens=value)
                    ),
                    clock=FixedClock(0.0, 0.1),
                )
                with self.assertRaises(LMStudioProtocolError):
                    client.comment(observation())

    def test_stored_or_malformed_response_is_rejected_without_echoing_body(self):
        secret = "dont-leak-this-secret"
        bodies = [
            response_body(response_id="resp_should_not_exist"),
            b'{"broken":"' + secret.encode("ascii"),
            json.dumps([secret]).encode("utf-8"),
            json.dumps(
                {
                    "model_instance_id": "",
                    "output": [{"type": "message", "content": secret}],
                    "stats": {"reasoning_output_tokens": 0},
                }
            ).encode("utf-8"),
            json.dumps(
                {
                    "model_instance_id": DEFAULT_MODEL,
                    "output": [{"type": "message", "content": secret}],
                }
            ).encode("utf-8"),
        ]
        for body in bodies:
            with self.subTest(body=body):
                client = NativeLMStudioClient(
                    transport=RecordingTransport(result=body),
                    clock=FixedClock(0.0, 0.1),
                )
                with self.assertRaises(LMStudioProtocolError) as raised:
                    client.comment(observation())
                self.assertNotIn(secret, str(raised.exception))

    def test_http_error_status_is_safe_and_typed(self):
        error = LMStudioHTTPError(503)
        self.assertEqual(error.status_code, 503)
        self.assertEqual(str(error), "LM Studio returned HTTP status 503")

    def test_default_transport_uses_direct_request_and_rejects_redirect(self):
        with patch.object(
            lm_studio,
            "direct_http_request",
            return_value=DirectHTTPResponse(
                status_code=302,
                headers=(("Location", "http://external.invalid/"),),
                body=b"",
            ),
        ) as direct:
            with self.assertRaises(LMStudioHTTPError) as raised:
                lm_studio._stdlib_post(
                    "http://127.0.0.1:1234/chat",
                    b"{}",
                    {"Content-Type": "application/json"},
                    1.0,
                    1_024,
                )

        self.assertEqual(raised.exception.status_code, 302)
        direct.assert_called_once_with(
            "POST",
            "http://127.0.0.1:1234/chat",
            {"Content-Type": "application/json"},
            b"{}",
            1.0,
            1_024,
        )

    def test_default_transport_maps_absolute_timeout(self):
        with patch.object(
            lm_studio,
            "direct_http_request",
            side_effect=DirectHTTPTimeoutError("private"),
        ):
            with self.assertRaises(LMStudioTimeoutError) as raised:
                lm_studio._stdlib_post(
                    "http://127.0.0.1:1234/chat",
                    b"secret",
                    {},
                    0.1,
                    1_024,
                )
        self.assertNotIn("private", str(raised.exception))
        self.assertNotIn("secret", str(raised.exception))

    def test_elapsed_deadline_is_enforced_if_transport_returns_late(self):
        client = NativeLMStudioClient(
            transport=RecordingTransport(),
            clock=FixedClock(10.0, 10.0 + REQUEST_TIMEOUT_SECONDS + 0.001),
        )

        with self.assertRaises(LMStudioTimeoutError) as raised:
            client.comment(observation())
        self.assertEqual(str(raised.exception), "LM Studio request timed out")

    def test_clock_regression_cannot_make_negative_latency(self):
        client = NativeLMStudioClient(
            transport=RecordingTransport(),
            clock=FixedClock(9.0, 8.0),
        )
        self.assertEqual(client.comment(observation()).latency_ms, 0)


if __name__ == "__main__":
    unittest.main()
