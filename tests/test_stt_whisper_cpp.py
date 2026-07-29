import json
import socket
import unittest

from robot_agent.http_transport import (
    DirectHTTPResponse,
    DirectHTTPTimeoutError,
    DirectHTTPTransportError,
)
from robot_agent.stt_contract import (
    MAX_TRANSCRIPT_CHARACTERS,
    ProviderTranscription,
    TranscriptionRequest,
    validate_pcm16_wav,
)
from robot_agent.stt_provider import (
    STTProviderProtocolError,
    STTProviderTimeoutError,
    STTProviderUnavailableError,
)
from robot_agent.stt_whisper_cpp import (
    MAX_STT_PROVIDER_RESPONSE_BYTES,
    WhisperCppTranscriber,
)

from test_stt_contract import canonical_wav


def response(status=200, value=None, raw=None):
    if raw is None:
        raw = json.dumps(
            {"text": "Vinka två gånger."}
            if value is None
            else value,
        ).encode("utf-8")
    return DirectHTTPResponse(
        status_code=status,
        headers=(("Content-Type", "application/json"),),
        body=raw,
    )


def request(language="sv", request_id="voice-request-secret"):
    return TranscriptionRequest(
        request_id=request_id,
        language_hint=language,
        audio=validate_pcm16_wav(canonical_wav()),
    )


class RecordingTransport:
    def __init__(self, outcome=None):
        self.outcome = response() if outcome is None else outcome
        self.calls = []

    def __call__(self, *args):
        self.calls.append(args)
        if isinstance(self.outcome, BaseException):
            raise self.outcome
        return self.outcome


class WhisperCppConfigurationTests(unittest.TestCase):
    def test_accepts_loopback_only_urls_with_optional_opaque_path(self):
        opaque = "/stt-" + "a" * 48
        cases = (
            ("http://127.0.0.1:8178", "http://127.0.0.1:8178"),
            ("http://127.0.0.1:8178/", "http://127.0.0.1:8178"),
            ("http://localhost:8178", "http://localhost:8178"),
            ("http://[::1]:8178", "http://[::1]:8178"),
            ("http://127.2.3.4:8178", "http://127.2.3.4:8178"),
            (
                "http://127.0.0.1:8178" + opaque,
                "http://127.0.0.1:8178" + opaque,
            ),
        )
        for value, expected in cases:
            with self.subTest(value=value):
                transcriber = WhisperCppTranscriber(
                    base_url=value,
                )
                self.assertEqual(transcriber.base_url, expected)

    def test_private_path_requirement_rejects_stock_root_server(self):
        opaque = "/stt-" + "a" * 48
        for value in (
            "http://127.0.0.1:8178",
            "http://localhost:8178/",
        ):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    WhisperCppTranscriber(
                        base_url=value,
                        require_opaque_path=True,
                    )
        transcriber = WhisperCppTranscriber(
            base_url="http://127.0.0.1:8178" + opaque,
            require_opaque_path=True,
        )
        self.assertEqual(
            transcriber.base_url,
            "http://127.0.0.1:8178" + opaque,
        )

    def test_rejects_non_loopback_or_ambiguous_urls(self):
        cases = (
            None,
            "",
            "https://127.0.0.1:8178",
            "http://192.168.1.2:8178",
            "http://8.8.8.8:8178",
            "http://whisper.local:8178",
            "http://user@127.0.0.1:8178",
            "http://127.0.0.1:8178/inference",
            "http://127.0.0.1:8178/short",
            "http://127.0.0.1:8178/" + "a" * 129,
            "http://127.0.0.1:8178/" + "a" * 24 + "/nested",
            "http://127.0.0.1:8178/" + "a" * 24 + ".dot",
            "http://127.0.0.1:8178/%2Finference" + "a" * 24,
            "http://127.0.0.1:8178?x=1",
            "http://127.0.0.1:8178/#fragment",
            "http://127.0.0.1:99999",
        )
        for value in cases:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    WhisperCppTranscriber(base_url=value)

    def test_rejects_invalid_model_timeout_and_dependencies(self):
        invalid = (
            {"model_id": ""},
            {"model_id": "bad model"},
            {"model_id": "bad/model"},
            {"timeout_seconds": True},
            {"timeout_seconds": 0},
            {"timeout_seconds": 61},
            {"timeout_seconds": float("nan")},
            {"transport": None},
            {"boundary_factory": None},
            {"require_opaque_path": "yes"},
        )
        for kwargs in invalid:
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    WhisperCppTranscriber(**kwargs)


class WhisperCppTranscriptionTests(unittest.TestCase):
    def test_posts_bounded_multipart_request_without_private_metadata(self):
        opaque = "/stt-" + "a" * 48
        transport = RecordingTransport(
            response(
                value={
                    "text": "  Vinka två gånger.  ",
                    "ignored": {"tokens": [1, 2]},
                }
            )
        )
        transcriber = WhisperCppTranscriber(
            base_url="http://127.0.0.1:8178" + opaque,
            model_id="ggml-small",
            timeout_seconds=4.5,
            transport=transport,
            boundary_factory=lambda: "Boundary123",
        )
        transcription_request = request()

        result = transcriber.transcribe(transcription_request)

        self.assertEqual(
            result,
            ProviderTranscription(
                text="Vinka två gånger.",
                provider_id="whisper.cpp",
                model_id="ggml-small",
            ),
        )
        self.assertEqual(len(transport.calls), 1)
        (
            method,
            url,
            headers,
            body,
            timeout_seconds,
            max_response_bytes,
        ) = transport.calls[0]
        self.assertEqual(method, "POST")
        self.assertEqual(
            url,
            "http://127.0.0.1:8178" + opaque + "/inference",
        )
        self.assertEqual(headers["Accept"], "application/json")
        self.assertEqual(
            headers["Content-Type"],
            "multipart/form-data; boundary=Boundary123",
        )
        self.assertEqual(timeout_seconds, 4.5)
        self.assertEqual(
            max_response_bytes,
            MAX_STT_PROVIDER_RESPONSE_BYTES,
        )
        self.assertEqual(
            body.count(transcription_request.audio.wav_bytes),
            1,
        )
        self.assertNotIn(
            transcription_request.request_id.encode("ascii"),
            body,
        )
        self.assertNotIn(
            transcription_request.audio.sha256.encode("ascii"),
            body,
        )
        for expected in (
            b'name="file"; filename="utterance.wav"',
            b"Content-Type: audio/wav",
            b'name="response_format"\r\n\r\njson',
            b'name="language"\r\n\r\nsv',
            b'name="temperature"\r\n\r\n0',
            b'name="temperature_inc"\r\n\r\n0',
            b'name="beam_size"\r\n\r\n1',
            b'name="best_of"\r\n\r\n1',
            b'name="no_timestamps"\r\n\r\ntrue',
            b'name="translate"\r\n\r\nfalse',
        ):
            self.assertIn(expected, body)
        self.assertTrue(body.endswith(b"--Boundary123--\r\n"))

    def test_auto_language_is_sent_without_command_vocabulary_prompt(self):
        transport = RecordingTransport()
        transcriber = WhisperCppTranscriber(
            transport=transport,
            boundary_factory=lambda: "SafeBoundary",
        )

        transcriber.transcribe(request(language="auto"))

        body = transport.calls[0][3]
        self.assertIn(b'name="language"\r\n\r\nauto', body)
        self.assertNotIn(b'name="prompt"', body)
        self.assertNotIn(b"motor", body.lower())
        self.assertNotIn(b"robot", body.lower())

    def test_rejects_invalid_boundary_before_transport(self):
        for boundary in ("", "contains-hyphen", "contains space", "å"):
            with self.subTest(boundary=boundary):
                transport = RecordingTransport()
                transcriber = WhisperCppTranscriber(
                    transport=transport,
                    boundary_factory=lambda value=boundary: value,
                )
                with self.assertRaises(ValueError):
                    transcriber.transcribe(request())
                self.assertEqual(transport.calls, [])

    def test_requires_typed_request(self):
        transport = RecordingTransport()
        transcriber = WhisperCppTranscriber(transport=transport)

        with self.assertRaisesRegex(
            ValueError,
            "request is invalid",
        ):
            transcriber.transcribe(object())
        self.assertEqual(transport.calls, [])

    def test_maps_expected_transport_failures(self):
        cases = (
            (
                DirectHTTPTimeoutError("private"),
                STTProviderTimeoutError,
            ),
            (
                DirectHTTPTransportError("private"),
                STTProviderUnavailableError,
            ),
            (OSError("private"), STTProviderUnavailableError),
            (socket.timeout("private"), STTProviderUnavailableError),
        )
        for outcome, expected in cases:
            with self.subTest(outcome=type(outcome).__name__):
                transcriber = WhisperCppTranscriber(
                    transport=RecordingTransport(outcome),
                )
                with self.assertRaises(expected) as raised:
                    transcriber.transcribe(request())
                self.assertNotIn("private", str(raised.exception))

    def test_maps_http_status_by_failure_class(self):
        cases = (
            (response(status=400), STTProviderProtocolError),
            (response(status=429), STTProviderProtocolError),
            (response(status=503), STTProviderUnavailableError),
            (response(status=599), STTProviderUnavailableError),
            (response(status=302), STTProviderProtocolError),
            (object(), STTProviderProtocolError),
        )
        for outcome, expected in cases:
            with self.subTest(
                status=getattr(outcome, "status_code", None)
            ):
                transcriber = WhisperCppTranscriber(
                    transport=RecordingTransport(outcome),
                )
                with self.assertRaises(expected):
                    transcriber.transcribe(request())

    def test_rejects_malformed_or_excessive_provider_responses(self):
        cases = (
            b"",
            b"not-json",
            b"\xff",
            b"[]",
            b'{"text":"one","text":"two"}',
            b'{"text":NaN}',
            b'{"text":""}',
            b'{"text":"   "}',
            b'{"text":"\\ud800"}',
            json.dumps(
                {"text": "x" * (MAX_TRANSCRIPT_CHARACTERS + 1)}
            ).encode("utf-8"),
            b"x" * (MAX_STT_PROVIDER_RESPONSE_BYTES + 1),
        )
        for raw in cases:
            with self.subTest(size=len(raw), prefix=raw[:20]):
                transcriber = WhisperCppTranscriber(
                    transport=RecordingTransport(response(raw=raw)),
                )
                with self.assertRaises(STTProviderProtocolError):
                    transcriber.transcribe(request())


class WhisperCppProbeTests(unittest.TestCase):
    def test_probe_uses_short_loopback_health_request(self):
        opaque = "/stt-" + "b" * 48
        transport = RecordingTransport(
            response(value={"status": "ok"})
        )
        transcriber = WhisperCppTranscriber(
            base_url="http://127.0.0.1:9000" + opaque,
            model_id="ggml-base",
            timeout_seconds=9,
            transport=transport,
        )

        view = transcriber.probe()

        self.assertEqual(
            view,
            {
                "state": "online",
                "provider_id": "whisper.cpp",
                "model_id": "ggml-base",
            },
        )
        self.assertEqual(
            transport.calls,
            [
                (
                    "GET",
                    "http://127.0.0.1:9000" + opaque + "/health",
                    {"Accept": "application/json"},
                    None,
                    2.0,
                    4 * 1024,
                )
            ],
        )

    def test_probe_rejects_status_or_schema_mismatch(self):
        cases = (
            response(status=503, value={"status": "ok"}),
            response(status=400, value={"status": "ok"}),
            response(value={"status": "loading"}),
            response(value={"status": "ok", "extra": True}),
            response(raw=b'{"status":"ok","status":"ok"}'),
            object(),
        )
        for outcome in cases:
            with self.subTest(outcome=outcome):
                transcriber = WhisperCppTranscriber(
                    transport=RecordingTransport(outcome),
                )
                expected = (
                    STTProviderUnavailableError
                    if 500
                    <= getattr(outcome, "status_code", 0)
                    < 600
                    else STTProviderProtocolError
                )
                with self.assertRaises(expected):
                    transcriber.probe()

    def test_probe_maps_transport_failures(self):
        cases = (
            (
                DirectHTTPTimeoutError("private"),
                STTProviderTimeoutError,
            ),
            (
                DirectHTTPTransportError("private"),
                STTProviderUnavailableError,
            ),
            (OSError("private"), STTProviderUnavailableError),
            (socket.timeout("private"), STTProviderUnavailableError),
        )
        for outcome, expected in cases:
            with self.subTest(outcome=type(outcome).__name__):
                transcriber = WhisperCppTranscriber(
                    transport=RecordingTransport(outcome),
                )
                with self.assertRaises(expected):
                    transcriber.probe()


if __name__ == "__main__":
    unittest.main()
