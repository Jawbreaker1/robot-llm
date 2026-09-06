"""Small loopback Piper service for the two physical robot voices."""

from __future__ import annotations

import argparse
import io
import json
import math
from pathlib import Path
import signal
import threading
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
import wave


MODEL_ID = "piper-sv"
LOOPBACK_HOST = "127.0.0.1"
MAX_REQUEST_BYTES = 16 * 1024
MAX_TEXT_CHARACTERS = 600
MAX_WAV_BYTES = 4 * 1024 * 1024


@dataclass(frozen=True)
class VoiceVariant:
    model_name: str
    rate_multiplier: float
    noise_scale: float
    noise_w_scale: float


VOICE_VARIANTS = {
    "lisa-bright": VoiceVariant(
        "sv_SE-lisa-medium",
        1.04,
        0.72,
        0.86,
    ),
    "nst-deep": VoiceVariant(
        "sv_SE-nst-medium",
        0.90,
        0.56,
        0.68,
    ),
}


@dataclass(frozen=True)
class SynthesisRequest:
    text: str
    voice: str
    speed: float


class SynthesisFailure(Exception):
    def __init__(
        self,
        message: str,
        status: int = HTTPStatus.BAD_REQUEST,
        code: str = "invalid_request",
    ):
        super().__init__(message)
        self.status = int(status)
        self.code = code


def parse_synthesis_request(payload: Any) -> SynthesisRequest:
    if not isinstance(payload, dict):
        raise SynthesisFailure("Request body must be a JSON object")
    if payload.get("model") != MODEL_ID:
        raise SynthesisFailure(
            "Unknown TTS model",
            HTTPStatus.NOT_FOUND,
            "model_not_found",
        )
    if payload.get("response_format", "wav") != "wav":
        raise SynthesisFailure(
            "Only WAV output is supported",
            code="unsupported_format",
        )
    text = payload.get("input")
    if not isinstance(text, str) or not text.strip():
        raise SynthesisFailure("input must contain text", code="empty_input")
    text = " ".join(text.split())
    if len(text) > MAX_TEXT_CHARACTERS:
        raise SynthesisFailure(
            "input is too long",
            HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
            "input_too_large",
        )
    voice = payload.get("voice")
    if voice not in VOICE_VARIANTS:
        raise SynthesisFailure("Unknown robot voice", code="voice_not_found")
    speed = payload.get("speed", 1.0)
    if (
        isinstance(speed, bool)
        or not isinstance(speed, (int, float))
        or not math.isfinite(float(speed))
        or not 0.5 <= float(speed) <= 2.0
    ):
        raise SynthesisFailure("speed must be between 0.5 and 2.0")
    return SynthesisRequest(text, voice, float(speed))


class PiperSynthesizer:
    def __init__(self, model_directory: Path):
        try:
            from piper.config import SynthesisConfig
            from piper.voice import PiperVoice
        except ImportError as error:
            raise RuntimeError(
                "Piper is unavailable; run scripts/setup_piper_service.sh"
            ) from error

        self._SynthesisConfig = SynthesisConfig
        self._voices = {}
        self._locks = {}
        for variant in VOICE_VARIANTS.values():
            model_name = variant.model_name
            if model_name in self._voices:
                continue
            model_path = model_directory / (model_name + ".onnx")
            config_path = model_directory / (model_name + ".onnx.json")
            if not model_path.is_file() or not config_path.is_file():
                raise RuntimeError(
                    "Piper model is missing: {}".format(model_name)
                )
            self._voices[model_name] = PiperVoice.load(
                str(model_path),
                config_path=str(config_path),
            )
            self._locks[model_name] = threading.Lock()

    @property
    def loaded_voices(self):
        return tuple(sorted(VOICE_VARIANTS))

    def synthesize(self, request: SynthesisRequest) -> bytes:
        variant = VOICE_VARIANTS[request.voice]
        effective_rate = max(
            0.65,
            min(1.45, request.speed * variant.rate_multiplier),
        )
        config = self._SynthesisConfig(
            length_scale=max(0.69, min(1.54, 1.0 / effective_rate)),
            noise_scale=variant.noise_scale,
            noise_w_scale=variant.noise_w_scale,
        )
        audio = bytearray()
        sample_rate = sample_width = channels = None
        with self._locks[variant.model_name]:
            chunks = self._voices[variant.model_name].synthesize(
                request.text,
                syn_config=config,
            )
            for chunk in chunks:
                sample_rate = chunk.sample_rate
                sample_width = chunk.sample_width
                channels = chunk.sample_channels
                audio.extend(chunk.audio_int16_bytes)
                if len(audio) > MAX_WAV_BYTES:
                    raise SynthesisFailure(
                        "Synthesized audio is too large",
                        HTTPStatus.BAD_GATEWAY,
                        "audio_too_large",
                    )
        if not audio or not sample_rate or not sample_width or not channels:
            raise SynthesisFailure(
                "Piper returned no audio",
                HTTPStatus.BAD_GATEWAY,
                "empty_audio",
            )
        output = io.BytesIO()
        with wave.open(output, "wb") as wav_file:
            wav_file.setnchannels(channels)
            wav_file.setsampwidth(sample_width)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(audio)
        return output.getvalue()


class PiperHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, synthesizer):
        super().__init__(address, PiperRequestHandler)
        self.synthesizer = synthesizer
        self.synthesis_lock = threading.Lock()


class PiperRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "RobotLLMPiper/1"

    @property
    def piper_server(self) -> PiperHTTPServer:
        return self.server

    def log_message(self, _format, *_args):
        return None

    def _send_json(self, status: int, payload) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_error(self, error: SynthesisFailure) -> None:
        self._send_json(
            error.status,
            {"error": {"code": error.code, "message": str(error)}},
        )

    def do_GET(self) -> None:
        if self.path == "/health":
            self._send_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "model": MODEL_ID,
                    "voices": self.piper_server.synthesizer.loaded_voices,
                },
            )
            return
        if self.path == "/v1/models":
            self._send_json(
                HTTPStatus.OK,
                {
                    "object": "list",
                    "data": [{"id": MODEL_ID, "object": "model"}],
                },
            )
            return
        self._send_error(
            SynthesisFailure("Not found", HTTPStatus.NOT_FOUND, "not_found")
        )

    def do_POST(self) -> None:
        if self.path != "/v1/audio/speech":
            self._send_error(
                SynthesisFailure(
                    "Not found",
                    HTTPStatus.NOT_FOUND,
                    "not_found",
                )
            )
            return
        try:
            content_length = int(self.headers.get("Content-Length", ""))
            if not 1 <= content_length <= MAX_REQUEST_BYTES:
                raise SynthesisFailure("Invalid request size")
            payload = json.loads(self.rfile.read(content_length))
            request = parse_synthesis_request(payload)
            with self.piper_server.synthesis_lock:
                body = self.piper_server.synthesizer.synthesize(request)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
            self._send_error(
                SynthesisFailure("Invalid JSON request", code="invalid_json")
            )
            return
        except SynthesisFailure as error:
            self._send_error(error)
            return
        except Exception:
            self._send_error(
                SynthesisFailure(
                    "Piper synthesis failed",
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    "synthesis_failed",
                )
            )
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "audio/wav")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def create_server(port: int, synthesizer) -> PiperHTTPServer:
    if not 0 <= port <= 65_535:
        raise ValueError("Piper port is invalid")
    return PiperHTTPServer((LOOPBACK_HOST, port), synthesizer)


def main() -> None:
    parser = argparse.ArgumentParser(description="Robot LLM Piper sidecar")
    parser.add_argument("--port", type=int, default=8179)
    parser.add_argument("--model-dir", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.port <= 65_535:
        parser.error("--port must be between 1 and 65535")
    server = create_server(
        args.port,
        PiperSynthesizer(args.model_dir.resolve()),
    )
    stop = lambda *_args: threading.Thread(
        target=server.shutdown,
        daemon=True,
    ).start()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(
        "Piper ready on http://{}:{} with {}".format(
            LOOPBACK_HOST,
            server.server_port,
            ", ".join(server.synthesizer.loaded_voices),
        ),
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
