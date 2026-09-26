"""Bounded host Piper synthesis streamed to an EV3 audio-only process."""

from __future__ import annotations

from dataclasses import dataclass
import io
import json
import math
import subprocess
import threading
import time
from typing import Callable, Optional, Tuple
from urllib.parse import urlsplit
import wave


DEFAULT_PIPER_BASE_URL = "http://127.0.0.1:8179/v1"
DEFAULT_PIPER_MODEL = "piper-sv"
DEFAULT_SWEDISH_VOICE = "nst-deep"
REMOTE_AUDIO_PLAYER = "/home/robot/robot-llm/ev3/audio_playback_cli.py"
MAX_TEXT_CHARACTERS = 160
MAX_WAV_BYTES = 4 * 1024 * 1024
MAX_PROCESS_OUTPUT_BYTES = 8192
MAX_AUDIO_DURATION_MS = 20_000
POLL_SECONDS = 0.05
STOP_SECONDS = 0.15
READER_JOIN_SECONDS = 0.2
READ_CHUNK_BYTES = 16 * 1024
EV3_PLAYBACK_STARTUP_ALLOWANCE_SECONDS = 10.0


class HostSpeechError(RuntimeError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class WAVMetadata:
    byte_count: int
    channels: int
    sample_width_bytes: int
    sample_rate_hz: int
    frames: int
    duration_ms: int


def validate_pcm16_mono_wav(raw: object) -> WAVMetadata:
    """Validate the exact bounded PCM surface accepted by the EV3 player."""

    if not isinstance(raw, bytes) or not 44 <= len(raw) <= MAX_WAV_BYTES:
        raise HostSpeechError("invalid_tts_wav", "TTS WAV size is invalid")
    try:
        with wave.open(io.BytesIO(raw), "rb") as source:
            channels = source.getnchannels()
            width = source.getsampwidth()
            rate = source.getframerate()
            frames = source.getnframes()
            compression = source.getcomptype()
            payload = source.readframes(frames + 1)
    except (EOFError, wave.Error):
        raise HostSpeechError("invalid_tts_wav", "TTS output is not WAV") from None
    if (
        channels != 1
        or width != 2
        or compression != "NONE"
        or not 8_000 <= rate <= 48_000
        or frames <= 0
        or len(payload) != frames * channels * width
    ):
        raise HostSpeechError(
            "unsupported_tts_wav",
            "TTS WAV must be mono 16-bit PCM at 8-48 kHz",
        )
    duration_ms = (frames * 1000 + rate - 1) // rate
    if duration_ms > MAX_AUDIO_DURATION_MS:
        raise HostSpeechError("tts_audio_too_long", "TTS WAV is too long")
    return WAVMetadata(
        byte_count=len(raw),
        channels=channels,
        sample_width_bytes=width,
        sample_rate_hz=rate,
        frames=frames,
        duration_ms=duration_ms,
    )


def _safe_target(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or value.startswith("-")
        or len(value) > 255
        or any(
            not (character.isalnum() or character in "._-@:%+")
            for character in value
        )
    ):
        raise ValueError("EV3 speech SSH target is invalid")
    return value


def _safe_remote_path(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value.startswith("/")
        or len(value) > 512
        or any(
            not (character.isalnum() or character in "/._-")
            for character in value
        )
    ):
        raise ValueError("EV3 audio player path is invalid")
    return value


def _stop_process(process, timeout_seconds: float = STOP_SECONDS) -> None:
    try:
        if process.poll() is not None:
            return
    except (OSError, ProcessLookupError):
        return
    try:
        process.terminate()
        process.wait(timeout=timeout_seconds)
        return
    except (OSError, ProcessLookupError):
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        process.kill()
        process.wait(timeout=timeout_seconds)
    except (OSError, ProcessLookupError, subprocess.TimeoutExpired):
        pass


def _read_bounded(stream, limit, destination, overflow, failed) -> None:
    try:
        while True:
            remaining = limit + 1 - len(destination)
            if remaining <= 0:
                overflow.set()
                return
            chunk = stream.read(min(READ_CHUNK_BYTES, remaining))
            if not chunk:
                return
            if not isinstance(chunk, bytes):
                failed.set()
                return
            destination.extend(chunk)
            if len(destination) > limit:
                overflow.set()
                return
    except BaseException:
        failed.set()


def _capture_bounded_cancellable(
    process,
    payload: bytes,
    cancel_event: threading.Event,
    timeout_seconds: float,
    monotonic: Callable[[], float],
):
    """Capture a process without allowing pipes to allocate past their caps."""

    stdout = bytearray()
    stderr = bytearray()
    overflow = threading.Event()
    failed = threading.Event()
    readers = (
        threading.Thread(
            target=_read_bounded,
            args=(process.stdout, MAX_WAV_BYTES, stdout, overflow, failed),
            name="piper-stdout",
            daemon=True,
        ),
        threading.Thread(
            target=_read_bounded,
            args=(
                process.stderr,
                MAX_PROCESS_OUTPUT_BYTES,
                stderr,
                overflow,
                failed,
            ),
            name="piper-stderr",
            daemon=True,
        ),
    )
    for reader in readers:
        reader.start()
    try:
        process.stdin.write(payload)
        process.stdin.close()
    except (BrokenPipeError, OSError, ValueError):
        _stop_process(process)
        raise HostSpeechError(
            "tts_provider_failed",
            "Piper request body could not be written",
        ) from None

    deadline = monotonic() + timeout_seconds
    while True:
        if cancel_event.is_set():
            _stop_process(process)
            for reader in readers:
                reader.join(READER_JOIN_SECONDS)
            return None
        if overflow.is_set():
            _stop_process(process)
            for reader in readers:
                reader.join(READER_JOIN_SECONDS)
            raise HostSpeechError(
                "tts_response_too_large",
                "Piper response exceeded its bounded capture",
            )
        if failed.is_set():
            _stop_process(process)
            raise HostSpeechError(
                "invalid_tts_response",
                "Piper response stream failed",
            )
        try:
            completed = process.poll() is not None
        except (OSError, ProcessLookupError):
            completed = True
        if completed:
            break
        if monotonic() >= deadline:
            _stop_process(process)
            raise HostSpeechError("speech_timeout", "Speech process timed out")
        time.sleep(POLL_SECONDS)

    for reader in readers:
        reader.join(READER_JOIN_SECONDS)
    if any(reader.is_alive() for reader in readers) or failed.is_set():
        _stop_process(process)
        raise HostSpeechError(
            "invalid_tts_response",
            "Piper response stream did not close",
        )
    if overflow.is_set():
        raise HostSpeechError(
            "tts_response_too_large",
            "Piper response exceeded its bounded capture",
        )
    return bytes(stdout), bytes(stderr)


def _communicate_cancellable(
    process,
    payload: bytes,
    cancel_event: threading.Event,
    timeout_seconds: float,
    monotonic: Callable[[], float],
):
    deadline = monotonic() + timeout_seconds
    pending = payload
    while True:
        if cancel_event.is_set():
            _stop_process(process)
            return None
        remaining = deadline - monotonic()
        if remaining <= 0:
            _stop_process(process)
            raise HostSpeechError("speech_timeout", "Speech process timed out")
        try:
            return process.communicate(
                input=pending,
                timeout=min(POLL_SECONDS, remaining),
            )
        except subprocess.TimeoutExpired:
            # Popen retains its internal input buffer across communicate retries.
            pending = None


@dataclass(frozen=True)
class PiperSpeechProfile:
    base_url: str = DEFAULT_PIPER_BASE_URL
    model: str = DEFAULT_PIPER_MODEL
    voices: Tuple[Tuple[str, str], ...] = (
        ("sv", DEFAULT_SWEDISH_VOICE),
    )
    speed: float = 1.0
    connect_timeout_seconds: float = 2.0
    request_timeout_seconds: float = 15.0

    def __post_init__(self) -> None:
        if not isinstance(self.base_url, str):
            raise ValueError("Piper URL must be loopback HTTP")
        parsed = urlsplit(self.base_url)
        if (
            parsed.scheme != "http"
            or parsed.hostname != "127.0.0.1"
            or parsed.username is not None
            or parsed.password is not None
            or parsed.query
            or parsed.fragment
            or not parsed.path.startswith("/")
        ):
            raise ValueError("Piper URL must be loopback HTTP")
        try:
            port = parsed.port
        except ValueError:
            raise ValueError("Piper port is invalid") from None
        if port is None or not 1 <= port <= 65535:
            raise ValueError("Piper port is invalid")
        for name, value in (("model", self.model),):
            if (
                not isinstance(value, str)
                or not value
                or len(value) > 128
                or not value.isascii()
                or any(not (c.isalnum() or c in "._-") for c in value)
            ):
                raise ValueError("Piper {} is invalid".format(name))
        if not isinstance(self.voices, tuple) or not self.voices:
            raise ValueError("Piper voice mapping is invalid")
        seen_locales = set()
        for item in self.voices:
            if not isinstance(item, tuple) or len(item) != 2:
                raise ValueError("Piper voice mapping is invalid")
            locale, voice = item
            if (
                locale not in ("sv", "en")
                or locale in seen_locales
                or not isinstance(voice, str)
                or not voice
                or len(voice) > 128
                or not voice.isascii()
                or any(not (c.isalnum() or c in "._-") for c in voice)
            ):
                raise ValueError("Piper voice mapping is invalid")
            seen_locales.add(locale)
        for value, minimum, maximum in (
            (self.speed, 0.5, 2.0),
            (self.connect_timeout_seconds, 0.1, 30.0),
            (self.request_timeout_seconds, 1.0, 120.0),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or not minimum <= float(value) <= maximum
            ):
                raise ValueError("Piper timing or speed is invalid")

    @property
    def endpoint(self) -> str:
        return self.base_url.rstrip("/") + "/audio/speech"

    def voice_for_locale(self, locale: str) -> str:
        try:
            return dict(self.voices)[locale]
        except KeyError:
            raise HostSpeechError(
                "unsupported_tts_locale",
                "No host TTS voice is configured for this locale",
            ) from None


class PiperLoopbackSynthesizer:
    def __init__(
        self,
        profile: PiperSpeechProfile = PiperSpeechProfile(),
        *,
        process_factory: Callable[..., object] = subprocess.Popen,
        monotonic: Callable[[], float] = time.monotonic,
        curl_path: str = "curl",
    ):
        if not isinstance(profile, PiperSpeechProfile):
            raise ValueError("Piper speech profile is invalid")
        if not callable(process_factory) or not callable(monotonic):
            raise ValueError("Piper process dependency is invalid")
        if curl_path != "curl" and not curl_path.startswith("/"):
            raise ValueError("Piper curl path is invalid")
        self.profile = profile
        self._process_factory = process_factory
        self._monotonic = monotonic
        self._curl_path = curl_path

    def synthesize(
        self,
        text: str,
        locale: str,
        cancel_event: threading.Event,
    ) -> Optional[bytes]:
        if (
            not isinstance(text, str)
            or not text
            or text != text.strip()
            or len(text) > MAX_TEXT_CHARACTERS
            or not isinstance(cancel_event, threading.Event)
        ):
            raise HostSpeechError("invalid_speech_text", "Speech text is invalid")
        voice = self.profile.voice_for_locale(locale)
        request = json.dumps(
            {
                "model": self.profile.model,
                "input": text,
                "voice": voice,
                "response_format": "wav",
                "speed": float(self.profile.speed),
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        if cancel_event.is_set():
            return None
        argv = [
            self._curl_path,
            "--silent",
            "--show-error",
            "--fail",
            "--noproxy",
            "*",
            "--connect-timeout",
            str(float(self.profile.connect_timeout_seconds)),
            "--max-time",
            str(float(self.profile.request_timeout_seconds)),
            "--max-filesize",
            str(MAX_WAV_BYTES),
            "--request",
            "POST",
            "--header",
            "Content-Type: application/json",
            "--header",
            "Accept: audio/wav",
            "--data-binary",
            "@-",
            self.profile.endpoint,
        ]
        try:
            process = self._process_factory(
                argv,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except OSError:
            raise HostSpeechError(
                "tts_provider_unavailable",
                "Piper request process could not start",
            ) from None
        result = _capture_bounded_cancellable(
            process,
            request,
            cancel_event,
            float(self.profile.request_timeout_seconds) + 1.0,
            self._monotonic,
        )
        if result is None:
            return None
        stdout, stderr = result
        if not isinstance(stdout, bytes) or not isinstance(stderr, bytes):
            raise HostSpeechError("invalid_tts_response", "Piper response is invalid")
        if len(stderr) > MAX_PROCESS_OUTPUT_BYTES:
            raise HostSpeechError("tts_provider_failed", "Piper error output is too large")
        if process.returncode != 0:
            raise HostSpeechError("tts_provider_failed", "Piper synthesis failed")
        validate_pcm16_mono_wav(stdout)
        return stdout


class EV3WAVSSHPlayer:
    def __init__(
        self,
        target: str,
        *,
        connect_timeout_seconds: int = 5,
        remote_player_path: str = REMOTE_AUDIO_PLAYER,
        process_factory: Callable[..., object] = subprocess.Popen,
        monotonic: Callable[[], float] = time.monotonic,
    ):
        self.target = _safe_target(target)
        self.remote_player_path = _safe_remote_path(remote_player_path)
        if (
            isinstance(connect_timeout_seconds, bool)
            or not isinstance(connect_timeout_seconds, int)
            or not 1 <= connect_timeout_seconds <= 30
            or not callable(process_factory)
            or not callable(monotonic)
        ):
            raise ValueError("EV3 WAV transport configuration is invalid")
        self.connect_timeout_seconds = connect_timeout_seconds
        self._process_factory = process_factory
        self._monotonic = monotonic

    @property
    def argv(self):
        return [
            "ssh",
            "-T",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout={}".format(self.connect_timeout_seconds),
            "-o",
            "StrictHostKeyChecking=yes",
            self.target,
            "python3",
            self.remote_player_path,
        ]

    def play(self, raw: bytes, cancel_event: threading.Event):
        metadata = validate_pcm16_mono_wav(raw)
        if not isinstance(cancel_event, threading.Event):
            raise HostSpeechError("invalid_speech_cancel", "Speech cancellation is invalid")
        if cancel_event.is_set():
            return None
        try:
            process = self._process_factory(
                self.argv,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except OSError:
            raise HostSpeechError(
                "speech_playback_unavailable",
                "EV3 audio process could not start",
            ) from None
        result = _communicate_cancellable(
            process,
            raw,
            cancel_event,
            (
                metadata.duration_ms / 1000.0
                + self.connect_timeout_seconds
                + EV3_PLAYBACK_STARTUP_ALLOWANCE_SECONDS
            ),
            self._monotonic,
        )
        if result is None:
            return None
        stdout, stderr = result
        if (
            not isinstance(stdout, bytes)
            or not isinstance(stderr, bytes)
            or len(stdout) > MAX_PROCESS_OUTPUT_BYTES
            or len(stderr) > MAX_PROCESS_OUTPUT_BYTES
        ):
            raise HostSpeechError("invalid_playback_response", "EV3 audio response is invalid")
        if process.returncode != 0:
            raise HostSpeechError("speech_playback_failed", "EV3 audio playback failed")
        try:
            response = json.loads(stdout.decode("ascii"))
        except (UnicodeError, ValueError):
            raise HostSpeechError("invalid_playback_response", "EV3 audio response is invalid") from None
        expected = {
            "status": "completed",
            "bytes": metadata.byte_count,
            "channels": metadata.channels,
            "sample_width_bytes": metadata.sample_width_bytes,
            "sample_rate_hz": metadata.sample_rate_hz,
            "frames": metadata.frames,
            "duration_ms": metadata.duration_ms,
        }
        if response != expected:
            raise HostSpeechError("invalid_playback_response", "EV3 audio receipt is invalid")
        return response


class HostPiperEV3Speaker:
    def __init__(self, synthesizer, player):
        if not callable(getattr(synthesizer, "synthesize", None)) or not callable(
            getattr(player, "play", None)
        ):
            raise ValueError("Host speech dependencies are invalid")
        self.synthesizer = synthesizer
        self.player = player

    def __call__(self, text: str, locale: str, cancel_event: threading.Event):
        audio = self.synthesizer.synthesize(text, locale, cancel_event)
        if audio is None or cancel_event.is_set():
            return None
        return self.player.play(audio, cancel_event)


__all__ = (
    "DEFAULT_PIPER_BASE_URL",
    "DEFAULT_PIPER_MODEL",
    "DEFAULT_SWEDISH_VOICE",
    "EV3WAVSSHPlayer",
    "HostPiperEV3Speaker",
    "HostSpeechError",
    "PiperLoopbackSynthesizer",
    "PiperSpeechProfile",
    "REMOTE_AUDIO_PLAYER",
    "validate_pcm16_mono_wav",
)
