#!/usr/bin/env python3
"""Persistent binary-safe WAV playback worker for one owning SSH session."""

from __future__ import print_function

import json
import os
import subprocess
import sys

if __package__:
    from .audio_playback_cli import AudioValidationError, play_wav
    from .robot_hal import RobotHAL, SpeechBusyError
else:
    from audio_playback_cli import AudioValidationError, play_wav
    from robot_hal import RobotHAL, SpeechBusyError


REQUEST_SCHEMA = "ev3-audio-request/v1"
RESPONSE_SCHEMA = "ev3-audio-response/v1"
READY_SCHEMA = "ev3-audio-ready/v1"
MAX_HEADER_BYTES = 512
MAX_WAV_BYTES = 4 * 1024 * 1024
MAX_REQUESTS = 1000


def default_config_path():
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "ev3rstorm.json")
    )


def _strict_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError("duplicate JSON key")
        value[key] = item
    return value


def _identifier(value):
    try:
        ascii_value = value.encode("ascii") if isinstance(value, str) else None
    except UnicodeEncodeError:
        ascii_value = None
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 128
        or ascii_value is None
        or not all(c.isalnum() or c in "-_" for c in value)
    ):
        raise ValueError("request_id is invalid")
    return value


def _decode_header(raw):
    if (
        not isinstance(raw, bytes)
        or not raw.endswith(b"\n")
        or len(raw) > MAX_HEADER_BYTES
    ):
        raise ValueError("audio header frame is invalid")
    value = json.loads(
        raw.decode("ascii"),
        object_pairs_hook=_strict_object,
        parse_constant=lambda _value: (_ for _ in ()).throw(ValueError()),
    )
    if not isinstance(value, dict) or set(value) != {
        "schema",
        "request_id",
        "operation",
        "byte_count",
    }:
        raise ValueError("audio header schema is invalid")
    if value["schema"] != REQUEST_SCHEMA:
        raise ValueError("audio protocol is unsupported")
    _identifier(value["request_id"])
    operation = value["operation"]
    count = value["byte_count"]
    if operation not in ("play", "shutdown"):
        raise ValueError("audio operation is unsupported")
    if isinstance(count, bool) or not isinstance(count, int):
        raise ValueError("audio byte_count is invalid")
    if operation == "shutdown":
        if count != 0:
            raise ValueError("shutdown byte_count is invalid")
    elif not 44 <= count <= MAX_WAV_BYTES:
        raise ValueError("audio byte_count is invalid")
    return value


def _read_exact(stream, count):
    chunks = []
    remaining = count
    while remaining:
        chunk = stream.read(min(16384, remaining))
        if not chunk:
            raise EOFError("audio payload ended early")
        if not isinstance(chunk, bytes) or len(chunk) > remaining:
            raise ValueError("audio payload is invalid")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _response(request_id, ok, payload):
    return {
        "schema": RESPONSE_SCHEMA,
        "request_id": request_id,
        "ok": bool(ok),
        "result" if ok else "error": payload,
    }


def _error(code, fatal):
    return {"code": code, "fatal": bool(fatal)}


def _write(output_stream, value):
    output_stream.write(
        json.dumps(value, ensure_ascii=True, separators=(",", ":"), sort_keys=True)
        + "\n"
    )
    output_stream.flush()


def run_worker(
    config_path=None,
    input_stream=None,
    output_stream=None,
    robot_factory=RobotHAL,
    player_factory=subprocess.Popen,
):
    input_stream = input_stream or getattr(sys.stdin, "buffer", sys.stdin)
    output_stream = output_stream or sys.stdout
    robot = robot_factory(config_path or default_config_path())
    _write(output_stream, {"schema": READY_SCHEMA, "status": "ready"})
    request_count = 0
    while True:
        raw_header = input_stream.readline(MAX_HEADER_BYTES + 1)
        if raw_header == b"":
            return 0
        request_id = None
        try:
            header = _decode_header(raw_header)
            request_id = header["request_id"]
            request_count += 1
            if request_count > MAX_REQUESTS:
                _write(
                    output_stream,
                    _response(request_id, False, _error("request_budget_exhausted", True)),
                )
                return 1
            if header["operation"] == "shutdown":
                _write(
                    output_stream,
                    _response(request_id, True, {"status": "shutdown"}),
                )
                return 0
            audio = _read_exact(input_stream, header["byte_count"])
            result = play_wav(
                audio,
                robot=robot,
                popen_factory=player_factory,
            )
            _write(output_stream, _response(request_id, True, result))
        except (UnicodeError, ValueError, EOFError) as error:
            fatal = request_id is None or isinstance(error, EOFError)
            _write(
                output_stream,
                _response(
                    request_id,
                    False,
                    _error(
                        "invalid_audio_frame" if fatal else "invalid_wav",
                        fatal,
                    ),
                ),
            )
            if fatal:
                return 1
        except SpeechBusyError:
            _write(
                output_stream,
                _response(request_id, False, _error("audio_busy", False)),
            )
        except (IOError, OSError, RuntimeError, AudioValidationError):
            _write(
                output_stream,
                _response(request_id, False, _error("audio_playback_failed", False)),
            )
        except BaseException:
            try:
                _write(
                    output_stream,
                    _response(request_id, False, _error("audio_worker_failed", True)),
                )
            except BaseException:
                pass
            return 1


def main():
    if len(sys.argv) != 1:
        return 2
    return run_worker()


if __name__ == "__main__":
    sys.exit(main())
