#!/usr/bin/env python3
"""Persistent, speech-only JSONL worker for the EV3.

The process loads :class:`RobotHAL` once, then accepts only bounded speech and
shutdown requests.  It never exposes motor, sensor, shell, or network
operations.  Closing the owning SSH channel terminates this process; while
audio is active RobotHAL's signal cleanup also terminates espeak and aplay.
"""

from __future__ import print_function

import json
import os
import sys

if __package__:
    from .robot_hal import (
        RobotHAL,
        SafetyError,
        SpeechBusyError,
    )
else:
    from robot_hal import RobotHAL, SafetyError, SpeechBusyError


REQUEST_SCHEMA = "ev3-speech-request/v1"
RESPONSE_SCHEMA = "ev3-speech-response/v1"
READY_SCHEMA = "ev3-speech-ready/v1"
MAX_FRAME_BYTES = 4096
MAX_REQUESTS = 1000
MAX_IDENTIFIER_CHARACTERS = 128
ALLOWED_VOICES = ("sv", "en")


def default_config_path():
    return os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "ev3rstorm.json",
        )
    )


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _reject_constant(_value):
    raise ValueError("non-finite JSON value")


def _identifier(value):
    try:
        ascii_value = value.encode("ascii") if isinstance(value, str) else None
    except UnicodeEncodeError:
        ascii_value = None
    if (
        not isinstance(value, str)
        or not value
        or len(value) > MAX_IDENTIFIER_CHARACTERS
        or ascii_value is None
        or not all(
            character.isalnum() or character in "-_"
            for character in value
        )
    ):
        raise ValueError("request_id is invalid")
    return value


def _decode_request(raw):
    if not isinstance(raw, str):
        raise ValueError("request frame is not text")
    if len(raw.encode("utf-8")) > MAX_FRAME_BYTES or not raw.endswith("\n"):
        raise ValueError("request frame is invalid")
    value = json.loads(
        raw,
        object_pairs_hook=_strict_object,
        parse_constant=_reject_constant,
    )
    if not isinstance(value, dict) or set(value) != {
        "schema",
        "request_id",
        "operation",
        "arguments",
    }:
        raise ValueError("request schema is invalid")
    if value["schema"] != REQUEST_SCHEMA:
        raise ValueError("request protocol is unsupported")
    _identifier(value["request_id"])
    if value["operation"] not in ("speak", "shutdown"):
        raise ValueError("operation is unsupported")
    if not isinstance(value["arguments"], dict):
        raise ValueError("request arguments are invalid")
    if value["operation"] == "shutdown":
        if value["arguments"]:
            raise ValueError("shutdown arguments are invalid")
    else:
        arguments = value["arguments"]
        if set(arguments) != {"text", "voice"}:
            raise ValueError("speech arguments are invalid")
        if (
            not isinstance(arguments["text"], str)
            or not arguments["text"]
            or len(arguments["text"]) > 160
            or "\x00" in arguments["text"]
        ):
            raise ValueError("speech text is invalid")
        if arguments["voice"] not in ALLOWED_VOICES:
            raise ValueError("speech voice is invalid")
    return value


def _response(request_id, ok, payload):
    return {
        "schema": RESPONSE_SCHEMA,
        "request_id": request_id,
        "ok": ok,
        "result" if ok else "error": payload,
    }


def _write(value, output_stream):
    output_stream.write(
        json.dumps(
            value,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        )
        + "\n"
    )
    output_stream.flush()


def _error(code, message, fatal=False):
    return {
        "code": code,
        "message": message,
        "fatal": bool(fatal),
    }


def run_worker(
    config_path=None,
    input_stream=None,
    output_stream=None,
    robot_factory=RobotHAL,
):
    """Run the fixed speech protocol until shutdown, EOF, or fatal failure."""

    input_stream = input_stream if input_stream is not None else sys.stdin
    output_stream = output_stream if output_stream is not None else sys.stdout
    robot = robot_factory(config_path or default_config_path())
    _write({"schema": READY_SCHEMA, "status": "ready"}, output_stream)
    request_count = 0

    while True:
        raw = input_stream.readline(MAX_FRAME_BYTES + 1)
        if raw == "":
            return 0
        request_id = None
        try:
            request = _decode_request(raw)
            request_id = request["request_id"]
            request_count += 1
            if request_count > MAX_REQUESTS:
                _write(
                    _response(
                        request_id,
                        False,
                        _error(
                            "speech_request_budget_exhausted",
                            "Speech worker request budget is exhausted",
                            fatal=True,
                        ),
                    ),
                    output_stream,
                )
                return 1
            if request["operation"] == "shutdown":
                _write(
                    _response(
                        request_id,
                        True,
                        {"status": "shutdown"},
                    ),
                    output_stream,
                )
                return 0
            result = robot.speak(
                request["arguments"]["text"],
                voice=request["arguments"]["voice"],
            )
            _write(_response(request_id, True, result), output_stream)
        except (TypeError, ValueError, SafetyError) as error:
            _write(
                _response(
                    request_id,
                    False,
                    _error("invalid_speech_request", str(error)[:160]),
                ),
                output_stream,
            )
        except SpeechBusyError as error:
            _write(
                _response(
                    request_id,
                    False,
                    _error("speech_output_busy", str(error)[:160]),
                ),
                output_stream,
            )
        except (IOError, OSError, RuntimeError) as error:
            _write(
                _response(
                    request_id,
                    False,
                    _error("speech_playback_failed", str(error)[:160]),
                ),
                output_stream,
            )
        except BaseException as error:
            try:
                _write(
                    _response(
                        request_id,
                        False,
                        _error(
                            "speech_worker_failed",
                            type(error).__name__,
                            fatal=True,
                        ),
                    ),
                    output_stream,
                )
            except BaseException:
                pass
            return 1


def main():
    return run_worker()


if __name__ == "__main__":
    sys.exit(main())
