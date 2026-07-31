#!/usr/bin/env python3
"""Validate and play one bounded WAV received on stdin.

This module is Python 3.5 compatible for ev3dev-stretch. It accepts no paths,
shell text, motor commands, or network operations.
"""

from __future__ import print_function

import io
import json
import os
import signal
import subprocess
import sys
import threading
import time
import wave

if __package__:
    from .robot_hal import RobotHAL, SpeechBusyError
else:
    from robot_hal import RobotHAL, SpeechBusyError


MAX_WAV_BYTES = 4 * 1024 * 1024
MAX_DURATION_MS = 20000
MAX_PROCESS_OUTPUT_BYTES = 8192


class AudioValidationError(ValueError):
    pass


class AudioInterruptedError(RuntimeError):
    pass


def default_config_path():
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "ev3rstorm.json")
    )


def validate_wav(raw):
    if not isinstance(raw, bytes) or not 44 <= len(raw) <= MAX_WAV_BYTES:
        raise AudioValidationError("WAV size is invalid")
    try:
        source = wave.open(io.BytesIO(raw), "rb")
        try:
            channels = source.getnchannels()
            width = source.getsampwidth()
            rate = source.getframerate()
            frames = source.getnframes()
            compression = source.getcomptype()
            payload = source.readframes(frames + 1)
        finally:
            source.close()
    except (EOFError, wave.Error):
        raise AudioValidationError("Input is not a WAV")
    if (
        channels != 1
        or width != 2
        or compression != "NONE"
        or not 8000 <= rate <= 48000
        or frames <= 0
        or len(payload) != frames * channels * width
    ):
        raise AudioValidationError(
            "WAV must be mono 16-bit PCM at 8-48 kHz"
        )
    duration_ms = (frames * 1000 + rate - 1) // rate
    if duration_ms > MAX_DURATION_MS:
        raise AudioValidationError("WAV duration is too long")
    return {
        "bytes": len(raw),
        "channels": channels,
        "sample_width_bytes": width,
        "sample_rate_hz": rate,
        "frames": frames,
        "duration_ms": duration_ms,
    }


def _terminate(process):
    if process is None:
        return
    try:
        if process.poll() is None:
            process.kill()
    except (IOError, OSError):
        pass
    try:
        process.wait(timeout=1.0)
    except (IOError, OSError, subprocess.TimeoutExpired):
        pass


def play_wav(
    raw,
    config_path=None,
    robot=None,
    robot_factory=RobotHAL,
    popen_factory=subprocess.Popen,
):
    metadata = validate_wav(raw)
    if robot is None:
        robot = robot_factory(config_path or default_config_path())
    lock_handle = robot._acquire_speech_lock()
    player = None
    previous_handlers = {}
    try:
        if threading.current_thread() is threading.main_thread():
            def interrupt(signum, _frame):
                raise AudioInterruptedError(
                    "Audio interrupted by signal {}".format(signum)
                )

            for signum in (signal.SIGHUP, signal.SIGTERM):
                previous_handlers[signum] = signal.getsignal(signum)
                signal.signal(signum, interrupt)
        player = popen_factory(
            ["aplay", "--quiet"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        output, error = player.communicate(
            input=raw,
            timeout=metadata["duration_ms"] / 1000.0 + 2.0,
        )
        if (
            not isinstance(output, bytes)
            or not isinstance(error, bytes)
            or len(output) > MAX_PROCESS_OUTPUT_BYTES
            or len(error) > MAX_PROCESS_OUTPUT_BYTES
        ):
            raise RuntimeError("Audio player output is invalid")
        if player.returncode != 0:
            raise RuntimeError("Audio player failed")
    except subprocess.TimeoutExpired:
        _terminate(player)
        raise RuntimeError("Audio playback timed out")
    except BaseException:
        _terminate(player)
        raise
    finally:
        for signum, handler in previous_handlers.items():
            try:
                signal.signal(signum, handler)
            except (ValueError, OSError, RuntimeError):
                pass
        try:
            import fcntl
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
        finally:
            lock_handle.close()
    result = {"status": "completed"}
    result.update(metadata)
    return result


def run(input_stream=None, output_stream=None, **kwargs):
    input_stream = input_stream or getattr(sys.stdin, "buffer", sys.stdin)
    output_stream = output_stream or sys.stdout
    try:
        raw = input_stream.read(MAX_WAV_BYTES + 1)
        if len(raw) > MAX_WAV_BYTES:
            raise AudioValidationError("WAV size is invalid")
        result = play_wav(raw, **kwargs)
        output_stream.write(
            json.dumps(result, separators=(",", ":"), sort_keys=True) + "\n"
        )
        output_stream.flush()
        return 0
    except (AudioValidationError, SpeechBusyError) as error:
        payload = {
            "status": "failed",
            "code": (
                "audio_busy"
                if isinstance(error, SpeechBusyError)
                else "invalid_wav"
            ),
        }
    except BaseException:
        payload = {"status": "failed", "code": "audio_playback_failed"}
    output_stream.write(
        json.dumps(payload, separators=(",", ":"), sort_keys=True) + "\n"
    )
    output_stream.flush()
    return 1


def main():
    if len(sys.argv) != 1:
        return 2
    return run()


if __name__ == "__main__":
    sys.exit(main())
