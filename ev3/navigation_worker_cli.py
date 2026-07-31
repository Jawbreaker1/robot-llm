#!/usr/bin/env python3
"""Foreground JSONL entry point for the EV3 navigation worker."""

from __future__ import print_function

import argparse
import select
import signal
import sys
import time

if __package__:
    from .navigation_profile import (
        MAX_FRAME_BYTES,
        MAX_PROCESS_SECONDS,
        MAX_REQUESTS,
    )
    from .navigation_worker import (
        DEFAULT_CONFIG_PATH,
        NavigationWorker,
    )
    from .navigation_worker_protocol import (
        WorkerError,
        decode_request,
        error_payload,
        response_object,
        write_response,
    )
else:
    from navigation_profile import (
        MAX_FRAME_BYTES,
        MAX_PROCESS_SECONDS,
        MAX_REQUESTS,
    )
    from navigation_worker import (
        DEFAULT_CONFIG_PATH,
        NavigationWorker,
    )
    from navigation_worker_protocol import (
        WorkerError,
        decode_request,
        error_payload,
        response_object,
        write_response,
    )


def _install_signal_handlers(flag):
    def handle_signal(_signum, _frame):
        flag[0] = True

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)


def _binary_input_stream(stream):
    """Use the raw pipe so select() and readline() see the same bytes."""
    binary = getattr(stream, "buffer", stream)
    raw = getattr(binary, "raw", None)
    if raw is not None and hasattr(raw, "readline"):
        return raw
    return binary


def _input_cancel_requested(binary_input, signal_flag):
    """Treat a signal, queued frame, EOF, or unreadable channel as cancel.

    The host protocol permits only one outstanding request.  While a pulse is
    executing, readable stdin therefore means either that the SSH channel
    closed or that the host queued a stop/new request before receiving the
    current response.  Both cases cancel locally instead of allowing later
    motor slices to start.
    """
    if signal_flag[0]:
        return True
    try:
        readable, _writable, _errors = select.select(
            [binary_input],
            [],
            [],
            0,
        )
    except (IOError, OSError, ValueError):
        return True
    return bool(readable)


def _worker_controller_id(worker):
    return getattr(worker, "controller_id", "ev3-worker-startup")


def _worker_state_version(worker):
    return getattr(worker, "state_version", 0)


def _write_worker_response(
    worker,
    request_id,
    ok,
    payload,
    output_stream,
):
    write_response(
        response_object(
            _worker_controller_id(worker),
            request_id,
            ok,
            payload,
            _worker_state_version(worker),
        ),
        output_stream=output_stream,
    )


def run_worker(
    config_path=DEFAULT_CONFIG_PATH,
    input_stream=None,
    output_stream=None,
):
    """Run one bounded worker session and return its process exit code."""
    input_stream = input_stream if input_stream is not None else sys.stdin
    output_stream = (
        output_stream if output_stream is not None else sys.stdout
    )
    binary_input = _binary_input_stream(input_stream)
    worker = None
    exit_requested = [False]
    _install_signal_handlers(exit_requested)
    active_request_id = None
    pending_error = None
    try:
        # Retain the partially initialized object so any acquired motor lock
        # still follows the verified cleanup path if startup fails.
        worker = NavigationWorker.__new__(NavigationWorker)
        worker.owner = None
        worker.closed = False
        NavigationWorker.__init__(
            worker,
            config_path=config_path,
            cancel_requested=lambda: _input_cancel_requested(
                binary_input,
                exit_requested,
            ),
        )
        while (
            not exit_requested[0]
            and not worker.shutdown_requested
            and time.monotonic() < worker.deadline
        ):
            remaining = max(
                0.0,
                worker.deadline - time.monotonic(),
            )
            readable, _writable, _errors = select.select(
                [binary_input],
                [],
                [],
                min(0.20, remaining),
            )
            if not readable:
                continue
            raw = binary_input.readline(MAX_FRAME_BYTES + 1)
            if raw == b"":
                break
            active_request_id = None
            try:
                request = decode_request(
                    raw,
                    worker.controller_id,
                )
                active_request_id = request["request_id"]
                if active_request_id in worker.seen_request_ids:
                    raise WorkerError(
                        "duplicate_request_id",
                        "request_id has already been used",
                        request_id=active_request_id,
                    )
                worker.seen_request_ids.add(active_request_id)
                worker.request_count += 1
                if worker.request_count > MAX_REQUESTS:
                    raise WorkerError(
                        "request_budget_exhausted",
                        "The fixed process request budget is exhausted",
                        request_id=active_request_id,
                        fatal=True,
                    )
                result = worker.execute(request)
                _write_worker_response(
                    worker,
                    active_request_id,
                    True,
                    result,
                    output_stream,
                )
            except BaseException as error:
                stop = None
                fatal = (
                    bool(getattr(error, "fatal", False))
                    or not isinstance(error, WorkerError)
                )
                try:
                    finish = worker._require_verified_finish()
                    stop = finish["stop"]
                except BaseException:
                    fatal = True
                payload = error_payload(
                    error,
                    worker,
                    stop=stop,
                )
                payload["fatal"] = fatal
                _write_worker_response(
                    worker,
                    getattr(error, "request_id", None)
                    or active_request_id,
                    False,
                    payload,
                    output_stream,
                )
                if fatal:
                    pending_error = error
                    break
    except BaseException as error:
        pending_error = error
        if worker is not None:
            try:
                _write_worker_response(
                    worker,
                    getattr(error, "request_id", None)
                    or active_request_id,
                    False,
                    error_payload(error, worker),
                    output_stream,
                )
            except BaseException:
                pass
    finally:
        if worker is not None and not worker.closed:
            try:
                worker.close()
            except BaseException as error:
                if pending_error is None:
                    pending_error = error
    return 1 if pending_error is not None else 0


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Run the bounded, policy-free EV3 navigation JSONL worker."
        )
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_PATH,
        help="Path to the validated EV3 robot configuration.",
    )
    return parser


def main(argv=None):
    arguments = build_parser().parse_args(argv)
    return run_worker(config_path=arguments.config)


if __name__ == "__main__":
    sys.exit(main())
