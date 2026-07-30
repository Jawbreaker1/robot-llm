"""Persistent, motor-free EV3 sensor transport over one SSH process."""

import json
import queue
import secrets
import subprocess
import threading
from typing import Any, Callable, Dict, Mapping, Optional


PROTOCOL_VERSION = 1
RESPONSE_SCHEMA = "ev3-peripheral-response/v1"
REMOTE_DAEMON = "/home/robot/robot-llm/ev3/peripheral_daemon.py"
MAX_REQUEST_BYTES = 2048
MAX_RESPONSE_BYTES = 4096
MAX_ERROR_BYTES = 16 * 1024
MAX_TTL_MS = 5000
_RECEIVE_FAILURE = object()
ALLOWED_OPERATIONS = frozenset(
    ("describe", "read_sensor", "shutdown")
)

ProcessFactory = Callable[..., Any]


class PeripheralSSHError(RuntimeError):
    """Base class for persistent peripheral transport failures."""


class PeripheralSSHConfigurationError(PeripheralSSHError):
    pass


class PeripheralSSHTransportError(PeripheralSSHError):
    pass


class PeripheralSSHTimeoutError(PeripheralSSHTransportError):
    pass


class PeripheralSSHChannelPoisonedError(
    PeripheralSSHTransportError
):
    pass


class PeripheralSSHProtocolError(PeripheralSSHError):
    pass


class PeripheralRemoteError(PeripheralSSHError):
    def __init__(self, code: str, message: str):
        self.code = code
        RuntimeError.__init__(self, message)


def _validate_identifier(
    name: str,
    value: str,
    maximum: int,
) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or "\x00" in value
        or "\n" in value
        or "\r" in value
    ):
        raise PeripheralSSHConfigurationError(
            "{} is invalid".format(name)
        )
    return value


def _validate_target(target: str) -> str:
    if (
        not isinstance(target, str)
        or not target
        or target != target.strip()
        or target.startswith("-")
        or len(target) > 255
        or any(
            not (
                character.isalnum()
                or character in "._-@:%+"
            )
            for character in target
        )
    ):
        raise PeripheralSSHConfigurationError(
            "SSH target is invalid"
        )
    return target


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key")
        result[key] = value
    return result


def _reject_constant(_value: str) -> None:
    raise ValueError("Non-finite JSON number")


def _exact_fields(
    value: object,
    expected,
    context: str,
) -> Mapping[str, object]:
    if not isinstance(value, dict) or frozenset(value) != frozenset(
        expected
    ):
        raise PeripheralSSHProtocolError(
            "{} fields are invalid".format(context)
        )
    return value


def _response_text(raw: bytes) -> str:
    if not isinstance(raw, bytes):
        raise PeripheralSSHProtocolError(
            "Peripheral response was not bytes"
        )
    if len(raw) > MAX_RESPONSE_BYTES:
        raise PeripheralSSHProtocolError(
            "Peripheral response was too large"
        )
    if not raw.endswith(b"\n") or raw.count(b"\n") != 1:
        raise PeripheralSSHProtocolError(
            "Peripheral response was not one JSON line"
        )
    try:
        return raw[:-1].decode("utf-8")
    except UnicodeDecodeError:
        raise PeripheralSSHProtocolError(
            "Peripheral response was not valid UTF-8"
        ) from None


def _validate_description(
    result: object,
    controller_id: str,
) -> Dict[str, object]:
    result = _exact_fields(
        result,
        (
            "protocol_version",
            "robot_id",
            "controller_id",
            "peripheral_instance_id",
            "motion_enabled",
            "speech_enabled",
            "capabilities",
        ),
        "describe result",
    )
    protocol_version = result["protocol_version"]
    if (
        isinstance(protocol_version, bool)
        or not isinstance(protocol_version, int)
        or protocol_version != PROTOCOL_VERSION
        or result["controller_id"] != controller_id
        or result["motion_enabled"] is not False
        or result["speech_enabled"] is not False
    ):
        raise PeripheralSSHProtocolError(
            "Peripheral capabilities are unsafe or mismatched"
        )
    for name in (
        "robot_id",
        "controller_id",
        "peripheral_instance_id",
    ):
        value = result[name]
        if (
            not isinstance(value, str)
            or not value
            or value != value.strip()
            or len(value) > 128
            or "\x00" in value
            or "\n" in value
            or "\r" in value
        ):
            raise PeripheralSSHProtocolError(
                "Peripheral identity is invalid"
            )
    capabilities = _exact_fields(
        result["capabilities"],
        ("configured_sensor_read",),
        "capabilities",
    )
    sensor_capability = _exact_fields(
        capabilities["configured_sensor_read"],
        ("enabled", "roles"),
        "sensor capability",
    )
    roles = sensor_capability["roles"]
    if (
        sensor_capability["enabled"] is not True
        or not isinstance(roles, list)
        or not roles
        or len(roles) > 64
        or any(
            not isinstance(role, str)
            or not role
            or role != role.strip()
            or len(role) > 64
            for role in roles
        )
    ):
        raise PeripheralSSHProtocolError(
            "Peripheral sensor capability is invalid"
        )
    if roles != sorted(set(roles)):
        raise PeripheralSSHProtocolError(
            "Peripheral sensor capability is invalid"
        )
    return dict(result)


def _validate_sensor_result(
    result: object,
    role: str,
) -> Dict[str, object]:
    result = _exact_fields(
        result,
        (
            "observed_monotonic_ms",
            "role",
            "port",
            "driver",
            "mode",
            "value0",
            "units",
        ),
        "sensor result",
    )
    observed_at_ms = result["observed_monotonic_ms"]
    value = result["value0"]
    if (
        isinstance(observed_at_ms, bool)
        or not isinstance(observed_at_ms, int)
        or observed_at_ms < 0
        or isinstance(value, bool)
        or not isinstance(value, int)
        or result["role"] != role
    ):
        raise PeripheralSSHProtocolError(
            "Peripheral sensor value is invalid"
        )
    for name, maximum, allow_empty in (
        ("port", 64, False),
        ("driver", 128, False),
        ("mode", 64, False),
        ("units", 64, True),
    ):
        item = result[name]
        if (
            not isinstance(item, str)
            or len(item) > maximum
            or (not allow_empty and not item)
            or item != item.strip()
            or "\x00" in item
            or "\n" in item
            or "\r" in item
        ):
            raise PeripheralSSHProtocolError(
                "Peripheral sensor metadata is invalid"
            )
    return dict(result)


def decode_peripheral_response(
    raw: bytes,
    expected_request_id: str,
    controller_id: str,
    operation: str,
    arguments: Mapping[str, object],
) -> Dict[str, object]:
    """Decode one exact response correlated to one outstanding request."""

    text = _response_text(raw)
    try:
        response = json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (TypeError, ValueError):
        raise PeripheralSSHProtocolError(
            "Peripheral response was not strict JSON"
        ) from None
    if not isinstance(response, dict):
        raise PeripheralSSHProtocolError(
            "Peripheral response was not an object"
        )
    common = {
        "schema",
        "request_id",
        "controller_id",
        "ok",
    }
    expected_fields = (
        common | {"result"}
        if response.get("ok") is True
        else common | {"error"}
    )
    _exact_fields(response, expected_fields, "response")
    if (
        response["schema"] != RESPONSE_SCHEMA
        or response["request_id"] != expected_request_id
        or response["controller_id"] != controller_id
        or not isinstance(response["ok"], bool)
    ):
        raise PeripheralSSHProtocolError(
            "Peripheral response identity is invalid"
        )
    if response["ok"] is False:
        error = _exact_fields(
            response["error"],
            ("code", "message"),
            "remote error",
        )
        code = error["code"]
        message = error["message"]
        if (
            not isinstance(code, str)
            or not code
            or code != code.strip()
            or len(code) > 128
            or not isinstance(message, str)
            or not message
            or message != message.strip()
            or len(message) > 240
            or any(
                token in code or token in message
                for token in ("\x00", "\n", "\r")
            )
        ):
            raise PeripheralSSHProtocolError(
                "Peripheral error payload is invalid"
            )
        raise PeripheralRemoteError(code, message)

    result = response["result"]
    if operation == "describe":
        return _validate_description(result, controller_id)
    if operation == "read_sensor":
        return _validate_sensor_result(
            result,
            str(arguments["role"]),
        )
    if operation == "shutdown":
        _exact_fields(result, ("status",), "shutdown result")
        if result["status"] != "closed":
            raise PeripheralSSHProtocolError(
                "Peripheral shutdown was not confirmed"
            )
        return dict(result)
    raise PeripheralSSHProtocolError(
        "Peripheral response operation is invalid"
    )


class _StdoutPump(threading.Thread):
    def __init__(self, stream, responses, failure_callback):
        threading.Thread.__init__(
            self,
            name="ev3-peripheral-stdout",
        )
        self.daemon = True
        self._stream = stream
        self._responses = responses
        self._failure_callback = failure_callback

    def run(self) -> None:
        while True:
            try:
                raw = self._stream.readline(MAX_RESPONSE_BYTES + 1)
            except BaseException:
                self._failure_callback(
                    "Peripheral response read failed"
                )
                return
            if raw == b"":
                self._failure_callback(
                    "Peripheral response stream closed"
                )
                return
            if (
                not isinstance(raw, bytes)
                or len(raw) > MAX_RESPONSE_BYTES
                or not raw.endswith(b"\n")
            ):
                self._failure_callback(
                    "Peripheral response frame was invalid"
                )
                return
            try:
                self._responses.put_nowait(raw)
            except queue.Full:
                self._failure_callback(
                    "Peripheral response queue overflowed"
                )
                return


class _StderrPump(threading.Thread):
    def __init__(self, stream):
        threading.Thread.__init__(
            self,
            name="ev3-peripheral-stderr",
        )
        self.daemon = True
        self._stream = stream
        self._chunks = bytearray()
        self._lock = threading.Lock()

    def run(self) -> None:
        while True:
            try:
                chunk = self._stream.read(1024)
            except BaseException:
                return
            if not chunk:
                return
            with self._lock:
                remaining = MAX_ERROR_BYTES - len(self._chunks)
                if remaining > 0:
                    self._chunks.extend(chunk[:remaining])

    def summary(self) -> str:
        with self._lock:
            raw = bytes(self._chunks)
        return " ".join(
            raw.decode("utf-8", errors="replace").split()
        )[:240]


class PeripheralSSHSession:
    """One bounded request stream to one motor-free EV3 process."""

    def __init__(
        self,
        target: str,
        controller_id: str,
        process_factory: ProcessFactory = subprocess.Popen,
        connect_timeout_seconds: int = 3,
        response_timeout_seconds: float = 3.0,
        startup_response_timeout_seconds: float = 30.0,
        remote_session_ms: int = 60_000,
        remote_max_requests: int = 128,
    ):
        self.target = _validate_target(target)
        self.controller_id = _validate_identifier(
            "controller_id",
            controller_id,
            128,
        )
        if not callable(process_factory):
            raise PeripheralSSHConfigurationError(
                "Process factory is invalid"
            )
        if (
            isinstance(connect_timeout_seconds, bool)
            or not isinstance(connect_timeout_seconds, int)
            or not 1 <= connect_timeout_seconds <= 30
        ):
            raise PeripheralSSHConfigurationError(
                "Connect timeout is invalid"
            )
        for name, value, minimum, maximum in (
            (
                "response timeout",
                response_timeout_seconds,
                0.1,
                30,
            ),
            (
                "startup response timeout",
                startup_response_timeout_seconds,
                0.1,
                60,
            ),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not minimum <= value <= maximum
            ):
                raise PeripheralSSHConfigurationError(
                    "{} is invalid".format(name)
                )
        if (
            isinstance(remote_session_ms, bool)
            or not isinstance(remote_session_ms, int)
            or not 1000 <= remote_session_ms <= 120_000
        ):
            raise PeripheralSSHConfigurationError(
                "Remote session duration is invalid"
            )
        if (
            isinstance(remote_max_requests, bool)
            or not isinstance(remote_max_requests, int)
            or not 2 <= remote_max_requests <= 128
        ):
            raise PeripheralSSHConfigurationError(
                "Remote request budget is invalid"
            )

        argv = [
            "ssh",
            "-T",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout={}".format(connect_timeout_seconds),
            "-o",
            "StrictHostKeyChecking=yes",
            self.target,
            "python3",
            REMOTE_DAEMON,
            "--max-session-ms",
            str(remote_session_ms),
            "--max-requests",
            str(remote_max_requests),
        ]
        try:
            process = process_factory(
                argv,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except OSError:
            raise PeripheralSSHTransportError(
                "Could not start SSH peripheral process"
            ) from None
        if (
            process.stdin is None
            or process.stdout is None
            or process.stderr is None
        ):
            raise PeripheralSSHTransportError(
                "SSH peripheral process has incomplete pipes"
            )

        self.argv = argv
        self._process = process
        self._response_timeout_seconds = float(
            response_timeout_seconds
        )
        self._startup_response_timeout_seconds = float(
            startup_response_timeout_seconds
        )
        self._responses = queue.Queue(maxsize=4)
        self._request_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._lifecycle_state = "OPEN"
        self._poison_reason = None
        self._receive_failure = None
        self._request_prefix = secrets.token_hex(8)
        self._request_number = 0
        self._described = False
        self._shutdown_confirmed = False
        self._stdout_pump = _StdoutPump(
            process.stdout,
            self._responses,
            self._mark_receive_failure,
        )
        self._stderr_pump = _StderrPump(process.stderr)
        self._stdout_pump.start()
        self._stderr_pump.start()

    @property
    def lifecycle_state(self) -> str:
        with self._state_lock:
            return self._lifecycle_state

    def _close_input(self) -> None:
        try:
            self._process.stdin.close()
        except (OSError, ValueError):
            pass

    def _mark_receive_failure(self, message: str) -> None:
        close_input = False
        with self._state_lock:
            if self._receive_failure is None:
                self._receive_failure = message
            if self._lifecycle_state == "OPEN":
                self._lifecycle_state = "POISONED"
                self._poison_reason = message
                close_input = True
        if close_input:
            self._close_input()
        try:
            self._responses.put_nowait(_RECEIVE_FAILURE)
        except queue.Full:
            pass

    def _poison(
        self,
        message: str,
        timeout: bool = False,
    ) -> PeripheralSSHTransportError:
        with self._state_lock:
            if self._lifecycle_state == "OPEN":
                self._lifecycle_state = "POISONED"
            if self._poison_reason is None:
                self._poison_reason = message
        self._close_input()
        if timeout:
            return PeripheralSSHTimeoutError(message)
        return PeripheralSSHChannelPoisonedError(message)

    def _require_open(self) -> None:
        with self._state_lock:
            state = self._lifecycle_state
            reason = self._poison_reason or self._receive_failure
        if state == "POISONED":
            raise PeripheralSSHChannelPoisonedError(
                reason or "Peripheral channel is poisoned"
            )
        if state != "OPEN":
            raise PeripheralSSHTransportError(
                "Peripheral session is not open"
            )

    def _encode_request(
        self,
        request_id: str,
        operation: str,
        arguments: Mapping[str, object],
        ttl_ms: int,
    ) -> bytes:
        try:
            frame = (
                json.dumps(
                    {
                        "protocol_version": PROTOCOL_VERSION,
                        "controller_id": self.controller_id,
                        "request_id": request_id,
                        "op": operation,
                        "queue_ttl_ms": ttl_ms,
                        "args": dict(arguments),
                    },
                    allow_nan=False,
                    ensure_ascii=False,
                    separators=(",", ":"),
                    sort_keys=True,
                ).encode("utf-8")
                + b"\n"
            )
        except (TypeError, ValueError):
            raise PeripheralSSHConfigurationError(
                "Peripheral request is not strict JSON"
            ) from None
        if len(frame) > MAX_REQUEST_BYTES:
            raise PeripheralSSHConfigurationError(
                "Peripheral request is too large"
            )
        return frame

    def request(
        self,
        operation: str,
        arguments: Optional[Mapping[str, object]] = None,
        ttl_ms: int = 1000,
    ) -> Dict[str, object]:
        if operation not in ALLOWED_OPERATIONS:
            raise PeripheralSSHConfigurationError(
                "Peripheral operation is not allowed"
            )
        try:
            resolved_arguments = (
                {} if arguments is None else dict(arguments)
            )
        except (TypeError, ValueError):
            raise PeripheralSSHConfigurationError(
                "Peripheral arguments are invalid"
            ) from None
        expected_fields = (
            frozenset(("role",))
            if operation == "read_sensor"
            else frozenset()
        )
        if frozenset(resolved_arguments) != expected_fields:
            raise PeripheralSSHConfigurationError(
                "Peripheral arguments are invalid"
            )
        if operation == "read_sensor":
            _validate_identifier(
                "sensor role",
                resolved_arguments["role"],
                64,
            )
        if (
            isinstance(ttl_ms, bool)
            or not isinstance(ttl_ms, int)
            or not 1 <= ttl_ms <= MAX_TTL_MS
        ):
            raise PeripheralSSHConfigurationError(
                "Peripheral request TTL is invalid"
            )

        with self._request_lock:
            self._require_open()
            if (
                not self._described
                and operation not in ("describe", "shutdown")
            ):
                raise PeripheralSSHConfigurationError(
                    "Peripheral describe must be the first request"
                )
            if not self._responses.empty():
                raise self._poison(
                    "Unexpected peripheral response before request"
                )
            self._request_number += 1
            request_id = "{}-{}".format(
                self._request_prefix,
                self._request_number,
            )
            frame = self._encode_request(
                request_id,
                operation,
                resolved_arguments,
                ttl_ms,
            )
            try:
                written = self._process.stdin.write(frame)
                self._process.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                raise self._poison(
                    "Peripheral request write failed"
                ) from None
            if written != len(frame):
                raise self._poison(
                    "Peripheral request write was partial"
                )

            timeout = self._response_timeout_seconds
            if self._request_number == 1 and operation == "describe":
                timeout = self._startup_response_timeout_seconds
            try:
                raw = self._responses.get(timeout=timeout)
            except queue.Empty:
                raise self._poison(
                    "Peripheral response timed out",
                    timeout=True,
                ) from None
            if raw is _RECEIVE_FAILURE:
                with self._state_lock:
                    receive_failure = self._receive_failure
                raise self._poison(
                    receive_failure
                    or "Peripheral response stream closed"
                )
            try:
                result = decode_peripheral_response(
                    raw,
                    expected_request_id=request_id,
                    controller_id=self.controller_id,
                    operation=operation,
                    arguments=resolved_arguments,
                )
            except PeripheralRemoteError:
                raise
            except PeripheralSSHProtocolError as error:
                raise self._poison(str(error)) from None
            while True:
                try:
                    queued = self._responses.get_nowait()
                except queue.Empty:
                    break
                if queued is not _RECEIVE_FAILURE:
                    raise self._poison(
                        "Peripheral returned more than one response"
                    )
            if operation == "shutdown":
                with self._state_lock:
                    self._shutdown_confirmed = True
                    self._lifecycle_state = "CLOSING"
            elif operation == "describe":
                self._described = True
            # A receive failure sequenced after this correlated response
            # poisons the channel for future requests, but cannot make this
            # already confirmed result ambiguous.
            return result

    def describe(self) -> Dict[str, object]:
        return self.request("describe")

    def read_sensor(self, role: str) -> Dict[str, object]:
        return self.request("read_sensor", {"role": role})

    def wait_closed(self, timeout_seconds: float = 2.0) -> int:
        try:
            return int(self._process.wait(timeout=timeout_seconds))
        except subprocess.TimeoutExpired:
            raise PeripheralSSHTransportError(
                "Peripheral process did not close in time"
            ) from None

    def close(self) -> None:
        with self._request_lock:
            with self._state_lock:
                state = self._lifecycle_state
            if state == "OPEN":
                try:
                    # Inline the call because this lock deliberately excludes
                    # a concurrent sensor request during shutdown.
                    self._request_number += 1
                    request_id = "{}-{}".format(
                        self._request_prefix,
                        self._request_number,
                    )
                    frame = self._encode_request(
                        request_id,
                        "shutdown",
                        {},
                        1000,
                    )
                    if not self._responses.empty():
                        raise PeripheralSSHProtocolError(
                            "Unexpected response before shutdown"
                        )
                    written = self._process.stdin.write(frame)
                    self._process.stdin.flush()
                    if written != len(frame):
                        raise OSError("Partial shutdown write")
                    raw = self._responses.get(
                        timeout=self._response_timeout_seconds
                    )
                    decode_peripheral_response(
                        raw,
                        expected_request_id=request_id,
                        controller_id=self.controller_id,
                        operation="shutdown",
                        arguments={},
                    )
                    with self._state_lock:
                        self._shutdown_confirmed = True
                except BaseException:
                    pass
            with self._state_lock:
                if self._lifecycle_state != "CLOSED":
                    self._lifecycle_state = "CLOSING"
            self._close_input()

        try:
            self._process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            try:
                self._process.terminate()
                self._process.wait(timeout=1.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    self._process.kill()
                    self._process.wait(timeout=1.0)
                except (OSError, subprocess.TimeoutExpired):
                    pass
        for stream in (
            self._process.stdin,
            self._process.stdout,
            self._process.stderr,
        ):
            try:
                stream.close()
            except (OSError, ValueError):
                pass
        self._stdout_pump.join(timeout=0.2)
        self._stderr_pump.join(timeout=0.2)
        with self._state_lock:
            self._lifecycle_state = "CLOSED"

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        self.close()
