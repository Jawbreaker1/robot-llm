"""Sequential JSONL transport for the single-owner EV3 navigation worker."""

import queue
import subprocess
import threading
import time
from typing import Callable, Mapping, Optional

from .physical_navigation_contract import (
    EXPECTED_WORKER_SAFETY,
    MOTION_ACTIONS,
    REQUEST_SCHEMA,
    RESPONSE_SCHEMA,
    SCAN_TURN_ALLOWED_DELTAS_MDEG,
    SCAN_TURN_OPERATION,
    SCAN_SAMPLE_OPERATION,
    expected_scan_sample_profile,
    expected_scan_turn_profile,
    json_bytes,
    strict_json_loads,
    validate_observation,
)


MAX_REQUEST_BYTES = 4 * 1024
MAX_RESPONSE_BYTES = 64 * 1024
MAX_STDERR_BYTES = 8 * 1024
ACTIVE_MOTOR_STATES = frozenset(("running", "ramping"))
OPERATIONS = frozenset(
    (
        "describe",
        "observe",
        "pulse",
        SCAN_TURN_OPERATION,
        SCAN_SAMPLE_OPERATION,
        "stop",
        "shutdown",
    )
)


class EV3NavigationTransportError(RuntimeError):
    pass


class EV3NavigationRemoteError(EV3NavigationTransportError):
    def __init__(
        self,
        code: str,
        message: str,
        fatal: bool,
        observation=None,
        stop=None,
    ):
        self.code = code
        self.fatal = fatal
        self.observation = observation
        self.stop = stop
        super().__init__(message)


def _identifier(name: str, value: object, maximum: int) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(character in value for character in "\x00\r\n")
    ):
        raise EV3NavigationTransportError(
            "{} is invalid".format(name)
        )
    return value


def _target(value: str) -> str:
    checked = _identifier("SSH target", value, 255)
    if checked.startswith("-") or any(
        not (character.isalnum() or character in "._-@:%+")
        for character in checked
    ):
        raise EV3NavigationTransportError("SSH target is invalid")
    return checked


def _remote_path(value: str) -> str:
    checked = _identifier("remote worker path", value, 512)
    if not checked.startswith("/") or any(
        character not in (
            "abcdefghijklmnopqrstuvwxyz"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "0123456789"
            "/._-"
        )
        for character in checked
    ):
        raise EV3NavigationTransportError(
            "remote worker path is invalid"
        )
    return checked


class EV3NavigationSSHTransport:
    """One outstanding request at a time to one foreground worker process."""

    def __init__(
        self,
        *,
        target: str,
        controller_id: str,
        remote_worker_path: str,
        process_factory: Callable[..., object] = subprocess.Popen,
        ssh_binary: str = "ssh",
        connect_timeout_seconds: int = 5,
    ):
        self.target = _target(target)
        self.controller_id = _identifier(
            "controller_id",
            controller_id,
            128,
        )
        self.remote_worker_path = _remote_path(remote_worker_path)
        self.ssh_binary = _identifier("ssh binary", ssh_binary, 128)
        if not callable(process_factory):
            raise EV3NavigationTransportError(
                "process factory is invalid"
            )
        if (
            isinstance(connect_timeout_seconds, bool)
            or not isinstance(connect_timeout_seconds, int)
            or not 1 <= connect_timeout_seconds <= 30
        ):
            raise EV3NavigationTransportError(
                "connect timeout is invalid"
            )
        self._process_factory = process_factory
        self._connect_timeout_seconds = connect_timeout_seconds
        self._process = None
        self._responses = queue.Queue(maxsize=1)
        self._stderr = bytearray()
        self._stderr_lock = threading.Lock()
        self._sequence = 0
        self._last_state_version = 0
        self.shutdown_complete = False
        self._abort_lock = threading.Lock()
        self._aborted = False
        self._worker_description = None

    @property
    def last_state_version(self) -> int:
        return self._last_state_version

    @property
    def worker_description(self):
        if self._worker_description is None:
            return None
        return dict(self._worker_description)

    @property
    def aborted(self) -> bool:
        with self._abort_lock:
            return self._aborted

    def start(self) -> None:
        if self._process is not None:
            raise EV3NavigationTransportError(
                "navigation worker is already started"
            )
        argv = [
            self.ssh_binary,
            "-T",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout={}".format(self._connect_timeout_seconds),
            "-o",
            "StrictHostKeyChecking=yes",
            self.target,
            "python3",
            self.remote_worker_path,
        ]
        try:
            self._process = self._process_factory(
                argv,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except OSError as error:
            raise EV3NavigationTransportError(
                "navigation worker could not start: {}".format(error)
            ) from error
        threading.Thread(
            target=self._read_stdout,
            name="ev3-navigation-worker-stdout",
            daemon=True,
        ).start()
        threading.Thread(
            target=self._read_stderr,
            name="ev3-navigation-worker-stderr",
            daemon=True,
        ).start()

    def _read_stdout(self) -> None:
        try:
            while True:
                raw = self._process.stdout.readline(MAX_RESPONSE_BYTES + 1)
                if raw == b"":
                    self._responses.put(("eof", None))
                    return
                if len(raw) > MAX_RESPONSE_BYTES or not raw.endswith(b"\n"):
                    self._responses.put(("invalid", None))
                    return
                self._responses.put(("line", raw))
        except BaseException as error:
            self._responses.put(("error", str(error)))

    def _read_stderr(self) -> None:
        try:
            while True:
                chunk = self._process.stderr.read(512)
                if not chunk:
                    return
                with self._stderr_lock:
                    remaining = MAX_STDERR_BYTES - len(self._stderr)
                    if remaining > 0:
                        self._stderr.extend(chunk[:remaining])
        except BaseException:
            return

    def stderr_summary(self) -> str:
        with self._stderr_lock:
            value = bytes(self._stderr)
        return " ".join(
            value.decode("utf-8", errors="replace").split()
        )[:500]

    @staticmethod
    def _validate_stop_proof(value: object) -> Mapping[str, object]:
        required = {
            "stop_attempts",
            "stop_confirmed",
            "states",
            "positions",
            "fault_tokens",
            "errors",
        }
        allowed = required | {"cleanup_errors"}
        if (
            not isinstance(value, dict)
            or not required <= set(value)
            or not set(value) <= allowed
            or value["stop_confirmed"] is not True
            or not isinstance(value["stop_attempts"], list)
            or not isinstance(value["states"], dict)
            or not isinstance(value["positions"], dict)
            or not isinstance(value["fault_tokens"], dict)
            or not isinstance(value["errors"], list)
            or value["fault_tokens"]
            or value["errors"]
            or not isinstance(value.get("cleanup_errors", []), list)
            or value.get("cleanup_errors", [])
        ):
            raise EV3NavigationTransportError(
                "worker stop proof is invalid"
            )
        return value

    @staticmethod
    def _validate_scan_slice(
        value: object,
        *,
        ordinal: int,
        expected_count: int,
    ) -> None:
        fields = {
            "slice_index",
            "slice_count",
            "duration_ms",
            "status",
            "reason",
            "started_monotonic_ms",
            "completed_monotonic_ms",
            "motors",
            "encoder_verification",
            "stop",
        }
        if (
            not isinstance(value, dict)
            or set(value) != fields
            or value["slice_index"] != ordinal
            or value["slice_count"] != expected_count
            or isinstance(value["duration_ms"], bool)
            or not isinstance(value["duration_ms"], int)
            or not 1 <= value["duration_ms"] <= 800
            or value["status"] != "completed"
            or value["reason"] != "duration_elapsed"
            or isinstance(value["started_monotonic_ms"], bool)
            or not isinstance(value["started_monotonic_ms"], int)
            or isinstance(value["completed_monotonic_ms"], bool)
            or not isinstance(value["completed_monotonic_ms"], int)
            or value["completed_monotonic_ms"]
            < value["started_monotonic_ms"]
        ):
            raise EV3NavigationTransportError(
                "scan-turn slice receipt is invalid"
            )
        verification = value["encoder_verification"]
        if (
            not isinstance(verification, dict)
            or set(verification) != {"passed", "error", "checks"}
            or verification["passed"] is not True
            or verification["error"] is not None
            or not isinstance(verification["checks"], list)
            or not verification["checks"]
            or any(
                not isinstance(check, dict)
                or check.get("passed") is not True
                for check in verification["checks"]
            )
        ):
            raise EV3NavigationTransportError(
                "scan-turn slice encoder proof is invalid"
            )
        motors = value["motors"]
        if not isinstance(motors, list) or len(motors) != 2:
            raise EV3NavigationTransportError(
                "scan-turn slice motors are invalid"
            )
        seen_sides = set()
        for motor in motors:
            if (
                not isinstance(motor, dict)
                or set(motor)
                != {
                    "side",
                    "role",
                    "position_before",
                    "position_after",
                    "position_delta",
                    "state",
                }
                or motor["side"] not in ("left", "right")
                or motor["side"] in seen_sides
                or not isinstance(motor["role"], str)
                or not motor["role"]
                or any(
                    isinstance(motor[field], bool)
                    or not isinstance(motor[field], int)
                    for field in (
                        "position_before",
                        "position_after",
                        "position_delta",
                    )
                )
                or motor["position_delta"]
                != motor["position_after"] - motor["position_before"]
                or not isinstance(motor["state"], str)
            ):
                raise EV3NavigationTransportError(
                    "scan-turn motor receipt is invalid"
                )
            seen_sides.add(motor["side"])
        if seen_sides != {"left", "right"}:
            raise EV3NavigationTransportError(
                "scan-turn motor sides are invalid"
            )
        EV3NavigationSSHTransport._validate_stop_proof(value["stop"])

    def _validate_success_result(
        self,
        operation: str,
        arguments: Mapping[str, object],
        response: Mapping[str, object],
    ) -> None:
        result = response["result"]
        if operation == "describe":
            if (
                not isinstance(result, dict)
                or result.get("scan_turn")
                != expected_scan_turn_profile()
                or result.get("scan_sample")
                != expected_scan_sample_profile()
                or not isinstance(result.get("operations"), list)
                or set(result["operations"]) != OPERATIONS
                or result.get("safety") != EXPECTED_WORKER_SAFETY
            ):
                raise EV3NavigationTransportError(
                    "worker scan-turn capability is invalid"
                )
            self._worker_description = dict(result)
            return
        if operation == SCAN_SAMPLE_OPERATION:
            profile = expected_scan_sample_profile()
            fields = {
                "sample_count",
                "raw_samples",
                "started_monotonic_ms",
                "completed_monotonic_ms",
                "observation",
                "stop",
            }
            if (
                not isinstance(result, dict)
                or set(result) != fields
                or result["sample_count"] != profile["sample_count"]
                or not isinstance(result["raw_samples"], list)
                or len(result["raw_samples"]) != profile["sample_count"]
                or any(
                    isinstance(sample, bool)
                    or not isinstance(sample, int)
                    or not 0 <= sample <= 100
                    for sample in result["raw_samples"]
                )
                or isinstance(result["started_monotonic_ms"], bool)
                or not isinstance(result["started_monotonic_ms"], int)
                or isinstance(result["completed_monotonic_ms"], bool)
                or not isinstance(result["completed_monotonic_ms"], int)
                or result["completed_monotonic_ms"]
                < result["started_monotonic_ms"]
            ):
                raise EV3NavigationTransportError(
                    "worker scan sample result is invalid"
                )
            observation = validate_observation(result["observation"])
            infrared = observation["infrared"]
            ordered_samples = sorted(result["raw_samples"])
            expected_median = ordered_samples[len(ordered_samples) // 2]
            if (
                observation["state_version"] != response["state_version"]
                or result["completed_monotonic_ms"]
                - result["started_monotonic_ms"]
                < profile["settled_duration_ms"]
                or infrared["sample_count"] != profile["sample_count"]
                or infrared["raw"] != result["raw_samples"][-1]
                or infrared["filtered"] != expected_median
                or observation["touch"]["pressed"] is not False
                or observation["budgets"]["motion_fault_latched"] is not False
                or any(
                    frozenset(motor["state"].split())
                    & ACTIVE_MOTOR_STATES
                    for motor in observation["motors"]
                )
            ):
                raise EV3NavigationTransportError(
                    "worker scan sample evidence is invalid"
                )
            self._validate_stop_proof(result["stop"])
            return
        if operation == "observe":
            if not isinstance(result, dict) or set(result) != {
                "observation"
            }:
                raise EV3NavigationTransportError(
                    "worker observe result is invalid"
                )
            observation = validate_observation(result["observation"])
            if observation["state_version"] != response["state_version"]:
                raise EV3NavigationTransportError(
                    "worker observe version is invalid"
                )
            return
        if operation == "stop":
            if (
                not isinstance(result, dict)
                or set(result) != {"outcome", "observation", "stop"}
                or not isinstance(result["outcome"], dict)
                or set(result["outcome"])
                != {
                    "kind",
                    "status",
                    "completed_monotonic_ms",
                    "stop_confirmed",
                }
                or result["outcome"]["kind"] != "stop"
                or result["outcome"]["status"] != "completed"
                or result["outcome"]["stop_confirmed"] is not True
            ):
                raise EV3NavigationTransportError(
                    "worker stop result is invalid"
                )
            observation = validate_observation(result["observation"])
            if (
                observation["state_version"] != response["state_version"]
                or observation["last_outcome"] != result["outcome"]
            ):
                raise EV3NavigationTransportError(
                    "worker stop result is uncorrelated"
                )
            self._validate_stop_proof(result["stop"])
            return
        if operation != SCAN_TURN_OPERATION:
            return

        relative_delta_mdeg = arguments["relative_delta_mdeg"]
        fields = {
            "relative_delta_mdeg",
            "outcome",
            "observation",
            "stop",
        }
        if (
            not isinstance(result, dict)
            or set(result) != fields
            or result["relative_delta_mdeg"] != relative_delta_mdeg
        ):
            raise EV3NavigationTransportError(
                "worker scan-turn result is uncorrelated"
            )
        outcome = result["outcome"]
        outcome_fields = {
            "kind",
            "requested_relative_delta_mdeg",
            "status",
            "reason",
            "profile_id",
            "calibration",
            "started_monotonic_ms",
            "completed_monotonic_ms",
            "stop_confirmed",
            "requested_slice_count",
            "completed_slice_count",
            "slices",
            "encoder_verification",
        }
        expected_profile = expected_scan_turn_profile()
        expected_spec = next(
            item
            for item in expected_profile["turns"]
            if item["relative_delta_mdeg"] == relative_delta_mdeg
        )
        if (
            not isinstance(outcome, dict)
            or set(outcome) != outcome_fields
            or outcome["kind"] != "scan_turn"
            or outcome["requested_relative_delta_mdeg"]
            != relative_delta_mdeg
            or outcome["status"] != "completed"
            or outcome["reason"] != "scan_turn_completed"
            or outcome["profile_id"] != expected_profile["profile_id"]
            or outcome["calibration"] != expected_profile["calibration"]
            or outcome["stop_confirmed"] is not True
            or outcome["requested_slice_count"]
            != expected_spec["slice_count"]
            or outcome["completed_slice_count"]
            != expected_spec["slice_count"]
            or not isinstance(outcome["slices"], list)
            or len(outcome["slices"]) != expected_spec["slice_count"]
        ):
            raise EV3NavigationTransportError(
                "worker scan-turn outcome is invalid"
            )
        for ordinal, value in enumerate(outcome["slices"], 1):
            self._validate_scan_slice(
                value,
                ordinal=ordinal,
                expected_count=expected_spec["slice_count"],
            )
            if (
                value["duration_ms"]
                != expected_spec["slice_durations_ms"][ordinal - 1]
            ):
                raise EV3NavigationTransportError(
                    "scan-turn slice profile changed"
                )
        encoder = outcome["encoder_verification"]
        encoder_fields = {
            "passed",
            "verified_slice_count",
            "requested_slice_count",
            "left_delta_degrees",
            "right_delta_degrees",
            "mean_abs_encoder_degrees",
            "target_mean_abs_encoder_degrees",
            "max_side_divergence_degrees",
        }
        if (
            not isinstance(encoder, dict)
            or set(encoder) != encoder_fields
            or encoder["passed"] is not True
            or encoder["verified_slice_count"]
            != expected_spec["slice_count"]
            or encoder["requested_slice_count"]
            != expected_spec["slice_count"]
            or any(
                isinstance(encoder[field], bool)
                or not isinstance(encoder[field], int)
                for field in (
                    "left_delta_degrees",
                    "right_delta_degrees",
                    "mean_abs_encoder_degrees",
                    "target_mean_abs_encoder_degrees",
                    "max_side_divergence_degrees",
                )
            )
            or encoder["target_mean_abs_encoder_degrees"]
            != expected_spec["target_mean_abs_encoder_degrees"]
            or encoder["max_side_divergence_degrees"]
            != expected_profile["max_side_divergence_degrees"]
        ):
            raise EV3NavigationTransportError(
                "worker scan-turn aggregate encoder proof is invalid"
            )
        observation = validate_observation(result["observation"])
        if (
            observation["state_version"] != response["state_version"]
            or observation["last_outcome"] != outcome
        ):
            raise EV3NavigationTransportError(
                "worker scan-turn observation is uncorrelated"
            )
        self._validate_stop_proof(result["stop"])

    def _validate_response(
        self,
        value: object,
        request_id: str,
    ) -> Mapping[str, object]:
        if not isinstance(value, dict) or type(value.get("ok")) is not bool:
            raise EV3NavigationTransportError(
                "worker response is not a valid object"
            )
        expected = {
            "schema",
            "controller_id",
            "request_id",
            "ok",
            "state_version",
            "result" if value["ok"] else "error",
        }
        if set(value) != expected:
            raise EV3NavigationTransportError(
                "worker response fields are invalid"
            )
        if (
            value["schema"] != RESPONSE_SCHEMA
            or value["controller_id"] != self.controller_id
            or value["request_id"] != request_id
            or isinstance(value["state_version"], bool)
            or not isinstance(value["state_version"], int)
            or value["state_version"] < self._last_state_version
        ):
            raise EV3NavigationTransportError(
                "worker response correlation/version is invalid"
            )
        payload = value["result"] if value["ok"] else value["error"]
        if not isinstance(payload, dict):
            raise EV3NavigationTransportError(
                "worker response payload is invalid"
            )
        self._last_state_version = value["state_version"]
        if not value["ok"]:
            required_error_fields = {"code", "message", "fatal"}
            allowed_error_fields = required_error_fields | {
                "observation",
                "stop",
            }
            if (
                not required_error_fields <= set(payload)
                or not set(payload) <= allowed_error_fields
                or not isinstance(payload["code"], str)
                or not payload["code"]
                or not isinstance(payload["message"], str)
                or not payload["message"]
                or type(payload["fatal"]) is not bool
            ):
                raise EV3NavigationTransportError(
                    "worker error payload is invalid"
                )
            observation = None
            if "observation" in payload:
                try:
                    observation = validate_observation(
                        payload["observation"]
                    )
                except Exception as error:
                    raise EV3NavigationTransportError(
                        "worker error observation is invalid"
                    ) from error
                if observation["state_version"] != value["state_version"]:
                    raise EV3NavigationTransportError(
                        "worker error observation version is invalid"
                    )
            stop = payload.get("stop")
            if stop is not None:
                self._validate_stop_proof(stop)
            raise EV3NavigationRemoteError(
                payload["code"],
                payload["message"],
                payload["fatal"],
                observation=observation,
                stop=stop,
            )
        return value

    def request(
        self,
        operation: str,
        arguments: Optional[Mapping[str, object]] = None,
        timeout_seconds: float = 8.0,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> Mapping[str, object]:
        if self.aborted:
            raise EV3NavigationTransportError(
                "aborted navigation transport cannot be reused"
            )
        if self._process is None or self._process.stdin is None:
            raise EV3NavigationTransportError(
                "navigation worker is not open"
            )
        args = {} if arguments is None else dict(arguments)
        if operation not in OPERATIONS:
            raise EV3NavigationTransportError(
                "unknown worker operation"
            )
        if operation == "pulse":
            if set(args) != {"action"} or args["action"] not in MOTION_ACTIONS:
                raise EV3NavigationTransportError(
                    "pulse arguments are invalid"
                )
        elif operation == SCAN_TURN_OPERATION:
            if (
                set(args) != {"relative_delta_mdeg"}
                or isinstance(args["relative_delta_mdeg"], bool)
                or not isinstance(args["relative_delta_mdeg"], int)
                or args["relative_delta_mdeg"]
                not in SCAN_TURN_ALLOWED_DELTAS_MDEG
            ):
                raise EV3NavigationTransportError(
                    "scan-turn arguments are invalid"
                )
        elif args:
            raise EV3NavigationTransportError(
                "worker operation has unexpected arguments"
            )
        if cancel_requested is not None and not callable(cancel_requested):
            raise EV3NavigationTransportError(
                "request cancellation probe is invalid"
            )
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not 0.1 <= timeout_seconds <= 60
        ):
            raise EV3NavigationTransportError(
                "request timeout is invalid"
            )

        self._sequence += 1
        request_id = "host-{:04d}".format(self._sequence)
        frame = json_bytes(
            {
                "schema": REQUEST_SCHEMA,
                "controller_id": self.controller_id,
                "request_id": request_id,
                "op": operation,
                "args": args,
            }
        ) + b"\n"
        if len(frame) > MAX_REQUEST_BYTES:
            raise EV3NavigationTransportError(
                "worker request exceeded its byte limit"
            )
        request_written = False
        try:
            try:
                self._process.stdin.write(frame)
                request_written = True
                self._process.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                self.abort()
                raise EV3NavigationTransportError(
                    "worker input closed: {}".format(
                        self.stderr_summary()
                    )
                ) from None
            response_deadline = time.monotonic() + float(timeout_seconds)
            while True:
                if cancel_requested is not None:
                    try:
                        cancelled = cancel_requested() is True
                    except BaseException:
                        raise EV3NavigationTransportError(
                            "request cancellation probe failed"
                        ) from None
                    if cancelled:
                        raise EV3NavigationTransportError(
                            "worker request cancelled; SSH channel closed"
                        )
                remaining = response_deadline - time.monotonic()
                if remaining <= 0:
                    raise EV3NavigationTransportError(
                        "worker response timed out: {}".format(
                            self.stderr_summary()
                        )
                    )
                try:
                    kind, raw = self._responses.get(
                        timeout=min(0.02, remaining)
                    )
                    break
                except queue.Empty:
                    continue
            if kind != "line":
                raise EV3NavigationTransportError(
                    "worker output ended ({}) {}".format(
                        kind,
                        self.stderr_summary(),
                    )
                )
            value = strict_json_loads(raw, MAX_RESPONSE_BYTES)
            response = self._validate_response(value, request_id)
            self._validate_success_result(operation, args, response)
            if operation == "shutdown":
                self.shutdown_complete = True
            return response
        except EV3NavigationRemoteError as error:
            if error.fatal:
                self.abort()
            raise
        except BaseException:
            if request_written:
                self.abort()
            raise

    def close(self) -> None:
        if self._process is None:
            return
        if self._process.stdin is not None:
            try:
                self._process.stdin.close()
            except OSError:
                pass
        try:
            self._process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._process.terminate()
            try:
                self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=1.0)
        if self.shutdown_complete and self._process.returncode != 0:
            raise EV3NavigationTransportError(
                "worker exited {} after shutdown: {}".format(
                    self._process.returncode,
                    self.stderr_summary(),
                )
            )

    def abort(self) -> None:
        """Break the SSH channel so the worker's EOF cleanup stops motors."""

        with self._abort_lock:
            if self._aborted:
                return
            self._aborted = True
            process = self._process
            if process is None:
                return
            if process.stdin is not None:
                try:
                    process.stdin.close()
                except OSError:
                    pass
            try:
                process.terminate()
            except OSError:
                pass
