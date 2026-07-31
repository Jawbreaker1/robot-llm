"""One-shot Mac-side IR -> model shadow -> deterministic EV3 TTS cycle."""

import argparse
import json
import subprocess
import sys
from typing import Any, Callable, Dict, List, Mapping, Optional

from .lm_studio import DEFAULT_BASE_URL, DEFAULT_MODEL, NativeLMStudioClient
from .peripheral_transport import PeripheralSSHSession
from .shadow_commentary import ShadowSpeechError, run_shadow_comment
from .ssh_policy import motion_free_ssh_options


REMOTE_ROBOT_CLI = "/home/robot/robot-llm/ev3/robot_cli.py"
MAX_SSH_OUTPUT_BYTES = 64 * 1024
IR_SAMPLE_COUNT = 3
DEFAULT_CONTROLLER_ID = "ev3rstorm-01.ev3-main"
EV3_SPEECH_VOICES = ("sv", "en")

Runner = Callable[..., Any]


class EV3SSHError(RuntimeError):
    """Base class for bounded, non-interactive EV3 transport failures."""


class EV3SSHConfigurationError(EV3SSHError):
    pass


class EV3SSHTimeoutError(EV3SSHError):
    pass


class EV3SSHTransportError(EV3SSHError):
    pass


class EV3SSHProtocolError(EV3SSHError):
    pass


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
        raise EV3SSHConfigurationError("SSH target is invalid")
    return target


def _parse_json_object(raw: str, context: str) -> Dict[str, object]:
    if not isinstance(raw, str):
        raise EV3SSHProtocolError("{} response was not text".format(context))
    if len(raw.encode("utf-8")) > MAX_SSH_OUTPUT_BYTES:
        raise EV3SSHProtocolError("{} response was too large".format(context))

    def reject_constant(_: str) -> None:
        raise ValueError

    try:
        value = json.loads(raw, parse_constant=reject_constant)
    except (TypeError, ValueError):
        raise EV3SSHProtocolError(
            "{} returned invalid JSON".format(context)
        ) from None
    if not isinstance(value, dict):
        raise EV3SSHProtocolError(
            "{} response was not an object".format(context)
        )
    return value


class EV3SSHTransport:
    """Allow only fixed sensor-read and stdin-based speech commands."""

    def __init__(
        self,
        target: str,
        runner: Runner = subprocess.run,
        connect_timeout_seconds: int = 3,
        read_timeout_seconds: int = 20,
        speech_timeout_seconds: int = 25,
    ):
        self.target = _validate_target(target)
        for name, value in (
            ("connect timeout", connect_timeout_seconds),
            ("read timeout", read_timeout_seconds),
            ("speech timeout", speech_timeout_seconds),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or value <= 0
                or value > 120
            ):
                raise EV3SSHConfigurationError(
                    "{} is invalid".format(name)
                )
        if not callable(runner):
            raise EV3SSHConfigurationError("SSH runner is invalid")
        self._runner = runner
        self._connect_timeout_seconds = connect_timeout_seconds
        self._read_timeout_seconds = read_timeout_seconds
        self._speech_timeout_seconds = speech_timeout_seconds

    def _argv(self, remote_arguments: List[str]) -> List[str]:
        return (
            ["ssh", "-T"]
            + motion_free_ssh_options(self._connect_timeout_seconds)
            + [self.target]
            + list(remote_arguments)
        )

    def _execute(
        self,
        remote_arguments: List[str],
        timeout_seconds: int,
        stdin_text: Optional[str] = None,
    ) -> str:
        argv = self._argv(remote_arguments)
        try:
            completed = self._runner(
                argv,
                input=stdin_text,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_seconds,
                check=False,
            )
        except subprocess.TimeoutExpired:
            raise EV3SSHTimeoutError("EV3 SSH command timed out") from None
        except OSError:
            raise EV3SSHTransportError("EV3 SSH command failed") from None

        returncode = getattr(completed, "returncode", None)
        stdout = getattr(completed, "stdout", None)
        stderr = getattr(completed, "stderr", None)
        if isinstance(returncode, bool) or not isinstance(returncode, int):
            raise EV3SSHProtocolError("SSH runner returned an invalid status")
        if not isinstance(stdout, str) or not isinstance(stderr, str):
            raise EV3SSHProtocolError("SSH runner returned invalid output")
        if len(stdout.encode("utf-8")) > MAX_SSH_OUTPUT_BYTES:
            raise EV3SSHProtocolError("EV3 SSH response was too large")
        if returncode != 0:
            detail = " ".join(stderr.split())[:160]
            message = "EV3 SSH command failed with status {}".format(returncode)
            if detail:
                message += ": " + detail
            raise EV3SSHTransportError(message)
        return stdout

    def read_infrared(self) -> Mapping[str, object]:
        output = self._execute(
            [
                "python3",
                REMOTE_ROBOT_CLI,
                "read-sensor",
                "--role",
                "infrared",
            ],
            timeout_seconds=self._read_timeout_seconds,
        )
        reading = _parse_json_object(output, "EV3 sensor")

        expected_strings = {
            "role": "infrared",
            "driver": "lego-ev3-ir",
            "mode": "IR-PROX",
            "units": "pct",
        }
        for key, expected in expected_strings.items():
            if reading.get(key) != expected:
                raise EV3SSHProtocolError(
                    "EV3 sensor response had invalid {}".format(key)
                )

        value = reading.get("value0")
        observed_at_ms = reading.get("observed_monotonic_ms")
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 0 <= value <= 100
        ):
            raise EV3SSHProtocolError("EV3 sensor value was invalid")
        if (
            isinstance(observed_at_ms, bool)
            or not isinstance(observed_at_ms, int)
            or observed_at_ms < 0
        ):
            raise EV3SSHProtocolError("EV3 sensor timestamp was invalid")
        return reading

    def speak(
        self,
        text: str,
        voice: str = "sv",
    ) -> Mapping[str, object]:
        if not isinstance(text, str):
            raise EV3SSHConfigurationError("Speech text must be a string")
        if voice not in EV3_SPEECH_VOICES:
            raise EV3SSHConfigurationError("Speech voice is unsupported")
        output = self._execute(
            [
                "python3",
                REMOTE_ROBOT_CLI,
                "speak-stdin",
                "--voice",
                voice,
            ],
            timeout_seconds=self._speech_timeout_seconds,
            stdin_text=text + "\n",
        )
        result = _parse_json_object(output, "EV3 speech")
        if result.get("status") != "completed":
            raise EV3SSHProtocolError("EV3 speech did not complete")
        return result


class PersistentShadowTransport:
    """Warm sensor channel plus independently bounded one-shot speech."""

    def __init__(
        self,
        sensor_session: PeripheralSSHSession,
        speech_transport: EV3SSHTransport,
    ):
        self._sensor_session = sensor_session
        self._speech_transport = speech_transport

    def read_infrared(self) -> Mapping[str, object]:
        return self._sensor_session.read_sensor("infrared")

    def speak(self, text: str) -> Mapping[str, object]:
        return self._speech_transport.speak(text)


def run_shadow_cycle(transport: Any, model_client: Any):
    readings = [
        transport.read_infrared()
        for _ in range(IR_SAMPLE_COUNT)
    ]
    timestamps = [
        reading["observed_monotonic_ms"]
        for reading in readings
    ]
    if any(
        later < earlier
        for earlier, later in zip(timestamps, timestamps[1:])
    ):
        raise EV3SSHProtocolError("EV3 sensor timestamp moved backwards")

    samples = [reading["value0"] for reading in readings]
    return run_shadow_comment(
        samples=samples,
        observed_at_ms=timestamps[-1],
        model_client=model_client,
        speaker=transport.speak,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run one motion-free IR/model shadow cycle. "
            "The model candidate is logged but never spoken."
        )
    )
    parser.add_argument("--ssh-target", required=True)
    parser.add_argument(
        "--controller-id",
        default=DEFAULT_CONTROLLER_ID,
    )
    parser.add_argument("--lm-studio-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    sensor_session = None
    try:
        sensor_session = PeripheralSSHSession(
            args.ssh_target,
            args.controller_id,
        )
        sensor_session.describe()
        transport = PersistentShadowTransport(
            sensor_session,
            EV3SSHTransport(args.ssh_target),
        )
        model_client = NativeLMStudioClient(
            base_url=args.lm_studio_url,
            model=args.model,
        )
        result = run_shadow_cycle(transport, model_client)
    except ShadowSpeechError as error:
        report = dict(error.audit)
        report["status"] = "failed"
        report["stage"] = "tts"
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))
        return 3
    except Exception as error:
        report = {
            "status": "failed",
            "stage": "sensor_or_transport",
            "error": "{}: {}".format(
                type(error).__name__,
                str(error),
            )[:240],
        }
        print(
            json.dumps(report, ensure_ascii=False, sort_keys=True),
            file=sys.stderr,
        )
        return 2
    finally:
        if sensor_session is not None:
            sensor_session.close()

    report = result.to_dict()
    report["status"] = "completed"
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
